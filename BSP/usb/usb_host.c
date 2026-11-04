#include "usb_host.h"

#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "usbh_core.h"

/* ==========================================================================
 * USB Host 适配层 + Xbox 360 接收器 class
 *
 * 硬件：USB_OTG_HS 内嵌 FS PHY，PA11/PA12。
 * 软件：ST USB Host Library（Middlewares/ST/STM32_USB_Host_Library）
 *       要求实现的 USBH_LL_* 适配函数在这里，内部全是 HAL_HCD 调用。
 *
 * 识别方式不是 VID/PID，而是接口类码 FF/5D/01 —— 这是 Xbox 360
 * 无线接收器的特征，飞智冰原狼等第三方接收器也走这套。
 * ========================================================================== */

HCD_HandleTypeDef hhcd_USB_OTG_HS;

static USBH_HandleTypeDef s_host;
static volatile uint8_t s_ll_ready;

volatile Usb_Host_Diag g_usb_diag;

#define USB_HOST_PIPE_NONE 0xFF
#define USB_HOST_PIPE_COUNT 8
#define USB_HOST_PAYLOAD 64
#define USB_HOST_URB_TIMEOUT_MS 100

/* 一次只挂一个 IN URB。无线接收器空闲时一直 NAK，靠 interval 重新提交，
 * 不能把 NAK 当错误重枚举，否则手柄会一直掉线重连。 */
typedef struct {
  uint8_t in_pipe;
  uint8_t out_pipe;
  uint8_t in_ep;
  uint8_t out_ep;
  uint8_t packet_size;
  uint8_t interval_ms;
  uint8_t pending;
  uint8_t clearing;
  uint32_t submitted;
  uint8_t buf[USB_HOST_PAYLOAD];
} Xbox_Pipe_s;

static Xbox_Pipe_s s_xbox;
static volatile Xbox_State s_state;
static volatile uint8_t s_ready;

/* ==================== HCD 初始化 ==================== */

static void Usb_Host_Gpio_Init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &gpio);
}

/* HAL_HCD_Init 内部会调这里。和原来 PCD 的 MSP 内容基本一样，
 * 多了一步 PA11/PA12 的 AF 配置（原来那份是空的，所以 CDC 从来没通）。 */
void HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd) {
  if (hhcd->Instance != USB_OTG_HS) {
    return;
  }

  RCC_PeriphCLKInitTypeDef clk = {0};

  clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK) {
    Error_Handler();
  }

  HAL_PWREx_EnableUSBVoltageDetector();
  __HAL_RCC_USB_OTG_HS_CLK_ENABLE();

  Usb_Host_Gpio_Init();

  HAL_NVIC_SetPriority(OTG_HS_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(OTG_HS_IRQn);
}

static void Usb_Host_Hcd_Init(void) {
  hhcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hhcd_USB_OTG_HS.Init.Host_channels = 12; /* OTG_HS 有 12 个 host channel */
  hhcd_USB_OTG_HS.Init.speed = HCD_SPEED_FULL;
  hhcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.phy_itface = USB_OTG_EMBEDDED_PHY;
  hhcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  /* 板子没有 VBUS 检测引脚。Host 靠设备插入时 DP 被上拉来感知，
   * 不需要 VBUS 检测；开了反而会因为检测不到而拒绝启动。 */
  hhcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hhcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hhcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;

  if (HAL_HCD_Init(&hhcd_USB_OTG_HS) != HAL_OK) {
    Error_Handler();
  }
}

/* ==================== HCD 回调 ==================== */

void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd) {
  if (s_ll_ready) {
    USBH_LL_SetTimer(&s_host, HAL_HCD_GetCurrentFrame(hhcd));
  }
}

void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd) {
  (void)hhcd;
  if (s_ll_ready) {
    g_usb_diag.connects++;
    (void)USBH_LL_Connect(&s_host);
  }
}

void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd) {
  (void)hhcd;
  if (s_ll_ready) {
    g_usb_diag.disconnects++;
    /* 先清本地状态，避免下次连接还挂着上一次的 URB */
    s_xbox.pending = 0;
    s_xbox.clearing = 0;
    s_ready = 0;
    memset((void *)&s_state, 0, sizeof(s_state));
    (void)USBH_LL_Disconnect(&s_host);
  }
}

void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd) {
  (void)hhcd;
  if (s_ll_ready) {
    USBH_LL_PortEnabled(&s_host);
  }
}

void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd) {
  (void)hhcd;
  if (s_ll_ready) {
    USBH_LL_PortDisabled(&s_host);
  }
}

/* ==================== USBH_LL_* 适配 ==================== */

static uint8_t Usb_Host_Valid(const USBH_HandleTypeDef *phost) {
  return (uint8_t)(s_ll_ready && phost != NULL && phost == &s_host);
}

USBH_StatusTypeDef USBH_LL_Init(USBH_HandleTypeDef *phost) {
  s_ll_ready = 0;

  if (phost == NULL || phost != &s_host) {
    return USBH_FAIL;
  }

  phost->pData = &hhcd_USB_OTG_HS;
  hhcd_USB_OTG_HS.pData = phost;

  USBH_LL_SetTimer(phost, HAL_HCD_GetCurrentFrame(&hhcd_USB_OTG_HS));
  s_ll_ready = 1;

  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_DeInit(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }
  return (HAL_HCD_Stop(&hhcd_USB_OTG_HS) == HAL_OK) ? USBH_OK : USBH_FAIL;
}

USBH_StatusTypeDef USBH_LL_Start(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }
  /* 硬件已经由 HAL_HCD_Init 配好，这里只使能端口 */
  return (HAL_HCD_Start(&hhcd_USB_OTG_HS) == HAL_OK) ? USBH_OK : USBH_FAIL;
}

USBH_StatusTypeDef USBH_LL_Stop(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }
  return (HAL_HCD_Stop(&hhcd_USB_OTG_HS) == HAL_OK) ? USBH_OK : USBH_FAIL;
}

USBH_SpeedTypeDef USBH_LL_GetSpeed(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_SPEED_FULL;
  }
  return (HAL_HCD_GetCurrentSpeed(&hhcd_USB_OTG_HS) == 0) ? USBH_SPEED_HIGH
                                                          : USBH_SPEED_FULL;
}

USBH_StatusTypeDef USBH_LL_ResetPort(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }
  return (HAL_HCD_ResetPort(&hhcd_USB_OTG_HS) == HAL_OK) ? USBH_OK : USBH_FAIL;
}

uint32_t USBH_LL_GetLastXferSize(USBH_HandleTypeDef *phost, uint8_t pipe) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return 0;
  }
  /* 返回实际完成长度，不能把配置的最大包长当报告长度 */
  return HAL_HCD_HC_GetXferCount(&hhcd_USB_OTG_HS, pipe);
}

USBH_StatusTypeDef USBH_LL_DriverVBUS(USBH_HandleTypeDef *phost, uint8_t state) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }
  /* 板子没有 VBUS 开关引脚，5V 必须由硬件常供 */
  (void)state;
  return USBH_OK;
}

USBH_StatusTypeDef USBH_LL_OpenPipe(USBH_HandleTypeDef *phost, uint8_t pipe,
                                    uint8_t epnum, uint8_t dev_address,
                                    uint8_t speed, uint8_t ep_type,
                                    uint16_t mps) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return USBH_FAIL;
  }
  return (HAL_HCD_HC_Init(&hhcd_USB_OTG_HS, pipe, epnum, dev_address, speed,
                          ep_type, mps) == HAL_OK)
             ? USBH_OK
             : USBH_FAIL;
}

USBH_StatusTypeDef USBH_LL_ActivatePipe(USBH_HandleTypeDef *phost,
                                        uint8_t pipe) {
  (void)pipe;
  return Usb_Host_Valid(phost) ? USBH_OK : USBH_FAIL;
}

USBH_StatusTypeDef USBH_LL_ClosePipe(USBH_HandleTypeDef *phost, uint8_t pipe) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return USBH_FAIL;
  }
  return (HAL_HCD_HC_Halt(&hhcd_USB_OTG_HS, pipe) == HAL_OK) ? USBH_OK
                                                            : USBH_FAIL;
}

USBH_StatusTypeDef USBH_LL_SubmitURB(USBH_HandleTypeDef *phost, uint8_t pipe,
                                     uint8_t direction, uint8_t ep_type,
                                     uint8_t token, uint8_t *pbuff,
                                     uint16_t length, uint8_t do_ping) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return USBH_FAIL;
  }
  if (pbuff == NULL && length != 0) {
    return USBH_FAIL;
  }
  return (HAL_HCD_HC_SubmitRequest(&hhcd_USB_OTG_HS, pipe, direction, ep_type,
                                   token, pbuff, length, do_ping) == HAL_OK)
             ? USBH_OK
             : USBH_FAIL;
}

USBH_URBStateTypeDef USBH_LL_GetURBState(USBH_HandleTypeDef *phost,
                                         uint8_t pipe) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return USBH_URB_ERROR;
  }
  return (USBH_URBStateTypeDef)HAL_HCD_HC_GetURBState(&hhcd_USB_OTG_HS, pipe);
}

USBH_StatusTypeDef USBH_LL_SetToggle(USBH_HandleTypeDef *phost, uint8_t pipe,
                                     uint8_t toggle) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return USBH_FAIL;
  }
  if (hhcd_USB_OTG_HS.hc[pipe].ep_is_in != 0) {
    hhcd_USB_OTG_HS.hc[pipe].toggle_in = toggle;
  } else {
    hhcd_USB_OTG_HS.hc[pipe].toggle_out = toggle;
  }
  return USBH_OK;
}

uint8_t USBH_LL_GetToggle(USBH_HandleTypeDef *phost, uint8_t pipe) {
  if (!Usb_Host_Valid(phost) || pipe >= USB_HOST_PIPE_COUNT) {
    return 0;
  }
  return (hhcd_USB_OTG_HS.hc[pipe].ep_is_in != 0)
             ? hhcd_USB_OTG_HS.hc[pipe].toggle_in
             : hhcd_USB_OTG_HS.hc[pipe].toggle_out;
}

/* 枚举时会调，调度器跑起来后必须让出 CPU，否则低优先级任务全饿死 */
void USBH_Delay(uint32_t delay_ms) {
  if (delay_ms == 0) {
    return;
  }
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  } else {
    HAL_Delay(delay_ms);
  }
}

/* ==================== Xbox class ==================== */

/* 接口必须是 FF/5D/01、双中断端点。
 * ST 的 USBH_FindInterface 用 0xFF 当通配符，不逐槽验证会匹配错接口。 */
static uint8_t Xbox_Interface_Valid(const USBH_InterfaceDescTypeDef *itf) {
  if (itf == NULL || itf->bLength != USB_INTERFACE_DESC_SIZE) {
    return 0;
  }
  if (itf->bInterfaceClass != 0xFF || itf->bInterfaceSubClass != 0x5D ||
      itf->bInterfaceProtocol != 0x01 || itf->bAlternateSetting != 0 ||
      itf->bNumEndpoints != 2) {
    return 0;
  }

  uint8_t directions = 0;

  for (uint8_t i = 0; i < itf->bNumEndpoints; i++) {
    const USBH_EpDescTypeDef *ep = &itf->Ep_Desc[i];
    const uint8_t dir = ((ep->bEndpointAddress & 0x80) != 0) ? 1 : 2;

    if ((directions & dir) != 0) {
      return 0; /* 同方向两个端点，描述符不可信 */
    }
    if ((ep->bmAttributes & 3) != USBH_EP_INTERRUPT) {
      return 0;
    }
    if ((ep->bEndpointAddress & 0x0F) == 0) {
      return 0; /* 端点号 0 是控制端点 */
    }
    if ((ep->bEndpointAddress & 0x70) != 0) {
      return 0;
    }
    if (ep->wMaxPacketSize == 0 || ep->wMaxPacketSize > USB_HOST_PAYLOAD) {
      return 0;
    }
    if (ep->bInterval == 0) {
      return 0;
    }
    if (dir == 1 && ep->wMaxPacketSize < XBOX_REPORT_SIZE) {
      return 0;
    }

    directions |= dir;
  }

  return (directions == 3) ? 1 : 0;
}

static USBH_StatusTypeDef Xbox_Class_DeInit(USBH_HandleTypeDef *phost);

static USBH_StatusTypeDef Xbox_Class_Init(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost)) {
    return USBH_FAIL;
  }

  memset(&s_xbox, 0, sizeof(s_xbox));
  s_xbox.in_pipe = USB_HOST_PIPE_NONE;
  s_xbox.out_pipe = USB_HOST_PIPE_NONE;
  s_ready = 0;

  if (phost->device.speed != USBH_SPEED_FULL) {
    return USBH_FAIL; /* 接收器都是 FS 设备 */
  }

  uint8_t itf_index = USB_HOST_PIPE_NONE;
  USBH_InterfaceDescTypeDef *itf = NULL;

  for (uint8_t i = 0; i < USBH_MAX_NUM_INTERFACES; i++) {
    if (Xbox_Interface_Valid(&phost->device.CfgDesc.Itf_Desc[i])) {
      itf_index = i;
      itf = &phost->device.CfgDesc.Itf_Desc[i];
      break;
    }
  }

  if (itf == NULL || USBH_SelectInterface(phost, itf_index) != USBH_OK) {
    return USBH_FAIL;
  }

  g_usb_diag.vid = phost->device.DevDesc.idVendor;
  g_usb_diag.pid = phost->device.DevDesc.idProduct;

  for (uint8_t i = 0; i < itf->bNumEndpoints; i++) {
    USBH_EpDescTypeDef *ep = &itf->Ep_Desc[i];
    const uint8_t pipe = USBH_AllocPipe(phost, ep->bEndpointAddress);

    if (pipe >= USB_HOST_PIPE_COUNT) {
      (void)Xbox_Class_DeInit(phost);
      return USBH_FAIL;
    }
    if (USBH_LL_OpenPipe(phost, pipe, ep->bEndpointAddress,
                         phost->device.address, phost->device.speed,
                         USBH_EP_INTERRUPT, ep->wMaxPacketSize) != USBH_OK) {
      (void)USBH_FreePipe(phost, pipe);
      (void)Xbox_Class_DeInit(phost);
      return USBH_FAIL;
    }

    (void)USBH_LL_SetToggle(phost, pipe, 0);

    if ((ep->bEndpointAddress & 0x80) != 0) {
      s_xbox.in_pipe = pipe;
      s_xbox.in_ep = ep->bEndpointAddress;
      s_xbox.packet_size = (uint8_t)ep->wMaxPacketSize;
      s_xbox.interval_ms = ep->bInterval;
      g_usb_diag.in_endpoint = ep->bEndpointAddress;
      g_usb_diag.max_packet = ep->wMaxPacketSize;
      g_usb_diag.interval_ms = ep->bInterval;
    } else {
      s_xbox.out_pipe = pipe;
      s_xbox.out_ep = ep->bEndpointAddress;
    }
  }

  if (s_xbox.in_pipe == USB_HOST_PIPE_NONE) {
    (void)Xbox_Class_DeInit(phost);
    return USBH_FAIL;
  }

  return USBH_OK;
}

static USBH_StatusTypeDef Xbox_Class_DeInit(USBH_HandleTypeDef *phost) {
  if (phost == NULL || phost != &s_host) {
    return USBH_FAIL;
  }

  if (s_xbox.in_pipe != USB_HOST_PIPE_NONE) {
    (void)USBH_LL_ClosePipe(phost, s_xbox.in_pipe);
    (void)USBH_FreePipe(phost, s_xbox.in_pipe);
    s_xbox.in_pipe = USB_HOST_PIPE_NONE;
  }
  if (s_xbox.out_pipe != USB_HOST_PIPE_NONE) {
    (void)USBH_LL_ClosePipe(phost, s_xbox.out_pipe);
    (void)USBH_FreePipe(phost, s_xbox.out_pipe);
    s_xbox.out_pipe = USB_HOST_PIPE_NONE;
  }

  s_xbox.pending = 0;
  s_xbox.clearing = 0;
  s_ready = 0;

  return USBH_OK;
}

/* class 不需要额外的 class 请求 */
static USBH_StatusTypeDef Xbox_Class_Requests(USBH_HandleTypeDef *phost) {
  (void)phost;
  return USBH_OK;
}

/* 核心：非阻塞轮询 IN URB。每帧推进一次，不等待。 */
static USBH_StatusTypeDef Xbox_Class_Process(USBH_HandleTypeDef *phost) {
  if (!Usb_Host_Valid(phost) || s_xbox.in_pipe == USB_HOST_PIPE_NONE) {
    return USBH_FAIL;
  }
  if (phost->device.is_connected == 0) {
    return USBH_BUSY;
  }

  /* STALL 后要清端点特性，再从头开始 */
  if (s_xbox.clearing != 0) {
    const USBH_StatusTypeDef ret = USBH_ClrFeature(phost, s_xbox.in_ep);

    if (ret == USBH_OK) {
      s_xbox.clearing = 0;
      (void)USBH_LL_SetToggle(phost, s_xbox.in_pipe, 0);
    } else if (ret != USBH_BUSY) {
      s_ready = 0;
    }
    return USBH_BUSY;
  }

  if (s_xbox.pending != 0) {
    const USBH_URBStateTypeDef state =
        USBH_LL_GetURBState(phost, s_xbox.in_pipe);
    const uint32_t elapsed = phost->Timer - s_xbox.submitted;

    if (state == USBH_URB_DONE) {
      const uint32_t len = USBH_LL_GetLastXferSize(phost, s_xbox.in_pipe);
      s_xbox.pending = 0;

      if (len > 0 && len <= USB_HOST_PAYLOAD) {
        Xbox_State parsed;

        if (Xbox_Parse(s_xbox.buf, (uint16_t)len, &parsed)) {
          s_state = parsed;
          s_ready = 1;
          g_usb_diag.received++;
        }
      }
    } else if (state == USBH_URB_STALL) {
      s_xbox.pending = 0;
      s_xbox.clearing = 1;
      g_usb_diag.transfer_errors++;
      s_ready = 0;
    } else if (state == USBH_URB_ERROR) {
      s_xbox.pending = 0;
      g_usb_diag.transfer_errors++;
      s_ready = 0;
    } else if (state == USBH_URB_NOTREADY || state == USBH_URB_NAK_WAIT ||
               state == USBH_URB_NYET) {
      /* 无线接收器没新数据时一直 NAK，到 interval 就重新提交，不算错误 */
      if (elapsed >= s_xbox.interval_ms) {
        s_xbox.pending = 0;
      }
    } else if (elapsed > USB_HOST_URB_TIMEOUT_MS) {
      s_xbox.pending = 0;
      g_usb_diag.transfer_errors++;
    }
  }

  if (s_xbox.pending == 0 && s_xbox.clearing == 0 &&
      (phost->Timer - s_xbox.submitted) >= s_xbox.interval_ms) {
    if (USBH_InterruptReceiveData(phost, s_xbox.buf, s_xbox.packet_size,
                                  s_xbox.in_pipe) == USBH_OK) {
      s_xbox.pending = 1;
      s_xbox.submitted = phost->Timer;
    }
  }

  return USBH_BUSY;
}

/* SOF 回调在 HCD 中断里跑，这里不能解析或排队任何东西 */
static USBH_StatusTypeDef Xbox_Class_Sof(USBH_HandleTypeDef *phost) {
  return Usb_Host_Valid(phost) ? USBH_OK : USBH_FAIL;
}

static USBH_ClassTypeDef s_xbox_class = {
    "Xbox360",          0xFF,
    Xbox_Class_Init,    Xbox_Class_DeInit,
    Xbox_Class_Requests, Xbox_Class_Process,
    Xbox_Class_Sof,     NULL,
};

/* ==================== 对外接口 ==================== */

void Usb_Host_Init(void) {
  memset((void *)&g_usb_diag, 0, sizeof(g_usb_diag));
  memset((void *)&s_state, 0, sizeof(s_state));
  s_ready = 0;

  Usb_Host_Hcd_Init();

  if (USBH_Init(&s_host, NULL, 0) != USBH_OK) {
    Error_Handler();
  }
  if (USBH_RegisterClass(&s_host, &s_xbox_class) != USBH_OK) {
    Error_Handler();
  }
  if (USBH_Start(&s_host) != USBH_OK) {
    Error_Handler();
  }
}

void Usb_Host_Process(void) {
  (void)USBH_Process(&s_host);
  g_usb_diag.host_state = (uint32_t)s_host.gState;
  g_usb_diag.ready = s_ready;
}

uint8_t Usb_Host_Is_Ready(void) { return s_ready; }

void Usb_Host_Get_Xbox(Xbox_State *out) {
  if (out == NULL) {
    return;
  }
  if (s_ready == 0) {
    memset(out, 0, sizeof(*out));
    return;
  }
  *out = s_state;
}
