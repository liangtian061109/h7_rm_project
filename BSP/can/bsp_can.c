#include "bsp_can.h"
#include "stm32h7xx_hal_fdcan.h"

#include <stdint.h>
#include <string.h>

static CAN_Instance can_instances[CAN_MAX_INSTANCES];
static uint8_t can_instance_count;

/* FDCAN 的 DataLength 是编码值不是字节数：
 * 0~8 → 就是字节数，9~15 → 12/16/20/24/32/48/64。
 * 收到非法 DLC 返回 0，否则 memcpy 会读越界。 */
static uint8_t CAN_DlcToBytes(uint32_t dlc) {
  switch (dlc) {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default: return 0;
  }
}

static uint32_t CAN_BytesToDlc(uint8_t len) {
  static const uint32_t table[9] = {
      FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1, FDCAN_DLC_BYTES_2,
      FDCAN_DLC_BYTES_3, FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
      FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7, FDCAN_DLC_BYTES_8,
  };
  return (len <= 8) ? table[len] : FDCAN_DLC_BYTES_8;
}

/* 标准元素和扩展元素**各自从 0 编号**，所以扫描必须连 ide 一起比。
 * 只比 filter_index 的话，标准 0 号和扩展 0 号会被判成冲突。 */
static uint32_t CAN_NextFilterIndex(FDCAN_HandleTypeDef *handle, uint32_t ide) {
  const uint32_t total = (ide == FDCAN_STANDARD_ID) ? handle->Init.StdFiltersNbr
                                                    : handle->Init.ExtFiltersNbr;

  for (uint32_t idx = 0; idx < total; idx++) {
    uint8_t used = 0;

    for (uint8_t i = 0; i < can_instance_count; i++) {
      if (can_instances[i].handle == handle &&
          can_instances[i].rx_ide == ide &&
          can_instances[i].filter_index == idx) {
        used = 1;
        break;
      }
    }

    if (used == 0) {
      return idx;
    }
  }
  return 0xFFFFFFFF;
}

/* FDCAN 没有 IDLIST，用 MASK 模式精确匹配：FilterID2 全 1 表示每位都必须相同。 */
static HAL_StatusTypeDef CAN_AddFilter(FDCAN_HandleTypeDef *handle,
                                       uint32_t rx_id, uint32_t rx_ide,
                                       uint32_t index) {
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = rx_ide;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = rx_id;
  filter.FilterID2 = (rx_ide == FDCAN_STANDARD_ID) ? 0x7FF : 0x1FFFFFFF;
  return HAL_FDCAN_ConfigFilter(handle, &filter);
}

HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *handle) {
  if (handle == NULL || handle->Instance == NULL) {
    return HAL_ERROR;
  }

  const HAL_FDCAN_StateTypeDef state = HAL_FDCAN_GetState(handle);

  if (state == HAL_FDCAN_STATE_READY) {
    /* 不配全局过滤器的话 GFC 复位值是 0，所有非匹配帧都会塞进 FIFO0，
     * 总线上别人的帧也会触发中断。 */
    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
      return HAL_ERROR;
    }

    if (HAL_FDCAN_Start(handle) != HAL_OK) {
      return HAL_ERROR;
    }
  } else if (state != HAL_FDCAN_STATE_BUSY) {
    /* BUSY = 已经在跑（重复调用），其余状态拒绝启动 */
    return HAL_ERROR;
  }

  /* 只订阅 Rx FIFO0 新报文。错误中断和总线关闭中断暂不开，
   * 它们走同一条中断线，但 HAL 的默认错误回调是空的。 */
  if (HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                     0) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

CAN_Instance *CAN_Register(CAN_Init_Config_s *conf) {
  if (conf == NULL || conf->handle == NULL || conf->handle->Instance == NULL ||
      conf->rx_callback == NULL || can_instance_count >= CAN_MAX_INSTANCES) {
    return NULL;
  }
  if (conf->rx_ide == FDCAN_STANDARD_ID && conf->rx_id > 0x7FF) {
    return NULL;
  }
  if (conf->rx_ide == FDCAN_EXTENDED_ID && conf->rx_id > 0x1FFFFFFF) {
    return NULL;
  }
  if (conf->rx_ide != FDCAN_STANDARD_ID && conf->rx_ide != FDCAN_EXTENDED_ID) {
    return NULL;
  }

  /* 同一总线同一 ID 只允许一个实例，否则两帧数据投给两个实例，谁最后写的说不清 */
  for (uint8_t i = 0; i < can_instance_count; i++) {
    if (can_instances[i].handle == conf->handle &&
        can_instances[i].rx_ide == conf->rx_ide &&
        can_instances[i].rx_id == conf->rx_id) {
      return NULL;
    }
  }

  const uint8_t index = can_instance_count;
  CAN_Instance *can = &can_instances[index];
  memset(can, 0, sizeof(CAN_Instance)); /* 槽位可能被上次失败的注册用过 */
  can->handle = conf->handle;
  can->rx_id = conf->rx_id;
  can->rx_ide = conf->rx_ide;
  can->rx_callback = conf->rx_callback;
  can->id = conf->id;

  const uint32_t filter_index = CAN_NextFilterIndex(conf->handle, conf->rx_ide);
  if (filter_index == 0xFFFFFFFF ||
      CAN_AddFilter(conf->handle, conf->rx_id, conf->rx_ide, filter_index) !=
          HAL_OK) {
    memset(can, 0, sizeof(CAN_Instance));
    return NULL;
  }
  can->filter_index = filter_index;

  can_instance_count = (uint8_t)(index + 1);

  return can;
}

HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *handle, uint32_t id,
                               uint32_t ide, const uint8_t *data, uint8_t len) {
  if (handle == NULL || handle->Instance == NULL || data == NULL) {
    return HAL_ERROR;
  }
  if (len == 0 || len > 8) {
    return HAL_ERROR;
  }
  if (ide != FDCAN_STANDARD_ID && ide != FDCAN_EXTENDED_ID) {
    return HAL_ERROR;
  }
  if (ide == FDCAN_STANDARD_ID && id > 0x7FF) {
    return HAL_ERROR;
  }
  if (ide == FDCAN_EXTENDED_ID && id > 0x1FFFFFFF) {
    return HAL_ERROR;
  }

  /* 队列满直接返回 BUSY，不阻塞。1 kHz 控制循环里阻塞会拖死整个任务，
   * 真满了说明总线出错（比如没接收器导致重传堆积）。 */
  if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0) {
    return HAL_BUSY;
  }

  FDCAN_TxHeaderTypeDef tx = {0};
  tx.Identifier = id;
  tx.IdType = ide;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = CAN_BytesToDlc(len);
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_CLASSIC_CAN; /* C620/GM6020 只认经典帧 */
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx, (uint8_t *)data);
}

/* 必须把 FIFO 读空：不读的话满了之后新帧被硬件直接丢掉，而且不报错。 */
static void CAN_RxDispatch(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_RxHeaderTypeDef hdr;
  uint8_t data[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK) {
      break;
    }

    const uint32_t id = hdr.Identifier;
    uint8_t dlc = CAN_DlcToBytes(hdr.DataLength);
    if (dlc > 8) {
      dlc = 8; /* 缓冲区只有 8 字节，FD 长帧也只拷前 8 字节 */
    }

    for (uint8_t i = 0; i < can_instance_count; i++) {
      CAN_Instance *can = &can_instances[i];
      if (can->handle != hfdcan) {
        continue;
      }
      if (can->rx_ide != hdr.IdType || can->rx_id != id) {
        continue;
      }

      memcpy(can->rx_data, data, dlc);
      can->rx_len = dlc;
      can->rx_header = hdr;
      can->rx_count++;
      can->last_rx_tick = HAL_GetTick();
      if (can->rx_callback != NULL) {
        can->rx_callback(can);
      }
    }
  }
}

/* H7 的回调签名和 F4 不同，多一个 RxFifo0ITs 参数 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  (void)RxFifo0ITs;
  CAN_RxDispatch(hfdcan);
}
