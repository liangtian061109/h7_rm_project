#ifndef USB_HOST_H
#define USB_HOST_H

#include <stdint.h>

#include "xbox.h"

/* USB Host（H7 FDCAN 无关，独立外设）。接 Xbox 360 兼容接收器。
 *
 * 用法：Usb_Host_Init() 在调度器启动前调一次，
 * 然后由 rc_task 每 1 ms 调 Usb_Host_Process() 推进状态机。
 * 注意 USBH_Process 内部枚举时会阻塞（vTaskDelay），不要放在控制任务里。 */

/* 调试镜像，调试器直接看 */
typedef struct {
  uint32_t connects;        /* 根端口连接次数 */
  uint32_t disconnects;     /* 断开次数 */
  uint32_t received;        /* 收到的有效报告数 */
  uint32_t transfer_errors; /* 硬故障，不含普通 NAK */
  uint32_t host_state;      /* ST 状态机值，卡住时看这个 */
  uint16_t vid;
  uint16_t pid;
  uint16_t max_packet;
  uint8_t interval_ms;
  uint8_t in_endpoint;
  uint8_t ready; /* 1 = 已连接并收到过报告 */
} Usb_Host_Diag;

extern volatile Usb_Host_Diag g_usb_diag;

/* 配置 HCD、注册 class。必须在调度器启动前调用（内部用 HAL_Delay 等设备稳定）。 */
void Usb_Host_Init(void);

/* 推进 Host 状态机 + 非阻塞收报告。每 1 ms 调一次，只能在任务里调。 */
void Usb_Host_Process(void);

/* 1 = 手柄已连接且有数据 */
uint8_t Usb_Host_Is_Ready(void);

/* 取最新手柄状态。未连接时返回全 0。 */
void Usb_Host_Get_Xbox(Xbox_State *out);

#endif
