#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "fdcan.h"
#include "main.h"
#include <stdint.h>

/* CAN 收发封装（STM32H7 FDCAN）。
 *
 * 用法：Register 一批实例 → CAN_Init 启动 → 回调里取数据。
 * 顺序要求：Register 必须在 CAN_Init 之前，过滤器要在总线启动前
 * 写进 Message RAM，启动后再改需要进 INIT 模式。 */

/* 最多注册多少个接收实例（每个占一个过滤器元素）。
 * 要和 fdcan.c 的 hfdcan1.Init.StdFiltersNbr 对得上，超了返回 NULL。 */
#define CAN_MAX_INSTANCES 16

/* 兼容 F4 写法，C620 之类的模块不用改 ID 类型宏 */
#ifndef CAN_ID_STD
#define CAN_ID_STD FDCAN_STANDARD_ID
#endif
#ifndef CAN_ID_EXT
#define CAN_ID_EXT FDCAN_EXTENDED_ID
#endif

typedef struct CAN_S {
  FDCAN_HandleTypeDef *handle; /* 本工程是 &hfdcan1 */
  uint32_t rx_id;              /* 期望接收的 ID */
  uint32_t rx_ide;             /* FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID */
  uint32_t filter_index;       /* 占用的过滤器元素下标 */
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8]; /* 经典帧最多 8 字节 */
  uint8_t rx_len;     /* 已从 DLC 编码换算成字节数 */
  uint32_t rx_count;  /* 累计收到的帧数，0 表示从没通过 */
  uint32_t last_rx_tick;
  void (*rx_callback)(struct CAN_S *can); /* 在中断里调用 */
  void *id;                               /* 回调时回传的私有数据 */
} CAN_Instance;

typedef struct {
  FDCAN_HandleTypeDef *handle;
  uint32_t rx_id;
  uint32_t rx_ide;
  void (*rx_callback)(struct CAN_S *can);
  void *id;
} CAN_Init_Config_s;

/* 启动总线并打开 Rx FIFO0 中断。必须在所有 CAN_Register 之后调用。 */
HAL_StatusTypeDef CAN_Init(FDCAN_HandleTypeDef *handle);

/* 注册一个接收 ID，独占一个过滤器元素（掩码模式精确匹配）。
 * 失败（参数非法/ID 重复/元素用尽）返回 NULL。 */
CAN_Instance *CAN_Register(CAN_Init_Config_s *conf);

/* 发一帧数据帧。len 为 1~8 字节。
 * 返回 HAL_BUSY 表示发送队列满，HAL_ERROR 表示参数非法。 */
HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *handle, uint32_t id,
                               uint32_t ide, const uint8_t *data, uint8_t len);

#endif
