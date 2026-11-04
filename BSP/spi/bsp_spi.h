#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "main.h"
#include <stdint.h>

#define SPI_MAX_INSTANCES 4
#define SPI_BUFFER_SIZE 512

/* 一个实例代表一个 SPI 从设备，多个设备可共享同一总线靠片选区分。
 * 实例由 BSP 静态分配（.dma_buffer 段），注册后不再移动。 */
typedef struct SPI_Instance_s {
  SPI_HandleTypeDef *handle; /* SPI 硬件句柄 */
  GPIO_TypeDef *cs_port;     /* 片选 GPIO 端口，低电平有效 */
  uint16_t cs_pin;           /* 片选 GPIO 引脚 */
  void *id;                  /* 用户私有数据（设备对象指针等），回调里用它区分设备 */

  /* 在 DMA 完成中断里调用。不要耗时操作，也不要再对本实例发起新传输 */
  void (*callback)(struct SPI_Instance_s *spi);

  volatile uint8_t busy; /* 重入保护：一次事务进行中置 1 */

  /* 常驻 DMA 收发缓冲（RAM_D1，DMA 访问不到 DTCM，所以不能放栈/普通 .bss） */
  uint8_t __attribute__((aligned(32))) tx_buffer[SPI_BUFFER_SIZE];
  uint8_t __attribute__((aligned(32))) rx_buffer[SPI_BUFFER_SIZE];

  uint16_t tx_length; /* 发送段长度：命令/哑字节的个数 */
  uint16_t rx_length; /* 有效接收字节数，数据从 rx_buffer[tx_length] 开始 */
  uint32_t rx_timestamp; /* 接收完成时的 HAL_GetTick() 毫秒时间戳 */

  /* 运行统计，方便查通信问题 */
  uint32_t error_count;
  uint32_t busy_count;
  uint32_t start_failure_count;
  uint8_t last_status; /* 最近一次启动传输的 HAL 状态 */
} SPI_Instance;

typedef struct {
  SPI_HandleTypeDef *handle; /* SPI 硬件句柄 */
  GPIO_TypeDef *cs_port;     /* 片选 GPIO 端口 */
  uint16_t cs_pin;           /* 片选 GPIO 引脚 */
  void *id;                  /* 用户私有数据，可填 NULL */
  void (*callback)(struct SPI_Instance_s *spi); /* 事务完成回调，可填 NULL */
} SPI_Init_Config_s;

/* 启动时调用一次（在 MX_SPIx_Init 之后）。参数无效、同一句柄+同一片选重复注册
 * 或实例池已满时返回 NULL。
 */
SPI_Instance *SPI_Register(const SPI_Init_Config_s *conf);

/* 异步接口：立即返回，收发由 DMA 完成。完成后中断里拉高片选 → 调 callback → 释放 busy。
 * 返回 HAL_OK 只表示"DMA 已启动"，数据要等回调。
 * 调用方缓冲区返回后即可复用（BSP 先拷进常驻缓冲）。 */
HAL_StatusTypeDef SPI_Transmit(SPI_Instance *spi, const uint8_t *data, uint16_t size);
HAL_StatusTypeDef SPI_Receive(SPI_Instance *spi, uint16_t size);
HAL_StatusTypeDef SPI_TransmitReceive(SPI_Instance *spi, const uint8_t *tx,
                                      uint16_t tx_size, uint16_t rx_size);

/* 同步阻塞发送，只用于寄存器写这种短事务。返回时总线已真正发完、
 * 片选已拉高、busy 已释放，接着读不会和上一笔打架。timeout_ms 给 5 就够。 */
HAL_StatusTypeDef SPI_TransmitBlocking(SPI_Instance *spi, const uint8_t *data,
                                       uint16_t size, uint32_t timeout_ms);

/* 强制中止当前事务并把片选拉高（硬件异常/超时后的恢复手段），只能在任务里调用 */
HAL_StatusTypeDef SPI_Abort(SPI_Instance *spi);

#endif /* BSP_SPI_H */
