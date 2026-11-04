#include "bsp_spi.h"
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

/* 静态实例池：一个实例 = 一个 SPI 从设备。
 * 必须放在 RAM_D1 的 .dma_buffer 段 —— H7 的 DMA1 访问不到 DTCMRAM。
 * 段是 NOLOAD 的，启动时不清零，所以注册时会 memset 整个实例。
 */
static SPI_Instance spi_instances[SPI_MAX_INSTANCES]
    __attribute__((section(".dma_buffer"), aligned(32)));
static uint8_t spi_instance_count = 0;

/* 片选低电平选中，与 CubeMX 里 CS 上电默认高一致 */
static void SPI_Cs_Select(const SPI_Instance *spi)
{
  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_RESET);
}

static void SPI_Cs_Release(const SPI_Instance *spi)
{
  HAL_GPIO_WritePin(spi->cs_port, spi->cs_pin, GPIO_PIN_SET);
}

SPI_Instance *SPI_Register(const SPI_Init_Config_s *conf)
{
  if (conf == NULL || conf->handle == NULL || conf->handle->Instance == NULL ||
      conf->cs_port == NULL || conf->cs_pin == 0 ||
      spi_instance_count >= SPI_MAX_INSTANCES) {
    return NULL;
  }

  /* 同一总线 + 同一片选重复注册没意义，直接拒绝 */
  for (uint8_t i = 0; i < spi_instance_count; i++) {
    if (spi_instances[i].handle == conf->handle &&
        spi_instances[i].cs_port == conf->cs_port &&
        spi_instances[i].cs_pin == conf->cs_pin) {
      return NULL;
    }
  }

  /* 先完整写候选槽位，最后提交计数，失败路径不会占用实例 */
  SPI_Instance *spi = &spi_instances[spi_instance_count];
  memset(spi, 0, sizeof(SPI_Instance));
  spi->handle = conf->handle;
  spi->cs_port = conf->cs_port;
  spi->cs_pin = conf->cs_pin;
  spi->id = conf->id;
  spi->callback = conf->callback;
  spi->busy = 0;
  spi->last_status = (uint8_t)HAL_OK;
  spi_instance_count = (uint8_t)(spi_instance_count + 1);

  return spi;
}

/* 抢事务：同一硬件总线同一时刻只能有一个实例在传输（共用一组 DMA 流），
 * 因此除了检查自己 busy，还要检查同 handle 的其他实例。
 * 关中断做"检查 + 置位"，避免任务和中断竞争。
 */
static bool SPI_Begin(SPI_Instance *spi)
{
  if (spi == NULL || spi->handle == NULL || spi->handle->Instance == NULL) {
    return false;
  }

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  bool acquired = (spi->busy == 0);
  if (acquired) {
    for (uint8_t i = 0; i < spi_instance_count; i++) {
      if (&spi_instances[i] != spi && spi_instances[i].handle == spi->handle &&
          spi_instances[i].busy != 0) {
        acquired = false;
        break;
      }
    }
  }
  if (acquired) {
    spi->busy = 1;
    __DMB();
  } else {
    spi->busy_count++;
  }

  if (primask == 0) {
    __enable_irq();
  }
  return acquired;
}

/* 启动失败统一收尾：记录状态、放片选、释放事务 */
static void SPI_Start_Failed(SPI_Instance *spi, HAL_StatusTypeDef status)
{
  spi->start_failure_count++;
  spi->last_status = (uint8_t)status;
  SPI_Cs_Release(spi);
  spi->busy = 0;
  __DMB();
}

HAL_StatusTypeDef SPI_Transmit(SPI_Instance *spi, const uint8_t *data,
                               uint16_t size)
{
  if (data == NULL || size == 0 || size > SPI_BUFFER_SIZE) {
    return HAL_ERROR;
  }
  if (!SPI_Begin(spi)) {
    return HAL_BUSY;
  }

  memcpy(spi->tx_buffer, data, size);
  spi->tx_length = size;
  spi->rx_length = 0;

  SPI_Cs_Select(spi);
  const HAL_StatusTypeDef status =
      HAL_SPI_Transmit_DMA(spi->handle, spi->tx_buffer, size);
  if (status != HAL_OK) {
    SPI_Start_Failed(spi, status);
    return status;
  }
  spi->last_status = (uint8_t)status;
  return status;
}

HAL_StatusTypeDef SPI_TransmitBlocking(SPI_Instance *spi, const uint8_t *data,
                                       uint16_t size, uint32_t timeout_ms)
{
  if (spi == NULL || data == NULL || size == 0 || size > SPI_BUFFER_SIZE) {
    return HAL_ERROR;
  }
  if (!SPI_Begin(spi)) {
    return HAL_BUSY;
  }

  memcpy(spi->tx_buffer, data, size);
  spi->tx_length = size;
  spi->rx_length = 0;

  SPI_Cs_Select(spi);
  /* HAL 的阻塞发送内部会等 EOT（传输真正结束），所以返回后可以直接拉高片选 */
  const HAL_StatusTypeDef status =
      HAL_SPI_Transmit(spi->handle, spi->tx_buffer, size, timeout_ms);
  SPI_Cs_Release(spi);

  spi->last_status = (uint8_t)status;
  spi->busy = 0;
  __DMB();
  return status;
}

HAL_StatusTypeDef SPI_Receive(SPI_Instance *spi, uint16_t size)
{
  if (size == 0 || size > SPI_BUFFER_SIZE) {
    return HAL_ERROR;
  }
  if (!SPI_Begin(spi)) {
    return HAL_BUSY;
  }

  /* SPI 主机要收到时钟才有数据，所以发 size 个哑字节 0xFF 把时钟打出去；
   * 这类纯接收没有命令段，因此 tx_length = 0，有效数据从 rx_buffer[0] 开始。
   */
  memset(spi->tx_buffer, 0xFF, size);
  spi->tx_length = 0;
  spi->rx_length = size;

  SPI_Cs_Select(spi);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive_DMA(spi->handle, spi->tx_buffer, spi->rx_buffer,
                                  size);
  if (status != HAL_OK) {
    SPI_Start_Failed(spi, status);
    return status;
  }
  spi->last_status = (uint8_t)status;
  return status;
}

HAL_StatusTypeDef SPI_TransmitReceive(SPI_Instance *spi, const uint8_t *tx,
                                      uint16_t tx_size, uint16_t rx_size)
{
  if (tx == NULL || tx_size == 0 || rx_size == 0) {
    return HAL_ERROR;
  }
  if ((uint32_t)tx_size + (uint32_t)rx_size > SPI_BUFFER_SIZE) {
    return HAL_ERROR;
  }
  if (!SPI_Begin(spi)) {
    return HAL_BUSY;
  }

  /* 全双工 DMA 一次移出 tx_size + rx_size 个字节：
   * 前 tx_size 个是发送阶段（命令 + 哑字节），接收数据在 rx_buffer[tx_size] 起。
   */
  const uint16_t transfer_length = (uint16_t)(tx_size + rx_size);
  memset(spi->tx_buffer, 0, transfer_length);
  memset(spi->rx_buffer, 0, transfer_length);
  memcpy(spi->tx_buffer, tx, tx_size);
  spi->tx_length = tx_size;
  spi->rx_length = rx_size;

  SPI_Cs_Select(spi);
  const HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
      spi->handle, spi->tx_buffer, spi->rx_buffer, transfer_length);
  if (status != HAL_OK) {
    SPI_Start_Failed(spi, status);
    return status;
  }
  spi->last_status = (uint8_t)status;
  return status;
}

HAL_StatusTypeDef SPI_Abort(SPI_Instance *spi)
{
  if (spi == NULL || spi->handle == NULL) {
    return HAL_ERROR;
  }

  const HAL_StatusTypeDef status = HAL_SPI_Abort(spi->handle);
  SPI_Cs_Release(spi);
  spi->busy = 0;
  spi->last_status = (uint8_t)status;
  __DMB();
  return status;
}

/* 用 handle 找到当前正在传输的实例（同一总线上最多一个 busy） */
static SPI_Instance *SPI_Find_Active(SPI_HandleTypeDef *handle)
{
  for (uint8_t i = 0; i < spi_instance_count; i++) {
    if (spi_instances[i].handle == handle && spi_instances[i].busy != 0) {
      return &spi_instances[i];
    }
  }
  return NULL;
}

/* 事务完成（中断上下文）：先拉高片选，再打时间戳，再回调，最后释放 busy */
static void SPI_Complete(SPI_Instance *spi)
{
  SPI_Cs_Release(spi);
  spi->rx_timestamp = HAL_GetTick();

  if (spi->callback != NULL) {
    spi->callback(spi);
  }

  spi->busy = 0;
  __DMB();
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Instance *spi = SPI_Find_Active(hspi);
  if (spi != NULL) {
    SPI_Complete(spi);
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Instance *spi = SPI_Find_Active(hspi);
  if (spi != NULL) {
    SPI_Complete(spi);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Instance *spi = SPI_Find_Active(hspi);
  if (spi != NULL) {
    SPI_Complete(spi);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  SPI_Instance *spi = SPI_Find_Active(hspi);
  if (spi == NULL) {
    return;
  }

  /* HAL 在 DMA 出错时已经停掉两条流，这里只负责放片选、释放事务，别让总线卡死 */
  spi->error_count++;
  SPI_Cs_Release(spi);
  spi->busy = 0;
  __DMB();
}
