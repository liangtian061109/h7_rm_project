#include "bmi088_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/* BMI088 数据就绪中断。这是 ISR 不是任务：不能阻塞、不能调 HAL_Delay、
 * 不能用带锁接口，只给任务发通知，重活留给任务。
 *
 * 不在 ISR 里直接读 SPI：DMA 收发有临界区和状态机，会拉长中断，
 * 还会和 BSP 的 busy 逻辑打架。 */

/* 两个数据就绪中断各占一个通知位 */
#define IMU_FLAG_ACCEL_READY (1 << 0)
#define IMU_FLAG_GYRO_READY (1 << 1)

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  BaseType_t woken = pdFALSE;

  if (g_bmi088_task_handle == NULL) {
    return; /* 任务还没建好，丢掉这次边沿 */
  }

  if (GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin) {
    (void)xTaskNotifyFromISR(g_bmi088_task_handle, IMU_FLAG_ACCEL_READY,
                             eSetBits, &woken);
  } else if (GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin) {
    (void)xTaskNotifyFromISR(g_bmi088_task_handle, IMU_FLAG_GYRO_READY, eSetBits,
                             &woken);
  }

  portYIELD_FROM_ISR(woken);
}
