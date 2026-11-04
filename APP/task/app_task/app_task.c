#include "app_task.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "app_init.h"
#include "bmi088_task.h"
#include "c620_task.h"
#include "rc_task.h"

/* 所有任务的栈、优先级、句柄集中在这张表里，一眼看全系统资源分配。
 * 任务函数本身在各自模块里，这里只管"按什么参数把它跑起来"。
 * 加任务就是加一行，不用再写一个 xxx_Task_Init()。 */

typedef struct {
  TaskFunction_t entry;
  const char *name;     /* 栈溢出钩子和调试器里显示的就是它 */
  uint16_t stack_words; /* 单位 word（4 字节），不是字节 */
  UBaseType_t priority; /* 数值越大越优先 */
  TaskHandle_t *handle; /* ISR 发通知要用 */
} Task_Config_s;

/* 按优先级从高到低排列。栈够不够别靠猜，跑起来看
 * uxTaskGetStackHighWaterMark()，返回历史最小剩余 words，低于 50 就该加。 */
static const Task_Config_s task_table[] = {
    {BMI088_Task, "bmi088", 512, 54, &g_bmi088_task_handle},
#if APP_CAN_ENABLE
    {C620_Task, "c620", 384, 50, &g_c620_task_handle},
#endif
#if APP_USB_HOST_ENABLE
    /* 优先级低于控制任务：USB 枚举时会 vTaskDelay，不能挡住电机控制 */
    {RC_Task, "rc", 512, 30, &g_rc_task_handle},
#endif
};

bool App_Init_Tasks(void) {
  /* 设备没就绪就别建了，任务里拿不到实例，跑起来也是空转 */
  if (g_app_status != APP_DEVICES_READY) {
    return false;
  }
  if (App_Get_BMI088() == NULL) {
    return false;
  }

  const uint32_t task_count =
      (uint32_t)(sizeof(task_table) / sizeof(task_table[0]));

  for (uint32_t i = 0; i < task_count; i++) {
    const Task_Config_s *task = &task_table[i];

    /* 建失败多半是 FreeRTOS 堆不够（configTOTAL_HEAP_SIZE） */
    if (xTaskCreate(task->entry, task->name, task->stack_words, NULL,
                    task->priority, task->handle) != pdPASS) {
      g_app_status = APP_TASK_CREATE_FAILED;
      return false;
    }
  }

  g_app_status = APP_READY;
  return true;
}
