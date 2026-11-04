#ifndef BMI088_TASK_H
#define BMI088_TASK_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "attitude.h"

/* 全局为了调试器能 watch，只有 BMI088 任务会写 */
extern Attitude_VQF g_attitude;

/* EXTI 回调用它发通知 */
extern TaskHandle_t g_bmi088_task_handle;

/* 由 app_task.c 的任务表统一创建，不要在别处直接调 */
void BMI088_Task(void *argument);

#endif
