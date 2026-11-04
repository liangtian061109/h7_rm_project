#ifndef C620_TASK_H
#define C620_TASK_H

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

/* 任务句柄。app_task.c 建任务时写入。 */
extern TaskHandle_t g_c620_task_handle;

/** @brief C620 控制任务入口。由 app_task.c 的任务表统一创建，不要在别处直接调。 */
void C620_Task(void *argument);

#endif
