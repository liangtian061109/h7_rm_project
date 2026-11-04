#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdbool.h>

/* 所有任务的总入口。栈、优先级、句柄都在 app_task.c 的表里，
 * 这里只管按顺序把它们建起来。 */

/* 必须在 osKernelInitialize() 之后调用（freertos.c 的 USER CODE BEGIN
 * RTOS_THREADS 区）。前置条件：App_Init_Devices() 已成功。 */
bool App_Init_Tasks(void);

#endif
