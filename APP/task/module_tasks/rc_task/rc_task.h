#ifndef RC_TASK_H
#define RC_TASK_H

#include "FreeRTOS.h"
#include "task.h"

/* 遥控任务：轮询 USB Host，把手柄摇杆映射成底盘速度。
 * 由 app_task.c 的任务表统一创建。 */

/* 遥控使能。0 = 不接管底盘速度，调试器仍可手动改 g_chassis_vx/vy/wz；
 * 1 = 手柄接管。默认关，避免插上手柄误碰摇杆车就窜出去。 */
extern volatile uint8_t g_rc_enable;

/* 手柄是否在线（1 = 收到过有效报告） */
extern volatile uint8_t g_rc_online;

extern TaskHandle_t g_rc_task_handle;

void RC_Task(void *argument);

#endif
