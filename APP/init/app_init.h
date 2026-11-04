#ifndef APP_INIT_H
#define APP_INIT_H

#include <stdbool.h>

#include "bmi088.h"
#include "c620.h"

/* 这条总线上接了几个 C620。改这里要确认 fdcan.c 的 StdFiltersNbr 留够
 * （每个电机占一个过滤器元素）。 */
#define APP_C620_MOTOR_NUM 4

/* CAN 功能开关：0 = 临时跳过，调其他功能时避免 CAN 故障拖累整个 APP */
#define APP_CAN_ENABLE 1

/* USB Host 开关：接 Xbox 手柄用。0 = 不初始化，省 3KB FLASH */
#define APP_USB_HOST_ENABLE 1

typedef enum {
  APP_NOT_INITIALIZED = 0, /* 还没跑 App_Init_Devices() */
  APP_DEVICES_READY,       /* 设备好了，可以建任务 */
  APP_IMU_INIT_FAILED,
  APP_CAN_INIT_FAILED,
  APP_TASK_CREATE_FAILED, /* 多半是 FreeRTOS 堆不够 */
  APP_READY,
} App_Status_t;

/* 全局而非 static：调试器直接 watch，看卡在哪一步 */
extern volatile App_Status_t g_app_status;

/* 必须在 MX_xxx_Init() 之后、osKernelInitialize() 之前调用（main.c 的
 * USER CODE BEGIN 2），因为内部要用 HAL_Delay 和忙等做陀螺标定。 */
bool App_Init_Devices(void);

BMI088_Instance *App_Get_BMI088(void);

/* index 从 0 开始，对应电调 ID index+1。越界返回 NULL。 */
C620_Instance *App_Get_C620(uint8_t index);

#endif
