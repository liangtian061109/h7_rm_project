#include "app_init.h"

#include "app_debug.h"
#include "fdcan.h"
#include "main.h"
#include "spi.h"
#include "usb_host.h"

/* 把 CubeMX 配好的外设变成"能用的设备"：IMU 和 CAN。
 * 全部跑在调度器启动之前，所以可以放心用 HAL_Delay 和忙等。 */

volatile App_Status_t g_app_status = APP_NOT_INITIALIZED;

/* 外部通过 App_Get_xxx() 拿，不要直接引用 */
BMI088_Instance *g_bmi088 = NULL;
C620_Instance *g_c620[APP_C620_MOTOR_NUM] = {NULL};

static const BMI088_Init_Config_s bmi088_config = {
    .handle = &hspi2,
    .acc_cs_port = BMI088_ACCEL_SPI_CS_GPIO_Port,
    .acc_cs_pin = BMI088_ACCEL_SPI_CS_Pin,
    .gyro_cs_port = BMI088_GYRO_SPI_CS_GPIO_Port,
    .gyro_cs_pin = BMI088_GYRO_SPI_CS_Pin,
    .callback = NULL,
    .id = NULL,
};

/* 电机安装方向：false 正装，true 反装（电流和反馈一起取反）。
 *
 * 标法：g_c620_mode 设成 1（开环电流），g_c620_test_motor 逐个试 0~3，
 * 每个给 +0.5A，看转向和"车往前走时这个轮子该转的方向"是否一致。
 * 只改这个数组，不用动 c620.c。 */
static const bool c620_reversed[APP_C620_MOTOR_NUM] = {
    false, /* 下标 0 → 电调 ID 1 */
    false, /* 下标 1 → 电调 ID 2 */
    false, /* 下标 2 → 电调 ID 3 */
    false, /* 下标 3 → 电调 ID 4 */
};

static bool App_Init_IMU(void) {
  g_bmi088 = BMI088_Register(&bmi088_config);
  if (g_bmi088 == NULL || BMI088_Init(g_bmi088) == 0) {
    return false;
  }

  /* 标定连续读 1000 帧、忙等约 50 ms。放进任务里会把 C620 饿死：
   * 电流指令停发，恢复后 vTaskDelayUntil 还会补发错过的 50 个周期，
   * 瞬间灌爆 8 深的 TxFIFO。放调度器启动前就没这个问题。 */
  HAL_Delay(300); /* 等电源稳定，静置采的零偏才准 */
  (void)BMI088_Calibrate_Gyro(g_bmi088); /* 失败不致命，offset 保持 0 照常采样 */

  return true;
}

/* 顺序不能反：C620_Init 配过滤器 → CAN_Init 启动总线。
 * 过滤器只能写进未启动的 Message RAM，反了就是静默失败（发送正常、接收为零）。 */
static bool App_Init_CAN(void) {
  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    /* 电调拨码 ID 从 1 开始，数组下标从 0 开始 */
    g_c620[i] = C620_Init((uint8_t)(i + 1), &hfdcan1, c620_reversed[i]);
    if (g_c620[i] == NULL) {
      return false;
    }
  }

  return CAN_Init(&hfdcan1) == HAL_OK;
}

bool App_Init_Devices(void) {
  /* 重复调用直接返回上次结果，避免重复注册把实例池占满 */
  if (g_app_status != APP_NOT_INITIALIZED) {
    return (g_app_status == APP_DEVICES_READY) || (g_app_status == APP_READY);
  }

  /* 尽早抓复位原因再清掉，下次复位才能重新记录。
   * 位定义见 stm32h723xx.h，和 F1/F4 不同。 */
  g_reset_flags = RCC->RSR;
  RCC->RSR |= RCC_RSR_RMVF;

  if (!App_Init_IMU()) {
    g_app_status = APP_IMU_INIT_FAILED;
    return false;
  }

#if APP_CAN_ENABLE
  if (!App_Init_CAN()) {
    g_app_status = APP_CAN_INIT_FAILED;
    return false;
  }
#else
  (void)App_Init_CAN; /* 开关关掉时引用一下，避免 -Wunused-function */
#endif

#if APP_USB_HOST_ENABLE
  /* USB Host 必须在调度器启动前初始化：内部枚举会用 HAL_Delay 等设备稳定 */
  Usb_Host_Init();
#endif

  g_app_status = APP_DEVICES_READY;
  return true;
}

BMI088_Instance *App_Get_BMI088(void) { return g_bmi088; }

C620_Instance *App_Get_C620(uint8_t index) {
  if (index >= APP_C620_MOTOR_NUM) {
    return NULL;
  }

  return g_c620[index];
}
