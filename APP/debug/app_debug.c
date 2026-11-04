#include "app_debug.h"

#include "FreeRTOS.h"
#include "task.h"

/* ==================== 断言 / 栈溢出现场 ==================== */

const char *volatile g_assert_file = NULL;
volatile int g_assert_line = 0;
const char *volatile g_overflow_task = NULL;

/* configASSERT 落点：先记现场，再关中断停住，方便挂调试器定位 */
void vAssertCalled(const char *file, int line) {
  g_assert_file = file;
  g_assert_line = line;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

/* FreeRTOS 栈溢出钩子：记录任务名后停住，调试器里看 g_overflow_task */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  g_overflow_task = pcTaskName;
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

/* ==================== 姿态输出（BMI088 任务写） ==================== */

volatile float g_att_roll = 0.0f;
volatile float g_att_pitch = 0.0f;
volatile float g_att_yaw = 0.0f;
volatile float g_att_bias[3] = {0.0f, 0.0f, 0.0f};
volatile uint8_t g_att_rest = 0;

volatile float g_att_roll_deg = 0.0f;
volatile float g_att_pitch_deg = 0.0f;
volatile float g_att_yaw_deg = 0.0f;
volatile float g_att_yaw_total_deg = 0.0f;

volatile float g_att_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float g_yaw_rate_deg_s = 0.0f;

volatile uint32_t g_att_dt_clamped = 0;
volatile float g_att_dt_lost_s = 0.0f;
volatile uint32_t g_att_rejected = 0;

/* ==================== 零位标定状态 ==================== */

volatile float g_att_roll_zero_deg = 0.0f;
volatile float g_att_pitch_zero_deg = 0.0f;
volatile float g_att_yaw_zero_deg = 0.0f;
volatile uint8_t g_att_zero_ready = 0;
volatile uint8_t g_att_zero_progress = 0;

/* ==================== 复位原因 ==================== */

volatile uint32_t g_reset_flags = 0;

/* ==================== C620 状态（C620 任务写） ==================== */

volatile float g_c620_angle_deg[APP_C620_MOTOR_NUM] = {0.0f};
volatile float g_c620_speed_rpm[APP_C620_MOTOR_NUM] = {0.0f};
volatile float g_c620_current_a[APP_C620_MOTOR_NUM] = {0.0f};
volatile uint8_t g_c620_online[APP_C620_MOTOR_NUM] = {0};
volatile uint8_t g_c620_error_code[APP_C620_MOTOR_NUM] = {0};

volatile uint32_t g_c620_send_fail = 0;

volatile float g_c620_pid_err[APP_C620_MOTOR_NUM] = {0.0f};
volatile float g_c620_pid_out[APP_C620_MOTOR_NUM] = {0.0f};
volatile float g_c620_pid_integral[APP_C620_MOTOR_NUM] = {0.0f};
volatile float g_c620_target_ramped[APP_C620_MOTOR_NUM] = {0.0f};

volatile uint8_t g_c620_blocked[APP_C620_MOTOR_NUM] = {0};
