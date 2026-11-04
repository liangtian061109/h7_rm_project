#include "c620_task.h"

#include "app_debug.h"
#include "app_init.h"
#include "c620.h"
#include "chassis.h"
#include "main.h"
#include "pid.h"
#include "slope.h"

/* 栈大小和优先级在 app_task.c 的任务表里，这里只管任务怎么跑。 */

#define C620_TASK_PERIOD_MS 1 /* C620 期望 1 kHz */
#define C620_CTRL_DT 0.001f   /* 必须和上一行一致 */

/* 电流原始值 ↔ 安培：16384 / 20 = 819.2 */
#define C620_RAW_PER_AMP ((float)C620_CURRENT_MAX / C620_CURRENT_MAX_A)

/* ==================== 可实时修改的调参变量 ====================
 * 普通全局变量，调试器直接写内存，下一帧生效，不用重烧。
 * 改 speed_pid[i].kp 无效 —— 那是每帧被覆盖的副本。 */

/* 单电机电流上限。1.25A = 整车 5A 平均分到 4 个轮子。
 * 这是相电流，低速时约 10W/电机，四个 41W，卡在步兵 45W 预算内；
 * 转速越高同样电流功率越大，高速会超，靠 60J 缓冲能量顶。 */
float g_c620_speed_limit_a = 1.25f;

/* Kp 量纲是「电流原始值 / rpm」。限幅 1024 时，误差 8.5 rpm 就接近满输出。 */
float g_c620_speed_kp = 120.0f;
float g_c620_speed_ki = 30.0f;
float g_c620_speed_kd = 0.0f;

/* 前馈：out += kf × target。标定值 = 稳态电流 / 目标转速，实测约 3.1。 */
float g_c620_speed_kf = 3.0f;

/* 积分限幅 = out_max × 这个比例。
 * 太小压不住稳态误差（目标 200 需要约 590，0.25 只给 256）；
 * 太大目标降 0 时退得慢，表现为刹不住。经验 0.5~0.7。 */
float g_c620_speed_integral_ratio = 0.6f;

/* 目标斜坡，单位 rpm/ms。减速比加速快是故意的：加速太快打滑，减速要干脆。 */
float g_c620_ramp_inc = 1.5f;
float g_c620_ramp_dec = 3.0f;

/* 四轮目标转速（输出轴 rpm），由 C620_UpdateTarget 每帧写。 */
float g_c620_target_rpm[APP_C620_MOTOR_NUM] = {0};

static Pid_Instance speed_pid[APP_C620_MOTOR_NUM];
static Slope_Instance s_slope[APP_C620_MOTOR_NUM];

/* ==================== 测试模式 ====================
 * 运行时可切，调试器改 g_c620_mode 即可。顺序：1 → 2 → 3 → 0。 */
#define C620_MODE_NORMAL 0     /* 解算 → 速度环（跑车） */
#define C620_MODE_OPEN_LOOP 1  /* 单电机固定电流，验接线/转向 */
#define C620_MODE_SPEED 2      /* 单电机闭环固定目标，调 PID */
#define C620_MODE_KINEMATICS 3 /* 只解算，电流保持 0，验公式 */

volatile uint8_t g_c620_mode = C620_MODE_NORMAL;

volatile uint8_t g_c620_test_motor = 0;         /* 0~3，对应电调 ID 1~4 */
volatile float g_c620_test_current_a = 0.5f;    /* 开环模式电流 */
volatile float g_c620_test_target_rpm = 100.0f; /* 闭环模式目标 */

TaskHandle_t g_c620_task_handle = NULL;

/* 堵转后多久重试。不重试的话目标还在、转速仍是 0，条件永远成立，电机再也不转。 */
#define C620_BLOCK_RETRY_MS 2000
static uint16_t s_block_retry_ms[APP_C620_MOTOR_NUM];

static void C620_AllOff(void) {
  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    C620_SetCurrent(App_Get_C620(i), 0);
  }
}

/* 模式 1：不看反馈，直接给指定电机固定电流。 */
static void C620_OpenLoop(void) {
  C620_AllOff();

  if (g_c620_test_motor >= APP_C620_MOTOR_NUM) {
    return;
  }

  C620_SetCurrent_A(App_Get_C620(g_c620_test_motor), g_c620_test_current_a);
}

/* 模式 2：只给测试电机目标，其余强制 0。每帧写，方便调试器做阶跃。 */
static void C620_SpeedTestTarget(void) {
  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    g_c620_target_rpm[i] = 0.0f;
  }

  if (g_c620_test_motor < APP_C620_MOTOR_NUM) {
    g_c620_target_rpm[g_c620_test_motor] = g_c620_test_target_rpm;
  }
}

/* 车体速度 → 四轮目标。以后接遥控、小陀螺都只改这里。
 * 轮子编号与电调 ID：rpm[0] LF→ID1，[1] RF→ID2，[2] LR→ID3，[3] RR→ID4。 */
static void C620_UpdateTarget(void) {
  const Chassis_Velocity_t cmd = {g_chassis_vx_mm_s, g_chassis_vy_mm_s,
                                  g_chassis_wz_rad_s};
  Chassis_Wheel_Rpm_t wheel;

  Chassis_Solve(&cmd, &wheel);

  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    g_c620_target_rpm[i] = wheel.rpm[i];
  }
}

/* 目标 → 斜坡 → PID → 电流。不关心目标从哪来。 */
static void C620_SpeedLoop(void) {
  /* 每帧同步参数，调试器改的是全局变量不是 pid->kp */
  const float out_max = g_c620_speed_limit_a * C620_RAW_PER_AMP;
  const float integral_max = out_max * g_c620_speed_integral_ratio;

  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    C620_Instance *motor = App_Get_C620(i);

    if (motor == NULL) {
      continue;
    }

    speed_pid[i].kp = g_c620_speed_kp;
    speed_pid[i].ki = g_c620_speed_ki;
    speed_pid[i].kd = g_c620_speed_kd;
    speed_pid[i].kf = g_c620_speed_kf;
    speed_pid[i].out_max = out_max;
    speed_pid[i].integral_max = integral_max;

    s_slope[i].inc = g_c620_ramp_inc;
    s_slope[i].dec = g_c620_ramp_dec;

    /* 掉线或故障：清干净再断输出，否则恢复瞬间带着积压的积分猛冲 */
    if (!C620_IsOnline(motor) || C620_HasFault(motor)) {
      Pid_Reset(&speed_pid[i]);
      Slope_Reset(&s_slope[i], 0.0f);
      C620_SetCurrent(motor, 0);
      g_c620_pid_err[i] = 0.0f;
      g_c620_pid_out[i] = 0.0f;
      g_c620_pid_integral[i] = 0.0f;
      g_c620_target_ramped[i] = 0.0f;
      g_c620_blocked[i] = 0;
      s_block_retry_ms[i] = 0;
      continue;
    }

    /* real 传当前转速，让斜坡知道实际跑到哪了，避免倒拉 */
    const float ramped = Slope_Update(&s_slope[i], g_c620_target_rpm[i],
                                      motor->measure.speed_rpm_out);

    const float out = Pid_Control(&speed_pid[i], ramped,
                                  motor->measure.speed_rpm_out, C620_CTRL_DT);

    C620_SetCurrent(motor, (int16_t)out);

    g_c620_pid_err[i] = speed_pid[i].err;
    g_c620_pid_out[i] = out;
    g_c620_pid_integral[i] = speed_pid[i].integral;
    g_c620_target_ramped[i] = ramped;
    g_c620_blocked[i] = speed_pid[i].blocked;

    if (speed_pid[i].blocked) {
      s_block_retry_ms[i] += (uint16_t)(C620_CTRL_DT * 1000.0f);
      if (s_block_retry_ms[i] >= C620_BLOCK_RETRY_MS) {
        Pid_Reset(&speed_pid[i]);
        s_block_retry_ms[i] = 0;
      }
    } else {
      s_block_retry_ms[i] = 0;
    }
  }
}

static void C620_Publish(void) {
  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    const C620_Instance *motor = App_Get_C620(i);

    if (motor == NULL) {
      continue;
    }

    g_c620_angle_deg[i] = motor->measure.angle_deg;
    g_c620_speed_rpm[i] = motor->measure.speed_rpm_out;
    g_c620_current_a[i] = motor->measure.current_a;
    g_c620_online[i] = C620_IsOnline(motor) ? 1 : 0;
    g_c620_error_code[i] = motor->measure.error_code;
  }
}

/* 每 1 ms：按模式算电流 → 发送 → 刷镜像。
 * 用 vTaskDelayUntil 而不是 vTaskDelay：前者补偿本帧耗时，周期严格 1 ms。 */
void C620_Task(void *argument) {
  (void)argument;

  for (uint8_t i = 0; i < APP_C620_MOTOR_NUM; i++) {
    const float out_max = g_c620_speed_limit_a * C620_RAW_PER_AMP;
    Pid_Init(&speed_pid[i], g_c620_speed_kp, g_c620_speed_ki, g_c620_speed_kd,
             g_c620_speed_kf, out_max,
             out_max * g_c620_speed_integral_ratio);
    Slope_Init(&s_slope[i], g_c620_ramp_inc, g_c620_ramp_dec, 0.0f);
    s_block_retry_ms[i] = 0;
  }

  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    switch (g_c620_mode) {
      case C620_MODE_OPEN_LOOP:
        C620_OpenLoop();
        break;

      case C620_MODE_KINEMATICS:
        /* 只验公式，电流保持 0，轮子不转 */
        C620_UpdateTarget();
        C620_AllOff();
        break;

      case C620_MODE_SPEED:
        C620_SpeedTestTarget();
        C620_SpeedLoop();
        break;

      case C620_MODE_NORMAL:
      default:
        C620_UpdateTarget();
        C620_SpeedLoop();
        break;
    }

    /* 掉线/故障的电机会被 C620_SendAll 内部强制写 0 */
    if (C620_SendAll(&hfdcan1) != HAL_OK) {
      g_c620_send_fail++;
    }

    C620_Publish();

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(C620_TASK_PERIOD_MS));
  }
}
