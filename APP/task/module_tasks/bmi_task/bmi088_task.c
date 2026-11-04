#include "bmi088_task.h"

#include <limits.h>
#include <stdint.h>

#include "app_debug.h"
#include "app_init.h"
#include "att_output.h"
#include "bmi088.h"

/* 栈大小和优先级在 app_task.c 的任务表里，这里只管任务怎么跑。 */

#define BMI088_WAIT_TIMEOUT_MS 5

#define ATT_RAD_TO_DEG 57.29578f

/* 全局而非 static：调试器里可以直接 watch（声明在 bmi088_task.h） */
TaskHandle_t g_bmi088_task_handle = NULL;
Attitude_VQF g_attitude;

/* 姿态后处理的跨帧状态。只有一个任务实例，文件级 static 即可。 */
static Att_Runtime s_rt;

/* 一帧完整处理：融合 → 取欧拉角 → 后处理 → 发布诊断量。
 *
 * dt 必须用 DWT 测：1.6 kHz 一帧 625 µs，1 ms 的 tick 精度不够。
 * 即使这帧不解算也要取一次，否则"上一帧时间"不更新，下一帧的 dt 会翻倍。 */
static void BMI088_Step(BMI088_Instance *bmi088) {
  const float dt = Attitude_Dt_Get();

  if (bmi088->gyro_calibrated == 0) {
    return; /* 零偏没标定，积分出来的角度没有意义 */
  }

  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;

  /* data.gyro 在 BMI088_Read 里已减过固定零偏，别再塞 gyro_offset 减两次。
   * 在线零偏估计从 0 开始，只修温漂和残余。 */
  Attitude_Update(&g_attitude, bmi088->data.gyro, bmi088->data.accel, dt);
  Attitude_Get_Euler(&g_attitude, &roll, &pitch, &yaw);

  const uint8_t rest = Attitude_Get_Rest_Detected(&g_attitude);

  g_att_roll = roll;
  g_att_pitch = pitch;
  g_att_yaw = yaw;
  g_att_rest = rest;

  Att_Output_Update(&s_rt, roll * ATT_RAD_TO_DEG, pitch * ATT_RAD_TO_DEG,
                    yaw * ATT_RAD_TO_DEG, rest, dt);
  Att_Output_Publish(&g_attitude, bmi088->data.gyro);
}

/* 加速度计 INT1 每来一个边沿唤醒一次（1.6 kHz），一次读 accel+gyro+温度，
 * 写进 BMI088_Instance->data，供解算/控制任务读最新帧。 */
void BMI088_Task(void *argument) {
  (void)argument;

  BMI088_Instance *bmi088 = App_Get_BMI088();
  if (bmi088 == NULL) {
    vTaskDelete(NULL); /* 理论上不会发生：设备初始化阶段已注册 */
  }

  /* 零偏标定已在 App_Init_Devices() 里做完，这里只管采样和解算 */
  Att_Output_Init(&s_rt);

  Attitude_Init(&g_attitude); /* 同时打开 DWT 计数器，后面测 dt 用 */
  (void)Attitude_Dt_Get();    /* 第一帧没有"上一帧"，丢弃 */

  for (;;) {
    /* 等数据就绪通知；超时返回 pdFALSE，做一帧兜底采样 */
    uint32_t flags = 0;

    if (xTaskNotifyWait(0, ULONG_MAX, &flags,
                        pdMS_TO_TICKS(BMI088_WAIT_TIMEOUT_MS)) == pdFALSE) {
      bmi088->timeout_count++; /* INT 没来，记一笔但仍读一帧，别让控制环饿死 */
    }

    (void)BMI088_Read(bmi088);
    BMI088_Step(bmi088);
  }
}
