#include "att_output.h"

#include <math.h>
#include <string.h>

#include "app_debug.h"

#define ATT_RAD_TO_DEG 57.29578f

/* 上电等滤波器收敛且静止，采 200 帧平均作零位；之后静止时用 10s 常数
 * 追踪 roll/pitch 零位吃温漂。yaw 没有绝对参考，只锁一次绝不追踪，
 * 否则真实转动会被当漂移吃掉。 */
#define ATT_ZERO_WARMUP_FRAMES 3000 /* ≈2.5 s @1.2 kHz */
#define ATT_ZERO_CALIB_SAMPLES 200
#define ATT_ZERO_TRACK_TAU 10.0f

/* 折到 ±180。用 fmodf 不用 while：yaw_unwrapped 会无限累积，
 * 转一小时上百万度，while 每帧循环几千次会把任务卡死。 */
static float Att_Wrap180(float deg) {
  deg = fmodf(deg + 180.0f, 360.0f);
  if (deg < 0.0f) {
    deg += 360.0f;
  }
  return deg - 180.0f;
}

/* yaw 展开：atan2 结果恒在 ±180，连续转圈会从 +180 跳到 -180。
 * 与上一帧比，差超 180° 就补一个 ±360。 */
static void Att_Unwrap_Yaw(Att_Runtime *rt, float yaw_deg) {
  if (rt->yaw_started == 0) {
    rt->yaw_prev = yaw_deg;
    rt->yaw_unwrapped = yaw_deg;
    rt->yaw_started = 1;
    return;
  }

  float d = yaw_deg - rt->yaw_prev;

  if (d > 180.0f) {
    d -= 360.0f;
  } else if (d < -180.0f) {
    d += 360.0f;
  }

  rt->yaw_unwrapped += d;
  rt->yaw_prev = yaw_deg;
}

/* 零位标定 + 三轴角度输出。标定期间一律输出 0，
 * 否则完成瞬间会从"原始角度"跳到 0，看着像数据突变。 */
static void Att_Zero_Update(Att_Runtime *rt, float roll_deg, float pitch_deg,
                            uint8_t rest, float dt) {
  if (g_att_zero_ready == 0) {
    /* 三轴在同一窗口一起采、一起平均，所以是同时标定完成 */
    if (rt->frame_count >= ATT_ZERO_WARMUP_FRAMES && rest != 0) {
      rt->zero_sum[0] += roll_deg;
      rt->zero_sum[1] += pitch_deg;
      rt->zero_sum[2] += rt->yaw_unwrapped; /* 用展开值，避免标定时卡在 ±180 */
      rt->zero_count++;

      if (rt->zero_count >= ATT_ZERO_CALIB_SAMPLES) {
        const float inv = (float)rt->zero_count;

        g_att_roll_zero_deg = rt->zero_sum[0] / inv;
        g_att_pitch_zero_deg = rt->zero_sum[1] / inv;
        g_att_yaw_zero_deg = rt->zero_sum[2] / inv;
        g_att_zero_progress = 100;
        g_att_zero_ready = 1;
      } else {
        g_att_zero_progress =
            (uint8_t)((rt->zero_count * 100) / ATT_ZERO_CALIB_SAMPLES);
      }
    } else {
      /* 中途动了就整段作废重来，宁可多等也不能把晃动采进零位 */
      rt->zero_count = 0;
      rt->zero_sum[0] = 0.0f;
      rt->zero_sum[1] = 0.0f;
      rt->zero_sum[2] = 0.0f;
      g_att_zero_progress = 0;
    }

    g_att_roll_deg = 0.0f;
    g_att_pitch_deg = 0.0f;
    g_att_yaw_deg = 0.0f;
    g_att_yaw_total_deg = 0.0f;
    return;
  }

  if (rest != 0) {
    /* 静止时追踪 roll/pitch 零位（吃加计零偏/温漂）。yaw 不追踪。 */
    const float alpha = dt / (ATT_ZERO_TRACK_TAU + dt);

    g_att_roll_zero_deg += alpha * (roll_deg - g_att_roll_zero_deg);
    g_att_pitch_zero_deg += alpha * (pitch_deg - g_att_pitch_zero_deg);
  }

  g_att_roll_deg = roll_deg - g_att_roll_zero_deg;
  g_att_pitch_deg = pitch_deg - g_att_pitch_zero_deg;

  /* yaw_total 是连续值（转几圈就是几千度）；
   * yaw_deg 折到 ±180（RM 标准），折返点离目标最远，控制上最不敏感。
   * 算误差仍要再 wrap 一次：err = wrap180(target - g_att_yaw_deg) */
  g_att_yaw_total_deg = rt->yaw_unwrapped - g_att_yaw_zero_deg;
  g_att_yaw_deg = Att_Wrap180(g_att_yaw_total_deg);
}

void Att_Output_Init(Att_Runtime *rt) {
  if (rt == NULL) {
    return;
  }
  memset(rt, 0, sizeof(Att_Runtime)); /* 任务被删了重建时也保证状态干净 */
}

void Att_Output_Update(Att_Runtime *rt, float roll_deg, float pitch_deg,
                       float yaw_deg, uint8_t rest, float dt) {
  if (rt == NULL) {
    return;
  }

  rt->frame_count++;
  Att_Unwrap_Yaw(rt, yaw_deg);
  Att_Zero_Update(rt, roll_deg, pitch_deg, rest, dt);
}

void Att_Output_Publish(const Attitude_VQF *att, const float gyro[3]) {
  if (att == NULL || gyro == NULL) {
    return;
  }

  float q[4];
  float bias[3];

  Attitude_Get_Quaternion(att, q);
  g_att_q[0] = q[0];
  g_att_q[1] = q[1];
  g_att_q[2] = q[2];
  g_att_q[3] = q[3];

  g_yaw_rate_deg_s = gyro[2] * ATT_RAD_TO_DEG;

  /* 诊断量，正常恒为 0。dt_clamped 一涨就说明主循环被阻塞过，
   * 那段时间的角度真丢了 —— 排查"转 90° 只读到 85°"先看这三个。 */
  g_att_dt_clamped = att->dt_clamped_count;
  g_att_dt_lost_s = att->dt_lost_time;
  g_att_rejected = att->rejected_count;

  Attitude_Get_Bias(att, bias);
  g_att_bias[0] = bias[0];
  g_att_bias[1] = bias[1];
  g_att_bias[2] = bias[2];
}
