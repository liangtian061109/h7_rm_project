#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdint.h>

/* VQF 姿态解算（Daniel Laidig, TU Berlin, MIT），C 版移植。
 *
 *   1. 陀螺仪减掉在线零偏估计，指数映射积分成 q_gyro（机体 → 地理系）
 *   2. 加速度计经 q_gyro 转到地理系，低通后乘 q_accel 得到残余倾角，
 *      构造最小旋转四元数修正 q_accel
 *   3. q = normalize(q_accel * q_gyro)
 *   4. 在线零偏估计：静止时用低通角速度直接观测，运动时用残余倾角观测
 *      （竖直轴几乎不观测）
 *
 * 第 2 步必须先乘 q_accel 再算修正量，否则 q_accel 没有反馈、
 * 会一直累积旋转，表现为角度乱跳。 */

typedef struct {
  /* -------- 可调参数（建议先改这里，再调 Attitude_Reset） -------- */
  float tau_accel;                 /* 重力方向修正时间常数 s，默认 3.0 */
  float rest_filter_tau;           /* 静止检测低通时间常数 s，默认 0.5 */
  float bias_sigma_init_deg_s;     /* 零偏初值标准差 deg/s，默认 0.5 */
  float bias_forgetting_time;      /* 零偏协方差遗忘时间 s，默认 100 */
  float bias_clip_deg_s;           /* 零偏限幅 deg/s，默认 2.0 */
  float bias_sigma_motion_deg_s;   /* 运动时零偏观测标准差 deg/s，默认 0.1 */
  float bias_sigma_rest_deg_s;     /* 静止时零偏观测标准差 deg/s，默认 0.03 */
  float rest_min_time;             /* 判定静止的最短时间 s，默认 1.5 */
  float rest_threshold_gyro_deg_s; /* 静止检测陀螺残差门限 deg/s，默认 2.0 */
  float rest_threshold_accel;      /* 静止检测加计残差门限 m/s^2，默认 0.5 */
  uint8_t motion_bias_enable;      /* EKF 的运动分支，默认 0（实测有害） */
  uint8_t rest_bias_enable;        /* 总开关：静止检测 + ZARU + EKF 静止分支，必须为 1 */
  uint8_t bias_ekf_enable;         /* 3×3 EKF 总开关，默认 0；关掉后零偏只由 ZARU 负责 */

  /* -------- 内部状态 -------- */
  float dt;      /* 最近一帧的采样周期 s */

  /* 一阶低通串联实现：每个通道两个状态（两级），比二阶 Butterworth 的
     双极点结构稳定得多（后者的极点在 z≈1 附近，浮点递推会发散）。 */
  double accel_lpf_state[6];
  double rest_gyro_lpf_state[6];
  double rest_accel_lpf_state[6];

  float covariance_init;
  float process_noise;
  float motion_noise;
  float vertical_noise;
  float rest_noise;

  uint8_t initialized; /* 0 = 还没用加速度计初始化 */
  float q_gyro[4];     /* 陀螺积分四元数：机体 -> 地理系 */
  float q_accel[4];    /* 加速度修正四元数 */
  float q[4];          /* 6D 姿态：机体 -> 地理系 */

  float accel_earth_lpf[3];
  float last_accel_correction_rate;

  float bias[3];            /* 在线零偏估计 rad/s */
  float covariance[3][3];
  double motion_rotation_lpf_state[18];
  double motion_bias_lpf_state[4];

  uint8_t rest_detected;
  float rest_time;
  float rest_gyro_sq_dev;
  float rest_accel_sq_dev;
  float rest_last_gyro_lpf[3];
  float rest_last_accel_lpf[3];

  float last_corrected_gyro[3];

  /* -------- 调试统计 -------- */
  float last_dt;         /* 最近一帧实际使用的 dt */
  uint32_t rejected_count; /* 被丢弃的无效帧数（NaN / 超量程） */
  /* dt 被钳到上限的次数，以及因此"丢掉"的时间（秒）。
     1.6 kHz 下一帧 625 µs，一旦某帧超过 ATT_DT_MAX(10 ms)，多出来的时间就被截掉，
     那段角速度对应的角度永久丢失 —— 表现为"转 90° 只读到 85°"。
     正常情况下这两个值应该一直是 0；只要在涨，就说明主循环被阻塞过
     （最常见的原因：调试器读内存时短暂停核、任务被更高优先级抢占）。 */
  uint32_t dt_clamped_count;
  float dt_lost_time;
} Attitude_VQF;

void Attitude_Init(Attitude_VQF *att);
void Attitude_Reset(Attitude_VQF *att);

/**
 * @brief 喂一帧 IMU 数据并更新姿态。
 * @param gyro  角速度 rad/s（机体系，已减掉 BMI088 模块的固定零偏）
 * @param accel 加速度 m/s²（机体系，静止时指向天）
 * @param dt    距上一帧的时间（秒），用 Attitude_Dt_Get() 取
 */
void Attitude_Update(Attitude_VQF *att, const float gyro[3],
                     const float accel[3], float dt);

/** @brief 欧拉角（弧度）。roll 绕 x、pitch 绕 y、yaw 绕 z；pitch 在 ±90° 有万向锁 */
void Attitude_Get_Euler(const Attitude_VQF *att, float *roll, float *pitch,
                        float *yaw);

/** @brief 取四元数 q = [w, x, y, z]（机体 -> 地理系）。控制用四元数，不要用欧拉角 */
void Attitude_Get_Quaternion(const Attitude_VQF *att, float q[4]);

/** @brief 取在线零偏估计（rad/s），可用来回填 BMI088 或看估计效果 */
void Attitude_Get_Bias(const Attitude_VQF *att, float bias[3]);

/** @brief 设置零偏初值（rad/s）。sigma_deg_s <= 0 时保持当前协方差 */
void Attitude_Set_Bias_Estimate(Attitude_VQF *att, const float bias[3],
                                float sigma_deg_s);

/** @brief 1 = 当前判定为静止 */
uint8_t Attitude_Get_Rest_Detected(const Attitude_VQF *att);

/* dt 计时：1.6 kHz 一帧只有 625 µs，HAL_GetTick 的 1 ms 精度不够 */
void Attitude_Dwt_Init(void);
float Attitude_Dt_Get(void);

#endif
