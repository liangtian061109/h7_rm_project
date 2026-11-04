#ifndef PID_H
#define PID_H

#include <stdint.h>

/* 位置式 PID：out = kp*err + ki*∫err·dt + kd*d(err)/dt + kf*target
 *
 * integral 字段存的是**已乘过 ki 的积分项**（Iout），不是原始误差累加。
 * 好处：ki=0 时不会攒垃圾值，运行时改 ki 也不会让积分突变。
 * 代价：integral_max 是"Iout 上限"，按输出量纲估。 */

typedef struct {
  float kp;
  float ki;
  float kd;

  /* 前馈：out += kf * target。标定值 = 稳态电流 / 目标转速。 */
  float kf;

  float out_max;      /* 输出限幅，正负对称 */
  float integral;     /* 积分项（已乘 ki） */
  float integral_max; /* 积分项限幅 */

  float err;      /* 当前误差，调试时能直接看 */
  float last_err; /* 上次误差，微分用 */

  /* 死区：目标 0 且 |实际| < deadband 时输出 0。
   * 不设的话反馈噪声会让 P 项一直吐小电流，电机嗡嗡响还发热。 */
  float deadband;

  /* 堵转检测：目标有需求 + 转速极低 + 输出饱和 + 持续够久。
   * "转速极低"这条把"参数不够追不上"和"真转不动"分开。
   * 判定后输出强制为 0，条件会一直成立，必须由上层调 Pid_Reset 清除。 */
  float block_time;       /* 已持续满足条件的时间，秒 */
  float block_time_limit; /* 判定阈值，秒。要大于正常加速时间 */
  float block_target_min; /* 目标门槛，rpm */
  float block_speed_max;  /* 转速门槛，rpm */
  uint8_t blocked;        /* 1 = 判定堵转 */
} Pid_Instance;

/* kf 填 0 表示不用前馈；integral_max 是 Iout 上限，建议 out_max 的 1/4~1/2。 */
void Pid_Init(Pid_Instance *pid, float kp, float ki, float kd, float kf,
              float out_max, float integral_max);

/* dt 是距上次计算的间隔（秒）。本工程任务周期固定，传 0.001f。 */
float Pid_Control(Pid_Instance *pid, float target, float actual, float dt);

/* 切模式、掉线恢复、任务重启时必须调，否则残留积分会让电机缓一下才停。 */
void Pid_Reset(Pid_Instance *pid);

#endif
