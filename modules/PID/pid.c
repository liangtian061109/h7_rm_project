#include "pid.h"

#include <math.h>

void Pid_Init(Pid_Instance *pid, float kp, float ki, float kd, float kf,
              float out_max, float integral_max) {
  if (pid == NULL) {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->kf = kf;
  pid->out_max = out_max;
  pid->integral_max = integral_max;

  pid->integral = 0.0f;
  pid->err = 0.0f;
  pid->last_err = 0.0f;

  pid->deadband = 1.0f;

  /* 0.4s 要比正常加速时间（本工程约 0.28s）长，否则起步就误判 */
  pid->block_time = 0.0f;
  pid->block_time_limit = 0.4f;
  pid->block_target_min = 20.0f;
  pid->block_speed_max = 10.0f;
  pid->blocked = 0;
}

void Pid_Reset(Pid_Instance *pid) {
  if (pid == NULL) {
    return;
  }

  pid->integral = 0.0f;
  pid->err = 0.0f;
  pid->last_err = 0.0f;

  /* 不清堵转标志的话，判过一次就永远返回 0，电机再也不转 */
  pid->block_time = 0.0f;
  pid->blocked = 0;
}

float Pid_Control(Pid_Instance *pid, float target, float actual, float dt) {
  if (pid == NULL || dt <= 0.0f) {
    return 0.0f;
  }

  pid->err = target - actual;

  /* 停机死区：不清积分的话，下次启动会带着残留先冲一下 */
  if (target == 0.0f && fabsf(actual) < pid->deadband) {
    pid->integral = 0.0f;
    pid->last_err = pid->err; /* 让 D 从 0 起步，退出死区不冲击 */
    pid->block_time = 0.0f;
    pid->blocked = 0;
    return 0.0f;
  }

  const float ff_out = pid->kf * target;
  const float p_out = pid->kp * pid->err;
  const float d_out = pid->kd * (pid->err - pid->last_err) / dt;
  const float out_unsat = p_out + pid->integral + d_out + ff_out;

  /* 抗积分饱和：输出已超限，且误差还在同方向推 → 跳过本次积分。
   * 不冻结的话积分会攒到远超实际需要，误差反向时退不下来，
   * 表现就是冲过头、卡在高转速回不来。 */
  const int saturated =
      (out_unsat > pid->out_max) || (out_unsat < -pid->out_max);
  const int pushing = (out_unsat * pid->err) > 0.0f;

  if (!(saturated && pushing)) {
    pid->integral += pid->ki * pid->err * dt;
  }

  if (pid->integral > pid->integral_max) {
    pid->integral = pid->integral_max;
  } else if (pid->integral < -pid->integral_max) {
    pid->integral = -pid->integral_max;
  }

  float out = pid->kp * pid->err + pid->integral + d_out + ff_out;

  if (out > pid->out_max) {
    out = pid->out_max;
  } else if (out < -pid->out_max) {
    out = -pid->out_max;
  }

  pid->last_err = pid->err;

  /* 堵转：四个条件全满足才累加时间，任一不满足就清零并解除，卡阻消失能自动恢复 */
  const int target_demand = fabsf(target) > pid->block_target_min;
  const int speed_low = fabsf(actual) < pid->block_speed_max;
  const int output_saturated = fabsf(out) >= pid->out_max * 0.99f;

  if (target_demand && speed_low && output_saturated) {
    pid->block_time += dt;
  } else {
    pid->block_time = 0.0f;
    pid->blocked = 0;
  }

  if (pid->block_time > pid->block_time_limit) {
    pid->blocked = 1;
    pid->integral = 0.0f;
    return 0.0f;
  }

  return out;
}
