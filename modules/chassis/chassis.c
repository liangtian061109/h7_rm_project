#include "chassis.h"

#include <math.h>

#define CHASSIS_PI 3.14159265f
#define CHASSIS_INV_SQRT2 0.70710678f /* 1/√2 */

/* 实测值：轮径 85mm，轴距 346mm，轮距 346mm */
float g_chassis_wheel_diameter_mm = 85.0f;
float g_chassis_wheel_base_mm = 346.0f;
float g_chassis_track_width_mm = 346.0f;

float g_chassis_vx_mm_s = 0.0f;
float g_chassis_vy_mm_s = 0.0f;
float g_chassis_wz_rad_s = 0.0f;

float Chassis_Mm_S_To_Rpm(float mm_s) {
  /* rpm = (mm/s × 60) / (π·D) */
  return (mm_s * 60.0f) / (CHASSIS_PI * g_chassis_wheel_diameter_mm);
}

float Chassis_Rpm_To_Mm_S(float rpm) {
  return (rpm * CHASSIS_PI * g_chassis_wheel_diameter_mm) / 60.0f;
}

void Chassis_Solve(const Chassis_Velocity_t *cmd, Chassis_Wheel_Rpm_t *out) {
  if (cmd == NULL || out == NULL) {
    return;
  }

  /* 几何参数可能被实时改，所以每帧算。M7 有 FPU，1 kHz 下可忽略。 */
  const float d = sqrtf(
      (g_chassis_track_width_mm / 2.0f) * (g_chassis_track_width_mm / 2.0f) +
      (g_chassis_wheel_base_mm / 2.0f) * (g_chassis_wheel_base_mm / 2.0f));

  const float rot = cmd->wz_rad_s * d;
  const float v_LF = (cmd->vx_mm_s - cmd->vy_mm_s - rot) * CHASSIS_INV_SQRT2;
  const float v_RF = (cmd->vx_mm_s + cmd->vy_mm_s + rot) * CHASSIS_INV_SQRT2;
  const float v_LR = (cmd->vx_mm_s + cmd->vy_mm_s - rot) * CHASSIS_INV_SQRT2;
  const float v_RR = (cmd->vx_mm_s - cmd->vy_mm_s + rot) * CHASSIS_INV_SQRT2;

  out->rpm[CHASSIS_WHEEL_LF] = Chassis_Mm_S_To_Rpm(v_LF);
  out->rpm[CHASSIS_WHEEL_RF] = Chassis_Mm_S_To_Rpm(v_RF);
  out->rpm[CHASSIS_WHEEL_LR] = Chassis_Mm_S_To_Rpm(v_LR);
  out->rpm[CHASSIS_WHEEL_RR] = Chassis_Mm_S_To_Rpm(v_RR);
}
