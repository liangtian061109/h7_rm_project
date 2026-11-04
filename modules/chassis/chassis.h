#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>

/* 麦轮运动学。右手系，俯视：+x 车头，+y 车身左侧，+wz 逆时针。
 *
 *   LF ─────── RF     LF = Left Front   左前
 *    │    ↑x   │      RF = Right Front  右前
 *    │    │    │      LR = Left Rear    左后
 *   LR ─────── RR     RR = Right Rear   右后
 *
 * 只做数学：不碰 CAN、不碰电机、不依赖 FreeRTOS。 */

typedef enum {
  CHASSIS_WHEEL_LF = 0,
  CHASSIS_WHEEL_RF = 1,
  CHASSIS_WHEEL_LR = 2,
  CHASSIS_WHEEL_RR = 3,
  CHASSIS_WHEEL_NUM = 4
} Chassis_Wheel_e;

typedef struct {
  float vx_mm_s;  /* + 车头方向 */
  float vy_mm_s;  /* + 车身左侧 */
  float wz_rad_s; /* + 逆时针 */
} Chassis_Velocity_t;

typedef struct {
  float rpm[CHASSIS_WHEEL_NUM]; /* 输出轴 rpm */
} Chassis_Wheel_Rpm_t;

/* 几何参数做成变量，调试器里能实时改，换车架不用重编译 */
extern float g_chassis_wheel_diameter_mm; /* 轮子直径，mm */
extern float g_chassis_wheel_base_mm;     /* 轴距，mm */
extern float g_chassis_track_width_mm;    /* 轮距，mm */

/* 速度指令，正常运行时由遥控器/上位机写 */
extern float g_chassis_vx_mm_s;
extern float g_chassis_vy_mm_s;
extern float g_chassis_wz_rad_s;

float Chassis_Mm_S_To_Rpm(float mm_s);
float Chassis_Rpm_To_Mm_S(float rpm);

/* 逆运动学：车体速度 → 四轮目标转速。
 * 辊子和轮轴成 45°，每项乘 1/√2；自转项用轮子到中心的距离 d，
 * 不是 (轴距+轮距)/2 —— 那个偏大 41%，自转会转太快。 */
void Chassis_Solve(const Chassis_Velocity_t *cmd, Chassis_Wheel_Rpm_t *out);

#endif
