#ifndef ATT_OUTPUT_H
#define ATT_OUTPUT_H

#include <stdint.h>

#include "attitude.h"

/* 姿态后处理：把欧拉角变成能直接给控制用的量。
 *   1. yaw 展开 —— atan2 结果恒在 ±180，连续转圈会跳
 *   2. 零位标定 —— 上电静止采 200 帧取平均，之后静止时追踪 roll/pitch 温漂
 *   3. 输出发布 —— 四元数、诊断量、零偏
 *
 * 和 BMI088 采样无关，换任何 IMU 都一样，只依赖 attitude.h 和 app_debug.h。 */

/* 跨帧状态。只有一个姿态任务，实例由调用方持有。 */
typedef struct {
  uint32_t frame_count; /* 上电以来总帧数，暖机判断用 */
  uint32_t zero_count;  /* 零位标定已采帧数 */
  float zero_sum[3];    /* [0]=roll [1]=pitch [2]=yaw */
  float yaw_prev;       /* 上一帧 yaw，折返检测用 */
  float yaw_unwrapped;  /* 展开后的连续 yaw，可超过 ±180 */
  uint8_t yaw_started;  /* 0 = 还没建立 yaw_prev */
} Att_Runtime;

void Att_Output_Init(Att_Runtime *rt);

/* 每帧调用。yaw_deg 传 ±180 范围内未展开的值，rest 为 1 表示判定静止。 */
void Att_Output_Update(Att_Runtime *rt, float roll_deg, float pitch_deg,
                       float yaw_deg, uint8_t rest, float dt);

/* 发布四元数、角速度、诊断计数、零偏。只给调试器看，不影响控制。 */
void Att_Output_Publish(const Attitude_VQF *att, const float gyro[3]);

#endif
