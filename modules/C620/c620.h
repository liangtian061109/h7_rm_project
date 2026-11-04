#ifndef C620_H
#define C620_H

#include "bsp_can.h"
#include <stdbool.h>
#include <stdint.h>

/* 大疆 C620 电调（配 M3508）。经典帧，8 字节，标准 ID。
 *
 * 接收（电调 → 单片机）：0x201~0x208 对应 ID1~ID8
 *   DATA[0..1] 机械角 0~8191（转子侧，不是输出轴）
 *   DATA[2..3] 转速 rpm（转子侧，int16，正负表示方向）
 *   DATA[4..5] 实际转矩电流（int16，±16384 对应 ±20A）
 *   DATA[6]    温度 ℃
 *   DATA[7]    错误码
 * 发送（单片机 → 电调）：0x200 控制 ID1~ID4，0x1FF 控制 ID5~ID8
 *   每 2 字节一个电机的电流指令，大端序 */

#define C620_MAX_MOTORS 8
#define C620_CURRENT_MAX 16384   /* 原始值上限 ↔ 20A */
#define C620_CURRENT_MAX_A 20.0f
#define C620_ECD_RANGE 8192.0f   /* 机械角满量程（转子侧一圈） */
#define C620_ECD_COUNT 8192      /* 整数版，镜像运算用（8192 = 2^13） */
#define C620_REDUCTION_RATIO (3591.0f / 187.0f) /* M3508 减速比 ≈ 19.2 */
#define C620_OFFLINE_TIMEOUT_MS 100
#define C620_SPEED_LPF_ALPHA 0.3f   /* 转速低通，1.0 表示不滤波 */
#define C620_CURRENT_LPF_ALPHA 0.3f

/* 原始值在上、换算值在下，调试时一眼分清哪层出错 */
typedef struct {
  uint16_t ecd;          /* 机械角 0~8191（已按 reversed 镜像） */
  int16_t speed_rpm;     /* 转子侧 rpm（已按 reversed 取反） */
  int16_t given_current; /* 实际转矩电流（已取反） */
  uint8_t temperature;
  uint8_t error_code;

  int16_t last_ecd;      /* 上一帧机械角，多圈判据用 */
  int32_t total_round;   /* 转子累计圈数 */
  float angle_deg;       /* 输出轴单圈 0~360 */
  float total_angle_deg; /* 输出轴多圈，连续 */
  float speed_rpm_out;   /* 输出轴转速，低通后 */
  float current_a;       /* 实际电流，低通后 */
} C620_Measure_s;

typedef struct C620_S {
  uint8_t can_id; /* 电调 ID 1~8 */
  FDCAN_HandleTypeDef *handle;

  bool reversed; /* true = 反向安装，指令和反馈统一翻到逻辑方向 */
  C620_Measure_s measure;
  int16_t current_cmd;

  CAN_Instance *can;
  uint32_t rx_count;
  uint32_t last_rx_tick;
} C620_Instance;

/* 必须在 CAN_Init() 之前调用：内部要配过滤器，而过滤器只能写进未启动的 Message RAM。
 * reversed 为 true 时，电流指令和反馈（角度/转速/电流）都会翻到统一逻辑方向，
 * 上层不用再关心这个电机装反了没有。失败返回 NULL。 */
C620_Instance *C620_Init(uint8_t can_id, FDCAN_HandleTypeDef *handle,
                         bool reversed);

/* 原始值 -16384~16384，内部限幅并按 reversed 取反 */
void C620_SetCurrent(C620_Instance *motor, int16_t current_raw);

/* 安培 -20~20，内部换算成原始值 */
void C620_SetCurrent_A(C620_Instance *motor, float current_a);

/* 把这条总线上所有电机的电流指令打包发出，每 1 ms 调一次。
 * 掉线或故障的电机会被强制写 0 —— 发旧电流会让轮子继续转。 */
HAL_StatusTypeDef C620_SendAll(FDCAN_HandleTypeDef *handle);

/* 100 ms 内收到过反馈才算在线 */
bool C620_IsOnline(const C620_Instance *motor);

/* 有硬件级故障码（三相断线/编码器丢失/过温/校准失败） */
bool C620_HasFault(const C620_Instance *motor);

#endif
