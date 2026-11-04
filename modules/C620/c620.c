#include "c620.h"

#include <string.h> /* memset */

static C620_Instance C620_Instances[C620_MAX_MOTORS];
static uint8_t c620_count;

/* 角度归一化到 0~360。 */
static float C620_Wrap360(float deg) {
  while (deg >= 360.0f) {
    deg -= 360.0f;
  }
  while (deg < 0.0f) {
    deg += 360.0f;
  }
  return deg;
}

/* 硬件级故障码：3 三相线未接入 / 4 位置传感器丢数据 / 5 温度异常 / 7 校准失败 /
 * 8 过热。 1、2 只在开机自检出现，偶发一次不算坏，所以不列入停机条件。
 * 手册的码是稀疏的（没有 6），因此逐个比较，不要写成范围判断。 */
static bool C620_Is_Hard_Fault(uint8_t code) {
  return (code == 3 || code == 4 || code == 5 || code == 7 || code == 8);
}

/* 反馈解析：跑在 CAN 中断里，只做拷贝和浮点换算，不能调阻塞接口 */
static void C620_RxHandler(CAN_Instance *can) {
  if (can == NULL || can->id == NULL || can->rx_len < 8) {
    return;
  }

  C620_Instance *motor = (C620_Instance *)can->id;
  C620_Measure_s *receive_measure = &motor->measure;
  const uint8_t *data = can->rx_data;

  /* 反装电机：反馈也翻到逻辑方向，和已取反的电流指令对齐，
   * 上层做速度/位置闭环时目标与反馈同号，误差不会反号导致环发散。 */
  const int32_t sign = motor->reversed ? -1 : 1;

  /* 编码器镜像必须在算 diff 之前做，而且 last_ecd 要存镜像值。
   *
   * 镜像后 0°→0°、1→8191、2→8190，相当于从另一头量角度。
   * 若只镜像 ecd 而 last_ecd 存原始值，diff = 镜像值 - 原始值毫无意义，
   * 每跨一次 0/8191 就被误判成跨圈，总角度跳一整圈。
   * 先镜像再算 diff，ecd / last_ecd / total_round / angle 才在同一坐标系里。
   *
   * 8192 = 2^13，所以镜像等价于 (8192 - x) & 0x1FFF。 */
  const uint16_t ecd_raw = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  uint16_t ecd = ecd_raw;
  if (motor->reversed) {
    ecd = (uint16_t)((C620_ECD_COUNT - ecd_raw) & (C620_ECD_COUNT - 1));
  }

  receive_measure->last_ecd = receive_measure->ecd;
  receive_measure->ecd = ecd;

  const int16_t speed_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  const int16_t current_raw = (int16_t)(((uint16_t)data[4] << 8) | data[5]);

  receive_measure->speed_rpm = (int16_t)(sign * (int32_t)speed_raw);
  receive_measure->given_current = (int16_t)(sign * (int32_t)current_raw);
  receive_measure->temperature = data[6];
  receive_measure->error_code = data[7];

  /* 多圈计数：相邻两帧差值超过半圈（4096）就认为跨过了 0/8191 边界。
   * 必须用 int32_t 相减，uint16_t 相减会按无符号回绕，判据就失效了。 */
  const int32_t diff =
      (int32_t)receive_measure->ecd - (int32_t)receive_measure->last_ecd;
  if (diff > 4096) {
    receive_measure->total_round--;
  } else if (diff < -4096) {
    receive_measure->total_round++;
  }

  /* 机械角是转子侧，除以减速比才是输出轴角度 */
  const float rotor_deg =
      (float)receive_measure->ecd * (360.0f / C620_ECD_RANGE);
  const float rotor_total_deg =
      (float)receive_measure->total_round * 360.0f + rotor_deg;
  receive_measure->total_angle_deg = rotor_total_deg / C620_REDUCTION_RATIO;
  receive_measure->angle_deg = C620_Wrap360(receive_measure->total_angle_deg);

  /* 一阶低通：静置无偏差，起转瞬间略滞后属正常 */
  const float speed_out =
      (float)receive_measure->speed_rpm / C620_REDUCTION_RATIO;
  receive_measure->speed_rpm_out =
      (1.0f - C620_SPEED_LPF_ALPHA) * receive_measure->speed_rpm_out +
      C620_SPEED_LPF_ALPHA * speed_out;

  const float current_a = (float)receive_measure->given_current *
                          (C620_CURRENT_MAX_A / (float)C620_CURRENT_MAX);
  receive_measure->current_a =
      (1.0f - C620_CURRENT_LPF_ALPHA) * receive_measure->current_a +
      C620_CURRENT_LPF_ALPHA * current_a;

  motor->rx_count++;
  motor->last_rx_tick = HAL_GetTick();
}

C620_Instance *C620_Init(uint8_t can_id, FDCAN_HandleTypeDef *handle,
                         bool reversed) {
  /* 1) 参数校验 */
  if (can_id < 1 || can_id > C620_MAX_MOTORS || handle == NULL ||
      handle->Instance == NULL || c620_count >= C620_MAX_MOTORS) {
    return NULL;
  }

  /* 2) 同一总线上 ID 不能重复 */
  for (uint8_t i = 0; i < c620_count; i++) {
    if (C620_Instances[i].handle == handle &&
        C620_Instances[i].can_id == can_id) {
      return NULL;
    }
  }

  /* 3) 占候选槽：先 memset 再填，槽位可能被上次失败的注册用过 */
  const uint8_t index = c620_count;
  C620_Instance *motor = &C620_Instances[index];
  memset(motor, 0, sizeof(C620_Instance));
  motor->can_id = can_id;
  motor->handle = handle;
  motor->reversed = reversed;

  /* 4) 注册接收实例。反馈 ID = 0x200 + 电调 ID */
  CAN_Init_Config_s conf = {
      .handle = handle,
      .rx_id = (uint32_t)(0x200 + can_id),
      .rx_ide = FDCAN_STANDARD_ID,
      .rx_callback = C620_RxHandler,
      .id = motor,
  };

  motor->can = CAN_Register(&conf);
  if (motor->can == NULL) {
    memset(motor, 0, sizeof(C620_Instance));
    return NULL;
  }

  c620_count = (uint8_t)(index + 1);
  return motor;
}

void C620_SetCurrent(C620_Instance *motor, int16_t current_raw) {
  if (motor == NULL) {
    return;
  }

  /* int32_t 中转：直接对 int16_t 取反时 -(-32768) 会溢出（UB） */
  int32_t cmd = current_raw;
  if (cmd > C620_CURRENT_MAX) {
    cmd = C620_CURRENT_MAX;
  }
  if (cmd < -C620_CURRENT_MAX) {
    cmd = -C620_CURRENT_MAX;
  }

  motor->current_cmd = motor->reversed ? (int16_t)(-cmd) : (int16_t)cmd;
}

void C620_SetCurrent_A(C620_Instance *motor, float current_a) {
  if (motor == NULL) {
    return;
  }

  /* 安培 → 原始值，再交给 C620_SetCurrent 统一限幅和取反 */
  float cmd = current_a * ((float)C620_CURRENT_MAX / C620_CURRENT_MAX_A);
  if (cmd > (float)C620_CURRENT_MAX) {
    cmd = (float)C620_CURRENT_MAX;
  } else if (cmd < -(float)C620_CURRENT_MAX) {
    cmd = -(float)C620_CURRENT_MAX;
  }
  C620_SetCurrent(motor, (int16_t)cmd);
}

HAL_StatusTypeDef C620_SendAll(FDCAN_HandleTypeDef *handle) {
  if (handle == NULL || handle->Instance == NULL) {
    return HAL_ERROR;
  }

  uint8_t frame[2][8] = {{0}}; /* [0]=0x200 组(ID1~4), [1]=0x1FF 组(ID5~8) */
  bool used[2] = {false, false};

  for (uint8_t i = 0; i < c620_count; i++) {
    C620_Instance *motor = &C620_Instances[i];
    if (motor->handle != handle) {
      continue; /* 不是这条总线上的电机 */
    }

    const uint8_t g = (motor->can_id <= 4) ? 0 : 1;
    const uint8_t slot = (uint8_t)((motor->can_id - 1) % 4);
    /* 掉线或硬件故障都强制输出 0：发上一帧的旧电流会让轮子继续转 */
    const bool output_enable = C620_IsOnline(motor) && !C620_HasFault(motor);
    const int16_t cmd = output_enable ? motor->current_cmd : 0;

    frame[g][slot * 2] = (uint8_t)((uint16_t)cmd >> 8);
    frame[g][slot * 2 + 1] = (uint8_t)((uint16_t)cmd & 0xFF);
    used[g] = true;
  }

  HAL_StatusTypeDef status = HAL_OK;
  if (used[0]) {
    const HAL_StatusTypeDef result =
        CAN_Transmit(handle, 0x200, FDCAN_STANDARD_ID, frame[0], 8);
    if (result != HAL_OK) {
      status = result;
    }
  }
  if (used[1]) {
    const HAL_StatusTypeDef result =
        CAN_Transmit(handle, 0x1FF, FDCAN_STANDARD_ID, frame[1], 8);
    if (result != HAL_OK) {
      status = result;
    }
  }
  return status;
}

bool C620_IsOnline(const C620_Instance *motor) {
  if (motor == NULL || motor->rx_count == 0) {
    return false; /* 一次反馈都没收到过，不算在线 */
  }
  return (HAL_GetTick() - motor->last_rx_tick) <= C620_OFFLINE_TIMEOUT_MS;
}

bool C620_HasFault(const C620_Instance *motor) {
  if (motor == NULL) {
    return true; /* 传空指针按故障处理，调用方不会误发电流 */
  }
  return C620_Is_Hard_Fault(motor->measure.error_code);
}
