#include "attitude.h"

#include <math.h>
#include <string.h>

#include "main.h" /* DWT / SystemCoreClock */

/* VQF 姿态解算实现。算法流程和参数说明见 attitude.h。
 * 下面注释里的实测数据都是本板子调出来的，改参数前先看一遍。 */

/* ============================ 常量 ============================ */

#define ATT_PI 3.14159265358979323846f
#define ATT_DEG2RAD (ATT_PI / 180.0f)

/* 物理量程保护：BMI088 陀螺 ±2000 dps ≈ 34.9 rad/s；加速度计给足余量 */
#define ATT_GYRO_MAX_RAD_S 35.0f
#define ATT_ACCEL_MIN_G 0.30f
#define ATT_ACCEL_MAX_G 3.00f
#define ATT_GRAVITY 9.80665f

/* dt 合理范围：小于 0.1 ms 或大于 10 ms 都当异常值钳住 */
#define ATT_DT_MIN 0.0001f
#define ATT_DT_MAX 0.01f

/* 运动状态下每帧零偏最大修正量（deg/s），防止观测噪声把零偏推跑 */
#define ATT_MOTION_BIAS_STEP_DEG_S 0.01f

/* ZARU 把零偏往低通角速度上拽的时间常数。它直接决定 yaw 的低频晃动：
 * yaw = ∫(gyro - bias)dt，bias 是 gyro 的低通，(gyro - bias) 相当于高通，
 * 所以 yaw 的低频成分 ≈ tau × gyro 的低频成分，tau 越小慢晃越小。
 * 实测 tau=1.0 时 yaw 有 ±0.03°、约 5s 周期的慢晃；0.4 时降到约 ±0.012°。
 * 可以放心改小：ZARU 前面还有 0.5°/s 的闸，真实慢转不会被吸掉。 */
#define ATT_REST_BIAS_TAU 0.4f

/* 退出静止的低通角速度门限：超过就认为在动（慢速转动也不能算静止） */
#define ATT_REST_EXIT_GYRO_DEG_S 1.0f
/* ZARU 只在低通角速度小于这个值时更新，防止把缓慢转动吸进零偏 */
#define ATT_REST_ZARU_MAX_DEG_S 0.5f

/* 判出静止后还要等这么久才开始 ZARU。
 * 快速转动结束后静止低通要好几秒才衰减干净，而静止判据 1.5s 就成立了。
 * 立刻开始 ZARU 会把残余角速度当零偏吸进去（实测顶到 0.3~0.4°/s），
 * 表现为"转 90° 再转回来差 0.x 度"，转得越快差得越多。
 * 配合下面 0.5°/s 的闸双重保护。 */
#define ATT_REST_ZARU_SETTLE_TIME 0.5f

/* 静止判据做成两级：
 *  - 严重异常（真在动 / 冲击）：计时清零，立刻退出静止态
 *  - 轻微异常（单个采样点毛刺）：只按 ATT_REST_TIME_DECAY 倍扣时间，不清零
 *    否则 1.6 kHz 采样下只要偶尔来一帧坏数据，1.5 s 的静止窗口就永远攒不满。 */
#define ATT_REST_SEVERE_GYRO_DEG_S 50.0f
#define ATT_REST_SEVERE_ACCEL_MPS2 8.0f
#define ATT_REST_TIME_DECAY 3.0f

/* ==========================================================================
 * 一、四元数运算
 * ========================================================================== */

/**
 * @brief 四元数归一化：把长度拉回 1。
 *
 * 浮点累积误差会让长度慢慢偏离 1，不归一化的话旋转矩阵就不再正交，
 * 角度会缓慢失真。另外必须处理 NaN —— 一旦 NaN 混进来会顺着积分污染所有输出。
 */
static void Att_Quat_Normalize(float q[4]) {
  /* 命名对照：qw = q[0]，qx = q[1]，qy = q[2]，qz = q[3] */
  float qw = q[0];
  float qx = q[1];
  float qy = q[2];
  float qz = q[3];
  float norm = sqrtf(qw * qw + qx * qx + qy * qy + qz * qz);

  /* 长度异常（全 0，或已经是 NaN）就退回单位四元数。
   * 注意 !isfinite 不能省：NaN 和任何数比较都返回 false，
   * 光写 norm < 1e-9 抓不到 NaN。 */
  if (norm < 1.0e-9f || !isfinite(norm)) {
    q[0] = 1.0f;
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }

  norm = 1.0f / norm; /* 先取倒数，省三次除法 */
  q[0] = qw * norm;
  q[1] = qx * norm;
  q[2] = qy * norm;
  q[3] = qz * norm;
}

/**
 * @brief 四元数乘法：把两个旋转串起来，out = a * b。
 *
 * 把四元数写成 a = (aw, ax, ay, az) = aw + ax·i + ay·j + az·k，
 * 用 i²=j²=k²=-1、ij=k、ji=-k 这些规则展开 16 项后按输出归并，就是下面四行。
 *
 * 注意：先算到局部变量再写出，这样 out 和 a/b 重叠也安全。
 * （原版直接写 out[0]=...，如果 out==a 就会出错。）
 */
static void Att_Quat_Multiply(const float a[4], const float b[4], float out[4]) {
  const float aw = a[0];
  const float ax = a[1];
  const float ay = a[2];
  const float az = a[3];
  const float bw = b[0];
  const float bx = b[1];
  const float by = b[2];
  const float bz = b[3];

  /* 标量部分：实部×实部 减去 三个"同轴相乘"（i²=j²=k²=-1，所以全是减号） */
  float w = aw * bw - ax * bx - ay * by - az * bz;

  /* 向量部分：实部交叉项（加） + 叉乘项（顺循环加、反循环减） */
  float x = aw * bx + ax * bw + ay * bz - az * by;
  float y = aw * by - ax * bz + ay * bw + az * bx;
  float z = aw * bz + ax * by - ay * bx + az * bw;

  out[0] = w;
  out[1] = x;
  out[2] = y;
  out[3] = z;
}

/**
 * @brief 四元数 → 3×3 旋转矩阵（机体 → 地理系）。
 *
 * 规律：对角线是 1 - 2×(另外两个虚部的平方和)；
 *       非对角线是 2×(两个乘积 ± 另一个乘积)，m[0][1]/m[1][0] 差一个符号。
 */
static void Att_Quat_To_Matrix(const float q[4], float m[3][3]) {
  const float qw = q[0];
  const float qx = q[1];
  const float qy = q[2];
  const float qz = q[3];

  m[0][0] = 1.0f - 2.0f * (qy * qy + qz * qz);
  m[0][1] = 2.0f * (qx * qy - qw * qz);
  m[0][2] = 2.0f * (qx * qz + qw * qy);

  m[1][0] = 2.0f * (qx * qy + qw * qz);
  m[1][1] = 1.0f - 2.0f * (qx * qx + qz * qz);
  m[1][2] = 2.0f * (qy * qz - qw * qx);

  m[2][0] = 2.0f * (qx * qz - qw * qy);
  m[2][1] = 2.0f * (qy * qz + qw * qx);
  m[2][2] = 1.0f - 2.0f * (qx * qx + qy * qy);
}

/**
 * @brief 3×3 矩阵相乘，out = a * b。
 * 先算到 tmp 再拷贝，因为 out 可能和 a/b 是同一块内存。
 */
static void Att_Matrix_Multiply(const float a[3][3], const float b[3][3],
                                float out[3][3]) {
  float tmp[3][3];
  uint32_t row;
  uint32_t col;
  uint32_t k;

  for (row = 0; row < 3; row++) {
    for (col = 0; col < 3; col++) {
      tmp[row][col] = 0.0f;
      for (k = 0; k < 3; k++) {
        tmp[row][col] += a[row][k] * b[k][col];
      }
    }
  }

  memcpy(out, tmp, sizeof(tmp));
}

/**
 * @brief 矩阵乘向量，out = m * v。每一行和向量做点乘。
 * 这是"把向量转到新坐标系"的基本操作。
 */
static void Att_Matrix_Multiply_Vector(const float m[3][3], const float v[3],
                                       float out[3]) {
  uint32_t row;

  for (row = 0; row < 3; row++) {
    out[row] = m[row][0] * v[0] + m[row][1] * v[1] + m[row][2] * v[2];
  }
}

/**
 * @brief 矩阵转置，行列互换。
 * 对旋转矩阵来说，转置就等于求逆（正交矩阵），而且不用做除法。
 */
static void Att_Matrix_Transpose(const float in[3][3], float out[3][3]) {
  uint32_t row;
  uint32_t col;

  for (row = 0; row < 3; row++) {
    for (col = 0; col < 3; col++) {
      out[row][col] = in[col][row];
    }
  }
}

/**
 * @brief 3×3 矩阵求逆。成功返回 1，奇异返回 0。
 *
 * 方法：逆矩阵 = 伴随矩阵 / 行列式。
 * 伴随矩阵的每个元素是"余子式"：划掉该元素所在的行和列，剩下 2×2 的行列式。
 *
 * 全程用 double：协方差矩阵元素小到 1e-5，行列式会到 1e-13，
 * float 只有 7 位有效数字，算到一半精度就没了。
 */
static uint8_t Att_Matrix_Inverse(const float in[3][3], float out[3][3]) {
  /* 矩阵元素命名成 m00..m22，比 a..i 好认 */
  const double m00 = in[0][0], m01 = in[0][1], m02 = in[0][2];
  const double m10 = in[1][0], m11 = in[1][1], m12 = in[1][2];
  const double m20 = in[2][0], m21 = in[2][1], m22 = in[2][2];

  /* 九个余子式 */
  const double cof00 = m11 * m22 - m12 * m21;
  const double cof01 = m02 * m21 - m01 * m22;
  const double cof02 = m01 * m12 - m02 * m11;
  const double cof10 = m12 * m20 - m10 * m22;
  const double cof11 = m00 * m22 - m02 * m20;
  const double cof12 = m02 * m10 - m00 * m12;
  const double cof20 = m10 * m21 - m11 * m20;
  const double cof21 = m01 * m20 - m00 * m21;
  const double cof22 = m00 * m11 - m01 * m10;

  const double determinant = m00 * cof00 + m01 * cof10 + m02 * cof20;

  double scale = 0.0;
  double element;
  double inv_det;
  uint32_t idx;

  /* 奇异判据必须按"元素最大值的立方"归一化。
   * 协方差矩阵的元素本身很小（~1e-5），行列式会小到 1e-13 量级，
   * 如果写 fabs(det) < 1e-6 这种绝对阈值，会永远判成奇异。 */
  for (idx = 0; idx < 9; idx++) {
    element = fabs((double)in[idx / 3][idx % 3]);
    if (element > scale) {
      scale = element;
    }
  }

  if (!isfinite(determinant) || scale <= 0.0 ||
      fabs(determinant) <= 1.0e-12 * scale * scale * scale) {
    return 0;
  }

  inv_det = 1.0 / determinant;
  out[0][0] = (float)(cof00 * inv_det);
  out[0][1] = (float)(cof01 * inv_det);
  out[0][2] = (float)(cof02 * inv_det);
  out[1][0] = (float)(cof10 * inv_det);
  out[1][1] = (float)(cof11 * inv_det);
  out[1][2] = (float)(cof12 * inv_det);
  out[2][0] = (float)(cof20 * inv_det);
  out[2][1] = (float)(cof21 * inv_det);
  out[2][2] = (float)(cof22 * inv_det);
  return 1;
}

/**
 * @brief "把单位向量 v 转到 +Z" 的最小旋转四元数。
 *
 * 推导：
 *   转角 θ 满足 cos θ = v·ẑ = v[2]，用半角公式 cos(θ/2) = √((1+cosθ)/2) 得 w；
 *   转轴是 v×ẑ，展开后 z 分量恒为 0，x/y 分量化成 v[1]/(2w) 和 -v[0]/(2w)。
 */
static void Att_Quat_From_Vector_To_Z(const float v[3], float q[4]) {
  const float half_cos = sqrtf(0.5f * (v[2] + 1.0f));

  /* v 指向 -Z（板子倒扣）时 half_cos = 0，下面要除以它，必须特判。
   * 这种退化情况的最短旋转是"绕 x 轴转 180°"。 */
  if (half_cos < 1.0e-6f) {
    q[0] = 0.0f;
    q[1] = 1.0f;
    q[2] = 0.0f;
    q[3] = 0.0f;
    return;
  }

  q[0] = half_cos;
  q[1] = v[1] / (2.0f * half_cos);
  q[2] = -v[0] / (2.0f * half_cos);
  q[3] = 0.0f;
  Att_Quat_Normalize(q);
}

/* ==========================================================================
 * 二、小工具
 * ========================================================================== */

/** @brief 限幅：把 value 卡在 [min_value, max_value] 之间。 */
static float Att_Constrain(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

/** @brief 三个分量都是有限数（不是 NaN、不是 ±Inf）？ */
static uint8_t Att_Vector_Is_Valid(const float v[3]) {
  return (uint8_t)(isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]));
}

/* 三、两级一阶串联低通
 * 每级 y += alpha*(x-y)，alpha = dt/(tau/2+dt)，两级串联得二阶响应。
 * 不用二阶 Butterworth biquad：它两个极点在 z≈1 附近，浮点递推的舍入误差
 * 会被放大成缓慢发散（实测 double 也飘到几千），两级一阶无条件稳定。
 * 状态数组长度 = 2×通道数；state[0] == NaN 表示未启动。 */

/** @brief 把滤波器状态全部填 NaN，标记"未启动"。 */
static void Att_Filter_Reset_State(uint32_t length, double state[]) {
  uint32_t i;

  for (i = 0; i < 2 * length; i++) {
    state[i] = (double)NAN;
  }
}

/**
 * @brief 对 length 个通道分别做两级一阶低通。
 *
 * 每个通道占两个状态：state[2i] 是第一级，state[2i+1] 是第二级。
 * 输入先过第一级，第一级的输出再过第二级，第二级的输出就是结果。
 */
static void Att_Filter_Vector(const float in[], uint32_t length, float tau,
                              float dt, double state[], float out[]) {
  const float section_tau = 0.5f * tau; /* 两级平分时间常数 */
  const float alpha =
      (section_tau > 0.0f) ? (dt / (section_tau + dt)) : 1.0f;
  uint32_t i;

  /* 第一次调用：用输入值填满两级状态，避免从 0 慢慢爬升的瞬态 */
  if (isnan(state[0])) {
    for (i = 0; i < length; i++) {
      state[2 * i] = (double)in[i];
      state[2 * i + 1] = (double)in[i];
    }
  }

  for (i = 0; i < length; i++) {
    /* 第一级 */
    state[2 * i] +=
        (double)alpha * ((double)in[i] - state[2 * i]);
    /* 第二级：输入是第一级的输出 */
    state[2 * i + 1] +=
        (double)alpha * (state[2 * i] - state[2 * i + 1]);
    out[i] = (float)state[2 * i + 1];
  }
}

/* ==========================================================================
 * 四、零偏估计的系数换算
 * ========================================================================== */

/**
 * @brief 把"标准差(deg/s)"换算成"方差(rad/s)²"，供 EKF 使用。
 *
 * 卡尔曼滤波里用的是方差，不是标准差，所以每个参数都要平方。
 * vertical_noise 故意放大一万倍（除以 1e-4），等于"竖直方向基本不信观测"：
 * 绕重力轴转不改变重力方向，z 轴零偏没有绝对参考可测，硬估会把 yaw 拽跑。
 */
static void Att_Calculate_Coefficient(Attitude_VQF *att) {
  const float sigma_init = att->bias_sigma_init_deg_s * ATT_DEG2RAD;
  const float sigma_motion = att->bias_sigma_motion_deg_s * ATT_DEG2RAD;
  const float sigma_rest = att->bias_sigma_rest_deg_s * ATT_DEG2RAD;

  att->covariance_init = sigma_init * sigma_init;
  att->motion_noise = sigma_motion * sigma_motion;
  att->rest_noise = sigma_rest * sigma_rest;
  att->vertical_noise = att->motion_noise / 1.0e-4f;
}

/* 五、在线零偏估计（3×3 卡尔曼滤波）
 * 标准五步：预测协方差 → 算残差 → 算增益 → 更新状态 → 更新协方差。
 *   静止：H = 单位矩阵，观测值 = 0.5s 低通后的角速度（此时它几乎就是零偏）
 *   运动：H = 低通后的旋转矩阵，观测值 = 重力残余倾角 / tau_accel
 * ⚠ 运动那一路量纲不成立（residual 是单帧增量不是稳态误差），慢约 4800 倍，
 *   等于没起作用，所以实际都靠 ZARU 压零偏。 */

/**
 * @brief 用一帧数据更新零偏估计。
 * @param tilt_error 残余倾角（已归一化的单位向量），由 Attitude_Update 传进来
 */
static void Att_Update_Bias(Attitude_VQF *att, const float tilt_error[3]) {
  float rot[3][3];             /* 当前姿态的旋转矩阵 R */
  float rot_flat[9];           /* R 摊平成一维，喂给低通 */
  float rot_lpf[9];            /* 低通后的 R，就是观测矩阵 H */
  float bias_earth[2];         /* 零偏在地理系水平面的投影 */
  float bias_earth_lpf[2];     /* 上式的低通 */
  float obs[3][3];             /* 观测矩阵 H */
  float obs_value[3];          /* 观测值 z */
  float obs_noise[3];          /* 观测噪声 R（方差） */
  float predicted[3];          /* H·x，预测的观测值 */
  float residual[3];           /* z - H·x，新息 */
  float obs_t[3][3];           /* Hᵀ */
  float obs_p[3][3];           /* H·P */
  float s_matrix[3][3];        /* S = H·P·Hᵀ + R */
  float s_inv[3][3];           /* S⁻¹ */
  float p_obs_t[3][3];         /* P·Hᵀ */
  float k_gain[3][3];          /* K = P·Hᵀ·S⁻¹ */
  float kh[3][3];              /* K·H */
  float khp[3][3];             /* K·H·P */
  float step[3];               /* K·新息，本帧要加到零偏上的量 */
  float step_clip;
  float clip;
  uint32_t i;
  uint32_t j;

  /* EKF 总开关。注意不要用 rest_bias_enable 来关它 ——
     那个变量还管着静止检测和 ZARU，关掉会让整个静止判据失效。 */
  if (att->bias_ekf_enable == 0) {
    return;
  }

  if (att->motion_bias_enable == 0 && att->rest_bias_enable == 0) {
    return; /* 两路都关了，什么都不做 */
  }

  /* ---- 准备观测矩阵和观测值 ---- */

  Att_Quat_To_Matrix(att->q, rot);

  /* 零偏在地理系水平面的投影（前两个分量） */
  bias_earth[0] = rot[0][0] * att->bias[0] + rot[0][1] * att->bias[1] +
                  rot[0][2] * att->bias[2];
  bias_earth[1] = rot[1][0] * att->bias[0] + rot[1][1] * att->bias[1] +
                  rot[1][2] * att->bias[2];

  /* 旋转矩阵摊平成一维，才能用 Att_Filter_Vector（它按一维数组处理） */
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      rot_flat[i * 3 + j] = rot[i][j];
    }
  }

  Att_Filter_Vector(rot_flat, 9, att->tau_accel, att->dt,
                    att->motion_rotation_lpf_state, rot_lpf);
  Att_Filter_Vector(bias_earth, 2, att->tau_accel, att->dt,
                    att->motion_bias_lpf_state, bias_earth_lpf);

  /* 一维数组还原成 3×3，作为运动模式下的观测矩阵 */
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      obs[i][j] = rot_lpf[i * 3 + j];
    }
  }

  if (att->rest_detected != 0 && att->rest_bias_enable != 0) {
    /* 静止模式：直接观测零偏，H = 单位矩阵 */
    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
        obs[i][j] = (i == j) ? 1.0f : 0.0f;
      }
      obs_value[i] = att->rest_last_gyro_lpf[i];
      obs_noise[i] = att->rest_noise;
    }
  } else if (att->motion_bias_enable != 0) {
    /* 运动模式：用残余倾角反推零偏在地理系水平面的分量 */
    obs_value[0] = -tilt_error[1] / att->tau_accel + bias_earth_lpf[0];
    obs_value[1] = tilt_error[0] / att->tau_accel + bias_earth_lpf[1];
    obs_value[2] = 0.0f; /* 竖直轴不观测 */
    obs_noise[0] = att->motion_noise;
    obs_noise[1] = att->motion_noise;
    obs_noise[2] = att->vertical_noise; /* 放大一万倍 = 基本放弃 */
  } else {
    return;
  }

  /* ---- 第 1 步：预测协方差，只增不减，直到撞上初值上限 ---- */
  for (i = 0; i < 3; i++) {
    if (att->covariance[i][i] < att->covariance_init) {
      att->covariance[i][i] += att->process_noise;
    }
  }

  /* ---- 第 2 步：算新息（观测值 - 预测值），并限幅 ---- */
  Att_Matrix_Multiply_Vector(obs, att->bias, predicted);

  clip = att->bias_clip_deg_s * ATT_DEG2RAD;
  for (i = 0; i < 3; i++) {
    residual[i] = Att_Constrain(obs_value[i] - predicted[i], -clip, clip);
  }

  /* ---- 第 3 步：算增益 K = P·Hᵀ·S⁻¹，其中 S = H·P·Hᵀ + R ---- */
  Att_Matrix_Transpose(obs, obs_t);
  Att_Matrix_Multiply(obs, att->covariance, obs_p);
  Att_Matrix_Multiply(obs_p, obs_t, s_matrix);
  for (i = 0; i < 3; i++) {
    s_matrix[i][i] += obs_noise[i];
  }

  if (Att_Matrix_Inverse(s_matrix, s_inv) == 0) {
    return; /* 奇异，本帧放弃更新 */
  }

  Att_Matrix_Multiply(att->covariance, obs_t, p_obs_t);
  Att_Matrix_Multiply(p_obs_t, s_inv, k_gain);

  /* ---- 第 4 步：更新状态（零偏） ---- */
  Att_Matrix_Multiply_Vector(k_gain, residual, step);

  if (att->rest_detected == 0 || att->rest_bias_enable == 0) {
    /* 运动状态：每帧限幅，避免观测噪声把零偏推跑 */
    step_clip = ATT_MOTION_BIAS_STEP_DEG_S * ATT_DEG2RAD;
    for (i = 0; i < 3; i++) {
      step[i] = Att_Constrain(step[i], -step_clip, step_clip);
      att->bias[i] += step[i];
    }
  } else {
    /* 静止状态：不限步长，收敛快一点 */
    for (i = 0; i < 3; i++) {
      att->bias[i] += step[i];
    }
  }

  for (i = 0; i < 3; i++) {
    att->bias[i] = Att_Constrain(att->bias[i], -clip, clip);
  }

  /* ---- 第 5 步：更新协方差 P = P - K·H·P ---- */
  Att_Matrix_Multiply(k_gain, obs, kh);
  Att_Matrix_Multiply(kh, att->covariance, khp);

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      att->covariance[i][j] -= khp[i][j];
    }
  }
}

/* ==========================================================================
 * 六、对外接口
 * ========================================================================== */

/**
 * @brief 初始化：设默认参数，然后清状态。
 *
 * 参数都放在结构体最前面，集中在这里赋默认值；
 * Attitude_Reset 只清状态不动参数，所以可以随时调 Reset 而不丢调参结果。
 */
void Attitude_Init(Attitude_VQF *att) {
  memset(att, 0, sizeof(*att));

  /* ---- 可调参数 ---- */
  att->tau_accel = 3.0f;              /* 加计修正时间常数 s */
  att->rest_filter_tau = 0.5f;        /* 静止检测低通时间常数 s */
  att->bias_sigma_init_deg_s = 0.5f;  /* 零偏初值标准差 deg/s */
  att->bias_forgetting_time = 100.0f; /* 零偏协方差遗忘时间 s */
  att->bias_clip_deg_s = 2.0f;        /* 零偏限幅 deg/s */
  att->bias_sigma_motion_deg_s = 0.1f;/* 运动时零偏观测标准差 */
  att->bias_sigma_rest_deg_s = 0.03f; /* 静止时零偏观测标准差 */
  att->rest_min_time = 1.5f;          /* 判定静止需要持续多久 s */
  att->rest_threshold_gyro_deg_s = 2.0f; /* 静止检测陀螺门限 */
  /* 加速度这一路只用来挡"剧烈冲击/自由落体"，不能当静止判据：
   * 桌面轻微振动就能让 |a - 低通(a)| 超过 0.5 m/s²，静止标志永远起不来。 */
  att->rest_threshold_accel = 2.0f;
  /* 运动零偏估计实测是"有害"的，不是"无效"。
   * 运动分支拿低通后的旋转矩阵（tau=3s）当观测矩阵 H，转动时 H 严重滞后，
   * 新息一直很大，每帧撞 0.01°/s 的步长限幅，0.5s 就能推 0.6°/s。
   * 实测 bias 从 -0.03 一路推到 +0.63、再甩到 -0.85 °/s，
   * 转 0.9s 就吃掉 0.5° —— 这就是"往返差 0.x 度"的来源。
   * 压零偏交给 ZARU 就够了。 */
  att->motion_bias_enable = 0;
  /* ★ rest_bias_enable 是"总开关"：它同时控制
   *   ① 静止检测（第 3、4 步的两条 Att_Filter_Vector）
   *   ② ZARU
   *   ③ EKF 的静止分支
   * 所以它必须是 1。之前为了关 EKF 把它设成 0，结果静止检测整个瘫掉，
   * rest_detected 永远是 0 → 零位标定永远不完成 → 输出恒为 0（"看不到数据"）。
   * 关 EKF 请用下面的 bias_ekf_enable。 */
  att->rest_bias_enable = 1;
  /* 3×3 EKF 整体关掉。两条分支实测都有害：
   *  - motion：H 滞后导致新息一直很大，0.5s 推 0.6°/s（同上）
   *  - rest：不限步长，刚转完的余速还在低通里，0.5s 就推走 0.72°/s
   * 压零偏交给 ZARU，实测静止漂移 0.05°/min。 */
  att->bias_ekf_enable = 0;

  Attitude_Reset(att);
  Attitude_Dwt_Init();
}

/**
 * @brief 清空所有状态，但保留可调参数。
 * 上电初始化时调用；检测到姿态异常时也可以调用它做软重启。
 */
void Attitude_Reset(Attitude_VQF *att) {
  uint32_t i;
  uint32_t j;

  att->dt = 0.001f;
  att->initialized = 0;

  /* 三个四元数都设成单位值 = "没有旋转" */
  att->q_gyro[0] = 1.0f;
  att->q_gyro[1] = 0.0f;
  att->q_gyro[2] = 0.0f;
  att->q_gyro[3] = 0.0f;

  att->q_accel[0] = 1.0f;
  att->q_accel[1] = 0.0f;
  att->q_accel[2] = 0.0f;
  att->q_accel[3] = 0.0f;

  att->q[0] = 1.0f;
  att->q[1] = 0.0f;
  att->q[2] = 0.0f;
  att->q[3] = 0.0f;

  /* 地理系重力低通 */
  att->accel_earth_lpf[0] = 0.0f;
  att->accel_earth_lpf[1] = 0.0f;
  att->accel_earth_lpf[2] = 0.0f;
  Att_Filter_Reset_State(3, att->accel_lpf_state);
  att->last_accel_correction_rate = 0.0f;

  /* 零偏 + EKF 内部滤波状态 */
  att->bias[0] = 0.0f;
  att->bias[1] = 0.0f;
  att->bias[2] = 0.0f;
  Att_Filter_Reset_State(9, att->motion_rotation_lpf_state);
  Att_Filter_Reset_State(2, att->motion_bias_lpf_state);

  /* 静止检测 */
  att->rest_detected = 0;
  att->rest_time = 0.0f;
  att->rest_gyro_sq_dev = 0.0f;
  att->rest_accel_sq_dev = 0.0f;
  att->rest_last_gyro_lpf[0] = 0.0f;
  att->rest_last_gyro_lpf[1] = 0.0f;
  att->rest_last_gyro_lpf[2] = 0.0f;
  att->rest_last_accel_lpf[0] = 0.0f;
  att->rest_last_accel_lpf[1] = 0.0f;
  att->rest_last_accel_lpf[2] = 0.0f;
  Att_Filter_Reset_State(3, att->rest_gyro_lpf_state);
  Att_Filter_Reset_State(3, att->rest_accel_lpf_state);

  att->last_corrected_gyro[0] = 0.0f;
  att->last_corrected_gyro[1] = 0.0f;
  att->last_corrected_gyro[2] = 0.0f;

  att->last_dt = 0.0f;
  att->rejected_count = 0;
  att->dt_clamped_count = 0;
  att->dt_lost_time = 0.0f;

  /* 系数换算（°/s → 方差） */
  Att_Calculate_Coefficient(att);

  /* 协方差矩阵重置成对角阵 */
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      att->covariance[i][j] = (i == j) ? att->covariance_init : 0.0f;
    }
  }
}

/**
 * @brief 主循环：喂一帧 IMU 数据，更新姿态。
 *
 * 执行顺序（不能乱）：
 *   1. 校验 gyro → 钳 dt → 量程保护
 *   2. 未初始化？用 accel 定初值，return
 *   3. 静止检测（gyro 路）→ 去零偏 → 指数映射积分 q_gyro
 *   4. 校验 accel → 静止检测（accel 路）→ 静止时跑 ZARU
 *   5. accel 转地理系 → 低通 → 乘 q_accel 求残余倾角 → 修正 q_accel
 *   6. q = q_accel × q_gyro，归一化
 *
 * @param gyro  角速度 rad/s（机体系，已减掉 BMI088 模块的固定零偏）
 * @param accel 加速度 m/s²（机体系，静止时指向天）
 * @param dt    距上一帧的时间（秒），用 Attitude_Dt_Get() 取
 */
void Attitude_Update(Attitude_VQF *att, const float gyro[3],
                     const float accel[3], float dt) {
  float accel_len;              /* |accel| */
  float accel_earth[3];         /* 转到地理系的加速度 */
  float tilt_error[3];          /* 残余倾角（q_accel 坐标系下） */
  float tilt_error_len;
  float q_correction[4];        /* 本帧的修正旋转 */
  float q_tmp[4];               /* 四元数乘法的临时输出 */
  float rate_norm;              /* 去零偏后的角速度大小 */
  float step_angle;             /* 本帧转过的角度 */
  float gyro_threshold;
  float accel_threshold;
  float corr_w;
  uint32_t i;

  /* ==================== 1) 入口保护 ==================== */

  if (!Att_Vector_Is_Valid(gyro)) {
    att->rejected_count++;
    return;
  }

  if (dt < ATT_DT_MIN) {
    dt = ATT_DT_MIN;
  } else if (dt > ATT_DT_MAX) {
    /* 这一帧太长：把超出的部分记下来。它对应的角度是真的丢了，
       不是"算错"——所以要在调试器里盯着 dt_lost_time，
       它一涨就说明主循环被阻塞，此时任何转角都会偏小。 */
    att->dt_clamped_count++;
    att->dt_lost_time += dt - ATT_DT_MAX;
    dt = ATT_DT_MAX;
  }

  att->dt = dt;
  att->last_dt = dt;

  att->process_noise =
      att->covariance_init * dt / att->bias_forgetting_time;

  /* 陀螺量程保护：超过物理量程的数据直接丢 */
  if (fabsf(gyro[0]) > ATT_GYRO_MAX_RAD_S ||
      fabsf(gyro[1]) > ATT_GYRO_MAX_RAD_S ||
      fabsf(gyro[2]) > ATT_GYRO_MAX_RAD_S) {
    att->rejected_count++;
    return;
  }

  /* ==================== 2) 还没初始化：用加速度计定初值 ==================== */

  if (att->initialized == 0) {
    accel_len = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                      accel[2] * accel[2]);

    if (!Att_Vector_Is_Valid(accel) ||
        accel_len < ATT_ACCEL_MIN_G * ATT_GRAVITY ||
        accel_len > ATT_ACCEL_MAX_G * ATT_GRAVITY) {
      return; /* 等一帧有效数据 */
    }

    {
      float unit[3];
      float inv_len = 1.0f / accel_len;

      unit[0] = accel[0] * inv_len;
      unit[1] = accel[1] * inv_len;
      unit[2] = accel[2] * inv_len;

      /* 把重力方向转到 +Z 的旋转，就是当前姿态 */
      Att_Quat_From_Vector_To_Z(unit, att->q_accel);
    }

    memcpy(att->q_gyro, att->q_accel, sizeof(att->q_gyro));
    memcpy(att->q, att->q_accel, sizeof(att->q));

    /* 低通初值：用当前姿态把加速度转到地理系 */
    {
      float rot[3][3];
      Att_Quat_To_Matrix(att->q, rot);
      Att_Matrix_Multiply_Vector(rot, accel, att->accel_earth_lpf);
    }

    att->initialized = 1;
    return; /* 第一帧只定初值，不积分不修正 */
  }

  /* ==================== 3) 静止检测（gyro 路）+ 陀螺积分 ==================== */

  if (att->rest_bias_enable != 0) {
    float gyro_lpf[3];
    float dev[3];

    /* 静止时陀螺读数应该是"平"的，所以比较当前值和低通值的偏差 */
    Att_Filter_Vector(gyro, 3, att->rest_filter_tau, dt,
                      att->rest_gyro_lpf_state, gyro_lpf);

    for (i = 0; i < 3; i++) {
      att->rest_last_gyro_lpf[i] = gyro_lpf[i];
      dev[i] = gyro[i] - gyro_lpf[i];
    }

    att->rest_gyro_sq_dev =
        dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2];

    gyro_threshold = att->rest_threshold_gyro_deg_s * ATT_DEG2RAD;

    /* 严重异常：抖动过大，或者低通角速度本身就不小（在匀速转动）
     * → 计时清零，立刻退出静止。
     * 第二个条件很关键：匀速转动时"抖动"也是 0，光看抖动会把转动判成静止，
     * 然后 ZARU 就把转动速度当成零偏吃掉了。 */
    if (att->rest_gyro_sq_dev >=
            (ATT_REST_SEVERE_GYRO_DEG_S * ATT_DEG2RAD) *
                (ATT_REST_SEVERE_GYRO_DEG_S * ATT_DEG2RAD) ||
        gyro_lpf[0] * gyro_lpf[0] + gyro_lpf[1] * gyro_lpf[1] +
                gyro_lpf[2] * gyro_lpf[2] >
            (ATT_REST_EXIT_GYRO_DEG_S * ATT_DEG2RAD) *
                (ATT_REST_EXIT_GYRO_DEG_S * ATT_DEG2RAD)) {
      att->rest_time = 0.0f;
      att->rest_detected = 0;
    } else if (att->rest_gyro_sq_dev >= gyro_threshold * gyro_threshold) {
      /* 轻微异常（单个采样点毛刺）：只扣时间，不清零。
       * 1.6 kHz 下静止窗口要攒 2400 帧，每个毛刺都清零就永远攒不满。 */
      att->rest_time -= ATT_REST_TIME_DECAY * dt;
      if (att->rest_time < 0.0f) {
        att->rest_time = 0.0f;
      }
    }
  }

  /* ---- 去零偏 ---- */
  for (i = 0; i < 3; i++) {
    att->last_corrected_gyro[i] = gyro[i] - att->bias[i];
  }

  /* ---- 指数映射积分：把"这一帧的角速度"变成"这一帧的旋转" ---- */
  rate_norm = sqrtf(att->last_corrected_gyro[0] * att->last_corrected_gyro[0] +
                    att->last_corrected_gyro[1] * att->last_corrected_gyro[1] +
                    att->last_corrected_gyro[2] * att->last_corrected_gyro[2]);
  step_angle = rate_norm * dt;

  if (rate_norm > 1.0e-9f) {
    float step_w = cosf(0.5f * step_angle);
    float step_scale = sinf(0.5f * step_angle) / rate_norm;
    float q_step[4];

    q_step[0] = step_w;
    q_step[1] = step_scale * att->last_corrected_gyro[0];
    q_step[2] = step_scale * att->last_corrected_gyro[1];
    q_step[3] = step_scale * att->last_corrected_gyro[2];

    /* 右乘：角速度是在机体系测的，增量要作用在机体侧 */
    Att_Quat_Multiply(att->q_gyro, q_step, q_tmp);
    memcpy(att->q_gyro, q_tmp, sizeof(q_tmp));
    Att_Quat_Normalize(att->q_gyro);
  }

  /* ==================== 4) 静止检测（accel 路）+ ZARU ==================== */

  accel_len = sqrtf(accel[0] * accel[0] + accel[1] * accel[1] +
                    accel[2] * accel[2]);

  if (!Att_Vector_Is_Valid(accel) ||
      accel_len < ATT_ACCEL_MIN_G * ATT_GRAVITY ||
      accel_len > ATT_ACCEL_MAX_G * ATT_GRAVITY) {
    att->rejected_count++;
    return; /* 只保留陀螺积分 */
  }

  if (att->rest_bias_enable != 0) {
    float accel_lpf[3];
    float dev[3];

    Att_Filter_Vector(accel, 3, att->rest_filter_tau, dt,
                      att->rest_accel_lpf_state, accel_lpf);

    for (i = 0; i < 3; i++) {
      att->rest_last_accel_lpf[i] = accel_lpf[i];
      dev[i] = accel[i] - accel_lpf[i];
    }

    att->rest_accel_sq_dev =
        dev[0] * dev[0] + dev[1] * dev[1] + dev[2] * dev[2];

    accel_threshold = att->rest_threshold_accel;

    /* 加速度这一路同样两级：8 m/s² 以上算冲击/自由落体（清零），
     * 2 m/s² 以上只扣时间。只有"正常"那一支才累加计时。 */
    if (att->rest_accel_sq_dev >=
        ATT_REST_SEVERE_ACCEL_MPS2 * ATT_REST_SEVERE_ACCEL_MPS2) {
      att->rest_time = 0.0f;
      att->rest_detected = 0;
    } else if (att->rest_accel_sq_dev >= accel_threshold * accel_threshold) {
      att->rest_time -= ATT_REST_TIME_DECAY * dt;
      if (att->rest_time < 0.0f) {
        att->rest_time = 0.0f;
      }
    } else {
      att->rest_time += dt;
      if (att->rest_time >= att->rest_min_time) {
        att->rest_detected = 1;
      }
    }
  }

  /* ---- ZARU：静止时把零偏朝"低通角速度"拽，时间常数 1.0 s ---- */
  if (att->rest_bias_enable != 0 && att->rest_detected != 0) {
    float lpf_len =
        sqrtf(att->rest_last_gyro_lpf[0] * att->rest_last_gyro_lpf[0] +
              att->rest_last_gyro_lpf[1] * att->rest_last_gyro_lpf[1] +
              att->rest_last_gyro_lpf[2] * att->rest_last_gyro_lpf[2]);

    /* 两道闸：
     * 1) rest_time 要超过 rest_min_time + ATT_REST_ZARU_SETTLE_TIME
     *    —— 等静止低通的残余衰减干净，否则会把"刚转完的余速"吸成零偏；
     * 2) 低通角速度小于 0.5°/s
     *    —— 否则手慢慢转 1.3°/s 会被当成静止，ZARU 把这个"真实转动"吸成零偏，
     *       松手后姿态被拽回去，现象是"挪一点就给我修正回去"。 */
    if ((att->rest_time >= (att->rest_min_time + ATT_REST_ZARU_SETTLE_TIME)) &&
        (lpf_len <= ATT_REST_ZARU_MAX_DEG_S * ATT_DEG2RAD)) {
      float gain = dt / (ATT_REST_BIAS_TAU + dt);
      float bias_clip = att->bias_clip_deg_s * ATT_DEG2RAD;

      for (i = 0; i < 3; i++) {
        att->bias[i] += gain * (att->rest_last_gyro_lpf[i] - att->bias[i]);
        att->bias[i] = Att_Constrain(att->bias[i], -bias_clip, bias_clip);
      }
    }
  }

  /* ==================== 5) 加速度计修正 ==================== */

  /* 5.1 转到地理系，再狠低通（tau = 3 s，压掉运动加速度） */
  {
    float rot[3][3];
    Att_Quat_To_Matrix(att->q_gyro, rot);
    Att_Matrix_Multiply_Vector(rot, accel, accel_earth);
  }

  Att_Filter_Vector(accel_earth, 3, att->tau_accel, dt,
                    att->accel_lpf_state, att->accel_earth_lpf);

  /* 5.2 ★关键★ 残余倾角要先乘 q_accel，修正量才有反馈。
   * 省掉这一步，每帧都在修正"同一个"误差，而 q_accel 一直在累积，
   * 表现为角度缓慢加速旋转、几秒内转到天上。 */
  {
    float rot[3][3];
    Att_Quat_To_Matrix(att->q_accel, rot);
    Att_Matrix_Multiply_Vector(rot, att->accel_earth_lpf, tilt_error);
  }

  tilt_error_len = sqrtf(tilt_error[0] * tilt_error[0] +
                         tilt_error[1] * tilt_error[1] +
                         tilt_error[2] * tilt_error[2]);

  if (tilt_error_len > 1.0e-6f) {
    float inv_len = 1.0f / tilt_error_len;

    tilt_error[0] *= inv_len;
    tilt_error[1] *= inv_len;
    tilt_error[2] *= inv_len;

    /* 5.3 构造"把 tilt_error 转到 +Z"的最小旋转。
     * 这里把 Att_Quat_From_Vector_To_Z 的公式内联展开，
     * 为了复用已经算好的 corr_w，省一次 sqrtf。 */
    corr_w = sqrtf(Att_Constrain(0.5f * (tilt_error[2] + 1.0f), 0.0f, 1.0f));

    if (corr_w > 1.0e-6f) {
      q_correction[0] = corr_w;
      q_correction[1] = 0.5f * tilt_error[1] / corr_w;
      q_correction[2] = -0.5f * tilt_error[0] / corr_w;
      q_correction[3] = 0.0f;
    } else {
      /* 残余倾角指向 -Z：退化成绕 x 轴转 180° */
      q_correction[0] = 0.0f;
      q_correction[1] = 1.0f;
      q_correction[2] = 0.0f;
      q_correction[3] = 0.0f;
    }

    /* 5.4 左乘累积到 q_accel 上（q_correction 在地理系侧，所以左乘） */
    Att_Quat_Multiply(q_correction, att->q_accel, q_tmp);
    memcpy(att->q_accel, q_tmp, sizeof(q_tmp));
    Att_Quat_Normalize(att->q_accel);

    /* 调试用：本帧的修正角速度，调 tau_accel 时看这个 */
    att->last_accel_correction_rate =
        acosf(Att_Constrain(tilt_error[2], -1.0f, 1.0f)) / dt;

    Att_Update_Bias(att, tilt_error);
  }

  /* ==================== 6) 6D 融合 ==================== */

  /* 顺序不能反：q_accel 是地理系侧的修正，所以左乘 */
  Att_Quat_Multiply(att->q_accel, att->q_gyro, q_tmp);
  memcpy(att->q, q_tmp, sizeof(q_tmp));
  Att_Quat_Normalize(att->q);
}

/**
 * @brief 取欧拉角（弧度）。roll 绕 x、pitch 绕 y、yaw 绕 z。
 * 传 NULL 表示不需要那一项。
 * 注意：pitch 在 ±90° 有万向锁，做控制请直接用四元数。
 */
void Attitude_Get_Euler(const Attitude_VQF *att, float *roll, float *pitch,
                        float *yaw) {
  const float qw = att->q[0];
  const float qx = att->q[1];
  const float qy = att->q[2];
  const float qz = att->q[3];
  /* asinf 收到超出 [-1,1] 的参数会返回 NaN，所以必须限幅 */
  const float sin_pitch = Att_Constrain(2.0f * (qw * qy - qz * qx), -1.0f, 1.0f);

  if (roll != NULL) {
    *roll = atan2f(2.0f * (qw * qx + qy * qz),
                   1.0f - 2.0f * (qx * qx + qy * qy));
  }

  if (pitch != NULL) {
    *pitch = asinf(sin_pitch);
  }

  if (yaw != NULL) {
    *yaw = atan2f(2.0f * (qw * qz + qx * qy),
                  1.0f - 2.0f * (qy * qy + qz * qz));
  }
}

/** @brief 取四元数 q = [w, x, y, z]（机体 → 地理系）。控制用四元数，不要用欧拉角。 */
void Attitude_Get_Quaternion(const Attitude_VQF *att, float q[4]) {
  q[0] = att->q[0];
  q[1] = att->q[1];
  q[2] = att->q[2];
  q[3] = att->q[3];
}

/** @brief 取在线零偏估计（rad/s），可用来回填 BMI088 或看估计效果。 */
void Attitude_Get_Bias(const Attitude_VQF *att, float bias[3]) {
  bias[0] = att->bias[0];
  bias[1] = att->bias[1];
  bias[2] = att->bias[2];
}

/** @brief 设置零偏初值（rad/s）。sigma_deg_s <= 0 时保持当前协方差。 */
void Attitude_Set_Bias_Estimate(Attitude_VQF *att, const float bias[3],
                                float sigma_deg_s) {
  const float clip = att->bias_clip_deg_s * ATT_DEG2RAD;

  if (!Att_Vector_Is_Valid(bias)) {
    return;
  }

  att->bias[0] = Att_Constrain(bias[0], -clip, clip);
  att->bias[1] = Att_Constrain(bias[1], -clip, clip);
  att->bias[2] = Att_Constrain(bias[2], -clip, clip);

  if (sigma_deg_s > 0.0f && isfinite(sigma_deg_s)) {
    float sigma = sigma_deg_s * ATT_DEG2RAD;
    float variance = sigma * sigma;

    /* 协方差重置成对角阵：只改对角，非对角清零 */
    att->covariance[0][0] = variance;
    att->covariance[1][1] = variance;
    att->covariance[2][2] = variance;
    att->covariance[0][1] = 0.0f;
    att->covariance[0][2] = 0.0f;
    att->covariance[1][0] = 0.0f;
    att->covariance[1][2] = 0.0f;
    att->covariance[2][0] = 0.0f;
    att->covariance[2][1] = 0.0f;
  }
}

/** @brief 1 = 当前判定为静止。 */
uint8_t Attitude_Get_Rest_Detected(const Attitude_VQF *att) {
  return att->rest_detected;
}

/* ==========================================================================
 * 七、DWT 计时
 * 1.6 kHz 一帧只有 625 µs，HAL_GetTick 的 1 ms 精度不够。
 * ========================================================================== */

/** @brief 打开 DWT 周期计数器。 */
void Attitude_Dwt_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/** @brief 返回距上次调用的时间（秒），已钳在 [0.1ms, 10ms]。 */
float Attitude_Dt_Get(void) {
  static uint32_t last_cycle = 0;
  static uint8_t started = 0;
  uint32_t now = DWT->CYCCNT;
  uint32_t delta_cycle;
  float dt;

  if (started == 0) {
    /* 第一帧没有"上一帧"，直接返回最小值 */
    last_cycle = now;
    started = 1;
    return ATT_DT_MIN;
  }

  /* 无符号减法：计数器溢出时会自动回绕，不用特判 */
  delta_cycle = now - last_cycle;
  last_cycle = now;

  dt = (float)delta_cycle / (float)SystemCoreClock;

  if (!isfinite(dt) || dt < ATT_DT_MIN) {
    dt = ATT_DT_MIN;
  } else if (dt > ATT_DT_MAX) {
    dt = ATT_DT_MAX;
  }

  return dt;
}
