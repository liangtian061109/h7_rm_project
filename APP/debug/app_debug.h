#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

#include "app_init.h"

/* 集中放"给调试器看的变量"。任务文件只往里写，这里不含任何逻辑。
 * 单独放是为了让任务逻辑不被一堆 volatile 声明淹没。 */

/* FreeRTOS 断言现场：触发后记录文件/行号再停住 */
extern const char *volatile g_assert_file;
extern volatile int g_assert_line;

/* 栈溢出任务名（configCHECK_FOR_STACK_OVERFLOW=2 时有效） */
extern const char *volatile g_overflow_task;

void vAssertCalled(const char *file, int line);

/* ==================== 姿态（BMI088 任务写） ==================== */

extern volatile float g_att_roll;
extern volatile float g_att_pitch;
extern volatile float g_att_yaw;
extern volatile float g_att_bias[3];
extern volatile uint8_t g_att_rest;

/* 度数版本，只给显示 */
extern volatile float g_att_roll_deg;
extern volatile float g_att_pitch_deg;
extern volatile float g_att_yaw_deg;       /* 折到 ±180，控制用 */
extern volatile float g_att_yaw_total_deg; /* 连续累积，多圈用 */

/* 四元数 [w,x,y,z]。pitch 过 ±90° 时欧拉角会跳，四元数不会。 */
extern volatile float g_att_q[4];

extern volatile float g_yaw_rate_deg_s; /* 机体系 z 轴角速度 */

/* 诊断量，正常应恒为 0。dt_clamped 一涨就说明主循环被阻塞过，
 * 那段时间的角度真丢了 —— 排查"转 90° 只读到 85°"先看这三个。 */
extern volatile uint32_t g_att_dt_clamped;
extern volatile float g_att_dt_lost_s;
extern volatile uint32_t g_att_rejected;

/* ==================== 零位标定 ==================== */

extern volatile float g_att_roll_zero_deg;
extern volatile float g_att_pitch_zero_deg;
extern volatile float g_att_yaw_zero_deg;
extern volatile uint8_t g_att_zero_ready;    /* 1 = 三轴标定完成 */
extern volatile uint8_t g_att_zero_progress; /* 0~100 */

/* ==================== 复位原因 ==================== */

/* RCC->RSR 快照。位定义见 stm32h723xx.h，别照抄 F1/F4：
 * bit17 CPURSTF / bit21 BORRSTF / bit22 PINRSTF / bit23 PORRSTF / bit24 SFTRSTF。 */
extern volatile uint32_t g_reset_flags;

/* ==================== C620 状态（C620 任务写） ==================== */

extern volatile float g_c620_angle_deg[APP_C620_MOTOR_NUM];
extern volatile float g_c620_speed_rpm[APP_C620_MOTOR_NUM];
extern volatile float g_c620_current_a[APP_C620_MOTOR_NUM];
extern volatile uint8_t g_c620_online[APP_C620_MOTOR_NUM];
extern volatile uint8_t g_c620_error_code[APP_C620_MOTOR_NUM];

/* 发送失败累计。持续增长 = 总线有问题（没接收器/波特率不对/线序反） */
extern volatile uint32_t g_c620_send_fail;

/* PID 内部状态，只在闭环模式下更新，其他模式恒为 0 */
extern volatile float g_c620_pid_err[APP_C620_MOTOR_NUM];
extern volatile float g_c620_pid_out[APP_C620_MOTOR_NUM];      /* 限幅后输出 */
extern volatile float g_c620_pid_integral[APP_C620_MOTOR_NUM]; /* 积分项 */
extern volatile float g_c620_target_ramped[APP_C620_MOTOR_NUM]; /* 斜坡后目标 */

/* 堵转标志。1 = 通电但转不动，PID 已断输出，2 秒后自动重试。 */
extern volatile uint8_t g_c620_blocked[APP_C620_MOTOR_NUM];

#endif
