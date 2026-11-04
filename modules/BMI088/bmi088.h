#ifndef BMI088_H
#define BMI088_H

#include "bsp_spi.h"
#include <stdint.h>

/* ======================== 加速度计寄存器 ======================== */
#define BMI088_ACC_CHIP_ID 0x00
#define BMI088_ACC_DATA 0x12/* X/Y/Z 共 6 字节，低字节在前 */
#define BMI088_ACC_TEMP_MSB 0x22
#define BMI088_ACC_TEMP_LSB 0x23
#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_RANGE 0x41
#define BMI088_ACC_INT1_IO_CTRL 0x53 /* INT1 引脚方向/电平/输出使能 */
#define BMI088_ACC_INT_MAP_DATA 0x58 /* 哪个事件映射到 INT1/INT2 */
#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_SOFTRESET 0x7E

/* ======================== 陀螺仪寄存器 ======================== */
#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_DATA 0x02 /* X/Y/Z 共 6 字节，低字节在前 */
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_BANDWIDTH 0x10
#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_SOFTRESET 0x14

/* ======================== 固定值 / 本工程配置 ======================== */
#define BMI088_READ_FLAG 0x80
#define BMI088_SOFTRESET_VALUE 0xB6

#define BMI088_ACC_CHIP_ID_VALUE 0x1E
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_ACC_PWR_CTRL_VALUE 0x04 /* bit2=1 打开加速度计 */
#define BMI088_ACC_PWR_CONF_VALUE 0x00 /* 默认 0x03 是挂起，改成 0x00 工作 */
#define BMI088_ACC_CONF_VALUE 0xAC /* 0x0A = 400Hz ODR，0x0C = 正常带宽；1.6kHz 用 0xCC */
#define BMI088_ACC_RANGE_VALUE 0x01 /* ±6g：0/1/2/3 = 3/6/12/24g */

#define BMI088_GYRO_RANGE_VALUE 0x00/* ±2000°/s */
/* 陀螺带宽 bits[3:0]：0x1=2000Hz/230Hz，0x2=1000Hz/116Hz（现在），0x3=400Hz/47Hz。
 * 降带宽是因为实测 z 轴噪声 0.637°/s rms（规格书 230Hz 下 0.21），频谱平坦属宽带噪声；
 * yaw 靠积分，噪声会变成随机游走，带宽减半噪声降 √2。代价是相位滞后 +1.7ms。
 * 对总转角没影响，只影响角速度实时性；以后拿 g_yaw_rate_deg_s 做高速内环觉得延迟大就改回 0x1。 */
#define BMI088_GYRO_BANDWIDTH_VALUE                                            \
  0x82 /* 1000Hz ODR / 116Hz 带宽，bit7 只读恒 1 */

/* 换算系数（改量程要一起改） */
#define BMI088_ACC_SCALE_G (6.0f / 32768.0f)
#define BMI088_GYRO_SCALE_DPS (2000.0f / 32768.0f)
#define BMI088_DEG_TO_RAD (0.01745329252f)

/* 灵敏度微调（scale factor trim），1.0 = 不修正。
 * ±2000°/s 量程下芯片灵敏度容差约 ±1%，真实转 90° 可能只读到 89.1°。
 * 标定：等零位锁定后精确转 360°（别转 90°，手抖 2° 就是 2% 误差），
 * 新系数 = 旧系数 × 360 / |读数|。一次就够，除非换芯片。
 * Z 轴已按实测标定：转 90° 17 次均值 89.66°，比例 90/89.66 = 1.0038。
 * X/Y 没做过对比测试，保持 1.0。 */
#define BMI088_GYRO_TRIM_X 1.0f
#define BMI088_GYRO_TRIM_Y 1.0f
#define BMI088_GYRO_TRIM_Z 1.0038f

typedef struct {
  float accel[3];    /* m/s²，机体系 x/y/z */
  float gyro[3];     /* rad/s */
  float temperature; /* ℃ */
} BMI088_Data;

/* ======================== 实例 ======================== */
typedef struct BMI088_Instance_s BMI088_Instance;

typedef struct {
  SPI_HandleTypeDef *handle; /* hspi2 */
  GPIO_TypeDef *acc_cs_port;
  uint16_t acc_cs_pin;

  GPIO_TypeDef *gyro_cs_port;
  uint16_t gyro_cs_pin;

  void (*callback)(BMI088_Instance *bmi088); /* 每次读完后调用，可为 NULL */
  void *id;
} BMI088_Init_Config_s;

struct BMI088_Instance_s {
  SPI_Instance *spi_accel;
  SPI_Instance *spi_gyro;
  void (*callback)(BMI088_Instance *bmi088);
  void *id;

  BMI088_Data data;
  uint32_t sample_count;
  uint32_t timeout_count; /* INT 超时兜底次数：一直涨说明中断路径有问题 */

  float gyro_offset[3];    /* 标定得到的零偏，rad/s，读的时候减掉 */
  float gyro_trim[3];      /* 灵敏度微调系数，默认 1.0，见 BMI088_GYRO_TRIM_* */
  uint8_t gyro_calibrated; /* 1 = 标定完成 */
};

BMI088_Instance *BMI088_Register(const BMI088_Init_Config_s *conf);
uint8_t BMI088_Init(BMI088_Instance *bmi088);           /* 1 成功，0 失败 */
HAL_StatusTypeDef BMI088_Read(BMI088_Instance *bmi088); /* 读一组数据到 data */
uint8_t BMI088_Calibrate_Gyro(BMI088_Instance *bmi088);
#endif