#include "bmi088.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_spi.h"
#include <stdint.h>
#include <string.h>

#define BMI088_TIMEOUT_MS 10
#define BMI088_MAX_INSTANCES 2

#define BMI088_CALIB_SAMPLES 1000 /* 閲?1000 甯ф眰骞冲潎锛?.6 kHz 涓嬬害 0.6 s */

uint8_t BMI088_Calibrate_Gyro(BMI088_Instance *bmi088); /* 1 鎴愬姛锛? 澶辫触 */

static BMI088_Instance bmi088_instances[BMI088_MAX_INSTANCES];
static uint8_t bmi088_instance_count = 0;

/* ---------- 绛夎繖娆?DMA 浜嬪姟缁撴潫锛圔SP 鍦ㄥ畬鎴愪腑鏂噷娓?busy锛?---------- */
static HAL_StatusTypeDef BMI088_Wait_Idle(SPI_Instance *spi) {
  const uint32_t start = HAL_GetTick();

  while (spi->busy != 0) {
    if ((HAL_GetTick() - start) > BMI088_TIMEOUT_MS) {
      SPI_Abort(spi);
      return HAL_TIMEOUT;
    }
  }
  return HAL_OK;
}

/* ---------- 鍐欎竴涓瘎瀛樺櫒锛堝湴鍧€鏈€楂樹綅 0锛?---------- */
static HAL_StatusTypeDef BMI088_Write_Register(SPI_Instance *spi, uint8_t reg,
                                               uint8_t value) {

  const uint8_t buffer[2] = {(uint8_t)(reg & (uint8_t)~BMI088_READ_FLAG),
                             value};
  /* 鐢ㄩ樆濉炲彂閫侊細鍐欏畬杩斿洖鏃舵€荤嚎宸茬粡鐪熸鍙戝畬锛屼笅涓€绗旇鍙互椹笂寮€濮嬨€?
     2 涓瓧鑺傚湪 10 MHz 涓嬩笉鍒?2 us锛屽紑閿€鍙互蹇界暐銆?*/
  return SPI_TransmitBlocking(spi, buffer, 2, 5);
}
/* ---------- 璇诲瘎瀛樺櫒锛堝湴鍧€鏈€楂樹綅 1锛?----------
 * dummy = 1锛氬姞閫熷害璁★紝鍦板潃鍚庤ˉ 1 涓搼瀛楄妭锛屾暟鎹湪 rx_buffer[2] 璧?
 * dummy = 0锛氶檧铻轰华锛? 鍦板潃鍚庣洿鎺ユ槸鏁版嵁锛屾暟鎹湪 rx_buffer[1] 璧?
 */
static HAL_StatusTypeDef BMI088_Read_Register(SPI_Instance *spi, uint8_t reg,
                                              uint8_t *rx, uint16_t size,
                                              uint8_t dummy) {
  uint8_t buffer[2] = {(uint8_t)(reg | BMI088_READ_FLAG), 0x00};
  const uint16_t tx_size = (dummy != 0) ? 2 : 1;

  HAL_StatusTypeDef status = SPI_TransmitReceive(spi, buffer, tx_size, size);
  if (status != HAL_OK) {
    return status;
  }
  if (BMI088_Wait_Idle(spi) != HAL_OK) {
    return HAL_TIMEOUT;
  }

  /* BSP 绾﹀畾锛氭湁鏁堟暟鎹粠 rx_buffer[tx_length] 寮€濮?*/
  memcpy(rx, &spi->rx_buffer[tx_size], size);
  return HAL_OK;
}

/* ---------- 寰幆璇昏姱鐗?ID锛氫笂鐢电涓€娆¤鏃犳晥锛岄潬閲嶈瘯鍒囨崲杩?SPI ---------- */
static HAL_StatusTypeDef BMI088_Wait_Chip_Id(SPI_Instance *spi,
                                             uint8_t expected, uint8_t dummy) {
  uint8_t id = 0;

  for (uint8_t attempt = 0; attempt < 20; attempt++) {
    if ((BMI088_Read_Register(spi, 0x00, &id, 1, dummy) == HAL_OK) &&
        (id == expected)) {
      return HAL_OK;
    }
    HAL_Delay(5);
  }
  return HAL_ERROR;
}

BMI088_Instance *BMI088_Register(const BMI088_Init_Config_s *conf) {
  if (conf == NULL || conf->handle == NULL || conf->acc_cs_port == NULL ||
      conf->gyro_cs_port == NULL ||
      bmi088_instance_count >= BMI088_MAX_INSTANCES) {
    return NULL;
  }

  /* 鍚屼竴鏍?SPI2 涓婃寕涓や釜浠庤澶囷紝鍚勮嚜娉ㄥ唽涓€涓疄渚嬶紝闈犵墖閫夊尯鍒?*/
  SPI_Init_Config_s acc_conf = {
      .handle = conf->handle,
      .cs_port = conf->acc_cs_port,
      .cs_pin = conf->acc_cs_pin,
      .id = NULL,
      .callback = NULL,
  };
  SPI_Init_Config_s gyro_conf = {
      .handle = conf->handle,
      .cs_port = conf->gyro_cs_port,
      .cs_pin = conf->gyro_cs_pin,
      .id = NULL,
      .callback = NULL,
  };

  SPI_Instance *spi_accel = SPI_Register(&acc_conf);
  SPI_Instance *spi_gyro = SPI_Register(&gyro_conf);
  if (spi_accel == NULL || spi_gyro == NULL) {
    return NULL;
  }

  BMI088_Instance *bmi088 = &bmi088_instances[bmi088_instance_count];
  memset(bmi088, 0, sizeof(BMI088_Instance));
  bmi088->spi_accel = spi_accel;
  bmi088->spi_gyro = spi_gyro;
  bmi088->callback = conf->callback;
  bmi088->id = conf->id;
  /* memset 会把 trim 清成 0（等于把陀螺输出全乘没了），必须显式设成 1.0 */
  bmi088->gyro_trim[0] = BMI088_GYRO_TRIM_X;
  bmi088->gyro_trim[1] = BMI088_GYRO_TRIM_Y;
  bmi088->gyro_trim[2] = BMI088_GYRO_TRIM_Z;
  bmi088_instance_count++;

  return bmi088;
}

#define BMI088_CFG_DELAY_MS 100

/* 鍐欎竴涓瘎瀛樺櫒鍐嶅洖璇绘瘮瀵癸紝鏈€澶?3 娆?*/
static uint8_t BMI088_Write_And_Verify(SPI_Instance *spi, uint8_t reg,
                                       uint8_t value, uint8_t dummy) {
  uint8_t readback;

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    readback = 0xFF;
    if (BMI088_Write_Register(spi, reg, value) != HAL_OK) {
      continue;
    }
    HAL_Delay(BMI088_CFG_DELAY_MS);
    if (BMI088_Read_Register(spi, reg, &readback, 1, dummy) != HAL_OK) {
      continue;
    }
    HAL_Delay(BMI088_CFG_DELAY_MS);
    if (readback == value) {
      return 1;
    }
  }
  return 0;
}

uint8_t BMI088_Init(BMI088_Instance *bmi088) {
  if (bmi088 == NULL || bmi088->spi_accel == NULL || bmi088->spi_gyro == NULL) {
    return 0;
  }

  SPI_Instance *acc = bmi088->spi_accel;
  SPI_Instance *gyro = bmi088->spi_gyro;

  HAL_Delay(2); /* 涓婄數 boot 鏃堕棿 */

  /* 鈶?澶嶄綅鍓嶅厛纭閫氫俊锛氫笂鐢垫槸 I虏C 妯″紡锛孋S 涓婂崌娌垮垏 SPI锛岀涓€娆¤鏃犳晥 */
  if (BMI088_Wait_Chip_Id(acc, BMI088_ACC_CHIP_ID_VALUE, 1) != HAL_OK) {
    return 0;
  }

  /* 鈶?杞浣嶏紝澶嶄綅鍚庢帴鍙ｅ張鍥?I虏C锛岄渶瑕侀噸鏂板搼璇诲垏鍥?SPI */
  if (BMI088_Write_Register(acc, BMI088_ACC_SOFTRESET,
                            BMI088_SOFTRESET_VALUE) != HAL_OK) {
    return 0;
  }
  HAL_Delay(BMI088_CFG_DELAY_MS);

  if (BMI088_Wait_Chip_Id(acc, BMI088_ACC_CHIP_ID_VALUE, 1) != HAL_OK) {
    return 0;
  }

  /* 鈶?鍔犻€熷害璁￠厤缃細姣忔潯閮芥槸鍐欏畬绛変竴浼氬効銆佸洖璇绘瘮瀵归€氳繃鍐嶅啓涓嬩竴鏉?*/
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_PWR_CTRL,
                               BMI088_ACC_PWR_CTRL_VALUE, 1)) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_PWR_CONF,
                               BMI088_ACC_PWR_CONF_VALUE, 1)) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_CONF, BMI088_ACC_CONF_VALUE,
                               1)) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_RANGE, BMI088_ACC_RANGE_VALUE,
                               1)) {
    return 0;
  }
  /* 鏁版嵁灏辩华涓柇锛欼NT1 鎺ㄦ尳杈撳嚭銆侀珮鐢靛钩鏈夋晥锛宒rdy 浜嬩欢鏄犲皠鍒?INT1(PE10) */
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_INT1_IO_CTRL, 0x08, 1)) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(acc, BMI088_ACC_INT_MAP_DATA, 0x04, 1)) {
    return 0;
  }

  /* 鈶?闄€铻轰华锛氬悓鏍风殑鑺傚 */
  if (BMI088_Wait_Chip_Id(gyro, BMI088_GYRO_CHIP_ID_VALUE, 0) != HAL_OK) {
    return 0;
  }
  if (BMI088_Write_Register(gyro, BMI088_GYRO_SOFTRESET,
                            BMI088_SOFTRESET_VALUE) != HAL_OK) {
    return 0;
  }
  HAL_Delay(30);
  if (BMI088_Wait_Chip_Id(gyro, BMI088_GYRO_CHIP_ID_VALUE, 0) != HAL_OK) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(gyro, BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_VALUE,
                               0)) {
    return 0;
  }
  if (!BMI088_Write_And_Verify(gyro, BMI088_GYRO_BANDWIDTH,
                               BMI088_GYRO_BANDWIDTH_VALUE, 0)) {
    return 0;
  }

  return 1;
}

static int16_t BMI088_To_Int16(const uint8_t *data) {
  return (int16_t)(((uint16_t)data[1] << 8) | data[0]); /* 浣庡瓧鑺傚湪鍓?*/
}

HAL_StatusTypeDef BMI088_Read(BMI088_Instance *bmi088) {
  if (bmi088 == NULL) {
    return HAL_ERROR;
  }

  uint8_t raw[6] = {0};
  HAL_StatusTypeDef status;

  /* 鈶?鍔犻€熷害璁★細鍦板潃 + 鍝戝瓧鑺?+ 6 瀛楄妭鏁版嵁 */
  status = BMI088_Read_Register(bmi088->spi_accel, BMI088_ACC_DATA, raw, 6, 1);
  if (status != HAL_OK) {
    return status;
  }
  for (uint8_t i = 0; i < 3; i++) {
    bmi088->data.accel[i] =
        (float)BMI088_To_Int16(&raw[i * 2]) * BMI088_ACC_SCALE_G * 9.80665f;
  }

  /* 鈶?闄€铻轰华锛氬湴鍧€ + 6 瀛楄妭鏁版嵁锛堟棤鍝戝瓧鑺傦級 */
  status = BMI088_Read_Register(bmi088->spi_gyro, BMI088_GYRO_DATA, raw, 6, 0);
  if (status != HAL_OK) {
    return status;
  }
  for (uint8_t i = 0; i < 3; i++) {
    /* 顺序：原始 LSB -> °/s（量程换算）-> ×trim（灵敏度补偿）-> rad/s -> 减零偏。
       注意 gyro_offset 是在 BMI088_Calibrate_Gyro 里对"已经乘过 trim 的 data.gyro"
       取平均得到的，所以改了 trim 之后必须重新上电（重新标定零偏），
       否则零偏会和 trim 对不上。 */
    bmi088->data.gyro[i] =
        (float)BMI088_To_Int16(&raw[i * 2]) * BMI088_GYRO_SCALE_DPS *
            bmi088->gyro_trim[i] * BMI088_DEG_TO_RAD -
        bmi088->gyro_offset[i];
  }

  /* 鈶?娓╁害锛?1 浣嶈ˉ鐮侊紝0.125鈩?LSB锛? 鐐?23鈩?*/
  uint8_t temp[2] = {0};
  status =
      BMI088_Read_Register(bmi088->spi_accel, BMI088_ACC_TEMP_MSB, temp, 2, 1);
  if (status != HAL_OK) {
    return status;
  }
  int16_t raw_temperature =
      (int16_t)(((uint16_t)temp[0] << 3) | (temp[1] >> 5));
  if ((raw_temperature & 0x0400) != 0) {
    raw_temperature -= 0x0800;
  }
  bmi088->data.temperature = 23.0f + 0.125f * (float)raw_temperature;

  bmi088->sample_count++;
  if (bmi088->callback != NULL) {
    bmi088->callback(bmi088);
  }
  return HAL_OK;
}

uint8_t BMI088_Calibrate_Gyro(BMI088_Instance *bmi088) {
  if (bmi088 == NULL) {
    return 0;
  }

  /* 鍏堟竻闆讹細閲嶅鏍囧畾鏃朵笉鑳藉湪涓婁竴娆＄殑 offset 涓婄户缁姞 */
  bmi088->gyro_calibrated = 0;
  for (uint8_t k = 0; k < 3; k++) {
    bmi088->gyro_offset[k] = 0.0f;
  }

  /* 分段取中位数，而不是 1000 帧直接平均。
   *
   * 原因：标定窗口有 0.6 s，如果这期间手碰了一下板子（或者桌子被撞），
   * 那几帧的大角速度会把直接平均出来的 offset 带偏，残余可能到 0.05~0.1 °/s。
   * 这个残余之后会被姿态的 ZARU 当成"真实零偏"吸收掉，
   * 结果就是 g_att_bias 从原来的 0.008 °/s 变成 0.07 °/s（用户看到的"0.1"）。
   * 虽然 ZARU 吸收后长时漂移仍然很小，但零偏估计值不好看，而且换方向转动时会差一点。
   *
   * 做法：把 1000 帧分成 20 段，每段求均值，再取 20 个段均值的**中位数**。
   * 只要被干扰的段不到一半，中位数就不受影响。代价是多 240 B 栈，只在上电跑一次。 */
  const uint32_t blocks = 20;
  const uint32_t per_block = BMI088_CALIB_SAMPLES / blocks;
  float block_mean[20][3];

  for (uint32_t b = 0; b < blocks; b++) {
    float sum[3] = {0.0f, 0.0f, 0.0f};

    for (uint32_t i = 0; i < per_block; i++) {
      if (BMI088_Read(bmi088) != HAL_OK) {
        return 0;
      }
      for (uint8_t k = 0; k < 3; k++) {
        sum[k] += bmi088->data.gyro[k];
      }
    }

    const float inv = 1.0f / (float)per_block;
    for (uint8_t k = 0; k < 3; k++) {
      block_mean[b][k] = sum[k] * inv;
    }
  }

  for (uint8_t k = 0; k < 3; k++) {
    float v[20];

    for (uint32_t b = 0; b < blocks; b++) {
      v[b] = block_mean[b][k];
    }

    /* 插入排序：只有 20 个元素，比调 qsort 划算 */
    for (uint32_t a = 1; a < blocks; a++) {
      const float key = v[a];
      int32_t j = (int32_t)a - 1;
      while (j >= 0 && v[j] > key) {
        v[j + 1] = v[j];
        j--;
      }
      v[j + 1] = key;
    }

    bmi088->gyro_offset[k] = 0.5f * (v[blocks / 2 - 1] + v[blocks / 2]);
  }
  bmi088->gyro_calibrated = 1;
  return 1;
}
