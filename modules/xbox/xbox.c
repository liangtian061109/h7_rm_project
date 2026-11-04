#include "xbox.h"

#include <stddef.h>

/* 小端 int16。用较宽类型做补码转换，避免未对齐访问和实现相关的截断。 */
static int16_t Xbox_Read_I16(const uint8_t *p) {
  const uint16_t raw = (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));

  return (int16_t)((raw <= 32767) ? (int32_t)raw : (int32_t)raw - 65536);
}

bool Xbox_Parse(const uint8_t *data, uint16_t len, Xbox_State *out) {
  if (data == NULL || out == NULL) {
    return false;
  }
  if (len < XBOX_REPORT_SIZE || len > XBOX_TRANSFER_MAX) {
    return false;
  }
  /* 包头必须声明 20 字节标准区，否则不是 Xbox 输入报告 */
  if (data[0] != 0x00 || data[1] != XBOX_REPORT_SIZE) {
    return false;
  }

  Xbox_State s;
  s.buttons = (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
  s.lt = data[4];
  s.rt = data[5];
  s.lx = Xbox_Read_I16(&data[6]);
  s.ly = Xbox_Read_I16(&data[8]);
  s.rx = Xbox_Read_I16(&data[10]);
  s.ry = Xbox_Read_I16(&data[12]);

  *out = s;
  return true;
}

float Xbox_Stick_Normalize(int16_t raw, float deadzone) {
  float v = (float)raw / 32767.0f;

  if (v > 1.0f) {
    v = 1.0f;
  } else if (v < -1.0f) {
    v = -1.0f;
  }

  const float mag = (v < 0.0f) ? -v : v;

  if (mag <= deadzone) {
    return 0.0f;
  }

  /* 死区外重新映射到满量程：过死区瞬间从 0 开始，不会跳一下 */
  const float scaled = (mag - deadzone) / (1.0f - deadzone);

  return (v < 0.0f) ? -scaled : scaled;
}
