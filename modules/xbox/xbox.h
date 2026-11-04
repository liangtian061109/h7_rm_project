#ifndef XBOX_H
#define XBOX_H

#include <stdbool.h>
#include <stdint.h>

/* Xbox 360 兼容手柄输入报告解析。
 *
 * 冰原狼（飞智）等第三方接收器走的是 Xbox 360 私有协议，不是标准 HID：
 * 报告以 00 14 开头，14 是十六进制的 20，表示标准区长度。
 * 实际 USB 传输可能 32 字节，后段是厂商扩展，不解析。
 *
 * 布局：
 *   [0]      0x00
 *   [1]      0x14 = 20
 *   [2..3]   buttons  16 位位图，小端
 *   [4]      LT       0~255
 *   [5]      RT       0~255
 *   [6..7]   LX       int16 小端
 *   [8..9]   LY
 *   [10..11] RX
 *   [12..13] RY
 *
 * 摇杆范围 -32768~32767，Y 轴向上为正。 */

#define XBOX_REPORT_SIZE 20 /* 标准区长度 */
#define XBOX_TRANSFER_MAX 64

/* 按键位。注意 0~3 是十字键，不是摇杆。 */
#define XBOX_BTN_UP (1 << 0)
#define XBOX_BTN_DOWN (1 << 1)
#define XBOX_BTN_LEFT (1 << 2)
#define XBOX_BTN_RIGHT (1 << 3)
#define XBOX_BTN_START (1 << 4)
#define XBOX_BTN_BACK (1 << 5)
#define XBOX_BTN_LS (1 << 6) /* 左摇杆按下 */
#define XBOX_BTN_RS (1 << 7) /* 右摇杆按下 */
#define XBOX_BTN_LB (1 << 8)
#define XBOX_BTN_RB (1 << 9)
#define XBOX_BTN_GUIDE (1 << 10)
#define XBOX_BTN_A (1 << 12)
#define XBOX_BTN_B (1 << 13)
#define XBOX_BTN_X (1 << 14)
#define XBOX_BTN_Y (1 << 15)

typedef struct {
  int16_t lx;
  int16_t ly;
  int16_t rx;
  int16_t ry;
  uint16_t buttons;
  uint8_t lt;
  uint8_t rt;
} Xbox_State;

/* 解析一帧报告。长度不在 20~64 或包头不对时返回 false，不改 *out。 */
bool Xbox_Parse(const uint8_t *data, uint16_t len, Xbox_State *out);

/* 摇杆原始值归一化到 -1.0~1.0，并套死区。
 * 死区外重新映射到满量程，避免过死区时速度跳一下。 */
float Xbox_Stick_Normalize(int16_t raw, float deadzone);

#endif
