#ifndef SLOPE_H
#define SLOPE_H

/* 目标斜坡：把阶跃目标变成"每周期走一小步"，避免 PID 瞬间饱和。
 *
 * 加减速分开：加速慢防打滑，减速快刹车才干脆。
 * 真实值优先：实际已经跑在规划值前面时，规划值直接跟上，不按步长硬追。
 * 否则 PID 会把"目标 51、实际 70"当超速，反向制动把人拽回来。 */

typedef struct {
  float out; /* 当前规划值，也是输出 */
  float inc; /* 每周期最大增量（正数） */
  float dec; /* 每周期最大减量（正数） */
} Slope_Instance;

/* inc/dec 单位是"每周期"，和调用周期绑定：1 kHz 下 1.5f = 1500 rpm/s。 */
void Slope_Init(Slope_Instance *s, float inc, float dec, float init_out);

/* 每周期调一次，返回本周期该给 PID 的目标值。real 传当前实际值。 */
float Slope_Update(Slope_Instance *s, float target, float real);

/* 掉线恢复、模式切换时用，避免规划值停在旧目标上。 */
void Slope_Reset(Slope_Instance *s, float value);

#endif
