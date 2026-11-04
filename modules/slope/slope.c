#include "slope.h"

#include <stddef.h> /* NULL */

void Slope_Init(Slope_Instance *s, float inc, float dec, float init_out) {
  if (s == NULL) {
    return;
  }

  s->inc = inc;
  s->dec = dec;
  s->out = init_out;
}

void Slope_Reset(Slope_Instance *s, float value) {
  if (s == NULL) {
    return;
  }

  s->out = value;
}

float Slope_Update(Slope_Instance *s, float target, float real) {
  if (s == NULL) {
    return target;
  }

  if (s->out < target) {
    /* 往上升：走一步，不越过目标 */
    s->out += s->inc;
    if (s->out > target) {
      s->out = target;
    }

    /* 真实值优先：实际已经跑在前面就跟上，别让 PID 倒拉 */
    if (real > s->out && real < target) {
      s->out = real;
    }
  } else if (s->out > target) {
    /* 往下降 */
    s->out -= s->dec;
    if (s->out < target) {
      s->out = target;
    }

    if (real < s->out && real > target) {
      s->out = real;
    }
  }
  /* out == target 时不动 */

  return s->out;
}
