#include "rc_task.h"

#include <stdint.h>

#include "chassis.h"
#include "usb_host.h"
#include "xbox.h"

#define RC_TASK_PERIOD_MS 1

/* 摇杆死区。冰原狼实测静止时抖动在 ±300 以内（约 0.01），
 * 给 0.08 足够滤掉，又不会让微操变迟钝。 */
#define RC_STICK_DEADZONE 0.08f

/* 满摇杆对应的速度。底盘上限：轮子 482 rpm ≈ 1.9 m/s，
 * 但功率只有 45W，实际跑 1.2~1.5 m/s 就够，再高加速会很肉。 */
float g_rc_max_vx_mm_s = 1200.0f;
float g_rc_max_vy_mm_s = 800.0f;
float g_rc_max_wz_rad_s = 3.5f;

volatile uint8_t g_rc_enable = 0;
volatile uint8_t g_rc_online = 0;

TaskHandle_t g_rc_task_handle = NULL;

/* 手柄 → 车体速度。
 *
 * 符号说明（实测确认，反了改这里）：
 *   左摇杆上推 → vx 正（车头方向）
 *   左摇杆左推 → vy 正（车身左侧）
 *   右摇杆右推 → wz 负（顺时针）
 * Y 轴向上为正、X 轴向右为正，所以后两项要取反。 */
static void RC_Map_To_Chassis(const Xbox_State *xb) {
  const float ly = Xbox_Stick_Normalize(xb->ly, RC_STICK_DEADZONE);
  const float lx = Xbox_Stick_Normalize(xb->lx, RC_STICK_DEADZONE);
  const float rx = Xbox_Stick_Normalize(xb->rx, RC_STICK_DEADZONE);

  g_chassis_vx_mm_s = ly * g_rc_max_vx_mm_s;
  g_chassis_vy_mm_s = -lx * g_rc_max_vy_mm_s;
  g_chassis_wz_rad_s = -rx * g_rc_max_wz_rad_s;
}

void RC_Task(void *argument) {
  (void)argument;

  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    /* 推进 Host 状态机 + 非阻塞收报告。
     * 枚举阶段内部会 vTaskDelay，所以这个任务不能是控制任务。 */
    Usb_Host_Process();

    g_rc_online = Usb_Host_Is_Ready();

    /* 手柄没插时不碰速度，保留调试器里手动改的值。
     * 插上但没使能也不碰，避免误动作。 */
    if (g_rc_online != 0 && g_rc_enable != 0) {
      Xbox_State xb;

      Usb_Host_Get_Xbox(&xb);
      RC_Map_To_Chassis(&xb);
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RC_TASK_PERIOD_MS));
  }
}
