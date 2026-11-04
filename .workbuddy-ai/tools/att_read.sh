#!/usr/bin/env bash
# 通过 EmberProbe 读姿态实时变量（需要 VS Code 里 EmberProbe 已启用 Agent 访问）
#
#   bash att_read.sh          读一次
#   bash att_read.sh 40       采 40 个点（250ms 一个 = 10 秒）看趋势
#
# 注意：g_att_zero_progress / g_att_dt_clamped / g_att_dt_lost_s / g_att_rejected
# 只有刷了新版固件才有；旧固件会报 VARIABLE_NOT_FOUND，不影响其它变量。
#
# dt_lost_s 是关键诊断：它涨了说明主循环被阻塞过（调试器停核等），
# 那段时间的角度真丢了，转角必然偏小。
NODE="C:/Users/30710/.workbuddy-ai/binaries/node/versions/22.22.2-6/node.exe"
SK="C:/Users/30710/.vscode/extensions/bakesheep.emberprobe-0.9.0/skills/mcu-variables/scripts/read.js"
WS="d:/stm32halproject/stm32h723/rm_project"
VARS="g_att_roll_deg,g_att_pitch_deg,g_att_yaw_deg,g_att_zero_ready,g_att_zero_progress,g_att_rest,g_att_bias[2],g_yaw_rate_deg_s,g_att_dt_clamped,g_att_dt_lost_s,g_att_rejected"

if [ -n "$1" ]; then
  "$NODE" "$SK" --workspace "$WS" --variables "$VARS" --trend --interval 250 --count "$1"
else
  "$NODE" "$SK" --workspace "$WS" --variables "$VARS"
fi
