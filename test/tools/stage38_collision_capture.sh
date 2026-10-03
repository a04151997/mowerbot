#!/usr/bin/env bash
# 階段 38：撞牆那一段的完整資料 (DWB 每個候選軌跡的 critic 分數、local_costmap、cmd_vel、odom、tf)。
# 與主基線 C1 同一組設定 (stage29_lawn、目前的 vehicle.yaml)，跑一趟 ab_run.sh，同時錄 bag。
# 只錄這份簡報需要的 topic，不錄相機。
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT=${1:?用法: stage38_collision_capture.sh <輸出目錄>}
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
( sleep 50
  setsid ros2 bag record -o "$OUT/bag" /evaluation /local_costmap/costmap /local_costmap/published_footprint \
      /cmd_vel /cmd_vel_nav /odom /tf /tf_static /local_plan /received_global_plan /mission_status \
      > "$OUT/bag_record.log" 2>&1 & echo $! > "$OUT/bag.pid" ) &
AB_MAP=stage29_lawn AB_WORLD=demo_lawn.world AB_LIMIT=240 \
    timeout -s TERM -k 60 900 bash test/tools/ab_run.sh run "$OUT/run" > "$OUT/ab_run.out" 2>&1
echo "ab_run rc=$?" >> "$OUT/ab_run.out"
P=$(cat "$OUT/bag.pid" 2>/dev/null); [ -n "$P" ] && kill -INT -- -"$P" 2>/dev/null; sleep 5
