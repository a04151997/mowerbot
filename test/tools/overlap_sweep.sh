#!/usr/bin/env bash
# 重疊率規劃面掃描 (階段 38 後續 6；由 stage38_blade_sweep.sh 改來)：存檔地圖 stage29_lawn -> map_to_boundary -> f2c_server，不開 Gazebo
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT=${1:?用法: overlap_sweep.sh <輸出目錄> <重疊率 ...>}; shift
mkdir -p "$OUT"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-95}
MAP=$WS/src/mowerbot_bringup/maps/reference/stage29_lawn.yaml
F2C_ARGS=$(python3 -c '
from mowerbot_description import vehicle_geometry as v; from mowerbot_action.manager import WAYPOINT_SPACING as w
g=v.load(); print("-p headland_width:=%r -p rotation_swept_radius:=%r -p headland_min_margin:=%r -p waypoint_spacing:=%r" % (g.headland_width, g.rotation_swept_radius, v.HEADLAND_MIN_MARGIN, w))')
# 訂閱端要先起來：map_to_boundary 的 /f2c_boundary 只在收到 /map 時發一次 (volatile QoS)
python3 -u "$WS/test/tools/overlap_sweep_plan.py" "$OUT/overlap_sweep.csv" "$@" > "$OUT/blade_sweep.txt" 2>&1 & P=$!
for i in $(seq 1 120); do grep -q '^READY' "$OUT/blade_sweep.txt" 2>/dev/null && break; sleep 0.25; done
setsid ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" > "$OUT/map_server.log" 2>&1 & A=$!
sleep 2; ros2 run nav2_util lifecycle_bringup map_server > /dev/null 2>&1
setsid ros2 run mowerbot_action map_to_boundary > "$OUT/m2b.log" 2>&1 & B=$!
setsid ros2 run mowerbot_planner f2c_server --ros-args $F2C_ARGS > "$OUT/f2c.log" 2>&1 & C=$!
wait $P; cat "$OUT/blade_sweep.txt"
kill -TERM -- -$A -$B -$C 2>/dev/null; sleep 2; kill -KILL -- -$A -$B -$C 2>/dev/null
