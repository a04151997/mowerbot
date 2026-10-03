#!/usr/bin/env bash
# 周邊環繞有效長度探測 (階段 38，smoke_test B5)：存檔地圖 -> map_to_boundary -> f2c_server -> manager 的切段函式
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP=${1:-$WS/src/mowerbot_bringup/maps/reference/stage29_lawn.yaml}
OUT=${2:-/tmp}
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
# 預設用 vehicle_geometry 推導的 headland；PERIMETER_F2C_ARGS 可覆寫 (例如拿舊 headland 0.70 做對照)
F2C_ARGS=${PERIMETER_F2C_ARGS:-$(python3 -c '
from mowerbot_description import vehicle_geometry as v; from mowerbot_action.manager import WAYPOINT_SPACING as w
g=v.load(); print("-p headland_width:=%r -p rotation_swept_radius:=%r -p headland_min_margin:=%r -p waypoint_spacing:=%r" % (g.headland_width, g.rotation_swept_radius, v.HEADLAND_MIN_MARGIN, w))')}
# 訂閱端先起：/f2c_boundary 只在收到 /map 時發一次
PERIMETER_DUMP_DIR="$OUT" python3 -u "$WS/test/tools/perimeter_probe.py" > "$OUT/perimeter_probe.out" 2>&1 & P=$!
for i in $(seq 1 120); do grep -q '^READY' "$OUT/perimeter_probe.out" 2>/dev/null && break; sleep 0.25; done
setsid ros2 run nav2_map_server map_server --ros-args -p yaml_filename:="$MAP" > "$OUT/perimeter_map_server.log" 2>&1 & A=$!
sleep 2; ros2 run nav2_util lifecycle_bringup map_server > /dev/null 2>&1
setsid ros2 run mowerbot_action map_to_boundary > "$OUT/perimeter_m2b.log" 2>&1 & B=$!
setsid ros2 run mowerbot_planner f2c_server --ros-args $F2C_ARGS > "$OUT/perimeter_f2c.log" 2>&1 & C=$!
wait $P; rc=$?
kill -TERM -- -$A -$B -$C 2>/dev/null; sleep 2; kill -KILL -- -$A -$B -$C 2>/dev/null
grep -E "^[A-Z_]+=" "$OUT/perimeter_probe.out"
exit $rc
