#!/usr/bin/env bash
# ==========================================================================
# 未觀測格（-1）在哪一層變成自由格（報告 29.1，只量測）
#
#   1. 建圖模式開車（與 ab_run.sh map 相同的開車段）-> 抓 slam_toolbox 的即時 /map
#   2. save_map.sh 存檔 -> 數 .pgm 的灰階值
#   3. nav2 map_server 讀回存檔（原本的 yaml）-> 抓它發的 /probe_map
#   4. 同一張 .pgm、只把 yaml 的 free_thresh 改成 0.196 -> 抓 /probe_map_196
#   5. 停掉建圖模式，改起定位模式載入同一份 posegraph -> 抓它發的 /map
#
# 用法: test/tools/unknown_probe.sh <輸出目錄> [世界檔，預設 demo_lawn_obstacle.world]
# ==========================================================================
set -u
OUT=${1:?用法: unknown_probe.sh <輸出目錄> [world]}
WORLD=${2:-demo_lawn_obstacle.world}
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
NAME=stage29_probe
MAP=$WS/src/mowerbot_bringup/maps/$NAME
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
SNAP="python3 -u test/tools/map_snapshot.py"

setsid ./run_demo.sh rviz:=false hmi:=false world:="$WORLD" > "$OUT/demo.log" 2>&1 &
DEMO=$!
sleep 40
python3 -u test/tools/coverage_run.py --map-only > "$OUT/drive.log" 2>&1
sleep 3

echo "===== 1. slam_toolbox 建圖模式的即時 /map（map_to_boundary 平常收到的）====="
$SNAP /map "$OUT/1_live.npz"
echo ""
echo "===== 2. save_map.sh 存檔 ====="
ros2 run mowerbot_bringup save_map.sh "$NAME" > "$OUT/save.log" 2>&1
grep -a "SaveMap_Response" "$OUT/save.log"
$SNAP --pgm "$MAP.yaml"
echo ""
echo "===== 3. map_server 讀回存檔（yaml 原樣）====="
setsid ros2 run nav2_map_server map_server --ros-args -r __node:=probe_ms -r map:=/probe_map \
    -p yaml_filename:="$MAP.yaml" -p use_sim_time:=true > "$OUT/ms1.log" 2>&1 &
MS1=$!
sleep 2; ros2 run nav2_util lifecycle_bringup probe_ms > /dev/null 2>&1
$SNAP /probe_map "$OUT/3_mapserver.npz"
echo ""
echo "===== 4. 同一張 .pgm，yaml 的 free_thresh 改成 0.196 ====="
sed 's/^free_thresh: .*/free_thresh: 0.196/' "$MAP.yaml" > "$OUT/probe_196.yaml"
sed -i "s#^image: .*#image: $MAP.pgm#" "$OUT/probe_196.yaml"
$SNAP --pgm "$OUT/probe_196.yaml" 2>/dev/null | sed -n '1p;3,9p' || true
setsid ros2 run nav2_map_server map_server --ros-args -r __node:=probe_ms2 -r map:=/probe_map_196 \
    -p yaml_filename:="$OUT/probe_196.yaml" -p use_sim_time:=true > "$OUT/ms2.log" 2>&1 &
MS2=$!
sleep 2; ros2 run nav2_util lifecycle_bringup probe_ms2 > /dev/null 2>&1
$SNAP /probe_map_196 "$OUT/4_mapserver_196.npz"
echo ""
echo "===== 5. 定位模式（localization_slam_toolbox_node 載入同一份 posegraph）的 /map ====="
P=$(pgrep -f 'async_slam_toolbox_node.*--ros-args'); kill -INT "$P"
for i in $(seq 1 40); do kill -0 "$P" 2>/dev/null || break; sleep 0.25; done
kill -0 "$P" 2>/dev/null && kill -KILL "$P"
setsid ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
    -r __node:=slam_toolbox \
    --params-file "$WS/install/mowerbot_bringup/share/mowerbot_bringup/config/localization_params.yaml" \
    -p use_sim_time:=true -p map_file_name:="$MAP" -p "map_start_pose:=[0.0, 0.0, 0.0]" \
    > "$OUT/loc.log" 2>&1 &
LOC=$!
sleep 15
$SNAP /map "$OUT/5_localization.npz"

kill -TERM -- "-$MS1" "-$MS2" "-$LOC" 2>/dev/null
kill -TERM "$DEMO"
wait
echo "完成：$OUT"
