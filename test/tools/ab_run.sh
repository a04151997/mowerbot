#!/usr/bin/env bash
# ==========================================================================
# 前後量測（A/B）：兩趟用同一張地圖跑完整覆蓋任務。量法見報告 25.3 節。
#
# 【兩條規則，違反的話兩趟不可比】
#
#  1. 兩趟必須用同一張地圖。
#     各自建圖的話，規劃出來的割草線條數就不同（23.3 節 40 vs 41 條），
#     E2b、總時間、段落數全部不可比。
#     這支腳本的做法：/map 由 nav2 map_server 發布存好的 .pgm，
#     slam_toolbox 改成定位模式只負責 map -> odom（它自己的 /map 改名到 /slam_map），
#     而且不開車建圖。原因（25.3 節）：定位模式會把即時掃描併進自己的 /map，
#     就算車子不動，兩趟的邊界也會不一樣（實測 78 vs 58 個頂點）。
#     跑完用 diff 確認兩趟的邊界與「📋 佇列」逐字相同，不同就不要比。
#
#  2. 對照組要 checkout 舊 commit，不要只 git stash。
#     改動一旦 commit 了，stash 只會收走還沒 commit 的東西，
#     跑出來的「對照組」其實還是新版程式（25.3 節就是這樣發現的）。
#     每次切換版本都要重新 colcon build，並確認 install/ 裡的檔案是對的版本。
#
# 【流程】
#
#   test/tools/ab_run.sh map <輸出目錄>          # 1. 建圖並存成 maps/<地圖名>
#   test/tools/ab_run.sh run <輸出目錄>/after    # 2. 目前版本跑一趟
#   git stash                                   # 3. 收起未 commit 的修改
#   git checkout <改動前的 commit>
#   colcon build --packages-select <改到的套件>
#   test/tools/ab_run.sh run <輸出目錄>/before   # 4. 舊版本同一張圖跑一趟
#   git checkout main && git stash pop           # 5. 回來並重建
#   colcon build --packages-select <改到的套件>
#
#   # 6. 確認兩趟同圖（必須沒有任何輸出）
#   diff <(grep -a '📋 佇列\|🎉 邊界' <before>/demo.log | sed 's/^.*\]: //') \
#        <(grep -a '📋 佇列\|🎉 邊界' <after>/demo.log  | sed 's/^.*\]: //')
#
#   # 7. 分析
#   python3 test/tools/segment_endpoint.py <dir>/run_traj.csv <dir>/manager.log
#   python3 test/tools/coverage_budget.py <dir>/run --log <dir>/manager.log
#
# 地圖名預設 ab_map，可用環境變數 AB_MAP 改。maps/ 裡的地圖檔不進版控。
# 世界預設 demo_lawn.world（run_demo.sh 的預設），可用環境變數 AB_WORLD 改（階段 28）。
# 額外的 demo launch 參數可用 AB_DEMO_ARGS 加在最後 (後面的覆蓋前面的；階段 38 後續 6 錄影用：rviz:=true)。
# 任務秒數上限預設 1500，可用 AB_LIMIT 改（階段 29：只為了抓規劃路徑時用短的）。
# ==========================================================================
set -u
MODE=${1:?用法: ab_run.sh map|run <輸出目錄>}
OUT=${2:?用法: ab_run.sh map|run <輸出目錄>}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MAP_NAME=${AB_MAP:-ab_map}
MAP=$WS/src/mowerbot_bringup/maps/$MAP_NAME
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
date +%s > "$OUT/t_start"

# 背景工作在非互動 shell 裡會忽略 SIGINT，所以收尾一律用 SIGTERM
# （run_demo.sh 有 trap TERM）。
setsid ./run_demo.sh rviz:=false hmi:=false world:="${AB_WORLD:-demo_lawn.world}" ${AB_DEMO_ARGS:-} > "$OUT/demo.log" 2>&1 &
DEMO=$!
LOC=; MS=
if [ "$MODE" = run ]; then
    # 等 demo 起建圖模式的 slam_toolbox，把它換掉
    P=
    for i in $(seq 1 120); do
        P=$(pgrep -f 'async_slam_toolbox_node.*--ros-args'); [ -n "$P" ] && break; sleep 0.25
    done
    echo "建圖模式 slam pid=$P" > "$OUT/swap.log"
    kill -INT "$P"
    for i in $(seq 1 40); do kill -0 "$P" 2>/dev/null || break; sleep 0.25; done
    if kill -0 "$P" 2>/dev/null; then
        echo "SIGINT 10 s 未退出，SIGKILL" >> "$OUT/swap.log"; kill -KILL "$P"; sleep 0.5
    fi
    echo "建圖模式已停止 $(date +%s.%N)" >> "$OUT/swap.log"

    # 定位模式：只負責 map -> odom。沒有 map_start_pose 的話 slam_toolbox 會報
    # "Map starting pose not specified"，車子一動地圖就跟著漂（25.3 節）。
    # 存圖時車子從 Gazebo 原點出發，所以起始位姿就是 (0, 0, 0)。
    setsid ros2 run slam_toolbox localization_slam_toolbox_node --ros-args \
        -r __node:=slam_toolbox -r map:=/slam_map -r map_metadata:=/slam_map_metadata \
        --params-file "$WS/install/mowerbot_bringup/share/mowerbot_bringup/config/localization_params.yaml" \
        -p use_sim_time:=true -p map_file_name:="$MAP" -p "map_start_pose:=[0.0, 0.0, 0.0]" \
        > "$OUT/loc.log" 2>&1 &
    LOC=$!
    # /map：存好的 .pgm，每一趟都是同一個檔案
    setsid ros2 run nav2_map_server map_server --ros-args \
        -p yaml_filename:="$MAP.yaml" -p use_sim_time:=true > "$OUT/map_server.log" 2>&1 &
    MS=$!
    sleep 2
    ros2 run nav2_util lifecycle_bringup map_server > "$OUT/map_server_lc.log" 2>&1
fi
sleep 40

if [ "$MODE" = map ]; then
    python3 -u test/tools/coverage_run.py --map-only > "$OUT/drive.log" 2>&1
    ros2 run mowerbot_bringup save_map.sh "$MAP_NAME" > "$OUT/save.log" 2>&1
else
    python3 -u test/tools/coverage_run.py "$OUT/run" "${AB_LIMIT:-1500}" --no-drive > "$OUT/coverage_run.log" 2>&1
fi

[ -n "$LOC" ] && kill -TERM -- "-$LOC" "-$MS"
kill -TERM "$DEMO"
wait

# manager 的 log（coverage_budget / segment_endpoint 要用）
ls -t ~/.ros/log/python3_*.log | while read -r f; do
    [ "$f" -nt "$OUT/t_start" ] && grep -aq '\[mower_manager\]' "$f" && { cp "$f" "$OUT/manager.log"; break; }
done
echo "完成：$OUT"
