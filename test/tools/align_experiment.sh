#!/usr/bin/env bash
# 對正控制器實驗 (階段 38 決定 1-A)：不改 repo 的 nav2_params.yaml，產生一份實驗用參數檔
# (現有內容；AlignController 只寫要實驗的鍵，其餘由 navigation.launch.py 注入，align_goal_checker 的 yaw 容忍改成 TOL)。
# 用法：align_experiment.sh <yaw 容忍 rad> <輸出目錄> [次數 5] [轉角 deg 171]
set -u
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
TOL=${1:?yaw 容忍}; OUT=${2:?輸出目錄}; N=${3:-5}; TURN=${4:-171}
# MIN_SPEED_THETA (選用)：只設在 AlignController 上的 DWB min_speed_theta (階段 38 後續 4 的掃描)
MST=${MIN_SPEED_THETA:-}
# ALIGN_EXTRA (選用，只用於實驗)：額外寫進 AlignController 的鍵，例 "min_speed_xy=0.01"
EXTRA=${ALIGN_EXTRA:-}
mkdir -p "$OUT"; OUT="$(cd "$OUT" && pwd)"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-97}
TAG="tol$TOL${MST:+_mst$MST}${EXTRA:+_$(echo $EXTRA | tr " =" "_-")}"
PARAMS="$OUT/nav2_align_$TAG.yaml"
python3 - "$WS/src/mowerbot_bringup/config/nav2_params.yaml" "$PARAMS" "$TOL" "$MST" "$EXTRA" <<'PY'
import sys, copy, yaml
src, dst, tol, mst, extra = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4], sys.argv[5]
d = yaml.safe_load(open(src))
cs = d['controller_server']['ros__parameters']
# AlignController 由 navigation.launch.py 從 FollowPath 注入 (只管權重 + max_vel_x + xy_goal_tolerance)；
# 這裡只寫要實驗的鍵。注入不歸它管的鍵不會被蓋掉，啟動 log 會印「保留參數檔的設定」(階段 38 後續 5)。
if mst:
    cs.setdefault('AlignController', {})['min_speed_theta'] = float(mst)
for kv in extra.split():
    k, v = kv.split('=')
    cs.setdefault('AlignController', {})[k] = float(v)
cs['align_goal_checker']['yaw_goal_tolerance'] = tol
yaml.safe_dump(d, open(dst, 'w'), allow_unicode=True, sort_keys=False)
PY
# C1 割草線 1 的實際起點 (stage38/batch2/cov/run1)；起始朝向讓「目標 = 往東」時差 TURN 度
YAW0=$(python3 -c "import math;print(math.radians(-$TURN))")
setsid ros2 launch "$WS/test/tools/align_experiment.launch.py" x:=-3.08 y:=-4.48 yaw:=$YAW0 \
    world:=demo_lawn.world params_file:="$PARAMS" > "$OUT/launch_$TAG.log" 2>&1 &
L=$!
for i in $(seq 1 60); do
    s=$(ros2 lifecycle get /controller_server --no-daemon 2>/dev/null | tail -1)
    case "$s" in active*) break;; esac; sleep 2
done
echo "controller_server: $s"
timeout 600 python3 -u "$WS/test/tools/align_probe.py" "$N" "$TURN" "$OUT/align_$TAG.jsonl"
kill -TERM -- -$L 2>/dev/null; sleep 5; kill -KILL -- -$L 2>/dev/null
