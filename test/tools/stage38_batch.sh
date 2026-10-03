#!/usr/bin/env bash
# ==========================================================================
# 階段 38 無人看顧批次（報告 38 節）。依序：
#
#   【F】腳輪摩擦掃描：腳輪 mu1/mu2 = 現況 0.1/0.05、0.1、0.3、0.5、1.0，各量原地旋轉角速度
#   【V】光達視野：有 / 沒有引擎方塊各量一次 /scan 自身遮擋；
#        各建一次圖 (ab_run.sh map) 並同時取樣 local_costmap，比較地圖內部假障礙物
#   【C】主基線：stage29_lawn 同圖，blade_width 0.50、blade_offset_x 0.375、mass 73.4，COV_N 趟
#   【S】blade_width 敏感度：0.45、0.55 各 1 趟
#   【B】blade_offset_x 敏感度：0.35、0.40 各 1 趟
#   【M】mass 探測：85 kg 1 趟 (其餘同主基線)
#   【A】完整套件 1 趟
#
# 規則與 stage32_overnight.sh 相同：每一趟都有超時上限，超時就記錄、殺乾淨、繼續下一趟；
# 每趟之間清殘留行程；每趟結束立刻把結果追加到 $ROOT/progress.tsv；不錄 bag；
# 每趟開始前查磁碟。
# 暫時改動 vehicle.yaml / car_wheels.xacro / car_base.xacro 的步驟，開始前先備份，
# 結束 (含中斷) 時一律還原 (trap EXIT)，並在 progress.tsv 記錄還原結果。
#
# 用法：
#   setsid nohup test/tools/stage38_batch.sh <輸出根目錄> > <根目錄>/batch.log 2>&1 &
# 只跑某幾項：STEPS="F V"（預設全部）
# ==========================================================================
set -u
ROOT=${1:?用法: stage38_batch.sh <輸出根目錄>}
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STEPS=${STEPS:-"F V C S B M A"}
COV_N=${COV_N:-5}
RUN_TIMEOUT=${RUN_TIMEOUT:-2400}      # 一趟任務 40 分鐘
SUITE_TIMEOUT=${SUITE_TIMEOUT:-2700}  # 一趟套件 45 分鐘
MIN_FREE_GB=${MIN_FREE_GB:-10}
PROG=$ROOT/progress.tsv
VY=$WS/src/mowerbot_description/config/vehicle.yaml
XW=$WS/src/mowerbot_description/urdf/car_wheels.xacro
XB=$WS/src/mowerbot_description/urdf/car_base.xacro
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"

log() { echo "[$(date '+%m/%d %H:%M:%S')] $*"; }
row() { printf '%s\n' "$(IFS=$'\t'; echo "$*")" >> "$PROG"; }
[ -f "$PROG" ] || row 項目 趟 開始 結束 秒數 結束方式 目錄 摘要

mkdir -p "$ROOT/orig"
cp "$VY" "$ROOT/orig/vehicle.yaml"; cp "$XW" "$ROOT/orig/car_wheels.xacro"; cp "$XB" "$ROOT/orig/car_base.xacro"
restore() {
    cp "$ROOT/orig/vehicle.yaml" "$VY"; cp "$ROOT/orig/car_wheels.xacro" "$XW"; cp "$ROOT/orig/car_base.xacro" "$XB"
    if cmp -s "$ROOT/orig/vehicle.yaml" "$VY" && cmp -s "$ROOT/orig/car_wheels.xacro" "$XW" \
       && cmp -s "$ROOT/orig/car_base.xacro" "$XB"; then echo ok; else echo 不一致; fi
}
trap 'r=$(restore); row 還原 - - - - "$r" - "vehicle.yaml / car_wheels.xacro / car_base.xacro"; log "結束時還原：$r"' EXIT

disk_ok() {
    local free; free=$(df -BG --output=avail "$WS" | tail -1 | tr -dc '0-9')
    log "磁碟剩 ${free} GB"
    if [ "$free" -lt "$MIN_FREE_GB" ]; then
        log "磁碟剩不到 ${MIN_FREE_GB} GB，停止批次"; row 停止 - - - - 磁碟不足 - "剩 ${free} GB"
        return 1
    fi
}

# ---- 清殘留：與 stage32_overnight.sh 相同 ----
ancestors() { local p=$$ o=""; while [ -n "$p" ] && [ "$p" != 0 ] && [ "$p" != 1 ]; do
    o="$o $p"; p=$(awk '{print $4}' /proc/$p/stat 2>/dev/null); done; echo "$o"; }
stale() {
    local ex d pid; ex=" $(ancestors) "
    for d in /proc/[0-9]*; do
        pid=${d#/proc/}; case "$ex" in *" $pid "*) continue;; esac
        grep -qa -- "--ros-args" "$d/cmdline" 2>/dev/null || continue
        if grep -qa -- "$WS/install" "$d/cmdline" 2>/dev/null ||
           grep -qa -- "$WS/install" "$d/environ" 2>/dev/null; then echo "$pid"; fi
    done
    for pid in $(pgrep -x gzserver) $(pgrep -x gzclient) $(pgrep -f 'ros2 bag record') \
               $(pgrep -f 'run_demo.sh') $(pgrep -f 'tools/coverage_run.py') $(pgrep -f 'tools/ab_run.sh') \
               $(pgrep -f 'tools/costmap_probe.py') $(pgrep -f 'tools/scan_fov.py') \
               $(pgrep -f 'stage38/spin.py') $(pgrep -f 'gazebo.launch.py'); do
        case "$ex" in *" $pid "*) continue;; esac; echo "$pid"; done
}
cleanup() {
    local p n=0
    for p in $(stale | sort -un); do kill -TERM "$p" 2>/dev/null && n=$((n+1)); done
    sleep 3
    for p in $(stale | sort -un); do kill -KILL "$p" 2>/dev/null; done
    ros2 daemon stop > /dev/null 2>&1
    sleep 3
    log "清殘留：TERM $n 個；剩下 $(stale | wc -l) 個"
}
set_yaml() {   # set_yaml <鍵> <值>：只改 vehicle.yaml 頂層那一行的數值
    python3 - "$VY" "$1" "$2" <<'PY'
import re, sys
p, k, v = sys.argv[1:]
s = open(p).read()
s2, n = re.subn(r'^(%s:\s*)\S+' % re.escape(k), r'\g<1>%s' % v, s, count=1, flags=re.M)
assert n == 1, k
open(p, 'w').write(s2)
PY
}
suffix() { python3 -c 'from mowerbot_description import vehicle_geometry as v; print(v.load().file_suffix())'; }

# 一趟覆蓋任務 + 全部分析。cov_run <項目> <趟> <目錄>
cov_run() {
    local item=$1 i=$2 d=$3 t0 t1 rc how s sfx
    disk_ok || exit 1
    log "【$item】第 $i 趟 -> $d ($(python3 -c 'from mowerbot_description import vehicle_geometry as v; g=v.load(); print("blade_width=%.2f blade_offset_x=%.3f mass=%.1f headland=%.2f" % (g.blade_width, g.blade_offset_x, g.mass, g.headland_width))'))"
    cleanup
    t0=$(date +%s)
    AB_MAP=stage29_lawn AB_WORLD=demo_lawn.world \
        timeout -s TERM -k 60 "$RUN_TIMEOUT" bash test/tools/ab_run.sh run "$d" > /dev/null 2>&1
    rc=$?; t1=$(date +%s)
    case $rc in 0) how=完成;; 124|137) how=超時;; *) how="rc=$rc";; esac
    cleanup
    sfx=$(suffix)
    s="沒有軌跡或 manager.log"
    if [ -f "$d/run_traj.csv" ] && [ -f "$d/manager.log" ]; then
        W="$WS/src/mowerbot_bringup/worlds/demo_lawn.world"
        python3 test/tools/coverage_budget.py "$d/run" --log "$d/manager.log" --world "$W" \
            --traj base --heatmap "$d/heatmap_base$sfx.png" > "$d/budget_base$sfx.txt" 2>&1
        python3 test/tools/coverage_budget.py "$d/run" --log "$d/manager.log" --world "$W" \
            --traj blade --heatmap "$d/heatmap_blade$sfx.png" > "$d/budget_blade$sfx.txt" 2>&1
        # 與階段 32 對照用：E1 帶寬用舊的車體半寬 0.34 (量測方式的差異單獨拆出來)
        python3 test/tools/coverage_budget.py "$d/run" --log "$d/manager.log" --world "$W" \
            --traj base --body-half 0.34 > "$d/budget_base_oldE1$sfx.txt" 2>&1
        python3 test/tools/swath_overshoot.py "$d" > "$d/overshoot$sfx.txt" 2>&1
        python3 test/tools/segment_endpoint.py "$d/run_traj.csv" "$d/manager.log" > "$d/endpoint$sfx.txt" 2>&1
        python3 test/tools/corner_offset.py "$d" > "$d/corner$sfx.txt" 2>&1
        s="$(grep -a '任務結束' "$d/coverage_run.log" | tr -d '\n') | base: $(grep -a '覆蓋率' "$d/budget_base$sfx.txt" | tr -s ' ' | tr '\n' ' ') | blade: $(grep -a '覆蓋率' "$d/budget_blade$sfx.txt" | tr -s ' ' | tr '\n' ' ')"
    fi
    row "$item" "$i" "$t0" "$t1" $((t1-t0)) "$how" "$d" "$s"
    log "【$item】第 $i 趟 $how $((t1-t0))s：$s"
}

df -h "$WS" | tail -1 | sed 's/^/磁碟 開始: /'

# ================= 【F】腳輪摩擦掃描 =================
if [[ " $STEPS " == *" F "* ]]; then
    mkdir -p "$ROOT/friction"
    for case in "0.1 0.05" "0.1 0.1" "0.3 0.3" "0.5 0.5" "1.0 1.0"; do
        set -- $case
        cleanup
        python3 - "$XW" "$1" "$2" <<'PY'
import re, sys
p, m1, m2 = sys.argv[1:]; s = open(p).read()
i = s.index('name="add_caster"'); head, tail = s[:i], s[i:]
tail = re.sub(r'<mu1>[^<]*</mu1>', '<mu1>%s</mu1>' % m1, tail, 1)
tail = re.sub(r'<mu2>[^<]*</mu2>', '<mu2>%s</mu2>' % m2, tail, 1)
open(p, 'w').write(head + tail)
PY
        t0=$(date +%s)
        export ROS_DOMAIN_ID=93
        setsid ros2 launch mowerbot_bringup gazebo.launch.py gui:=false world:=demo_lawn.world \
            > "$ROOT/friction/gz_$1_$2.log" 2>&1 &
        sleep 15
        r=$(timeout 90 python3 test/logs/stage38/spin.py 2>&1 | tr '\n' ' ')
        unset ROS_DOMAIN_ID
        t1=$(date +%s)
        cleanup
        row F "mu=$1/$2" "$t0" "$t1" $((t1-t0)) - "$ROOT/friction" "$r"
        log "【F】腳輪 mu1=$1 mu2=$2：$r"
    done
    cp "$ROOT/orig/car_wheels.xacro" "$XW"
fi

# ================= 【V】光達視野與建圖 =================
if [[ " $STEPS " == *" V "* ]]; then
    mkdir -p "$ROOT/fov"
    for variant in engine noengine; do
        cp "$ROOT/orig/car_base.xacro" "$XB"
        if [ "$variant" = noengine ]; then
            sed -i 's|^\(\s*\)<xacro:include filename="car_engine.xacro"/>|\1<!-- 階段 38 批次【V】暫時拿掉引擎 --><!-- <xacro:include filename="car_engine.xacro"/> -->|' "$XB"
            grep -q '暫時拿掉引擎' "$XB" || { log "【V】拿掉引擎失敗"; row V "$variant" - - - 失敗 - "sed 沒有改到 car_base.xacro"; continue; }
        fi
        # 1. 靜止的 /scan 自身遮擋
        cleanup
        export ROS_DOMAIN_ID=93
        setsid ros2 launch mowerbot_bringup gazebo.launch.py gui:=false world:=demo_lawn.world \
            > "$ROOT/fov/gz_$variant.log" 2>&1 &
        sleep 15
        timeout 30 python3 test/tools/scan_fov.py 4 > "$ROOT/fov/scan_$variant.txt" 2>&1
        unset ROS_DOMAIN_ID
        cleanup
        row V "scan_$variant" - - - - "$ROOT/fov" "$(head -3 "$ROOT/fov/scan_$variant.txt" | tr '\n' ' ')"
        log "【V】$variant /scan：$(head -3 "$ROOT/fov/scan_$variant.txt" | tr '\n' ' ')"
        # 2. 建圖 (ab_run.sh map 會開整套 demo + 開車繞場) + local_costmap 取樣
        disk_ok || exit 1
        d="$ROOT/fov/map_$variant"
        t0=$(date +%s)
        ( sleep 45; timeout 150 python3 test/tools/costmap_probe.py 140 "$d/costmap.csv" > "$d/costmap.txt" 2>&1 ) &
        mkdir -p "$d"
        AB_MAP="stage38_map_$variant" AB_WORLD=demo_lawn.world \
            timeout -s TERM -k 60 900 bash test/tools/ab_run.sh map "$d" > /dev/null 2>&1
        rc=$?; t1=$(date +%s)
        wait
        cleanup
        python3 test/tools/map_interior.py "$WS/src/mowerbot_bringup/maps/stage38_map_$variant.yaml" \
            > "$d/interior.txt" 2>&1
        row V "map_$variant" "$t0" "$t1" $((t1-t0)) "rc=$rc" "$d" \
            "$(cat "$d/interior.txt" | tr '\n' ' ') | $(cat "$d/costmap.txt" | tr '\n' ' ')"
        log "【V】$variant 建圖 rc=$rc：$(cat "$d/interior.txt") | $(cat "$d/costmap.txt")"
    done
    cp "$ROOT/orig/car_base.xacro" "$XB"
fi

# ================= 【C】主基線 =================
if [[ " $STEPS " == *" C "* ]]; then
    cp "$ROOT/orig/vehicle.yaml" "$VY"
    for i in $(seq 1 "$COV_N"); do cov_run C "$i" "$ROOT/cov/run$i"; done
fi

# ================= 【S】blade_width 敏感度 =================
if [[ " $STEPS " == *" S "* ]]; then
    for bw in 0.45 0.55; do
        cp "$ROOT/orig/vehicle.yaml" "$VY"; set_yaml blade_width "$bw"
        cov_run S "bw$bw" "$ROOT/sens/bw$bw"
    done
    cp "$ROOT/orig/vehicle.yaml" "$VY"
fi

# ================= 【B】blade_offset_x 敏感度 =================
if [[ " $STEPS " == *" B "* ]]; then
    for bo in 0.35 0.40; do
        cp "$ROOT/orig/vehicle.yaml" "$VY"; set_yaml blade_offset_x "$bo"
        cov_run B "bo$bo" "$ROOT/sens/bo$bo"
    done
    cp "$ROOT/orig/vehicle.yaml" "$VY"
fi

# ================= 【M】mass 探測 =================
if [[ " $STEPS " == *" M "* ]]; then
    cp "$ROOT/orig/vehicle.yaml" "$VY"; set_yaml mass 85
    cov_run M mass85 "$ROOT/mass/m85"
    cp "$ROOT/orig/vehicle.yaml" "$VY"
fi

# ================= 【A】完整套件 =================
if [[ " $STEPS " == *" A "* ]]; then
    cp "$ROOT/orig/vehicle.yaml" "$VY"
    disk_ok || exit 1
    log "【A】完整套件"
    cleanup
    before=$(ls -t test/logs | head -1)
    t0=$(date +%s)
    timeout -s TERM -k 60 "$SUITE_TIMEOUT" python3 -u test/smoke_test.py > "$ROOT/suite$(suffix).out" 2>&1
    rc=$?; t1=$(date +%s)
    case $rc in 124|137) how=超時;; *) how="rc=$rc";; esac
    cleanup
    after=$(ls -t test/logs | grep '^20' | head -1)
    s=$(grep -a "總計" "$ROOT/suite$(suffix).out" | tr -s ' ')
    s="$s $(grep -a '^\[FAIL\]\|^\[SKIP\]\|^\[FIXTURE\]' "$ROOT/suite$(suffix).out" | cut -c1-80 | tr '\n' ';')"
    row A 1 "$t0" "$t1" $((t1-t0)) "$how" "test/logs/$after" "$s"
    log "【A】$how $((t1-t0))s：$s"
fi

df -h "$WS" | tail -1 | sed 's/^/磁碟 結束: /'
log "全部結束"
