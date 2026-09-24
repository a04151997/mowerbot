#!/usr/bin/env bash
# ==========================================================================
# 階段 28 無人看顧批次（報告 28 節）。依序：
#
#   【1】demo_lawn、存好的地圖 stage25_ab，跑 COV_N 趟完整任務，每趟做拆帳
#   【3】demo_lawn_obstacle：先建一張圖 stage28_obst，再用它跑 OBST_N 趟
#   【4】完整套件連跑 SUITE_N 趟（不加任何重試）
#
# 每一趟都有超時上限；超時就記錄、殺乾淨、繼續下一趟。
# 每趟之間清殘留行程（規則與 run_demo.sh 的 kill_stale 相同）。
# 每趟結束立刻把結果追加到 $ROOT/progress.tsv，掛掉也有資料。
#
# 用法：
#   setsid nohup test/tools/stage28_overnight.sh <輸出根目錄> > <根目錄>/batch.log 2>&1 &
# 只跑某幾項：STEPS="1 3 4"（預設全部）
# ==========================================================================
set -u
ROOT=${1:?用法: stage28_overnight.sh <輸出根目錄>}
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STEPS=${STEPS:-"1 3 4"}
COV_N=${COV_N:-5}
OBST_N=${OBST_N:-3}
SUITE_N=${SUITE_N:-6}
RUN_TIMEOUT=${RUN_TIMEOUT:-2400}      # 一趟任務 40 分鐘
SUITE_TIMEOUT=${SUITE_TIMEOUT:-2700}  # 一趟套件 45 分鐘
PROG=$ROOT/progress.tsv
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"

log() { echo "[$(date '+%m/%d %H:%M:%S')] $*"; }
row() { printf '%s\n' "$(IFS=$'\t'; echo "$*")" >> "$PROG"; }
[ -f "$PROG" ] || row 項目 趟 開始 結束 秒數 結束方式 目錄 摘要

# ---- 清殘留：與 run_demo.sh kill_stale 同樣的兩條規則 ----
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
               $(pgrep -f 'run_demo.sh') $(pgrep -f 'tools/coverage_run.py') $(pgrep -f 'tools/ab_run.sh'); do
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

# ---- 跑一趟 ab_run.sh（有超時）----
ab_once() {   # $1=項目 $2=趟 $3=輸出目錄 $4=map|run
    local t0 t1 rc how
    cleanup
    t0=$(date +%s)
    # 不加 setsid：timeout 會對整個 process group 送訊號（coverage_run.py 也在裡面）；
    # ab_run.sh 用 setsid 起的 demo / 定位 / map_server 逃出了這個 group，由 cleanup 收
    timeout -s TERM -k 60 "$RUN_TIMEOUT" bash test/tools/ab_run.sh "$4" "$3"
    rc=$?; t1=$(date +%s)
    case $rc in 0) how=完成;; 124|137) how=超時;; *) how="rc=$rc";; esac
    cleanup
    echo "$how"
    AB_T0=$t0; AB_T1=$t1; AB_HOW=$how
}

budget() {   # $1=輸出目錄 $2=世界檔
    [ -f "$1/run_traj.csv" ] && [ -f "$1/manager.log" ] || { echo "沒有軌跡或 manager.log"; return; }
    python3 test/tools/coverage_budget.py "$1/run" --log "$1/manager.log" \
        --world "$WS/src/mowerbot_bringup/worlds/$2" --heatmap "$1/heatmap.png" \
        > "$1/budget.txt" 2>&1
    grep -a "任務結束" "$1/coverage_run.log" | tr -d '\n'
    printf ' | '
    grep -a "E2b\|覆蓋率" "$1/budget.txt" | tr -s ' ' | tr '\n' ' '
}

df -h "$WS" | tail -1 | sed 's/^/磁碟 開始: /'

# ================= 【1】覆蓋率變異量 =================
if [[ " $STEPS " == *" 1 "* ]]; then
    for i in $(seq 1 "$COV_N"); do
        d=$ROOT/cov/run$i
        log "【1】第 $i/$COV_N 趟 -> $d"
        AB_MAP=stage25_ab ab_once 1 "$i" "$d" run > /dev/null
        s=$(budget "$d" demo_lawn.world)
        row 1 "$i" "$AB_T0" "$AB_T1" $((AB_T1-AB_T0)) "$AB_HOW" "$d" "$s"
        log "【1】第 $i 趟 $AB_HOW：$s"
    done
fi

# ================= 【3】障礙物世界 =================
if [[ " $STEPS " == *" 3 "* ]]; then
    d=$ROOT/obst/map
    log "【3】建圖 stage28_obst -> $d"
    AB_MAP=stage28_obst AB_WORLD=demo_lawn_obstacle.world ab_once 3 map "$d" map > /dev/null
    ok=$([ -f "$WS/src/mowerbot_bringup/maps/stage28_obst.pgm" ] && echo 有地圖 || echo 沒有地圖)
    row 3 map "$AB_T0" "$AB_T1" $((AB_T1-AB_T0)) "$AB_HOW" "$d" "$ok"
    log "【3】建圖 $AB_HOW：$ok"
    if [ "$ok" = 有地圖 ]; then
        for i in $(seq 1 "$OBST_N"); do
            d=$ROOT/obst/run$i
            log "【3】第 $i/$OBST_N 趟 -> $d"
            # 另外抓一次障礙物清單與邊界，確認內環有沒有被挖出來
            # map_server 只發一次 /map、map_to_boundary 只算一次，所以要一開始就聽
            mkdir -p "$d"
            ( sleep 5; timeout 150 ros2 topic echo /f2c_obstacles > "$d/f2c_obstacles.txt" 2>&1 ) &
            ( sleep 5; timeout 150 ros2 topic echo /f2c_boundary > "$d/f2c_boundary.txt" 2>&1 ) &
            AB_MAP=stage28_obst AB_WORLD=demo_lawn_obstacle.world ab_once 3 "$i" "$d" run > /dev/null
            wait
            s=$(budget "$d" demo_lawn_obstacle.world)
            row 3 "$i" "$AB_T0" "$AB_T1" $((AB_T1-AB_T0)) "$AB_HOW" "$d" "$s"
            log "【3】第 $i 趟 $AB_HOW：$s"
        done
    fi
fi

# ================= 【4】完整套件連跑 =================
if [[ " $STEPS " == *" 4 "* ]]; then
    for i in $(seq 1 "$SUITE_N"); do
        log "【4】完整套件第 $i/$SUITE_N 趟"
        cleanup
        before=$(ls -t test/logs | head -1)
        t0=$(date +%s)
        timeout -s TERM -k 60 "$SUITE_TIMEOUT" python3 -u test/smoke_test.py \
            > "$ROOT/suite$i.out" 2>&1
        rc=$?; t1=$(date +%s)
        case $rc in 124|137) how=超時;; *) how="rc=$rc";; esac
        cleanup
        after=$(ls -t test/logs | head -1)
        [ "$after" = "$before" ] && after="(沒有新目錄)"
        s=$(grep -a "總計" "$ROOT/suite$i.out" | tr -s ' ')
        s="$s $(grep -a '^  \[FAIL\]\|^  \[SKIP\]' "$ROOT/suite$i.out" | cut -c1-60 | tr '\n' ';')"
        row 4 "$i" "$t0" "$t1" $((t1-t0)) "$how" "test/logs/$after" "$s"
        log "【4】第 $i 趟 $how：$s"
    done
fi

df -h "$WS" | tail -1 | sed 's/^/磁碟 結束: /'
log "全部結束"
