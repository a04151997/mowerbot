#!/usr/bin/env bash
# ==========================================================================
# 階段 32 無人看顧批次（報告 32 節）。依序：
#
#   【A】完整套件連跑 SUITE_N 趟（不加任何重試）
#   【B】demo_lawn、存好的地圖 stage29_lawn，跑 COV_N 趟完整任務，
#        每趟做拆帳（coverage_budget）、掉頭外擺（swath_overshoot）、段落終點（segment_endpoint）
#
# 規則與 stage28_overnight.sh 相同：每一趟都有超時上限，超時就記錄、殺乾淨、繼續下一趟；
# 每趟之間清殘留行程；每趟結束立刻把結果追加到 $ROOT/progress.tsv；不錄 bag。
# 另外每趟開始前查磁碟，剩不到 MIN_FREE_GB 就停止整個批次（不要把磁碟寫滿）。
#
# 用法：
#   setsid nohup test/tools/stage32_overnight.sh <輸出根目錄> > <根目錄>/batch.log 2>&1 &
# 只跑某幾項：STEPS="A B"（預設全部）
# ==========================================================================
set -u
ROOT=${1:?用法: stage32_overnight.sh <輸出根目錄>}
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
STEPS=${STEPS:-"A B"}
SUITE_N=${SUITE_N:-6}
COV_N=${COV_N:-5}
RUN_TIMEOUT=${RUN_TIMEOUT:-2400}      # 一趟任務 40 分鐘
SUITE_TIMEOUT=${SUITE_TIMEOUT:-2700}  # 一趟套件 45 分鐘
MIN_FREE_GB=${MIN_FREE_GB:-10}
PROG=$ROOT/progress.tsv
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"

log() { echo "[$(date '+%m/%d %H:%M:%S')] $*"; }
row() { printf '%s\n' "$(IFS=$'\t'; echo "$*")" >> "$PROG"; }
[ -f "$PROG" ] || row 項目 趟 開始 結束 秒數 結束方式 目錄 摘要

disk_ok() {
    local free; free=$(df -BG --output=avail "$WS" | tail -1 | tr -dc '0-9')
    log "磁碟剩 ${free} GB"
    if [ "$free" -lt "$MIN_FREE_GB" ]; then
        log "磁碟剩不到 ${MIN_FREE_GB} GB，停止批次"; row 停止 - - - - 磁碟不足 - "剩 ${free} GB"
        return 1
    fi
}

# ---- 清殘留：與 stage28_overnight.sh 相同 ----
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

df -h "$WS" | tail -1 | sed 's/^/磁碟 開始: /'

# ================= 【A】完整套件連跑 =================
if [[ " $STEPS " == *" A "* ]]; then
    for i in $(seq 1 "$SUITE_N"); do
        disk_ok || exit 1
        log "【A】完整套件第 $i/$SUITE_N 趟"
        cleanup
        before=$(ls -t test/logs | head -1)
        t0=$(date +%s)
        timeout -s TERM -k 60 "$SUITE_TIMEOUT" python3 -u test/smoke_test.py \
            > "$ROOT/suite$i.out" 2>&1
        rc=$?; t1=$(date +%s)
        case $rc in 124|137) how=超時;; *) how="rc=$rc";; esac
        cleanup
        after=$(ls -t test/logs | grep '^20' | head -1)
        [ "$after" = "$before" ] && after="(沒有新目錄)"
        s=$(grep -a "總計" "$ROOT/suite$i.out" | tr -s ' ')
        s="$s $(grep -a '^\[FAIL\]\|^\[SKIP\]' "$ROOT/suite$i.out" | cut -c1-60 | tr '\n' ';')"
        # 兩個階段 31 的斷言絕對不該在正常值下觸發
        a=$(grep -rl '車輛幾何變更後 headland 未同步更新\|比後輪外緣寬度還窄' \
                "$ROOT/suite$i.out" "test/logs/$after" 2>/dev/null | wc -l)
        s="$s 斷言觸發檔案數=$a"
        row A "$i" "$t0" "$t1" $((t1-t0)) "$how" "test/logs/$after" "$s"
        log "【A】第 $i 趟 $how $((t1-t0))s：$s"
    done
fi

# ================= 【B】覆蓋率基準線 =================
if [[ " $STEPS " == *" B "* ]]; then
    for i in $(seq 1 "$COV_N"); do
        disk_ok || exit 1
        d=$ROOT/cov/run$i
        log "【B】第 $i/$COV_N 趟 -> $d"
        cleanup
        t0=$(date +%s)
        # 不加 setsid：timeout 會對整個 process group 送訊號（coverage_run.py 也在裡面）；
        # ab_run.sh 用 setsid 起的 demo / 定位 / map_server 逃出了這個 group，由 cleanup 收
        AB_MAP=stage29_lawn AB_WORLD=demo_lawn.world \
            timeout -s TERM -k 60 "$RUN_TIMEOUT" bash test/tools/ab_run.sh run "$d" > /dev/null 2>&1
        rc=$?; t1=$(date +%s)
        case $rc in 0) how=完成;; 124|137) how=超時;; *) how="rc=$rc";; esac
        cleanup
        s="沒有軌跡或 manager.log"
        if [ -f "$d/run_traj.csv" ] && [ -f "$d/manager.log" ]; then
            python3 test/tools/coverage_budget.py "$d/run" --log "$d/manager.log" \
                --world "$WS/src/mowerbot_bringup/worlds/demo_lawn.world" --heatmap "$d/heatmap.png" \
                > "$d/budget.txt" 2>&1
            python3 test/tools/swath_overshoot.py "$d" > "$d/overshoot.txt" 2>&1
            python3 test/tools/segment_endpoint.py "$d/run_traj.csv" "$d/manager.log" \
                > "$d/endpoint.txt" 2>&1
            s="$(grep -a '任務結束' "$d/coverage_run.log" | tr -d '\n') | $(grep -a 'E2b\|覆蓋率' "$d/budget.txt" | tr -s ' ' | tr '\n' ' ')"
        fi
        row B "$i" "$t0" "$t1" $((t1-t0)) "$how" "$d" "$s"
        log "【B】第 $i 趟 $how $((t1-t0))s：$s"
    done
fi

df -h "$WS" | tail -1 | sed 's/^/磁碟 結束: /'
log "全部結束"
