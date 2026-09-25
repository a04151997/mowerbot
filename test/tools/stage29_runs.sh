#!/usr/bin/env bash
# ==========================================================================
# 階段 29【1c】：修正 free_thresh 之後，demo_lawn 與 demo_lawn_obstacle 各跑 N 趟（報告 29.3）。
#
# 地圖 stage29_lawn / stage29_obst 是 stage25_ab / stage28_obst 的逐位元組複本，
# 只有 .yaml 的 free_thresh 0.25 -> 0.10（見 save_map.sh）。
# 每趟另外抓一次 map_server 發的 /map，數 -1 格數，證明未觀測格讀回來仍是未知。
# 每趟超時上限 40 分鐘，趟與趟之間清殘留（規則同 stage28_overnight.sh）。
#
# 用法：setsid nohup test/tools/stage29_runs.sh <輸出根目錄> > <根目錄>/batch.log 2>&1 &
# ==========================================================================
set -u
ROOT=${1:?用法: stage29_runs.sh <輸出根目錄>}
mkdir -p "$ROOT"; ROOT="$(cd "$ROOT" && pwd)"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
N=${N:-3}
RUN_TIMEOUT=${RUN_TIMEOUT:-2400}
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
log() { echo "[$(date '+%m/%d %H:%M:%S')] $*"; }

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
    for pid in $(pgrep -x gzserver) $(pgrep -x gzclient) $(pgrep -f 'run_demo.sh') \
               $(pgrep -f 'tools/coverage_run.py') $(pgrep -f 'tools/ab_run.sh'); do
        case "$ex" in *" $pid "*) continue;; esac; echo "$pid"; done
}
cleanup() {
    local p n=0
    for p in $(stale | sort -un); do kill -TERM "$p" 2>/dev/null && n=$((n+1)); done
    sleep 3
    for p in $(stale | sort -un); do kill -KILL "$p" 2>/dev/null; done
    ros2 daemon stop > /dev/null 2>&1; sleep 3
    log "清殘留：TERM $n 個；剩下 $(stale | wc -l) 個"
}

df -h "$WS" | tail -1 | sed 's/^/磁碟 開始: /'
for spec in "lawn stage29_lawn demo_lawn.world" "obst stage29_obst demo_lawn_obstacle.world"; do
    set -- $spec
    for i in $(seq 1 "$N"); do
        d=$ROOT/$1/run$i; mkdir -p "$d"
        log "$1 第 $i/$N 趟 -> $d"
        cleanup
        ( sleep 50; timeout 60 python3 -u test/tools/map_snapshot.py /map "$d/map_snapshot.npz" \
              > "$d/map_snapshot.txt" 2>&1 ) &
        t0=$(date +%s)
        AB_MAP=$2 AB_WORLD=$3 timeout -s TERM -k 60 "$RUN_TIMEOUT" bash test/tools/ab_run.sh run "$d" > /dev/null 2>&1
        rc=$?; t1=$(date +%s)
        wait
        cleanup
        python3 test/tools/coverage_budget.py "$d/run" --log "$d/manager.log" \
            --world "$WS/src/mowerbot_bringup/worlds/$3" --heatmap "$d/heatmap.png" > "$d/budget.txt" 2>&1
        log "$1 第 $i 趟 rc=$rc $((t1-t0))s：$(grep -a '任務結束' "$d/coverage_run.log") | $(grep -a '格數' "$d/map_snapshot.txt")"
    done
done
df -h "$WS" | tail -1 | sed 's/^/磁碟 結束: /'
log "全部結束"
