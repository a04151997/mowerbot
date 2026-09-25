#!/usr/bin/env bash
# ==========================================================================
# 某一段送出之後 N 秒抓一次 local costmap（報告 29.4，只量測）
#
# 用法（與 ab_run.sh 同時跑，看它的 demo.log）：
#   test/tools/stuck_snapshot.sh <ab_run 輸出目錄> '<label>' [延遲秒數，預設 6] &
# 輸出 <目錄>/costmap_<label>.npz 與 .txt（格值分布）。
# ==========================================================================
set -u
OUT=$1; LABEL=$2; DELAY=${3:-6}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
TAG=$(echo "$LABEL" | tr -c 'A-Za-z0-9' '_')
for i in $(seq 1 1800); do
    grep -aq "送出任務 \[$LABEL\]" "$OUT/demo.log" 2>/dev/null && break
    sleep 1
done
sleep "$DELAY"
python3 -u "$WS/test/tools/map_snapshot.py" /local_costmap/costmap "$OUT/costmap_$TAG.npz" --volatile \
    > "$OUT/costmap_$TAG.txt" 2>&1
