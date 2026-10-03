#!/usr/bin/env bash
# ==========================================================================
# 階段 38 後續 6：一趟完整 C1 (stage29_lawn 同圖、demo_lawn.world) + 四項驗收分析。
#
#   test/tools/stage38_demo_run.sh <yaw 容忍 rad> <輸出目錄>
#
# 環境變數 (選用)：
#   ALIGN_MST=<rad/s>        暫時改 nav2_params.yaml 的 AlignController.min_speed_theta (第二層)
#   ALIGN_ACC_THETA=<rad/s²> 暫時在 AlignController 加 acc_lim_theta / decel_lim_theta (第三層)
#   AB_WORLD / AB_MAP        場地 (預設 demo_lawn.world / stage29_lawn)
#   RECORD=1                 開 RViz 並錄螢幕 (test/tools/screen_record.py)
#   RUN_TIMEOUT=<秒>         預設 2400
#
# nav2_params.yaml 只在這一趟期間暫時改 align_goal_checker / AlignController 的值，
# 開始前備份、結束 (含中斷) 一律還原並重編 bringup。FollowPath 一個位元都不動 (結束時 cmp 檢查)。
# 驗收 (四項)：最終狀態 DONE、完成段數 >= 90 %、前 2 m 車頭角 < 0.558 m、全程無碰撞 (幾何)。
# ==========================================================================
set -u
TOL=${1:?yaw 容忍}; D=${2:?輸出目錄}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
mkdir -p "$D"; D="$(cd "$D" && pwd)"
NP=$WS/src/mowerbot_bringup/config/nav2_params.yaml
RUN_TIMEOUT=${RUN_TIMEOUT:-2400}
WORLD=${AB_WORLD:-demo_lawn.world}
MAPN=${AB_MAP:-stage29_lawn}
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$D/run.log"; }

cp "$NP" "$D/nav2_params.orig.yaml"
fp_sig() { python3 -c "import yaml,sys,json; print(json.dumps(yaml.safe_load(open(sys.argv[1]))['controller_server']['ros__parameters']['FollowPath'], sort_keys=True))" "$1"; }
FP0=$(fp_sig "$NP")
restore() {
    cp "$D/nav2_params.orig.yaml" "$NP"
    colcon build --packages-select mowerbot_bringup > /dev/null 2>&1
    cmp -s "$D/nav2_params.orig.yaml" "$NP" && log "nav2_params.yaml 已還原" || log "⚠️ nav2_params.yaml 還原不一致"
}
trap restore EXIT

# ---- 暫時參數 (只動 align_goal_checker.yaw_goal_tolerance 與 AlignController) ----
python3 - "$NP" "$TOL" "${ALIGN_MST:-}" "${ALIGN_ACC_THETA:-}" <<'PY'
import re, sys
p, tol, mst, acc = sys.argv[1:]
s = open(p).read()
blk = s.index('    align_goal_checker:')
m = re.compile(r'^(      yaw_goal_tolerance:\s*)\S+', re.M).search(s, blk)
s = s[:m.start()] + m.group(1) + tol + s[m.end():]
if mst:
    s, n = re.subn(r'^(      min_speed_theta:\s*)\S+', r'\g<1>' + mst, s, count=1, flags=re.M)
    assert n == 1
if acc:
    s = s.replace('      min_speed_xy: 0.01\n',
                  '      min_speed_xy: 0.01\n      acc_lim_theta: %s\n      decel_lim_theta: -%s\n' % (acc, acc), 1)
open(p, 'w').write(s)
PY
[ "$(fp_sig "$NP")" = "$FP0" ] || { log "❌ FollowPath 被改到了，停止"; exit 3; }
colcon build --packages-select mowerbot_bringup > /dev/null 2>&1
log "C1：yaw 容忍 $TOL  min_speed_theta ${ALIGN_MST:-(repo 值)}  acc_theta ${ALIGN_ACC_THETA:-(同 FollowPath)}  場地 $WORLD / $MAPN"
diff <(sed -n '/^    AlignController:/,/^    FollowPath:/p;/^    align_goal_checker:/,/stateful/p' "$D/nav2_params.orig.yaml") \
     <(sed -n '/^    AlignController:/,/^    FollowPath:/p;/^    align_goal_checker:/,/stateful/p' "$NP") > "$D/param_diff.txt"
bash "$D/../clean.sh" > /dev/null 2>&1 || bash "$WS/test/logs/stage38/clean.sh" > /dev/null 2>&1

REC_PID=
DEMO_ARGS=
if [ "${RECORD:-0}" = 1 ]; then
    DEMO_ARGS="rviz:=true"
    ( sleep 25; exec python3 "$WS/test/tools/screen_record.py" "$D/screen_raw.mp4" ) > "$D/record.log" 2>&1 &
    REC_PID=$!
fi
t0=$(date +%s)
AB_MAP=$MAPN AB_WORLD=$WORLD AB_LIMIT=$((RUN_TIMEOUT-120)) AB_DEMO_ARGS="$DEMO_ARGS" \
    timeout -s TERM -k 60 "$RUN_TIMEOUT" bash test/tools/ab_run.sh run "$D" > "$D/ab_run.out" 2>&1
rc=$?; t1=$(date +%s)
[ -n "$REC_PID" ] && { pkill -INT -f "screen_record.py $D/screen_raw.mp4"; sleep 5; }
bash "$WS/test/logs/stage38/clean.sh" > /dev/null 2>&1
log "ab_run 結束 rc=$rc，$((t1-t0)) s"

# ---- 分析 ----
W="$WS/src/mowerbot_bringup/worlds/$WORLD"
python3 test/tools/footprint_collision.py "$D" --world "$W" > "$D/collision.txt" 2>&1
python3 test/tools/corner_offset.py "$D" > "$D/corner.txt" 2>&1
python3 test/tools/coverage_budget.py "$D/run" --log "$D/manager.log" --world "$W" --traj base \
    --heatmap "$D/heatmap_base.png" > "$D/budget_base.txt" 2>&1
python3 test/tools/coverage_budget.py "$D/run" --log "$D/manager.log" --world "$W" --traj blade \
    --heatmap "$D/heatmap_blade.png" > "$D/budget_blade.txt" 2>&1
python3 - "$D" > "$D/verdict.txt" <<'PY'
import re, sys, os
d = sys.argv[1]
rd = lambda f: open(os.path.join(d, f), encoding='utf-8', errors='replace').read() if os.path.exists(os.path.join(d, f)) else ''
end = re.search(r'任務結束：state=(\d+) 完成 (\d+)/(\d+) 跳過 (\d+)，耗時 (\d+) s', rd('coverage_run.log'))
cor = re.search(r'【前 2\.0 m】.*?最大 ([\d.]+) m.*?= ([\d.]+) m', rd('corner.txt'))
col = 'COLLISION_FREE=yes' in rd('collision.txt')
mgr = rd('manager.log')
n_align = mgr.count('對正 [')
n_to = mgr.count('對正超時')
if end:
    st, ok, tot, sk, sec = int(end.group(1)), int(end.group(2)), int(end.group(3)), int(end.group(4)), int(end.group(5))
else:
    st, ok, tot, sk, sec = -1, 0, 1, 0, 0
c1 = st == 3
c2 = tot > 0 and ok >= 0.9 * tot
c3 = bool(cor) and float(cor.group(1)) < float(cor.group(2))
print('1 最終狀態 DONE        : %s (state=%d)' % ('通過' if c1 else '未通過', st))
print('2 完成段數 >= 90 %%     : %s (%d/%d = %.1f %%，跳過 %d)' % ('通過' if c2 else '未通過', ok, tot, 100.0 * ok / max(tot, 1), sk))
print('3 前 2 m 車頭角 < 判準 : %s (%s)' % ('通過' if c3 else '未通過',
      ('最大 %s m，判準 %s m' % (cor.group(1), cor.group(2))) if cor else '沒有取樣點'))
print('4 全程無碰撞 (幾何)     : %s' % ('通過' if col else '未通過'))
print('耗時 %d s；對正送出 %d 次、對正超時 %d 次' % (sec, n_align, n_to))
print('VERDICT=%s' % ('PASS' if (c1 and c2 and c3 and col) else 'FAIL'))
PY
cat "$D/verdict.txt" | tee -a "$D/run.log"
grep -a "覆蓋率 = \|扣掉 E1\|規劃可割" "$D/budget_base.txt" "$D/budget_blade.txt" | tee -a "$D/run.log"
