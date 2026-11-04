#!/usr/bin/env bash
# ==========================================================================
# 階段 38 後續 6 的四項驗收判定段（從 stage38_demo_run.sh 的「分析」段原封搬出，可單獨重跑）：
#
#   [MIN_CLEARANCE=0.15] test/tools/stage38_verdict.sh <輸出目錄> [world 檔名]
#
# 第 3 項 (2026-10-06 使用者授權，decisions 第 89 項)：車身四角到任何牆面 / 障礙物的最小幾何距離 >= MIN_CLEARANCE，
# 由 footprint_collision.py 量。舊的「前 2 m 車頭角」偏移量照樣印出，只當覆蓋品質指標，不參與 VERDICT。
# ==========================================================================
set -u
D=${1:?輸出目錄}; D="$(cd "$D" && pwd)"
WORLD=${2:-${AB_WORLD:-demo_lawn.world}}
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MIN_CLEARANCE=${MIN_CLEARANCE:-0.15}
set +u; source /opt/ros/humble/setup.bash; source "$WS/install/setup.bash"; set -u
cd "$WS"
log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$D/run.log"; }

# ---- 分析 ----
W="$WS/src/mowerbot_bringup/worlds/$WORLD"
python3 test/tools/footprint_collision.py "$D" --world "$W" --min-clearance "$MIN_CLEARANCE" > "$D/collision.txt" 2>&1
python3 test/tools/corner_offset.py "$D" > "$D/corner.txt" 2>&1
python3 test/tools/coverage_budget.py "$D/run" --log "$D/manager.log" --world "$W" --traj base \
    --heatmap "$D/heatmap_base.png" > "$D/budget_base.txt" 2>&1
python3 test/tools/coverage_budget.py "$D/run" --log "$D/manager.log" --world "$W" --traj blade \
    --heatmap "$D/heatmap_blade.png" > "$D/budget_blade.txt" 2>&1
python3 - "$D" "$MIN_CLEARANCE" > "$D/verdict.txt" <<'PY'
import re, sys, os
d = sys.argv[1]
min_clearance = float(sys.argv[2])
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
# 第 3 項 (decisions 第 89 項)：全程最小幾何淨空 >= min_clearance。數字由我們自己的 min_clearance 比，
# 不採信工具的 CLEARANCE_OK 字樣；工具沒輸出或解析不出數字 -> 未通過並印出原因，不預設通過。
clr = re.search(r'A\. footprint 與方塊的最小距離 ([\d.]+) m（([^，]*)，段 (.*?)，位置 (\([^)]*\))）', rd('collision.txt'))
if not rd('collision.txt'):
    c3, c3_info = False, '沒有 collision.txt (footprint_collision.py 沒有輸出)'
elif not clr:
    c3, c3_info = False, '解析不出最小距離 (footprint_collision.py 輸出格式不符或失敗)'
else:
    c3 = float(clr.group(1)) >= min_clearance
    c3_info = '最小 %s m，門檻 %.2f m；%s，%s，位置 %s' % (clr.group(1), min_clearance, clr.group(2), clr.group(3), clr.group(4))
print('1 最終狀態 DONE        : %s (state=%d)' % ('通過' if c1 else '未通過', st))
print('2 完成段數 >= 90 %%     : %s (%d/%d = %.1f %%，跳過 %d)' % ('通過' if c2 else '未通過', ok, tot, 100.0 * ok / max(tot, 1), sk))
print('3 最小淨空 >= %.2f m   : %s (%s)' % (min_clearance, '通過' if c3 else '未通過', c3_info))
print('  覆蓋品質指標 (不判定) 前 2 m 車頭角: %s' % (('最大 %s m，舊判準 %s m' % (cor.group(1), cor.group(2))) if cor else '沒有取樣點'))
print('4 全程無碰撞 (幾何)     : %s' % ('通過' if col else '未通過'))
print('耗時 %d s；對正送出 %d 次、對正超時 %d 次' % (sec, n_align, n_to))
print('VERDICT=%s' % ('PASS' if (c1 and c2 and c3 and col) else 'FAIL'))
PY
cat "$D/verdict.txt" | tee -a "$D/run.log"
grep -a "覆蓋率 = \|扣掉 E1\|規劃可割" "$D/budget_base.txt" "$D/budget_blade.txt" | tee -a "$D/run.log"
