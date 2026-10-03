# -*- coding: utf-8 -*-
"""單次對正成功率 p 與「n 次對正全部成功」的機率 p^n (階段 38 後續 6)。

用法：python3 test/tools/align_pn_curve.py <輸出.png> <標籤>=<成功>/<總數> [...]
  例：python3 test/tools/align_pn_curve.py pn.png "tol0.10=10/20" "tol0.25=55/55"
每個標籤畫一條 p^n (n = 1~60)，並在 n = 50 (demo_lawn C1 的對正次數) 與 n = 10 標出數值。
0 次失敗時 p 的點估計是 1，另外畫「三分之一法則」(rule of three) 的 95 % 下界 1 - 3/N。
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

out = sys.argv[1]
n = np.arange(1, 61)
fig, ax = plt.subplots(figsize=(8, 5))
for arg in sys.argv[2:]:
    lab, frac = arg.split('=')
    k, N = (int(v) for v in frac.split('/'))
    p = k / N
    line, = ax.plot(n, p ** n, label='%s  p = %d/%d = %.3f' % (lab, k, N, p))
    print('%-10s p = %d/%d = %.3f   p^10 = %.3f   p^50 = %.4f' % (lab, k, N, p, p ** 10, p ** 50))
    if k == N:
        pl = 1 - 3.0 / N
        ax.plot(n, pl ** n, '--', color=line.get_color(),
                label='%s  95%% lower bound p >= %.3f (rule of three)' % (lab, pl))
        print('%-10s 0 failures: 95%% lower bound p >= %.3f   p^10 >= %.3f   p^50 >= %.4f'
              % (lab, pl, pl ** 10, pl ** 50))
for x in (10, 50):
    ax.axvline(x, color='gray', lw=0.6)
ax.set_xlabel('number of alignments in one mission (n)')
ax.set_ylabel('P(all n alignments succeed) = p^n')
ax.set_ylim(0, 1.02)
ax.grid(alpha=0.3)
ax.legend(fontsize=8)
ax.set_title('Align success rate p and mission completion p^n (simulation)')
fig.tight_layout()
fig.savefig(out, dpi=120)
print('saved', out)
