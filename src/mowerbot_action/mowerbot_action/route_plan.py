"""割草線的割草順序與繞過障礙物的轉場路線 (2026-10-21 will 選 1：分區割、繞過障礙)。純計算，不依賴 ROS。

背景：10-20 will 在 lab1 用紅線圍出範圍、中間畫一個方形禁區、樓梯旁邊畫一個 U 形凹口，
按 Y 預覽，路線一條一條斜著穿過禁區與樓梯。沙盒用真的 Fields2Cover 2.1.0 跑他那張圖重現了：
- F2C 的 BruteForce 把每一條割草線被禁區 / 凹口切成的兩段都放在同一個清單裡，f2c_server
  只是照清單順序「這條往前、下一條往回」，相鄰兩段常常在障礙物的兩邊，接起來就穿過障礙物。
- mower_manager 從車子到下一段起點的 approach 是一條直線，也會穿過去。
實車上光達看得到的會被 collision_monitor 擋下、禁區在 local costmap 裡是牆，DWB 走不過去就跳過那一段：
不會撞，但會漏割、亂跳。

這個檔案做四件事：

1. plan_order：把割草線分成一塊一塊 (分區)，每一塊裡的割草線一列接一列、而且左右有重疊，
   在塊裡面照弓字形一來一回割；割完一塊再去離得最近的下一塊。這等於「弓字形分解」
   (boustrophedon decomposition)，只是直接用 F2C 擺好的割草線來分，割草線的位置與條數完全不變，
   只改割的順序與方向。F2C 原本的順序已經是好的弓字形 (沒有障礙物、沒有凹口) 時原樣回傳，
   跟以前一個位元都不差。

2. TransitPlanner：兩點之間的轉場路線。直線走得過去就走直線 (跟以前一樣)；走不過去就繞：
   在割草範圍裡找最短的折線，轉折點都選在「原地掉頭不會掃到邊界或障礙物」的地方
   (manager 每一段開始前會原地對正，轉折點就是對正的地方)。

3. EdgeLoops：割草線割完再沿邊多繞幾圈，把地頭那一條割掉 (第 3 節)。

4. WallClearance / trim_ends：要原地轉身的地方離地圖上的牆要夠遠，光達停止區轉一圈才掃不到牆 (第 4 節)。

作業區一樣用點陣算 (跟 swath_angle 同一套)：把邊界塗成格子圖、挖掉內環，算每一格離邊界多遠。
"""
import math

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 1. 割草順序
# ---------------------------------------------------------------------------

# 上下兩列的割草線左右至少要重疊這麼多才算「同一塊」：掉頭只要往旁邊挪一個線距，
# 不必沿著線走一大段。比這個少就分成兩塊，中間用轉場路線接。
MIN_OVERLAP = 0.3


def _row_coords(ends, angle):
    """每一條割草線轉成 (列的位置 v, 起點 u, 終點 u)。u 沿割草線方向，v 垂直。"""
    c, s = math.cos(angle), math.sin(angle)
    out = []
    for (ax, ay), (bx, by) in ends:
        ua, ub = c * ax + s * ay, c * bx + s * by
        va, vb = -s * ax + c * ay, -s * bx + c * by
        out.append(((va + vb) / 2.0, ua, ub))
    return out


def _rows(coords, spacing):
    """列的編號 (從 0 開始)：F2C 的割草線剛好一個線距一條，四捨五入就是第幾列。"""
    v0 = min(v for v, _a, _b in coords)
    return [int(round((v - v0) / spacing)) for v, _a, _b in coords]


def _overlap(a, b):
    """兩段 [u0, u1] 重疊的長度 (負值 = 沒重疊、中間差這麼多)。"""
    return min(a[1], b[1]) - max(a[0], b[0])


def _is_boustrophedon(rows, coords, spacing):
    """F2C 給的順序是不是已經是好的弓字形：每一條都跟下一條相鄰一列、左右有重疊、方向相反。"""
    for i in range(len(coords) - 1):
        _v, a0, a1 = coords[i]
        _w, b0, b1 = coords[i + 1]
        if abs(rows[i + 1] - rows[i]) != 1:
            return False
        if _overlap((min(a0, a1), max(a0, a1)), (min(b0, b1), max(b0, b1))) < MIN_OVERLAP:
            return False
        if (a1 - a0) * (b1 - b0) > 0:
            return False
    return True


# 同一列被切成好幾段、中間的空隙不到這麼長而且直線走得過去 (例如邊界小小的凹凸) 時，
# 當成同一條線一路割過去 (中間那一小段只是開過去)，不要每一小段都自成一塊、另外繞過去割。
MAX_BRIDGE = 2.0


def _row_groups(rows, coords, ends, clear):
    """同一列裡可以一路割過去的幾段併成一組。回傳 [(列, u0, u1, [割草線編號 (u 由小到大)])]。"""
    by_row = {}
    for i, r in enumerate(rows):
        by_row.setdefault(r, []).append(i)
    groups = []
    for r, idx in by_row.items():
        idx.sort(key=lambda i: min(coords[i][1], coords[i][2]))
        cur = [idx[0]]
        for i in idx[1:]:
            j = cur[-1]
            j_hi = max(coords[j][1], coords[j][2])
            i_lo = min(coords[i][1], coords[i][2])
            ok = (i_lo - j_hi) <= MAX_BRIDGE
            if ok and clear is not None:
                a = ends[j][1] if coords[j][2] > coords[j][1] else ends[j][0]   # j 的 u 大那一端
                b = ends[i][0] if coords[i][2] > coords[i][1] else ends[i][1]   # i 的 u 小那一端
                ok = clear(a, b)
            if ok:
                cur.append(i)
            else:
                groups.append(cur)
                cur = [i]
        groups.append(cur)
    out = []
    for g in groups:
        u0 = min(min(coords[i][1], coords[i][2]) for i in g)
        u1 = max(max(coords[i][1], coords[i][2]) for i in g)
        out.append((rows[g[0]], u0, u1, g))
    return out


def build_chains(groups):
    """把各列的組分塊 (弓字形分解)。回傳 [[組編號, ...], ...]，每一塊從最低那一列往上排。

    一塊裡相鄰兩列「一對一」：下面這一組往上只碰到一組、上面那一組往下也只碰到這一組
    (左右重疊至少 MIN_OVERLAP)。碰到障礙物時一列變成左右兩組 (分岔)，過了障礙物又合成一組 (合併)，
    分岔與合併的地方都切開成不同塊；哪一塊先割、從哪一頭割，交給 plan_order 比過所有排法再決定。
    (不要在分岔的地方隨便挑一邊接下去：10-21 測過，挑錯邊會在錯開的那一頭多開一大段。)
    """
    by_row = {}
    for k, g in enumerate(groups):
        by_row.setdefault(g[0], []).append(k)

    def touching(k, dr):
        g = groups[k]
        return [t for t in by_row.get(g[0] + dr, ())
                if _overlap(g[1:3], groups[t][1:3]) >= MIN_OVERLAP]

    free = set(range(len(groups)))
    chains = []
    while free:
        s = min(free, key=lambda k: (groups[k][0], groups[k][1]))
        free.discard(s)
        chain = [s]
        while True:
            up = touching(chain[-1], +1)
            if len(up) != 1 or up[0] not in free or len(touching(up[0], -1)) != 1:
                break
            free.discard(up[0])
            chain.append(up[0])
        chains.append(chain)
    return chains


def _chain_options(chain, groups, ends, coords):
    """一塊有 4 種割法：由下往上或由上往下 × 第一條往 +u 或往 -u。回傳 [(起點, 終點, [(編號, 反向)], 塊裡的空跑)]。

    塊裡的空跑 = 一條割完到下一條開始之間要開的直線距離總和 (掉頭、跨過小空隙)。
    上下兩列左右錯開時 (例如障礙物下緣那一列比上一列短)，某一種割法會在錯開的那一頭多開一大段，
    所以 4 種割法的空跑不一樣，排順序時要一起算。
    """
    opts = []
    for seq in (chain, chain[::-1]):
        for first_pos in (True, False):
            plan = []
            pos = first_pos
            for k in seq:
                members = groups[k][3] if pos else groups[k][3][::-1]
                for i in members:
                    _v, ua, ub = coords[i]
                    rev = (ub > ua) != pos          # 這一條要往 +u 走，但 F2C 給的是往 -u (或相反)
                    plan.append((i, rev))
                pos = not pos
            i0, r0 = plan[0]
            i1, r1 = plan[-1]
            start = ends[i0][1] if r0 else ends[i0][0]
            end = ends[i1][0] if r1 else ends[i1][1]
            inner = 0.0
            for (ia, ra), (ib, rb) in zip(plan[:-1], plan[1:]):
                a = ends[ia][0] if ra else ends[ia][1]
                b = ends[ib][1] if rb else ends[ib][0]
                inner += math.hypot(b[0] - a[0], b[1] - a[1])
            opts.append((start, end, plan, inner))
    return opts


def plan_order(ends, angle, spacing, start_xy, dist=None, clear=None):
    """割草的順序。ends = [((ax, ay), (bx, by)), ...] 是 F2C 給的割草線 (順序、方向照 F2C)。

    回傳 (order, info)：order = [(編號, 反向), ...] 照割草的先後；info = dict(chains=幾塊, changed=有沒有改)。
    F2C 的順序已經是好的弓字形就原樣回傳 (changed=False，反向全部 False)。
    dist(a, b) 是兩點之間要開多遠 (預設直線距離；manager 給的是繞過障礙物的距離)。
    clear(a, b) 是 a 到 b 的直線走不走得過去 (同一列的兩段要不要一路割過去；沒給就只看距離)。
    """
    n = len(ends)
    if n == 0:
        return [], {'chains': 0, 'changed': False}
    coords = _row_coords(ends, angle)
    rows = _rows(coords, spacing)
    if _is_boustrophedon(rows, coords, spacing):
        return [(i, False) for i in range(n)], {'chains': 1, 'changed': False}
    if dist is None:
        def dist(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])
    groups = _row_groups(rows, coords, ends, clear)
    chains = build_chains(groups)
    options = [_chain_options(ch, groups, ends, coords) for ch in chains]
    seq = _best_sequence(options, start_xy, dist)
    order = []
    for k, o in seq:
        order.extend(options[k][o][2])
    return order, {'chains': len(chains), 'changed': True}


def order_loops(loops, start_xy):
    """周邊環繞的每一圈從哪裡開始繞、先繞哪一圈：從車子 (或上一圈的起點) 最近的那一點開始。

    loops = [[(x, y), ...], ...]，每一圈最後一點 = 第 0 點 (f2c_server 的規則)。
    回傳 [(第幾圈, 從第幾點開始), ...]。繞完一圈會回到起點，所以下一圈從那裡找最近的。
    """
    left = set(range(len(loops)))
    here = start_xy
    out = []
    while left:
        best = None
        for k in left:
            core = loops[k][:-1] if len(loops[k]) > 1 else loops[k]
            for i, (x, y) in enumerate(core):
                d = math.hypot(x - here[0], y - here[1])
                if best is None or (d, k, i) < best:
                    best = (d, k, i)
        _d, k, i = best
        left.discard(k)
        out.append((k, i))
        here = loops[k][i]
    return out


# 塊數不超過這個就把所有的先後順序都比過 (動態規劃，8 塊約 1 秒)，再多就從近的開始一塊一塊挑。
DP_MAX_CHAINS = 8


def _best_sequence(options, start, dist):
    """各塊的先後與割法，讓轉場總長最短。回傳 [(塊, 割法), ...]。"""
    n = len(options)
    m = len(options[0]) if n else 0
    # 每一種割法的成本 = 開到它的起點 + 塊裡的空跑
    d0 = [[dist(start, options[k][o][0]) + options[k][o][3] for o in range(m)] for k in range(n)]
    if n > DP_MAX_CHAINS:
        left = set(range(n))
        here = start
        seq = []
        while left:
            best = None
            for k in left:
                for o in range(m):
                    d = dist(here, options[k][o][0]) + options[k][o][3]
                    if best is None or (d, k, o) < best:
                        best = (d, k, o)
            _d, k, o = best
            left.discard(k)
            seq.append((k, o))
            here = options[k][o][1]
        return seq
    # dd[(k, o)][(j, p)] = 割完 k (割法 o) 到開始 j (割法 p) 要開多遠
    dd = {}
    for k in range(n):
        for o in range(m):
            end = options[k][o][1]
            dd[(k, o)] = {(j, p): dist(end, options[j][p][0]) + options[j][p][3]
                          for j in range(n) if j != k for p in range(m)}
    INF = float('inf')
    cost = {}
    for k in range(n):
        for o in range(m):
            cost[(1 << k, k, o)] = (d0[k][o], None)
    for mask in range(1, 1 << n):
        for k in range(n):
            if not mask & (1 << k):
                continue
            for o in range(m):
                cur = cost.get((mask, k, o))
                if cur is None or cur[0] == INF:
                    continue
                for (j, p), d in dd[(k, o)].items():
                    if mask & (1 << j):
                        continue
                    key = (mask | (1 << j), j, p)
                    c = cur[0] + d
                    if key not in cost or c < cost[key][0]:
                        cost[key] = (c, (mask, k, o))
    full = (1 << n) - 1
    best = min(((cost[(full, k, o)][0], k, o) for k in range(n) for o in range(m)
                if (full, k, o) in cost), default=None)
    if best is None:
        return [(k, 0) for k in range(n)]
    seq = []
    key = (full, best[1], best[2])
    while key is not None:
        seq.append((key[1], key[2]))
        key = cost[key][1]
    return seq[::-1]


# ---------------------------------------------------------------------------
# 2. 轉場路線
# ---------------------------------------------------------------------------

# 格子圖的解析度：至少 0.05 m，場地大時放寬，讓一邊不超過約 MAX_CELLS 格
MIN_RES = 0.05
MAX_CELLS = 800


class TransitPlanner:
    """在割草範圍 (外圈 + 內環) 裡規劃兩點之間的轉場路線。

    travel_clearance：直線開過去時車子中心離邊界 / 障礙物至少這麼遠 (側向半寬 + 餘裕)。
    turn_clearance：轉折點要原地轉身，離邊界 / 障礙物至少這麼遠 (原地旋轉掃出的半徑 + 餘裕)。
    walls / wall_need：轉折點另外還要離地圖上的牆 (WallClearance) 至少 wall_need (光達停止區原地轉一圈
    掃不到牆，第 4 節)；None = 不查。
    """

    def __init__(self, rings, travel_clearance, turn_clearance, walls=None, wall_need=0.0):
        outer = np.asarray(rings[0], dtype=float)
        lo, hi = outer.min(axis=0), outer.max(axis=0)
        span = float(max(hi - lo))
        self.res = res = max(MIN_RES, span / MAX_CELLS)
        pad = 2 * res
        self.origin = lo - pad
        w = int(math.ceil((hi[0] - lo[0] + 2 * pad) / res)) + 1
        h = int(math.ceil((hi[1] - lo[1] + 2 * pad) / res)) + 1

        def px(ring):
            p = (np.asarray(ring, dtype=float) - self.origin) / res - 0.5
            return np.round(p * 16).astype(np.int32).reshape(-1, 1, 2)   # 4 bit 小數 (shift=4)

        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [px(rings[0])], 1, lineType=cv2.LINE_8, shift=4)
        for hole in rings[1:]:
            if len(hole) >= 3:
                cv2.fillPoly(mask, [px(hole)], 0, lineType=cv2.LINE_8, shift=4)
        self.dist = _distance(mask, res)
        self.travel_clearance = float(travel_clearance)
        self.turn_clearance = float(turn_clearance)
        self.travel = self.dist >= self.travel_clearance
        self.turnable = self.dist >= self.turn_clearance + res
        if walls is not None and wall_need > 0.0:
            cx = self.origin[0] + (np.arange(w) + 0.5) * res
            cy = self.origin[1] + (np.arange(h) + 0.5) * res
            gx, gy = np.meshgrid(cx, cy)
            self.turnable &= walls.at(np.stack([gx, gy], axis=-1)) >= wall_need + res
        self.nodes = self._corner_nodes()
        self._adj = None
        self._cache = {}

    def _corner_nodes(self):
        """轉折點的候選：掉得了頭的區域 (離邊界 >= turn_clearance、離牆夠遠) 的外形頂點，往裡面再縮一格。

        最短的繞行路線只會在障礙物的角 (掉得了頭的區域往外凸的角) 轉彎，所以只留那些角：
        判斷方法是那個角兩邊的鄰點連線的中點落在區域外面 (區域在那裡凹進去)。
        """
        res = self.res
        inner = self.turnable.astype(np.uint8)
        contours, _h = cv2.findContours(inner, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        pts = []
        for c in contours:
            if len(c) < 3:
                continue
            ap = cv2.approxPolyDP(c, 1.5, True).reshape(-1, 2)
            m = len(ap)
            for k in range(m):
                a, p, b = ap[k - 1], ap[k], ap[(k + 1) % m]
                mid = (a + b) / 2.0
                u, v = int(round(mid[0])), int(round(mid[1]))
                if 0 <= v < inner.shape[0] and 0 <= u < inner.shape[1] and inner[v, u]:
                    continue    # 中點還在區域裡：這個角是往外凸的，繞行路線不會在這裡轉
                pts.append(self.origin + (p + 0.5) * res)
        return np.array(pts, dtype=float).reshape(-1, 2)

    def _cell(self, xy):
        g = (np.asarray(xy, dtype=float) - self.origin) / self.res
        return np.floor(g).astype(np.int64)

    def clearance(self, x, y):
        """這一點離邊界 / 障礙物多遠 (公尺；範圍外是 0)。"""
        u, v = self._cell((x, y))
        if 0 <= v < self.dist.shape[0] and 0 <= u < self.dist.shape[1]:
            return float(self.dist[v, u])
        return 0.0

    def _visible(self, p, qs, relax_p=0.0, relax_q=0.0):
        """從 p 直線開到 qs 裡每一點走不走得過去 (bool 陣列)。離 p 或 q 這麼近的取樣點不檢查。"""
        qs = np.asarray(qs, dtype=float).reshape(-1, 2)
        if len(qs) == 0:
            return np.zeros(0, bool)
        p = np.asarray(p, dtype=float)
        d = qs - p
        length = np.hypot(d[:, 0], d[:, 1])
        step = self.res * 0.5
        m = int(math.ceil(float(length.max()) / step)) + 1 if len(length) else 1
        t = np.linspace(0.0, 1.0, max(m, 2))
        pts = p[None, None, :] + t[None, :, None] * d[:, None, :]          # (k, m, 2)
        along = t[None, :] * length[:, None]
        check = (along >= relax_p) & (along <= length[:, None] - relax_q)
        g = np.floor((pts - self.origin) / self.res).astype(np.int64)
        inside = ((g[..., 0] >= 0) & (g[..., 0] < self.travel.shape[1]) &
                  (g[..., 1] >= 0) & (g[..., 1] < self.travel.shape[0]))
        ok = np.zeros(inside.shape, bool)
        gi = g[inside]
        ok[inside] = self.travel[gi[:, 1], gi[:, 0]]
        return np.all(ok | ~check, axis=1)

    def clear(self, a, b, relax_a=0.0, relax_b=0.0):
        """a 到 b 的直線走不走得過去。"""
        return bool(self._visible(a, [b], relax_a, relax_b)[0])

    def plan(self, a, b, relax=0.5):
        """a 到 b 的轉場路線 [a, 轉折點..., b]；直線走得過去就是 [a, b]。找不到路回傳 None。

        relax：離 a、b 這麼近的地方不檢查 (車子本來就停在那裡、或要開到跑道的起點，
        那兩個點本身已經由 manager 的跑道 / 掉頭檢查把關)。
        """
        a = (float(a[0]), float(a[1]))
        b = (float(b[0]), float(b[1]))
        key = (round(a[0], 2), round(a[1], 2), round(b[0], 2), round(b[1], 2), relax)
        if key in self._cache:
            return self._cache[key]
        if self.clear(a, b, relax, relax):
            out = [a, b]
        else:
            out = self._astar(a, b, relax)
        self._cache[key] = out
        return out

    def length(self, a, b, relax=0.5):
        """a 到 b 要開多遠 (找不到路是無限大)。"""
        path = self.plan(a, b, relax)
        if path is None:
            return float('inf')
        return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
                   for i in range(len(path) - 1))

    def _graph(self):
        """轉折點之間誰看得到誰 (第一次要繞路時才算，之後每次規劃共用)。回傳距離矩陣 (看不到 = inf)。"""
        if self._adj is None:
            nodes = self.nodes
            n = len(nodes)
            adj = np.full((n, n), np.inf)
            for i in range(n - 1):
                vis = self._visible(nodes[i], nodes[i + 1:])
                for j in np.flatnonzero(vis) + i + 1:
                    d = float(np.hypot(*(nodes[j] - nodes[i])))
                    adj[i, j] = adj[j, i] = d
            self._adj = adj
        return self._adj

    def _astar(self, a, b, relax):
        """直線走不過去時：起點 -> 轉折點 ... -> 終點 的最短折線 (Dijkstra，轉折點不多)。"""
        nodes = self.nodes
        n = len(nodes)
        if n == 0:
            return None
        adj = self._graph()
        pa, pb = np.asarray(a), np.asarray(b)
        da = np.where(self._visible(pa, nodes, relax_p=relax),
                      np.hypot(nodes[:, 0] - pa[0], nodes[:, 1] - pa[1]), np.inf)
        db = np.where(self._visible(pb, nodes, relax_p=relax),
                      np.hypot(nodes[:, 0] - pb[0], nodes[:, 1] - pb[1]), np.inf)
        dist = da.copy()
        parent = np.full(n, -1)
        done = np.zeros(n, bool)
        while True:
            cand = np.where(done, np.inf, dist)
            u = int(np.argmin(cand))
            if not np.isfinite(cand[u]):
                break
            done[u] = True
            nd = dist[u] + adj[u]
            better = (nd < dist) & ~done
            dist[better] = nd[better]
            parent[better] = u
        total = dist + db
        u = int(np.argmin(total))
        if not np.isfinite(total[u]):
            return None
        chain = []
        while u != -1:
            chain.append((float(nodes[u][0]), float(nodes[u][1])))
            u = int(parent[u])
        return [a] + chain[::-1] + [b]


# ---------------------------------------------------------------------------
# 3. 沿邊多繞幾圈 (2026-10-21 will 同意)
# ---------------------------------------------------------------------------
#
# 割草線兩頭、F2C 的環繞都要原地轉身：車頭在後輪軸前 0.975 m，原地轉一圈車頭角掃出 1.06 m，
# 所以割草線和環繞都離邊界 1.2 m (地頭) 以上，這一圈從來沒割 (will 的 lab1 圖紅線內只割到約 56%)。
# 這裡在地頭裡沿著邊界再繞幾圈 (離邊界 offsets，實車 1.10、0.75 m)：
#   * 轉角改成邊走邊轉的圓弧，不原地轉。外凸的轉角 (前面是邊界) 圓弧要夠大，車頭角才不會
#     伸出邊界 (corner_radius)；內凹的轉角 (繞著障礙物的角轉) 本來就離那個角 d。
#     做法是開運算：離邊界 >= d + r 的區域往外長 r，外凸的角自然變成半徑 r 的圓弧。
#   * 進圈、出圈各一條斜斜的「匝道」：從離邊界 safe_offset (地頭寬，原地轉身掃不到邊界) 的點
#     斜著開進這一圈，繞完一圈再斜著開出去。原地轉身只會發生在匝道的兩頭。
#   * 一圈切成幾段交給 Nav2，段與段之間方向連續，manager 不再原地對正 (見 manager 的 send_next_swath)。
#   * 每一個位置都檢查整個車身在割草範圍裡 (離邊緣至少 EDGE_MARGIN)；有給 walls (地圖上真的牆) 時，
#     光達停止區也不能掃到牆 (掃到車子就停，那一段 15 s 後判失敗)。檢查不過的地方那一圈就斷開，
#     剩下的部分各自進圈出圈，太短就不割。
# 用點陣算 (跟 TransitPlanner 同一套)，格子細一點。

EDGE_RES = 0.025            # 格子大小 (m)；場地大時放寬，一邊不超過 EDGE_MAX_CELLS 格
EDGE_MAX_CELLS = 2400
EDGE_MARGIN = 0.05          # 車身離割草範圍邊緣至少這麼遠
# 轉角圓弧至少這麼大：太小的弧 DWB 跟不好 (10-21 模擬實跑：半徑 0.5 m 的地方車頭方向落後路線 30° 以上，
# 有一段 U 形彎卡在彎前面 15 s 判失敗)。1.0 m 在 speed_scale 0.5 時 (0.35 m/s、0.6 rad/s) 不用減速就轉得過去。
EDGE_MIN_RADIUS = 1.0
EDGE_STEP = 0.1             # 航點間距 (跟 F2C 一樣)
EDGE_RAMPS = (2.0, 1.5, 1.0)    # 匝道長度，先試長的 (越長越平緩)
# 四周是牆、匝道起點要往裡面多挪 (第 4 節) 時，上面那幾種都太斜 (開進圈時停止區掃到牆) 才試的更長匝道
EDGE_RAMPS_LONG = (3.0, 2.5)
EDGE_RAMP_MAX_TURN = 0.35   # 匝道旁邊那一段圈的方向最多偏這麼多 (rad)，太彎就換一個起點
# 匝道起點離牆不夠遠 (第 4 節) 時往裡面多挪，每次 EDGE_RAMP_OFF_STEP，匝道最斜跟圈差 EDGE_RAMP_MAX_ANGLE
EDGE_RAMP_OFF_STEP = 0.1
EDGE_RAMP_MAX_ANGLE = math.radians(20.0)
EDGE_SEG = 6.0              # 每一段大約這麼長 (m)
# 每一段最多轉這麼多 (rad)：DWB 只看前面 2 m 左右的路線，U 形彎整個在同一段裡的話，
# 它把彎後面那一截當目標，選「原地慢慢轉」不往前開 (10-21 模擬實跑卡住的那一段轉了 188°)
EDGE_SEG_MAX_TURN = math.radians(120.0)
EDGE_MIN_PIECE = 2.0        # 斷開之後剩不到這麼長的就不割
EDGE_SMOOTH = 0.15          # 等高線的鋸齒用前後這麼長平均掉 (m)
EDGE_MIN_LOOP = 4.0         # 比這個短的圈 (很小的一塊) 不繞


def corner_radius(d, front, half_w, margin=EDGE_MARGIN):
    """離邊界 d 的那一圈，外凸轉角的圓弧至少要多大，車頭角才不會伸出邊界 (a <= 0 回傳 None)。

    車子沿半徑 r 的圓弧走 (圓心 O)，外側的車頭角離 O 是 hypot(r + half_w, front)；
    開運算保證 O 離邊界至少 r + d。要 hypot(r + half_w, front) <= r + d - margin：
        r >= (front² - a²) / (2a) - half_w，a = d - margin - half_w。
    """
    a = d - margin - half_w
    if a <= 0:
        return None
    return max(0.0, (front * front - a * a) / (2.0 * a) - half_w)


# 割草範圍離牆至少這麼遠 (map_to_boundary 把空地往內腐蝕 2 格 = 0.10 m)：沿邊那一圈離牆 = d + 這個值
EDGE_WALL_GAP = 0.10
# 地圖上的佔據格連成一塊、長或寬至少這麼大才當牆
EDGE_WALL_MIN_SIZE = 0.30
# 車頭方向會偏離路線多少 (rad)：光達停止區在這個範圍裡怎麼轉都不能掃到牆。
# 10-21 模擬實跑 (speed_scale 0.5)：直的地方 6° 內，轉彎的地方 18°，急彎 (半徑 0.5 m) 35°；
# 有一個牆角路線上停止區離牆 0.16 m，車頭落後 15° 就掃到牆，車子停住、不能倒車也不能原地轉，整趟停止。
EDGE_YAW_TOL = math.radians(20.0)
# 停止區 (轉過 ±EDGE_YAW_TOL 的範圍) 離牆至少這麼遠
EDGE_ZONE_MARGIN = 0.05
# 牆邊轉角要這麼大的圓弧才轉得過去的話，那一圈不在牆角轉彎：牆邊斷開，一截一截割 (離牆太近的那幾截不割)
EDGE_WALL_MAX_RADIUS = 2.5


def tolerant_box(front, half_w, tol):
    """車頭方向往外偏 tol 時，長方形 (前緣 front、半寬 half_w) 外側前角的位置：回傳 (往前, 往外)。

    給 corner_radius 用：偏 tol 的長方形外側前角，當成沒偏的長方形 (front', half_w') 的角。
    """
    c, s = math.cos(tol), math.sin(tol)
    return max(front * c - half_w * s, 0.0), front * s + half_w * c


def _swept_samples(x0, x1, half_w, tol, step=0.05):
    """長方形 (x0..x1, ±half_w) 繞原點轉 -tol..tol 掃過的地方，取樣點落在 step 的格點上 (不重複)。"""
    box = _box_samples(x0, x1, half_w, step / 2.0)
    n = max(int(math.ceil(2.0 * tol / math.radians(1.0))), 1)
    pts = []
    for a in np.linspace(-tol, tol, n + 1):
        c, s = math.cos(a), math.sin(a)
        pts.append(np.column_stack([c * box[:, 0] - s * box[:, 1], s * box[:, 0] + c * box[:, 1]]))
    q = np.unique(np.round(np.vstack(pts) / step).astype(np.int64), axis=0)
    return q.astype(float) * step


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _resample_closed(pts, step):
    """閉合折線照弧長重新取樣成等間距 (最後一點不重複第 0 點)。回傳 (點, 周長)。"""
    p = np.vstack([pts, pts[:1]])
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    length = float(s[-1])
    n = max(int(round(length / step)), 3)
    t = np.arange(n) * (length / n)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])]), length


def _smooth_closed(pts, k):
    """閉合折線的前後各 k 點平均 (把格子的鋸齒抹平)。"""
    if k <= 0 or len(pts) <= 2 * k + 1:
        return pts
    c = np.concatenate([pts[-k:], pts, pts[:k]])
    ker = np.ones(2 * k + 1) / (2 * k + 1)
    return np.column_stack([np.convolve(c[:, 0], ker, 'valid'),
                            np.convolve(c[:, 1], ker, 'valid')])


def _headings(pts, closed):
    """每一點的行進方向 (rad)。"""
    if closed:
        d = np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)
    else:
        d = np.gradient(pts, axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def _line(a, b, step):
    """a 到 b 的直線，間距約 step，含兩端。"""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(int(math.ceil(float(np.hypot(*(b - a))) / step)), 1)
    t = np.linspace(0.0, 1.0, n + 1)[:, None]
    return a + t * (b - a)


def _distance(mask, res):
    """每一格離最近的 mask 外格子多遠 (公尺)。"""
    return cv2.distanceTransform(np.asarray(mask).astype(np.uint8), cv2.DIST_L2,
                                 cv2.DIST_MASK_PRECISE) * np.float32(res)


def _box_samples(x0, x1, half_w, step):
    """長方形 (x0..x1, -half_w..half_w) 裡面的取樣點 (含邊)，間距不超過 step。"""
    xs = np.linspace(x0, x1, max(int(math.ceil((x1 - x0) / step)), 1) + 1)
    ys = np.linspace(-half_w, half_w, max(int(math.ceil(2 * half_w / step)), 1) + 1)
    return np.array([(x, y) for x in xs for y in ys])


def _big_walls(wall, res, min_size=EDGE_WALL_MIN_SIZE):
    """佔據格連成一塊、長或寬至少 min_size 才當牆。

    零星的小點 (雜訊、細桌腳) 不當牆：割草範圍本來也不理它們 (map_to_boundary 的 MIN_OBSTACLE_AREA)，
    割草線一樣會從旁邊經過；拿它們當牆的話沿邊那一圈會被推得很遠。
    """
    _n, lab, stats, _c = cv2.connectedComponentsWithStats(
        np.asarray(wall).astype(np.uint8), connectivity=8)
    size = np.maximum(stats[:, cv2.CC_STAT_WIDTH], stats[:, cv2.CC_STAT_HEIGHT]) * res
    big = size >= min_size
    big[0] = False                     # 第 0 塊是背景
    return big[lab]


def segment_route(pts, seg_len=EDGE_SEG, min_gap=1.0, max_turn=EDGE_SEG_MAX_TURN):
    """把一條沿邊路線切成幾段交給 Nav2：回傳 [(i0, i1)] (含兩端，下一段從上一段的終點開始)。

    每段約 seg_len (最長 1.25 倍)、方向最多轉 max_turn；切點挑附近最直的地方
    (上一段結束時車頭方向跟下一段的起點差最少，接著走不用原地對正)。
    每段的起點和終點至少相距 min_gap (起點 = 終點的段落 goal checker 一送出就判到達，
    見 manager 的不變量 ②)；剩下不到半段的併進前一段。
    """
    pts = np.asarray(pts, dtype=float)
    n = len(pts)
    if n < 3:
        return []
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])
    yaw = np.unwrap(_headings(pts, False))
    # 每一點前後 0.5 m 方向差多少：越小越直
    h = max(int(round(0.5 / max(float(s[-1]) / (n - 1), 1e-6))), 1)
    lo_i, hi_i = np.clip(np.arange(n) - h, 0, n - 1), np.clip(np.arange(n) + h, 0, n - 1)
    bend = np.round(np.abs(yaw[hi_i] - yaw[lo_i]) / 0.05)     # 0.05 rad 一級：一樣直的挑最遠的

    def gap(i, j):
        return float(np.hypot(*(pts[j] - pts[i])))

    out = []
    i0 = 0
    while i0 < n - 1:
        # 這一段最遠可以到哪裡：長度 <= 1.25 seg_len、方向範圍 <= max_turn
        j = i0 + 1
        ymin, ymax = min(yaw[i0], yaw[j]), max(yaw[i0], yaw[j])
        while j < n - 1 and s[j + 1] - s[i0] <= 1.25 * seg_len:
            y = yaw[j + 1]
            if max(ymax, y) - min(ymin, y) > max_turn:
                break
            ymin, ymax = min(ymin, y), max(ymax, y)
            j += 1
        if j >= n - 1:
            i1 = n - 1
        else:
            # 在後半段 (起點往後至少 min_gap) 挑最直的地方切
            want = s[i0] + max(min_gap, 0.5 * min(seg_len, s[j] - s[i0]))
            cand = [c for c in range(int(np.searchsorted(s, want)), j + 1)
                    if gap(i0, c) >= min_gap]
            if cand:
                cand = np.array(cand)
                i1 = int(cand[np.lexsort((-cand, bend[cand]))][0])
            else:
                i1 = j
            # 剩下不到半段：這一段直接走到底 (加起來轉太多或太長就不併，剩下的另成一段)
            if s[-1] - s[i1] < 0.5 * seg_len and np.ptp(yaw[i0:]) <= max_turn \
                    and s[-1] - s[i0] <= 1.5 * seg_len:
                i1 = n - 1
        if i1 - i0 < 2 or gap(i0, i1) < min_gap:
            if out:                             # 太短或起點離終點太近：併進前一段
                out[-1] = (out[-1][0], i1)
                i0 = i1
                continue
            more = [k for k in range(i1 + 1, n) if k - i0 >= 2 and gap(i0, k) >= min_gap]
            if not more:
                return []                       # 整條都不到 min_gap
            i1 = more[0]
        out.append((i0, i1))
        i0 = i1
    return out


class EdgeLoops:
    """在割草範圍 (外圈 + 內環) 的地頭裡規劃沿邊繞圈的路線。

    front / rear / half_w：車身 footprint (後輪軸中心往前、往後、往旁邊伸出多少)。
    walls：(bool 網格, 解析度, origin_x, origin_y)，地圖上真的牆 (光達看得到)；None = 不檢查停止區。
    stop_len / stop_side：光達停止區 (車頭前緣往前 stop_len、左右比車身各寬 stop_side)。
    turn_wall：要原地轉身的點 (匝道兩頭、escape 的終點) 離牆至少這麼遠 (第 4 節)；0 = 不查。
    """

    def __init__(self, rings, front, rear, half_w, walls=None, stop_len=0.40, stop_side=0.05,
                 turn_wall=0.0):
        outer = np.asarray(rings[0], dtype=float)
        lo, hi = outer.min(axis=0), outer.max(axis=0)
        span = float(max(hi - lo))
        self.res = res = max(EDGE_RES, span / EDGE_MAX_CELLS)
        # 四周多留停止區伸得到的距離：範圍外面的牆也要在格子裡
        pad = float(front) + stop_len + float(half_w) + stop_side
        self.origin = lo - pad
        w = int(math.ceil((hi[0] - lo[0] + 2 * pad) / res)) + 1
        h = int(math.ceil((hi[1] - lo[1] + 2 * pad) / res)) + 1

        def px(ring):
            p = (np.asarray(ring, dtype=float) - self.origin) / res - 0.5
            return np.round(p * 16).astype(np.int32).reshape(-1, 1, 2)   # 4 bit 小數 (shift=4)

        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [px(rings[0])], 1, lineType=cv2.LINE_8, shift=4)
        for hole in rings[1:]:
            if len(hole) >= 3:
                cv2.fillPoly(mask, [px(hole)], 0, lineType=cv2.LINE_8, shift=4)
        self.dist = _distance(mask, res)
        self.front, self.rear, self.half_w = float(front), float(rear), float(half_w)
        # 車身裡面的取樣點 (base_footprint 座標)：內部也取，比車身小的障礙物才不會漏掉
        self._body = _box_samples(-self.rear, self.front, self.half_w, 0.1)
        self.wall = None
        self._zone = None
        if walls is not None:
            grid, wres, wox, woy = walls
            cx = self.origin[0] + (np.arange(w) + 0.5) * res
            cy = self.origin[1] + (np.arange(h) + 0.5) * res
            cu = np.floor((cx - wox) / wres).astype(np.int64)
            cv_ = np.floor((cy - woy) / wres).astype(np.int64)
            ok_u = (cu >= 0) & (cu < grid.shape[1])
            ok_v = (cv_ >= 0) & (cv_ < grid.shape[0])
            wall = np.zeros((h, w), bool)
            if ok_u.any() and ok_v.any():
                wall[np.ix_(ok_v, ok_u)] = grid[np.ix_(cv_[ok_v], cu[ok_u])]
            wall = _big_walls(wall, res)
            self.wall = wall
            zw = self.half_w + stop_side
            # 停止區在車頭方向偏 ±EDGE_YAW_TOL 的範圍裡掃過的地方 (DWB 跟路線會有方向誤差)
            self._zone = _swept_samples(self.front, self.front + stop_len, zw, EDGE_YAW_TOL)
            # 離牆多遠 (轉角的圓弧要讓停止區掃不到牆，見 loops)；停止區要離牆 EDGE_ZONE_MARGIN 以上
            self.wall_dist = _distance(~wall, res)
            self._wall_near = self.wall_dist <= EDGE_ZONE_MARGIN + 0.5 * res
            self.zone_front, self.zone_half = self.front + stop_len, zw
        self.turn_wall = float(turn_wall) if self.wall is not None else 0.0

    # ---- 查格子 ----
    def _cells(self, xy):
        g = np.floor((np.asarray(xy, dtype=float) - self.origin) / self.res).astype(np.int64)
        h, w = self.dist.shape
        inside = (g[..., 0] >= 0) & (g[..., 0] < w) & (g[..., 1] >= 0) & (g[..., 1] < h)
        return g, inside

    def clearance_at(self, xy):
        """每一點離割草範圍邊緣多遠 (範圍外 = 0)。xy: (..., 2)。"""
        g, inside = self._cells(xy)
        out = np.zeros(inside.shape, np.float32)
        out[inside] = self.dist[g[inside][:, 1], g[inside][:, 0]]
        return out

    def turn_ok(self, xy):
        """每一點離牆夠不夠遠，原地轉身時光達停止區掃不到牆 (turn_wall)。xy: (..., 2)。"""
        xy = np.asarray(xy, dtype=float)
        if self.turn_wall <= 0.0:
            return np.ones(xy.shape[:-1], bool)
        g, inside = self._cells(xy)
        d = np.full(inside.shape, np.inf)
        d[inside] = self.wall_dist[g[inside][:, 1], g[inside][:, 0]] - 0.5 * self.res
        return d >= self.turn_wall

    def _sample(self, pts, yaw, local):
        c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
        x = pts[:, 0:1] + c * local[None, :, 0] - s * local[None, :, 1]
        y = pts[:, 1:2] + s * local[None, :, 0] + c * local[None, :, 1]
        return np.stack([x, y], axis=-1)

    def poses_ok(self, pts, yaw, margin=EDGE_MARGIN):
        """每一個位姿 (pts[i], yaw[i]) 車身是不是都在範圍裡 (離邊緣 >= margin)，停止區有沒有掃到牆
        (車頭方向偏 ±EDGE_YAW_TOL 都算，離牆 EDGE_ZONE_MARGIN 以內就算掃到)。"""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        yaw = np.asarray(yaw, dtype=float).reshape(-1)
        if len(pts) == 0:
            return np.zeros(0, bool)
        ok = self.clearance_at(self._sample(pts, yaw, self._body)).min(axis=1) >= margin
        if self.wall is not None:
            g, inside = self._cells(self._sample(pts, yaw, self._zone))
            hit = np.zeros(inside.shape, bool)
            hit[inside] = self._wall_near[g[inside][:, 1], g[inside][:, 0]]
            ok &= ~hit.any(axis=1)
        return ok

    # ---- 一圈 ----
    def loops(self, d):
        """離邊界 d 的那幾圈 (每一塊、每一個內環各一圈)，每圈是等間距的閉合折線，割草範圍在行進方向的左邊。"""
        r = corner_radius(d, self.front, self.half_w)
        if r is None:
            return []
        region = self._opened(self.dist, d, max(r, EDGE_MIN_RADIUS))
        if self.wall is not None:
            # 地圖上真的牆：光達停止區 (車頭前 stop_len、左右各寬 stop_side) 在轉角也不能掃到牆，
            # 牆邊的轉角圓弧要更大。離牆的距離是 d + EDGE_WALL_GAP (割草範圍本來就離牆這麼遠)。
            # 車頭方向偏 EDGE_YAW_TOL (轉彎時落後) 也不能掃到：當成比較大的停止區算圓弧。
            # 圓弧大到 EDGE_WALL_MAX_RADIUS 以上 (或怎麼轉都不行) 就不在牆角轉彎，
            # 留給 routes 的檢查把這一圈在牆邊斷開 (離牆太近的那幾截不割)。
            dw = d + EDGE_WALL_GAP
            zf, zh = tolerant_box(self.zone_front, self.zone_half, EDGE_YAW_TOL)
            rw = corner_radius(dw, zf, zh, margin=EDGE_ZONE_MARGIN)
            if rw is not None and rw <= EDGE_WALL_MAX_RADIUS:
                region &= self._opened(self.wall_dist, dw, max(rw, EDGE_MIN_RADIUS))
                # 兩種邊界交會的地方會有尖角：再用最小的圓弧修圓
                region = self._opened(_distance(region, self.res), 0.0, EDGE_MIN_RADIUS)
        if not region.any():
            return []
        contours, hier = cv2.findContours(
            region.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        out = []
        k = int(round(EDGE_SMOOTH / self.res))
        for ci, c in enumerate(contours):
            if len(c) < 8:
                continue
            pts = self.origin + (c.reshape(-1, 2).astype(float) + 0.5) * self.res
            pts, _l = _resample_closed(pts, self.res)
            pts = _smooth_closed(pts, k)
            pts, length = _resample_closed(pts, EDGE_STEP)
            if length < EDGE_MIN_LOOP:
                continue
            nx = np.roll(pts, -1, axis=0)
            area = 0.5 * float(np.sum(pts[:, 0] * nx[:, 1] - nx[:, 0] * pts[:, 1]))
            outer = hier is None or hier[0][ci][3] == -1
            if (area > 0) != outer:          # 外圈逆時針、內環順時針：割草範圍都在左邊
                pts = pts[::-1].copy()
            out.append(pts)
        return out

    def _opened(self, dist, d, r):
        """離邊界 >= d 的區域做半徑 r 的開運算 (外凸的角變成半徑 r 的圓弧，窄於 2r 的地方去掉)。"""
        core = dist >= d + r
        if not core.any():
            return np.zeros(dist.shape, bool)
        return _distance(~core, self.res) <= r

    def _ramp(self, p, yaw, off, length, entry, rot_need):
        """匝道：entry 時從 q 斜斜開到 p，否則從 p 斜斜開到 q。q 在 p 的後方 (或前方) length、往裡面 off。

        q 要能原地轉身 (離邊緣 >= rot_need、離牆夠遠)，整條匝道車身都要在範圍裡。不行回傳 None。
        回傳不含 p 的那些點 (entry：q ... p 之前；exit：p 之後 ... q)。
        """
        t = np.array([math.cos(yaw), math.sin(yaw)])
        n_in = np.array([-t[1], t[0]])
        offs = [off]
        if self.turn_wall > 0.0:
            # q 離牆不夠遠 (光達停止區原地轉一圈會掃到牆) 就再往裡面挪，匝道斜一點，最斜 EDGE_RAMP_MAX_ANGLE
            top = length * math.tan(EDGE_RAMP_MAX_ANGLE)
            more = np.arange(off + EDGE_RAMP_OFF_STEP, top + 1e-9, EDGE_RAMP_OFF_STEP)
            offs += [float(o) for o in more]
        for o in offs:
            q = p + (-length if entry else length) * t + o * n_in
            if float(self.clearance_at(q)) < rot_need or not bool(self.turn_ok(q)):
                continue
            line = _line(q, p, EDGE_STEP) if entry else _line(p, q, EDGE_STEP)
            hd = math.atan2(*(line[-1] - line[0])[::-1])
            if not self.poses_ok(line, np.full(len(line), hd)).all():
                continue
            return line[:-1] if entry else line[1:]
        return None

    def _ramp_lengths(self):
        """匝道長度依序試這些：要查離牆 (turn_wall) 時最後再試更長的。"""
        return EDGE_RAMPS + (EDGE_RAMPS_LONG if self.turn_wall > 0.0 else ())

    def _straightness(self, pts, yaw, closed, half):
        """每一點前後 half 個點之內，方向最多偏離這一點多少 (rad)。開放折線兩端以外當成無限大。"""
        n = len(pts)
        dev = np.zeros(n)
        idx = np.arange(n)
        for k in range(-half, half + 1):
            j = idx + k
            if closed:
                j %= n
                dev = np.maximum(dev, np.abs(_wrap(yaw[j] - yaw)))
            else:
                valid = (j >= 0) & (j < n)
                jj = np.clip(j, 0, n - 1)
                dev = np.maximum(dev, np.where(valid, np.abs(_wrap(yaw[jj] - yaw)), np.inf))
        return dev

    def _full_loop(self, loop, yaw, off, rot_need):
        """整圈都走得過去：挑直的地方當起點 (進出匝道都放得下)。回傳路線或 None。"""
        for length in self._ramp_lengths():
            half = int(round(length / EDGE_STEP))
            dev = self._straightness(loop, yaw, True, half)
            for i in np.argsort(dev, kind='stable')[:60]:
                if dev[i] > EDGE_RAMP_MAX_TURN:
                    break
                rin = self._ramp(loop[i], yaw[i], off, length, True, rot_need)
                if rin is None:
                    continue
                rout = self._ramp(loop[i], yaw[i], off, length, False, rot_need)
                if rout is None:
                    continue
                body = np.vstack([loop[i:], loop[:i], loop[i:i + 1]])
                return np.vstack([rin, body, rout]), len(rin), len(rout)
        return None

    def _open_piece(self, pts, yaw, off, rot_need):
        """斷開的一段 (開放折線)：從前面找得到進圈匝道的第一點、從後面找得到出圈匝道的最後一點。"""
        n = len(pts)
        need = int(round(EDGE_MIN_PIECE / EDGE_STEP))
        for length in self._ramp_lengths():
            half = int(round(length / EDGE_STEP))
            dev = self._straightness(pts, yaw, False, half)
            start = None
            for i in range(0, n - need):
                if dev[i] <= EDGE_RAMP_MAX_TURN:
                    rin = self._ramp(pts[i], yaw[i], off, length, True, rot_need)
                    if rin is not None:
                        start = (i, rin)
                        break
            if start is None:
                continue
            for j in range(n - 1, start[0] + need - 1, -1):
                if dev[j] <= EDGE_RAMP_MAX_TURN:
                    rout = self._ramp(pts[j], yaw[j], off, length, False, rot_need)
                    if rout is not None:
                        i, rin = start
                        return np.vstack([rin, pts[i:j + 1], rout]), len(rin), len(rout)
        return None

    def routes(self, offsets, safe_offset, rot_need):
        """每一個離邊界距離 d 的每一圈的路線：[{'d', 'pts' (M, 2), 'ramp_in', 'ramp_out', 'full', 'length'}]。

        safe_offset：匝道從離邊界這麼遠的地方開始 (地頭寬)；rot_need：匝道兩頭要能原地轉身的淨空。
        """
        out = []
        for d in offsets:
            off = max(0.0, float(safe_offset) - float(d))
            for loop in self.loops(d):
                yaw = _headings(loop, True)
                ok = self.poses_ok(loop, yaw)
                got = []
                if ok.all():
                    r = self._full_loop(loop, yaw, off, rot_need)
                    if r is not None:
                        got.append((r, True))
                else:
                    # 走不過去的地方斷開：一段一段 (環狀) 各自進出
                    n = len(loop)
                    if ok.any():
                        first = int(np.argmin(ok))           # 從一個走不過去的點後面開始數
                        order = (np.arange(n) + first) % n
                        run = []
                        for i in list(order) + [order[0]]:
                            if ok[i]:
                                run.append(i)
                                continue
                            if len(run) * EDGE_STEP >= EDGE_MIN_PIECE:
                                piece = loop[run]
                                r = self._open_piece(piece, _headings(piece, False), off, rot_need)
                                if r is not None:
                                    got.append((r, False))
                            run = []
                for (pts, nin, nout), full in got:
                    length = float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))
                    out.append({'d': float(d), 'pts': pts, 'ramp_in': nin, 'ramp_out': nout,
                                'full': full, 'length': length})
        return out

    def rotation_ok(self, xy, yaw0, yaw1, step=math.radians(5.0)):
        """在 xy 原地從 yaw0 轉到 yaw1 (走比較短的那一邊)，整個過程車身都在範圍裡嗎。"""
        dy = _wrap(yaw1 - yaw0)
        n = max(int(math.ceil(abs(dy) / step)), 1)
        yaws = yaw0 + dy * np.linspace(0.0, 1.0, n + 1)
        pts = np.repeat(np.asarray(xy, float).reshape(1, 2), len(yaws), axis=0)
        return bool(self.poses_ok(pts, yaws).all())

    def escape(self, xy, yaw, rot_need, max_len=3.0):
        """沿邊那一段失敗、車子停在離邊很近的地方時，往前斜斜開回能原地轉身的地方。

        方向試 yaw 偏 0、±10、±20、±30 度 (先試偏得少的)：原地轉到那個方向的過程、
        往前開的整條直線，車身都要在範圍裡；開到離邊緣 >= rot_need、離牆夠遠 (turn_ok) 就停
        (最多 max_len)。
        回傳 (直線的點, 方向)，找不到回傳 None。
        """
        p = np.asarray(xy, dtype=float)
        for deg in (0, 10, -10, 20, -20, 30, -30):
            hd = yaw + math.radians(deg)
            line = _line(p, p + max_len * np.array([math.cos(hd), math.sin(hd)]), EDGE_STEP)
            reach = np.flatnonzero((self.clearance_at(line) >= rot_need) & self.turn_ok(line))
            if len(reach) == 0:
                continue
            line = line[:max(int(reach[0]), 2) + 1]
            if not self.rotation_ok(p, yaw, hd):
                continue
            if self.poses_ok(line[1:], np.full(len(line) - 1, hd)).all():
                return line, hd
        return None


# ---------------------------------------------------------------------------
# 4. 原地轉身的地方，光達停止區不能掃到牆 (2026-10-21)
# ---------------------------------------------------------------------------
#
# 割草線兩頭、跑道起點、轉場的轉折點、周邊環繞的轉角都要原地轉身。以前只查車身 (原地轉一圈
# 車頭角掃出 1.06 m) 不碰邊界；但車頭前面還有光達停止區 (車頭前緣往前 0.40 m、左右各比車身寬
# 0.05 m)，原地轉身時它的外角掃出 hypot(0.975 + 0.40, 0.42 + 0.05) = 1.45 m 的圓。停止區碰到牆
# 車子就停住，不能倒車也不能原地轉，那一段 15 s 後判失敗，車子還是停在那裡。
# 邊界就是牆的地方 (室內、邊界取自地圖) 割草線盡頭離牆只有 1.30 m (地頭 1.20 + 腐蝕 0.10)：
# lab1 紅線當牆的模擬實跑，第 7 條割到盡頭時停止區已經碰到牆，轉不了身，連續三段失敗整趟停止。
# 所以要原地轉身的點離地圖上真的牆 (佔據格，太小的雜點不算，跟沿邊繞圈一樣) 至少
# 「停止區外角的半徑 + TURN_WALL_MARGIN」，割草線、環繞的直邊不夠的那一頭往裡面縮短。
# 開車圈的外框、沒有牆的草地：牆在外框外面至少半個車寬，割草線盡頭離牆 >= 1.72 m，完全不變。
# 只有光達停止區開著 (實車) 才查；模擬預設沒開，跟以前完全一樣。

# 到達判定 0.10 m (車子可能多開一點才停) + 停止區離牆 0.05 m
TURN_WALL_MARGIN = 0.15
# 縮短之後剩不到這麼長的就不割
TURN_MIN_PIECE = 0.5


def stop_zone_turn_radius(front, half_w, stop_len, stop_side):
    """原地轉身時光達停止區外角掃出的半徑 (後輪軸中心到停止區前角)。"""
    return math.hypot(float(front) + float(stop_len), float(half_w) + float(stop_side))


class WallClearance:
    """地圖上真的牆 (光達看得到) 的距離場：每一點離最近的牆多遠。

    walls：(bool 網格, 解析度, origin_x, origin_y)，跟 EdgeLoops 的一樣。
    """

    def __init__(self, walls, min_size=EDGE_WALL_MIN_SIZE):
        grid, res, ox, oy = walls
        self.res = float(res)
        self.origin = np.array([float(ox), float(oy)])
        wall = _big_walls(np.asarray(grid, bool), self.res, min_size)
        if wall.any():
            self.dist = _distance(~wall, self.res)
        else:
            self.dist = np.full(wall.shape, np.inf, np.float32)

    def at(self, xy):
        """每一點離牆多遠 (公尺，算到牆那一格的邊)；地圖外面當作沒有牆 (inf)。xy: (..., 2)。"""
        xy = np.asarray(xy, dtype=float)
        g = np.floor((xy - self.origin) / self.res).astype(np.int64)
        h, w = self.dist.shape
        inside = (g[..., 0] >= 0) & (g[..., 0] < w) & (g[..., 1] >= 0) & (g[..., 1] < h)
        out = np.full(inside.shape, np.inf)
        out[inside] = np.maximum(self.dist[g[inside][:, 1], g[inside][:, 0]] - 0.5 * self.res, 0.0)
        return out


def trim_ends(pts, ok, min_len=TURN_MIN_PIECE):
    """一條直線 (割草線、環繞的一邊) 兩頭要原地轉身：從兩頭往裡面找第一個能轉身的點。

    pts：(N, 2) 航點；ok：(N,) 每一點能不能原地轉身。回傳 (i0, i1) (含)：兩頭本來就行就是
    (0, N - 1)；縮短之後剩不到 min_len、或整條都不行回傳 None (這一條不割)。
    中間的點不用能轉身 (車子只是直直開過去)。
    """
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    idx = np.flatnonzero(np.asarray(ok, bool))
    n = len(pts)
    if len(idx) == 0:
        return None
    i0, i1 = int(idx[0]), int(idx[-1])
    if i0 == 0 and i1 == n - 1:
        return 0, n - 1
    if i1 <= i0 or float(np.sum(np.hypot(*np.diff(pts[i0:i1 + 1], axis=0).T))) < min_len:
        return None
    return i0, i1


def zone_sweep_clearance(walls, xy, yaw0, yaw1, x0, x1, half_w, step=math.radians(3.0)):
    """在 xy 原地從 yaw0 轉到 yaw1 (轉比較短的那一邊)，光達停止區掃過的地方離牆最近多遠 (公尺)。

    停止區 = 車頭前 x0..x1、左右 ±half_w 的長方形 (後輪軸中心座標)；walls：WallClearance。
    """
    dy = _wrap(yaw1 - yaw0)
    n = max(int(math.ceil(abs(dy) / step)), 1)
    box = _box_samples(x0, x1, half_w, 0.05)
    p = np.asarray(xy, dtype=float).reshape(2)
    best = float('inf')
    for a in yaw0 + dy * np.linspace(0.0, 1.0, n + 1):
        c, s = math.cos(a), math.sin(a)
        pts = p + np.column_stack([c * box[:, 0] - s * box[:, 1], s * box[:, 0] + c * box[:, 1]])
        best = min(best, float(walls.at(pts).min()))
    return best
