"""割草線的割草順序與繞過障礙物的轉場路線 (2026-10-21 will 選 1：分區割、繞過障礙)。純計算，不依賴 ROS。

背景：10-20 will 在 lab1 用紅線圍出範圍、中間畫一個方形禁區、樓梯旁邊畫一個 U 形凹口，
按 Y 預覽，路線一條一條斜著穿過禁區與樓梯。沙盒用真的 Fields2Cover 2.1.0 跑他那張圖重現了：
- F2C 的 BruteForce 把每一條割草線被禁區 / 凹口切成的兩段都放在同一個清單裡，f2c_server
  只是照清單順序「這條往前、下一條往回」，相鄰兩段常常在障礙物的兩邊，接起來就穿過障礙物。
- mower_manager 從車子到下一段起點的 approach 是一條直線，也會穿過去。
實車上光達看得到的會被 collision_monitor 擋下、禁區在 local costmap 裡是牆，DWB 走不過去就跳過那一段：
不會撞，但會漏割、亂跳。

這個檔案做兩件事：

1. plan_order：把割草線分成一塊一塊 (分區)，每一塊裡的割草線一列接一列、而且左右有重疊，
   在塊裡面照弓字形一來一回割；割完一塊再去離得最近的下一塊。這等於「弓字形分解」
   (boustrophedon decomposition)，只是直接用 F2C 擺好的割草線來分，割草線的位置與條數完全不變，
   只改割的順序與方向。F2C 原本的順序已經是好的弓字形 (沒有障礙物、沒有凹口) 時原樣回傳，
   跟以前一個位元都不差。

2. TransitPlanner：兩點之間的轉場路線。直線走得過去就走直線 (跟以前一樣)；走不過去就繞：
   在割草範圍裡找最短的折線，轉折點都選在「原地掉頭不會掃到邊界或障礙物」的地方
   (manager 每一段開始前會原地對正，轉折點就是對正的地方)。

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
    """

    def __init__(self, rings, travel_clearance, turn_clearance):
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
        self.dist = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * np.float32(res)
        self.travel_clearance = float(travel_clearance)
        self.turn_clearance = float(turn_clearance)
        self.travel = self.dist >= self.travel_clearance
        self.nodes = self._corner_nodes()
        self._adj = None
        self._cache = {}

    def _corner_nodes(self):
        """轉折點的候選：掉得了頭的區域 (離邊界 >= turn_clearance) 的外形頂點，往裡面再縮一格。

        最短的繞行路線只會在障礙物的角 (掉得了頭的區域往外凸的角) 轉彎，所以只留那些角：
        判斷方法是那個角兩邊的鄰點連線的中點落在區域外面 (區域在那裡凹進去)。
        """
        res = self.res
        inner = (self.dist >= self.turn_clearance + res).astype(np.uint8)
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
