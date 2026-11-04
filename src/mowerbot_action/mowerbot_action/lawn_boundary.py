"""草地邊界：戶外沒有牆時，用「開車繞草地邊緣一圈」定義割草範圍 (純計算，不依賴 ROS)。

背景 (2026-10-08)：map_to_boundary 把佔據網格裡最大的一塊自由區域當成草地。
室內或四周有牆時沒問題；戶外草地旁邊如果是水泥地、步道，2D 光達看不到
「草地在哪裡結束」，自由區域會一路延伸到光達掃得到的地方，F2C 會把水泥地也規劃進去。

做法：使用者在建圖模式下用手把沿著草地邊緣開一圈，boundary_recorder 記下車子的軌跡
(map 座標系的 base_footprint，也就是後輪軸中心)，收成一個多邊形發布到 /lawn_boundary；
map_to_boundary 把自由區域與這個多邊形取交集，之後的腐蝕、地頭、割草線都照舊。
交集只會讓作業區變小，不會變大：牆、障礙物的限制一個都沒有少。

這個檔案只放計算，節點在 boundary_recorder.py，單元測試在 test/test_lawn_boundary.py。
"""
import math

import cv2
import numpy as np
import yaml

# 軌跡取樣：車子移動超過這個距離才記一個點。0.10 m = 2 個 costmap cell。
MIN_STEP_M = 0.10
# 簡化容差，與 map_to_boundary.BOUNDARY_EPSILON_M 相同 (1 個 cell)。
SIMPLIFY_EPS_M = 0.05
# 停止記錄時，終點離起點超過這個距離就不當成「繞完一圈」，拒絕存檔。
# 不自動補一條直線把它接起來：缺口那一段使用者沒有開過，不知道那裡是不是草地。
MAX_CLOSE_GAP_M = 1.0
# 合理性下限。真正夠不夠割，由下游 (manager 的 min_boundary_area、F2C 的地頭) 判斷；
# 這裡只擋「按了開始馬上按停止」這種明顯不是一圈的軌跡。
MIN_AREA_M2 = 1.0


def append_if_moved(track, x, y, min_step=MIN_STEP_M):
    """車子離上一個記錄點超過 min_step 才把 (x, y) 加進 track，回傳有沒有加。"""
    if track:
        px, py = track[-1]
        if math.hypot(x - px, y - py) < min_step:
            return False
    track.append((float(x), float(y)))
    return True


# ---- 倒車擦掉 (2026-10-22 will：圈外框開錯了要能倒回去，不用整圈重來) ----
# 車子移動超過這個距離才判斷是往前還是倒退 (TF 抖動比這個小)
REVERSE_MIN_STEP_M = 0.03
# 倒車時離軌跡這麼近才擦 (沿著原路退回去)；離開原路倒車不會擦，也不會記
REVERSE_TOL_M = 0.40
# 只在最後這麼長的那一段裡找 (繞完一圈回到起點附近倒車時，不會找到起點那一頭)
REVERSE_LOOKBACK_M = 30.0


def motion_direction(prev_xy, xy, yaw, min_step=REVERSE_MIN_STEP_M):
    """從 prev_xy 移到 xy 是往車頭方向 (+1)、往車尾 (-1)，還是沒動夠遠或往旁邊滑 (0)。"""
    dx, dy = xy[0] - prev_xy[0], xy[1] - prev_xy[1]
    d = math.hypot(dx, dy)
    if d < min_step:
        return 0
    c = (dx * math.cos(yaw) + dy * math.sin(yaw)) / d
    if c > 0.5:
        return 1
    if c < -0.5:
        return -1
    return 0


def erase_back(track, x, y, tol=REVERSE_TOL_M, lookback=REVERSE_LOOKBACK_M):
    """倒車沿原路退回去：把軌跡上車子已經退過的那一截刪掉 (就地改 track)，回傳刪掉多長 (m)。

    從軌跡的尾巴往回找，第一個離 (x, y) 在 tol 以內、而且比前後都近的點 = 車子現在退到的地方，
    它後面的點都刪掉。從尾巴找、最多找 lookback 那麼長：開過的地方又經過一次
    (凹口兩邊、繞完一圈回到起點附近) 只會對到最近開過的那一次。找不到 (離開原路倒車) 不刪。
    """
    n = len(track)
    if n < 2:
        return 0.0

    def dist(i):
        return math.hypot(track[i][0] - x, track[i][1] - y)

    acc = 0.0
    k = None
    for i in range(n - 1, -1, -1):
        if i < n - 1:
            acc += math.hypot(track[i + 1][0] - track[i][0], track[i + 1][1] - track[i][1])
            if acc > lookback:
                break
        di = dist(i)
        if di <= tol and (i == 0 or dist(i - 1) >= di):
            k = i
            break
    if k is None or k >= n - 1:
        return 0.0
    gone = sum(math.hypot(track[i + 1][0] - track[i][0], track[i + 1][1] - track[i][1])
               for i in range(k, n - 1))
    del track[k + 1:]
    return gone


def polygon_area(poly):
    """Shoelace 公式，回傳面積 (正值)。"""
    n = len(poly)
    if n < 3:
        return 0.0
    acc = 0.0
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        acc += x1 * y2 - x2 * y1
    return abs(acc) * 0.5


def polygon_perimeter(poly):
    """閉合多邊形的周長。"""
    n = len(poly)
    return sum(math.hypot(poly[(i + 1) % n][0] - poly[i][0],
                          poly[(i + 1) % n][1] - poly[i][1]) for i in range(n))


def _orient(a, b, c):
    v = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    if abs(v) < 1e-12:
        return 0
    return 1 if v > 0 else -1


def _on_segment(a, b, p):
    return (min(a[0], b[0]) - 1e-12 <= p[0] <= max(a[0], b[0]) + 1e-12 and
            min(a[1], b[1]) - 1e-12 <= p[1] <= max(a[1], b[1]) + 1e-12)


def _segments_intersect(p1, p2, q1, q2):
    o1, o2 = _orient(p1, p2, q1), _orient(p1, p2, q2)
    o3, o4 = _orient(q1, q2, p1), _orient(q1, q2, p2)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and _on_segment(p1, p2, q1):
        return True
    if o2 == 0 and _on_segment(p1, p2, q2):
        return True
    if o3 == 0 and _on_segment(q1, q2, p1):
        return True
    if o4 == 0 and _on_segment(q1, q2, p2):
        return True
    return False


def ring_crossings(ring):
    """閉合折線 (最後一點接回第 0 點) 自己交叉、碰到的地方 (相鄰的兩段共用的點不算)。

    回傳 [(i, j, (x, y))]：第 i 段 (ring[i] -> ring[i + 1]) 跟第 j 段交叉，i < j。
    一圈幾百個點也很快 (每一段跟其他段一次用 numpy 算)。
    """
    p = np.asarray(ring, dtype=float).reshape(-1, 2)
    n = len(p)
    if n < 4:
        return []
    a, b = p, np.roll(p, -1, axis=0)                 # 第 k 段：a[k] -> b[k]
    d = b - a
    lo, hi = np.minimum(a, b), np.maximum(a, b)

    def cross(u, v):
        return u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]

    out = []
    for i in range(n - 2):
        j = np.arange(i + 2, n - (1 if i == 0 else 0))     # 第 0 段跟最後一段相鄰
        if len(j) == 0:
            continue
        # 外框對得上才細算
        j = j[np.all((lo[j] <= hi[i] + 1e-9) & (hi[j] >= lo[i] - 1e-9), axis=1)]
        if len(j) == 0:
            continue
        d1, d2 = cross(d[i], a[j] - a[i]), cross(d[i], b[j] - a[i])
        d3, d4 = cross(d[j], a[i] - a[j]), cross(d[j], b[i] - a[j])
        eps = 1e-12
        hit = (d1 * d2 <= eps) & (d3 * d4 <= eps)
        flat = (np.abs(d1) <= eps) & (np.abs(d2) <= eps)  # 在同一條直線上：要真的重疊才算
        if flat.any():
            u = d[i] / max(float(np.hypot(*d[i])), 1e-12)
            ti = sorted((0.0, float(np.dot(d[i], u))))
            ta, tb = (a[j] - a[i]) @ u, (b[j] - a[i]) @ u
            overlap = (np.maximum(ta, tb) >= ti[0] - 1e-9) & (np.minimum(ta, tb) <= ti[1] + 1e-9)
            hit &= ~flat | overlap
        for k in np.flatnonzero(hit):
            jj = int(j[k])
            den = d1[k] - d2[k]
            t = float(d1[k] / den) if abs(den) > eps else 0.0
            x = a[jj] + min(max(t, 0.0), 1.0) * d[jj]
            out.append((i, jj, (float(x[0]), float(x[1]))))
    return out


def self_intersects(poly):
    """閉合多邊形有沒有自己交叉 (相鄰邊共用的頂點不算)。

    軌跡繞成 8 字形時，fillPoly 填出來的區域跟使用者想的不一樣，所以直接拒絕。
    """
    return bool(ring_crossings(poly))


class SelfIntersectError(ValueError):
    """軌跡自己交叉。crossings = 交叉的地方 [(x, y)]；ring = 拿掉小圈之後的那一圈 (修一段的底)。"""

    def __init__(self, message, crossings=(), ring=None):
        super().__init__(message)
        self.crossings = list(crossings)
        self.ring = ring


def trim_overshoot(track, max_gap=MAX_CLOSE_GAP_M):
    """開回起點時多開過頭的那一段剪掉 (2026-10-20)。

    用手把開回起點，很難剛好停在起點上，多半會開過頭 0.3～0.5 m。過頭的那一段跟一開始
    的軌跡疊在一起，收成多邊形就變成自己交叉，按「完成」會失敗；又因為是繼續記錄，
    再往前開只會更糟，只能整圈重來。
    做法：只看最後一次開進起點 max_gap 範圍內的那一段 (從終點往回數、離起點都在
    max_gap 內的點)，取其中離起點最近的那一點當終點。中途經過起點附近的點不會被當成
    終點，所以剪掉的只會是最後那一小段。
    """
    if len(track) < 3:
        return list(track)
    x0, y0 = track[0]
    d = [math.hypot(x - x0, y - y0) for x, y in track]
    i = len(track) - 1
    if d[i] > max_gap:
        return list(track)
    while i > 0 and d[i - 1] <= max_gap:
        i -= 1
    if i == 0:              # 整條軌跡都沒離開起點附近，不是一圈，交給後面的檢查
        return list(track)
    end = min(range(i, len(track)), key=d.__getitem__)
    return list(track[:end + 1])


def close_and_simplify(track, max_gap=MAX_CLOSE_GAP_M, eps=SIMPLIFY_EPS_M,
                       min_area=MIN_AREA_M2):
    """把一圈軌跡收成閉合多邊形。

    回傳 dict(polygon=[(x, y), ...], gap=終點到起點距離, area=面積, perimeter=周長,
    knots=[(x, y, 面積)] 自動拿掉的小圈)。
    不合格時丟 ValueError，訊息直接給使用者看 (繁體中文、說明要怎麼做)。
    收出來自己交叉、而且是因為開過頭的話，剪掉過頭的那段再收一次 (見 trim_overshoot)；
    還是交叉的話，交叉圍出來的小圈拿掉 (remove_knots)；拿不掉的丟 SelfIntersectError，
    帶著交叉的地方，讓使用者只修那一段 (splice)。
    """
    try:
        return _close_once(track, max_gap, eps, min_area)
    except SelfIntersectError:
        pass
    trimmed = trim_overshoot(track, max_gap)
    if len(trimmed) != len(track):
        try:
            return _close_once(trimmed, max_gap, eps, min_area)
        except SelfIntersectError:
            pass
    res = finish_ring(trimmed, eps, min_area)
    res['gap'] = math.hypot(trimmed[-1][0] - trimmed[0][0], trimmed[-1][1] - trimmed[0][1])
    return res


def _simplify(ring, eps):
    pts = np.array(ring, dtype=np.float32).reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(pts, eps, True).reshape(-1, 2)
    poly = [(float(x), float(y)) for x, y in approx]
    if len(poly) >= 2 and poly[0] == poly[-1]:
        poly.pop()
    return poly


def _close_once(track, max_gap, eps, min_area):
    if len(track) < 3:
        raise ValueError('只記到 %d 個點，還不是一圈。請沿著草地邊緣開完一整圈再停止。'
                         % len(track))
    gap = math.hypot(track[-1][0] - track[0][0], track[-1][1] - track[0][1])
    if gap > max_gap:
        raise ValueError('終點離起點還有 %.2f m (上限 %.2f m)。請開回起點附近再停止。'
                         % (gap, max_gap))
    poly = _simplify(track, eps)
    if len(poly) < 3:
        raise ValueError('軌跡簡化後只剩 %d 個頂點，圍不出面積。' % len(poly))
    area = polygon_area(poly)
    if area < min_area:
        raise ValueError('圍出來的面積只有 %.2f m² (下限 %.2f m²)。' % (area, min_area))
    if self_intersects(poly):
        raise SelfIntersectError('軌跡自己交叉 (例如繞成 8 字形)，圍出來的範圍不明確。'
                                 '請取消後重新繞一圈。')
    return {'polygon': poly, 'gap': gap, 'area': area,
            'perimeter': polygon_perimeter(poly), 'knots': []}


# ---- 只修錯的那一段 (2026-10-28 will：圈外框失敗請使用者修復錯誤的地方就好，不要讓使用者整趟重跑) ----
# 10-24 will 第一次圈外框，按完成九次都是「軌跡自己交叉」，只能取消、整圈重開 (又開了 4 分鐘)。現在：
# 1. 交叉圍出來的小圈 (KNOT_MAX_AREA 以下：倒車、原地轉、開出去又開回來繞出來的) 自動拿掉。
# 2. 拿不掉的 (交叉兩邊都很大) 標出交叉的地方；使用者開到錯的那一段前面按「修一段」，沿正確的邊
#    開過去、接回原來的線 (FIX_MAX_GAP_M 內)，再按「完成」：新開的這一段換掉原來那一截 (splice)。
# 3. 已經圈好的外框也可以這樣修一段 (例如有一段開到草地外面、漏了一塊)。
KNOT_MAX_AREA = 1.0         # 交叉圍出來的圈比這個小 (m²) 就自動拿掉 (= MIN_AREA_M2)
FIX_MAX_GAP_M = 1.0         # 修的這一段頭尾離原來的線多近才接得上 (跟 MAX_CLOSE_GAP_M 一樣)
DENSE_STEP_M = 0.10         # 圈好的外框 (頂點) 補點的間距，修一段時找接頭用


def remove_knots(ring, max_area=KNOT_MAX_AREA):
    """自己交叉圍出來的小圈拿掉 (交叉的兩邊比較小的那一邊)。

    回傳 (新的一圈 [(x, y)], 拿掉的 [(x, y, 面積)], 拿不掉的交叉 [(x, y)])。
    一次拿掉一個最小的，再重找交叉；剩下的交叉兩邊都比 max_area 大時停下來。
    """
    ring = [(float(x), float(y)) for x, y in ring]
    removed = []
    while True:
        found = ring_crossings(ring)
        if not found:
            return ring, removed, []
        best = None
        for i, j, x in found:
            inner = ring[i + 1:j + 1]                   # 第 i 段跟第 j 段之間那一圈
            outer = ring[:i + 1] + [x] + ring[j + 1:]
            a_in, a_out = polygon_area([x] + inner), polygon_area(outer)
            small = min(a_in, a_out)
            if small < max_area and (best is None or small < best[0]):
                best = (small, i, j, x, a_in <= a_out)
        if best is None:
            return ring, removed, [x for _i, _j, x in found]
        small, i, j, x, inside = best
        ring = (ring[:i + 1] + [x] + ring[j + 1:]) if inside else ([x] + ring[i + 1:j + 1])
        removed.append((x[0], x[1], small))
        if len(ring) < 3:
            return ring, removed, []


def finish_ring(ring, eps=SIMPLIFY_EPS_M, min_area=MIN_AREA_M2):
    """一圈 (已經閉合、開回起點附近) 收成外框：拿掉小圈、簡化、檢查面積。

    回傳 dict(polygon, area, perimeter, knots, ring)；還有拿不掉的交叉丟 SelfIntersectError
    (crossings = 交叉的地方，ring = 拿掉小圈之後的那一圈)。
    簡化 (approxPolyDP) 會讓靠得很近的兩段碰在一起：碰到了就簡化少一點，最後用原來的點。
    """
    if len(ring) < 3:
        raise ValueError('只記到 %d 個點，還不是一圈。' % len(ring))
    clean, knots, left = remove_knots(ring)
    if left:
        raise SelfIntersectError(
            '外框有 %d 個地方交叉了 (地圖上的紅圈)，兩邊圍的都不小，不知道哪一邊才對。'
            '把車開到紅圈前面的外框線上，按「修一段」，沿著正確的草地邊緣開過紅圈、'
            '接回原來的線 (%.0f m 內) 再按「完成」，其他地方不用重開。'
            % (len(left), FIX_MAX_GAP_M), crossings=left, ring=clean)
    if len(clean) < 3 or polygon_area(clean) < min_area:
        raise ValueError('圍出來的面積只有 %.2f m² (下限 %.2f m²)。'
                         % (polygon_area(clean) if len(clean) >= 3 else 0.0, min_area))
    for e in (eps, eps * 0.4, 0.0):
        poly = _simplify(clean, e) if e > 0.0 else list(clean)
        if len(poly) >= 3 and not self_intersects(poly):
            break
    area = polygon_area(poly)
    if area < min_area:
        raise ValueError('圍出來的面積只有 %.2f m² (下限 %.2f m²)。' % (area, min_area))
    return {'polygon': poly, 'area': area, 'perimeter': polygon_perimeter(poly),
            'knots': knots, 'ring': clean}


def densify(poly, step=DENSE_STEP_M):
    """閉合多邊形的頂點之間補點 (間距 step)，回傳一圈的點 (最後一點不重複第 0 點)。"""
    out = []
    n = len(poly)
    for k in range(n):
        (ax, ay), (bx, by) = poly[k], poly[(k + 1) % n]
        m = max(int(math.ceil(math.hypot(bx - ax, by - ay) / step)), 1)
        out += [(ax + (bx - ax) * t / m, ay + (by - ay) * t / m) for t in range(m)]
    return out


def _anchors(ring, p, max_gap):
    """一圈上離 p 在 max_gap 內、而且比前後都近的點 (每經過一次 p 附近一個)。"""
    d = np.hypot(ring[:, 0] - p[0], ring[:, 1] - p[1])
    n = len(d)
    idx = [k for k in range(n)
           if d[k] <= max_gap and d[k] <= d[k - 1] and d[k] <= d[(k + 1) % n]]
    out = []
    for k in idx:                                   # 平平的一段 (一樣近) 只留一個
        if not out or k - out[-1] > 1:
            out.append(k)
    return out, float(d.min())


def splice(base, patch, max_gap=FIX_MAX_GAP_M):
    """修一段：patch (新開的一段，開放折線) 換掉 base (閉合的一圈) 上它頭尾之間的那一截。

    patch 的頭尾各自找 base 上最近的地方 (base 經過那附近好幾次時每一次都試)；換掉哪一截：
    換完交叉最少的，再來是換掉的那一截最短的 (修一小段，不會把整圈換掉)。開的方向跟原來的
    一樣或相反都可以。頭尾都在同一個地方 = 在原來的線外面多圍一塊 (換掉的那一截是 0)。
    回傳 dict(ring=新的一圈, replaced=換掉多長, added=接上多長, crossings=還有的交叉 [(x, y)])；
    接不上 (頭或尾離原來的線超過 max_gap) 丟 ValueError，訊息給使用者看。
    """
    ring = np.asarray(base, dtype=float).reshape(-1, 2)
    pt = np.asarray(patch, dtype=float).reshape(-1, 2)
    if len(pt) < 2:
        raise ValueError('修的這一段還沒開 (只記到 %d 個點)。' % len(pt))
    starts, d0 = _anchors(ring, pt[0], max_gap)
    if not starts:
        raise ValueError('修的這一段的起點離原來的外框 %.2f m (上限 %.2f m)，接不上。'
                         '取消這一段，開到外框線上再按「修一段」。' % (d0, max_gap))
    ends, d1 = _anchors(ring, pt[-1], max_gap)
    if not ends:
        raise ValueError('終點離原來的外框還有 %.2f m (上限 %.2f m)。開回原來的線上再按完成。'
                         % (d1, max_gap))
    n = len(ring)
    seg = np.hypot(*(np.roll(ring, -1, axis=0) - ring).T)
    total = float(seg.sum())
    cum = np.concatenate([[0.0], np.cumsum(seg)])[:n]      # cum[k] = 從第 0 點走到第 k 點多長

    def between(i, j):
        """一圈上從 i 往前走到 j，中間經過的點 (不含 i、j；i == j 時是其他所有的點)。"""
        k = (np.arange(1, (j - i) % n if j != i else n) + i) % n
        return [tuple(ring[m]) for m in k]

    def length(i, j):
        """從 i 往前走到 j 多長 (i == j 時是 0)。"""
        return float((cum[j] - cum[i]) % total)

    patch_pts = [tuple(p) for p in pt]
    best = None
    for ia in starts:
        for ib in ends:
            # 照原來的方向開：ia -> ib 那一截換掉；反方向開：ib -> ia 那一截換掉
            for new, gone in ((patch_pts + between(ib, ia), length(ia, ib)),
                              (between(ia, ib) + patch_pts[::-1],
                               length(ib, ia) if ia != ib else total)):
                if len(new) < 3 or polygon_area(new) < MIN_AREA_M2:
                    continue
                left = remove_knots(new)[2]
                key = (len(left), gone)
                if best is None or key < best[0]:
                    best = (key, new, gone, left)
    if best is None:
        raise ValueError('修的這一段接上去圍不出範圍，取消這一段再修一次。')
    _key, new, gone, left = best
    added = float(np.sum(np.hypot(*np.diff(pt, axis=0).T)))
    return {'ring': new, 'replaced': gone, 'added': added, 'crossings': left}


def rasterize(poly, origin_x, origin_y, resolution, width, height):
    """把 map 座標系的多邊形畫成跟佔據網格同尺寸的遮罩 (多邊形內 = 255)。

    座標換算與 map_to_boundary 一致：cell (u, v) 的世界座標 = origin + (u, v) x resolution，
    row = v、col = u (OccupancyGrid 的 data 是 row-major，row 0 在 origin 那一側)。
    地圖原點的旋轉假設為 0，跟 map_to_boundary 的假設相同。
    """
    mask = np.zeros((height, width), dtype=np.uint8)
    if len(poly) < 3:
        return mask
    pix = np.array([[(x - origin_x) / resolution, (y - origin_y) / resolution]
                    for x, y in poly])
    pix = np.round(pix).astype(np.int32).reshape(-1, 1, 2)
    cv2.fillPoly(mask, [pix], 255)
    return mask


def save_yaml(path, poly, frame_id, stamp_sec=None, session_id=None):
    """存成 yaml。session_id (2026-10-21)：哪一次建圖的座標系，boundary_recorder 開機時只載回同一個 session 的。"""
    data = {
        'frame_id': frame_id,
        'stamp_sec': stamp_sec,
        'session_id': session_id,
        'area_m2': round(polygon_area(poly), 3),
        'perimeter_m': round(polygon_perimeter(poly), 3),
        'polygon': [[round(x, 4), round(y, 4)] for x, y in poly],
    }
    with open(path, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)


def load_yaml(path, with_session=False):
    """回傳 (polygon, frame_id)；with_session 時回傳 (polygon, frame_id, session_id)。"""
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    poly = [(float(p[0]), float(p[1])) for p in data['polygon']]
    if with_session:
        return poly, data.get('frame_id', 'map'), data.get('session_id')
    return poly, data.get('frame_id', 'map')
