"""路線上有東西擋住：先停、等一下、還在就從旁邊繞過去，最後回來補割 (2026-11-03 will)。

以前割草中前面有東西 (人、箱子) 只有光達停止區 (車頭前 0.40 m) 會停，停了 15 s Nav2 判那一段失敗、
整段跳過，路線不會重算，那一段沒割到的也不會回來補。現在 manager 在停止區之前就先看：

  1. 照著現在這一段往前走 look 公尺 (預設 1.3 m)，車身 (加側邊 0.05 m) 會碰到光達的點 -> 先停
     (跟手把斷線一樣每個 tick 送零，不取消這一段)。車子還會滑約 0.2 m，停在車頭前約 1.1 m。
  2. 等 wait 秒 (預設 10 s)：人通常就走開了，前面空了 1 s 就照原路接著走。
  3. 還在的話當成不會動的東西：把它 (看得到的那一面往後補一個深度) 往外擴 clearance (預設 1.0 m)
     當成轉場規劃器的內環，從旁邊繞到同一段後面、掉得了頭的地方接回去；中間沒割到的那一截排到最後補割。
     繞不過去 (靠邊、靠牆、太窄) 就把這一段剩下的排到最後補割，接著做下一段 (轉場也會繞開它)。
  4. 補割時又被擋住，等完就跳過，不再繞 (不會一直繞來繞去)。

光達停止區 (collision_monitor) 一直開著，是最後一道保險，這裡不改它。

這個檔只有幾何計算 (numpy，不碰 ROS)，什麼時候用、佇列怎麼改在 manager。
"""

import math

import numpy as np


def scan_to_map(ranges, angle_min, angle_increment, range_min, range_max, pose, max_range=8.0):
    """光達一圈的距離 -> 地圖座標的點 (M x 2)。

    pose = 光達在地圖上的 (x, y, yaw)。無效值 (inf、nan、太近、太遠) 丟掉；只留 max_range 以內
    (要判斷的只有車子附近，遠的點只會讓計算變慢)。
    """
    r = np.asarray(ranges, dtype=float)
    if r.size == 0:
        return np.zeros((0, 2))
    a = angle_min + angle_increment * np.arange(r.size)
    lo = max(float(range_min), 0.05)
    hi = float(range_max) if math.isfinite(range_max) and range_max > 0 else max_range
    hi = min(hi, max_range)
    ok = np.isfinite(r) & (r >= lo) & (r <= hi)
    r, a = r[ok], a[ok]
    x, y, yaw = pose
    return np.stack([x + r * np.cos(a + yaw), y + r * np.sin(a + yaw)], axis=1)


def _cum(pts):
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])


def project(pts, xy):
    """xy 投影到折線上：(沿線走了幾公尺, 離線多遠)。"""
    pts = np.asarray(pts, dtype=float)
    if len(pts) == 1:
        return 0.0, float(np.hypot(*(np.asarray(xy) - pts[0])))
    a, b = pts[:-1], pts[1:]
    d = b - a
    L2 = np.maximum((d * d).sum(axis=1), 1e-12)
    p = np.asarray(xy, dtype=float)
    t = np.clip(((p - a) * d).sum(axis=1) / L2, 0.0, 1.0)
    q = a + t[:, None] * d
    dist = np.hypot(*(p - q).T)
    i = int(np.argmin(dist))
    cum = _cum(pts)
    return float(cum[i] + t[i] * math.sqrt(L2[i])), float(dist[i])


def point_at(pts, s):
    """折線上走了 s 公尺的位置與方向 (x, y, yaw)；s 超出頭尾時夾在頭尾。"""
    pts = np.asarray(pts, dtype=float)
    cum = _cum(pts)
    if len(pts) < 2:
        return float(pts[0][0]), float(pts[0][1]), 0.0
    s = min(max(s, 0.0), float(cum[-1]))
    i = int(np.searchsorted(cum, s, side='right') - 1)
    i = min(max(i, 0), len(pts) - 2)
    seg = cum[i + 1] - cum[i]
    t = 0.0 if seg < 1e-12 else (s - cum[i]) / seg
    a, b = pts[i], pts[i + 1]
    j = i
    while j < len(pts) - 2 and np.hypot(*(pts[j + 1] - pts[j])) < 1e-9:
        j += 1
    yaw = math.atan2(pts[j + 1][1] - pts[j][1], pts[j + 1][0] - pts[j][0])
    return float(a[0] + t * (b[0] - a[0])), float(a[1] + t * (b[1] - a[1])), yaw


def _inside_box(points, pose, front, rear, half):
    """points 在車身方塊 (pose 那個位置、x 從 -rear 到 front、y 正負 half) 裡面的 bool 陣列。"""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx = points[:, 0] - x
    dy = points[:, 1] - y
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return (lx >= -rear) & (lx <= front) & (np.abs(ly) <= half)


def _normal_offset(path, s0, xy):
    """車子在這一段的左邊 (+) 還是右邊 (-) 多遠。"""
    px, py, yaw = point_at(path, s0)
    return -math.sin(yaw) * (xy[0] - px) + math.cos(yaw) * (xy[1] - py)


def blocked_ahead(path_pts, robot_pose, points, front, rear, half_w, side=0.05, look=1.3,
                  step=0.1, min_points=3):
    """車子接著照 path_pts 走 look 公尺，車身方塊 (側邊加 side) 會碰到哪些點。

    車子不一定剛好在線上 (割草線開頭會偏 0.28 m 左右、慢慢收回來)，所以同時算「在線上」和
    「跟現在一樣偏在線的旁邊」兩種，哪一種碰到都算。轉彎的地方跟著線轉 (沿邊那幾圈)，
    不是直直往前看：直直往前看的話沿邊轉角前面的牆會被當成擋路。

    回傳 (gap, idx)：gap = 開多遠就會碰到 (約等於車頭到障礙物的距離，公尺)，idx = 會碰到的點 (points 的
    索引)；碰到的點不到 min_points 個 (光達雜訊、一兩個反光點) 回傳 (None, 空陣列)。
    現在就已經在車身方塊裡的點不算 (光達掃到自己的車架；真的貼著車子的東西由停止區、急停處理)。
    只看這一段剩下的部分：這一段走完車子會停在終點 (車頭在終點前方 front)，接著原地轉，
    那是對正 / 下一段的事 (停止區照樣看著)；割草線終點離牆至少 1.6 m，所以牆不會被當成擋路。
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    none = (None, np.zeros(0, dtype=int))
    if len(pts) < min_points or len(path_pts) < 2:
        return none
    rx, ry, ryaw = robot_pose
    reach = look + front + half_w + side + 0.5
    near = np.flatnonzero(np.hypot(pts[:, 0] - rx, pts[:, 1] - ry) <= reach)
    if len(near) < min_points:
        return none
    half = half_w + side
    sub = pts[near]
    keep = ~_inside_box(sub, robot_pose, front, rear, half)
    near, sub = near[keep], sub[keep]
    if len(near) < min_points:
        return none
    path = np.asarray(path_pts, dtype=float)
    total = float(_cum(path)[-1])
    s0, _d = project(path, (rx, ry))
    off = _normal_offset(path, s0, (rx, ry))
    first = np.full(len(near), np.inf)
    n = max(1, int(math.ceil(look / step)))
    for k in range(1, n + 1):
        t = min(k * step, look)
        if s0 + t > total + 1e-9:
            t = total - s0
            if t <= 1e-6:
                break
        px, py, pyaw = point_at(path, s0 + t)
        poses = [(px, py, pyaw)]
        if abs(off) > 0.02:
            poses.append((px - off * math.sin(pyaw), py + off * math.cos(pyaw), pyaw))
        for pose in poses:
            hit = _inside_box(sub, pose, front, rear, half) & ~np.isfinite(first)
            first[hit] = t
    contact = np.isfinite(first)
    if int(contact.sum()) < min_points:
        return none
    return float(first[contact].min()), near[contact]


def cluster(points, seeds, link=0.25, max_extent=2.0):
    """從 seeds (points 的索引) 開始，把相鄰 link 公尺以內的點連成一團 (人兩條腿、箱子看得到的那一面)。

    只連到離 seeds 中心 max_extent 以內：碰到牆的話不會沿著牆連一整排。回傳那一團的點 (K x 2)。
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    seeds = np.asarray(seeds, dtype=int)
    if len(seeds) == 0:
        return np.zeros((0, 2))
    center = pts[seeds].mean(axis=0)
    cand = np.flatnonzero(np.hypot(*(pts - center).T) <= max_extent)
    member = np.zeros(len(pts), bool)
    member[seeds] = True
    frontier = pts[seeds]
    rest = np.setdiff1d(cand, seeds)
    while len(frontier) and len(rest):
        d = np.hypot(pts[rest][:, None, 0] - frontier[None, :, 0],
                     pts[rest][:, None, 1] - frontier[None, :, 1]).min(axis=1)
        add = rest[d <= link]
        if len(add) == 0:
            break
        member[add] = True
        frontier = pts[add]
        rest = np.setdiff1d(rest, add)
    return pts[member]


def convex_hull(pts):
    """凸包 (逆時針，不重複第一點)。"""
    p = sorted(set((round(float(x), 4), round(float(y), 4)) for x, y in pts))
    if len(p) <= 2:
        return p

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(p):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def solid(pts, sensor_xy, depth=0.5):
    """光達只看得到朝向它的那一面：每一點順著光線往後多補 depth 公尺 (箱子的厚度、人的身體)。"""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) == 0 or depth <= 0.0:
        return pts
    v = pts - np.asarray(sensor_xy, dtype=float)
    n = np.maximum(np.hypot(*v.T), 1e-6)
    return np.vstack([pts, pts + depth * v / n[:, None]])


def hole_polygon(pts, clearance, sides=16):
    """點的凸包往外擴 clearance 公尺 (凸包加圓的 Minkowski 和，再取凸包)：給 TransitPlanner 當內環。"""
    hull = convex_hull(pts)
    if not hull:
        return []
    ang = np.linspace(0.0, 2.0 * math.pi, sides, endpoint=False)
    # 多邊形的邊在兩個頂點之間會往內縮一點 (cos(pi/sides))，半徑放大到邊也在 clearance 以外
    r = clearance / math.cos(math.pi / sides)
    ring = [(x + r * math.cos(a), y + r * math.sin(a)) for x, y in hull for a in ang]
    return [(float(x), float(y)) for x, y in convex_hull(ring)]


def segment_gap(a, b, pts, step=0.05):
    """a -> b 這條線離這些點的凸包最近多遠 (碰到、穿過是 0；沒有點是無限大)。

    轉場規劃器找不到路、要照舊走直線的時候用：直線會從這一趟擋過路的東西旁邊擦過去就不走。"""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    hull = np.asarray(convex_hull(pts), dtype=float).reshape(-1, 2)
    if len(hull) == 0:
        return float('inf')
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = max(2, int(math.ceil(float(np.hypot(*(b - a))) / step)) + 1)
    q = a + np.linspace(0.0, 1.0, n)[:, None] * (b - a)          # 線上的點 (n, 2)
    e0 = hull
    e1 = np.roll(hull, -1, axis=0)
    d = e1 - e0                                                    # 凸包的邊 (m, 2)
    L2 = np.maximum((d * d).sum(1), 1e-12)
    t = np.clip(((q[:, None, :] - e0[None]) * d[None]).sum(2) / L2[None], 0.0, 1.0)
    near = e0[None] + t[..., None] * d[None]
    dist = np.hypot(*(q[:, None, :] - near).transpose(2, 0, 1)).min(1)
    if len(hull) >= 3:
        # 逆時針凸包：每一條邊都在左手邊 = 點在裡面
        rx = q[:, None, 0] - e0[None, :, 0]
        ry = q[:, None, 1] - e0[None, :, 1]
        cr = d[None, :, 0] * ry - d[None, :, 1] * rx
        dist[(cr >= 0.0).all(1)] = 0.0
    return float(dist.min())


def min_distance(pts, xy):
    """xy 離這些點最近多遠 (沒有點是無限大)。"""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return float('inf')
    return float(np.hypot(pts[:, 0] - xy[0], pts[:, 1] - xy[1]).min())


def far_side(path_pts, pts):
    """這些點投影到這一段上最遠到哪裡 (沿線公尺)：要接回去的點至少在這之後。"""
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    if len(pts) == 0:
        return 0.0
    return max(project(path_pts, p)[0] for p in pts)


def rejoin_index(path_pts, s_from, ok, min_rest=0.5):
    """這一段在 s_from (沿線公尺) 之後第一個 ok(x, y) 成立的航點 (從那裡接回去)，而且後面至少還有
    min_rest 公尺 (剩不到就不用接，直接算這一段做到這裡)。找不到回傳 None。"""
    path = np.asarray(path_pts, dtype=float)
    cum = _cum(path)
    if len(path) < 2:
        return None
    for i in range(len(path) - 1):
        if cum[i] < s_from:
            continue
        if cum[-1] - cum[i] < min_rest:
            return None
        if ok(float(path[i][0]), float(path[i][1])):
            return i
    return None
