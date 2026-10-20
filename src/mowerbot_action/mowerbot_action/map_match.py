"""光達跟地圖對不對得上 (2026-10-21，純計算，不依賴 ROS)。

用存好的地圖開機 (slam_toolbox 定位模式) 時，車子在地圖上的起始位置是猜的
(上次關機時記下的位置，或建圖時出發的地方)。slam_toolbox 只在猜的位置附近 ±0.25 m、±20° 比對，
猜得太遠就會對錯，而且畫面上看起來一切正常 —— 禁區、外框、割草線全部跟著偏。

這裡做兩件事 (map_session 節點用)：
  * score：光達的點有多少落在地圖上的牆 (佔據格) 旁邊 —— 定位對了大部分都會落在牆上
  * locate：在猜的位置附近一格一格試，找光達跟地圖最對得上的位置 (給 slam_toolbox 當新的起點)

地圖原點的旋轉假設為 0 (與 keepout、map_to_boundary 相同)。
"""
import math

import cv2
import numpy as np

OCCUPIED = 65          # OccupancyGrid 的值 >= 這個算牆 (跟 map_server 的 occupied_thresh 0.65 一樣)
NEAR = 0.10            # 光達的點離牆這麼近以內算「對上」(m)
SIGMA = 0.08           # locate 的評分：離牆 d 的點得 exp(-d² / 2σ²)
MAX_POINTS = 360       # 一圈光達最多取這麼多點 (720 點的光達隔一點取一點)


class MapField:
    """地圖上每一格離最近的牆多遠 (m)。grid：OccupancyGrid 的 data (row 0 在 origin 那側)。"""

    def __init__(self, grid, res, ox, oy):
        g = np.asarray(grid)
        occ = g >= OCCUPIED
        self.res, self.ox, self.oy = float(res), float(ox), float(oy)
        self.h, self.w = occ.shape
        self.n_occupied = int(np.count_nonzero(occ))
        free = (~occ).astype(np.uint8)
        self.dist = cv2.distanceTransform(free, cv2.DIST_L2, cv2.DIST_MASK_PRECISE).astype(np.float32) * self.res
        self.far = float(max(self.h, self.w) * self.res)

    def distance_at(self, xy):
        """xy：(..., 2) 的 map 座標。地圖外面 = far。"""
        xy = np.asarray(xy, dtype=float)
        u = np.floor((xy[..., 0] - self.ox) / self.res).astype(np.int64)
        v = np.floor((xy[..., 1] - self.oy) / self.res).astype(np.int64)
        inside = (u >= 0) & (u < self.w) & (v >= 0) & (v < self.h)
        out = np.full(u.shape, self.far, dtype=np.float32)
        out[inside] = self.dist[v[inside], u[inside]]
        return out


def scan_points(ranges, angle_min, angle_inc, range_min, range_max, max_range=11.5, max_points=MAX_POINTS):
    """LaserScan -> 光達座標系的點 (N, 2)。無效、太近、太遠 (超過 max_range) 的不要；最多 max_points 點。"""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + np.arange(len(r)) * angle_inc
    ok = np.isfinite(r) & (r >= max(range_min, 0.05)) & (r <= min(range_max, max_range))
    r, a = r[ok], a[ok]
    if len(r) > max_points:
        k = np.linspace(0, len(r) - 1, max_points).round().astype(int)
        r, a = r[k], a[k]
    return np.column_stack([r * np.cos(a), r * np.sin(a)])


def transform(points, x, y, yaw):
    """(N, 2) 的點從車子座標系換到地圖：先轉 yaw 再平移 (x, y)。"""
    c, s = math.cos(yaw), math.sin(yaw)
    p = np.asarray(points, dtype=float)
    return np.column_stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]])


def score(field, points_map, near=NEAR):
    """(對上的比例, 點數)。點數 0 回傳 (0.0, 0)。"""
    if len(points_map) == 0:
        return 0.0, 0
    d = field.distance_at(points_map)
    return float(np.mean(d <= near)), int(len(points_map))


def _search(field, pts, x0, y0, yaw0, xy_half, xy_step, yaw_half, yaw_step):
    """在 (x0, y0, yaw0) 附近的格子點上找評分最高的位置。回傳 (x, y, yaw, 評分)。"""
    xs = x0 + np.arange(-xy_half, xy_half + 1e-9, xy_step)
    ys = y0 + np.arange(-xy_half, xy_half + 1e-9, xy_step)
    yaws = yaw0 + np.arange(-yaw_half, yaw_half + 1e-9, yaw_step)
    best = (x0, y0, yaw0, -1.0)
    inv = 1.0 / (2.0 * SIGMA * SIGMA)
    gx, gy = np.meshgrid(xs, ys, indexing='ij')
    for a in yaws:
        c, s = math.cos(a), math.sin(a)
        rx = c * pts[:, 0] - s * pts[:, 1]
        ry = s * pts[:, 0] + c * pts[:, 1]
        # (位置數, 點數)
        px = gx.reshape(-1, 1) + rx[None, :]
        py = gy.reshape(-1, 1) + ry[None, :]
        d = field.distance_at(np.stack([px, py], axis=-1))
        sc = np.exp(-(d * d) * inv).mean(axis=1)
        i = int(np.argmax(sc))
        if sc[i] > best[3]:
            best = (float(gx.reshape(-1)[i]), float(gy.reshape(-1)[i]), float(a), float(sc[i]))
    return best


def locate(field, points_base, guess, xy_half=1.5, yaw_half=math.radians(30.0)):
    """在猜的位置 guess = (x, y, yaw) 附近找光達跟地圖最對得上的位置。

    points_base：車子座標系 (base_footprint) 的光達點。先粗找 (0.10 m、3°)，再在最好的那一點附近細找
    (0.02 m、0.5°)。回傳 dict(x, y, yaw, score = 對上的比例, n = 點數, moved = 離猜的位置多遠)。
    """
    pts = np.asarray(points_base, dtype=float)
    x0, y0, yaw0 = (float(v) for v in guess)
    if len(pts) == 0:
        return {'x': x0, 'y': y0, 'yaw': yaw0, 'score': 0.0, 'n': 0, 'moved': 0.0}
    x, y, a, _s = _search(field, pts, x0, y0, yaw0, xy_half, 0.10, yaw_half, math.radians(3.0))
    x, y, a, _s = _search(field, pts, x, y, a, 0.10, 0.02, math.radians(3.0), math.radians(0.5))
    frac, n = score(field, transform(pts, x, y, a))
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return {'x': x, 'y': y, 'yaw': a, 'score': frac, 'n': n, 'moved': math.hypot(x - x0, y - y0)}
