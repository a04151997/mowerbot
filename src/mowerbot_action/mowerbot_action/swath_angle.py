"""割草線方向 (2026-10-19 will：割草線要順著長邊跑直線、到底再掉頭，不要斜著跑)。純計算，不依賴 ROS。

背景：f2c_server 固定用 BruteForce.generateSwaths(0.0, ...)，割草線永遠沿著 map 的 x 軸。
map 的 x 軸是開機時車頭的方向，跟場地的長邊沒有關係：lab1 的走廊跟 x 軸差 45°，
割草線就斜著切過走廊，每條只有幾公尺長，大部分時間花在掉頭。

做法 (不改 C++、不改 srv 介面)：mower_manager 先挑一個角度 θ，把邊界與內部障礙物轉 -θ
再交給 F2C，拿回來的路線 (位置與朝向) 再轉 +θ。Fields2Cover 2.1.0 的 generateSwaths(angle)
自己就是「繞原點轉 -angle、沿 x 產生割草線、再轉回 +angle」(swath_generator_base.cpp)，
地頭內縮 (ConstHL) 與周邊環繞 (buffer) 跟方向無關，所以這樣做等於叫 F2C 用角度 θ。

怎麼挑 θ：0° ~ 179° 每 1° 試一次，用跟 F2C 一樣的間距與起點擺出割草線，數有幾「段」。
每一段兩頭各要掉頭一次，段數越少掉頭越少；割草線總長大約是面積 / 間距，跟方向無關，
所以段數最少的方向就是最快的方向。長走廊會選順著走廊的方向，L 形會選段數少的那一種切法，
中間有禁區或障礙物時被切成兩段的線也算兩段。
同分時取垂直方向寬度最小的 (線的條數最少)，再同分取最接近 0° 的 (結果穩定)。

作業區用點陣算：把邊界塗成一張格子圖、挖掉內環，算每一格離邊界多遠；離邊界超過 margin
(= f2c_server 的地頭寬，F2C 是在內縮後的區域上擺割草線) 的才算作業區。每個角度把它轉正，
取割草線所在的那幾列數連續段。
"""
import math

import cv2
import numpy as np

# 格子圖的解析度：至少 0.05 m，場地大時放寬，讓格子圖一邊不超過約 MAX_CELLS 格 (一次規劃約 0.3 s)
MIN_RES = 0.05
MAX_CELLS = 500
# 比這個短的一段不算 (點陣的雜訊；F2C 在這麼窄的地方也擺不出像樣的一段)
MIN_RUN = 0.3
# 同一條線上比這個窄的空隙不算斷開 (格子圖的鋸齒)
MIN_GAP = 0.2


def parse_angle(value):
    """swath_angle 參數：'auto' 回傳 None，數字 (度) 回傳弧度。其他一律 ValueError，不要猜。"""
    v = str(value).strip().lower()
    if v == 'auto':
        return None
    try:
        deg = float(v)
    except ValueError:
        raise ValueError('swath_angle 只能是 auto 或角度 (度)，收到 %r' % (value,))
    if not math.isfinite(deg):
        raise ValueError('swath_angle 只能是 auto 或角度 (度)，收到 %r' % (value,))
    return math.radians(deg)


def rotate_xy(points, angle):
    """[(x, y), ...] 繞原點轉 angle (弧度，逆時針為正)。"""
    c, s = math.cos(angle), math.sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in points]


def rotate_quat(qx, qy, qz, qw, angle):
    """四元數 (x, y, z, w) 左乘「繞 z 轉 angle」：車頭方向跟著位置一起轉。"""
    sz, cz = math.sin(angle / 2.0), math.cos(angle / 2.0)
    return (cz * qx - sz * qy, cz * qy + sz * qx, cz * qz + sz * qw, cz * qw - sz * qz)


def _rasterize(rings):
    """作業區的距離場：每一格到邊界或內環的距離 (公尺，外面是 0)。回傳 (距離場, 中心, 解析度)。

    用距離場而不是 0 / 1 的格子圖：轉角度時用線性內插，「離邊界 margin」那條線轉完還是平滑的；
    0 / 1 的格子圖轉完邊緣有一格的鋸齒，貼著邊走的那條線會被切成很多小段。
    """
    outer = np.asarray(rings[0], dtype=float)
    lo, hi = outer.min(axis=0), outer.max(axis=0)
    center = (lo + hi) / 2.0
    radius = float(np.max(np.hypot(*(outer - center).T))) + 2 * MIN_RES
    res = max(MIN_RES, 2.0 * radius / MAX_CELLS)
    n = int(math.ceil(2.0 * radius / res)) + 2
    origin = center - n * res / 2.0

    def px(ring):
        p = (np.asarray(ring, dtype=float) - origin) / res - 0.5
        return np.round(p * 16).astype(np.int32).reshape(-1, 1, 2)   # 4 bit 小數 (shift=4)

    mask = np.zeros((n, n), np.uint8)
    cv2.fillPoly(mask, [px(rings[0])], 1, lineType=cv2.LINE_8, shift=4)
    for hole in rings[1:]:
        if len(hole) >= 3:
            cv2.fillPoly(mask, [px(hole)], 0, lineType=cv2.LINE_8, shift=4)
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    return dist * np.float32(res), center, res


def _rotated(field, angle):
    """把距離場轉成「割草線方向 = 行 (column 增加的方向)」：輸出第 b 列就是距中心 v 的那條線。"""
    n = field.shape[0]
    dx, dy = math.cos(angle), math.sin(angle)
    nx, ny = -dy, dx
    h = n / 2.0 - 0.5
    # 輸出格 (a, b) 對到輸入格 (col, row)：col = h + (a - h) dx + (b - h) nx，row 同理
    m = np.array([[dx, nx, h - h * (dx + nx)],
                  [dy, ny, h - h * (dy + ny)]], dtype=np.float64)
    return cv2.warpAffine(field, m, (n, n), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def _threshold(margin, res):
    """距離場大於這個值的格子就是作業區 (扣掉地頭之後)。margin = 0 時取半格，只去掉邊界本身。"""
    return max(float(margin), 0.5 * res)


def score_angle(field, res, angle, spacing, margin=0.0):
    """角度 angle 的 (段數, 垂直寬度, 割草線條數)。作業區是空的時回傳 None。

    割草線的位置照 F2C (BruteForce)：從作業區最低處往上 0.5、1.5、2.5 … 個間距，
    最後一條離頂端至少半個間距。
    """
    r = _rotated(field, angle) > _threshold(margin, res)
    rows = np.flatnonzero(r.any(axis=1))
    if rows.size == 0:
        return None
    vmin, vmax = rows[0] * res, (rows[-1] + 1) * res
    height = vmax - vmin
    min_cells = max(1, int(math.ceil(MIN_RUN / res)))
    gap_cells = max(2, int(math.ceil(MIN_GAP / res)))
    n_lines = max(0, int(math.ceil(height / spacing)) - 1)      # (k + 1) * spacing < height
    while n_lines > 0 and n_lines * spacing >= height:
        n_lines -= 1
    if n_lines == 0:
        return 0, height, 0
    b = ((vmin + (np.arange(n_lines) + 0.5) * spacing) / res).astype(int)
    sel = np.zeros((n_lines, r.shape[1] + 2), np.int8)
    sel[:, 1:-1] = r[b]
    d = np.diff(sel, axis=1)
    # 每一列的起點與終點個數相同、依序配對 (np.nonzero 是 row-major)
    sr, sc = np.nonzero(d > 0)
    _er, ec = np.nonzero(d < 0)
    if sr.size == 0:
        return 0, height, 0
    # 同一列上間隔小於 MIN_GAP 的兩段併成一段 (格子圖的雜訊，不是真的要掉頭)
    new = np.ones(sr.size, bool)
    new[1:] = (sr[1:] != sr[:-1]) | ((sc[1:] - ec[:-1]) >= gap_cells)
    last = np.append(new[1:], True)
    keep = (ec[last] - sc[new]) >= min_cells
    per_line = np.bincount(sr[new][keep], minlength=n_lines)
    segments = int(per_line.sum())
    lines = int(np.count_nonzero(per_line))
    return segments, height, lines


def choose_angle(rings, spacing, margin=0.0, step_deg=1.0):
    """挑掉頭最少的割草線方向。rings = [外圈, 內環 1, ...]，每個是 [(x, y), ...] (公尺)。

    回傳 (角度 (弧度，0 ~ π)，說明 dict)。作業區內縮之後是空的就回傳 (0.0, None)，
    讓 F2C 自己報「地頭之後沒有可作業面積」。
    """
    if spacing <= 0.0 or not rings or len(rings[0]) < 3:
        return 0.0, None
    field, _center, res = _rasterize(rings)
    if not (field > _threshold(margin, res)).any():
        return 0.0, None
    best = None
    n = int(round(180.0 / step_deg))
    for i in range(n):
        deg = i * step_deg
        s = score_angle(field, res, math.radians(deg), spacing, margin)
        if s is None:
            continue
        segments, height, lines = s
        if segments == 0:
            continue
        key = (segments, round(height / res), min(deg, 180.0 - deg))
        if best is None or key < best[0]:
            best = (key, deg, segments, lines, height)
    if best is None:
        return 0.0, None
    _key, deg, segments, lines, height = best
    s0 = score_angle(field, res, 0.0, spacing, margin)
    info = {'deg': deg, 'segments': segments, 'lines': lines, 'height': height,
            'segments_at_0': s0[0] if s0 else 0, 'res': res}
    return math.radians(deg), info
