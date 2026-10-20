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


def self_intersects(poly):
    """閉合多邊形有沒有自己交叉 (相鄰邊共用的頂點不算)。

    軌跡繞成 8 字形時，fillPoly 填出來的區域跟使用者想的不一樣，所以直接拒絕。
    簡化後頂點通常只有幾十個，O(n^2) 沒問題。
    """
    n = len(poly)
    if n < 4:
        return False
    for i in range(n):
        a1, a2 = poly[i], poly[(i + 1) % n]
        for j in range(i + 1, n):
            if j == i or (j + 1) % n == i or (i + 1) % n == j:
                continue        # 相鄰邊
            b1, b2 = poly[j], poly[(j + 1) % n]
            if _segments_intersect(a1, a2, b1, b2):
                return True
    return False


class SelfIntersectError(ValueError):
    """軌跡自己交叉。"""


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

    回傳 dict(polygon=[(x, y), ...], gap=終點到起點距離, area=面積, perimeter=周長)。
    不合格時丟 ValueError，訊息直接給使用者看 (繁體中文、說明要怎麼做)。
    收出來自己交叉、而且是因為開過頭的話，剪掉過頭的那段再收一次 (見 trim_overshoot)。
    """
    try:
        return _close_once(track, max_gap, eps, min_area)
    except SelfIntersectError:
        trimmed = trim_overshoot(track, max_gap)
        if len(trimmed) == len(track):
            raise
        return _close_once(trimmed, max_gap, eps, min_area)


def _close_once(track, max_gap, eps, min_area):
    if len(track) < 3:
        raise ValueError('只記到 %d 個點，還不是一圈。請沿著草地邊緣開完一整圈再停止。'
                         % len(track))
    gap = math.hypot(track[-1][0] - track[0][0], track[-1][1] - track[0][1])
    if gap > max_gap:
        raise ValueError('終點離起點還有 %.2f m (上限 %.2f m)。請開回起點附近再停止。'
                         % (gap, max_gap))
    pts = np.array(track, dtype=np.float32).reshape(-1, 1, 2)
    approx = cv2.approxPolyDP(pts, eps, True).reshape(-1, 2)
    poly = [(float(x), float(y)) for x, y in approx]
    if len(poly) >= 2 and poly[0] == poly[-1]:
        poly.pop()
    if len(poly) < 3:
        raise ValueError('軌跡簡化後只剩 %d 個頂點，圍不出面積。' % len(poly))
    area = polygon_area(poly)
    if area < min_area:
        raise ValueError('圍出來的面積只有 %.2f m² (下限 %.2f m²)。' % (area, min_area))
    if self_intersects(poly):
        raise SelfIntersectError('軌跡自己交叉 (例如繞成 8 字形)，圍出來的範圍不明確。'
                                 '請取消後重新繞一圈。')
    return {'polygon': poly, 'gap': gap, 'area': area,
            'perimeter': polygon_perimeter(poly)}


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


def save_yaml(path, poly, frame_id, stamp_sec=None):
    """存檔只是留紀錄 (與除錯用)，見 boundary_recorder 裡 load_on_start 的說明。"""
    data = {
        'frame_id': frame_id,
        'stamp_sec': stamp_sec,
        'area_m2': round(polygon_area(poly), 3),
        'perimeter_m': round(polygon_perimeter(poly), 3),
        'polygon': [[round(x, 4), round(y, 4)] for x, y in poly],
    }
    with open(path, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)


def load_yaml(path):
    """回傳 (polygon, frame_id)。"""
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    poly = [(float(p[0]), float(p[1])) for p in data['polygon']]
    return poly, data.get('frame_id', 'map')
