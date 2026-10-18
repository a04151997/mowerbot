"""禁區：在存好的地圖圖片上用紅色畫的區域，割草與 Nav2 都不進去 (2026-10-18，純計算，不依賴 ROS)。

背景：光達只看得到它那一個高度 (lidar_z_ground，約 0.55 m) 的東西。玻璃門只掃得到門框，
地圖上是一條虛線，自由區域會從縫裡漏到門外；坑洞、低矮的石頭更是完全看不到。
光靠地圖，F2C 會把這些地方規劃進割草範圍。

做法：save_map.sh 存圖時多存一張跟 .pgm 同尺寸的圖片 (<名稱>_draw.png)，使用者用紅色
把牆、禁區畫上去；keepout_tool 把紅色的格子換成禁區遮罩交給 keepout_server 發布。
map_to_boundary 把遮罩裡的格子當成不能割 (會變成 F2C 的內環，或切掉外圈)，
local costmap 的 keepout_layer 把它當成牆，Nav2 也不往裡面開 (approach 是直線，可能穿過禁區)。

【只在同一次開機有效】每次 bringup 都重新建圖，map 座標系的原點是這次開機時車子停的位置
(與 boundary_recorder 的 load_on_start 是同一個理由)。所以存圖時記下這次開機的 session_id，
keepout_server 只接受同一個 session_id 畫出來的禁區。

座標：圖片與 slam_toolbox 存的 .pgm 同尺寸、同方向 (第 0 列 = 地圖最上方 = y 最大)。
yaml 的 origin 是左下角那一格的世界座標 (與 map_server 相同)；OccupancyGrid 的 data 是
row 0 在 origin 那一側，所以「圖片」與「網格」之間差一個上下翻轉。
地圖原點的旋轉假設為 0，與 map_to_boundary、lawn_boundary 相同。

這個檔案只放計算，節點在 keepout_server.py，命令列工具在 keepout_tool.py，
單元測試在 test/test_keepout.py。
"""
import os

import cv2
import numpy as np
import yaml

# ---- 什麼顏色算「紅色」 ----
# Mac 預覽程式、小畫家的紅色筆都是 R 接近 255、G 與 B 很低。筆畫邊緣有反鋸齒，跟白色混成
# 粉紅 (255, 150, 150) 仍算紅，跟黑色混成暗紅 (120, 0, 0) 不算。
# 地圖本身只有灰階 (R = G = B)，絕對不會被當成紅色。
RED_MIN = 150         # R 至少要這麼亮
RED_MARGIN = 80       # R 要比 G、B 都大這麼多

# 跟原本的地圖比，顏色差超過這個值、但不是紅色的像素：多半是用了別的顏色的筆。
# 只用來提醒使用者，不算禁區。
OTHER_CHANGE = 60

# 遮罩檔裡禁區的像素值 (map_server 的慣例：黑 = 佔據)，其餘是白
MASK_KEEPOUT = 0
MASK_FREE = 254


def load_map_meta(yaml_path):
    """讀 map_server 格式的 yaml，回傳 dict(image=絕對路徑, resolution, origin_x, origin_y, raw)。"""
    with open(yaml_path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise ValueError('%s 不是地圖的 yaml 檔' % yaml_path)
    for key in ('image', 'resolution', 'origin'):
        if key not in data:
            raise ValueError('%s 缺少 %s' % (yaml_path, key))
    image = str(data['image'])
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    origin = data['origin']
    if len(origin) > 2 and abs(float(origin[2])) > 1e-6:
        raise ValueError('地圖原點有旋轉 (yaw = %r)，不支援。' % origin[2])
    resolution = float(data['resolution'])
    if not resolution > 0.0:
        raise ValueError('解析度 %r 不合理' % data['resolution'])
    return {'image': image, 'resolution': resolution,
            'origin_x': float(origin[0]), 'origin_y': float(origin[1]), 'raw': data}


def read_gray(path):
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise ValueError('讀不到圖片 %s' % path)
    return img


def template_from_gray(gray):
    """存好的 .pgm (灰階) -> 給人畫的彩色圖 (BGR)。顏色不變：白 = 空地、黑 = 牆、灰 = 沒掃到。"""
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def red_mask(bgr):
    """圖片裡算紅色的像素 (bool，圖片方向)。"""
    img = bgr.astype(np.int16)
    b, g, r = img[:, :, 0], img[:, :, 1], img[:, :, 2]
    return (r >= RED_MIN) & (r - np.maximum(g, b) >= RED_MARGIN)


def drawing_to_keepout(drawn_bgr, template_bgr):
    """畫好的圖 -> (紅色遮罩, 被改過但不是紅色的像素數)。尺寸不同就丟 ValueError。"""
    dh, dw = drawn_bgr.shape[:2]
    th, tw = template_bgr.shape[:2]
    if (dh, dw) != (th, tw):
        raise ValueError(
            '畫好的圖是 %d x %d 像素，地圖是 %d x %d。請直接在存圖時給的那張圖上畫，'
            '不要裁切、縮放或旋轉。' % (dw, dh, tw, th))
    red = red_mask(drawn_bgr)
    diff = np.abs(drawn_bgr.astype(np.int16) - template_bgr.astype(np.int16)).max(axis=2)
    other = int(np.count_nonzero((diff > OTHER_CHANGE) & ~red))
    return red, other


def summarize(mask, resolution):
    """(禁區塊數, 總面積 m²)。8 連通算同一塊。"""
    n, _labels = cv2.connectedComponents(mask.astype(np.uint8), connectivity=8)
    return n - 1, float(np.count_nonzero(mask)) * resolution * resolution


def write_mask(prefix, mask, resolution, origin_x, origin_y, session_id, source=''):
    """把禁區遮罩 (bool，圖片方向) 存成 <prefix>.pgm + <prefix>.yaml，回傳 yaml 路徑。

    yaml 是 map_server 能讀的格式，另外多記 session_id 與統計。
    """
    pgm = np.full(mask.shape, MASK_FREE, dtype=np.uint8)
    pgm[mask] = MASK_KEEPOUT
    pgm_path = prefix + '.pgm'
    yaml_path = prefix + '.yaml'
    if not cv2.imwrite(pgm_path, pgm):
        raise ValueError('寫不出 %s' % pgm_path)
    zones, area = summarize(mask, resolution)
    data = {
        'image': os.path.basename(pgm_path),
        'mode': 'trinary',
        'resolution': resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
        'session_id': session_id,
        'zones': zones,
        'keepout_area_m2': round(area, 3),
        'source_image': source,
    }
    with open(yaml_path, 'w', encoding='utf-8') as fh:
        yaml.safe_dump(data, fh, allow_unicode=True, sort_keys=False)
    return yaml_path


def load_mask(yaml_path):
    """讀 write_mask 存的遮罩。回傳 dict(grid=bool 網格 (row 0 在 origin 那側), resolution,
    origin_x, origin_y, session_id, zones, area)。"""
    meta = load_map_meta(yaml_path)
    img = read_gray(meta['image'])
    grid = np.flipud(img < 128)
    zones, area = summarize(grid, meta['resolution'])
    return {'grid': np.ascontiguousarray(grid), 'resolution': meta['resolution'],
            'origin_x': meta['origin_x'], 'origin_y': meta['origin_y'],
            'session_id': meta['raw'].get('session_id'), 'zones': zones, 'area': area}


def resample(src, src_res, src_ox, src_oy, dst_w, dst_h, dst_res, dst_ox, dst_oy):
    """把一張網格 (bool，row 0 在 origin 那側) 換到另一張網格的格子上 (同為 map 座標系)。

    目標的每一格取它的中心點落在來源的哪一格。建圖中的地圖會長大，origin 會跟著變，
    所以不能假設兩張網格對齊，一律用世界座標對應。落在來源範圍外的格子 = False。
    """
    out = np.zeros((dst_h, dst_w), dtype=bool)
    src_h, src_w = src.shape
    xs = dst_ox + (np.arange(dst_w) + 0.5) * dst_res
    ys = dst_oy + (np.arange(dst_h) + 0.5) * dst_res
    cu = np.floor((xs - src_ox) / src_res).astype(np.int64)
    cv = np.floor((ys - src_oy) / src_res).astype(np.int64)
    ok_u = (cu >= 0) & (cu < src_w)
    ok_v = (cv >= 0) & (cv < src_h)
    if not ok_u.any() or not ok_v.any():
        return out
    out[np.ix_(ok_v, ok_u)] = src[np.ix_(cv[ok_v], cu[ok_u])]
    return out
