# -*- coding: utf-8 -*-
"""存檔地圖裡「草坪內部」被標成佔據的格數 (階段 38，引擎遮擋 / 光達自身回波對建圖的影響)。

用法：python3 test/tools/map_interior.py <地圖 .yaml> [--world <世界檔>] [--margin 0.5]
草坪內部 = 世界檔自由區域裡、離任何牆 / 障礙物 > margin 的格子 (真實世界裡那裡什麼都沒有)。
在那裡被標成佔據 (pgm 值 < occupied_thresh) 的格子 = 地圖上的假障礙物。
"""
import argparse, math, os, sys
import numpy as np, cv2, yaml
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from coverage_budget import world_boxes

ap = argparse.ArgumentParser()
ap.add_argument('map_yaml')
ap.add_argument('--world', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                '..', '..', 'src/mowerbot_bringup/worlds/demo_lawn.world'))
ap.add_argument('--margin', type=float, default=0.5)
a = ap.parse_args()
meta = yaml.safe_load(open(a.map_yaml))
img = cv2.imread(os.path.join(os.path.dirname(a.map_yaml), meta['image']), cv2.IMREAD_GRAYSCALE)
res = meta['resolution']; ox, oy = meta['origin'][0], meta['origin'][1]
H, W = img.shape
# pgm 的第 0 列是地圖最上方 (y 最大)
occ_map = (255 - img.astype(np.float32)) / 255.0 > meta.get('occupied_thresh', 0.65)
unknown = img == 205
occ_w = np.zeros((H, W), np.uint8)
for _n, cx, cy, yaw, sx, sy in world_boxes(a.world):
    hx, hy = sx / 2, sy / 2; c, s = math.cos(yaw), math.sin(yaw)
    pts = [((cx + c * dx - s * dy - ox) / res, H - 1 - (cy + s * dx + c * dy - oy) / res)
           for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]
    cv2.fillConvexPoly(occ_w, np.array(pts, np.int32), 1)
dist = cv2.distanceTransform((occ_w == 0).astype(np.uint8), cv2.DIST_L2, 5) * res
free = (occ_w == 0)
n_lab, lab = cv2.connectedComponents(free.astype(np.uint8), 4)
c0 = lab[int(H - 1 - (0 - oy) / res), int((0 - ox) / res)]
interior = (lab == c0) & (dist > a.margin)
fake = occ_map & interior
print('%s：草坪內部 (離牆 > %.2f m) %d 格；其中被標成佔據 %d 格 (%.3f m²)；未知 %d 格 (%.2f m²)'
      % (os.path.basename(a.map_yaml), a.margin, int(interior.sum()), int(fake.sum()),
         fake.sum() * res * res, int((unknown & interior).sum()), (unknown & interior).sum() * res * res))
