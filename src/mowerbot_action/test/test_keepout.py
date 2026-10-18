"""keepout 的單元測試 (不需要 ROS，pytest 直接跑)。"""
import cv2
import numpy as np
import pytest
import yaml

from mowerbot_action import keepout as ko


def make_map(tmp_path, w=40, h=30, res=0.05, ox=-1.0, oy=-0.5):
    """一張 w x h 的 map_saver 格式地圖：外圈一格牆 (0)、裡面空地 (254)、右上角未知 (205)。"""
    gray = np.full((h, w), 254, dtype=np.uint8)
    gray[0, :] = gray[-1, :] = gray[:, 0] = gray[:, -1] = 0
    gray[1:5, w - 6:w - 1] = 205
    cv2.imwrite(str(tmp_path / 'm.pgm'), gray)
    with open(tmp_path / 'm.yaml', 'w') as fh:
        yaml.safe_dump({'image': 'm.pgm', 'resolution': res, 'origin': [ox, oy, 0.0],
                        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.10}, fh)
    return gray


def test_template_keeps_map_colors_and_has_no_red(tmp_path):
    gray = make_map(tmp_path)
    t = ko.template_from_gray(gray)
    assert t.shape == gray.shape + (3,)
    assert (t[:, :, 0] == gray).all() and (t[:, :, 2] == gray).all()
    assert not ko.red_mask(t).any()


def test_red_detection_counts_antialiased_pink_but_not_dark_red():
    px = np.array([[[0, 0, 255], [150, 150, 255], [0, 0, 120], [205, 205, 205],
                    [48, 59, 255], [255, 0, 0], [0, 255, 0]]], dtype=np.uint8)   # BGR
    assert ko.red_mask(px).tolist() == [[True, True, False, False, True, False, False]]


def test_drawing_to_keepout_reports_other_colors_and_size_mismatch(tmp_path):
    gray = make_map(tmp_path)
    t = ko.template_from_gray(gray)
    d = t.copy()
    d[10:14, 10:16] = (0, 0, 255)          # 紅色 4 x 6
    d[20:22, 20:23] = (255, 0, 0)          # 藍色 2 x 3：不算
    red, other = ko.drawing_to_keepout(d, t)
    assert int(red.sum()) == 24 and other == 6
    with pytest.raises(ValueError, match='不要裁切'):
        ko.drawing_to_keepout(d[:-1], t)


def test_mask_roundtrip_flips_image_rows_into_grid_rows(tmp_path):
    gray = make_map(tmp_path)
    meta = ko.load_map_meta(str(tmp_path / 'm.yaml'))
    red = np.zeros(gray.shape, dtype=bool)
    red[2, 3] = True                        # 圖片第 2 列 (從上數) = 網格第 h-1-2 列
    red[25:28, 30:35] = True                # 另一塊
    path = ko.write_mask(str(tmp_path / 'm_keepout'), red, meta['resolution'],
                         meta['origin_x'], meta['origin_y'], 'S1', source='x.png')
    m = ko.load_mask(path)
    h = gray.shape[0]
    assert m['session_id'] == 'S1' and m['zones'] == 2
    assert m['area'] == pytest.approx(16 * 0.05 * 0.05)
    assert m['grid'][h - 1 - 2, 3]
    assert int(m['grid'].sum()) == 16
    # 圖片第 2 列、第 3 行那一格的世界座標
    x = m['origin_x'] + (3 + 0.5) * m['resolution']
    y = m['origin_y'] + (h - 1 - 2 + 0.5) * m['resolution']
    assert (x, y) == pytest.approx((-1.0 + 0.175, -0.5 + 1.375))


def test_resample_matches_world_coordinates_when_origin_moves():
    src = np.zeros((20, 20), dtype=bool)
    src[5:8, 10:12] = True                  # 世界座標 x 0.50~0.60、y 0.25~0.40 (origin 0,0、0.05)
    # 建圖中的地圖長大：origin 往左下移了 0.30 m、網格變大
    out = ko.resample(src, 0.05, 0.0, 0.0, 40, 40, 0.05, -0.30, -0.30)
    v, u = np.nonzero(out)
    assert out.sum() == 6
    assert (v.min(), v.max(), u.min(), u.max()) == (11, 13, 16, 17)
    # 完全不重疊
    assert not ko.resample(src, 0.05, 0.0, 0.0, 10, 10, 0.05, 5.0, 5.0).any()


def test_resample_handles_different_resolution():
    src = np.zeros((10, 10), dtype=bool)
    src[2, 3] = True                        # x 0.30~0.40, y 0.20~0.30 (0.10 m 一格)
    out = ko.resample(src, 0.10, 0.0, 0.0, 20, 20, 0.05, 0.0, 0.0)
    v, u = np.nonzero(out)
    assert sorted(zip(v.tolist(), u.tolist())) == [(4, 6), (4, 7), (5, 6), (5, 7)]


def test_rotated_origin_is_rejected(tmp_path):
    make_map(tmp_path)
    with open(tmp_path / 'r.yaml', 'w') as fh:
        yaml.safe_dump({'image': 'm.pgm', 'resolution': 0.05, 'origin': [0, 0, 0.3]}, fh)
    with pytest.raises(ValueError, match='旋轉'):
        ko.load_map_meta(str(tmp_path / 'r.yaml'))
