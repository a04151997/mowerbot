# -*- coding: utf-8 -*-
"""車輛幾何的單一來源載入器 (階段 38)。

所有讀 vehicle.yaml 的地方 (launch、manager、bridge、geometry_guard、測試、工具)
一律透過 load()，不要自己 yaml.safe_load。衍生量只在這個檔案裡算一次。

三個半徑的用途固定，不得互換 (階段 38 原則 2)：
  costmap_inscribed_radius  0.155      僅供 Nav2 inflation (costmap 自己會從 footprint 算出同一個數)
  lateral_half_extent       0.420      直線行進的通過門檻 (manager)
  rotation_swept_radius     1.0616143  需要原地旋轉的點 (manager、headland)
淨空檢查只接受後兩者 (smoke_test A6 會掃原始碼確認)。

f2c_server (C++) 不讀這個檔：headland_width 與 rotation_swept_radius 由
mower_control.launch.py 從這裡取出後當參數傳進去，f2c_server 只做斷言。
"""
import math
import os

import yaml

PROVENANCE_VALUES = ('measured', 'measured_coarse', 'provisional', 'derived')
IMPACT_VALUES = ('high', 'medium', 'low')
_IMPACT_ORDER = {'high': 0, 'medium': 1, 'low': 2}

# ---- 不是車輛量測值、但屬於幾何來源的常數 (只在這裡定義一次) ----
# headland = ceil((rotation_swept_radius + TURN_MARGIN) / HEADLAND_STEP) x HEADLAND_STEP
# TURN_MARGIN 0.12：沿用舊值 0.70 - 0.5841 ≈ 0.1159 的同等絕對餘裕。
TURN_MARGIN = 0.12
HEADLAND_STEP = 0.05
# 啟動斷言：headland_width >= rotation_swept_radius + HEADLAND_MIN_MARGIN (f2c_server 也用同一個值)
HEADLAND_MIN_MARGIN = 0.10
# local_costmap 的 inflation_radius。本輪固定值，待重新推導 (柔性避讓，不是安全機制)。
# nav2 (navigation.launch.py 注入)、manager 的跑道離障礙物門檻、Phase Q 全部讀這一個。
# 必須 > lateral_half_extent，否則梯度在車身邊緣之前就衰減完 (load() 斷言)。
SOFT_INFLATION_RADIUS = 0.45

# 階段 38 的手算值。vehicle.yaml 的值改了之後這三個斷言會擋下來，
# 要重新手算一次再更新，確認推導公式沒有被改壞。
_EXPECTED_ROTATION_SWEPT = 1.0616143
_EXPECTED_INSCRIBED = 0.155
_EXPECTED_LATERAL = 0.420

# 量測工單 (docs/measurement_worklist.md) 的小節標題就是鍵名
WORKLIST_DOC = 'docs/measurement_worklist.md'


class GeometryError(RuntimeError):
    pass


def default_path():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('mowerbot_description'),
                        'config', 'vehicle.yaml')


def _headland(rotation_swept_radius):
    """headland 的進位規則 (只在這裡)。round(...,9) 吸收浮點誤差，免得 1.2000000001 進位成 1.25"""
    required = rotation_swept_radius + TURN_MARGIN
    return round(math.ceil(round(required / HEADLAND_STEP, 9)) * HEADLAND_STEP, 6)


def _extents(L, r_rear, W):
    """footprint 的三個伸出量 (base_link = 後輪軸中心)，回傳 (front, rear, lateral)。

    front_extent = wheelbase + front_wheel_radius
                 = (L - r_rear - r_front) + r_front
                 = L - r_rear
    【front_wheel_radius 在這個推導裡完全抵消】：前輪半徑的量測誤差對 footprint 沒有任何影響，
    不需要為了 footprint 精度去重量前輪。它只影響 wheelbase (腳輪在 URDF 的位置)。
    """
    return L - r_rear, r_rear, W / 2.0


class VehicleGeometry:
    """vehicle.yaml 的原始值 + 衍生量。屬性名稱就是 YAML 的鍵名。"""

    def __init__(self, raw, path):
        self.path = path
        self.provenance = dict(raw.pop('provenance', None) or {})
        self.uncertainty = dict(raw.pop('uncertainty', None) or {})
        self.impact = dict(raw.pop('impact', None) or {})
        self.values = raw
        self._validate()
        for k, v in raw.items():
            setattr(self, k, v)
        self._derive()
        self.ranges = self.propagate_uncertainty()

    # ------------------------------------------------------------------
    def _validate(self):
        missing = [k for k in self.values if k not in self.provenance]
        if missing:
            raise GeometryError(
                '%s：下列幾何值在 provenance 沒有對應項：%s'
                % (self.path, ', '.join(missing)))
        orphan = [k for k in self.provenance if k not in self.values]
        if orphan:
            raise GeometryError(
                '%s：provenance 列了不存在的幾何值：%s' % (self.path, ', '.join(orphan)))
        bad = ['%s=%r' % (k, v) for k, v in self.provenance.items()
               if v not in PROVENANCE_VALUES]
        if bad:
            raise GeometryError(
                '%s：provenance 只允許 %s，下列不合法：%s'
                % (self.path, ' / '.join(PROVENANCE_VALUES), ', '.join(bad)))
        no_unc = [k for k, v in self.provenance.items()
                  if v == 'measured_coarse'
                  and (isinstance(self.uncertainty.get(k), bool)
                       or not isinstance(self.uncertainty.get(k), (int, float)))]
        if no_unc:
            raise GeometryError(
                '%s：下列 measured_coarse 項目在 uncertainty 沒有誤差值：%s'
                % (self.path, ', '.join(no_unc)))
        no_imp = [k for k, v in self.provenance.items()
                  if v in ('provisional', 'measured_coarse')
                  and self.impact.get(k) not in IMPACT_VALUES]
        if no_imp:
            raise GeometryError(
                '%s：下列 provisional / measured_coarse 項目缺少 impact (high / medium / low)：%s'
                % (self.path, ', '.join(no_imp)))
        for k, v in self.values.items():
            if k == 'front_wheel_type':
                continue
            if isinstance(v, bool) or not isinstance(v, (int, float)):
                raise GeometryError('%s：%s = %r 不是數字' % (self.path, k, v))

    def _derive(self):
        # base_link = 後輪軸中心，x 朝前
        self.wheelbase = (self.body_length_total - self.rear_wheel_radius
                          - self.front_wheel_radius)
        self.front_extent, self.rear_extent, self.lateral_half_extent = _extents(
            self.body_length_total, self.rear_wheel_radius, self.body_width_total)
        hw = self.lateral_half_extent
        self.footprint = [[self.front_extent, hw], [self.front_extent, -hw],
                          [-self.rear_extent, -hw], [-self.rear_extent, hw]]
        self.rotation_swept_radius = math.hypot(self.front_extent, hw)
        self.costmap_inscribed_radius = min(self.front_extent, self.rear_extent, hw)
        self.headland_width = _headland(self.rotation_swept_radius)
        self.soft_inflation_radius = SOFT_INFLATION_RADIUS
        # 光達相對 base_link 的高度 (base_link 離地 rear_wheel_radius)
        self.lidar_z = self.lidar_z_ground - self.rear_wheel_radius
        # 引擎前後中心 = 引擎曲軸 = 刀盤中心 (同心)，不另設一個值
        self.engine_center_x = self.blade_offset_x
        # 周邊環繞的轉角偵測 (階段 38 決定 3)：在一個車身長度的弧長內累積轉角
        # >= 「車身長 / 掉頭半徑」(弧度) 就視為轉角、必須切開 —— 等於曲率半徑小於 rotation_swept_radius
        # 的彎，車子無法邊走邊跟。目前 1.13 / 1.0616 = 1.064 rad ≈ 61.0°。
        self.perimeter_corner_window = self.body_length_total
        self.perimeter_corner_angle = self.body_length_total / self.rotation_swept_radius

        # ---- 啟動斷言 ----
        if abs(self.rotation_swept_radius - _EXPECTED_ROTATION_SWEPT) >= 1e-6:
            raise GeometryError(
                'rotation_swept_radius = %.7f，與階段 38 手算值 %.7f 不符。'
                'vehicle.yaml 的值改了的話，重新手算後更新 vehicle_geometry.py 的 '
                '_EXPECTED_ROTATION_SWEPT。' % (self.rotation_swept_radius,
                                                _EXPECTED_ROTATION_SWEPT))
        if abs(self.costmap_inscribed_radius - _EXPECTED_INSCRIBED) >= 1e-9:
            raise GeometryError(
                'costmap_inscribed_radius = %.9f，與階段 38 手算值 %.3f 不符。'
                % (self.costmap_inscribed_radius, _EXPECTED_INSCRIBED))
        if abs(self.lateral_half_extent - _EXPECTED_LATERAL) >= 1e-9:
            raise GeometryError(
                'lateral_half_extent = %.9f，與階段 38 手算值 %.3f 不符。'
                % (self.lateral_half_extent, _EXPECTED_LATERAL))
        if not self.soft_inflation_radius > self.lateral_half_extent:
            raise GeometryError(
                'soft_inflation_radius %.3f <= lateral_half_extent %.3f：'
                'inflation 的梯度會在車身邊緣之前衰減完，柔性避讓失效。'
                % (self.soft_inflation_radius, self.lateral_half_extent))
        if self.headland_width < self.rotation_swept_radius + HEADLAND_MIN_MARGIN - 1e-9:
            raise GeometryError(
                'headland_width %.4f < rotation_swept_radius %.4f + %.2f'
                % (self.headland_width, self.rotation_swept_radius, HEADLAND_MIN_MARGIN))
        self._consistency()
        # 階段 31 的斷言改用實測車寬：車寬必須把後輪整個包住
        wheel_span = self.wheel_separation + self.rear_wheel_width
        if self.body_width_total < wheel_span - 1e-9:
            raise GeometryError(
                'body_width_total = %.4f m 比後輪外緣寬度還窄：wheel_separation %.4f + '
                'rear_wheel_width %.4f = %.4f m。footprint 沒有包住輪子。'
                % (self.body_width_total, self.wheel_separation,
                   self.rear_wheel_width, wheel_span))

    # ------------------------------------------------------------------
    def engine_occlusion(self):
        """引擎方塊對光達掃描面的遮擋：回傳 (遮擋角 deg, 方向說明)；沒有遮擋回傳 (0.0, '無')。

        光達在 (lidar_x, 0)，引擎方塊 x 在 engine_center_x ± engine_length/2、y 在 ±engine_width/2。
        引擎頂高於掃描面且光達在引擎前 / 後時，遮擋角 = 2 atan((engine_width/2) / 到近面的距離)。
        """
        if self.engine_top_z_ground <= self.lidar_z_ground:
            return 0.0, '無 (引擎頂 %.2f <= 光達掃描面 %.2f)' % (self.engine_top_z_ground, self.lidar_z_ground)
        ex0 = self.engine_center_x - self.engine_length / 2.0
        ex1 = self.engine_center_x + self.engine_length / 2.0
        if self.lidar_x < ex0:
            d, where = ex0 - self.lidar_x, '車頭方向 (0° 為中心)'
        elif self.lidar_x > ex1:
            d, where = self.lidar_x - ex1, '車尾方向 (180° 為中心)'
        else:
            return 360.0, '光達在引擎方塊裡面'
        return math.degrees(2.0 * math.atan2(self.engine_width / 2.0, d)), where

    def _consistency(self):
        """跨參數的幾何一致性 (階段 38)：兩個獨立估計值之間應該成立、但沒有任何測試會檢查的關係。

        這類錯誤不會讓任何測試變紅，只會讓模擬悄悄變差 (例如 SLAM 因光達被車體自己遮住而變差)，
        所以在啟動時就擋下。物理上不可能的組態 -> GeometryError；允許但必須被看見的 -> self.warnings。
        """
        self.warnings = []
        bad = []
        lz, b0, b1 = self.lidar_z_ground, self.body_ground_clearance, self.body_height
        # 1. 光達掃描面不得落在車體方塊的高度範圍內
        if b0 <= lz <= b1:
            bad.append('光達掃描平面 %.3f 落在車體方塊 %.3f~%.3f 內，模擬中車體會遮蔽自身光達。'
                       '這是物理上不可能的組態，代表 lidar_z_ground 或 body_height 至少有一個是錯的。'
                       % (lz, b0, b1))
        # 2. 各方塊自己的上下界
        if not b0 < b1:
            bad.append('body_ground_clearance %.3f >= body_height %.3f' % (b0, b1))
        if not 0.0 < self.engine_bottom_z_ground < self.engine_top_z_ground:
            bad.append('引擎方塊高度不合理：底 %.3f、頂 %.3f'
                       % (self.engine_bottom_z_ground, self.engine_top_z_ground))
        # 3. 引擎：光達在引擎體積裡 = 不可能；引擎比光達高 = 允許但要警告並報遮擋角
        ang, where = self.engine_occlusion()
        if ang >= 360.0 and self.engine_bottom_z_ground <= lz:
            bad.append('光達 (x %.3f, 離地 %.3f) 落在引擎方塊體積內 (x %.3f~%.3f、離地 %.3f~%.3f)，'
                       '物理上不可能：lidar_x / lidar_z_ground / engine_* 至少有一個是錯的。'
                       % (self.lidar_x, lz, self.engine_center_x - self.engine_length / 2.0,
                          self.engine_center_x + self.engine_length / 2.0,
                          self.engine_bottom_z_ground, self.engine_top_z_ground))
        elif ang > 0.0:
            self.warnings.append('引擎頂 %.3f 高於光達掃描面 %.3f：預期遮擋 %.1f°，%s。'
                                 '允許 (實車引擎可能真的比光達高)，但必須是刻意的。'
                                 % (self.engine_top_z_ground, lz, ang, where))
        # 4. 光達、引擎、刀盤都要在 footprint 裡
        fx0, fx1, hw = -self.rear_extent, self.front_extent, self.lateral_half_extent
        if not fx0 <= self.lidar_x <= fx1:
            bad.append('lidar_x %.3f 在 footprint 前後範圍 %.3f~%.3f 之外' % (self.lidar_x, fx0, fx1))
        ex0 = self.engine_center_x - self.engine_length / 2.0
        ex1 = self.engine_center_x + self.engine_length / 2.0
        if ex0 < fx0 or ex1 > fx1 or self.engine_width / 2.0 > hw:
            bad.append('引擎方塊 (x %.3f~%.3f、半寬 %.3f) 超出 footprint (x %.3f~%.3f、半寬 %.3f)'
                       % (ex0, ex1, self.engine_width / 2.0, fx0, fx1, hw))
        r = self.blade_width / 2.0
        if self.blade_offset_x - r < fx0 or self.blade_offset_x + r > fx1 or r > hw:
            bad.append('刀盤 (中心 x %.3f、半徑 %.3f) 超出 footprint (x %.3f~%.3f、半寬 %.3f)'
                       % (self.blade_offset_x, r, fx0, fx1, hw))
        # 5. 質心要落在支撐範圍 (後輪軸 ~ 腳輪軸) 內，高度在地面與車體頂之間
        if not 0.0 < self.com_x < self.wheelbase:
            bad.append('com_x %.3f 不在後輪軸 (0) 與腳輪軸 (%.3f) 之間，車會翻'
                       % (self.com_x, self.wheelbase))
        if not 0.0 < self.com_height < b1:
            bad.append('com_height %.3f 不在地面與車體頂 %.3f 之間' % (self.com_height, b1))
        # 6. 腳輪要在車寬裡；轉 180° 時輪子會往前多伸 2 x caster_trail
        if self.caster_track / 2.0 + self.front_wheel_width / 2.0 > hw + 1e-9:
            bad.append('腳輪外緣 %.3f 超出車寬一半 %.3f'
                       % (self.caster_track / 2.0 + self.front_wheel_width / 2.0, hw))
        over = self.wheelbase + 2.0 * self.caster_trail + self.front_wheel_radius - self.front_extent
        if over > 1e-9:
            self.warnings.append('腳輪轉向 180° 時輪子前緣超出 footprint %.3f m (= 2 x caster_trail)；'
                                 'footprint 用的是腳輪朝後時的實測長度' % over)
        if bad:
            raise GeometryError('vehicle.yaml 幾何不自洽：\n  ' + '\n  '.join(bad))

    def _unc(self, k):
        if self.provenance.get(k) != 'measured_coarse':
            return 0.0
        return float(self.uncertainty[k])

    def propagate_uncertainty(self):
        """把 measured_coarse 的誤差傳到衍生量，回傳 {名稱: (下界, 標稱, 上界)}。

        各衍生量對每個輸入都是單調的，所以取區間端點的組合就是精確的上下界：
          front_extent = L - r_rear              (r_rear 大 -> 小)
          lateral_half_extent = W / 2
          rotation_swept_radius = hypot(front, lateral)
        【硬性斷言】headland_width 的上下界 (各自套 ceil 規則) 必須相同，
        否則量測精度不足以決定 headland —— 拒絕啟動並要求重新量測。
        本輪不為量測誤差加任何保守餘裕 (階段 38 第 6 節)。
        """
        L = self.body_length_total
        r, W = self.rear_wheel_radius, self.body_width_total
        dr, dw = self._unc('rear_wheel_radius'), self._unc('body_width_total')
        f_lo, _rr, h_lo = _extents(L, r + dr, W - dw)
        f_hi, _rr, h_hi = _extents(L, r - dr, W + dw)
        rs = (math.hypot(f_lo, h_lo), self.rotation_swept_radius, math.hypot(f_hi, h_hi))
        hl = (_headland(rs[0]), self.headland_width, _headland(rs[2]))
        dx, db = self._unc('lidar_x'), self._unc('blade_offset_x')
        out = {
            'front_extent': (f_lo, self.front_extent, f_hi),
            'rear_extent': (r - dr, self.rear_extent, r + dr),
            'lateral_half_extent': (h_lo, self.lateral_half_extent, h_hi),
            'rotation_swept_radius': rs,
            'headland_width': hl,
            'lidar_x': (self.lidar_x - dx, self.lidar_x, self.lidar_x + dx),
            'blade_offset_x': (self.blade_offset_x - db, self.blade_offset_x,
                               self.blade_offset_x + db),
        }
        if abs(hl[0] - hl[2]) > 1e-9:
            raise GeometryError(
                '量測誤差範圍內 headland_width 不唯一：rotation_swept_radius 下界 %.4f -> headland %.2f，'
                '上界 %.4f -> headland %.2f。量測精度不足以決定 headland，請重新量測 '
                'rear_wheel_radius / body_width_total (見 %s)。'
                % (rs[0], hl[0], rs[2], hl[2], WORKLIST_DOC))
        return out

    def headland_stable(self):
        return abs(self.ranges['headland_width'][0] - self.ranges['headland_width'][2]) < 1e-9

    def uncertainty_report(self):
        rows = ['誤差傳遞 (measured_coarse 的 ± 傳到衍生量；front_wheel_radius 在 footprint 推導中抵消)：']
        for k in ('front_extent', 'rear_extent', 'lateral_half_extent', 'rotation_swept_radius',
                  'headland_width', 'lidar_x', 'blade_offset_x'):
            a, b, c = self.ranges[k]
            rows.append('  %-22s %.4f  [%.4f ~ %.4f]' % (k, b, a, c))
        rows.append('  headland_width 在誤差範圍內%s' % ('穩定' if self.headland_stable() else '不穩定'))
        a, b, c = self.ranges['lidar_x']
        d = (c - a) / 2.0
        rows.append('光達前後位置 lidar_x %.2f ~ %.2f：base_link -> radar 的 TF x 在這個範圍內變動 (±%.2f m)。'
                    % (a, c, d))
        rows.append('  對 local_costmap：TF 錯 Δx 時每一個掃描點都沿車身 x 平移 Δx，'
                    '障礙物位置誤差最大 %.2f m (= %.1f 格，解析度 0.05)；' % (d, d / 0.05))
        rows.append('  原地旋轉時同一個障礙物會在半徑 %.2f m 的圓上「甩」，scan matching 會看到牆在動。' % d)
        return '\n'.join(rows)

    # ------------------------------------------------------------------
    def _items(self, kind):
        items = [(k, self.values[k], self.impact[k]) for k, p in self.provenance.items() if p == kind]
        return sorted(items, key=lambda t: (_IMPACT_ORDER[t[2]], t[0]))

    def provisional_items(self):
        """[(鍵, 值, impact)]，依 impact (high -> low) 再依鍵名排序"""
        return self._items('provisional')

    def coarse_items(self):
        return self._items('measured_coarse')

    def stamp(self):
        """報告 / CSV 開頭的蓋章 (多行)。依 provenance 動態產生：
        沒有 provisional 就沒有 PROVISIONAL 行、沒有 measured_coarse 就沒有 COARSE 行，
        兩者都沒有時連 DERIVED-RANGE 一起消失，回傳空字串。"""
        lines = []
        p, c = self.provisional_items(), self.coarse_items()
        if p:
            lines.append('PROVISIONAL: ' + ' '.join('%s=%s(%s)' % (k, _fmt(v), i) for k, v, i in p))
        if c:
            lines.append('COARSE: ' + ' '.join(
                '%s=%s±%s(%s)' % (k, _fmt(v), _fmt(self.uncertainty[k]), i) for k, v, i in c))
        if p or c:
            r = self.ranges
            lines.append('DERIVED-RANGE: lateral_half_extent=%.3f[%.4f~%.4f] '
                         'rotation_swept_radius=%.4f[%.4f~%.4f] headland=%.2f[%s]'
                         % (r['lateral_half_extent'][1], r['lateral_half_extent'][0],
                            r['lateral_half_extent'][2], r['rotation_swept_radius'][1],
                            r['rotation_swept_radius'][0], r['rotation_swept_radius'][2],
                            r['headland_width'][1],
                            '穩定' if self.headland_stable() else '%.2f~%.2f' % (
                                r['headland_width'][0], r['headland_width'][2])))
        return '\n'.join(lines)

    def file_suffix(self):
        """含 provisional 值的報告檔名後綴。沒有 provisional 時回傳空字串。"""
        return '_PROVISIONAL' if self.provisional_items() else ''

    def banner(self):
        p, c = self.provisional_items(), self.coarse_items()
        if not p and not c:
            return ''
        bar = '!' * 78
        lines = [bar]
        if p:
            lines.append('!!  車輛幾何含 %d 個暫定值 (provisional)，沒有量過' % len(p))
            for k, v, imp in p:
                lines.append('!!    %-24s = %-8s impact=%s' % (k, _fmt(v), imp))
        if c:
            lines.append('!!  %d 個粗略實測值 (measured_coarse)，附誤差' % len(c))
            for k, v, imp in c:
                lines.append('!!    %-24s = %-8s ±%-6s impact=%s'
                             % (k, _fmt(v), _fmt(self.uncertainty[k]), imp))
        for w in self.warnings:
            lines.append('!!  一致性警告：%s' % w)
        lines.append('!!  量測方式見 %s' % WORKLIST_DOC)
        lines.append(bar)
        return '\n'.join(lines)

    def xacro_mappings(self):
        """car.xacro 需要的衍生量 (xacro 參數)。xacro 裡不重算公式。"""
        return {k: repr(float(getattr(self, k))) for k in
                ('wheelbase', 'front_extent', 'rear_extent', 'lateral_half_extent',
                 'lidar_z', 'engine_center_x')}

    def summary(self):
        return ('wheelbase %.4f, front_extent %.4f, rear_extent %.4f, '
                'lateral_half_extent %.4f, costmap_inscribed_radius %.4f, '
                'rotation_swept_radius %.7f, headland_width %.2f, soft_inflation_radius %.2f, '
                'lidar x %.3f z(base_link) %.3f'
                % (self.wheelbase, self.front_extent, self.rear_extent,
                   self.lateral_half_extent, self.costmap_inscribed_radius,
                   self.rotation_swept_radius, self.headland_width,
                   self.soft_inflation_radius, self.lidar_x, self.lidar_z))


def _fmt(v):
    """0.50 / 0.375 / 0.08 / 73.4 這種寫法：至少兩位小數，多的有效位數保留"""
    if not isinstance(v, (int, float)):
        return str(v)
    if abs(v) >= 10:
        return '%g' % v
    s = ('%.4f' % v).rstrip('0')
    whole, frac = s.split('.')
    return '%s.%s' % (whole, frac.ljust(2, '0'))


def load(path=None):
    path = path or default_path()
    with open(path) as fh:
        raw = yaml.safe_load(fh)
    return VehicleGeometry(raw, path)


def check_gate(geom, allow_provisional, override_reason=''):
    """allow_provisional 閘門 (階段 38 第 4 節)。

    只有兩個地方呼叫：geometry_guard 節點 (實機 / 模擬 launch 的最前面)，
    以及 bridge_node 以 read_only=false (會送馬達指令) 啟動時。
      provisional     allow_provisional=false 時擋下 (除非給非空的 provisional_override_reason)
      measured_coarse 一律放行，但印警告與誤差範圍

    回傳要寫進 log 的訊息 list [(level, text)]；不通過就丟 GeometryError。
    """
    p, c = geom.provisional_items(), geom.coarse_items()
    msgs = [('info', geom.uncertainty_report())] + [('warn', '幾何一致性警告：' + w) for w in geom.warnings]
    if not p and not c:
        return msgs + [('info', '車輛幾何全部是 measured，閘門通過')]
    if allow_provisional or not p:
        return msgs + [('warn', geom.banner())]
    reason = (override_reason or '').strip()
    listing = '\n'.join('  - %s = %s (impact %s)，量測方式：%s 「%s」一節'
                        % (k, _fmt(v), imp, WORKLIST_DOC, k) for k, v, imp in p)
    if reason:
        return msgs + [('warn', geom.banner()),
                       ('warn', 'allow_provisional=false 但以 provisional_override_reason 強制通行。'
                                '理由：「%s」' % reason)]
    raise GeometryError(
        'allow_provisional=false，但車輛幾何仍有 %d 個暫定值 (provisional)，拒絕啟動：\n%s\n'
        '(measured_coarse 項目放行，只警告。)\n'
        '要強制通行必須同時給非空的 provisional_override_reason (會寫進 log)。'
        % (len(p), listing))
