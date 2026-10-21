"""route_plan 的單元測試 (純計算，不需要 ROS)。

場景都用跟 F2C 一樣的擺法手做割草線：線距 0.3 m、沿 x 軸，障礙物把某幾列切成兩段。
"""
import math

import numpy as np

from mowerbot_action import route_plan

SP = 0.3
HL = 1.2          # 地頭：割草線離邊界 / 障礙物這麼遠 (跟 f2c_server 一樣)
BOX = (4.0, 1.4, 6.0, 1.9)                         # 障礙物 (x0, y0, x1, y1)
N_ROWS = 16                                        # 第 0 列 y = 0，第 15 列 y = 4.5


def field_with_hole():
    """割草範圍：外圈離割草線兩端與最外側的線 HL，中間一個障礙物 BOX。"""
    top = (N_ROWS - 1) * SP + HL
    outer = [(-HL, -HL), (10.0 + HL, -HL), (10.0 + HL, top), (-HL, top)]
    x0, y0, x1, y1 = BOX
    return [outer, [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]]


def rows_with_hole(box=BOX):
    """F2C 會擺出來的割草線：離障礙物 HL 以內的那幾列被切成左右兩段。照 F2C 清單的順序與方向。"""
    ends = []
    rev = False
    for r in range(N_ROWS):
        y = r * SP
        pieces = [(0.0, 10.0)]
        if box is not None and box[1] - HL < y < box[3] + HL:
            pieces = [(0.0, box[0] - HL), (box[2] + HL, 10.0)]
        for a, b in pieces:
            ends.append(((b, y), (a, y)) if rev else ((a, y), (b, y)))
            rev = not rev
    return ends


def walk(ends, order):
    """照 order 走一遍，回傳 [(起點, 終點)]。"""
    out = []
    for i, rev in order:
        a, b = ends[i]
        out.append((b, a) if rev else (a, b))
    return out


def test_good_order_is_untouched():
    ends = rows_with_hole(box=None)                    # 沒有洞：F2C 的順序就是弓字形
    order, info = route_plan.plan_order(ends, 0.0, SP, (0.0, 0.0))
    assert info == {'chains': 1, 'changed': False}
    assert order == [(i, False) for i in range(len(ends))]


def test_hole_splits_into_blocks_and_every_swath_once():
    ends = rows_with_hole()
    tp = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11)
    order, info = route_plan.plan_order(ends, 0.0, SP, (0.0, 0.0), dist=tp.length, clear=tp.clear)
    assert info['chains'] >= 2
    assert info['changed']
    assert sorted(i for i, _r in order) == list(range(len(ends)))
    # 分成 4 塊：障礙物下面、左邊、右邊、上面
    assert info['chains'] == 4
    # 不在割草的直線距離 (掉頭 + 換塊) 很少：一塊接一塊都在同一頭接上，沒有沿著割好的線白跑一大段
    path = walk(ends, order)
    dead = sum(math.hypot(b0[0] - a1[0], b0[1] - a1[1])
               for (_a0, a1), (b0, _b1) in zip(path[:-1], path[1:]))
    turns = len(path) - 1
    assert dead < turns * SP + 6.0, dead
    # 同一塊裡一來一回：方向相反
    for (a0, a1), (b0, b1) in zip(path[:-1], path[1:]):
        if math.hypot(b0[0] - a1[0], b0[1] - a1[1]) < SP + 1e-6:
            assert (a1[0] - a0[0]) * (b1[0] - b0[0]) < 0


def test_old_order_crossed_the_hole():
    """對照組：F2C 原本的順序在被切開的那幾列，相鄰兩段在洞的兩邊 (這就是 10-20 的問題)。"""
    ends = rows_with_hole()
    crossings = 0
    for (_a0, a1), (b0, _b1) in zip(ends[:-1], ends[1:]):
        if seg_hits_box(a1, b0, BOX, 0.0):
            crossings += 1
    assert crossings > 0
    tp = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11)
    order, _info = route_plan.plan_order(ends, 0.0, SP, (0.0, 0.0), dist=tp.length, clear=tp.clear)
    path = walk(ends, order)
    for (_a0, a1), (b0, _b1) in zip(path[:-1], path[1:]):
        if math.hypot(b0[0] - a1[0], b0[1] - a1[1]) >= 0.5:
            legs = tp.plan(a1, b0)
            assert legs is not None
            for p, q in zip(legs[:-1], legs[1:]):
                assert not seg_hits_box(p, q, BOX, 0.45)


def test_rotated_field_same_blocks():
    """割草線不沿 x 軸 (manager 轉回來之後的座標) 也一樣分塊。"""
    ends = rows_with_hole()
    ang = math.radians(37.0)
    c, s = math.cos(ang), math.sin(ang)
    rot = [((c * a[0] - s * a[1], s * a[0] + c * a[1]), (c * b[0] - s * b[1], s * b[0] + c * b[1]))
           for a, b in ends]
    o1, i1 = route_plan.plan_order(ends, 0.0, SP, (0.0, 0.0))
    o2, i2 = route_plan.plan_order(rot, ang, SP, (0.0, 0.0))
    assert i1['changed'] and i1 == i2

    def dead(e, order, start):     # 不在割草的直線距離總和 (同分的順序可能不同，總長要一樣)
        here, tot = start, 0.0
        for a, b in walk(e, order):
            tot += math.hypot(a[0] - here[0], a[1] - here[1])
            here = b
        return tot
    assert abs(dead(ends, o1, (0.0, 0.0)) - dead(rot, o2, (0.0, 0.0))) < 1e-6


def test_small_gap_is_mowed_straight_through():
    """同一列只被很短的空隙切開 (邊界的小凹凸) 時當成同一列一路割過去，不另外成一塊。"""
    ends = []
    rev = False
    for r in range(N_ROWS):
        y = r * SP
        pieces = [(0.0, 4.8), (5.2, 10.0)] if r == 3 else [(0.0, 10.0)]
        for a, b in pieces:
            ends.append(((b, y), (a, y)) if rev else ((a, y), (b, y)))
            rev = not rev
    outer = field_with_hole()[0]
    tp = route_plan.TransitPlanner([outer], 0.47, 1.11)
    _order, info = route_plan.plan_order(ends, 0.0, SP, (0.0, 0.0), dist=tp.length, clear=tp.clear)
    assert info == {'chains': 1, 'changed': True}


def seg_hits_box(a, b, box, margin):
    x0, y0, x1, y1 = box[0] - margin, box[1] - margin, box[2] + margin, box[3] + margin
    for k in range(201):
        t = k / 200.0
        x, y = a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t
        if x0 < x < x1 and y0 < y < y1:
            return True
    return False


def test_transit_goes_around_obstacle():
    tp = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11)
    a, b = (2.0, 1.65), (8.0, 1.65)                    # 障礙物的左邊到右邊，直線正好穿過去
    assert not tp.clear(a, b)
    path = tp.plan(a, b, relax=0.0)
    assert path is not None and len(path) > 2
    for p, q in zip(path[:-1], path[1:]):
        assert not seg_hits_box(p, q, BOX, 0.45)
    for c in path[1:-1]:                               # 轉折點掉得了頭
        assert tp.clearance(*c) >= 1.11 - 1e-6
    length = tp.length(a, b, relax=0.0)
    assert 6.0 < length < 12.0


def test_transit_straight_when_clear():
    tp = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11)
    a, b = (0.5, 3.6), (10.0, 3.6)                     # 障礙物的上方，直線走得過去
    assert tp.plan(a, b) == [a, b]


def test_transit_no_path():
    """兩塊之間沒有夠寬的通道：回傳 None，manager 照舊走直線。"""
    outer = [(0.0, 0.0), (10.0, 0.0), (10.0, 3.0), (0.0, 3.0)]
    wall = [(4.8, -1.0), (5.2, -1.0), (5.2, 4.0), (4.8, 4.0)]     # 整面牆把場地切開
    tp = route_plan.TransitPlanner([outer, wall], 0.47, 1.11)
    assert tp.plan((2.0, 1.5), (8.0, 1.5)) is None
    assert math.isinf(tp.length((2.0, 1.5), (8.0, 1.5)))


def test_order_loops_starts_nearest():
    sq = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0), (0.0, 0.0)]
    far = [(10.0, 0.0), (12.0, 0.0), (12.0, 2.0), (10.0, 2.0), (10.0, 0.0)]
    out = route_plan.order_loops([far, sq], (3.9, 4.2))
    assert out == [(1, 2), (0, 3)]          # 繞完 sq 回到 (4, 4)，far 離它最近的是 (10, 2)


# ---- 沿邊多繞幾圈 (EdgeLoops) ----
FRONT, REAR, HALF_W = 0.975, 0.155, 0.42
ROT_NEED = math.hypot(FRONT, HALF_W) + 0.05
SAFE = 1.2


def rect(w, h):
    return [(0.0, 0.0), (w, 0.0), (w, h), (0.0, h)]


def route_yaw(pts):
    d = np.gradient(np.asarray(pts), axis=0)
    return np.arctan2(d[:, 1], d[:, 0])


def check_route(el, r):
    pts = r['pts']
    yaw = route_yaw(pts)
    assert el.poses_ok(pts, yaw, margin=0.0).all()               # 車身 (與停止區) 從頭到尾都在範圍裡
    assert el.clearance_at(pts[0]) >= ROT_NEED - 1e-6           # 匝道兩頭掉得了頭
    assert el.clearance_at(pts[-1]) >= ROT_NEED - 1e-6
    steps = np.hypot(*np.diff(pts, axis=0).T)
    assert steps.max() < 0.2                                     # 中間沒有跳接


def test_corner_radius_keeps_nose_inside():
    for d in (0.6, 0.75, 1.1):
        r = route_plan.corner_radius(d, FRONT, HALF_W, margin=0.05)
        assert abs(math.hypot(r + HALF_W, FRONT) - (r + d - 0.05)) < 1e-9 or r == 0.0
    assert route_plan.corner_radius(0.45, FRONT, HALF_W) is None    # 側向半寬都塞不下


def test_edge_loops_rectangle():
    el = route_plan.EdgeLoops([rect(10.0, 6.0)], FRONT, REAR, HALF_W)
    routes = el.routes([1.10, 0.75], SAFE, ROT_NEED)
    assert sorted(r['d'] for r in routes) == [0.75, 1.10]
    for r in routes:
        assert r['full']
        check_route(el, r)
        body = r['pts'][r['ramp_in']:len(r['pts']) - r['ramp_out']]
        clear = el.clearance_at(body)
        # 直的地方離邊界就是 d (格子與平滑的誤差 3 cm 內)，轉角往裡面縮
        assert abs(float(np.median(clear)) - r['d']) < 0.03
        assert clear.min() >= r['d'] - 0.03


def room_walls(w, h, sides='bottom left top right', gap=0.10, res=0.05):
    """場地 (0..w, 0..h) 外面 gap 的地方立著牆 (只有 sides 列出的那幾邊)：(grid, res, ox, oy)。"""
    grid = np.zeros((int(h / res) + 40, int(w / res) + 40), bool)
    ox, oy = -1.0, -1.0
    lo, hx, hy = -gap - 0.1, w + gap + 0.1, h + gap + 0.1
    boxes = {'bottom': (lo, lo, hx, -gap), 'top': (lo, h + gap, hx, hy),
             'left': (lo, lo, -gap, hy), 'right': (w + gap, lo, hx, hy)}
    for side in sides.split():
        x0, y0, x1, y1 = boxes[side]
        r0, r1 = int((y0 - oy) / res), int((y1 - oy) / res) + 1
        c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res) + 1
        grid[r0:r1, c0:c1] = True
    return grid, res, ox, oy


def test_edge_loops_walls_keep_stop_zone_clear():
    """場地四周是真的牆：停止區在車頭偏 ±20° 的範圍都要離牆 0.05 m 以上 (10-21 模擬實跑車頭落後 15°
    就掃到牆、卡死)。0.75 m 那一圈沿著牆根本走不了 (偏 20° 停止區伸到 0.96 m)，不規劃；
    1.10 m 那一圈牆角的圓弧加大 (比沒有牆時短)。"""
    w, h = 12.0, 8.0
    el = route_plan.EdgeLoops([rect(w, h)], FRONT, REAR, HALF_W, walls=room_walls(w, h))
    routes = el.routes([1.10, 0.75], SAFE, ROT_NEED)
    assert [r['d'] for r in routes] == [1.10]
    for r in routes:
        assert r['full']
        check_route(el, r)
    plain = route_plan.EdgeLoops([rect(w, h)], FRONT, REAR, HALF_W).routes(
        [1.10], SAFE, ROT_NEED)[0]
    assert routes[0]['length'] < plain['length']


def test_edge_loops_wall_on_one_side_cuts_the_inner_loop():
    """只有下緣是牆：0.75 m 那一圈在牆邊斷開，剩下那一截 (另外三邊) 照割，離牆的那幾截不割。"""
    w, h = 12.0, 8.0
    el = route_plan.EdgeLoops([rect(w, h)], FRONT, REAR, HALF_W, walls=room_walls(w, h, 'bottom'))
    inner = [r for r in el.routes([0.75], SAFE, ROT_NEED)]
    assert len(inner) == 1 and not inner[0]['full']
    r = inner[0]
    check_route(el, r)
    body = r['pts'][r['ramp_in']:len(r['pts']) - r['ramp_out']]
    assert body[:, 1].min() > 0.75 + 0.1                       # 沒有沿著下緣 (牆) 走
    assert r['length'] > 2 * h                                 # 另外三邊大部分都有


def test_tolerant_box_and_swept_zone():
    zf, zh = route_plan.tolerant_box(1.375, 0.47, math.radians(20.0))
    c, s = math.cos(math.radians(20.0)), math.sin(math.radians(20.0))
    assert abs(zh - (1.375 * s + 0.47 * c)) < 1e-9 and abs(zf - (1.375 * c - 0.47 * s)) < 1e-9
    pts = route_plan._swept_samples(0.975, 1.375, 0.47, math.radians(20.0))
    # 轉 20° 的外側前角掃得到、沒轉的長方形裡面也都在
    far = np.array([1.375 * c + 0.47 * s, 1.375 * s - 0.47 * c])
    assert np.hypot(*(pts - far).T).min() < 0.04
    assert np.hypot(*(pts - np.array([1.2, 0.0])).T).min() < 0.04


def test_edge_loops_hole_and_narrow():
    """中間有障礙物的場地每一圈都合法；太窄的地方 (寬 2.4 m) 沒有 0.75 m 那一圈。"""
    el = route_plan.EdgeLoops(field_with_hole(), FRONT, REAR, HALF_W)
    for r in el.routes([1.10, 0.75], SAFE, ROT_NEED):
        check_route(el, r)
    narrow = route_plan.EdgeLoops([rect(12.0, 2.4)], FRONT, REAR, HALF_W)
    assert narrow.routes([0.75], SAFE, ROT_NEED) == []


def test_escape_back_to_turning_room():
    el = route_plan.EdgeLoops([rect(10.0, 6.0)], FRONT, REAR, HALF_W)
    got = el.escape((5.0, 0.75), 0.0, ROT_NEED)             # 貼著下緣、往 +x 走
    assert got is not None
    line, _hd = got
    assert el.clearance_at(line[-1]) >= ROT_NEED - 1e-6
    hd = math.atan2(line[-1][1] - line[0][1], line[-1][0] - line[0][0])
    assert 0.0 < hd <= math.radians(30.0) + 1e-9            # 往裡面 (左邊) 斜，不原地轉


def test_segment_route_chains():
    el = route_plan.EdgeLoops([rect(10.0, 6.0)], FRONT, REAR, HALF_W)
    r = el.routes([0.75], SAFE, ROT_NEED)[0]
    segs = route_plan.segment_route(r['pts'])
    assert segs[0][0] == 0 and segs[-1][1] == len(r['pts']) - 1
    for (a0, a1), (b0, _b1) in zip(segs[:-1], segs[1:]):
        assert a1 == b0                                       # 下一段從上一段的終點開始
    for a0, a1 in segs:
        assert np.hypot(*(r['pts'][a1] - r['pts'][a0])) >= 1.0


def hairpin(r=1.0, leg=4.0, step=0.1):
    """直走 leg、半徑 r 左轉 180°、再直走 leg 回來 (U 形急彎)。"""
    a = np.column_stack([np.arange(0.0, leg, step), np.zeros(int(round(leg / step)))])
    t = np.linspace(-math.pi / 2, math.pi / 2, int(round(math.pi * r / step)) + 1)
    arc = np.column_stack([leg + r * np.cos(t), r + r * np.sin(t)])
    b = np.column_stack([np.arange(leg - step, -1e-9, -step),
                         np.full(int(round(leg / step)), 2 * r)])
    return np.vstack([a, arc, b])


def test_segment_route_splits_u_turn_on_straight_parts():
    """U 形急彎不能整個在同一段裡 (10-21 模擬：DWB 卡在彎前面)：每段最多轉 120°，切點在直的地方。"""
    pts = hairpin()
    segs = route_plan.segment_route(pts)
    assert len(segs) >= 2
    assert segs[0][0] == 0 and segs[-1][1] == len(pts) - 1
    yaw = np.unwrap(route_yaw(pts))
    for a0, a1 in segs:
        assert np.ptp(yaw[a0:a1 + 1]) <= math.radians(120.0) + 1e-6
        assert np.hypot(*(pts[a1] - pts[a0])) >= 1.0
    assert pts[segs[0][1]][0] <= 4.0 + 1e-6                    # 第一刀切在彎前面直的地方


def test_rotation_ok_near_edge():
    """離下緣 0.75 m 往 +x：往裡面 (左) 轉 30° 掃不到邊，轉 90° 對著外面就會。"""
    el = route_plan.EdgeLoops([rect(10.0, 6.0)], FRONT, REAR, HALF_W)
    assert el.rotation_ok((5.0, 0.75), 0.0, math.radians(30.0))
    assert not el.rotation_ok((5.0, 0.75), 0.0, math.radians(-90.0))
    assert el.rotation_ok((5.0, 3.0), 0.0, math.pi - 0.01)     # 場地中間轉一圈都沒問題


# ---- 原地轉身的地方離牆夠遠 (第 4 節) ----
ZONE_NEED = (route_plan.stop_zone_turn_radius(FRONT, HALF_W, 0.40, 0.05)
             + route_plan.TURN_WALL_MARGIN)


def test_stop_zone_turn_radius():
    r = route_plan.stop_zone_turn_radius(0.975, 0.42, 0.40, 0.05)
    assert abs(r - math.hypot(1.375, 0.47)) < 1e-12
    assert 1.59 < ZONE_NEED < 1.61


def test_wall_clearance_ignores_specks_and_outside():
    res, ox, oy = 0.05, -1.0, -1.0
    grid = np.zeros((200, 200), bool)
    grid[100:102, 20:180] = True                 # 一道牆 y = 4.0 ~ 4.1 (x 0 ~ 8)
    grid[40, 40] = True                          # 一個雜點 (1, 1)：不算牆
    wc = route_plan.WallClearance((grid, res, ox, oy))
    d = wc.at(np.array([[3.0, 2.0], [1.0, 1.0], [3.0, 3.5], [50.0, 50.0]]))
    assert abs(d[0] - 2.0) < 0.06 and abs(d[2] - 0.5) < 0.06
    assert d[1] > 2.5                            # 雜點旁邊不算離牆近
    assert math.isinf(d[3])                      # 地圖外面當作沒有牆
    empty = route_plan.WallClearance((np.zeros((10, 10), bool), res, 0.0, 0.0))
    assert math.isinf(float(empty.at((0.2, 0.2))))


def test_trim_ends():
    pts = np.column_stack([np.arange(0.0, 5.01, 0.1), np.zeros(51)])
    assert route_plan.trim_ends(pts, np.ones(51, bool)) == (0, 50)
    ok = pts[:, 0] <= 4.0                        # 盡頭那 1 m 離牆太近
    assert route_plan.trim_ends(pts, ok) == (0, 40)
    ok = (pts[:, 0] >= 0.5) & (pts[:, 0] <= 4.0)
    ok[25] = False                               # 中間不能轉身沒關係 (直直開過去)
    assert route_plan.trim_ends(pts, ok) == (5, 40)
    assert route_plan.trim_ends(pts, (pts[:, 0] >= 2.4) & (pts[:, 0] <= 2.8)) is None   # 剩 0.4 m
    assert route_plan.trim_ends(pts, np.zeros(51, bool)) is None


def box_walls(box, res=0.05):
    """一個方塊 (x0, y0, x1, y1) 是光達看得到的牆：(grid, res, ox, oy)。"""
    ox, oy = -3.0, -3.0
    grid = np.zeros((int(14 / res), int(18 / res)), bool)
    x0, y0, x1, y1 = box
    r0, r1 = int((y0 - oy) / res), int((y1 - oy) / res)
    c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
    grid[r0:r1, c0:c1] = True
    return grid, res, ox, oy


def test_transit_corners_keep_away_from_walls():
    """障礙物是真的牆：繞過去的轉折點離它 >= ZONE_NEED (光達停止區原地轉一圈掃不到)，還是繞得過去。"""
    wc = route_plan.WallClearance(box_walls(BOX))
    tp = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11, walls=wc, wall_need=ZONE_NEED)
    a, b = (2.0, 1.65), (8.0, 1.65)
    path = tp.plan(a, b, relax=0.0)
    assert path is not None and len(path) > 2
    for p, q in zip(path[:-1], path[1:]):
        assert not seg_hits_box(p, q, BOX, 0.45)
    for c in path[1:-1]:
        assert float(wc.at(c)) >= ZONE_NEED - 1e-6
        assert tp.clearance(*c) >= 1.11 - 1e-6
    plain = route_plan.TransitPlanner(field_with_hole(), 0.47, 1.11)
    # 以前轉折點離牆比較近
    assert min(float(wc.at(c)) for c in plain.plan(a, b, relax=0.0)[1:-1]) < ZONE_NEED


def test_edge_loops_ramp_ends_keep_away_from_walls():
    """四周是牆：匝道兩頭 (原地轉身的地方) 要離牆 >= ZONE_NEED，匝道往裡面多挪、斜一點，還是繞得了。"""
    w, h = 12.0, 8.0
    walls = room_walls(w, h)
    el = route_plan.EdgeLoops([rect(w, h)], FRONT, REAR, HALF_W, walls=walls, turn_wall=ZONE_NEED)
    routes = el.routes([1.10, 0.75], SAFE, ROT_NEED)
    assert [r['d'] for r in routes] == [1.10]
    wc = route_plan.WallClearance(walls)
    for r in routes:
        check_route(el, r)
        assert float(wc.at(r['pts'][0])) >= ZONE_NEED - 0.03
        assert float(wc.at(r['pts'][-1])) >= ZONE_NEED - 0.03
    old = route_plan.EdgeLoops([rect(w, h)], FRONT, REAR, HALF_W, walls=walls).routes(
        [1.10], SAFE, ROT_NEED)[0]
    assert float(wc.at(old['pts'][0])) < ZONE_NEED                    # 以前匝道起點離牆只有 1.30 m
    got = el.escape((6.0, 1.10), 0.0, ROT_NEED)                       # 往回開也要開到離牆夠遠
    assert got is not None and float(wc.at(got[0][-1])) >= ZONE_NEED - 0.03


def test_zone_sweep_clearance():
    """前面 1.3 m 有一道牆 (y = 1.3)：朝牆的停止區已經碰到；背對牆轉 30° 掃不到；轉 180° 一定掃到。"""
    res, ox, oy = 0.05, -5.0, -5.0
    grid = np.zeros((200, 200), bool)
    grid[int((1.3 - oy) / res):int((1.5 - oy) / res), :] = True
    wc = route_plan.WallClearance((grid, res, ox, oy))
    up, down = math.pi / 2, -math.pi / 2
    x0, x1, hw = FRONT, FRONT + 0.40, HALF_W + 0.05
    assert route_plan.zone_sweep_clearance(wc, (0.0, 0.0), up, up, x0, x1, hw) < 0.05
    assert route_plan.zone_sweep_clearance(wc, (0.0, 0.0), down, down + 0.5, x0, x1, hw) > 1.0
    assert route_plan.zone_sweep_clearance(wc, (0.0, 0.0), down, up - 0.01, x0, x1, hw) < 0.05
