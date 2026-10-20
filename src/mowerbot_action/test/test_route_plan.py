"""route_plan 的單元測試 (純計算，不需要 ROS)。

場景都用跟 F2C 一樣的擺法手做割草線：線距 0.3 m、沿 x 軸，障礙物把某幾列切成兩段。
"""
import math

from mowerbot_action import route_plan

SP = 0.3
HL = 1.2          # 地頭：割草線離邊界 / 障礙物這麼遠 (跟 f2c_server 一樣)
BOX = (4.0, 1.4, 6.0, 1.9)                         # 障礙物 (x0, y0, x1, y1)
N_ROWS = 16                                        # 第 0 列 y = 0，第 15 列 y = 4.5


def field_with_hole():
    """割草範圍：外圈離割草線兩端與最外側的線 HL，中間一個障礙物 BOX。"""
    outer = [(-HL, -HL), (10.0 + HL, -HL), (10.0 + HL, (N_ROWS - 1) * SP + HL), (-HL, (N_ROWS - 1) * SP + HL)]
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
    dead = sum(math.hypot(b0[0] - a1[0], b0[1] - a1[1]) for (_a0, a1), (b0, _b1) in zip(path[:-1], path[1:]))
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
    rot = [((c * a[0] - s * a[1], s * a[0] + c * a[1]), (c * b[0] - s * b[1], s * b[0] + c * b[1])) for a, b in ends]
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
