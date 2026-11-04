"""割草要多久 (2026-10-21 will：想知道一張圖跑完要多久，判斷時間上要不要加快)。純計算，不依賴 ROS。

地圖本來就是公尺 (光達量的，一格 0.05 m)，路線的長度、要轉幾度都是真的。分兩步：

1. plan_geometry：mower_manager 規劃好路線 (預覽或真的割) 時，照它真的會怎麼走，算出每一段的幾何：
   從上一段的終點 (第一段是車子現在的位置) 開過去的轉場 (approach)、轉場前原地轉幾度、
   這一段開始前原地對正轉幾度、這一段多長、走的時候邊走邊轉了幾度。發在 /mission_plan。
2. seconds：照「現在的速度上限」換算成秒。網頁做這一步，所以割草中改速度，剩下的時間跟著變。

時間模型 (沙盒 loopback 底盤實跑擬合，數字見下面的 CALIBRATION)：
  原地對正：角度在 align_goal_checker 的容許 (0.10 rad) 以內不轉；
            否則 align_base + 角度 / (align_k x 角速度上限)
  走一段：  drive_base + 長度 / (DRIVE_K x 速度上限) + 邊走邊轉的角度 / (TURN_K x 角速度上限)
  速度上限 = 0.7 x 速度比例、角速度上限 = 1.2 x 速度比例 (nav2_params 的 FollowPath 乘上 speed_scale)。
  原地轉身比較慢的時候比較接近上限 (加速度一樣)，所以 align_base、align_k、drive_base 照速度比例
  在兩個量過的速度之間內插，超出範圍用最近的那一組。
模擬裡的車子加速、轉向都是理想的，真車通常會慢一些：實車跑過之後拿 car-log 的實際時間再調這幾個常數。
"""
import math

NAV_V_FULL = 0.7            # FollowPath max_vel_x (speed_scale = 1.0 時)
NAV_W_FULL = 1.2            # FollowPath max_vel_theta
ALIGN_TOL = 0.10            # align_goal_checker yaw_goal_tolerance：比這個小不轉
APPROACH_SKIP = 0.5         # manager APPROACH_SKIP_DISTANCE：離下一段起點比這個近就不插轉場
HEADING_LOOKAHEAD = 0.2     # manager ALIGN_HEADING_LOOKAHEAD：取路徑方向時跳過起點附近這麼遠
CORNER_MIN = math.radians(5.0)      # 邊走邊轉：比這個小的方向變化當作直線上的雜訊

# 2026-10-21 擬合：速度比例 -> (align_base 秒, align_k, drive_base 秒)
# 0.50：lab1 (map_1019_181616) 紅線當牆整趟；拿 map_1020_221724 整趟驗證：估 29 分、實際 27.9 分。
# 0.25：lab1 前 21 次對正、19 段 (整趟沒跑完)。
CALIBRATION = ((0.25, (1.34, 0.96, 1.44)),
               (0.50, (1.60, 0.78, 2.10)))
DRIVE_K = 1.0
TURN_K = 1.4

GEOM_KEYS = ('approach_align', 'approach_m', 'approach_turn', 'align', 'length', 'turn')


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))


def heading(pts, from_end=False, lookahead=HEADING_LOOKAHEAD):
    """路徑開頭 (或結尾) 的方向：跳過起點附近 lookahead 以內的點，跟 manager.send_align 一樣。"""
    seq = list(reversed(pts)) if from_end else list(pts)
    p0 = seq[0]
    far = next((q for q in seq[1:] if math.hypot(q[0] - p0[0], q[1] - p0[1]) >= lookahead),
               seq[-1])
    if from_end:
        return math.atan2(p0[1] - far[1], p0[0] - far[0])
    return math.atan2(far[1] - p0[1], far[0] - p0[0])


def turning(pts, step=0.3):
    """沿著折線邊走邊轉的總角度 (rad)：每 step 公尺取一次方向，比 CORNER_MIN 小的變化不算。"""
    if len(pts) < 3:
        return 0.0
    keep = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - keep[-1][0], p[1] - keep[-1][1]) >= step:
            keep.append(p)
    if keep[-1] is not pts[-1] and len(keep) > 1:
        keep[-1] = pts[-1]
    total = 0.0
    for i in range(len(keep) - 2):
        a = math.atan2(keep[i + 1][1] - keep[i][1], keep[i + 1][0] - keep[i][0])
        b = math.atan2(keep[i + 2][1] - keep[i + 1][1], keep[i + 2][0] - keep[i + 1][0])
        d = abs(_wrap(b - a))
        if d >= CORNER_MIN:
            total += d
    return total


def plan_geometry(entries, start, transit_plan=None):
    """每一段的幾何 [approach_align, approach_m, approach_turn, align, length, turn] (rad, m)。

    entries：照順序 [(折線 [(x, y), ...], cont)]；cont = 沿邊「接著上一段」(manager 不原地對正，
    角度算在邊走邊轉)。start：(x, y, yaw) 車子現在的位姿。
    transit_plan(a, b)：轉場路線 [a, ..., b] (manager 的 TransitPlanner.plan)；None = 直線。
    """
    x, y, yaw = start
    out = []
    for pts, cont in entries:
        if len(pts) < 2:
            out.append([0.0] * len(GEOM_KEYS))
            continue
        p0 = pts[0]
        a_align = a_len = a_turn = 0.0
        if math.hypot(p0[0] - x, p0[1] - y) >= APPROACH_SKIP:
            leg = transit_plan((x, y), p0) if transit_plan is not None else None
            if not leg or len(leg) < 2:
                leg = [(x, y), p0]
            leg = [(float(p[0]), float(p[1])) for p in leg]
            a_align = abs(_wrap(heading(leg, lookahead=0.0) - yaw))
            a_len = polyline_length(leg)
            a_turn = sum(abs(_wrap(heading(leg[i + 1:i + 3], lookahead=0.0)
                                   - heading(leg[i:i + 2], lookahead=0.0)))
                         for i in range(len(leg) - 2))
            yaw = heading(leg, from_end=True, lookahead=0.0)
            cont = False            # 開過去的，到了照常對正
        h0 = heading(pts)
        corner = abs(_wrap(h0 - yaw))
        align = 0.0 if cont else corner
        turn = turning(pts) + (corner if cont else 0.0)
        out.append([a_align, a_len, a_turn, align, polyline_length(pts), turn])
        x, y = pts[-1]
        yaw = heading(pts, from_end=True)
    return out


def calibration(speed):
    """(align_base, align_k, drive_base)：在 CALIBRATION 量過的速度之間線性內插。"""
    (s0, c0), (s1, c1) = CALIBRATION[0], CALIBRATION[-1]
    f = min(1.0, max(0.0, (speed - s0) / (s1 - s0)))
    return tuple(a + (b - a) * f for a, b in zip(c0, c1))


def align_seconds(angle, speed):
    if angle < ALIGN_TOL:
        return 0.0
    base, k, _ = calibration(speed)
    return base + angle / (k * NAV_W_FULL * speed)


def drive_seconds(length, turn, speed):
    if length <= 0.0:
        return 0.0
    return (calibration(speed)[2] + length / (DRIVE_K * NAV_V_FULL * speed)
            + turn / (TURN_K * NAV_W_FULL * speed))


def seconds(seg, speed):
    """一段 (含它前面的轉場) 要幾秒。speed = 速度比例 (car-start / 網頁的速度上限，0.10 ~ 1.0)。"""
    a_align, a_len, a_turn, align, length, turn = seg[:6]
    t = 0.0
    if a_len > 0.0:
        t += align_seconds(a_align, speed) + drive_seconds(a_len, a_turn, speed)
    return t + align_seconds(align, speed) + drive_seconds(length, turn, speed)


def total_seconds(segs, speed):
    return sum(seconds(s, speed) for s in segs)


def remaining_seconds(segs, speed, current, elapsed):
    """還要幾秒：current 這一段已經走了 elapsed 秒 (算到 0 為止)，後面的照估。current = None 表示還沒開始。"""
    if current is None:
        return total_seconds(segs, speed)
    if current >= len(segs):
        return 0.0
    now = max(0.0, seconds(segs[current], speed) - max(0.0, elapsed))
    return now + sum(seconds(s, speed) for s in segs[current + 1:])


def route_meters(segs):
    """實際要開的總距離 (轉場 + 每一段)。"""
    return sum(s[1] + s[4] for s in segs)
