"""停下來之後「接著割」(2026-10-22 will：急停之後要可以接著剛剛的自動割草進度，或重來)。純計算，不依賴 ROS。

【存什麼】自動割草一出發，mower_manager 把這一趟要走的每一段 (周邊環繞、割草線、沿邊) 存成一份
「路線檔」，割草中每送出一段、車子每在這一段上開了 0.2 m，就更新一份小小的「進度檔」
(現在在第幾段、車子開到這一段的哪裡、做完幾段、跳過哪幾段)。兩個檔跟著地圖 (session) 存：
    ~/.ros/mowerbot_progress/<session>.plan.json   路線 (一趟寫一次)
    ~/.ros/mowerbot_progress/<session>.cur.json    進度 (割草中一直更新)
馬達總開關也會把 N150 關掉，那時候什麼都來不及寫，所以進度是「一直寫」而不是「停的時候寫」。
跟禁區、外框一樣：用同一張存好的地圖開機才找得回來；建新地圖是新的 session (座標不一樣)，不會拿舊的。

【接著割從哪裡開始】resume_entries：
  * 做完的、跳過的段不再走 (跳過的照樣列在「跳過」裡)
  * 停在一條割草線 (或周邊環繞) 的中間：從車子停的地方往回 0.3 m 開始 (重疊一點，不留縫)，
    割草線前面重新接一段跑道；停的地方離終點不到 0.3 m 就當作割完了
  * 停在沿邊那一圈的中間：那一圈從頭繞 (沿邊離邊很近，中間原地轉身車頭會掃出邊界)
  * 轉場 (approach)、沿邊退出不存：接著割的時候 manager 從車子當下的位置重新開過去
【不能接著割】停下來之後改過禁區或外框 (路線是照舊的禁區、外框算的，可能會壓到)：要重新開始。
"""
import glob
import hashlib
import json
import math
import os
import time

VERSION = 1
PROGRESS_DIR = os.path.expanduser('~/.ros/mowerbot_progress')
BACK_M = 0.3            # 停在一段中間：從停的地方往回這麼多開始 (重疊，不留縫)
NEAR_M = 0.6            # 車子離這一段比這個遠，就不是停在這一段上 (例如在轉場、對正)：整段重來
MIN_LEFT_M = 0.3        # 停的地方離這一段的終點不到這麼遠就當作做完了
KEEP_SESSIONS = 10      # 只留最近幾張地圖的進度


def kind_of(label):
    """佇列標籤 -> 段落種類：perimeter / swath / edge；其他 (approach、沿邊退出) 是 None (不存)。"""
    if label.startswith('周邊環繞'):
        return 'perimeter'
    if label.startswith('割草線'):
        return 'swath'
    if label.startswith('沿邊') and not label.startswith('沿邊退出'):
        return 'edge'
    return None


def label_index(label):
    """'割草線 12/40' -> (12, 40)；看不懂就 (1, 1)。"""
    try:
        a, b = label.split()[-1].split('/')
        return int(a), int(b)
    except (ValueError, IndexError):
        return 1, 1


def make_entry(label, pts, cont=False, lead=0):
    """一段路線：pts = [[x, y, yaw], ...] (map 座標)，cont = 沿邊接著上一段 (不原地對正)，
    lead = 前面幾個點是跑道 (割草線)。"""
    return {'label': label, 'kind': kind_of(label),
            'pts': [[round(float(x), 3), round(float(y), 3), round(float(a), 4)]
                    for x, y, a in pts],
            'cont': bool(cont), 'lead': int(lead)}


def _cum(pts):
    out = [0.0]
    for i in range(len(pts) - 1):
        out.append(out[-1] + math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]))
    return out


def project(pts, xy):
    """xy 投影到折線上：(沿線走了幾公尺, 離線多遠, 每個點的累積長度)。"""
    cum = _cum(pts)
    best = (0.0, math.hypot(xy[0] - pts[0][0], xy[1] - pts[0][1]))
    for i in range(len(pts) - 1):
        ax, ay = pts[i][0], pts[i][1]
        dx, dy = pts[i + 1][0] - ax, pts[i + 1][1] - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((xy[0] - ax) * dx + (xy[1] - ay) * dy) / L2))
        d = math.hypot(xy[0] - (ax + t * dx), xy[1] - (ay + t * dy))
        if d < best[1] - 1e-9:
            best = (cum[i] + t * math.sqrt(L2), d)
    return best[0], best[1], cum


def length_of(entry):
    return _cum(entry['pts'])[-1] if len(entry['pts']) > 1 else 0.0


def resume_entries(entries, k, xy, back=BACK_M, near=NEAR_M, min_left=MIN_LEFT_M):
    """接著割要走的段落 (新的 list，不改 entries) 與說明。

    k = 第一個還沒做完的段落 (前面的做完了或跳過了)；xy = 車子在第 k 段上開到哪裡 (沒有就 None)。
    回傳 (段落 list, info)：info = {'from': 第一段的標籤, 'cut_m': 第 k 段已經割掉不再走的長度,
    'loop_back': 沿邊那一圈退回去幾段, 'dropped': 第 k 段剩太短直接當作做完}。
    第 k 段被切短時，那一段多一個 'relead': True (manager 重新接跑道)，lead 歸零。
    """
    info = {'from': None, 'cut_m': 0.0, 'loop_back': 0, 'dropped': False}
    if k < 0 or k >= len(entries):
        return [], info
    start = k
    if entries[k]['kind'] == 'edge':
        # 沿邊一圈從頭繞：往前找到這一圈的第一段 (不是接著上一段的那一段)
        while start > 0 and entries[start]['cont'] and entries[start - 1]['kind'] == 'edge':
            start -= 1
        info['loop_back'] = k - start
    rest = [dict(e, pts=[list(p) for p in e['pts']]) for e in entries[start:]]
    first = rest[0]
    if (start == k and xy is not None and first['kind'] in ('swath', 'perimeter')
            and len(first['pts']) >= 2):
        s, d, cum = project(first['pts'], xy)
        body0 = min(first['lead'], len(cum) - 1) if first['kind'] == 'swath' else 0
        if d <= near and s - back > cum[body0] + 1e-6:
            cut = next(i for i, c in enumerate(cum) if c >= s - back - 1e-9)
            cut = max(cut, body0)
            if cum[-1] - s < min_left:
                info['dropped'] = True
                info['cut_m'] = round(cum[-1] - cum[body0], 2)
                rest = rest[1:]
            else:
                info['cut_m'] = round(cum[cut] - cum[body0], 2)
                first['pts'] = first['pts'][cut:]
                if first['kind'] == 'swath':
                    first['lead'] = 0
                    first['relead'] = True
    if rest:
        info['from'] = rest[0]['label']
    return rest, info


# ---- 存檔 -----------------------------------------------------------------
def paths(session, base=PROGRESS_DIR):
    return (os.path.join(base, session + '.plan.json'), os.path.join(base, session + '.cur.json'))


def write_text(path, text):
    """先寫暫存檔再改名：寫到一半斷電，舊的那一份還在。"""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        fh.write(text)
        fh.flush()
        os.fsync(fh.fileno())
    os.replace(tmp, path)


def remove(session, base=PROGRESS_DIR):
    for p in paths(session, base):
        try:
            os.remove(p)
        except OSError:
            pass


def prune(base=PROGRESS_DIR, keep=KEEP_SESSIONS):
    """只留最近 keep 張地圖的進度 (一張圖的路線檔約 100 KB)。"""
    plans = sorted(glob.glob(os.path.join(glob.escape(base), '*.plan.json')),
                   key=os.path.getmtime, reverse=True)
    for p in plans[keep:]:
        for f in (p, p[:-len('.plan.json')] + '.cur.json'):
            try:
                os.remove(f)
            except OSError:
                pass


def _read(path):
    try:
        with open(path, encoding='utf-8') as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def load(session, base=PROGRESS_DIR):
    """讀這張地圖存的進度：(plan, cur) 或 None。路線檔壞了、不是這張地圖的、版本不對都當作沒有。"""
    plan_path, cur_path = paths(session, base)
    plan = _read(plan_path)
    if (not isinstance(plan, dict) or plan.get('version') != VERSION
            or plan.get('session') != session or not isinstance(plan.get('entries'), list)):
        return None
    try:
        for e in plan['entries']:
            if e['kind'] not in ('perimeter', 'swath', 'edge') or len(e['pts']) < 2:
                return None
    except (KeyError, TypeError):
        return None
    cur = _read(cur_path)
    if not isinstance(cur, dict) or cur.get('plan') != plan.get('id'):
        cur = new_cursor(plan)
    return plan, cur


def new_plan(session, entries, rings, fingerprint, carry=None):
    """carry = 接著割之前那幾趟的 {'succeeded': 做完幾段, 'skipped': [跳過的段]}，接著累計
    (網頁上的進度、跳過清單是整片草地的，不是只有這一趟)。"""
    carry = carry or {}
    return {'version': VERSION, 'session': session, 'id': '%.3f' % time.time(),
            'created': round(time.time(), 1), 'entries': entries,
            'rings': [[[round(float(x), 3), round(float(y), 3)] for x, y in r] for r in rings],
            'fp': dict(fingerprint or {}), 'base_done': int(carry.get('succeeded', 0)),
            'base_skipped': [list(s) for s in carry.get('skipped', [])]}


def new_cursor(plan):
    """k = 第一個還沒做完的段落；xy = 車子在第 k 段上開到哪裡；succeeded = 這一趟做完幾段
    (跟任務狀態的「完成」一樣，含轉場)；skipped = 到現在為止跳過的段 (含之前那幾趟)。"""
    return {'plan': plan.get('id'), 'k': 0, 'xy': None, 'succeeded': 0,
            'skipped': [list(s) for s in plan.get('base_skipped', [])],
            'state': 'running', 'why': '', 't': round(time.time(), 1)}


def summary(plan, cur):
    """給網頁看的「上一趟停在哪」(/mower_manager/resume_info)。"""
    entries = plan['entries']
    k = max(0, min(int(cur.get('k', 0)), len(entries)))
    rest, info = resume_entries(entries, k, cur.get('xy'))
    done = int(plan.get('base_done', 0)) + int(cur.get('succeeded', 0))
    skipped = len(cur.get('skipped', []))
    stopped = cur.get('state') != 'running'
    return {
        'available': bool(rest), 'session': plan.get('session'), 'planned': plan.get('created'),
        'stopped_t': cur.get('t'), 'why': cur.get('why') or ('' if stopped else '關電或程式中斷'),
        'next': info['from'] or '', 'left': len(rest),
        'left_m': round(sum(length_of(e) for e in rest), 1),
        'swaths_left': sum(1 for e in rest if e['kind'] == 'swath'),
        'edge_left': sum(1 for e in rest if e['kind'] == 'edge'),
        'done': done, 'skipped': skipped, 'total': done + skipped + len(rest),
        'cut_m': info['cut_m'], 'loop_back': info['loop_back'],
    }


# ---- 禁區、外框有沒有改過 -------------------------------------------------------
def _as_bytes(data):
    if isinstance(data, (bytes, bytearray)):
        return bytes(data)
    if hasattr(data, 'tobytes'):            # rclpy 的 OccupancyGrid.data 是 array.array('b')
        return data.tobytes()
    return bytes(int(v) & 0xFF for v in data)


def mask_fingerprint(resolution, origin_x, origin_y, width, height, data):
    """禁區遮罩 (/keepout_mask) 的指紋：同樣的禁區 (重開機從檔案載回來的也一樣) 得到同樣的字串。
    沒有禁區 (全部是 0，包括開機時 keepout_server 先發的 1x1 空遮罩) 一律是 ''。"""
    raw = _as_bytes(data)
    if raw.count(0) == len(raw):
        return ''
    h = hashlib.md5()
    h.update(('%.6f %.3f %.3f %d %d|' % (resolution, origin_x, origin_y, width, height)).encode())
    h.update(raw)
    return h.hexdigest()[:16]


def poly_fingerprint(points):
    """外框 (/lawn_boundary) 的指紋，座標取到公分。空的 (清除外框) 是 ''。"""
    if not points:
        return ''
    h = hashlib.md5()
    for x, y in points:
        h.update(('%.2f,%.2f;' % (x, y)).encode())
    return h.hexdigest()[:16]


def changed(old_fp, new_fp):
    """出發時的指紋 vs 現在的：回傳改過的東西 ['禁區', '外框']。"""
    old_fp, new_fp = old_fp or {}, new_fp or {}
    out = []
    for key, name in (('keepout', '禁區'), ('lawn', '外框')):
        if (old_fp.get(key) or '') != (new_fp.get(key) or ''):
            out.append(name)
    return out
