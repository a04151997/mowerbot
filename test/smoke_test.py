#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mowerbot workspace 自動化冒煙測試 (可重複執行)

用法:
    python3 test/smoke_test.py              # 跑全部 Phase A/B/C/D/L/N
    python3 test/smoke_test.py --phases AB  # 只跑指定 Phase
    python3 test/smoke_test.py --overlap=0.3           # 覆寫割草線重疊率
    python3 test/smoke_test.py --world=demo_lawn.world # 換模擬世界

特性:
    - 會自行 source /opt/ros/humble 與本 workspace 的 install/setup.bash
    - 使用獨立的 ROS_DOMAIN_ID (預設 77) 與機器上其他 ROS 節點隔離
    - 每個 Phase 結束會 kill 掉整個 process group，並清掃殘留的 gzserver/gzclient
    - 所有背景 launch 的 log 會存到 test/logs/<timestamp>/ 底下
"""

import os
import re
import sys
import math
import time
import bisect
import signal
import shlex
import subprocess
from datetime import datetime

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_SETUP = '/opt/ros/humble/setup.bash'
WS_SETUP = os.path.join(WS, 'install', 'setup.bash')


# --------------------------------------------------------------------------
# 0. 環境自動 source (只做一次，之後用環境變數標記避免無限遞迴)
# --------------------------------------------------------------------------
def ensure_ros_env():
    if os.environ.get('MOWERBOT_SMOKE_SOURCED') == '1':
        return
    if not os.path.isfile(ROS_SETUP):
        sys.stderr.write('找不到 %s，無法繼續\n' % ROS_SETUP)
        sys.exit(2)
    inner = (
        'source {ros}; '
        'if [ -f {ws} ]; then source {ws}; fi; '
        'export MOWERBOT_SMOKE_SOURCED=1; '
        'exec python3 {me} "$@"'
    ).format(ros=shlex.quote(ROS_SETUP), ws=shlex.quote(WS_SETUP),
             me=shlex.quote(os.path.abspath(__file__)))
    os.execvp('bash', ['bash', '-c', inner, 'bash'] + sys.argv[1:])


ensure_ros_env()

ENV = dict(os.environ)
ENV.setdefault('ROS_DOMAIN_ID', os.environ.get('MOWERBOT_SMOKE_DOMAIN', '77'))
ENV['RCUTILS_COLORIZED_OUTPUT'] = '0'
# ros2 launch 是 Python 程式，輸出導到檔案時預設會 block buffering，
# 導致測試中途想貼節點 log 只會讀到啟動那幾行。強制不緩衝。
ENV['PYTHONUNBUFFERED'] = '1'
os.environ['ROS_DOMAIN_ID'] = ENV['ROS_DOMAIN_ID']

STAMP = datetime.now().strftime('%Y%m%d_%H%M%S')
LOGDIR = os.path.join(WS, 'test', 'logs', STAMP)
os.makedirs(LOGDIR, exist_ok=True)

# 每條割草線前面的跑道 (lead-in) 長度，單位公尺。
# None = 不覆寫，直接用 mower_control.launch.py 的預設值。
# 用 --lead-in=L 或環境變數 MOWERBOT_LEAD_IN 指定，
# 目的是讓 L 的實測掃描不用每次重建 workspace。
LEAD_IN = os.environ.get('MOWERBOT_LEAD_IN')
for _a in sys.argv[1:]:
    if _a.startswith('--lead-in'):
        LEAD_IN = _a.split('=', 1)[1] if '=' in _a else None
LEAD_IN = float(LEAD_IN) if LEAD_IN not in (None, '') else None


# 割草線重疊率。None = 不覆寫，直接用 mower_control.launch.py 的預設值 (0.2)。
# 用 --overlap=R 或環境變數 MOWERBOT_OVERLAP 指定，理由同 LEAD_IN：
# 讓重疊率的實測掃描不用每次重建 workspace。
OVERLAP = os.environ.get('MOWERBOT_OVERLAP')
for _a in sys.argv[1:]:
    if _a.startswith('--overlap'):
        OVERLAP = _a.split('=', 1)[1] if '=' in _a else None
OVERLAP = float(OVERLAP) if OVERLAP not in (None, '') else None
# mower_control.launch.py 的 overlap_ratio 預設值。沒有覆寫時，測試自己呼叫 F2C
# 拿參考路徑也要用這個值，否則量到的是別條路徑的落差。兩邊必須同步修改。
DEFAULT_OVERLAP = 0.4

# 實際刀盤寬，單位公尺。這是車體的物理屬性，不隨重疊率改變，
# 覆蓋落差的判定基準固定是它的一半 (COVERAGE_TOL)。
BLADE_WIDTH = 0.5
COVERAGE_TOL = BLADE_WIDTH / 2.0
# 割草線間距 = 刀盤寬 x (1 - 重疊率)，要與 mower_manager 算出來的一致，
# 測試自己呼叫 F2C 拿參考路徑時必須用同一個值，否則量到的是別條路徑的落差。
SWATH_SPACING = BLADE_WIDTH * (1.0 - (OVERLAP if OVERLAP is not None else DEFAULT_OVERLAP))

# 模擬世界檔名。None = 用 gazebo.launch.py 的預設值 (mow_field.world)。
# 用 --world=<檔名> 或環境變數 MOWERBOT_WORLD 指定。
WORLD = os.environ.get('MOWERBOT_WORLD')
for _a in sys.argv[1:]:
    if _a.startswith('--world'):
        WORLD = _a.split('=', 1)[1] if '=' in _a else None
WORLD = WORLD or None


def mower_control_cmd():
    """mower_control.launch.py 的啟動指令，需要時帶上 lead_in_length / overlap_ratio 覆寫"""
    cmd = ['ros2', 'launch', 'mowerbot_bringup', 'mower_control.launch.py']
    if LEAD_IN is not None:
        cmd.append('lead_in_length:=%.3f' % LEAD_IN)
    if OVERLAP is not None:
        cmd.append('overlap_ratio:=%.3f' % OVERLAP)
    return cmd


def gazebo_cmd():
    """gazebo.launch.py 的啟動指令 (無頭)，需要時帶上 world 覆寫"""
    cmd = ['ros2', 'launch', 'mowerbot_bringup', 'gazebo.launch.py', 'gui:=false']
    if WORLD is not None:
        cmd.append('world:=%s' % WORLD)
    return cmd


class Tee(object):
    """把 stdout 同時寫到終端機與 transcript 檔"""

    def __init__(self, path):
        self.term = sys.__stdout__
        self.f = open(path, 'w', encoding='utf-8')

    def write(self, s):
        self.term.write(s)
        self.f.write(s)

    def flush(self):
        self.term.flush()
        self.f.flush()


sys.stdout = Tee(os.path.join(LOGDIR, 'smoke_test.log'))


# --------------------------------------------------------------------------
# 1. 報表框架
# --------------------------------------------------------------------------
PHASES = [
    ('A', '靜態檢查'),
    ('B', '單元功能'),
    ('C', '模擬整合'),
    ('D', '安全機制'),
    ('L', '存圖與定位'),
    ('N', 'Nav2 路徑跟隨'),
]
RESULTS = []          # (phase, cid, name, status, detail)


def hdr(text):
    print('')
    print('=' * 78)
    print(text)
    print('=' * 78)


def sub(text):
    print('  · %s' % text)


def record(phase, cid, name, status, detail=''):
    RESULTS.append((phase, cid, name, status, detail))
    line = '[%s] %s  %s' % (status, cid, name)
    if detail:
        line += '  ->  %s' % detail
    print('')
    print(line)
    print('')


# --------------------------------------------------------------------------
# 2. 子行程工具
# --------------------------------------------------------------------------
def run(cmd, timeout=60, cwd=None):
    """同步執行，回傳 (rc, 合併後的輸出)"""
    try:
        p = subprocess.run(cmd, cwd=cwd or WS, env=ENV, timeout=timeout,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        return p.returncode, p.stdout.decode('utf-8', 'replace')
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b'').decode('utf-8', 'replace')
        return 124, out + '\n[TIMEOUT after %ss]' % timeout
    except FileNotFoundError as e:
        return 127, '[NOT FOUND] %s' % e


def run_for(cmd, seconds):
    """跑固定秒數後送 SIGINT 收工 (給 ros2 topic hz / tf2_echo 用)"""
    return run(['timeout', '-s', 'INT', str(seconds)] + cmd, timeout=seconds + 20)


class Bg(object):
    """背景行程；獨立 session 以便整個 process group 一起 kill"""

    def __init__(self, name, cmd):
        self.name = name
        self.cmd = cmd
        self.logpath = os.path.join(LOGDIR, name + '.log')
        self.f = open(self.logpath, 'wb')
        self.f.write(('$ ' + ' '.join(cmd) + '\n').encode())
        self.f.flush()
        # 輸出導到檔案時，C++ 節點 (controller_server) 的 libc stdio 會 block buffering，
        # 測試中途想貼它的 log 只會讀到空的。stdbuf 透過 LD_PRELOAD 會被子行程繼承，
        # 因此在 ros2 launch 前面加上它就能讓底下的節點改成行緩衝。
        cmd = ['stdbuf', '-oL', '-eL'] + cmd
        self.p = subprocess.Popen(cmd, cwd=WS, env=ENV, stdout=self.f,
                                  stderr=subprocess.STDOUT,
                                  start_new_session=True)
        sub('啟動 %s (pid=%d, log=%s)' % (name, self.p.pid, self.logpath))

    def alive(self):
        return self.p.poll() is None

    def log_tail(self, n=25):
        try:
            with open(self.logpath, 'r', encoding='utf-8', errors='replace') as fh:
                return ''.join(fh.readlines()[-n:])
        except Exception as e:
            return '(讀不到 log: %s)' % e

    def log_grep(self, pattern, n=10):
        try:
            with open(self.logpath, 'r', encoding='utf-8', errors='replace') as fh:
                hits = [l.rstrip() for l in fh if re.search(pattern, l)]
            return hits[-n:]
        except Exception:
            return []

    def stop(self):
        if self.p.poll() is not None:
            self.f.close()
            return
        try:
            pgid = os.getpgid(self.p.pid)
        except OSError:
            return
        for sig, wait in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            try:
                os.killpg(pgid, sig)
            except OSError:
                break
            t0 = time.time()
            while time.time() - t0 < wait:
                if self.p.poll() is not None:
                    break
                time.sleep(0.2)
            if self.p.poll() is not None:
                break
        self.f.close()
        sub('已關閉 %s' % self.name)


BG = []


def bg_start(name, cmd):
    b = Bg(name, cmd)
    BG.append(b)
    return b


def bg_stop_all():
    for b in reversed(BG):
        b.stop()
    del BG[:]


def pgrep(pattern):
    """回傳符合 pattern 的 pid 清單，排除自己與自己的祖先行程"""
    rc, out = run(['pgrep', '-f', pattern], timeout=10)
    mine = {os.getpid(), os.getppid()}
    pids = []
    for tok in out.split():
        if not tok.strip().isdigit():
            continue
        pid = int(tok)
        if pid in mine:
            continue
        pids.append(pid)
    return pids


def sweep(patterns):
    """清掃 killpg 之後仍殘留的行程 (Gazebo classic 常有)。
    用 pid 逐一 kill，不用 pkill -f，因為 pattern 可能誤中執行本腳本的 shell。"""
    for pat in patterns:
        pids = pgrep(pat)
        if not pids:
            continue
        sub('清掃殘留 %s: pid %s' % (pat, ','.join(str(p) for p in pids)))
        for pid in pids:
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        time.sleep(1.0)


def domain_processes():
    """回傳 ROS_DOMAIN_ID 與本次測試相同的行程 [(pid, cmdline)]，排除自己與自己的祖先。

    用 /proc/<pid>/environ 逐一比對 domain，而不是比對行程名稱：
    機器上可能同時有跑在別的 domain 的 ROS 節點，用名稱比對會誤殺它們。"""
    want = ('ROS_DOMAIN_ID=' + ENV['ROS_DOMAIN_ID']).encode()
    mine = {os.getpid(), os.getppid()}
    found = []
    for name in os.listdir('/proc'):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in mine:
            continue
        try:
            with open('/proc/%d/environ' % pid, 'rb') as fh:
                if want not in fh.read().split(b'\0'):
                    continue
            with open('/proc/%d/cmdline' % pid, 'rb') as fh:
                cmd = fh.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()
        except (IOError, OSError):
            continue          # 行程剛結束，或沒有權限讀 (不是我們啟動的)
        if 'ros2cli.daemon' in cmd or 'ros2-daemon' in cmd:
            continue          # ros2 cli 自己的快取 daemon，不是機器人節點，留著
        found.append((pid, cmd or '(無 cmdline)'))
    return found


def preflight_domain_clean():
    """開跑前的防呆：清掉 ROS_DOMAIN_ID 上前一次執行留下的節點。

    測試若被 Ctrl-C 或其他方式中途打斷，gzserver / controller_server /
    mower_manager 有機會活下來。下一次執行時這些殘留節點會跟新起的節點
    搶同一組 topic 與 service，數據會被安靜地污染 (階段 8 就因此丟棄過
    一整輪結果)。所以開跑前先看一眼，有殘留就清掉並記錄清掉了什麼。"""
    sub('防呆檢查：ROS_DOMAIN_ID=%s 上有沒有前次留下的殘留節點'
        % ENV['ROS_DOMAIN_ID'])
    # 一律加 --no-daemon：ros2 cli 的 daemon 會快取圖形資訊，
    # 可能報出已經消失的節點 (或漏報剛出現的)，殘留判斷要看當下的真實圖形。
    _rc, out = run(['ros2', 'node', 'list', '--no-daemon'], timeout=30)
    nodes = [l.strip() for l in out.splitlines() if l.strip().startswith('/')]
    procs = domain_processes()
    if not nodes and not procs:
        sub('  乾淨，沒有殘留 (節點 0 個、行程 0 個)')
        return
    sub('  發現殘留，開跑前先清掉：')
    for n in nodes:
        sub('    節點 %s' % n)
    for pid, cmd in procs:
        sub('    行程 pid=%-7d %s' % (pid, cmd[:110]))
    for pid, _cmd in procs:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(2.0)
    _rc, out = run(['ros2', 'node', 'list', '--no-daemon'], timeout=30)
    left = [l.strip() for l in out.splitlines() if l.strip().startswith('/')]
    if left:
        sub('  清完仍看得到節點 %s —— 這次的數據要當成可疑，請人工確認' % left)
    else:
        sub('  已清乾淨 (殺掉 %d 個行程)' % len(procs))


# --------------------------------------------------------------------------
# 3. Phase A：靜態檢查
# --------------------------------------------------------------------------
EXPECTED_PKGS = ['joy_tester', 'mowerbot_action', 'mowerbot_bridge',
                 'mowerbot_bringup', 'mowerbot_description',
                 'mowerbot_interfaces', 'mowerbot_planner']


def a1_build():
    hdr('A1  colcon build')
    t0 = time.time()
    rc, out = run(['colcon', 'build'], timeout=1800)
    dt = time.time() - t0
    with open(os.path.join(LOGDIR, 'colcon_build.log'), 'w', encoding='utf-8') as fh:
        fh.write(out)
    print(out.strip()[-3000:])
    m = re.search(r'Summary:\s+(\d+)\s+packages? finished', out)
    n_fin = int(m.group(1)) if m else 0
    n_fail = 0
    mf = re.search(r'(\d+)\s+packages? failed', out)
    if mf:
        n_fail = int(mf.group(1))
    rc2, names = run(['colcon', 'list', '--names-only'], timeout=120)
    found = sorted([n for n in names.split() if n])
    print('')
    sub('耗時: %.1f s' % dt)
    sub('colcon 回傳碼: %d' % rc)
    sub('finished=%d  failed=%d' % (n_fin, n_fail))
    sub('workspace 套件 (%d 個): %s' % (len(found), ', '.join(found)))
    missing = [p for p in EXPECTED_PKGS if p not in found]
    ok = (rc == 0 and n_fail == 0 and n_fin == 7 and not missing)
    detail = 'build rc=%d, finished=%d/7, failed=%d' % (rc, n_fin, n_fail)
    if missing:
        detail += ', 缺少套件=%s' % missing
    record('A', 'A1', 'colcon build 全部通過 (7 套件)',
           'PASS' if ok else 'FAIL', detail)


LAUNCH_FILES = [
    ('mowerbot_bringup', 'gazebo.launch.py'),
    ('mowerbot_bringup', 'mapping.launch.py'),
    ('mowerbot_bringup', 'mower_control.launch.py'),
    ('mowerbot_bringup', 'navigation.launch.py'),
    ('mowerbot_bringup', 'localization.launch.py'),
    ('mowerbot_description', 'robot_state_publisher.launch.py'),
    ('mowerbot_description', 'display.launch.py'),
    ('mowerbot_description', 'rviz.launch.py'),
]


def a2_show_args():
    hdr('A2  每個 launch 檔 --show-args 解析')
    bad = []
    for pkg, lf in LAUNCH_FILES:
        rc, out = run(['ros2', 'launch', pkg, lf, '--show-args'], timeout=120)
        args = re.findall(r"^\s+'([A-Za-z0-9_]+)':", out, re.M)
        status = 'OK' if rc == 0 else 'ERR(rc=%d)' % rc
        sub('%-22s %-34s %s  args=%s' % (pkg, lf, status, args))
        if rc != 0:
            bad.append('%s/%s' % (pkg, lf))
            print(('     ' + out.strip()[-600:]).replace('\n', '\n     '))
    record('A', 'A2', '所有 launch 檔可正常解析',
           'PASS' if not bad else 'FAIL',
           '%d/%d 可解析' % (len(LAUNCH_FILES) - len(bad), len(LAUNCH_FILES)) +
           ('' if not bad else ', 失敗=%s' % bad))


EXECUTABLES = [
    ('mowerbot_planner', 'f2c_server'),
    ('mowerbot_action', 'mower_manager'),
    ('mowerbot_action', 'map_to_boundary'),
    ('mowerbot_bridge', 'teleop_node'),
]


def a3_executables():
    hdr('A3  執行檔存在性')
    missing = []
    for pkg, exe in EXECUTABLES:
        rc, out = run(['ros2', 'pkg', 'executables', pkg], timeout=60)
        listed = ('%s %s' % (pkg, exe)) in out
        path = os.path.join(WS, 'install', pkg, 'lib', pkg, exe)
        on_disk = os.path.exists(path)
        sub('%-18s %-16s ros2-pkg-executables=%-5s  檔案=%-5s  %s'
            % (pkg, exe, listed, on_disk, path))
        if not (listed and on_disk):
            missing.append('%s/%s' % (pkg, exe))
    record('A', 'A3', '4 個執行檔都找得到',
           'PASS' if not missing else 'FAIL',
           '%d/4 存在' % (len(EXECUTABLES) - len(missing)) +
           ('' if not missing else ', 缺少=%s' % missing))


INTERFACES = [
    ('mowerbot_interfaces/msg/MowerStatus',
     ['battery_voltage', 'battery_current', 'battery_percentage',
      'bumper_pressed', 'stop_active', 'is_overheated', 'mode']),
    ('mowerbot_interfaces/msg/MotorStatus',
     ['left_front_rpm', 'right_front_rpm', 'left_behind_rpm', 'right_behind_rpm',
      'left_front_encoder', 'right_front_encoder', 'left_behind_encoder',
      'right_behind_encoder']),
    ('mowerbot_interfaces/srv/SetDriveMode', ['mode', 'success']),
    ('mowerbot_interfaces/srv/GenerateCoveragePath',
     ['boundary', 'tool_width', 'turning_radius', 'coverage_path', 'success']),
]


def a4_interfaces():
    hdr('A4  ros2 interface show')
    bad = []
    for iface, fields in INTERFACES:
        rc, out = run(['ros2', 'interface', 'show', iface], timeout=60)
        print('--- %s (rc=%d) ---' % (iface, rc))
        print(out.rstrip())
        lack = [f for f in fields if not re.search(r'\b%s\b' % re.escape(f), out)]
        if rc != 0 or lack:
            bad.append('%s%s' % (iface, (' 缺欄位%s' % lack) if lack else ' rc!=0'))
        else:
            sub('%s 欄位齊全 (%d 個)' % (iface, len(fields)))
        print('')
    record('A', 'A4', '4 個介面欄位正確',
           'PASS' if not bad else 'FAIL',
           '%d/4 正確' % (len(INTERFACES) - len(bad)) +
           ('' if not bad else ', 問題=%s' % bad))


def a5_xacro():
    hdr('A5  xacro 解析 car.xacro')
    cands = [os.path.join(WS, 'install', 'mowerbot_description', 'share',
                          'mowerbot_description', 'urdf', 'car.xacro'),
             os.path.join(WS, 'src', 'mowerbot_description', 'urdf', 'car.xacro')]
    path = next((c for c in cands if os.path.exists(c)), None)
    if path is None:
        record('A', 'A5', 'xacro 解析 car.xacro', 'SKIP', '找不到 car.xacro')
        return
    sub('使用: %s' % path)
    rc, out = run(['xacro', path], timeout=120)
    if rc != 0:
        print(out.strip()[-2000:])
        record('A', 'A5', 'xacro 解析 car.xacro', 'FAIL',
               'xacro rc=%d，見上方錯誤' % rc)
        return
    urdf_path = os.path.join(LOGDIR, 'car.urdf')
    with open(urdf_path, 'w', encoding='utf-8') as fh:
        fh.write(out)
    try:
        import xml.etree.ElementTree as ET
        root = ET.fromstring(out)
        links = [e.get('name') for e in root.findall('link')]
        joints = [(e.get('name'), e.get('type')) for e in root.findall('joint')]
    except Exception as e:
        record('A', 'A5', 'xacro 解析 car.xacro', 'FAIL', 'XML 解析失敗: %s' % e)
        return
    sub('link  總數 = %d : %s' % (len(links), links))
    sub('joint 總數 = %d : %s' % (len(joints), [j[0] for j in joints]))
    sub('展開後 URDF 已存到 %s' % urdf_path)
    record('A', 'A5', 'xacro 解析無錯誤', 'PASS',
           'link=%d, joint=%d' % (len(links), len(joints)))


# --------------------------------------------------------------------------
# 4. Phase B：單元功能
# --------------------------------------------------------------------------
def b1_f2c():
    hdr('B1  F2C 路徑規劃 (只啟動 f2c_server)')
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point32
    from mowerbot_interfaces.srv import GenerateCoveragePath

    srv = bg_start('B1_f2c_server',
                   ['ros2', 'run', 'mowerbot_planner', 'f2c_server',
                    '--ros-args', '-p', 'use_sim_time:=false'])
    rclpy.init()
    node = Node('smoke_b1')
    cli = node.create_client(GenerateCoveragePath, 'generate_coverage_path')
    try:
        if not cli.wait_for_service(timeout_sec=20.0):
            print(srv.log_tail(30))
            record('B', 'B1', 'F2C 路徑規劃', 'FAIL',
                   'generate_coverage_path 服務 20 秒內沒上線 (見上方 f2c_server log)')
            return
        req = GenerateCoveragePath.Request()
        corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)]
        for x, y in corners:
            req.boundary.points.append(Point32(x=x, y=y, z=0.0))
        req.tool_width = 0.5
        req.turning_radius = 1.0
        sub('邊界 = %s (10m x 10m 正方形)' % corners)
        sub('tool_width = %.2f m, turning_radius = %.2f m' % (req.tool_width, req.turning_radius))

        fut = cli.call_async(req)
        t0 = time.time()
        rclpy.spin_until_future_complete(node, fut, timeout_sec=90.0)
        dt = time.time() - t0
        if not fut.done():
            print(srv.log_tail(30))
            record('B', 'B1', 'F2C 路徑規劃', 'FAIL', '服務呼叫 90 秒逾時')
            return
        resp = fut.result()
        poses = resp.coverage_path.poses
        pts = [(p.pose.position.x, p.pose.position.y) for p in poses]

        seglens = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        total = sum(seglens)
        avg = (total / len(seglens)) if seglens else 0.0
        mx = max(seglens) if seglens else 0.0

        print('')
        sub('success                = %s' % resp.success)
        sub('coverage_path.poses 總數 = %d' % len(poses))
        sub('路徑總長度              = %.3f m' % total)
        sub('相鄰航點平均間距        = %.3f m' % avg)
        sub('相鄰航點最大間距        = %.3f m' % mx)
        sub('服務耗時                = %.2f s' % dt)
        sub('前 5 個 pose (x, y):')
        for i, (x, y) in enumerate(pts[:5]):
            print('        [%d] (%.4f, %.4f)' % (i, x, y))
        if not pts:
            print('        (沒有任何 pose)')
        print('')
        print('  註：間距數值僅供觀察，不列入 PASS/FAIL 判定 (依規格)')

        record('B', 'B1', 'F2C 路徑規劃 success==true',
               'PASS' if resp.success else 'FAIL',
               'success=%s, poses=%d, 總長=%.2fm, 平均間距=%.3fm, 最大間距=%.3fm'
               % (resp.success, len(poses), total, avg, mx))
    finally:
        node.destroy_node()
        rclpy.shutdown()


def b2_boundary():
    hdr('B2  邊界提取 (只啟動 map_to_boundary，餵假地圖)')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PolygonStamped

    W = H = 200
    RES = 0.05
    FREE_LO, FREE_HI = 50, 149          # 含頭含尾的 cell index
    exp_lo = FREE_LO * RES              # 2.50 m
    exp_hi = FREE_HI * RES              # 7.45 m

    node_proc = bg_start('B2_map_to_boundary',
                         ['ros2', 'run', 'mowerbot_action', 'map_to_boundary',
                          '--ros-args', '-p', 'use_sim_time:=false'])
    rclpy.init()
    node = Node('smoke_b2')
    map_qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(OccupancyGrid, '/map', map_qos)
    got = []
    node.create_subscription(PolygonStamped, '/f2c_boundary',
                             lambda m: got.append(m), 10)

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = RES
    grid.info.width = W
    grid.info.height = H
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0
    grid.info.origin.orientation.w = 1.0
    data = [100] * (W * H)
    for row in range(FREE_LO, FREE_HI + 1):
        base = row * W
        for col in range(FREE_LO, FREE_HI + 1):
            data[base + col] = 0
    grid.data = data

    sub('假地圖: %dx%d, resolution=%.3f  (=%.1fm x %.1fm)' % (W, H, RES, W * RES, H * RES))
    sub('free space cell [%d..%d]^2  ->  世界座標 %.2f ~ %.2f m' % (FREE_LO, FREE_HI, exp_lo, exp_hi))
    sub('其餘 cell 值 = 100 (佔據)')

    try:
        deadline = time.time() + 25.0
        while time.time() < deadline and not got:
            grid.header.stamp = node.get_clock().now().to_msg()
            pub.publish(grid)
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.8)
        if not got:
            print(node_proc.log_tail(30))
            record('B', 'B2', '邊界提取', 'FAIL',
                   '25 秒內 /f2c_boundary 沒有任何訊息 (見上方 map_to_boundary log)')
            return
        msg = got[-1]
        pts = [(p.x, p.y, p.z) for p in msg.polygon.points]
        print('')
        sub('收到 PolygonStamped, frame_id = %s' % msg.header.frame_id)
        sub('多邊形頂點數 = %d' % len(pts))
        sub('各頂點座標 (x, y, z) 公尺:')
        for i, (x, y, z) in enumerate(pts):
            print('        [%d] (%.4f, %.4f, %.4f)  -> cell (%.1f, %.1f)'
                  % (i, x, y, z, x / RES, y / RES))
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        dev = max(abs(min(xs) - exp_lo), abs(max(xs) - exp_hi),
                  abs(min(ys) - exp_lo), abs(max(ys) - exp_hi))
        print('')
        sub('觀測 x 範圍 = %.3f ~ %.3f m   (預期 %.2f ~ %.2f)' % (min(xs), max(xs), exp_lo, exp_hi))
        sub('觀測 y 範圍 = %.3f ~ %.3f m   (預期 %.2f ~ %.2f)' % (min(ys), max(ys), exp_lo, exp_hi))
        sub('與預期 free space 區域最大偏差 = %.3f m (%.1f 個 cell)' % (dev, dev / RES))
        ok = len(pts) >= 4
        record('B', 'B2', '邊界提取 (收到 PolygonStamped 且頂點數>=4)',
               'PASS' if ok else 'FAIL',
               '頂點數=%d, 座標偏差=%.3fm' % (len(pts), dev))
    finally:
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# 5. Phase C：模擬整合
# --------------------------------------------------------------------------
EXPECTED_NODES = ['robot_state_publisher', 'joint_state_publisher', 'slam_toolbox',
                  'joy_node', 'mower_teleop', 'mower_manager', 'map_to_boundary',
                  'f2c_server']

# 節點名 -> 用來 pgrep 的行程特徵，C1 診斷用
NODE_PROC_PATTERN = {
    'robot_state_publisher': 'robot_state_publisher',
    'joint_state_publisher': 'joint_state_publisher',
    'slam_toolbox': 'async_slam_toolbox_node',
    'joy_node': 'lib/joy/joy_node',
    'mower_teleop': 'mowerbot_bridge/teleop_node',
    'mower_manager': 'mowerbot_action/mower_manager',
    'map_to_boundary': 'mowerbot_action/map_to_boundary',
    'f2c_server': 'mowerbot_planner/f2c_server',
}


def topic_hz(topic, dur=5):
    rc, out = run_for(['ros2', 'topic', 'hz', topic], dur)
    rates = re.findall(r'average rate:\s*([\d.]+)', out)
    return (float(rates[-1]) if rates else None), out


def tf_echo(parent, child, dur=5):
    rc, out = run_for(['ros2', 'run', 'tf2_ros', 'tf2_echo', parent, child,
                       '--ros-args', '-p', 'use_sim_time:=true'], dur)
    m = re.findall(r'Translation:\s*\[\s*([-\d.eE+]+),\s*([-\d.eE+]+),\s*([-\d.eE+]+)\]', out)
    if m:
        return tuple(float(v) for v in m[-1]), out
    return None, out


def phase_c():
    hdr('Phase C  模擬整合測試 (gazebo.launch.py gui:=false + mower_control.launch.py)')
    gz = bg_start('C_gazebo',
                  gazebo_cmd())
    sub('等待 Gazebo 起來 (18 秒)...')
    time.sleep(18)
    if not gz.alive():
        print(gz.log_tail(40))
        for cid, name in [('C1', '節點清單'), ('C2', 'Topic 頻率'), ('C3', 'TF 樹'),
                          ('C4', '服務存在'), ('C5', 'Gazebo RTF')]:
            record('C', cid, name, 'SKIP', 'gazebo.launch.py 啟動失敗，無法測試')
        return
    ctl = bg_start('C_mower_control',
                   mower_control_cmd())
    sub('等待系統穩定 (25 秒)...')
    time.sleep(25)

    # ---- C1 節點清單 ----
    hdr('C1  節點清單')
    # 節點多的時候 ros2 node list 預設 1 秒的 discovery 有機會漏掉節點，
    # 因此拉長 spin-time 並重試 3 次取聯集，避免把「探索漏抓」誤判成「節點沒起來」。
    seen = []
    rsp_count = 0
    for attempt in range(1, 4):
        rc, out = run(['ros2', 'node', 'list', '--spin-time', '3'], timeout=60)
        nodes = [n.strip() for n in out.split() if n.strip().startswith('/')]
        bare = [n.lstrip('/').split('/')[-1] for n in nodes]
        rsp_count = max(rsp_count, bare.count('robot_state_publisher'))
        for b in bare:
            if b not in seen:
                seen.append(b)
        sub('第 %d 次查詢: %d 個節點，累計聯集 %d 個' % (attempt, len(bare), len(seen)))
        if all(n in seen for n in EXPECTED_NODES):
            break
        time.sleep(2.0)
    print('')
    print('  節點聯集: %s' % ', '.join(sorted(seen)))
    print('')
    missing = [n for n in EXPECTED_NODES if n not in seen]
    for n in EXPECTED_NODES:
        sub('%-24s %s' % (n, 'OK' if n in seen else '缺少'))
    sub('robot_state_publisher 數量 = %d (預期 1)' % rsp_count)
    if missing:
        print('')
        sub('缺少節點的行程狀態診斷 (區分「行程掛掉」與「discovery 漏抓」):')
        for n in missing:
            pat = NODE_PROC_PATTERN.get(n, n)
            pids = pgrep(pat)
            print('        %-24s 行程 pattern=%-28s pid=%s'
                  % (n, pat, pids if pids else '(找不到行程 -> 節點確實沒起來)'))
    ok = (not missing) and rsp_count == 1
    record('C', 'C1', '必要節點都在、robot_state_publisher 只有一個',
           'PASS' if ok else 'FAIL',
           '缺少=%s, rsp數量=%d' % (missing if missing else '無', rsp_count))

    # ---- C2 Topic 頻率 ----
    hdr('C2  Topic 頻率 (每個量測 5 秒)')
    specs = [('/scan', 7.0, 13.0, '約 10 Hz'),
             ('/odom', 24.0, 36.0, '約 30 Hz'),
             ('/tf', 0.01, 1e9, '有資料即可'),
             ('/clock', 0.01, 1e9, '有資料即可 (驗證 use_sim_time)')]
    bad = []
    for topic, lo, hi, desc in specs:
        hz, raw = topic_hz(topic, 5)
        if hz is None:
            sub('%-8s 實測 = 沒有資料      預期 %s   -> 不合格' % (topic, desc))
            bad.append('%s(無資料)' % topic)
        else:
            good = lo <= hz <= hi
            sub('%-8s 實測 = %8.3f Hz   預期 %s   -> %s'
                % (topic, hz, desc, 'OK' if good else '不合格'))
            if not good:
                bad.append('%s(%.2fHz)' % (topic, hz))
    record('C', 'C2', 'Topic 頻率符合預期',
           'PASS' if not bad else 'FAIL',
           '%d/4 符合' % (len(specs) - len(bad)) + ('' if not bad else ', 不合格=%s' % bad))

    # ---- C3 TF 樹 ----
    hdr('C3  TF 樹完整性 map -> odom -> base_footprint -> base_link -> radar')
    chain = [('map', 'odom'), ('odom', 'base_footprint'),
             ('base_footprint', 'base_link'), ('base_link', 'radar')]
    bad = []
    radar_z = None
    for parent, child in chain:
        tr, raw = tf_echo(parent, child, 5)
        if tr is None:
            sub('%-16s -> %-14s 查不到 transform' % (parent, child))
            print(('     ' + raw.strip()[-400:]).replace('\n', '\n     '))
            bad.append('%s->%s' % (parent, child))
        else:
            sub('%-16s -> %-14s Translation = [%.4f, %.4f, %.4f]'
                % (parent, child, tr[0], tr[1], tr[2]))
            if (parent, child) == ('base_link', 'radar'):
                radar_z = tr[2]
    if radar_z is None:
        z_ok = False
        sub('base_link -> radar 的 z: 查不到')
    else:
        z_ok = abs(radar_z - 0.45) < 0.01
        sub('base_link -> radar 的 z = %.4f (預期 0.45) -> %s' % (radar_z, 'OK' if z_ok else '不符'))
    record('C', 'C3', 'TF 鏈路完整且 base_link->radar z==0.45',
           'PASS' if (not bad and z_ok) else 'FAIL',
           '斷鏈=%s, radar_z=%s' % (bad if bad else '無',
                                    ('%.4f' % radar_z) if radar_z is not None else 'N/A'))

    # ---- C4 服務 ----
    hdr('C4  服務存在')
    rc, out = run(['ros2', 'service', 'list'], timeout=60)
    want = ['change_mower_mode', 'generate_coverage_path']
    miss = []
    for s in want:
        found = re.search(r'^/?%s$' % re.escape(s), out, re.M) is not None
        sub('%-26s %s' % (s, 'OK' if found else '缺少'))
        if not found:
            miss.append(s)
    record('C', 'C4', '2 個服務都存在', 'PASS' if not miss else 'FAIL',
           '缺少=%s' % (miss if miss else '無'))

    # ---- C5 RTF ----
    hdr('C5  Gazebo real-time factor (gz stats -d 5)')
    rc, out = run(['gz', 'stats', '-d', '5'], timeout=40)
    print(out.strip()[-1500:])
    factors = [float(x) for x in re.findall(r'Factor\[\s*([\d.]+)\]', out)]
    if not factors:
        record('C', 'C5', 'Gazebo real-time factor', 'FAIL',
               'gz stats 沒有解析到 Factor (rc=%d)' % rc)
    else:
        avg = sum(factors) / len(factors)
        print('')
        sub('取樣數 = %d' % len(factors))
        sub('RTF 平均 = %.3f，最小 = %.3f，最大 = %.3f' % (avg, min(factors), max(factors)))
        ok = avg >= 0.5
        record('C', 'C5', 'Gazebo real-time factor >= 0.5',
               'PASS' if ok else 'FAIL',
               'RTF 平均=%.3f (min=%.3f, max=%.3f)%s'
               % (avg, min(factors), max(factors),
                  '' if ok else '  ← 低於 0.5，VM 效能不足'))


# --------------------------------------------------------------------------
# 6. Phase D：安全機制
# --------------------------------------------------------------------------
class SafetyRig(object):
    """Phase D 專用：只啟動 mower_manager + mower_teleop (不含 Gazebo，走 wall clock)"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Joy
        from geometry_msgs.msg import Twist
        from mowerbot_interfaces.srv import SetDriveMode

        yaml = os.path.join(WS, 'install', 'mowerbot_bridge', 'share',
                            'mowerbot_bridge', 'config', 'joystick.yaml')
        self.manager = bg_start('D_mower_manager',
                                ['ros2', 'run', 'mowerbot_action', 'mower_manager',
                                 '--ros-args', '-p', 'use_sim_time:=false'])
        self.teleop = bg_start('D_mower_teleop',
                               ['ros2', 'run', 'mowerbot_bridge', 'teleop_node',
                                '--ros-args', '--params-file', yaml,
                                '-p', 'use_sim_time:=false'])
        sub('teleop 參數檔: %s' % yaml)
        rclpy.init()
        self.node = Node('smoke_d')
        self.joy_pub = self.node.create_publisher(Joy, '/joy', 10)
        self.joy_vel_pub = self.node.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.nav_vel_pub = self.node.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.cmd_vel = []
        self.cmd_vel_joy = []
        self.node.create_subscription(Twist, '/cmd_vel',
                                      lambda m: self.cmd_vel.append(m.linear.x), 50)
        self.node.create_subscription(Twist, '/cmd_vel_joy',
                                      lambda m: self.cmd_vel_joy.append(m.linear.x), 50)
        self.mode_cli = self.node.create_client(SetDriveMode, 'change_mower_mode')
        self.Joy = Joy
        self.Twist = Twist
        self.SetDriveMode = SetDriveMode

    def spin(self, seconds):
        import rclpy
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self, timeout=20.0):
        return self.mode_cli.wait_for_service(timeout_sec=timeout)

    def set_mode(self, mode):
        import rclpy
        req = self.SetDriveMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        return fut.result().success if fut.done() and fut.result() is not None else None

    def pub_joy(self, axes, buttons, seconds, rate=20.0):
        msg = self.Joy()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        t0 = time.time()
        period = 1.0 / rate
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                msg.header.stamp = self.node.get_clock().now().to_msg()
                self.joy_pub.publish(msg)
                nxt = now + period
            self.spin(0.01)

    def pub_twist(self, pub, x, seconds, rate=10.0):
        msg = self.Twist()
        msg.linear.x = float(x)
        t0 = time.time()
        period = 1.0 / rate
        nxt = t0
        n = 0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                pub.publish(msg)
                n += 1
                nxt = now + period
            self.spin(0.01)
        return n

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()


def phase_d():
    hdr('Phase D  安全機制測試 (只啟動 mower_manager + mower_teleop，使用 wall clock)')
    rig = SafetyRig()
    try:
        if not rig.wait_ready(20.0):
            print(rig.manager.log_tail(30))
            for cid, name in [('D1', '手把死鎖 deadman'), ('D2', 'Watchdog 超時'),
                              ('D3', '急停鎖定'), ('D4', '模式隔離')]:
                record('D', cid, name, 'SKIP', 'change_mower_mode 服務沒上線，mower_manager 可能沒起來')
            return
        sub('mower_manager 服務已上線，暖機 3 秒')
        rig.spin(3.0)

        # ---- D1 ----
        hdr('D1  手把死鎖 deadman (axis_linear=1, button_deadman=4)')
        axes = [0.0, 0.8, 0.0, 0.0]
        del rig.cmd_vel_joy[:]
        sub('(a) axes=%s, buttons=[0,0,0,0,0,0] (沒按 LB)，持續 2 秒' % axes)
        rig.pub_joy(axes, [0, 0, 0, 0, 0, 0], 2.0)
        rig.spin(0.5)
        a_vals = list(rig.cmd_vel_joy)
        a_max = max([abs(v) for v in a_vals]) if a_vals else None
        sub('    /cmd_vel_joy 收到 %d 筆, linear.x 最大絕對值 = %s'
            % (len(a_vals), ('%.6f' % a_max) if a_max is not None else 'N/A'))

        del rig.cmd_vel_joy[:]
        sub('(b) axes=%s, buttons=[0,0,0,0,1,0] (按住 LB)，持續 2 秒' % axes)
        rig.pub_joy(axes, [0, 0, 0, 0, 1, 0], 2.0)
        rig.spin(0.5)
        b_vals = list(rig.cmd_vel_joy)
        b_max = max([abs(v) for v in b_vals]) if b_vals else None
        b_typ = max(set([round(v, 4) for v in b_vals]), key=[round(v, 4) for v in b_vals].count) if b_vals else None
        sub('    /cmd_vel_joy 收到 %d 筆, linear.x 最大絕對值 = %s, 最常見值 = %s'
            % (len(b_vals), ('%.6f' % b_max) if b_max is not None else 'N/A', b_typ))
        sub('    (預期 0.8 * scale_linear 0.7 = 0.56)')

        if not a_vals or not b_vals:
            record('D', 'D1', '手把死鎖 deadman', 'FAIL',
                   '(a)收到%d筆 (b)收到%d筆，teleop 可能沒有回應 /joy' % (len(a_vals), len(b_vals)))
        else:
            ok = (a_max == 0.0) and (b_max > 0.0)
            record('D', 'D1', '沒按 LB 全為 0、按住 LB 非 0',
                   'PASS' if ok else 'FAIL',
                   '(a) %d筆 max=%.6f ; (b) %d筆 max=%.6f'
                   % (len(a_vals), a_max, len(b_vals), b_max))

        # ---- D2 ----
        hdr('D2  Watchdog 超時保護 (mode 2 手動，manager 預設值)')
        del rig.cmd_vel[:]
        sub('(a) 對 /cmd_vel_joy 以 10Hz 送 linear.x=0.3，持續 2 秒')
        n = rig.pub_twist(rig.joy_vel_pub, 0.3, 2.0, 10.0)
        rig.spin(0.3)
        fwd = list(rig.cmd_vel)
        got03 = [v for v in fwd if abs(v - 0.3) < 1e-6]
        sub('    發出 %d 筆；/cmd_vel 收到 %d 筆，其中 linear.x==0.3 有 %d 筆'
            % (n, len(fwd), len(got03)))
        sub('    /cmd_vel 最大值 = %s' % (('%.4f' % max(fwd)) if fwd else 'N/A'))

        del rig.cmd_vel[:]
        sub('(b) 停止發布，等 1.5 秒 (watchdog_timeout = 0.5 s)')
        rig.spin(1.5)
        tail = list(rig.cmd_vel)
        last = tail[-1] if tail else None
        sub('    這 1.5 秒 /cmd_vel 收到 %d 筆，最後一筆 linear.x = %s'
            % (len(tail), ('%.6f' % last) if last is not None else 'N/A'))
        a_ok = len(got03) > 0
        b_ok = (last is not None and last == 0.0)
        record('D', 'D2', 'Watchdog：轉發 0.3 後超時歸零',
               'PASS' if (a_ok and b_ok) else 'FAIL',
               '(a) 收到0.3=%s(%d筆) ; (b) 最後一筆=%s'
               % (a_ok, len(got03), ('%.6f' % last) if last is not None else 'N/A'))

        # ---- D3 ----
        hdr('D3  急停鎖定 (mode 4)')
        ok_srv = rig.set_mode(4)
        sub('change_mower_mode(mode=4) 回傳 success = %s' % ok_srv)
        rig.spin(0.5)
        del rig.cmd_vel[:]
        sub('持續 3 秒對 /cmd_vel_joy 送 linear.x=0.5 (10Hz)，同時蒐集 /cmd_vel')
        n = rig.pub_twist(rig.joy_vel_pub, 0.5, 3.0, 10.0)
        rig.spin(0.3)
        vals = list(rig.cmd_vel)
        nonzero = [v for v in vals if v != 0.0]
        sub('    發出 %d 筆；/cmd_vel 蒐集到 %d 筆' % (n, len(vals)))
        sub('    非 0 的筆數 = %d，最大絕對值 = %s'
            % (len(nonzero), ('%.6f' % max([abs(v) for v in vals])) if vals else 'N/A'))
        hits = rig.manager.log_grep(r'急停|攔截|E-STOP|強制鎖定', 10)
        sub('    mower_manager log 攔截訊息 (%d 筆):' % len(hits))
        for h in hits:
            print('        %s' % h)
        if not vals:
            record('D', 'D3', '急停鎖定', 'FAIL', '/cmd_vel 完全沒有訊息，無法驗證')
        else:
            record('D', 'D3', '急停時 /cmd_vel 全為 0',
                   'PASS' if not nonzero else 'FAIL',
                   '%d 筆中有 %d 筆非 0；log 攔截訊息 %d 筆'
                   % (len(vals), len(nonzero), len(hits)))

        # ---- D4 ----
        hdr('D4  模式隔離 (mode 2 手動時不轉發 /cmd_vel_nav)')
        ok_srv = rig.set_mode(2)
        sub('change_mower_mode(mode=2) 回傳 success = %s' % ok_srv)
        rig.spin(0.5)
        del rig.cmd_vel[:]
        sub('對 /cmd_vel_nav 以 10Hz 送 linear.x=0.4，持續 2 秒')
        n = rig.pub_twist(rig.nav_vel_pub, 0.4, 2.0, 10.0)
        rig.spin(0.3)
        vals = list(rig.cmd_vel)
        leaked = [v for v in vals if abs(v - 0.4) < 1e-6]
        sub('    發出 %d 筆；/cmd_vel 蒐集到 %d 筆' % (n, len(vals)))
        sub('    出現 0.4 的筆數 = %d，最大絕對值 = %s'
            % (len(leaked), ('%.6f' % max([abs(v) for v in vals])) if vals else 'N/A'))
        record('D', 'D4', '手動模式不轉發導航指令',
               'PASS' if not leaked else 'FAIL',
               '%d 筆中有 %d 筆是 0.4' % (len(vals), len(leaked)))
    finally:
        rig.close()


# --------------------------------------------------------------------------
# 6.5 Phase N：Nav2 區域路徑控制 (只有 controller_server)
# --------------------------------------------------------------------------
def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def split_swaths_like_manager(pts):
    """用與 mower_manager.split_path_into_swaths 相同的規則切割：
    相鄰段方向變化達 90 度就切開，只有 2 個點的橫向連接段丟掉。"""
    cuts = []
    prev = None
    for i in range(len(pts) - 1):
        dx = pts[i + 1][0] - pts[i][0]
        dy = pts[i + 1][1] - pts[i][1]
        n = math.hypot(dx, dy)
        if n < 1e-9:
            continue
        cur = (dx / n, dy / n)
        if prev is not None and prev[0] * cur[0] + prev[1] * cur[1] <= 1e-9:
            cuts.append(i)
        prev = cur
    out = []
    start = 0
    for c in cuts + [len(pts) - 1]:
        seg = pts[start:c + 1]
        if len(seg) >= 3:
            out.append(seg)
        start = c
    return out


def coverage_gap(pts, traj):
    """覆蓋落差：對每一個「被指令要走過的航點」，算它離車子實際軌跡最近有多遠。

    這是最直接回答「會不會留下沒割到的縫」的指標，而且沒有「這個取樣點屬於哪條
    割草線」的指派問題 —— 方向反過來算，從路徑看向軌跡。approach 多走的路只會
    讓覆蓋更好、不會更差，所以也不需要扣除。

    判定基準固定是「實際刀盤寬的一半」= BLADE_WIDTH / 2 = 0.25 m，
    航點離實際軌跡超過這個距離的地方就是沒割到的縫。
    重疊率只會縮小割草線間距，不會改變刀盤寬，所以這個門檻不隨重疊率變動。

    回傳 (最大落差, 平均, 中位, 超過門檻的航點數, 總航點數, 最差的位置)。
    """
    if len(traj) < 2 or not pts:
        return (float('nan'),) * 3 + (0, 0, (float('nan'), float('nan')))
    vals = []
    worst = -1.0
    worst_pt = (float('nan'), float('nan'))
    for px, py in pts:
        d = min(_point_seg_dist(px, py, traj[i][1], traj[i][2],
                                traj[i + 1][1], traj[i + 1][2])
                for i in range(len(traj) - 1))
        vals.append(d)
        if d > worst:
            worst, worst_pt = d, (px, py)
    ordered = sorted(vals)
    return (worst, sum(vals) / len(vals), ordered[len(ordered) // 2],
            sum(1 for v in vals if v > COVERAGE_TOL), len(vals), worst_pt)


def lateral_deviation_per_swath(traj, swaths, trim_radius=None):
    """割草品質指標：對每個軌跡取樣點，判斷它當下正在割第幾條割草線，
    再算它到那條線的垂直距離。

    判斷方式用「單調前進的最近割草線」而不是累積弧長：弓字形的線距只有 0.5 m，
    而車子每條線都會在 goal tolerance 0.25 m 內提早結束、掉頭又會多繞一點路，
    累積弧長跟路徑弧長會逐漸對不起來，一旦偏掉就會把車子指派到隔壁那條線，
    量出來的是線距不是循跡誤差。

    只採計「投影落在該條割草線頭尾範圍內」的取樣點 —— 掉頭迴轉時車子本來就會
    超出線段兩端，那是轉彎不是割歪。

    trim_radius：先丟掉軌跡開頭的 approach 移動段。這一步是必要的 ——
    覆蓋任務開始時車子停在場地中央，那個位置離「最後幾條」割草線反而最近，
    不丟掉的話單調前進的指標一開始就會跳到最後一條，前面幾條全部量不到。

    回傳 (最大偏差, 平均偏差, 採用點數, 每條割草線的最大偏差, 丟棄點數)。
    """
    if not swaths or not traj:
        return float('nan'), float('nan'), 0, [], 0
    lines = [(sw[0], sw[-1]) for sw in swaths]

    dropped = 0
    if trim_radius is not None:
        for i, item in enumerate(traj):
            if math.dist((item[1], item[2]), swaths[0][0]) <= trim_radius:
                traj = traj[i:]
                dropped = i
                break
    k = 0
    worst = 0.0
    acc = 0.0
    used = 0
    per_swath = [0.0] * len(lines)
    for item in traj:
        x, y = item[1], item[2]
        while k + 1 < len(lines):
            a1, b1 = lines[k]
            a2, b2 = lines[k + 1]
            d_cur = _point_seg_dist(x, y, a1[0], a1[1], b1[0], b1[1])
            d_next = _point_seg_dist(x, y, a2[0], a2[1], b2[0], b2[1])
            if d_next < d_cur:
                k += 1
            else:
                break
        a, b = lines[k]
        vx, vy = b[0] - a[0], b[1] - a[1]
        l2 = vx * vx + vy * vy
        t = (((x - a[0]) * vx + (y - a[1]) * vy) / l2) if l2 > 0 else 0.0
        if t < 0.0 or t > 1.0:
            continue          # 迴轉中，不算循跡誤差
        d = _point_seg_dist(x, y, a[0], a[1], b[0], b[1])
        worst = max(worst, d)
        per_swath[k] = max(per_swath[k], d)
        acc += d
        used += 1
    return worst, (acc / used if used else float('nan')), used, per_swath, dropped


def lateral_deviation_by_arclength(traj, pts, trim_radius=None):
    """真實的循跡誤差：先用累積弧長判斷車子當下「應該在第幾條線段」，
    再算到那條指定線段的垂直距離。

    舊寫法是取「到路徑上任一線段的最短距離」，弓字形線距 0.5 m 時
    數學上恆 <= 0.25 m，車子卡在兩條線中間也看起來很漂亮，量不出東西。

    trim_radius：先把軌跡開頭「還沒抵達路徑起點」的那一段丟掉。覆蓋任務前面
    有一段 approach 移動 (刀盤關閉、不屬於割草作業)，不扣掉的話那整段的距離
    會被算進循跡誤差，量出來的是 approach 的長度而不是割草品質。

    回傳 (最大偏差, 平均偏差, 丟棄的取樣數, 實際採用的取樣數)。
    """
    if len(pts) < 2 or not traj:
        return float('nan'), float('nan'), 0, 0

    used = traj
    dropped = 0
    if trim_radius is not None:
        for i, item in enumerate(traj):
            if math.dist((item[1], item[2]), pts[0]) <= trim_radius:
                used = traj[i:]
                dropped = i
                break
        else:
            # 車子從來沒有靠近過路徑起點，無從扣除，整段照算
            used = traj
            dropped = 0
    if len(used) < 2:
        return float('nan'), float('nan'), dropped, len(used)

    seg_cum = [0.0]
    for i in range(len(pts) - 1):
        seg_cum.append(seg_cum[-1] + math.dist(pts[i], pts[i + 1]))
    path_total = seg_cum[-1]

    worst = 0.0
    acc = 0.0
    travelled = 0.0
    prev = None
    for item in used:
        x, y = item[1], item[2]
        if prev is not None:
            travelled += math.dist(prev, (x, y))
        prev = (x, y)
        s_clamped = min(travelled, path_total)
        j = bisect.bisect_right(seg_cum, s_clamped) - 1
        j = max(0, min(j, len(pts) - 2))
        d = _point_seg_dist(x, y, pts[j][0], pts[j][1], pts[j + 1][0], pts[j + 1][1])
        worst = max(worst, d)
        acc += d
    return worst, acc / len(used), dropped, len(used)


def _point_seg_dist(px, py, ax, ay, bx, by):
    """點到線段的最短距離"""
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    l2 = vx * vx + vy * vy
    t = 0.0 if l2 == 0.0 else max(0.0, min(1.0, (wx * vx + wy * vy) / l2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _resample(corners, spacing):
    """把折線以固定間距重新取樣，回傳 [(x, y), ...]"""
    pts = [corners[0]]
    for i in range(len(corners) - 1):
        ax, ay = corners[i]
        bx, by = corners[i + 1]
        seg = math.hypot(bx - ax, by - ay)
        n = max(1, int(round(seg / spacing)))
        for k in range(1, n + 1):
            t = float(k) / n
            pts.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return pts


class NavRig(object):
    """Phase N 用的測試節點：讀 odom、送 FollowPath、監聽兩條速度話題"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import FollowPath
        from mowerbot_interfaces.srv import SetDriveMode

        rclpy.init()
        self.node = Node('smoke_n')
        self.odom = []            # (wall_t, x, y)
        # 速度指令必須同時記錄 linear.x 與 angular.z：
        # 原地旋轉的指令 linear.x 是 0、只有 angular.z，只看 linear.x 會誤判成「沒有輸出」
        self.cmd_vel = []         # (linear.x, angular.z)
        self.cmd_vel_nav = []     # (linear.x, angular.z)
        self.node.create_subscription(Odometry, '/odom', self._odom_cb, 20)
        self.node.create_subscription(
            Twist, '/cmd_vel', lambda m: self.cmd_vel.append((m.linear.x, m.angular.z)), 50)
        self.node.create_subscription(
            Twist, '/cmd_vel_nav',
            lambda m: self.cmd_vel_nav.append((m.linear.x, m.angular.z)), 50)
        self.mode_cli = self.node.create_client(SetDriveMode, 'change_mower_mode')
        self.ac = ActionClient(self.node, FollowPath, 'follow_path')
        from geometry_msgs.msg import PolygonStamped
        self.boundary_pub = self.node.create_publisher(PolygonStamped, '/f2c_boundary', 10)
        self.PolygonStamped = PolygonStamped
        self.FollowPath = FollowPath
        self.SetDriveMode = SetDriveMode

    def _odom_cb(self, msg):
        self.odom.append((time.time(),
                          msg.pose.pose.position.x,
                          msg.pose.pose.position.y))

    def spin(self, seconds):
        import rclpy
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_odom(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not self.odom:
            self.spin(0.2)
        return self.odom[-1] if self.odom else None

    def set_mode(self, mode):
        import rclpy
        if not self.mode_cli.wait_for_service(timeout_sec=10.0):
            return None
        req = self.SetDriveMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        return fut.result().success if fut.done() and fut.result() is not None else None

    def send_path(self, path_msg, timeout=15.0):
        """送出 FollowPath goal，回傳 (goal_handle 或 None, 說明)"""
        import rclpy
        if not self.ac.wait_for_server(timeout_sec=timeout):
            return None, 'follow_path action server 沒上線'
        goal = self.FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = 'FollowPath'
        goal.goal_checker_id = 'general_goal_checker'
        fut = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            return None, 'send_goal 逾時'
        gh = fut.result()
        if not gh.accepted:
            return None, 'goal 被 controller_server 拒絕'
        return gh, 'accepted'

    def publish_boundary(self, x0, y0, half=3.0, seconds=3.0):
        """對 /f2c_boundary 發布一個以 (x0, y0) 為中心的正方形邊界，
        讓 mower_manager 有邊界可以拿去呼叫 F2C"""
        from geometry_msgs.msg import Point32
        msg = self.PolygonStamped()
        msg.header.frame_id = 'map'
        corners = [(x0 - half, y0 - half), (x0 + half, y0 - half),
                   (x0 + half, y0 + half), (x0 - half, y0 + half)]
        for cx, cy in corners:
            msg.polygon.points.append(Point32(x=float(cx), y=float(cy), z=0.0))
        t0 = time.time()
        while time.time() - t0 < seconds:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.boundary_pub.publish(msg)
            self.spin(0.3)
        return corners

    def close(self):
        import rclpy
        self.node.destroy_node()
        rclpy.shutdown()


STATUS_NAME = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
               4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}

# 世界檔裡的物體 (名稱, 形狀, 中心, 尺寸/半徑, yaw)。
# 以前這份清單是寫死的 mow_field.world 內容，用 --world 換世界時會去檢查
# 根本不存在的障礙物，印出來的淨空數字是假的。改成直接解析實際載入的 .world。


def parse_world_models(world_name):
    """從 mowerbot_bringup/worlds/<world_name> 解析出所有靜態物體。

    回傳 [(名稱, 'box'|'cylinder', (cx, cy), 尺寸, yaw)]，
    尺寸對 box 是 (sx, sy)、對 cylinder 是 (radius,)。
    <include> 進來的 sun / ground_plane 沒有 <model> 標籤，自然不會被收進來。
    """
    import xml.etree.ElementTree as ET
    share = run(['ros2', 'pkg', 'prefix', 'mowerbot_bringup'], timeout=30)[1].strip()
    path = os.path.join(share, 'share', 'mowerbot_bringup', 'worlds', world_name)
    if not os.path.isfile(path):
        return None, path
    out = []
    for model in ET.parse(path).getroot().iter('model'):
        name = model.get('name') or '(無名)'
        pose = (model.findtext('pose') or '0 0 0 0 0 0').split()
        cx, cy = float(pose[0]), float(pose[1])
        yaw = float(pose[5]) if len(pose) >= 6 else 0.0
        geom = None
        for link in model.iter('link'):
            for coll in link.iter('collision'):
                box = coll.find('.//box/size')
                cyl = coll.find('.//cylinder/radius')
                if box is not None:
                    v = box.text.split()
                    geom = ('box', (float(v[0]), float(v[1])))
                elif cyl is not None:
                    geom = ('cylinder', (float(cyl.text),))
                if geom:
                    break
            if geom:
                break
        if geom:
            out.append((name, geom[0], (cx, cy), geom[1], yaw))
    return out, path


def _model_footprint(kind, center, dims, yaw):
    """物體在地面上的多邊形輪廓；圓柱用外接正方形近似，只拿來判斷相鄰關係。"""
    if kind == 'cylinder':
        r = dims[0]
        return _rot_box_corners(center[0], center[1], 2 * r, 2 * r, 0.0)
    return _rot_box_corners(center[0], center[1], dims[0], dims[1], yaw)


def classify_world_models(models, touch_tol=0.05):
    """把物體分成「邊界圍牆」與「作業區內部的障礙物」。

    規則：先把碰到整體外緣的物體當成圍牆 (種子)，
    再把任何與已知圍牆相接的物體也算成圍牆 (角落的短翼、方柱、凹角側板)，
    重複到收斂為止。剩下的就是內部障礙物。
    這樣才不會把 demo_lawn.world 貼在牆上的角落特徵誤判成內部障礙物。
    """
    if not models:
        return [], []
    foots = [_model_footprint(k, c, d, y) for _, k, c, d, y in models]
    xs = [p[0] for f in foots for p in f]
    ys = [p[1] for f in foots for p in f]
    X0, X1, Y0, Y1 = min(xs), max(xs), min(ys), max(ys)

    wall = set()
    for i, f in enumerate(foots):
        fx0 = min(p[0] for p in f); fx1 = max(p[0] for p in f)
        fy0 = min(p[1] for p in f); fy1 = max(p[1] for p in f)
        if (abs(fx0 - X0) <= touch_tol or abs(fx1 - X1) <= touch_tol
                or abs(fy0 - Y0) <= touch_tol or abs(fy1 - Y1) <= touch_tol):
            wall.add(i)
    changed = True
    while changed:
        changed = False
        for i in range(len(foots)):
            if i in wall:
                continue
            if any(_poly_poly_dist(foots[i], foots[j]) <= touch_tol for j in wall):
                wall.add(i)
                changed = True
    walls = [models[i] for i in sorted(wall)]
    inner = [models[i] for i in range(len(models)) if i not in wall]
    return inner, walls

# N2 的測試邊界：5m x 5m，中心 (-1.5, -1.5)。
# 這是掃過 mow_field.world 之後挑的——它包含車子起始位置 (0, 0)，
# 而且離每一個障礙物與牆面都超過 1.5 m (實際最小 2.05 m)。
# 障礙物情境要另外單獨測，而且正解是讓 F2C 把障礙物當成 Cell 的內環(hole)
# 排除掉，讓路徑根本不經過，不是靠 costmap 硬閃。
TEST_BOUNDARY_CENTER = (-1.5, -1.5)
TEST_BOUNDARY_HALF = 2.5
MIN_OBSTACLE_CLEARANCE = 1.5


def _rot_box_corners(cx, cy, sx, sy, yaw):
    hx, hy = sx / 2.0, sy / 2.0
    c, sn = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - sn * dy, cy + sn * dx + c * dy)
            for dx, dy in ((hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy))]


def _poly_poly_dist(a, b):
    best = float('inf')
    for i in range(len(a)):
        a1, a2 = a[i], a[(i + 1) % len(a)]
        for j in range(len(b)):
            b1, b2 = b[j], b[(j + 1) % len(b)]
            best = min(best,
                       _point_seg_dist(b1[0], b1[1], a1[0], a1[1], a2[0], a2[1]),
                       _point_seg_dist(a1[0], a1[1], b1[0], b1[1], b2[0], b2[1]))
    return best


def boundary_clearances(corners, models):
    """回傳 [(障礙物名稱, 中心, 與邊界最近邊的距離)]"""
    out = []
    for name, kind, center, dims, yaw in models:
        if kind == 'cylinder':
            d = min(_point_seg_dist(center[0], center[1],
                                    corners[i][0], corners[i][1],
                                    corners[(i + 1) % 4][0], corners[(i + 1) % 4][1])
                    for i in range(4)) - dims[0]
        else:
            d = _poly_poly_dist(corners,
                                _rot_box_corners(center[0], center[1],
                                                 dims[0], dims[1], yaw))
        out.append((name, center, d))
    return out


# mower_manager 的任務 log 格式，Phase N 靠這些字串判讀任務進度
RE_SPLIT = re.compile(r'覆蓋路徑切出 (\d+) 條割草線')
RE_APPROACH_MADE = re.compile(r'產生 approach 路徑：距離 ([\d.]+) m，(\d+) 個航點')
RE_APPROACH_SKIP = re.compile(r'跳過 approach：車子距離第 1 條割草線起點只有 ([\d.]+) m')
RE_APPROACH_DONE = re.compile(r'approach 完成，開始割草')
RE_DONE_ONE = re.compile(r'✅ 割草線 (\d+)/(\d+) 完成')
RE_FAIL_ONE = re.compile(r'❌ \[([^\]]+)\] 失敗 \(status=(\S+?)\)')
RE_MISSION_DONE = re.compile(r'覆蓋任務完成！共完成 (\d+) 條割草線')
# f2c_server 的地頭 (headland) 資訊，由 mower_control.launch.py 一起帶起來，
# 所以會出現在 N_mower_control.log 裡。
RE_HEADLAND = re.compile(
    r'地頭寬度 ([\d.]+) m：原始面積 ([\d.]+) m\^2 -> 內縮後作業面積 ([\d.]+) m\^2')
RE_SWATH_COUNT = re.compile(r'割草線計算成功！割草線 (\d+) 條')


def read_headland_state(bg):
    """從 f2c_server 的 log 讀出地頭寬度、面積與割草線數。

    割草線數一律從 log 動態取得，不要寫死：加了地頭之後可作業面積縮小，
    割草線數量本來就會跟著變 (5x5 邊界扣掉兩側地頭)。
    """
    try:
        with open(bg.logpath, 'r', encoding='utf-8', errors='replace') as fh:
            txt = fh.read()
    except Exception:
        return None
    hl = RE_HEADLAND.findall(txt)
    sw = RE_SWATH_COUNT.findall(txt)
    if not hl:
        return None
    return {
        'width': float(hl[-1][0]),
        'field_area': float(hl[-1][1]),
        'mainland_area': float(hl[-1][2]),
        'swaths': int(sw[-1]) if sw else 0,
    }


def read_mission_state(bg):
    """從 mower_manager 的 log 判讀覆蓋任務進度"""
    try:
        with open(bg.logpath, 'r', encoding='utf-8', errors='replace') as fh:
            txt = fh.read()
    except Exception:
        return {'total': 0, 'completed': 0, 'failures': [], 'done': False}
    splits = RE_SPLIT.findall(txt)
    dones = RE_DONE_ONE.findall(txt)
    fails = RE_FAIL_ONE.findall(txt)
    finish = RE_MISSION_DONE.findall(txt)
    made = RE_APPROACH_MADE.findall(txt)
    skipped = RE_APPROACH_SKIP.findall(txt)
    return {
        'total': int(splits[-1]) if splits else 0,
        'completed': len(dones),
        'done_list': dones,
        'failures': fails,
        'done': bool(finish),
        'approach': ('made', float(made[-1][0]), int(made[-1][1])) if made
                    else (('skipped', float(skipped[-1]), 0) if skipped else None),
        'approach_done': bool(RE_APPROACH_DONE.search(txt)),
    }


def build_mow_path(rig, x0, y0):
    """以車子目前位置為起點，產生 3 條 4m 割草線、間距 0.5m 的弓字形路徑。

    往 -y 方向鋪設：世界座標 (3.5, 2.0) 有障礙物，往 +y 會太靠近它的膨脹層。
    """
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped

    line_len = 4.0
    spacing_line = 0.5
    wp_spacing = 0.1
    corners = [
        (x0, y0), (x0 + line_len, y0),
        (x0 + line_len, y0 - spacing_line), (x0, y0 - spacing_line),
        (x0, y0 - 2 * spacing_line), (x0 + line_len, y0 - 2 * spacing_line),
    ]
    pts = _resample(corners, wp_spacing)

    path = Path()
    path.header.frame_id = 'odom'
    path.header.stamp = rig.node.get_clock().now().to_msg()
    for i, (x, y) in enumerate(pts):
        nxt = pts[i + 1] if i + 1 < len(pts) else pts[i]
        prv = pts[i - 1] if i > 0 else pts[i]
        dx, dy = (nxt[0] - x, nxt[1] - y) if i + 1 < len(pts) else (x - prv[0], y - prv[1])
        yaw = math.atan2(dy, dx)
        q = _quat_from_yaw(yaw)
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation.x, ps.pose.orientation.y = q[0], q[1]
        ps.pose.orientation.z, ps.pose.orientation.w = q[2], q[3]
        path.poses.append(ps)
    total = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    return path, pts, total, corners


def phase_n():
    hdr('Phase N  Nav2 區域路徑控制 (gazebo gui:=false + mower_control + navigation)')
    gz = bg_start('N_gazebo',
                  gazebo_cmd())
    sub('等待 Gazebo 起來 (18 秒)...')
    time.sleep(18)
    ctl = bg_start('N_mower_control',
                   mower_control_cmd())
    sub('等待 mower_control 穩定 (20 秒)...')
    time.sleep(20)
    nav = bg_start('N_navigation',
                   ['ros2', 'launch', 'mowerbot_bringup', 'navigation.launch.py'])

    def skip_rest(reason, start_at):
        allc = [('N2', '真實 F2C 覆蓋路徑'), ('N3', '端到端覆蓋任務'),
                ('N4', 'remap 驗證'), ('N5', 'Nav2 運行下的 RTF'),
                ('N6', '急停清空佇列')]
        for cid, name in allc:
            if cid >= start_at:
                record('N', cid, name, 'SKIP', reason)

    # ---- N1 生命週期 ----
    hdr('N1  Nav2 生命週期 (等 controller_server 進入 active，最多 30 秒)')
    state = None
    t0 = time.time()
    while time.time() - t0 < 30.0:
        rc, out = run(['ros2', 'lifecycle', 'get', '/controller_server'], timeout=15)
        cur = out.strip().splitlines()[-1].strip() if out.strip() else '(無回應)'
        if cur != state:
            sub('t=%4.1fs  lifecycle state = %s' % (time.time() - t0, cur))
            state = cur
        if cur.startswith('active'):
            break
        time.sleep(2.0)
    sub('最終 lifecycle state = %s' % state)
    active = bool(state and state.startswith('active'))
    record('N', 'N1', 'controller_server 進入 active',
           'PASS' if active else 'FAIL', 'lifecycle state = %s' % state)
    if not active:
        print('')
        print('--- controller_server / navigation.launch.py 完整 log ---')
        print(nav.log_tail(200))
        skip_rest('controller_server 沒有 active，後續無法測試', 'N2')
        return

    rig = NavRig()
    try:
        # ---- N2 真實 F2C 路徑 ----
        hdr('N2  取得真實的 F2C 覆蓋路徑 (驗證航點內插)')
        cur = rig.wait_odom(20.0)
        if cur is None:
            record('N', 'N2', '真實 F2C 覆蓋路徑', 'FAIL', '20 秒內讀不到 /odom')
            skip_rest('沒有 odom 起點，無法產生邊界', 'N3')
            return
        _, x0, y0 = cur
        sub('車子目前 odom 位置 = (%.3f, %.3f)' % (x0, y0))

        import rclpy
        from geometry_msgs.msg import Point32
        from mowerbot_interfaces.srv import GenerateCoveragePath
        f2c_cli = rig.node.create_client(GenerateCoveragePath, 'generate_coverage_path')
        if not f2c_cli.wait_for_service(timeout_sec=20.0):
            record('N', 'N2', '真實 F2C 覆蓋路徑', 'FAIL', 'generate_coverage_path 服務沒上線')
            skip_rest('拿不到 F2C 路徑', 'N3')
            return
        req = GenerateCoveragePath.Request()
        bcx, bcy = TEST_BOUNDARY_CENTER
        half = TEST_BOUNDARY_HALF
        corners = [(bcx - half, bcy - half), (bcx + half, bcy - half),
                   (bcx + half, bcy + half), (bcx - half, bcy + half)]
        for cx, cy in corners:
            req.boundary.points.append(Point32(x=float(cx), y=float(cy), z=0.0))
        req.tool_width = SWATH_SPACING
        req.turning_radius = 1.0
        sub('測試邊界 = %.1fm x %.1fm，中心 (%.2f, %.2f)'
            % (half * 2, half * 2, bcx, bcy))
        sub('四個角座標: %s' % ['(%.2f, %.2f)' % c for c in corners])
        inside = (bcx - half <= x0 <= bcx + half) and (bcy - half <= y0 <= bcy + half)
        sub('是否包含車子起始位置 (%.2f, %.2f) = %s' % (x0, y0, inside))
        sub('實際刀盤寬 = %.2f m, 重疊率 = %.2f, tool_width (割草線間距) = %.3f m, '
            'turning_radius = 1.00 m'
            % (BLADE_WIDTH, OVERLAP if OVERLAP is not None else DEFAULT_OVERLAP,
               SWATH_SPACING))
        sub('(與 mower_manager 的 blade_width x (1 - overlap_ratio) 一致，'
            '否則量到的是別條路徑的落差)')
        print('')
        world_name = WORLD or 'mow_field.world'
        models, world_path = parse_world_models(world_name)
        if models is None:
            sub('障礙物淨空檢查：找不到世界檔 %s，無法解析' % world_path)
            inner, walls, clearances = [], [], []
            min_clear, too_close = float('nan'), []
            clearance_ok = False
        else:
            inner, walls = classify_world_models(models)
            sub('障礙物淨空檢查 (依實際載入的 %s 解析，不再寫死):' % world_name)
            sub('  解析到 %d 個靜態物體：邊界圍牆 %d 個、作業區內部障礙物 %d 個'
                % (len(models), len(walls), len(inner)))
            clearances = boundary_clearances(corners, inner)
            if not inner:
                sub('  此世界無內部障礙物，淨空檢查不適用')
                if walls:
                    wall_min = min(d for _, _, d in boundary_clearances(corners, walls))
                    sub('  (參考用：測試邊界離最近的圍牆 %.2f m)' % wall_min)
                min_clear, too_close = float('nan'), []
                clearance_ok = True
            else:
                sub('  內部障礙物與測試邊界的距離 (要求全部 >= %.1f m):'
                    % MIN_OBSTACLE_CLEARANCE)
                for name, center, dist in clearances:
                    flag = 'OK' if dist >= MIN_OBSTACLE_CLEARANCE else '太近'
                    print('        %-22s 位置 (%6.2f, %6.2f)   與邊界最近邊距離 %6.2f m   %s'
                          % (name, center[0], center[1], dist, flag))
                min_clear = min(d for _, _, d in clearances)
                too_close = [n for n, _, d in clearances if d < MIN_OBSTACLE_CLEARANCE]
                sub('  最小淨空 = %.2f m  (%s)'
                    % (min_clear, '全部達標' if not too_close
                       else '未達標: %s' % too_close))
                clearance_ok = not too_close
            if walls:
                sub('  邊界圍牆 (參考用，不列入判定): %s'
                    % ', '.join(n for n, _, _, _, _ in walls))
        fut = f2c_cli.call_async(req)
        rclpy.spin_until_future_complete(rig.node, fut, timeout_sec=60.0)
        if not fut.done() or fut.result() is None:
            record('N', 'N2', '真實 F2C 覆蓋路徑', 'FAIL', 'F2C 服務呼叫逾時')
            skip_rest('拿不到 F2C 路徑', 'N3')
            return
        resp = fut.result()
        f2c_path = resp.coverage_path
        pts = [(q.pose.position.x, q.pose.position.y) for q in f2c_path.poses]
        gaps = [math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
        avg_gap = sum(gaps) / len(gaps) if gaps else float('nan')
        max_gap = max(gaps) if gaps else float('nan')
        total_len = sum(gaps)
        print('')
        sub('success        = %s' % resp.success)
        sub('航點總數       = %d' % len(pts))
        sub('平均間距       = %.4f m  (判定門檻 < 0.15)' % avg_gap)
        sub('最大間距       = %.4f m  (= 割草線間距，那是兩條割草線之間的橫向連接段)' % max_gap)
        sub('路徑總長       = %.2f m' % total_len)
        sub('frame_id       = %s' % f2c_path.header.frame_id)
        obstacles = [(3.5, 2.0), (-4.2, 5.5), (6.0, -5.0), (-6.5, -3.0)]
        obs_clear = min(min(math.hypot(px - ox, py - oy) for px, py in pts)
                        for ox, oy in obstacles)
        sub('路徑離最近障礙物中心 = %.2f m  (world 座標的箱子在 (3.5, 2.0))' % obs_clear)
        hl = read_headland_state(ctl)
        print('')
        if hl is None:
            sub('地頭 (headland)   = f2c_server log 裡沒有地頭資訊 (可能是舊版節點)')
        else:
            sub('地頭寬度         = %.2f m  (0 代表停用)' % hl['width'])
            sub('原始邊界面積     = %.2f m^2' % hl['field_area'])
            sub('內縮後作業面積   = %.2f m^2  (佔原始 %.1f%%)'
                % (hl['mainland_area'],
                   100.0 * hl['mainland_area'] / hl['field_area']
                   if hl['field_area'] else 0.0))
            sub('割草線數         = %d 條  (動態取得，不寫死)' % hl['swaths'])
        n2_ok = bool(resp.success) and avg_gap < 0.15 and clearance_ok
        record('N', 'N2', '真實 F2C 路徑、平均間距 < 0.15 m、邊界離內部障礙物 >= 1.5 m',
               'PASS' if n2_ok else 'FAIL',
               'success=%s, 航點=%d, 平均間距=%.4fm, 最大間距=%.4fm, 總長=%.2fm, 最小淨空=%s'
               % (resp.success, len(pts), avg_gap, max_gap, total_len,
                  '%.2fm' % min_clear if inner else '不適用 (無內部障礙物)'))

        # ---- N3 端到端覆蓋任務 ----
        hdr('N3  端到端覆蓋任務 (manager 逐條割草線循序執行)')
        sub('說明：manager 的 call_f2c_planner() 綁在 mode 1 (F2C)，mode 3 不會觸發規劃，')
        sub('      因此這裡用 mode 1 啟動任務；nav_vel_cb 在 mode 1 與 3 都會轉發速度。')
        sub('先對 /f2c_boundary 發布同一個 5m x 5m 邊界給 manager')
        rig.publish_boundary(bcx, bcy, half=half, seconds=3.0)

        del rig.odom[:]
        del rig.cmd_vel[:]
        del rig.cmd_vel_nav[:]
        before = read_mission_state(ctl)
        ok_mode = rig.set_mode(1)
        sub('change_mower_mode(mode=1 F2C) 回傳 success = %s' % ok_mode)

        # 【量測修正，不是放寬判定】監聽窗口原本寫死 180 秒。
        # 重疊率掃到 0.5 時路徑從 35.5 m 變成 67.8 m，任務還在正常割草、
        # 一條都沒有 ABORTED，窗口就先到了，於是被記成 done=False、完成 15/16。
        # 那是「看得不夠久」而不是「機器做不到」。窗口改成跟著路徑長度伸縮。
        # 判定標準完全沒有動：仍然是「所有割草線 SUCCEEDED 且完成數 == 總數」。
        # 係數 4.0 s/m 來自實測 (55.6 m 的路徑跑 157 s，約 2.8 s/m) 再加四成餘裕。
        mission_timeout = max(180.0, 4.0 * total_len)
        sub('監聽最多 %.0f 秒  (= max(180, 4.0 s/m x 路徑總長 %.1f m))'
            % (mission_timeout, total_len))
        t0 = time.time()
        last_report = 0.0
        st = before
        while time.time() - t0 < mission_timeout:
            rclpy.spin_once(rig.node, timeout_sec=0.05)
            st = read_mission_state(ctl)
            if st['done'] or st['failures']:
                break
            el = time.time() - t0
            if el - last_report >= 20.0:
                last_report = el
                pos = (rig.odom[-1][1], rig.odom[-1][2]) if rig.odom else (float('nan'),) * 2
                sub('  t=%5.1fs  位置 (%.2f, %.2f)  已完成 %d/%d 條'
                    % (el, pos[0], pos[1], st['completed'], st['total']))
        elapsed = time.time() - t0

        traj = list(rig.odom)
        moved = sum(math.dist((traj[i][1], traj[i][2]), (traj[i + 1][1], traj[i + 1][2]))
                    for i in range(len(traj) - 1)) if len(traj) > 1 else 0.0
        end_err = math.dist((traj[-1][1], traj[-1][2]), pts[-1]) if traj else float('nan')
        # 把軌跡與路徑存檔，方便事後離線覆核這些數字
        try:
            with open(os.path.join(LOGDIR, 'n3_path.csv'), 'w') as fh:
                fh.write('x,y\n')
                for px, py in pts:
                    fh.write('%.6f,%.6f\n' % (px, py))
            with open(os.path.join(LOGDIR, 'n3_traj.csv'), 'w') as fh:
                fh.write('t,x,y\n')
                for tt, px, py in traj:
                    fh.write('%.3f,%.6f,%.6f\n' % (tt, px, py))
        except Exception as exc:
            sub('(軌跡存檔失敗: %s)' % exc)

        swath_lines = split_swaths_like_manager(pts)
        # 0.35 m = goal checker 的 xy_goal_tolerance 0.25 m 再加一點餘裕
        max_dev, mean_dev, n_used, per_swath, n_drop_sw = lateral_deviation_per_swath(
            traj, swath_lines, trim_radius=0.35)
        # 對照用：舊的累積弧長指標 (扣掉 approach 移動段)
        arc_dev, arc_mean, n_drop, _n = lateral_deviation_by_arclength(
            traj, pts, trim_radius=0.35)
        print('')
        if st['approach'] is None:
            sub('approach             = (manager 沒有產生，也沒有記錄跳過)')
        elif st['approach'][0] == 'made':
            sub('approach             = 距離 %.2f m，%d 個航點，結果 = %s'
                % (st['approach'][1], st['approach'][2],
                   'SUCCEEDED' if st['approach_done'] else '未完成'))
        else:
            sub('approach             = 跳過 (車子距離第 1 條起點只有 %.2f m)'
                % st['approach'][1])
        sub('manager 切出割草線數 = %d' % st['total'])
        sub('各割草線結果         = %s'
            % (', '.join('%s/%s SUCCEEDED' % d for d in st['done_list']) or '無'))
        sub('完成條數             = %d / %d' % (st['completed'], st['total']))
        sub('失敗的任務           = %s'
            % ([('[%s] status=%s' % f) for f in st['failures']] if st['failures'] else '無'))
        sub('任務是否走完         = %s' % st['done'])
        sub('耗時                 = %.1f s' % elapsed)
        sub('車子軌跡總長         = %.2f m   (F2C 路徑總長 %.2f m)' % (moved, total_len))
        sub('終點位置             = (%.3f, %.3f)'
            % ((traj[-1][1], traj[-1][2]) if traj else (float('nan'),) * 2))
        sub('路徑終點             = (%.3f, %.3f)' % pts[-1])
        sub('終點位置誤差         = %.3f m  (判定門檻 < 0.3)' % end_err)
        gap_max, gap_mean, gap_med, gap_over, gap_n, gap_pt = coverage_gap(pts, traj)
        print('')
        sub('【覆蓋品質】實際刀盤寬 %.2f m，指令航點離實際軌跡超過 %.2f m 就是沒割到的縫'
            % (BLADE_WIDTH, COVERAGE_TOL))
        sub('  最大覆蓋落差       = %.3f m   於 (%.2f, %.2f)' % (gap_max, gap_pt[0], gap_pt[1]))
        sub('  平均 / 中位        = %.3f m / %.3f m' % (gap_mean, gap_med))
        sub('  超過 %.2f m 的航點 = %d / %d  (%.1f%%)'
            % (COVERAGE_TOL, gap_over, gap_n,
               100.0 * gap_over / gap_n if gap_n else 0.0))
        print('')
        sub('最大橫向偏差         = %.3f m   (逐割草線指派，與上面的覆蓋落差互相印證)'
            % max_dev)
        sub('平均橫向偏差         = %.3f m' % mean_dev)
        sub('  (逐割草線指派，已丟棄 approach 的 %d 筆取樣，'
            % n_drop_sw)
        sub('   只採計投影落在割草線範圍內的 %d 筆；迴轉段不列入)' % n_used)
        sub('每條割草線的最大偏差 = %s' % ['%.3f' % d for d in per_swath])
        sub('  (對照：舊的累積弧長指標算出來是 %.3f m，那個在 0.5 m 線距下會漂到隔壁線，'
            % arc_dev)
        sub('   量到的其實是線距不是循跡誤差，已改用上面的逐割草線指派)')

        # 把 N3 的關鍵數值另存一份 JSON，方便掃描不同跑道長度 L 時做對照表，
        # 不用再從 log 文字裡回頭剖析。
        try:
            import json
            with open(os.path.join(LOGDIR, 'n3_metrics.json'), 'w') as fh:
                json.dump({
                    'lead_in': LEAD_IN,
                    'world': WORLD or 'mow_field.world',
                    'blade_width': BLADE_WIDTH,
                    'overlap_ratio': OVERLAP if OVERLAP is not None else DEFAULT_OVERLAP,
                    'swath_spacing': SWATH_SPACING,
                    'coverage_tol': COVERAGE_TOL,
                    'headland': read_headland_state(ctl),
                    'swaths': st['total'],
                    'completed': st['completed'],
                    'failures': len(st['failures']),
                    'done': st['done'],
                    'elapsed_s': elapsed,
                    'end_err_m': end_err,
                    'gap_max_m': gap_max,
                    'gap_mean_m': gap_mean,
                    'gap_median_m': gap_med,
                    'gap_over_025': gap_over,
                    'gap_n': gap_n,
                    'gap_over_pct': (100.0 * gap_over / gap_n) if gap_n else None,
                    'per_swath_max_dev': per_swath,
                    'moved_m': moved,
                    'path_len_m': total_len,
                }, fh, indent=2)
        except Exception as exc:
            sub('(n3_metrics.json 存檔失敗: %s)' % exc)

        n3_ok = (st['total'] > 0 and not st['failures']
                 and st['completed'] == st['total'] and st['done']
                 and end_err < 0.3)
        if not n3_ok:
            print('')
            print('--- controller_server 完整 log ---')
            print(nav.log_tail(250))
            print('')
            print('--- mower_manager 相關 log ---')
            for line in ctl.log_grep(r'割草線|覆蓋|Nav2|F2C|佇列', 60):
                print('    %s' % line)
        record('N', 'N3', '所有割草線 SUCCEEDED 且完成數 == 總數',
               'PASS' if n3_ok else 'FAIL',
               '切出=%d, 完成=%d, 失敗=%d, 走完=%s, 終點誤差=%.3fm, 最大橫偏=%.3fm'
               % (st['total'], st['completed'], len(st['failures']), st['done'],
                  end_err, max_dev))
        _ = arc_mean

        # ---- N4 remap 驗證 ----
        hdr('N4  驗證 cmd_vel -> cmd_vel_nav remap 有生效 (安全關鍵)')

        def nonzero(seq):
            return [v for v in seq if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9]

        def peak(seq):
            if not seq:
                return (0.0, 0.0)
            return (max(abs(v[0]) for v in seq), max(abs(v[1]) for v in seq))

        nav_nonzero = nonzero(rig.cmd_vel_nav)
        cmd_nonzero = nonzero(rig.cmd_vel)
        sub('(覆蓋任務執行期間，manager 在 mode 1)')
        sub('    /cmd_vel_nav 共 %d 筆，非零 %d 筆，最大 linear.x=%.4f angular.z=%.4f'
            % ((len(rig.cmd_vel_nav), len(nav_nonzero)) + peak(rig.cmd_vel_nav)))
        sub('    /cmd_vel     共 %d 筆，非零 %d 筆，最大 linear.x=%.4f angular.z=%.4f'
            % ((len(rig.cmd_vel), len(cmd_nonzero)) + peak(rig.cmd_vel)))
        part1 = bool(nav_nonzero) and bool(cmd_nonzero)

        sub('(切回 mode 2 手動，由測試自己送一條路徑，Nav2 的輸出不應該到達底盤)')
        ok_mode2 = rig.set_mode(2)
        sub('    change_mower_mode(mode=2) 回傳 success = %s' % ok_mode2)
        rig.spin(1.0)
        cur2 = rig.odom[-1] if rig.odom else (0, x0, y0)
        path2, pts2, total2, _ = build_mow_path(rig, cur2[1], cur2[2])
        del rig.cmd_vel[:]
        del rig.cmd_vel_nav[:]
        gh2, why2 = rig.send_path(path2)
        sub('    直接送給 controller_server 的 goal 是否被接受 = %s (%s)' % (gh2 is not None, why2))
        if gh2 is not None:
            rig.spin(10.0)
            gh2.cancel_goal_async()
            rig.spin(2.0)
        nav2_nonzero = nonzero(rig.cmd_vel_nav)
        cmd2_nonzero = nonzero(rig.cmd_vel)
        sub('    /cmd_vel_nav 共 %d 筆，非零 %d 筆，最大 linear.x=%.4f angular.z=%.4f'
            % ((len(rig.cmd_vel_nav), len(nav2_nonzero)) + peak(rig.cmd_vel_nav)))
        sub('    /cmd_vel     共 %d 筆，非零 %d 筆，最大 linear.x=%.6f angular.z=%.6f'
            % ((len(rig.cmd_vel), len(cmd2_nonzero)) + peak(rig.cmd_vel)))
        part2 = bool(nav2_nonzero) and not cmd2_nonzero
        record('N', 'N4', 'Nav2 輸出走 /cmd_vel_nav 且受 manager 仲裁',
               'PASS' if (part1 and part2) else 'FAIL',
               'mode1: nav非零=%d cmd非零=%d ; mode2: nav非零=%d cmd非零=%d'
               % (len(nav_nonzero), len(cmd_nonzero), len(nav2_nonzero), len(cmd2_nonzero)))

        # ---- N5 RTF ----
        hdr('N5  Nav2 運行下的 Gazebo real-time factor')
        rc, out = run(['gz', 'stats', '-d', '5'], timeout=40)
        factors = [float(x) for x in re.findall(r'Factor\[\s*([\d.]+)\]', out)]
        if not factors:
            record('N', 'N5', 'Nav2 運行下的 RTF', 'FAIL',
                   'gz stats 沒有解析到 Factor (rc=%d)' % rc)
        else:
            avg = sum(factors) / len(factors)
            sub('取樣數 = %d' % len(factors))
            sub('RTF 平均 = %.3f，最小 = %.3f，最大 = %.3f' % (avg, min(factors), max(factors)))
            sub('(對照組：Phase C 無 Nav2 時為 1.000)')
            record('N', 'N5', 'Nav2 運行下 RTF >= 0.5',
                   'PASS' if avg >= 0.5 else 'FAIL',
                   'RTF 平均=%.3f (min=%.3f, max=%.3f)%s'
                   % (avg, min(factors), max(factors),
                      '' if avg >= 0.5 else '  <- 低於 0.5，VM 效能不足'))

        # ---- N6 急停要清空佇列 ----
        hdr('N6  急停要清空割草線佇列 (安全測試)')
        sub('(a) 重新發布同一個空曠邊界並切到 mode 1 開始覆蓋任務')
        rig.publish_boundary(bcx, bcy, half=half, seconds=3.0)
        del rig.odom[:]
        rig.set_mode(1)
        sub('(b) 等第一條割草線走到一半 (5 秒)')
        rig.spin(5.0)
        traj_b = list(rig.odom)
        moved_b = sum(math.dist((traj_b[i][1], traj_b[i][2]),
                                (traj_b[i + 1][1], traj_b[i + 1][2]))
                      for i in range(len(traj_b) - 1)) if len(traj_b) > 1 else 0.0
        st_b = read_mission_state(ctl)
        sub('    這 5 秒車子移動 %.3f m；manager 已切出 %d 條割草線 (含 approach 共 %d 個任務)'
            % (moved_b, st_b['total'],
               st_b['total'] + (1 if st_b['approach'] and st_b['approach'][0] == 'made' else 0)))
        if moved_b < 0.1:
            record('N', 'N6', '急停後解除不會自己動起來', 'SKIP',
                   '覆蓋任務沒有真的啟動 (5 秒只移動 %.3f m)，這項證明不了任何事' % moved_b)
        else:
            sub('(c) 切到 mode 4 急停')
            rig.set_mode(4)
            del rig.cmd_vel[:]
            del rig.odom[:]
            sub('(d) 等 3 秒，確認 /cmd_vel 全為 0')
            rig.spin(3.0)
            estop_cmds = list(rig.cmd_vel)
            estop_nonzero = [v for v in estop_cmds if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9]
            sub('    /cmd_vel 收到 %d 筆，非零 %d 筆' % (len(estop_cmds), len(estop_nonzero)))

            sub('(e) 切回 mode 3，監聽 5 秒，車子不應該自己動起來')
            rig.set_mode(3)
            del rig.cmd_vel[:]
            del rig.odom[:]
            rig.spin(5.0)
            after_cmds = list(rig.cmd_vel)
            after_nonzero = [v for v in after_cmds if abs(v[0]) > 1e-9 or abs(v[1]) > 1e-9]
            traj_e = list(rig.odom)
            moved_e = sum(math.dist((traj_e[i][1], traj_e[i][2]),
                                    (traj_e[i + 1][1], traj_e[i + 1][2]))
                          for i in range(len(traj_e) - 1)) if len(traj_e) > 1 else 0.0
            sub('    /cmd_vel 收到 %d 筆，非零 %d 筆' % (len(after_cmds), len(after_nonzero)))
            sub('    這 5 秒車子移動 %.4f m' % moved_e)
            st_e = read_mission_state(ctl)
            sub('    manager 佇列狀態: 完成 %d/%d，最後有無清空 log = %s'
                % (st_e['completed'], st_e['total'],
                   bool(ctl.log_grep(r'清空覆蓋任務佇列', 5))))
            n6_ok = (not estop_nonzero) and (not after_nonzero)
            record('N', 'N6', '急停清空佇列，解除後不會自己恢復任務',
                   'PASS' if n6_ok else 'FAIL',
                   '(d) 急停 3 秒非零 %d 筆 ; (e) 解除後 5 秒非零 %d 筆、移動 %.4f m'
                   % (len(estop_nonzero), len(after_nonzero), moved_e))
    finally:
        rig.close()


# --------------------------------------------------------------------------
# 6.5 Phase L：存圖與定位模式
# --------------------------------------------------------------------------
class LocRig(object):
    """Phase L 用的測試夾具：偽造手把速度指令、讀 /map 與 /odom、切模式"""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from nav_msgs.msg import OccupancyGrid, Odometry
        from geometry_msgs.msg import Twist
        from mowerbot_interfaces.srv import SetDriveMode

        rclpy.init()
        self.node = Node('smoke_l')
        self.odom = []
        self.maps = []            # (width, height, resolution, origin_x, origin_y)
        # /map 是 latched (TRANSIENT_LOCAL)，用預設 QoS 會收不到已經發布過的那一張
        map_qos = QoSProfile(depth=1,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(OccupancyGrid, '/map', self._map_cb, map_qos)
        self.node.create_subscription(
            Odometry, '/odom',
            lambda m: self.odom.append((m.pose.pose.position.x,
                                        m.pose.pose.position.y)), 20)
        self.joy_vel_pub = self.node.create_publisher(Twist, '/cmd_vel_joy', 10)
        self.mode_cli = self.node.create_client(SetDriveMode, 'change_mower_mode')
        self.Twist = Twist
        self.SetDriveMode = SetDriveMode
        self.closed = False

    def _map_cb(self, msg):
        self.maps.append((msg.info.width, msg.info.height, msg.info.resolution,
                          msg.info.origin.position.x, msg.info.origin.position.y))

    def spin(self, seconds):
        import rclpy
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def set_mode(self, mode):
        import rclpy
        if not self.mode_cli.wait_for_service(timeout_sec=15.0):
            return None
        req = self.SetDriveMode.Request()
        req.mode = mode
        fut = self.mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=10.0)
        return fut.result().success if fut.done() and fut.result() is not None else None

    def drive(self, lin, ang, seconds, rate=20.0):
        """持續對 /cmd_vel_joy 送速度指令。

        必須持續送：mower_manager 的 watchdog 超過 watchdog_timeout
        收不到指令就會自己停車，送一次是開不動的。
        """
        msg = self.Twist()
        msg.linear.x = float(lin)
        msg.angular.z = float(ang)
        t0 = time.time()
        period = 1.0 / rate
        nxt = t0
        while time.time() - t0 < seconds:
            now = time.time()
            if now >= nxt:
                self.joy_vel_pub.publish(msg)
                nxt = now + period
            self.spin(0.01)

    def wait_map(self, timeout=20.0):
        t0 = time.time()
        while time.time() - t0 < timeout and not self.maps:
            self.spin(0.3)
        return self.maps[-1] if self.maps else None

    def close(self):
        import rclpy
        if self.closed:
            return
        self.closed = True
        self.node.destroy_node()
        rclpy.shutdown()


def _pgm_size(path):
    """讀 PGM 檔頭拿 width/height，用來與 /map 對照。

    PGM 檔頭是純文字：magic (P5) / 可選註解行 / width height / maxval，
    之後才是二進位像素資料。
    """
    try:
        with open(path, 'rb') as fh:
            tokens = []
            while len(tokens) < 4:
                line = fh.readline()
                if not line:
                    return None
                line = line.split(b'#', 1)[0]
                tokens.extend(line.split())
            return int(tokens[1]), int(tokens[2])
    except Exception:
        return None


def phase_l():
    hdr('Phase L  存圖與定位模式 (建圖 -> 存檔 -> 切定位模式)')
    sub('現況全程都用 async_slam_toolbox_node 的建圖模式，等於一邊割草一邊改地圖，')
    sub('Nav2 的 costmap 會跟著漂。這個 Phase 驗證「建圖 -> 存檔 -> 切定位模式」走得通。')

    map_name = 'smoke_l_map'
    map_dir = os.path.join(WS, 'src', 'mowerbot_bringup', 'maps')
    target = os.path.join(map_dir, map_name)
    exts = ('posegraph', 'data', 'pgm', 'yaml')

    # 每次重跑都從乾淨狀態開始，否則會拿到上一輪的舊檔案而誤判成功
    for ext in exts:
        try:
            os.remove('%s.%s' % (target, ext))
        except OSError:
            pass

    def skip_rest(reason, start_at):
        allc = [('L1', '建圖模式開車建出地圖'), ('L2', 'serialize_map 與 save_map 存檔'),
                ('L3', '切換到定位模式'), ('L4', 'map -> odom TF 與 /map 穩定性')]
        for cid, name in allc:
            if cid >= start_at:
                record('L', cid, name, 'SKIP', reason)

    gz = bg_start('L_gazebo',
                  gazebo_cmd())
    sub('等待 Gazebo 起來 (18 秒)...')
    time.sleep(18)
    ctl = bg_start('L_mower_control', mower_control_cmd())
    sub('等待 mower_control (含建圖模式 slam_toolbox) 穩定 (20 秒)...')
    time.sleep(20)

    rig = LocRig()
    try:
        # ---- L1 建圖 ----
        hdr('L1  建圖模式下開車走一個小方形，讓 SLAM 建出有內容的地圖')
        ok_mode = rig.set_mode(2)
        sub('change_mower_mode(mode=2 手動) 回傳 success = %s' % ok_mode)
        if ok_mode is None:
            record('L', 'L1', '建圖模式開車建出地圖', 'FAIL',
                   'change_mower_mode 服務沒上線')
            skip_rest('沒辦法切到手動模式，開不了車', 'L2')
            return

        start = rig.odom[-1] if rig.odom else None
        sub('開始開車 (mode 2 下對 /cmd_vel_joy 送指令，共約 30 秒)')
        # 走一個方形：直線 5 秒 + 原地轉 90 度，重複 4 次
        for leg in range(4):
            rig.drive(0.4, 0.0, 5.0)
            rig.drive(0.0, 0.8, 2.0)       # 0.8 rad/s x 2 s = 1.6 rad，約 92 度
            sub('  第 %d 邊走完' % (leg + 1))
        rig.drive(0.0, 0.0, 1.0)
        rig.spin(2.0)

        end = rig.odom[-1] if rig.odom else None
        moved = math.dist(start, end) if (start and end) else float('nan')
        travelled = 0.0
        if len(rig.odom) > 1:
            travelled = sum(math.dist(rig.odom[i], rig.odom[i + 1])
                            for i in range(len(rig.odom) - 1))
        sub('起點 = %s' % ('(%.2f, %.2f)' % start if start else '(沒收到 odom)'))
        sub('終點 = %s' % ('(%.2f, %.2f)' % end if end else '(沒收到 odom)'))
        sub('軌跡總長 = %.2f m，起終點直線距離 = %.2f m' % (travelled, moved))

        m = rig.wait_map(20.0)
        if m is None:
            record('L', 'L1', '建圖模式開車建出地圖', 'FAIL', '20 秒內收不到 /map')
            skip_rest('沒有地圖可存', 'L2')
            return
        map_w, map_h, map_res = m[0], m[1], m[2]
        sub('/map 尺寸 = %d x %d cells，解析度 %.3f m (%.1f x %.1f m)'
            % (map_w, map_h, map_res, map_w * map_res, map_h * map_res))
        sub('/map 收到 %d 次更新' % len(rig.maps))
        l1_ok = travelled > 2.0 and map_w > 0 and map_h > 0
        record('L', 'L1', '建圖模式開車 30 秒建出有內容的地圖',
               'PASS' if l1_ok else 'FAIL',
               '軌跡 %.2f m, /map %d x %d cells' % (travelled, map_w, map_h))

        # ---- L2 存檔 ----
        hdr('L2  呼叫 serialize_map 與 save_map 存檔')
        sub('用一鍵存圖腳本: ros2 run mowerbot_bringup save_map.sh %s' % map_name)
        rc, out = run(['ros2', 'run', 'mowerbot_bringup', 'save_map.sh', map_name],
                      timeout=120)
        print(('     ' + out.strip()).replace('\n', '\n     '))
        sizes = {}
        for ext in exts:
            path = '%s.%s' % (target, ext)
            sizes[ext] = os.path.getsize(path) if os.path.isfile(path) else None
        print('')
        sub('存檔結果 (目錄 %s):' % map_dir)
        for ext in exts:
            if sizes[ext] is None:
                print('        %-10s (沒有產生)' % ('.' + ext))
            else:
                print('        %-10s %10d bytes' % ('.' + ext, sizes[ext]))
        pgm_size = _pgm_size('%s.pgm' % target)
        sub('.pgm 檔頭的 width x height = %s'
            % ('%d x %d' % pgm_size if pgm_size else '(讀不到)'))
        missing = [e for e in exts if not sizes[e]]
        l2_ok = not missing
        record('L', 'L2', 'serialize_map 與 save_map 四個檔案都產生且非空',
               'PASS' if l2_ok else 'FAIL',
               '腳本 rc=%d, 缺少或空檔=%s' % (rc, missing or '無'))
        if not l2_ok:
            skip_rest('地圖沒存成功，定位模式起不來', 'L3')
            return

        # ---- L3 切定位模式 ----
        hdr('L3  關掉建圖模式的節點，改用 localization.launch.py 啟動')
        sub('(Gazebo 留著不關：它提供 /scan 與 odom -> base_footprint，')
        sub(' 這兩個在真實流程裡是車子本身提供的，不屬於建圖模式)')
        rig.close()
        ctl.stop()
        sub('等建圖模式的節點完全退出 (5 秒)...')
        time.sleep(5.0)

        loc = bg_start('L_localization',
                       ['ros2', 'launch', 'mowerbot_bringup', 'localization.launch.py',
                        'map_file_name:=%s' % target])
        sub('等定位模式載入序列化地圖 (20 秒)...')
        time.sleep(20)
        alive = loc.alive()
        sub('localization.launch.py 還活著 = %s' % alive)
        if not alive:
            print('')
            print('--- localization.launch.py 完整 log ---')
            print(loc.log_tail(120))
        record('L', 'L3', 'localization_slam_toolbox_node 啟動且沒有掛掉',
               'PASS' if alive else 'FAIL',
               'launch 行程 alive=%s' % alive)
        if not alive:
            skip_rest('定位模式沒起來', 'L4')
            return

        # ---- L4 TF 與 /map ----
        # 判定標準在階段 7 由使用者放寬。原本要求「/map 尺寸與存檔時完全一致」，
        # 但這對 SLAM 地圖本來就不成立：地圖尺寸取決於 pose graph 的涵蓋範圍，
        # 實測建圖端自己就會在 402x401 / 402x402 之間跳。1 格 = 5 公分，
        # 功能上沒有差別。真正要驗的「地圖不漂」是「尺寸會不會持續變動」。
        MAP_SIZE_TOL = 5            # 格，1 格 = 5 cm
        TF_TRANS_TOL = 0.5          # m
        MAP_STABLE_SEC = 20.0
        hdr('L4  確認 map -> odom TF 平移 < %.1f m、/map 有發布、'
            '尺寸 %.0f 秒內不變且與存檔時差異 <= %d 格'
            % (TF_TRANS_TOL, MAP_STABLE_SEC, MAP_SIZE_TOL))
        trans, tf_out = tf_echo('map', 'odom', dur=8)
        if trans is None:
            sub('tf2_echo map -> odom 查不到，原始輸出:')
            print(('     ' + tf_out.strip()[-800:]).replace('\n', '\n     '))
            tf_norm = float('nan')
        else:
            tf_norm = math.sqrt(sum(v * v for v in trans))
            sub('map -> odom 平移 = (%.3f, %.3f, %.3f)，模長 %.4f m  (門檻 < %.1f)'
                % (trans + (tf_norm, TF_TRANS_TOL)))
        tf_ok = (trans is not None) and tf_norm < TF_TRANS_TOL

        rig2 = LocRig()
        m2 = rig2.wait_map(25.0)
        stable = False
        if m2 is None:
            sub('/map 在定位模式下 25 秒內沒有收到')
            loc_w = loc_h = None
        else:
            loc_w, loc_h, loc_res = m2[0], m2[1], m2[2]
            sub('定位模式 /map 尺寸 = %d x %d cells，解析度 %.3f m'
                % (loc_w, loc_h, loc_res))
            # 階段 3 的目的就是「地圖不要再漂」，所以再等一段時間看尺寸會不會變。
            # 這是 L4 的判定項之一。
            del rig2.maps[:]
            rig2.spin(MAP_STABLE_SEC)
            if rig2.maps:
                seen = sorted({(mm[0], mm[1]) for mm in rig2.maps} | {(loc_w, loc_h)})
                stable = len(seen) == 1
                sub('再監聽 %.0f 秒收到 %d 張 /map，尺寸集合 = %s  -> %s'
                    % (MAP_STABLE_SEC, len(rig2.maps), ['%dx%d' % z for z in seen],
                       '固定不變' if stable else '仍然會變動'))
            else:
                stable = True
                sub('再監聽 %.0f 秒沒有收到新的 /map (latched 之後就不再更新) -> 視為不變'
                    % MAP_STABLE_SEC)
        rig2.close()

        print('')
        sub('存檔時 /map          = %d x %d cells' % (map_w, map_h))
        sub('.pgm 檔頭            = %s'
            % ('%d x %d' % pgm_size if pgm_size else '(讀不到)'))
        sub('定位模式 /map        = %s'
            % ('%d x %d' % (loc_w, loc_h) if loc_w else '(沒收到)'))
        if loc_w is None:
            dw = dh = None
            close_enough = False
        else:
            dw, dh = abs(loc_w - map_w), abs(loc_h - map_h)
            close_enough = dw <= MAP_SIZE_TOL and dh <= MAP_SIZE_TOL
        sub('與存檔時的尺寸差      = %s 格  (門檻 <= %d 格 = %.2f m)'
            % ('%d x %d' % (dw, dh) if dw is not None else '(沒收到)',
               MAP_SIZE_TOL, MAP_SIZE_TOL * 0.05))
        sub('尺寸 %.0f 秒內不變     = %s' % (MAP_STABLE_SEC, stable))
        l4_ok = tf_ok and (loc_w is not None) and stable and close_enough
        record('L', 'L4',
               'map->odom TF 平移 < %.1f m、/map 有發布、尺寸 %.0f 秒不變且與存檔差 <= %d 格'
               % (TF_TRANS_TOL, MAP_STABLE_SEC, MAP_SIZE_TOL),
               'PASS' if l4_ok else 'FAIL',
               'TF=%s(%.4fm), 存檔時=%sx%s, 定位模式=%s, 差=%s格, %.0f秒內不變=%s'
               % ('有' if trans is not None else '無', tf_norm, map_w, map_h,
                  '%dx%d' % (loc_w, loc_h) if loc_w else '沒收到',
                  '%dx%d' % (dw, dh) if dw is not None else '-',
                  MAP_STABLE_SEC, stable))
    finally:
        try:
            rig.close()
        except Exception:
            pass


# --------------------------------------------------------------------------
# 7. 總結
# --------------------------------------------------------------------------
def summary():
    hdr('總結報告')
    total_pass = 0
    total_all = 0
    print('')
    for pid, pname in PHASES:
        items = [r for r in RESULTS if r[0] == pid]
        if not items:
            continue
        npass = sum(1 for r in items if r[3] == 'PASS')
        nskip = sum(1 for r in items if r[3] == 'SKIP')
        bad = [r[1] for r in items if r[3] == 'FAIL']
        skipped = [r[1] for r in items if r[3] == 'SKIP']
        status = 'PASS' if not bad and not skipped else ('FAIL' if bad else 'SKIP')
        note = ''
        if bad:
            note = ' (%s 失敗)' % ', '.join(bad)
        elif skipped:
            note = ' (%s 略過)' % ', '.join(skipped)
        print('  Phase %s  %-10s %d/%d  %s%s'
              % (pid, pname, npass, len(items), status, note))
        total_pass += npass
        total_all += len(items)
        _ = nskip
    print('')
    print('  總計     %d/%d' % (total_pass, total_all))

    fails = [r for r in RESULTS if r[3] in ('FAIL', 'SKIP')]
    if fails:
        print('')
        print('-' * 78)
        print('失敗 / 略過項目明細')
        print('-' * 78)
        for phase, cid, name, status, detail in fails:
            print('')
            print('  [%s] %s  %s' % (status, cid, name))
            print('        實際觀察: %s' % (detail or '(無)'))
    print('')
    print('  完整 log 目錄: %s' % LOGDIR)
    print('')
    return 0 if all(r[3] == 'PASS' for r in RESULTS) else 1


def main():
    phases = 'ABCDLN'
    for arg in sys.argv[1:]:
        if arg.startswith('--phases'):
            phases = arg.split('=', 1)[1] if '=' in arg else 'ABCDLN'
        elif arg.startswith(('--lead-in', '--overlap', '--world')):
            pass          # 已在模組載入時解析成 LEAD_IN / OVERLAP / WORLD
        elif arg in ('-h', '--help'):
            print(__doc__)
            return 0

    print('mowerbot 冒煙測試   %s' % datetime.now().strftime('%Y-%m-%d %H:%M:%S'))
    print('workspace     : %s' % WS)
    print('ROS_DOMAIN_ID : %s' % ENV['ROS_DOMAIN_ID'])
    print('log 目錄      : %s' % LOGDIR)
    print('要跑的 Phase  : %s' % ' '.join(phases))
    print('跑道長度 L    : %s'
          % ('%.2f m (覆寫)' % LEAD_IN if LEAD_IN is not None
             else '(用 launch 預設值)'))
    print('重疊率        : %s  -> 割草線間距 %.3f m (刀盤寬 %.2f m)'
          % ('%.2f (覆寫)' % OVERLAP if OVERLAP is not None
             else '%.2f (launch 預設值)' % DEFAULT_OVERLAP,
             SWATH_SPACING, BLADE_WIDTH))
    print('模擬世界      : %s' % (WORLD or 'mow_field.world (launch 預設值)'))
    print('')
    preflight_domain_clean()

    try:
        if 'A' in phases:
            hdr('Phase A  靜態檢查')
            a1_build()
            a2_show_args()
            a3_executables()
            a4_interfaces()
            a5_xacro()

        if 'B' in phases:
            hdr('Phase B  單元功能測試')
            try:
                b1_f2c()
            finally:
                bg_stop_all()
            try:
                b2_boundary()
            finally:
                bg_stop_all()

        if 'C' in phases:
            try:
                phase_c()
            finally:
                bg_stop_all()
                sweep(['gzserver', 'gzclient', 'spawn_entity.py'])

        if 'D' in phases:
            try:
                phase_d()
            finally:
                bg_stop_all()

        if 'L' in phases:
            try:
                phase_l()
            finally:
                bg_stop_all()
                sweep(['gzserver', 'gzclient', 'spawn_entity.py'])

        if 'N' in phases:
            try:
                phase_n()
            finally:
                bg_stop_all()
                sweep(['gzserver', 'gzclient', 'spawn_entity.py'])
    except KeyboardInterrupt:
        print('\n[中斷] 使用者按下 Ctrl-C，正在收拾背景行程...')
    finally:
        bg_stop_all()
        sweep(['gzserver', 'gzclient'])

    return summary()


if __name__ == '__main__':
    sys.exit(main())
