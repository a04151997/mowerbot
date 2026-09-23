#!/usr/bin/env python3
"""mowerbot 人機介面 (HMI)。

一鍵啟動之後跳出來，用來切換模式與觀察系統狀態。

=========================================================================
【安全聲明】這個 GUI 的緊急停止按鈕是「操作便利性」功能，不是主要安全裝置。
=========================================================================
它依賴三件事，而這三件事都無法保證：

  1. 跑這個介面的筆電沒有當機
  2. 與機器人之間的網路 / DDS 連線正常
  3. change_mower_mode 服務呼叫真的送達並成功

74 kg 的載具**必須**有實體急停按鈕在機器本體上，那才是主要安全裝置。
GUI 急停、手把急停（deadman）、機身實體急停是三層，缺一不可。
這個介面的「與系統失去聯繫」指示就是為了讓操作者知道
「你現在看到的不是即時狀態，按鈕也可能沒有用」。

=========================================================================
【刻意不做】GUI 上沒有手動駕駛（前進 / 後退 / 轉向）按鈕。
=========================================================================
手動駕駛必須有 deadman（放開就停）。滑鼠點擊無法提供「持續按住」的語意 ——
按下去之後如果滑鼠飄走、視窗失焦、或程式卡住，車子會繼續動。
對 74 kg 的載具這是不可接受的風險。
手動駕駛維持只能透過實體手把（teleop_node 的 LB deadman）。

=========================================================================
【執行緒】不另開執行緒跑 rclpy.spin。
=========================================================================
用 QTimer 每 50 ms 呼叫 rclpy.spin_once(node, timeout_sec=0)，
讓 ROS 的回呼在 Qt 的主執行緒裡執行。
這樣 ROS 回呼與 Qt widget 的更新天生就在同一個執行緒，
不需要 Qt signal 跨執行緒 marshal，也就少掉一整類的競態問題
（「回呼在背景執行緒直接改 widget」是 PyQt 最常見的當機原因）。
代價是 ROS 回呼的延遲上限是 50 ms，對一個給人看的介面完全足夠。
"""

import os
import sys

import rclpy
from rclpy.node import Node

from PyQt5 import QtCore, QtGui, QtWidgets

from geometry_msgs.msg import PolygonStamped
from mowerbot_interfaces.msg import MowerStatus, MissionStatus, JoyStatus
from mowerbot_interfaces.srv import SetDriveMode

# mode 3 留在表裡但沒有按鈕：它是保留值、沒有實作，介面上不提供給使用者。
# 還是要有名字，因為如果有東西（例如手把、或直接呼叫服務）把模式設成 3，
# 畫面必須誠實顯示它現在是什麼狀態，而不是「未知」。
MODE_NAMES = {
    0: '建圖模式',
    1: '自動割草 (F2C)',
    2: '手動模式',
    3: '保留（未實作）',
    4: '緊急停止',
}

# 畫面上提供給使用者的模式按鈕。
# 不含 3：自動導航需要 planner_server + bt_navigator，不在本專題的宣告範圍內。
# 不含 4：急停在最上面那顆大按鈕，重複放一顆小的只會讓人在急的時候按錯。
SELECTABLE_MODES = (0, 1, 2)

MISSION_STATE_NAMES = {
    MissionStatus.STATE_IDLE: '待命',
    MissionStatus.STATE_PLANNING: '規劃中',
    MissionStatus.STATE_EXECUTING: '執行中',
    MissionStatus.STATE_DONE: '已完成',
    MissionStatus.STATE_ABORTED: '已中止',
}

# 超過這個秒數沒收到 /mower_status 就視為失去聯繫
CONNECTION_TIMEOUT = 1.0

# 超過這個秒數沒收到 /joy_status 就視為「手把狀態不明」-> 一律顯示未連線。
# 這與上面那個 manager 的連線判斷是**兩件獨立的事**：
# teleop 掛掉時 manager 可能還活著，畫面不能停在最後一筆綠色狀態。
JOY_STATUS_TIMEOUT = 1.0

# 與 mower_manager 的 min_boundary_area 預設值一致。
# 介面上只是拿來提示「這個邊界會被拒絕」，真正的判定在 manager。
BOUNDARY_MIN_AREA_HINT = 4.0


class HmiBackend(Node):
    """ROS 那一側：訂閱狀態、呼叫服務。不碰任何 Qt widget。"""

    def __init__(self):
        super().__init__('mower_hmi')
        self.mower_status = None
        self.mission_status = None
        self.last_status_time = None        # 單調時鐘，用來判斷有沒有失去聯繫
        self.boundary = None
        self.boundary_time = None
        self.joy_status = None
        self.joy_status_time = None

        self.create_subscription(MowerStatus, '/mower_status', self._on_mower, 10)
        self.create_subscription(MissionStatus, '/mission_status', self._on_mission, 10)
        self.create_subscription(PolygonStamped, '/f2c_boundary', self._on_boundary, 10)
        self.create_subscription(JoyStatus, '/joy_status', self._on_joy_status, 10)

        self.mode_cli = self.create_client(SetDriveMode, 'change_mower_mode')

        # 存圖用的兩個 slam_toolbox 服務。型別是選用相依：
        # slam_toolbox 沒裝或沒跑的時候，介面其他部分仍然要能用。
        self.serialize_cli = None
        self.savemap_cli = None
        self.SerializePoseGraph = None
        self.SaveMap = None
        try:
            from slam_toolbox.srv import SerializePoseGraph, SaveMap
            self.SerializePoseGraph = SerializePoseGraph
            self.SaveMap = SaveMap
            self.serialize_cli = self.create_client(
                SerializePoseGraph, '/slam_toolbox/serialize_map')
            self.savemap_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        except ImportError:
            self.get_logger().warn(
                '找不到 slam_toolbox 的服務型別，存圖按鈕會停用')

    # ---- 回呼 --------------------------------------------------------
    def _on_mower(self, msg):
        self.mower_status = msg
        self.last_status_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_mission(self, msg):
        self.mission_status = msg

    def _on_boundary(self, msg):
        self.boundary = msg
        self.boundary_time = self.get_clock().now().nanoseconds * 1e-9

    def _on_joy_status(self, msg):
        self.joy_status = msg
        self.joy_status_time = self.get_clock().now().nanoseconds * 1e-9

    def joy_view(self):
        """回傳 (connected, deadman_held, 說明文字) 給畫面用。

        失效方向一律偏向「未連線」：
          - 從來沒收到 /joy_status (teleop 沒跑) -> 未連線
          - /joy_status 自己超過 1 秒沒更新 (teleop 掛了) -> 未連線
          - 收到的訊息說 connected=False -> 未連線
        任何不確定的情況都不會顯示綠色。
        """
        if self.joy_status is None or self.joy_status_time is None:
            return (False, False, '沒有收到 /joy_status（teleop 沒在跑？）')
        age = self.get_clock().now().nanoseconds * 1e-9 - self.joy_status_time
        if age > JOY_STATUS_TIMEOUT:
            return (False, False, '/joy_status 已 %.1f 秒沒更新（teleop 可能掛了）' % age)
        if not self.joy_status.connected:
            return (False, False,
                    '最後一筆 /joy 是 %.1f 秒前' % self.joy_status.last_msg_age)
        return (True, bool(self.joy_status.deadman_held), '')

    # ---- 查詢 --------------------------------------------------------
    def seconds_since_status(self):
        if self.last_status_time is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        return now - self.last_status_time

    def boundary_info(self):
        """回傳 (頂點數, 面積 m²) 或 None"""
        if self.boundary is None:
            return None
        pts = self.boundary.polygon.points
        n = len(pts)
        if n < 3:
            return (n, 0.0)
        acc = 0.0
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            acc += a.x * b.y - b.x * a.y
        return (n, abs(acc) * 0.5)

    # ---- 服務 --------------------------------------------------------
    def request_mode(self, mode):
        if not self.mode_cli.service_is_ready():
            return None
        req = SetDriveMode.Request()
        req.mode = int(mode)
        return self.mode_cli.call_async(req)

    def request_save_map(self, path):
        """同時呼叫 serialize_map 與 save_map，兩個都要成功才算存好。

        定位模式讀的是位姿圖 (.posegraph/.data)，Nav2 的靜態圖層讀的是
        佔據網格 (.pgm/.yaml)，少存一種就會有一邊起不來 ——
        與 save_map.sh 的理由相同。
        """
        if self.serialize_cli is None or self.savemap_cli is None:
            return None, None
        from std_msgs.msg import String
        s_req = self.SerializePoseGraph.Request()
        s_req.filename = path
        m_req = self.SaveMap.Request()
        m_req.name = String(data=path)
        return (self.serialize_cli.call_async(s_req),
                self.savemap_cli.call_async(m_req))


class MainWindow(QtWidgets.QWidget):

    def __init__(self, backend):
        super().__init__()
        self.backend = backend
        self.setWindowTitle('mowerbot 控制介面')
        self.resize(560, 780)
        self._mode_future = None
        self._mode_requested = None
        self._save_futures = None
        self._save_path = None

        root = QtWidgets.QVBoxLayout(self)
        root.setSpacing(10)

        # ---- 緊急停止 ------------------------------------------------
        self.estop_btn = QtWidgets.QPushButton('緊急停止')
        self.estop_btn.setMinimumHeight(110)
        self.estop_btn.setStyleSheet(
            'QPushButton { background-color: #c0392b; color: white;'
            ' font-size: 34px; font-weight: bold; border-radius: 8px; }'
            'QPushButton:pressed { background-color: #7f231a; }')
        # 【刻意不加確認對話框】確認框會讓急停失去意義：
        # 真的需要急停的那一秒，沒有人有空再點一次「確定」。
        self.estop_btn.clicked.connect(lambda: self.on_mode_clicked(4))
        root.addWidget(self.estop_btn)

        self.conn_label = QtWidgets.QLabel('')
        self.conn_label.setAlignment(QtCore.Qt.AlignCenter)
        self.conn_label.setStyleSheet('font-size: 18px; font-weight: bold;')
        root.addWidget(self.conn_label)

        # ---- 手把狀態 -------------------------------------------------
        # 刻意不放進下面的「系統狀態」群組：manager 斷線時那一區會整個變灰，
        # 但手把是不是連著是另一回事，那個訊息在斷線時反而更需要看得到。
        self.joy_label = QtWidgets.QLabel('')
        self.joy_label.setAlignment(QtCore.Qt.AlignCenter)
        self.joy_label.setWordWrap(True)
        root.addWidget(self.joy_label)

        # ---- 模式 ----------------------------------------------------
        mode_box = QtWidgets.QGroupBox('模式')
        mode_layout = QtWidgets.QVBoxLayout(mode_box)
        self.mode_buttons = {}
        for mode in SELECTABLE_MODES:
            btn = QtWidgets.QPushButton('%d  %s' % (mode, MODE_NAMES[mode]))
            btn.setMinimumHeight(46)
            btn.setStyleSheet('font-size: 17px;')
            btn.clicked.connect(lambda _checked, m=mode: self.on_mode_clicked(m))
            mode_layout.addWidget(btn)
            self.mode_buttons[mode] = btn
        self.mode_result = QtWidgets.QLabel('（尚未切換過模式）')
        self.mode_result.setStyleSheet('color: #555;')
        self.mode_result.setWordWrap(True)
        mode_layout.addWidget(self.mode_result)
        root.addWidget(mode_box)

        # ---- 狀態 ----------------------------------------------------
        self.status_box = QtWidgets.QGroupBox('系統狀態')
        grid = QtWidgets.QGridLayout(self.status_box)
        self.lbl_mode = QtWidgets.QLabel('—')
        self.lbl_mission = QtWidgets.QLabel('—')
        self.lbl_progress = QtWidgets.QLabel('—')
        self.lbl_current = QtWidgets.QLabel('—')
        for row, (name, widget) in enumerate([
                ('目前模式', self.lbl_mode),
                ('任務狀態', self.lbl_mission),
                ('進度', self.lbl_progress),
                ('目前段落', self.lbl_current)]):
            grid.addWidget(QtWidgets.QLabel(name + '：'), row, 0)
            widget.setStyleSheet('font-size: 16px; font-weight: bold;')
            grid.addWidget(widget, row, 1)
        # 任務訊息：目前唯一的來源是「拒絕開始」(淨空不足以原地掉頭)。
        # 只有 state 的「已中止」不夠 —— 使用者需要知道該做什麼才能繼續。
        # 佔滿整行並允許換行，因為這則訊息會很長。
        self.lbl_mission_msg = QtWidgets.QLabel('')
        self.lbl_mission_msg.setWordWrap(True)
        self.lbl_mission_msg.setVisible(False)
        grid.addWidget(self.lbl_mission_msg, 7, 0, 1, 2)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setMinimumHeight(24)
        grid.addWidget(self.progress, 4, 0, 1, 2)
        grid.addWidget(QtWidgets.QLabel('跳過的段落：'), 5, 0, 1, 2)
        self.skipped_list = QtWidgets.QListWidget()
        self.skipped_list.setMaximumHeight(110)
        grid.addWidget(self.skipped_list, 6, 0, 1, 2)
        root.addWidget(self.status_box)

        # ---- 工具 ----------------------------------------------------
        tool_box = QtWidgets.QGroupBox('工具')
        tool_layout = QtWidgets.QVBoxLayout(tool_box)

        name_row = QtWidgets.QHBoxLayout()
        name_row.addWidget(QtWidgets.QLabel('地圖名稱：'))
        self.map_name = QtWidgets.QLineEdit('mowerbot_map')
        name_row.addWidget(self.map_name)
        tool_layout.addLayout(name_row)

        self.save_btn = QtWidgets.QPushButton('儲存地圖（位姿圖 + 佔據網格）')
        self.save_btn.setMinimumHeight(40)
        self.save_btn.clicked.connect(self.on_save_map)
        tool_layout.addWidget(self.save_btn)
        self.save_result = QtWidgets.QLabel('（尚未存圖）')
        self.save_result.setWordWrap(True)
        self.save_result.setStyleSheet('color: #555;')
        tool_layout.addWidget(self.save_result)

        tool_layout.addWidget(self._hline())
        self.lbl_boundary = QtWidgets.QLabel('邊界：尚未收到 /f2c_boundary')
        self.lbl_boundary.setWordWrap(True)
        tool_layout.addWidget(self.lbl_boundary)
        root.addWidget(tool_box)

        root.addStretch(1)
        note = QtWidgets.QLabel(
            '⚠ 這個介面的急停是便利功能，不是主要安全裝置。'
            '機身上的實體急停才是。手動駕駛請用實體手把（按住 LB）。')
        note.setWordWrap(True)
        note.setStyleSheet('color: #7f231a; font-size: 12px;')
        root.addWidget(note)

        # ---- ROS 的心跳 ---------------------------------------------
        # 見檔頭【執行緒】：不另開執行緒，ROS 回呼跑在 Qt 主執行緒裡。
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(50)

    @staticmethod
    def _hline():
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        return line

    # ------------------------------------------------------------------
    def on_mode_clicked(self, mode):
        fut = self.backend.request_mode(mode)
        if fut is None:
            self.mode_result.setText(
                '❌ change_mower_mode 服務沒有上線，指令沒有送出去')
            return
        self._mode_future = fut
        self._mode_requested = mode
        self.mode_result.setText('切換到 %s… 等待回應' % MODE_NAMES[mode])

    def on_save_map(self):
        name = self.map_name.text().strip() or 'mowerbot_map'
        try:
            from ament_index_python.packages import get_package_share_directory
            map_dir = os.path.join(
                get_package_share_directory('mowerbot_bringup'), 'maps')
        except Exception:
            map_dir = os.getcwd()
        os.makedirs(map_dir, exist_ok=True)
        path = os.path.join(map_dir, name)
        futs = self.backend.request_save_map(path)
        if futs == (None, None):
            self.save_result.setText('❌ slam_toolbox 的存圖服務不可用')
            return
        self._save_futures = futs
        self._save_path = path
        self.save_result.setText('存圖中… %s' % path)

    # ------------------------------------------------------------------
    def tick(self):
        rclpy.spin_once(self.backend, timeout_sec=0)
        self._poll_futures()
        self._refresh()

    def _poll_futures(self):
        if self._mode_future is not None and self._mode_future.done():
            res = self._mode_future.result()
            ok = bool(res.success) if res is not None else False
            self.mode_result.setText(
                '%s 切換到 %s：服務回傳 success=%s'
                % ('✅' if ok else '❌', MODE_NAMES[self._mode_requested], ok))
            self._mode_future = None

        if self._save_futures is not None:
            s_fut, m_fut = self._save_futures
            if s_fut.done() and m_fut.done():
                s_res, m_res = s_fut.result(), m_fut.result()
                s_ok = (s_res is not None and s_res.result == 0)
                m_ok = (m_res is not None and m_res.result == 0)
                lines = [
                    '%s 位姿圖 serialize_map -> %s.posegraph / .data'
                    % ('✅' if s_ok else '❌', self._save_path),
                    '%s 佔據網格 save_map -> %s.pgm / .yaml'
                    % ('✅' if m_ok else '❌', self._save_path),
                ]
                if not (s_ok and m_ok):
                    lines.append('兩種都要存成功才算存好：'
                                 '定位模式讀位姿圖、Nav2 讀佔據網格。')
                self.save_result.setText('\n'.join(lines))
                self._save_futures = None

    def _refresh(self):
        # 手把狀態要先更新：下面 manager 斷線時會提早 return，
        # 放在後面的話手把那一列會停在舊資料 —— 那正是這個功能要避免的事。
        self._refresh_joy()

        age = self.backend.seconds_since_status()
        connected = (age is not None and age <= CONNECTION_TIMEOUT)

        # ---- 連線指示（安全相關）----
        # 斷線時整個狀態區變灰、模式按鈕停用，
        # 絕對不能讓操作者把舊畫面當成即時狀態。
        self.status_box.setEnabled(connected)
        for btn in self.mode_buttons.values():
            btn.setEnabled(connected)
        self.estop_btn.setEnabled(connected)
        if not connected:
            waited = '從未收到' if age is None else '已 %.1f 秒沒有更新' % age
            self.conn_label.setText('⚠ 與系統失去聯繫（%s）' % waited)
            self.conn_label.setStyleSheet(
                'font-size: 18px; font-weight: bold; color: white;'
                ' background-color: #c0392b; padding: 6px; border-radius: 6px;')
            self.estop_btn.setText('緊急停止（連線中斷，按鈕可能無效）')
            return

        self.conn_label.setText('● 已連線（%.2f 秒前更新）' % age)
        self.conn_label.setStyleSheet(
            'font-size: 15px; font-weight: bold; color: #1e8449;')

        st = self.backend.mower_status
        mode = st.mode if st else None
        stop_active = bool(st.stop_active) if st else False

        if stop_active:
            self.estop_btn.setText('急停中 — 按下方模式鍵解除')
            self.estop_btn.setStyleSheet(
                'QPushButton { background-color: #7f231a; color: #ffd6d0;'
                ' font-size: 26px; font-weight: bold; border-radius: 8px;'
                ' border: 4px solid #ffd6d0; }')
        else:
            self.estop_btn.setText('緊急停止')
            self.estop_btn.setStyleSheet(
                'QPushButton { background-color: #c0392b; color: white;'
                ' font-size: 34px; font-weight: bold; border-radius: 8px; }'
                'QPushButton:pressed { background-color: #7f231a; }')

        for m, btn in self.mode_buttons.items():
            if m == mode:
                btn.setStyleSheet(
                    'font-size: 17px; font-weight: bold;'
                    ' background-color: #1e8449; color: white;')
            else:
                btn.setStyleSheet('font-size: 17px;')

        self.lbl_mode.setText(
            '%s（mode %s）' % (MODE_NAMES.get(mode, '未知'), mode))

        ms = self.backend.mission_status
        if ms is None:
            self.lbl_mission.setText('—')
            self.lbl_progress.setText('—')
            self.lbl_current.setText('—')
            self.progress.setValue(0)
            self.skipped_list.clear()
            self.lbl_mission_msg.setVisible(False)
        else:
            self.lbl_mission.setText(
                MISSION_STATE_NAMES.get(ms.state, '未知 (%d)' % ms.state))
            done = ms.completed_segments + ms.skipped_segments
            self.lbl_progress.setText(
                '完成 %d / 共 %d 段，已跳過 %d 段'
                % (ms.completed_segments, ms.total_segments, ms.skipped_segments))
            self.lbl_current.setText(ms.current_label or '—')
            self.progress.setMaximum(max(1, ms.total_segments))
            self.progress.setValue(min(done, max(1, ms.total_segments)))
            self.skipped_list.clear()
            for label in ms.skipped_labels:
                self.skipped_list.addItem(label)
            message = getattr(ms, 'message', '')
            self.lbl_mission_msg.setText(message)
            self.lbl_mission_msg.setStyleSheet(
                'font-size: 15px; font-weight: bold; color: #b00020;'
                if ms.state == MissionStatus.STATE_ABORTED
                else 'font-size: 15px;')
            self.lbl_mission_msg.setVisible(bool(message))

        info = self.backend.boundary_info()
        if info is None:
            self.lbl_boundary.setText(
                '邊界：尚未收到 /f2c_boundary —— '
                '切自動割草之前要先在建圖模式下把場地繞完')
            self.lbl_boundary.setStyleSheet('color: #b9770e;')
        else:
            n, area = info
            too_small = area < BOUNDARY_MIN_AREA_HINT
            self.lbl_boundary.setText(
                '邊界：面積 %.2f m²、%d 個頂點%s'
                % (area, n,
                   '\n⚠ 小於 %.1f m²，manager 會拒絕這個邊界（地圖還沒建夠大）'
                   % BOUNDARY_MIN_AREA_HINT if too_small else ''))
            self.lbl_boundary.setStyleSheet(
                'color: #c0392b; font-weight: bold;' if too_small else 'color: #1e8449;')

    def _refresh_joy(self):
        joy_ok, deadman, note = self.backend.joy_view()
        if joy_ok:
            self.joy_label.setText(
                '● 手把已連線　—　安全鈕 %s'
                % ('按住中' if deadman else '未按住'))
            self.joy_label.setStyleSheet(
                'font-size: 16px; font-weight: bold; color: white;'
                ' background-color: #1e8449; padding: 5px; border-radius: 6px;')
        else:
            # 紅色而不是灰色：手把不在線上代表「現在沒有人能用實體按鈕停下它」，
            # 那是要讓人一眼看到的事。
            self.joy_label.setText('● 手把未連線　%s' % note)
            self.joy_label.setStyleSheet(
                'font-size: 16px; font-weight: bold; color: white;'
                ' background-color: #c0392b; padding: 5px; border-radius: 6px;')

    def closeEvent(self, event):
        self.timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    backend = HmiBackend()
    app = QtWidgets.QApplication(sys.argv)
    app.setFont(QtGui.QFont('Noto Sans CJK TC', 11))
    win = MainWindow(backend)
    win.show()
    try:
        code = app.exec_()
    finally:
        backend.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
