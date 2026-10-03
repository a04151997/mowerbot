import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, OpaqueFunction,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mowerbot_description import vehicle_geometry


def _geometry_gate_failed(_context):
    # 丟例外讓 ros2 launch 以非零結束碼收掉整個 launch (只送 Shutdown 事件的話結束碼是 0，
    # 腳本會以為啟動成功)。原因 geometry_guard 已經印在上面。
    raise RuntimeError('幾何閘門 (geometry_guard) 不通過，launch 中止。原因見上方 geometry_guard 的 FATAL 訊息。')


def generate_launch_description():
    """實車啟動檔：對應 mower_control.launch.py，但把模擬的部分換成真實底盤。

    與模擬版的三個差別：
      1. 不啟動 Gazebo。
      2. 啟動 bridge_node 取代 Gazebo 的 diff_drive plugin ——
         實車上 /odom 與 odom -> base_footprint 的 TF 只有它會發，
         而 slam_toolbox 硬性要求這個 TF。
      3. use_sim_time 預設 false，start_rsp 預設 true
         (模擬時 robot_state_publisher 是 gazebo.launch.py 起的，實車沒有那一支)。

    【還沒有接雷達】實體雷達的驅動節點在下面留了註解佔位。
    不先填是因為現在還不知道會用哪一顆雷達，猜一個套件名稱寫進去，
    之後只會變成「看起來已經處理好、實際上是錯的」。
    """
    pkg_bringup = get_package_share_directory('mowerbot_bringup')

    # 車輛幾何的單一來源 (階段 30；階段 38 起一律經過 vehicle_geometry.load())
    geom = vehicle_geometry.load()

    use_sim_time = LaunchConfiguration('use_sim_time')
    ticks_per_rev = LaunchConfiguration('encoder_ticks_per_rev')
    driver_type = LaunchConfiguration('driver_type')
    serial_port = LaunchConfiguration('serial_port')
    max_vel_scale = LaunchConfiguration('max_vel_scale')
    allow_provisional = LaunchConfiguration('allow_provisional')
    override_reason = LaunchConfiguration('provisional_override_reason')

    # ---- 0. 幾何閘門 (階段 38) -------------------------------------------
    # 整個 launch 的第一個節點。vehicle.yaml 還有 provisional 值而 allow_provisional=false
    # (實車預設) 時以 exit code 1 結束，下面的 OnProcessExit 就讓整個 launch 關掉；
    # 通過 (exit code 0) 才啟動其餘節點。
    geometry_guard = Node(
        package='mowerbot_description',
        executable='geometry_guard',
        name='geometry_guard',
        output='screen',
        parameters=[{
            'allow_provisional': ParameterValue(allow_provisional, value_type=bool),
            'provisional_override_reason': ParameterValue(override_reason, value_type=str),
        }]
    )

    # ---- 1. 底盤橋接 ----------------------------------------------------
    # 這是實車與模擬唯一真正不同的節點。
    # 驅動板的序列埠固定為 /dev/mowerbot_base（udev 規則：deploy/99-mowerbot.rules）。
    # 輪趣 C30D 用 driver_type:=wheeltec（drivers/wheeltec.py，階段 36，
    # 只對著 test/tools/fake_c30d.py 驗證過，協定未經實機確認）。
    bridge_node = Node(
        package='mowerbot_bridge',
        executable='bridge_node',
        name='mower_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 車體物理量：來自 vehicle.yaml，與 URDF 同一個來源，不要各填各的。
            # 驅動輪是後輪 (前輪是腳輪，不驅動)。
            'wheel_radius': geom.rear_wheel_radius,
            'wheel_separation': geom.wheel_separation,
            # 第二道幾何閘門：bridge_node 以 read_only=false (會送馬達指令) 啟動時再查一次
            'allow_provisional': ParameterValue(allow_provisional, value_type=bool),
            'provisional_override_reason': ParameterValue(override_reason, value_type=str),
            # 【必填】每轉的編碼器 tick 數，要查驅動板/編碼器的文件。
            # 預設 0 代表「還沒填」，bridge_node 會拒絕啟動並印出說明 ——
            # 這是刻意的：猜一個值會讓里程計的尺度安靜地錯掉。
            'encoder_ticks_per_rev': ticks_per_rev,
            # 校正係數：跑 test/tools/calibrate_odometry.py 量出來之後填這裡。
            # separation_correction 對四輪 skid-steer 特別重要，
            # 通常落在 1.3 ~ 1.8，維持 1.0 等於沒校正。
            'wheel_radius_correction': 1.0,
            'wheel_separation_correction': 1.0,
            # 方向接反時改這兩個，不要去改接線
            'invert_left': False,
            'invert_right': False,
            # 編碼器計數器的模數 (16 位元填 65536)，0 = 不會回繞。
            # 要查板子文件 (drivers/README.md 的 C3)。
            'encoder_wrap': 0,
            'odom_rate': 30.0,
            # 與 mower_manager 的 watchdog 是兩層獨立保護，不要省略
            'cmd_vel_timeout': 0.5,
            'driver_type': driver_type,
            'serial_port': serial_port,
            'serial_baud': 115200,
            # /cmd_vel 等比例縮放，急停 (零速度) 縮放後仍然是零。
            # ParameterValue 強制成 float：命令列給 max_vel_scale:=1 時
            # 會被解析成整數，與 bridge_node 宣告的 double 型別衝突。
            'max_vel_scale': ParameterValue(max_vel_scale, value_type=float),
        }]
    )

    # ---- 2. 實體雷達 (待填) ---------------------------------------------
    # TODO: 型號待現場跑 deploy/hw_probe.sh 確認（第 3 節 VID:PID、第 4 節原始位元組）。
    # 拿到雷達之後在這裡啟動它的驅動節點。需要先確認的事:
    #   - 雷達型號與對應的 ROS 2 套件
    #   - 裝置路徑固定成 /dev/mowerbot_lidar (在 deploy/99-mowerbot.rules 加一行)，
    #     不要用 /dev/ttyUSB0
    #   - 鮑率
    #   - frame_id 必須是 "radar" (car.xacro 裡雷達的 link 名稱)，
    #     否則 TF 對不起來、scan matching 會拿到錯的外參
    #   - 掃描頻率與角解析度，要與 mapper_params.yaml 的設定相容
    # lidar_node = Node(
    #     package='<雷達套件>',
    #     executable='<驅動節點>',
    #     name='lidar',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'frame_id': 'radar',
    #         'serial_port': '/dev/mowerbot_lidar',
    #         'serial_baudrate': 0,
    #     }]
    # )

    # ---- 3. 其餘節點照舊 -------------------------------------------------
    # SLAM / teleop / manager / map_to_boundary / f2c_server 與模擬時完全相同，
    # 直接沿用 mower_control.launch.py，不要複製一份 (複製會兩邊分岔)。
    # 用 GroupAction 包起來隔離 launch configuration，理由見報告 9.2 節。
    mower_control = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'mower_control.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                # 實車沒有 gazebo.launch.py 幫忙起 robot_state_publisher
                'start_rsp': 'true',
            }.items()
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='實車用實際時鐘。只有在對著 rosbag 重播時才設 true。'),

        DeclareLaunchArgument(
            'encoder_ticks_per_rev',
            default_value='0',
            description='每轉的編碼器 tick 數 (含減速比與四倍頻)。'
                        '0 = 還沒填，bridge_node 會拒絕啟動。'
                        '這個值要查驅動板/編碼器的文件，不要猜。'),

        DeclareLaunchArgument(
            'driver_type',
            # 留空：實車啟動檔不預設成假驅動。忘了帶這個參數時 bridge_node 會以
            # 「未知的 driver_type」拒絕啟動，而不是安靜地用 loopback 跑
            # （那會變成「車子看起來正常、里程計完美，但輪子根本沒轉」）。
            default_value='',
            description='馬達驅動實作，必填。loopback 是沒有硬體時用的假驅動；'
                        '接上真實驅動板之後改成該實作的名稱 '
                        '(見 mowerbot_bridge/drivers/README.md)。'),

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/mowerbot_base',
            description='驅動板的序列埠。實車用 udev 固定的 /dev/mowerbot_base；'
                        '測試時指向假 C30D 的 pty (test/tools/fake_c30d.py)。'),

        DeclareLaunchArgument(
            'max_vel_scale',
            default_value='1.0',
            description='/cmd_vel 的等比例縮放 (乘在 linear.x 與 angular.z 上)。'
                        '實機階段 2 用 0.2 之類的值。急停永遠是零。'),

        LogInfo(msg=['[bringup_real] 啟動實車節點 (driver_type=', driver_type,
                     ', encoder_ticks_per_rev=', ticks_per_rev,
                     ', serial_port=', serial_port,
                     ', max_vel_scale=', max_vel_scale, ')']),
        DeclareLaunchArgument(
            'allow_provisional',
            default_value='false',
            description='車輛幾何含 provisional (暫定) 值時是否放行。實車預設 false：'
                        '有任何暫定值就拒絕啟動 (清單與量測方式見 docs/measurement_worklist.md)。'),

        DeclareLaunchArgument(
            'provisional_override_reason',
            default_value='',
            description='allow_provisional:=false 但仍要強制通行時的理由 (非空字串，寫進 log)。'),

        LogInfo(msg='[bringup_real] 注意：雷達驅動尚未填入，SLAM 不會收到 /scan。'),

        geometry_guard,
        RegisterEventHandler(OnProcessExit(
            target_action=geometry_guard,
            on_exit=lambda event, _ctx: (
                [bridge_node,
                 # lidar_node,          # TODO: 見上面的說明
                 mower_control]
                if event.returncode == 0 else
                [OpaqueFunction(function=_geometry_gate_failed)]))),
    ])
