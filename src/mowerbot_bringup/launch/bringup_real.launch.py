import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    use_sim_time = LaunchConfiguration('use_sim_time')
    ticks_per_rev = LaunchConfiguration('encoder_ticks_per_rev')
    driver_type = LaunchConfiguration('driver_type')

    # ---- 1. 底盤橋接 ----------------------------------------------------
    # 這是實車與模擬唯一真正不同的節點。
    bridge_node = Node(
        package='mowerbot_bridge',
        executable='bridge_node',
        name='mower_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 車體物理量：與 URDF 一致，不要各填各的
            'wheel_radius': 0.17,
            'wheel_separation': 0.58,
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
        }]
    )

    # ---- 2. 實體雷達 (待填) ---------------------------------------------
    # TODO: 拿到雷達之後在這裡啟動它的驅動節點。需要先確認的事:
    #   - 雷達型號與對應的 ROS 2 套件
    #   - 裝置路徑 (建議做 udev rule 固定成 /dev/lidar，不要用 /dev/ttyUSB0)
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
    #         'serial_port': '/dev/lidar',
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
            default_value='loopback',
            description='馬達驅動實作。loopback 是沒有硬體時用的假驅動；'
                        '接上真實驅動板之後改成該實作的名稱 '
                        '(見 mowerbot_bridge/drivers/README.md)。'),

        LogInfo(msg=['[bringup_real] 啟動實車節點 (driver_type=', driver_type,
                     ', encoder_ticks_per_rev=', ticks_per_rev, ')']),
        LogInfo(msg='[bringup_real] 注意：雷達驅動尚未填入，SLAM 不會收到 /scan。'),

        bridge_node,
        # lidar_node,          # TODO: 見上面的說明
        mower_control,
    ])
