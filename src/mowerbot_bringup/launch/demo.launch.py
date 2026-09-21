import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """一鍵啟動：把 gazebo / mower_control / navigation / rviz 四支整合成一支。

    原本示範時要開四個終端機各跑一支，其中任何一支掛掉 (在 VMware 上最常見的
    就是 gzclient 閃退) 就得四個全部重開。這支把啟動順序固定下來，
    搭配 workspace 根目錄的 run_demo.sh 使用。
    """
    pkg_bringup = get_package_share_directory('mowerbot_bringup')
    pkg_description = get_package_share_directory('mowerbot_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    nav = LaunchConfiguration('nav')
    overlap_ratio = LaunchConfiguration('overlap_ratio')

    # ---- 各階段要啟動的東西 ----------------------------------------------
    # 用 GroupAction 包起來 (scoped=True 是預設值) 把 Gazebo 的 launch 參數關在
    # 自己的作用域裡。IncludeLaunchDescription 的 launch configuration 是所有被
    # include 的檔案共用的，而 gazebo_ros 的 gzserver.launch.py 會宣告一個
    # params_file (預設空字串)。不隔離的話，那個空字串會留在 context 裡，
    # 稍後 include 進來的 navigation.launch.py 會認為 params_file 已經有人給了，
    # 於是它自己的預設值 nav2_params.yaml 不生效，變成空路徑 (正規化後是 ".")，
    # lifecycle_manager 讀不到 node_names 就直接 abort (exit -6)，Nav2 起不來。
    # 單獨跑 navigation.launch.py 不會有這個問題，只有一鍵啟動會踩到。
    gazebo = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'world': world,
                'gui': gui,
            }.items()
        )
    ])

    mower_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'mower_control.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'overlap_ratio': overlap_ratio,
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'rviz.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ---- 啟動順序 --------------------------------------------------------
    # 用 TimerAction 錯開，每一段都先印出正在啟動什麼，
    # 這樣畫面卡住時一眼就看得出是卡在哪一段。
    #
    #   t = 0 s   gazebo：模擬器本體 + robot_state_publisher + spawn_entity
    #   t = 10 s  mower_control：要等機器人 spawn 完、/clock 開始發布，
    #             SLAM 與 manager 才有 sim time 可用
    #   t = 16 s  navigation：要等 TF 鏈完整 (含 SLAM 的 map -> odom)，
    #             否則 local_costmap 會抓不到轉換
    #   t = 18 s  rviz：最後開畫面，此時 /map 與 TF 都已經在了
    stage_gazebo = TimerAction(
        period=0.0,
        actions=[
            LogInfo(msg=['[demo] t=0s  啟動 Gazebo (world=', world, ', gui=', gui, ')']),
            gazebo,
        ]
    )

    stage_control = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='[demo] t=10s 啟動 mower_control (SLAM / teleop / manager / F2C)'),
            mower_control,
        ]
    )

    # navigation.launch.py 內部自己還有一個 TimerAction (延後 5 秒才起
    # lifecycle_manager，避開 change_state 的 DDS 競態)，所以 Nav2 實際進入
    # active 會在 t=21s 前後。這裡不要再往上疊，疊太多只是讓示範等更久，
    # 對那個競態一點幫助也沒有 (見 docs/simulation_results.md 7.12 節)。
    stage_nav = TimerAction(
        period=16.0,
        condition=IfCondition(nav),
        actions=[
            LogInfo(msg='[demo] t=16s 啟動 navigation (controller_server + lifecycle_manager)'),
            navigation,
        ]
    )

    stage_rviz = TimerAction(
        period=18.0,
        condition=IfCondition(rviz),
        actions=[
            LogInfo(msg='[demo] t=18s 啟動 RViz'),
            rviz_launch,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'world',
            default_value='demo_lawn.world',
            description='World file name under mowerbot_bringup/worlds. '
                        'demo_lawn.world is the obstacle-free demo lawn; '
                        'use mow_field.world for the original test field.'),

        # 預設無頭 (gui:=false)。理由：gzclient 在這台 VMware 虛擬機的
        # 軟體 OpenGL (mesa svga) 底下不穩定，示範到一半會閃退，
        # 一閃退連帶整組要重開。而自動化測試全部跑無頭模式，
        # 幾十次沒有當過一次 —— 問題在圖形介面，不在模擬本身。
        # 需要看 Gazebo 的 3D 畫面時用 gui:=true 開啟。
        DeclareLaunchArgument(
            'gui',
            default_value='false',
            description='Start the Gazebo GUI (gzclient). Default false: gzclient '
                        'is unstable on this VMware VM (software OpenGL) and crashes '
                        'mid-demo, while headless runs have never crashed.'),

        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz2 with the mowerbot config'),

        DeclareLaunchArgument(
            'nav',
            default_value='true',
            description='Start the Nav2 stack. Set false to run mapping only.'),

        # 割草線重疊率，沿用 mower_control.launch.py 的預設值 0.4
        # (選定理由見 docs/simulation_results.md 4.6.5 節)。
        # 兩邊的預設值要一起改，不要只改一邊。
        DeclareLaunchArgument(
            'overlap_ratio',
            default_value='0.4',
            description='Swath overlap ratio, passed through to '
                        'mower_control.launch.py (same default: 0.4).'),

        stage_gazebo,
        stage_control,
        stage_nav,
        stage_rviz,
    ])
