import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
# 注意：SetParameter 位於 launch_ros.actions 之中
from launch_ros.actions import Node, SetParameter

from mowerbot_action.manager import DEFAULT_OVERLAP_RATIO, WAYPOINT_SPACING
from mowerbot_description import vehicle_geometry

def check_headland(context, geom, nav2_params_path):
    """斷言 headland_width >= 外接半徑 + xy_goal_tolerance (階段 31)。

    階段 38 起 headland_width 不再是 launch 參數，而是 vehicle_geometry 由幾何推導：
      ceil((rotation_swept_radius + TURN_MARGIN 0.12) / 0.05) x 0.05 = 1.20 m
    f2c_server 自己另有一道 headland >= rotation_swept_radius + 0.10 的斷言 (階段 38 規格)。
    這一道保留，因為它看的是 nav2 的 xy_goal_tolerance (它決定割草線終點「停在哪裡算到了」)：
    之後有人放寬 xy_goal_tolerance 而 TURN_MARGIN 沒跟著改，就在這裡直接失敗。
    """
    with open(nav2_params_path) as fh:
        nav2 = yaml.safe_load(fh)
    xy_tol = float(nav2['controller_server']['ros__parameters']
                   ['general_goal_checker']['xy_goal_tolerance'])
    need = geom.rotation_swept_radius + xy_tol
    if geom.headland_width < need:
        raise RuntimeError(
            'headland_width = %.4f m (vehicle_geometry 推導) 小於 '
            'rotation_swept_radius %.4f m + xy_goal_tolerance %.2f m = %.4f m。'
            'xy_goal_tolerance 放寬了的話，vehicle_geometry.TURN_MARGIN 要跟著重新決定。'
            % (geom.headland_width, geom.rotation_swept_radius, xy_tol, need))
    return []


def generate_launch_description():
    # 1. 取得各個套件的路徑
    bringup_dir = get_package_share_directory('mowerbot_bringup')
    description_dir = get_package_share_directory('mowerbot_description')
    bridge_dir = get_package_share_directory('mowerbot_bridge')

    # 車輛幾何的單一來源 (階段 30；階段 38 起一律經過 vehicle_geometry.load())
    geom = vehicle_geometry.load()
    # 起點 = 終點的判定門檻 = nav2 general_goal_checker 的 xy_goal_tolerance (單一來源，階段 38)
    with open(os.path.join(bringup_dir, 'config', 'nav2_params.yaml')) as fh:
        goal_xy_tolerance = float(yaml.safe_load(fh)['controller_server']['ros__parameters']
                                  ['general_goal_checker']['xy_goal_tolerance'])

    # 2. 宣告 Launch 參數
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 3. 定義節點與包含的 Launch 檔案

    # 機器人描述檔與狀態發布器
    # 模擬時 gazebo.launch.py 已經啟動 robot_state_publisher
    # (spawn_entity 需要從 /robot_description 讀 URDF)，
    # 這裡再啟動一次會出現兩個同名節點同時對 /tf 與 /robot_description 發布相同內容，
    # 產生大量 TF_REPEATED_DATA 警告，所以預設 false；
    # 之後跑實體車 (沒有 Gazebo) 時要用 start_rsp:=true 啟動它。
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(LaunchConfiguration('start_rsp'))
    )

    # 關節狀態發布器 (解決前輪不顯示的關鍵)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # SLAM 建圖
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 手把硬體驅動
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[
            os.path.join(bridge_dir, 'config', 'joystick.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # 邊界提取節點
    boundary_node = Node(
        package='mowerbot_action',
        executable='map_to_boundary',
        name='map_to_boundary',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 手把控制邏輯
    teleop_node = Node(
        package='mowerbot_bridge',
        executable='teleop_node',
        name='mower_teleop',
        output='screen',
        parameters=[
            os.path.join(bridge_dir, 'config', 'joystick.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )

    # F2C 路徑規劃伺服器 (C++)
    f2c_server_node = Node(
        package='mowerbot_planner',
        executable='f2c_server',
        name='f2c_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 地頭寬度：由車輛幾何推導 (階段 38)，不是 launch 參數、也沒有寫死的預設值。
            # rotation_swept_radius 給 f2c_server 做啟動斷言 (headland >= 它 + 0.10)。
            'headland_width': geom.headland_width,
            'rotation_swept_radius': geom.rotation_swept_radius,
            'headland_min_margin': vehicle_geometry.HEADLAND_MIN_MARGIN,
            'waypoint_spacing': WAYPOINT_SPACING,
        }]
    )

    # 模式管理與 Watchdog
    manager_node = Node(
        package='mowerbot_action',
        executable='mower_manager',
        name='mower_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # 跑道 (lead-in) 長度，讓實測可以掃不同數值而不用重建
            'lead_in_length': LaunchConfiguration('lead_in_length'),
            # 割草線重疊率，讓實測可以掃不同數值而不用重建
            'overlap_ratio': LaunchConfiguration('overlap_ratio'),
            # 淨空門檻與刀盤寬：全部來自 vehicle_geometry (階段 38)，manager 不自己算幾何。
            #   lateral_half_extent    直線通過的門檻 (body_width_total / 2)
            #   rotation_swept_radius  要原地旋轉的點的門檻 (後輪軸到車頭角的距離)
            #   soft_inflation_radius  跑道離內部障礙物的門檻 (= local_costmap 的 inflation_radius)
            'lateral_half_extent': geom.lateral_half_extent,
            'rotation_swept_radius': geom.rotation_swept_radius,
            'soft_inflation_radius': geom.soft_inflation_radius,
            'blade_width': geom.blade_width,
            # 周邊環繞的轉角偵測 (vehicle_geometry) 與起點 = 終點的判定門檻 (nav2)
            'perimeter_corner_window': geom.perimeter_corner_window,
            'perimeter_corner_angle': geom.perimeter_corner_angle,
            'goal_xy_tolerance': goal_xy_tolerance,
        }]
    )

    # 4. 回傳 LaunchDescription
    return LaunchDescription([
        # 模擬時預設不啟動 robot_state_publisher (由 gazebo.launch.py 負責)
        DeclareLaunchArgument(
            'start_rsp',
            default_value='false',
            description='Start robot_state_publisher. Keep false under Gazebo '
                        '(gazebo.launch.py already starts it); set true on the real robot.'
        ),

        # 每條割草線前面的跑道 (lead-in) 長度，單位公尺，0 代表停用
        DeclareLaunchArgument(
            'lead_in_length',
            default_value='0.5',
            description='Lead-in (headland run-up) length in metres prepended to '
                        'each mowing swath. 0 disables it.'
        ),

        # 割草線重疊率：割草線間距 = blade_width x (1 - overlap_ratio)。
        # 0 代表零重疊 (間距 = 刀盤寬)。
        DeclareLaunchArgument(
            'overlap_ratio',
            default_value=str(DEFAULT_OVERLAP_RATIO),
            description='Swath overlap ratio. Swath spacing = blade_width * '
                        '(1 - overlap_ratio). 0 means no overlap.'
        ),

        # F2C 地頭 (headland) 寬度：階段 38 起不再是 launch 參數，由車輛幾何推導
        # (vehicle_geometry.headland_width = 1.20 m，推導見 f2c_server.cpp 與 vehicle_geometry.py)。

        # headland 與 xy_goal_tolerance 的一致性斷言 (階段 31)。放在所有節點之前，
        # 不成立時整個 launch 在啟動任何節點之前就失敗。
        OpaqueFunction(function=check_headland, args=[
            geom, os.path.join(bringup_dir, 'config', 'nav2_params.yaml')]),

        # 【核心修正】全域設定模擬時間參數
        SetParameter(name='use_sim_time', value=use_sim_time),

        robot_description_launch,
        joint_state_publisher,
        slam_launch,
        joy_node,
        teleop_node,
        f2c_server_node,
        manager_node,
        boundary_node,
    ])