import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
# 注意：SetParameter 位於 launch_ros.actions 之中
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue

from mowerbot_action.manager import DEFAULT_OVERLAP_RATIO, WAYPOINT_SPACING
from mowerbot_action import swath_angle
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


def launch_bool(value, name):
    """launch 參數的布林值：只認 true / false (與 1 / 0)，其他一律拒絕啟動，不要猜。"""
    v = value.strip().lower()
    if v in ('true', '1'):
        return True
    if v in ('false', '0'):
        return False
    raise RuntimeError('%s 只能是 true 或 false，收到 %r' % (name, value))


# 自動模式的放行條件 (2026-10-15)：mower_manager 的三個參數，說明見 manager.py。
# launch 參數留空 = 跟著 use_sim_time 走：實車 (false) 開、模擬 (true) 關。
# 模擬沒有手把；Gazebo 的既有測試 (Phase C/N/O/P/Q/L) 也都是不按 LB 直接切 mode 1。
# auto_requires_joystick (2026-10-19)：手把連著就好、不用按住 LB；bringup_real 用它取代 deadman。
AUTO_GATES = ('auto_requires_deadman', 'auto_requires_joystick', 'auto_requires_scan')


def resolve_sim_default(context, keys):
    """launch 參數留空 = 實車 (use_sim_time false) 開、模擬關；有寫就照寫的。"""
    sim = launch_bool(LaunchConfiguration('use_sim_time', default='true').perform(context),
                      'use_sim_time')
    out = {}
    for k in keys:
        v = LaunchConfiguration(k).perform(context)
        out[k] = (not sim) if v.strip() == '' else launch_bool(v, k)
    return out


def resolve_auto_gates(context):
    return resolve_sim_default(context, AUTO_GATES)


def resolve_swath_angle(context):
    """割草線方向 (2026-10-19)：auto 或角度 (度)。留空 = 跟著 use_sim_time：實車 auto、模擬 0
    (Gazebo 的既有測試與報告數據都是沿 x 軸的割草線)。其他值拒絕啟動，不要猜。"""
    sim = launch_bool(LaunchConfiguration('use_sim_time', default='true').perform(context),
                      'use_sim_time')
    v = LaunchConfiguration('swath_angle').perform(context).strip().lower()
    if v == '':
        return '0' if sim else 'auto'
    try:
        swath_angle.parse_angle(v)
    except ValueError as e:
        raise RuntimeError(str(e))
    return v


# 手把模式 (2026-10-18 will)：LB + X 手動 (純遙控車)、LB + A 開始建圖、LB + B 規劃割草、Y 存圖 (10-19)。
#   mapping_on_button    slam_toolbox 讀 /scan_slam，slam_scan_gate 等第一次切到建圖模式才轉送 /scan；
#                        同時開 map_autosave 給 Y 存圖。留空 = 跟著 use_sim_time：實車開、模擬關
#                        (模擬的既有測試一開始就要有地圖)。
#   save_map_before_f2c  mower_manager 切進 mode 1 先呼叫 map_autosave 存圖，存成功才規劃。
#                        10-18 的做法；10-19 will 改成 LB + B 直接規劃、存圖按 Y，所以預設 false
#                        (留空 = false，不跟著 use_sim_time)。
MODE_OPTIONS = ('mapping_on_button',)


def resolve_mode_options(context):
    opts = resolve_sim_default(context, MODE_OPTIONS)
    v = LaunchConfiguration('save_map_before_f2c').perform(context)
    opts['save_map_before_f2c'] = False if v.strip() == '' else launch_bool(v, 'save_map_before_f2c')
    return opts


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

    # SLAM 建圖。實車 (mapping_on_button) 要等按 LB + A 才開始：slam_toolbox 改讀 /scan_slam，
    # 由 slam_scan_gate 在第一次切到建圖模式之後才把 /scan 轉過去 (理由見 slam_scan_gate.py)。
    def slam_action(context):
        opts = resolve_mode_options(context)
        on = opts['mapping_on_button']
        actions = [
            LogInfo(msg='[mower_control] 建圖：%s' % (
                '按 LB + A (建圖模式) 才開始，在那之前沒有地圖' if on
                else 'bringup 一開就開始 (模擬)')),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'mapping.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time,
                                  'scan_topic': '/scan_slam' if on else '/scan'}.items()),
        ]
        if on:
            actions.append(Node(
                package='mowerbot_action',
                executable='slam_scan_gate',
                name='slam_scan_gate',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'input_topic': '/scan', 'output_topic': '/scan_slam'}]))
        if on or opts['save_map_before_f2c']:
            # 手把 Y 存圖 (teleop 呼叫 /map_autosave/save)；save_map_before_f2c 時 mower_manager 也呼叫
            actions.append(Node(
                package='mowerbot_action',
                executable='map_autosave',
                name='map_autosave',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]))
        return actions

    slam_launch = OpaqueFunction(function=slam_action)

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

    # 草地邊界記錄器 (2026-10-08)：用手把沿草地邊緣開一圈，記成 /lawn_boundary，
    # map_to_boundary 會把割草範圍限制在它裡面。不呼叫它的服務就什麼都不做。
    # 用法見 mowerbot_action/boundary_recorder.py 開頭。
    boundary_recorder_node = Node(
        package='mowerbot_action',
        executable='boundary_recorder',
        name='boundary_recorder',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 禁區 (2026-10-18)：使用者在存好的地圖上用紅色畫的區域，發布成 /keepout_mask，
    # map_to_boundary 與 local costmap 的 keepout_layer 都會避開。開機永遠沒有禁區
    # (每次開機地圖原點不同)。用法見 mowerbot_action/keepout.py 開頭。
    keepout_node = Node(
        package='mowerbot_action',
        executable='keepout_server',
        name='keepout_server',
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
            {'use_sim_time': use_sim_time,
             # 速度上限比例 (2026-10-20 D1)：乘在 joystick.yaml 的 scale_linear / scale_angular 上
             'speed_scale': ParameterValue(LaunchConfiguration('speed_scale', default='1.0'), value_type=float)}
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
    # 放行條件 (auto_requires_*) 要等 launch 執行時才知道 use_sim_time，所以包在 OpaqueFunction 裡
    def manager_action(context):
        gates = resolve_auto_gates(context)
        gates['save_map_before_f2c'] = resolve_mode_options(context)['save_map_before_f2c']
        gates['swath_angle'] = resolve_swath_angle(context)
        log = LogInfo(msg='[mower_control] 自動模式放行條件：按住 LB = %s，手把連線 = %s，'
                          '光達 /scan 不中斷 = %s；切到自動割草先存圖 = %s；割草線方向 = %s'
                          % (gates['auto_requires_deadman'], gates['auto_requires_joystick'],
                             gates['auto_requires_scan'], gates['save_map_before_f2c'],
                             gates['swath_angle']))
        manager_node = Node(
            package='mowerbot_action',
            executable='mower_manager',
            name='mower_manager',
            output='screen',
            parameters=[gates, {
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
                # 割草線方向 auto 跟 F2C 一樣在扣掉地頭的區域上挑 (2026-10-19)
                'headland_width': geom.headland_width,
                # 周邊環繞的轉角偵測 (vehicle_geometry) 與起點 = 終點的判定門檻 (nav2)
                'perimeter_corner_window': geom.perimeter_corner_window,
                'perimeter_corner_angle': geom.perimeter_corner_angle,
                'goal_xy_tolerance': goal_xy_tolerance,
            }]
        )
        return [log, manager_node]

    manager_node = OpaqueFunction(function=manager_action)

    # 4. 回傳 LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'speed_scale',
            default_value='1.0',
            description='手把速度比例 (0.05 ~ 1.0)，乘在 joystick.yaml 的 scale_linear / scale_angular 上。'
                        '實車由 car-start 的速度上限傳進來；Nav2 那邊是 navigation.launch.py 的 speed_scale。'),
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

        # 自動模式的放行條件 (2026-10-15)，留空 = 實車開、模擬關 (見 AUTO_GATES 的註解)
        DeclareLaunchArgument(
            'auto_requires_deadman',
            default_value='',
            description='自動模式 (mode 1/3) 要按住 LB 才轉發 Nav2 速度指令。'
                        '留空 = 跟著 use_sim_time：實車 true、模擬 false。'
        ),
        DeclareLaunchArgument(
            'auto_requires_joystick',
            default_value='',
            description='自動模式 (mode 1/3) 手把要連著才轉發 Nav2 速度指令 (不用按住 LB；'
                        '斷線過要重新連上再按一下 LB)。auto_requires_deadman 開著時沒有作用。'
                        '留空 = 跟著 use_sim_time：實車 true、模擬 false。'
        ),
        DeclareLaunchArgument(
            'auto_requires_scan',
            default_value='',
            description='自動模式 (mode 1/3) 光達 /scan 中斷超過 0.5 s 就停車。'
                        '留空 = 跟著 use_sim_time：實車 true、模擬 false。'
        ),

        # 手把模式 (2026-10-18 / 10-19)，見 MODE_OPTIONS 的註解
        DeclareLaunchArgument(
            'mapping_on_button',
            default_value='',
            description='按 LB + A (建圖模式) 才開始建圖，在那之前 slam_toolbox 收不到光達資料；'
                        '同時開 map_autosave 給手把 Y 存圖。留空 = 跟著 use_sim_time：實車 true、模擬 false。'
        ),
        DeclareLaunchArgument(
            'swath_angle',
            default_value='',
            description='割草線方向：auto = 每次規劃挑掉頭最少的方向，數字 = 固定角度 (度，從 map 的 x 軸量)。'
                        '留空 = 跟著 use_sim_time：實車 auto、模擬 0。'
        ),
        DeclareLaunchArgument(
            'save_map_before_f2c',
            default_value='',
            description='切到自動割草 (mode 1) 先存一份地圖，存成功才規劃。'
                        '留空 = false (2026-10-19 起存圖改按手把 Y)。'
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
        boundary_recorder_node,
        keepout_node,
    ])