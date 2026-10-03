import copy
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.logging
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mowerbot_description import vehicle_geometry


# AlignController 裡「注入負責的鍵」(階段 38 後續 5)：權重 (critics 清單與每個 critic 的 <Critic>.<參數>)、
# max_vel_x = 0 (只能原地轉)、xy_goal_tolerance (= general_goal_checker，與 FollowPath 同一來源)。
# 其餘的鍵 (速度 / 加速度 / 取樣 / min_speed_theta ...) 不歸注入管：參數檔的 AlignController 有寫就用它的，
# 沒寫才繼承 FollowPath 的值 (繼承不算覆寫)。
# 舊版把 FollowPath 整塊複製蓋上去，參數檔裡任何 AlignController 的設定都被靜默蓋掉 ——
# min_speed_theta 掃描 15 次全部量到 0.0 的行為，就是這個 (stage38_decisions.md 15、16 節)。
ALIGN_OWNED_SCALARS = ('critics', 'max_vel_x', 'xy_goal_tolerance')


def _align_owned(key):
    return key in ALIGN_OWNED_SCALARS or '.' in key


def controller_injection(params_path, geom):
    """回傳 (要排在 params_file 後面的 controller_server 參數, 啟動警告字串)。

    每一個注入值和參數檔裡既有的值不同時印一行 WARN「注入覆寫 ...」——
    即使是正常情況 (max_vel_x 0.70 → 0.0) 也印，讓覆寫永遠看得見。
    """
    log = launch.logging.get_logger('navigation.launch')
    with open(params_path) as fh:
        cs = yaml.safe_load(fh)['controller_server']['ros__parameters']
    follow = cs['FollowPath']
    xy_goal_tolerance = float(cs['general_goal_checker']['xy_goal_tolerance'])
    file_align = cs.get('AlignController', {}) or {}
    owned = {k: copy.deepcopy(v) for k, v in follow.items() if _align_owned(k)}
    owned['max_vel_x'] = 0.0
    owned['xy_goal_tolerance'] = xy_goal_tolerance
    align = {}
    for k, v in owned.items():
        # 參數檔 AlignController 沒寫的鍵，「既有的值」就是它會從 FollowPath 繼承的值
        old = file_align.get(k, follow.get(k, '(未設定)'))
        if old != v:
            log.warning('注入覆寫 align_controller.%s：%r → %r' % (k, old, v))
        align[k] = v
    for k, v in follow.items():
        if k not in owned and k not in file_align:
            align[k] = copy.deepcopy(v)          # 繼承，不是覆寫
    # min_speed_xy 的正當性依賴 max_vel_x = 0 (階段 38 後續 6)：AlignController 不能平移時，
    # min_speed_xy 只是打開 min_speed_theta 的開關；max_vel_x 一旦非零，它就真的變成速度限制了。
    if 'min_speed_xy' in align or 'min_speed_xy' in file_align:
        if float(align['max_vel_x']) != 0.0:
            raise RuntimeError(
                'AlignController 設定了 min_speed_xy = %r，但 max_vel_x = %r ≠ 0。'
                'min_speed_xy 的正當性依賴 max_vel_x = 0：只有在不能平移時，它才只是打開 min_speed_theta 的開關，'
                '否則它就是一個速度限制 (凍結項目)。' % (file_align.get('min_speed_xy'), align['max_vel_x']))
    kept = sorted(k for k in file_align if k not in owned)
    if kept:
        log.info('align_controller 保留參數檔的設定 (不歸注入管)：%s'
                 % ', '.join('%s=%r' % (k, file_align[k]) for k in kept))
    injected = {'AlignController': align,
                'FollowPath': {'xy_goal_tolerance': xy_goal_tolerance}}
    old = follow.get('xy_goal_tolerance')
    if old is not None and old != xy_goal_tolerance:
        log.warning('注入覆寫 FollowPath.xy_goal_tolerance：%r → %r' % (old, xy_goal_tolerance))
    align_xy = float(cs['approach_goal_checker']['xy_goal_tolerance'])
    old = cs.get('align_goal_checker', {}).get('xy_goal_tolerance')
    if old is not None and old != align_xy:
        log.warning('注入覆寫 align_goal_checker.xy_goal_tolerance：%r → %r' % (old, align_xy))
    injected['align_goal_checker'] = {'xy_goal_tolerance': align_xy}

    align_yaw = float(cs['align_goal_checker']['yaw_goal_tolerance'])
    # 啟動警告 (不是錯誤)：對正容忍值吃掉多少轉彎餘裕。每次啟動都要看得見。
    lat = geom.lateral_half_extent * math.cos(align_yaw) + geom.front_extent * math.sin(align_yaw)
    margin = geom.headland_width - geom.rotation_swept_radius
    align_warning = (
        '⚠️ 對正容忍值 align_goal_checker.yaw_goal_tolerance = %.3f rad：車頭角側向伸出 lateral(%.3f) = '
        '%.3f m，超出 lateral_half_extent %.3f m 共 %.3f m，佔轉彎餘裕 (headland %.2f - rotation_swept_radius %.4f '
        '= %.4f m) 的 %.0f %%。這是模擬限制下的實驗值，實機必須重新定案。'
        % (align_yaw, align_yaw, lat, geom.lateral_half_extent, lat - geom.lateral_half_extent,
           geom.headland_width, geom.rotation_swept_radius, margin,
           100.0 * (lat - geom.lateral_half_extent) / margin))
    return injected, align_warning


def generate_launch_description():
    # 1. 取得套件路徑與預設參數檔
    bringup_dir = get_package_share_directory('mowerbot_bringup')
    default_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # 車體 footprint 與 inflation_radius 由 vehicle_geometry (車輛幾何的單一來源) 算出來，
    # 不寫在 nav2_params.yaml。local_costmap 是 controller_server 行程裡另一個節點
    # (/local_costmap/local_costmap)，所以寫成獨立的參數檔、排在 params_file 後面
    # 傳給 controller_server，以節點全名對應，後面的檔案覆蓋前面的。
    #
    # footprint (階段 38)：base_link = 後輪軸中心，真實四角多邊形，前 0.975 / 後 0.155 / 半寬 0.42。
    # 不要改用 robot_radius —— 圓形近似對這個 footprint 失效 (內接 0.155 : 外接 1.06)。
    # inflation_radius：vehicle_geometry.SOFT_INFLATION_RADIUS，manager 與 Phase Q 讀同一個。
    geom = vehicle_geometry.load()
    with open(default_params) as fh:
        nav2_yaml = yaml.safe_load(fh)
    # 斷言：nav2_params.yaml 不得自己再寫一份 inflation_radius / footprint / robot_radius
    # (寫了就是第二個來源；值不同時 Nav2 會用哪一份取決於檔案順序，很難察覺)
    lc = nav2_yaml['local_costmap']['local_costmap']['ros__parameters']
    dup = [k for k in ('footprint', 'robot_radius') if k in lc]
    infl = lc.get('inflation_layer', {}).get('inflation_radius')
    if infl is not None and abs(float(infl) - geom.soft_inflation_radius) > 1e-9:
        dup.append('inflation_layer.inflation_radius=%r' % infl)
    if dup:
        raise RuntimeError(
            'nav2_params.yaml 的 local_costmap 不得自行設定 %s：footprint 與 inflation_radius '
            '只能來自 mowerbot_description/vehicle_geometry.py (soft_inflation_radius = %.2f)。'
            % (', '.join(dup), geom.soft_inflation_radius))
    # DWB (FollowPath) 的 xy_goal_tolerance 一律等於 general_goal_checker 的 (單一來源，階段 38)
    cs = nav2_yaml['controller_server']['ros__parameters']
    if 'xy_goal_tolerance' in cs.get('FollowPath', {}):
        raise RuntimeError('nav2_params.yaml 的 FollowPath 不得自行設定 xy_goal_tolerance：'
                           '它由 navigation.launch.py 從 general_goal_checker 注入。')
    # align_goal_checker 的 xy 容忍沿用 approach_goal_checker (單一來源)
    if 'xy_goal_tolerance' in cs.get('align_goal_checker', {}):
        raise RuntimeError('nav2_params.yaml 的 align_goal_checker 不得自行設定 xy_goal_tolerance：'
                           '它沿用 approach_goal_checker，由 navigation.launch.py 注入。')
    footprint = str(geom.footprint)
    footprint_params = tempfile.NamedTemporaryFile(
        mode='w', prefix='mowerbot_footprint_', suffix='.yaml', delete=False)
    yaml.safe_dump({
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'footprint': footprint,
            'inflation_layer': {'inflation_radius': geom.soft_inflation_radius},
        }}},
    }, footprint_params)
    footprint_params.close()

    # 2. 宣告 Launch 參數
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # 3. Nav2 區域路徑控制器
    # 本專案只啟動 controller_server：路徑由 F2C 產生、mower_manager 直接呼叫
    # FollowPath action，不需要 planner_server / bt_navigator 等全局規劃元件。
    def controller_server_action(context):
        # 注入要以「這次實際用的參數檔」為準 (params_file 可能是實驗用的檔案)，所以延到 launch 執行時才算
        params_path = params_file.perform(context)
        injected, align_warning = controller_injection(params_path, geom)
        inj_file = tempfile.NamedTemporaryFile(
            mode='w', prefix='mowerbot_controller_inject_', suffix='.yaml', delete=False)
        yaml.safe_dump({'controller_server': {'ros__parameters': injected}}, inj_file)
        inj_file.close()
        return [LogInfo(msg=align_warning), Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, footprint_params.name, inj_file.name,
                    {'use_sim_time': use_sim_time}],
        remappings=[
            # 【安全關鍵】controller_server 預設把速度指令發到 /cmd_vel，
            # 那是直接進底盤的話題，會完全繞過 mower_manager 的模式仲裁
            # 與急停(mode 4)攔截 —— 等於 Nav2 可以在急停狀態下把車開走。
            # 因此改發到 /cmd_vel_nav，強制 Nav2 的輸出必須先經過 manager
            # 審核 (只有 mode 1/3 才轉發)，再由 manager 送到 /cmd_vel。
            ('cmd_vel', 'cmd_vel_nav'),
        ]
    )]

    controller_server = OpaqueFunction(function=controller_server_action)

    # 4. 生命週期管理器：負責把 controller_server 帶到 active 狀態
    #
    # 延後 5 秒才啟動：autostart 會在 lifecycle_manager 一上線就立刻對
    # controller_server 送出 change_state 請求。兩個節點同時啟動時，請求送得出去、
    # 轉換也會成功，但回應有機會在 DDS 尚未完成配對前就發出，於是出現
    #   failed to send response to /controller_server/change_state (timeout)
    # lifecycle_manager 永遠等不到回應，controller_server 就卡在 inactive。
    # 先讓 controller_server 把服務註冊好再啟動管理器可以避開這個競態。
    #
    # 階段 8：試過把 period 從 5.0 拉長到 10.0，連跑 10 次 Phase N，失敗率沒有降
    # (5.0 秒是 31 次失敗 2 次，10.0 秒是 10 次失敗 2 次)，因此已還原成 5.0。
    # 量測失敗當下的時間戳可知 configure 只花 140~153 ms，根本不是設定太慢，
    # 競態發生在 lifecycle_manager 建立 client 的那一刻與 controller_server
    # 的 response writer 還沒配對之間，跟管理器第幾秒啟動無關，延長治不到它。
    # 這個延遲仍然保留，因為它擋掉的是「兩個節點同時啟動」那個更大的窗口。
    # 實車上開機自動啟動仍然需要「啟動失敗要重試」的機制，
    # 不能假設 Nav2 一定會起來 (見報告 7.12 節)。
    lifecycle_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time}]
            )
        ]
    )

    # 5. 回傳 LaunchDescription
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Full path to the Nav2 parameters file'),

        controller_server,
        lifecycle_manager,
    ])
