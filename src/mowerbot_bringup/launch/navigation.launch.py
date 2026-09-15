import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 取得套件路徑與預設參數檔
    bringup_dir = get_package_share_directory('mowerbot_bringup')
    default_params = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    # 2. 宣告 Launch 參數
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # 3. Nav2 區域路徑控制器
    # 本專案只啟動 controller_server：路徑由 F2C 產生、mower_manager 直接呼叫
    # FollowPath action，不需要 planner_server / bt_navigator 等全局規劃元件。
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            # 【安全關鍵】controller_server 預設把速度指令發到 /cmd_vel，
            # 那是直接進底盤的話題，會完全繞過 mower_manager 的模式仲裁
            # 與急停(mode 4)攔截 —— 等於 Nav2 可以在急停狀態下把車開走。
            # 因此改發到 /cmd_vel_nav，強制 Nav2 的輸出必須先經過 manager
            # 審核 (只有 mode 1/3 才轉發)，再由 manager 送到 /cmd_vel。
            ('cmd_vel', 'cmd_vel_nav'),
        ]
    )

    # 4. 生命週期管理器：負責把 controller_server 帶到 active 狀態
    #
    # 延後 5 秒才啟動：autostart 會在 lifecycle_manager 一上線就立刻對
    # controller_server 送出 change_state 請求。兩個節點同時啟動時，請求送得出去、
    # 轉換也會成功，但回應有機會在 DDS 尚未完成配對前就發出，於是出現
    #   failed to send response to /controller_server/change_state (timeout)
    # lifecycle_manager 永遠等不到回應，controller_server 就卡在 inactive。
    # 先讓 controller_server 把服務註冊好再啟動管理器可以避開這個競態。
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
