import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得各個套件的路徑
    bringup_dir = get_package_share_directory('mowerbot_bringup')
    description_dir = get_package_share_directory('mowerbot_description')

    # 2. 引入機器人模型 (Include description.launch.py)
    # 我們不直接跑 display，而是跑只載入模型、不帶 GUI 的 launch
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_dir, 'launch', 'robot_state_publisher.launch.py')
        )
    )

    # 3. 啟動手把驅動節點
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'dev': '/dev/input/js0'}] # 視你的設備路徑而定
    )

    # 4. 啟動你的手把組合鍵控制節點 (Teleop)
    teleop_node = Node(
        package='mowerbot_bridge',
        executable='teleop_node',
        name='mower_teleop',
        # 如果你有參數檔，放在這裡
        # parameters=[os.path.join(bringup_dir, 'config', 'teleop.yaml')]
    )

    # 5. 啟動你的大腦管理節點 (Manager)
    manager_node = Node(
        package='mowerbot_action',
        executable='manager',
        name='mower_manager'
    )

    # 6. 啟動 RViz2 (視覺化)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(description_dir, 'rviz', 'mower_view.rviz')]
    )

    return LaunchDescription([
        robot_description_launch,
        joy_node,
        teleop_node,
        manager_node,
        rviz_node
    ])