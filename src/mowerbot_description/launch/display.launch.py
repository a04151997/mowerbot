import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'mowerbot_description'
    # 取得 xacro 檔案路徑
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'car.xacro')
    # 解析 xacro 轉成 urdf 內容
    robot_description_config = xacro.process_file(xacro_file).toxml()

    return LaunchDescription([
        # 發布靜態 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),
        # 啟動虛擬關節控制面板 (讓輪子可以手動轉動測試)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        # 啟動 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])