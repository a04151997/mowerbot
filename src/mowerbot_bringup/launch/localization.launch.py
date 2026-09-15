import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """slam_toolbox 定位模式。

    建圖模式 (mapping.launch.py / async_slam_toolbox_node) 會一邊跑一邊改地圖，
    割草時 Nav2 的 costmap 會跟著漂。真實流程是
    建圖 -> 存檔 -> 切定位模式 -> 割草，這支 launch 負責最後兩步的「切定位模式」。

    使用前要先存過圖 (見 mowerbot_bringup/maps/README.md)：
        ros2 run mowerbot_bringup save_map.sh
    """
    pkg_path = get_package_share_directory('mowerbot_bringup')
    params_file = os.path.join(pkg_path, 'config', 'localization_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file_name = LaunchConfiguration('map_file_name')

    # 注意是 localization_slam_toolbox_node，不是建圖用的 async_slam_toolbox_node。
    localization_node = Node(
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                # 序列化地圖路徑，不帶副檔名
                'map_file_name': map_file_name,
            }
        ],
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'map_file_name',
            # 預設指向套件安裝目錄裡的地圖，不帶副檔名；
            # slam_toolbox 會自己補上 .posegraph 與 .data
            default_value=os.path.join(pkg_path, 'maps', 'mowerbot_map'),
            description='Serialized map path WITHOUT extension '
                        '(slam_toolbox appends .posegraph / .data)'),

        localization_node
    ])
