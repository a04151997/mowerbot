import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 取得相關路徑
    pkg_description = get_package_share_directory('mowerbot_description')
    pkg_bringup = get_package_share_directory('mowerbot_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # world 檔放在 mowerbot_bringup/worlds 底下
    world_file = os.path.join(pkg_bringup, 'worlds', 'mow_field.world')

    # 2. 定義參數：是否啟動模擬時間 (在 Gazebo 裡必須為 True)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 是否開啟 Gazebo GUI (gzclient)。自動化測試時用 gui:=false 跑無頭模式比較快
    gui = LaunchConfiguration('gui', default='true')

    # 3. 引入 robot_state_publisher (發布 URDF)
    # 這裡直接呼叫你之前寫好的描述檔 launch
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 4. 啟動 Gazebo 伺服器 (gzserver) 與 客戶端 (gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file, 'gui': gui}.items()
    )

    # 5. 呼叫 gazebo_ros 的節點來「生成」機器人
    # 它會從 robot_description 話題讀取 URDF 並放入 Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'mowerbot',
                   '-x', '0', '-y', '0', '-z', '0.03'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Start the Gazebo GUI (gzclient). Set false for headless runs.'
        ),
        robot_state_publisher,
        gazebo,
        spawn_entity
    ])