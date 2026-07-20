from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ws_dir = os.path.expanduser("~/Projects/esrobo")

    # 1. 启动Ranger Mini3底盘
    ranger_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ranger_bringup"), "launch", "ranger_mini_v3.launch.py")
        )
    )

    # 2. 启动RoboSense Fairy雷达
    rslidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rslidar_sdk"), "launch", "start.py")
        )
    )

    # 3. 启动静态TF变换
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ws_dir, "src/slam_launch/launch/static_tf.launch.py")
        )
    )

    # 4. Cartographer SLAM 核心
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        arguments=[
            '-configuration_directory', get_package_share_directory('cartographer_ros') + '/configuration_files',
            '-configuration_basename', 'revo_lds.lua'
        ],
        output='screen'
    )

    # 5. 地图坐标转换
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'resolution': 0.05},
            {'publish_period_sec': 1.0}
        ]
    )

    # 6. RVIZ2 可视化
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', get_package_share_directory('cartographer_ros') + '/config/demo_2d.rviz'],
        output='screen'
    )

    return LaunchDescription([
        ranger_base,
        rslidar,
        static_tf,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz2
    ])

