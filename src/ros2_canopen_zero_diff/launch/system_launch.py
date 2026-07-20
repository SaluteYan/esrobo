
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='zero_diff_canopen_driver').find('zero_diff_canopen_driver')
    
    # CANopen 设备容器节点 (负责底层CAN通信和状态机管理)
    canopen_node = Node(
        package='canopen_core',
        executable='device_container',
        name='device_container',
        namespace='',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'canopen_config.yaml'])
        ],
    )
    
    # ROS2 Control 控制器管理器
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'ros2_control_config.yaml'])
        ],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output='screen',
    )
    
    # 加载并启动控制器
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        canopen_node,
        controller_manager,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
    ])
