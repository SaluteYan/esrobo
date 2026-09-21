from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/serial/by-path/pci-0000:00:14.0-usb-0:5.2:1.0'),
        DeclareLaunchArgument('allow_motion', default_value='false'),
        Node(
            package="servo_driver",       # 包名
            executable="servo_node",      # 可执行文件名
            name="servo_driver_node",     # 节点名
            output="screen",
            # 串口号参数（在这里改就行）
            parameters=[
                {"port": LaunchConfiguration('port'),
                 "allow_motion": ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool)}
            ]
        )
    ])
