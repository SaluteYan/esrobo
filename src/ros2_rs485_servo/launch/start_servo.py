from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="servo_driver",       # 包名
            executable="servo_node",      # 可执行文件名
            name="servo_driver_node",     # 节点名
            output="screen",
            # 串口号参数（在这里改就行）
            parameters=[
                {"port": "/dev/ttyACM0"}
            ]
        )
    ])
