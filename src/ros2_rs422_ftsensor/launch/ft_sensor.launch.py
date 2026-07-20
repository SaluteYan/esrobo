from launch import LaunchDescription
from launch_ros.actions import Node
import os


def port_or_fallback(alias, fallback):
    return alias if os.path.exists(alias) else fallback

def generate_launch_description():
    left_port = port_or_fallback("/dev/rs422_ft1", "/dev/ttyUSB0")
    right_port = port_or_fallback("/dev/rs422_ft2", "/dev/ttyUSB1")

    return LaunchDescription([
        # ==========================
        # 左侧力传感器
        # ==========================
        Node(
            package="kw_ft_sensor_ros2",
            executable="kw_ft_sensor_node",
            namespace="left",
            name="ft_sensor_node",
            output="screen",
            parameters=[
                {"port": left_port},
                {"sensor_type": "serial"},
                {"long_data_format": True}
            ]
        ),

        # ==========================
        # 右侧力传感器
        # ==========================
        Node(
            package="kw_ft_sensor_ros2",
            executable="kw_ft_sensor_node",
            namespace="right",
            name="ft_sensor_node",
            output="screen",
            parameters=[
                {"port": right_port},
                {"sensor_type": "serial"},
                {"long_data_format": True}
            ]
        ),
    ])
