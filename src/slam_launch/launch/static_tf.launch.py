from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 静态TF: base_link -> laser_link
    # x/y/z: 雷达相对底盘中心偏移  roll/pitch/yaw: 姿态角
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            "0.12", "0.0", "0.25",   # x y z 偏移（根据实际安装修改）
            "0", "0", "0",           # 翻滚角 俯仰角 偏航角
            "base_link",             # 父坐标系（底盘）
            "laser_link"             # 子坐标系（雷达）
        ]
    )

    return LaunchDescription([static_tf_node])

