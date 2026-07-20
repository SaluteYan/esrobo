from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # ========== 1. 包1：erob_canopen 单节点 ros2 run ==========
    erob_driver_node = Node(
        package="erob_canopen",
        executable="erob_driver_node",
        name="erob_driver_node",
        output="screen"
    )
    ld.add_action(erob_driver_node)

    # ========== 2. 包2：agx_arm_ctrl 嵌套launch，带参数 speed_percent:=10 ==========
    agx_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("agx_arm_ctrl"), "launch", "agx_arm_dual_launch.py")
        ),
        launch_arguments={
            "speed_percent": "10"
        }.items()
    )
    ld.add_action(agx_arm_launch)

    # ========== 3. 包3：ranger_bringup 雷达启动文件 ==========
    ranger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ranger_bringup"), "launch", "ranger_mini_v3.launch.py")
        )
    )
    ld.add_action(ranger_launch)

    # ========== 4. 包4：rslidar_sdk 激光雷达 ==========
    rslidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rslidar_sdk"), "launch", "start.py")
        )
    )
    ld.add_action(rslidar_launch)

    # ========== 5. 包5：servo_driver 舵机驱动 ==========
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("servo_driver"), "launch", "start_servo.py")
        )
    )
    ld.add_action(servo_launch)

    # ========== 6. 包6：linker_hand 夹爪双控 ==========
    linker_hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("linker_hand_ros2_sdk"), "launch", "linker_hand_double.launch.py")
        )
    )
    ld.add_action(linker_hand_launch)

    # ========== 7. 包7：kw_ft_sensor 力传感器，带端口参数 port:=/dev/rs422_ft2 ==========
    left_ft_sensor_node = Node(
        package="kw_ft_sensor_ros2",
        executable="kw_ft_sensor_node",
        namespace="left",
        name="ft_sensor_node",
        output="screen",
        parameters=[
            {"port": "/dev/rs422_ft1"},
            {"sensor_type": "serial"},
            {"long_data_format": True}
        ]
    )
    ld.add_action(left_ft_sensor_node)
    
    right_ft_sensor_node = Node(
        package="kw_ft_sensor_ros2",
        executable="kw_ft_sensor_node",
        namespace="right",
        name="ft_sensor_node",
        output="screen",
        parameters=[
            {"port": "/dev/rs422_ft2"},
            {"sensor_type": "serial"},
            {"long_data_format": True}
        ]
    )
    ld.add_action(right_ft_sensor_node)

    # ========== 8. 包8：左RealSense ==========
    #rs_left_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_left_launch.py")
    #    )
    #)
    #ld.add_action(rs_left_launch)

    # ========== 9. 包9：右RealSense ==========
    #rs_right_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(get_package_share_directory("realsense2_camera"), "launch", "rs_right_launch.py")
    #    )
    #)
    #ld.add_action(rs_right_launch)

    return ld
