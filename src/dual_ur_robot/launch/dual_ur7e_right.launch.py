# dual_ur7e.launch.py
# 同时启动两台UR7e机器人
# left  机器人 IP: 192.168.91.100
# right 机器人 IP: 192.168.91.101

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    ThisLaunchFileDir,
)
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # 参数声明
    # ============================================================
    declared_args = [
        DeclareLaunchArgument(
            "left_ip",
            default_value="192.168.91.100",
            description="IP address of LEFT UR7e robot",
        ),
        DeclareLaunchArgument(
            "right_ip",
            default_value="192.168.91.101",
            description="IP address of RIGHT UR7e robot",
        ),
        DeclareLaunchArgument(
            "left_tf_prefix",
            default_value="left_",
            description="TF prefix for left robot",
        ),
        DeclareLaunchArgument(
            "right_tf_prefix",
            default_value="right_",
            description="TF prefix for right robot",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for testing without real robot",
        ),
    ]

    # ============================================================
    # 路径定义
    # ============================================================
    #ur_control_launch = PathJoinSubstitution(
    #    [FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"]
    #)

    ur_moveit_launch = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py"]
    )
    
    # 1. 定义手动启动 spawner 的函数
    def get_spawner_node(ns, controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            #namespace=ns,
            # 关键：这里直接指定 controller-manager 的服务地址
            arguments=[
                controller_name, 
                "--controller-manager", f"/right/controller_manager"
            ],
            output="screen",
        )

    # ============================================================
    # LEFT 机器人 (192.168.91.100)
    # 端口组: 50001 ~ 50004
    # ============================================================
    left_driver = GroupAction(
        actions=[
            LogInfo(msg="========== Starting LEFT UR7e (192.168.91.100) =========="),
            PushRosNamespace("right"),
            IncludeLaunchDescription(
                #PythonLaunchDescriptionSource(ur_control_launch),
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/right_ur_control.launch.py"]),
                launch_arguments={
                    "ur_type":               "ur7e",
                    "robot_ip":              LaunchConfiguration("right_ip"),
                    "tf_prefix":             LaunchConfiguration("right_tf_prefix"),
                    "use_fake_hardware":     LaunchConfiguration("use_fake_hardware"),
                    "launch_rviz":           "false",
                    "controllers_file": "/home/esrobo/Projects/esrobo/src/dual_ur_robot/config/right_ur_controllers.yaml",
                    "load_controllers": "false",          # 禁用默认spawner
            		"start_joint_controller": "false",    # 禁用关节控制器自动启动
            		"activate_joint_controller": "false", # 禁用控制器自动激活
                    # 端口组1
                    "reverse_port":          "50011",
                    "script_sender_port":    "50012",
                    "trajectory_port":       "50013",
                    "script_command_port":   "50014",
                }.items(),
                
            ),
            
            # 手动启动 spawner，确保它连接到 /left/controller_manager
            #get_spawner_node("right", "joint_state_broadcaster"),
            get_spawner_node("right", "scaled_joint_trajectory_controller"),
        ]
    )
    
    # 手动启动 spawner，确保它连接到 /left/controller_manager
	#get_spawner_node("left", "joint_state_broadcaster1"),
	#get_spawner_node("left", "scaled_joint_trajectory_controller"),


    return LaunchDescription(
        declared_args + [
            left_driver,
        ]
    )

