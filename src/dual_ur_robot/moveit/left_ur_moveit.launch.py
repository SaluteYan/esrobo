from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Remap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. 声明参数（与驱动一致）
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur7e", description="UR型号"),
        DeclareLaunchArgument("namespace", default_value="left", description="命名空间（left/right）"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false", description="是否仿真"),
    ]

    # 2. 参数变量
    ur_type = LaunchConfiguration("ur_type")
    namespace = LaunchConfiguration("namespace")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # 3. 找到 ur_moveit.launch.py 的路径
    ur_moveit_launch_path = PythonLaunchDescriptionSource(
        [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]
    )

    # 4. 命名空间分组：所有 MoveIt2 节点都在指定命名空间下
    moveit_group = GroupAction([
        # 核心：推送命名空间（如 left/right）
        PushRosNamespace(namespace),
        # 重映射关键话题/服务到命名空间
        Remap(src="/joint_states", dst=[namespace, "/joint_states"]),
        Remap(src="/scaled_joint_trajectory_controller/follow_joint_trajectory", 
              dst=[namespace, "/scaled_joint_trajectory_controller/follow_joint_trajectory"]),
        # 包含 ur_moveit.launch.py 并传递基础参数
        IncludeLaunchDescription(
            ur_moveit_launch_path,
            launch_arguments={
                "ur_type": ur_type,
                "use_fake_hardware": use_fake_hardware,
                # 禁用 MoveIt2 自带的 RViz（可选，避免重复）
                "launch_rviz": "true",
                # 指定控制器配置文件（确保指向命名空间下的控制器）
                "controllers_file": "/home/esrobo/Projects/esrobo/src/dual_ur_robot/config/left_moveit_controllers.yaml",
            }.items(),
        ),
    ])

    # 5. 组装 LaunchDescription
    ld = LaunchDescription(declared_args)
    ld.add_action(moveit_group)

    return ld
