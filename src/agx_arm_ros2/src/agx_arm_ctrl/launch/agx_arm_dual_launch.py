from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

os.environ["RCUTILS_COLORIZED_OUTPUT"] = "1"

def generate_launch_description():
    # ========== 全局共用参数（左右臂共用） ==========
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal).'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='true',
        choices=['true', 'false'],
        description='Automatically enable the AGX Arm node.'
    )
    fast_mode_arg = DeclareLaunchArgument(
        'fast_mode',
        default_value='false',
        choices=['true', 'false'],
        description='Enable fast mode for the AGX Arm node.'
    )
    speed_percent_arg = DeclareLaunchArgument(
        'speed_percent',
        default_value='100',
        description='Movement speed as a percentage of maximum speed.'
    )
    pub_rate_arg = DeclareLaunchArgument(
        'pub_rate',
        default_value='200',
        description='Publishing rate for the AGX Arm node.'
    )
    enable_timeout_arg = DeclareLaunchArgument(
        'enable_timeout',
        default_value='5.0',
        description='Timeout in seconds for arm enable/disable operations.'
    )
    gripper_default_effort_arg = DeclareLaunchArgument(
        'gripper_default_effort',
        default_value='1.0',
        description='Default effort for gripper commands (>= 0.0).'
    )
    publish_gripper_joint_arg = DeclareLaunchArgument(
        'publish_gripper_joint',
        default_value='true',
        choices=['true', 'false'],
        description='Publish "gripper" joint in /feedback/joint_states.'
    )
    control_enabled_arg = DeclareLaunchArgument(
        'control_enabled',
        default_value='true',
        choices=['true', 'false'],
        description='Whether to accept /control/* commands.'
    )

    # ========== 左臂独立参数 left_* ==========
    left_can_port_arg = DeclareLaunchArgument(
        'left_can_port',
        default_value='can_piper1',
        description='Left arm CAN interface (default can0)'
    )
    left_arm_type_arg = DeclareLaunchArgument(
        'left_arm_type',
        default_value='nero',
        choices=['nero', 'piper', 'piper_h', 'piper_l', 'piper_x'],
        description='Left robotic arm type'
    )
    left_effector_type_arg = DeclareLaunchArgument(
        'left_effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='Left end effector type'
    )
    left_tcp_offset_arg = DeclareLaunchArgument(
        'left_tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='Left arm TCP offset [x,y,z,r,p,y]'
    )

    # ========== 右臂独立参数 right_* ==========
    right_can_port_arg = DeclareLaunchArgument(
        'right_can_port',
        default_value='can_piper2',
        description='Right arm CAN interface (default can1)'
    )
    right_arm_type_arg = DeclareLaunchArgument(
        'right_arm_type',
        default_value='nero',
        choices=['nero', 'piper', 'piper_h', 'piper_l', 'piper_x'],
        description='Right robotic arm type'
    )
    right_effector_type_arg = DeclareLaunchArgument(
        'right_effector_type',
        default_value='none',
        choices=['none', 'agx_gripper', 'revo2'],
        description='Right end effector type'
    )
    right_tcp_offset_arg = DeclareLaunchArgument(
        'right_tcp_offset',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='Right arm TCP offset [x,y,z,r,p,y]'
    )

    # ========== 左臂节点 namespace=left ==========
    left_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_left_node',
        namespace='left',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('left_can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('left_arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('left_effector_type'),
            'tcp_offset': LaunchConfiguration('left_tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'publish_gripper_joint': LaunchConfiguration('publish_gripper_joint'),
            'control_enabled': LaunchConfiguration('control_enabled'),
        }],
        remappings=[
            ('feedback/joint_states', 'feedback/joint_states'),
            ('feedback/tcp_pose', 'feedback/tcp_pose'),
            ('feedback/arm_status', 'feedback/arm_status'),
            ('feedback/leader_joint_states', 'feedback/leader_joint_states'),
            ('feedback/gripper_status', 'feedback/gripper_status'),
            ('feedback/hand_status', 'feedback/hand_status'),
            ('control/joint_states', 'control/joint_states'),
            ('control/move_j', 'control/move_j'),
            ('control/move_p', 'control/move_p'),
            ('control/move_l', 'control/move_l'),
            ('control/move_c', 'control/move_c'),
            ('control/move_js', 'control/move_js'),
            ('control/move_mit', 'control/move_mit'),
            ('control/hand', 'control/hand'),
            ('control/hand_position_time', 'control/hand_position_time'),
            ('enable_agx_arm', 'enable_agx_arm'),
            ('control_enable', 'control_enable'),
            ('move_home', 'move_home'),
            ('emergency_stop', 'emergency_stop'),
            ('exit_teach_mode', 'exit_teach_mode'),
        ],
    )

    # ========== 右臂节点 namespace=right ==========
    right_arm_node = Node(
        package='agx_arm_ctrl',
        executable='agx_arm_ctrl_single',
        name='agx_arm_right_node',
        namespace='right',
        output='screen',
        ros_arguments=['--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'can_port': LaunchConfiguration('right_can_port'),
            'pub_rate': LaunchConfiguration('pub_rate'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'fast_mode': LaunchConfiguration('fast_mode'),
            'arm_type': LaunchConfiguration('right_arm_type'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'enable_timeout': LaunchConfiguration('enable_timeout'),
            'effector_type': LaunchConfiguration('right_effector_type'),
            'tcp_offset': LaunchConfiguration('right_tcp_offset'),
            'gripper_default_effort': LaunchConfiguration('gripper_default_effort'),
            'publish_gripper_joint': LaunchConfiguration('publish_gripper_joint'),
            'control_enabled': LaunchConfiguration('control_enabled'),
        }],
        remappings=[
            ('feedback/joint_states', 'feedback/joint_states'),
            ('feedback/tcp_pose', 'feedback/tcp_pose'),
            ('feedback/arm_status', 'feedback/arm_status'),
            ('feedback/leader_joint_states', 'feedback/leader_joint_states'),
            ('feedback/gripper_status', 'feedback/gripper_status'),
            ('feedback/hand_status', 'feedback/hand_status'),
            ('control/joint_states', 'control/joint_states'),
            ('control/move_j', 'control/move_j'),
            ('control/move_p', 'control/move_p'),
            ('control/move_l', 'control/move_l'),
            ('control/move_c', 'control/move_c'),
            ('control/move_js', 'control/move_js'),
            ('control/move_mit', 'control/move_mit'),
            ('control/hand', 'control/hand'),
            ('control/hand_position_time', 'control/hand_position_time'),
            ('enable_agx_arm', 'enable_agx_arm'),
            ('control_enable', 'control_enable'),
            ('move_home', 'move_home'),
            ('emergency_stop', 'emergency_stop'),
            ('exit_teach_mode', 'exit_teach_mode'),
        ],
    )

    return LaunchDescription([
        # 全局共用参数
        log_level_arg, auto_enable_arg, fast_mode_arg, speed_percent_arg,
        pub_rate_arg, enable_timeout_arg, gripper_default_effort_arg,
        publish_gripper_joint_arg, control_enabled_arg,
        # 左臂独有参数
        left_can_port_arg, left_arm_type_arg, left_effector_type_arg, left_tcp_offset_arg,
        # 右臂独有参数
        right_can_port_arg, right_arm_type_arg, right_effector_type_arg, right_tcp_offset_arg,
        # 启动双臂
        left_arm_node, right_arm_node
    ])
