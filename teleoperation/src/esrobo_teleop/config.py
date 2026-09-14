"""Configuration dataclasses for the ESROBO real-machine teleoperation node.

All values mirror the reference ``Dual-arm-teleoperation`` defaults so the
real-robot behaviour matches the IsaacLab bring-up, with the ROS2/PhysX layers
replaced by direct ``pyAgxArm`` / ``LinkerHand`` calls.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotConfig:
    """Physical robot wiring and safety limits."""

    left_can_channel: str = "can_piper1"      # NERO left arm
    right_can_channel: str = "can_piper2"     # NERO right arm
    nero_firmware: str = "V112"               # default/left-arm firmware
    right_nero_firmware: str = "DEFAULT"      # right arm publishes legacy 0x501..0x507 feedback
    can_interface: str = "socketcan"
    can_bitrate: int = 1000000
    speed_percent: int = 25                   # applies to the safe move_j mode
    command_mode: str = "j"                   # "j" (smoothed) | "js" (unsafe passthrough)
    allow_unsafe_js: bool = False
    # physical = direction * URDF + offset. The J2 +pi/2 offset follows the
    # pyAgxArm NERO limits and this project's URDF limits exactly.
    left_joint_directions: tuple = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    # Both installed arms use the SDK-positive direction for all seven axes.
    right_joint_directions: tuple = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    left_joint_offsets: tuple = (0.0, 1.5707963267948966, 0.0, 0.0, 0.0, 0.0, 0.0)
    right_joint_offsets: tuple = (0.0, 1.5707963267948966, 0.0, 0.0, 0.0, 0.0, 0.0)
    # SDK NERO physical limits (rad), with a soft margin applied by the driver.
    joint_lower_limits: tuple = (
        -2.705261, -1.745330, -2.757621, -1.012291, -2.757621, -0.733039, -1.570797,
    )
    joint_upper_limits: tuple = (
        2.705261, 1.745330, 2.757621, 2.146755, 2.757621, 0.959932, 1.570797,
    )
    joint_limit_margin: float = 0.05
    # Initial full-arm commissioning envelope in URDF coordinates.  These
    # deliberately asymmetric limits sit inside the SDK hard limits and are
    # converted to each arm's physical convention by the driver.
    joint_soft_lower_limits_urdf: tuple = (
        -1.047198, -1.308997, -1.047198, -0.139626, -1.047198, -0.523599, -0.959931,
    )
    joint_soft_upper_limits_urdf: tuple = (
        1.047198, 0.122173, 1.047198, 1.483530, 1.047198, 0.610865, 0.959931,
    )
    # Initial full-arm commissioning rate limits for J1..J7.  Scalars remain
    # accepted by the driver for compatibility with older configuration files.
    max_joint_step: tuple = (
        0.038397, 0.038397, 0.038397, 0.043633, 0.043633, 0.043633, 0.043633,
    )
    max_joint_velocity: tuple = (
        0.314159, 0.314159, 0.314159, 0.349066, 0.349066, 0.349066, 0.349066,
    )
    max_joint_acceleration: tuple = (
        0.785398, 0.785398, 0.872665, 1.047198, 1.047198, 1.047198, 1.047198,
    )
    # Second-stage full-arm commissioning limits, shared by both sides.
    left_max_joint_step: Optional[tuple] = (
        0.038397, 0.038397, 0.038397, 0.043633, 0.043633, 0.043633, 0.043633,
    )
    left_max_joint_velocity: Optional[tuple] = (
        0.314159, 0.314159, 0.314159, 0.349066, 0.349066, 0.349066, 0.349066,
    )
    left_max_joint_acceleration: Optional[tuple] = (
        0.785398, 0.785398, 0.872665, 1.047198, 1.047198, 1.047198, 1.047198,
    )
    # Match the left-arm pilot envelope while commissioning the right arm.
    right_max_joint_step: Optional[tuple] = (
        0.038397, 0.038397, 0.038397, 0.043633, 0.043633, 0.043633, 0.043633,
    )
    right_max_joint_velocity: Optional[tuple] = (
        0.314159, 0.314159, 0.314159, 0.349066, 0.349066, 0.349066, 0.349066,
    )
    right_max_joint_acceleration: Optional[tuple] = (
        0.785398, 0.785398, 0.872665, 1.047198, 1.047198, 1.047198, 1.047198,
    )
    synchronize_joint_motion: bool = False
    joint_sync_error_deadband_deg: float = 0.05
    # Legacy optional limit relative to the angles captured when armed.  Keep
    # disabled when absolute per-joint URDF soft limits are configured.
    max_joint_deviation_from_start: float = 0.0
    # Hand-IMU pilot mode: only the terminal NERO joints 5, 6 and 7 may move.
    right_wrist_joint_indices: tuple = (4, 5, 6)
    wrist_neutral_joint_positions: tuple = (0.0, 0.0, 0.0)
    right_wrist_max_angle_deg: float = 40.0
    wrist_return_timeout_s: float = 10.0
    wrist_return_tolerance_deg: float = 1.0
    return_feedback_grace_s: float = 1.0
    return_verify_samples: int = 3
    return_phase_attempts: int = 2
    disable_on_unconfirmed_return: bool = False
    right_wrist_max_step: float = 0.004
    right_wrist_max_velocity: float = 0.1396263  # 8 deg/s
    right_wrist_max_acceleration: float = 0.2792527  # 16 deg/s^2
    left_arm_disable_on_connect: bool = True
    right_arm_disable_on_connect: bool = True
    # The installed left V1.12 controller stops CAN push while power-on
    # disabled. In single-arm commissioning, defer its first enable/zero check
    # until the operator explicitly presses "e" and supports the arm.
    left_arm_allow_silent_disabled_startup: bool = True
    # Single-arm PICO commissioning starts only when all seven joints match the
    # URDF natural-down zero (after applying the SDK joint offsets).
    full_arm_startup_zero_tolerance_deg: float = 8.0
    full_arm_return_timeout_s: float = 30.0
    full_arm_return_tolerance_deg: float = 1.5

    @property
    def firmware_enum(self):
        from pyAgxArm import NeroFW
        return getattr(NeroFW, self.nero_firmware, NeroFW.DEFAULT)

    def firmware_enum_for(self, side: str):
        from pyAgxArm import NeroFW

        name = self.right_nero_firmware if side == "right" else self.nero_firmware
        return getattr(NeroFW, name, NeroFW.DEFAULT)


@dataclass
class RetargetConfig:
    """PICO full-body -> robot end-effector retargeting parameters."""

    host: str = "0.0.0.0"
    port: int = 15050
    max_packet_bytes: int = 65535
    max_stale_time_s: float = 0.5
    retargeting_mode: str = "arm_vector"          # arm_vector | wrist_delta
    # Natural-down calibration applies only the minimum rotation needed to
    # align the upper-arm zero direction; it must not redefine transverse axes.
    arm_vector_position_mode: str = "segment_direction_relative"
    position_delta_signs: tuple = (1.0, 1.0, 1.0)
    position_scale: float = 1.0
    source_to_robot_rotation: tuple = (
        0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    )
    swap_left_right_targets: bool = False
    enable_elbow_ik_targets: bool = True
    require_calibration: bool = False
    auto_start_reference: bool = True
    auto_start_reference_prepare_s: float = 5.0
    auto_start_reference_delay_s: float = 4.0
    auto_start_reference_sample_start_s: float = 1.0
    auto_start_reference_max_position_std_m: float = 0.05
    auto_start_reference_min_samples: int = 5
    auto_start_reference_require_waist: bool = True
    calibration_delay_s: float = 10.0
    calibration_sample_start_s: float = 6.0
    arm_vector_max_reach: float = 0.72
    arm_vector_prediction_horizon_s: float = 0.02
    arm_vector_prediction_lookback_s: float = 0.03
    arm_vector_prediction_max_velocity_m_s: float = 2.5
    arm_vector_prediction_max_displacement_m: float = 0.035
    max_endpoint_translation_velocity_m_s: float = 0.18
    upper_arm_angular_deadband_deg: float = 0.25
    forearm_angular_deadband_deg: float = 0.30
    use_hand_imu_orientation: bool = True
    # Full-arm PICO control uses shoulder/elbow/wrist positions only.  Require a
    # calibrated glove IMU for terminal orientation instead of falling back to
    # the PICO wrist quaternion when the glove stream is absent.
    require_hand_imu_for_active_arm: bool = False
    # Never silently substitute PICO's wrist quaternion for a missing glove.
    # With no glove, partitioned full-arm mode holds J5..J7 at feedback.
    allow_pico_wrist_orientation_fallback: bool = False
    hand_imu_max_angle_deg: float = 45.0
    hand_imu_max_step_deg: float = 2.0
    # Calibration removes only orientation offset. The relative rotation stays
    # in the glove's anatomical X/Y/Z axes. Per-side overrides account for
    # the mirrored physical wrist directions.
    hand_imu_input_frame: str = "senseglove_zeroed_anatomical_axes"
    hand_imu_local_rotvec_to_robot_hand: tuple = (
        0.0, 1.0, 0.0,
        0.0, 0.0, -1.0,
        1.0, 0.0, 0.0,
    )
    # Per-side overrides keep the two measured wrist directions independent.
    # The left J7 sign follows the latest isolated flexion/extension check.
    left_hand_imu_local_rotvec_to_robot_hand: Optional[tuple] = (
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
    )
    right_hand_imu_local_rotvec_to_robot_hand: Optional[tuple] = (
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
        1.0, 0.0, 0.0,
    )
    # Track each hand's anatomical axes by integrating accepted body-local frame
    # increments. Separate switches preserve compatibility with existing files.
    left_hand_imu_xyz_decomposition: bool = True
    right_hand_imu_xyz_decomposition: bool = True
    # Retained for legacy UDP producers that still publish a world-frame delta.
    hand_imu_source_to_robot_rotation: Optional[tuple] = (
        0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0,
    )
    hand_joint_count: int = 20
    packet_quaternion_order: str = "wxyz"

    # Robot kinematic reference (world frame, base at waist_link3).
    initial_left_wrist_pose: tuple = (
        -0.14664, 0.24538, 0.81949, 0.0, 0.70711, -0.70710, 0.0,
    )
    initial_right_wrist_pose: tuple = (
        -0.14680, -0.23480, 0.81949, 0.0, -0.70710, -0.70711, 0.0,
    )
    initial_left_elbow_pose: tuple = (
        -0.146645, 0.245585, 1.219004, 0.0, 0.70711, -0.70710, 0.0,
    )
    initial_right_elbow_pose: tuple = (
        -0.146802, -0.235014, 1.219003, 0.0, -0.70710, -0.70711, 0.0,
    )
    robot_left_shoulder_position: tuple = (-0.146646, 0.245584, 1.529004)
    robot_right_shoulder_position: tuple = (-0.146803, -0.235015, 1.529003)
    robot_left_elbow_position: tuple = (-0.146645, 0.245585, 1.219004)
    robot_right_elbow_position: tuple = (-0.146802, -0.235014, 1.219003)
    robot_left_reference_rotation: tuple = (
        0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    )
    robot_right_reference_rotation: tuple = (
        0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    )

    def hand_imu_local_rotvec_map_for(self, side: str):
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        override = (
            self.left_hand_imu_local_rotvec_to_robot_hand
            if side == "left"
            else self.right_hand_imu_local_rotvec_to_robot_hand
        )
        return self.hand_imu_local_rotvec_to_robot_hand if override is None else override


@dataclass
class IkConfig:
    """Pink IK solver settings over the full ESROBO URDF."""

    urdf_path: str = ""
    base_link_name: str = "waist_link3"
    left_shoulder_frame: str = "left_nero_link1"
    left_elbow_frame: str = "left_nero_link3"
    left_hand_frame: str = "leftHand_link"
    right_shoulder_frame: str = "right_nero_link1"
    right_elbow_frame: str = "right_nero_link3"
    right_hand_frame: str = "rightHand_link"
    left_arm_joints: list = field(
        default_factory=lambda: [
            "leftArm_joint", "left_nero_joint2", "left_nero_joint3", "left_nero_joint4",
            "left_nero_joint5", "left_nero_joint6", "left_nero_joint7",
        ]
    )
    right_arm_joints: list = field(
        default_factory=lambda: [
            "rightArm_joint", "right_nero_joint2", "right_nero_joint3", "right_nero_joint4",
            "right_nero_joint5", "right_nero_joint6", "right_nero_joint7",
        ]
    )
    dt: float = 0.02
    position_cost: float = 32.0
    orientation_cost: float = 0.05
    elbow_position_cost: float = 80.0
    elbow_orientation_cost: float = 0.0
    lm_damping: float = 0.012
    elbow_lm_damping: float = 0.006
    gain: float = 1.0
    iterations_per_cycle: int = 1
    enable_elbow_tasks: bool = True
    # In single-arm commissioning, solve human upper/forearm positions with
    # J1..J4 and reserve J5..J7 for the calibrated SenseGlove orientation.
    partition_terminal_wrist_ik: bool = True
    # A NERO J4 angle is the geometric bend between the upper-arm and forearm
    # segments. Set it explicitly so Cartesian IK cannot absorb elbow flexion
    # into the shoulder joints J1..J3.
    partition_elbow_flexion_from_segments: bool = True
    task_error_tolerance: float = 1.0e-4
    max_command_lead: float = 0.50
    max_command_step: float = 0.0
    max_joint_position_delta: float = 0.50
    clamp_to_limits: bool = True


@dataclass
class HandConfig:
    """LinkerHand command publishing with safety limits.

    The physical LinkerHand is commanded through ``/cb_{left|right}_hand_control_cmd``
    (``sensor_msgs/JointState``) with **0..255 integer servo positions**.  The
    teleop pipeline produces per-hand joint angles in radians (10 active joints
    per hand); this driver maps rad -> 0..255 and applies safety clamps.
    """

    mode: str = "udp"                       # "udp" | "ros2"
    udp_host: str = "127.0.0.1"
    udp_port: int = 15051
    feedback_udp_host: str = "127.0.0.1"
    feedback_udp_port: int = 15052
    left_topic: str = "/cb_left_hand_control_cmd"
    right_topic: str = "/cb_right_hand_control_cmd"
    hand_joint_count: int = 20              # teleop packet: 10 left + 10 right active
    publish_hz: float = 50.0
    max_stale_time_s: float = 0.5
    require_feedback_on_enable: bool = True
    # The installed L20Lite hand is controlled through the SDK's 10-actuator
    # "L10" interface; its full URDF expands passive joints with mimic relations.
    # "L20" remains available for SDKs that expose all 20 values directly.
    model: str = "L10"
    # rad -> 0..255 linear map per active joint.
    # Optional lists of 10 [scale, offset]; empty lists fall back to defaults
    # computed from the ESROBO hand URDF joint limits (offset + scale, value =
    # clamp(round(rad * scale + offset))).
    rad_scale: list = field(default_factory=list)
    rad_offset: list = field(default_factory=list)

    # ---- safety limits (0..255 servo units) ----
    enable_on_start: bool = False           # UI default; feedback gate still applies
    out_min: int = 0
    out_max: int = 255
    max_step: int = 18                      # max per-command change per joint
    max_velocity: int = 400                 # max servo units / second
    home: list = field(default_factory=list)  # safe open pose (0..255) per physical joint
    return_open_on_quit: bool = True
    return_open_duration_s: float = 1.5
    startup_open_tolerance: float = 12.0
    startup_open_verify_timeout_s: float = 2.0
    # Right L10 endpoints are SDK-documented; enable all three thumb axes plus flexion.
    enabled_physical_joints: list = field(default_factory=lambda: [0, 1, 2, 3, 4, 5, 9])
    left_open: list = field(
        default_factory=lambda: [255, 255, 255, 255, 255, 255, 128, 128, 128, 128]
    )
    left_closed: list = field(
        default_factory=lambda: [0, 0, 0, 0, 0, 0, 128, 128, 128, 128]
    )
    right_open: list = field(
        default_factory=lambda: [255, 210, 255, 255, 255, 255, 110, 110, 120, 33]
    )
    # Optional measured sensor values at the physical open pose.  Some L10
    # actuators use different command and feedback calibrations.
    left_open_feedback: list = field(default_factory=list)
    right_open_feedback: list = field(default_factory=list)
    right_closed: list = field(
        default_factory=lambda: [73, 75, 0, 0, 0, 0, 110, 110, 120, 78]
    )
    # Optional teleoperation-only closed endpoints.  These let teleoperation
    # use a wider verified actuator range without changing the safe open pose
    # used during startup and shutdown.
    left_teleop_closed: list = field(default_factory=list)
    right_teleop_closed: list = field(
        default_factory=lambda: [40, 40, 0, 0, 0, 0, 110, 110, 120, 100]
    )


@dataclass
class TeleopConfig:
    robot: RobotConfig = field(default_factory=RobotConfig)
    retarget: RetargetConfig = field(default_factory=RetargetConfig)
    ik: IkConfig = field(default_factory=IkConfig)
    hand: HandConfig = field(default_factory=HandConfig)


def build_config() -> TeleopConfig:
    return TeleopConfig()


def load_config(path: str) -> TeleopConfig:
    """Load a ``teleop_config.yaml`` into a :class:`TeleopConfig`.

    Only keys present in the YAML override the defaults.  The top-level
    sections ``robot`` / ``retarget`` / ``ik`` / ``hand`` map onto the
    corresponding dataclasses.
    """
    import dataclasses

    import yaml

    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    cfg = build_config()
    for section in ("robot", "retarget", "ik", "hand"):
        payload = data.get(section)
        if not isinstance(payload, dict):
            continue
        target = getattr(cfg, section)
        fields = {f.name for f in dataclasses.fields(target)}
        for key, value in payload.items():
            if key in fields:
                setattr(target, key, value)
    return cfg
