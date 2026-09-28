"""One-shot, feedback-based operator commands for the separate ROS waist driver.

The arm gateway assumes a fixed waist pose. The encoder baseline below was
measured at the operator-confirmed upright URDF neutral pose. This module only
offers small, slow commissioning moves; the laptop console enforces the arm
interlock before invoking it.  No command is sent on import or status reads.
"""

from __future__ import annotations

import argparse
import json
import math
import time


# Encoder readings at the operator-confirmed upright URDF zero pose.
# Recheck these encoder readings after any encoder battery/zero-point service.
REFERENCE_DEG = {31: 139.0, 32: 215.0, 33: 144.0}
MAX_OFFSET_DEG = 5.0
SINGLE_TURN_MIN_DEG = 0.0
SINGLE_TURN_MAX_DEG = 360.0
STEP_DEG = 1.0
MAX_JOG_SPEED_DEG_S = 5.0
JOG_ACCEL_DEG_S2 = 12.0
TARGET_SPEED_DEG_S = 4.0
TARGET_ACCEL_DEG_S2 = 10.0
# The waist is intentionally operated within one turn.  This code reports a
# missing/low multi-turn encoder backup battery; keep it visible in feedback.
SINGLE_TURN_BATTERY_WARNING = 0x0C26


def waist_error_allows_motion(code):
    return code in (0, SINGLE_TURN_BATTERY_WARNING)


def parse_feedback(data, errors):
    """Reject incomplete or malformed feedback; retain the full motor error code."""
    if len(data) != 15 or len(errors) != 6:
        raise ValueError("腰部反馈缺少三个关节或错误码")
    axes = {}
    for i in range(0, len(data), 5):
        group = data[i:i + 5]
        axis = int(group[0])
        if group[0] != axis or axis not in REFERENCE_DEG or axis in axes:
            raise ValueError("腰部关节编号无效或重复")
        if not all(math.isfinite(float(value)) for value in group):
            raise ValueError("腰部反馈包含非有限数值")
        axes[axis] = dict(position_deg=float(group[1]), velocity=float(group[2]),
                          current=float(group[3]), temperature=float(group[4]))
    faults = {}
    for i in range(0, len(errors), 2):
        axis = int(errors[i])
        if errors[i] != axis or axis not in axes or axis in faults:
            raise ValueError("腰部错误码关节编号无效或重复")
        code = int(errors[i + 1])
        if errors[i + 1] != code or not 0 <= code <= 0xFFFFFFFF:
            raise ValueError("腰部错误码无效")
        faults[axis] = code
    if len(axes) != 3 or len(faults) != 3:
        raise ValueError("腰部三轴反馈不完整")
    for axis in axes:
        axes[axis]['error'] = faults[axis]
    return axes


def check_faults(axes):
    faults = [(axis, value['error']) for axis, value in axes.items()
              if not waist_error_allows_motion(value['error'])]
    if faults:
        detail = '、'.join(f'{axis}:0x{code:04X}' for axis, code in faults)
        raise RuntimeError(f'腰部错误码 {detail}；排除故障后才能运动')


def checked_absolute_target(axes, axis, target_deg):
    if type(axis) is not int or axis not in REFERENCE_DEG:
        raise ValueError("腰部关节编号无效")
    if type(target_deg) not in (int, float) or not math.isfinite(target_deg):
        raise ValueError("腰部目标角度无效")
    check_faults(axes)
    for joint in REFERENCE_DEG:
        position = axes[joint]['position_deg']
        if not math.isfinite(position) or not SINGLE_TURN_MIN_DEG <= position <= SINGLE_TURN_MAX_DEG:
            raise RuntimeError(f"腰部 {joint} 号反馈超出单圈范围")
    if not SINGLE_TURN_MIN_DEG <= target_deg <= SINGLE_TURN_MAX_DEG:
        raise RuntimeError(f"腰部 {axis} 号目标超出单圈范围")
    # Match the 0.01° feedback displayed by the operator console so its
    # visible ±5° input bounds and the robot-side check agree.
    displayed_position = math.floor(axes[axis]['position_deg'] * 100 + 0.5) / 100
    if abs(target_deg - displayed_position) > MAX_OFFSET_DEG + 1e-6:
        raise RuntimeError(f"腰部 {axis} 号目标超出当前反馈 ±{MAX_OFFSET_DEG:g}° 操作范围")
    if abs(target_deg - axes[axis]['position_deg']) < 0.05:
        raise RuntimeError(f"腰部 {axis} 号已在目标角度附近")
    return float(target_deg)


def checked_jog_target(axes, axis, direction):
    if type(axis) is not int or axis not in REFERENCE_DEG or type(direction) is not int or direction not in (-1, 1):
        raise ValueError("腰部小步参数无效")
    target = axes[axis]['position_deg'] + direction * STEP_DEG
    return checked_absolute_target(axes, axis, target)


def run(action, axis=None, direction=None, target_deg=None, command_state=None):
    if command_state is None:
        command_state = {}
    command_state['sent'] = False
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import Bool, Empty, Float32MultiArray, UInt32MultiArray

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('esrobo_waist_operator')
    state = {'data': None, 'errors': None, 'state_time': 0.0, 'error_time': 0.0}

    def on_state(message):
        state['data'] = list(message.data)
        state['state_time'] = time.monotonic()

    def on_errors(message):
        state['errors'] = list(message.data)
        state['error_time'] = time.monotonic()

    node.create_subscription(Float32MultiArray, '/erobo/joint_states', on_state, 10)
    node.create_subscription(UInt32MultiArray, '/erobo/joint_errors', on_errors, 10)
    topics = {
        'jog': node.create_publisher(Float32MultiArray, '/erobo/pos_target', 10),
        'target': node.create_publisher(Float32MultiArray, '/erobo/pos_target', 10),
        'stop': node.create_publisher(Empty, '/erobo/stop', 10),
        'enable': node.create_publisher(Bool, '/erobo/enable', 10),
    }
    try:
        if action != 'stop':
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if (state['data'] is not None and state['errors'] is not None
                        and time.monotonic() - state['state_time'] < 0.5
                        and time.monotonic() - state['error_time'] < 0.5):
                    break
        axes = None
        if state['data'] is not None and state['errors'] is not None:
            if (time.monotonic() - state['state_time'] < 0.5
                    and time.monotonic() - state['error_time'] < 0.5):
                axes = parse_feedback(state['data'], state['errors'])
        if action == 'status':
            if axes is None:
                raise RuntimeError('腰部三轴 ROS 反馈或错误码未更新；检查腰部驱动')
            return dict(axes=axes, reference_deg=REFERENCE_DEG,
                        limit_offset_deg=MAX_OFFSET_DEG,
                        single_turn_min_deg=SINGLE_TURN_MIN_DEG,
                        single_turn_max_deg=SINGLE_TURN_MAX_DEG)
        publisher = topics[action]
        # A fresh one-shot ROS node can take several seconds to discover the
        # already-running waist driver, especially after the dashboard restarts.
        connection_deadline = time.monotonic() + 6.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < connection_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError('腰部驱动未订阅控制话题，未发送命令')
        if action in ('jog', 'target'):
            if axes is None:
                raise RuntimeError('腰部三轴反馈不新鲜，拒绝运动')
            target = (checked_jog_target(axes, axis, direction) if action == 'jog'
                      else checked_absolute_target(axes, axis, target_deg))
            acceleration = JOG_ACCEL_DEG_S2 if action == 'jog' else TARGET_ACCEL_DEG_S2
            speed = MAX_JOG_SPEED_DEG_S if action == 'jog' else TARGET_SPEED_DEG_S
            message = Float32MultiArray()
            message.data = [float(axis), target, acceleration, speed]
            # If publish or any later step raises, treat the motion as possibly
            # issued. Only errors before this point can clear a pending target.
            command_state['sent'] = True
            publisher.publish(message)
            result = dict(axis=axis, from_deg=axes[axis]['position_deg'], target_deg=target,
                          speed_deg_s=speed, accel_deg_s2=acceleration,
                          message='腰部目标已发送；请核对后续实测反馈')
        elif action == 'enable':
            if axes is None:
                raise RuntimeError('腰部三轴反馈不新鲜，拒绝使能')
            check_faults(axes)
            if any(not math.isfinite(axes[joint]['position_deg'])
                   or not SINGLE_TURN_MIN_DEG <= axes[joint]['position_deg'] <= SINGLE_TURN_MAX_DEG
                   for joint in REFERENCE_DEG):
                raise RuntimeError('腰部位置超出单圈范围，拒绝使能')
            command_state['sent'] = True
            publisher.publish(Bool(data=True))
            result = dict(message='腰部三轴使能请求已发送')
        elif action == 'stop':
            command_state['sent'] = True
            publisher.publish(Empty())
            result = dict(message='腰部三轴停止请求已发送；请核对实际运动状态')
        else:
            raise ValueError('未知腰部操作')
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('status', 'jog', 'target', 'stop', 'enable'))
    parser.add_argument('--axis', type=int)
    parser.add_argument('--direction', type=int)
    parser.add_argument('--target-deg', type=float)
    args = parser.parse_args(argv)
    command_state = {'sent': False}
    try:
        print(json.dumps(dict(ok=True, **run(args.action, args.axis, args.direction,
                                             args.target_deg, command_state)), ensure_ascii=False))
    except Exception as exc:
        print(json.dumps(dict(ok=False, error=str(exc), command_sent=command_state['sent']), ensure_ascii=False))
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
