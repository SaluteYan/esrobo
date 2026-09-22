"""Integration template, imported by the future laptop PICO/SenseGlove loop.

No PICO/glove acquisition or robot enable happens on import. Match the measured
pose while the operator enables; retarget from that pose once mode is ACTIVE.
"""
import time


def send_solved_frame(client, arm_urdf_rad, hand_unit, input_received_monotonic):
    state = client.receive(timeout=0.1)
    feedback = state["feedback"]
    ages = (state.get("feedback_cache_age_s"), feedback.get("arm_feedback_age_s"))
    if (not 0 <= time.monotonic() - input_received_monotonic <= .1
            or state["mode"] not in ("IDLE", "ARMING", "ACTIVE")):
        return state  # No automatic retransmit; robot watchdog will stop following.
    # During the finite enable/torque-baseline operation telemetry can be old.
    # ARMING packets only keep the lease alive; the robot executes a local
    # measured-pose hold. All normal following requires fresh feedback again.
    if state["mode"] != "ARMING" and (any(age is None for age in ages) or sum(ages) > .15):
        return state
    if state["mode"] in ("IDLE", "ARMING"):
        arm_urdf_rad = feedback["arm_urdf_rad"]
        if state["contract"]["with_hand"]:
            hand_feedback = feedback.get("hand")
            if not hand_feedback or hand_feedback.get("position_unit") is None:
                return state
            if state["mode"] != "ARMING" and (hand_feedback.get("age_s") is None or hand_feedback["age_s"] + ages[0] > .15):
                return state
            hand_unit = [round(v) for v in hand_feedback["position_unit"]]
    client.send_target(arm_urdf_rad, hand_unit)
    return state


def send_dual_solved_frame(client, *, left_arm_urdf_rad, right_arm_urdf_rad,
                           left_hand_unit, right_hand_unit, input_received_monotonic):
    """Four NEW inputs form one frame. Mapping keys: left_arm/right_arm/left_hand/right_hand.

    Timestamps are local laptop monotonic reception times for the actual source
    frames, not the time of this function call. A stale member suppresses all.
    """
    required = {'left_arm', 'right_arm', 'left_hand', 'right_hand'}
    if set(input_received_monotonic) != required:
        raise ValueError('freshness timestamps required for all four inputs')
    state = client.receive(timeout=0.1)
    if state['contract']['side'] != 'both':
        raise ValueError('dual helper requires --side both gateway')
    now = time.monotonic()
    if (any(not 0 <= now-stamp <= .1 for stamp in input_received_monotonic.values())
            or state['mode'] not in ('IDLE', 'ARMING', 'ACTIVE')):
        return state
    targets = {
        'left': dict(arm_urdf_rad=left_arm_urdf_rad, hand_unit=left_hand_unit),
        'right': dict(arm_urdf_rad=right_arm_urdf_rad, hand_unit=right_hand_unit)}
    for side in ('left', 'right'):
        feedback = state.get('feedback', {}).get('sides', {}).get(side, {})
        hand = feedback.get('hand') or {}
        ages = (state.get('feedback_cache_age_s'), feedback.get('arm_feedback_age_s'), hand.get('age_s'))
        if feedback.get('arm_urdf_rad') is None or hand.get('position_unit') is None:
            return state
        if state['mode'] != 'ARMING' and (any(age is None for age in ages)
                or ages[0]+max(ages[1:]) > .15):
            return state
        if state['mode'] in ('IDLE', 'ARMING'):
            targets[side] = dict(arm_urdf_rad=feedback['arm_urdf_rad'],
                                 hand_unit=[round(v) for v in hand['position_unit']])
    client.send_dual_target(left_arm_urdf_rad=targets['left']['arm_urdf_rad'],
                           right_arm_urdf_rad=targets['right']['arm_urdf_rad'],
                           left_hand_unit=targets['left']['hand_unit'],
                           right_hand_unit=targets['right']['hand_unit'])
    return state
