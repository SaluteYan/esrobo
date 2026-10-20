"""Passive execution trace adapters. All observations are existing caches."""
from esrobo_teleop.debug.motion_trace import session_tag

FIELDS = ('ik_urdf_rad', 'trajectory_urdf_rad', 'command_urdf_rad', 'velocity_urdf_rad_s',
          'feedback_urdf_rad', 'sent_monotonic_s', 'position_frame_sent', 'feedback_snapshot',
          'configured_speed_bound_active', 'acceleration_bound_active', 'velocity_bound_active',
          'step_bound_active', 'feedback_dt_s', 'feedback_velocity_rad_s',
          'directional_lead_braking_active', 'feedback_motion_braking_active',
          'first_follow_hold', 'position_limit_hold', 'fault',
          'collision_search_progress',
          'feasible_velocity_lower_rad_s', 'feasible_velocity_upper_rad_s',
          'collision_rejections', 'fault_context')


def attach(backend, trace):
    members = getattr(backend, 'members', None) or {getattr(backend, 'side', 'single'): backend}
    for side, member in members.items():
        arm = getattr(member, 'arm', None)
        if arm is None:
            continue
        arm._motion_trace_enabled = True
        arm._motion_trace_feedback = None
        previous = getattr(arm, 'probe_sink', None)
        def observe(event, side=side, previous=previous):
            if previous is not None:
                previous(event)
            trace.emit('driver', side=side, driver_event=event.get('event'),
                       context=getattr(backend, '_motion_trace_context', None),
                       **{k:event[k] for k in FIELDS if k in event})
        arm.probe_sink = observe


def command_context(message, now):
    return dict(command_seq=message.get('seq'), session_tag=session_tag(message.get('session')),
                accepted_monotonic_s=now, kind=message.get('type'))


def clear_cycle_feedback(backend):
    members = getattr(backend, 'members', None) or {getattr(backend, 'side', 'single'): backend}
    for member in members.values():
        arm = getattr(member, 'arm', None)
        if arm is not None:
            arm._motion_trace_feedback = None


def cached_feedback(backend):
    result = {}
    members = getattr(backend, 'members', None) or {getattr(backend, 'side', 'single'): backend}
    for side, member in members.items():
        arm = getattr(member, 'arm', None)
        if arm is None:
            continue
        snap = arm.__dict__.get('_motion_trace_feedback')
        if snap is None:
            snap = arm.__dict__.get('_cycle_feedback')
        result[side] = dict(
            feedback=(None if snap is None else dict(snap.diagnostics(), received_monotonic_s=snap.received_monotonic)),
            motion=arm.__dict__.get('motion_limit_diagnostics'),
            last_command_skipped=arm.__dict__.get('last_command_skipped'),
            skip_reason=arm.__dict__.get('last_command_skip_reason'),
            fault=arm.__dict__.get('_safety_fault_reason'))
    return result
