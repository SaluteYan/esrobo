"""Explicitly supervised SDK commissioning."""


def wake_confirmed_left_zero(arm, zero, tolerance):
    """Supervised commissioning only; always finish with bounded disable."""
    import numpy as np
    result = dict(wake_attempted=False, zero_verified=False, disabled=False)
    try:
        q = arm.get_joint_angles(timeout=.3)
        states = arm.get_joint_enable_states()
        fault = arm.controller_fault(allow_disabled=True, allow_estop=True)
        if fault and fault.get('category') not in (
                'controller_feedback_stale', 'driver_feedback_stale'):
            raise RuntimeError('controller/driver fault refuses commissioning: '+str(fault))
        if q is not None:
            if not np.all(np.isfinite(q)) or np.shape(q) != (7,) or np.max(np.abs(q-zero)) > tolerance:
                raise RuntimeError('measured pose differs from operator-confirmed project zero')
        if q is None or states is None:
            result['wake_attempted'] = True
            q, states = arm.enable_at_known_target(zero, speed_percent=5, timeout=3.)
        result['zero_verified'] = bool(q is not None and np.shape(q) == (7,)
            and np.all(np.isfinite(q)) and np.max(np.abs(q-zero)) <= tolerance
            and states is not None and len(states) == 7)
        if not result['zero_verified']:
            raise RuntimeError('wake failed to verify seven-joint zero feedback')
        result['physical_rad'] = np.asarray(q).tolist()
        result['enable_states_after_wake'] = states
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        # An operator has explicitly supported the arm for this operation.
        # Missing feedback is never reported as successful disable.
        try:
            if result['wake_attempted']:
                result['disabled'] = bool(arm.disable(timeout=3.))
                if not result['disabled']:
                    arm.best_effort_disable_once()
            else:
                result['disabled'] = arm.get_joint_enable_states() == [False]*7
            result['final_enable_states'] = arm.get_joint_enable_states()
            result['disabled'] = bool(result['disabled'] and result['final_enable_states'] == [False]*7)
            result['controller_fault'] = arm.controller_fault(allow_disabled=True)
        except Exception as exc:
            result['cleanup_error'] = str(exc)
            result['disabled'] = False
            if result['wake_attempted']:
                try:
                    arm.best_effort_disable_once()
                except Exception as disable_exc:
                    result['disable_send_error'] = str(disable_exc)
    return result


def wake_left_zero_script(remote):
    """Fixed SDK operation, serialized with the gateway's hardware lock."""
    import inspect
    return ('import sys,os,fcntl,json,numpy as np\n'
        + f'sys.path.insert(0, {remote + "/teleoperation/src"!r})\n'
        + 'lock=open(f"/tmp/esrobo-robot-link-{os.getuid()}.lock", "a")\n'
        + 'fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)\n'
        + inspect.getsource(wake_confirmed_left_zero)
        + '\nfrom esrobo_teleop.config import load_config\n'
        + 'from esrobo_teleop.robot.nero_driver import NeroArm\n'
        + f'cfg=load_config({remote + "/teleoperation/config/teleop_config.yaml"!r}).robot\n'
        + 'arm=NeroArm(cfg, "can_piper1", "left")\n'
        + 'try:\n arm.connect()\n'
        + ' result=wake_confirmed_left_zero(arm, np.asarray(cfg.left_joint_offsets, float), '
          'np.deg2rad(cfg.full_arm_startup_zero_tolerance_deg))\n'
        + ' print("SDK_WAKE_RESULT="+json.dumps(result), flush=True)\n'
        + 'finally:\n arm.disconnect()\n')
