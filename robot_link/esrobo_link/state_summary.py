"""Bounded diagnostic views; core feedback and protocol fields are never dropped."""
import copy


def short_text(value, budget=384):
    text = str(value)
    raw = text.encode('utf-8')
    return text if len(raw) <= budget else raw[:budget-3].decode('utf-8', errors='ignore')+'...'


def fault_summary(value):
    if isinstance(value, str):
        return short_text(value)
    if isinstance(value, dict):
        return {k: fault_summary(v) for k, v in value.items()}
    if isinstance(value, list):
        return [fault_summary(v) for v in value]
    return value


def feedback_summary(state):
    if not isinstance(state, dict):
        return state
    if 'left' in state or 'right' in state:
        return {k: feedback_summary(v) for k,v in state.items()}
    keys = ('arm_urdf_rad', 'arm_feedback_age_s', 'enable_states',
            'controller_fault', 'driver_fault')
    result = {k: fault_summary(state[k]) for k in keys if k in state}
    hand = state.get('hand')
    if isinstance(hand, dict):
        result['hand'] = {k: copy.deepcopy(hand[k]) for k in
                          ('position_unit', 'age_s', 'geometry', 'command_unit', 'command_seq') if k in hand}
    elif 'hand' in state:
        result['hand'] = hand
    return result


def return_summary(result):
    if not isinstance(result, dict):
        return result
    keys = ('returned', 'disabled', 'arm_returned', 'hands_open', 'hands_ready',
            'holding', 'stage', 'near_zero_finger_exemption')
    out = {k: result[k] for k in keys if k in result}
    if 'reason' in result:
        out['reason'] = short_text(result['reason'])
    if 'hand_return_feedback' in result:
        out['hand_return_feedback'] = feedback_summary(result['hand_return_feedback'])
    return out


def return_failure(prefix, result):
    return f"{prefix} [{result.get('stage', 'unknown')}]: {short_text(result.get('reason', 'verification failed'))}"


def compact_state(message):
    out = dict(message, diagnostics_truncated=True)
    out['feedback'] = feedback_summary(message.get('feedback'))
    out['last_return'] = return_summary(message.get('last_return'))
    for key in ('reason', 'last_rejection', 'hold_reason'):
        if out.get(key) is not None:
            out[key] = short_text(out[key], 256)
    out.pop('entry_execution', None)
    return out
