"""CAN receive regression checks without ROS or real CAN sockets."""
import ast
from pathlib import Path
from types import SimpleNamespace
import time

SOURCE = Path(__file__).parents[1] / 'linker_hand_ros2_sdk/LinkerHand/core/can/linker_hand_l10_can.py'

def load_receiver():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LinkerHandL10Can')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'process_response')
    from enum import Enum
    enum = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'FrameProperty')
    ns = {'Enum': Enum, 'time': time}
    exec(compile(ast.Module(body=[enum, method], type_ignores=[]), str(SOURCE), 'exec'), ns)
    return ns['process_response']

def frame(data, **kw):
    attrs = dict(data=bytearray(data), arbitration_id=0x27, is_rx=True,
                 is_error_frame=False, is_remote_frame=False, is_extended_id=False)
    attrs.update(kw)
    return SimpleNamespace(**attrs)

def test_local_command_cannot_masquerade_as_feedback():
    receive = load_receiver()
    hand = SimpleNamespace(can_id=0x27, x01=[136,210,254,254,254,254], x04=[109,101,119,33])
    receive(hand, frame([1,255,210,254,254,254,254], is_rx=False))
    assert hand.x01[0] == 136
    receive(hand, frame([1,137,210,254,254,254,254]))
    assert hand.x01[0] == 137

def test_queries_and_malformed_frames_preserve_last_measurement():
    receive = load_receiver()
    hand = SimpleNamespace(can_id=0x27, x01=[136]*6, x04=[33]*4)
    for msg in [frame([]), frame([1]), frame([1,255]), frame([4,1,2,3,4,5]),
                frame([1]+[255]*6, is_error_frame=True),
                frame([1]+[255]*6, is_remote_frame=True),
                frame([1]+[255]*6, is_extended_id=True),
                frame([1]+[255]*6, arbitration_id=0x28)]:
        receive(hand, msg)
    assert hand.x01 == [136]*6 and hand.x04 == [33]*4
    receive(hand, frame([4,109,101,119,33]))
    assert hand.x04 == [109,101,119,33]


def test_only_recent_hardware_replies_renew_both_halves():
    from unittest.mock import patch
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LinkerHandL10Can')
    fn = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'position_feedback_fresh')
    ns = {'time': time}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(SOURCE), 'exec'), ns)
    fresh = ns['position_feedback_fresh']
    receive = load_receiver()
    hand = SimpleNamespace(can_id=0x27, x01=[], x04=[])
    with patch.object(time, 'monotonic', return_value=10.):
        assert not fresh(hand)
        receive(hand, frame([1]+[136]*6))
        assert not fresh(hand)
        receive(hand, frame([4]+[33]*4))
        assert fresh(hand)
    with patch.object(time, 'monotonic', return_value=10.2):
        receive(hand, frame([1]+[255]*6, is_rx=False))
        receive(hand, frame([4]+[33]*4))
        assert not fresh(hand)
        receive(hand, frame([1]+[136]*6))
        assert fresh(hand)
