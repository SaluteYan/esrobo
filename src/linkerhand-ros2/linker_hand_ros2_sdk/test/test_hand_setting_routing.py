"""Exercise the SDK callback without importing ROS or opening CAN devices."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

SOURCE = Path(__file__).parents[1] / 'linker_hand_ros2_sdk/linker_hand.py'


def callback():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LinkerHand')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'hand_setting_cb')
    namespace = {'json': json, 'ColorMsg': lambda **kw: None}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace['hand_setting_cb']


def test_right_setting_never_changes_left():
    cb = callback()
    left = SimpleNamespace(hand_type='left', hand_joint='L10', api=Mock(), cmd_lock=False)
    right = SimpleNamespace(hand_type='right', hand_joint='L10', api=Mock(), cmd_lock=False)
    msg = SimpleNamespace(data=json.dumps({'setting_cmd': 'set_speed', 'params': {'hand_type': 'right', 'speed': [10] * 10}}))
    cb(left, msg)
    cb(right, msg)
    assert left.api.mock_calls == []
    right.api.set_speed.assert_called_once_with(speed=[10] * 10)
    assert not left.cmd_lock and not right.cmd_lock


def test_malformed_setting_does_not_touch_hardware():
    cb = callback()
    node = SimpleNamespace(hand_type='right', hand_joint='L10', api=Mock(), cmd_lock=False)
    for text in ['bad json', '{}', '[]', '{"params":null}', '{"params":{"hand_type":"right"}}']:
        cb(node, SimpleNamespace(data=text))
    assert node.api.mock_calls == []
    assert not node.cmd_lock


def test_right_l10_clear_fault_does_not_call_unsupported_api():
    node = SimpleNamespace(hand_type='right', hand_joint='L10', api=Mock(), cmd_lock=False)
    callback()(node, SimpleNamespace(data=json.dumps({'setting_cmd': 'clear_faults', 'params': {'hand_type': 'right'}})))
    assert node.api.mock_calls == []
    assert not node.cmd_lock
