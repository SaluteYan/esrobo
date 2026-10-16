import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import console


class HandoverTests(unittest.TestCase):
    def setUp(self):
        self.state = {
            "connected": True, "owner": None, "processes": {},
            "robot": {"mode": "IDLE", "age_s": .1,
                      "feedback": {"enable_states": [False] * 7}},
        }

    def test_disabled_idle_allows_handover(self):
        console.ensure_handover_safe(self.state)

    def test_active_or_held_arm_cannot_be_exited(self):
        for mode in ("ACTIVE", "HOLD_LATCHED", "APPROACHING", "FAULT"):
            with self.subTest(mode=mode):
                s = copy.deepcopy(self.state)
                s["robot"]["mode"] = mode
                with self.assertRaises(RuntimeError):
                    console.ensure_handover_safe(s)

    def test_stale_feedback_and_partial_disable_are_rejected(self):
        for age, enabled in ((2, [False]*7), (.1, [False]*6+[True]), (.1, None)):
            with self.subTest(age=age, enabled=enabled):
                s = copy.deepcopy(self.state)
                s["robot"].update(age_s=age, feedback={"enable_states": enabled})
                with self.assertRaises(RuntimeError):
                    console.ensure_handover_safe(s)

    def test_one_enabled_arm_blocks_dual_handover(self):
        self.state["robot"]["feedback"] = {"sides": {
            "left": {"enable_states": [False]*7},
            "right": {"enable_states": [True]*7},
        }}
        with self.assertRaises(RuntimeError):
            console.ensure_handover_safe(self.state)

    def test_running_teleop_or_other_operator_blocks_handover(self):
        for patch in ({"owner": "browser-123"}, {"processes": {"teleop": {"running": True}}}):
            with self.subTest(patch=patch):
                s = copy.deepcopy(self.state)
                s.update(patch)
                with self.assertRaises(RuntimeError):
                    console.ensure_handover_safe(s)


class ConfigTests(unittest.TestCase):
    def test_arm_bus_cannot_be_selected_as_hand_bus(self):
        c = console.load_config(console.DEFAULT_CONFIG)
        c["hand_interfaces"]["left"] = "can_piper1"
        with tempfile.TemporaryDirectory() as d:
            p = Path(d)/"config.json"
            p.write_text(json.dumps(c))
            with self.assertRaises(ValueError):
                console.load_config(p)


class StartTests(unittest.TestCase):
    def setUp(self):
        self.config = console.load_config(console.DEFAULT_CONFIG)

    def test_conflicting_hand_driver_prevents_launch(self):
        probe = {"conflicts": [{"pid": 1234, "args": ["python3", "linker_hand_sdk"]}],
                 "interfaces": [{"ifname": "can_hand1"}, {"ifname": "can_hand2"}]}
        with patch.object(console, "service_state", return_value={"ActiveState": "inactive"}), \
                patch.object(console, "ssh", return_value=json.dumps(probe)) as remote:
            with self.assertRaises(RuntimeError):
                console.start(self.config, "both")
            self.assertEqual(remote.call_count, 1)
            self.assertEqual(remote.call_args.args[1][:2], ["python3", "-c"])

    def test_missing_hand_interface_prevents_launch(self):
        probe = {"conflicts": [], "interfaces": [{"ifname": "can_piper1"}]}
        with patch.object(console, "service_state", return_value={"ActiveState": "inactive"}), \
                patch.object(console, "ssh", return_value=json.dumps(probe)) as remote:
            with self.assertRaises(RuntimeError):
                console.start(self.config, "both")
            self.assertEqual(remote.call_count, 1)

    def test_changing_sides_never_restarts_existing_service(self):
        with patch.object(console, "service_state", return_value={
                "ActiveState": "active", "Environment": "CAN_PORTS=can_hand2,can_hand1"}), \
                patch.object(console, "ssh") as remote:
            with self.assertRaises(RuntimeError):
                console.start(self.config, "left")
            remote.assert_not_called()

    def test_unloaded_transient_service_is_reported_inactive(self):
        with patch.object(console, "ssh", return_value="LoadState=not-found\nActiveState=inactive\nSubState=dead"):
            self.assertEqual(console.service_state(self.config)["ActiveState"], "inactive")


if __name__ == "__main__":
    unittest.main()
