import importlib.util
import sys
import types
import unittest
from pathlib import Path


BRIDGE_PATH = (
    Path(__file__).resolve().parents[1]
    / "bridges"
    / "senseglove_ros_to_esrobo_hand_bridge.py"
)
SPEC = importlib.util.spec_from_file_location("senseglove_thumb_bridge", BRIDGE_PATH)
bridge = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = bridge
SPEC.loader.exec_module(bridge)


def straight_chain(x):
    return [[x, 20.0, 0.0], [x, 50.0, 0.0], [x, 78.0, 0.0], [x, 103.0, 0.0]]


def hand_points(thumb):
    return [
        *thumb,
        *straight_chain(34.0),
        *straight_chain(12.0),
        *straight_chain(-10.0),
        *straight_chain(-32.0),
    ]


OPEN_THUMB = [
    [42.0, 16.0, -3.0],
    [62.0, 29.0, -2.0],
    [81.0, 43.0, -1.0],
    [99.0, 57.0, 0.0],
]
CLOSED_THUMB = [
    [42.0, 16.0, -3.0],
    [55.0, 29.0, 8.0],
    [50.0, 31.0, 28.0],
    [35.0, 24.0, 40.0],
]


class ThumbRetargetingTests(unittest.TestCase):
    def setUp(self):
        self.args = bridge.parse_args(["--single-side", "right"])
        self.opened = bridge.extract_hand_vector_features(hand_points(OPEN_THUMB))
        self.closed = bridge.extract_hand_vector_features(hand_points(CLOSED_THUMB))
        self.opened["thumb"] = 0.1
        self.closed["thumb"] = 1.0
        self.opened["thumb_brake"] = -0.2
        self.closed["thumb_brake"] = 0.8

    def test_wrist_to_thumb_tip_features_include_cmc_motion(self):
        for key in (
            "vector_thumb_internal_curl",
            "vector_thumb_tip_spread",
            "vector_thumb_tip_elevation",
        ):
            self.assertIn(key, self.opened)
        self.assertGreater(
            abs(
                self.closed["vector_thumb_internal_curl"]
                - self.opened["vector_thumb_internal_curl"]
            ),
            0.05,
        )
        self.assertGreater(
            abs(
                self.closed["vector_thumb_tip_spread"]
                - self.opened["vector_thumb_tip_spread"]
            ),
            0.05,
        )
        self.assertGreater(
            abs(
                self.closed["vector_thumb_tip_elevation"]
                - self.opened["vector_thumb_tip_elevation"]
            ),
            0.1,
        )

    def test_all_zero_driver_placeholder_is_not_live_sensor_data(self):
        state = bridge.SideState("/test")
        state.hand_positions_mm = [[0.0, 0.0, 0.0] for _ in range(20)]
        self.assertFalse(state.has_live_sensor_payload())

        state.hand_positions_mm[3][1] = 0.01
        self.assertTrue(state.has_live_sensor_payload())

    def test_disconnected_device_rejects_last_nonzero_pose(self):
        state = bridge.SideState("/test")
        state.hand_positions_mm = hand_points(OPEN_THUMB)
        state.connected = True
        self.assertTrue(state.has_live_sensor_payload())

        state.connected = False
        self.assertFalse(state.has_live_sensor_payload())

    def test_single_side_disconnect_watchdog_ignores_mirrored_side(self):
        fake_bridge = object.__new__(bridge.SenseGloveUdpBridge)
        fake_bridge.args = types.SimpleNamespace(single_side="right")
        fake_bridge.left = bridge.SideState("/left", connected=False)
        fake_bridge.right = bridge.SideState("/right", connected=True)
        self.assertFalse(fake_bridge.has_explicit_disconnect())

        fake_bridge.right.connected = False
        self.assertTrue(fake_bridge.has_explicit_disconnect())

    def test_sdk_thumb_flexion_drives_robot_pitch_monotonically(self):
        mid = dict(self.opened)
        mid["thumb"] = 0.55
        for key in (
            "vector_thumb_internal_curl",
            "vector_thumb_tip_spread",
            "vector_thumb_tip_elevation",
        ):
            mid[key] = (self.opened[key] + self.closed[key]) * 0.5

        open_targets, _ = bridge.retarget_side_vectors(
            self.opened, self.opened, self.closed, "right", self.args
        )
        mid_targets, _ = bridge.retarget_side_vectors(
            mid, self.opened, self.closed, "right", self.args
        )
        closed_targets, _ = bridge.retarget_side_vectors(
            self.closed, self.opened, self.closed, "right", self.args
        )
        self.assertEqual(open_targets[2], 0.0)
        self.assertGreater(mid_targets[2], 0.25)
        self.assertGreater(closed_targets[2], mid_targets[2])

    def test_right_thumb_flexion_is_linear(self):
        mid = dict(self.opened)
        for key in ("thumb", "vector_thumb_internal_curl"):
            mid[key] = 0.5 * (self.opened[key] + self.closed[key])

        _targets, diagnostics = bridge.retarget_side_vectors(
            mid, self.opened, self.closed, "right", self.args
        )

        self.assertAlmostEqual(diagnostics["thumb_pitch"], 0.5, places=3)

    def test_unexercised_thumb_abduction_does_not_fall_back_to_flexion(self):
        closed = dict(self.closed)
        closed["thumb_brake"] = self.opened["thumb_brake"]
        targets, diagnostics = bridge.retarget_side_vectors(
            closed, self.opened, closed, "right", self.args
        )
        self.assertAlmostEqual(diagnostics["thumb_yaw"], 0.0)
        self.assertAlmostEqual(targets[1], 0.0)

    def test_thumb_flexion_does_not_drive_side_swing_or_roll(self):
        flexed = dict(self.opened)
        for key in ("thumb", "vector_thumb_internal_curl", "vector_thumb_curl"):
            flexed[key] = (self.opened[key] + self.closed[key]) * 0.5
        # Distal motion used to leak into roll/yaw through the tip direction.
        flexed["vector_thumb_tip_spread"] = self.closed["vector_thumb_tip_spread"]
        flexed["vector_thumb_tip_elevation"] = self.closed[
            "vector_thumb_tip_elevation"
        ]

        targets, diagnostics = bridge.retarget_side_vectors(
            flexed, self.opened, self.closed, "right", self.args
        )

        self.assertGreater(diagnostics["thumb_pitch"], 0.0)
        self.assertAlmostEqual(diagnostics["thumb_roll"], 0.0)
        self.assertAlmostEqual(diagnostics["thumb_yaw"], 0.0)
        self.assertAlmostEqual(targets[0], 0.0)
        self.assertAlmostEqual(targets[1], 0.0)

    def test_sdk_thumb_abduction_only_drives_robot_yaw(self):
        abducted = dict(self.opened)
        abducted["thumb_brake"] = 0.3

        targets, diagnostics = bridge.retarget_side_vectors(
            abducted, self.opened, self.closed, "right", self.args
        )

        self.assertAlmostEqual(diagnostics["thumb_roll"], 0.0)
        self.assertGreater(diagnostics["thumb_yaw"], 0.0)
        self.assertAlmostEqual(diagnostics["thumb_pitch"], 0.0)
        self.assertAlmostEqual(targets[0], 0.0)
        self.assertGreater(targets[1], 0.0)
        self.assertAlmostEqual(targets[2], 0.0)

    def test_tip_direction_is_diagnostic_only(self):
        moved_tip = dict(self.opened)
        moved_tip["vector_thumb_tip_spread"] += 1.0
        moved_tip["vector_thumb_tip_elevation"] -= 1.0

        open_targets, _ = bridge.retarget_side_vectors(
            self.opened, self.opened, self.closed, "right", self.args
        )
        moved_targets, _ = bridge.retarget_side_vectors(
            moved_tip, self.opened, self.closed, "right", self.args
        )
        self.assertEqual(moved_targets[:3], open_targets[:3])

    def test_official_fingertip_distances_are_exposed(self):
        features = bridge.extract_hand_vector_features(hand_points(OPEN_THUMB))
        points = hand_points(OPEN_THUMB)
        for finger, tip_index in (
            ("index", 7),
            ("middle", 11),
            ("ring", 15),
            ("pinky", 19),
        ):
            self.assertAlmostEqual(
                features[f"thumb_{finger}_tip_distance_mm"],
                bridge.math.dist(points[3], points[tip_index]),
            )

    def test_official_thumb_joint_decomposition_is_preserved(self):
        msg = types.SimpleNamespace(
            joint_names=[
                "r_thumb_mcp",
                "r_thumb_pip",
                "r_thumb_dip",
                "r_thumb_brake",
            ],
            position=[0.2, 0.4, 0.6, -0.3],
        )
        features = bridge.extract_features(msg)
        self.assertAlmostEqual(features["thumb_mcp"], 0.2)
        self.assertAlmostEqual(features["thumb_pip"], 0.4)
        self.assertAlmostEqual(features["thumb_dip"], 0.6)
        self.assertAlmostEqual(features["thumb"], 0.4)
        self.assertAlmostEqual(features["thumb_brake"], -0.3)

    def test_reversed_middle_vector_falls_back_to_joint_flexion(self):
        opened = dict(self.opened)
        closed = dict(self.closed)
        current = dict(self.opened)
        opened.update({"vector_middle_curl": 2.404, "middle": 0.955})
        closed.update({"vector_middle_curl": 2.312, "middle": 1.512})
        current.update({"vector_middle_curl": 2.360, "middle": 1.2335})

        candidates = bridge.finger_flexion_candidates(opened, closed, "middle")
        self.assertEqual(candidates[0][1], "middle")
        _targets, diagnostics = bridge.retarget_side_vectors(
            current, opened, closed, "right", self.args
        )
        expected = (
            self.args.right_finger_flexion_gain
            * 0.5 ** self.args.right_finger_flexion_exponent
        )
        self.assertAlmostEqual(diagnostics["middle"], expected, places=3)
        self.assertAlmostEqual(diagnostics["middle"], 0.5, places=3)

    def test_left_and_right_use_the_same_default_response(self):
        opened = dict(self.opened)
        closed = dict(self.closed)
        current = {
            key: open_value + 0.5 * (closed[key] - open_value)
            for key, open_value in opened.items()
            if key in closed
        }
        for finger in ("index", "middle", "ring", "pinky"):
            for key in (finger, f"vector_{finger}_curl"):
                opened[key] = 0.0
                closed[key] = 1.0
                current[key] = 0.5

        _right_targets, right = bridge.retarget_side_vectors(
            current, opened, closed, "right", self.args
        )
        _left_targets, left = bridge.retarget_side_vectors(
            current, opened, closed, "left", self.args
        )

        for finger in ("index", "middle", "ring", "pinky"):
            self.assertAlmostEqual(right[finger], 0.5)
            self.assertAlmostEqual(left[finger], right[finger])
        for channel in ("thumb_pitch", "thumb_roll", "thumb_yaw"):
            self.assertAlmostEqual(left[channel], right[channel])

    def test_pinky_uses_ring_input_in_vector_mode(self):
        current = dict(self.opened)
        current["vector_ring_curl"] = self.opened["vector_ring_curl"] + 0.7 * (
            self.closed["vector_ring_curl"] - self.opened["vector_ring_curl"]
        )
        current["vector_pinky_curl"] = self.opened["vector_pinky_curl"]
        current["vector_ring_spread"] = self.opened["vector_ring_spread"] - 0.1
        current["vector_pinky_spread"] = self.opened["vector_pinky_spread"] + 0.2

        targets, diagnostics = bridge.retarget_side_vectors(
            current, self.opened, self.closed, "right", self.args
        )

        self.assertAlmostEqual(diagnostics["pinky"], diagnostics["ring"])
        self.assertAlmostEqual(
            diagnostics["pinky_spread"], diagnostics["ring_spread"]
        )
        self.assertAlmostEqual(targets[9], targets[7])

    def test_pinky_uses_ring_input_in_curl_mode(self):
        args = bridge.parse_args(
            ["--single-side", "right", "--retargeting-mode", "curl"]
        )
        opened = {
            finger: 0.0 for finger in ("thumb", "index", "middle", "ring", "pinky")
        }
        closed = {
            finger: 1.0 for finger in ("thumb", "index", "middle", "ring", "pinky")
        }
        current = dict(opened)
        current.update({"ring": 0.7, "pinky": 0.1})
        for finger in bridge.FINGERS:
            opened[f"{finger}_brake"] = 0.0
            closed[f"{finger}_brake"] = 1.0
            current[f"{finger}_brake"] = 0.0
        current.update({"ring_brake": 0.5, "pinky_brake": 0.0})

        targets, diagnostics = bridge.retarget_side(
            current, opened, closed, None, "right", args
        )

        self.assertAlmostEqual(diagnostics["pinky"], diagnostics["ring"])
        self.assertAlmostEqual(targets[9], targets[7])

    def test_both_thumbs_use_dedicated_pinch_direction_endpoint(self):
        pinched = dict(self.closed)
        for side in ("left", "right"):
            _targets, diagnostics = bridge.retarget_side_vectors(
                pinched, self.opened, self.closed, side, self.args, pinched
            )

            self.assertAlmostEqual(diagnostics["thumb_roll"], 0.85, places=3)
            self.assertAlmostEqual(diagnostics["thumb_yaw"], 0.45, places=3)

            beyond = dict(pinched)
            for key in ("thumb_brake", "vector_thumb_elevation"):
                beyond[key] = self.opened[key] + 1.5 * (
                    pinched[key] - self.opened[key]
                )
            _targets, beyond_diagnostics = bridge.retarget_side_vectors(
                beyond, self.opened, self.closed, side, self.args, pinched
            )
            self.assertGreater(beyond_diagnostics["thumb_yaw"], 0.45)
            self.assertGreater(beyond_diagnostics["thumb_roll"], 0.85)

    def test_index_pinch_reference_does_not_warp_finger_flexion(self):
        opened = dict(self.opened)
        closed = dict(self.closed)
        for finger in ("index", "middle", "ring", "pinky"):
            opened[f"vector_{finger}_curl"] = 0.0
            closed[f"vector_{finger}_curl"] = 1.0
        pose = dict(opened)
        pose["thumb"] = opened["thumb"] + 0.8 * (
            closed["thumb"] - opened["thumb"]
        )
        pose["vector_thumb_internal_curl"] = opened[
            "vector_thumb_internal_curl"
        ] + 0.8 * (
            closed["vector_thumb_internal_curl"]
            - opened["vector_thumb_internal_curl"]
        )
        pose["thumb_brake"] = closed["thumb_brake"]
        pose["vector_thumb_elevation"] = closed["vector_thumb_elevation"]
        pose["vector_index_curl"] = 0.5
        pose["vector_middle_curl"] = 0.4
        pose["vector_ring_curl"] = 0.3
        pose["thumb_index_tip_distance_mm"] = (
            opened["thumb_index_tip_distance_mm"] - 20.0
        )
        pinches = {"index": pose}

        _targets, diagnostics = bridge.retarget_side_vectors(
            pose, opened, closed, "right", self.args, pinches
        )

        self.assertAlmostEqual(diagnostics["pinch_contact_confidence"], 0.0)
        self.assertAlmostEqual(diagnostics["index"], 0.5, places=3)
        self.assertAlmostEqual(diagnostics["middle"], 0.4, places=3)
        self.assertAlmostEqual(diagnostics["ring"], 0.3, places=3)
        self.assertAlmostEqual(diagnostics["pinky"], diagnostics["ring"], places=3)
        bridge.validate_calibration_motion(
            {"right": opened},
            {"index": {"right": pose}},
            {"right": closed},
            self.args,
        )

    def test_pinch_calibration_rejects_missing_distance(self):
        opened = {
            "thumb": 0.0,
            "vector_thumb_internal_curl": 0.0,
            "thumb_brake": 0.0,
            "vector_thumb_elevation": 0.0,
            "thumb_index_tip_distance_mm": 62.0,
            **{
                f"vector_{finger}_curl": 0.0
                for finger in ("index", "middle", "ring", "pinky")
            },
        }
        closed = {
            "thumb": 1.0,
            "vector_thumb_internal_curl": 1.0,
            **{
                f"vector_{finger}_curl": 1.0
                for finger in ("index", "middle", "ring", "pinky")
            },
        }
        invalid_pinch = {
            "thumb_brake": 0.3,
            "vector_thumb_elevation": 0.4,
            "thumb_index_tip_distance_mm": float("nan"),
        }

        with self.assertRaisesRegex(RuntimeError, "no finite fingertip distance"):
            bridge.validate_calibration_motion(
                {"right": opened},
                {"right": invalid_pinch},
                {"right": closed},
                self.args,
            )

    def test_pinch_calibration_accepts_contact_with_two_axis_motion(self):
        opened = {
            "thumb": 0.0,
            "vector_thumb_internal_curl": 0.0,
            "thumb_brake": 0.0,
            "vector_thumb_elevation": 0.0,
            "thumb_index_tip_distance_mm": 49.3,
            **{
                f"vector_{finger}_curl": 0.0
                for finger in ("index", "middle", "ring", "pinky")
            },
        }
        closed = {
            "thumb": 1.0,
            "vector_thumb_internal_curl": 1.0,
            **{
                f"vector_{finger}_curl": 1.0
                for finger in ("index", "middle", "ring", "pinky")
            },
        }
        pinched = {
            "thumb_brake": 0.3,
            "vector_thumb_elevation": 0.4,
            "thumb_index_tip_distance_mm": 47.7,
        }

        bridge.validate_calibration_motion(
            {"right": opened}, {"right": pinched}, {"right": closed}, self.args
        )

    def test_right_fingers_do_not_saturate_before_closed_endpoint(self):
        opened = dict(self.opened)
        closed = dict(self.closed)
        current = dict(self.opened)
        for finger in ("index", "middle", "ring", "pinky"):
            opened[f"vector_{finger}_curl"] = 0.0
            closed[f"vector_{finger}_curl"] = 1.0
            current[f"vector_{finger}_curl"] = 0.9

        _targets, diagnostics = bridge.retarget_side_vectors(
            current, opened, closed, "right", self.args
        )

        for finger in ("index", "middle", "ring", "pinky"):
            self.assertAlmostEqual(diagnostics[finger], 0.9)

    def test_live_straighter_pose_expands_open_endpoint(self):
        left = types.SimpleNamespace(features={"index": 0.4})
        right = types.SimpleNamespace(features={"index": 0.5})
        fake_bridge = types.SimpleNamespace(
            args=types.SimpleNamespace(swap_left_right_targets=False),
            left=left,
            right=right,
        )
        calibration = {
            "open": {
                "left": {"index": 0.6},
                "right": {"index": 0.6},
            },
            "closed": {
                "left": {"index": 1.4},
                "right": {"index": 1.4},
            },
        }

        bridge.adapt_flexion_calibration_envelope(fake_bridge, calibration)
        self.assertEqual(calibration["open"]["left"]["index"], 0.4)
        self.assertEqual(calibration["open"]["right"]["index"], 0.5)


if __name__ == "__main__":
    unittest.main()
