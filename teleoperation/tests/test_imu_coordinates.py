import math
import time
import unittest
from types import SimpleNamespace

import numpy as np

from bridges.senseglove_ros_to_esrobo_hand_bridge import (
    SenseGloveUdpBridge,
    SideState,
    correct_imu_quat_xyzw,
    estimate_imu_axis_decoupling,
    estimate_imu_frame_corrections,
    extract_imu_quat_wxyz,
    opposite_reconstructed_w_branch_wxyz,
    orientation_delta_wxyz,
    persistent_imu_w_repair_sides,
    repair_senseglove_imu_w,
    rotation_vector_wxyz,
    runtime_axis_decoupling,
)
from esrobo_teleop import math_utils as mu
from esrobo_teleop.config import build_config
from esrobo_teleop.device.body_device import BodyDevice
from esrobo_teleop.robot.nero_driver import imu_local_rotation_to_wrist_offsets


class ImuCoordinateTests(unittest.TestCase):
    def test_persistent_w_repair_requires_enough_samples_and_high_ratio(self):
        bridge = SenseGloveUdpBridge.__new__(SenseGloveUdpBridge)
        bridge.left = SideState("left-topic")
        bridge.right = SideState("right-topic")
        bridge.right.imu_sample_count = 120
        bridge.right.imu_w_repaired_count = 119
        bridge.left.imu_sample_count = 120
        bridge.left.imu_w_repaired_count = 20

        self.assertEqual(
            persistent_imu_w_repair_sides(bridge, "right"),
            [("right", 119, 120)],
        )
        self.assertEqual(persistent_imu_w_repair_sides(bridge, "left"), [])

    def test_runtime_w_branch_is_seeded_from_persisted_calibration(self):
        bridge = SenseGloveUdpBridge.__new__(SenseGloveUdpBridge)
        bridge.left = SideState("left-topic")
        bridge.right = SideState("right-topic")
        calibration = {
            "imu_neutral": {
                "left": [0.5, 0.5, 0.5, 0.5],
                "right": [0.5, -0.5, 0.5, -0.5],
            }
        }

        bridge.seed_imu_continuity_from_calibration(calibration)

        np.testing.assert_allclose(bridge.left.imu_quat_wxyz, calibration["imu_neutral"]["left"])
        np.testing.assert_allclose(bridge.right.imu_quat_wxyz, calibration["imu_neutral"]["right"])

    def test_auto_repairs_duplicated_w_even_when_raw_norm_is_near_one(self):
        raw = [-0.829955995, -0.554748952, -0.055176001, -0.055176001]
        repaired, raw_norm, did_repair = repair_senseglove_imu_w(raw, "auto")

        self.assertTrue(did_repair)
        self.assertLess(abs(raw_norm - 1.0), 0.02)
        self.assertAlmostEqual(sum(value * value for value in repaired), 1.0, places=6)
        self.assertNotAlmostEqual(repaired[2], repaired[3], places=5)

    def test_reconstructed_w_does_not_inherit_duplicated_z_sign(self):
        positive_z, _, positive_repaired = repair_senseglove_imu_w(
            [0.2, -0.3, 0.4, 0.4], "auto"
        )
        negative_z, _, negative_repaired = repair_senseglove_imu_w(
            [0.2, -0.3, -0.4, -0.4], "auto"
        )

        self.assertTrue(positive_repaired)
        self.assertTrue(negative_repaired)
        self.assertGreater(positive_z[3], 0.0)
        self.assertGreater(negative_z[3], 0.0)

    def test_reconstructed_w_uses_the_branch_closest_to_previous_frame(self):
        xyz = [0.2, -0.3, 0.4]
        w_abs = math.sqrt(1.0 - sum(value * value for value in xyz))
        previous_xyzw = correct_imu_quat_xyzw(xyz + [-w_abs], "unity_to_ros")
        self.assertIsNotNone(previous_xyzw)
        px, py, pz, pw = previous_xyzw
        previous_wxyz = [pw, px, py, pz]
        msg = SimpleNamespace(
            imu_orientation=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2], w=xyz[2])
        )

        current = extract_imu_quat_wxyz(
            msg,
            "unity_to_ros",
            "auto",
            previous_quat_wxyz=previous_wxyz,
        )

        self.assertGreater(abs(float(np.dot(current, previous_wxyz))), 1.0 - 1.0e-6)

    def test_bridge_calibration_only_removes_zero_and_preserves_anatomical_axes(self):
        neutral_rotvecs = (
            np.asarray([0.37, -0.51, math.pi / 3]),
            np.asarray([-0.62, 0.28, -0.91]),
        )
        for neutral_rotvec in neutral_rotvecs:
            neutral_matrix = mu.rotvec_to_rotation_matrix(neutral_rotvec)
            neutral_wxyz = mu.matrix_to_quat_wxyz(neutral_matrix).tolist()
            for axis in np.eye(3):
                anatomical_axis_motion = mu.rotvec_to_rotation_matrix(axis * 0.1)
                current_matrix = neutral_matrix @ anatomical_axis_motion
                current_wxyz = mu.matrix_to_quat_wxyz(current_matrix).tolist()
                delta = orientation_delta_wxyz(current_wxyz, neutral_wxyz)
                anatomical_rotvec = mu.rotation_matrix_to_rotvec(
                    mu.quat_wxyz_to_matrix(np.asarray(delta))
                )
                np.testing.assert_allclose(anatomical_rotvec, axis * 0.1, atol=1e-6)

    def test_multi_pose_calibration_recovers_power_cycle_imu_basis(self):
        boot_basis = mu.rotvec_to_rotation_matrix(np.asarray([0.42, -0.31, 0.27]))
        neutral = {side: [1.0, 0.0, 0.0, 0.0] for side in ("left", "right")}
        axis_poses = {axis_name: {} for axis_name in ("x", "y", "z")}
        action_signs = {
            "left": np.asarray([-1.0, 1.0, 1.0]),
            "right": np.asarray([1.0, -1.0, 1.0]),
        }
        for side, signs in action_signs.items():
            for index, axis_name in enumerate(("x", "y", "z")):
                anatomical_pose = mu.rotvec_to_rotation_matrix(
                    np.eye(3)[index] * signs[index] * math.radians(30.0)
                )
                measured_pose = boot_basis.T @ anatomical_pose @ boot_basis
                axis_poses[axis_name][side] = mu.matrix_to_quat_wxyz(
                    measured_pose
                ).tolist()

        corrections = estimate_imu_frame_corrections(neutral, axis_poses)

        np.testing.assert_allclose(corrections["left"], boot_basis, atol=1e-6)
        anatomical_motion = mu.rotvec_to_rotation_matrix(
            np.deg2rad([18.0, -12.0, 9.0])
        )
        measured_motion = boot_basis.T @ anatomical_motion @ boot_basis
        corrected = orientation_delta_wxyz(
            mu.matrix_to_quat_wxyz(measured_motion).tolist(),
            neutral["left"],
            corrections["left"],
        )
        np.testing.assert_allclose(
            mu.quat_wxyz_to_matrix(np.asarray(corrected)),
            anatomical_motion,
            atol=1e-6,
        )

    def test_multi_pose_calibration_rejects_unmoved_axis(self):
        neutral = {side: [1.0, 0.0, 0.0, 0.0] for side in ("left", "right")}
        axis_poses = {
            axis: {
                side: mu.matrix_to_quat_wxyz(
                    mu.rotvec_to_rotation_matrix(vector * angle)
                ).tolist()
                for side in ("left", "right")
            }
            for axis, vector, angle in (
                ("x", np.asarray([1.0, 0.0, 0.0]), math.radians(30.0)),
                ("y", np.asarray([0.0, 1.0, 0.0]), math.radians(3.0)),
                ("z", np.asarray([0.0, 0.0, 1.0]), math.radians(30.0)),
            )
        }

        with self.assertRaisesRegex(RuntimeError, "Y calibration span is too small"):
            estimate_imu_frame_corrections(neutral, axis_poses)

    def test_axis_decoupling_removes_residual_cross_axis_components(self):
        neutral = {side: [1.0, 0.0, 0.0, 0.0] for side in ("left", "right")}
        measured = (
            np.asarray([1.0, 0.18, -0.08]),
            np.asarray([0.16, 1.0, 0.12]),
            np.asarray([-0.10, 0.14, 1.0]),
        )
        axis_poses = {axis: {} for axis in ("x", "y", "z")}
        signs = {"left": (-1.0, 1.0, 1.0), "right": (1.0, -1.0, 1.0)}
        for side in ("left", "right"):
            for index, axis in enumerate(("x", "y", "z")):
                vector = measured[index] / np.linalg.norm(measured[index])
                pose = mu.rotvec_to_rotation_matrix(
                    vector * signs[side][index] * math.radians(30.0)
                )
                axis_poses[axis][side] = mu.matrix_to_quat_wxyz(pose).tolist()

        corrections = estimate_imu_frame_corrections(neutral, axis_poses)
        decoupling = estimate_imu_axis_decoupling(
            neutral, axis_poses, corrections, {"left": False, "right": False}
        )
        for side in ("left", "right"):
            for index, axis in enumerate(("x", "y", "z")):
                delta = orientation_delta_wxyz(
                    axis_poses[axis][side],
                    neutral[side],
                    corrections[side],
                    decoupling[side],
                )
                rotvec = rotation_vector_wxyz(delta)
                direction = rotvec / np.linalg.norm(rotvec)
                expected = np.eye(3)[index] * signs[side][index]
                np.testing.assert_allclose(direction, expected, atol=1e-6)

    def test_left_runtime_does_not_apply_nonorthogonal_axis_decoupling(self):
        decoupling = np.asarray(
            [[1.0, 0.14, 0.14], [0.14, 1.0, 0.06], [0.14, 0.06, 1.0]]
        )

        self.assertIsNone(runtime_axis_decoupling("left", decoupling))
        self.assertIs(runtime_axis_decoupling("right", decoupling), decoupling)
        self.assertIsNone(
            runtime_axis_decoupling("left", decoupling, "static-orthogonal")
        )
        self.assertIs(
            runtime_axis_decoupling("left", decoupling, "static-linear"),
            decoupling,
        )

    def test_multi_pose_calibration_reports_axis_separations(self):
        neutral = {side: [1.0, 0.0, 0.0, 0.0] for side in ("left", "right")}
        close_y_to_x = np.asarray([0.94, 0.342, 0.0])
        axis_poses = {
            axis: {
                side: mu.matrix_to_quat_wxyz(
                    mu.rotvec_to_rotation_matrix(vector * math.radians(30.0))
                ).tolist()
                for side in ("left", "right")
            }
            for axis, vector in (
                ("x", np.asarray([1.0, 0.0, 0.0])),
                ("y", close_y_to_x),
                ("z", np.asarray([0.0, 0.0, 1.0])),
            )
        }

        with self.assertRaisesRegex(RuntimeError, "axis separations XY/XZ/YZ"):
            estimate_imu_frame_corrections(neutral, axis_poses)

    def test_right_only_calibration_uses_right_signs_for_mirrored_samples(self):
        boot_basis = mu.rotvec_to_rotation_matrix(np.asarray([-0.28, 0.35, 0.19]))
        neutral = {side: [1.0, 0.0, 0.0, 0.0] for side in ("left", "right")}
        axis_poses = {}
        right_action_signs = np.asarray([1.0, -1.0, 1.0])
        for index, axis_name in enumerate(("x", "y", "z")):
            anatomical_pose = mu.rotvec_to_rotation_matrix(
                np.eye(3)[index] * right_action_signs[index] * math.radians(30.0)
            )
            measured_pose = boot_basis.T @ anatomical_pose @ boot_basis
            quat = mu.matrix_to_quat_wxyz(measured_pose).tolist()
            # --single-side right deliberately mirrors this packet internally.
            axis_poses[axis_name] = {"left": quat, "right": quat}

        corrections = estimate_imu_frame_corrections(
            neutral, axis_poses, single_side="right"
        )

        np.testing.assert_allclose(corrections["right"], boot_basis, atol=1e-6)
        np.testing.assert_allclose(corrections["left"], boot_basis, atol=1e-6)

    def test_right_calibration_selects_opposite_reconstructed_w_branch(self):
        neutral_quat = [0.666, -0.343, 0.659, 0.075]
        pose_quats = {
            "x": [0.746, -0.157, 0.629, -0.155],
            "y": [0.806, -0.357, 0.469, 0.045],
            "z": [0.565, -0.503, 0.644, -0.114],
        }
        neutral = {side: neutral_quat for side in ("left", "right")}
        axis_poses = {
            axis: {side: quat for side in ("left", "right")}
            for axis, quat in pose_quats.items()
        }
        selected = {}

        corrections = estimate_imu_frame_corrections(
            neutral,
            axis_poses,
            single_side="right",
            repaired_imu_sides={"right"},
            selected_w_branches=selected,
        )

        self.assertTrue(selected["right"])
        self.assertTrue(selected["left"])
        corrected_neutral = opposite_reconstructed_w_branch_wxyz(
            neutral_quat, "unity_to_ros"
        )
        for axis, expected in zip(("x", "y", "z"), np.diag([1.0, -1.0, 1.0]).T):
            corrected_pose = opposite_reconstructed_w_branch_wxyz(
                pose_quats[axis], "unity_to_ros"
            )
            delta = orientation_delta_wxyz(
                corrected_pose, corrected_neutral, corrections["right"]
            )
            direction = rotation_vector_wxyz(delta)
            direction /= np.linalg.norm(direction)
            self.assertGreater(float(np.dot(direction, expected)), math.cos(math.radians(9.0)))

    def test_body_maps_senseglove_anatomical_axes_to_robot_palm_axes(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        imu_map = np.asarray(
            device._cfg.hand_imu_local_rotvec_map_for("left"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": imu_map, "right": imu_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False
        glove_angles = np.asarray([0.1, 0.2, 0.3])
        source_delta = (
            mu.rotvec_to_rotation_matrix([glove_angles[0], 0.0, 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, glove_angles[1], 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, 0.0, glove_angles[2]])
        )
        quat = mu.matrix_to_quat_wxyz(source_delta)
        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"left": quat.tolist()},
            "hand_orientation_delta_order": "wxyz",
        })
        robot_rotvec = mu.rotation_matrix_to_rotvec(parsed["left"])
        np.testing.assert_allclose(robot_rotvec, [0.2, 0.3, 0.1], atol=1e-6)

    def test_left_local_increment_changes_only_the_moved_anatomical_axis(self):
        cfg = build_config().retarget
        cfg.hand_imu_max_step_deg = 0.0
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        left_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("left"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": left_map, "right": left_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False

        initial = np.deg2rad([20.0, -15.0, 10.0])
        source = (
            mu.rotvec_to_rotation_matrix([initial[0], 0.0, 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, initial[1], 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, 0.0, initial[2]])
        )

        def parse(rotation):
            quat = mu.matrix_to_quat_wxyz(rotation)
            parsed = device._parse_hand_orientation_deltas({
                "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
                "hand_orientation_deltas": {"left": quat.tolist()},
                "hand_orientation_delta_order": "wxyz",
            })
            robot_components = mu.rotation_matrix_to_rotvec(parsed["left"])
            return np.linalg.solve(left_map, robot_components)

        np.testing.assert_allclose(parse(source), initial, atol=1e-6)
        moved = source @ mu.rotvec_to_rotation_matrix([0.0, np.deg2rad(8.0), 0.0])
        expected = initial.copy()
        expected[1] += np.deg2rad(8.0)
        np.testing.assert_allclose(parse(moved), expected, atol=1e-6)

    def test_right_imu_map_can_be_tuned_without_changing_left(self):
        cfg = build_config().retarget
        shared = np.asarray(cfg.hand_imu_local_rotvec_to_robot_hand, dtype=np.float32)
        original_left = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("left"), dtype=np.float32
        ).reshape(3, 3).copy()
        cfg.right_hand_imu_local_rotvec_to_robot_hand = tuple((-shared).tolist())

        left_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("left"), dtype=np.float32
        ).reshape(3, 3)
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)
        np.testing.assert_allclose(left_map, original_left)
        np.testing.assert_allclose(right_map.reshape(-1), -shared)

    def test_right_measured_axes_follow_latest_isolated_arm_correction(self):
        cfg = build_config().retarget
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)

        # X -> palm Z (J5), Y -> palm X (J6), Z -> palm Y (J7).
        np.testing.assert_allclose(right_map @ [0.1, 0.0, 0.0], [0.0, 0.0, 0.1])
        np.testing.assert_allclose(right_map @ [0.0, 0.1, 0.0], [0.1, 0.0, 0.0])
        np.testing.assert_allclose(right_map @ [0.0, 0.0, 0.1], [0.0, 0.1, 0.0])

    def test_right_xyz_decomposition_separates_anatomical_axes(self):
        cfg = build_config().retarget
        angles = np.asarray([0.25, -0.20, 0.15])
        source_delta = (
            mu.rotvec_to_rotation_matrix(np.asarray([angles[0], 0.0, 0.0]))
            @ mu.rotvec_to_rotation_matrix(np.asarray([0.0, angles[1], 0.0]))
            @ mu.rotvec_to_rotation_matrix(np.asarray([0.0, 0.0, angles[2]]))
        )
        np.testing.assert_allclose(
            mu.rotation_matrix_to_xyz_angles(source_delta), angles, atol=1e-6
        )

        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        device._hand_imu_local_rotvec_maps = {
            side: np.asarray(cfg.hand_imu_local_rotvec_map_for(side), dtype=np.float32).reshape(3, 3)
            for side in ("left", "right")
        }
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False
        quat = mu.matrix_to_quat_wxyz(source_delta)
        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"right": quat.tolist()},
            "hand_orientation_delta_order": "wxyz",
        })
        robot_components = mu.rotation_matrix_to_rotvec(parsed["right"])
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)
        np.testing.assert_allclose(robot_components, right_map @ angles, atol=1e-6)

    def test_right_pure_axes_change_only_their_declared_components(self):
        for axis_index in range(3):
            expected = np.zeros(3)
            expected[axis_index] = 0.3
            rotation = mu.rotvec_to_rotation_matrix(expected)
            components = mu.rotation_matrix_to_xyz_angles(rotation)
            np.testing.assert_allclose(components, expected, atol=1e-6)

    def test_right_axis_limit_does_not_scale_other_targets(self):
        cfg = build_config().retarget
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        device._hand_imu_local_rotvec_maps = {
            side: np.asarray(cfg.hand_imu_local_rotvec_map_for(side), dtype=np.float32).reshape(3, 3)
            for side in ("left", "right")
        }
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False
        angles = np.deg2rad([50.0, 10.0, -8.0])
        source_delta = (
            mu.rotvec_to_rotation_matrix([angles[0], 0.0, 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, angles[1], 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, 0.0, angles[2]])
        )
        quat = mu.matrix_to_quat_wxyz(source_delta)

        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"right": quat.tolist()},
            "hand_orientation_delta_order": "wxyz",
        })

        target = mu.rotation_matrix_to_rotvec(parsed["right"])
        np.testing.assert_allclose(
            np.rad2deg(target), [10.0, -8.0, 45.0], atol=1e-4
        )

    def test_right_xyz_channels_stay_fixed_when_one_compound_angle_changes(self):
        fixed = np.deg2rad([25.0, -18.0, 12.0])

        def compose(angles):
            return (
                mu.rotvec_to_rotation_matrix([angles[0], 0.0, 0.0])
                @ mu.rotvec_to_rotation_matrix([0.0, angles[1], 0.0])
                @ mu.rotvec_to_rotation_matrix([0.0, 0.0, angles[2]])
            )

        for changed_axis in (1, 2):
            moved = fixed.copy()
            moved[changed_axis] += np.deg2rad(20.0)
            before = mu.rotation_matrix_to_xyz_angles(compose(fixed))
            after = mu.rotation_matrix_to_xyz_angles(compose(moved))
            unchanged_axes = [index for index in range(3) if index != changed_axis]
            np.testing.assert_allclose(
                after[unchanged_axes], fixed[unchanged_axes], atol=1e-6
            )

    def test_right_local_increment_changes_only_the_moved_anatomical_axis(self):
        cfg = build_config().retarget
        cfg.hand_imu_max_step_deg = 0.0
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": right_map, "right": right_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False

        initial = np.deg2rad([20.0, -15.0, 10.0])
        source = (
            mu.rotvec_to_rotation_matrix([initial[0], 0.0, 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, initial[1], 0.0])
            @ mu.rotvec_to_rotation_matrix([0.0, 0.0, initial[2]])
        )

        def parse(rotation):
            quat = mu.matrix_to_quat_wxyz(rotation)
            parsed = device._parse_hand_orientation_deltas({
                "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
                "hand_orientation_deltas": {"right": quat.tolist()},
                "hand_orientation_delta_order": "wxyz",
            })
            robot_components = mu.rotation_matrix_to_rotvec(parsed["right"])
            return np.linalg.solve(right_map, robot_components)

        np.testing.assert_allclose(parse(source), initial, atol=1e-6)
        moved = source @ mu.rotvec_to_rotation_matrix([0.0, np.deg2rad(8.0), 0.0])
        expected = initial.copy()
        expected[1] += np.deg2rad(8.0)
        np.testing.assert_allclose(parse(moved), expected, atol=1e-6)

    def test_right_accepts_large_and_discontinuous_imu_samples(self):
        cfg = build_config().retarget
        cfg.hand_imu_max_step_deg = 0.0
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": right_map, "right": right_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False

        def parse(angle_deg):
            rotation = mu.rotvec_to_rotation_matrix(
                [np.deg2rad(angle_deg), 0.0, 0.0]
            )
            quat = mu.matrix_to_quat_wxyz(rotation)
            return device._parse_hand_orientation_deltas({
                "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
                "hand_orientation_deltas": {"right": quat.tolist()},
                "hand_orientation_delta_order": "wxyz",
            })

        self.assertIn("right", parse(5.0))
        self.assertIn("right", parse(30.0))

        fresh_device = BodyDevice.__new__(BodyDevice)
        fresh_device._cfg = cfg
        fresh_device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        fresh_device._hand_imu_local_rotvec_maps = {"left": right_map, "right": right_map}
        fresh_device._hand_orientation_delta_matrices = {}
        fresh_device._printed_hand_imu_frame_mismatch = False
        rotation = mu.rotvec_to_rotation_matrix([np.deg2rad(100.0), 0.0, 0.0])
        quat = mu.matrix_to_quat_wxyz(rotation)
        parsed = fresh_device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"right": quat.tolist()},
            "hand_orientation_delta_order": "wxyz",
        })
        self.assertIn("right", parsed)

    def test_right_accepts_large_captured_delta_after_calibration(self):
        cfg = build_config().retarget
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        right_map = np.asarray(
            cfg.hand_imu_local_rotvec_map_for("right"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": right_map, "right": right_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False

        captured_delta = mu.rotvec_to_rotation_matrix(
            np.deg2rad([-3.6299276, 4.0062876, -68.880714])
        )
        quat = mu.matrix_to_quat_wxyz(captured_delta)
        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"right": quat.tolist()},
            "hand_orientation_delta_order": "wxyz",
        })
        self.assertIn("right", parsed)

    def test_anatomical_z_flexion_maps_only_to_terminal_j7(self):
        cfg = build_config()
        neutral = mu.rotvec_to_rotation_matrix(np.asarray([0.37, -0.51, 0.82]))
        flexion = mu.rotvec_to_rotation_matrix(np.asarray([0.0, 0.0, 0.12]))
        current = neutral @ flexion
        delta = orientation_delta_wxyz(
            mu.matrix_to_quat_wxyz(current).tolist(),
            mu.matrix_to_quat_wxyz(neutral).tolist(),
        )

        device = BodyDevice.__new__(BodyDevice)
        device._cfg = cfg.retarget
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        imu_map = np.asarray(
            cfg.retarget.hand_imu_local_rotvec_map_for("left"), dtype=np.float32
        ).reshape(3, 3)
        device._hand_imu_local_rotvec_maps = {"left": imu_map, "right": imu_map}
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False
        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"left": delta},
            "hand_orientation_delta_order": "wxyz",
        })

        # At terminal zero the physical J5/J6/J7 axes are palm Z/X/Y.
        terminal_axes = np.asarray(
            [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]
        )
        offsets = imu_local_rotation_to_wrist_offsets(
            parsed["left"], cfg.robot, terminal_axes
        )
        np.testing.assert_allclose(offsets, [0.0, 0.0, 0.12], atol=1e-6)

    def test_body_rejects_nonfinite_imu_quaternion(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._hand_imu_rotation = np.eye(3, dtype=np.float32)
        device._hand_imu_local_rotvec_maps = {
            "left": np.eye(3, dtype=np.float32),
            "right": np.eye(3, dtype=np.float32),
        }
        device._hand_orientation_delta_matrices = {}
        device._printed_hand_imu_frame_mismatch = False
        parsed = device._parse_hand_orientation_deltas({
            "hand_orientation_source_frame": "senseglove_zeroed_anatomical_axes",
            "hand_orientation_deltas": {"left": [float("nan"), 0.0, 0.0, 1.0]},
        })
        self.assertEqual(parsed, {})

    def test_enable_reference_produces_no_orientation_jump(self):
        device = BodyDevice.__new__(BodyDevice)
        device._cfg = build_config().retarget
        device._last_hand_orientation_packet_time_monotonic = time.monotonic()
        first = mu.rotvec_to_rotation_matrix(np.asarray([0.1, -0.2, 0.05]))
        device._hand_orientation_delta_matrices = {"right": first}
        device._hand_imu_reference_delta_matrices = {}
        initial = mu.rotvec_to_rotation_matrix(np.asarray([0.3, 0.1, -0.2]))
        device._initial_right_pose_matrix = np.eye(4, dtype=np.float32)
        device._initial_right_pose_matrix[:3, :3] = initial

        at_enable = device._compose_wrist_rotation("right", None)
        np.testing.assert_allclose(at_enable, initial, atol=1e-6)

        relative_motion = mu.rotvec_to_rotation_matrix(np.asarray([0.0, 0.08, 0.0]))
        device._hand_orientation_delta_matrices["right"] = first @ relative_motion
        after_motion = device._compose_wrist_rotation("right", None)
        np.testing.assert_allclose(after_motion, initial @ relative_motion, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
