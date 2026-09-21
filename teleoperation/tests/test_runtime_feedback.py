import unittest
from types import SimpleNamespace
from unittest import mock

import numpy as np

from esrobo_teleop.robot.nero_driver import NeroArm, JointFeedbackSnapshot


class RuntimeFeedbackTests(unittest.TestCase):
    def arm(self):
        arm = NeroArm.__new__(NeroArm)
        arm._robot = mock.Mock()
        arm._feedback_source = "get_joint_angles"
        arm._runtime_feedback = JointFeedbackSnapshot(np.zeros(7), 100., 20., "get_joint_angles")
        arm._last_feedback_timestamp = 100.
        arm._runtime_feedback_recovery_candidate = None
        arm._robot.get_joint_angles.return_value = SimpleNamespace(msg=np.zeros(7), timestamp=100.02)
        return arm

    @mock.patch("esrobo_teleop.robot.nero_driver.time.sleep")
    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.02)
    def test_new_frame_nonblocking_and_duplicate_does_not_refresh_age(self, clock, sleep):
        arm = self.arm()
        first = arm.runtime_feedback()
        clock.return_value = 20.05
        self.assertIs(arm.runtime_feedback(), first)
        self.assertEqual(first.received_monotonic, 20.02)
        self.assertEqual(arm.last_feedback_timestamp, 100.02)
        arm._robot.get_leader_joint_angles.assert_not_called()
        sleep.assert_not_called()

    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.02)
    def test_missing_pinned_source_never_falls_back(self, clock):
        arm = self.arm()
        arm._robot.get_joint_angles.return_value = None
        self.assertIsNone(arm.runtime_feedback())
        arm._robot.get_leader_joint_angles.assert_not_called()

    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.02)
    def test_invalid_frames_do_not_update_snapshot(self, clock):
        for values, stamp in ((np.zeros(6),100.02), (np.full(7,np.nan),100.02),
                              (np.ones(7),100.02), (np.zeros(7),99.99),
                              (np.zeros(7),100.11), (np.zeros(7),float('nan'))):
            arm = self.arm()
            old = arm._runtime_feedback
            arm._robot.get_joint_angles.return_value = SimpleNamespace(msg=values, timestamp=stamp)
            self.assertIsNone(arm.runtime_feedback())
            self.assertIs(arm._runtime_feedback, old)
            self.assertIsNotNone(arm.runtime_feedback_failure)

    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.02)
    def test_partial_can_group_update_with_same_j7_stamp_is_ignored(self, clock):
        arm = self.arm()
        old = arm._runtime_feedback
        arm._robot.get_joint_angles.return_value = SimpleNamespace(msg=np.ones(7)*.01, timestamp=100.)
        self.assertIs(arm.runtime_feedback(), old)
        self.assertEqual(arm.last_feedback_timestamp, 100.)
        self.assertEqual(old.received_monotonic, 20.)
        self.assertIsNone(arm.runtime_feedback_failure)
        arm._robot.get_joint_angles.return_value.timestamp = 100.02
        updated = arm.runtime_feedback()
        np.testing.assert_allclose(updated.position, .01)
        self.assertEqual(updated.stamp, 100.02)
        clock.return_value = 20.13
        self.assertIsNone(arm.runtime_feedback())
        self.assertTrue(arm.runtime_feedback_failure["transient"])
        self.assertIn("waiting", arm.runtime_feedback_failure["reason"])

    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.11)
    def test_gap_requires_two_consecutive_fresh_packets(self, clock):
        arm = self.arm()
        arm._robot.get_joint_angles.return_value = SimpleNamespace(
            msg=np.ones(7) * .01, timestamp=100.12
        )
        self.assertIsNone(arm.runtime_feedback())
        self.assertTrue(arm.runtime_feedback_failure["transient"])
        self.assertEqual(arm.runtime_feedback_failure["recovery_samples"], 1)
        self.assertFalse(arm.runtime_feedback_recovered)
        clock.return_value = 20.16
        arm._robot.get_joint_angles.return_value = SimpleNamespace(
            msg=np.ones(7) * .015, timestamp=100.17
        )
        recovered = arm.runtime_feedback()
        self.assertIsNotNone(recovered)
        self.assertTrue(arm.runtime_feedback_recovered)
        self.assertEqual(recovered.stamp, 100.17)

    @mock.patch("esrobo_teleop.robot.nero_driver.time.monotonic", return_value=20.31)
    def test_gap_beyond_recovery_timeout_is_fatal(self, clock):
        arm = self.arm()
        arm._robot.get_joint_angles.return_value = SimpleNamespace(
            msg=np.zeros(7), timestamp=100.31
        )
        self.assertIsNone(arm.runtime_feedback(recovery_timeout_s=.3))
        self.assertFalse(arm.runtime_feedback_failure["transient"])
        self.assertIn("300 ms", arm.runtime_feedback_failure["reason"])

    @mock.patch("esrobo_teleop.robot.nero_driver.time.sleep")
    def test_startup_requires_two_samples_of_same_source(self, sleep):
        arm = NeroArm.__new__(NeroArm)
        arm._robot = mock.Mock()
        arm._robot.get_joint_angles.side_effect = [SimpleNamespace(msg=np.zeros(7),timestamp=1.), None, None]
        arm._robot.get_leader_joint_angles.side_effect = [None,
            SimpleNamespace(msg=np.zeros(7),timestamp=2.),
            SimpleNamespace(msg=np.zeros(7),timestamp=3.)]
        self.assertIsNotNone(arm.get_joint_angles())
        self.assertEqual(arm._feedback_source, "get_leader_joint_angles")
        self.assertEqual(sleep.call_count, 2)


if __name__ == "__main__":
    unittest.main()
