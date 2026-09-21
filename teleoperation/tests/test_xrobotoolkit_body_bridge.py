import importlib.util
from pathlib import Path


BRIDGE_PATH = (
    Path(__file__).resolve().parents[1]
    / "bridges"
    / "xrobotoolkit_body_udp_bridge.py"
)
SPEC = importlib.util.spec_from_file_location("xrobotoolkit_body_udp_bridge_test", BRIDGE_PATH)
bridge = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(bridge)


def packet(*, body=0, joint=0, xr=1, x=0.1):
    return {
        "xrt_body_timestamp_ns": body,
        "xrt_body_joint_timestamp_ns": joint,
        "xrt_timestamp_ns": xr,
        "frames": {
            "right_elbow": {
                "pos": [x, 0.2, 0.3],
                "quat_xyzw": [0.0, 0.0, 0.0, 1.0],
            }
        },
    }


def test_generic_xr_timestamp_does_not_make_cached_body_pose_fresh():
    first = bridge._packet_signature(packet(xr=1))
    second = bridge._packet_signature(packet(xr=2))
    assert first == second


def test_pose_change_is_freshness_evidence_when_body_timestamps_are_missing():
    first = bridge._packet_signature(packet(x=0.1))
    second = bridge._packet_signature(packet(x=0.11))
    assert first != second
    assert first[0] == "pose-change"


def test_body_and_joint_timestamps_take_precedence_over_generic_xr_timestamp():
    assert bridge._packet_signature(packet(body=10, xr=1)) == ("body-timestamp", 10)
    assert bridge._packet_signature(packet(joint=20, xr=1)) == ("joint-timestamp", 20)


def test_packet_ignores_unrelated_joint_timestamp_for_arm_freshness():
    class FakeXrt:
        def is_body_data_available(self):
            return True

        def get_body_joints_pose(self):
            return [[0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0] for _ in range(24)]

        def get_body_timestamp_ns(self):
            return 0

        def get_time_stamp_ns(self):
            return 999

        def get_body_joints_timestamp(self):
            result = [0] * 24
            result[2] = 123  # A non-arm cache entry must not refresh teleoperation.
            return result

    built, valid = bridge._build_packet(FakeXrt(), 1, False)
    assert valid == 24
    assert built["xrt_body_joint_timestamp_ns"] == 0
    assert built["body_freshness_source"] == "pose-change"
