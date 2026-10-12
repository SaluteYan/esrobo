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


def packet(*, sequence=1, x=0.1):
    return {"body_sample_sequence": sequence,
            "frames": {"right_elbow": {"pos": [x, .2, .3], "quat_xyzw": [0, 0, 0, 1]}}}


def test_pose_change_cannot_renew_cached_body_sequence():
    assert bridge._packet_signature(packet(x=.1)) == bridge._packet_signature(packet(x=.11))


def test_static_pose_advances_with_new_component_sequence():
    assert bridge._packet_signature(packet(sequence=1)) != bridge._packet_signature(packet(sequence=2))


def test_receipt_sequence_is_required():
    import pytest
    with pytest.raises(KeyError):
        bridge._packet_signature({"frames": {}})


def test_packet_uses_atomic_sample_without_legacy_timestamp_getters():
    class FakeXrt:
        def get_body_sample(self):
            return dict(valid=True, sequence=3, age_s=.01, packet_timestamp_ns=999,
                        poses=[[.1, .2, .3, 0., 0., 0., 1.] for _ in range(24)])
        def get_time_stamp_ns(self):
            raise AssertionError("global XR timestamp must not be read")
    built, valid = bridge._build_packet(FakeXrt(), 1, False)
    assert valid == 24
    assert built["xrt_body_packet_timestamp_ns"] == 999
    assert built["body_sample_sequence"] == 3
    assert built["body_freshness_source"] == "component-packet"
    assert "xrt_body_timestamp_ns" not in built
    assert "xrt_body_joint_timestamp_ns" not in built
