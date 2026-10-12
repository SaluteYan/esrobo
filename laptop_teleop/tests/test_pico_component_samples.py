"""Regression coverage for real packet receipt versus repeated pose/cache reads."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
from types import SimpleNamespace

import pytest

from esrobo_laptop.acquisition import load_bridge, pico
from esrobo_teleop.debug.skeleton_viewer import SkeletonViewerState

ROOT = Path(__file__).resolve().parents[2]


def sample(**changes):
    return dict(dict(valid=True, sequence=1, age_s=.02,
                     packet_timestamp_ns=100, poses=[[1, 2, 3, 0, 0, 0, 1]]*24), **changes)


def test_stationary_body_uses_component_sequence_not_pose_or_global_xr(monkeypatch):
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    current = sample()
    sdk = SimpleNamespace(get_body_sample=lambda: current)
    monkeypatch.setattr(bridge.time, 'monotonic', lambda: 10.)
    first, count = bridge._build_packet(sdk, 1, True)
    assert count == 24
    assert first['body_sample_monotonic'] == pytest.approx(9.98)
    assert 'xrt_body_timestamp_ns' not in first
    assert first['xrt_body_packet_timestamp_ns'] == 100
    repeated, _ = bridge._build_packet(sdk, 2, False)
    assert bridge._packet_signature(first) == bridge._packet_signature(repeated)
    current.update(sequence=2, packet_timestamp_ns=101)
    fresh, _ = bridge._build_packet(sdk, 3, False)
    assert fresh['frames'] == first['frames']
    assert bridge._packet_signature(first) != bridge._packet_signature(fresh)
    assert SkeletonViewerState._signature(first) != SkeletonViewerState._signature(fresh)
    current['age_s'] = .101
    assert bridge._build_packet(sdk, 4, False) == (None, 0)


@pytest.mark.parametrize('age', [float('nan'), -.001, .101])
def test_invalid_age_does_not_create_body_packet(age):
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    assert bridge._build_packet(SimpleNamespace(get_body_sample=lambda: sample(age_s=age)), 1, False) == (None, 0)


def test_legacy_sdk_fails_with_actionable_rebuild_message():
    bridge = load_bridge('xrobotoolkit_body_udp_bridge')
    with pytest.raises(RuntimeError, match='atomic Body samples'):
        bridge._build_packet(SimpleNamespace(), 1, False)


def test_viewer_includes_sdk_age_and_silent_tail_in_rate(monkeypatch):
    monkeypatch.setattr('esrobo_teleop.debug.skeleton_viewer.time.monotonic', lambda: 10.)
    state = SkeletonViewerState()
    state.update({'body_freshness_source': 'component-packet',
                  'body_sample_sequence': 2, 'body_sample_monotonic': 9.92})
    assert state.snapshot()['age_ms'] == 80.
    state._source_updates.clear()
    state._source_updates.extend([9.2, 9.3, 9.4])
    assert state.snapshot()['source_update_hz'] == 2.5  # Includes .6 s without updates.


def test_acquisition_delegates_without_monkeypatching_body_builder(monkeypatch):
    from unittest.mock import Mock
    bridge = SimpleNamespace(_build_packet=Mock(), parse_args=lambda argv: 'parsed')
    original = bridge._build_packet
    run = Mock(return_value=0)
    monkeypatch.setattr('esrobo_laptop.acquisition.load_bridge', lambda name: bridge)
    monkeypatch.setattr('esrobo_laptop.pico_transport.run_pico', run)
    assert pico([]) == 0
    run.assert_called_once_with(bridge, 'parsed')
    assert bridge._build_packet is original


def test_native_packet_cache(tmp_path):
    compiler = shutil.which('g++')
    include = ROOT/'laptop_teleop/external/XRoboToolkit-PC-Service-Pybind/include'
    if not compiler or not (include/'nlohmann/json.hpp').exists():
        pytest.skip('Native regression needs g++ and the PICO nlohmann headers')
    binary = tmp_path/'component_samples'
    subprocess.run([compiler, '-std=c++17', '-pthread', '-I'+str(include),
                    '-I'+str(ROOT/'laptop_teleop/scripts'),
                    str(Path(__file__).with_name('fixtures')/'pico_component_sample.cpp'),
                    '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)


def test_patch_upgrades_existing_v1_and_is_idempotent(tmp_path):
    vendor = ROOT/'laptop_teleop/external/XRoboToolkit-PC-Service-Pybind'
    if not (vendor/'.git').exists():
        pytest.skip('Patch layout regression needs the vendor checkout')
    source = subprocess.check_output(['git', 'show', 'HEAD:bindings/py_bindings.cpp'], cwd=vendor, text=True)
    spec = importlib.util.spec_from_file_location('pico_patch', ROOT/'laptop_teleop/scripts/patch_pico_sdk.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    path = tmp_path/'py_bindings.cpp'
    path.write_text(source)
    module._patch_hand_v1(path)
    module.patch(path)
    patched = path.read_text()
    module.patch(path)
    assert path.read_text() == patched
    assert patched.count('ComponentTracking.ingest(value, received)') == 1
    assert patched.count('m.def("get_body_sample"') == 1
    assert 'BodyTimeStampNs' not in patched
    assert 'BodyJointsTimestamp' not in patched
    assert 'm.def("get_body_timestamp_ns"' not in patched
    assert path.with_name('pico_component_sample.hpp').exists()
    assert patched.count('m.def("wait_tracking_updates"') == 1
    assert 'gil_scoped_release' in patched
    assert 'received_monotonic_ns' in patched
