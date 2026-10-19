import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.profile_runtime import Profile


def test_native_profile_keeps_coalesced_receipts_once_without_subscribing(tmp_path):
    sdk = SimpleNamespace(get_tracking_diagnostics=lambda: {
        'body': {'committed': 12, 'receipts_ns': [100, 200, 500]}})
    prof = Profile(tmp_path/'native.jsonl')
    prof.native_receipts(sdk)
    prof.native_receipts(sdk)
    sdk.get_tracking_diagnostics = lambda: {
        'body': {'committed': 13, 'receipts_ns': [200, 500, 600]}}
    prof.native_receipts(sdk)
    summary = prof.close()
    records = [json.loads(line) for line in prof.path.read_text().splitlines()]
    assert [r['sequence'] for r in records] == [10, 11, 12, 13]
    assert [r['receipt_ns'] for r in records] == [100, 200, 500, 600]
    assert summary['dropped'] == 0
    assert summary['persistence_complete']


def test_profiling_restores_functions_even_after_failure(tmp_path):
    def fail():raise ValueError('original error')
    owner=SimpleNamespace(work=fail)
    prof=Profile(tmp_path/'measure.jsonl')
    prof.wrap(owner,'work')
    with pytest.raises(ValueError,match='original error'):owner.work()
    summary=prof.close()
    assert owner.work is fail
    assert summary['stages']['work']['calls']==1
    assert summary['stages']['work']['max_ms']>=0
    assert summary['persistence_complete']


def test_production_has_no_performance_instrumentation_or_test_imports():
    root=Path(__file__).resolve().parents[2]
    paths=list((root/'laptop_teleop/src').rglob('*.py'))
    paths+=list((root/'teleoperation/src').rglob('*.py'))
    paths+=list((root/'robot_link/esrobo_link').glob('*.py'))
    for path in paths:
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node,ast.Attribute):
                assert node.attr not in ('perf_counter','percentile'),path
            if isinstance(node,ast.Name):
                assert node.id not in ('CycleTimings','timed_stage','profile_runtime'),path
            if isinstance(node,ast.ImportFrom):
                assert 'tests' not in (node.module or '').split('.'),path
                assert (node.module or '').split('.')[-1] not in ('profile_runtime','audit_transfers'),path
            if isinstance(node,ast.Import):
                assert all(a.name.split('.')[-1] not in ('profile_runtime','audit_transfers') for a in node.names),path


def test_runtime_diagnostics_has_no_durations(tmp_path):
    from esrobo_laptop.diagnostics import RuntimeDiagnostics
    log=RuntimeDiagnostics(tmp_path/'run.jsonl')
    log.submit(dict(monotonic_s=10.,record_type='terminal_fault',status='original fault'),fault=True)
    assert log.close()
    record=json.loads(log.path.read_text())
    assert 'timings' not in record
    assert not any(k.startswith('write_ms') for k in record['diagnostics'])
    assert set(record['diagnostics'])=={'files','console','errors','serialization_errors'}
