import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from profile_runtime import Profile


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
                assert node.module not in ('profile_runtime','audit_transfers'),path
            if isinstance(node,ast.Import):
                assert all(a.name not in ('profile_runtime','audit_transfers') for a in node.names),path


def test_runtime_diagnostics_has_no_durations(tmp_path):
    from esrobo_laptop.diagnostics import RuntimeDiagnostics
    log=RuntimeDiagnostics(tmp_path/'run.jsonl')
    log.submit(dict(monotonic_s=10.,record_type='terminal_fault',status='original fault'),fault=True)
    assert log.close()
    record=json.loads(log.path.read_text())
    assert 'timings' not in record
    assert not any(k.startswith('write_ms') for k in record['diagnostics'])
    assert set(record['diagnostics'])=={'files','console','errors','serialization_errors'}
