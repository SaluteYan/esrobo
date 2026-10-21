import json
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
