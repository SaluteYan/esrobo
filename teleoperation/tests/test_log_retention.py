"""Retention tests use temporary logs only, without services or hardware."""
import os
import time

from esrobo_teleop.debug import log_retention as retention


def write(root, name, *, age=7200, size=8):
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'x' * size)
    stamp = time.time() - age
    os.utime(path, (stamp, stamp))
    return path


def test_dry_run_then_apply_expiry_preserves_state_and_pinned_tree(tmp_path):
    root = tmp_path / 'log'
    old = write(root, 'old.jsonl', age=9*86400)
    fresh = write(root, 'new.jsonl')
    state = write(root, 'dashboard_waist_moved.json', age=90*86400)
    pinned = write(root, 'evidence/old.jsonl', age=90*86400)
    write(root, 'evidence/.keep')
    report = retention.clean_logs(root)
    assert report['files'] == 1 and old.exists()
    retention.clean_logs(root, apply=True)
    assert not old.exists()
    assert fresh.exists() and state.exists() and pinned.exists()


def test_count_and_size_limits_keep_newest_and_open_files(tmp_path):
    root = tmp_path / 'log'
    for i in range(6):
        write(root, f'pico_transport_{1000000000000000+i}.jsonl', age=7200-i, size=10)
    active = write(root, 'old_open.log', age=9*86400, size=50)
    with active.open('rb'):
        report = retention.clean_logs(root, apply=True, max_bytes=70)
        assert active.exists()
    assert report['files'] == 4  # Three count removals, then the oldest size removal.
    assert len(list(root.glob('pico_transport_*.jsonl'))) == 2


def test_recently_modified_log_is_protected_even_when_over_budget(tmp_path):
    root = tmp_path / 'log'
    recent = write(root, 'active_without_open_fd.jsonl', age=10, size=100)
    old = write(root, 'completed.jsonl', size=20)
    report = retention.clean_logs(root, apply=True, max_bytes=1)
    assert report['files'] == 1
    assert recent.exists() and not old.exists()


def test_symlinks_never_followed_and_pinned_root_not_pruned(tmp_path):
    root = tmp_path / 'log'
    outside = write(tmp_path / 'elsewhere', 'old.txt', age=30*86400)
    root.mkdir()
    (root / 'outside').symlink_to(outside.parent, target_is_directory=True)
    (root / 'alias.log').symlink_to(outside)
    assert retention.clean_logs(root, apply=True)['files'] == 0
    assert outside.exists() and (root / 'alias.log').is_symlink()
    write(root, '.keep')
    pinned = write(root, 'old.jsonl', age=30*86400)
    assert retention.clean_logs(root, apply=True)['files'] == 0
    assert pinned.exists()


def test_file_changed_after_scan_is_deferred(tmp_path, monkeypatch):
    root = tmp_path / 'log'
    changing = write(root, 'changing.log', age=30*86400)
    def update_before_delete():
        changing.write_text('fresh record appended')
        return set()
    monkeypatch.setattr(retention, 'open_file_ids', update_before_delete)
    report = retention.clean_logs(root, apply=True, opened=set())
    assert report['files'] == 0 and str(changing) in report['skipped']
    assert changing.read_text() == 'fresh record appended'


def test_empty_directories_removed_but_root_and_sentinel_remain(tmp_path):
    root = tmp_path / 'log'
    write(root, 'old-run/output.log', age=30*86400)
    sentinel = write(root, 'COLCON_IGNORE', age=30*86400)
    retention.clean_logs(root, apply=True)
    assert not (root / 'old-run').exists()
    assert root.is_dir() and sentinel.exists()
