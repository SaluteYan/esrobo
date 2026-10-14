"""Maintain completed local logs outside control loops; dry-run CLI by default."""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import stat
import threading
import time


STATE_FILES = {'dashboard_teleop.json', 'dashboard_waist_moved.json',
               'COLCON_IGNORE', 'CACHEDIR.TAG', '.keep'}
SERIES = re.compile(r'(.+)_\d{16,20}\.jsonl$')
LOG_DIRS = ('log', 'laptop_teleop/log', 'teleoperation/log', 'robot_link/log')


def open_file_ids():
    """Protect open inodes, including a log opened through a different name."""
    opened = set()
    for process in Path('/proc').glob('[0-9]*'):
        try:
            for fd in (process / 'fd').iterdir():
                try:
                    s = fd.stat()
                    opened.add((s.st_dev, s.st_ino))
                except OSError:
                    pass
        except OSError:
            pass
    return opened


def clean_logs(root, *, apply=False, now=None, days=7, max_bytes=512*1024**2,
               keep_per_series=10, grace_s=3600, opened=None):
    """Age/count/size retention; never unlink active, pinned or changing files.

    A directory containing .keep protects its whole subtree. Symlinks are never
    followed. The size target is soft: protected files always take precedence.
    """
    root = Path(root)
    if root.name != 'log' or root.is_symlink():
        raise ValueError('retention requires a real log directory')
    if days <= 0 or max_bytes <= 0 or keep_per_series < 1 or grace_s < 0:
        raise ValueError('invalid retention policy')
    now = time.time() if now is None else now
    opened = open_file_ids() if opened is None else opened
    files = []
    directories = []
    for directory, subdirs, names in os.walk(root, followlinks=False):
        directory = Path(directory)
        subdirs[:] = [n for n in subdirs if not (directory / n).is_symlink()]
        if '.keep' in names:
            subdirs[:] = []
            continue
        directories.append(directory)
        for name in names:
            path = directory / name
            try:
                s = path.lstat()
            except OSError:
                continue
            if not stat.S_ISREG(s.st_mode):
                continue
            protected = (name in STATE_FILES or now-s.st_mtime < grace_s
                         or (s.st_dev, s.st_ino) in opened)
            files.append((path, s, protected))
    files.sort(key=lambda row: row[1].st_mtime, reverse=True)
    reasons = {}
    groups = defaultdict(int)
    for path, s, protected in files:
        match = SERIES.fullmatch(path.name)
        excess = False
        if match:
            key = (path.parent, match[1])
            groups[key] += 1
            limit = 3 if match[1] == 'pico_transport' else keep_per_series
            excess = groups[key] > limit
        if not protected:
            if now-s.st_mtime > days*86400:
                reasons[path] = 'age'
            elif excess:
                reasons[path] = 'count'
    retained_bytes = sum(s.st_size for p, s, _ in files if p not in reasons)
    for path, s, protected in reversed(files):
        if retained_bytes <= max_bytes:
            break
        if not protected and path not in reasons:
            reasons[path] = 'size'
            retained_bytes -= s.st_size
    deleted, skipped = [], []
    # Rescan immediately before mutation, then compare each file's identity and
    # write metadata. A file changed since planning is deferred to the next pass.
    if apply:
        opened = opened | open_file_ids()
    for path, before, _ in files:
        if path not in reasons:
            continue
        row = dict(path=str(path), bytes=before.st_size, reason=reasons[path])
        if apply:
            try:
                after = path.lstat()
                parents = (path.parent, *path.parent.parents)
                in_root = parents[:parents.index(root)+1]
                if (any(p.is_symlink() or (p / '.keep').exists() for p in in_root)
                        or not stat.S_ISREG(after.st_mode)
                        or (after.st_dev, after.st_ino) in opened
                        or (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns)
                        != (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns, before.st_ctime_ns)):
                    skipped.append(str(path))
                    continue
                path.unlink()
            except (OSError, ValueError):
                skipped.append(str(path))
                continue
        deleted.append(row)
    if apply:
        for directory in reversed(directories):
            if directory == root:
                continue
            try:
                directory.rmdir()  # Only empty directories, never recursive deletion.
            except OSError:
                pass
    return dict(root=str(root), apply=apply, files=len(deleted),
                bytes=sum(row['bytes'] for row in deleted), skipped=skipped, deleted=deleted)


def start_log_maintenance(workspace, stopped):
    """One background pass at service startup and hourly until shutdown."""
    def run():
        while not stopped.is_set():
            for relative in LOG_DIRS:
                if stopped.is_set():
                    return
                try:
                    report = clean_logs(Path(workspace) / relative, apply=True)
                    if report['files']:
                        print(f"[logs] {relative}: removed {report['files']} completed files "
                              f"({report['bytes']/1024**2:.1f} MiB)", flush=True)
                except Exception as exc:
                    print(f'[logs] retention deferred: {type(exc).__name__}', flush=True)
            stopped.wait(3600)
    thread = threading.Thread(target=run, name='log-retention', daemon=True)
    thread.start()
    return thread


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, default=Path.cwd())
    parser.add_argument('--apply', action='store_true', help='delete eligible completed logs')
    args = parser.parse_args()
    opened = open_file_ids()
    reports = [clean_logs(args.workspace.resolve() / p, apply=args.apply, opened=opened)
               for p in LOG_DIRS]
    print(json.dumps(reports, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
