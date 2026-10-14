"""Deployment tests use a local fake SFTP filesystem; no hardware connections."""
import hashlib
import os
from pathlib import Path
import subprocess
import threading
from types import SimpleNamespace

import pytest

from esrobo_laptop.robot_maintenance import sync_sources


class LocalSftp:
    def __enter__(self): return self
    def __exit__(self, *args): pass
    def stat(self, p): return Path(p).stat()
    def mkdir(self, p): Path(p).mkdir()
    def open(self, p, mode): return open(p, mode)
    def chmod(self, p, mode): os.chmod(p, mode)
    def posix_rename(self, a, b): os.replace(a, b)


def setup(tmp_path, monkeypatch):
    workspace = tmp_path/'local'
    remote = tmp_path/'remote'
    path = 'robot_link/esrobo_link/backends.py'
    for root in (workspace, remote):
        (root/path).parent.mkdir(parents=True)
    (workspace/path).write_bytes(b'# new code\n')
    (remote/path).write_bytes(b'# old code\n')
    real_output = subprocess.check_output
    def output(args, **kwargs):
        if args[0] == 'git': return (path+'\0').encode()
        return real_output(args, **kwargs)
    monkeypatch.setattr('esrobo_laptop.robot_maintenance.subprocess.check_output', output)
    c = SimpleNamespace(remote_lock=threading.Lock(),
        ssh=SimpleNamespace(open_sftp=LocalSftp),
        command=lambda args, **kwargs: real_output(args).decode())
    return c, workspace, remote, path


def test_sync_backs_up_replaces_and_verifies_only_manifest_sources(tmp_path, monkeypatch):
    c, workspace, remote, path = setup(tmp_path, monkeypatch)
    (workspace/'secret.key').write_text('never upload')
    result = sync_sources(c, workspace, str(remote))
    assert result['changed'] == 1
    assert (remote/path).read_bytes() == b'# new code\n'
    assert (Path(result['backup'])/'old'/path).read_bytes() == b'# old code\n'
    assert not (remote/'secret.key').exists()
    assert sync_sources(c, workspace, str(remote))['changed'] == 0


def test_corrupted_upload_is_not_installed(tmp_path, monkeypatch):
    c, workspace, remote, path = setup(tmp_path, monkeypatch)
    class CorruptSftp(LocalSftp):
        def chmod(self, p, mode):
            Path(p).write_bytes(b'# damaged upload\n')
    c.ssh.open_sftp = CorruptSftp
    with pytest.raises(RuntimeError, match='上传校验失败'):
        sync_sources(c, workspace, str(remote))
    assert (remote/path).read_bytes() == b'# old code\n'


def test_symlink_source_is_not_uploaded(tmp_path, monkeypatch):
    c, workspace, remote, path = setup(tmp_path, monkeypatch)
    (workspace/'secret.key').write_text('never upload')
    (workspace/path).unlink()
    (workspace/path).symlink_to(workspace/'secret.key')
    with pytest.raises(RuntimeError, match='非普通项目文件'):
        sync_sources(c, workspace, str(remote))
    assert (remote/path).read_bytes() == b'# old code\n'
