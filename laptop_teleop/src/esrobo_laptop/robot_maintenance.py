"""Fixed, non-motion robot diagnostics and source deployment over existing SSH."""
import hashlib
import json
import posixpath
import subprocess
import time


CAN_DIAGNOSTICS = r'''
import collections,json,socket,struct,subprocess,time
result={'links':json.loads(subprocess.check_output(['ip','-details','-statistics','-json','link','show','type','can'])),'frames':{}}
for name in ('can_piper1','can_piper2'):
 counts=collections.Counter()
 try:
  with socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW) as bus:
   bus.bind((name,)); bus.settimeout(.05)
   deadline=time.monotonic()+1.
   while time.monotonic()<deadline:
    try:
     frame=bus.recv(16); ident=struct.unpack('=I',frame[:4])[0]
     counts[hex(ident)]+=1
    except socket.timeout: pass
  result['frames'][name]={'count':sum(counts.values()),'ids':dict(counts)}
 except OSError as exc: result['frames'][name]={'error':str(exc)}
print(json.dumps(result))
'''


def sync_sources(console, workspace, remote):
    """Upload only tracked runtime sources/config; never restart or command devices."""
    paths = subprocess.check_output([
        'git', 'ls-files', '-z', '--', 'robot_link/esrobo_link',
        'teleoperation/src/esrobo_teleop', 'teleoperation/config',
        'teleoperation/urdf/esrobo_waist_with_head.urdf',
    ], cwd=workspace).decode().split('\0')
    sources = {}
    for path in filter(None, paths):
        local = workspace / path
        if local.is_symlink() or not local.is_file() or workspace.resolve() not in local.resolve().parents:
            raise RuntimeError('不能同步非普通项目文件：' + path)
        if local.suffix not in ('.py', '.yaml', '.yml', '.json', '.urdf'):
            continue
        sources[path] = local.read_bytes()
    if not sources:
        raise RuntimeError('未找到受版本控制的运行文件')
    # Hash only these fixed local manifest paths; HTTP cannot choose a path or command.
    script = ('import hashlib,json,pathlib\n'
              f'root=pathlib.Path({remote!r})\n'
              f'paths={list(sources)!r}\n'
              'print(json.dumps({p:hashlib.sha256((root/p).read_bytes()).hexdigest() '
              'if (root/p).is_file() else None for p in paths}))')
    hashes = json.loads(console.command(['python3', '-c', script], timeout=30))
    changed = [p for p, data in sources.items() if hashes.get(p) != hashlib.sha256(data).hexdigest()]
    if not changed:
        return dict(changed=0, message='机器人磁盘源码已一致；运行中的网关仍须核对恢复版本。')
    backup = remote + '/.deploy-backups/' + str(time.time_ns())
    with console.remote_lock:
        with console.ssh.open_sftp() as sftp:
            def mkdirs(path):
                try:
                    sftp.stat(path)
                except FileNotFoundError:
                    mkdirs(posixpath.dirname(path))
                    sftp.mkdir(path)
            for path in changed:
                target = remote + '/' + path
                temporary = backup + '/new/' + path
                saved = backup + '/old/' + path
                mkdirs(posixpath.dirname(temporary))
                mkdirs(posixpath.dirname(saved))
                mkdirs(posixpath.dirname(target))
                if hashes.get(path) is not None:
                    with sftp.open(target, 'rb') as old, sftp.open(saved, 'wb') as copy:
                        copy.write(old.read())
                with sftp.open(temporary, 'wb') as out:
                    out.write(sources[path])
                sftp.chmod(temporary, 0o644)
                with sftp.open(temporary, 'rb') as check:
                    if hashlib.sha256(check.read()).digest() != hashlib.sha256(sources[path]).digest():
                        raise RuntimeError('上传校验失败：' + path)
                # OpenSSH atomic replace: readers never see a partial Python file.
                sftp.posix_rename(temporary, target)
    verification = json.loads(console.command(['python3', '-c', script], timeout=30))
    if any(verification.get(p) != hashlib.sha256(data).hexdigest() for p, data in sources.items()):
        raise RuntimeError('同步后源码校验不一致；未重启网关，请检查备份：' + backup)
    return dict(changed=len(changed), backup=backup,
                message=f'已同步并校验 {len(changed)} 个运行文件；备份：{backup}。尚未重启网关或执行运动。')
