#!/usr/bin/env python3
"""Fetch only this URDF's collision meshes from a pinned, matching GitHub model.
Verifies Git LFS SHA256/size (or Git blob SHA1); never overwrites different files.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET

REPO = 'SaluteYan/Dual-arm-teleoperation'
REVISION = 'df1b41b9e7557064478f42012f644343e5584a7a'
PREFIX = 'urdf/esrobo_waist_with_head/'
ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'assets' / 'esrobo_waist_with_head'


def read(url):
    return urllib.request.urlopen(urllib.request.Request(url, headers={
        'User-Agent': 'esrobo-collision-assets'}), timeout=60).read()


def main():
    local = ET.parse(ROOT / 'urdf/esrobo_waist_with_head.urdf').getroot()
    remote = ET.fromstring(read(f'https://raw.githubusercontent.com/{REPO}/{REVISION}/{PREFIX}urdf/esrobo_waist_with_head.urdf'))
    for link in local.findall('link'):
        collision = link.find('collision')
        other = remote.find("link[@name='%s']/collision" % link.attrib['name'])
        if collision is not None and (other is None or ET.tostring(collision) != ET.tostring(other)):
            raise RuntimeError(f"collision geometry differs: {link.attrib['name']}")
    tree = json.loads(read(f'https://api.github.com/repos/{REPO}/git/trees/{REVISION}?recursive=1'))
    blobs = {p['path']: p['sha'] for p in tree['tree'] if p['type'] == 'blob'}
    paths = sorted({m.attrib['filename'].removeprefix('package://esrobo_waist_with_head/')
                    for m in local.findall('.//collision/geometry/mesh')})
    def fetch(relative):
        path = PREFIX + relative
        data = read(f'https://raw.githubusercontent.com/{REPO}/{REVISION}/{path}')
        if hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest() != blobs[path]:
            raise RuntimeError(f'Git blob checksum failed: {path}')
        if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
            meta = dict(line.split(' ', 1) for line in data.decode().splitlines())
            expected = meta['oid'].removeprefix('sha256:')
            target = DEST / relative
            if target.exists():
                data = target.read_bytes()
            else:
                data = read(f'https://media.githubusercontent.com/media/{REPO}/{REVISION}/{path}')
            if len(data) != int(meta['size']) or hashlib.sha256(data).hexdigest() != expected:
                raise RuntimeError(f'LFS checksum failed: {path}')
        target = DEST / relative
        if target.exists() and target.read_bytes() != data:
            raise RuntimeError(f'refusing to replace different mesh: {target}')
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        return {'path': relative, 'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        records = list(pool.map(fetch, paths))
    (DEST / 'SOURCE.json').write_text(json.dumps(dict(repository=REPO, revision=REVISION,
        matching_collision_geometry=True, files=records), indent=2) + '\n')
    print(f'Verified {len(records)} collision meshes ({sum(r["size"] for r in records)/1e6:.1f} MB) at {DEST}')


if __name__ == '__main__':
    main()
