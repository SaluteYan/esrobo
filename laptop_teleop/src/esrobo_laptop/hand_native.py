"""Explicit build and hash-checked loading of the pure numerical hand kernel.

Compilation is never performed in the collection/control loop. Only the Python
and pybind11 build headers plus a C++17 compiler are required; the extension has
no robotics/optimizer runtime dependency and never opens hardware or sockets.
"""
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import sysconfig

SOURCE = Path(__file__).with_name('_hand_native.cpp')
FLAGS = ('-O3', '-DNDEBUG', '-std=c++17', '-shared', '-fPIC')
_MODULE = None


def artifact():
    abi = sysconfig.get_config_var('SOABI') or sys.implementation.cache_tag
    digest = hashlib.sha256(SOURCE.read_bytes()+repr(FLAGS).encode()+abi.encode()).hexdigest()
    cache = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home()/'.cache')))
    return cache/'esrobo_teleop'/'hand_native'/digest/('_hand_native_ext'+sysconfig.get_config_var('EXT_SUFFIX'))


def build():
    destination = artifact()
    if destination.is_file():
        return destination
    import pybind11  # Build-time dependency; not imported by the runtime loader.
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name+f'.{os.getpid()}.tmp')
    command = [os.environ.get('CXX', 'c++'), *FLAGS, '-I'+pybind11.get_include(),
               '-I'+sysconfig.get_paths()['include'], str(SOURCE), '-o', str(temporary)]
    try:
        subprocess.run(command, check=True, timeout=120.)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    return destination


def load_model(model, rest, seeds, policy):
    global _MODULE
    path = artifact()
    if not path.is_file():
        return None
    if _MODULE is None:
        spec = importlib.util.spec_from_file_location('_hand_native_ext', path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        _MODULE = module
    data = dict(offsets=[j[1] for j in model.joints], rotations=model._rot,
                axes=[j[3] for j in model.joints], sources=model._indices,
                ratios=model._ratios, biases=model._biases, tips=model.tip_offsets,
                lower=model.lower, upper=model.upper, neutral=model.neutral,
                width=model.width, rest=rest, seeds=seeds)
    data['finger_ids'] = [[-1 if finger=='middle' else model.joint_index[finger+'_mcp_roll']]+
                          [model.joint_index[finger+name] for name in ('_mcp_pitch','_pip','_dip')]
                          for finger in ('index','middle','ring','pinky')]
    return _MODULE.Model(data, policy)


if __name__ == '__main__':
    print(build())
