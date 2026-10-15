"""Spawn-only geometry calculation; the parent exclusively owns hardware I/O."""
import dataclasses
import hashlib
import json
import multiprocessing
from pathlib import Path
import time
import uuid


def request_digest(request):
    identity = {k:v for k,v in request.items() if k != 'meshes'}
    return hashlib.sha256(json.dumps(identity, sort_keys=True, allow_nan=False).encode()).hexdigest()


def calculate(request):
    # Import only geometry/model code. No SDK, ROS or hardware constructors.
    from types import SimpleNamespace
    from .backends import ForwardModel
    from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
    from esrobo_teleop.robot.hand_geometry import HandGeometryState
    if 'urdf_digest' in request:
        if hashlib.sha256(Path(request['ik']['urdf_path']).read_bytes()).hexdigest() != request['urdf_digest']:
            raise ValueError('collision URDF digest mismatch')
    if 'meshes' in request:
        actual = hashlib.sha256(json.dumps(request['meshes'], sort_keys=True, allow_nan=False).encode()).hexdigest()
        if actual != request['meshes_digest']:
            raise ValueError('collision mesh snapshot digest mismatch')
    model = ForwardModel(SimpleNamespace(**request['ik']))
    guard = TorsoCollisionGuard(model, request['side'], request['package_dirs'],
                               request['margin'], request['shoulder_margin'],
                               include_fingers=not request['near_zero'], mesh_snapshot=request.get('meshes'))
    if request['near_zero']:
        ok = guard.check_arm_only(request['lower'], request['upper'])
    else:
        state = HandGeometryState(**request['hand'])
        ok = guard.check_hand_return(request['lower'], request['upper'], state)
    return dict(certified=bool(ok), diagnostics=guard.last_diagnostics)


def _worker(connection, request, digest):
    try:
        result = calculate(request)
        connection.send(dict(request_id=request['request_id'], digest=digest, **result))
    except BaseException as exc:
        connection.send(dict(request_id=request['request_id'], digest=digest,
                             error=f'{type(exc).__name__}: {exc}'))
    finally:
        connection.close()


def certify(cfg, side, lower, upper, hand, near_zero, monitor, guard):
    request = dict(request_id=uuid.uuid4().hex, ik=dataclasses.asdict(cfg.ik), side=side,
                   meshes=guard.proof_mesh_snapshot, meshes_digest=guard.proof_mesh_digest,
                   urdf_digest=guard.proof_urdf_digest,
                   package_dirs=list(cfg.robot.collision_package_dirs),
                   margin=cfg.robot.torso_collision_margin_m,
                   shoulder_margin=cfg.robot.shoulder_collision_margin_m,
                   lower=lower.tolist(), upper=upper.tolist(), near_zero=bool(near_zero),
                   hand=None if hand is None else dataclasses.asdict(hand))
    return run_request(request, cfg.robot.return_plan_timeout_s, monitor)


def run_request(request, timeout, monitor):
    """Poll hardware every 20 ms, including initialization; reject late replies."""
    context = multiprocessing.get_context('spawn')
    reader, writer = context.Pipe(duplex=False)
    digest = request_digest(request)
    process = context.Process(target=_worker, args=(writer, request, digest), daemon=True)
    deadline = time.monotonic() + timeout
    started = False
    try:
        if not monitor():
            raise RuntimeError('hand return monitoring rejected before geometry')
        process.start()
        started = True
        writer.close()
        while True:
            if not monitor():
                raise RuntimeError('hand return monitoring rejected during geometry')
            if time.monotonic() >= deadline:
                raise RuntimeError('hand return geometry timeout')
            if reader.poll(min(.02, max(0., deadline-time.monotonic()))):
                try:
                    result = reader.recv()
                except EOFError as exc:
                    raise RuntimeError('hand return geometry worker exited without result') from exc
                if not monitor():
                    raise RuntimeError('hand return monitoring rejected after geometry')
                if time.monotonic() >= deadline:
                    raise RuntimeError('hand return geometry result expired')
                if (result.get('request_id') != request['request_id']
                        or result.get('digest') != digest):
                    raise RuntimeError('hand return geometry result does not match request')
                if result.get('error'):
                    raise RuntimeError('hand return geometry worker failed: '+result['error'])
                if type(result.get('certified')) is not bool or not isinstance(result.get('diagnostics'), dict):
                    raise RuntimeError('invalid hand return geometry result')
                return result
            if not process.is_alive():
                raise RuntimeError('hand return geometry worker exited without result')
    finally:
        reader.close()
        writer.close()
        if started:
            if process.is_alive():
                process.terminate()
            process.join(timeout=.2)
            if process.is_alive():
                process.kill()
                process.join(timeout=.2)
            process.close()
