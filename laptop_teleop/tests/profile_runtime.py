"""Test-only profiling launcher. Production code never imports this module.

Run with the project PYTHONPATH. `preview` uses an isolated mock gateway;
`acquisition` owns the only SDK subscription (stop normal acquisition first).
"""
import argparse
from collections import defaultdict, deque
from contextlib import ExitStack
from functools import wraps
import json
import os
from pathlib import Path
import queue
import signal
import socket
import sys
import tempfile
import threading
import time
from unittest.mock import patch

import numpy as np


def serve_mock(contract, key, address_pipe, stop):
    """Separate process: a test gateway must not compete for the controller GIL."""
    from esrobo_link.gateway import Gateway
    from esrobo_link.backends import MockBackend
    backend=MockBackend('right',True)
    backend.contract=contract
    gateway=Gateway(backend,key,port=0)
    worker=threading.Thread(target=gateway.run,daemon=True);worker.start()
    address_pipe.send(gateway.address[1]);address_pipe.close()
    try: stop.wait()
    finally:
        gateway.request('q');worker.join(1.)


class Profile:
    def __init__(self, path):
        self.path = Path(path)
        self.queue = queue.Queue(maxsize=256)
        self.closed = threading.Event()
        self.dropped = 0
        self.error = None
        self.samples = defaultdict(lambda: deque(maxlen=200))
        self.counts = defaultdict(int)
        self.maximum = defaultdict(float)
        self.last = {}
        self.long_gaps = defaultdict(int)
        self.component_sequence = {}
        self.notifications = {}
        self.stack = ExitStack()
        self.worker = threading.Thread(target=self._write, daemon=True)
        self.worker.start()

    def submit(self, name, duration_ms=0., **facts):
        item = dict(stage=name, monotonic_ns=time.monotonic_ns(), duration_ms=duration_ms, **facts)
        try:
            self.queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    def wrap(self, owner, name, label=None, observe=None, before=None):
        original = getattr(owner, name)
        @wraps(original)
        def measured(*args, **kwargs):
            if before:
                before(args, kwargs)
            start = time.perf_counter()
            cpu_start = time.thread_time_ns()
            try:
                result = original(*args, **kwargs)
            finally:
                elapsed=1000*(time.perf_counter()-start)
                self.submit(label or name, elapsed,
                            thread_cpu_ms=(time.thread_time_ns()-cpu_start)/1.e6)
            if observe:
                observe(args, kwargs, result)
            return result
        self.stack.enter_context(patch.object(owner, name, measured))

    def packet(self, payload, point):
        # Only public component metadata; never persist control protocol packets.
        try:
            p = json.loads(payload) if isinstance(payload, (bytes, str)) else payload
            meta = p.get('laptop_input', {})
            kind = meta.get('kind')
            if kind not in ('pico', 'pico_hand'):
                return
            now = time.monotonic_ns()
            if kind == 'pico':
                self.submit(point+'_body', sequence=p.get('body_sample_sequence'),
                    age_ms=(now/1e9-p['body_sample_monotonic'])*1000)
                if point == 'udp_receive':
                    self.submit('local_udp_delivery', (now-p['sender_monotonic_ns'])/1e6)
                elif point == 'udp_send':
                    notified = self.notifications.get(p.get('body_sample_sequence'))
                    if notified is not None:
                        self.submit('notification_to_send', (now-notified)/1e6)
            else:
                for side, sample in p.get('pico_hands', {}).items():
                    self.submit(point+'_'+side, sequence=sample.get('sequence'),
                        age_ms=(now/1e9-sample['sample_monotonic'])*1000)
        except (TypeError, ValueError, KeyError):
            self.submit('invalid_observed_metadata')

    def tracking(self, args, kwargs, result):
        for side in ('body','left','right'):
            sample=result.get(side)
            if not sample or sample['sequence']<=self.component_sequence.get(side,0):
                continue
            self.component_sequence[side]=sample['sequence']
            now=time.monotonic_ns()
            self.submit('sdk_'+side, sequence=sample['sequence'],
                age_ms=(now-sample['received_monotonic_ns'])/1e6)
            if side=='body':
                self.notifications[sample['sequence']]=now
                if len(self.notifications)>200:
                    self.notifications.pop(next(iter(self.notifications)))

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('x') as stream:
                while not self.closed.is_set() or not self.queue.empty():
                    try:
                        item = self.queue.get(timeout=.05)
                    except queue.Empty:
                        continue
                    key = item['stage']
                    self.counts[key] += 1
                    value = item['duration_ms']
                    self.samples[key].append(value)
                    self.maximum[key] = max(self.maximum[key], value)
                    if key in self.last:
                        gap = (item['monotonic_ns']-self.last[key])/1e6
                        item['gap_ms'] = gap
                        if gap > 100:
                            self.long_gaps[key] += 1
                    self.last[key] = item['monotonic_ns']
                    stream.write(json.dumps(item, allow_nan=False)+'\n')
        except (OSError, ValueError) as exc:
            self.error = str(exc)

    def close(self):
        self.stack.close()  # Restore all functions before closing the collector.
        self.closed.set()
        self.worker.join(1.)
        summary = dict(test_only=True, dropped=self.dropped, error=self.error,
            persistence_complete=not self.worker.is_alive() and self.error is None,
            stages={k:dict(calls=self.counts[k], median_ms=float(np.median(v)),
                p95_ms=float(np.percentile(v,95)), max_ms=self.maximum[k],
                gaps_over_100ms=self.long_gaps[k]) for k,v in self.samples.items() if v})
        self.path.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2))
        return summary

    def install(self, sdk=False):
        from esrobo_laptop.app import Controller
        from esrobo_laptop.pipeline import Pipeline
        from esrobo_laptop.inputs import FreshBodyDevice
        from esrobo_laptop.pico_hand import PicoHands
        from esrobo_laptop.pico_transport import PicoForwarder
        from esrobo_laptop.mapping import WristMapping
        from esrobo_link.client import RobotClient
        from esrobo_teleop.ik.solver import IkSolver
        from esrobo_teleop.robot.nero_driver import NeroArm, NeroSingleArmDriver
        from esrobo_teleop.robot.torso_collision import TorsoCollisionGuard
        for owner,name,label in [(Controller,'step','controller'),
                (Controller,'_send_targets','target_send'),(Pipeline,'capture','input_capture'),
                (Pipeline,'solve','all_ik'),(Pipeline,'map_hand_target','finger_mapping'),
                (Pipeline,'publish_diagnostics','skeleton_publish'),
                (PicoHands,'map_sample','raw_hand_mapping'),(IkSolver,'solve','arm_ik'),
                (WristMapping,'absolute_terminal','wrist_ik'),
                (RobotClient,'receive','robot_state_receive'),
                (NeroArm,'get_joint_angles','startup_feedback_wait'),
                (NeroArm,'runtime_feedback','feedback_snapshot'),
                (NeroSingleArmDriver,'_send_return_position','entry_send'),
                (TorsoCollisionGuard,'_check','geometry')]:
            self.wrap(owner,name,label)
        self.wrap(FreshBodyDevice,'_decode_input','input_parse',
            before=lambda a,k:self.packet(a[1],'udp_receive'))
        self.wrap(FreshBodyDevice,'_commit_input','input_commit')
        self.wrap(WristMapping,'terminal','wrist_terminal', before=lambda a,k:self.submit(
            'wrist_case', axes=a[0].axes.tolist(), desired=np.asarray(a[1]).tolist(),
            neutral=a[0].neutral_urdf.tolist(), lower=a[0].terminal_lower.tolist(),
            upper=a[0].terminal_upper.tolist(),
            previous=None if a[0]._last_terminal_target is None else a[0]._last_terminal_target.tolist()))
        self.wrap(PicoForwarder,'_send','udp_send',
            lambda a,k,r:self.packet(a[2],'udp_send') if r else None)
        if sdk:
            import xrobotoolkit_sdk
            self.wrap(xrobotoolkit_sdk,'wait_tracking_updates','sdk_wait',self.tracking)
            self.wrap(PicoForwarder,'__init__','forwarder_init',
                lambda a,k,r:self.wrap(a[2],'_packet_from_sample','body_build'))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode',choices=['preview','acquisition'])
    p.add_argument('--seconds',type=int,default=60)
    p.add_argument('--output',required=True)
    p.add_argument('--mapping-baseline', help='Test-only archived mapping.py for paired comparisons')
    p.add_argument('--synthetic', action='store_true', help='Isolated synthetic source; preview only')
    p.add_argument('--switch-interval', type=float, help='Test-only Python thread scheduling comparison')
    p.add_argument('--compute-seconds', type=float,
                   help='Stop this many seconds after the first actual solve; --seconds is still the total cap')
    p.add_argument('--observe-only', action='store_true',
                   help='Preview without per-function timing wrappers; use normal stream-rate records')
    args=p.parse_args()
    previous_switch_interval=sys.getswitchinterval()
    if args.switch_interval is not None: sys.setswitchinterval(args.switch_interval)
    prof=Profile(args.output)
    compute_timer=None
    try:
        if args.synthetic:
            if args.mode != 'preview':
                raise ValueError('synthetic source is only available for isolated preview')
            from esrobo_laptop.pipeline import Pipeline
            from esrobo_laptop.demo import SyntheticSensors
            original_init,original_close=Pipeline.__init__,Pipeline.close
            def initialize(pipeline,cfg,settings,contracts):
                # Bind a kernel-assigned port, never the real acquisition port.
                settings.input_port=0
                cfg.retarget.auto_start_reference_prepare_s=0
                cfg.retarget.auto_start_reference_delay_s=.12
                cfg.retarget.auto_start_reference_sample_start_s=0
                cfg.retarget.auto_start_reference_min_samples=5
                original_init(pipeline,cfg,settings,contracts)
                producer=SyntheticSensors(pipeline)
                solver=next(iter(pipeline.solvers.values()))
                q=np.zeros(14);q[7:]=[-.1,-.4,1.,.8,-1.3,.05,-.5]
                poses=solver.current_task_frame_poses(q)
                rotation=np.asarray(cfg.retarget.source_to_robot_rotation).reshape(3,3)
                from esrobo_teleop import math_utils as mu
                for name,pose in poses.items():
                    producer.frames[name]=dict(pos=(rotation.T@pose[:3]).tolist(),quat_wxyz=[1,0,0,0])
                producer.palm_bases['right']=(rotation.T@mu.quat_wxyz_to_matrix(poses['right_wrist'][3:])).tolist()
                pipeline._test_producer=producer
                producer.thread.start()
            def close(pipeline):
                if hasattr(pipeline,'_test_producer'): pipeline._test_producer.close()
                original_close(pipeline)
            prof.stack.enter_context(patch.object(Pipeline,'__init__',initialize))
            prof.stack.enter_context(patch.object(Pipeline,'close',close))
        if args.mapping_baseline:
            import importlib.util
            from esrobo_laptop.mapping import WristMapping
            spec=importlib.util.spec_from_file_location('esrobo_laptop.mapping_baseline',args.mapping_baseline)
            module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
            prof.stack.enter_context(patch.object(WristMapping,'terminal',module.WristMapping.terminal))
        if args.observe_only and args.mode != 'preview':
            raise ValueError('observe-only requires preview')
        if not args.observe_only: prof.install(sdk=args.mode=='acquisition')
        if args.compute_seconds is not None:
            if args.mode != 'preview' or args.compute_seconds <= 0:
                raise ValueError('positive compute-seconds requires preview')
            from esrobo_laptop.pipeline import Pipeline
            def compute_started(a,k,result):
                nonlocal compute_timer
                if compute_timer is None:
                    prof.submit('compute_window_start')
                    compute_timer=threading.Timer(args.compute_seconds,
                                                 lambda:os.kill(os.getpid(),signal.SIGTERM))
                    compute_timer.start()
            if args.observe_only:
                original_solve=Pipeline.solve
                def untimed_solve(*a,**k):
                    result=original_solve(*a,**k)
                    if compute_timer is None:compute_started(a,k,result)
                    return result
                prof.stack.enter_context(patch.object(Pipeline,'solve',untimed_solve))
            else:
                prof.wrap(Pipeline,'solve','solve_window',observe=compute_started)
        if args.mode=='preview':
            from esrobo_link.gateway import Gateway
            from esrobo_link.backends import MockBackend
            from esrobo_laptop.app import main as run
            # Protocol key files strip whitespace; ASCII hex avoids random
            # leading/trailing bytes shortening a test key below 32 bytes.
            key=os.urandom(32).hex().encode('ascii')
            # Match the commissioned envelope without constructing or connecting
            # any physical backend. The generic mock's +/-1 rad wrist/arm limits
            # otherwise create artificial unreachable poses and retry load.
            from esrobo_laptop.config import ROOT, read_settings, read_robot_config
            from esrobo_teleop.robot.nero_driver import NeroSingleArmDriver
            cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
            limits=NeroSingleArmDriver.__new__(NeroSingleArmDriver)
            limits._cfg=cfg.robot; limits._session_start=None
            lo,hi=limits._effective_position_limits('right')
            direction,offset=limits._mapping('right')
            a,b=(lo-offset)/direction,(hi-offset)/direction
            backend=MockBackend('right',True)
            backend.contract.update(lower_rad=np.minimum(a,b).tolist(),upper_rad=np.maximum(a,b).tolist())
            import multiprocessing
            context=multiprocessing.get_context('spawn')
            parent_pipe,child_pipe=context.Pipe(duplex=False)
            stop=context.Event()
            worker=context.Process(target=serve_mock,args=(backend.contract,key,child_pipe,stop))
            worker.start();child_pipe.close()
            try:
                if not parent_pipe.poll(10):raise RuntimeError('isolated mock gateway failed to start')
                port=parent_pipe.recv();parent_pipe.close()
                with tempfile.TemporaryDirectory() as temp:
                    keypath=Path(temp)/'key';keypath.write_bytes(key);keypath.chmod(0o600)
                    code=run(['run','--mock','--preview','--side','right','--host','127.0.0.1',
                        '--port',str(port),'--key-file',str(keypath),
                        '--seconds',str(args.seconds),'--log-file',str(Path(args.output).with_suffix('.session.jsonl'))])
            finally:
                stop.set();worker.join(2.)
                if worker.is_alive():worker.terminate();worker.join(1.)
                parent_pipe.close()
        else:
            from esrobo_laptop.acquisition import pico
            receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
            receiver.bind(('127.0.0.1',15050));receiver.settimeout(.05)
            receiving=threading.Event()
            def receive():
                while not receiving.is_set():
                    try:payload,_=receiver.recvfrom(65535)
                    except socket.timeout:continue
                    prof.packet(payload,'udp_receive')
            rx=threading.Thread(target=receive,daemon=True);rx.start()
            timer=threading.Timer(args.seconds,lambda:os.kill(os.getpid(),signal.SIGTERM))
            timer.start()
            try:code=pico([])
            finally:
                timer.cancel();receiving.set();rx.join(.1);receiver.close()
        return code or 0
    finally:
        if compute_timer: compute_timer.cancel()
        summary=prof.close()
        sys.setswitchinterval(previous_switch_interval)
        print(json.dumps(summary,ensure_ascii=False))


if __name__=='__main__':
    raise SystemExit(main())
