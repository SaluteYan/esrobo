"""Paired offline trace benchmark. Spawned fake SDK/gateway; no real ports/devices."""
import argparse
import json
import multiprocessing as mp
import os
from pathlib import Path
import resource
import tempfile
import threading
import time

import numpy as np


def measure_trace(trace):
    samples=[]
    original=trace.emit
    def emit(*args, **kwargs):
        started=time.perf_counter_ns()
        try:return original(*args, **kwargs)
        finally:samples.append((time.perf_counter_ns()-started)/1e6)
    trace.emit=emit
    return samples


def gateway_worker(contract,key,pipe,stop,enabled,folder):
    from esrobo_link.backends import MockBackend
    from esrobo_link.gateway import Gateway
    from esrobo_teleop.debug.motion_trace import MotionTrace
    from types import SimpleNamespace
    from esrobo_link.motion_trace import attach
    from esrobo_teleop.robot.nero_driver import JointFeedbackSnapshot
    b=MockBackend('right',True);b.contract=contract;b.side='right'
    # Exercise the actual passive cache/probe adapter with representative
    # seven-axis payloads; these are synthetic feedback, never CAN reads.
    b.arm=SimpleNamespace(_cycle_feedback=None)
    original_step=b.step
    def step(target,cancelled):
        before=np.asarray(b.q);original_step(target,cancelled);now=time.monotonic()
        q=np.asarray(b.q);velocity=(q-before)/.02
        snap=JointFeedbackSnapshot(q,now,now,'benchmark-synthetic')
        b.arm._cycle_feedback=snap
        b.arm.motion_limit_diagnostics=dict(command_rad=q.tolist(),velocity_rad_s=velocity.tolist(),
            command_feedback_error_rad=[0.]*7,target_command_error_rad=(np.asarray(target.get('arm_urdf_rad',q))-q).tolist())
        if hasattr(b.arm,'probe_sink'):
            b.arm.probe_sink(dict(event='command_sent',ik_urdf_rad=target.get('arm_urdf_rad'),
                trajectory_urdf_rad=q.tolist(),command_urdf_rad=q.tolist(),velocity_urdf_rad_s=velocity.tolist(),
                feedback_urdf_rad=q.tolist(),feedback_snapshot=snap.diagnostics(),position_frame_sent=True,
                configured_speed_bound_active=[False]*7,acceleration_bound_active=[False]*7,
                directional_lead_braking_active=[False]*7,feedback_motion_braking_active=[False]*7,
                sent_monotonic_s=now))
    b.step=step
    g=Gateway(b,key,port=0);g.motion_trace=MotionTrace(Path(folder)/'execution.jsonl',enabled=enabled)
    emissions=measure_trace(g.motion_trace);cpu=time.process_time()
    if enabled:attach(b,g.motion_trace)
    thread=threading.Thread(target=g.run);thread.start();pipe.send(g.address[1])
    while not stop.wait(.01):
        if pipe.poll():
            action=pipe.recv()
            if action=='enable':g.request('e')
    g.request('q');thread.join(2);g.motion_trace.close();pipe.send(dict(trace=g.motion_trace.stats(),producer_trace_ms=percentiles(emissions),cpu_s=time.process_time()-cpu,rss_kb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss))


def pico_worker(address,frames,palms,pipe,stop,enabled,folder):
    from test_pico_transport import FakeSDK
    from test_pico_hand import skeleton
    from esrobo_laptop.pico_hand import PicoHands,palm_basis
    from esrobo_laptop.acquisition import load_bridge
    from esrobo_laptop.pico_transport import PicoForwarder
    from esrobo_teleop.debug.motion_trace import MotionTrace
    bridge=load_bridge('xrobotoolkit_body_udp_bridge')
    args=bridge.parse_args(['--host',address[0],'--port',str(address[1]),'--samples','0','--rate-hz','90'])
    base=skeleton(.15);basis=palm_basis(base)
    class SDK(FakeSDK):
        def emit(self,seq,**kw):
            with self.condition:
                now=time.monotonic_ns();body=self.sample(seq)
                body['received_monotonic_ns']=now
                body['poses']=[p.copy() for p in body['poses']]
                for i,name in bridge.JOINT_TO_FRAME.items():
                    if name in frames:body['poses'][i]=frames[name]['pos']+[0.,0.,0.,1.]
                hands={}
                for side in ('left','right'):
                    pose=base.copy();pose[:,:3]=(np.asarray(palms[side])@basis.T@pose[:,:3].T).T
                    h=self.hand(seq);h.update(poses=pose.tolist(),received_monotonic_ns=now);hands[side]=h
                self.latest=dict(notification_id=seq,body=body,**hands);self.receipts.append(now);self.condition.notify_all()
        def start(self):
            def produce():
                seq=0;start=due=time.monotonic()
                while not self.end.is_set():
                    # Deterministic 70 ms source gaps every two seconds after warmup.
                    elapsed=time.monotonic()-start
                    if elapsed>5 and elapsed%2<.07:
                        self.end.wait(.07-elapsed%2);due=time.monotonic()
                    seq+=1;self.emit(seq);due+=1/90;self.end.wait(max(0.,due-time.monotonic()))
            self.thread=threading.Thread(target=produce);self.thread.start()
    sdk=SDK(90);trace=MotionTrace(Path(folder)/'pico.jsonl',enabled=enabled)
    emissions=measure_trace(trace)
    f=PicoForwarder(sdk,bridge,args,trace=trace);latencies=[]
    original=f._send
    def send(sock,data,component):
        result=original(sock,data,component)
        if component=='body':
            packet=json.loads(data);latencies.append((time.monotonic()-packet['body_sample_monotonic'])*1000)
        return result
    f._send=send
    thread=threading.Thread(target=f.run);cpu=time.process_time();sdk.start();thread.start();pipe.send('ready')
    stop.wait();f.stop.set();thread.join(2);sdk.close();trace.close()
    pipe.send(dict(cpu_s=time.process_time()-cpu,rss_kb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                   forwarding_ms=percentiles(latencies),producer_trace_ms=percentiles(emissions),trace=trace.stats()))


def percentiles(values):
    return dict(zip(('p50','p95','p99','max'),np.percentile(values,[50,95,99,100]).tolist())) if len(values) else None


def run(enabled,seconds,folder):
    from esrobo_laptop.config import ROOT,read_settings,read_robot_config
    from esrobo_laptop.pipeline import Pipeline
    from esrobo_laptop.demo import SyntheticSensors
    from esrobo_laptop.app import Controller
    from esrobo_laptop.inputs import InputUnavailable
    from esrobo_link.backends import MockBackend
    from esrobo_link.client import RobotClient
    from esrobo_teleop.debug.motion_trace import MotionTrace
    from esrobo_teleop import math_utils as mu
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    settings=read_settings(ROOT/'config/laptop.yaml');settings.side='right';settings.with_hand=True
    settings.robot_host='127.0.0.1';settings.input_port=0
    cfg=read_robot_config(settings);contract=MockBackend('right',True).contract
    contract['lower_rad']=list(cfg.robot.right_joint_soft_lower_limits_urdf)
    contract['upper_rad']=list(cfg.robot.right_joint_soft_upper_limits_urdf)
    p=Pipeline(cfg,settings,{'right':contract});q=np.zeros(14);q[7:]=[-.1,-.4,1.,.8,-1.3,.05,-.5]
    sensor=SyntheticSensors(p);poses=p.solvers['right'].current_task_frame_poses(q)
    rot=np.asarray(cfg.retarget.source_to_robot_rotation).reshape(3,3)
    for name,pose in poses.items():sensor.frames[name]=dict(pos=(rot.T@pose[:3]).tolist(),quat_wxyz=[1,0,0,0])
    sensor.palm_bases['right']=(rot.T@mu.quat_wxyz_to_matrix(poses['right_wrist'][3:])).tolist()
    ctx=mp.get_context('spawn');stop=ctx.Event();gp,gc=ctx.Pipe();pp,pc=ctx.Pipe();key=b'offline-trace-only-key-12345678901234567890'
    gateway=ctx.Process(target=gateway_worker,args=(contract,key,gc,stop,enabled,str(folder)));gateway.start();assert gp.poll(10);port=gp.recv()
    pico=ctx.Process(target=pico_worker,args=(p.body._sock.getsockname(),sensor.frames,sensor.palm_bases,pc,stop,enabled,str(folder)));pico.start();assert pp.poll(10);pp.recv()
    keypath=folder/'key';keypath.write_bytes(key);client=RobotClient('127.0.0.1',port,keypath);client.connect();keypath.unlink()
    trace=MotionTrace(folder/'compute.jsonl',enabled=enabled);p.motion_trace=client.motion_trace=trace;p.body.motion_trace_enabled=enabled;p.body.motion_trace=trace
    client.trace_context=lambda:p.__dict__.get('trace_context')
    emission=[];original_emit=trace.emit
    def emit(*a,**kw):
        start=time.perf_counter_ns();r=original_emit(*a,**kw);emission.append((time.perf_counter_ns()-start)/1e6);return r
    trace.emit=emit
    all_commands=[]
    original_command=client._command
    def command(kind,**payload):
        result=original_command(kind,**payload)
        all_commands.append((time.monotonic(),kind))
        return result
    client._command=command
    c=Controller(client,p,settings,contract);c.pipeline.restart_preparation()
    cpu=time.process_time();start=time.monotonic();running_start=None;due=start;cycles=[];compute=[];send_times=[];holds=0;errors=[];enabled_sent=False
    original_solve=p.solve
    def solve(*a,**kw):
        tick=time.perf_counter_ns()
        try:return original_solve(*a,**kw)
        finally:compute.append((time.perf_counter_ns()-tick)/1e6)
    p.solve=solve
    try:
        while time.monotonic()-start<seconds+15 and (running_start is None or time.monotonic()-running_start<seconds):
            tick=time.perf_counter_ns();state=client.receive(.2);before=c.holds_sent
            try:
                targets=c.step(state)
                if targets is None:c.hold_missing_input(state,'benchmark source gap')
                if targets is not None:
                    send_times.append(time.monotonic())
                    if not enabled_sent:gp.send('enable');enabled_sent=True
                if state['mode']=='ACTIVE' and running_start is None:running_start=time.monotonic()
            except InputUnavailable as exc:
                if running_start is not None:
                    c.hold_missing_input(state,str(exc))
            if state['mode']=='FAULT':raise RuntimeError(state['reason'])
            holds+=c.holds_sent-before
            trace.emit('cycle',started_ns=tick,mode=state['mode'],context=p.__dict__.get('trace_context'))
            if running_start is not None:cycles.append((time.perf_counter_ns()-tick)/1e6)
            due+=.02;time.sleep(max(0.,due-time.monotonic()))
        if running_start is None:raise RuntimeError('No actual ACTIVE computation: '+p.body.last_rejection)
    finally:
        client.close();p.close();sensor.socket.close();stop.set();gateway.join(3);pico.join(3);trace.close()
        for worker in (gateway,pico):
            if worker.is_alive():worker.terminate();worker.join()
    active_commands=[t for t,kind in all_commands if t>=running_start and kind in ('target','hold')]
    result=dict(enabled=enabled,command_gap_ms=percentiles(np.diff(active_commands)*1000),
        command_gaps_over_80ms=int(np.sum(np.diff(active_commands)>.08)),
        active_duration_s=time.monotonic()-running_start,compute_calls=len(compute),
        cycle_ms=percentiles(cycles),compute_ms=percentiles(compute),producer_trace_ms=percentiles(emission),
        target_gap_ms=percentiles(np.diff(send_times)*1000),target_gaps_over_80ms=int(np.sum(np.diff(send_times)>.08)),
        hold_packets=holds,cpu_s=time.process_time()-cpu,rss_kb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        trace=trace.stats(),pico=pp.recv() if pp.poll(2) else None,gateway_trace=gp.recv() if gp.poll(2) else None)
    (folder/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--enabled',type=int,default=1);parser.add_argument('--seconds',type=float,default=60);parser.add_argument('--output',required=True);args=parser.parse_args()
    run(bool(args.enabled),args.seconds,args.output)
