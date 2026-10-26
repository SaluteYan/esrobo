"""Isolated process/loopback benchmark. No SDK, robot gateway, or hardware.

Replays recorded SDK arrivals (Body/Hand) at their original spacing; only the
benchmark clock epoch is shifted. A separate producer owns the latest slot.
Body payloads are serialized; both palms and legacy right fingers use the real
calculation path at arrival, representing the same capture work in both variants; this is not a full arm IK or hardware timing benchmark.
"""
import argparse
import collections
import copy
import hashlib
import inspect
import json
import multiprocessing as mp
from pathlib import Path
import socket
import threading
import time

import numpy as np
from esrobo_laptop.config import ROOT, read_settings, read_robot_config
from esrobo_laptop.pico_hand import PicoHands, palm_sample
from replay_hand_contact import baseline_module, stats


def worker(pipe, mode, baseline, cfg, warmup):
    from esrobo_laptop.hand_retarget import HandRetargeter
    from esrobo_laptop.pico_hand import PicoHands
    solver = (HandRetargeter if mode=='new' else baseline_module(baseline).HandRetargeter)(cfg)
    for sample in warmup:
        try:
            kw = dict(source_time=sample['received_monotonic_ns']/1e9) if mode=='new' else {}
            solver.solve(sample['poses'], budget_s=1., **kw)
        except (ValueError, TimeoutError): pass
    stamp = None
    mapper = PicoHands(lambda side, poses: solver.solve(poses, **(
        dict(source_time=stamp, commit=False) if mode=='new' else {})))
    pipe.send('ready'); offset = pipe.recv()
    if mode=='new' and solver.source_time is not None: solver.source_time += offset
    pending = None
    while True:
        message = pipe.recv()
        if message is None: break
        if message[0] == 'ack':
            if mode=='new' and pending is not None and message[1] == pending[0]:
                if message[2]: solver.commit(pending[1])
                else: solver.discard()
                pending = None
            continue
        _, seq, sample = message; stamp = sample['received_monotonic_ns']/1e9
        if mode=='new': solver.discard(); pending = None
        started = time.monotonic()
        try:
            result = mapper.map_sample('left', sample)
            if time.monotonic()-started > .020: raise TimeoutError('worker exceeded 20 ms')
            if mode=='new' and result is not None: pending = (seq, result.pop('candidate_id'))
            pipe.send((seq, result, None))
        except Exception as exc:
            if mode=='new': solver.discard()
            pipe.send((seq, None, str(exc)))
    pipe.close()


def run(mode, baseline, cfg, warmup, arrivals, start, duration):
    parent, child = mp.get_context('spawn').Pipe()
    process = mp.get_context('spawn').Process(target=worker,args=(child,mode,baseline,cfg,warmup))
    process.start(); child.close()
    if not parent.poll(120): raise RuntimeError('offline worker initialization timed out')
    assert parent.recv() == 'ready'
    epoch = time.monotonic()+.1; offset=epoch-start; parent.send(offset)
    lock=threading.Lock(); state={'latest':None}; stop=threading.Event()
    right_mapper=PicoHands(); capture_counts=collections.Counter()
    def produce():
        for r in arrivals:
            if stop.wait(max(0.,r['received_monotonic_ns']/1e9+offset-time.monotonic())): return
            if r['component']=='body':
                json.dumps(r['poses'], separators=(',',':')); capture_counts['body']+=1
                continue
            sample=dict(sequence=r['source_sequence'],received_monotonic_ns=int(r['received_monotonic_ns']+offset*1e9),
                active=r.get('active',1),valid=r.get('valid',True),poses=r['poses'],flags=r['flags'],
                timestamp_ns=r.get('device_timestamp_ns',0),packet_timestamp_ns=r.get('packet_timestamp_ns',0),
                freshness_source='component-packet',age_s=0.)
            side=r['component']
            try:
                palm_sample(sample)
                capture_counts[side+'_palm']+=1
                if side=='right': right_mapper.map_sample(side,sample)
            except (ValueError,KeyError,TypeError): capture_counts[side+'_invalid']+=1
            if side=='left':
                with lock: state['latest']=sample
    producer=threading.Thread(target=produce); producer.start()
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); receiver=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    receiver.bind(('127.0.0.1',0)); receiver.settimeout(.1); destination=receiver.getsockname()
    last=0; due=epoch; last_report=epoch; solved=[]; latency=[]; roundtrip=[]; sendtimes=[]; failures=collections.Counter(); late=0; pending=None
    try:
        while time.monotonic()<epoch+duration:
            if time.monotonic()-last_report>=30.:
                print('progress',mode,round(time.monotonic()-epoch,1),'s accepted',len(solved),
                      'failures',sum(failures.values()),'late',late,flush=True)
                last_report=time.monotonic()
            stop.wait(max(0.,due-time.monotonic())); due=time.monotonic()+.020
            if pending is not None:
                if not parent.poll(): continue
                oldseq,_,_=parent.recv();parent.send(('ack',oldseq,False));pending=None
            with lock: sample=state['latest']
            if sample is None or sample['sequence']<=last: continue
            last=sample['sequence']; begin=time.monotonic(); pending=last
            parent.send(('sample',last,sample))
            if not parent.poll(.030): failures['30 ms reply timeout']+=1;continue
            seq,result,error=parent.recv();pending=None
            roundtrip.append((time.monotonic()-begin)*1000)
            if error: failures[error]+=1;parent.send(('ack',seq,False));continue
            age=time.monotonic()-sample['received_monotonic_ns']/1e9
            if seq!=last or age>.1 or time.monotonic()-begin>.030:
                late+=1;parent.send(('ack',seq,False));continue
            if result is None: continue
            # The isolated receiver parses the same ten-axis output, without
            # sending any command to the production input or gateway ports.
            encoded=json.dumps(dict(sequence=seq,joints=result['joints'],source=sample['received_monotonic_ns'])).encode()
            sender.sendto(encoded,destination);received=json.loads(receiver.recv(8192))
            assert received['sequence']==seq and len(received['joints'])==10
            now=time.monotonic();parent.send(('ack',seq,True));solved.append(result['diagnostics']['solve_ms']);latency.append((now-sample['received_monotonic_ns']/1e9)*1000);sendtimes.append(now)
    finally:
        stop.set();producer.join();parent.send(None);process.join(3)
        if process.is_alive():process.terminate();process.join()
        parent.close();sender.close();receiver.close()
    gaps=np.diff(sendtimes)*1000
    return dict(mode=mode,duration_s=duration,capture_counts=dict(capture_counts),accepted=len(solved),failures=dict(failures),late=late,
        solve_ms=stats(solved),source_to_loopback_ms=stats(latency),worker_roundtrip_ms=stats(roundtrip),
        send_interval_ms=stats(gaps),gaps_over_40ms=int(np.sum(gaps>40)),gaps_over_100ms=int(np.sum(gaps>100)))


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--pico',required=True);ap.add_argument('--raw',required=True);ap.add_argument('--session',required=True);ap.add_argument('--baseline',required=True);ap.add_argument('--output',required=True);ap.add_argument('--repeats',type=int,default=3);ap.add_argument('--duration',type=float)
    args=ap.parse_args();sessions=[json.loads(l) for l in Path(args.session).open()];active=[r for r in sessions if r.get('mode')=='ACTIVE'];start=active[0]['monotonic_s'];duration=args.duration or active[-1]['monotonic_s']-start
    warm=[]
    for line in Path(args.raw).open():
        r=json.loads(line)
        if r.get('side')!='left':continue
        if r['sample']['received_monotonic_ns']/1e9>=start:break
        warm.append(r['sample'])
    arrivals=[]
    for line in Path(args.pico).open():
        r=json.loads(line)
        if r.get('event')!='raw_sample' or r.get('component') not in ('left','right','body'):continue
        t=(r.get('received_monotonic_ns') or 0)/1e9
        if start<=t<=start+duration:arrivals.append(r)
        if t>start+duration+.5:break
    arrivals.sort(key=lambda r:r['received_monotonic_ns']);cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'));results=[]
    from esrobo_laptop.hand_retarget import HandRetargeter, POLICY
    provenance=dict(source_sha256=hashlib.sha256(Path(inspect.getfile(HandRetargeter)).read_bytes()).hexdigest(),
        baseline_sha256=hashlib.sha256(Path(args.baseline).read_bytes()).hexdigest(),policy=POLICY,
        kernel_sha256=hashlib.sha256(Path(inspect.getfile(HandRetargeter)).with_name("_hand_native.cpp").read_bytes()).hexdigest())
    for repeat in range(args.repeats):
        for mode in ('old','new') if repeat%2==0 else ('new','old'):
            print('starting',repeat,mode,'arrivals',len(arrivals),'warm',len(warm),flush=True)
            r=run(mode,args.baseline,cfg,warm,arrivals,start,duration);r['repeat']=repeat;results.append(r)
            Path(args.output).write_text(json.dumps(dict(runs=results,note=__doc__,provenance=provenance),indent=2));print(json.dumps(r),flush=True)


if __name__=='__main__':main()
