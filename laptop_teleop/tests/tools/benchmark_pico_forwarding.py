"""Same synthetic source against historical, pre-fix and notification pipelines.

Does not initialize the real SDK. Historical freshness labels are not reused.
Run with the laptop/test/robot_link PYTHONPATH and --output result.json.
"""
import argparse
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from unittest.mock import patch

import numpy as np

from test_pico_transport import run_profiled, FakeSDK, Mapper, forwarder, Socket
from esrobo_laptop.config import WORKSPACE


class LegacySDK(FakeSDK):
    def get_body_sample(self):
        with self.condition:
            return dict(self.latest['body'], age_s=time.monotonic()-self.latest['body']['received_monotonic_ns']/1e9)
    def is_body_data_available(self):
        return self.latest['body']['sequence'] > 0
    def get_body_joints_pose(self):
        return self.get_body_sample()['poses']
    def get_body_timestamp_ns(self):
        return self.latest['body']['sequence']
    def get_body_joints_timestamp(self):
        return [self.latest['body']['sequence']]*24
    def get_time_stamp_ns(self):
        return self.latest['body']['sequence']


def module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def interval_stats(times):
    intervals = np.diff(times)*1000.
    return dict(p95_ms=float(np.percentile(intervals,95)) if len(intervals) else None,
                max_ms=float(max(intervals)) if len(intervals) else None)


def sources(root, pre_fix_ref, baseline_patch=None):
    paths=['laptop_teleop/src/esrobo_laptop/acquisition.py','teleoperation/bridges/xrobotoolkit_body_udp_bridge.py']
    for revision in ('cca9023','pre_fix'):
        for path in paths:
            target = root/revision/path
            target.parent.mkdir(parents=True,exist_ok=True)
            ref = pre_fix_ref if revision == 'pre_fix' else revision
            target.write_bytes(subprocess.check_output(['git','show',f'{ref}:{path}'],cwd=WORKSPACE))
        if revision=='pre_fix' and baseline_patch is not None:
            # An optional saved workspace diff is supplied explicitly; fresh
            # clones use a committed baseline and need no local runtime logs.
            diff=Path(baseline_patch).read_text()
            chunks=diff.split('diff --git ')
            selected=''.join('diff --git '+chunk for chunk in chunks[1:] if chunk.splitlines()[0].split()[0][2:] in paths)
            subprocess.run(['patch','--silent','-p1','-d',str(root/revision)],input=selected,text=True,check=True)


def baseline(root, revision, hz, delay, duration):
    base=root/revision
    bridge=module(base/'teleoperation/bridges/xrobotoolkit_body_udp_bridge.py','benchmark_bridge_'+revision)
    acq=module(base/'laptop_teleop/src/esrobo_laptop/acquisition.py','esrobo_laptop.benchmark_'+revision)
    sdk=LegacySDK(hz)
    times=[];sequences=[]
    class Hands:
        status={}
        def poll(self, sdk):
            time.sleep(delay)
            return {}
    def main(argv):
        sdk.start()
        last=None;seq=0;start=time.monotonic()
        try:
            while time.monotonic()-start < duration:
                tick=time.monotonic()
                packet,_=bridge._build_packet(sdk,seq,False)
                if packet is not None and bridge._packet_signature(packet)!=last:
                    last=bridge._packet_signature(packet)
                    sequences.append(packet.get('body_sample_sequence',packet.get('xrt_body_timestamp_ns')))
                    times.append(time.monotonic());seq+=1
                time.sleep(max(0.,1/90-(time.monotonic()-tick)))
        finally:
            sdk.close()
        return 0
    bridge.main=main
    with patch.object(acq,'load_bridge',lambda name:bridge), patch('esrobo_laptop.pico_hand.PicoHands',Hands), patch.object(acq.socket,'socket',lambda *a:Socket()), patch('sys.stdout',io.StringIO()):
        acq.pico(['--print-interval','1000'])
    return dict(input_hz=hz, hand_delay_ms=delay*1000., duration_s=duration,
                forwarded=len(times), sdk_committed=len(sdk.receipts), fraction=len(times)/max(len(sdk.receipts),1),
                intervals=interval_stats(times))


def fixed(hz,delay,duration):
    sdk=FakeSDK(hz);f=forwarder(sdk,mapper=Mapper(delay));f.args.samples=0
    timer=threading.Timer(duration,f.stop.set);sdk.start();timer.start()
    try:run_profiled(f)
    finally:sdk.close();timer.cancel()
    times=[t/1e9 for t,p in f.body_socket.packets]
    # Count source frames only through the last forwarded timestamp (hand worker
    # shutdown can finish later; it does not change forwarding performance).
    committed=sum(t<=times[-1]*1e9 for t in sdk.receipts) if times else 0
    return dict(input_hz=hz,hand_delay_ms=delay*1000.,duration_s=duration,forwarded=len(times),
                sdk_committed=committed,fraction=len(times)/max(committed,1),intervals=interval_stats(times),
                notification_to_send=f.test_profile['stages']['notification_to_send'])


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--output',required=True);ap.add_argument('--duration',type=float,default=1.2)
    ap.add_argument('--pre-fix-ref',default='6ffecfd',help='Committed baseline; does not change after publishing this tool')
    ap.add_argument('--baseline-patch',help='Optional workspace diff applied only to the committed baseline')
    args=ap.parse_args()
    result={}
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp);sources(root,args.pre_fix_ref,args.baseline_patch)
        for revision in ('cca9023','pre_fix','fixed'):
            cases=[]
            for hz,delay in [(60,0.),(74,0.),(90,0.),(74,.02),(74,.1),(74,.5)]:
                cases.append(fixed(hz,delay,args.duration) if revision=='fixed' else baseline(root,revision,hz,delay,args.duration))
            result[revision]=cases
    result['baseline'] = dict(pre_fix_ref=args.pre_fix_ref, workspace_patch=bool(args.baseline_patch))
    Path(args.output).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
