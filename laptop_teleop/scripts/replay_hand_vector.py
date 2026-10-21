#!/usr/bin/env python3
"""Offline full-prefix replay; never opens a socket or controls a device.

Example: external/dex-vector-venv/bin/python scripts/replay_hand_vector.py \
  --input log/retained/hand_episode_v13_20261021/input.npz \
  --published log/retained/hand_episode_v13_20261021/published.json \
  --output log/retained/dex_vector_20261021 --mode palm10 --repeats 3
"""
import argparse
from pathlib import Path
import sys
import json
import time
import hashlib
import gc
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):
    sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandRetargeter
from esrobo_laptop.hand_vector_experiment import VectorHandRetargeter, PRESETS


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--published',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--mode',choices=['v13', *PRESETS],required=True)
    ap.add_argument('--scale-mode',choices=['palm','fixed','none'],default='palm')
    ap.add_argument('--scale-factor',type=float,default=1.)
    ap.add_argument('--reference-width-m',type=float)
    ap.add_argument('--label',help='unique output label for scale comparisons')
    ap.add_argument('--repeats',type=int,default=3)
    ap.add_argument('--budget-ms',type=float,default=20.)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    label=a.label or a.mode
    if Path(label).name != label or not label:ap.error('label must be a file name')
    if a.mode=='v13' and (a.scale_mode!='palm' or a.scale_factor!=1 or a.reference_width_m is not None):
        ap.error('v13 scaling is unchanged; scaling options apply only to vector experiments')
    z=dict(np.load(a.input));published=set(json.loads(a.published.read_text()))
    cfg=read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml'))
    reports=[]
    for repeat in range(a.repeats):
        s=HandRetargeter(cfg) if a.mode=='v13' else VectorHandRetargeter(cfg,preset=a.mode,scale_mode=a.scale_mode,
            scale_factor=a.scale_factor,reference_width_m=a.reference_width_m)
        count=len(z['seq']);qs=np.full((count,10),np.nan);elapsed=np.zeros(count)
        accepted=np.zeros(count,dtype=bool);evaluations=np.zeros(count,dtype=int)
        errors=[];gc.collect()
        for i,(seq,t,poses) in enumerate(zip(z['seq'],z['t'],z['poses'])):
            start=time.perf_counter()
            try:
                r=s.solve(poses,source_time=float(t),commit=False,budget_s=a.budget_ms/1000)
                elapsed[i]=(time.perf_counter()-start)*1000
                if elapsed[i]>=a.budget_ms:
                    s.discard();raise TimeoutError('publication deadline')
                qs[i]=r['joints'];evaluations[i]=r['diagnostics'].get('evaluations',0)
                if int(seq) in published:
                    accepted[i]=s.commit(r['candidate_id'])
                else:s.discard()
            except Exception as e:
                elapsed[i]=(time.perf_counter()-start)*1000
                s.discard();errors.append(dict(seq=int(seq),kind=type(e).__name__,reason=str(e)))
            if i and i%3000==0:print(a.mode,repeat,i,'frames',len(errors),'rejected',flush=True)
        suffix='' if a.budget_ms==20 else '-quality'
        np.savez_compressed(a.output/f'{label}{suffix}-{repeat}.npz',seq=z['seq'],t=z['t'],q=qs,
                            elapsed_ms=elapsed,accepted=accepted,evaluations=evaluations)
        meta=s.metadata if a.mode!='v13' else dict(version=r['mapping_version'],fingerprint=r['mapping_fingerprint'])
        report=dict(mode=a.mode,label=label,repeat=repeat,frames=count,accepted=int(accepted.sum()),
            budget_ms=a.budget_ms,solve_ms=dict(zip(['p50','p95','p99','max'],np.percentile(elapsed,[50,95,99,100]).tolist())),
            errors=errors,metadata=meta,input_sha256=hashlib.sha256(a.input.read_bytes()).hexdigest(),
            published_sha256=hashlib.sha256(a.published.read_bytes()).hexdigest())
        reports.append(report)
        (a.output/f'{label}{suffix}-report.json').write_text(json.dumps(reports,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k not in ('errors','metadata')})+' rejected='+str(len(errors)),flush=True)
        if hasattr(s,'close'):s.close()

if __name__=='__main__':main()
