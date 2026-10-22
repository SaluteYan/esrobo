#!/usr/bin/env python3
"""Controlled offline replay: freeze four human finger chains; move thumb only."""
from pathlib import Path
import argparse,sys,json
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandRetargeter
from esrobo_laptop.hand_vector_experiment import VectorHandRetargeter
from esrobo_laptop.hand_vector_geometry import GeometryVectorHandRetargeter, GEOMETRY_MODES

def create(cfg,mode):
    if mode in GEOMETRY_MODES:
        return GeometryVectorHandRetargeter(cfg,geometry_mode=mode)
    return HandRetargeter(cfg) if mode=='v13' else VectorHandRetargeter(cfg,preset=mode,objective_backend='analytic')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--modes',nargs='+',default=['v13','palm10','bones19'])
    a=ap.parse_args();z=dict(np.load(a.input))
    cfg=read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml'))
    result={}
    for label,start,end in [('pinky_near',8860,8920),('ring_near',16930,17060)]:
        chosen=np.flatnonzero((z['seq']>=start)&(z['seq']<=end))
        if len(chosen)<2:continue
        result[label]={}
        for mode in a.modes:
            s=create(cfg,mode)
            fixed=s.model.human_target(z['poses'][chosen[0]]);stamp=1.
            # Settled fixed posture, so residual convergence is not scored as thumb-induced.
            for _ in range(100):
                s.solve(None,target=fixed,source_time=stamp,budget_s=1.,commit=True);stamp+=.02
            qs=[];gaps=[]
            for i in chosen:
                thumb=s.model.human_target(z['poses'][i]);target=fixed.copy();target[2:6]=thumb[2:6]
                r=s.solve(None,target=target,source_time=stamp,budget_s=1.,commit=True);stamp+=.02
                q=np.asarray(r['joints']);qs.append(q)
                p=s.model.forward(q)[0];gaps.append(np.linalg.norm(p[[10,15,20,25]]-p[5],axis=1)*1000)
            qs=np.array(qs);gaps=np.array(gaps)
            control=create(cfg,mode)
            stamp=1.;control_q=[]
            for j in range(100+len(chosen)):
                cr=control.solve(None,target=fixed,source_time=stamp,budget_s=1.,commit=True);stamp+=.02
                if j>=100:control_q.append(cr['joints'])
            control_q=np.asarray(control_q)
            if hasattr(control,'close'):control.close()
            result[label][mode]=dict(frames=len(qs),
                four_finger_max_difference_from_static_control_deg=np.degrees(np.max(np.abs(qs[:,3:]-control_q[:,3:]),axis=0)).tolist(),
                static_control_peak_to_peak_deg=np.degrees(np.ptp(control_q[:,3:],axis=0)).tolist(),
                four_finger_axis_peak_to_peak_deg=np.degrees(np.ptp(qs[:,3:],axis=0)).tolist(),
                thumb_axis_peak_to_peak_deg=np.degrees(np.ptp(qs[:,:3],axis=0)).tolist(),model_min_gap_mm=np.min(gaps,axis=0).tolist())
            if hasattr(s,'close'):s.close()
    (a.output/'isolation.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
