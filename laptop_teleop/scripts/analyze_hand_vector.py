#!/usr/bin/env python3
"""Compare raw-input geometry against offline vector/v13 outputs, not hardware."""
from pathlib import Path
import argparse,json,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandModel,TIPS,BONES


def stats(a):
    a=np.asarray(a);a=a[np.isfinite(a)]
    return None if not len(a) else dict(n=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)))


def directions(p):
    v=p[BONES[:,1]]-p[BONES[:,0]]
    return v/np.maximum(np.linalg.norm(v,axis=1)[:,None],1e-12)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--record',type=Path,required=True,help='saved real-session seq/used mask')
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--historical-v12',type=Path,help='optional exact-history v12 replay for last physical run')
    ap.add_argument('--modes',nargs='+',default=['v13','palm10','bones19'])
    a=ap.parse_args();o=a.output
    z=dict(np.load(a.input));record=dict(np.load(a.record))
    used=np.isin(z['seq'],record['seq'][record['used']])
    model=HandModel(read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml')))
    modes=a.modes.copy()
    runs={n:dict(np.load(o/f'{n}-0.npz')) for n in modes}
    if a.historical_v12 is not None:
        modes.insert(0,'v12_recorded_baseline')
        runs['v12_recorded_baseline']=dict(np.load(a.historical_v12))
    valid=used.copy()
    for d in runs.values():
        np.testing.assert_array_equal(d['seq'],z['seq']);valid &= np.isfinite(d['q']).all(axis=1)
    human=np.array([model.human_target(p) for p in z['poses']])
    rawgap=np.linalg.norm(z['poses'][:,TIPS[1:],:3]-z['poses'][:,5,None,:3],axis=2)*1000
    pair=human[:,TIPS[1:]]-human[:,5,None,:]
    human_dir=np.array([directions(p) for p in human])
    results={};metrics={}
    width=np.linalg.norm(z['poses'][:,7,:3]-z['poses'][:,22,:3],axis=1)
    for mode,data in runs.items():
        points=np.array([model.forward(q)[0] if np.isfinite(q).all() else np.full((26,3),np.nan) for q in data['q']])
        gap=np.linalg.norm(points[:,TIPS[1:]]-points[:,5,None,:],axis=2)*1000
        relerr=np.linalg.norm(points[:,TIPS[1:]]-points[:,5,None,:]-pair,axis=2)*1000
        wristerr=np.linalg.norm((points[:,TIPS]-points[:,1,None,:])-(human[:,TIPS]-human[:,1,None,:]),axis=2)*1000
        tiperr=np.linalg.norm(points[:,TIPS]-human[:,TIPS],axis=2)*1000
        robotdir=np.array([directions(p) for p in points])
        direrr=np.degrees(np.arccos(np.clip(np.sum(robotdir*human_dir,axis=2),-1,1)))
        metrics[mode]=dict(wrist_vector_error=wristerr,gap=gap,relative_error=relerr,tip_error=tiperr,direction_error=direrr,points=points)
        q=data['q'];adjacent=valid[1:]&valid[:-1]&(np.diff(z['t'])<=.1)
        delta=np.abs(np.degrees(np.diff(q,axis=0)))[adjacent]
        report_path=o/f'{mode}-report.json'
        meta=json.loads(report_path.read_text())[0]['metadata'] if report_path.exists() else {}
        factor=np.full(len(width),meta.get('scale_factor',1.),dtype=float)
        if meta.get('scale_mode')=='fixed':factor*=width/meta['reference_width_m']
        elif meta.get('scale_mode')=='none':factor*=width/model.width
        own_wrist=(human[:,TIPS]-human[:,1,None,:])*factor[:,None,None]
        own_pair=pair*factor[:,None,None]
        own_wrist_error=np.linalg.norm(points[:,TIPS]-points[:,1,None,:]-own_wrist,axis=2)*1000
        own_pair_error=np.linalg.norm(points[:,TIPS[1:]]-points[:,5,None,:]-own_pair,axis=2)*1000
        results[mode]=dict(own_unfiltered_scaled_wrist_error_mm=stats(own_wrist_error[valid]),
            own_unfiltered_scaled_pair_error_mm=stats(own_pair_error[valid]),
            finite_formal_samples=int((used&np.isfinite(q).all(axis=1)).sum()),
            wrist_vector_error_mm=stats(wristerr[valid]),thumb_proximal_deg=stats(direrr[valid,0]),thumb_three_segment_deg=stats(direrr[valid,:3].mean(axis=1)),
            all_finger_direction_deg=stats(direrr[valid,3:]),relative_tip_error_mm=stats(relerr[valid]),tip_position_error_mm=stats(tiperr[valid]),
            joint_total_variation_deg=np.sum(delta,axis=0).tolist(),joint_step_p99_deg=np.percentile(delta,99,axis=0).tolist(),
            limit_fraction_per_axis=np.mean((q[valid]-model.lower<np.deg2rad(.2))|(model.upper-q[valid]<np.deg2rad(.2)),axis=0).tolist(),
            per_finger={})
        for f,name in enumerate(['index','middle','ring','pinky']):
            near=valid&(rawgap[:,f]<16)
            # Pure geometric stratum, not human-confirmed contact labels.
            results[mode]['per_finger'][name]=dict(raw_gap_under16mm_count=int(near.sum()),
                model_gap_mm=stats(gap[near,f]),raw_human_gap_mm=stats(rawgap[near,f]),
                relative_vector_error_mm=stats(relerr[near,f]),thumb_proximal_deg=stats(direrr[near,0]))
    incidents={}
    for seq,f in [(8909,3),(17002,2)]:
        hits=np.flatnonzero(z['seq']==seq)
        if not len(hits):continue
        i=int(hits[0]);incidents[str(seq)]=dict(raw_gap_mm=float(rawgap[i,f]),
            modes={n:dict(model_gap_mm=float(v['gap'][i,f]),thumb_direction_deg=v['direction_error'][i,:3].tolist(),joint_deg=np.degrees(runs[n]['q'][i]).tolist()) for n,v in metrics.items()})
    report=dict(input_frames=len(z['seq']),formal_used=int(used.sum()),common_valid=int(valid.sum()),
        limitation='Raw gap <16 mm is only a geometric stratum, not confirmed intended pinch. Offline model output is not actual feedback. Same source/publication history, no CAN/executor simulation.',
        reference='All common-reference errors use original per-frame palm-width scaling, independent of the solver scale. Own-target errors are separately named, unfiltered, and not a fair cross-scale quality ranking.',
        fixed_calibration_overlaps_formal=False if not any((used & (z['t']<=z['t'][0]+2))) else True,
        human_palm_width_mm=stats(width[valid]*1000),
        human_palm_scale=stats(model.width/width[valid]),
        joint_order=model.names,results=results,incidents=incidents)
    (o/'comparison.json').write_text(json.dumps(report,indent=2))
    np.savez_compressed(o/'metrics.npz',seq=z['seq'],t=z['t'],valid=valid,human=human,raw_gap=rawgap,
                        **{n+'_'+k:v for n,d in metrics.items() for k,v in d.items()})
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
