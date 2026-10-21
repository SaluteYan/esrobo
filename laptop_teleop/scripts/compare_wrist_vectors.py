#!/usr/bin/env python3
"""Reproduce the offline 5/9-vector and hand-size sensitivity experiment.

Run with external/dex-vector-venv/bin/python and single-thread BLAS. No hardware.
The first two seconds establish one fixed width; this is not a marked calibration
pose. The coarse multiplier sweep is sensitivity analysis, not model selection.
"""
import argparse,json,os,subprocess,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandModel,TIPS,BONES,FINGERS


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True)
    ap.add_argument('--published',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'.keep').touch()
    z=np.load(a.input);width=np.linalg.norm(z['poses'][:,7,:3]-z['poses'][:,22,:3],axis=1)
    calibration=z['t']<=z['t'][0]+2
    fixed=float(np.median(width[calibration]))
    m=HandModel(read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml')))
    points=np.array([m.human_target(p) for p in z['poses']]);neutral=m.forward(m.neutral)[0]
    sizes=dict(robot_width_m=float(m.width),human_width_quantile_mm=dict(zip(['min','p05','p50','p95','max'],(np.percentile(width,[0,5,50,95,100])*1000).tolist())),
        alpha_palm_quantile=dict(zip(['p05','p50','p95'],np.percentile(m.width/width,[5,50,95]).tolist())),
        reference_width_m=fixed,reference_prefix_seconds=2,reference_samples=int(calibration.sum()),reference_last_source_time=float(z['t'][calibration][-1]),
        prefix_has_no_marked_calibration_pose=True,
        robot_wrist_to_mcp_centroid_mm=float(np.linalg.norm(m.origin)*1000),
        human_scaled_wrist_to_mcp_centroid_quantile_mm=np.percentile(np.linalg.norm(points[:,[7,12,17,22]].mean(axis=1)-points[:,1],axis=1)*1000,[5,50,95]).tolist(),
        robot_neutral_wrist_tip_mm=(np.linalg.norm(neutral[TIPS]-neutral[1],axis=1)*1000).tolist(),
        human_scaled_wrist_tip_p95_mm=(np.percentile(np.linalg.norm(points[:,TIPS]-points[:,1,None,:],axis=2),95,axis=0)*1000).tolist())
    raw=z['poses'][:,:,:3]
    human_bones=np.linalg.norm(raw[:,BONES[:,1]]-raw[:,BONES[:,0]],axis=2).reshape(-1,5,3).sum(2)*1000
    robot_bones=np.linalg.norm(neutral[BONES[:,1]]-neutral[BONES[:,0]],axis=1).reshape(5,3).sum(1)*1000
    bones={f:dict(robot_three_bones_mm=float(robot_bones[i]),human_median_three_bones_mm=float(np.median(human_bones[:,i])),
        length_ratio=float(robot_bones[i]/np.median(human_bones[:,i]))) for i,f in enumerate(FINGERS)}
    (a.output/'bone-size.json').write_text(json.dumps(bones,indent=2))
    (a.output/'size-input.json').write_text(json.dumps(sizes,indent=2))
    cases=[('v13','v13',3,[])]
    cases += [(mode,mode,3,[]) for mode in ('wrist5','wrist9')]
    for mode in ('wrist5','wrist9'):
        cases += [(f'{mode}-s{v}',mode,1,['--scale-factor',str(v)]) for v in (.8,1.2,1.4)]
        cases += [(mode+'-fixed',mode,1,['--scale-mode','fixed','--reference-width-m',str(fixed)]),
                  (mode+'-none',mode,1,['--scale-mode','none'])]
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONHASHSEED='0')
    for label,mode,repeats,extra in cases:
        cmd=[sys.executable,str(Path(__file__).with_name('replay_hand_vector.py')),
             '--input',str(a.input),'--published',str(a.published),'--output',str(a.output),
             '--mode',mode,'--label',label,'--repeats',str(repeats)]+extra
        print('Starting',label,flush=True)
        with (a.output/f'{label}.log').open('w') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env,check=True)
        report=json.loads((a.output/f'{label}-report.json').read_text())
        print(label,[(r['solve_ms'],len(r['errors'])) for r in report],flush=True)
    (a.output/'cases.json').write_text(json.dumps([c[0] for c in cases],indent=2))

if __name__=='__main__':main()
