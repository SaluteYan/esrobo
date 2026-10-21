#!/usr/bin/env python3
"""High-budget multi-start checks at difficult observed poses, offline only.

Rebuild the exact filtered target with full prefix history. Keep the same last
published q in the temporal loss while varying optimizer initial guesses.
A finite multi-start search is not proof of the global optimum/unreachability.
"""
from pathlib import Path
import argparse,sys,json
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_vector_experiment import VectorHandRetargeter
from esrobo_laptop.hand_retarget import TIPS


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();z=np.load(a.input);metrics=np.load(a.output/'metrics.npz');results={}
    cfg=read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml'))
    for mode in ('wrist5','wrist9'):
        s=VectorHandRetargeter(cfg,preset=mode);q=np.load(a.output/f'{mode}-0.npz')['q']
        assert np.isfinite(q).all(), 'reference reconstruction requires no rejected frames'
        indices=set(int(np.flatnonzero(z['seq']==seq)[0]) for seq in (8909,17002))
        for f in range(4):
            near=np.flatnonzero(metrics['valid']&(metrics['raw_gap'][:,f]<16))
            indices.add(int(near[np.argmax(metrics[mode+'_gap'][near,f])]))
        wanted={}
        for i,(p,t) in enumerate(zip(z['poses'],z['t'])):
            state,_=s.shape_filter.propose(s.human_points(p),float(t));s.shape_filter.commit(state)
            if i in indices:wanted[i]=state.output.copy()
            if i>=max(indices):break
        s.optimizer.opt.set_maxeval(400);s.optimizer.opt.set_ftol_abs(1e-10);s.optimizer.opt.set_maxtime(.5)
        cases=[];rng=np.random.default_rng(7)
        for i,points in wanted.items():
            last=q[i-1];vectors=points[s.pairs[:,1]]-points[s.pairs[:,0]]
            upstream=s.optimizer.get_objective_function(vectors,np.array([]),last.copy())
            def objective(x,g):return upstream(x,g)+.004*float((x-last)@(x-last))
            s.optimizer.opt.set_min_objective(objective)
            starts=[last,s.model.neutral,(s.model.lower+s.model.upper)/2,s.model.lower,s.model.upper]
            starts += [rng.uniform(s.model.lower,s.model.upper) for _ in range(4)]
            best=q[i].copy();online_loss=best_loss=objective(best,np.array([]));errors=[]
            for seed in starts:
                try:
                    found=s.optimizer.opt.optimize(np.array(seed));status=s.optimizer.opt.last_optimize_result()
                    loss=objective(found,np.array([]))
                    if status>0 and status!=6 and np.isfinite(found).all() and loss<best_loss:
                        best=found.copy();best_loss=loss
                except Exception as e:errors.append(type(e).__name__)
            def describe(x):
                p=s.model.forward(x)[0];h=metrics['human'][i]
                direction=(p[3]-p[2])/np.linalg.norm(p[3]-p[2]);human=(h[3]-h[2])/np.linalg.norm(h[3]-h[2])
                return dict(thumb_direction_deg=float(np.degrees(np.arccos(np.clip(direction@human,-1,1)))),
                    model_gaps_mm=(np.linalg.norm(p[TIPS[1:]]-p[5],axis=1)*1000).tolist(),joint_deg=np.degrees(x).tolist())
            cases.append(dict(seq=int(z['seq'][i]),online_loss=online_loss,reference_loss=best_loss,
                relative_loss_reduction=float((online_loss-best_loss)/max(online_loss,1e-12)),online=describe(q[i]),reference=describe(best),errors=errors))
        results[mode]=cases;s.close()
    report=dict(cases=results,starts_per_case=9,maxeval=400,deadline_per_start_s=.5,
        scope='Diagnostic finite multi-start search of the SAME filtered objective/previous-q penalty, not global optimality proof, not runtime parameters.')
    (a.output/'reference.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))

if __name__=='__main__':main()
