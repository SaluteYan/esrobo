#!/usr/bin/env python3
"""Offline recorded-input target-geometry comparison. No robot connection.

Uses the retained full-prefix publication history and the latest aligned input
snapshot. Latest snapshot has actual filtered points but no pre-run raw prefix:
all variants share the first recorded q; first 2 seconds are excluded from
scoring, never silently described as a full-history reconstruction.
"""
from pathlib import Path
import argparse,json,sys,time,hashlib
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
for p in ('laptop_teleop/src','teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandRetargeter,FINGERS,TIPS,BONES
from esrobo_laptop.hand_vector_experiment import VectorHandRetargeter
from esrobo_laptop.hand_vector_geometry import GeometryVectorHandRetargeter,GEOMETRY_MODES
from esrobo_laptop.hand_shape_filter import HandShapeFilter


def stat(a):
    a=np.asarray(a);a=a[np.isfinite(a)]
    return None if not a.size else dict(n=int(a.size),median=float(np.median(a)),p95=float(np.percentile(a,95)),p99=float(np.percentile(a,99)))


def prepare(dataset,output,cfg):
    retained=ROOT/'laptop_teleop/log/retained'
    if dataset=='latest':
        path=retained/'wrist5_four_fingers_20261021/aligned.npz';z=dict(np.load(path))
        result={k:z[k] for k in ('seq','t','poses','h')}
        result.update(published=np.ones(len(z['seq']),bool),used=z['t']>=z['t'][0]+2,
                      initial_q=z['q'][0],recorded_q=z['q'])
        limitation='Saved normal-following samples only; actual filtered points; initial q is first recorded result, first 2 s excluded. No missing prehistory invented.'
    else:
        path=retained/'hand_episode_v13_20261021/input.npz';z=dict(np.load(path))
        pub=json.loads((path.parent/'published.json').read_text());published=np.isin(z['seq'],pub)
        ref=HandRetargeter(cfg);sf=HandShapeFilter(cfg.hand);h=[]
        for pose,t,accept in zip(z['poses'],z['t'],published):
            candidate,_=sf.propose(ref.model.human_target(pose),float(t));h.append(candidate.output.copy())
            if accept:sf.commit(candidate)
        old=dict(np.load(retained/'thumb_unintended_v12_20261020/data.npz'))
        result=dict(z,h=np.asarray(h),published=published,used=np.isin(z['seq'],old['seq'][old['used']]),initial_q=ref.model.neutral)
        limitation='Full retained prefix and actual publication sequence; shape filter commits only published inputs. Geometry experiments do not reuse contact labels as ground truth.'
    result['h'][:,0]=HandRetargeter(cfg).model.origin
    np.savez_compressed(output/f'{dataset}-input.npz',**result)
    (output/f'{dataset}-provenance.json').write_text(json.dumps(dict(source=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),limitation=limitation),indent=2))
    return result


def metrics(z,qs,points,ref):
    m=ref.model;valid=z['used']&np.isfinite(qs).all(axis=1);human=z['h']
    hv=human[:,BONES[:,1]]-human[:,BONES[:,0]];rv=points[:,BONES[:,1]]-points[:,BONES[:,0]]
    hd=hv/np.linalg.norm(hv,axis=2)[:,:,None];rd=rv/np.linalg.norm(rv,axis=2)[:,:,None]
    err=np.degrees(np.arccos(np.clip((hd*rd).sum(axis=2),-1,1))).reshape(-1,5,3)
    direct=np.array([ref.direct_fingers(h) for h in human]);flex=[m.names.index(f+'_mcp_pitch') for f in FINGERS[1:]]
    rawgap=np.linalg.norm(z['poses'][:,TIPS[1:],:3]-z['poses'][:,5,None,:3],axis=2)*1000
    gap=np.linalg.norm(points[:,TIPS[1:]]-points[:,5,None,:],axis=2)*1000
    relerr=np.linalg.norm((points[:,TIPS[1:]]-points[:,5,None,:])-(human[:,TIPS[1:]]-human[:,5,None,:]),axis=2)*1000
    adjacent=valid[1:]&valid[:-1]&(np.diff(z['t'])<=.1)
    result=dict(valid=int(valid.sum()),four_direction_deg=stat(err[valid,1:]),thumb_three_bone_deg=stat(err[valid,0].mean(1)),
                per_finger={},joint_step_p99_deg=np.percentile(abs(np.degrees(np.diff(qs,axis=0)))[adjacent],99,axis=0).tolist(),
                limit_violations=int(np.sum(np.any((qs<m.lower-1e-9)|(qs>m.upper+1e-9),axis=1)&valid)))
    for f,name in enumerate(FINGERS[1:]):
        k=flex[f];open_mask=valid&(direct[:,k]<np.deg2rad(15));near=valid&(rawgap[:,f]<16)
        result['per_finger'][name]=dict(direction_deg=stat(err[valid,f+1].mean(1)),
            upper_limit_fraction=float(np.mean(qs[valid,k]>m.upper[k]-np.deg2rad(.2))),
            open_target_flex_deg=stat(np.degrees(qs[open_mask,k])),
            open_over45_fraction=float(np.mean(qs[open_mask,k]>np.deg2rad(45))) if open_mask.any() else None,
            flex_vs_three_segment_deg=stat(abs(np.degrees(qs[valid,k]-direct[valid,k]))),
            near_model_gap_mm=stat(gap[near,f]),near_raw_gap_mm=stat(rawgap[near,f]),near_relative_error_mm=stat(relerr[near,f]))
    return result,dict(direction=err,gap=gap,rawgap=rawgap,direct=direct)


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--dataset',choices=['latest','historical'],required=True)
    ap.add_argument('--modes',nargs='+',default=['original',*GEOMETRY_MODES]);ap.add_argument('--repeats',type=int,default=1)
    ap.add_argument('--thumb-ulnar-gain',type=float)
    ap.add_argument('--finger-lateral-offset-mm',nargs=4,type=float)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True);(a.output/'.keep').touch()
    cfg=read_robot_config(read_settings(ROOT/'laptop_teleop/config/laptop.yaml'))
    if a.thumb_ulnar_gain is not None:cfg.hand.left_vector_thumb_ulnar_gain=a.thumb_ulnar_gain
    if a.finger_lateral_offset_mm is not None:cfg.hand.left_vector_finger_lateral_offset_mm=a.finger_lateral_offset_mm
    ref=HandRetargeter(cfg)
    path=a.output/f'{a.dataset}-input.npz';z=dict(np.load(path)) if path.exists() else prepare(a.dataset,a.output,cfg)
    for mode in a.modes:
        reports=[]
        for repeat in range(a.repeats):
            s=VectorHandRetargeter(cfg,preset='wrist5',objective_backend='analytic') if mode=='original' else GeometryVectorHandRetargeter(cfg,geometry_mode=mode)
            s.previous=z['initial_q'].copy();qs=[];points=[];timing=[];errors=[];accepted=[];construction=[]
            for seq,t,h,pub in zip(z['seq'],z['t'],z['h'],z['published']):
                start=time.perf_counter()
                try:
                    r=s.solve(None,target=h,source_time=float(t),budget_s=.020)
                    elapsed=(time.perf_counter()-start)*1000
                    q=np.asarray(r['joints']);p=s._diagnostic_points(q)
                    if pub:
                        if not s.commit(r['candidate_id']):raise RuntimeError('candidate commit rejected')
                    else:s.discard()
                except Exception as ex:
                    timing.append((time.perf_counter()-start)*1000);qs.append(np.full(10,np.nan));points.append(np.full((26,3),np.nan));accepted.append(False)
                    errors.append(dict(seq=int(seq),error=str(ex)));s.discard()
                else:
                    timing.append(elapsed);qs.append(q);points.append(p);accepted.append(bool(pub))
                tick=time.perf_counter();s.build_vectors(h);construction.append((time.perf_counter()-tick)*1000)
            qs=np.array(qs);points=np.array(points)
            report,extra=metrics(z,qs,points,ref);report.update(mode=mode,repeat=repeat,errors=errors,
                solve_ms=stat(timing),construction_ms=stat(construction),metadata=s.metadata)
            if 'recorded_q' in z:report['recorded_thumb_delta_deg']=stat(abs(np.degrees(qs[z['used'],:3]-z['recorded_q'][z['used'],:3])))
            np.savez_compressed(a.output/f'{a.dataset}-{mode}-{repeat}.npz',seq=z['seq'],q=qs,points=points,elapsed_ms=timing,accepted=accepted,**extra)
            reports.append(report);s.close()
            (a.output/f'{a.dataset}-{mode}.json').write_text(json.dumps(reports,indent=2))
            print(a.dataset,mode,repeat,'dir',report['four_direction_deg'],'solve',report['solve_ms'],'errors',len(errors),flush=True)

if __name__=='__main__':main()
