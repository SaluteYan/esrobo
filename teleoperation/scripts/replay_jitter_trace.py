#!/usr/bin/env python3
"""Offline left-arm field trace replay; never opens CAN, SDK or a control socket.

Production filters -> segment limiter -> URDF IK -> wrist mapping -> trajectory.
Windows are relative to local trace start, not confirmed stationary labels.
Recorded feedback is a common counterfactual input, not predicted new feedback.
See docs/07_CODE_AND_COMPUTATION.md#滤波与连续死区 for reproducible invocation and limitations.
"""
import argparse
import hashlib
import inspect
import json,copy,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.signal import butter,sosfiltfilt
from esrobo_laptop.config import ROOT,read_settings,read_robot_config
from esrobo_laptop.mapping import WristMapping
from esrobo_teleop.ik.solver import IkSolver
from esrobo_teleop.device.body_device import BodyDevice
from esrobo_teleop.robot.command_trajectory import CommandTrajectory,TrajectoryStep
from esrobo_teleop.config import retarget_filter_fingerprint
def load(args):
    global BASE,cfg,A,t0,R,V,S,T,stages,observed,measured,qref,source_id,E,D,s0,poses0,sh,ul,fl
    BASE=args.output;BASE.mkdir(parents=True,exist_ok=True)
    cfg=read_robot_config(read_settings(args.settings))
    A=[json.loads(l) for l in args.motion.open()]
    t0=A[0]['unix_ns']/1e9
    R={x['context']['capture_id']:x for x in A if x['event']=='retarget_candidate'}
    V={x['context']['capture_id']:x for x in A if x['event']=='solve_candidate'}
    S=[x for x in A if x['event']=='network_send' and x.get('sent') and x.get('context')
       and x['kind']=='target' and args.start<x['unix_ns']/1e9-t0<args.end
       and x['context']['capture_id'] in V and x['context']['capture_id'] in R]
    if not S or len({x['session_tag'] for x in S})!=1:
        raise ValueError('select one nonempty control session')
    T=np.array([x['unix_ns']/1e9-t0 for x in S])
    stages=[R[x['context']['capture_id']]['stages'] for x in S]
    observed=np.array([x['payload']['arm_urdf_rad'] for x in S])
    measured=np.array([V[x['context']['capture_id']]['measured_rad'] for x in S])
    qref=np.array(next(x for x in A if x['event']=='solve_candidate')['measured_rad'])
    source_id={x['command_seq']:i for i,x in enumerate(S)}
    E=[json.loads(l) for l in args.execution.open()]
    D=[x for x in E if x.get('driver_event')=='command_sent'
       and (x.get('context') or {}).get('command_seq') in source_id
       and x.get('context',{}).get('session_tag')==S[0]['session_tag']]
    if not D or any(x['payload']['side']!='left' for x in S):
        raise ValueError('this field-session replay requires left-arm command/feedback pairs')
    references=[json.loads(l) for l in args.reference.open()]
    load.flexion=next(x['reference']['flexion_deg'] for x in references
                     if x.get('event')=='calibrated' and x.get('target_side')=='left')
    s0=solver_new();poses0=s0.current_task_frame_poses(qref)
    sh=poses0['left_shoulder'][:3]
    ul=np.linalg.norm(poses0['left_elbow'][:3]-sh)
    fl=np.linalg.norm(poses0['left_wrist'][:3]-poses0['left_elbow'][:3])

def solver_new():
 s=IkSolver(copy.deepcopy(cfg.ik));s.configure_position_envelope('left',cfg.robot.left_joint_soft_lower_limits_urdf,cfg.robot.left_joint_soft_upper_limits_urdf)
 s.initialize_arm_session('left',qref,np.deg2rad(load.flexion),relative_elbow_reference=False);s.reset_command_seed(measured[0]);return s
def solve_series(u,f,p):
 s=solver_new();w=WristMapping(s,cfg.robot,'left',qref,absolute_pose=True);qs=[];errors=[]
 for i,(uv,fv,pv) in enumerate(zip(u,f,p)):
  uv,fv=np.asarray(uv),np.asarray(fv);a=stages[i]['arm']['left'];poses={k:v.copy() for k,v in poses0.items()};el=poses['left_elbow'];wr=poses['left_wrist'];el[:3]=sh+ul*uv;wr[:3]=el[:3]+fl*fv
  q=s.solve(wr,poses['right_wrist'],measured[i],el,poses['right_elbow'],partitioned_side='left',terminal_joint_targets=w.reference_terminal)
  if not s.last_solution_valid:raise RuntimeError(s.solution_diagnostics)
  q=q.copy();q[4:7]=w.absolute_terminal(pv,q);qs.append(q[:7]);errors.append(s.solution_diagnostics.get('wrist_error_m'))
 return np.array(qs)
def filter_series():
    """Call the production filters; only their input acquisition is replayed."""
    from esrobo_laptop.orientation import AbsoluteHandOrientation,ForearmFrame
    d=BodyDevice.__new__(BodyDevice);d._cfg=copy.deepcopy(cfg.retarget)
    d.motion_trace_enabled=True
    d._filtered_segment_directions={'left':{}}
    d._filtered_segment_timestamps={'left':{}}
    d._held_segment_directions={'left':{}}
    for seg,key in [('upper_arm','upper'),('forearm','forearm')]:
        first=stages[0]['arm']['left']
        d._filtered_segment_directions['left'][seg]=np.array(first['lowpass_'+key+'_direction'],dtype=np.float32)
        d._filtered_segment_timestamps['left'][seg]=first['filter_details'][seg]['source_time']
        d._held_segment_directions['left'][seg]=np.array(first['filtered_before_plane_'+key+'_direction'])
    # The trace already contains the paired, forearm-local raw palm rotation.
    # A fixed carrier reproduces exactly this local input without inventing SDK poses.
    points=dict(shoulder=np.zeros(3),elbow=np.array([0.,0.,-.3]),wrist=np.array([.3,0.,-.3]))
    carrier=ForearmFrame().update(points)
    first=stages[0]['wrist']['left'];initial=np.array(first['absolute_local_orientation'])
    w=AbsoluteHandOrientation(points,carrier,palm_rotation=carrier@initial)
    w.reset_filter(initial);w.filter_stamp=first['source_sample_monotonic']
    outputs=[[],[],[]];durations=[]
    for i,stage in enumerate(stages):
        start=time.perf_counter_ns();arm=stage['arm']['left'];palm=stage['wrist']['left']
        for j,(seg,key,db) in enumerate([('upper_arm','upper',cfg.retarget.upper_arm_angular_deadband_deg),
                                        ('forearm','forearm',cfg.retarget.forearm_angular_deadband_deg)]):
            if i:
                d._last_body_frame_packet_time_monotonic=arm['filter_details'][seg]['source_time']
                value=d._filter_segment_direction('left',seg,np.array(arm['mapped_before_filter_'+key+'_direction']))
                value=d._apply_segment_direction_deadband('left',seg,value,db)
            else:value=d._held_segment_directions['left'][seg]
            outputs[j].append(value.copy())
        if i:
            value=w.update(points,carrier,palm_rotation=carrier@np.array(palm['source_local_orientation']),
                           sample_time=palm['source_sample_monotonic'],
                           min_cutoff_hz=cfg.retarget.wrist_orientation_filter_min_cutoff_hz,
                           max_cutoff_hz=cfg.retarget.wrist_orientation_filter_max_cutoff_hz,
                           speed_coefficient=cfg.retarget.wrist_orientation_filter_speed_coefficient,
                           deadband_deg=cfg.retarget.wrist_orientation_deadband_deg,
                           max_interval_s=.1)
        else:value=initial
        outputs[2].append(value.copy());durations.append((time.perf_counter_ns()-start)/1e6)
    return [np.array(v) for v in outputs],durations

def trajectory(qs,feedback_mode='recorded',tau=.06,targets_recorded=False):
 c=CommandTrajectory(cfg.robot.left_max_joint_velocity,cfg.robot.left_max_joint_acceleration,cfg.robot.left_max_joint_step,np.deg2rad(cfg.robot.tracking_command_trajectory_lead_deg),max_dt=cfg.robot.command_trajectory_max_dt_s,lead_timeout=cfg.robot.command_trajectory_lead_timeout_s)
 first=D[0];stamp=first['feedback_snapshot']['stamp_s'];dt=first['feedback_dt_s'];vel=np.array(first['velocity_urdf_rad_s']);q=np.array(first['command_urdf_rad']);c.reset(q-dt*vel,stamp-dt,feedback=np.array(first['feedback_urdf_rad']))
 # Recorded predecessor command/velocity comes from the preceding execution cycle.
 prev=[x for x in E if x['event']=='execution_cycle' and x['monotonic_ns']<first['monotonic_ns'] and x.get('feedback',{}).get('left',{}).get('motion')][-1]['feedback']['left']['motion']
 # The cycle before the first driver event is the previous cycle, because each driver event precedes its own execution_cycle.
 c.state=TrajectoryStep(np.array(prev['command_rad']),np.array(prev['velocity_rad_s']),stamp-dt,np.full(7,np.nan),{'feedback_physical_rad':first['feedback_urdf_rad'],'feedback_motion_stamp_s':stamp-dt,'feedback_velocity_rad_s':[0.]*7})
 fb=np.array(first['feedback_urdf_rad']);cmd=[];feedbacks=[];ts=[];fault=[]
 for j,row in enumerate(D):
  i=source_id[row['context']['command_seq']];stamp=row['feedback_snapshot']['stamp_s'];dt=stamp-c.state.stamp
  if dt<=0:continue
  if feedback_mode=='recorded':fb=np.array(row['feedback_urdf_rad'])
  else:fb=fb+(1-np.exp(-dt/tau))*(c.state.position-fb) if tau>0 else c.state.position.copy()
  target=np.array(row['ik_urdf_rad']) if targets_recorded else qs[i]
  try:step=c.propose(target,fb,stamp,np.array(cfg.robot.left_joint_soft_lower_limits_urdf),np.array(cfg.robot.left_joint_soft_upper_limits_urdf),coordinate_elbow=True)
  except Exception as ex:fault.append((j,str(ex)));break
  c.commit(step);cmd.append(step.position);feedbacks.append(fb.copy());ts.append(stamp-D[0]['feedback_snapshot']['stamp_s']+T[source_id[D[0]['context']['command_seq']]])
 return dict(t=np.array(ts),command=np.array(cmd),feedback=np.array(feedbacks),fault=fault)


def metrics(t,q,window):
    lo,hi=window;grid=np.arange(lo+.1,hi-.1,.02)
    if len(grid)<30 or t[0]>grid[0] or t[-1]<grid[-1]:
        raise ValueError('window outside replay or too short for spectrum')
    values=np.array([np.interp(grid,t,q[:,j]) for j in range(7)]).T*180/np.pi
    high=sosfiltfilt(butter(3,2,fs=50,btype='highpass',output='sos'),values,axis=0)
    return dict(span_deg=np.ptp(values,axis=0).tolist(),
                high_frequency_rms_deg=np.sqrt(np.mean(high**2,axis=0)).tolist(),
                high_frequency_combined_rms_deg=float(np.sqrt(np.mean(np.sum(high**2,axis=1)))),
                total_travel_deg=float(np.abs(np.diff(values,axis=0)).sum()))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('motion','execution','reference','output'):p.add_argument('--'+key,type=Path,required=True)
    p.add_argument('--settings',type=Path,default=ROOT/'config/laptop.yaml')
    p.add_argument('--start',type=float,required=True);p.add_argument('--end',type=float,required=True)
    p.add_argument('--windows',nargs='+',required=True,help='relative seconds, e.g. 27:33 42:49.5')
    args=p.parse_args();windows=[tuple(map(float,w.split(':'))) for w in args.windows]
    load(args)
    actual=np.array([x['command_urdf_rad'] for x in D])
    reconstructed=solve_series([d['arm']['left']['output_upper_direction'] for d in stages],
                               [d['arm']['left']['output_forearm_direction'] for d in stages],
                               [d['wrist']['left']['absolute_local_orientation'] for d in stages])
    baseline=trajectory(observed,targets_recorded=True)
    if baseline['fault']:raise RuntimeError(baseline['fault'])
    ik_error=float(np.max(abs(np.degrees(reconstructed-observed))))
    trajectory_error=float(np.max(abs(np.degrees(baseline['command']-actual))))
    if max(ik_error,trajectory_error)>.001:raise AssertionError('baseline IK/trajectory not reproduced')
    inputs,durations=filter_series();targets=solve_series(*inputs);candidate=trajectory(targets)
    if candidate['fault']:raise RuntimeError(candidate['fault'])
    report=dict(windows=windows,source_files={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
                for f in (args.motion,args.execution,args.reference)},
                filter_config=retarget_filter_fingerprint(cfg.retarget),
                implementation_sha256={str(f):hashlib.sha256(f.read_bytes()).hexdigest()
                    for f in (Path(__file__).resolve(),Path(cfg.ik.urdf_path),
                              Path(inspect.getfile(BodyDevice)),Path(inspect.getfile(IkSolver)),
                              Path(inspect.getfile(CommandTrajectory)),
                              ROOT/'src/esrobo_laptop/orientation.py',
                              ROOT.parent/'teleoperation/src/esrobo_teleop/math_utils.py')},
                source_session_tag=S[0]['session_tag'],target_count=len(S),execution_count=len(D),
                baseline_ik_max_error_deg=ik_error,baseline_trajectory_max_error_deg=trajectory_error,
                filter_group_ms=dict(zip(['p50','p95','p99'],np.percentile(durations,[50,95,99]).tolist())),
                comparisons=[],sensitivity=[],
                limitations=['Low-motion windows are not confirmed stationary labels.',
                    'Recorded feedback replay does not predict changed actuator dynamics.',
                    'Filters start at recorded low-pass/held state; allow warmup before analysis.',
                    'No contact, torque or collision certification is performed by this replay.'])
    for window in windows:
        old=metrics(baseline['t'],baseline['command'],window);new=metrics(candidate['t'],candidate['command'],window)
        ratio=1-new['high_frequency_combined_rms_deg']/old['high_frequency_combined_rms_deg']
        per_axis=1-np.array(new['high_frequency_rms_deg'])/np.array(old['high_frequency_rms_deg'])
        report['comparisons'].append(dict(before=old,after=new,reduction=ratio,per_axis_reduction=per_axis.tolist()))
    for tau in (0.,.03,.06,.1):
        old=trajectory(observed,feedback_mode='model',tau=tau);new=trajectory(targets,feedback_mode='model',tau=tau)
        if old['fault'] or new['fault']:raise RuntimeError((old['fault'],new['fault']))
        report['sensitivity'].append(dict(assumed_feedback_time_constant_s=tau,reduction=[
            1-metrics(new['t'],new['command'],w)['high_frequency_combined_rms_deg']/
            metrics(old['t'],old['command'],w)['high_frequency_combined_rms_deg'] for w in windows]))
    report['passed']=all(c['reduction']>=.2 and min(c['per_axis_reduction'])>=-.05 for c in report['comparisons'])
    np.savez(BASE/'series.npz',input_time=T,old_targets=observed,new_targets=targets,
             execution_time=candidate['t'],old_commands=baseline['command'],new_commands=candidate['command'])
    (BASE/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['passed']:raise SystemExit('jitter reduction acceptance failed')


if __name__=='__main__':main()
