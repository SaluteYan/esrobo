"""Offline raw-hand replay and self-contained 3-D HTML; never opens hardware."""
import argparse
import copy
import json
from pathlib import Path

import numpy as np

from esrobo_laptop.config import ROOT, read_robot_config, read_settings
from esrobo_laptop.hand_retarget import HandRetargeter, TIPS, BONES
from esrobo_laptop.mapping import hand_units
from esrobo_laptop.pico_hand import retarget_hand
from esrobo_teleop.robot.linker_hand_driver import ACTIVE_HAND_JOINTS, L10_PHYSICAL_JOINT_NAMES


def feedback_angles(units, cfg):
    q = np.zeros(10)
    for i, name in enumerate(L10_PHYSICAL_JOINT_NAMES):
        c = cfg.hand.geometry_feedback_calibration['left'][name]
        q[ACTIVE_HAND_JOINTS.index(name)] = c['rad'][0]+(units[i]-c['raw'][0])/(c['raw'][1]-c['raw'][0])*(c['rad'][1]-c['rad'][0])
    return q


def synthetic_samples(model):
    """Explicitly labelled model fixtures, not recovered human observations."""
    for i, phase in enumerate(np.linspace(0, 2*np.pi, 160)):
        q = model.upper*(.45+.4*np.sin(phase+np.arange(10)*.21))
        points, _ = model.forward(q)
        poses = np.zeros((26,7)); poses[:,:3] = points; poses[:,6] = 1
        yield dict(source_sequence=i+1, sample=dict(poses=poses.tolist(), flags=[15]*26), synthetic=True)


def analyze(records, cfg, execution=()):
    solver = HandRetargeter(cfg); m = solver.model
    legacy = copy.deepcopy(cfg.hand)
    legacy.left_retarget_mode='legacy'; legacy.left_enabled_physical_joints=None
    observations = {}
    for row in execution:
        sequence = row.get('source_sequences',{}).get('left')
        if sequence is not None:
            observations[sequence] = row
    frames, new_errors, old_errors, latencies, dropped = [], [], [], [], 0
    for row in records:
        if 'sample' not in row or 'poses' not in row['sample']:
            continue
        sample = row['sample']
        poses = np.asarray(sample['poses'],dtype=float)
        # Replay valid geometry only; timing/occlusion rejections remain in source logs.
        if not np.isfinite(poses).all() or np.any((np.asarray(sample.get('flags',[15]*26)) & 3) != 3):
            dropped += 1; continue
        try:
            result = solver.solve(poses, budget_s=.020)
            old = hand_units(retarget_hand(poses),cfg.hand.left_open,legacy,'left',absolute_angles=True)
        except (ValueError,TimeoutError):
            dropped += 1; continue
        target = np.asarray(result['target_points'])
        predicted = np.asarray(result['solved_points'])
        old_points,_ = m.forward(feedback_angles(old,cfg))
        def errors(points):
            delta = (points[TIPS[1:]]-points[TIPS[0]])-(target[TIPS[1:]]-target[TIPS[0]])
            return np.linalg.norm(delta,axis=1)*1000
        new_errors.extend(errors(predicted)); old_errors.extend(errors(old_points))
        latencies.append(result['diagnostics']['solve_ms'])
        seq = row.get('source_sequence',sample.get('sequence'))
        frame = dict(sequence=seq,target=target.tolist(),solved=predicted.tolist(),legacy=old_points.tolist(),
                     diagnostics=result['diagnostics'],q=result['joints'])
        if 'reconstructed_points' in result:
            frame['reconstructed'] = result['reconstructed_points']
        observed = observations.get(seq)
        if observed:
            hand = observed.get('feedback_snapshot',{}).get('left') or {}
            if hand.get('position_unit') is not None:
                frame['feedback'] = m.forward(feedback_angles(hand['position_unit'],cfg))[0].tolist()
                frame['feedback_age_s'] = hand.get('age_s')
                # A simultaneous feedback snapshot is delayed execution, not same-target arrival.
                frame['feedback_note'] = '发送时反馈快照，含执行延迟，不代表该目标已经到位'
            frame['command'] = observed.get('targets',{}).get('left',{}).get('hand_unit')
        frames.append(frame)
    def stats(values):
        return dict(median=float(np.median(values)),p95=float(np.percentile(values,95)),maximum=float(np.max(values))) if values else None
    report = dict(frames=len(frames), rejected=dropped, model_fingerprint=m.digest,
                  new_pinch_error_mm=stats(new_errors),legacy_pinch_error_mm=stats(old_errors),
                  solve_ms=stats(latencies),
                  note='模型拟合误差；反馈使用独立标定。历史十角度日志不能还原原始人手姿态。')
    return frames, report


HTML = '''<!doctype html><meta charset="utf-8"><title>左手十轴离线回放</title>
<style>body{font:16px sans-serif;background:#102027;color:#eee;margin:24px}canvas{background:#162d36;border-radius:8px;width:100%;max-width:1000px}input{width:75%}pre{white-space:pre-wrap}button{padding:8px}span{margin-right:20px}</style>
<h2>左手十轴离线回放</h2><p>__LABEL__ · 拖动旋转视角；滑动选择帧。本页面不会连接机器人。</p>
<p><span style="color:#80cbc4">掌宽缩放人手目标</span><span style="color:#b9f6ca">机器人骨长重建目标</span><span style="color:#ffcc80">当前求解</span><span style="color:#ef9a9a">旧启发式映射（非 v2）</span><span style="color:#90caf9">发送时反馈快照（如有）</span></p>
<canvas id="view" width="1000" height="650"></canvas><p><button id="play">播放 / 暂停</button> <input id="frame" type="range" min="0" value="0" step="1"></p><pre id="detail"></pre>
<script>const data=__DATA__, report=__REPORT__;
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d'),slider=document.getElementById('frame');slider.max=Math.max(0,data.length-1);
let yaw=.6,pitch=-.4,drag=null,playing=false;const chains=[[2,3,4,5],[6,7,8,9,10],[11,12,13,14,15],[16,17,18,19,20],[21,22,23,24,25],[7,12,17,22]];
function project(p){let x=p[0],y=p[1],z=p[2]-.14;let a=x*Math.cos(yaw)-y*Math.sin(yaw),b=x*Math.sin(yaw)+y*Math.cos(yaw);return [500+2100*a,360-2100*(z*Math.cos(pitch)-b*Math.sin(pitch))];}
function draw(){ctx.clearRect(0,0,1000,650);const f=data[+slider.value];if(!f){document.getElementById('detail').textContent=JSON.stringify(report,null,2);return;}
for(const [key,color] of [['legacy','#ef9a9a'],['target','#80cbc4'],['reconstructed','#b9f6ca'],['solved','#ffcc80'],['feedback','#90caf9']]){if(!f[key])continue;ctx.strokeStyle=color;ctx.lineWidth=key==='solved'?3:1.5;for(const chain of chains){ctx.beginPath();chain.forEach((id,i)=>{const [x,y]=project(f[key][id]);i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();}}
document.getElementById('detail').textContent=JSON.stringify({frame:+slider.value,sequence:f.sequence,...f.diagnostics,feedback_age_s:f.feedback_age_s,feedback_note:f.feedback_note,summary:report},null,2);}
slider.oninput=draw;canvas.onpointerdown=e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);};canvas.onpointerup=()=>drag=null;canvas.onpointermove=e=>{if(drag){yaw+=(e.clientX-drag[0])*.008;pitch+=(e.clientY-drag[1])*.008;drag=[e.clientX,e.clientY];draw();}};
document.getElementById('play').onclick=()=>playing=!playing;setInterval(()=>{if(playing&&data.length){slider.value=(+slider.value+1)%data.length;draw();}},50);draw();</script>'''


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--raw',type=Path); source.add_argument('--synthetic',action='store_true')
    parser.add_argument('--execution',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    cfg=read_robot_config(read_settings(ROOT/'config/laptop.yaml'))
    def rows(path):
        with path.open() as stream:
            for line in stream:
                if line.strip(): yield json.loads(line)
    records=synthetic_samples(HandRetargeter(cfg).model) if args.synthetic else rows(args.raw)
    frames,report=analyze(records,cfg,rows(args.execution) if args.execution else ())
    report['source']='synthetic URDF fixtures' if args.synthetic else str(args.raw)
    label='URDF 合成样例，非实测人手' if args.synthetic else '原始人手日志离线拟合'
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(HTML.replace('__LABEL__',label).replace('__DATA__',json.dumps(frames).replace('<','\\u003c')).replace('__REPORT__',json.dumps(report,ensure_ascii=False).replace('<','\\u003c')),encoding='utf-8')
    args.output.with_suffix('.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
