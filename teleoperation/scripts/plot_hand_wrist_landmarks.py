#!/usr/bin/env python3
"""Locate the exact wrist markers used by the offline 5/9-vector experiment.

Draw the recorded PICO skeleton and actual URDF visual meshes. No hardware,
retargeter changes, inferred skin surface, or anatomical wrist calibration.
"""
from pathlib import Path
import argparse,json,sys,xml.etree.ElementTree as ET
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.font_manager import FontProperties
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[2]
for p in ('teleoperation/src','robot_link'):sys.path.insert(0,str(ROOT/p))
from esrobo_laptop.config import read_settings,read_robot_config
from esrobo_laptop.hand_retarget import HandModel,TIPS


def transform(origin):
    t=np.eye(4)
    if origin is not None:
        t[:3,3]=np.fromstring(origin.get('xyz','0 0 0'),sep=' ')
        t[:3,:3]=Rotation.from_euler('xyz',np.fromstring(origin.get('rpy','0 0 0'),sep=' ')).as_matrix()
    return t


def mesh_triangles(cfg,model):
    tree=ET.parse(cfg.ik.urdf_path).getroot();links={l.get('name'):l for l in tree.findall('link')}
    joints={j.get('name'):j for j in tree.findall('joint')}
    frames={'leftHand_link':np.eye(4)};values={}
    for name,i in model.joint_index.items():
        j=joints['left_'+name];v=model.joints[i];angle=model.neutral[v[4]]*v[5]+v[6];values[name]=float(angle)
        t=frames[j.find('parent').get('link')]@transform(j.find('origin'))
        rot=np.eye(4);rot[:3,:3]=Rotation.from_rotvec(v[3]*angle).as_matrix()
        frames[j.find('child').get('link')]=t@rot
    triangles=[]
    for link,t in frames.items():
        for visual in links[link].findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:continue
            filename=mesh.get('filename').replace('package://esrobo_waist_with_head/','')
            path=ROOT/'teleoperation/assets/esrobo_waist_with_head'/filename
            data=path.read_bytes();count=int.from_bytes(data[80:84],'little')
            d=np.frombuffer(data,dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('attr','<u2')]),count=count,offset=84)
            p=d['v'].astype(float)*np.fromstring(mesh.get('scale','1 1 1'),sep=' ')
            world=t@transform(visual.find('origin'));p=p@world[:3,:3].T+world[:3,3]
            triangles.append(p)
    return np.concatenate(triangles),frames,values


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--input',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    plt.rcParams.update({'font.family':font.get_name(),'axes.unicode_minus':False,'font.size':11})
    cfg=read_robot_config(read_settings(ROOT/'teleoperation/config/laptop.yaml'));m=HandModel(cfg)
    z=np.load(a.input);raw=z['poses'][:,:,:3]
    # Select a clearly extended recorded pose for a legible node drawing only.
    # This is not a labelled neutral/calibration frame.
    chains=np.stack([raw[:,start:start+4] for start in (7,12,17,22)],axis=1)
    bone=np.diff(chains,axis=2);unit=bone/np.maximum(np.linalg.norm(bone,axis=3,keepdims=True),1e-12)
    score=np.arccos(np.clip((unit[:,:,:-1]*unit[:,:,1:]).sum(3),-1,1)).sum((1,2))
    widths=np.linalg.norm(raw[:,7]-raw[:,22],axis=1)
    lengths=np.linalg.norm(raw[:,[7,12,17,22]].mean(1)-raw[:,1],axis=1)*m.width/widths
    # Avoid choosing a narrow-palm outlier just because fingers look straight.
    typical=(np.abs(widths-np.median(widths))<.05*np.median(widths)) & (np.abs(lengths-np.median(lengths))<.05*np.median(lengths))
    possible=np.flatnonzero(typical)
    if not len(possible):raise ValueError('no representative frame')
    i=int(possible[np.argmin(score[possible])]);h=m.human_target(z['poses'][i]);h=h-h[1].copy()
    p,_=m.forward(m.neutral);center=p[[7,12,17,22]].mean(0)
    triangles,frames,angles=mesh_triangles(cfg,m)
    # Check mesh joint-frame evaluation against existing model markers.
    for slot,name in [(2,'thumb_cmc_pitch'),(3,'thumb_mcp'),(4,'thumb_ip'),(12,'middle_mcp_pitch')]:
        j=ET.parse(cfg.ik.urdf_path).getroot().find(f"joint[@name='left_{name}']")
        np.testing.assert_allclose(frames[j.find('child').get('link')][:3,3],p[slot],atol=1e-12)
    basis=m.basis;local_tri=triangles@basis*1000;local_h=h@basis*1000;local_p=p@basis*1000
    fig,axes=plt.subplots(1,3,figsize=(16,8.5),gridspec_kw={'width_ratios':[1,1,1]})
    red='#c62828';green='#00876c';purple='#9652a6';blue='#2375b5'
    def mark(ax,xy,color,label,where,marker='o'):
        ax.scatter(*xy,c=color,s=85,marker=marker,edgecolors='white',linewidths=1,zorder=8)
        ax.annotate(label,xy,xytext=where,textcoords='data',color=color,fontsize=10,
            ha='left',va='center',bbox=dict(boxstyle='round,pad=.3',fc='white',ec=color,alpha=.95),
            arrowprops=dict(arrowstyle='->',color=color,lw=1.3),zorder=9)
    for k,ax in enumerate(axes):
        horizontal=1 if k<2 else 0
        ax.set_aspect('equal');ax.set_ylim(-32,265);ax.set_xlim(-100,105)
        ax.grid(alpha=.15);ax.set_ylabel('沿手掌长度方向 / mm')
        ax.set_xlabel(('横向' if k<2 else '掌面法向')+' / mm')
        if k==0:
            for chain in ([1,2,3,4,5],[1,6,7,8,9,10],[1,11,12,13,14,15],[1,16,17,18,19,20],[1,21,22,23,24,25]):
                pts=local_h[chain];ax.plot(pts[:,horizontal],pts[:,2],'-o',color='#6c8191',markersize=4,lw=2)
            for slot in range(26):
                if slot not in (0,1):ax.annotate(str(slot),(local_h[slot,horizontal],local_h[slot,2]),xytext=(4,3),textcoords='offset points',fontsize=8)
            c=local_h[[7,12,17,22]].mean(0)
            mark(ax,local_h[1,[horizontal,2]],red,'本次人手腕点：节点1\nPICO WRIST',(-92,-17))
            mark(ax,local_h[0,[horizontal,2]],purple,'节点0：PICO掌心\n不是本次起点',(-92,35),marker='s')
            mark(ax,c[[horizontal,2]],green,'四指MCP平均点\n也不是本次腕点',(-92,112),marker='D')
            ax.set_title('① 人手：PICO实测节点\n掌宽缩放，腕点平移到原点',fontsize=13,pad=16)
            ax.text(-94,240,f"示意源序号：{int(z['seq'][i])}\n尺寸接近中位数的较伸展帧\n非人工标定姿态",fontsize=10,color='#444')
            dist=np.linalg.norm(h[[7,12,17,22]].mean(0))*1000
            ax.text(-94,205,f'本帧腕→MCP中心三维距离\n{dist:.1f} mm',fontsize=10,color=green)
        else:
            # Orthographic mesh rendering, depth-sorted at triangle centres.
            depth=0 if horizontal==1 else 1
            order=np.argsort(local_tri[:,:,depth].mean(1))
            tris=local_tri[order];normal=np.cross(tris[:,1]-tris[:,0],tris[:,2]-tris[:,0])
            light=np.abs(normal[:,depth])/np.maximum(np.linalg.norm(normal,axis=1),1e-12)
            colors=np.stack([.63+.24*light,.69+.21*light,.74+.18*light],axis=1)
            ax.add_collection(PolyCollection(tris[:,:,[horizontal,2]],facecolors=colors,edgecolors='none',rasterized=True,zorder=2))
            mark(ax,np.zeros(2),red,'本次机器人腕点\nleftHand_link 原点\n局部坐标 (0, 0, 0)',(-92,-17))
            c=center@basis*1000
            mark(ax,c[[horizontal,2]],green,'四指MCP平均点\n旧 palm10 的参考点',(-92,177),marker='D')
            ax.plot([0,c[horizontal]],[0,c[2]],'--',color=green,lw=1.4,zorder=4)
            ax.set_title('② 机器人：URDF模型正视' if k==1 else '③ 同一模型侧视',fontsize=13,pad=16)
            if k==1:ax.text(-94,247,'真实URDF网格＋中立关节姿态\n不是实机照片或实测反馈',fontsize=10,color='#444')
            else:ax.text(-94,244,f'根原点→MCP中心\n三维距离 {np.linalg.norm(center)*1000:.1f} mm',fontsize=11,color=green)
    fig.suptitle('刚才5／9向量测试的腕点：红点才是实际向量起点',fontsize=18,y=.98)
    fig.text(.5,.035,'三图同一毫米尺度；人手仅做原有掌宽缩放和显示平移。机器人腕点是URDF根原点，尚未标定为人体解剖腕点的等价位置。',ha='center',fontsize=11)
    fig.subplots_adjust(top=.87,bottom=.12,wspace=.28)
    path=a.output/'wrist_landmarks.png';fig.savefig(path,dpi=170);plt.close(fig)
    meta=dict(source_seq=int(z['seq'][i]),source_time=float(z['t'][i]),human_wrist_node=1,human_palm_node=0,
        human_raw_wrist_xyz_m=raw[i,1].tolist(),human_display_wrist_xyz_m=h[1].tolist(),human_scaled_wrist_to_mcp_m=float(np.linalg.norm(h[[7,12,17,22]].mean(0))),
        selection='Most extended among frames with palm width and scaled wrist-MCP length within 5% of recorded medians; not a calibration pose',
        robot_wrist_link='leftHand_link',robot_local_wrist_xyz_m=[0,0,0],robot_neutral_mcp_centroid_xyz_m=center.tolist(),robot_wrist_to_mcp_m=float(np.linalg.norm(center)),
        neutral_active_q_rad=m.neutral.tolist(),joint_frames_match_model=True,
        scope='Recorded human skeleton and actual URDF visual mesh; display only, no robot commands; shared mm scale; human wrist aligned only for display.')
    (a.output/'wrist_landmarks.json').write_text(json.dumps(meta,ensure_ascii=False,indent=2));print(json.dumps(meta,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
