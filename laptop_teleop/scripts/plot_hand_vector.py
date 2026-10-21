#!/usr/bin/env python3
"""Render metric comparison and 3D URDF/raw-input overlays from offline replay."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--modes',nargs='+',default=['v13','palm10','bones19'])
    ap.add_argument('--align-wrist',action='store_true')
    a=ap.parse_args();o=a.output
    m=dict(np.load(o/'metrics.npz'));report=json.loads((o/'comparison.json').read_text())
    names=a.modes;colors=plt.get_cmap('tab10').colors
    fig,axes=plt.subplots(2,2,figsize=(12,8),constrained_layout=True)
    for ax,(key,title) in zip(axes.ravel(),[('thumb_proximal_deg','Thumb proximal direction error (deg)'),('relative_tip_error_mm','Thumb-to-finger vector error (mm)'),(('wrist_vector_error_mm' if a.align_wrist else 'tip_position_error_mm'),('Wrist-to-tip vector error (mm)' if a.align_wrist else 'Fingertip position error (mm)')),('all_finger_direction_deg','Four-finger bone direction error (deg)')]):
        x=np.arange(len(names))
        ax.bar(x-.16,[report['results'][n][key]['median'] for n in names],width=.32,label='median')
        ax.bar(x+.16,[report['results'][n][key]['p95'] for n in names],width=.32,label='P95')
        ax.set_xticks(x,names);ax.set_title(title);ax.grid(axis='y',alpha=.2)
    axes[0,0].legend();fig.suptitle('Same recorded PICO input; offline model predictions, NOT measured execution')
    fig.savefig(o/'comparison.png',dpi=160);plt.close(fig)
    fig=plt.figure(figsize=(16,8))
    for row,seq in enumerate([8909,17002]):
        i=int(np.flatnonzero(m['seq']==seq)[0]);human=m['human'][i].copy()
        if a.align_wrist:human-=human[1].copy()
        slots=[1,2,3,4,5,7,8,9,10,12,13,14,15,17,18,19,20,22,23,24,25]
        allp=np.vstack([human[slots]]+[m[n+'_points'][i,slots] for n in names])*1000
        center=(allp.max(0)+allp.min(0))/2;radius=max(np.ptp(allp,axis=0))/2+5
        for col,mode in enumerate(names):
            ax=fig.add_subplot(2,len(names),row*len(names)+col+1,projection='3d');p=m[mode+'_points'][i]
            if a.align_wrist:
                for pts,c in [(human,'#555555'),(p,colors[col])]:
                    ax.scatter(*pts[1]*1000,color=c,marker='x')
                    for tip in (5,10,15,20,25):
                        seg=pts[[1,tip]]*1000
                        ax.plot(seg[:,0],seg[:,1],seg[:,2],color=c,alpha=.25,linewidth=.7)
            for start in (2,7,12,17,22):
                for pts,c,ls in [(human,'#555555','--'),(p,colors[col],'-')]:
                    seg=pts[start:start+4]*1000
                    ax.plot(seg[:,0],seg[:,1],seg[:,2],color=c,linestyle=ls,marker='o',markersize=2)
            for slot,label in zip((5,10,15,20,25),('T','I','M','R','P')):
                ax.text(*(p[slot]*1000),label,fontsize=8,color=colors[col])
            ax.set_xlim(center[0]-radius,center[0]+radius);ax.set_ylim(center[1]-radius,center[1]+radius);ax.set_zlim(center[2]-radius,center[2]+radius)
            ax.set_box_aspect((1,1,1));ax.view_init(elev=25,azim=-45)
            ax.set_title(f'{mode}, source {seq}');ax.set_xlabel('X mm');ax.set_ylabel('Y mm');ax.set_zlabel('Z mm')
    fig.suptitle('Dashed: palm-width-scaled human; solid: robot FK. '+('Wrist origins aligned; no resizing for display.' if a.align_wrist else 'No root repositioning.'))
    fig.tight_layout();fig.savefig(o/'skeletons.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
