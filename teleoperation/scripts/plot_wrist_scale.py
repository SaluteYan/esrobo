#!/usr/bin/env python3
"""Plot the predefined uniform-scale sweep; no fitting/tuning on the test set."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();r=json.loads((a.output/'comparison.json').read_text())['results']
    fig,axes=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
    factors=[.8,1.,1.2,1.4]
    for row,mode in enumerate(('wrist5','wrist9')):
        names=[mode if f==1 else f'{mode}-s{f}' for f in factors]
        for col,(key,title) in enumerate([('own_unfiltered_scaled_wrist_error_mm','Fit to each scaled wrist target (mm)'),('thumb_proximal_deg','Thumb direction error (deg)'),('relative_tip_error_mm','Pair error vs SAME reference (mm)')]):
            ax=axes[row,col]
            ax.plot(factors,[r[n][key]['median'] for n in names],'o-',label='median')
            ax.plot(factors,[r[n][key]['p95'] for n in names],'s--',label='P95')
            ax.set_title(mode+': '+title);ax.set_xlabel('Extra uniform scale multiplier');ax.grid(alpha=.2)
    axes[0,0].legend();fig.suptitle('Scale sensitivity, same full history. Own-target fit is NOT a cross-scale accuracy score.')
    fig.savefig(a.output/'scale_sensitivity.png',dpi=150);plt.close(fig)
    # Exact vector definitions drawn over one representative observed hand.
    data=np.load(a.output/'metrics.npz');i=int(np.flatnonzero(data['valid'])[0])
    h=(data['human'][i]-data['human'][i,1])*1000
    fig=plt.figure(figsize=(12,6))
    for col,n in enumerate((5,9)):
        ax=fig.add_subplot(1,2,col+1,projection='3d')
        for start in (2,7,12,17,22):
            p=h[start:start+4];ax.plot(*p.T,color='#777777',marker='o',markersize=3)
        for k,tip in enumerate((5,10,15,20,25)):
            v=h[tip]-h[1];ax.quiver(*h[1],*v,color='#2375b5',arrow_length_ratio=.05,alpha=.8)
            ax.text(*h[tip],('T','I','M','R','P')[k])
        if n==9:
            for tip in (10,15,20,25):
                v=h[tip]-h[5];ax.quiver(*h[5],*v,color='#d55e00',arrow_length_ratio=.1)
        ax.text(*h[1],'W');ax.set_title(f'{n} vectors: W -> tips'+(' + T -> other tips' if n==9 else ''))
        ax.set_xlabel('X mm');ax.set_ylabel('Y mm');ax.set_zlabel('Z mm');ax.set_box_aspect((1,1,1));ax.view_init(elev=20,azim=-50)
    fig.suptitle('PICO node 1 = wrist (W); tips = 5, 10, 15, 20, 25. Recorded pose, not contact labels.')
    fig.tight_layout();fig.savefig(a.output/'vector_definitions.png',dpi=160);plt.close(fig)

if __name__=='__main__':main()
