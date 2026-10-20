import numpy as np, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import sys, os
os.environ['SKIP_INTERF']='1'
import model as Mo
from geom import *
exec(open('build.py').read().split('# ---------------------------------------------------------------- 1.')[0])
COL=dict(alu=(.78,.80,.82),alu2=(.55,.58,.62),steel=(.70,.72,.74),black=(.18,.19,.21),red=(.72,.22,.18),pcb=(.20,.50,.32),belt=(.12,.12,.12),stop=(.95,.55,.10),white=(.93,.93,.90))
def draw(pose, view, fn, lims=None):
    Tb=body_T(pose); tris=[]; cols=[]
    L=np.array([0.4,0.6,0.7]); L/=np.linalg.norm(L)
    for c in Mo.COMP:
        M=Tb[c['body']]; V=c['mesh'].V@M[:3,:3].T+M[:3,3]; t=V[c['mesh'].F]
        nn=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]); nn/=np.linalg.norm(nn,axis=1,keepdims=True)+1e-12
        sh=0.35+0.65*np.abs(nn@L)
        tris.append(t); cols.append(np.array(COL[c['mat']])[None,:]*sh[:,None])
    t=np.vstack(tris); cl=np.vstack(cols)
    fig=plt.figure(figsize=(10,9)); ax=fig.add_subplot(111,projection='3d')
    pc=Poly3DCollection(t[:,:,[0,2,1]],facecolors=np.clip(cl,0,1),edgecolors='none'); ax.add_collection3d(pc)
    lims=lims or [(-950,350),(-450,250),(-350,750)]
    ax.set_xlim(*lims[0]); ax.set_ylim(*lims[1]); ax.set_zlim(*lims[2]); ax.set_box_aspect([lims[i][1]-lims[i][0] for i in range(3)])
    ax.view_init(*view); ax.set_axis_off(); plt.tight_layout(); plt.savefig(fn,dpi=90); plt.close()
draw(dict(REF,theta=15,yaw=25,q=120,psi=30,grip=40),(18,-60),'out/preview_iso.png')
draw(dict(REF,theta=15,yaw=0,q=120,psi=30,grip=40),(0,-90),'out/preview_side.png')
draw(dict(REF,theta=15,yaw=25,q=120,psi=30,grip=40),(25,-130),'out/preview_back.png')
