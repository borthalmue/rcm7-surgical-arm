"""Small z-buffer renderer with crease-aware shading and CAD-style outlines, for labelled stills."""
import numpy as np, math, os
os.environ['SKIP_INTERF'] = '1'
import model as Mo
from geom import *
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
_src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'build.py')).read()
exec(_src.split('# ---------------------------------------------------------------- 1.')[0])
exec(_src[_src.index('def crease'):_src.index('payload = dict(')])

COL = dict(alu=(.83, .85, .87), alu2=(.50, .55, .60), steel=(.74, .76, .79), black=(.19, .20, .22), red=(.78, .22, .16),
           pcb=(.16, .56, .33), belt=(.12, .12, .13), stop=(.97, .57, .10), white=(.95, .94, .91))
_cache = {}
def part_geom(c):
    if c['name'] not in _cache:
        P, N = crease(c['mesh']); _cache[c['name']] = (P.astype(float).reshape(-1, 3, 3), (N.astype(float)/127).reshape(-1, 3, 3))
    return _cache[c['name']]

def look_at(eye, target, up=(0, 1, 0)):
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    f = target - eye; f /= np.linalg.norm(f); r = np.cross(f, up); r /= np.linalg.norm(r); u = np.cross(r, f)
    return eye, np.stack([r, u, -f])

def render(pose, eye, target, W=1400, H=1000, fov=30, hide=(), ss=2, light=(-0.45, 0.8, 0.6), bg=(0.933, 0.945, 0.953), ghost=()):
    Tb = body_T(pose); eye, Rc = look_at(eye, target)
    w, h = W*ss, H*ss; fpx = 0.5*h/math.tan(math.radians(fov)/2)
    zbuf = np.full((h, w), np.inf); col = np.zeros((h, w, 3)); pid = np.full((h, w), -1, int); nbuf = np.zeros((h, w, 3))
    L = np.asarray(light, float); L /= np.linalg.norm(L)
    for k, c in enumerate(Mo.COMP):
        if c['name'] in hide: continue
        P, N = part_geom(c); M = Tb[c['body']]
        Pw = P @ M[:3, :3].T + M[:3, 3]; Nw = N @ M[:3, :3].T
        Pc = (Pw - eye) @ Rc.T                       # camera coords, looking down -z
        z = -Pc[..., 2]
        ok = (z > 5).all(1)
        Pc, z, Nw, Pw = Pc[ok], z[ok], Nw[ok], Pw[ok]
        if not len(z): continue
        sx = w/2 + fpx*Pc[..., 0]/z; sy = h/2 - fpx*Pc[..., 1]/z
        base = np.array(COL[c['mat']])
        V = eye - Pw; V /= np.linalg.norm(V, axis=2, keepdims=True)
        for t in range(len(z)):
            x0, x1 = int(max(0, math.floor(sx[t].min()))), int(min(w-1, math.ceil(sx[t].max())))
            y0, y1 = int(max(0, math.floor(sy[t].min()))), int(min(h-1, math.ceil(sy[t].max())))
            if x0 > x1 or y0 > y1: continue
            X, Y = np.meshgrid(np.arange(x0, x1+1)+0.5, np.arange(y0, y1+1)+0.5)
            (xa, xb, xc), (ya, yb, yc) = sx[t], sy[t]
            den = (yb-yc)*(xa-xc) + (xc-xb)*(ya-yc)
            if abs(den) < 1e-9: continue
            l1 = ((yb-yc)*(X-xc) + (xc-xb)*(Y-yc))/den; l2 = ((yc-ya)*(X-xc) + (xa-xc)*(Y-yc))/den; l3 = 1-l1-l2
            inside = (l1 >= -1e-6) & (l2 >= -1e-6) & (l3 >= -1e-6)
            if not inside.any(): continue
            iz = l1/z[t, 0] + l2/z[t, 1] + l3/z[t, 2]; zz = 1/iz
            sub = zbuf[y0:y1+1, x0:x1+1]; m = inside & (zz < sub)
            if not m.any(): continue
            w1, w2, w3 = (l1/z[t, 0])*zz, (l2/z[t, 1])*zz, (l3/z[t, 2])*zz
            nn = w1[..., None]*Nw[t, 0] + w2[..., None]*Nw[t, 1] + w3[..., None]*Nw[t, 2]
            nn /= np.linalg.norm(nn, axis=2, keepdims=True) + 1e-12
            vv = V[t].mean(0)
            facing = np.sign((nn @ vv))[..., None]; nn = nn*np.where(facing == 0, 1, facing)
            dif = np.clip(nn @ L, 0, 1); hv = L + vv; hv /= np.linalg.norm(hv)
            spec = np.clip(nn @ hv, 0, 1)**40
            metal = 0.35 if c['mat'] in ('steel', 'alu', 'alu2') else 0.12
            shade = base*(0.36 + 0.64*dif)[..., None] + metal*spec[..., None]
            sub[m] = zz[m]; col[y0:y1+1, x0:x1+1][m] = shade[m]; pid[y0:y1+1, x0:x1+1][m] = k; nbuf[y0:y1+1, x0:x1+1][m] = nn[m]
    img = np.where(np.isinf(zbuf)[..., None], np.array(bg), col)
    # outlines: part boundaries, depth jumps, normal creases
    edge = np.zeros((h, w), bool)
    for dy, dx in ((0, 1), (1, 0)):
        a, b = pid, np.roll(np.roll(pid, -dy, 0), -dx, 1)
        za, zb = zbuf, np.roll(np.roll(zbuf, -dy, 0), -dx, 1)
        na, nb = nbuf, np.roll(np.roll(nbuf, -dy, 0), -dx, 1)
        e = (a != b) | (np.abs(np.nan_to_num(za-zb, posinf=1e9, neginf=1e9)) > 0.012*np.minimum(np.nan_to_num(za, posinf=1e9), np.nan_to_num(zb, posinf=1e9))) | ((na*nb).sum(2) < 0.82)
        edge |= e & ~(np.isinf(za) & np.isinf(zb))
    img[edge] = img[edge]*0.25 + np.array([0.11, 0.16, 0.20])*0.75
    img = img.reshape(H, ss, W, ss, 3).mean((1, 3))
    proj = lambda p: (lambda pc: (W/2 + fpx/ss*pc[0]/-pc[2], H/2 - fpx/ss*pc[1]/-pc[2]))((np.asarray(p, float)-eye) @ Rc.T)
    return np.clip(img, 0, 1), proj, Tb

def figure(img, proj, Tb, labels, title, sub, fn):
    H, W = img.shape[:2]
    fig = plt.figure(figsize=(W/100, (H+90)/100), dpi=100)
    ax = fig.add_axes([0, 0, 1, H/(H+90)]); ax.imshow(img); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis('off')
    fig.text(0.025, 1-32/(H+90), title, fontsize=21, fontweight='bold', color='#1c2a33', va='top', family='DejaVu Sans')
    fig.text(0.025, 1-64/(H+90), sub, fontsize=12.5, color='#5a6a75', va='top', family='DejaVu Sans')
    pts = []
    for txt, body, p, side in labels:
        q = pt(Tb[body], np.asarray(p, float)); x, y = proj(q); pts.append([txt, x, y, side])
    for side in ('L', 'R'):
        grp = sorted([p for p in pts if p[3] == side], key=lambda p: p[2])
        ys = []
        for p in grp:
            y = max(p[2], (ys[-1] + 40) if ys else 30); ys.append(min(y, H-20))
        for p, ly in zip(grp, ys):
            lx = 22 if side == 'L' else W-22
            ax.annotate(p[0], xy=(p[1], p[2]), xytext=(lx, ly), fontsize=12, color='#1c2a33', fontweight='bold',
                        ha='left' if side == 'L' else 'right', va='center', family='DejaVu Sans',
                        bbox=dict(boxstyle='round,pad=0.28', fc='white', ec='#9fb0bb', lw=0.8),
                        arrowprops=dict(arrowstyle='-', color='#d4470f', lw=1.3, shrinkA=0, shrinkB=2))
            ax.plot([p[1]], [p[2]], 'o', ms=5, color='#d4470f', mec='white', mew=1)
    fig.savefig(fn, facecolor='#eef1f3'); plt.close(fig)
