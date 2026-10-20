"""Mesh primitives: watertight extrusions (ear clipping with hole bridging), cylinders, tubes, transforms."""
import numpy as np, math

class Mesh:
    def __init__(self, V=None, F=None):
        self.V = np.zeros((0, 3)) if V is None else np.asarray(V, float)
        self.F = np.zeros((0, 3), int) if F is None else np.asarray(F, int)
    def __add__(self, o):
        return Mesh(np.vstack([self.V, o.V]), np.vstack([self.F, o.F + len(self.V)]))

# ---------- 2D profiles ----------
def area2(p):
    x, y = p[:, 0], p[:, 1]
    return np.dot(x, np.roll(y, -1)) - np.dot(np.roll(x, -1), y)

def circle(c, r, seg=48):
    a = np.linspace(0, 2*np.pi, seg, endpoint=False)
    return np.stack([c[0]+r*np.cos(a), c[1]+r*np.sin(a)], 1)

def stadium(x0, x1, hw, seg=48):
    h = seg//2
    a1 = np.linspace(-np.pi/2, np.pi/2, h+1)
    a2 = np.linspace(np.pi/2, 3*np.pi/2, h+1)
    right = np.stack([x1+hw*np.cos(a1), hw*np.sin(a1)], 1)
    left = np.stack([x0+hw*np.cos(a2), hw*np.sin(a2)], 1)
    return np.vstack([right, left])

def rect(x0, x1, y0, y1):
    return np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], float)

# ---------- triangulation ----------
def _segx(p1, p2, p3, p4):
    def o(a, b, c): return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    d1, d2, d3, d4 = o(p3, p4, p1), o(p3, p4, p2), o(p1, p2, p3), o(p1, p2, p4)
    return d1*d2 < -1e-12 and d3*d4 < -1e-12

def _bridge(outer, holes):
    from matplotlib.path import Path
    paths = [Path(np.asarray(h)) for h in holes]
    opath = Path(np.asarray(outer))
    ts = np.linspace(0.02, 0.98, 25)[:, None]
    def clear(M, P):
        s = np.asarray(M) + ts*(np.asarray(P) - np.asarray(M))
        if not opath.contains_points(s, radius=-1e-9).all() and not opath.contains_points(s, radius=1e-9).all(): return False
        return not any(pth.contains_points(s, radius=1e-9).any() for pth in paths)
    poly = [tuple(p) for p in outer]
    holes = sorted([[tuple(p) for p in h] for h in holes], key=lambda h: -max(q[0] for q in h))
    for k, h in enumerate(holes):
        mi = max(range(len(h)), key=lambda i: h[i][0]); M = h[mi]
        rest = holes[k+1:]
        order = sorted(range(len(poly)), key=lambda i: (poly[i][0]-M[0])**2 + (poly[i][1]-M[1])**2)
        chosen = None
        for ci in order:
            P = poly[ci]; ok = clear(M, P)
            for loop in ([poly, h] + rest) if ok else []:
                L = len(loop)
                for j in range(L):
                    a, b = loop[j], loop[(j+1) % L]
                    if a in (P, M) or b in (P, M): continue
                    if _segx(M, P, a, b): ok = False; break
                if not ok: break
            if ok: chosen = ci; break
        assert chosen is not None
        poly = poly[:chosen+1] + h[mi:] + h[:mi] + [h[mi], poly[chosen]] + poly[chosen+1:]
    return poly

def _earclip(poly):
    P = np.array(poly); idx = list(range(len(P))); tris = []
    def cr(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    while len(idx) > 3:
        n = len(idx); found = False
        for i in range(n):
            ia, ib, ic = idx[i-1], idx[i], idx[(i+1) % n]
            a, b, c = P[ia], P[ib], P[ic]
            if cr(a, b, c) <= 1e-10: continue
            ok = True
            for j in idx:
                if j in (ia, ib, ic): continue
                q = P[j]
                if (abs(q-a).sum() < 1e-9) or (abs(q-b).sum() < 1e-9) or (abs(q-c).sum() < 1e-9): continue
                if cr(a, b, q) >= -1e-10 and cr(b, c, q) >= -1e-10 and cr(c, a, q) >= -1e-10: ok = False; break
            if ok: tris.append((ia, ib, ic)); idx.pop(i); found = True; break
        assert found, "earclip failed"
    tris.append(tuple(idx))
    return P, tris

def extrude(outer, holes, z0, z1):
    outer = np.asarray(outer, float); holes = [np.asarray(h, float) for h in holes]
    if area2(outer) < 0: outer = outer[::-1]
    holes = [h[::-1] if area2(h) > 0 else h for h in holes]
    P2, tris = _earclip(_bridge(outer, holes))
    key, U = {}, []
    def vid(p, z):
        k = (round(p[0], 6), round(p[1], 6), z)
        if k not in key: key[k] = len(U); U.append([p[0], p[1], z])
        return key[k]
    F = []
    for a, b, c in tris:
        F.append([vid(P2[a], z1), vid(P2[b], z1), vid(P2[c], z1)])
        F.append([vid(P2[c], z0), vid(P2[b], z0), vid(P2[a], z0)])
    for loop in [outer] + holes:
        L = len(loop)
        for i in range(L):
            a, b = loop[i], loop[(i+1) % L]
            a0, b0, a1, b1 = vid(a, z0), vid(b, z0), vid(a, z1), vid(b, z1)
            F += [[a0, b0, b1], [a0, b1, a1]]
    return Mesh(np.array(U), np.array(F))

def box(x0, x1, y0, y1, z0, z1):
    return extrude(rect(x0, x1, y0, y1), [], z0, z1)

def cyl_x(r, x0, x1, r_in=0.0, seg=48):
    a = np.linspace(0, 2*np.pi, seg, endpoint=False); cs = np.stack([np.cos(a), np.sin(a)], 1)
    V, F = [], []
    def ring(x, rr): b = len(V); V.extend([[x, rr*c, rr*s] for c, s in cs]); return b
    o0, o1 = ring(x0, r), ring(x1, r)
    for i in range(seg):
        j = (i+1) % seg; F += [[o0+i, o0+j, o1+j], [o0+i, o1+j, o1+i]]
    if r_in > 0:
        i0, i1 = ring(x0, r_in), ring(x1, r_in)
        for i in range(seg):
            j = (i+1) % seg
            F += [[i0+i, i1+j, i0+j], [i0+i, i1+i, i1+j]]
            F += [[o0+i, i0+j, o0+j], [o0+i, i0+i, i0+j]]
            F += [[o1+i, o1+j, i1+j], [o1+i, i1+j, i1+i]]
    else:
        c0 = len(V); V.append([x0, 0, 0]); c1 = len(V); V.append([x1, 0, 0])
        for i in range(seg):
            j = (i+1) % seg; F += [[c0, o0+j, o0+i], [c1, o1+i, o1+j]]
    return Mesh(V, F)

# ---------- transforms ----------
def T(x=0, y=0, z=0):
    M = np.eye(4); M[:3, 3] = [x, y, z]; return M
def Rz(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[:2, :2] = [[c, -s], [s, c]]; return M
def Rx(a):
    c, s = math.cos(a), math.sin(a); M = np.eye(4); M[1:3, 1:3] = [[c, -s], [s, c]]; return M
def Raxis(axis, a, point=(0, 0, 0)):
    k = np.asarray(axis, float); k = k/np.linalg.norm(k)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    Rm = np.eye(3) + math.sin(a)*K + (1-math.cos(a))*K@K
    M = np.eye(4); M[:3, :3] = Rm; p = np.asarray(point, float); M[:3, 3] = p - Rm@p
    return M
def frame(origin, ex, ey):
    ex = np.asarray(ex, float); ex = ex/np.linalg.norm(ex)
    ey = np.asarray(ey, float); ey = ey - ex*np.dot(ex, ey); ey = ey/np.linalg.norm(ey)
    M = np.eye(4); M[:3, 0] = ex; M[:3, 1] = ey; M[:3, 2] = np.cross(ex, ey); M[:3, 3] = origin
    return M
def xf(mesh, M):
    V = mesh.V @ M[:3, :3].T + M[:3, 3]
    F = mesh.F if np.linalg.det(M[:3, :3]) > 0 else mesh.F[:, ::-1]
    return Mesh(V, F)

ZX = np.array([[0, 1, 0, 0], [0, 0, 1, 0], [1, 0, 0, 0], [0, 0, 0, 1]], float)   # local x -> world z
def cyl_z(c, r, z0, z1, r_in=0.0, seg=48):
    return xf(cyl_x(r, z0, z1, r_in, seg), T(c[0], c[1], 0) @ ZX)

# ---------- checks / io ----------
def check(mesh):
    E = {}
    for f in mesh.F:
        for a, b in ((f[0], f[1]), (f[1], f[2]), (f[2], f[0])):
            E[(a, b)] = E.get((a, b), 0) + 1
    bad = sum(1 for (a, b), c in E.items() if c != 1 or E.get((b, a), 0) != 1)
    v = mesh.V[mesh.F]
    vol = np.einsum('ij,ij->i', v[:, 0], np.cross(v[:, 1], v[:, 2])).sum()/6
    com = (np.einsum('ij,ij->i', v[:, 0], np.cross(v[:, 1], v[:, 2]))[:, None] * v.sum(1)).sum(0)/(24*vol) if abs(vol) > 1e-9 else v.mean((0, 1))
    return bad, vol, com

def write_stl(path, mesh, name):
    v = mesh.V[mesh.F].astype(np.float32)
    n = np.cross(v[:, 1]-v[:, 0], v[:, 2]-v[:, 0]); l = np.linalg.norm(n, axis=1, keepdims=True); l[l == 0] = 1; n /= l
    rec = np.zeros(len(v), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'] = n; rec['v'] = v
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' ')); f.write(np.uint32(len(v)).tobytes()); f.write(rec.tobytes())

def sample_surface(mesh, density=0.5, rng=None):
    rng = rng or np.random.default_rng(0)
    v = mesh.V[mesh.F]
    ar = 0.5*np.linalg.norm(np.cross(v[:, 1]-v[:, 0], v[:, 2]-v[:, 0]), axis=1)
    nper = rng.poisson(ar*density) + (ar > 0.2).astype(int)
    idx = np.repeat(np.arange(len(v)), nper)
    r1, r2 = rng.random(len(idx)), rng.random(len(idx)); s = np.sqrt(r1)
    p = (1-s)[:, None]*v[idx, 0] + (s*(1-r2))[:, None]*v[idx, 1] + (s*r2)[:, None]*v[idx, 2]
    return np.vstack([p, mesh.V])
