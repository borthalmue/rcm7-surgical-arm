"""Build, verify and export the RCM-7 arm.  Run: python3 build.py"""
import numpy as np, math, json, os, csv, base64, itertools, time
from scipy.spatial import cKDTree
import model as Mo
from geom import *

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'out')
os.makedirs(OUT + '/stl', exist_ok=True)
D = math.radians

# ---------------------------------------------------------------- kinematics (mirrored in the viewer)
def body_T(pose):
    th, yaw = D(pose['theta']), D(pose['yaw'])
    d = np.array([-math.sin(th), math.cos(th)])
    Y = Raxis(Mo.b3, yaw, Mo.R3)
    rz = lambda p, a: Raxis((0, 0, 1), a, (p[0], p[1], 0))
    Tb = {'ground': np.eye(4), 'yaw': Y}
    Tb['link1'] = Y @ rz(Mo.O, th)
    Tb['link_opap'] = Y @ rz(Mo.Op, th)
    dd = d - Mo.d0
    Tb['coupler1'] = Y @ T(*(Mo.L1*dd), 0)
    Tb['coupler2'] = Y @ T(*((Mo.L1+Mo.M)*dd), 0)
    Tb['spar'] = Y @ rz((0, 0), th)
    Tb['carriage'] = Tb['spar'] @ T(0, -(pose['q'] - Mo.Q0), 0)
    Tb['instrument'] = Tb['carriage'] @ Raxis((0, -1, 0), D(pose['roll']), Mo.R3)
    Tb['wrist'] = Tb['instrument'] @ Raxis((0, 0, 1), D(pose['psi']), Mo.W0)
    Tb['jaw1'] = Tb['wrist'] @ Raxis((1, 0, 0), D(pose['chi'] - pose['grip']/2), Mo.J0)
    Tb['jaw2'] = Tb['wrist'] @ Raxis((1, 0, 0), D(pose['chi'] + pose['grip']/2), Mo.J0)
    return Tb

REF = dict(yaw=0, theta=0, q=Mo.Q0, roll=0, psi=0, chi=0, grip=0)
def pt(M, p): return (M @ np.r_[p, 1.0])[:3]

report = {}

# ---------------------------------------------------------------- 1. mesh validity + mass properties
t0 = time.time()
mass = {}
for c in Mo.COMP:
    bad, vol, com = check(c['mesh'])
    c['bad'], c['vol'], c['com'] = bad, vol, com
    c['kg'] = vol*Mo.DENS[c['mat']]
bad_parts = [c['name'] for c in Mo.COMP if c['bad'] or c['vol'] <= 0]
report['mesh'] = dict(components=len(Mo.COMP), triangles=int(sum(len(c['mesh'].F) for c in Mo.COMP)),
                      non_manifold_or_inverted=bad_parts)
print('mesh check:', len(Mo.COMP), 'components, bad:', bad_parts, f'{time.time()-t0:.1f}s')

# ---------------------------------------------------------------- 2. counterbalance: aim spring at the first-moment vector
centers = {'link1': Mo.O, 'link_opap': Mo.Op, 'spar': np.zeros(2), 'carriage': np.zeros(2), 'instrument': np.zeros(2),
           'wrist': np.zeros(2), 'jaw1': np.zeros(2), 'jaw2': np.zeros(2)}
S = np.zeros(2); m_pitch = 0
for c in Mo.COMP:
    bdy = c['body']
    if bdy in centers: S += c['kg']*(c['com'][:2] - centers[bdy]); m_pitch += c['kg']
    elif bdy == 'coupler1': S += c['kg']*np.array([0, Mo.L1]); m_pitch += c['kg']
    elif bdy == 'coupler2': S += c['kg']*np.array([0, Mo.L1+Mo.M]); m_pitch += c['kg']
k_r, g_r = 100.0, 150.0
ang = math.atan2(S[0]*-1, S[1])          # angle of S from d0 toward rot90(d0) = (-1,0)
K_local = np.array([k_r*math.cos(ang), k_r*math.sin(ang)])
assert abs(K_local[1]) < 15, K_local
for i, c in enumerate(Mo.COMP):
    if c['name'] == 'spring_stud_K':
        st, Kp = Mo.spring_stud(K_local); c['mesh'] = st; c['bad'], c['vol'], c['com'] = check(st); c['kg'] = c['vol']*Mo.DENS['steel']
KS = 9.81*np.linalg.norm(S)/(g_r*k_r)          # N/mm, zero-free-length equivalent
Kp_ref = Mo.O + K_local[0]*Mo.d0 + K_local[1]*np.array([-1, 0])
report['counterbalance'] = dict(first_moment_kg_mm=S.round(1).tolist(), moving_mass_kg=round(m_pitch, 3),
                                K_local_mm=K_local.round(2).tolist(), spring_rate_N_per_mm=round(KS, 3),
                                note='Zero-free-length equivalent (spring + cable over pulley). Balanced at q = %g mm, yaw 0.' % Mo.Q0)
print('first moment', S.round(1), 'mass', round(m_pitch, 2), 'kg; spring', round(KS, 3), 'N/mm, K', K_local.round(1))

# ---------------------------------------------------------------- 3. RCM verification (transform model vs. closure solve)
def closure_drift(theta):
    """Independent loop-closure solve of both parallelograms from link lengths only."""
    th = D(theta); d = np.array([-math.sin(th), math.cos(th)])
    A = Mo.O + Mo.L1*d; E = Mo.O + (Mo.L1+Mo.M)*d
    def circ(p0, r0, p1, r1, near):
        v = p1-p0; dd = np.linalg.norm(v); a = (r0*r0-r1*r1+dd*dd)/(2*dd); h = math.sqrt(max(0, r0*r0-a*a))
        pm = p0 + a*v/dd; pp = np.array([-v[1], v[0]])/dd
        i1, i2 = pm+h*pp, pm-h*pp
        return i1 if np.linalg.norm(i1-near) < np.linalg.norm(i2-near) else i2
    Ap = circ(Mo.Op, Mo.L1, A, Mo.m, Mo.Op + Mo.L1*d)
    C = A + Mo.n*(Ap-A)/np.linalg.norm(Ap-A)
    Fp = circ(C, Mo.M, E, Mo.n, C + Mo.M*d)
    u = (C-Fp)/np.linalg.norm(C-Fp)
    return abs(-C[0]*u[1] + C[1]*u[0])
rng = np.random.default_rng(1); worst = 0; worst_cl = 0
for _ in range(4000):
    pose = dict(yaw=rng.uniform(*Mo.YAW_LIM), theta=rng.uniform(*Mo.PITCH_LIM), q=rng.uniform(*Mo.Q_LIM),
                roll=rng.uniform(-260, 260), psi=rng.uniform(-80, 80), chi=rng.uniform(-80, 80), grip=rng.uniform(0, 60))
    Tb = body_T(pose)
    p1, p2 = pt(Tb['instrument'], [0, 200, Mo.ZI]), pt(Tb['instrument'], [0, -100, Mo.ZI])
    u = (p2-p1)/np.linalg.norm(p2-p1)
    worst = max(worst, np.linalg.norm(np.cross(Mo.R3-p1, u)))
    worst_cl = max(worst_cl, closure_drift(pose['theta']))
report['rcm'] = dict(poses=4000, max_shaft_to_R_mm=float(worst), closure_solver_max_drift_mm=float(worst_cl))
print('RCM worst distance (3D, 4000 poses):', worst, ' closure solver:', worst_cl)

# ---------------------------------------------------------------- 4. gravity, brakes, actuator loads
GV = np.array([0, -9.81, 0])
def potential(pose, spring=True):
    Tb = body_T(pose); U = 0.0
    for c in Mo.COMP:
        if c['body'] == 'ground': continue
        U -= c['kg']*np.dot(GV, pt(Tb[c['body']], c['com']))*1e-3      # J
    if spring:
        G = pt(Tb['yaw'], [Mo.Gs[0], Mo.Gs[1], 59]); K = pt(Tb['link1'], [Kp_ref[0], Kp_ref[1], 59])
        U += 0.5*KS*np.sum((G-K)**2)*1e-3
    return U
def torque(pose, key, spring=True, h=0.05):
    p1, p2 = dict(pose), dict(pose); p1[key] -= h; p2[key] += h
    return -(potential(p2, spring)-potential(p1, spring))/D(2*h)       # N m
grid = []
for yaw in (0, 30, 60, 75):
    for th in np.linspace(*Mo.PITCH_LIM, 11):
        for q in (Mo.Q_LIM[0], Mo.Q0, Mo.Q_LIM[1]):
            p = dict(REF, yaw=yaw, theta=th, q=q)
            grid.append((yaw, th, q, torque(p, 'theta', False), torque(p, 'theta', True), torque(p, 'yaw', True)))
g = np.array(grid)
# rescale the spring so the worst residual over the whole yaw range is minimised (gravity in the arm plane ~ cos(yaw))
best = None
for lam in np.linspace(0.3, 1.1, 33):
    KS_try = KS*lam; res = 0
    for (yaw, th, q, tg, ts, ty) in grid:
        ts_l = tg + lam*(ts - tg)            # spring torque is linear in KS
        res = max(res, abs(ts_l))
    if best is None or res < best[1]: best = (lam, res)
lam = best[0]; KS = KS*lam
g[:, 4] = g[:, 3] + lam*(g[:, 4] - g[:, 3])
grid2 = []
for (yaw, th, q, tg, ts, ty) in grid:
    p = dict(REF, yaw=yaw, theta=th, q=q); grid2.append(torque(p, 'yaw', True))
g[:, 5] = np.array(grid2)
report['counterbalance']['spring_rate_N_per_mm'] = round(KS, 3)
report['counterbalance']['note'] = ('Zero-free-length equivalent (spring + cable over pulley). Rate chosen to minimise the worst '
                                    'residual pitch torque over yaw -75..75 deg (gravity in the arm plane scales ~cos(yaw)).')
report['loads'] = dict(
    pitch_gravity_no_spring_max_Nm=round(float(np.abs(g[:, 3]).max()), 2),
    pitch_with_spring_max_Nm=round(float(np.abs(g[:, 4]).max()), 2),
    pitch_with_spring_yaw0_max_Nm=round(float(np.abs(g[g[:, 0] == 0][:, 4]).max()), 2),
    pitch_with_spring_yaw75_max_Nm=round(float(np.abs(g[g[:, 0] == 75][:, 4]).max()), 2),
    yaw_gravity_max_Nm=round(float(np.abs(g[:, 5]).max()), 2),
    insertion_gravity_max_N=round(float(9.81*sum(c['kg'] for c in Mo.COMP if c['body'] in ('carriage', 'instrument', 'wrist', 'jaw1', 'jaw2'))), 2),
    yaw_hold_by_brake_Nm=70.0, pitch_hold_by_brake_Nm=140.0, yaw_CSF17_rated_Nm=24.0, yaw_CSF17_repeated_peak_Nm=54.0, pitch_SHG17_rated_Nm=31.0, pitch_SHG17_repeated_peak_Nm=70.0,
    insertion_hold_by_brake_N=round(2*math.pi*0.4/0.004*0.9, 0))
print('loads', report['loads'])

# ---------------------------------------------------------------- 5. interference sweep
ALLOW = [('pin_', 'brg_'), ('pulley_', 'belt_'), ('enc_ring', 'readhead'), ('xrb_inner', 'xrb_outer'),
         ('yaw_hd_output', 'yaw_harmonic'), ('mgn12h_block', 'rail_MGN12'), ('ball_nut', 'ballscrew'),
         ('instrument_shaft', 'cannula'), ('instrument_shaft', 'isa_housing'), ('instrument_roll_hub', 'isa_housing'),
         ('instrument_shaft', 'carriage_plate'), ('wrist_', 'instrument_shaft'), ('jaw_', 'wrist_'), ('jaw_1', 'jaw_2'),
         ('mgn12h_block', 'insertion_stop'), ('sleeve_A', 'pin_A'), ('sleeve_A', 'brg_L1_A'), ('pulley_1b', 'pin_A'), ('pitch_ground_shaft', 'brg_L1_O'), ('pitch_ground_shaft', 'link1_hub'), ('pitch_ground_shaft', 'link1_plate'), ('pulley_1a', 'link1_plate_front'), ('instrument_roll_hub', 'carriage_plate'), ('cannula', 'wrist_'), ('cannula', 'jaw_'), ('yaw_stop_dog', 'yaw_hardstop')]
def allowed(a, b):
    return any((x in a and y in b) or (x in b and y in a) for x, y in ALLOW)
t0 = time.time()
SKIP = os.environ.get('SKIP_INTERF') == '1'
prev = json.load(open(OUT + '/report.json')) if SKIP and os.path.exists(OUT + '/report.json') else None
for c in ([] if SKIP else Mo.COMP):
    c['pts'] = sample_surface(c['mesh'], density=0.6)
bodies = sorted(set(c['body'] for c in Mo.COMP))
poses = [dict(REF, theta=th, q=q, yaw=yaw, psi=ps if q >= Mo.WRIST_Q_MIN else 0, grip=gr if q >= Mo.WRIST_Q_MIN else 0)
         for th in np.linspace(*Mo.PITCH_LIM, 11)
         for q in (Mo.Q_LIM[0], Mo.WRIST_Q_MIN, 120, Mo.Q_LIM[1]) for yaw, ps, gr in ((0, 0, 0), (75, 60, 40), (-75, -80, 60))]
closest = {}
for pose in ([] if SKIP else poses):
    Tb = body_T(pose)
    W = []
    for c in Mo.COMP:
        P = c['pts'] @ Tb[c['body']][:3, :3].T + Tb[c['body']][:3, 3]
        W.append((c, P, P.min(0), P.max(0)))
    for (ca, Pa, la, ha), (cb, Pb, lb, hb) in itertools.combinations(W, 2):
        if ca['body'] == cb['body'] or allowed(ca['name'], cb['name']): continue
        if np.any(la > hb + 3) or np.any(lb > ha + 3): continue
        dist, _ = cKDTree(Pb).query(Pa, k=1, distance_upper_bound=3.0)
        dm = float(dist.min())
        if dm < 3.0:
            key = tuple(sorted((ca['name'], cb['name'])))
            if key not in closest or dm < closest[key][0]:
                closest[key] = (dm, {k: round(float(v), 1) for k, v in pose.items()})
clash = {f'{a} | {b}': v for (a, b), v in sorted(closest.items(), key=lambda kv: kv[1][0]) if v[0] < 0.9}
near = {f'{a} | {b}': v for (a, b), v in sorted(closest.items(), key=lambda kv: kv[1][0]) if 0.9 <= v[0] < 3.0}
report['interference'] = prev['interference'] if SKIP else dict(poses=len(poses), clashes_under_0p9mm=clash, near_under_3mm=near)
print(f'interference: {len(clash)} clashes, {len(near)} near-misses  ({time.time()-t0:.0f}s)')
for k, v in list(clash.items())[:40]: print('  CLASH', k, v)
for k, v in list(near.items())[:40]: print('  near ', k, v)
json.dump(report, open(OUT + '/report.json', 'w'), indent=1)

# ---------------------------------------------------------------- 6. exports
import zipfile
for f in os.listdir(OUT + '/stl'): os.remove(OUT + '/stl/' + f)
asm = Mesh()
for c in Mo.COMP:
    write_stl(f"{OUT}/stl/{c['name']}.stl", c['mesh'], c['name']); asm = asm + c['mesh']
write_stl(OUT + '/RCM7_assembly_reference_pose.stl', asm, 'RCM7 assembly')

# BOM grouped by part number
SRC = {'THK': 'THK', 'HD ': 'Harmonic Drive', 'maxon': 'maxon', 'Kendrion': 'Kendrion', 'RLS': 'RLS', 'SKF': 'SKF',
       'HIWIN': 'HIWIN', 'TBI': 'TBI Motion', 'MISUMI': 'MISUMI', 'Gates': 'Gates', 'AT3': 'Gates / MISUMI', 'BF10': 'TBI / generic',
       'BK10': 'TBI / generic', 'Ruland': 'Ruland', 'McMaster': 'McMaster-Carr', '316 SS': 'tube stock', '304 SS': 'tube stock'}
groups = {}
for c in Mo.COMP:
    g = groups.setdefault(c['part'], dict(part=c['part'], desc=c['desc'], qty=0, mass=0.0, names=[]))
    g['qty'] += 1; g['mass'] += c['kg']; g['names'].append(c['name'])
rows = []
for g in groups.values():
    kind = 'machined' if g['part'].startswith('MACH') else 'catalog'
    vendor = next((v for k, v in SRC.items() if g['part'].startswith(k) or k in g['part']), 'in-house' if kind == 'machined' else '')
    rows.append(dict(part=g['part'], type=kind, vendor=vendor, qty=g['qty'], description=g['desc'], est_mass_g=round(1000*g['mass']/g['qty'], 1),
                     files=';'.join(g['names'])))
rows.sort(key=lambda r: (r['type'] != 'catalog', r['part']))
with open(OUT + '/BOM.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
report['bom'] = dict(line_items=len(rows), components=len(Mo.COMP), total_mass_kg=round(sum(c['kg'] for c in Mo.COMP if c['body'] != 'ground'), 2))
json.dump(report, open(OUT + '/report.json', 'w'), indent=1)

# viewer payload: crease-angle normals, float32 positions, int8 normals
def crease(mesh, ang=35):
    V, F = mesh.V, mesh.F
    fn = np.cross(V[F[:, 1]]-V[F[:, 0]], V[F[:, 2]]-V[F[:, 0]]); fa = np.linalg.norm(fn, axis=1, keepdims=True); fa[fa == 0] = 1; fnu = fn/fa
    inc = [[] for _ in range(len(V))]
    for fi, f in enumerate(F):
        for v in f: inc[v].append(fi)
    cosT = math.cos(math.radians(ang)); N = np.zeros((len(F), 3, 3))
    for fi, f in enumerate(F):
        for k, v in enumerate(f):
            fs = inc[v]; s = np.zeros(3)
            for g in fs:
                if np.dot(fnu[g], fnu[fi]) >= cosT: s += fn[g]
            N[fi, k] = s/(np.linalg.norm(s) or 1)
    return V[F].reshape(-1, 3).astype(np.float32), np.clip(np.round(N.reshape(-1, 3)*127), -127, 127).astype(np.int8)
payload = dict(parts=[], meta=dict(L1=Mo.L1, M=Mo.M, m=Mo.m, n=Mo.n, beta=20.0, ZI=Mo.ZI, Q0=Mo.Q0,
               O=Mo.O.tolist(), Op=Mo.Op.tolist(), W0=Mo.W0.tolist(), J0=Mo.J0.tolist(),
               pitch=Mo.PITCH_LIM, yaw=Mo.YAW_LIM, q=Mo.Q_LIM, wristQmin=Mo.WRIST_Q_MIN,
               G=Mo.Gs.tolist(), K=Kp_ref.tolist(), KS=KS, report=report, bom=rows))
for c in Mo.COMP:
    P, N = crease(c['mesh'])
    payload['parts'].append(dict(name=c['name'], body=c['body'], mat=c['mat'], part=c['part'], desc=c['desc'],
                                 kg=round(float(c['kg']), 5), com=[round(float(x), 3) for x in c['com']],
                                 p=base64.b64encode(P.tobytes()).decode(), n=base64.b64encode(N.tobytes()).decode()))
json.dump(payload, open(OUT + '/viewer_payload.json', 'w'), separators=(',', ':'))
print('payload MB', round(os.path.getsize(OUT + '/viewer_payload.json')/1e6, 2), 'tris', sum(len(c['mesh'].F) for c in Mo.COMP))
