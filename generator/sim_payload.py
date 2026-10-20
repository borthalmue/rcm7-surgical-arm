"""Payload for the RCM-7 simulator: every CAD part + spin metadata for internal moving parts + index markers."""
import numpy as np, math, json, os, base64
os.environ['SKIP_INTERF'] = '1'
import model as Mo
from geom import *
_src = open('build.py').read()
exec(_src.split('# ---------------------------------------------------------------- 6.')[0])      # checks + counterbalance + report
exec(_src[_src.index('def crease'):_src.index('payload = dict(')])

Z3 = np.array([0, 0, 1.0]); D3 = np.array([0, 1.0, 0])
PM = Mo.PM
SCREW_P = (Mo.SF @ np.array([0, -36, -28, 1.0]))[:3]
YAW_P = Mo.R3

# spin spec: joint, ratio (deg of part per unit of joint), point, axis -- all in the body's reference frame
SPIN = {}
def spin(name, j, k, p, a): SPIN[name] = dict(j=j, k=k, p=[float(x) for x in p], a=[float(x) for x in a])
for nm in ('pitch_motor_pulley',): spin(nm, 'theta', 200, [PM[0], PM[1], 0], Z3)
spin('pitch_input_pulley', 'theta', 100, [Mo.O[0], Mo.O[1], 0], Z3)
for nm in ('ballscrew_SFU1204', 'screw_coupling', 'retraction_knob'): spin(nm, 'q', -90, SCREW_P, D3)   # 4 mm lead: 90 deg per mm

MARK = []
def mark(name, body, mesh, j=None, k=None, p=None, a=None):
    MARK.append(dict(name=name, body=body, mesh=mesh, spin=None if j is None else dict(j=j, k=k, p=[float(x) for x in p], a=[float(x) for x in a])))
# rotor indicators (housings stay still, rotors turn at the true gear ratio)
mark('yaw_rotor_stub', 'ground', xf(cyl_x(6, 618, 626, seg=24) + box(622, 626, 0, 15, -1.6, 1.6), Mo.FA), 'yaw', 100, YAW_P, Mo.b3)
mark('pitch_rotor_stub', 'yaw', cyl_z(PM, 5, -146, -141, seg=24) + xf(box(0, 19, -1.6, 1.6, -147, -145), T(PM[0], PM[1], 0)), 'theta', 200, [PM[0], PM[1], 0], Z3)
mark('pitch_motor_pulley_mark', 'yaw', xf(box(3, 8.6, -1.1, 1.1, -83, -82), T(PM[0], PM[1], 0)), 'theta', 200, [PM[0], PM[1], 0], Z3)
mark('pitch_input_pulley_mark', 'yaw', xf(box(10, 17.4, -1.4, 1.4, -83, -82), T(Mo.O[0], Mo.O[1], 0)), 'theta', 100, [Mo.O[0], Mo.O[1], 0], Z3)
mark('screw_coupling_mark', 'spar', xf(box(224, 232, -37.2, -34.8, -28, -17.4), Mo.SF), 'q', -90, SCREW_P, D3)
mark('knob_mark', 'spar', xf(box(295, 296.4, -37.4, -34.6, -28, -14), Mo.SF), 'q', -90, SCREW_P, D3)
names = ['roll', 'psi', 'chi', 'grip']
for i, (xx, yy) in enumerate(((20, 12), (20, -12), (60, 12), (60, -12))):
    p = (Mo.SF @ np.array([Mo.XT+xx, yy, 0, 1]))[:2]
    mark(f'isa_rotor_{i+1}', 'carriage', cyl_z(p, 3, Mo.ZI-74, Mo.ZI-70, seg=16) + xf(box(0, 7.5, -1, 1, Mo.ZI-75, Mo.ZI-74), T(p[0], p[1], 0)),
         names[i], 30, [p[0], p[1], 0], Z3)
# pulley index marks (show which pulleys turn with which body)
mark('pulley_1a_mark', 'yaw', xf(box(9.6, 11.8, -1.2, 1.2, 48.5, 49.2), T(Mo.O[0], Mo.O[1], 0)))
mark('pulley_1b_mark', 'coupler1', xf(box(7, 11.6, -1.2, 1.2, 51, 51.7), T(Mo.A[0], Mo.A[1], 0)))
mark('pulley_2a_mark', 'link1', xf(box(6.4, 11.6, -1.2, 1.2, 25.5, 26.2), T(Mo.A[0], Mo.A[1], 0)))
mark('pulley_2b_mark', 'spar', xf(box(6.4, 11.6, -1.2, 1.2, 25.5, 26.2), T(Mo.C[0], Mo.C[1], 0)))

def pack(mesh):
    P, N = crease(mesh)
    return base64.b64encode(P.tobytes()).decode(), base64.b64encode(N.tobytes()).decode()

parts = []
for c in Mo.COMP:
    p, nn = pack(c['mesh'])
    parts.append(dict(name=c['name'], body=c['body'], mat=c['mat'], part=c['part'], desc=c['desc'],
                      kg=round(float(c['kg']), 5), com=[round(float(x), 3) for x in c['com']], spin=SPIN.get(c['name']), p=p, n=nn))
for mk in MARK:
    p, nn = pack(mk['mesh'])
    parts.append(dict(name=mk['name'], body=mk['body'], mat='mark', part='', desc='index mark', kg=0, com=[0, 0, 0], spin=mk['spin'], p=p, n=nn))

# belts: carrier body, frame (origin, ex, ey), pulley centre distance, pitch radius, z band, surface travel per unit joint
belts = [dict(name='belt_1', body='link1', o=[Mo.O[0], Mo.O[1], 0], ex=[0, 1, 0], ey=[-1, 0, 0], L=Mo.L1, r=10.35, z=[42.5, 47.5], j='theta', k=-9.55*math.pi/180),
         dict(name='belt_2', body='coupler1', o=[Mo.A[0], Mo.A[1], 0], ex=[Mo.b[0], Mo.b[1], 0], ey=[Mo.nh[0], Mo.nh[1], 0], L=Mo.n, r=10.35, z=[15.5, 24.5], j='theta', k=9.55*math.pi/180)]
rows = []
import csv
with open('out/BOM.csv') as f: rows = list(csv.DictReader(f))
for r in rows: r['qty'] = int(r['qty']); r['est_mass_g'] = float(r['est_mass_g'])
report['bom'] = dict(line_items=len(rows), components=len(Mo.COMP), total_mass_kg=round(sum(c['kg'] for c in Mo.COMP if c['body'] != 'ground'), 2))
json.dump(report, open('out/report.json', 'w'), indent=1)
meta = dict(L1=Mo.L1, M=Mo.M, m=Mo.m, n=Mo.n, beta=20.0, ZI=Mo.ZI, Q0=Mo.Q0, O=Mo.O.tolist(), Op=Mo.Op.tolist(),
            W0=Mo.W0.tolist(), J0=Mo.J0.tolist(), pitch=Mo.PITCH_LIM, yaw=Mo.YAW_LIM, q=Mo.Q_LIM, wristQmin=Mo.WRIST_Q_MIN,
            G=Mo.Gs.tolist(), K=Kp_ref.tolist(), KS=KS, report=report, bom=rows, belts=belts)
json.dump(dict(parts=parts, meta=meta), open('out/sim_payload.json', 'w'), separators=(',', ':'))
print('parts', len(parts), 'spin', sum(1 for p in parts if p['spin']), 'MB', round(os.path.getsize('out/sim_payload.json')/1e6, 2))
