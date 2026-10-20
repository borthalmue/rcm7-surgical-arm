"""
RCM-7 : fault-tolerant spherical remote-center-of-motion arm.

Kinematic core (unchanged from the simulator): double parallelogram, |OA|=|O'A'|=300, |AE|=|CF|=250,
|OO'|=130, |AC|=|EF|=420, baseline tilted 20 deg. New: outer yaw axis along the baseline THROUGH R,
instrument roll about the shaft (also through R), 2-DOF wrist + grip.  R = (0, 0, ZI).

Frames. Arm frame = world at yaw 0: x right, y up, z out of the arm plane. All geometry is built at the
reference pose theta=0, q=Q0, yaw=roll=wrist=0, jaws closed. Body transforms (also used by the viewer):
  ground      I
  yaw         Raxis(bh3, yaw, R3)
  link1       yaw . Rz(theta) about O
  link_opap   yaw . Rz(theta) about O'
  coupler1    yaw . T(L1 (d - d0))
  coupler2    yaw . T((L1+M)(d - d0))
  spar        yaw . Rz(theta) about R
  carriage    spar . T(u0 (q - Q0))
  instrument  carriage . Raxis(u0, roll, R3)
  wrist       instrument . Raxis(z, psi, W0)
  jaw1/jaw2   wrist . Raxis(x, chi +/- grip/2, J0)
"""
import numpy as np, math
from geom import *

L1, M, m, n = 300.0, 250.0, 130.0, 420.0
BETA = math.radians(20.0)
ZI = -60.0            # instrument axis / yaw axis plane (behind the link layers)
Q0 = 100.0            # reference insertion (wrist axis W below R)
PITCH_LIM = (-50.0, 44.0)       # +44: parallel links keep >= 13 mm gap before the parallelogram flattens
WRIST_Q_MIN = 55.0               # interlock: wrist/grip locked until jaws clear the cannula tip
YAW_LIM = (-75.0, 75.0)
Q_LIM = (15.0, 180.0)

b = np.array([math.cos(BETA), -math.sin(BETA)])
nh = np.array([math.sin(BETA), math.cos(BETA)])
O = -n*b; Op = O + m*b
d0 = np.array([0.0, 1.0]); u0 = -d0
A = O + L1*d0; E = O + (L1+M)*d0; Ap = Op + L1*d0
C = np.array([0.0, L1]); F = np.array([0.0, L1+M])
R3 = np.array([0, 0, ZI]); b3 = np.array([b[0], b[1], 0.0]); nh3 = np.array([nh[0], nh[1], 0.0])
W0 = np.array([0, -Q0, ZI]); J0 = W0 + 9*np.array([0, -1, 0])

def rot90(v): return np.array([-v[1], v[0]])
def rot2(v, a): return np.array([v[0]*math.cos(a)-v[1]*math.sin(a), v[0]*math.sin(a)+v[1]*math.cos(a)])
def P3(p2, z=0.0): return np.array([p2[0], p2[1], z])
def f2(origin2, ex2):
    """planar frame: local x along ex2, local y = rot90(ex2), local z = world z"""
    return frame(P3(origin2), P3(ex2), P3(rot90(ex2)))
FA = frame(R3, -b3, nh3)               # yaw-axis frame: local x = s (distance upstream from R), y = n, z = w
def ax_pt(s, nn=0.0, w=0.0): return (FA @ np.array([s, nn, w, 1.0]))[:3]

def cyl_between(p0, p1, r, r_in=0.0, seg=48):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); v = p1-p0; L = np.linalg.norm(v)
    t = np.array([0, 0, 1.0]) if abs(v[2]) < 0.9*L else np.array([1.0, 0, 0])
    return xf(cyl_x(r, 0, L, r_in, seg), frame(p0, v, np.cross(np.cross(v, t), v)))

def annulus_sector(r0, r1, a0, a1, z0, z1, seg=40):
    a = np.linspace(a0, a1, seg)
    pts = np.vstack([np.stack([r1*np.cos(a), r1*np.sin(a)], 1), np.stack([r0*np.cos(a[::-1]), r0*np.sin(a[::-1])], 1)])
    return extrude(pts, [], z0, z1)

COMP = []
def add(name, body, mat, mesh, part, desc=''):
    COMP.append(dict(name=name, body=body, mat=mat, mesh=mesh, part=part, desc=desc))

# ------------------------------------------------------------------ ground (setup-arm side, fixed)
add('yaw_housing', 'ground', 'alu2', xf(cyl_x(52, 492, 612, r_in=40, seg=64), FA), 'MACH-101', 'Yaw housing, 6061')
add('yaw_end_cap', 'ground', 'alu2', xf(cyl_x(52, 612, 618, seg=64), FA), 'MACH-102', 'Yaw end cap')
add('yaw_xrb_outer', 'ground', 'steel', xf(cyl_x(40, 495, 508, r_in=32.6, seg=64), FA), 'THK RB5013', 'Yaw crossed-roller bearing 50x80x13 (outer ring)')
add('yaw_harmonic', 'ground', 'black', xf(cyl_x(39.5, 512, 557, seg=64), FA), 'HD CSF-17-100-2UH', 'Yaw strain-wave gear unit, 100:1, OD79 x 45')
add('yaw_motor', 'ground', 'black', xf(cyl_x(21.4, 557, 579), FA), 'maxon EC 45 flat 50W', 'Yaw motor')
add('yaw_motor_enc', 'ground', 'black', xf(cyl_x(21.4, 579, 588), FA), 'maxon MILE 1024', 'Yaw motor-side encoder (channel A)')
add('yaw_brake', 'ground', 'red', xf(cyl_x(25.5, 588, 608), FA), 'Kendrion Servo Slim Line 05', 'Yaw spring-applied brake, 0.7 Nm (70 Nm at joint)')
rh =annulus_sector(18, 37, math.radians(-65), math.radians(65), 491.2, 494.0)        # built along local z -> map z to s
add('yaw_readhead', 'ground', 'pcb', xf(rh, FA @ np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]], float)),
    'RLS AksIM-2 MB053', 'Yaw joint-side absolute encoder readhead (channel B)')
for i, a in enumerate((math.radians(89.2), math.radians(-89.2))):
    blk = xf(box(489, 492, -5, 5, 42, 51), Rx(a)); add(f'yaw_hardstop_{i+1}', 'ground', 'stop', xf(blk, FA), 'MACH-103', 'Yaw hard stop on housing face, +/-75 deg')
add('setup_saddle', 'ground', 'alu2', xf(box(530, 585, -30, 30, 46, 72), FA), 'MACH-104', 'Setup-arm clamp saddle')
add('setup_beam', 'ground', 'alu2', xf(box(535, 580, -24, 24, 72, 260), FA), 'MACH-105', 'Setup-arm beam (to cart / bed rail)')
p_beam = ax_pt(557, 0, 240)
add('setup_column', 'ground', 'alu2', cyl_between(p_beam, p_beam + np.array([0, -420, 0]), 34, seg=64), 'MACH-106', 'Setup column (down to cart / bed-rail clamp)')

# ------------------------------------------------------------------ yaw body (rotates about axis through R)
add('yaw_output_shaft', 'yaw', 'steel', xf(cyl_x(15, 488, 495), FA), 'MACH-201', 'Yaw output shaft')
add('yaw_xrb_inner', 'yaw', 'steel', xf(cyl_x(32.4, 495, 508, r_in=25), FA), 'THK RB5013', 'Yaw crossed-roller bearing (inner ring)')
add('yaw_hd_output', 'yaw', 'steel', xf(cyl_x(30, 508, 512), FA), 'HD CSF-17-100-2UH', 'Gear unit output flange')
add('yaw_enc_ring', 'yaw', 'pcb', xf(cyl_x(26.5, 489, 491, r_in=15), FA), 'RLS MRA053BC030', 'Yaw encoder ring 30x53x2')
add('yaw_stop_dog', 'yaw', 'stop', xf(box(488, 491, -5, 5, 42, 51), FA), 'MACH-202', 'Yaw stop dog')
add('yoke', 'yaw', 'alu', xf(box(478, 488, -50, 50, ZI+4, 52), FA), 'MACH-203', 'Yaw yoke, 10 mm 6061')
SWN = frame(R3, -b3, np.array([0, 0, -1.0]))       # local (s, w, n)
for sgn in (1, -1):
    g = extrude(np.array([[478, ZI+16], [478, 52], [430, ZI+16]]), [], 44 if sgn > 0 else -50, 50 if sgn > 0 else -44)
    add(f'yoke_gusset_{"L" if sgn>0 else "R"}', 'yaw', 'alu', xf(g, SWN), 'MACH-204', 'Yoke gusset, 6 mm')
PF = f2(O, b)
pul = cyl_z((0, 0), 9.55, 42.5, 47.5, r_in=6, seg=40) + cyl_z((0, 0), 12, 41.5, 42.5, r_in=6, seg=40) + cyl_z((0, 0), 12, 47.5, 48.5, r_in=6, seg=40)
plate = extrude(stadium(-15, 125, 50, 64), [circle((0, 0), 16, 48), circle((m, 0), 6, 32)] +
                [circle((x, y), 3.3, 16) for x, y in ((70, 38), (70, -38), (160, 30), (160, -30))], -16, -4)
add('pitch_frame', 'yaw', 'alu', xf(plate, PF), 'MACH-205', 'Pitch frame, 12 mm 6061. Carries O and O\'.')
add('pitch_harmonic', 'yaw', 'black', cyl_z(O, 42, -72.5, -16, r_in=9.5, seg=64), 'HD SHG-17-100-2UH', 'Pitch hollow-shaft strain-wave unit, 100:1, OD84 x 56.5, 19 mm bore')
PM = O + 75*b                                     # offset pitch motor (belt drive to hollow input)
from scipy.spatial import ConvexHull
def belt_loop(c0, r0, c1, r1, z0, z1, t=1.6):
    def hull(dr):
        p = np.vstack([circle(c0, r0+dr, 64), circle(c1, r1+dr, 64)]); h = ConvexHull(p); return p[h.vertices]
    return extrude(hull(t), [hull(0)], z0, z1)
add('pitch_input_pulley', 'yaw', 'alu2', cyl_z(O, 18, -82, -74, r_in=9.5), 'HTD 3M 40T pulley', 'Wave-generator input pulley (hollow)')
add('pitch_motor_pulley', 'yaw', 'alu2', cyl_z(PM, 9, -82, -74), 'HTD 3M 20T pulley', 'Motor pulley, 2:1 pre-reduction')
add('pitch_input_belt', 'yaw', 'belt', belt_loop(O, 18, PM, 9, -81, -75), 'Gates HTD 3M-6 belt', 'Input belt (inside guard)')
for k, a in enumerate((math.radians(140), math.radians(220))):
    p = O + 36*rot2(b, a)
    add(f'pitch_standoff_{k+1}', 'yaw', 'steel', cyl_z(p, 4, -82, -72.5, seg=16), 'M6 standoff', 'Rear-bracket standoff')
add('pitch_rear_bracket', 'yaw', 'alu', xf(extrude(stadium(0, 75, 32, 48), [], -90, -82), PF), 'MACH-209', 'Rear bracket: anchors stationary shaft, carries motor')
add('pitch_motor', 'yaw', 'black', cyl_z(PM, 21.4, -112, -90), 'maxon EC 45 flat 50W', 'Pitch motor')
add('pitch_motor_enc', 'yaw', 'black', cyl_z(PM, 21.4, -121, -112), 'maxon MILE 1024', 'Pitch motor-side encoder (channel A)')
add('pitch_brake', 'yaw', 'red', cyl_z(PM, 25.5, -141, -121), 'Kendrion Servo Slim Line 05', 'Pitch spring-applied brake, 0.7 Nm x 200 = 140 Nm at joint')
add('pitch_brake_release', 'yaw', 'stop', xf(box(20, 52, -4, 4, -137, -129), T(PM[0], PM[1], 0)), 'Kendrion manual release', 'Manual brake-release lever')
add('pitch_ground_shaft', 'yaw', 'steel', cyl_z(O, 6, -90, 49, seg=32) + cyl_z(O, 9, 49, 52, seg=6), 'MACH-210', 'Stationary shaft through hollow gear: carries belt-1 ground pulley')
add('pulley_1a_O', 'yaw', 'alu2', xf(pul, T(O[0], O[1], 0)), 'AT3 20T pulley 10 mm', 'Belt-1 pulley, keyed to stationary shaft (ground side)')
add('pitch_readhead', 'yaw', 'pcb', xf(annulus_sector(17, 29.5, math.radians(110+180-95), math.radians(110+180+95), -6.0, -3.2), PF), 'RLS AksIM-2 MB049', 'Pitch joint-side absolute encoder readhead (channel B)')
pin_o = cyl_z(Op, 6, -16, 40, seg=32) + cyl_z(Op, 9, 40, 43, seg=32) + cyl_z(Op, 9, -19, -16, seg=6)
add("pin_Op", 'yaw', 'steel', pin_o, 'MISUMI CBRS12 (mod.)', "Shoulder pin O', fixed in pitch frame")
post = O + (-30)*b + (-38)*nh; Gs = O + np.array([0, 150.0])
add('spring_post', 'yaw', 'steel', cyl_z(post, 6, -4, 68, seg=32), 'MACH-206', 'Counterbalance post')
vv = Gs - post; arm = extrude(stadium(0, np.linalg.norm(vv), 8, 32), [], 62, 68)
add('spring_arm', 'yaw', 'alu', xf(arm, f2(post, vv/np.linalg.norm(vv))), 'MACH-207', 'Counterbalance arm')
add('spring_anchor_G', 'yaw', 'steel', cyl_z(Gs, 4, 56, 62, seg=24), 'MACH-208', 'Spring anchor G (above O)')
def dvec(th): return np.array([-math.sin(math.radians(th)), math.cos(math.radians(th))])
for i, ths in enumerate((PITCH_LIM[1]+2.0, PITCH_LIM[0]-2.0)):
    a = math.radians(90 + ths) + (1 if ths > 0 else -1)*math.asin(27/40)
    c = O + 40*np.array([math.cos(a), math.sin(a)])
    add(f'pitch_hardstop_{i+1}', 'yaw', 'stop', cyl_z(c, 5, -4, 6, seg=24), 'McMaster 9541K bumper', f'Pitch hard stop at {ths:+.1f} deg')

# ------------------------------------------------------------------ link 1 (driven, O-A-E), twin plate
L1F = f2(O, d0)
for zz, nm in (((0, 6), 'back'), ((34, 40), 'front')):
    holes = [circle((0, 0), 7, 32), circle((L1, 0), 6, 32), circle((L1+M, 0), 6, 32)] if nm == 'back' else \
            [circle((0, 0), 10.5, 32), circle((L1, 0), 13, 40), circle((L1+M, 0), 6, 32)]
    pl = extrude(stadium(0, L1+M, 22, 48), holes, *zz)
    add(f'link1_plate_{nm}', 'link1', 'alu', xf(pl, L1F), 'MACH-301', 'Link 1 plate, 6 mm 6061, holes 300/550')
add('link1_hub', 'link1', 'steel', cyl_z(O, 12.5, -14, 0, r_in=7), 'MACH-302', 'Link 1 drive hub (bolts to hollow gear output)')
add('link1_hub_tube', 'link1', 'alu', cyl_z(O, 15, 6, 34, r_in=7), 'MACH-303', 'Hub spacer between plates')
add('brg_L1_O_front', 'link1', 'steel', cyl_z(O, 10.5, 34.5, 39.5, r_in=6, seg=32), 'SKF 6801-2RS', 'Link 1 front bearing on stationary shaft')
add('brg_L1_A_front', 'link1', 'steel', cyl_z(A, 13, 34.5, 39.5, r_in=8.5, seg=40), 'SKF 6803-2RS', 'Link 1 front bearing on coupler-1 sleeve (17x26x5)')
belt1 = extrude(stadium(0, L1, 11.15, 48), [stadium(0, L1, 9.55, 48)], 42.5, 47.5)
add('belt_1', 'link1', 'belt', xf(belt1, L1F), 'Gates AT3 steel-cord PU belt, 10 mm', 'Belt 1 (carried by link 1): backs up bar O\'A\' in parallelogram 1')
add('link1_enc_ring', 'link1', 'pcb', cyl_z(O, 24.5, -3.0, -1.0, r_in=12.5), 'RLS MRA049BC025', 'Pitch encoder ring 25x49x2')
for x0 in (55, 358):
    add(f'link1_spacer_{x0}', 'link1', 'alu', xf(box(x0, x0+18, -15, 15, 6, 34), L1F), 'MACH-304', 'Plate spacer')
add('pin_A', 'link1', 'steel', cyl_z(A, 6, 0, 32, seg=32) + cyl_z(A, 9, -3, 0, seg=32), 'MISUMI CBRS12 (mod.)', 'Shoulder pin A, clamped in link 1, carries belt-2 pulley')
add('pin_E', 'link1', 'steel', cyl_z(E, 6, 0, 40, seg=32) + cyl_z(E, 9, 40, 43, seg=32) + cyl_z(E, 9, -3, 0, seg=6), 'MISUMI CBRS12 (mod.)', 'Shoulder pin E, clamped in link 1 (double shear)')
pul2 = cyl_z((0, 0), 9.55, 15.5, 24.5, r_in=6, seg=40) + cyl_z((0, 0), 12, 14.5, 15.5, r_in=6, seg=40) + cyl_z((0, 0), 12, 24.5, 25.5, r_in=6, seg=40)
add('pulley_2a_A', 'link1', 'alu2', xf(pul2, T(A[0], A[1], 0)), 'AT3 20T pulley 10 mm', 'Belt-2 pulley keyed to link 1 at A')
K_LOCAL = np.array([100.0, 0.0])      # spring attachment in link-1 frame (re-aimed in build.py from mass data)
def spring_stud(klocal):
    p = O + klocal[0]*d0 + klocal[1]*rot90(d0)
    return cyl_z(p, 5, 40, 56, seg=24), p
st, K0 = spring_stud(K_LOCAL)
add('spring_stud_K', 'link1', 'steel', st, 'MACH-305', 'Counterbalance stud K')

# ------------------------------------------------------------------ link O'A' (passive, bar path of parallelogram 1)
LPF = f2(Op, d0)
for zz, nm in (((0, 6), 'back'), ((34, 40), 'front')):
    pl = extrude(stadium(0, L1, 22, 48), [circle((0, 0), 10.5, 32), circle((L1, 0), 10.5, 32)], *zz)
    add(f'linkOpAp_plate_{nm}', 'link_opap', 'alu', xf(pl, LPF), 'MACH-401', "Link O'A' plate, 6 mm 6061, holes 300")
    for xx in (0, L1):
        add(f'brg_OpAp_{nm}_{int(xx)}', 'link_opap', 'steel', xf(cyl_z((xx, 0), 10.5, zz[0]+0.5, zz[0]+5.5, r_in=6, seg=32), LPF), 'SKF 6801-2RS', 'Deep groove ball bearing 12x21x5')
add('linkOpAp_spacer', 'link_opap', 'alu', xf(box(60, 78, -15, 15, 6, 34), LPF), 'MACH-304', 'Plate spacer')

# ------------------------------------------------------------------ coupler 1 (translates; A-A'-C), box section
C1F = f2(A, b)
for zz, nm in (((8, 13), 'back'), ((27, 32), 'front')):
    w = extrude(stadium(0, n, 22, 48), [circle((0, 0), 10.5, 32), circle((m, 0), 6, 32), circle((n, 0), 10.5, 32)], *zz)
    add(f'coupler1_wall_{nm}', 'coupler1', 'alu2', xf(w, C1F), 'MACH-501', 'Coupler 1 wall, 5 mm 6061, holes 130/420')
    for xx in (0, n):
        add(f'brg_C1_{nm}_{int(xx)}', 'coupler1', 'steel', xf(cyl_z((xx, 0), 10.5, zz[0], zz[1], r_in=6, seg=32), C1F), 'SKF 6801-2RS', 'Deep groove ball bearing 12x21x5')
for sy in (1, -1):
    add(f'coupler1_rail_{"L" if sy>0 else "R"}', 'coupler1', 'alu2', xf(box(28, n-28, 16 if sy > 0 else -22, 22 if sy > 0 else -16, 13, 27), C1F), 'MACH-502', 'Coupler 1 side web')
belt2 = extrude(stadium(0, n, 11.15, 48), [stadium(0, n, 9.55, 48)], 15.5, 24.5)
add('belt_2', 'coupler1', 'belt', xf(belt2, C1F), 'Gates AT3 steel-cord PU belt, 9 mm', 'Belt 2 (carried by coupler 1): backs up bar EF in parallelogram 2')
pin_ap = cyl_z(Ap, 6, 0, 40, seg=32) + cyl_z(Ap, 9, 40, 43, seg=32) + cyl_z(Ap, 9, -3, 0, seg=6)
add("pin_Ap", 'coupler1', 'steel', pin_ap, 'MISUMI CBRS12 (mod.)', "Shoulder pin A', clamped in coupler 1")
add('sleeve_A', 'coupler1', 'steel', cyl_z(A, 8.5, 32, 49, r_in=6.2, seg=40) + cyl_z(A, 12, 49, 51, r_in=6.2, seg=40), 'MACH-503', 'Coupler-1 sleeve, concentric with pin A, through link-1 front plate')
pul1b = cyl_z((0, 0), 9.55, 42.5, 47.5, r_in=8.5, seg=40) + cyl_z((0, 0), 12, 41.5, 42.5, r_in=8.5, seg=40) + cyl_z((0, 0), 12, 47.5, 48.5, r_in=8.5, seg=40)
add("pulley_1b_A", 'coupler1', 'alu2', xf(pul1b, T(A[0], A[1], 0)), 'AT3 20T pulley 10 mm (17 bore)', 'Belt-1 pulley keyed to coupler-1 sleeve at A')

# ------------------------------------------------------------------ coupler 2 (translates; E-F)
C2F = f2(E, b)
for zz, nm in (((8, 13), 'back'), ((27, 32), 'front')):
    w = extrude(stadium(0, n, 22, 48), [circle((0, 0), 10.5, 32), circle((n, 0), 10.5, 32)], *zz)
    add(f'coupler2_wall_{nm}', 'coupler2', 'alu2', xf(w, C2F), 'MACH-601', 'Coupler 2 wall, 5 mm 6061, holes 420')
    for xx in (0, n):
        add(f'brg_C2_{nm}_{int(xx)}', 'coupler2', 'steel', xf(cyl_z((xx, 0), 10.5, zz[0], zz[1], r_in=6, seg=32), C2F), 'SKF 6801-2RS', 'Deep groove ball bearing 12x21x5')
for sy in (1, -1):
    add(f'coupler2_rail_{"L" if sy>0 else "R"}', 'coupler2', 'alu2', xf(box(28, n-28, 16 if sy > 0 else -22, 22 if sy > 0 else -16, 13, 27), C2F), 'MACH-502', 'Coupler 2 side web')

# ------------------------------------------------------------------ spar (rotates about R), rail, insertion drive, cannula
SF = f2(C, d0)           # spar local: x up the spar from C, y = world -x at ref
for zz, nm in (((0, 6), 'back'), ((34, 40), 'front')):
    pl = extrude(stadium(-210, M, 22, 48), [circle((0, 0), 6, 32), circle((M, 0), 6, 32)], *zz)
    add(f'spar_plate_{nm}', 'spar', 'alu', xf(pl, SF), 'MACH-701', 'Spar plate, 6 mm 6061, holes 0/250, 210 mm extension')
for x0 in (176, -130, -196):
    add(f'spar_spacer_{x0}', 'spar', 'alu', xf(box(x0, x0+14, -15, 15, 6, 34), SF), 'MACH-304', 'Plate spacer')
add('pin_C', 'spar', 'steel', cyl_z(C, 6, 0, 40, seg=32) + cyl_z(C, 9, 40, 43, seg=32), 'MISUMI CBRS12 (mod.)', 'Shoulder pin C, flush back face (rail passes over)')
add('pin_F', 'spar', 'steel', cyl_z(F, 6, 0, 40, seg=32) + cyl_z(F, 9, 40, 43, seg=32) + cyl_z(F, 9, -3, 0, seg=6), 'MISUMI CBRS12 (mod.)', 'Shoulder pin F')
add('pulley_2b_C', 'spar', 'alu2', xf(pul2, T(C[0], C[1], 0)), 'AT3 20T pulley 10 mm', 'Belt-2 pulley keyed to spar at C')
add('rail_MGN12', 'spar', 'steel', xf(box(-184, 190, -6, 6, -8, 0), SF), 'HIWIN MGNR12R-375', 'Linear rail 12 x 8, 375 mm')
for x0 in (-40, 180):
    add(f'insertion_stop_{x0}', 'spar', 'stop', xf(box(x0, x0+6, -7, 7, -12, -8), SF), 'MACH-702', 'Insertion hard stop (rail clamp)')
for x0, x1 in ((-202, -186), (196, 240)):
    add(f'screw_bracket_{x0}', 'spar', 'alu', xf(box(x0, x1, -66, -8, -6, 0), SF), 'MACH-703', 'Ball-screw support bracket')
add('screw_BF10', 'spar', 'black', xf(box(-200, -188, -64, -8, -38, -6), SF), 'BF10 support', 'Floating-end support, 6000ZZ')
add('screw_BK10', 'spar', 'black', xf(box(198, 223, -64, -8, -45, -6), SF), 'BK10 support', 'Fixed-end support, 7000 angular pair')
def on_spar_axis(x0, x1, r, y=-36.0, z=-28.0, r_in=0.0, seg=40):
    return xf(xf(cyl_x(r, x0, x1, r_in, seg), T(0, y, z)), SF)
add('ballscrew_SFU1204', 'spar', 'steel', on_spar_axis(-200, 223, 6), 'TBI SFU1204', 'Ball screw 12 mm, 4 mm lead (back-drivable)')
add('screw_coupling', 'spar', 'alu2', on_spar_axis(223, 233, 10), 'Ruland MCLX-6-6', 'Clamp coupling')
mp = extrude(rect(-60, -12, -50, -6), [circle((-36, -28), 11, 32)], 233, 236)
add('insertion_motor_plate', 'spar', 'alu', xf(mp, SF @ np.array([[0,0,1,0],[1,0,0,0],[0,1,0,0],[0,0,0,1]], float)), 'MACH-704', 'Insertion motor plate')
add('insertion_motor', 'spar', 'black', on_spar_axis(236, 258, 21.4), 'maxon EC 45 flat 50W', 'Insertion motor')
add('insertion_motor_enc', 'spar', 'black', on_spar_axis(258, 267, 21.4), 'maxon MILE 1024', 'Insertion motor-side encoder (channel A)')
add('insertion_brake', 'spar', 'red', on_spar_axis(267, 287, 19), 'Kendrion Servo Slim Line 04', 'Insertion spring-applied brake 0.4 Nm (>600 N at carriage)')
add('retraction_knob', 'spar', 'stop', on_spar_axis(287, 295, 15, seg=24), 'MACH-705', 'Manual retraction knob (screw is back-drivable)')
add('insertion_scale', 'spar', 'pcb', xf(box(-184, 190, 14, 22, -1.5, 0), SF), 'RLS LM10 magnetic scale', 'Insertion joint-side encoder scale (channel B)')
add('cannula_mount', 'spar', 'alu', xf(box(-232, -214, -18, 18, ZI+13, 0), SF), 'MACH-706', 'Cannula mount')
add('cannula_clamp', 'spar', 'alu', on_spar_axis(-232, -214, 13, y=0, z=ZI, r_in=6.0), 'MACH-707', 'Cannula clamp ring')
add('cannula', 'spar', 'steel', on_spar_axis(-330, -214, 6.0, y=0, z=ZI, r_in=4.25), '316 SS 12 x 0.875 tube', 'Cannula, remote-center band 90 mm below clamp')
add('cannula_rcm_band', 'spar', 'black', on_spar_axis(-302.5, -297.5, 6.6, y=0, z=ZI, r_in=6.0), 'MACH-708', 'RCM band: align with incision')

# ------------------------------------------------------------------ carriage (insertion q) and instrument drive
XT = 430.0 - L1 - Q0          # shaft top, spar-local x
add('mgn12h_block', 'carriage', 'steel', xf(box(XT+40-22.7, XT+40+22.7, -13.5, 13.5, -13, -3), SF), 'HIWIN MGN12H', 'Carriage block 27 x 45.4 x 13')
add('carriage_plate', 'carriage', 'alu', xf(box(XT, XT+80, -25, 25, -17, -13), SF), 'MACH-801', 'Carriage adapter plate')
add('nut_bracket', 'carriage', 'alu', xf(box(XT+28, XT+57, -47, -25, -17, -13), SF), 'MACH-802', 'Ball-nut bracket')
add('ball_nut', 'carriage', 'black', on_spar_axis(XT+30, XT+55, 11), 'TBI SFU1204 nut', 'Ball nut')
add('isa_housing', 'carriage', 'white', xf(box(XT, XT+80, -24, 24, ZI-30, -17), SF), 'MACH-803', 'Instrument drive (4 cable-drive outputs)')
for i, (xx, yy) in enumerate(((20, 12), (20, -12), (60, 12), (60, -12))):
    p = (SF @ np.array([XT+xx, yy, 0, 1]))[:2]
    add(f'isa_motor_{i+1}', 'carriage', 'black', cyl_z(p, 8, ZI-70, ZI-30, seg=32), 'maxon DCX 16 + GPX 16', ['Roll', 'Wrist pitch', 'Wrist yaw', 'Grip'][i] + ' drive motor')
add('instrument_release', 'carriage', 'stop', xf(box(XT+60, XT+75, 24, 30, ZI-20, ZI+10), SF), 'MACH-804', 'Instrument quick-release (manual removal)')

# ------------------------------------------------------------------ instrument (roll), wrist, jaws
add('instrument_shaft', 'instrument', 'steel', on_spar_axis(-(L1+Q0), XT, 4.0, y=0, z=ZI, r_in=2.6), '304 SS 8 x 0.7 tube', 'Instrument shaft, 430 mm to wrist axis W')
add('instrument_roll_hub', 'instrument', 'alu2', on_spar_axis(XT+2, XT+14, 9, y=0, z=ZI, r_in=4.0), 'MACH-901', 'Roll hub (inside drive)')
clev = cyl_between(W0, J0, 3.4, seg=32) + cyl_between(W0 + [0, 0, -3.8], W0 + [0, 0, 3.8], 0.8, seg=16)
add('wrist_clevis', 'wrist', 'steel', clev, 'MACH-902', 'Wrist link, pitch pin at W, yaw pin at J (9 mm)')
add('wrist_yaw_pin', 'wrist', 'steel', cyl_between(J0 + [-3.6, 0, 0], J0 + [3.6, 0, 0], 0.7, seg=16), 'MACH-903', 'Jaw pin J')
JF = frame(J0, np.array([0, -1.0, 0]), np.array([0, 0, 1.0]))      # local x along jaw, y = opening (+z), z = thickness (-x)
add('jaw_1', 'jaw1', 'steel', xf(extrude(np.array([[-1.5, 0], [14, 0], [14, 0.5], [1.5, 2.2], [-1.5, 2.2]]), [], -1.3, 1.3), JF), 'MACH-904', 'Jaw (upper)')
add('jaw_2', 'jaw2', 'steel', xf(extrude(np.array([[-1.5, -2.2], [1.5, -2.2], [14, -0.5], [14, 0], [-1.5, 0]]), [], -1.3, 1.3), JF), 'MACH-904', 'Jaw (lower)')

# ------------------------------------------------------------------ orientation monitors (second detection layer)
add('imu_coupler1', 'coupler1', 'pcb', xf(box(200, 222, -8, 8, 32, 33.6), C1F), 'Bosch BMI088 board', 'IMU on coupler 1: must stay level relative to yaw frame (parallelogram-1 integrity)')
add('imu_link1', 'link1', 'pcb', xf(box(460, 482, -8, 8, 40, 41.6), L1F), 'Bosch BMI088 board', 'IMU on link 1 (reference for spar comparison)')
add('imu_spar', 'spar', 'pcb', xf(box(110, 132, -8, 8, 40, 41.6), SF), 'Bosch BMI088 board', 'IMU on spar: must match link 1 (parallelogram-2 integrity)')

DENS = dict(alu=2.70e-6, alu2=2.70e-6, steel=7.9e-6, black=2.4e-6, red=3.0e-6, pcb=2.0e-6, belt=1.3e-6, stop=1.4e-6, white=0.9e-6)
