import sys, time, numpy as np
from zrender import *
which = sys.argv[1:] or ['hero']
pose = dict(REF, theta=12, yaw=22, q=125, psi=30, chi=-20, grip=35)
O, Op, A, Ap, C, E, F, b = Mo.O, Mo.Op, Mo.A, Mo.Ap, Mo.C, Mo.E, Mo.F, Mo.b
Rm = Mo.R3
def run(name, **kw):
    t = time.time(); img, proj, Tb = render(kw['pose'], kw['eye'], kw['target'], W=kw.get('W', 1400), H=kw.get('H', 1000), fov=kw.get('fov', 30), hide=kw.get('hide', ()))
    figure(img, proj, Tb, kw['labels'], kw['title'], kw['sub'], f'views/{name}.png'); print(name, round(time.time()-t, 1), 's')
if 'hero' in which:
    run('1_overall', pose=pose, eye=(-1450, 780, 1500), target=(-280, 170, -40), fov=30,
        labels=[('Yaw drive: axis passes through R', 'ground', [*(Rm[:2] - 560*b), Rm[2]+52], 'L'),
                ('Setup column (to cart / bed rail)', 'ground', list(Mo.ax_pt(557, 0, 240) + np.array([0, -250, 34])), 'L'),
                ('Pitch drive at O (hollow gear)', 'yaw', [O[0], O[1], -60], 'L'),
                ('Link 1 (driven, twin plate)', 'link1', [O[0], O[1]+470, 40], 'L'),
                ("Link O'A' (bar path, parallelogram 1)", 'link_opap', [Op[0], Op[1]+200, 40], 'L'),
                ('Coupler EF (bar path, parallelogram 2)', 'coupler2', [*(E + 260*b), 32], 'R'),
                ('Coupler 1 (carries belt 2)', 'coupler1', [*(A + 300*b), 32], 'R'),
                ('Belt 1 (backup path, parallelogram 1)', 'link1', [O[0], O[1]+200, 47.5], 'L'),
                ('Spar with rail and ball screw', 'spar', [0, C[1]+150, 40], 'R'),
                ('Instrument drive (4 motors)', 'carriage', [0, C[1]+Mo.XT+40, -120], 'R'),
                ('Cannula: RCM band at the incision', 'spar', [0, 5, Rm[2]+6], 'R'),
                ('2-DOF wrist + grip', 'jaw1', list(Mo.J0), 'R')],
        title='RCM-7 arm', sub='7 DOF · yaw and instrument roll both pass through the remote center R · 129 parts')
if 'pitchf' in which:
    run('2_pitch_front', pose=dict(pose, yaw=0, theta=18), eye=(-620, 360, 640), target=(-380, 160, 10), fov=30,
        labels=[('Belt 1 rides on link 1', 'link1', [O[0], O[1]+150, 47.5], 'L'),
                ('Ground pulley on stationary shaft', 'yaw', [O[0], O[1], 48.5], 'L'),
                ('Counterbalance spring anchor G', 'yaw', [*(Mo.Gs), 62], 'L'),
                ('Counterbalance arm', 'yaw', [*(O + (-30)*b + (-38)*Mo.nh + 0.5*(Mo.Gs - (O + (-30)*b + (-38)*Mo.nh))), 68], 'L'),
                ("O' pivot, double shear", 'yaw', [Op[0], Op[1], 43], 'R'),
                ('Pitch hard stops', 'yaw', [*(O + 40*np.array([np.cos(np.radians(90+46)+np.arcsin(27/40)), np.sin(np.radians(90+46)+np.arcsin(27/40))])), 6], 'R'),
                ('Spring stud K on link 1', 'link1', [O[0]+1.6, O[1]+100, 56], 'R'),
                ('Pitch frame', 'yaw', [*(O + 120*b - 30*Mo.nh), -4], 'R')],
        title='Pitch joint, front', sub='Belt 1 is the backup for link O′A′: ground pulley at O, coupler pulley at A')
if 'pitchb' in which:
    run('3_pitch_back', pose=dict(pose, yaw=0, theta=10), eye=(-520, 260, -820), target=(-390, 130, -60), fov=32,
        labels=[('SHG-17-100 hollow strain-wave gear', 'yaw', [O[0]-42*b[0], O[1]-42*b[1], -45], 'L'),
                ('Input belt, 2:1', 'yaw', [*(O + 40*b), -78], 'R'),
                ('maxon EC 45 flat motor', 'yaw', [*(O + 75*b + 21*Mo.nh), -100], 'R'),
                ('MILE motor encoder (A)', 'yaw', [*(O + 75*b + 21*Mo.nh), -117], 'R'),
                ('Spring-applied brake', 'yaw', [*(O + 75*b - 25*Mo.nh), -131], 'R'),
                ('Manual brake release', 'yaw', [*(O + 75*b + np.array([45, 0])), -133], 'R'),
                ('Yaw housing (CSF-17 + RB5013)', 'ground', [*(Rm[:2] - 545*b - 52*Mo.nh), Rm[2]], 'L'),
                ('Yaw yoke', 'yaw', [*(Rm[:2] - 483*b), -90], 'L')],
        title='Pitch joint, back (drive side)', sub='Hollow gear lets the belt-1 ground shaft pass through; the motor is offset on a 2:1 belt')
if 'coupler' in which:
    hide = ('coupler1_wall_front', 'link1_plate_front', 'spar_plate_front', 'brg_C1_front_0', 'brg_C1_front_420', 'imu_coupler1',
            'belt_1', 'pulley_1b_A', 'sleeve_A', 'brg_L1_A_front', 'linkOpAp_plate_front', 'pin_C', 'brg_L1_O_front', 'pin_Ap',
            'brg_OpAp_front_300', 'brg_OpAp_front_0', 'imu_spar', 'imu_link1')
    run('4_coupler_belt2', pose=dict(pose, yaw=0, theta=8), eye=(-90, 520, 560), target=(-190, 300, 15), fov=34, hide=hide,
        labels=[('Belt 2 inside coupler 1 (backup for EF)', 'coupler1', [*(A + 230*b + 10.4*Mo.nh), 20], 'R'),
                ('Pulley keyed to link 1 (at A)', 'link1', [A[0], A[1], 25.5], 'L'),
                ('Pulley keyed to spar (at C)', 'spar', [C[0], C[1], 25.5], 'R'),
                ('6801 bearings in both walls', 'coupler1', [*(A + 0*b + 10.5*Mo.nh), 10.5], 'L'),
                ("Pin A', clamped in coupler 1", 'coupler1', [Ap[0], Ap[1], 20], 'L'),
                ('Coupler side webs', 'coupler1', [*(A + 330*b + 19*Mo.nh), 20], 'R'),
                ('Link 1 back plate', 'link1', [O[0]-10, O[1]+250, 6], 'L')],
        title='Coupler 1, front wall removed', sub='Belt 2 keeps the spar parallel to link 1 even if coupler EF breaks')
if 'insert' in which:
    p2 = dict(pose, yaw=0, theta=0, q=110)
    run('5_insertion', pose=p2, eye=(420, 520, -560), target=(10, 360, -40), fov=34,
        labels=[('HIWIN MGN12 rail + MGN12H block', 'spar', [0, C[1]+150, -4], 'L'),
                ('TBI SFU1204 ball screw (back-drivable)', 'spar', [36, C[1]+150, -22], 'R'),
                ('BK10 fixed support', 'spar', [36, C[1]+210, -40], 'R'),
                ('Insertion motor + 0.4 N·m brake', 'spar', [36, C[1]+262, -6.6], 'R'),
                ('Manual retraction knob', 'spar', [36, C[1]+295, -28], 'R'),
                ('Insertion hard stops', 'spar', [0, C[1]+183, -12], 'L'),
                ('Instrument drive: roll, wrist ×2, grip', 'carriage', [0, C[1]+Mo.XT+40, -90], 'L'),
                ('Cannula mount', 'spar', [0, C[1]-223, Mo.ZI+13], 'L')],
        title='Insertion stage', sub='Back of the spar · 165 mm stroke along the shaft axis, which always passes through R')
if 'wrist' in which:
    p3 = dict(pose, yaw=0, theta=0, q=120, roll=30, psi=35, chi=-25, grip=40)
    Tb = body_T(p3); Wp = pt(Tb['wrist'], Mo.W0)
    run('6_wrist', pose=p3, eye=Wp + np.array([55, 25, 70]), target=Wp + np.array([0, -6, 0]), fov=34, W=1200, H=900,
        labels=[('Wrist pitch pin W', 'wrist', list(Mo.W0 + np.array([0, 0, 3.8])), 'L'),
                ('Jaw pin J (9 mm from W)', 'wrist', list(Mo.J0 + np.array([3.6, 0, 0])), 'R'),
                ('Jaw', 'jaw1', list(Mo.J0 + np.array([0, -10, 1.2])), 'R'),
                ('Jaw', 'jaw2', list(Mo.J0 + np.array([0, -10, -1.2])), 'R'),
                ('8 mm instrument shaft', 'instrument', [0, -Mo.Q0+25, Mo.ZI+4], 'L')],
        title='2-DOF wrist + grip', sub='Pitch at W, yaw and grip at J. Locked until the jaws clear the cannula (q ≥ 55 mm)')
