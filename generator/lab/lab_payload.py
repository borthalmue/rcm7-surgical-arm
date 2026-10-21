"""Adds per-part inertia tensors to the simulator payload for the Motion Lab dynamics.

Every mass-carrying part mesh is a closed solid, so its volume integrals come straight from the
triangle soup (signed tetrahedra to the origin). The tensor is scaled to the part's catalog or
modelled mass and stored about the part's own centre of mass, in the reference-pose world frame,
as kg*mm^2 [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] (tensor entries, so products carry their minus sign).
"""
import base64, json, os, sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'out', 'sim_payload.json')
DST = os.path.join(HERE, '..', 'out', 'lab_payload.json')
C0 = np.array([[2, 1, 1], [1, 2, 1], [1, 1, 2]]) / 120.0


def inertia(part):
    P = np.frombuffer(base64.b64decode(part['p']), dtype=np.float32).reshape(-1, 3, 3).astype(float)
    com = np.array(part['com'])
    P = P - com
    vol, cov, first = 0.0, np.zeros((3, 3)), np.zeros(3)
    for a, b, c in P:
        A = np.column_stack([a, b, c])
        d = np.linalg.det(A)
        vol += d / 6
        first += d / 24 * (a + b + c)
        cov += d * A @ C0 @ A.T
    if vol <= 0:
        return None, 0
    cen = first / vol
    cov -= vol * np.outer(cen, cen)            # about the mesh centroid (matches com to < 1 mm)
    rho = part['kg'] / vol
    I = rho * (np.trace(cov) * np.eye(3) - cov)
    return [I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]], float(np.linalg.norm(cen))


def main():
    d = json.load(open(SRC))
    worst = 0
    for p in d['parts']:
        if p.get('kg', 0) > 0:
            I, off = inertia(p)
            if I is None:
                sys.exit(f"{p['name']}: open or inverted mesh")
            p['I'] = [round(v, 3) for v in I]
            worst = max(worst, off)
    json.dump(d, open(DST, 'w'), separators=(',', ':'))
    print(f"inertia added to {sum(1 for p in d['parts'] if 'I' in p)} parts; worst centroid offset {worst:.3f} mm -> {DST}")


if __name__ == '__main__':
    main()
