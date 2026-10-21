/* RCM-7 Motion Lab: planning, dynamics, servo and test engine.
 *
 * Frames and units
 *   Task space: millimetres, origin at the remote center R (the incision), world axes:
 *   +X right, +Y up (out of the body), +Z toward the viewer. Depth below R is negative Y.
 *   Joint vector x = [yaw, pitch, insertion, roll, wristPitch, wristYaw, grip]
 *   with angles in radians and insertion in mm (wrist axis W below R).
 *   The tool point (TCP) is the jaw midline tip, 22.5 mm beyond W when the wrist is straight.
 */
(function (root) {
'use strict';

function createCore(MT, PARTS) {
  const D2R = Math.PI / 180;
  const beta = MT.beta * D2R, ZI = MT.ZI, L1 = MT.L1, ML = MT.M, Q0 = MT.Q0;
  const O = MT.O, Op = MT.Op, W0 = MT.W0, J0 = MT.J0;
  const B3 = [Math.cos(beta), -Math.sin(beta), 0], R3 = [0, 0, ZI];
  const LJ = W0[1] - J0[1], LJAW = 13.5, LT = LJ + LJAW;
  const JN = ['yaw', 'pitch', 'insertion', 'roll', 'wrist pitch', 'wrist yaw', 'grip'];
  const UNIT = ['rad', 'rad', 'mm', 'rad', 'rad', 'rad', 'rad'];
  const LO = [MT.yaw[0] * D2R, MT.pitch[0] * D2R, MT.q[0], -260 * D2R, -80 * D2R, -80 * D2R, 0];
  const HI = [MT.yaw[1] * D2R, MT.pitch[1] * D2R, MT.q[1], 260 * D2R, 80 * D2R, 80 * D2R, 60 * D2R];
  const VLIM = [45 * D2R, 45 * D2R, 80, 360 * D2R, 180 * D2R, 180 * D2R, 240 * D2R];
  const ALIM = [120 * D2R, 120 * D2R, 300, 1200 * D2R, 900 * D2R, 900 * D2R, 1200 * D2R];
  const WQMIN = MT.wristQmin;
  // drive data (catalog values; reflected rotor inertia uses an approximate maxon EC 45 flat rotor inertia)
  const DRIVE = {
    name: ['yaw', 'pitch', 'insertion'], unit: ['N·m', 'N·m', 'N'],
    rated: [24, 31, 119],          // CSF-17 rated, SHG-17 rated, insertion motor continuous through SFU1204
    peak: [54, 70, 357],           // gear repeated-peak limits; insertion = 3x continuous (assumed)
    motorCont: [6.3, 12.6, 119],   // 84.3 mN·m nominal x ratio x 0.75 efficiency
    rotor: [1.35e-5 * 100 * 100, 1.35e-5 * 200 * 200, 1.35e-5 * Math.pow(2 * Math.PI / 0.004, 2)],
    coulomb: [0.5, 0.5, 3.0], viscous: [0.2, 0.2, 20],
    encoder: [2 * Math.PI / Math.pow(2, 20), 2 * Math.PI / Math.pow(2, 20), 0.001]   // AksIM-2 20-bit; 1 um linear scale
  };

  // ---------------------------------------------------------------- small math
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
  const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
  const crs = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const nrm = a => Math.hypot(a[0], a[1], a[2]);
  const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  function rod(k, a, v) { const c = Math.cos(a), s = Math.sin(a), kv = crs(k, v), kd = dot(k, v) * (1 - c);
    return [v[0] * c + kv[0] * s + k[0] * kd, v[1] * c + kv[1] * s + k[1] * kd, v[2] * c + kv[2] * s + k[2] * kd]; }
  const mj = u => u * u * u * (10 + u * (-15 + 6 * u));          // minimum-jerk shape
  const mjd = u => 30 * u * u * (1 - u) * (1 - u);
  const mjdd = u => 60 * u * (1 - u) * (1 - 2 * u);
  function rng(seed) { let s = seed >>> 0 || 1; return () => { s ^= s << 13; s >>>= 0; s ^= s >> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; }; }

  // ---------------------------------------------------------------- forward kinematics of the tool point (exact, closed form)
  function tipRel(x) {
    const cc = Math.cos(x[5]), sc = Math.sin(x[5]);
    const w1 = -LJ - LJAW * cc, w2 = -LJAW * sc;
    const cp = Math.cos(x[4]), sp = Math.sin(x[4]);
    const wx = -w1 * sp, wy = w1 * cp, wz = w2;
    const cr = Math.cos(x[3]), sr = Math.sin(x[3]);
    const rx = wx * cr - wz * sr, rz = wx * sr + wz * cr;
    const vx = rx, vy = -x[2] + wy, vz = rz;
    const ct = Math.cos(x[1]), st = Math.sin(x[1]);
    return rod(B3, x[0], [vx * ct - vy * st, vx * st + vy * ct, vz]);
  }
  // shaft direction (unit, R toward the wrist)
  function shaftDir(x) { return rod(B3, x[0], [Math.sin(x[1]), -Math.cos(x[1]), 0]); }

  // ---------------------------------------------------------------- inverse kinematics
  function ikStraight(p, off = LT) {
    const r = nrm(p); if (r < 1e-9) return null;
    const u = mul(p, 1 / r), sb = dot(u, B3);
    const th = Math.asin(clamp(sb, -1, 1)) - beta;
    const s0 = [Math.sin(th), -Math.cos(th), 0];
    const sp = sub(s0, mul(B3, dot(s0, B3))), up = sub(u, mul(B3, sb));
    const yaw = Math.atan2(dot(B3, crs(sp, up)), dot(sp, up));
    return [yaw, th, r - off];
  }
  function det3(A) { return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]); }
  function solve3(A, b) { const D = det3(A); if (Math.abs(D) < 1e-18) return null;
    const out = []; for (let c = 0; c < 3; c++) out.push(det3(A.map((row, i) => row.map((v, j) => j === c ? b[i] : v))) / D); return out; }
  // Newton solve for [yaw, pitch, insertion] that puts the tool point exactly on p, for a given wrist pose [roll, wp, wy]
  function ikTip(p, wr, seed) {
    const full = y => [y[0], y[1], y[2], wr[0], wr[1], wr[2]];
    let x = seed ? [seed[0], seed[1], seed[2]] : ikStraight(p);
    if (!x) return { x: null, err: Infinity, it: 0 };
    if (!seed) for (let k = 0; k < 4; k++) {        // fixed-point seed: remove the wrist offset, solve for the wrist point
      const o = sub(tipRel(full(x)), mul(shaftDir(x), x[2])), xw = ikStraight(sub(p, o), 0);
      if (!xw) break; x = xw;
    }
    let err = Infinity, it = 0;
    for (; it < 30; it++) {
      const f = sub(tipRel(full(x)), p); err = nrm(f); if (err < 1e-11) break;
      const hs = [1e-7, 1e-7, 1e-5], Jc = [];
      for (let j = 0; j < 3; j++) { const a = x.slice(), b = x.slice(); a[j] += hs[j]; b[j] -= hs[j];
        Jc.push(mul(sub(tipRel(full(a)), tipRel(full(b))), 1 / (2 * hs[j]))); }
      const dx = solve3([[Jc[0][0], Jc[1][0], Jc[2][0]], [Jc[0][1], Jc[1][1], Jc[2][1]], [Jc[0][2], Jc[1][2], Jc[2][2]]], mul(f, -1));
      if (!dx) break;
      const sc = Math.min(1, 0.5 / Math.max(Math.abs(dx[0]), Math.abs(dx[1]), Math.abs(dx[2]) / 60));   // trust region
      x = [x[0] + dx[0] * sc, x[1] + dx[1] * sc, x[2] + dx[2] * sc];
    }
    const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
    x = [wrap(x[0]), wrap(x[1]), x[2]];
    return { x, err, it };
  }
  function limitIssues(x, from = 0, to = 7) {
    const out = [];
    for (let j = from; j < to; j++) {
      if (x[j] < LO[j] - 1e-9) out.push(`${JN[j]} ${fmtJ(j, x[j])} below ${fmtJ(j, LO[j])}`);
      if (x[j] > HI[j] + 1e-9) out.push(`${JN[j]} ${fmtJ(j, x[j])} above ${fmtJ(j, HI[j])}`);
    }
    return out;
  }
  const fmtJ = (j, v) => j === 2 ? v.toFixed(1) + ' mm' : v.toFixed(3) + ' rad';
  // reachability of a tool point with a given wrist pose
  function reach(p, wr = [0, 0, 0]) {
    const s = ikTip(p, wr);
    if (!s.x || s.err > 1e-6) return { ok: false, x: null, issues: ['no inverse-kinematics solution'] };
    const issues = limitIssues([...s.x, 0, 0, 0, 0], 0, 3);
    if (s.x[2] < WQMIN && (Math.abs(wr[1]) > 1e-6 || Math.abs(wr[2]) > 1e-6)) issues.push('wrist must be straight this close to the cannula (insertion < 55 mm)');
    return { ok: issues.length === 0, x: s.x, issues };
  }
  // closest reachable point in joint space (clamp)
  function snap(p) { const x = ikStraight(p) || [0, 0, 100]; const c = [0, 1, 2].map(j => clamp(x[j], LO[j], HI[j])); return tipRel([...c, 0, 0, 0]); }

  // ---------------------------------------------------------------- full-body kinematics (row-major 4x4), mirrors the CAD simulator
  const mI = () => [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
  const mT = (x, y, z) => [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1];
  function mRot(k, a, p) {
    const c = Math.cos(a), s = Math.sin(a), t = 1 - c, x = k[0], y = k[1], z = k[2];
    const R = [t * x * x + c, t * x * y - s * z, t * x * z + s * y, t * x * y + s * z, t * y * y + c, t * y * z - s * x, t * x * z - s * y, t * y * z + s * x, t * z * z + c];
    return [R[0], R[1], R[2], p[0] - (R[0] * p[0] + R[1] * p[1] + R[2] * p[2]),
            R[3], R[4], R[5], p[1] - (R[3] * p[0] + R[4] * p[1] + R[5] * p[2]),
            R[6], R[7], R[8], p[2] - (R[6] * p[0] + R[7] * p[1] + R[8] * p[2]), 0, 0, 0, 1];
  }
  function mMul(a, b) { const r = new Array(16); for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    r[i * 4 + j] = a[i * 4] * b[j] + a[i * 4 + 1] * b[4 + j] + a[i * 4 + 2] * b[8 + j] + a[i * 4 + 3] * b[12 + j]; } return r; }
  const mApp = (m, p) => [m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + m[3], m[4] * p[0] + m[5] * p[1] + m[6] * p[2] + m[7], m[8] * p[0] + m[9] * p[1] + m[10] * p[2] + m[11]];
  const Zax = [0, 0, 1], Xax = [1, 0, 0], U0 = [0, -1, 0];
  function bodyT(x) {
    const th = x[1], d = [-Math.sin(th), Math.cos(th)], dd = [d[0], d[1] - 1];
    const Y = mRot(B3, x[0], R3), T = { ground: mI(), yaw: Y };
    T.link1 = mMul(Y, mRot(Zax, th, [O[0], O[1], 0]));
    T.link_opap = mMul(Y, mRot(Zax, th, [Op[0], Op[1], 0]));
    T.coupler1 = mMul(Y, mT(L1 * dd[0], L1 * dd[1], 0));
    T.coupler2 = mMul(Y, mT((L1 + ML) * dd[0], (L1 + ML) * dd[1], 0));
    T.spar = mMul(Y, mRot(Zax, th, [0, 0, 0]));
    T.carriage = mMul(T.spar, mT(0, -(x[2] - Q0), 0));
    T.instrument = mMul(T.carriage, mRot(U0, x[3], R3));
    T.wrist = mMul(T.instrument, mRot(Zax, x[4], W0));
    T.jaw1 = mMul(T.wrist, mRot(Xax, x[5] - x[6] / 2, J0));
    T.jaw2 = mMul(T.wrist, mRot(Xax, x[5] + x[6] / 2, J0));
    return T;
  }
  function rcmError(x) { // distance from R to the shaft centreline, from the full-body chain
    const T = bodyT(x), a = mApp(T.instrument, [0, 300, ZI]), b = mApp(T.instrument, [0, -200, ZI]);
    const u = mul(sub(b, a), 1 / nrm(sub(b, a))); return nrm(crs(sub(R3, a), u));
  }

  // ---------------------------------------------------------------- mass properties per body (from every CAD part's mass, CoM and inertia)
  const BODIES = {};
  PARTS.forEach(p => {
    if (!p.kg || p.body === 'ground') return;
    const b = BODIES[p.body] || (BODIES[p.body] = { m: 0, mc: [0, 0, 0], parts: [] });
    b.m += p.kg; b.mc = add(b.mc, mul(p.com, p.kg)); b.parts.push(p);
  });
  Object.values(BODIES).forEach(b => {
    b.c = mul(b.mc, 1 / b.m);
    const I = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    b.parts.forEach(p => {
      const r = sub(p.com, b.c), r2 = dot(r, r), Ip = p.I || [0, 0, 0, 0, 0, 0];   // kg·mm², [xx,yy,zz,xy,xz,yz]
      const own = [[Ip[0], Ip[3], Ip[4]], [Ip[3], Ip[1], Ip[5]], [Ip[4], Ip[5], Ip[2]]];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) I[i][j] += own[i][j] + p.kg * ((i === j ? r2 : 0) - r[i] * r[j]);
    });
    b.I = I.map(row => row.map(v => v * 1e-6));   // kg·m²
    delete b.parts;
  });
  const BNAMES = Object.keys(BODIES);
  const GS = [MT.G[0], MT.G[1], 59], KSP = [MT.K[0], MT.K[1], 59], KS = MT.KS;
  function bodyState(x) {
    const T = bodyT(x), S = {};
    for (const nm of BNAMES) { const m = T[nm]; S[nm] = { p: mApp(m, BODIES[nm].c), R: [[m[0], m[1], m[2]], [m[4], m[5], m[6]], [m[8], m[9], m[10]]] }; }
    S._G = mApp(T.yaw, GS); S._K = mApp(T.link1, KSP);
    return S;
  }
  function potential(S) {       // J
    let U = 0; for (const nm of BNAMES) U += BODIES[nm].m * 9.81 * S[nm].p[1] * 1e-3;
    const d = nrm(sub(S._G, S._K)); return U + 0.5 * KS * d * d * 1e-3;
  }
  // generalised mass matrix (3x3, SI) and gravity vector for yaw, pitch, insertion
  function massGrav(x) {
    const hs = [1e-5, 1e-5, 1e-3], SI = [1, 1, 1e-3];
    const v = [], w = [], g = [0, 0, 0];
    for (let j = 0; j < 3; j++) {
      const a = x.slice(), b = x.slice(); a[j] += hs[j]; b[j] -= hs[j];
      const Sa = bodyState(a), Sb = bodyState(b), h2 = 2 * hs[j] * SI[j];
      g[j] = (potential(Sa) - potential(Sb)) / h2;
      const vj = {}, wj = {};
      for (const nm of BNAMES) {
        vj[nm] = mul(sub(Sa[nm].p, Sb[nm].p), 1e-3 / h2);
        if (j === 2) { wj[nm] = [0, 0, 0]; continue; }
        const Ra = Sa[nm].R, Rb = Sb[nm].R, R0 = Sa[nm].R;   // dR Rᵀ (skew)
        const dR = Ra.map((row, i) => row.map((val, k) => (val - Rb[i][k]) / h2));
        const W = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
        for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) for (let l = 0; l < 3; l++) W[i][k] += dR[i][l] * R0[k][l];
        wj[nm] = [(W[2][1] - W[1][2]) / 2, (W[0][2] - W[2][0]) / 2, (W[1][0] - W[0][1]) / 2];
      }
      v.push(vj); w.push(wj);
    }
    const S0 = bodyState(x), M = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
    for (const nm of BNAMES) {
      const B = BODIES[nm], R = S0[nm].R;
      const Iw = R.map(ri => [0, 1, 2].map(k => ri[0] * (B.I[0][0] * R[k][0] + B.I[0][1] * R[k][1] + B.I[0][2] * R[k][2])
        + ri[1] * (B.I[1][0] * R[k][0] + B.I[1][1] * R[k][1] + B.I[1][2] * R[k][2]) + ri[2] * (B.I[2][0] * R[k][0] + B.I[2][1] * R[k][1] + B.I[2][2] * R[k][2])));
      for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) {
        const wi = w[i][nm], wk = w[k][nm];
        M[i][k] += B.m * dot(v[i][nm], v[k][nm]) + dot(wi, [dot(Iw[0], wk), dot(Iw[1], wk), dot(Iw[2], wk)]);
      }
    }
    for (let j = 0; j < 3; j++) M[j][j] += DRIVE.rotor[j];
    return { M, g };
  }

  // ---------------------------------------------------------------- obstacles: the whole shaft (R to tool point) must clear every keep-out sphere
  function segDist(c, p) {   // distance from point c to segment [R, p] (R is the origin in task space)
    const pp = dot(p, p), t = pp > 0 ? clamp(dot(c, p) / pp, 0, 1) : 0; return nrm(sub(c, mul(p, t)));
  }
  function clearance(p, obstacles) { let m = Infinity; for (const o of obstacles) m = Math.min(m, segDist(o.c, p) - o.r); return m; }

  // ---------------------------------------------------------------- path geometry: Bézier optimised around keep-out zones
  const bez = (P, u) => { const a = 1 - u, b0 = a * a * a, b1 = 3 * a * a * u, b2 = 3 * a * u * u, b3 = u * u * u;
    return [b0 * P[0][0] + b1 * P[1][0] + b2 * P[2][0] + b3 * P[3][0], b0 * P[0][1] + b1 * P[1][1] + b2 * P[2][1] + b3 * P[3][1], b0 * P[0][2] + b1 * P[1][2] + b2 * P[2][2] + b3 * P[3][2]]; };
  function nelderMead(f, x0, step, iters) {
    const n = x0.length; let S = [x0.slice()];
    for (let i = 0; i < n; i++) { const x = x0.slice(); x[i] += step[i]; S.push(x); }
    let F = S.map(f);
    for (let k = 0; k < iters; k++) {
      const idx = F.map((v, i) => i).sort((a, b) => F[a] - F[b]); S = idx.map(i => S[i]); F = idx.map(i => F[i]);
      if (Math.abs(F[n] - F[0]) < 1e-10) break;
      const c = new Array(n).fill(0); for (let i = 0; i < n; i++) for (let d = 0; d < n; d++) c[d] += S[i][d] / n;
      const xr = c.map((v, d) => v + (v - S[n][d])), fr = f(xr);
      if (fr < F[0]) { const xe = c.map((v, d) => v + 2 * (v - S[n][d])), fe = f(xe); if (fe < fr) { S[n] = xe; F[n] = fe; } else { S[n] = xr; F[n] = fr; } }
      else if (fr < F[n - 1]) { S[n] = xr; F[n] = fr; }
      else { const xc = c.map((v, d) => v + 0.5 * (S[n][d] - v)), fc = f(xc);
        if (fc < F[n]) { S[n] = xc; F[n] = fc; }
        else { for (let i = 1; i <= n; i++) { S[i] = S[i].map((v, d) => S[0][d] + 0.5 * (v - S[0][d])); F[i] = f(S[i]); } } }
    }
    let bi = 0; for (let i = 1; i <= n; i++) if (F[i] < F[bi]) bi = i; return { x: S[bi], f: F[bi] };
  }
  function avoidPath(A, B, obstacles, margin = 3) {
    const K = 48, D = sub(B, A), Lab = nrm(D);
    const cost = c => {
      const P = [A, [c[0], c[1], c[2]], [c[3], c[4], c[5]], B];
      let L = 0, pen = 0, prev = A;
      for (let k = 1; k <= K; k++) {
        const p = bez(P, k / K); L += nrm(sub(p, prev)); prev = p;
        for (const o of obstacles) { const d = segDist(o.c, p) - o.r - margin; if (d < 0) pen += d * d; }
        const x = ikStraight(p);
        if (!x) pen += 1e4; else for (let j = 0; j < 3; j++) { const s = j === 2 ? 1 : 120;
          const lo = (j === 2 ? LO[j] + 2 : LO[j] + 0.01), hi = (j === 2 ? HI[j] - 2 : HI[j] - 0.01);
          if (x[j] < lo) pen += Math.pow((lo - x[j]) * s, 2); if (x[j] > hi) pen += Math.pow((x[j] - hi) * s, 2); }
      }
      const c1 = sub(add(P[0], P[2]), mul(P[1], 2)), c2 = sub(add(P[1], P[3]), mul(P[2], 2));
      return L + 2000 * pen + 0.0015 * (dot(c1, c1) + dot(c2, c2));
    };
    // seeds: straight thirds, then detours shallower (toward R), deeper, and to either side
    const t1 = lerp3(A, B, 1 / 3), t2 = lerp3(A, B, 2 / 3), mid = lerp3(A, B, 0.5);
    const straight = [...t1, ...t2];
    if (cost(straight) <= Lab * (1 + 1e-9) + 1e-9) { const P = [A, t1, t2, B]; return { P, cost: Lab, fn: u => bez(P, u), straight: true }; }
    const radial = mul(mid, -1 / Math.max(1, nrm(mid)));                       // toward R
    let side = crs(D, mid); side = nrm(side) > 1e-9 ? mul(side, 1 / nrm(side)) : [0, 0, 1];
    const amp = Math.max(25, 0.6 * Lab), seeds = [[0, 0, 0], mul(radial, amp), mul(radial, -amp * 0.6), mul(side, amp), mul(side, -amp)];
    let best = null;
    for (const s of seeds) {
      const x0 = [...add(t1, s), ...add(t2, s)];
      const r = nelderMead(cost, x0, [8, 8, 8, 8, 8, 8], 700);
      const r2 = nelderMead(cost, r.x, [2, 2, 2, 2, 2, 2], 400);
      if (!best || r2.f < best.f) best = r2;
    }
    const P = [A, best.x.slice(0, 3), best.x.slice(3, 6), B];
    return { P, cost: best.f, fn: u => bez(P, u) };
  }

  function gauss(A0, b0) {      // dense solve with partial pivoting
    const n = b0.length, A = A0.map(r => r.slice()), b = b0.slice();
    for (let i = 0; i < n; i++) {
      let p = i; for (let r = i + 1; r < n; r++) if (Math.abs(A[r][i]) > Math.abs(A[p][i])) p = r;
      [A[i], A[p]] = [A[p], A[i]]; [b[i], b[p]] = [b[p], b[i]];
      for (let r = i + 1; r < n; r++) { const f = A[r][i] / A[i][i]; if (!f) continue; for (let c = i; c < n; c++) A[r][c] -= f * A[i][c]; b[r] -= f * b[i]; }
    }
    const x = new Array(n).fill(0);
    for (let i = n - 1; i >= 0; i--) { let s = b[i]; for (let c = i + 1; c < n; c++) s -= A[i][c] * x[c]; x[i] = s / A[i][i]; }
    return x;
  }
  // ---------------------------------------------------------------- learned movement primitive (discrete DMP, 30 basis functions)
  const DMP = { K: 150, D: 2 * Math.sqrt(150), alpha: 4, nb: 30 };
  function frameOf(d) {
    const e1 = mul(d, 1 / nrm(d)); let up = [0, 1, 0]; if (Math.abs(dot(up, e1)) > 0.95) up = [1, 0, 0];
    const e2u = sub(up, mul(e1, dot(up, e1))), e2 = mul(e2u, 1 / nrm(e2u)), e3 = crs(e1, e2); return [e1, e2, e3];
  }
  function dmpFit(demo) {
    const P = demo.p, Tm = demo.t, n = 200, T0 = Tm[0], tau = Tm[Tm.length - 1] - T0;
    const x0 = P[0], gl = P[P.length - 1], L = nrm(sub(gl, x0));
    if (L < 5 || tau < 0.2) return null;
    const E = frameOf(sub(gl, x0));
    // resample uniform in time, express in the normalised start→goal frame
    const Y = [];
    for (let i = 0; i < n; i++) {
      const t = T0 + tau * i / (n - 1); let k = 0; while (k < Tm.length - 2 && Tm[k + 1] < t) k++;
      const f = clamp((t - Tm[k]) / Math.max(1e-9, Tm[k + 1] - Tm[k]), 0, 1), p = lerp3(P[k], P[k + 1], f), v = sub(p, x0);
      Y.push([dot(E[0], v) / L, dot(E[1], v) / L, dot(E[2], v) / L]);
    }
    for (let pass = 0; pass < 2; pass++) for (let i = 2; i < n - 2; i++) for (let d = 0; d < 3; d++)   // light smoothing
      Y[i][d] = (Y[i - 2][d] + 2 * Y[i - 1][d] + 3 * Y[i][d] + 2 * Y[i + 1][d] + Y[i + 2][d]) / 9;
    const dt = tau / (n - 1), gN = [1, 0, 0];
    const c = [], h = []; for (let b = 0; b < DMP.nb; b++) c.push(Math.exp(-DMP.alpha * b / (DMP.nb - 1)));
    for (let b = 0; b < DMP.nb; b++) h.push(Math.pow(DMP.nb, 1.5) / c[b] / DMP.alpha);
    // forcing-term targets, then one global ridge least-squares fit per axis
    const Phi = [], F = [[], [], []];
    for (let i = 1; i < n - 1; i++) {
      const s = Math.exp(-DMP.alpha * (i * dt) / tau);
      let sw = 0; const ps = c.map((cb, bb) => { const q = Math.exp(-h[bb] * Math.pow(s - cb, 2)); sw += q; return q; });
      Phi.push(ps.map(q => q * s / Math.max(sw, 1e-12)));
      for (let d = 0; d < 3; d++) {
        const yd = (Y[i + 1][d] - Y[i - 1][d]) / (2 * dt), ydd = (Y[i + 1][d] - 2 * Y[i][d] + Y[i - 1][d]) / (dt * dt);
        F[d].push((tau * tau * ydd - DMP.K * (gN[d] - Y[i][d]) + DMP.D * tau * yd) / DMP.K + gN[d] * s);
      }
    }
    const nb = DMP.nb, AtA = Array.from({ length: nb }, () => new Array(nb).fill(0));
    for (const r of Phi) for (let a2 = 0; a2 < nb; a2++) { if (!r[a2]) continue; for (let b2 = 0; b2 < nb; b2++) AtA[a2][b2] += r[a2] * r[b2]; }
    for (let a2 = 0; a2 < nb; a2++) AtA[a2][a2] += 1e-6;
    const W = [0, 1, 2].map(d => { const rhs = new Array(nb).fill(0); Phi.forEach((r, i) => { for (let a2 = 0; a2 < nb; a2++) rhs[a2] += r[a2] * F[d][i]; }); return gauss(AtA, rhs); });
    return { W, c, h, tau, L, name: demo.name, id: demo.id };
  }
  function dmpRoll(model, A, B, nOut = 1200) {
    const E = frameOf(sub(B, A)), L = nrm(sub(B, A)), tau = model.tau, gN = [1, 0, 0];
    const steps = 2400, Tend = 1.0 * tau, dt = Tend / steps;
    let y = [0, 0, 0], v = [0, 0, 0], t = 0; const Ys = [y.slice()], Ts = [0];
    for (let i = 0; i < steps; i++) {
      const s = Math.exp(-DMP.alpha * t / tau);
      let sw = 0; const ps = model.c.map((cb, b) => { const q = Math.exp(-model.h[b] * Math.pow(s - cb, 2)); sw += q; return q; });
      for (let d = 0; d < 3; d++) {
        let f = 0; for (let b = 0; b < DMP.nb; b++) f += ps[b] * model.W[d][b]; f = f * s / Math.max(sw, 1e-12);
        const vdot = (DMP.K * (gN[d] - y[d]) - DMP.D * v[d] - DMP.K * (gN[d]) * s + DMP.K * f) / tau;
        v[d] += vdot * dt; y[d] += v[d] / tau * dt;
      }
      t += dt; Ys.push(y.slice()); Ts.push(t);
    }
    // the DMP converges asymptotically; remove the small residual with a smooth ramp so the path ends exactly on B
    const res = sub(gN, Ys[Ys.length - 1]);
    const out = [], times = [];
    for (let k = 0; k <= nOut; k++) {
      const i = Math.round(k / nOut * steps), r = mj(k / nOut), yy = add(Ys[i], mul(res, r));
      out.push(add(A, mul(add(add(mul(E[0], yy[0]), mul(E[1], yy[1])), mul(E[2], yy[2])), L))); times.push(Ts[i]);
    }
    out[0] = A.slice(); out[nOut] = B.slice();
    return { pts: out, times, T: Ts[steps] };
  }

  // ---------------------------------------------------------------- time parameterisation
  function derivs(X) {      // dX/du and d²X/du² on a uniform u grid
    const N = X.length - 1, du = 1 / N, Xp = [], Xpp = [];
    for (let k = 0; k <= N; k++) {
      const a = X[Math.max(0, k - 1)], b = X[k], c = X[Math.min(N, k + 1)];
      const span = (k === 0 || k === N) ? du : 2 * du;
      Xp.push(c.map((v, j) => (v - a[j]) / span));
      Xpp.push(k === 0 || k === N ? null : c.map((v, j) => (v - 2 * b[j] + a[j]) / (du * du)));
    }
    Xpp[0] = Xpp[1].slice(); Xpp[N] = Xpp[N - 1].slice();
    return { Xp, Xpp };
  }
  // Time-optimal path parameterisation (reachability form, as in TOPP-RA).
  // State x = u̇² on a uniform u grid, control ü constant per segment. For every joint, the acceleration
  // q'ü + q''x must stay within ±A at BOTH ends of each segment, and |q'|u̇ ≤ V at every node.
  // Backward pass: K[k] = largest x at node k from which the rest of the path can still stop at u = 1.
  // Forward pass: from rest, take the largest feasible ü that keeps x inside K. The result is the fastest
  // timing for this path under these limits (up to grid resolution).
  function topp(Xp, Xpp, V, Acc) {
    const N = Xp.length - 1, du = 1 / N, nj = V.length;
    const rows = k => { const r = [];
      for (let j = 0; j < nj; j++) {
        r.push([Xp[k][j], Xpp[k][j], Acc[j]]);
        if (k < N) r.push([Xp[k + 1][j] + 2 * du * Xpp[k + 1][j], Xpp[k + 1][j], Acc[j]]);
      } return r; };
    const R = [], mvc = new Array(N + 1);
    for (let k = 0; k <= N; k++) {
      const r = rows(k); R.push(r);
      let m = Infinity; const L = [];
      for (let j = 0; j < nj; j++) if (Math.abs(Xp[k][j]) > 1e-9) m = Math.min(m, Math.pow(V[j] / Math.abs(Xp[k][j]), 2));
      for (const [a, b, A] of r) {
        if (Math.abs(a) > 1e-9) L.push([A / Math.abs(a), -b / a]);          // ü ∈ [−c + s·x, c + s·x]
        else if (Math.abs(b) > 1e-12) m = Math.min(m, A / Math.abs(b));
      }
      for (const [c1, s1] of L) for (const [c2, s2] of L) { const den = s1 - s2; if (den > 1e-12) m = Math.min(m, (c1 + c2) / den); }
      mvc[k] = m;
    }
    const lohi = (k, x) => { let lo = -Infinity, hi = Infinity;
      for (const [a, b, A] of R[k]) { if (Math.abs(a) <= 1e-9) continue;
        const c = A / Math.abs(a), s = -b / a; lo = Math.max(lo, -c + s * x); hi = Math.min(hi, c + s * x); }
      return [lo, hi]; };
    const K = new Array(N + 1); K[N] = 0;
    for (let k = N - 1; k >= 0; k--) {
      let m = mvc[k];
      for (const [a, b, A] of R[k]) { if (Math.abs(a) <= 1e-9) continue;
        const c = A / Math.abs(a), s = -b / a;
        const e1 = 1 + 2 * du * s;                                  // x + 2du·lo(x) ≤ K[k+1]
        if (e1 > 1e-12) m = Math.min(m, (K[k + 1] + 2 * du * c) / e1);
        if (e1 < 0) m = Math.min(m, 2 * du * c / -e1);               // x + 2du·hi(x) ≥ 0
      }
      K[k] = Math.max(0, m);
    }
    const x = new Array(N + 1).fill(0), sdd = [];
    for (let k = 0; k < N; k++) {
      const [lo, hi] = lohi(k, x[k]);
      let ud = Math.min(hi, (K[k + 1] - x[k]) / (2 * du)); if (ud < lo) ud = lo;
      x[k + 1] = Math.min(K[k + 1], Math.max(k + 1 < N ? 1e-10 : 0, x[k] + 2 * du * ud));
      sdd.push((x[k + 1] - x[k]) / (2 * du));
    }
    const t = [0], sd = x.map(v => Math.sqrt(Math.max(0, v)));
    for (let k = 0; k < N; k++) t.push(t[k] + 2 * du / Math.max(1e-9, sd[k] + sd[k + 1]));
    return { t, sd, sdd, T: t[N] };
  }
  // shortest duration for which a shape s(τ) keeps every joint within limits
  function minDuration(Xp, Xpp, V, Acc, shape) {
    let T = 0; const n = 600, N = Xp.length - 1;
    for (let i = 0; i <= n; i++) {
      const tau = i / n, u = shape.s(tau), sd = shape.sd(tau), sdd = shape.sdd(tau);
      const k = Math.min(N - 1, Math.floor(u * N)), f = u * N - k;
      for (let j = 0; j < V.length; j++) {
        const xp = Xp[k][j] + (Xp[k + 1][j] - Xp[k][j]) * f, xpp = Xpp[k][j] + (Xpp[k + 1][j] - Xpp[k][j]) * f;
        T = Math.max(T, Math.abs(xp) * sd / V[j], Math.sqrt(Math.abs(xp * sdd + xpp * sd * sd) / Acc[j]));
      }
    }
    return T;
  }
  const SHAPE = { smooth: { s: mj, sd: mjd, sdd: mjdd }, linear: { s: u => u, sd: () => 1, sdd: () => 0 } };

  // ---------------------------------------------------------------- the planner
  const DT = 1 / 200;
  function plan(task) {
    const t0 = (typeof performance !== 'undefined' ? performance : Date).now();
    const res = { task, ok: true, issues: [], notes: [], mode: task.path, timing: task.timing };
    const A = task.A.slice(), B = task.B.slice();
    const ws = (task.wristStart || [0, 0, 0, 0]).slice(), we0 = (task.wristEnd || [0, 0, 0, 0]).slice();
    const obstacles = task.obstacles || [], speed = task.speed || 1;
    const wristMoves = [0, 1, 2, 3].some(j => Math.abs(ws[j] - we0[j]) > 1e-6), gap = nrm(sub(A, B));
    if (gap < 1e-3 && !wristMoves) { res.ok = false; res.issues.push('the arm is already at B; set a different goal'); res.planMs = 0; return res; }
    if (task.path === 'learned' && gap < 5) { res.ok = false; res.issues.push('a learned path needs A and B at least 5 mm apart'); res.planMs = 0; return res; }
    const V = VLIM.map(v => v * 0.98 * speed), Acc = ALIM.map(a => a * 0.98 * speed * speed);
    // endpoint checks
    const rA = reach(A, [ws[0], ws[1], ws[2]]), rB = reach(B, [we0[0], we0[1], we0[2]]);
    if (!rA.ok) { res.ok = false; res.issues.push('Start A is out of reach: ' + rA.issues.join('; ')); }
    if (!rB.ok) {
      const straight = reach(B, [we0[0], 0, 0]);
      const interlockOnly = straight.ok && straight.x[2] < WQMIN + 3;      // only the wrist interlock objects: straighten it
      if (!interlockOnly) { res.ok = false; res.issues.push('Goal B is out of reach: ' + rB.issues.join('; ')); }
    }
    for (const o of obstacles) {
      if (clearance(A, [o]) < 0) { res.ok = false; res.issues.push('the shaft already passes through a keep-out zone at A'); }
      if (clearance(B, [o]) < 0) { res.ok = false; res.issues.push('the shaft would pass through a keep-out zone at B'); }
    }
    if (!res.ok) { res.planMs = (typeof performance !== 'undefined' ? performance : Date).now() - t0; return res; }
    // geometry
    const N = 400; let pathFn = null, demoT = null;
    if (task.path === 'straight') pathFn = u => lerp3(A, B, u);
    else if (task.path === 'avoid') { const av = avoidPath(A, B, obstacles); res.bezier = av.P; res.avoidStraight = !!av.straight; pathFn = av.fn; }
    else if (task.path === 'learned') {
      if (!task.demoModel) { res.ok = false; res.issues.push('no demonstration selected for the learned path'); return res; }
      const r = dmpRoll(task.demoModel, A, B); demoT = r.T; const pts = r.pts, n = pts.length - 1;
      pathFn = u => { const k = Math.min(n - 1, Math.floor(u * n)), f = u * n - k; return lerp3(pts[k], pts[k + 1], f); };
    }
    // wrist profile: start pose → end pose over the move (minimum-jerk in u)
    let we = we0.slice(), uStraight = null;
    const wristAt = u => {
      const out = [0, 0, 0, 0];
      out[0] = ws[0] + (we[0] - ws[0]) * mj(u);
      for (let j = 1; j < 4; j++) {
        if (uStraight !== null) { const r = uStraight > 1e-9 ? mj(Math.min(1, u / uStraight)) : 1; out[j] = ws[j] * (1 - r); }
        else out[j] = ws[j] + (we[j] - ws[j]) * mj(u);
      }
      return out;
    };
    const build = () => {
      const X = [], P = [];
      let seed = null;
      const xa = ikTip(A, wristAt(0)).x, xb = ikTip(B, wristAt(1)).x;
      for (let k = 0; k <= N; k++) {
        const u = k / N, w = wristAt(u); let x3, p;
        if (task.path === 'arc') { x3 = [0, 1, 2].map(j => xa[j] + (xb[j] - xa[j]) * u); p = tipRel([...x3, w[0], w[1], w[2]]); }
        else { p = pathFn(u); const s = ikTip(p, [w[0], w[1], w[2]], seed); x3 = s.x; seed = s.x; }
        X.push([...x3, ...w]); P.push(p);
      }
      return { X, P };
    };
    let G = build();
    // interlock: the wrist must be straight wherever insertion is below 55 mm
    const wristBent = ws.slice(1).some(v => Math.abs(v) > 1e-6) || we.slice(1).some(v => Math.abs(v) > 1e-6);
    const tooLow = X => X.findIndex(x => x[2] < WQMIN + 0.5 && (Math.abs(x[4]) > 1e-12 || Math.abs(x[5]) > 1e-12 || Math.abs(x[6]) > 1e-12));
    let kLow = wristBent ? tooLow(G.X) : -1;
    if (kLow >= 0) {
      we = [we[0], 0, 0, 0];
      for (let iter = 0; iter < 12 && kLow >= 0; iter++) { uStraight = Math.max(0, (kLow - 2) / N); G = build(); kLow = tooLow(G.X); }
      res.notes.push(uStraight > 0 ? 'The wrist straightens before the jaws enter the cannula zone (insertion < 55 mm).' : 'The wrist stays straight: the move starts inside the cannula zone.');
    }
    // joint limits along the whole path
    for (let k = 0; k <= N; k++) { const li = limitIssues(G.X[k]); if (li.length) { res.ok = false; res.issues.push(`path leaves the joint range at ${Math.round(k / N * 100)}% (${li[0]})`); break; } }
    if (obstacles.length) {
      let mc = Infinity, which = 0;
      for (const p of G.P) obstacles.forEach((o, i) => { const c = clearance(p, [o]); if (c < mc) { mc = c; which = i; } });
      res.minClear = mc;
      if (mc < 0) { res.ok = false;
        res.issues.push(task.path === 'avoid' ? `no collision-free route found (the best path still enters keep-out zone K${which + 1} by ${(-mc).toFixed(1)} mm)`
          : `the ${{ straight: 'straight line', arc: 'joint-space arc', learned: 'learned path' }[task.path]} passes through keep-out zone K${which + 1} (by ${(-mc).toFixed(1)} mm); choose "Avoid keep-out zones"`); }
    }
    if (!res.ok) { res.planMs = (typeof performance !== 'undefined' ? performance : Date).now() - t0; res.path = G; return res; }
    const { Xp, Xpp } = derivs(G.X);
    // timing law
    let law = task.timing, T, sample;
    if (task.path === 'learned' && law === 'demo') {
      const Tmin = minDuration(Xp, Xpp, V, Acc, SHAPE.linear); T = Math.max(demoT, Tmin);
      if (T > demoT * 1.001) res.notes.push(`Demonstration pace slowed to ${T.toFixed(2)} s to stay within joint limits.`);
      sample = t => { const u = clamp(t / T, 0, 1); return [u, 1 / T, 0]; };
    } else if (law === 'fastest') {
      const tp = topp(Xp, Xpp, V, Acc); T = tp.T;
      sample = t => { if (t >= T) return [1, 0, 0]; let lo = 0, hi = N; while (hi - lo > 1) { const m = (lo + hi) >> 1; if (tp.t[m] <= t) lo = m; else hi = m; }
        const tt = t - tp.t[lo], u = lo / N + tp.sd[lo] * tt + 0.5 * tp.sdd[lo] * tt * tt; return [clamp(u, lo / N, (lo + 1) / N), tp.sd[lo] + tp.sdd[lo] * tt, tp.sdd[lo]]; };
    } else {
      const Tmin = minDuration(Xp, Xpp, V, Acc, SHAPE.smooth);
      if (law === 'fixed') { T = Math.max(task.duration || 0, Tmin); if ((task.duration || 0) < Tmin - 1e-9) res.notes.push(`Requested ${(task.duration || 0).toFixed(2)} s is faster than the joints allow; using ${Tmin.toFixed(2)} s.`); }
      else T = Tmin;
      T = Math.max(T, 0.05);
      sample = t => { const tau = clamp(t / T, 0, 1); return [mj(tau), mjd(tau) / T, mjdd(tau) / (T * T)]; };
    }
    if (!(T > 0)) T = 0.05;
    // resample at 200 Hz with exact tool-point placement
    const n = Math.max(2, Math.ceil(T / DT) + 1), out = { t: [], X: [], XD: [], XDD: [], P: [] };
    let seed = null;
    for (let i = 0; i < n; i++) {
      const t = Math.min(i * DT, T), [u, sd, sdd] = sample(t);
      const k = Math.min(N - 1, Math.floor(u * N)), f = u * N - k;
      const xi = G.X[k].map((v, j) => v + (G.X[k + 1][j] - v) * f);
      const xp = Xp[k].map((v, j) => v + (Xp[k + 1][j] - v) * f), xpp = Xpp[k].map((v, j) => v + (Xpp[k + 1][j] - v) * f);
      let x = xi;
      if (task.path !== 'arc') { const w = wristAt(u), s = ikTip(pathFn(u), [w[0], w[1], w[2]], seed || xi); x = [...s.x, ...w]; seed = s.x; }
      else { const w = wristAt(u); x = [...xi.slice(0, 3), ...w]; }
      out.t.push(t); out.X.push(x); out.XD.push(xp.map(v => v * sd)); out.XDD.push(xp.map((v, j) => v * sdd + xpp[j] * sd * sd)); out.P.push(tipRel(x));
    }
    out.X[0] = G.X[0].slice(); out.X[n - 1] = G.X[N].slice(); out.P[0] = A.slice(); out.P[n - 1] = tipRel(out.X[n - 1]);
    out.XD[0] = out.XD[0].map(() => 0); out.XD[n - 1] = out.XD[n - 1].map(() => 0);
    res.traj = out; res.T = T; res.path = G; res.planMs = (typeof performance !== 'undefined' ? performance : Date).now() - t0;
    return res;
  }

  // ---------------------------------------------------------------- dynamics along a trajectory + servo execution
  function dynamics(tr) {
    const n = tr.t.length, step = Math.max(1, Math.floor(n / 60)), idx = [];
    for (let i = 0; i < n; i += step) idx.push(i); if (idx[idx.length - 1] !== n - 1) idx.push(n - 1);
    const MG = idx.map(i => massGrav(tr.X[i]));
    const at = i => { let a = 0; while (a < idx.length - 2 && idx[a + 1] < i) a++; const f = clamp((i - idx[a]) / Math.max(1, idx[a + 1] - idx[a]), 0, 1);
      const A = MG[a], B = MG[a + 1]; return { M: A.M.map((r, x) => r.map((v, y) => v + (B.M[x][y] - v) * f)), g: A.g.map((v, x) => v + (B.g[x] - v) * f) }; };
    const TAU = [], MM = [], GV = [];
    for (let i = 0; i < n; i++) {
      const { M, g } = at(i), a = [tr.XDD[i][0], tr.XDD[i][1], tr.XDD[i][2] * 1e-3];
      TAU.push([0, 1, 2].map(j => M[j][0] * a[0] + M[j][1] * a[1] + M[j][2] * a[2] + g[j])); MM.push(M); GV.push(g);
    }
    tr.TAU = TAU; tr.MM = MM; tr.G = GV;
  }
  function inv3(A) { const D = det3(A), c = (i, j) => { const r = [0, 1, 2].filter(x => x !== i), s = [0, 1, 2].filter(x => x !== j);
      return ((i + j) % 2 ? -1 : 1) * (A[r[0]][s[0]] * A[r[1]][s[1]] - A[r[0]][s[1]] * A[r[1]][s[0]]); };
    return [0, 1, 2].map(i => [0, 1, 2].map(j => c(j, i) / D)); }
  // Executes the trajectory on a simulated drive train: computed-torque PID + feedforward at 1 kHz.
  // Plant = coupled 3x3 mass matrix and gravity from the CAD parts. "Realistic" adds a mass and gravity model
  // error (the plant is 8-12 % heavier than the controller believes), Coulomb + viscous friction, and encoder quantisation.
  function servo(tr, realism, seed = 7) {
    const R = rng(seed), n = tr.t.length, dt = 0.001, nsub = Math.round(DT / dt), settle = 0.35;
    const wc = 2 * Math.PI * 14, mErr = realism ? 0.08 + 0.04 * R() : 0, gErr = realism ? 0.05 * (R() - 0.5) : 0, fr = realism ? 1 + 0.3 * (R() - 0.5) : 0;
    const enc = DRIVE.encoder, si = [1, 1, 1e-3];          // SI conversion (insertion in m)
    const q = [0, 1, 2].map(j => tr.X[0][j] * si[j]), qd = [0, 0, 0], ie = [0, 0, 0];
    const Xa = [], ERR = [], TAUA = [];
    const total = n + Math.round(settle / DT);
    let Minv = null, lastIi = -1;
    for (let i = 0; i < total; i++) {
      const ii = Math.min(i, n - 1), held = i >= n, i2 = Math.min(ii + 1, n - 1);
      const M = tr.MM[ii], g = tr.G[ii];
      if (ii !== lastIi) { Minv = inv3(M.map(r => r.map(v => v * (1 + mErr)))); lastIi = ii; }
      let tauOut = [0, 0, 0];
      for (let s = 0; s < nsub; s++) {
        const f = held ? 0 : s / nsub, u = [0, 0, 0];
        for (let j = 0; j < 3; j++) {
          const ref = (tr.X[ii][j] + (tr.X[i2][j] - tr.X[ii][j]) * f) * si[j];
          const refd = held ? 0 : (tr.XD[ii][j] + (tr.XD[i2][j] - tr.XD[ii][j]) * f) * si[j];
          const refdd = held ? 0 : (tr.XDD[ii][j] + (tr.XDD[i2][j] - tr.XDD[ii][j]) * f) * si[j];
          const meas = realism ? Math.round(q[j] / (enc[j] * si[j])) * enc[j] * si[j] : q[j];
          const e = ref - meas, ed = refd - qd[j];
          ie[j] += e * dt;
          u[j] = refdd + wc * wc * e + 2 * wc * ed + wc * wc * (wc / 6) * ie[j];
        }
        const tau = [0, 1, 2].map(j => clamp(M[j][0] * u[0] + M[j][1] * u[1] + M[j][2] * u[2] + g[j], -DRIVE.peak[j], DRIVE.peak[j]));
        const net = [0, 1, 2].map(j => tau[j] - g[j] * (1 + gErr) - (realism ? fr * (DRIVE.coulomb[j] * Math.tanh(qd[j] / 1e-3) + DRIVE.viscous[j] * qd[j]) : 0));
        for (let j = 0; j < 3; j++) { const qdd = Minv[j][0] * net[0] + Minv[j][1] * net[1] + Minv[j][2] * net[2]; qd[j] += qdd * dt; }
        for (let j = 0; j < 3; j++) q[j] += qd[j] * dt;
        tauOut = tau;
      }
      const xa = [q[0], q[1], q[2] / si[2], ...tr.X[ii].slice(3)];
      Xa.push(xa); ERR.push(nrm(sub(tipRel(xa), tr.P[ii]))); TAUA.push(tauOut);
    }
    tr.Xa = Xa; tr.ERR = ERR; tr.TAUA = TAUA; tr.settleN = total - n; tr.realism = !!realism;
    return tr;
  }

  // ---------------------------------------------------------------- metrics and pass/fail
  function evaluate(res, realism) {
    if (!res.ok || !res.traj) return { pass: false, reasons: res.issues.slice(), metrics: null };
    const tr = res.traj; dynamics(tr); servo(tr, realism, res.task.seed || 7);
    const n = tr.t.length, ob = res.task.obstacles || [];
    let vpk = 0, apk = 0, vj = 0, aj = 0, len = 0, track = 0, dev = 0, clr = Infinity, rcm = 0;
    const tq = [0, 0, 0], rms = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < 7; j++) { const v = Math.abs(tr.XD[i][j]) / VLIM[j], a = Math.abs(tr.XDD[i][j]) / ALIM[j]; if (v > vpk) { vpk = v; vj = j; } if (a > apk) { apk = a; aj = j; } }
      if (i) len += nrm(sub(tr.P[i], tr.P[i - 1]));
      for (let j = 0; j < 3; j++) { tq[j] = Math.max(tq[j], Math.abs(tr.TAU[i][j])); rms[j] += tr.TAU[i][j] * tr.TAU[i][j]; }
      track = Math.max(track, tr.ERR[i]);
      const pa = tipRel(tr.Xa[i]); clr = Math.min(clr, clearance(pa, ob));
      if (res.task.path === 'straight') { const A = res.task.A, d = sub(res.task.B, A), L = nrm(d), w = sub(pa, A), s = clamp(dot(w, d) / (L * L), 0, 1); dev = Math.max(dev, nrm(sub(w, mul(d, s)))); }
      if (i % 25 === 0) rcm = Math.max(rcm, rcmError(tr.Xa[i]));
    }
    const endAct = tipRel(tr.Xa[tr.Xa.length - 1]), endErr = nrm(sub(endAct, res.task.B)), endRef = nrm(sub(tr.P[n - 1], res.task.B));
    const m = {
      duration: res.T, length: len, planMs: res.planMs, peakSpeed: vpk, peakSpeedJoint: JN[vj], peakAcc: apk, peakAccJoint: JN[aj],
      torque: tq, torquePct: tq.map((v, j) => v / DRIVE.rated[j]), rms: rms.map((v, j) => Math.sqrt(v / n)),
      worstTorquePct: Math.max(...tq.map((v, j) => v / DRIVE.rated[j])), track, endErr, endRef, pathDev: dev, clearance: clr, rcm
    };
    const reasons = [];
    if (vpk > 1.005) reasons.push(`${m.peakSpeedJoint} speed ${(vpk * 100).toFixed(0)}% of limit`);
    if (apk > 1.02) reasons.push(`${m.peakAccJoint} acceleration ${(apk * 100).toFixed(0)}% of limit`);
    tq.forEach((v, j) => { if (v > DRIVE.peak[j]) reasons.push(`${DRIVE.name[j]} load ${v.toFixed(1)} ${DRIVE.unit[j]} above the ${DRIVE.peak[j]} ${DRIVE.unit[j]} peak`); });
    if (ob.length && clr < 0) reasons.push(`shaft enters a keep-out zone by ${(-clr).toFixed(1)} mm`);
    if (endErr > 0.05) reasons.push(`final position off by ${(endErr * 1000).toFixed(0)} µm`);
    if (rcm > 1e-6) reasons.push(`remote center moved ${rcm.toExponential(1)} mm`);
    const warnings = [];
    m.rms.forEach((v, j) => { if (v > DRIVE.motorCont[j]) warnings.push(`${DRIVE.name[j]} RMS load ${v.toFixed(1)} ${DRIVE.unit[j]} is above the motor's continuous rating (${DRIVE.motorCont[j]} ${DRIVE.unit[j]})`); });
    if (ob.length && clr >= 0 && clr < 3) warnings.push(`only ${clr.toFixed(1)} mm clearance to a keep-out zone`);
    return { pass: reasons.length === 0, reasons, warnings, metrics: m };
  }

  // ---------------------------------------------------------------- test suites
  function randomPose(R, margin = 0.08) {
    const x = [0, 1, 2].map(j => LO[j] + (HI[j] - LO[j]) * (margin + (1 - 2 * margin) * R()));
    return tipRel([...x, 0, 0, 0]);
  }
  function randomTasks(nTasks, seed, obstacles) {
    const R = rng(seed), out = [];
    let guard = 0;
    while (out.length < nTasks && guard++ < nTasks * 50) {
      const A = randomPose(R), B = randomPose(R);
      if (nrm(sub(A, B)) < 20) continue;
      if (obstacles.some(o => clearance(A, [o]) < 3 || clearance(B, [o]) < 3)) continue;
      out.push({ A, B });
    }
    return out;
  }
  function sweepTasks(A, depths = [80, 120, 160]) {
    const out = [];
    for (const dpt of depths) for (let a = 0; a < 7; a++) for (let b = 0; b < 6; b++) {
      const yaw = LO[0] + (HI[0] - LO[0]) * (0.04 + 0.92 * a / 6), th = LO[1] + (HI[1] - LO[1]) * (0.04 + 0.92 * b / 5);
      const B = tipRel([yaw, th, dpt - LT, 0, 0, 0]); if (nrm(sub(A, B)) > 5) out.push({ A, B });
    }
    return out;
  }

  return { JN, UNIT, LO, HI, VLIM, ALIM, WQMIN, LT, DRIVE, B3, R3, DT, BODIES, derivs, topp, frameOf,
    tipRel, shaftDir, ikStraight, ikTip, reach, snap, limitIssues, bodyT, rcmError, massGrav, clearance, avoidPath,
    dmpFit, dmpRoll, plan, dynamics, servo, evaluate, randomTasks, sweepTasks, rng, mj, nrm, sub, add, mul, dot, lerp3 };
}
root.RCM7Core = { createCore };
if (typeof module !== 'undefined') module.exports = { createCore };
})(typeof window !== 'undefined' ? window : globalThis);
