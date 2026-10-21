// Headless checks of the Motion Lab engine against the CAD simulator's own kinematics.
//   node lab/test/test_core.js        (run from generator/, after lab/lab_payload.py)
const fs = require('fs'), path = require('path');
require('./mini3.js');
const G = path.join(__dirname, '..', '..');
const D = JSON.parse(fs.readFileSync(path.join(G, 'out', 'lab_payload.json'), 'utf8'));
const { createCore } = require('../planner_core.js');
const C = createCore(D.meta, D.parts);
const MT = D.meta, D2R = Math.PI / 180;
// reference: the CAD simulator's kin(), lifted verbatim out of sim_template.html
const tpl = fs.readFileSync(path.join(G, 'sim_template.html'), 'utf8');
const pre = tpl.slice(tpl.indexOf('const beta = MT.beta'), tpl.indexOf('// ---------------- renderer'));
const kinSrc = tpl.slice(tpl.indexOf('const M4 = () =>'), tpl.indexOf('const wristPoint'));
const ref = new Function('MT', 'THREE', 'D2R', `const V3=(x,y,z)=>new THREE.Vector3(x,y,z); ${pre} ${kinSrc}; return {kin, R3, J0};`)(MT, THREE, D2R);
let fails = 0; const T0 = Date.now();
const check = (name, ok, detail) => { console.log((ok ? 'PASS ' : 'FAIL ') + name.padEnd(58) + detail); if (!ok) fails++; };
const R = C.rng(42), rnd = (a, b) => a + (b - a) * R();
// legal states only: the interlock keeps the wrist straight below 55 mm insertion
const randX = () => { const x = [rnd(C.LO[0], C.HI[0]), rnd(C.LO[1], C.HI[1]), rnd(C.LO[2], C.HI[2]), rnd(-3, 3), rnd(-1.3, 1.3), rnd(-1.3, 1.3), rnd(0, 1)]; if (x[2] < C.WQMIN) { x[4] = x[5] = x[6] = 0; } return x; };
const toDeg = x => ({ yaw: x[0] / D2R, theta: x[1] / D2R, q: x[2], roll: x[3] / D2R, psi: x[4] / D2R, chi: x[5] / D2R, grip: x[6] / D2R });

// 1 forward kinematics: closed-form tool point vs CAD kinematic chain (jaw midline tip)
let e1 = 0, e2 = 0;
for (let i = 0; i < 2000; i++) {
  const x = randX(), T = ref.kin(toDeg(x));
  const mid = T.wrist.clone().multiply(new THREE.Matrix4().makeTranslation(0, 0, 0)); // wrist frame
  const Jw = new THREE.Vector3(...MT.J0); const tipLocal = new THREE.Vector3(0, MT.J0[1] - 13.5 * Math.cos(x[5]), MT.J0[2] - 13.5 * Math.sin(x[5]));
  const tip = tipLocal.applyMatrix4(mid).sub(ref.R3);
  const p = C.tipRel(x); e1 = Math.max(e1, Math.hypot(p[0] - tip.x, p[1] - tip.y, p[2] - tip.z));
  const Tc = C.bodyT(x); for (const k of ['yaw', 'link1', 'coupler2', 'spar', 'instrument', 'jaw2']) for (let r = 0; r < 4; r++) for (let c = 0; c < 4; c++)
    e2 = Math.max(e2, Math.abs(Tc[k][r * 4 + c] - T[k].elements[c * 4 + r]));
}
check('FK: tool point vs CAD chain, 2000 random 7-DOF poses', e1 < 1e-9, e1.toExponential(2) + ' mm');
check('FK: every body transform vs CAD simulator', e2 < 1e-9, e2.toExponential(2));
// 2 inverse kinematics
let ie = 0, iw = 0, maxIt = 0;
for (let i = 0; i < 2000; i++) {
  const x = randX(), wr = [x[3], x[4], x[5]], p = C.tipRel(x);
  const s = C.ikTip(p, wr); maxIt = Math.max(maxIt, s.it);
  ie = Math.max(ie, s.err); iw = Math.max(iw, Math.abs(s.x[0] - x[0]), Math.abs(s.x[1] - x[1]), Math.abs(s.x[2] - x[2]) / 100);
}
check('IK: tool-point residual, 2000 poses with articulated wrist', ie < 1e-9, ie.toExponential(2) + ' mm, ≤' + maxIt + ' Newton steps');
check('IK: recovers the original joints', iw < 1e-9, iw.toExponential(2));
let rcm = 0; for (let i = 0; i < 300; i++) rcm = Math.max(rcm, C.rcmError(randX()));
check('RCM: shaft distance from R over 300 poses', rcm < 1e-9, rcm.toExponential(2) + ' mm');
// 3 mass properties
const totM = Object.values(C.BODIES).reduce((s, b) => s + b.m, 0);
check('Mass model: moving mass', Math.abs(totM - MT.report.bom.total_mass_kg) < 0.05, totM.toFixed(2) + ' kg (BOM ' + MT.report.bom.total_mass_kg.toFixed(2) + ')');
const mg = C.massGrav([0, 0, 100, 0, 0, 0, 0]);
const sym = Math.max(Math.abs(mg.M[0][1] - mg.M[1][0]), Math.abs(mg.M[0][2] - mg.M[2][0]), Math.abs(mg.M[1][2] - mg.M[2][1]));
const ev = mg.M[0][0] > 0 && mg.M[0][0] * mg.M[1][1] - mg.M[0][1] ** 2 > 0;
check('Mass matrix symmetric and positive definite', sym < 1e-6 && ev, `diag ${mg.M.map((r, i) => r[i].toFixed(3)).join(', ')}`);
// gravity vs the counterbalance report: pitch torque with spring stays within the report's worst case
let gp = 0, gy = 0; for (let i = 0; i < 400; i++) { const x = randX(); const g = C.massGrav(x).g; gp = Math.max(gp, Math.abs(g[1])); gy = Math.max(gy, Math.abs(g[0])); }
const L = MT.report.loads;
check('Gravity: pitch (counterbalanced) within report', gp <= L.pitch_with_spring_max_Nm * 1.05, gp.toFixed(2) + ' N·m (report ' + L.pitch_with_spring_max_Nm.toFixed(2) + ')');
check('Gravity: yaw within report', gy <= L.yaw_gravity_max_Nm * 1.05, gy.toFixed(2) + ' N·m (report ' + L.yaw_gravity_max_Nm.toFixed(2) + ')');

// 4 planning: one task per path/timing mode
const A = C.tipRel([0, 0, 100, 0, 0, 0, 0]), B = [45, -100, 30], OB = [{ c: [22, -112, 15], r: 9 }];
const modes = [['straight', 'smooth'], ['straight', 'fastest'], ['arc', 'smooth'], ['arc', 'fastest'], ['avoid', 'smooth'], ['avoid', 'fastest'], ['straight', 'fixed']];
for (const [pth, tim] of modes) {
  const task = { A, B, path: pth, timing: tim, duration: 1.0, obstacles: pth === 'avoid' ? OB : [], wristStart: [0, 0, 0, 0], wristEnd: [0, 0.3, -0.2, 0.4] };
  const res = C.plan(task); const ev = C.evaluate(res, true); const m = ev.metrics;
  check(`Plan ${pth}/${tim}`, ev.pass && (pth !== 'avoid' || m.clearance >= 0), m ? `${m.duration.toFixed(2)} s, peak speed ${(m.peakSpeed * 100).toFixed(0)}% (${m.peakSpeedJoint}), acc ${(m.peakAcc * 100).toFixed(0)}% (${m.peakAccJoint}), end ${(m.endErr * 1000).toFixed(1)} µm, track ${(m.track * 1000).toFixed(0)} µm${pth === 'avoid' ? ', clear ' + m.clearance.toFixed(1) + ' mm' : ''}${pth === 'straight' ? ', line dev ' + (m.pathDev * 1000).toFixed(0) + ' µm' : ''}, plan ${m.planMs.toFixed(0)} ms` : ev.reasons.join('; '));
}
// straight line with ideal servo should be exact
{ const res = C.plan({ A, B, path: 'straight', timing: 'smooth', obstacles: [] }); const ev = C.evaluate(res, false);
  check('Straight line, ideal servo: deviation from the line', ev.metrics.pathDev < 0.01, (ev.metrics.pathDev * 1000).toFixed(2) + ' µm');
  const tr = res.traj; let dev = 0; for (const p of tr.P) { const d = C.sub(B, A), w = C.sub(p, A), s = C.dot(w, d) / C.dot(d, d); dev = Math.max(dev, C.nrm(C.sub(w, C.mul(d, s)))); }
  check('Straight line: planned points on the line', dev < 1e-8, dev.toExponential(2) + ' mm'); }
// TOPP is faster than smooth and respects limits
{ const a = C.plan({ A, B, path: 'straight', timing: 'fastest', obstacles: [] }), b = C.plan({ A, B, path: 'straight', timing: 'smooth', obstacles: [] });
  check('Fastest is faster than smooth', a.T < b.T, `${a.T.toFixed(3)} s vs ${b.T.toFixed(3)} s`); }
// infeasible requests are reported, not executed
{ const r = C.plan({ A, B: [0, -400, 0], path: 'straight', timing: 'smooth', obstacles: [] }); check('Out-of-reach goal rejected', !r.ok, r.issues[0]); }
{ const r = C.plan({ A, B, path: 'straight', timing: 'smooth', obstacles: [{ c: B, r: 5 }] }); check('Goal inside keep-out rejected', !r.ok, r.issues[0]); }
{ const r = C.plan({ A, B, path: 'straight', timing: 'fixed', duration: 0.05, obstacles: [] }); check('Too-short duration is stretched', r.ok && r.T > 0.05, r.notes[0]); }
// interlock: wrist bent at start, goal shallow
{ const xs = [0.2, 0.1, 90, 0, 0.6, -0.4, 0.5], As = C.tipRel(xs), Bs = C.tipRel([0.1, -0.2, 30, 0, 0, 0, 0]);
  const r = C.plan({ A: As, B: Bs, path: 'straight', timing: 'smooth', obstacles: [], wristStart: [0, 0.6, -0.4, 0.5], wristEnd: [0, 0.6, -0.4, 0.5] });
  let bad = 0; if (r.traj) r.traj.X.forEach(x => { if (x[2] < 55 && (Math.abs(x[4]) > 1e-9 || Math.abs(x[5]) > 1e-9 || Math.abs(x[6]) > 1e-9)) bad++; });
  check('Interlock: wrist straight whenever insertion < 55 mm', r.ok && bad === 0, (r.notes[0] || '') + ` (${bad} bad samples)`);
  const e = C.evaluate(r, false); check('Interlock move ends on B', e.pass, (e.metrics.endErr * 1000).toFixed(2) + ' µm'); }
// learned path from a synthetic demonstration (a scoop: dips 25 mm deeper mid-way)
{ const dA = [-30, -120, -10], dB = [30, -110, 10], demo = { p: [], t: [] };
  for (let i = 0; i <= 150; i++) { const u = C.mj(i / 150), p = C.lerp3(dA, dB, u); p[1] -= 25 * Math.sin(Math.PI * u); demo.p.push(p); demo.t.push(i / 150 * 2.0); }
  const model = C.dmpFit(demo); const roll = C.dmpRoll(model, dA, dB);
  let dev = 0; for (let k = 0; k <= 50; k++) { const u = k / 50; const i = Math.round(u * (roll.pts.length - 1)), j = Math.round(u * 150); dev = Math.max(dev, C.nrm(C.sub(roll.pts[i], demo.p[j]))); }
  check('DMP reproduces its own demonstration', dev < 2.0, dev.toFixed(2) + ' mm max deviation');
  const nA = A, nB = B, r = C.plan({ A: nA, B: nB, path: 'learned', timing: 'demo', demoModel: model, obstacles: [] });
  const e = C.evaluate(r, true); let mid = r.traj.P[Math.floor(r.traj.P.length / 2)], line = C.lerp3(nA, nB, 0.5);
  check('DMP generalises to new A→B and ends exactly on B', e.pass, `end ${(e.metrics.endErr * 1000).toFixed(1)} µm, mid-path dip ${(line[1] - mid[1]).toFixed(1)} mm below the straight line, ${e.metrics.duration.toFixed(2)} s`); }
// random test suite
{ const tasks = C.randomTasks(40, 9, OB); let pass = 0, worstEnd = 0, worstV = 0, worstA = 0, worstClr = Infinity, infeasible = 0; const why = {};
  for (const t of tasks) { const r = C.plan({ ...t, path: 'avoid', timing: 'fastest', obstacles: OB }); if (!r.ok) { infeasible++; why[r.issues[0].slice(0, 40)] = 1; continue; }
    const e = C.evaluate(r, true); if (e.pass) pass++; else why[e.reasons[0]] = 1; worstEnd = Math.max(worstEnd, e.metrics.endErr); worstV = Math.max(worstV, e.metrics.peakSpeed); worstA = Math.max(worstA, e.metrics.peakAcc); worstClr = Math.min(worstClr, e.metrics.clearance); }
  check('Random suite: 40 avoid/fastest moves', pass + infeasible === tasks.length && pass >= 34, `${pass} pass, ${infeasible} infeasible; worst end ${(worstEnd * 1000).toFixed(1)} µm, speed ${(worstV * 100).toFixed(0)}%, acc ${(worstA * 100).toFixed(0)}%, clearance ${worstClr.toFixed(1)} mm  ${Object.keys(why).join(' | ')}`); }
console.log(`\n${fails ? fails + ' FAILED' : 'all passed'} in ${((Date.now() - T0) / 1000).toFixed(1)} s`);
process.exit(fails ? 1 : 0);
