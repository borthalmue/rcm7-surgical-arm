"""Headless Chromium test of the Motion Lab page (three.js replaced by a stand-in, so no WebGL drawing).
   python3 lab/test/test_page.py out/rcm7_motionlab.html [shots_dir]"""
import json, os, sys, time
from playwright.sync_api import sync_playwright

page_path = os.path.abspath(sys.argv[1]); shots = sys.argv[2] if len(sys.argv) > 2 else None
stub = open(os.path.join(os.path.dirname(__file__), 'three_stub.js')).read()
fails = []
def check(name, ok, detail=''):
    print(('PASS ' if ok else 'FAIL ') + name.ljust(56) + str(detail)); (None if ok else fails.append(name))

with sync_playwright() as p:
    b = p.chromium.launch(executable_path='/opt/pw-browsers/chromium-1194/chrome-linux/chrome' if os.path.exists('/opt/pw-browsers/chromium-1194/chrome-linux/chrome') else None)
    def open_page(w, h, scheme='light'):
        ctx = b.new_context(viewport={'width': w, 'height': h}, color_scheme=scheme)
        pg = ctx.new_page(); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' and 'Failed to load resource' not in m.text else None)
        pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.route('**/three.min.js', lambda r: r.fulfill(status=200, content_type='application/javascript', body=stub))
        pg.route('**/OrbitControls.js', lambda r: r.fulfill(status=200, content_type='application/javascript', body='/* stub */'))
        pg.route('**/fonts.googleapis.com/**', lambda r: r.abort()); pg.route('**/fonts.gstatic.com/**', lambda r: r.abort())
        pg.goto('file://' + page_path); pg.wait_for_function('window.__lab && window.__lab.plan', timeout=20000)
        return pg, errs
    pg, errs = open_page(1440, 1000)
    L = 'window.__lab'
    st = pg.evaluate(f"(() => {{ const e = {L}.ev; return {{ok:{L}.plan.ok, pass:e.pass, reasons:e.reasons, T:{L}.plan.T, paths:document.querySelectorAll('#charts path.ln').length, pill:document.getElementById('res-pill').textContent, head:document.getElementById('res-head').textContent }}; }})()")
    check('Boot: default plan (avoid, smooth) passes', st['ok'] and st['pass'], f"{st['head']} · {st['pill']} {st['reasons']}")
    check('Boot: charts drawn', st['paths'] == 13, f"{st['paths']} series lines")
    pg.wait_for_function(f"{L}.rows.length >= 12 && !{L}.suite", timeout=60000)
    rows = pg.evaluate(f"{L}.rows.map(r => [r.pass, r.ok, (r.reasons||[])[0] || ''])")
    check('Boot: sample suite of 12 moves ran', len(rows) == 12, f"{sum(1 for r in rows if r[0])} pass, {sum(1 for r in rows if not r[1])} infeasible; {[r[2] for r in rows if not r[0]][:2]}")
    check('Test log table filled', pg.evaluate("document.querySelectorAll('#t-body tr').length") == 12)
    # run the default move
    pg.evaluate(f"{L}.run()"); pg.wait_for_function(f"!{L}.running", timeout=20000)
    fin = pg.evaluate(f"(() => {{ const p = {L}.C.tipRel({L}.ST.x), B = {L}.ST.B; return {{err: {L}.C.nrm({L}.C.sub(p, B)), state: document.getElementById('state').textContent, r: document.getElementById('r-x').textContent}}; }})()")
    check('Run: arm arrives on B', fin['err'] < 0.05, f"{fin['err']*1000:.2f} µm · '{fin['state']}' · readout X {fin['r']}")
    # command line
    def cmd(c): pg.evaluate(f"{L}.cmd({json.dumps(c)})")
    cmd('go 30 -110 20'); pg.wait_for_function(f"!{L}.running", timeout=20000)
    tip = pg.evaluate(f"{L}.C.tipRel({L}.ST.x)")
    check('Command go 30 -110 20', max(abs(a-b) for a, b in zip(tip, [30, -110, 20])) < 0.05, [round(v, 4) for v in tip])
    cmd('B -40 -95 -20'); cmd('path straight'); cmd('zone clear'); cmd('time fastest'); cmd('plan')
    s = pg.evaluate(f"({{ok:{L}.plan.ok, pass:{L}.ev.pass, T:{L}.plan.T, path:{L}.ST.path, timing:{L}.ST.timing}})")
    check('Commands: B, path, zone clear, time, plan', s['ok'] and s['pass'] and s['path'] == 'straight' and s['timing'] == 'fastest', s)
    cmd('zone add -5 -103 0 6'); s = pg.evaluate(f"({{ok:{L}.plan.ok, issue:({L}.ev.reasons||[])[0]}})")
    check('Straight line through a zone is refused', not s['ok'], s['issue'])
    cmd('path avoid'); s = pg.evaluate(f"({{ok:{L}.plan.ok, pass:{L}.ev.pass, clr:{L}.ev.metrics && {L}.ev.metrics.clearance}})")
    check('Avoid finds a way round it', s['ok'] and s['pass'] and s['clr'] >= 0, f"clearance {s['clr']:.2f} mm" if s['clr'] is not None else s)
    cmd('get tip'); cmd('bogus'); log = pg.evaluate("document.getElementById('log').textContent")
    check('Console answers and reports errors', 'tip = ' in log and 'unknown command' in log)
    # joint-mode input
    pg.click('#m-jnt'); pg.fill('#b-in-0', '0.3'); pg.fill('#b-in-1', '-0.2'); pg.fill('#b-in-2', '120'); pg.press('#b-in-2', 'Tab'); pg.wait_for_timeout(400)
    s = pg.evaluate(f"(() => {{ const j = {L}.C.ikTip({L}.ST.B, [0,0,0]).x; return {{j, conv: document.getElementById('b-conv').textContent, labels: [...document.querySelectorAll('#b-in label span')].map(e => e.textContent)}}; }})()")
    check('Joint mode: B from yaw/pitch/insertion', abs(s['j'][0]-0.3) < 1e-9 and abs(s['j'][1]+0.2) < 1e-9 and abs(s['j'][2]-120) < 1e-7, f"{s['labels']} {s['conv']}")
    pg.click('#m-xyz')
    # JSON round trip
    pg.evaluate(f"{L}.applyJSON({{B:[10,-130,-15], path:'arc', timing:'fixed', duration:3, obstacles:[]}}); {L}.replan()")
    s = pg.evaluate(f"({{T:{L}.plan.T, pass:{L}.ev.pass, json:{L}.taskJSON()}})")
    check('JSON task: arc, fixed 3 s', abs(s['T'] - 3) < 1e-6 and s['pass'], f"T = {s['T']:.3f} s")
    csv = pg.evaluate(f"{L}.trajectoryCSV()").split('\n')
    check('Trajectory CSV at 200 Hz', len(csv) - 1 == round(3 / 0.005) + 1 and csv[0].startswith('t_s,x_mm'), f"{len(csv)-1} rows, {len(csv[0].split(','))} columns")
    # jog + record a demonstration
    pg.evaluate(f"{L}.ST.aFromArm = true"); t0 = pg.evaluate(f"{L}.C.tipRel({L}.ST.x)")
    pg.evaluate(f"{L}.rec()"); pg.evaluate(f"{L}.jog('x+')"); pg.wait_for_timeout(1200); pg.evaluate(f"{L}.jog('y-')"); pg.wait_for_timeout(900)
    pg.evaluate(f"{L}.jog('x+')"); pg.wait_for_timeout(1000); pg.evaluate(f"{L}.jog(null)"); pg.wait_for_timeout(600); pg.evaluate(f"{L}.rec()"); pg.wait_for_timeout(500)
    t1 = pg.evaluate(f"{L}.C.tipRel({L}.ST.x)"); s = pg.evaluate(f"({{n:{L}.DEMOS.length, path:{L}.ST.path, name:{L}.DEMOS[{L}.ST.demo].name, msg:document.getElementById('rec-msg').textContent}})")
    check('Jog moved the tip (+X then −Y then +X)', t1[0] - t0[0] > 25 and t1[1] - t0[1] < -8 and abs(t1[2] - t0[2]) < 0.01, [round(a-b, 2) for a, b in zip(t1, t0)])
    check('Recorded demo saved and selected', s['n'] == 5 and s['path'] == 'learned', s['msg'])
    pg.evaluate(f"{L}.applyJSON({{B:[35,-95,25], timing:'demo'}}); {L}.ST.aFromArm = false; {L}.ST.A = [-35,-120,-10]; {L}.replan()")
    s = pg.evaluate(f"({{ok:{L}.plan.ok, pass:{L}.ev.pass, r:{L}.ev.reasons, T:{L}.plan.T}})")
    check('Learned path from my demo, new A and B', s['ok'] and s['pass'], f"{s['T']:.2f} s {s['r']}")
    # suites
    pg.evaluate(f"{L}.applyJSON({{path:'avoid', timing:'smooth', obstacles:[{{c:[0,-110,0], r:10}}]}}); {L}.replan(); {L}.runSuite('repeat', 6)")
    pg.wait_for_function(f"!{L}.suite", timeout=60000)
    s = pg.evaluate("document.getElementById('t-dl').textContent")
    check('Repeatability suite', 'Spread of final positions' in s, s[:160])
    pg.evaluate(f"{L}.runSuite('sweep', 0)"); pg.wait_for_function(f"!{L}.suite", timeout=120000)
    s = pg.evaluate(f"({{n:{L}.rows.length, pass:{L}.rows.filter(r => r.pass).length, inf:{L}.rows.filter(r => !r.ok).length, fail:{L}.rows.filter(r => r.ok && !r.pass).map(r => r.reasons[0]).slice(0,3)}})")
    check('Workspace sweep', s['n'] == 126 and s['pass'] + s['inf'] == s['n'], s)
    pg.evaluate(f"{L}.runSuite('stress', 15)"); pg.wait_for_function(f"!{L}.suite", timeout=60000)
    s = pg.evaluate(f"({{n:{L}.rows.length, pass:{L}.rows.filter(r => r.pass).length, inf:{L}.rows.filter(r => !r.ok).length, warn:{L}.rows.reduce((a, r) => a + (r.warnings||[]).length, 0), w:({L}.rows.find(r => r.warnings && r.warnings.length)||{{warnings:['']}}).warnings[0]}})")
    check('Stress suite (fastest, 100 %)', s['pass'] + s['inf'] == s['n'], s)
    check('No console errors', not errs, errs[:3])
    if shots:
        os.makedirs(shots, exist_ok=True)
        pg.evaluate(f"{L}.applyJSON({{startFromArm:false, A:[0,-122.5,0], B:[45,-100,30], path:'avoid', timing:'smooth', obstacles:[{{c:[22,-112,15], r:9}}]}}); {L}.replan(); {L}.setPreview(0.6, null)")
        pg.wait_for_timeout(300); pg.screenshot(path=os.path.join(shots, 'desktop.png'), full_page=True)
    # phone width + dark
    for w, scheme in [(400, 'dark'), (390, 'light')]:
        pg2, errs2 = open_page(w, 860, scheme); pg2.wait_for_timeout(800)
        sw = pg2.evaluate("document.documentElement.scrollWidth")
        check(f'No horizontal scroll at {w}px ({scheme})', sw <= w, f"scrollWidth {sw}")
        check(f'No console errors at {w}px', not errs2, errs2[:2])
        if shots: pg2.screenshot(path=os.path.join(shots, f'phone_{scheme}.png'), full_page=True)
    b.close()
print('\n' + (f'{len(fails)} FAILED: {fails}' if fails else 'all passed')); sys.exit(1 if fails else 0)
