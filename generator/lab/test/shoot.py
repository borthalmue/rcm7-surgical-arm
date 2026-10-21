"""Real-WebGL screenshots (SwiftShader) of the Motion Lab with local three.js r128 files.
   python3 lab/test/shoot.py out/rcm7_motionlab.html THREE_DIR OUT_DIR"""
import os, sys
from playwright.sync_api import sync_playwright
page, tdir, out = os.path.abspath(sys.argv[1]), sys.argv[2], sys.argv[3]; os.makedirs(out, exist_ok=True)
three = open(os.path.join(tdir, 'build/three.min.js')).read(); orbit = open(os.path.join(tdir, 'examples/js/controls/OrbitControls.js')).read()
with sync_playwright() as p:
    b = p.chromium.launch(args=['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'])
    def go(w, h, scheme, name, js=None, wait=1500, full=False):
        ctx = b.new_context(viewport={'width': w, 'height': h}, color_scheme=scheme, device_scale_factor=1)
        pg = ctx.new_page(); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' and 'Failed to load resource' not in m.text else None)
        pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.route('**/three.min.js', lambda r: r.fulfill(status=200, content_type='application/javascript', body=three))
        pg.route('**/OrbitControls.js', lambda r: r.fulfill(status=200, content_type='application/javascript', body=orbit))
        pg.route('**/fonts.googleapis.com/**', lambda r: r.abort())
        pg.goto('file://' + page); pg.wait_for_function('window.__lab && window.__lab.plan', timeout=30000)
        if js: pg.evaluate(js)
        pg.wait_for_timeout(wait)
        pg.screenshot(path=os.path.join(out, name), full_page=full)
        print(name, 'errors:', errs[:3]); ctx.close()
    go(1440, 900, 'light', 'w_desktop.png', "window.__lab.setPreview(0.62, null)")
    go(1440, 900, 'dark', 'w_dark_front.png', "document.getElementById('viewsel').value='front'; document.getElementById('viewsel').dispatchEvent(new Event('change'))")
    go(400, 860, 'light', 'w_phone.png', None, wait=2500)
    b.close()
