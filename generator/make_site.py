"""
Assemble the published site and CAD folder from the generator output.

  python3 build.py        # geometry, checks, STL + BOM export   (~6 min, runs the interference sweep)
  python3 sim_payload.py  # simulator data (meshes, spin ratios, belts)
  python3 make_site.py    # writes ../index.html, ../assets/rcm7-data.js, ../cad/*
"""
import os, shutil, re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, 'out')

tpl = open(os.path.join(HERE, 'sim_template.html'), encoding='utf-8').read()
head, body = tpl.split('</style>', 1)
head += '</style>'
head = head.replace('<meta name="claude:viewport-fit-sides" content="cover">\n', '')
body = body.replace('<script type="application/json" id="payload">__PAYLOAD__</script>', '<script src="assets/rcm7-data.js"></script>')
body = body.replace("const DATA = JSON.parse(document.getElementById('payload').textContent);", "const DATA = window.RCM7_DATA;")
assert 'window.RCM7_DATA' in body and '__PAYLOAD__' not in body

embed_css = """
<style>
[hidden]{display:none!important}
body{margin:0}
img{max-width:100%}
/* ?embed: just the stage and the control pendant, for iframes */
body.embed .wrap{padding-block:0;gap:0;max-width:none}
body.embed .wrap>header, body.embed .lede, body.embed section{display:none}
body.embed .stage{position:relative;top:0}
body.embed #view{height:calc(100vh - 2px);min-height:420px}
@media (max-width:1000px){ body.embed #view{height:62vh} }
</style>"""
embed_js = "<script>if (/[?&]embed\\b/.test(location.search) || location.hash === '#embed') document.documentElement.classList.add('embed-mode');</script>"
page = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="description" content="Interactive simulator of RCM-7, a 7-DOF remote-center-of-motion arm for minimally invasive surgery.">
<meta property="og:title" content="RCM-7 Simulator">
<meta property="og:description" content="Drive all seven joints of a fault-tolerant surgical arm in the browser.">
<meta property="og:image" content="images/overall.png">
{embed_js}
{head}
{embed_css}
</head>
<body>
<script>if (document.documentElement.classList.contains('embed-mode')) document.body.classList.add('embed');</script>
{body}
</body>
</html>
"""
open(os.path.join(ROOT, 'index.html'), 'w', encoding='utf-8').write(page)

os.makedirs(os.path.join(ROOT, 'assets'), exist_ok=True)
data = open(os.path.join(OUT, 'sim_payload.json'), encoding='utf-8').read()
open(os.path.join(ROOT, 'assets', 'rcm7-data.js'), 'w', encoding='utf-8').write('window.RCM7_DATA = ' + data + ';\n')

cad = os.path.join(ROOT, 'cad')
if os.path.isdir(os.path.join(cad, 'stl')): shutil.rmtree(os.path.join(cad, 'stl'))
shutil.copytree(os.path.join(OUT, 'stl'), os.path.join(cad, 'stl'))
for f, g in (('RCM7_assembly_reference_pose.stl', 'RCM7_assembly.stl'), ('BOM.csv', 'BOM.csv'), ('report.json', 'verification_report.json')):
    shutil.copy(os.path.join(OUT, f), os.path.join(cad, g))
if os.path.exists(os.path.join(OUT, 'rcm7-demo.gif')):
    shutil.copy(os.path.join(OUT, 'rcm7-demo.gif'), os.path.join(ROOT, 'images', 'rcm7-demo.gif'))
print('site written:', os.path.getsize(os.path.join(ROOT, 'index.html'))//1024, 'KB page,',
      os.path.getsize(os.path.join(ROOT, 'assets', 'rcm7-data.js'))//1024, 'KB data,', len(os.listdir(os.path.join(cad, 'stl'))), 'STL parts')
