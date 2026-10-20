"""Copy the simulator into the portfolio repo (borthalmue/portfolio/rcm7/). Run after make_site.py.
   python3 export_portfolio.py /path/to/portfolio"""
import os, sys, shutil
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
dst = os.path.join(sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '..', 'portfolio'), 'rcm7')
os.makedirs(dst + '/assets', exist_ok=True); os.makedirs(dst + '/images', exist_ok=True)
h = open(os.path.join(ROOT, 'index.html'), encoding='utf-8').read()
theme = "<script>try{var t=localStorage.getItem('theme');if(t)document.documentElement.setAttribute('data-theme',t);}catch(e){}</script>\n"
h = h.replace('<meta charset="utf-8">', '<meta charset="utf-8">\n' + theme, 1)
h = h.replace('<title>RCM-7 Simulator</title>', '<title>RCM-7 Simulator · Brock Pola</title>')
crumb = ('<nav class="crumb"><a href="../">← Brock Pola · Portfolio</a>'
         '<a href="https://github.com/borthalmue/rcm7-surgical-arm" target="_blank" rel="noopener">CAD and code on GitHub</a></nav>\n  ')
h = h.replace('<div class="wrap">\n  <header>', '<div class="wrap">\n  ' + crumb + '<header>', 1)
h = h.replace('</head>', "<style>.crumb{display:flex;flex-wrap:wrap;justify-content:space-between;gap:8px 20px;font-size:.84rem}"
              ".crumb a{color:var(--muted);text-decoration:none;border-bottom:1px solid var(--rule)}.crumb a:hover{color:var(--ink)}"
              "body.embed .crumb{display:none}</style>\n</head>", 1)
assert 'class="crumb"' in h
open(os.path.join(dst, 'index.html'), 'w', encoding='utf-8').write(h)
shutil.copy(os.path.join(ROOT, 'assets', 'rcm7-data.js'), os.path.join(dst, 'assets', 'rcm7-data.js'))
shutil.copy(os.path.join(ROOT, 'images', 'overall.png'), os.path.join(dst, 'images', 'overall.png'))
print('exported to', os.path.abspath(dst))
