"""Builds the RCM-7 Motion Lab page.

  python3 lab/lab_payload.py     # adds inertia tensors -> out/lab_payload.json
  python3 lab/make_lab.py        # -> out/rcm7_motionlab.html (single self-contained page)
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, '..', 'out')


def build():
    tpl = open(os.path.join(HERE, 'lab_template.html'), encoding='utf-8').read()
    core = open(os.path.join(HERE, 'planner_core.js'), encoding='utf-8').read()
    payload = open(os.path.join(OUT, 'lab_payload.json'), encoding='utf-8').read().replace('</', '<\\/')
    assert '__PAYLOAD__' in tpl and '/*__CORE__*/' in tpl
    html = tpl.replace('/*__CORE__*/', core).replace('__PAYLOAD__', payload)
    dst = os.path.join(OUT, 'rcm7_motionlab.html')
    open(dst, 'w', encoding='utf-8').write(html)
    print(f'{dst}: {len(html) / 1e6:.2f} MB')
    return dst


if __name__ == '__main__':
    build()
