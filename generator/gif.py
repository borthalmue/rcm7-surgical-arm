"""Render an animated loop of the demo motion for the README (uses zrender, takes a few minutes)."""
import math, time, numpy as np, sys
from zrender import *
from PIL import Image
N = int(sys.argv[1]) if len(sys.argv) > 1 else 48
frames = []
t0 = time.time()
for i in range(N):
    t = 2*math.pi*i/N
    pose = dict(REF, yaw=40*math.sin(t), theta=-3+34*math.sin(2*t+0.6), q=115+55*math.sin(t+1.2),
                roll=120*math.sin(t), psi=40*math.sin(2*t), chi=30*math.sin(t+1), grip=28+22*math.sin(3*t))
    img, proj, Tb = render(pose, (-1400, 820, 1450), (-300, 215, -40), W=820, H=600, fov=37, ss=2)
    # RCM marker
    im = Image.fromarray((img*255).astype(np.uint8))
    from PIL import ImageDraw
    x, y = proj(Mo.R3); d = ImageDraw.Draw(im); d.ellipse([x-5, y-5, x+5, y+5], fill=(212, 71, 15), outline=(255, 255, 255))
    frames.append(im.convert('P', palette=Image.ADAPTIVE, colors=128))
    print(i, round(time.time()-t0), flush=True)
frames[0].save('out/rcm7-demo.gif', save_all=True, append_images=frames[1:], duration=90, loop=0, optimize=True)
print('done')
