#!/usr/bin/env python3
"""Turns a take (recorder.py) into web/data.js, which the two pages load.

Usage: build_data.py TAKE_DIR
"""

import json
import math
import os
import sys

import make_world

take = sys.argv[1]
T = json.load(open(os.path.join(take, 'trace.json')))
(x, y, z), (lx, ly, lz) = make_world.CAM_POS, make_world.CAM_LOOK
unknown = sum(1 for _, s in T['out'] if s == '?')
assert unknown == 0, f'{unknown} output messages could not be attributed to an input'
data = {
    'frames': T['frames'],                       # time of each recorded frame
    'size': T['size'],
    'pose': T['pose'][::3],                      # [t, x, y, yaw]
    'inT': T['in'],
    'outT': [t for t, _ in T['out']], 'outSrc': [s for _, s in T['out']],
    'lockT': [t for t, _ in T['lock']], 'lockV': [v for _, v in T['lock']],
    'camera': {
        'pos': [x, y, z], 'hfov': make_world.CAM_HFOV,
        'yaw': math.atan2(ly - y, lx - x),
        'pitch': math.atan2(z - lz, math.hypot(lx - x, ly - y)),
    },
}
out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web', 'data.js')
with open(out, 'w') as f:
    f.write('window.DATA = ' + json.dumps(data, separators=(',', ':')) + ';\n')
print(out, len(data['frames']), 'frames,', len(data['outT']), 'forwarded messages')
