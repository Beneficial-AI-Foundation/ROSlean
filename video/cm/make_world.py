#!/usr/bin/env python3
"""Writes the filming world for the collision monitor video.

It is the world of the twist_mux video (../make_world.py: the demo's
apartment with furniture built from boxes, a textured floor, a fixed camera
and a lowered real-time factor), written by that script. With WIDTH = 0 the
camera is left out, for a take that records messages only.

Usage: make_world.py APARTMENT_SDF OUT_SDF WIDTH HEIGHT RTF
"""

import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import make_world  # noqa: E402

if __name__ == '__main__':
    src, dst, width, height, rtf = sys.argv[1:6]
    sys.argv = [sys.argv[0], src, dst, width if int(width) else '640', height if int(width) else '360', rtf]
    make_world.main()
    if not int(width):
        world = open(dst).read()
        world, n = re.subn(r'\s*<model name="video_camera">.*?</model>\n', '\n', world, flags=re.S)
        assert n == 1
        open(dst, 'w').write(world)
