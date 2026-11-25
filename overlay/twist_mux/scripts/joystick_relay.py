#!/usr/bin/env python3
# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""Forwards to the executable of the same name in upstream twist_mux.

The ROSlean overlay package named twist_mux provides the multiplexer only.
`ros2 run` and launch files look for executables in the first package named
twist_mux on AMENT_PREFIX_PATH, which is the overlay, so the helper nodes of
upstream would no longer be found. This script is installed under their
names. It looks through the other prefixes for the real executable and
replaces itself with it, arguments unchanged.
"""

import os
import sys


def main():
    name = os.path.basename(sys.argv[0])
    here = os.path.dirname(os.path.realpath(__file__))
    for var in ('AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH'):
        for prefix in os.environ.get(var, '').split(os.pathsep):
            if not prefix:
                continue
            candidate = os.path.join(prefix, 'lib', 'twist_mux', name)
            if not os.path.isfile(candidate) or not os.access(candidate, os.X_OK):
                continue
            if os.path.dirname(os.path.realpath(candidate)) == here:
                continue  # this script
            os.execv(candidate, [candidate] + sys.argv[1:])
    sys.exit(
        f'{name}: not provided by the ROSlean overlay of twist_mux, and no upstream '
        f'twist_mux was found under it on AMENT_PREFIX_PATH. Install upstream '
        f'twist_mux (for example ros-$ROS_DISTRO-twist-mux) and source it before '
        f'this workspace.')


if __name__ == '__main__':
    main()
