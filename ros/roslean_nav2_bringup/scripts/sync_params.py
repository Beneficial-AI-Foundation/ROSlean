#!/usr/bin/env python3
# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""Keeps this package's parameter files equal to Nav2's outside one section.

    sync_params.py [DIR]            check (exit code 1 and a diff if they differ)
    sync_params.py [DIR] --update   rewrite the files from Nav2's

Each `nav2_params*.yaml` in DIR (default: the installed params directory of
roslean_nav2_bringup; name the source directory to change the sources) must
be nav2_bringup's installed `params/nav2_params.yaml`, line for line, except
for the `collision_monitor` section. --update writes each file again as
Nav2's installed file with that file's own `collision_monitor` section, so
that a Nav2 update is taken over and the zones stay.
"""

import difflib
import glob
import os
import sys

from ament_index_python.packages import get_package_share_directory

SECTION = 'collision_monitor:'


def split(path):
    """(lines before the section, the section, lines after it). The section
    runs from its key to the next top-level key; blank lines at its end go
    with what follows."""
    with open(path) as f:
        lines = f.read().split('\n')
    if lines.count(SECTION) != 1:
        sys.exit(f'{path}: expected one top-level key {SECTION!r}, found {lines.count(SECTION)}')
    start = lines.index(SECTION)
    end = next((i for i in range(start + 1, len(lines))
                if lines[i] and not lines[i][0].isspace() and not lines[i].startswith('#')),
               len(lines))
    while not lines[end - 1].strip():
        end -= 1
    return lines[:start], lines[start:end], lines[end:]


def main():
    args = [a for a in sys.argv[1:] if a != '--update']
    update = '--update' in sys.argv[1:]
    directory = args[0] if args else os.path.join(
        get_package_share_directory('roslean_nav2_bringup'), 'params')
    nav2 = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    before, _, after = split(nav2)
    files = sorted(glob.glob(os.path.join(directory, 'nav2_params*.yaml')))
    if not files:
        sys.exit(f'no nav2_params*.yaml in {directory}')
    code = 0
    for file in files:
        mine_before, section, mine_after = split(file)
        if mine_before == before and mine_after == after:
            print(f'{file}: equal to {nav2} outside the collision_monitor section')
        elif update:
            with open(file, 'w') as f:
                f.write('\n'.join(before + section + after))
            print(f'{file}: rewritten from {nav2}, collision_monitor section kept')
        else:
            code = 1
            print(f'{file}: differs from {nav2} outside the collision_monitor section:')
            for a, b in ((before, mine_before), (after, mine_after)):
                for line in difflib.unified_diff(a, b, nav2, file, lineterm='', n=1):
                    print('    ' + line)
            print('    (Nav2 changed its defaults, or the file was edited outside the section. '
                  'To take over Nav2\'s: sync_params.py SOURCE_DIR --update)')
    sys.exit(code)


if __name__ == '__main__':
    main()
