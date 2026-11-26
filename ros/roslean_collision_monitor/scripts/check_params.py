#!/usr/bin/env python3
# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""A first look at a parameter file: would the Lean-backed collision monitor refuse it?

    python3 check_params.py FILE.yaml [FILE.yaml ...]

Reads every collision monitor section of each file (a `ros__parameters`
mapping with `polygons` or `observation_sources`, under any node name) and
prints what is in it that the node does not provide: actions other than
`stop` and `slowdown`, zone types other than `polygon` and `circle`, more or
fewer than one source, a source that is not a laser scan, and the smaller
cases of docs/collision_monitor/SPEC.md, "Excluded features". Only the zones
named in `polygons` and the sources named in `observation_sources` are
looked at, because only those are read by the node (and by upstream's).

Needs Python with PyYAML and nothing else: it runs before anything is built.

Exit code: 0 nothing unsupported found in any section; 1 something found;
2 no collision monitor section in a file, or a file that cannot be read.

This is not the node's own check and does not replace it. It does not see
values that come from launch substitutions or from the command line, and it
does not check numbers (timeouts, coordinates, `min_points`). The authority
is the node: its `configure` transition fails with one message that names
every refused parameter (docs/DROP_IN_COLLISION_MONITOR.md, "Check your
parameters first").
"""

import sys

import yaml


def sections(node, path=''):
    """Every `ros__parameters` mapping that has `polygons` or `observation_sources`."""
    if not isinstance(node, dict):
        return
    for key, value in node.items():
        if key == 'ros__parameters' and isinstance(value, dict) and (
                'polygons' in value or 'observation_sources' in value):
            yield path or '/', value
        else:
            yield from sections(value, f'{path}/{key}')


def unsupported(p):
    """What the section `p` asks for that the node does not provide."""
    bad = []
    if p.get('use_realtime_priority'):
        bad.append('use_realtime_priority: true')
    sources = list(p.get('observation_sources') or [])
    if len(sources) != 1:
        bad.append(f'observation_sources names {len(sources)} sources; the node takes exactly one')
    for name in sources:
        source = p.get(name) or {}
        if source.get('type', 'scan') != 'scan':
            bad.append(f"{name}.type: {source.get('type')} (only scan)")
        if 'roslean_sensor_pose' in source:
            bad.append(f'{name}.roslean_sensor_pose (removed; the pose comes from TF)')
    zones = list(p.get('polygons') or [])
    if len(set(zones)) != len(zones) or '' in zones or 'invalid source' in zones:
        bad.append('polygons: a name twice, an empty name, or the name "invalid source"')
    for name in zones:
        zone = p.get(name) or {}
        if zone.get('action_type') not in ('stop', 'slowdown'):
            bad.append(f"{name}.action_type: {zone.get('action_type')} (only stop and slowdown)")
        if zone.get('type') not in ('polygon', 'circle'):
            bad.append(f"{name}.type: {zone.get('type')} (only polygon and circle)")
        if zone.get('type') == 'circle' and 'radius' not in zone:
            bad.append(f'{name}: a circle without radius')
        if (zone.get('type') == 'polygon' and zone.get('action_type') != 'approach'
                and 'points' not in zone and 'polygon_sub_topic' not in zone):
            bad.append(f'{name}: a polygon without points and without polygon_sub_topic'
                       + (' (footprint_topic is read for approach only)'
                          if 'footprint_topic' in zone else ''))
        if 'max_points' in zone:
            bad.append(f'{name}.max_points (use min_points)')
        if 'sources_names' in zone and list(zone['sources_names'] or []) != sources:
            bad.append(f'{name}.sources_names: not the one source')
        ratio = zone.get('slowdown_ratio', 0.5)
        if zone.get('action_type') == 'slowdown' and isinstance(ratio, (int, float)) and (
                ratio < 0 or str(ratio).startswith('-')):
            bad.append(f'{name}.slowdown_ratio: {ratio} (negative)')
    return bad


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    code = 0
    for file in sys.argv[1:]:
        try:
            with open(file) as f:
                content = yaml.safe_load(f)
        except (OSError, yaml.YAMLError) as e:
            print(f'{file}: cannot be read: {e}')
            code = max(code, 2)
            continue
        found = False
        for path, p in sections(content):
            if 'frequency' in p and 'cmd_vel_in_topic' not in p and 'detector' in path:
                continue  # a collision_detector section
            found = True
            bad = unsupported(p)
            if bad:
                code = max(code, 1)
                print(f'{file}: {path}: NOT ACCEPTED by the Lean-backed node:')
                for line in bad:
                    print(f'    {line}')
            else:
                print(f'{file}: {path}: nothing unsupported found')
        if not found:
            print(f'{file}: no collision monitor section found')
            code = max(code, 2)
    sys.exit(code)


if __name__ == '__main__':
    main()
