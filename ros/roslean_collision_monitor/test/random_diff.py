#!/usr/bin/env python3
"""Randomised differential test of two collision-monitor nodes.

    random_diff.py [NODE [OTHER [SEED_FROM SEED_TO]]] [options]

NODE and OTHER are executable paths or `<package>/<executable>`; both default
to upstream's `nav2_collision_monitor/collision_monitor`, so that without
arguments the test compares upstream with itself (which measures the
nondeterminism of the harness and of the node). Default seeds: 0..19.

For each seed the test generates a random configuration and a random sequence
of events (clock updates including backward jumps, scans, commands), runs
both nodes side by side on the very same messages and the same `/clock`, and
compares what they publish, command by command: the velocity messages bit
for bit, and the state messages. Both are also compared with the Python
model in cm_model.py, a rendering of the behaviour described in
docs/collision_monitor/UPSTREAM_BEHAVIOUR.md. A divergence between the nodes
is retried once with a long settle time, to separate harness timing from a
real difference.

Options:
  --lifecycle auto|yes|no   as scenario.py (applies to both nodes)
  --no-tf                   only generate cases with the scan in the base frame
  --ignore-state            do not compare the state topic
  --settle S                wall seconds per event (default 0.05)
  --no-model                compare the two nodes only
  --logs DIR                keep the nodes' output
  --roslean                 NODE is the ROSlean node and OTHER is upstream; see
                            "ROSlean against upstream" below
  --stats FILE              with --roslean: write the counts as JSON
  --yaw                     with --roslean: half of the cases have the scan
                            frame turned by an angle that is not a multiple of
                            half a turn (upstream: static transform; ROSlean:
                            the yaw of the sensor pose)
  --self-test swap|shrink   with --roslean: judge wrongly on purpose, to see
                            that the judgement can fail. `swap` exchanges the
                            two nodes (upstream then looks like a ROSlean node
                            that misses points); `shrink` allows zones to grow
                            by a tenth of the stated bounds only. The run
                            succeeds if a violation is reported.

Coverage per case: 1-3 zones (circles and convex polygons, stop and slowdown,
min_points 0-5); source timeouts including 0; scans with 0-8 rays, invalid
ranges, stamps in the past and in the future (never negative), and a `ranges` length that
disagrees with `angle_max`; the scan in the base frame or in a frame offset
and possibly turned by half a turn, with and without base-shift correction;
commands with zero, negative and non-finite components; clock bases of 0,
100 s and Unix scale; events before the first clock message; 25 % of the
cases with TwistStamped.

What is kept away from boundaries: event times lie on a 50 ms grid and the
timeouts are odd multiples of 25 ms; scans with a point within 0.1 mm of a
zone boundary are redrawn (a third of the rays are aimed within 5 % of a
zone's size). Exact boundaries are probed in scenario.py.

ROSlean against upstream (`--roslean`). The ROSlean node differs from
upstream by design (docs/collision_monitor/SPEC.md), so:

  * only configurations that both nodes accept are generated: no base-shift
    correction (D11); ratios, timeouts and min_points are those above, which
    both accept;
  * the ROSlean node gets no state topic here (it has one since task T06;
    random_nav2.py compares the state messages), and both nodes get the
    static transform for the scan frame (the ROSlean node applied a fixed
    sensor pose instead until task T06);
  * points near zone boundaries are kept, and a fifth of the scans are aimed
    at a boundary: at a point of it, or up to 6 mm to either side;
  * the answers are not compared bit for bit but against the SPEC's bounds
    for the integer-millimetre zones (D3, D4), as cm_bounds.py states them:
    ROSlean never demands a less restrictive velocity than upstream, and it
    never counts a point farther than 3.5 mm (circle) or 4.4 mm (polygon)
    outside the zone. The run reports how often the answers differ and the
    distance of the point that decided each difference.
"""

import argparse
import math
import os
import random
import sys
import tempfile
import time

import rclpy

from cm_harness import (
    NS, UPSTREAM, Driver, Side, f32, fmt, resolve, same_floats, static_tf, twist6)
from cm_bounds import GROWTH, Oracle
from cm_bounds import scan_points as exact_points
from cm_model import Model, Zone, scan_points

STEP = 50_000_000  # 0.05 s grid; timeouts are odd multiples of 0.025 s
BASES = [0, 0, 100, 100, 1_800_000_000]
SOURCE_TIMEOUTS = [0.0, 0.075, 0.125, 0.275, 0.525, 1.025, 2.075]
STOP_PUB_TIMEOUTS = [0.075, 0.125, 0.225, 0.475, 1.025]
SIZES = [0.5, 0.75, 1.0, 1.5]
RATIOS = [0.5, 0.25, 0.3, 0.75, 0.0, 1.0]
SPEEDS = [0.0, 0.0, 0.0, 0.5, -0.5, 0.25, 1.0, -1.0, 0.1, -0.0]
MARGIN = 1e-4


def polygon_points(shape, size):
    if shape == 'square':
        return [(size, size), (size, -size), (-size, -size), (-size, size)]
    if shape == 'front':  # a rectangle ahead of the robot only
        return [(size, 0.25), (size, -0.25), (0.125, -0.25), (0.125, 0.25)]
    # a pentagon pointing forwards
    return [(size, 0.0), (0.25, -size), (-size, -size), (-size, size), (0.25, size)]


def boundary_distance(z, p):
    """Distance from a point to the boundary of a zone."""
    x, y = p
    if z.kind == 'circle':
        return abs(math.hypot(x, y) - z.radius)
    best = math.inf
    i = len(z.poly) - 1
    for j in range(len(z.poly)):
        (xi, yi), (xj, yj) = z.poly[i], z.poly[j]
        dx, dy = xj - xi, yj - yi
        t = max(0.0, min(1.0, ((x - xi) * dx + (y - yi) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(x - (xi + t * dx), y - (yi + t * dy)))
        i = j
    return best


def zone_yaml(z):
    lines = [f'    {z.name}:', f'      type: "{z.kind}"']
    if z.kind == 'circle':
        lines.append(f'      radius: {z.radius!r}')
    else:
        pts = ', '.join(f'[{x!r}, {y!r}]' for x, y in z.poly)
        lines.append(f'      points: "[{pts}]"')
    lines.append(f'      action_type: "{z.action}"')
    if z.action == 'slowdown':
        lines.append(f'      slowdown_ratio: {z.ratio!r}')
    lines.append(f'      min_points: {z.min_points}')
    return lines


def points_of(spec, tf):
    return scan_points(spec['ranges'], spec['angle_min'], spec['angle_increment'],
                       spec['range_min'], spec['range_max'], tf)


def gen(seed, allow_tf=True, roslean=False, turned=False):
    r = random.Random(seed)
    # A second generator, so that the cases without --yaw stay as they are.
    r2 = random.Random(seed + 1_000_003)
    yaw = r2.choice([math.pi / 2, -math.pi / 2, 0.3, -2.5, 1.0, 3.0]) if (
        turned and r2.random() < 0.5) else None
    zones = []
    for i in range(r.randint(1, 3)):
        kind = r.choice(['circle', 'polygon'])
        action = r.choice(['stop', 'slowdown', 'slowdown'])
        shape, size = r.choice(['square', 'front', 'pentagon']), r.choice(SIZES)
        zones.append(Zone(
            f'Zone{i}', kind, size if kind == 'circle' else polygon_points(shape, size), action,
            r.choice(RATIOS), r.choice([0, 1, 1, 2, 3, 4, 5])))
    tf = (0.0, 0.0, False)
    shift = False
    if allow_tf and r.random() < 0.5:
        tf = (r.choice([0.25, -0.25, 0.5, 0.0]), r.choice([0.0, 0.0, 0.125, -0.25]),
              r.random() < 0.3)
        shift = r.random() < 0.5 and not roslean  # ROSlean refuses the correction (D11)
    if yaw is not None:
        tf = (r2.choice([0.25, -0.25, 0.5, 0.0]), r2.choice([0.0, 0.125, -0.25]), False)
    cfg = {'yaw': yaw,
        'zones': zones, 'tf': tf, 'shift': shift,
        'use_tf': tf != (0.0, 0.0, False) or shift or yaw is not None,
        'source_timeout': r.choice(SOURCE_TIMEOUTS),
        'stop_pub_timeout': r.choice(STOP_PUB_TIMEOUTS),
        'stamped': r.random() < 0.25,
    }

    def aimed_ranges():
        """Rays that all hit one point at, or up to 6 mm from, the boundary of a zone."""
        z = r.choice(zones)
        if z.kind == 'circle':
            a = r.uniform(-math.pi, math.pi)
            bx, by = z.radius * math.cos(a), z.radius * math.sin(a)
        else:
            i = r.randrange(len(z.poly))
            (x0, y0), (x1, y1) = z.poly[i - 1], z.poly[i]
            t = r.choice([0.0, r.random(), r.random()])  # a vertex, or a point of an edge
            bx, by = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        off = r.choice([0.0, r.uniform(0.0, 0.0005), r.uniform(0.0, 0.006), r.uniform(0.0, 0.006)])
        phi = r.uniform(-math.pi, math.pi)
        bx, by = bx + off * math.cos(phi), by + off * math.sin(phi)
        sx, sy = bx - tf[0], by - tf[1]  # in the scan frame
        if yaw is not None:
            sx, sy = (math.cos(yaw) * sx + math.sin(yaw) * sy,
                      -math.sin(yaw) * sx + math.cos(yaw) * sy)
        elif tf[2]:
            sx, sy = -sx, -sy
        ranges = [f32(math.hypot(sx, sy))] * r.choice([1, 1, 2, 3, 4, 5, 6])
        ranges += [f32(r.uniform(0.1, 3.0))] * r.choice([0, 0, 1, 3])
        return ranges, f32(math.atan2(sy, sx))

    def rand_scan(now):
        while True:
            n = r.choice([0, 1, 2, 3, 3, 4, 5, 6, 8])
            near = r.random() < 0.6  # most scans have returns near the robot
            ranges = []
            for _ in range(n):
                x = r.random()
                if x < 0.08:
                    ranges.append(r.choice([math.nan, math.inf, -math.inf, 0.01, 30.0, -0.5]))
                elif x < 0.4:  # close to the nominal size of a zone, inside or outside
                    size = r.choice(SIZES)
                    ranges.append(f32(size * (1 + r.choice([-1, 1]) * r.uniform(0.001, 0.05))))
                else:
                    ranges.append(f32(r.uniform(0.1, 1.2 if near else 3.0)))
            inc = r.choice([0.0, 0.01, 0.05, 0.3, -0.2, math.pi / 4])
            spec = {'ranges': ranges, 'angle_min': f32(r.uniform(-math.pi, math.pi)),
                    'angle_increment': f32(inc), 'range_min': 0.05, 'range_max': 10.0}
            if roslean and r.random() < 0.2:
                spec['ranges'], spec['angle_min'] = aimed_ranges()
                spec['angle_increment'] = 0.0
            if r.random() < 0.2:  # an angle_max that disagrees with len(ranges)
                spec['angle_max'] = spec['angle_min'] + r.choice([0.0, 0.1, 10.0])
            x = r.random()
            if x < 0.15:
                spec['age'] = r.choice([1, 2, 5, 10, 20, 40, 400]) * STEP
            elif x < 0.22:
                spec['age'] = -r.choice([1, 5, 20, 100]) * STEP
            # Not before time 0: a scan with a negative stamp makes upstream
            # abort at the next command (scenario.py, negative-stamp).
            if now - spec.get('age', 0) < 0:
                spec['age'] = now
            if roslean:  # boundary cases are kept
                return spec
            pts = points_of(spec, tf)
            if all(boundary_distance(z, p) > MARGIN for z in zones for p in pts):
                return spec

    events = []
    base = r.choice(BASES) * NS
    now = base
    if base:
        if r.random() < 0.3:  # some events at clock 0, before any /clock message
            for _ in range(r.randint(1, 4)):
                events.append(('cmd', tuple(r.choice(SPEEDS) for _ in range(3))))
        events.append(('clock', now))
    for _ in range(r.randint(150, 250)):
        x = r.random()
        if x < 0.25:
            now += r.choice([1, 1, 1, 2, 2, 3, 5, 6, 8, 14, 15, 27, 40]) * STEP
            events.append(('clock', now))
        elif x < 0.28 and now > 0:
            now = max(0, now - r.choice([1, 3, 10, 30, 2000]) * STEP)
            events.append(('clock', now))
        elif x < 0.55:
            events.append(('scan', rand_scan(now)))
        else:
            vel = [r.choice(SPEEDS) for _ in range(3)]
            y = r.random()
            if y < 0.04:
                vel[r.randrange(3)] = r.choice([math.nan, math.inf, -math.inf])
            if y > 0.85:  # all six components
                vel = [vel[0], vel[1], r.choice([0.0, 0.5, math.nan] if y > 0.98 else [0.0, 0.5]),
                       r.choice([0.0, -0.25]), r.choice([0.0, 2.0]), vel[2]]
            events.append(('cmd', tuple(vel)))
    return cfg, events


def write_yaml(cfg, path):
    lines = [
        '/**:', '  ros__parameters:',
        '    base_frame_id: base_footprint', '    odom_frame_id: odom',
        '    transform_tolerance: 0.0',
        f'    source_timeout: {cfg["source_timeout"]!r}',
        f'    stop_pub_timeout: {cfg["stop_pub_timeout"]!r}',
        f'    base_shift_correction: {"true" if cfg["shift"] else "false"}',
        f'    enable_stamped_cmd_vel: {"true" if cfg["stamped"] else "false"}',
        '    observation_sources: ["scan"]', '    scan:', '      type: "scan"',
        '    polygons: [%s]' % ', '.join(f'"{z.name}"' for z in cfg['zones']),
    ]
    for z in cfg['zones']:
        lines += zone_yaml(z)
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def observed(d, outs, states):
    """What one node published in answer to one command, in comparable form."""
    return ([twist6(d.twist_of(m)) for m in outs],
            [(m.polygon_name, m.action_type) for m in states])


def same_answer(a, b, check_state):
    return (len(a[0]) == len(b[0]) and all(same_floats(x, y) for x, y in zip(a[0], b[0]))
            and (not check_state or a[1] == b[1]))


def show(ans):
    return ' + '.join([fmt(v) for v in ans[0]] + [f'state{st}' for st in ans[1]]) or 'nothing'


def run_case(tag, cfg, events, exes, opts, settle):
    """Returns dict: answers per side, model answers, problems."""
    cfg_dir = tempfile.mkdtemp(prefix='roslean_cm_diff_')
    path = os.path.join(cfg_dir, f'{tag}.yaml')
    write_yaml(cfg, path)
    stamped = cfg['stamped']
    yaw = cfg.get('yaw')
    scan_frame = ('base_scan' if cfg['tf'] != (0.0, 0.0, False) or yaw is not None
                  else 'base_footprint')
    sides = []
    for letter, exe in zip('ab', exes):
        roslean = opts.roslean and letter == 'a'
        side = Side(exe, f'cm_{tag}_{letter}', f'{tag}_out_{letter}',
                    None if roslean else f'{tag}_state_{letter}')
        log = os.path.join(opts.logs, f'{tag}_{letter}.log') if opts.logs else None
        params = [f'cmd_vel_in_topic:={tag}_cmd', f'scan.topic:={tag}_scan']
        # (Until task T06 the ROSlean node got the pose of the scan frame as
        # `scan.roslean_sensor_pose` here, D11. It now looks the static
        # transform up, like upstream, and the parameter is gone.)
        side.start(path, params, log)
        sides.append(side)
    d = Driver('diff_' + tag, sides, stamped, cmd_topic=f'{tag}_cmd', scan_topic=f'{tag}_scan',
               scan_frame=scan_frame)
    model = Model(cfg['zones'], cfg['source_timeout'], cfg['stop_pub_timeout'])
    res = {'a': [], 'b': [], 'model': [], 'problems': [], 'sent': []}
    try:
        transforms = []
        if cfg['use_tf']:
            transforms.append(static_tf('odom', 'base_footprint'))
            if scan_frame != 'base_footprint':
                tx, ty, flip = cfg['tf']
                t = static_tf('base_footprint', scan_frame, tx, ty, int(flip))
                if yaw is not None:
                    t.transform.rotation.z = math.sin(yaw / 2)
                    t.transform.rotation.w = math.cos(yaw / 2)
                transforms.append(t)
        d.publish_tf(transforms)
        for side in sides:
            if d.bring_up(side, opts.lifecycle) != 'active':
                res['problems'].append(f'{side.name} refused its configuration')
                return res
        d.wait_connected(tf_listeners=sum(bool(sd.lifecycle) for sd in sides) if transforms else 0)
        ident = 0
        for ev in events:
            if ev[0] == 'clock':
                model.clock(ev[1])
                d.set_clock(ev[1], settle)
            elif ev[0] == 'scan':
                model.scan_in(ev[1].get('stamp', model.now - ev[1].get('age', 0)),
                              points_of(ev[1], cfg['tf']))
                d.send_scan(ev[1], settle)
            else:
                ident += 1
                exp = model.cmd(ev[1])
                want = 0 if exp[0] is None else 1
                res['msg'] = d.make_cmd(ev[1], ident)
                answers = d.send_cmd(
                    res['msg'], settle,
                    wait_for=(lambda i, o, st: len(o) >= want) if want and opts.model else None,
                    max_wait=1.0)
                for letter, (outs, states) in zip('ab', answers):
                    res[letter].append(observed(d, outs, states))
                    if stamped and any(m.header != res['msg'].header for m in outs):
                        res['problems'].append(f'command {ident}: header not copied by {letter}')
                res['model'].append(([] if exp[0] is None else [exp[0]],
                                     [] if exp[1] is None else [exp[1]]))
                res['sent'].append(ev[1])
        d.spin_for(0.3)
        for side in sides:
            if sum(len(a[0]) for a in res[side.name[-1]]) != len(side.out):
                res['problems'].append(f'late velocity messages from {side.name}')
            if side.proc.poll() is not None:
                res['problems'].append(f'{side.name} died with {side.proc.returncode}')
    except RuntimeError as e:
        res['problems'].append(str(e))
    finally:
        for side in sides:
            side.stop()
        d.destroy_node()
    return res


def judge_bounds(cfg, events, res, self_test=''):
    """ROSlean (a) against upstream (b) under the SPEC's bounds; see cm_bounds.py.

    Returns (problems, stats). A problem is (severity, command number, text).
    """
    growth = {k: v / 10 for k, v in GROWTH.items()} if self_test == 'shrink' else None
    oracle = Oracle(cfg['zones'], int(cfg['source_timeout'] * 1e9),
                    int(cfg['stop_pub_timeout'] * 1e9), growth)
    lean, upstream = ('b', 'a') if self_test == 'swap' else ('a', 'b')
    problems = []
    stats = {'commands': 0, 'differ': 0, 'published_differ': 0, 'ambiguous': 0, 'reach': [],
             'scans': 0, 'band': {'circle': [0, 0], 'polygon': [0, 0]}}
    i = 0
    last_scan = None
    for ev in events:
        if ev[0] == 'clock':
            oracle.clock(ev[1])
        elif ev[0] == 'scan':
            last_scan = ev[1]
            oracle.scan_in(ev[1].get('stamp', oracle.now - ev[1].get('age', 0)),
                           exact_points(ev[1], cfg['tf'], cfg.get('yaw')))
            stats['scans'] += 1
            for kind, (band, total) in oracle.band_points().items():
                stats['band'][kind][0] += band
                stats['band'][kind][1] += total
        else:
            if i >= len(res['a']):
                break
            verdict = oracle.cmd(ev[1], res[lean][i][0], res[upstream][i][0])
            stats['commands'] += 1
            stats['differ'] += verdict['differ']
            stats['published_differ'] += verdict['published_differ']
            stats['ambiguous'] += verdict['ambiguous']
            if verdict['reach'] is not None:
                stats['reach'].append(verdict['reach'])
            for severity, text in verdict['problems']:
                problems.append((severity, i + 1, f'{text}: command {fmt(ev[1])} at clock '
                                 f'{oracle.now} ns, roslean {show(res["a"][i])}, upstream '
                                 f'{show(res["b"][i])}, scan in force {last_scan}'))
            i += 1
    return problems, stats


def main_roslean(opts, exes):
    """Seeds opts.seed_from..seed_to, ROSlean against upstream, judged by judge_bounds."""
    import json
    bad = retried = 0
    total = {'cases': 0, 'commands': 0, 'differ': 0, 'published_differ': 0, 'ambiguous': 0,
             'reach': [], 'scans': 0,
             'band': {'circle': [0, 0], 'polygon': [0, 0]}, 'failed_seeds': [], 'retried_seeds': []}
    for seed in range(opts.seed_from, opts.seed_to):
        cfg, events = gen(seed, allow_tf=not opts.no_tf, roslean=True, turned=opts.yaw)
        t0 = time.monotonic()
        res = run_case(f's{seed}', cfg, events, exes, opts, opts.settle)
        problems, stats = judge_bounds(cfg, events, res, opts.self_test)
        status = 'ok  '
        if problems or res['problems'] or not res['a']:
            first = problems, res['problems']
            res = run_case(f's{seed}r', cfg, events, exes, opts, 0.3)
            problems, stats = judge_bounds(cfg, events, res, opts.self_test)
            if problems or res['problems'] or not res['a']:
                status = 'FAIL'
                bad += 1
                total['failed_seeds'].append(seed)
                for severity, n, text in problems[:10]:
                    print(f'  {severity}: command {n}: {text}')
                print(f'  problems: {res["problems"]}')
                print('  zones: ' + '; '.join(' '.join(zone_yaml(z)[1:]) for z in cfg['zones'])
                      + f'; tf {cfg["tf"]}; yaw {cfg["yaw"]}; source_timeout {cfg["source_timeout"]}; '
                      f'stop_pub_timeout {cfg["stop_pub_timeout"]}')
            else:
                status = 'ok   (on slow retry)'
                retried += 1
                total['retried_seeds'].append(seed)
                print(f'  first attempt: {[p[:2] for p in first[0][:5]]} {first[1]}')
        total['cases'] += 1
        for key in ('commands', 'differ', 'published_differ', 'ambiguous', 'scans'):
            total[key] += stats[key]
        total['reach'] += stats['reach']
        for kind in total['band']:
            for k in (0, 1):
                total['band'][kind][k] += stats['band'][kind][k]
        far = max([d for d, _ in stats['reach']], default=0.0)
        print(f'{status}  seed={seed} zones={len(cfg["zones"])} tf={int(cfg["use_tf"])} '
              f'yaw={cfg["yaw"]} stamped={int(cfg["stamped"])} commands={stats["commands"]} '
              f'boundary-dependent={stats["ambiguous"]} differ={stats["differ"]} '
              f'published-differ={stats["published_differ"]} '
              f'farthest={far * 1000:.3f}mm {time.monotonic() - t0:.1f}s', flush=True)
    for kind in ('circle', 'polygon'):
        ds = sorted(d for d, k in total['reach'] if k == kind)
        band, pts = total['band'][kind]
        print(f'{kind} zones: {pts} point tests, {band} of them within {GROWTH[kind] * 1000} mm '
              f'outside the zone; {len(ds)} answers differ because of such a point'
              + (f', the deciding point {ds[0] * 1000:.3f} to {ds[-1] * 1000:.3f} mm outside '
                 f'(median {ds[len(ds) // 2] * 1000:.3f})' if ds else ''))
    print(f'{bad} case(s) outside the bounds in {opts.seed_to - opts.seed_from} random cases, '
          f'{total["commands"]} commands, of which {total["ambiguous"]} depend on a point in a '
          f'growth band; the two nodes demand different velocities for {total["differ"]} and '
          f'publish differently for {total["published_differ"]}; '
          f'{retried} case(s) passed only on the slow retry')
    if opts.stats:
        with open(opts.stats, 'w') as f:
            json.dump(total, f)
    if opts.self_test:
        print('self-test: ' + ('violations were reported, as they must be' if bad
                               else 'NO violation was reported'))
        return 0 if bad else 1
    return bad


def differences(res, x, y, check_state):
    return [i for i, (p, q) in enumerate(zip(res[x], res[y])) if not same_answer(p, q, check_state)]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node', nargs='?', default=UPSTREAM)
    ap.add_argument('other', nargs='?', default=UPSTREAM)
    ap.add_argument('seed_from', nargs='?', type=int, default=0)
    ap.add_argument('seed_to', nargs='?', type=int, default=20)
    ap.add_argument('--lifecycle', choices=['auto', 'yes', 'no'], default='auto')
    ap.add_argument('--no-tf', action='store_true')
    ap.add_argument('--ignore-state', action='store_true')
    ap.add_argument('--settle', type=float, default=0.05)
    ap.add_argument('--no-model', dest='model', action='store_false')
    ap.add_argument('--logs', default='')
    ap.add_argument('--roslean', action='store_true')
    ap.add_argument('--stats', default='')
    ap.add_argument('--yaw', action='store_true')
    ap.add_argument('--self-test', choices=['', 'swap', 'shrink'], default='')
    opts = ap.parse_args()
    if opts.logs:
        os.makedirs(opts.logs, exist_ok=True)
    exes = [resolve(opts.node), resolve(opts.other)]
    print(f'a: {exes[0]}\nb: {exes[1]}')
    check_state = not opts.ignore_state
    rclpy.init()
    if opts.roslean:
        bad = main_roslean(opts, exes)
        rclpy.shutdown()
        sys.exit(1 if bad else 0)
    bad = retried = total = 0
    for seed in range(opts.seed_from, opts.seed_to):
        cfg, events = gen(seed, allow_tf=not opts.no_tf)
        t0 = time.monotonic()

        def verdict(res):
            ab = differences(res, 'a', 'b', check_state)
            am = differences(res, 'a', 'model', check_state) if opts.model else []
            bm = differences(res, 'b', 'model', check_state) if opts.model else []
            return ab, am, bm

        res = run_case(f's{seed}', cfg, events, exes, opts, opts.settle)
        ab, am, bm = verdict(res)
        status = 'ok  '
        if ab or am or bm or res['problems'] or not res['a']:
            first = res
            res = run_case(f's{seed}r', cfg, events, exes, opts, 0.3)
            ab, am, bm = verdict(res)
            if ab or am or bm or res['problems'] or not res['a']:
                status = 'FAIL'
                bad += 1
                for i in sorted(set(ab + am + bm))[:10]:
                    print(f'  command {i + 1} {fmt(res["sent"][i])}: a {show(res["a"][i])} | '
                          f'b {show(res["b"][i])} | model {show(res["model"][i])}')
                print(f'  problems: {res["problems"]}')
            else:
                status = 'ok   (on slow retry)'
                retried += 1
                fa, fm, fb = verdict(first)
                print(f'  first attempt: a/b differ at {fa[:5]}, a/model at {fm[:5]}, '
                      f'b/model at {fb[:5]}, problems {first["problems"]}')
        n = len(res['model'])
        total += n
        published = sum(1 for a in res['a'] if a[0])
        changed = sum(1 for a, v in zip(res['a'], res['sent'])
                      if a[0] and not same_floats(a[0][0], (v[0], v[1], 0.0, 0.0, 0.0, v[-1])))
        print(f'{status}  seed={seed} zones={len(cfg["zones"])} tf={int(cfg["use_tf"])} '
              f'stamped={int(cfg["stamped"])} commands={n} published={published} '
              f'altered={changed} a!=b:{len(ab)} a!=model:{len(am)} b!=model:{len(bm)} '
              f'{time.monotonic() - t0:.1f}s', flush=True)
    rclpy.shutdown()
    print(f'{bad} divergent case(s) in {opts.seed_to - opts.seed_from} random cases, '
          f'{total} commands; {retried} case(s) passed only on the slow retry')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
