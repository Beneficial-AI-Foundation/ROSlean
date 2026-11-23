#!/usr/bin/env python3
"""Fault injection: malformed, missing and hostile input for a collision monitor.

    fault_injection.py [NODE [UPSTREAM]] [options]

NODE (default: the ROSlean node) is driven through every case below; with
UPSTREAM given (`nav2_collision_monitor/collision_monitor`) upstream runs
beside it on the same messages. Each case starts fresh processes, so that a
node that dies in one case is back for the next.

For NODE the script asserts two things: the process is alive at the end of
every case, and every command is answered as the case says. The expected
answers are the ROSlean node's as the SPEC (docs/collision_monitor/SPEC.md)
gives them. For UPSTREAM the expected answers are what its binary was seen
to do, including where it dies; they are asserted too, so that the table in
docs/collision_monitor/COMPARISON.md stays true.

Options:
  --only A,B,...   run only these cases
  --skip A,B,...   leave these cases out
  --list           list the cases
  --settle S       wall seconds per event (default 0.2)
  --big            also run the cases with very large scans (they need
                   seconds each and a lot of memory)
  --logs DIR       keep the nodes' output

Both nodes run with zones.yaml, zones StopCircle (0.5 m, stop) and
SlowCircle (1.0 m, ratio 0.5), min_points 3, source_timeout 0.5 s,
stop_pub_timeout 1 s, under simulated time.
"""

import argparse
import math
import os
import sys

import rclpy
from rosgraph_msgs.msg import Clock

from cm_harness import HERE, Driver, Side, fan, fmt, resolve, s, same_floats, scan_msg, twist6

INF, NAN = math.inf, math.nan
V = (0.5, 0.25, 1.0)
ZERO = (0.0, 0.0, 0.0)
HALF = (0.25, 0.125, 0.5)
T = s(100)
CLEAR = fan(5, 5.0)
NEAR = fan(3, 0.25)
MID = fan(3, 0.75)
DEAD = 'dead'      # the process has exited
ANY = 'any'        # not asserted (timing-dependent)
LEAN = 'roslean_collision_monitor/collision_monitor'


def clock(t_ns):
    return ('clock', t_ns)


def raw_clock(sec, nanosec):
    """A /clock message with these fields, whatever they are."""
    return ('raw_clock', sec, nanosec)


def scan(spec, **extra):
    return ('scan', dict(spec, **extra))


def cmd(vel, label, node, upstream=None):
    """A command; `node` and `upstream` are the expected answers:
    None (nothing published), three components, DEAD or ANY. `upstream`
    defaults to `node`."""
    return ('cmd', vel, label, node, node if upstream is None else upstream)


def burst(n, label):
    """n commands back to back, without waiting for answers."""
    return ('burst', n, label)


def scans(n, spec, label):
    """n scans back to back."""
    return ('scans', n, spec, label)


def full(n, value, **extra):
    """A scan of n readings over a full turn, all with the same value."""
    return dict({'ranges': [value] * n, 'angle_min': -3.14159, 'angle_increment': 6.28318 / n},
                **extra)


BIG = {}


def big(n, value):
    """As `full`, built once: these lists are large."""
    if (n, value) not in BIG:
        BIG[(n, value)] = full(n, value)
    return BIG[(n, value)]


CASES = {
    # ---- scans that do not come ---------------------------------------------
    'dropped-scans': [
        clock(T), scan(CLEAR),
        cmd(V, 'a clear scan: passes', V),
        clock(T + s(0.5)),
        cmd(V, 'no scan for exactly source_timeout: passes', V),
        clock(T + s(0.5) + 1),
        cmd(V, 'no scan for 1 ns more: stop', ZERO),
        clock(T + s(1.5) + 1),
        cmd(V, 'still none, stop_pub_timeout after the stop began: zero still published', ZERO),
        clock(T + s(1.5) + 2),
        cmd(V, '1 ns later: nothing is published any more (R15)', None),
        clock(T + s(600)),
        cmd(V, 'ten minutes without a scan: nothing', None),
        scan(CLEAR),
        cmd(V, 'the scans come back: passes', V),
    ],
    'never-a-scan': [
        clock(T),
        cmd(V, 'no scan has ever arrived: stop', ZERO),
        cmd((NAN, 0.0, 0.0), 'a NaN command meanwhile: nothing', None),
        cmd(V, 'stop', ZERO),
    ],

    # ---- bad values inside a scan -------------------------------------------
    'all-nan': [
        clock(T), scan(NEAR),
        cmd(V, 'obstacle in the stop zone: stop', ZERO),
        scan(full(360, NAN)),
        cmd(V, '360 readings, all NaN: free space, PASSES (R4; SPEC open decision 1)', V),
        scan(full(360, INF)),
        cmd(V, 'all +inf: passes', V),
        scan(full(360, -INF)),
        cmd(V, 'all -inf: passes', V),
        scan({'ranges': [NAN, INF, 0.25, -INF, 0.25, NAN, 0.25], 'angle_increment': 0.01}),
        cmd(V, 'three valid readings in the stop zone among NaN and inf: stop', ZERO),
    ],
    'limits-not-finite': [
        clock(T),
        scan(full(360, INF, range_max=INF)),
        cmd(V, 'range_max +inf and every reading +inf (valid by R4): points in no zone, passes', V),
        scan({'ranges': [-INF, INF, NAN, 1e38, -1e38, 0.25, 0.25, 0.25], 'angle_increment': 0.0,
              'range_min': -INF, 'range_max': INF}),
        cmd(V, 'limits -inf..+inf, three of eight readings in the stop zone: stop', ZERO),
        scan(dict(NEAR, range_min=NAN)),
        cmd(V, 'range_min NaN: no reading is valid, passes', V),
        scan(dict(NEAR, range_max=NAN)),
        cmd(V, 'range_max NaN: no reading is valid, passes', V),
        scan(dict(NEAR, range_min=1.0, range_max=0.1)),
        cmd(V, 'range_min above range_max: no reading is valid, passes', V),
        scan(dict(NEAR, range_min=-INF, range_max=-INF)),
        cmd(V, 'both limits -inf: no reading is valid, passes', V),
    ],
    'angles-not-finite': [
        clock(T),
        scan(dict(NEAR, angle_min=NAN)),
        cmd(V, 'angle_min NaN: every point is (NaN, NaN), in no zone, passes', V),
        scan(dict(NEAR, angle_min=INF)),
        cmd(V, 'angle_min +inf: the same', V),
        scan({'ranges': [0.25] * 5, 'angle_min': 0.0, 'angle_increment': INF}),
        cmd(V, 'angle_increment +inf: only the first of five points is real (needs 3), passes', V),
        scan({'ranges': [0.25] * 5, 'angle_min': 0.0, 'angle_increment': NAN}),
        cmd(V, 'angle_increment NaN: the same', V),
        scan({'ranges': [0.25] * 5, 'angle_min': 1e30, 'angle_increment': 1e30}),
        cmd(V, 'angles of 1e30 rad: finite cosines, five points at 0.25 m: stop', ZERO),
        scan({'ranges': [3e38] * 5, 'angle_min': 0.5, 'angle_increment': 0.1, 'range_max': INF}),
        cmd(V, 'readings of 3e38 m, valid: far points, passes', V),
        scan({'ranges': [1e-45, -0.0, 0.0], 'angle_increment': 1.0, 'range_min': 0.0}),
        cmd(V, 'a subnormal, -0.0 and 0.0 with range_min 0: three points at the origin, stop', ZERO),
    ],

    # ---- the length of `ranges` ---------------------------------------------
    'zero-length': [
        clock(T), scan(NEAR),
        cmd(V, 'obstacle in the stop zone: stop', ZERO),
        scan({'ranges': []}),
        cmd(V, 'a scan without readings replaces it: passes', V),
        scan({'ranges': [], 'angle_min': NAN, 'angle_increment': NAN, 'range_min': NAN,
              'range_max': NAN}),
        cmd(V, 'without readings and with NaN in every field: passes', V),
    ],
    'length-mismatch': [
        clock(T),
        scan({'ranges': [5.0] * 1077 + [0.25] * 3, 'angle_min': -3.14159,
              'angle_increment': 6.28318 / 360, 'angle_max': 3.14159}),
        cmd(V, '1080 readings where the angle fields allow 360; the last three are near: stop', ZERO),
        scan({'ranges': [0.25] * 3, 'angle_min': -3.14159, 'angle_increment': 6.28318 / 1080,
              'angle_max': 3.14159}),
        cmd(V, '3 readings where the angle fields allow 1080: used, stop', ZERO),
        scan({'ranges': [0.25, 0.25], 'angle_min': -3.14159, 'angle_increment': 6.28318 / 1080,
              'angle_max': 3.14159}),
        cmd(V, '2 readings in the zones (3 are needed): passes', V),
        scan({'ranges': [0.25] * 3, 'angle_min': 1.0, 'angle_increment': 0.1, 'angle_max': -5.0}),
        cmd(V, 'angle_max below angle_min: not read, stop', ZERO),
    ],
    'scan-10000': [
        clock(T),
        scan(full(10000, 5.0)),
        cmd(V, '10,000 readings at 5 m: passes', V),
        scan(full(10000, 0.75)),
        cmd(V, '10,000 readings at 0.75 m: slowed', HALF),
        scan(full(10000, 0.25)),
        cmd(V, '10,000 readings at 0.25 m: stop', ZERO),
    ],

    # ---- stamps -------------------------------------------------------------
    'negative-stamp': [
        clock(T), scan(CLEAR),
        cmd(V, 'a clear scan: passes', V),
        scan(CLEAR, raw_stamp=(-1, 0)),
        cmd(V, 'a clear scan stamped -1 s: ROSlean takes the stamp as 0, too old at 100 s, '
               'stop (D8); upstream aborts', ZERO, DEAD),
        scan(CLEAR),
        cmd(V, 'the next good scan: passes; upstream is gone', V, DEAD),
    ],
    'negative-stamp-nanosec': [
        clock(T),
        scan(CLEAR, raw_stamp=(-1, 2_000_000_000)),
        cmd(V, 'sec -1 with nanosec 2e9 (sum +1 s): too old at 100 s, stop; upstream aborts',
            ZERO, DEAD),
        scan(CLEAR, raw_stamp=(-2 ** 31, 0)),
        cmd(V, 'sec at the most negative value: stop', ZERO, DEAD),
    ],
    # D8: while the clock is below source_timeout a scan with a negative stamp
    # counts as fresh for ROSlean. This is the documented case.
    'negative-stamp-early-clock': [
        clock(s(0.25)),
        scan(NEAR, raw_stamp=(-5, 0)),
        cmd(V, 'clock at 0.25 s, obstacle scan stamped -5 s: ROSlean uses it (stamp 0, age '
               '0.25 s): stop; upstream aborts', ZERO, DEAD),
        scan(CLEAR, raw_stamp=(-5, 0)),
        cmd(V, 'a CLEAR scan stamped -5 s: ROSlean uses it and PASSES (D8)', V, DEAD),
        clock(s(0.5) + 1),
        cmd(V, 'clock past source_timeout: the scan is too old, stop', ZERO, DEAD),
    ],
    'far-future-stamp': [
        clock(T),
        scan(CLEAR, raw_stamp=(2 ** 31 - 1, 999_999_999)),
        cmd(V, 'a clear scan stamped in 2038: used', V),
        clock(T + s(100000)),
        cmd(V, '... and still used 100,000 s later with no further scan (R7: never too old)', V),
        scan(NEAR, raw_stamp=(2 ** 31 - 1, 4_294_967_295)),
        cmd(V, 'nanosec at its largest value, obstacle: stop', ZERO),
    ],
    'stamps-out-of-order': [
        clock(T),
        scan(NEAR), scan(CLEAR, age=s(0.25)),
        cmd(V, 'an older clear scan arrives after a newer obstacle scan: the last one received '
               'counts, passes (R5)', V),
        scan(CLEAR), scan(NEAR, age=s(0.25)),
        cmd(V, 'the other way round: stop', ZERO),
    ],

    # ---- commands -----------------------------------------------------------
    'non-finite-commands': [
        clock(T), scan(NEAR),
        cmd(V, 'obstacle in the stop zone: stop', ZERO),
        cmd((NAN, 0.0, 0.0), 'linear.x NaN: nothing (R2)', None),
        cmd((0.0, INF, 0.0), 'linear.y +inf: nothing', None),
        cmd((0.0, 0.0, -INF), 'angular.z -inf: nothing', None),
        cmd((0.5, 0.0, NAN, 0.0, 0.0, 0.0), 'linear.z NaN: nothing', None),
        cmd((0.5, 0.0, 0.0, -INF, 0.0, 0.0), 'angular.x -inf: nothing', None),
        cmd((0.5, 0.0, 0.0, 0.0, INF, 0.0), 'angular.y +inf: nothing', None),
        cmd((NAN,) * 6, 'all six NaN: nothing', None),
        cmd(V, 'the next finite command: stop', ZERO),
        scan(CLEAR),
        cmd((1.7e308, -1.7e308, 1.7e308), 'the largest finite values: pass', (1.7e308, -1.7e308, 1.7e308)),
        cmd((5e-324, 0.0, 0.0), 'the smallest subnormal: passes', (5e-324, 0.0, 0.0)),
    ],
    'extreme-commands-slowdown': [
        clock(T), scan(MID),
        cmd(V, 'obstacle in the slowdown zone: halved', HALF),
        cmd((1.7e308, -1.7e308, 1.7e308), 'the largest finite values: halved by ROSlean; '
            'upstream passes them unslowed, the squared norm being infinite (D6)',
            (8.5e307, -8.5e307, 8.5e307), (1.7e308, -1.7e308, 1.7e308)),
        cmd((5e-324, 0.0, 0.0), 'the smallest subnormal: its half rounds to 0.0 for ROSlean; '
            'upstream passes it unslowed (D6)', ZERO, (5e-324, 0.0, 0.0)),
        cmd((NAN, 0.0, 0.0), 'NaN: nothing', None),
    ],
    'command-burst': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        burst(1000, '1000 commands back to back'),
        cmd(V, 'a command after the burst is answered', V),
        scan(NEAR),
        burst(1000, '1000 commands back to back with an obstacle'),
        cmd(V, 'a command after the burst: stop', ZERO),
    ],
    'scan-burst': [
        clock(T),
        scans(500, NEAR, '500 obstacle scans back to back'),
        cmd(V, 'stop', ZERO),
        scans(500, CLEAR, '500 clear scans back to back'),
        cmd(V, 'passes', V),
    ],

    # ---- frames -------------------------------------------------------------
    'foreign-frame': [
        clock(T), scan(CLEAR),
        cmd(V, 'a clear scan in the base frame: passes', V),
        scan(CLEAR, frame='laser'),
        cmd(V, 'a clear scan in a frame that is not the base frame, no transform, no pose: '
               'not used, stop (D11 / R8)', ZERO),
        scan(CLEAR),
        cmd(V, 'in the base frame again: passes', V),
        scan(CLEAR, frame=''),
        cmd(V, 'an empty frame name: stop', ZERO),
        scan(CLEAR, frame='base_footprint '),
        cmd(V, 'the base frame name with a trailing blank: stop', ZERO),
        scan(CLEAR, frame='/base_footprint'),
        cmd(V, 'the base frame name with a leading slash: stop', ZERO),
        scan(CLEAR, frame='x' * 100000),
        cmd(V, 'a frame name of 100,000 characters: stop', ZERO),
    ],

    # ---- the clock ----------------------------------------------------------
    'clock-backwards': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        clock(T + s(0.5) + 1),
        cmd(V, 'the scan is 1 ns too old: stop', ZERO),
        clock(T - s(50)),
        cmd(V, 'the clock jumps back 50 s: the scan has a negative age and is used again (A2)', V),
        scan(NEAR, stamp=T),
        cmd(V, 'obstacle: stop; the stop period starts at T - 50 s', ZERO),
        clock(T),
        cmd(V, '50 s later, same scan: zero has been due for longer than stop_pub_timeout: nothing',
            None),
        clock(T - s(100)),
        cmd(V, 'the clock jumps back before the stop began: zero is published again', ZERO),
        clock(0),
        cmd(V, 'the clock jumps to 0: zero published', ZERO),
        clock(1),
        cmd(V, 'the clock at 1 ns', ZERO),
    ],
    # A /clock message with a negative time. The expectation for the node is
    # that it survives. At present it does NOT: rclcpp's time source throws
    # "cannot store a negative time point in rclcpp::Time" in its clock
    # thread, nothing catches it, and the process aborts, as upstream's does
    # (docs/collision_monitor/COMPARISON.md, defect 1). The case is therefore
    # registered as a test that is expected to fail.
    'clock-negative': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        raw_clock(-5, 0),
        cmd(V, 'after a /clock message of -5 s the node must still be there '
               '(any answer); upstream aborts', ANY, DEAD),
    ],
}

BIG_CASES = {
    'scan-100000': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        ('scan', lambda: big(100_000, 0.25)),
        cmd(V, '100,000 readings (400 kB) at 0.25 m: stop if the message was delivered', ZERO, ANY),
    ],
    'scan-1000000': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        ('scan', lambda: big(1_000_000, 0.25)),
        cmd(V, '1,000,000 readings (4 MB) at 0.25 m', ANY, ANY),
        cmd(V, 'a second command', ANY, ANY),
        scan(CLEAR),
        cmd(V, 'an ordinary clear scan afterwards: passes', V, ANY),
    ],
    'scan-10000000': [
        clock(T), scan(CLEAR),
        cmd(V, 'passes', V),
        ('scan', lambda: big(10_000_000, 0.25)),
        cmd(V, '10,000,000 readings (40 MB) at 0.25 m', ANY, ANY),
        scan(CLEAR),
        cmd(V, 'an ordinary clear scan afterwards: passes', V, ANY),
    ],
}


def make_scan(spec, now_ns, frame):
    if callable(spec):
        spec = spec()
    m = scan_msg(spec, now_ns, frame)
    if 'raw_stamp' in spec:
        m.header.stamp.sec, m.header.stamp.nanosec = spec['raw_stamp']
    return m


def answer(d, side, outs):
    if side.proc.poll() is not None:
        return DEAD
    if not outs:
        return None
    if len(outs) > 1:
        return ('several', len(outs))
    t = twist6(d.twist_of(outs[0]))
    if t[2] != 0.0 or t[3] != 0.0 or t[4] != 0.0:
        return ('bad', t)
    return (t[0], t[1], t[5])


def agrees(got, want):
    if want == ANY:
        return True
    if want is None or want == DEAD or got is None or got == DEAD:
        return got == want
    return len(got) == 3 and len(want) == 3 and same_floats(got, want)


def show(a):
    return a if isinstance(a, str) else 'nothing' if a is None else fmt(a)


def run_case(name, steps, exes, opts):
    sides = [Side(exe, f'cm_fault_{k}', f'fault_out_{k}', None) for k, exe in enumerate(exes)]
    for k, side in enumerate(sides):
        log = os.path.join(opts.logs, f'{name}_{k}.log') if opts.logs else None
        side.start(os.path.join(HERE, 'zones.yaml'), ['polygons:=[StopCircle,SlowCircle]'], log)
    d = Driver('fault_driver', sides, False)
    lines = []
    try:
        for side in sides:
            if d.bring_up(side) != 'active':
                raise RuntimeError(f'{side.exe} refused the configuration')
        d.wait_connected()
        ident = 0
        for step in steps:
            if step[0] == 'clock':
                d.set_clock(step[1], opts.settle)
            elif step[0] == 'raw_clock':
                m = Clock()
                m.clock.sec, m.clock.nanosec = step[1], step[2]
                d.clock_pub.publish(m)
                d.spin_for(max(opts.settle, 1.0))
            elif step[0] == 'scan':
                msg = make_scan(step[1], d.now_ns, d.scan_frame)
                d.scan_pub.publish(msg)
                # A large message needs longer to arrive and to be taken in.
                d.spin_for(opts.settle + len(msg.ranges) / 500_000)
            elif step[0] == 'scans':
                for _ in range(step[1]):
                    d.scan_pub.publish(make_scan(step[2], d.now_ns, d.scan_frame))
                d.spin_for(1.0)
                lines.append((True, f'{step[3]}: sent'))
            elif step[0] == 'burst':
                marks = [len(sd.out) for sd in sides]
                for _ in range(step[1]):
                    ident += 1
                    d.cmd_pub.publish(d.make_cmd(V, ident))
                d.spin_for(2.0)
                counts = [len(sd.out) - m for sd, m in zip(sides, marks)]
                alive = [sd.proc.poll() is None for sd in sides]
                lines.append((all(alive) and all(c >= 1 for c in counts),
                              f'{step[2]}: answers {counts}, alive {alive}'))
            else:
                _, vel, label, *want = step
                ident += 1
                n = len(sides)
                answers = d.send_cmd(
                    d.make_cmd(vel, ident), opts.settle,
                    wait_for=lambda i, o, st: want[i] in (None, DEAD) or len(o) >= 1, max_wait=5.0)
                for sd, w in zip(sides, want):
                    if w == DEAD:  # an abort takes a while to be over
                        d.spin_until(lambda: sd.proc.poll() is not None, 60.0)
                got = [answer(d, sd, outs) for sd, (outs, _) in zip(sides, answers)]
                # ANY leaves the answer open, not whether the process is there.
                ok = all(agrees(g, w) and not (w == ANY and g == DEAD)
                         for g, w in zip(got, want[:n]))
                text = f'{label}\n        node: {show(got[0])}'
                if n > 1:
                    text += f'   upstream: {show(got[1])}'
                if not ok:
                    text += '   [expected ' + ' / '.join(show(w) for w in want[:n]) + ']'
                lines.append((ok, text))
        d.spin_for(opts.settle)
        code = sides[0].proc.poll()
        expected_dead = any(st[0] == 'cmd' and st[3] == DEAD for st in steps)
        lines.append(((code is None) != expected_dead,
                      'the node is alive at the end' if code is None
                      else f'the node has exited with code {code}'))
    except RuntimeError as e:
        lines.append((False, str(e)))
    finally:
        for side in sides:
            side.stop()
        d.destroy_node()
    print(f'--- {name}')
    for ok, text in lines:
        print(f"{'ok  ' if ok else 'FAIL'}  {text}")
    sys.stdout.flush()
    return len(lines), sum(not ok for ok, _ in lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node', nargs='?', default=LEAN)
    ap.add_argument('upstream', nargs='?', default='')
    ap.add_argument('--only', default='')
    ap.add_argument('--skip', default='')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--settle', type=float, default=0.2)
    ap.add_argument('--big', action='store_true')
    ap.add_argument('--logs', default='')
    opts = ap.parse_args()
    cases = dict(CASES, **(BIG_CASES if opts.big else {}))
    if opts.list:
        print('\n'.join(cases))
        return
    names = [n for n in opts.only.split(',') if n] or list(cases)
    skip = [n for n in opts.skip.split(',') if n]
    unknown = [n for n in names + skip if n not in cases]
    if unknown:
        sys.exit(f'unknown case(s): {unknown}')
    names = [n for n in names if n not in skip]
    if opts.logs:
        os.makedirs(opts.logs, exist_ok=True)
    exes = [resolve(opts.node)] + ([resolve(opts.upstream)] if opts.upstream else [])
    print('node: ' + exes[0] + (f'\nupstream: {exes[1]}' if opts.upstream else ''))
    rclpy.init()
    checks = bad = 0
    failed = []
    try:
        for name in names:
            n, b = run_case(name, cases[name], exes, opts)
            checks += n
            bad += b
            if b:
                failed.append(name)
    finally:
        rclpy.shutdown()
    print(f'{bad} failure(s) in {checks} checks, {len(names)} cases'
          + (f'; failed: {failed}' if failed else ''))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
