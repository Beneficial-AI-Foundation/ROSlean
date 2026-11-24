#!/usr/bin/env python3
"""Randomised test of the Nav2 extension: transforms, base shift correction,
zones fed by a topic and lifecycle transitions, interleaved with scans and
commands.

    random_nav2.py [NODE [OTHER]] [SEED_FROM SEED_TO] [options]

NODE and OTHER are executable paths or `<package>/<executable>`; the default
is upstream's `nav2_collision_monitor/collision_monitor` alone. It is the
extension's counterpart of random_diff.py: for each seed a random
configuration and a random sequence of events are generated, the node (or
both nodes, side by side on the same messages) is driven through it under
simulated time, and every answer to a command, velocity bit for bit and
state message, is compared with the Python model below, which renders
SPEC_NAV2.md N4-N7, N13, N23-N27 and N32-N35 on top of cm_model.py. Two
nodes are thereby also compared with each other. A case that disagrees is
run once more with a long settle time, to separate harness timing from a
real difference.

Options:
  --settle S     wall seconds per event (default 0.05)
  --logs DIR     keep the nodes' output
  --self-test    judge wrongly on purpose (the model ignores base shift
                 correction): the run succeeds if a disagreement is reported

Coverage per case: 1 to 3 zones (circles, fixed polygons, polygons fed by
`polygon_sub_topic`; stop and slowdown; min_points 1 to 3); the scan in the
base frame or in a sensor frame with a static transform whose rotation is
exact (identity, half turn), a general yaw, upside down, not normalised or
a general 3D rotation; base shift correction on or off, with the base
moving and turning in the odom frame between the events; clock ticks
without an odom sample, so that the correction's lookup fails; scans stamped
now or at an earlier odom sample, and scans in a frame without transform;
shape messages in the base frame, in a static frame, in the odom frame (the
footprint's case: the zone is then left behind when the base moves) and in
a frame without transform, and messages with two points; deactivate and
activate, and cleanup, configure, activate, at random places; source
timeouts including 0; non-finite commands.

What is kept away: `transform_tolerance` is 0, so no command waits (the
waits are in scenario_nav2.py); the clock never runs backwards (that clears
the TF buffer); a command is not generated when a point would lie within
5 mm of a zone boundary, when its lookup would need interpolation between
odom samples, or when a stamp lies at the edge of tf2's 10 s history. So
the 5 mm that SPEC.md D3/D4 and SPEC_NAV2.md ND1 allow the ROSlean node are
excluded, and both nodes must give the model's answer exactly. Not
generated: shutdown (ND4), non-finite shapes (ND6), zones with equal names
(ND5). Polygon topics are not compared (scenario_nav2.py does that).
"""

import argparse
import math
import os
import random
import sys
import tempfile

import rclpy
from lifecycle_msgs.msg import Transition

from cm_harness import NS, UPSTREAM, Driver, Side, f32, fmt, resolve, same_floats, twist6, xf
from cm_model import DO_NOTHING, INVALID_SOURCE, SLOWDOWN, STOP, Zone, scan_points, to_ns
from random_diff import boundary_distance, polygon_points, zone_yaml

STEP = 50_000_000
SOURCE_TIMEOUTS = [0.0, 0.525, 1.025, 2.075]
STOP_PUB_TIMEOUTS = [0.075, 0.225, 0.475, 1.025]
SIZES = [0.5, 0.75, 1.0]
RATIOS = [0.5, 0.25, 0.3, 0.75]
SPEEDS = [0.0, 0.0, 0.5, -0.5, 0.25, 1.0, -1.0, 0.1]
MARGIN = 0.005
HISTORY = 10 * NS      # tf2's cache time
BASE = 'base_footprint'
SQ = math.sqrt(0.5)


def yaw_q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


LASER_ROTATIONS = [
    (0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 1.0, 0.0), (0.0, 0.0, SQ, SQ), yaw_q(0.3), yaw_q(-2.5),
    (1.0, 0.0, 0.0, 0.0),            # upside down
    (0.0, 0.0, 0.0, 2.0),            # not normalised: the identity
    (0.1, 0.2, 0.3, 0.9),            # a general 3D rotation, not normalised
]


def matrix(q):
    """Rows 0 and 1, columns 0 and 1, of the rotation of a quaternion (SPEC_NAV2 N25)."""
    qx, qy, qz, qw = q
    s = 2.0 / (qx * qx + qy * qy + qz * qz + qw * qw)
    xs, ys, zs = qx * s, qy * s, qz * s
    return ((1.0 - (qy * ys + qz * zs), qx * ys - qw * zs),
            (qx * ys + qw * zs, 1.0 - (qx * xs + qz * zs)))


def to_odom(pose, p):
    x, y, a = pose
    return (math.cos(a) * p[0] - math.sin(a) * p[1] + x, math.sin(a) * p[0] + math.cos(a) * p[1] + y)


def to_base(pose, p):
    x, y, a = pose
    dx, dy = p[0] - x, p[1] - y
    return (math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy)


class Nav2Model:
    """What upstream does with the events of this test. `ambiguous()` tells the
    generator that the next command is too close to call."""

    def __init__(self, cfg, ignore_shift=False):
        self.cfg = cfg
        self.zones = cfg['zones']
        self.source_timeout = to_ns(cfg['source_timeout'])
        self.stop_pub_timeout = to_ns(cfg['stop_pub_timeout'])
        self.shift = cfg['shift'] and not ignore_shift
        self.now = 0
        self.active = True
        self.scan = None                 # (stamp, frame, points in the scan frame)
        self.shapes = {}                 # zone index -> (frame, vertices)
        self.odom = {}                   # stamp -> pose of the base in odom
        self.prev_name, self.prev_zero, self.stop_stamp = '', False, 0

    # -- events ---------------------------------------------------------------

    def clock(self, t):
        self.now = t

    def odom_in(self, stamp, pose):
        self.odom[stamp] = pose
        newest = max(self.odom)
        self.odom = {t: p for t, p in self.odom.items() if t >= newest - HISTORY}

    def scan_in(self, stamp, frame, spec):
        self.scan = (stamp, frame, scan_points(
            spec['ranges'], spec['angle_min'], spec['angle_increment'],
            spec['range_min'], spec['range_max']))

    def shape_in(self, index, frame, points):
        if len(points) < 3:
            return
        if frame == 'nowhere' or (frame == 'odom' and not self.odom):
            return
        self.shapes[index] = (frame, [(f32(x), f32(y)) for x, y in points])

    def transition(self, name):
        if name == 'activate':
            self.active = True
        else:
            # N6: the previous action is reset, so the next zero starts a new
            # stop_pub_timeout period and the state is reported anew.
            self.active = False
            self.prev_name, self.prev_zero = '', False
        if name == 'recycle':            # cleanup: scan, shapes and TF buffer are gone
            self.scan, self.shapes, self.odom = None, {}, {}
            self.active = True

    # -- a command ------------------------------------------------------------

    def lookup(self, stamp):
        """The base's pose in odom at a stamp: ('ok', pose), ('fail',) or ('unsure',)."""
        if stamp in self.odom:
            newest = max(self.odom)
            if stamp < newest - HISTORY + NS // 2:
                return ('unsure',)
            return ('ok', self.odom[stamp])
        if self.odom and min(self.odom) < stamp < max(self.odom):
            return ('unsure',)           # tf2 would interpolate
        if self.odom and stamp < min(self.odom) and stamp > max(self.odom) - HISTORY - NS // 2:
            return ('unsure',)
        return ('fail',)

    def points(self):
        """(points in the base frame or None if the source is invalid, sure)."""
        if self.scan is None:
            return None, True
        stamp, frame, pts = self.scan
        if self.source_timeout != 0 and self.now - stamp > self.source_timeout:
            return None, True
        laser = self.cfg['laser']
        if frame == 'nowhere':
            return None, True
        if frame != BASE:
            (tx, ty, q) = laser
            m = matrix(q)
            pts = [(m[0][0] * x + m[0][1] * y + tx, m[1][0] * x + m[1][1] * y + ty) for x, y in pts]
        if self.shift:
            a, b = self.lookup(self.now), self.lookup(stamp)
            if a[0] == 'fail' or b[0] == 'fail':
                return None, True
            if a[0] == 'unsure' or b[0] == 'unsure':
                return None, False
            pts = [to_base(a[1], to_odom(b[1], p)) for p in pts]
        return pts, True

    def zone_now(self, index):
        """The zone as it is tested at this command, or None if it has no shape."""
        z = self.zones[index]
        if z.kind != 'topic':
            return z
        if index not in self.shapes:
            return None
        frame, verts = self.shapes[index]
        if frame == 'shapef':
            ox, oy = self.cfg['shapef']
            verts = [(x + ox, y + oy) for x, y in verts]
        elif frame == 'odom':
            if not self.odom:
                return None     # cannot happen: a shape in odom is accepted only with a sample
            pose = self.odom[max(self.odom)]
            verts = [to_base(pose, v) for v in verts]
        out = Zone(z.name, 'polygon', [(0.0, 0.0)], z.action, z.ratio, z.min_points)
        out.poly = verts
        return out

    def ambiguous(self):
        if not self.active:
            return False
        pts, sure = self.points()
        if not sure:
            return True
        if pts is None:
            return False
        for i in range(len(self.zones)):
            z = self.zone_now(i)
            if z is not None and any(
                    not (math.isfinite(p[0]) and math.isfinite(p[1]))
                    or boundary_distance(z, p) < MARGIN for p in pts):
                return True
        return False

    def cmd(self, vel):
        """(velocity or None, state or None), as cm_model.Model.cmd."""
        if not self.active or not all(math.isfinite(c) for c in vel):
            return None, None
        req, name, action = vel, '', DO_NOTHING
        pts, _ = self.points()
        if pts is None:
            pts = []
            if self.source_timeout != 0:
                req, name, action = (0.0, 0.0, 0.0), INVALID_SOURCE, STOP
        if action != STOP:
            for i in range(len(self.zones)):
                z = self.zone_now(i)
                if z is None or sum(z.inside(p) for p in pts) < z.min_points:
                    continue
                if z.action == 'stop':
                    req, name, action = (0.0, 0.0, 0.0), z.name, STOP
                    break
                slow = tuple(c * z.ratio for c in vel)
                if sum(c * c for c in slow) < sum(c * c for c in req):
                    req, name, action = slow, z.name, SLOWDOWN
        state = (name, action) if name != self.prev_name else None
        zero = all(c == 0.0 for c in req)
        publish = True
        if zero:
            if not self.prev_zero:
                self.stop_stamp = self.now
            elif self.now - self.stop_stamp > self.stop_pub_timeout:
                publish = False
        self.prev_name, self.prev_zero = name, zero
        return ((req[0], req[1], 0.0, 0.0, 0.0, req[2]) if publish else None), state


def apply(model, ev):
    """Feed one event to the model; a command's answer is returned."""
    if ev[0] == 'clock':
        model.clock(ev[1])
    elif ev[0] == 'odom':
        model.odom_in(ev[1], ev[2])
    elif ev[0] == 'scan':
        model.scan_in(ev[1]['stamp'], ev[1]['frame'], ev[1])
    elif ev[0] == 'shape':
        model.shape_in(ev[1], ev[3], ev[2])
    elif ev[0] == 'lc':
        model.transition(ev[1])
    else:
        return model.cmd(ev[1])
    return None


def gen(seed):
    r = random.Random(seed)
    zones = []
    for i in range(r.randint(1, 3)):
        kind = r.choice(['circle', 'polygon', 'topic', 'topic'])
        size = r.choice(SIZES)
        geometry = size if kind == 'circle' else polygon_points(
            r.choice(['square', 'front', 'pentagon']), size)
        z = Zone(f'Zone{i}', 'circle' if kind == 'circle' else 'polygon', geometry,
                 r.choice(['stop', 'slowdown', 'slowdown']), r.choice(RATIOS), r.choice([1, 1, 2, 3]))
        z.kind = kind
        zones.append(z)
    laser = None
    if r.random() < 0.6:
        laser = (r.choice([0.25, -0.25, 0.5, 0.0]), r.choice([0.0, 0.125, -0.25]),
                 r.choice(LASER_ROTATIONS))
    cfg = {'zones': zones, 'laser': laser, 'shift': r.random() < 0.5,
           'shapef': (r.choice([0.5, -0.25, 1.0]), r.choice([0.0, 0.25])),
           'source_timeout': r.choice(SOURCE_TIMEOUTS),
           'stop_pub_timeout': r.choice(STOP_PUB_TIMEOUTS)}
    scan_frame = 'laser' if laser else BASE
    topic_zones = [i for i, z in enumerate(zones) if z.kind == 'topic']
    model = Nav2Model(cfg)
    events = []
    now = r.choice([0, 100, 1_800_000_000]) * NS + STEP
    pose = (0.0, 0.0, 0.0)

    def emit(ev):
        events.append(ev)
        apply(model, ev)

    def tick(sample=True):
        emit(('clock', now))
        if sample:
            emit(('odom', now, pose))

    def rand_scan():
        n = r.choice([1, 2, 3, 3, 4, 6])
        near = r.random() < 0.7
        ranges = [math.nan if r.random() < 0.05 else f32(r.uniform(0.1, 1.2 if near else 3.0))
                  for _ in range(n)]
        spec = {'ranges': ranges, 'angle_min': f32(r.uniform(-math.pi, math.pi)),
                'angle_increment': f32(r.choice([0.0, 0.01, 0.05, 0.3, -0.2])),
                'range_min': 0.05, 'range_max': 10.0, 'stamp': now,
                'frame': 'nowhere' if r.random() < 0.05 else scan_frame}
        if model.odom and r.random() < 0.3:
            spec['stamp'] = r.choice(sorted(model.odom)[-8:])     # an earlier odom sample
        return spec

    def rand_shape(index):
        size = r.choice(SIZES)
        pts = polygon_points(r.choice(['square', 'front', 'pentagon']), size)
        if r.random() < 0.15:
            pts = [(size, 0.25), (-0.25, -size), (-0.25, size)]   # three points are enough
        x = r.random()
        if x < 0.1:
            return ('shape', index, pts[:2], BASE)
        if x < 0.2:
            return ('shape', index, pts, 'nowhere')
        if x < 0.45:
            return ('shape', index, pts, 'shapef')
        if x < 0.75:
            # As the costmap publishes the footprint: around the base, in odom.
            return ('shape', index, [to_odom(pose, p) for p in pts], 'odom')
        return ('shape', index, pts, BASE)

    tick()
    for _ in range(r.randint(120, 180)):
        x = r.random()
        if not model.active and x < 0.2:      # inactive for five events on average
            emit(('lc', 'activate'))
        elif x < 0.22:
            now += r.choice([1, 1, 1, 2, 2, 3, 5, 8, 14, 27]) * STEP
            if r.random() < 0.5:
                pose = (pose[0] + r.choice([0.0, 0.125, 0.25, 0.5, -0.25]),
                        pose[1] + r.choice([0.0, 0.0, 0.125, -0.25]),
                        r.choice([pose[2], 0.0, math.pi / 2, math.pi, 0.3, -1.0]))
            tick(sample=r.random() < 0.9)
        elif x < 0.47:
            emit(('scan', rand_scan()))
        elif x < 0.57 and topic_zones:
            emit(rand_shape(r.choice(topic_zones)))
        elif x < 0.61:
            if model.active:
                emit(('lc', r.choice(['deactivate', 'deactivate', 'recycle'])))
            else:
                emit(('lc', 'activate'))
        else:
            vel = [r.choice(SPEEDS) for _ in range(3)]
            if r.random() < 0.03:
                vel[r.randrange(3)] = r.choice([math.nan, math.inf])
            if not model.ambiguous():
                emit(('cmd', tuple(vel)))
    return cfg, events


def write_yaml(cfg, tag, path):
    lines = [
        '/**:', '  ros__parameters:',
        f'    base_frame_id: {BASE}', '    odom_frame_id: odom',
        '    transform_tolerance: 0.0',
        f'    source_timeout: {cfg["source_timeout"]!r}',
        f'    stop_pub_timeout: {cfg["stop_pub_timeout"]!r}',
        f'    base_shift_correction: {"true" if cfg["shift"] else "false"}',
        '    observation_sources: ["scan"]', '    scan:', '      type: "scan"',
        '    polygons: [%s]' % ', '.join(f'"{z.name}"' for z in cfg['zones']),
    ]
    for i, z in enumerate(cfg['zones']):
        if z.kind != 'topic':
            lines += zone_yaml(z)
            continue
        lines += [f'    {z.name}:', '      type: "polygon"',
                  f'      polygon_sub_topic: "{tag}_shape{i}"',
                  f'      action_type: "{z.action}"', f'      slowdown_ratio: {z.ratio!r}',
                  f'      min_points: {z.min_points}']
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def show(ans):
    return ' + '.join([fmt(v) for v in ans[0]] + [f'state{st}' for st in ans[1]]) or 'nothing'


def run_case(tag, cfg, events, exes, opts, settle):
    """The problems found: a list of texts (empty: every answer is the model's)."""
    path = os.path.join(tempfile.mkdtemp(prefix='roslean_cm_rnav2_'), f'{tag}.yaml')
    write_yaml(cfg, tag, path)
    sides = []
    for letter, exe in zip('ab', exes):
        side = Side(exe, f'cm_{tag}_{letter}', f'{tag}_out_{letter}', f'{tag}_state_{letter}')
        log = os.path.join(opts.logs, f'{tag}_{letter}.log') if opts.logs else None
        side.start(path, [f'cmd_vel_in_topic:={tag}_cmd', f'scan.topic:={tag}_scan'], log)
        sides.append(side)
    d = Driver('rnav2_' + tag, sides, False, cmd_topic=f'{tag}_cmd', scan_topic=f'{tag}_scan')
    model = Nav2Model(cfg, ignore_shift=opts.self_test)
    problems = []
    topics = {i: f'{tag}_shape{i}' for i, z in enumerate(cfg['zones']) if z.kind == 'topic'}
    n = len(sides)

    def connected():
        return (d.cmd_pub.get_subscription_count() == n
                and d.scan_pub.get_subscription_count() == n
                and all(d.shape_publisher(t).get_subscription_count() == n
                        for t in topics.values())
                and all(len(d.endpoints_of(sd, '/tf', False)) == 1
                        and len(d.endpoints_of(sd, '/tf_static', False)) == 1
                        and d.count_publishers(sd.out_topic) == 1 for sd in sides))

    def transition(tid):
        for side in sides:
            if not d.change_state(side, tid):
                raise RuntimeError(f'{side.name}: transition {tid} failed')

    def wait_connected():
        if not d.spin_until(connected, 20.0):
            raise RuntimeError('the nodes\' endpoints did not appear')
        d.spin_for(0.5)

    try:
        statics = [xf(BASE, 'shapef', x=cfg['shapef'][0], y=cfg['shapef'][1])]
        if cfg['laser']:
            tx, ty, q = cfg['laser']
            statics.append(xf(BASE, 'laser', x=tx, y=ty, q=q))
        d.publish_tf(statics)
        for side in sides:
            if not d.detect_lifecycle(side, 'auto'):
                return [f'{side.name} offers no lifecycle services']
        if not d.spin_until(lambda: d.clock_pub.get_subscription_count() >= n, 20.0):
            return ['the nodes did not subscribe to /clock']
        transition(Transition.TRANSITION_CONFIGURE)
        wait_connected()
        transition(Transition.TRANSITION_ACTIVATE)
        d.spin_for(0.3)
        count = 0
        for index, ev in enumerate(events):
            want = apply(model, ev)
            if ev[0] == 'clock':
                d.set_clock(ev[1], settle)
            elif ev[0] == 'odom':
                x, y, a = ev[2]
                d.publish_dynamic([xf('odom', BASE, x=x, y=y, q=yaw_q(a), stamp=ev[1])], settle)
            elif ev[0] == 'scan':
                d.scan_frame = ev[1]['frame']
                d.send_scan(ev[1], settle)
            elif ev[0] == 'shape':
                d.send_shape(topics[ev[1]], ev[2], ev[3], settle)
            elif ev[0] == 'lc':
                if ev[1] == 'activate':
                    transition(Transition.TRANSITION_ACTIVATE)
                else:
                    transition(Transition.TRANSITION_DEACTIVATE)
                    if ev[1] == 'recycle':
                        transition(Transition.TRANSITION_CLEANUP)
                        transition(Transition.TRANSITION_CONFIGURE)
                        wait_connected()
                        transition(Transition.TRANSITION_ACTIVATE)
                d.spin_for(max(settle, 0.1))
            else:
                count += 1
                n_out = 0 if want[0] is None else 1
                answers = d.send_cmd(
                    d.make_cmd(ev[1], count), settle,
                    wait_for=(lambda i, o, st: len(o) >= n_out) if n_out else None, max_wait=1.0)
                exp = ([] if want[0] is None else [want[0]], [] if want[1] is None else [want[1]])
                for side, (outs, states) in zip(sides, answers):
                    got = ([twist6(m) for m in outs],
                           [(m.polygon_name, m.action_type) for m in states])
                    if not (len(got[0]) == len(exp[0])
                            and all(same_floats(a, b) for a, b in zip(got[0], exp[0]))
                            and got[1] == exp[1]):
                        problems.append(
                            f'command {count} (event {index}) {fmt(ev[1])} at clock {model.now}: '
                            f'{side.name} {show(got)}, model {show(exp)}')
            if len(problems) >= 5:
                break
        d.spin_for(0.3)
        for side in sides:
            if side.proc.poll() is not None:
                problems.append(f'{side.name} died with {side.proc.returncode}')
    except RuntimeError as e:
        problems.append(str(e))
    finally:
        for side in sides:
            side.stop()
        d.destroy_node()
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('args', nargs='*')
    ap.add_argument('--settle', type=float, default=0.05)
    ap.add_argument('--logs', default='')
    ap.add_argument('--self-test', action='store_true')
    opts = ap.parse_args()
    nodes = [a for a in opts.args if not a.lstrip('-').isdigit()] or [UPSTREAM]
    seeds = [int(a) for a in opts.args if a.lstrip('-').isdigit()] or [0, 20]
    if len(nodes) > 2 or len(seeds) != 2:
        sys.exit('usage: random_nav2.py [NODE [OTHER]] [SEED_FROM SEED_TO]')
    if opts.logs:
        os.makedirs(opts.logs, exist_ok=True)
    exes = [resolve(n) for n in nodes]
    print('nodes under test: ' + ', '.join(exes))
    rclpy.init()
    bad, commands, retried, kinds, answers = [], 0, 0, {}, {}
    try:
        for seed in range(*seeds):
            cfg, events = gen(seed)
            n_cmd = sum(ev[0] == 'cmd' for ev in events)
            commands += n_cmd
            replay = Nav2Model(cfg)
            for ev in events:
                key = ev[1] if ev[0] == 'lc' else ev[0]
                kinds[key] = kinds.get(key, 0) + 1
                was_active = replay.active
                ans = apply(replay, ev)
                if ev[0] == 'cmd':
                    key = ('while inactive' if not was_active else
                           'non-finite' if not all(math.isfinite(c) for c in ev[1]) else
                           'invalid source' if replay.prev_name == INVALID_SOURCE else
                           'stop' if replay.prev_name and replay.prev_zero else
                           'slowdown' if replay.prev_name else 'pass')
                    answers[key] = answers.get(key, 0) + 1
                    if was_active and ans[0] is None and key != 'non-finite':
                        answers['(of these withheld)'] = answers.get('(of these withheld)', 0) + 1
                    if ans[1] is not None:
                        answers['(state messages)'] = answers.get('(state messages)', 0) + 1
            problems = run_case(f's{seed}', cfg, events, exes, opts, opts.settle)
            if problems and not opts.self_test:
                retried += 1
                print(f'seed {seed}: {len(problems)} disagreement(s), e.g. {problems[0]}; '
                      'once more with a settle time of 0.3 s')
                problems = run_case(f's{seed}r', cfg, events, exes, opts, 0.3)
            what = ', '.join(f'{z.kind} {z.action}' for z in cfg['zones'])
            print(f"seed {seed}: {'FAIL' if problems else 'ok  '} {n_cmd} commands; zones: {what}; "
                  f"laser {cfg['laser']}; shift {cfg['shift']}")
            for p in problems:
                print('    ' + p)
            sys.stdout.flush()
            if problems:
                bad.append(seed)
    finally:
        rclpy.shutdown()
    print('events: ' + ', '.join(f'{k} {v}' for k, v in sorted(kinds.items())))
    print('commands by the model\'s answer: '
          + ', '.join(f'{k} {v}' for k, v in sorted(answers.items())))
    print(f'{len(bad)} of {seeds[1] - seeds[0]} cases disagree with the model, {commands} commands, '
          f'{retried} case(s) retried' + (f'; seeds {bad}' if bad else ''))
    if opts.self_test:
        print('self-test: ' + ('a disagreement was reported, as it must be' if bad
                               else 'NO disagreement was reported'))
        sys.exit(0 if bad else 1)
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
