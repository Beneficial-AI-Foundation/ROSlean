#!/usr/bin/env python3
"""Turns a take (recorder.py) into web/data.js, which the two pages load.

Usage: build_data.py TAKE_DIR [MONITOR_YAML]

What the pages show about the monitor is decided here, from the recorded
messages only:

- Each message on the monitor's output is paired with the input message that
  has the same header stamp (the monitor copies the header) and classified by
  comparing the two velocities, as the demo's monitor_narrator.py does:
  equal -> 'pass'; zero for a non-zero command -> 'stop'; the command times
  one ratio -> 'slow'; a zero command passed on -> 'zero'.
- A stretch in which non-zero commands arrive and nothing is published is
  'silent'.
- The zone that is lit is the one the monitor's own state topic names.

The take is refused (an assertion fails) if this record is inconsistent:
an output without an input, an output that is none of the above, a ratio
other than the configured one, an output that contradicts the state topic at
that moment, a silence that is not inside a reported stop or does not follow
a published zero, or a scenario that did not run once from drop to drive-on.
Neither page computes what the monitor "should" do: the zones' shapes are
drawn from monitor.yaml, but whether a point is inside one is never tested.
"""

import json
import math
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import make_world  # noqa: E402

LEAD = 19.5        # s of video before the box is dropped
AFTER = 12.6       # s of the run shown after the box is removed; then the end card
END_CARD = 7.4     # s
FPS = 30
SILENCE = 0.3      # s without an output, with commands arriving, counts as silent
LIDAR_X = -0.032   # m, base_scan in base_footprint (turtlebot3_burger.urdf)
SCAN_RADIUS = 2.6  # m: lidar points further from the robot are not drawn
DO_NOTHING, STOP, SLOWDOWN = 0, 1, 2   # nav2_msgs/CollisionMonitorState.action_type

take = sys.argv[1]
config = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
    HERE, '..', '..', 'ros', 'roslean_demos', 'gazebo', 'monitor', 'monitor.yaml')
T = json.load(open(os.path.join(take, 'trace.json')))
params = yaml.safe_load(open(config))['cm_gate']['ros__parameters']

# ---- The zones, from the monitor's configuration ----
zones = {}
for name in params['polygons']:
    z = params[name]
    assert z['type'] == 'polygon' and z['enabled'], name
    zones[name] = {'points': json.loads(z['points']), 'action': z['action_type'],
                   'ratio': z.get('slowdown_ratio')}
slow_ratios = {n: z['ratio'] for n, z in zones.items() if z['action'] == 'slowdown'}
stop_zones = [n for n, z in zones.items() if z['action'] == 'stop']

# ---- The scenario's events ----
events = {}
for e in T['events']:
    assert e['name'] != 'missed', 'the robot missed the box in this take; record again'
    assert e['name'] not in events, f"event {e['name']} twice"
    events[e['name']] = e
assert list(events) == ['dropped', 'rest', 'removed', 'moving', 'end'], list(events)
assert events['end']['ok'] and events['removed']['still'], 'the scenario did not succeed'
t0 = events['dropped']['t'] - LEAD            # video time = simulated time - t0
box = events['dropped']

# ---- The monitor's output, message by message ----
velocity = {}
for _, sec, nsec, vx, vy, wz in T['in']:
    key = (sec, nsec)
    assert key not in velocity, 'two commands with one stamp'
    velocity[key] = (vx, vy, wz)


def state_at(t):
    """The last message on the state topic at or before t: (action, zone)."""
    last = (DO_NOTHING, '')        # nothing reported yet
    for ts, action, zone in T['state']:
        if ts > t:
            break
        last = (action, zone)
    return last


for _, action, zone in T['state']:
    assert action in (DO_NOTHING, STOP, SLOWDOWN), f'state topic: action {action}'
    assert (zone == '') if action == DO_NOTHING else (zone in zones), f'state topic: zone {zone!r}'
    if action == STOP:
        assert zones[zone]['action'] == 'stop'
    if action == SLOWDOWN:
        assert zones[zone]['action'] == 'slowdown'

out = []           # [t, kind, ratio, vx_in, wz_in, vx_out, wz_out]
for t, sec, nsec, vx, vy, wz in T['out']:
    assert (sec, nsec) in velocity, f'output at {t} s has no input with its stamp'
    cmd, pub = velocity[(sec, nsec)], (vx, vy, wz)
    ratio = None
    if not any(cmd):
        assert not any(pub), f'output at {t} s: non-zero for a zero command'
        kind = 'zero'
    elif pub == cmd:
        kind = 'pass'
    elif not any(pub):
        kind = 'stop'
    else:
        i = max(range(3), key=lambda k: abs(cmd[k]))
        ratio = pub[i] / cmd[i]
        assert all(abs(p - ratio * c) < 1e-9 for p, c in zip(pub, cmd)), \
            f'output at {t} s is not the command times one ratio'
        kind = 'slow'
    # What the state topic said just before or just after this message (the
    # two topics are not ordered against each other on arrival).
    said = {state_at(t - 0.06), state_at(t + 0.06)}
    if kind == 'pass':
        assert any(a == DO_NOTHING for a, _ in said), f'pass at {t} s, state topic says {said}'
    elif kind == 'stop':
        assert any(a == STOP and z in stop_zones for a, z in said), \
            f'stop at {t} s, state topic says {said}'
    elif kind == 'slow':
        named = [z for a, z in said if a == SLOWDOWN]
        assert named, f'slowdown at {t} s, state topic says {said}'
        assert any(abs(ratio - slow_ratios[z]) < 1e-6 for z in named), \
            f'ratio {ratio} at {t} s is not the configured one'
    out.append([t, kind, ratio, cmd[0], cmd[2], pub[0], pub[2]])
out.sort(key=lambda r: r[0])

# ---- Silences: commands arrive, nothing is published ----
silent = []
commands = sorted((t, any(v[2:]), v[2], v[4]) for t, *v in T['in'])
out_times = [r[0] for r in out]
for a, b in zip(out_times, out_times[1:] + [commands[-1][0]]):
    arriving = [t for t, nonzero, _, _ in commands if a + 0.05 < t < b - 0.05 and nonzero]
    if b - a > SILENCE and len(arriving) >= 2:
        last = next(r for r in out if r[0] == a)
        assert last[1] in ('stop', 'zero'), \
            f'silence from {a} s follows a published {last[1]}, not a zero'
        for t in arriving:
            action, zone = state_at(t)
            assert action == STOP, f'silent at {t} s although the state topic does not say stop'
        silent.append([arriving[0], b])
kinds = [r[1] for r in out]
assert 'slow' in kinds and 'stop' in kinds and silent, 'the take has no slowdown, stop or silence'
assert all(abs(r[5]) <= abs(r[3]) + 1e-12 and abs(r[6]) <= abs(r[4]) + 1e-12 for r in out), \
    'an output is larger than its command'


def first(condition, after=-1e9):
    return next(r[0] for r in out if r[0] > after and condition(r))


marks = {
    'drop': box['t'],
    'slow': first(lambda r: r[1] == 'slow', box['t']),
    'stop': first(lambda r: r[1] == 'stop', box['t']),
    'silent': silent[0][0],
    'removed': events['removed']['t'],
    'pass': first(lambda r: r[1] == 'pass', events['removed']['t']),
    'end': events['end']['t'],
}
assert marks['drop'] < marks['slow'] < marks['stop'] < marks['silent'] < marks['removed'] \
    < marks['pass'], marks
end_start = marks['removed'] - t0 + AFTER
assert marks['end'] - t0 >= end_start, 'the take ends before the video does'

# ---- The lidar: points in the base frame, and the nearest one ahead ----
S = T['scan']
half = max(abs(p[1]) for z in zones.values() for p in z['points'])   # widest zone, half width
scans, nearest = [], []
for ts, ranges in zip(S['t'], S['ranges']):
    pts, ahead = [], None
    for i, mm in enumerate(ranges):
        if not mm:
            continue
        r, a = mm / 1000.0, S['angle_min'] + i * S['angle_increment']
        x, y = LIDAR_X + r * math.cos(a), r * math.sin(a)
        if math.hypot(x, y) <= SCAN_RADIUS:
            pts += [round(x, 3), round(y, 3)]
        if x > 0 and abs(y) <= half and (ahead is None or x < ahead):
            ahead = x
    scans.append(pts)
    nearest.append(None if ahead is None else round(ahead, 3))

V = lambda t: round(t - t0, 4)   # noqa: E731
data = {
    't0': t0,
    'endStart': round(end_start, 3), 'length': round(end_start + END_CARD, 3),
    'frameCount': round((end_start + END_CARD) * FPS),
    'marks': {k: V(t) for k, t in marks.items()},
    'zones': zones,
    'box': {'x': box['box'][0], 'y': box['box'][1], 'yaw': box['box'][2], 'size': box['size'],
            'from': V(box['t']), 'to': V(events['removed']['t'])},
    'restGap': events['rest']['gap'],
    'pose': [[V(t), x, y, yaw] for t, x, y, yaw in T['pose'][::3]],
    'inT': [V(c[0]) for c in commands], 'inV': [[round(c[2], 4), round(c[3], 4)] for c in commands],
    'outT': [V(r[0]) for r in out], 'outKind': [r[1] for r in out],
    'outRatio': [None if r[2] is None else round(r[2], 4) for r in out],
    'outCmd': [[round(r[3], 4), round(r[4], 4)] for r in out],
    'outPub': [[round(r[5], 4), round(r[6], 4)] for r in out],
    'stateT': [V(t) for t, _, _ in T['state']],
    'stateAction': [a for _, a, _ in T['state']], 'stateZone': [z for _, _, z in T['state']],
    'scanT': [V(t) for t in S['t']], 'scan': scans, 'nearest': nearest,
    'lidarX': LIDAR_X,
}
frames = os.path.join(take, 'frames.json')
if os.path.exists(frames):
    F = json.load(open(frames))
    (x, y, z), (lx, ly, lz) = make_world.CAM_POS, make_world.CAM_LOOK
    data.update(frames=[V(t) for t in F['frames']], size=F['size'], camera={
        'pos': [x, y, z], 'hfov': make_world.CAM_HFOV,
        'yaw': math.atan2(ly - y, lx - x),
        'pitch': math.atan2(z - lz, math.hypot(lx - x, ly - y)),
    })
path = os.path.join(HERE, 'web', 'data.js')
with open(path, 'w') as f:
    f.write('window.DATA = ' + json.dumps(data, separators=(',', ':')) + ';\n')

changes, prev = [], None
for r in out:
    label = r[1] if r[2] is None else f'slow x{r[2]:.2f}'
    if label != prev:
        changes.append(f'{V(r[0]):.2f} {label}')
        prev = label
print(path)
print(f"  {len(out)} outputs, all paired and consistent with the state topic; t0 = {t0:.3f} s")
print('  output changes (video time):', ', '.join(changes))
print('  silent:', ', '.join(f'{V(a):.2f}-{V(b):.2f}' for a, b in silent))
print('  state topic:', ', '.join(f'{V(t):.2f} {a} {z!r}' for t, a, z in T['state']))
print('  marks:', data['marks'])
print(f"  video: {data['length']} s, {data['frameCount']} frames")
