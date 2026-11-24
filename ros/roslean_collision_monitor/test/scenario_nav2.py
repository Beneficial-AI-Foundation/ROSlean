#!/usr/bin/env python3
"""Fixed scenarios for the Nav2 extension: lifecycle, state topic, polygon
topics, transforms from TF, base shift correction, zones fed by a topic, and
the edit of Nav2's footprint zone.

    scenario_nav2.py [NODE] [options]

NODE is a path to an executable or `<package>/<executable>`; the default is
upstream's `nav2_collision_monitor/collision_monitor`. Every expectation is
what that binary was observed to do (Nav2 1.3.12, ROS 2 Jazzy, RoboStack,
macOS arm64, 2026-11-24); docs/collision_monitor/UPSTREAM_BEHAVIOUR.md,
"The Nav2 extension", refers to the scenarios here by name, and
docs/collision_monitor/SPEC_NAV2.md, section 10, lists what they cover.

Options (as scenario.py):
  --only A,B,...     run only these scenarios (a family name selects a family)
  --skip A,B,...     leave these out
  --repeat N         run everything N times
  --settle S         wall seconds allowed for delivery (default 0.2)
  --logs DIR         keep the node's output, one file per scenario
  --list             list the scenarios with their families and exit
  --profile MODULE   the node is not upstream's: MODULE (next to this script,
                     e.g. profile_roslean_nav2) states every answer that is
                     expected to differ from upstream's and why. Everything
                     not stated must be as upstream.
  --first-failure    stop after the first scenario with a failure

The node must be a lifecycle node: the scenarios drive its transitions. It
runs with `use_sim_time`; this script publishes `/clock`, scans, commands,
`/tf` (with chosen stamps), `/tf_static` and zone shapes, and records the
velocity output, the state topic and the polygon topics a scenario watches.

A command whose transform lookup fails is answered only once the node clock
has advanced by `transform_tolerance` (the node waits inside the callback):
such a command expects nothing, and the answer is claimed by the following
`late` (the clock moves) or `arrives` (the missing data comes) step.
"""

import argparse
import math
import os
import sys
import tempfile

import rclpy

from cm_harness import (
    DO_NOTHING, HERE, SLOWDOWN, STOP, UPSTREAM, Driver, Side, f32, f32_next, resolve, s, xf)
from scenario import INVALID, NORMAL, P, check_answer, clock, cmd, describe, late, quiet, ray, scan

PI = math.pi
NAN = math.nan
HALF_PI = f32(PI / 2)
SQ = math.sqrt(0.5)
T = s(100)
V = (0.5, 0.25, 1.0)
HALF = (0.25, 0.125, 0.5)
ZERO = (0.0, 0.0, 0.0)
BASE = 'base_footprint'
TRANSITIONS = {'configure': 1, 'cleanup': 2, 'activate': 3, 'deactivate': 4,
               'shutdown-unconfigured': 5, 'shutdown-inactive': 6, 'shutdown-active': 7}


# ---- steps beyond those of scenario.py ---------------------------------------

def tf(*transforms):
    """Transforms on /tf with the stamps they carry."""
    return ('tf', transforms)


def static(*transforms):
    """Static transforms sent while the node runs."""
    return ('static', transforms)


def shape(points, frame=BASE, topic='zone_shape'):
    """A PolygonStamped on a zone's polygon_sub_topic."""
    return ('shape', topic, points, frame)


def transition(name, label='', ok=True):
    """A lifecycle transition. It must succeed (or fail, ok=False) and publish
    no velocity and no state message."""
    return ('transition', name, ok, label)


def arrives(inner, out, label, state=None):
    """`inner` (a tf, static or shape step) is sent while the previous command
    waits for it; the command is answered now, without the clock moving."""
    return ('arrives', inner, out, state, label)


def polys(step, *want):
    """The step makes the node publish exactly these polygons on the watched
    topics: (topic, frame, points), stamped with the node clock. In a scenario
    that watches topics, a step without this wrapper must publish none."""
    return ('polys', step, want)


def graph(label, kind, topic, count):
    """The node has `count` publishers ('pub') or subscriptions ('sub') on a topic."""
    return ('graph', label, kind, topic, count)


def qos(label, kind, topic, reliability, durability):
    return ('qos', label, kind, topic, reliability, durability)


def lcstate(want, label=''):
    return ('lcstate', want, label)


def log(label, *needles):
    """The node's output contains every needle."""
    return ('log', label, needles)


def odom(stamp, x=0.0, y=0.0, q=(0.0, 0.0, 0.0, 1.0)):
    """Where the base is in the odom frame at `stamp`."""
    return xf('odom', BASE, x=x, y=y, q=q, stamp=stamp)


def circle16(radius):
    return ('circle', radius)


# ---- lifecycle (SPEC_NAV2 N2-N8; test plan rows 1 to 6) -----------------------

SC = ('StopCircle', STOP)
FB = ('FrontBox', STOP)
BOX = [(0.5, 0.25), (0.5, -0.25), (0.0, -0.25), (0.0, 0.25)]
VIS = ['StopCircle.visualize:=true', 'FrontBox.visualize:=true']
BOTH_POLYS = (('front_box', BASE, BOX), ('StopCircle', BASE, circle16(0.5)))
NEAR = ray(0.25)
FAR = ray(2.0)

LIFECYCLE_STATES = [
    graph('unconfigured: no command subscription', 'sub', 'cmd_vel_in', 0),
    graph('unconfigured: no scan subscription', 'sub', 'scan', 0),
    graph('unconfigured: no TF subscription', 'sub', '/tf', 0),
    graph('unconfigured: no velocity publisher', 'pub', 'cmd_vel_out', 0),
    graph('unconfigured: no state publisher', 'pub', 'cm_state', 0),
    clock(T), scan(NEAR),
    cmd(V, None, 'unconfigured: a command reaches nobody'),
    transition('configure'),
    lcstate('inactive'),
    graph('inactive: command subscription', 'sub', 'cmd_vel_in', 1),
    graph('inactive: scan subscription', 'sub', 'scan', 1),
    graph('inactive: /tf subscription', 'sub', '/tf', 1),
    graph('inactive: /tf_static subscription', 'sub', '/tf_static', 1),
    graph('inactive: velocity publisher', 'pub', 'cmd_vel_out', 1),
    graph('inactive: state publisher', 'pub', 'cm_state', 1),
    graph('inactive: polygon publisher of a visualised polygon', 'pub', 'front_box', 1),
    graph('inactive: polygon publisher of a visualised circle', 'pub', 'StopCircle', 1),
    cmd(V, None, 'inactive, no scan since configure: nothing (not even a stop)'),
    scan(NEAR),
    cmd(V, None, 'inactive, obstacle scan stored: nothing'),
    cmd((NAN, 0.0, 0.0), None, 'inactive, a NaN command: nothing'),
    polys(transition('activate', 'no velocity, no state; each visualised zone once'), *BOTH_POLYS),
    lcstate('active'),
    polys(cmd(V, ZERO, 'active: the scan received while inactive is used', state=SC), *BOTH_POLYS),
    polys(cmd(V, ZERO, 'again: zero, no state'), *BOTH_POLYS),
    cmd((NAN, 0.0, 0.0), None, 'active, a NaN command: nothing, no polygons either'),
    transition('deactivate', 'publishes nothing'),
    lcstate('inactive'),
    cmd(V, None, 'inactive again: nothing'),
]

# stop_pub_timeout 1.0.
LIFECYCLE_REACTIVATE = [
    clock(T), scan(NEAR),
    cmd(V, ZERO, 'stop', state=SC),
    clock(T + s(1)),
    cmd(V, ZERO, 'exactly stop_pub_timeout into the stop: still published'),
    clock(T + s(1) + 1),
    cmd(V, None, '1 ns later: withheld'),
    transition('deactivate'),
    cmd(V, None, 'inactive: nothing'),
    transition('activate'),
    cmd(V, ZERO, 'active again, same clock, same scan: zero AND the state are published anew',
        state=SC),
    clock(T + s(2) + 1),
    cmd(V, ZERO, 'stop_pub_timeout after the zero that followed re-activation: still published'),
    clock(T + s(2) + 2),
    cmd(V, None, '1 ns later: withheld'),
    scan(FAR),
    cmd(V, V, 'obstacle gone: passes', state=NORMAL),
    transition('deactivate'), transition('activate'),
    cmd(V, V, 're-activated while passing: passes, no state message (the name is "" as at start)'),
]

LIFECYCLE_CLEANUP = [
    clock(T), scan(FAR),
    cmd(V, V, 'a command passes'),
    transition('deactivate'), transition('cleanup'),
    lcstate('unconfigured'),
    graph('after cleanup: no command subscription', 'sub', 'cmd_vel_in', 0),
    graph('after cleanup: no scan subscription', 'sub', 'scan', 0),
    graph('after cleanup: no TF subscription', 'sub', '/tf', 0),
    graph('after cleanup: no velocity publisher', 'pub', 'cmd_vel_out', 0),
    graph('after cleanup: no state publisher', 'pub', 'cm_state', 0),
    cmd(V, None, 'unconfigured again: nothing'),
    transition('configure'), transition('activate'),
    cmd(V, ZERO, 'after cleanup, configure, activate: the stored scan is gone', state=INVALID),
    scan(FAR),
    cmd(V, V, 'a new scan: passes', state=NORMAL),
]

LIFECYCLE_SHUTDOWN_ACTIVE = [
    clock(T), scan(FAR),
    cmd(V, V, 'a command passes'),
    transition('shutdown-active'),
    lcstate('finalized'),
    cmd(V, V, 'finalized after a shutdown from active: the command is STILL FORWARDED'),
    scan(NEAR),
    cmd(V, ZERO, 'finalized: a stop is still decided and published', state=SC),
]

LIFECYCLE_SHUTDOWN_INACTIVE = [
    clock(T),
    transition('configure'),
    scan(FAR),
    transition('shutdown-inactive'),
    lcstate('finalized'),
    cmd(V, None, 'finalized after a shutdown from inactive: nothing'),
]

LIFECYCLE_SHUTDOWN_DEACTIVATED = [
    clock(T), scan(FAR),
    cmd(V, V, 'a command passes'),
    transition('deactivate'),
    transition('shutdown-inactive'),
    lcstate('finalized'),
    cmd(V, None, 'deactivated, then shut down: nothing'),
]

# ---- state topic (N12-N15; row 7) ---------------------------------------------

SLOW = ('SlowCircle', SLOWDOWN)
MID = ray(0.75)

# stop_pub_timeout 1.0, source_timeout 5.0.
STATE_TOPIC = [
    qos('the state publisher is reliable and volatile', 'pub', 'cm_state', 'RELIABLE', 'VOLATILE'),
    clock(T), scan(FAR),
    cmd(V, V, 'first command, clear: passes, no state message'),
    scan(MID),
    cmd(V, HALF, 'point in the slowdown circle', state=SLOW),
    cmd(V, HALF, 'the same: no state message'),
    scan(NEAR),
    cmd(V, ZERO, 'point in both circles: stop', state=SC),
    clock(T + s(2)),
    cmd(V, None, '2 s later, still stop: velocity withheld, no state message'),
    scan(MID),
    cmd(V, HALF, 'slowdown: the velocity returns', state=SLOW),
    scan(NEAR),
    cmd(V, ZERO, 'stop again', state=SC),
    clock(T + s(4)),
    cmd(V, None, 'withheld again'),
    scan(MID),
    cmd(ZERO, None, 'zero command in the slowdown zone while withheld: state ("", 0), no velocity',
        state=NORMAL),
    scan(FAR),
    cmd(ZERO, None, 'zero command, clear, while withheld: no state message'),
    clock(T + s(20)),
    cmd(ZERO, None, 'scan now 16 s old: "invalid source" is reported while withheld',
        state=INVALID),
    cmd(V, None, 'a non-zero command, source still invalid: withheld, no state message'),
    cmd((NAN, 0.0, 0.0), None, 'a NaN command: nothing'),
]

# A zone that is called "invalid source" (N15): names are compared.
STATE_NAME_COLLISION = [
    clock(T), scan(NEAR),
    cmd(V, ZERO, 'stop by a zone named "invalid source"', state=('invalid source', STOP)),
    clock(T + s(6)),
    cmd(V, ZERO, 'the scan is now too old: the change to an invalid source is NOT reported'),
    scan(FAR),
    cmd(V, V, 'clear', state=NORMAL),
]

# ---- polygon topics (N16; row 8) ----------------------------------------------

# stop_pub_timeout 1.0.
POLYGON_PUB = [
    clock(T),
    transition('configure', 'publishes no polygon'),
    polys(transition('activate', 'each visualised zone once'), *BOTH_POLYS),
    qos('polygon publisher (polygon): reliable, transient local', 'pub', 'front_box',
        'RELIABLE', 'TRANSIENT_LOCAL'),
    qos('polygon publisher (circle): reliable, transient local', 'pub', 'StopCircle',
        'RELIABLE', 'TRANSIENT_LOCAL'),
    clock(T + s(1)),
    scan({'ranges': [0.25, 0.25, 2.0], 'angle_increment': 0.01}),
    polys(cmd(V, ZERO, 'a processed command: both polygons, stamped with the clock', state=SC),
          *BOTH_POLYS),
    cmd((NAN, 0.0, 0.0), None, 'a NaN command: no polygon'),
    clock(T + s(5)),
    polys(cmd(V, None, 'velocity withheld: the polygons are published all the same'), *BOTH_POLYS),
    transition('deactivate', 'publishes no polygon'),
    cmd(V, None, 'inactive: no polygon'),
]

# ---- transforms without base shift correction (N23, N25-N27; rows 11 to 14) ---

LASER = dict(frame='laser')

TF_LATEST = [
    clock(T),
    tf(xf(BASE, 'laser', x=1.0, stamp=T)),
    scan(ray(0.25, **LASER)),
    cmd(V, V, 'sensor 1 m ahead of the base at the scan stamp: point at 1.25 m, passes'),
    tf(xf(BASE, 'laser', x=0.0, stamp=T + s(0.5))),
    cmd(V, ZERO, 'a transform stamped 0.5 s AFTER the scan and the clock puts the sensor at the '
        'base: point at 0.25 m, stop (same scan)', state=SC),
    clock(T + s(4)),
    cmd(V, ZERO, 'the newest transform is 3.5 s old: still used'),
    tf(xf(BASE, 'laser', x=1.0, stamp=T + s(0.25))),
    cmd(V, ZERO, 'a transform with an older stamp (sensor 1 m ahead) arrives late: changes nothing'),
    tf(xf(BASE, 'laser', x=1.0, stamp=T + s(0.75))),
    cmd(V, V, 'a transform with a newer stamp: used at once', state=NORMAL),
]

RB = ('RightBox', STOP)
TILT_TF = [
    xf(BASE, 'laser_up'),
    xf(BASE, 'laser_high', z=5.0),
    xf(BASE, 'laser_flipped', q=(1.0, 0.0, 0.0, 0.0)),
    xf(BASE, 'laser_pitched', q=(0.0, SQ, 0.0, SQ)),
    xf(BASE, 'laser_double', q=(0.0, 0.0, 0.0, 2.0)),
    xf(BASE, 'laser_yaw90', q=(0.0, 0.0, SQ, SQ)),
]
# RightBox: x in [-0.25, 0.25], y in [-0.5, -0.125].
TF_TILT = [
    clock(T),
    scan(ray(0.25, HALF_PI, frame='laser_up')),
    cmd(V, V, 'upright sensor, ray to the left; the zone is on the right: passes'),
    scan(ray(0.25, HALF_PI, frame='laser_flipped')),
    cmd(V, ZERO, 'sensor upside down (roll of half a turn): the ray lands on the right', state=RB),
    scan(ray(0.25, HALF_PI, frame='laser_high')),
    cmd(V, V, 'sensor 5 m above the base, ray to the left: passes', state=NORMAL),
    scan(ray(0.25, -HALF_PI, frame='laser_high')),
    cmd(V, ZERO, 'sensor 5 m above the base, ray to the right: the height is ignored', state=RB),
    scan(ray(0.25, -HALF_PI, frame='laser_double')),
    cmd(V, ZERO, 'quaternion (0, 0, 0, 2), not normalised: acts as the identity'),
    scan(ray(0.25, 0.0, frame='laser_yaw90')),
    cmd(V, V, 'sensor turned a quarter turn to the left, ray ahead: lands on the left',
        state=NORMAL),
    scan(ray(0.25, PI, frame='laser_yaw90')),
    cmd(V, ZERO, 'the same sensor, ray backwards: lands on the right', state=RB),
    scan(dict(ranges=[3.0, 3.0], angle_min=-HALF_PI, angle_increment=0.0, frame='laser_pitched')),
    cmd(V, V, 'sensor pitched a quarter turn, rays to the right at 3 m: y is unchanged, outside',
        state=NORMAL),
    scan(ray(0.25, -HALF_PI, frame='laser_pitched')),
    cmd(V, ZERO, 'the same sensor, ray to the right at 0.25 m: inside', state=RB),
]

TF_PITCH_CIRCLE = [
    clock(T),
    scan(ray(3.0, frame='laser_up')),
    cmd(V, V, 'upright sensor, point 3 m ahead; stop circle of 0.5 m: passes'),
    scan(ray(3.0, frame='laser_pitched')),
    cmd(V, ZERO, 'sensor pitched a quarter turn: the point 3 m along its x axis is projected '
        'onto the base: stop', state=SC),
]


def tf_missing(tolerance_ns):
    half = tolerance_ns // 2
    return [
        clock(T), scan(ray(0.25, **LASER)),
        cmd(V, None, 'scan in a frame without transform: no answer while the clock stands still'),
        quiet('still none after a second settle time'),
        clock(T + half), quiet('half the tolerance later: none'),
        clock(T + tolerance_ns - 1), quiet('transform_tolerance - 1 ns: none'),
        late(T + tolerance_ns, ZERO, 'exactly transform_tolerance after the command: the stop',
             state=INVALID),
        cmd(V, None, 'the next command waits again'),
        clock(T + 2 * tolerance_ns - 1), quiet('its tolerance - 1 ns: none'),
        arrives(static(xf(BASE, 'laser')), ZERO,
                'the transform arrives during the wait: answered without the clock moving, and '
                'the point is in the circle', state=SC),
        cmd(V, ZERO, 'with the transform present: answered at once'),
        scan(ray(0.25, frame='')),
        cmd(V, None, 'a scan with an empty frame_id: waits like an unknown frame'),
        late(T + 3 * tolerance_ns, ZERO, 'after the tolerance: invalid source', state=INVALID),
    ]


TF_MISSING_TIMEOUT0 = [
    clock(T), scan(ray(0.25, **LASER)),
    cmd(V, V, 'no transform, source_timeout 0, tolerance 0: PASSES at once'),
    static(xf(BASE, 'laser')),
    cmd(V, ZERO, 'transform present: the point is in the circle, stop', state=SC),
]

TF_TOLERANCE0 = [
    clock(T), scan(ray(0.25, **LASER)),
    cmd(V, ZERO, 'no transform, tolerance 0: the stop comes at once', state=INVALID),
    static(xf(BASE, 'laser', x=1.0)),
    cmd(V, V, 'transform present, point at 1.25 m: passes', state=NORMAL),
]

TF_WAIT_TIMEOUT0 = [
    clock(T), scan(ray(0.25, **LASER)),
    cmd(V, None, 'no transform, source_timeout 0, tolerance 0.5: the command still waits'),
    late(T + s(0.5), V, 'after the tolerance: it PASSES'),
]

# A backward jump of the clock (N28): tf2 clears its buffer, which forgets
# the transforms received on /tf but not the static ones.
LASER_DYN = dict(frame='laser_dyn')
TF_CLOCK_JUMP_BACK = [
    clock(T), tf(xf(BASE, 'laser_dyn', stamp=T)),
    scan(ray(0.25, **LASER_DYN)),
    cmd(V, ZERO, 'scan in a frame known from /tf, point in the circle: stop', state=SC),
    clock(T - s(1)), scan(ray(0.25, **LASER)),
    cmd(V, ZERO, 'the clock has jumped back by 1 s; scan in a frame known from /tf_static: the '
        'static transform is still there, stop'),
    scan(ray(0.25, **LASER_DYN)),
    cmd(V, None, 'scan in the frame known from /tf: that transform is FORGOTTEN, the command '
        'waits'),
    late(T - s(0.5), ZERO, 'after the tolerance: invalid source', state=INVALID),
    cmd(V, None, 'and so does the next command'),
    arrives(tf(xf(BASE, 'laser_dyn', stamp=T - s(1))), ZERO,
            'until the transform is published again', state=SC),
]

# The clock jumps back while a command waits for a transform (N26, N28;
# review CM5, finding 2). tf2's wait ends when the clock is more than 3 s
# before the start of the wait ("bag loop"); a smaller jump leaves the wait
# running until the clock is at start + tolerance again. In both cases the
# buffer is cleared at the jump, during the wait: a transform that arrives
# after the jump is used by the waiting command and is still there for the
# next one.
TF_WAIT_CLOCK_RESET = [
    clock(T), scan(ray(0.25, **LASER)),
    cmd(V, None, 'scan in a frame without transform: the command waits'),
    late(s(1), ZERO, 'the clock jumps back to 1 s during the wait: the wait ends AT ONCE, with the '
         'stop', state=INVALID),
    cmd(V, None, 'the next command waits again, from 1 s'),
    clock(s(1.5) - 1), quiet('its tolerance - 1 ns: none'),
    late(s(1.5), ZERO, 'after its tolerance: the stop'),
    scan(ray(0.25, **LASER_DYN)),
    cmd(V, None, 'scan in another unknown frame: waits'),
    arrives(tf(xf(BASE, 'laser_dyn', stamp=s(1.5))), ZERO,
            'the transform arrives: answered, the point is in the circle', state=SC),
    cmd(V, ZERO, 'and the node goes on: answered at once'),
]

TF_WAIT_CLOCK_BACK = [
    clock(T), scan(ray(0.25, **LASER_DYN)),
    cmd(V, None, 'scan in a frame without transform: the command waits'),
    clock(T - s(1)), quiet('the clock jumps back by 1 s, less than 3 s: the wait goes on'),
    clock(T - s(2.5)), quiet('2.5 s before the start: still waiting'),
    clock(T + s(0.5) - 1), quiet('start + tolerance - 1 ns, start being the clock before the '
                                 'jump: none'),
    late(T + s(0.5), ZERO, 'start + tolerance: the stop', state=INVALID),
    cmd(V, None, 'the next command waits'),
    clock(T - s(2)), quiet('the clock jumps back by 2.5 s: the wait goes on'),
    arrives(tf(xf(BASE, 'laser_dyn', stamp=T - s(2))), ZERO,
            'the transform arrives after the jump: answered', state=SC),
    cmd(V, ZERO, 'the transform that arrived after the jump is STILL THERE for the next command: '
        'the buffer was cleared at the jump, not after the waiting command'),
    clock(T - s(3)),
    cmd(V, None, 'another jump back, outside any wait: the transform is forgotten, the command '
        'waits'),
    late(T - s(2.5), ZERO, 'after the tolerance: invalid source', state=INVALID),
]

# The same in the ordinary situation with base shift correction: a command
# newer than the newest odometry sample waits for the next one, and the
# simulation is reset meanwhile.
SHIFT_WAIT_RESET = [
    clock(T), tf(odom(T - s(1)), odom(T)), scan(NEAR),
    cmd(V, ZERO, 'odometry up to the clock, point in the circle: stop', state=SC),
    clock(T + s(0.05)),
    cmd(V, None, 'the clock is 50 ms past the newest odometry sample: the command waits'),
    late(s(0.01), ZERO, 'the simulation is reset during the wait (clock 0.01 s): the wait ends at '
         'once, invalid source', state=INVALID),
    clock(s(1)), tf(odom(s(0.5)), odom(s(1))), scan(NEAR),
    cmd(V, ZERO, 'clock 1 s, fresh odometry and scan: the node answers, stop by the circle',
        state=SC),
    clock(s(2)), tf(odom(s(2))), scan(FAR),
    cmd(V, V, 'clock 2 s, fresh odometry, a far scan: passes', state=NORMAL),
]

# Transforms tf2 does not take (N25): a zero quaternion and a NaN. The frame
# stays unknown. (If one got through, the ROSlean node would count it as a
# failed lookup, ND7: the same answers.)
TF_BAD_QUATERNION = [
    clock(T), scan(ray(0.25, **LASER)),
    static(xf(BASE, 'laser', q=(0.0, 0.0, 0.0, 0.0))),
    cmd(V, None, 'the transform was published with the quaternion (0, 0, 0, 0): the frame is '
        'still unknown, the command waits'),
    late(T + s(0.5), ZERO, 'after the tolerance: invalid source', state=INVALID),
    static(xf(BASE, 'laser', q=(0.0, 0.0, NAN, 1.0))),
    cmd(V, None, 'published with a NaN in the quaternion: still unknown'),
    late(T + s(1), ZERO, 'after the tolerance: invalid source'),
    static(xf(BASE, 'laser', x=NAN)),
    cmd(V, None, 'published with a NaN translation: still unknown'),
    late(T + s(1.5), ZERO, 'after the tolerance: invalid source'),
    static(xf(BASE, 'laser')),
    cmd(V, ZERO, 'published properly: the point is in the circle', state=SC),
]

# ---- base shift correction (N24; rows 15 to 19) --------------------------------

SHIFT_STRAIGHT = [
    clock(T),
    cmd(V, ZERO, 'no scan yet: stop at once (no lookup is made)', state=INVALID),
    tf(odom(T)),
    scan(ray(1.25)),
    cmd(V, V, 'scan stamped now, point 1.25 m ahead, one odom sample at this instant: passes',
        state=NORMAL),
    tf(odom(T + s(1), x=1.0)),
    clock(T + s(0.5)),
    cmd(V, V, 'half way between the odom samples: the base is at 0.5 m, the point 0.75 m ahead'),
    clock(T + s(1)),
    cmd(V, ZERO, 'the base has moved 1 m since the scan: the point is 0.25 m ahead, stop',
        state=SC),
    clock(T + s(1.25)),
    cmd(V, None, 'a command 0.25 s after the newest odom sample: waits'),
    clock(T + s(1.75) - 1), quiet('transform_tolerance - 1 ns: none'),
    late(T + s(1.75), ZERO, 'after the tolerance: invalid source', state=INVALID),
    cmd(V, None, 'the next command waits again'),
    arrives(tf(odom(T + s(2), x=1.0)), ZERO,
            'an odom sample newer than the command arrives: answered without the clock moving',
            state=SC),
    cmd(V, ZERO, 'odom samples on both sides of the clock: answered at once'),
]

# The interpolation between two odom samples, at a zone boundary. Samples at
# 0 s (x = 0) and 1 s (x = 1), a scan stamped 0 s, commands in between: the
# base is at 0.5 m and at 0.25 m exactly, and a point at range 1.0 or 0.75
# lands exactly on the 0.5 m circle.
SHIFT_INTERPOLATION = [
    clock(T), tf(odom(T), odom(T + s(1), x=1.0)),
    scan(ray(0.75)),
    clock(T + s(0.25)),
    cmd(V, V, 'a quarter of the way between the samples, point at range 0.75: on the circle, '
        'outside'),
    scan(ray(f32_next(0.75, -1), stamp=T)),
    cmd(V, ZERO, 'a quarter of the way: one float32 step closer, inside', state=SC),
    scan(ray(1.0, stamp=T)),
    clock(T + s(0.5)),
    cmd(V, V, 'half way, point at range 1.0: on the circle, outside', state=NORMAL),
    scan(ray(f32_next(1.0, -1), stamp=T)),
    cmd(V, ZERO, 'half way: one float32 step closer, inside', state=SC),
    scan(ray(1.25, stamp=T)),
    clock(T + s(0.75)),
    cmd(V, V, 'three quarters of the way, point at range 1.25: on the circle, outside',
        state=NORMAL),
    scan(ray(f32_next(1.25, -1), stamp=T)),
    cmd(V, ZERO, 'three quarters of the way: one float32 step closer, inside', state=SC),
]

SHIFT_OFF = [
    clock(T),
    tf(odom(T)),
    scan(ray(1.25)),
    cmd(V, V, 'correction off: passes'),
    tf(odom(T + s(1), x=1.0)),
    clock(T + s(1)),
    cmd(V, V, 'correction off, the base has moved 1 m since the scan: the scan stays where it '
        'was in the base frame, passes'),
    clock(T + s(1.25)),
    cmd(V, V, 'correction off, command newer than every odom sample: no wait, passes'),
]

SHIFT_TURN = [
    clock(T), tf(odom(T)),
    scan(ray(0.25)),
    cmd(V, ZERO, 'point 0.25 m ahead, in the front box', state=FB),
    tf(odom(T + s(1), q=(0.0, 0.0, 1.0, 0.0))),
    clock(T + s(1)),
    cmd(V, V, 'the base has turned half a turn since the scan: the point is behind it',
        state=NORMAL),
    scan(ray(0.25, PI, stamp=T)),
    cmd(V, ZERO, 'a scan stamped before the turn with the point behind: now ahead', state=FB),
    scan(ray(0.25, PI, stamp=T + s(1))),
    cmd(V, V, 'the same scan stamped after the turn: behind', state=NORMAL),
]

# A quarter turn to the left: the point that was ahead is on the right.
SHIFT_QUARTER = [
    clock(T), tf(odom(T)),
    scan(ray(0.25)),
    cmd(V, V, 'point 0.25 m ahead; the zone is on the right: passes'),
    tf(odom(T + s(1), q=(0.0, 0.0, SQ, SQ))),
    clock(T + s(1)),
    cmd(V, ZERO, 'the base has turned a quarter turn to the left since the scan: the point is '
        'on its right', state=RB),
    scan(ray(0.25, stamp=T + s(1))),
    cmd(V, V, 'the same scan stamped after the turn: ahead', state=NORMAL),
]

SHIFT_SENSOR_FRAME = [
    clock(T),
    tf(odom(T), odom(T + s(1), x=1.0)),
    scan(ray(0.75, **LASER)),
    clock(T + s(1)),
    cmd(V, ZERO, 'sensor 0.5 m ahead of the base, point 0.75 m ahead of it at the scan stamp, '
        'base moved 1 m: 0.25 m ahead, stop', state=SC),
    scan(ray(0.75, stamp=T - s(1), **LASER)),
    cmd(V, None, 'a scan stamped 1 s before the oldest odom sample: waits'),
    late(T + s(1.5), ZERO, 'after the tolerance: invalid source', state=INVALID),
    tf(odom(T + s(3), x=1.0)),
    scan(ray(0.75, stamp=T + s(2), **LASER)),
    clock(T + s(2)),
    cmd(V, V, 'scan and command at 2 s, base at rest at 1 m: the point is 1.25 m ahead',
        state=NORMAL),
]

SHIFT_NO_ODOM = [
    clock(T), scan(FAR),
    cmd(V, None, 'scan in the base frame, nothing near, no odom frame at all: waits'),
    late(T + s(0.5), ZERO, 'after the tolerance: invalid source', state=INVALID),
    tf(odom(T), odom(T + s(1))),
    cmd(V, V, 'odom samples around the clock: passes', state=NORMAL),
]

# ---- zones fed by polygon_sub_topic (N29-N35; rows 22 to 25) -------------------

DYN = ('Dyn', STOP)
TRI = [(-0.125, 0.25), (-0.125, -0.25), (-0.5, 0.0)]
NANBOX = [(NAN, 0.25)] + BOX[1:]
CLOSED = BOX + BOX[:1]
DEGENERATE = [(0.5, 0.0)] * 3
BEHIND = ray(0.25, PI)


def dyn(points, frame=BASE):
    return ('Dyn', frame, points)


POLYGON_TOPIC = [
    clock(T),
    transition('configure'),
    qos('the shape subscription is best effort and volatile', 'sub', 'zone_shape',
        'BEST_EFFORT', 'VOLATILE'),
    polys(transition('activate', 'an empty polygon before the first shape'), dyn([])),
    scan(NEAR),
    polys(cmd(V, V, 'no shape received yet, point 0.25 m ahead: PASSES'), dyn([])),
    shape(BOX),
    polys(cmd(V, ZERO, 'front box received: stop', state=DYN), dyn(BOX)),
    shape(BOX[:2]),
    polys(cmd(V, ZERO, 'a message with two points is ignored: the box stays'), dyn(BOX)),
    shape([]),
    polys(cmd(V, ZERO, 'a message with no points is ignored: the box stays'), dyn(BOX)),
    shape(TRI),
    polys(cmd(V, V, 'a triangle behind the base is accepted: the point ahead is outside',
              state=NORMAL), dyn(TRI)),
    scan(BEHIND),
    polys(cmd(V, ZERO, 'point 0.25 m behind: inside the triangle', state=DYN), dyn(TRI)),
    shape(BOX, frame='nowhere'),
    clock(T + s(1)),
    polys(cmd(V, ZERO, 'a box in a frame without transform is ignored (after the tolerance): '
              'the triangle is still the shape'), dyn(TRI)),
    static(xf(BASE, 'shifted', x=-0.5)),
    shape(BOX, frame='shifted'),
    polys(cmd(V, ZERO, 'the box given in a frame 0.5 m behind the base lies behind it: the point '
              'behind is inside'), dyn(BOX, 'shifted')),
    scan(NEAR),
    polys(cmd(V, V, 'point 0.25 m ahead: outside the shifted box', state=NORMAL),
          dyn(BOX, 'shifted')),
    shape(BOX, frame=''),
    clock(T + s(2)),
    polys(cmd(V, V, 'a box with an empty frame_id is ignored (after the tolerance)'),
          dyn(BOX, 'shifted')),
    shape(BOX),
    polys(cmd(V, ZERO, 'the box in the base frame replaces the shape: stop', state=DYN), dyn(BOX)),
    shape(NANBOX),
    polys(cmd(V, V, 'a box with a NaN coordinate is ACCEPTED: the point inside the box passes',
              state=NORMAL), dyn(NANBOX)),
    shape(BOX),
    polys(cmd(V, ZERO, 'the box once more: stop', state=DYN), dyn(BOX)),
    shape(CLOSED),
    polys(cmd(V, ZERO, 'a closed box (first point repeated) is accepted: stop'), dyn(CLOSED)),
    shape(DEGENERATE),
    polys(cmd(V, V, 'three equal points are accepted: the zone contains nothing', state=NORMAL),
          dyn(DEGENERATE)),
]

# A NaN in a vertex does not blind upstream's zone in general (review CM5,
# finding 1): the crossing test skips the two edges at the NaN vertex and
# counts the others. Which points the zone then catches is an accident of the
# arithmetic: with the NaN in the 4th vertex of a 2 m square, a reading inside
# the square and one 5 m behind it both stop; with it in the 1st vertex the
# reading inside PASSES and the one behind stops. (Observed with
# review/collision_monitor/nav2/review_probe.py nan-vertex.)
SQUARE = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
NAN4 = SQUARE[:3] + [(NAN, 1.0)]
NAN1 = [(NAN, 1.0)] + SQUARE[1:]
INSIDE = ray(0.5)
FAR_BEHIND = ray(5.0, PI)
POLYGON_TOPIC_NAN = [
    clock(T),
    shape(NAN4),
    scan(INSIDE),
    cmd(V, ZERO, 'the FIRST shape has a NaN in its 4th vertex, reading 0.5 m ahead: stop',
        state=DYN),
    scan(FAR_BEHIND),
    cmd(V, ZERO, 'reading 5 m behind, outside the square: STILL A STOP'),
    shape(NAN1),
    scan(INSIDE),
    cmd(V, V, 'NaN in the 1st vertex instead, reading 0.5 m ahead, inside the square: PASSES',
        state=NORMAL),
    scan(FAR_BEHIND),
    cmd(V, ZERO, 'NaN in the 1st vertex, reading 5 m behind: stop', state=DYN),
    shape(SQUARE[:2]),
    cmd(V, ZERO, 'a message with two points is ignored: nothing changes'),
    shape(SQUARE),
    cmd(V, V, 'the square without NaN: the reading 5 m behind is outside, passes', state=NORMAL),
    scan(INSIDE),
    cmd(V, ZERO, 'the reading 0.5 m ahead is inside the square: stop', state=DYN),
]

DYN_SLOW = ('Dyn', SLOWDOWN)
POLYGON_TOPIC_MOVING = [
    clock(T),
    cmd(V, ZERO, 'no scan: stop (the zone is not reached)', state=INVALID),
    tf(xf(BASE, 'carrier', stamp=T)),
    shape(BOX, frame='carrier'),
    scan(NEAR),
    cmd(V, HALF, 'box given in the frame "carrier", which coincides with the base: slowdown',
        state=DYN_SLOW),
    tf(xf(BASE, 'carrier', x=2.0, stamp=T + s(1))),
    cmd(V, V, 'carrier has moved 2 m ahead, NO new shape message: the zone has moved with it',
        state=NORMAL),
    scan(ray(2.25)),
    cmd(V, HALF, 'point 2.25 m ahead: inside the moved box', state=DYN_SLOW),
]

DL = ('DynLatched', STOP)
POLYGON_TOPIC_INACTIVE = [
    clock(T),
    shape(BOX),
    transition('configure'),
    qos('with polygon_subscribe_transient_local the subscription is transient local, still best '
        'effort', 'sub', 'zone_shape', 'BEST_EFFORT', 'TRANSIENT_LOCAL'),
    transition('activate'),
    scan(NEAR),
    cmd(V, V, 'a box published latched (reliable, transient local) before configure was NOT '
        'delivered: passes'),
    transition('deactivate'),
    shape(TRI),
    transition('activate'),
    cmd(V, V, 'active again, point ahead: outside the triangle'),
    scan(BEHIND),
    cmd(V, ZERO, 'point behind: the triangle sent while inactive is the shape', state=DL),
    transition('deactivate'), transition('cleanup'), transition('configure'),
    transition('activate'),
    scan(BEHIND),
    cmd(V, V, 'after cleanup, configure, activate and no new message: the shape is gone, passes'),
]

FOOT = ('Foot', STOP)
FOOTPRINT_TOPIC_IGNORED = [
    graph('fixed points: no subscription to polygon_sub_topic', 'sub', 'zone_shape', 0),
    graph('fixed points: no subscription to footprint_topic', 'sub',
          '/local_costmap/published_footprint', 0),
    clock(T), scan(NEAR),
    cmd(V, ZERO, 'the fixed points are the zone', state=FOOT),
]

# ---- Nav2's footprint zone edited (N29, N32, N34; rows 20 and 21) --------------

FOOTPRINT = '/local_costmap/published_footprint'
FA = ('FootprintApproach', STOP)
UPSTREAM_REFUSAL = (
    "parameter 'FootprintApproach.polygon_sub_topic' is not initialized",
    'static points and sub topic both not defined')


def package_file(package, *path):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        return os.path.join(get_package_share_directory(package), *path)
    except PackageNotFoundError:
        return None


def nav2_params(action, extra=None):
    """The collision_monitor section of nav2_bringup's nav2_params.yaml with
    FootprintApproach.action_type changed and, if given, further keys of the zone."""
    def make():
        import yaml
        path = package_file('nav2_bringup', 'params', 'nav2_params.yaml')
        if path is None:
            return None
        with open(path) as f:
            section = yaml.safe_load(f)['collision_monitor']
        zone = section['ros__parameters']['FootprintApproach']
        zone['action_type'] = action
        zone.update(extra or {})
        return yaml.safe_dump({'collision_monitor': section})
    return make


def example_params():
    """The package's collision_monitor_params.yaml with `polygons: [FootprintApproach]`
    and that zone's action_type changed to stop."""
    import yaml
    path = package_file('nav2_collision_monitor', 'params', 'collision_monitor_params.yaml')
    if path is None:
        return None
    with open(path) as f:
        doc = yaml.safe_load(f)
    prm = doc['collision_monitor']['ros__parameters']
    prm['polygons'] = ['FootprintApproach']
    prm['FootprintApproach']['action_type'] = 'stop'
    return yaml.safe_dump(doc)


FOOTPRINT_EDIT = [
    transition('configure', 'Nav2\'s zone with only action_type changed does not configure',
               ok=False),
    lcstate('unconfigured', 'the node stays unconfigured (and the process stays up)'),
    log('the node says why', *UPSTREAM_REFUSAL),
    graph('no command subscription', 'sub', 'cmd_vel_in', 0),
]

FOOTPRINT_UNCHANGED = [
    transition('configure', 'unchanged, with action_type approach, the file configures'),
    graph('the approach zone subscribes to footprint_topic', 'sub', FOOTPRINT, 1),
]

# nav2_params.yaml: base_shift_correction true, transform_tolerance 0.2,
# source_timeout 1.0, stop_pub_timeout 2.0, min_points 6. The footprint is a
# 0.5 m square; the base is at (5, 3) in odom, later at (6, 3).
RING = dict(ranges=[0.125] * 8, angle_increment=0.01)
RING_BEHIND = dict(ranges=[1.125] * 8, angle_min=PI, angle_increment=0.0)
SQUARE = [(5.25, 3.25), (5.25, 2.75), (4.75, 2.75), (4.75, 3.25)]
SQUARE2 = [(x + 1.0, y) for x, y in SQUARE]
FOOTPRINT_SUB_TOPIC = [
    qos('the footprint subscription is best effort and volatile', 'sub', FOOTPRINT,
        'BEST_EFFORT', 'VOLATILE'),
    clock(T),
    tf(odom(T, x=5.0, y=3.0), odom(T + s(1), x=5.0, y=3.0)),
    scan(RING),
    cmd(V, V, 'no footprint message yet, 8 points 0.125 m ahead (min_points 6): PASSES'),
    shape(SQUARE, frame='odom', topic=FOOTPRINT),
    cmd(V, ZERO, 'footprint received, a square around the base given in odom: stop', state=FA),
    tf(odom(T + s(2), x=6.0, y=3.0), odom(T + s(3), x=6.0, y=3.0)),
    clock(T + s(2)),
    scan(RING),
    cmd(V, V, 'the base has moved 1 m and no new footprint has come: the same points PASS',
        state=NORMAL),
    scan(RING_BEHIND),
    cmd(V, ZERO, '8 points 1.125 m behind the base, where the footprint was left: stop', state=FA),
    scan(RING),
    cmd(V, V, 'points ahead again: pass', state=NORMAL),
    shape(SQUARE2, frame='odom', topic=FOOTPRINT),
    cmd(V, ZERO, 'a footprint message at the new position: stop', state=FA),
]

# The same zone with slowdown: slowdown_ratio takes its default, 0.5.
FOOTPRINT_SUB_TOPIC_SLOW = [
    clock(T),
    tf(odom(T, x=5.0, y=3.0), odom(T + s(1), x=5.0, y=3.0)),
    scan(RING),
    cmd(V, V, 'no footprint message yet: passes'),
    shape(SQUARE, frame='odom', topic=FOOTPRINT),
    cmd(V, HALF, 'footprint received: slowed to the default ratio 0.5',
        state=('FootprintApproach', SLOWDOWN)),
]

SUB = {'polygon_sub_topic': FOOTPRINT}
IN = 'cmd_vel_in_topic:=cmd_vel_in'


# name -> dict(family, steps, params[, start][, tf][, watch][, shapes][, file][, latched])
#   family   the group of the task brief the scenario belongs to
#   params   `-p` arguments on top of zones_nav2.yaml (or of `file`)
#   start    'active' (default: configured and activated before the first step)
#            or 'unconfigured' (the steps drive the transitions)
#   tf       static transforms published before the node starts
#   watch    polygon topics to record
#   shapes   polygon_sub_topic names the node must have subscribed to once configured
#   file     a function returning the text of a parameter file to use instead
#            of zones_nav2.yaml (None: the file's package is not installed)
#   latched  shapes are published reliable and transient local
SCENARIOS = {
    'lifecycle-states': dict(
        family='lifecycle', params=[P('StopCircle', 'FrontBox')] + VIS, steps=LIFECYCLE_STATES,
        start='unconfigured', watch=['front_box', 'StopCircle']),
    'lifecycle-reactivate': dict(
        family='lifecycle', params=[P('StopCircle'), 'stop_pub_timeout:=1.0'],
        steps=LIFECYCLE_REACTIVATE),
    'lifecycle-cleanup': dict(family='lifecycle', params=[P('StopCircle')],
                              steps=LIFECYCLE_CLEANUP),
    'lifecycle-shutdown-active': dict(family='lifecycle', params=[P('StopCircle')],
                                      steps=LIFECYCLE_SHUTDOWN_ACTIVE),
    'lifecycle-shutdown-inactive': dict(family='lifecycle', params=[P('StopCircle')],
                                        steps=LIFECYCLE_SHUTDOWN_INACTIVE, start='unconfigured'),
    'lifecycle-shutdown-deactivated': dict(family='lifecycle', params=[P('StopCircle')],
                                           steps=LIFECYCLE_SHUTDOWN_DEACTIVATED),

    'state-topic': dict(
        family='state', params=[P('StopCircle', 'SlowCircle'), 'stop_pub_timeout:=1.0'],
        steps=STATE_TOPIC),
    'state-name-collision': dict(family='state', params=[P('"invalid source"')],
                                 steps=STATE_NAME_COLLISION),
    'polygon-pub': dict(
        family='state', params=[P('StopCircle', 'FrontBox'), 'stop_pub_timeout:=1.0'] + VIS,
        steps=POLYGON_PUB, start='unconfigured', watch=['front_box', 'StopCircle']),

    'tf-latest': dict(family='transforms', params=[P('StopCircle')], steps=TF_LATEST),
    'tf-tilt': dict(family='transforms', params=[P('RightBox')], steps=TF_TILT, tf=TILT_TF),
    'tf-pitch-circle': dict(family='transforms', params=[P('StopCircle')], steps=TF_PITCH_CIRCLE,
                            tf=TILT_TF),
    'tf-missing': dict(family='transforms', params=[P('StopCircle')], steps=tf_missing(s(0.5))),
    # tf2::durationFromSec rounds the tolerance to the nearest nanosecond
    # (source_timeout, by contrast, is truncated: scenario.py, timeout-decimal).
    'tf-tolerance-decimal': dict(
        family='transforms', params=[P('StopCircle'), 'transform_tolerance:=1.001'],
        steps=tf_missing(1_001_000_000)),
    'tf-tolerance-0.3': dict(
        family='transforms', params=[P('StopCircle'), 'transform_tolerance:=0.3'],
        steps=tf_missing(300_000_000)),
    'tf-missing-timeout0': dict(
        family='transforms',
        params=[P('StopCircle'), 'transform_tolerance:=0.0', 'source_timeout:=0.0'],
        steps=TF_MISSING_TIMEOUT0),
    'tf-tolerance0': dict(family='transforms', params=[P('StopCircle'), 'transform_tolerance:=0.0'],
                          steps=TF_TOLERANCE0),
    'tf-wait-timeout0': dict(family='transforms', params=[P('StopCircle'), 'source_timeout:=0.0'],
                             steps=TF_WAIT_TIMEOUT0),

    'tf-clock-jump-back': dict(family='transforms', params=[P('StopCircle')],
                               steps=TF_CLOCK_JUMP_BACK, tf=[xf(BASE, 'laser')]),

    'tf-wait-clock-reset': dict(family='transforms', params=[P('StopCircle')],
                                steps=TF_WAIT_CLOCK_RESET),
    'tf-wait-clock-back': dict(family='transforms', params=[P('StopCircle')],
                               steps=TF_WAIT_CLOCK_BACK),
    'shift-wait-reset': dict(family='shift', params=[P('StopCircle'), 'base_shift_correction:=true'],
                             steps=SHIFT_WAIT_RESET),

    'tf-bad-quaternion': dict(family='transforms', params=[P('StopCircle')],
                              steps=TF_BAD_QUATERNION),

    'shift-straight': dict(family='shift', params=[P('StopCircle'), 'base_shift_correction:=true'],
                           steps=SHIFT_STRAIGHT),
    'shift-interpolation': dict(
        family='shift', params=[P('StopCircle'), 'base_shift_correction:=true'],
        steps=SHIFT_INTERPOLATION),
    'shift-off': dict(family='shift', params=[P('StopCircle')], steps=SHIFT_OFF),
    'shift-turn': dict(family='shift', params=[P('FrontBox'), 'base_shift_correction:=true'],
                       steps=SHIFT_TURN),
    'shift-quarter-turn': dict(family='shift', params=[P('RightBox'), 'base_shift_correction:=true'],
                               steps=SHIFT_QUARTER),
    'shift-sensor-frame': dict(
        family='shift', params=[P('StopCircle'), 'base_shift_correction:=true'],
        steps=SHIFT_SENSOR_FRAME, tf=[xf(BASE, 'laser', x=0.5)]),
    'shift-no-odom': dict(family='shift', params=[P('StopCircle'), 'base_shift_correction:=true'],
                          steps=SHIFT_NO_ODOM),

    'polygon-topic': dict(
        family='topic-zones', params=[P('Dyn'), 'Dyn.visualize:=true'], steps=POLYGON_TOPIC,
        start='unconfigured', watch=['Dyn'], shapes=['zone_shape']),
    'polygon-topic-nan': dict(
        family='topic-zones', params=[P('Dyn')], steps=POLYGON_TOPIC_NAN, shapes=['zone_shape']),
    'polygon-topic-moving': dict(
        family='topic-zones', params=[P('Dyn'), 'Dyn.action_type:=slowdown'],
        steps=POLYGON_TOPIC_MOVING, shapes=['zone_shape']),
    'polygon-topic-inactive': dict(
        family='topic-zones', params=[P('DynLatched')], steps=POLYGON_TOPIC_INACTIVE,
        start='unconfigured', shapes=['zone_shape'], latched=True),
    'footprint-topic-ignored': dict(family='topic-zones', params=[P('Foot')],
                                    steps=FOOTPRINT_TOPIC_IGNORED),

    'footprint-edit-stop': dict(family='footprint', file=nav2_params('stop'), params=[IN],
                                steps=FOOTPRINT_EDIT, start='unconfigured'),
    'footprint-edit-slowdown': dict(family='footprint', file=nav2_params('slowdown'), params=[IN],
                                    steps=FOOTPRINT_EDIT, start='unconfigured'),
    'footprint-edit-example-file': dict(family='footprint', file=example_params, params=[IN],
                                        steps=FOOTPRINT_EDIT, start='unconfigured'),
    'footprint-approach-unchanged': dict(family='footprint', file=nav2_params('approach'),
                                         params=[IN], steps=FOOTPRINT_UNCHANGED,
                                         start='unconfigured'),
    'footprint-sub-topic': dict(family='footprint', file=nav2_params('stop', SUB), params=[IN],
                                steps=FOOTPRINT_SUB_TOPIC, shapes=[FOOTPRINT]),
    'footprint-sub-topic-slowdown': dict(
        family='footprint', file=nav2_params('slowdown', SUB), params=[IN],
        steps=FOOTPRINT_SUB_TOPIC_SLOW, shapes=[FOOTPRINT]),
}


# ---- the runner ----------------------------------------------------------------

def same_coord(a, b):
    return (math.isnan(a) and math.isnan(b)) or f32(a) == f32(b)


def polygon_matches(msg, frame, points):
    got = [(p.x, p.y) for p in msg.polygon.points]
    if msg.header.frame_id != frame or any(p.z != 0.0 for p in msg.polygon.points):
        return False
    if isinstance(points, tuple) and points and points[0] == 'circle':
        # 16 points radius * (cos, sin)(k * pi / 8), computed in double and
        # narrowed to float32; compared to a float32 step or two, because
        # the last bit of cos and sin is the platform's.
        r = points[1]
        return len(got) == 16 and all(
            abs(x - r * math.cos(k * PI / 8)) < 1e-6 and abs(y - r * math.sin(k * PI / 8)) < 1e-6
            for k, (x, y) in enumerate(got))
    return len(got) == len(points) and all(
        same_coord(gx, wx) and same_coord(gy, wy) for (gx, gy), (wx, wy) in zip(got, points))


def describe_polys(got):
    return '[' + ', '.join(
        f'{t}: {len(m.polygon.points)} points in {m.header.frame_id!r} at '
        f'{m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec}' for t, m in got) + ']'


def check_polys(got, want, now_ns):
    """Why the polygons published by a step are not the expected ones, or None."""
    if len(got) != len(want):
        return f'expected {len(want)} polygon message(s), got {describe_polys(got)}'
    left = list(got)
    for topic, frame, points in want:
        hit = [g for g in left if g[0] == topic and polygon_matches(g[1], frame, points)]
        if not hit:
            return f'no polygon on {topic} in {frame!r} with the expected points: {describe_polys(got)}'
        left.remove(hit[0])
    for _, m in got:
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        if stamp != now_ns:
            return f'polygon stamped {stamp}, the node clock is {now_ns}'
    return None


def override_for(expect, label):
    hits = [(k, v) for k, v in expect.get('diffs', {}).items() if k in label]
    if len(hits) > 1:
        raise SystemExit(f'profile: more than one expected difference matches {label!r}')
    return hits[0] if hits else (None, None)


class Run:
    """One scenario on one node."""

    def __init__(self, exe, name, sc, expect, opts):
        self.name, self.sc, self.expect, self.opts = name, sc, expect, opts
        self.lines, self.tags, self.used = [], [], set()
        self.side = Side(exe, 'collision_monitor', 'cmd_vel_out', 'cm_state')
        self.d = None
        self.ident = 0
        self.last_cmd = None
        self.watching = bool(sc.get('watch'))

    def note(self, ok, text):
        self.lines.append((ok, text))

    def expected(self, label, out, state, want_polys):
        """The expectation of a check after the profile's override, and a marker."""
        key, ov = override_for(self.expect, label)
        if ov is None:
            return out, state, want_polys, ''
        self.used.add(key)
        self.tags.append(ov['tag'])
        new = (ov.get('out', out), ov.get('state', state), ov.get('polys', want_polys))
        if repr(new) == repr((out, state, want_polys)):
            self.note(False, f'profile: the expected difference {label!r} is upstream\'s answer')
        return new + (f"[{ov['tag']}: {ov['why']}] ",)

    def wait_configured(self, up):
        """After configure: every subscription and publisher is there; after cleanup: gone."""
        d, side, sc = self.d, self.side, self.sc
        n = 1 if up else 0

        def ready():
            return (d.cmd_pub.get_subscription_count() == n
                    and d.scan_pub.get_subscription_count() == n
                    and d.count_publishers(side.out_topic) == n
                    and len(d.endpoints_of(side, '/tf', False)) == n
                    and len(d.endpoints_of(side, '/tf_static', False)) == n
                    and all(p.get_subscription_count() == n for p in d.shape_pubs.values()))

        if not d.spin_until(ready, 20.0):
            raise RuntimeError('the node\'s endpoints did not %s after the transition'
                               % ('appear' if up else 'go away'))
        d.spin_for(0.5)

    def do_transition(self, name):
        ok = self.d.change_state(self.side, TRANSITIONS[name])
        if ok and name == 'configure' and self.sc.get('connect', True):
            self.wait_configured(True)
        elif ok and name == 'cleanup':
            self.wait_configured(False)
        else:
            self.d.spin_for(self.opts.settle)
        return ok

    def answer(self, before, out, state, want_polys, label, mark):
        d, side = self.d, self.side
        if want_polys:
            d.spin_until(lambda: len(side.polys) - before[2] >= len(want_polys), 2.0)
        outs, states = side.out[before[0]:], side.states[before[1]:]
        why = check_answer(d, self.last_cmd, outs, states, out, state, True)
        if why is None and self.watching:
            why = check_polys(side.polys[before[2]:], want_polys, d.now_ns)
        text = describe(d, outs, states)
        if self.watching:
            text += f' + {len(side.polys) - before[2]} polygon(s)'
        self.note(why is None, f'{mark}{text}   {label}' + (f'   [{why}]' if why else ''))

    def input_step(self, step, settle):
        d = self.d
        if step[0] == 'clock':
            d.set_clock(step[1], settle)
        elif step[0] == 'scan':
            d.send_scan(step[1], settle)
        elif step[0] == 'tf':
            d.publish_dynamic(step[1], settle)
        elif step[0] == 'static':
            d.publish_static(step[1], settle)
        elif step[0] == 'shape':
            _, topic, points, frame = step
            d.shape_publisher(topic, self.sc.get('latched', False))
            d.send_shape(topic, points, frame, settle)
        else:
            raise SystemExit(f'not an input step: {step[0]}')

    def step(self, step):
        d, side, settle = self.d, self.side, self.opts.settle
        want_polys = ()
        if step[0] == 'polys':
            _, step, want_polys = step
        before = (len(side.out), len(side.states), len(side.polys))
        kind = step[0]
        if kind == 'cmd':
            _, vel, out, state, label = step
            out, state, want_polys, mark = self.expected(label, out, state, want_polys)
            self.ident += 1
            msg = self.last_cmd = d.make_cmd(vel, self.ident)
            want_n = 0 if out is None else 1
            d.send_cmd(msg, settle,
                       wait_for=(lambda i, o, st: len(o) >= want_n) if want_n else None)
            self.answer(before, out, state, want_polys, label, mark)
            return
        if kind in ('late', 'arrives'):
            if kind == 'late':
                _, t_ns, out, state, label = step
                inner = clock(t_ns)
            else:
                _, inner, out, state, label = step
            out, state, want_polys, mark = self.expected(label, out, state, want_polys)
            self.input_step(inner, 0.0)
            d.spin_until(lambda: len(side.out) > before[0], 2.0)
            d.spin_for(settle)
            self.answer(before, out, state, want_polys, label, mark)
            return
        if kind == 'transition':
            _, name, ok, label = step
            got = self.do_transition(name)
            if want_polys:
                d.spin_until(lambda: len(side.polys) - before[2] >= len(want_polys), 2.0)
            why = None if got == ok else f'expected success={ok}'
            if why is None and (len(side.out) != before[0] or len(side.states) != before[1]):
                why = 'the transition published ' + describe(
                    d, side.out[before[0]:], side.states[before[1]:])
            if why is None and self.watching:
                why = check_polys(side.polys[before[2]:], want_polys, d.now_ns)
            self.note(why is None, f'{name}: success={got}'
                      + (f', {len(side.polys) - before[2]} polygon(s)' if self.watching else '')
                      + (f'   {label}' if label else '') + (f'   [{why}]' if why else ''))
            return
        if kind == 'quiet':
            d.spin_for(settle)
            self.note(True, f'nothing        {step[1]}')
        elif kind == 'graph':
            _, label, what, topic, count = step
            got = []

            def settled():
                got[:] = d.endpoints_of(side, topic, what == 'pub')
                return len(got) == count
            d.spin_until(settled, 5.0)
            self.note(len(got) == count, f'{len(got)} {what} on {topic}   {label}')
        elif kind == 'qos':
            _, label, what, topic, reliability, durability = step
            got = []
            d.spin_until(lambda: bool(got.extend(d.endpoints_of(side, topic, what == 'pub'))
                                      or got), 5.0)
            have = sorted({(i.qos_profile.reliability.name, i.qos_profile.durability.name)
                           for i in got})
            self.note(have == [(reliability, durability)], f'{what} on {topic}: {have}   {label}')
        elif kind == 'lcstate':
            got = d.get_state(side)
            self.note(got == step[1], f'lifecycle state {got}' + (f'   {step[2]}' if step[2] else ''))
        elif kind == 'log':
            _, label, needles = step
            if 'log' in self.expect:
                tag, why, needles = self.expect['log']
                self.tags.append(tag)
                label = f'[{tag}: {why}] {label}'

            def found():
                with open(self.log_path) as f:
                    text = f.read()
                return all(n in text for n in needles)
            ok = d.spin_until(found, 3.0)
            self.note(ok, f'the node\'s output contains {list(needles)}   {label}')
        else:
            self.input_step(step, settle)
        # None of these steps may make the node publish.
        if len(side.out) != before[0] or len(side.states) != before[1] or (
                self.watching and len(side.polys) != before[2]):
            self.note(False, f'the node published without a command after a {kind} step: '
                      + describe(d, side.out[before[0]:], side.states[before[1]:])
                      + (' ' + describe_polys(side.polys[before[2]:]) if self.watching else ''))

    def run(self):
        sc, opts, side = self.sc, self.opts, self.side
        params = list(sc['params'])
        if 'file' in sc:
            text = sc['file']()
            if text is None:
                print(f'--- {self.name}: not run (the parameter file\'s package is not installed)')
                return None
            tmp = tempfile.mkdtemp(prefix='roslean_cm_nav2_')
            params_file = os.path.join(tmp, 'params.yaml')
            with open(params_file, 'w') as f:
                f.write(text)
        else:
            params_file = os.path.join(HERE, 'zones_nav2.yaml')
        if opts.logs:
            self.log_path = os.path.join(opts.logs, f'{self.name}.log')
        else:
            self.log_path = os.path.join(tempfile.mkdtemp(prefix='roslean_cm_nav2_'), 'node.log')
        steps = sc['steps']
        if 'refused' in self.expect:
            tag, why = self.expect['refused']
            self.tags.append(tag)
            steps = [transition('configure', f'[{tag}: {why}] the configuration is refused',
                                ok=False)]
        self.d = d = Driver('scenario_nav2_driver', [side], False)
        # The node of the previous scenario had the same name. Until its
        # endpoints have left the graph, "the node has subscribed to /clock"
        # below could be true of the old process, and the first clock
        # message would be sent to nobody.
        d.spin_until(lambda: not d.endpoints_of(side, '/clock', False), 10.0)
        side.start(params_file, params, self.log_path)
        try:
            for topic in sc.get('watch', []):
                d.watch_polygon(side, topic)
            for topic in sc.get('shapes', []):
                d.shape_publisher(topic, sc.get('latched', False))
            d.publish_tf(sc.get('tf', []))
            if not d.detect_lifecycle(side, 'auto'):
                self.note(False, 'the node offers no lifecycle services (or has exited)')
            else:
                # The node's clock subscription exists from start-up; a clock
                # message sent before it is matched would be lost.
                if not d.spin_until(lambda: d.clock_pub.get_subscription_count() >= 1
                                    and len(d.endpoints_of(side, '/clock', False)) == 1, 20.0):
                    raise RuntimeError('the node did not subscribe to /clock')
                d.spin_for(0.3)
                if sc.get('start', 'active') == 'active' and 'refused' not in self.expect:
                    for name in ('configure', 'activate'):
                        if not self.do_transition(name):
                            raise RuntimeError(f'{name} failed')
                for step in steps:
                    self.step(step)
                d.spin_for(opts.settle)
                if side.proc.poll() is not None:
                    self.note(False, f'the node exited with code {side.proc.returncode}')
                for key in self.expect.get('diffs', {}):
                    if key not in self.used:
                        self.note(False, f'profile: no check matched the expected difference {key!r}')
        except RuntimeError as e:
            self.note(False, str(e))
        finally:
            side.stop()
            d.destroy_node()
        print(f'--- {self.name}  [{sc["family"]}]  ({" ".join(params)})')
        for ok, text in self.lines:
            print(f"{'ok  ' if ok else 'FAIL'}  {text}")
        sys.stdout.flush()
        return len(self.lines), sum(not ok for ok, _ in self.lines), self.tags


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node', nargs='?', default=UPSTREAM)
    ap.add_argument('--only', default='')
    ap.add_argument('--skip', default='')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--settle', type=float, default=0.2)
    ap.add_argument('--logs', default='')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--profile', default='')
    ap.add_argument('--first-failure', action='store_true')
    opts = ap.parse_args()
    expect = {}
    if opts.profile:
        import importlib
        expect = importlib.import_module(opts.profile).EXPECT
        unknown = [n for n in expect if n not in SCENARIOS]
        if unknown:
            sys.exit(f'profile: unknown scenario(s): {unknown}')
    if opts.list:
        for name, sc in SCENARIOS.items():
            print(f'{name:32} {sc["family"]}')
        return
    families = {sc['family'] for sc in SCENARIOS.values()}

    def select(text):
        names = []
        for n in (x for x in text.split(',') if x):
            if n in families:
                names += [k for k, sc in SCENARIOS.items() if sc['family'] == n]
            elif n in SCENARIOS:
                names.append(n)
            else:
                sys.exit(f'unknown scenario or family: {n}')
        return names

    skip = select(opts.skip)
    names = [n for n in (select(opts.only) or list(SCENARIOS)) if n not in skip]
    if opts.logs:
        os.makedirs(opts.logs, exist_ok=True)
    exe = resolve(opts.node)
    print(f'node under test: {exe}')
    rclpy.init()
    per_family = {}
    failed, tags, not_run = [], {}, []
    stop = False
    try:
        for rep in range(opts.repeat):
            if opts.repeat > 1:
                print(f'=== run {rep + 1} of {opts.repeat}')
            for name in names:
                if 'skip' in expect.get(name, {}):
                    tag, why = expect[name]['skip']
                    print(f'--- {name}: not run [{tag}: {why}]')
                    not_run.append(name)
                    continue
                result = Run(exe, name, SCENARIOS[name], expect.get(name, {}), opts).run()
                if result is None:
                    not_run.append(name)
                    continue
                fam = per_family.setdefault(SCENARIOS[name]['family'], [0, 0, 0])
                fam[0] += 1
                fam[1] += result[0]
                fam[2] += result[1]
                if result[1]:
                    failed.append(name)
                    stop = opts.first_failure
                for tag in result[2]:
                    tags[tag] = tags.get(tag, 0) + 1
                if stop:
                    break
            if stop:
                break
    finally:
        rclpy.shutdown()
    for fam, (n, checks, bad) in per_family.items():
        print(f'{fam:12} {n} scenario runs, {checks} checks, {bad} failure(s)')
    checks = sum(v[1] for v in per_family.values())
    bad = sum(v[2] for v in per_family.values())
    print(f'{bad} failure(s) in {checks} checks, {sum(v[0] for v in per_family.values())} '
          'scenario runs'
          + (f', {len(not_run)} not run: {sorted(set(not_run))}' if not_run else '')
          + (f'; failed: {sorted(set(failed))}' if failed else '')
          + ('; stopped at the first failure' if stop else ''))
    if opts.profile:
        print('expected differences from upstream, by number in the SPEC: '
              + (', '.join(f'{t}: {n}' for t, n in sorted(tags.items())) or 'none'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
