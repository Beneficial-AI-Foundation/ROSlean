#!/usr/bin/env python3
"""Drives a collision-monitor node through fixed scenarios over real ROS topics.

    scenario.py [NODE] [options]

NODE is a path to an executable or `<package>/<executable>`; the default is
upstream's `nav2_collision_monitor/collision_monitor`. The expectations are
what that upstream binary (Nav2 1.3.12, ROS 2 Jazzy) was observed to do; see
docs/collision_monitor/UPSTREAM_BEHAVIOUR.md, which refers to the scenarios
here by name.

Options:
  --lifecycle auto|yes|no   whether to configure and activate the node through
                            its lifecycle services (default: auto-detect)
  --no-tf                   never publish transforms; scenarios that need one
                            are skipped
  --ignore-state            do not check the state topic
  --only A,B,...            run only these scenarios
  --skip A,B,...            leave these scenarios out
  --repeat N                run everything N times (to look for flakiness)
  --settle S                wall seconds allowed for delivery (default 0.2)
  --logs DIR                keep the node's output, one file per scenario
  --list                    list the scenarios and exit
  --profile MODULE          the node is not upstream's: MODULE (a Python module
                            next to this script, e.g. profile_roslean) states,
                            scenario by scenario, every answer that is expected
                            to differ from upstream's and why, the scenarios
                            the node must refuse, and scenarios of its own.
                            Everything not stated must be as upstream.
  --own                     with --profile: run only the profile's own scenarios

The node runs with `use_sim_time`; this script publishes `/clock`, the scans,
the commands and (where a scenario asks for it) static transforms.
"""

import argparse
import math
import os
import sys

import rclpy

from cm_harness import (
    DO_NOTHING, HERE, SLOWDOWN, STOP, UPSTREAM, Driver, Side, f32_next, fan, fmt, ray, resolve,
    s, same_floats, static_tf, twist6)

PI = math.pi
INF, NAN = math.inf, math.nan
INVALID = ('invalid source', STOP)   # the state reported when the scan cannot be used
NORMAL = ('', DO_NOTHING)            # the state reported on return to pass-through

V = (0.5, 0.25, 1.0)
ZERO = (0.0, 0.0, 0.0)
CLEAR = fan(5, 5.0)        # five returns far outside every zone
NEAR = fan(3, 0.25)        # three returns inside every zone
MID = fan(3, 0.75)         # three returns inside the 1.0 zones only


# Steps. Every command says what must come out: `out` is None (nothing is
# published) or (linear.x, linear.y, angular.z), the other three components
# being 0; `state` is None (no state message) or (polygon_name, action_type).

def clock(t_ns):
    return ('clock', t_ns)


def scan(spec):
    return ('scan', spec)


def cmd(vel, out, label, state=None):
    return ('cmd', vel, out, state, label)


def quiet(label):
    """Nothing may have been published since the previous step."""
    return ('quiet', label)


def late(t_ns, out, label, state=None):
    """Advance the clock; the previous command, so far unanswered, is answered now."""
    return ('late', t_ns, out, state, label)


def crash(vel, label):
    """A command that is not answered and after which the node process is gone."""
    return ('crash', vel, label)


def scaled(v, k):
    return tuple(x * k for x in v)


T = s(100)

PASS_THROUGH = [
    clock(T), scan(CLEAR),
    cmd((0.5, 0.25, -1.5, 0.75, -0.125, 1.0), V,
        'no obstacle: x, y and yaw rate pass; linear.z and angular.x/y come out as 0'),
    cmd((0.5, 0.0, 0.0), (0.5, 0.0, 0.0), 'second command passes'),
    cmd((1e-300, 0.0, 0.0), (1e-300, 0.0, 0.0), 'a tiny velocity passes unchanged'),
    cmd((1e300, -1e300, 1e300), (1e300, -1e300, 1e300), 'a huge velocity passes unchanged'),
]

STOP_CIRCLE = [
    clock(T), scan(NEAR),
    cmd(V, ZERO, '3 points in the stop circle: zero velocity', state=('StopCircle', STOP)),
    cmd(V, ZERO, 'still stopped; the state is reported only on change'),
    scan(CLEAR),
    cmd(V, V, 'obstacle gone: passes again', state=NORMAL),
    scan(NEAR),
    cmd(V, ZERO, 'obstacle back: stopped', state=('StopCircle', STOP)),
]

STOP_POLYGON = [
    clock(T), scan(NEAR),
    cmd(V, ZERO, '3 points in the stop square: zero velocity', state=('StopPoly', STOP)),
    scan(CLEAR),
    cmd(V, V, 'obstacle gone: passes again', state=NORMAL),
    # (0.44, 0.44): inside the square of half-width 0.5, outside a circle of radius 0.5.
    scan(fan(3, 0.625, centre=PI / 4)),
    cmd(V, ZERO, 'points in the corner of the square: stopped', state=('StopPoly', STOP)),
    scan(fan(3, 0.625)),
    cmd(V, V, 'the same range straight ahead is outside the square', state=NORMAL),
]

SLOWDOWN_STEPS = [
    clock(T), scan(MID),
    cmd(V, scaled(V, 0.5), '3 points in the slowdown square: velocity halved',
        state=('SlowPoly', SLOWDOWN)),
    cmd((0.5, 0.25, -1.5, 0.75, -0.125, 1.0), scaled(V, 0.5),
        'slowed; linear.z and angular.x/y come out as 0'),
    scan(CLEAR),
    cmd(V, V, 'obstacle gone: full speed', state=NORMAL),
]

SLOWDOWN_RATIO = [
    clock(T), scan(MID),
    cmd(V, scaled(V, 0.3), 'ratio 0.3: each component is the double product v * 0.3',
        state=('SlowWide', SLOWDOWN)),
    cmd((0.1, 0.7, -0.9), scaled((0.1, 0.7, -0.9), 0.3), 'same, values that are not binary fractions'),
]

# Both orders of a slowdown zone and a stop zone give the same outputs.
BOTH = [
    clock(T), scan(CLEAR),
    cmd(V, V, 'no obstacle'),
    scan(MID),
    cmd(V, scaled(V, 0.5), 'points in the slowdown zone only: halved', state=('SlowPoly', SLOWDOWN)),
    scan(NEAR),
    cmd(V, ZERO, 'points in both zones: stop wins', state=('StopCircle', STOP)),
    scan({'ranges': [0.25, 0.25, 0.75, 0.75, 0.75], 'angle_min': -0.02, 'angle_increment': 0.01}),
    cmd(V, scaled(V, 0.5), '2 points in the stop zone (needs 3), all 5 in the slowdown zone: halved',
        state=('SlowPoly', SLOWDOWN)),
    scan(CLEAR),
    cmd(V, V, 'no obstacle', state=NORMAL),
]


# SlowWide (ratio 0.3) and SlowPoly (ratio 0.5) both triggered: the smaller
# resulting velocity wins whatever the order of the zones.
TWO_SLOWDOWNS = [
    clock(T), scan(MID),
    cmd(V, scaled(V, 0.3), 'two slowdown zones triggered: the lower ratio (0.3) wins',
        state=('SlowWide', SLOWDOWN)),
    scan(fan(3, 1.5)),
    cmd(V, scaled(V, 0.3), 'points in the wide zone only: 0.3'),
]


MIN_POINTS = [
    clock(T), scan(fan(2, 0.25)),
    cmd(V, V, '2 points inside, min_points 3: passes'),
    scan(fan(3, 0.25)),
    cmd(V, ZERO, '3 points inside, min_points 3: stopped', state=('StopCircle', STOP)),
    scan({'ranges': [0.25, 0.25, 5.0, 5.0], 'angle_increment': 0.01}),
    cmd(V, V, '4 points, only 2 of them inside: passes', state=NORMAL),
    scan(fan(4, 0.25)),
    cmd(V, ZERO, '4 points inside: stopped', state=('StopCircle', STOP)),
]

BOUNDARY_CIRCLE = [
    clock(T),
    scan(ray(0.5)),
    cmd(V, V, 'circle r=0.5, point at exactly 0.5: outside'),
    scan(ray(f32_next(0.5, -1))),
    cmd(V, ZERO, 'one float32 step below 0.5: inside', state=('Circle1', STOP)),
    scan(ray(f32_next(0.5, 1))),
    cmd(V, V, 'one float32 step above 0.5: outside', state=NORMAL),
]

BOUNDARY_CIRCLE_07 = [
    clock(T),
    scan(ray(0.7)),
    cmd(V, ZERO, 'circle r=0.7, range 0.7: INSIDE, because float32(0.7) < 0.7',
        state=('Circle07', STOP)),
    scan(ray(f32_next(0.7, 1))),
    cmd(V, V, 'one float32 step above float32(0.7): outside', state=NORMAL),
]

BOUNDARY_CIRCLE_03 = [
    clock(T),
    scan(ray(0.3)),
    cmd(V, V, 'circle r=0.3, range 0.3: outside, because float32(0.3) > 0.3'),
    scan(ray(f32_next(0.3, -1))),
    cmd(V, ZERO, 'one float32 step below float32(0.3): inside', state=('Circle03', STOP)),
]

# Square with corners at (+-0.5, +-0.5). A point on the left or the top edge
# counts as inside, on the right or the bottom edge as outside.
BOUNDARY_POLYGON = [
    clock(T),
    scan(ray(0.5, 0.0)),
    cmd(V, V, 'point (0.5, 0) on the right edge: outside'),
    scan(ray(f32_next(0.5, -1), 0.0)),
    cmd(V, ZERO, 'one float32 step inside the right edge: inside', state=('Poly1', STOP)),
    scan(ray(0.5, PI)),
    cmd(V, ZERO, 'point (-0.5, -4e-8) on the left edge: inside'),
    scan(ray(f32_next(0.5, 1), PI)),
    cmd(V, V, 'one float32 step beyond the left edge: outside', state=NORMAL),
    scan(ray(0.5, PI / 2)),
    cmd(V, ZERO, 'point (-2e-8, 0.5) on the top edge: inside', state=('Poly1', STOP)),
    scan(ray(f32_next(0.5, 1), PI / 2)),
    cmd(V, V, 'one float32 step beyond the top edge: outside', state=NORMAL),
    scan(ray(0.5, -PI / 2)),
    cmd(V, V, 'point (-2e-8, -0.5) on the bottom edge: outside'),
    scan(ray(f32_next(0.5, -1), -PI / 2)),
    cmd(V, ZERO, 'one float32 step inside the bottom edge: inside', state=('Poly1', STOP)),
]

# Polygon coordinates are parsed as float32, so the edge of a "0.3" square is
# at float32(0.3), exactly where a range of "0.3" puts the point.
BOUNDARY_POLYGON_03 = [
    clock(T),
    scan(ray(0.3, PI)),
    cmd(V, ZERO, 'square +-0.3, point at range 0.3 on the left edge: inside (edge is float32)',
        state=('Poly03', STOP)),
    scan(ray(f32_next(0.3, 1), PI)),
    cmd(V, V, 'one float32 step beyond: outside', state=NORMAL),
    scan(ray(0.3, 0.0)),
    cmd(V, V, 'range 0.3 on the right edge: outside'),
    scan(ray(f32_next(0.3, -1), 0.0)),
    cmd(V, ZERO, 'one float32 step inside the right edge: inside', state=('Poly03', STOP)),
]

BOUNDARY_POLYGON_07 = [
    clock(T),
    scan(ray(0.7, 0.0)),
    cmd(V, V, 'square +-0.7, range 0.7 on the right edge: outside (a 0.7 circle says inside)'),
    scan(ray(0.7, PI)),
    cmd(V, ZERO, 'range 0.7 on the left edge: inside', state=('Poly07', STOP)),
]

# The four corners, reached exactly through scan frames offset from the base.
VERTEX_TF = [
    static_tf('base_footprint', 'v_tr', 0.0, 0.5),
    static_tf('base_footprint', 'v_br', 0.0, -0.5),
    static_tf('base_footprint', 'v_tl', -1.0, 0.5),
    static_tf('base_footprint', 'v_bl', -1.0, -0.5),
]
BOUNDARY_VERTICES = [
    clock(T),
    scan(ray(0.5, frame='v_tr')),
    cmd(V, V, 'corner (0.5, 0.5): outside'),
    scan(ray(0.5, frame='v_br')),
    cmd(V, V, 'corner (0.5, -0.5): outside'),
    scan(ray(0.5, frame='v_bl')),
    cmd(V, V, 'corner (-0.5, -0.5): outside'),
    scan(ray(0.5, frame='v_tl')),
    cmd(V, ZERO, 'corner (-0.5, 0.5): inside', state=('Poly1', STOP)),
]


def timeout_steps(base, timeout_ns=s(0.5)):
    """Source timeout `timeout_ns`; the age of a scan is clock minus header stamp."""
    return [
        clock(base), scan(CLEAR),
        clock(base + timeout_ns - 1),
        cmd(V, V, 'scan 1 ns younger than source_timeout: used'),
        clock(base + timeout_ns),
        cmd(V, V, 'scan exactly source_timeout old: still used'),
        clock(base + timeout_ns + 1),
        cmd(V, ZERO, 'scan 1 ns older than source_timeout: stop', state=INVALID),
        scan(CLEAR),
        cmd(V, V, 'fresh scan: passes again', state=NORMAL),
        # The age is taken from the header stamp, not from the arrival time.
        scan(dict(CLEAR, age=timeout_ns)),
        cmd(V, V, 'a scan that arrives exactly source_timeout old: used'),
        scan(dict(CLEAR, age=timeout_ns + 1)),
        cmd(V, ZERO, 'a scan that arrives 1 ns too old: stop at once', state=INVALID),
        scan(dict(CLEAR, stamp=base + s(10))),
        cmd(V, V, 'a scan stamped 10 s in the future: used', state=NORMAL),
        clock(base + s(10) + timeout_ns),
        cmd(V, V, '... and is used until source_timeout after its stamp'),
        clock(base + s(10) + timeout_ns + 1),
        cmd(V, ZERO, '... and not 1 ns later', state=INVALID),
        # The clock jumps backwards: the age is negative, the scan is used.
        clock(base - s(50)),
        cmd(V, V, 'after a backwards clock jump the old scan is used again', state=NORMAL),
    ]


# source_timeout 1.001: the double nearest to 1.001, times 1e9, is
# 1000999999.9999999. Truncated to whole nanoseconds that is 1000999999.
TIMEOUT_DECIMAL = [
    clock(T), scan(CLEAR),
    clock(T + 1_000_999_999),
    cmd(V, V, 'source_timeout 1.001, scan 1000999999 ns old: used'),
    clock(T + 1_001_000_000),
    cmd(V, ZERO, 'scan 1001000000 ns old: stop (1.001 s was truncated to 1000999999 ns)',
        state=INVALID),
]

TIMEOUT_ZERO = [
    clock(T),
    cmd(V, V, 'source_timeout 0 and no scan ever received: PASSES'),
    scan(dict(MID, age=s(50))),
    cmd(V, scaled(V, 0.5), 'a scan 50 s old on arrival is used', state=('SlowCircle', SLOWDOWN)),
    clock(T + s(1000)),
    cmd(V, scaled(V, 0.5), '... and still 1000 s later'),
]

TIMEOUT_NEGATIVE = [
    clock(T), scan(CLEAR),
    cmd(V, ZERO, 'source_timeout -1: a scan stamped now is too old', state=INVALID),
    scan(dict(CLEAR, age=-s(1))),
    cmd(V, V, 'a scan stamped exactly 1 s ahead is used', state=NORMAL),
    scan(dict(CLEAR, age=-s(1) + 1)),
    cmd(V, ZERO, 'a scan stamped 1 ns less than 1 s ahead is not', state=INVALID),
]

NO_SCAN = [
    clock(T),
    cmd(V, ZERO, 'no scan ever received: stop', state=INVALID),
    cmd((0.5, 0.25, -1.5, 0.75, -0.125, 1.0), ZERO, 'still stopped'),
    scan(CLEAR),
    cmd(V, V, 'first scan arrives: passes', state=NORMAL),
]

# Before the first /clock message the node's clock reads 0.
BEFORE_CLOCK = [
    cmd(V, ZERO, 'before any /clock message, no scan: stop', state=INVALID),
    scan(CLEAR),
    cmd(V, V, 'scan stamped 0 at clock 0: passes', state=NORMAL),
    scan(dict(CLEAR, stamp=s(0.5) + 1)),
    cmd(V, V, 'scan stamped in the future: passes'),
    clock(s(1)),
    cmd(V, V, 'first clock message: 1 s; scan stamped 0.5 s + 1 ns still young enough'),
    clock(s(1) + 2),
    cmd(V, ZERO, '2 ns later it is too old', state=INVALID),
]

# A LaserScan stamp may be negative on the wire (sec is a signed integer).
NEGATIVE_STAMP = [
    clock(T), scan(CLEAR),
    cmd(V, V, 'a command passes'),
    scan(dict(CLEAR, stamp=-1)),
    quiet('a scan stamped -1 ns arrives: nothing happens yet'),
    crash(V, 'the next command: the node ABORTS (uncaught "cannot store a negative time point")'),
]

# One zone, min_points 1: a ray counts if range_min <= range <= range_max.
INVALID_RANGES = [
    clock(T),
    scan(ray(NAN)),
    cmd(V, V, 'range NaN: ignored (the scan itself is valid, so no stop)'),
    scan(ray(INF)),
    cmd(V, V, 'range +inf: ignored'),
    scan(ray(-INF)),
    cmd(V, V, 'range -inf: ignored'),
    scan(ray(0.25, range_min=f32_next(0.25, 1))),
    cmd(V, V, 'range one float32 step below range_min: ignored'),
    scan(ray(0.25, range_min=0.25)),
    cmd(V, ZERO, 'range exactly range_min: counted', state=('Circle1', STOP)),
    scan(ray(0.25, range_max=f32_next(0.25, -1))),
    cmd(V, V, 'range one float32 step above range_max: ignored', state=NORMAL),
    scan(ray(0.25, range_max=0.25)),
    cmd(V, ZERO, 'range exactly range_max: counted', state=('Circle1', STOP)),
    scan(ray(0.0, range_min=0.25)),
    cmd(V, V, 'range 0 below range_min: ignored', state=NORMAL),
    scan(ray(0.0, range_min=0.0)),
    cmd(V, ZERO, 'range 0 with range_min 0: a point at the origin, counted',
        state=('Circle1', STOP)),
    scan(ray(-0.25, range_min=0.0)),
    cmd(V, V, 'negative range: ignored', state=NORMAL),
    scan(ray(-0.25, range_min=-1.0)),
    cmd(V, ZERO, 'negative range with a negative range_min: counted, point behind the sensor',
        state=('Circle1', STOP)),
    scan(ray(INF, range_max=INF)),
    cmd(V, V, 'range +inf with range_max +inf: accepted, but the point is in no circle',
        state=NORMAL),
    scan(ray(0.25, range_min=NAN)),
    cmd(V, V, 'range_min NaN: every ray ignored'),
    scan(ray(0.25, range_max=NAN)),
    cmd(V, V, 'range_max NaN: every ray ignored'),
    scan({'ranges': [NAN, INF, 0.25, 0.01, 20.0], 'angle_increment': 0.01}),
    cmd(V, ZERO, 'one valid ray among invalid ones: counted', state=('Circle1', STOP)),
    scan(ray(0.25, range_min=1.0, range_max=0.1)),
    cmd(V, V, 'range_min above range_max: every ray ignored', state=NORMAL),
]

# Same with the square: the angle advances over ignored rays too.
INVALID_RANGES_POLYGON = [
    clock(T),
    scan({'ranges': [NAN, 0.625], 'angle_min': 0.0, 'angle_increment': PI / 4}),
    cmd(V, ZERO, 'second ray at 45 degrees (after a NaN) lands in the corner: inside',
        state=('Poly1', STOP)),
    scan({'ranges': [0.625, NAN], 'angle_min': 0.0, 'angle_increment': PI / 4}),
    cmd(V, V, 'the same range in the first ray, straight ahead: outside', state=NORMAL),
    scan(ray(INF, range_max=INF)),
    cmd(V, V, 'accepted +inf at angle 0, point (inf, NaN): outside'),
    scan(ray(INF, PI / 4, range_max=INF)),
    cmd(V, V, 'accepted +inf at 45 degrees, point (inf, inf): outside'),
    scan(ray(INF, 3.0, range_max=INF)),
    cmd(V, V, 'accepted +inf at 3 rad, point (-inf, inf): outside'),
    scan(ray(INF, -3.0, range_max=INF)),
    cmd(V, V, 'accepted +inf at -3 rad, point (-inf, -inf): outside'),
]

# Only `ranges` decides how many rays there are; angle_max is never read.
LENGTH_MISMATCH = [
    clock(T),
    scan({'ranges': [5.0, 5.0, 5.0, 5.0, 0.25], 'angle_increment': 0.1, 'angle_max': 0.2}),
    cmd(V, ZERO, '5 ranges where the angles allow 3: the 5th is counted', state=('Circle1', STOP)),
    scan({'ranges': [5.0], 'angle_increment': 0.1, 'angle_max': 1.0}),
    cmd(V, V, '1 range where the angles allow 11: passes', state=NORMAL),
    scan({'ranges': [0.25], 'angle_increment': 0.1, 'angle_max': 1.0}),
    cmd(V, ZERO, '1 range (inside) where the angles allow 11: counted', state=('Circle1', STOP)),
    scan({'ranges': [], 'angle_increment': 0.1, 'angle_max': 1.0}),
    cmd(V, V, 'no ranges at all: a valid scan without points, passes', state=NORMAL),
    scan({'ranges': [0.25, 0.25], 'angle_min': 1.0, 'angle_increment': -0.1, 'angle_max': 0.0}),
    cmd(V, ZERO, 'negative angle_increment: counted', state=('Circle1', STOP)),
    scan({'ranges': [5.0, 0.25], 'angle_min': 0.0, 'angle_increment': NAN}),
    cmd(V, V, 'angle_increment NaN: the 2nd point is (NaN, NaN), in no zone', state=NORMAL),
    scan({'ranges': [0.25, 5.0], 'angle_min': 0.0, 'angle_increment': NAN}),
    cmd(V, ZERO, 'angle_increment NaN: the 1st ray is still at angle_min, counted',
        state=('Circle1', STOP)),
]

# The 5th ray lies in the corner of the square only if its angle is
# angle_min + 4 * angle_increment, beyond the stated angle_max.
LENGTH_MISMATCH_POLYGON = [
    clock(T),
    scan({'ranges': [5.0, 5.0, 5.0, 5.0, 0.625], 'angle_increment': PI / 16, 'angle_max': PI / 8}),
    cmd(V, ZERO, 'the 5th of 5 ranges is placed at angle_min + 4 * angle_increment',
        state=('Poly1', STOP)),
]

# A 361-ray scan from -3.14159 in steps of 0.0174533 (one degree). Adding the
# increment 360 times in float32 gives 3.1415865 for the last ray, just short
# of pi, so its point lies above the x axis. angle_min + 360 * increment is
# 3.1415977, just beyond pi, which would put the point below the x axis.
LIDAR = {'ranges': [NAN] * 360 + [0.5], 'angle_min': -3.14159, 'angle_increment': 0.0174533}
ANGLE_ACCUMULATION = [
    clock(T),
    scan(LIDAR),
    cmd(V, ZERO, 'last of 361 rays: the angle is the float32 running sum, point at y = +3e-6',
        state=('PolyUpper', STOP)),
    scan(ray(0.5, 3.1415977)),
    cmd(V, V, 'a single ray at angle_min + 360 * increment: point at y = -3e-6, outside',
        state=NORMAL),
    scan(ray(0.5, 3.1415865)),
    cmd(V, ZERO, 'a single ray at the running sum 3.1415865: inside', state=('PolyUpper', STOP)),
]

# The node publishes only in answer to a command.
CEASING = [
    clock(T), scan(CLEAR),
    cmd(V, V, 'a command passes'),
    clock(T + s(0.25)), quiet('no command for 0.25 s: nothing published'),
    clock(T + s(1)), quiet('scan now too old, no command: nothing published'),
    clock(T + s(5)), quiet('5 s without a command: nothing published'),
    scan(NEAR), quiet('obstacle appears in the stop zone, no command: nothing published'),
    clock(T + s(600)), quiet('10 min without a command: nothing published'),
    scan(NEAR),
    cmd(V, ZERO, 'next command, obstacle still there: stop', state=('StopCircle', STOP)),
]

# Zero output is published for stop_pub_timeout (1 s) after the output first
# became zero, and then not at all, until an output is non-zero again.
STOP_PUB = [
    clock(T), scan(NEAR),
    cmd(V, ZERO, 'stop', state=('StopCircle', STOP)),
    clock(T + s(0.5)), scan(NEAR),
    cmd(V, ZERO, '0.5 s into the stop: zero still published'),
    clock(T + s(1)), scan(NEAR),
    cmd(V, ZERO, 'exactly stop_pub_timeout into the stop: zero still published'),
    clock(T + s(1) + 1), scan(NEAR),
    cmd(V, None, '1 ns later: NOTHING published'),
    clock(T + s(2)), scan(NEAR),
    cmd(V, None, 'still nothing'),
    clock(T + s(3)),
    cmd(V, None, 'scan times out during the silence: state changes, still nothing published',
        state=INVALID),
    scan(CLEAR),
    cmd(V, V, 'obstacle gone: passes', state=NORMAL),
    scan(NEAR),
    cmd(V, ZERO, 'a new stop is published again', state=('StopCircle', STOP)),
    clock(T + s(4)), scan(NEAR),
    cmd(V, ZERO, '... for stop_pub_timeout from its start'),
    clock(T + s(4) + 1), scan(NEAR),
    cmd(V, None, '... and not 1 ns longer'),
]

# The same rule applies to zero commands that merely pass through.
ZERO_INPUT = [
    clock(T), scan(CLEAR),
    cmd(ZERO, ZERO, 'first command ever is zero: forwarded'),
    clock(T + s(1)), scan(CLEAR),
    cmd(ZERO, ZERO, 'zero command 1 s later: forwarded'),
    clock(T + s(1) + 1), scan(CLEAR),
    cmd(ZERO, None, 'zero command 1 ns after stop_pub_timeout: NOT forwarded'),
    cmd((0.0, 0.0, 0.5, 0.25, 0.125, 0.0), None,
        'only linear.z and angular.x/y non-zero: counts as zero, not forwarded'),
    cmd((-0.0, -0.0, -0.0), None, 'negative zeros count as zero'),
    cmd((0.0, 0.0, 1e-300), (0.0, 0.0, 1e-300), 'any non-zero yaw rate is forwarded'),
    cmd((-0.0, 0.0, -0.0), (-0.0, 0.0, -0.0), 'negative zero is forwarded as negative zero'),
    clock(T + s(2) + 1),
    scan(CLEAR),
    cmd(ZERO, ZERO, 'zero forwarded for 1 s after the output became zero'),
    clock(T + s(2) + 2), scan(CLEAR),
    cmd(ZERO, None, '... counted from the first zero output (the negative zero)'),
]

# Slowdown zone triggered throughout (ratio 0.5).
SIGNS = [
    clock(T), scan(MID),
    cmd((-0.5, -0.25, -1.0), (-0.25, -0.125, -0.5), 'all negative: halved, signs kept',
        state=('SlowCircle', SLOWDOWN)),
    cmd((0.5, 0.0, 0.0), (0.25, 0.0, 0.0), 'x only'),
    cmd((-0.5, 0.0, 0.0), (-0.25, 0.0, 0.0), '-x only'),
    cmd((0.0, 0.5, 0.0), (0.0, 0.25, 0.0), 'y only'),
    cmd((0.0, -0.5, 0.0), (0.0, -0.25, 0.0), '-y only'),
    cmd((0.0, 0.0, 1.0), (0.0, 0.0, 0.5), 'yaw only'),
    cmd((0.0, 0.0, -1.0), (0.0, 0.0, -0.5), '-yaw only'),
    cmd((-0.5, -0.0, 0.0), (-0.25, -0.0, 0.0), 'a negative-zero component stays negative zero'),
    cmd(ZERO, ZERO, 'zero command in a triggered slowdown zone: zero out, state back to normal',
        state=NORMAL),
    cmd((-0.0, 0.0, 0.0), (-0.0, 0.0, 0.0), 'negative zero passes as negative zero'),
    cmd((0.5, 0.0, 0.0), (0.25, 0.0, 0.0), 'moving again: slowed', state=('SlowCircle', SLOWDOWN)),
    # The slowed velocity is applied only if its squared norm is smaller, and
    # the squares of both 1e-200 and 5e-201 underflow to 0.
    cmd((1e-200, 0.0, 0.0), (1e-200, 0.0, 0.0),
        'velocity whose square underflows: NOT slowed, state back to normal', state=NORMAL),
    cmd((1e-150, 0.0, 0.0), (5e-151, 0.0, 0.0), 'velocity 1e-150: slowed',
        state=('SlowCircle', SLOWDOWN)),
]

# Stop zone triggered throughout.
SIGNS_STOP = [
    clock(T), scan(NEAR),
    cmd((-0.5, -0.25, -1.0), ZERO, 'negative command: +0 on all axes', state=('StopCircle', STOP)),
    cmd((-0.0, -0.0, -0.0), ZERO, 'negative-zero command: +0 on all axes'),
    cmd(ZERO, ZERO, 'zero command'),
]

SIGNS_CLEAR = [
    clock(T), scan(CLEAR),
    cmd((-0.5, 0.0, 0.0), (-0.5, 0.0, 0.0), '-x passes'),
    cmd((0.0, -0.25, 0.0), (0.0, -0.25, 0.0), '-y passes'),
    cmd((0.0, 0.0, -1.0), (0.0, 0.0, -1.0), '-yaw passes'),
    cmd((-0.5, -0.25, -1.0), (-0.5, -0.25, -1.0), 'all negative passes'),
    cmd((-0.0, -0.0, -0.0), (-0.0, -0.0, -0.0), 'negative zeros pass as negative zeros'),
]

NON_FINITE_CMD = [
    clock(T), scan(CLEAR),
    cmd(V, V, 'a command passes'),
    cmd((NAN, 0.0, 0.0), None, 'linear.x NaN: nothing published (not a stop)'),
    cmd((0.5, INF, 0.0), None, 'linear.y +inf: nothing published'),
    cmd((0.5, 0.0, -INF), None, 'angular.z -inf: nothing published'),
    cmd((0.5, 0.0, NAN, 0.0, 0.0, 0.0), None, 'linear.z NaN (a component never forwarded): '
        'nothing published'),
    cmd((0.5, 0.0, 0.0, INF, 0.0, 0.0), None, 'angular.x +inf: nothing published'),
    cmd((0.5, 0.0, 0.0, 0.0, NAN, 0.0), None, 'angular.y NaN: nothing published'),
    cmd(V, V, 'the next finite command passes'),
    scan(NEAR),
    cmd((NAN, 0.0, 0.0), None, 'NaN with an obstacle in the stop zone: nothing published either'),
]

# Scan frame 0.25 m ahead of the base (static transform), min_points 1, r = 0.5.
OFFSET_TF = [static_tf('base_footprint', 'base_scan', 0.25, 0.0)]
ODOM_TF = [static_tf('odom', 'base_footprint')]
OFFSET = [
    clock(T),
    scan(ray(0.25)),
    cmd(V, V, 'scan frame 0.25 ahead, range 0.25: point (0.5, 0), on the circle: outside'),
    scan(ray(f32_next(0.25, -1))),
    cmd(V, ZERO, 'one float32 step closer: inside', state=('Circle1', STOP)),
    scan(ray(0.625, PI)),
    cmd(V, ZERO, 'range 0.625 backwards: point (-0.375, 0): inside'),
    scan(ray(0.875, PI)),
    cmd(V, V, 'range 0.875 backwards: point (-0.625, 0): outside', state=NORMAL),
]

# Scan frame 0.25 ahead and turned by half a turn.
ROT_TF = [static_tf('base_footprint', 'base_scan', 0.25, 0.0, yaw_half_turns=1)]
ROTATED = [
    clock(T),
    scan(ray(0.75)),
    cmd(V, V, 'scanner facing backwards, range 0.75: point (-0.5, 0), on the circle: outside'),
    scan(ray(f32_next(0.75, -1))),
    cmd(V, ZERO, 'one float32 step closer: inside', state=('Circle1', STOP)),
    scan(ray(0.875, PI)),
    cmd(V, V, 'range 0.875 at pi: point (1.125, 0): outside', state=NORMAL),
]

NO_TRANSFORM = [
    clock(T), scan(dict(CLEAR, frame='nowhere')),
    cmd(V, ZERO, 'no transform from the scan frame to the base: stop', state=INVALID),
    scan(CLEAR),
    cmd(V, V, 'a scan in the base frame needs no transform: passes', state=NORMAL),
]

# With source_timeout 0 an unusable scan does not stop the robot either.
TIMEOUT_ZERO_NO_TRANSFORM = [
    clock(T), scan(dict(NEAR, frame='nowhere')),
    cmd(V, V, 'source_timeout 0, obstacle scan in a frame without transform: PASSES'),
    scan(NEAR),
    cmd(V, ZERO, 'the same scan in the base frame: stop', state=('StopCircle', STOP)),
]

NO_ODOM = [
    clock(T), scan(CLEAR),
    cmd(V, ZERO, 'base_shift_correction on, scan in the base frame, no odom transform: stop',
        state=INVALID),
]

# transform_tolerance 0.25 and a missing transform: the node blocks inside
# the command callback until its clock has advanced by the tolerance.
TF_WAIT = [
    clock(T), scan(dict(CLEAR, frame='nowhere')),
    cmd(V, None, 'missing transform, tolerance 0.25: no answer while the clock stands still'),
    clock(T + s(0.125)), quiet('clock + 0.125: still no answer'),
    late(T + s(0.25), ZERO, 'clock + 0.25: the stop comes out', state=INVALID),
]

STAMPED = [
    clock(T), scan(CLEAR),
    cmd((0.5, 0.25, -1.5, 0.75, -0.125, 1.0), V, 'TwistStamped in, TwistStamped out, header copied'),
    scan(MID),
    cmd(V, scaled(V, 0.5), 'slowed, header copied', state=('SlowPoly', SLOWDOWN)),
    scan(NEAR),
    cmd(V, ZERO, 'stopped, header copied', state=('StopCircle', STOP)),
]


def ratio_steps(out, state, label):
    return [clock(T), scan(MID), cmd(V, out, label, state=state)]


def P(*names):
    return 'polygons:=[' + ','.join(names) + ']'


# name -> dict(params, steps[, stamped][, tf][, refused])
#   params    extra `-p` arguments on top of zones.yaml
#   tf        static transforms to publish (the scenario is skipped with --no-tf)
#   stamped   TwistStamped on both sides (enable_stamped_cmd_vel)
#   refused   the node must reject this configuration
SCENARIOS = {
    'pass-through': dict(params=[P('SlowPoly', 'StopCircle')], steps=PASS_THROUGH),
    'stop-circle': dict(params=[P('StopCircle')], steps=STOP_CIRCLE),
    'stop-polygon': dict(params=[P('StopPoly')], steps=STOP_POLYGON),
    'slowdown': dict(params=[P('SlowPoly')], steps=SLOWDOWN_STEPS),
    'slowdown-ratio': dict(params=[P('SlowWide')], steps=SLOWDOWN_RATIO),
    'both': dict(params=[P('SlowPoly', 'StopCircle')], steps=BOTH),
    'both-stop-first': dict(params=[P('StopCircle', 'SlowPoly')], steps=BOTH),
    'two-slowdowns': dict(params=[P('SlowPoly', 'SlowWide')], steps=TWO_SLOWDOWNS),
    'two-slowdowns-reversed': dict(params=[P('SlowWide', 'SlowPoly')], steps=TWO_SLOWDOWNS),
    'min-points': dict(params=[P('StopCircle')], steps=MIN_POINTS),
    'boundary-circle': dict(params=[P('Circle1')], steps=BOUNDARY_CIRCLE),
    'boundary-circle-0.7': dict(params=[P('Circle07')], steps=BOUNDARY_CIRCLE_07),
    'boundary-circle-0.3': dict(params=[P('Circle03')], steps=BOUNDARY_CIRCLE_03),
    'boundary-polygon': dict(params=[P('Poly1')], steps=BOUNDARY_POLYGON),
    'boundary-polygon-0.3': dict(params=[P('Poly03')], steps=BOUNDARY_POLYGON_03),
    'boundary-polygon-0.7': dict(params=[P('Poly07')], steps=BOUNDARY_POLYGON_07),
    'boundary-vertices': dict(params=[P('Poly1')], steps=BOUNDARY_VERTICES, tf=VERTEX_TF),
    'timeout': dict(params=[P('StopCircle')], steps=timeout_steps(T)),
    'timeout-unix-clock': dict(params=[P('StopCircle')], steps=timeout_steps(s(1_800_000_000))),
    'timeout-decimal': dict(params=[P('StopCircle'), 'source_timeout:=1.001'], steps=TIMEOUT_DECIMAL),
    'timeout-zero': dict(params=[P('SlowCircle'), 'source_timeout:=0.0'], steps=TIMEOUT_ZERO),
    'timeout-negative': dict(params=[P('StopCircle'), 'source_timeout:=-1.0'],
                             steps=TIMEOUT_NEGATIVE),
    'no-scan': dict(params=[P('StopCircle')], steps=NO_SCAN),
    'before-clock': dict(params=[P('StopCircle')], steps=BEFORE_CLOCK),
    'negative-stamp': dict(params=[P('StopCircle')], steps=NEGATIVE_STAMP),
    'invalid-ranges': dict(params=[P('Circle1')], steps=INVALID_RANGES),
    'invalid-ranges-polygon': dict(params=[P('Poly1')], steps=INVALID_RANGES_POLYGON),
    'length-mismatch': dict(params=[P('Circle1')], steps=LENGTH_MISMATCH),
    'length-mismatch-polygon': dict(params=[P('Poly1')], steps=LENGTH_MISMATCH_POLYGON),
    'angle-accumulation': dict(params=[P('PolyUpper')], steps=ANGLE_ACCUMULATION),
    'commands-cease': dict(params=[P('StopCircle', 'SlowCircle')], steps=CEASING),
    'stop-publication': dict(params=[P('StopCircle')], steps=STOP_PUB),
    'zero-input': dict(params=[P('StopCircle')], steps=ZERO_INPUT),
    'signs-slowdown': dict(params=[P('SlowCircle')], steps=SIGNS),
    'signs-stop': dict(params=[P('StopCircle')], steps=SIGNS_STOP),
    'signs-pass': dict(params=[P('StopCircle')], steps=SIGNS_CLEAR),
    'non-finite-command': dict(params=[P('StopCircle')], steps=NON_FINITE_CMD),
    'stamped': dict(params=[P('SlowPoly', 'StopCircle'), 'enable_stamped_cmd_vel:=true'],
                    steps=STAMPED, stamped=True),
    'ratio-negative': dict(params=[P('SlowNegative')], steps=ratio_steps(
        scaled(V, -0.5), ('SlowNegative', SLOWDOWN),
        'slowdown_ratio -0.5: the command is REVERSED at half speed')),
    'ratio-minus-one': dict(params=[P('SlowMinusOne')], steps=ratio_steps(
        V, None, 'slowdown_ratio -1: not smaller in magnitude, command passes')),
    'ratio-one': dict(params=[P('SlowOne')], steps=ratio_steps(
        V, None, 'slowdown_ratio 1: command passes, no state change')),
    'ratio-two': dict(params=[P('SlowTwo')], steps=ratio_steps(
        V, None, 'slowdown_ratio 2: command passes (never sped up)')),
    'ratio-zero': dict(params=[P('SlowZero')], steps=ratio_steps(
        ZERO, ('SlowZero', SLOWDOWN), 'slowdown_ratio 0: zero, reported as slowdown')),
    'min-points-zero': dict(params=[P('StopMin0')], steps=[
        clock(T), scan(CLEAR),
        cmd(V, ZERO, 'min_points 0: stops with no point in the zone', state=('StopMin0', STOP)),
        scan({'ranges': []}),
        cmd(V, ZERO, 'min_points 0: stops on an empty scan')]),
    'min-points-negative': dict(params=[P('StopMinNeg')], steps=[
        clock(T), scan(CLEAR),
        cmd(V, ZERO, 'min_points -1: stops with no point in the zone',
            state=('StopMinNeg', STOP))]),
    'action-none': dict(params=[P('NoAction')], steps=[
        clock(T), scan(ray(0.25)),
        cmd(V, V, 'action_type none: a point in the zone changes nothing')]),
    'triangle': dict(params=[P('Triangle')], steps=[], refused=True),
    'offset': dict(params=[P('Circle1')], steps=OFFSET, tf=OFFSET_TF, scan_frame='base_scan'),
    'offset-shift-correction': dict(
        params=[P('Circle1'), 'base_shift_correction:=true'], steps=OFFSET,
        tf=OFFSET_TF + ODOM_TF, scan_frame='base_scan'),
    'shift-correction-same-frame': dict(
        params=[P('Circle1'), 'base_shift_correction:=true'], steps=BOUNDARY_CIRCLE, tf=ODOM_TF),
    'rotated': dict(params=[P('Circle1')], steps=ROTATED, tf=ROT_TF, scan_frame='base_scan'),
    'no-transform': dict(params=[P('StopCircle')], steps=NO_TRANSFORM, tf=ODOM_TF),
    'timeout-zero-no-transform': dict(params=[P('StopCircle'), 'source_timeout:=0.0'],
                                      steps=TIMEOUT_ZERO_NO_TRANSFORM, tf=ODOM_TF),
    'no-odom': dict(params=[P('StopCircle'), 'base_shift_correction:=true'], steps=NO_ODOM,
                    tf=OFFSET_TF),
    'transform-wait': dict(params=[P('StopCircle'), 'transform_tolerance:=0.25'], steps=TF_WAIT,
                           tf=ODOM_TF),
}


def expected6(out):
    return None if out is None else (out[0], out[1], 0.0, 0.0, 0.0, out[2])


def describe(d, outs, states):
    parts = [fmt(twist6(d.twist_of(m))) for m in outs] or ['nothing']
    parts += [f'state({m.polygon_name!r}, {m.action_type})' for m in states]
    return ' + '.join(parts)


def check_answer(d, msg, outs, states, out, state, check_state):
    """Why the answer to a command is not the expected one, or None."""
    want = expected6(out)
    if want is None:
        if outs:
            return 'expected nothing'
    else:
        if len(outs) != 1:
            return f'expected {fmt(want)}'
        if not same_floats(twist6(d.twist_of(outs[0])), want):
            return f'expected {fmt(want)}'
        if d.stamped and outs[0].header != msg.header:
            return 'header not copied'
    if check_state:
        got = [(m.polygon_name, m.action_type) for m in states]
        if got != ([] if state is None else [state]):
            return f'expected state {state}'
    return None


def override_for(expect, label):
    """The profile's entry for the check with this label: (out, tag, reason) or None."""
    hits = [v for k, v in expect.get('diffs', {}).items() if k in label]
    if len(hits) > 1:
        raise SystemExit(f'profile: more than one expected difference matches {label!r}')
    return hits[0] if hits else None


def run_scenario(exe, name, opts):
    """Returns (number of checks, number of failures, tags of expected differences),
    or None if skipped."""
    sc = opts.scenarios[name]
    expect = opts.expect.get(name, {})
    tags = []
    tf = sc.get('tf', [])
    params = list(sc['params'])
    if tf and not opts.profile_tf:
        # A node without transforms: the profile must say, for every scenario
        # that publishes one, what takes its place.
        if 'skip' in expect:
            tag, why = expect['skip']
            print(f'--- {name}: not run [{tag}: {why}]')
            return None
        if not ({'refused', 'params', 'no_tf'} & set(expect)):
            raise SystemExit(f'profile: {name} needs a transform; say what replaces it')
        tf = []
    elif tf and opts.no_tf:
        print(f'--- {name}: skipped (needs a transform)')
        return None
    if 'params' in expect:
        tag, why, extra = expect['params']
        params += extra
        tags.append(tag)
    stamped = sc.get('stamped', False)
    side = Side(exe, 'collision_monitor', 'cmd_vel_out',
                'cm_state' if opts.state_topic else None)
    log = os.path.join(opts.logs, f'{name}.log') if opts.logs else None
    side.start(os.path.join(HERE, 'zones.yaml'), params, log)
    d = Driver('scenario_driver', [side], stamped, scan_frame=sc.get('scan_frame', 'base_footprint'))
    check_state = not opts.ignore_state
    lines = []  # (ok, text)
    used = set()

    def note(ok, text):
        lines.append((ok, text))

    def expected(label, out):
        """The expected output of a check, and a marker for the line."""
        ov = override_for(expect, label)
        if ov is None:
            return out, ''
        used.add(id(ov))
        tags.append(ov[1])
        return ov[0], f'[{ov[1]}: {ov[2]}] '

    try:
        d.publish_tf(tf)
        status = d.bring_up(side, opts.lifecycle)
        must_refuse = sc.get('refused') or 'refused' in expect
        if must_refuse:
            mark = ''
            if 'refused' in expect:
                tags.append(expect['refused'][0])
                mark = '[%s: %s] ' % expect['refused']
            note(status == 'refused', f'{mark}configuration is refused (node is {status})')
        elif status != 'active':
            note(False, 'node refused its configuration')
        else:
            d.wait_connected(tf_listeners=1 if tf and side.lifecycle else 0)
            pub_type = d.get_publishers_info_by_topic('cmd_vel_out')[0].topic_type
            sub_types = {i.topic_type for i in d.get_subscriptions_info_by_topic('cmd_vel_in')}
            want = 'geometry_msgs/msg/' + ('TwistStamped' if stamped else 'Twist')
            note(pub_type == want and sub_types == {want},
                 f'command topics are {want} (in: {sorted(sub_types)}, out: {pub_type})')
            ident = 0
            last_cmd = None
            crashed = False
            for step in sc['steps']:
                before = (len(side.out), len(side.states))
                if step[0] == 'crash' and override_for(expect, step[2]) is not None:
                    # The profile says this node survives: an ordinary command.
                    step = ('cmd', step[1], None, None, step[2])
                if step[0] == 'clock':
                    d.set_clock(step[1], opts.settle)
                elif step[0] == 'scan':
                    d.send_scan(step[1], opts.settle)
                elif step[0] == 'quiet':
                    d.spin_for(opts.settle)
                    note(True, f'nothing        {step[1]}')
                elif step[0] == 'crash':
                    ident += 1
                    (outs, states), = d.send_cmd(d.make_cmd(step[1], ident), opts.settle)
                    d.spin_until(lambda: side.proc.poll() is not None, 60.0)
                    code = side.proc.poll()
                    crashed = True
                    note(code is not None and code != 0 and not outs,
                         f'{describe(d, outs, states)}, exit code {code}   {step[2]}')
                    break
                elif step[0] == 'late':
                    _, t_ns, out, state, label = step
                    out, mark = expected(label, out)
                    d.set_clock(t_ns, 0.0)
                    d.spin_until(lambda: len(side.out) > before[0], 2.0)
                    d.spin_for(opts.settle)
                    outs, states = side.out[before[0]:], side.states[before[1]:]
                    why = check_answer(d, last_cmd, outs, states, out, state, check_state)
                    note(why is None, f'{mark}{describe(d, outs, states)}   {label}'
                         + (f'   [{why}]' if why else ''))
                    continue
                else:
                    _, vel, out, state, label = step
                    out, mark = expected(label, out)
                    ident += 1
                    msg = last_cmd = d.make_cmd(vel, ident)
                    want_n = 0 if out is None else 1
                    (outs, states), = d.send_cmd(
                        msg, opts.settle,
                        wait_for=(lambda i, o, st: len(o) >= want_n) if want_n else None)
                    why = check_answer(d, msg, outs, states, out, state, check_state)
                    note(why is None, f'{mark}{describe(d, outs, states)}   {label}'
                         + (f'   [{why}]' if why else ''))
                    continue
                # Clock, scan and quiet steps must not make the node publish.
                if len(side.out) != before[0] or (
                        check_state and len(side.states) != before[1]):
                    note(False, f'node published without a command after {step[0]} step: '
                         + describe(d, side.out[before[0]:], side.states[before[1]:]))
            d.spin_for(opts.settle)
            if side.proc.poll() is not None and not crashed:
                note(False, f'node exited with code {side.proc.returncode}')
            for key, ov in expect.get('diffs', {}).items():
                if id(ov) not in used:
                    note(False, f'profile: no check matched the expected difference {key!r}')
    except RuntimeError as e:
        note(False, str(e))
    finally:
        side.stop()
        d.destroy_node()

    print(f'--- {name}  ({" ".join(params)})')
    for ok, text in lines:
        print(f"{'ok  ' if ok else 'FAIL'}  {text}")
    return len(lines), sum(not ok for ok, _ in lines), tags


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node', nargs='?', default=UPSTREAM)
    ap.add_argument('--lifecycle', choices=['auto', 'yes', 'no'], default='auto')
    ap.add_argument('--no-tf', action='store_true')
    ap.add_argument('--ignore-state', action='store_true')
    ap.add_argument('--only', default='')
    ap.add_argument('--skip', default='')
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--settle', type=float, default=0.2)
    ap.add_argument('--logs', default='')
    ap.add_argument('--list', action='store_true')
    ap.add_argument('--profile', default='')
    ap.add_argument('--own', action='store_true')
    opts = ap.parse_args()
    opts.scenarios, opts.expect = dict(SCENARIOS), {}
    opts.state_topic = opts.profile_tf = True
    if opts.profile:
        import importlib
        profile = importlib.import_module(opts.profile)
        opts.scenarios.update(profile.SCENARIOS)
        opts.expect = profile.EXPECT
        unknown = [n for n in opts.expect if n not in opts.scenarios]
        if unknown:
            sys.exit(f'profile: unknown scenario(s): {unknown}')
        opts.state_topic, opts.profile_tf = profile.STATE_TOPIC, profile.TF
        if not opts.state_topic:
            opts.ignore_state = True
    if opts.list:
        print('\n'.join(opts.scenarios))
        return
    names = [n for n in opts.only.split(',') if n] or list(
        profile.SCENARIOS if opts.own and opts.profile else opts.scenarios)
    skip = [n for n in opts.skip.split(',') if n]
    unknown = [n for n in names + skip if n not in opts.scenarios]
    if unknown:
        sys.exit(f'unknown scenario(s): {unknown}')
    names = [n for n in names if n not in skip]
    if opts.logs:
        os.makedirs(opts.logs, exist_ok=True)
    exe = resolve(opts.node)
    print(f'node under test: {exe}')
    rclpy.init()
    checks = bad = skipped = 0
    failed = []
    tags = {}
    try:
        for rep in range(opts.repeat):
            if opts.repeat > 1:
                print(f'=== run {rep + 1} of {opts.repeat}')
            for name in names:
                result = run_scenario(exe, name, opts)
                if result is None:
                    skipped += 1
                    continue
                checks += result[0]
                bad += result[1]
                if result[1]:
                    failed.append(name)
                for tag in result[2]:
                    tags[tag] = tags.get(tag, 0) + 1
    finally:
        rclpy.shutdown()
    print(f'{bad} failure(s) in {checks} checks, {len(names) * opts.repeat - skipped} scenario runs'
          + (f', {skipped} skipped' if skipped else '')
          + (f'; failed: {sorted(set(failed))}' if failed else ''))
    if opts.profile:
        print('expected differences from upstream, by number in the SPEC: '
              + (', '.join(f'{t}: {n}' for t, n in sorted(tags.items())) or 'none'))
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
