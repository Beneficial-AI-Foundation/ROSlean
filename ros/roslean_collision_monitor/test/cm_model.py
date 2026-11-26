"""A Python model of the observed behaviour of upstream nav2_collision_monitor.

Covers the subset under test: one laser scan, stop and slowdown zones,
circles and fixed polygons, min_points, the source timeout and the
stop-publication timeout. It is an executable summary of
docs/collision_monitor/UPSTREAM_BEHAVIOUR.md and has two users:
random_diff.py compares every answer of the nodes under test with it, and
reference_node.py wraps it into a plain ROS node.

Times are integer nanoseconds. Scan geometry follows upstream's precision
(float32 angles, summed up ray by ray, float32 products) when numpy is
available, and falls back to doubles otherwise; the difference shows only
within about 1e-5 m of a zone boundary.
"""

import math
import struct

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

DO_NOTHING, STOP, SLOWDOWN = 0, 1, 2
INVALID_SOURCE = 'invalid source'


def f32(x):
    return struct.unpack('f', struct.pack('f', x))[0]


def to_ns(seconds):
    """Seconds parameter to nanoseconds, truncating as upstream does."""
    return int(seconds * 1e9)


class Zone:
    """A circle (radius) or a polygon (list of (x, y)), with its action."""

    def __init__(self, name, kind, geometry, action, ratio=0.5, min_points=4):
        self.name, self.kind, self.action = name, kind, action
        self.ratio, self.min_points = ratio, min_points
        if kind == 'circle':
            self.radius = float(geometry)           # a double
        else:
            self.poly = [(f32(x), f32(y)) for x, y in geometry]   # float32 vertices

    def inside(self, p):
        x, y = p
        if self.kind == 'circle':
            return x * x + y * y < self.radius * self.radius
        # Ray crossing; points on a left or top edge come out as inside.
        res = False
        i = len(self.poly) - 1
        for j in range(len(self.poly)):
            (xi, yi), (xj, yj) = self.poly[i], self.poly[j]
            if (y <= yi) == (y > yj):
                # yj == yi only gets here if y is NaN; C++ then compares a NaN.
                if yj != yi and xi + (y - yi) * (xj - xi) / (yj - yi) > x:
                    res = not res
            i = j
        return res


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, tf=(0.0, 0.0, False)):
    """Points of a scan in the base frame; rays outside [range_min, range_max] are dropped.

    tf = (x, y, half_turn): the pose of the scan frame in the base frame.
    """
    tx, ty, flip = tf
    pts = []
    if np is not None:
        with np.errstate(all='ignore'):
            angle = np.float32(angle_min)
            inc = np.float32(angle_increment)
            lo, hi = np.float32(range_min), np.float32(range_max)
            for r in ranges:
                r = np.float32(r)
                if r >= lo and r <= hi:  # False for NaN
                    x, y = float(r * np.cos(angle)), float(r * np.sin(angle))
                    if flip:
                        x, y = -x, -y
                    pts.append((x + tx, y + ty))
                angle = np.float32(angle + inc)
        return pts
    lo, hi = f32(range_min), f32(range_max)
    for i, r in enumerate(ranges):
        r = f32(r)
        if r >= lo and r <= hi:
            a = f32(angle_min) + i * f32(angle_increment)
            x, y = r * math.cos(a), r * math.sin(a)
            if flip:
                x, y = -x, -y
            pts.append((x + tx, y + ty))
    return pts


class Model:
    """State machine: feed it clock values, scans and commands in order."""

    def __init__(self, zones, source_timeout, stop_pub_timeout):
        self.zones = zones
        self.source_timeout = to_ns(source_timeout)
        self.stop_pub_timeout = to_ns(stop_pub_timeout)
        self.now = 0
        self.scan = None          # (stamp in ns, points), points None if unusable
        self.prev_name = ''
        self.prev_zero = False    # was the previous output zero?
        self.stop_stamp = 0

    def clock(self, t_ns):
        self.now = t_ns

    def scan_in(self, stamp_ns, points):
        """The latest scan received replaces the previous one, whatever its stamp."""
        self.scan = (stamp_ns, points)

    def cmd(self, vel):
        """vel: (vx, vy, wz) or all six components.

        Returns (velocity, state): velocity is the six output components or
        None if nothing is published; state is (zone name, action type) or
        None if no state message is published.
        """
        if len(vel) == 3:
            vel = (vel[0], vel[1], 0.0, 0.0, 0.0, vel[2])
        if not all(math.isfinite(c) for c in vel):
            return None, None                       # ignored altogether
        v = (vel[0], vel[1], vel[5])
        req, name, action = v, '', DO_NOTHING
        points = []
        usable = (self.scan is not None and self.scan[1] is not None and (
            self.source_timeout == 0 or self.now - self.scan[0] <= self.source_timeout))
        if usable:
            points = self.scan[1]
        elif self.source_timeout != 0:
            req, name, action = (0.0, 0.0, 0.0), INVALID_SOURCE, STOP
        if action != STOP:
            for z in self.zones:
                if z.action not in ('stop', 'slowdown'):
                    continue
                if sum(z.inside(p) for p in points) < z.min_points:
                    continue
                if z.action == 'stop':
                    req, name, action = (0.0, 0.0, 0.0), z.name, STOP
                    break
                slow = tuple(c * z.ratio for c in v)
                if sum(c * c for c in slow) < sum(c * c for c in req):
                    req, name, action = slow, z.name, SLOWDOWN
        state = (name, action) if name != self.prev_name else None
        zero = all(c == 0.0 for c in req)
        publish = True
        if zero:
            if not self.prev_zero:
                self.stop_stamp = self.now          # the output just became zero
            elif self.now - self.stop_stamp > self.stop_pub_timeout:
                publish = False
        self.prev_name, self.prev_zero = name, zero
        return ((req[0], req[1], 0.0, 0.0, 0.0, req[2]) if publish else None), state
