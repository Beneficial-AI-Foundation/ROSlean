"""The bounds within which the ROSlean node may differ from upstream at a zone boundary.

Used by `random_diff.py --roslean`. The ROSlean core tests points in integer
millimetres against zones that are slightly larger than configured
(docs/collision_monitor/SPEC.md, D3 and D4, "Safe direction"). So the two
nodes may answer the same command differently when a point of the scan lies
within a few millimetres of a boundary, and a bit-for-bit comparison would
have to look away from boundaries. This module states instead what the SPEC
promises, and checks exactly that, with boundary cases kept:

  S  (safe direction) Every point that upstream counts as inside a zone is
     counted by ROSlean. Checked on the outputs alone, with no tolerance:
     what ROSlean demands is never less restrictive than what upstream
     demands for the same command (`at_least_as_restrictive`).

  G  (growth) ROSlean counts no point that lies farther outside the zone
     upstream uses than 3.5 mm (circle) or 4.4 mm (polygon). Checked by
     computing, for every command, the answers that are possible when each
     point within that distance may or may not count (`Oracle`).

Upstream's own answer is checked against the same computation with a band
of zero width, which keeps the computation honest.

The points of a scan are computed here as upstream computes them: the angle
accumulated in float32, the C library's float sine and cosine (through
ctypes), float32 products, then the fixed transform in double. Which
library routine matters. Upstream's binary, compiled with optimisation,
gets sine and cosine from one call (`__sincosf_stret` on macOS, `sincosf`
with glibc); on macOS that call differs from `sinf` and `cosf` by one
float32 step for about 2.6 % of all angles. The same routine is used here
when the library has it. `EPS` is the distance from a boundary below which
this module does not claim to know what upstream's double arithmetic
decides; such a point may or may not count for upstream.

The ROSlean node computes its own points, and as built by `pixi run build`
(no optimisation) it calls `sinf` and `cosf`, so its point can lie one
float32 step from upstream's: at most 0.25 micrometres at 3 m. `LEAN_SLACK`
allows for that in bound G; bound S is checked on the outputs and needs no
allowance.
"""

import ctypes
import ctypes.util
import itertools
import math

from cm_harness import bits, same_floats
from cm_model import f32

# SPEC, D4: how far outside the configured zone a point can lie and still
# count as inside for ROSlean, in metres.
GROWTH = {'circle': 3.5e-3, 'polygon': 4.4e-3}
EPS = 1e-9
LEAN_SLACK = 1e-6

_libm = ctypes.CDLL(ctypes.util.find_library('m') or ctypes.util.find_library('c'))
for _f in (_libm.cosf, _libm.sinf):
    _f.restype = ctypes.c_float
    _f.argtypes = [ctypes.c_float]


class _SinCos(ctypes.Structure):
    _fields_ = [('sin', ctypes.c_float), ('cos', ctypes.c_float)]


def _sincos_routine():
    """Sine and cosine of a float32 as an optimised C++ build obtains them."""
    try:
        stret = _libm.__sincosf_stret  # macOS
        stret.restype, stret.argtypes = _SinCos, [ctypes.c_float]
        return lambda a: (lambda r: (r.sin, r.cos))(stret(a))
    except AttributeError:
        pass
    try:
        both = _libm.sincosf  # glibc
        both.restype = None
        both.argtypes = [ctypes.c_float, ctypes.POINTER(ctypes.c_float),
                         ctypes.POINTER(ctypes.c_float)]

        def call(a):
            s, c = ctypes.c_float(), ctypes.c_float()
            both(a, ctypes.byref(s), ctypes.byref(c))
            return s.value, c.value
        return call
    except AttributeError:
        return lambda a: (_libm.sinf(a), _libm.cosf(a))


sincos = _sincos_routine()


def rotation(yaw):
    """The rotation matrix of tf2 for a turn of `yaw` about z, from its quaternion."""
    z, w = math.sin(yaw / 2), math.cos(yaw / 2)
    return (1 - 2 * z * z, -2 * z * w), (2 * z * w, 1 - 2 * z * z)


def scan_points(spec, tf, yaw=None):
    """Points of a scan in the base frame, with the C library's float cosine and sine.

    tf = (x, y, half_turn) is the pose of the scan frame; with `yaw` given the
    frame is turned by that angle instead (and the result is exact only up to
    the rounding of the rotation, about 1e-16 m: far below EPS).
    """
    tx, ty, flip = tf
    rot = rotation(yaw) if yaw is not None else None
    angle, inc = f32(spec['angle_min']), f32(spec['angle_increment'])
    lo, hi = f32(spec['range_min']), f32(spec['range_max'])
    pts = []
    for r in spec['ranges']:
        r = f32(r)
        if lo <= r <= hi:
            # The product of two float32 values is exact in double, so rounding
            # it to float32 is the float32 product.
            sin, cos = sincos(angle)
            x, y = f32(r * cos), f32(r * sin)
            if rot:
                x, y = rot[0][0] * x + rot[0][1] * y, rot[1][0] * x + rot[1][1] * y
            elif flip:
                x, y = -x, -y
            pts.append((x + tx, y + ty))
        angle = f32(angle + inc)
    return pts


def segment_distance(p, a, b):
    (x, y), (xi, yi), (xj, yj) = p, a, b
    dx, dy = xj - xi, yj - yi
    if dx == 0 and dy == 0:
        return math.hypot(x - xi, y - yi)
    t = max(0.0, min(1.0, ((x - xi) * dx + (y - yi) * dy) / (dx * dx + dy * dy)))
    return math.hypot(x - (xi + t * dx), y - (yi + t * dy))


def outside_by(z, p):
    """How far p lies outside upstream's zone z, in metres; negative if inside."""
    if z.kind == 'circle':
        return math.hypot(p[0], p[1]) - abs(z.radius)
    d = min(segment_distance(p, z.poly[i - 1], z.poly[i]) for i in range(len(z.poly)))
    return -d if z.inside(p) else d


def demand(zones, triggered, v):
    """The velocity demanded for command v when exactly these zones trigger (R12 to R14)."""
    req = v
    for z, t in zip(zones, triggered):
        if not t or z.action not in ('stop', 'slowdown'):
            continue
        if z.action == 'stop':
            return (0.0, 0.0, 0.0)
        slow = tuple(c * z.ratio for c in v)
        if sum(c * c for c in slow) < sum(c * c for c in req):
            req = slow
    return req


def distinct(demands):
    """The demands without repetitions. Compared by bit pattern: (0.0, -0.0, 0.0), a
    command scaled by 0, and (0.0, 0.0, 0.0), a stop, are different answers, although
    they are equal as Python tuples."""
    return list({tuple(bits(c) for c in d): d for d in demands}.values())


def is_zero(v):
    return all(c == 0.0 for c in v)


def at_least_as_restrictive(a, b):
    """Velocity a is b scaled by factors in [0, 1], component by component."""
    return all(abs(x) <= abs(y) and (x == 0.0 or (x > 0) == (y > 0)) for x, y in zip(a, b))


class Tracker:
    """The stop-publication state (R15) of one node, followed from what it publishes."""

    def __init__(self, stop_pub_timeout_ns):
        self.timeout = stop_pub_timeout_ns
        self.prev_zero = False
        self.stop_stamp = 0

    def answer(self, req, now):
        """What the node publishes if it demands req: None or the three components."""
        if is_zero(req) and self.prev_zero and now - self.stop_stamp > self.timeout:
            return None
        return req

    def update(self, zero, now):
        if zero and not self.prev_zero:
            self.stop_stamp = now
        self.prev_zero = zero


def demanded(outs):
    """What a node demanded, from what it published in answer to a finite command.

    Nothing published means zero: a finite command is left unanswered only
    when zero has been demanded for longer than stop_pub_timeout (R15).
    """
    if not outs:
        return (0.0, 0.0, 0.0)
    v = outs[0]
    return (v[0], v[1], v[5])


class Oracle:
    """Replays a case and judges, command by command, the answers of the two nodes."""

    def __init__(self, zones, source_timeout_ns, stop_pub_timeout_ns, growth=None):
        self.zones = zones
        self.growth = growth or GROWTH
        self.source_timeout = source_timeout_ns
        self.now = 0
        self.scan = None  # (stamp in ns, points)
        self.lean = Tracker(stop_pub_timeout_ns)
        self.upstream = Tracker(stop_pub_timeout_ns)
        self.distances = []  # per zone: outside_by of every point of the stored scan

    def clock(self, t_ns):
        self.now = t_ns

    def scan_in(self, stamp_ns, points):
        self.scan = (stamp_ns, points)
        self.distances = [[outside_by(z, p) for p in points] for z in self.zones]

    def _sets(self, v, reach):
        """Demands possible when a point up to reach[kind] outside a zone may count."""
        if self.source_timeout != 0 and (
                self.scan is None or self.now - self.scan[0] > self.source_timeout):
            return [(0.0, 0.0, 0.0)]
        usable = self.scan is not None and (
            self.source_timeout == 0 or self.now - self.scan[0] <= self.source_timeout)
        options = []
        for k, z in enumerate(self.zones):
            ds = self.distances[k] if usable else []
            sure = sum(d < -EPS for d in ds)
            may = sum(d <= reach[z.kind] + EPS for d in ds)
            if sure >= z.min_points:
                options.append((True,))
            elif may >= z.min_points:
                options.append((False, True))
            else:
                options.append((False,))
        return distinct(demand(self.zones, t, v) for t in itertools.product(*options))

    def band_points(self):
        """Per zone kind: points of the stored scan in the growth band, and all points."""
        out = {'circle': [0, 0], 'polygon': [0, 0]}
        for z, ds in zip(self.zones, self.distances):
            out[z.kind][0] += sum(EPS < d <= self.growth[z.kind] + EPS for d in ds)
            out[z.kind][1] += len(ds)
        return out

    def cmd(self, vel, lean_outs, upstream_outs):
        """Judges the two answers to one command.

        Returns a dict:
          problems   list of (severity, text); severity 'UNSAFE', 'bound' or 'oracle'
          differ     the two nodes demanded different velocities. (A node that
                     publishes nothing demands a zero whose signs are not
                     known; it differs from the other only if that one
                     demands a velocity that is not zero.)
          published_differ
                     what the two nodes published differs. More than `differ`:
                     after a difference the two can be at different points of
                     their stop-publication periods (R15), so that one still
                     publishes zero where the other is silent.
          ambiguous  a point in the growth band could change the answer
          reach      (metres, zone kind): how far outside upstream's zone the
                     farthest point lies that ROSlean must have counted to
                     give its answer; None if the demands agree
        """
        res = {'problems': [], 'differ': False, 'published_differ': False, 'ambiguous': False,
               'reach': None}
        if len(vel) == 3:
            vel = (vel[0], vel[1], 0.0, 0.0, 0.0, vel[2])
        if not all(math.isfinite(c) for c in vel):
            for name, outs in (('roslean', lean_outs), ('upstream', upstream_outs)):
                if outs:
                    res['problems'].append(
                        ('oracle' if name == 'upstream' else 'bound',
                         f'{name} answered a command that is not finite'))
            return res
        v = (vel[0], vel[1], vel[5])
        # Upstream: points surely inside count; points within EPS of a boundary may.
        up_set = self._sets(v, {'circle': 0.0, 'polygon': 0.0})
        lean_set = distinct(self._sets(
            v, {k: g + LEAN_SLACK for k, g in self.growth.items()}) + up_set)
        res['ambiguous'] = len(lean_set) > 1

        def matches(tracker, outs, req):
            want = tracker.answer(req, self.now)
            if want is None:
                return not outs
            return len(outs) == 1 and same_floats(
                outs[0], (want[0], want[1], 0.0, 0.0, 0.0, want[2]))

        if not any(matches(self.upstream, upstream_outs, r) for r in up_set):
            res['problems'].append(('oracle', 'upstream is not as the model of this test says'))
        if not any(matches(self.lean, lean_outs, r) for r in lean_set):
            res['problems'].append(
                ('bound', 'ROSlean gives an answer that no set of points within the stated '
                          'growth of the zones explains'))
        got_l, got_u = demanded(lean_outs), demanded(upstream_outs)
        if not at_least_as_restrictive(got_l, got_u):
            res['problems'].append(
                ('UNSAFE', 'ROSlean demands a less restrictive velocity than upstream'))
        res['published_differ'] = not (len(lean_outs) == len(upstream_outs) and all(
            same_floats(a, b) for a, b in zip(lean_outs, upstream_outs)))
        if lean_outs and upstream_outs:
            res['differ'] = not same_floats(got_l, got_u)
        else:
            res['differ'] = is_zero(got_l) != is_zero(got_u)
        if res['differ']:
            # The smallest reach that explains what ROSlean demands.
            def explains(r):
                return same_floats(got_l, r) if lean_outs else is_zero(r)

            cands = sorted({(d, z.kind) for z, ds in zip(self.zones, self.distances)
                            for d in ds if d > -EPS})
            for d, kind in [(0.0, 'circle')] + cands:
                reach = {'circle': d, 'polygon': d}
                if any(explains(r) for r in self._sets(v, reach) + up_set):
                    res['reach'] = (max(d, 0.0), kind)
                    break
        self.lean.update(is_zero(got_l), self.now)
        self.upstream.update(is_zero(got_u), self.now)
        return res
