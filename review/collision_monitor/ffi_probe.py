#!/usr/bin/env python3
"""CM3/CM4 review: the Lean conversions and the C interface, called directly.

Loads libroslean_core through ctypes (no C++ wrapper, no ROS) and compares
every exported conversion with an exact reference in rational arithmetic, on
special patterns (signed zeros, subnormals, NaN with either sign, infinities,
values one ulp either side of each limit) and on random patterns. Then drives
a monitor through the raw interface with argument values the C++ wrapper never
produces.

    python3 ffi_probe.py <path to libroslean_core.{dylib,so}>
"""
import ctypes
import math
import random
import struct
import sys
from fractions import Fraction

lib = ctypes.CDLL(sys.argv[1])
U8, U32, U64, P = ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint64, ctypes.c_void_p


def sig(name, res, *args):
    f = getattr(lib, name)
    f.restype, f.argtypes = res, list(args)
    return f


init = sig("roslean_core_initialize", ctypes.c_int)
unit = sig("roslean_core_unit", P)
inc = sig("roslean_core_inc", None, P)
dec = sig("roslean_core_dec", None, P)
barr = sig("roslean_core_byte_array", P, ctypes.c_char_p, ctypes.c_size_t)
b_new = sig("roslean_cm_builder_new", P, P)
b_src = sig("roslean_cm_builder_set_source", P, P, U8, U64, U8, U64)
b_spt = sig("roslean_cm_builder_set_stop_pub_timeout", P, P, U64)
b_circ = sig("roslean_cm_builder_add_circle", P, P, U64, U8, U64, U64, U8)
b_poly = sig("roslean_cm_builder_add_polygon", P, P, U8, U64, U64, U8)
b_vert = sig("roslean_cm_builder_add_vertex", P, P, U32, U32)
cm_new = sig("roslean_cm_new", P, P)
cm_ok = sig("roslean_cm_ok", U8, P)
cm_err = sig("roslean_cm_error", U8, P)
cm_zerr = sig("roslean_cm_zone_error", U8, P, U32)
on_scan = sig("roslean_cm_on_scan", P, P, U64, U64, U8, U32, U32, P)
on_cmd = sig("roslean_cm_on_cmd", P, P, U64, U64, U64, U64, U64, U64, U64)
o_kind = sig("roslean_cm_out_kind", U8, P)
o_zone = sig("roslean_cm_out_zone", U32, P)
o_ratio = sig("roslean_cm_out_ratio", U64, P)
c_coord = sig("roslean_cm_coord_mm", U64, U64)
c_radius = sig("roslean_cm_radius_mm", U64, U64)
c_vertex = sig("roslean_cm_vertex_mm", U64, U32)
c_timeout = sig("roslean_cm_timeout_ns", U64, U64)
c_stimeout = sig("roslean_cm_source_timeout_ns", U64, U64, U8, U64)

assert init() != 0


def s64(u):
    return u - (1 << 64) if u >= 1 << 63 else u


def d2b(v):
    return struct.unpack("<Q", struct.pack("<d", v))[0]


def b2d(b):
    return struct.unpack("<d", struct.pack("<Q", b))[0]


def f2b(v):
    return struct.unpack("<I", struct.pack("<f", v))[0]


def b2f(b):
    return struct.unpack("<f", struct.pack("<I", b))[0]


LIMIT = 1000000
fails = 0
counts = {}


def check(name, got, want, what):
    global fails
    counts[name] = counts.get(name, 0) + 1
    if got != want:
        fails += 1
        if fails < 40:
            print(f"DIFF {name}: {what}: Lean {got}, reference {want}")


# ---- reference conversions, exact --------------------------------------

def ref_coord(b):
    v = b2d(b)
    if math.isnan(v):
        return LIMIT
    if math.isinf(v):
        return LIMIT if v > 0 else -LIMIT
    mm = min(LIMIT, math.floor(abs(Fraction(v)) * 1000))
    return -mm if (b >> 63) else mm


def ref_radius(b):
    v = b2d(b)
    if math.isnan(v) or math.isinf(v):
        return -1
    r = math.ceil(abs(Fraction(v)) * 1000) + 1
    return -1 if r > LIMIT else r


def ref_vertex(b):
    v = b2f(b)
    if math.isnan(v) or math.isinf(v):
        return -(1 << 63)
    mm = math.floor(abs(Fraction(v)) * 1000 + Fraction(1, 2))
    mm = min(mm, 1 << 62)
    return -mm if (b >> 31) else mm


def ref_timeout(b):
    v = b2d(b)
    if math.isnan(v) or math.isinf(v):
        return -1
    if v < 0:
        return -1
    if Fraction(v) >= 9200000000:
        return -1
    return int(v * 1e9)  # one IEEE product, then truncation


def ref_via_seconds(ns):
    return int(float(ns) / 1e9 * 1e9)


def specials64():
    out = set()
    vals = [0.0, 5e-324, 2.2250738585072009e-308, 2.2250738585072014e-308, 1e-300, 1e-12, 1e-9,
            9.99e-10, 1e-6, 0.0005, 0.001, 0.0015, 0.25, 0.3, 0.5, 0.999, 0.9999, 1.0, 1.0005,
            2.0, 2.094, 999.0, 999.997, 999.998, 999.9985, 999.999, 999.9995, 1000.0, 1000.001,
            1023.999, 1024.0, 1025.0, 1e6, 2.0 ** 52, 2.0 ** 53, 2.0 ** 63, 9.1e9, 9.2e9, 9.3e9,
            9223372036.854775, 1e10, 1e18, 1e19, 1e300, 1.7976931348623157e308, math.inf]
    for v in vals:
        b = d2b(v)
        for delta in range(-3, 4):
            c = b + delta
            if 0 <= c < 1 << 63:
                out.add(c)
                out.add(c | 1 << 63)
    for b in [0x7FF0000000000001, 0x7FF8000000000000, 0x7FFFFFFFFFFFFFFF, 0xFFF0000000000001,
              0xFFF8000000000000, 0xFFFFFFFFFFFFFFFF, 0x7FF0000000000000, 0xFFF0000000000000]:
        out.add(b)
    for k in range(0, 1000):  # whole and half millimetres
        for v in [k / 1000.0, (k + 0.5) / 1000.0, k + 0.0005, 999.0 + k / 1000.0]:
            b = d2b(v)
            for delta in (-1, 0, 1):
                if b + delta >= 0:
                    out.add(b + delta)
                    out.add((b + delta) | 1 << 63)
    return sorted(out)


def specials32():
    out = set()
    vals = [0.0, 1e-45, 1.17549435e-38, 1e-6, 0.0005, 0.001, 0.0015, 0.0025, 0.3, 0.5, 1.0, 999.997,
            999.9975, 999.998, 1000.0, 16777216.0, 1e30, 3.4028234e38, math.inf]
    for v in vals:
        b = f2b(v)
        for delta in range(-3, 4):
            c = b + delta
            if 0 <= c < 1 << 31:
                out.add(c)
                out.add(c | 1 << 31)
    for k in range(0, 2000):
        b = f2b((k + 0.5) / 1000.0)
        for delta in (-1, 0, 1):
            out.add(b + delta)
            out.add((b + delta) | 1 << 31)
    out |= {0x7F800001, 0x7FC00000, 0x7FFFFFFF, 0xFF800001, 0xFFC00000, 0xFFFFFFFF, 0xFF800000}
    return sorted(out)


def conversions():
    rnd = random.Random(20261123)
    pats = specials64() + [rnd.getrandbits(64) for _ in range(200000)]
    # random values of ordinary size, where the arithmetic is not trivial
    pats += [d2b(rnd.uniform(-1100, 1100)) for _ in range(200000)]
    pats += [d2b(rnd.choice([-1, 1]) * 10 ** rnd.uniform(-12, 11)) for _ in range(200000)]
    for b in pats:
        check("coord_mm", s64(c_coord(b)), ref_coord(b), f"{b2d(b)!r} ({b:#018x})")
        check("radius_mm", s64(c_radius(b)), ref_radius(b), f"{b2d(b)!r} ({b:#018x})")
        check("timeout_ns", s64(c_timeout(b)), ref_timeout(b), f"{b2d(b)!r} ({b:#018x})")
    pats32 = specials32() + [rnd.getrandbits(32) for _ in range(200000)]
    pats32 += [f2b(rnd.uniform(-1100, 1100)) for _ in range(200000)]
    for b in pats32:
        check("vertex_mm", s64(c_vertex(b)), ref_vertex(b), f"{b2f(b)!r} ({b:#010x})")
    # The default of the source's timeout: every k/1000 s up to 30 s, and
    # random values up to the limit.
    secs = [k / 1000.0 for k in range(0, 30001)]
    secs += [rnd.uniform(0, 100) for _ in range(100000)]
    secs += [10 ** rnd.uniform(-10, 9.96) for _ in range(100000)]
    secs += [b2d(d2b(9.2e9) - k) for k in range(1, 50)]
    lost = 0
    for s in secs:
        b = d2b(s)
        ns = ref_timeout(b)
        want = ref_via_seconds(ns) if ns >= 0 else -1
        lost += want != ns
        check("source_timeout_ns (default)", s64(c_stimeout(b, 0, 0)), want, repr(s))
        check("source_timeout_ns (own)", s64(c_stimeout(d2b(1.0), 1, b)), ns, repr(s))
    print(f"default source timeout differs from the single conversion for {lost} of {len(secs)}")


# ---- the monitor through the raw interface -----------------------------

def rec(rng, x, y):
    return struct.pack("<fdd", rng, x, y)


class Mon:
    def __init__(self, zones, src=(1, 2.0, 0, 0.0), spt=1.0):
        b = b_new(unit())
        b = b_src(b, src[0], d2b(src[1]), src[2], d2b(src[3]))
        b = b_spt(b, d2b(spt))
        for z in zones:
            b = z(b)
        self.m = cm_new(b)

    def q(self, f, *a):
        inc(self.m)
        return f(self.m, *a)

    def scan(self, now, stamp, recs, usable=1, rmin=0.0, rmax=100.0):
        data = b"".join(recs)
        self.m = on_scan(self.m, now, stamp & (2 ** 64 - 1), usable, f2b(rmin), f2b(rmax),
                         barr(data, len(data)))

    def cmd(self, now, v=(0.5, 0.0, 0.0, 0.0, 0.0, 0.0)):
        self.m = on_cmd(self.m, now, *[d2b(c) for c in v])
        return self.q(o_kind), self.q(o_zone), self.q(o_ratio)


def circle(radius, action=0, ratio=0.5, min_points=1, enabled=1):
    return lambda b: b_circ(b, d2b(radius), action, d2b(ratio), min_points & (2 ** 64 - 1), enabled)


def polygon(pts, action=0, ratio=0.5, min_points=1, enabled=1):
    def add(b):
        b = b_poly(b, action, d2b(ratio), min_points & (2 ** 64 - 1), enabled)
        for x, y in pts:
            b = b_vert(b, f2b(x), f2b(y))
        return b
    return add


S = 10 ** 9
KIND = {0: "silent", 1: "pass", 2: "slowdown", 3: "stopZone", 4: "stopSource"}


def show(label, out):
    k, z, r = out
    extra = f" zone {z}" if k in (2, 3) else ""
    extra += f" ratio {b2d(r)!r}" if k == 2 else ""
    print(f"  {label}: {KIND.get(k, k)}{extra}")


def monitors():
    print("monitor cases:")
    # A circle of radius 0 and of the smallest radius: what the zone contains.
    for radius in (0.0, -0.0, 5e-324, 0.0005, 0.001):
        m = Mon([circle(radius)])
        res = []
        for x in (0.0, 0.0009, 0.001, 0.0019, 0.002, 0.003):
            m.scan(S, S, [rec(1.0, x, 0.0)])
            res.append((x, KIND[m.cmd(S)[0]]))
        print(f"  radius {radius!r}: ok={m.q(cm_ok)}; " + ", ".join(f"{x} m {k}" for x, k in res))
    # A timeout that is positive but below 1 ns is 0: the check is off.
    for t in (1e-9, 9.999999999e-10, 5e-324):
        m = Mon([circle(0.3)], src=(1, t, 0, 0.0))
        show(f"source_timeout {t!r}, no scan, command at 5 s", m.cmd(5 * S))
    # Calls in an unusual order.
    b = b_new(unit())
    b = b_vert(b, f2b(1.0), f2b(1.0))          # vertex with no zone
    b = b_circ(b, d2b(0.3), 0, 0, 1, 1)
    b = b_vert(b, f2b(math.nan), f2b(1.0))     # refused vertex after a circle
    m = cm_new(b)
    inc(m)
    print(f"  vertex before any zone, NaN vertex after a circle: ok={cm_ok(m)}")
    dec(m)
    # set_source never called: the default is timeout 0, the check is off.
    b = b_circ(b_new(unit()), d2b(0.3), 0, 0, 1, 1)
    m = cm_new(b)
    m = on_cmd(m, 5 * S, d2b(0.5), 0, 0, 0, 0, 0)
    inc(m)
    print(f"  builder without set_source, no scan, command: {KIND[o_kind(m)]}")
    dec(m)
    # A refused source timeout followed by an accepted one clears the refusal.
    b = b_new(unit())
    b = b_src(b, 1, d2b(-1.0), 0, 0)
    b = b_src(b, 1, d2b(2.0), 0, 0)
    m = cm_new(b)
    inc(m)
    print(f"  set_source(-1.0) then set_source(2.0): ok={cm_ok(m)}")
    dec(m)
    # Unusable scan, then commands: R15 state across dropScan.
    m = Mon([circle(0.3)], spt=1.0)
    m.scan(S, S, [rec(5.0, 5.0, 0.0)])
    show("usable far scan, command", m.cmd(S))
    m.scan(S, S, [rec(5.0, 5.0, 0.0)], usable=0)
    show("unusable scan, command at 1 s", m.cmd(S))
    show("command at 2 s (stop_pub_timeout 1 s)", m.cmd(2 * S))
    show("command at 2 s + 1 ns", m.cmd(2 * S + 1))
    # Unusable scan with a refused configuration.
    m = Mon([circle(math.nan)])
    m.scan(S, S, [rec(5.0, 5.0, 0.0)], usable=0)
    show("refused configuration, unusable scan, NaN command", m.cmd(S, (math.nan,) * 6))
    # Non-canonical flags and codes.
    m = Mon([circle(0.3, action=255, enabled=128)])
    m.scan(S, S, [rec(0.1, 0.1, 0.0)], usable=255)
    show("action 255, enabled 128, usable 255, point inside", m.cmd(S))
    # min_points at the limits of the accepted range.
    for mp in (2 ** 31 - 1, 2 ** 31, -2 ** 31, -2 ** 31 - 1, -1, 2 ** 63 - 1, -2 ** 63):
        m = Mon([circle(0.3, min_points=mp)])
        m.scan(S, S, [rec(0.1, 0.1, 0.0)])
        show(f"min_points {mp}: ok={m.q(cm_ok)} zone_error={m.q(cm_zerr, 0)}; one point inside",
             m.cmd(S))
    # Polygon vertices at the coordinate limit; a saturated point next to it.
    for edge in (999.997, 999.9975, 999.998):
        m = Mon([polygon([(edge, 1.0), (edge, -1.0), (990.0, -1.0), (990.0, 1.0)])])
        ok = m.q(cm_ok)
        m.scan(S, S, [rec(1.0, 1e9, 0.0), rec(1.0, math.nan, 0.0), rec(1.0, math.inf, 0.5)])
        show(f"box up to x = {edge} m: ok={ok}; saturated points", m.cmd(S))
        m.scan(S, S, [rec(1.0, 999.9999, 0.0)])
        show(f"box up to x = {edge} m; point at 999.9999 m", m.cmd(S))
    # A circle at the radius limit and a point just outside the limit.
    for radius in (999.998, 999.9989, 999.999, 999.9991):
        m = Mon([circle(radius)])
        ok = m.q(cm_ok)
        m.scan(S, S, [rec(1.0, 999.9999, 0.0), rec(1.0, 1000.0, 0.0), rec(1.0, -math.inf, 0.0)])
        show(f"radius {radius}: ok={ok} radius_mm={s64(c_radius(d2b(radius)))}; points at 999.9999, "
             "1000 and -inf m", m.cmd(S))


conversions()
for name, n in counts.items():
    print(f"{name}: {n} values compared")
monitors()
print(f"{fails} difference(s)")
sys.exit(1 if fails else 0)
