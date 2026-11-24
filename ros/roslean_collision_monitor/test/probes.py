"""Scenarios that decide between the SPEC and UPSTREAM_BEHAVIOUR.md.

docs/collision_monitor/SPEC.md was written from upstream's source and
docs/collision_monitor/UPSTREAM_BEHAVIOUR.md from its binary, independently.
Each scenario here puts to the binary a statement that only one of the two
documents makes, or on which they differ; the expectations are what the
binary did. docs/collision_monitor/COMPARISON.md lists them one by one.

    scenario.py --profile profile_upstream --own       # upstream
    scenario.py roslean_collision_monitor/collision_monitor --profile profile_roslean

The format is scenario.py's. They are kept out of its table because the
reference node (reference_node.py) implements only what
UPSTREAM_BEHAVIOUR.md describes.
"""

from scenario import (
    CLEAR, INVALID, MID, NEAR, NORMAL, P, SLOWDOWN, STOP, V, ZERO, clock, cmd, fan, ray, s, scaled,
    scan)

T = s(100)
HALF = scaled(V, 0.5)

PROBES = {
    # R17: the default of <source>.source_timeout is the node's timeout
    # converted to nanoseconds, back to seconds, and to nanoseconds again.
    # For 2.094 s the first conversion gives 2,093,999,999 ns and the second
    # 2,093,999,998 ns. UPSTREAM_BEHAVIOUR.md, finding 16: converted once.
    'probe-timeout-converted-twice': dict(
        params=[P('StopCircle'), 'source_timeout:=2.094'], steps=[
            clock(T), scan(CLEAR),
            clock(T + 2_093_999_998),
            cmd(V, V, 'source_timeout 2.094 from the node: a scan 2,093,999,998 ns old is used'),
            clock(T + 2_093_999_999),
            cmd(V, ZERO, 'a scan 2,093,999,999 ns old is TOO OLD: the timeout lost 1 ns in a '
                         'second conversion', state=INVALID),
        ]),
    # ... and a timeout given to the source itself is converted once.
    'probe-timeout-of-source': dict(
        params=[P('StopCircle'), 'scan.source_timeout:=2.094'], steps=[
            clock(T), scan(CLEAR),
            clock(T + 2_093_999_999),
            cmd(V, V, 'scan.source_timeout 2.094: a scan 2,093,999,999 ns old is used'),
            clock(T + 2_094_000_000),
            cmd(V, ZERO, 'a scan 2,094,000,000 ns old is too old', state=INVALID),
        ]),
    # stop_pub_timeout is converted once (finding 16 left this open).
    'probe-stop-pub-decimal': dict(
        params=[P('StopCircle'), 'stop_pub_timeout:=2.094', 'source_timeout:=0.0'], steps=[
            clock(T), scan(NEAR),
            cmd(V, ZERO, 'stop', state=('StopCircle', STOP)),
            clock(T + 2_093_999_999),
            cmd(V, ZERO, 'stop_pub_timeout 2.094: zero still published 2,093,999,999 ns later'),
            clock(T + 2_094_000_000),
            cmd(V, None, '... and not 1 ns after that'),
        ]),
    # D9: a negative stop_pub_timeout publishes zero once.
    'probe-stop-pub-negative': dict(
        params=[P('StopCircle'), 'stop_pub_timeout:=-1.0'], steps=[
            clock(T), scan(NEAR),
            cmd(V, ZERO, 'stop_pub_timeout -1: the first zero is published',
                state=('StopCircle', STOP)),
            cmd(V, None, 'the second is not, at the same clock time'),
            scan(CLEAR),
            cmd(V, V, 'obstacle gone: passes', state=NORMAL),
            scan(NEAR),
            cmd(V, ZERO, 'again one zero', state=('StopCircle', STOP)),
            cmd(V, None, 'and no more'),
        ]),
    # R7: a positive timeout below 1 ns is zero, and zero switches the check off.
    'probe-timeout-below-1ns': dict(
        params=[P('StopCircle'), 'source_timeout:=1.0e-10'], steps=[
            clock(T),
            cmd(V, V, 'source_timeout 1e-10 s and no scan ever received: PASSES, as with 0'),
            scan(dict(NEAR, age=s(50))),
            cmd(V, ZERO, 'a scan 50 s old is used', state=('StopCircle', STOP)),
        ]),
    # R6: a disabled source contributes no points and cannot cause a stop.
    'probe-source-disabled': dict(
        params=[P('StopCircle'), 'scan.enabled:=false'], steps=[
            clock(T),
            cmd(V, V, 'source disabled, no scan ever received: passes'),
            scan(NEAR),
            cmd(V, V, 'source disabled, obstacle in the stop zone: PASSES'),
        ]),
    # R11: a disabled zone is skipped.
    'probe-zone-disabled': dict(
        params=[P('StopCircle', 'SlowCircle'), 'StopCircle.enabled:=false'], steps=[
            clock(T), scan(NEAR),
            cmd(V, HALF, 'stop zone disabled, obstacle in both zones: slowed only',
                state=('SlowCircle', SLOWDOWN)),
        ]),
    # R11: the deprecated max_points replaces min_points by max_points + 1.
    'probe-max-points': dict(
        params=[P('StopCircle'), 'StopCircle.max_points:=1'], steps=[
            clock(T), scan(fan(1, 0.25)),
            cmd(V, V, 'max_points 1 (min_points 3 in the file): 1 point passes'),
            scan(fan(2, 0.25)),
            cmd(V, ZERO, '2 points stop: the threshold is max_points + 1',
                state=('StopCircle', STOP)),
        ]),
    # D10: min_points is read into a 32-bit int; 2^32 + 1 is 1.
    'probe-min-points-wraps': dict(
        params=[P('StopCircle'), 'StopCircle.min_points:=4294967297'], steps=[
            clock(T), scan(fan(1, 0.25)),
            cmd(V, ZERO, 'min_points 4294967297: one point stops', state=('StopCircle', STOP)),
        ]),
    # R9: the sign of the radius is lost.
    'probe-radius-negative': dict(
        params=[P('Circle1'), 'Circle1.radius:=-0.5'], steps=[
            clock(T), scan(ray(0.25)),
            cmd(V, ZERO, 'radius -0.5: a point at 0.25 m is inside', state=('Circle1', STOP)),
            scan(ray(0.75)),
            cmd(V, V, 'a point at 0.75 m is outside', state=NORMAL),
        ]),
    # R13: an infinite or NaN ratio never takes effect; -0.0 gives zero (D5).
    'probe-ratio-inf': dict(
        params=[P('SlowOne'), 'SlowOne.slowdown_ratio:=.inf'], steps=[
            clock(T), scan(MID),
            cmd(V, V, 'slowdown_ratio +inf: the command passes'),
        ]),
    'probe-ratio-nan': dict(
        params=[P('SlowOne'), 'SlowOne.slowdown_ratio:=.nan'], steps=[
            clock(T), scan(MID),
            cmd(V, V, 'slowdown_ratio NaN: the command passes'),
        ]),
    'probe-ratio-negative-zero': dict(
        params=[P('SlowZero'), 'SlowZero.slowdown_ratio:=-0.0'], steps=[
            clock(T), scan(MID),
            cmd(V, (-0.0, -0.0, -0.0), 'slowdown_ratio -0.0: negative zeros come out',
                state=('SlowZero', SLOWDOWN)),
        ]),
    # R13: of two triggered slowdown zones with the same ratio, the earlier
    # one acts. Only the state topic shows which.
    'probe-equal-ratios': dict(
        params=[P('SlowPoly', 'SlowCircle')], steps=[
            clock(T), scan(MID),
            cmd(V, HALF, 'two zones with ratio 0.5: the first in `polygons` is reported',
                state=('SlowPoly', SLOWDOWN)),
        ]),
    'probe-equal-ratios-reversed': dict(
        params=[P('SlowCircle', 'SlowPoly')], steps=[
            clock(T), scan(MID),
            cmd(V, HALF, 'the other order: the other zone is reported',
                state=('SlowCircle', SLOWDOWN)),
        ]),
    # R17: a parameter of the wrong YAML type makes configuration fail.
    'probe-wrong-type': dict(
        params=[P('Circle1'), 'Circle1.radius:=1'], steps=[], refused=True),
    # R17: min_points defaults to 4 and slowdown_ratio to 0.5.
    'probe-defaults': dict(
        params=[P('Defaults')], steps=[
            clock(T), scan(fan(3, 0.25)),
            cmd(V, V, 'min_points not set: 3 points pass'),
            scan(fan(4, 0.25)),
            cmd(V, HALF, '4 points: slowed by the default ratio 0.5',
                state=('Defaults', SLOWDOWN)),
        ]),
    # R8 and R11 together: timeout 0, no scan, min_points 0.
    'probe-min-points-zero-no-scan': dict(
        params=[P('StopMin0'), 'source_timeout:=0.0'], steps=[
            clock(T),
            cmd(V, ZERO, 'source_timeout 0, no scan ever, min_points 0: the zone triggers',
                state=('StopMin0', STOP)),
        ]),
}
