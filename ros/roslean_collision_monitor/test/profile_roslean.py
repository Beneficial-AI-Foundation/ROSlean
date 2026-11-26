"""What the ROSlean collision monitor is expected to do in the scenarios of scenario.py.

(Brought up to the node with the Nav2 extension by task T06: see the comment
at STATE_TOPIC below.)

    scenario.py roslean_collision_monitor/collision_monitor --profile profile_roslean

The expectations in scenario.py are what upstream's binary does. This file
lists every place where the ROSlean node (ros/roslean_collision_monitor) is
expected to answer differently, each with the number of the deliberate
difference in docs/collision_monitor/SPEC.md that explains it. Anything not
listed here must be exactly as upstream, and a listed difference that does
not occur is a failure too: if the node changes, this file has to change
with it.

EXPECT maps a scenario name to a dict with any of

  refused   (D, why, parameter)
                               the node must refuse to start, and its log must
                               say that this one parameter was refused
  diffs     {text: (out, D, why)}
                               the check whose label contains `text` gives
                               `out` (None: nothing published) instead of
                               upstream's answer. For a step at which upstream
                               aborts, `out` is what this node answers.
  params    (D, why, [..])     extra parameters
  no_tf     (D, why)           the scenario publishes a transform for upstream;
                               this node is run without it
  skip      (D, why)           not run, with the reason

SCENARIOS holds scenarios for this node only, in scenario.py's format.

The results of a run are recorded in docs/collision_monitor/COMPARISON.md.
"""

from probes import PROBES
from scenario import ZERO, s

# Since task T06 the node has the Nav2 extension (SPEC_NAV2.md): it looks
# transforms up as upstream does, so every scenario that publishes one runs
# with it. The entries that stated the differences D11 (no TF, the fixed
# sensor pose, base_shift_correction refused) and D13 (no state topic) are
# gone with those differences.
TF = True
# The node has a state topic now, but this profile still runs it without
# one: where a velocity differs from upstream's (D4, D8 below) the state
# message differs with it, and the entries below state velocities only.
# Tried with True on 2026-11-24 (macOS): 32 of 318 checks failed, 29 of them
# state messages, all in scenarios that have a velocity difference stated
# below, and 3 velocities in the two base-shift scenarios, which are stated
# below since. The state topic is compared with upstream's in
# scenario_nav2.py and random_nav2.py.
STATE_TOPIC = False

D4_CIRCLE = 'the circle is the radius rounded up to a millimetre plus 1 mm; the point is inside it'
D4_POLY = 'the polygon has a band of sqrt(5) mm around its boundary; every boundary point is inside'
T = s(100)

EXPECT = {
    # ---- zone boundaries: D3 (integer millimetres) and D4 (zones grow) -----
    'boundary-circle': {'diffs': {
        'point at exactly 0.5: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)'),
        'one float32 step above 0.5: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)'),
    }},
    'boundary-circle-0.7': {'diffs': {
        'one float32 step above float32(0.7): outside':
            (ZERO, 'D4', D4_CIRCLE + ' (700 mm < 701 mm)'),
    }},
    'boundary-circle-0.3': {'diffs': {
        'range 0.3: outside': (ZERO, 'D4', D4_CIRCLE + ' (300 mm < 301 mm)'),
    }},
    'boundary-polygon': {'diffs': {
        'on the right edge: outside': (ZERO, 'D4', D4_POLY),
        'beyond the left edge: outside': (ZERO, 'D4', D4_POLY),
        'beyond the top edge: outside': (ZERO, 'D4', D4_POLY),
        'on the bottom edge: outside': (ZERO, 'D4', D4_POLY),
    }},
    'boundary-polygon-0.3': {'diffs': {
        'one float32 step beyond: outside': (ZERO, 'D4', D4_POLY),
        'on the right edge: outside': (ZERO, 'D4', D4_POLY),
    }},
    'boundary-polygon-0.7': {'diffs': {
        'on the right edge: outside': (ZERO, 'D4', D4_POLY),
    }},
    'angle-accumulation': {'diffs': {
        'point at y = -3e-6, outside': (ZERO, 'D4', D4_POLY + ' (3 micrometres below the edge)'),
    }},

    # ---- slowdown selection: D6 ---------------------------------------------
    'signs-slowdown': {'diffs': {
        'velocity whose square underflows': (
            (5e-201, 0.0, 0.0), 'D6',
            'the zone is chosen by its ratio, not by the squared norm of the product: slowed'),
    }},

    # ---- stamps: D8 ---------------------------------------------------------
    'negative-stamp': {'diffs': {
        'the node ABORTS': (ZERO, 'D8', 'no abort; the stamp counts as 0, so at clock 100 s the '
                                        'scan is too old: stop'),
    }},

    # ---- configurations upstream accepts and this node refuses --------------
    'ratio-negative': {'refused': ('D5', 'a slowdown ratio with the sign bit set is refused',
                                   'SlowNegative.slowdown_ratio')},
    'ratio-minus-one': {'refused': ('D5', 'a slowdown ratio with the sign bit set is refused',
                                    'SlowMinusOne.slowdown_ratio')},
    'timeout-negative': {'refused': ('D9', 'a negative source_timeout is refused', 'source_timeout')},
    'action-none': {'refused': ('D1', 'action_type none is outside the subset',
                                'NoAction.action_type')},

    # ---- transforms (N23, N25): the points are transformed in the shim; a
    # point that lands exactly on a zone boundary is inside (D4).
    'boundary-vertices': {'diffs': {
        'corner (0.5, 0.5): outside': (ZERO, 'D4', D4_POLY),
        'corner (0.5, -0.5): outside': (ZERO, 'D4', D4_POLY),
        'corner (-0.5, -0.5): outside': (ZERO, 'D4', D4_POLY),
    }},
    'offset': {
        'diffs': {'on the circle: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)')},
    },
    'rotated': {
        'diffs': {'on the circle: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)')},
    },
    # Base shift correction (N24) is provided since task T06; these two
    # scenarios put points on the circle.
    'offset-shift-correction': {
        'diffs': {'on the circle: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)')},
    },
    'shift-correction-same-frame': {'diffs': {
        'point at exactly 0.5: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)'),
        'one float32 step above 0.5: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)'),
    }},
}


SCENARIOS = {}

# The scenarios of probes.py: statements of the SPEC put to upstream's
# binary. This node must answer them as upstream does, except:
SCENARIOS.update(PROBES)
EXPECT.update({
    'probe-stop-pub-negative': {
        'refused': ('D9', 'a negative stop_pub_timeout is refused', 'stop_pub_timeout')},
    'probe-max-points': {'refused': ('D10', 'max_points is refused', 'StopCircle.max_points')},
    'probe-min-points-wraps': {
        'refused': ('D10', 'min_points outside the range of a 32-bit int is refused',
                    'StopCircle.min_points')},
    'probe-ratio-negative-zero': {
        'refused': ('D5', 'a slowdown ratio with the sign bit set is refused',
                    'SlowZero.slowdown_ratio')},
})
