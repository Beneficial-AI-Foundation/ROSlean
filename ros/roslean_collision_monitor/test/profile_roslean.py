"""What the ROSlean collision monitor is expected to do in the scenarios of scenario.py.

    scenario.py roslean_collision_monitor/collision_monitor --profile profile_roslean

The expectations in scenario.py are what upstream's binary does. This file
lists every place where the ROSlean node (ros/roslean_collision_monitor) is
expected to answer differently, each with the number of the deliberate
difference in docs/collision_monitor/SPEC.md that explains it. Anything not
listed here must be exactly as upstream, and a listed difference that does
not occur is a failure too: if the node changes, this file has to change
with it.

EXPECT maps a scenario name to a dict with any of

  refused   (D, why)           the node must refuse to start
  diffs     {text: (out, D, why)}
                               the check whose label contains `text` gives
                               `out` (None: nothing published) instead of
                               upstream's answer. For a step at which upstream
                               aborts, `out` is what this node answers.
  params    (D, why, [..])     extra parameters, e.g. the sensor pose that
                               stands in for a static transform
  no_tf     (D, why)           the scenario publishes a transform for upstream;
                               this node is run without it
  skip      (D, why)           not run, with the reason

SCENARIOS holds scenarios for this node only, in scenario.py's format.

The results of a run are recorded in docs/collision_monitor/COMPARISON.md.
"""

import math

from probes import PROBES
from scenario import BOUNDARY_VERTICES, P, ZERO, clock, cmd, ray, scan, s

# The node has no state topic and refuses the parameter (D13), so the
# harness does not set it and checks no state message. That refusal is the
# scenario `roslean-state-topic` below.
STATE_TOPIC = False
# The node looks up no transform (D11). Every scenario that publishes one
# has an entry below.
TF = False

D4_CIRCLE = 'the circle is the radius rounded up to a millimetre plus 1 mm; the point is inside it'
D4_POLY = 'the polygon has a band of sqrt(5) mm around its boundary; every boundary point is inside'
D11_SHIFT = 'base_shift_correction must be false: scan points are not corrected for motion'


def pose(x, y, yaw=0.0):
    return ('D11', 'the fixed sensor pose stands in for the static transform',
            [f'scan.roslean_sensor_pose:=[{x!r},{y!r},{yaw!r}]'])


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
    'ratio-negative': {'refused': ('D5', 'a slowdown ratio with the sign bit set is refused')},
    'ratio-minus-one': {'refused': ('D5', 'a slowdown ratio with the sign bit set is refused')},
    'timeout-negative': {'refused': ('D9', 'a negative source_timeout is refused')},
    'action-none': {'refused': ('D1', 'action_type none is outside the subset')},

    # ---- transforms: D11 ----------------------------------------------------
    # The four corners need four sensor poses, hence four runs of the node:
    # the scenarios roslean-vertex-* below.
    'boundary-vertices': {'skip': ('D11', 'one fixed sensor pose per run; replaced by '
                                          'roslean-vertex-tr, -br, -bl and -tl')},
    'offset': {
        'params': pose(0.25, 0.0),
        'diffs': {'on the circle: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)')},
    },
    'rotated': {
        'params': pose(0.25, 0.0, math.pi),
        'diffs': {'on the circle: outside': (ZERO, 'D4', D4_CIRCLE + ' (500 mm < 501 mm)')},
    },
    'offset-shift-correction': {'refused': ('D11', D11_SHIFT)},
    'shift-correction-same-frame': {'refused': ('D11', D11_SHIFT)},
    'no-odom': {'refused': ('D11', D11_SHIFT)},
    # A scan in a frame that is not the base frame, without a sensor pose,
    # is not used: the same answers as upstream without a transform.
    'no-transform': {'no_tf': ('D11', 'no lookup; a scan in a foreign frame is not used')},
    'timeout-zero-no-transform': {
        'no_tf': ('D11', 'no lookup; a scan in a foreign frame is not used')},
    'transform-wait': {
        'no_tf': ('D11', 'no lookup; transform_tolerance is ignored'),
        'diffs': {
            'no answer while the clock stands still': (
                ZERO, 'D11', 'nothing is waited for: the stop comes at once'),
            'clock + 0.25: the stop comes out': (
                None, 'D11', 'the stop was published already'),
        },
    },
}


def vertex(frame):
    """The steps of boundary-vertices that concern one corner."""
    steps = [clock(T)]
    for i, step in enumerate(BOUNDARY_VERTICES):
        if step[0] == 'scan' and step[1].get('frame') == frame:
            # The label is upstream's verdict; the expected answer is in EXPECT.
            _, vel, out, _, label = BOUNDARY_VERTICES[i + 1]
            steps += [scan(ray(0.5)), cmd(vel, out, label + ' (upstream)')]
    assert len(steps) == 3, frame
    return steps


SCENARIOS = {
    # The four corners of the square (+-0.5, +-0.5), each reached exactly by
    # a range of 0.5 straight ahead from a sensor 0.5 to the side. Upstream
    # counts only the corner (-0.5, 0.5) as inside (boundary-vertices); here
    # all four are (D4).
    'roslean-vertex-tr': dict(
        params=[P('Poly1'), 'scan.roslean_sensor_pose:=[0.0,0.5,0.0]'], steps=vertex('v_tr')),
    'roslean-vertex-br': dict(
        params=[P('Poly1'), 'scan.roslean_sensor_pose:=[0.0,-0.5,0.0]'], steps=vertex('v_br')),
    'roslean-vertex-bl': dict(
        params=[P('Poly1'), 'scan.roslean_sensor_pose:=[-1.0,-0.5,0.0]'], steps=vertex('v_bl')),
    'roslean-vertex-tl': dict(
        params=[P('Poly1'), 'scan.roslean_sensor_pose:=[-1.0,0.5,0.0]'], steps=vertex('v_tl')),
    # The harness gives every other node a state topic; this one must refuse it.
    'roslean-state-topic': dict(
        params=[P('StopCircle'), 'state_topic:=cm_state'], steps=[], refused=True),
}
EXPECT.update({
    'roslean-vertex-tr': {'diffs': {'corner (0.5, 0.5)': (ZERO, 'D4', D4_POLY)}},
    'roslean-vertex-br': {'diffs': {'corner (0.5, -0.5)': (ZERO, 'D4', D4_POLY)}},
    'roslean-vertex-bl': {'diffs': {'corner (-0.5, -0.5)': (ZERO, 'D4', D4_POLY)}},
    'roslean-state-topic': {'refused': ('D13', 'the state topic is not provided')},
})

# The scenarios of probes.py: statements of the SPEC put to upstream's
# binary. This node must answer them as upstream does, except:
SCENARIOS.update(PROBES)
EXPECT.update({
    'probe-stop-pub-negative': {'refused': ('D9', 'a negative stop_pub_timeout is refused')},
    'probe-max-points': {'refused': ('D10', 'max_points is refused')},
    'probe-min-points-wraps': {
        'refused': ('D10', 'min_points outside the range of a 32-bit int is refused')},
    'probe-ratio-negative-zero': {
        'refused': ('D5', 'a slowdown ratio with the sign bit set is refused')},
})
