"""Where the ROSlean node is expected to differ from upstream in scenario_nav2.py.

    scenario_nav2.py roslean_collision_monitor/collision_monitor --profile profile_roslean_nav2

The expectations of scenario_nav2.py are what upstream's binary does. This
file is the one place that states an answer the ROSlean node is to give
differently, each with the number of the difference in
docs/collision_monitor/SPEC_NAV2.md, section 6. Anything not listed must be
exactly as upstream, and a listed difference that does not occur is a
failure too.

Written by T07, before the node has the extension (T06): nothing here has
been run against a node that implements it. Only the differences a scenario
can see and that do not depend on where the transform is applied are stated
(ND4, ND5, ND6, and the wording of the refusal, ND11). T08 runs the node
through the scenarios and completes this file: what the node does at a zone
boundary after a transform (ND1, and D3/D4 of SPEC.md) is deliberately not
stated here; the scenarios keep every point at least 0.125 m from every
zone edge except where a label says "on" an edge.

EXPECT maps a scenario name to a dict with any of

  diffs    {text: dict(tag, why[, out][, state][, polys])}
                           the check whose label contains `text` differs:
                           `out` (None: nothing published), `state` (None: no
                           state message) and `polys` replace upstream's
  refused  (tag, why)      the node must fail `configure`; the steps are not run
  log      (tag, why, [needle, ...])
                           the node's refusal is worded differently: these
                           must be in its output instead of upstream's text
  skip     (tag, why)      not run
"""

ND4 = 'after shutdown the node publishes nothing (SPEC_NAV2 E5); upstream keeps answering'
ND6 = ('a shape with a non-finite coordinate is ignored and the previous shape, the box, '
       'stays: the point in it still stops')
ND11 = 'the same refusal; the message also says that footprint_topic is read for approach only'

EXPECT = {
    'lifecycle-shutdown-active': {'diffs': {
        'STILL FORWARDED': dict(tag='ND4', why=ND4, out=None),
        'a stop is still decided and published': dict(tag='ND4', why=ND4, out=None, state=None),
    }},
    'polygon-topic': {'diffs': {
        # Upstream: the NaN box replaces the box, the point passes and the
        # state returns to normal; the next box stops again and reports it.
        'a box with a NaN coordinate is ACCEPTED': dict(
            tag='ND6', why=ND6, out=(0.0, 0.0, 0.0), state=None,
            polys=(('Dyn', 'base_footprint',
                    [(0.5, 0.25), (0.5, -0.25), (0.0, -0.25), (0.0, 0.25)]),)),
        'the box once more: stop': dict(
            tag='ND6', why='the stop never ended, so no state message', state=None),
    }},
    'state-name-collision': {'refused': (
        'ND5', 'a zone named "invalid source" is refused: the state topic could not tell it '
               'from an invalid source')},
    'footprint-approach-unchanged': {'skip': (
        'D1', 'action_type approach is outside the subset; the node refuses it')},
    'footprint-edit-stop': {'log': ('ND11', ND11, ['polygon_sub_topic'])},
    'footprint-edit-slowdown': {'log': ('ND11', ND11, ['polygon_sub_topic'])},
    'footprint-edit-example-file': {'log': ('ND11', ND11, ['polygon_sub_topic'])},
}
