# Collision monitor: the main theorems

What is proved about the Lean core of the collision monitor, in plain words,
with a link to each statement. The theorems are in
`ROSlean/CollisionMonitor/`, are checked by `lake build` in the repository
root, and use no `sorry`, no `native_decide` and no axioms beyond Lean's
three standard ones (`propext`, `Classical.choice`, `Quot.sound`; checked
with `#print axioms` on 54 theorems, listed in
[`tasks/reports/T05.md`](../../tasks/reports/T05.md): the characterisations
of each output and action, the properties, the lifecycle, shape, transform
and trace theorems, and the end-to-end theorems of the C interface; the
sources contain no `axiom` declaration). Rule numbers
(R…), differences (D…), assumptions (A…) and trusted parts (T…) refer to
[SPEC.md](SPEC.md); N…, ND… and E1–E8 to [SPEC_NAV2.md](SPEC_NAV2.md).

The model contains the Nav2 extension of SPEC_NAV2.md, sections 3 and 4: the
lifecycle, the transforms that arrive with each command, zones whose shape
arrives on a topic, and the report for the state topic. The C interface and
the node do not yet (task T06); see "The C interface and the conversions"
for how the present node's calls are read in the extended model.

Links point to line numbers as of 2026-11-24; if a link lands a few lines
off, search the file for the theorem name.

| File | Content |
| --- | --- |
| [`Basic.lean`](../../ROSlean/CollisionMonitor/Basic.lean) | Which points are tested, which zone is selected, the zones with their shapes filled in |
| [`Properties.lean`](../../ROSlean/CollisionMonitor/Properties.lean) | One command: the output, the state, the properties of the SPEC |
| [`Lifecycle.lean`](../../ROSlean/CollisionMonitor/Lifecycle.lean) | The other events: lifecycle transitions, shape messages; the invariants of reachable states |
| [`Trace.lean`](../../ROSlean/CollisionMonitor/Trace.lean) | Event sequences: the trace invariant, the history, when nothing is published, `stop_pub_timeout` |
| [`Geometry.lean`](../../ROSlean/CollisionMonitor/Geometry.lean) | Circle and polygon tests, the margin, saturated points, `Config.ok` |
| [`Transform.lean`](../../ROSlean/CollisionMonitor/Transform.lean) | The transform is applied exactly: points, vertices, the identity, the margin with transforms |
| [`Bits.lean`](../../ROSlean/CollisionMonitor/Bits.lean) | The IEEE-754 bit-pattern helpers |
| [`Convert.lean`](../../ROSlean/CollisionMonitor/Convert.lean) | The conversions at the boundary: millimetres, nanoseconds, refusals |
| [`FFI.lean`](../../ROSlean/CollisionMonitor/FFI.lean) | The C interface computes the model; refused configurations stop every command |

## What the theorems are about

The model is in
[`core/ROSleanCore/CollisionMonitor.lean`](../../core/ROSleanCore/CollisionMonitor.lean)
and, for points and transforms,
[`core/ROSleanCore/CollisionMonitorXform.lean`](../../core/ROSleanCore/CollisionMonitorXform.lean).

| Name | Meaning |
| --- | --- |
| [`step`](../../core/ROSleanCore/CollisionMonitor.lean#L607) | Processes one event. Returns the new state and the output. Total. |
| [`Event`](../../core/ROSleanCore/CollisionMonitor.lean#L222) | `configure`, `activate`, `deactivate`, `cleanup`, `shutdown` (E1–E5); `scan now s` (E6); `shape k verts place`, a polygon message for zone `k` (E7); `cmd now v scanTf zoneTfs`, a velocity command with the transform looked up for the stored scan (or `none`) and one transform per zone (E8). A missing transform is not an event. |
| [`run`](../../core/ROSleanCore/CollisionMonitor.lean#L647) | `step` over a list of events. |
| [`State.init`](../../core/ROSleanCore/CollisionMonitor.lean#L266) | The state at start-up, for a configuration: unconfigured, no scan, no shape. |
| [`Phase`](../../core/ROSleanCore/CollisionMonitor.lean#L204) | `unconfigured`, `inactive`, `active`, `finalized`. |
| [`Output`](../../core/ROSleanCore/CollisionMonitor.lean#L549) | `silent` (nothing is published, on any topic), or `answer velocity report` for a processed command: `velocity` is the action to publish or `none` if it is withheld (R15); `report` is the state message or `none`. `answer none none` differs from `silent`: the zone polygons are published (N16). [`Output.velocity`](../../core/ROSleanCore/CollisionMonitor.lean#L562) and [`Output.report`](../../core/ROSleanCore/CollisionMonitor.lean#L567) read the two parts and are `none` for `silent`. |
| [`Action`](../../core/ROSleanCore/CollisionMonitor.lean#L436) | `pass` (command unchanged), `slowdown k r` (multiply by the double with bit pattern `r`; zone `k`), `stopZone k` (zero; zone `k`), `stopSource` (zero; the source is invalid). |
| [`Report`](../../core/ROSleanCore/CollisionMonitor.lean#L213) | `normal`, `stop k`, `slowdown k`, `source`: what a state message says (N13). [`Action.report`](../../core/ROSleanCore/CollisionMonitor.lean#L541): what an action reports for a command. |
| [`evaluate`](../../core/ROSleanCore/CollisionMonitor.lean#L517) | The action demanded at a given time with given transforms. It does not depend on the command's velocity. |
| [`sourcePoints`](../../core/ROSleanCore/CollisionMonitor.lean#L464) | The points the zones are tested against, or `none` if the source is invalid. |
| [`liveConfig`](../../core/ROSleanCore/CollisionMonitor.lean#L511) | The zones as they are tested for a command: the configured zones, each topic zone with the shape the state holds for it, refreshed with the command's zone transforms (N34). |
| [`Zone.triggered`](../../core/ROSleanCore/CollisionMonitor.lean#L359) | The zone is enabled, has a shape, and at least `minPoints` points are inside it. |
| [`Xform.pointMm`](../../core/ROSleanCore/CollisionMonitorXform.lean#L261), [`Xform.vertexMm`](../../core/ROSleanCore/CollisionMonitorXform.lean#L267) | A scan-frame point, and a received vertex, placed in the base frame by a transform, in millimetres. Exact integer arithmetic. |
| [`Action.yieldsZero`](../../core/ROSleanCore/CollisionMonitor.lean#L533) | The velocity the action yields for a command is zero: a stop, or a zero command, or a ratio of zero. |
| `prevZero`, `stopStamp`, `prevReport` | The state fields behind `stop_pub_timeout` (R15) and the state topic (N13). |

"The output of a command" below is what `step` returns for a command in a
state `st`. Unless said otherwise the state is arbitrary, nothing is assumed
about the clock, and nothing is assumed about the configuration (`Config.ok`
is needed only where stated).

Two predicates name the zone that wins:
[`IsFirstStop`](../../ROSlean/CollisionMonitor/Basic.lean#L174) `zones pts k`
(zone `k` is a triggered stop zone and no stop zone before it is triggered)
and
[`IsBestSlowdown`](../../ROSlean/CollisionMonitor/Basic.lean#L253) `zones pts k r`
(zone `k` is a triggered slowdown zone with ratio pattern `r` below that of
1.0, no triggered slowdown zone has a smaller ratio pattern, and none before
`k` has the same). Each holds for at most one `k`
([`IsFirstStop.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L182), [`IsBestSlowdown.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L262)),
which is the answer to "which zone is named when several trigger". The zones
are always those in force, `(liveConfig cfg st zoneTfs).zones`.

Two more name the source:
[`SourceInvalid`](../../ROSlean/CollisionMonitor/Basic.lean#L94) (the source is enabled, its timeout is not 0, and no scan
is stored, or the stored scan's stamp is more than the timeout before `now`,
or the command carries no usable transform for it) and
[`NoTransform`](../../ROSlean/CollisionMonitor/Basic.lean#L78) (the command's transform for the scan is missing or not
usable).

## The two central theorems

**When a command is processed, and what is then published.**

| | Theorem | Statement |
| --- | --- | --- |
| Processed | [`processed_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L52), [`output_eq_silent_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L225) | The output of an event is not `silent` **iff** the event is a command, the node is active, and the command has no infinite or NaN component. Every other event, and every event in another phase, gives `silent`. |
| Velocity | [`velocity_eq_some_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L276), [`velocity_eq_none_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L304), [`velocity_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L239) | The output carries the velocity action `a` **iff** the command is processed, `a` is the action `evaluate` demands, and it is not the case that all three hold: the velocity `a` yields is zero, the velocity demanded for the previous command was zero, and the stop stamp is more than `stop_pub_timeout` before `now` (R15). In that case, and for a command that is not processed, there is no velocity. Nothing else is ever published ([`velocity_eq_none_or`](../../ROSlean/CollisionMonitor/Properties.lean#L321)). |
| Report | [`report_eq_some_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L338), [`report_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L252), [`step_cmd_prevReport`](../../ROSlean/CollisionMonitor/Properties.lean#L206) | The output carries the report `r` **iff** the command is processed, `r` is what the demanded action reports for it, and `r` differs from the previous report. The new previous report is what the action reports, in both cases. Whether the velocity is published or withheld does not enter: a report is made also when the velocity is withheld. |
| Whole | [`step_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L153), [`output_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L262) | The new state and the output of a processed command, in one equation. |

**Which action is demanded.** Four if-and-only-if statements, one per action:

| Action | Demanded iff | Theorem |
| --- | --- | --- |
| `stopSource` | the source is invalid (`SourceInvalid`) | [`evaluate_eq_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L68) |
| `stopZone k` | the source is valid and `k` is the first stop zone triggered by its points | [`evaluate_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L79) |
| `slowdown k r` | the source is valid, **no stop zone is triggered**, and `k` is the first triggered slowdown zone with the smallest ratio pattern, `r`, which is below the pattern of 1.0 | [`evaluate_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L93) |
| `pass` | the source is valid, no stop zone is triggered, and no slowdown zone with ratio below 1 is triggered | [`evaluate_eq_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L107) |

"The source is valid, with points `pts`" is
[`sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L130):
the source is disabled and there are no points; or the timeout is 0, no scan
is stored, and there are no points; or a scan is stored, its stamp is not
more than the timeout before `now`, and either the command carries a usable
transform `t` and `pts` are the points of the scan's valid readings under
`t`, or it carries none, the timeout is 0, and there are no points (N27).
[`sourcePoints_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L100): otherwise the source is invalid.

Combined, they say when each velocity occurs:
[`velocity_eq_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L403), [`velocity_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L418),
[`velocity_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L433), [`velocity_eq_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L445).

Because these are equivalences, they also give the "only if" directions that
a list of safety properties alone does not: a monitor that is silent
whenever a stop is demanded, or that stops whenever a slowdown zone
triggers, or that always stops, satisfies "never publishes more than
allowed" but not these. Read off separately:
[`evaluate_stopZone_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L570)
(a stop named after zone `k` only if zone `k` is an enabled stop zone that
has a shape with at least `minPoints` points inside),
[`evaluate_slowdown_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L589),
[`evaluate_isStop_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L606)
(zero is demanded whatever the command iff the source is invalid or a stop
zone is triggered).

## The property table

One command, from an arbitrary state. "Published" and "withheld" are about
the velocity of the output; a withheld velocity is `answer none _`, not
`silent`.

| Property | Theorem | Statement |
| --- | --- | --- |
| Stop | [`stop`](../../ROSlean/CollisionMonitor/Properties.lean#L482) | Node active, command finite, source valid, some stop zone in force triggered: the action is `stopZone k` for the first such zone, and it is published, unless `prevZero` holds and the stop stamp is more than `stop_pub_timeout` old; then, and only then, the velocity is withheld. |
| A demanded stop is published | [`velocity_of_isStop`](../../ROSlean/CollisionMonitor/Properties.lean#L465) | The same for any stop, by a zone or for an invalid source. |
| Stop wins over slowdown | [`stop_not_pass_or_slowdown`](../../ROSlean/CollisionMonitor/Properties.lean#L500) | With a triggered stop zone neither `pass` nor a slowdown is published, wherever the stop zone stands in the configuration. |
| Stale data | [`stale`](../../ROSlean/CollisionMonitor/Properties.lean#L536) | Source enabled, timeout not 0, and no scan or a scan stamped more than the timeout ago: the action is `stopSource` **whatever transform the command carries**, published or withheld exactly as for Stop. |
| No transform | [`no_transform`](../../ROSlean/CollisionMonitor/Properties.lean#L557) | Source enabled, timeout not 0, and the command carries no usable transform for the scan: the action is `stopSource` whatever scan is stored, published or withheld exactly as for Stop. Both are readings of [`source_invalid`](../../ROSlean/CollisionMonitor/Properties.lean#L519). |
| … needs its hypotheses | [`evaluate_ne_stopSource_of_off`](../../ROSlean/CollisionMonitor/Properties.lean#L640), [`sourcePoints_of_no_transform_timeout_zero`](../../ROSlean/CollisionMonitor/Properties.lean#L653) | With the source disabled or timeout 0, `stopSource` is never demanded (R6, R8, N27); with timeout 0 and no transform the zones are tested against no points at all. The counterexample is in `Examples.lean`: the scan that stops the robot when its transform is known lets it pass when the transform is missing. Upstream's behaviour, kept; fail-open. |
| Bad data (a) | [`output_filter_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L697), [`points_of_not_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L711) | Readings that are not valid never change a decision: with them removed from the stored scan, the output of every command is the same; one such reading can be removed whatever coordinates it carries and whatever the transform. More generally two scans with the same stamp and the same valid points under every transform are answered alike ([`output_congr_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L678)). |
| … what "valid" is | [`validBeam_iff_value`](../../ROSlean/CollisionMonitor/Bits.lean#L187) | A reading is valid iff neither it nor a range limit is NaN and the number it denotes lies between the limits, inclusive (R4). |
| … what the points are | [`points_eq`](../../ROSlean/CollisionMonitor/Properties.lean#L664), [`mem_points_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L671) | The points of a scan under a transform are `Xform.pointMm` of the coordinates of its valid readings, in order. |
| Bad data (b) | [`step_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Properties.lean#L146) | A command with an infinite or NaN component: output `silent`, state unchanged. |
| Bad data (c), Totality | by definition | `step`, `run`, `Xform.pointMm` and `Xform.vertexMm` are total functions; Lean accepts them without `partial` or `unsafe`. |
| No amplification | [`no_amplification`](../../ROSlean/CollisionMonitor/Properties.lean#L763) | Whatever velocity is published, for whatever event, is `pass`, a stop, or `slowdown k r` where `r` is below the pattern of 1.0 and is the configured ratio of zone `k`. Holds for every configuration and every state. |
| … as a number | [`published_ratio_in_unit_interval`](../../ROSlean/CollisionMonitor/Properties.lean#L779), [`lt_one_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L239) | Such an `r` has its sign bit clear and denotes a number in `[0, 1)`. |
| Slowdown bound | [`slowdown_bound`](../../ROSlean/CollisionMonitor/Properties.lean#L790), [`evaluate_slowdown_le`](../../ROSlean/CollisionMonitor/Properties.lean#L817) | If a slowdown zone in force with ratio pattern `rz` below 1.0 is triggered, the action is a stop by a zone or a slowdown with ratio pattern at most `rz`; a demanded ratio is at most that of *every* triggered slowdown zone. |
| Pass-through | [`pass_through`](../../ROSlean/CollisionMonitor/Properties.lean#L834), [`pass_through_of_nonzero`](../../ROSlean/CollisionMonitor/Properties.lean#L855) | Node active, command finite, source valid, no zone triggered: the action is `pass`; it is published, except that a zero command is withheld under R15. A non-zero command is always published unchanged. ("Source valid" includes, when a scan is stored and the timeout is not 0, a usable transform.) |
| Monotonicity | [`zonesAction_stop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L876), [`zonesAction_slowdown_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L892), [`evaluate_isStop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L923) | Adding points never lifts a stop (the zone named stays or moves earlier), and never turns a slowdown into `pass` or into a larger ratio. Adding readings to the stored scan never lifts a stop, for the same clock and the same transforms. |
| State | [`step_cmd_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L173), [`step_cmd_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L185), [`step_cmd_stopStamp`](../../ROSlean/CollisionMonitor/Properties.lean#L194), [`step_cmd_shapes`](../../ROSlean/CollisionMonitor/Properties.lean#L215) | A command never changes the scan or the phase. After a processed command `prevZero` is "the demanded velocity was zero" (published or not), `stopStamp` is `now` if it was zero and the previous one was not, else unchanged, and the shapes are the refreshed ones, or unchanged if the source was invalid. |
| R15, simple cases | [`velocity_of_not_yieldsZero`](../../ROSlean/CollisionMonitor/Properties.lean#L371), [`velocity_of_not_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L380), [`velocity_of_within_stopPubTimeout`](../../ROSlean/CollisionMonitor/Properties.lean#L390) | A processed command gets a velocity if the demanded velocity is not zero, or the previous one was not zero, or `now` is at most `stopStamp + stop_pub_timeout`. |

### What the two velocity properties do and do not say

The core never multiplies. For a slowdown it hands the shim the ratio's bit
pattern, and the shim computes `command × ratio` in floating point.

- **Proved in Lean:** a published slowdown carries a pattern `r` with
  `r < F64.one`; this is equivalent to "sign bit clear, and the number
  denoted is in `[0, 1)`" (`lt_one_iff`); `r` is the configured ratio of the
  zone named; `r` is at most the ratio pattern of every triggered slowdown
  zone, and for patterns with the sign bit clear the order of patterns is the
  order of the numbers
  ([`magVal_strictMono`](../../ROSlean/CollisionMonitor/Bits.lean#L39)).
- **Not proved, trusted (T1):** that the IEEE-754 product of a finite double
  with a double in `[0, 1)` has magnitude at most the original and the same
  sign or is zero, and that it does not grow when the ratio shrinks. "The
  published magnitude is at most the commanded one" and "at most that of
  `command × rz`" are therefore the Lean theorems *plus* T1, not Lean
  theorems.

## The Nav2 extension

The properties of SPEC_NAV2.md, section 4, that are not in the tables above.
One step from an arbitrary state, in
[`Lifecycle.lean`](../../ROSlean/CollisionMonitor/Lifecycle.lean); the
versions over runs are in the next section.

| Property | Theorem | Statement |
| --- | --- | --- |
| Inactive | [`output_of_not_active`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L73), [`step_cmd_of_not_active`](../../ROSlean/CollisionMonitor/Properties.lean#L139) | While the node is not active the output of every event is `silent`, and a command leaves the state unchanged (N4). |
| After shutdown | [`step_of_finalized`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L149) | In the finalized phase no event changes the state and every output is `silent` (E5, ND4). |
| The lifecycle | [`step_phase`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L94), [`nextPhase`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L84), [`step_configure`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L110), [`step_activate`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L116), [`step_deactivate`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L122), [`step_cleanup`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L129), [`step_shutdown`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L137), [`step_scan`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L142) | The phase after an event is a function of the phase before it and the event: a transition the phase does not allow changes nothing; `shutdown` leads to `finalized` from everywhere; scans, shapes and commands never change the phase. Activation changes nothing else. Deactivation clears `prevZero` and sets the previous report to `normal`; the scan, the shapes and the stop stamp stay (N6). Cleanup discards the scan and every shape (N7). A scan is stored while the node is inactive or active and not otherwise (N4). No transition publishes anything ([`output_of_not_cmd`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L39)). |
| Shape accepted (iff) | [`acceptShape_eq_some_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L202) | A polygon message for zone `k` installs a shape **iff** the node is inactive or active, zone `k` exists and is a topic zone, the message has at least three vertices, all with finite coordinates, it is in the base frame or its transform was found and is usable, and every vertex, placed and rounded to the millimetre, is within the coordinate limit of `Shape.ok` (N33, ND6, ND7). The shape then holds the vertices as received, whether the frame is foreign, and the placed vertices. |
| … the state | [`step_shape_of_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L281), [`step_shape_of_refused`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L288), [`shapes_after_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L323), [`step_shape_changes`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L297), [`step_shape_same`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L306) | An accepted message replaces entry `k` of the shapes and changes nothing else: not the other entries, the scan, `prevZero`, the stop stamp, the previous report, the phase. A refused message changes nothing. The state changes **only if** the message is accepted; the converse is false (see below). |
| No shape | [`not_triggered_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L402), [`evaluate_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L417), [`output_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L452) | If zone `k` is a topic zone and the state holds no shape for it, zone `k` is not triggered by any points, whatever its `min_points`, and no output names it, neither as a velocity action nor in a report. This is upstream's behaviour (N32) and it is **not** fail-safe: a stop zone without shape does not stop. |
| Replacing a shape affects that zone only | [`withShapes_getElem?_congr`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L478), [`zonesAction_withShapes_congr`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L487) | A zone as it stands depends on the shapes only through its own entry; for two lists of shapes that agree on every zone but `k`, the actions for any points are the same unless zone `k` is triggered under the one and not under the other. |
| The shapes at a command | [`refreshShapes_getElem?`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L341), [`refresh_of_base`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L375), [`refresh_of_foreign`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L382), [`refresh_of_no_transform`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L391), [`refresh_raw`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L362), [`refreshShapes_getD_eq_none_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L350) | For a command the shape of every enabled topic zone is refreshed with that zone's transform: a shape received in the base frame never moves; one received in another frame has its current vertices recomputed from the received ones under a usable transform that keeps them within the coordinate limit, and keeps its vertices otherwise (N34). The refresh neither gives a zone a shape nor takes one away. |
| Lifecycle invariants | [`Inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L532), [`inv_init`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L541), [`inv_step`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L583), [`reachable_inv`](../../ROSlean/CollisionMonitor/Trace.lean#L149) | In every state reachable from `State.init cfg`: while the node is unconfigured or inactive, `prevZero` is false and the previous report is `normal`; while it is unconfigured, no scan and no shape is stored; the shapes have one entry per zone, and a shape only at a topic zone; every stored shape has at least three vertices, as many current vertices as received ones, all within the coordinate limit ([`ShapeOk`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L516)). |
| After activation | [`after_activation`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L714), [`after_activation_of_inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L735), [`run_after_activation`](../../ROSlean/CollisionMonitor/Trace.lean#L617) | From an inactive state with no zero on record and previous report `normal` (every reachable inactive state), `activate` makes the node active, and a finite command that comes next gets a velocity, also when that velocity is zero: a stop is never withheld right after an activation (N6). Its report is published unless it is `normal`. |
| Saturated points and topic zones | [`liveConfig_bounded`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L755), [`zonesAction_filter_inRange'`](../../ROSlean/CollisionMonitor/Geometry.lean#L692), [`contains_eq_false_of_far'`](../../ROSlean/CollisionMonitor/Geometry.lean#L623) | In a reachable state with an accepted configuration every zone in force, topic zones with their shapes included, lies within the coordinate limit ([`ShapeBounded`](../../ROSlean/CollisionMonitor/Geometry.lean#L597)); so a saturated point, and the point given for a reading that is not finite, is in none of them and does not change the action. |

### Where SPEC_NAV2.md, section 4, was false as stated

Two statements of the design did not hold of the model it specifies. The
model was not changed; the theorems state what is true, and SPEC_NAV2.md is
corrected.

1. **"A `shape` event changes the state iff it is accepted."** Only "only
   if" holds. Counterexample: the same accepted message delivered twice. The
   second is accepted and leaves the state exactly as it was
   ([`step_shape_same`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L306)). True: a refused message changes nothing
   ([`step_shape_of_refused`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L288)); an accepted one sets entry `k` to the new
   shape and changes nothing else ([`step_shape_of_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L281)).
2. **"If `phase ≠ active` then `prevZero = false` and `prevReport =
   normal`."** False in the finalized phase. Counterexample: `configure`,
   `activate`, a finite command with no scan stored and a non-zero source
   timeout (a stop: `prevZero` becomes true, the previous report `source`),
   `shutdown`. E5 changes the phase and nothing else, so the finalized state
   has `prevZero = true`. True, and proved ([`Inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L532), field `quiet`): the
   statement for the phases unconfigured and inactive. In the finalized
   phase the two fields are never read again ([`step_of_finalized`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L149)).

Two statements were made precise:

3. **"`Xform.identity.pointMm x y = (coordMm x, coordMm y)`"** holds for
   finite `x` and `y` ([`pointMm_identity`](../../ROSlean/CollisionMonitor/Transform.lean#L595)). For a coordinate that is not
   finite the model's point is `Point.far`, `(coordLimit, coordLimit)`, as
   section 3.1 says, where `coordMm` gives `±coordLimit` on that axis alone.
   Both lie outside every accepted zone.
4. **"On runs: no output names a topic zone `k` before an accepted `shape k`
   event that follows the last cleanup."** "The last cleanup" has to be one
   that took effect (a `cleanup` outside the inactive phase is a no-op and
   discards nothing). Proved in the form: after any history that leaves the
   node unconfigured, and any events that follow among which there is no
   shape message for zone `k`, no output names zone `k`
   ([`run_output_of_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L596)).

## Over sequences of events

In [`Trace.lean`](../../ROSlean/CollisionMonitor/Trace.lean). "From
start-up" is from `State.init cfg`, where the node is unconfigured; the
events are arbitrary, lifecycle transitions in any order included. The phase
reached is [`phaseAfter`](../../ROSlean/CollisionMonitor/Trace.lean#L127) `.unconfigured es`, the scan in force [`lastScan`](../../ROSlean/CollisionMonitor/Trace.lean#L186)
`es .unconfigured none`.

| Theorem | Statement |
| --- | --- |
| [`run_getElem?`](../../ROSlean/CollisionMonitor/Trace.lean#L84), [`run_output`](../../ROSlean/CollisionMonitor/Trace.lean#L95) | **Trace invariant.** The `n`-th output of any run from any state is the single-step output for the `n`-th event in the state reached after the first `n` events. Every theorem above therefore holds at every point of every run. [`run_append`](../../ROSlean/CollisionMonitor/Trace.lean#L59), [`run_concat`](../../ROSlean/CollisionMonitor/Trace.lean#L69): running two sequences one after the other; one more event. |
| [`run_phase`](../../ROSlean/CollisionMonitor/Trace.lean#L134), [`reachable_inv`](../../ROSlean/CollisionMonitor/Trace.lean#L149), [`run_inv`](../../ROSlean/CollisionMonitor/Trace.lean#L141) | The phase after a run is the fold of `nextPhase` over its events. Every state reachable from start-up satisfies the lifecycle invariants. |
| [`run_scan`](../../ROSlean/CollisionMonitor/Trace.lean#L210) | After any run the stored scan is the scan in force: that of the scan event received last while the node was inactive or active, unless a cleanup that took effect followed it; whatever the stamps say (R5). [`lastScan_append_scan`](../../ROSlean/CollisionMonitor/Trace.lean#L224), [`lastScan_append_cmd`](../../ROSlean/CollisionMonitor/Trace.lean#L231), [`lastScan_append`](../../ROSlean/CollisionMonitor/Trace.lean#L216). |
| [`run_sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Trace.lean#L239) | The points tested after any events since start-up, stated on the events and the command's transform alone. |
| [`run_processed`](../../ROSlean/CollisionMonitor/Trace.lean#L110) | **Only processed commands are answered.** If a position of a run has an output other than `silent`, the event there is a finite command and the node was active. |
| [`run_silent_before_activate`](../../ROSlean/CollisionMonitor/Trace.lean#L272) | **Inactive.** In any run from start-up, if no event before position `n` is an `activate`, the output at position `n` is `silent`. |
| [`run_silent_after_shutdown`](../../ROSlean/CollisionMonitor/Trace.lean#L308) | In any run from any state, every output after a `shutdown` event is `silent`, whatever follows it (ND4). |
| [`run_silent_after_deactivate`](../../ROSlean/CollisionMonitor/Trace.lean#L349), [`run_silent_of_no_activate`](../../ROSlean/CollisionMonitor/Trace.lean#L257) | If position `m` is a `deactivate` and no `activate` lies strictly between `m` and `n`, the output at position `n` is `silent`. |
| [`run_stop`](../../ROSlean/CollisionMonitor/Trace.lean#L389) | **Stop.** After any events since start-up that leave the node active: if the scan in force is not too old, the command carries a usable transform, and the scan's points under it trigger a stop zone with a fixed shape, a finite command gets the velocity `stopZone k` for the first triggered stop zone, or none in the one case of R15. |
| [`run_stale`](../../ROSlean/CollisionMonitor/Trace.lean#L417) | **Stale data.** Likewise with `stopSource` when no scan is in force or the one in force is too old, whatever the transform. |
| [`run_no_transform`](../../ROSlean/CollisionMonitor/Trace.lean#L437) | **No transform.** Likewise with `stopSource` when the command carries no usable transform, whatever scan is in force. |
| [`run_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Trace.lean#L450) | **Bad data.** A non-finite command leaves no trace: the run with it is the run without it, plus one `silent`. |
| [`run_no_amplification`](../../ROSlean/CollisionMonitor/Trace.lean#L461) | **No amplification.** Every velocity action published anywhere in any run is `pass`, a stop, or a slowdown with a ratio pattern below that of 1.0, configured for the zone named. |
| [`run_slowdown_bound`](../../ROSlean/CollisionMonitor/Trace.lean#L482) | **Slowdown bound**, with hypotheses on the scan in force and a slowdown zone with a fixed shape. |
| [`run_pass_through`](../../ROSlean/CollisionMonitor/Trace.lean#L503) | **Pass-through.** Node active, the scan in force not too old, a usable transform, no zone in force triggered: a finite non-zero command is published unchanged. |
| [`run_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L564), [`run_output_of_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L596), [`no_shape_of_unconfigured`](../../ROSlean/CollisionMonitor/Trace.lean#L577), [`step_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L524) | **No shape.** From a state without a shape for zone `k`, over events none of which is a shape message for zone `k`, the state still has none: an accepted shape message is the only way to give a zone a shape. After any history that leaves the node unconfigured (start-up, or a cleanup that took effect) and any such events, no output names the topic zone `k`. |
| [`run_after_activation`](../../ROSlean/CollisionMonitor/Trace.lean#L617) | **After activation**, for a history that leaves the node inactive. |

In the hypotheses of `run_stop` and `run_slowdown_bound` the zone has a
fixed shape, because such a zone is in force in every state
([`liveConfig_of_fixed`](../../ROSlean/CollisionMonitor/Trace.lean#L374)) and the statement then speaks about events only.
For a topic zone the single-step theorems apply at every point of a run,
with the shape the state holds; there is no closed description of that shape
in terms of the history (it depends on every refresh).

### `stop_pub_timeout`: when the node withholds although a stop is demanded

R15 is upstream's rule and the model reproduces it: zero is published for
`stop_pub_timeout` after it first appears, then no velocity, until a
non-zero velocity is demanded. Three theorems say exactly what that means.

| Theorem | Statement |
| --- | --- |
| [`run_withheld_last_published_zero`](../../ROSlean/CollisionMonitor/Trace.lean#L847) | **A withheld velocity follows a published zero.** In any run from start-up, if a command is processed and gets no velocity, then the velocity published last before it was a zero: an earlier command was answered with an action that yields zero for it, and no velocity was published in between ([`ZeroPublishedLast`](../../ROSlean/CollisionMonitor/Trace.lean#L750)). |
| [`zero_period`](../../ROSlean/CollisionMonitor/Trace.lean#L722) | **A period of zero.** If a processed command at time `t` is the first for which zero is demanded, no `deactivate` follows, and every velocity published afterwards is zero, then a later finite command at `now'`, with the node still active, gets the demanded action unless that is again zero and `t + stop_pub_timeout < now'`. So zeros are published from `t` up to and including `t + stop_pub_timeout`, whatever their cause, and a non-zero demand is always published. |
| [`step_zero_start`](../../ROSlean/CollisionMonitor/Trace.lean#L640), [`run_zero_streak`](../../ROSlean/CollisionMonitor/Trace.lean#L675), [`step_zero_record`](../../ROSlean/CollisionMonitor/Trace.lean#L656) | What the state records: the first zero is published and sets the stop stamp to its time; during the period `prevZero` stays set and the stop stamp does not move. Only a command and a `deactivate` touch the record. |
| [`run_withheld_after_published`](../../ROSlean/CollisionMonitor/Trace.lean#L863) | No velocity is withheld before one has been published: the first processed command after start-up always gets a velocity. |

**What the robot receives.** The statement "a demanded stop is always
published" is false, for upstream and for the model, and the theorems do not
claim it. The scenario of review finding 5 is real (it is an example in
[`Examples.lean`](../../ROSlean/CollisionMonitor/Examples.lean), checked by
`decide`): the planner sends zero commands for longer than
`stop_pub_timeout`, so the node has published zero for one second and then
stopped publishing velocities; now an obstacle enters a stop zone and the
planner starts to command 0.5 m/s. A stop is demanded, and **no velocity is
published, not once**, because the idle zeros and the stop are one period of
zero. What `run_withheld_last_published_zero` guarantees is that the last
velocity the base received is a zero (here: one of the idle zeros). A base
that holds its last command stays stopped. A base that needs fresh zero
messages to stay stopped, or that takes commands from elsewhere in the
meantime, is not protected by this node during that time; that is assumption
A3, and open decision 3 of the SPEC (publish zero for as long as a stop is
demanded) would remove it. With the extension the state topic does say
"stop" in that scenario (the report does not depend on the velocity), and a
`deactivate` followed by `activate` starts a new period
(`run_after_activation`).

The user has decided to keep upstream's behaviour here (tasks/PLAN.md,
"Decisions"). Why the theorem is therefore weaker than "a demanded stop is
published", with the sequence, how much it matters and what would
strengthen it, is written up in
[`findings/stop_pub_timeout_theorem.md`](../../findings/stop_pub_timeout_theorem.md).

The invariant needs the run to start at `State.init cfg`. From an arbitrary
state with `prevZero` set, a velocity can be withheld with nothing published
before.

## The transform

In [`Transform.lean`](../../ROSlean/CollisionMonitor/Transform.lean), about
`Xform.pointMm` and `Xform.vertexMm` of
[`CollisionMonitorXform.lean`](../../core/ROSleanCore/CollisionMonitorXform.lean)
(N25, ND1; SPEC_NAV2, section 5, option B). [`sval64`](../../ROSlean/CollisionMonitor/Transform.lean#L96) and [`sval32`](../../ROSlean/CollisionMonitor/Transform.lean#L99) are
the numbers a finite pattern denotes, with their sign; [`imageX`](../../ROSlean/CollisionMonitor/Transform.lean#L194) and
[`imageY`](../../ROSlean/CollisionMonitor/Transform.lean#L198) are the x and y, in metres, of the image of `(x, y, 0)` under a
transform by upstream's formula in exact real arithmetic ([`rotX`](../../ROSlean/CollisionMonitor/Transform.lean#L143),
[`rotY`](../../ROSlean/CollisionMonitor/Transform.lean#L149)): the first two rows of the rotation matrix of the quaternion, each
entry divided by `|q|²`, then the translation. The third row is not used:
the image is projected on the x–y plane.

| Theorem | Statement |
| --- | --- |
| [`pointMm_x_eq`](../../ROSlean/CollisionMonitor/Transform.lean#L390), [`pointMm_y_eq`](../../ROSlean/CollisionMonitor/Transform.lean#L404) | **A scan point is the exact image, truncated.** For a usable transform and finite coordinates, each coordinate of `t.pointMm x y` is the exact image in millimetres, truncated toward zero and saturated at ±1,000,000 ([`truncReal`](../../ROSlean/CollisionMonitor/Transform.lean#L277)). |
| [`pointMm_x_abs_le`](../../ROSlean/CollisionMonitor/Transform.lean#L418), [`pointMm_y_abs_le`](../../ROSlean/CollisionMonitor/Transform.lean#L424) | Hence a point never lies further from an axis than its exact image … |
| [`pointMm_x_abs_gt`](../../ROSlean/CollisionMonitor/Transform.lean#L431), [`pointMm_y_abs_gt`](../../ROSlean/CollisionMonitor/Transform.lean#L438), [`pointMm_x_sub_lt`](../../ROSlean/CollisionMonitor/Transform.lean#L447), [`pointMm_y_sub_lt`](../../ROSlean/CollisionMonitor/Transform.lean#L454) | … and unless it is saturated is less than 1 mm short of it on each axis. |
| [`vertexMm_x_sub_le`](../../ROSlean/CollisionMonitor/Transform.lean#L489), [`vertexMm_y_sub_le`](../../ROSlean/CollisionMonitor/Transform.lean#L505) | **A vertex is within half a millimetre of its exact image** on each axis, for a usable transform and finite coordinates. |
| [`pointMm_identity`](../../ROSlean/CollisionMonitor/Transform.lean#L595), [`coordMm_eq_truncReal`](../../ROSlean/CollisionMonitor/Transform.lean#L549), [`image_identity`](../../ROSlean/CollisionMonitor/Transform.lean#L542) | **Under the identity a scan point is converted as before**: for finite coordinates `Xform.identity.pointMm x y = (coordMm x, coordMm y)`. A scan in the base frame without base shift correction is treated exactly as by the node of SPEC.md. |
| [`rotX_of_unit`](../../ROSlean/CollisionMonitor/Transform.lean#L156), [`rotY_of_unit`](../../ROSlean/CollisionMonitor/Transform.lean#L163), [`rotX_smul`](../../ROSlean/CollisionMonitor/Transform.lean#L171), [`rotY_smul`](../../ROSlean/CollisionMonitor/Transform.lean#L180) | For a quaternion of unit length the exact image is the rotation it denotes (the usual matrix entries) followed by the translation; for any other usable quaternion it is that of any non-zero multiple of it, in particular of its normalisation. |
| [`usable_iff`](../../ROSlean/CollisionMonitor/Transform.lean#L207) | A transform is usable iff its six numbers are finite and its quaternion is not zero. |
| [`pointMm_inRange`](../../ROSlean/CollisionMonitor/Transform.lean#L463), [`pointMm_of_not`](../../ROSlean/CollisionMonitor/Transform.lean#L480) | A point is within ±1,000,000 whatever the transform and the patterns; if the transform is not usable or a coordinate is not finite it is `Point.far`. |
| [`contains_of_transformed`](../../ROSlean/CollisionMonitor/Transform.lean#L660), [`contains_of_image_on_boundary`](../../ROSlean/CollisionMonitor/Transform.lean#L679), [`contains_of_rounded`](../../ROSlean/CollisionMonitor/Transform.lean#L620) | **The margin covers the rounding, with transforms.** Let a polygon be received as vertices `raw` and placed by a usable transform, and a reading be placed by a usable transform without saturating. If a point of the boundary of the **exact** image of the polygon is within 1 mm per axis of the model's point (in particular if the exact image of the reading lies on that boundary), the model counts the reading as inside the zone. The analogue of `nearBoundary_of_rounded` for topic zones and scans in another frame. |

**What is proved and what is prose in "a point upstream counts is always
counted"**, for a scan in another frame and for a topic zone:

- **Proved.** Measured from the exact images: a vertex moves by at most
  1/2 mm per axis, a point by less than 1 mm per axis toward the origin, and
  under these two bounds a point whose exact image is on the boundary is
  counted (`contains_of_transformed`). For circles: truncation moves a point
  toward the origin, so `contains_circle_of_truncated` applies with the exact
  image as the real point.
- **Prose, not proved, as for fixed zones.** That the boundary passes over
  the point when a point inside the exact polygon ends up outside the
  rounded one (the topological half, see "Geometry").
- **Prose, not provable here (trusted item T7).** That the point and the
  vertices **upstream** computes, in `double`, with or without fused
  multiply-add, are within 10⁻⁶ mm of the exact images. Lean has no model
  of binary64 arithmetic in this repository; the estimate is in SPEC_NAV2,
  section 5. The budget it has to fit in is 0.114 mm.

Not proved either: that the transform handed over is the right one (T6),
and that `sval64` is IEEE-754's reading of a pattern (it is `val64` of
`Convert.lean` with the sign, see "The bit-pattern helpers").

## Geometry

In [`Geometry.lean`](../../ROSlean/CollisionMonitor/Geometry.lean).

| Theorem | Statement |
| --- | --- |
| [`contains_circle_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L36) | A point is inside a circle zone iff `x² + y² < r²`, strictly (R9). |
| [`contains_circle_of_truncated`](../../ROSlean/CollisionMonitor/Geometry.lean#L48) | **Safe direction, circle.** A real point strictly inside the circle of real radius `R ≥ 0` is counted, if the model's radius is at least `R` and the model's point is no further from each axis (truncation toward zero). |
| [`crosses_iff_upstream`](../../ROSlean/CollisionMonitor/Geometry.lean#L105) | The crossing test with the division multiplied out is upstream's test `a.x + (p.y − a.y)·(b.x − a.x)/(b.y − a.y) > p.x` together with its half-open straddle condition, evaluated in exact rational arithmetic (R10, D3). |
| [`contains_polygon_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L141) | A point is inside a polygon zone iff an odd number of edges is crossed (upstream's rule on the integer polygon) or it is within the margin of some edge. In particular every point upstream's rule puts inside the integer polygon is counted. |
| [`nearSegment_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L160) | **The margin is a distance.** The margin test for an edge holds iff some point of the closed segment (real parameter in `[0, 1]`) is at squared distance at most 5 mm² from the point: at most √5 mm from the segment. |
| [`nearBoundary_of_rounded`](../../ROSlean/CollisionMonitor/Geometry.lean#L321), [`nearSegment_of_close`](../../ROSlean/CollisionMonitor/Geometry.lean#L275) | **The margin covers the rounding.** Take a polygon with real vertices, any rounding that moves each vertex by at most 1/2 mm per axis, a point `q` on the real polygon's boundary, and an integer point within 1 mm per axis of `q`. Then the model counts that integer point as inside the rounded polygon. (`2·(1/2 + 1)² = 4.5 ≤ 5`.) |
| [`insidePolygon_box`](../../ROSlean/CollisionMonitor/Geometry.lean#L547) | For an axis-parallel box, every point strictly inside is inside by the crossing rule. |
| [`contains_polygon_eq_false_of_beyond`](../../ROSlean/CollisionMonitor/Geometry.lean#L482) | A point at least 3 mm beyond every vertex along one axis is not counted: a polygon zone never reaches outside its bounding box grown by 3 mm. |
| [`config_ok_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L586), [`shape_ok_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L566) | **What `Config.ok` guarantees:** every fixed circle has radius at most 1,000,000 mm; every fixed polygon has at least four vertices, each within 999,997 mm of both axes; every slowdown ratio has its sign bit clear. A topic zone has no shape in the configuration. |
| [`contains_eq_false_of_far'`](../../ROSlean/CollisionMonitor/Geometry.lean#L623), [`contains_eq_false_of_far`](../../ROSlean/CollisionMonitor/Geometry.lean#L659), [`ShapeBounded`](../../ROSlean/CollisionMonitor/Geometry.lean#L597) | **A saturated point is in no zone.** If a shape lies within the coordinate limit (a circle of radius at most 1,000,000 mm, a polygon whose vertices are within 999,997 mm of both axes; `Shape.ok` implies it, and so does the check on a received shape), a point with a coordinate of magnitude 1,000,000 mm or more is not inside it (D12). The SPEC relies on this for readings that are NaN, infinite or 1000 m away. |
| [`zonesAction_filter_inRange'`](../../ROSlean/CollisionMonitor/Geometry.lean#L692), [`zonesAction_filter_inRange`](../../ROSlean/CollisionMonitor/Geometry.lean#L711) | If every zone lies within the coordinate limit, in particular with an accepted configuration, saturated points do not change the action. |

### The safe-direction claim for polygons: what is proved and what is prose

The SPEC claims: every point that upstream's rule, in exact arithmetic, puts
inside the polygon as configured is counted by the model, after the vertices
are rounded to the nearest millimetre and the point is truncated.

- **Proved.** (1) If the truncated point is inside the rounded polygon by
  upstream's rule, it is counted (`contains_polygon_iff`). (2) If, for any
  polygon whose vertices are within 1/2 mm per axis of the rounded ones, a
  point of its boundary is within 1 mm per axis of the truncated point, the
  truncated point is counted (`nearBoundary_of_rounded`).
- **Prose, not proved.** That one of these two cases always applies:
  if upstream's rule puts the real point inside the real polygon and the
  truncated point outside the rounded polygon, then, moving every vertex
  and the point in straight lines from their real positions to their rounded
  ones, at some moment the moving point lies on an edge of the moving
  polygon. That moment's configuration satisfies the hypotheses of (2). The
  missing step is that the parity of crossings cannot change while the point
  stays off the boundary. It needs a definition of upstream's rule over the
  reals and a continuity argument; neither is in this repository.

The margin argument of the SPEC was checked against these theorems and no
error was found in it: the SPEC's Euclidean bound 0.71 + 1.42 = 2.13 < 2.24
and the per-axis bound used here (1.5 mm on each axis, squared distance
4.5 ≤ 5) are the same fact. The slack between 4.5 and 5 is what covers the
floating-point error of the two multiplications by 1000 (T2, T3).

Also not proved: the "other direction" figures of the SPEC (a point up to
3.5 mm outside a circle, 4.4 mm outside a polygon, can count as inside).

## The bit-pattern helpers

In [`Bits.lean`](../../ROSlean/CollisionMonitor/Bits.lean). The model
compares and classifies IEEE-754 bit patterns without ever computing with
them. The theorems relate the helpers to
[`magVal`](../../ROSlean/CollisionMonitor/Bits.lean#L35), the magnitude a
sign-less pattern denotes, as a whole number of units of the smallest
subnormal (IEEE-754's formula: the fraction for exponent field 0, otherwise
`(2^w + fraction) · 2^(exponent − 1)`).

| Theorem | Statement |
| --- | --- |
| [`magVal_strictMono`](../../ROSlean/CollisionMonitor/Bits.lean#L39) | Magnitudes are ordered as their patterns, for any fraction width. |
| [`le_of_isNaN_left`](../../ROSlean/CollisionMonitor/Bits.lean#L109), [`le_of_isNaN_right`](../../ROSlean/CollisionMonitor/Bits.lean#L113) | `F32.le` is false whenever either side is NaN. |
| [`le_iff_value`](../../ROSlean/CollisionMonitor/Bits.lean#L150) | For two patterns that are not NaN, `F32.le a b` iff the number denoted by `a` is at most that denoted by `b` ([`f32Value`](../../ROSlean/CollisionMonitor/Bits.lean#L99): the magnitude with the sign). Includes `-0 <= +0`, `+0 <= -0` and the infinities. |
| [`le_iff_of_le_inf`](../../ROSlean/CollisionMonitor/Bits.lean#L130), [`le_iff_of_nonneg`](../../ROSlean/CollisionMonitor/Bits.lean#L123) | Non-negative patterns (up to that of +∞) are ordered as natural numbers. |
| [`isFinite_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L206) | `F64.isFinite b` iff `b` without its sign bit is below the pattern of +∞. |
| [`isZero_iff_of_lt`](../../ROSlean/CollisionMonitor/Bits.lean#L216), [`isZero_iff_magVal`](../../ROSlean/CollisionMonitor/Bits.lean#L222) | A 64-bit pattern is a zero iff it is that of `+0.0` or `-0.0`; iff the magnitude it denotes is 0. |
| [`magVal_one`](../../ROSlean/CollisionMonitor/Bits.lean#L233), [`lt_one_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L239) | `F64.one` denotes 1.0, and `r < F64.one` iff `r` has its sign bit clear and denotes a magnitude below 1.0. |
| [`finite_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L260), [`planarZero_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L272) | `Twist.finite` and `Twist.planarZero` written out per component. |

**Trusted, not proved:**

- that `magVal` and `f32Value` are the right reading of a pattern (they are
  short and were written from the standard; `f32Value` gives an infinity the
  value just above the largest finite number, which is what an order needs);
- that the C++ compiler's `float` and `double` are IEEE-754 binary32 and
  binary64 and that `memcpy` into an integer yields the pattern;
- that upstream's own `<=`, `isfinite` and `== 0.0` are the IEEE operations.

The reviewer's `review/collision_monitor/Checks.lean` compares the helpers
with the hardware operations on some 300,000 patterns each (0
disagreements); that is a test and complements the theorems on exactly the
trusted points.

Bits above the format's width: `F32.le`, `F64.isFinite` and `F64.isZero`
ignore them. A slowdown ratio is compared as a natural number, so a pattern
of 2⁶⁴ or more is never selected (it is not below `F64.one`); `Config.ok`
refuses it.

## Supporting results

| Theorem | Statement |
| --- | --- |
| [`firstStop_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L214), [`firstStop_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L234) | The stop loop returns the first triggered stop zone, or nothing iff none is triggered. |
| [`bestSlowdown_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L411), [`bestSlowdown_snd_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L429) | The slowdown loop returns the best slowdown zone, or nothing iff no slowdown zone with a ratio below 1 is triggered. |
| [`zonesAction_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L453), [`zonesAction_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L464), [`zonesAction_eq_pass_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L476) | The action for given points, by cases. |
| [`zonesAction_congr`](../../ROSlean/CollisionMonitor/Basic.lean#L529) | The action depends on the points only through which zones they trigger. |
| [`zonesAction_eq_of_map`](../../ROSlean/CollisionMonitor/Basic.lean#L497) | More generally, the action depends on the zones and the points only through the action of each zone, in order, and whether it is triggered. |
| [`triggered_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L46), [`triggered_nil_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L69), [`not_triggered_of_topic`](../../ROSlean/CollisionMonitor/Basic.lean#L58) | A zone triggers iff it is enabled, has a shape, and has at least `minPoints` points inside; without points, only a zone with `minPoints = 0` triggers (R11); a zone without shape never triggers (N32). |
| [`withShapes_getElem?`](../../ROSlean/CollisionMonitor/Basic.lean#L573), [`withShape_fields`](../../ROSlean/CollisionMonitor/Basic.lean#L540), [`withShape_of_fixed`](../../ROSlean/CollisionMonitor/Basic.lean#L547), [`withShape_of_topic`](../../ROSlean/CollisionMonitor/Basic.lean#L563), [`withShape_none`](../../ROSlean/CollisionMonitor/Basic.lean#L555) | The zones with their shapes filled in: zone `k` is the configured zone `k` with entry `k` of the shapes; only the shape differs; a fixed zone is unaffected; a topic zone with a shape is the polygon of its current vertices, and without one stays without. |
| [`evaluate_congr`](../../ROSlean/CollisionMonitor/Properties.lean#L60) | The demanded action reads the state only through the stored scan and the shapes. |

## The C interface and the conversions

The node calls the exported wrappers of
`core/ROSleanCore/CollisionMonitorFFI.lean`, and the shim hands every
parameter and every scan coordinate over as a bit pattern, to be converted in
Lean (`core/ROSleanCore/CollisionMonitorConvert.lean`). The theorems are in
[`FFI.lean`](../../ROSlean/CollisionMonitor/FFI.lean) and
[`Convert.lean`](../../ROSlean/CollisionMonitor/Convert.lean).

| Theorem | Statement |
| --- | --- |
| **End to end** | |
| [`cm_readBack_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1092), [`cm_out_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1078), [`cm_readBack_at`](../../ROSlean/CollisionMonitor/FFI.lean#L1115) | Stated on the exported functions in the order the node calls them. Create a monitor from a builder that is ok (`cmNew`), make any sequence of calls of `cmOnScan` and `cmOnCmd` ([`Call`](../../ROSlean/CollisionMonitor/FFI.lean#L874); scans usable or not, no condition on the sequence), then deliver a command with `cmOnCmd`: the kind, zone index and ratio that `cmOutKind`, `cmOutZone` and `cmOutRatio` return determine, and are the codes of, the **velocity** of the output of `step` for that command's event, in the state `run` reaches on the events the calls deliver ([`events`](../../ROSlean/CollisionMonitor/FFI.lean#L897)), from the state after `configure` and `activate` ([`startState`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L234), [`startState_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L232)), on the builder's configuration. `cm_readBack_at` says the same of every call in the sequence. With [`run_startState`](../../ROSlean/CollisionMonitor/FFI.lean#L238) this makes every theorem above a statement about the running node, with `es := [.configure, .activate] ++ events true cs`. |
| [`cm_not_ok_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1140), [`foldl_exec_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L967), [`apply_cmd_of_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L809), [`exec_st_of_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L956) | **Fail closed.** If the configuration was refused, the answer to every command, after any calls whatever, is `stopSource`: also for a non-finite command, and without the withholding of R15. (In the active phase, in which every monitor of `cmNew` is and stays.) |
| [`readBack_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1025), [`actionOfCodes_codes`](../../ROSlean/CollisionMonitor/FFI.lean#L996), [`step_publish_stopZone`](../../ROSlean/CollisionMonitor/FFI.lean#L1004), [`step_publish_slowdown`](../../ROSlean/CollisionMonitor/FFI.lean#L1012) | Nothing is truncated on the way to C: a published zone index is below the number of zones (at most 2³² assumed), and a published ratio is below the pattern of 1.0. Kind 0 means "no velocity": `silent`, or a processed command whose velocity is withheld. |
| **Events** | |
| [`cmOnCmd_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L838), [`cmOnScan_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L827), [`exec_eq_apply`](../../ROSlean/CollisionMonitor/FFI.lean#L907), [`apply_of_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L800), [`foldl_apply`](../../ROSlean/CollisionMonitor/FFI.lean#L862), [`foldl_exec_run`](../../ROSlean/CollisionMonitor/FFI.lean#L941), [`foldl_exec_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L927) | Each of the two event calls is `step` on the event it delivers, and every call delivers one; a sequence of calls is `run` on those events, with no hypothesis about the calls. A scan call delivers a `scan` event and records whether the scan was reported usable; a command call delivers a `cmd` event with the identity transform for the scan if the scan stored last was reported usable and with no transform if it was not, and with no zone transforms ([`cmdEvent`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L329), [`Call.event`](../../ROSlean/CollisionMonitor/FFI.lean#L887)). |
| [`cmOnScan_unusable_stop`](../../ROSlean/CollisionMonitor/FFI.lean#L849) | After a scan call with `usable` 0 (wrong frame, no sensor pose: D11) commands carry no transform, and with an enabled source and a non-zero timeout the demanded action is `stopSource` in every state (N27), until a scan is reported usable. This is what the removed event `scanUnusable` did; over runs, `run_no_transform`. |
| **Configuration and refusals** | |
| [`builderAddCircle_zones`](../../ROSlean/CollisionMonitor/FFI.lean#L121), [`builderAddPolygon_zones`](../../ROSlean/CollisionMonitor/FFI.lean#L135), [`foldl_addVertex`](../../ROSlean/CollisionMonitor/FFI.lean#L188), [`zoneOfRaw_fields`](../../ROSlean/CollisionMonitor/FFI.lean#L107), [`builderSetSource_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L86), [`builderSetStopPubTimeout_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L93) | The configuration is what the builder calls say: zones in the order added, a polygon's vertices in the order of its `add_vertex` calls, each value the conversion of its argument. |
| [`cmOk_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L283), [`cmOk_cfg_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L292), [`error_eq_zero_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L262), [`builderNew_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L79) | A monitor is ok iff the source was set, no timeout was refused, no vertex was given to no polygon, no zone carries a refused value, and every zone passes `Zone.ok`; so `Config.ok` holds for it, which is the hypothesis of "a saturated point is in no zone" (`contains_eq_false_of_far`). A new builder is not ok. No builder call makes a topic zone yet (T06). |
| [`not_ok_of_refused_call`](../../ROSlean/CollisionMonitor/FFI.lean#L506) | **No refused value is dropped, wherever it occurs.** Take any builder and any sequence of the five builder calls ([`BuilderCall`](../../ROSlean/CollisionMonitor/FFI.lean#L365)). If any one call carries a refused value ([`BuilderCall.Refused`](../../ROSlean/CollisionMonitor/FFI.lean#L383): a timeout of `set_source` or `set_stop_pub_timeout`, a radius, a `min_points`, a vertex with a coordinate that is not finite), the monitor made from the result is not ok, whatever the calls before and after it. In particular a refused vertex makes the monitor not ok also when the zone added last is a circle or when there is no zone, and no later call (a second `set_source` with a good value, further vertices) clears a refusal. |
| [`not_ok_of_stray_vertex`](../../ROSlean/CollisionMonitor/FFI.lean#L517), [`not_ok_without_set_source`](../../ROSlean/CollisionMonitor/FFI.lean#L544) | A vertex, refused or not, that is added when the zone added last is not a polygon makes the monitor not ok. A monitor made from a new builder by calls among which there is no `set_source` is not ok: the source timeout is never 0, "check off", by omission. |
| [`exec_marked_of_refused`](../../ROSlean/CollisionMonitor/FFI.lean#L467), [`exec_marked_mono`](../../ROSlean/CollisionMonitor/FFI.lean#L445), [`not_ok_of_marked`](../../ROSlean/CollisionMonitor/FFI.lean#L397), [`not_ok_of_source_timeout`](../../ROSlean/CollisionMonitor/FFI.lean#L310), [`not_ok_of_stop_pub_timeout`](../../ROSlean/CollisionMonitor/FFI.lean#L321), [`entryError_addCircle_ne_zero`](../../ROSlean/CollisionMonitor/FFI.lean#L327), [`foldl_addVertex_refused`](../../ROSlean/CollisionMonitor/FFI.lean#L557), [`not_ok_of_entry`](../../ROSlean/CollisionMonitor/FFI.lean#L299), [`vertexOfRaw_eq_none_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L594) | The steps of the two rows above: each refused value leaves a mark on the builder, no call removes a mark, and a marked builder is not ok. Within one polygon the mark is a code on that polygon, which is how the node names the parameter. |
| **Scan decoding** | |
| [`decodeBeams_length`](../../ROSlean/CollisionMonitor/FFI.lean#L648), [`getElem_decodeBeams`](../../ROSlean/CollisionMonitor/FFI.lean#L652), [`beamAt_lt`](../../ROSlean/CollisionMonitor/FFI.lean#L642) | A byte array of any length is a scan of `size / 20` readings; every decoded pattern fits its field, whatever the bytes. |
| [`decodeBeams_encodeBeams`](../../ROSlean/CollisionMonitor/FFI.lean#L732) | Decoding inverts the shim's encoding: the model sees the ranges and the two coordinate patterns as written, unconverted. They become points when a command is evaluated, under that command's transform (`Xform.pointMm`); under the identity that is `coordMm` (`pointMm_identity`). |
| **Conversions** (against the number a bit pattern denotes, `val64`, `val32`) | |
| [`coordMm_abs_le`](../../ROSlean/CollisionMonitor/Convert.lean#L232), [`coordMm_abs_gt`](../../ROSlean/CollisionMonitor/Convert.lean#L243), [`coordMm_sign`](../../ROSlean/CollisionMonitor/Convert.lean#L254), [`coordMm_natAbs_le`](../../ROSlean/CollisionMonitor/Convert.lean#L221) | A scan coordinate in millimetres is never farther from zero than the true value, is less than 1 mm short of it unless it saturates, keeps its sign, and is within ±1,000,000. |
| [`radiusMm_ge`](../../ROSlean/CollisionMonitor/Convert.lean#L298), [`radiusMm_lt`](../../ROSlean/CollisionMonitor/Convert.lean#L306), [`radiusMm_abs`](../../ROSlean/CollisionMonitor/Convert.lean#L268), [`radiusMm_le_limit`](../../ROSlean/CollisionMonitor/Convert.lean#L289), [`radiusMm_of_not_finite`](../../ROSlean/CollisionMonitor/Convert.lean#L264) | The model's radius is at least the configured magnitude plus 1 mm and less than that plus 2 mm; the sign of the parameter does not matter (R9, review finding 1); NaN, infinities and too large values are refused. |
| [`circle_safe`](../../ROSlean/CollisionMonitor/Convert.lean#L319) | **Circles are safe**: a point strictly inside the configured circle, in exact arithmetic on the values the shim passed, is inside the model's circle. This is the SPEC's safe-direction claim for circles, with the conversions included. |
| [`vertexMm_abs_sub_le`](../../ROSlean/CollisionMonitor/Convert.lean#L345), [`vertexMm_isSome_iff`](../../ROSlean/CollisionMonitor/Convert.lean#L340) | A vertex coordinate moves by at most half a millimetre; it is refused iff it is not finite. |
| [`timeoutNs_of_neg`](../../ROSlean/CollisionMonitor/Convert.lean#L375), [`timeoutNs_of_not_finite`](../../ROSlean/CollisionMonitor/Convert.lean#L371), [`timeoutNs_of_huge`](../../ROSlean/CollisionMonitor/Convert.lean#L383), [`timeoutNs_isSome`](../../ROSlean/CollisionMonitor/Convert.lean#L398), [`sourceTimeoutNs_eq_none`](../../ROSlean/CollisionMonitor/Convert.lean#L411) | A timeout is accepted only if it is finite, not negative and below 9.2·10⁹ s; so a refused timeout cannot become 0, "check disabled" (review finding 7a). |
| [`minPointsOfRaw_eq_some_iff`](../../ROSlean/CollisionMonitor/Convert.lean#L431) | `min_points` is accepted iff it is in the range of a 32-bit `int` (review finding 13). |
| [`mant_shift_eq_magVal`](../../ROSlean/CollisionMonitor/Convert.lean#L119) | The reading of a binary64 pattern used for the conversions is `magVal` of `Bits.lean`. |

Two limits of these theorems:

- **Timeouts are tested, not proved, to be upstream's.** `timeoutNs`
  reproduces `(int64_t)(seconds * 1e9)` by rounding an exact product to the
  nearest binary64 value in integer arithmetic (`nearest`). That this is the
  IEEE-754 product is checked against `rclcpp::Duration` and against the
  hardware on 530,000 values with no disagreement, and not proved: there is
  no formal IEEE-754 multiplication here to prove it against. What is proved
  is which timeouts are refused.
- **Whether a command carries a transform is decided by one byte from the
  shim.** The end-to-end theorems hold for every sequence of calls. Whether a
  scan *is* usable, that is, the comparison of its `frame_id` with the base
  frame and whether a sensor pose is configured, is made in C++ and is
  trusted (T5); so is the sensor pose arithmetic of D11, which the present
  node still performs before it hands the coordinates over. With T06 the
  byte and the pose go, and the transform comes from tf2 (T6).
- **The interface is the old one.** No call delivers a lifecycle event, a
  shape message, a transform or zone transforms, and none returns the
  report or tells `silent` from a withheld velocity. Those are the calls of
  SPEC_NAV2.md, section 8, and task T06. The theorems of the extension are
  about the model; they become statements about a node when that interface
  exists.

## What is not proved

- **That the model equals upstream `nav2_collision_monitor`.** The rules were
  read from the source and tested by the reviewer; the known differences are
  D1–D14 of the SPEC and ND1–ND12 of SPEC_NAV2.
- **The slowdown multiplication** (T1), as said above. No theorem speaks
  about a published velocity value other than "unchanged" and "zero", and
  those two only through the shim's contract.
- **That a transform is the right one** (T6): tf2's tree, interpolation and
  product, and the shim's lookup. The theorems hold for whatever transform
  each command carries, and for none.
- **That upstream's floating-point image of a point is within 10⁻⁶ mm of
  the exact one** (T7), on which "a point upstream counts is always counted"
  rests for scans in another frame and for topic zones. See "The
  transform".
- **The C++ shim** (T2, T3, T5): the scan's points in metres in the scan
  frame (`cos`, `sin` and which library entry the compiler calls for them,
  the angle), the parsing of the `points` text, the clock, and that the shim
  passes values unaltered and publishes what the output says. The
  conversions to millimetres and nanoseconds, the refusals and the
  application of the transform are not in the shim.
- **The topological half of the polygon safe-direction claim**, and the
  "other direction" tolerances (see Geometry).
- **That `magVal` is IEEE-754** (see the bit-pattern helpers), and likewise
  the reading of a pattern as a number that `Xform.pointMm` rests on.
- **That a demanded stop is always published.** It is not (R15); see "What
  the robot receives". Likewise the node publishes nothing when no command
  arrives (R1), nothing for a non-finite command (R2), and nothing while it
  is not active (N4), also while a stop is demanded (review finding 14).
- **That a topic zone has a shape, or a current one.** A stop zone fed by a
  topic takes no action until its first accepted message (N32, "No shape"),
  and a shape in another frame is where that frame was at the last command
  that carried its transform (N34). Both are upstream's behaviour and both
  are fail-open.
- **That a missing transform stops the robot when `source_timeout` is 0.**
  It does not (N27; `sourcePoints_of_no_transform_timeout_zero`).
- **That an accepted configuration protects anything.** `Config.ok` accepts
  no zones, a circle of radius 0, a degenerate polygon (review finding 15),
  and a configuration whose only zone is a topic zone.
  The node warns about the first three at start-up (and about a source
  timeout of 0, a disabled source, `min_points` of 0 or less and a slowdown
  ratio of 1 or more); it does not refuse them.
- **That `Xform.identity.vertexMm` equals the conversion `vertexMm` of a
  fixed polygon's vertices.** Both round the same number to the nearest
  millimetre, a half away from zero, and each is proved to be within 1/2 mm
  of it; that they are the same integer was not proved.
- **That time moves forward.** No theorem assumes it. If the clock jumps
  back, a stale scan counts as fresh and zeros keep being published, as the
  theorems state.
- **Anything about the time spent waiting for a transform** (N26), the bond,
  or the lifecycle manager: the model has the outcome of each lookup, and
  the transitions as events.
- **Anything about concurrent callbacks.** The model handles one event at a
  time.
