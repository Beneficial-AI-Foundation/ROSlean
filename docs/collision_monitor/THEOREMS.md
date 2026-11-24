# Collision monitor: the main theorems

What is proved about the Lean core of the collision monitor, in plain words,
with a link to each statement. The theorems are in
`ROSlean/CollisionMonitor/`, are checked by `lake build` in the repository
root, and use no `sorry`, no `native_decide` and no axioms beyond Lean's
three standard ones (`propext`, `Classical.choice`, `Quot.sound`; checked
with `#print axioms` on 71 theorems, listed in
[`tasks/reports/T05b.md`](../../tasks/reports/T05b.md), and on the 52 of
`NoClock.lean`, `Fast.lean` and the changed ones of `FFI.lean`, listed in
[`tasks/reports/T03.md`](../../tasks/reports/T03.md): the characterisations
of each output and action, the properties, the lifecycle, shape, margin
and trace theorems, and the end-to-end theorems of the C interface; the
sources contain no `axiom` declaration). Rule numbers
(R…), differences (D…), assumptions (A…) and trusted parts (T…) refer to
[SPEC.md](SPEC.md), and so do N… and ND… since task T09 made it one
document again; E1–E8 are the events of [SPEC_NAV2.md](SPEC_NAV2.md),
section 3.2.

**Added and changed by task T09** (the fixes after the review CM5): the
mark of an unusable polygon message (ND6: "The Nav2 extension", row
"Unusable message", and `run_stop_after_unusable` under "Over sequences of
events"); what holds from start-up after a command without a clock reading
(`noClock_unreachable`, `mixed_from_init`: "A command without a clock
reading"); the statements of `acceptShape_eq_some_iff`, `withShape_fields`,
`withShape_of_topic`, `withShapes_action`, `liveConfig_action`,
`evaluate_stopZone_only_if` and `evaluate_slowdown_only_if`, which gained
the case of the mark. `#print axioms` on the 33 new and changed theorems and on seven unchanged headline theorems that rest on the changed model (`stepCmdFast_eq` among them):
the three standard axioms, listed in
[`tasks/reports/T09.md`](../../tasks/reports/T09.md).

The model contains the Nav2 extension of SPEC_NAV2.md, sections 3 and 4: the
lifecycle, the points and vertices that arrive with each command, zones
whose shape arrives on a topic, and the report for the state topic. Since task T06 the C
interface and the node have it too; see "The C interface and the
conversions" for the theorems that tie the exported calls to the model.

**The sensor-to-base transform is not in the model** (the user's decision
for option A of SPEC_NAV2.md, section 5; task T05b). The C++ shim applies it
in `double`, as upstream does, and the model receives coordinates in the
base frame. Task T05 had built the transform into the model in exact
arithmetic and proved that every scan point is the exact image of its
reading, truncated; those theorems are gone, and "What T05 proved and
option A does not have" below says what replaces each.

Links point to line numbers as of 2026-11-24; if a link lands a few lines
off, search the file for the theorem name.

| File | Content |
| --- | --- |
| [`Basic.lean`](../../ROSlean/CollisionMonitor/Basic.lean) | Which points are tested, which zone is selected, the zones with their shapes filled in |
| [`Properties.lean`](../../ROSlean/CollisionMonitor/Properties.lean) | One command: the output, the state, the properties of the SPEC |
| [`Lifecycle.lean`](../../ROSlean/CollisionMonitor/Lifecycle.lean) | The other events: lifecycle transitions, shape messages; the invariants of reachable states |
| [`Trace.lean`](../../ROSlean/CollisionMonitor/Trace.lean) | Event sequences: the trace invariant, the history, when nothing is published, `stop_pub_timeout` |
| [`Geometry.lean`](../../ROSlean/CollisionMonitor/Geometry.lean) | Circle and polygon tests, the margin, saturated points, `Config.ok` |
| [`Margin.lean`](../../ROSlean/CollisionMonitor/Margin.lean) | The conversions of the base-frame coordinates a command carries; the polygon margin with room for the shim's floating-point transform |
| [`Bits.lean`](../../ROSlean/CollisionMonitor/Bits.lean) | The IEEE-754 bit-pattern helpers |
| [`Convert.lean`](../../ROSlean/CollisionMonitor/Convert.lean) | The conversions at the boundary: millimetres, nanoseconds, refusals |
| [`FFI.lean`](../../ROSlean/CollisionMonitor/FFI.lean) | The C interface computes the model; refused configurations stop every command |
| [`NoClock.lean`](../../ROSlean/CollisionMonitor/NoClock.lean) | A command without a valid clock reading (D15): always a stop |
| [`Fast.lean`](../../ROSlean/CollisionMonitor/Fast.lean) | The command path that C calls computes the same function as `step` |

## What the theorems are about

The model is in
[`core/ROSleanCore/CollisionMonitor.lean`](../../core/ROSleanCore/CollisionMonitor.lean)
and, for bit patterns, points and the two conversions the model applies,
[`core/ROSleanCore/CollisionMonitorPoint.lean`](../../core/ROSleanCore/CollisionMonitorPoint.lean).

| Name | Meaning |
| --- | --- |
| [`step`](../../core/ROSleanCore/CollisionMonitor.lean#L649) | Processes one event. Returns the new state and the output. Total. |
| [`Event`](../../core/ROSleanCore/CollisionMonitor.lean#L229) | `configure`, `activate`, `deactivate`, `cleanup`, `shutdown` (E1–E5); `scan now s` (E6); `shape k verts foreign`, a polygon message for zone `k` with its vertices in the base frame, or `none` if its transform was not found (E7); `cmd now v scanPts zoneVerts`, a velocity command with the base-frame coordinates of the readings of the stored scan as the shim computed them for this command (or `none`: no transform), and for each zone the base-frame vertices of its stored message (or `none`) (E8). A missing transform is not an event. Coordinates are pairs of binary64 patterns ([`Coords`](../../core/ROSleanCore/CollisionMonitor.lean#L190)). |
| [`run`](../../core/ROSleanCore/CollisionMonitor.lean#L715) | `step` over a list of events. |
| [`State.init`](../../core/ROSleanCore/CollisionMonitor.lean#L279) | The state at start-up, for a configuration: unconfigured, no scan, no shape. |
| [`Phase`](../../core/ROSleanCore/CollisionMonitor.lean#L211) | `unconfigured`, `inactive`, `active`, `finalized`. |
| [`Output`](../../core/ROSleanCore/CollisionMonitor.lean#L591) | `silent` (nothing is published, on any topic), or `answer velocity report` for a processed command: `velocity` is the action to publish or `none` if it is withheld (R15); `report` is the state message or `none`. `answer none none` differs from `silent`: the zone polygons are published (N16). [`Output.velocity`](../../core/ROSleanCore/CollisionMonitor.lean#L604) and [`Output.report`](../../core/ROSleanCore/CollisionMonitor.lean#L609) read the two parts and are `none` for `silent`. |
| [`Action`](../../core/ROSleanCore/CollisionMonitor.lean#L477) | `pass` (command unchanged), `slowdown k r` (multiply by the double with bit pattern `r`; zone `k`), `stopZone k` (zero; zone `k`), `stopSource` (zero; the source is invalid). |
| [`Report`](../../core/ROSleanCore/CollisionMonitor.lean#L220) | `normal`, `stop k`, `slowdown k`, `source`: what a state message says (N13). [`Action.report`](../../core/ROSleanCore/CollisionMonitor.lean#L583): what an action reports for a command. |
| [`evaluate`](../../core/ROSleanCore/CollisionMonitor.lean#L559) | The action demanded at a given time with given coordinates. It does not depend on the command's velocity. |
| [`sourcePoints`](../../core/ROSleanCore/CollisionMonitor.lean#L506) | The points the zones are tested against, or `none` if the source is invalid. |
| [`Scan.points`](../../core/ROSleanCore/CollisionMonitor.lean#L301), [`Scan.fits`](../../core/ROSleanCore/CollisionMonitor.lean#L293) | `s.points c`: the points of the scan's valid readings, given one coordinate pair per reading: reading `i` is paired with pair `i`, pairs of readings that are not valid (R4) are dropped, the others converted to millimetres. `s.fits c`: there is exactly one pair per reading. |
| [`liveConfig`](../../core/ROSleanCore/CollisionMonitor.lean#L553) | The zones as they are tested for a command: the configured zones, each topic zone with the shape the state holds for it, refreshed with the command's zone vertices (N34). |
| [`Zone.triggered`](../../core/ROSleanCore/CollisionMonitor.lean#L376) | The zone is enabled, has a shape, and at least `minPoints` points are inside it. |
| [`scanPointMm`](../../core/ROSleanCore/CollisionMonitorPoint.lean#L149), [`vertexPointMm`](../../core/ROSleanCore/CollisionMonitorPoint.lean#L155) | A scan point from its base-frame coordinates: each truncated toward zero to whole millimetres and saturated at ±1,000,000 ([`coordMm`](../../core/ROSleanCore/CollisionMonitorPoint.lean#L120)). A vertex of a topic zone: each coordinate rounded to the nearest millimetre ([`vertexCoordMm`](../../core/ROSleanCore/CollisionMonitorPoint.lean#L139)). Integer arithmetic on the bit patterns. |
| [`Action.yieldsZero`](../../core/ROSleanCore/CollisionMonitor.lean#L575) | The velocity the action yields for a command is zero: a stop, or a zero command, or a ratio of zero. |
| `prevZero`, `stopStamp`, `prevReport` | The state fields behind `stop_pub_timeout` (R15) and the state topic (N13). |

"The output of a command" below is what `step` returns for a command in a
state `st`. Unless said otherwise the state is arbitrary, nothing is assumed
about the clock, and nothing is assumed about the configuration (`Config.ok`
is needed only where stated).

Two predicates name the zone that wins:
[`IsFirstStop`](../../ROSlean/CollisionMonitor/Basic.lean#L182) `zones pts k`
(zone `k` is a triggered stop zone and no stop zone before it is triggered)
and
[`IsBestSlowdown`](../../ROSlean/CollisionMonitor/Basic.lean#L261) `zones pts k r`
(zone `k` is a triggered slowdown zone with ratio pattern `r` below that of
1.0, no triggered slowdown zone has a smaller ratio pattern, and none before
`k` has the same). Each holds for at most one `k`
([`IsFirstStop.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L190), [`IsBestSlowdown.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L270)),
which is the answer to "which zone is named when several trigger". The zones
are always those in force, `(liveConfig cfg st zoneVerts).zones`.

Two more name the source:
[`SourceInvalid`](../../ROSlean/CollisionMonitor/Basic.lean#L98) (the source is enabled, its timeout is not 0, and no scan
is stored, or the stored scan's stamp is more than the timeout before `now`,
or the command carries no usable coordinates for it) and
[`NoScanPoints`](../../ROSlean/CollisionMonitor/Basic.lean#L79) (the command carries no coordinates for the scan, or a
list that does not have one pair per reading of the stored scan).

## The two central theorems

**When a command is processed, and what is then published.**

| | Theorem | Statement |
| --- | --- | --- |
| Processed | [`processed_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L53), [`output_eq_silent_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L274) | The output of an event is not `silent` **iff** the event is a command, the node is active, and the command has no infinite or NaN component. Every other event, and every event in another phase, gives `silent`. |
| Velocity | [`velocity_eq_some_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L328), [`velocity_eq_none_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L356), [`velocity_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L288) | The output carries the velocity action `a` **iff** the command is processed, `a` is the action `evaluate` demands, and it is not the case that all three hold: the velocity `a` yields is zero, the velocity demanded for the previous command was zero, and the stop stamp is more than `stop_pub_timeout` before `now` (R15). In that case, and for a command that is not processed, there is no velocity. Nothing else is ever published ([`velocity_eq_none_or`](../../ROSlean/CollisionMonitor/Properties.lean#L373)). |
| Report | [`report_eq_some_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L390), [`report_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L302), [`step_cmd_prevReport`](../../ROSlean/CollisionMonitor/Properties.lean#L252) | The output carries the report `r` **iff** the command is processed, `r` is what the demanded action reports for it, and `r` differs from the previous report. The new previous report is what the action reports, in both cases. Whether the velocity is published or withheld does not enter: a report is made also when the velocity is withheld. |
| Whole | [`step_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L195), [`output_cmd`](../../ROSlean/CollisionMonitor/Properties.lean#L313) | The new state and the output of a processed command, in one equation. |

**Which action is demanded.** Four if-and-only-if statements, one per action:

| Action | Demanded iff | Theorem |
| --- | --- | --- |
| `stopSource` | the source is invalid (`SourceInvalid`) | [`evaluate_eq_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L77) |
| `stopZone k` | the source is valid and `k` is the first stop zone triggered by its points | [`evaluate_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L88) |
| `slowdown k r` | the source is valid, **no stop zone is triggered**, and `k` is the first triggered slowdown zone with the smallest ratio pattern, `r`, which is below the pattern of 1.0 | [`evaluate_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L103) |
| `pass` | the source is valid, no stop zone is triggered, and no slowdown zone with ratio below 1 is triggered | [`evaluate_eq_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L118) |

"The source is valid, with points `pts`" is
[`sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L136):
the source is disabled and there are no points; or the timeout is 0, no scan
is stored, and there are no points; or a scan is stored, its stamp is not
more than the timeout before `now`, and either the command carries
coordinates `c` that fit the scan and `pts` are the points of the scan's
valid readings under `c`, or it carries none that fit, the timeout is 0, and
there are no points (N27).
[`sourcePoints_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L104): otherwise the source is invalid.

Combined, they say when each velocity occurs:
[`velocity_eq_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L458), [`velocity_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L473),
[`velocity_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L488), [`velocity_eq_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L500).

Because these are equivalences, they also give the "only if" directions that
a list of safety properties alone does not: a monitor that is silent
whenever a stop is demanded, or that stops whenever a slowdown zone
triggers, or that always stops, satisfies "never publishes more than
allowed" but not these. Read off separately:
[`evaluate_stopZone_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L662)
(a stop named after zone `k` only if zone `k` is an enabled stop zone that
has a shape with at least `minPoints` points inside, or is a topic zone for
which the state holds the mark of an unusable message, ND6),
[`evaluate_slowdown_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L686),
[`evaluate_isStop_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L708)
(zero is demanded whatever the command iff the source is invalid or a stop
zone is triggered).

## The property table

One command, from an arbitrary state. "Published" and "withheld" are about
the velocity of the output; a withheld velocity is `answer none _`, not
`silent`.

| Property | Theorem | Statement |
| --- | --- | --- |
| Stop | [`stop`](../../ROSlean/CollisionMonitor/Properties.lean#L538) | Node active, command finite, source valid, some stop zone in force triggered: the action is `stopZone k` for the first such zone, and it is published, unless `prevZero` holds and the stop stamp is more than `stop_pub_timeout` old; then, and only then, the velocity is withheld. |
| A demanded stop is published | [`velocity_of_isStop`](../../ROSlean/CollisionMonitor/Properties.lean#L520) | The same for any stop, by a zone or for an invalid source. |
| Stop wins over slowdown | [`stop_not_pass_or_slowdown`](../../ROSlean/CollisionMonitor/Properties.lean#L557) | With a triggered stop zone neither `pass` nor a slowdown is published, wherever the stop zone stands in the configuration. |
| Stale data | [`stale`](../../ROSlean/CollisionMonitor/Properties.lean#L595) | Source enabled, timeout not 0, and no scan or a scan stamped more than the timeout ago: the action is `stopSource` **whatever coordinates the command carries**, published or withheld exactly as for Stop. |
| No transform | [`no_transform`](../../ROSlean/CollisionMonitor/Properties.lean#L617), [`no_transform_none`](../../ROSlean/CollisionMonitor/Properties.lean#L630) | Source enabled, timeout not 0, and the command carries no usable coordinates for the scan (none at all, which is what the shim hands over when the lookup failed): the action is `stopSource` whatever scan is stored, published or withheld exactly as for Stop. Both are readings of [`source_invalid`](../../ROSlean/CollisionMonitor/Properties.lean#L577). |
| Wrong number of points | [`wrong_count`](../../ROSlean/CollisionMonitor/Properties.lean#L643) | Coordinates that are not one pair per reading of the stored scan are not used: the answer is that of No transform. A shim that works as specified never hands such a list over; the model does not rely on it. |
| … needs its hypotheses | [`evaluate_ne_stopSource_of_off`](../../ROSlean/CollisionMonitor/Properties.lean#L742), [`sourcePoints_of_no_transform_timeout_zero`](../../ROSlean/CollisionMonitor/Properties.lean#L755) | With the source disabled or timeout 0, `stopSource` is never demanded (R6, R8, N27); with timeout 0 and no usable coordinates the zones are tested against no points at all. The counterexample is in `Examples.lean`: the scan that stops the robot when its points are known lets it pass when the transform is missing. Upstream's behaviour, kept; fail-open. |
| Bad data (a) | [`output_filter_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L858), [`points_of_not_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L887) | Readings that are not valid never change a decision: with them removed from the stored scan, and their pairs from the command's coordinates, the output of the command is the same; one such reading can be removed whatever coordinates the command carries for it. More generally two scans with the same stamp, the same number of readings and the same points under all coordinates are answered alike ([`output_congr_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L843)). |
| … what "valid" is | [`validBeam_iff_value`](../../ROSlean/CollisionMonitor/Bits.lean#L187) | A reading is valid iff neither it nor a range limit is NaN and the number it denotes lies between the limits, inclusive (R4). |
| … what the points are | [`points_eq`](../../ROSlean/CollisionMonitor/Properties.lean#L772), [`mem_points_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L779) | The points of a scan under the coordinates a command carries are the converted pairs of its valid readings, in order. |
| … how many there are | [`points_length`](../../ROSlean/CollisionMonitor/Properties.lean#L798), [`points_length_le`](../../ROSlean/CollisionMonitor/Properties.lean#L805), [`points_eq_nil_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L899) | **The number of points is decided by the stored scan**: if the coordinates fit, there are exactly as many points as valid readings, whatever the coordinates are; never more, also if they do not fit; none iff no reading is valid. The shim cannot add a point or drop one. Which readings count (R4) stays a function of the stored ranges. |
| Bad data (b) | [`step_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Properties.lean#L187) | A command with an infinite or NaN component: output `silent`, state unchanged. |
| Bad data (c), Totality | by definition | `step`, `run`, `scanPointMm` and `vertexPointMm` are total functions; Lean accepts them without `partial` or `unsafe`. |
| No amplification | [`no_amplification`](../../ROSlean/CollisionMonitor/Properties.lean#L941) | Whatever velocity is published, for whatever event, is `pass`, a stop, or `slowdown k r` where `r` is below the pattern of 1.0 and is the configured ratio of zone `k`. Holds for every configuration and every state. |
| … as a number | [`published_ratio_in_unit_interval`](../../ROSlean/CollisionMonitor/Properties.lean#L957), [`lt_one_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L239) | Such an `r` has its sign bit clear and denotes a number in `[0, 1)`. |
| Slowdown bound | [`slowdown_bound`](../../ROSlean/CollisionMonitor/Properties.lean#L968), [`evaluate_slowdown_le`](../../ROSlean/CollisionMonitor/Properties.lean#L995) | If a slowdown zone in force with ratio pattern `rz` below 1.0 is triggered, the action is a stop by a zone or a slowdown with ratio pattern at most `rz`; a demanded ratio is at most that of *every* triggered slowdown zone. |
| Pass-through | [`pass_through`](../../ROSlean/CollisionMonitor/Properties.lean#L1014), [`pass_through_of_nonzero`](../../ROSlean/CollisionMonitor/Properties.lean#L1036) | Node active, command finite, source valid, no zone triggered: the action is `pass`; it is published, except that a zero command is withheld under R15. A non-zero command is always published unchanged. ("Source valid" includes, when a scan is stored and the timeout is not 0, coordinates that fit it.) |
| Monotonicity | [`zonesAction_stop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L1057), [`zonesAction_slowdown_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L1073), [`evaluate_isStop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L1105) | Adding points never lifts a stop (the zone named stays or moves earlier), and never turns a slowdown into `pass` or into a larger ratio. Adding readings to the stored scan, each with its coordinates, never lifts a stop, for the same clock and the same zone vertices (both coordinate lists must fit their scan). |
| State | [`step_cmd_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L216), [`step_cmd_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L229), [`step_cmd_stopStamp`](../../ROSlean/CollisionMonitor/Properties.lean#L239), [`step_cmd_shapes`](../../ROSlean/CollisionMonitor/Properties.lean#L262) | A command never changes the scan or the phase. After a processed command `prevZero` is "the demanded velocity was zero" (published or not), `stopStamp` is `now` if it was zero and the previous one was not, else unchanged, and the shapes are the refreshed ones, or unchanged if the source was invalid. |
| R15, simple cases | [`velocity_of_not_yieldsZero`](../../ROSlean/CollisionMonitor/Properties.lean#L423), [`velocity_of_not_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L433), [`velocity_of_within_stopPubTimeout`](../../ROSlean/CollisionMonitor/Properties.lean#L444) | A processed command gets a velocity if the demanded velocity is not zero, or the previous one was not zero, or `now` is at most `stopStamp + stop_pub_timeout`. |

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
| Inactive | [`output_of_not_active`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L74), [`step_cmd_of_not_active`](../../ROSlean/CollisionMonitor/Properties.lean#L179) | While the node is not active the output of every event is `silent`, and a command leaves the state unchanged (N4). |
| After shutdown | [`step_of_finalized`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L150) | In the finalized phase no event changes the state and every output is `silent` (E5, ND4). |
| The lifecycle | [`step_phase`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L95), [`nextPhase`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L85), [`step_configure`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L111), [`step_activate`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L117), [`step_deactivate`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L123), [`step_cleanup`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L130), [`step_shutdown`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L138), [`step_scan`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L143) | The phase after an event is a function of the phase before it and the event: a transition the phase does not allow changes nothing; `shutdown` leads to `finalized` from everywhere; scans, shapes and commands never change the phase. Activation changes nothing else. Deactivation clears `prevZero` and sets the previous report to `normal`; the scan, the shapes and the stop stamp stay (N6). Cleanup discards the scan and every shape (N7). A scan is stored while the node is inactive or active and not otherwise (N4). No transition publishes anything ([`output_of_not_cmd`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L39)). |
| Shape accepted (iff) | [`acceptShape_eq_some_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L208), [`placeVerts_eq_some_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L174) | A polygon message for zone `k` installs a shape **iff** the node is inactive or active, zone `k` exists and is a topic zone, the shim handed over its vertices in the base frame (for a message in another frame: the transform was found), and there are at least three (N33). What it installs: if every vertex, rounded to the millimetre, is within the coordinate limit of `Shape.ok`, which a coordinate that is not finite never is ([`vertexCoordMm_finite_of_lt`](../../ROSlean/CollisionMonitor/Margin.lean#L221)), the shape (whether the frame is foreign, the rounded vertices); otherwise the mark of an unusable message (ND6, next row). (Until task T09 the last condition was part of "iff" and such a message was ignored.) |
| Unusable message (ND6; review CM5, finding 1) | [`acceptShape_unusable_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L507), [`step_shape_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L535), [`triggered_withShape_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L547), [`triggered_of_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L559), [`triggered_unusable_of_triggered`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L571), [`liveZone_triggered_of_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L581), [`evaluate_isStop_of_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L606), [`output_of_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L629), [`step_keeps_unusable`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L643), [`shapeUnusable_refreshShapes`](../../ROSlean/CollisionMonitor/Properties.lean#L147), [`refresh_unusable`](../../ROSlean/CollisionMonitor/Properties.lean#L139), [`shapeUnusable`](../../core/ROSleanCore/CollisionMonitor.lean#L398), [`TopicShape.unusableMark`](../../core/ROSleanCore/CollisionMonitor.lean#L420) | **When (iff):** a polygon message installs the mark iff the node is inactive or active, zone `k` is a topic zone, the vertices were handed over, there are at least three, and one of them, rounded to the millimetre, is not within the coordinate limit. **What the zone then does (iff):** a topic zone with an entry is triggered iff it is enabled and either the entry is the mark or at least `minPoints` points are inside its polygon; with the mark: exactly when it is enabled, whatever the points and `minPoints`. **Never less restrictive for that zone:** if the zone is triggered by some points with whatever else the state could hold for it (no shape, or any shape, in particular the one it had before, which is what the node kept until T09), it is triggered by them with the mark. **An enabled stop zone with the mark stops every command:** the action is a stop, by that zone or an earlier triggered one or for an invalid source, with no hypothesis on the scan, the coordinates, the clock or `source_timeout`; so every velocity published is zero (it may be withheld by R15, like any stop). **The mark stays** over every event but a polygon message for that zone and a cleanup; the refresh at a command neither sets nor removes it. Not proved, because it is false: that the node is never more restrictive than upstream here. It is, on purpose. |
| … the state | [`step_shape_of_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L267), [`step_shape_of_refused`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L274), [`shapes_after_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L309), [`step_shape_changes`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L283), [`step_shape_same`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L292) | An accepted message replaces entry `k` of the shapes and changes nothing else: not the other entries, the scan, `prevZero`, the stop stamp, the previous report, the phase. A refused message changes nothing. The state changes **only if** the message is accepted; the converse is false (see below). |
| No shape | [`not_triggered_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L417), [`evaluate_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L432), [`output_of_no_shape`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L468) | If zone `k` is a topic zone and the state holds no shape for it, zone `k` is not triggered by any points, whatever its `min_points`, and no output names it, neither as a velocity action nor in a report. This is upstream's behaviour (N32) and it is **not** fail-safe: a stop zone without shape does not stop. |
| Replacing a shape affects that zone only | [`withShapes_getElem?_congr`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L675), [`zonesAction_withShapes_congr`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L684) | A zone as it stands depends on the shapes only through its own entry; for two lists of shapes that agree on every zone but `k`, the actions for any points are the same unless zone `k` is triggered under the one and not under the other. |
| The shapes at a command | [`refreshShapes_getElem?`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L327), [`refresh_of_base`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L354), [`refresh_of_foreign`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L367), [`refresh_of_no_transform`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L360), [`refresh_of_refused`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L377), [`refresh_foreign`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L394), [`refreshShapes_getD_eq_none_iff`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L336) | For a command the shape of every enabled topic zone is refreshed with that zone's vertices: a shape received in the base frame never moves; one received in another frame takes the vertices the command carries, rounded, if they are as many as it has and all within the coordinate limit, and keeps its vertices otherwise, in particular when the command carries none (N34). The refresh changes neither the frame flag nor the number of vertices, and neither gives a zone a shape nor takes one away. |
| Lifecycle invariants | [`Inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L729), [`inv_init`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L738), [`inv_step`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L782), [`reachable_inv`](../../ROSlean/CollisionMonitor/Trace.lean#L149) | In every state reachable from `State.init cfg`: while the node is unconfigured or inactive, `prevZero` is false and the previous report is `normal`; while it is unconfigured, no scan and no shape is stored; the shapes have one entry per zone, and a shape only at a topic zone; every stored shape has at least three vertices, all within the coordinate limit ([`ShapeOk`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L713)). |
| After activation | [`after_activation`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L913), [`after_activation_of_inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L935), [`run_after_activation`](../../ROSlean/CollisionMonitor/Trace.lean#L664) | From an inactive state with no zero on record and previous report `normal` (every reachable inactive state), `activate` makes the node active, and a finite command that comes next gets a velocity, also when that velocity is zero: a stop is never withheld right after an activation (N6). Its report is published unless it is `normal`. |
| Saturated points and topic zones | [`liveConfig_bounded`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L955), [`zonesAction_filter_inRange'`](../../ROSlean/CollisionMonitor/Geometry.lean#L692), [`contains_eq_false_of_far'`](../../ROSlean/CollisionMonitor/Geometry.lean#L623) | In a reachable state with an accepted configuration every zone in force, topic zones with their shapes included, lies within the coordinate limit ([`ShapeBounded`](../../ROSlean/CollisionMonitor/Geometry.lean#L597)); so a saturated point, which is what a coordinate that is not finite becomes, is in none of them and does not change the action. |

### Where SPEC_NAV2.md, section 4, was false as stated

Two statements of the design did not hold of the model it specifies. The
model was not changed; the theorems state what is true, and SPEC_NAV2.md is
corrected.

1. **"A `shape` event changes the state iff it is accepted."** Only "only
   if" holds. Counterexample: the same accepted message delivered twice. The
   second is accepted and leaves the state exactly as it was
   ([`step_shape_same`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L292)). True: a refused message changes nothing
   ([`step_shape_of_refused`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L274)); an accepted one sets entry `k` to the new
   shape and changes nothing else ([`step_shape_of_accepted`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L267)).
2. **"If `phase ≠ active` then `prevZero = false` and `prevReport =
   normal`."** False in the finalized phase. Counterexample: `configure`,
   `activate`, a finite command with no scan stored and a non-zero source
   timeout (a stop: `prevZero` becomes true, the previous report `source`),
   `shutdown`. E5 changes the phase and nothing else, so the finalized state
   has `prevZero = true`. True, and proved ([`Inv`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L729), field `quiet`): the
   statement for the phases unconfigured and inactive. In the finalized
   phase the two fields are never read again ([`step_of_finalized`](../../ROSlean/CollisionMonitor/Lifecycle.lean#L150)).

One statement of section 4 no longer exists and one was made precise:

3. **"`Xform.identity.pointMm x y = (coordMm x, coordMm y)`"** was about
   the transform in the model. With option A there is no transform in the
   model: a scan point **is** `(coordMm x, coordMm y)` of the coordinates
   the command carries, by definition ([`scanPointMm`](../../core/ROSleanCore/CollisionMonitorPoint.lean#L149)).
4. **"On runs: no output names a topic zone `k` before an accepted `shape k`
   event that follows the last cleanup."** "The last cleanup" has to be one
   that took effect (a `cleanup` outside the inactive phase is a no-op and
   discards nothing). Proved in the form: after any history that leaves the
   node unconfigured, and any events that follow among which there is no
   shape message for zone `k`, no output names zone `k`
   ([`run_output_of_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L603)).

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
| [`run_sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Trace.lean#L240) | The points tested after any events since start-up, stated on the events and the command's coordinates alone. |
| [`run_processed`](../../ROSlean/CollisionMonitor/Trace.lean#L110) | **Only processed commands are answered.** If a position of a run has an output other than `silent`, the event there is a finite command and the node was active. |
| [`run_silent_before_activate`](../../ROSlean/CollisionMonitor/Trace.lean#L274) | **Inactive.** In any run from start-up, if no event before position `n` is an `activate`, the output at position `n` is `silent`. |
| [`run_silent_after_shutdown`](../../ROSlean/CollisionMonitor/Trace.lean#L310) | In any run from any state, every output after a `shutdown` event is `silent`, whatever follows it (ND4). |
| [`run_silent_after_deactivate`](../../ROSlean/CollisionMonitor/Trace.lean#L351), [`run_silent_of_no_activate`](../../ROSlean/CollisionMonitor/Trace.lean#L259) | If position `m` is a `deactivate` and no `activate` lies strictly between `m` and `n`, the output at position `n` is `silent`. |
| [`run_stop`](../../ROSlean/CollisionMonitor/Trace.lean#L392) | **Stop.** After any events since start-up that leave the node active: if the scan in force is not too old, the command carries coordinates that fit it, and the scan's points under them trigger a stop zone with a fixed shape, a finite command gets the velocity `stopZone k` for the first triggered stop zone, or none in the one case of R15. |
| [`run_stale`](../../ROSlean/CollisionMonitor/Trace.lean#L420) | **Stale data.** Likewise with `stopSource` when no scan is in force or the one in force is too old, whatever the command carries. |
| [`run_no_transform`](../../ROSlean/CollisionMonitor/Trace.lean#L441) | **No transform.** Likewise with `stopSource` when the command carries no coordinates that fit the scan in force, whatever that scan is. |
| [`run_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Trace.lean#L455) | **Bad data.** A non-finite command leaves no trace: the run with it is the run without it, plus one `silent`. |
| [`run_no_amplification`](../../ROSlean/CollisionMonitor/Trace.lean#L467) | **No amplification.** Every velocity action published anywhere in any run is `pass`, a stop, or a slowdown with a ratio pattern below that of 1.0, configured for the zone named. |
| [`run_slowdown_bound`](../../ROSlean/CollisionMonitor/Trace.lean#L488) | **Slowdown bound**, with hypotheses on the scan in force and a slowdown zone with a fixed shape. |
| [`run_pass_through`](../../ROSlean/CollisionMonitor/Trace.lean#L510) | **Pass-through.** Node active, the scan in force not too old, coordinates that fit it, no zone in force triggered: a finite non-zero command is published unchanged. |
| [`run_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L571), [`run_output_of_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L603), [`no_shape_of_unconfigured`](../../ROSlean/CollisionMonitor/Trace.lean#L584), [`step_no_shape`](../../ROSlean/CollisionMonitor/Trace.lean#L531) | **No shape.** From a state without a shape for zone `k`, over events none of which is a shape message for zone `k`, the state still has none: an accepted shape message is the only way to give a zone a shape. After any history that leaves the node unconfigured (start-up, or a cleanup that took effect) and any such events, no output names the topic zone `k`. |
| [`run_stop_after_unusable`](../../ROSlean/CollisionMonitor/Trace.lean#L639), [`run_keeps_unusable`](../../ROSlean/CollisionMonitor/Trace.lean#L621) | **After an unusable message, over a run (ND6).** Zone `k` an enabled stop zone fed by a topic; any events since start-up; a polygon message for zone `k` that installs the mark; then any events among which there is no polygon message for zone `k` and no cleanup (deactivations and activations included): whatever velocity is published for the next event is a stop. |
| [`run_after_activation`](../../ROSlean/CollisionMonitor/Trace.lean#L664) | **After activation**, for a history that leaves the node inactive. |

In the hypotheses of `run_stop` and `run_slowdown_bound` the zone has a
fixed shape, because such a zone is in force in every state
([`liveConfig_of_fixed`](../../ROSlean/CollisionMonitor/Trace.lean#L376)) and the statement then speaks about events only.
For a topic zone the single-step theorems apply at every point of a run,
with the shape the state holds; there is no closed description of that shape
in terms of the history (it depends on every refresh).

### `stop_pub_timeout`: when the node withholds although a stop is demanded

R15 is upstream's rule and the model reproduces it: zero is published for
`stop_pub_timeout` after it first appears, then no velocity, until a
non-zero velocity is demanded. Three theorems say exactly what that means.

| Theorem | Statement |
| --- | --- |
| [`run_withheld_last_published_zero`](../../ROSlean/CollisionMonitor/Trace.lean#L899) | **A withheld velocity follows a published zero.** In any run from start-up, if a command is processed and gets no velocity, then the velocity published last before it was a zero: an earlier command was answered with an action that yields zero for it, and no velocity was published in between ([`ZeroPublishedLast`](../../ROSlean/CollisionMonitor/Trace.lean#L802)). |
| [`zero_period`](../../ROSlean/CollisionMonitor/Trace.lean#L772) | **A period of zero.** If a processed command at time `t` is the first for which zero is demanded, no `deactivate` follows, and every velocity published afterwards is zero, then a later finite command at `now'`, with the node still active, gets the demanded action unless that is again zero and `t + stop_pub_timeout < now'`. So zeros are published from `t` up to and including `t + stop_pub_timeout`, whatever their cause, and a non-zero demand is always published. |
| [`step_zero_start`](../../ROSlean/CollisionMonitor/Trace.lean#L687), [`run_zero_streak`](../../ROSlean/CollisionMonitor/Trace.lean#L723), [`step_zero_record`](../../ROSlean/CollisionMonitor/Trace.lean#L704) | What the state records: the first zero is published and sets the stop stamp to its time; during the period `prevZero` stays set and the stop stamp does not move. Only a command and a `deactivate` touch the record. |
| [`run_withheld_after_published`](../../ROSlean/CollisionMonitor/Trace.lean#L915) | No velocity is withheld before one has been published: the first processed command after start-up always gets a velocity. |

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

## The coordinates a command carries, and the margin (option A)

In [`Margin.lean`](../../ROSlean/CollisionMonitor/Margin.lean), about
`scanPointMm` and `vertexPointMm` of
[`CollisionMonitorPoint.lean`](../../core/ROSleanCore/CollisionMonitorPoint.lean)
(SPEC_NAV2, section 5, option A). [`sval64`](../../ROSlean/CollisionMonitor/Margin.lean#L58) is the number a finite
binary64 pattern denotes, with its sign. "The value the shim handed over"
is `sval64` of a pattern, in metres.

| Theorem | Statement |
| --- | --- |
| [`coordMm_eq_truncReal`](../../ROSlean/CollisionMonitor/Margin.lean#L131), [`coordMm_sub_lt`](../../ROSlean/CollisionMonitor/Margin.lean#L174), [`scanPointMm_sub_lt`](../../ROSlean/CollisionMonitor/Margin.lean#L182) | **A scan point is the shim's point, truncated.** For a finite coordinate, `coordMm` is the value the shim handed over, in millimetres, truncated toward zero and saturated at ±1,000,000 ([`truncReal`](../../ROSlean/CollisionMonitor/Margin.lean#L70)); unless it saturates it is less than 1 mm from that value, on each axis. With `coordMm_abs_le` and `coordMm_sign` of `Convert.lean`: never further from an axis, same sign. |
| [`scanPointMm_inRange`](../../ROSlean/CollisionMonitor/Margin.lean#L189) | A scan point is within ±1,000,000 whatever the patterns (NaN and the infinities included). |
| [`vertexCoordMm_sub_le`](../../ROSlean/CollisionMonitor/Margin.lean#L237), [`vertexPointMm_sub_le`](../../ROSlean/CollisionMonitor/Margin.lean#L253), [`vertexMagMm_eq`](../../ROSlean/CollisionMonitor/Margin.lean#L196) | **A vertex of a topic zone is within half a millimetre of the shim's vertex** on each axis, if it is within the coordinate limit (as every accepted vertex is). |
| [`vertexCoordMm_finite_of_lt`](../../ROSlean/CollisionMonitor/Margin.lean#L221), [`vertexPointMm_finite`](../../ROSlean/CollisionMonitor/Margin.lean#L265) | **A vertex that is not finite is never accepted** (ND6): a vertex coordinate below the coordinate limit denotes a finite number below 1024 m. A NaN, an infinity or a value of 1024 m or more becomes ±1,000,000 mm, which no shape may reach. |
| [`contains_of_close`](../../ROSlean/CollisionMonitor/Margin.lean#L289), [`contains_of_rounded`](../../ROSlean/CollisionMonitor/Margin.lean#L307) | **The margin covers any displacement within its budget.** If every vertex of the model's polygon is within `dv` per axis of a real polygon's vertex, a point `q` of that polygon's boundary is within `dp` per axis of the model's point, and `2·(dv + dp)² ≤ 5` (that is, `dv + dp ≤ 1.5811` mm), the model counts the point. `contains_of_rounded` is the case `dv = 1/2`, `dp = 1` of SPEC.md. |
| [`contains_of_shim`](../../ROSlean/CollisionMonitor/Margin.lean#L336) | **Option A, a topic zone.** Let the shim hand over the vertices of an accepted shape and the finite, unsaturated coordinates of a reading. If upstream's vertices and upstream's point are each within `δ` per axis of the shim's, upstream's point lies on the boundary of upstream's polygon, and **`δ ≤ 1/25` mm**, the model counts the reading. (`2·(3/2 + 2δ)² ≤ 5`.) |
| [`contains_of_shim_fixed`](../../ROSlean/CollisionMonitor/Margin.lean#L367) | **Option A, a fixed polygon.** The same when the vertices are the parameters, rounded by the model, and only the point differs: **`δ ≤ 2/25` mm** suffices. |
| [`circle_safe_shim`](../../ROSlean/CollisionMonitor/Margin.lean#L394) | **Option A, a circle.** If upstream's point is within `δ ≤ 1/2` mm per axis of the shim's and lies strictly inside the configured circle, the model counts the reading: the 1 mm added to every radius covers it. |

**What is proved, what is prose and what is trusted in "a point upstream
counts is always counted"**, with the transform in the shim. Write `u` for
the point upstream computes for a reading, `s` for the point the shim
computes, `m` for the model's point; likewise for the vertices of a topic
zone.

- **Proved.** `m` is `s` truncated: less than 1 mm from it per axis, toward
  the origin (`scanPointMm_sub_lt`). A topic-zone vertex of the model is
  within 1/2 mm of the shim's (`vertexPointMm_sub_le`); a fixed vertex within
  1/2 mm of the parameter (`vertexMm_abs_sub_le`). **If** `u` and `s`, and
  the vertices, differ by at most `δ` per axis with `δ ≤ 0.04` mm, a point
  `u` on the boundary of upstream's polygon is counted (`contains_of_shim`);
  for circles with `δ ≤ 0.5` mm (`circle_safe_shim`).
- **Trusted, not proved (T2, T7).** That `|u − s| ≤ δ` for such a `δ`. The
  shim evaluates upstream's formula (N25) in `double` **without** fused
  multiply-add (the node is built with `-ffp-contract=off` on every
  platform; the user's decision, tasks/PLAN.md); upstream's arm64 binary
  fuses the two sums, baseline x86-64 builds cannot. So `s` and `u` are two
  roundings of the same exact expression and can differ in the last bits of
  a coordinate. The estimate, from the IEEE-754 bound of half a unit in the
  last place per operation, about fourteen operations, and coordinates below
  1000 m, is 10⁻⁸ mm for each of them against the exact image; SPEC_NAV2,
  section 5, allows 10⁻⁶ mm. The budget the theorems leave is 0.04 mm,
  40,000 times that allowance. It is an estimate: Lean has no model of
  binary64 arithmetic or of the two compilers in this repository. What is
  tested of the shim's code (`transforms.hpp`, test `smoke_transforms`): on
  200,000 transforms its points are, bit for bit, a reference written out
  operation by operation, never fused.
- **Prose, not proved, as for fixed zones.** That the boundary passes over
  the point when a point inside upstream's polygon ends up outside the
  rounded one (the topological half, see "Geometry").

Not proved either: that the coordinates handed over are those of the stored
scan's readings, in order, under the right transform (T2, T6); and that
`sval64` is IEEE-754's reading of a pattern (it is `val64` of `Convert.lean`
with the sign, see "The bit-pattern helpers").

### What T05 proved and option A does not have

| T05 (option B, the transform in the model) | Now |
| --- | --- |
| `pointMm_x_eq`, `pointMm_y_eq`: a scan point is the **exact image** of the scan-frame point under the transform, truncated | Cannot be stated: the model has neither the scan-frame point nor the transform. Replaced by `coordMm_eq_truncReal`: a scan point is **the shim's point**, truncated. That the shim's point is the image, to within rounding, is trusted (T2). |
| `pointMm_x_abs_le`, `pointMm_x_abs_gt`, `pointMm_x_sub_lt` (and `y`): bounds against the exact image | `coordMm_abs_le`, `coordMm_abs_gt`, `coordMm_sub_lt`: the same bounds against the shim's point. |
| `vertexMm_x_sub_le`, `vertexMm_y_sub_le`: a topic-zone vertex is within 1/2 mm of its exact image | `vertexPointMm_sub_le`: within 1/2 mm of the shim's vertex. |
| `pointMm_identity`: under the identity the point is `coordMm` | True by definition for every command. |
| `rotX_of_unit`, `rotY_of_unit`, `rotX_smul`, `rotY_smul`, `usable_iff`: the formula of N25 is the rotation of the normalised quaternion; when a transform is usable | Dropped: the formula is the shim's, in C++ (T06 tests it against tf2). ND7 (a non-finite or zero quaternion) is the shim's to turn into "no points". |
| `contains_of_transformed`, `contains_of_image_on_boundary`: the margin, measured from exact images; upstream within 10⁻⁶ mm of them (T7) with a budget of 0.114 mm Euclidean | `contains_of_shim`, `contains_of_shim_fixed`: the margin, measured from the shim's values, with the distance `δ` between the shim and upstream as an explicit hypothesis; budget 0.04 mm per axis (0.08 mm for fixed polygons). |
| "The points are a function of the stored scan and the transform" | The points are a function of the stored scan (which readings, how many: `points_length`) and of the coordinates the command carries (where). That those coordinates belong to the stored scan is trusted (T2). |
| The shape of a foreign topic zone was recomputed in Lean from the received vertices (`refresh_of_foreign` with `raw`) | The shim keeps the message and hands over the recomputed vertices; the model checks their number and range (`refresh_foreign`, `refresh_of_refused`). |

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
| [`firstStop_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L222), [`firstStop_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L242) | The stop loop returns the first triggered stop zone, or nothing iff none is triggered. |
| [`bestSlowdown_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L419), [`bestSlowdown_snd_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L437) | The slowdown loop returns the best slowdown zone, or nothing iff no slowdown zone with a ratio below 1 is triggered. |
| [`zonesAction_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L461), [`zonesAction_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L472), [`zonesAction_eq_pass_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L484) | The action for given points, by cases. |
| [`zonesAction_congr`](../../ROSlean/CollisionMonitor/Basic.lean#L537) | The action depends on the points only through which zones they trigger. |
| [`zonesAction_eq_of_map`](../../ROSlean/CollisionMonitor/Basic.lean#L505) | More generally, the action depends on the zones and the points only through the action of each zone, in order, and whether it is triggered. |
| [`triggered_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L46), [`triggered_nil_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L69), [`not_triggered_of_topic`](../../ROSlean/CollisionMonitor/Basic.lean#L58) | A zone triggers iff it is enabled, has a shape, and has at least `minPoints` points inside; without points, only a zone with `minPoints = 0` triggers (R11); a zone without shape never triggers (N32). |
| [`withShapes_getElem?`](../../ROSlean/CollisionMonitor/Basic.lean#L595), [`withShape_fields`](../../ROSlean/CollisionMonitor/Basic.lean#L550), [`withShape_of_fixed`](../../ROSlean/CollisionMonitor/Basic.lean#L566), [`withShape_of_topic`](../../ROSlean/CollisionMonitor/Basic.lean#L583), [`withShape_none`](../../ROSlean/CollisionMonitor/Basic.lean#L574) | The zones with their shapes filled in: zone `k` is the configured zone `k` with entry `k` of the shapes; the action and the flag are the configured ones, and so is the threshold unless the zone is a topic zone whose entry is the mark of an unusable message, where it is 0 (ND6); a fixed zone is unaffected; a topic zone with an entry is the polygon of its current vertices, and without one stays without. |
| [`evaluate_congr`](../../ROSlean/CollisionMonitor/Properties.lean#L68) | The demanded action reads the state only through the stored scan and the shapes. |

## The C interface and the conversions

The node calls the exported wrappers of
`core/ROSleanCore/CollisionMonitorFFI.lean`, and the shim hands every
parameter, every range, and every coordinate of a point or vertex over as a
bit pattern, to be converted in Lean
(`core/ROSleanCore/CollisionMonitorConvert.lean`,
`CollisionMonitorPoint.lean`). The theorems are in
[`FFI.lean`](../../ROSlean/CollisionMonitor/FFI.lean) and
[`Convert.lean`](../../ROSlean/CollisionMonitor/Convert.lean).

Since task T06 the interface is the one of SPEC_NAV2.md, section 8: a
monitor starts unconfigured; the node delivers every lifecycle transition
(`cmOnLifecycle`), a scan as its ranges (`cmOnScan`), a polygon message as
its vertices in the base frame (`cmOnShape`), and a command together with
one coordinate pair per reading of the stored scan and the vertices of the
topic zones (`cmOnCmd`). **The interface keeps nothing between calls but the
monitor**: everything an event carries is an argument of its call, so the
events a sequence of calls delivers are a function of the calls alone
(`events`).

| Theorem | Statement |
| --- | --- |
| **End to end** | |
| [`cm_readBack_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1640), [`cm_out_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1626), [`cm_readBack_after_fast`](../../ROSlean/CollisionMonitor/Fast.lean#L525), [`cm_phase_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1662) | Stated on the exported functions in the order the node calls them. Create a monitor from a builder that is ok (`cmNew`), make any sequence of lifecycle, scan, shape and command calls ([`Call`](../../ROSlean/CollisionMonitor/FFI.lean#L1339), [`Call.exec`](../../ROSlean/CollisionMonitor/FFI.lean#L1347); no condition on the sequence or on any argument), then deliver a command with `cmOnCmd`: the five numbers that `cmOutKind`, `cmOutZone`, `cmOutRatio`, `cmOutReport` and `cmOutReportZone` return determine, and are the codes of, the **whole output** of `step` for that command's event ([`cmdEvent`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L440)): whether it was processed at all, the velocity action or that it is withheld, and the report. The state is the one `run` reaches on the events the calls deliver ([`events`](../../ROSlean/CollisionMonitor/FFI.lean#L1358), [`Call.events`](../../ROSlean/CollisionMonitor/FFI.lean#L1358)), **from the initial state `State.init`**, on the builder's configuration. So every theorem above about runs from start-up is a statement about the running node with `es := events cs`; there is no prefix to add, **for a node that has processed no command without a clock reading** (D15). `Call` has no such command. Once the node has processed one, its state is one that no sequence of calls reaches ([`noClock_unreachable`](../../ROSlean/CollisionMonitor/NoClock.lean#L181)), and the theorems stated from start-up apply only through [`mixed_from_init`](../../ROSlean/CollisionMonitor/NoClock.lean#L310) ("A command without a clock reading" below): read on the events in which that command is `deactivate, activate`, for everything but one state message. `cm_readBack_after_fast` is the same statement on the functions C calls. `cm_phase_after`: the phase query returns the phase of that run. |
| [`cm_not_ok_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1675), [`foldl_exec_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1432), [`apply_cmd_of_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1018), [`apply_out_of_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1036), [`apply_st_of_not_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1026) | **Fail closed.** If the configuration was refused, the answer to every command, after any calls whatever, is `stopSource` (kind 4) if the monitor is in the active phase and `silent` otherwise: never a pass, never a slowdown; also for a non-finite command, and without the withholding of R15. The node fails `configure` for such a monitor and drops it; this is the backstop. A monitor that is not ok follows the lifecycle like any other. |
| [`readBack_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1561), [`outputOfCodes_codes`](../../ROSlean/CollisionMonitor/FFI.lean#L1471), [`kindOf_eq_zero_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L1484), [`kindOf_eq_five_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L1493), [`step_publish_stopZone`](../../ROSlean/CollisionMonitor/FFI.lean#L1502), [`step_publish_slowdown`](../../ROSlean/CollisionMonitor/FFI.lean#L1510), [`step_report_zone`](../../ROSlean/CollisionMonitor/FFI.lean#L1517) | Nothing is truncated on the way to C: a published or reported zone index is below the number of zones (at most 2³² assumed), and a published ratio is below the pattern of 1.0. **Kind 0 is `silent` and nothing else** (the shim then publishes nothing, on any topic); kind 5 is a processed command whose velocity is withheld (the report and the polygons are still published). |
| **Events** | |
| [`cmOnLifecycle_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1066), [`cmOnLifecycle_of_unknown`](../../ROSlean/CollisionMonitor/FFI.lean#L1071), [`lifecycleEvent_codes`](../../ROSlean/CollisionMonitor/FFI.lean#L1048), [`lifecycleEvent_eq_none`](../../ROSlean/CollisionMonitor/FFI.lean#L1054), [`cmPhase_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1076), [`cmOnScan_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1084), [`cmOnShape_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1091), [`cmOnCmd_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1169), [`exec_st`](../../ROSlean/CollisionMonitor/FFI.lean#L1385), [`exec_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L1369), [`apply_of_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1009), [`foldl_apply`](../../ROSlean/CollisionMonitor/FFI.lean#L1325), [`foldl_exec_run`](../../ROSlean/CollisionMonitor/FFI.lean#L1417), [`foldl_exec_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L1405) | Each call is `step` on the event it delivers: transition codes 0 to 4 are `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`, and any other code delivers nothing and changes nothing; a scan call delivers a `scan` with the decoded ranges; a shape call a `shape` with the decoded vertices, or none if `has_verts` is 0; a command call a `cmd` with the decoded coordinate pairs, or none if `has_points` is 0, and the decoded zone vertices. A sequence of calls is `run` on those events, with no hypothesis about the calls. |
| [`cmShapeUnusable_after`](../../ROSlean/CollisionMonitor/FFI.lean#L1118), [`cmShapeUnusable_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L1111), [`cmShapeAccepted_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L1102), [`cmOnShape_st_of_accepted`](../../ROSlean/CollisionMonitor/FFI.lean#L1139), [`cmOnShape_st_of_refused`](../../ROSlean/CollisionMonitor/FFI.lean#L1150) | **Shape messages.** The query after a shape call returns true iff the monitor is ok and the model accepts the message (`acceptShape`, characterised by `acceptShape_eq_some_iff`). An accepted call installs the shape, or the mark, in its zone and changes nothing else; a refused one changes no state. The shim keeps a message for later lookups only on "true". The second query, `roslean_cm_shape_unusable`, returns whether the state holds the mark of an unusable message for the zone; after an accepted call for a zone the state has an entry for, it is true iff what the call installed is the mark (ND6): the shim then keeps no message for the zone. |
| [`step_cmd_of_plan_zero`](../../ROSlean/CollisionMonitor/FFI.lean#L1237), [`step_cmd_of_plan_one`](../../ROSlean/CollisionMonitor/FFI.lean#L1253), [`cmOnCmd_of_plan_zero`](../../ROSlean/CollisionMonitor/FFI.lean#L1291), [`cmOnCmd_of_plan_one`](../../ROSlean/CollisionMonitor/FFI.lean#L1307), [`cmdPlan_eq_three_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L1263), [`cmdPlan_cases`](../../ROSlean/CollisionMonitor/FFI.lean#L1179), [`sourcePoints_of_plan_zero`](../../ROSlean/CollisionMonitor/FFI.lean#L1187), [`sourcePoints_of_plan_one`](../../ROSlean/CollisionMonitor/FFI.lean#L1211), [`cmCmdPlan_of_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L1285) | **The plan of a command** ([`cmdPlan`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L450); the query returns a number and no monitor, so it cannot change one). If the plan is 0 the command call returns the same monitor for all coordinate arrays and all zone-vertex arrays: the shim looks nothing up and waits for nothing. If it is 1 the result does not depend on the scan's coordinates: only the zones' transforms are looked up. It is 3 iff the command is processed, the source is enabled, a scan is stored and it is not too old: exactly when upstream looks the scan's transform up (N22). The plan is always 0, 1 or 3. |
| **Configuration and refusals** | |
| [`builderAddCircle_zones`](../../ROSlean/CollisionMonitor/FFI.lean#L134), [`builderAddPolygon_zones`](../../ROSlean/CollisionMonitor/FFI.lean#L148), [`foldl_addVertex`](../../ROSlean/CollisionMonitor/FFI.lean#L209), [`zoneOfRaw_fields`](../../ROSlean/CollisionMonitor/FFI.lean#L119), [`builderSetSource_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L98), [`builderSetStopPubTimeout_cfg`](../../ROSlean/CollisionMonitor/FFI.lean#L105) | The configuration is what the builder calls say: zones in the order added, a polygon's vertices in the order of its `add_vertex` calls, each value the conversion of its argument. |
| [`cmOk_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L289), [`cmOk_cfg_ok`](../../ROSlean/CollisionMonitor/FFI.lean#L298), [`error_eq_zero_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L268), [`builderNew_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L91), [`cmNew_eq`](../../ROSlean/CollisionMonitor/FFI.lean#L246), [`builderAddTopicPolygon_zones`](../../ROSlean/CollisionMonitor/FFI.lean#L154) | A monitor is ok iff the source was set, no timeout was refused, no vertex was given to no polygon, no zone carries a refused value, and every zone passes `Zone.ok`; so `Config.ok` holds for it, which is the hypothesis of "a saturated point is in no zone" (`contains_eq_false_of_far`). A new builder is not ok. A new monitor is unconfigured (`State.init`). `add_topic_polygon` appends a zone without shape. |
| [`not_ok_of_refused_call`](../../ROSlean/CollisionMonitor/FFI.lean#L520) | **No refused value is dropped, wherever it occurs.** Take any builder and any sequence of the six builder calls ([`BuilderCall`](../../ROSlean/CollisionMonitor/FFI.lean#L371)). If any one call carries a refused value ([`BuilderCall.Refused`](../../ROSlean/CollisionMonitor/FFI.lean#L391): a timeout of `set_source` or `set_stop_pub_timeout`, a radius, a `min_points`, a vertex with a coordinate that is not finite), the monitor made from the result is not ok, whatever the calls before and after it. In particular a refused vertex makes the monitor not ok also when the zone added last is a circle or when there is no zone, and no later call (a second `set_source` with a good value, further vertices) clears a refusal. |
| [`not_ok_of_stray_vertex`](../../ROSlean/CollisionMonitor/FFI.lean#L531), [`not_ok_without_set_source`](../../ROSlean/CollisionMonitor/FFI.lean#L559) | A vertex, refused or not, that is added when the zone added last is not a polygon makes the monitor not ok. A monitor made from a new builder by calls among which there is no `set_source` is not ok: the source timeout is never 0, "check off", by omission. |
| [`exec_marked_of_refused`](../../ROSlean/CollisionMonitor/FFI.lean#L477), [`exec_marked_mono`](../../ROSlean/CollisionMonitor/FFI.lean#L454), [`not_ok_of_marked`](../../ROSlean/CollisionMonitor/FFI.lean#L406), [`not_ok_of_source_timeout`](../../ROSlean/CollisionMonitor/FFI.lean#L316), [`not_ok_of_stop_pub_timeout`](../../ROSlean/CollisionMonitor/FFI.lean#L327), [`entryError_addCircle_ne_zero`](../../ROSlean/CollisionMonitor/FFI.lean#L333), [`foldl_addVertex_refused`](../../ROSlean/CollisionMonitor/FFI.lean#L572), [`not_ok_of_entry`](../../ROSlean/CollisionMonitor/FFI.lean#L305), [`vertexOfRaw_eq_none_iff`](../../ROSlean/CollisionMonitor/FFI.lean#L609) | The steps of the two rows above: each refused value leaves a mark on the builder, no call removes a mark, and a marked builder is not ok. Within one polygon the mark is a code on that polygon, which is how the node names the parameter. |
| **Scan decoding** | |
| [`decodeBeams_length`](../../ROSlean/CollisionMonitor/FFI.lean#L665), [`decodePairs_length`](../../ROSlean/CollisionMonitor/FFI.lean#L669), [`getElem_decodeBeams`](../../ROSlean/CollisionMonitor/FFI.lean#L673), [`getElem_decodePairs`](../../ROSlean/CollisionMonitor/FFI.lean#L678), [`beamAt_lt`](../../ROSlean/CollisionMonitor/FFI.lean#L657), [`pairAt_lt`](../../ROSlean/CollisionMonitor/FFI.lean#L660), [`decodeZoneVerts_empty`](../../ROSlean/CollisionMonitor/FFI.lean#L986) | A byte array of any length is a scan of `size / 4` readings, or `size / 16` coordinate pairs; every decoded pattern fits its field, whatever the bytes. The zone vertices are decoded by a total function too ([`decodeZoneVerts`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L334)): it stops at the end of the array or at the first entry the array does not hold in full, and a zone it does not reach has none. |
| [`fits_decode`](../../ROSlean/CollisionMonitor/FFI.lean#L685), [`points_decode`](../../ROSlean/CollisionMonitor/FFI.lean#L693) | The coordinates of a command fit the stored scan iff the coordinate array holds as many pairs as the scan array held readings (otherwise the model treats the lookup as failed, E8 step 3). Then the scan's points are the converted pairs of the readings whose range is valid, reading `i` with pair `i`, in order. |
| [`decodeBeams_encodeRanges`](../../ROSlean/CollisionMonitor/FFI.lean#L855), [`decodePairs_encodePairs`](../../ROSlean/CollisionMonitor/FFI.lean#L866), [`decodeZoneVerts_encodeZoneVerts`](../../ROSlean/CollisionMonitor/FFI.lean#L971), [`u32At_encodeRanges`](../../ROSlean/CollisionMonitor/FFI.lean#L794), [`pairAt_encodePairs`](../../ROSlean/CollisionMonitor/FFI.lean#L815), [`parseZoneVerts_flatMap`](../../ROSlean/CollisionMonitor/FFI.lean#L915), [`takePairs_flatMap`](../../ROSlean/CollisionMonitor/FFI.lean#L883), [`leNat_leBytes`](../../ROSlean/CollisionMonitor/FFI.lean#L723) | Decoding inverts the shim's encoding, for each of the four arrays: the model sees the ranges, the coordinate pairs of a command, the vertices of a shape message and the zone vertices of a command (for each zone a list of fewer than 2³² − 1 vertices, or none: [`ZoneEntryFits`](../../ROSlean/CollisionMonitor/FFI.lean#L911)) as written, unconverted. Coordinates become points when a command is evaluated (`coordMm`), vertices when a shape is accepted or refreshed (`vertexCoordMm`). |
| **Conversions** (against the number a bit pattern denotes, `val64`, `val32`) | |
| [`coordMm_abs_le`](../../ROSlean/CollisionMonitor/Convert.lean#L232), [`coordMm_abs_gt`](../../ROSlean/CollisionMonitor/Convert.lean#L243), [`coordMm_sign`](../../ROSlean/CollisionMonitor/Convert.lean#L254), [`coordMm_natAbs_le`](../../ROSlean/CollisionMonitor/Convert.lean#L221) | A scan coordinate in millimetres is never farther from zero than the true value, is less than 1 mm short of it unless it saturates, keeps its sign, and is within ±1,000,000. |
| [`radiusMm_ge`](../../ROSlean/CollisionMonitor/Convert.lean#L298), [`radiusMm_lt`](../../ROSlean/CollisionMonitor/Convert.lean#L306), [`radiusMm_abs`](../../ROSlean/CollisionMonitor/Convert.lean#L268), [`radiusMm_le_limit`](../../ROSlean/CollisionMonitor/Convert.lean#L289), [`radiusMm_of_not_finite`](../../ROSlean/CollisionMonitor/Convert.lean#L264) | The model's radius is at least the configured magnitude plus 1 mm and less than that plus 2 mm; the sign of the parameter does not matter (R9, review finding 1); NaN, infinities and too large values are refused. |
| [`circle_safe`](../../ROSlean/CollisionMonitor/Convert.lean#L319) | **Circles are safe**: a point strictly inside the configured circle, in exact arithmetic on the values the shim passed, is inside the model's circle. This is the SPEC's safe-direction claim for circles, with the conversions included. |
| [`vertexMm_abs_sub_le`](../../ROSlean/CollisionMonitor/Convert.lean#L345), [`vertexMm_isSome_iff`](../../ROSlean/CollisionMonitor/Convert.lean#L340) | A vertex coordinate of a fixed polygon (a `float` parameter) moves by at most half a millimetre; it is refused iff it is not finite. For the vertices of a topic zone (`double`, base frame) see `vertexCoordMm_sub_le` above. |
| [`timeoutNs_of_neg`](../../ROSlean/CollisionMonitor/Convert.lean#L375), [`timeoutNs_of_not_finite`](../../ROSlean/CollisionMonitor/Convert.lean#L371), [`timeoutNs_of_huge`](../../ROSlean/CollisionMonitor/Convert.lean#L383), [`timeoutNs_isSome`](../../ROSlean/CollisionMonitor/Convert.lean#L398), [`sourceTimeoutNs_eq_none`](../../ROSlean/CollisionMonitor/Convert.lean#L411) | A timeout is accepted only if it is finite, not negative and below 9.2·10⁹ s; so a refused timeout cannot become 0, "check disabled" (review finding 7a). |
| [`minPointsOfRaw_eq_some_iff`](../../ROSlean/CollisionMonitor/Convert.lean#L431) | `min_points` is accepted iff it is in the range of a 32-bit `int` (review finding 13). |
| [`mant_shift_eq_magVal`](../../ROSlean/CollisionMonitor/Convert.lean#L119) | The reading of a binary64 pattern used for the conversions is `magVal` of `Bits.lean`. |

### A command without a clock reading (D15)

Task T03. [`stepNoClock`](../../core/ROSleanCore/CollisionMonitor.lean#L706)
is the transition the node takes for a command that arrives while the
`/clock` message received last carried a negative time (SPEC.md, D15, T9).
It is a function beside `step`, not a constructor of `Event`, so that
`step` and every theorem above stand as they are. The price: `run`,
`events` and the trace theorems speak of sequences of `Event`s and do not
mention such commands.

**What that means, exactly** (review CM5, finding 4; the earlier text here
said less than is the case). A processed no-clock command leaves the report
`source` on record without a zero on record, and no run of `step` from
`State.init` reaches such a state (`noClock_unreachable`). So from the
node's first processed no-clock command until its next cleanup, **no theorem
whose hypothesis is "the state is `(run cfg (State.init cfg) es).1`"
applies**: not `cm_readBack_after`, `cm_out_after`, `run_stop`,
`run_stale`, `run_pass_through`, `run_after_activation`,
`run_withheld_last_published_zero`, `reachable_inv`; also not on the
stretches without such commands. What does apply:

- every theorem about one step from an arbitrary state, and every trace
  theorem stated from an arbitrary state (`noClock_state`: only `prevZero`
  and `prevReport` change);
- every theorem with the hypothesis `Inv` (`inv_noClock`: the invariant of
  `Lifecycle.lean` is kept);
- **`mixed_from_init`**: after any sequence of events and no-clock commands
  from start-up, the node's state agrees with the state `run` reaches on
  the *substituted* events, in which every processed no-clock command is
  `deactivate, activate` and every other one is dropped, in phase, stored
  scan, shapes, record of zeros and stop stamp; the report on record can
  differ (`source` where the other has `normal`). With
  `mixed_next_velocity`: the next event is answered with the same velocity
  in both, and is a processed command in the one iff in the other. So a
  theorem stated from start-up about those fields or about a velocity holds
  for the node after no-clock commands, read on the substituted events;
  one about a state message holds except for the first processed command
  after a no-clock command.

| Theorem | Statement |
| --- | --- |
| [`noClock_velocity`](../../ROSlean/CollisionMonitor/NoClock.lean#L68), [`noClock_output`](../../ROSlean/CollisionMonitor/NoClock.lean#L61) | **Stop without a clock.** Node active and command finite: the velocity `stopSource` is published. No hypothesis on the configuration, the stored scan, the zones or `prevZero`: also with `source_timeout: 0`, with the source disabled, and after zero has been published for longer than `stop_pub_timeout`. The report is `source` unless that was reported last. |
| [`noClock_processed_iff`](../../ROSlean/CollisionMonitor/NoClock.lean#L85), [`noClock_of_not_active`](../../ROSlean/CollisionMonitor/NoClock.lean#L47), [`noClock_of_not_finite`](../../ROSlean/CollisionMonitor/NoClock.lean#L53) | The output is not `silent` iff the node is active and the command is finite: the two conditions of `processed_iff`. Otherwise nothing changes. |
| [`noClock_velocity_isStop`](../../ROSlean/CollisionMonitor/NoClock.lean#L74) | Whatever velocity is published without a clock reading is `stopSource`: never `pass`, never a slowdown. |
| [`noClock_state`](../../ROSlean/CollisionMonitor/NoClock.lean#L96), [`noClock_keeps`](../../ROSlean/CollisionMonitor/NoClock.lean#L102) | A processed one sets `prevZero := false` and `prevReport := source`. The phase, the stored scan, the shapes and the stop stamp are never changed. |
| [`inv_noClock`](../../ROSlean/CollisionMonitor/NoClock.lean#L112) | `Inv` is kept. |
| [`noClock_unreachable`](../../ROSlean/CollisionMonitor/NoClock.lean#L181), [`source_report_has_zero`](../../ROSlean/CollisionMonitor/NoClock.lean#L148) | **The state after a processed no-clock command is reached by no run from start-up**, for any configuration and any events: every state reached by events has a zero on record whenever it has the report `source` on record; after the no-clock command the report is `source` and no zero is on record. |
| [`mixed_from_init`](../../ROSlean/CollisionMonitor/NoClock.lean#L310), [`mixed_next_velocity`](../../ROSlean/CollisionMonitor/NoClock.lean#L320), [`agreesUpToReport_mixed`](../../ROSlean/CollisionMonitor/NoClock.lean#L270), [`agreesUpToReport_step`](../../ROSlean/CollisionMonitor/NoClock.lean#L210), [`AgreesUpToReport`](../../ROSlean/CollisionMonitor/NoClock.lean#L194), [`MixedEvent`](../../ROSlean/CollisionMonitor/NoClock.lean#L246), [`mixedStep`](../../ROSlean/CollisionMonitor/NoClock.lean#L254), [`substNoClock`](../../ROSlean/CollisionMonitor/NoClock.lean#L263) | **Mixed runs from start-up.** For every list of events and no-clock commands (`MixedEvent`): the state after it (`mixedStep`) and the state `run` reaches from `State.init` on the substituted events (`substNoClock`: `deactivate, activate` for each processed no-clock command, nothing for the others) have the same phase, scan, shapes, `prevZero` and `stopStamp`, and the same `prevReport` or `source` against `normal` with no zero on record (`AgreesUpToReport`). One event from two states that agree in this sense gives states that agree again, the same velocity, and "processed" in the one iff in the other; hence the velocity for whatever event follows a mixed run is the one of the substituted run. Taken from the review's `NoClockMixedRuns.lean`. |
| [`evaluate_after_noClock`](../../ROSlean/CollisionMonitor/NoClock.lean#L123) | The action demanded for a later command with a clock reading is the same as if the command without one had not arrived. |
| [`velocity_after_noClock`](../../ROSlean/CollisionMonitor/NoClock.lean#L132) | The first command with a clock reading after a processed one without has its velocity published, whatever it is: the period of R15 starts anew. |
| [`cmOnCmdNoClock_kind`](../../ROSlean/CollisionMonitor/NoClock.lean#L350), [`cmOnCmdNoClock_kind_cases`](../../ROSlean/CollisionMonitor/NoClock.lean#L362), [`cmOnCmdNoClock_keeps`](../../ROSlean/CollisionMonitor/NoClock.lean#L377), [`applyNoClock_of_ok`](../../ROSlean/CollisionMonitor/NoClock.lean#L342) | On the exported call: in the active phase a finite command reads back as kind 4, for a monitor that is ok or not; the kind is 0 or 4 in every case; the configuration and the record of the last shape call are untouched. The lifecycle node uses this call unchanged; no lookup is made for such a command. |

Not proved, and trusted (T9): *when* the node has no valid clock reading.
That is decided in C++, by the sign of `sec` in the `/clock` message.

### The command path that C calls

Task T03. `roslean_cm_on_cmd` is
[`cmOnCmdFast`](../../core/ROSleanCore/CollisionMonitorFFI.lean#L544), which
computes the command by
[`stepCmdFast`](../../core/ROSleanCore/CollisionMonitorFast.lean#L157), a
second way to compute `step` for a command (SPEC.md, "Command event").
Every theorem above is stated on `step` and `cmOnCmd`. The theorems of
[`Fast.lean`](../../ROSlean/CollisionMonitor/Fast.lean) say that this is no
loss: the two are the same function.

| Theorem | Statement |
| --- | --- |
| [`execFast_eq_exec`](../../ROSlean/CollisionMonitor/Fast.lean#L508), [`execFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L517) | **What C calls is what the theorems are about.** The exported calls ([`Call.execFast`](../../ROSlean/CollisionMonitor/Fast.lean#L498): `cmOnLifecycle`, `cmOnScan`, `cmOnShape`, `cmOnCmdFast`) and the definitions the theorems are stated on (`Call.exec`, with `cmOnCmd`) are the same function of monitor and call; so folding either over any sequence of calls, from any monitor, gives the same monitor, every field. No hypothesis. |
| [`stepCmdFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L306) | **The fast step is `step`.** For every configuration (accepted or not), state, time, velocity, zone vertices, number `n` of coordinates (or none) and functions `f`, `g` with `f i = scanPointMm (g i)` for all `i`: `stepCmdFast cfg st now v n f zoneVerts = step cfg st (.cmd now v (n.map fun n => (List.range n).map g) zoneVerts)`. State and output. The command call of T06 uses it with `n` the number of pairs in the array, `f := pointAt points` and `g := pairAt points`. |
| [`cmOnCmdFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L490), [`applyCmdFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L479) | The exported command call returns what `cmOnCmd` returns, on every monitor and for all arguments: it is `Monitor.apply` on the command event that carries the coordinate pairs of the array (or none) and the decoded zone vertices. The coordinates are read from the bytes into machine words and are never decoded into a list; the zone vertices, which are few, are decoded as lists. |
| [`pointsBy_toList`](../../ROSlean/CollisionMonitor/Fast.lean#L56), [`sourcePointsBy_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L64) | The points produced in one pass are `Scan.points`; `sourcePointsBy` is `sourcePoints`. |
| [`coordMm64_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L430), [`coordMagMm64_toNat`](../../ROSlean/CollisionMonitor/Fast.lean#L363), [`word64At_toNat`](../../ROSlean/CollisionMonitor/Fast.lean#L331), [`coordMmAt_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L467), [`pointAt_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L472) | **The conversion on machine words.** Eight bytes read into a `UInt64` are the pattern `u64At` reads, for every array and offset. `coordMm64`, which takes sign and magnitude apart and shifts on a 64-bit word, is `coordMm` of the word's value, for every one of the 2⁶⁴ words (no overflow: the significand is below 2⁵³ and the factor 1000 below 2¹⁰; a shift of 64 bits or more is treated as the zero it is). So the point read from a record is `scanPointMm` of the record's coordinates. |
| [`nearSegment_box`](../../ROSlean/CollisionMonitor/Fast.lean#L129), [`nearBoundary_near`](../../ROSlean/CollisionMonitor/Fast.lean#L218), [`boxOf_spec`](../../ROSlean/CollisionMonitor/Fast.lean#L189), [`mem_of_mem_edges`](../../ROSlean/CollisionMonitor/Fast.lean#L202) | **The bounding box.** A point within √5 mm of a segment (`nearSegment`) lies, on each axis, at most 2 mm outside the range the segment's end points span. So a point within the band of D4 around a polygon's boundary is within 2 mm of the polygon's bounding box, and the band test that the fast path skips for all other points would have said "no". |
| [`containsEdges_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L237), [`countIn_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L252), [`countFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L265), [`triggeredFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L271), [`firstStopFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L275), [`bestSlowdownFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L287), [`zonesActionFast_eq`](../../ROSlean/CollisionMonitor/Fast.lean#L292) | The zone tests with the edges and the box computed once are `Shape.contains`; the counts, the triggers and the selected action are those of `Zone.count`, `Zone.triggered` and `zonesAction`. |

The box is used for the band only. The crossing count of R10 is made for
every point as before: that a point left of the box is crossed an even
number of times is true of a closed polygon but is not proved here.
`cmOnCmdNoClock` has no fast variant; it needs none.

Two limits of these theorems:

- **Timeouts are tested, not proved, to be upstream's.** `timeoutNs`
  reproduces `(int64_t)(seconds * 1e9)` by rounding an exact product to the
  nearest binary64 value in integer arithmetic (`nearest`). That this is the
  IEEE-754 product is checked against `rclcpp::Duration` and against the
  hardware on 530,000 values with no disagreement, and not proved: there is
  no formal IEEE-754 multiplication here to prove it against. What is proved
  is which timeouts are refused.
- **What the arrays of a command hold is the shim's word.** The end-to-end
  theorems hold for every sequence of calls and all bytes. That pair `i` of
  a command is the image of reading `i` of the scan the model has stored,
  under the transform upstream would use for this command, and that the
  vertices handed over for a zone are those of the message the model last
  accepted for it, is the work of `transforms.hpp` and of the node (trusted:
  T2, T6 of SPEC.md; tested in `smoke_transforms` and by the scenarios
  of `scenario_nav2.py`). The model checks the number of pairs and of
  vertices, and nothing else about them.
- **Commands without a clock reading are not in `Call`.** `cmOnCmdNoClock`
  is not an event of the model; a sequence of calls that contains such
  commands is not covered by `foldl_exec_run`, and after the first processed
  one no theorem stated from start-up applies as it stands. What holds is
  `mixed_from_init` on the model's states ("A command without a clock
  reading"); it is not lifted to the exported functions: there is no
  "`cm_readBack_after` for mixed sequences of calls".
- **Zone names are the shim's.** A report carries a zone index; the name in
  the state message is the shim's table (T8), and that names are distinct
  and none is `invalid source` or empty (ND5) is checked in C++ when the
  parameters are read, not in Lean.

## What is not proved

- **That the model equals upstream `nav2_collision_monitor`.** The rules were
  read from the source and tested by the reviewer; the known differences are
  D1–D15 and ND1–ND13 of the SPEC.
- **The slowdown multiplication** (T1), as said above. No theorem speaks
  about a published velocity value other than "unchanged" and "zero", and
  those two only through the shim's contract.
- **That the points a command carries are the right ones** (T2, T6): tf2's
  tree, interpolation and product, the shim's lookup, the shim's evaluation
  of N25 in `double`, and that the pairs are those of the stored scan's
  readings, in order. The theorems hold for whatever coordinates each
  command carries, and for none. What the model does check: that there is
  one pair per reading, and which readings count.
- **That the shim's floating-point point is within 0.04 mm of upstream's**
  (T7), on which "a point upstream counts is always counted" rests. See
  "The coordinates a command carries, and the margin".
- **The C++ shim** (T2, T3, T5): the scan's points in metres (`cos`, `sin`
  and which library entry the compiler calls for them, the angle, the transform of N25 and how it is compiled, and which transform
  is looked up: `transforms.hpp`), the parsing of the `points`
  text, the clock, and that the shim passes values unaltered and publishes
  what the output says. The conversions to millimetres and nanoseconds and
  the refusals are not in the shim.
- **The topological half of the polygon safe-direction claim**, and the
  "other direction" tolerances (see Geometry).
- **That `magVal` is IEEE-754** (see the bit-pattern helpers), and likewise
  the reading of a pattern as a number that `coordMm` and `vertexCoordMm`
  rest on.
- **That a demanded stop is always published.** It is not (R15); see "What
  the robot receives". Likewise the node publishes nothing when no command
  arrives (R1), nothing for a non-finite command (R2), and nothing while it
  is not active (N4), also while a stop is demanded (review finding 14).
- **That a topic zone has a shape, or a current one.** A stop zone fed by a
  topic takes no action until its first accepted message (N32, "No shape"),
  and a shape in another frame is where the last command that carried its
  vertices put it (N34). Both are upstream's behaviour and both
  are fail-open.
- **That a missing transform stops the robot when `source_timeout` is 0.**
  It does not (N27; `sourcePoints_of_no_transform_timeout_zero`).
- **That an accepted configuration protects anything.** `Config.ok` accepts
  no zones, a circle of radius 0, a degenerate polygon (review finding 15),
  and a configuration whose only zone is a topic zone.
  The node warns about the first three at start-up (and about a source
  timeout of 0, a disabled source, `min_points` of 0 or less and a slowdown
  ratio of 1 or more); it does not refuse them.
- **That a topic zone's vertices in the base frame are rounded like a fixed
  polygon's.** They are not the same function: a fixed vertex is a `float`
  parameter (`vertexMm`), a topic vertex a `double` from the shim
  (`vertexCoordMm`). Each is proved to be within 1/2 mm of its number.
- **That time moves forward.** No theorem assumes it. If the clock jumps
  back, a stale scan counts as fresh and zeros keep being published, as the
  theorems state.
- **Anything about the time spent waiting for a transform** (N26), the bond,
  or the lifecycle manager: the model has the outcome of each lookup, and
  the transitions as events.
- **Anything about concurrent callbacks.** The model handles one event at a
  time.
