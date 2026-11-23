# Collision monitor: the main theorems

What is proved about the Lean core of the collision monitor, in plain words,
with a link to each statement. The theorems are in
`ROSlean/CollisionMonitor/`, are checked by `lake build` in the repository
root, and use no `sorry`, no `native_decide` and no axioms beyond Lean's
three standard ones (`propext`, `Classical.choice`, `Quot.sound`; checked
with `#print axioms` on every public theorem). Rule numbers (R…),
differences (D…), assumptions (A…) and trusted parts (T…) refer to
[SPEC.md](SPEC.md).

Links point to line numbers as of 2026-11-23; if a link lands a few lines
off, search the file for the theorem name.

| File | Content |
| --- | --- |
| [`Basic.lean`](../../ROSlean/CollisionMonitor/Basic.lean) | Which points are tested, which zone is selected |
| [`Properties.lean`](../../ROSlean/CollisionMonitor/Properties.lean) | One step: the output, the state, the properties of the SPEC |
| [`Trace.lean`](../../ROSlean/CollisionMonitor/Trace.lean) | Event sequences: the trace invariant, the history, `stop_pub_timeout` |
| [`Geometry.lean`](../../ROSlean/CollisionMonitor/Geometry.lean) | Circle and polygon tests, the margin, saturated points, `Config.ok` |
| [`Bits.lean`](../../ROSlean/CollisionMonitor/Bits.lean) | The IEEE-754 bit-pattern helpers |

## What the theorems are about

The model is in
[`core/ROSleanCore/CollisionMonitor.lean`](../../core/ROSleanCore/CollisionMonitor.lean).

| Name | Meaning |
| --- | --- |
| [`step`](../../core/ROSleanCore/CollisionMonitor.lean#L369) | Processes one event (a scan or a command). Returns the new state and the output. This is what the node calls. |
| [`run`](../../core/ROSleanCore/CollisionMonitor.lean#L385) | `step` over a list of events. |
| [`Output`](../../core/ROSleanCore/CollisionMonitor.lean#L354) | `silent` (publish nothing) or `publish a` for an action `a`. |
| [`Action`](../../core/ROSleanCore/CollisionMonitor.lean#L275) | `pass` (command unchanged), `slowdown k r` (multiply by the double with bit pattern `r`; zone `k`), `stopZone k` (zero; zone `k`), `stopSource` (zero; no usable scan). |
| [`evaluate`](../../core/ROSleanCore/CollisionMonitor.lean#L340) | The action demanded at a given time. It does not depend on the command. |
| [`sourcePoints`](../../core/ROSleanCore/CollisionMonitor.lean#L300) | The points the zones are tested against, or `none` if the source is invalid. |
| [`Zone.triggered`](../../core/ROSleanCore/CollisionMonitor.lean#L264) | The zone is enabled and at least `minPoints` points are inside its shape. |
| [`Action.yieldsZero`](../../core/ROSleanCore/CollisionMonitor.lean#L347) | The velocity the action yields for a command is zero: a stop, or a zero command, or a ratio of zero. |
| `prevZero`, `stopStamp` | The two state fields behind `stop_pub_timeout` (R15). |

"The output of a command" below is what `step` returns for a command `v` at
node-clock time `now` in a state `st`. Unless said otherwise the state is
arbitrary, nothing is assumed about the clock, and nothing is assumed about
the configuration (`Config.ok` is needed only where stated).

Two predicates name the zone that wins:
[`IsFirstStop zones pts k`](../../ROSlean/CollisionMonitor/Basic.lean#L101)
(zone `k` is a triggered stop zone and no stop zone before it is triggered)
and
[`IsBestSlowdown zones pts k r`](../../ROSlean/CollisionMonitor/Basic.lean#L180)
(zone `k` is a triggered slowdown zone with ratio pattern `r` below that of
1.0, no triggered slowdown zone has a smaller ratio pattern, and none before
`k` has the same). Each holds for at most one `k`
([`IsFirstStop.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L109),
[`IsBestSlowdown.unique`](../../ROSlean/CollisionMonitor/Basic.lean#L189)),
which is the answer to "which zone is named when several trigger".

## The two central theorems

**When something is published, and when not.**
[`output_eq_silent_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L179):
the output of a command is `silent` if and only if

1. the command has an infinite or NaN component (R2), or
2. the demanded velocity is zero, the velocity demanded for the previous
   command was zero too, and the stop stamp is more than `stop_pub_timeout`
   before `now` (R15).

[`output_eq_publish_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L149):
otherwise, and only then, the output is `publish a`, where `a` is the action
`evaluate` demands
([`output_eq_silent_or_publish`](../../ROSlean/CollisionMonitor/Properties.lean#L202):
nothing else is ever published).

**Which action is demanded.** Four if-and-only-if statements, one per action:

| Action | Demanded iff | Theorem |
| --- | --- | --- |
| `stopSource` | the source is enabled, its timeout is not 0, and no scan is stored or the stored scan's stamp is more than the timeout before `now` | [`evaluate_eq_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L47) |
| `stopZone k` | the source is valid and `k` is the first stop zone triggered by its points | [`evaluate_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L59) |
| `slowdown k r` | the source is valid, **no stop zone is triggered**, and `k` is the first triggered slowdown zone with the smallest ratio pattern, `r`, which is below the pattern of 1.0 | [`evaluate_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L71) |
| `pass` | the source is valid, no stop zone is triggered, and no slowdown zone with ratio below 1 is triggered | [`evaluate_eq_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L83) |

"The source is valid, with points `pts`" is
[`sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L81):
the source is disabled and there are no points; or the timeout is 0, no scan
is stored, and there are no points; or a scan is stored, its stamp is not
more than the timeout before `now`, and `pts` are the points of its valid
readings.

Combined, they say when each output occurs:
[`output_eq_publish_pass_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L241),
[`output_eq_publish_slowdown_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L253),
[`output_eq_publish_stopZone_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L267),
[`output_eq_publish_stopSource_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L279).

Because these are equivalences, they also give the "only if" directions that
a list of safety properties alone does not: a monitor that is silent
whenever a stop is demanded, or that stops whenever a slowdown zone
triggers, or that always stops, satisfies "never publishes more than
allowed" but not these. Read off separately:
[`evaluate_stopZone_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L373)
(a stop named after zone `k` only if zone `k` is an enabled stop zone with at
least `minPoints` points inside),
[`evaluate_slowdown_only_if`](../../ROSlean/CollisionMonitor/Properties.lean#L384),
[`evaluate_isStop_iff`](../../ROSlean/CollisionMonitor/Properties.lean#L394)
(zero is demanded whatever the command iff the source is invalid or a stop
zone is triggered).

## The property table

One step, from an arbitrary state.

| Property | Theorem | Statement |
| --- | --- | --- |
| Stop | [`stop`](../../ROSlean/CollisionMonitor/Properties.lean#L325) | If the source is valid and some stop zone is triggered, the action is `stopZone k` for the first such zone, and the output is `publish (stopZone k)`, unless the command is not finite or (`prevZero` and the stop stamp is more than `stop_pub_timeout` old); in those two cases, and only then, it is `silent`. |
| A demanded stop is published | [`output_of_isStop`](../../ROSlean/CollisionMonitor/Properties.lean#L300) | The same for any stop, by a zone or for an invalid source. |
| Stop wins over slowdown | [`stop_not_pass_or_slowdown`](../../ROSlean/CollisionMonitor/Properties.lean#L340) | With a triggered stop zone the output is never `publish pass` or `publish (slowdown …)`, wherever the stop zone stands in the configuration. |
| Stale data | [`stale`](../../ROSlean/CollisionMonitor/Properties.lean#L358) | Source enabled, timeout not 0, and no scan or a scan stamped more than the timeout ago: the action is `stopSource`, published or silent exactly as for Stop. |
| … needs its hypotheses | [`evaluate_ne_stopSource_of_off`](../../ROSlean/CollisionMonitor/Properties.lean#L425) | With the source disabled or timeout 0, `stopSource` is never demanded (R6, R8). |
| Bad data (a) | [`output_filter_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L461), [`points_of_not_validBeam`](../../ROSlean/CollisionMonitor/Properties.lean#L473) | Readings that are not valid never change a decision: with them removed from the stored scan, the output of every command is the same; one such reading can be removed whatever point it carries. More generally two scans with the same stamp and the same valid points are answered alike ([`output_congr_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L442)). |
| … what "valid" is | [`validBeam_iff_value`](../../ROSlean/CollisionMonitor/Bits.lean#L187) | A reading is valid iff neither it nor a range limit is NaN and the number it denotes lies between the limits, inclusive (R4). |
| Bad data (b) | [`step_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Properties.lean#L108) | A command with an infinite or NaN component: output `silent`, state unchanged. |
| Bad data (c), Totality | by definition | `step` and `run` are total functions; Lean accepts them without `partial` or `unsafe`. |
| No amplification | [`no_amplification`](../../ROSlean/CollisionMonitor/Properties.lean#L503) | Whatever is published is `pass`, a stop, or `slowdown k r` where `r` is below the pattern of 1.0 and is the configured ratio of zone `k`. Holds for every configuration. |
| … as a number | [`published_ratio_in_unit_interval`](../../ROSlean/CollisionMonitor/Properties.lean#L518), [`lt_one_iff`](../../ROSlean/CollisionMonitor/Bits.lean#L239) | Such an `r` has its sign bit clear and denotes a number in `[0, 1)`. |
| Slowdown bound | [`slowdown_bound`](../../ROSlean/CollisionMonitor/Properties.lean#L529), [`evaluate_slowdown_le`](../../ROSlean/CollisionMonitor/Properties.lean#L552) | If a slowdown zone with ratio pattern `rz` below 1.0 is triggered, the action is a stop by a zone or a slowdown with ratio pattern at most `rz`; a demanded ratio is at most that of *every* triggered slowdown zone. |
| Pass-through | [`pass_through`](../../ROSlean/CollisionMonitor/Properties.lean#L566), [`pass_through_of_nonzero`](../../ROSlean/CollisionMonitor/Properties.lean#L591) | Finite command, valid source, no zone triggered: the action is `pass`; it is published, except that a zero command is dropped under R15. A non-zero command is always published unchanged. |
| Monotonicity | [`zonesAction_stop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L611), [`zonesAction_slowdown_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L627), [`evaluate_isStop_mono`](../../ROSlean/CollisionMonitor/Properties.lean#L657) | Adding points never lifts a stop (the zone named stays or moves earlier), and never turns a slowdown into `pass` or into a larger ratio. Adding readings to the stored scan never lifts a stop. |
| State | [`step_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L103), [`step_cmd_scan`](../../ROSlean/CollisionMonitor/Properties.lean#L113), [`step_cmd_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L122), [`step_cmd_stopStamp`](../../ROSlean/CollisionMonitor/Properties.lean#L130) | A scan event stores the scan and changes nothing else. A command never changes the scan. After a finite command `prevZero` is "the demanded velocity was zero" (published or not), and `stopStamp` is `now` if it was zero and the previous one was not, else unchanged. |
| R15, simple cases | [`output_of_not_yieldsZero`](../../ROSlean/CollisionMonitor/Properties.lean#L215), [`output_of_not_prevZero`](../../ROSlean/CollisionMonitor/Properties.lean#L222), [`output_of_within_stopPubTimeout`](../../ROSlean/CollisionMonitor/Properties.lean#L230) | A finite command is answered if the demanded velocity is not zero, or the previous one was not zero, or `now` is at most `stopStamp + stop_pub_timeout`. |

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

## Over sequences of events

In [`Trace.lean`](../../ROSlean/CollisionMonitor/Trace.lean).

| Theorem | Statement |
| --- | --- |
| [`run_getElem?`](../../ROSlean/CollisionMonitor/Trace.lean#L71) | **Trace invariant.** The `n`-th output of any run from any state is the single-step output for the `n`-th event in the state reached after the first `n` events. Every theorem above therefore holds at every point of every run. [`run_append`](../../ROSlean/CollisionMonitor/Trace.lean#L46), [`run_concat`](../../ROSlean/CollisionMonitor/Trace.lean#L56): running two sequences one after the other; one more event. |
| [`run_scan`](../../ROSlean/CollisionMonitor/Trace.lean#L106) | After any run the stored scan is the one received last ([`lastScan`](../../ROSlean/CollisionMonitor/Trace.lean#L99)), whatever the stamps say (R5). |
| [`run_evaluate`](../../ROSlean/CollisionMonitor/Trace.lean#L130), [`run_sourcePoints_eq_some_iff`](../../ROSlean/CollisionMonitor/Trace.lean#L139) | The action demanded after any events since start-up depends on the history only through the scan received last; the points tested, stated on the events alone. |
| [`run_stop`](../../ROSlean/CollisionMonitor/Trace.lean#L161) | **Stop.** After any events since start-up: if the scan received last is not too old and triggers a stop zone, the next command gets `publish (stopZone k)` for the first such zone, or silence in the two cases of `output_eq_silent_iff`. |
| [`run_stale`](../../ROSlean/CollisionMonitor/Trace.lean#L181) | **Stale data.** Likewise with `publish stopSource` when no scan has arrived or the last one is too old. |
| [`run_cmd_of_not_finite`](../../ROSlean/CollisionMonitor/Trace.lean#L196) | **Bad data.** A non-finite command leaves no trace: the run with it is the run without it, plus one `silent`. |
| [`run_no_amplification`](../../ROSlean/CollisionMonitor/Trace.lean#L206) | **No amplification.** Every action published anywhere in any run is `pass`, a stop, or a slowdown with a ratio pattern below that of 1.0, configured for the zone named. |
| [`run_slowdown_bound`](../../ROSlean/CollisionMonitor/Trace.lean#L227) | **Slowdown bound**, with hypotheses on the scan received last. |
| [`run_pass_through`](../../ROSlean/CollisionMonitor/Trace.lean#L243) | **Pass-through.** If the scan received last is not too old and triggers no zone, a finite non-zero command is published unchanged. |
| [`run_scan_output`](../../ROSlean/CollisionMonitor/Trace.lean#L90) | A scan event never produces output (R1). |

### `stop_pub_timeout`: when the node is silent although a stop is demanded

R15 is upstream's rule and the model reproduces it: zero is published for
`stop_pub_timeout` after it first appears, then nothing, until a non-zero
velocity is demanded. Three theorems say exactly what that means.

| Theorem | Statement |
| --- | --- |
| [`run_silent_last_published_zero`](../../ROSlean/CollisionMonitor/Trace.lean#L414) | **Silence follows a published zero.** In any run from start-up, if a finite command gets no answer, then the message published last before it was a zero: an earlier command was answered with an action that yields zero for it, and nothing was published in between ([`ZeroPublishedLast`](../../ROSlean/CollisionMonitor/Trace.lean#L341)). |
| [`zero_period`](../../ROSlean/CollisionMonitor/Trace.lean#L313) | **A period of zero.** If a finite command at time `t` is the first for which zero is demanded, and everything published afterwards is zero, then a later finite command at `now'` is answered with the demanded action unless that is again zero and `t + stop_pub_timeout < now'`. So zeros are published from `t` up to and including `t + stop_pub_timeout`, whatever their cause, and a non-zero demand is always published. |
| [`step_zero_start`](../../ROSlean/CollisionMonitor/Trace.lean#L258), [`run_zero_streak`](../../ROSlean/CollisionMonitor/Trace.lean#L272) | What the state records: the first zero is published and sets the stop stamp to its time; during the period `prevZero` stays set and the stop stamp does not move. |
| [`run_first_command_answered`](../../ROSlean/CollisionMonitor/Trace.lean#L424) | The first finite command after start-up is always answered. |

**What the robot receives.** The statement "a demanded stop is always
published" is false, for upstream and for the model, and the theorems do not
claim it. The scenario of review finding 5 is real (it is an example in
[`Examples.lean`](../../ROSlean/CollisionMonitor/Examples.lean), checked by
`decide`): the planner sends zero commands for longer than
`stop_pub_timeout`, so the node has published zero for one second and then
gone silent; now an obstacle enters a stop zone and the planner starts to
command 0.5 m/s. A stop is demanded, and **nothing is published, not once**,
because the idle zeros and the stop are one period of zero. What
`run_silent_last_published_zero` guarantees is that the last message the base
received is a zero (here: one of the idle zeros). A base that holds its last
command stays stopped. A base that needs fresh zero messages to stay
stopped, or that takes commands from elsewhere in the meantime, is not
protected by this node during the silence; that is assumption A3, and open
decision 3 of the SPEC (publish zero for as long as a stop is demanded)
would remove it.

The invariant needs the run to start at `State.init`. From an arbitrary
state with `prevZero` set, silence can occur with nothing published before.

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
| [`config_ok_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L585), [`shape_ok_iff`](../../ROSlean/CollisionMonitor/Geometry.lean#L566) | **What `Config.ok` guarantees:** every circle has radius at most 1,000,000 mm; every polygon has at least four vertices, each within 999,997 mm of both axes; every slowdown ratio has its sign bit clear. |
| [`contains_eq_false_of_far`](../../ROSlean/CollisionMonitor/Geometry.lean#L597) | **A saturated point is in no zone.** If a shape passes `Shape.ok`, a point with a coordinate of magnitude 1,000,000 mm or more is not inside it (D12). The SPEC relies on this for readings that are NaN, infinite or 1000 m away. |
| [`zonesAction_filter_inRange`](../../ROSlean/CollisionMonitor/Geometry.lean#L647) | With an accepted configuration, saturated points do not change the action. |

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
3.5 mm outside a circle, 4.4 mm outside a polygon, can count as inside), and
anything about the conversions themselves, which happen in C++.

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
| [`firstStop_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L141), [`firstStop_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L161) | The stop loop returns the first triggered stop zone, or nothing iff none is triggered. |
| [`bestSlowdown_eq_some_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L338), [`bestSlowdown_snd_eq_none_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L356) | The slowdown loop returns the best slowdown zone, or nothing iff no slowdown zone with a ratio below 1 is triggered. |
| [`zonesAction_eq_stopZone_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L380), [`zonesAction_eq_slowdown_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L391), [`zonesAction_eq_pass_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L403) | The action for given points, by cases. |
| [`zonesAction_congr`](../../ROSlean/CollisionMonitor/Basic.lean#L423) | The action depends on the points only through which zones they trigger. |
| [`triggered_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L40), [`triggered_nil_iff`](../../ROSlean/CollisionMonitor/Basic.lean#L58) | A zone triggers iff it is enabled and has at least `minPoints` points inside; without points, only a zone with `minPoints = 0` triggers (R11). |
| [`evaluate_congr`](../../ROSlean/CollisionMonitor/Properties.lean#L40) | The demanded action reads the state only through the stored scan. |

## What is not proved

- **That the model equals upstream `nav2_collision_monitor`.** The rules were
  read from the source and tested by the reviewer; the known differences are
  D1–D14 of the SPEC.
- **The slowdown multiplication** (T1), as said above. No theorem speaks
  about a published velocity value other than "unchanged" and "zero", and
  those two only through the shim's contract.
- **Anything about the C++ shim and the C interface** (T2, T3, T5): the scan
  conversion with `cos` and `sin`, truncation and saturation to millimetres,
  the configuration conversions (review findings 1, 2, 7, 13 are about
  these), the clock, and that the shim publishes what the action says. The
  C interface is being written separately; its theorems are not in these
  files.
- **The topological half of the polygon safe-direction claim**, and the
  "other direction" tolerances (see Geometry).
- **That `magVal` is IEEE-754** (see the bit-pattern helpers).
- **That a demanded stop is always published.** It is not (R15); see "What
  the robot receives". Likewise the node publishes nothing when no command
  arrives (R1) and nothing for a non-finite command (R2), also while a stop
  is demanded (review finding 14).
- **That an accepted configuration protects anything.** `Config.ok` accepts
  no zones, a circle of radius 0, a degenerate polygon (review finding 15).
- **That time moves forward.** No theorem assumes it. If the clock jumps
  back, a stale scan counts as fresh and zeros keep being published, as the
  theorems state.
- **Anything about concurrent callbacks.** The model handles one event at a
  time.
