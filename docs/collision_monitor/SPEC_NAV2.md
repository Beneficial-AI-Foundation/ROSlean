# Collision monitor: the Nav2 extension, design record and what is still missing

[SPEC.md](SPEC.md) describes the current behavior. This document
records the design of the lifecycle interface, TF handling, and
topic-fed zones. See [§8.6](#86-what-a-user-changes-to-run-it-in-nav2)
for Nav2 setup and [§12](#12-what-still-separates-this-from-nav2s-default-configuration) for remaining gaps.

The design sections keep their original numbering. For current
behavior of unusable polygon messages, transform waits, and TF buffer
resets, use ND6, N26, and N28 in [SPEC.md](SPEC.md).

## Sources, and how each statement is marked

Moved with the rules: [SPEC.md](SPEC.md), "Rules of the Nav2 extension as
implemented upstream".

## 1. Upstream's rules

N1–N36: moved to [SPEC.md](SPEC.md), "Rules of the Nav2 extension as
implemented upstream", with the corrections of the review CM5 in N16, N26,
N28, N29 and N33.

## 2. What the documentation does not say

Moved to [SPEC.md](SPEC.md), "Things the source does that the documentation
does not say".

## 3. The model after the extension

Additions to `core/ROSleanCore/CollisionMonitor.lean`. Names are proposals;
T05 may choose others but not other meanings. Everything stays `Nat`, `Int`,
`Bool` and lists; no `Float`; `step` stays total.

(T05, T05b) The names chosen: a zone's shape is `ZoneShape.fixed s` or
`ZoneShape.topic`; a coordinate pair as the shim hands it over is `Coords`
(two binary64 patterns, metres, base frame); `State.init cfg` takes the
configuration; the zones with their shapes are `liveConfig cfg st
zoneVerts`; the shapes after a command are `shapesAfter`; the acceptance of
a shape message is `acceptShape`; the report of an action is
`Action.report`; `Output.velocity` and `Output.report` read an output. The
points, the two conversions the model applies (`scanPointMm`,
`vertexPointMm`) and the bit-pattern reading are in
`core/ROSleanCore/CollisionMonitorPoint.lean`.

### 3.1 Data

- (T05b) **There is no transform in the model.** `Xform`, `Xform.usable`,
  `Xform.pointMm` and `Xform.vertexMm` of T04 and T05 are gone (section 5).
- **`Coords`**: a pair of binary64 bit patterns, the `x` and `y` in metres
  of a point **in the base frame**, as the shim computed them (N25).
- **`Beam`** carries the range pattern only. Where a reading's point lies
  is not stored with the scan: it depends on the transform at the time of
  each command (N22, N24) and arrives with the command.
- **`scanPointMm c : Point`**: each coordinate of `c` in millimetres,
  truncated toward zero and saturated at `±coordLimit` (`coordMm`, as before
  the extension; a NaN is `+coordLimit`). **`vertexPointMm c : Point`**:
  each coordinate rounded to the nearest millimetre, a half away from zero
  (`vertexCoordMm`), for the vertices of a topic zone; a coordinate that is
  not finite or is 1024 m or more becomes `±coordLimit`, which no shape may
  reach, so such a vertex is always refused (E7, ND6).
- **`Scan.fits s c`**: the list `c` has exactly one pair per reading of
  `s`. **`Scan.points s c`**: reading `i` is paired with pair `i` of `c`;
  the pairs of the readings that are not valid (R4) are dropped; the others
  become points by `scanPointMm`, in scan order. Which readings count is
  decided here, from the stored ranges and limits, not by the shim.
- **Zones.** A zone's shape is `fixed (s : Shape)` or `topic`. A topic zone
  has no shape in the configuration.
- **`TopicShape`**: `foreign : Bool` (the message's frame is not the base
  frame), `current : List Point` (the vertices in the base frame, in
  millimetres: upstream's `poly_`). (T05b) The vertices as received are no
  longer kept in the state: the shim keeps the message and recomputes the
  base-frame vertices for each command (N34).
- **`Report`**: `normal`, `stop (zone)`, `slowdown (zone)`, `source`: what a
  state message says (N13).
- **`Phase`**: `unconfigured`, `inactive`, `active`, `finalized`.
- **`State`** gains `phase` (initially `unconfigured`), `prevReport`
  (initially `normal`) and `shapes : List (Option TopicShape)`, one entry
  per zone, `none` for a fixed zone and for a topic zone without shape.
  (T05) For the list to have one entry per zone from the start, the initial
  state is a function of the configuration: `State.init cfg`.
- **`Output`**: `silent`, or `answer (velocity : Option Action) (report :
  Option Report)` for a processed command. `answer none _` is today's
  silence of R15; `answer (some a) _` is today's `publish a`. The
  distinction between `silent` and `answer none none` is what tells the
  shim whether to publish the zone polygons (N16).

### 3.2 Events and `step`

Each rule names what upstream does (N…) and where the model differs (ND…).

- **E1 `configure`.** `unconfigured` → `inactive`. In any other phase: no
  change. Output `silent`. (N3)
- **E2 `activate`.** `inactive` → `active`. Otherwise no change. `silent`.
  (N5)
- **E3 `deactivate`.** `active` → `inactive`, `prevZero := false`,
  `prevReport := normal`. Otherwise no change. `silent`. (N6)
- **E4 `cleanup`.** `inactive` → `unconfigured`, `scan := none`, every
  entry of `shapes` `:= none`. Otherwise no change. `silent`. (N7)
- **E5 `shutdown`.** Any phase but `finalized` → `finalized`. `silent`.
  From `finalized` no event changes the state and every output is `silent`
  (ND4).
- **E6 `scan now s`.** In `inactive` and `active`: `scan := some s`. In
  `unconfigured` and `finalized`: no change. `silent`. (N4, R5)
- **E7 `shape k verts foreign`** (T05b), with `verts : Option (List
  Coords)` the vertices of the message **in the base frame**, as the shim
  computed them (N33, step 3), or `none` if the transform from the message's
  frame could not be looked up, and `foreign` saying that the message's
  frame is not the base frame. The message is **accepted** iff the phase is
  `inactive` or `active`, zone `k` exists and is a topic zone, `verts = some
  vs`, `vs` has at least three entries, and every vertex of `vs.map
  vertexPointMm` lies within the coordinate limit of `Shape.ok` (which
  excludes every coordinate that is not finite). If accepted: `shapes[k] :=
  some { foreign, current := vs.map vertexPointMm }`. Otherwise no change.
  `silent` in both cases. (N33, ND6)
  (T09, after the review CM5, finding 1) **Changed.** A message that meets
  every condition but the last (one of at least three vertices is not finite
  or beyond the coordinate limit) is no longer ignored: it is accepted and
  sets `shapes[k] := some TopicShape.unusableMark` (no vertices, `unusable
  := true`), and a zone with that mark is triggered by every command while
  it is enabled. Upstream's zone with a NaN vertex still catches points;
  ignoring the message could pass where upstream stops. SPEC.md, "The
  model", "Properties" ("Unusable message") and ND6.
- **E8 `cmd now v scanPts zoneVerts`** (T05b), with `scanPts : Option (List
  Coords)` (the base-frame coordinates of the readings of the stored scan,
  one pair per reading in scan order, as the shim computed them under the
  transform it looked up for this command; `none` if the lookup failed or
  was not made) and `zoneVerts : List (Option (List Coords))` (one entry per
  zone: the base-frame vertices of that zone's stored message under the
  transform looked up for this command, or `none`).
  1. Phase not `active`: no change, `silent`. (N4)
  2. `v` not finite: no change, `silent`. (R2)
  3. **Source** (`sourcePoints`, R6–R8 with N22 and N27): `some []` if the
     source is disabled; if no scan is stored, `some []` when the timeout is
     0 and `none` otherwise; if the scan is too old (timeout not 0), `none`;
     otherwise, if `scanPts = some c` and `s.fits c`, `some (s.points c)`;
     otherwise (no coordinates, or not one pair per reading) `some []` when
     the timeout is 0 and `none` when it is not.
  4. **Shapes** (N34, ND3): if the source is `none`, unchanged. Otherwise,
     for every enabled topic zone `k` with `shapes[k] = some sh` and
     `sh.foreign`: if `zoneVerts[k] = some vs`, `vs` has as many entries as
     `sh.current`, and every vertex of `vs.map vertexPointMm` is within the
     coordinate limit, `current` is replaced by that list; otherwise it
     stays.
  5. **Action** (`evaluate`): `stopSource` if the source is `none`; else
     `zonesAction` (R12, R13) over the zones, where a zone is triggered iff
     it is enabled, **has a shape** (a fixed one, or `shapes[k] = some _`
     after step 4), and at least `minPoints` points are inside it; a topic
     zone's shape is the polygon `current`, with the margin of D4. (N32)
  6. **Report** (N13, N14): `r := source` for `stopSource`; `stop k` for
     `stopZone k`; for `slowdown k _`: `slowdown k` if `v` is not zero in
     the three components and `normal` if it is; `normal` for `pass`. The
     output's report is `some r` if `r ≠ st.prevReport` and `none` if it is
     equal.
  7. **Velocity** (R15, unchanged): `none` iff the velocity is zero,
     `prevZero` holds and `stopStamp + stopPubTimeout < now`; else `some
     action`.
  8. New state: `prevZero`, `stopStamp` as today; `prevReport := r`;
     `shapes` from step 4. Output `answer velocity report`.

  (T05b) Two checks in steps 3 and 4 are there because the coordinates now
  come from the shim: a list of scan coordinates that is not one pair per
  reading counts as a failed lookup, and zone vertices that are not as many
  as the shape has are not taken. A shim that works as specified produces
  neither; the model does not rely on it.

`step` for a monitor that is not ok stays as it is ("Fail closed" in
SPEC.md): every command in the `active` phase is answered `stopSource`,
whatever else; in other phases `silent`.

### 3.3 What the model does not contain

The wait of N26 (the event carries only its outcome), frame names and (T05b)
the transform itself (the shim resolves a frame into a transform, applies it
and hands over points, or `none`), the scan's angles and the scan-frame
points (R3), the bond, publishers and QoS,
the polygons shown for `visualize`, zone names (the shim maps a zone index
to its name).

## 4. Properties

Proved by T05 about `step` and lifted to `run`, in the manner of SPEC.md,
"Properties", and reworked by T05b for option A. `cfg`, `st`, `now`, `v`,
the coordinates are arbitrary unless restricted; no property needs
`Config.ok` unless it says so. "The output of the command" is `(step cfg st
(.cmd now v scanPts zoneVerts)).2`. *Source invalid* means:
`cfg.sourceEnabled`, `0 < cfg.sourceTimeout`, and either `st.scan = none`,
or `st.scan = some s` with `s.stamp + cfg.sourceTimeout < now`, or `scanPts`
is `none` or does not fit the stored scan.

**Processed (iff).** The output of an event is not `silent` iff the event is
a command, `st.phase = active` and `v.finite`; it is then `answer vel rep`
with the values of **Velocity** and **Report**. Every other event, and every
event in another phase, gives `silent`.

**Inactive.** If `st.phase ≠ active`, the output of every event is `silent`,
and a command leaves the state unchanged. On runs from `State.init`: no
output before the first `activate` is other than `silent`; no output after a
`shutdown` is; between a `deactivate` and the next `activate` none is.

**Velocity (iff).** For a processed command, `vel = none` iff `(evaluate
…).yieldsZero v` and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout <
now`; otherwise `vel = some (evaluate …)`. This is "Output" of SPEC.md.

**Report (iff).** For a processed command, `rep = some r` iff `r` is the
report of step 6 and `r ≠ st.prevReport`; and the new `prevReport` is the
report of step 6 in both cases. In particular a report is made also when
`vel = none`.

**Only if.** `evaluate` is `stopSource` iff the source is invalid; `stopZone
k` iff the source is not invalid and `k` is the first triggered stop zone;
`slowdown k r` iff the source is not invalid, no stop zone is triggered, and
`k` is the first triggered slowdown zone with the smallest ratio pattern
`r`, below `F64.one`; `pass` iff the source is not invalid and no zone of
either kind is triggered. "Triggered" as in E8 step 5.

**No transform.** If `cfg.sourceEnabled`, `0 < cfg.sourceTimeout`,
`st.phase = active`, `v.finite` and `scanPts` is `none` (or does not fit the
stored scan), then `evaluate = stopSource` and the output is `answer (some
stopSource) _` or `answer none _`, the latter exactly in the case of
**Velocity**. It is **false without the two hypotheses on the
configuration**, as "Stale data" is: with `source_timeout: 0` a missing
transform means "no obstacle" (N27; example in `Examples.lean`).

**Stale data, Stop, Bad data, No amplification, Slowdown bound,
Pass-through, Monotonicity** of SPEC.md hold with these changes: the
hypothesis `st.phase = active` is added; "the output is `publish a`"
becomes "`answer (some a) _`" and "`silent`" in their conclusions becomes
"`answer none _`"; the points are `s.points c` for the `scanPts = some c`
of the command. Stale data keeps its conclusion whatever `scanPts` is.
Pass-through needs coordinates that fit among its hypotheses when a scan is
stored (it is part of "`sourcePoints = some pts`"). (T05b) Bad data (a) and
Monotonicity are stated on the stored scan **and** the command's
coordinates together: removing the readings that are not valid, with their
pairs, changes no output (`output_filter_validBeam`); adding readings, each
with its pair, never lifts a stop (`evaluate_isStop_mono`).

**Scan points (T05b; replaces "Transform").** What is still a function of
the stored scan, and what is not:

- *Of the stored scan alone:* whether it is too old (R7); which readings
  are valid (R4); **how many points there are**: if `s.fits c`, `(s.points
  c).length` is the number of valid readings of `s`, whatever `c` is
  (`points_length`), and never more than that for any `c`
  (`points_length_le`); so no points iff no reading is valid
  (`points_eq_nil_iff`). The shim can neither add a point nor drop one.
- *Of the command's coordinates:* **where** each counted reading lies. Point
  `j` is `scanPointMm` of the pair at the position of the `j`-th valid
  reading (`points_eq`, `mem_points_iff`): for finite coordinates, the value
  the shim handed over, in millimetres, truncated toward zero and saturated
  (`coordMm_eq_truncReal`), less than 1 mm from it unless saturated
  (`coordMm_sub_lt`).
- *Trusted (T2, T6):* that pair `i` is the image of reading `i` of the scan
  the model has stored, under the transform N23 or N24 prescribes for this
  command. The model checks the number of pairs and nothing else about
  them.

T04's property **Transform** ("`Xform.pointMm` is the exact image,
truncated"; `Xform.vertexMm` within 1/2 mm of the exact image; the identity
gives `coordMm`) was proved by T05 and cannot be stated any more: the model
has no transform. What replaces each theorem is listed in THEOREMS.md, "What
T05 proved and option A does not have". For topic-zone vertices:
`vertexPointMm` is within 1/2 mm of the vertex the shim handed over, per
axis, for every accepted vertex (`vertexPointMm_sub_le`).

(T09) For the mark of an unusable message, see SPEC.md, "Properties",
"Unusable message": `acceptShape_unusable_iff`, `triggered_withShape_iff`,
`triggered_unusable_of_triggered`, `evaluate_isStop_of_unusable`,
`run_stop_after_unusable`.

**Shape accepted (iff).** A `shape k verts foreign` event changes the state
iff it is accepted (E7); then `shapes[k]` is as E7 says and nothing else
changes: not the other entries of `shapes`, not `scan`, `prevZero`,
`stopStamp`, `prevReport`, `phase`.
(T05) **False as an "iff".** Counterexample: the same accepted message
delivered twice; the second is accepted and the state does not change
(`step_shape_same`). What holds and is proved: the conditions of E7 are
equivalent to "the message installs a shape" (`acceptShape_eq_some_iff`); a
message that is not accepted changes nothing; an accepted one sets
`shapes[k]` as E7 says and changes nothing else; hence the state changes
**only if** the message is accepted.
(T05b) At a command, a refresh moves the vertices of a foreign shape and
does nothing else: the frame flag and the number of vertices stay
(`refresh_foreign`); vertices that are refused leave the shape as it was
(`refresh_of_refused`).

**No shape.** If zone `k` is a topic zone and `st.shapes[k] = none`, zone
`k` is not triggered by any points, and no output for a command names it.
On runs from `State.init`: no output names a topic zone `k` before an
accepted `shape k` event that follows the last `cleanup`.
(T05) "The last cleanup" must be one that took effect: a `cleanup` outside
the inactive phase is a no-op by E4 and discards nothing. Proved as: after
any history that leaves the node unconfigured, and any events that follow
among which there is no `shape k` event, no output names zone `k`
(`run_output_of_no_shape`). This is upstream's
behaviour (N32) and it is **not** fail-safe: a stop zone without shape does
not stop. See question 1 in section 13.

**Replacing a shape affects that zone only.** For two states that differ
only in `shapes[k]`, and any points: every zone other than `k` is triggered
in the one iff in the other. Hence the actions for a command differ only if
zone `k` is triggered in one and not in the other.

**Lifecycle invariants.** On every state reachable from `State.init`: if
`phase ≠ active` then `prevZero = false` and `prevReport = normal`; if
`phase = unconfigured` then `scan = none` and every entry of `shapes` is
`none`; `shapes` has one entry per zone and `some` only at topic zones;
every `current` has at least three vertices, all within the coordinate
limit.
(T05) **The first clause is false for the finalized phase**, with E5 as
written. Counterexample: `configure`, `activate`, a finite command while no
scan is stored and `source_timeout` is not 0 (a stop: `prevZero = true`,
`prevReport = source`), `shutdown`. E5 changes the phase only, so the
finalized state has `prevZero = true`. The model was not changed. Proved:
the clause for `phase = unconfigured` or `phase = inactive` (`Inv.quiet`),
which is what "After activation" needs; in the finalized phase the two
fields are never read again (`step_of_finalized`). The other clauses are
proved as stated.

**After activation.** If `step` on `activate` moves the state to `active`,
then for the next finite command: if the velocity is zero it **is
published** (`vel = some _`), and the report is `some r` unless `r =
normal`. (N6: a stop is never withheld right after an activation.)

**Totality.** As before; the conversions (`scanPointMm`, `vertexPointMm`,
and with T06 the decoding of coordinates and vertices from byte arrays of
any length) are total.

Trace theorems (`Trace.lean`): each of the above lifted to `run` from
`State.init` over arbitrary event lists, including lists with lifecycle
events in orders the lifecycle does not allow (they are no-ops by E1–E5).

## 5. Where the transform is applied

The transform's numbers come from tf2 in both options; neither option
checks tf2. The question is who computes N25.

**Option A.** The shim computes N25 in `double` and hands Lean points in
metres as now. For every command it must hand over the whole point list
again, because the transform depends on the command's time (N22, N24).

**Option B.** The shim passes the looked-up transform as six bit patterns
with the command; Lean computes the image exactly.

**Decision (the user, 2026-11-24; tasks/PLAN.md, "Decisions"): option A.**
T04 recommended B, and T05 built and proved B. The reasons for A: B costs
1.2 to 1.9 µs per reading and command (below); through the node, task T02
measured about 20 times upstream's time per command, and commands left
unanswered at 10,000 readings. With A the node does what upstream does, in
the arithmetic upstream uses. **Also decided:** the shim is compiled with
`-ffp-contract=off` on every platform, as the node is today (the test of
task T01 that enforces the flag stands): no fused multiply-add. T05b
reworked the model and the proofs; T06 writes the shim.

| | A (decided) | B (built by T05, removed by T05b) |
| --- | --- | --- |
| Trusted, beyond tf2 and R3 | The matrix and the two sums of N25 in C++, about 15 lines, **and how they are compiled**; that the points handed over are those of the stored scan | Nothing more |
| Agreement with upstream before the millimetre | Bit for bit where upstream's build does not fuse either (baseline x86-64 cannot; inference from the instruction set, the Linux binaries were not examined). Upstream's arm64 binary fuses the two sums (N25), so there the last bits of a coordinate can differ | Not bit for bit. Lean's point is the exact image; upstream's is that image computed in `double` |
| Model | The command event carries the coordinates; the stored scan decides which readings count and how many points there are, the command where they lie | The command event carries six numbers or `none`; points are a function of the stored scan and the transform, inside the theorems |
| Cost per reading and command | The shim's arithmetic, a few ns (estimated), plus decoding and conversion in Lean, now per command instead of per scan: 41 ns per reading was the figure before the extension, 0.19 µs is what the model costs now (below). Not measured through a node (T06) | Measured below: 1.2 to 1.9 µs exact; 0.45 µs in a fixed-point form |

**What the model receives (T05b).** With every command: one pair of
binary64 patterns per reading of the stored scan, the reading's point in
the base frame in metres, in scan order; or "none". Bit patterns, converted
in Lean by the existing `coordMm`, and not millimetres computed in C++,
because the conversion (truncation toward zero, saturation, what a NaN
becomes) is part of what the theorems are about and is proved; the shim
still "converts nothing". One pair per reading, and not only the valid
readings' points, because then the validity of a reading (R4) and the
number of points stay decisions of the model, taken from the stored scan:
the shim cannot drop or add a point (section 4, "Scan points"). The price
is 16 bytes per reading and command also for readings that do not count.
For a topic zone in another frame: the base-frame vertices of the stored
message, as binary64 pairs, or "none", per zone.

**Cost of A in the model, measured (T05b).** With the present interface
(a scan call remembers the coordinates, every command carries them),
`ros/roslean_collision_monitor/test/cmd_cost` (task T02's program: 1080
readings, nothing near, median of 400 commands back to back, macOS arm64,
machine in use by other tests):

| | Two circles | Two 4-vertex polygons |
| --- | --- | --- |
| Before the extension (`91c21f9`, T02's COMPARISON.md) | 18 µs | 0.41 ms |
| After T05 (option B; T02) | 2.5 µs per reading, about 2.7 ms | 2.9 µs per reading, about 3.1 ms |
| After T05b (option A) | **0.21 ms**, 0.19 µs per reading | **0.63 ms** |
| After T03 (option A, the fast command path) | **0.026 ms**, 24 ns per reading | **0.044 ms** |

(T03, added: the last row is task T03's, measured with the same program,
which now takes the lowest of three medians: COMPARISON.md, "Cost of a
command". The exported command call converts each coordinate on a machine
word, produces the points once, and prepares each polygon once per command;
it is proved to be `step` (`stepCmdFast_eq`). The rest of this paragraph
describes the state after T05b, which T03 was given to repair.)

So a command is 13 and 5 times cheaper than under B, and for circle zones
still 12 times dearer than before the extension. The difference to "before"
is the conversion to millimetres, which was made once per scan and under
option A is made for every command, because the points arrive with the
command. Of the 0.21 ms, about 0.14 ms is `coordMm` on 2160 coordinates
(measured by replacing the conversion with a constant, then restored),
0.05 ms the lists (`zip`, `filter`, `map`), 0.02 ms the zone tests. The
conversion is slow here because a coordinate is kept as a `Nat` pattern and
a pattern with the sign bit set is a big number for Lean's runtime; before
the extension the decoder took sign and magnitude apart
(`coordMmOfParts`, 41 ns per reading). Tasks T03 and T06 closed this: the
command call reads each coordinate into a machine word (`coordMmAt`,
`pointAt`), proved to compute the same points (`stepCmdFast_eq`,
`cmOnCmdFast_eq`); the cost of a command through the new call is in the
table "Cost of a command through the calls of section 8" below. `test_cm_ffi` is back to 47 s (169 s after T05, 39 to 42 s
before).

**Cost of a command through the calls of section 8 (T06, measured).** One
machine (Apple M5, macOS arm64), nothing else running. `bench_cm` runs the
callback bodies of both nodes in one process on the same messages; the Lean
side does what the node does for a command (clock, plan, the points of
every stored reading under the transform, the core, the product, publish)
with the identity as the transform and no TF buffer; upstream is its own
class. Median of 20,000 commands (4,000 at 10,000 readings), nothing in a
zone, microseconds:

| Stored readings | Two circles: Lean / upstream | Two 4-vertex polygons: Lean / upstream | Budget (core alone) |
| --- | --- | --- | --- |
| 360 | 10.3 / 1.7 | 16.5 / 2.8 | 50 |
| 1000 | 27.8 / 3.5 | 44.8 / 6.4 | |
| 1080 | 30.0 / 4.1 | 48.9 / 7.2 | 150 |
| 10,000 | 281 / 31 | 438 / 60 | 1500 |

So a command costs 6 to 9 times upstream's and stays under the budget T03
set for the core alone, the shim's work included. The scan callback takes
10 ns per reading (the ranges are copied and decoded), where upstream
stores a pointer. The core alone (`cmd_cost`, the points handed over as
bytes): 13.7 and 22.9 us at 360 readings, 26.7 and 44.2 at 1080, 251 and 409
at 10,000. Through the two node binaries over topics
(`run_cm_bench.py node`, 50 commands and 25 scans a second for 20 s per
size): both nodes answered every command (851, 851 and 850) with no answer
differing; the median latency, which is mostly the measuring program's and
the middleware's, was 5.8, 6.7 and 7.5 ms for the Lean node and 6.3, 6.6 and
6.4 ms for upstream at 360, 1080 and 10,000 readings. Not measured: a
transform from the TF buffer in the loop, base shift correction, topic
zones, Linux.

**Cost of B, measured (history).**
`review/collision_monitor/nav2/TransformCost.lean` is a Mathlib-free
prototype of the exact transform on unbounded integers, compiled with
`leanc -O3`; `run_transform_cost.sh` runs it; output in
`results/macos-arm64-transform.txt`. On an Apple M-series core, while
another job used the machine, over eight runs: **1.2 to 1.5 µs per point**
for a quaternion with four non-zero components, **1.6 to 1.9 µs** for a
pure yaw. A fixed-point variant in the same file costs **0.43 to 0.50 µs
per point**. T05's implementation in the model measured the same: the leak
test of `test_cm_ffi` took 169 s, about 1.3 µs per reading and command.

**How far the model's points are from upstream's (option A).** For one
reading let `e` be the exact image of its scan-frame point under the
transform tf2 returned, `u` the point upstream computes, `s` the point the
shim computes, `m` the model's point. Per axis:

- `|m − s| < 1 mm`, toward the origin, unless saturated: **proved**
  (`coordMm_sub_lt`). For a topic-zone vertex `|m − s| ≤ 1/2 mm`: **proved**
  (`vertexPointMm_sub_le`).
- `|u − s| ≤ δ`: **not proved; estimated and trusted (T7).** Both are
  evaluations in `double` of the same expression (N25) on the same seven
  numbers from tf2: the matrix entries, about six operations each on values
  of magnitude at most 2, then three products and three sums on values
  below `|p| + |o|`. Each IEEE-754 operation is within half a unit in the
  last place of its exact result, 1.1·10⁻¹⁶ relative; with coordinates and
  translations below 1000 m that gives each evaluation a distance of at
  most about 5·10⁻⁹ mm from `e`, fused or not, hence `δ` of the order of
  **10⁻⁸ mm**. This document allows **10⁻⁶ mm**. Where neither build fuses,
  and the shim uses tf2's own inline code for the matrix and the product
  (8.4), `s = u` bit for bit (inference; not checked on a Linux binary).
  Errors inside tf2 (interpolation, the product of N24 with its large odom
  coordinates) do not enter `δ`: both programs receive the same numbers.

**Does the polygon margin of √5 mm still guarantee "a point upstream counts
is always counted"?** Yes, with room to spare, under the same split into a
proved and a prose half as SPEC.md ("Safe direction"). The proved half is
per axis: if every vertex of the model is within `dv` of upstream's and the
model's point within `dp` of upstream's point `u` on the boundary, with
`2·(dv + dp)² ≤ 5`, that is `dv + dp ≤ 1.5811 mm`, the point is counted
(`contains_of_close`). The budget:

| Zone | `dv` | `dp` | Room for `δ` | Theorem |
| --- | --- | --- | --- | --- |
| Fixed polygon | 1/2 (rounding of the parameter) | 1 + δ | **δ ≤ 0.08 mm** | `contains_of_shim_fixed` |
| Topic zone | 1/2 + δ (rounding of the shim's vertex, which is within δ of upstream's) | 1 + δ | **δ ≤ 0.04 mm** | `contains_of_shim` |
| Circle | radius rounded up, plus 1 mm | truncation toward the origin, δ | **δ ≤ 0.5 mm** | `circle_safe_shim` |

So the guarantee holds if the shim's coordinates are within 0.04 mm of
upstream's; the allowance of 10⁻⁶ mm is 40,000 times smaller. **Proved:**
the three theorems, with `δ` as a hypothesis. **Prose:** that the boundary
passes over the point (THEOREMS.md, "Geometry"). **Trusted:** the bound on
`δ` (T7), the shim's arithmetic and flags (T2). The same budget is what any
later change of the shim's arithmetic has to respect: a shim whose point is
within 0.04 mm of upstream's keeps the guarantee, whatever its flags.

For the differential tests nothing changes: they exclude points within
5 mm of a boundary (SPEC.md, "Note on testing"), far more than `δ`.

**Base shift correction.** The same in both options: it is one transform,
composed inside tf2 (N24), and reaches the shim as the same seven numbers.
Its precision is tf2's: two interpolations, one product of transforms and
one conversion from matrix to quaternion, in `double`, none of it in the
shim under either option. Its order of operations is N24 followed by N25.

**What option A gives up, for the record.** The points are no longer a
function of what the model stores; "every point is the exact image of its
reading, truncated" is no longer a theorem but "every point is the shim's
point, truncated"; about 15 lines of floating-point arithmetic and the
compiler's treatment of them join the trusted base; and the restriction to
correctly compiled flags is of the kind of CM3/CM4 finding 5. What it
gains: upstream's cost per command, upstream's arithmetic, and on builds
that do not fuse, upstream's points.

## 6. Differences from upstream

ND1–ND12, and ND13 added by T09: moved to [SPEC.md](SPEC.md), "Deliberate
differences in ROSlean". ND3, ND6, ND10 and ND11 were amended there after
the review CM5.

## 7. What is trusted after the change

Moved to [SPEC.md](SPEC.md), "Where the velocity is computed": T1–T10 (T2,
T6, T7 and T8 are this section's; T10 is new).

## 8. Shim boundary

In the form of SPEC.md, "Shim boundary", whose general rules stand: scalars
and Lean objects only; the shim converts nothing and refuses no value; every
object argument is consumed.

### 8.1 Start-up and lifecycle

| Call | Arguments | Meaning |
| --- | --- | --- |
| `roslean_cm_builder_add_topic_polygon` | builder, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a topic zone (no shape) |
| `roslean_cm_on_lifecycle` | monitor, `transition : uint8` → monitor | 0 `configure`, 1 `activate`, 2 `deactivate`, 3 `cleanup`, 4 `shutdown`: events E1–E5. Any other value: no change |
| `roslean_cm_phase` | monitor → `uint8` | 0 unconfigured, 1 inactive, 2 active, 3 finalized. For logs and tests |

The monitor is built in `on_configure` from the parameters, as upstream
reads them there (N3), and `configure` is delivered to it. If
`roslean_cm_ok` is 0 the transition fails as upstream's does: the node stays
unconfigured and the process stays up (this replaces D14). "Fail closed"
stays as the backstop.

The shim delivers a lifecycle event from the corresponding `on_…` callback,
before it returns success. It does not keep an "active" flag of its own to
decide whether to publish: that decision is the output of `on_cmd`.
Lifecycle publishers, which drop messages while the node is inactive, may be
used: they can only suppress.

### 8.2 Scan and shape events

(T05b: rewritten for option A. T06 implemented the encodings as proposed
here.)

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_scan` | monitor, `now_ns : uint64`, `stamp_ns : int64`, `range_min_bits : uint32`, `range_max_bits : uint32`, `ranges : ByteArray` → monitor |
| `roslean_cm_on_shape` | monitor, `zone : uint32`, `foreign : uint8`, `has_verts : uint8`, `verts : ByteArray` → monitor |
| `roslean_cm_shape_accepted` | monitor → `uint8`: the last `on_shape` call was accepted (E7) |

- `on_scan`: without the argument `usable`. `ranges` has 4 bytes per
  reading, the range pattern, little-endian; a remainder is ignored. No
  coordinates: they come with each command. The shim keeps the message
  (ranges, `angle_min`, `angle_increment`, `frame_id`, stamp) for the
  lookups and the points of the next commands (T2, T6).
- `on_shape`: `verts` has 16 bytes per vertex, the bit patterns of `x` and
  `y` of the vertex **in the base frame** as little-endian `double`s (N33,
  step 3: the `float` coordinates of the message under the transform, `z`
  taken as 0; for a message in the base frame the identity, which leaves
  the widened `float`s); a remainder is ignored. `has_verts` is 0 if the
  lookup failed: `verts = none`. `foreign` is 0 iff the message's
  `frame_id` equals `base_frame_id` (a string comparison, as upstream's).
  The lookup is N33 step 2, made by the shim before the call, and only if
  the message has at least three points (so that a short message is ignored
  without a wait, as upstream; the shim then still makes the call, with the
  short list, and Lean refuses it).
- If `shape_accepted` is 1 the shim stores the message for the lookups of
  later commands and for N16; otherwise it logs an error and keeps the
  previous one. (T09) If `shape_accepted` is 1 and the new query
  `roslean_cm_shape_unusable(monitor, zone)` is 1 too, the message had an
  unusable vertex (ND6): the shim drops what it had stored for the zone and
  shows an empty polygon.

### 8.3 Command event

| Call | Arguments |
| --- | --- |
| `roslean_cm_cmd_plan` | monitor, `now_ns : uint64`, six `uint64` (the twist) → `uint8` |
| `roslean_cm_on_cmd` | monitor, `now_ns : uint64`, six `uint64`, `has_points : uint8`, `points : ByteArray`, `zone_verts : ByteArray` → monitor |
| `roslean_cm_out_kind` | monitor → `uint8` |
| `roslean_cm_out_zone`, `roslean_cm_out_ratio` | as today |
| `roslean_cm_out_report` | monitor → `uint8` |
| `roslean_cm_out_report_zone` | monitor → `uint32` |

1. The shim reads the clock once: `now_ns`.
2. `cmd_plan` says which lookups upstream would make for this command, so
   that the shim neither waits where upstream answers at once nor skips a
   lookup: 0 none (not active, command not finite, or source invalid before
   any lookup: no scan or scan too old with a non-zero timeout); 1 the
   topic zones' only (source disabled, or no scan with timeout 0); 3 the
   scan's and the zones'. It changes nothing.
3. Lookups (T6). For the scan: N23 or N24, with `now_ns` as the target
   time, with the timeout `transform_tolerance`, on a buffer whose listener
   has its own thread, so that the wait of N26 is upstream's. For every
   enabled topic zone whose stored message is in another frame: N34.
4. **Points (T2, new with option A).** If the scan's lookup succeeded: for
   **every** reading `i` of the stored message, valid or not, the point of
   R3 in `float`, widened, and its image by N25 in `double`; `points` gets
   16 bytes per reading, the patterns of `x` and `y` in the base frame,
   little-endian, in scan order, and `has_points` is 1. Otherwise
   `has_points` is 0. The shim does not test the range of a reading: which
   readings count is Lean's decision, and what it writes for a reading that
   does not count is not looked at (`output_filter_validBeam`). For every
   zone looked up in step 3: the vertices of the stored message by N25.
5. `on_cmd`. `scanPts` is `none` if `has_points` is 0, else the decoded
   pairs (a remainder of fewer than 16 bytes is ignored; a list that is not
   one pair per stored reading counts as no points, E8 step 3). `zone_verts`:
   for each zone in order a 4-byte little-endian count `n` and `16·n` bytes
   of vertices, with `n = 0xFFFFFFFF` for "none"; a zone the array does not
   reach has none.
6. `out_kind`: 0 `silent`; 1 to 4 as today (`answer (some a) _`); 5
   `answer none _` (processed, velocity withheld). `out_report`: 0 no
   report; 1 `normal`; 2 `stop`; 3 `slowdown`; 4 `source`; the zone index
   in `out_report_zone` for 2 and 3.
7. The shim then: publishes the state message if `out_report` is not 0 and
   `state_topic` is set (1 → `("", 0)`; 2 → `(<name>, 1)`; 3 → `(<name>,
   2)`; 4 → `("invalid source", 1)`), **before** the velocity (N13);
   publishes the velocity for kinds 1 to 4 as today; publishes the
   visualised polygons for kinds 1 to 5 (N16); for kind 0 nothing.

(T06) As implemented, three details beyond the list above:

- **The wait of N26 is the node's own loop** (`wait_for`), with upstream's
  rule (sleep 10 ms of wall time at a time while the node clock is before
  the start of the lookup plus `transform_tolerance`, try again each time,
  and once more at the end; (T09, review CM5, finding 2) **and end the wait
  when the clock is more than 3 s before its start**, which T06's loop
  lacked: the node then waited until the clock was back, handling nothing;
  the clock of the wait is now the newest `/clock` message taken by hand,
  and the node clears the TF buffer itself on a message that shows a
  backward jump, SPEC.md, "Command event"), around `try_lookup` of `transforms.hpp`, which
  never waits. `tf2_ros::Buffer::lookupTransform` with a timeout is not
  called: it reads the node clock, and under simulated time that clock
  cannot advance inside a callback of this node, which runs without
  rclcpp's clock thread (SPEC.md, D15). While it waits the node takes the
  `/clock` messages of its own subscription by hand. If one of them carries
  a negative time the wait ends and the command is delivered as "without a
  clock reading" (`roslean_cm_on_cmd_no_clock`): a stop.
- **After a failed lookup for the scan with a non-zero source timeout the
  zones' lookups are not made**: upstream does not reach its zone loop then
  (N34), and the core leaves the shapes unchanged in that case (E8, step 4),
  so nothing is lost and no wait is added. Whether the timeout is zero the
  shim asks the core's conversion (`roslean_cm_source_timeout_ns`).
- **The array of zone vertices ends after the last zone that has
  vertices**; a zone it does not reach has none.

Theorems for these calls (`ROSlean/CollisionMonitor/FFI.lean`, `Fast.lean`;
THEOREMS.md, "The C interface and the conversions"): a sequence of calls
yields `run` of the corresponding events from the initial state
(`foldl_exec_run`); the five numbers read back determine the model's
output, including the report (`cm_readBack_after`, and
`cm_readBack_after_fast` on the calls C makes); if `cmd_plan` is 0 the
result of `on_cmd` depends on neither array, if it is 1 not on `points`
(`cmOnCmd_of_plan_zero`, `cmOnCmd_of_plan_one`); `cmd_plan` returns a number
and no monitor; decoding of `ranges`, `points`, `verts` and `zone_verts` is
total and inverts the shim's encoding (`decodeBeams_encodeRanges`,
`decodePairs_encodePairs`, `decodeZoneVerts_encodeZoneVerts`); the exported
command call is the definition the theorems are stated on
(`cmOnCmdFast_eq`, `execFast_eq`).

### 8.4 What stays in C++

- `transforms.hpp` (new, the only file that calls tf2 and the only one
  that computes with a transform; T05b: it now holds arithmetic). (a) The
  two lookups, the comparison of frame names, the check of ND7 (seven
  finite numbers, a quaternion that is not zero; otherwise "none"). (b)
  **N25**: the looked-up message into a `tf2::Transform` with tf2's own
  `fromMsg`, and the image of each point with tf2's own `operator*`, so
  that the operations and their order are upstream's by construction;
  `x` and `y` of the result, the third coordinate dropped. For a base-frame
  scan without correction the identity is applied all the same, as upstream
  does (N23). (c) Compiled with **`-ffp-contract=off`**, like the rest of
  the node: decided by the user. On a platform where upstream's build fuses
  (arm64) the last bits of a coordinate can therefore differ from
  upstream's (section 5, T7); the flag is not to be changed per platform.
  Tests: against a `tf2::BufferCore` filled by the test, that the lookups
  ask for N23 and N24 with the arguments in the right order (source and
  target frame, the two times) and that every kind of failure gives "none";
  for the arithmetic, that the points equal those of a reference
  computation written out operation by operation (N25) on exact and on
  general quaternions, bit for bit, and that the object code contains no
  fused multiply-add (the existing check of task T01, extended to this
  file).
- `conversions.hpp`: `encode_scan` without the sensor pose and without
  coordinates (ranges only); the scan-frame point of R3 (`cos`, `sin`, the
  `float` product) moves to where the points of a command are computed
  (T2).
- The polygons for `visualize` (N16): the fixed polygon's vertices as read,
  the 16 points of a circle, the last accepted message. T8.

### 8.5 Node plumbing

- The node derives from `nav2_util::LifecycleNode`, which gives the bond,
  `bond_heartbeat_period`, `autostart_node` and the pre-shutdown clean-up
  exactly as upstream has them (N9, N11). New build dependencies of the
  package: `nav2_util`, `nav2_msgs`, `rclcpp_lifecycle`, `tf2_ros`. If `nav2_util` proves too heavy a dependency, the
  same 30 lines over `rclcpp_lifecycle::LifecycleNode` and `bondcpp`.
- Subscriptions and publishers are created in `on_configure` and destroyed
  in `on_cleanup` with upstream's types and QoS (N3, N12, N16, N31). The TF
  listener has its own thread (N1).
- Parameters newly read: `state_topic`, `odom_frame_id`,
  `transform_tolerance` (converted as `tf2::durationFromSec` does, by
  calling it), `base_shift_correction` (default `true`, as upstream),
  `<zone>.polygon_sub_topic`, `<zone>.polygon_subscribe_transient_local`,
  `<zone>.visualize`, `<zone>.polygon_pub_topic`, `bond_heartbeat_period`.
  Read and ignored without refusal, because upstream does not read them
  either: `<zone>.footprint_topic`, `time_before_collision`,
  `simulation_time_step` on a zone that is not `approach` (with the message
  of ND11 where it applies), `<source>.min_height`, `max_height` on a scan
  source. `<source>.roslean_sensor_pose` is refused with a message that
  points to TF.
- **Composition.** T06 tries whether the node can be registered as an
  rclcpp component and loaded into `nav2_container` beside Nav2's own
  nodes (the Lean runtime is initialised once per process). (T06) It can:
  the plugin is `roslean_collision_monitor::CollisionMonitorNode` in the
  library `collision_monitor_component`; see ND10 for what was observed and
  for what a container does not give (D15). In a container with one
  executor for all nodes (`component_container`) a command that waits for a
  transform holds up every node of the container, as upstream's does.
- (T06) **Zone names** (ND5) are checked by the shim when it reads
  `polygons`: the core knows zones by index. **Parameters** are read in
  `on_configure`; every later change is rejected (ND8), except the
  declarations that `on_configure` itself causes (the QoS overrides of the
  TF listener's subscriptions). On cleanup the core's monitor is dropped
  after the `cleanup` event, and the next `configure` builds a new one from
  the parameters: its state is the model's state after `cleanup` (no scan,
  no shapes, nothing published yet).

### 8.6 What a user changes to run it in Nav2

(T06) True of the node as it is: the package's
`collision_monitor_params.yaml` starts it unchanged, and the collision
monitor section of `nav2_params.yaml` with either edit below (scenarios
`footprint-*` of `scenario_nav2.py`). T10 demonstrates it in Nav2. The
package's own launch file, `launch/collision_monitor_launch.py`, starts the
node with a lifecycle manager.

1. **Parameters.** In the `collision_monitor` section, the zone must not be
   `approach`. Either the footprint as a fixed polygon, which is the
   recommended form (N34):

   ```yaml
   polygons: ["FootprintStop"]
   FootprintStop:
     type: "polygon"
     points: "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"
     action_type: "stop"
     min_points: 6
   ```

   or Nav2's zone with two lines changed:

   ```yaml
   FootprintApproach:
     type: "polygon"
     action_type: "stop"                                        # was "approach"
     polygon_sub_topic: "/local_costmap/published_footprint"    # new; footprint_topic is read for approach only
     footprint_topic: "/local_costmap/published_footprint"      # may stay, is not read
     time_before_collision: 1.2                                  # may stay, is not read
     simulation_time_step: 0.1                                   # may stay, is not read
     min_points: 6
     visualize: False
     enabled: True
   ```

   Upstream accepts the second form too [B: `footprint-sub-topic`].
   Everything else in the section stays, including `state_topic` and
   `base_shift_correction: True`. The package's own
   `collision_monitor_params.yaml` needs no change: its lists name a fixed
   stop polygon and a scan only (N21).
2. **Launch.** In a copy of `navigation_launch.py`: in the branch without
   composition, `package='roslean_collision_monitor'` in place of
   `package='nav2_collision_monitor'` (the executable is `collision_monitor`
   in both; `name`, parameters and remappings stay). With composition
   either start Nav2 with `use_composition:=False`, or remove the
   `ComposableNode` of the monitor from the container and add the `Node`
   action above next to it; or change the `ComposableNode` to
   `package='roslean_collision_monitor'`,
   `plugin='roslean_collision_monitor::CollisionMonitorNode'` (the name
   that is built; T04 had proposed `…::CollisionMonitor`). Loaded as a
   component the node lacks the handling of a negative `/clock` (SPEC.md,
   D15, ND10): that is promised for the executable only.
3. Nothing else: the node name, the lifecycle manager's list, the topics
   and the bond are as upstream's.

## 9. Edits to make to SPEC.md later

Not made by task T04. (T06) Made now, in SPEC.md; its head says how.

1. **Scope paragraph and title of "Rules as implemented upstream"**: add
   the lifecycle, transforms and topic zones; refer here for N….
2. **R3**: the point is in the scan frame; the transform is N25.
3. **R8**: "its transform to the base frame cannot be obtained" now refers
   to N23, N24, N27.
4. **R17**: add the parameters of 8.5.
5. **R18, R19**: the node is a lifecycle node; drop "no lifecycle" wording;
   `~/toggle` and run-time `enabled` move to ND8.
6. **"Units and representation"**: (T05b) scan points are `binary64`
   patterns in the base frame, handed over with each command, and converted
   by `coordMm` as before; add topic-zone vertices (`vertexCoordMm`).
7. **"Safe direction"**: the scan-point paragraph and the polygon paragraph
   get the argument of section 5.
8. **T1–T5**: replaced by section 7.
9. **The model, Properties**: sections 3 and 4.
10. **Shim boundary**: section 8. `roslean_cm_on_scan` loses `usable`; the
    sensor pose disappears from the pseudo-code and from "What the shim
    still refuses". `Monitor.dropScan`, and `Event.scanUnusable` if T01 has
    introduced it, are removed: a scan is no longer usable or unusable when
    it arrives; whether it can be placed is decided per command (the command
    carries its points or none).
11. **Excluded features**: remove the rows "Motion correction",
    "Transforms", "State topic", "Zone visualisation" and "Lifecycle";
    "Dynamic polygon or radius" becomes "Dynamic radius" (ND12); "Footprint
    subscription" stays (`approach` only); add "Component" (ND10) and
    "Toggle and run-time parameters" (ND8); "Collision point markers" stays
    (ND9).
12. **D2**: replaced. The node is a lifecycle node with a bond; what
    remains is ND8 and ND4.
13. **D7**: add ND2.
14. **D11**: replaced. TF is used as upstream uses it, including base shift
    correction. `roslean_sensor_pose` is removed (refused with a message).
    The restriction to upright sensors goes; what upstream does with a
    tilted sensor (N25) is reproduced.
15. **D13**: the state topic and the polygons are provided; only the point
    markers are not (ND9).
16. **D14**: replaced by "as upstream: the transition fails".
17. **The frame check** (`header.frame_id == base_frame_id`): survives only
    as the string comparison that selects the identity (N23) and `foreign`
    (N33).
18. **A4**: becomes "the TF tree is right and timely (T6), and the sensor's
    clock agrees with the node's".
19. **Open decisions** 7 and 10: closed by this extension. Decision 7's
    last sentence (`Event.scanUnusable`) is overtaken by edit 10.
20. **"Note on testing against upstream"**: the restrictions
    `base_shift_correction: false`, scans in the base frame and an empty
    `state_topic` go.

## 10. Test plan

For T07 (upstream, in the harness) and T08 (the Lean node through the same
scenarios). "Upstream does" is what the named probe case showed on
2026-11-24 (macOS arm64, 1.3.12). The harness needs, beyond what it has: a
`/tf` publisher with chosen stamps; static transforms with arbitrary
quaternions; a `PolygonStamped` publisher; lifecycle transitions in the
middle of a scenario; recording of the polygon topics; and for every command
whose lookup fails, a clock advance before the answer can come (N26; the
harness has `late` for that).

| # | Scenario | Upstream does | Probe case |
| --- | --- | --- | --- |
| 1 | Command and scan while unconfigured | No subscription exists; nothing | `lifecycle` |
| 2 | Scan, command, non-finite command while inactive | Nothing published; the scan is used after activation | `lifecycle` |
| 3 | Activation | No velocity, no state; each visualised polygon once | `lifecycle`, `visualize` |
| 4 | Stop held beyond `stop_pub_timeout`, then deactivate and activate, same clock | Zero and the state are published again; zero for a further `stop_pub_timeout`, to the nanosecond | `lifecycle` |
| 5 | Cleanup, configure, activate | Stored scan gone: `invalid source`; a topic zone's shape gone | `lifecycle`, `polygon-topic-inactive` |
| 6 | Shutdown from active, then a command | Forwarded (N8). The Lean node: nothing (ND4) | `lifecycle` |
| 7 | State messages: pass first; slowdown; stop; zero command in a slowdown zone; invalid source; changes while the velocity is withheld; non-finite command | N13, N14 | `state` |
| 8 | Polygon topics: after activation, after a processed command, after a withheld one, after a non-finite one; circle and polygon; contents | N16 | `visualize` |
| 9 | Bond on, wall clock: messages on `/bond`; under a lifecycle manager; node killed | N9, N10 | `bond`, `manager` |
| 10 | Scan frame offset and rotated, exact transforms | SPEC scenarios `offset`, `rotated` (UPSTREAM_BEHAVIOUR.md 23) | not re-run here |
| 11 | No correction: transform changes after the scan; newest transform old; older transform arrives late | Newest is used, any age | `tf-latest` |
| 12 | Sensor upside down, raised, pitched, quarter turn, quaternion not normalised | N25 | `tf-tilt`, `tf-pitch-circle` |
| 13 | Missing transform: clock still; tolerance − 1 ns; tolerance; transform arriving during the wait; empty `frame_id` | N26 | `tf-missing` |
| 14 | Missing transform with `source_timeout: 0` and with `transform_tolerance: 0` | Passes at once; stops once the transform exists and a point is inside | `tf-missing-timeout0` |
| 15 | Correction: base moving straight between scan and command; half way between odom samples; correction off | N24 | `shift-straight` |
| 16 | Correction: base turning by half a turn | N24 | `shift-turn` |
| 17 | Correction with a sensor frame; scan older than the odom history | N24 | `shift-sensor-frame` |
| 18 | Correction: command newer than the newest odom sample; a newer sample arriving during the wait | Waits, then `invalid source`; or answers when the sample arrives | `shift-straight` |
| 19 | Correction, scan in the base frame, no odom frame | `invalid source` after the tolerance | `shift-no-odom` |
| 20 | Nav2's zone with only `action_type` changed, `stop` and `slowdown`, both parameter files | `configure` fails with the messages of N29 | `footprint-edit` |
| 21 | The same with `polygon_sub_topic` added: no message; footprint in odom; base moved, no new message; new message | N32, N34 | `footprint-sub-topic` |
| 22 | Fixed points with `footprint_topic` and `polygon_sub_topic` set | Neither is subscribed | `footprint-topic-ignored` |
| 23 | Topic zone: no shape; box; two points; none; triangle; unknown frame; other frame; empty frame; NaN; closed polygon; degenerate | N32, N33. The Lean node differs at NaN (ND6) | `polygon-topic` |
| 24 | Topic zone in a moving frame, `slowdown` | N34 | `polygon-topic-moving` |
| 25 | Shape while inactive; shape lost at cleanup; latched message with `polygon_subscribe_transient_local` | N31, N35 | `polygon-topic-inactive` |

### 10.1 Where each row is tested (added by T07)

The rows above are scenarios of
`ros/roslean_collision_monitor/test/scenario_nav2.py` (37 scenarios in six
families), written from runs of upstream's binary, not from the column
"Upstream does"; what they show is in UPSTREAM_BEHAVIOUR.md, findings 24 to
38. No row was contradicted; N28 is made more precise below. The node under test is an argument;
`test/profile_roslean_nav2.py` is the one place that states where the
ROSlean node is to answer differently (ND4, ND5, ND6, ND11). Every point of
a scenario is at least 0.125 m from every zone boundary, except in
`shift-interpolation`, which pins upstream's interpolation with points on a
boundary and is therefore not run for the ROSlean node (ND1).

| Rows | Scenarios |
| --- | --- |
| 1, 2, 3 | `lifecycle-states` |
| 4 | `lifecycle-reactivate` |
| 5 | `lifecycle-cleanup`, `polygon-topic-inactive` |
| 6 | `lifecycle-shutdown-active`; also `lifecycle-shutdown-inactive`, `lifecycle-shutdown-deactivated` |
| 7 | `state-topic`; also `state-name-collision` (N15 on the binary) |
| 8 | `polygon-pub`, `lifecycle-states`, `polygon-topic` (not: the point markers of N17) |
| 9 | `test/bond_manager.py` (wall clock) |
| 10 | `offset`, `rotated` of `scenario.py`, as before |
| 11 | `tf-latest` |
| 12 | `tf-tilt`, `tf-pitch-circle`; also `tf-bad-quaternion` (transforms tf2 refuses; ND7 is not reachable through them) |
| 13 | `tf-missing`; also `tf-tolerance-decimal`, `tf-tolerance-0.3` |
| 14 | `tf-missing-timeout0`, `tf-tolerance0`; also `tf-wait-timeout0` |
| 15 | `shift-straight`, `shift-off`; also `shift-interpolation` |
| 16 | `shift-turn`; also `shift-quarter-turn` |
| 17 | `shift-sensor-frame` |
| 18 | `shift-straight` |
| 19 | `shift-no-odom` |
| 20 | `footprint-edit-stop`, `footprint-edit-slowdown`, `footprint-edit-example-file`, `footprint-approach-unchanged` |
| 21 | `footprint-sub-topic`; also `footprint-sub-topic-slowdown` |
| 22 | `footprint-topic-ignored` |
| 23 | `polygon-topic` |
| 24 | `polygon-topic-moving` |
| 25 | `polygon-topic-inactive` |

Observed by these scenarios and stated above from the source only, or not
at all (all on 1.3.12, macOS arm64, and, except the scenarios that need
`nav2_bringup`'s parameter file, on 1.3.14 in a Linux arm64 container):

- N8: shut down from `active`, the node also still stores scans and
  publishes a stop with its state message; shut down from `inactive` it
  publishes nothing.
- N15: a zone named `invalid source` hides the change to an invalid source
  (`state-name-collision`).
- N23: a transform with a newer stamp is used by the next command at once.
- N24: after a quarter turn of the base to the left a point that was ahead
  is on the right (points 0.125 m from the zone's edge; not a boundary
  test).
- N26: the rounding of `transform_tolerance` to the nearest nanosecond:
  with 1.001 no answer at 1,000,999,999 ns and the stop at 1,001,000,000 ns
  (`source_timeout: 1.001` is 1,000,999,999 ns: `timeout-decimal` of
  `scenario.py`).
- N24: the interpolation between two odom samples decides a zone boundary
  exactly where the numbers are binary fractions (`shift-interpolation`: a
  quarter, a half and three quarters of the way between samples 1 m apart).
- N25: a transform published with a zero quaternion, a NaN in the
  quaternion or a NaN translation is not taken by tf2; the frame stays
  unknown (`tf-bad-quaternion`).
- N27: with `source_timeout: 0` a command whose lookup fails still waits
  for `transform_tolerance` before it passes.
- N28: a backward jump of the clock clears the transforms received on
  `/tf`; those received on `/tf_static` survive it (`tf-clock-jump-back`).
  A command that needs a cleared transform waits and is stopped for
  `invalid source` until the transform is published again.
- N29: the `slowdown` form of the edited footprint zone with
  `polygon_sub_topic` configures and slows to the default ratio 0.5.

The random test of the extension is `test/random_nav2.py`: transforms with
exact and general quaternions, a moving and turning base with odom samples
at every command's clock reading unless the tick is meant to fail,
lifecycle transitions and shape messages interleaved with scans and
commands, judged against a Python model; points within 5 mm of a zone
boundary, lookups that need interpolation and the wait of N26 are kept
out. It compares velocities and state messages, not polygon topics.

Not observed for this document, to be pinned by T07 if wanted (T07 pinned
the interpolation at a boundary, the quarter turn, coarsely, and the
backward clock jump; the others remain open): the exact
interpolation of N24 at a point where it decides a boundary; a quarter-turn
base rotation with correction; transforms with inexact quaternions at a zone
boundary (they belong to the 5 mm the comparison excludes); a clock that
jumps backwards (N28); the real local costmap as the publisher of N30; the
component form of the node (N18); `autostart_node` (N11); anything on
Linux.

For the random differential test: transforms drawn from exact quaternions
and from general ones, with the points kept 5 mm from every zone boundary
as now; odom samples on both sides of every command unless the case is
meant to fail; lifecycle transitions and shape messages interleaved with
scans and commands; and the Lean node compared with upstream on velocity,
state messages and polygon topics.

**Added by T09 (after the review CM5).** Scenarios of `scenario_nav2.py`,
each run on upstream's binary first (macOS arm64, 1.3.12; and 1.3.14 in the
Linux arm64 container) and then on the node, which failed each before its
fix:

| Scenario | What it pins | Rule |
| --- | --- | --- |
| `polygon-topic-nan` | A 2 m square with a NaN in its 4th, then in its 1st vertex: which readings stop upstream. The node stops for all of them (ND6: two stated differences) | N33, ND6 |
| `tf-wait-clock-reset` | The clock jumps back by 99 s while a command waits for a transform: answered at once; the node goes on | N26 |
| `tf-wait-clock-back` | Jumps back by 1 s and 2.5 s during a wait: the wait goes on; a transform that arrives after the jump is used and kept; a jump outside a wait forgets it | N26, N28 |
| `shift-wait-reset` | The same with base shift correction: a command 50 ms ahead of the newest odometry sample, then a reset of the clock | N24, N26 |
| `points-short-topic`, `points-malformed-topic` | `points` with three points, or with a point that is not a pair, beside `polygon_sub_topic`: the zone is fed by the topic | N29 |
| `sub-topic-empty` | `polygon_sub_topic: ""`: configures, no subscription, never a shape | N29, N32 |
| `stop-zone-ratio-type` | A `slowdown_ratio` that is not a number on a stop zone: configures | N29 |
| `polygon-pub-disabled` | A disabled visualised zone is not published, on activation or after a command | N16 |

Beside the scenarios: `smoke_node` (an exception in `configure`; the new
accepted configurations), `smoke_transforms` (`wait_goes_on`; the angle
over 1080 readings), `test_cm_ffi` (the mark of an unusable message; the
wrapper from three threads), `smoke_component_single`, `smoke_component_mt`.

## 11. Implementation plan

**T05, model and proofs** (needs T01 merged and this document). Done, for
option B; **T05b** then moved the transform out of the model (option A):
items 1 and 3 below describe what T05 built, not what the model has now.
1. The data of 3.1 and the events of 3.2 in
   `core/ROSleanCore/CollisionMonitor.lean`; `Xform.pointMm` and
   `Xform.vertexMm` in a Mathlib-free file beside it, exact, with zero
   components handled without large shifts.
2. The properties of section 4 and their lifts to `run`.
3. The geometry lemmas for the margin argument with transformed vertices
   (the analogue of `nearBoundary_of_rounded`).
4. Examples: a missing transform with and without `source_timeout: 0`; a
   topic zone before and after its first shape; a shape replaced; a refused
   shape; deactivate and activate inside a stop; shutdown.
5. Hands to T06: the definitions of 3.1, and `Output` with `silent`
   distinct from `answer none none`.

**T06, C interface and node** (needs T03 and T05).
1. The calls of section 8 and their theorems.
2. The lifecycle node of 8.5; `transforms.hpp` of 8.4 with its tests.
3. The parameter handling of 8.5 and the refusal message of ND11.
4. `smoke_*` tests at node level for the rows of section 10 that need no
   upstream; the try of 8.5 "Composition".
5. A benchmark of a command with 360 and with 1000 readings through the
   node, against upstream (section 5 has only an estimate for option A).
6. The edits of section 9.

**T07, upstream scenarios** (needs T02 and this document; beside T05 and
T06).
1. The harness additions listed at the head of section 10.
2. The scenarios of the table, with upstream's outputs as expectations;
   `nav2_probe.py` shows each sequence of messages.
3. The Lean node registered as expected failure until T06 lands, with ND4,
   ND5 and ND6 as the named differences.

What each needs from the others: T06 needs T05's definitions and nothing
from T07. T07 needs nothing from T05 and T06; it needs this document's
table and the probes. T08 needs all three.

## 12. What still separates this from Nav2's default configuration

After pieces 1 to 3, **as built** (T05, T05b, T06; compared with upstream
by T07 and T08; corrected by T09 after the review CM5). What was built: a
lifecycle node with the bond, as a stand-alone executable and as the
component `roslean_collision_monitor::CollisionMonitorNode`; the state
topic and the polygon publishers; the scan's transform from TF, with base
shift correction, applied in C++; polygon zones fed by `polygon_sub_topic`.
Nav2's own `navigation_launch.py` with the package name changed, and
`nav2_params.yaml` with the fixed-polygon edit of 8.6, bring the node up,
activate it and bond with it, stand-alone and as a component in
`nav2_container` (review CM5, "Nav2"; without a robot, localisation or a
goal: that is task T10). What was not built is the table below. For each
feature: what it is in upstream's source, who
uses it in the two shipped parameter files (N21), and what closing the gap
would take. Sizes are rough estimates relative to all the collision monitor
work so far (model, proofs, interface, node), not measurements.

**The headline.** `nav2_bringup`'s `nav2_params.yaml` configures the
monitor with exactly one zone, and that zone is `approach`. After this
extension the Lean node runs in Nav2's bring-up with a **stop zone of the
footprint's shape** in its place (8.6). It still does not do what Nav2's
default does: slow the robot down in proportion to the time before a
predicted collision. The package's own example file, as shipped, is covered
completely: its lists name one fixed stop polygon and one scan.

| Feature | In upstream [S] | Used by `nav2_params.yaml` | Used by the package's example file | To close the gap |
| --- | --- | --- | --- | --- |
| `approach` action | `CollisionMonitor::processApproach`, `Polygon::getCollisionTime`: the robot's motion is simulated in steps of `simulation_time_step` up to `time_before_collision` (`src/kinematics.cpp`, `projectState`, `transformPoints`, with `cos` and `sin` of the turned angle); at each step the points inside the footprint are counted; the command is scaled by `collision_time / time_before_collision`. The footprint comes through `nav2_costmap_2d::FootprintSubscriber`, placed with the robot's pose at the footprint's stamp | **Yes: its only zone** | Defined, not in `polygons` | Model: the simulation loop over a rotating and translating point set, in exact arithmetic with `cos` and `sin` supplied by the shim (trusted) or enclosed in rational bounds (proved, a project of its own). To prove: the scale factor is in [0, 1]; no amplification; if the unscaled command reaches `min_points` points within the horizon, the published speed is at most the scaled one. Trusted: `cos`, `sin`, one division. The shim also needs the footprint subscriber's pose lookup. The largest single item: about half of the work so far |
| `limit` action | `processStopSlowdownLimit`: `ratio = min(1, linear_limit / hypot(vx, vy), angular_limit / |w|)`, clamped to [0, 1], applied to all three components | No | Defined (`PolygonLimit`), not in `polygons` | A new action kind. The ratio needs a square root and a division: computed by the shim and checked in Lean on bit patterns (trusted: the check's premises), or compared in squares exactly. To prove: no amplification; the published speeds respect both limits up to rounding. About a tenth |
| Velocity polygons | `src/velocity_polygon.cpp`: the zone's shape is the first sub-polygon whose ranges contain the command (`linear_min/max`, `theta_min/max`, and for `holonomic` a direction from `atan2`); if none matches, the previous shape stays | No | Defined (`VelocityPolygonStop`), not in `polygons` | Comparisons of `double`s, exact on bit patterns (a signed `F64.le`). The shape becomes a function of the command, so `evaluate` depends on the command (it does not today). The holonomic branch needs `hypot` and `atan2` from the shim. Non-holonomic: about a tenth |
| Dynamic polygons | `polygon_sub_topic` for a polygon | No (it uses `footprint_topic`, which only `approach` reads) | No | **Closed by this extension** (N29–N35) |
| Dynamic radius | `polygon_sub_topic` on a circle: `std_msgs/Float32` (N36) | No | No | One event; `radiusMm` exists. Small |
| Point cloud source | `src/pointcloud.cpp`: `PointCloud2` with `x`, `y`, `z` (and optionally `height`), `min_range` on the 3D length, the 3D transform, a height filter `min_height`…`max_height` on the transformed `z` or the `height` field | No (its `scan` entry carries `min_height` and `max_height`, which a scan never reads) | Defined (`pointcloud`), not in `observation_sources` | The shim's transform keeps the third coordinate; the height comparison in Lean on a third pattern per point; decoding of `PointCloud2` layouts in the shim (trusted). Clouds have 10⁵ points and more, handed over with every command (option A): the cost of the hand-over has to be measured first. About a fifth |
| Range source | `src/range.cpp`: one `sensor_msgs/Range` becomes an arc of points every `obstacles_angle`, with `cos` and `sin` in `float` | No | No | Like the scan path with a different generator of scan-frame points in the shim (T2). Small |
| Polygon source | `src/polygon_source.cpp`: `PolygonInstanceStamped` messages kept per id, dropped when too old, sampled along their edges every `sampling_distance` (`hypot`, `ceil`, a division) | No | No | The sampling in the shim (trusted) or in Lean with a rational square-root bound; a list of stored polygons with ids in the state. About a tenth |
| Costmap source | Not in Jazzy: `src/` has no such file | — | — | — |
| Several sources | `process`, first loop: every enabled source in order, the first invalid one stops; `<zone>.sources_names` selects which sources a zone counts (`Polygon::getPointsInside`) | No (one source) | No (one in the list) | `State.scan` and `sourcePoints` become lists; the theorems generalise without new ideas. About a tenth, mostly proofs |
| `~/toggle`, run-time `enabled` | `toggleCMServiceCallback`; `Polygon::dynamicParametersCallback`, `Source::dynamicParametersCallback` | No | No | Three events and a flag in the state (ND8). Small |
| Collision point markers | N17 | No | No | The shim has the points (option A); a publisher (ND9). Small, display only |
| Component | N18: Nav2's bring-up loads the monitor into a container by default (`bringup_launch.py`) | Bring-up, not parameters | — | **Closed by this extension**, with one limit: loaded as a component (tested in the three container kinds of `rclcpp_components`) the node does not survive a `/clock` message with a negative time, as upstream's component does not; the stand-alone executable does (SPEC.md, D15, ND10). To close that too: the container would have to catch the exception, which is not this node's code |
| A zone fed by a topic, after a message with a NaN or a vertex a kilometre away | Taken as it is (N33) | No | No | Not reproduced, by decision: the zone demands its action until a usable message arrives (ND6); a refresh that cannot be taken leaves the zone where it was (ND13) |
| `action_type: none` | Accepted, does nothing (UPSTREAM_BEHAVIOUR.md 23) | No | No | Trivial; refused today |
| `max_points` | Deprecated alias (R11) | No | No | Refused (D10) |
| `use_realtime_priority` | `on_configure`: `nav2_util::setSoftRealTimePriority` | No | No | A call in the shim. Trivial |

What remains different by design even where a feature is covered: D3 and D4
(millimetres and the grown zones), D5 (negative ratios refused), D6, D8–D10,
D12, and ND1–ND13 (SPEC.md, "Deliberate differences").

What is upstream's own weakness and is reproduced, not fixed, unless the
user decides otherwise (section 13 and PLAN.md): no output without commands
(R1); zero only for `stop_pub_timeout` (R15); `source_timeout: 0` switching
off the stop for a missing transform (N27); a topic zone without shape
(N32); a footprint zone left behind (N34).

## 13. Questions for the user

Each is implemented as upstream does it until decided, as PLAN.md says.

(T09) All were decided by the user on 2026-11-24 (tasks/PLAN.md,
"Decisions"): 1 as upstream, no action; 2 accept, as upstream, and document
the fixed polygon as the recommended edit; 3 option A; 4 as upstream, yes;
5 reproduce the wait; 6 silent; 7 a stand-alone process is acceptable (the
component works too); 8 the fixed polygon, the two-line change mentioned.
One question arose later and has only a default (the review CM5, finding
1): what a topic zone does after a polygon message with a NaN vertex. The
node stops (SPEC.md, ND6); confirmed by the user on 2026-11-25.

1. **A stop zone fed by a topic that has no shape yet** takes no action
   (N32, "No shape"). The fail-closed alternative: such a zone, if its
   action is `stop`, is triggered; or the source rule is extended to "a
   stop zone without shape stops, named after the zone". With Nav2 this
   would hold the robot until the local costmap has published its first
   footprint, which is before anything can move. It would be a numbered
   difference.
2. **A footprint zone that is left behind** (N34): accept as upstream;
   or give a shape in another frame a maximum age after which the zone
   counts as having no shape; or recommend the fixed polygon only and
   refuse `polygon_sub_topic` on the costmap's topic. Interacts with
   question 1.
3. **Option A or B** for the transform (section 5). Recommended: B.
   **Decided: A** (2026-11-24), with `-ffp-contract=off` on every platform.
4. **`source_timeout: 0` also disables the stop for a missing transform**
   (N27). Same question as SPEC.md, open decision 5.
5. **The wait for a transform inside the callback** (N26) holds up every
   command for up to `transform_tolerance` and, with base shift
   correction, for the time until the next odom sample. Reproduced as
   upstream does it. The alternative, a lookup without wait that stops at
   once, would stop the robot at every command in a normal Nav2 set-up with
   correction on, so it is not offered as a default.
6. **Shutdown** (ND4): the Lean node falls silent where upstream keeps
   forwarding. Assumed wanted.
7. **Composition** (ND10): is a stand-alone process acceptable if loading
   the Lean runtime into Nav2's container turns out not to work?
8. **Which form of the Nav2 edit to document** (8.6): the fixed polygon
   (recommended here), the two-line change of Nav2's zone, or both.
