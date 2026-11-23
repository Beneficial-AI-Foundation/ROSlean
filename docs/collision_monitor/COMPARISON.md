# Collision monitor: the Lean node against upstream

The ROSlean collision monitor (`ros/roslean_collision_monitor`) run through
the black-box harness that pins down upstream's `nav2_collision_monitor`
(`ros/roslean_collision_monitor/test/`, findings in
[UPSTREAM_BEHAVIOUR.md](UPSTREAM_BEHAVIOUR.md)), beside upstream's binary on
random cases, under fault injection, and under load. Rule numbers `R…`,
differences `D…` and trusted items `T…` are those of [SPEC.md](SPEC.md).

**Short answer.**

- *Behaviour.* In every fixed scenario and every random case the Lean node
  differs from upstream only where the SPEC says it does. No divergence in
  the unsafe direction was found: in {{R_cmds}} commands of {{R_cases}}
  random cases, and in 34 million point tests against the core, upstream
  never stopped or slowed where the Lean node did not. The zones grow as
  stated and no more: the farthest point outside upstream's zone that the
  core counted was 3.41 mm for a circle (bound 3.5) and 4.17 mm for a
  polygon (bound 4.4).
- *Defects.* A `/clock` message with a negative time aborts the node (as
  it aborts upstream). The node's command path is slow for polygon zones,
  about 200 ns per reading and zone, 50 to 65 times upstream's: with a
  1080-reading scan and two polygons a command takes 0.43 ms, with 10,000
  readings 4.0 ms. And two matters of the build: the shim is compiled
  without optimisation, and on macOS it then computes 2.6 % of the scan
  points one float32 step away from upstream's, far inside the margin of D4.
- *Memory.* Resident memory of the node over 32 minutes of load
  (1.15 million messages sent to it): 26.25 MB after the first minute,
  26.31 MB from the tenth minute to the end. Over 2 million events in
  process it does not move at all. It does not grow.
- *The two documents.* SPEC and UPSTREAM_BEHAVIOUR.md contradict each other
  in one place (how often a timeout is converted); the binary sides with the
  SPEC. Every statement that only the SPEC makes and that could be put to
  the binary was confirmed (18 new scenarios).

Run on 2026-11-23 on macOS 26.7 arm64 (Apple M5 Pro, 18 cores), ROS 2 Jazzy
from RoboStack through pixi, Fast DDS, `nav2_collision_monitor` 1.3.12, Lean
4.37.0-rc1. Other work ran on the machine at the same time (see
[Measurements](#measurements)). Nothing here was run on Linux.

## What was run

| What | Command | Result |
| --- | --- | --- |
| Lean node, fixed scenarios and probes | `scenario.py NODE --profile profile_roslean` (ctest `roslean_scenario`) | {{C_scen}} |
| Upstream, fixed scenarios, after the changes to the harness | `scenario.py` | 55 scenarios, 277 checks, 0 failures |
| Reference node, after the changes | `scenario.py reference_node.py --no-tf --skip negative-stamp` | 45 scenarios, 238 checks, 0 failures |
| Upstream, statements that only the SPEC makes | `scenario.py --profile profile_upstream --own` (ctest `upstream_probes`) | 18 scenarios, 48 checks, 0 failures |
| Lean node beside upstream, random | `random_diff.py NODE UPSTREAM 0 600 --roslean` (`pixi run cm-diff`) | {{R_line}} |
| The same with the scan frame turned | `… 1000 1120 --roslean --yaw` | {{Y_line}} |
| Can that test fail? | `… 0 3 --roslean --self-test swap`, `… shrink` | {{T_line}} |
| Core against upstream's zone tests, point by point | `band_bounds 100000 7` (ctest `band_bounds` with 20,000) | 34.0 million points, 340 zones: both bounds hold |
| Fault injection, both nodes | `fault_injection.py NODE UPSTREAM --big` (ctest `roslean_fault_injection`; `pixi run cm-faults`) | 23 cases, 124 checks, as expected; one case kills the node |
| Quality of service and bursts | `probe_qos.py NODE --no-state-topic` | as upstream |
| Per-call latency in process | `run_cm_bench.py core` | [below](#per-call-latency) |
| Latency through the nodes | `run_cm_bench.py node` | [below](#through-the-nodes) |
| Memory | `run_cm_bench.py soak --minutes 32` | [below](#memory) |

`NODE` is `roslean_collision_monitor/collision_monitor`, `UPSTREAM` is
`nav2_collision_monitor/collision_monitor`. The registered tests are listed
under [Running it](#running-it).

## Behavioural differences from Nav2

Every difference observed, with its number in the SPEC. "Seen in" names the
scenario of `scenario.py` or `probes.py`, the case of `fault_injection.py`,
or the random test.

| # | Upstream | Lean node | Seen in |
| --- | --- | --- | --- |
| D1 | `action_type: none` accepted, zone does nothing | refused at start-up | `action-none` |
| D2 | lifecycle node | plain node, active at once | every run: the harness activates upstream and finds the Lean node running |
| D3, D4 | a point exactly on a circle is outside; polygon: left and top edges inside, right and bottom outside; one corner of four inside | every boundary point is inside, and points up to 3.5 mm (circle) or 4.4 mm (polygon) beyond it can be | `boundary-circle` (2 checks), `-0.7`, `-0.3`, `boundary-polygon` (4), `-0.3` (2), `-0.7`, `angle-accumulation`, `offset`, `rotated`, `roslean-vertex-tr`, `-br`, `-bl`; random test; `band_bounds` |
| D5 | `slowdown_ratio` −0.5 reverses the command, −1 does nothing, −0.0 gives negative zeros | refused | `ratio-negative`, `ratio-minus-one`, `probe-ratio-negative-zero` |
| D6 | a command of 10⁻²⁰⁰ in a slowdown zone passes unslowed (squared norm underflows); 1.7·10³⁰⁸ passes unslowed (overflows); 5·10⁻³²⁴ passes | slowed: 5·10⁻²⁰¹; 8.5·10³⁰⁷; 0.0 | `signs-slowdown`, fault case `extreme-commands-slowdown` |
| D8 | a scan with a negative stamp aborts the process at the next command | no abort. The stamp counts as 0: stop at clock 100 s; **used as fresh** while the clock is below `source_timeout` | `negative-stamp`; fault cases `negative-stamp`, `negative-stamp-nanosec`, `negative-stamp-early-clock` |
| D9 | negative `source_timeout`: scans must be stamped that far ahead; negative `stop_pub_timeout`: each stop publishes zero once | refused | `timeout-negative`, `probe-stop-pub-negative` |
| D10 | `max_points: 1` makes the threshold 2; `min_points: 4294967297` is 1 | refused | `probe-max-points`, `probe-min-points-wraps` |
| D11 | `base_shift_correction: true` | refused | `offset-shift-correction`, `shift-correction-same-frame`, `no-odom` |
| D11 | scan frame placed by a static transform | placed by `scan.roslean_sensor_pose`; same answers except D4 | `offset`, `rotated`, `roslean-vertex-*`; random test, with yaw 0, π and other angles |
| D11 | missing transform: the stop is delayed until the node clock has advanced by `transform_tolerance` | the stop is published at once; the parameter is ignored | `transform-wait` |
| D13 | `state_topic` publishes the acting zone | parameter refused; no state messages | `roslean-state-topic`; no scenario checks a state message for the Lean node |
| D14 | a refused configuration fails the `configure` transition | exit code 1 | all refusals above, `triangle`, `probe-wrong-type` |

New differences, not recorded in the SPEC: **none in behaviour.** What the
SPEC does not say is listed under [Defects](#defects): the abort on a
negative `/clock` (the same as upstream, so not a difference), the run time
of the command path, and scan points that differ from upstream's in the
last bit (invisible in every answer).

Not observable from outside and so not confirmed here: D6(b) (which action
the core reports for a zero command in a slowdown zone; the published bits
are equal and were compared), D7 (one clock reading per event; under
simulated time both readings are equal) and D12 (points beyond 1000 m; the
fault cases with infinite and 3·10³⁸ m readings agree with upstream).

Where the two nodes were found to agree although one might not expect it:
a scan in a frame other than the base frame without a transform (both
stop; also for an empty name, a trailing blank, a leading slash and a name
of 100,000 characters); `source_timeout` below 1 ns (both treat it as 0:
the check is off); `radius: -0.5` (a 0.5 m zone in both); ratios of +∞ and
NaN (accepted, never applied); a disabled source or zone; the default
`<source>.source_timeout` to the nanosecond, including the nanosecond that
upstream's double conversion loses; negative zeros in commands and in
slowed commands; the clock before the first `/clock` message; a backward
jump of the clock; quality of service; and how many commands of a burst
are answered (of 100 sent back to back: Lean 7 to 10, upstream 2 to 10; of
1000: Lean 22 and 22, upstream 37 and 27).

## Fixed scenarios

`scenario.py` has 55 scenarios with 277 checks whose expected values are
what upstream publishes. The Lean node cannot be given `state_topic` (D13)
and has no transforms (D11), so the harness was extended: a *profile*
(`test/profile_roslean.py`) states, scenario by scenario, each answer that
is expected to differ and the number of the difference, the scenarios the
node must refuse, and what replaces a transform. Everything else must be
exactly as upstream, and a stated difference that does not occur is a
failure as well. With the profile the node passes: {{C_scen}}.

Of the 55 scenarios:

| Outcome for the Lean node | Scenarios |
| --- | --- |
| identical to upstream in every check (state messages apart, D13) | 34, among them `timeout`, `timeout-unix-clock`, `timeout-decimal`, `timeout-zero`, `stop-publication`, `zero-input`, `non-finite-command`, `invalid-ranges`, `length-mismatch`, `before-clock`, `commands-cease`, `stamped`, `no-transform`, `timeout-zero-no-transform` |
| some checks differ, each stated with its number | 12: `boundary-circle`, `boundary-circle-0.7`, `boundary-circle-0.3`, `boundary-polygon`, `boundary-polygon-0.3`, `boundary-polygon-0.7`, `angle-accumulation` (D4, 12 checks); `offset`, `rotated` (pose for transform, D11; one D4 check each); `signs-slowdown` (D6, 1 check); `negative-stamp` (D8, 1 check); `transform-wait` (D11, 2 checks) |
| must be refused, and is | 7: `ratio-negative`, `ratio-minus-one` (D5), `timeout-negative` (D9), `action-none` (D1), `offset-shift-correction`, `shift-correction-same-frame`, `no-odom` (D11) |
| refused by upstream too | 1: `triangle` |
| not run, replaced | 1: `boundary-vertices` needs four scan frames; replaced by four scenarios with one sensor pose each (`roslean-vertex-*`, three D4 checks) |

Classification of the checks that fail without a profile: all are (a), a
numbered difference of the SPEC. None is (b), a difference the SPEC does
not record; none is (c), a defect of node, shim or model; none is (d), a
wrong expectation of the harness. The defects found came from the other
tests.

One loss of sharpness should be known. For upstream the boundary scenarios
distinguish "one float32 step inside" from "on the boundary", and so pin
down float32 products and the running sum of angles (finding 13). For the
Lean node both sides of every such pair are inside (D4), so these scenarios
no longer test its arithmetic to the last bit. That is done by
`smoke_conversions.cpp` on the shim's `encode_scan`, and statistically by
the random test, where a point less than a micrometre outside a boundary
decided {{R_tiny}} answers.

## Random cases, Lean node beside upstream

`random_diff.py` generates a configuration and 150 to 250 events per seed,
runs two nodes on the same messages and the same `/clock`, and compares
their answers command by command. For this comparison (`--roslean`):

- only configurations both nodes accept: no base-shift correction; the
  ratios, timeouts and `min_points` of the generator are accepted by both;
- the Lean node gets no state topic, and the pose that upstream gets as a
  static transform it gets as `scan.roslean_sensor_pose`;
- boundary cases are kept. The original generator redraws a scan with a
  point within 0.1 mm of a boundary. Here nothing is redrawn, and a fifth of
  the scans are aimed at a point of a zone's boundary (also at vertices), or
  up to 6 mm from it;
- the answers are not compared bit for bit, nor with a tolerance. They are
  judged by the two bounds of the SPEC (`test/cm_bounds.py`):

  **S, safe direction.** Whatever upstream counts, the Lean node counts. By
  the monotonicity theorem this means that what the Lean node demands is
  never less restrictive than what upstream demands: each component is
  upstream's scaled by a factor between 0 and 1. This is checked on the two
  outputs alone. It needs no model and has no tolerance.

  **G, growth.** The Lean node counts no point farther than 3.5 mm (circle)
  or 4.4 mm (polygon) outside upstream's zone. For every command the test
  computes all answers that are possible when each point within that
  distance may or may not count; the node's answer must be one of them. The
  points are computed as upstream computes them (float32 angle sum, the C
  library's float sine and cosine as an optimised build calls them, float32
  products, the transform in double). Upstream's own answer is held against
  the same computation with a band of 1 nm, which it met for every command.

  Each node's stop-publication state (R15) is followed from its own
  outputs, because after a legitimate difference the two nodes can be at
  different points of that period.

Results, seeds 0 to 599:

| | |
| --- | --- |
| Cases outside the bounds | {{R_bad}} |
| Unsafe divergences (S violated) | {{R_unsafe}} |
| Cases that passed only on the slow retry | {{R_retried}} |
| Commands | {{R_cmds}} |
| Scans; point-in-zone tests | {{R_scans}}; {{R_tests}} |
| Points within the growth band of a zone | {{R_band_c}} of {{R_pts_c}} for circles, {{R_band_p}} of {{R_pts_p}} for polygons |
| Commands whose answer depends on such a point | {{R_amb}} ({{R_amb_pct}} %) |
| Commands for which the two nodes demand different velocities | {{R_differ}} ({{R_differ_pct}} %); in all of them the Lean node is the more restrictive |
| Commands for which the published messages differ | {{R_pubdiff}} |

So when a point lies in the band and decides, the Lean node gives the more
restrictive answer in {{R_frac}} % of the cases, and upstream's answer in
the rest. The last row is larger than the one before because a difference
shifts the Lean node's stop-publication period: it then goes silent, or
resumes zero, at other commands than upstream.

How far outside upstream's zone the point lay that decided a difference
(the smallest reach that explains the Lean node's answer):

{{R_reach_table}}

With the scan frame turned by π/2, −π/2, 0.3, −2.5, 1.0 or 3.0 rad in half
of the cases (`--yaw`; upstream gets the quaternion, the Lean node the yaw):
{{Y_line}}.

*Can the test fail?* `--self-test swap` exchanges the two nodes, so that
upstream is judged as if it were a Lean node that misses points:
{{T_swap}}. `--self-test shrink` allows a tenth of the growth: {{T_shrink}}.

*Defects of this test, found and fixed while it was written.* (1) The first
version collected the possible answers in a Python `set`. `(0.0, -0.0, 0.0)`,
a command scaled by a ratio of 0, and `(0.0, 0.0, 0.0)`, a stop, are equal
as tuples, so one of them was lost, and three seeds were reported as
outside the bounds (26, 387, 389). The answers are now told apart by bit
pattern. (2) The count of differences was wrong for commands that one node
leaves unanswered. (3) Upstream's points were computed with `sinf` and
`cosf`. Upstream's binary calls `__sincosf_stret`, which on this machine
returns a value one float32 step away for 51,381 of 2,000,000 random
angles; seed 1057 of the `--yaw` run was reported for it ("upstream is not
as the model says": five points 15 nm inside a circle by the test's
arithmetic, outside by upstream's). The test now calls the same routine.
This is also how defect 4 below was found. The 600 seeds were run three
times, once after each correction, with the same verdict each time (0
cases outside the bounds, 54,043 commands); the numbers above are from the
last run.

### Point by point

The random test sees a point only through `min_points` and the action. The
question "how often, and by how much" is answered exactly by
`test/band_bounds.cpp`: for 162 circles and 178 polygons (the test zones,
Nav2's example, random radii and star-shaped polygons with float32 vertices
up to 50 m from the origin, some crossing themselves, some with the first
vertex repeated, slivers 1 to 20 mm wide, many shapes and points on whole
millimetres) it draws 100,000 points within 6 mm of each boundary, and asks
upstream's test (the expressions of `circle.cpp` and `polygon.cpp` in
double) and the Lean core through its C interface, as the node does.

| | Circles | Polygons |
| --- | ---: | ---: |
| Points | 16,200,000 | 17,800,000 |
| Inside for upstream, outside for the core (S) | 0 | 0 |
| Counted by the core beyond the bound (G) | 0 | 0 |
| The two tests disagree | 32.7 % | 36.4 % |
| Farthest point outside upstream's zone that the core counts | 3.409 mm | 4.167 mm |
| Nearest point outside upstream's zone that the core does not count | 1.000 mm | 0.360 mm |

Share of points that the core counts as inside, by distance outside
upstream's zone (the same run; every point inside upstream's zone is
counted):

| Distance outside | Circles | Polygons |
| --- | ---: | ---: |
| 0 to 0.25 mm | 100 % | 100 % |
| 0.25 to 0.5 mm | 100 % | above 99.99 % |
| 0.5 to 1 mm | 100 % | 99.86 % |
| 1 to 1.5 mm | 94.8 % | 97.4 % |
| 1.5 to 2 mm | 73.8 % | 87.1 % |
| 2 to 2.5 mm | 38.9 % | 66.0 % |
| 2.5 to 3 mm | 11.3 % | 37.5 % |
| 3 to 3.5 mm | 0.8 % | 10.3 % |
| 3.5 to 4 mm | 0 | 0.8 % |
| 4 to 4.5 mm | 0 | 0.02 % (none beyond 4.167 mm) |
| beyond 4.5 mm | 0 | 0 |

In words: a zone of the Lean node is upstream's zone grown by 1 mm for
certain (circle) or about 0.4 mm (polygon), by 2 mm for half of the points,
and by the stated bound at most. Among random points *within 6 mm of a
boundary* a third are classified differently; how often that matters on a
robot is the share of scan points that close to a boundary, times how
often such a point decides `min_points`.

## SPEC against UPSTREAM_BEHAVIOUR.md

The SPEC was written from upstream's source, UPSTREAM_BEHAVIOUR.md (UB)
from its binary, without sight of each other. Statements were compared one
by one. Where only one document speaks, or the two differ, the statement
was put to the binary: by an existing scenario, or by one of the 18 new
ones in `test/probes.py` (expectations: what the binary did; all 18 passed
at the first run, i.e. every prediction made from the SPEC was right).

**They disagree.**

| Statement | SPEC | UB | Binary |
| --- | --- | --- | --- |
| Conversion of `source_timeout` | R17: the source's default is the node's timeout converted to ns, back to seconds, and to ns again; 2.094 s becomes 2,093,999,998 ns | finding 16: "Rounding enters once, when the parameter is converted" | **SPEC.** `probe-timeout-converted-twice`: a scan 2,093,999,998 ns old is used, one 2,093,999,999 ns old is too old. UB's example, 1.001, is a value that loses nothing the second time. UB corrected |
| A scan stamped in the future | R7: "is never too old" | finding 5: a negative `source_timeout` rejects every scan not stamped that far ahead | **UB**, for negative timeouts (`timeout-negative`). R7's formula is right; its sentence holds for timeouts that are not negative. No consequence for the Lean node, which refuses them (D9) |
| A zone with `min_points <= 0` | R11 with R8: triggers without any point, also when there is no scan and the timeout is 0 | finding 18: "triggered by any usable scan" | **SPEC**: `probe-min-points-zero-no-scan` stops with no scan at all |

**Only the SPEC says it; the binary agrees.**

| Statement (SPEC) | Scenario |
| --- | --- |
| R17: a timeout given to the source itself is converted once (2.094 s: 2,093,999,999 ns) | `probe-timeout-of-source` |
| R15/R17: `stop_pub_timeout` is converted once (UB: "not probed") | `probe-stop-pub-decimal` |
| R7: a positive timeout below 1 ns is 0, i.e. the check is off | `probe-timeout-below-1ns` |
| D9: a negative `stop_pub_timeout` publishes zero once per stop | `probe-stop-pub-negative` |
| R6: a disabled source contributes no points and cannot cause a stop, also when no scan ever came | `probe-source-disabled` |
| R11: a disabled zone is skipped | `probe-zone-disabled` |
| R11: `max_points` replaces `min_points` by `max_points + 1` | `probe-max-points` |
| D10: `min_points` is read into 32 bits; 2³² + 1 is 1 | `probe-min-points-wraps` |
| R9: the sign of the radius is lost | `probe-radius-negative` |
| R13: a ratio of +∞ or NaN never takes effect | `probe-ratio-inf`, `probe-ratio-nan` |
| D5: a ratio of −0.0 gives zero (the binary publishes negative zeros) | `probe-ratio-negative-zero` |
| R13: of two triggered slowdown zones with equal ratios the earlier acts (UB: "the smallest resulting velocity wins, whatever the order") | `probe-equal-ratios`, `-reversed` (by the state topic) |
| R17: a parameter of the wrong type makes configuration fail | `probe-wrong-type` (`radius: 1`) |
| R17: `min_points` defaults to 4, `slowdown_ratio` to 0.5 | `probe-defaults` |
| D8: `sec = -1` with `nanosec ≥ 10⁹` is refused by upstream although the sum is positive | fault case `negative-stamp-nanosec`: upstream aborts |
| D6: components above about 10¹⁵⁴ are not slowed | fault case `extreme-commands-slowdown` |
| A2: after a backward jump of the clock zero keeps being published | fault case `clock-backwards` |

**Only the SPEC says it; not put to the binary.** R19 (commands dropped
while the node is inactive; `~/toggle`; `enabled` changed at run time); D7
(two clock readings); D9 (non-finite and huge timeouts are undefined
behaviour upstream); D14 (the unconfigured process stays up; the harness
sees the failed transition only); the defaults of the timeouts and topic
names in R17, which the test configuration always sets; R2's log message.

**Only UB says it.**

| Statement (UB) | Relation to the SPEC and the Lean node |
| --- | --- |
| 10: a missing transform delays the stop by `transform_tolerance` of node time; no answer at all while a simulated clock stands still | Not in the SPEC. The Lean node answers at once (`transform-wait`); covered by D11, "accepted and ignored" |
| 10: with `base_shift_correction: true` a transform from `odom` is needed even for a scan in the base frame | Not in the SPEC; the Lean node refuses the setting (D11) |
| 11: of N commands sent back to back only some are answered | Follows from R18 (keep-last 1). The Lean node behaves alike (measured) |
| 12: the state topic reports changes of the zone name only | Outside the SPEC's subset (D13) |
| 15: of the four corners of the square only (−0.5, 0.5) is inside | Follows from R10; not stated. Lean node: all four inside (D4) |
| 13: float32 details (at the float32 nearest −π/2 the point is exactly on the edge; the 361-ray sum ends 1.1·10⁻⁵ rad short) | Consequences of R3. Reproduced by the shim's `encode_scan`; not distinguishable through the Lean node's zones (see above) |
| 21: before the first `/clock` message the clock reads 0 | Not in the SPEC. The Lean node behaves alike (`before-clock`) |
| 19: a negative reading with a negative `range_min` is a point behind the sensor | Follows from R3 and R4. The Lean node behaves alike (`invalid-ranges`) |
| 23: quality of service of each topic | R18 says the same of the command topics and the scan. The Lean node has the same settings (`probe_qos.py`) |

Everything else is said by both in compatible words: triggers (R1, UB 2),
non-finite commands (R2, UB 7), which readings count (R4, UB 19), the scan
received last (R5, UB 5), age by header stamp and its exact boundary (R7,
UB 5 and 16), no scan and timeout 0 (R8, UB 4 and 21), strict circle (R9,
UB 14), polygon edges and the four-vertex minimum (R10, UB 8 and 15),
`min_points` as "at least" (R11, UB 18), stop before slowdown and the
strictly smaller norm (R12, R13, UB 9), `stop_pub_timeout` (R15, UB 3),
three components and the copied header (R16, UB 6 and 22), `ranges` alone
deciding the number of readings (R3, UB 20), and the abort on a negative
stamp (D8, UB 1).

## Fault injection

`test/fault_injection.py`: 23 cases, each on fresh processes, both nodes on
the same messages under simulated time (zones: stop circle 0.5 m, slowdown
circle 1.0 m, `min_points` 3, `source_timeout` 0.5 s). The script asserts
the Lean node's answer to every command and that the process is alive at
the end of each case; upstream's observed answers are asserted too.

| Input | Lean node | Upstream |
| --- | --- | --- |
| Scans cease | passes until the scan is exactly `source_timeout` old; stop 1 ns later; zero for `stop_pub_timeout`, then silence; passes when scans return | same |
| No scan ever | stop | same |
| 360 readings, all NaN, all +∞ or all −∞ | **passes** (free space, R4; SPEC open decision 1) | same |
| NaN and ∞ among three valid readings in the stop zone | stop | same |
| `range_max` +∞ with every reading +∞ | passes (points saturate at 1000 m) | passes |
| `range_min` or `range_max` NaN; `range_min > range_max`; both −∞ | passes (no reading valid) | same |
| `angle_min` NaN or ∞; `angle_increment` NaN or ∞ | points after the first are in no zone: passes | same |
| Angles of 10³⁰ rad; readings of 3·10³⁸ m; a subnormal, −0.0 and 0.0 with `range_min: 0` | stop; passes; stop | same |
| `ranges` empty, also with NaN in every other field | passes | same |
| 1080 readings where the angle fields allow 360; 3 where they allow 1080; `angle_max < angle_min` | all readings used, angle fields other than `angle_min` and `angle_increment` ignored | same |
| 10,000 readings | passes, slows, stops as the readings say | same |
| 100,000 readings (400 kB) | stop | stop |
| 1,000,000 and 10,000,000 readings (4 and 40 MB) | the message does not arrive (best-effort subscription): the previous scan stays in force | same |
| the same sizes handed to the callback in process (`bench_cm`) | handled: 47 ms and 0.48 s per scan, 15 ms and 0.15 s per command; no crash | 2.8 ms and 31 ms per command |
| Stamp of −1 s | stop (stamp taken as 0, D8); next good scan passes | **aborts** at the next command |
| Stamp `sec = -1, nanosec = 2·10⁹`; `sec = -2³¹` | stop | aborts |
| Clock at 0.25 s, scan stamped −5 s | **used as fresh** (D8): stops for an obstacle scan, passes for a clear one, until the clock passes `source_timeout` | aborts |
| Stamp in 2038 | used, and still 100,000 s later with no further scan (R7) | same |
| An older clear scan arriving after a newer obstacle scan | the last received counts: passes (R5) | same |
| Commands with NaN or ±∞ in any of the six components, also with an obstacle in the stop zone | nothing published (R2) | same |
| Commands of ±1.7·10³⁰⁸ and 5·10⁻³²⁴, no obstacle | pass unchanged | same |
| The same in a slowdown zone | halved; the subnormal becomes 0.0 (D6) | passed unslowed |
| 1000 commands back to back | 22 answered, then normal | 37 and 27 answered |
| 500 scans back to back | the last counts | same |
| Scan in a foreign frame; empty frame name; base frame name with a blank or a slash; a name of 100,000 characters | stop (D11), also discarding the stored scan | stop (no transform) |
| Clock jumps back 50 s, 100 s, to 0 | an expired scan is used again; a stop period that began "later" publishes zero again (A2) | same |
| **`/clock` message of −5 s** | **aborts** (defect 1) | aborts |

So one input crashes the Lean node. Everything else is handled, and
handled as the SPEC says.

## Defects

Found in the node, the shim or the model (category c). Not fixed here.

1. **A `/clock` message with a negative time ends the process.** With
   `use_sim_time`, a `rosgraph_msgs/Clock` message with `sec < 0` makes
   rclcpp throw `cannot store a negative time point in rclcpp::Time` in the
   thread of its time source; nothing can catch it there, and the process
   aborts (signal 6). The shim's `std::max(now().nanoseconds(), 0)` is never
   reached. Upstream dies in the same way, so this is not a difference, but
   the SPEC's aim is that no input ends the node (D8, "Bad data (c)"), and
   A2 says nothing is assumed about the clock.
   Reproduction: `fault_injection.py NODE --only clock-negative`, or
   `ros2 topic pub --once /clock rosgraph_msgs/msg/Clock "{clock: {sec: -5}}"`
   at a node started with `-p use_sim_time:=true`.
   Registered as `roslean_fault_clock_negative`, which asserts that the
   node survives and is marked `WILL_FAIL`. A fix probably needs the node
   option `use_clock_thread(false)` and a `catch` around the spin, or a
   clock subscription of the node's own; the node cannot repair it inside
   a callback. Severity: low on a robot (a simulator would have to publish
   a negative time), but it is the one way found to end the process from a
   topic.

2. **The command path is slow for polygon zones.** Each command tests every
   stored point against every zone in exact integer arithmetic, including
   the √5 mm band (`nearBoundary`). For circles this costs about 8 ns per
   reading and zone, for four-vertex polygons about 200 ns; upstream needs 3
   to 7 ns per reading for everything. See [Per-call latency](#per-call-latency): with two polygons a
   command costs 0.43 ms at 1080 readings and 4.0 ms at
   10,000, which at 50 commands a second is 20 % of a core.
   No test is registered for this: a threshold on run time would fail on a
   busy machine. The answers are right. What can go wrong is what BENCHMARK.md says of a
   slow mux: commands queue (keep-last 1, so they are dropped), and a
   command is judged at the time its callback starts. Nothing warns at
   start-up. Reproduction: `run_cm_bench.py core`.

3. **The shim is compiled without optimisation.** `pixi run build` sets no
   build type, so `collision_monitor_node.cpp` and `conversions.hpp` are
   compiled at `-O0` (the Lean core's C is compiled `-O3` by its own
   `CMakeLists.txt`). `encode_scan` then takes 165 µs for 1080 readings
   instead of 52 µs. The in-process measurements below use the optimised
   build; the node-level ones use the node as built.

4. **T2: the shim's points are not always upstream's, to the last bit.**
   SPEC, "Scan event", says the point of a reading is computed as
   upstream's `Scan::getData` computes it. The source is the same, the
   library call is not: the node as built calls `sinf` and `cosf`
   (`nm -u`), upstream's binary and any optimised build of the same header
   call `__sincosf_stret`, and Apple's library returns a different float32
   from that routine for 2.6 % of angles. So for about one reading in forty
   one coordinate of the node's point differs from upstream's by one
   float32 step: 6·10⁻⁸ m at 1 m. It cannot change a verdict of the
   comparison: the margin of D4 is a millimetre, both bounds were checked
   with points computed upstream's way, and the node passed. But "the same
   points as upstream" then depends on the optimisation level and the
   platform, `smoke_conversions.cpp` cannot see it (its reference is
   compiled with the same flags), and fixing defect 3 changes it. Not
   checked on Linux, where the two routines are `sincosf` and
   `sinf`/`cosf` of glibc.
   Reproduction: `test/t2_point_as_upstream.cpp`, registered on macOS with
   `WILL_FAIL`: for the angle 1.1233999729156494 the shim's cosine is
   0.4326196014881134, upstream's 0.4326196312904358. Seen from outside:
   `random_diff.py NODE UPSTREAM 1057 1058 --roslean --yaw` with the test's
   points computed by `sinf` and `cosf`.

Notes that are not defects of the node:

- D8's early-clock case is as the SPEC describes it and was seen: with the
  clock below `source_timeout`, a clear scan with a negative stamp lets
  commands pass. Upstream is dead at that point. REVIEWS, CM1/CM2 finding 9,
  proposes a separate input for a negative `sec`; this run supports it.
- Scans that consist only of NaN or ∞ pass, as upstream (open decision 1).

Found in the harness (category d), fixed here:

- `cm_harness.Side` always passed `state_topic`, which the Lean node
  refuses: now optional per node.
- `scenario.py` could only skip a scenario that publishes a transform; a
  profile now says what replaces it.
- `random_diff.py` could not compare nodes whose zones differ by design; see
  above. Its own first version had the zero-sign defect described there.
- UPSTREAM_BEHAVIOUR.md, finding 16, corrected (conversion of timeouts).
- `bench/roslean_bench` did not run on macOS (`$ORIGIN` in the run path;
  executables linked to the Python generator libraries abort at start).

## Measurements

In the manner of [BENCHMARK.md](../BENCHMARK.md). Sources:
`bench/roslean_bench/src/bench_cm.cpp`, `cm_load.cpp`,
`scripts/run_cm_bench.py`; raw data in `bench/results/cm_*.json`.

**Environment and its limits.** Apple M5 Pro, 18 cores, macOS; not a robot
computer, not a real-time system. Other agents ran simulators on the same
machine throughout, so the tails (p99.9, max) contain their scheduling. The
two columns of every table were measured in the same process or at the same
time on the same messages and can be compared with each other.

Upstream converts a scan to points when a command arrives (R3); the Lean
node converts it when the scan arrives and stores the points. So upstream's
scan callback only stores a pointer, and the two paths must be read
together.

Configuration: Nav2's example zones. "polygons": a stop box 0.3 × 0.6 m and
a slowdown box 0.8 × 0.8 m (ratio 0.3), `min_points` 4. "circles": radii 0.3
and 0.6 m. Scenes: "clear" (nothing within 3 m), "slow" (1.5 % of the
readings in the slowdown zone only), "near" (3 % in the stop zone).

### Per-call latency

In one process: the two callback bodies of the Lean node (clock reading,
`encode_scan`, core, product, publish to a topic without subscribers) and
upstream's own library, configured and activated, its `Scan::dataCallback`
and `CollisionMonitor::cmdVelInCallbackUnstamped` called directly. A new
message object per call; 20,000 calls per row (5,000 for 10,000 readings)
after a warm-up of a tenth. Microseconds.

| Readings | Zones | Call | Lean p50 | p99 | p99.9 | max | Upstream p50 | p99 | p99.9 | max |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 17.1 | 18.2 | 28.4 | 75.1 | 0.0 | 0.2 | 0.5 | 2.1 |
| 360 | polygons | command, clear | 147 | 159 | 172 | 284 | 3.0 | 3.9 | 6.5 | 22.9 |
| 360 | polygons | command, slowdown | 144 | 155 | 170 | 261 | 3.0 | 3.8 | 7.1 | 16.5 |
| 360 | polygons | command, stop | 71.8 | 77.8 | 90.0 | 185 | 2.2 | 2.8 | 5.0 | 18.9 |
| 360 | circles | scan | 17.2 | 18.1 | 25.5 | 50.0 | 0.0 | 0.2 | 0.5 | 1.1 |
| 360 | circles | command, clear | 6.6 | 7.2 | 9.8 | 25.4 | 1.8 | 2.1 | 3.9 | 34.9 |
| 360 | circles | command, slowdown | 6.4 | 7.1 | 11.9 | 23.8 | 1.8 | 2.1 | 3.5 | 13.6 |
| 360 | circles | command, stop | 5.5 | 6.1 | 10.0 | 57.2 | 1.7 | 2.0 | 4.1 | 15.5 |
| 1080 | polygons | scan | 51.0 | 58.0 | 84.0 | 150 | 0.0 | 0.4 | 0.7 | 1.9 |
| 1080 | polygons | command, clear | 429 | 452 | 533 | 1 250 | 7.5 | 10.2 | 18.5 | 83.1 |
| 1080 | polygons | command, slowdown | 428 | 470 | 540 | 1 322 | 7.5 | 9.9 | 19.2 | 31.9 |
| 1080 | polygons | command, stop | 211 | 228 | 265 | 498 | 5.4 | 7.5 | 12.2 | 38.4 |
| 1080 | circles | scan | 50.9 | 55.5 | 66.7 | 136 | 0.0 | 0.2 | 0.5 | 0.7 |
| 1080 | circles | command, clear | 18.1 | 19.8 | 30.2 | 40.1 | 4.2 | 4.9 | 7.0 | 118 |
| 1080 | circles | command, slowdown | 18.2 | 21.0 | 28.7 | 77.0 | 4.2 | 5.3 | 10.0 | 122 |
| 1080 | circles | command, stop | 15.8 | 19.5 | 29.1 | 141 | 4.0 | 5.6 | 10.2 | 59.2 |
| 10000 | polygons | scan | 487 | 548 | 788 | 898 | 0.2 | 1.2 | 3.6 | 12.8 |
| 10000 | polygons | command, clear | 4 006 | 4 334 | 4 932 | 5 625 | 62.4 | 79.4 | 112 | 164 |
| 10000 | polygons | command, slowdown | 3 957 | 4 193 | 4 573 | 4 983 | 62.1 | 71.4 | 93.1 | 110 |
| 10000 | polygons | command, stop | 1 983 | 2 078 | 2 204 | 2 351 | 45.5 | 53.6 | 72.8 | 170 |
| 10000 | circles | scan | 481 | 504 | 561 | 600 | 0.1 | 0.5 | 1.3 | 3.2 |
| 10000 | circles | command, clear | 172 | 193 | 212 | 287 | 32.7 | 37.9 | 52.2 | 71.2 |
| 10000 | circles | command, slowdown | 181 | 204 | 251 | 296 | 32.8 | 37.7 | 49.5 | 104 |
| 10000 | circles | command, stop | 155 | 172 | 213 | 297 | 30.8 | 35.5 | 48.8 | 55.0 |

"clear", "slowdown" and "stop" are the scenes; in the stop scene the first
zone decides and the second is not evaluated, and after `stop_pub_timeout`
nothing is published. Sum of the two medians, clear scene:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 164 | 3.0 | 54 |
| 360 | circles | 23.8 | 1.8 | 13 |
| 1080 | polygons | 480 | 7.5 | 64 |
| 1080 | circles | 69.0 | 4.3 | 16 |
| 10000 | polygons | 4 493 | 62.7 | 72 |
| 10000 | circles | 653 | 32.8 | 20 |

Reading:

- **Scan path.** The Lean node spends 47 to 49 ns per reading when a scan
  arrives: computing the point (`encode_scan`), copying 20 bytes per
  reading into the core, and converting to millimetres. Upstream spends
  that time when the command arrives. As built by `pixi run build` the
  shim is not optimised (defect 3) and the scan callback takes 165 µs
  instead of 51 µs at 1080 readings.
- **Command path, circles.** 17 ns per reading for the two zones, against
  upstream's 3 to 4 ns including its conversion of the scan: 4 to 5 times
  slower, 18 µs at 1080 readings.
- **Command path, polygons.** 400 ns per reading for the two four-vertex
  polygons, i.e. 200 ns per reading and zone, about 50 ns per edge:
  0.43 ms at 1080 readings and 4.0 ms at 10,000, where upstream needs 7.5 µs
  and 62 µs. This is defect 2. The cost is the exact integer test with its
  band, done for every point against every edge; most points of a real
  scan are metres away from every zone and could be excluded by one
  comparison with a bounding box.
- **Tail.** The Lean callbacks are steady: p99 within 1.1 times the median,
  p99.9 within 1.7 times, and the largest of 20,000 calls within 3 times
  (1.3 ms at 1080 readings), on a machine that was not idle. As with
  twist_mux there is no collector that pauses the node.
- **What it costs.** With 1080 readings at 25 Hz, commands at 50 Hz and two
  polygon zones: 25 × 0.05 ms + 50 × 0.43 ms = 23 ms of work per second,
  2.3 % of a core (upstream: 0.04 %). With 10,000 readings: 212 ms per
  second, 21 % of a core, and each command blocks the node for 4 ms. With
  circle zones: 0.2 % and 2 %.

### Through the nodes

Both node binaries side by side on the same topics (upstream activated
through its lifecycle), wall-clock time, `enable_stamped_cmd_vel`, polygon
zones. `cm_load` publishes scans at 25 Hz and commands at 50 Hz for 20 s;
the scene changes every 2.5 s. Latency from `publish` of a command to the
callback of a subscriber on the node's output. Microseconds.

| Readings | Node | Answered / sent | p50 | p90 | p99 | p99.9 | max | Answers that differ |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | lean | 852 / 1000 | 6 341 | 9 348 | 10 659 | 11 264 | 11 264 | 0 of 850 |
| 360 | upstream | 850 / 1000 | 5 661 | 9 059 | 10 539 | 11 142 | 11 142 | 0 of 850 |
| 1080 | lean | 851 / 1000 | 5 867 | 8 918 | 10 770 | 13 571 | 13 571 | 0 of 850 |
| 1080 | upstream | 851 / 1000 | 5 988 | 8 931 | 10 710 | 19 112 | 19 112 | 0 of 850 |
| 10000 | lean | 841 / 1000 | 9 249 | 12 319 | 14 350 | 28 503 | 28 503 | 0 of 841 |
| 10000 | upstream | 846 / 1000 | 5 807 | 9 118 | 11 127 | 105 580 | 105 580 | 0 of 841 |

About 850 of 1000 commands are answered by either node: the rest fall into
the silence after `stop_pub_timeout` in the stop scene. The two nodes gave
the same answer to every command that both answered.

The latencies are those of the middleware on this machine, not of the
nodes: they spread evenly between about 1 and 11 ms for both nodes, as if
messages were picked up on a 10 ms tick (BENCHMARK.md measured 0.1 to
0.3 ms on Linux). Against that, the Lean node's extra work does not show at
360 and 1080 readings (medians 6.3 against 5.7 ms, and 5.9 against 6.0 ms).
At 10,000 readings its median is 3.4 ms above upstream's, which is its scan
callback (about 1.5 ms as built) and its command callback (4 ms) queuing
behind each other. The node-level comparison should be repeated on Linux.

### Memory

Both nodes side by side for 32 minutes: 1080-reading scans at
200 Hz and commands at 400 Hz, i.e. 5 to 8 times the usual rates, the scene
changing every 2.5 s through pass, slowdown and stop:
384,000 scans and 768,000 commands, 1,152,000 messages sent. Resident set
size and CPU time of each process from `ps` every 10 s.

| Time | Lean node, RSS kB | Upstream, RSS kB |
| ---: | ---: | ---: |
| start | 26,352 | 30,416 |
| 10 s | 26,848 | 30,688 |
| 30 s | 26,880 | 30,704 |
| 220 s | 26,896 | 30,736 |
| 560 s | 26,944 | 30,784 |
| every sample from 560 s to 1920 s | 26,944 | 30,784 |

The table lists every change in 193 samples.

The Lean node's resident memory rises by 592 kB in the first ten minutes,
in four steps, and is then constant for the remaining 22.7 minutes. Upstream
rises by 368 kB in steps at the very same sample times, so the steps belong
to what the two processes share (the middleware), not to the core. The Lean
node is the smaller process here: 26.3 MB against 30.1 MB. (The 70 MB of the
twist_mux comparison were measured on Linux as proportional set size; these
are resident set sizes on macOS, where the Lean runtime's library is counted
differently. Only the curve is the result.)

CPU time over the run: Lean node 432 s (22.5 % of a core), upstream 111 s
(5.8 %). The difference is defect 2.

Not every message sent was processed. Of the 768,000 commands the Lean node
answered 282,882 and upstream 322,851; about 614,000 answers were to be
expected (the rest fall into the silence of the stop scene). At 400 Hz
several commands arrive within one of the middleware's 10 ms ticks, and a
subscription that keeps one message hands over the last (finding 11). So
the node processed commands at about 150 to 190 per second for 32 minutes,
still above the usual rates. Of 164,298 commands answered by both nodes,
192 (0.12 %) got different answers; they lie at the scene changes, where
one node has taken in the new scan and the other not yet. The same count
at the usual rates, above, is 0 of 2,541.

In process, 2 million events (`bench_cm --soak`, 1080 readings):
{{S_inproc}}

## What was not tested

- **Linux.** Everything ran on macOS arm64 against upstream 1.3.12. The
  harness authors ran upstream 1.3.14 in Docker; the Lean node was not built
  or run there in this work.
- **Wall-clock decisions.** All behavioural tests run under simulated time.
  Under wall-clock time only the load runs were made; no boundary of a
  timeout was probed there, and D7 remains unobserved.
- **The features outside the subset** (lifecycle management, `toggle`,
  run-time `enabled`, base-shift correction, several sources, point clouds,
  `approach` and `limit`, velocity polygons, the state topic): refusals are
  checked, behaviour is not comparable.
- **A sensor pose against a transform that changes**, or with roll and
  pitch. Fixed poses with yaw 0, π and six other angles were compared.
- **Ratios one rounding apart** (D6) and commands at exactly the under- and
  overflow limits: not generated. Single values far beyond the limits were.
- **Scans above 100,000 readings through the middleware**: they did not
  reach either node. In process the core took 10 million readings.
- **The wall-clock assumption of the harness**: one event at a time, 50 ms
  (random) or 200 ms (scenarios) apart. Nothing was done about the order of
  a scan and a command that arrive together, in either node.
- **Long runs.** 32 minutes; not hours or days.
- **Other middlewares, a real-time kernel, a robot computer, a robot.**
- **`pixi run test` as a whole** was not re-run; the new tests were run
  through `ctest` one at a time ({{C_line}}), the three changed ones
  (`upstream_scenario`, `reference_scenario` and the two random tests of
  upstream) by their commands or not at all: `upstream_random_diff` and
  `reference_random_diff` were not re-run after the changes to
  `random_diff.py`.

## Running it

```
pixi run build
pixi run test          # registers, for this comparison:
```

| Test | What | Time |
| --- | --- | --- |
| `band_bounds` | core against upstream's zone tests, 6.8 million points | {{C_t_band}} |
| `roslean_scenario` | the Lean node through all scenarios and probes, differences stated per check | {{C_t_scen}} |
| `roslean_fault_injection` | 19 fault cases, upstream beside the node | {{C_t_fault}} |
| `roslean_fault_clock_negative` | **expected to fail** (defect 1); passes once the node survives a negative `/clock`, then remove `WILL_FAIL` | {{C_t_clock}} |
| `upstream_probes` | the 18 probes on upstream's binary | {{C_t_probes}} |
| `roslean_random_diff` | seeds 0 to 5 of the random comparison | {{C_t_rand}} |
| `t2_point_as_upstream` | macOS only, **expected to fail** (defect 4): the shim's point for one reading against upstream's value | under 1 s |

Those that use topics have `ROS_DOMAIN_ID` 91, 97, 98, 92 and 93. Together
they add about nine minutes to `pixi run test`.

```
pixi run cm-diff               # 600 random cases beside upstream; cm-diff 0 50 for fewer
pixi run cm-faults             # all fault cases, the very large scans included
pixi run bench-cm              # latency in process and through the nodes, and the soak
```

Single scripts, from `ros/roslean_collision_monitor/test` with the workspace
sourced: see the table at the top. `scenario.py --profile profile_roslean
--own` runs only the Lean node's own scenarios and the probes;
`random_diff.py … --roslean --yaw` turns the scan frame.
