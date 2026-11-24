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
  the unsafe direction was found: in 65,056 commands of 720 random cases,
  run on each of two platforms, and in 34 million point tests against the
  core on each, upstream never stopped or slowed where the Lean node did
  not. The zones grow as stated and no more: the farthest point outside
  upstream's zone that the core counted was 3.409 mm for a circle (bound
  3.5) and 4.167 mm for a polygon (bound 4.4).
- *Defects, open.* Two. A `/clock` message with a negative time aborts the
  node (as it aborts upstream). And the node's command path is slow for
  polygon zones: about 200 ns per reading and zone in a hot loop, 50 to 70
  times upstream's, and about four times that again at the rate a node is
  really called. With a 1080-reading scan and two polygons a command takes
  0.4 ms back to back and 1.2 to 1.7 ms when commands come 20 ms apart;
  through the nodes on Linux the Lean node's median latency is 2.4 ms
  where upstream's is 0.4 ms.
- *Defects, fixed since the first draft of this document.* The shim was
  compiled without optimisation, and on macOS it then computed 2.6 % of the
  scan points one float32 step away from upstream's. The package now builds
  optimised by default (review CM3/CM4, finding 5); both were re-measured
  here and are gone.
- *Memory.* Resident memory of the node over 32 minutes of load
  (1.15 million messages sent to it): @@MEM_SHORT@@
- *The two documents.* SPEC and UPSTREAM_BEHAVIOUR.md contradict each other
  in one place (how often a timeout is converted); the binary sides with the
  SPEC. Every statement that only the SPEC makes and that could be put to
  the binary was confirmed (18 new scenarios), by both versions of upstream
  that were run.

Everything was run on 2026-11-24, on two platforms, with the Lean node as
on the branch `collision-monitor` at commit `b503265` (Lean 4.37.0-rc1):
after the fixes of the review CM3/CM4 and after the core's model was
extended for transforms and the lifecycle (task T05; the node itself, the
C++ shim, is not extended yet and uses the new core through the old calls).
The behavioural runs had been made on the node before that extension
(commit `91c21f9`) first and were all repeated; every count of every run
came out the same, on both platforms. The run times did not: see
[Defects](#defects), 2. Where this document gives a number from the earlier
node it says "before the extension".

| | macOS | Linux |
| --- | --- | --- |
| Machine | Apple M5 Pro, 18 cores, macOS 26.7.2, arm64 | the same machine: Docker Desktop 29.9.3, a Linux virtual machine (kernel 7.0.14-linuxkit, aarch64, 18 cores, 8 GB), container with Ubuntu 24.04.5 |
| ROS 2 Jazzy | RoboStack through pixi (`pixi.lock`) | the ROS apt repository (`ros:jazzy-ros-base`) |
| `nav2_collision_monitor` | 1.3.12 | 1.3.14 |
| Middleware | Fast DDS 2.14.6 | Fast DDS 2.14.7 |
| Compiler | clang 21.1.8 from pixi | gcc 13.3.0 |
| How | `pixi run build`, then the commands below | `bench/cm_linux/Dockerfile`, `bench/cm_linux/run.sh` |

The Linux runs are a container on a Mac, not a Linux computer: the
architecture is arm64 and the kernel is Docker's. Nothing was run on
x86-64, on a robot computer or on a real-time kernel. Logs of every run
named here are in `bench/results/cm_comparison/`.

## What was run

`NODE` is `roslean_collision_monitor/collision_monitor`, `UPSTREAM` is
`nav2_collision_monitor/collision_monitor`; the scripts are in
`ros/roslean_collision_monitor/test/`. On macOS each line was run as
`ROS_DOMAIN_ID=… pixi run bash -c 'source install/setup.bash && python3 …'`;
on Linux by the step of `bench/cm_linux/run.sh` given in the last column.

| What | Command | macOS | Linux | Step |
| --- | --- | --- | --- | --- |
| Lean node, fixed scenarios and probes | `scenario.py NODE --profile profile_roslean` (ctest `roslean_scenario`) | 77 scenarios, 318 checks, 0 failures; 1 not run (replaced) | the same | `scenarios` |
| Upstream, fixed scenarios | `scenario.py UPSTREAM` | 55 scenarios, 277 checks, 0 failures | the same | `scenarios` |
| Reference node | `scenario.py reference_node.py --no-tf --skip negative-stamp` | 45 scenarios, 238 checks, 0 failures; 9 skipped (transforms) | the same | `scenarios` |
| Upstream, statements that only the SPEC makes | `scenario.py UPSTREAM --profile profile_upstream --own` (ctest `upstream_probes`) | 18 scenarios, 48 checks, 0 failures | the same | `scenarios` |
| Lean node beside upstream, random | `random_diff.py NODE UPSTREAM A B --roslean`, seeds 0 to 599 in six processes of 100 | 0 cases outside the bounds, 54,043 commands; none retried | the same | `random` |
| The same with the scan frame turned | `… --roslean --yaw`, seeds 1000 to 1119 in two processes | 0 cases outside the bounds, 11,013 commands | the same | `random` |
| Can that test fail? | `… 0 3 --roslean --self-test swap`, `… shrink` | violations reported in 3 of 3 cases, and in 1 of 3 | the same | `selftest` |
| Core against upstream's zone tests, point by point | `band_bounds 100000 7` (ctest `band_bounds` with 20,000) | 34.0 million points, 340 zones: both bounds hold | the same | `points` |
| Fault injection, both nodes | `fault_injection.py NODE UPSTREAM --big` (ctest `roslean_fault_injection` without the large scans; `pixi run cm-faults`) | 23 cases, 124 checks, 122 as expected; one case kills the node (2 checks) | the same | `faults` |
| Quality of service and bursts | `probe_qos.py NODE --no-state-topic`, `probe_qos.py UPSTREAM` | as upstream | as upstream | `qos` |
| Cost of a command per reading and zone | `cmd_cost` (ctest `roslean_cmd_cost`, expected to fail) | [below](#defects) | below | `ctest` |
| Per-call latency in process | `run_cm_bench.py core` | [below](#per-call-latency) | below | `bench` |
| Latency through the nodes | `run_cm_bench.py node` | [below](#through-the-nodes) | below | `bench` |
| Memory | `run_cm_bench.py soak --minutes 32`; `bench_cm --which lean --scan slow --soak 100000 --iters 1000000 --warmup 0` | [below](#memory) | below | `soak` |
| The registered tests | `colcon test --base-paths ros --packages-select roslean_collision_monitor` | @@CTEST_MAC@@ | @@CTEST_LINUX@@ | `ctest` |

The registered tests are listed under [Running it](#running-it).

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

New differences, not recorded in the SPEC: **none in behaviour**, on either
platform. What the SPEC does not say is listed under [Defects](#defects):
the abort on a negative `/clock` (the same as upstream, so not a
difference) and the run time of the command path.

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
are answered (of 100 sent back to back, five times: Lean 3 to 5 and
upstream 4 to 10 on macOS, both 10 every time on Linux; of 1000: Lean 20
and 18, upstream 22 and 20 on macOS, 10 for both on Linux).

## Fixed scenarios

`scenario.py` has 55 scenarios with 277 checks whose expected values are
what upstream publishes. The Lean node cannot be given `state_topic` (D13)
and has no transforms (D11), so the harness was extended: a *profile*
(`test/profile_roslean.py`) states, scenario by scenario, each answer that
is expected to differ and the number of the difference, the scenarios the
node must refuse, and what replaces a transform. Everything else must be
exactly as upstream, and a stated difference that does not occur is a
failure as well. With the profile the node passes on both platforms: 77
scenarios (54 of the 55, the 5 of the profile and the 18 probes), 318
checks, 0 failures. The checks that carry a stated difference, by number:
D1 1, D4 17, D5 3, D6 1, D8 1, D9 2, D10 2, D11 7, D13 1.

What the profile may say was read entry by entry for this document. Every
entry names a numbered difference; there is no entry that switches a
scenario off without a replacement (the one scenario not run,
`boundary-vertices`, is replaced by four). Two things were tightened:

- A refusal must be for the stated reason. The 12 scenarios that the
  profile expects to be refused name the parameter, and the harness requires
  the node's message `1 parameter refused: parameter '<name>'`, so a node
  that exits for another reason, or refuses two parameters, fails (12 of
  the 318 checks).
- An entry whose expected answer is upstream's own answer is an error: it
  would be counted as a difference without being one.

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
`smoke_conversions.cpp` on the shim's `encode_scan` (against a copy of
upstream's loop, bit for bit), and statistically by the random test, where
a point less than a micrometre outside a boundary decided 361 answers on
macOS and 363 on Linux.

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

  Bound G is asserted as stated, with no allowance beyond the nanometre
  just mentioned. (The first draft of the test allowed the Lean node a
  further micrometre, because the node was then built without optimisation
  and its points could lie one float32 step from upstream's. That allowance
  is gone.)

Results, seeds 0 to 599 (the generator is deterministic, so both platforms
ran the same cases):

| | macOS | Linux |
| --- | ---: | ---: |
| Cases outside the bounds | 0 | 0 |
| Unsafe divergences (S violated) | 0 | 0 |
| Cases that passed only on the slow retry | 0 | 0 |
| Commands | 54,043 | 54,043 |
| Scans | 32,289 | 32,289 |
| Point-in-zone tests, circles; of them in the growth band | 104,915; 5,551 | 104,915; 5,526 |
| Point-in-zone tests, polygons; of them in the growth band | 113,769; 5,459 | 113,769; 5,466 |
| Commands whose answer depends on a point in a band | 1,590 (2.94 %) | 1,592 (2.95 %) |
| Commands for which the two nodes demand different velocities | 1,259 (2.33 %) | 1,261 (2.33 %) |
| Commands for which the published messages differ | 1,610 | 1,612 |

In every command of the second-to-last row the Lean node is the more
restrictive one: that is bound S, which held for all 54,043 commands on
both platforms. So when a point lies in the band and decides, the Lean node
gives the more restrictive answer in 79 % of the cases, and upstream's
answer in the rest. The last row is larger than the one before because a
difference shifts the Lean node's stop-publication period: it then goes
silent, or resumes zero, at other commands than upstream. The two platforms
differ in a few counts because their C libraries round sine and cosine
differently, which moves points that lie within nanometres of a boundary
(first row of the next table).

In the earlier run of the same 600 cases, on the node before the
extension, one case on macOS passed only on the slow retry (seed 415): at
the first attempt both nodes, at the same command (number 67), gave an
answer that the test's model does not explain; with 0.3 s per event instead
of 0.05 s it passed, as it did at once on Linux and in the run reported
here. Both nodes deviating together at one command is what a message that
arrived late at both looks like (eight such processes and the Linux runs
were going at the same time); it was not investigated further, and bound S
was not violated in that first attempt either.

How far outside upstream's zone the point lay that decided a difference
(the smallest reach that explains the Lean node's answer), macOS; Linux is
the same except for the first row (281 and 82):

| Deciding point outside upstream's zone by | Circle | Polygon |
| --- | ---: | ---: |
| below 1 µm | 279 | 82 |
| 1 µm to 0.25 mm | 154 | 192 |
| 0.25 to 0.5 mm | 93 | 100 |
| 0.5 to 1 mm | 48 | 61 |
| 1 to 1.5 mm | 71 | 56 |
| 1.5 to 2 mm | 10 | 28 |
| 2 to 2.5 mm | 9 | 46 |
| 2.5 to 3 mm | 0 | 25 |
| 3 to 3.5 mm | 0 | 5 |
| beyond 3.5 mm | 0 | 0 |
| all | 664 | 595 |
| farthest | 2.056 mm | 3.106 mm |

With the scan frame turned by π/2, −π/2, 0.3, −2.5, 1.0 or 3.0 rad in half
of the cases (`--yaw`, seeds 1000 to 1119; upstream gets the quaternion, the
Lean node the yaw): 0 cases outside the bounds on either platform, none
retried, 11,013 commands; the nodes demand different velocities for 303
(macOS) and 299 (Linux), the deciding point at most 1.791 mm (circle) and
2.653 mm (polygon) outside.

*Can the test fail?* `--self-test swap` exchanges the two nodes, so that
upstream is judged as if it were a Lean node that misses points: all 3
cases (seeds 0 to 2) are reported, each with "ROSlean demands a less
restrictive velocity than upstream". `--self-test shrink` allows a
tenth of the growth: seed 0 is reported ("an answer that no set of points
within the stated growth of the zones explains": the deciding point lies
1.196 mm outside a polygon and 0.44 mm are allowed); seeds 1 and 2, whose
deciding points lie at most 0.118 mm outside a circle (0.35 mm allowed),
pass. The same on both platforms.

*Defects of this test, found and fixed while it was written* (by the
session that wrote it, on 2026-11-23; recorded from its notes, not
reproduced here). (1) The first version collected the possible answers in a
Python `set`. `(0.0, -0.0, 0.0)`, a command scaled by a ratio of 0, and
`(0.0, 0.0, 0.0)`, a stop, are equal as tuples, so one of them was lost, and
three seeds were reported as outside the bounds (26, 387, 389). The answers
are now told apart by bit pattern. (2) The count of differences was wrong
for commands that one node leaves unanswered. (3) Upstream's points were
computed with `sinf` and `cosf`. Upstream's binary calls `__sincosf_stret`
on macOS, which returns a value one float32 step away for 2.6 % of angles;
seed 1057 of the `--yaw` run was reported for it. The test now calls the
routine upstream's binary calls (`sincosf` with glibc). All four seeds
named here pass in the runs of this document.

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
| The two tests disagree | 32.66 % (Linux 32.64 %) | 36.45 % |
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

The table above and the one below are from macOS; on Linux the program
prints the same farthest and nearest points and 0 violations; the number
of circle points on which the two tests disagree differs by 3,563 of 16.2
million (the cause was not looked for), that of polygon points is equal.

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
ones in `test/probes.py` (expectations: what the binary did; the session
that wrote them notes that all 18 passed at the first run, i.e. every
prediction made from the SPEC was right). Here the 18 were run on upstream
1.3.12 (macOS) and 1.3.14 (Linux): 48 checks, 0 failures on each.

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

`test/fault_injection.py`: 23 cases with 124 checks, each case on fresh
processes, both nodes on the same messages under simulated time (zones: stop circle 0.5 m, slowdown
circle 1.0 m, `min_points` 3, `source_timeout` 0.5 s). The script asserts
the Lean node's answer to every command and that the process is alive at
the end of each case; upstream's observed answers are asserted too. The
table holds for both platforms: the two logs are equal line by line except
for the number of commands answered in a burst.

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
| the same sizes handed to the callback in process (`bench_cm --readings N --zones circles --scan near`, macOS) | handled: 47 ms and 0.48 s per scan, 15 ms and 0.15 s per command; no crash | 2.8 ms and 30 ms per command |
| Stamp of −1 s | stop (stamp taken as 0, D8); next good scan passes | **aborts** at the next command |
| Stamp `sec = -1, nanosec = 2·10⁹`; `sec = -2³¹` | stop | aborts |
| Clock at 0.25 s, scan stamped −5 s | **used as fresh** (D8): stops for an obstacle scan, passes for a clear one, until the clock passes `source_timeout` | aborts |
| Stamp in 2038 | used, and still 100,000 s later with no further scan (R7) | same |
| An older clear scan arriving after a newer obstacle scan | the last received counts: passes (R5) | same |
| Commands with NaN or ±∞ in any of the six components, also with an obstacle in the stop zone | nothing published (R2) | same |
| Commands of ±1.7·10³⁰⁸ and 5·10⁻³²⁴, no obstacle | pass unchanged | same |
| The same in a slowdown zone | halved; the subnormal becomes 0.0 (D6) | passed unslowed |
| 1000 commands back to back, twice | macOS: 20 and 18 answered; Linux: 10 and 10; then normal | macOS: 22 and 20; Linux: 10 and 10 |
| 500 scans back to back | the last counts | same |
| Scan in a foreign frame; empty frame name; base frame name with a blank or a slash; a name of 100,000 characters | stop (D11), also discarding the stored scan | stop (no transform) |
| Clock jumps back 50 s, 100 s, to 0 | an expired scan is used again; a stop period that began "later" publishes zero again (A2) | same |
| **`/clock` message of −5 s** | **aborts** (defect 1) | aborts |

So one input crashes the Lean node, on both platforms (exit by signal 6).
Everything else is handled, and handled as the SPEC says.

## Defects

No case was found in which upstream stops or slows and the Lean node does
not: not in the fixed scenarios, not in 720 random cases on each platform,
not under fault injection, not in the point test.

### Open

Found in the node, the shim or the model (category c). Not fixed here; each
has a registered test that is expected to fail.

1. **A `/clock` message with a negative time ends the process.** With
   `use_sim_time`, a `rosgraph_msgs/Clock` message with `sec < 0` ends the
   node with `terminating due to uncaught exception of type
   std::runtime_error: cannot store a negative time point in rclcpp::Time`
   (signal 6; the node's own log, both platforms). The exception comes from
   rclcpp's time source, which by default runs in a thread of its own, where
   the node cannot catch it (this is rclcpp's design; the thread was not
   inspected here). The shim's `std::max(now().nanoseconds(), 0)` is never
   reached. Upstream dies in the same way with the same message, so this is
   not a difference, but the SPEC's aim is that no input ends the node (D8,
   "Bad data (c)"), and A2 says nothing is assumed about the clock.
   Smallest reproduction: a node started with `-p use_sim_time:=true`, then
   `ros2 topic pub --once /clock rosgraph_msgs/msg/Clock "{clock: {sec: -5}}"`;
   or `fault_injection.py NODE --only clock-negative`, which prints
   `FAIL  the node has exited with code -6` and `2 failure(s) in 3 checks`.
   Registered as `roslean_fault_clock_negative`, which asserts that the
   node survives and is marked `WILL_FAIL`. A fix probably needs the node
   option `use_clock_thread(false)` and a `catch` around the spin, or a
   clock subscription of the node's own; the node cannot repair it inside
   a callback. Severity: low on a robot (a simulator would have to publish
   a negative time), but it is the one way found to end the process from a
   topic.

2. **A command is slow, and since the model was extended it is too slow
   for an ordinary scanner.** Each command places every stored reading in
   the base frame again and then tests every point against every zone, all
   in exact integer arithmetic.

   | Per command, in a hot loop | macOS | Linux (container) | Upstream |
   | --- | ---: | ---: | ---: |
   | 360 readings, two polygons | 1.0 ms | 0.85 ms | 3 to 5 µs |
   | 1080 readings, two polygons | 3.1 ms | 2.5 ms | 8 to 12 µs |
   | 1080 readings, two circles | 2.7 ms | 2.1 ms | 4 to 9 µs |
   | 10,000 readings, two polygons | 29 ms | 24 ms | 64 to 77 µs |

   That is 1.9 to 2.9 µs per reading; scan and command callbacks together
   take 180 to 810 times what upstream's take
   ([Per-call latency](#per-call-latency)). A node is not called in a hot
   loop: with one command every 20 ms the same call at 1080 readings takes
   about 7.5 ms on macOS with either kind of zone (in one of three runs
   12 ms with polygons) and 5.7 ms (circles) or 6.8 ms (polygons) in the
   container (`cmd_cost`).

   Where the time goes. Before the extension (commit `91c21f9`) a scan was
   converted to millimetres once, when it arrived, and a command cost 8 ns
   per reading and zone for circles and 200 ns for four-vertex polygons:
   0.41 ms at 1080 readings with two polygons on macOS, 18 µs with two
   circles. Since the extension `Scan.points` applies the scan's transform
   to every reading at every command (`Xform.pointMm`, exact arithmetic on
   the binary64 patterns), and the node passes the identity transform. The
   cost no longer depends much on the zones: 2.5 µs per reading with two
   circles, 2.9 µs with two polygons on macOS. The zone tests are the same
   code as before; the placement was read in the source and its cost is
   inferred from these figures, it was not profiled.

   What follows from it, measured through the nodes
   ([Through the nodes](#through-the-nodes)): @@NODE_SUMMARY@@

   The answers to a given sequence of events are right, and they are the
   same as before the extension in every run of this document. What is
   wrong is when they come, and which events the node gets to see.
   Nothing warns at start-up.

   Smallest reproduction: `build/roslean_collision_monitor/cmd_cost` (no
   ROS; the two boxes of Nav2's example, 1080 stored readings, 400
   commands). It asserts at most 25 ns per reading and zone back to back
   and prints 1,430 to 1,480 ns on macOS and 1,160 to 1,180 ns in the
   container (185 to 230 ns before the extension). Registered as
   `roslean_cmd_cost`, marked `WILL_FAIL`. The limit is three times what
   the circle path needed before the extension and several times upstream's
   whole cost per reading; it is a line drawn here, not a requirement taken
   from anywhere (a question for the user in the task report). A threshold
   on run time is fragile in principle; here the margin is a factor of 50.

### Fixed since the first draft

The first draft of this document (2026-11-23) listed two more, both about
the build. The review CM3/CM4 found them independently (finding 5) and the
package was changed before this document was finished. Re-measured here:

3. **The shim was compiled without optimisation.** `pixi run build` sets no
   build type, and the node was compiled at `-O0`. Now the package's
   `CMakeLists.txt` selects `Release` when no build type is given
   (`CMAKE_BUILD_TYPE:STRING=Release` in the build tree on both platforms).
   The scan callback at 1080 readings takes 49 µs in process on macOS; the
   first draft measured 165 µs for the unoptimised node. All measurements
   below are of the optimised node.

4. **T2: the shim's points were not always upstream's, to the last bit.**
   Compiled without optimisation the node called `sinf` and `cosf`, upstream's
   binary `__sincosf_stret`, and Apple's library returns a different float32
   from the two for 2.6 % of angles. Now both binaries import the same
   routine: `___sincosf_stret` on macOS, `sincosf` with glibc (`nm -u` on
   the node and on upstream's library, both platforms).
   `test/t2_point_as_upstream.cpp`, which the first draft registered as an
   expected failure, passes and is registered as an ordinary test on macOS
   (for the angle 1.1233999729156494 the shim's cosine is now upstream's
   0.4326196312904358). `smoke_conversions` compares 200 scans with a copy
   of upstream's loop on every platform. And the random comparison no
   longer allows the Lean node the micrometre it was given for this.

Notes that are not defects of the node:

- D8's early-clock case is as the SPEC describes it and was seen: with the
  clock below `source_timeout`, a clear scan with a negative stamp lets
  commands pass. Upstream is dead at that point. REVIEWS, CM1/CM2 finding 9,
  proposes a separate input for a negative `sec`; this run supports it.
- Scans that consist only of NaN or ∞ pass, as upstream (open decision 1).

### Found in the harness and fixed (category d)

By the first session:

- `cm_harness.Side` always passed `state_topic`, which the Lean node
  refuses: now optional per node.
- `scenario.py` could only skip a scenario that publishes a transform; a
  profile now says what replaces it.
- `random_diff.py` could not compare nodes whose zones differ by design; see
  above. Its own first version had the zero-sign defect described there.
- UPSTREAM_BEHAVIOUR.md, finding 16, corrected (conversion of timeouts).
- `bench/roslean_bench` did not run on macOS (`$ORIGIN` in the run path;
  executables linked to the Python generator libraries abort at start).

When this document was finished:

- **A crash of `scenario.py`.** `reference_scenario` once ended with
  `IndexError` at `get_publishers_info_by_topic('cmd_vel_out')[0]`, right
  after `wait_connected` had counted one publisher (report of task T01;
  seen once, under load, not reproduced here). `wait_connected` required
  its conditions once and then spun for half a second, in which the count
  can change: the graph can still show the node of the previous run, or the
  node has died. It now requires the conditions before and after that half
  second, and the two lists of endpoints are read by a function that waits
  for an entry and reports a failed check, not a crash, if none comes. In
  the runs of this document `scenario.py` started more than a thousand
  scenario runs on the two platforms without the error; that does not prove the
  race is gone, only that the crash cannot occur any more.
- A refusal passed whatever its reason; see
  [Fixed scenarios](#fixed-scenarios).
- The micrometre of slack in bound G; see
  [Random cases](#random-cases-lean-node-beside-upstream).
- The two tests that drive the node under fault injection used
  `ROS_DOMAIN_ID` 97 and 98, outside the range the other tests of the
  repository use; now 120 and 121.

## Measurements

In the manner of [BENCHMARK.md](../BENCHMARK.md). Sources:
`bench/roslean_bench/src/bench_cm.cpp`, `cm_load.cpp`,
`scripts/run_cm_bench.py`; raw data in `bench/results/cm_*.json` (macOS)
and `bench/results/cm_comparison/linux/cm_*.json`.

**Environment and its limits.** The two platforms of the table at the top:
an Apple M5 Pro with 18 cores, and a Linux container in Docker's virtual
machine on that same computer. Neither is a robot computer or a real-time
system, and the second is not a Linux computer: its processor is the Mac's,
its scheduler and its C library are Linux's. Another session was working on
the same machine (Lean builds), so the tails (p99.9, max) may contain its
scheduling; the latency runs of this section were made one at
a time, with none of this document's other runs going. The two columns of
every table were measured in the same process or at the same time on the
same messages and can be compared with each other; a macOS table and a
Linux table can not, beyond the order of magnitude.

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
after a warm-up of a tenth, back to back. Microseconds.

macOS:

| Readings | Zones | Call | Lean p50 | p99 | p99.9 | max | Upstream p50 | p99 | p99.9 | max |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 16.1 | 18.6 | 36.6 | 47.2 | 0.0 | 0.5 | 1.9 | 2.1 |
| 360 | polygons | command, clear | 135 | 161 | 169 | 230 | 2.8 | 3.6 | 6.9 | 25.9 |
| 360 | polygons | command, slowdown | 137 | 188 | 198 | 223 | 2.8 | 4.1 | 7.8 | 30.4 |
| 360 | polygons | command, stop | 66.8 | 77.7 | 96.8 | 104 | 2.0 | 2.6 | 6.1 | 21.7 |
| 360 | circles | scan | 16.2 | 18.3 | 35.8 | 43.1 | 0.0 | 0.3 | 0.5 | 0.6 |
| 360 | circles | command, clear | 6.0 | 6.7 | 15.3 | 29.8 | 1.7 | 1.9 | 3.6 | 22.5 |
| 360 | circles | command, slowdown | 6.0 | 6.8 | 13.2 | 39.4 | 1.7 | 2.0 | 5.6 | 86.5 |
| 360 | circles | command, stop | 5.4 | 6.0 | 10.4 | 29.5 | 1.6 | 1.9 | 3.9 | 21.9 |
| 1080 | polygons | scan | 48.9 | 57.5 | 80.2 | 93.3 | 0.0 | 0.5 | 1.8 | 2.8 |
| 1080 | polygons | command, clear | 406 | 443 | 456 | 538 | 7.0 | 8.4 | 19.6 | 73.2 |
| 1080 | polygons | command, slowdown | 408 | 446 | 457 | 475 | 7.0 | 8.2 | 24.7 | 43.7 |
| 1080 | polygons | command, stop | 202 | 231 | 239 | 259 | 5.1 | 6.2 | 10.6 | 26.9 |
| 1080 | circles | scan | 48.3 | 56.1 | 76.8 | 83.3 | 0.0 | 0.9 | 1.9 | 2.2 |
| 1080 | circles | command, clear | 17.6 | 20.1 | 41.8 | 47.2 | 4.0 | 5.0 | 12.9 | 29.2 |
| 1080 | circles | command, slowdown | 18.4 | 20.9 | 41.5 | 58.8 | 4.0 | 5.0 | 9.3 | 26.9 |
| 1080 | circles | command, stop | 14.9 | 17.3 | 34.4 | 50.1 | 3.8 | 4.3 | 6.3 | 28.3 |
| 10000 | polygons | scan | 474 | 517 | 551 | 565 | 0.2 | 0.4 | 1.6 | 11.5 |
| 10000 | polygons | command, clear | 3 878 | 4 058 | 4 127 | 4 235 | 60.2 | 70.2 | 86.5 | 87.6 |
| 10000 | polygons | command, slowdown | 3 879 | 4 063 | 4 137 | 4 173 | 60.4 | 66.0 | 85.5 | 92.6 |
| 10000 | polygons | command, stop | 1 952 | 2 051 | 2 413 | 2 463 | 44.1 | 51.3 | 83.2 | 131 |
| 10000 | circles | scan | 473 | 507 | 544 | 548 | 0.1 | 0.4 | 0.5 | 0.6 |
| 10000 | circles | command, clear | 170 | 196 | 210 | 215 | 32.2 | 35.2 | 55.0 | 117 |
| 10000 | circles | command, slowdown | 175 | 203 | 211 | 215 | 32.2 | 35.2 | 55.1 | 56.3 |
| 10000 | circles | command, stop | 151 | 173 | 187 | 205 | 30.0 | 33.2 | 53.2 | 65.6 |

Linux (container):

| Readings | Zones | Call | Lean p50 | p99 | p99.9 | max | Upstream p50 | p99 | p99.9 | max |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 25.4 | 31.0 | 53.0 | 59.6 | 0.1 | 0.2 | 0.4 | 3.5 |
| 360 | polygons | command, clear | 154 | 184 | 193 | 239 | 3.2 | 4.6 | 8.4 | 29.1 |
| 360 | polygons | command, slowdown | 154 | 184 | 193 | 250 | 3.3 | 4.8 | 10.0 | 41.9 |
| 360 | polygons | command, stop | 77.5 | 93.8 | 111 | 178 | 2.5 | 3.8 | 7.1 | 25.8 |
| 360 | circles | scan | 25.2 | 31.7 | 50.9 | 61.2 | 0.0 | 0.1 | 0.4 | 6.7 |
| 360 | circles | command, clear | 8.7 | 12.2 | 23.8 | 38.1 | 2.0 | 2.8 | 6.1 | 31.4 |
| 360 | circles | command, slowdown | 8.9 | 12.5 | 26.2 | 39.0 | 2.0 | 2.8 | 6.3 | 30.8 |
| 360 | circles | command, stop | 7.5 | 10.7 | 19.6 | 35.6 | 2.0 | 2.6 | 6.1 | 31.6 |
| 1080 | polygons | scan | 81.4 | 100 | 117 | 193 | 0.1 | 0.4 | 0.5 | 77.6 |
| 1080 | polygons | command, clear | 519 | 630 | 663 | 734 | 8.4 | 12.6 | 28.2 | 44.5 |
| 1080 | polygons | command, slowdown | 530 | 632 | 671 | 752 | 8.5 | 12.7 | 29.2 | 54.1 |
| 1080 | polygons | command, stop | 270 | 330 | 341 | 405 | 6.2 | 10.2 | 17.0 | 396 |
| 1080 | circles | scan | 80.4 | 99.2 | 114 | 129 | 0.1 | 0.1 | 0.5 | 5.6 |
| 1080 | circles | command, clear | 26.9 | 33.0 | 55.4 | 65.0 | 5.2 | 9.4 | 15.5 | 45.5 |
| 1080 | circles | command, slowdown | 27.9 | 34.7 | 56.1 | 147 | 5.2 | 9.4 | 18.0 | 44.4 |
| 1080 | circles | command, stop | 24.0 | 29.6 | 50.3 | 57.1 | 4.7 | 8.7 | 13.8 | 45.0 |
| 10000 | polygons | scan | 802 | 858 | 896 | 1 918 | 0.3 | 0.6 | 1.5 | 1.8 |
| 10000 | polygons | command, clear | 5 579 | 5 810 | 6 108 | 6 159 | 98.4 | 128 | 146 | 170 |
| 10000 | polygons | command, slowdown | 5 540 | 5 789 | 5 978 | 6 074 | 100 | 131 | 155 | 157 |
| 10000 | polygons | command, stop | 2 631 | 2 727 | 2 863 | 3 670 | 84.1 | 112 | 135 | 322 |
| 10000 | circles | scan | 789 | 841 | 867 | 967 | 0.1 | 0.4 | 0.7 | 4.0 |
| 10000 | circles | command, clear | 278 | 317 | 338 | 352 | 70.5 | 106 | 120 | 183 |
| 10000 | circles | command, slowdown | 257 | 295 | 307 | 320 | 72.0 | 107 | 119 | 124 |
| 10000 | circles | command, stop | 228 | 280 | 502 | 1 736 | 70.0 | 117 | 247 | 336 |

"clear", "slowdown" and "stop" are the scenes; in the stop scene the first
zone decides and the second is not evaluated, and after `stop_pub_timeout`
nothing is published. Sum of the two medians, clear scene, macOS:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 151 | 2.8 | 54 |
| 360 | circles | 22.2 | 1.7 | 13 |
| 1080 | polygons | 455 | 7.0 | 65 |
| 1080 | circles | 65.9 | 4.0 | 16 |
| 10000 | polygons | 4 352 | 60.3 | 72 |
| 10000 | circles | 643 | 32.3 | 20 |

and Linux:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 179 | 3.3 | 54 |
| 360 | circles | 33.9 | 2.1 | 16 |
| 1080 | polygons | 601 | 8.5 | 71 |
| 1080 | circles | 107 | 5.3 | 20 |
| 10000 | polygons | 6 381 | 98.7 | 65 |
| 10000 | circles | 1 067 | 70.7 | 15 |

Reading:

- **Scan path.** The Lean node spends 45 to 47 ns per reading on macOS (70
  to 80 in the container) when a scan arrives: computing the point
  (`encode_scan`), copying 20 bytes per reading into the core, and
  converting to millimetres. Upstream spends that time when the command
  arrives.
- **Command path, circles.** 16 to 17 ns per reading for the two zones on
  macOS (24 to 28 in the container), against upstream's 3 to 5 ns (5 to 7)
  including its conversion of the scan: 3.5 to 5 times slower, 18 µs at
  1080 readings on macOS.
- **Command path, polygons.** 376 to 388 ns per reading for the two
  four-vertex polygons on macOS (426 to 558 in the container), i.e. about
  200 ns per reading and zone, 50 ns per edge: 0.41 ms at 1080 readings and
  3.9 ms at 10,000 on macOS, where upstream needs 7 µs and 60 µs. This is
  defect 2. The cost is the exact integer test with its band, done for
  every point against every edge; most points of a real scan are metres
  away from every zone and could be excluded by one comparison with a
  bounding box.
- **Tail.** The Lean callbacks are steady: on macOS p99 is within 1.4 times
  the median, and the largest call exceeds its median by less than 0.15 ms
  up to 1080 readings and by at most 0.5 ms at 10,000; in the container the
  largest excess in the table is 1.5 ms (one call of 1.7 ms against a
  median of 0.23 ms). As with twist_mux there is no collector that pauses
  the node.
- **These are hot-loop figures.** Called every 20 ms, as a node is, the
  same command costs about four times as much (defect 2, `cmd_cost`): 1.7 ms
  instead of 0.4 ms at 1080 readings with two polygons on macOS.
- **What it costs.** With 1080 readings at 25 Hz, commands at 50 Hz and two
  polygon zones, from the hot-loop medians on macOS: 25 × 0.05 ms + 50 ×
  0.41 ms = 22 ms of work per second, 2.2 % of a core (upstream: 0.04 %);
  with the 1.7 ms of a paced command, 86 ms per second, 9 % of a core. With
  10,000 readings: 206 ms per second from the hot-loop medians, 21 % of a
  core, and each command blocks the node for 4 ms or more. With circle
  zones: 0.2 % and 2 %.

### Through the nodes

Both node binaries side by side on the same topics (upstream activated
through its lifecycle), wall-clock time, `enable_stamped_cmd_vel`, polygon
zones. `cm_load` publishes scans at 25 Hz and commands at 50 Hz for 20 s;
the scene changes every 2.5 s. Latency from `publish` of a command to the
callback of a subscriber on the node's output. Microseconds.

macOS:

| Readings | Node | Answered / sent | p50 | p90 | p99 | p99.9 | max | Answers that differ |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | lean | 851 / 1000 | 6 316 | 9 745 | 11 318 | 11 493 | 11 493 | 0 of 851 |
| 360 | upstream | 851 / 1000 | 5 930 | 9 387 | 11 317 | 11 393 | 11 393 | 0 of 851 |
| 1080 | lean | 851 / 1000 | 7 136 | 11 204 | 12 428 | 12 929 | 12 929 | 0 of 850 |
| 1080 | upstream | 850 / 1000 | 5 921 | 9 046 | 11 299 | 11 362 | 11 362 | 0 of 850 |
| 10000 | lean | 852 / 1000 | 11 728 | 14 802 | 16 532 | 19 021 | 19 021 | 0 of 852 |
| 10000 | upstream | 852 / 1000 | 6 041 | 9 397 | 10 936 | 11 356 | 11 356 | 0 of 852 |

Linux (container):

| Readings | Node | Answered / sent | p50 | p90 | p99 | p99.9 | max | Answers that differ |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | lean | 851 / 1000 | 985 | 1 190 | 1 449 | 1 670 | 1 670 | 0 of 851 |
| 360 | upstream | 851 / 1000 | 410 | 690 | 937 | 1 439 | 1 439 | 0 of 851 |
| 1080 | lean | 851 / 1000 | 2 418 | 2 636 | 2 889 | 3 159 | 3 159 | 0 of 851 |
| 1080 | upstream | 851 / 1000 | 371 | 559 | 840 | 1 030 | 1 030 | 0 of 851 |
| 10000 | lean | 852 / 1000 | 7 540 | 7 812 | 9 049 | 12 448 | 12 448 | 0 of 850 |
| 10000 | upstream | 850 / 1000 | 312 | 362 | 423 | 1 110 | 1 110 | 0 of 850 |

About 850 of 1000 commands are answered by either node: the rest fall into
the silence after `stop_pub_timeout` in the stop scene. The two nodes gave
the same answer to every command that both answered, on both platforms.

On macOS the latencies are mostly those of the middleware: upstream's
median is 6 ms and its maximum 11 ms at every size, as if messages were
picked up on a 10 ms tick, and the Lean node's extra work shows as 0.4 ms
of the median at 360 readings, 1.2 ms at 1080 and 5.7 ms at 10,000.

In the Linux container the middleware takes 0.3 to 0.4 ms, and the node's
own time is what is left: the Lean node's median is 0.6 ms above upstream's
at 360 readings, 2.0 ms at 1080 and 7.2 ms at 10,000. Its p99 stays within
1.5 times its median (1.4, 2.9 and 9.0 ms), so it is slow and steady, not
erratic. These differences are larger than the sum of the hot-loop medians
of the table before (0.18, 0.60 and 6.4 ms); the paced measurement of
`cmd_cost` accounts for it at 1080 readings (1.2 to 1.7 ms for the command
alone; the other sizes were not measured that way). For a robot this is the number to look at: with a
1080-reading scanner and two polygon zones the Lean node delays every
command by about 2 ms more than upstream does, on this processor.

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

- **Linux on its own hardware, x86-64.** The Linux runs are a container in
  Docker's virtual machine on an Apple processor. Behaviour is covered by
  them (upstream 1.3.14 from the ROS apt repository, glibc, gcc); run times
  and memory on a Linux computer are not.
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
  reach either node, on either platform. In process the core took 10
  million readings (macOS).
- **The wall-clock assumption of the harness**: one event at a time, 50 ms
  (random) or 200 ms (scenarios) apart. Nothing was done about the order of
  a scan and a command that arrive together, in either node.
- **Long runs.** 32 minutes; not hours or days.
- **Other middlewares, a real-time kernel, a robot computer, a robot.**
- **The cost of a paced command at sizes other than 1080 readings**, and
  why a paced command costs four times a hot one (caches and the
  processor's idle states are the likely cause; not examined).
- **`pixi run test` as a whole** was not run for this document (the rule of
  the task: the orchestrator runs it once at the end). What was run instead
  is in the last line of the table at the top and under
  [Running it](#running-it).

## Running it

```
pixi run build
pixi run test-quick    # without the tests labelled long
pixi run test          # everything
```

The tests of `roslean_collision_monitor` that belong to this comparison,
with the time each took in `colcon test` on macOS and in the Linux
container:

| Test | What | macOS | Linux | Label |
| --- | --- | ---: | ---: | --- |
| `band_bounds` | core against upstream's zone tests, 6.8 million points | @@T_band@@ | |
| `roslean_cmd_cost` | **expected to fail** (defect 2): a command costs at most 25 ns per stored reading and zone | @@T_cost@@ | |
| `t2_point_as_upstream` | macOS only: the shim's point for one reading against upstream's bits | @@T_t2@@ | |
| `roslean_fault_clock_negative` | **expected to fail** (defect 1); passes once the node survives a negative `/clock`, then remove `WILL_FAIL` | @@T_clock@@ | |
| `upstream_probes` | the 18 probes on upstream's binary | @@T_probes@@ | |
| `roslean_scenario` | the Lean node through all scenarios and probes, differences stated per check | @@T_scen@@ | long |
| `roslean_fault_injection` | 19 fault cases, upstream beside the node | @@T_fault@@ | long |
| `roslean_random_diff` | seeds 0 to 5 of the random comparison | @@T_rand@@ | long |

Those that use topics have `ROS_DOMAIN_ID` 121, 92, 91, 120 and 93. A test
marked `WILL_FAIL` passes while the defect is there; `WILL_FAIL` cannot
tell why a test fails, so after a change to the harness run the two
programs by hand once and look for the lines quoted under
[Defects](#defects).

@@T_total@@

```
pixi run cm-diff               # 600 random cases beside upstream, one process; cm-diff 0 50 for fewer
pixi run cm-faults             # all fault cases, the very large scans included
pixi run bench-cm              # latency in process and through the nodes, and the soak
pixi run cm-linux-image        # the Linux image; then bench/cm_linux/run.sh OUT_DIR STEP...
```

`bench/cm_linux/run.sh` names its steps (`versions`, `scenarios`,
`selftest`, `random`, `points`, `faults`, `qos`, `bench`, `soak`, `ctest`);
each is one container. Several runs of `random_diff.py --roslean --stats
FILE` are added up by `test/diff_stats.py`.

Single scripts, from `ros/roslean_collision_monitor/test` with the workspace
sourced: see the table at the top. `scenario.py --profile profile_roslean
--own` runs only the Lean node's own scenarios and the probes;
`random_diff.py … --roslean --yaw` turns the scan frame.
