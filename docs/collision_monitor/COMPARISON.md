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
- *Defects, fixed by task T03 (2026-11-24, after this document's runs).*
  Both of the two this document reported as open: the node no longer ends
  on a `/clock` message with a negative time (SPEC D15; it answers with
  zero until the clock is usable again), and a command with 1080 stored
  readings costs 25.2 to 27.5 µs with two circle zones and 41.9 to 46.1 µs with two
  polygon zones in a hot loop on macOS, where it cost 208 µs and 640 µs
  after the transform moved to C++ (T05b) and 2.7 ms and 3.1 ms when this
  document's runs were made; the scan callback costs 15 µs where it cost
  240 µs. See [Defects](#defects) and
  [Cost of a command](#cost-of-a-command). The text of this item as it
  stood, for the record: *Two. A `/clock` message with a negative time
  aborts the node (as it aborts upstream). And a command is slow.* On the node before
  the model was extended for transforms (commit `91c21f9`): 0.4 ms for a
  1080-reading scan and two polygon zones, 50 to 70 times upstream. On the
  node as merged now, which applies an exact transform in Lean to every
  reading at every command (T05's option B): 3 ms in a hot loop and 6 to
  8 ms at the rate a node is really called; through the nodes in the Linux
  container its median latency is 5.8 ms where upstream's is 0.26 ms, and
  with 10,000 readings it answers 532 of 1000 commands on macOS and 678 in
  the container where upstream answers 850. **Option B is being replaced** (the user's decision: the
  transform moves to C++, task T05b), so the second set of figures
  describes code on its way out; the timing comparison must be repeated
  after T05b and T06.
- *Defects, fixed since the first draft of this document.* The shim was
  compiled without optimisation, and on macOS it then computed 2.6 % of the
  scan points one float32 step away from upstream's. The package now builds
  optimised by default (review CM3/CM4, finding 5); both were re-measured
  here and are gone.
- *Memory.* Resident memory of the node over 32 minutes of load
  (1.15 million messages sent to it): after the first ten seconds it rises
  by 48 kB on macOS (to 26.1 MB) and by 84 kB in the Linux container (to
  77.1 MB). It does not grow in any way that matters in that time.
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
node it says "before the extension". After these runs the user decided that
the transform will be applied in C++ and not in Lean (option A; task T05b
reworks the model). Behaviour is expected to stay as measured here, and
must be re-checked; every cost and latency figure of the merged node is
labelled "option B, being replaced" and has to be measured again after T05b
and T06.

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
| Cost of a command | `cmd_cost` (ctest `roslean_cmd_cost`; since T03 it asserts a budget and passes) | [below](#cost-of-a-command) | not repeated after T03 | `ctest` |
| Per-call latency in process | `run_cm_bench.py core` | [below](#per-call-latency) | below | `bench` |
| Latency through the nodes | `run_cm_bench.py node` | [below](#through-the-nodes) | below | `bench` |
| Memory | `run_cm_bench.py soak --minutes 32`; `bench_cm --which lean --scan slow --soak 100000 --iters 1000000 --warmup 0` | [below](#memory) | below | `soak` |
| The registered tests | `colcon test --base-paths ros --packages-select roslean_collision_monitor` | 15 of 15 passed, 22 min 11 s | 14 of 14 passed, 21 min 45 s | `ctest` |

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
| the same sizes handed to the callback in process (`bench_cm --readings N --zones circles --scan near`, macOS) | handled without a crash: 0.23 s per scan and 2.6 s per command at 1,000,000 readings; 2.6 s per scan at 10,000,000, where the command could not be timed (the scan is older than `source_timeout` by the time the scan callback returns). Before the extension: 47 ms and 0.48 s per scan, 15 ms and 0.15 s per command | 3.0 ms and 34 ms per command (`--which upstream`) |
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
| **`/clock` message of −5 s** | when this document's runs were made: **aborts** (defect 1). Since T03: zero velocity for every command until a usable `/clock` message arrives, then as before (D15) | aborts |

When these runs were made one input crashed the Lean node, on both
platforms (exit by signal 6). Since task T03 none does: the case
`clock-negative` (a `/clock` of −5 s, of −1 ns short of 0, of −2³¹ s, each
followed by a usable one) passes with upstream beside the node and gone
after the first message; all 23 cases, 129 checks, 0 failures on macOS and
in the Linux container (`bench/results/cm_comparison/t03/fault_injection.log`,
`…/t03/linux/fault_injection.log`). Everything else
is handled, and handled as the SPEC says.

## Defects

No case was found in which upstream stops or slows and the Lean node does
not: not in the fixed scenarios, not in 720 random cases on each platform,
not under fault injection, not in the point test.

### Open

None known. The two that this document reported as open were fixed by task
T03; they follow.

### Fixed by task T03

Found in the node, the shim or the model (category c). Each had a registered
test that was expected to fail; both tests are ordinary tests now.

1. **A `/clock` message with a negative time ended the process.** With
   `use_sim_time`, a `rosgraph_msgs/Clock` message with `sec < 0` ended the
   node with `terminating due to uncaught exception of type
   std::runtime_error: cannot store a negative time point in rclcpp::Time`
   (signal 6, both platforms). Upstream dies in the same way. The exception
   comes from rclcpp's time source, which by default runs in a thread of its
   own, where the node cannot catch it.

   *The fix* (SPEC D15, T9). The node is created with
   `use_clock_thread(false)`, so the `/clock` subscription is served by the
   executor that spins the node; the exception leaves `spin()`, `main`
   catches it and spins again (the message has been taken; the next one is
   handled normally). Under simulated time the node also looks at each
   `/clock` message itself: while the one received last has `sec < 0` it
   has no valid clock reading, and it hands every command to the core
   through a call of its own, `roslean_cm_on_cmd_no_clock`, which has no
   time argument. In the model that is the transition `stepNoClock`: in the
   active phase a finite command is answered with the stop for an invalid
   source, whatever the configuration, the stored scan and the zones; the
   zero is not withheld after `stop_pub_timeout`. With the next usable
   `/clock` message commands are evaluated as before. Proved
   (`ROSlean/CollisionMonitor/NoClock.lean`): the answer is that stop
   (`noClock_velocity`), nothing but `prevZero` and `prevReport` changes
   (`noClock_state`), no later decision changes
   (`evaluate_after_noClock`), and the first command with a clock again has
   its velocity published (`velocity_after_noClock`). Trusted: when the
   node has no clock reading (the sign of `sec`, in C++).

   *Difference from upstream:* D15. Upstream ends; the node stops the robot
   and goes on.

   *Test:* `roslean_fault_clock_negative` (`fault_injection.py NODE UPSTREAM
   --only clock-negative`), no longer `WILL_FAIL`: 8 checks, the node
   answers zero after each of three negative messages and passes commands
   again after each usable one; upstream is gone after the first.
   `test_cm_ffi` checks the call on the core alone (also with
   `source_timeout: 0` and with the source disabled, where a missing scan
   does not stop the robot but a missing clock does).

   *Side effect to know:* without rclcpp's clock thread the simulated time
   advances only between callbacks of the node, not during one.

2. **A command was slow.** Each command tests every stored reading against
   every zone in exact integer arithmetic. When this document's runs were
   made the model also applied an exact transform to every reading at every
   command (T05's option B: 2.7 ms and 3.1 ms per command at 1080 readings;
   the table that stood here is under
   [Per-call latency](#per-call-latency)). The transform has since moved to
   C++ (T05b), and the model no longer has `Xform.pointMm`, by which the
   first version of this item explained the cost. What remained after T05b,
   measured by `cmd_cost` on macOS at 1080 readings: 208 µs with two
   circles and 640 µs with two 4-vertex polygons. Where that time went:
   about 135 µs in the conversion of the coordinates to millimetres and
   50 µs in the three lists the points passed through (T05b's experiment:
   the conversion replaced by a constant); the rest, 18 µs for circles and
   about 430 µs for polygons, in the zone tests. Why, read in the source
   and not profiled part by part: each coordinate was a `Nat` pattern, and
   a pattern with the sign bit set is a big number in Lean's runtime;
   `evaluate` and `shapesAfter` each asked for the points; and the polygon
   test built the list of a polygon's edges anew, twice, for every point.
   That these were the causes is supported by what removing them gained
   (below), not by a profile.

   *The fix.* `core/ROSleanCore/CollisionMonitorFast.lean` is a second way
   to compute the same function, and the C interface calls it. Each part is
   proved equal to the definition it replaces
   (`ROSlean/CollisionMonitor/Fast.lean`; THEOREMS.md, "The command path
   that C calls"): a coordinate's eight bytes are read into a machine word
   and converted there, sign and magnitude apart (`coordMm64_eq`,
   `word64At_toNat`); a scan call keeps the scan's bytes and no longer
   decodes the coordinates into a list when the scan arrives; the points
   are produced once, in one pass, into an array (`pointsBy_toList`); a
   polygon's edges are computed once per zone and command, and the band
   test of D4 is made only for points within 2 mm of the polygon's bounding
   box (`nearBoundary_near`, `containsEdges_eq`); and the whole is `step`
   (`stepCmdFast_eq`), so that after any sequence of calls the exported
   functions have produced the monitor the theorems speak about
   (`execFast_eq`). No decision changed: besides the proof, `test_cm_ffi`
   runs the old and the new command call side by side on 9,600 commands
   (none differs), and the random comparison with upstream was repeated on
   300 seeds with the same counts for every seed
   ([Cost of a command](#cost-of-a-command)).

   *Test:* `roslean_cmd_cost`, no longer `WILL_FAIL`. It asserts the budget
   below in place of T02's line of 25 ns per reading and zone.

   *Not done:* the crossing count of R10 is still made for every point.
   Skipping it for points beside the bounding box needs a proof that a
   closed polygon is crossed an even number of times, which is not there.
   And each command still builds the points anew from the bytes (one small
   object per valid reading); that is what a command with coordinates of
   its own, as SPEC_NAV2 has it, needs anyway.

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

**Read this first.** Every cost and latency figure of this section that is
not marked "before the extension" was measured on the node as merged at
`b503265`, that is with T05's exact transform in Lean (option B). The user
has since decided that the transform is applied in C++ (option A); the
model is being reworked (task T05b) and the node follows (T06). So these
figures describe code that is being replaced. They are kept because they
are what was measured and because they are the reason to look at the cost
again; **the timing comparison must be repeated after T05b and T06.** The
figures "before the extension" (commit `91c21f9`) are given alongside.

In the manner of [BENCHMARK.md](../BENCHMARK.md). Sources:
`bench/roslean_bench/src/bench_cm.cpp`, `cm_load.cpp`,
`scripts/run_cm_bench.py`; raw data in `bench/results/cm_*.json` (macOS),
`bench/results/cm_comparison/linux/cm_*.json`, and for the node before the
extension in `bench/results/cm_comparison/before_t05/`.

**Environment and its limits.** The two platforms of the table at the top:
an Apple M5 Pro with 18 cores, and a Linux container in Docker's virtual
machine on that same computer. Neither is a robot computer or a real-time
system, and the second is not a Linux computer: its processor is the Mac's,
its scheduler and its C library are Linux's. Another session was working on
the same machine (Lean builds), and some runs of this section overlapped
with each other (noted where it matters), so the tails (p99.9, max) contain
scheduling that is not the node's. The two columns of every table were
measured in the same process or at the same time on the same messages and
can be compared with each other; a macOS table and a Linux table can not,
beyond the order of magnitude.

Upstream converts a scan to points when a command arrives (R3). The Lean
node computes the points in the scan's frame when the scan arrives and
places them in the base frame, in millimetres, when a command arrives
(since the extension of the model; before it, both were done when the scan
arrived). So the two paths must be read together.

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

macOS (option B, being replaced):

| Readings | Zones | Call | Lean p50 | p99 | p99.9 | max | Upstream p50 | p99 | p99.9 | max |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 74.9 | 86.6 | 105 | 164 | 0.0 | 0.2 | 0.5 | 1.6 |
| 360 | polygons | command, clear | 999 | 1 078 | 1 142 | 1 232 | 2.9 | 4.5 | 9.0 | 23.6 |
| 360 | polygons | command, slowdown | 1 010 | 1 095 | 1 304 | 2 086 | 3.0 | 4.6 | 13.8 | 27.3 |
| 360 | polygons | command, stop | 954 | 1 035 | 1 116 | 1 387 | 2.1 | 3.2 | 7.0 | 22.0 |
| 360 | circles | scan | 78.0 | 95.4 | 110 | 159 | 0.0 | 0.2 | 1.0 | 2.5 |
| 360 | circles | command, clear | 887 | 964 | 1 207 | 1 674 | 1.9 | 3.1 | 11.9 | 56.8 |
| 360 | circles | command, slowdown | 892 | 998 | 1 293 | 2 403 | 1.9 | 5.4 | 19.1 | 66.6 |
| 360 | circles | command, stop | 905 | 1 033 | 1 192 | 2 456 | 1.5 | 3.1 | 11.7 | 90.6 |
| 1080 | polygons | scan | 236 | 271 | 390 | 704 | 0.1 | 1.0 | 2.2 | 32.5 |
| 1080 | polygons | command, clear | 3 100 | 3 532 | 4 521 | 5 804 | 7.6 | 18.3 | 29.7 | 395 |
| 1080 | polygons | command, slowdown | 3 085 | 3 287 | 3 470 | 4 869 | 7.6 | 13.0 | 23.5 | 69.3 |
| 1080 | polygons | command, stop | 2 845 | 3 029 | 3 306 | 3 650 | 5.5 | 9.2 | 13.4 | 23.8 |
| 1080 | circles | scan | 235 | 264 | 275 | 552 | 0.0 | 0.4 | 1.1 | 2.9 |
| 1080 | circles | command, clear | 2 652 | 2 816 | 2 960 | 3 821 | 4.4 | 8.0 | 16.9 | 61.3 |
| 1080 | circles | command, slowdown | 2 659 | 2 933 | 3 623 | 3 996 | 4.4 | 6.7 | 18.6 | 30.1 |
| 1080 | circles | command, stop | 2 724 | 2 859 | 3 185 | 3 374 | 3.9 | 6.3 | 11.0 | 20.5 |
| 10000 | polygons | scan | 2 243 | 2 583 | 3 267 | 3 500 | 0.5 | 2.9 | 11.1 | 22.6 |
| 10000 | polygons | command, clear | 28 927 | 31 722 | 36 700 | 39 434 | 64.0 | 81.7 | 130 | 191 |
| 10000 | polygons | command, slowdown | 29 028 | 30 662 | 34 775 | 38 050 | 62.1 | 76.9 | 87.8 | 150 |
| 10000 | polygons | command, stop | 27 228 | 28 771 | 30 251 | 35 385 | 46.1 | 59.7 | 86.4 | 133 |
| 10000 | circles | scan | 2 272 | 2 921 | 3 438 | 3 574 | 0.3 | 2.8 | 4.9 | 20.6 |
| 10000 | circles | command, clear | 25 653 | 29 470 | 33 540 | 38 795 | 34.2 | 52.4 | 92.1 | 138 |
| 10000 | circles | command, slowdown | 25 826 | 28 025 | 29 538 | 32 893 | 35.0 | 91.2 | 101 | 131 |
| 10000 | circles | command, stop | 25 727 | 28 331 | 32 590 | 33 185 | 32.3 | 48.8 | 64.2 | 94.2 |

Linux (container, option B; the macOS runs of this section were going on at
the same time):

| Readings | Zones | Call | Lean p50 | p99 | p99.9 | max | Upstream p50 | p99 | p99.9 | max |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 65.1 | 73.7 | 92.1 | 378 | 0.1 | 0.4 | 0.9 | 5.0 |
| 360 | polygons | command, clear | 850 | 883 | 910 | 1 889 | 5.0 | 9.0 | 13.5 | 47.1 |
| 360 | polygons | command, slowdown | 841 | 870 | 889 | 1 042 | 5.0 | 8.4 | 13.7 | 33.0 |
| 360 | polygons | command, stop | 763 | 788 | 802 | 850 | 4.0 | 6.5 | 9.0 | 27.5 |
| 360 | circles | scan | 65.3 | 72.3 | 87.4 | 173 | 0.1 | 0.3 | 0.6 | 1.4 |
| 360 | circles | command, clear | 688 | 714 | 729 | 1 729 | 3.9 | 6.7 | 10.4 | 24.0 |
| 360 | circles | command, slowdown | 683 | 713 | 757 | 901 | 3.8 | 7.1 | 11.0 | 24.1 |
| 360 | circles | command, stop | 689 | 715 | 732 | 805 | 3.4 | 6.2 | 8.5 | 23.2 |
| 1080 | polygons | scan | 190 | 211 | 249 | 612 | 0.1 | 0.5 | 1.5 | 15.5 |
| 1080 | polygons | command, clear | 2 519 | 2 607 | 2 987 | 3 790 | 12.0 | 17.2 | 27.2 | 41.6 |
| 1080 | polygons | command, slowdown | 2 519 | 2 709 | 3 265 | 3 748 | 12.5 | 18.8 | 31.2 | 46.7 |
| 1080 | polygons | command, stop | 2 325 | 2 521 | 3 019 | 3 647 | 10.3 | 16.2 | 28.2 | 59.9 |
| 1080 | circles | scan | 195 | 225 | 319 | 482 | 0.1 | 0.5 | 1.2 | 3.8 |
| 1080 | circles | command, clear | 2 053 | 2 268 | 2 844 | 3 122 | 9.4 | 15.6 | 22.5 | 60.1 |
| 1080 | circles | command, slowdown | 2 057 | 2 358 | 2 869 | 4 394 | 9.2 | 15.7 | 31.0 | 103 |
| 1080 | circles | command, stop | 2 042 | 2 358 | 2 978 | 3 704 | 8.8 | 19.0 | 38.6 | 116 |
| 10000 | polygons | scan | 1 888 | 2 447 | 2 659 | 2 955 | 0.5 | 2.9 | 7.0 | 11.8 |
| 10000 | polygons | command, clear | 24 159 | 27 397 | 30 684 | 30 893 | 76.6 | 200 | 217 | 244 |
| 10000 | polygons | command, slowdown | 24 282 | 26 226 | 27 443 | 28 261 | 75.1 | 188 | 217 | 229 |
| 10000 | polygons | command, stop | 21 324 | 23 116 | 25 733 | 27 303 | 58.9 | 96.8 | 134 | 146 |
| 10000 | circles | scan | 1 887 | 2 179 | 2 887 | 3 129 | 0.4 | 2.8 | 5.8 | 7.6 |
| 10000 | circles | command, clear | 18 777 | 20 678 | 22 832 | 24 879 | 46.1 | 78.0 | 113 | 144 |
| 10000 | circles | command, slowdown | 18 808 | 19 983 | 23 241 | 25 980 | 47.0 | 79.7 | 110 | 172 |
| 10000 | circles | command, stop | 18 554 | 19 526 | 22 748 | 23 026 | 43.6 | 79.8 | 101 | 136 |

"clear", "slowdown" and "stop" are the scenes; in the stop scene the first
zone decides, and after `stop_pub_timeout` nothing is published. Sum of the
two medians, clear scene, macOS (option B):

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 1 074 | 2.9 | 368 |
| 360 | circles | 965 | 1.9 | 503 |
| 1080 | polygons | 3 336 | 7.7 | 435 |
| 1080 | circles | 2 887 | 4.4 | 654 |
| 10000 | polygons | 31 170 | 64.5 | 483 |
| 10000 | circles | 27 925 | 34.5 | 809 |

Linux:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 915 | 5.0 | 182 |
| 360 | circles | 753 | 4.0 | 188 |
| 1080 | polygons | 2 709 | 12.1 | 223 |
| 1080 | circles | 2 248 | 9.5 | 238 |
| 10000 | polygons | 26 046 | 77.0 | 338 |
| 10000 | circles | 20 665 | 46.5 | 444 |

The same sums for the node before the extension of the model (commit
`91c21f9`, measured the same way earlier on the same day), macOS:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 151 | 2.8 | 54 |
| 360 | circles | 22.2 | 1.7 | 13 |
| 1080 | polygons | 455 | 7.0 | 65 |
| 1080 | circles | 65.9 | 4.0 | 16 |
| 10000 | polygons | 4 352 | 60.3 | 72 |
| 10000 | circles | 643 | 32.3 | 20 |

Linux:

| Readings | Zones | Lean, scan + command | Upstream, scan + command | Ratio |
| ---: | --- | ---: | ---: | ---: |
| 360 | polygons | 179 | 3.3 | 54 |
| 360 | circles | 33.9 | 2.1 | 16 |
| 1080 | polygons | 601 | 8.5 | 71 |
| 1080 | circles | 107 | 5.3 | 20 |
| 10000 | polygons | 6 381 | 98.7 | 65 |
| 10000 | circles | 1 067 | 70.7 | 15 |

Reading (option B, being replaced, unless it says "before"):

- **Scan path.** The Lean node spends 210 to 230 ns per reading on macOS
  (175 to 190 in the container) when a scan arrives: computing the point
  (`encode_scan`), copying 20 bytes per reading into the core, and storing
  the coordinates. Before the extension it was 45 to 47 ns (70 to 80).
- **Command path.** 2.5 to 2.9 µs per reading on macOS (1.9 to 2.4 in the
  container), with little difference between circles and polygons: 2.7 ms
  and 3.1 ms at 1080 readings on macOS, 26 ms and 29 ms at 10,000. Upstream
  needs 4 to 8 ns per reading (5 to 14) for the scan and the command
  together. This is defect 2. Before the extension the command path cost
  16 to 17 ns per reading with two circles and 376 to 388 ns with two
  polygons on macOS; the extension made a command 7 to 8 times slower with
  polygon zones and 150 times slower with circle zones.
- **Tail.** The Lean callbacks are steady in relative terms: p99 is within
  1.3 times the median in every row of both tables; the largest single
  call is at most 2.7 times the median for a command and up to 6 times for
  the short scan callback. There is no collector that pauses the node; a
  call is long because it computes for long.
- **These are hot-loop figures.** Called every 20 ms, as a node is, the
  same command costs two to three times as much (defect 2, `cmd_cost`):
  about 7.5 ms instead of 2.7 to 3.2 ms at 1080 readings on macOS.
- **What it costs.** With 1080 readings at 25 Hz, commands at 50 Hz and two
  polygon zones, from the hot-loop medians on macOS: 25 × 0.24 ms + 50 ×
  3.1 ms = 161 ms of work per second, 16 % of a core (upstream: 0.04 %);
  with the 7.5 ms of a paced command, 38 % of a core. With 10,000 readings
  a command takes 29 ms, longer than the 20 ms between two commands: the
  node cannot answer every command at 50 Hz, whatever the zones.

### Cost of a command

Added by task T03; everything in this section was measured on 2026-11-24 on
the two platforms of this document, on the branch
`claude/cm-comparison-defects` (`cmd_cost`, the runs through the nodes and
everything on Linux at commit `1c68b38`; `bench_cm` on macOS at `771c053`,
which has the same code), with no other test of this task running; another
session was working on the same machine. The Linux image was built from the
branch (`roslean-cm-linux-t03`). Logs: `bench/results/cm_comparison/t03/`.

**The core alone**, `build/roslean_collision_monitor/cmd_cost`: the scan is
stored, then 400 commands are timed back to back and the median is taken,
three times, and the lowest of the three counts; nothing is near, so every
zone is evaluated in full. Microseconds per command; "now" is the lowest
and the highest of five runs of the program. (The columns before are
single medians of the program as it was, on macOS.)

| Stored readings | Zones | Before T05 (`91c21f9`, T02) | After T05b (`55dcc45`) | Now, macOS | Now, Linux | Budget |
| ---: | --- | ---: | ---: | ---: | ---: | ---: |
| 360 | two circles | | | 9.1 to 14.1 | 10.9 to 13.8 | 50 |
| 360 | two 4-vertex polygons | | | 15.2 to 22.4 | 17.2 to 21.1 | 50 |
| 1080 | two circles | 18 | 208 | 25.2 to 27.5 | 29.5 to 32.9 | 150 |
| 1080 | two 4-vertex polygons | 410 | 640 | 41.9 to 46.1 | 46.0 to 49.8 | 150 |
| 10,000 | two circles | | | 242 to 257 | 275 to 293 | 1500 |
| 10,000 | two 4-vertex polygons | | | 396 to 424 | 436 to 458 | 1500 |

Empty cells were not measured. A single median of 400 short commands
varies with what the machine does in those 10 to 20 ms (10 to 20 µs at 360
readings in four single runs on macOS), which is why the program takes the
lowest of three. With one command every 20 ms, as a node gets them, the
same call at 1080 readings took 65 to 136 µs (circles) and 63 to 208 µs
(polygons) on macOS and 158 to 173 µs and 232 to 253 µs in the container: a
thread that wakes from sleep runs up to five times slower than in a hot
loop, and how much slower differs from run to run (T05b measured 864 µs
and 2.5 ms on the code before, four times its hot figures).

**The budget** proposed for a command, back to back, with two zones of
either kind: 50 µs at 360 stored readings, 150 µs at 1080, 1.5 ms at
10,000, that is about 140 ns per reading. It is met with a factor of two
or more to spare on both platforms, and `roslean_cmd_cost` asserts it. It
is a line drawn here: 1080 readings at 150 µs are 0.75 % of the 20 ms
between two commands at 50 Hz. It is not upstream's speed: see the next
tables. A scan with which a command is estimated to exceed 1 ms makes the
node warn once.

**The two callbacks, with upstream's beside** (`run_cm_bench.py core`,
that is `bench_cm`: the Lean node's callback bodies and upstream's own
library in one process, 20,000 calls per row, 5,000 at 10,000 readings,
back to back). Microseconds, median and 99th percentile.

macOS:

| Readings | Zones | Call | Lean p50 | p99 | Upstream p50 | p99 |
| ---: | --- | --- | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 4.9 | 7.8 | 0.0 | 0.3 |
| 360 | polygons | command, clear | 14.8 | 22.9 | 2.7 | 4.7 |
| 360 | polygons | command, slowdown | 15.3 | 17.9 | 2.7 | 3.6 |
| 360 | polygons | command, stop | 11.7 | 15.7 | 2.1 | 2.8 |
| 360 | circles | scan | 4.9 | 6.0 | 0.0 | 0.3 |
| 360 | circles | command, clear | 9.0 | 11.3 | 1.6 | 2.0 |
| 360 | circles | command, slowdown | 9.0 | 9.8 | 1.7 | 1.8 |
| 360 | circles | command, stop | 8.6 | 10.9 | 1.6 | 2.0 |
| 1080 | polygons | scan | 14.5 | 17.5 | 0.0 | 0.5 |
| 1080 | polygons | command, clear | 42.8 | 52.6 | 6.9 | 8.3 |
| 1080 | polygons | command, slowdown | 45.0 | 54.5 | 7.0 | 8.2 |
| 1080 | polygons | command, stop | 33.3 | 39.8 | 5.3 | 6.0 |
| 1080 | circles | scan | 14.5 | 21.7 | 0.0 | 0.5 |
| 1080 | circles | command, clear | 26.1 | 37.9 | 4.0 | 5.9 |
| 1080 | circles | command, slowdown | 26.0 | 31.9 | 4.0 | 4.6 |
| 1080 | circles | command, stop | 24.6 | 29.8 | 3.8 | 4.4 |
| 10000 | polygons | scan | 132 | 157 | 0.1 | 0.5 |
| 10000 | polygons | command, clear | 403 | 438 | 56.7 | 69.0 |
| 10000 | polygons | command, slowdown | 404 | 441 | 56.8 | 67.5 |
| 10000 | polygons | command, stop | 321 | 361 | 41.8 | 51.6 |
| 10000 | circles | scan | 132 | 157 | 0.1 | 0.5 |
| 10000 | circles | command, clear | 238 | 268 | 30.1 | 37.5 |
| 10000 | circles | command, slowdown | 242 | 272 | 30.0 | 38.1 |
| 10000 | circles | command, stop | 237 | 268 | 28.1 | 32.1 |

Linux (container):

| Readings | Zones | Call | Lean p50 | p99 | Upstream p50 | p99 |
| ---: | --- | --- | ---: | ---: | ---: | ---: |
| 360 | polygons | scan | 6.3 | 8.6 | 0.0 | 0.1 |
| 360 | polygons | command, clear | 17.2 | 23.2 | 3.0 | 4.0 |
| 360 | polygons | command, slowdown | 17.0 | 23.9 | 3.1 | 4.4 |
| 360 | polygons | command, stop | 13.5 | 20.8 | 2.5 | 3.5 |
| 360 | circles | scan | 6.4 | 8.6 | 0.0 | 0.1 |
| 360 | circles | command, clear | 11.0 | 14.8 | 2.0 | 2.3 |
| 360 | circles | command, slowdown | 11.4 | 15.3 | 2.0 | 2.8 |
| 360 | circles | command, stop | 10.6 | 16.0 | 2.0 | 3.7 |
| 1080 | polygons | scan | 19.1 | 24.7 | 0.1 | 0.1 |
| 1080 | polygons | command, clear | 49.8 | 63.4 | 7.9 | 11.7 |
| 1080 | polygons | command, slowdown | 50.4 | 68.1 | 8.0 | 12.1 |
| 1080 | polygons | command, stop | 39.0 | 48.8 | 6.1 | 8.4 |
| 1080 | circles | scan | 19.4 | 24.0 | 0.0 | 0.1 |
| 1080 | circles | command, clear | 31.6 | 39.8 | 5.0 | 6.8 |
| 1080 | circles | command, slowdown | 31.6 | 38.9 | 5.0 | 6.8 |
| 1080 | circles | command, stop | 29.8 | 38.5 | 4.8 | 6.5 |
| 10000 | polygons | scan | 195 | 229 | 0.1 | 0.5 |
| 10000 | polygons | command, clear | 451 | 502 | 95.7 | 128 |
| 10000 | polygons | command, slowdown | 464 | 507 | 97.4 | 129 |
| 10000 | polygons | command, stop | 373 | 420 | 82.2 | 117 |
| 10000 | circles | scan | 204 | 245 | 0.1 | 0.5 |
| 10000 | circles | command, clear | 294 | 402 | 69.0 | 109 |
| 10000 | circles | command, slowdown | 292 | 405 | 69.2 | 108 |
| 10000 | circles | command, stop | 287 | 340 | 68.8 | 108 |

The command callback of the Lean node now takes 5.4 to 8.4 times
upstream's on macOS and 4.2 to 6.4 times in the container (macOS:
9.0 against 1.6 µs at 360 readings with circles; 43 against 6.9 µs at
1080 with polygons; 0.40 ms against 57 µs at 10,000 with polygons). In
the runs of this document it took 340 to 780 times upstream's on macOS.

**The scan callback** takes 4.9 µs at 360 readings, 14.5 µs at 1080 and
132 µs at 10,000 on macOS (6.4, 19.4 and 204 µs in the container), about
13 ns per reading on macOS: the shim computes the points (`encode_scan`)
and the core decodes the ranges and keeps the bytes. Upstream stores a
pointer (0.04 µs) and does all its work in the command. In the runs of
this document the scan callback took 210 to 230 ns per reading, and still
225 ns after the first step of task T03 (240 to 250 µs at 1080 readings,
`bench/results/cm_comparison/t03/at_759d895/`): the scan call decoded the
coordinates of every record into a list of pairs of `Nat` patterns, which
nothing but the theorems read any more. Now the monitor keeps the bytes,
and that list is a function of them that no exported call evaluates
(`Monitor.coords`).

With 1080 readings at 25 Hz, commands at 50 Hz and two polygon zones that
is 25 × 14.5 µs + 50 × 42.8 µs = 2.5 ms of work per second, 0.25 % of
a core, from the hot-loop medians on macOS (16 % in the runs of this
document; upstream 0.04 %).

**Through the nodes** (`run_cm_bench.py node`, as in
[Through the nodes](#through-the-nodes) below: both nodes side by side,
scans at 25 Hz, commands at 50 Hz for 20 s, polygon zones; one run per
platform; latency in microseconds).

macOS:

| Readings | Node | Answers | p50 | p90 | p99 | max | Answers that differ |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 360 | lean | 849 | 6 302 | 9 485 | 11 317 | 15 165 | 0 of 849 |
| 360 | upstream | 850 | 5 802 | 9 474 | 11 340 | 28 145 | 0 of 849 |
| 1080 | lean | 850 | 6 880 | 10 246 | 11 358 | 11 457 | 0 of 850 |
| 1080 | upstream | 850 | 6 292 | 9 770 | 11 335 | 11 418 | 0 of 850 |
| 10000 | lean | 852 | 7 681 | 10 967 | 12 430 | 13 097 | 0 of 852 |
| 10000 | upstream | 852 | 5 776 | 9 588 | 11 335 | 11 693 | 0 of 852 |

Linux (container):

| Readings | Node | Answers | p50 | p90 | p99 | max | Answers that differ |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 360 | lean | 851 | 372 | 495 | 745 | 2 005 | 0 of 851 |
| 360 | upstream | 852 | 346 | 478 | 737 | 1 999 | 0 of 851 |
| 1080 | lean | 851 | 451 | 592 | 707 | 800 | 0 of 851 |
| 1080 | upstream | 851 | 328 | 428 | 533 | 765 | 0 of 851 |
| 10000 | lean | 852 | 2 239 | 2 519 | 2 713 | 2 992 | 0 of 852 |
| 10000 | upstream | 852 | 767 | 928 | 1 104 | 1 592 | 0 of 852 |

On macOS both nodes sit on the middleware's tick of several milliseconds,
as in the runs of this document. With 10,000 readings the Lean node now
answers every command that upstream answers (in the runs of this document
532 on macOS and 678 in the container, against upstream's 850). The tables
of [Per-call latency](#per-call-latency) above and of
[Through the nodes](#through-the-nodes) below are those of the node as it
was when this document's runs were made (option B); they are kept as the
record of the defect.

**The random comparison, repeated.** `random_diff.py NODE UPSTREAM A B
--roslean`, seeds 0 to 299 in six processes of 50. On the final code (commit `771c053`): 0 cases outside the bounds, none retried, 26,951 commands, of
which 745 depend on a point in a growth band; the two nodes demand
different velocities for 586 and publish differently for 761. **For every
one of the 300 seeds the line of the log (verdict, number of commands,
commands that depend on a boundary, commands answered differently from
upstream, published answers that differ, farthest deciding point) is the
same as in the run of this document made before the change**
(`bench/results/cm_comparison/macos/random_[0-2].log` against
`bench/results/cm_comparison/t03/random_?.log`), and so are the totals. A
first run, on the first version of the fast path (`759d895`), gave the same
300 lines except that seed 223 passed only on the harness's slow retry: on
the first attempt *upstream's* answer to one command was not what the
test's own model of upstream says (an `oracle` problem, command 82); a
benchmark of this task was running on the machine then, and in the final
run the seed passed at once. In the Linux container (`1c68b38`): 0 cases
outside the bounds, none retried, 26,951 commands, 747 that depend on a
growth band, 588 demanded and 763 published differently, and again every
one of the 300 lines is that of this document's Linux run
(`bench/results/cm_comparison/linux/random_[0-2].log` against
`bench/results/cm_comparison/t03/linux/random_?.log`).

**The point test, repeated.** `band_bounds 100000 7` (34 million points
against upstream's zone tests): the log is the same as this document's,
line for line, on macOS and in the Linux container: the same 5,291,086
points of 16.2 million on which the two circle tests disagree, the same
6,487,576 of 17.8 million for polygons, the same count in every band of
distance. Both bounds hold.

### Through the nodes

Both node binaries side by side on the same topics (upstream activated
through its lifecycle), wall-clock time, `enable_stamped_cmd_vel`, polygon
zones. `cm_load` publishes scans at 25 Hz and commands at 50 Hz for 20 s;
the scene changes every 2.5 s (nothing near, an obstacle in the slowdown
zone, an obstacle in the stop zone). Latency from `publish` of a command to
the callback of a subscriber on the node's output. Microseconds.

macOS (option B, being replaced):

| Readings | Node | Answered / sent | p50 | p90 | p99 | p99.9 | max |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | lean | 850 / 1000 | 8 614 | 11 725 | 14 117 | 14 792 | 14 792 |
| 360 | upstream | 851 / 1000 | 5 971 | 9 355 | 11 271 | 11 524 | 11 524 |
| 1080 | lean | 852 / 1000 | 10 241 | 14 035 | 16 696 | 18 683 | 18 683 |
| 1080 | upstream | 851 / 1000 | 5 922 | 9 055 | 10 715 | 11 348 | 11 348 |
| 10000 | lean | 532 / 1000 | 46 253 | 55 976 | 60 213 | 62 256 | 62 256 |
| 10000 | upstream | 851 / 1000 | 6 202 | 9 206 | 10 968 | 11 319 | 11 319 |

Linux (container, option B):

| Readings | Node | Answered / sent | p50 | p90 | p99 | p99.9 | max |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 360 | lean | 851 / 1000 | 3 169 | 3 558 | 3 784 | 4 467 | 4 467 |
| 360 | upstream | 851 / 1000 | 384 | 535 | 746 | 1 750 | 1 750 |
| 1080 | lean | 850 / 1000 | 5 804 | 6 285 | 6 990 | 7 718 | 7 718 |
| 1080 | upstream | 850 / 1000 | 260 | 336 | 448 | 656 | 656 |
| 10000 | lean | 678 / 1000 | 34 071 | 42 169 | 46 317 | 48 287 | 48 287 |
| 10000 | upstream | 850 / 1000 | 235 | 281 | 368 | 843 | 843 |

Answers of the two nodes to the same command, compared by `linear.x`
(commands that one node left unanswered are not in the comparison). macOS:

| Readings | Answered by both | Answers differ | Lean node less restrictive | Upstream less restrictive |
| ---: | ---: | ---: | ---: | ---: |
| 360 | 850 | 0 | 0 | 0 |
| 1080 | 851 | 0 | 0 | 0 |
| 10000 | 531 | 2 | 0 | 2 |

Linux:

| Readings | Answered by both | Answers differ | Lean node less restrictive | Upstream less restrictive |
| ---: | ---: | ---: | ---: | ---: |
| 360 | 851 | 0 | n/a | n/a |
| 1080 | 850 | 0 | n/a | n/a |
| 10000 | 677 | 1 | n/a | n/a |

The Linux table is from the first of the runs made (its `cm_load` did not
yet record the direction of a difference, hence "n/a"); the macOS tables
are from a later run that did. Two more runs on macOS: one before the
direction was recorded (0 of 850, 5 of 846 and 1 of 545 answers differ at
the three sizes) and one after (none differ at any size).

Reading (all of it with the exact transform in Lean, option B, which is
being replaced; to be repeated after T05b and T06):

- **Latency.** In the Linux container the middleware takes 0.2 to 0.4 ms,
  and the node's own time is what is left: the Lean node's median is 3.2 ms
  at 360 readings, 5.8 ms at 1080 and 34 ms at 10,000, upstream's 0.24 to
  0.38 ms. On macOS, where upstream's median is 6 ms at every size (as if
  messages were picked up on a 10 ms tick), the Lean node's is 8.6, 10.2
  and 46 ms.
- **Commands lost.** At 10,000 readings a command takes longer than the
  20 ms between two commands, and the subscription keeps one message: the
  Lean node answered 532 of 1000 commands on macOS and 678 in the
  container, upstream 851 and 850 (the other 150 fall into the silence
  after `stop_pub_timeout` in the stop scene, for both nodes). At 360 and
  1080 readings both nodes answered the same number, within two.
- **Answers that differ.** Both nodes decide alike on the same events, but
  a node that is busy takes in a new scan later, so at a change of scene
  the two can answer the same command from different scans. Seen: 9
  commands in four runs of 20 s per size (three on macOS, one in the
  container), 5 of them at 1080 readings and 4 at 10,000; none at 360. For the 2 whose direction was recorded, the Lean
  node was the more restrictive (it still had the obstacle). The other
  direction, the Lean node passing a command that upstream already stops
  because the obstacle's scan is still waiting behind a long command
  callback, follows from the same delay and must be expected; it was not
  observed in the runs whose direction was recorded, and the longer run
  that was to count it was stopped (see "What was not tested"). This is
  not a divergence of the decision, which the random comparison covers; it
  is the delay of defect 2 seen from outside, bounded by the time of one
  command callback.
- **Before the extension of the model** (commit `91c21f9`, the same
  command, `bench/results/cm_comparison/before_t05/`): the Lean node's
  median in the container was 0.99, 2.4 and 7.5 ms at the three sizes
  (upstream 0.41, 0.37 and 0.31), on macOS 6.3, 7.1 and 11.7 ms (upstream
  5.9, 5.9 and 6.0); both nodes answered 850 to 852 of 1000 commands at
  every size and no answer differed, on either platform.

### Memory

Both nodes side by side for 32 minutes: 1080-reading scans at 200 Hz and
commands at 400 Hz, i.e. 5 to 8 times the usual rates, the scene changing
every 2.5 s through pass, slowdown and stop: 384,000 scans and 768,000
commands, 1,152,000 messages sent. Resident set size and CPU time of each
process from `ps` every 10 s (193 samples). `run_cm_bench.py soak --minutes
32`, once on each platform, on the node as merged at `b503265` (option B).
The tables list every change.

macOS:

| Time | Lean node, RSS kB | Upstream, RSS kB |
| ---: | ---: | ---: |
| start | 25,488 | 30,592 |
| 10 s | 26,032 | 30,816 |
| 20 s | 26,032 | 30,848 |
| 430 s | 26,064 | 30,864 |
| 770 s | 26,048 | 30,848 |
| 980 s | 26,064 | 30,880 |
| 1870 s | 26,080 | 30,896 |
| end, 1920 s | 26,080 | 30,896 |

Linux (container):

| Time | Lean node, RSS kB | Upstream, RSS kB |
| ---: | ---: | ---: |
| start | 76,512 | 41,100 |
| 10 s | 76,988 | 41,520 |
| 670 s | 76,988 | 41,524 |
| 700 s | 77,032 | 41,912 |
| 710 s | 77,032 | 41,960 |
| 1790 s | 77,032 | 41,968 |
| 1820 s | 77,072 | 41,968 |
| 1910 s | 77,072 | 41,980 |
| end, 1920 s | 77,072 | 41,980 |

The Lean node's resident memory rises by about 0.5 MB in the first ten
seconds and by 48 kB (macOS) and 84 kB (Linux) in the 32 minutes after
that, in steps of 16 to 44 kB. Upstream's rises by 80 kB and 460 kB in the
same time, and on macOS in steps at the very same samples, so those steps
belong to what the two processes share, not to the core. In 32 minutes it
does not grow in any way that matters; whether the last small steps would
continue over hours was not measured. The two platforms count resident
memory differently (26 MB on macOS, 77 MB on Linux for the same node: the
Lean runtime's library is counted on Linux); only the curves are the
result.

CPU time over the run (option B, being replaced): the Lean node 1,412 s on
macOS (74 % of a core) and 1,914 s in the container (99.7 %: it computed
without pause); upstream 106 s (5.5 %) and 53 s (2.8 %).

Not every message sent was processed, by either node: several commands
arrive between two turns of a node's executor and a subscription that
keeps one message hands over the last (UPSTREAM_BEHAVIOUR.md, finding 11).
Of the 768,000 commands the Lean node answered 314,906 on macOS and 569,489
in the container, upstream 388,463 and 614,366. Of the answers that both
nodes gave to the same command, 202 of 215,391 differ on macOS (0.09 %) and
210 of 569,413 in the container (0.04 %); the direction of these
differences was not recorded in the soak (see
[Through the nodes](#through-the-nodes) for what they are). Both processes
were alive at the end on both platforms.

A soak of the node before the extension was started and stopped when the
integration branch changed; there is no memory curve of that node from
this work. The in-process soak (`bench_cm --soak`) was not completed
either; see below.

## The Nav2 extension (review CM5, task T08)

Run on 2026-11-24 on macOS arm64 (18 cores), the node of commit `e481459`
(lifecycle node, transforms from TF, zones fed by a topic, state topic)
beside upstream 1.3.12 from the pixi environment. The sections above
describe the node before the extension where they speak of D2, D11, D13 and
of `roslean_sensor_pose`; the numbers here are the node as it is now.
Findings: `docs/REVIEWS.md`, "CM5".

| Run | Command (in `ros/roslean_collision_monitor/test`, `NODE` the Lean node, `UP` upstream) | Result |
| --- | --- | --- |
| Fixed scenarios | `scenario.py NODE --profile profile_roslean` | 0 failures in 318 checks, 73 scenario runs; stated differences that occurred: D1 1, D4 20, D5 3, D6 1, D8 1, D9 2, D10 2 |
| The same with state messages compared | a copy of the profile with `STATE_TOPIC = True` | 32 failures in 318 checks, all state messages, no velocity (classified below) |
| Scenarios of the extension | `scenario_nav2.py NODE --profile profile_roslean_nav2` | 0 failures in 246 checks, 35 scenario runs, 2 not run (`footprint-approach-unchanged`, `shift-interpolation`); ND4 2, ND5 1, ND6 2, ND11 3 |
| Random cases | `random_diff.py NODE UP a b --roslean`, seeds 0 to 299 in six processes of 50 (`review/collision_monitor/nav2/run_review_random.sh diff OUT 6 50 180`) | 300 cases, 26,951 commands, 16,092 scans: 0 cases outside the bounds of D4, 0 passed only on the slow retry |
| Random cases of the extension | `random_nav2.py NODE UP a b`, seeds 0 to 299 in six processes of 50 (`run_review_random.sh nav2 OUT 6 50 180`) | 300 cases, 16,797 commands: 0 cases in which either node disagrees with the model of N4–N7, N13, N23–N27, N32–N35 (velocities and state messages), 0 retried |

**Classification of every difference between the two nodes.**

| Where | Differences | Class |
| --- | --- | --- |
| `scenario.py` | 30 checks with another velocity | stated: D1 1, D4 20, D5 3, D6 1, D8 1, D9 2, D10 2 |
| `scenario.py`, state messages on | 32 state checks: 18 at a check whose velocity is a stated difference (D4 16, D6 1, D8 1), 14 at the check after one (D4 13, D6 1), where upstream announces a stop that the Lean node announced one check earlier | stated (consequences of D4, D6, D8); the profile's format cannot express them, so `STATE_TOPIC` stays `False` |
| `scenario_nav2.py` | 8 checks | stated: ND4 2, ND5 1, ND6 2, ND11 3 |
| `random_diff.py` | 586 commands with different demanded velocities (2.17 % of all; 761 published differently), each decided by a point in a growth band: 323 by a circle (farthest 2.056 mm outside upstream's zone), 263 by a polygon (farthest 3.044 mm); the Lean node the more restrictive one every time | stated: D4 (bounds 3.5 mm and 4.4 mm) |
| `random_nav2.py` | none: both nodes gave the model's answer to all 16,797 commands (the generator stays 0.1 mm away from zone boundaries, so D4 does not occur) | |
| unrecorded difference, defect of the node or model, defect of the harness | none in these runs | |

No case in these runs has upstream stopping or slowing and the Lean node
not. The two cases the review found where it happens or where the node
hangs are outside what the scenarios and the generators produce (a polygon
message with a NaN vertex other than the one arrangement of `polygon-topic`;
a backward clock jump during a wait for a transform): REVIEWS.md, CM5,
findings 1 and 2, with `review/collision_monitor/nav2/review_probe.py
nan-vertex` and `wait-clock-reset`.

**Nav2's own launch file** (`review/collision_monitor/nav2/run_bringup.sh OUT
node|component`): `navigation_launch.py` of `nav2_bringup` 1.3.12 with
`package='roslean_collision_monitor'` in the monitor's `Node` action
(`use_composition:=False`), or package and plugin
(`roslean_collision_monitor::CollisionMonitorNode`) changed in its
`ComposableNode` and loaded into `component_container_isolated`;
`nav2_params.yaml` with the fixed-polygon edit of SPEC_NAV2 8.6; wall clock;
static transforms and a scan from a test script, no simulator. In both forms
Nav2's lifecycle manager configured, activated and bonded the node with the
other servers ("Managed nodes are active"); a command of 0.2 m/s sent to
`cmd_vel_nav` arrived on `cmd_vel` through the velocity smoother and the
monitor; with twenty readings at 0.15 m it became zero with state
`('FootprintStop', STOP)`; with the scan clear again it passed, state
normal. No robot model, no localisation and no navigation goal were
involved.

**Gazebo** (`pixi run cm-check`, with a display): passed in 75 s (27 s
simulated), every check ok.

Not run in this review: the Linux container (`bench/cm_linux`), so its
image was not changed either; soaks; benchmarks; `component_container` and
`component_container_mt`.

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
  why a paced command costs two to four times a hot one (caches and the
  processor's idle states are the likely cause; not examined).
- **Stopped by the orchestrator on the user's decision** (the transform
  moves to C++, so further timing of the merged node measures code that is
  being replaced): a node-level run of 120 s per scan size on each
  platform, which was to count in which direction answers differ at scene
  changes (the Linux runs have no direction at all, the macOS runs 2
  commands); the in-process soak (`bench_cm --soak`, 400,000 events) on
  both platforms; a profile of the command path. Also not made: a memory
  curve and a node-level direction count of the node before the extension.
- **Everything about run time after T05b and T06.**
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
| `band_bounds` | core against upstream's zone tests, 6.8 million points | 35 s | 30 s | |
| `roslean_cmd_cost` | a command is within the budget of [Cost of a command](#cost-of-a-command) (it was expected to fail until T03) | 6 s | not run as a test since (the program: within the budget) | |
| `t2_point_as_upstream` | macOS only: the shim's point for one reading against upstream's bits | under 1 s | not registered | |
| `roslean_fault_clock_negative` | a negative `/clock` message: the node answers zero and goes on, upstream is gone (D15; it was expected to fail until T03) | 8 s | not run as a test since (the case passes in `faults`) | |
| `upstream_probes` | the 18 probes on upstream's binary | 35 s | 33 s | |
| `roslean_scenario` | the Lean node through all scenarios and probes, differences stated per check | 274 s | 271 s | long |
| `roslean_fault_injection` | 19 fault cases, upstream beside the node | 93 s | 92 s | long |
| `roslean_random_diff` | seeds 0 to 5 of the random comparison | 95 s | 95 s | long |

All 15 tests of the package (14 in the container) passed in that run: 22
min 11 s on macOS, 21 min 45 s in the container, while the soak of the
section before was running on the same machine. `band_bounds` took 2.4 s
before the extension of the model; it is the same program on the same
points (defect 2).

Those that use topics have `ROS_DOMAIN_ID` 121, 92, 91, 120 and 93.
Neither of the two tests that were expected to fail is marked `WILL_FAIL`
any more. (The three `roslean_nav2_*` tests that task T07 added are, until
the node has the lifecycle interface of T06.)

`pixi run test` as a whole was not run for this document. An estimate of
its duration on this Mac: 34 minutes, namely the 22 minutes above plus
what the other packages took when task T01 ran the whole (24 min 25 s in
all, of which this package's seven tests of that time take about 13
today).
`pixi run test-quick` took 7 min 9 s on macOS (11 tests, 0 failures).

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
