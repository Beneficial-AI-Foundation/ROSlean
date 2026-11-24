# Independent reviews

After each milestone a separate agent reviewed the work without seeing the
author's reasoning. This file records what each review found and what was
done. "Fixed" means changed in the repo; "documented" means the behaviour is
intended and now stated; "open" means not addressed.

## M1 — specification

No rule was wrong.

| Finding | Outcome |
| --- | --- |
| A never-heard lock with a timeout is locked only once the clock has passed the timeout | Fixed in R6 |
| Upstream re-reads the clock for every expiry check; one reading per event is our idealisation | Documented as D4 |
| Upstream converts timeouts to integer ns by truncation, not rounding | Fixed (R1, D2; the shim truncates) |
| Wrong YAML type for a parameter is fatal | Documented in R14, reproduced |
| Clamping interacts with the distinctness check | Fixed (D1; check runs on clamped values) |
| NaN / huge timeouts, 32-bit priority read | Documented as D5, D6 |
| Backwards clock is fail-open | Documented in A2 |
| Upstream's own tests do not pin arbitration | Documented; motivated M6 |

## M2 — Lean model

No decision differed from upstream for states reached from `State.init`.

| Finding | Outcome |
| --- | --- |
| A state shorter than the config silently dropped updates (e.g. an e-stop `true`) | Fixed: writes extend the list (`setPad`); proved total-map behaviour (`getD_setPad`) |
| Priorities not clamped in the model | Fixed: `Config.validate` clamps |
| Lock priority recomputed per input | Fixed |
| Missing example scenarios | Added |

## M3 — C interface

No reference-counting, signature or conversion defect.

| Finding | Outcome |
| --- | --- |
| Stale toolchain after a `lean-toolchain` bump | Fixed: re-detected on every configure |
| A changed Lean signature broke nothing at build time | Fixed: `ffi.h` is compiled against the generated C |
| `initialize()` not thread-safe, retry after failure falsely succeeds | Fixed |
| Moved-from `Mux` crashed | Fixed: throws |
| Test gaps (values ≥ 2⁶³, clamping, moves, builder reuse) | Added |
| Linux loading of transitive Lean libraries may fail | Open: mitigation added but unverified |
| Install tree not relocatable | Open (deployment) |

## M4 — node

No behavioural bug in the shim.

| Finding | Outcome |
| --- | --- |
| Scenario never run against upstream | Fixed in M6 |
| Scenario not registered as a test | Fixed |
| Two expectations confounded by an expired watchdog lock | Fixed: scenario rewritten under simulated time |
| Wall-clock timing fragility | Fixed: simulated time |
| Clamping and timeout normalisation lived in C++, outside the proofs | Fixed: moved to Lean (`handleOfRaw`, `Handle.clamp`), characterised by `handleOfRaw_*` and `validate_ok_iff`. The seconds-to-nanoseconds truncation stays in C++ |
| Publisher captured by reference before assignment | Fixed |
| Start-up failure mode differs (exit 1 vs abort) | Documented as D7 |
| No launch file | Documented as D9 |

## M5 — proofs

No statement wrong or vacuous; only standard axioms.

| Finding | Outcome |
| --- | --- |
| Nothing tied the C interface or `Config.validate` to the theorems | Fixed: `ROSlean/Mux/FFI.lean`, ending in `muxForward_after` |
| No state-free characterisation | Fixed: `run_forward_iff_history` |
| "The lock freezes everything" had no theorem; an input at 255 is never blocked | Fixed: `not_forward_of_top_lock`; shim warns on priority 255 |
| `inputLive_boundary` docstring overclaimed | Fixed: replaced by `inputLive_iff_le` |
| One example did not show what its comment said | Fixed |
| `run_*` theorems not phrased on `run` output via an append lemma | Open (cosmetic) |

## M6 — comparison with upstream

420 random cases (9,586 decisions) run by the reviewer: 0 divergences.

| Finding | Outcome |
| --- | --- |
| Shipped comparison small and confined to exactly representable times | Fixed: `random_diff.py` adopted, 20 seeds in the test suite, Unix-scale clock bases included |
| D2 is real and, exactly at a boundary under simulated time, frequent | Documented ("Extent of D2") |
| D8 unobservable: `rclcpp` aborts on a negative clock | Fixed in SPEC |
| Second node's own failures were discarded | Fixed |
| Bad configs are checked for our node only | Open |

## M7 — demo, documentation, reproducibility

Build, tests, proofs and demo all reproduced; every theorem named in the
README exists and matches its row.

| Finding | Outcome |
| --- | --- |
| Narrator could print "planner drives" just after an e-stop (message in flight) | Fixed: grace period after a lock change; unexpected cases are labelled |
| README had no demo instructions | Fixed |
| README "not proved" list incomplete | Fixed |
| No theorem characterised `handleOfRaw` | Fixed |
| Stale statements in PLAN.md | Fixed |
| `pixi run prove` would build Mathlib from source on a fresh clone | Fixed: fetches the cache first |
| CI is the Lean template and covers none of the ROS side | Open; listed under "Not done" |
| No LICENSE file although `package.xml` declares Apache-2.0 | Open: needs a decision by the owner |
| `twist_mux.md` still shows the two decisions as open and links to private pages | Decisions recorded and a reader's guide added on 2026-11-20; the links to private pages remain |

## After the reviews: equal priorities restored (2026-11-20)

At the user's request the node again accepts equal input priorities and
resolves them as upstream does (SPEC R10; D1 withdrawn). Entries above that
mention the distinctness check, `Config.validate`, `validate_ok_iff` or
`builderCheck` describe the code as it was when reviewed: `Config.validate`
is now the total `Config.ofRaw`, the start-up check and `roslean::ConfigError`
are gone, and `forward_iff` carries the tie-break. An independent review of
this change found no error in the model, proofs, C interface or tests:

| Finding | Outcome |
| --- | --- |
| "Alphabetical by name" is looser than the real order (byte order of parameter names) | Fixed in SPEC, README, `twist_mux.md`, example config |
| "Accepts the same configurations as upstream" ignores D5 | Fixed |
| That `list_parameters` returns prefixes in map order is not verified against the `rclcpp` source (not in the pixi env); both nodes make the same call, and the tie scenario and random test agree with it | Open, noted |
| No test name contains a byte below `.`, where the order differs from a plain name sort | Open |
| No fixed ROS-level scenario for a tie created by clamping (300 vs 255); covered in the FFI test and by chance in the random test | Open |
| `forward_priority` is now `≤`; the strict fact for an earlier input needs `forward_first_of_equal` too | By design; `forward_iff` carries both |

## Longer random cases (2026-11-23)

The random differential test now generates 350–550 events per case instead
of 35–55 (`random_diff.py`, `gen`); its CMake timeout went from 900 s to
2400 s. The generator is otherwise unchanged, so this adds longer histories
per case (more clock jumps, lock changes and take-overs before each
decision), not new kinds of input: still 1–6 inputs, 0–4 locks, and no event
within 25 ms of a timeout boundary (D2).

Full suite in a Linux arm64 container (Ubuntu 24.04, ROS Jazzy and upstream
`twist_mux` from apt):

| Test | Result | Time |
| --- | --- | --- |
| `test_mux_ffi` | passed | 0.3 s |
| `scenario` | passed; both nodes agree on all 69 decisions | 58 s |
| `random_diff`, seeds 0–19 | passed; 0 divergences in 4,692 decisions | 412 s |

Decisions per random case: 157 to 359 (13 to 33 before; 447 in total
before). Every seed passed on the first attempt, with no slow retry.

| Finding | Outcome |
| --- | --- |
| No full run on macOS: the host runs were interrupted by another process on the machine. Seeds 0–9 completed there and passed with the same decision counts as in the container | Open |

## Linux through pixi (2026-11-23)

Until now the pixi route had been run on macOS only. Run on Ubuntu 24.04,
x86-64, four cores, no screen and no graphics card; pixi 0.84.2, Lean
4.37.0-rc1.

| Step | Result | Time |
| --- | --- | --- |
| `pixi run build`, first run | `3 packages finished` | 76 s including the download of the ROS environment (5.9 GB on disk) |
| `test_mux_ffi` | passed | 0.4 s |
| `scenario` | passed | 58 s |
| `random_diff`, seeds 0–19 | passed; 0 divergences | 413 s |
| turtlesim demo on a virtual display, messages published in place of the keyboard | narrator output and turtle as in the tutorial | |
| `teleop_twist_keyboard` fed key presses through a pseudo-terminal | one message per key press on `key_vel` | |
| Gazebo demo, patrol and Nav2, virtual display, software rendering, world without the furniture | patrol, take-over, e-stop and release as in the tutorial; Nav2 reached the first stop of its tour | |

| Finding | Outcome |
| --- | --- |
| `pixi run demo-estop` used `ros2 topic pub --once`, which waits for one subscriber and exits. With Gazebo loading the machine, the lock message reached only one of its two subscribers: in two runs the narrator missed it, and in one run the mux missed it, so the robot kept driving with the e-stop "engaged". Not specific to Linux; it depends on how busy the machine is | Fixed: `demo-estop` and `demo-release` wait for two subscribers and send the message five times. Two further Gazebo runs and one Nav2 run delivered it to both |
| The mux subscribes to lock topics with the system default QoS, as upstream does; `ros2 topic info` reports it as best effort here. A lock message can therefore be lost without notice, which matters for an e-stop | Open. A real e-stop source should publish periodically, with a lock timeout |
| Gazebo does not start when the furniture cannot be downloaded (the test machine could not reach `fuel.gazebosim.org`); the tutorial said the room would be empty | Tutorial corrected. The Linux run used a world without the furniture, so the furnished apartment is untested on Linux |
| With no display at all, `gz sim` crashes at start in the Ogre2 renderer, also with `gui:=false` | Documented in the tutorial |
| `pixi run prove` not run on Linux | Open |

## CM1/CM2 — collision monitor specification and model

No rule R1–R19 contradicts the source, the model implements the rules, and
the numerical claims held under test; the defects are at the shim boundary
(two of them lose a stop that upstream makes) and in the properties, which a
node that never publishes a stop would satisfy.

Findings are ordered by severity. Each names the smallest input or
configuration that shows it. Scripts are in `review/collision_monitor/`;
"run" means a run of the upstream binary with `upstream_probe.py`.

| # | Finding | Outcome |
| --- | --- | --- |
| 1 | **A negative `radius` gives an empty zone.** Upstream squares the radius (R9), so `radius: -0.3` is a 0.3 m zone; in the run, one point at 0.2 m with `min_points: 1` stops the robot. The shim conversion of "Shim boundary", `(int64_t)ceil(radius * 1000.0) + 1`, gives −299, which Lean's `.toInt.toNat` makes 0; `Config.ok` accepts a circle of radius 0, and the command passes. No row of D1–D14 refuses or mentions it. The conversions are said to act on doubles "already range-checked", but no range is given for the radius or for vertices; for NaN or a huge value the cast is undefined behaviour before Lean sees anything | Fixed: the radius crosses as its bit pattern and Lean takes the magnitude (`radiusMm`, `radiusMm_abs`); NaN, infinite and too large values are refused in Lean; no cast in the shim depends on a floating-point value. `test_cm_ffi`: `radius: -0.3`, a point at 0.2 m, stops |
| 2 | **D11: the scan's frame is not read, and the default sensor pose is zero.** With `base_shift_correction: false` and a scan whose `frame_id` is not `base_frame_id`, upstream looks up the transform, and stops with `invalid source` if there is none (run: `frame_id: laser_link`, nothing within 2 m, command 0.5 → 0.0). The Lean node would take the same points as base-frame points and pass. With an existing Nav2 parameter file plus the one required edit (`base_shift_correction: false`), a lidar in `base_scan` mounted backwards puts an obstacle that is ahead of the robot behind it, with only a warning that `base_frame_id` is ignored. A pose `[x, y, yaw]` also cannot express a lidar mounted upside down (roll π mirrors `y`), which upstream's transform handles. Smallest fix in the spirit of the SPEC: treat a scan whose `frame_id` differs from `base_frame_id` as an invalid source unless `roslean_sensor_pose` is set explicitly, with the result of the comparison passed to Lean so that the stop is in the model | Fixed, fail closed: a scan whose `frame_id` is not `base_frame_id` is not used unless `roslean_sensor_pose` is set, and discards the stored scan, so the source rule stops the robot (`Monitor.dropScan`, `dropScan_stop`; D11). The comparison of the two strings is in the shim, its consequence in Lean. Not planar poses are documented as inexpressible. That the transition is not an event of the model is open (decision 7) |
| 3 | **"Stop" and "Stale data" are satisfied by a node that never publishes zero.** Both end in "`.publish (.stop…)` or `.silent`" without saying when silence is allowed. A `step` that is silent whenever a stop is demanded satisfies every listed property (Pass-through only constrains the case with no triggered zone). The model's actual behaviour is stronger and should be the statement: the output is `.silent` iff `!v.finite`, or the action yields zero and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout < now`; otherwise it is `.publish (evaluate …)` | Fixed: `output_eq_silent_iff`, `output_of_isStop`; `stop` and `stale` now state the output exactly |
| 4 | **No "only if" direction.** "Slowdown bound" allows `.stopZone _` whenever a slowdown zone triggers, and nothing says that `.stopZone k` implies zone `k` is an enabled, triggered stop zone, that `.stopSource` implies an invalid source, or that `.slowdown k r` implies zone `k` triggered and no stop zone did. A model that stops whenever any slowdown zone triggers satisfies all properties. twist_mux has `forward_iff`; the equivalent here is one theorem giving `evaluate` by cases | Fixed: `evaluate_eq_*_iff`, one per action, and `output_eq_publish_*_iff` |
| 5 | **R15 has no property, and the invariant that makes silence acceptable is missing.** The properties take `st` as arbitrary and never constrain `prevZero` and `stopStamp` after a step (only `scan`). So a model that never records a zero, or records the wrong stamp, is not excluded. Missing statements on `run`: (a) if the output for a finite command is `.silent`, the action published last before it yields zero for its command (this is what A3 relies on; it holds by inspection of `step`, not proved here); (b) after a zero is published with a non-zero predecessor at `t`, later zeros are published exactly while `now ≤ t + stop_pub_timeout`. Scenario that shows why (a) matters, checked in `ModelCases.lean` and in the run: zero commands for longer than `stop_pub_timeout`, then two points in the stop zone and a command of 0.5 m/s: the stop is demanded and never published, not once. It is safe only because the last published message was the idle zero | Fixed: `step_cmd_prevZero`, `step_cmd_stopStamp`, `run_silent_last_published_zero`, `zero_period`. The scenario is confirmed and is now an example; a demanded stop is not always published (THEOREMS, "What the robot receives") |
| 6 | **No property mentions geometry.** Every property is relative to `Zone.triggered`. For polygons there is no theorem at all: not `insidePolygon vs p → contains`, not a characterisation for an axis-parallel box, not the sentence the SPEC states as fact ("`Config.ok` … so a saturated point is in no zone"). A wrong sign in `crosses` would pass every property and be caught only by the examples. Also missing: monotonicity (adding points to a scan never turns a stop into a slowdown or a pass, and never raises the selected ratio), which holds by inspection and is what a safety engineer expects of a zone test | Fixed: `Geometry.lean` (polygon, margin, saturated points) and the monotonicity theorems. The topological half of the polygon safe-direction claim stays prose |
| 7 | **Start-up refusals and conversions that decide safety sit in C++.** (a) D9 is refused by the shim; if that check is missing or wrong, Lean's `.toInt.toNat` turns a negative `source_timeout` into 0, which means "check disabled": the command passes with no scan at all, where upstream treats nearly every scan as too old and stops (`ModelCases.lean`). (b) `roslean_cm_ok` is advisory: a monitor whose configuration is not ok still answers `pass`. (c) Radius, vertex and timeout conversions are floating point in C++ (T3). All three can move: pass the `double` bit patterns (radius, timeouts) and `float` bit patterns (vertices) and convert in Lean with exact integer arithmetic, keep the sign so that Lean refuses, and make a monitor that is not ok answer `stopSource` to every command. (d) `to_mm` (truncation, saturation, the NaN rule) can move as well if a beam record carries the two `double` coordinates as bits instead of two `int32`; then only `cos`, `sin` and the pose remain in T2. Otherwise the boundary can be implemented as written | Fixed: (a), (c) radius, vertices, timeouts and `min_points` cross as raw values and are converted and refused in Lean (`CollisionMonitorConvert.lean`, theorems in `Convert.lean`); (b) a monitor that is not ok answers every command with `stopSource` (`apply_cmd_of_not_ok`, `cm_not_ok_after`); (d) `to_mm` is in Lean (`coordMm`): a record carries the two coordinates as `double` bits, at 41 ns per reading |
| 8 | **One example does not show what its comment says.** `Examples.lean`, "A zero command while a stop is demanded continues the same period": the scan is cleared before the zero command, so no stop is demanded at 1900 ms (the output is `.publish .pass`). It shows that a zero command after a stop continues the stop's period | Fixed: comment corrected |
| 9 | **D8's claim is false while the clock is below the timeout.** "The stamp is taken as 0: the scan is too old under any non-zero timeout." With simulated time at 1.5 s and `source_timeout: 2.0` the scan is fresh and is used: `[.scan _ (stamp 0, nothing near), .cmd 1500 ms]` gives `.publish .pass` (`ModelCases.lean`); upstream ends the process (run: SIGABRT). Also, `sec = -1` with `nanosec ≥ 10⁹` gives a positive `stamp_ns`, so the shim's sum does not detect every stamp that upstream refuses. A negative `sec` should be an input of its own and mean "invalid source" | Wording of D8 fixed. A separate input for a negative `sec` is open |
| 10 | **D6 is incomplete.** (a) For a zero command in a triggered slowdown zone upstream takes no action (`0 < 0` is false; run: output 0.0 and no state message), the model returns `.slowdown k r`. The published bits are equal, but `out_kind`, the log line and any comparison of actions differ; "`evaluate` is independent of the command" is a difference, not a rule. (b) The parenthesis names command components only; a ratio below about 10⁻¹⁶² with an ordinary command also makes the squared norm underflow. Apart from these and from ratios one rounding apart, 300,000 random cases showed no difference | Fixed in D6 |
| 11 | **Timeouts do not always "agree exactly".** Upstream's default for `<source>.source_timeout` is the node's timeout converted to nanoseconds, back to seconds and to nanoseconds again (`Source::getCommonParameters`). For 398 of the 30,000 values `k/1000` s up to 30 s the second conversion loses 1 ns, e.g. 2.094 s: 2,093,999,999 then 2,093,999,998. A shim that converts the node parameter once differs by 1 ns at the boundary of R7. Either reproduce the double conversion or list it as a difference | Fixed: reproduced. Lean computes the default of `<source>.source_timeout` through seconds and back (`nsViaSeconds`); `smoke_timeouts.cpp` compares it with `rclcpp::Duration` (the same 398 of 30,001 values lose 1 ns; 2.094 s gives 2,093,999,998) |
| 12 | **"Bad data (a)" is the definition restated, and false if read about points.** "A beam that is not valid does not occur in `Scan.points`": `points` is a list of points, and the point of an invalid beam occurs when a valid beam has the same point (`ModelCases.lean`). The useful statement is about decisions: changing the point of an invalid beam, or removing the beam, changes no zone count | Fixed: `output_filter_validBeam`, `points_of_not_validBeam`; SPEC reworded |
| 13 | **D10 is fail-open relative to upstream.** `min_points: 4294967297` is 1 upstream (32-bit `int`; run: one point stops), and never triggers in Lean. Refuse values above 2³¹ − 1 instead of documenting the difference | Fixed: `min_points` outside −2³¹ … 2³¹ − 1 is refused (`minPointsOfRaw`, D10) |
| 14 | **R2 is not among the open decisions.** A non-finite command is the one case where the node goes silent after a non-zero message while a stop may be demanded (obstacle in the stop zone, planner emits NaN: nothing is published, a base without watchdog keeps 0.5 m/s). Decisions 2 and 3 offer a difference for the other two silences of A3; this one deserves the same question ("publish zero for a non-finite command") | open |
| 15 | **`Config.ok` accepts zones that cannot protect anything.** No zones; a circle of radius 0; a polygon whose four vertices coincide or are collinear (a disc of √5 mm, a strip 4.5 mm wide: only the D4 margin). Upstream accepts the same, so this calls for a warning at start-up, not a refusal. Related: R10's "fewer than four vertices is refused" holds only without `polygon_sub_topic`; upstream then reports "static points and sub topic both not defined" (run), and with a topic it accepts the zone as dynamic | open |
| 16 | **A refusal that is stricter than its neighbour.** `state_topic` non-empty is refused while `visualize: true` is accepted with a warning; both carry no decision (D13), and upstream's example file sets both. Not dangerous: every refusal ends before a subscription exists, so the robot does not move. No refusal was found that is unsafe | open |
| 17 | **Statements about bit patterns that are slightly off.** The model's header says bits above the format's width are ignored; a slowdown ratio is compared as a `Nat`, so `2⁶⁴ + bits(0.5)` is never selected rather than read as 0.5 (harmless: `Config.ok` refuses it and the C interface cannot produce it). The SPEC says the four helpers were compared with the hardware on 200,000 patterns; no script was committed. `Checks.lean` now does this | Fixed in the model's header and the SPEC; `Bits.lean` proves the helpers against the value formula |
| 18 | **Behaviours without an example** (the first six are in `ModelCases.lean` and can be moved): the stop that is never published (finding 5); a stop through `step` with a polygon zone listed after a slowdown zone; a slowdown ratio of 0 through `run` (R15); a zero command in a slowdown zone; a clock that jumps backwards (A2); a polygon with its closing vertex repeated; a concave polygon near its reflex vertex; `range_min > range_max`; a reading of `0.0` or `-0.0` with `range_min: 0`; `range_max` NaN (only `range_min` NaN is shown); a non-finite `angular.z`; the IEEE helpers on subnormals, negative NaN and `-0.0 <= +0.0` | open |

Confirmed and not findings: R1–R19 against `collision_monitor_node.cpp`,
`scan.cpp`, `source.cpp`, `circle.cpp`, `polygon.cpp`, `types.hpp`,
`robot_utils.cpp`, `array_parser.cpp` at `c704655`; the edge order, the
multiplied-out division and the boundary rule of `crosses`; R15 in `step`;
the first-stop and smallest-ratio selection; D5 (refusing a set sign bit is
justified: in the run `slowdown_ratio: -0.5` publishes −0.25 for 0.5).

Checks run by the reviewer:

| Check | Result |
| --- | --- |
| In `core`: `lake build ROSleanCore.CollisionMonitor`, then `lake env lean` on `Examples.lean` and on `review/collision_monitor/ModelCases.lean` | all `decide` proofs accepted; a deliberately false example was rejected |
| `Checks.lean`: `F32.le` against hardware `<=` on 323,328 pairs (every pair of 108 special patterns, also with bits above bit 31 set, and 300,000 random); `F64.isFinite` and `F64.isZero` on 300,216 patterns; "pattern below `F64.one` iff value in [0, 1) and not `-0.0`"; order of patterns against order of values | 0 disagreements |
| `Checks.lean`: upstream's polygon test in `double` against `Shape.contains` after the shim's rounding; 20,000 polygons with `float` vertices (boxes, star-shaped, self-intersecting, slivers, repeated closing vertex; 3 mm to 50 m across; centred up to 900 m away), 1,200,000 points within 3 mm of the boundary, many next to whole millimetres | no point inside for upstream and outside for the model. 178,501 points are counted only because of the margin; the farthest of them is 1.91 mm from the rounded boundary (SPEC: below 2.13; margin 2.236). Largest overshoot 3.81 mm (SPEC: 4.4) |
| `Checks.lean`: the same for circles, 300,000 points within 4 mm of the radius | none missed; largest overshoot 3.17 mm (SPEC: 3.5) |
| `Checks.lean`: upstream's slowdown loop against `zonesAction` with the shim's multiplication, 300,000 cases | ordinary commands: 49 differences, all between ratios one rounding apart (D6); never amplified, never a sign flip |
| `upstream_probe.py`: the upstream binary, version 1.3.12 from the pixi environment, macOS arm64, wall-clock time, ten cases | as quoted in the findings; in addition the boundary rule of R10 on the example box (a point on the front side is outside, one on the rear side inside), the refusal of a triangle, and the abort on a negative stamp are as the SPEC says |

Not run or not checked: the source of `rclcpp` (`duration.cpp`, `time.cpp`)
is not in the environment, so the truncation in `Duration::from_seconds`
and the form of `Duration::seconds()` assumed in finding 11 are from memory
(the abort on a negative stamp was observed); the claim that the cited files
are identical in tags 1.3.12 and 1.3.13 (only `types.hpp` was compared with
the installed 1.3.12 header: identical); `lake build ROSlean` with Mathlib
(the examples import only the core); upstream under simulated time or on
Linux; `docs.nav2.org`. There is no Lean node yet, so nothing was compared
end to end.

## CM3/CM4 — collision monitor C interface and node

No conversion, refusal, ownership or signature defect was found, and on the
same messages the node decides as upstream does up to D4 and D11; the defects
are around the node: the shipped example switches the frame check off, a
parameter change at run time is accepted and ignored, and the theorems about
event sequences do not cover a history that contains an unusable scan.

Findings are ordered by severity. Each names the smallest input that shows
it. Scripts are in `review/collision_monitor/`; "side by side" means both
nodes running at once on the same topics, parameter file and simulated clock
(`side_by_side.py`), upstream being version 1.3.12 from the pixi environment.

| # | Finding | Outcome |
| --- | --- | --- |
| 1 | **The example configuration switches the frame check off.** `config/example.yaml` contains `roslean_sensor_pose: [0.0, 0.0, 0.0]`. With the parameter set, a scan is used whatever its `frame_id` (D11), and a pose of zero means "the sensor is the base frame". Side by side (`example_pose`), with a stop box ahead of the robot, a scan in `laser_link` and a transform that says the sensor is mounted backwards: an obstacle 0.3 m ahead of the robot stops upstream (0.0) and passes the Lean node (0.5); an obstacle behind stops the Lean node only. This is CM1/CM2 finding 2 again, behind an opt-in that the file people copy takes for them. The node logs nothing at start-up when the pose is set, and nothing when a scan then arrives in another frame. `smoke_node.py` has no case with a zero pose | Fixed. `roslean_sensor_pose` is no longer set in `config/example.yaml`; a comment says what setting it switches off and when to set it. The node warns once at start-up when a pose is set, with the pose, and (throttled) when a scan arrives in another frame while it is set. `smoke_node.py`: the reviewer's case on `example.yaml` (the scan in `laser_link` is not used, the robot stops), a backwards pose in both directions, a zero pose, and the log lines |
| 2 | **A parameter change at run time is accepted and ignored.** The node allows undeclared parameters and has no callback for changes. Start with `Zone.enabled: false`, one point at 0.2 m in a 0.5 m stop circle, then set `Zone.enabled` to `true` through the parameter service (`dynamic`): both nodes answer "accepted"; upstream then stops, the Lean node keeps passing 0.5. Any name is accepted, misspelt ones included. D2 says parameters are read once, but the operator is told the change succeeded. The opposite change (`enabled` to `false`) leaves the Lean node stopping, which is the safe side. Smallest fix: reject every change after start-up | Fixed. Every parameter change after start-up is rejected with "parameters of this node are read once" (D2), and undeclared parameters cannot be set. `smoke_node.py` goes through `set_parameters` and `set_parameters_atomically`: `Stop.enabled`, `source_timeout`, a ratio, `use_sim_time`, a misspelt name and a name not given at start-up are all refused, the parameter reads back unchanged and the node behaves as before |
| 3 | **A history with an unusable scan is outside every theorem about sequences.** `Monitor.dropScan` is not `step` on an event, so `foldl_exec` needs "every scan usable", and `cm_out_after` and `cm_readBack_after` are stated on `List.foldl Monitor.apply` over events, not on the exported functions. Size of the gap: all of `Trace.lean` (`run_stop`, `run_stale`, `run_pass_through`, `run_silent_last_published_zero`, `zero_period`, `run_scan`) and the two end-to-end theorems. Three consequences. (a) The theorems that read the state from the history are false of the node once such a scan occurred: after `[usable scan with nothing near, unusable scan, command]`, `run_pass_through` on the usable events says `pass`, and the node publishes a stop (kind 4; the safe side). (b) What A3 rests on, "a finite command that gets no answer was preceded by a published zero", is not proved for these histories. `InterfaceChecks.lean` reaches the state "no scan, previous velocity not zero, stop stamp 4 s", which `run` from `State.init` cannot reach under that configuration. By inspection the statement still holds, because `dropScan` leaves `prevZero` and `stopStamp` alone; no wrong behaviour was found. (c) What remains proved for such a history is one step: `dropScan_stop` and the theorems about `step` for an arbitrary state; "until a usable scan arrives" is prose. Smallest sound fix: a third event, `Event.scanUnusable now`, with `step` returning `({ st with scan := none }, .silent)`; then `Call.event` is total, `foldl_exec` loses its hypothesis, `cm_readBack_after` can be stated on `cs.foldl Call.exec`, and each proof by cases on the event gains one case. Proving the trace theorems for a separate reachability relation instead would be more work, not less | Fixed as proposed. `Event.scanUnusable now` is a third event of the model; `step` on it clears the stored scan and is silent; `roslean_cm_on_scan` with `usable = 0` is `step` on it (`Monitor.dropScan` is gone). `Call.event` is total, `foldl_exec` has no hypothesis, and `cm_out_after`, `cm_readBack_after` (and the new `cm_readBack_at`) are stated on `cs.foldl Call.exec (cmNew b)`. `lastScan` is "the scan of the last scan event if it was usable, none if it was unusable", so every theorem of `Trace.lean` holds for all histories; `run_scanUnusable_stop` is the statement for the review's history (a). SPEC, THEOREMS.md and `Examples.lean` updated |
| 4 | **Tests that cannot fail, and what no test covers.** (a) `smoke_conversions`, "a zero pose changes nothing", compares `SensorPose{}` with `SensorPose{0.0, 0.0, 0.0}`, the same value. (b) "1000 readings: single-precision angle accumulation and products" recomputes the shim's expression with the same compiler and flags; it cannot see finding 5. (c) `test_cm_ffi`, "catches reference-count mistakes (leak or double free)": nothing is measured, so a leak passes. `leak_probe.cpp` measures it: peak memory is the same after 20,000 and after 100,000 rounds of every wrapper call. (d) In `smoke_node.py` an expectation of "nothing" is also met by a lost message or a dead node; a dead node would be noticed by a later step, a lost message would not. (e) No test in the suite runs upstream; `smoke_node.py` says so. `side_by_side.py` is the first comparison, by hand, on 88 commands. (f) Not covered at node level: a parameter change at run time, a command of `-0.0`, wall-clock time, a scan of more than five readings, a parameter file in Nav2's layout other than the two shipped ones (the refusals are passed with `-p`), a zero sensor pose | Fixed, each shown to fail once with the property broken. (a) The zero-pose test compares, bit for bit and on a scan with a `-0.0` coordinate, with a reference that has no pose (fails with the bypass of a zero pose removed). (b) The points of 200 scans are compared with `test/upstream_points.cpp`, a copy of upstream's loop in its own translation unit compiled with `-O2` (fails in a `Debug` build: 4,740 of 400,000 coordinates). (c) `test_cm_ffi` measures peak memory over 600,000 rounds of every wrapper call (fails with one extra reference taken per command). (d) "Nothing" in `smoke_node.py` requires the node's log line for an unanswered command (fails when the command is not sent); outputs are compared bit for bit. (f) Added at node level: `-0.0`, a scan of 720 readings, a zero pose, parameter changes, a parameter file with eleven refused parameters. (e) is left to the harness under `test/` that runs the node against upstream |
| 5 | **"Exactly as R3" depends on how the node is compiled, and `pixi run build` compiles it without optimisation.** The build passes no build type, so `collision_monitor_node.cpp` is compiled without `-O` (`build.ninja`: `-std=gnu++17 -arch arm64 -Wall -Wextra -Wpedantic -ffp-contract=off`), and the node calls `cosf` and `sinf`; upstream's library calls `__sincosf_stret`, which an optimising compiler substitutes for the pair (`nm -u` on both). On this machine the two ways give different points for 2,296 of 200,000 coordinates, by one unit in the last place of a `float`, at most 9.5·10⁻⁷ m for ranges up to 12 m (`fp_probe.sh`, `-O0` against `-O2`). No stop can be lost by this (D4 grows every zone by a millimetre or more) and no decision outside D4's band can differ, but the SPEC's "the two agree" for scans in the base frame is not bit for bit, and T2 is larger than "`cos`, `sin`": it includes which library entry the compiler picks. Fused multiply-add: with `-ffp-contract=on`, the compiler's default on arm64, 58 % of the coordinates computed with a sensor pose differ in the last bits and none in whole millimetres; the node's CMake sets `off`, and a second user of `conversions.hpp` would have to as well. x86-64 (under Rosetta): equal to arm64 at `-O2` without a pose, different in the last bits with one. The slowdown products were identical in all seven builds | Fixed. The package builds as `Release` when no build type is given (the node then calls `__sincosf_stret`, as upstream's binary does); `-ffp-contract=off` stays. The SPEC ("Scan event", "Safe direction", T2, D11) now says which builds agree bit for bit and gives the bound otherwise; `smoke_conversions` checks it (fails in a `Debug` build), and checks against a reference that cannot be fused that the pose is applied without fused multiply-add (fails with `-ffp-contract=fast`: verified in T01, on macOS arm64) |
| 6 | **The builder's defaults are fail-open, and one refused value is dropped.** Without `set_source` the source timeout is 0, "check off": `add_circle`, `cm_new`, a command with no scan gives `pass`. A vertex that is not finite, added after a circle or before any zone, leaves no mark and the monitor is ok (`foldl_addVertex_refused` covers vertices after `add_polygon` only; THEOREMS says "a refused vertex is never dropped silently"). The node's wrapper produces neither sequence. A builder that is not ok until `set_source` was called, and that records a stray vertex, would make the interface safe against a second shim (`ffi_probe.py`, `InterfaceChecks.lean`) | Fixed. A builder is not ok until `set_source` was called (code 5); a vertex added when the zone added last is not a polygon is recorded (code 6); a refused timeout is not cleared by a later call. `not_ok_of_refused_call` proves that a refused value anywhere in any sequence of builder calls makes the monitor not ok, which is what THEOREMS.md now claims; also `not_ok_of_stray_vertex`, `not_ok_without_set_source`. `test_cm_ffi` has the cases |
| 7 | **Messages at start-up.** (a) `radius: 999.9995` is refused with "must be finite and below 1000 m in magnitude"; the limit is that the rounded radius plus 1 mm is at most 1,000,000 mm, so 999.999 is refused and 999.9989 accepted. (b) Refusals made by the shim name one parameter per run: Nav2's own `collision_monitor_params.yaml` fails on `state_topic`, and on `base_shift_correction` once that is fixed; refusals made by the core are listed together. (c) Nothing is logged for a configuration that protects nothing: `source_timeout: 0.0`, `scan.enabled: False`, `min_points: 0`, `slowdown_ratio: 1.5`, `radius: 0.0` and four collinear vertices all start without a warning (`param_probe.py`; CM1/CM2 finding 15 is still open) | Fixed. (a) The radius message gives the bound 999.999 m and how it comes about. (b) All refusals, the shim's and the core's, are collected and reported in one fatal message. (c) A warning each for a source timeout of 0, a disabled source, `min_points` of 0 or less, a slowdown ratio not below 1, radius 0, a polygon without area, and no zones (the last cannot be reached from a parameter file). Warnings only: whether to refuse is open decisions 5 and 9 of the SPEC |
| 8 | **An output kind the shim does not know is answered with silence.** `on_cmd` logs an error and publishes nothing. `kindOf` cannot return such a value, so this is unreachable; publishing zero would match "fail closed" | Fixed. An unknown kind is answered with zero velocity and an error in the log; the comment says why it cannot occur. What is published for an answer is one function, `apply_answer` in `conversions.hpp`, so that this can be tested although the core cannot produce such a kind: `smoke_conversions` calls it with every kind from 5 to 255 (fails when an unknown kind is answered with silence, or with the command) |

Confirmed and not findings:

- Conversions. `coordMm`, `radiusMm`, `vertexMm`, `timeoutNs` and
  `sourceTimeoutNs`, called through the C interface, equal exact rational
  references on signed zeros, subnormals, NaN of either sign, infinities, the
  values three patterns either side of every limit, every whole and half
  millimetre up to 1 m, and random patterns. No parameter value or scan
  record was found on which Lean accepts what the SPEC refuses, or on which a
  zone is smaller than upstream's.
- Ownership. In the generated C every exported function that was read
  releases its object arguments (`roslean_cm_on_scan` releases the
  `ByteArray` on both branches, `roslean_cm_ok` the monitor); the wrapper
  takes one reference per query; moved-from monitors throw. The prototypes
  of `ffi.h` compile against the generated ones; `min_points` and `stamp_ns`
  (Lean `Int64`, `uint64_t` in C) were checked at −1, ±2³¹ and their
  neighbours, and at the ends of the 64-bit range.
- `mux.hpp`: the change moves `initialize()` to `runtime.hpp` unchanged;
  `test_mux_ffi` passes.
- The node. No command is published without a call into Lean, and no test on
  a floating-point value of a command is made in C++. What C++ decides is
  what the SPEC says it does: the comparison of two frame names, whether the
  pose is zero, and which features are refused. `conversions.hpp` ran on
  arbitrary bit patterns, poses up to 10³⁰⁰ and 200,000 arbitrary `points`
  strings under the undefined-behaviour and address sanitizers without a
  report. A scan with more readings than its angle fields imply, an
  `angle_increment` of 0 or NaN and an `angle_min` of NaN are treated as
  upstream treats them.
- Parameters. Nav2's `collision_monitor_params.yaml` starts once
  `state_topic` is emptied or removed and `base_shift_correction` is false;
  unchanged it is refused, as is `nav2_params.yaml` of `nav2_bringup`. For
  wrong types, a listed zone that is not defined, `points: ""`, an action
  type in capitals and the older list form of `points`, both nodes refuse
  and name the same parameter. `polygon_sub_topic` next to static points:
  upstream creates no subscription (observed), so ignoring it is right.
- Topics and QoS. Command input and output reliable and volatile, scan best
  effort, same message types, as upstream reports them. Upstream also has
  `~/collision_points_marker`, `/tf`, `/tf_static` and the lifecycle topics
  (D2, D11, D13).
- Theorems. `lake build ROSlean` succeeds; the twelve interface and
  conversion theorems that were queried depend on `propext`,
  `Classical.choice` and `Quot.sound` only; their hypotheses are
  satisfiable; each is as strong as its row in THEOREMS.md, with the limit
  of finding 3, which THEOREMS.md states.

Side by side, 88 commands: 78 answered alike, bit for bit, including `-0.0`.
The ten that differ:

| Case | Lean | Upstream | |
| --- | --- | --- | --- |
| Point on a 0.5 m circle, and 0.5 mm outside it | stop | pass | D4 |
| Point on the front and on the right side of a square (on the left and rear sides and on a corner both stop) | stop | pass | D4 |
| Point on the circle, sensor pose `[0.1, 0, 0]` against the same transform | stop | pass | D4 |
| Scan in `laser_link`, transform published, nothing near | stop | pass | D11 |
| Example's zero pose, obstacle ahead of a backwards sensor; obstacle behind it | pass; stop | stop; pass | finding 1 |
| `enabled` of a zone set to `true`; to `false` | pass; stop | stop; pass | finding 2 |

Alike, among others: the boundary of `source_timeout: 2.094` (pass at stamp +
2,093,999,998 ns, stop 1 ns later, in both: R17's double conversion); the
boundary of `stop_pub_timeout` to the nanosecond; `radius: -0.3`; a scan in
another frame with no transform, with timeout 2 s (stop) and 0 (pass); a
zero command in a slowdown zone; ratios 0, 0.3, 0.9 and 1.5; `min_points` 0
and exactly reached; NaN and infinite readings and limits; `TwistStamped`; a
scan of 100,000 readings; wall-clock time.

Checks run by the reviewer (macOS arm64, commit `cb25673`):

| Check | Result |
| --- | --- |
| `pixi run build`; `lake build ROSlean` | 4 packages finished; 6647 jobs, no error |
| `test_cm_ffi`, `test_mux_ffi`, `smoke_conversions`, `smoke_timeouts`, `smoke_node.py` (on `ROS_DOMAIN_ID` 92) | all pass; the default of the source's timeout loses 1 ns for 398 of 30,001 values |
| `ffi_probe.py`: conversions against exact references | 624,522 patterns each for coordinates, radii and timeouts, 412,222 for vertices, 230,050 for each of the two source timeouts: 0 differences |
| `ffi_probe.py`, `InterfaceChecks.lean`: the raw interface with flags and codes the wrapper does not produce, call orders, limits | as the SPEC says, except finding 6 |
| `leak_probe.cpp` | no growth |
| `side_by_side.py`, 12 groups of cases | above |
| `param_probe.py`, 32 parameter files on both nodes | above; findings 7b, 7c |
| `fp_probe.sh`: seven builds of `conversions.hpp`, and one with sanitizers | finding 5; no sanitizer report |

Not run or not checked: Linux, and x86-64 other than `conversions.hpp` under
Rosetta; `colcon test` as such (the test programs were run directly, the
node test on another domain than the one CMake sets); the twist_mux
`scenario` and `random_diff` tests after the change to `mux.hpp`; a scan of
1,000,000 readings over ROS (the middleware dropped it for both nodes; the
wrapper test covers that size); a scan stamp with negative seconds (upstream
ends, as CM1/CM2 found); upstream with `base_shift_correction: true`; a
real lidar or a simulator; the source of `rclcpp`; the generated C of the
builder functions was not read line by line (the memory measurement covers
them). Upstream waits for a missing transform in node time, so under a
frozen simulated clock it never answers; in those two cases the clock was
moved on after the command.

After the review (2026-11-23): all eight findings were addressed on the
branch `claude/cm-node-fixes`; the table above says how. The scripts in
`review/collision_monitor/` are the reviewer's and record the state at
`cb25673`; `InterfaceChecks.lean` was adapted to the renamed theorems and
now prints the fixed behaviour for finding 6. Also added at the request of
the demo's author: each change of the action is logged at INFO, in
upstream's words. Decisions still open are those of the SPEC ("Open
decisions"); the part of decision 7 about unusable scans is closed.

Verified afterwards (task T01, 2026-11-24, macOS arm64; the session that
wrote the fixes had been stopped before it reported). Each fix was taken out
again, one at a time, in a copy of the packages, and the tests were run on
the result: 34 such changes, among them every one named in the table above,
and each made `test_cm_ffi`, `smoke_conversions` or `smoke_node.py` fail.
The list and the counts are in `tasks/reports/T01.md`. Two things were added
in doing so: the test for finding 8 (there was none, the case being
unreachable through the core) and the test that the pose computation is not
fused (the flag was set but no test noticed its removal). Not covered by a
test: the warning for a configuration without zones, which a parameter file
cannot express.

## CM5 — Nav2 extension

The Lean node answers every scenario and 600 random cases as upstream does up to the stated differences, and Nav2's own launch file and lifecycle manager run it; but it lets a command pass where upstream stops when a zone's polygon message has a NaN vertex, and under simulated time it hangs when the clock jumps back while a command waits for a transform.

Review of 2026-11-24 (task T08), macOS arm64, upstream 1.3.12 from the pixi
environment, source read at `c704655` (1.3.14). Findings are ordered by
severity. "Observed" means seen on the running binaries, both nodes on the
same messages (`review/collision_monitor/nav2/review_probe.py CASE`);
"from source" means read in upstream's and the node's code and not run;
"Lean" means checked by `lake env lean` on a file in
`review/collision_monitor/nav2/`. Nothing in the node, the model or the SPEC
was changed.

| # | Finding | Outcome |
| --- | --- | --- |
| 1 | **A polygon message with a NaN vertex: upstream stops, the Lean node passes.** ND6 refuses such a message and says a NaN "blinds" upstream's zone, so that keeping the previous shape protects more. That is true of one arrangement only (the one `polygon-topic` tests). Upstream stores the vertices unchecked and its crossing test skips only the two edges at the NaN vertex; the others still count. Observed (`nan-vertex`): topic stop zone `Dyn`, `min_points: 1`, first message `(1,1), (1,-1), (-1,-1), (NaN,1)` in the base frame, one reading at (0.5, 0), command (0.5, 0, 0.25): upstream publishes zero and state `('Dyn', STOP)`; the Lean node, whose zone has no shape, publishes (0.5, 0.25). With the NaN in the first vertex the same happens for a reading at (-5, 0). With an earlier shape the Lean node keeps that one, and passes whatever only the NaN shape would catch (from source). Smallest fix that is never less restrictive than upstream: a zone whose last message was refused for a non-finite or out-of-range vertex demands a stop until a usable message arrives (a decision for the user: upstream's own behaviour here is an accident of its arithmetic) | open |
| 2 | **Under simulated time the node hangs when the clock jumps back by more than 3 s while a callback waits for a transform (N26).** tf2's waiting loop also ends when `now + 3 s < start` (`tf2_ros/src/buffer.cpp:125`, `:155`); SPEC_NAV2 N26 and 8.3 omit that condition and the node's own loop (`wait_for`) does not have it: `clock_while_waiting` returns `max(clock, start)`, so the loop runs until the clock is back at `start + transform_tolerance`. Meanwhile the one thread handles nothing: no command, scan, lifecycle service or bond heartbeat. Observed (`wait-clock-reset`): clock 100 s, scan in a frame without transform, one command, clock to 1 s: upstream publishes zero and `invalid source` at once and answers the following commands; the Lean node publishes nothing and does not answer `get_state` until the clock reaches 100.5 s again. Observed in the ordinary configuration too (`shift-wait-reset`, `base_shift_correction: true`, a command 50 ms ahead of the newest odometry sample, then a reset of the simulation): upstream stops and goes on, the Lean node is silent for as long as the test ran. With base shift correction every command newer than the last odometry sample waits, so a reset of the simulator or a looping bag is likely to land in a wait. Not reachable on the wall clock. From source, not run: upstream's clock thread also clears the `/tf` buffer during the wait; the Lean node clears it only after the callback returns, and not at all if the clock has passed its old value again by then | open |
| 3 | **Unrecorded differences, each in the restrictive or the display direction (from source, not run, except a and e).** (a) N16: upstream publishes the polygon of a zone only while the zone is enabled (`collision_monitor_node.cpp:663`); the Lean node publishes every visualised zone (observed, `disabled-visualize`: three commands, three polygon messages from the Lean node, none from upstream), and the SPEC lists the polygon publishers under "not differences". (b) N29: `points` that does not parse to four pairs leaves upstream with the topic (`polygon.cpp:477-491`); the Lean node refuses the configuration; likewise `polygon_sub_topic: ""`. (c) `slowdown_ratio` of the wrong type on a stop zone fails `configure` here; upstream does not read it for a stop zone. (d) ND3 does not say that a stop is delayed: with a triggered stop zone ahead of a topic zone in another frame whose lookup fails, the Lean node waits `transform_tolerance` before it publishes the zero. (e) Lean (`ModelProbes.lean`): a refresh of a topic zone's vertices that the core refuses (a vertex 999.997 m or more from an axis, a NaN) leaves the zone where it was; upstream moves it. E8 says so, the ND table does not. (f) If an exception leaves `configure()`, `configuring_` stays true and later parameter changes are accepted without effect (ND8) | open |
| 4 | **After a command without a clock reading, no theorem stated from start-up applies to that monitor any more.** `stepNoClock` in the active phase leaves `prevReport = source` with `prevZero = false`, and no run of `step` from `State.init` reaches such a state (Lean: `noClock_unreachable` in `NoClockUnreachable.lean`, for every configuration and event list). So `cm_readBack_after`, `cm_out_after`, `run_stop`, `run_stale`, `run_pass_through`, `run_after_activation`, `run_withheld_last_published_zero`, `reachable_inv` and the others quantified over `run cfg (State.init cfg) es` say nothing about the node from its first no-clock command until the next cleanup, also about the segments without such commands. THEOREMS.md says only that these theorems "do not mention such commands", and its "End to end" row ("there is no prefix to add") holds only for a monitor that never had one. Not a safety defect: every one-step theorem and every "from any state" theorem still applies, and what holds in general is proved in `NoClockMixedRuns.lean` (`mixed_from_init`): replace every processed no-clock command by `deactivate, activate` and drop the others; the mixed run then agrees with `run` on the substituted list in phase, scan, shapes, `prevZero`, `stopStamp` and every velocity, and differs at most in the report of the first processed command after a no-clock command. Smallest fix: take that statement into `NoClock.lean` and correct the two passages. Also: the docstring of `run_withheld_last_published_zero` says "nor has the node been deactivated since", which `ZeroPublishedLast` does not state (THEOREMS.md words it correctly) | open |
| 5 | **`lean_initialize_thread` is never called.** Lean's documentation requires it in every thread that runs Lean code and was not created by Lean. The stand-alone node runs Lean in the main thread only; as a component its callbacks run in the container's executor threads. In this toolchain (4.37.0-rc1, built with mimalloc) the function only reads an environment variable (disassembly of `libleanrt.a`), and the component ran in Nav2's container without fault (below), so no effect was observed; an object is created in one thread and used in another, never at once (one mutually exclusive callback group), which non-atomic reference counts allow. The requirement is still unmet, and a toolchain built with Lean's own small-object allocator would need the call | open |
| 6 | **Tests that would not fail.** Of 19 mutations in a scratch copy (`mutations.py`) the process-level tests caught 17. Missed: the scan's angle accumulated in double precision (`scan_points`; `smoke_transforms` has seven readings, on which the two sums agree, and `smoke_conversions` tests `scan_frame_point`, not the accumulation). One mutation was equivalent (N23 at time `target_ns`, which is 0). No test moves the clock back during a wait (finding 2), puts the NaN anywhere but where upstream's zone goes blind (finding 1), has a disabled visualised zone (3a), or loads the component into `component_container` or `component_container_mt`. `STATE_TOPIC = False` in `profile_roslean.py`: with it on, 32 of 318 checks fail, all of them state messages that follow a stated velocity difference (below); the profile's format cannot state them | open |

**What was run.** `scenario.py` with `profile_roslean`: 0 failures in 318
checks (D1 1, D4 20, D5 3, D6 1, D8 1, D9 2, D10 2). `scenario_nav2.py` with
`profile_roslean_nav2`: 0 failures in 246 checks, 35 scenarios (ND4 2, ND5 1,
ND6 2, ND11 3). `random_diff.py … 0 300 --roslean` in six processes: 300
cases, 26,951 commands, none outside the bounds of D4; the nodes demand
different velocities for 586 commands, each decided by a point in a growth
band (at most 2.056 mm outside a circle, 3.044 mm outside a polygon), the
Lean node the more restrictive one. `random_nav2.py … 0 300` in six
processes: 300 cases, 16,797 commands, both nodes give the model's answer (velocity and state message) to every one, none retried. No unrecorded difference, defect of the node or
defect of the harness appeared in these runs; findings 1 and 2 are outside
what they generate. Counts and commands: COMPARISON.md, "The Nav2 extension".

**The 32 state checks** (old profile with `STATE_TOPIC = True`): 18 are
checks whose velocity the profile already states as different (D4 16, D6 1,
D8 1): the state message comes with the changed velocity. The other 14 are
the checks that follow them (D4 13, D6 1): upstream announces the stop there,
the Lean node announced it one check earlier. No velocity differs. None is a
defect; the comparison stays off until the profile can state a state message.

**Mutations** (`mutations.py`; tests: `test_cm_ffi`, `smoke_transforms`,
`smoke_conversions`, `band_bounds`, `t2_point_as_upstream`). The trusted
transform, 11: frames swapped, times swapped, no lookup for a scan in the
base frame, the angle not advanced over a non-finite reading, the cosine in
double precision, x and y exchanged, the point at height 1, a zero quaternion
used, the base frame as the fixed frame: caught; the angle accumulated in
double precision: missed; N23 at `target_ns`: equivalent. The fast path, 8:
the band tested only inside the bounding box, `min_points` off by one, the
index not advanced over invalid readings, the slowdown tie, the stop stamp
moved by every zero, the circle's `<` as `≤`, the timeout's `<` as `≤`, more
coordinates than readings accepted: all caught by `test_cm_ffi` (and the
first two by `band_bounds`). A mutation of the fast path also breaks
`lake build` of `stepCmdFast_eq`.

**Points examined without a finding.**
- *Exceptions caught in `main`.* The only exception found that leaves a
  callback is rclcpp's on a negative `/clock`; tf2's are caught in
  `try_lookup`. Under simulated time a caught exception makes the clock
  invalid and every command a stop until a usable `/clock` message; on the
  wall clock it is logged only. A command whose callback threw gets no
  answer. No path was found that throws after the core's call and before
  the publication. A negative `/clock` taken by hand inside a wait ends the
  wait in a stop. As a component the node dies with its container on a
  negative `/clock`, as upstream's component does (its clock thread
  throws): from source.
- *The wait loop otherwise.* Tolerance boundary to the nanosecond, a
  transform arriving during the wait, tolerance 0, `source_timeout: 0`:
  T07's scenarios pass. Nothing but `/clock` is taken inside the wait:
  commands and scans queue (observed, `wait-reentrancy`; which of a queued
  scan and a queued command runs first afterwards differed between the two
  nodes in that run, and is the executor's order in both). A command queued
  during a wait was evaluated with the clock after the wait in three runs
  (`stale-clock`). SIGINT during a wait ends both nodes with a segmentation
  fault (observed, `wait-sigint`; equal).
- *Reading i and coordinate i.* One thread; the scan handed to the core and
  the one kept by the shim are set in one callback; the model ignores a scan
  only when finalized, where no command is processed; a shape is kept by the
  shim exactly when the core accepted it. No way to break the correspondence
  was found.
- *Cleanup.* After an effective cleanup the model's state is `State.init`
  except `stopStamp`, which is not read while `prevZero` is false (Lean,
  `ModelProbes.lean`); upstream keeps only its toggle and `stop_stamp_`.
  The shim keeps `clock_valid_` and the last `/clock` reading.
- *The model's events and the fast path.* No input found where the model
  passes or slows less than SPEC_NAV2 says; malformed byte arrays stop or
  are ignored; saturated coordinates lie in no accepted zone; no `sorry`,
  `native_decide`, `axiom`, `implemented_by` or `extern`; the headline
  theorems are on the exported calls and their hypotheses are satisfiable.
- *Nav2.* `navigation_launch.py` with the package name changed and
  `nav2_params.yaml` with the fixed-polygon edit (`run_bringup.sh`): every
  server active, the node configured, activated and bonded by Nav2's
  lifecycle manager, a command through the velocity smoother passes at
  0.2 m/s, is zero with twenty readings in the footprint zone (state
  `FootprintStop`), and passes again. The same with the node loaded as a
  component into `nav2_container`. No robot, no localisation, no goal.
- *Gazebo.* `pixi run cm-check`: passed in 75 s (27 s simulated).
