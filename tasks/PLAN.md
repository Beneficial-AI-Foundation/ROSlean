# Collision monitor: plan and task list

Status on 2026-11-23. Goal: a Lean replacement for the core of Nav2's
`nav2_collision_monitor` that Nav2 can launch and manage in place of its own
monitor, for `stop` and `slowdown` zones, with proved decision logic, tested
against the upstream binary, and a precise public statement of what is still
missing. Rules for sessions: [README.md](README.md).

All of this lives on the integration branch `collision-monitor`; `main` holds
none of it until the user merges the branch at the end.

## Where things stand

On `collision-monitor` (each merged after a rebuild on macOS arm64):

| Part | Where | State |
| --- | --- | --- |
| Specification | `docs/collision_monitor/SPEC.md` | R1–R19, D1–D14, read from source |
| Lean model | `core/ROSleanCore/CollisionMonitor.lean` | done |
| Proofs | `ROSlean/CollisionMonitor/` | every property of the plan's table, iff characterisations, trace theorems; no `sorry` |
| C interface and node | `core/ROSleanCore/CollisionMonitor{FFI,Convert}.lean`, `ros/roslean_core`, `ros/roslean_collision_monitor` | done; conversions in Lean |
| Upstream harness | `ros/roslean_collision_monitor/test/` | 55 scenarios and a random differential test, run on upstream only |
| Reviews | `docs/REVIEWS.md` | CM1/CM2 answered; CM3/CM4 has eight open findings |
| Gazebo demo | `ros/roslean_collision_monitor/demo/gazebo/` | headless check passes; never watched on a screen |

Not on `collision-monitor`:

| Branch | Contents | State |
| --- | --- | --- |
| `claude/cm-node-fixes` | Fixes for the eight CM3/CM4 findings, including `Event.scanUnusable` in the model | One commit on top of `collision-monitor`. The session was stopped before it reported. **Unverified.** |
| `claude/cm-comparison-wip` | Lean node through the harness, random comparison, fault injection, benchmarks, `COMPARISON.md` | Stopped before finishing. Uncommitted work was saved as one WIP commit on top of `462088e`. Placeholders (`{{…}}`) remain in the document. **Unverified.** It reports four defects. |

Nothing has been run on Linux x86-64 except the node's own tests under
emulation. The comparison with upstream has not been run on Linux at all.

## Tasks

**Decisions taken by the user on 2026-11-24** ("Decisions" below): the
transform is applied in C++ as upstream does (option A of SPEC_NAV2 section
5), which undoes part of T05; every other question is decided as its default.
T05b does the rework. Tasks start again as their needs are merged.

Run a task when everything in "Needs" is merged into `collision-monitor`. Tasks in the
same row group can run at the same time.

| Task | What | Needs | Runs in cloud | Status |
| --- | --- | --- | --- | --- |
| [T00](T00-cloud-setup.md) | Cloud environment: set-up script, what runs there | — | yes | merged 586f733 |
| [T01](T01-finish-node-fixes.md) | Verify and finish `claude/cm-node-fixes` | — | yes | merged 39ad5e1 |
| [T04](T04-nav2-spec.md) | Specification and design of the Nav2 extension (new file only) | — | yes | merged c2d5f85 |
| [T02](T02-finish-comparison.md) | Finish the comparison with upstream from the WIP branch; run it on Linux | T00, T01 | yes | merged 259589a |
| [T03](T03-comparison-defects.md) | Fix the defects the comparison found (negative clock, slow polygon path) | T02, T05b (both change the model; T05b changes the command path that item 2 measures) | yes | merged 91a7c3c |
| [T05](T05-nav2-model-proofs.md) | Extension: model events and proofs | T01, T04 | yes | merged add0540 |
| [T05b](T05b-transform-in-cpp.md) | Extension: move the transform out of the model (option A), rework the proofs | T05 | yes | merged 8bd13c8 |
| [T07](T07-nav2-harness.md) | Extension: upstream scenarios for lifecycle, transforms, footprint | T02, T04 | yes | merged 56ac73b |
| [T06](T06-nav2-node.md) | Extension: C interface and node (lifecycle, state topic, TF, footprint) | T03, T05b | yes | merged 173b52a |
| [T08](T08-nav2-review.md) | Independent review of the extension; Lean node through T07's scenarios | T06, T07 | yes | merged 363ce85 |
| [T09](T09-nav2-fixes.md) | Fixes from T08 | T08 | yes | merged f7fa7cf |
| [T10](T10-nav2-demo.md) | Demo: Nav2 launches the Lean node in place of its monitor | T09 | no (needs a display) | running |
| [T11](T11-writeup-website.md) | Website page on what is missing and what it takes; README; trusted base | T09 (T10 if done) | yes | running |
| [T12](T12-fresh-machine-audit.md) | Someone with nothing but the README builds and runs it | T11 | yes | open |

```
T00 ─┐
T01 ─┼─ T02 ── T03 ─┐
     │      └─ T07 ─┼─────┐
T04 ─┴─ T05 ────────┴ T06 ┴ T08 ── T09 ─┬─ T11 ── T12
                                        └─ T10 (local)
```

First wave: T00, T01 and T04 together.

### File ownership

So that parallel tasks do not collide.

| Files | Tasks that may change them |
| --- | --- |
| `core/ROSleanCore/CollisionMonitor*.lean`, `ROSlean/CollisionMonitor/` | T01, then T03, T05, T06 (FFI files only), T09 |
| `ros/roslean_core`, `ros/roslean_collision_monitor/{src,include,config}`, `smoke_*` tests, the package's `package.xml` and the non-harness part of its `CMakeLists.txt` | T01, then T03, T06, T09 |
| `ros/roslean_collision_monitor/test/` (harness), `bench/` | T02, then T07, T08 |
| `docs/collision_monitor/SPEC.md` | T01, T03, T05, T06, T09 |
| `docs/collision_monitor/SPEC_NAV2.md` | T04; later tasks correct it |
| `docs/collision_monitor/COMPARISON.md` | T02, T08 |
| `site/`, `README.md`, `docs/PLAN.md` | T11 |
| `scripts/cloud_setup.sh`, `tasks/README.md` (environment section) | T00 |

## Decisions

Taken:

- ROS 2 Jazzy. Integers in the Lean core. Match upstream; every difference is
  numbered in the SPEC.
- Scope is extended by pieces 1 to 3 (lifecycle interface and state topic;
  transforms from TF with base shift correction; the footprint as a zone).
  `approach`, `limit`, velocity and dynamic polygons other than the footprint,
  and other sensor types stay out.
- No stretch work (braking model, velocity smoother) before the above is
  finished.
- With pieces 1 to 3: the sensor pose comes from TF, `state_topic` is
  published, the node is a lifecycle node.

Decided by the user on 2026-11-24:

| Question | Decision |
| --- | --- |
| Stop on structurally invalid scans (fail-closed)? | As upstream: readings are filtered one by one |
| Publish zero for as long as a stop is demanded, not only for `stop_pub_timeout`? This removes the never-published stop (THEOREMS.md, "What the robot receives"). | As upstream. Why the theorem is weaker for it: `findings/stop_pub_timeout_theorem.md` |
| A timer that publishes zero when commands cease? | As upstream: none |
| Scan age by arrival time instead of header stamp? | As upstream: header stamp |
| Publish zero for a non-finite command? | As upstream: ignored |
| Refuse zones that protect nothing (no zones, radius 0, `source_timeout: 0`)? | Accept and warn |
| Apply the transform in C++ as upstream (bit-for-bit, larger trusted part) or in Lean in exact arithmetic (smaller trusted part)? | In C++, as upstream (option A). T05, already merged, built option B; T05b reworks the model and proofs, T06 writes the shim code. T02 measured B through the node at about 20 times upstream's time per command |
| With the transform in C++: compile the shim to match upstream's floating-point results bit for bit on each platform (upstream's arm64 binary fuses multiply-adds), or keep `-ffp-contract=off`? | Keep `-ffp-contract=off`: the same results on every platform, T01's test stands. Our points may differ from upstream's in the last bit on arm64 |
| Hysteresis against stop/slowdown flicker at a zone edge? | No |
| (T05b) A valid reading whose base-frame coordinate is not finite becomes a saturated point outside every zone. Keep, or stop? | Keep: as upstream and as before T05 |
| (T05b) Scan coordinates of the wrong count with `source_timeout: 0` let the command pass. Keep, or stop regardless? | Keep: consistent with N27; a shim that works as specified cannot produce it |
| (T05b) The model checks that a refreshed topic shape has as many vertices as before. Keep? | Keep |
| (T05b) After the move to C++ a command still costs more than before the extension (1080 readings: 207 µs for two circles against 18 µs; 628 µs for two polygons against 0.41 ms), because the conversion to millimetres now runs per command on patterns that are big numbers when the sign bit is set. Who fixes it? | T03, with its item 2: decode sign and magnitude apart, proved equal |
| (T07) After a backward clock jump upstream forgets all `/tf` transforms (observed); with base shift correction that means stops until new odometry arrives (inferred). The Lean node? | As upstream; listed among what the documentation does not say |
| (T07) The Linux container image lacks `nav2_bringup` and the lifecycle manager, so five footprint scenarios and half the bond test do not run there. Add them? | Yes, T08 adds them to `bench/cm_linux/Dockerfile` when it runs the Lean node through the scenarios |
| (T03) The no-clock step: a transition `stepNoClock` beside `step` (every existing theorem stands; the trace theorems do not speak about no-clock commands), or a constructor of `Event` (several theorems restated)? | Beside `step`, as T03 built it. T08 reviews whether the trace theorems should cover it |
| (T03) Without a usable clock reading the node answers every command with a stop, also with `source_timeout: 0` or a disabled source. | Keep: there is no "as upstream" here, upstream's process ends (D15) |
| (T03) `main` catches every exception that leaves a callback and goes on, with an error in the log. | Keep for now; T08 reviews it (an exception caught and ignored can hide a fault) |
| (T03) `roslean_cmd_cost` is a run-time threshold inside `test-quick`. | Keep it there; if it proves flaky on a loaded machine it gets `LABELS long` |
| (T06) Composition works (the node loads as a component), but as a component the node lacks the handling of a negative `/clock` (D15), which lives in `main`. Which form is supported? | The stand-alone executable is the supported and documented form; the component is offered with that limitation stated. T08 reviews whether the component should get the protection |
| (T06) The node waits for a missing transform in a loop of its own (N26), because T03's "no clock thread" and tf2's blocking wait do not fit under simulated time. Keep, or give rclcpp its clock thread back? | Keep; T08 reviews the loop against upstream's rule |
| (T06) The old scenario profile does not compare state messages for the Lean node (`STATE_TOPIC = False`). | T08 decides in its review whether the 32 differing checks are stated differences or defects, and turns the comparison on where it can |
| (T06) The component's plugin name is `roslean_collision_monitor::CollisionMonitorNode`; SPEC_NAV2.md proposed `…::CollisionMonitor`. | Keep the name that is built; T09 corrects the document |
| (T06) `test-quick` takes 6 minutes with `smoke_component` (9 s) and `roslean_nav2_bond` (21 s) in it. | Leave them in |
| (T08, finding 2) Under simulated time a backward clock jump during the wait for a transform hangs the node until the clock is back. Add tf2's rule to the node's loop, or give rclcpp its clock thread back and call tf2? | Add tf2's rule (give up when the clock jumps back) to the loop, and clear the TF buffer as upstream's clock thread does |
| (T08, finding 3) Unrecorded differences: disabled visualised zones are published; unusable `points` with `polygon_sub_topic`, an empty `polygon_sub_topic` and a wrongly typed `slowdown_ratio` on a stop zone are refused where upstream configures. | Follow upstream in each (publish only enabled zones; fall back to the topic; ignore the unused parameter). 3d, 3e, 3f: correct the document or the code as the finding says |
| (T08, finding 4) After a processed no-clock command no theorem stated from start-up applies. Take the review's `mixed_from_init` into `NoClock.lean`, or make the no-clock command an `Event`? | Take `mixed_from_init` in and correct THEOREMS.md |
| (T08, finding 5) `lean_initialize_thread` is never called. | Call it where the runtime requires it (the component's executor threads); say in the SPEC what was checked |
| (T08) The component dies with its container on a negative `/clock`, as upstream's does. | "As upstream" for the component; D15 is promised for the executable only, and the documentation says so |
| (T08) State messages in the old scenario suite are not compared for the Lean node. | Leave it to `scenario_nav2.py` and `random_nav2.py`; T09 adds boundary cases for state messages there if cheap |
| (T09) The TF buffer has a clock of its own and the node clears it at a backward clock jump (trusted node code instead of tf2's hook). | Keep; it matches upstream in every scenario |
| (T02) Negative `/clock`: fix it (T03 item 1) or accept "as upstream"? | Fix, as T03 says: no topic content may end the node |
| Keep the extra 1 mm on circle radii? | Keep |
| (T04) Should a stop zone fed by a topic stop the robot while it has no shape yet? | As upstream: no action |
| (T04) A footprint zone fed from odom is left behind when its publisher stops: accept, give shapes a maximum age, or document only the fixed polygon? | Accept, as upstream; document the fixed polygon as the recommended edit |
| (T04) Does `source_timeout: 0` also disable the stop for a missing transform? | As upstream: yes |
| (T04) Reproduce upstream's wait for a transform inside the command callback? | Yes |
| (T04) After shutdown from the active state: silent (ours, ND4) or keep forwarding (upstream)? | Silent |
| (T04) Stand-alone process if the node cannot be loaded into Nav2's component container? | Acceptable; T06 tries composition first |
| (T05) Should `shutdown` also clear `prevZero` and `prevReport`, so that the lifecycle invariant holds in the finalized phase too? Nothing observable depends on it. | No: as SPEC_NAV2 E5 |
| (T05) `test_cm_ffi` now takes 169 s (its leak test places a scan 650,000 times through the exact transform). | T06 reworks the C interface: it reduces the rounds or moves the leak test under `LABELS long`, so that `test-quick` stays short |
| (T04) Which Nav2 parameter edit to document? | The footprint as a fixed polygon; the two-line `polygon_sub_topic` change is mentioned |

Open or not confirmed (the rows are unchanged; they stood in the table above until
task T11 grouped them). Each is built as its row says until the user decides:

| Question | Decision |
| --- | --- |
| (T08, finding 1) A polygon message with a non-finite or out-of-range vertex: upstream accepts it and its zone can still stop; our node refuses the message (ND6) and can pass where upstream stops. Stop until a usable message arrives, or reproduce upstream's NaN arithmetic? | Stop: after such a message the zone demands its action (a stop zone stops) until a usable message arrives. Never less restrictive than upstream; a numbered difference. **Default chosen by the orchestrator; the user has not confirmed it** |
| (T09) A zone marked unusable does its own action: a marked slowdown zone slows, it does not stop. Should a marked zone always stop? | No: its own action. **Not confirmed by the user** |
| (T09) ND13: a refused refresh of a topic zone in another frame leaves the zone where it was; the one place left where the node can in principle be less restrictive than upstream (points about a kilometre from the base). Should it also make the zone demand its action? | Left as built for now. **Open for the user; the orchestrator recommends yes** |
| (T02, T03) The cost of a command: target? | T03's budget until the user says otherwise: 50 / 150 / 1500 µs per command at 360 / 1080 / 10,000 stored readings and two zones, in process. Met on macOS and in the Linux arm64 container (at most 45 % of it). The command callback is still 4 to 8 times upstream's |
| (T02) A run on Linux on its own hardware or on x86-64 before the write-up? | Not decided. Linux so far is an arm64 container on the Mac |

## Watching the tasks

A task session can stop without saying so: a test that never returns, a
process that waits for input, an agent that waits for a build that died.
Whoever orchestrates keeps a check running for as long as any task is
running. In the orchestrating Claude Code session, in the main checkout:

```
/loop 15m Run tasks/watch.sh and act on it as tasks/PLAN.md "Watching the tasks" says.
```

`tasks/watch.sh` looks at every worktree on a task branch and prints, for
each, when a file last changed (sources, colcon logs, the Lean build, git),
what is running there, and a verdict. What the loop does with it:

| Verdict | Meaning | Action |
| --- | --- | --- |
| `active` | A file changed in the last 20 minutes, or its processes use the processor. | None. Check that the newest file is a different one from the last round, or that the same test has moved on (its log grows); the same file and the same process for three rounds running is treated as `waiting`. |
| `waiting` | Nothing changed, but processes exist. | Look at the process list: a test with a timeout of its own (the `TIMEOUT` in the CMakeLists) may wait; anything older than its timeout, or a process that waits for a terminal, is killed by PID, and the task session is told what was killed and why. |
| `STUCK` | Nothing changed for 20 minutes and nothing runs. | Ask the task session for its state. If it does not answer or cannot say what it is waiting for, the orchestrator takes the task back: commits what is in the worktree as a WIP commit on the task's branch, records the state in the status column below, and either restarts the task from that branch with a note on where it stopped, or finishes it itself. |
| `done` | Head pushed, tree clean, nothing running. | Read the report, rebuild, merge (README, "Merging"). |

The loop also asks, every round: has each running task moved towards what
its task file asks for since the last round (a commit, a new section of its
deliverable, a build or test further along)? A task that is busy but has
shown nothing new for an hour is asked what it is doing, and is taken back
like a `STUCK` one if the answer is not a concrete next step. The loop ends
when no task is running.

So that the check has something to see, every task session, in addition to
[README.md](README.md):

- appends one line to `tasks/reports/T??.progress` (time, what was just
  finished, what comes next) whenever it finishes a step of its task file
  and before it starts anything that takes more than five minutes;
- commits to its branch after each finished step (work in progress is
  fine; the last commit is the one that counts) and pushes;
- starts long commands in the background with a time limit of its own
  (`timeout`, or the tool's limit), and reads their log when they end rather
  than waiting without a limit;
- gives up on a step after three failed attempts at the same thing, writes
  down what it tried, and goes on with the next step or, if nothing else can
  be done, finishes with the report: an honest "not done, because" is a
  result, a session that sits is not.

## After each merge

Update the status column above (`open`, `running`, `branch pushed`,
`merged <commit>`), and move anything a report raised for the user into
"Decisions".
