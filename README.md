# ROSlean

ROS 2 nodes whose decision logic is written and proved in Lean. Two nodes so
far:

- [`twist_mux`](https://index.ros.org/p/twist_mux/), which picks one of
  several velocity command sources by priority, freshness and locks. A
  drop-in replacement for the upstream node.
- The stop and slowdown core of Nav2's
  [`nav2_collision_monitor`](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_collision_monitor),
  which passes a velocity command on, scales it down or replaces it by zero,
  depending on what a laser scan shows in configured zones. Nav2's lifecycle
  manager can launch it in place of its own monitor. It is **not** a drop-in
  replacement for Nav2's default configuration, which uses the `approach`
  action; see [The collision monitor](#the-collision-monitor).

Project page: <https://beneficial-ai-foundation.github.io/ROSlean/> (source in
[`site/`](site), deployed by the `pages.yml` workflow on pushes to `main`).

Documents, twist_mux:

- [twist_mux.md](twist_mux.md): scope, and where to start as a new reader.
- [docs/SPEC.md](docs/SPEC.md): the rules, read from the upstream C++ source,
  and every deliberate difference.
- [docs/THEOREMS.md](docs/THEOREMS.md): the main theorems in plain words,
  with links to the statements.
- [docs/TUTORIAL.md](docs/TUTORIAL.md): from an empty machine to the
  turtlesim experiment, step by step.
- [docs/TUTORIAL_GAZEBO.md](docs/TUTORIAL_GAZEBO.md): the same experiment
  on a TurtleBot3 in a furnished apartment in Gazebo, with Nav2.
- [docs/BENCHMARK.md](docs/BENCHMARK.md): run-time comparison with upstream
  `twist_mux`, and what it means on a robot.

Documents, collision monitor:

- [docs/collision_monitor/SPEC.md](docs/collision_monitor/SPEC.md): upstream's
  rules (R1–R19, N1–N36), every difference (D1–D15, ND1–ND13), what is
  trusted (T1–T10), the assumptions (A1–A4).
- [docs/collision_monitor/THEOREMS.md](docs/collision_monitor/THEOREMS.md):
  the theorems in plain words, and what is not proved.
- [docs/collision_monitor/UPSTREAM_BEHAVIOUR.md](docs/collision_monitor/UPSTREAM_BEHAVIOUR.md):
  what upstream's binary does, found by black-box tests.
- [docs/collision_monitor/COMPARISON.md](docs/collision_monitor/COMPARISON.md):
  the Lean node beside upstream: scenarios, random cases, fault injection,
  run time.
- [docs/collision_monitor/SPEC_NAV2.md](docs/collision_monitor/SPEC_NAV2.md):
  the design record of the Nav2 extension; section 8.6 is what a user
  changes to run the node in Nav2, section 12 what still separates it from
  Nav2's default configuration.
- [docs/TUTORIAL_COLLISION_MONITOR.md](docs/TUTORIAL_COLLISION_MONITOR.md):
  the Gazebo demo.
- [site/gaps.html](site/gaps.html) (on the project page: "What is
  missing"): every gap, what the node does there today, and what closing it
  would take.

Both:

- [docs/PLAN.md](docs/PLAN.md): development plan, status, what is not done,
  lessons.
- [docs/REVIEWS.md](docs/REVIEWS.md): the independent reviews and what was
  done about each finding.
- [tasks/PLAN.md](tasks/PLAN.md): the task list of the collision monitor
  work and the decisions, with who took each.

## Layout

| Path | Contents |
| --- | --- |
| `core/` | A Lean package with no dependencies. `core/ROSleanCore/` holds the executable models (`Mux.lean`; `CollisionMonitor.lean` with `CollisionMonitorPoint.lean`, `CollisionMonitorConvert.lean`, `CollisionMonitorFast.lean`) and their C interfaces (`FFI.lean`, `CollisionMonitorFFI.lean`). Compiled to C and linked into the library that the nodes load. |
| `ROSlean/` | Lean + Mathlib (the root package, which requires `core`). Theorems about `ROSleanCore`, in `ROSlean/Mux/` and `ROSlean/CollisionMonitor/`. Never linked into a node. |
| `ros/roslean_core` | Compiles the C that Lean generates into `libroslean_core` and installs the Lean runtime it needs with it; C++ wrappers, FFI tests. |
| `ros/roslean_twist_mux` | The mux node. A C++ shim with no arbitration logic: it reads parameters, subscribes, reads the clock and publishes. Its `demo/` holds the turtlesim and Gazebo demos. |
| `ros/roslean_collision_monitor` | The collision monitor node, as an executable and as a component. A C++ shim: lifecycle, subscriptions, TF lookups and the transform of scan points (`include/…/transforms.hpp`), publishing. `test/` holds the harness that runs it beside upstream; `demo/gazebo/` the demo. |
| `ros/roslean_noop` | Minimal node that calls a Lean function; the original pipeline check. |
| `docker/` | Linux build and run-time image, both nodes. |
| `bench/` | Benchmarks against upstream (`pixi run bench`, `pixi run bench-cm`), their results, and the Linux image in which the collision monitor comparison was run (`bench/cm_linux/`). Not part of the normal build. |
| `review/` | Scripts and Lean files of the independent reviews of the collision monitor. |
| `findings/` | Notes on single questions. `stop_pub_timeout_theorem.md`: why a demanded stop is not always published. |
| `tasks/` | The briefs, plan and reports of the sessions that built the collision monitor. |
| `site/`, `video/` | The project page, and the scripts that made its video. |

## Quick start

Requires [elan](https://github.com/leanprover/elan) and
[pixi](https://pixi.sh), both on `PATH`. The ROS 2 Jazzy environment
(RoboStack, with Nav2 and Gazebo; about 6 GB) is installed into `.pixi/` on
first use, and the first `pixi run prove` also downloads Mathlib.

```sh
pixi run build        # lake build of the core + colcon build of the ROS packages
pixi run test-quick   # the tests without the long comparisons with upstream; about 7 minutes
pixi run test         # all tests; about 20 minutes before the collision monitor's Nav2 extension, longer now
pixi run prove        # check all theorems (first run fetches or builds Mathlib)
pixi run demo         # turtlesim driven through the Lean-backed mux
```

What has been run where:

- **macOS arm64:** everything.
- **Linux x86-64** (Ubuntu 24.04, through pixi): twist_mux: `pixi run build`
  and `pixi run test` pass and the turtlesim demo runs. The collision
  monitor: one cloud session ran build, all tests then present (11 tests,
  0 failures), `lake build` of all theorems then present, and the headless
  Gazebo check, on the node as it was **before** it became a lifecycle node
  with transforms from TF. The node as it is now has not been run on x86-64.
- **Linux arm64:** a container in Docker on the same Mac, for the collision
  monitor's comparison with upstream (`bench/cm_linux/`) and the Docker
  build below. Not Linux on its own hardware.

Run the mux the same way as upstream:

```sh
pixi run bash -c 'source install/setup.bash && ros2 run roslean_twist_mux twist_mux \
  --ros-args --params-file ros/roslean_twist_mux/config/example.yaml'
```

Run the collision monitor with a lifecycle manager that configures and
activates it, as Nav2 does for upstream's:

```sh
pixi run bash -c 'source install/setup.bash && ros2 launch roslean_collision_monitor \
  collision_monitor_launch.py params_file:=ros/roslean_collision_monitor/config/example.yaml'
```

or by hand (it is a lifecycle node and does nothing until activated):

```sh
ros2 run roslean_collision_monitor collision_monitor --ros-args \
  --params-file ros/roslean_collision_monitor/config/example.yaml
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate
```

## Linux and deployment

A Lean installation is needed to build, not to run, and building the nodes
never needs Mathlib. `roslean_core` installs `libroslean_core` together with
the Lean runtime it uses, so the install tree does not refer to the Lean
toolchain or to this checkout. Built normally (without `--symlink-install`,
which `pixi run build` uses for development), it can be copied to another
machine of the same OS and architecture that has ROS 2 Jazzy.

- **macOS:** the runtime is linked statically into `libroslean_core` (4 MB).
- **Linux:** Lean's static runtime cannot be put in a shared library, so a
  stripped copy of Lean's shared runtime, `libleanshared.so` (about 190 MB),
  is installed next to `libroslean_core.so`.

```sh
docker build -f docker/Dockerfile -t roslean-twist-mux .
docker run --rm --network host roslean-twist-mux                # example config
docker run --rm --network host -v $PWD/my.yaml:/cfg.yaml roslean-twist-mux \
  ros2 run roslean_twist_mux twist_mux --ros-args --params-file /cfg.yaml
```

The image is built in two stages on the official `ros:jazzy-ros-base`
(Ubuntu 24.04, ROS from apt). The first installs Lean, builds the packages,
runs the tests including the comparison with apt's `ros-jazzy-twist-mux`, and
checks that the install tree is self-contained; the build fails if any of
that fails. The final image holds ROS and the install tree only.
`--build-arg RUN_TESTS=0` skips the tests. The collision monitor is in the
same image. With the collision monitor in its present form the image has
been built on arm64 with `RUN_TESTS=0` only; its tests have not been run in
this image since the Nav2 extension.

Without Docker, on a Linux machine with ROS 2 Jazzy and elan installed:
`colcon build --base-paths ros` in this directory, which is what the
Dockerfile does.

## Demo

A turtlesim turtle driven through the mux by three sources: a scripted
planner (priority 10), keyboard teleop (priority 100) and an e-stop lock
(priority 255).

```sh
pixi run demo            # terminal 1: turtlesim, mux, planner, narrator
pixi run demo-teleop     # terminal 2: keyboard teleop; hold a key to keep driving
pixi run demo-estop      # terminal 3: engage the e-stop lock
pixi run demo-release    #             release it
```

The turtle circles under the planner. Holding a teleop key takes over; half
a second after the key is released the planner resumes. Engaging the e-stop
freezes the turtle whatever else is publishing. The narrator in terminal 1
prints each change together with the theorem that explains it, for example:

```
>>> teleop drives; planner messages are dropped while teleop is live
    theorems forward_of_highest_live, run_not_forward_of_higher_live
>>> e-stop lock ENGAGED
>>> nothing is forwarded: the e-stop lock (255) is above every input
    theorem not_forward_of_top_lock
```

There is also a Gazebo version: a TurtleBot3 in a furnished apartment, with
either a patrol script (`pixi run gazebo`) or Nav2 (`pixi run gazebo-nav2`)
as the planner, `pixi run gazebo-teleop` for the keyboard, and the same
e-stop commands. See [docs/TUTORIAL_GAZEBO.md](docs/TUTORIAL_GAZEBO.md).

To see the same demo on upstream's node, pass `mux_package:=twist_mux` to the
launch file (the theorem names then describe behaviour that upstream shares,
not something proved about it).

**The collision monitor in Gazebo.** The same TurtleBot3 with the mux and
the collision monitor in series: a script drops a box in front of the
robot, the robot slows down, stops, and drives on when the box is removed.

```sh
pixi run cm-gazebo-scenario   # with a window
pixi run cm-check             # without windows, checked against the simulator's ground truth
pixi run cm-check --monitor nav2_collision_monitor   # the same with upstream's node
```

See [docs/TUTORIAL_COLLISION_MONITOR.md](docs/TUTORIAL_COLLISION_MONITOR.md),
which also covers Nav2 as the planner and Nav2 launching the Lean node in
place of its own monitor (section 7; `pixi run cm-gazebo-bringup`,
`pixi run cm-check --nav2`). The check passed in nine runs of nine on macOS
with an earlier version of the node (the robot at rest 0.255 to 0.268 m from
the box, 0 to 14 mm after the first zero command) and in three runs of three
with the node as a lifecycle node that reads TF (at rest 0.254 to 0.260 m,
0 to 2 mm). It shows the rule at work in one head-on approach in simulation;
it is not evidence that a robot does not collide.

In the variant in which Nav2's own bring-up starts the monitor
(`cm-check --nav2`, a stop and a slowdown zone), the same check passed in
ten runs of ten with the Lean node and in four of four with upstream's node
on the same parameters (at rest 0.269 to 0.275 m and 0.267 to 0.270 m from
the box). One start before these, the first in a freshly installed
environment, failed: Nav2's lifecycle manager did not get past
`behavior_server` and the monitor was never activated. With Nav2 itself
driving a tour through these zones the outcome was different: in three runs
of six the robot ended standing for good. See [Under Nav2](#under-nav2).

## twist_mux: what is proved

All in `ROSlean/Mux/`, with no `sorry` and only Lean's three standard axioms.

| Property | Theorem | Statement |
| --- | --- | --- |
| Decision | `forward_iff` | A message on input *i* is forwarded iff *i* is configured with positive priority, no locked lock has higher priority, and every other live input has strictly lower priority, or the same priority and a later position in the input list. |
| Provenance | `forward_configured` | Only messages on configured inputs are forwarded. The message body never enters Lean, so it cannot be altered there. |
| Priority | `forward_priority`, `forward_first_of_equal` | No live input has higher priority than a forwarded one; among live inputs of equal priority only the first in list order is forwarded. |
| Lock | `forward_lock`, `not_forward_of_top_lock` | A forwarded input is not outranked by a locked lock; a lock strictly above all inputs freezes everything. |
| Dead lock publisher | `not_forward_of_silent_lock` | A lock whose publisher is silent past its timeout blocks lower inputs. |
| Expiry | `forward_of_others_expired`, `inputLive_iff_le` | An expired input never suppresses another; an input is live up to and including `stamp + timeout`. |
| Liveness | `forward_of_highest_live` | The highest live input that no lock outranks is forwarded (given positive priority). |
| Trace invariant | `run_forward_iff`, `run_forward_iff_history` | The above holds at every point of every event sequence; the decision is characterised by the event history alone. |
| Code that runs | `muxForward_after` (in `FFI.lean`) | The decision the node reads from the C interface is the model's, on exactly the handles the node configured. |

What is **not** proved, and is trusted or only tested:

- **That the model matches upstream `twist_mux`.** The model is a reading of
  the C++ source, checked by running both nodes on the same event sequences
  (fixed scenarios and a randomised differential test). The one known
  difference in decisions is at exact timeout boundaries (SPEC, D2).
- **The C++ shim** (`twist_mux_node.cpp`, `roslean_core/mux.hpp`): mapping
  subscriptions to indices, reading the clock, converting timeouts from
  seconds to nanoseconds, reference counting of Lean objects, calling the
  core once per message in order, and publishing on a `true` decision.
- **The declared C signatures.** They are checked against Lean's generated C
  at build time, except that Lean passes `Int64` as `uint64_t`, so signedness
  is not checked.
- **One event at a time.** The node uses a single-threaded executor; the
  theorems say nothing about concurrent callbacks.
- **The clock.** Theorems are about recorded clock readings. If the clock
  jumps backwards (a simulation reset), expired inputs and dead lock
  publishers count as fresh again until the clock catches up, as in upstream.
- **The order of inputs.** Ties between equal priorities go to the earlier
  input. The theorems take the order as given; the shim gets it from
  `list_parameters` (ASCII order of the handle names), as upstream does.
- The Lean compiler and runtime, and ROS itself.

Two things the theorems make precise that are easy to get wrong:

- A lock blocks only inputs of *strictly* lower priority, and priorities are
  clamped to 255. An input at 255 can never be blocked. Give an e-stop lock a
  priority above every input.
- When all inputs are silent the mux publishes nothing; it does not send a
  stop command. Stopping is the base driver's job, as with upstream.

## The collision monitor

`roslean_collision_monitor` covers this much of `nav2_collision_monitor`
(Jazzy): one laser scan source; circle and polygon zones with fixed shapes,
and polygon zones whose shape arrives on a topic; the actions `stop` and
`slowdown`; as a lifecycle node with a bond, the state topic and the zone
polygons, with transforms from TF and base shift correction. Anything else
in a parameter file (`approach`, `limit`, velocity polygons, point cloud,
range and polygon sources, several sources) fails the `configure`
transition with one message that names every refused parameter.

The claim is about commands, not physics: which velocity the node publishes
for the messages it received. Whether a robot then avoids an obstacle
depends also on the sensor, the zone sizes, the transforms and the base.

### What is proved

All in `ROSlean/CollisionMonitor/`, with no `sorry`, no `native_decide` and
only Lean's three standard axioms. "Command" means a velocity command that
arrives while the node is active. The full list, with what each statement
assumes, is [docs/collision_monitor/THEOREMS.md](docs/collision_monitor/THEOREMS.md).

| Property | Theorem | Statement |
| --- | --- | --- |
| What is answered | `processed_iff` | An event produces output iff it is a command, the node is active, and the command has no infinite or NaN component. |
| Decision | `evaluate_eq_stopSource_iff`, `evaluate_eq_stopZone_iff`, `evaluate_eq_slowdown_iff`, `evaluate_eq_pass_iff` | Four if-and-only-if statements, one per action: stop for an invalid source; stop by the first triggered stop zone; slowdown by the triggered slowdown zone with the smallest ratio, only if no stop zone is triggered; pass otherwise. |
| What is published | `velocity_eq_some_iff` | The demanded action is published unless it yields zero, the previous one did too, and `stop_pub_timeout` has passed since the zero first appeared; then nothing is published. |
| Stop | `stop`, `run_stop` | With a triggered stop zone the node publishes zero or nothing, never a non-zero velocity; after any sequence of events since start-up. |
| Stale data, no transform | `stale`, `no_transform`, `run_stale`, `run_no_transform` | With a non-zero `source_timeout`: no scan, a scan older than the timeout, or no usable transform for it gives the stop for an invalid source. |
| No amplification | `no_amplification`, `run_no_amplification`, `published_ratio_in_unit_interval` | Every published action is pass, a stop, or a slowdown whose ratio is the configured one of the zone named, with a value in [0, 1). |
| Slowdown bound | `slowdown_bound` | The ratio applied is at most that of every triggered slowdown zone. |
| Pass-through | `pass_through`, `run_pass_through` | With a valid source and no zone triggered, a non-zero command is published unchanged. |
| Monotonicity | `zonesAction_stop_mono`, `evaluate_isStop_mono` | Adding points never lifts a stop. |
| Bad data | `output_filter_validBeam`, `step_cmd_of_not_finite` | Readings that are NaN or outside the range limits never change a decision; a non-finite command is ignored and leaves the state unchanged. |
| Lifecycle | `output_of_not_active`, `run_silent_before_activate`, `run_silent_after_shutdown` | Nothing is published while the node is not active, nor after a shutdown. |
| After the zero period | `run_withheld_last_published_zero` | In any run from start-up, when a velocity is withheld, the last velocity this node published was a zero. |
| Unusable polygon message | `evaluate_isStop_of_unusable`, `run_stop_after_unusable` | After a polygon message with a non-finite or out-of-range vertex, an enabled stop zone fed by that topic stops every command until a usable message arrives (ND6). |
| Without a clock | `noClock_velocity` | A finite command that arrives while the node has no valid clock reading is answered with zero, whatever the configuration (D15). |
| Rounding errs towards "inside" | `circle_safe`, `nearBoundary_of_rounded`, `contains_of_shim` | A point strictly inside a configured circle is inside the model's integer circle. For polygons the quantitative half of the same claim is proved; see below. |
| Code that runs | `cm_readBack_after_fast`, `stepCmdFast_eq`, `cm_not_ok_after` | The five numbers the node reads from the C interface after any sequence of calls determine the model's whole output for the command; the fast command path that C calls is the same function as the model's `step`; a refused configuration stops every command. |

### What is not proved, and is trusted or only tested

The trusted parts are numbered T1 to T10 in
[SPEC.md](docs/collision_monitor/SPEC.md), "Where the velocity is computed".

- **That a demanded stop is published.** It is not always, as upstream's is
  not: zero is published for `stop_pub_timeout` and then nothing, whatever
  the cause of the zero (R15). The theorems say "zero or nothing" (`stop`)
  and "the last velocity published was a zero"
  (`run_withheld_last_published_zero`). That keeps the robot stopped only if
  the base holds its last command and nothing else commands it (assumption
  A3). The user decided to keep upstream's behaviour;
  [findings/stop_pub_timeout_theorem.md](findings/stop_pub_timeout_theorem.md)
  has the sequence and what would make the theorem strong.
- **That the model matches upstream.** Read from the source and tested
  against the binary. On the final node on macOS: 318 checks in 73 scenario
  runs and 289 checks in 44 runs of the Nav2 extension, 0 failures, every
  difference a numbered one; 300 random cases with 26,951 commands, none
  outside the stated bounds, the Lean node the more restrictive one in each
  of the 586 commands that differed; 300 random cases of the extension with
  16,797 commands, no disagreement. In the Linux arm64 container: the 289
  checks and 20 random cases. Details and the earlier, larger runs:
  [COMPARISON.md](docs/collision_monitor/COMPARISON.md).
- **The slowdown multiplication** (T1). The core returns the ratio's bit
  pattern; the shim multiplies in `double`. That the product is no larger
  than the command is IEEE-754's, not a Lean theorem.
- **The points of a scan** (T2, T6, T7). The shim computes each scan point,
  looks the transform up in TF and applies it in `double`
  (`transforms.hpp`), as upstream does; the user decided this over an exact
  transform in Lean, which was built and cost about 20 times upstream's time
  per command. The theorems hold for whatever coordinates a command carries;
  that they are the right ones is tested (`smoke_transforms`, the `tf-*` and
  `shift-*` scenarios), not proved.
- **The topological half of the polygon margin argument.** That a point
  inside the configured polygon which ends up outside the rounded one was
  passed over by the boundary is prose. Tested: 34.0 million points against
  upstream's zone tests (`band_bounds`).
- **The IEEE-754 reading of bit patterns** (T4): the formula for the number
  a pattern denotes is written in this repository and compared with the
  hardware on some 300,000 patterns per helper, not derived from a
  formalisation of the standard.
- **Timeouts.** Which values are refused is proved; that an accepted value
  is upstream's nanosecond count is tested on 530,000 values (T3).
- **The C++ shim** otherwise (T3, T5, T8, T9): one event per message in
  order, parameters passed unaltered, publishing what the output says, zone
  names, when the clock is valid.
- **Threads** (T10): one event at a time. As a component the node runs in
  its container's threads; that no two are in the Lean core at once is
  ensured by a callback group and checked by no test.
- **That an accepted configuration protects anything.** The node warns
  about, and accepts, a source timeout of 0, a disabled source, no zones, a
  circle of radius 0.
- Lean's compiler and runtime, tf2, rclcpp, the middleware.

Where the node is fail-open, as upstream is, by the user's decision: a zone
fed by a topic takes no action until its first message; `source_timeout: 0`
switches off the stop for an old or missing scan and for a missing
transform; a scan with no valid reading, or with unusable range limits,
reads as free space; without incoming commands the node publishes nothing.

Where it differs from upstream it is the more restrictive one (zones grown
by up to 3.5 mm for a circle and 4.4 mm for a polygon, D4; ND6; D15; refused
parameter values), with one exception in principle: ND13, a zone in another
frame whose recomputed vertices lie about a kilometre from the base stays
where it was. One difference is a delay, not a decision: ND3.

### Under Nav2

`nav2_bringup`'s `nav2_params.yaml` gives the monitor one zone,
`FootprintApproach`, with `action_type: approach`. The Lean node refuses it:

```
2 parameters refused: parameter 'FootprintApproach.action_type': only 'stop' and 'slowdown' are supported; got 'approach'. (…); parameter 'FootprintApproach.polygon_sub_topic': is not initialized, and the zone has no 'points': (…)
```

Changing only `action_type` to `stop` does not configure either, on
upstream as on this node: a stop zone does not read `footprint_topic`. The
documented edit is the footprint as a fixed stop polygon (the coordinates
are an example; choose them for your robot):

```yaml
polygons: ["FootprintStop"]
FootprintStop:
  type: "polygon"
  points: "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"
  action_type: "stop"
  min_points: 6
```

and, in a copy of Nav2's `navigation_launch.py`,
`package='roslean_collision_monitor'` in place of
`package='nav2_collision_monitor'` (the executable is `collision_monitor`
in both). Start Nav2 with `use_composition:=False`, or see
[SPEC_NAV2.md, section 8.6](docs/collision_monitor/SPEC_NAV2.md) for the
component and for the second form of the edit (`polygon_sub_topic`).

The stand-alone executable is the supported form. The component
(`roslean_collision_monitor::CollisionMonitorNode`) loads and runs in the
three container kinds of `rclcpp_components`, but lacks the protection
against a `/clock` message with a negative time (D15), which lives in the
executable's `main`.

With this edit Nav2's lifecycle manager configured, activated and bonded
the node, and it stopped and released a command (review CM5; macOS,
scripted scan, no simulator).

With a simulated robot (Gazebo, TurtleBot3, macOS;
[docs/TUTORIAL_COLLISION_MONITOR.md](docs/TUTORIAL_COLLISION_MONITOR.md),
section 7): Nav2's bring-up started the Lean node as its collision monitor,
with a stop and a slowdown zone in place of `FootprintApproach`. The
automated check of the box scenario (`pixi run cm-check --nav2`) passed in
ten runs of ten, and in four of four with upstream's node on the same
parameters. The first start of all, in a freshly installed environment,
failed before the monitor was activated: Nav2's lifecycle manager stopped at
`Configuring behavior_server`. That did not recur in 27 further starts.

**A stop zone is not a substitute for `approach`.** The robot stops where
stock Nav2 slows down in proportion to the time before a predicted
collision, and in a stop zone every command becomes zero, including the turn
or the retreat that would clear the zone, and including Nav2's recovery
behaviours. With Nav2 driving a tour of the apartment for 300 simulated
seconds through these zones, **the robot ended standing for good in three
runs of six**: in two of three runs with the Lean node (1, 1 and 8 places
reached) and in one of three with upstream's node on the same zones (8, 2
and 5 places). Stock Nav2 with `approach` reached 11 places in its one run
and did not stand. Three runs each do not compare the two nodes; the
standing is what a `stop` zone does under a planner, with either node.

### Cost and platforms

In process, on one Apple M5 Pro (macOS arm64), a command costs 6 to 9 times
upstream's: 48.9 µs against 7.2 µs with 1080 stored readings and two polygon
zones. In the Linux arm64 container an earlier version of the command path
measured 4.2 to 6.4 times upstream's. Both are inside the budget set in
this project (50, 150 and 1500 µs at 360, 1080 and 10,000 readings), which
the test `roslean_cmd_cost` asserts for the Lean core. Nothing was timed on
x86-64, on Linux on its own hardware, or on a robot computer.
