# ROSlean

ROSlean implements decision logic for two ROS 2 Jazzy nodes in Lean.
Lean checks the rules for selecting or changing velocity commands. ROS,
the C++ integration, and the robot hardware are outside those proofs.

- **`roslean_twist_mux`** selects a velocity source by priority, timeout, and locks. It accepts the upstream YAML format and uses the same command topics. See the [compatibility differences](docs/SPEC.md#deliberate-differences-in-roslean).
- **`roslean_collision_monitor`** supports one lidar, circle and polygon zones, and the `stop` and `slowdown` actions. To use it with Nav2, [edit the launch file and replace the default `approach` zone](#under-nav2).

**IMPORTANT** This repo is in an exploratory, still very unstable phase.

| | twist_mux | Collision monitor |
| --- | --- | --- |
| Run a demo | [Turtlesim](docs/TUTORIAL.md), [Gazebo](docs/TUTORIAL_GAZEBO.md) | [Gazebo and Nav2](docs/TUTORIAL_COLLISION_MONITOR.md) |
| Read the rules | [Specification](docs/SPEC.md) | [Specification](docs/collision_monitor/SPEC.md), [Nav2 integration](docs/collision_monitor/SPEC_NAV2.md) |
| Read the proofs | [Theorems](docs/THEOREMS.md) | [Theorems](docs/collision_monitor/THEOREMS.md) |
| Replace the upstream node | [Drop-in guide](docs/DROP_IN_TWIST_MUX.md), [the same for a coding agent](docs/DROP_IN_TWIST_MUX_AGENTS.md) | |
| Compare with upstream | [Benchmarks](docs/BENCHMARK.md) | [Tests and benchmarks](docs/collision_monitor/COMPARISON.md), [upstream behavior](docs/collision_monitor/UPSTREAM_BEHAVIOUR.md) |
| Use it in place of upstream | | [What to change, how to check, how to go back](docs/DROP_IN_COLLISION_MONITOR.md); [the same as steps for a coding agent](docs/DROP_IN_COLLISION_MONITOR_AGENTS.md). Only for `stop` and `slowdown` zones on one laser scan: Nav2's default configuration is refused |

See the [project site](https://beneficial-ai-foundation.github.io/ROSlean/),
[development plan](docs/PLAN.md), and [review findings](docs/REVIEWS.md).
GitHub Pages publishes [`site/`](site/) on pushes to `main` that change the
site or its workflow.

## Layout

| Path | Contents |
| --- | --- |
| `core/` | A Lean package with no dependencies. `core/ROSleanCore/` holds the executable models (`Mux.lean`; `CollisionMonitor.lean` with `CollisionMonitorPoint.lean`, `CollisionMonitorConvert.lean`, `CollisionMonitorFast.lean`) and their C interfaces (`FFI.lean`, `CollisionMonitorFFI.lean`). Compiled to C and linked into the library that the nodes load. |
| `ROSlean/` | Lean + Mathlib (the root package, which requires `core`). Theorems about `ROSleanCore`, in `ROSlean/Mux/` and `ROSlean/CollisionMonitor/`. Never linked into a node. |
| `ros/roslean_core` | Compiles the C that Lean generates into `libroslean_core` and installs the Lean runtime it needs with it; C++ wrappers, FFI tests. |
| `ros/roslean_twist_mux` | The mux node. A C++ shim with no arbitration logic: it reads parameters, subscribes, reads the clock and publishes. Its `demo/` holds the turtlesim and Gazebo demos. |
| `ros/roslean_collision_monitor` | The collision monitor node, as an executable and as a component. A C++ shim: lifecycle, subscriptions, TF lookups and the transform of scan points (`include/…/transforms.hpp`), publishing. `test/` holds the harness that runs it beside upstream; `demo/gazebo/` the demo. |
| `ros/roslean_nav2_bringup` | Launch files and parameter files that start Nav2 with the collision monitor above: `nav2_bringup`'s `navigation_launch.py` and `bringup_launch.py`, derived from the installed files when they are launched, and Nav2's `nav2_params.yaml` with a `stop` and a `slowdown` zone (or, selectable, the `slowdown` zone alone) in the place of `approach`. No node. [docs/DROP_IN_COLLISION_MONITOR.md, option C](docs/DROP_IN_COLLISION_MONITOR.md#option-c-the-bring-up-package). |
| `overlay/twist_mux` | The same node as a package named `twist_mux`, which shadows upstream's in a workspace: same executable, launch file and config file names. Not under `ros/`, and not part of `pixi run build`, because the tests compare with upstream's package of that name (`pixi run build-overlay`). |
| `overlay/opt_in/nav2_collision_monitor` | The same collision monitor as a package named `nav2_collision_monitor`, which shadows upstream's in a workspace, so that Nav2's unedited launch files start it: same executable, component, launch file and params file names. Not under `ros/`, not part of `pixi run build` (`pixi run build-overlay`), and skipped by colcon unless its directory is named. Read [docs/DROP_IN_COLLISION_MONITOR.md](docs/DROP_IN_COLLISION_MONITOR.md) first. |
| `ros/roslean_noop` | Minimal node that calls a Lean function; the original pipeline check. |
| `docker/` | Linux build and run-time image, both nodes. |
| `bench/` | Benchmarks against upstream (`pixi run bench`, `pixi run bench-cm`), their results, and the Linux image in which the collision monitor comparison was run (`bench/cm_linux/`). Not part of the normal build. |
| `review/` | Scripts and Lean files of the independent reviews of the collision monitor. |
| `findings/` | Notes on single questions. `stop_pub_timeout_theorem.md`: why a demanded stop is not always published. |
| `tasks/` | The briefs, plan and reports of the sessions that built the collision monitor. |
| `site/`, `video/` | The project page, and the scripts that made its video. |

## Quick start

Install Git, curl, [elan](https://github.com/leanprover/elan), and
[pixi](https://pixi.sh). On macOS, install the Xcode Command Line Tools
for Git and the SDK. Open a new terminal so `lake` and `pixi` are on
`PATH`. Pixi installs ROS 2 Jazzy, Nav2, and Gazebo locally. Allow roughly
21–24 GB for the environment, caches, Lean, and Mathlib if you run every
step. The graphical demos need a display.

To get both nodes from this branch, clone `collision-monitor`. Run the
build commands from the repository root.

```sh
git clone -b collision-monitor https://github.com/Beneficial-AI-Foundation/ROSlean.git
cd ROSlean
pixi run build        # build the Lean core and four ROS packages
pixi run test-quick   # skip tests labeled long
pixi run test         # full suite, about 40 minutes in the audit
pixi run prove        # fetch Mathlib's cache and check the proofs
pixi run demo         # run turtlesim with the mux
```

The [fresh-machine audit](docs/AUDIT.md) ran the build, proofs, tests,
and demos on macOS arm64. It also ran the full suite in Linux arm64
containers using Docker and pixi. The pixi run needed two fixes, now in
the branch. The repaired build passed five ROS smoke and interface tests.
The twist mux and an earlier collision monitor were tested on Linux
x86-64. See the [platform results](site/gaps.html#platforms) for details.

Run the mux with the example parameters.

```sh
pixi run bash -c 'source install/setup.bash && ros2 run roslean_twist_mux twist_mux \
  --ros-args --params-file ros/roslean_twist_mux/config/example.yaml'
```

Upstream's launch file is there too, with the same arguments
(`ros2 launch roslean_twist_mux twist_mux_launch.py`). To replace upstream in
an existing workspace without editing launch files, see
[docs/DROP_IN_TWIST_MUX.md](docs/DROP_IN_TWIST_MUX.md).

Run the collision monitor with a lifecycle manager that configures and
activates it, as Nav2 does for upstream's:

```sh
pixi run bash -c 'source install/setup.bash && ros2 launch roslean_collision_monitor \
  collision_monitor_launch.py params_file:=ros/roslean_collision_monitor/config/example.yaml'
```

or by hand (it is a lifecycle node and does nothing until activated), in
two terminals inside the environment (`pixi run bash`, then
`source install/setup.bash`):

```sh
ros2 run roslean_collision_monitor collision_monitor --ros-args \
  --params-file ros/roslean_collision_monitor/config/example.yaml
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate
```

Upstream's launch file and example parameter file are there too, under
upstream's names and with the same arguments
(`ros2 launch roslean_collision_monitor collision_monitor_node.launch.py`,
`params/collision_monitor_params.yaml`). To run the node in the place of
`nav2_collision_monitor` in an existing workspace, with or without editing
launch files, see
[docs/DROP_IN_COLLISION_MONITOR.md](docs/DROP_IN_COLLISION_MONITOR.md). It
starts with what the node refuses, which includes Nav2's default
configuration.

To start Nav2 as a whole with this node by one command there is the package
`roslean_nav2_bringup`:
`ros2 launch roslean_nav2_bringup bringup_launch.py map:=MAP.yaml`. It is
Nav2's bring-up with this collision monitor and with zones the monitor
accepts, not stock Nav2's behaviour: there is no `approach` zone. The
default zones are a stop zone inside a slowdown zone, which under a planner
can hold the robot for good; `zones:=slowdown` selects the slowdown zone
alone, with which the monitor never stops the robot
([docs/DROP_IN_COLLISION_MONITOR.md, option C](docs/DROP_IN_COLLISION_MONITOR.md#option-c-the-bring-up-package)).

## Linux and deployment

Builds need Lean. The installed nodes carry the Lean runtime with them.
Use a regular colcon install to copy the tree to a machine with the same
OS, architecture, and ROS 2 Jazzy dependencies. The development build
uses `--symlink-install`.

```sh
docker build -f docker/Dockerfile -t roslean-twist-mux .
docker run --rm --network host roslean-twist-mux
docker run --rm --network host -v "$PWD/my.yaml:/cfg.yaml" roslean-twist-mux \
  ros2 run roslean_twist_mux twist_mux --ros-args --params-file /cfg.yaml
docker run --rm --network host roslean-twist-mux \
  ros2 launch roslean_collision_monitor collision_monitor_launch.py
```

The image builds both nodes on Ubuntu 24.04 with ROS 2 Jazzy. It runs
the available ROS tests and verifies the install tree before copying it
into the runtime image. Use `--build-arg RUN_TESTS=0` to skip the tests.
Four Nav2 scenarios require `nav2_bringup`, which the image omits. The
[arm64 audit](docs/AUDIT.md) recorded 28 passing tests in 39 minutes.
The final image also holds apt's `ros-jazzy-twist-mux`, for its helper
nodes. In it, `ros2 run twist_mux twist_mux` and
`ros2 launch twist_mux twist_mux_launch.py` start the Lean-backed node (the
overlay package).

On a Linux host with the ROS dependencies and elan installed, source
`/opt/ros/jazzy/setup.bash` and run `colcon build --base-paths ros`.

## Demo

In turtlesim, the mux selects between a planner (priority 10) and the
keyboard (100). The e-stop lock (255) blocks both inputs.

```sh
pixi run demo            # terminal 1: turtlesim, mux, planner, narrator
pixi run demo-teleop     # terminal 2: keyboard teleop; hold a key to keep driving
pixi run demo-estop      # terminal 3: engage the e-stop lock
pixi run demo-release    #             release it
```

The turtle circles under the planner. Holding a teleop key takes over; half
a second after the key is released the planner resumes. Engaging the e-stop
blocks both sources. The simulator stops when its last command expires.
The narrator prints each change and the corresponding theorem.

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
`pixi run cm-check --nav2`). On Linux the check still needs a display for
lidar rendering. Use `pixi run cm-check-cloud` with `xvfb-run` on a
headless host. The check passed in nine runs of nine on macOS
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
  (fixed scenarios and a randomised differential test). Integer time arithmetic and one clock read per event can change decisions near timeout boundaries. Large priority values can also be clamped differently (SPEC D2, D4, D6).
- **The C++ shim** (`twist_mux_node.cpp`, `roslean_core/mux.hpp`): mapping
  subscriptions to indices, reading the clock, converting timeouts from
  seconds to nanoseconds, reference counting of Lean objects, calling the
  core once per message in order, and publishing on a `true` decision.
- **The declared C signatures.** They are checked against Lean's generated C
  at build time, except that Lean passes `Int64` as `uint64_t`, so signedness
  is not checked.
- **One event at a time.** The mux uses a single-threaded executor; the
  theorems say nothing about concurrent callbacks.
- **The clock.** Theorems are about recorded clock readings. If the clock
  jumps backwards, an expired input can become live again. A lock active
  only because of expiry can become inactive. A lock last set to true
  stays active.
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
polygons, with transforms from TF and base shift correction. Configuring `approach`, `limit`, velocity polygons, non-scan sources, or several sources fails the `configure` transition. The error lists the refused parameters.

The proofs cover velocity decisions. Obstacle avoidance also depends on
the sensor, zone sizes, transforms, and base controller.

### What is proved

All in `ROSlean/CollisionMonitor/`, with no `sorry`, no `native_decide` and
only Lean's three standard axioms. "Command" means a velocity command that
arrives while the node is active. The full list, with what each statement
assumes, is [docs/collision_monitor/THEOREMS.md](docs/collision_monitor/THEOREMS.md).

| Property | Theorem | Statement |
| --- | --- | --- |
| What is answered | `processed_iff` | The model processes only finite commands received while active. A processed command can still produce no ROS messages. |
| Decision | `evaluate_eq_stopSource_iff`, `evaluate_eq_stopZone_iff`, `evaluate_eq_slowdown_iff`, `evaluate_eq_pass_iff` | Four if-and-only-if statements, one per action: Stop for an invalid source or the first triggered stop zone. Otherwise apply the smallest triggered slowdown ratio below 1, breaking ties by zone order, or pass. |
| What is published | `velocity_eq_some_iff` | For a processed command, withhold velocity after `stop_pub_timeout` if the demanded velocity stays zero. State and visualization have separate rules. |
| Stop | `stop`, `run_stop` | For an active monitor and a finite command, a triggered stop zone yields zero or withheld velocity. The trace theorem covers fixed zones with an enabled source, a current scan, and matching coordinates. |
| Stale data, no transform | `stale`, `no_transform`, `run_stale`, `run_no_transform` | With the source enabled and a non-zero `source_timeout`: no scan, a scan older than the timeout, or no usable transform for it gives the stop for an invalid source. |
| No amplification | `no_amplification`, `run_no_amplification`, `published_ratio_in_unit_interval` | Every published action is pass, a stop, or a slowdown whose ratio is the configured one of the zone named, with a value in [0, 1). |
| Slowdown bound | `slowdown_bound` | The ratio applied is at most that of every triggered slowdown zone. |
| Pass-through | `pass_through`, `run_pass_through` | With an active monitor, a valid source, and no triggered zone, a finite command with nonzero planar velocity keeps its planar components. |
| Monotonicity | `zonesAction_stop_mono`, `evaluate_isStop_mono` | Adding points never lifts a stop. |
| Bad data | `output_filter_validBeam`, `step_cmd_of_not_finite` | Readings that are NaN or outside the range limits never change a decision; a non-finite command is ignored and leaves the state unchanged. |
| Lifecycle | `output_of_not_active`, `run_silent_before_activate`, `run_silent_after_shutdown` | Nothing is published while the node is not active, nor after a shutdown. |
| After the zero period | `run_withheld_last_published_zero` | In any run from start-up, when a velocity is withheld, the last velocity this node published was a zero. |
| Unusable polygon message | `evaluate_isStop_of_unusable`, `run_stop_after_unusable` | For an enabled topic stop zone, a polygon with at least three vertices and a usable transform triggers the zone if any transformed vertex is unusable. A usable shape or cleanup clears it (ND6). |
| Without a clock | `noClock_velocity` | The active standalone node answers finite commands with zero while its clock is invalid (D15). |
| Rounding errs towards "inside" | `circle_safe`, `nearBoundary_of_rounded`, `contains_of_shim` | For an accepted radius and finite coordinates, the model counts points strictly inside the circle. The polygon proof covers the rounding margin under its stated assumptions. |
| Code that runs | `cm_readBack_after_fast`, `stepCmdFast_eq`, `cm_not_ok_after` | For an accepted configuration with at most 2³² zones, the fast C interface returns the model's output after the calls covered by the theorem. No-clock commands use a separate path. A refused core configuration returns a stop only in the active phase. The ROS node fails configuration and stays inactive. |

### What is not proved, and is trusted or only tested

The trusted parts are numbered T1 to T10 in
[SPEC.md](docs/collision_monitor/SPEC.md), "Where the velocity is computed".

- **When a stop is withheld.** After `stop_pub_timeout` of zero demand,
  the monitor stops publishing velocity. Its last published velocity was
  zero. The base must hold that command and receive no other commands
  for this to keep the robot stopped (A3).
  [Example and proof](findings/stop_pub_timeout_theorem.md).
- **That the model matches upstream.** Read from the source and tested
  against the binary. On the final node on macOS: 318 checks in 73 scenario
  runs and 289 checks in 44 runs of the Nav2 extension, 0 failures, every
  difference a numbered one; 300 random cases with 26,951 commands, none
  outside the stated bounds, the Lean node the more restrictive one in each
  of the 586 commands that differed in demanded velocity; 300 random cases of the extension with
  16,797 commands, no disagreement. In the Linux arm64 container: the 289
  checks and 20 random cases. Details and the earlier, larger runs:
  [COMPARISON.md](docs/collision_monitor/COMPARISON.md).
- **The slowdown multiplication** (T1). The core returns the ratio's bit
  pattern; the shim multiplies in `double`. Lean proves the ratio bound. The shim relies on IEEE-754 for the product.
- **The points of a scan** (T2, T6, T7). The shim computes each scan point,
  looks the transform up in TF and applies it in `double`
  (`transforms.hpp`). The proofs use those coordinates as inputs.
  `smoke_transforms` and the `tf-*` and `shift-*` scenarios check them.
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

A topic zone takes no action before its first usable shape or unusable
message that triggers ND6. A zero `source_timeout` disables stops for
missing or stale scans and missing transforms. A scan with no valid
readings supplies zero points, and the zone rules still run. Without
incoming commands the monitor publishes no velocity.

ROSlean was stricter in the randomized zone cases that differed from
upstream. Other differences affect timing or edge cases. For example,
ND13 leaves a moving zone in its previous position if the recomputed
vertices are out of range. See [the difference table](docs/collision_monitor/SPEC.md#deliberate-differences-in-roslean).

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
Without a copy of the launch file: the package `nav2_collision_monitor` in
`overlay/opt_in/` shadows upstream's package, and Nav2's unedited
`navigation_launch.py` then starts the Lean node, as an executable or as a
component ([docs/DROP_IN_COLLISION_MONITOR.md](docs/DROP_IN_COLLISION_MONITOR.md)).
The edit of the parameters is needed either way.

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
