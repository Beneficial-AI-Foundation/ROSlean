# ROSlean

ROS 2 nodes whose decision logic is written and proved in Lean. First target:
[`twist_mux`](https://index.ros.org/p/twist_mux/), which picks one of several
velocity command sources by priority, freshness and locks.

- [twist_mux.md](twist_mux.md): scope, and where to start as a new reader.
- [docs/SPEC.md](docs/SPEC.md): the rules, read from the upstream C++ source,
  and every deliberate difference.
- [docs/PLAN.md](docs/PLAN.md): development plan and status.
- [docs/THEOREMS.md](docs/THEOREMS.md): the main theorems in plain words,
  with links to the statements.
- [docs/TUTORIAL.md](docs/TUTORIAL.md): from an empty machine to the
  turtlesim experiment, step by step.
- [docs/TUTORIAL_GAZEBO.md](docs/TUTORIAL_GAZEBO.md): the same experiment
  on a TurtleBot3 in a furnished apartment in Gazebo, with Nav2.

## Layout

| Path | Contents |
| --- | --- |
| `core/` | A Lean package with no dependencies. `core/ROSleanCore/` holds the executable model (`Mux.lean`) and its C interface (`FFI.lean`). Compiled to C and linked into the library that the node loads. |
| `ROSlean/` | Lean + Mathlib (the root package, which requires `core`). Theorems about `ROSleanCore`. Never linked into a node. |
| `ros/roslean_core` | Compiles the C that Lean generates into `libroslean_core` and installs the Lean runtime it needs with it; C++ wrapper, FFI tests. |
| `docker/` | Linux build and run-time image. |
| `ros/roslean_twist_mux` | The node. A C++ shim with no arbitration logic: it reads parameters, subscribes, reads the clock and publishes. |
| `ros/roslean_noop` | Minimal node that calls a Lean function; the original pipeline check. |

## Quick start

Requires [elan](https://github.com/leanprover/elan) and
[pixi](https://pixi.sh), both on `PATH`. The ROS 2 Jazzy environment
(RoboStack, about 1 GB) is installed into `.pixi/` on first use, and the first
build also downloads Mathlib. Verified on macOS arm64 this way; on Linux the
verified route is the Docker build below.

```sh
pixi run build     # lake build of the core + colcon build of the ROS packages
pixi run test      # FFI tests, and scenarios compared against upstream twist_mux
pixi run prove     # check all theorems (first run fetches or builds Mathlib)
pixi run demo      # turtlesim driven through the Lean-backed mux
```

Run the node the same way as upstream:

```sh
pixi run bash -c 'source install/setup.bash && ros2 run roslean_twist_mux twist_mux \
  --ros-args --params-file ros/roslean_twist_mux/config/example.yaml'
```

## Linux and deployment

A Lean installation is needed to build, not to run, and building the node
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
runs all tests including the comparison with apt's `ros-jazzy-twist-mux`, and
checks that the install tree is self-contained; the build fails if any of
that fails. The final image holds ROS and the install tree only.
`--build-arg RUN_TESTS=0` skips the tests.

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

## What is proved

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

## GitHub configuration

To set up your new GitHub repository, follow these steps:

* Under your repository name, click **Settings**.
* In the **Actions** section of the sidebar, click "General".
* Check the box **Allow GitHub Actions to create and approve pull requests**.
* Click the **Pages** section of the settings sidebar.
* In the **Source** dropdown menu, select "GitHub Actions".

After following the steps above, you can remove this section from the README file.
