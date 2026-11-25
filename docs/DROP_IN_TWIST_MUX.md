# Using the Lean-backed node in place of `twist_mux`

You run upstream [`twist_mux`](https://index.ros.org/p/twist_mux/) on ROS 2
Jazzy today. This page says what to change to run the node of this repository
instead, how to check that it is the one running, how to go back, and what is
different. The same procedure as checkable steps for a coding agent is in
[DROP_IN_TWIST_MUX_AGENTS.md](DROP_IN_TWIST_MUX_AGENTS.md).

The repository is in an exploratory phase. What was actually run is listed
under [What was tested](#what-was-tested); nothing here has been run on a
physical robot.

## What you get

Two packages provide the same node (the same source file, compiled twice):

| Package | In | Use it when |
| --- | --- | --- |
| `twist_mux` | `overlay/twist_mux` | You want to change nothing. Built in your workspace, it shadows upstream's package of the same name. |
| `roslean_twist_mux` | `ros/roslean_twist_mux` | You prefer to see in your launch files which node runs. You change the package name, one line per place. |

What is the same as upstream 4.5.0 in both:

- the executable name, `twist_mux`, and the node name, `twist_mux`;
- the parameters: `use_stamped`, `topics.<name>.{topic,timeout,priority}`,
  `locks.<name>.{topic,timeout,priority}`, `use_sim_time`; your yaml files
  are used as they are;
- the topics: one subscription per configured input (`TwistStamped`, or
  `Twist` with `use_stamped: false`) and per lock (`std_msgs/Bool`), output on
  `cmd_vel_out` with the same QoS;
- the launch file `twist_mux_launch.py` with the arguments `config_locks`,
  `config_topics`, `config_joy`, `cmd_vel_out`, `use_sim_time` and the same
  defaults;
- the config files `config/twist_mux_locks.yaml`, `config/twist_mux_topics.yaml`
  and `config/joystick.yaml` (copies of upstream's).

What is not there: the `/diagnostics` publisher, and the helper nodes
`twist_marker` and `joystick_relay.py`. For the helpers, upstream's own are
used when upstream is installed; see [Differences](#differences-from-upstream).

## Install

### From source, in your workspace

You need ROS 2 Jazzy, `colcon`, a C/C++ compiler, and
[elan](https://github.com/leanprover/elan), the Lean version manager. elan
installs into `~/.elan` without root. The first build downloads the Lean
toolchain named in `lean-toolchain`. Mathlib is not needed.

```sh
curl -sSf https://elan.lean-lang.org/elan-init.sh | sh -s -- -y --default-toolchain none
export PATH="$HOME/.elan/bin:$PATH"

cd ~/ws/src
git clone https://github.com/Beneficial-AI-Foundation/ROSlean.git
cd ~/ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to twist_mux          # roslean_core and twist_mux
source install/setup.bash
```

For the renamed package instead of the overlay, build
`--packages-up-to roslean_twist_mux`. That package also holds the demos, and
its `package.xml` names their dependencies (turtlesim, Gazebo, Nav2), so
`rosdep` would install them. The overlay package depends only on `rclcpp`,
`geometry_msgs`, `std_msgs`, `launch_ros` and `ament_index_python`.

Lean is needed to build, not to run. The install tree contains the Lean
runtime it uses and can be copied to another machine of the same OS and
architecture with ROS 2 Jazzy, provided it was built without
`--symlink-install`. On Linux this includes `libleanshared.so`, about 190 MB.

The build writes into the checkout (`core/.lake`), so the source directory
must be writable.

If your workspace also contains the source of upstream `twist_mux`, colcon
refuses to build two packages of the same name. Remove one, or put a
`COLCON_IGNORE` file in one of them.

### Docker

```sh
docker build -f docker/Dockerfile -t roslean-twist-mux .
docker run --rm --network host -v $PWD/my.yaml:/cfg.yaml roslean-twist-mux \
  ros2 run twist_mux twist_mux --ros-args --params-file /cfg.yaml
docker run --rm --network host roslean-twist-mux ros2 launch twist_mux twist_mux_launch.py
```

The image has ROS 2 Jazzy from apt, upstream `ros-jazzy-twist-mux` (for the
helper nodes), and both packages of this repository. Its entrypoint sources
this repository's install tree last, so `twist_mux` there means the
Lean-backed node. There is no published image; you build it.

### Binary packages

There is none: no `.deb`, no conda package. What one would take is in
[What is not done](#what-is-not-done).

## The change to make

### Option A: change nothing (overlay)

Source your workspace after ROS, as you do already. From then on these start
the Lean-backed node:

```sh
ros2 run twist_mux twist_mux --ros-args --params-file my_mux.yaml
ros2 launch twist_mux twist_mux_launch.py config_topics:=my_topics.yaml config_locks:=my_locks.yaml
```

and so does any launch file of yours or of a robot bring-up package that says

```python
Node(package='twist_mux', executable='twist_mux', ...)
```

or includes `twist_mux_launch.py` from `get_package_share_directory('twist_mux')`.
No file is edited. The condition is that the process that launches has the
workspace sourced: a systemd unit or a robot's start-up script that sources
only `/opt/ros/jazzy/setup.bash` keeps starting upstream.

`ros2 run twist_mux twist_marker` and `ros2 run twist_mux joystick_relay.py`
keep working if upstream is installed underneath: the overlay installs small
scripts under these names that find upstream's executable in the other
prefixes of `AMENT_PREFIX_PATH` and replace themselves with it. Without
upstream they exit with code 1 and a message.

### Option B: change the package name

| Where | Before | After |
| --- | --- | --- |
| Command line | `ros2 run twist_mux twist_mux --ros-args --params-file my_mux.yaml` | `ros2 run roslean_twist_mux twist_mux --ros-args --params-file my_mux.yaml` |
| Command line | `ros2 launch twist_mux twist_mux_launch.py ...` | `ros2 launch roslean_twist_mux twist_mux_launch.py ...` |
| Launch file | `Node(package='twist_mux', executable='twist_mux', ...)` | `Node(package='roslean_twist_mux', executable='twist_mux', ...)` |
| Launch file | `get_package_share_directory('twist_mux')` (to include the launch file or to find the default yaml files) | `get_package_share_directory('roslean_twist_mux')` |
| `package.xml` | `<exec_depend>twist_mux</exec_depend>` | `<exec_depend>roslean_twist_mux</exec_depend>` (keep `twist_mux` as well if you use the helper nodes) |

Leave `package='twist_mux'` on `twist_marker` and `joystick_relay.py`: they
stay upstream's. The parameter files need no change. Their top-level key is
the node name, which is still `twist_mux`.

`ros2 launch roslean_twist_mux twist_mux_launch.py` starts the two helper
nodes from the package `twist_mux` if one is installed, and prints a line and
leaves them out if not.

### A robot bring-up

Bring-up packages that use `twist_mux` (Clearpath's and TurtleBot 4's do, as
far as I remember; I did not check their current sources) start it with a
`Node(package='twist_mux', executable='twist_mux', ...)` action with their own
yaml file and a remapping of `cmd_vel_out`. With option A you build the
overlay in the workspace from which the bring-up is launched and edit
nothing. With option B you edit that one `Node` action, in a copy of the
bring-up package in your workspace.

Nav2 itself does not start a `twist_mux`.

## Check which node is running

```sh
ros2 pkg prefix twist_mux                 # option A: your workspace, not /opt/ros/jazzy
ros2 node info /twist_mux | grep diagnostics   # no output: the Lean-backed node
ps -o command= -C twist_mux               # Linux: the path of the executable
```

At start-up, the Lean-backed node writes `Lock handler 'locks.…' subscribed to
topic …` for each lock; upstream writes `Topic handler 'locks.…'`.

## Go back

- Option A: do not source the workspace, or delete `install/twist_mux` and
  `build/twist_mux` and open a new shell, or put a `COLCON_IGNORE` file in
  `src/ROSlean/overlay/twist_mux` before the next clean build. Then
  `ros2 pkg prefix twist_mux` prints `/opt/ros/jazzy` again.
- Option B: change the package name back.

Upstream's package is never modified or removed by either option.

## Differences from upstream

The full list, with reasons, is the table "Deliberate differences in ROSlean"
in [SPEC.md](SPEC.md#deliberate-differences-in-roslean) (D2 to D8; D9, the missing launch file, is withdrawn). Those a
user can meet:

- **No `/diagnostics`** (D3). Upstream publishes
  `diagnostic_msgs/DiagnosticArray` on `/diagnostics` and has the parameters
  `diagnostic_updater.period` and `diagnostic_updater.use_fqn`. This node
  publishes nothing there. A diagnostic aggregator that expects a `twist_mux`
  entry will report it missing or stale.
- **No helper nodes of its own** (D3). `twist_marker` and `joystick_relay.py`
  are upstream's, used as described above.
- **Rejected at start-up** (D5): a timeout that is NaN, infinite, or 9.2e9 s
  or more in magnitude. The node logs FATAL and exits with code 1.
- **Exit on a start-up error** (D7): exit code 1 with a FATAL log line, where
  upstream aborts (SIGABRT).
- **Exact timeout boundaries** (D2, D4): decisions can differ from upstream's
  when a message arrives within about a microsecond of an input's timeout.
- **No C++ headers.** Upstream installs `include/twist_mux/*.hpp`. The overlay
  does not, so a package that compiles against them does not build against
  the overlay. `find_package(twist_mux)` itself succeeds.
- **Version.** The overlay's `package.xml` says 0.1.0, not 4.5.0.
- **The ROS 1 launch file** `twist_mux.launch` that upstream still installs is
  not copied. It is not usable with `ros2 launch`.

The decisions (which message is forwarded) are otherwise the same in every
comparison run so far: fixed scenarios and a randomised differential test
against upstream, part of `pixi run test`. That the model matches upstream is
tested, not proved; see "What is proved" in the [README](../README.md).

## What was tested

On 2026-11-25, with upstream `twist_mux` 4.5.0:

| What | macOS arm64, ROS from RoboStack (pixi) | Linux arm64, Ubuntu 24.04, ROS from apt (Docker) |
| --- | --- | --- |
| `ros2 run twist_mux twist_mux` with upstream's `twist_mux_locks.yaml` and `twist_mux_topics.yaml`, overlay sourced: same subscriptions, publishers (except `/diagnostics`), parameters (except `diagnostic_updater.*`) and output QoS as upstream; priority, timeout and lock behaviour checked with `ros2 topic pub` | run | run, without the QoS comparison |
| The same with `ros2 run roslean_twist_mux twist_mux` | run | not run |
| `ros2 launch twist_mux twist_mux_launch.py cmd_vel_out:=cmd_vel use_sim_time:=true`, overlay sourced: same arguments, nodes and topics as upstream's launch file (except `/diagnostics`) | run | run |
| `ros2 launch roslean_twist_mux twist_mux_launch.py`, same arguments | run | not run |
| An unedited launch file with `Node(package='twist_mux', ...)`, a namespace, `use_stamped: false` and a remapping, before and after sourcing the overlay | run | not run |
| `colcon build --packages-up-to twist_mux` in a separate workspace that contains a copy of this repository | run | run |
| `twist_marker` through the overlay's forwarding script | run | run |
| `joystick_relay.py` through the overlay's forwarding script | starts upstream's script, which fails in this environment with and without the overlay (its `#!/usr/bin/python3` line selects the system Python, not the environment's) | run |
| The repository's tests (`pixi run test`; in Docker the build runs them) | run | run |

Not tested: Linux x86-64 for anything on this page (the node itself was built
and tested there earlier, see the README); any ROS distribution other than
Jazzy; a real robot bring-up package; `rosdep install` on these packages;
copying the install tree to another machine; a merged install
(`colcon build --merge-install`).

## What is not done

- **Other ROS distributions.** Only Jazzy. Upstream's default for
  `use_stamped`, among other things, differs between distributions; this node
  follows 4.5.0.
- **Building without Lean.** The C files that Lean generates (four, in
  `core/.lake/build/ir/`) could be committed, but compiling them also needs
  Lean's headers (`lean/lean.h`) and linking needs Lean's runtime: static
  libraries on macOS, `libleanshared.so` on Linux. A build without elan
  would have to vendor those per platform or build the runtime from Lean's
  sources. Neither was tried.
- **A `.deb`.** `bloom-generate rosdebian` followed by `fakeroot debian/rules
  binary` for `roslean_core` and `roslean_twist_mux` on an Ubuntu 24.04
  machine with elan is the likely route. To be settled: the generated rules
  must not strip or rewrite the bundled `libleanshared.so`; the build needs
  network access for the Lean toolchain unless it is installed beforehand;
  the demo dependencies of `roslean_twist_mux` would become dependencies of
  the `.deb` unless the demos move to a package of their own. A release
  through the ROS build farm is not possible as long as Lean is not available
  there. The overlay package cannot be a `.deb` installed into
  `/opt/ros/jazzy`: its files would collide with `ros-jazzy-twist-mux`.
- **A conda package (RoboStack).** Needs a Lean toolchain as a build
  dependency in the recipe. Not tried.
- **`/diagnostics`.** Would be C++ outside the proved core; not written.
