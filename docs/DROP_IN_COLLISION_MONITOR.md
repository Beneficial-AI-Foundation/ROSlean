# Using the Lean-backed node in place of `nav2_collision_monitor`

## Read this first: it does not replace Nav2's default configuration

The node of this repository provides the `stop` and `slowdown` actions on
**one laser scan**, with zones that are circles, polygons, or polygons whose
shape arrives on a topic. It is a replacement for `nav2_collision_monitor`
only for configurations that use nothing else.

- **Nav2's own parameter file is refused.** `nav2_bringup`'s
  `nav2_params.yaml` configures the collision monitor with one zone,
  `FootprintApproach`, whose `action_type` is `approach`. This node does not
  have `approach`. With that file its `configure` transition fails, and
  Nav2's lifecycle manager aborts the bring-up of the whole navigation stack
  (the output is under [Nav2's default parameters](#nav2s-default-parameters-the-expected-refusal)).
- **Also refused:** `action_type: limit` and `none`; zones of type
  `velocity_polygon`; a circle whose radius arrives on a topic; sources of
  type `pointcloud`, `range` or `polygon`; more than one source; `max_points`;
  `use_realtime_priority: true`. The whole list is the table "Excluded
  features" in [collision_monitor/SPEC.md](collision_monitor/SPEC.md#excluded-features).
- **A `stop` zone in the place of `approach` changes what the robot does.**
  `approach` slows the robot in proportion to the time before a predicted
  collision and lets a command that leads away from the obstacle pass. A
  stop zone turns every command into zero while it is triggered, including
  the turn or the retreat that would clear it, and including Nav2's recovery
  behaviours. With Nav2 driving a simulated robot through such zones, the
  robot ended standing for good in three runs of six, with this node and
  with upstream's node on the same zones
  ([TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), section
  7.6; [site/gaps.html](../site/gaps.html)).

So the first step is to look at your parameters
([Check your parameters first](#check-your-parameters-first)). If they use
only what the node provides, the rest of this page says what to change to
run it instead of upstream's node, how to check that it is the one running,
how to go back, and what is different. The same procedure as checkable steps
for a coding agent is in
[DROP_IN_COLLISION_MONITOR_AGENTS.md](DROP_IN_COLLISION_MONITOR_AGENTS.md).

If you have no bring-up of your own yet and want Nav2 started with this node
by one command, with a parameter file the node accepts, that is
[option C, the bring-up package](#option-c-the-bring-up-package). It is not
stock Nav2's behaviour either.

The repository is in an exploratory phase. What was actually run is listed
under [What was tested](#what-was-tested); nothing here has been run on a
physical robot.

## What you get

Two packages provide the same node (the same source file, compiled twice):

| Package | In | Use it when |
| --- | --- | --- |
| `nav2_collision_monitor` | `overlay/opt_in/nav2_collision_monitor` | You want to edit no launch file. Built in your workspace, which takes a build command that names it, it shadows upstream's package of the same name: Nav2's `navigation_launch.py` and every other launch file that names `nav2_collision_monitor` then start the Lean-backed node. |
| `roslean_collision_monitor` | `ros/roslean_collision_monitor` | You prefer to see in your launch files which node runs. You change the package name (and, for a composed bring-up, the plugin name). |

A third package, `roslean_nav2_bringup` (`ros/roslean_nav2_bringup`), has
no node. It has the launch files and the parameter file that start Nav2 with
the node of `roslean_collision_monitor`:
[option C](#option-c-the-bring-up-package).

What is the same as upstream 1.3.12 in both:

- the executable name, `collision_monitor`, and the node name,
  `collision_monitor`;
- it is a lifecycle node built on `nav2_util::LifecycleNode`, with the same
  transitions and the same bond: Nav2's lifecycle manager configures it,
  activates it and watches it, and `collision_monitor` stays in the
  manager's `node_names` as it is;
- the parameters of the supported subset, under upstream's names and with
  upstream's meaning; your yaml file is used as it is;
- the topics: the command in on `cmd_vel_in_topic` and out on
  `cmd_vel_out_topic` (`Twist`, or `TwistStamped` with
  `enable_stamped_cmd_vel: true`), the scan, `state_topic`
  (`nav2_msgs/CollisionMonitorState`), one `PolygonStamped` per zone with
  `visualize: true`, a subscription per zone with `polygon_sub_topic`, and
  TF;
- the launch file `collision_monitor_node.launch.py` with the arguments
  `namespace`, `use_sim_time`, `params_file`, `use_composition` and
  `container_name` and the same defaults;
- the parameter file `params/collision_monitor_params.yaml`: upstream's
  example, with the same active zone (`PolygonStop`) and source, without the
  example zones and the example source that this node would refuse.

The overlay package also has upstream's component name,
`nav2_collision_monitor::CollisionMonitor`, which is what Nav2 loads with
`use_composition:=True`. `roslean_collision_monitor` has the component under
its own name, `roslean_collision_monitor::CollisionMonitorNode`.

What is not there, in either package: the `~/toggle` service, changing
parameters while the node runs, the `~/collision_points_marker` topic, and
everything listed at the top of this page. The collision detector
(`collision_detector`) is not reimplemented; see
[The collision detector](#the-collision-detector).

## Check your parameters first

Before anything is built:

```sh
python3 ros/roslean_collision_monitor/scripts/check_params.py my_nav2_params.yaml
```

(the script needs only PyYAML; run it from a checkout of this repository).
It reads every collision monitor section of the file and prints either
`nothing unsupported found` or the parameters the node would refuse, for
example for Nav2's own file:

```
nav2_params.yaml: /collision_monitor: NOT ACCEPTED by the Lean-backed node:
    FootprintApproach.action_type: approach (only stop and slowdown)
```

The script is a first look. It does not see values set from a launch file
or the command line, and it does not check numbers. The authority is the
node itself: after the build, start it on your file in a ROS domain that
nothing else uses, and configure it without activating it:

```sh
export ROS_DOMAIN_ID=99        # a domain your robot does not use
ros2 run roslean_collision_monitor collision_monitor --ros-args --params-file my_nav2_params.yaml &
ros2 lifecycle set /collision_monitor configure
```

(with the overlay: `ros2 run nav2_collision_monitor collision_monitor`).
`Transitioning successful` means the file is accepted. `Transitioning
failed` comes with one error line in the node's output that names every
refused parameter. Stop the node afterwards (`kill %1`). A node that is
configured and not activated publishes no velocity.

If the file is refused because of an `approach` zone, there is no setting of
this node that does the same. The edit this repository documents is to put a
fixed `stop` polygon (and, if you want, a `slowdown` polygon) in its place:
[collision_monitor/SPEC_NAV2.md](collision_monitor/SPEC_NAV2.md), section
8.6, and [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md),
sections 7.1 and 7.2. Changing only the word `approach` to `stop` does not
configure, with upstream's node or with this one, because `footprint_topic`
is read for `approach` only. Whether a stop zone is acceptable for your
robot is your decision; read section 7.6 of the tutorial before you take it.
The same edited file works with upstream's node, so you can try the zones
with upstream first and change the node afterwards.

## Install

### From source, in your workspace

You need ROS 2 Jazzy with Nav2 (`nav2_util`, `nav2_msgs`, `nav2_common`,
`nav2_lifecycle_manager`; with `ros-jazzy-navigation2` you have them),
`colcon`, a C/C++ compiler, and [elan](https://github.com/leanprover/elan),
the Lean version manager. elan installs into `~/.elan` without root. The
first build downloads the Lean toolchain named in `lean-toolchain`. Mathlib
is not needed.

```sh
curl -sSf https://elan.lean-lang.org/elan-init.sh | sh -s -- -y --default-toolchain none
export PATH="$HOME/.elan/bin:$PATH"

cd ~/ws/src
git clone -b collision-monitor https://github.com/Beneficial-AI-Foundation/ROSlean.git
cd ~/ws
source /opt/ros/jazzy/setup.bash
# option A: roslean_core and the overlay
colcon build --base-paths src src/ROSlean/overlay/opt_in/nav2_collision_monitor \
  --packages-up-to nav2_collision_monitor
source install/setup.bash
```

The overlay is not found by colcon unless its directory is named, as above:
`overlay/opt_in` has a `COLCON_IGNORE` file. A plain `colcon build` in a
workspace that contains this repository therefore never replaces Nav2's
collision monitor; it also does not rebuild the overlay after a change, so
repeat the command above for that.

For the renamed package instead of the overlay (option B):

```sh
colcon build --packages-up-to roslean_collision_monitor   # roslean_core, roslean_collision_monitor
```

For the bring-up package (option C):

```sh
colcon build --packages-up-to roslean_nav2_bringup   # the two above and roslean_nav2_bringup
```

`roslean_collision_monitor` also holds the node's tests; its `package.xml`
names, as test dependencies, upstream's `nav2_collision_monitor` and
`nav2_bringup` (one test reads Nav2's parameter file). The demos, with the
simulator and the TurtleBot3 packages, are in a package of their own,
`roslean_demos`, which none of the commands above builds. The overlay
package depends only on what the node needs.

Lean is needed to build, not to run. The install tree contains the Lean
runtime it uses and can be copied to another machine of the same OS and
architecture with ROS 2 Jazzy and Nav2, provided it was built without
`--symlink-install`. On Linux this includes `libleanshared.so`, about
190 MB. One path in the overlay is absolute, see
[The collision detector](#the-collision-detector).

The build writes into the checkout (`core/.lake`), so the source directory
must be writable.

If your workspace also contains the source of Nav2, colcon refuses to build
two packages named `nav2_collision_monitor`. Use option B, or put a
`COLCON_IGNORE` file in Nav2's `nav2_collision_monitor` directory.

### Docker

`docker/Dockerfile` builds an image with `roslean_collision_monitor`
(option B) and Nav2's lifecycle manager; in it,
`ros2 launch roslean_collision_monitor collision_monitor_launch.py` starts
the node. The image has neither the overlay package nor `nav2_common`,
which `collision_monitor_node.launch.py` imports; what would be added is in
[What is not done](#what-is-not-done).

### Binary packages

There is none: no `.deb`, no conda package.

## The change to make

### Option A: edit no launch file (overlay)

Source your workspace after ROS, as you do already. From then on these start
the Lean-backed node:

```sh
ros2 run nav2_collision_monitor collision_monitor --ros-args --params-file my_params.yaml
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py params_file:=my_params.yaml use_composition:=False
ros2 launch nav2_bringup navigation_launch.py params_file:=my_nav2_params.yaml
ros2 launch nav2_bringup bringup_launch.py params_file:=my_nav2_params.yaml map:=my_map.yaml use_composition:=False
```

(the first three were run; `bringup_launch.py` includes `navigation_launch.py`
and was not run as a whole, see [What was tested](#what-was-tested)), and so
does any launch file of yours or of a robot bring-up package that says

```python
Node(package='nav2_collision_monitor', executable='collision_monitor', ...)
ComposableNode(package='nav2_collision_monitor', plugin='nav2_collision_monitor::CollisionMonitor', ...)
```

No launch file is edited. What you must know before choosing this:

- **It applies to everything launched from that shell.** Every bring-up
  started with the workspace sourced gets the Lean-backed node, also one that
  uses Nav2's default parameters, which then does not come up (see below).
  A systemd unit or a start-up script that sources only
  `/opt/ros/jazzy/setup.bash` keeps starting upstream.
- **Composition.** With `use_composition:=True` (the default of
  `bringup_launch.py` and of `collision_monitor_node.launch.py`; the default
  of `navigation_launch.py` is `False`) Nav2 loads the node as a component
  into `nav2_container`. That works with the overlay. The stand-alone
  executable is the form that has every property claimed for the node: only
  it survives a `/clock` message with a negative time
  ([SPEC.md](collision_monitor/SPEC.md#deliberate-differences-in-roslean),
  D15 and ND10; upstream's node survives it in neither form). Prefer
  `use_composition:=False`.
- **No library and no headers.** Upstream's package exports the libraries
  `collision_monitor_core` and `collision_detector_core` and the headers
  `nav2_collision_monitor/*.hpp` to CMake. The overlay exports nothing. A
  package that does `find_package(nav2_collision_monitor)` and links or
  includes them does not build while the overlay is sourced. Among the
  packages installed with Nav2 1.3.12 here, none does: only the metapackage
  `navigation2` names `nav2_collision_monitor` in its `package.xml`, as a
  run-time dependency, and no installed library or executable other than
  upstream's own two nodes links against the two libraries. Upstream's libraries
  themselves stay where they are and are not hidden: the overlay's library
  has another file name.
- **The collision detector** keeps working if upstream is installed
  underneath; see [The collision detector](#the-collision-detector).

### Option B: change the package name

| Where | Before | After |
| --- | --- | --- |
| Command line | `ros2 run nav2_collision_monitor collision_monitor --ros-args --params-file my_params.yaml` | `ros2 run roslean_collision_monitor collision_monitor --ros-args --params-file my_params.yaml` |
| Command line | `ros2 launch nav2_collision_monitor collision_monitor_node.launch.py ...` | `ros2 launch roslean_collision_monitor collision_monitor_node.launch.py ...` |
| Launch file | `Node(package='nav2_collision_monitor', executable='collision_monitor', ...)` | `Node(package='roslean_collision_monitor', executable='collision_monitor', ...)` |
| Launch file, composed | `ComposableNode(package='nav2_collision_monitor', plugin='nav2_collision_monitor::CollisionMonitor', ...)` | `ComposableNode(package='roslean_collision_monitor', plugin='roslean_collision_monitor::CollisionMonitorNode', ...)` |
| Launch file | `get_package_share_directory('nav2_collision_monitor')` (to include the launch file or to find the default yaml file) | `get_package_share_directory('roslean_collision_monitor')` |
| `package.xml` | `<exec_depend>nav2_collision_monitor</exec_depend>` | `<exec_depend>roslean_collision_monitor</exec_depend>` (keep `nav2_collision_monitor` as well if you use the collision detector) |

`executable`, `name`, `namespace`, `parameters`, `remappings` and the
lifecycle manager's `node_names` stay. The parameter files need no change
for the change of node: their top-level key is the node name, which is still
`collision_monitor`. Leave `package='nav2_collision_monitor'` on anything
that starts `collision_detector`.

For Nav2's own bring-up this means a copy of
`nav2_bringup/launch/navigation_launch.py` with the two places above
changed (the `Node` action for `use_composition:=False`, the
`ComposableNode` for `True`), and, because `bringup_launch.py` names the
installed `navigation_launch.py`, a copy of that file too or a launch file
of your own that includes the two. The diff of the one line is in
[TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), section 7.1.

`ros2 launch roslean_collision_monitor collision_monitor_launch.py` is an
older, shorter launch file of this package (the node and a lifecycle
manager, no composition); it stays.

### Option C: the bring-up package

**What it is not.** It is not stock Nav2. Nav2's default collision monitor
zone is an `approach` zone, which this node does not have. The package
starts Nav2 with other zones, so the robot is protected differently: the
default parameter file has a stop zone inside a slowdown zone, and a stop
zone answers every command with zero while something is in it, including
the turn that would clear it, so that under a planner the robot can end up
standing for good. The zone coordinates in the package's parameter files are an example for
the robot Nav2's default parameters assume (a circle of radius 0.22 m); they
are not fitted to your robot. Nothing here was run on a physical robot.

**What it is.** `ros/roslean_nav2_bringup`: two launch files and two
parameter files. No package is shadowed, no launch file of yours is edited,
and stock Nav2 stays installed beside it: `ros2 launch nav2_bringup …`
starts upstream's monitor as before, `ros2 launch roslean_nav2_bringup …`
starts the Lean-backed one.

```sh
ros2 launch roslean_nav2_bringup bringup_launch.py map:=my_map.yaml
ros2 launch roslean_nav2_bringup bringup_launch.py slam:=True
ros2 launch roslean_nav2_bringup navigation_launch.py
```

| File | What it is |
| --- | --- |
| `launch/navigation_launch.py` | `nav2_bringup`'s `navigation_launch.py` with the collision monitor taken from `roslean_collision_monitor`, as an executable and as a component |
| `launch/bringup_launch.py` | `nav2_bringup`'s `bringup_launch.py` (localisation or SLAM, and navigation) including the file above. Localisation and SLAM are Nav2's launch files, unchanged |
| `params/nav2_params.yaml` | `nav2_bringup`'s `nav2_params.yaml` with another `collision_monitor` section: a stop zone inside a slowdown zone. The default |
| `params/nav2_params_slowdown.yaml` | the same with the slowdown zone only |

The launch files take the arguments of Nav2's files of the same name, with
the same meaning (`namespace`, `use_sim_time`, `autostart`, `params_file`,
`use_composition`, `use_respawn`, `log_level`, `container_name`, and for
`bringup_launch.py` `map`, `slam`, `use_namespace`, `use_localization`).
`ros2 launch roslean_nav2_bringup bringup_launch.py --show-args` lists them.
What differs from Nav2's:

| Argument | Here | Why |
| --- | --- | --- |
| `params_file` | default: this package's `params/nav2_params.yaml` | Nav2's own file is refused by the node |
| `zones` (new) | `stop_slowdown` (default) or `slowdown`: which of the package's two files is used when `params_file` is not given. Without effect when it is | see [Which zones](#which-zones-and-what-was-measured) |
| `use_composition` | default `False`, also in `bringup_launch.py` (Nav2: `True` there) | the stand-alone executable is the form that has every property claimed for the node: only it survives a `/clock` message with a negative time ([SPEC.md](collision_monitor/SPEC.md#deliberate-differences-in-roslean), D15 and ND10). `use_composition:=True` works: the component `roslean_collision_monitor::CollisionMonitorNode` is loaded into `nav2_container` |
| `check_params` (new) | `true` (default): the parameter file is looked at before anything is started | see [Your own parameter file](#your-own-parameter-file) |

**How the launch files are made.** They are not copies of Nav2's. Each reads
the installed `nav2_bringup` file of the same name when it is launched,
changes the package (and, for the component, the plugin) of the collision
monitor in its text, or the path of the included `navigation_launch.py`, and
runs the result. A Nav2 update is therefore followed without a change here.
If a place that is edited is not found exactly once, or the edited text
still names `nav2_collision_monitor`, the launch stops with a message that
says so and starts nothing; it does not fall back to upstream's monitor. The
edits are in `roslean_nav2_bringup/derive.py`. They were written for
`nav2_bringup` 1.3.12.

**The parameter files** are Nav2's `nav2_params.yaml`, line for line, outside
the `collision_monitor` section; a comment at the top of that section says
what was changed. `ros2 run roslean_nav2_bringup sync_params.py` checks this
against the installed `nav2_bringup`, and the package's test
`params_follow_nav2` fails when the two differ anywhere else, which is how a
change of Nav2's defaults is noticed (`sync_params.py SOURCE_DIR --update`
takes it over and keeps the section).

#### Your own parameter file

With `params_file:=my_nav2_params.yaml` the launch file runs
`check_params.py` ([Check your parameters first](#check-your-parameters-first))
on the file before it starts anything. If the node would refuse the file,
the launch ends with exit code 1 and this, and no process was started:

```
[ERROR] [launch]: Caught exception in launch (see debug for traceback): roslean_nav2_bringup: the
  collision monitor of this bring-up (roslean_collision_monitor) would refuse the parameter file
  …/nav2_bringup/params/nav2_params.yaml:
    /collision_monitor: FootprintApproach.action_type: approach (only stop and slowdown)
It provides the actions stop and slowdown, zones of type polygon and circle, and one laser scan.
  Nothing was started. What to do: take the collision_monitor section of
  …/roslean_nav2_bringup/params/nav2_params.yaml into your file and fit the zones to your robot,
  or start stock Nav2 (ros2 launch nav2_bringup ...) to keep upstream's monitor. …
```

(with Nav2's own `nav2_params.yaml` as `params_file`). Otherwise one line
says which file is used and that nothing unsupported was found. The check
is the first look described above, not the node's own: what it does not see
(numbers, values set elsewhere) the node still refuses when it is
configured, as under
[Nav2's default parameters](#nav2s-default-parameters-the-expected-refusal).
`check_params:=false` skips it.

To start from your own Nav2 file: replace its `collision_monitor` section by
the one of `params/nav2_params.yaml` and fit the zones.

#### Which zones, and what was measured

Neither file does what `approach` does. The choice between them:

- **`zones:=stop_slowdown`** (default). A stop zone, 0.20 m beyond the
  assumed footprint ahead and 0.05 m to the sides and behind, inside a
  slowdown zone, 0.60 m ahead and 0.10 m to the sides and behind. While six
  readings are in the slowdown zone every command is multiplied by 0.4;
  while six are in the stop zone every command becomes zero. That stops the
  robot before an obstacle, and it also answers with zero the turn or the
  retreat that would clear the zone, and Nav2's recovery behaviours. Under
  a planner the robot can end up standing for good: in the tours of
  [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), section
  7.5, it did in three runs of six (this node: 1, 1 and 8 places reached,
  standing at the end of two runs; upstream's node on the same zones: 8, 2
  and 5, standing in one; stock Nav2 with `approach`, one run: 11 places,
  never standing).
- **`zones:=slowdown`**. The slowdown zone alone. The monitor never stops
  the robot. Keeping it off obstacles is left to Nav2's planner and
  controller (the costmaps). If they command a collision, the monitor lets
  it happen at 0.4 times the speed.

**What was measured with the slowdown zone alone.** In the simulated
apartment of the tutorial, Nav2 drove a TurtleBot3 on the tour of five
places for 300 simulated seconds per run, started by this package's
`bringup_launch.py` (`pixi run nav2-bringup-tour --zones slowdown`, which is
`nav2_observe.py tour --bringup roslean_nav2_bringup --zones slowdown`). The
parameters were the demo's for that robot (radius 0.105 m, the slowdown
zone of the demo's `nav2_monitor.yaml`), not this package's example file.
Three runs, one after the other:

| Run | Places reached | Distance driven | Time the monitor answered non-zero commands with zero | At the end | Closest to obstacles |
| --- | --- | --- | --- | --- | --- |
| 1 | 3 | 21.3 m | 0 s | standing (0.03 m in the last 30 s) at (0.91, 1.84), at the edge of the dining table and its chairs; Nav2's localisation had the robot at (2.2, −1.7), 3.7 m from where it was, and the planner found no path | smallest lidar reading 0.120 m, the lidar's lower limit; for 50 s the robot's outline lay over cells the map marks as occupied |
| 2 | 1 | 14.6 m | 0 s | at (0.14, 2.69), 0.3 m from the second place, 0.50 m in the last 30 s: the planner failed 88 times to find a path to it | smallest lidar reading 0.154 m; outline 0.10 m from the nearest occupied cell |
| 3 | 6 | 27.9 m | 0 s | driving | smallest lidar reading 0.120 m; outline 0.01 m from the nearest occupied cell |

What the three runs show, and what they do not:

- The monitor never held the robot. Two runs of three still ended with the
  robot stuck, for Nav2's own reasons: lost localisation in the first, no
  path in the second.
- The robot came within centimetres of obstacles with nothing to stop it.
  In two runs something was at the lidar's lower limit of 0.12 m, where the
  body is 0.03 to 0.05 m from it, and in the first run the robot ended
  inside the area the map reserves for the table and chairs.
- Whether the robot touched anything is not known: the simulation has no
  contact sensor. In no run was it driven for a second without moving.
- There is no baseline. The last column is new; no run with `approach` or
  with the stop zone has it, so it is not known how close those let the
  robot come on this tour. The runs with the stop zone and with `approach`
  quoted above were made earlier, on another day.
- Three runs, one machine (macOS arm64, load average 2.5 to 6.5 while
  another session used it), the TurtleBot3's zones.

#### Check that it is this node

The launch prints
`[roslean_nav2_bringup] collision monitor: roslean_collision_monitor; parameters: …`,
and everything under [Check which node is running](#check-which-node-is-running)
applies: the node's line `Monitoring 'cmd_vel_smoothed' -> 'cmd_vel' …`, the
path of the executable
(`…/lib/roslean_collision_monitor/collision_monitor`), and composed the
container's `Load Library: …/libcollision_monitor_component.so` (`.dylib` on
macOS).

#### What the package depends on

`package.xml` names `nav2_bringup`, `roslean_collision_monitor` and what the
launch files import. `colcon build --packages-up-to roslean_nav2_bringup`
builds `roslean_core`, `roslean_collision_monitor` and this package; no
simulator is among the dependencies.

### Nav2's default parameters: the expected refusal

With either option, Nav2's unedited `nav2_params.yaml` gives this, and it is
what should happen:

```
[lifecycle_manager_navigation]: Configuring velocity_smoother
[lifecycle_manager_navigation]: Configuring collision_monitor
[collision_monitor]: 2 parameters refused: parameter 'FootprintApproach.action_type': only 'stop'
  and 'slowdown' are supported; got 'approach'. (To use Nav2's footprint zone, give it 'points', or
  'action_type: stop' together with 'polygon_sub_topic'; 'footprint_topic' is read for 'approach'
  only.); parameter 'FootprintApproach.polygon_sub_topic': is not initialized, and the zone has no
  'points': static points and sub topic both not defined. 'footprint_topic' is read for
  'action_type: approach' only, which this node does not provide; give the zone 'points'
  (recommended), or set 'polygon_sub_topic' to the topic the footprint is published on
[lifecycle_manager_navigation]: Failed to change state for node: collision_monitor
[lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

(`ros2 launch nav2_bringup navigation_launch.py` with the overlay sourced
and no `params_file`; the error is one line, wrapped here.) The node stays
unconfigured and its process stays up. The lifecycle manager gives up: the
six nodes before `collision_monitor` in its list have been configured and
none is activated, and the nodes after it (`bt_navigator`,
`waypoint_follower`, `docking_server`) are never configured. Upstream's node
accepts the same file. The way out is the edit of the parameters described under
[Check your parameters first](#check-your-parameters-first), or going back
to upstream's node.

### The collision detector

`nav2_collision_monitor` also contains a second node, the collision
detector (`collision_detector`, component
`nav2_collision_monitor::CollisionDetector`, launch file
`collision_detector_node.launch.py`). This repository has no such node.

With option B nothing changes for it. With option A the overlay hides
upstream's package, so it carries three forwarders:

- `ros2 run nav2_collision_monitor collision_detector` runs a script that
  finds upstream's executable in the other prefixes of `AMENT_PREFIX_PATH`
  and replaces itself with it. Without upstream it exits with code 1 and a
  message.
- `ros2 launch nav2_collision_monitor collision_detector_node.launch.py`
  includes upstream's launch file of that name, with upstream's default
  parameter file.
- The component `nav2_collision_monitor::CollisionDetector` is listed in the
  overlay's entry of the component index with the **absolute path** of
  upstream's library, as found when the overlay was built. If upstream was
  not installed then, or is moved afterwards, the component cannot be loaded
  while the overlay is sourced; rebuild the overlay.

## Check which node is running

```sh
ros2 pkg prefix nav2_collision_monitor      # option A: your workspace, not /opt/ros/jazzy
ros2 pkg xml nav2_collision_monitor -t version    # option A: 0.1.0; upstream: 1.3.12
ros2 node info /collision_monitor | grep -e collision_points_marker -e toggle   # no output: the Lean-backed node
ps -o command= -C collision_monitor         # Linux, not composed: the path of the executable
```

When it is configured, the Lean-backed node writes one line,

```
[collision_monitor]: Monitoring 'cmd_vel_smoothed' -> 'cmd_vel' (Twist) with scans from 'scan' and 1 zone(s).
```

where upstream writes `Configuring` and a line `[<name>]: Creating …` for
each source and zone. In a composed bring-up the container names the library
it loads: `Load Library: …/libroslean_collision_monitor_overlay.so` (the
overlay), `…/libcollision_monitor_component.so` (`roslean_collision_monitor`)
or `…/libcollision_monitor_core.so` (upstream); `.dylib` on macOS.

`ros2 param list /collision_monitor` differs too: this node lists the
parameters your file sets, upstream lists the ones it reads, including those
your file leaves at their defaults (`<zone>.max_points`,
`<zone>.sources_names`, `<zone>.polygon_subscribe_transient_local`,
`<source>.source_timeout`, `enable_stamped_cmd_vel`,
`use_realtime_priority`).

Nav2's route server logs `Created route operation CollisionMonitor of type
nav2_route::CollisionMonitor`. That is a plugin of `nav2_route` with the same
class name, not this node, and neither option touches it.

## Go back

- Option A: do not source the workspace, or delete
  `install/nav2_collision_monitor` and `build/nav2_collision_monitor` and
  open a new shell. Then `ros2 pkg prefix nav2_collision_monitor` prints
  `/opt/ros/jazzy` again. A later `colcon build` does not bring the overlay
  back, because it is built only when its directory is named.
- Option B: change the names back.
- Option C: launch `nav2_bringup` instead of `roslean_nav2_bringup`.

Upstream's package is never modified or removed by either option. The edit
of the parameters (a stop zone in the place of `approach`) is separate: it
is in your file, and upstream's node accepts the edited file as well as the
original.

## Differences from upstream

They are not repeated here. Where to read them:

- what is not provided, and what the node does when asked for it:
  [collision_monitor/SPEC.md, "Excluded features"](collision_monitor/SPEC.md#excluded-features);
- what is done differently on purpose, D1 to D15 and ND1 to ND13 (zones
  grown by up to 4.4 mm, negative slowdown ratios refused, a zone fed by a
  topic after an unusable message, no output after shutdown, and others):
  [collision_monitor/SPEC.md, "Deliberate differences in ROSlean"](collision_monitor/SPEC.md#deliberate-differences-in-roslean);
- what separates the node from Nav2's default configuration, feature by
  feature, with what closing each gap would take:
  [collision_monitor/SPEC_NAV2.md](collision_monitor/SPEC_NAV2.md), section 12;
- a stop zone under Nav2's planner, measured:
  [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), sections
  7.5 and 7.6;
- how the node compares with upstream's in the tests and in run time:
  [collision_monitor/COMPARISON.md](collision_monitor/COMPARISON.md);
- a summary for readers: [site/gaps.html](../site/gaps.html).

Differences of the packaging only:

- **Version.** The overlay's `package.xml` says 0.1.0, not 1.3.12.
- **Dependencies.** Upstream's package depends on `nav2_costmap_2d`,
  `visualization_msgs` and `std_msgs`; the overlay does not, and it depends
  on `roslean_core`.
- **No exported library, no headers** (option A, above).
- **Two names for one component** in the overlay's library: upstream's and
  `roslean_collision_monitor::CollisionMonitorNode`. `ros2 component types`
  lists upstream's.
- **The example parameter file** lacks upstream's unused example zones
  (`PolygonLimit`, `FootprintApproach`, `VelocityPolygonStop`) and the
  unused `pointcloud` source.

## What was tested

On 2026-11-25, and where a row says so on 2026-11-26, on macOS arm64 with ROS 2 Jazzy from RoboStack (pixi) and
upstream `nav2_collision_monitor` and `nav2_bringup` 1.3.12. Nothing on this
page was run on Linux. No simulator and no robot: where Nav2 was started,
the transforms were static (`map` → `odom` → `base_footprint` → `base_link`
→ `base_scan`), there was no scan, no map, no localisation and no goal, so
"active" below means that Nav2's lifecycle manager configured and activated
the node and formed the bond with it, not that a robot was protected.

| What | Result |
| --- | --- |
| `ros2 run <package> collision_monitor` with a file of one stop polygon, one slowdown polygon and one slowdown circle on one scan; `configure`, `activate`, `deactivate`, `cleanup` by `ros2 lifecycle set`: upstream, `roslean_collision_monitor`, the overlay | All transitions successful in all three. `ros2 node info`: the two Lean-backed packages identical; against upstream, `~/collision_points_marker` and `~/toggle` are missing and nothing else differs (topics, types, bond, lifecycle services). `ros2 topic info -v` of the output topic: identical QoS. `ros2 param list`: as described under [Check which node is running](#check-which-node-is-running) |
| `ros2 launch <package> collision_monitor_node.launch.py use_composition:=False`, with that file and with the package's default file: upstream (that file only), `roslean_collision_monitor`, the overlay | Lifecycle manager: `Managed nodes are active`; node active and bonded |
| The same launch file with `use_composition:=True` and a `nav2_container` started by hand: `roslean_collision_monitor`, the overlay | Component loaded under each package's plugin name; active and bonded |
| `ros2 launch nav2_bringup navigation_launch.py`, unedited, with `nav2_params.yaml` in which the zone is the fixed stop polygon of SPEC_NAV2.md 8.6, `use_composition:=False`: upstream only, then with the overlay sourced | Upstream: `collision_monitor` is upstream's executable, all managed nodes active. Overlay: `collision_monitor` is the overlay's executable, all managed nodes active, bonded. Repeated with the overlay built in a separate workspace without `--symlink-install` |
| The same with `use_composition:=True` and a `nav2_container` started as `bringup_launch.py` starts it | Upstream: `libcollision_monitor_core` loaded. Overlay: `libroslean_collision_monitor_overlay` loaded for `nav2_collision_monitor::CollisionMonitor`; all managed nodes active, bonded |
| `ros2 launch nav2_bringup navigation_launch.py`, unedited, with Nav2's unedited `nav2_params.yaml`, overlay sourced, with and without composition | Refused as shown under [Nav2's default parameters](#nav2s-default-parameters-the-expected-refusal); node `unconfigured`; bring-up aborted |
| This repository's `smoke_node.py` (pass-through, slowdown, stop, refused configurations, under simulated time) and `bond_manager.py` (bond with Nav2's lifecycle manager, the manager's reaction to the node's death) on the overlay's executable | 0 failures |
| The collision detector through the overlay: `ros2 run`, `collision_detector_node.launch.py use_composition:=False`, `ros2 component load` of `nav2_collision_monitor::CollisionDetector` | Upstream's executable runs; the node becomes active; upstream's library is loaded |
| `colcon build` in a separate workspace that contains a copy of this repository: the command of option A; then the command of option B | 2 packages; 3 packages (the third was `roslean_twist_mux`, while the demos were in `roslean_collision_monitor`; not repeated since). A plain `colcon list` does not show the overlay |
| The guard of the tests: `roslean_collision_monitor` configured with the overlay sourced; a test script asked for `nav2_collision_monitor/collision_monitor` with the overlay sourced | CMake warns and registers no test against upstream; the script exits with a message |
| `check_params.py` on Nav2's `nav2_params.yaml` and `nav2_multirobot_params_1.yaml`, upstream's `collision_monitor_params.yaml`, and the parameter files of this repository | Refusal reported for the first two, none for the others, as the node decides |
| `roslean_nav2_bringup`: `ros2 launch roslean_nav2_bringup navigation_launch.py`, with each of the two parameter files, each with `use_composition` `False` (default) and `True` (with a `nav2_container` started by hand) | `[roslean_nav2_bringup] collision monitor: roslean_collision_monitor; parameters: …; nothing unsupported found`; `Managed nodes are active`; the node's `Monitoring …` line with 1 and 2 zones; stand-alone: the process is `…/lib/roslean_collision_monitor/collision_monitor`, `active`, bonded; composed: `Load Library: …/libcollision_monitor_component.dylib` |
| `ros2 launch roslean_nav2_bringup bringup_launch.py map:=…` with the demo's map (static transforms, no scan): with the slowdown file; with `use_composition:=True` and the stop and slowdown file; `slam:=True` without a map | `lifecycle_manager_localization` (with `slam:=True`: `lifecycle_manager_slam`) and `lifecycle_manager_navigation`: `Managed nodes are active`; the monitor as in the row above |
| The same two launch files with Nav2's own `nav2_params.yaml` as `params_file` | The message under [Your own parameter file](#your-own-parameter-file), exit code 1, no node started. With `check_params:=false`: the node's own refusal and `Aborting bringup`, as under [Nav2's default parameters](#nav2s-default-parameters-the-expected-refusal) |
| `bringup_launch.py` of `roslean_nav2_bringup` in the simulated apartment (Gazebo without a window, TurtleBot3, the demo's parameters for that robot), Nav2 driving the tour, 300 simulated seconds, slowdown zone only, three runs | The table under [Which zones, and what was measured](#which-zones-and-what-was-measured) |
| The tests of `roslean_nav2_bringup` (`params_follow_nav2`, `derive`): the parameter files against the installed `nav2_params.yaml`; the edits on the installed launch files, and on altered texts | Passed; an altered text is refused |
| On 2026-11-26, with the demos in `roslean_demos`: `colcon build --packages-up-to roslean_nav2_bringup` in a separate workspace that contains a copy of this repository | 3 packages: `roslean_core`, `roslean_collision_monitor`, `roslean_nav2_bringup` |
| The same day: `ros2 launch roslean_nav2_bringup navigation_launch.py` without `zones`, and with `zones:=slowdown` | `Monitoring … 2 zone(s)` and `… 1 zone(s)`; `Managed nodes are active`; the node `active` |
| The same day, the demos from `roslean_demos`: `pixi run demo` (turtlesim without a window, 25 s), `pixi run cm-check`, `pixi run cm-check --nav2`, `pixi run nav2-bringup-sim gui:=false` (90 s) | The narrator's `planner drives` line; `PASSED` twice; Nav2 from `roslean_nav2_bringup` active with 2 zones and the tour reaching its first place |
| The same day: the repository's short test set (`pixi run test-quick`), six packages | 19 of 19 passed, in 5 min 37 s |
| The same day: the repository's tests (`pixi run test`), six packages | 31 of 31 passed, in 31 min 18 s |
| The repository's tests (`pixi run test`), after the change, the overlay not sourced | 28 of 29 passed, among them all 25 of `roslean_collision_monitor`. The one failure was the `scenario` test of `roslean_twist_mux`, which this change does not touch: one message was missing while another program had the machine at a load of about 40; run again alone it passed twice. In an earlier run of the short set (`pixi run test-quick`), `upstream_probes`, which drives upstream's node alone, failed on a source timeout probe; repeated, it failed once more and passed four times |

Not tested: Linux (the node itself and its tests were run on Linux arm64
earlier, see the [README](../README.md); the overlay package, the new launch
file and this procedure were not); any ROS distribution other than Jazzy;
Nav2's own `bringup_launch.py` as a whole through the overlay (it starts
localisation and needs a map; the one of `roslean_nav2_bringup` was run);
of `roslean_nav2_bringup`: `slam:=True` with a scan, a namespace, `use_respawn`, the
package's own parameter files (the example zones for a radius of 0.22 m)
with a robot or a simulator, the stop zone in the simulator through
this package, any `nav2_bringup` other than 1.3.12;
Nav2 with a simulator or a robot through the overlay (the simulated runs of
the tutorial use the renamed package and an edited copy of
`navigation_launch.py`); a namespaced bring-up; `rosdep install` on these
packages; copying the install tree to another machine; a merged install
(`colcon build --merge-install`); a workspace that contains Nav2's sources;
the Docker image.

## What is not done

- **Nav2's default configuration.** `approach` is not provided, nor `limit`,
  velocity polygons, point cloud, range and polygon sources, several
  sources, `~/toggle`, run-time parameters. What each would take:
  [collision_monitor/SPEC_NAV2.md](collision_monitor/SPEC_NAV2.md), section 12.
- **The collision detector.** Not reimplemented; upstream's is used.
- **The overlay in the Docker image.** `docker/Dockerfile` builds
  `roslean_collision_monitor` and runs its tests against apt's
  `ros-jazzy-nav2-collision-monitor`. To add the overlay: after the tests
  (they compare with the package called `nav2_collision_monitor`, which until
  then must be upstream's), `COPY overlay overlay` and
  `colcon build --base-paths overlay/opt_in/nav2_collision_monitor
  --build-base build_overlay --install-base /opt/roslean`; in the final
  image, install `ros-jazzy-nav2-common` (for
  `collision_monitor_node.launch.py`), `ros-jazzy-nav2-collision-monitor`
  (for the detector) and whatever of Nav2 is to be launched. Not written and
  not run.
- **Other ROS distributions.** Only Jazzy; the node follows
  `nav2_collision_monitor` 1.3.12.
- **Building without Lean, a `.deb`, a conda package.** Not tried. Compiling
  the C that Lean generates also needs Lean's headers and its runtime, so a
  build without elan would have to carry those per platform; a release
  through the ROS build farm is not possible as long as Lean is not
  available there. The overlay package cannot be a `.deb` installed into
  `/opt/ros/jazzy`: its files would collide with
  `ros-jazzy-nav2-collision-monitor`.
