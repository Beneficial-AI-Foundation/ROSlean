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

The repository is in an exploratory phase. What was actually run is listed
under [What was tested](#what-was-tested); nothing here has been run on a
physical robot.

## What you get

Two packages provide the same node (the same source file, compiled twice):

| Package | In | Use it when |
| --- | --- | --- |
| `nav2_collision_monitor` | `overlay/opt_in/nav2_collision_monitor` | You want to edit no launch file. Built in your workspace, which takes a build command that names it, it shadows upstream's package of the same name: Nav2's `navigation_launch.py` and every other launch file that names `nav2_collision_monitor` then start the Lean-backed node. |
| `roslean_collision_monitor` | `ros/roslean_collision_monitor` | You prefer to see in your launch files which node runs. You change the package name (and, for a composed bring-up, the plugin name). |

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
colcon build --packages-up-to roslean_collision_monitor   # roslean_core, roslean_twist_mux, roslean_collision_monitor
```

`roslean_collision_monitor` also holds the demos and the tests, and its
`package.xml` names their dependencies: `roslean_twist_mux` (hence the third
package), Gazebo, the TurtleBot3 packages, `nav2_bringup` and upstream's
`nav2_collision_monitor`. `rosdep` on it installs all of them. The overlay
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

On 2026-11-25, on macOS arm64 with ROS 2 Jazzy from RoboStack (pixi) and
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
| `colcon build` in a separate workspace that contains a copy of this repository: the command of option A; then the command of option B | 2 packages; 3 packages. A plain `colcon list` does not show the overlay |
| The guard of the tests: `roslean_collision_monitor` configured with the overlay sourced; a test script asked for `nav2_collision_monitor/collision_monitor` with the overlay sourced | CMake warns and registers no test against upstream; the script exits with a message |
| `check_params.py` on Nav2's `nav2_params.yaml` and `nav2_multirobot_params_1.yaml`, upstream's `collision_monitor_params.yaml`, and the parameter files of this repository | Refusal reported for the first two, none for the others, as the node decides |
| The repository's tests (`pixi run test`), after the change, the overlay not sourced | 28 of 29 passed, among them all 25 of `roslean_collision_monitor`. The one failure was the `scenario` test of `roslean_twist_mux`, which this change does not touch: one message was missing while another program had the machine at a load of about 40; run again alone it passed twice. In an earlier run of the short set (`pixi run test-quick`), `upstream_probes`, which drives upstream's node alone, failed on a source timeout probe; repeated, it failed once more and passed four times |

Not tested: Linux (the node itself and its tests were run on Linux arm64
earlier, see the [README](../README.md); the overlay package, the new launch
file and this procedure were not); any ROS distribution other than Jazzy;
`bringup_launch.py` as a whole (it starts localisation and needs a map);
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
- **A package for the demos.** As long as the demos and the tests are in
  `roslean_collision_monitor`, that package depends on the simulator and on
  `roslean_twist_mux`.
- **Building without Lean, a `.deb`, a conda package.** Not tried. Compiling
  the C that Lean generates also needs Lean's headers and its runtime, so a
  build without elan would have to carry those per platform; a release
  through the ROS build farm is not possible as long as Lean is not
  available there. The overlay package cannot be a `.deb` installed into
  `/opt/ros/jazzy`: its files would collide with
  `ros-jazzy-nav2-collision-monitor`.
