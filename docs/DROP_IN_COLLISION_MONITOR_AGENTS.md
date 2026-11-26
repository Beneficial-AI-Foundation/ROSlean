# Replacing `nav2_collision_monitor` with the Lean-backed node: procedure for coding agents

Audience: a coding agent asked to make a ROS 2 workspace use the node of this
repository instead of upstream `nav2_collision_monitor`. Background and the
list of what was tested: [DROP_IN_COLLISION_MONITOR.md](DROP_IN_COLLISION_MONITOR.md).

**The node is not a replacement for Nav2's default configuration.** It
provides `stop` and `slowdown` zones on one laser scan and refuses every
other configuration when it is configured. Nav2's `nav2_params.yaml` has one
zone with `action_type: approach` and is refused. Step 2 checks the user's
parameters, before anything is changed. If that check finds something, you
stop and ask (step 0, case 1). You do not edit the user's zones on your own.

Conventions: `$WS` is the root of the user's colcon workspace (it has `src/`).
`$ROSLEAN` is the checkout of this repository, normally `$WS/src/ROSlean`.
Every step has a check. If a check fails and the step does not say what to do,
stop and report the command and its output to the human. Do not edit files
outside `$WS`, apart from the temporary files under `/tmp` that the steps
name. Do not use `sudo`.

## 0. When to stop and ask the human

Stop before changing anything, and ask, in each of these cases:

1. **The parameters use something the node does not provide** (step 2):
   `action_type` other than `stop` or `slowdown` (`approach`, `limit`,
   `none`); a zone of type `velocity_polygon`; a circle without `radius`;
   a source of type `pointcloud`, `range` or `polygon`; `observation_sources`
   with more or fewer than one name; `max_points`; `sources_names`;
   `use_realtime_priority: true`; a negative `slowdown_ratio`. Report the
   lines that `check_params.py` printed. Replacing an `approach` zone by a
   `stop` zone changes how the robot behaves near obstacles (the robot can
   end up standing for good; see
   [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), section
   7.6). That decision, and the size of the zone, are the human's. Offer the
   documented edit ([collision_monitor/SPEC_NAV2.md](collision_monitor/SPEC_NAV2.md),
   section 8.6); do not apply it unasked.
2. **No parameter file for the collision monitor can be found**, or the
   bring-up is started without `params_file` (then Nav2's default file is
   used, which is case 1).
3. `$ROS_DISTRO` is not `jazzy` (step 1).
4. The workspace contains the source of Nav2's `nav2_collision_monitor`
   (step 3). Two packages of one name cannot be built together; option A is
   then not available without the human's decision.
5. Some package compiles against upstream's library or headers:
   `find_package(nav2_collision_monitor` in a `CMakeLists.txt`, or
   `#include "nav2_collision_monitor/…"` (step 3). The overlay exports
   neither. Option B remains possible.
6. Something calls the service `collision_monitor/toggle`, sets parameters
   of the collision monitor at run time (`ros2 param set`, a parameter
   client, `enabled` of a zone or source), or subscribes to
   `collision_monitor/collision_points_marker` (step 3). The node has none
   of these.
7. The collision monitor is started by something that does not source
   `$WS/install` (a systemd unit, a vendor start-up script, a container
   image). Option A has no effect there.
8. `elan` is not installed and you are not allowed to install software into
   the user's home directory (step 4).
9. The node's `configure` transition fails (steps 5 and 8). The node prints
   one error line that names every refused parameter. Report it. Do not
   change the user's configuration to make it pass.
10. The bring-up runs composed (`use_composition:=True`, the default of
    `bringup_launch.py`) and the human has not been told that the
    stand-alone executable is the form with every property claimed (as a
    component the node ends with its container on a `/clock` message with a
    negative time, as upstream's does). Recommend `use_composition:=False`;
    the choice is the human's.
11. The human wants the change on a physical robot. Nothing here was tested
    on hardware; say so.
12. The human has not chosen between option A (overlay), option B
    (renamed package) and option C (the bring-up package, step 7a). Option
    A changes which node every Nav2 bring-up launched from that workspace
    starts, not only the one you were asked about, and a bring-up with
    Nav2's default parameters then no longer comes up. Option C is for a
    human who starts Nav2 with `nav2_bringup`'s own launch files and has no
    zones of their own: it brings its own zones, which are an example and
    are not Nav2's `approach` zone. Explain that and ask.
13. Option C, and the human has not been told that its default parameter
    file has **no stop zone** (the monitor slows the robot near obstacles
    and never stops it), that `zones:=stop_slowdown` adds one with the
    consequence of case 1, and that the zone coordinates are an example for
    a robot of radius 0.22 m. Which zones, and their size, are the human's
    decision.

## 1. Preconditions

```sh
echo "$ROS_DISTRO"                                  # expected: jazzy
ros2 pkg prefix nav2_collision_monitor              # expected: a path, usually /opt/ros/jazzy
ros2 pkg xml nav2_collision_monitor -t version      # expected: 1.3.x (tested with 1.3.12)
for p in nav2_util nav2_msgs nav2_common nav2_lifecycle_manager; do ros2 pkg prefix $p >/dev/null || echo "missing: $p"; done   # expected: no output
uname -sm                                           # tested: "Darwin arm64"
```

If `ros2 pkg prefix nav2_collision_monitor` fails, upstream is not
installed. That is allowed; then the collision detector will not be
available (note it in your report). If the path it prints is inside `$WS`
and `ros2 pkg xml nav2_collision_monitor -t version` prints `0.1.0`, the
overlay is already built and sourced: say so and go to step 8.

Record the current interface, to compare in step 8. With the user's
collision monitor running and active as they normally start it:

```sh
ros2 node info /collision_monitor > /tmp/cm_before.txt       # use the node's real name if it is namespaced
ros2 param list /collision_monitor > /tmp/cm_params_before.txt
ros2 lifecycle get /collision_monitor                         # expected: active [3]
```

If you cannot run it (no robot, no simulation), skip this and say so.

## 2. Check the parameters (before anything is changed)

Find the parameter files that configure the collision monitor:

```sh
cd $WS
grep -rlI --include='*.yaml' --include='*.yml' -e 'observation_sources' -e 'polygons:' src | grep -v '/ROSlean/'
```

and ask the human which file their bring-up passes as `params_file` if that
is not evident from their launch files. A bring-up that passes none uses
`$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/params/nav2_params.yaml`.

The check needs the checkout but no build:

```sh
git clone -b collision-monitor https://github.com/Beneficial-AI-Foundation/ROSlean.git /tmp/ROSlean-check   # or use $ROSLEAN if it exists
python3 /tmp/ROSlean-check/ros/roslean_collision_monitor/scripts/check_params.py FILE.yaml
echo $?
```

| Output | Exit code | Meaning |
| --- | --- | --- |
| `FILE.yaml: /collision_monitor: nothing unsupported found` | 0 | Go on. |
| `FILE.yaml: /collision_monitor: NOT ACCEPTED by the Lean-backed node:` and one line per parameter | 1 | **Stop** (case 1). |
| `FILE.yaml: no collision monitor section found` | 2 | Not the right file. Find the right one, or stop (case 2). |

For Nav2's unedited `nav2_params.yaml` the output is:

```
…/nav2_params.yaml: /collision_monitor: NOT ACCEPTED by the Lean-backed node:
    FootprintApproach.action_type: approach (only stop and slowdown)
```

The script does not see parameters set in a launch file (`parameters=[{…}]`,
`RewrittenYaml(param_rewrites=…)`) or on the command line, and it does not
check numbers. Look for such overrides of the collision monitor in the
launch files; step 5 is the check that counts.

## 3. Find the uses of `nav2_collision_monitor` in the workspace

```sh
cd $WS
colcon list | awk '$1 == "nav2_collision_monitor"' | grep -v '/ROSlean/'       # expected: no output
grep -rnI --include=package.xml 'nav2_collision_monitor' src | grep -v '/ROSlean/'
grep -rnI --include='*.py' --include='*.xml' --include='*.yaml' --include='*.yml' \
  -e 'nav2_collision_monitor' -e 'collision_detector' src | grep -v '/ROSlean/'
grep -rnI --include='CMakeLists.txt' --include='*.cmake' --include='*.cpp' --include='*.hpp' --include='*.h' \
  -e 'nav2_collision_monitor' src | grep -v '/ROSlean/'
grep -rnI -e 'collision_monitor/toggle' -e 'collision_points_marker' -e 'nav2_msgs/srv/Toggle' -e 'srv import Toggle' src | grep -v '/ROSlean/'
grep -rnI -e 'use_composition' -e 'bringup_launch.py' -e 'navigation_launch.py' src | grep -v '/ROSlean/'
```

Interpretation:

- First command prints a line: stop (case 4).
- Launch files with `package='nav2_collision_monitor'` and
  `executable='collision_monitor'`, with
  `plugin='nav2_collision_monitor::CollisionMonitor'`, or with
  `get_package_share_directory('nav2_collision_monitor')` /
  `FindPackageShare('nav2_collision_monitor')`: these are the places option B
  edits. Option A edits none.
- `collision_detector` (executable, plugin `…::CollisionDetector`, or
  `collision_detector_node.launch.py`): the detector. It stays upstream's;
  never change its package name. Note that it is in use.
- The CMake/C++ grep prints something: stop (case 5).
- The toggle/marker grep prints something that concerns the collision
  monitor: stop (case 6).
- The last grep tells you how Nav2 is started. If the collision monitor is
  started by `nav2_bringup`'s own launch files (installed under `/opt/ros`,
  not in `src`), option A applies without edits; option B requires copies of
  `navigation_launch.py` and of whatever includes it in `src`, which is the
  human's decision. Note the value of `use_composition` (case 10).

## 4. Get and build

```sh
command -v lake || ls ~/.elan/bin/lake         # expected: a path
```

If neither exists, and you may install into the home directory:

```sh
curl -sSf https://elan.lean-lang.org/elan-init.sh | sh -s -- -y --default-toolchain none
export PATH="$HOME/.elan/bin:$PATH"
```

Then:

```sh
cd $WS/src && git clone -b collision-monitor https://github.com/Beneficial-AI-Foundation/ROSlean.git
cd $WS && source /opt/ros/jazzy/setup.bash
# option A:
colcon build --base-paths src src/ROSlean/overlay/opt_in/nav2_collision_monitor \
  --packages-up-to nav2_collision_monitor
# option B:
# colcon build --packages-up-to roslean_collision_monitor
# option C:
# colcon build --packages-up-to roslean_nav2_bringup
```

Expected last lines: `Summary: 2 packages finished` (`roslean_core` and
`nav2_collision_monitor`); with option B, `Summary: 3 packages finished`
(`roslean_core`, `roslean_twist_mux`, `roslean_collision_monitor`; the
second is there because the demos in the third depend on it); with option
C, `Summary: 4 packages finished` (those three and `roslean_nav2_bringup`).
The first
build downloads the Lean toolchain pinned in `$ROSLEAN/lean-toolchain`; it
needs network access. Warnings from `install_name_tool` on macOS are
harmless.

The overlay is built only by the command of option A, which names its
directory: `$ROSLEAN/overlay/opt_in/COLCON_IGNORE` keeps colcon from finding
it otherwise. Do not delete that file. A later `colcon build` in `$WS`
neither removes nor rebuilds the overlay; after updating `$ROSLEAN`, repeat
the command.

With option A, the configure output must contain

```
-- nav2_collision_monitor (ROSlean overlay): the component nav2_collision_monitor::CollisionDetector is upstream's, /opt/ros/jazzy/lib/libcollision_detector_core.so
```

(`colcon build --event-handlers console_direct+` shows it; it is also in
`$WS/log/latest_build/nav2_collision_monitor/stdout.log`). A CMake warning
`upstream nav2_collision_monitor was not found` instead means the detector's
component will not be loadable while the overlay is sourced; report it if
step 3 found the detector in use.

Do not use `--symlink-install` if the install tree is to be copied to
another machine. Do not run `colcon build` without `--packages-up-to`
unless the human wants all packages of the repository.

If the build fails in `roslean_core` with `lake` or `lean` not found, `PATH`
lacks `~/.elan/bin`. If it fails there on a read-only source directory: the
build writes `$ROSLEAN/core/.lake`; the checkout must be writable. Any other
failure: stop and report.

## 5. Dry run: let the node judge the parameters

Before the user's bring-up is switched, configure the node on the user's
file in a ROS domain that nothing else uses. It is configured, never
activated, so it publishes no velocity.

```sh
source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash
export ROS_DOMAIN_ID=99                 # any domain the robot does not use
PKG=nav2_collision_monitor              # option B: roslean_collision_monitor
$(ros2 pkg prefix $PKG)/lib/$PKG/collision_monitor --ros-args --params-file FILE.yaml > /tmp/cm_dry.log 2>&1 &
pid=$!
sleep 4
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle get /collision_monitor
kill $pid; ros2 daemon stop; unset ROS_DOMAIN_ID
grep -e ERROR /tmp/cm_dry.log
```

Expected: `Transitioning successful`, `inactive [2]`, and no output from the
`grep`. `Transitioning failed`, `unconfigured [1]` and a line

```
[ERROR] [...] [collision_monitor]: 2 parameters refused: parameter '…': …; parameter '…': …
```

mean the file is refused: stop (case 9). (The count and the parameters are
those of your file; the example is the form of the line for Nav2's default
file.) If the user's node runs in a
namespace or under another name, add `-r __ns:=/their_ns` or
`-r __node:=their_name` so that the same section of the file is read, and
use that name in the `ros2 lifecycle` command.

## 6. Option A: overlay, no edits

Nothing to edit. Go to step 8.

## 7. Option B: rename the package where the monitor is started

For each place found in step 3, and only for the collision monitor itself:

| File kind | Replace | With |
| --- | --- | --- |
| Launch (Python) | `package='nav2_collision_monitor'` on the `Node` action whose `executable` is `'collision_monitor'` | `package='roslean_collision_monitor'` |
| Launch (Python) | `package='nav2_collision_monitor'`, `plugin='nav2_collision_monitor::CollisionMonitor'` on a `ComposableNode` | `package='roslean_collision_monitor'`, `plugin='roslean_collision_monitor::CollisionMonitorNode'` |
| Launch (Python) | `get_package_share_directory('nav2_collision_monitor')` or `FindPackageShare('nav2_collision_monitor')` used for `collision_monitor_node.launch.py` or `collision_monitor_params.yaml` | the same with `'roslean_collision_monitor'` |
| Launch (XML/YAML) | `pkg="nav2_collision_monitor" exec="collision_monitor"` | `pkg="roslean_collision_monitor" exec="collision_monitor"` |
| `package.xml` of a package you edited | `<exec_depend>nav2_collision_monitor</exec_depend>` | add `<exec_depend>roslean_collision_monitor</exec_depend>`; keep the `nav2_collision_monitor` line if that package starts the detector |

Do not change: `executable='collision_monitor'`, `name=…`, `namespace=…`,
`parameters=…`, `remappings=…`, the lifecycle manager's `node_names`, any
yaml file, anything that starts `collision_detector`.

Rebuild the packages you edited (`colcon build --packages-select <names>`).

## 7a. Option C: the bring-up package

Nothing to edit. Where the human started `ros2 launch nav2_bringup
bringup_launch.py …` or `navigation_launch.py …`, the command becomes
`ros2 launch roslean_nav2_bringup …` with the same arguments. Differences
to tell the human, from
[DROP_IN_COLLISION_MONITOR.md](DROP_IN_COLLISION_MONITOR.md#option-c-the-bring-up-package):
`use_composition` defaults to `False` in both files; without `params_file`
the package's own file is used (`zones:=slowdown`, the default, or
`zones:=stop_slowdown`); with `params_file` the file is checked first.

```sh
ros2 launch roslean_nav2_bringup bringup_launch.py --show-args | grep -A2 -e "'zones'" -e "'check_params'"
# expected: both arguments, zones with the choices ['slowdown', 'stop_slowdown'] and the default 'slowdown'
ros2 run roslean_nav2_bringup sync_params.py
# expected, for each of the two files: "... equal to .../nav2_bringup/params/nav2_params.yaml outside the collision_monitor section"
```

If `sync_params.py` reports a difference, the installed Nav2 has other
defaults than the ones the package's files were made from: report it, and
do not launch on a robot.

If the launch ends at once with `roslean_nav2_bringup: cannot derive a
launch file from …`, the installed `nav2_bringup` is not the version the
package was written for (1.3.12). Nothing was started. Stop and report the
message; do not copy or edit launch files to get around it.

If it ends with `… would refuse the parameter file …`, that is case 1 of
step 0 for the file given as `params_file`.

Then step 8; the expected first line of the launch output is

```
[roslean_nav2_bringup] collision monitor: roslean_collision_monitor; parameters: <file> (<zones:=… or given as params_file>); collision monitor section: nothing unsupported found
```

## 8. Verify

In a new shell:

```sh
source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash
```

Option A:

```sh
ros2 pkg prefix nav2_collision_monitor            # expected: $WS/install/nav2_collision_monitor
ros2 pkg executables nav2_collision_monitor       # expected: two lines: collision_detector, collision_monitor
ros2 pkg xml nav2_collision_monitor -t version    # expected: 0.1.0
ros2 component types | grep -A2 '^nav2_collision_monitor'
# expected:   nav2_collision_monitor::CollisionMonitor
#             nav2_collision_monitor::CollisionDetector     (missing if upstream was not found in step 4)
ros2 launch nav2_collision_monitor collision_monitor_node.launch.py --show-args
# expected arguments: namespace, use_sim_time, params_file, use_composition, container_name
```

Option B:

```sh
ros2 pkg executables roslean_collision_monitor    # expected: two lines: check_params.py, collision_monitor
ros2 component types | grep -A1 '^roslean_collision_monitor'   # expected: roslean_collision_monitor::CollisionMonitorNode
ros2 launch roslean_collision_monitor collision_monitor_node.launch.py --show-args   # the same five arguments
ros2 pkg prefix nav2_collision_monitor            # expected: NOT inside $WS (the overlay must not have been built)
```

Start the bring-up the way the user starts it. Expected in its output:

```
[lifecycle_manager_navigation]: Configuring collision_monitor
[collision_monitor]: Monitoring '<cmd_vel_in_topic>' -> '<cmd_vel_out_topic>' (Twist) with scans from '<scan topic>' and <n> zone(s).
[lifecycle_manager_navigation]: Activating collision_monitor
[collision_monitor]: Creating bond (collision_monitor) to lifecycle manager.
[lifecycle_manager_navigation]: Server collision_monitor connected with bond.
[lifecycle_manager_navigation]: Managed nodes are active
```

(other nodes' lines in between; the manager's name is
`lifecycle_manager_collision_monitor` with the package's own launch file;
`(TwistStamped)` with `enable_stamped_cmd_vel: true`.) The line `Monitoring …`
is the sign of the Lean-backed node; upstream writes `[collision_monitor]:
Configuring` and `[<name>]: Creating …` lines. Composed, the container also
writes

```
[nav2_container]: Load Library: $WS/install/nav2_collision_monitor/lib/libroslean_collision_monitor_overlay.so
[nav2_container]: Found class: rclcpp_components::NodeFactoryTemplate<nav2_collision_monitor::CollisionMonitor>
```

(option B: `…/roslean_collision_monitor/lib/libcollision_monitor_component.so`
and `…<roslean_collision_monitor::CollisionMonitorNode>`). If the library is
`libcollision_monitor_core.so`, upstream's node was loaded: the launching
shell does not have `$WS/install` sourced (case 7).

A line `[route_server]: Created route operation CollisionMonitor of type
nav2_route::CollisionMonitor` is unrelated: a plugin of `nav2_route`.

A line `[ERROR] [...] [collision_monitor]: … refused: …` followed by the
lifecycle manager's `Failed to change state for node: collision_monitor` and
`Failed to bring up all requested nodes. Aborting bringup.`: stop (case 9).

Then, in another shell with the same environment:

```sh
ros2 lifecycle get /collision_monitor                # expected: active [3]
ros2 node info /collision_monitor > /tmp/cm_after.txt
diff /tmp/cm_before.txt /tmp/cm_after.txt
ros2 param list /collision_monitor > /tmp/cm_params_after.txt
diff /tmp/cm_params_before.txt /tmp/cm_params_after.txt
```

Expected in the first diff, exactly these two lines, present before and
absent after:

```
<     /collision_monitor/collision_points_marker: visualization_msgs/msg/MarkerArray
<     /collision_monitor/toggle: nav2_msgs/srv/Toggle
```

Any other difference there: stop and report it.

The second diff is not empty and is not a failure by itself. Upstream lists
the parameters it reads, including those the file leaves at their defaults;
the Lean-backed node lists the parameters the file sets. Expected: lines
only before for parameters the file does not set (`<zone>.max_points`,
`<zone>.sources_names`, `<zone>.polygon_subscribe_transient_local`,
`<source>.source_timeout`, `enable_stamped_cmd_vel`,
`use_realtime_priority`, and defaults such as `<zone>.enabled` or
`<zone>.visualize` where the file omits them), and lines only after for
parameters the file sets and upstream does not read (zones and sources that
the lists do not name, `min_height` and `max_height` on a scan). Report the
diff; stop if a parameter that the file sets for a zone or source named in
the lists is missing after.

Do not publish test commands on the monitor's input topic of a robot that
can move. A behaviour check without a robot is the repository's own, on the
executable you built, in a ROS domain that nothing else uses (about three
minutes):

```sh
ROS_DOMAIN_ID=99 python3 $ROSLEAN/ros/roslean_collision_monitor/test/smoke_node.py \
  $WS/install/nav2_collision_monitor/lib/nav2_collision_monitor/collision_monitor
```

(option B: `$WS/install/roslean_collision_monitor/lib/roslean_collision_monitor/collision_monitor`).
Expected last line: `0 failure(s)`.

## 9. Roll back

Option A:

```sh
rm -rf $WS/install/nav2_collision_monitor $WS/build/nav2_collision_monitor
# new shell, source as before
ros2 pkg prefix nav2_collision_monitor      # expected: /opt/ros/jazzy (or wherever upstream is)
ros2 pkg xml nav2_collision_monitor -t version   # expected: 1.3.x
```

Option B: revert the edits of step 7 (`git diff` in each edited repository
shows them) and rebuild those packages.

Option C: launch `nav2_bringup` again in the place of
`roslean_nav2_bringup`.

Removing everything: additionally delete `$WS/src/ROSlean`,
`$WS/install/roslean_*`, `$WS/build/roslean_*`. `~/.elan` is left to the
human. If the human changed their zones (case 1), that edit is theirs to
keep or revert: upstream's node accepts both versions of the file.

## 10. What to report to the human

- The output of the parameter check (step 2) and of the dry run (step 5).
- Which option, which files were edited (option B) or that none were.
- Option C: which zones file is used, that its coordinates are an example,
  and, for the default, that there is no stop zone.
- The outputs of the checks in step 8, including the two diffs.
- That `~/toggle`, run-time parameter changes and
  `~/collision_points_marker` are gone, and whether anything used them.
- Whether the bring-up runs composed, and that the stand-alone executable is
  the form with every property claimed.
- Whether the collision detector is in use and that it is still upstream's.
- If the zones were changed from `approach`: that this was the human's
  decision, and the reference to section 7.6 of the tutorial.
- Which checks were skipped and why.
- The platform, and that this procedure was tested only on the platforms
  listed in [DROP_IN_COLLISION_MONITOR.md](DROP_IN_COLLISION_MONITOR.md#what-was-tested).
