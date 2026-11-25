# Replacing `twist_mux` with the Lean-backed node: procedure for coding agents

Audience: a coding agent asked to make a ROS 2 workspace use the node of this
repository instead of upstream `twist_mux`. Background and the list of what
was tested: [DROP_IN_TWIST_MUX.md](DROP_IN_TWIST_MUX.md).

Conventions: `$WS` is the root of the user's colcon workspace (it has `src/`).
`$ROSLEAN` is the checkout of this repository, normally `$WS/src/ROSlean`.
Every step has a check. If a check fails and the step does not say what to do,
stop and report the command and its output to the human. Do not edit files
outside `$WS`. Do not use `sudo`.

## 0. When to stop and ask the human

Stop before changing anything, and ask, in each of these cases:

1. `$ROS_DISTRO` is not `jazzy` (step 1).
2. The workspace contains the source of upstream `twist_mux` (step 2). Two
   packages of one name cannot be built together; which one to keep is the
   human's decision.
3. Something reads `/diagnostics` and expects an entry from `twist_mux`, for
   example a `diagnostic_aggregator` analyzer configuration that names it
   (step 2). The Lean-backed node does not publish diagnostics.
4. Some package compiles against upstream's headers (`#include <twist_mux/…>`)
   (step 2). The overlay installs no headers.
5. A parameter file sets `diagnostic_updater.*` for the mux and the human
   relies on it. The node accepts and ignores it.
6. `elan` is not installed and you are not allowed to install software into
   the user's home directory (step 3).
7. The node exits at start-up with a FATAL line (step 6). Typical causes: a
   timeout that is NaN, infinite or ≥ 9.2e9 s (refused on purpose, SPEC D5);
   a timeout written as an integer (`0` instead of `0.0`) or a missing field
   (refused by upstream as well). Do not "fix" the user's configuration
   without asking, except for nothing: report the line.
8. The mux is started by something that does not source `$WS/install`
   (a systemd unit, a vendor start-up script, a container image). Option A
   below has no effect there.
9. The human wants the change on a physical robot. Nothing here was tested on
   hardware; say so.

## 1. Preconditions

```sh
echo "$ROS_DISTRO"                       # expected: jazzy
ros2 pkg prefix twist_mux                # expected: a path (upstream installed), usually /opt/ros/jazzy
ros2 pkg xml twist_mux -t version        # expected: 4.5.0 or another 4.x
uname -sm                                # tested: "Darwin arm64", "Linux aarch64"; built earlier on "Linux x86_64"
```

If `ros2 pkg prefix twist_mux` fails, upstream is not installed. That is
allowed; then `twist_marker` and `joystick_relay.py` will not be available
(note it in your report).

Record the current interface, to compare in step 6. With the user's mux
running as they normally start it:

```sh
ros2 node info /twist_mux > /tmp/mux_before.txt      # use the node's real name if it is namespaced
ros2 param dump /twist_mux > /tmp/mux_params_before.yaml
```

If you cannot run it (no robot, no simulation), skip this and say so.

## 2. Find the uses of `twist_mux` in the workspace

```sh
cd $WS
colcon list | awk '$1 == "twist_mux"'                                  # expected: no output
grep -rnI --include=package.xml 'twist_mux' src | grep -v '/ROSlean/'
grep -rnI --include='*.py' --include='*.xml' --include='*.yaml' --include='*.yml' \
  -e "twist_mux" src | grep -v '/ROSlean/'
grep -rnI --include='*.cpp' --include='*.hpp' --include='*.h' 'twist_mux/' src | grep -v '/ROSlean/'
grep -rnI --include='*.yaml' --include='*.yml' -e 'diagnostic' src | grep -i -e mux -e analyzers | grep -v '/ROSlean/'
```

Interpretation:

- First command prints a line: stop (case 2).
- Launch files with `package='twist_mux'` and `executable='twist_mux'`, or
  with `get_package_share_directory('twist_mux')` / `FindPackageShare('twist_mux')`:
  these are the places option B edits. Option A edits none.
- `executable='twist_marker'` or `'joystick_relay.py'`: helper nodes. They
  stay upstream's; never change their package name.
- yaml files whose top-level key is `twist_mux` or `/**`: parameter files.
  They are not edited by either option.
- The C++ grep prints an include of `twist_mux/…`: stop (case 4).
- The diagnostics grep prints something that concerns the mux: stop (case 3).

Bring-up packages installed from binaries (under `/opt/ros`) are not in
`src`. If the mux is started from one of them, option A still applies; option
B requires a copy of that package in `src`, which is the human's decision.

## 3. Get and build

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
cd $WS/src && git clone https://github.com/Beneficial-AI-Foundation/ROSlean.git
cd $WS && source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to twist_mux              # option A
# colcon build --packages-up-to roslean_twist_mux   # option B (its package.xml also names demo dependencies)
```

Expected last lines: `Summary: 2 packages finished` (`roslean_core`,
`twist_mux`); with option B, `roslean_core` and `roslean_twist_mux`. The first
build downloads the Lean toolchain pinned in `$ROSLEAN/lean-toolchain`; it
needs network access. Warnings from `install_name_tool` on macOS are
harmless.

Do not use `--symlink-install` if the install tree is to be copied to another
machine. Do not run `colcon build` without `--packages-up-to` or
`--packages-select` unless the human wants all packages of the repository:
that also builds the demo package and `roslean_bench`, which downloads
upstream's sources.

If the build fails in `roslean_core` with `lake` or `lean` not found, `PATH`
lacks `~/.elan/bin`. If it fails there on a read-only source directory: the
build writes `$ROSLEAN/core/.lake`; the checkout must be writable. Any other failure: stop and report.

## 4. Option A: overlay, no edits

Nothing to edit. Go to step 6.

## 5. Option B: rename the package where the mux is started

For each place found in step 2, and only for the mux itself:

| File kind | Replace | With |
| --- | --- | --- |
| Launch (Python) | `package='twist_mux'` on the action whose `executable` is `'twist_mux'` | `package='roslean_twist_mux'` |
| Launch (Python) | `get_package_share_directory('twist_mux')` or `FindPackageShare('twist_mux')` used for `twist_mux_launch.py`, `twist_mux_topics.yaml`, `twist_mux_locks.yaml` or `joystick.yaml` | the same with `'roslean_twist_mux'` |
| Launch (XML/YAML) | `pkg="twist_mux" exec="twist_mux"` | `pkg="roslean_twist_mux" exec="twist_mux"` |
| `package.xml` of a package you edited | `<exec_depend>twist_mux</exec_depend>` | add `<exec_depend>roslean_twist_mux</exec_depend>`; keep the `twist_mux` line if that package starts a helper node |

Do not change: `executable='twist_mux'`, `name=…`, `namespace=…`,
`parameters=…`, `remappings=…`, any yaml file, any action whose executable is
`twist_marker` or `joystick_relay.py`.

Rebuild the packages you edited (`colcon build --packages-select <names>`).

## 6. Verify

In a new shell:

```sh
source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash
```

Option A:

```sh
ros2 pkg prefix twist_mux            # expected: $WS/install/twist_mux
ros2 pkg executables twist_mux       # expected: three lines: joystick_relay.py, twist_marker, twist_mux
ros2 pkg xml twist_mux -t version    # expected: 0.1.0
ros2 launch twist_mux twist_mux_launch.py --show-args
# expected arguments: config_locks, config_topics, config_joy, cmd_vel_out, use_sim_time
```

Option B:

```sh
ros2 pkg executables roslean_twist_mux      # expected: roslean_twist_mux twist_mux
ros2 launch roslean_twist_mux twist_mux_launch.py --show-args   # the same five arguments
```

Start the mux the way the user starts it (their launch file, or
`ros2 run <package> twist_mux --ros-args --params-file <their yaml>`). Expected
in its output, once per input and per lock:

```
[INFO] [...] [twist_mux]: Topic handler 'topics.<name>' subscribed to topic '<topic>': timeout = 0.500000s, priority = <n>.
[INFO] [...] [twist_mux]: Lock handler 'locks.<name>' subscribed to topic '<topic>': timeout = None, priority = <n>.
```

`Lock handler` is the sign of the Lean-backed node; upstream writes
`Topic handler 'locks.…'`. A line `[FATAL] [...] [twist_mux]: …` followed by
exit code 1: stop (case 7).

Then, in another shell with the same environment:

```sh
ros2 node info /twist_mux > /tmp/mux_after.txt
diff /tmp/mux_before.txt /tmp/mux_after.txt
```

Expected: exactly one line differs, present before and absent after:

```
<     /diagnostics: diagnostic_msgs/msg/DiagnosticArray
```

```sh
ros2 param dump /twist_mux > /tmp/mux_params_after.yaml
diff /tmp/mux_params_before.yaml /tmp/mux_params_after.yaml
```

Expected: only the block `diagnostic_updater:` (`period`, `use_fqn`) is
missing after. If the user's file sets `diagnostic_updater.*`, it is still
listed; that is case 5.

Any other difference: stop and report it.

A behaviour check that needs no robot, for a configuration with at least two
inputs. Replace `LOW`, `HIGH` with the topics of a lower- and a
higher-priority input of the user's configuration, `OUT` with the output
topic after remapping, and use `geometry_msgs/msg/Twist` with
`'{linear: {x: 1.0}}'` if `use_stamped` is false:

```sh
ros2 topic echo OUT --field twist.linear.x &
ros2 topic pub -w 1 --times 5 --rate 10 LOW  geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 1.0}}}'
# expected: 1.0 printed 5 times
ros2 topic pub -w 1 --times 5 --rate 10 HIGH geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 2.0}}}' &
sleep 0.2
ros2 topic pub -w 1 --times 5 --rate 10 LOW  geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 3.0}}}'
# expected: 2.0 printed (5 times; one may be missing from the echo), 3.0 never
```

Do this only where publishing on these topics cannot move a robot: the
output topic must not be connected to a base driver. If you are not sure,
skip it and say so.

## 7. Roll back

Option A:

```sh
rm -rf $WS/install/twist_mux $WS/build/twist_mux
touch $WS/src/ROSlean/overlay/twist_mux/COLCON_IGNORE
# new shell, source as before
ros2 pkg prefix twist_mux      # expected: /opt/ros/jazzy (or wherever upstream is)
```

Option B: revert the edits of step 5 (`git diff` in each edited repository
shows them) and rebuild those packages.

Removing everything: additionally delete `$WS/src/ROSlean`,
`$WS/install/roslean_*`, `$WS/build/roslean_*`. `~/.elan` is left to the
human.

## 8. What to report to the human

- Which option, which files were edited (option B) or that none were.
- The outputs of the checks in step 6, including the two diffs.
- That `/diagnostics` is no longer published by the mux.
- Whether the helper nodes are in use and that they are still upstream's.
- Which checks were skipped and why.
- The platform, and that this procedure was tested only on the platforms
  listed in [DROP_IN_TWIST_MUX.md](DROP_IN_TWIST_MUX.md#what-was-tested).
