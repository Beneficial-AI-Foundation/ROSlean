# Tutorial: the collision monitor in Gazebo

You will run two Lean-backed nodes in series on a simulated TurtleBot3: the
`twist_mux` of the [Gazebo tutorial](TUTORIAL_GAZEBO.md) picks a velocity
command, and the collision monitor passes it on, scales it down or replaces
it by zero, depending on what the lidar sees. A box dropped in the robot's
path makes it slow down and stop; when the box is taken away it drives on.

Do steps 1 and 2 of the [Gazebo tutorial](TUTORIAL_GAZEBO.md) first (install
elan and pixi, clone, `pixi run build`). The build now ends with
`Summary: 4 packages finished`. Time: about 15 minutes.

What has been run where:

- **macOS on Apple silicon:** everything below, without windows (`gui:=false`)
  and driven by scripts: the scenario of step 3, the commands of step 4 (with
  a script publishing what the keyboard would), the check of step 5 with the
  Lean node and with upstream's, and Nav2 as the planner (step 7) as far as
  that section says. **Not looked at: the Gazebo window.** The demo was started once
  with its window and the scenario ran through, but the screen could not be
  captured, so nobody has seen this demo on a screen yet, nor driven it
  with a real keyboard.
- **Linux:** nothing.

## 1. What is in the simulation

```
planner (priority 10)  ─┐
teleop  (priority 100) ─┼─▶ twist_mux ─▶ collision monitor ─▶ base watchdog ─▶ robot
e-stop lock (255)      ─┘                       ▲
                                         lidar ─┘
```

The apartment, the robot, the patrol, the mux and its configuration, and the
base watchdog are those of the Gazebo tutorial; the launch file of this demo
uses them from where `roslean_twist_mux` installs them. New are:

- **The collision monitor** (`roslean_collision_monitor`), between the mux
  and the base. It is a replacement for the core of Nav2's
  `nav2_collision_monitor`; what it supports is listed in
  [collision_monitor/SPEC.md](collision_monitor/SPEC.md).
- **Two zones** in front of the robot, in
  `ros/roslean_collision_monitor/demo/gazebo/monitor.yaml`:

  ```
            y (left, m)
       0.22 ┌────────────────────────────────────────────┐
       0.14 │ ┌────────────────────┐                     │
            │ │  [robot]▶   Stop   │   Slow: command × 0.4
      -0.14 │ └────────────────────┘                     │
      -0.22 └────────────────────────────────────────────┘
          -0.14       0           0.30                  0.70   x (ahead, m)
  ```

  If at least four lidar points lie in the stop zone, every command is
  replaced by zero. Otherwise, if at least four lie in the slowdown zone,
  the command is multiplied by 0.4. Otherwise it passes unchanged. The robot
  itself reaches from x = −0.10 to x = +0.04.
- **A box**, 0.4 m on each side, which a script puts in front of the robot
  and removes again (`obstacle.py`). It uses the simulator's own services, so
  the monitor learns of the box only through the lidar.
- **A second narrator** (`monitor_narrator.py`), which reports what the
  monitor does to the command and names the theorems that explain it. The
  mux narrator of the Gazebo tutorial runs next to it.

### Where the lidar is

The lidar's scans are in the frame `base_scan`; the zones are in the base
frame, `base_footprint`. Upstream's monitor asks TF for the transform between
the two. The Lean node looks up no transform (SPEC, D11): it uses a scan only
if it is in the base frame already, or if the sensor's pose in the base frame
is given as a parameter, which is what `monitor.yaml` does:

```yaml
scan:
  roslean_sensor_pose: [-0.032, 0.0, 0.0]   # x, y in metres, yaw in radians
```

The numbers are the robot's. In `turtlebot3_description`'s
`turtlebot3_burger.urdf`, `base_joint` puts `base_link` at xyz (0, 0, 0.010)
in `base_footprint`, and `scan_joint` puts `base_scan` at xyz
(−0.032, 0, 0.172) in `base_link`, both without rotation. So the lidar is
3.2 cm behind the base's origin, on the centre line, facing forward; its
height does not matter for a planar scan. The simulated sensor in
`turtlebot3_gazebo`'s `model.sdf` is at the same x and y. If the pose were
left out, the robot would not move at all: see step 8.

## 2. Start it

Terminal 1:

```sh
pixi run cm-gazebo-scenario
```

A Gazebo window opens, the room fills in (the first launch downloads the
furniture, see the Gazebo tutorial) and the robot starts its patrol. With
`scenario:=true`, which this task sets, the box appears by itself.

## 3. Experiment 1: the box, without touching anything

Watch terminal 1. First both narrators report the normal state:

```
>>> planner drives: it is the only live input and no lock is active
    theorem forward_of_highest_live

>>> monitor: the command passes unchanged; no zone is occupied
    theorems run_pass_through, output_eq_publish_pass_iff
```

When the robot has turned the first corner and drives straight with two
metres of free floor ahead, the script drops an orange box 1.2 m in front of
it:

```
[obstacle] box dropped 1.2 m ahead of the robot, at (0.37, -1.65)
```

The robot drives on at full speed for half a metre. Then the box is in the
slowdown zone and the robot continues at 40 % of its speed:

```
>>> monitor: SLOWDOWN, the command is published times 0.40: the slowdown zone is
    occupied and the stop zone is not
    theorems output_eq_publish_slowdown_iff, run_slowdown_bound, run_no_amplification
```

About five seconds later the box reaches the stop zone and the robot stops,
a quarter of a metre from the box:

```
>>> monitor: STOP, zero is published instead of the command: scans are arriving, so
    the stop zone is occupied
    theorems evaluate_isStop_iff, run_stop, stop_not_pass_or_slowdown

[obstacle] the robot is at rest, 0.26 m from the box; the box stays for 6 s
```

The patrol does not know any of this and keeps commanding 0.18 m/s. One
second later the monitor's output changes once more:

```
>>> monitor: publishes NOTHING although commands arrive: it has published zero for
    stop_pub_timeout and a stop is still demanded. The last message the base got
    is a zero; holding the robot is now the base watchdog's job (SPEC R15, A3)
    theorems output_eq_silent_iff, run_silent_last_published_zero
```

This is upstream's behaviour and the Lean node reproduces it: zero is
published for `stop_pub_timeout` (here 1 s) and then nothing, until a
velocity other than zero is due again. Step 6 says what that means for the
robot.

After six seconds the script removes the box, and the robot drives on:

```
[obstacle] box removed

>>> monitor: the command passes unchanged; no zone is occupied
    theorems run_pass_through, output_eq_publish_pass_iff

[obstacle] the robot drives on
```

Near the edge of a zone the narrator can report STOP, SLOWDOWN and STOP in
quick succession. The simulated lidar has noise (a standard deviation of
1 cm on each range), so while the face of the box is within a centimetre or
two of the zone's edge, one scan has four points inside and the next has
not. The robot then creeps forward for a fifth of a second and is stopped
again, further inside.

The script is done after one box. For more, use the commands of the next
step. Stop everything with Ctrl-C before going on.

## 4. Experiment 2: by hand, with keyboard and e-stop

Terminal 1:

```sh
pixi run cm-gazebo
```

Terminal 2, whenever the robot drives straight and has room ahead:

```sh
pixi run cm-box-drop      # a box 1.2 m in front of the robot, wherever it is
pixi run cm-box-remove    # take it away
```

The narrator's output is that of step 3. A box dropped while the robot
turns a corner is simply left behind; remove it and try again.

**The monitor gates whoever the mux picked.** Start the keyboard in
terminal 3 (`pixi run gazebo-teleop`), drop a box, and hold `i` to drive at
it. The mux now forwards your commands and drops the patrol's; the monitor
slows and stops the robot in front of the box all the same, and no key gets
it closer. (Checked with a script publishing 0.2 m/s on the keyboard's
topic: 0.20 m/s before the box, 0.08 m/s in the slowdown zone, at rest
0.27 m from the box with the commands still arriving.)

No key gets it away either: in the stop zone *every* command becomes zero,
including backing up and turning. The box has to go. That is how upstream's
`stop` action works as well; its `approach` and `limit` actions, which this
node does not have, are the ones that let a robot manoeuvre near an
obstacle.

**With the e-stop on, the monitor is idle.** `pixi run demo-estop` makes
the mux forward nothing, so the monitor receives no command and publishes
nothing:

```
>>> e-stop lock ENGAGED

>>> nothing is forwarded: the e-stop lock (255) is above every input
    theorem not_forward_of_top_lock

>>> monitor: no command arrives, so nothing is published, whatever the lidar sees
    (the monitor acts only on commands: SPEC R1)
    theorem run_scan_output
```

`pixi run demo-release` brings both back.

## 5. The automated check

```sh
pixi run cm-check
```

runs step 3 without windows and checks it against the simulator's ground
truth. It takes about half a minute and ends with `PASSED` or `FAILED`:

```
1. Free driving (the two seconds before the box)
  ok    the monitor passed all 20 commands unchanged
  ok    true speed 0.169 m/s (planner commands 0.18 m/s)

2. Slowdown zone
  ok    54 commands were published times 0.4
  ok    10 non-zero commands were answered with zero
  ok    the slowdown came before the stop
  ok    true speed in the slowdown zone 0.072 m/s (42% of the free speed; 5.4 s)
        slowdown began with the box 0.647 m from the robot

3. Stop zone
  ok    from the first zero until the box was removed, no command passed unchanged
        the decision went back from stop to slowdown 0 time(s) (lidar noise at the edge of the zone)
  ok    the robot was at rest when the box was removed, 6.2 s after it had come to rest, and moved 0 mm in between
  ok    the robot never touched the box: smallest gap 0.255 m
        first zero published with the box 0.256 m from the robot
        stopping distance 0.001 m, from the first zero to rest
        at rest 0.255 m from the box (the front edge of the stop zone is 0.262 m ahead of the robot)
        the monitor published 10 zeros over 0.90 s and was then silent for 6.2 s (stop_pub_timeout)

4. After the box was removed
  ok    45 commands passed unchanged
        first command passed 0.15 s after the removal
  ok    the robot drove on: 0.78 m in 4.6 s
  ok    and reached 0.178 m/s again

5. Whole run
  ok    none of the 205 published velocities exceeded its command or had the other sign
  ok    every published velocity was the command, zero, or the command times 0.4

PASSED in 32 s (27 s simulated)
```

How to read the numbers:

- *True speed* is the robot's displacement in the simulator divided by
  simulated time, not the commanded velocity and not the wheel odometry.
- *Stopping distance* is how far the robot still moved after the monitor's
  first zero. It includes any creeping at the zone's edge (step 3).
- *Gap* is the distance between the outline of the robot's collision shapes
  and the box. The front edge of the stop zone is 0.262 m ahead of the
  robot's front, so a gap at rest just below that means the robot stopped
  as soon as the box was in the zone.

In nine runs on an Apple M-series laptop, all passed:

| | Range over nine runs |
| --- | --- |
| True speed before the box | 0.168 to 0.178 m/s |
| True speed in the slowdown zone | 0.071 to 0.072 m/s |
| Gap when the slowdown began | 0.626 to 0.653 m |
| Gap when the first zero was published | 0.256 to 0.273 m |
| Stopping distance | 0 to 14 mm |
| Gap at rest | 0.255 to 0.268 m |
| Runs in which the stop flickered once | 1 |

An earlier version of the check demanded that nothing but zero be published
after the first zero. It failed in two runs of five, because of the flicker
at the zone's edge described in step 3; the check now demands that no
command passes unchanged while the box is there, and that the robot moves
less than 3 cm after it has first come to rest.

The check is a script and not one of the `colcon` tests because Gazebo
renders the lidar and so needs a machine with a display, and the world
needs the furniture from Gazebo's online library.

## 6. Upstream's node, for comparison

```sh
pixi run bash -c 'source install/setup.bash && ros2 launch \
  $(ros2 pkg prefix roslean_collision_monitor)/share/roslean_collision_monitor/demo/gazebo/cm_gazebo_launch.py \
  scenario:=true monitor_package:=nav2_collision_monitor'
```

or, for the check, `pixi run cm-check --monitor nav2_collision_monitor`.
The same `monitor.yaml` is used. Two things are different in the launch
file, because upstream's node cannot simply be started:

- it is a lifecycle node and does nothing until it is configured and
  activated, so a `nav2_lifecycle_manager` is started for it;
- it looks up the lidar's pose in TF, which `robot_state_publisher`
  provides from the robot's URDF. (It ignores `roslean_sensor_pose`.)

The behaviour is the same and the check passed with it in four runs of
four: at rest 0.253 to 0.257 m from the box, stopping distance 0 to 12 mm,
one run with a flicker. That is about half a centimetre closer than the Lean
node, which is the direction SPEC D4 predicts (the Lean node's zones are up
to 4.4 mm larger), but with this few runs and a lidar noise of 1 cm it is
not a measurement of that difference.
Upstream also writes a line to the log whenever the action changes (`Robot to
stop due to Stop polygon`); the Lean node logs these only at debug level.

## 7. Nav2 as the planner

```sh
pixi run cm-gazebo-nav2
```

Nav2 replaces the patrol, as in step 5 of the Gazebo tutorial, and the tour
script sends it from place to place. Nav2 has a collision monitor of its
own inside (upstream's, with the `approach` action, named
`collision_monitor`); the Lean monitor of this demo is a second, independent
gate behind the mux, under the name `cm_gate`.

There is less to see than with the patrol, because Nav2 avoids obstacles by
itself. Tried once, without windows: a box dropped 0.8 m in front of the
moving robot made the Lean monitor report SLOWDOWN, Nav2's controller
logged `Failed to make progress` twice, then Nav2 drove around the box and
reached its goal; the robot never came closer to the box than 0.27 m and the
stop zone was not entered. The narrator also reports short slowdowns
whenever Nav2 passes close to furniture or turns near a wall. To see a stop
with Nav2, use the keyboard as in step 4.

Nav2 came up in one of two starts of this variant. In the other, on a
machine busy with other work, AMCL never became active and the robot did not
move; a restart is the remedy.

## 8. What the demo shows, and what it does not

**It shows the rule at work in a full stack.** A lidar, a planner that knows
nothing of obstacles, the mux and the monitor run together under simulated
time, and the three outcomes of the monitor's rule appear in the order one
expects, for the planner's commands and for the keyboard's. Every published
velocity in the check was the command, the command times 0.4, or zero; that
is what `run_no_amplification` and the four `output_eq_publish_…_iff`
theorems say must be so.

**A proved rule about commands is not a proof about physics.** The theorems
are about the Lean function the node calls for each message. `run_stop`
says: *if* the scan received last is not too old and has at least four valid
points in the stop zone, the next command is answered with zero, or with
nothing after `stop_pub_timeout`. Whether the robot then stands still in
front of the obstacle depends on things no theorem here mentions:

- **Whether the lidar sees the obstacle.** It scans one plane, 18 cm above
  the floor, between 0.12 m and 3.5 m. A lower object, a table top, or
  something closer than 12 cm to the sensor is not in the scan, and an
  empty scan is free space to the monitor (SPEC, R4).
- **Whether it sees it in time.** The simulated lidar delivers five scans a
  second, and the monitor accepts a scan up to `source_timeout` (1 s) old.
  At 0.18 m/s the robot moves 3.6 cm between two scans.
- **Whether the zones are large enough.** Their sizes were chosen by hand
  for this robot at this speed. Nothing relates them to a braking distance.
- **Whether the base obeys.** The stopping distance of a few millimetres
  measured in step 5 is a property of Gazebo's differential drive, which
  sets the wheel speeds to the commanded value at once. A real TurtleBot3
  has inertia, motor controllers and a communication delay.
- **Whether the sensor pose in `monitor.yaml` is right.** It was copied from
  the URDF by hand. A wrong pose shifts both zones, and nothing notices.
- **Sensor noise.** At the edge of a zone the decision flickers (step 3).
  Each single decision is the one the theorems prescribe for the scan at
  hand.

That is lesson 16 of [PLAN.md](PLAN.md) again: in the mux demo Gazebo showed
that "nothing is forwarded" does not stop a robot; here it shows that "zero
is published when four points are in the zone" is a statement about
messages.

**The monitor is not a watchdog.** It acts only when a command arrives
(SPEC, R1) and it stops publishing zero after `stop_pub_timeout` (R15).
In this demo the robot stays put during that silence for two reasons that
are outside the monitor: Gazebo's drive keeps executing the last command it
got, which was a zero (`run_silent_last_published_zero` guarantees that
much), and the base watchdog sends zeros of its own once the monitor has
been silent for 0.3 s. A base that needs fresh commands to hold still, or
that also takes commands from elsewhere, is not protected during the
silence (SPEC, A3).

**What the check does not cover.** One approach, head-on, at one speed, on
a flat floor, against one box, in simulation. It does not compare the Lean
node with upstream message by message (both pass the same check; the
scenario tests in `ros/roslean_collision_monitor/test/` are for that), and
it does not exercise a stale scan, a disabled source or malformed input.

**What is trusted in the running node.** The C++ shim computes the points
of a scan in metres, including the shift by the sensor pose, and multiplies
the command by the ratio in a slowdown; both are short and neither is
covered by a theorem (SPEC, T1 and T2). The narrator sees topics, not the
node's reasoning: it reports a stop when a non-zero command is answered with
zero, and concludes from the arriving scans that a zone, not a missing scan,
is the cause.

**Ground truth.** The patrol steers by the robot's true position, the script
places the box by it, and the check measures by it. A real robot has none of
this; the monitor itself uses only the lidar and the commands.

## 9. Where the Lean node is not a drop-in replacement

Found while putting this demo together. Each is a consequence of a
documented difference, not a defect of the demo.

- **It must be told where the lidar is** (D11). The TurtleBot3's scan is in
  `base_scan`, and the node reads no TF, so the pose was copied from the
  URDF into `monitor.yaml` (step 1). Tried without it: the node logs `Scan
  is in frame 'base_scan', not in the base frame 'base_footprint', and
  'scan.roslean_sensor_pose' is not set` every five seconds and treats the
  lidar as having no data, which stops every command.
- **It refuses the collision monitor parameters Nav2 ships.** Given the
  `collision_monitor` section of `nav2_bringup`'s `nav2_params.yaml` (the
  one the TurtleBot3 set-up uses, and `nav2_params.yaml` of the mux demo),
  it exits with, in turn: `parameter 'state_topic': the state topic is not
  provided` (D13); then `parameter 'base_shift_correction': must be set to
  false explicitly` (D11; Nav2 sets `True`); then `parameter
  'FootprintApproach.action_type': only 'stop' and 'slowdown' are
  supported; got 'approach'`. It names one parameter per start, so it took
  three starts to see the three. The first two can be edited away. The
  third cannot: Nav2's default protection *is* the `approach` action. So
  the Lean node cannot take the place of the monitor inside Nav2's own
  pipeline with Nav2's configuration, and in this demo it is a gate of its
  own behind the mux, with zones written for it. The example parameter
  file of `nav2_collision_monitor` itself is refused too, at `state_topic`
  (only that first refusal was tried; the file also asks for `limit`, a
  velocity polygon and a point cloud source).
- **It is not a lifecycle node** (D2). Nav2's bring-up lists
  `collision_monitor` among the nodes its lifecycle manager configures and
  activates. A lifecycle manager pointed at the Lean node prints `Waiting
  for service cm_gate/get_state...` every two seconds and gets no further
  (tried). The Lean node needs no bring-up: it acts from its first command.
  That is why the launch file starts a lifecycle manager for upstream's node
  only.
- **It shows nothing of its state.** No state topic, no zone polygons for
  RViz (`visualize` is ignored with a warning), and its decisions are logged
  at debug level only, where upstream logs each change of action. The
  narrator of this demo therefore has to infer the action from the
  velocities, and the zones cannot be displayed.
- **There is no way out of a stop zone** without `approach` or `limit`
  (step 4). That holds for upstream with this configuration too; upstream
  has the other actions to offer.

What needed no change: the parameter file is the same for both nodes
(upstream ignores the one extra parameter); the node can be renamed from the
launch file (`cm_gate` here, because Nav2 starts a `collision_monitor` of
its own); simulated time, stamped messages and the topic parameters work as
upstream's do.

## If something goes wrong

See also the table at the end of the [Gazebo tutorial](TUTORIAL_GAZEBO.md).

| Symptom | Likely cause |
| --- | --- |
| The robot never moves, and the monitor narrator says `STOP … no scan has arrived` | The lidar is not publishing. On Linux this happens when Gazebo has no display to render it with. |
| The robot never moves, and the log repeats `Scan is in frame 'base_scan', not in the base frame …` | `roslean_sensor_pose` is missing from `monitor.yaml`. |
| The monitor exits at once with `[FATAL] … parameter '…'` | The configuration asks for something the Lean node does not support; the message names the parameter. |
| `STOP … no scan has arrived` for a moment at the very start | The planner's first commands arrive before the lidar's first scan; without a scan the monitor stops the robot (SPEC, R8). |
| `[obstacle] the robot did not come to rest in front of the box; box removed, trying again` | The robot turned away before it reached the box. The script tries three times. |
| `cm-box-drop` prints `no ground truth from the simulator` | The demo is not running, or runs with another `ROS_DOMAIN_ID`. |
| `cm-box-drop` prints `the simulator did not accept the box`, or `cm-box-remove` prints `no box to remove` | The simulator is not running, or runs with another `GZ_PARTITION`; or there was no box. |
| The robot stands in front of a box and no key moves it | Intended: see step 4. `pixi run cm-box-remove`. |
| Narrator output of two demos is mixed up, robots stop at random | Two demos are running at once (for example a `cm-check` next to a `cm-gazebo` with the same `ROS_DOMAIN_ID`). Stop one. |
