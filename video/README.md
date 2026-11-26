# Demo videos

Two one-minute videos of the same run of the Gazebo demo
([docs/TUTORIAL_GAZEBO.md](../docs/TUTORIAL_GAZEBO.md), experiment 1):

- [`twist_mux_gazebo.mp4`](../site/media/twist_mux_gazebo.mp4): the run as filmed by a
  camera in the Gazebo world, with captions and a panel that shows which
  input the mux is forwarding.
- [`twist_mux_generated.mp4`](../site/media/twist_mux_generated.mp4): a 3D replay of the recorded path and decisions, with the same captions and panel.

The project page plays the 3D replay first and lets viewers switch to Gazebo footage.

## What is real in them

The run is the demo as it is: the Lean-backed `roslean_twist_mux` with
`ros/roslean_demos/gazebo/mux/gazebo.yaml`, the patrol planner and the base watchdog, on a
TurtleBot3 in Gazebo. A script plays the operator (it publishes on `key_vel`
what a held teleop key sends) and switches the e-stop lock.

Everything the videos show about the mux comes from the messages of that run.
The recorder notes every message on the two inputs, the lock and the mux
output; an output message is attributed to the input that sent the same
stamp and velocity. The panel, the colours and the label over the robot are
drawn from that record, and `build_data.py` refuses a take in which any
output message could not be attributed. Neither page computes what the mux
"should" do.

In the second video the robot's shape and size, the floor and the camera are
drawn; its path is the recorded one.

Differences from the demo, for filming only:

- The furniture is built from boxes instead of fetched from Gazebo Fuel, and
  the floor is textured (`make_world.py`).
- The world has a fixed camera, and the simulation runs slower than real
  time so that software rendering keeps up. All nodes use simulated time.
- Nobody touches a keyboard; `scenario.py` publishes the operator's commands.

## Files

| File | Purpose |
| --- | --- |
| `make_world.py` | Writes the filming world from the demo's `apartment.sdf`. |
| `record_launch.py`, `record.sh` | Run the demo without a window and record a take. |
| `scenario.py` | The operator and the e-stop, on a fixed timetable. |
| `recorder.py` | Saves camera frames and the message trace. |
| `build_data.py` | Take → `web/data.js`. |
| `web/story.js`, `web/common.css` | Storyboard, captions and the panel, shared by both videos. |
| `web/gazebo.html` | Video 1: footage, pan and zoom, trail, overlays. |
| `web/generated.html` | Video 2: the 3D scene (three.js). |
| `render.mjs`, `make_video.sh` | Draw each frame in headless Chromium and encode with ffmpeg. |

## Making them again

After `pixi run build`:

```sh
pixi run bash video/record.sh /tmp/take          # about 30 minutes without a GPU
python3 video/build_data.py /tmp/take
ln -sfn /tmp/take/frames video/web/frames
(cd video && npm install)                        # three.js; playwright must be installed too
video/make_video.sh video/web/gazebo.html site/media/twist_mux_gazebo.mp4
video/make_video.sh video/web/generated.html site/media/twist_mux_generated.mp4
```

`record.sh` uses the X display if there is one. Without one it starts Xvfb
and renders with Mesa's software renderer, which is how the videos here were
made (Linux, no GPU). To change the wording or the timing, edit `BEATS` in
`web/story.js`; to change what the operator does, edit `scenario.py` and
record again.

## The collision monitor video

[`collision_monitor_generated.mp4`](../site/media/collision_monitor_generated.mp4) and
[`collision_monitor_gazebo.mp4`](../site/media/collision_monitor_gazebo.mp4) show one
recorded run of the box scenario of
[docs/TUTORIAL_COLLISION_MONITOR.md](../docs/TUTORIAL_COLLISION_MONITOR.md) (steps 2 and 3),
the first as a 3D replay, the second as filmed in Gazebo. Their sources are in
[`cm/`](cm/); they reuse `make_world.py`, `render.mjs` and `web/common.css` from here.

### What is real in them

The run is the demo as it is: the Lean-backed `roslean_twist_mux` and, behind
it, the Lean-backed `roslean_collision_monitor` with the demo's
`monitor.yaml`, the patrol and the base watchdog, on a TurtleBot3 in Gazebo.
The box is dropped and removed by the demo's own `obstacle.py`.

Everything the videos show about the monitor comes from the messages of that
run. `cm/build_data.py` pairs every message on the monitor's output with the
input message of the same header stamp and compares the two velocities, as
the demo's narrator does: equal is PASS, the command times one ratio is
SLOWDOWN, zero for a non-zero command is STOP, and commands arriving with
nothing published is SILENT. The zone that lights up is the one the
monitor's own state topic names. The numbers in the panel are the recorded
velocities; "nearest lidar point ahead" is read off the recorded scan (the
smallest distance ahead of the base's origin among the points within the
zones' width). The box appears and disappears at the times, and at the
pose, that `obstacle.py` reported. The lidar's points are those of the
recorded scans, placed with the recorded pose.

`build_data.py` refuses a take whose record is inconsistent: an output
without an input, an output that is neither the command, nor zero, nor the
command times the configured ratio, an output that contradicts what the state
topic said at that moment, a silence outside a reported stop or not preceded
by a published zero, or a scenario that did not run once from drop to
drive-on. Neither page computes what the monitor "should" do: the zones are
drawn with the shapes of `monitor.yaml` at the recorded pose, but no page
tests whether a point lies inside one. If the decision alternates at the edge
of a zone, the video shows it as recorded.

In the replay the robot's shape, the floor and the camera are drawn; the
robot is the one of the twist_mux replay, at the true size of the burger.
The captions name the theorems the demo's narrator prints. They are about
the commands the node publishes, not about where the robot ends up.

Differences from the demo, for filming only:

- The world is the filming world of the twist_mux video (furniture from
  boxes, textured floor, fixed camera, slowed-down simulation).
- The monitor gets one more parameter, `state_topic`, so that it publishes
  its state (upstream's `CollisionMonitorState`).
- The scenario starts 12 s into the run instead of at once
  (`cm/scenario.py`), so the box is dropped on the second leg of the patrol,
  not the first, and the video has time to show the robot driving freely.
  Its waits are bounded in simulated time only, since the simulation is slow.

### Files

| File | Purpose |
| --- | --- |
| `cm/make_world.py` | The filming world, by `make_world.py`; without the camera if asked. |
| `cm/record_launch.py`, `cm/record.sh` | Run the demo without a window and record a take. |
| `cm/scenario.py` | Runs `obstacle.py`'s scenario and reports its events. |
| `cm/recorder.py` | Saves the message trace and the camera frames. |
| `cm/build_data.py` | Take → `cm/web/data.js`, with the consistency checks. |
| `cm/web/story.js`, `cm/web/cm.css` | Storyboard, captions and the panel. |
| `cm/web/generated.html`, `cm/web/gazebo.html` | The two pages. |
| `cm/make_video.sh` | Draws the frames (`render.mjs`), encodes, writes the poster. |

### Making them again

After `pixi run build`:

```sh
pixi run bash video/cm/record.sh /tmp/cm-take 3840 2160 0.03   # about 35 minutes on an Apple M-series
pixi run python video/cm/build_data.py /tmp/cm-take
ln -sfn /tmp/cm-take/frames video/cm/web/frames
(cd video && npm install && npm install --no-save playwright \
   @fontsource-variable/inter @fontsource-variable/jetbrains-mono)
video/cm/make_video.sh video/cm/web/generated.html site/media/collision_monitor_generated.mp4
video/cm/make_video.sh video/cm/web/gazebo.html site/media/collision_monitor_gazebo.mp4
```

The takes here were recorded on macOS (arm64) with the screen unlocked; the
simulator renders the lidar and the camera without a window. The camera
delivers about one 4K frame a second there, hence the real-time factor of
0.03. `record.sh DIR 0` records the messages only, in real time, which is
all the replay needs. The story's times come from the take (`marks` in
`data.js`); to change the wording, edit `BEATS` in `cm/web/story.js`. The
pages use the fonts Inter and JetBrains Mono, installed ones if there are,
else those in `node_modules`.
