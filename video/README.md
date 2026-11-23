# Demo videos

Two one-minute videos of the same run of the Gazebo demo
([docs/TUTORIAL_GAZEBO.md](../docs/TUTORIAL_GAZEBO.md), experiment 1):

- [`twist_mux_gazebo.mp4`](../site/media/twist_mux_gazebo.mp4): the run as filmed by a
  camera in the Gazebo world, with captions and a panel that shows which
  input the mux is forwarding.
- [`twist_mux_generated.mp4`](../site/media/twist_mux_generated.mp4): the same run drawn in
  3D from the recording (not Gazebo), with a better-looking robot and the
  same captions and panel.

The files are in [`site/media/`](../site/media), from where the project page
shows them: the generated one first, with a button for the Gazebo one.

## What is real in them

The run is the demo as it is: the Lean-backed `roslean_twist_mux` with
`demo/gazebo/gazebo.yaml`, the patrol planner and the base watchdog, on a
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

The end card of `twist_mux_gazebo.mp4` was later changed in the finished
video (the old text removed from its last seven seconds and the new card
drawn in), because the take's frames were no longer at hand. Rendering it
again from a take gives the same card without that step.

To add a soundtrack, give `make_video.sh` a sound file and the second of it
at which the video should begin; it fades in and out at the ends. No sound
file is kept in this repository.

```sh
AUDIO=track.m4a AUDIO_START=42 video/make_video.sh video/web/generated.html out.mp4
```

`record.sh` uses the X display if there is one. Without one it starts Xvfb
and renders with Mesa's software renderer, which is how the videos here were
made (Linux, no GPU). To change the wording or the timing, edit `BEATS` in
`web/story.js`; to change what the operator does, edit `scenario.py` and
record again.
