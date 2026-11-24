# Collision monitor: specification of the Nav2 extension

Written by task T04. It specifies three additions to the node of
[SPEC.md](SPEC.md), so that Nav2's lifecycle manager can run it in place of
`nav2_collision_monitor`, still with the `stop` and `slowdown` actions only:

1. the lifecycle interface, the state topic and the polygon publishers;
2. transforms from TF, with base shift correction;
3. zones whose shape arrives on a topic (the route by which the robot's
   footprint becomes a zone).

Sections 1 and 2 say what upstream does; sections 3 to 9 say what the Lean
model, the C interface and the node are to do; section 10 is the test plan,
section 11 the plan for tasks T05 to T07, section 12 what will still be
missing afterwards.

**What is implemented.** Task T05 has implemented sections 3 and 4: the
model (`core/ROSleanCore/CollisionMonitor.lean`) and the theorems
([THEOREMS.md](THEOREMS.md)). Where the model or a theorem differs from what
T04 wrote in those sections, the text below is corrected and marked
"(T05)". The C interface and the node (section 8) are not implemented yet.

**The transform is applied in C++ (option A).** T04 recommended applying the
sensor-to-base transform in Lean in exact arithmetic (option B of section
5), and T05 built and proved that. On 2026-11-24 the user decided for option
A: the shim applies the transform in `double`, as upstream does, and Lean
receives points. Task T05b reworked the model, the proofs and this document
accordingly; its changes are marked "(T05b)". Sections 3, 4, 5, 6 (ND1,
ND7, ND9), 7 and 8.2 to 8.4 are the ones that changed. The lowest layer of
the model is now `core/ROSleanCore/CollisionMonitorPoint.lean` (it was
`CollisionMonitorXform.lean`).

Rules are numbered N1, N2, …, differences from upstream ND1, …, in addition
to the R…, D…, A… and T… of SPEC.md.

## Sources, and how each statement is marked

| | Version | Used for |
| --- | --- | --- |
| Source of `navigation2` | branch `jazzy`, commit `c704655`, package version 1.3.14 (the commit SPEC.md names) | **[S]** |
| Source of `geometry2` (`tf2`, `tf2_ros`, `tf2_geometry_msgs`) | branch `jazzy`, commit `f702874`, version 0.36.23 | **[S]** |
| Binary `nav2_collision_monitor` | 1.3.12, RoboStack, macOS arm64, from this repository's pixi environment, with `tf2` 0.36.22, `nav2_lifecycle_manager` 1.3.12, `rmw_fastrtps_cpp` | **[B: case]** |

- **[S]** means read from the source, with file and function. Paths are
  relative to `nav2_collision_monitor/` unless they start with another
  package's name.
- **[B: case]** means observed on the binary on 2026-11-24 by the named case
  of `review/collision_monitor/nav2/nav2_probe.py`; the output of that run is
  `review/collision_monitor/nav2/results/macos-arm64.txt`. Nothing is marked
  [B] that was not run for this document. Where SPEC.md or
  UPSTREAM_BEHAVIOUR.md already reports an observation, it is cited as such
  and not marked [B].
- Between the binary's version 1.3.12 and the source's 1.3.14, `git diff
  1.3.12 c704655` shows, in the files cited here: one added `using`
  declaration in `circle.hpp`; the handle of the pre-shutdown callback in
  `nav2_lifecycle_manager/src/lifecycle_manager.cpp`; and changes to the
  publication schedule and an empty-footprint check in
  `nav2_costmap_2d/src/costmap_2d_ros.cpp`. `src/`, `params/` and `launch/`
  of the package, `nav2_util/src/robot_utils.cpp`,
  `nav2_util/src/lifecycle_node.cpp`,
  `nav2_costmap_2d/src/footprint_subscriber.cpp` and `nav2_bringup/launch`,
  `nav2_bringup/params` are identical.
- The binary was run on macOS arm64 only. Linux was not run for this
  document.

Probes: `review/collision_monitor/nav2/run_nav2_probe.sh <env prefix>
[case …]` (20 cases, about five minutes; simulated time except the cases
`bond` and `manager`), `run_transform_cost.sh` and `fma_check.sh`.

## 1. Upstream's rules

### 1.1 Lifecycle

- **N1. The process.** `main` creates the node and calls `rclcpp::spin`: one
  thread runs all callbacks of the node, one at a time
  (`src/collision_monitor_main.cpp`) [S]. The TF listener is created with
  its own thread (`tf2_ros::TransformListener(*tf_buffer_, this, true)`,
  `CollisionMonitor::on_configure`) [S], and so, by rclcpp's default, is
  the `/clock` subscription under simulated time; that both keep running
  while a callback waits is observed [B: `tf-missing`].
- **N2. Unconfigured.** The node is a `nav2_util::LifecycleNode` named
  `collision_monitor`. After start it has the lifecycle services
  (`~/change_state`, `~/get_state`, `~/get_available_states`,
  `~/get_available_transitions`, `~/get_transition_graph`), the publisher
  `~/transition_event`, the parameter services and, with `use_sim_time`, a
  subscription to `/clock`. It has no subscription to commands, scans or TF
  and no publisher for velocities [B: `lifecycle`]. A command or scan sent
  now reaches nobody.
- **N3. Configure** (`CollisionMonitor::on_configure`) [S]. Creates the TF
  buffer and listener (subscriptions `/tf`, `/tf_static`); reads every
  parameter; creates the sources (scan subscription, `SensorDataQoS`) and
  the zones (N16, N31); the command subscription and the velocity publisher
  (both keep-last 1); the state publisher if `state_topic` is not empty
  (N12); the publisher `~/collision_points_marker`; the service `~/toggle`.
  The lists of publishers, subscriptions and services after the transition
  are as this says [B: `lifecycle`]. If a parameter is missing or wrong the
  callback calls `on_cleanup` and returns failure: the node stays
  unconfigured and the process stays up [S] [B: `footprint-edit`].
- **N4. Inactive.** A command is dropped: `process` returns at
  `!process_active_`, having changed nothing [S]; nothing is published
  [B: `lifecycle`]. A scan is stored as always (`Scan::dataCallback` does
  not know the lifecycle state) [S], and a scan received while inactive is
  the one used after activation [B: `lifecycle`, "the scan was received
  while inactive"]. The same holds for a zone shape received on a topic
  [B: `polygon-topic-inactive`].
- **N5. Activate** (`on_activate`) [S]. Activates the velocity, state and
  marker publishers and the polygon publishers; publishes every visualised
  zone once (N16); sets `process_active_`; creates the bond (N9). The
  transition publishes no velocity and no state message [B: `lifecycle`].
- **N6. Deactivate** (`on_deactivate`) [S]. Clears `process_active_`; sets
  `robot_action_prev_` back to its initial value (no action, velocity
  (−1, −1, −1), name `""`); deactivates the publishers; destroys the bond.
  The stored scan and shapes stay. The stop stamp of R15 is not reset, but
  because the previous velocity is non-zero again, the next zero stores a
  new one. Observed consequences [B: `lifecycle`]: nothing is published by
  the transition; after the next activation, with the same scan and the
  same clock, the zero velocity is published again although
  `stop_pub_timeout` had run out before, it is published for another
  `stop_pub_timeout` (still at +1 s − 1 ns, withheld at +1 s + 1 ns), and
  the state `(StopCircle, 1)` is reported again.
- **N7. Cleanup** (`on_cleanup`) [S]. Destroys subscriptions, publishers,
  zones, sources and the TF buffer. The stored scan is gone: after cleanup,
  configure and activate the first command is answered with a stop for
  `invalid source` [B: `lifecycle`]; a zone's received shape is gone too
  [B: `polygon-topic-inactive`]. The service `~/toggle` is not destroyed: it
  is still listed after cleanup [B: `lifecycle`]. The toggle flag and the
  stop stamp survive [S].
- **N8. Shutdown** (`on_shutdown`) [S] logs and does nothing else. Shut down
  from the active state, the node reports state `finalized` but keeps its
  subscriptions and publishers and **keeps answering commands**: a command
  sent afterwards was forwarded [B: `lifecycle`, last step]. Nav2's
  lifecycle manager does not take this path: it deactivates and cleans up
  before it shuts a node down
  (`nav2_lifecycle_manager/src/lifecycle_manager.cpp`,
  `shutdownAllNodes`) [S], and on `SIGINT` the node deactivates and cleans
  itself up first (`nav2_util/src/lifecycle_node.cpp`, `on_rcl_preshutdown`,
  `runCleanups`) [S].
- **N9. Bond** (`nav2_util/src/lifecycle_node.cpp`, `createBond`,
  `destroyBond`) [S]. The parameter `bond_heartbeat_period` (default 0.1 s)
  is read in the constructor. If it is positive, activation creates
  `bond::Bond("bond", <node name>, node)` with that heartbeat period and a
  heartbeat timeout of 4.0 s, and deactivation destroys it. Observed
  [B: `bond`]: no message on `/bond` while inactive; after activation 23
  messages of type `bond/msg/Status` in 2 s, with `id` equal to the node
  name, `active: true`, `heartbeat_timeout: 4.0`, `heartbeat_period: 0.1`,
  publisher reliable and volatile; one message with `active: false` on
  deactivation. With `bond_heartbeat_period: 0.0` there is no bond (how the
  harness runs the node).
- **N10. The lifecycle manager**
  (`nav2_lifecycle_manager/src/lifecycle_manager.cpp`) [S]. With
  `autostart` it configures every node of `node_names` in order, then
  activates every node in order, each through `<node>/change_state`
  followed by a check through `<node>/get_state`
  (`changeStateForNode`). After activating a node it creates its own end of
  the bond, with heartbeat timeout `bond_timeout` (default 4.0 s), and waits
  up to half of it for the bond to form (`createBondConnection`); if it does
  not form the bring-up fails. Afterwards a 200 ms timer checks every bond
  (`checkBondConnections`); a broken one resets all managed nodes.
  Observed [B: `manager`]: a manager with `node_names: [collision_monitor]`
  and `autostart: true` logs `Configuring collision_monitor`, `Activating
  collision_monitor`, `Server collision_monitor connected with bond.`,
  `Managed nodes are active`, and the node is active. After the node is
  killed with `SIGKILL` it logs `CRITICAL FAILURE: SERVER collision_monitor
  IS DOWN after not receiving a heartbeat for 4000 ms. Shutting down related
  nodes.` and resets.

  So a replacement must offer the lifecycle services under the node name
  the manager is given, succeed in `configure` and `activate`, and form the
  bond on activation, or the manager must be run with `bond_timeout: 0.0`.
- **N11. Other parameters of the base class** [S]: `autostart_node`
  (default false: configure and activate by itself),
  `bond_disable_heartbeat_timeout` (set to true).

### 1.2 The state topic

- **N12. Publisher.** `state_topic` (default `""`). If not empty, configure
  creates a publisher of `nav2_msgs/msg/CollisionMonitorState`, keep-last 1
  (`on_configure`) [S]; reliable and volatile [B: `state`]. The message is
  `uint8 action_type` (0 `DO_NOTHING`, 1 `STOP`, 2 `SLOWDOWN`, 3 `APPROACH`,
  4 `LIMIT`) and `string polygon_name` (`nav2_msgs/msg/CollisionMonitorState.msg`) [S].
- **N13. When.** In `process`, after the action for the command has been
  determined and before the velocity is published: if the action's
  `polygon_name` differs from that of the previous processed command
  (`robot_action_prev_`, initially `""`, reset by deactivation), one message
  with the new name and action type is published and a line is logged
  (`process`, `notifyActionState`) [S]. The comparison is of names only.
  Values: `(<zone>, 1)` for a stop zone, `(<zone>, 2)` for a slowdown zone,
  `("invalid source", 1)` for an invalid source, `("", 0)` otherwise.
  Observed [B: `state`]:
  - no message for the first command if it passes;
  - a message for each change, none while the name stays;
  - **also while the velocity is withheld** (R15): during the silence,
    `("", 0)` and `("invalid source", 1)` were published without a velocity;
  - none for a non-finite command, none while inactive [B: `lifecycle`];
  - after deactivation and activation the current state is reported again
    [B: `lifecycle`].
- **N14. A zero command in a triggered slowdown zone** has the name `""`:
  the slowed velocity is not strictly smaller than the command (R13), so no
  action is taken. Observed [B: `state`]: `("", 0)` for a zero command with
  a point in the slowdown zone. (UPSTREAM_BEHAVIOUR.md, finding 9, has the
  same.) A slowdown whose ratio is 1 or more is not reported either (same
  finding).
- **N15. Names that coincide** [S only]: because names are compared, a
  change between two zones of the same name, or between a zone named
  `invalid source` and an invalid source, is not reported.

### 1.3 Zone polygons and point markers

- **N16. Polygon publishers** (`Polygon::configure`, `Polygon::publish`,
  `CollisionMonitor::publishPolygons`, `Circle::getPolygon`) [S].
  `<zone>.visualize` (default false) creates a publisher of
  `geometry_msgs/msg/PolygonStamped` on `<zone>.polygon_pub_topic` (default:
  the zone's name) with `SystemDefaultsQoS`: reliable and transient local
  with this middleware [B: `visualize`]. Every visualised zone is published
  once on activation and once after every processed command, also when the
  velocity is withheld; not for a non-finite command and not while inactive
  [B: `visualize`, `lifecycle`]. Contents [B: `visualize`]: frame
  `base_frame_id`; stamp the node clock; for a polygon its vertices as
  `float`, `z = 0`; for a circle 16 points `radius · (cos, sin)(k·π/8)`.
  For a zone fed by a topic (N31) the last accepted message is republished
  with a new stamp and its own frame, and before the first message an empty
  polygon in the base frame [B: `polygon-topic`].
- **N17. Point markers** (`process`) [S]. If `~/collision_points_marker`
  (`visualization_msgs/msg/MarkerArray`) has a subscriber, every processed
  command publishes one `POINTS` marker per source visited, with namespace
  `collision_points_<source>`, frame `base_frame_id` and the source's points
  in the base frame [B: `visualize`].

### 1.4 Who starts the node

- **N18. `nav2_bringup`** (`nav2_bringup/launch/navigation_launch.py`) [S].
  The list `lifecycle_nodes` names `collision_monitor` between
  `velocity_smoother` and `bt_navigator`; a lifecycle manager
  `lifecycle_manager_navigation` gets that list as `node_names`. Without
  composition the node is `Node(package='nav2_collision_monitor',
  executable='collision_monitor', name='collision_monitor',
  parameters=[configured_params], remappings=[('/tf', 'tf'), ('/tf_static',
  'tf_static')])`. With composition it is
  `ComposableNode(package='nav2_collision_monitor',
  plugin='nav2_collision_monitor::CollisionMonitor',
  name='collision_monitor', …)` loaded into the container `nav2_container`.
  `use_composition` defaults to `False` in `navigation_launch.py` and to
  `True` in `bringup_launch.py`, which passes it on. The simulation launch
  files reach the node only through these two. No other launch file of
  `nav2_bringup` names it.
- **N19. The velocity chain** [S]: the controller's output is remapped to
  `cmd_vel_nav` (`navigation_launch.py`); `nav2_velocity_smoother` publishes
  `cmd_vel_smoothed` (`nav2_velocity_smoother/src/velocity_smoother.cpp`);
  the collision monitor reads `cmd_vel_smoothed` and publishes `cmd_vel`
  (its parameters).
- **N20. The package's own launch file**
  (`launch/collision_monitor_node.launch.py`) [S] starts the node with
  `params/collision_monitor_params.yaml` and a lifecycle manager
  `lifecycle_manager_collision_monitor` with `node_names:
  ['collision_monitor']`, with or without composition.
- **N21. The parameter files** [S]. `nav2_bringup/params/nav2_params.yaml`
  (and, with the same values, the three `nav2_multirobot_params_*.yaml`):
  `state_topic:
  "collision_monitor_state"`, `base_shift_correction: True`,
  `transform_tolerance: 0.2`, `source_timeout: 1.0`, `stop_pub_timeout:
  2.0`, one zone `FootprintApproach` (`type: polygon`, `action_type:
  approach`, `footprint_topic: "/local_costmap/published_footprint"`,
  `time_before_collision: 1.2`, `simulation_time_step: 0.1`, `min_points:
  6`, `visualize: False`), one source `scan` of type `scan` (with
  `min_height` and `max_height`, which a scan source never reads).
  `params/collision_monitor_params.yaml` of the package: `polygons:
  ["PolygonStop"]`, a fixed stop polygon with `visualize: True`;
  `observation_sources: ["scan"]`; `state_topic`, `base_shift_correction:
  True`, `transform_tolerance: 0.5`, `source_timeout: 5.0`. It also defines
  `PolygonSlow`, `PolygonLimit`, `FootprintApproach`, `VelocityPolygonStop`
  and a `pointcloud` source, none of which is in the two lists, so none is
  created.

### 1.5 Transforms

- **N22. When.** A scan's points are computed when a command is processed,
  not when the scan arrives (R3): `process` reads the clock once
  (`curr_time`) and calls `Scan::getData(curr_time, …)`, which returns
  failure if no scan is stored, then if the scan is too old (R7), and only
  then looks up the transform (`Scan::getData`) [S]. Every command repeats
  the lookup.
- **N23. Without base shift correction** (`Source::getTransform`;
  `nav2_util/src/robot_utils.cpp`, the `getTransform` overloads without
  time arguments) [S].
  If the scan's `frame_id` equals `base_frame_id` as a string, the transform
  is the identity and nothing is looked up. Otherwise
  `tf_buffer->lookupTransform(base_frame_id, frame_id, tf2::TimePointZero,
  transform_tolerance)`: **the newest transform available**, whatever the
  stamp of the scan and whatever the clock. Observed [B: `tf-latest`]: with
  a transform at the scan's stamp that puts the point outside the zone, the
  command passes; a transform stamped 0.5 s *after* the scan and after the
  clock, putting the point inside, makes the next command stop without a new
  scan; that transform is still used 3.5 s later; a transform with an older
  stamp arriving late changes nothing. The age of a transform is never
  checked.
- **N24. With base shift correction** (the default, `base_shift_correction:
  true`) [S]: `tf_buffer->lookupTransform(base_frame_id, curr_time,
  frame_id, <scan stamp>, odom_frame_id, transform_tolerance)`. tf2 computes
  it as `T(base ← odom at curr_time) · T(odom ← frame at the scan stamp)`
  (`tf2/src/buffer_core.cpp`, `BufferCore::lookupTransformImpl` with a fixed
  frame: `transform = transform2 * transform1`), each factor interpolated
  between the two samples around its time, linearly for the translation
  (`tf2/src/cache.cpp`, `TimeCache::interpolate`) [S]. The product is turned
  into a translation and a quaternion (`BufferCore::lookupTransform`:
  `getRotation()`) [S]. The lookup is made also when the scan is already in
  the base frame. Observed:
  - [B: `shift-straight`] a point 1.2 m ahead at the scan stamp, the base
    1 m further along odom's x at the command: stop by a 0.5 m circle; with
    the base half way (between two odom samples): pass; the same sequence
    with `base_shift_correction: False`: every command passes;
  - [B: `shift-turn`] the base turned by half a turn since the scan: a point
    that was ahead is behind (outside a zone in front), and one that was
    behind is ahead (inside);
  - [B: `shift-sensor-frame`] with the scan in a sensor frame 0.5 m ahead of
    the base, both factors apply;
  - [B: `shift-no-odom`] scan in the base frame, no odom frame: no answer
    until the tolerance has passed, then a stop for `invalid source`.

  The lookup needs odom data **at or after the command's clock reading**.
  A command that arrives 0.25 s after the newest odom sample waits (N26) and
  is then stopped for `invalid source` (`Lookup would require extrapolation
  into the future`) [B: `shift-straight`]. A scan stamped before the oldest
  sample fails likewise (`extrapolation into the past`)
  [B: `shift-sensor-frame`]; tf2 keeps 10 s of samples
  (`tf2/include/tf2/buffer_core.hpp`, `BUFFER_CORE_DEFAULT_CACHE_TIME`) [S].
- **N25. Applying it** (`Scan::getData`;
  `tf2_geometry_msgs.hpp`, `fromMsg`; `tf2/LinearMath/Matrix3x3.hpp`,
  `setRotation`; `tf2/LinearMath/Transform.hpp`, `operator*`) [S]. The
  looked-up message, a translation `(ox, oy, oz)` and a quaternion
  `(qx, qy, qz, qw)`, all `double`, becomes a 3×3 matrix and an origin:

  ```
  d = qx² + qy² + qz² + qw²;  s = 2 / d
  xs = qx·s, ys = qy·s, zs = qz·s
  m00 = 1 − (qy·ys + qz·zs)   m01 = qx·ys − qw·zs
  m10 = qx·ys + qw·zs         m11 = 1 − (qx·xs + qz·zs)
  ```

  and for reading `i` with the scan-frame point `(px, py, 0)` of R3
  (`px = ranges[i] * cos(angle)` in `float`, widened to `double`):

  ```
  x = m00·px + m01·py + m02·0 + ox
  y = m10·px + m11·py + m12·0 + oy
  ```

  in `double`. The third coordinate is dropped: the point is **projected**
  on the base frame's x–y plane. Height, roll and pitch of the sensor are
  not checked, and a scan source has no height filter. Observed
  [B: `tf-tilt`, `tf-pitch-circle`]: a sensor mounted upside down mirrors
  left and right; a sensor 5 m above the base gives the same points; with a
  sensor pitched by a quarter turn, a point 3 m along its x axis lands on
  the base's origin and triggers a 0.5 m stop circle.

  Because of the division by `d` a quaternion that is not of unit length
  acts as its normalisation. tf2 also normalises on entry (a transform is
  stored as a matrix) and refuses NaN (`tf2/src/buffer_core.cpp`,
  `setTransformImpl`) [S]; a transform published with the quaternion
  (0, 0, 0, 2) acted as the identity [B: `tf-tilt`].

  In the macOS arm64 binary the two sums are compiled to fused
  multiply-adds (`fmadd`; `review/collision_monitor/nav2/fma_check.sh`,
  output in `results/macos-arm64-transform.txt`). A build without them, as
  on baseline x86-64, can differ in the last bit of a coordinate. The Linux
  binaries were not examined.
- **N26. Waiting** (`tf2_ros/src/buffer.cpp`, `Buffer::lookupTransform`,
  `Buffer::canTransform` with a timeout) [S]. If the lookup cannot succeed at
  once, the calling thread sleeps 10 ms of wall time at a time while the
  node clock is before `start + transform_tolerance`, `start` being the
  clock when the lookup began, and then tries once more. The callback thread
  is the node's only one (N1): nothing else is handled meanwhile.
  `transform_tolerance` is therefore a **waiting time**, not a bound on the
  age or the extrapolation of a transform. It is converted by
  `tf2::durationFromSec`: whole seconds plus the remainder times 10⁹
  rounded to nearest (`tf2/src/time.cpp`) [S]. Observed [B: `tf-missing`],
  tolerance 0.5 s, scan in a frame without transform: no answer while the
  clock stands still (1.5 s of wall time); none at `start + 0.5 s − 1 ns`;
  at `start + 0.5 s` the stop for `invalid source`. If the transform arrives
  during the wait, the answer comes then, without the clock moving. A scan
  with an empty `frame_id` behaves like one in an unknown frame. With
  `transform_tolerance: 0.0` there is no wait [B: `tf-missing-timeout0`].
  (UPSTREAM_BEHAVIOUR.md, finding 10, has the delayed stop.)
- **N27. Failure.** A failed lookup makes `getData` fail, which is R8: a
  stop named `invalid source` if `source_timeout` is not zero
  [B: `tf-missing`, `shift-straight`, `shift-sensor-frame`,
  `shift-no-odom`]; with `source_timeout: 0.0` the source has no points and
  the command passes [B: `tf-missing-timeout0`].
- **N28. Clock.** `publishVelocity` reads the clock again (D7), now after
  the wait [S]. A backward jump of the clock clears the TF buffer
  (`tf2_ros/src/buffer.cpp`, `Buffer::onTimeJump`) [S]: the transforms
  received on `/tf` are forgotten, those received on `/tf_static` are not
  [B: scenario `tf-clock-jump-back` of `scenario_nav2.py`, added by T07].

### 1.6 Zones whose shape arrives on a topic

- **N29. Which zone gets which subscription**
  (`Polygon::getParameters`, `Polygon::getCommonParameters`) [S].
  - `<zone>.points` present: the shape is fixed. Neither
    `polygon_sub_topic` nor `footprint_topic` is read; with both set next to
    `points` the node subscribes to neither [B: `footprint-topic-ignored`].
  - `points` absent, `action_type: approach`: `footprint_topic` (default
    `local_costmap/published_footprint`) through
    `nav2_costmap_2d::FootprintSubscriber`.
  - `points` absent, any other action: `polygon_sub_topic` is required. If
    it is not set, configuration fails.

  **`footprint_topic` is read for `approach` only.** Therefore Nav2's
  default zone with `action_type` changed to `stop` or to `slowdown` and
  nothing else changed **does not configure**: `[FootprintApproach]: Error
  while getting common polygon parameters: parameter
  'FootprintApproach.polygon_sub_topic' is not initialized`, then `Error
  while getting polygon parameters: static points and sub topic both not
  defined`; `configure` fails and the node stays unconfigured
  [B: `footprint-edit`: `nav2_params.yaml` with `stop`, with `slowdown`, and
  the package's example file with `polygons: ["FootprintApproach"]` and
  `stop`; unchanged, with `approach`, the first configures]. With
  `polygon_sub_topic: "/local_costmap/published_footprint"` added, the
  `stop` zone configures and the node subscribes to that topic
  [B: `footprint-sub-topic`]; `footprint_topic`, `time_before_collision` and
  `simulation_time_step` may stay, they are not read.
- **N30. What the costmap publishes** [S only, the costmap was not run]:
  `nav2_costmap_2d/src/costmap_2d_ros.cpp` publishes `published_footprint`
  as `geometry_msgs/msg/PolygonStamped` with `SystemDefaultsQoS`, in
  `updateMap`, i.e. once per map update (`update_frequency: 5.0` for the
  local costmap in `nav2_params.yaml`), with the header of the robot's pose
  in the costmap's global frame (`odom` for the local costmap) and the
  padded footprint placed at that pose. So the message is the footprint
  **in the odom frame**.
- **N31. Subscription** (`Polygon::createSubscription`) [S]:
  `geometry_msgs/msg/PolygonStamped` on `polygon_sub_topic`,
  `SystemDefaultsQoS`: best effort and volatile with this middleware
  [B: `polygon-topic`, `footprint-sub-topic`].
  `polygon_subscribe_transient_local: true` makes it transient local, still
  best effort; a message published reliably and transient-local before the
  node was configured was **not** delivered [B: `polygon-topic-inactive`].
- **N32. Before the first accepted message** the zone has no shape and
  takes no action (`processStopSlowdownLimit`: `!polygon->isShapeSet()`,
  with a warning per command; `min_points` is not consulted) [S]. A stop
  zone without shape does not stop: with a point 0.2 m ahead the command
  passed [B: `polygon-topic`, `footprint-sub-topic`].
- **N33. A message** (`Polygon::polygonCallback`,
  `Polygon::updatePolygon(PolygonStamped)`) [S], in this order:
  1. fewer than **three** points: an error is logged and the message is
     ignored; the shape stays [B: `polygon-topic`, two points and none].
     Three points are enough [B: same case, a triangle], where a fixed
     polygon needs four (R10).
  2. The transform from the message's frame to the base frame: the identity
     if the names are equal, else the newest transform with the wait of N26,
     **inside the subscription callback**. On failure the message is
     ignored and the shape stays [B: `polygon-topic`, frame `nowhere` and an
     empty `frame_id`: nothing is answered until the tolerance has passed,
     and the old shape is still in force].
  3. The vertices become `M·(x, y, 0) + o` as in N25, x and y kept, from the
     `float` coordinates of the message; `z` is ignored. The message is
     stored for N34 and N16.

  The callback runs to completion before the next command is handled (N1),
  so a command sees the old shape or the new one, never a mixture [S]. The
  stamp of the message is not read, and a shape never becomes too old [S].
  The values are not checked: a box with one NaN coordinate was accepted and
  the zone then no longer contained a point inside the box; three equal
  points and a polygon whose first point is repeated at the end were
  accepted [B: `polygon-topic`].
- **N34. At each command** (`process`, second loop;
  `Polygon::updatePolygon(const Velocity &)`) [S]. For every enabled zone
  the loop reaches (it ends at the first stop, R12, and at once if the
  source is invalid): if the stored message's frame is not the base frame,
  the newest transform is looked up again, with the wait of N26, and the
  vertices are recomputed from the stored message; if the lookup fails the
  vertices stay as they were. A shape given in another frame therefore
  follows that frame between messages [B: `polygon-topic-moving`].

  For the footprint this means: the zone is where the footprint was **in
  odom** when the costmap last published it. Observed
  [B: `footprint-sub-topic`]: footprint received at the base's position:
  stop for points 0.1 m ahead; base moved 1 m with no new footprint message:
  the same points pass, and points 1.1 m behind the base, where the
  footprint was, stop; after a new footprint message: stop again. Between
  two costmap updates the zone lags by the distance driven; if the costmap
  stops publishing, the robot drives out of its zone.
- **N35. Lifecycle.** Shape messages are taken while inactive (N4) and lost
  at cleanup (N7).
- **N36. A circle** with `polygon_sub_topic` receives its radius as
  `std_msgs/msg/Float32` (`Circle::createSubscription`,
  `Circle::radiusCallback`) [S only]. Not part of this extension.

## 2. What the documentation does not say

Flagged, as in SPEC.md. Each is upstream's behaviour and is kept unless a
difference in section 6 says otherwise.

- The edit "change `action_type` of Nav2's default zone" does not work
  (N29). A stop or slowdown zone needs `points` or `polygon_sub_topic`.
- A stop zone fed by a topic protects nothing until its first message, and
  nothing at all if the publisher never starts (N32).
- A footprint zone fed from the costmap lags behind the robot and is left
  behind if the costmap stops (N34). A fixed polygon with the footprint's
  coordinates has neither problem.
- `transform_tolerance` is how long a command is held up, by the node
  clock, before a missing transform turns into a stop (N26). Under a clock
  that stands still the node never answers.
- Without base shift correction the newest transform is used however old
  it is (N23). With it, every command waits for an odom sample newer than
  itself (N24).
- A tilted sensor is projected, not rejected (N25).
- A node shut down from the active state keeps forwarding commands (N8).
- Deactivating and activating restarts the `stop_pub_timeout` period and
  re-reports the state (N6).
- A polygon message with a NaN coordinate blinds the zone (N33).

## 3. The model after the extension

Additions to `core/ROSleanCore/CollisionMonitor.lean`. Names are proposals;
T05 may choose others but not other meanings. Everything stays `Nat`, `Int`,
`Bool` and lists; no `Float`; `step` stays total.

(T05, T05b) The names chosen: a zone's shape is `ZoneShape.fixed s` or
`ZoneShape.topic`; a coordinate pair as the shim hands it over is `Coords`
(two binary64 patterns, metres, base frame); `State.init cfg` takes the
configuration; the zones with their shapes are `liveConfig cfg st
zoneVerts`; the shapes after a command are `shapesAfter`; the acceptance of
a shape message is `acceptShape`; the report of an action is
`Action.report`; `Output.velocity` and `Output.report` read an output. The
points, the two conversions the model applies (`scanPointMm`,
`vertexPointMm`) and the bit-pattern reading are in
`core/ROSleanCore/CollisionMonitorPoint.lean`.

### 3.1 Data

- (T05b) **There is no transform in the model.** `Xform`, `Xform.usable`,
  `Xform.pointMm` and `Xform.vertexMm` of T04 and T05 are gone (section 5).
- **`Coords`**: a pair of binary64 bit patterns, the `x` and `y` in metres
  of a point **in the base frame**, as the shim computed them (N25).
- **`Beam`** carries the range pattern only. Where a reading's point lies
  is not stored with the scan: it depends on the transform at the time of
  each command (N22, N24) and arrives with the command.
- **`scanPointMm c : Point`**: each coordinate of `c` in millimetres,
  truncated toward zero and saturated at `±coordLimit` (`coordMm`, as before
  the extension; a NaN is `+coordLimit`). **`vertexPointMm c : Point`**:
  each coordinate rounded to the nearest millimetre, a half away from zero
  (`vertexCoordMm`), for the vertices of a topic zone; a coordinate that is
  not finite or is 1024 m or more becomes `±coordLimit`, which no shape may
  reach, so such a vertex is always refused (E7, ND6).
- **`Scan.fits s c`**: the list `c` has exactly one pair per reading of
  `s`. **`Scan.points s c`**: reading `i` is paired with pair `i` of `c`;
  the pairs of the readings that are not valid (R4) are dropped; the others
  become points by `scanPointMm`, in scan order. Which readings count is
  decided here, from the stored ranges and limits, not by the shim.
- **Zones.** A zone's shape is `fixed (s : Shape)` or `topic`. A topic zone
  has no shape in the configuration.
- **`TopicShape`**: `foreign : Bool` (the message's frame is not the base
  frame), `current : List Point` (the vertices in the base frame, in
  millimetres: upstream's `poly_`). (T05b) The vertices as received are no
  longer kept in the state: the shim keeps the message and recomputes the
  base-frame vertices for each command (N34).
- **`Report`**: `normal`, `stop (zone)`, `slowdown (zone)`, `source`: what a
  state message says (N13).
- **`Phase`**: `unconfigured`, `inactive`, `active`, `finalized`.
- **`State`** gains `phase` (initially `unconfigured`), `prevReport`
  (initially `normal`) and `shapes : List (Option TopicShape)`, one entry
  per zone, `none` for a fixed zone and for a topic zone without shape.
  (T05) For the list to have one entry per zone from the start, the initial
  state is a function of the configuration: `State.init cfg`.
- **`Output`**: `silent`, or `answer (velocity : Option Action) (report :
  Option Report)` for a processed command. `answer none _` is today's
  silence of R15; `answer (some a) _` is today's `publish a`. The
  distinction between `silent` and `answer none none` is what tells the
  shim whether to publish the zone polygons (N16).

### 3.2 Events and `step`

Each rule names what upstream does (N…) and where the model differs (ND…).

- **E1 `configure`.** `unconfigured` → `inactive`. In any other phase: no
  change. Output `silent`. (N3)
- **E2 `activate`.** `inactive` → `active`. Otherwise no change. `silent`.
  (N5)
- **E3 `deactivate`.** `active` → `inactive`, `prevZero := false`,
  `prevReport := normal`. Otherwise no change. `silent`. (N6)
- **E4 `cleanup`.** `inactive` → `unconfigured`, `scan := none`, every
  entry of `shapes` `:= none`. Otherwise no change. `silent`. (N7)
- **E5 `shutdown`.** Any phase but `finalized` → `finalized`. `silent`.
  From `finalized` no event changes the state and every output is `silent`
  (ND4).
- **E6 `scan now s`.** In `inactive` and `active`: `scan := some s`. In
  `unconfigured` and `finalized`: no change. `silent`. (N4, R5)
- **E7 `shape k verts foreign`** (T05b), with `verts : Option (List
  Coords)` the vertices of the message **in the base frame**, as the shim
  computed them (N33, step 3), or `none` if the transform from the message's
  frame could not be looked up, and `foreign` saying that the message's
  frame is not the base frame. The message is **accepted** iff the phase is
  `inactive` or `active`, zone `k` exists and is a topic zone, `verts = some
  vs`, `vs` has at least three entries, and every vertex of `vs.map
  vertexPointMm` lies within the coordinate limit of `Shape.ok` (which
  excludes every coordinate that is not finite). If accepted: `shapes[k] :=
  some { foreign, current := vs.map vertexPointMm }`. Otherwise no change.
  `silent` in both cases. (N33, ND6)
- **E8 `cmd now v scanPts zoneVerts`** (T05b), with `scanPts : Option (List
  Coords)` (the base-frame coordinates of the readings of the stored scan,
  one pair per reading in scan order, as the shim computed them under the
  transform it looked up for this command; `none` if the lookup failed or
  was not made) and `zoneVerts : List (Option (List Coords))` (one entry per
  zone: the base-frame vertices of that zone's stored message under the
  transform looked up for this command, or `none`).
  1. Phase not `active`: no change, `silent`. (N4)
  2. `v` not finite: no change, `silent`. (R2)
  3. **Source** (`sourcePoints`, R6–R8 with N22 and N27): `some []` if the
     source is disabled; if no scan is stored, `some []` when the timeout is
     0 and `none` otherwise; if the scan is too old (timeout not 0), `none`;
     otherwise, if `scanPts = some c` and `s.fits c`, `some (s.points c)`;
     otherwise (no coordinates, or not one pair per reading) `some []` when
     the timeout is 0 and `none` when it is not.
  4. **Shapes** (N34, ND3): if the source is `none`, unchanged. Otherwise,
     for every enabled topic zone `k` with `shapes[k] = some sh` and
     `sh.foreign`: if `zoneVerts[k] = some vs`, `vs` has as many entries as
     `sh.current`, and every vertex of `vs.map vertexPointMm` is within the
     coordinate limit, `current` is replaced by that list; otherwise it
     stays.
  5. **Action** (`evaluate`): `stopSource` if the source is `none`; else
     `zonesAction` (R12, R13) over the zones, where a zone is triggered iff
     it is enabled, **has a shape** (a fixed one, or `shapes[k] = some _`
     after step 4), and at least `minPoints` points are inside it; a topic
     zone's shape is the polygon `current`, with the margin of D4. (N32)
  6. **Report** (N13, N14): `r := source` for `stopSource`; `stop k` for
     `stopZone k`; for `slowdown k _`: `slowdown k` if `v` is not zero in
     the three components and `normal` if it is; `normal` for `pass`. The
     output's report is `some r` if `r ≠ st.prevReport` and `none` if it is
     equal.
  7. **Velocity** (R15, unchanged): `none` iff the velocity is zero,
     `prevZero` holds and `stopStamp + stopPubTimeout < now`; else `some
     action`.
  8. New state: `prevZero`, `stopStamp` as today; `prevReport := r`;
     `shapes` from step 4. Output `answer velocity report`.

  (T05b) Two checks in steps 3 and 4 are there because the coordinates now
  come from the shim: a list of scan coordinates that is not one pair per
  reading counts as a failed lookup, and zone vertices that are not as many
  as the shape has are not taken. A shim that works as specified produces
  neither; the model does not rely on it.

`step` for a monitor that is not ok stays as it is ("Fail closed" in
SPEC.md): every command in the `active` phase is answered `stopSource`,
whatever else; in other phases `silent`.

### 3.3 What the model does not contain

The wait of N26 (the event carries only its outcome), frame names and (T05b)
the transform itself (the shim resolves a frame into a transform, applies it
and hands over points, or `none`), the scan's angles and the scan-frame
points (R3), the bond, publishers and QoS,
the polygons shown for `visualize`, zone names (the shim maps a zone index
to its name).

## 4. Properties

Proved by T05 about `step` and lifted to `run`, in the manner of SPEC.md,
"Properties", and reworked by T05b for option A. `cfg`, `st`, `now`, `v`,
the coordinates are arbitrary unless restricted; no property needs
`Config.ok` unless it says so. "The output of the command" is `(step cfg st
(.cmd now v scanPts zoneVerts)).2`. *Source invalid* means:
`cfg.sourceEnabled`, `0 < cfg.sourceTimeout`, and either `st.scan = none`,
or `st.scan = some s` with `s.stamp + cfg.sourceTimeout < now`, or `scanPts`
is `none` or does not fit the stored scan.

**Processed (iff).** The output of an event is not `silent` iff the event is
a command, `st.phase = active` and `v.finite`; it is then `answer vel rep`
with the values of **Velocity** and **Report**. Every other event, and every
event in another phase, gives `silent`.

**Inactive.** If `st.phase ≠ active`, the output of every event is `silent`,
and a command leaves the state unchanged. On runs from `State.init`: no
output before the first `activate` is other than `silent`; no output after a
`shutdown` is; between a `deactivate` and the next `activate` none is.

**Velocity (iff).** For a processed command, `vel = none` iff `(evaluate
…).yieldsZero v` and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout <
now`; otherwise `vel = some (evaluate …)`. This is "Output" of SPEC.md.

**Report (iff).** For a processed command, `rep = some r` iff `r` is the
report of step 6 and `r ≠ st.prevReport`; and the new `prevReport` is the
report of step 6 in both cases. In particular a report is made also when
`vel = none`.

**Only if.** `evaluate` is `stopSource` iff the source is invalid; `stopZone
k` iff the source is not invalid and `k` is the first triggered stop zone;
`slowdown k r` iff the source is not invalid, no stop zone is triggered, and
`k` is the first triggered slowdown zone with the smallest ratio pattern
`r`, below `F64.one`; `pass` iff the source is not invalid and no zone of
either kind is triggered. "Triggered" as in E8 step 5.

**No transform.** If `cfg.sourceEnabled`, `0 < cfg.sourceTimeout`,
`st.phase = active`, `v.finite` and `scanPts` is `none` (or does not fit the
stored scan), then `evaluate = stopSource` and the output is `answer (some
stopSource) _` or `answer none _`, the latter exactly in the case of
**Velocity**. It is **false without the two hypotheses on the
configuration**, as "Stale data" is: with `source_timeout: 0` a missing
transform means "no obstacle" (N27; example in `Examples.lean`).

**Stale data, Stop, Bad data, No amplification, Slowdown bound,
Pass-through, Monotonicity** of SPEC.md hold with these changes: the
hypothesis `st.phase = active` is added; "the output is `publish a`"
becomes "`answer (some a) _`" and "`silent`" in their conclusions becomes
"`answer none _`"; the points are `s.points c` for the `scanPts = some c`
of the command. Stale data keeps its conclusion whatever `scanPts` is.
Pass-through needs coordinates that fit among its hypotheses when a scan is
stored (it is part of "`sourcePoints = some pts`"). (T05b) Bad data (a) and
Monotonicity are stated on the stored scan **and** the command's
coordinates together: removing the readings that are not valid, with their
pairs, changes no output (`output_filter_validBeam`); adding readings, each
with its pair, never lifts a stop (`evaluate_isStop_mono`).

**Scan points (T05b; replaces "Transform").** What is still a function of
the stored scan, and what is not:

- *Of the stored scan alone:* whether it is too old (R7); which readings
  are valid (R4); **how many points there are**: if `s.fits c`, `(s.points
  c).length` is the number of valid readings of `s`, whatever `c` is
  (`points_length`), and never more than that for any `c`
  (`points_length_le`); so no points iff no reading is valid
  (`points_eq_nil_iff`). The shim can neither add a point nor drop one.
- *Of the command's coordinates:* **where** each counted reading lies. Point
  `j` is `scanPointMm` of the pair at the position of the `j`-th valid
  reading (`points_eq`, `mem_points_iff`): for finite coordinates, the value
  the shim handed over, in millimetres, truncated toward zero and saturated
  (`coordMm_eq_truncReal`), less than 1 mm from it unless saturated
  (`coordMm_sub_lt`).
- *Trusted (T2, T6):* that pair `i` is the image of reading `i` of the scan
  the model has stored, under the transform N23 or N24 prescribes for this
  command. The model checks the number of pairs and nothing else about
  them.

T04's property **Transform** ("`Xform.pointMm` is the exact image,
truncated"; `Xform.vertexMm` within 1/2 mm of the exact image; the identity
gives `coordMm`) was proved by T05 and cannot be stated any more: the model
has no transform. What replaces each theorem is listed in THEOREMS.md, "What
T05 proved and option A does not have". For topic-zone vertices:
`vertexPointMm` is within 1/2 mm of the vertex the shim handed over, per
axis, for every accepted vertex (`vertexPointMm_sub_le`).

**Shape accepted (iff).** A `shape k verts foreign` event changes the state
iff it is accepted (E7); then `shapes[k]` is as E7 says and nothing else
changes: not the other entries of `shapes`, not `scan`, `prevZero`,
`stopStamp`, `prevReport`, `phase`.
(T05) **False as an "iff".** Counterexample: the same accepted message
delivered twice; the second is accepted and the state does not change
(`step_shape_same`). What holds and is proved: the conditions of E7 are
equivalent to "the message installs a shape" (`acceptShape_eq_some_iff`); a
message that is not accepted changes nothing; an accepted one sets
`shapes[k]` as E7 says and changes nothing else; hence the state changes
**only if** the message is accepted.
(T05b) At a command, a refresh moves the vertices of a foreign shape and
does nothing else: the frame flag and the number of vertices stay
(`refresh_foreign`); vertices that are refused leave the shape as it was
(`refresh_of_refused`).

**No shape.** If zone `k` is a topic zone and `st.shapes[k] = none`, zone
`k` is not triggered by any points, and no output for a command names it.
On runs from `State.init`: no output names a topic zone `k` before an
accepted `shape k` event that follows the last `cleanup`.
(T05) "The last cleanup" must be one that took effect: a `cleanup` outside
the inactive phase is a no-op by E4 and discards nothing. Proved as: after
any history that leaves the node unconfigured, and any events that follow
among which there is no `shape k` event, no output names zone `k`
(`run_output_of_no_shape`). This is upstream's
behaviour (N32) and it is **not** fail-safe: a stop zone without shape does
not stop. See question 1 in section 13.

**Replacing a shape affects that zone only.** For two states that differ
only in `shapes[k]`, and any points: every zone other than `k` is triggered
in the one iff in the other. Hence the actions for a command differ only if
zone `k` is triggered in one and not in the other.

**Lifecycle invariants.** On every state reachable from `State.init`: if
`phase ≠ active` then `prevZero = false` and `prevReport = normal`; if
`phase = unconfigured` then `scan = none` and every entry of `shapes` is
`none`; `shapes` has one entry per zone and `some` only at topic zones;
every `current` has at least three vertices, all within the coordinate
limit.
(T05) **The first clause is false for the finalized phase**, with E5 as
written. Counterexample: `configure`, `activate`, a finite command while no
scan is stored and `source_timeout` is not 0 (a stop: `prevZero = true`,
`prevReport = source`), `shutdown`. E5 changes the phase only, so the
finalized state has `prevZero = true`. The model was not changed. Proved:
the clause for `phase = unconfigured` or `phase = inactive` (`Inv.quiet`),
which is what "After activation" needs; in the finalized phase the two
fields are never read again (`step_of_finalized`). The other clauses are
proved as stated.

**After activation.** If `step` on `activate` moves the state to `active`,
then for the next finite command: if the velocity is zero it **is
published** (`vel = some _`), and the report is `some r` unless `r =
normal`. (N6: a stop is never withheld right after an activation.)

**Totality.** As before; the conversions (`scanPointMm`, `vertexPointMm`,
and with T06 the decoding of coordinates and vertices from byte arrays of
any length) are total.

Trace theorems (`Trace.lean`): each of the above lifted to `run` from
`State.init` over arbitrary event lists, including lists with lifecycle
events in orders the lifecycle does not allow (they are no-ops by E1–E5).

## 5. Where the transform is applied

The transform's numbers come from tf2 in both options; neither option
checks tf2. The question is who computes N25.

**Option A.** The shim computes N25 in `double` and hands Lean points in
metres as now. For every command it must hand over the whole point list
again, because the transform depends on the command's time (N22, N24).

**Option B.** The shim passes the looked-up transform as six bit patterns
with the command; Lean computes the image exactly.

**Decision (the user, 2026-11-24; tasks/PLAN.md, "Decisions"): option A.**
T04 recommended B, and T05 built and proved B. The reasons for A: B costs
1.2 to 1.9 µs per reading and command (below); through the node, task T02
measured about 20 times upstream's time per command, and commands left
unanswered at 10,000 readings. With A the node does what upstream does, in
the arithmetic upstream uses. **Also decided:** the shim is compiled with
`-ffp-contract=off` on every platform, as the node is today (the test of
task T01 that enforces the flag stands): no fused multiply-add. T05b
reworked the model and the proofs; T06 writes the shim.

| | A (decided) | B (built by T05, removed by T05b) |
| --- | --- | --- |
| Trusted, beyond tf2 and R3 | The matrix and the two sums of N25 in C++, about 15 lines, **and how they are compiled**; that the points handed over are those of the stored scan | Nothing more |
| Agreement with upstream before the millimetre | Bit for bit where upstream's build does not fuse either (baseline x86-64 cannot; inference from the instruction set, the Linux binaries were not examined). Upstream's arm64 binary fuses the two sums (N25), so there the last bits of a coordinate can differ | Not bit for bit. Lean's point is the exact image; upstream's is that image computed in `double` |
| Model | The command event carries the coordinates; the stored scan decides which readings count and how many points there are, the command where they lie | The command event carries six numbers or `none`; points are a function of the stored scan and the transform, inside the theorems |
| Cost per reading and command | The shim's arithmetic, a few ns (estimated), plus decoding and conversion in Lean, now per command instead of per scan: 41 ns per reading was the figure before the extension, 0.19 µs is what the model costs now (below). Not measured through a node (T06) | Measured below: 1.2 to 1.9 µs exact; 0.45 µs in a fixed-point form |

**What the model receives (T05b).** With every command: one pair of
binary64 patterns per reading of the stored scan, the reading's point in
the base frame in metres, in scan order; or "none". Bit patterns, converted
in Lean by the existing `coordMm`, and not millimetres computed in C++,
because the conversion (truncation toward zero, saturation, what a NaN
becomes) is part of what the theorems are about and is proved; the shim
still "converts nothing". One pair per reading, and not only the valid
readings' points, because then the validity of a reading (R4) and the
number of points stay decisions of the model, taken from the stored scan:
the shim cannot drop or add a point (section 4, "Scan points"). The price
is 16 bytes per reading and command also for readings that do not count.
For a topic zone in another frame: the base-frame vertices of the stored
message, as binary64 pairs, or "none", per zone.

**Cost of A in the model, measured (T05b).** With the present interface
(a scan call remembers the coordinates, every command carries them),
`ros/roslean_collision_monitor/test/cmd_cost` (task T02's program: 1080
readings, nothing near, median of 400 commands back to back, macOS arm64,
machine in use by other tests):

| | Two circles | Two 4-vertex polygons |
| --- | --- | --- |
| Before the extension (`91c21f9`, T02's COMPARISON.md) | 18 µs | 0.41 ms |
| After T05 (option B; T02) | 2.5 µs per reading, about 2.7 ms | 2.9 µs per reading, about 3.1 ms |
| After T05b (option A) | **0.21 ms**, 0.19 µs per reading | **0.63 ms** |
| After T03 (option A, the fast command path) | **0.026 ms**, 24 ns per reading | **0.044 ms** |

(T03, added: the last row is task T03's, measured with the same program,
which now takes the lowest of three medians: COMPARISON.md, "Cost of a
command". The exported command call converts each coordinate on a machine
word, produces the points once, and prepares each polygon once per command;
it is proved to be `step` (`stepCmdFast_eq`). The rest of this paragraph
describes the state after T05b, which T03 was given to repair.)

So a command is 13 and 5 times cheaper than under B, and for circle zones
still 12 times dearer than before the extension. The difference to "before"
is the conversion to millimetres, which was made once per scan and under
option A is made for every command, because the points arrive with the
command. Of the 0.21 ms, about 0.14 ms is `coordMm` on 2160 coordinates
(measured by replacing the conversion with a constant, then restored),
0.05 ms the lists (`zip`, `filter`, `map`), 0.02 ms the zone tests. The
conversion is slow here because a coordinate is kept as a `Nat` pattern and
a pattern with the sign bit set is a big number for Lean's runtime; before
the extension the decoder took sign and magnitude apart
(`coordMmOfParts`, 41 ns per reading). T06's decoder can do that again, with
a proof that it computes `Scan.points`; this is an open item for T06, not
done here. `test_cm_ffi` is back to 47 s (169 s after T05, 39 to 42 s
before).

**Cost of B, measured (history).**
`review/collision_monitor/nav2/TransformCost.lean` is a Mathlib-free
prototype of the exact transform on unbounded integers, compiled with
`leanc -O3`; `run_transform_cost.sh` runs it; output in
`results/macos-arm64-transform.txt`. On an Apple M-series core, while
another job used the machine, over eight runs: **1.2 to 1.5 µs per point**
for a quaternion with four non-zero components, **1.6 to 1.9 µs** for a
pure yaw. A fixed-point variant in the same file costs **0.43 to 0.50 µs
per point**. T05's implementation in the model measured the same: the leak
test of `test_cm_ffi` took 169 s, about 1.3 µs per reading and command.

**How far the model's points are from upstream's (option A).** For one
reading let `e` be the exact image of its scan-frame point under the
transform tf2 returned, `u` the point upstream computes, `s` the point the
shim computes, `m` the model's point. Per axis:

- `|m − s| < 1 mm`, toward the origin, unless saturated: **proved**
  (`coordMm_sub_lt`). For a topic-zone vertex `|m − s| ≤ 1/2 mm`: **proved**
  (`vertexPointMm_sub_le`).
- `|u − s| ≤ δ`: **not proved; estimated and trusted (T7).** Both are
  evaluations in `double` of the same expression (N25) on the same seven
  numbers from tf2: the matrix entries, about six operations each on values
  of magnitude at most 2, then three products and three sums on values
  below `|p| + |o|`. Each IEEE-754 operation is within half a unit in the
  last place of its exact result, 1.1·10⁻¹⁶ relative; with coordinates and
  translations below 1000 m that gives each evaluation a distance of at
  most about 5·10⁻⁹ mm from `e`, fused or not, hence `δ` of the order of
  **10⁻⁸ mm**. This document allows **10⁻⁶ mm**. Where neither build fuses,
  and the shim uses tf2's own inline code for the matrix and the product
  (8.4), `s = u` bit for bit (inference; not checked on a Linux binary).
  Errors inside tf2 (interpolation, the product of N24 with its large odom
  coordinates) do not enter `δ`: both programs receive the same numbers.

**Does the polygon margin of √5 mm still guarantee "a point upstream counts
is always counted"?** Yes, with room to spare, under the same split into a
proved and a prose half as SPEC.md ("Safe direction"). The proved half is
per axis: if every vertex of the model is within `dv` of upstream's and the
model's point within `dp` of upstream's point `u` on the boundary, with
`2·(dv + dp)² ≤ 5`, that is `dv + dp ≤ 1.5811 mm`, the point is counted
(`contains_of_close`). The budget:

| Zone | `dv` | `dp` | Room for `δ` | Theorem |
| --- | --- | --- | --- | --- |
| Fixed polygon | 1/2 (rounding of the parameter) | 1 + δ | **δ ≤ 0.08 mm** | `contains_of_shim_fixed` |
| Topic zone | 1/2 + δ (rounding of the shim's vertex, which is within δ of upstream's) | 1 + δ | **δ ≤ 0.04 mm** | `contains_of_shim` |
| Circle | radius rounded up, plus 1 mm | truncation toward the origin, δ | **δ ≤ 0.5 mm** | `circle_safe_shim` |

So the guarantee holds if the shim's coordinates are within 0.04 mm of
upstream's; the allowance of 10⁻⁶ mm is 40,000 times smaller. **Proved:**
the three theorems, with `δ` as a hypothesis. **Prose:** that the boundary
passes over the point (THEOREMS.md, "Geometry"). **Trusted:** the bound on
`δ` (T7), the shim's arithmetic and flags (T2). The same budget is what any
later change of the shim's arithmetic has to respect: a shim whose point is
within 0.04 mm of upstream's keeps the guarantee, whatever its flags.

For the differential tests nothing changes: they exclude points within
5 mm of a boundary (SPEC.md, "Note on testing"), far more than `δ`.

**Base shift correction.** The same in both options: it is one transform,
composed inside tf2 (N24), and reaches the shim as the same seven numbers.
Its precision is tf2's: two interpolations, one product of transforms and
one conversion from matrix to quaternion, in `double`, none of it in the
shim under either option. Its order of operations is N24 followed by N25.

**What option A gives up, for the record.** The points are no longer a
function of what the model stores; "every point is the exact image of its
reading, truncated" is no longer a theorem but "every point is the shim's
point, truncated"; about 15 lines of floating-point arithmetic and the
compiler's treatment of them join the trusted base; and the restriction to
correctly compiled flags is of the kind of CM3/CM4 finding 5. What it
gains: upstream's cost per command, upstream's arithmetic, and on builds
that do not fuse, upstream's points.

## 6. Differences from upstream

In addition to D1–D14 of SPEC.md, which stay unless section 9 changes them.

| # | Upstream | ROSlean | Why |
| --- | --- | --- | --- |
| ND1 | The transform is applied in `double`, with or without fused multiply-add depending on the build (N25); the arm64 binary fuses | (T05b) Applied in `double` by the shim, by the same formula, **never fused** (`-ffp-contract=off`, decided by the user); then truncated to millimetres in Lean (`coordMm`) | Section 5. Before truncation the two are equal where upstream's build does not fuse, and differ by an estimated 10⁻⁸ mm at most where it does; after it, as D3 and D4 already say. |
| ND2 | The clock is read at the start of `process` and again in `publishVelocity`, after any wait for a transform (N28) | One reading per command, taken before the lookups | Extends D7. The two readings can now differ by up to `transform_tolerance` per lookup that waits, instead of by the run time of the callback. Affects only the boundary of `stop_pub_timeout`, and only for commands that waited. |
| ND3 | The transform of a topic zone's shape in another frame is looked up when the zone loop reaches the zone (N34) | The shim looks up the transforms of all such zones before the command is evaluated, and the model refreshes all enabled ones | The lookups cannot be interleaved with a decision taken in Lean in one call. Differences: lookups, and their waits, for zones behind a triggered stop zone; and the vertices of such a zone are refreshed where upstream leaves them until the zone is next reached. No command is decided differently unless a later lookup for that zone fails. |
| ND4 | Shut down from the active state, the node keeps answering commands (N8) | After `shutdown` nothing is published (E5) | A finalized node that drives the robot is a defect. Nav2's manager never takes that path (N8). |
| ND5 | A state message is published when the acting zone's **name** changes (N13, N15) | When the report changes, which identifies a zone by its position. A configuration in which two zones have the same name, or a zone is named `invalid source` or has an empty name, is refused | Only then can the two rules differ. |
| ND6 | A shape message is accepted whatever its coordinates; a NaN blinds the zone (N33) | A message with a non-finite coordinate, or a vertex beyond the coordinate limit of D12, is ignored and the previous shape stays (E7) | Integers cannot hold it, and the previous shape protects more than a blind one. If there is no previous shape the zone stays without one (question 1). |
| ND7 | tf2 does not return a non-finite or all-zero quaternion, by N25 [S] | (T05b) Such a transform counts as a failed lookup: the **shim** checks the seven numbers before it uses them and hands over "none" (8.4; trusted and tested there, since the model no longer sees the transform) | Totality; fail closed. Not reachable through tf2 as read. Without the check a NaN would become a saturated point, which lies in no zone. |
| ND8 | `~/toggle`; `enabled` of a zone or source changeable at run time (R19) | Not provided; an attempt to change a parameter at run time is refused | Scope. Was part of D2. |
| ND9 | `~/collision_points_marker` (N17) | Not provided | Display only. (T05b) Under option A the shim has the base-frame points and could publish them; still out of scope. |
| ND10 | Loadable as the component `nav2_collision_monitor::CollisionMonitor` (N18) | A stand-alone executable only, unless T06 finds the Lean runtime can be loaded into Nav2's container | See section 8.5. |
| ND11 | A zone without `points` and with an action other than `approach` and no `polygon_sub_topic` fails configuration with a message about `polygon_sub_topic` (N29) | The same failure, and if `footprint_topic` is set the message says that `footprint_topic` is read for `approach` only and names the two ways out | The edit users will try first (N29). |
| ND12 | A circle's radius can arrive on a topic (N36) | Refused | Scope. |

Not differences: the lifecycle transitions and what each creates and
destroys (N2–N7); the bond (N9); that commands are dropped while inactive
and scans and shapes are stored; the state topic's contents and timing
(N12–N14) apart from ND5; the polygon publishers (N16); which transform is
looked up, at which times, with which timeout, and the wait (N23, N24,
N26); the projection (N25); that a failed lookup stops only with a non-zero
`source_timeout` (N27); that a topic zone without shape takes no action
(N32); the three-point minimum and the handling of a message whose transform
fails (N33); that a shape in another frame follows the newest transform and
is left behind when its publisher stops (N34).

## 7. What is trusted after the change

Replaces T1–T5 of SPEC.md.

- **T1. The slowdown multiplication.** Unchanged.
- **T2. The scan's points in the base frame (T05b: larger than under
  option B).** (a) The computation of R3 in `float`: the angle
  accumulation, `cos`, `sin`, the product, widened to `double`. (b) **The
  transform arithmetic of N25 in `double`**: the matrix from the
  quaternion, the two sums, the projection; for a topic zone's vertices the
  same. (c) **How (a) and (b) are compiled**: with `-ffp-contract=off`, so
  that no multiply-add is fused (decided by the user; enforced by the
  build's test from task T01), and which library entries `cos` and `sin`
  resolve to. (d) **That the coordinates handed over with a command are
  those of the scan last handed to Lean**, one pair per reading, in order,
  under the transform looked up for that command; and likewise that the
  vertices handed over for a zone are those of the message Lean last
  accepted for it. Lean checks the number of pairs and of vertices, and
  nothing else about them. The sensor pose arithmetic of D11 is gone.
  Everything after the base-frame coordinates (which readings count,
  millimetres, saturation, the zones) is in Lean.
- **T3. The configuration.** As before, plus that the shim tells Lean
  truthfully which zones are topic zones.
- **T4. The bit-pattern definitions** and the formula for the number a
  pattern denotes.
- **T5. The shim's delivery.** Every message is one event with one clock
  reading; every lifecycle transition that `rclcpp_lifecycle` performs is
  delivered as its event, in order, and no other; the fields named in
  section 8 are passed unaltered; the node publishes what section 8 says
  for Lean's output and nothing else.
- **T6. The transforms (new).** (a) tf2: that the transform it returns is
  the one between the frames and at the times asked for (its tree, its
  interpolation, its product, its conversion to a quaternion). (b) The
  shim's lookup: that it asks tf2 for what N23, N24, N33 and N34 say, for
  the frame and stamp of the scan it last handed to Lean and the frame of
  the shape message Lean last accepted for that zone; that it uses the
  seven numbers unaltered (T2 b); that a failed lookup, and a transform
  with a number that is not finite or with a zero quaternion (ND7), is
  passed as "none". This code sits in one file with its own tests (8.4).
- **T7. The distance between the shim's points and upstream's (new, for
  the comparison only; T05b).** That the point and the vertices upstream
  computes by N25 are within 0.04 mm per axis of those the shim computes,
  for coordinates below 1000 m. The estimate is 10⁻⁸ mm (section 5): two
  evaluations of one expression in `double`, one of them possibly with
  fused multiply-add. It enters no theorem about this node as a fact; it is
  the hypothesis `δ` of `contains_of_shim`, `contains_of_shim_fixed` and
  `circle_safe_shim`, on which "a point upstream counts is always counted"
  rests.
- **T8. Display (new).** The names in state messages (the shim's table from
  zone index to name) and the polygons published for `visualize`. A mistake
  here misinforms an observer; it changes no velocity.

What the theorems say: for every sequence of lifecycle transitions, scans,
shape messages and commands, with whatever coordinates the shim handed over
or failed to hand over, which velocity action and which report the node
gives, including that it gives none unless active, that a missing transform
stops under the stated hypotheses, that the number of points is the number
of valid readings of the stored scan, and that every point is the
coordinate pair it was given, truncated toward the origin.

What they do not say: that a coordinate pair is the image of its reading
(T2), under the right transform (T6); that the scan-frame point is the
right one (T2); anything about time spent waiting, about the order in which
the middleware delivers messages, about the bond or the lifecycle manager;
that a topic zone has a shape, or a current one (N32, N34); that the base
stops when the node is silent (A3).

## 8. Shim boundary

In the form of SPEC.md, "Shim boundary", whose general rules stand: scalars
and Lean objects only; the shim converts nothing and refuses no value; every
object argument is consumed.

### 8.1 Start-up and lifecycle

| Call | Arguments | Meaning |
| --- | --- | --- |
| `roslean_cm_builder_add_topic_polygon` | builder, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a topic zone (no shape) |
| `roslean_cm_on_lifecycle` | monitor, `transition : uint8` → monitor | 0 `configure`, 1 `activate`, 2 `deactivate`, 3 `cleanup`, 4 `shutdown`: events E1–E5. Any other value: no change |
| `roslean_cm_phase` | monitor → `uint8` | 0 unconfigured, 1 inactive, 2 active, 3 finalized. For logs and tests |

The monitor is built in `on_configure` from the parameters, as upstream
reads them there (N3), and `configure` is delivered to it. If
`roslean_cm_ok` is 0 the transition fails as upstream's does: the node stays
unconfigured and the process stays up (this replaces D14). "Fail closed"
stays as the backstop.

The shim delivers a lifecycle event from the corresponding `on_…` callback,
before it returns success. It does not keep an "active" flag of its own to
decide whether to publish: that decision is the output of `on_cmd`.
Lifecycle publishers, which drop messages while the node is inactive, may be
used: they can only suppress.

### 8.2 Scan and shape events

(T05b: rewritten for option A. The encodings are proposals; T06 may choose
others but not other meanings.)

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_scan` | monitor, `now_ns : uint64`, `stamp_ns : int64`, `range_min_bits : uint32`, `range_max_bits : uint32`, `ranges : ByteArray` → monitor |
| `roslean_cm_on_shape` | monitor, `zone : uint32`, `foreign : uint8`, `has_verts : uint8`, `verts : ByteArray` → monitor |
| `roslean_cm_shape_accepted` | monitor → `uint8`: the last `on_shape` call was accepted (E7) |

- `on_scan`: without the argument `usable`. `ranges` has 4 bytes per
  reading, the range pattern, little-endian; a remainder is ignored. No
  coordinates: they come with each command. The shim keeps the message
  (ranges, `angle_min`, `angle_increment`, `frame_id`, stamp) for the
  lookups and the points of the next commands (T2, T6).
- `on_shape`: `verts` has 16 bytes per vertex, the bit patterns of `x` and
  `y` of the vertex **in the base frame** as little-endian `double`s (N33,
  step 3: the `float` coordinates of the message under the transform, `z`
  taken as 0; for a message in the base frame the identity, which leaves
  the widened `float`s); a remainder is ignored. `has_verts` is 0 if the
  lookup failed: `verts = none`. `foreign` is 0 iff the message's
  `frame_id` equals `base_frame_id` (a string comparison, as upstream's).
  The lookup is N33 step 2, made by the shim before the call, and only if
  the message has at least three points (so that a short message is ignored
  without a wait, as upstream; the shim then still makes the call, with the
  short list, and Lean refuses it).
- If `shape_accepted` is 1 the shim stores the message for the lookups of
  later commands and for N16; otherwise it logs an error and keeps the
  previous one.

### 8.3 Command event

| Call | Arguments |
| --- | --- |
| `roslean_cm_cmd_plan` | monitor, `now_ns : uint64`, six `uint64` (the twist) → `uint8` |
| `roslean_cm_on_cmd` | monitor, `now_ns : uint64`, six `uint64`, `has_points : uint8`, `points : ByteArray`, `zone_verts : ByteArray` → monitor |
| `roslean_cm_out_kind` | monitor → `uint8` |
| `roslean_cm_out_zone`, `roslean_cm_out_ratio` | as today |
| `roslean_cm_out_report` | monitor → `uint8` |
| `roslean_cm_out_report_zone` | monitor → `uint32` |

1. The shim reads the clock once: `now_ns`.
2. `cmd_plan` says which lookups upstream would make for this command, so
   that the shim neither waits where upstream answers at once nor skips a
   lookup: 0 none (not active, command not finite, or source invalid before
   any lookup: no scan or scan too old with a non-zero timeout); 1 the
   topic zones' only (source disabled, or no scan with timeout 0); 3 the
   scan's and the zones'. It changes nothing.
3. Lookups (T6). For the scan: N23 or N24, with `now_ns` as the target
   time, with the timeout `transform_tolerance`, on a buffer whose listener
   has its own thread, so that the wait of N26 is upstream's. For every
   enabled topic zone whose stored message is in another frame: N34.
4. **Points (T2, new with option A).** If the scan's lookup succeeded: for
   **every** reading `i` of the stored message, valid or not, the point of
   R3 in `float`, widened, and its image by N25 in `double`; `points` gets
   16 bytes per reading, the patterns of `x` and `y` in the base frame,
   little-endian, in scan order, and `has_points` is 1. Otherwise
   `has_points` is 0. The shim does not test the range of a reading: which
   readings count is Lean's decision, and what it writes for a reading that
   does not count is not looked at (`output_filter_validBeam`). For every
   zone looked up in step 3: the vertices of the stored message by N25.
5. `on_cmd`. `scanPts` is `none` if `has_points` is 0, else the decoded
   pairs (a remainder of fewer than 16 bytes is ignored; a list that is not
   one pair per stored reading counts as no points, E8 step 3). `zone_verts`:
   for each zone in order a 4-byte little-endian count `n` and `16·n` bytes
   of vertices, with `n = 0xFFFFFFFF` for "none"; a zone the array does not
   reach has none.
6. `out_kind`: 0 `silent`; 1 to 4 as today (`answer (some a) _`); 5
   `answer none _` (processed, velocity withheld). `out_report`: 0 no
   report; 1 `normal`; 2 `stop`; 3 `slowdown`; 4 `source`; the zone index
   in `out_report_zone` for 2 and 3.
7. The shim then: publishes the state message if `out_report` is not 0 and
   `state_topic` is set (1 → `("", 0)`; 2 → `(<name>, 1)`; 3 → `(<name>,
   2)`; 4 → `("invalid source", 1)`), **before** the velocity (N13);
   publishes the velocity for kinds 1 to 4 as today; publishes the
   visualised polygons for kinds 1 to 5 (N16); for kind 0 nothing.

Theorems T06 owes for these calls (`ROSlean/CollisionMonitor/FFI.lean`): a
sequence of calls yields `run` of the corresponding events; the numbers
read back determine the model's output, including the report; if
`cmd_plan` is 0 or 1 the result of `on_cmd` does not depend on `points`,
and if it is 0 not on `zone_verts`; `cmd_plan` does not change the monitor;
decoding of `ranges`, `points`, `verts` and `zone_verts` is total and
inverts the shim's encoding. (The present interface has the corresponding
theorems for its one array: `decodeBeams_encodeBeams`,
`decodeCoords_encodeBeams`, `fits_decode`, `points_decode`.)

### 8.4 What stays in C++

- `transforms.hpp` (new, the only file that calls tf2 and the only one
  that computes with a transform; T05b: it now holds arithmetic). (a) The
  two lookups, the comparison of frame names, the check of ND7 (seven
  finite numbers, a quaternion that is not zero; otherwise "none"). (b)
  **N25**: the looked-up message into a `tf2::Transform` with tf2's own
  `fromMsg`, and the image of each point with tf2's own `operator*`, so
  that the operations and their order are upstream's by construction;
  `x` and `y` of the result, the third coordinate dropped. For a base-frame
  scan without correction the identity is applied all the same, as upstream
  does (N23). (c) Compiled with **`-ffp-contract=off`**, like the rest of
  the node: decided by the user. On a platform where upstream's build fuses
  (arm64) the last bits of a coordinate can therefore differ from
  upstream's (section 5, T7); the flag is not to be changed per platform.
  Tests: against a `tf2::BufferCore` filled by the test, that the lookups
  ask for N23 and N24 with the arguments in the right order (source and
  target frame, the two times) and that every kind of failure gives "none";
  for the arithmetic, that the points equal those of a reference
  computation written out operation by operation (N25) on exact and on
  general quaternions, bit for bit, and that the object code contains no
  fused multiply-add (the existing check of task T01, extended to this
  file).
- `conversions.hpp`: `encode_scan` without the sensor pose and without
  coordinates (ranges only); the scan-frame point of R3 (`cos`, `sin`, the
  `float` product) moves to where the points of a command are computed
  (T2).
- The polygons for `visualize` (N16): the fixed polygon's vertices as read,
  the 16 points of a circle, the last accepted message. T8.

### 8.5 Node plumbing

- The node derives from `nav2_util::LifecycleNode`, which gives the bond,
  `bond_heartbeat_period`, `autostart_node` and the pre-shutdown clean-up
  exactly as upstream has them (N9, N11). New build dependencies of the
  package: `nav2_util`, `nav2_msgs`, `rclcpp_lifecycle`, `tf2_ros`. If `nav2_util` proves too heavy a dependency, the
  same 30 lines over `rclcpp_lifecycle::LifecycleNode` and `bondcpp`.
- Subscriptions and publishers are created in `on_configure` and destroyed
  in `on_cleanup` with upstream's types and QoS (N3, N12, N16, N31). The TF
  listener has its own thread (N1).
- Parameters newly read: `state_topic`, `odom_frame_id`,
  `transform_tolerance` (converted as `tf2::durationFromSec` does, by
  calling it), `base_shift_correction` (default `true`, as upstream),
  `<zone>.polygon_sub_topic`, `<zone>.polygon_subscribe_transient_local`,
  `<zone>.visualize`, `<zone>.polygon_pub_topic`, `bond_heartbeat_period`.
  Read and ignored without refusal, because upstream does not read them
  either: `<zone>.footprint_topic`, `time_before_collision`,
  `simulation_time_step` on a zone that is not `approach` (with the message
  of ND11 where it applies), `<source>.min_height`, `max_height` on a scan
  source. `<source>.roslean_sensor_pose` is refused with a message that
  points to TF.
- **Composition.** T06 tries whether the node can be registered as an
  rclcpp component and loaded into `nav2_container` beside Nav2's own
  nodes (the Lean runtime is initialised once per process). If it can, the
  plugin name is `roslean_collision_monitor::CollisionMonitor`. If not,
  ND10 stands.

### 8.6 What a user changes to run it in Nav2

To be true once T06 is done; T10 demonstrates it.

1. **Parameters.** In the `collision_monitor` section, the zone must not be
   `approach`. Either the footprint as a fixed polygon, which is the
   recommended form (N34):

   ```yaml
   polygons: ["FootprintStop"]
   FootprintStop:
     type: "polygon"
     points: "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"
     action_type: "stop"
     min_points: 6
   ```

   or Nav2's zone with two lines changed:

   ```yaml
   FootprintApproach:
     type: "polygon"
     action_type: "stop"                                        # was "approach"
     polygon_sub_topic: "/local_costmap/published_footprint"    # new; footprint_topic is read for approach only
     footprint_topic: "/local_costmap/published_footprint"      # may stay, is not read
     time_before_collision: 1.2                                  # may stay, is not read
     simulation_time_step: 0.1                                   # may stay, is not read
     min_points: 6
     visualize: False
     enabled: True
   ```

   Upstream accepts the second form too [B: `footprint-sub-topic`].
   Everything else in the section stays, including `state_topic` and
   `base_shift_correction: True`. The package's own
   `collision_monitor_params.yaml` needs no change: its lists name a fixed
   stop polygon and a scan only (N21).
2. **Launch.** In a copy of `navigation_launch.py`: in the branch without
   composition, `package='roslean_collision_monitor'` in place of
   `package='nav2_collision_monitor'` (the executable is `collision_monitor`
   in both; `name`, parameters and remappings stay). With composition
   either start Nav2 with `use_composition:=False`, or remove the
   `ComposableNode` of the monitor from the container and add the `Node`
   action above next to it; or, if 8.5 succeeds, change package and plugin
   of the `ComposableNode`.
3. Nothing else: the node name, the lifecycle manager's list, the topics
   and the bond are as upstream's.

## 9. Edits to make to SPEC.md later

Not made by this task. T05 and T06 make them when the code exists.

1. **Scope paragraph and title of "Rules as implemented upstream"**: add
   the lifecycle, transforms and topic zones; refer here for N….
2. **R3**: the point is in the scan frame; the transform is N25.
3. **R8**: "its transform to the base frame cannot be obtained" now refers
   to N23, N24, N27.
4. **R17**: add the parameters of 8.5.
5. **R18, R19**: the node is a lifecycle node; drop "no lifecycle" wording;
   `~/toggle` and run-time `enabled` move to ND8.
6. **"Units and representation"**: (T05b) scan points are `binary64`
   patterns in the base frame, handed over with each command, and converted
   by `coordMm` as before; add topic-zone vertices (`vertexCoordMm`).
7. **"Safe direction"**: the scan-point paragraph and the polygon paragraph
   get the argument of section 5.
8. **T1–T5**: replaced by section 7.
9. **The model, Properties**: sections 3 and 4.
10. **Shim boundary**: section 8. `roslean_cm_on_scan` loses `usable`; the
    sensor pose disappears from the pseudo-code and from "What the shim
    still refuses". `Monitor.dropScan`, and `Event.scanUnusable` if T01 has
    introduced it, are removed: a scan is no longer usable or unusable when
    it arrives; whether it can be placed is decided per command (the command
    carries its points or none).
11. **Excluded features**: remove the rows "Motion correction",
    "Transforms", "State topic", "Zone visualisation" and "Lifecycle";
    "Dynamic polygon or radius" becomes "Dynamic radius" (ND12); "Footprint
    subscription" stays (`approach` only); add "Component" (ND10) and
    "Toggle and run-time parameters" (ND8); "Collision point markers" stays
    (ND9).
12. **D2**: replaced. The node is a lifecycle node with a bond; what
    remains is ND8 and ND4.
13. **D7**: add ND2.
14. **D11**: replaced. TF is used as upstream uses it, including base shift
    correction. `roslean_sensor_pose` is removed (refused with a message).
    The restriction to upright sensors goes; what upstream does with a
    tilted sensor (N25) is reproduced.
15. **D13**: the state topic and the polygons are provided; only the point
    markers are not (ND9).
16. **D14**: replaced by "as upstream: the transition fails".
17. **The frame check** (`header.frame_id == base_frame_id`): survives only
    as the string comparison that selects the identity (N23) and `foreign`
    (N33).
18. **A4**: becomes "the TF tree is right and timely (T6), and the sensor's
    clock agrees with the node's".
19. **Open decisions** 7 and 10: closed by this extension. Decision 7's
    last sentence (`Event.scanUnusable`) is overtaken by edit 10.
20. **"Note on testing against upstream"**: the restrictions
    `base_shift_correction: false`, scans in the base frame and an empty
    `state_topic` go.

## 10. Test plan

For T07 (upstream, in the harness) and T08 (the Lean node through the same
scenarios). "Upstream does" is what the named probe case showed on
2026-11-24 (macOS arm64, 1.3.12). The harness needs, beyond what it has: a
`/tf` publisher with chosen stamps; static transforms with arbitrary
quaternions; a `PolygonStamped` publisher; lifecycle transitions in the
middle of a scenario; recording of the polygon topics; and for every command
whose lookup fails, a clock advance before the answer can come (N26; the
harness has `late` for that).

| # | Scenario | Upstream does | Probe case |
| --- | --- | --- | --- |
| 1 | Command and scan while unconfigured | No subscription exists; nothing | `lifecycle` |
| 2 | Scan, command, non-finite command while inactive | Nothing published; the scan is used after activation | `lifecycle` |
| 3 | Activation | No velocity, no state; each visualised polygon once | `lifecycle`, `visualize` |
| 4 | Stop held beyond `stop_pub_timeout`, then deactivate and activate, same clock | Zero and the state are published again; zero for a further `stop_pub_timeout`, to the nanosecond | `lifecycle` |
| 5 | Cleanup, configure, activate | Stored scan gone: `invalid source`; a topic zone's shape gone | `lifecycle`, `polygon-topic-inactive` |
| 6 | Shutdown from active, then a command | Forwarded (N8). The Lean node: nothing (ND4) | `lifecycle` |
| 7 | State messages: pass first; slowdown; stop; zero command in a slowdown zone; invalid source; changes while the velocity is withheld; non-finite command | N13, N14 | `state` |
| 8 | Polygon topics: after activation, after a processed command, after a withheld one, after a non-finite one; circle and polygon; contents | N16 | `visualize` |
| 9 | Bond on, wall clock: messages on `/bond`; under a lifecycle manager; node killed | N9, N10 | `bond`, `manager` |
| 10 | Scan frame offset and rotated, exact transforms | SPEC scenarios `offset`, `rotated` (UPSTREAM_BEHAVIOUR.md 23) | not re-run here |
| 11 | No correction: transform changes after the scan; newest transform old; older transform arrives late | Newest is used, any age | `tf-latest` |
| 12 | Sensor upside down, raised, pitched, quarter turn, quaternion not normalised | N25 | `tf-tilt`, `tf-pitch-circle` |
| 13 | Missing transform: clock still; tolerance − 1 ns; tolerance; transform arriving during the wait; empty `frame_id` | N26 | `tf-missing` |
| 14 | Missing transform with `source_timeout: 0` and with `transform_tolerance: 0` | Passes at once; stops once the transform exists and a point is inside | `tf-missing-timeout0` |
| 15 | Correction: base moving straight between scan and command; half way between odom samples; correction off | N24 | `shift-straight` |
| 16 | Correction: base turning by half a turn | N24 | `shift-turn` |
| 17 | Correction with a sensor frame; scan older than the odom history | N24 | `shift-sensor-frame` |
| 18 | Correction: command newer than the newest odom sample; a newer sample arriving during the wait | Waits, then `invalid source`; or answers when the sample arrives | `shift-straight` |
| 19 | Correction, scan in the base frame, no odom frame | `invalid source` after the tolerance | `shift-no-odom` |
| 20 | Nav2's zone with only `action_type` changed, `stop` and `slowdown`, both parameter files | `configure` fails with the messages of N29 | `footprint-edit` |
| 21 | The same with `polygon_sub_topic` added: no message; footprint in odom; base moved, no new message; new message | N32, N34 | `footprint-sub-topic` |
| 22 | Fixed points with `footprint_topic` and `polygon_sub_topic` set | Neither is subscribed | `footprint-topic-ignored` |
| 23 | Topic zone: no shape; box; two points; none; triangle; unknown frame; other frame; empty frame; NaN; closed polygon; degenerate | N32, N33. The Lean node differs at NaN (ND6) | `polygon-topic` |
| 24 | Topic zone in a moving frame, `slowdown` | N34 | `polygon-topic-moving` |
| 25 | Shape while inactive; shape lost at cleanup; latched message with `polygon_subscribe_transient_local` | N31, N35 | `polygon-topic-inactive` |

### 10.1 Where each row is tested (added by T07)

The rows above are scenarios of
`ros/roslean_collision_monitor/test/scenario_nav2.py` (37 scenarios in six
families), written from runs of upstream's binary, not from the column
"Upstream does"; what they show is in UPSTREAM_BEHAVIOUR.md, findings 24 to
38. No row was contradicted; N28 is made more precise below. The node under test is an argument;
`test/profile_roslean_nav2.py` is the one place that states where the
ROSlean node is to answer differently (ND4, ND5, ND6, ND11). Every point of
a scenario is at least 0.125 m from every zone boundary, except in
`shift-interpolation`, which pins upstream's interpolation with points on a
boundary and is therefore not run for the ROSlean node (ND1).

| Rows | Scenarios |
| --- | --- |
| 1, 2, 3 | `lifecycle-states` |
| 4 | `lifecycle-reactivate` |
| 5 | `lifecycle-cleanup`, `polygon-topic-inactive` |
| 6 | `lifecycle-shutdown-active`; also `lifecycle-shutdown-inactive`, `lifecycle-shutdown-deactivated` |
| 7 | `state-topic`; also `state-name-collision` (N15 on the binary) |
| 8 | `polygon-pub`, `lifecycle-states`, `polygon-topic` (not: the point markers of N17) |
| 9 | `test/bond_manager.py` (wall clock) |
| 10 | `offset`, `rotated` of `scenario.py`, as before |
| 11 | `tf-latest` |
| 12 | `tf-tilt`, `tf-pitch-circle`; also `tf-bad-quaternion` (transforms tf2 refuses; ND7 is not reachable through them) |
| 13 | `tf-missing`; also `tf-tolerance-decimal`, `tf-tolerance-0.3` |
| 14 | `tf-missing-timeout0`, `tf-tolerance0`; also `tf-wait-timeout0` |
| 15 | `shift-straight`, `shift-off`; also `shift-interpolation` |
| 16 | `shift-turn`; also `shift-quarter-turn` |
| 17 | `shift-sensor-frame` |
| 18 | `shift-straight` |
| 19 | `shift-no-odom` |
| 20 | `footprint-edit-stop`, `footprint-edit-slowdown`, `footprint-edit-example-file`, `footprint-approach-unchanged` |
| 21 | `footprint-sub-topic`; also `footprint-sub-topic-slowdown` |
| 22 | `footprint-topic-ignored` |
| 23 | `polygon-topic` |
| 24 | `polygon-topic-moving` |
| 25 | `polygon-topic-inactive` |

Observed by these scenarios and stated above from the source only, or not
at all (all on 1.3.12, macOS arm64, and, except the scenarios that need
`nav2_bringup`'s parameter file, on 1.3.14 in a Linux arm64 container):

- N8: shut down from `active`, the node also still stores scans and
  publishes a stop with its state message; shut down from `inactive` it
  publishes nothing.
- N15: a zone named `invalid source` hides the change to an invalid source
  (`state-name-collision`).
- N23: a transform with a newer stamp is used by the next command at once.
- N24: after a quarter turn of the base to the left a point that was ahead
  is on the right (points 0.125 m from the zone's edge; not a boundary
  test).
- N26: the rounding of `transform_tolerance` to the nearest nanosecond:
  with 1.001 no answer at 1,000,999,999 ns and the stop at 1,001,000,000 ns
  (`source_timeout: 1.001` is 1,000,999,999 ns: `timeout-decimal` of
  `scenario.py`).
- N24: the interpolation between two odom samples decides a zone boundary
  exactly where the numbers are binary fractions (`shift-interpolation`: a
  quarter, a half and three quarters of the way between samples 1 m apart).
- N25: a transform published with a zero quaternion, a NaN in the
  quaternion or a NaN translation is not taken by tf2; the frame stays
  unknown (`tf-bad-quaternion`).
- N27: with `source_timeout: 0` a command whose lookup fails still waits
  for `transform_tolerance` before it passes.
- N28: a backward jump of the clock clears the transforms received on
  `/tf`; those received on `/tf_static` survive it (`tf-clock-jump-back`).
  A command that needs a cleared transform waits and is stopped for
  `invalid source` until the transform is published again.
- N29: the `slowdown` form of the edited footprint zone with
  `polygon_sub_topic` configures and slows to the default ratio 0.5.

The random test of the extension is `test/random_nav2.py`: transforms with
exact and general quaternions, a moving and turning base with odom samples
at every command's clock reading unless the tick is meant to fail,
lifecycle transitions and shape messages interleaved with scans and
commands, judged against a Python model; points within 5 mm of a zone
boundary, lookups that need interpolation and the wait of N26 are kept
out. It compares velocities and state messages, not polygon topics.

Not observed for this document, to be pinned by T07 if wanted (T07 pinned
the interpolation at a boundary, the quarter turn, coarsely, and the
backward clock jump; the others remain open): the exact
interpolation of N24 at a point where it decides a boundary; a quarter-turn
base rotation with correction; transforms with inexact quaternions at a zone
boundary (they belong to the 5 mm the comparison excludes); a clock that
jumps backwards (N28); the real local costmap as the publisher of N30; the
component form of the node (N18); `autostart_node` (N11); anything on
Linux.

For the random differential test: transforms drawn from exact quaternions
and from general ones, with the points kept 5 mm from every zone boundary
as now; odom samples on both sides of every command unless the case is
meant to fail; lifecycle transitions and shape messages interleaved with
scans and commands; and the Lean node compared with upstream on velocity,
state messages and polygon topics.

## 11. Implementation plan

**T05, model and proofs** (needs T01 merged and this document). Done, for
option B; **T05b** then moved the transform out of the model (option A):
items 1 and 3 below describe what T05 built, not what the model has now.
1. The data of 3.1 and the events of 3.2 in
   `core/ROSleanCore/CollisionMonitor.lean`; `Xform.pointMm` and
   `Xform.vertexMm` in a Mathlib-free file beside it, exact, with zero
   components handled without large shifts.
2. The properties of section 4 and their lifts to `run`.
3. The geometry lemmas for the margin argument with transformed vertices
   (the analogue of `nearBoundary_of_rounded`).
4. Examples: a missing transform with and without `source_timeout: 0`; a
   topic zone before and after its first shape; a shape replaced; a refused
   shape; deactivate and activate inside a stop; shutdown.
5. Hands to T06: the definitions of 3.1, and `Output` with `silent`
   distinct from `answer none none`.

**T06, C interface and node** (needs T03 and T05).
1. The calls of section 8 and their theorems.
2. The lifecycle node of 8.5; `transforms.hpp` of 8.4 with its tests.
3. The parameter handling of 8.5 and the refusal message of ND11.
4. `smoke_*` tests at node level for the rows of section 10 that need no
   upstream; the try of 8.5 "Composition".
5. A benchmark of a command with 360 and with 1000 readings through the
   node, against upstream (section 5 has only an estimate for option A).
6. The edits of section 9.

**T07, upstream scenarios** (needs T02 and this document; beside T05 and
T06).
1. The harness additions listed at the head of section 10.
2. The scenarios of the table, with upstream's outputs as expectations;
   `nav2_probe.py` shows each sequence of messages.
3. The Lean node registered as expected failure until T06 lands, with ND4,
   ND5 and ND6 as the named differences.

What each needs from the others: T06 needs T05's definitions and nothing
from T07. T07 needs nothing from T05 and T06; it needs this document's
table and the probes. T08 needs all three.

## 12. What still separates this from Nav2's default configuration

After pieces 1 to 3. For each feature: what it is in upstream's source, who
uses it in the two shipped parameter files (N21), and what closing the gap
would take. Sizes are rough estimates relative to all the collision monitor
work so far (model, proofs, interface, node), not measurements.

**The headline.** `nav2_bringup`'s `nav2_params.yaml` configures the
monitor with exactly one zone, and that zone is `approach`. After this
extension the Lean node runs in Nav2's bring-up with a **stop zone of the
footprint's shape** in its place (8.6). It still does not do what Nav2's
default does: slow the robot down in proportion to the time before a
predicted collision. The package's own example file, as shipped, is covered
completely: its lists name one fixed stop polygon and one scan.

| Feature | In upstream [S] | Used by `nav2_params.yaml` | Used by the package's example file | To close the gap |
| --- | --- | --- | --- | --- |
| `approach` action | `CollisionMonitor::processApproach`, `Polygon::getCollisionTime`: the robot's motion is simulated in steps of `simulation_time_step` up to `time_before_collision` (`src/kinematics.cpp`, `projectState`, `transformPoints`, with `cos` and `sin` of the turned angle); at each step the points inside the footprint are counted; the command is scaled by `collision_time / time_before_collision`. The footprint comes through `nav2_costmap_2d::FootprintSubscriber`, placed with the robot's pose at the footprint's stamp | **Yes: its only zone** | Defined, not in `polygons` | Model: the simulation loop over a rotating and translating point set, in exact arithmetic with `cos` and `sin` supplied by the shim (trusted) or enclosed in rational bounds (proved, a project of its own). To prove: the scale factor is in [0, 1]; no amplification; if the unscaled command reaches `min_points` points within the horizon, the published speed is at most the scaled one. Trusted: `cos`, `sin`, one division. The shim also needs the footprint subscriber's pose lookup. The largest single item: about half of the work so far |
| `limit` action | `processStopSlowdownLimit`: `ratio = min(1, linear_limit / hypot(vx, vy), angular_limit / |w|)`, clamped to [0, 1], applied to all three components | No | Defined (`PolygonLimit`), not in `polygons` | A new action kind. The ratio needs a square root and a division: computed by the shim and checked in Lean on bit patterns (trusted: the check's premises), or compared in squares exactly. To prove: no amplification; the published speeds respect both limits up to rounding. About a tenth |
| Velocity polygons | `src/velocity_polygon.cpp`: the zone's shape is the first sub-polygon whose ranges contain the command (`linear_min/max`, `theta_min/max`, and for `holonomic` a direction from `atan2`); if none matches, the previous shape stays | No | Defined (`VelocityPolygonStop`), not in `polygons` | Comparisons of `double`s, exact on bit patterns (a signed `F64.le`). The shape becomes a function of the command, so `evaluate` depends on the command (it does not today). The holonomic branch needs `hypot` and `atan2` from the shim. Non-holonomic: about a tenth |
| Dynamic polygons | `polygon_sub_topic` for a polygon | No (it uses `footprint_topic`, which only `approach` reads) | No | **Closed by this extension** (N29–N35) |
| Dynamic radius | `polygon_sub_topic` on a circle: `std_msgs/Float32` (N36) | No | No | One event; `radiusMm` exists. Small |
| Point cloud source | `src/pointcloud.cpp`: `PointCloud2` with `x`, `y`, `z` (and optionally `height`), `min_range` on the 3D length, the 3D transform, a height filter `min_height`…`max_height` on the transformed `z` or the `height` field | No (its `scan` entry carries `min_height` and `max_height`, which a scan never reads) | Defined (`pointcloud`), not in `observation_sources` | The shim's transform keeps the third coordinate; the height comparison in Lean on a third pattern per point; decoding of `PointCloud2` layouts in the shim (trusted). Clouds have 10⁵ points and more, handed over with every command (option A): the cost of the hand-over has to be measured first. About a fifth |
| Range source | `src/range.cpp`: one `sensor_msgs/Range` becomes an arc of points every `obstacles_angle`, with `cos` and `sin` in `float` | No | No | Like the scan path with a different generator of scan-frame points in the shim (T2). Small |
| Polygon source | `src/polygon_source.cpp`: `PolygonInstanceStamped` messages kept per id, dropped when too old, sampled along their edges every `sampling_distance` (`hypot`, `ceil`, a division) | No | No | The sampling in the shim (trusted) or in Lean with a rational square-root bound; a list of stored polygons with ids in the state. About a tenth |
| Costmap source | Not in Jazzy: `src/` has no such file | — | — | — |
| Several sources | `process`, first loop: every enabled source in order, the first invalid one stops; `<zone>.sources_names` selects which sources a zone counts (`Polygon::getPointsInside`) | No (one source) | No (one in the list) | `State.scan` and `sourcePoints` become lists; the theorems generalise without new ideas. About a tenth, mostly proofs |
| `~/toggle`, run-time `enabled` | `toggleCMServiceCallback`; `Polygon::dynamicParametersCallback`, `Source::dynamicParametersCallback` | No | No | Three events and a flag in the state (ND8). Small |
| Collision point markers | N17 | No | No | The shim has the points (option A); a publisher (ND9). Small, display only |
| Component | N18: Nav2's bring-up loads the monitor into a container by default (`bringup_launch.py`) | Bring-up, not parameters | — | 8.5 (ND10) |
| `action_type: none` | Accepted, does nothing (UPSTREAM_BEHAVIOUR.md 23) | No | No | Trivial; refused today |
| `max_points` | Deprecated alias (R11) | No | No | Refused (D10) |
| `use_realtime_priority` | `on_configure`: `nav2_util::setSoftRealTimePriority` | No | No | A call in the shim. Trivial |

What remains different by design even where a feature is covered: D3 and D4
(millimetres and the grown zones), D5 (negative ratios refused), D6, D8–D10,
D12, and ND1–ND12 above.

What is upstream's own weakness and is reproduced, not fixed, unless the
user decides otherwise (section 13 and PLAN.md): no output without commands
(R1); zero only for `stop_pub_timeout` (R15); `source_timeout: 0` switching
off the stop for a missing transform (N27); a topic zone without shape
(N32); a footprint zone left behind (N34).

## 13. Questions for the user

Each is implemented as upstream does it until decided, as PLAN.md says.

1. **A stop zone fed by a topic that has no shape yet** takes no action
   (N32, "No shape"). The fail-closed alternative: such a zone, if its
   action is `stop`, is triggered; or the source rule is extended to "a
   stop zone without shape stops, named after the zone". With Nav2 this
   would hold the robot until the local costmap has published its first
   footprint, which is before anything can move. It would be a numbered
   difference.
2. **A footprint zone that is left behind** (N34): accept as upstream;
   or give a shape in another frame a maximum age after which the zone
   counts as having no shape; or recommend the fixed polygon only and
   refuse `polygon_sub_topic` on the costmap's topic. Interacts with
   question 1.
3. **Option A or B** for the transform (section 5). Recommended: B.
   **Decided: A** (2026-11-24), with `-ffp-contract=off` on every platform.
4. **`source_timeout: 0` also disables the stop for a missing transform**
   (N27). Same question as SPEC.md, open decision 5.
5. **The wait for a transform inside the callback** (N26) holds up every
   command for up to `transform_tolerance` and, with base shift
   correction, for the time until the next odom sample. Reproduced as
   upstream does it. The alternative, a lookup without wait that stops at
   once, would stop the robot at every command in a normal Nav2 set-up with
   correction on, so it is not offered as a default.
6. **Shutdown** (ND4): the Lean node falls silent where upstream keeps
   forwarding. Assumed wanted.
7. **Composition** (ND10): is a stand-alone process acceptable if loading
   the Lean runtime into Nav2's container turns out not to work?
8. **Which form of the Nav2 edit to document** (8.6): the fixed polygon
   (recommended here), the two-line change of Nav2's zone, or both.
