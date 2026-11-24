# Collision monitor: specification of the Nav2 extension

Written by task T04. It specifies three additions to the node of
[SPEC.md](SPEC.md), so that Nav2's lifecycle manager can run it in place of
`nav2_collision_monitor`, still with the `stop` and `slowdown` actions only:

1. the lifecycle interface, the state topic and the polygon publishers;
2. transforms from TF, with base shift correction;
3. zones whose shape arrives on a topic (the route by which the robot's
   footprint becomes a zone).

Nothing here is implemented yet. Sections 1 and 2 say what upstream does;
sections 3 to 9 say what the Lean model, the C interface and the node are to
do; section 10 is the test plan, section 11 the plan for tasks T05 to T07,
section 12 what will still be missing afterwards.

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
  (`tf2_ros/src/buffer.cpp`, `Buffer::onTimeJump`) [S].

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

### 3.1 Data

- **`Xform`**: six binary64 bit patterns `qx qy qz qw ox oy`, the quaternion
  and the x and y of the translation of a looked-up transform. `oz` is not
  needed (N25).
  `Xform.usable`: all six are finite and `qx² + qy² + qz² + qw² ≠ 0` as
  exact numbers.
  `Xform.identity`: the patterns of (0, 0, 0, 1, 0, 0).
- **`Beam`** carries the range pattern and the **scan-frame** coordinates
  as two binary64 patterns, as the shim hands them over today, instead of a
  point in millimetres.
- **`Xform.pointMm t x y : Point`** (N25, ND1): for a usable `t` and finite
  `x`, `y`, with `n00 = d − 2(qy² + qz²)`, `n01 = 2(qx·qy − qw·qz)`,
  `n10 = 2(qx·qy + qw·qz)`, `n11 = d − 2(qx² + qz²)` as exact rationals:
  the point `((n00·x + n01·y)/d + ox, (n10·x + n11·y)/d + oy)` in metres,
  each coordinate times 1000, truncated toward zero and saturated at
  `±coordLimit`. If `x` or `y` is not finite: `(coordLimit, coordLimit)`,
  which lies in no accepted zone. **`Xform.vertexMm`**: the same with
  rounding to the nearest integer, a half away from zero, for zone vertices
  (as `vertexMm` today).
- **`Scan.points s t`**: the valid readings (R4) mapped by `t.pointMm`.
- **Zones.** A zone's shape is `fixed (s : Shape)` or `topic`. A topic zone
  has no shape in the configuration.
- **`TopicShape`**: `raw : List (Nat × Nat)` (the vertices as binary32
  patterns, as received), `foreign : Bool` (the message's frame is not the
  base frame), `current : List Point` (the vertices in the base frame, in
  millimetres: upstream's `poly_`).
- **`Report`**: `normal`, `stop (zone)`, `slowdown (zone)`, `source`: what a
  state message says (N13).
- **`Phase`**: `unconfigured`, `inactive`, `active`, `finalized`.
- **`State`** gains `phase` (initially `unconfigured`), `prevReport`
  (initially `normal`) and `shapes : List (Option TopicShape)`, one entry
  per zone, `none` for a fixed zone and for a topic zone without shape.
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
- **E7 `shape k verts place`**, with `place` either `base` or `foreign (tf
  : Option Xform)`. The message is **accepted** iff the phase is `inactive`
  or `active`, zone `k` exists and is a topic zone, `verts` has at least
  three entries, every pattern in `verts` is finite, `place` is `base` or
  `foreign (some t)` with `t.usable`, and every resulting vertex lies within
  the coordinate limit of `Shape.ok`. If accepted: `shapes[k] := some
  { raw := verts, foreign := (place ≠ base), current := verts.map (t.vertexMm) }`
  with `t` the identity for `base`. Otherwise no change. `silent` in both
  cases. (N33, ND6, ND7)
- **E8 `cmd now v scanTf zoneTfs`**, with `scanTf : Option Xform` (the
  transform for the stored scan, `none` if the lookup failed or was not
  made) and `zoneTfs : List (Option Xform)` (one entry per zone).
  1. Phase not `active`: no change, `silent`. (N4)
  2. `v` not finite: no change, `silent`. (R2)
  3. **Source** (`sourcePoints`, R6–R8 with N22 and N27): `some []` if the
     source is disabled; if no scan is stored, `some []` when the timeout is
     0 and `none` otherwise; if the scan is too old (timeout not 0), `none`;
     otherwise, if `scanTf = some t` with `t.usable`, `some (s.points t)`;
     otherwise `some []` when the timeout is 0 and `none` when it is not.
  4. **Shapes** (N34, ND3): if the source is `none`, unchanged. Otherwise,
     for every enabled topic zone `k` with `shapes[k] = some sh` and
     `sh.foreign`: if `zoneTfs[k] = some t`, `t.usable` and every vertex of
     `sh.raw.map t.vertexMm` is within the coordinate limit, `current` is
     replaced by that list; otherwise it stays.
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

`step` for a monitor that is not ok stays as it is ("Fail closed" in
SPEC.md): every command in the `active` phase is answered `stopSource`,
whatever else; in other phases `silent`.

### 3.3 What the model does not contain

The wait of N26 (the event carries only its outcome), frame names (the shim
resolves them into a transform or `none`), the bond, publishers and QoS,
the polygons shown for `visualize`, zone names (the shim maps a zone index
to its name).

## 4. Properties

To be proved by T05 about `step` and lifted to `run`, in the manner of
SPEC.md, "Properties". `cfg`, `st`, `now`, `v`, the transforms are arbitrary
unless restricted; no property needs `Config.ok` unless it says so. "The
output of the command" is `(step cfg st (.cmd now v scanTf zoneTfs)).2`.
*Source invalid* means: `cfg.sourceEnabled`, `0 < cfg.sourceTimeout`, and
either `st.scan = none`, or `st.scan = some s` with `s.stamp +
cfg.sourceTimeout < now`, or `scanTf` is `none` or not usable.

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
`st.phase = active`, `v.finite` and `scanTf` is `none` or not usable, then
`evaluate = stopSource` and the output is `answer (some stopSource) _` or
`answer none _`, the latter exactly in the case of **Velocity**. It is
**false without the two hypotheses on the configuration**, as "Stale data"
is: with `source_timeout: 0` a missing transform means "no obstacle" (N27;
example to be added).

**Stale data, Stop, Bad data, No amplification, Slowdown bound,
Pass-through, Monotonicity** of SPEC.md hold with these changes: the
hypothesis `st.phase = active` is added; "the output is `publish a`"
becomes "`answer (some a) _`" and "`silent`" in their conclusions becomes
"`answer none _`"; the points are `s.points t` for the `scanTf = some t`
of the command. Stale data keeps its conclusion whatever `scanTf` is.
Pass-through needs a usable `scanTf` among its hypotheses when a scan is
stored (it is part of "`sourcePoints = some pts`").

**Transform.** For usable `t` and finite `x`, `y`, writing `(X, Y)` for the
exact image in millimetres: `|(t.pointMm x y).x| ≤ |X|` and `|X| −
|(t.pointMm x y).x| < 1` unless saturated, and the same for `y`
(truncation toward zero, as `coordMm_abs_le` and `coordMm_abs_gt` today);
`|(t.vertexMm x y).x − X| ≤ 1/2`, likewise `y`; and
`Xform.identity.pointMm x y = (coordMm x, coordMm y)`, so that a scan in the
base frame without correction is treated exactly as today. For a quaternion
of unit length the exact image is the rotation it denotes followed by the
translation; for any usable quaternion it is that of its normalisation.

**Shape accepted (iff).** A `shape k verts place` event changes the state
iff it is accepted (E7); then `shapes[k]` is as E7 says and nothing else
changes: not the other entries of `shapes`, not `scan`, `prevZero`,
`stopStamp`, `prevReport`, `phase`.

**No shape.** If zone `k` is a topic zone and `st.shapes[k] = none`, zone
`k` is not triggered by any points, and no output for a command names it.
On runs from `State.init`: no output names a topic zone `k` before an
accepted `shape k` event that follows the last `cleanup`. This is upstream's
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

**After activation.** If `step` on `activate` moves the state to `active`,
then for the next finite command: if the velocity is zero it **is
published** (`vel = some _`), and the report is `some r` unless `r =
normal`. (N6: a stop is never withheld right after an activation.)

**Totality.** As before; the new conversions (`Xform.pointMm`, decoding of
transforms and vertices from byte arrays of any length) are total.

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
with the command; Lean computes `Xform.pointMm` exactly.

| | A | B |
| --- | --- | --- |
| Trusted, beyond tf2 and R3 | The matrix and the two sums of N25 in C++, about 15 lines, **and how they are compiled** | Nothing more |
| Agreement with upstream before the millimetre | Bit for bit only if the shim's sums are fused exactly where upstream's are. Upstream's arm64 binary fuses them (N25); a baseline x86-64 build cannot. The flag `-ffp-contract=off` the node is built with today would be wrong on arm64. The same kind of defect as CM3/CM4 finding 5 | Not bit for bit. Lean's point is the exact image; upstream's is that image computed in `double` |
| Model | The command event carries the point list; the stored scan no longer determines the points. "The points belong to the stored scan" is trusted | The command event carries six numbers or `none`; points are a function of the stored scan and the transform, inside the theorems |
| Cost per reading and command | The shim's arithmetic, a few ns (estimated), plus the decoding and conversion of today, 41 ns, now per command instead of per scan | Measured below: 1.2 to 1.9 µs exact; 0.45 µs in a fixed-point form |

**Cost of B, measured.** `review/collision_monitor/nav2/TransformCost.lean`
is a Mathlib-free prototype of `Xform.pointMm` on unbounded integers,
compiled with `leanc -O3`; `run_transform_cost.sh` runs it; output in
`results/macos-arm64-transform.txt`. On an Apple M-series core, while
another job used the machine, over eight runs: **1.2 to 1.5 µs per point**
for a quaternion with four non-zero components, **1.6 to 1.9 µs** for a
pure yaw (the zero components make the prototype align numbers of very
different exponents; a real implementation treats zero separately). That is
0.4 to 0.7 ms for a scan of 360 readings and 1.2 to 1.9 ms for 1000
readings, **per command**; at the 20 Hz of `controller_frequency` in
`nav2_params.yaml`, 1000 readings cost under 4 % of one core. Today's
figure for comparison: 41 ns per reading, once per scan. One of the eight
runs reported 177 µs per point in its first loop and normal figures in the
three others; it was not reproduced and is unexplained (that run sat in the
background for three minutes).

A fixed-point variant in the same file (matrix rounded once per command to
30 fractional bits, coordinates in units of 2⁻²⁰ m) costs **0.43 to 0.50 µs
per point**, still on unbounded integers, and differed from the exact result by
at most 1 mm on the 2000 test points, as two truncations of nearly equal
numbers do. With 64-bit machine integers it would be far cheaper, at the
price of overflow proofs. It is an optimisation T05 or T06 may add **only**
with a proof that it stays within the bound below; the specification is the
exact `Xform.pointMm`.

**How far B's points are from upstream's.** Let `e` be the exact image of a
scan-frame point, `u` the point upstream computes for it, `m` the model's
point.
- `|m − e| < 1 mm` on each axis, toward the origin (**Transform**, proved in
  Lean): less than √2 mm ≈ 1.4143 mm in distance.
- `|u − e|` is the rounding error of about fourteen `double` operations on
  numbers below 1000 m: of the order of 10⁻⁹ mm, and below 10⁻⁶ mm with a
  wide allowance, with or without fused multiply-add. This is an estimate
  from the error bounds of IEEE-754 operations; it is not proved in Lean
  and would need a model of binary64 arithmetic (trusted item T7).

**Does the polygon margin of √5 mm still guarantee "a point upstream counts
is always counted"?** Yes, with the same argument and the same gap as
SPEC.md ("Safe direction"). A vertex moves by at most √2/2 mm ≈ 0.7072 mm
when rounded (for a topic zone in another frame: from its exact image,
which upstream's vertex approximates within 10⁻⁶ mm); a point moves by less
than 1.4143 mm from `e`, hence by less than 1.4143 mm + 10⁻⁶ mm from `u`.
The sum is below 2.1216 mm < √5 mm ≈ 2.2360 mm. The budget left over is
0.114 mm: any implementation whose point is within 0.1 mm of
`Xform.pointMm`'s exact image before truncation keeps the guarantee. For
circles nothing changes: truncation moves a point toward the origin and the
radius is rounded up with 1 mm to spare. What is proved and what is prose
stays as THEOREMS.md says: the quantitative half in Lean, the step "the
boundary passed over the point" in prose, and now also the estimate of
`|u − e|`.

**Base shift correction.** The same in both options: it is one transform,
composed inside tf2 (N24), and reaches the shim as the same seven numbers.
Its precision is tf2's: two interpolations, one product of transforms and
one conversion from matrix to quaternion, in `double`, none of it in the
shim under either option. Its order of operations is N24 followed by N25.

**Recommendation: B.** It removes floating-point arithmetic from the shim
instead of adding some; it makes the points a function of what the model
stores, so the theorems about scans keep their meaning; it does not depend
on compiler flags; it serves zone vertices in another frame with the same
function; and it also lifts the restriction of D11 to upright sensors,
because the projection is computed from the full quaternion. Its price is
about a millisecond per command for a large scan, and that agreement with
upstream is "within 10⁻⁶ mm before truncation" instead of "bit for bit on
platforms where the flags happen to match". The differential tests already
exclude points within 5 mm of a boundary (SPEC.md, "Note on testing"), so
they lose nothing.

The decision is the user's (PLAN.md, "Decisions"). Until it is taken, T05
and T06 follow B.

## 6. Differences from upstream

In addition to D1–D14 of SPEC.md, which stay unless section 9 changes them.

| # | Upstream | ROSlean | Why |
| --- | --- | --- | --- |
| ND1 | The transform is applied in `double`, with or without fused multiply-add depending on the build (N25) | Applied exactly to the numbers the patterns denote, then truncated to millimetres (`Xform.pointMm`) | Section 5. Before truncation the two differ by an estimated 10⁻⁶ mm at most; after it, as D3 and D4 already say. |
| ND2 | The clock is read at the start of `process` and again in `publishVelocity`, after any wait for a transform (N28) | One reading per command, taken before the lookups | Extends D7. The two readings can now differ by up to `transform_tolerance` per lookup that waits, instead of by the run time of the callback. Affects only the boundary of `stop_pub_timeout`, and only for commands that waited. |
| ND3 | The transform of a topic zone's shape in another frame is looked up when the zone loop reaches the zone (N34) | The shim looks up the transforms of all such zones before the command is evaluated, and the model refreshes all enabled ones | The lookups cannot be interleaved with a decision taken in Lean in one call. Differences: lookups, and their waits, for zones behind a triggered stop zone; and the vertices of such a zone are refreshed where upstream leaves them until the zone is next reached. No command is decided differently unless a later lookup for that zone fails. |
| ND4 | Shut down from the active state, the node keeps answering commands (N8) | After `shutdown` nothing is published (E5) | A finalized node that drives the robot is a defect. Nav2's manager never takes that path (N8). |
| ND5 | A state message is published when the acting zone's **name** changes (N13, N15) | When the report changes, which identifies a zone by its position. A configuration in which two zones have the same name, or a zone is named `invalid source` or has an empty name, is refused | Only then can the two rules differ. |
| ND6 | A shape message is accepted whatever its coordinates; a NaN blinds the zone (N33) | A message with a non-finite coordinate, or a vertex beyond the coordinate limit of D12, is ignored and the previous shape stays (E7) | Integers cannot hold it, and the previous shape protects more than a blind one. If there is no previous shape the zone stays without one (question 1). |
| ND7 | tf2 does not return a non-finite or all-zero quaternion, by N25 [S] | Such a transform counts as a failed lookup (E8 step 3, E7) | Totality; fail closed. Not reachable through tf2 as read. |
| ND8 | `~/toggle`; `enabled` of a zone or source changeable at run time (R19) | Not provided; an attempt to change a parameter at run time is refused | Scope. Was part of D2. |
| ND9 | `~/collision_points_marker` (N17) | Not provided | Display only; under option B the base-frame points exist only inside Lean. |
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
- **T2. The scan-frame points.** The computation of R3 in `float`: the
  angle accumulation, `cos`, `sin`, the product, widened to `double`. The
  sensor pose arithmetic of D11 is gone. Everything after it (transform,
  millimetres, saturation) is in Lean.
- **T3. The configuration.** As before, plus that the shim tells Lean
  truthfully which zones are topic zones.
- **T4. The bit-pattern definitions** and the formula for the number a
  pattern denotes, now also the basis of `Xform.pointMm`.
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
  the shape message Lean last accepted for that zone; that it passes the
  seven numbers unaltered; and that a failed lookup is passed as "none".
  This code sits in one file with its own tests (section 8.4).
- **T7. Upstream's rounding (new, for the comparison only).** That
  upstream's own computation of N25 is within 10⁻⁶ mm of the exact image
  for coordinates below 1000 m. It enters no theorem about this node; it is
  what "a point upstream counts is always counted" rests on (section 5).
- **T8. Display (new).** The names in state messages (the shim's table from
  zone index to name) and the polygons published for `visualize`. A mistake
  here misinforms an observer; it changes no velocity.

What the theorems will say: for every sequence of lifecycle transitions,
scans, shape messages and commands, with whatever transforms tf2 returned
or failed to return, which velocity action and which report the node
gives, including that it gives none unless active, that a missing transform
stops under the stated hypotheses, and that every point is the exact image
of the scan-frame point under the transform it was given, truncated toward
the origin.

What they will not say: that the transform is the right one (T6); that the
scan-frame point is the right one (T2); anything about time spent waiting,
about the order in which the middleware delivers messages, about the bond
or the lifecycle manager; that a topic zone has a shape, or a current one
(N32, N34); that the base stops when the node is silent (A3).

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

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_scan` | monitor, `now_ns : uint64`, `stamp_ns : int64`, `range_min_bits : uint32`, `range_max_bits : uint32`, `beams : ByteArray` → monitor |
| `roslean_cm_on_shape` | monitor, `zone : uint32`, `foreign : uint8`, `tf : ByteArray`, `verts : ByteArray` → monitor |
| `roslean_cm_shape_accepted` | monitor → `uint8`: the last `on_shape` call was accepted (E7) |

- `on_scan`: as today without the argument `usable`. `beams` keeps its
  20-byte records; bytes 4–19 are now the coordinates **in the scan
  frame** (R3, T2), whatever the frame. The shim keeps the `frame_id` and
  the stamp of the message for the lookup at the next commands (T6).
- `on_shape`: `verts` has 8 bytes per vertex, the bit patterns of `x` and
  `y` of the `Point32`, little-endian; a remainder is ignored. `foreign` is
  0 iff the message's `frame_id` equals `base_frame_id` (a string
  comparison, as upstream's). `tf` is read only if `foreign` is not 0: 48
  bytes, the six patterns `qx qy qz qw ox oy` as little-endian `double`s, or
  any other length for "the lookup failed". The lookup is N33 step 2, made
  by the shim before the call, and only if the message has at least three
  points (so that a short message is ignored without a wait, as upstream).
- If `shape_accepted` is 1 the shim stores the message for the lookups of
  later commands and for N16; otherwise it logs an error and keeps the
  previous one.

### 8.3 Command event

| Call | Arguments |
| --- | --- |
| `roslean_cm_cmd_plan` | monitor, `now_ns : uint64`, six `uint64` (the twist) → `uint8` |
| `roslean_cm_on_cmd` | monitor, `now_ns : uint64`, six `uint64`, `scan_tf : ByteArray`, `zone_tfs : ByteArray` → monitor |
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
4. `on_cmd`. `scan_tf`: 48 bytes as in 8.2, or another length for "failed or
   not made". `zone_tfs`: 49 bytes per zone, in zone order: a flag byte (1:
   the 48 bytes that follow are a transform) and 48 bytes; any other total
   length means "none for every zone".
5. `out_kind`: 0 `silent`; 1 to 4 as today (`answer (some a) _`); 5
   `answer none _` (processed, velocity withheld). `out_report`: 0 no
   report; 1 `normal`; 2 `stop`; 3 `slowdown`; 4 `source`; the zone index
   in `out_report_zone` for 2 and 3.
6. The shim then: publishes the state message if `out_report` is not 0 and
   `state_topic` is set (1 → `("", 0)`; 2 → `(<name>, 1)`; 3 → `(<name>,
   2)`; 4 → `("invalid source", 1)`), **before** the velocity (N13);
   publishes the velocity for kinds 1 to 4 as today; publishes the
   visualised polygons for kinds 1 to 5 (N16); for kind 0 nothing.

Theorems T06 owes for these calls (`ROSlean/CollisionMonitor/FFI.lean`): a
sequence of calls yields `run` of the corresponding events; the numbers
read back determine the model's output, including the report; if
`cmd_plan` is 0 or 1 the result of `on_cmd` does not depend on `scan_tf`,
and if it is 0 not on `zone_tfs`; `cmd_plan` does not change the monitor;
decoding of `tf`, `zone_tfs` and `verts` is total and inverts the shim's
encoding.

### 8.4 What stays in C++

- `transforms.hpp` (new, the only file that calls tf2): the two lookups,
  the copy of seven numbers into bytes, the comparison of frame names. No
  arithmetic on the numbers. Tests: against a `tf2::BufferCore` filled by
  the test, that the bytes are tf2's numbers for N23 and N24, that every
  kind of failure gives "none", that the arguments are in the right order
  (source and target frame, the two times).
- `conversions.hpp`: `encode_scan` without the sensor pose. `cos`, `sin`,
  the `float` product (T2).
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
6. **"Units and representation"**: scan points are `binary64` patterns in
   the scan frame until a command; `Xform.pointMm` replaces `coordMm` on the
   scan path (it equals it for the identity); add topic-zone vertices
   (`Xform.vertexMm`).
7. **"Safe direction"**: the scan-point paragraph and the polygon paragraph
   get the argument of section 5.
8. **T1–T5**: replaced by section 7.
9. **The model, Properties**: sections 3 and 4.
10. **Shim boundary**: section 8. `roslean_cm_on_scan` loses `usable`; the
    sensor pose disappears from the pseudo-code and from "What the shim
    still refuses". `Monitor.dropScan`, and `Event.scanUnusable` if T01 has
    introduced it, are removed: a scan is no longer usable or unusable when
    it arrives; whether it can be placed is decided per command by `scanTf`.
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

Not observed for this document, to be pinned by T07 if wanted: the exact
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

**T05, model and proofs** (needs T01 merged and this document).
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
5. A benchmark of a command with 360 and with 1000 readings against the
   figures of section 5; the fixed-point form only if needed and proved.
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
| Point cloud source | `src/pointcloud.cpp`: `PointCloud2` with `x`, `y`, `z` (and optionally `height`), `min_range` on the 3D length, the 3D transform, a height filter `min_height`…`max_height` on the transformed `z` or the `height` field | No (its `scan` entry carries `min_height` and `max_height`, which a scan never reads) | Defined (`pointcloud`), not in `observation_sources` | The third row of the matrix in `Xform`; the height comparison in Lean; decoding of `PointCloud2` layouts in the shim (trusted). Clouds have 10⁵ points and more: at the costs of section 5 the exact transform is too slow, so the fixed-point form on machine integers with its overflow proofs is needed first. About a fifth |
| Range source | `src/range.cpp`: one `sensor_msgs/Range` becomes an arc of points every `obstacles_angle`, with `cos` and `sin` in `float` | No | No | Like the scan path with a different generator of scan-frame points in the shim (T2). Small |
| Polygon source | `src/polygon_source.cpp`: `PolygonInstanceStamped` messages kept per id, dropped when too old, sampled along their edges every `sampling_distance` (`hypot`, `ceil`, a division) | No | No | The sampling in the shim (trusted) or in Lean with a rational square-root bound; a list of stored polygons with ids in the state. About a tenth |
| Costmap source | Not in Jazzy: `src/` has no such file | — | — | — |
| Several sources | `process`, first loop: every enabled source in order, the first invalid one stops; `<zone>.sources_names` selects which sources a zone counts (`Polygon::getPointsInside`) | No (one source) | No (one in the list) | `State.scan` and `sourcePoints` become lists; the theorems generalise without new ideas. About a tenth, mostly proofs |
| `~/toggle`, run-time `enabled` | `toggleCMServiceCallback`; `Polygon::dynamicParametersCallback`, `Source::dynamicParametersCallback` | No | No | Three events and a flag in the state (ND8). Small |
| Collision point markers | N17 | No | No | An interface call that returns the model's points (ND9). Small, display only |
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
