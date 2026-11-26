# Collision monitor core: specification

This specification covers one laser scan source, one velocity input,
circle and polygon zones, and the `stop` and `slowdown` actions. It also
covers the Nav2 lifecycle and bond, state and polygon topics, TF, base
shift correction, and topic-fed polygons.

The upstream reference is Nav2 Jazzy commit `c704655` (1.3.14). The
cited files are identical in 1.3.12 and 1.3.13, the versions pinned by
`pixi.lock`. Paths start in `nav2_collision_monitor/` unless shown
otherwise. The Lean model, C++ node, and tests cite the rule and
difference numbers below.

For design decisions and Nav2 setup, see [SPEC_NAV2.md](SPEC_NAV2.md).
For changes after review, see [REVIEWS.md](../REVIEWS.md). C++ applies the
sensor transform in `double` in `transforms.hpp`. Lean receives the
transformed points.

## Rules as implemented upstream

**Triggers**

- R1. The only thing that makes the node compute or publish is a message on
  the velocity input (`CollisionMonitor::cmdVelInCallback*` →
  `process`, `src/collision_monitor_node.cpp`). A scan message is only stored
  (`Scan::dataCallback`, `src/scan.cpp`). There is no timer. When commands
  cease, nothing is published, whatever the scan shows; when the scan goes
  stale, nothing is published until the next command arrives.
- R2. A command in which any of the six components is infinite or NaN is
  ignored: an error is logged, nothing is published and no state changes
  (`cmdVelInCallbackStamped`; `validateTwist`,
  `nav2_util/src/robot_utils.cpp`). The check covers `linear.z`, `angular.x`
  and `angular.y` although the node uses only `linear.x`, `linear.y` and
  `angular.z`.

**Scan data**

- R3. The points of a scan are computed when a command is processed
  (`Scan::getData`). With `float angle = angle_min`, for each index `i` of
  `ranges` in order: if reading `i` is valid (R4), the point is
  `(ranges[i] * cos(angle), ranges[i] * sin(angle))`, computed in single
  precision, in the scan's frame, and then transformed to the base frame by
  the transform of N23 to N25, looked up for this command; in
  every case
  `angle += angle_increment` follows, also in single precision. The number of
  readings is `ranges.size()`. `angle_max`, `intensities`, `time_increment`
  and `scan_time` are not read, and the length of `ranges` is never compared
  with the angle fields.
- R4. Reading `i` is valid iff `ranges[i] >= range_min && ranges[i] <=
  range_max`, as `float` comparisons (`Scan::getData`). Therefore: a NaN
  reading is invalid; a reading of `+inf` is valid only if `range_max` is
  `+inf`; if `range_min` or `range_max` is NaN no reading is valid; a reading
  of `0` is valid if `range_min <= 0`. Invalid readings are skipped one by
  one. No property of the scan as a whole is checked: an empty scan, or one
  with no valid reading, is a scan with no obstacles.
- R5. The node keeps the scan received last (`data_ = msg`). It is not
  the one with the newest stamp: a scan that arrives late replaces a newer
  one.
- R6. A source has a parameter `enabled`. A disabled source is skipped: it
  contributes no points and cannot cause a stop (`process`, first loop).
- R7. A stored scan is *too old* at node-clock time `now` iff the source
  timeout is not zero and `now − header.stamp > source_timeout`, strictly, in
  integer nanoseconds (`Source::sourceValid`, `src/source.cpp`). The age is
  measured from the stamp in the message header, not from the arrival time.
  A scan stamped in the future is never too old. The timeout is converted
  from seconds by `rclcpp::Duration::from_seconds`, which multiplies by 10⁹
  and truncates (`rclcpp/src/rclcpp/duration.cpp`), so a positive timeout
  below 1 ns is zero.
- R8. If the source is enabled and its timeout is not zero, and either no
  scan has been received yet, or the stored scan is too old, or its transform
  to the base frame cannot be obtained (N23, N24, N26, N27),
  the action is *stop* with the name
  `invalid source`, and no zone is evaluated (`process`, first loop). If the
  timeout is zero, the same conditions only mean that the source contributes
  no points.

**Zones**

- R9. Circle (`Circle::getPointsInside`, `src/circle.cpp`): a point is inside
  iff `x² + y² < radius²`, strictly, around the base frame's origin. The
  radius is the parameter `radius`; its sign is lost by the squaring.
- R10. Polygon (`Polygon::isPointInside`, `src/polygon.cpp`): ray crossing.
  With the vertices in order and each edge running from the previous vertex
  `a` to the vertex `b` (the first edge from the last vertex to the first),
  an edge is crossed iff `(p.y <= a.y) == (p.y > b.y)` and
  `a.x + (p.y − a.y)·(b.x − a.x)/(b.y − a.y) > p.x`. The point is inside iff
  the number of crossed edges is odd. Consequences: horizontal edges are never
  crossed; points on the boundary are inside on some sides and outside on
  others (for the axis-parallel box of upstream's example, inside on the sides
  with the largest `y` and the smallest `x`, outside on the other two).
  The vertices come from the string parameter `points`, parsed as `float`
  (`nav2_util/src/array_parser.cpp`) and widened to `double`. A polygon with
  fewer than **four** vertices is refused (`Polygon::getPolygonFromString`:
  `vvf.size() <= 3`), although the log message says "at least three".
- R11. A zone has parameters `enabled` (default `true`) and `min_points`
  (default 4). A disabled zone is skipped. An enabled zone *triggers* iff the
  number of points inside is at least `min_points`
  (`processStopSlowdownLimit`: `>=`). With `min_points <= 0` a zone triggers
  without any point. The deprecated parameter `max_points`, if set, replaces
  `min_points` by `max_points + 1` (`Polygon::getCommonParameters`).

**Action**

- R12. Zones are visited in the order of the `polygons` parameter. A
  triggered `stop` zone sets the action to *stop* with that zone's name, and
  the loop ends before the next zone (`process`, second loop). So the first
  triggered stop zone wins over everything, including slowdown zones before
  it.
- R13. A triggered `slowdown` zone proposes the velocity `command ×
  slowdown_ratio`: the ratio is always applied to the incoming command, never
  to an already reduced velocity. The proposal replaces the current one iff
  its squared norm `x² + y² + tw²` is strictly smaller (`Velocity::operator<`,
  `include/nav2_collision_monitor/types.hpp`). Starting from the command
  itself, the result is: the triggered slowdown zone with the smallest
  `|slowdown_ratio|` wins, if that is below 1; of two equal ratios, the
  earlier zone; a ratio of 1 or more, infinite or NaN never takes effect.
  `slowdown_ratio` (default 0.5) is not range-checked: a negative ratio above
  −1 takes effect and **reverses** the command.
- R14. If no zone changes the action, the velocity is the command.
- R15. `publishVelocity`: let *zero* mean that the three components of the
  velocity to publish are all `== 0.0`. If the velocity is zero and the
  previous one (initially non-zero) was not, the current time is stored as
  the stop stamp (initially 0) and the velocity is published. If both are
  zero, it is published iff `now − stop_stamp > stop_pub_timeout` is false.
  A non-zero velocity is always published. The previous velocity is updated
  on every processed command, published or not. So zero is published for
  `stop_pub_timeout` (default 1 s) after it first appears, and then nothing
  is published until a non-zero velocity occurs. This applies in the same way
  to a stop demanded by a zone, to a stop for an invalid source, and to a
  command that is itself zero.
- R16. The published message is new: `linear.x`, `linear.y` and `angular.z`
  are the computed velocity, and `linear.z`, `angular.x` and `angular.y` are
  0. With `enable_stamped_cmd_vel` the header of the command is copied.

**Interface**

- R17. Parameters of the subset, with upstream's defaults:

  | Parameter | Type | Default |
  | --- | --- | --- |
  | `cmd_vel_in_topic` | string | `cmd_vel_smoothed` |
  | `cmd_vel_out_topic` | string | `cmd_vel` |
  | `enable_stamped_cmd_vel` | bool | `false` |
  | `source_timeout` | double, s | 2.0 |
  | `stop_pub_timeout` | double, s | 1.0 |
  | `observation_sources` | string array | none: required |
  | `<source>.type` | string | `scan` |
  | `<source>.topic` | string | `scan` |
  | `<source>.enabled` | bool | `true` |
  | `<source>.source_timeout` | double, s | the node's `source_timeout`, converted twice (see below) |
  | `polygons` | string array | none: required |
  | `<zone>.type` | string | none: required (`circle`, `polygon`) |
  | `<zone>.radius` | double, m | circle: required |
  | `<zone>.points` | string `"[[x, y], …]"`, m | polygon: required |
  | `<zone>.action_type` | string | none: required (`stop`, `slowdown`) |
  | `<zone>.min_points` | integer | 4 |
  | `<zone>.slowdown_ratio` | double | 0.5 |
  | `<zone>.enabled` | bool | `true` |
  | `<zone>.sources_names` | string array | all of `observation_sources` |

  A parameter of the wrong YAML type makes configuration fail.

  With the Nav2 extension the node also reads (SPEC_NAV2.md, 8.5):
  `base_frame_id` (`base_footprint`), `odom_frame_id` (`odom`),
  `transform_tolerance` (0.1 s; converted by `tf2::durationFromSec`),
  `base_shift_correction` (`true`), `state_topic` (empty: none),
  `<zone>.polygon_sub_topic` (required for a polygon without `points`),
  `<zone>.polygon_subscribe_transient_local` (`false`), `<zone>.visualize`
  (`false`), `<zone>.polygon_pub_topic` (the zone's name), and, through
  `nav2_util::LifecycleNode`, `bond_heartbeat_period` (0.1 s) and
  `autostart_node` (`false`).

  The default of `<source>.source_timeout` is not the node's parameter value
  but the node's timeout after conversion to nanoseconds, read back as
  seconds (`Source::getCommonParameters`: `source_timeout_.seconds()`), and
  it is then converted to nanoseconds again. `Duration::seconds()` is
  `(double)ns / 1e9`. The second conversion loses 1 ns for 398 of the 30,001
  values `k/1000` s up to 30 s: `source_timeout: 2.094` is 2,093,999,999 ns
  for the node and 2,093,999,998 ns for a source that does not set its own.
  The Lean node reproduces this (`nsViaSeconds`), so that R7 has the same
  boundary; `smoke_timeouts.cpp` compares it with `rclcpp::Duration`.
- R18. Topics: the command input is `Twist`, or `TwistStamped` with
  `enable_stamped_cmd_vel`, QoS keep-last 1; the output has the same type and
  QoS (`nav2_util/twist_subscriber.hpp`, `twist_publisher.hpp`). The scan
  subscription uses `SensorDataQoS`. The node's name is `collision_monitor`.
  The state topic, the polygon topics and the subscription of a zone fed by
  a topic: N12, N16, N31.
- R19. Upstream is a lifecycle node, and since task T06 so is this node:
  the transitions are N2 to N10. While it is not active,
  commands are dropped without output. It also has a service `~/toggle` that switches all
  zones off (the invalid-source stop stays), and `enabled` of a zone or source
  can be changed at run time (not provided: ND8). Callbacks run one at a time
  (`rclcpp::spin`).

## Rules of the Nav2 extension as implemented upstream

Three additions to the subset above, so that Nav2's lifecycle manager can
run the node in place of `nav2_collision_monitor`: the lifecycle interface,
the state topic and the polygon publishers; transforms from TF, with base
shift correction; and zones whose shape arrives on a topic (the route by
which the robot's footprint becomes a zone). The rules were written by task
T04 in [SPEC_NAV2.md](SPEC_NAV2.md) and moved here by task T09, with the
corrections of the review CM5 ([REVIEWS.md](../REVIEWS.md)) in N16, N26,
N28, N29 and N33.

### Sources, and how each statement is marked

| | Version | Used for |
| --- | --- | --- |
| Source of `navigation2` | branch `jazzy`, commit `c704655`, package version 1.3.14 (the commit named at the top) | **[S]** |
| Source of `geometry2` (`tf2`, `tf2_ros`, `tf2_geometry_msgs`) | branch `jazzy`, commit `f702874`, version 0.36.23 | **[S]** |
| Binary `nav2_collision_monitor` | 1.3.12, RoboStack, macOS arm64, from this repository's pixi environment, with `tf2` 0.36.22, `nav2_lifecycle_manager` 1.3.12, `rmw_fastrtps_cpp` | **[B: case]** |

- **[S]** means read from the source, with file and function. Paths are
  relative to `nav2_collision_monitor/` unless they start with another
  package's name.
- **[B: case]** means observed on the binary on 2026-11-24 by the named case
  of `review/collision_monitor/nav2/nav2_probe.py`; the output of that run is
  `review/collision_monitor/nav2/results/macos-arm64.txt`. Nothing is marked
  [B] that was not run for this document (by T04, or by T09 where it says
  so). Where the rules above or
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

### Lifecycle (N1–N11)

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

### The state topic (N12–N15)

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

### Zone polygons and point markers (N16, N17)

- **N16. Polygon publishers** (`Polygon::configure`, `Polygon::publish`,
  `CollisionMonitor::publishPolygons`, `Circle::getPolygon`) [S].
  `<zone>.visualize` (default false) creates a publisher of
  `geometry_msgs/msg/PolygonStamped` on `<zone>.polygon_pub_topic` (default:
  the zone's name) with `SystemDefaultsQoS`: reliable and transient local
  with this middleware [B: `visualize`]. Every visualised zone is published
  once on activation and once after every processed command, also when the
  velocity is withheld; not for a non-finite command and not while inactive
  [B: `visualize`, `lifecycle`]. **Only a zone that is enabled is
  published** (`CollisionMonitor::publishPolygons`: `polygon->getEnabled()`)
  [S] [B: scenario `polygon-pub-disabled` of `scenario_nav2.py`, added by
  T09; the review CM5, finding 3a, found the node publishing disabled zones
  too]. Contents [B: `visualize`]: frame
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

### Who starts the node (N18–N21)

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

### Transforms (N22–N28)

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
  clock when the lookup began, and then tries once more. **The wait also
  ends when the node clock is more than 3 s before `start`** (the loop's
  condition has `clock_->now() + rclcpp::Duration(3, 0) >= start_time`, with
  the comment "don't wait bag loop detected") [S]: after a backward jump of
  the clock by more than 3 s the lookup is tried once more and the command
  is answered at once; a jump back by 3 s or less leaves the wait running
  until the clock is at `start + transform_tolerance` again, `start` being
  the clock before the jump [B: scenarios `tf-wait-clock-reset`,
  `tf-wait-clock-back` and `shift-wait-reset` of `scenario_nav2.py`, added
  by T09; this condition was missing from this rule and from the node until
  the review CM5, finding 2]. The callback thread
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
  The buffer is cleared by rclcpp's clock thread when the `/clock` message
  arrives, also while a callback waits for a transform: a transform that
  arrives after the jump is used by the waiting command and is still there
  for the next one [B: `tf-wait-clock-back`].

### Zones whose shape arrives on a topic (N29–N36)

- **N29. Which zone gets which subscription**
  (`Polygon::getParameters`, `Polygon::getCommonParameters`) [S].
  - `<zone>.points` present and usable: the shape is fixed. Neither
    `polygon_sub_topic` nor `footprint_topic` is read; with both set next to
    `points` the node subscribes to neither [B: `footprint-topic-ignored`].
  - `points` present but not usable (the text does not parse, a point is not
    a pair of numbers, or there are fewer than **four** points:
    `Polygon::getPolygonFromString`, `vvf.size() <= 3`): an error is logged
    and the zone is treated as if `points` were absent (`use_dynamic_sub`)
    [S] [B: scenarios `points-short-topic`, `points-malformed-topic` of
    `scenario_nav2.py`, added by T09].
  - `points` absent, `action_type: approach`: `footprint_topic` (default
    `local_costmap/published_footprint`) through
    `nav2_costmap_2d::FootprintSubscriber`.
  - `points` absent, any other action: `polygon_sub_topic` is required. If
    it is not set, configuration fails. If it is set to the empty string,
    configuration succeeds, no subscription is created
    (`Polygon::createSubscription`) and the zone never has a shape [S]
    [B: scenario `sub-topic-empty`].
  - `<zone>.slowdown_ratio` is read only if `action_type` is `slowdown`
    (`Polygon::getCommonParameters`): on a stop zone it may hold anything
    [S] [B: scenario `stop-zone-ratio-type`].

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
  The values are not checked. A box with one NaN coordinate was accepted
  and, in that arrangement, the zone no longer contained a point inside the
  box; three equal points and a polygon whose first point is repeated at the
  end were accepted [B: `polygon-topic`]. **A NaN does not blind the zone in
  general** (review CM5, finding 1; T04 had concluded that it does from the
  one arrangement it tried): the crossing test of R10 skips the two edges
  that touch the NaN vertex and still counts the others, so the zone catches
  some points and misses others, by accident of the arithmetic. Observed
  [B: scenario `polygon-topic-nan` of `scenario_nav2.py`, added by T09]: a
  2 m square around the base whose 4th vertex has a NaN `x` stops for a
  reading inside the square and for a reading 5 m behind the base; with the
  NaN in the 1st vertex the reading inside passes and the one 5 m behind
  stops. What this node does instead: ND6.
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

## Things the source does that the documentation does not say

Compared with the package README and the comments in the example parameter
file. `docs.nav2.org` could not be reached while this was written, so the
configuration guide was not compared.

- The node is not a watchdog on its own: without incoming commands it
  publishes nothing (R1), and after `stop_pub_timeout` of continuous zero it
  stops publishing zero (R15). `stop_pub_timeout` is not a timer.
- The README says a zone acts "if more than `N` points" are inside; the code
  uses at least `min_points` (R11).
- Scan age is taken from the header stamp. A sensor whose clock is ahead of
  the node's never times out; one whose clock is behind by more than the
  timeout always stops the robot (R7).
- A scan whose stamp has negative seconds makes upstream throw in the command
  callback (`rclcpp::Time` refuses it, `rclcpp/src/rclcpp/time.cpp`); nothing
  catches it, so the process ends (D8).
- There is no validation of a scan as a whole, and an all-NaN or empty scan
  reads as free space (R4).
- `source_timeout: 0` switches off not only the age check but also the stop
  before the first scan (R8).
- A negative `slowdown_ratio` reverses the robot (R13). Slowdown is skipped
  when the squared norms underflow to zero or overflow (command components
  below about 10⁻¹⁶² or above 10¹⁵⁴ in magnitude), because the comparison in
  R13 is then false.
- A polygon needs four vertices, not three (R10). Its boundary belongs to it
  on some sides only.
- The output drops `linear.z`, `angular.x` and `angular.y` (R16), yet a NaN
  in one of them discards the command (R2).
- `min_points: 0` makes a zone trigger always (R11).

From the Nav2 extension:

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
- A polygon message with a NaN coordinate is taken as it is; the zone then
  catches some points and misses others (N33). This node's zone demands its
  action instead (ND6).
- A backward jump of the clock makes the node forget every transform it
  received on `/tf` (N28). With base shift correction every command then
  stops for `invalid source` until new odometry has arrived (inferred from
  N24 and N28; `shift-wait-reset` shows the node answering again once it
  has).
- A clock that jumps back by 3 s or less while a command waits for a
  transform holds the node until the clock has caught up with the time the
  wait began, plus `transform_tolerance`; only a jump of more than 3 s ends
  the wait (N26).
- `points` that cannot be used (three points, for instance) do not fail the
  configuration if `polygon_sub_topic` is set: the zone silently becomes one
  fed by that topic, without a shape until a message arrives (N29, N32).

## Units and representation in the Lean core

No `Float` occurs in the core. Three kinds of integers are used.

| Quantity | Representation | Conversion at the boundary |
| --- | --- | --- |
| Node clock, scan stamp | `Nat`, nanoseconds | Upstream's own integers |
| Timeouts | `Nat`, nanoseconds | In Lean, from the parameter's bit pattern: `(int64_t)(seconds * 1e9)` as upstream computes it (R7), reproduced in integer arithmetic (`timeoutNs`) |
| Scan points | `Int`, millimetres, base frame | In Lean, from the binary64 bit patterns of the coordinates in metres in the base frame, which the shim computes and hands over with each command: truncated toward zero, saturated (`coordMm`) |
| Vertices of a fixed polygon | `Int`, millimetres, base frame | In Lean, from the `float` bit patterns of the parameter: rounded to nearest (`vertexMm`) |
| Vertices of a zone fed by a topic | `Int`, millimetres, base frame | In Lean, from the binary64 bit patterns of the vertices in the base frame, which the shim computes from the message: rounded to nearest, ±1,000,000 if not finite or 1024 m or more, which no shape may reach (`vertexCoordMm`) |
| Circle radius | `Nat`, millimetres | In Lean, from the bit pattern: magnitude, rounded up, plus 1 (`radiusMm`) |
| Scan ranges, `range_min`, `range_max` | `Nat`: binary32 bit pattern | None: bits copied |
| Command components | `Nat`: binary64 bit pattern | None: bits copied |
| Slowdown ratio | `Nat`: binary64 bit pattern | None: bits copied |
| `min_points` | `Nat` | Negative values become 0 (same behaviour, R11); refused outside the range of a 32-bit `int` (D10) |

All conversions of the table are in `core/ROSleanCore/CollisionMonitorConvert.lean`.
They work on the exact rational number a pattern denotes, so "rounded up",
"truncated" and "nearest" are meant exactly: there is no intermediate
floating-point product whose own rounding would have to be allowed for.

Values that are only compared or classified are passed as bit patterns
because that needs no rounding: `F32.le` is IEEE `<=` on patterns, `F64.isFinite`
and `F64.isZero` are the tests of R2 and R15, and for non-negative doubles the
order of the patterns is the order of the values, which is all R13 needs.
These definitions were compared with the hardware operations on random and
special patterns, with no disagreement (`review/collision_monitor/Checks.lean`).
`ROSlean/CollisionMonitor/Bits.lean` proves them against the IEEE-754 formula
for the number a pattern denotes (`F32.le` is `≤` on the denoted numbers and
false for NaN; a pattern below `F64.one` denotes a number in `[0, 1)`); what
stays trusted is listed in `THEOREMS.md`.

Bits above the format's width are ignored by `F32.le`, `F64.isFinite` and
`F64.isZero`. A slowdown ratio is compared as a natural number, so a pattern
of 2⁶⁴ or more is never selected rather than read modulo 2⁶⁴; `Config.ok`
refuses it and the C interface cannot produce it.

### Safe direction

"Safe" means: the integer model never misses an obstacle that the configured
zone contains, and never treats data as fresher than it is.

- **Ranges.** Not rounded. R4 is applied to the received bits.
- **Times.** Timeouts are the nanosecond values upstream computes, to the
  nanosecond (R7, R17), so the two nodes have the same boundary. That value
  is a truncation of a rounded product, as upstream's is.
- **Scan points.** The shim computes each point in metres in floating point
  by the operations of R3 and of the transform of N25, in the same order and
  precision. Whether the result is upstream's bit for bit depends on how
  both were compiled; "Scan and shape events" below says for which builds it
  is, and gives the bound otherwise (one unit in the last place of a `float`
  coordinate for R3; an estimated 10⁻⁸ mm for the transform where upstream's
  build fuses multiply-adds, with the margin argument of SPEC_NAV2.md,
  section 5, and the theorems `contains_of_shim`, `contains_of_shim_fixed`,
  `circle_safe_shim`). Lean converts each coordinate to millimetres and
  truncates toward zero, in exact arithmetic. A point therefore never moves
  away from the base frame's origin, and moves by less than 1 mm per axis
  (`coordMm_abs_le`, `coordMm_abs_gt`).
- **Circle.** The magnitude of the radius in millimetres is rounded up and 1
  is added (`radiusMm_ge`: the model's radius is at least the configured one
  plus 1 mm). The rounding up alone would do for the statement that follows;
  the extra millimetre is kept as a margin against the rounding inside
  upstream's own test `x² + y² < r²`. Together with the truncation of points:
  every point strictly inside the configured circle is inside the integer
  circle (`circle_safe`, proved). In the other direction, a point up to 3.5 mm
  outside the configured radius can count as inside (the radius grows by less
  than 2 mm, `radiusMm_lt`, and a point moves inward by less than 1.42 mm).
- **Polygon.** Truncation toward the origin is not a safe direction for a
  polygon: a zone that does not contain the origin loses points across its
  near edge. Vertices are instead rounded to the nearest millimetre (moved by
  at most 0.71 mm), points are truncated as above (moved by less than
  1.42 mm), and the model counts a point as inside if it is inside the integer
  polygon by R10 **or within √5 ≈ 2.24 mm of its boundary** (`nearBoundary`).
  Since 0.71 + 1.42 < 2.24, every point that R10 in exact arithmetic puts
  inside the configured polygon counts as inside in the model: if the rounded
  point is not inside the rounded polygon, the boundary passed over the point
  while both were being rounded, so the point is within 2.13 mm of the rounded
  boundary. In the other direction, a point up to 4.4 mm outside the
  configured polygon can count as inside. Of this argument the quantitative
  half is proved (`nearSegment_iff`: the margin test is exactly "at most √5 mm
  from the segment"; `nearBoundary_of_rounded`: a point of the configured
  boundary is, after rounding and truncation, within the margin). The
  topological half, "the boundary passed over the point", is prose; see
  `THEOREMS.md`.
- **Far and non-finite points.** A coordinate that is NaN, or whose magnitude
  is 1000 m or more, becomes ±1,000,000 mm (NaN of either sign as
  +1,000,000; `coordMm_natAbs_le`).
  `Config.ok` requires every zone to stay inside that limit (circle radius at
  most 1,000,000 mm; polygon vertices at most 999,997 mm from each axis), so a
  saturated point is in no zone. Upstream agrees: a point with a NaN or
  infinite coordinate is inside no circle and no polygon.
- **Velocities.** Not converted at all; see the next section. Zero is the
  literal `0.0`.

## Where the velocity is computed

The core returns an **action**; the shim applies it to the floating-point
command. The core does not compute the output velocity in integers.

| Action | The shim publishes |
| --- | --- |
| `pass` | `linear.x`, `linear.y`, `angular.z` copied from the command, bit for bit |
| `stopZone k`, `stopSource` | 0.0 in all components |
| `slowdown k r` | each of the three components multiplied by the double whose bit pattern is `r` |

In every case `linear.z`, `angular.x`, `angular.y` are 0.0 as upstream (R16),
and the header is copied if messages are stamped.

Reasons:

1. Pass-through and stop are exact. No value goes through integers and back,
   so "the output equals the input" and "the output is zero" are statements
   about the published bits.
2. The core still sees the command, as six bit patterns, and itself decides
   what upstream decides from its value: whether it is finite (R2) and
   whether it is zero (R15). The shim performs no floating-point test.
3. The only floating-point operations left on the command path are the three
   multiplications of a slowdown. The ratio is handed back by the core as the
   same bit pattern it was configured with, so the shim holds no table of
   ratios and cannot pick the wrong one.
4. With velocities converted to integers the theorems would be about integer
   values on both sides of two rounding steps, the published value would
   differ from upstream's in the last digits, and pass-through would need a
   special case anyway.

What remains trusted, i.e. is not covered by a theorem about `step`:

- T1. *The slowdown multiplication.* For a finite double `x` and a double
  `r` with `0 ≤ r < 1`, the IEEE-754 product `x * r` rounded to nearest has
  the sign of `x` or is zero, its magnitude is at most `|x|` and within
  half a unit in the last place of `r·|x|`, and it does not grow when `r`
  is made smaller (the last is what "Slowdown bound" needs). The core guarantees the premise:
  a `slowdown` action always carries a pattern below `F64.one`.
- T2. *The scan's points in the base frame.* (a) The computation of R3 in
  `float`: the angle accumulation, `cos`, `sin`, the product, widened to
  `double`, including which library entry the compiler calls for `cos` and
  `sin`. (b) The transform arithmetic of N25 in `double`: the matrix from
  the quaternion, the two sums, the projection, by tf2's own inline code;
  for a topic zone's vertices the same. (c) How (a) and (b) are compiled:
  with `-ffp-contract=off`, so that no multiply-add is fused (decided by the
  user; enforced by a test of the build). (d) That the coordinates handed
  over with a command are those of the scan last handed to Lean, one pair
  per reading, in order, under the transform looked up for that command;
  and likewise that the vertices handed over for a zone are those of the
  message Lean last took as that zone's shape. Lean checks the number of
  pairs and of vertices, and nothing else about them. All of it is
  `transforms.hpp`, tested by `smoke_transforms` (the reference computation
  bit for bit; the angle over a scan of 1080 readings, where accumulation in
  double precision would give other points) and `smoke_conversions`. The
  conversion of those metres to saturated integer millimetres is **not**
  trusted: it is done in Lean (`coordMm`) and proved to truncate toward
  zero; so is which readings count (R4).
- T3. *The configuration.* The parsing of the `points` text into `float`s
  (upstream's grammar, `parse_vvf`), that the shim passes every parameter
  as it read it and tells the core truthfully which zones are fed by a
  topic, that zone names are distinct, not empty and not `invalid
  source` (ND5: checked in C++ when the parameters are read), and which
  parameter a polygon's shape comes from (N29: `points` if upstream can use
  them, `fixed_points_usable` in `conversions.hpp`, otherwise
  `polygon_sub_topic`). The conversions themselves (radius, vertices, timeouts,
  `min_points`) and every refusal of a value are in Lean. For the radius,
  vertices and coordinates they are proved against the number a bit pattern
  denotes (`ROSlean/CollisionMonitor/Convert.lean`). For timeouts, what is
  proved is which values are refused; that an accepted value is upstream's
  nanosecond count is tested against `rclcpp::Duration` on 230,000 values
  (`smoke_timeouts.cpp`) and against `(int64_t)(seconds * 1e9)` on 300,000
  (`test_cm_ffi.cpp`), not proved, because that would need a formal model of
  IEEE-754 multiplication.
- T4. *The bit-pattern definitions* `F32.le`, `F64.isFinite`, `F64.isZero`
  mean what IEEE-754 says. Proved against the formula for the number a
  pattern denotes (`Bits.lean`); trusted: that formula (`magVal`), and that
  the compiler's `float` and `double` are binary32 and binary64.
- T5. *The shim* delivers every message as one event with one clock reading
  and every lifecycle transition as its event, in order, and no other;
  passes the fields named under "Shim boundary" unaltered; and publishes what
  the table above and "Command event" say and nothing else (the state
  message and the zone polygons included; their contents are display, T8). The table itself is one function,
  `apply_answer` in `conversions.hpp`, tested on its own
  (`smoke_conversions`), including what it does with a kind the core cannot
  return ("Command event").
- T6. *The transforms.* (a) tf2: that the transform it returns is the one
  between the frames and at the times asked for (its tree, its
  interpolation, its product, its conversion to a quaternion). (b) The
  shim's lookup: that it asks tf2 for what N23, N24, N33 and N34 say, for
  the frame and stamp of the scan it last handed to Lean and the frame of
  the shape message Lean last accepted for that zone; that it uses the
  seven numbers unaltered (T2 b); that a failed lookup, and a transform
  with a number that is not finite or with a zero quaternion (ND7), is
  passed as "none". (c) The wait of N26, which is the node's own loop
  (`wait_for`; its two conditions on the clock are `wait_goes_on` in
  `transforms.hpp`, with a unit test), and the clearing of the TF buffer on
  a backward jump of the clock (N28), which the node does itself on the
  `/clock` message that shows the jump, because it has no clock thread
  ("Command event"). Tested on the node by the scenarios `tf-*` and
  `shift-*`.
- T7. *The distance between the shim's points and upstream's* (for the
  comparison only). That the point and the vertices upstream computes by
  N25 are within 0.04 mm per axis of those the shim computes, for
  coordinates below 1000 m. The estimate is 10⁻⁸ mm (SPEC_NAV2.md, section
  5): two evaluations of one expression in `double`, one of them possibly
  with fused multiply-add. It enters no theorem about this node as a fact;
  it is the hypothesis `δ` of `contains_of_shim`, `contains_of_shim_fixed`
  and `circle_safe_shim`, on which "a point upstream counts is always
  counted" rests.
- T8. *Display.* The names in state messages (the shim's table from zone
  index to name) and the polygons published for `visualize` (N16): a fixed
  polygon's vertices as read, the 16 points of a circle, the last message
  taken as a topic zone's shape, an empty polygon before the first such
  message and after a message with an unusable vertex (ND6). A mistake here
  misinforms an observer; it changes no velocity.
- T9. *The clock's validity* (D15). Under simulated time the node decides
  in C++ whether it has a valid clock reading: it has none while the
  `/clock` message received last has `sec < 0`
  (`clock_message_usable` in `conversions.hpp`, tested in
  `smoke_conversions`), or after an exception has left a callback and until
  the next usable `/clock` message. The condition repeats rclcpp's own
  (`rclcpp::Time` refuses a negative `sec`); that rclcpp refuses nothing
  else is read from its source (Jazzy, `rclcpp/src/rclcpp/time.cpp`) and
  seen on the binary for `sec` of −5, −1 and −2³¹ (`fault_injection.py`,
  case `clock-negative`). What the node answers without a clock reading is
  the model's (`stepNoClock`); when it has none is the shim's. The handling is in the
  stand-alone executable's `main`: **D15 is promised for the executable
  only.** Loaded as a component the node ends with its container on such a
  message, as upstream's component does (ND10).
- T10. *The Lean runtime in the node's threads* (review CM5, finding 5).
  The Lean runtime requires `lean_initialize_thread()` in every thread that
  runs Lean code and that Lean did not create, and `lean_finalize_thread()`
  before such a thread ends. `roslean::initialize()` (`runtime.hpp`) does
  both, once per thread, and every member of the C++ wrapper that calls
  into Lean calls it first (`test_cm_ffi`: a monitor built in one thread,
  used in two others and destroyed in a third; the number of
  initialisations is counted). The stand-alone node runs Lean in one thread
  only; as a component it runs in the threads of its container's executor
  (tests `smoke_component`, `smoke_component_single`, `smoke_component_mt`).
  What was checked in the toolchain this repository pins (Lean 4.37.0-rc1,
  built with mimalloc, macOS arm64, by disassembling `libleanrt.a`):
  `lean_initialize_thread` reads an environment variable and does nothing
  else, `lean_finalize_thread` is empty, and neither is declared in
  `lean.h`. A toolchain built with Lean's own small-object allocator sets
  up the thread's heap there. Trusted, and not checked by any test: that
  Lean objects, whose reference counts are not atomic, are never used by
  two threads at once. The node guarantees it by running every callback
  that reaches the core in one mutually exclusive callback group.

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

T1 could be removed later by multiplying in the core on bit patterns (a
correctly rounded binary64 product in integer arithmetic, about fifty lines)
and returning the output bits. It was left out because the proof of "no
amplification" for a hand-written multiplication is a project of its own.

## The model

`core/ROSleanCore/CollisionMonitor.lean`, namespace
`ROSleanCore.CollisionMonitor`; points, bit patterns and the two
conversions the model applies in
`core/ROSleanCore/CollisionMonitorPoint.lean`. The design is SPEC_NAV2.md,
section 3; where a name or a detail differs from the proposal there, it is
said here and corrected there.

- `Config`: `zones : List Zone`, `sourceEnabled`, `sourceTimeout`,
  `stopPubTimeout`. `Zone`: `shape`, `action` (`stop` or `slowdown ratio`),
  `minPoints`, `enabled`. A zone's `shape` is a `ZoneShape`: `fixed s` with
  `s : Shape` (`circle radius` or `polygon vertices`), or `topic`: the shape
  arrives in polygon messages and is held in the state.
- `Coords`: a pair of binary64 patterns, the `x` and `y` in metres of a
  point **in the base frame**, as the shim computed them. There is no
  transform in the model (SPEC_NAV2, section 5).
- `Scan` is `stamp`, `rangeMin`, `rangeMax` and `beams`; a `Beam` is a range
  pattern. The coordinates of the readings are not stored with the scan:
  they arrive with each command. `Twist` is six patterns.
- `TopicShape`, what the state holds for a topic zone after a message
  that counted: `foreign` (the message's frame is not the base frame),
  `current` (the vertices in the base frame, in millimetres), and
  `unusable`: the message had a vertex the node cannot use, the zone has no
  vertices and demands its action (ND6; `TopicShape.unusableMark` is
  `foreign := false`, `current := []`, `unusable := true`).
- `Phase`: `unconfigured`, `inactive`, `active`, `finalized`.
  `Report`: `normal`, `stop zone`, `slowdown zone`, `source`.
- `State`: `phase`, `scan : Option Scan` (R5), `shapes : List (Option
  TopicShape)` with one entry per zone, `prevZero : Bool`, `stopStamp : Nat`
  (R15), `prevReport : Report` (N13). `State.init cfg` is unconfigured, no
  scan, `none` for every zone, `false`, 0, `normal`. (It takes the
  configuration, so that the shapes have one entry per zone from the
  start.)
- `Event`: `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`
  (E1–E5); `scan now s` (E6); `shape k verts foreign` with `verts : Option
  (List Coords)` the message's vertices in the base frame, `none` if its
  transform was not found (E7); `cmd now v scanPts zoneVerts` with `scanPts
  : Option (List Coords)`, one pair per reading of the stored scan, and
  `zoneVerts : List (Option (List Coords))`, one entry per zone (E8). A
  missing transform is not an event: it is `scanPts = none`. The event
  `scanUnusable` of the node before the extension is gone: a scan is not
  usable or unusable when it arrives; whether it can be placed is decided
  at each command, by what the command carries.
- `Output`: `silent`, or `answer velocity report` with `velocity : Option
  Action` and `report : Option Report`. An `Action` is one of `pass`,
  `slowdown zone ratio`, `stopZone zone`, `stopSource`. `answer none _` is a
  processed command whose velocity is withheld (R15); it differs from
  `silent`, for which nothing at all is published (N16). `Output.velocity`
  and `Output.report` read the two parts (`none` for `silent`).
- `step : Config → State → Event → State × Output`, total, and `run` over a
  list of events.

`step` by event (SPEC_NAV2, 3.2):

| Event | In which phase it acts | What it does | Output |
| --- | --- | --- | --- |
| `configure` | unconfigured | phase := inactive | `silent` |
| `activate` | inactive | phase := active | `silent` |
| `deactivate` | active | phase := inactive, `prevZero := false`, `prevReport := normal` | `silent` |
| `cleanup` | inactive | phase := unconfigured, `scan := none`, every shape := `none` | `silent` |
| `shutdown` | any | phase := finalized; from there nothing changes any more | `silent` |
| `scan now s` | inactive, active | `scan := some s` | `silent` |
| `shape k verts foreign` | inactive, active | if accepted (`acceptShape`), entry `k` of the shapes := the new shape, or the mark of an unusable message (ND6) | `silent` |
| `cmd …` | active, and `v` finite | below | `answer velocity report` |

In every other phase the event changes nothing and the output is `silent`.

A shape message is accepted iff zone `k` exists and is a topic zone, `verts
= some vs` and `vs` has at least three entries (N33). What it installs: if
every vertex `vertexPointMm v` is within `coordLimit − 3` of both axes,
which a coordinate that is not finite never is, the shape `{ foreign,
current := vs.map vertexPointMm, unusable := false }`; otherwise the mark
`TopicShape.unusableMark`, in place of whatever the zone had (ND6). A
message that is not accepted is ignored: the zone keeps what it had.

`step` on a processed command is built from named parts, so that properties
can be stated on each:

| Definition | Meaning | Rules |
| --- | --- | --- |
| `scanPointMm c` | the point of a reading from its base-frame coordinates: each in millimetres, truncated toward zero and saturated at ±`coordLimit` (`coordMm`); a NaN is `+coordLimit` | D3, D12 |
| `vertexPointMm c` | the vertex of a topic zone from its base-frame coordinates: each rounded to the nearest millimetre, a half away from zero; ±`coordLimit` (always refused) if not finite or 1024 m or more | N33, N34, ND6 |
| `Scan.fits s c`, `Scan.points s c` | `c` has one pair per reading of `s`; the points of the valid readings: reading `i` with pair `i`, the pairs of readings that are not valid dropped | R4, N25 |
| `sourcePoints cfg st now scanPts` | `some pts`: the points to test; `none`: invalid source. Coordinates that do not fit the stored scan count as none | R5–R8, N22, N27 |
| `refreshShapes cfg shapes zoneVerts`, `TopicShape.refresh` | every enabled topic zone's shape, if it is in another frame, takes the vertices the command carries for it, when they are as many as it has and all within the limit | N34, ND3 |
| `liveConfig cfg st zoneVerts` | the zones as tested: the configuration with each topic zone given its refreshed shape as a polygon, or left without shape; a topic zone whose entry is the mark of an unusable message is tested with the threshold 0, so that it is triggered whenever it is enabled (`Zone.withShape`) | N32, N34, ND6 |
| `Shape.contains`, `Zone.count`, `Zone.triggered` | zone tests; a zone without shape is never triggered | R9–R11, D3, D4, N32 |
| `firstStop`, `bestSlowdown`, `zonesAction cfg pts` | action for given points | R12–R14 |
| `evaluate cfg st now scanPts zoneVerts` | action for a command; independent of its velocity: `stopSource` if the source is invalid, else `zonesAction (liveConfig …) pts` | R8, R12–R14 |
| `shapesAfter …` | the shapes after the command: refreshed, or unchanged if the source is invalid | N34 |
| `Action.yieldsZero a v` | the velocity to publish is zero | R15, D6 |
| `Action.report a v` | `source` for `stopSource`, `stop k` for `stopZone k`, for `slowdown k _` `slowdown k` unless `v` is zero in the three published components, then `normal`; `normal` for `pass` | N13, N14 |
| `Config.ok` | the configuration is accepted; a topic zone needs only an accepted action | D5, D12, R10 |

For a processed command: `a := evaluate …`; the velocity of the output is
`none` if `a.yieldsZero v` and `st.prevZero` and `st.stopStamp +
stopPubTimeout < now`, else `some a`; the report of the output is `some r`
with `r := a.report v` if `r ≠ st.prevReport`, else `none`; the new state has
the shapes of `shapesAfter`, `prevZero := a.yieldsZero v`, `stopStamp := now`
if that is true and `st.prevZero` was not, and `prevReport := r`.

## Properties

Proved about `step` and lifted to `run` in `ROSlean/CollisionMonitor/`; the
theorem for each is named in [THEOREMS.md](THEOREMS.md). `cfg`, `st`, `now`,
`v`, the coordinates are arbitrary unless restricted; no property needs
`Config.ok` unless it says so. "The output of the command" is
`(step cfg st (.cmd now v scanPts zoneVerts)).2`; its *velocity* and its
*report* are the two parts, both `none` for `silent`. A zone *is triggered
by `pts`* means `z.triggered pts = true`, for `z` one of the zones in force,
`(liveConfig cfg st zoneVerts).zones`. *The source is invalid* means:
`cfg.sourceEnabled`, `0 < cfg.sourceTimeout`, and either `st.scan = none`,
or `st.scan = some s` with `s.stamp + cfg.sourceTimeout < now`, or
`scanPts` is `none` or is not one pair per reading of the stored scan.

**Processed.** The output of an event is not `silent` iff the event is a
command, `st.phase = active` and `v.finite`. Every other event, and every
event in another phase, gives `silent`.

**Inactive.** If `st.phase ≠ active`, the output of every event is `silent`,
and a command leaves the state unchanged. On runs from `State.init cfg`: no
output before the first `activate` is other than `silent`; none after a
`shutdown` is; none between a `deactivate` and the next `activate` is.

**Velocity.** For a processed command the velocity is `none` iff `(evaluate
…).yieldsZero v` and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout <
now`; otherwise it is `some (evaluate …)`. The properties below that end in
"or withheld" are to be read with this: a processed command has no velocity
in this one case and in no other.

**Report.** For a processed command the report is `some r` iff `r = (evaluate
…).report v` and `r ≠ st.prevReport`; the new `prevReport` is `(evaluate
…).report v` in both cases. A report is made also when the velocity is
withheld.

**Only if.** `evaluate …` is `.stopSource` iff the source is invalid;
`.stopZone k` iff the source is not invalid and `k` is the first triggered
stop zone; `.slowdown k r` iff the source is not invalid, no stop zone is
triggered, and `k` is the first triggered slowdown zone with the smallest
ratio pattern `r`, which is below `F64.one`; `.pass` iff the source is not
invalid and neither kind is triggered.

**State.** After a processed command, `prevZero` is whether the demanded
velocity was zero, and `stopStamp` is `now` if it was zero and the previous
one was not, and unchanged otherwise. On runs from `State.init cfg`: if a
processed command gets no velocity, the velocity published last before it
was a zero. (Not: "a stop is published whenever it is demanded". After zero
commands for longer than `stop_pub_timeout`, an obstacle entering a stop
zone together with a non-zero command gets no velocity; the last published
one is the earlier zero. Example in `Examples.lean`. The state message does
report the stop.)

**Monotonicity.** Adding points never turns a stop into anything else,
and never turns a slowdown into `pass` or into a slowdown with a larger
ratio.

**Stop.** If `st.phase = active`, `v.finite`, `sourcePoints cfg st now
scanPts = some pts` and some zone in force with action `stop` is triggered by `pts`,
then `evaluate … = .stopZone k`, where `k` is the index of the first such
zone; and the velocity of the output is `some (.stopZone k)` or withheld,
the latter exactly in the case of **Velocity**. In particular it is never
`some .pass` or `some (.slowdown …)`.

In words: if at least `min_points` valid readings of the stored scan, at
the coordinates the command carries for them, lie in an enabled stop zone that has a shape, an
active node publishes exactly zero or no velocity. "No velocity" occurs
after `stop_pub_timeout` of continuous zero (R15); nothing at all is
published for a non-finite command (R2) and while the node is not active
(N4). The literal statement "the output velocity is exactly zero" is
therefore **false for upstream and for the model** when read as "zero is
published": counterexample in `Examples.lean`, the command at 2001 ms in the
`stop_pub_timeout` example. See A3.

The hypothesis `sourcePoints … = some pts` unfolds to: the source is enabled,
a scan `s` is stored, it is not too old, and either `scanPts = some c` with
one pair per reading and `pts = s.points c`, or there are no such
coordinates, the timeout is 0 and `pts = []`; or the source is disabled or (timeout 0 and no
scan) and `pts = []`. With `pts = []` only a zone with `minPoints = 0`
triggers.

**Stale data.** If `st.phase = active`, `v.finite`, `cfg.sourceEnabled`,
`0 < cfg.sourceTimeout`, and either `st.scan = none` or `st.scan = some s`
with `s.stamp + cfg.sourceTimeout < now`, then, whatever `scanPts` is,
`evaluate … = .stopSource`, and the velocity of the output is `some
.stopSource` or withheld, the latter exactly in the case of **Velocity**.

"Older than `source_timeout`" is by header stamp (R7), and "newest scan" is
the scan received last (R5). The property is **false without the two
hypotheses on the configuration**: with `source_timeout: 0` or a disabled
source, stale or missing data does not stop the robot (examples
`cfgNoTimeout`, `cfgSourceOff`). That is upstream's documented way of
switching the check off. On event sequences: after `run`, `st.scan` is the
scan in force: that of the scan event received last while the node was
inactive or active, unless a cleanup that took effect followed it
(`lastScan`, `run_scan`).

**No transform.** If `st.phase = active`, `v.finite`, `cfg.sourceEnabled`,
`0 < cfg.sourceTimeout`, and `scanPts` is `none` (or does not fit the stored
scan), then, whatever scan is stored, `evaluate … = .stopSource`, and the
velocity of the output is `some .stopSource` or withheld, the latter exactly
in the case of **Velocity**. It is **false without the two hypotheses on the
configuration**, as Stale data is: with `source_timeout: 0` a missing
transform means "no obstacle" (N27). Counterexample in `Examples.lean`: with
timeout 0, a scan with two points in the stop zone stops the robot when the
command carries its points and lets it pass when the command carries none.
This is upstream's behaviour and it is fail-open.

**Bad data.** The rule is R4 for readings (`Scan.validBeam`: a reading counts
iff `F32.le rangeMin range && F32.le range rangeMax`), R2 for commands
(`Twist.finite`), and D8 for a negative stamp. Statements: (a) readings that
are not valid never change a decision: the output of a command is the same
when they are removed from the stored scan and their pairs from the
command's coordinates, whatever those pairs are; (b) the output of a command
that is not finite is `.silent` and the state is unchanged; (c) `step` is
total, and so are `scanPointMm`, `vertexPointMm` and the conversions of the C
interface (a byte array of any length is a scan). Upstream itself does not
satisfy "never crashes": see D8.

**Scan points.** If the command's coordinates fit the stored scan, the
number of points tested is the number of valid readings of the stored scan,
whatever the coordinates are; it is never larger. Each point is the pair of
a valid reading, converted: for finite coordinates the value handed over, in
millimetres, truncated toward zero and saturated at ±`coordLimit`; unless
saturated, less than 1 mm from it on each axis. That a pair is the image of
its reading under the right transform is not a property of the model: the
shim computes it (T2, T6).

**No amplification.** If the velocity of the output of any event is `some
a`, then `a` is `.pass`, `.stopZone _`, `.stopSource`, or `.slowdown k r`
with `r < F64.one`; in the last case zone `k` of `cfg.zones` exists and has
action `.slowdown r`. With the table of the previous section and T1 this
gives: on each of the three axes the published magnitude is at most the
commanded one and the sign does not flip; the other three axes are 0. This
holds for every configuration, because a ratio pattern above `F64.one` (a
ratio above 1, negative, infinite or NaN) is never selected. It is **false
for upstream** with a negative ratio: `slowdown_ratio: -0.5` publishes
`−0.5 ×` the command (D5).

**Slowdown bound.** If `sourcePoints cfg st now scanPts = some pts`, and a
zone in force with action `.slowdown rz` is triggered by `pts`, and
`rz < F64.one`, then `evaluate …` is `.stopZone _`, or `.slowdown k r` with
`r ≤ rz`. Moreover `r` is the smallest ratio pattern among the triggered
slowdown zones and `k` the first zone that has it. With T1: the published
magnitude on each axis is at most that of `command × rz` computed in
floating point, for *every* triggered slowdown zone. If `rz = F64.one` the
action may be `.pass`, which satisfies the bound with equality.

**Pass-through.** If `st.phase = active`, `v.finite`, `sourcePoints cfg st
now scanPts = some pts` and no zone in force is triggered by `pts`, then
`evaluate … = .pass`, and the velocity of the output is `some .pass`, unless
`v.planarZero` and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout <
now`, in which case it is withheld. In particular a non-zero command is
always published unchanged. This rules out the always-stop node. The
exception is upstream's: a zero command is dropped like any other zero after
`stop_pub_timeout` (R15; example with `halt`). When a scan is stored and the
timeout is not 0, coordinates that fit it are part of the hypothesis on
`sourcePoints`.

**Vertices of a topic zone.** Every accepted vertex has finite coordinates
below 1024 m and is within 1/2 mm, on each axis, of the vertex the shim
handed over. (T05's property "Transform", that a point is the exact image of
its reading under the command's transform, truncated, went with the
transform: THEOREMS.md, "What T05 proved and option A does not have".)

**The margin with the shim's transform.** If the points and vertices
upstream computes differ from the shim's by at most `δ` on each axis, a
point upstream has on the boundary of its polygon is counted by the model:
for a topic zone if `δ ≤ 1/25` mm, for a fixed polygon if `δ ≤ 2/25` mm, and
a point upstream has strictly inside a circle if `δ ≤ 1/2` mm. `δ` itself is
estimated at 10⁻⁸ mm and trusted (SPEC_NAV2, section 5, T7).

**Shape accepted.** A `shape k verts foreign` event is accepted iff the
conditions of "The model" hold. If it is not accepted the state is
unchanged. If it is, entry `k` of the shapes becomes what the message
installs (the shape: whether the frame is foreign, the rounded vertices; or
the mark of an unusable message) and nothing else changes: not
the other entries, not `scan`, `prevZero`, `stopStamp`, `prevReport`,
`phase`. So the state changes only if the message is accepted. (Not "iff":
an accepted message that carries the shape the zone already has changes
nothing.) At a command a refresh changes the positions of a foreign shape's
vertices and nothing else, not their number.

**Unusable message (ND6; review CM5, finding 1).** A message installs the
mark iff it is accepted and one of its vertices, rounded to the millimetre,
is not within the coordinate limit (iff). A topic zone with an entry in the
state is triggered iff it is enabled and either the entry is the mark or at
least `minPoints` points are inside the polygon of its vertices (iff); with
the mark, exactly when it is enabled, whatever the points. Hence: *never
less restrictive for that zone*: if the zone is triggered with whatever
else the state could hold for it (no shape, or any shape, in particular the
one it had before the message), it is triggered with the mark. If zone `k`
is an enabled stop zone fed by a topic and the state holds the mark for it,
the action for every command is a stop (by zone `k` or an earlier triggered
stop zone, or for an invalid source), with no hypothesis on the scan, the
coordinates, the clock or `source_timeout`; so every velocity published is
zero. The mark is neither set nor removed by any event other than a polygon
message for that zone and a cleanup. On runs from `State.init cfg`: after a
message that installed the mark, and any events that follow among which
there is no polygon message for zone `k` and no cleanup, whatever velocity
is published for the next event is a stop. For the only-if direction of
"Only if (stop zone)" and "Only if (slowdown)" this adds a case: the zone
has `minPoints` points inside, **or** it is a topic zone with the mark.

**No shape.** If zone `k` is a topic zone and the state holds no shape for
it, zone `k` is not triggered by any points, and no output names it, as a
velocity action or in a report. On runs from `State.init cfg`: after any
history that leaves the node unconfigured (start-up, or a cleanup that took
effect), and any events that follow among which there is no shape message
for zone `k`, no output names zone `k`. This is upstream's behaviour (N32)
and it is **not** fail-safe: a stop zone without shape does not stop.

**Replacing a shape affects that zone only.** For two lists of shapes that
agree on every zone but `k`, and any points: every zone other than `k` is
the same zone under both; hence the actions differ only if zone `k` is
triggered under one and not under the other.

**Lifecycle invariants.** In every state reachable from `State.init cfg`: if
the phase is unconfigured or inactive then `prevZero = false` and
`prevReport = normal`; if it is unconfigured then `scan = none` and every
entry of `shapes` is `none`; `shapes` has one entry per zone and a shape only
at a topic zone; every stored shape has at least three vertices, all within
`coordLimit − 3` of both axes.
(Not claimed for the finalized phase: a node shut down from the active phase
keeps `prevZero` and `prevReport` as they were. They are never read again.)

**After activation.** If the node is inactive in a reachable state, then
after `activate` it is active, and for a finite command that comes next the
velocity is `some (evaluate …)`, also when it is zero, and the report is
`some r` unless `r = normal`. A stop is never withheld right after an
activation (N6).

**Saturated points.** With `Config.ok`, in a reachable state, every zone in
force lies within the coordinate limit, so a saturated point is in no zone
and does not change the action.

**Totality.** `step` and `run` are total functions; Lean accepts them without
`partial` or `unsafe`.

## Shim boundary

Lean's C interface passes scalars and Lean objects, never C structs. The
interface follows the pattern of `core/ROSleanCore/FFI.lean`: an opaque
builder, an opaque monitor object holding configuration, state and the last
output, and scalar queries. It is `core/ROSleanCore/CollisionMonitorFFI.lean`;
the C prototypes are in `ros/roslean_core/include/roslean_core/ffi.h` and are
checked against the generated C at build time. Every Lean object argument is
consumed (PLAN, lesson 8).

**The shim converts nothing and refuses no value.** Every parameter crosses
as the shim read it: a `double` as its eight bytes (`uint64`), a number of a
polygon's `points` as the four bytes of the `float` upstream parses it into
(`uint32`), `min_points` as the 64-bit integer. Lean converts to millimetres
and nanoseconds, refuses what the node does not accept, and reports which
parameter it refused. No cast in the shim depends on the value of a
floating-point number, so none has undefined behaviour for any input. (This
section was changed after the review CM1/CM2, findings 1, 7, 11 and 13: an
earlier version had the shim convert radius, vertices and timeouts in
floating point and refuse out-of-range values itself.)

What the shim still refuses by itself are requests for features outside the
subset ("Excluded features"), a zone list that the state topic could not
report unambiguously (ND5), and the removed parameter
`<source>.roslean_sensor_pose` (with a message that points to TF): they
concern which parameters exist, not the value of a number.

**All refusals are reported together.** The shim does not stop at the first
parameter it refuses: it reads everything, builds the monitor, asks the core
which values it refused (each timeout through `roslean_cm_timeout_ns`, each
zone through `roslean_cm_zone_error`), logs one error message of the form
"N parameters refused: parameter 'a': …; parameter 'b': …", and fails the
`configure` transition: the node stays unconfigured and the process stays
up, as upstream (N3; this replaced the exit of D14). One reason
is given per parameter and, from the core, one per zone (the first of
`radius` or `points`, `slowdown_ratio`, with a refused raw value before
either).

**Parameters are read in `on_configure` (N3) and cannot be changed (ND8).**
Every change through the parameter service is rejected with a reason that
says so, and a name that was not given at start-up is not declared and
cannot be set. Before the review CM3/CM4 (finding 2) a change was answered
"successful" and ignored.

**Warnings when configured.** For an accepted configuration that protects
nothing, or less than it seems to, the node logs one warning each: a source
timeout of 0; a disabled source; a zone with `min_points` of 0 or less; a
slowdown zone whose ratio is not below 1; a circle of radius 0; a polygon
without area (computed exactly on the millimetre vertices the core uses);
no zones (which a parameter file cannot express: rcl rejects an empty list).
A zone fed by a topic gets one too: it takes no action until its first
accepted message (N32). These are warnings, not refusals (decided by the
user: accept and warn).

Integer conversions on the Lean side: `stamp_ns` is converted with
`.toInt.toNat`, so a negative value becomes 0 (D8); unsigned arguments are
converted with `.toNat`. A flag is true iff the byte is not 0.

### Start-up

In `on_configure` the shim reads the parameters of R17 and of SPEC_NAV2.md,
8.5, refuses what the table under "Excluded features" tells it to refuse,
and calls:

| Call | Arguments | Meaning |
| --- | --- | --- |
| `roslean_cm_builder_new` | `()` | Empty builder: no zones, `stop_pub_timeout` 0, and the source not set. Its monitor is **not ok** until `set_source` has been called |
| `roslean_cm_builder_set_source` | builder, `enabled : uint8`, `node_timeout_bits : uint64`, `has_own : uint8`, `own_timeout_bits : uint64` | `sourceEnabled`, `sourceTimeout`. Required. A refusal recorded by an earlier call stays |
| `roslean_cm_builder_set_stop_pub_timeout` | builder, `timeout_bits : uint64` | `stopPubTimeout`. A refusal recorded by an earlier call stays |
| `roslean_cm_builder_add_circle` | builder, `radius_bits : uint64`, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a circle zone |
| `roslean_cm_builder_add_polygon` | builder, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a polygon zone with no vertices yet |
| `roslean_cm_builder_add_topic_polygon` | builder, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a polygon zone whose shape arrives on a topic (N31): no shape until a shape call is accepted |
| `roslean_cm_builder_add_vertex` | builder, `x_bits : uint32`, `y_bits : uint32` | Appends a vertex to the zone added last. If that is not a fixed polygon, or there is no zone, the vertex belongs to nothing and the monitor will not be ok (code 6) |
| `roslean_cm_new` | builder | The monitor, unconfigured (`State.init`), output `silent` |
| `roslean_cm_ok` | monitor | `uint8`: the configuration is accepted. If 0 the `configure` transition fails and the monitor is dropped |
| `roslean_cm_on_lifecycle` | monitor, `transition : uint8` → monitor | 0 `configure`, 1 `activate`, 2 `deactivate`, 3 `cleanup`, 4 `shutdown` (E1–E5); any other value changes nothing. Called from the corresponding `on_…` callback |
| `roslean_cm_phase` | monitor | `uint8`: 0 unconfigured, 1 inactive, 2 active, 3 finalized. For logs and tests |
| `roslean_cm_error` | monitor | `uint8`: 0 accepted; 1 `source_timeout`; 2 `<source>.source_timeout`; 3 `stop_pub_timeout`; 4 a zone; 5 the source was never set; 6 a vertex was added to no polygon |
| `roslean_cm_zone_error` | monitor, `k : uint32` | `uint8` for zone `k`: 0 accepted; 1 `radius`; 2 `points`; 3 `slowdown_ratio`; 4 `min_points`; 5 no such zone |

`action` is 0 for `stop` and 1 for `slowdown`; any other value is `stop`.
`ratio_bits` is ignored for `stop`. Zones are added in the order of the
`polygons` parameter; the zone index in an output is the position in that
order, from 0. The two error queries exist only so that the shim's message
can name the parameter; `roslean_cm_ok` is the decision.

What Lean does with the raw values (`CollisionMonitorConvert.lean`):

- **Timeouts** (`timeoutNs`): `(int64_t)(seconds * 1e9)`, the expression of
  `rclcpp::Duration::from_seconds`, reproduced exactly: the binary64 product
  rounded to nearest, then truncated. Refused (D9): negative values (`-0.0`
  is 0), NaN, infinities, and 9.2·10⁹ s or more.
- **The source's timeout** (`sourceTimeoutNs`): `own_timeout_bits` converted
  as above if `has_own`, i.e. if `<source>.source_timeout` is set; otherwise
  the node's timeout through upstream's double conversion (R17,
  `nsViaSeconds`). The node's `source_timeout` must be acceptable in both
  cases, since upstream converts it in both.
- **Radius** (`radiusMm`): the magnitude (upstream squares the radius, R9, so
  `radius: -0.3` is a 0.3 m zone), in millimetres, rounded up, plus 1.
  Refused: NaN, infinities, and any value whose result exceeds 1,000,000 mm
  (D12). The largest accepted magnitude is therefore 999.999 m exactly, not
  "below 1000 m": the double written `999.999` lies just above it and is
  refused, `999.9989` is accepted. The node's message says this.
- **Vertices** (`vertexMm`): millimetres rounded to nearest, a half away from
  zero. Refused: NaN and infinities; a refused vertex marks its polygon, or
  the builder if the zone added last is not a polygon; it is never dropped.
  Whether the polygon has four vertices inside the coordinate limit is
  `Shape.ok`.
- **`slowdown_ratio`**: the bits, unchanged. Refused if the sign bit is set
  (`ZoneAction.ok`, D5).
- **`min_points`** (`minPointsOfRaw`): negative values become 0. Refused
  outside −2³¹ … 2³¹ − 1 (D10).

The monitor is ok iff the source was set, nothing was refused, no vertex was
given to no polygon, and `Config.ok` holds (`cmOk_iff`).

**The builder is fail-closed** (review CM3/CM4, finding 6). There is no
default source timeout: without `set_source` the monitor is not ok
(`not_ok_without_set_source`). A refused value makes the monitor not ok
wherever in the sequence of calls it occurs and whatever follows it
(`not_ok_of_refused_call`): a second `set_source` with a good value does not
clear a refused timeout, and a refused vertex counts also after a circle or
before any zone. The node's own call sequence produced none of these cases;
the rule is for any other caller of the interface.

**Fail closed.** `roslean_cm_ok` is not advisory. A monitor that is not ok
ignores scans and answers **every** command with `stopSource`: finite or not,
whatever the scans, and without the silence of R15 (`Monitor.apply`,
`apply_cmd_of_not_ok`, `cm_not_ok_after`), once it is in the active phase;
before that it is silent like any monitor. The node fails `configure` for
such a monitor and never activates it; this is the backstop should that
ever be removed or bypassed. In particular a refused `source_timeout` cannot turn
into 0, which would mean "check disabled" (review finding 7a).

### Scan and shape events

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_scan` | monitor, `now_ns : uint64`, `stamp_ns : int64`, `range_min_bits : uint32`, `range_max_bits : uint32`, `ranges : ByteArray` → monitor |
| `roslean_cm_on_shape` | monitor, `zone : uint32`, `foreign : uint8`, `has_verts : uint8`, `verts : ByteArray` → monitor |
| `roslean_cm_shape_accepted` | monitor → `uint8`: the shape call made last was accepted (E7): it installed a shape or the mark of an unusable message |
| `roslean_cm_shape_unusable` | monitor, `zone : uint32` → `uint8`: the state holds the mark of an unusable message for that zone (ND6). A query |

- `now_ns`: the node clock, read once in the callback.
- `stamp_ns = (int64_t)header.stamp.sec * 1000000000 + header.stamp.nanosec`
  (integer arithmetic; it cannot overflow). A negative value becomes 0 in
  Lean (D8).
- `range_min_bits`, `range_max_bits`: the four bytes of the `float` fields.
- `ranges`: a Lean `ByteArray` of `4 × ranges.size()` bytes, the bit pattern
  of each range, little-endian (`encode_ranges` in `conversions.hpp`). Lean
  decodes `size / 4` readings and ignores a remainder. There is no argument
  `usable` any more and no coordinates: where the readings lie in the base
  frame depends on the transform at the time of each command (N22) and is
  handed over with the command. The shim keeps the message (ranges,
  `angle_min`, `angle_increment`, `frame_id`, stamp) for that.
- `on_shape` (SPEC_NAV2.md, 8.2): `verts` has 16 bytes per vertex, the bit
  patterns of `x` and `y` **in the base frame** as little-endian `double`s,
  computed by the shim from the `float` coordinates of the message under the
  transform of N33 (`polygon_vertices` in `transforms.hpp`); `has_verts` is 0
  if that lookup failed; `foreign` is 0 iff the message's `frame_id` equals
  `base_frame_id`. The lookup is made only for a message with at least three
  points; a shorter one is handed over at once and ignored by Lean. What
  the shim keeps follows Lean's answer: if `roslean_cm_shape_accepted`
  returns 0 the message is ignored, an error is logged and the shim keeps
  what it had; if it returns 1 and `roslean_cm_shape_unusable` for the zone
  returns 0, the shim keeps the message for the lookups of later commands
  (N34) and for display (N16); if both return 1 (ND6) the shim drops the
  message it had kept for the zone, shows an empty polygon and logs an
  error: there is nothing to look up for a zone without vertices. The
  wrapper returns the three cases as `ShapeResult` (`Ignored`, `Shape`,
  `Unusable`); that the second query after an accepted call says what the
  call installed is `cmShapeUnusable_after`.

The cost, measured on an Apple M5 core: the scan call with the shim's
encoding is a copy of the ranges and a decoding into a list, a few
nanoseconds per reading; a command is under "Command event".

The point of reading `i` in metres (T2), computed **for each command**, in
this order of operations (`scan_points` in `transforms.hpp`; `tf` is the
looked-up transform as a `tf2::Transform`):

```
float angle = msg.angle_min;
for (size_t i = 0; i < msg.ranges.size(); i++) {      // every reading, valid or not
  float xf = msg.ranges[i] * std::cos(angle);         // float × float
  float yf = msg.ranges[i] * std::sin(angle);
  tf2::Vector3 p_b = tf * tf2::Vector3(xf, yf, 0.0);  // N25, in double; tf2's own operator
  pair i = { bits(p_b.x()), bits(p_b.y()) };          // the third coordinate is dropped
  angle += msg.angle_increment;                       // float
}
```

compiled with `-ffp-contract=off`, so that no multiplication and addition
are fused (decided by the user for every platform; upstream's arm64 binary
fuses the two sums, so the last bits of a coordinate can differ from
upstream's there: ND1). The shim does not test the range of a reading and
skips none: Lean refuses a list that is not one pair per stored reading and
decides which readings count (R4).

**How far these points are upstream's** (review CM3/CM4, finding 5). The
operations are upstream's; the bits of the result also depend on the
compiler and on the C library, in two ways.

- *Which library entry computes `cos` and `sin`.* With optimisation a
  compiler replaces the two calls by one that returns both
  (`__sincosf_stret` on macOS, `sincosf` with glibc); without, it calls
  `cosf` and `sinf`. On macOS arm64 the two ways differ in about 1 % of the
  coordinates, by one unit in the last place of the `float` product: at most
  9.5·10⁻⁷ m for ranges up to 12 m (measured: 2,296 of 200,000 coordinates
  by the reviewer, 4,740 of 400,000 by `smoke_conversions`). Upstream's
  binary from the pixi environment makes the single call. The package
  therefore builds with optimisation by default: `CMakeLists.txt` sets the
  build type to `Release` when none is given, and the node then makes the
  single call too.
- *Fused multiply-add.* The transform of N25 has multiplications followed
  by additions. `-ffp-contract=off` keeps them unfused; a fused evaluation
  of the two sums (what upstream's arm64 binary does) differs in the last
  bits of 13 % of the coordinates (`smoke_transforms`: 52,757 of 400,000),
  none by anything near a millimetre (SPEC_NAV2.md, section 5). A second
  user of `transforms.hpp` must set the flag as well. `smoke_transforms`
  compares the transform with a reference that cannot be fused, on 200,000
  transforms, and fails on arm64 when the flag is missing.

What is true, then:

| Case | Agreement with upstream's points |
| --- | --- |
| Scan in the base frame without base shift correction; node built with optimisation (`Release`, the default, or `RelWithDebInfo`); upstream built with optimisation; same machine and C library | Bit for bit: both apply the identity to the scan-frame point (which changes no value but the sign of a zero). The scan-frame points are checked by `smoke_conversions` against a copy of upstream's loop compiled with `-O2`: 0 of 400,000 coordinates differ |
| The same, node built without optimisation (`Debug`, or `-O0`) | Not bit for bit on macOS arm64: about 1 % of the coordinates differ by one unit in the last place of a `float`, under 10⁻⁶ m for ranges up to 12 m. `smoke_conversions` fails in such a build, on purpose. Not measured for other C libraries |
| Any other transform | The same operations in the same order (tf2's own code). Bit for bit where upstream's build does not fuse multiply-adds; where it does (arm64), within an estimated 10⁻⁸ mm (ND1; SPEC_NAV2.md, section 5 and T7). Not compared bit for bit with the binary |

In every case the difference is far below the millimetre by which D4 grows
each zone, so no point that upstream counts as inside a zone is lost by it;
a decision can differ only for a point within D4's band of a boundary, where
the two nodes differ anyway. T2 is correspondingly larger than "`cos` and
`sin`": it includes the library entry the compiler chooses.

Nothing is returned that the shim acts on: the output of a scan event is
always `silent`.

### Command event

| Call | Arguments |
| --- | --- |
| `roslean_cm_cmd_plan` | monitor, `now_ns : uint64`, `lin_x`, `lin_y`, `lin_z`, `ang_x`, `ang_y`, `ang_z : uint64` → `uint8` |
| `roslean_cm_on_cmd` | monitor, `now_ns : uint64`, the six `uint64`, `has_points : uint8`, `points : ByteArray`, `zone_verts : ByteArray` → monitor |
| `roslean_cm_on_cmd_no_clock` | monitor, `lin_x`, `lin_y`, `lin_z`, `ang_x`, `ang_y`, `ang_z : uint64` → monitor |
| `roslean_cm_on_cmd_reference` | as `roslean_cm_on_cmd`; for tests only |
| `roslean_cm_out_kind` | monitor → `uint8` |
| `roslean_cm_out_zone` | monitor → `uint32` |
| `roslean_cm_out_ratio` | monitor → `uint64` |
| `roslean_cm_out_report` | monitor → `uint8` |
| `roslean_cm_out_report_zone` | monitor → `uint32` |

The sequence for one command is SPEC_NAV2.md, 8.3, as implemented:

- `now_ns`: the node clock, read once at the start of the callback (D7,
  ND2).
- `cmd_plan` says which transforms upstream would look up for this command
  (`cmdPlan`): 0 none (the command is not processed, or the source is
  invalid before any lookup); 1 those of the topic zones only; 3 the scan's
  and the zones'. It changes nothing. Proved: with plan 0 the result of
  `on_cmd` depends on neither array, with plan 1 not on `points`
  (`cmOnCmd_of_plan_zero`, `cmOnCmd_of_plan_one`).
- The lookups (T6; `transforms.hpp`): for the scan N23 or N24; for every
  enabled topic zone whose stored message is in another frame N34. Each with
  upstream's wait (N26): if the transform cannot be had at once, the callback
  sleeps 10 ms of wall time at a time while the node clock is before the
  time the lookup began plus `transform_tolerance`, trying again each time,
  and tries once more at the end; and the wait ends at once when the clock
  is more than 3 s before the time the lookup began (tf2's second
  condition, N26: `wait_goes_on` in `transforms.hpp`). The loop is the
  node's own (`wait_for`), not `tf2_ros::Buffer::lookupTransform` with a
  timeout: under simulated time rclcpp's clock cannot advance inside a
  callback of this node, which has no clock thread (D15), so while it waits
  the node takes the `/clock` messages of its own subscription by hand, and
  the clock of the wait is the time of the newest of them (it can go
  backwards). For the same reason the node clears the TF buffer itself when
  a `/clock` message carries an earlier time than the one before it (N28):
  tf2's own hook would run only after the callback, and would then also
  forget transforms that arrived after the jump, which upstream keeps; so
  the buffer is created with a clock of its own, which never jumps, and
  `on_clock` calls `clear()` (T6 c). If the
  scan's lookup fails and the source timeout is not 0, the zones' lookups
  are not made (upstream does not reach its zone loop; the core leaves the
  shapes as they are).
- `points`: 16 bytes per reading of the stored scan, the patterns of `x` and
  `y` in the base frame (above); `has_points` is 0 if the scan's transform
  was not looked up or the lookup failed. `zone_verts`: for each zone in
  order a 4-byte little-endian count `n` and `16·n` bytes of vertices, with
  `n = 0xFFFFFFFF` for "none"; the array ends after the last zone that has
  vertices, and a zone it does not reach has none.
- `on_cmd_no_clock` (task T03, D15) is called instead of `on_cmd` while the
  node has no valid clock reading (T9). It has no time argument. It
  delivers the model's transition `stepNoClock`: in the active phase a
  finite command is answered with kind 4, "publish zero, invalid source",
  whatever the configuration, the stored scan and the zones, and the zero
  is never withheld (`noClock_velocity`, `cmOnCmdNoClock_kind`); a command
  that is not finite is ignored as always (R2). The record of published
  zeros of R15 is reset, so the first command with a clock reading again has
  its velocity published (`velocity_after_noClock`); nothing else in the
  state changes, and no later decision does (`evaluate_after_noClock`).
- **Two ways to compute a command (task T03).** `step` is written to be
  reasoned about and is slow when run as it stands. `roslean_cm_on_cmd` is
  `cmOnCmdFast`, which calls `stepCmdFast`
  (`core/ROSleanCore/CollisionMonitorFast.lean`): a coordinate's eight
  bytes are read into a machine word and converted there (as a `Nat`, a
  pattern with the sign bit set is a big number in Lean's runtime), the
  points are produced once and in one pass, the edges of a polygon are
  computed once per zone and command, and the band test of D4 is made only
  for points within 2 mm of the polygon's bounding box.
  `ROSlean/CollisionMonitor/Fast.lean` proves that this is the same
  function: `stepCmdFast_eq` (the fast step is `step` on the command event,
  for every configuration, state, time, velocity and coordinates) and
  `execFast_eq` (after any sequence of scan and command calls, from any
  monitor, the exported functions have produced exactly the monitor that
  `cmOnScan` and `cmOnCmd`, on which every other theorem is stated,
  produce). The coordinates of a command are read from the bytes of
  `points` (`pointAt`, `coordMmAt`) and are never decoded into a list; the
  zone vertices, which are few, are. `roslean_cm_on_cmd_reference` is
  `cmOnCmd` itself, exported so that `test_cm_ffi` can run the two side by
  side (9,600 commands with scan coordinates, and 3,000 with random
  coordinates, shape messages and zone vertices, well formed and not; none
  differs); the node does not call it.
- **Cost of a command, and the budget.** Per command, back to back, two
  zones of either kind: at most 50 µs with 360 stored readings, 150 µs with
  1080 and 1.5 ms with 10,000 (about 140 ns per reading). The test
  `roslean_cmd_cost` asserts it; the measurements are in
  [COMPARISON.md](COMPARISON.md), "Cost of a command". When a scan arrives
  whose size, with the configured zones, makes the estimated cost of a
  command exceed 1 ms, the node warns once (again for a larger scan). The
  warning changes nothing the node does.
- The six `uint64` are the bytes of the six `double` components of the twist
  (`memcpy`). The shim does not test them.
- `out_kind`: 0 `silent` (not a processed command: nothing is published, on
  any topic); 1 `pass`, 2 `slowdown zone ratio`, 3 `stopZone zone`,
  4 `stopSource` (`answer (some a) _`); 5 a processed command whose velocity
  is withheld (`answer none _`, R15). `out_zone` is the zone index for kinds
  2 and 3 and 0 otherwise; `out_ratio` is the ratio pattern for kind 2 and 0
  otherwise. `out_report`: 0 no state message; 1 `normal`; 2 `stop`;
  3 `slowdown`; 4 `source`; the zone index in `out_report_zone` for 2 and 3.
- The shim then: publishes the state message if `out_report` is not 0 and
  `state_topic` is set (1 → `("", 0)`; 2 → `(<name>, 1)`; 3 → `(<name>, 2)`;
  4 → `("invalid source", 1)`), **before** the velocity (N13); publishes the
  velocity as the table in "Where the velocity is computed" says: none for
  kinds 0 and 5; for kind 2 it copies `out_ratio` into a `double` and
  multiplies (`apply_answer` in `conversions.hpp`); and publishes the
  visualised polygons for kinds 1 to 5 (N16). A kind outside 0 to 5 cannot
  be returned (`kindOf`); were it, the shim publishes zero and logs an
  error, not silence. `smoke_conversions` checks this for every value from 6
  to 255. The shim keeps no flag "active" of its own: that nothing is
  published while the node is not active is kind 0.
- Logging. Each *change* of the answer is logged once, as upstream logs each
  change of its action and in upstream's words: "Robot to continue normal
  operation", "Robot to slowdown for … percents due to <zone> polygon",
  "Robot to stop due to <zone> polygon" (INFO), "Robot to stop due to
  invalid source" (WARN). The text is built from the three numbers Lean
  returned, and "change" is an integer comparison of those numbers with the
  ones logged last. A command that gets no answer (kind 0) is logged at
  debug level and is not a change. The numbers logged last are the only
  thing the shim remembers between events; nothing but the logging reads
  them, and no decision depends on them.

### Queries for tests and logs

`roslean_cm_coord_mm`, `roslean_cm_vertex_coord_mm`, `roslean_cm_radius_mm`, `roslean_cm_vertex_mm`,
`roslean_cm_timeout_ns` and `roslean_cm_source_timeout_ns` expose the
conversions on their own. The node decides nothing with them; the tests
compare them with reference computations.

### Theorems about the interface

`ROSlean/CollisionMonitor/FFI.lean` and `Convert.lean`; listed in
`THEOREMS.md`. In short: the configuration is what the builder calls say; a
refused value makes the monitor not ok, and a monitor that is not ok stops
every command, wherever in the sequence of builder calls the refused value
occurs (`not_ok_of_refused_call`); the decoding of each of the four byte
arrays is total and inverts the shim's encoding; every lifecycle, scan,
shape and command call is `step` on the event it delivers, so any sequence
of calls yields `run` of those events from the initial state
(`foldl_exec_run`); the plan of a command says truthfully when the arrays
do not matter; and the five numbers read back after a command determine the
model's whole output, report included (`cm_readBack_after`, and
`cm_readBack_after_fast` on the functions C calls).

## Excluded features

What the Lean node does when a configuration asks for a feature outside the
subset. "Refused" means: the `configure` transition fails with one error
message that names every refused parameter, before any subscription is
created; the node stays unconfigured and the process stays up (N3).

| Feature | Requested by | Lean node |
| --- | --- | --- |
| Several sources, or none | `observation_sources` with other than one name | Refused |
| Point cloud, range, polygon sources | `<source>.type` other than `scan` | Refused |
| Costmap source | not in Jazzy | — |
| `approach`, `limit` actions | `<zone>.action_type` | Refused |
| Zone that does nothing | `action_type: none` | Refused |
| Velocity polygons | `<zone>.type: velocity_polygon` | Refused |
| Dynamic radius | `radius` absent on a circle (upstream then needs `polygon_sub_topic`, N36) | Refused (ND12). A polygon without `points` is fed by `polygon_sub_topic` (N31), as upstream; without that parameter it is refused, as upstream, with the message of ND11. `polygon_sub_topic` next to `points` is ignored, as upstream |
| Footprint subscription | only with `approach` | Refused with it |
| Zone bound to a subset of sources | `<zone>.sources_names` other than the one source | Refused |
| Deprecated `max_points` | `<zone>.max_points` set | Refused (D10) |
| Component | loading the node into a component container | Provided as `roslean_collision_monitor::CollisionMonitorNode` (library `collision_monitor_component`; tests `smoke_component`, `smoke_component_single`, `smoke_component_mt`), without the handling of D15, which lives in the stand-alone executable's `main` (ND10) |
| Toggle and run-time parameters | `~/toggle`; setting a parameter while the node runs | Not provided; a change is rejected (ND8) |
| The removed sensor pose | `<source>.roslean_sensor_pose` | Refused, with a message that points to TF |
| Zone names the state topic cannot report | two zones of one name, a zone named `invalid source`, an empty name | Refused (ND5) |
| Collision point markers | always present upstream | Not provided (ND9) |
| Real-time priority | `use_realtime_priority: true` | Refused |
| `footprint_topic` on a zone that is not `approach` | Nav2's default zone with `action_type` changed | Not read, as upstream (N29); the zone then needs `points` or `polygon_sub_topic` (SPEC_NAV2.md, 8.6) |

## Deliberate differences in ROSlean

| # | Upstream | ROSlean | Why |
| --- | --- | --- | --- |
| D1 | Features in the table above | Refused or ignored as stated | Scope. |
| D2 | Lifecycle node with a bond; `~/toggle`; `enabled` changeable at run time (R19) | (Replaced by task T06.) A lifecycle node with the same transitions and the same bond (`nav2_util::LifecycleNode`; N2–N10). What remains: no toggle and no run-time parameter changes (ND8), and after shutdown from the active state the node is silent (ND4) | Scope; SPEC_NAV2.md, section 6. |
| D3 | Points in `double` metres; zone tests in floating point, the polygon test with a division | Points and shapes in integer millimetres; tests exact, the division multiplied out | No floating point in the core. A point within a few millimetres of a boundary can be classified differently; the direction is fixed by D4. |
| D4 | Zone is the configured shape | Circle radius rounded up plus 1 mm; polygon with a band of √5 mm around its boundary | Makes the rounding of D3 err only towards "inside" ("Safe direction"). Zones grow by up to 3.5 mm (circle) or 4.4 mm (polygon). Upstream's uneven treatment of polygon boundary points (R10) disappears: all boundary points are inside. |
| D5 | Any `slowdown_ratio` accepted; negative ones reverse the command or do nothing, `-0.0` gives zero | A ratio with the sign bit set is refused at start-up | The reversal is a hazard, and "no amplification" is false with it. The model would treat such a ratio as never selected; refusing avoids a silent difference. Ratios of 1 or more, infinite or NaN are accepted and never take effect, as upstream. |
| D6 | Slowdown selection and the zero test use the computed floating-point velocity | Selection by exact order of the ratios; zero iff stop, or the command is zero, or the selected ratio is 0 | The core does not compute velocities. Three differences. (a) When a product or squared norm underflows or overflows (command components below about 10⁻¹⁶² or above 10¹⁵⁴ in magnitude, or a ratio below about 10⁻¹⁶² with an ordinary command), or two ratios are within rounding of each other: upstream then passes the command unscaled or picks the other zone, and a product that underflows to zero is not treated as zero for R15. (b) For a command that is zero in the three components, in a triggered slowdown zone: upstream takes no action (`0 < 0` is false), the model returns `slowdown k r`. The published bits are the same (zero), but the action, and so `out_kind` and any log line, differ. That `evaluate` does not depend on the command is therefore a difference from upstream, not one of its rules. |
| D7 | The clock is read at the start of `process` and again in `publishVelocity` | One reading per event, taken before the lookups (ND2) | A decision is a function of (state, event). Differs by the run time of the callback, which now includes any wait for a transform (up to `transform_tolerance` per lookup); only for the boundary of `stop_pub_timeout`. |
| D8 | A scan stamp with negative seconds ends the process with an uncaught exception when the next command arrives | The stamp is taken as 0: under a non-zero timeout the scan is too old once the node clock is past the timeout (`0 + source_timeout < now`) | The node must not crash on input. The scan is **used** with timeout 0, and also while the node clock is still at or below the timeout (simulated time shortly after start: with `source_timeout: 2.0` and the clock at 1.5 s the scan counts as fresh). The shim's sum does not detect every stamp upstream refuses (`sec = -1` with `nanosec ≥ 10⁹` is positive); see REVIEWS, CM1/CM2 finding 9. |
| D9 | Negative timeouts accepted (a negative `source_timeout` makes nearly every scan too old; a negative `stop_pub_timeout` publishes zero once); non-finite or huge ones are undefined behaviour in the conversion | Negative, non-finite, and timeouts of 9.2·10⁹ s or more are refused, in Lean (`timeoutNs`); the core returns a stop for a refused timeout only in the active phase | Keeps times in `Nat`; avoids undefined behaviour. A refused value never becomes 0, which would switch the source check off. Accepted values are upstream's, to the nanosecond, including the default of `<source>.source_timeout` (R17). |
| D10 | `max_points` overrides `min_points`; `min_points` read into a 32-bit `int`, so `min_points: 4294967297` is 1 | `max_points` refused; `min_points` outside −2³¹ … 2³¹ − 1 refused | One rule for the threshold in the core. Inside the range the two agree; outside it upstream's value is the wrapped one, and taking the number at face value would make a zone that triggers upstream never trigger here. |
| D11 | Scan points transformed to `base_frame_id` by TF, by default with correction for the robot's motion since the stamp; a failed lookup stops the robot (R8) | (Replaced by task T06.) The same: TF is used as upstream uses it, base shift correction included (N22–N27), and a tilted sensor is projected as upstream projects it (N25). The transform is applied in the shim, never fused (ND1). `<source>.roslean_sensor_pose` is removed and refused | SPEC_NAV2.md, sections 5 and 6. |
| D12 | Coordinates unbounded | Coordinates saturate at ±1000 m; zones must lie inside | Bounded integers at the boundary; see "Safe direction". |
| D13 | Publishes zone shapes, collision point markers and, if `state_topic` is set, the current action | The state topic and the zone polygons are provided (N12–N16). Only the point markers are not (ND9) | Display only. |
| D14 | A configuration error fails the lifecycle transition; the process stays up, unconfigured | (Replaced by task T06.) As upstream: `configure` fails, the node stays unconfigured, the process stays up | — |
| D15 | Under `use_sim_time`, a `/clock` message with a negative time (`sec < 0`) ends the process: rclcpp's time source throws "cannot store a negative time point in rclcpp::Time" in its clock thread, where nothing catches it (seen on the binary, 1.3.12, signal 6) | The node goes on. It runs without rclcpp's clock thread (`use_clock_thread(false)`), so the message is taken in by the executor that spins the node and the exception arrives in `main`, which spins again. While the `/clock` message received last is such a one the node has no valid clock reading (T9), and every command goes to the core as "without a clock reading" (`roslean_cm_on_cmd_no_clock`, the model's `stepNoClock`): a finite command is answered with **zero velocity** (the stop for an invalid source), whatever the configuration, the scan and the zones, also with `source_timeout: 0` and with the source disabled, and the zero is not withheld after `stop_pub_timeout`. A command that is not finite is ignored (R2). With the next usable `/clock` message commands are evaluated as before; the period of R15 starts anew. Scans keep being stored meanwhile | No topic content may end the node (as D8). Without a clock neither the age of a scan nor the period of R15 can be judged, so the answer is the stop, for every configuration. Not a lifecycle matter and not an `Event` of the model: `stepNoClock` is a transition beside `step` (THEOREMS.md, "A command without a clock reading"). The difference is observable only after a message that kills upstream. Test: `roslean_fault_clock_negative`. **Promised for the stand-alone executable only**: the handling is in its `main`; loaded as a component the node ends with its container on such a message, as upstream's component does (ND10; decided: "as upstream" for the component). |
| ND1 | The transform is applied in `double`, with or without fused multiply-add depending on the build (N25); the arm64 binary fuses | Applied in `double` by the shim, by the same formula, **never fused** (`-ffp-contract=off`, decided by the user); then truncated to millimetres in Lean (`coordMm`) | SPEC_NAV2.md, section 5. Before truncation the two are equal where upstream's build does not fuse, and differ by an estimated 10⁻⁸ mm at most where it does; after it, as D3 and D4 already say. |
| ND2 | The clock is read at the start of `process` and again in `publishVelocity`, after any wait for a transform (N28) | One reading per command, taken before the lookups | Extends D7. The two readings can now differ by up to `transform_tolerance` per lookup that waits, instead of by the run time of the callback. Affects only the boundary of `stop_pub_timeout`, and only for commands that waited. |
| ND3 | The transform of a topic zone's shape in another frame is looked up when the zone loop reaches the zone (N34) | The shim looks up the transforms of all such zones before the command is evaluated, and the model refreshes all enabled ones | The lookups cannot be interleaved with a decision taken in Lean in one call. Differences: lookups, and their waits, for zones behind a triggered stop zone, so that **a stop by an earlier zone is published later than upstream publishes it, by up to `transform_tolerance` for each such lookup that fails** (review CM5, finding 3d; from source, not run); and the vertices of such a zone are refreshed where upstream leaves them until the zone is next reached. No command is decided differently unless a later lookup for that zone fails. |
| ND4 | Shut down from the active state, the node keeps answering commands (N8) | After `shutdown` nothing is published (E5) | A finalized node that drives the robot is a defect. Nav2's manager never takes that path (N8). |
| ND5 | A state message is published when the acting zone's **name** changes (N13, N15) | When the report changes, which identifies a zone by its position. A configuration in which two zones have the same name, or a zone is named `invalid source` or has an empty name, is refused | Only then can the two rules differ. |
| ND6 | A shape message is taken whatever its coordinates. With a NaN in a vertex the zone catches some points and misses others, by accident of the arithmetic (N33) | A message with at least three vertices of which one, in the base frame, is not finite or lies beyond the coordinate limit of D12 (999.997 m from an axis) is not stored as a shape. It leaves a mark in the model's state (`TopicShape.unusable`): the zone has no vertices and **demands its action for every command while it is enabled** (a stop zone stops, a slowdown zone slows down), whatever the points and whatever `min_points`, until a usable message for the zone arrives or the node is cleaned up. Deactivation does not lift the mark, and neither does a message that is ignored (fewer than three points, no transform). The zone's published polygon is empty meanwhile | Integers cannot hold such a vertex, and what upstream's zone does with it is not a rule that could be reproduced on purpose. Never less restrictive than upstream for that zone: with the mark the zone is triggered whenever it is enabled (`triggered_of_unusable`), hence whenever it would be with any shape (`triggered_unusable_of_triggered`); an enabled stop zone with the mark stops every command (`evaluate_isStop_of_unusable`, `run_stop_after_unusable`); when the mark is set: `acceptShape_unusable_iff`. More restrictive than upstream, which may pass where this node stops (scenarios `polygon-topic`, `polygon-topic-nan`). Until the review CM5 (finding 1) the node ignored such a message and kept the zone as it was, and passed where upstream stopped. Confirmed by the user on 2026-11-25 (tasks/PLAN.md, "Decisions"). Upstream's side as a hardening note: `findings/polygon_nan_vertex.md` |
| ND7 | tf2 does not return a non-finite or all-zero quaternion, by N25 [S] | Such a transform counts as a failed lookup: the **shim** checks the seven numbers before it uses them and hands over "none" (T6; trusted and tested there, since the model no longer sees the transform) | Totality; fail closed. Not reachable through tf2 as read. Without the check a NaN would become a saturated point, which lies in no zone. |
| ND8 | `~/toggle`; `enabled` of a zone or source changeable at run time (R19) | Not provided; an attempt to change a parameter at run time is refused | Scope. Was part of D2. |
| ND9 | `~/collision_points_marker` (N17) | Not provided | Display only. Under option A the shim has the base-frame points and could publish them; still out of scope. |
| ND10 | Loadable as the component `nav2_collision_monitor::CollisionMonitor` (N18) | Loadable as the component `roslean_collision_monitor::CollisionMonitorNode` (library `collision_monitor_component`), and as the stand-alone executable `collision_monitor`. Observed (test `smoke_component`, macOS): in `component_container_isolated`, in one process with upstream's monitor and Nav2's lifecycle manager, the node is configured, activated and bonded by the manager and answers commands. The same in `component_container` (one thread for all nodes) and `component_container_mt` (tests `smoke_component_single`, `smoke_component_mt`, T09); every thread that calls into Lean is prepared for it (T10). **In a container the handling of D15 is not there: D15 is promised for the stand-alone executable only** (decided: "as upstream" for the component): an exception that leaves a callback (a `/clock` message with a negative time) is the container's to catch, and Nav2's containers do not. The package `nav2_collision_monitor` in `overlay/opt_in/` (not part of the normal build; [DROP_IN_COLLISION_MONITOR.md](../DROP_IN_COLLISION_MONITOR.md)) compiles the same source into a library of its own that registers the class under upstream's name, `nav2_collision_monitor::CollisionMonitor`, as well; `roslean_collision_monitor` does not, because two libraries that register one name cannot be loaded into one process, which is what `smoke_component` does with upstream's | See SPEC_NAV2.md, 8.5. The stand-alone executable is the form that has every property claimed. |
| ND11 | A zone without `points` and with an action other than `approach` and no `polygon_sub_topic` fails configuration with a message about `polygon_sub_topic` (N29) | The same failure, and if `footprint_topic` is set the message says that `footprint_topic` is read for `approach` only and names the two ways out. If the zone has `points` that cannot be used and no `polygon_sub_topic`, upstream gives the same message about `polygon_sub_topic`; this node names `points` and says what is wrong with them. `configure` fails in both | The edit users will try first (N29). |
| ND12 | A circle's radius can arrive on a topic (N36) | Refused | Scope. |
| ND13 | At a command, the vertices of a topic zone whose message is in another frame are recomputed under the newest transform whatever the result (N34) | If one of the recomputed vertices is not finite or lies beyond the coordinate limit (999.997 m from an axis), or their number is not the shape's, the zone stays where it was for that command (`refresh_of_refused`) | Integers, D12. tf2 holds no transform with a NaN (N25), so this needs the zone's frame to be about a kilometre from the base. Upstream's zone is then a kilometre away; this node's is where it last was, which catches the points near the robot that it caught before and misses points a kilometre away that upstream's would catch. Unlike ND6 the zone does not demand its action. Recorded by the review CM5 (finding 3e), not changed |

ND1 to ND13 are the differences of the Nav2 extension (written by T04 as
section 6 of SPEC_NAV2.md; ND3, ND6, ND10 and ND11 amended and ND13 added by
T09 after the review CM5).

Everything else is reproduced, including R1 (no output without commands),
R4 (an invalid scan is free space), R7 (age by header stamp), R8 with
timeout 0, R11 with `min_points <= 0`, R15 and R16; and of the extension:
the lifecycle transitions and what each creates and destroys (N2–N7); the
bond (N9); that commands are dropped while inactive and scans and shapes
are stored; the state topic's contents and timing (N12–N14) apart from ND5;
the polygon publishers (N16), including that a disabled zone is not
published; which transform is looked up, at which times, with which
timeout, and the wait with both its conditions (N23, N24, N26); that a
backward jump of the clock clears the TF buffer when it happens (N28); the
projection (N25); that a failed lookup stops only with a non-zero
`source_timeout` (N27); which parameter a polygon's shape comes from,
including the fall-back from unusable `points` to the topic (N29); that a
topic zone without shape takes no action (N32); the three-point minimum and
the handling of a message whose transform fails (N33); that a shape in
another frame follows the newest transform and is left behind when its
publisher stops (N34).

## Model assumptions

- A1. Events are delivered one at a time, each with one node-clock reading
  (R19, D7).
- A2. Nothing is assumed about the clock being monotonic, nor about stamps
  being ordered. If the clock jumps backwards, a stored scan is not too old
  and zero keeps being published, as upstream. A `/clock` message with a
  negative time is not a clock reading at all: D15.
- A3. Stopping the robot when the node publishes nothing is the base driver's
  job. The node is silent when no command arrives (R1), when a command is not
  finite (R2), and once zero has been published for `stop_pub_timeout` (R15).
  A base that keeps executing its last command needs a watchdog (PLAN, lesson
  16).
- A4. The TF tree is right and timely (T6), and the sensor's
  clock agrees with the node's well enough for R7 to be meaningful.

## Open decisions

For the user. Each is implemented as upstream does it until decided.

1. **Fail-closed on invalid scans.** Upstream skips invalid readings and
   never rejects a scan (R4). A fail-closed rule would add a predicate
   `Scan.malformed` and make `sourcePoints` return `none` (stop, named
   `invalid source`) for a malformed stored scan, regardless of
   `source_timeout`. What it would change depends on what "invalid" means:
   - *Structural only*: empty `ranges`; `range_min` or `range_max` NaN, or
     `range_min > range_max`; non-finite `angle_min` or `angle_increment`;
     `ranges.size()` inconsistent with `(angle_max − angle_min) /
     angle_increment`; negative stamp. No effect on a healthy sensor. The
     length check needs a tolerance, since drivers differ by one.
   - *Any NaN reading*: REP-117 defines NaN as an erroneous reading, but many
     drivers and simulators emit NaN or 0 for "no return"; the robot would
     stop in open space with those.
   - *Any reading outside the limits*: `+inf` is the standard "nothing in
     range" value, so nearly every real scan would stop the robot. Not
     usable.
   - A threshold, e.g. fewer than a configured fraction of valid readings.

   In each case: a new numbered difference; Pass-through gets the extra
   hypothesis "the stored scan is well formed"; Bad data becomes "a malformed
   scan stops the robot"; the comparison with upstream must leave such scans
   out. The shim would pass `angle_min`, `angle_max`, `angle_increment` as bit
   patterns and the length, so that the test is in Lean.
2. **No output without commands.** Upstream has no timer (R1). A timer that
   publishes zero when the scan is too old or a stop zone is occupied, also
   when no command arrives, would be a deliberate difference and a new event
   kind. Same question as the twist_mux stretch goal "silence leads to zero".
3. **Zero is published only for `stop_pub_timeout`** (R15). As upstream, the
   Stop property ends in "zero or nothing". Publishing zero for as long as a
   stop is demanded would make it "zero"; it is a difference.
4. **Age by header stamp** (R7). A sensor clock ahead of the node's defeats
   the timeout. Measuring from arrival time instead, or taking the older of
   the two, is a difference; the scan event already carries the arrival time.
5. **`source_timeout: 0` and `<source>.enabled: false`** switch the
   protection off. Accept as upstream (now, with a warning at start-up), or
   refuse at start-up. The same question for the other configurations the
   node warns about: no zones, a circle of radius 0, a polygon without area,
   a slowdown ratio of 1 or more.
6. **Pass-through forwards three components.** The task statement asks for
   the received message unchanged; upstream publishes a new message with
   `linear.z`, `angular.x`, `angular.y` set to 0 (R16), and the spec follows
   upstream. Forwarding all six is a difference visible only to robots that
   use those components.
7. *Closed by the Nav2 extension (task T06).* `base_shift_correction` is
   read with upstream's default, the transform comes from TF, and
   `<source>.roslean_sensor_pose` is gone. Whether a scan can be placed is
   decided per command: the command carries its points or none.
8. **Zone growth of D4.** Up to 3.5 mm and 4.4 mm. The alternative without
   growth is a model that can miss a point within about 2 mm inside a
   boundary.
9. **`min_points <= 0`** makes a zone trigger always, as upstream (now with
   a warning at start-up). Refusing it at start-up is the alternative.
10. *Closed by the Nav2 extension (task T06).* `state_topic` and
    `visualize` are provided.

## Note on testing against upstream

The comparison was made and is recorded in [COMPARISON.md](COMPARISON.md):
the node through the fixed scenarios of the black-box harness with every
expected difference stated by its number here, a randomised differential
test against upstream's binary judged by the bounds of D4, fault injection,
and run-time measurements.

How to set the two nodes up for a comparison on the same messages: the
same parameter file and the same lifecycle transitions for both; nothing
else (the restrictions `base_shift_correction: false`, scans in the base
frame and an empty `state_topic` went with task T06).
Differences then occur only where D3, D4 and D6 say: a point up to 3.5 mm
(circle) or 4.4 mm (polygon) outside a zone when it decides whether
`min_points` is reached, always towards "inside" for this node; and commands
with components below about 10⁻¹⁶² or above 10¹⁵⁴ in a slowdown zone. A scan
stamp with negative seconds ends upstream (D8). Times agree exactly, unlike
twist_mux D2: upstream compares integer nanoseconds here; only D7 remains,
at the boundary of `stop_pub_timeout`, and it does not show under simulated
time.
