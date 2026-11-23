# What upstream `nav2_collision_monitor` does

Observed behaviour of the upstream binary, as a black box, for the subset
that ROSlean will replace: one laser scan source, one velocity command
input, `stop` and `slowdown` zones, circles and fixed polygons, `min_points`,
and the source timeout.

Every finding below names the scenario in
`ros/roslean_collision_monitor/test/scenario.py` that shows it. The expected
outputs in the scenarios are what the binary published, not what the
documentation or the source says it should publish. Where the behaviour
looks like a bug or is unsafe, the scenario still expects what upstream does
and the finding is marked **(flag)**.

Not covered: point clouds, range sensors, costmap and polygon sources, the
`approach` and `limit` actions, velocity polygons, polygons received on a
topic, the `enabled` parameters, the `toggle` service, a base that moves
between the scan and the command (base-shift correction with a changing
odometry transform), and more than one source.

## What was run

| | macOS arm64, pixi/RoboStack | Linux arm64, Docker, apt |
| --- | --- | --- |
| `nav2_collision_monitor` | 1.3.12 | 1.3.14 |
| `scenario.py` on upstream: 55 scenarios, 277 checks | 6 runs, 0 failures | 3 runs, 0 failures |
| `random_diff.py`, upstream against itself and against the model | 60 seeds, 5267 commands, 0 differences | 30 seeds, 2625 commands, 0 differences |
| `scenario.py` on the reference node: 45 scenarios, 238 checks | 0 failures | 0 failures |
| `random_diff.py`, reference node against upstream | 5 seeds, 501 commands, 0 differences | 5 seeds, 501 commands, 0 differences |

All of it was run on 2026-11-23. The two upstream versions behaved
identically. The reference node is the Python model of this document as a
ROS node; see [The model](#the-model).

"Documentation" below means the Nav2 configuration guide for the collision
monitor (Jazzy branch of docs.nav2.org) and the package README.

How the tests work: the node runs with `use_sim_time`, and the test
publishes `/clock`, so the node's clock reads exactly what the test says.
The test publishes one scan or one command at a time and waits for the
answer. See [Running the tests](#running-the-tests).

## Findings that differ from the documentation, or that the source does not make obvious

### 1. A scan with a negative time stamp kills the node (flag)

A `LaserScan` whose header stamp is negative (`sec` is a signed integer, so
this is a legal message) is accepted silently. The next velocity command
makes the process abort with an uncaught `std::runtime_error`, "cannot store
a negative time point in rclcpp::Time". From then on nothing is published:
the last command that was forwarded stays the last word to the base.

Scenario `negative-stamp`. Found by the random test, not by reading the
source: the conversion that throws is implicit.

### 2. The node only speaks when spoken to (flag)

The node publishes a velocity only in answer to a velocity command, at most
one message per command. Without commands it publishes nothing, whatever
happens: the scan times out, an obstacle enters a stop zone, ten minutes
pass. If the planner dies after a non-zero command, the collision monitor
does not stop the robot.

Scenario `commands-cease`.

### 3. After `stop_pub_timeout` a stop is no longer published, and zero commands are swallowed

`stop_pub_timeout` (default 1.0 s) is documented as the time "after which
zero-velocity ceases to be published". What that means exactly:

- The clock starts when the output first becomes zero after a non-zero
  output (or at the first command after activation). It is not restarted by
  further zero outputs.
- A zero output is published while `now - start <= stop_pub_timeout`. At
  exactly the timeout it is still published; 1 ns later it is not.
- After that, commands get **no answer at all** while the output would be
  zero, however long the obstacle stays.
- The rule looks only at the output value, not at its cause. A zero command
  that merely passes through (no obstacle) is forwarded for
  `stop_pub_timeout` and then dropped as well.
- "Zero" means `linear.x == 0`, `linear.y == 0` and `angular.z == 0` in the
  output. Negative zero counts as zero. A command with only `linear.z` or
  `angular.x/y` non-zero counts as zero (see finding 6).
- Any non-zero output ends the silence, and the next zero output starts a
  new period.
- The state topic (finding 12) is not affected: a state change during the
  silence is still reported.

Scenarios `stop-publication`, `zero-input`.

### 4. With `source_timeout: 0.0`, "no usable scan" means "no obstacle" (flag)

The documentation says that `source_timeout: 0.0` "disables this blocking
mechanism". It disables more than the age check: with a timeout of zero the
robot is not stopped when no scan has ever been received, nor when the scan
cannot be transformed into the base frame. In both cases the command passes
as if the zones were empty, even if the untransformable scan is full of
obstacles.

Scenarios `timeout-zero` (first step), `timeout-zero-no-transform`.

### 5. The age of a scan is measured from its header stamp, not from its arrival

The documentation says "If no new data is received within this interval, the
robot will be stopped". What is compared is the node's clock at the time of
the command against the header stamp of the latest scan:

- A scan that arrives with an old stamp is too old at once.
- A scan stamped in the future is used until `source_timeout` after its
  stamp, however long ago it arrived.
- After a backwards jump of the clock an old scan is used again.
- A negative `source_timeout` rejects every scan that is not stamped at
  least that far in the future.
- The latest scan *received* is used, not the one with the latest stamp.

The comparison is exact, in integer nanoseconds (finding 16).

Scenarios `timeout`, `timeout-unix-clock`, `timeout-negative`, `before-clock`.

### 6. Three of the six velocity components are dropped

Only `linear.x`, `linear.y` and `angular.z` reach the output. `linear.z`,
`angular.x` and `angular.y` come out as `+0.0`, also when no zone is
triggered and the command "passes through unchanged".

Scenarios `pass-through` (first step), `slowdown`, `stamped`.

### 7. A command with a NaN or an infinity is ignored, not answered with a stop

If any of the six components of a command is NaN or infinite, including the
three that are never forwarded, nothing is published and no state changes.
This holds with an obstacle in a stop zone too.

Scenario `non-finite-command`.

### 8. A triangle is refused

The documentation says "Minimum 3 points for a triangle polygon". A `points`
parameter with three vertices is rejected (the node logs "Polygon must have
at least three points"), and the node fails to configure. Four
vertices are the minimum.

Scenario `triangle`.

### 9. `slowdown` is applied only if it makes the velocity strictly smaller

The slowed velocity is `v * slowdown_ratio`, component by component: the
output is bit for bit the double product, also for a ratio like 0.3 that
is not a binary fraction. It replaces the velocity chosen so far only if its
squared norm `x*x + y*y + w*w` is strictly smaller. Consequences:

- `slowdown_ratio: 1.0` and `2.0` have no effect, and the zone is not
  reported on the state topic.
- `slowdown_ratio: -1.0` has no effect. `slowdown_ratio: -0.5` **reverses**
  the command at half speed **(flag)**.
- `slowdown_ratio: 0.0` gives a zero output, reported as a slowdown (action
  type 2), not as a stop.
- A zero command in a triggered slowdown zone is not "slowed": the state
  goes back to normal (`""`, 0) and returns to slowdown with the next
  non-zero command. The state topic flaps with the commands.
- For a velocity so small that its square underflows (`1e-200`), the
  comparison is `0 < 0`: the command passes unslowed. `1e-150` is slowed.
- With several slowdown zones triggered, the one with the smallest
  resulting velocity wins, whatever the order of the zones.
- A triggered stop zone wins over slowdown zones, whatever the order.

Scenarios `ratio-negative`, `ratio-minus-one`, `ratio-one`, `ratio-two`,
`ratio-zero`, `signs-slowdown`, `slowdown-ratio`, `two-slowdowns`,
`two-slowdowns-reversed`, `both`, `both-stop-first`.

### 10. A missing transform blocks the node for `transform_tolerance` of *node* time

If the scan frame cannot be transformed to the base frame, the answer to a
command is a stop ("invalid source"), but it is delayed until the node's
clock has advanced by `transform_tolerance`. Under simulated time with a
clock that stands still the node does not answer at all. With
`transform_tolerance: 0.0` the stop comes at once.

With `base_shift_correction: true` (the default) a transform between
`odom_frame_id` and the base frame is needed even if the scan is already in
the base frame; without it every command is answered with a stop. With
`base_shift_correction: false` and the scan in the base frame, no transform
is needed.

Scenarios `transform-wait`, `no-transform`, `no-odom`,
`shift-correction-same-frame`.

### 11. Commands that arrive faster than they are handled are dropped

The command subscription keeps one message. Of N commands published back to
back, only some are answered, in order, and always including the last.
Measured with `probe_qos.py`, five bursts of each size:

| Burst | Answers, macOS | Answers, Linux |
| --- | --- | --- |
| 2 | 1, 1, 1, 1, 1 | 1, 1, 1, 1, 1 |
| 5 | 1, 2, 1, 1, 1 | 1, 2, 1, 2, 3 |
| 20 | 2, 2, 2, 2, 1 | 10, 3, 4, 5, 6 |
| 100 | 9, 10, 11, 5, 9 | 11, 11, 10, 11, 11 |

This is the one source of nondeterminism found; the tests avoid it by
sending one command at a time and waiting for the answer.

### 12. The state topic reports changes of the zone name only

`state_topic` is empty by default (no publisher). When set, a
`CollisionMonitorState` message is published when the name of the acting
zone differs from that of the previous command: `("<zone>", 1)` for a stop,
`("<zone>", 2)` for a slowdown, `("invalid source", 1)` for an unusable
scan, `("", 0)` on return to normal. Nothing is published for the first
command if it passes. A state change is also published when the velocity
is withheld (finding 3).

Every scenario checks the state messages; `stop-circle` is the simplest.

## Boundaries and floating point

All of these gave the same result in every run, on both platforms. They are
properties of the arithmetic, not of timing.

### 13. Which numbers are float32

- Scan ranges, `range_min`, `range_max`, `angle_min` and `angle_increment`
  are float32 on the wire.
- The angle of ray *i* is a float32 running sum: `angle_min`, then
  `+ angle_increment` *i* times, rounded to float32 each time. For a
  361-ray scan from −3.14159 in steps of 0.0174533 the last angle is
  3.1415865, where `angle_min + 360 * angle_increment` is 3.1415977: the two
  lie on opposite sides of π, so the point lies on opposite sides of the x
  axis. Scenario `angle-accumulation`.
- The point is `(range * cos(angle), range * sin(angle))` with float32
  cosine, sine and products. At angle 0 it is exactly `(float32(range), 0)`.
  At the float32 nearest to −π/2 the y coordinate of a range 0.5 is exactly
  −0.5, on the bottom edge of the square and therefore outside. With a
  double-precision sine it would be −0.4999999999999995, inside. Scenario
  `boundary-polygon`, bottom edge.
- Polygon vertices are parsed as float32: a vertex written `0.3` is at
  0.30000001192092896. Scenario `boundary-polygon-0.3`.
- The circle radius is a double: a radius written `0.7` is the double 0.7.
  Scenario `boundary-circle-0.7`.
- According to the source, the transform to the base frame and the zone
  tests are computed in double. The scenarios with an offset scan frame
  agree with that but use only transforms that are exact in both.

### 14. Circle: strictly inside

A point is in a circle if `x*x + y*y < radius*radius`. A point exactly on
the circle is **outside**.

| Radius | Range in the scan | Result | Why |
| --- | --- | --- | --- |
| 0.5 | 0.5 | outside | on the circle |
| 0.5 | 0.49999997 (one float32 step below) | inside | |
| 0.5 | 0.50000006 (one step above) | outside | |
| 0.7 | 0.7 | **inside** | float32(0.7) = 0.699999988 < 0.7 |
| 0.7 | 0.70000005 (one step above) | outside | |
| 0.3 | 0.3 | outside | float32(0.3) = 0.300000012 > 0.3 |
| 0.3 | 0.29999998 (one step below) | inside | |

So "an obstacle at exactly the radius" is inside or outside depending on
how the decimal radius rounds to float32.

Scenarios `boundary-circle`, `boundary-circle-0.7`, `boundary-circle-0.3`;
with a scan frame offset from the base: `offset`, `offset-shift-correction`,
`rotated`.

### 15. Polygon: left and top edges inside, right and bottom edges outside

For the square with corners (±0.5, ±0.5), a point exactly on an edge:

| Point | Result |
| --- | --- |
| right edge (0.5, 0) | outside |
| left edge (−0.5, ≈0) | inside |
| top edge (≈0, 0.5) | inside |
| bottom edge (≈0, −0.5) | outside |
| corner (0.5, 0.5) | outside |
| corner (0.5, −0.5) | outside |
| corner (−0.5, −0.5) | outside |
| corner (−0.5, 0.5) | inside |

One float32 step to either side of each edge gives the obvious result.
Because the vertices are float32 too, a range written `0.3` lands exactly on
the edge of a square written `0.3` (left edge inside, right edge outside),
and the same for `0.7`. A circle and a square of the same nominal size 0.7
therefore disagree about a point at range 0.7 straight ahead: inside the
circle, outside the square.

Scenarios `boundary-polygon`, `boundary-vertices`, `boundary-polygon-0.3`,
`boundary-polygon-0.7`.

### 16. Source timeout: exact, and "exactly at the timeout" is still valid

A scan is too old if `clock - stamp > source_timeout`, in integer
nanoseconds.

| Age of the scan | Result |
| --- | --- |
| `source_timeout` − 1 ns | used |
| exactly `source_timeout` | used |
| `source_timeout` + 1 ns | stop, "invalid source" |

The same at a clock of 100 s and of 1.8 × 10⁹ s (Unix scale): no rounding
is involved in the comparison. Rounding enters once, when the parameter is
converted: seconds times 10⁹, truncated. `source_timeout: 1.001` becomes
1000999999 ns (the double nearest to 1.001, times 10⁹, is
1000999999.9999999), so a scan aged 1001000000 ns is too old.
`stop_pub_timeout` (finding 3) is compared the same way, to the nanosecond.

Added after the comparison with the SPEC ([COMPARISON.md](COMPARISON.md),
scenarios in `test/probes.py`): "rounding enters once" holds for
`stop_pub_timeout` and for a timeout given to the source itself
(`scan.source_timeout: 2.094` is 2093999999 ns). The node's
`source_timeout`, when the source inherits it, is converted **twice**
(seconds to nanoseconds, back to seconds, to nanoseconds again), which for
some values loses another nanosecond: `source_timeout: 2.094` gives the
source 2093999998 ns, so a scan aged 2093999999 ns is too old. 1.001 is not
such a value.

Scenarios `timeout`, `timeout-unix-clock`, `timeout-decimal`,
`stop-publication`.

## Findings that match the documentation or the obvious reading

### 17. Pass-through, stop, slowdown

With no zone triggered, `linear.x`, `linear.y` and `angular.z` pass bit for
bit, including values like `1e-300` and `1e300` and negative zero. A
triggered stop zone gives `+0.0` on all components, whatever the signs of
the command. A triggered slowdown zone scales each component, keeping signs
(and the sign of a negative zero).

Scenarios `pass-through`, `stop-circle`, `stop-polygon`, `slowdown`,
`signs-pass`, `signs-stop`, `signs-slowdown`.

### 18. `min_points` is "at least"

A zone is triggered when at least `min_points` points are inside it (the
README says "more than N"; the configuration guide is right). Two points
with `min_points: 3` pass, three stop. Points outside the zone do not
count. With `min_points: 0` or a negative value the zone is triggered by
any usable scan, including one without points.

Scenarios `min-points`, `min-points-zero`, `min-points-negative`.

### 19. Which rays count

A ray counts if `range_min <= range <= range_max`, both ends included,
compared in float32.

| Ray | Result |
| --- | --- |
| NaN, +inf, −inf | ignored |
| one float32 step below `range_min` / above `range_max` | ignored |
| exactly `range_min`, exactly `range_max` | counted |
| 0 with `range_min: 0` | counted, a point at the origin of the scan frame |
| negative with a negative `range_min` | counted, a point behind the sensor |
| +inf with `range_max: +inf` | no effect (tried at angle 0 with the circle, at 0, 45° and ±3 rad with the square) |
| anything, if `range_min` or `range_max` is NaN, or `range_min > range_max` | ignored |

Ignored rays do not make the scan invalid: a scan of only NaNs is a valid
scan without points, and the command passes. The angle advances over
ignored rays.

Scenarios `invalid-ranges`, `invalid-ranges-polygon`.

### 20. `ranges` alone decides the number of rays

`angle_max` is never read. Five ranges where `angle_min`, `angle_max` and
`angle_increment` allow three: all five are used, the fifth at
`angle_min + 4 * angle_increment`. One range where the angles allow eleven:
one point. No ranges: a valid scan without points. A negative
`angle_increment` works. With `angle_increment: NaN` the first ray is still
at `angle_min` and counts; the second ray has no effect.

Scenarios `length-mismatch`, `length-mismatch-polygon`.

### 21. No scan yet

With a non-zero `source_timeout` and no scan received, every command is
answered with a stop and the state `("invalid source", 1)` (subject to
finding 3). Before the first `/clock` message the node's clock reads 0.

Scenarios `no-scan`, `before-clock`.

### 22. `Twist` or `TwistStamped`

Jazzy's node uses `geometry_msgs/Twist` on both the command input and the
output by default. With `enable_stamped_cmd_vel: true` both are
`geometry_msgs/TwistStamped`, and the header of the command (frame and
stamp) is copied to the output unchanged, also for a stop. The stamp of the
command plays no role in any decision. Every scenario checks the two topic
types; `stamped` runs with the parameter set.

### 23. Other observations

- `action_type: "none"` is accepted and does nothing (scenario
  `action-none`).
- Quality of service (`probe_qos.py`): scan subscription best-effort,
  command subscription reliable, both outputs reliable, all volatile.
- A scan frame offset from the base by a fixed transform, also turned by
  half a turn, gives the points the transform says, exactly for these
  transforms (scenarios `offset`, `rotated`).

## Nondeterminism

Two upstream processes that were fed the same messages published the same
answers to every command, bit for bit: 5267 commands in 60 random cases on
macOS, 2625 commands in 30 cases on Linux, none of which needed the slow
retry that `random_diff.py` falls back to when two nodes differ. The fixed
scenarios gave the same 277 results in each of 6 runs on macOS and 3 on
Linux, every boundary case included. With one command in flight at a time
and a clock set by the test, what the node publishes is a function of the
sequence of messages it has received.

One exception was seen. The random test also runs as part of `pixi run
test` (seeds 0 to 19), and in the second of two such runs on macOS seed 16
passed only on the retry: one of the two upstream processes answered one
command differently from the other and from the model. That command came
50 ms after a scan, which itself came 50 ms after another scan. With 300 ms
between events the two processes agreed. So in 100 random cases of upstream
against itself on macOS (8753 commands), one answer was given before the
latest scan had been taken in, or without it: the scan subscription is
best-effort, so a lost scan cannot be ruled out. The 50 ms that the random
test allows per event is the one wall-clock assumption of the harness, and
this is its failure rate here.

What is not deterministic, or was not tried:

- Commands sent faster than the node handles them (finding 11).
- The order in which the node sees a scan or a clock update and a command
  sent at almost the same moment, as above. The tests leave 50 ms (random
  test) or 200 ms (scenarios) of wall time between events; nothing tighter
  was tried.
- Wall-clock time. Everything here ran under simulated time.

## The model

`test/cm_model.py` is a Python model of the findings above for the subset
under test (about 100 lines of logic), and `test/reference_node.py` wraps it
into a plain ROS node without lifecycle and without transforms.

- Upstream agrees with the model on every command of every random case.
- Run through `scenario.py` in upstream's place (`--no-tf`), the reference
  node passes 45 of the 46 scenarios that need no transform. The exception
  is `negative-stamp`: it does not abort. The 45 include all float32
  boundary cases, so the precision rules of finding 13 are sufficient to
  reproduce them.
- The reference node is also how the harness was checked on a node that is
  not a lifecycle node.

Can the random test fail? Three deliberate errors were put into the model,
one at a time, and the reference node compared with upstream: `min_points`
off by one, the squared radius of circles 10 % too large, and
`stop_pub_timeout` 50 ms too long. Each was reported within four seeds. The
last two were *not* reported within four seeds by the first version of the
generator; it now aims a third of the rays within 5 % of a zone's size and
draws shorter timeouts.

## Running the tests

```
pixi run build
pixi run test            # all packages
```

The package registers four tests: `upstream_scenario`,
`upstream_random_diff` (seeds 0 to 19), `reference_scenario` and
`reference_random_diff` (seeds 0 to 4). Together they take about 11 minutes
on the macOS machine used here. The tests that run the ROSlean node through
the same harness, and `upstream_probes`, were added later; they are
described in [COMPARISON.md](COMPARISON.md).

or one at a time, with any node as the first argument:

```
source install/setup.bash
cd ros/roslean_collision_monitor/test
python3 scenario.py                                       # upstream
python3 scenario.py roslean_collision_monitor/NODE        # another node, same expectations
python3 scenario.py NODE --no-tf                          # a node that uses no transforms
python3 random_diff.py NODE nav2_collision_monitor/collision_monitor 0 20
python3 probe_qos.py NODE
```

`scenario.py --list` lists the scenarios, `--only a,b` selects some,
`--skip a,b` leaves some out, `--repeat N` repeats them, `--ignore-state` skips the state topic, and
`--logs DIR` keeps the node's output.

What the tests assume about a node, beyond upstream's parameter names:
it subscribes to `/clock` (`use_sim_time`), to the scan topic and to
`cmd_vel_in_topic`; it publishes on `cmd_vel_out_topic` and, unless
`--ignore-state` is given, on `state_topic`. If the node offers the
lifecycle service `<node>/change_state`, the test configures and activates
it; otherwise it is taken to be running (`--lifecycle yes|no` overrides the
detection). A lifecycle node refuses a
configuration by failing to configure, a plain node by exiting. With
`--no-tf` the test publishes no transforms and skips the scenarios that
need one.

The random cases stay 25 ms away from timeout boundaries and 0.1 mm away
from zone boundaries; those are the business of the fixed scenarios. They
also never stamp a scan before time 0 (finding 1).
