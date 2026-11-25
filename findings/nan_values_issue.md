# Collision monitor: laser scans with unusable limits read as "no obstacles"

A note on a hardening opportunity in `nav2_collision_monitor`. A correct sensor driver does not trigger this bug, but may happen for custom drivers, nont written carefully.

## What happens

The scan source keeps a reading only if `range_min <= reading <= range_max` ([`scan.cpp` L94-L108](https://github.com/ros-navigation/navigation2/blob/64d5227338a6a8fef50bbae47e704cc9957e2baf/nav2_collision_monitor/src/scan.cpp#L94-L108)). The two limits come from the message and are not checked. If they are unusable, every reading is dropped, the scan yields zero points, and zero points means free space: the command passes and nothing is logged.

Unusable means any of:

- `range_min` or `range_max` is NaN (every comparison with NaN is false);
- `range_min > range_max`;
- both are 0, the message default when a driver never sets them;
- `ranges` is empty.

The node stops for a source only when it is stale or cannot be transformed ([`collision_monitor_node.cpp` L453-L465](https://github.com/ros-navigation/navigation2/blob/64d5227338a6a8fef50bbae47e704cc9957e2baf/nav2_collision_monitor/src/collision_monitor_node.cpp#L453-L465)). A scan that arrives on time with bad limits is neither.

Same code on `humble`, `jazzy`, `kilted`, `lyrical` and `main` (read at `64d5227`).

## How realistic it is

Not very.

- The drivers we read (`rplidar_ros`, `sllidar_ros2`, `urg_node`) set sensible limits and appear not to publish on a failed read.
- We found no driver that publishes NaN, inverted or zero limits.
- The plausible sources are outside the sensor: a hand-written or microcontroller driver that fills `ranges` and forgets the limits (they stay 0), a wrong parameter, or a filter that empties the scan. `laser_filters`' angular-bounds filter can produce a scan with no readings when its bounds lie outside the scan (read from its code, not run).

So this is about what the node does when something upstream of it is carelessly written, not about a fault in Nav2 or in common hardware.

## The case of a blocked/covered sensor
This produces values below  `range_min` and silently lets the robot drive through. It would be useful to report on that failure (or even stop in that case).

## Why it is still worth handling

- The monitor is the last check before the motors. When its input carries no information, stopping is the safer reading than "nothing there".
- The failure is silent. A robot with such a driver drives normally and its stop and slowdown zones never trigger; nothing in the log says so. With a check, the first test drive shows `invalid source`.
- It matches what the node already does for stale sources (#3880), and what the `Range` source already does for its own limits since #6231 ([`range.cpp` L172-L183](https://github.com/ros-navigation/navigation2/blob/64d5227338a6a8fef50bbae47e704cc9957e2baf/nav2_collision_monitor/src/range.cpp#L172-L183)).
- A working sensor never sends these values, so the check costs healthy robots nothing.

## Seen on the binary

`nav2_collision_monitor` 1.3.12 (Jazzy, macOS arm64). Stop circle of 0.5 m, `min_points: 1`, `source_timeout: 2.0`, command 0.5 m/s, scans at 10 Hz with current stamps, 360 readings at 0.2 m.

| Scan | `cmd_vel_out` |
| --- | --- |
| limits 0.05 / 30 (control) | 0.0 |
| `range_max` NaN | 0.5 |
| `range_min` 30, `range_max` 0.05 | 0.5 |
| both limits 0 | 0.5 |
| `ranges` empty | 0.5 |

```bash
# limits left unset: passes 0.5 although the readings are inside the stop circle
# (also run this way, with three readings and source_timeout 1.0)
ros2 topic pub -r 10 scan sensor_msgs/msg/LaserScan --qos-reliability best_effort \
  '{header: {stamp: now, frame_id: base_link}, angle_increment: 0.1, ranges: [0.2, 0.2, 0.2]}'
```

Not built or run on `main`; there it rests on reading the code.

## A narrow fix

In `Scan::getSourceData()`, report the source as invalid (return `false`) when the limits are not finite, when `range_max <= range_min`, or when `ranges` is empty. With a non-zero `source_timeout` the node then stops with `invalid source`, as it does for a stale scan.

Leave single readings alone, and also scans in which every reading is NaN or `+inf`. Those come from healthy pipelines in open space: `+inf` is the REP 117 value for "nothing in range", `laser_filters`' range filter writes NaN by default, and `depthimage_to_laserscan` starts from all NaN. Rejecting them would stop working robots, which is what happened to the `Range` source in #6669.
