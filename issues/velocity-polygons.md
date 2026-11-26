# Collision monitor: add velocity polygons

A proposal. Nothing here is built. The description of upstream is from `docs/collision_monitor/SPEC_NAV2.md` (the table of what is not implemented) and has not been re-read in upstream's source for this issue.

## Why

A `stop` zone looks only at where the lidar points are. While enough points are inside it, every command becomes zero: forward, backward and turning alike. A robot that has stopped in front of an obstacle cannot back away from it through the monitor.

- With Nav2's planner the robot ended standing for good in 3 of 6 simulated tours with a stop zone, with either node: Nav2's recovery behaviours (back up, spin) are zeroed too (`docs/TUTORIAL_COLLISION_MONITOR.md`, section 7.5).
- With a person at the controls and the monitor as the last node before the base, as in this project's Gazebo demo, the person cannot drive out either.

Upstream has three answers: the `approach` action, a source that bypasses the monitor during the silence after `stop_pub_timeout`, and **velocity polygons**. This node has none of them. Velocity polygons are the smallest of the three to add.

## What upstream does

A zone of `type: velocity_polygon` has several sub-polygons, each with ranges of the commanded velocity (`linear_min`/`linear_max`, `theta_min`/`theta_max`; for `holonomic` robots also a direction). For each command the zone's shape is the first sub-polygon whose ranges contain the command. If none matches, the shape of the previous command stays. The zone's action (`stop`, `slowdown`, …) is then applied as for any polygon (`src/velocity_polygon.cpp`).

So a configuration can give a forward command a zone ahead of the robot and a reverse command a zone behind it. An obstacle ahead then stops forward commands and lets a reverse command pass.

It is not Nav2's default: the package's example file defines `VelocityPolygonStop` and does not list it.

## What it does not solve

- The robot is freed only if something commands the direction that is free. Whether Nav2's planner and recoveries do so often enough to complete the tour is not known; it has to be measured.
- A sub-polygon for rotation normally surrounds the robot, so turning next to an obstacle can still be stopped. That depends on the configuration.
- It does not give `approach`'s gradual slowing.

## What this node does today

Refuses the zone: `parameter '<zone>.type': only 'circle' and 'polygon' are supported; got 'velocity_polygon'`.

## What adding it involves

| Part | Work |
| --- | --- |
| Model | A zone kind with a list of sub-polygons and their ranges. The shape tested becomes a function of the command. Today `evaluate` does not depend on the command at all; every definition and theorem that says "these points trigger this zone" gains the command as an argument. |
| Selection rule | Comparisons of `double`s. They can be done exactly on bit patterns in Lean (a signed comparison of the kind the core already has), so the selection needs no trusted arithmetic. |
| "Previous shape stays" | State: the last selected sub-polygon per zone, and what holds before the first match. To be read from upstream's source and tested on the binary, as the other rules were. |
| Holonomic robots | The direction needs `hypot` and `atan2`, which would come from the shim and be trusted. Proposed: non-holonomic first, `holonomic: true` refused with a message. |
| Shim | Parse the sub-polygon parameters; pass them as the other polygons are passed; publish the selected polygon for visualisation as upstream does. |
| Proofs | The existing theorems restated for a command-dependent shape. New: which sub-polygon is selected (if and only if); a stop zone whose selected sub-polygon is triggered stops; a command whose selected sub-polygon is not triggered and no other zone is triggered passes. |
| Tests | Scenarios beside upstream for forward, reverse, rotation, no match (previous shape), first command; the random comparison extended to velocity polygons; the bring-up tour repeated with a velocity-polygon stop zone. |
| Documents | SPEC (rules, differences), THEOREMS, the list of what is missing on the site, the bring-up package's parameter files if the tour supports a new default. |

Size: the specification estimates the non-holonomic case at about a tenth of the collision monitor work so far. That is an estimate made while writing the specification.

Cost per command: one polygon test as now, plus the selection (a few comparisons). No measurable change is expected; `roslean_cmd_cost` would confirm.

## Open questions

- Upstream's exact behaviour at range boundaries (inclusive or exclusive), with NaN in a command (this node ignores non-finite commands before the zones are consulted), and before the first match.
- Whether a zone fed by a topic can be a velocity polygon upstream.
- Whether the measured tour improves. If it does not, the argument for this feature is weaker and `approach` remains the real fix.

## Done when

- A `velocity_polygon` zone with `stop` or `slowdown` configures for non-holonomic robots and decides as upstream does in the scenarios and the random comparison, every difference a numbered one.
- The theorems above are proved with no `sorry` and no new axioms.
- The tour of the tutorial is measured with a velocity-polygon stop zone, at least five runs, beside stock `approach`, with the result stated whatever it is.

## Related

`issues/nav2-bringup-zones.md` (the tour measurements and the default zones of the bring-up package); `site/gaps.html#velocity-polygon` and `#approach`.
