# Nav2 bring-up with the Lean collision monitor: no zone set behaves like stock Nav2

Measurements and an open problem. The default of `roslean_nav2_bringup` is decided (stop + slowdown); this issue records why neither zone set is satisfactory and what would settle or remove the question.

## The problem

Stock Nav2 configures its collision monitor with one zone of action `approach`: the command is scaled by the time to collision, so the robot slows smoothly near an obstacle and a command that turns away from it passes. The Lean node has `stop` and `slowdown` only (`site/gaps.html#approach`). `roslean_nav2_bringup` therefore replaces that zone, and has two parameter files to choose from:

| `zones:=` | Zones | Protection | Known cost |
| --- | --- | --- | --- |
| `stop_slowdown` (default) | a stop polygon with a slowdown polygon around it | the robot is stopped before the footprint | every command is zeroed while a point is in the stop zone, including the turn or retreat that would clear it and Nav2's recovery behaviours |
| `slowdown` | the slowdown polygon alone | commands are scaled by 0.4 near obstacles; nothing stops the robot | the monitor never prevents contact |

## Measured

Nav2 planning and driving a tour of the demo apartment for 300 simulated seconds per run (`nav2_observe.py tour`, Gazebo, TurtleBot3, macOS arm64, one Apple M5 Pro; TurtleBot3-sized zones of the demo, not the package's example zones for a 0.22 m robot).

| Zones, node | Run | Places reached | Distance | Monitor answered a non-zero command with zero or nothing | At the end | Closest |
| --- | --- | ---: | ---: | ---: | --- | --- |
| slowdown, Lean | 1 | 3 | 21.3 m | 0 s | standing; localisation 3.7 m off, planner found no path | lidar at its lower limit of 0.12 m; outline over map-occupied cells for 50 s |
| | 2 | 1 | 14.6 m | 0 s | 0.3 m from a goal, 0.50 m in the last 30 s; planner failed 88 times | outline 0.10 m from the nearest occupied cell |
| | 3 | 6 | 27.9 m | 0 s | driving | lidar at its lower limit; outline 0.01 m from the nearest occupied cell |
| stop + slowdown, Lean | 1–3 | 1, 1, 8 | 10.1, 10.8, 38.6 m | 51.5, 39.0, 0 s | stood for good in two | not recorded |
| stop + slowdown, upstream's node | 1–3 | 8, 2, 5 | 38.9, 13.5, 28.9 m | 0.5, 11.0, 0.5 s | stood for good in one | not recorded |
| `approach`, upstream's node (stock Nav2) | 1 | 11 | 47.4 m | 0.5 s | driving | not recorded |

The slowdown runs were made on 2026-11-25 through `roslean_nav2_bringup`'s `bringup_launch.py`; the other rows are the earlier runs of the collision monitor tutorial, section 7.5.

What the numbers say:

- With slowdown only the monitor never held the robot. Two of three runs still ended stuck, for Nav2's own reasons (lost localisation; no plan), with the robot close to obstacles.
- With the stop zone the robot ended standing for good in three runs of six, with either node.
- Neither comes near the one stock run.

## Not known

- **Contact.** The simulation has no contact sensor. "Outline over map-occupied cells" is computed from the true pose and a map with 5 cm cells; the lidar reports nothing below 0.12 m, where the body is 3 to 5 cm from the obstacle. No run had a second in which the monitor published at least 0.03 m/s and the robot did not move.
- **A baseline for closeness.** No stock or stop-zone run recorded these figures, so it is not known whether slowdown-only comes closer to obstacles than stock Nav2 does.
- **Spread.** Three runs per variant, one stock run, one machine, under varying load. Places reached range from 1 to 8 within one variant.
- **Other robots and zone sizes.** Everything used the TurtleBot3 and the demo's zones.

## What would settle the default

1. The same tour, on a machine that stays awake, at least five runs each of: stock `approach`, `stop_slowdown` and `slowdown` with the Lean node, all recording the closeness figures.
2. A contact sensor on the robot model, or ground-truth distance to obstacle geometry instead of the map.
3. The package's own example zones on a robot of that size.

## What would remove the problem

An `approach` action in the Lean node. It is the largest missing item: a simulation of the robot's motion with trigonometry inside the decision, and a computed scale factor instead of one of three cases (`site/gaps.html#approach`).

A smaller step that may help the stop zone: Nav2's recovery behaviours publish on the same velocity path and are zeroed too. Whether a narrower stop zone, or one that lets pure rotation pass, would free the robot without giving up the stop is untested, and the second would be a change of the node's rules, not of its parameters.

## Done when

- The table above has the runs of item 1 with closeness for every row, and the default is confirmed or changed on that basis; or
- the node has `approach` and the bring-up package uses Nav2's parameters unchanged.
