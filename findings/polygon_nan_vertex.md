# Collision monitor: a polygon message with a NaN vertex gives a zone nobody specified

A note on a hardening opportunity in `nav2_collision_monitor`. A correct publisher does not trigger it. It concerns only zones whose shape comes from a topic (`polygon_sub_topic`), which Nav2's default configuration does not use.

## What happens

A shape message is stored without a check on its coordinates. `Polygon::updatePolygon(msg)` rejects a message with fewer than three points and otherwise transforms and stores every vertex ([`polygon.cpp` L531-L568](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_collision_monitor/src/polygon.cpp#L531-L568)).

The point test is a ray crossing along X+ ([`polygon.cpp` L598-L628](https://github.com/ros-navigation/navigation2/blob/c7046550547d382a5186d106b0895eda02485d7c/nav2_collision_monitor/src/polygon.cpp#L598-L628)). With a NaN in a vertex, the two edges that touch that vertex never toggle the result:

- NaN in `x`: `x_inter` is NaN and `x_inter > point.x` is false;
- NaN in `y`: both comparisons of the edge condition are false, the edge is entered, `x_inter` is NaN, same result.

The other edges still count. The zone becomes the region given by the remaining edges, which is open on one side. It is not the previous shape, not empty, and not the intended shape. Which points it catches depends on which vertex holds the NaN.

Nothing is logged beyond the usual "Polygon shape update has been arrived".

Read on `jazzy` at `c704655`. Other branches not checked.

## How realistic it is

Not very.

- It needs a zone with `polygon_sub_topic`.
- It needs a publisher that sends a non-finite coordinate: a division by zero or an uninitialised value in the node that computes the shape.
- We know of no publisher that does this.

So this is about what the node does with a careless publisher, not about a fault in Nav2.

## Why it is still worth handling

- The outcome can be less protective than the intended shape: a point inside the intended polygon passes (second case below).
- The outcome can also be more protective by accident: a point far outside stops the robot. Neither is a rule a user could rely on.
- The failure is silent: nothing in the log says the shape was unusable.
- A correct publisher never sends such a message, so a check costs working robots nothing.
- The node already rejects a message with fewer than three points, with an error.

## Seen on the binary

`nav2_collision_monitor` 1.3.12 (Jazzy, macOS arm64). One stop zone `Dyn` with `polygon_sub_topic`, `min_points: 1`, shapes in the base frame, one reading per scan, command 0.5 m/s. Intended shape: the square `(1,1), (1,-1), (-1,-1), (-1,1)`.

| Shape sent | Reading | `cmd_vel_out` |
| --- | --- | --- |
| 4th vertex `(NaN, 1)` | 0.5 m ahead, inside the square | 0.0 |
| 4th vertex `(NaN, 1)` | 5 m behind, outside the square | 0.0 |
| 1st vertex `(NaN, 1)` | 0.5 m ahead, inside the square | 0.5 |
| 1st vertex `(NaN, 1)` | 5 m behind, outside the square | 0.0 |
| the square (control) | 0.5 m ahead | 0.0 |
| the square (control) | 5 m behind | 0.5 |

With the NaN in the 4th vertex the left edge is lost and the zone is the strip `-1 < y <= 1`, `x < 1`. With it in the 1st vertex the right edge is lost and the zone is the strip `-1 < y <= 1`, `x < -1`. The strips are derived from the code; the six rows are observed.

Scenario `polygon-topic-nan` in `ros/roslean_collision_monitor/test/scenario_nav2.py`. Not built or run on `main`.

## A narrow fix

In `Polygon::updatePolygon(msg)`, reject a message in which any coordinate is not finite, with an error, as for fewer than three points. The zone then keeps its previous shape, or stays without one.

A stricter variant: treat the zone as triggered until a usable message arrives. This also covers the case where the broken message is the first one and the zone would otherwise have no shape. It is what our node does (SPEC.md, ND6); it stops a robot on one bad message, which upstream may not want as a default.
