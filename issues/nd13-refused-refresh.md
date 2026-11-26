# Collision monitor, ND13: a zone whose refresh is refused stays where it was. The case for keeping it

Decision of 2026-11-25 (`tasks/PLAN.md`): ND13 stays as built. This issue records why the less restrictive behaviour is acceptable and what it buys, so that the question can be closed or reopened on the arguments.

## The behaviour

A zone fed by a topic can be given in a frame other than the base frame (for example `map`). At every command the node recomputes the zone's vertices under the newest transform (N34).

If a recomputed vertex is not finite or lies beyond the coordinate limit (999.997 m from an axis of the base frame), the node cannot hold it in millimetre integers (D12). It then **keeps the zone where it was at the last successful refresh**, for that command (`refresh_of_refused`).

Upstream recomputes whatever the result: its zone is then about a kilometre away.

The alternative, by analogy with ND6: such a zone demands its action (a stop zone stops) until a refresh succeeds.

## Where the built behaviour is less restrictive than upstream

Only here: a lidar point about a kilometre from the robot, inside upstream's displaced zone, stops upstream and not this node. It is the one remaining place where the node can in principle pass where upstream stops (SPEC.md, ND13; `site/gaps.html`).

## Why it is safe

- **The points do not exist.** A 2D lidar on a mobile robot has a range of tens of metres. A reading a kilometre away is not produced by the sensors this monitor is used with; a `range_max` that large would itself be a configuration error.
- **tf2 holds no NaN.** A transform with a non-finite component is not stored by tf2 (N25), so the non-finite case does not arise through TF. What remains is distance: the zone's frame must be about a kilometre from the base.
- **A zone a kilometre away protects nothing near the robot.** Upstream's behaviour in this case is to move the zone off the robot. Whatever is in front of the robot then passes upstream's zone. This node's zone stays around the robot and still catches what it caught before. For every point within sensor range, this node is the more restrictive of the two.
- **Other zones are unaffected.** A fixed stop zone, or any zone in the base frame, works as before.

So the difference from upstream is confined to points no sensor delivers, and for the points sensors do deliver the built behaviour protects more.

## What it buys

- **No stop from geography.** A zone defined in `map` or `odom` at a fixed place is a legitimate use: a keep-out area, a dock. When the robot drives a kilometre from it, "the zone demands its action" would stop the robot for good, far from the zone, for a reason the operator cannot see on the robot. With ND13 as built the robot keeps working.
- **No stop from odometry drift or a re-localisation jump.** The same holds when the `odom` or `map` origin ends up far from the base without the robot having gone anywhere unusual.
- **A different kind of event than ND6.** ND6 marks a zone after a *message* that is broken: a fault in the publisher, rare, and a clear signal. ND13 is triggered by the *transform at command time*, with a well-formed message. Treating a far-away zone as a fault would turn a normal situation into a stop.
- **No new state.** ND6 needed a mark in the model's state and its theorems. ND13 as built needs none: the zone simply is not updated.

## What it costs

- One numbered place remains where "never less restrictive than upstream" needs a qualification, and every document that states the claim must name ND13.
- The zone stays at a stale position. If the zone was meant to move with a frame, it no longer does while the refresh is refused. I found no log message for it in the node's source.
- The argument "the points do not exist" is about sensors, not about the node: it is an assumption, and belongs with the stated assumptions in THEOREMS.md.

## Proposed resolution

Keep ND13 as built, and:

1. state the sensor-range assumption next to ND13 in SPEC.md and in THEOREMS.md;
2. log a throttled warning naming the zone when a refresh is refused;
3. add a scenario that drives the base a kilometre from a `map`-frame zone and checks that the node keeps running and the zone stays put (the review CM5, finding 3e, showed the behaviour on the Lean model in `ModelProbes.lean`; no test runs it through the two nodes).

Reopen if a configuration turns up in which points at that distance are real, for example a source other than a 2D lidar.
