# T06 — Nav2 extension: C interface and node

Branch: `claude/cm-nav2-node`. Needs: T03 and T05 merged.
Read [README.md](README.md) first.

## Read

`docs/collision_monitor/SPEC_NAV2.md` (shim boundary, trusted list, lifecycle
rules, transform rules, footprint rules), SPEC.md, the model as extended by
T05, the existing interface and node, lessons 3 to 8 and 14 in `docs/PLAN.md`,
and the CM3/CM4 review in `docs/REVIEWS.md` for the kind of defect to avoid.

## Do

1. **Interface.** Exported functions for each new event, thin wrappers
   around `step`. Conversions and refusals in Lean. Theorems in
   `ROSlean/CollisionMonitor/FFI.lean` tying the new calls to the model, up
   to the end-to-end statement on the call sequence the node makes.
2. **Lifecycle node.** The transitions as upstream, driven by Nav2's
   lifecycle manager, with bond. While inactive nothing is published; the
   decision that nothing is published is Lean's (the activation events), not
   an `if` in C++.
3. **`state_topic`** and **`visualize`**, as SPEC_NAV2.md specifies, with the
   state built from what Lean returned.
4. **Transforms.** TF lookups for each scan and base shift correction as
   specified. A failed or late lookup becomes the model's event. The
   floating-point part that stays in C++ sits in one small file with unit
   tests for its rounding and its agreement with upstream's computation.
   `roslean_sensor_pose` is kept or removed as SPEC_NAV2.md decides.
5. **Footprint zones.** Subscription with upstream's QoS; each message
   becomes the zone-shape event; malformed shapes handled by Lean's rule.
6. **Parameters.** Nav2's `collision_monitor_params.yaml` and the collision
   monitor section of `nav2_bringup`'s `nav2_params.yaml` start the node
   after the single edit SPEC_NAV2.md names (`action_type`). Everything still
   out of scope is refused in one message that names every such parameter.
7. **Tests** (`smoke_*`): each new path at node level under simulated time,
   including a lifecycle manager driving the node, a missing transform, a
   moving odom frame, a footprint arriving late, and the `/clock` and
   performance fixes of T03 still holding. FFI tests for the new calls.
8. Docker build (`docker/Dockerfile`) for the platform you are on.

## May change

`core/ROSleanCore/CollisionMonitor{FFI,Convert}.lean`,
`ROSlean/CollisionMonitor/{FFI,Convert}.lean`, `ros/roslean_core`, the node
and its `smoke_*` tests and config, `docker/`, SPEC.md (shim boundary,
differences, trusted list, applying the edits SPEC_NAV2.md lists),
THEOREMS.md. Not the model's event definitions (report a defect instead) and
not the harness files.
