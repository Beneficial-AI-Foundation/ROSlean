# T06 — Nav2 extension: C interface and node

Branch: `claude/cm-nav2-node`. Needs: T03 and T05 merged.
Read [README.md](README.md) first.

## Read

`docs/collision_monitor/SPEC_NAV2.md` (section 8, shim boundary and node
plumbing; section 7, trusted list; N1–N36; section 6, differences; section
9, the edits to SPEC.md), SPEC.md, the model as extended by
T05, the existing interface and node, lessons 3 to 8 and 14 in `docs/PLAN.md`,
and the CM3/CM4 review in `docs/REVIEWS.md` for the kind of defect to avoid.

## Do

1. **Interface.** The calls of SPEC_NAV2.md, 8.1 to 8.3, thin wrappers
   around `step`, plus the query `roslean_cm_cmd_plan` that tells the shim
   which lookups to make. Conversions and refusals in Lean. Theorems in
   `ROSlean/CollisionMonitor/FFI.lean` tying the new calls to the model, up
   to the end-to-end statement on the call sequence the node makes; the
   list is at the end of 8.3.
2. **Lifecycle node.** The transitions as upstream (N2–N8), driven by
   Nav2's lifecycle manager, with bond (N9, N10; 8.5 proposes
   `nav2_util::LifecycleNode` as the base class). While inactive nothing is
   published; the decision that nothing is published is Lean's (the
   lifecycle events), not an `if` in C++. A configuration Lean refuses fails
   the `configure` transition; the process stays up (this replaces D14).
   Try whether the node can also be loaded as a component into Nav2's
   container (8.5); report the result either way.
3. **`state_topic`** and **`visualize`**, as SPEC_NAV2.md specifies, with the
   state built from what Lean returned.
4. **Transforms.** The lookup is made **at every command**, not per scan
   (N22): N23 without and N24 with base shift correction, with upstream's
   timeout and its wait inside the callback (N26), on a buffer whose
   listener has its own thread. A failed lookup is passed to Lean as "none"
   with the command; it is not an event of its own. Under option B, which
   SPEC_NAV2.md recommends and which holds until the user decides, the shim
   does **no arithmetic** on the transform: it copies seven numbers. The
   one file that calls tf2 (`transforms.hpp`, 8.4) has unit tests for which
   transform it asks for and for every kind of failure. `-ffp-contract=off`
   stays for `encode_scan`. `roslean_sensor_pose` is removed: the parameter
   is refused with a message that points to TF.
5. **Zones fed by a topic.** `polygon_sub_topic` on a polygon zone without
   `points`, with upstream's QoS (N31). `footprint_topic` is **not** such a
   subscription: upstream reads it for `approach` only (N29). Each message
   with at least three points becomes the shape event, after the lookup of
   N33 if its frame is not the base frame; Lean decides whether it is
   accepted. For shapes in another frame the transform is looked up again at
   every command (N34, ND3).
6. **Parameters.** Nav2's `collision_monitor_params.yaml` starts the node
   unchanged (its lists name a fixed stop polygon and a scan, N21). The
   collision monitor section of `nav2_bringup`'s `nav2_params.yaml` starts
   it after the edit of SPEC_NAV2.md, 8.6. That edit is **not**
   `action_type` alone: T04 observed that upstream itself fails to configure
   then (N29); `polygon_sub_topic` must be added, or the zone given fixed
   `points`. With `action_type` alone changed the node fails `configure`
   with the message of ND11. Everything still out of scope is refused in
   one message that names every such parameter.
7. **Tests** (`smoke_*`): each new path at node level under simulated time,
   including Nav2's `lifecycle_manager` binary driving the node with the
   bond on, a missing transform (under simulated time the stop comes only
   when the clock has advanced by `transform_tolerance`, N26), a moving
   odom frame, a shape arriving late, a state message while the velocity is
   withheld, and the `/clock` and
   performance fixes of T03 still holding. FFI tests for the new calls.
8. Docker build (`docker/Dockerfile`) for the platform you are on.

## May change

`core/ROSleanCore/CollisionMonitor{FFI,Convert}.lean`,
`ROSlean/CollisionMonitor/{FFI,Convert}.lean`, `ros/roslean_core`, the node
and its `smoke_*` tests and config, the package's `package.xml` and the
part of its `CMakeLists.txt` outside the harness block (new dependencies:
SPEC_NAV2.md, 8.5), `docker/`, SPEC.md (shim boundary,
differences, trusted list, applying the edits SPEC_NAV2.md lists),
THEOREMS.md. Not the model's event definitions (report a defect instead) and
not the harness files.
