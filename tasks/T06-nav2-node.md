# T06 — Nav2 extension: C interface and node

Branch: `claude/cm-nav2-node`. Needs: T03 and T05b merged.
Read [README.md](README.md) first.

**The transform is applied in the shim (option A; corrected by T05b).** The
user decided on 2026-11-24 that the sensor-to-base transform is computed in
C++ in `double`, as upstream computes it, and that Lean receives points.
Task T05b took the transform out of the model. Where this brief said "the
shim does no arithmetic on the transform", the opposite now holds: this
task writes that arithmetic, and it is part of the trusted base
(SPEC_NAV2.md, section 5 and T2). **Compiler flags: `-ffp-contract=off` on
every platform, decided by the user**; it is not an open point and is not
to be chosen per platform. Upstream's arm64 binary fuses the two sums of
N25, so there the last bits of a coordinate can differ from upstream's;
that is ND1, and the margin theorems leave 0.04 mm for it.

## Read

`docs/collision_monitor/SPEC_NAV2.md` (section 8, shim boundary and node
plumbing; section 7, trusted list; N1–N36; section 6, differences; section
9, the edits to SPEC.md), SPEC.md, the model as extended by
T05 and reworked by T05b (`tasks/reports/T05b.md`: the shape of the command
event, and what is trusted), the existing interface and node, lessons 3 to 8 and 14 in `docs/PLAN.md`,
and the CM3/CM4 review in `docs/REVIEWS.md` for the kind of defect to avoid.

## Do

1. **Interface.** The calls of SPEC_NAV2.md, 8.1 to 8.3 (rewritten by
   T05b), thin wrappers around `step`, plus the query `roslean_cm_cmd_plan`
   that tells the shim which lookups to make. The scan call hands over the
   ranges only; **the command call hands over one coordinate pair per
   reading of the stored scan, in the base frame**, or "none", and for each
   topic zone in another frame its vertices in the base frame or "none"
   (events E7 and E8 as they are in the model now). The byte layouts in 8.2
   and 8.3 are proposals; the meanings are not. Conversions and refusals in
   Lean: the shim hands over `double` bit patterns in metres and Lean makes
   the millimetres (`coordMm`, `vertexCoordMm`). **The cost of a command was an
   open item here; task T03 closed it** (`tasks/reports/T03.md`,
   "Signatures T06 builds on"; SPEC.md, "Command event"; COMPARISON.md,
   "Cost of a command"). Build the new command call on what T03 left:
   `stepCmdFast cfg st now v n f zoneVerts`
   (`core/ROSleanCore/CollisionMonitorFast.lean`) with `n` the number of
   coordinate pairs handed over (or `none`) and `f i` the point of pair
   `i`, read from the bytes with `coordMmAt` (a machine word per
   coordinate; never a `Nat` pattern, which is a big number in Lean's
   runtime when the sign bit is set). `stepCmdFast_eq`
   (`ROSlean/CollisionMonitor/Fast.lean`) says it is `step` on the command
   event that carries the pairs `g 0, …, g (n - 1)` whenever `f i =
   scanPointMm (g i)`; `coordMmAt_eq` gives that for `g i` the pair of
   `u64At` patterns of pair `i`. The zone vertices still go in as lists;
   they are few. Do not decode coordinates into lists when a scan or a
   command arrives: that cost 225 ns per reading in the scan call until
   T03 took it out. Keep the budget: `roslean_cmd_cost` (50, 150 and
   1500 µs per command at 360, 1080 and 10,000 stored readings) must still
   pass with the new calls, and a command without a clock reading
   (`roslean_cm_on_cmd_no_clock`, SPEC D15) must exist in the new interface
   too. The
   present calls (`on_scan` with coordinates and `usable`, `on_cmd` without
   arguments for points) are replaced; `Monitor.coords`, which stands in
   for them today, goes. Theorems in
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
   with the command; it is not an event of its own. **The shim then
   computes the points** (option A): for every reading of the stored scan,
   valid or not, the scan-frame point of R3 in `float`, widened, and its
   image by N25 in `double` (the matrix from the quaternion, the two sums,
   the third coordinate dropped: the projection), and hands the `x` and `y`
   over; the same for the vertices of a topic zone in another frame. Use
   tf2's own `fromMsg` and `operator*` for N25, so that the operations and
   their order are upstream's by construction. The shim does not test the
   range of a reading and does not skip one: which readings count is
   Lean's decision, and Lean refuses a list that is not one pair per
   reading. A transform with a number that is not finite, or with a zero
   quaternion, is passed as "none" (ND7): the shim checks this, since the
   model no longer sees the transform. All of this sits in the one file
   that calls tf2 (`transforms.hpp`, 8.4), with unit tests for which
   transform it asks for, for every kind of failure, and for the
   arithmetic: the points equal a reference computation written out
   operation by operation, bit for bit, on exact and on general
   quaternions. **`-ffp-contract=off` stays for the whole node, this file
   included (decided)**; T01's check that the object code has no fused
   multiply-add is extended to it. `roslean_sensor_pose` is removed: the
   parameter is refused with a message that points to TF.
5. **Zones fed by a topic.** `polygon_sub_topic` on a polygon zone without
   `points`, with upstream's QoS (N31). `footprint_topic` is **not** such a
   subscription: upstream reads it for `approach` only (N29). Each message
   with at least three points becomes the shape event, after the lookup of
   N33 if its frame is not the base frame; Lean decides whether it is
   accepted. The event carries the vertices **in the base frame**, computed
   by the shim (N33, step 3; for a message in the base frame the identity
   is applied, as upstream does), or "none" if the lookup failed. The shim
   keeps the accepted message: for shapes in another frame the transform is
   looked up again at every command and the vertices are recomputed and
   handed over with it (N34, ND3).
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
   `test_cm_ffi` is back to 48 s after T05b (169 s after T05, 39 to 42 s
   before it), so its leak test needs no fewer rounds on that account;
   measure it again with the new calls. Measure a command with 360 and with
   1000 readings through the node against upstream: the cost of handing the
   points over with every command is only estimated so far (SPEC_NAV2.md,
   section 5).
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
