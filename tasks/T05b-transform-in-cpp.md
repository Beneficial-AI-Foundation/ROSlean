# T05b — The transform in C++ (option A)

Branch: `claude/cm-nav2-model-a`. Needs: T05 merged. Read [README.md](README.md)
and "Watching the tasks" in [PLAN.md](PLAN.md) first.

## Why

T04 recommended applying the sensor-to-base transform in Lean in exact
arithmetic (option B of `docs/collision_monitor/SPEC_NAV2.md`, section 5),
and T05 built and proved that. The user has decided for option A: the
transform is applied in the C++ shim in `double`, as upstream does, and Lean
receives points. Reasons: B costs 1.2 to 1.9 µs per reading and command;
through the node T02 measured about 20 times upstream's time per command,
and commands left unanswered at 10,000 readings.

This task changes the model, the proofs and the specification. It writes no
shim code: T06 does.

## Do

1. **Model.** The command event carries what option A says (SPEC_NAV2
   section 5, column A): the points of the stored scan in the base frame, as
   the shim computed them for this command, or the fact that no usable
   transform was available; and for each topic zone whose shape is in
   another frame, its vertices in the base frame or "no transform". Design
   the representation (bit patterns of the coordinates, converted to
   millimetres in Lean by the existing `coordMm`/`vertexMm`, is the obvious
   one) and say why. Everything else of T05 stays: lifecycle, shapes,
   reports, `Output.answer`, the fail-open cases as upstream.
2. **What the stored scan still decides.** Age, validity of the scan as a
   whole (R7, R8) and the count of readings stay functions of the stored
   scan. Say precisely what is no longer a function of it. If the shim hands
   over one point per valid reading, state and prove what can be proved
   about the correspondence (for example the number of points against the
   number of valid readings), and list the rest as trusted.
3. **Proofs.** Rework every theorem of T05 that mentions `Xform`. Keep the
   statements as strong as option A allows; where a theorem of T05 can no
   longer be stated (for example "a scan point is the exact image,
   truncated"), say so in THEOREMS.md and in the report, with what replaces
   it. `CollisionMonitorXform.lean` and `Transform.lean`: remove them, or
   keep the parts that are still used (vertex rounding, the margin
   argument) and say which. No `sorry`, no `native_decide`, no new axioms.
4. **The margin.** Redo the argument "a point upstream counts is always
   counted" for option A: the shim's point is upstream's point up to the
   compiler's choices (fused multiply-add or not, SPEC_NAV2 N25), then
   truncated to millimetres as before T05. State the budget and what is
   proved, what is prose and what is trusted.
5. **Existing C interface and node** keep building and behaving as now
   (`cmNew` after configure and activate, a scan call, a command call). With
   the transform gone from Lean the cost of a command should return to what
   it was before T05; measure `test_cm_ffi` (169 s after T05) and report.
6. **Documents.** SPEC_NAV2.md: section 5 records the decision and why;
   sections 3, 4, 7 (the trusted list: the shim's transform arithmetic and
   how it is compiled are now trusted), 8 (the shim boundary for the command
   call) and 9 follow. THEOREMS.md and SPEC.md agree with the code. Correct
   `tasks/T06-nav2-node.md` and `tasks/T07-nav2-harness.md` where they
   assume option B (T06: the shim now computes N25 and the projection; say
   which compiler flags are assumed and that the choice of `-ffp-contract`
   is an open point for the user).

Then: `lake build` in `core/` and the root, `#print axioms` on the main
theorems, `pixi run build`, the tests that start no ROS node
(`roslean_core`'s tests; `smoke_conversions`, `smoke_timeouts`). The node
tests are run by the orchestrator at the merge.

## May change

`core/ROSleanCore/CollisionMonitor*.lean`, `ROSlean/CollisionMonitor/`,
`core/ROSleanCore.lean`, `ROSlean.lean`, the module list in
`ros/roslean_core/CMakeLists.txt`, `docs/collision_monitor/SPEC.md`,
`SPEC_NAV2.md`, `THEOREMS.md`, `tasks/T06-*.md`, `tasks/T07-*.md`,
`tasks/reports/T05b.*`. Not the node, not the harness.
