# T01 — Verify and finish the node fixes

Branch: continue `claude/cm-node-fixes` (push to the same name). Needs: nothing.
Read [README.md](README.md) first.

## State

`claude/cm-node-fixes` holds one commit on top of `collision-monitor`: "Collision monitor:
fix the eight findings of the CM3/CM4 review". The session that wrote it was
stopped before it reported, so nothing about it is verified and it may be
incomplete. Treat it as a draft by someone else.

## Read

The section "CM3/CM4 — collision monitor C interface and node" at the end of
`docs/REVIEWS.md` (eight findings, each with its smallest input), the scripts
under `review/collision_monitor/` that back them, `docs/collision_monitor/SPEC.md`
and `THEOREMS.md`, and the branch's diff against `collision-monitor`.

## Do

For each finding, check that the branch does what is listed here, complete it
if not, and make sure a test fails when the fix is removed.

1. `config/example.yaml` no longer sets `roslean_sensor_pose`. A warning at
   start-up when a pose is set; a throttled warning when a scan's `frame_id`
   differs from the base frame while a pose is set. The review's
   backwards-sensor case is a test.
2. Every parameter change after start-up is rejected with a reason;
   undeclared parameters cannot be set. Tested through the parameter service.
3. The model has `Event.scanUnusable (now : Nat)`; `step` clears the scan and
   is silent; the interface's unusable-scan path is a `step` on that event;
   no interface theorem assumes "every scan usable"; the trace theorems hold
   for all histories; the end-to-end theorems are stated on the exported
   functions as the node calls them. `Examples.lean`, SPEC and THEOREMS.md
   agree with the code.
4. Each test the review calls unable to fail can now fail. Node-level cases
   exist for `-0.0`, a scan of realistic size and a zero pose.
5. The node builds with optimisation when no build type is given, keeps
   `-ffp-contract=off`, and no text claims bit-for-bit agreement with
   upstream's point computation beyond what is true.
6. A monitor built without a source is not ok; a refused value anywhere in
   the call sequence makes it not ok; THEOREMS.md's claim about refused
   vertices matches a proved theorem.
7. The radius message states the true bound; all refusals are reported in one
   start-up failure; a warning (not a refusal) for each configuration that
   protects nothing.
8. An unknown output kind is answered with zero velocity.
9. Each change of the action is logged at INFO, naming the zone, with the
   text built from what Lean returned (upstream logs "Robot to stop due to …").

Then: `lake build` in `core/` and the root; `#print axioms` on the main
theorems; `pixi run build`; `pixi run test`; `pixi run cm-check` if rendering
works (the demo's `monitor.yaml` sets a sensor pose on purpose; the new
warning is expected there). Set the outcome of each finding in
`docs/REVIEWS.md`.

## May change

The model, the proofs, `ros/roslean_core`, the node, its `smoke_*` tests and
config, SPEC.md, THEOREMS.md, REVIEWS.md. Not the harness files
`test/scenario.py`, `random_diff.py`, `cm_harness.py`. Behaviour the review
did not ask about stays as it is.
