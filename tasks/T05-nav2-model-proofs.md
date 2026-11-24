# T05 — Nav2 extension: model and proofs

Branch: `claude/cm-nav2-model`. Needs: T01 and T04 merged.
Read [README.md](README.md) first.

## Read

`docs/collision_monitor/SPEC_NAV2.md` (section 3, the data and the events
E1–E8; section 4, the properties; section 5, the transform; section 9, the
edits to SPEC.md), SPEC.md, THEOREMS.md,
`core/ROSleanCore/CollisionMonitor.lean`, `ROSlean/CollisionMonitor/*.lean`.

## Do

1. Add the data and events of SPEC_NAV2.md, section 3, exactly as stated:
   the five lifecycle events and the phase; the shape event for a zone fed
   by a topic; the command event carrying the looked-up transforms; the
   report for the state topic; `Output` with `silent` distinct from a
   processed command whose velocity is withheld. A missing transform is
   **not an event**: T04 found that upstream looks the transform up at every
   command, so it is a field of the command event (`scanTf = none`). `step`
   stays total. Note what SPEC_NAV2.md says is fail-open in upstream and
   kept (a topic zone without shape takes no action; `source_timeout: 0`):
   state those theorems as written there, do not strengthen the model.
2. SPEC_NAV2.md recommends applying transforms in Lean (option B), and that
   is the default until the user decides otherwise (PLAN.md, "Decisions").
   A Mathlib-free definition in `core/` (`Xform.pointMm`, `Xform.vertexMm`)
   that applies the transform a quaternion and a translation denote, given
   as bit patterns, in exact arithmetic: the first two rows of the **3D**
   rotation, not a planar transform (upstream projects a tilted sensor on
   the plane, N25). Theorems: the properties under "Transform" in section
   4. "A point upstream counts is always counted" is **not** provable in
   Lean: it also rests on an estimate of upstream's floating-point error
   (T7). Prove the quantitative half as today (the analogue of
   `nearBoundary_of_rounded` for transformed points and vertices) and say in
   THEOREMS.md what is prose.
   `review/collision_monitor/nav2/TransformCost.lean` is a prototype to
   measure with, not the definition to use: it handles zero components
   badly.
3. Extend every theorem in `ROSlean/CollisionMonitor/` to the new events: the
   iff characterisations of each output and action, the properties (stop,
   stale, no amplification, slowdown bound, pass-through, bad data,
   monotonicity), the state invariants, and the trace theorems for every
   event sequence from `State.init` (which is now unconfigured: every
   existing trace theorem needs the lifecycle prefix or a start state).
   New theorems, with the exact statements in SPEC_NAV2.md, section 4:
   Processed, Inactive, Report, No transform (with its two hypotheses on the
   configuration and the counterexample without them), Shape accepted, No
   shape, Replacing a shape affects that zone only, Lifecycle invariants,
   After activation.
4. `Examples.lean`: a scenario for each new event. THEOREMS.md: the new
   theorems in plain words, and the updated list of what is not proved.

If T01 introduced `Event.scanUnusable` or `Monitor.dropScan`: they are
superseded (SPEC_NAV2.md, section 9, edit 10). A scan is no longer usable or
unusable on arrival; remove them and say so in the report.

Do not write the C interface or C++ (T06). Leave the interface files
compiling: if a changed definition breaks `CollisionMonitorFFI.lean` or
`ROSlean/CollisionMonitor/FFI.lean`, make the smallest change that restores
the build and list it for T06.

## May change

`core/ROSleanCore/CollisionMonitor.lean` and new Mathlib-free files beside
it, `ROSlean/CollisionMonitor/`, SPEC.md (model and properties sections),
THEOREMS.md, SPEC_NAV2.md (corrections).
