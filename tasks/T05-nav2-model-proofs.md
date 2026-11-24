# T05 — Nav2 extension: model and proofs

Branch: `claude/cm-nav2-model`. Needs: T01 and T04 merged.
Read [README.md](README.md) first.

## Read

`docs/collision_monitor/SPEC_NAV2.md` (the new events, their statements, and
the chosen transform option), SPEC.md, THEOREMS.md,
`core/ROSleanCore/CollisionMonitor.lean`, `ROSlean/CollisionMonitor/*.lean`.

## Do

1. Add the new events to the model exactly as SPEC_NAV2.md states them:
   transform unavailable, zone shape replaced (the footprint), node activated
   and deactivated, and what else that document lists. `step` stays total;
   every failure case answers with a stop or with silence as specified.
2. If SPEC_NAV2.md chose to apply transforms in Lean: a Mathlib-free
   definition in `core/` that applies a planar rigid transform, given as bit
   patterns, to a point in exact arithmetic, with its rounding direction, and
   theorems for its error bound and for "a point upstream counts is always
   counted".
3. Extend every theorem in `ROSlean/CollisionMonitor/` to the new events: the
   iff characterisations of each output and action, the properties (stop,
   stale, no amplification, slowdown bound, pass-through, bad data,
   monotonicity), the state invariants, and the trace theorems for every
   event sequence from `State.init`. New theorems: an inactive node publishes
   nothing; a missing transform stops; a zone whose shape was never received
   behaves as specified; replacing a shape affects exactly that zone.
4. `Examples.lean`: a scenario for each new event. THEOREMS.md: the new
   theorems in plain words, and the updated list of what is not proved.

Do not write the C interface or C++ (T06). Leave the interface files
compiling: if a changed definition breaks `CollisionMonitorFFI.lean` or
`ROSlean/CollisionMonitor/FFI.lean`, make the smallest change that restores
the build and list it for T06.

## May change

`core/ROSleanCore/CollisionMonitor.lean` and new Mathlib-free files beside
it, `ROSlean/CollisionMonitor/`, SPEC.md (model and properties sections),
THEOREMS.md, SPEC_NAV2.md (corrections).
