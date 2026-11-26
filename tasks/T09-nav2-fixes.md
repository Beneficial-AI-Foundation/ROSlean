# T09 — Fixes from the review of the Nav2 extension

Branch: `claude/cm-nav2-fixes`. Needs: T08 merged.
Read [README.md](README.md) first.

For each finding of the section "CM5 — Nav2 extension" in `docs/REVIEWS.md`:
fix it, add or repair a test that fails without the fix (show once that it
does), and set the finding's outcome. Where a finding needs a decision that
is the user's ([PLAN.md](PLAN.md), "Decisions"), keep the default and say so
in the outcome. Turn every expected-failure registration that the fix makes
pass into a normal test.

Decisions for the findings are in [PLAN.md](PLAN.md), "Decisions", rows
marked (T08): follow them. Finding 1 comes first: it is the one place where
upstream stops and the node does not.

**Tests (changed since this brief was written):** the full `pixi run test`
is run once by the orchestrator at the very end, not by this task. Run
`pixi run test-quick`, every long test of the collision monitor once on the
final tree, and the two random comparisons with 300 seeds each.

Then run everything else: `lake build` in `core/` and the root, `#print axioms` on
the main theorems, `pixi run build`, `pixi run test`, the long random
comparison beside upstream (at least 600 seeds), the Docker build. No
decision may differ from upstream outside the numbered differences.

Update `docs/collision_monitor/SPEC.md` so that it is one document again:
fold SPEC_NAV2.md's rules, differences and trusted list into it, leave
SPEC_NAV2.md's section "What still separates this from Nav2's default
configuration" where it is, and correct that section for what was actually
built.

## May change

Everything under `core/`, `ROSlean/CollisionMonitor/`, `ros/`, `docs/`,
`docker/`. Not `site/` or `README.md`.
