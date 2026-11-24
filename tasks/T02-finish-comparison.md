# T02 — Finish the comparison with upstream

Branch: start from `claude/cm-comparison-wip`, push to `claude/cm-comparison`.
Needs: T00 and T01 merged. Read [README.md](README.md) first.

## State

`claude/cm-comparison-wip` is one WIP commit on top of an older `collision-monitor`
(`462088e`). It extends the harness so that the Lean node runs through it,
adds a random comparison of the Lean node with upstream that asserts the
SPEC's millimetre bounds, fault injection, benchmarks, and
`docs/collision_monitor/COMPARISON.md`. The session was stopped while its long
runs were going: the document still contains placeholders of the form
`{{name}}`, and nothing in it has been checked by anyone else. All its numbers
are from macOS arm64, measured while other work loaded the machine.

## Do

1. Merge `origin/collision-monitor` into the branch (T01 changed the node, the model and
   the package's `CMakeLists.txt`; the merge of build files is the union).
   Resolve what T01 made obsolete: the document's defects 3 and 4 (build
   type, `sinf`/`cosf` against `sincos`) may be fixed; re-measure and say so.
2. Read the harness changes critically before trusting them. For each
   expected difference per scenario, check that it names a D number of the
   SPEC and is not a blanket skip. Run the self-tests that show the random
   comparison can fail (`--self-test swap`, `shrink`).
3. Run everything in the document's "What was run" table on this Linux
   machine, with the Lean node as now on `collision-monitor`, and replace every
   placeholder and every macOS number that you re-measured. Keep the macOS
   numbers that you cannot re-measure, marked with platform and date. At
   least 600 random seeds beside upstream.
4. Any case where upstream stops or slows and the Lean node does not is the
   most serious finding: seed, smallest reproduction, cause.
5. Report every defect of the node, shim or model with a reproduction and a
   registered test marked as an expected failure. Do not fix them (T03 does).
6. Keep the registered tests short; long runs go behind pixi tasks. State how
   long `pixi run test` takes afterwards.

## May change

`ros/roslean_collision_monitor/test/` (harness files, not `smoke_*`), the
marked harness block of the package's `CMakeLists.txt`, `bench/`,
`pixi.toml` (tasks), `docs/collision_monitor/COMPARISON.md`,
`UPSTREAM_BEHAVIOUR.md`, the "Note on testing against upstream" of SPEC.md.
