# T03 — Fix the defects the comparison found

Branch: `claude/cm-comparison-defects`. Needs: T02 merged.
Read [README.md](README.md) first.

## Read

`docs/collision_monitor/COMPARISON.md`, section "Defects", and the tests T02
registered as expected failures.

## Do

1. **A `/clock` message with a negative time ends the process.** No topic
   content may end the node. Find a fix that works with `use_sim_time`
   (the comparison suggests `use_clock_thread(false)` with a `catch` around
   the spin, or a clock subscription of the node's own). After the fix the
   node keeps running and, while it has no valid clock reading, answers
   commands with a stop through Lean (a new model event or an existing one;
   state which, and prove what it does). Upstream aborts here; record the
   difference as a numbered D.
2. **The command path is slow for polygon zones** (about 200 ns per reading
   and zone; 4 ms per command at 10,000 readings and two polygons). Make it
   fast without changing a single decision: for example a bounding-box test
   per zone, or counting once per scan instead of once per command, each
   proved equivalent to the current definition in `ROSlean/CollisionMonitor/`
   (the theorems are stated on the current definitions; add the fast
   function and a theorem that it equals the slow one, and have the
   interface call the fast one). Measure before and after with
   `bench/roslean_bench`. Add a start-up warning when the configured zones
   and the first scan's size make a command cost more than a stated bound.
3. Any other defect T02 reported. For each: fix, test (turn the expected
   failure into a passing test), and note it in COMPARISON.md.

Re-run the random comparison (at least 300 seeds) after the changes: no
decision may differ from before.

## May change

The model (additions only, plus what item 1 needs), the proofs,
`ros/roslean_core`, the node, SPEC.md, THEOREMS.md, COMPARISON.md (the
defects' outcomes), `bench/` results.
