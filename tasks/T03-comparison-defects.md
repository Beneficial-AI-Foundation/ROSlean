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
   **Since T05b** (read `tasks/reports/T05b.md`, "Cost of a command"): the
   command now carries the base-frame coordinates of every reading, and the
   conversion to millimetres (`coordMm`) runs per command. Measured with
   `cmd_cost` at 1080 readings: 207 µs for two circles (18 µs before the
   extension) and 628 µs for two 4-vertex polygons (0.41 ms before). Of the
   207 µs about 135 µs is `coordMm` on `Nat` patterns (a pattern with the
   sign bit set is a big number in Lean's runtime), 50 µs the lists, 18 µs
   the zones. So this item has two parts now, in this order: (a) make the
   conversion cheap, for example by decoding sign and magnitude apart as
   before T05, proved equal to the current definition; (b) the polygon path
   as described above. "Counting once per scan" is no longer possible: the
   points differ with every command. State a budget per command for 360,
   1080 and 10,000 readings, measure against it and against upstream
   (8 µs at 1080 readings and two polygons, T02), and replace the limit of
   T02's `roslean_cmd_cost` test (25 ns per reading and zone, a line T02
   drew) by that budget. Correct COMPARISON.md where it still explains the
   cost by `Xform.pointMm`, which no longer exists.
3. Any other defect T02 reported. For each: fix, test (turn the expected
   failure into a passing test), and note it in COMPARISON.md.

Re-run the random comparison (at least 300 seeds) after the changes: no
decision may differ from before.

## May change

The model (additions only, plus what item 1 needs), the proofs,
`ros/roslean_core`, the node, SPEC.md, THEOREMS.md, COMPARISON.md (the
defects' outcomes), `bench/` results.
