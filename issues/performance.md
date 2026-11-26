# Collision monitor: bring the cost of a command closer to upstream's

Experimental. Nothing here is required: the budget is met and accepted (`tasks/PLAN.md`, "Decisions", 2026-11-25).

## Where we are

Median per command, both nodes' callback code in one process on the same messages, Apple M5 Pro, macOS arm64 (task T06):

| Stored readings | Two circles: Lean / upstream, µs | Two polygons: Lean / upstream, µs | Budget, µs |
| ---: | ---: | ---: | ---: |
| 360 | 10.3 / 1.7 | 16.5 / 2.8 | 50 |
| 1080 | 30.0 / 4.1 | 48.9 / 7.2 | 150 |
| 10,000 | 281 / 31 | 438 / 60 | 1500 |

- 6 to 9 times upstream's. In the Linux arm64 container an earlier version of the command path measured 4.2 to 6.4 times.
- The scan callback costs 14.5 µs at 1080 readings; upstream stores a pointer.
- With one command every 20 ms instead of back to back, a command took 2 to 5 times longer (cold caches), and the factor varied between runs.
- No worst-case time is known.

Details and method: `docs/collision_monitor/COMPARISON.md`, "Cost of a command". Programs: `cmd_cost` (test `roslean_cmd_cost`, asserts the budget), `pixi run bench-cm`.

## What was already done

- The scan is kept as bytes; no list of readings is rebuilt per command.
- Sign and magnitude of a coordinate are decoded apart, proved equal to the definition.
- A fast path for the command, proved equal to `step` (`stepCmdFast_eq`).
- The transform is applied in C++ (user's decision); the Lean core receives base-frame coordinates as bit patterns.

This took the command from 340 to 780 times upstream's to 6 to 9 times.

## Step 0: find out where the time goes

Not done since T03. Before choosing a scenario:

- profile `cmd_cost` at 1080 readings (`perf` on Linux, Instruments on macOS): share of allocation, reference counting, the conversion to millimetres, the zone tests;
- count allocations per command;
- read the C that Lean emits for the fast path and look for boxed values in the per-reading loop.

T05b's experiment (conversion replaced by a constant) put about two thirds of a circle command into conversion and list handling before T03. The split after T03 is not known.

## Scenarios

Estimates of gain are guesses until step 0 is done.

| | Scenario | What changes | Proof work | Trusted part | Guess at gain |
| --- | --- | --- | --- | --- | --- |
| A | Unboxed loop in Lean | The per-reading loop works on `ByteArray` and machine integers (`UInt64`/`Int64`) with no intermediate list or pair; millimetre values that fit 64 bits stay unboxed | A second fast path, proved equal to the present one; bounds lemmas for the machine arithmetic (D12 already bounds coordinates) | Unchanged | up to 2 times |
| B | Convert once, at the scan | The conversion to millimetres moves from the command to the scan callback; a command only tests integers | Small: the stored value changes representation, equality with `Monitor.coords` | Unchanged | Shifts cost from 50 Hz to 25 Hz; less per command, more per scan. Not possible for topic zones in another frame or with base shift correction, where the transform depends on the command's time |
| C | Early exit | Stop testing a zone once `min_points` are inside; skip later zones after a stop | Small; the decision is the same, only the count is cut short | Unchanged | Large when something is near, none in free space, which is the case that is measured |
| D | Bounding test first | Reject a reading against a zone's bounding box or radius before the exact test | Small: a lemma that the pre-test implies "outside" | Unchanged | Polygons only; depends on zone size against scan range |
| E | The loop in C++ | The shim tests the points; Lean decides from the counts | The theorems about containment would be about code that no longer runs | Larger: the point tests become trusted | Upstream's speed. Gives up the main claim of the project for this path |
| F | Checked C++ | As E, and Lean recomputes a sample, or the counts on every n-th command, and stops on disagreement | New model of the check | The C++ loop is trusted between checks | As E, with a weaker guarantee than now and a stronger one than E |

A to D keep every theorem. E and F trade the guarantee for speed and need the user's decision before any work.

## Suggested order

1. Step 0.
2. C and D, which are cheap and independent.
3. A, if step 0 shows boxing or allocation in the loop.
4. B only if the per-command cost matters more than the per-scan cost for a real configuration.

## Done when

- `bench_cm` on the same machine shows the new ratio, with the old one beside it;
- `roslean_cmd_cost` passes, with a tighter budget if the gain is stable;
- no `sorry`, no new axiom; the random comparison (`pixi run cm-diff 0 300`) gives the same 300 lines as before.

## Not in scope

A bound on the worst-case time. It needs a different kind of measurement (hours, paced, on the target) and belongs to the Linux hardware issue.
