# Independent reviews

After each milestone a separate agent reviewed the work without seeing the
author's reasoning. This file records what each review found and what was
done. "Fixed" means changed in the repo; "documented" means the behaviour is
intended and now stated; "open" means not addressed.

## M1 — specification

No rule was wrong.

| Finding | Outcome |
| --- | --- |
| A never-heard lock with a timeout is locked only once the clock has passed the timeout | Fixed in R6 |
| Upstream re-reads the clock for every expiry check; one reading per event is our idealisation | Documented as D4 |
| Upstream converts timeouts to integer ns by truncation, not rounding | Fixed (R1, D2; the shim truncates) |
| Wrong YAML type for a parameter is fatal | Documented in R14, reproduced |
| Clamping interacts with the distinctness check | Fixed (D1; check runs on clamped values) |
| NaN / huge timeouts, 32-bit priority read | Documented as D5, D6 |
| Backwards clock is fail-open | Documented in A2 |
| Upstream's own tests do not pin arbitration | Documented; motivated M6 |

## M2 — Lean model

No decision differed from upstream for states reached from `State.init`.

| Finding | Outcome |
| --- | --- |
| A state shorter than the config silently dropped updates (e.g. an e-stop `true`) | Fixed: writes extend the list (`setPad`); proved total-map behaviour (`getD_setPad`) |
| Priorities not clamped in the model | Fixed: `Config.validate` clamps |
| Lock priority recomputed per input | Fixed |
| Missing example scenarios | Added |

## M3 — C interface

No reference-counting, signature or conversion defect.

| Finding | Outcome |
| --- | --- |
| Stale toolchain after a `lean-toolchain` bump | Fixed: re-detected on every configure |
| A changed Lean signature broke nothing at build time | Fixed: `ffi.h` is compiled against the generated C |
| `initialize()` not thread-safe, retry after failure falsely succeeds | Fixed |
| Moved-from `Mux` crashed | Fixed: throws |
| Test gaps (values ≥ 2⁶³, clamping, moves, builder reuse) | Added |
| Linux loading of transitive Lean libraries may fail | Open: mitigation added but unverified |
| Install tree not relocatable | Open (deployment) |

## M4 — node

No behavioural bug in the shim.

| Finding | Outcome |
| --- | --- |
| Scenario never run against upstream | Fixed in M6 |
| Scenario not registered as a test | Fixed |
| Two expectations confounded by an expired watchdog lock | Fixed: scenario rewritten under simulated time |
| Wall-clock timing fragility | Fixed: simulated time |
| Clamping and timeout normalisation lived in C++, outside the proofs | Fixed: moved to Lean (`handleOfRaw`), with theorems |
| Publisher captured by reference before assignment | Fixed |
| Start-up failure mode differs (exit 1 vs abort) | Documented as D7 |
| No launch file | Documented as D9 |

## M5 — proofs

No statement wrong or vacuous; only standard axioms.

| Finding | Outcome |
| --- | --- |
| Nothing tied the C interface or `Config.validate` to the theorems | Fixed: `ROSlean/Mux/FFI.lean`, ending in `muxForward_after` |
| No state-free characterisation | Fixed: `run_forward_iff_history` |
| "The lock freezes everything" had no theorem; an input at 255 is never blocked | Fixed: `not_forward_of_top_lock`; shim warns on priority 255 |
| `inputLive_boundary` docstring overclaimed | Fixed: replaced by `inputLive_iff_le` |
| One example did not show what its comment said | Fixed |
| `run_*` theorems not phrased on `run` output via an append lemma | Open (cosmetic) |

## M6 — comparison with upstream

420 random cases (9,586 decisions) run by the reviewer: 0 divergences.

| Finding | Outcome |
| --- | --- |
| Shipped comparison small and confined to exactly representable times | Fixed: `random_diff.py` adopted, 20 seeds in the test suite, Unix-scale clock bases included |
| D2 is real and, exactly at a boundary under simulated time, frequent | Documented ("Extent of D2") |
| D8 unobservable: `rclcpp` aborts on a negative clock | Fixed in SPEC |
| Second node's own failures were discarded | Fixed |
| Bad configs are checked for our node only | Open |
