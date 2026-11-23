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
| Clamping and timeout normalisation lived in C++, outside the proofs | Fixed: moved to Lean (`handleOfRaw`, `Handle.clamp`), characterised by `handleOfRaw_*` and `validate_ok_iff`. The seconds-to-nanoseconds truncation stays in C++ |
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

## M7 — demo, documentation, reproducibility

Build, tests, proofs and demo all reproduced; every theorem named in the
README exists and matches its row.

| Finding | Outcome |
| --- | --- |
| Narrator could print "planner drives" just after an e-stop (message in flight) | Fixed: grace period after a lock change; unexpected cases are labelled |
| README had no demo instructions | Fixed |
| README "not proved" list incomplete | Fixed |
| No theorem characterised `handleOfRaw` | Fixed |
| Stale statements in PLAN.md | Fixed |
| `pixi run prove` would build Mathlib from source on a fresh clone | Fixed: fetches the cache first |
| CI is the Lean template and covers none of the ROS side | Open; listed under "Not done" |
| No LICENSE file although `package.xml` declares Apache-2.0 | Open: needs a decision by the owner |
| `twist_mux.md` still shows the two decisions as open and links to private pages | Decisions recorded and a reader's guide added on 2026-11-20; the links to private pages remain |

## After the reviews: equal priorities restored (2026-11-20)

At the user's request the node again accepts equal input priorities and
resolves them as upstream does (SPEC R10; D1 withdrawn). Entries above that
mention the distinctness check, `Config.validate`, `validate_ok_iff` or
`builderCheck` describe the code as it was when reviewed: `Config.validate`
is now the total `Config.ofRaw`, the start-up check and `roslean::ConfigError`
are gone, and `forward_iff` carries the tie-break. An independent review of
this change found no error in the model, proofs, C interface or tests:

| Finding | Outcome |
| --- | --- |
| "Alphabetical by name" is looser than the real order (byte order of parameter names) | Fixed in SPEC, README, `twist_mux.md`, example config |
| "Accepts the same configurations as upstream" ignores D5 | Fixed |
| That `list_parameters` returns prefixes in map order is not verified against the `rclcpp` source (not in the pixi env); both nodes make the same call, and the tie scenario and random test agree with it | Open, noted |
| No test name contains a byte below `.`, where the order differs from a plain name sort | Open |
| No fixed ROS-level scenario for a tie created by clamping (300 vs 255); covered in the FFI test and by chance in the random test | Open |
| `forward_priority` is now `≤`; the strict fact for an earlier input needs `forward_first_of_equal` too | By design; `forward_iff` carries both |

## Longer random cases (2026-11-23)

The random differential test now generates 350–550 events per case instead
of 35–55 (`random_diff.py`, `gen`); its CMake timeout went from 900 s to
2400 s. The generator is otherwise unchanged, so this adds longer histories
per case (more clock jumps, lock changes and take-overs before each
decision), not new kinds of input: still 1–6 inputs, 0–4 locks, and no event
within 25 ms of a timeout boundary (D2).

Full suite in a Linux arm64 container (Ubuntu 24.04, ROS Jazzy and upstream
`twist_mux` from apt):

| Test | Result | Time |
| --- | --- | --- |
| `test_mux_ffi` | passed | 0.3 s |
| `scenario` | passed; both nodes agree on all 69 decisions | 58 s |
| `random_diff`, seeds 0–19 | passed; 0 divergences in 4,692 decisions | 412 s |

Decisions per random case: 157 to 359 (13 to 33 before; 447 in total
before). Every seed passed on the first attempt, with no slow retry.

| Finding | Outcome |
| --- | --- |
| No full run on macOS: the host runs were interrupted by another process on the machine. Seeds 0–9 completed there and passed with the same decision counts as in the container | Open |
