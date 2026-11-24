# ROSlean development plan

Scope: [`twist_mux.md`](../twist_mux.md). Goal: a drop-in replacement for the
ROS 2 `twist_mux` node whose arbitration logic is a Lean function with proved
properties, plus a reusable Lean-to-ROS shim.

Second node, since 2026-11: the stop and slowdown core of Nav2's
`nav2_collision_monitor`, as a node that Nav2 can launch in place of its own
monitor. Its milestones, what is not done and its lessons are under
[The collision monitor](#the-collision-monitor) and
[Lessons from the collision monitor](#lessons-from-the-collision-monitor)
below. The sections in between are about twist_mux and the shared pipeline
and are unchanged, so that references such as "PLAN, lesson 16" still hold.

## Decisions

| Question | Decision |
| --- | --- |
| ROS environment | pixi + RoboStack, project-local (`pixi.toml`), native on macOS arm64 |
| ROS distribution | Jazzy |
| All inputs silent | Match upstream: publish nothing, state the base-driver watchdog as an assumption. Timer plus proved "silence leads to zero" is a stretch goal. |
| Equal input priorities | As upstream: the first live input in handle order wins (SPEC R10). Initially rejected at start-up; changed back on 2026-11-20 at the user's request |
| Shim language | C++ (`rclcpp`), calling Lean through the C ABI |

## Architecture

```
core/          A lake package with no dependencies. core/ROSleanCore is the
               executable model: Config, State, Event, step. Lake compiles it
               to C; `ros/roslean_core` builds that C into a shared library,
               which the node loads.
ROSlean/       The root lake package: Lean + Mathlib, requires `core`. Theorems
               about ROSleanCore. Never linked into a node.
ros/<pkg>/     ament_cmake packages. C++ shim only: subscriptions, parameters,
               clock, publishing. No decision logic.
```

The message body never crosses into Lean. The shim passes (input index,
timestamp in ns) and gets back a forward / drop decision.

## Milestones

Milestones M1 to M7 were each followed by an independent review by a separate
agent; the findings and what was done about them are in
[REVIEWS.md](REVIEWS.md).

| | Milestone | Status | Result |
| --- | --- | --- | --- |
| M0 | Pipeline spike | done | `ros/roslean_noop`: an `rclcpp` node calling a Lean function. |
| M1 | Specification from the upstream source | done | `docs/SPEC.md`: rules R1–R16, differences D1–D9. |
| M2 | Lean model | done | `core/ROSleanCore/Mux.lean`, examples in `ROSlean/Mux/Examples.lean`. |
| M3 | FFI surface | done | `core/ROSleanCore/FFI.lean`, `ros/roslean_core` (C++ wrapper, build-time signature check, tests). |
| M4 | `roslean_twist_mux` node | done | Same parameter format, topics and QoS as upstream. Accepts every configuration upstream accepts except non-finite or huge timeouts (D5), including its example config with two inputs at priority 100. |
| M5 | Proofs | done | `ROSlean/Mux/{Basic,Properties,Trace,FFI}.lean`; table in the README. |
| M6 | Comparison with upstream | done | Fixed scenarios with exact boundaries and a randomised differential test; no divergence outside D2. This replaces the recorded teleop/Nav2 session of the scope doc with scripted and random event sequences under simulated time. |
| M7 | Demo | done | turtlesim: planner, teleop, e-stop lock, with a narrator naming theorems. |

### Not done

- **CI for the ROS side.** The GitHub workflows are the Lean project
  template: they run `lake build` (the proofs). Nothing in CI builds the
  ROS packages or runs the tests.
- **Linux beyond the container.** The Docker build (`docker/Dockerfile`,
  Ubuntu 24.04, apt ROS) builds, passes all tests against apt's upstream
  `twist_mux` and yields an install tree that needs no Lean installation.
  Linux through pixi/RoboStack (Ubuntu 24.04, x86-64): `pixi run build`
  and `pixi run test` pass, and the turtlesim demo runs. Not tried there:
  `pixi run prove`, arm64, and an install tree copied to another machine.
- **Packaging.** No Debian or conda package. A ROS build farm release would
  need the generated C and a Lean runtime vendored, since the farm has
  neither Lean nor network.
- **Size on Linux.** The install tree carries Lean's whole shared runtime
  (190 MB stripped) for a model of a few hundred lines. A runtime built from
  source as position-independent code would bring this down to a few MB.
- **Gazebo / Nav2 demo: done on macOS, with gaps.** A TurtleBot3 in a
  furnished apartment, with a patrol script or Nav2 as the planner
  (`demo/gazebo/`). On Linux it has run only without the furniture and
  with software rendering (see the tutorial). Not done: a Linux run of the
  furnished apartment on a desktop, and an
  automated test; it was checked by scripts that are not in the repository.
- **Stretch: silence leads to zero.** A timer and a proved stop command, as a
  deliberate difference from upstream.
- **Upstream extras.** Diagnostics, `twist_marker`, `joystick_relay`, launch
  file (D3, D9).

## The collision monitor

Goal: a Lean replacement for the core of Nav2's `nav2_collision_monitor`
that Nav2 can launch and manage in place of its own monitor, for `stop` and
`slowdown` zones, with proved decision logic, tested against the upstream
binary, and a precise public statement of what is still missing
([`site/gaps.html`](../site/gaps.html)). It is not a drop-in replacement for
Nav2's default configuration, which uses the `approach` action.

The decisions, with who took each and which are defaults that the user has
not confirmed, are in [`tasks/PLAN.md`](../tasks/PLAN.md), "Decisions". The
ones that shaped the work: ROS 2 Jazzy; integers in the Lean core; match
upstream and number every difference; the sensor-to-base transform applied
in C++ as upstream applies it (decided by the user on 2026-11-24, after it
had been built in Lean in exact arithmetic and measured at about 20 times
upstream's time per command); every question of the form "fix upstream's
weakness, or reproduce it" answered "as upstream".

### Milestones

The numbers CM1 to CM5 are those under which [REVIEWS.md](REVIEWS.md)
records the independent reviews. Steps without a review of their own are
named by their task (`tasks/T??-*.md`; each has a report in
`tasks/reports/`). Everything was built and run on macOS arm64; "Linux" is
an arm64 container in Docker on the same machine unless said otherwise.

| | Milestone | Status | Result |
| --- | --- | --- | --- |
| CM1 | Specification from the upstream source | done | [`collision_monitor/SPEC.md`](collision_monitor/SPEC.md): rules R1–R19 for one scan source, circle and polygon zones, `stop` and `slowdown`; later N1–N36 for the Nav2 extension. Differences D1–D15 and ND1–ND13, trusted parts T1–T10, assumptions A1–A4. |
| CM2 | Lean model | done | `core/ROSleanCore/CollisionMonitor.lean`: integer millimetres and nanoseconds, bit patterns for everything that is only compared, no `Float`. Review CM1/CM2: no rule contradicts the source; 18 findings, at the shim boundary and in the properties, answered. |
| | Proofs | done | `ROSlean/CollisionMonitor/`: an if-and-only-if characterisation of each action and each output, the property table, trace theorems, the lifecycle, the margin, the conversions, the C interface end to end. [`collision_monitor/THEOREMS.md`](collision_monitor/THEOREMS.md). No `sorry`, no `native_decide`, the three standard axioms. |
| CM3 | C interface | done | `core/ROSleanCore/CollisionMonitorFFI.lean`. Every parameter crosses as the bit pattern the shim read; conversions and refusals are in Lean. |
| CM4 | Node | done | `ros/roslean_collision_monitor`. Review CM3/CM4: no conversion, refusal, ownership or signature defect; eight findings around the node. Fixed, and verified by T01: each fix removed in turn makes a test fail. |
| | Upstream harness | done | `ros/roslean_collision_monitor/test/`, [`UPSTREAM_BEHAVIOUR.md`](collision_monitor/UPSTREAM_BEHAVIOUR.md): what upstream's binary does, written without sight of the SPEC. 55 scenarios with 277 checks and a random differential test against a reference model. |
| | Gazebo demo behind the mux | done | `demo/gazebo/`, [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md). The headless check passed in nine runs of nine. |
| T00 | Cloud environment | done | `scripts/cloud_setup.sh`. The one run on Linux x86-64 so far: build, 11 tests, all theorems, the headless Gazebo check, on the node as it then was. |
| T02 | Comparison with upstream | done | [`COMPARISON.md`](collision_monitor/COMPARISON.md). 720 random cases with 65,056 commands and 34.0 million point tests on each of macOS and Linux: no divergence in the unsafe direction, the zones grow as stated and no more. Found two defects: a negative `/clock` ended the node, and a command was 50 to 70 times slower than upstream's. |
| T03 | The two defects | done | D15: the node answers with zero while the clock is unusable (`stepNoClock`). A second command path over machine words, proved equal to `step` (`stepCmdFast_eq`); a budget for the cost of a command, asserted by a test. |
| T04 | Specification of the Nav2 extension | done | [`SPEC_NAV2.md`](collision_monitor/SPEC_NAV2.md): lifecycle and state topic, transforms from TF with base shift correction, zones fed by a topic; with probes of upstream's binary. |
| T05, T05b | Extension: model and proofs | done | T05 built the transform into the model in exact arithmetic (option B). T05b moved it out again on the user's decision (option A) and reworked the proofs: the model receives base-frame coordinates with each command. Two statements of the design were false of the model it specified; the theorems state what is true. |
| T07 | Extension: upstream scenarios | done | `scenario_nav2.py`, `random_nav2.py`, `bond_manager.py`: now 46 scenario runs with 299 checks on upstream. No contradiction with SPEC_NAV2; findings 24 to 38 of UPSTREAM_BEHAVIOUR.md. |
| T06 | Extension: C interface and node | done | A lifecycle node with the bond, the state topic, the zone polygons, TF, zones fed by a topic; as an executable and as a component. Nav2's own launch file brings it up. A command costs 6 to 9 times upstream's and is within the budget. |
| CM5 | Independent review of the extension (T08) | done | The node answers every scenario and 600 random cases as upstream up to the stated differences. Six findings, two of them defects: a pass where upstream stops after a polygon message with a NaN vertex, and a hang when the clock jumps back during a wait for a transform. |
| T09 | Fixes after CM5 | done | ND6, the wait and the TF buffer at a clock jump, unrecorded differences removed or recorded (ND13), `mixed_from_init`, `lean_initialize_thread`. SPEC.md is one document again. Final counts: COMPARISON.md, "After the fixes of the review CM5". |
| T10 | Demo: Nav2 launches the Lean node | done | [TUTORIAL_COLLISION_MONITOR.md](TUTORIAL_COLLISION_MONITOR.md), section 7; `tasks/reports/T10.md`. Nav2's whole bring-up starts the Lean executable as its monitor, with a stop and a slowdown zone in place of `FootprintApproach`. `pixi run cm-check --nav2`: 10 runs of 10 passed with the Lean node, 4 of 4 with upstream's on the same parameters; the first start in a fresh environment failed (Nav2 did not come up). With Nav2 driving a tour through the stop zone the robot ended standing for good in three runs of six (two of three with the Lean node, one of three with upstream's); stock Nav2 with `approach` reached 11 places in its one run and did not stand: `stop` is not a substitute for `approach`. Nobody looked at a window (locked screen). |
| T11 | Project page, README, this plan | done | `site/gaps.html`, the collision monitor on the other pages. |
| T12 | Fresh-machine audit | open | Someone with nothing but the README builds and runs it, and checks every number on the project page. |

### Not done

What the node does not do, by feature, with what closing each gap would
take, is on [`site/gaps.html`](../site/gaps.html) and in SPEC_NAV2.md,
section 12: `approach` (Nav2's default; about half of the work so far),
`limit`, velocity polygons, a circle's radius on a topic, point cloud, range
and polygon sources, several sources, `~/toggle` and run-time parameters,
point markers. What follows is the rest.

- **`pixi run test` as a whole on the final tree.** Each task ran
  `test-quick` and the long tests it was about; the plan leaves the full
  run to the end of the effort.
- **x86-64, and Linux on its own hardware.** The node with the Nav2
  extension has run on macOS arm64 and in a Linux arm64 container only.
  Whether to make such a run before publication is not decided
  (tasks/PLAN.md).
- **Timing and memory of the final node.** In-process timing is from macOS
  (T06). Not repeated after T06: timing in the Linux container, the soak
  (32 minutes, on an earlier version), a transform from the TF buffer in
  the timed loop, base shift correction, topic zones.
- **Proofs.** The topological half of the polygon margin argument; the
  slowdown multiplication (T1); the transform code (T2, T6, T7); the
  IEEE-754 reading of bit patterns (T4); that an accepted timeout is
  upstream's nanosecond count. THEOREMS.md, "What is not proved".
- **Runs with a command without a clock reading, on the exported
  functions.** `mixed_from_init` is on the model's states; there is no
  `cm_readBack_after` for mixed sequences of calls (review CM5, finding 4).
- **ND3's delay.** A stop behind a failing lookup of a later topic zone is
  published late. Documented from source, not run, not removed: removing it
  needs a second call into the core per command.
- **The component on a negative `/clock`.** That it ends with its
  container is read from source and was never run. D15 is promised for the
  stand-alone executable only.
- **`lean_initialize_thread` on Linux.** What the function does was checked
  by disassembly for the macOS arm64 toolchain only. The mux's wrapper
  (`mux.hpp`) prepares only the thread that builds the mux; the mux node is
  single-threaded.
- **State messages in the older scenario suite** are not compared for the
  Lean node (`STATE_TOPIC = False` in `profile_roslean.py`): the profile's
  format cannot state the 32 checks that follow from stated velocity
  differences. `scenario_nav2.py` and `random_nav2.py` compare them, without
  boundary cases.
- **The demo on a screen.** Nobody has seen the Gazebo demo, in either
  variant, in a window: the screen of the machine was locked during T10 and
  during the audit (T12). The two pictures in the tutorial are Gazebo's own
  captures of the 3D scene. RViz was started and not looked at; the
  keyboard was not used. (T10 rewrote the tutorial for the node as it is
  and replaced the four theorem names that the Lean sources do not
  contain, in the tutorial and in `demo/gazebo/monitor_narrator.py`.)
  `check_demo.py` does not notice when Gazebo exits early and waits for
  minutes (T00).
- **A stop zone under Nav2's planner.** With the documented edit (a stop
  zone in place of `FootprintApproach`) Nav2's tour of the apartment ended
  with the robot standing for good in three runs of six, with both nodes:
  the monitor zeroes Nav2's spin and back-up recoveries as well. Stock Nav2
  with `approach` did not stand in its one run. This is the behaviour of a
  `stop` zone, not a defect of the node; what would remove it is `approach`
  (above). Whether the documented edit should stay a stop polygon, and the
  demo's margins, are T10's questions for the user (`tasks/reports/T10.md`).
  Not investigated: the flicker between stop and slowdown at the zone's
  edge, seen in 7 of 10 runs of the Nav2 variant (3 of 4 with upstream's
  node); the one failed start of Nav2 in 28.
- **The component in the simulator**, and how close `approach` lets the
  robot come: not run, not measured (T10).
- **Open for the user** (tasks/PLAN.md, "Open or not confirmed"): the
  default for a polygon message with an unusable vertex (ND6); whether a
  marked slowdown zone should stop; ND13; the cost target.
- **CI.** As for twist_mux: nothing in CI builds the ROS packages or runs
  the tests.

## Lessons from the spike (M0)

These changed the plan above.

1. **The core must be a separate, Mathlib-free library.** Linking anything
   that imports Mathlib into a node is impractical, so the model and the proofs
   live in two `lean_lib`s. This is why the architecture splits `ROSleanCore`
   from `ROSlean`.
2. **Lake prefixes names with the package.** The initialiser is
   `initialize_ROSleanCore_ROSleanCore(uint8_t builtin)`, with one argument.
   Older tutorials show a different name and a second `world` argument. The
   generated C in `.lake/build/ir/` is the authority.
3. **The generated C is not standalone, and how to supply the runtime
   differs by platform.** The spike made the node load `libleanshared` from
   the elan toolchain through an absolute rpath, so the install tree could
   not be copied anywhere. Since 2026-11-20 `roslean_core` compiles the
   generated C itself into `libroslean_core` and puts the runtime in the
   install tree: on macOS linked in statically (4 MB), on Linux as a
   stripped copy of `libleanshared.so` (see lesson 14). Programs that use
   the library call the Lean runtime only through four helper functions in
   `src/runtime.c`, so they need neither Lean headers nor a link to the
   runtime. Lean is then a build-time dependency only.
4. **On macOS the node aborts at load unless unused dylibs are stripped.**
   ROS message targets pull in `*_rosidl_generator_py` libraries that expect
   Python symbols from the host process. `-Wl,-dead_strip_dylibs` fixes it;
   `roslean_core` passes it on to its dependants.
5. **colcon does not know about Lean.** `roslean_core`'s CMake runs
   `lake build ROSleanCore` on every build. A changed `@[export]`
   signature used to go unnoticed by C++; `roslean_core` now compiles
   `ffi.h` against the Lean-generated C, so a mismatch fails the build.
   One blind spot remains: Lean passes `Int64` as `uint64_t`, so signedness
   is not checked.
6. **Lean is only initialised for the main thread.** The spike uses the
   default single-threaded executor. A multi-threaded executor would need
   `lean_initialize_thread` per thread; M4 stays single-threaded, which also
   matches the model of one event at a time.
7. **Builds are fast.** A full colcon build of the node takes about 15 s and
   an incremental one under a second, so no extra build caching is needed.

## Risks

- **Lean's runtime inside a ROS process on Linux.** `libleanshared.so`
  exports symbols that also exist elsewhere in a ROS process (an exception
  unwinder, GMP, libuv). The unwinder clash is handled (lesson 14) and the
  tests pass; a node that also loads the system GMP or libuv has not been
  tried.
- **Thin proofs.** As noted in the scope doc; the pipeline and shim are the
  main deliverable.

## Later lessons

8. **Exported Lean functions cannot borrow.** `@&` on an `@[export]`
   parameter is a compile error in this Lean version; every object argument
   is consumed, so the C++ wrapper takes an extra reference before queries.
9. **`omega` does not see through type aliases.** `abbrev Priority := Nat`
   made arithmetic goals fail; the model uses plain `Nat`.
10. **Read the source, then test against the binary.** The documentation
    misses several behaviours (never-expiring inputs live from start-up,
    priority 0 never selected, 255 never lockable), and the source alone
    would not have shown how often floating-point rounding flips the
    comparison exactly at a timeout boundary (D2).
11. **Simulated time makes ROS tests deterministic.** Publishing `/clock`
    from the test fixes the time of every event, which made 1 ns boundary
    checks and a randomised differential test against upstream practical.
12. **Independent review after each milestone paid off.** It found, among
    others: state updates that could be silently dropped, missing clamping
    in the model, no theorems tying the C interface to the model, two
    confounded test expectations, and one false theorem attempt of mine was
    caught by Lean itself.
13. **"It compiles to C" is true and is not the hard part.** The generated C
    built unchanged with gcc on Ubuntu. The work was in supplying the Lean
    runtime in a way that is sound inside a ROS process.
14. **Two Linux traps, both silent on one architecture.** (a) Lean's static
    runtime (`libleanrt.a`) is compiled for thread-local storage in
    executables. Linking it into a shared library is an error on x86-64, but
    on arm64 the linker accepted it and all tests passed, although such a
    library is not valid. Only the x86-64 build exposed it. (b) Lean's shared
    runtime exports its own `_Unwind_*` functions; when the loader finds
    them before libgcc's, a C++ `catch` no longer catches (the FFI test
    aborted on the first expected exception). The fix is to keep
    `libleanshared` out of the executable's direct dependencies and to list
    libgcc before it. Test every target architecture, and keep a test that
    throws and catches.
15. **Keep the compiled core in its own lake package.** While the model and
    the proofs shared a package, building the node made lake fetch Mathlib.
    `core/` now has no dependencies; the root package requires it by path,
    and CMake checks that both pin the same toolchain.
16. **A physics simulator shows what the proofs leave to others.** In
    turtlesim the e-stop "worked". In Gazebo the robot kept rolling with the
    e-stop engaged: the mux correctly forwarded nothing, and the simulated
    drive kept executing its last command. The proofs say when a message is
    forwarded; that the robot stops when none is, is the base's job (SPEC,
    A3). The Gazebo demo needs a base watchdog for this, which is what the
    "silence leads to zero" stretch goal would replace with a proved rule.
17. **Wheel odometry is not a position.** The patrol script first steered
    by odometry and ended up in the wrong room within one lap (wheel slip in
    turns). It now uses the simulator's ground truth; Nav2 uses lidar
    localisation.

## Lessons from the collision monitor

Collected from the "Friction" and "Found wrong" sections of the task
reports (`tasks/reports/T00.md` to `T09.md`) and from the reviews. The
numbering continues.

About the method:

18. **Exact arithmetic in the model is affordable for decisions, not for
    every point of a scan.** The transform in Lean in exact rational
    arithmetic gave the strongest theorem (every scan point is the exact
    image of its reading) and cost about 20 times upstream's time per
    command (T02, T05). It was replaced by upstream's floating-point
    formula in C++, with the distance between the two as an explicit
    hypothesis of the margin theorems (`contains_of_shim`). Measure the
    cost of a design before proving it.
19. **A slow model can be kept if a fast path is proved equal to it.** The
    command path the node calls works on machine words and bounding boxes;
    `stepCmdFast_eq` says it is `step`. Every other theorem stayed as it
    was, and the cost of a command with 1080 readings went from 208 µs and
    640 µs to under 50 µs (T03).
20. **The prover corrects the design, and so does the binary.** Two
    properties written into the design were false of the model it
    specified; the theorems state what is true instead, each with its
    counterexample (THEOREMS.md, "Where SPEC_NAV2.md, section 4, was false
    as stated"). One conclusion drawn
    from a single probe of the binary was false too: "a NaN vertex blinds
    upstream's zone" held for the one arrangement tried, and the node built
    on it passed where upstream stops (review CM5, finding 1). One
    arrangement is not a rule.
21. **Write the specification from the source and the behaviour from the
    binary, apart, then compare.** SPEC.md and UPSTREAM_BEHAVIOUR.md were
    written without sight of each other. They disagreed in three places;
    the binary sided with the SPEC in two and with the other document in
    one, and the 18 scenarios written for statements that only the SPEC
    made all passed on the binary (COMPARISON.md).
22. **Every test must be shown able to fail.** T01 found two fixes that no
    test noticed when removed; the review CM5 changed the code in 19 places
    and the tests caught 17. Such checks need a second copy of the
    packages, because the colcon tests read files from the source tree
    while they run. `WILL_FAIL` cannot say why a test fails: a harness
    error makes an expected failure pass, so such tests print one
    distinctive line.
23. **Under simulated time a node that waits looks exactly like a node
    that lost a message.** A probe that forgets to advance the clock sees
    "no answer" and cannot tell a stop from a hang (T04, T07). A clock that
    ran backwards by mistake in two scenarios is how upstream's forgetting
    of transforms at a clock jump was found.
24. **A measuring task needs a frozen commit.** The integration branch
    moved under the comparison while its measurements ran, and everything
    was run twice (T02). The second set found the largest defect.
25. **The comparison is real time.** One pass on one platform is about 75
    minutes of machine time, most of it waiting on real nodes. Parallel
    runs work, each on its own `ROS_DOMAIN_ID`; with sixteen test processes
    at once one random case and one twist_mux scenario needed a retry.
    With `--symlink-install` a rebuild replaces the binaries under running
    tests, so nothing can be built while a long comparison runs.
26. **Sessions stop without saying so.** Two sessions ended before they
    reported and left unverified branches, which two later tasks had to
    verify and finish (T01, T02). Since then every task writes a progress
    line and commits after each step, and a watch script looks for
    worktrees in which nothing changes (`tasks/PLAN.md`, "Watching the
    tasks"). A table of which task may change which files kept parallel
    tasks apart.

About Lean:

27. **`decide` in the elaborator has limits the kernel does not.** It does
    not evaluate a power with an exponent above 256, and it fails with
    "maximum recursion depth" on a list computation the kernel checks at
    once, without naming the function (T05, T05b).
28. **`simp only [decide_eq_true_eq] at h` can leave `decide (…) = true` in
    place**, report the lemma as unused, and `omega` then ignores the
    hypothesis silently. `of_decide_eq_true` by hand (T03, T05b).
29. **`cases h : f x` does not rewrite under an unreduced `match`,** and
    rewriting under the large structure literal of `step` fails with
    "motive is not type correct". Rewrite the hypothesis first, then
    `simp only […, h]`. `split` does not see through a `let`; `dsimp only`
    first (T03, T05, T05b).
30. **A structure update in a theorem statement breaks when the structure
    gains a field.** A third field in `TopicShape` broke only what
    constructs one by position, and seven statements; the fast path needed
    no change because it is stated on a function, not on fields (T03, T09).
31. **Machine-word proofs were easier than feared:** `UInt64.toNat_div`,
    `toNat_mod`, `toNat_add`, `toNat_mul`, `toNat_shiftRight`,
    `le_iff_toNat_le` and `omega` (T03).
32. **Links to line numbers rot.** THEOREMS.md links every theorem to its
    line; every edit of a proof file moves them. Two tasks wrote the same
    twenty-line script to look each name up again and neither committed it
    (T05b, T06).

About ROS and the tooling:

33. **Lesson 6, revisited: Lean in threads Lean did not create.** As a
    component the node runs in its container's executor threads. The
    runtime requires `lean_initialize_thread` there; in the pinned
    toolchain the call happens to do nothing, so its absence showed in no
    test and was found by reading (review CM5, finding 5; SPEC.md, T10).
34. **A component gets no global arguments,** so parameters on the
    container's command line do not reach it; and the TF listener declares
    parameters during `on_configure`, so a node that rejects parameter
    changes must let those through (T06).
35. **Do not hammer a managed node's lifecycle services.**
    `nav2_lifecycle_manager` failed its bring-up while a probe polled
    `get_state` in a loop; with one query it succeeded. Cause not
    investigated (T04).
36. **The hand-written list of Lean modules** in
    `ros/roslean_core/CMakeLists.txt` breaks the link whenever the core
    gains a module (T05).
37. **A blocked download host can cost hours without an error.** With
    `cache.mathlib.org` blocked, `lake exe cache get` tries 9000 files one
    after the other and does not fail (T00). `scripts/cloud_setup.sh`
    probes the host first.
38. **Small things that cost time in every session:** macOS has no
    `timeout` command (a two-line `perl` wrapper does); `colcon`'s
    `setup.bash` reads unset variables, so a script with `set -u` dies;
    Python buffers a redirected log (`PYTHONUNBUFFERED=1`); the
    repository's `.dockerignore` does not exclude `core/.lake`, which on a
    Mac holds macOS objects; and the sessions' own shell guard refused
    compound commands, so multi-line steps went through script files.
