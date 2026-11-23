# ROSlean development plan

Scope: [`twist_mux.md`](../twist_mux.md). Goal: a drop-in replacement for the
ROS 2 `twist_mux` node whose arbitration logic is a Lean function with proved
properties, plus a reusable Lean-to-ROS shim.

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
  Not tried: Linux through pixi/RoboStack.
- **Packaging.** No Debian or conda package. A ROS build farm release would
  need the generated C and a Lean runtime vendored, since the farm has
  neither Lean nor network.
- **Size on Linux.** The install tree carries Lean's whole shared runtime
  (190 MB stripped) for a model of a few hundred lines. A runtime built from
  source as position-independent code would bring this down to a few MB.
- **Gazebo / Nav2 demo: done on macOS, with gaps.** A TurtleBot3 in a
  furnished apartment, with a patrol script or Nav2 as the planner
  (`demo/gazebo/`). Not done: any Linux run, and an
  automated test; it was checked by scripts that are not in the repository.
- **Stretch: silence leads to zero.** A timer and a proved stop command, as a
  deliberate difference from upstream.
- **Upstream extras.** Diagnostics, `twist_marker`, `joystick_relay`, launch
  file (D3, D9).

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
