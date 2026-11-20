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
ROSleanCore/   Lean, no Mathlib. Executable model: Config, State, Event, step.
               Built as a shared library and loaded by the ROS node.
ROSlean/       Lean + Mathlib. Theorems about ROSleanCore. Never linked into a node.
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
| M2 | Lean model | done | `ROSleanCore/Mux.lean`, examples in `ROSlean/Mux/Examples.lean`. |
| M3 | FFI surface | done | `ROSleanCore/FFI.lean`, `ros/roslean_core` (C++ wrapper, build-time signature check, tests). |
| M4 | `roslean_twist_mux` node | done | Same parameter format, topics and QoS as upstream. Accepts every configuration upstream accepts, including its example config with two inputs at priority 100. |
| M5 | Proofs | done | `ROSlean/Mux/{Basic,Properties,Trace,FFI}.lean`; table in the README. |
| M6 | Comparison with upstream | done | Fixed scenarios with exact boundaries and a randomised differential test; no divergence outside D2. This replaces the recorded teleop/Nav2 session of the scope doc with scripted and random event sequences under simulated time. |
| M7 | Demo | done | turtlesim: planner, teleop, e-stop lock, with a narrator naming theorems. |

### Not done

- **CI for the ROS side.** The GitHub workflows are the Lean project
  template: they run `lake build` (the proofs) and a docs deployment that
  needs GitHub Pages enabled. Nothing in CI builds the ROS packages or runs
  the tests.
- **Linux.** Nothing has been built or run on Linux. The CMake rpath handling
  for Linux is a guess (marked UNVERIFIED in `lean_core.cmake`).
- **Deployment.** The node loads the Lean runtime from the elan toolchain and
  the core library from `.lake/build`; the install tree is not relocatable.
  Static linking is the intended fix.
- **Gazebo / Nav2 demo.** `ros-gz-sim` exists on RoboStack for macOS arm64
  but was not tried; the demo uses turtlesim and a scripted planner.
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
2. **Lake prefixes names with the package.** The library is
   `libROSlean_ROSleanCore.dylib` and the initialiser is
   `initialize_ROSlean_ROSleanCore(uint8_t builtin)`, with one argument.
   Older tutorials show a different name and a second `world` argument. The
   generated C in `.lake/build/ir/` is the authority.
3. **The core library does not carry the Lean runtime.** Lake builds it with
   undefined runtime symbols, so the node itself must link `libleanshared`
   and carry an rpath to the elan toolchain. This works for development but
   the binary is not relocatable. Deploying to a robot needs either static
   linking of `libleanrt` / `libInit` or shipping the toolchain libraries;
   this is new work and has not been done.
4. **On macOS the node aborts at load unless unused dylibs are stripped.**
   ROS message targets pull in `*_rosidl_generator_py` libraries that expect
   Python symbols from the host process. `-Wl,-dead_strip_dylibs` fixes it.
   The CMake logic for this and for finding Lean lives in
   `ros/roslean_core/cmake/lean_core.cmake`.
5. **colcon does not know about Lean.** `roslean_core`'s CMake now runs
   `lake build ROSleanCore:shared` on every build. A changed `@[export]`
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

- **Simulator on macOS.** Gazebo and Nav2 under RoboStack on osx-arm64 are
  unverified; the demo uses turtlesim.
- **Linux parity.** Everything so far is verified on macOS arm64 only.
  `pixi.toml` lists Linux platforms, but the rpath and dylib handling in
  CMake has not been exercised there.
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
