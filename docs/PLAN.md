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
| Equal input priorities | Rejected at start-up; the Lean `Config` carries a distinctness proof |
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

### M0. Pipeline spike — done

`ros/roslean_noop`: an `rclcpp` node that initialises the Lean runtime and
calls `ROSleanCore.noop` once per second. Verified with `ros2 node list` and
`ros2 topic echo` from a second process. Run it with `pixi run noop`.

### M1. Pin the specification from the upstream source

Read `twist_mux` 4.5.1 (Jazzy branch) and write down as plain sentences:

- whether a lock blocks inputs of equal priority, or only lower;
- what happens when a lock publisher goes silent past its timeout;
- what upstream does on equal input priorities (we reject, but the difference
  must be documented);
- which clock the timeouts use, and whether stamped messages use header time.

Output: `docs/SPEC.md`. Everything after this depends on it.

### M2. Lean model

In `ROSleanCore`: `Config`, `State`, `Event`, `step : Config → State → Event → State × Bool`,
and config validation that returns either a valid `Config` or an error string.
Unit examples as `#guard` / `example ... := by decide`.

### M3. FFI surface

A small, flat C API, since the shim should not build Lean structures by hand:

- `roslean_mux_new(priorities, timeouts, ...)` → opaque handle or error;
- `roslean_mux_on_input(handle, index, t_ns)` → `bool` forward;
- `roslean_mux_on_lock(handle, index, value, t_ns)`.

State lives on the Lean side behind the handle. Scalars and arrays of `UInt64`
only. A hand-written header `roslean_core.h` mirrors the `@[export]`
signatures, and a C++ test without ROS exercises it, because nothing checks
that the header and the Lean signatures agree.

### M4. `roslean_twist_mux` node

One subscription per configured input and lock, the upstream parameter file
format, `TwistStamped` first (the Jazzy default) with plain `Twist` behind the
upstream `use_stamped` parameter. Start-up fails with a clear message on
invalid config.

### M5. Proofs

In `ROSlean`: priority, lock, dead lock publisher, expiry, liveness, then the
trace invariant by induction. Provenance holds by construction. M4 and M5 can
run in parallel once M2 is stable, which suits two people.

### M6. Compare with upstream

Install upstream `ros-jazzy-twist-mux` from RoboStack, record a bag with three
inputs and a lock, replay it through both nodes, and diff the forwarded
messages. Scripted publishers are enough; Nav2 is not needed for this.

### M7. Demo

Three inputs (planner, teleop, lock) driving a simulated robot; each
transition named after its theorem. See the simulator risk below.

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
   this is new work, placed after M6.
4. **On macOS the node aborts at load unless unused dylibs are stripped.**
   ROS message targets pull in `*_rosidl_generator_py` libraries that expect
   Python symbols from the host process. `-Wl,-dead_strip_dylibs` fixes it.
   The CMake logic for this and for finding Lean is in
   `ros/roslean_noop/cmake/FindLeanCore.cmake` and should move to a shared
   location when the second package appears.
5. **colcon does not know about Lean.** `pixi run build` runs
   `lake build ROSleanCore:shared` first. A changed `@[export]` signature is
   not detected by the C++ build, which motivates the header and test in M3.
6. **Lean is only initialised for the main thread.** The spike uses the
   default single-threaded executor. A multi-threaded executor would need
   `lean_initialize_thread` per thread; M4 stays single-threaded, which also
   matches the model of one event at a time.
7. **Builds are fast.** A full colcon build of the node takes about 15 s and
   an incremental one under a second, so no extra build caching is needed.

## Risks

- **Simulator on macOS.** Gazebo and Nav2 under RoboStack on osx-arm64 are
  unverified. Check early in M7; the fallback is `turtlesim` or an RViz-only
  demo, and M6 does not depend on it.
- **Linux parity.** Everything so far is verified on macOS arm64 only.
  `pixi.toml` lists Linux platforms, but the rpath and dylib handling in
  CMake has not been exercised there. Add a Linux CI job with M4.
- **Thin proofs.** As noted in the scope doc; the pipeline and shim are the
  main deliverable.
