# Lean for ROS, choice 2: twist_mux

2026-11-18 · ivan Gavran

Source doc: https://claude.ai/artifact/1hWP2MRQvpNEWHnAZbGzb3

This file is the original scope for the project, kept up to date with what
was built. If you are new to the repository, read the next section first.

## Start here

**What this is.** A re-implementation of the ROS 2 `twist_mux` node in which
the decision "forward this message or drop it" is made by a Lean function,
and the rules that function obeys are Lean theorems. The C++ that remains
only reads parameters, subscribes, reads the clock and publishes.

**Status (2026-11-20).** Everything in the build plan below is done except the
Gazebo/Nav2 demo (turtlesim is used) and the optional "silence leads to zero"
rule. The node accepts the same configurations as upstream `twist_mux` 4.5 (except
non-finite or absurdly large timeouts, SPEC D5) and
makes the same decisions in every comparison run so far; the one known
exception is at exact timeout boundaries ([docs/SPEC.md](docs/SPEC.md), D2).
Built and tested on macOS arm64 only; Linux is untested.

**Set-up.** Install [elan](https://github.com/leanprover/elan) (Lean) and
[pixi](https://pixi.sh) (ROS 2 Jazzy from RoboStack) and put both on `PATH`.
Nothing else is needed; no system ROS installation is used.

```sh
pixi run build     # Lean core + the three ROS packages (first run downloads about 1 GB)
pixi run test      # C interface tests, then both nodes side by side against upstream
pixi run prove     # check every theorem (first run fetches Mathlib)
pixi run demo      # turtlesim: planner, keyboard teleop and an e-stop lock
```

**Reading order.**

1. The rest of this file: why `twist_mux`, what the package does, what was to
   be proved.
2. [docs/SPEC.md](docs/SPEC.md): the rules R1–R16 as read from the upstream
   C++ source, and each place where this node deliberately differs (D2–D9).
   The Lean code and the tests cite these numbers.
3. [`ROSleanCore/Mux.lean`](ROSleanCore/Mux.lean): the model, about 150
   lines. `step` is the whole node. Then
   [`ROSlean/Mux/Examples.lean`](ROSlean/Mux/Examples.lean) for worked
   scenarios.
4. [`ROSlean/Mux/Properties.lean`](ROSlean/Mux/Properties.lean): the
   theorems. Start at `forward_iff`; the rest are corollaries.
   [`Trace.lean`](ROSlean/Mux/Trace.lean) lifts them to event sequences and
   [`FFI.lean`](ROSlean/Mux/FFI.lean) ties them to the functions the node
   calls. The README has a table of theorem names and a list of what is
   *not* proved.
5. [`ros/roslean_twist_mux/src/twist_mux_node.cpp`](ros/roslean_twist_mux/src/twist_mux_node.cpp):
   the C++ shim, about 200 lines.
6. [docs/PLAN.md](docs/PLAN.md) for the build set-up and its lessons, and
   [docs/REVIEWS.md](docs/REVIEWS.md) for what independent reviews found.

**Where things live.**

| Path | Contents |
| --- | --- |
| `ROSleanCore/` | Lean without Mathlib: the model and its C interface. This is what gets compiled and linked into the node. |
| `ROSlean/` | Lean with Mathlib: theorems about `ROSleanCore`. Never linked into a node. |
| `ros/roslean_core` | ROS package that builds the Lean library and wraps it for C++ (`mux.hpp`). |
| `ros/roslean_twist_mux` | The node, its tests against upstream, and the demo. |
| `ros/roslean_noop` | A node that calls one trivial Lean function: the smallest example of the Lean-to-ROS pipeline. |

**Changing something.**

- *A rule:* edit `step` or its helpers in `ROSleanCore/Mux.lean`, then run
  `pixi run prove` (the theorems that no longer hold will fail) and
  `pixi run test` (the comparison with upstream will show the difference).
  Record a deliberate difference in `docs/SPEC.md`.
- *The C interface:* an `@[export]` in `ROSleanCore/FFI.lean` must be mirrored
  by hand in `ros/roslean_core/include/roslean_core/ffi.h` and `mux.hpp`. The
  build fails if the C prototypes disagree with what Lean generated.
- *A new verified node:* copy the pattern of `roslean_noop`: a Lean function
  with `@[export]`, a prototype in `ffi.h`, and a package that links
  `roslean_core::roslean_core`.

## Why it ranks second

`twist_mux` is the most doable candidate and has crisp rules to prove, but it ranks below the collision monitor because its rules are about who controls the robot, not about obstacles.

| Criterion | Rating | Reason |
| --- | --- | --- |
| Doability | High | Two message types, no geometry, no sensor data, no coordinate frames. The node forwards messages unchanged. |
| Safety properties | Medium to high | Arbitration rules are discrete and fully provable. They matter in practice: a joystick or emergency lock must override the planner. |
| Proof effort | Low | No arithmetic on velocities. The only numbers compared are priorities and timestamps, both integers. |

It is also the right first build regardless of ranking, as set out in the [main feasibility doc](https://claude.ai/artifact/Qo4d1tB8db3fYY91fmNoeX): it exercises the whole Lean-to-ROS pipeline with the least proof work.

## What the package does

`twist_mux` takes several velocity command topics and forwards exactly one of them to the robot, chosen by priority, freshness and locks ([package wiki](https://mirror.umd.edu/roswiki/twist_mux.html), [ROS Index](https://index.ros.org/p/twist_mux/)).

| Element | Rule as documented |
| --- | --- |
| Inputs | Each has a name, topic, timeout in seconds and a priority from 0 to 255. Higher number wins. |
| Input timeout | If no message arrives within the timeout, other inputs can be selected. A timeout of 0 means the input never expires. |
| Locks | Boolean topics with their own priority and timeout. A lock inhibits inputs; the wiki example implies only those of lower priority. |
| Lock timeout | Used to detect a dead lock publisher "in order to enable the lock". A timeout of 0 disables the check. |
| Output | The selected message, republished unchanged. |
| When it decides | In the input callbacks, each time a message arrives. There is no timer. |

Three things are left open by the documentation and were settled from the C++ source ([docs/SPEC.md](docs/SPEC.md)):

- **Equal priorities.** The first live input in the node's handle list wins. That list is in ASCII order of the handle names (upper case before lower case), not the order in the YAML file.
- **Lock against an input of equal priority.** Not blocked; a lock blocks only strictly lower priorities.
- **Dead lock publisher.** A lock with a timeout whose publisher is silent past it counts as locked, including one that has never published.
 Version 4.4.0 added stamped messages; current releases are 4.5.1 on Jazzy, Kilted and Lyrical.

## Lean model and properties

The whole node reduces to one pure function over a small state, and every documented rule becomes a theorem.

### Model

- **Config.** A list of inputs and a list of locks, each with a priority and a timeout. Inputs may share a priority; the earlier one in the list then wins, as upstream. (An earlier version rejected equal priorities at start-up, with a distinctness proof in the type. That refused upstream's own example config, so it was changed back.)
- **State.** For each input, the time of its last message. For each lock, its last value and time.
- **Events.** A command on input *i* at time *t*, or a lock value on lock *j* at time *t*. Time is an integer count of nanoseconds, supplied by the shim.
- **Step.** `step : Config → State → Event → State × Bool`. The boolean says whether to forward the message just received. The message body never enters Lean, so no floating point is involved.

### Properties

| Property | Statement |
| --- | --- |
| Provenance | Every output is a message that arrived on some configured input, unchanged. This holds by construction, since the core only says yes or no. |
| Priority | A message on input *i* is forwarded only if no input of higher priority is live, and no earlier input of the same priority is live. |
| Lock | A message on input *i* is forwarded only if no active lock outranks it. |
| Dead lock publisher | A lock whose publisher has gone silent past its timeout counts as active. |
| Expiry | An input that has timed out never suppresses another input. |
| Liveness | If input *i* is the highest-priority live input (the first such, on a tie) and no active lock outranks it, its message is forwarded. |
| Trace invariant | The above hold after any sequence of events, by induction from the initial state. |

### A finding worth stating up front

Because the original decides only when a message arrives, it publishes nothing when every input goes silent. It does not send a stop command. Stopping then depends on the base driver's own watchdog. The Lean version matches this and states the assumption (SPEC, A3). Adding a timer and a proved "silence leads to zero" rule remains an option; it has not been started.

## Build plan

As a standalone project this is about four days for two people; most of that is the shim and build set-up, which every later Lean node reuses. This is the plan as first written; [docs/PLAN.md](docs/PLAN.md) records what was done. Two departures: the spike and the demo use turtlesim, not Gazebo and Nav2, and step 6 runs both nodes live on the same scripted and randomised messages under simulated time, not a recorded session.

1. **Read the C++ source** and write the three open rules down as plain sentences. Half a day.
2. **Spike the pipeline:** a Lean function exported to C, linked into a C++ ROS node, driving a robot in Gazebo. One to two days.
3. **Write the model and `step`** in Lean, with unit examples checked by `#eval`. Half a day.
4. **Prove the property table.** The proofs are case analysis over lists and integer comparisons. One day.
5. **Wire the shim:** one subscription per configured input and lock, the same parameter file format as the original. Half a day.
6. **Compare with the original:** record a session with keyboard teleop, Nav2 and a lock toggled by hand, replay it through both nodes, and compare which messages each forwards. Half a day.

### Running it

Use a TurtleBot-class robot in Gazebo with three inputs: Nav2 at low priority, keyboard teleop above it, and a lock topic above both. The demonstration is short: Nav2 drives, the keyboard takes over, the lock freezes everything, and each transition matches a named theorem.

## Risks and open questions

The main risk is that the result looks too simple to be interesting on its own, which is why it is positioned as the first step and not the headline.

- **Thin proofs.** The theorems are true and useful but short. The value is the end-to-end pipeline and a reusable shim.
- **Specification taken from documentation.** The wiki describes the ROS 1 version. The ROS 2 source is the authority, and the dead-lock-publisher rule in particular needs confirming.
- **Dynamic configuration.** Inputs are defined in a parameter file, so subscriptions are created at start-up. The Lean config is built then, in the same handle order as upstream, since that order decides ties.
- **Stamped and plain messages.** Both exist across releases. Since the core never reads the message body, this affects only the shim.

Decisions taken:

- [x] Silence: match the original (publish nothing). A timer and a zero command stay a possible extension.
- [x] Equal priorities: reproduce the original (first live input in handle order wins).

Still open: Linux build and deployment, a Gazebo/Nav2 demo, and CI for the ROS packages (the GitHub workflows check the Lean proofs only).

## Sources

- [twist_mux package wiki (mirror)](https://mirror.umd.edu/roswiki/twist_mux.html)
- [twist_mux on ROS Index](https://index.ros.org/p/twist_mux/)
- [Main feasibility doc](https://claude.ai/artifact/Qo4d1tB8db3fYY91fmNoeX)
