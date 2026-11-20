# Lean for ROS, choice 2: twist_mux

2026-11-18 · ivan Gavran

Source doc: https://claude.ai/artifact/1hWP2MRQvpNEWHnAZbGzb3

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

Three things are left open by the documentation and must be settled from the C++ source: how equal priorities are resolved, whether a lock blocks inputs of equal priority, and the exact behaviour when a lock publisher dies. Version 4.4.0 added stamped messages; current releases are 4.5.1 on Jazzy, Kilted and Lyrical.

## Lean model and properties

The whole node reduces to one pure function over a small state, and every documented rule becomes a theorem.

### Model

- **Config.** A list of inputs and a list of locks, each with a priority and a timeout. The type carries a proof that input priorities are distinct, so the tie-breaking question cannot arise at run time.
- **State.** For each input, the time of its last message. For each lock, its last value and time.
- **Events.** A command on input *i* at time *t*, or a lock value on lock *j* at time *t*. Time is an integer count of nanoseconds, supplied by the shim.
- **Step.** `step : Config → State → Event → State × Bool`. The boolean says whether to forward the message just received. The message body never enters Lean, so no floating point is involved.

### Properties

| Property | Statement |
| --- | --- |
| Provenance | Every output is a message that arrived on some configured input, unchanged. This holds by construction, since the core only says yes or no. |
| Priority | A message on input *i* is forwarded only if no input of higher priority is live. |
| Lock | A message on input *i* is forwarded only if no active lock outranks it. |
| Dead lock publisher | A lock whose publisher has gone silent past its timeout counts as active. |
| Expiry | An input that has timed out never suppresses another input. |
| Liveness | If input *i* is the highest-priority live input and no active lock outranks it, its message is forwarded. |
| Trace invariant | The above hold after any sequence of events, by induction from the initial state. |

### A finding worth stating up front

Because the original decides only when a message arrives, it publishes nothing when every input goes silent. It does not send a stop command. Stopping then depends on the base driver's own watchdog. The Lean version should either match this and state the assumption, or add a timer and a proved "silence leads to zero" rule as a deliberate difference.

## Build plan

As a standalone project this is about four days for two people; most of that is the shim and build set-up, which every later Lean node reuses.

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
- **Dynamic configuration.** Inputs are defined in a parameter file, so subscriptions are created at start-up. The Lean config must be built and validated then, including rejecting duplicate priorities.
- **Stamped and plain messages.** Both exist across releases. Since the core never reads the message body, this affects only the shim.

Open decisions:

- [ ] Match the original's silence behaviour, or add a timer and a zero command?
- [ ] Reject equal priorities at start-up, or reproduce whatever the original does?

## Sources

- [twist_mux package wiki (mirror)](https://mirror.umd.edu/roswiki/twist_mux.html)
- [twist_mux on ROS Index](https://index.ros.org/p/twist_mux/)
- [Main feasibility doc](https://claude.ai/artifact/Qo4d1tB8db3fYY91fmNoeX)
