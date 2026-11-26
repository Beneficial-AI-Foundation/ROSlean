# Collision monitor: a demanded stop is not always published

Not a finding and not a bug. `nav2_collision_monitor` does this on purpose and documents it. It is recorded here because it is the reason one of our theorems is weaker than one would expect.

## What happens

When the node's output is zero, it publishes that zero for `stop_pub_timeout` (default 1 s) and then publishes nothing for as long as the output stays zero ([`collision_monitor_node.cpp`, `publishVelocity`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_collision_monitor/src/collision_monitor_node.cpp#L213)).

- The clock starts when the output changes from non-zero to zero. Further zeros do not restart it.
- The rule looks at the value, not at its cause. A stop for an obstacle, a stop for a stale scan and a zero command from the planner that merely passes through are one period of zero with one clock.
- Any non-zero output ends the silence.
- The node has no timer. It only answers commands.

Our node and model do the same (SPEC.md, R15).

## Why upstream does it

From the Nav2 documentation of `stop_pub_timeout`: zero stops being published so that systems outside Nav2 can bring the robot out of a state close to a collision, and so that a standing robot can go into sleep mode.

## The sequence that weakens the theorem

| Time | Planner commands | Zone | Node publishes |
| --- | --- | --- | --- |
| 0 to 1 s | 0 (idle) | free | 0 |
| 1 to 5 s | 0 (idle) | free | nothing, the timeout has run out |
| 5 s onward | 0.5 m/s | obstacle in stop zone | nothing |

A stop is demanded from 5 s on and not one zero is published for it: the idle zeros used up the second, and the output never left zero, so the clock was not restarted.

The robot does not move. The 0.5 m/s commands are withheld, and the last thing the base received was a zero.

The sequence is an example in `ROSlean/CollisionMonitor/Examples.lean`, checked by `decide`. Upstream behaves the same in our scenarios `stop-publication` and `zero-input`.

## What the theorems say because of it

- **Not provable, because it is false:** "whenever a stop is demanded, zero is published."
- **Proved (`stop`, `run_stop`):** when a stop is demanded, the node publishes zero or nothing. It never publishes a non-zero velocity.
- **Proved (`run_withheld_last_published_zero`):** in any run from start-up, when a velocity is withheld, the last velocity this node published was a zero.

The second and third together keep the robot stopped only under an assumption that is not about the node (A3 in THEOREMS.md): the base holds its last command, and nothing else commands it in the meantime.

## How much it matters

Little on an ordinary robot. Most bases stop by themselves when commands cease.

It matters when:

- another node publishes to the base during the silence (teleop, a recovery behaviour, docking). The monitor does not oppose it, although an obstacle is in the stop zone. Upstream intends exactly this, so it is a gap only where it was not meant to be allowed;
- the base needs fresh zeros to stay stopped. Unusual;
- something reads "no zeros" as "no stop". The state topic does report the stop.

## What would make the theorem strong

Publish zero for every command that meets a demanded stop (obstacle or invalid source), without a time limit. A zero command that merely passes through could still go silent after the timeout, so a standing robot can still go quiet.

- Gain: "a demanded stop is published" becomes a theorem, and A3 is no longer needed for stops.
- Cost: a deliberate difference from upstream. An outside system can no longer take over the velocity topic while the obstacle is in the stop zone and the planner is still commanding. The comparison tests against upstream must leave these cases out.

Decided by the user on 2026-11-24 (`tasks/PLAN.md`, "Decisions"): as upstream. A parameter with upstream's behaviour as the default would keep the node a drop-in replacement and offer the stronger guarantee on request.

It does not make the node publish without an incoming command. A zero when commands cease altogether is a separate decision (a timer).
