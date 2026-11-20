# twist_mux arbitration: specification

Taken from the upstream C++ source, `ros-teleop/twist_mux` tag `4.5.1`
(commit `2a2790a`): `include/twist_mux/topic_handle.hpp` and
`src/twist_mux.cpp`. Rule numbers are referenced from the Lean model and
theorems.

## Rules as implemented upstream

**Configuration**

- R1. An input ("topic") and a lock each have a name, a topic, a timeout in
  seconds (floating point) and an integer priority. Priorities are clamped to
  0..255 with a warning.
- R2. A timeout ≤ 0 disables expiry for that handle.

**State**

- R3. Each handle stores the node-clock time of its last message (`stamp`),
  initially 0. A lock also stores its last boolean value, initially `false`.
  The time is the arrival time on the node's clock (`now()`), never the
  message header stamp. With `use_sim_time` it is simulation time.

**Expiry**

- R4. A handle *has expired* at time `now` iff its timeout is > 0 and
  `now − stamp > timeout` (strict).
- R5. Consequence of R3 + R4: a handle with a positive timeout that has never
  received a message is expired (once the clock has passed the timeout). A
  handle with timeout 0 is never expired, *even if it has never received a
  message*.

**Locks**

- R6. A lock *is locked* iff it has expired or its last value is `true`.
  So a lock whose publisher has gone silent past its timeout is locked, and a
  lock with a positive timeout is locked from start-up until its first
  `false` message. This confirms the "dead lock publisher" rule.
- R7. The *lock priority* is the maximum priority over locked locks, or 0 if
  none is locked.

**Selection**

- R8. An input is *masked* iff it has expired or its priority is strictly
  below the lock priority. A lock therefore does **not** block inputs of
  equal priority. (The upstream example config relies on this.)
- R9. The *selected* input is the unmasked input of maximum priority, where
  the maximum must be strictly greater than 0. An input with priority 0 is
  never selected. If no input qualifies, nothing is selected.
- R10. On equal priorities upstream keeps the first one in its handle list.
  That order comes from `list_parameters` and is not documented, so the
  winner is unspecified.

**Decision**

- R11. When a message arrives on input *i* at time `now`: set
  `stamp_i := now`, then forward the message unchanged iff *i* is the
  selected input at `now`. Because the stamp was just refreshed, *i* itself
  is never expired at that moment.
- R12. When a message arrives on lock *j* at time `now`: set
  `stamp_j := now` and store its value. Nothing is published.
- R13. There is no other trigger. A 1 Hz timer exists but only publishes
  diagnostics. When all inputs are silent, nothing is published.

**Interface**

- R14. Parameters: `topics.<name>.{topic,timeout,priority}`,
  `locks.<name>.{topic,timeout,priority}`, `use_stamped` (default `true`).
  A missing field is a fatal error.
- R15. Output topic `cmd_vel_out`, QoS keep-last 1. Subscriptions use system
  default QoS. Message type is `TwistStamped` if `use_stamped`, else `Twist`.
- R16. Single-threaded executor: events are processed one at a time.

## Answers to the open questions in the scope doc

| Question | Answer |
| --- | --- |
| Equal input priorities | First in an unspecified list order wins (R10). |
| Does a lock block inputs of equal priority? | No, only strictly lower (R8). |
| Dead lock publisher | Counts as locked (R6), including before the first message. |
| Clock | Node clock at arrival; header stamps are ignored (R3). |

## Things the documentation does not say

- A never-expiring input (timeout 0) is live from start-up and suppresses
  every lower-priority input for ever, without having sent a message (R5).
- A priority-0 input is never forwarded (R9).
- "Live" is evaluated only when some input message arrives; a higher-priority
  input that went silent stops suppressing others only once its timeout has
  elapsed (R4, R13).

## Deliberate differences in ROSlean

| # | Upstream | ROSlean | Why |
| --- | --- | --- | --- |
| D1 | Equal input priorities allowed, winner unspecified (R10) | Rejected at start-up | Makes selection a function of the config. Note: the upstream *example* `twist_mux_topics.yaml` has two inputs at priority 100 and is therefore rejected. Equal priorities between a lock and an input, or between locks, stay legal. |
| D2 | Times compared as floating-point seconds | Integer nanoseconds; timeouts rounded to the nearest ns | No floating point in the core. Can differ from upstream only when `now − stamp` is within about a microsecond of the timeout (a `double` holding a Unix time resolves ~0.24 µs). |
| D3 | Diagnostics publisher, `twist_marker`, `joystick_relay` | Not provided | Out of scope. |

Everything else, including R5, R9 and R13, is reproduced as is.

## Model assumptions

- A1. Events are delivered one at a time with the node-clock reading taken at
  delivery (R16).
- A2. Nothing is assumed about time being monotonic for the safety
  properties. If the clock jumps backwards, `now − stamp` is negative and the
  handle is not expired, as upstream.
- A3. Stopping the robot when all inputs are silent is the base driver's job
  (R13).
