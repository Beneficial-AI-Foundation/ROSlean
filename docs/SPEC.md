# twist_mux arbitration: specification

Taken from the upstream C++ source, `ros-teleop/twist_mux` tag `4.5.1`
(commit `2a2790a`): `include/twist_mux/topic_handle.hpp` and
`src/twist_mux.cpp`. Rule numbers are referenced from the Lean model and
theorems.

## Rules as implemented upstream

**Configuration**

- R1. An input ("topic") and a lock each have a name, a topic, a timeout in
  seconds (floating point) and an integer priority. The name is the full
  parameter prefix, e.g. `topics.navigation`. Priorities are clamped to
  0..255 with a warning; negative priorities become 0. The timeout is
  converted once to integer nanoseconds by truncation.
- R2. A timeout ≤ 0 disables expiry for that handle. Because of the
  truncation in R1, so does a positive timeout below 1 ns.

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
  So a lock whose publisher has gone silent past its timeout is locked. A
  lock with a positive timeout that has never received a message is locked
  once the clock exceeds the timeout; under simulation time, while the clock
  is still below the timeout, it is *not* locked. This confirms the "dead
  lock publisher" rule.
- R7. The *lock priority* is the maximum priority over locked locks, or 0 if
  none is locked.

**Selection**

- R8. An input is *masked* iff it has expired or its priority is strictly
  below the lock priority. A lock therefore does **not** block inputs of
  equal priority. (The upstream example config relies on this.)
- R9. The *selected* input is the unmasked input of maximum priority, where
  the maximum must be strictly greater than 0. An input with priority 0 is
  never selected. If no input qualifies, nothing is selected.
- R10. On equal priorities the first unmasked input in the handle list is
  selected: the loop replaces its candidate only on a strictly greater
  priority. A later input of the same priority is forwarded only while every
  earlier one is expired. The list order comes from `list_parameters`; it is
  not a documented contract, but in `rclcpp` it is the byte order of the full
  parameter names (`topics.<name>.…`), i.e. ASCII order of the handle names (upper case before lower case), not
  the order in the YAML file. In the upstream example config `joystick`
  beats `tablet` while both are live.
- R10a. An input at priority 255 can never be masked by a lock (R8 is
  strict).

**Decision**

- R11. When a message arrives on input *i* at time `now`: set
  `stamp_i := now`, then forward the message unchanged iff *i* is the
  selected input. Upstream re-reads the clock for every expiry check inside
  the callback, so the checks happen a few hundred nanoseconds after the
  stamp and at slightly different times per handle (see D4).
- R12. When a message arrives on lock *j* at time `now`: set
  `stamp_j := now` and store its value. Nothing is published.
- R13. There is no other trigger. A 1 Hz timer exists but only publishes
  diagnostics. When all inputs are silent, nothing is published.

**Interface**

- R14. Parameters: `topics.<name>.{topic,timeout,priority}`,
  `locks.<name>.{topic,timeout,priority}`, `use_stamped` (default `true`).
  A missing field is a fatal error, and so is a field of the wrong YAML
  type: `timeout: 0` (integer) or `priority: 10.0` terminates the node.
  Parameters are read once at start-up. Empty or absent `topics` or `locks` is legal. With no inputs there is no velocity output. With no locks the node still forwards eligible inputs.
- R15. Output topic `cmd_vel_out`, QoS keep-last 1. Subscriptions use system
  default QoS. Inputs and output are all `TwistStamped` if `use_stamped`,
  else all `Twist`; the two cannot be mixed.
- R16. Single-threaded executor: events are processed one at a time.

## Answers to the open questions in the scope doc

| Question | Answer |
| --- | --- |
| Equal input priorities | First live one in list order wins; the order is the ASCII order of the handle names in practice (R10). ROSlean reproduces this. |
| Does a lock block inputs of equal priority? | No, only strictly lower (R8). |
| Dead lock publisher | Counts as locked (R6), including before the first message once the clock has passed the timeout. |
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
| D1 | — | — | Withdrawn. ROSlean used to reject equal input priorities at start-up; it now reproduces R10, and the node obtains the handle order from the same `list_parameters` call as upstream. The number is kept so that D2–D9 stay stable. |
| D2 | `now − stamp` computed in floating-point seconds | Integer nanoseconds throughout; timeouts truncated to ns as upstream | No floating point in the core. Can differ from upstream only when `now − stamp` is within about a microsecond of the timeout; see "Extent of D2" below. |
| D3 | Diagnostics publisher, `twist_marker`, `joystick_relay` | Not provided | Out of scope. |
| D4 | Clock re-read for every expiry check within one callback | One clock reading per event, used for the stamp and all checks | Makes a decision a function of (state, event). An expiry check can cross the timeout boundary during the callback. Upstream can expire even the input that just arrived if its timeout is short enough. |
| D5 | NaN, infinite or very large timeouts are accepted; beyond ±2⁶³ ns the conversion is undefined behaviour | Non-finite timeouts and timeouts of magnitude 9.2e9 s or more are rejected at start-up | Avoids undefined behaviour. (Slightly conservative: 9.2e9 s to 9.22e9 s is still well defined upstream.) |
| D6 | Priority read as 32-bit `int` before clamping | Clamped from the 64-bit parameter value | Differs only for priorities beyond ±2³¹. |
| D7 | A start-up error escapes `main` and aborts the process (SIGABRT) | Logged as FATAL, exit code 1 | Clean failure. Visible to launch files that inspect exit codes. |
| D8 | — | Negative node time would be treated as 0 | Not observable: `rclcpp` aborts both nodes on a negative `/clock` value before either sees it. The clamp in the shim is defensive only. |
| D9 | Ships a launch file | None; run the executable with `--ros-args --params-file` | Out of scope. |

Everything else, including R5, R9, R10, R13 and the fatal errors in R14, is
reproduced as is.

### Extent of D2

Measured against upstream by the M6 review, and reproduced bit for bit by
emulating upstream's arithmetic (`double(ns) / 1e9`):

- The two nodes agree whenever `now − stamp` is more than 400 ns from the
  timeout at today's Unix time, and always under simulated time except
  *exactly* at the boundary.
- Exactly at `now − stamp = timeout` with a decimal timeout such as 0.1, 0.3
  or 0.7 s, upstream's rounding makes its strict `>` behave as `≥` for a
  fraction of (stamp, timeout) pairs, so it expires the handle one tick
  early; ROSlean never does. Under simulated time with a ticking clock,
  exact-boundary hits are an ordinary case, not a rare one.
- At Unix-scale clock values both directions occur: upstream may expire a
  handle up to a few hundred nanoseconds early or late.
- Timeouts that are binary fractions (0.25, 0.5, …) show no difference under
  simulated time.

ROSlean's behaviour is the documented rule (R4) applied exactly.

## Model assumptions

- A1. Events are delivered one at a time with the node-clock reading taken at
  delivery (R16).
- A2. Nothing is assumed about time being monotonic for the safety
  properties. If the clock moves back to or before `stamp + timeout`,
  an expired input can become live again, and a lock active only because
  of expiry can become inactive. A lock last set to true stays active.
- A3. Stopping the robot when all inputs are silent is the base driver's job
  (R13).

## Note on upstream tests

Upstream's own tests do not pin the arbitration rules: only
`test_joystick_relay.py` is registered with CMake, and the priority test in
`system.test.py` is commented out. The comparison therefore supplies its own:
fixed scenarios with exact boundaries (`test/scenario.py`) and randomised
configurations and event sequences (`test/random_diff.py`), both run against
upstream and ROSlean on the same messages.
