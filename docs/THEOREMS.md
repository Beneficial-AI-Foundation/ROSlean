# The main theorems

What is proved about the Lean-backed `twist_mux`, in plain words, with a link
to each statement. All theorems are in `ROSlean/Mux/`, are checked by
`pixi run prove`, and use no `sorry` and no axioms beyond Lean's three
standard ones. Rule numbers (R…) refer to [SPEC.md](SPEC.md).

Links point to line numbers as of 2026-11-23; if a link lands a few lines off,
search the file for the theorem name.

## What the theorems are about

The model is in [`core/ROSleanCore/Mux.lean`](../core/ROSleanCore/Mux.lean).
The definitions the theorems mention:

| Name | Meaning |
| --- | --- |
| [`Config`](../core/ROSleanCore/Mux.lean#L30) | The inputs and the locks, each with a priority and a timeout. |
| [`State`](../core/ROSleanCore/Mux.lean#L48) | Time of the last message on each input and lock, and each lock's last value. |
| [`Event`](../core/ROSleanCore/Mux.lean#L120) | A command arrived on input *i* at time *t*, or a value arrived on lock *j* at time *t*. |
| [`step`](../core/ROSleanCore/Mux.lean#L130) | Processes one event. Returns the new state and whether to forward the message. This is what the node calls. |
| [`run`](../core/ROSleanCore/Mux.lean#L145) | `step` over a list of events. |
| [`inputLive`](../core/ROSleanCore/Mux.lean#L91) | The input has no timeout, or its last message is at most its timeout old. |
| [`lockActive`](../core/ROSleanCore/Mux.lean#L80) | The lock's last value is `true`, or it has a timeout and its publisher has been silent for longer. |

"Forwarded" below means `step` returned `true` for a message on input *i* at
time *t*.

## The central theorem

**[`forward_iff`](../ROSlean/Mux/Properties.lean#L122)** says exactly when a
message is forwarded. A message on input *i* at time *t* is forwarded if and
only if all three hold:

1. *i* is a configured input with priority above 0;
2. every active lock has priority at most *i*'s;
3. every other live input has lower priority than *i*, or the same priority
   and a later position in the input list.

Locks and liveness are those of the state just before the message. Everything
in the next section follows from this one statement.

## The property table

One step, from an arbitrary state. Nothing is assumed about the clock.

| Property | Theorem | Statement |
| --- | --- | --- |
| Provenance | [`forward_configured`](../ROSlean/Mux/Properties.lean#L154) | Only a message on a configured input with positive priority is forwarded. The message body never enters Lean, so it cannot be changed there. |
| Priority | [`forward_priority`](../ROSlean/Mux/Properties.lean#L162) | If a message on *i* is forwarded, no live input has higher priority than *i*. |
| Tie-break | [`forward_first_of_equal`](../ROSlean/Mux/Properties.lean#L176) | Among live inputs of equal priority, only the first in list order is forwarded (R10). |
| Lock | [`forward_lock`](../ROSlean/Mux/Properties.lean#L190) | If a message on *i* is forwarded, no active lock has higher priority than *i*. |
| Dead lock publisher | [`lockActive_of_silent`](../ROSlean/Mux/Properties.lean#L208), [`not_forward_of_silent_lock`](../ROSlean/Mux/Properties.lean#L216) | A lock whose publisher is silent past its timeout is active, whatever its last value; inputs below it are not forwarded. |
| Liveness | [`forward_of_highest_live`](../ROSlean/Mux/Properties.lean#L230) | If *i* has positive priority, no active lock outranks it, and every other live input ranks below it, its message *is* forwarded. |
| Expiry | [`forward_of_others_expired`](../ROSlean/Mux/Properties.lean#L243) | An input that has timed out never suppresses another. |
| Timeout boundary | [`inputLive_iff_le`](../ROSlean/Mux/Properties.lean#L262) | An input with a timeout is live up to and including `stamp + timeout`, and not after (R4). |
| E-stop | [`not_forward_of_top_lock`](../ROSlean/Mux/Properties.lean#L272) | If a lock with priority strictly above every input has last value `true`, nothing is forwarded. |
| Lock messages | [`step_lock_snd`](../ROSlean/Mux/Properties.lean#L88) | A lock message is never forwarded. |

The first six of these are the two directions of `forward_iff` read
separately: "forwarded only if" (safety) and "if …, then forwarded"
(liveness).

## Over sequences of events

The node runs for a long time, so the single-step statements are lifted to
whole runs in [`Trace.lean`](../ROSlean/Mux/Trace.lean).

| Theorem | Statement |
| --- | --- |
| [`run_forward_iff`](../ROSlean/Mux/Trace.lean#L56) | **Trace invariant.** In any run from any state, the *n*-th event is forwarded iff it is an input message satisfying the three conditions of `forward_iff` in the state reached after the first *n* events. |
| [`run_forward_iff_history`](../ROSlean/Mux/Trace.lean#L224) | **Decision from history alone.** After any events since start-up, the decision is characterised without mentioning internal state: only the time of each input's and lock's last message and each lock's last value matter. |
| [`run_not_forward_of_lock_true`](../ROSlean/Mux/Trace.lean#L264) | If the last message on a lock above *i* said `true`, *i* is not forwarded, however long ago that was. |
| [`run_not_forward_of_silent_lock`](../ROSlean/Mux/Trace.lean#L252) | If a lock above *i* with a timeout last spoke (or never spoke) more than its timeout ago, *i* is not forwarded. |
| [`run_not_forward_of_higher_live`](../ROSlean/Mux/Trace.lean#L283) | If a higher-priority input spoke within its timeout, or never expires, *i* is not forwarded. |
| [`run_not_forward_of_earlier_equal_live`](../ROSlean/Mux/Trace.lean#L302) | The same for an earlier input of equal priority. |

`run_forward_iff_history` is the one to read if you want a specification you
could hand to someone who has not seen the code.

## The code that actually runs

The node does not call `step` directly; it calls exported wrappers in
[`core/ROSleanCore/FFI.lean`](../core/ROSleanCore/FFI.lean). These theorems,
in [`ROSlean/Mux/FFI.lean`](../ROSlean/Mux/FFI.lean), tie the wrappers to the
model.

| Theorem | Statement |
| --- | --- |
| [`muxForward_after`](../ROSlean/Mux/FFI.lean#L107) | **End to end.** Build a mux from the handles the node added, feed it any sequence of events, deliver a message: the decision the node reads is the model's decision for exactly those handles and events. |
| [`ofRaw_inputs`](../ROSlean/Mux/FFI.lean#L30), [`builderAddInput_inputs`](../ROSlean/Mux/FFI.lean#L62) | The configuration is the handles in the order they were added, with nothing dropped or reordered. The order decides ties. |
| [`ofRaw_priority_le`](../ROSlean/Mux/FFI.lean#L39) | Every priority in a built configuration is at most 255 (R1). |
| [`handleOfRaw_priority_of_nonpos`](../ROSlean/Mux/FFI.lean#L49), [`handleOfRaw_timeout_of_nonpos`](../ROSlean/Mux/FFI.lean#L53) | A zero or negative priority becomes 0 (never selected); a zero or negative timeout disables expiry (R2). |

## Supporting results

Used by the proofs above; worth knowing if you extend the model.

| Theorem | Statement |
| --- | --- |
| [`select_eq_some_iff`](../ROSlean/Mux/Basic.lean#L214) | Characterises upstream's selection loop: input *i* is selected iff it is live, has positive priority, is not below the lock priority, and beats every other live input (with the tie-break). `forward_iff` is this plus the effect of recording the new message. |
| [`lockPriority_le_iff`](../ROSlean/Mux/Basic.lean#L118) | The lock priority is at most *p* iff every active lock has priority at most *p*. |
| [`lockActive_iff`](../ROSlean/Mux/Properties.lean#L32), [`inputLive_iff`](../ROSlean/Mux/Properties.lean#L42) | The two definitions written out as plain conditions on stamps and timeouts. |
| [`getD_setPad`](../ROSlean/Mux/Basic.lean#L20) | State updates behave as writes to a total map, so no update is ever lost, even from a malformed state. |

## What these theorems do not say

- **That the model equals upstream `twist_mux`.** That is tested, not proved:
  both nodes are run on the same messages and their decisions compared. The
  tests exercise ordinary arbitration. Timeout arithmetic, clock reads, and clamping very large priorities can change decisions (SPEC D2, D4, D6).
- **Anything about the C++ shim:** mapping topics to indices, reading the
  clock, converting seconds to nanoseconds, and publishing on `true`.
- **That the mux stops the robot when all inputs go silent.** It publishes
  nothing in that case, as upstream (R13).
- **Anything about concurrent callbacks.** The model handles one event at a
  time, and the node uses a single-threaded executor.
- **That time moves forward.** No theorem assumes it. If the clock jumps
  back, expired inputs can become live again. A lock active only because of
  expiry can become inactive. A lock last set to true stays active.

Two consequences that are easy to miss: a lock blocks only *strictly* lower
priorities, and priorities are capped at 255, so an input at 255 can never be
blocked. `not_forward_of_top_lock` requires the lock to be strictly above
every input for that reason.
