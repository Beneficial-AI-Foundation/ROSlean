/-!
# twist_mux arbitration model

Executable model of the arbitration logic of the ROS 2 `twist_mux` node, as
specified in `docs/SPEC.md` (rule numbers `R…` and differences `D…` refer to
that file). The message body never enters this model: the node asks "a message
just arrived on input `i` at time `t`; forward it?".

This file must not import Mathlib. Theorems about it live in `ROSlean.Mux`.
-/

namespace ROSleanCore.Mux

/-! Times are node-clock readings in nanoseconds (D2) and priorities are
natural numbers, higher wins. Both are plain `Nat`. -/

/-- Largest priority; larger values are clamped to it (R1). -/
def maxPriority : Nat := 255

/-- Static configuration of one input or one lock (R1). -/
structure Handle where
  priority : Nat
  /-- Timeout in nanoseconds; `0` disables expiry (R2). -/
  timeout : Nat
  deriving Repr, DecidableEq

/-- Node configuration. Input priorities are pairwise distinct (D1), so the
selected input is determined by the config and never by list order. -/
structure Config where
  inputs : List Handle
  locks : List Handle
  inputs_distinct : (inputs.map (·.priority)).Nodup

/-- Why a proposed configuration was rejected. -/
inductive ConfigError where
  | duplicateInputPriority
  deriving Repr, DecidableEq

/-- R1: clamps the priority to `maxPriority`. (Negative priorities cannot be
represented; the shim maps them to 0.) -/
def Handle.clamp (h : Handle) : Handle := { h with priority := min h.priority maxPriority }

/-- Builds a configuration from raw handles: clamps priorities (R1), then
rejects duplicate input priorities (D1). -/
def Config.validate (inputs locks : List Handle) : Except ConfigError Config :=
  let inputs := inputs.map Handle.clamp
  let locks := locks.map Handle.clamp
  if h : (inputs.map (·.priority)).Nodup then
    .ok { inputs, locks, inputs_distinct := h }
  else
    .error .duplicateInputPriority

/-- The configuration with no inputs and no locks. -/
def Config.empty : Config := { inputs := [], locks := [], inputs_distinct := .nil }

/-- Dynamic state (R3). Handles are addressed by their index in the config.
The lists need not have the config's length: an index with no entry reads as
the initial value, and writes extend the list (`setPad`), so no update is ever
lost. -/
structure State where
  /-- Arrival time of the last message on each input. -/
  inputStamps : List Nat
  /-- Arrival time of the last message on each lock. -/
  lockStamps : List Nat
  /-- Last value received on each lock. -/
  lockValues : List Bool
  deriving Repr, DecidableEq

/-- Initial state: all stamps 0, all lock values `false` (R3). -/
def State.init (cfg : Config) : State where
  inputStamps := List.replicate cfg.inputs.length 0
  lockStamps := List.replicate cfg.locks.length 0
  lockValues := List.replicate cfg.locks.length false

/-- Writes `v` at index `i`, first extending the list with `d` if it is too
short. Together with `List.getD · · d` this behaves as a total map. -/
def setPad {α : Type} (l : List α) (i : Nat) (v d : α) : List α :=
  if i < l.length then l.set i v else l ++ List.replicate (i - l.length) d ++ [v]

def State.inputStamp (st : State) (i : Nat) : Nat := st.inputStamps.getD i 0
def State.lockStamp (st : State) (j : Nat) : Nat := st.lockStamps.getD j 0
def State.lockValue (st : State) (j : Nat) : Bool := st.lockValues.getD j false

/-- R4: a handle has expired iff its timeout is enabled and strictly more than
`timeout` has elapsed since `stamp`. Upstream computes `now - stamp > timeout`
over the reals, which is `now > stamp + timeout`; in particular a clock that
jumped backwards never expires anything (A2). -/
def expired (timeout : Nat) (stamp now : Nat) : Bool :=
  0 < timeout && stamp + timeout < now

/-- R6: lock `j` is locked iff it has expired or its last value is `true`. -/
def lockActive (cfg : Config) (st : State) (now : Nat) (j : Nat) : Bool :=
  match cfg.locks[j]? with
  | some h => expired h.timeout (st.lockStamp j) now || st.lockValue j
  | none => false

/-- R7: the maximum priority over locked locks, or 0 if none is locked. -/
def lockPriority (cfg : Config) (st : State) (now : Nat) : Nat :=
  cfg.locks.zipIdx.foldl
    (fun acc (h, j) => if lockActive cfg st now j then max acc h.priority else acc) 0

/-- Input `k` has not expired. -/
def inputLive (cfg : Config) (st : State) (now : Nat) (k : Nat) : Bool :=
  match cfg.inputs[k]? with
  | some h => !expired h.timeout (st.inputStamp k) now
  | none => false

/-- R8, negated: input `k` is live and its priority is not below the lock
priority `lp`. -/
def unmasked (cfg : Config) (st : State) (now : Nat) (lp : Nat) (k : Nat) : Bool :=
  match cfg.inputs[k]? with
  | some h => inputLive cfg st now k && lp ≤ h.priority
  | none => false

/-- One iteration of upstream's selection loop: keep the best `(priority,
index)` seen so far, replacing it only on a strictly greater priority. -/
def selectStep (cfg : Config) (st : State) (now : Nat) (lp : Nat)
    (best : Nat × Option Nat) (hk : Handle × Nat) : Nat × Option Nat :=
  if unmasked cfg st now lp hk.2 && best.1 < hk.1.priority then (hk.1.priority, some hk.2)
  else best

/-- R9: the index of the selected input, if any. A literal transcription of
the loop in upstream `TwistMux::hasPriority`: start from priority 0 and no
selection, and scan the inputs in order. -/
def select (cfg : Config) (st : State) (now : Nat) : Option Nat :=
  let lp := lockPriority cfg st now
  (cfg.inputs.zipIdx.foldl (selectStep cfg st now lp) (0, none)).2

/-- What the node reacts to. -/
inductive Event where
  /-- A velocity command arrived on input `i` at time `t`. -/
  | input (i : Nat) (t : Nat)
  /-- A lock message with the given value arrived on lock `j` at time `t`. -/
  | lock (j : Nat) (value : Bool) (t : Nat)
  deriving Repr, DecidableEq

/-- R11, R12: process one event. The boolean says whether to forward the
message that was just received; it is always `false` for lock events. An
event for an index that is not in the config changes nothing. -/
def step (cfg : Config) (st : State) : Event → State × Bool
  | .input i t =>
    if i < cfg.inputs.length then
      let st' := { st with inputStamps := setPad st.inputStamps i t 0 }
      (st', select cfg st' t == some i)
    else (st, false)
  | .lock j value t =>
    if j < cfg.locks.length then
      ({ st with
          lockStamps := setPad st.lockStamps j t 0
          lockValues := setPad st.lockValues j value false }, false)
    else (st, false)

/-- Runs a sequence of events from a state, returning the final state and the
forwarding decision for each event. -/
def run (cfg : Config) (st : State) : List Event → State × List Bool
  | [] => (st, [])
  | e :: es =>
    let (st', fwd) := step cfg st e
    let (st'', fwds) := run cfg st' es
    (st'', fwd :: fwds)

end ROSleanCore.Mux
