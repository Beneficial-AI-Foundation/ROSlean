import ROSlean.Mux.Properties

/-!
# twist_mux over event sequences

Lifts the single-step theorems of `ROSlean.Mux.Properties` to `run`: the
decision for the `n`-th event of any sequence is the single-step decision in
the state reached after the first `n` events, so every property of
`forward_iff` holds at every point of every run (the *trace invariant*).

It also ties the state to the event history: after any run, the stored stamps
and lock values are those of the most recent message on each handle. Together
these give statements purely about the sequence of events, with no reference
to internal state.
-/

namespace ROSlean.Mux

open ROSleanCore.Mux

theorem run_nil (cfg : Config) (st : State) : run cfg st [] = (st, []) := rfl

theorem run_cons (cfg : Config) (st : State) (e : Event) (es : List Event) :
    run cfg st (e :: es) =
      ((run cfg (step cfg st e).1 es).1,
        (step cfg st e).2 :: (run cfg (step cfg st e).1 es).2) := by
  cases h : step cfg st e with
  | mk st' fwd =>
    cases h2 : run cfg st' es with
    | mk st'' fwds => simp [run, h, h2]

/-- One decision per event. -/
theorem run_length (cfg : Config) (st : State) (es : List Event) :
    (run cfg st es).2.length = es.length := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih => simp [run_cons, ih]

/-- The `n`-th decision of a run is the single-step decision for the `n`-th
event in the state reached after the first `n` events. -/
theorem run_getElem? (cfg : Config) (st : State) (es : List Event) (n : Nat) :
    (run cfg st es).2[n]? =
      es[n]?.map (fun e => (step cfg (run cfg st (es.take n)).1 e).2) := by
  induction es generalizing st n with
  | nil => simp [run_nil]
  | cons e es ih =>
    cases n with
    | zero => simp [run_cons, run_nil]
    | succ n => simp [run_cons, ih]

/-- **Trace invariant.** In any run from any state, the `n`-th event is
forwarded iff it is a message on some input `i` at some time `t` that satisfies
the conditions of `forward_iff` in the state reached after the first `n`
events. In particular a lock message is never forwarded, and every forwarded
message respects priority, locks and expiry at the moment it is forwarded. -/
theorem run_forward_iff (cfg : Config) (st : State) (es : List Event) (n : Nat) :
    (run cfg st es).2[n]? = some true ↔
    ∃ i t, es[n]? = some (.input i t) ∧
      ∃ hi, cfg.inputs[i]? = some hi ∧ 0 < hi.priority ∧
        (∀ j hj, cfg.locks[j]? = some hj →
          lockActive cfg (run cfg st (es.take n)).1 t j = true → hj.priority ≤ hi.priority) ∧
        (∀ k hk, cfg.inputs[k]? = some hk → k ≠ i →
          inputLive cfg (run cfg st (es.take n)).1 t k = true → hk.priority < hi.priority) := by
  rw [run_getElem?]
  cases he : es[n]? with
  | none => simp
  | some e =>
    cases e with
    | input i t =>
      simp only [Option.map_some, Option.some.injEq, Event.input.injEq]
      rw [forward_iff]
      constructor
      · intro h; exact ⟨i, t, ⟨rfl, rfl⟩, h⟩
      · rintro ⟨i', t', ⟨rfl, rfl⟩, h⟩; exact h
    | lock j v t =>
      simp [step_lock_snd]

/-! ## State as a function of history -/

/-- Time of the last message on input `k` in `es`, or `d` if there is none. -/
def lastInputStamp (k : Nat) : List Event → Nat → Nat
  | [], d => d
  | .input i t :: es, d => lastInputStamp k es (if i = k then t else d)
  | .lock _ _ _ :: es, d => lastInputStamp k es d

/-- Time of the last message on lock `k` in `es`, or `d` if there is none. -/
def lastLockStamp (k : Nat) : List Event → Nat → Nat
  | [], d => d
  | .input _ _ :: es, d => lastLockStamp k es d
  | .lock j _ t :: es, d => lastLockStamp k es (if j = k then t else d)

/-- Value of the last message on lock `k` in `es`, or `d` if there is none. -/
def lastLockValue (k : Nat) : List Event → Bool → Bool
  | [], d => d
  | .input _ _ :: es, d => lastLockValue k es d
  | .lock j v _ :: es, d => lastLockValue k es (if j = k then v else d)

private theorem step_inputStamp (cfg : Config) (st : State) (e : Event) (k : Nat)
    (hk : k < cfg.inputs.length) :
    (step cfg st e).1.inputStamp k =
      match e with
      | .input i t => if i = k then t else st.inputStamp k
      | .lock _ _ _ => st.inputStamp k := by
  cases e with
  | input i t =>
    by_cases hi : i < cfg.inputs.length
    · rw [step_input _ _ _ _ hi]
      simp only [inputStamp_recvInput]
      by_cases h : k = i
      · simp [h]
      · simp [h, Ne.symm h]
    · rw [step_input_of_not_lt _ _ _ _ hi]
      have : i ≠ k := by omega
      simp [this]
  | lock j v t =>
    by_cases hj : j < cfg.locks.length
    · rw [step_lock _ _ _ _ _ hj]; rfl
    · rw [step_lock_of_not_lt _ _ _ _ _ hj]

private theorem step_lockStamp (cfg : Config) (st : State) (e : Event) (k : Nat)
    (hk : k < cfg.locks.length) :
    (step cfg st e).1.lockStamp k =
      match e with
      | .input _ _ => st.lockStamp k
      | .lock j _ t => if j = k then t else st.lockStamp k := by
  cases e with
  | input i t =>
    by_cases hi : i < cfg.inputs.length
    · rw [step_input _ _ _ _ hi]; rfl
    · rw [step_input_of_not_lt _ _ _ _ hi]
  | lock j v t =>
    by_cases hj : j < cfg.locks.length
    · rw [step_lock _ _ _ _ _ hj]
      simp only [lockStamp_recvLock]
      by_cases h : k = j
      · simp [h]
      · simp [h, Ne.symm h]
    · rw [step_lock_of_not_lt _ _ _ _ _ hj]
      have : j ≠ k := by omega
      simp [this]

private theorem step_lockValue (cfg : Config) (st : State) (e : Event) (k : Nat)
    (hk : k < cfg.locks.length) :
    (step cfg st e).1.lockValue k =
      match e with
      | .input _ _ => st.lockValue k
      | .lock j v _ => if j = k then v else st.lockValue k := by
  cases e with
  | input i t =>
    by_cases hi : i < cfg.inputs.length
    · rw [step_input _ _ _ _ hi]; rfl
    · rw [step_input_of_not_lt _ _ _ _ hi]
  | lock j v t =>
    by_cases hj : j < cfg.locks.length
    · rw [step_lock _ _ _ _ _ hj]
      simp only [lockValue_recvLock]
      by_cases h : k = j
      · simp [h]
      · simp [h, Ne.symm h]
    · rw [step_lock_of_not_lt _ _ _ _ _ hj]
      have : j ≠ k := by omega
      simp [this]

/-- After any run, the stamp of a configured input is the time of the last
message received on it (R3, R11). -/
theorem run_inputStamp (cfg : Config) (st : State) (es : List Event) (k : Nat)
    (hk : k < cfg.inputs.length) :
    (run cfg st es).1.inputStamp k = lastInputStamp k es (st.inputStamp k) := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih =>
    rw [run_cons, ih, step_inputStamp _ _ _ _ hk]
    cases e <;> rfl

/-- After any run, the stamp of a configured lock is the time of the last
message received on it (R3, R12). -/
theorem run_lockStamp (cfg : Config) (st : State) (es : List Event) (k : Nat)
    (hk : k < cfg.locks.length) :
    (run cfg st es).1.lockStamp k = lastLockStamp k es (st.lockStamp k) := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih =>
    rw [run_cons, ih, step_lockStamp _ _ _ _ hk]
    cases e <;> rfl

/-- After any run, the value of a configured lock is that of the last message
received on it (R3, R12). -/
theorem run_lockValue (cfg : Config) (st : State) (es : List Event) (k : Nat)
    (hk : k < cfg.locks.length) :
    (run cfg st es).1.lockValue k = lastLockValue k es (st.lockValue k) := by
  induction es generalizing st with
  | nil => rfl
  | cons e es ih =>
    rw [run_cons, ih, step_lockValue _ _ _ _ hk]
    cases e <;> rfl

/-- In the initial state every stamp is 0 and every lock value is `false` (R3). -/
theorem init_reads (cfg : Config) (k : Nat) :
    (State.init cfg).inputStamp k = 0 ∧ (State.init cfg).lockStamp k = 0 ∧
    (State.init cfg).lockValue k = false := by
  refine ⟨?_, ?_, ?_⟩ <;>
    simp only [State.init, State.inputStamp, State.lockStamp, State.lockValue,
      List.getD_eq_getElem?_getD, List.getElem?_replicate] <;>
    split <;> rfl

/-! ## Statements about event sequences only -/

/-- **Decision as a function of history.** After any sequence of events `es`
from start-up, a message on input `i` at time `t` is forwarded iff

* `i` is a configured input with positive priority;
* every lock that is *locked* has priority at most `i`'s, where a lock is
  locked if its last message carried `true`, or it has a timeout and its last
  message (or start-up, time 0, if there was none) is more than the timeout
  before `t`;
* every other input that is *live* has priority strictly below `i`'s, where
  an input is live if it never expires or its last message (or time 0) is at
  most its timeout before `t`.

Nothing here refers to internal state. "Before `t`" is arithmetic on the
recorded clock readings; no monotonicity of the clock is assumed (A2). -/
theorem run_forward_iff_history (cfg : Config) (es : List Event) (i t : Nat) :
    (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = true ↔
    ∃ hi, cfg.inputs[i]? = some hi ∧ 0 < hi.priority ∧
      (∀ j hj, cfg.locks[j]? = some hj →
        ((0 < hj.timeout ∧ lastLockStamp j es 0 + hj.timeout < t) ∨
          lastLockValue j es false = true) →
        hj.priority ≤ hi.priority) ∧
      (∀ k hk, cfg.inputs[k]? = some hk → k ≠ i →
        (hk.timeout = 0 ∨ t ≤ lastInputStamp k es 0 + hk.timeout) →
        hk.priority < hi.priority) := by
  rw [forward_iff]
  refine exists_congr fun hi => and_congr_right fun _ => and_congr_right fun _ => and_congr ?_ ?_
  · refine forall_congr' fun j => forall_congr' fun hj => forall_congr' fun hg => ?_
    have hl := (List.getElem?_eq_some_iff.mp hg).1
    rw [lockActive_iff, run_lockStamp _ _ _ _ hl, run_lockValue _ _ _ _ hl,
      (init_reads cfg j).2.1, (init_reads cfg j).2.2]
    simp [hg]
  · refine forall_congr' fun k => forall_congr' fun hk => forall_congr' fun hg =>
      forall_congr' fun _ => ?_
    have hl := (List.getElem?_eq_some_iff.mp hg).1
    rw [inputLive_iff, run_inputStamp _ _ _ _ hl, (init_reads cfg k).1]
    simp only [hg, Option.some.injEq, exists_eq_left']
    constructor <;> intro h h' <;> apply h <;> omega

/-- **Dead lock publisher, over a run from start-up.** Take any sequence of
events `es` followed by a message on input `i` at time `t`. If some lock `j`
with a timeout and a priority above `i`'s received its last message (or none
at all) more than its timeout before `t`, the message is not forwarded. -/
theorem run_not_forward_of_silent_lock (cfg : Config) (es : List Event) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (j : Nat) (hj : Handle) (hgj : cfg.locks[j]? = some hj) (hto : 0 < hj.timeout)
    (hsilent : lastLockStamp j es 0 + hj.timeout < t) (hlow : hi.priority < hj.priority) :
    (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = false := by
  apply not_forward_of_silent_lock cfg _ i t hi hgi j hj hgj hto _ hlow
  rw [run_lockStamp _ _ _ _ (List.getElem?_eq_some_iff.mp hgj).1, (init_reads cfg j).2.1]
  exact hsilent

/-- **Lock, over a run from start-up.** If the last message on a lock `j`
with priority above `i`'s carried `true`, a message on input `i` is not
forwarded, however long ago that was. -/
theorem run_not_forward_of_lock_true (cfg : Config) (es : List Event) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (j : Nat) (hj : Handle) (hgj : cfg.locks[j]? = some hj)
    (hval : lastLockValue j es false = true) (hlow : hi.priority < hj.priority) :
    (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = false := by
  by_contra hfwd
  have hfwd' : (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = true := by
    simpa using hfwd
  have hact : lockActive cfg (run cfg (State.init cfg) es).1 t j = true := by
    apply lockActive_of_value _ _ _ _ hj hgj
    rw [run_lockValue _ _ _ _ (List.getElem?_eq_some_iff.mp hgj).1, (init_reads cfg j).2.2]
    exact hval
  have := forward_lock cfg _ i t hfwd' hi hgi j hj hgj hact
  omega

/-- **Priority, over a run from start-up.** If some other input `k` of higher
priority never expires, or its last message (or start-up, time 0, if it has
never spoken) is at most its timeout before `t`, a message on input `i` at
time `t` is not forwarded. -/
theorem run_not_forward_of_higher_live (cfg : Config) (es : List Event) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (k : Nat) (hk : Handle) (hgk : cfg.inputs[k]? = some hk) (hne : k ≠ i)
    (hfresh : hk.timeout = 0 ∨ t ≤ lastInputStamp k es 0 + hk.timeout)
    (hhigh : hi.priority < hk.priority) :
    (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = false := by
  by_contra hfwd
  have hfwd' : (step cfg (run cfg (State.init cfg) es).1 (.input i t)).2 = true := by
    simpa using hfwd
  have hlive : inputLive cfg (run cfg (State.init cfg) es).1 t k = true := by
    refine (inputLive_iff ..).mpr ⟨hk, hgk, ?_⟩
    rw [run_inputStamp _ _ _ _ (List.getElem?_eq_some_iff.mp hgk).1, (init_reads cfg k).1]
    omega
  have := forward_priority cfg _ i t hfwd' hi hgi k hk hgk hne hlive
  omega

end ROSlean.Mux
