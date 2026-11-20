import ROSlean.Mux.Basic

/-!
# Properties of the twist_mux model

The property table of `twist_mux.md`, as theorems about `ROSleanCore.Mux.step`
and `ROSleanCore.Mux.run`. Rule numbers refer to `docs/SPEC.md`.

The central result is `forward_iff`: the decision for a message on input `i`
is a function of the state *before* the message and of the arrival time, and
it is `true` exactly when

* `i` is a configured input with positive priority,
* no locked lock has a priority above `i`'s, and
* every other live input has strictly lower priority.

Everything else is a corollary or lifts this to event sequences.

All theorems hold for an arbitrary state and arbitrary times: nothing is
assumed about the clock being monotonic (A2).
-/

namespace ROSlean.Mux

open ROSleanCore.Mux

/-! ## Reading the definitions -/

/-- R6: a lock is locked iff it is configured and has expired or its last
value is `true`. -/
theorem lockActive_iff (cfg : Config) (st : State) (now j : Nat) :
    lockActive cfg st now j = true ↔
    ∃ hj, cfg.locks[j]? = some hj ∧
      ((0 < hj.timeout ∧ st.lockStamp j + hj.timeout < now) ∨ st.lockValue j = true) := by
  unfold lockActive
  cases h : cfg.locks[j]? with
  | none => simp
  | some hj => simp [expired_iff]

/-- R4: an input is live iff it is configured and has not expired. -/
theorem inputLive_iff (cfg : Config) (st : State) (now k : Nat) :
    inputLive cfg st now k = true ↔
    ∃ hk, cfg.inputs[k]? = some hk ∧ ¬ (0 < hk.timeout ∧ st.inputStamp k + hk.timeout < now) := by
  unfold inputLive
  cases h : cfg.inputs[k]? with
  | none => simp
  | some hk =>
    simp only [expired, Option.some.injEq, exists_eq_left']
    simp only [Bool.not_eq_eq_eq_not, Bool.not_true, Bool.and_eq_false_iff, decide_eq_false_iff_not,
      not_and]
    omega

/-- R2: an input whose timeout is 0 is always live. -/
theorem inputLive_of_timeout_zero (cfg : Config) (st : State) (now k : Nat) (hk : Handle)
    (hget : cfg.inputs[k]? = some hk) (h0 : hk.timeout = 0) : inputLive cfg st now k = true :=
  (inputLive_iff ..).mpr ⟨hk, hget, by omega⟩

/-! ## One step -/

/-- The state after a message on input `i` at time `t`. -/
def recvInput (st : State) (i t : Nat) : State :=
  { st with inputStamps := setPad st.inputStamps i t 0 }

/-- The state after a message with value `v` on lock `j` at time `t`. -/
def recvLock (st : State) (j : Nat) (v : Bool) (t : Nat) : State :=
  { st with
      lockStamps := setPad st.lockStamps j t 0
      lockValues := setPad st.lockValues j v false }

theorem step_input (cfg : Config) (st : State) (i t : Nat) (hi : i < cfg.inputs.length) :
    step cfg st (.input i t) = (recvInput st i t, select cfg (recvInput st i t) t == some i) := by
  simp [step, hi, recvInput]

theorem step_input_of_not_lt (cfg : Config) (st : State) (i t : Nat)
    (hi : ¬ i < cfg.inputs.length) : step cfg st (.input i t) = (st, false) := by
  simp [step, hi]

theorem step_lock (cfg : Config) (st : State) (j : Nat) (v : Bool) (t : Nat)
    (hj : j < cfg.locks.length) : step cfg st (.lock j v t) = (recvLock st j v t, false) := by
  simp [step, hj, recvLock]

theorem step_lock_of_not_lt (cfg : Config) (st : State) (j : Nat) (v : Bool) (t : Nat)
    (hj : ¬ j < cfg.locks.length) : step cfg st (.lock j v t) = (st, false) := by
  simp [step, hj]

/-- R12: a lock message is never forwarded. -/
theorem step_lock_snd (cfg : Config) (st : State) (j : Nat) (v : Bool) (t : Nat) :
    (step cfg st (.lock j v t)).2 = false := by
  by_cases hj : j < cfg.locks.length
  · rw [step_lock _ _ _ _ _ hj]
  · rw [step_lock_of_not_lt _ _ _ _ _ hj]

theorem inputStamp_recvInput (st : State) (i t k : Nat) :
    (recvInput st i t).inputStamp k = if k = i then t else st.inputStamp k :=
  getD_setPad ..

theorem lockStamp_recvLock (st : State) (j : Nat) (v : Bool) (t k : Nat) :
    (recvLock st j v t).lockStamp k = if k = j then t else st.lockStamp k :=
  getD_setPad ..

theorem lockValue_recvLock (st : State) (j : Nat) (v : Bool) (t k : Nat) :
    (recvLock st j v t).lockValue k = if k = j then v else st.lockValue k :=
  getD_setPad ..

private theorem lockPriority_recvInput (cfg : Config) (st : State) (i t now : Nat) :
    lockPriority cfg (recvInput st i t) now = lockPriority cfg st now := rfl

private theorem inputLive_recvInput_self (cfg : Config) (st : State) (i t : Nat) (hi : Handle)
    (hget : cfg.inputs[i]? = some hi) : inputLive cfg (recvInput st i t) t i = true := by
  simp [inputLive, hget, inputStamp_recvInput, not_expired_self]

private theorem inputLive_recvInput_of_ne (cfg : Config) (st : State) (i t now k : Nat)
    (hne : k ≠ i) : inputLive cfg (recvInput st i t) now k = inputLive cfg st now k := by
  simp [inputLive, inputStamp_recvInput, hne]

/-- **Decision.** A message arriving on input `i` at time `t` is forwarded iff
`i` is a configured input with positive priority, no locked lock outranks it,
and every other live input has strictly lower priority. Locks and liveness are
those of the state before the message (R4–R11). -/
theorem forward_iff (cfg : Config) (st : State) (i t : Nat) :
    (step cfg st (.input i t)).2 = true ↔
    ∃ hi, cfg.inputs[i]? = some hi ∧ 0 < hi.priority ∧
      (∀ j hj, cfg.locks[j]? = some hj → lockActive cfg st t j = true →
        hj.priority ≤ hi.priority) ∧
      (∀ k hk, cfg.inputs[k]? = some hk → k ≠ i → inputLive cfg st t k = true →
        hk.priority < hi.priority) := by
  by_cases hlen : i < cfg.inputs.length
  · rw [step_input _ _ _ _ hlen]
    simp only [beq_iff_eq]
    rw [select_eq_some_iff, lockPriority_recvInput]
    constructor
    · rintro ⟨hi, hget, hpos, -, hlp, hothers⟩
      refine ⟨hi, hget, hpos, (lockPriority_le_iff ..).mp hlp, ?_⟩
      intro k hk hgk hne hlive
      exact hothers k hk hgk hne (by rwa [inputLive_recvInput_of_ne _ _ _ _ _ _ hne])
    · rintro ⟨hi, hget, hpos, hlocks, hothers⟩
      refine ⟨hi, hget, hpos, inputLive_recvInput_self _ _ _ _ hi hget,
        (lockPriority_le_iff ..).mpr hlocks, ?_⟩
      intro k hk hgk hne hlive
      exact hothers k hk hgk hne (by rwa [inputLive_recvInput_of_ne _ _ _ _ _ _ hne] at hlive)
  · rw [step_input_of_not_lt _ _ _ _ hlen]
    constructor
    · intro h; cases h
    · rintro ⟨hi, hget, -⟩
      exact absurd (List.getElem?_eq_some_iff.mp hget).1 hlen

/-! ## The property table -/

/-- **Provenance.** Only a message on a configured input with positive
priority is ever forwarded (R9). The message itself never enters the model, so
it is forwarded unchanged by construction. -/
theorem forward_configured (cfg : Config) (st : State) (i t : Nat)
    (hfwd : (step cfg st (.input i t)).2 = true) :
    ∃ hi, cfg.inputs[i]? = some hi ∧ 0 < hi.priority := by
  obtain ⟨hi, hget, hpos, -⟩ := (forward_iff ..).mp hfwd
  exact ⟨hi, hget, hpos⟩

/-- **Priority.** A message on input `i` is forwarded only if no other live
input has priority greater than or equal to `i`'s (R8, R9). -/
theorem forward_priority (cfg : Config) (st : State) (i t : Nat)
    (hfwd : (step cfg st (.input i t)).2 = true)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (k : Nat) (hk : Handle) (hgk : cfg.inputs[k]? = some hk) (hne : k ≠ i)
    (hlive : inputLive cfg st t k = true) :
    hk.priority < hi.priority := by
  obtain ⟨hi', hgi', -, -, hothers⟩ := (forward_iff ..).mp hfwd
  rw [hgi] at hgi'; cases hgi'
  exact hothers k hk hgk hne hlive

/-- **Lock.** A message on input `i` is forwarded only if no locked lock has
priority greater than `i`'s (R6–R8). -/
theorem forward_lock (cfg : Config) (st : State) (i t : Nat)
    (hfwd : (step cfg st (.input i t)).2 = true)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (j : Nat) (hj : Handle) (hgj : cfg.locks[j]? = some hj)
    (hact : lockActive cfg st t j = true) :
    hj.priority ≤ hi.priority := by
  obtain ⟨hi', hgi', -, hlocks, -⟩ := (forward_iff ..).mp hfwd
  rw [hgi] at hgi'; cases hgi'
  exact hlocks j hj hgj hact

/-- A lock whose last value is `true` is locked, however old that value is. -/
theorem lockActive_of_value (cfg : Config) (st : State) (now j : Nat) (hj : Handle)
    (hgj : cfg.locks[j]? = some hj) (hval : st.lockValue j = true) :
    lockActive cfg st now j = true :=
  (lockActive_iff ..).mpr ⟨hj, hgj, Or.inr hval⟩

/-- **Dead lock publisher.** A lock with a timeout whose publisher has been
silent for longer than the timeout is locked, whatever its last value (R6). -/
theorem lockActive_of_silent (cfg : Config) (st : State) (now j : Nat) (hj : Handle)
    (hgj : cfg.locks[j]? = some hj) (hto : 0 < hj.timeout)
    (hsilent : st.lockStamp j + hj.timeout < now) :
    lockActive cfg st now j = true :=
  (lockActive_iff ..).mpr ⟨hj, hgj, Or.inl ⟨hto, hsilent⟩⟩

/-- **Dead lock publisher, consequence.** While a lock's publisher is silent
past its timeout, no input of lower priority is forwarded. -/
theorem not_forward_of_silent_lock (cfg : Config) (st : State) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi)
    (j : Nat) (hj : Handle) (hgj : cfg.locks[j]? = some hj) (hto : 0 < hj.timeout)
    (hsilent : st.lockStamp j + hj.timeout < t) (hlow : hi.priority < hj.priority) :
    (step cfg st (.input i t)).2 = false := by
  by_contra hfwd
  have hfwd' : (step cfg st (.input i t)).2 = true := by simpa using hfwd
  have := forward_lock cfg st i t hfwd' hi hgi j hj hgj
    (lockActive_of_silent cfg st t j hj hgj hto hsilent)
  omega

/-- **Liveness.** If input `i` has positive priority, no locked lock outranks
it, and every other live input has lower priority, then its message is
forwarded (R9, R11). -/
theorem forward_of_highest_live (cfg : Config) (st : State) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi) (hpos : 0 < hi.priority)
    (hlocks : ∀ j hj, cfg.locks[j]? = some hj → lockActive cfg st t j = true →
      hj.priority ≤ hi.priority)
    (hothers : ∀ k hk, cfg.inputs[k]? = some hk → k ≠ i → inputLive cfg st t k = true →
      hk.priority < hi.priority) :
    (step cfg st (.input i t)).2 = true :=
  (forward_iff ..).mpr ⟨hi, hgi, hpos, hlocks, hothers⟩

/-- **Expiry.** An input that has timed out never suppresses another input:
if every other input either has lower priority or has expired, and no locked
lock outranks `i`, then `i`'s message is forwarded (R4, R8). -/
theorem forward_of_others_expired (cfg : Config) (st : State) (i t : Nat)
    (hi : Handle) (hgi : cfg.inputs[i]? = some hi) (hpos : 0 < hi.priority)
    (hlocks : ∀ j hj, cfg.locks[j]? = some hj → lockActive cfg st t j = true →
      hj.priority ≤ hi.priority)
    (hothers : ∀ k hk, cfg.inputs[k]? = some hk → k ≠ i →
      hk.priority < hi.priority ∨ (0 < hk.timeout ∧ st.inputStamp k + hk.timeout < t)) :
    (step cfg st (.input i t)).2 = true := by
  apply forward_of_highest_live cfg st i t hi hgi hpos hlocks
  intro k hk hgk hne hlive
  rcases hothers k hk hgk hne with h | h
  · exact h
  · obtain ⟨hk', hgk', hnot⟩ := (inputLive_iff ..).mp hlive
    rw [hgk] at hgk'; cases hgk'
    exact absurd h hnot

/-- An input stays live for exactly `timeout` after its last message: it
suppresses lower inputs at `stamp + timeout` and no longer at
`stamp + timeout + 1` (R4, strict comparison). -/
theorem inputLive_boundary (cfg : Config) (st : State) (k : Nat) (hk : Handle)
    (hgk : cfg.inputs[k]? = some hk) (hto : 0 < hk.timeout) :
    inputLive cfg st (st.inputStamp k + hk.timeout) k = true ∧
    inputLive cfg st (st.inputStamp k + hk.timeout + 1) k = false := by
  constructor
  · exact (inputLive_iff ..).mpr ⟨hk, hgk, by omega⟩
  · rw [Bool.eq_false_iff]
    intro h
    obtain ⟨hk', hgk', hnot⟩ := (inputLive_iff ..).mp h
    rw [hgk] at hgk'; cases hgk'
    exact hnot ⟨hto, by omega⟩

end ROSlean.Mux
