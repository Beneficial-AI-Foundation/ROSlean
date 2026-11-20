import ROSlean.Mux.Trace
import ROSleanCore.FFI

/-!
# The C interface computes the model

The ROS node does not call `step` directly: it calls the exported wrappers in
`ROSleanCore.FFI`. These theorems tie the wrappers to the model, so that the
properties proved about `step` and `run` are properties of what the node
executes:

* the configuration the node runs with is exactly the handles it added, in
  order, with priorities clamped (`validate_ok_iff`, `builderAddInput_inputs`);
* a mux is only ever created from a configuration that passed validation
  (`builderCheck_eq_zero_iff`, `muxNew_of_ok`);
* the decision returned for an input message is `step`'s, and the state after
  any sequence of calls is `run`'s (`muxForward_onInput`, `foldl_apply`).

What remains outside Lean is the C++ shim: mapping subscriptions to indices,
reading the clock, and acting on the returned decision.
-/

namespace ROSlean.Mux

open ROSleanCore.Mux ROSleanCore.FFI

/-! ## Configuration -/

/-- Validation neither reorders nor drops handles: the resulting config is the
given lists with priorities clamped. -/
theorem validate_ok_iff (ins locks : List Handle) (c : Config) :
    Config.validate ins locks = .ok c ↔
      c.inputs = ins.map Handle.clamp ∧ c.locks = locks.map Handle.clamp := by
  simp only [Config.validate]
  constructor
  · intro h
    split at h
    · cases h; exact ⟨rfl, rfl⟩
    · cases h
  · rintro ⟨h1, h2⟩
    have hn := c.inputs_distinct
    rw [h1] at hn
    rw [dif_pos hn]
    cases c; simp_all

/-- Clamping leaves priorities up to 255 unchanged. -/
theorem clamp_of_le (h : Handle) (hp : h.priority ≤ maxPriority) : h.clamp = h := by
  cases h; simp_all [Handle.clamp]

/-- Every priority in a validated config is at most 255 (R1). -/
theorem validate_priority_le (ins locks : List Handle) (c : Config)
    (h : Config.validate ins locks = .ok c) :
    (∀ hi ∈ c.inputs, hi.priority ≤ maxPriority) ∧ (∀ hj ∈ c.locks, hj.priority ≤ maxPriority) := by
  obtain ⟨h1, h2⟩ := (validate_ok_iff ..).mp h
  rw [h1, h2]
  constructor <;>
  · intro x hx
    obtain ⟨y, -, rfl⟩ := List.mem_map.mp hx
    exact Nat.min_le_right ..

/-- Validation fails exactly when two inputs have the same clamped priority (D1). -/
theorem validate_isOk_iff (ins locks : List Handle) :
    (∃ c, Config.validate ins locks = .ok c) ↔ (ins.map (·.clamp.priority)).Nodup := by
  simp only [Config.validate, List.map_map]
  constructor
  · rintro ⟨c, h⟩
    split at h
    · assumption
    · cases h
  · intro h
    exact ⟨_, dif_pos h⟩

/-- Inputs are indexed in the order they are added. -/
theorem builderAddInput_inputs (b : Builder) (p t : Int64) :
    (builderAddInput b p t).inputs = b.inputs ++ [handleOfRaw p t] ∧
    (builderAddInput b p t).locks = b.locks := ⟨rfl, rfl⟩

/-- Locks are indexed in the order they are added. -/
theorem builderAddLock_locks (b : Builder) (p t : Int64) :
    (builderAddLock b p t).locks = b.locks ++ [handleOfRaw p t] ∧
    (builderAddLock b p t).inputs = b.inputs := ⟨rfl, rfl⟩

/-- The check the shim performs before creating a mux succeeds exactly when
validation does. -/
theorem builderCheck_eq_zero_iff (b : Builder) :
    builderCheck b = 0 ↔ ∃ c, Config.validate b.inputs b.locks = .ok c := by
  unfold builderCheck
  cases h : Config.validate b.inputs b.locks with
  | ok c => simp [errorCode]
  | error e => cases e; simp [errorCode]

/-- A mux created from a builder that passed the check starts in the initial
state of the validated config. -/
theorem muxNew_of_ok (b : Builder) (c : Config) (h : Config.validate b.inputs b.locks = .ok c) :
    muxNew b = { cfg := c, st := State.init c, forward := false } := by
  simp [muxNew, h]

/-! ## Events -/

theorem apply_cfg (m : Mux) (e : Event) : (m.apply e).cfg = m.cfg := rfl

theorem apply_st (m : Mux) (e : Event) : (m.apply e).st = (step m.cfg m.st e).1 := rfl

/-- The decision the node reads after an input message is `step`'s decision. -/
theorem muxForward_onInput (m : Mux) (i : UInt32) (t : UInt64) :
    muxForward (muxOnInput m i t) = (step m.cfg m.st (.input i.toNat t.toNat)).2 := rfl

theorem muxOnInput_eq (m : Mux) (i : UInt32) (t : UInt64) :
    muxOnInput m i t = m.apply (.input i.toNat t.toNat) := rfl

theorem muxOnLock_eq (m : Mux) (j : UInt32) (v : Bool) (t : UInt64) :
    muxOnLock m j v t = m.apply (.lock j.toNat v t.toNat) := rfl

/-- After any sequence of calls the config is unchanged and the state is the
one `run` reaches. -/
theorem foldl_apply (m : Mux) (es : List Event) :
    (es.foldl Mux.apply m).cfg = m.cfg ∧ (es.foldl Mux.apply m).st = (run m.cfg m.st es).1 := by
  induction es generalizing m with
  | nil => exact ⟨rfl, rfl⟩
  | cons e es ih =>
    obtain ⟨h1, h2⟩ := ih (m.apply e)
    simp only [List.foldl_cons, h1, h2, run_cons]
    exact ⟨rfl, rfl⟩

/-- **End to end.** Create a mux from a builder that passed the check, feed it
any sequence of events, then deliver a message on input `i` at time `t`. The
decision the node reads is the one characterised by
`run_forward_iff_history` for the validated config. -/
theorem muxForward_after (b : Builder) (c : Config)
    (h : Config.validate b.inputs b.locks = .ok c) (es : List Event) (i : UInt32) (t : UInt64) :
    muxForward (muxOnInput (es.foldl Mux.apply (muxNew b)) i t) =
      (step c (run c (State.init c) es).1 (.input i.toNat t.toNat)).2 := by
  rw [muxForward_onInput, (foldl_apply ..).1, (foldl_apply ..).2, muxNew_of_ok b c h]

end ROSlean.Mux
