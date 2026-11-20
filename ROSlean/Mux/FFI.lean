import ROSlean.Mux.Trace
import ROSleanCore.FFI

/-!
# The C interface computes the model

The ROS node does not call `step` directly: it calls the exported wrappers in
`ROSleanCore.FFI`. These theorems tie the wrappers to the model, so that the
properties proved about `step` and `run` are properties of what the node
executes:

* the configuration the node runs with is exactly the handles it added, in
  order, with priorities clamped (`ofRaw_inputs`, `builderAddInput_inputs`,
  `muxNew_eq`); the order is what decides ties between equal priorities;
* the decision returned for an input message is `step`'s, and the state after
  any sequence of calls is `run`'s (`muxForward_onInput`, `foldl_apply`).

What remains outside Lean is the C++ shim: mapping subscriptions to indices,
reading the clock, and acting on the returned decision.
-/

namespace ROSlean.Mux

open ROSleanCore.Mux ROSleanCore.FFI

/-! ## Configuration -/

/-- Building a config neither reorders nor drops handles: it is the given
lists with priorities clamped. -/
theorem ofRaw_inputs (ins locks : List Handle) :
    (Config.ofRaw ins locks).inputs = ins.map Handle.clamp ∧
    (Config.ofRaw ins locks).locks = locks.map Handle.clamp := ⟨rfl, rfl⟩

/-- Clamping leaves priorities up to 255 unchanged. -/
theorem clamp_of_le (h : Handle) (hp : h.priority ≤ maxPriority) : h.clamp = h := by
  cases h; simp_all [Handle.clamp]

/-- Every priority in a built config is at most 255 (R1). -/
theorem ofRaw_priority_le (ins locks : List Handle) :
    (∀ hi ∈ (Config.ofRaw ins locks).inputs, hi.priority ≤ maxPriority) ∧
    (∀ hj ∈ (Config.ofRaw ins locks).locks, hj.priority ≤ maxPriority) := by
  constructor <;>
  · intro x hx
    obtain ⟨y, -, rfl⟩ := List.mem_map.mp hx
    exact Nat.min_le_right ..

/-- A raw priority that is zero or negative becomes 0, so the input is never
selected (R1, R9). -/
theorem handleOfRaw_priority_of_nonpos (p t : Int64) (h : p.toInt ≤ 0) :
    (handleOfRaw p t).priority = 0 := Int.toNat_of_nonpos h

/-- A raw timeout that is zero or negative becomes 0: expiry is disabled (R2). -/
theorem handleOfRaw_timeout_of_nonpos (p t : Int64) (h : t.toInt ≤ 0) :
    (handleOfRaw p t).timeout = 0 := Int.toNat_of_nonpos h

/-- Non-negative raw values are passed through unchanged. -/
theorem handleOfRaw_of_nonneg (p t : Int64) (hp : 0 ≤ p.toInt) (ht : 0 ≤ t.toInt) :
    ((handleOfRaw p t).priority : Int) = p.toInt ∧ ((handleOfRaw p t).timeout : Int) = t.toInt :=
  ⟨Int.toNat_of_nonneg hp, Int.toNat_of_nonneg ht⟩

/-- Inputs are indexed in the order they are added. -/
theorem builderAddInput_inputs (b : Builder) (p t : Int64) :
    (builderAddInput b p t).inputs = b.inputs ++ [handleOfRaw p t] ∧
    (builderAddInput b p t).locks = b.locks := ⟨rfl, rfl⟩

/-- Locks are indexed in the order they are added. -/
theorem builderAddLock_locks (b : Builder) (p t : Int64) :
    (builderAddLock b p t).locks = b.locks ++ [handleOfRaw p t] ∧
    (builderAddLock b p t).inputs = b.inputs := ⟨rfl, rfl⟩

/-- A mux starts in the initial state of the config built from its handles. -/
theorem muxNew_eq (b : Builder) :
    muxNew b = { cfg := Config.ofRaw b.inputs b.locks,
                 st := State.init (Config.ofRaw b.inputs b.locks), forward := false } := rfl

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

/-- **End to end.** Create a mux from a builder, feed it any sequence of
events, then deliver a message on input `i` at time `t`. The decision the node
reads is the one characterised by `run_forward_iff_history` for the config
built from the builder's handles. -/
theorem muxForward_after (b : Builder) (es : List Event) (i : UInt32) (t : UInt64) :
    muxForward (muxOnInput (es.foldl Mux.apply (muxNew b)) i t) =
      (step (Config.ofRaw b.inputs b.locks)
        (run (Config.ofRaw b.inputs b.locks) (State.init (Config.ofRaw b.inputs b.locks)) es).1
        (.input i.toNat t.toNat)).2 := by
  rw [muxForward_onInput, (foldl_apply ..).1, (foldl_apply ..).2, muxNew_eq]

end ROSlean.Mux
