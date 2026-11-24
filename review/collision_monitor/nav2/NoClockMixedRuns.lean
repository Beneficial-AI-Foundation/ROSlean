import ROSlean.CollisionMonitor.NoClock
import ROSlean.CollisionMonitor.Trace

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor

/-- Equal, or equal except that `a` reports `source` where `b` reports `normal`, with no zero on
record, in the active or finalized phase. -/
def Q (a b : State) : Prop :=
  a.phase = b.phase ∧ a.scan = b.scan ∧ a.shapes = b.shapes ∧ a.prevZero = b.prevZero ∧
    a.stopStamp = b.stopStamp ∧
    (a.prevReport = b.prevReport ∨
      (a.prevReport = .source ∧ b.prevReport = .normal ∧ a.prevZero = false ∧
        (a.phase = .active ∨ a.phase = .finalized)))

theorem Q_refl (a : State) : Q a a := ⟨rfl, rfl, rfl, rfl, rfl, Or.inl rfl⟩

theorem Q_step (cfg : Config) (a b : State) (e : Event) (h : Q a b) :
    Q (step cfg a e).1 (step cfg b e).1 ∧
      (step cfg a e).2.velocity = (step cfg b e).2.velocity ∧
      (step cfg a e).2.processed = (step cfg b e).2.processed := by
  rcases a with ⟨p, s, sh, z, t, r⟩
  rcases b with ⟨p', s', sh', z', t', r'⟩
  simp only [Q] at h
  obtain ⟨rfl, rfl, rfl, rfl, rfl, h⟩ := h
  rcases h with rfl | ⟨rfl, rfl, rfl, hp⟩
  · exact ⟨Q_refl _, rfl, rfl⟩
  · cases e with
    | cmd now v pts zv =>
      have he : evaluate cfg ⟨p, s, sh, false, t, .source⟩ now pts zv =
          evaluate cfg ⟨p, s, sh, false, t, .normal⟩ now pts zv :=
        evaluate_congr cfg _ _ now pts zv rfl rfl
      have hs : shapesAfter cfg ⟨p, s, sh, false, t, .source⟩ now pts zv =
          shapesAfter cfg ⟨p, s, sh, false, t, .normal⟩ now pts zv := by
        simp [shapesAfter, sourcePoints]
      simp only [step]
      split
      · exact ⟨⟨rfl, rfl, rfl, rfl, rfl, Or.inr ⟨rfl, rfl, rfl, hp⟩⟩, rfl, rfl⟩
      · split
        · exact ⟨⟨rfl, rfl, rfl, rfl, rfl, Or.inr ⟨rfl, rfl, rfl, hp⟩⟩, rfl, rfl⟩
        · rw [he, hs]
          simp [Q, Output.velocity, Output.processed]
    | shape k verts foreign =>
      simp only [step, acceptShape]
      repeat' split
      all_goals simp_all [Q, Output.velocity, Output.processed]
    | _ =>
      rcases hp with rfl | rfl <;> simp [step, Q, Output.velocity, Output.processed]

inductive MEv where
  | ev (e : Event)
  | nc (v : Twist)

def mstep (cfg : Config) (st : State) : MEv → State
  | .ev e => (step cfg st e).1
  | .nc v => (stepNoClock st v).1

/-- Every processed no-clock command replaced by `deactivate, activate`; every other one dropped. -/
def subst (cfg : Config) : State → List MEv → List Event
  | _, [] => []
  | a, .ev e :: l => e :: subst cfg (step cfg a e).1 l
  | a, .nc v :: l =>
    (if a.phase = .active ∧ v.finite = true then [.deactivate, .activate] else []) ++
      subst cfg (stepNoClock a v).1 l

theorem Q_mrun (cfg : Config) (l : List MEv) (a b : State) (h : Q a b) :
    Q (l.foldl (mstep cfg) a) (run cfg b (subst cfg a l)).1 := by
  induction l generalizing a b with
  | nil => exact h
  | cons m l ih =>
    cases m with
    | ev e =>
      simp only [List.foldl_cons, subst, run_cons, mstep]
      exact ih _ _ (Q_step cfg a b e h).1
    | nc v =>
      simp only [List.foldl_cons, subst, mstep, run_append]
      refine ih _ _ ?_
      by_cases hp : a.phase = .active
      · cases hf : v.finite with
        | false =>
          rw [noClock_of_not_finite a v hf]
          simpa [run_nil] using h
        | true =>
          rw [noClock_state a v hp hf]
          have hb : b.phase = .active := h.1 ▸ hp
          simp only [hp, true_and, if_true, run_cons, run_nil, step_deactivate, hb,
            step_activate]
          exact ⟨rfl, h.2.1, h.2.2.1, rfl, h.2.2.2.2.1, Or.inr ⟨rfl, rfl, rfl, Or.inl rfl⟩⟩
      · rw [noClock_of_not_active a v hp]
        simpa [hp, run_nil] using h

/-- From start-up: the state after a mixed sequence equals, up to `Q`, the state `run` reaches on
the substituted events. -/
theorem mixed_from_init (cfg : Config) (l : List MEv) :
    Q (l.foldl (mstep cfg) (State.init cfg))
      (run cfg (State.init cfg) (subst cfg (State.init cfg) l)).1 :=
  Q_mrun cfg l _ _ (Q_refl _)

#print axioms Q_step
#print axioms mixed_from_init
