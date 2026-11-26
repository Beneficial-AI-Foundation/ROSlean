import ROSlean.CollisionMonitor.FFI

/-!
# Collision monitor: a command without a valid clock reading (D15)

Theorems about `ROSleanCore.CollisionMonitor.stepNoClock`, the transition
the node takes for a velocity command that arrives while the last message on
`/clock` carried a negative time (SPEC.md, D15; COMPARISON.md, defect 1), and
about the C call that delivers it (`cmOnCmdNoClock`).

`stepNoClock` is a transition beside `step`, not a constructor of `Event`.
So the theorems of the other files, which are stated on `step` and on
sequences of `Event`s, say nothing about it by themselves. What connects the
two is here:

* what the transition answers (`noClock_output`, `noClock_velocity`,
  `noClock_processed_iff`): the stop for an invalid source, never withheld,
  whatever the configuration, the stored scan and the zones;
* what it changes in the state (`noClock_state`): `prevZero` and
  `prevReport`, nothing else, so every property of later events that is
  stated for an arbitrary state holds after it;
* that it keeps the invariant of `Lifecycle.lean` (`inv_noClock`), so that
  the theorems with the hypothesis `Inv` apply after it;
* that it changes no later decision (`evaluate_after_noClock`) and that the
  first command with a clock reading after it has its velocity published
  (`velocity_after_noClock`);
* **what holds from start-up once such a command has been processed**
  (review CM5, finding 4). The state after it is one that no sequence of
  `Event`s reaches from `State.init` (`noClock_unreachable`), so the
  theorems of `Trace.lean` and of `FFI.lean` that are stated on `run cfg
  (State.init cfg) es` do not apply to the node from then on, also not to
  the stretches without such commands. What does hold is
  `mixed_from_init`: a sequence of events and no-clock commands leaves the
  node in the state that `run` reaches on the sequence in which every
  processed no-clock command is replaced by `deactivate, activate` and
  every other one is dropped, except possibly for the report on record
  (`AgreesUpToReport`); and whatever comes next is answered with the same
  velocity in both (`mixed_next_velocity`).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-- A command without a clock reading is not processed outside the active
phase: nothing is published and nothing changes (N4). -/
theorem noClock_of_not_active (st : State) (v : Twist) (h : st.phase ≠ .active) :
    stepNoClock st v = (st, .silent) := by
  simp [stepNoClock, h]

/-- A command with a component that is not finite is ignored without a clock
reading as with one (R2). -/
theorem noClock_of_not_finite (st : State) (v : Twist) (h : v.finite = false) :
    stepNoClock st v = (st, .silent) := by
  simp [stepNoClock, h]

/-- **The answer without a clock reading (D15).** In the active phase a
finite command is answered with the stop for an invalid source; the report
is `source` unless that is what was reported last. Nothing else enters: not
the configuration, not the stored scan, not the zones. -/
theorem noClock_output (st : State) (v : Twist) (hp : st.phase = .active) (hf : v.finite = true) :
    (stepNoClock st v).2 =
      .answer (some .stopSource) (if Report.source = st.prevReport then none else some .source) := by
  simp [stepNoClock, hp, hf]

/-- **Stop without a clock.** The velocity is published and it is a stop: it
is never withheld (R15 does not apply; there is no time to measure). -/
theorem noClock_velocity (st : State) (v : Twist) (hp : st.phase = .active)
    (hf : v.finite = true) :
    (stepNoClock st v).2.velocity = some .stopSource := by
  rw [noClock_output st v hp hf]; rfl

/-- Whatever velocity is published without a clock reading is a stop. -/
theorem noClock_velocity_isStop (st : State) (v : Twist) (a : Action)
    (h : (stepNoClock st v).2.velocity = some a) : a = .stopSource := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [noClock_of_not_finite st v hf] at h; cases h
    · rw [noClock_velocity st v hp hf] at h; exact (Option.some.inj h).symm
  · rw [noClock_of_not_active st v hp] at h; cases h

/-- **Processed (iff).** A command without a clock reading has an output
other than `silent` iff the node is active and the command is finite: the
same two conditions as with a clock reading (`processed_iff`). -/
theorem noClock_processed_iff (st : State) (v : Twist) :
    (stepNoClock st v).2 ≠ .silent ↔ st.phase = .active ∧ v.finite = true := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [noClock_of_not_finite st v hf]; simp
    · rw [noClock_output st v hp hf]; simp [hp]
  · rw [noClock_of_not_active st v hp]; simp [hp]

/-- **What changes.** The state after a processed command without a clock
reading: the record of published zeros is reset and the report is `source`.
The phase, the stored scan, the shapes and the stop stamp are as before. -/
theorem noClock_state (st : State) (v : Twist) (hp : st.phase = .active) (hf : v.finite = true) :
    (stepNoClock st v).1 = { st with prevZero := false, prevReport := .source } := by
  simp [stepNoClock, hp, hf]

/-- The phase, the stored scan, the shapes and the stop stamp are never
changed by a command without a clock reading, processed or not. -/
theorem noClock_keeps (st : State) (v : Twist) :
    (stepNoClock st v).1.phase = st.phase ∧ (stepNoClock st v).1.scan = st.scan ∧
      (stepNoClock st v).1.shapes = st.shapes ∧ (stepNoClock st v).1.stopStamp = st.stopStamp := by
  unfold stepNoClock
  split
  · exact ⟨rfl, rfl, rfl, rfl⟩
  · split <;> exact ⟨rfl, rfl, rfl, rfl⟩

/-- **The invariants are kept.** So every theorem with the hypothesis `Inv`
applies to states reached through commands without a clock reading too. -/
theorem inv_noClock (cfg : Config) (st : State) (v : Twist) (h : Inv cfg st) :
    Inv cfg (stepNoClock st v).1 := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [noClock_of_not_finite st v hf]; exact h
    · rw [noClock_state st v hp hf]
      exact ⟨fun hq => by simp [hp] at hq, fun hq => by simp [hp] at hq, h.len, h.topic, h.ok⟩
  · rw [noClock_of_not_active st v hp]; exact h

/-- **No later decision changes.** The action demanded for a command with a
clock reading is the same after a command without one as before it. -/
theorem evaluate_after_noClock (cfg : Config) (st : State) (v : Twist) (now : Nat)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords))) :
    evaluate cfg (stepNoClock st v).1 now scanPts zoneVerts =
      evaluate cfg st now scanPts zoneVerts :=
  evaluate_congr cfg _ st now scanPts zoneVerts (noClock_keeps st v).2.1 (noClock_keeps st v).2.2.1

/-- **When the clock is back.** The first command with a clock reading after
a processed command without one has its velocity published, whatever it is:
a zero is not withheld, because the period of R15 starts anew. -/
theorem velocity_after_noClock (cfg : Config) (st : State) (v v' : Twist) (now : Nat)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (hp : st.phase = .active) (hf : v.finite = true) (hf' : v'.finite = true) :
    (step cfg (stepNoClock st v).1 (.cmd now v' scanPts zoneVerts)).2.velocity =
      some (evaluate cfg st now scanPts zoneVerts) := by
  rw [velocity_cmd cfg _ now v' scanPts zoneVerts (by rw [(noClock_keeps st v).1]; exact hp) hf',
    evaluate_after_noClock]
  simp [noClock_state st v hp hf]

/-! ## Runs that contain commands without a clock reading

Review CM5, finding 4. -/

/-- What every state reached from start-up by `Event`s satisfies and a
processed no-clock command breaks: a report `source` on record comes with a
zero on record. -/
theorem source_report_has_zero (cfg : Config) (es : List Event) :
    (run cfg (State.init cfg) es).1.prevReport = .source →
      (run cfg (State.init cfg) es).1.prevZero = true := by
  suffices h : ∀ st : State, (st.prevReport = .source → st.prevZero = true) →
      ((run cfg st es).1.prevReport = .source → (run cfg st es).1.prevZero = true) from
    h _ (by simp [State.init])
  induction es with
  | nil => intro st h; exact h
  | cons e es ih =>
    intro st h
    rw [run_cons]
    apply ih
    cases e with
    | cmd now v p z =>
      simp only [step]
      split
      · exact h
      · split
        · exact h
        · simp only
          cases evaluate cfg st now p z <;> simp [Action.report, Action.yieldsZero]
          split <;> simp
    | shape k verts foreign =>
      simp only [step]
      split <;> exact h
    | deactivate => simp only [step]; split <;> simp_all
    | _ => simp only [step] <;> (try split) <;> exact h

/-- **The state after a processed no-clock command is reached by no run
from start-up**, for any configuration and any events: it has the report
`source` on record without a zero on record. So no theorem whose hypothesis
is "the state is `(run cfg (State.init cfg) es).1`" applies to it or to any
state that follows it; `mixed_from_init` says what does. -/
theorem noClock_unreachable (st : State) (v : Twist) (hp : st.phase = .active)
    (hf : v.finite = true) (cfg : Config) (es : List Event) :
    (stepNoClock st v).1 ≠ (run cfg (State.init cfg) es).1 := by
  intro heq
  have hJ := source_report_has_zero cfg es
  rw [← heq, noClock_state st v hp hf] at hJ
  simp at hJ

/-- Two states agree up to the report on record: they are equal, or they
differ only in that `a` has `source` on record where `b` has `normal`, with
no zero on record, in the active or the finalized phase. `a` is the node
after no-clock commands, `b` the state `run` reaches on the substituted
events (`substNoClock`). -/
def AgreesUpToReport (a b : State) : Prop :=
  a.phase = b.phase ∧ a.scan = b.scan ∧ a.shapes = b.shapes ∧ a.prevZero = b.prevZero ∧
    a.stopStamp = b.stopStamp ∧
    (a.prevReport = b.prevReport ∨
      (a.prevReport = .source ∧ b.prevReport = .normal ∧ a.prevZero = false ∧
        (a.phase = .active ∨ a.phase = .finalized)))

theorem agreesUpToReport_refl (a : State) : AgreesUpToReport a a :=
  ⟨rfl, rfl, rfl, rfl, rfl, Or.inl rfl⟩

/-- **One event from two states that agree up to the report.** The states
after it agree up to the report again; the velocity published is the same,
and so is whether the event is a processed command. Only the state message
can differ: after a no-clock command the node has `source` on record, so the
first processed command after it reports where the other run, coming from
`normal`, might not, or the other way round. -/
theorem agreesUpToReport_step (cfg : Config) (a b : State) (e : Event)
    (h : AgreesUpToReport a b) :
    AgreesUpToReport (step cfg a e).1 (step cfg b e).1 ∧
      (step cfg a e).2.velocity = (step cfg b e).2.velocity ∧
      (step cfg a e).2.processed = (step cfg b e).2.processed := by
  rcases a with ⟨p, s, sh, z, t, r⟩
  rcases b with ⟨p', s', sh', z', t', r'⟩
  simp only [AgreesUpToReport] at h
  obtain ⟨rfl, rfl, rfl, rfl, rfl, h⟩ := h
  rcases h with rfl | ⟨rfl, rfl, rfl, hp⟩
  · exact ⟨agreesUpToReport_refl _, rfl, rfl⟩
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
          simp [AgreesUpToReport, Output.velocity, Output.processed]
    | shape k verts foreign =>
      simp only [step, acceptShape]
      repeat' split
      all_goals simp_all [AgreesUpToReport, Output.velocity, Output.processed]
    | _ =>
      rcases hp with rfl | rfl <;>
        simp [step, AgreesUpToReport, Output.velocity, Output.processed]

/-- What the node reacts to, with the command without a clock reading
beside the events of the model. -/
inductive MixedEvent where
  /-- An event of the model (`step`). -/
  | ev (e : Event)
  /-- A command that arrived without a valid clock reading
  (`stepNoClock`). -/
  | noClock (v : Twist)

/-- The state after one event of either kind. -/
def mixedStep (cfg : Config) (st : State) : MixedEvent → State
  | .ev e => (step cfg st e).1
  | .noClock v => (stepNoClock st v).1

/-- The events of the model that stand for a mixed sequence run from the
state `a`: every processed no-clock command (active phase, finite) becomes
`deactivate, activate`; every other no-clock command is dropped; the events
of the model stay. Which no-clock commands are processed depends on the
state they arrive in, hence the argument `a`. -/
def substNoClock (cfg : Config) : State → List MixedEvent → List Event
  | _, [] => []
  | a, .ev e :: l => e :: substNoClock cfg (step cfg a e).1 l
  | a, .noClock v :: l =>
    (if a.phase = .active ∧ v.finite = true then [.deactivate, .activate] else []) ++
      substNoClock cfg (stepNoClock a v).1 l

theorem agreesUpToReport_mixed (cfg : Config) (l : List MixedEvent) (a b : State)
    (h : AgreesUpToReport a b) :
    AgreesUpToReport (l.foldl (mixedStep cfg) a) (run cfg b (substNoClock cfg a l)).1 := by
  induction l generalizing a b with
  | nil => exact h
  | cons m l ih =>
    cases m with
    | ev e =>
      simp only [List.foldl_cons, substNoClock, run_cons, mixedStep]
      exact ih _ _ (agreesUpToReport_step cfg a b e h).1
    | noClock v =>
      simp only [List.foldl_cons, substNoClock, mixedStep, run_append]
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

/-- **Mixed runs from start-up (review CM5, finding 4).** After any sequence
of events and commands without a clock reading since start-up, the node's
state agrees, up to the report on record, with the state that `run` reaches
from `State.init` on the substituted events (`substNoClock`: `deactivate,
activate` for every processed no-clock command). Phase, stored scan, shapes,
the record of zeros and the stop stamp are equal.

So every theorem stated on `run cfg (State.init cfg) es` that concludes
something about those five fields, or (with `mixed_next_velocity`) about the
velocity for the next event, holds for the node after no-clock commands,
read on the substituted events. In words: for everything but one state
message, a processed command without a clock reading acts like a
deactivation followed at once by an activation. -/
theorem mixed_from_init (cfg : Config) (l : List MixedEvent) :
    AgreesUpToReport (l.foldl (mixedStep cfg) (State.init cfg))
      (run cfg (State.init cfg) (substNoClock cfg (State.init cfg) l)).1 :=
  agreesUpToReport_mixed cfg l _ _ (agreesUpToReport_refl _)

/-- **… and the next event is answered with the same velocity.** Whatever
event follows a mixed sequence from start-up, the velocity published for it
(an action, or none) is the one `step` gives in the state `run` reaches on
the substituted events, and it is a processed command in the one iff in the
other. -/
theorem mixed_next_velocity (cfg : Config) (l : List MixedEvent) (e : Event) :
    (step cfg (l.foldl (mixedStep cfg) (State.init cfg)) e).2.velocity =
        (step cfg (run cfg (State.init cfg) (substNoClock cfg (State.init cfg) l)).1 e).2.velocity ∧
      (step cfg (l.foldl (mixedStep cfg) (State.init cfg)) e).2.processed =
        (step cfg (run cfg (State.init cfg) (substNoClock cfg (State.init cfg) l)).1
          e).2.processed :=
  (agreesUpToReport_step cfg _ _ e (mixed_from_init cfg l)).2

end ROSlean.CollisionMonitor

namespace ROSlean.CollisionMonitor.FFI

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI

/-- The call for a command without a clock reading is `Monitor.applyNoClock`
on the six patterns in the order `linear.x`, `linear.y`, `linear.z`,
`angular.x`, `angular.y`, `angular.z`. -/
theorem cmOnCmdNoClock_eq (m : Monitor) (lx ly lz ax ay az : UInt64) :
    cmOnCmdNoClock m lx ly lz ax ay az =
      m.applyNoClock ⟨lx.toNat, ly.toNat, lz.toNat, ax.toNat, ay.toNat, az.toNat⟩ := rfl

/-- An accepted monitor does what `stepNoClock` says. -/
theorem applyNoClock_of_ok (m : Monitor) (v : Twist) (h : m.ok = true) :
    (m.applyNoClock v).st = (stepNoClock m.st v).1 ∧
      (m.applyNoClock v).out = (stepNoClock m.st v).2 := by
  simp [Monitor.applyNoClock, h]

/-- **What the shim reads back without a clock.** For a monitor in the
active phase, accepted or not, a finite command without a clock reading
gives the output kind 4, "publish zero, invalid source". -/
theorem cmOnCmdNoClock_kind (m : Monitor) (lx ly lz ax ay az : UInt64)
    (hp : m.st.phase = .active)
    (hf : Twist.finite ⟨lx.toNat, ly.toNat, lz.toNat, ax.toNat, ay.toNat, az.toNat⟩ = true) :
    cmOutKind (cmOnCmdNoClock m lx ly lz ax ay az) = 4 := by
  rw [cmOnCmdNoClock_eq]
  cases hok : m.ok
  · simp [Monitor.applyNoClock, hok, hp, cmOutKind, kindOf, Output.velocity]
  · simp [cmOutKind, (applyNoClock_of_ok m _ hok).2, kindOf, stepNoClock, hp, hf]

/-- Whatever the phase and the command: the output kind of a command without
a clock reading is 0 (nothing to publish) or 4 (publish zero). It is never
"pass" and never a slowdown. -/
theorem cmOnCmdNoClock_kind_cases (m : Monitor) (lx ly lz ax ay az : UInt64) :
    cmOutKind (cmOnCmdNoClock m lx ly lz ax ay az) = 0 ∨
      cmOutKind (cmOnCmdNoClock m lx ly lz ax ay az) = 4 := by
  rw [cmOnCmdNoClock_eq]
  cases hok : m.ok
  · by_cases hp : m.st.phase = .active <;>
      simp [Monitor.applyNoClock, hok, hp, cmOutKind, kindOf, Output.velocity]
  · simp only [cmOutKind, (applyNoClock_of_ok m _ hok).2, stepNoClock]
    by_cases hp : m.st.phase = .active
    · cases hf : Twist.finite ⟨lx.toNat, ly.toNat, lz.toNat, ax.toNat, ay.toNat, az.toNat⟩ <;>
        simp [hp, hf, kindOf]
    · simp [hp, kindOf]

/-- The configuration, its acceptance and the record of the last shape call
are not changed by a command without a clock reading. -/
theorem cmOnCmdNoClock_keeps (m : Monitor) (lx ly lz ax ay az : UInt64) :
    (cmOnCmdNoClock m lx ly lz ax ay az).cfg = m.cfg ∧
      (cmOnCmdNoClock m lx ly lz ax ay az).ok = m.ok ∧
      (cmOnCmdNoClock m lx ly lz ax ay az).accepted = m.accepted := by
  rw [cmOnCmdNoClock_eq]
  unfold Monitor.applyNoClock
  split <;> exact ⟨rfl, rfl, rfl⟩

end ROSlean.CollisionMonitor.FFI
