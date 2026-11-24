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
  (`velocity_after_noClock`).
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
