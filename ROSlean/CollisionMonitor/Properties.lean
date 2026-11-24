import ROSlean.CollisionMonitor.Basic
import ROSlean.CollisionMonitor.Bits

/-!
# Properties of the collision monitor model: one command

The properties of `docs/collision_monitor/SPEC.md`, section "Properties", and
of `docs/collision_monitor/SPEC_NAV2.md`, section 4, that concern the answer
to one command, as theorems about `ROSleanCore.CollisionMonitor.step` for an
arbitrary state. The lifecycle, shape messages and the invariants of
reachable states are in `ROSlean.CollisionMonitor.Lifecycle`;
`ROSlean.CollisionMonitor.Trace` lifts everything to event sequences.

A command event is `.cmd now v scanTf zoneTfs`: the clock, the velocity, the
transform looked up for the stored scan (or `none`), and the transforms
looked up for the zones. Its output is `silent` or `answer vel rep`
(`Output.velocity`, `Output.report` read the two parts; both are `none` for
`silent`).

* **Processed** (`output_eq_silent_iff`, `processed_iff`): a command is
  processed iff the node is active and the command is finite; nothing else
  ever is.
* **Velocity** (`velocity_eq_some_iff`, `velocity_eq_none_iff`,
  `velocity_cmd`): the velocity of a processed command, in terms of the
  action `evaluate` demands and the two fields of the state that implement
  `stop_pub_timeout` (R15).
* **Report** (`report_eq_some_iff`, `report_cmd`, `step_cmd_prevReport`): the
  state message (N13).
* **Only if** (`evaluate_eq_pass_iff`, `evaluate_eq_slowdown_iff`,
  `evaluate_eq_stopZone_iff`, `evaluate_eq_stopSource_iff`): when each action
  is demanded, in terms of the stored scan, the clock, the transform and the
  zones as they stand for this command (`liveConfig`: the configured zones,
  each topic zone with the shape the state holds for it, refreshed by the
  command's zone transforms).

Nothing is assumed about the clock (A2), and no theorem here needs
`Config.ok`.

What these theorems do **not** cover is the arithmetic the C++ shim performs
on the command: a `slowdown k r` output tells the shim to multiply by the
double with pattern `r`. The theorems bound `r`; the multiplication itself is
trusted (SPEC, T1). Nor do they say that the transforms are the right ones
(SPEC_NAV2, T6).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## The demanded action (`evaluate`) -/

theorem evaluate_of_sourcePoints {cfg : Config} {st : State} {now : Nat} {scanTf : Option Xform}
    {pts : List Point} (zoneTfs : List (Option Xform))
    (h : sourcePoints cfg st now scanTf = some pts) :
    evaluate cfg st now scanTf zoneTfs = zonesAction (liveConfig cfg st zoneTfs) pts := by
  simp [evaluate, h]

/-- `evaluate` reads the state only through the stored scan and the
shapes. -/
theorem evaluate_congr (cfg : Config) (st st' : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (h : st.scan = st'.scan) (hs : st.shapes = st'.shapes) :
    evaluate cfg st now scanTf zoneTfs = evaluate cfg st' now scanTf zoneTfs := by
  simp [evaluate, liveConfig, sourcePoints_congr cfg st st' now scanTf h, hs]

/-- **When a stop for an invalid source is demanded (R8, N27).** Iff the
source is invalid: enabled, with a non-zero timeout, and no scan stored, or
the stored scan too old, or no usable transform for it. -/
theorem evaluate_eq_stopSource_iff (cfg : Config) (st : State) (now : Nat)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    evaluate cfg st now scanTf zoneTfs = .stopSource ↔ SourceInvalid cfg st now scanTf := by
  rw [← sourcePoints_eq_none_iff]
  unfold evaluate
  cases h : sourcePoints cfg st now scanTf with
  | none => simp
  | some pts => simp [zonesAction_ne_stopSource]

/-- **When a stop by zone `k` is demanded (R12).** Iff the source is valid
and `k` is the first stop zone triggered by the points tested. -/
theorem evaluate_eq_stopZone_iff (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k : Nat) :
    evaluate cfg st now scanTf zoneTfs = .stopZone k ↔
      ∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        IsFirstStop (liveConfig cfg st zoneTfs).zones pts k := by
  unfold evaluate
  cases h : sourcePoints cfg st now scanTf with
  | none => simp
  | some pts => simp [zonesAction_eq_stopZone_iff]

/-- **When a slowdown by ratio `r`, named after zone `k`, is demanded (R12,
R13).** Iff the source is valid, no stop zone is triggered, and `k` is the
first triggered slowdown zone with the smallest ratio pattern, `r`, which is
below the pattern of 1.0. -/
theorem evaluate_eq_slowdown_iff (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k r : Nat) :
    evaluate cfg st now scanTf zoneTfs = .slowdown k r ↔
      ∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        NoStop (liveConfig cfg st zoneTfs).zones pts ∧
        IsBestSlowdown (liveConfig cfg st zoneTfs).zones pts k r := by
  unfold evaluate
  cases h : sourcePoints cfg st now scanTf with
  | none => simp
  | some pts => simp [zonesAction_eq_slowdown_iff]

/-- **When the command is left alone (R14).** Iff the source is valid, no
stop zone is triggered, and no slowdown zone with a ratio below 1 is
triggered. -/
theorem evaluate_eq_pass_iff (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) :
    evaluate cfg st now scanTf zoneTfs = .pass ↔
      ∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        NoStop (liveConfig cfg st zoneTfs).zones pts ∧
        NoSlowdown (liveConfig cfg st zoneTfs).zones pts := by
  unfold evaluate
  cases h : sourcePoints cfg st now scanTf with
  | none => simp
  | some pts => simp [zonesAction_eq_pass_iff]

/-- R15, D6: the velocity an action yields is zero iff the action is a stop,
or the command is zero in the three published components, or the action is a
slowdown whose ratio is a zero. -/
theorem yieldsZero_iff (a : Action) (v : Twist) :
    a.yieldsZero v = true ↔
      a.isStop = true ∨ v.planarZero = true ∨ ∃ k r, a = .slowdown k r ∧ F64.isZero r = true := by
  cases a <;> simp [Action.yieldsZero, Action.isStop]

/-- A zone of the zones in force is a configured zone with the same action,
threshold and flag. -/
theorem liveConfig_action (cfg : Config) (st : State) (zoneTfs : List (Option Xform)) (k : Nat)
    (z : Zone) (h : (liveConfig cfg st zoneTfs).zones[k]? = some z) :
    ∃ z0, cfg.zones[k]? = some z0 ∧ z.action = z0.action ∧ z.minPoints = z0.minPoints ∧
      z.enabled = z0.enabled := by
  obtain ⟨z0, h0, -, h1, h2, h3⟩ := withShapes_action cfg _ k z h
  exact ⟨z0, h0, h1, h2, h3⟩

/-! ## One command: the state -/

/-- **Inactive (N4).** A command that arrives while the node is not active
is dropped: nothing is published and the state is unchanged. -/
theorem step_cmd_of_not_active (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (h : st.phase ≠ .active) :
    step cfg st (.cmd now v scanTf zoneTfs) = (st, .silent) := by
  simp [step, h]

/-- **Bad data (b), R2.** A command with an infinite or NaN component is
ignored: nothing is published and the state is unchanged. -/
theorem step_cmd_of_not_finite (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (h : v.finite = false) :
    step cfg st (.cmd now v scanTf zoneTfs) = (st, .silent) := by
  by_cases hp : st.phase = .active <;> simp [step, hp, h]

/-- **A processed command (E8).** In the active phase a finite command
changes the state and is answered as follows, with `a` the demanded action. -/
theorem step_cmd (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true) :
    step cfg st (.cmd now v scanTf zoneTfs) =
      ({ st with
          shapes := shapesAfter cfg st now scanTf zoneTfs
          prevZero := (evaluate cfg st now scanTf zoneTfs).yieldsZero v
          stopStamp :=
            if (evaluate cfg st now scanTf zoneTfs).yieldsZero v && !st.prevZero then now
            else st.stopStamp
          prevReport := (evaluate cfg st now scanTf zoneTfs).report v },
        .answer
          (if (evaluate cfg st now scanTf zoneTfs).yieldsZero v && st.prevZero &&
              decide (st.stopStamp + cfg.stopPubTimeout < now) then none
            else some (evaluate cfg st now scanTf zoneTfs))
          (if (evaluate cfg st now scanTf zoneTfs).report v = st.prevReport then none
            else some ((evaluate cfg st now scanTf zoneTfs).report v))) := by
  simp only [step]
  rw [if_neg (by simp [hp]), if_neg (by simp [hf])]

/-- A command never changes the phase or the stored scan. -/
theorem step_cmd_scan (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).1.scan = st.scan ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).1.phase = st.phase := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; exact ⟨rfl, rfl⟩
    · rw [step_cmd _ _ _ _ _ _ hp hf]; exact ⟨rfl, rfl⟩
  · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; exact ⟨rfl, rfl⟩

/-- R15: after a processed command, `prevZero` says whether the velocity
demanded for it was zero, published or not. -/
theorem step_cmd_prevZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).1.prevZero =
      (evaluate cfg st now scanTf zoneTfs).yieldsZero v := by
  rw [step_cmd _ _ _ _ _ _ hp hf]

/-- R15: after a processed command, the stop stamp is `now` if the demanded
velocity is zero and the previous one was not; otherwise it is unchanged. -/
theorem step_cmd_stopStamp (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).1.stopStamp =
      if (evaluate cfg st now scanTf zoneTfs).yieldsZero v = true ∧ st.prevZero = false then now
      else st.stopStamp := by
  rw [step_cmd _ _ _ _ _ _ hp hf]
  simp

/-- **Report, the state (N13).** After a processed command the previous
report is what that command reports, whether a message was published for it
or not. -/
theorem step_cmd_prevReport (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).1.prevReport =
      (evaluate cfg st now scanTf zoneTfs).report v := by
  rw [step_cmd _ _ _ _ _ _ hp hf]

/-- E8 step 4: after a processed command the shapes are the refreshed ones,
or unchanged if the source was invalid. -/
theorem step_cmd_shapes (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).1.shapes = shapesAfter cfg st now scanTf zoneTfs := by
  rw [step_cmd _ _ _ _ _ _ hp hf]

/-! ## One command: the output -/

/-- **Processed (iff), for a command.** The output of a command is `silent`
iff the node is not active (N4) or the command is not finite (R2). -/
theorem output_eq_silent_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2 = .silent ↔
      st.phase ≠ .active ∨ v.finite = false := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; simp
    · rw [step_cmd _ _ _ _ _ _ hp hf]; simp [hp]
  · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; simp [hp]

/-- **Velocity of a processed command (R15).** The demanded action, unless
the velocity it yields is zero, the velocity demanded for the previous
command was zero too, and the stop stamp is more than `stop_pub_timeout`
before `now`: then none. -/
theorem velocity_cmd (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
      if (evaluate cfg st now scanTf zoneTfs).yieldsZero v = true ∧ st.prevZero = true ∧
        st.stopStamp + cfg.stopPubTimeout < now then none
      else some (evaluate cfg st now scanTf zoneTfs) := by
  rw [step_cmd _ _ _ _ _ _ hp hf]
  simp [Output.velocity, and_assoc]

/-- **Report of a processed command (N13).** What the action reports, unless
that is what the previous processed command reported: then none. The
velocity does not enter: a report is made also when the velocity is
withheld. -/
theorem report_cmd (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.report =
      if (evaluate cfg st now scanTf zoneTfs).report v = st.prevReport then none
      else some ((evaluate cfg st now scanTf zoneTfs).report v) := by
  rw [step_cmd _ _ _ _ _ _ hp hf]
  rfl

/-- A processed command's output is an `answer` with the two parts of
`velocity_cmd` and `report_cmd`. -/
theorem output_cmd (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2 =
      .answer (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity
        (step cfg st (.cmd now v scanTf zoneTfs)).2.report := by
  rw [step_cmd _ _ _ _ _ _ hp hf]
  rfl

/-- **Velocity (iff): when an action is published.** The output of a command
carries the velocity action `a` iff the node is active (N4), the command is
finite (R2), `a` is the action demanded, and it is not the case that all
three hold: the velocity `a` yields is zero, the velocity demanded for the
previous command was zero, and the stop stamp is more than
`stop_pub_timeout` before `now` (R15). -/
theorem velocity_eq_some_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (a : Action) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some a ↔
      st.phase = .active ∧ v.finite = true ∧ evaluate cfg st now scanTf zoneTfs = a ∧
        ¬ (a.yieldsZero v = true ∧ st.prevZero = true ∧
          st.stopStamp + cfg.stopPubTimeout < now) := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; simp [Output.velocity]
    · rw [velocity_cmd _ _ _ _ _ _ hp hf]
      split
      · rename_i h
        constructor
        · intro h'; cases h'
        · rintro ⟨-, -, rfl, hn⟩; exact absurd h hn
      · rename_i h
        simp only [Option.some.injEq, hp, true_and]
        constructor
        · rintro rfl; exact ⟨rfl, h⟩
        · exact fun h' => h'.1
  · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; simp [Output.velocity, hp]

/-- **Velocity (iff): when none is published.** The output of a command
carries no velocity iff the node is not active, or the command is not finite,
or the demanded velocity is zero, the one demanded for the previous command
was zero too, and the stop stamp is more than `stop_pub_timeout` before `now`
(R15). In the first two cases the output is `silent`; in the third it is
`answer none _`. -/
theorem velocity_eq_none_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = none ↔
      st.phase ≠ .active ∨ v.finite = false ∨
        ((evaluate cfg st now scanTf zoneTfs).yieldsZero v = true ∧ st.prevZero = true ∧
          st.stopStamp + cfg.stopPubTimeout < now) := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; simp [Output.velocity]
    · rw [velocity_cmd _ _ _ _ _ _ hp hf]
      split
      · rename_i h; simp [h]
      · rename_i h; simp [hp, h]
  · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; simp [Output.velocity, hp]

/-- The velocity of a command is either none or the demanded action; the
node never publishes anything else. -/
theorem velocity_eq_none_or (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = none ∨
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        some (evaluate cfg st now scanTf zoneTfs) := by
  by_cases h : (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = none
  · exact Or.inl h
  · right
    rw [velocity_eq_none_iff] at h
    rw [velocity_eq_some_iff]
    simp only [not_or, Bool.not_eq_false, not_not] at h
    exact ⟨h.1, h.2.1, rfl, h.2.2⟩

/-- **Report (iff).** The output of a command carries the report `r` iff the
node is active, the command is finite, `r` is what the demanded action
reports for it (N13, N14), and `r` differs from the previous report. Whether
the velocity is published or withheld does not matter. -/
theorem report_eq_some_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (r : Report) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.report = some r ↔
      st.phase = .active ∧ v.finite = true ∧
        (evaluate cfg st now scanTf zoneTfs).report v = r ∧ r ≠ st.prevReport := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; simp [Output.report]
    · rw [report_cmd _ _ _ _ _ _ hp hf]
      split
      · rename_i h
        simp only [reduceCtorEq, hp, true_and, false_iff, not_and, ne_eq, not_not]
        rintro rfl
        exact h
      · rename_i h
        simp only [Option.some.injEq, hp, true_and]
        constructor
        · rintro rfl; exact ⟨rfl, h⟩
        · exact fun h' => h'.1
  · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; simp [Output.report, hp]

/-- What each action reports (N13, N14). -/
theorem report_eq (a : Action) (v : Twist) :
    a.report v =
      match a with
      | .stopSource => .source
      | .stopZone k => .stop k
      | .slowdown k _ => if v.planarZero = true then .normal else .slowdown k
      | .pass => .normal := by
  cases a <;> rfl

/-- R15: a processed command for which the demanded velocity is not zero is
always answered with it. -/
theorem velocity_of_not_yieldsZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) (hz : (evaluate cfg st now scanTf zoneTfs).yieldsZero v = false) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
      some (evaluate cfg st now scanTf zoneTfs) := by
  rw [velocity_eq_some_iff]
  exact ⟨hp, hf, rfl, fun h => by simp [hz] at h⟩

/-- R15: the first zero after a non-zero velocity is always published. -/
theorem velocity_of_not_prevZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) (hpz : st.prevZero = false) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
      some (evaluate cfg st now scanTf zoneTfs) := by
  rw [velocity_eq_some_iff]
  exact ⟨hp, hf, rfl, fun h => by simp [hpz] at h⟩

/-- R15: up to and including `stop_pub_timeout` after the stop stamp, every
processed command is answered with a velocity. -/
theorem velocity_of_within_stopPubTimeout (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) (ht : now ≤ st.stopStamp + cfg.stopPubTimeout) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
      some (evaluate cfg st now scanTf zoneTfs) := by
  rw [velocity_eq_some_iff]
  exact ⟨hp, hf, rfl, fun h => by omega⟩

/-! ### Each published action, written out -/

/-- **`pass` is published** iff the node is active, the command is finite,
the source is valid, no stop zone and no slowdown zone with a ratio below 1
is triggered, and the command is not a zero that R15 suppresses. -/
theorem velocity_eq_pass_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some .pass ↔
      st.phase = .active ∧ v.finite = true ∧
      (∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        NoStop (liveConfig cfg st zoneTfs).zones pts ∧
        NoSlowdown (liveConfig cfg st zoneTfs).zones pts) ∧
      ¬ (v.planarZero = true ∧ st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [velocity_eq_some_iff, evaluate_eq_pass_iff]
  simp [Action.yieldsZero]

/-- **`slowdown k r` is published** iff the node is active, the command is
finite, the source is valid, no stop zone is triggered, `k` is the best
slowdown zone with ratio `r`, and the result is not a zero that R15
suppresses. -/
theorem velocity_eq_slowdown_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (k r : Nat) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some (.slowdown k r) ↔
      st.phase = .active ∧ v.finite = true ∧
      (∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        NoStop (liveConfig cfg st zoneTfs).zones pts ∧
        IsBestSlowdown (liveConfig cfg st zoneTfs).zones pts k r) ∧
      ¬ ((v.planarZero = true ∨ F64.isZero r = true) ∧ st.prevZero = true ∧
        st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [velocity_eq_some_iff, evaluate_eq_slowdown_iff]
  simp [Action.yieldsZero]

/-- **`stopZone k` is published** iff the node is active, the command is
finite, the source is valid, `k` is the first triggered stop zone, and R15
does not suppress the zero. -/
theorem velocity_eq_stopZone_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (k : Nat) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some (.stopZone k) ↔
      st.phase = .active ∧ v.finite = true ∧
      (∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        IsFirstStop (liveConfig cfg st zoneTfs).zones pts k) ∧
      ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [velocity_eq_some_iff, evaluate_eq_stopZone_iff]
  simp [Action.yieldsZero]

/-- **`stopSource` is published** iff the node is active, the command is
finite, the source is invalid, and R15 does not suppress the zero. -/
theorem velocity_eq_stopSource_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some .stopSource ↔
      st.phase = .active ∧ v.finite = true ∧ SourceInvalid cfg st now scanTf ∧
      ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [velocity_eq_some_iff, evaluate_eq_stopSource_iff]
  simp [Action.yieldsZero]

/-! ## The properties of the SPEC -/

theorem yieldsZero_of_isStop (a : Action) (v : Twist) (h : a.isStop = true) :
    a.yieldsZero v = true := by
  cases a <;> simp_all [Action.yieldsZero, Action.isStop]

/-- **A demanded stop is published, except in one case.** If the node is
active, the command is finite and the action demanded is a stop (by a zone
or for an invalid source), the velocity published is that stop, unless the
velocity demanded for the previous command was already zero and the stop
stamp is more than `stop_pub_timeout` before `now` (R15); then, and only
then, the velocity is withheld (the output is `answer none _`). -/
theorem velocity_of_isStop (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .active)
    (hf : v.finite = true) (h : (evaluate cfg st now scanTf zoneTfs).isStop = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
      if st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now then none
      else some (evaluate cfg st now scanTf zoneTfs) := by
  rw [velocity_cmd _ _ _ _ _ _ hp hf]
  simp [yieldsZero_of_isStop _ v h]

/-- **Stop.** If the node is active, the command is finite, the source is
valid with points `pts`, and some stop zone is triggered by `pts`, then the
action demanded is `stopZone k` for the first such zone `k`, and that is the
velocity published, unless zero has already been demanded for more than
`stop_pub_timeout`; then, and only then, the velocity is withheld. A stop
zone thereby wins over every slowdown zone, wherever it stands in the
configuration. The zone is one of the zones in force (`liveConfig`): a fixed
zone, or a topic zone with the shape it has. -/
theorem stop (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (pts : List Point) (z : Zone) (hp : st.phase = .active)
    (hf : v.finite = true) (hpts : sourcePoints cfg st now scanTf = some pts)
    (hz : z ∈ (liveConfig cfg st zoneTfs).zones) (hs : z.action = .stop)
    (ht : z.triggered pts = true) :
    ∃ k, IsFirstStop (liveConfig cfg st zoneTfs).zones pts k ∧
      evaluate cfg st now scanTf zoneTfs = .stopZone k ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        if st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now then none
        else some (.stopZone k) := by
  obtain ⟨k, hk⟩ := exists_isFirstStop hz hs ht
  have he : evaluate cfg st now scanTf zoneTfs = .stopZone k :=
    (evaluate_eq_stopZone_iff cfg st now scanTf zoneTfs k).mpr ⟨pts, hpts, hk⟩
  refine ⟨k, hk, he, ?_⟩
  rw [velocity_of_isStop cfg st now v scanTf zoneTfs hp hf (by rw [he]; rfl), he]

/-- **A stop zone wins over any slowdown zone.** With a triggered stop zone,
neither `pass` nor a slowdown is ever published. -/
theorem stop_not_pass_or_slowdown (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (pts : List Point) (z : Zone)
    (hpts : sourcePoints cfg st now scanTf = some pts)
    (hz : z ∈ (liveConfig cfg st zoneTfs).zones) (hs : z.action = .stop)
    (ht : z.triggered pts = true) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity ≠ some .pass ∧
      ∀ k r, (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity ≠ some (.slowdown k r) := by
  obtain ⟨k, hk⟩ := exists_isFirstStop hz hs ht
  have he : evaluate cfg st now scanTf zoneTfs = .stopZone k :=
    (evaluate_eq_stopZone_iff cfg st now scanTf zoneTfs k).mpr ⟨pts, hpts, hk⟩
  rcases velocity_eq_none_or cfg st now v scanTf zoneTfs with h | h
  · rw [h]; simp
  · rw [h, he]; simp

/-- **Invalid source.** If the node is active, the command is finite and the
source is invalid, the action demanded is `stopSource`, and that is the
velocity published, unless zero has already been demanded for more than
`stop_pub_timeout`; then, and only then, the velocity is withheld. `stale`
and `no_transform` are its two readings. -/
theorem source_invalid (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true)
    (hs : SourceInvalid cfg st now scanTf) :
    evaluate cfg st now scanTf zoneTfs = .stopSource ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        if st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now then none
        else some .stopSource := by
  have h : evaluate cfg st now scanTf zoneTfs = .stopSource :=
    (evaluate_eq_stopSource_iff cfg st now scanTf zoneTfs).mpr hs
  refine ⟨h, ?_⟩
  rw [velocity_of_isStop cfg st now v scanTf zoneTfs hp hf (by rw [h]; rfl), h]

/-- **Stale data.** If the source is enabled with a non-zero timeout, and no
scan is stored or the stored scan's stamp is more than the timeout before
`now`, then, whatever transform the command carries, the action demanded is
`stopSource`; an active node publishes it for a finite command, unless zero
has already been demanded for more than `stop_pub_timeout`. -/
theorem stale (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout)
    (hs : st.scan = none ∨ ∃ s, st.scan = some s ∧ s.stamp + cfg.sourceTimeout < now) :
    evaluate cfg st now scanTf zoneTfs = .stopSource ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        if st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now then none
        else some .stopSource :=
  source_invalid cfg st now v scanTf zoneTfs hp hf
    ⟨he, ht, hs.elim Or.inl (fun h => Or.inr (Or.inl h))⟩

/-- **No transform (N27).** If the source is enabled with a non-zero timeout
and the command carries no usable transform for the scan (the lookup failed,
or was not made), then, whatever scan is stored, the action demanded is
`stopSource`; an active node publishes it for a finite command, unless zero
has already been demanded for more than `stop_pub_timeout`.

Both hypotheses on the configuration are needed
(`evaluate_ne_stopSource_of_off`, and the example with `source_timeout: 0`
in `Examples.lean`): with timeout 0 a missing transform means "no
obstacle". -/
theorem no_transform (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (hp : st.phase = .active) (hf : v.finite = true)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout) (hn : NoTransform scanTf) :
    evaluate cfg st now scanTf zoneTfs = .stopSource ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        if st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now then none
        else some .stopSource :=
  source_invalid cfg st now v scanTf zoneTfs hp hf ⟨he, ht, Or.inr (Or.inr hn)⟩

/-- **Only if (stop zone).** A stop named after zone `k` is demanded only if
zone `k` of the configuration is a stop zone, is enabled, has a shape (its
fixed one, or the one the state holds for it), and has at least `minPoints`
of the tested points inside that shape. -/
theorem evaluate_stopZone_only_if (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k : Nat)
    (h : evaluate cfg st now scanTf zoneTfs = .stopZone k) :
    ∃ pts z z0, sourcePoints cfg st now scanTf = some pts ∧
      (liveConfig cfg st zoneTfs).zones[k]? = some z ∧ cfg.zones[k]? = some z0 ∧
      z0.action = .stop ∧ z0.enabled = true ∧ z.shape.isSet = true ∧
      z0.minPoints ≤ pts.countP z.shape.contains := by
  obtain ⟨pts, hpts, z, hz, ha, ht, -⟩ :=
    (evaluate_eq_stopZone_iff cfg st now scanTf zoneTfs k).mp h
  obtain ⟨z0, h0, h1, h2, h3⟩ := liveConfig_action cfg st zoneTfs k z hz
  obtain ⟨t1, t2, t3⟩ := (triggered_iff z pts).mp ht
  exact ⟨pts, z, z0, hpts, hz, h0, by rw [← h1]; exact ha, by rw [← h3]; exact t1, t2,
    by rw [← h2]; exact t3⟩

/-- **Only if (slowdown).** A slowdown is demanded only if no stop zone is
triggered and the zone it names is an enabled slowdown zone of the
configuration with that ratio, has a shape, and has at least `minPoints` of
the tested points inside. So a monitor that stops whenever a slowdown zone
triggers is not this one. -/
theorem evaluate_slowdown_only_if (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k r : Nat)
    (h : evaluate cfg st now scanTf zoneTfs = .slowdown k r) :
    ∃ pts z z0, sourcePoints cfg st now scanTf = some pts ∧
      NoStop (liveConfig cfg st zoneTfs).zones pts ∧
      (liveConfig cfg st zoneTfs).zones[k]? = some z ∧ cfg.zones[k]? = some z0 ∧
      z0.action = .slowdown r ∧ r < F64.one ∧ z0.enabled = true ∧ z.shape.isSet = true ∧
      z0.minPoints ≤ pts.countP z.shape.contains := by
  obtain ⟨pts, hpts, hns, z, hz, ha, ht, hlt, -⟩ :=
    (evaluate_eq_slowdown_iff cfg st now scanTf zoneTfs k r).mp h
  obtain ⟨z0, h0, h1, h2, h3⟩ := liveConfig_action cfg st zoneTfs k z hz
  obtain ⟨t1, t2, t3⟩ := (triggered_iff z pts).mp ht
  exact ⟨pts, z, z0, hpts, hns, hz, h0, by rw [← h1]; exact ha, hlt, by rw [← h3]; exact t1, t2,
    by rw [← h2]; exact t3⟩

/-- **Only if (stop at all).** Zero is demanded whatever the command only if
the source is invalid or a stop zone is triggered. -/
theorem evaluate_isStop_iff (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) :
    (evaluate cfg st now scanTf zoneTfs).isStop = true ↔
      sourcePoints cfg st now scanTf = none ∨
      ∃ pts, sourcePoints cfg st now scanTf = some pts ∧
        ∃ z ∈ (liveConfig cfg st zoneTfs).zones, z.action = .stop ∧ z.triggered pts = true := by
  unfold evaluate
  generalize liveConfig cfg st zoneTfs = lc
  cases hsp : sourcePoints cfg st now scanTf with
  | none => simp [Action.isStop]
  | some pts =>
    simp only [reduceCtorEq, Option.some.injEq, exists_eq_left', false_or]
    cases hact : zonesAction lc pts with
    | stopZone k =>
      obtain ⟨z, hz, ha, ht, -⟩ := (zonesAction_eq_stopZone_iff lc pts k).mp hact
      simp only [Action.isStop, true_iff]
      exact ⟨z, List.mem_of_getElem? hz, ha, ht⟩
    | stopSource => exact absurd hact (zonesAction_ne_stopSource lc pts)
    | pass =>
      have hn := ((zonesAction_eq_pass_iff lc pts).mp hact).1
      simp only [Action.isStop, Bool.false_eq_true, false_iff, not_exists, not_and]
      intro z hz ha ht
      rw [hn z hz ha] at ht
      exact Bool.false_ne_true ht
    | slowdown k r =>
      have hn := ((zonesAction_eq_slowdown_iff lc pts k r).mp hact).1
      simp only [Action.isStop, Bool.false_eq_true, false_iff, not_exists, not_and]
      intro z hz ha ht
      rw [hn z hz ha] at ht
      exact Bool.false_ne_true ht

/-- The hypotheses of `stale` and `no_transform` on the configuration are
needed: with timeout 0 or a disabled source, the source is never invalid
(R6, R8, N27), whatever is stored and whatever transform is missing. -/
theorem evaluate_ne_stopSource_of_off (cfg : Config) (st : State) (now : Nat)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform))
    (h : cfg.sourceEnabled = false ∨ cfg.sourceTimeout = 0) :
    evaluate cfg st now scanTf zoneTfs ≠ .stopSource := by
  rw [Ne, evaluate_eq_stopSource_iff]
  rintro ⟨h1, h2, -⟩
  rcases h with h | h
  · simp [h] at h1
  · omega

/-- N27, the fail-open case, as upstream: with `source_timeout` 0, an enabled
source, a stored scan and no usable transform, the zones are tested against
no points at all. -/
theorem sourcePoints_of_no_transform_timeout_zero (cfg : Config) (st : State) (now : Nat)
    (scanTf : Option Xform) (s : Scan) (he : cfg.sourceEnabled = true)
    (h0 : cfg.sourceTimeout = 0) (hs : st.scan = some s) (hn : NoTransform scanTf) :
    sourcePoints cfg st now scanTf = some [] := by
  rw [sourcePoints_eq_some_iff]
  exact Or.inr (Or.inr ⟨he, s, hs, by omega, Or.inr ⟨hn, h0, rfl⟩⟩)

/-! ## Scan points -/

/-- The points of a scan under a transform are `t.pointMm` of the
coordinates of its valid readings, in order. -/
theorem points_eq (s : Scan) (t : Xform) :
    s.points t = (s.beams.filter s.validBeam).map (fun b => t.pointMm b.x b.y) := by
  unfold Scan.points Xform.pointMm
  split <;> rfl

/-- **Bad data (a), R4.** The points of a scan are the images of its valid
readings. -/
theorem mem_points_iff (s : Scan) (t : Xform) (p : Point) :
    p ∈ s.points t ↔ ∃ b ∈ s.beams, s.validBeam b = true ∧ t.pointMm b.x b.y = p := by
  simp [points_eq, and_assoc]

/-- Two scans with the same stamp and the same valid points under every
transform are answered alike: the stored scan enters the decision only
through these. -/
theorem output_congr_scan (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (s s' : Scan)
    (hstamp : s.stamp = s'.stamp) (hpts : ∀ t, s.points t = s'.points t) :
    (step cfg { st with scan := some s } (.cmd now v scanTf zoneTfs)).2 =
      (step cfg { st with scan := some s' } (.cmd now v scanTf zoneTfs)).2 := by
  have he : evaluate cfg { st with scan := some s } now scanTf zoneTfs =
      evaluate cfg { st with scan := some s' } now scanTf zoneTfs := by
    simp [evaluate, sourcePoints, liveConfig, hstamp, hpts]
  by_cases hp : st.phase = .active
  · cases hf : v.finite
    · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf, step_cmd_of_not_finite _ _ _ _ _ _ hf]
    · rw [step_cmd _ _ _ _ _ _ (by exact hp) hf, step_cmd _ _ _ _ _ _ (by exact hp) hf, he]
  · rw [step_cmd_of_not_active _ _ _ _ _ _ (by exact hp),
      step_cmd_of_not_active _ _ _ _ _ _ (by exact hp)]

/-- **Bad data (a), R4: invalid readings never change the decision.** Removing
from the stored scan every reading that is not valid (NaN, below `range_min`,
above `range_max`, or any reading at all when a limit is NaN) leaves the
output of every command unchanged. -/
theorem output_filter_validBeam (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (s : Scan) :
    (step cfg { st with scan := some { s with beams := s.beams.filter s.validBeam } }
        (.cmd now v scanTf zoneTfs)).2 =
      (step cfg { st with scan := some s } (.cmd now v scanTf zoneTfs)).2 := by
  apply output_congr_scan
  · rfl
  · intro t
    have hv : Scan.validBeam { s with beams := s.beams.filter s.validBeam } = s.validBeam := rfl
    simp [points_eq, hv]

/-- **Bad data (a), R4.** A reading that is not valid contributes nothing: the
scan has the same points with it as without it, whatever its coordinates
and whatever the transform. So it changes no zone count and no decision. -/
theorem points_of_not_validBeam (stamp rangeMin rangeMax : Nat) (bs₁ bs₂ : List Beam) (b : Beam)
    (t : Xform)
    (h : (Scan.mk stamp rangeMin rangeMax (bs₁ ++ b :: bs₂)).validBeam b = false) :
    (Scan.mk stamp rangeMin rangeMax (bs₁ ++ b :: bs₂)).points t =
      (Scan.mk stamp rangeMin rangeMax (bs₁ ++ bs₂)).points t := by
  have hv : ∀ bs, Scan.validBeam ⟨stamp, rangeMin, rangeMax, bs⟩ =
      fun b => F32.le rangeMin b.range && F32.le b.range rangeMax := fun _ => rfl
  rw [hv] at h
  simp [points_eq, hv, h]

/-- A scan without valid readings is a scan without obstacles (R4): it has no
points, under any transform. -/
theorem points_eq_nil_iff (s : Scan) (t : Xform) :
    s.points t = [] ↔ ∀ b ∈ s.beams, s.validBeam b = false := by
  simp [points_eq, List.filter_eq_nil_iff]

/-! ## No amplification, slowdown bound, pass-through -/

/-- **No amplification, on the demanded action.** A demanded slowdown
carries a ratio pattern below that of 1.0, and it is the configured ratio of
the zone it names. -/
theorem evaluate_slowdown (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k r : Nat)
    (h : evaluate cfg st now scanTf zoneTfs = .slowdown k r) :
    r < F64.one ∧ ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨pts, z, z0, -, -, -, h0, ha, hlt, -⟩ :=
    evaluate_slowdown_only_if cfg st now scanTf zoneTfs k r h
  exact ⟨hlt, z0, h0, ha⟩

/-- Whatever velocity `step` publishes, for whatever event, is what
`evaluate` demands for a command. -/
theorem step_velocity (cfg : Config) (st : State) (e : Event) (a : Action)
    (h : (step cfg st e).2.velocity = some a) :
    ∃ now v scanTf zoneTfs, e = .cmd now v scanTf zoneTfs ∧ st.phase = .active ∧
      v.finite = true ∧ a = evaluate cfg st now scanTf zoneTfs := by
  cases e with
  | cmd now v scanTf zoneTfs =>
    obtain ⟨hp, hf, he, -⟩ := (velocity_eq_some_iff cfg st now v scanTf zoneTfs a).mp h
    exact ⟨now, v, scanTf, zoneTfs, rfl, hp, hf, he.symm⟩
  | shape k verts place =>
    simp only [step] at h
    split at h <;> simp [Output.velocity] at h
  | _ => simp [step, Output.velocity] at h

/-- **No amplification.** Whatever velocity is published, in answer to
whatever event, is `pass`, a stop, or a slowdown whose ratio pattern `r` is
below `F64.one` and is the configured ratio of zone `k`. By `lt_one_iff`,
such a pattern has its sign bit clear and denotes a number in `[0, 1)`.

This holds for every configuration, accepted or not, and every state. What
it does not say is anything about the product the shim computes with that
ratio (SPEC, T1). -/
theorem no_amplification (cfg : Config) (st : State) (e : Event) (a : Action)
    (h : (step cfg st e).2.velocity = some a) :
    a = .pass ∨ (∃ k, a = .stopZone k) ∨ a = .stopSource ∨
      ∃ k r, a = .slowdown k r ∧ r < F64.one ∧
        ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨now, v, scanTf, zoneTfs, -, -, -, he⟩ := step_velocity cfg st e a h
  cases a with
  | pass => exact Or.inl rfl
  | stopZone k => exact Or.inr (Or.inl ⟨k, rfl⟩)
  | stopSource => exact Or.inr (Or.inr (Or.inl rfl))
  | slowdown k r =>
    exact Or.inr (Or.inr (Or.inr ⟨k, r, rfl,
      evaluate_slowdown cfg st now scanTf zoneTfs k r he.symm⟩))

/-- A published slowdown ratio, read as a number: sign bit clear and
magnitude below that of 1.0 (in units of 2⁻¹⁰⁷⁴). -/
theorem published_ratio_in_unit_interval (cfg : Config) (st : State) (e : Event) (k r : Nat)
    (h : (step cfg st e).2.velocity = some (.slowdown k r)) :
    r < 0x8000000000000000 ∧ magVal 52 r < 2 ^ 1074 := by
  obtain ⟨now, v, scanTf, zoneTfs, -, -, -, he⟩ := step_velocity cfg st e _ h
  exact (lt_one_iff r).mp (evaluate_slowdown cfg st now scanTf zoneTfs k r he.symm).1

/-- **Slowdown bound.** If the source is valid with points `pts` and a
slowdown zone with ratio pattern `rz` below that of 1.0 is triggered, the
action demanded is a stop by a zone, or a slowdown whose ratio pattern is at
most `rz`; in the latter case the ratio is the smallest among the triggered
slowdown zones and the zone named is the first that has it. -/
theorem slowdown_bound (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (pts : List Point) (z : Zone) (rz : Nat)
    (hpts : sourcePoints cfg st now scanTf = some pts)
    (hz : z ∈ (liveConfig cfg st zoneTfs).zones)
    (ha : z.action = .slowdown rz) (ht : z.triggered pts = true) (hlt : rz < F64.one) :
    (∃ k, evaluate cfg st now scanTf zoneTfs = .stopZone k) ∨
      ∃ k r, evaluate cfg st now scanTf zoneTfs = .slowdown k r ∧ r ≤ rz ∧
        IsBestSlowdown (liveConfig cfg st zoneTfs).zones pts k r := by
  rw [evaluate_of_sourcePoints zoneTfs hpts]
  generalize liveConfig cfg st zoneTfs = lc at hz ⊢
  cases hact : zonesAction lc pts with
  | stopZone k => exact Or.inl ⟨k, rfl⟩
  | stopSource => exact absurd hact (zonesAction_ne_stopSource lc pts)
  | pass =>
    have := ((zonesAction_eq_pass_iff lc pts).mp hact).2 z hz rz ha ht
    omega
  | slowdown k r =>
    right
    have hbest := ((zonesAction_eq_slowdown_iff lc pts k r).mp hact).2
    obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp hz
    obtain ⟨z0, -, -, -, -, hmin⟩ := id hbest
    exact ⟨k, r, rfl, (hmin j z rz hj ha ht).1, hbest⟩

/-- **Slowdown bound, for every zone at once.** If a slowdown by `r` is
demanded, then `r` is at most the ratio pattern of every triggered slowdown
zone. For patterns with the sign bit clear this is the order of the numbers
they denote (`magVal_le_iff`). -/
theorem evaluate_slowdown_le (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k r : Nat) (pts : List Point)
    (hpts : sourcePoints cfg st now scanTf = some pts)
    (h : evaluate cfg st now scanTf zoneTfs = .slowdown k r)
    (z : Zone) (rz : Nat) (hz : z ∈ (liveConfig cfg st zoneTfs).zones)
    (ha : z.action = .slowdown rz) (ht : z.triggered pts = true) : r ≤ rz := by
  rw [evaluate_of_sourcePoints zoneTfs hpts, zonesAction_eq_slowdown_iff] at h
  obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp hz
  obtain ⟨z0, -, -, -, -, hmin⟩ := h.2
  exact (hmin j z rz hj ha ht).1

/-- **Pass-through.** If the node is active, the command is finite, the
source is valid with points `pts` (which, when a scan is stored, requires a
usable transform or a timeout of 0), and no zone is triggered by `pts`, then
the action demanded is `pass`, and it is published, except that a command
that is zero in the published components is withheld once zero has been
demanded for more than `stop_pub_timeout` (R15). -/
theorem pass_through (cfg : Config) (st : State) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (pts : List Point) (hp : st.phase = .active)
    (hf : v.finite = true) (hpts : sourcePoints cfg st now scanTf = some pts)
    (hno : ∀ z ∈ (liveConfig cfg st zoneTfs).zones, z.triggered pts = false) :
    evaluate cfg st now scanTf zoneTfs = .pass ∧
      (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity =
        if v.planarZero = true ∧ st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now
        then none else some .pass := by
  have he : evaluate cfg st now scanTf zoneTfs = .pass := by
    rw [evaluate_eq_pass_iff]
    refine ⟨pts, hpts, fun z hz _ => hno z hz, fun z hz r _ ht => ?_⟩
    rw [hno z hz] at ht
    exact absurd ht Bool.false_ne_true
  refine ⟨he, ?_⟩
  rw [velocity_cmd _ _ _ _ _ _ hp hf, he]
  rfl

/-- **Pass-through for a non-zero command.** A finite command that is not
zero in the published components, with an active node, a valid source and
no zone triggered, is always published unchanged, and reports `normal`. The
node is not the one that always stops. -/
theorem pass_through_of_nonzero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (pts : List Point)
    (hp : st.phase = .active) (hf : v.finite = true) (hnz : v.planarZero = false)
    (hpts : sourcePoints cfg st now scanTf = some pts)
    (hno : ∀ z ∈ (liveConfig cfg st zoneTfs).zones, z.triggered pts = false) :
    (step cfg st (.cmd now v scanTf zoneTfs)).2.velocity = some .pass := by
  rw [(pass_through cfg st now v scanTf zoneTfs pts hp hf hpts hno).2]
  simp [hnz]

/-! ## Monotonicity: more points never relax the action -/

/-- A zone triggered by some points is triggered by any list that contains
them. -/
theorem triggered_mono (z : Zone) {pts pts' : List Point} (h : pts.Sublist pts')
    (ht : z.triggered pts = true) : z.triggered pts' = true := by
  rw [triggered_iff] at ht ⊢
  exact ⟨ht.1, ht.2.1, le_trans ht.2.2 (h.countP_le)⟩

/-- **More points never lift a stop.** If the zones demand a stop for `pts`,
they demand a stop for every list that contains `pts`, named after the same
zone or an earlier one. -/
theorem zonesAction_stop_mono (cfg : Config) {pts pts' : List Point} (h : pts.Sublist pts')
    (k : Nat) (hk : zonesAction cfg pts = .stopZone k) :
    ∃ k', k' ≤ k ∧ zonesAction cfg pts' = .stopZone k' := by
  obtain ⟨z, hz, ha, ht, -⟩ := (zonesAction_eq_stopZone_iff cfg pts k).mp hk
  obtain ⟨k', hk'⟩ := exists_isFirstStop (List.mem_of_getElem? hz) ha (triggered_mono z h ht)
  refine ⟨k', ?_, (zonesAction_eq_stopZone_iff cfg pts' k').mpr hk'⟩
  by_contra hlt
  obtain ⟨-, -, -, -, hmin⟩ := hk'
  have := hmin k z (by omega) hz ha
  rw [triggered_mono z h ht] at this
  simp at this

/-- **More points never relax a slowdown.** If the zones demand a slowdown by
`r` for `pts`, then for every list that contains `pts` they demand a stop or
a slowdown whose ratio pattern is at most `r`; never `pass`, never a larger
ratio. -/
theorem zonesAction_slowdown_mono (cfg : Config) {pts pts' : List Point} (h : pts.Sublist pts')
    (k r : Nat) (hk : zonesAction cfg pts = .slowdown k r) :
    (∃ k', zonesAction cfg pts' = .stopZone k') ∨
      ∃ k' r', zonesAction cfg pts' = .slowdown k' r' ∧ r' ≤ r := by
  obtain ⟨-, z, hz, ha, ht, hlt, -⟩ := (zonesAction_eq_slowdown_iff cfg pts k r).mp hk
  have ht' := triggered_mono z h ht
  cases hact : zonesAction cfg pts' with
  | stopZone k' => exact Or.inl ⟨k', rfl⟩
  | stopSource => exact absurd hact (zonesAction_ne_stopSource cfg pts')
  | pass =>
    have := ((zonesAction_eq_pass_iff cfg pts').mp hact).2 z (List.mem_of_getElem? hz) r ha ht'
    omega
  | slowdown k' r' =>
    obtain ⟨-, z0, -, -, -, -, hmin⟩ := (zonesAction_eq_slowdown_iff cfg pts' k' r').mp hact
    exact Or.inr ⟨k', r', rfl, (hmin k z r hz ha ht').1⟩

/-- Adding readings to a scan, with the same range limits, only adds points,
under any transform. -/
theorem points_sublist (stamp stamp' rangeMin rangeMax : Nat) {bs bs' : List Beam}
    (h : bs.Sublist bs') (t : Xform) :
    ((Scan.mk stamp rangeMin rangeMax bs).points t).Sublist
      ((Scan.mk stamp' rangeMin rangeMax bs').points t) := by
  have hv : Scan.validBeam ⟨stamp, rangeMin, rangeMax, bs⟩ =
      Scan.validBeam ⟨stamp', rangeMin, rangeMax, bs'⟩ := rfl
  simp only [points_eq, hv]
  exact (h.filter _).map _

/-- **More readings never lift a stop (one command).** If zero is demanded
with scan `s` stored, it is demanded, for the same clock and the same
transforms, with any scan of the same stamp and range limits that has the
readings of `s` and more. -/
theorem evaluate_isStop_mono (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (stamp rangeMin rangeMax : Nat) {bs bs' : List Beam}
    (h : bs.Sublist bs')
    (hs : (evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs⟩ } now scanTf
      zoneTfs).isStop = true) :
    (evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs'⟩ } now scanTf
      zoneTfs).isStop = true := by
  cases scanTf with
  | none => exact hs
  | some t =>
    cases hu : t.usable
    · have e : ∀ bs, evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs⟩ } now
          (some t) zoneTfs =
          evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, []⟩ } now (some t)
            zoneTfs := by
        intro bs
        simp [evaluate, sourcePoints, hu, liveConfig]
      rw [e] at hs ⊢
      exact hs
    · have hsub := points_sublist stamp stamp rangeMin rangeMax h t
      have key : ∀ bs, evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs⟩ } now
          (some t) zoneTfs =
          if cfg.sourceEnabled = false then zonesAction (liveConfig cfg st zoneTfs) []
          else if 0 < cfg.sourceTimeout ∧ stamp + cfg.sourceTimeout < now then .stopSource
          else zonesAction (liveConfig cfg st zoneTfs)
            (Scan.points ⟨stamp, rangeMin, rangeMax, bs⟩ t) := by
        intro bs
        simp only [evaluate, sourcePoints, hu, liveConfig]
        cases cfg.sourceEnabled
        · simp
        · by_cases ht : 0 < cfg.sourceTimeout ∧ stamp + cfg.sourceTimeout < now <;> simp [ht]
      rw [key] at hs ⊢
      split_ifs at hs ⊢
      · exact hs
      · rfl
      · cases hact : zonesAction (liveConfig cfg st zoneTfs)
            (Scan.points ⟨stamp, rangeMin, rangeMax, bs⟩ t) with
        | stopZone k =>
          obtain ⟨k', -, hk'⟩ := zonesAction_stop_mono _ hsub k hact
          simp [hk', Action.isStop]
        | stopSource => exact absurd hact (zonesAction_ne_stopSource _ _)
        | pass => simp [hact, Action.isStop] at hs
        | slowdown k r => simp [hact, Action.isStop] at hs

end ROSlean.CollisionMonitor
