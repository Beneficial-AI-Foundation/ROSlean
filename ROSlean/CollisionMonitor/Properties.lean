import ROSlean.CollisionMonitor.Basic
import ROSlean.CollisionMonitor.Bits

/-!
# Properties of the collision monitor model: one step

The properties of `docs/collision_monitor/SPEC.md`, section "Properties", as
theorems about `ROSleanCore.CollisionMonitor.step`, for an arbitrary state.
`ROSlean.CollisionMonitor.Trace` lifts them to event sequences.

Two characterisations carry everything else:

* `output_eq_publish_iff`, `output_eq_silent_iff`: what the node does in
  answer to a command, in terms of the action `evaluate` demands and the two
  fields of the state that implement `stop_pub_timeout` (R15);
* `evaluate_eq_pass_iff`, `evaluate_eq_slowdown_iff`,
  `evaluate_eq_stopZone_iff`, `evaluate_eq_stopSource_iff`: when each action
  is demanded, in terms of the stored scan, the clock and the zones.

"The output of the command" is `(step cfg st (.cmd now v)).2`. Nothing is
assumed about the clock (A2), and no theorem here needs `Config.ok`.

What these theorems do **not** cover is the arithmetic the C++ shim performs
on the command: a `slowdown k r` output tells the shim to multiply by the
double with pattern `r`. The theorems bound `r`; the multiplication itself is
trusted (SPEC, T1).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## The demanded action (`evaluate`) -/

theorem evaluate_of_sourcePoints {cfg : Config} {st : State} {now : Nat} {pts : List Point}
    (h : sourcePoints cfg st now = some pts) : evaluate cfg st now = zonesAction cfg pts := by
  simp [evaluate, h]

/-- `evaluate` reads the state only through the stored scan. -/
theorem evaluate_congr (cfg : Config) (st st' : State) (now : Nat) (h : st.scan = st'.scan) :
    evaluate cfg st now = evaluate cfg st' now := by
  simp [evaluate, sourcePoints_congr cfg st st' now h]

/-- **When a stop for an invalid source is demanded (R8).** Iff the source is
enabled, its timeout is not zero, and no scan is stored or the stored scan's
stamp is more than the timeout before `now`. -/
theorem evaluate_eq_stopSource_iff (cfg : Config) (st : State) (now : Nat) :
    evaluate cfg st now = .stopSource ↔
      cfg.sourceEnabled = true ∧ 0 < cfg.sourceTimeout ∧
        (st.scan = none ∨ ∃ s, st.scan = some s ∧ s.stamp + cfg.sourceTimeout < now) := by
  rw [← sourcePoints_eq_none_iff]
  unfold evaluate
  cases h : sourcePoints cfg st now with
  | none => simp
  | some pts => simp [zonesAction_ne_stopSource]

/-- **When a stop by zone `k` is demanded (R12).** Iff the source is valid
and `k` is the first stop zone triggered by the points tested. -/
theorem evaluate_eq_stopZone_iff (cfg : Config) (st : State) (now k : Nat) :
    evaluate cfg st now = .stopZone k ↔
      ∃ pts, sourcePoints cfg st now = some pts ∧ IsFirstStop cfg.zones pts k := by
  unfold evaluate
  cases h : sourcePoints cfg st now with
  | none => simp
  | some pts => simp [zonesAction_eq_stopZone_iff]

/-- **When a slowdown by ratio `r`, named after zone `k`, is demanded (R12,
R13).** Iff the source is valid, no stop zone is triggered, and `k` is the
first triggered slowdown zone with the smallest ratio pattern, `r`, which is
below the pattern of 1.0. -/
theorem evaluate_eq_slowdown_iff (cfg : Config) (st : State) (now k r : Nat) :
    evaluate cfg st now = .slowdown k r ↔
      ∃ pts, sourcePoints cfg st now = some pts ∧ NoStop cfg.zones pts ∧
        IsBestSlowdown cfg.zones pts k r := by
  unfold evaluate
  cases h : sourcePoints cfg st now with
  | none => simp
  | some pts => simp [zonesAction_eq_slowdown_iff]

/-- **When the command is left alone (R14).** Iff the source is valid, no
stop zone is triggered, and no slowdown zone with a ratio below 1 is
triggered. -/
theorem evaluate_eq_pass_iff (cfg : Config) (st : State) (now : Nat) :
    evaluate cfg st now = .pass ↔
      ∃ pts, sourcePoints cfg st now = some pts ∧ NoStop cfg.zones pts ∧
        NoSlowdown cfg.zones pts := by
  unfold evaluate
  cases h : sourcePoints cfg st now with
  | none => simp
  | some pts => simp [zonesAction_eq_pass_iff]

/-- R15, D6: the velocity an action yields is zero iff the action is a stop,
or the command is zero in the three published components, or the action is a
slowdown whose ratio is a zero. -/
theorem yieldsZero_iff (a : Action) (v : Twist) :
    a.yieldsZero v = true ↔
      a.isStop = true ∨ v.planarZero = true ∨ ∃ k r, a = .slowdown k r ∧ F64.isZero r = true := by
  cases a <;> simp [Action.yieldsZero, Action.isStop]

/-! ## One step: state -/

/-- R5: a scan is stored, nothing else changes, and nothing is published. -/
theorem step_scan (cfg : Config) (st : State) (now : Nat) (s : Scan) :
    step cfg st (.scan now s) = ({ st with scan := some s }, .silent) := rfl

/-- D11: an unusable scan discards the stored scan, nothing else changes, and
nothing is published. -/
theorem step_scanUnusable (cfg : Config) (st : State) (now : Nat) :
    step cfg st (.scanUnusable now) = ({ st with scan := none }, .silent) := rfl

/-- **An unusable scan stops the robot by the source rule (D11, R8).** In the
state after an unusable scan, from any state: with the source enabled and a
non-zero timeout, the action demanded at any time is `stopSource`, exactly as
when no scan was ever received. (For how long: `run_stale` in `Trace.lean`,
until the next usable scan.) -/
theorem evaluate_after_scanUnusable (cfg : Config) (st : State) (t now : Nat)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout) :
    evaluate cfg (step cfg st (.scanUnusable t)).1 now = .stopSource := by
  simp [evaluate, sourcePoints, step, he, Nat.pos_iff_ne_zero.mp ht]

/-- An unusable scan leaves the record of published zeros (R15) alone. -/
theorem step_scanUnusable_zero (cfg : Config) (st : State) (now : Nat) :
    (step cfg st (.scanUnusable now)).1.prevZero = st.prevZero ∧
      (step cfg st (.scanUnusable now)).1.stopStamp = st.stopStamp := ⟨rfl, rfl⟩

/-- Only a command is answered: scans, usable or not, publish nothing (R1). -/
theorem output_of_not_cmd (cfg : Config) (st : State) (e : Event)
    (h : ∀ now v, e ≠ .cmd now v) : (step cfg st e).2 = .silent := by
  cases e with
  | scan now s => rfl
  | scanUnusable now => rfl
  | cmd now v => exact absurd rfl (h now v)

/-- **Bad data (b), R2.** A command with an infinite or NaN component is
ignored: nothing is published and the state is unchanged. -/
theorem step_cmd_of_not_finite (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : v.finite = false) : step cfg st (.cmd now v) = (st, .silent) := by
  simp [step, h]

/-- A command never changes the stored scan. -/
theorem step_cmd_scan (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    (step cfg st (.cmd now v)).1.scan = st.scan := by
  simp only [step]
  split
  · rfl
  · split <;> rfl

/-- R15: after a finite command, `prevZero` says whether the velocity demanded
for it was zero, published or not. -/
theorem step_cmd_prevZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : v.finite = true) :
    (step cfg st (.cmd now v)).1.prevZero = (evaluate cfg st now).yieldsZero v := by
  simp only [step, h, Bool.not_true, Bool.false_eq_true, ↓reduceIte]
  split <;> rfl

/-- R15: after a finite command, the stop stamp is `now` if the demanded
velocity is zero and the previous one was not; otherwise it is unchanged. -/
theorem step_cmd_stopStamp (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : v.finite = true) :
    (step cfg st (.cmd now v)).1.stopStamp =
      if (evaluate cfg st now).yieldsZero v = true ∧ st.prevZero = false then now
      else st.stopStamp := by
  simp only [step, h, Bool.not_true, Bool.false_eq_true, ↓reduceIte]
  split <;> simp

/-! ## One step: output -/

/-- A scan event publishes nothing (R1, R5). -/
theorem output_scan (cfg : Config) (st : State) (now : Nat) (s : Scan) :
    (step cfg st (.scan now s)).2 = .silent := rfl

/-- **When an action is published.** The output of a command is
`publish a` iff the command is finite (R2), `a` is the action demanded, and
it is not the case that all three hold: the velocity `a` yields is zero, the
velocity demanded for the previous command was zero, and the stop stamp is
more than `stop_pub_timeout` before `now` (R15). -/
theorem output_eq_publish_iff (cfg : Config) (st : State) (now : Nat) (v : Twist) (a : Action) :
    (step cfg st (.cmd now v)).2 = .publish a ↔
      v.finite = true ∧ evaluate cfg st now = a ∧
        ¬ (a.yieldsZero v = true ∧ st.prevZero = true ∧
          st.stopStamp + cfg.stopPubTimeout < now) := by
  simp only [step]
  cases hf : v.finite
  · simp
  · simp only [Bool.not_true, Bool.false_eq_true, ↓reduceIte, true_and]
    split
    · rename_i h
      simp only [Bool.and_eq_true, decide_eq_true_eq] at h
      constructor
      · intro h'
        cases h'
      · rintro ⟨rfl, hn⟩
        exact absurd ⟨h.1.1, h.1.2, h.2⟩ hn
    · rename_i h
      simp only [Bool.and_eq_true, decide_eq_true_eq] at h
      constructor
      · intro h'
        cases h'
        exact ⟨rfl, fun hc => h ⟨⟨hc.1, hc.2.1⟩, hc.2.2⟩⟩
      · rintro ⟨rfl, _⟩
        rfl

/-- **When nothing is published.** The output of a command is `silent` iff
the command is not finite (R2), or the demanded velocity is zero, the one
demanded for the previous command was zero too, and the stop stamp is more
than `stop_pub_timeout` before `now` (R15). -/
theorem output_eq_silent_iff (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    (step cfg st (.cmd now v)).2 = .silent ↔
      v.finite = false ∨
        ((evaluate cfg st now).yieldsZero v = true ∧ st.prevZero = true ∧
          st.stopStamp + cfg.stopPubTimeout < now) := by
  simp only [step]
  cases hf : v.finite
  · simp
  · simp only [Bool.not_true, Bool.false_eq_true, ↓reduceIte, Bool.true_eq_false, false_or]
    split
    · rename_i h
      simp only [Bool.and_eq_true, decide_eq_true_eq] at h
      simp [h.1.1, h.1.2, h.2]
    · rename_i h
      simp only [Bool.and_eq_true, decide_eq_true_eq] at h
      constructor
      · intro h'
        cases h'
      · rintro ⟨h1, h2, h3⟩
        exact absurd ⟨⟨h1, h2⟩, h3⟩ h

/-- The output of a command is either nothing or the demanded action; the
node never publishes anything else. -/
theorem output_eq_silent_or_publish (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    (step cfg st (.cmd now v)).2 = .silent ∨
      (step cfg st (.cmd now v)).2 = .publish (evaluate cfg st now) := by
  by_cases h : (step cfg st (.cmd now v)).2 = .silent
  · exact Or.inl h
  · right
    rw [output_eq_silent_iff] at h
    rw [output_eq_publish_iff]
    simp only [not_or, Bool.not_eq_false] at h
    exact ⟨h.1, rfl, h.2⟩

/-- R15: a finite command for which the demanded velocity is not zero is
always answered. -/
theorem output_of_not_yieldsZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (hf : v.finite = true) (hz : (evaluate cfg st now).yieldsZero v = false) :
    (step cfg st (.cmd now v)).2 = .publish (evaluate cfg st now) := by
  rw [output_eq_publish_iff]
  exact ⟨hf, rfl, fun h => by simp [hz] at h⟩

/-- R15: the first zero after a non-zero velocity is always published. -/
theorem output_of_not_prevZero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (hf : v.finite = true) (hp : st.prevZero = false) :
    (step cfg st (.cmd now v)).2 = .publish (evaluate cfg st now) := by
  rw [output_eq_publish_iff]
  exact ⟨hf, rfl, fun h => by simp [hp] at h⟩

/-- R15: up to and including `stop_pub_timeout` after the stop stamp, every
finite command is answered. -/
theorem output_of_within_stopPubTimeout (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (hf : v.finite = true) (ht : now ≤ st.stopStamp + cfg.stopPubTimeout) :
    (step cfg st (.cmd now v)).2 = .publish (evaluate cfg st now) := by
  rw [output_eq_publish_iff]
  exact ⟨hf, rfl, fun h => by omega⟩

/-! ### Each published action, written out -/

/-- **`publish pass`**: iff the command is finite, the source is valid, no
stop zone and no slowdown zone with a ratio below 1 is triggered, and the
command is not a zero that R15 suppresses. -/
theorem output_eq_publish_pass_iff (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    (step cfg st (.cmd now v)).2 = .publish .pass ↔
      v.finite = true ∧
      (∃ pts, sourcePoints cfg st now = some pts ∧ NoStop cfg.zones pts ∧
        NoSlowdown cfg.zones pts) ∧
      ¬ (v.planarZero = true ∧ st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [output_eq_publish_iff, evaluate_eq_pass_iff]
  simp [Action.yieldsZero]

/-- **`publish (slowdown k r)`**: iff the command is finite, the source is
valid, no stop zone is triggered, `k` is the best slowdown zone with ratio
`r`, and the result is not a zero that R15 suppresses. -/
theorem output_eq_publish_slowdown_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (k r : Nat) :
    (step cfg st (.cmd now v)).2 = .publish (.slowdown k r) ↔
      v.finite = true ∧
      (∃ pts, sourcePoints cfg st now = some pts ∧ NoStop cfg.zones pts ∧
        IsBestSlowdown cfg.zones pts k r) ∧
      ¬ ((v.planarZero = true ∨ F64.isZero r = true) ∧ st.prevZero = true ∧
        st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [output_eq_publish_iff, evaluate_eq_slowdown_iff]
  simp [Action.yieldsZero]

/-- **`publish (stopZone k)`**: iff the command is finite, the source is
valid, `k` is the first triggered stop zone, and R15 does not suppress the
zero. -/
theorem output_eq_publish_stopZone_iff (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (k : Nat) :
    (step cfg st (.cmd now v)).2 = .publish (.stopZone k) ↔
      v.finite = true ∧
      (∃ pts, sourcePoints cfg st now = some pts ∧ IsFirstStop cfg.zones pts k) ∧
      ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [output_eq_publish_iff, evaluate_eq_stopZone_iff]
  simp [Action.yieldsZero]

/-- **`publish stopSource`**: iff the command is finite, the source is
invalid (enabled, non-zero timeout, and no scan or a scan that is too old),
and R15 does not suppress the zero. -/
theorem output_eq_publish_stopSource_iff (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    (step cfg st (.cmd now v)).2 = .publish .stopSource ↔
      v.finite = true ∧
      (cfg.sourceEnabled = true ∧ 0 < cfg.sourceTimeout ∧
        (st.scan = none ∨ ∃ s, st.scan = some s ∧ s.stamp + cfg.sourceTimeout < now)) ∧
      ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now) := by
  rw [output_eq_publish_iff, evaluate_eq_stopSource_iff]
  simp [Action.yieldsZero]

/-! ## The properties of the SPEC -/

theorem yieldsZero_of_isStop (a : Action) (v : Twist) (h : a.isStop = true) :
    a.yieldsZero v = true := by
  cases a <;> simp_all [Action.yieldsZero, Action.isStop]

/-- **A demanded stop is published, except in one case.** If the action
demanded is a stop (by a zone or for an invalid source), the output is that
stop, unless the command is not finite (R2), or the velocity demanded for the
previous command was already zero and the stop stamp is more than
`stop_pub_timeout` before `now` (R15); in those two cases, and only then,
nothing is published. -/
theorem output_of_isStop (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : (evaluate cfg st now).isStop = true) :
    (step cfg st (.cmd now v)).2 =
      if v.finite = true ∧ ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now)
      then .publish (evaluate cfg st now) else .silent := by
  have hz := yieldsZero_of_isStop _ v h
  split
  · rename_i hc
    rw [output_eq_publish_iff]
    exact ⟨hc.1, rfl, fun h' => hc.2 h'.2⟩
  · rename_i hc
    rw [output_eq_silent_iff]
    by_cases hf : v.finite = true
    · right
      rw [not_and, not_not] at hc
      exact ⟨hz, hc hf⟩
    · left
      simpa using hf

/-- **Stop.** If the source is valid with points `pts` and some stop zone is
triggered by `pts`, then the action demanded is `stopZone k` for the first
such zone `k`, and the output of a command is `publish (stopZone k)`, unless
the command is not finite or zero has already been demanded for more than
`stop_pub_timeout`; then, and only then, it is `silent`. A stop zone thereby
wins over every slowdown zone, wherever it stands in the configuration. -/
theorem stop (cfg : Config) (st : State) (now : Nat) (v : Twist) (pts : List Point) (z : Zone)
    (hpts : sourcePoints cfg st now = some pts) (hz : z ∈ cfg.zones) (hs : z.action = .stop)
    (ht : z.triggered pts = true) :
    ∃ k, IsFirstStop cfg.zones pts k ∧ evaluate cfg st now = .stopZone k ∧
      (step cfg st (.cmd now v)).2 =
        if v.finite = true ∧ ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now)
        then .publish (.stopZone k) else .silent := by
  obtain ⟨k, hk⟩ := exists_isFirstStop hz hs ht
  have he : evaluate cfg st now = .stopZone k :=
    (evaluate_eq_stopZone_iff cfg st now k).mpr ⟨pts, hpts, hk⟩
  refine ⟨k, hk, he, ?_⟩
  rw [output_of_isStop cfg st now v (by rw [he]; rfl), he]

/-- **A stop zone wins over any slowdown zone.** With a triggered stop zone,
the output is never `publish pass` nor `publish (slowdown …)`. -/
theorem stop_not_pass_or_slowdown (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (pts : List Point) (z : Zone) (hpts : sourcePoints cfg st now = some pts)
    (hz : z ∈ cfg.zones) (hs : z.action = .stop) (ht : z.triggered pts = true) :
    (step cfg st (.cmd now v)).2 ≠ .publish .pass ∧
      ∀ k r, (step cfg st (.cmd now v)).2 ≠ .publish (.slowdown k r) := by
  obtain ⟨k, -, -, h⟩ := stop cfg st now v pts z hpts hz hs ht
  rw [h]
  constructor
  · split <;> simp
  · intro k r
    split <;> simp

/-- **Stale data.** If the source is enabled with a non-zero timeout, and no
scan is stored or the stored scan's stamp is more than the timeout before
`now`, then the action demanded is `stopSource`, and the output of a command
is `publish stopSource`, unless the command is not finite or zero has
already been demanded for more than `stop_pub_timeout`; then, and only then,
it is `silent`. -/
theorem stale (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (he : cfg.sourceEnabled = true) (ht : 0 < cfg.sourceTimeout)
    (hs : st.scan = none ∨ ∃ s, st.scan = some s ∧ s.stamp + cfg.sourceTimeout < now) :
    evaluate cfg st now = .stopSource ∧
      (step cfg st (.cmd now v)).2 =
        if v.finite = true ∧ ¬ (st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now)
        then .publish .stopSource else .silent := by
  have h : evaluate cfg st now = .stopSource :=
    (evaluate_eq_stopSource_iff cfg st now).mpr ⟨he, ht, hs⟩
  refine ⟨h, ?_⟩
  rw [output_of_isStop cfg st now v (by rw [h]; rfl), h]

/-- **Only if (stop zone).** A stop named after zone `k` is demanded only if
zone `k` exists, is a stop zone, is enabled, and has at least `minPoints` of
the tested points inside. -/
theorem evaluate_stopZone_only_if (cfg : Config) (st : State) (now k : Nat)
    (h : evaluate cfg st now = .stopZone k) :
    ∃ pts z, sourcePoints cfg st now = some pts ∧ cfg.zones[k]? = some z ∧ z.action = .stop ∧
      z.enabled = true ∧ z.minPoints ≤ pts.countP z.shape.contains := by
  obtain ⟨pts, hpts, z, hz, ha, ht, -⟩ := (evaluate_eq_stopZone_iff cfg st now k).mp h
  exact ⟨pts, z, hpts, hz, ha, (triggered_iff z pts).mp ht⟩

/-- **Only if (slowdown).** A slowdown is demanded only if no stop zone is
triggered and the zone it names is an enabled slowdown zone with that ratio
and at least `minPoints` of the tested points inside. So a monitor that stops
whenever a slowdown zone triggers is not this one. -/
theorem evaluate_slowdown_only_if (cfg : Config) (st : State) (now k r : Nat)
    (h : evaluate cfg st now = .slowdown k r) :
    ∃ pts z, sourcePoints cfg st now = some pts ∧ NoStop cfg.zones pts ∧
      cfg.zones[k]? = some z ∧ z.action = .slowdown r ∧ r < F64.one ∧
      z.enabled = true ∧ z.minPoints ≤ pts.countP z.shape.contains := by
  obtain ⟨pts, hpts, hns, z, hz, ha, ht, hlt, -⟩ := (evaluate_eq_slowdown_iff cfg st now k r).mp h
  exact ⟨pts, z, hpts, hns, hz, ha, hlt, (triggered_iff z pts).mp ht⟩

/-- **Only if (stop at all).** Zero is demanded whatever the command only if
the source is invalid or a stop zone is triggered. -/
theorem evaluate_isStop_iff (cfg : Config) (st : State) (now : Nat) :
    (evaluate cfg st now).isStop = true ↔
      sourcePoints cfg st now = none ∨
      ∃ pts, sourcePoints cfg st now = some pts ∧ ∃ z ∈ cfg.zones, z.action = .stop ∧
        z.triggered pts = true := by
  unfold evaluate
  cases hsp : sourcePoints cfg st now with
  | none => simp [Action.isStop]
  | some pts =>
    simp only [reduceCtorEq, Option.some.injEq, exists_eq_left', false_or]
    cases hact : zonesAction cfg pts with
    | stopZone k =>
      obtain ⟨z, hz, ha, ht, -⟩ := (zonesAction_eq_stopZone_iff cfg pts k).mp hact
      simp only [Action.isStop, true_iff]
      exact ⟨z, List.mem_of_getElem? hz, ha, ht⟩
    | stopSource => exact absurd hact (zonesAction_ne_stopSource cfg pts)
    | pass =>
      have hn := ((zonesAction_eq_pass_iff cfg pts).mp hact).1
      simp only [Action.isStop, Bool.false_eq_true, false_iff, not_exists, not_and]
      intro z hz ha ht
      rw [hn z hz ha] at ht
      exact Bool.false_ne_true ht
    | slowdown k r =>
      have hn := ((zonesAction_eq_slowdown_iff cfg pts k r).mp hact).1
      simp only [Action.isStop, Bool.false_eq_true, false_iff, not_exists, not_and]
      intro z hz ha ht
      rw [hn z hz ha] at ht
      exact Bool.false_ne_true ht

/-- The hypotheses of `stale` on the configuration are needed: with timeout
0 or a disabled source, the source is never invalid (R6, R8). -/
theorem evaluate_ne_stopSource_of_off (cfg : Config) (st : State) (now : Nat)
    (h : cfg.sourceEnabled = false ∨ cfg.sourceTimeout = 0) :
    evaluate cfg st now ≠ .stopSource := by
  rw [Ne, evaluate_eq_stopSource_iff]
  rintro ⟨h1, h2, -⟩
  rcases h with h | h
  · simp [h] at h1
  · omega

/-- **Bad data (a), R4.** The points of a scan are the points of its valid
readings. -/
theorem mem_points_iff (s : Scan) (p : Point) :
    p ∈ s.points ↔ ∃ b ∈ s.beams, s.validBeam b = true ∧ b.point = p := by
  simp [Scan.points, and_assoc]

/-- Two scans with the same stamp and the same valid points are answered
alike: the stored scan enters the decision only through these. -/
theorem output_congr_scan (cfg : Config) (st : State) (now : Nat) (v : Twist) (s s' : Scan)
    (hstamp : s.stamp = s'.stamp) (hpts : s.points = s'.points) :
    (step cfg { st with scan := some s } (.cmd now v)).2 =
      (step cfg { st with scan := some s' } (.cmd now v)).2 := by
  have he : evaluate cfg { st with scan := some s } now =
      evaluate cfg { st with scan := some s' } now := by
    simp [evaluate, sourcePoints, hstamp, hpts]
  rcases output_eq_silent_or_publish cfg { st with scan := some s' } now v with h | h
  · rw [h, output_eq_silent_iff]
    rw [output_eq_silent_iff] at h
    simpa [he] using h
  · rw [h, output_eq_publish_iff]
    rw [output_eq_publish_iff] at h
    simpa [he] using h

/-- **Bad data (a), R4: invalid readings never change the decision.** Removing
from the stored scan every reading that is not valid (NaN, below `range_min`,
above `range_max`, or any reading at all when a limit is NaN) leaves the
output of every command unchanged. -/
theorem output_filter_validBeam (cfg : Config) (st : State) (now : Nat) (v : Twist) (s : Scan) :
    (step cfg { st with scan := some { s with beams := s.beams.filter s.validBeam } }
        (.cmd now v)).2 =
      (step cfg { st with scan := some s } (.cmd now v)).2 := by
  apply output_congr_scan
  · rfl
  · have hv : Scan.validBeam { s with beams := s.beams.filter s.validBeam } = s.validBeam := rfl
    simp [Scan.points, hv]

/-- **Bad data (a), R4.** A reading that is not valid contributes nothing: the
scan has the same points with it as without it, whatever the point it
carries. So it changes no zone count and no decision. -/
theorem points_of_not_validBeam (stamp rangeMin rangeMax : Nat) (bs₁ bs₂ : List Beam) (b : Beam)
    (h : (Scan.mk stamp rangeMin rangeMax (bs₁ ++ b :: bs₂)).validBeam b = false) :
    (Scan.mk stamp rangeMin rangeMax (bs₁ ++ b :: bs₂)).points =
      (Scan.mk stamp rangeMin rangeMax (bs₁ ++ bs₂)).points := by
  have hv : ∀ bs, Scan.validBeam ⟨stamp, rangeMin, rangeMax, bs⟩ =
      fun b => F32.le rangeMin b.range && F32.le b.range rangeMax := fun _ => rfl
  rw [hv] at h
  simp [Scan.points, hv, h]

/-- A scan without valid readings is a scan without obstacles (R4): it has no
points. -/
theorem points_eq_nil_iff (s : Scan) : s.points = [] ↔ ∀ b ∈ s.beams, s.validBeam b = false := by
  simp [Scan.points, List.filter_eq_nil_iff]

/-- **No amplification, on the demanded action.** A demanded slowdown
carries a ratio pattern below that of 1.0, and it is the configured ratio of
the zone it names, which is triggered. -/
theorem evaluate_slowdown (cfg : Config) (st : State) (now k r : Nat)
    (h : evaluate cfg st now = .slowdown k r) :
    r < F64.one ∧ ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨pts, -, -, z, hz, ha, -, hlt, -⟩ := (evaluate_eq_slowdown_iff cfg st now k r).mp h
  exact ⟨hlt, z, hz, ha⟩

/-- **No amplification.** Whatever is published is `pass`, a stop, or a
slowdown whose ratio pattern `r` is below `F64.one` and is the configured
ratio of zone `k`. By `lt_one_iff`, such a pattern has its sign bit clear and
denotes a number in `[0, 1)`.

This holds for every configuration, accepted or not. What it does not say is
anything about the product the shim computes with that ratio (SPEC, T1). -/
theorem no_amplification (cfg : Config) (st : State) (now : Nat) (v : Twist) (a : Action)
    (h : (step cfg st (.cmd now v)).2 = .publish a) :
    a = .pass ∨ (∃ k, a = .stopZone k) ∨ a = .stopSource ∨
      ∃ k r, a = .slowdown k r ∧ r < F64.one ∧
        ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨-, he, -⟩ := (output_eq_publish_iff cfg st now v a).mp h
  cases a with
  | pass => exact Or.inl rfl
  | stopZone k => exact Or.inr (Or.inl ⟨k, rfl⟩)
  | stopSource => exact Or.inr (Or.inr (Or.inl rfl))
  | slowdown k r =>
    exact Or.inr (Or.inr (Or.inr ⟨k, r, rfl, evaluate_slowdown cfg st now k r he⟩))

/-- A published slowdown ratio, read as a number: sign bit clear and
magnitude below that of 1.0 (in units of 2⁻¹⁰⁷⁴). -/
theorem published_ratio_in_unit_interval (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (k r : Nat) (h : (step cfg st (.cmd now v)).2 = .publish (.slowdown k r)) :
    r < 0x8000000000000000 ∧ magVal 52 r < 2 ^ 1074 := by
  obtain ⟨-, he, -⟩ := (output_eq_publish_iff cfg st now v _).mp h
  exact (lt_one_iff r).mp (evaluate_slowdown cfg st now k r he).1

/-- **Slowdown bound.** If the source is valid with points `pts` and a
slowdown zone with ratio pattern `rz` below that of 1.0 is triggered, the
action demanded is a stop by a zone, or a slowdown whose ratio pattern is at
most `rz`; in the latter case the ratio is the smallest among the triggered
slowdown zones and the zone named is the first that has it. -/
theorem slowdown_bound (cfg : Config) (st : State) (now : Nat) (pts : List Point) (z : Zone)
    (rz : Nat) (hpts : sourcePoints cfg st now = some pts) (hz : z ∈ cfg.zones)
    (ha : z.action = .slowdown rz) (ht : z.triggered pts = true) (hlt : rz < F64.one) :
    (∃ k, evaluate cfg st now = .stopZone k) ∨
      ∃ k r, evaluate cfg st now = .slowdown k r ∧ r ≤ rz ∧ IsBestSlowdown cfg.zones pts k r := by
  rw [evaluate_of_sourcePoints hpts]
  cases hact : zonesAction cfg pts with
  | stopZone k => exact Or.inl ⟨k, rfl⟩
  | stopSource => exact absurd hact (zonesAction_ne_stopSource cfg pts)
  | pass =>
    have := ((zonesAction_eq_pass_iff cfg pts).mp hact).2 z hz rz ha ht
    omega
  | slowdown k r =>
    right
    have hbest := ((zonesAction_eq_slowdown_iff cfg pts k r).mp hact).2
    obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp hz
    obtain ⟨z0, -, -, -, -, hmin⟩ := id hbest
    exact ⟨k, r, rfl, (hmin j z rz hj ha ht).1, hbest⟩

/-- **Slowdown bound, for every zone at once.** If a slowdown by `r` is
demanded, then `r` is at most the ratio pattern of every triggered slowdown
zone. For patterns with the sign bit clear this is the order of the numbers
they denote (`magVal_le_iff`). -/
theorem evaluate_slowdown_le (cfg : Config) (st : State) (now k r : Nat) (pts : List Point)
    (hpts : sourcePoints cfg st now = some pts) (h : evaluate cfg st now = .slowdown k r)
    (z : Zone) (rz : Nat) (hz : z ∈ cfg.zones) (ha : z.action = .slowdown rz)
    (ht : z.triggered pts = true) : r ≤ rz := by
  rw [evaluate_of_sourcePoints hpts, zonesAction_eq_slowdown_iff] at h
  obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp hz
  obtain ⟨z0, -, -, -, -, hmin⟩ := h.2
  exact (hmin j z rz hj ha ht).1

/-- **Pass-through.** If the command is finite, the source is valid with
points `pts`, and no zone is triggered by `pts`, then the action demanded is
`pass`, and the output is `publish pass`, except that a command that is zero
in the published components is dropped once zero has been demanded for more
than `stop_pub_timeout` (R15). -/
theorem pass_through (cfg : Config) (st : State) (now : Nat) (v : Twist) (pts : List Point)
    (hf : v.finite = true) (hpts : sourcePoints cfg st now = some pts)
    (hno : ∀ z ∈ cfg.zones, z.triggered pts = false) :
    evaluate cfg st now = .pass ∧
      (step cfg st (.cmd now v)).2 =
        if v.planarZero = true ∧ st.prevZero = true ∧ st.stopStamp + cfg.stopPubTimeout < now
        then .silent else .publish .pass := by
  have he : evaluate cfg st now = .pass := by
    rw [evaluate_eq_pass_iff]
    refine ⟨pts, hpts, fun z hz _ => hno z hz, fun z hz r _ ht => ?_⟩
    rw [hno z hz] at ht
    exact absurd ht Bool.false_ne_true
  refine ⟨he, ?_⟩
  split
  · rename_i h
    rw [output_eq_silent_iff, he]
    exact Or.inr h
  · rename_i h
    rw [output_eq_publish_iff]
    exact ⟨hf, he, h⟩

/-- **Pass-through for a non-zero command.** A finite command that is not
zero in the published components, with a valid source and no zone
triggered, is always published unchanged. The node is not the one that
always stops. -/
theorem pass_through_of_nonzero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (pts : List Point) (hf : v.finite = true) (hnz : v.planarZero = false)
    (hpts : sourcePoints cfg st now = some pts)
    (hno : ∀ z ∈ cfg.zones, z.triggered pts = false) :
    (step cfg st (.cmd now v)).2 = .publish .pass := by
  rw [(pass_through cfg st now v pts hf hpts hno).2]
  simp [hnz]

/-! ## Monotonicity: more points never relax the action -/

/-- A zone triggered by some points is triggered by any list that contains
them. -/
theorem triggered_mono (z : Zone) {pts pts' : List Point} (h : pts.Sublist pts')
    (ht : z.triggered pts = true) : z.triggered pts' = true := by
  rw [triggered_iff] at ht ⊢
  exact ⟨ht.1, le_trans ht.2 (h.countP_le)⟩

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

/-- Adding readings to a scan, with the same range limits, only adds
points. -/
theorem points_sublist (stamp stamp' rangeMin rangeMax : Nat) {bs bs' : List Beam}
    (h : bs.Sublist bs') :
    (Scan.mk stamp rangeMin rangeMax bs).points.Sublist
      (Scan.mk stamp' rangeMin rangeMax bs').points := by
  have hv : Scan.validBeam ⟨stamp, rangeMin, rangeMax, bs⟩ =
      Scan.validBeam ⟨stamp', rangeMin, rangeMax, bs'⟩ := rfl
  simp only [Scan.points, hv]
  exact (h.filter _).map _

/-- **More readings never lift a stop (one command).** If zero is demanded
with scan `s` stored, it is demanded with any scan of the same stamp and
range limits that has the readings of `s` and more. -/
theorem evaluate_isStop_mono (cfg : Config) (st : State) (now : Nat)
    (stamp rangeMin rangeMax : Nat) {bs bs' : List Beam} (h : bs.Sublist bs')
    (hs : (evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs⟩ } now).isStop
      = true) :
    (evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs'⟩ } now).isStop = true := by
  have hsub := points_sublist stamp stamp rangeMin rangeMax h
  have key : ∀ bs, evaluate cfg { st with scan := some ⟨stamp, rangeMin, rangeMax, bs⟩ } now =
      if cfg.sourceEnabled = false then zonesAction cfg []
      else if 0 < cfg.sourceTimeout ∧ stamp + cfg.sourceTimeout < now then .stopSource
      else zonesAction cfg (Scan.points ⟨stamp, rangeMin, rangeMax, bs⟩) := by
    intro bs
    simp only [evaluate, sourcePoints]
    cases cfg.sourceEnabled
    · simp
    · by_cases ht : 0 < cfg.sourceTimeout ∧ stamp + cfg.sourceTimeout < now <;> simp [ht]
  rw [key] at hs ⊢
  split_ifs at hs ⊢
  · exact hs
  · rfl
  · cases hact : zonesAction cfg (Scan.points ⟨stamp, rangeMin, rangeMax, bs⟩) with
    | stopZone k =>
      obtain ⟨k', -, hk'⟩ := zonesAction_stop_mono cfg hsub k hact
      simp [hk', Action.isStop]
    | stopSource => exact absurd hact (zonesAction_ne_stopSource cfg _)
    | pass => simp [hact, Action.isStop] at hs
    | slowdown k r => simp [hact, Action.isStop] at hs

end ROSlean.CollisionMonitor
