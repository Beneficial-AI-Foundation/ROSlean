import Mathlib.Tactic
import ROSleanCore.CollisionMonitor

/-!
# Collision monitor: the selection of the action

Characterisations of the parts `evaluate` is built from
(`ROSleanCore.CollisionMonitor`): which points are tested (`sourcePoints`),
which stop zone is named (`firstStop`), which slowdown zone wins
(`bestSlowdown`), and from these the action for given points (`zonesAction`).

The specifications are stated as predicates on the zone list that do not
mention the loops of the model:

* `IsFirstStop zones pts k`: zone `k` is a triggered stop zone and no stop
  zone before it is triggered;
* `IsBestSlowdown zones pts k r`: zone `k` is a triggered slowdown zone with
  ratio pattern `r < F64.one`, no triggered slowdown zone has a smaller
  ratio, and none before `k` has the same ratio.

A zone list is always that of some configuration. For a command it is the
configuration with the shapes of the topic zones filled in
(`Config.withShapes`, `liveConfig`); the lemmas at the end of this file say
what that configuration is, zone by zone.

Rule numbers refer to `docs/collision_monitor/SPEC.md` (`R…`) and
`docs/collision_monitor/SPEC_NAV2.md` (`N…`).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## Reading the zone definitions -/

theorem isStop_iff (z : Zone) : z.isStop = true ↔ z.action = .stop := by
  unfold Zone.isStop
  cases z.action <;> simp

theorem isStop_eq_false_iff (z : Zone) : z.isStop = false ↔ ∃ r, z.action = .slowdown r := by
  unfold Zone.isStop
  cases z.action <;> simp

/-- R11, N32: a zone triggers iff it is enabled, has a shape, and at least
`minPoints` of the points are inside its shape. -/
theorem triggered_iff (z : Zone) (pts : List Point) :
    z.triggered pts = true ↔
      z.enabled = true ∧ z.shape.isSet = true ∧ z.minPoints ≤ pts.countP z.shape.contains := by
  unfold Zone.triggered Zone.count
  rw [Bool.and_eq_true, Bool.and_eq_true, decide_eq_true_iff, and_assoc]

/-- R11: a disabled zone never triggers. -/
theorem not_triggered_of_disabled (z : Zone) (pts : List Point) (h : z.enabled = false) :
    z.triggered pts = false := by
  simp [Zone.triggered, h]

/-- N32: a zone without shape never triggers, whatever `minPoints` is. -/
theorem not_triggered_of_topic (z : Zone) (pts : List Point) (h : z.shape = .topic) :
    z.triggered pts = false := by
  simp [Zone.triggered, h, ZoneShape.isSet]

/-- R11: an enabled zone that has a shape and `minPoints = 0` triggers on any
points, also on none. -/
theorem triggered_of_minPoints_zero (z : Zone) (pts : List Point) (he : z.enabled = true)
    (hs : z.shape.isSet = true) (h0 : z.minPoints = 0) : z.triggered pts = true := by
  simp [Zone.triggered, he, hs, h0]

/-- Without points, only a zone with `minPoints = 0` triggers. -/
theorem triggered_nil_iff (z : Zone) :
    z.triggered [] = true ↔ z.enabled = true ∧ z.shape.isSet = true ∧ z.minPoints = 0 := by
  simp [Zone.triggered, Zone.count, and_assoc]

/-! ## The points that are tested (R5 to R8, N22, N27) -/

/-- The command carries no transform for the scan that can be applied: the
lookup failed or was not made, or (never, with tf2) its result is not usable
(ND7). -/
def NoTransform (scanTf : Option Xform) : Prop := ∀ t, scanTf = some t → t.usable = false

theorem noTransform_none : NoTransform none := fun _ h => by cases h

theorem noTransform_some_iff (t : Xform) : NoTransform (some t) ↔ t.usable = false := by
  simp [NoTransform]

theorem not_noTransform_iff (scanTf : Option Xform) :
    ¬ NoTransform scanTf ↔ ∃ t, scanTf = some t ∧ t.usable = true := by
  cases scanTf with
  | none => simp [NoTransform]
  | some t => simp [NoTransform]

/-- **The source is invalid** (R8, N27): it is enabled, its timeout is not
zero, and no scan is stored, or the stored scan's stamp is more than the
timeout before `now`, or the command carries no usable transform for it. -/
def SourceInvalid (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform) : Prop :=
  cfg.sourceEnabled = true ∧ 0 < cfg.sourceTimeout ∧
    (st.scan = none ∨ (∃ s, st.scan = some s ∧ s.stamp + cfg.sourceTimeout < now) ∨
      NoTransform scanTf)

/-- R8, N27: the robot must stop for its source iff the source is invalid. -/
theorem sourcePoints_eq_none_iff (cfg : Config) (st : State) (now : Nat)
    (scanTf : Option Xform) :
    sourcePoints cfg st now scanTf = none ↔ SourceInvalid cfg st now scanTf := by
  unfold sourcePoints SourceInvalid
  cases he : cfg.sourceEnabled <;> cases hs : st.scan with
  | none => simp
  | some s =>
    by_cases hstale : 0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now
    · simp [hstale.1, hstale.2]
    · cases scanTf with
      | none =>
        by_cases h0 : cfg.sourceTimeout = 0
        · simp [h0, noTransform_none]
        · have hpos : 0 < cfg.sourceTimeout := by omega
          simp only [not_and] at hstale
          simp [h0, hpos, hstale hpos, noTransform_none]
      | some t =>
        by_cases h0 : cfg.sourceTimeout = 0
        · cases hu : t.usable <;> simp [h0, hu]
        · have hpos : 0 < cfg.sourceTimeout := by omega
          simp only [not_and] at hstale
          cases hu : t.usable <;> simp [h0, hpos, hstale hpos, hu, noTransform_some_iff]

/-- R5 to R8, N22, N27: the zones are tested against `pts` iff one of

* the source is disabled and `pts = []` (R6);
* the timeout is 0, no scan is stored and `pts = []` (R8);
* a scan is stored and is not too old (R7), and either the command carries a
  usable transform `t` and `pts` are the scan's valid points under `t`, or it
  carries none, the timeout is 0 and `pts = []` (N27). -/
theorem sourcePoints_eq_some_iff (cfg : Config) (st : State) (now : Nat)
    (scanTf : Option Xform) (pts : List Point) :
    sourcePoints cfg st now scanTf = some pts ↔
      (cfg.sourceEnabled = false ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ st.scan = none ∧ cfg.sourceTimeout = 0 ∧ pts = []) ∨
      (cfg.sourceEnabled = true ∧ ∃ s, st.scan = some s ∧
        ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now) ∧
        ((∃ t, scanTf = some t ∧ t.usable = true ∧ pts = s.points t) ∨
          (NoTransform scanTf ∧ cfg.sourceTimeout = 0 ∧ pts = []))) := by
  unfold sourcePoints
  cases he : cfg.sourceEnabled
  · simp [eq_comm (b := pts)]
  cases hs : st.scan with
  | none => simp [eq_comm (b := pts)]
  | some s =>
    by_cases hstale : 0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now
    · simp [hstale.1, hstale.2]
    · have hst : (decide (0 < cfg.sourceTimeout) && decide (s.stamp + cfg.sourceTimeout < now))
          = false := by simpa using hstale
      have hfresh : 0 < cfg.sourceTimeout → now ≤ s.stamp + cfg.sourceTimeout := by
        intro hp; omega
      cases scanTf with
      | none =>
        by_cases h0 : cfg.sourceTimeout = 0
        · simp [h0, noTransform_none, eq_comm (b := pts)]
        · simp [h0, hst, noTransform_none]
      | some t =>
        by_cases h0 : cfg.sourceTimeout = 0
        · cases hu : t.usable <;> simp [h0, hu, noTransform_some_iff, eq_comm (b := pts)]
        · cases hu : t.usable
          · simp [h0, hst, hu, noTransform_some_iff]
          · simpa [h0, hst, hu, noTransform_some_iff, eq_comm (b := pts)] using
              fun (_ : pts = s.points t) => hfresh

/-- `sourcePoints` reads the state only through the stored scan. -/
theorem sourcePoints_congr (cfg : Config) (st st' : State) (now : Nat) (scanTf : Option Xform)
    (h : st.scan = st'.scan) :
    sourcePoints cfg st now scanTf = sourcePoints cfg st' now scanTf := by
  simp [sourcePoints, h]

/-! ## The first triggered stop zone (R12) -/

/-- Zone `k` is a triggered stop zone, and no stop zone before it is
triggered. -/
def IsFirstStop (zones : List Zone) (pts : List Point) (k : Nat) : Prop :=
  ∃ z, zones[k]? = some z ∧ z.action = .stop ∧ z.triggered pts = true ∧
    ∀ j z', j < k → zones[j]? = some z' → z'.action = .stop → z'.triggered pts = false

/-- No stop zone is triggered. -/
def NoStop (zones : List Zone) (pts : List Point) : Prop :=
  ∀ z ∈ zones, z.action = .stop → z.triggered pts = false

theorem IsFirstStop.unique {zones : List Zone} {pts : List Point} {k k' : Nat}
    (h : IsFirstStop zones pts k) (h' : IsFirstStop zones pts k') : k = k' := by
  obtain ⟨z, hz, hs, ht, hmin⟩ := h
  obtain ⟨z', hz', hs', ht', hmin'⟩ := h'
  rcases lt_trichotomy k k' with hlt | heq | hgt
  · have := hmin' k z hlt hz hs
    simp [ht] at this
  · exact heq
  · have := hmin k' z' hgt hz' hs'
    simp [ht'] at this

private theorem find?_zipIdx_map {α : Type} (p : α → Bool) (l : List α) (n : Nat) :
    ((l.zipIdx n).find? (fun zi => p zi.1)).map (·.2) = (l.findIdx? p).map (· + n) := by
  induction l generalizing n with
  | nil => simp
  | cons a l ih =>
    rw [List.zipIdx_cons, List.find?_cons, List.findIdx?_cons]
    by_cases ha : p a = true
    · simp [ha]
    · simp only [ha, Bool.false_eq_true, ↓reduceIte, ih, Option.map_map]
      congr 1
      funext i
      simp only [Function.comp_apply]
      omega

theorem firstStop_eq_findIdx? (cfg : Config) (pts : List Point) :
    firstStop cfg pts = cfg.zones.findIdx? (fun z => z.isStop && z.triggered pts) := by
  unfold firstStop
  have := find?_zipIdx_map (fun z => z.isStop && z.triggered pts) cfg.zones 0
  simpa using this

/-- R12: `firstStop` returns the index of the first triggered stop zone. -/
theorem firstStop_eq_some_iff (cfg : Config) (pts : List Point) (k : Nat) :
    firstStop cfg pts = some k ↔ IsFirstStop cfg.zones pts k := by
  rw [firstStop_eq_findIdx?, List.findIdx?_eq_some_iff_getElem]
  constructor
  · rintro ⟨hk, hp, hmin⟩
    simp only [Bool.and_eq_true, isStop_iff] at hp
    refine ⟨cfg.zones[k], by simp, hp.1, hp.2, ?_⟩
    intro j z' hj hz' hs'
    obtain ⟨hj', rfl⟩ := List.getElem?_eq_some_iff.mp hz'
    have := hmin j hj
    simp only [Bool.and_eq_true, isStop_iff, not_and, Bool.not_eq_true] at this
    exact this hs'
  · rintro ⟨z, hz, hs, ht, hmin⟩
    obtain ⟨hk, rfl⟩ := List.getElem?_eq_some_iff.mp hz
    refine ⟨hk, by simp [isStop_iff, hs, ht], ?_⟩
    intro j hj
    simp only [Bool.and_eq_true, isStop_iff, not_and, Bool.not_eq_true]
    exact hmin j _ hj (by simp [Nat.lt_trans hj hk])

/-- R12: `firstStop` returns nothing iff no stop zone is triggered. -/
theorem firstStop_eq_none_iff (cfg : Config) (pts : List Point) :
    firstStop cfg pts = none ↔ NoStop cfg.zones pts := by
  rw [firstStop_eq_findIdx?, List.findIdx?_eq_none_iff]
  simp [NoStop, isStop_iff]

/-- If some stop zone is triggered, there is a first one. -/
theorem exists_isFirstStop {zones : List Zone} {pts : List Point} {z : Zone} (hz : z ∈ zones)
    (hs : z.action = .stop) (ht : z.triggered pts = true) : ∃ k, IsFirstStop zones pts k := by
  cases h : firstStop ⟨zones, true, 0, 0⟩ pts with
  | none =>
    have := (firstStop_eq_none_iff _ _).mp h z hz hs
    simp [ht] at this
  | some k => exact ⟨k, (firstStop_eq_some_iff ⟨zones, true, 0, 0⟩ pts k).mp h⟩

/-! ## The slowdown zone that wins (R13) -/

/-- Zone `k` is a triggered slowdown zone whose ratio pattern `r` is below
that of 1.0; no triggered slowdown zone has a smaller ratio pattern, and none
before `k` has the same. -/
def IsBestSlowdown (zones : List Zone) (pts : List Point) (k r : Nat) : Prop :=
  ∃ z, zones[k]? = some z ∧ z.action = .slowdown r ∧ z.triggered pts = true ∧ r < F64.one ∧
    ∀ j z' r', zones[j]? = some z' → z'.action = .slowdown r' → z'.triggered pts = true →
      r ≤ r' ∧ (j < k → r < r')

/-- No slowdown zone with a ratio pattern below that of 1.0 is triggered. -/
def NoSlowdown (zones : List Zone) (pts : List Point) : Prop :=
  ∀ z ∈ zones, ∀ r, z.action = .slowdown r → z.triggered pts = true → F64.one ≤ r

theorem IsBestSlowdown.unique {zones : List Zone} {pts : List Point} {k k' r r' : Nat}
    (h : IsBestSlowdown zones pts k r) (h' : IsBestSlowdown zones pts k' r') :
    k = k' ∧ r = r' := by
  obtain ⟨z, hz, ha, ht, -, hmin⟩ := h
  obtain ⟨z', hz', ha', ht', -, hmin'⟩ := h'
  have h1 := hmin k' z' r' hz' ha' ht'
  have h2 := hmin' k z r hz ha ht
  have hr : r = r' := le_antisymm h1.1 h2.1
  refine ⟨?_, hr⟩
  rcases lt_trichotomy k k' with hlt | heq | hgt
  · have := h2.2 hlt
    omega
  · exact heq
  · have := h1.2 hgt
    omega

theorem IsBestSlowdown.not_noSlowdown {zones : List Zone} {pts : List Point} {k r : Nat}
    (h : IsBestSlowdown zones pts k r) : ¬ NoSlowdown zones pts := by
  obtain ⟨z, hz, ha, ht, hlt, -⟩ := h
  intro hn
  have := hn z (List.mem_of_getElem? hz) r ha ht
  omega

/-- The selection loop over a zone list, as in `bestSlowdown`. -/
private def bestOf (zones : List Zone) (pts : List Point) : Nat × Option Nat :=
  zones.zipIdx.foldl (slowdownStep pts) (F64.one, none)

private theorem bestOf_spec (zones : List Zone) (pts : List Point) :
    (bestOf zones pts = (F64.one, none) ∧ NoSlowdown zones pts) ∨
    ∃ k r, bestOf zones pts = (r, some k) ∧ IsBestSlowdown zones pts k r := by
  induction zones using List.reverseRecOn with
  | nil => left; simp [bestOf, NoSlowdown]
  | append_singleton zs z ih =>
    have hfold : bestOf (zs ++ [z]) pts = slowdownStep pts (bestOf zs pts) (z, zs.length) := by
      simp [bestOf, List.zipIdx_append, List.foldl_append]
    rw [hfold]
    -- what the new zone contributes
    cases haz : z.action with
    | stop =>
      have hstep : ∀ b, slowdownStep pts b (z, zs.length) = b := by
        intro b; simp [slowdownStep, haz]
      rw [hstep]
      rcases ih with ⟨hb, hn⟩ | ⟨k, r, hb, z0, hz0, ha0, ht0, hlt0, hmin⟩
      · left
        refine ⟨hb, ?_⟩
        intro z' hz' r' ha' ht'
        rcases List.mem_append.mp hz' with h | h
        · exact hn z' h r' ha' ht'
        · simp only [List.mem_singleton] at h
          subst h
          simp [haz] at ha'
      · right
        refine ⟨k, r, hb, z0, ?_, ha0, ht0, hlt0, ?_⟩
        · have hk := (List.getElem?_eq_some_iff.mp hz0).1
          simp [List.getElem?_append_left hk, hz0]
        · intro j z' r' hz' ha' ht'
          by_cases hj : j < zs.length
          · rw [List.getElem?_append_left hj] at hz'
            exact hmin j z' r' hz' ha' ht'
          · have hj' := (List.getElem?_eq_some_iff.mp hz').1
            simp only [List.length_append, List.length_singleton] at hj'
            have : j = zs.length := by omega
            subst this
            simp only [List.getElem?_concat_length, Option.some.injEq] at hz'
            subst hz'
            simp [haz] at ha'
    | slowdown rz =>
      by_cases hsel : z.triggered pts = true ∧ rz < (bestOf zs pts).1
      · -- the new zone replaces the best so far
        have hstep : slowdownStep pts (bestOf zs pts) (z, zs.length) = (rz, some zs.length) := by
          simp [slowdownStep, haz, hsel.1, hsel.2]
        rw [hstep]
        right
        refine ⟨zs.length, rz, rfl, z, by simp, haz, hsel.1, ?_, ?_⟩
        · rcases ih with ⟨hb, -⟩ | ⟨k, r, hb, z0, -, -, -, hlt0, -⟩
          · have := hsel.2; rw [hb] at this; exact this
          · have := hsel.2; rw [hb] at this; exact lt_trans this hlt0
        · intro j z' r' hz' ha' ht'
          by_cases hj : j < zs.length
          · rw [List.getElem?_append_left hj] at hz'
            rcases ih with ⟨hb, hn⟩ | ⟨k, r, hb, z0, -, -, -, -, hmin⟩
            · have h1 := hn z' (List.mem_of_getElem? hz') r' ha' ht'
              have := hsel.2; rw [hb] at this
              simp only at this
              omega
            · have h1 := (hmin j z' r' hz' ha' ht').1
              have := hsel.2; rw [hb] at this
              simp only at this
              omega
          · have hj' := (List.getElem?_eq_some_iff.mp hz').1
            simp only [List.length_append, List.length_singleton] at hj'
            have : j = zs.length := by omega
            subst this
            simp only [List.getElem?_concat_length, Option.some.injEq] at hz'
            subst hz'
            rw [haz] at ha'
            cases ha'
            simp
      · -- the best so far is kept
        have hstep : slowdownStep pts (bestOf zs pts) (z, zs.length) = bestOf zs pts := by
          simp only [slowdownStep, haz]
          split
          · rename_i h
            exact absurd (by simpa using h) hsel
          · rfl
        rw [hstep]
        rcases ih with ⟨hb, hn⟩ | ⟨k, r, hb, z0, hz0, ha0, ht0, hlt0, hmin⟩
        · left
          refine ⟨hb, ?_⟩
          intro z' hz' r' ha' ht'
          rcases List.mem_append.mp hz' with h | h
          · exact hn z' h r' ha' ht'
          · simp only [List.mem_singleton] at h
            subst h
            rw [haz] at ha'
            cases ha'
            rw [hb] at hsel
            simp only [not_and, not_lt] at hsel
            exact hsel ht'
        · right
          have hk := (List.getElem?_eq_some_iff.mp hz0).1
          refine ⟨k, r, hb, z0, ?_, ha0, ht0, hlt0, ?_⟩
          · simp [List.getElem?_append_left hk, hz0]
          · intro j z' r' hz' ha' ht'
            by_cases hj : j < zs.length
            · rw [List.getElem?_append_left hj] at hz'
              exact hmin j z' r' hz' ha' ht'
            · have hj' := (List.getElem?_eq_some_iff.mp hz').1
              simp only [List.length_append, List.length_singleton] at hj'
              have : j = zs.length := by omega
              subst this
              simp only [List.getElem?_concat_length, Option.some.injEq] at hz'
              subst hz'
              rw [haz] at ha'
              cases ha'
              rw [hb] at hsel
              simp only [not_and, not_lt] at hsel
              exact ⟨hsel ht', fun h => by omega⟩

/-- `bestSlowdown` either selects nothing and returns the ratio 1, with no
slowdown zone below 1 triggered, or selects the best slowdown zone. -/
theorem bestSlowdown_spec (cfg : Config) (pts : List Point) :
    (bestSlowdown cfg pts = (F64.one, none) ∧ NoSlowdown cfg.zones pts) ∨
    ∃ k r, bestSlowdown cfg pts = (r, some k) ∧ IsBestSlowdown cfg.zones pts k r :=
  bestOf_spec cfg.zones pts

/-- R13: `bestSlowdown` selects zone `k` with ratio `r` iff `k` is the first
triggered slowdown zone with the smallest ratio pattern, and that is below
the pattern of 1.0. -/
theorem bestSlowdown_eq_some_iff (cfg : Config) (pts : List Point) (k r : Nat) :
    bestSlowdown cfg pts = (r, some k) ↔ IsBestSlowdown cfg.zones pts k r := by
  rcases bestSlowdown_spec cfg pts with ⟨hb, hn⟩ | ⟨k', r', hb, hbest⟩
  · rw [hb]
    constructor
    · intro h; simp at h
    · intro h; exact absurd hn h.not_noSlowdown
  · rw [hb]
    constructor
    · intro h
      simp only [Prod.mk.injEq, Option.some.injEq] at h
      rw [← h.1, ← h.2]; exact hbest
    · intro h
      obtain ⟨rfl, rfl⟩ := hbest.unique h
      rfl

/-- R13: `bestSlowdown` selects nothing iff no slowdown zone with a ratio
below 1 is triggered. -/
theorem bestSlowdown_snd_eq_none_iff (cfg : Config) (pts : List Point) :
    (bestSlowdown cfg pts).2 = none ↔ NoSlowdown cfg.zones pts := by
  rcases bestSlowdown_spec cfg pts with ⟨hb, hn⟩ | ⟨k', r', hb, hbest⟩
  · simp [hb, hn]
  · simp only [hb, reduceCtorEq, false_iff]
    exact hbest.not_noSlowdown

/-- If a slowdown zone with ratio below 1 is triggered, there is a best one,
and its ratio is at most that zone's. -/
theorem exists_isBestSlowdown {zones : List Zone} {pts : List Point} {z : Zone} {rz : Nat}
    (hz : z ∈ zones) (ha : z.action = .slowdown rz) (ht : z.triggered pts = true)
    (hlt : rz < F64.one) : ∃ k r, IsBestSlowdown zones pts k r ∧ r ≤ rz := by
  rcases bestSlowdown_spec ⟨zones, true, 0, 0⟩ pts with ⟨-, hn⟩ | ⟨k, r, -, hbest⟩
  · have := hn z hz rz ha ht
    omega
  · refine ⟨k, r, hbest, ?_⟩
    obtain ⟨j, hj⟩ := List.mem_iff_getElem?.mp hz
    obtain ⟨z0, -, -, -, -, hmin⟩ := hbest
    exact (hmin j z rz hj ha ht).1

/-! ## The action for given points (R12 to R14) -/

/-- R12: the zones demand a stop named after zone `k` iff `k` is the first
triggered stop zone. -/
theorem zonesAction_eq_stopZone_iff (cfg : Config) (pts : List Point) (k : Nat) :
    zonesAction cfg pts = .stopZone k ↔ IsFirstStop cfg.zones pts k := by
  rw [← firstStop_eq_some_iff]
  unfold zonesAction
  cases h : firstStop cfg pts with
  | some k' => simp
  | none =>
    rcases hb : bestSlowdown cfg pts with ⟨r, _ | k'⟩ <;> simp

/-- R12, R13: the zones demand a slowdown by `r`, named after zone `k`, iff no
stop zone is triggered and `k` is the best slowdown zone, with ratio `r`. -/
theorem zonesAction_eq_slowdown_iff (cfg : Config) (pts : List Point) (k r : Nat) :
    zonesAction cfg pts = .slowdown k r ↔
      NoStop cfg.zones pts ∧ IsBestSlowdown cfg.zones pts k r := by
  rw [← firstStop_eq_none_iff, ← bestSlowdown_eq_some_iff]
  unfold zonesAction
  cases h : firstStop cfg pts with
  | some k' => simp
  | none =>
    rcases hb : bestSlowdown cfg pts with ⟨r', _ | k'⟩ <;> simp [and_comm]

/-- R14: the zones leave the command alone iff no stop zone is triggered and
no slowdown zone with a ratio below 1 is triggered. -/
theorem zonesAction_eq_pass_iff (cfg : Config) (pts : List Point) :
    zonesAction cfg pts = .pass ↔ NoStop cfg.zones pts ∧ NoSlowdown cfg.zones pts := by
  rw [← firstStop_eq_none_iff, ← bestSlowdown_snd_eq_none_iff]
  unfold zonesAction
  cases h : firstStop cfg pts with
  | some k' => simp
  | none =>
    rcases hb : bestSlowdown cfg pts with ⟨r', _ | k'⟩ <;> simp

/-- `stopSource` is never the answer of the zones; it is reserved for an
invalid source (R8). -/
theorem zonesAction_ne_stopSource (cfg : Config) (pts : List Point) :
    zonesAction cfg pts ≠ .stopSource := by
  unfold zonesAction
  cases h : firstStop cfg pts with
  | some k' => simp
  | none =>
    rcases hb : bestSlowdown cfg pts with ⟨r', _ | k'⟩ <;> simp

/-- The action depends on the zones and the points only through the action of
each zone, in order, and whether it is triggered. -/
theorem zonesAction_eq_of_map (cfg cfg' : Config) (pts pts' : List Point)
    (h : cfg.zones.map (fun z => (z.action, z.triggered pts)) =
      cfg'.zones.map (fun z => (z.action, z.triggered pts'))) :
    zonesAction cfg pts = zonesAction cfg' pts' := by
  have hfirst : firstStop cfg pts = firstStop cfg' pts' := by
    rw [firstStop_eq_findIdx?, firstStop_eq_findIdx?]
    have key : ∀ (zs : List Zone) (q : List Point),
        zs.findIdx? (fun z => z.isStop && z.triggered q) =
          (zs.map (fun z => (z.action, z.triggered q))).findIdx?
            (fun a => (match a.1 with | .stop => true | .slowdown _ => false) && a.2) := by
      intro zs q
      rw [List.findIdx?_map]
      rfl
    rw [key, key, h]
  have hbest : bestSlowdown cfg pts = bestSlowdown cfg' pts' := by
    have key : ∀ (zs : List Zone) (q : List Point),
        zs.zipIdx.foldl (slowdownStep q) (F64.one, none) =
          (zs.map (fun z => (z.action, z.triggered q))).zipIdx.foldl
            (fun (best : Nat × Option Nat) (ai : (ZoneAction × Bool) × Nat) =>
              match ai.1.1 with
              | .stop => best
              | .slowdown r => if ai.1.2 && decide (r < best.1) then (r, some ai.2) else best)
            (F64.one, none) := by
      intro zs q
      rw [List.zipIdx_map, List.foldl_map]
      rfl
    unfold bestSlowdown
    rw [key, key, h]
  unfold zonesAction
  rw [hfirst, hbest]

/-- The action depends on the points only through which zones they trigger. -/
theorem zonesAction_congr (cfg : Config) (pts pts' : List Point)
    (h : ∀ z ∈ cfg.zones, z.triggered pts = z.triggered pts') :
    zonesAction cfg pts = zonesAction cfg pts' := by
  apply zonesAction_eq_of_map
  apply List.map_congr_left
  intro z hz
  rw [h z hz]

/-! ## The zones with their shapes filled in (N32, N34) -/

/-- Filling in a shape changes nothing but the shape. -/
theorem withShape_fields (z : Zone) (sh : Option TopicShape) :
    (z.withShape sh).action = z.action ∧ (z.withShape sh).minPoints = z.minPoints ∧
      (z.withShape sh).enabled = z.enabled := by
  unfold Zone.withShape
  split <;> exact ⟨rfl, rfl, rfl⟩

/-- A fixed zone is not affected by the state. -/
theorem withShape_of_fixed (z : Zone) (sh : Option TopicShape) (s : Shape)
    (h : z.shape = .fixed s) : z.withShape sh = z := by
  unfold Zone.withShape
  split
  · rename_i h'; rw [h] at h'; cases h'
  · rfl

/-- N32: a topic zone without shape stays without one. -/
theorem withShape_none (z : Zone) : z.withShape none = z := by
  unfold Zone.withShape
  split
  · rename_i h; cases h
  · rfl

/-- A topic zone with an accepted shape is the polygon of its current
vertices. -/
theorem withShape_of_topic (z : Zone) (sh : TopicShape) (h : z.shape = .topic) :
    z.withShape (some sh) = { z with shape := .fixed (.polygon sh.current) } := by
  simp [Zone.withShape, h]

theorem withShapes_length (cfg : Config) (shapes : List (Option TopicShape)) :
    (cfg.withShapes shapes).zones.length = cfg.zones.length := by
  simp [Config.withShapes]

/-- Zone `k` of the configuration with shapes is zone `k` with entry `k` of
the shapes. -/
theorem withShapes_getElem? (cfg : Config) (shapes : List (Option TopicShape)) (k : Nat) :
    (cfg.withShapes shapes).zones[k]? =
      cfg.zones[k]?.map (fun z => z.withShape (shapes.getD k none)) := by
  simp only [Config.withShapes, List.getElem?_map, List.getElem?_zipIdx]
  cases cfg.zones[k]? <;> simp

theorem withShapes_fields (cfg : Config) (shapes : List (Option TopicShape)) :
    (cfg.withShapes shapes).sourceEnabled = cfg.sourceEnabled ∧
      (cfg.withShapes shapes).sourceTimeout = cfg.sourceTimeout ∧
      (cfg.withShapes shapes).stopPubTimeout = cfg.stopPubTimeout := ⟨rfl, rfl, rfl⟩

/-- A configured zone with its action, as it stands in the configuration with
shapes. -/
theorem withShapes_action (cfg : Config) (shapes : List (Option TopicShape)) (k : Nat) (z : Zone)
    (h : (cfg.withShapes shapes).zones[k]? = some z) :
    ∃ z0, cfg.zones[k]? = some z0 ∧ z = z0.withShape (shapes.getD k none) ∧
      z.action = z0.action ∧ z.minPoints = z0.minPoints ∧ z.enabled = z0.enabled := by
  rw [withShapes_getElem?] at h
  cases h0 : cfg.zones[k]? with
  | none => simp [h0] at h
  | some z0 =>
    simp only [h0, Option.map_some, Option.some.injEq] at h
    subst h
    exact ⟨z0, rfl, rfl, (withShape_fields z0 _).1, (withShape_fields z0 _).2.1,
      (withShape_fields z0 _).2.2⟩

end ROSlean.CollisionMonitor
