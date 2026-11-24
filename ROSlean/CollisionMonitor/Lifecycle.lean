import ROSlean.CollisionMonitor.Properties
import ROSlean.CollisionMonitor.Geometry

/-!
# Collision monitor: lifecycle, shape messages, invariants

The properties of `docs/collision_monitor/SPEC_NAV2.md`, section 4, that
concern events other than a processed command, as theorems about one `step`
from an arbitrary state. `ROSlean.CollisionMonitor.Trace` lifts them to runs.

* **Processed, Inactive** (`processed_iff`, `output_of_not_active`,
  `step_of_finalized`): only a finite command in the active phase has an
  output other than `silent`; after `shutdown` nothing changes any more.
* **The lifecycle** (`step_phase`, `nextPhase`): the phase after an event is
  a function of the phase before it and the event (E1–E5); what each
  transition does to the rest of the state (`step_configure` …).
* **Shape accepted** (`acceptShape_eq_some_iff`, `step_shape_of_accepted`,
  `step_shape_of_refused`, `step_shape_changes`): when a polygon message is
  taken, and that it replaces the shape of its zone and nothing else.
* **No shape** (`not_triggered_of_no_shape`, `evaluate_of_no_shape`,
  `output_of_no_shape`): a topic zone without shape is never triggered and
  never named. This is upstream's behaviour (N32) and it is not fail-safe: a
  stop zone without shape does not stop.
* **Replacing a shape affects that zone only**
  (`withShapes_getElem?_congr`, `zonesAction_withShapes_congr`).
* **Lifecycle invariants** (`Inv`, `inv_init`, `inv_step`).
* **After activation** (`after_activation`).
* **Saturated points and topic zones** (`liveConfig_bounded`).
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## Which events are answered -/

/-- Only a command is answered: lifecycle transitions, scans and shape
messages publish nothing (R1, N5, N6). -/
theorem output_of_not_cmd (cfg : Config) (st : State) (e : Event)
    (h : ∀ now v scanPts zoneVerts, e ≠ .cmd now v scanPts zoneVerts) :
    (step cfg st e).2 = .silent := by
  cases e with
  | cmd now v scanPts zoneVerts => exact absurd rfl (h now v scanPts zoneVerts)
  | shape k verts foreign =>
    simp only [step]
    split <;> rfl
  | _ => rfl

/-- **Processed (iff).** The output of an event is not `silent` iff the
event is a command, the node is active and the command is finite. The output
is then the `answer` of `output_cmd`, with the velocity of `velocity_cmd` and
the report of `report_cmd`. -/
theorem processed_iff (cfg : Config) (st : State) (e : Event) :
    (step cfg st e).2 ≠ .silent ↔
      ∃ now v scanPts zoneVerts, e = .cmd now v scanPts zoneVerts ∧ st.phase = .active ∧
        v.finite = true := by
  constructor
  · intro h
    cases e with
    | cmd now v scanPts zoneVerts =>
      rw [Ne, output_eq_silent_iff] at h
      simp only [not_or, not_not, Bool.not_eq_false] at h
      exact ⟨now, v, scanPts, zoneVerts, rfl, h.1, h.2⟩
    | shape k verts foreign =>
      exact absurd (output_of_not_cmd cfg st _ (fun _ _ _ _ => by simp)) h
    | _ => exact absurd rfl h
  · rintro ⟨now, v, scanPts, zoneVerts, rfl, hp, hf⟩
    rw [Ne, output_eq_silent_iff]
    simp [hp, hf]

/-- **Inactive.** While the node is not active the output of every event is
`silent`. (A command then also leaves the state unchanged:
`step_cmd_of_not_active`.) -/
theorem output_of_not_active (cfg : Config) (st : State) (e : Event) (h : st.phase ≠ .active) :
    (step cfg st e).2 = .silent := by
  by_contra hn
  obtain ⟨_, _, _, _, -, hp, -⟩ := (processed_iff cfg st e).mp hn
  exact h hp

/-! ## The lifecycle (E1–E5) -/

/-- The phase after an event (N2–N8). A transition that the current phase
does not allow changes nothing; `shutdown` leads to `finalized` from
everywhere; no other event changes the phase. -/
def nextPhase (p : Phase) : Event → Phase
  | .configure => if p = .unconfigured then .inactive else p
  | .activate => if p = .inactive then .active else p
  | .deactivate => if p = .active then .inactive else p
  | .cleanup => if p = .inactive then .unconfigured else p
  | .shutdown => .finalized
  | _ => p

/-- The phase follows the lifecycle events alone: not scans, not shapes, not
commands, not the configuration. -/
theorem step_phase (cfg : Config) (st : State) (e : Event) :
    (step cfg st e).1.phase = nextPhase st.phase e := by
  cases e with
  | cmd now v scanPts zoneVerts => exact (step_cmd_scan cfg st now v scanPts zoneVerts).2
  | shape k verts foreign =>
    simp only [step, nextPhase]
    split <;> rfl
  | scan now s =>
    simp only [step, nextPhase]
    split <;> rfl
  | shutdown => rfl
  | _ =>
    simp only [step, nextPhase]
    split <;> simp_all

/-- E1 (N3). -/
theorem step_configure (cfg : Config) (st : State) :
    step cfg st .configure =
      (if st.phase = .unconfigured then { st with phase := .inactive } else st, .silent) := rfl

/-- E2 (N5): activation publishes nothing and changes nothing but the
phase. -/
theorem step_activate (cfg : Config) (st : State) :
    step cfg st .activate =
      (if st.phase = .inactive then { st with phase := .active } else st, .silent) := rfl

/-- E3 (N6): deactivation resets the record of published zeros and the
previous report; the stored scan, the shapes and the stop stamp stay. -/
theorem step_deactivate (cfg : Config) (st : State) :
    step cfg st .deactivate =
      (if st.phase = .active then
          { st with phase := .inactive, prevZero := false, prevReport := .normal }
        else st, .silent) := rfl

/-- E4 (N7): cleanup discards the stored scan and every shape. -/
theorem step_cleanup (cfg : Config) (st : State) :
    step cfg st .cleanup =
      (if st.phase = .inactive then
          { st with phase := .unconfigured, scan := none,
                    shapes := st.shapes.map (fun _ => none) }
        else st, .silent) := rfl

/-- E5. -/
theorem step_shutdown (cfg : Config) (st : State) :
    step cfg st .shutdown = ({ st with phase := .finalized }, .silent) := rfl

/-- E6 (N4, R5): a scan is stored while the node is inactive or active, and
nothing else changes; nothing is published. -/
theorem step_scan (cfg : Config) (st : State) (now : Nat) (s : Scan) :
    step cfg st (.scan now s) =
      (if st.phase = .inactive ∨ st.phase = .active then { st with scan := some s } else st,
        .silent) := rfl

/-- **After shutdown nothing happens (E5, ND4).** In the finalized phase no
event changes the state, and every output is `silent`. -/
theorem step_of_finalized (cfg : Config) (st : State) (e : Event) (h : st.phase = .finalized) :
    step cfg st e = (st, .silent) := by
  cases e with
  | cmd now v scanPts zoneVerts => exact step_cmd_of_not_active _ _ _ _ _ _ (by simp [h])
  | shape k verts foreign => simp [step, acceptShape, h]
  | shutdown =>
    cases st
    simp only at h
    subst h
    rfl
  | _ => simp [step, h]

/-! ## Shape messages (E7) -/

theorem step_shape (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) :
    step cfg st (.shape k verts foreign) =
      match acceptShape cfg st k verts foreign with
      | some sh => ({ st with shapes := st.shapes.set k (some sh) }, .silent)
      | none => (st, .silent) := rfl

/-- What `placeVerts` accepts: every vertex, rounded to the millimetre, lies
within the coordinate limit of `Shape.ok`; the result is the rounded
vertices, in order. -/
theorem placeVerts_eq_some_iff (verts : List Coords) (ps : List Point) :
    placeVerts verts = some ps ↔
      (∀ v ∈ verts, (vertexPointMm v).inRange (coordLimit - 3) = true) ∧
        ps = verts.map vertexPointMm := by
  unfold placeVerts
  simp only [List.all_map]
  split
  · rename_i h
    simp only [List.all_eq_true, Function.comp_apply] at h
    simp only [Option.some.injEq]
    exact ⟨fun h' => ⟨h, h'.symm⟩, fun h' => h'.2.symm⟩
  · rename_i h
    simp only [List.all_eq_true, Function.comp_apply] at h
    simp only [reduceCtorEq, false_iff, not_and]
    exact fun h' => absurd h' h

/-- **Shape accepted (iff), the decision.** A polygon message for zone `k`
installs the shape `sh` iff

* the node is inactive or active (N4, N35);
* zone `k` exists and is a topic zone;
* the shim handed over the vertices in the base frame: `verts = some vs`
  (if the message is in another frame, its transform was found, N33);
* there are at least three of them (N33);
* every vertex, rounded to the millimetre, lies within the coordinate limit
  of `Shape.ok`, which a coordinate that is not finite never does (ND6;
  `vertexCoordMm_finite_of_lt` in `Margin.lean`);

and `sh` holds whether the frame is foreign, and the rounded vertices. -/
theorem acceptShape_eq_some_iff (cfg : Config) (st : State) (k : Nat)
    (verts : Option (List Coords)) (foreign : Bool) (sh : TopicShape) :
    acceptShape cfg st k verts foreign = some sh ↔
      (st.phase = .inactive ∨ st.phase = .active) ∧
      (∃ z, cfg.zones[k]? = some z ∧ z.shape = .topic) ∧
      ∃ vs, verts = some vs ∧ 3 ≤ vs.length ∧
        (∀ v ∈ vs, (vertexPointMm v).inRange (coordLimit - 3) = true) ∧
        sh = ⟨foreign, vs.map vertexPointMm⟩ := by
  unfold acceptShape
  by_cases hp : st.phase = .inactive ∨ st.phase = .active
  swap
  · simp [hp]
  rw [if_pos hp]
  cases hz : cfg.zones[k]? with
  | none => simp
  | some z =>
    cases hs : z.shape with
    | fixed s => simp [hs]
    | topic =>
      simp only [Option.some.injEq, exists_eq_left', hp, true_and, hs]
      unfold TopicShape.ofMessage
      cases verts with
      | none => simp
      | some vs =>
        simp only [Option.some.injEq, exists_eq_left']
        by_cases hl : 3 ≤ vs.length
        · rw [if_pos hl]
          cases hpv : placeVerts vs with
          | none =>
            simp only [Option.map_none, reduceCtorEq, false_iff]
            rintro ⟨-, h1, -⟩
            have := (placeVerts_eq_some_iff vs _).mpr ⟨h1, rfl⟩
            rw [hpv] at this
            cases this
          | some ps =>
            obtain ⟨h1, rfl⟩ := (placeVerts_eq_some_iff vs ps).mp hpv
            simp only [Option.map_some, Option.some.injEq]
            exact ⟨fun h => ⟨hl, h1, h.symm⟩, fun h => h.2.2.symm⟩
        · rw [if_neg hl]
          simp only [reduceCtorEq, false_iff]
          rintro ⟨h, -⟩
          exact hl h

/-- **Shape accepted: the state.** An accepted message replaces entry `k` of
the shapes by the new shape; nothing else changes: not the other entries,
not the scan, the record of zeros, the stop stamp, the previous report, the
phase. Nothing is published. -/
theorem step_shape_of_accepted (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) (sh : TopicShape) (h : acceptShape cfg st k verts foreign = some sh) :
    step cfg st (.shape k verts foreign) =
      ({ st with shapes := st.shapes.set k (some sh) }, .silent) := by
  rw [step_shape, h]

/-- A message that is not accepted changes nothing. -/
theorem step_shape_of_refused (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) (h : acceptShape cfg st k verts foreign = none) :
    step cfg st (.shape k verts foreign) = (st, .silent) := by
  rw [step_shape, h]

/-- **Shape accepted (only if).** A shape message changes the state only if
it is accepted. The converse does not hold: an accepted message that carries
the shape the zone already has leaves the state as it is
(`step_shape_same`). -/
theorem step_shape_changes (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) (h : (step cfg st (.shape k verts foreign)).1 ≠ st) :
    ∃ sh, acceptShape cfg st k verts foreign = some sh := by
  cases ha : acceptShape cfg st k verts foreign with
  | some sh => exact ⟨sh, rfl⟩
  | none => rw [step_shape_of_refused _ _ _ _ _ ha] at h; exact absurd rfl h

/-- An accepted message that carries the shape the zone already has does not
change the state: "accepted" and "the state changes" are not the same. -/
theorem step_shape_same (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) (sh : TopicShape) (h : acceptShape cfg st k verts foreign = some sh)
    (hs : st.shapes[k]? = some (some sh)) : (step cfg st (.shape k verts foreign)).1 = st := by
  rw [step_shape_of_accepted _ _ _ _ _ _ h]
  have : st.shapes.set k (some sh) = st.shapes := by
    apply List.ext_getElem?
    intro i
    by_cases hi : k = i
    · subst hi
      rw [hs, List.getElem?_set_self (List.getElem?_eq_some_iff.mp hs).1]
    · rw [List.getElem?_set_ne hi]
  simp only
  rw [this]

/-- After an accepted message, entry `k` is the new shape (if the state has
an entry for zone `k`, as every reachable state has: `Inv.len`), and every
other entry is what it was. -/
theorem shapes_after_accepted (st : State) (k : Nat) (sh : TopicShape) :
    (k < st.shapes.length → (st.shapes.set k (some sh))[k]? = some (some sh)) ∧
      ∀ j, j ≠ k → (st.shapes.set k (some sh))[j]? = st.shapes[j]? := by
  constructor
  · intro hk
    exact List.getElem?_set_self hk
  · intro j hj
    exact List.getElem?_set_ne (Ne.symm hj)

/-! ## The shapes at a command (N34) -/

theorem refreshShapes_length (cfg : Config) (shapes : List (Option TopicShape))
    (zoneVerts : List (Option (List Coords))) :
    (refreshShapes cfg shapes zoneVerts).length = shapes.length := by
  simp [refreshShapes]

/-- Entry `k` after the refresh: refreshed with entry `k` of the vertices
if zone `k` is an enabled topic zone, unchanged otherwise. -/
theorem refreshShapes_getElem? (cfg : Config) (shapes : List (Option TopicShape))
    (zoneVerts : List (Option (List Coords))) (k : Nat) :
    (refreshShapes cfg shapes zoneVerts)[k]? =
      shapes[k]?.map (fun o =>
        if cfg.refreshes k then o.map (·.refresh (zoneVerts.getD k none)) else o) := by
  simp only [refreshShapes, List.getElem?_map, List.getElem?_zipIdx]
  cases shapes[k]? <;> simp

/-- The refresh neither gives a zone a shape nor takes one away. -/
theorem refreshShapes_getD_eq_none_iff (cfg : Config) (shapes : List (Option TopicShape))
    (zoneVerts : List (Option (List Coords))) (k : Nat) :
    (refreshShapes cfg shapes zoneVerts).getD k none = none ↔ shapes.getD k none = none := by
  simp only [List.getD_eq_getElem?_getD, refreshShapes_getElem?]
  cases h : shapes[k]? with
  | none => simp
  | some o =>
    cases o with
    | none => simp
    | some sh => by_cases hr : cfg.refreshes k = true <;> simp [hr]

/-- What `placeVerts` gives has one point per vertex handed over. -/
theorem placeVerts_length (verts : List Coords) (ps : List Point) (h : placeVerts verts = some ps) :
    ps.length = verts.length := by
  obtain ⟨-, rfl⟩ := (placeVerts_eq_some_iff verts ps).mp h
  simp

/-- A shape received in the base frame is never moved. -/
theorem refresh_of_base (sh : TopicShape) (verts : Option (List Coords))
    (h : sh.foreign = false) : sh.refresh verts = sh := by
  simp [TopicShape.refresh, h]

/-- N34: without a transform (the command carries no vertices for the zone)
the vertices stay where they were. -/
theorem refresh_of_no_transform (sh : TopicShape) : sh.refresh none = sh := by
  unfold TopicShape.refresh
  split <;> rfl

/-- N34: a shape received in another frame follows that frame: when the
command carries as many vertices as the shape has, all within the coordinate
limit, they become its current vertices, rounded to the millimetre. -/
theorem refresh_of_foreign (sh : TopicShape) (vs : List Coords) (h : sh.foreign = true)
    (hlen : vs.length = sh.current.length)
    (hr : ∀ v ∈ vs, (vertexPointMm v).inRange (coordLimit - 3) = true) :
    (sh.refresh (some vs)).current = vs.map vertexPointMm := by
  have hp : placeVerts vs = some (vs.map vertexPointMm) :=
    (placeVerts_eq_some_iff _ _).mpr ⟨hr, rfl⟩
  simp [TopicShape.refresh, h, hp, hlen]

/-- Vertices that are refused (one of them is not within the coordinate
limit, or they are not as many as the shape has) leave the shape as it is. -/
theorem refresh_of_refused (sh : TopicShape) (vs : List Coords)
    (h : placeVerts vs = none ∨ vs.length ≠ sh.current.length) : sh.refresh (some vs) = sh := by
  unfold TopicShape.refresh
  by_cases hf : sh.foreign = true
  swap
  · rw [if_neg hf]
  rw [if_pos hf]
  cases hp : placeVerts vs with
  | none => simp [hp]
  | some ps =>
    have hl := placeVerts_length vs ps hp
    rcases h with h | h
    · rw [hp] at h; cases h
    · simp [hp, hl, h]

/-- **The refresh moves vertices; it does nothing else.** The frame flag and
the number of vertices are what they were, whatever the command carries. -/
theorem refresh_foreign (sh : TopicShape) (verts : Option (List Coords)) :
    (sh.refresh verts).foreign = sh.foreign ∧
      (sh.refresh verts).current.length = sh.current.length := by
  by_cases hf : sh.foreign = true
  swap
  · rw [refresh_of_base sh verts (by simpa using hf)]; exact ⟨rfl, rfl⟩
  cases verts with
  | none => rw [refresh_of_no_transform]; exact ⟨rfl, rfl⟩
  | some vs =>
    cases hp : placeVerts vs with
    | none => rw [refresh_of_refused sh vs (Or.inl hp)]; exact ⟨rfl, rfl⟩
    | some ps =>
      by_cases hlen : vs.length = sh.current.length
      · obtain ⟨h1, rfl⟩ := (placeVerts_eq_some_iff vs ps).mp hp
        refine ⟨?_, ?_⟩
        · simp [TopicShape.refresh, hf, hp, hlen]
        · rw [refresh_of_foreign sh vs hf hlen h1]; simpa using hlen
      · rw [refresh_of_refused sh vs (Or.inr hlen)]; exact ⟨rfl, rfl⟩

/-! ## No shape (N32) -/

/-- **No shape: not triggered.** A topic zone for which the shapes hold no
entry is not triggered by any points, whatever its `minPoints`. -/
theorem not_triggered_of_no_shape (cfg : Config) (shapes : List (Option TopicShape)) (k : Nat)
    (z0 z : Zone) (h0 : cfg.zones[k]? = some z0) (ht : z0.shape = .topic)
    (hn : shapes.getD k none = none) (hz : (cfg.withShapes shapes).zones[k]? = some z)
    (pts : List Point) : z.triggered pts = false := by
  rw [withShapes_getElem?, h0, hn] at hz
  simp only [Option.map_some, Option.some.injEq] at hz
  rw [withShape_none] at hz
  subst hz
  exact not_triggered_of_topic _ pts ht

/-- **No shape: no action names the zone.** If zone `k` is a topic zone and
the state holds no shape for it, no command, whatever it carries, is
answered with a stop or a slowdown by zone `k`. This is upstream's behaviour
(N32) and it is **not** fail-safe: a stop zone without shape does not
stop. -/
theorem evaluate_of_no_shape (cfg : Config) (st : State) (now : Nat)
    (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) (k : Nat) (z0 : Zone) (h0 : cfg.zones[k]? = some z0)
    (ht : z0.shape = .topic) (hn : st.shapes.getD k none = none) :
    evaluate cfg st now scanPts zoneVerts ≠ .stopZone k ∧
      ∀ r, evaluate cfg st now scanPts zoneVerts ≠ .slowdown k r := by
  have hn' : (refreshShapes cfg st.shapes zoneVerts).getD k none = none :=
    (refreshShapes_getD_eq_none_iff cfg st.shapes zoneVerts k).mpr hn
  constructor
  · intro h
    obtain ⟨pts, hpts, z, hz, -, htr, -⟩ :=
      (evaluate_eq_stopZone_iff cfg st now scanPts zoneVerts k).mp h
    rw [not_triggered_of_no_shape cfg _ k z0 z h0 ht hn' hz pts] at htr
    exact Bool.false_ne_true htr
  · intro r h
    obtain ⟨pts, hpts, -, z, hz, -, htr, -⟩ :=
      (evaluate_eq_slowdown_iff cfg st now scanPts zoneVerts k r).mp h
    rw [not_triggered_of_no_shape cfg _ k z0 z h0 ht hn' hz pts] at htr
    exact Bool.false_ne_true htr

/-- An action reports a stop by zone `k` only if it is that stop, and a
slowdown by zone `k` only if it is a slowdown by that zone. -/
theorem report_names (a : Action) (v : Twist) (k : Nat) :
    (a.report v = .stop k → a = .stopZone k) ∧
      (a.report v = .slowdown k → ∃ r, a = .slowdown k r) := by
  cases a with
  | pass => simp [Action.report]
  | stopSource => simp [Action.report]
  | stopZone j => simp [Action.report]
  | slowdown j r =>
    simp only [Action.report]
    split <;> simp

/-- **No shape: no output names the zone.** If zone `k` is a topic zone and
the state holds no shape for it, the output for any event carries neither a
velocity action nor a report that names zone `k`. -/
theorem output_of_no_shape (cfg : Config) (st : State) (e : Event) (k : Nat) (z0 : Zone)
    (h0 : cfg.zones[k]? = some z0) (ht : z0.shape = .topic)
    (hn : st.shapes.getD k none = none) :
    (step cfg st e).2.velocity ≠ some (.stopZone k) ∧
      (∀ r, (step cfg st e).2.velocity ≠ some (.slowdown k r)) ∧
      (step cfg st e).2.report ≠ some (.stop k) ∧
      (step cfg st e).2.report ≠ some (.slowdown k) := by
  by_cases hproc : (step cfg st e).2 = .silent
  · rw [hproc]
    simp [Output.velocity, Output.report]
  · obtain ⟨now, v, scanPts, zoneVerts, rfl, hp, hf⟩ := (processed_iff cfg st e).mp hproc
    obtain ⟨h1, h2⟩ := evaluate_of_no_shape cfg st now scanPts zoneVerts k z0 h0 ht hn
    refine ⟨?_, ?_, ?_, ?_⟩
    · intro h
      exact h1 ((velocity_eq_some_iff _ _ _ _ _ _ _).mp h).2.2.1
    · intro r h
      exact h2 r ((velocity_eq_some_iff _ _ _ _ _ _ _).mp h).2.2.1
    · intro h
      exact h1 ((report_names _ v k).1 ((report_eq_some_iff _ _ _ _ _ _ _).mp h).2.2.1)
    · intro h
      obtain ⟨r, hr⟩ := (report_names _ v k).2 ((report_eq_some_iff _ _ _ _ _ _ _).mp h).2.2.1
      exact h2 r hr

/-! ## Replacing a shape affects that zone only -/

/-- A zone as it stands depends on the shapes only through its own entry. -/
theorem withShapes_getElem?_congr (cfg : Config) (shapes shapes' : List (Option TopicShape))
    (j : Nat) (h : shapes.getD j none = shapes'.getD j none) :
    (cfg.withShapes shapes).zones[j]? = (cfg.withShapes shapes').zones[j]? := by
  rw [withShapes_getElem?, withShapes_getElem?, h]

/-- **Replacing a shape affects that zone only.** For two lists of shapes
that agree on every zone but `k`, every zone other than `k` is the same zone,
hence triggered by the same points; and the actions for any points are the
same unless zone `k` is triggered under the one and not under the other. -/
theorem zonesAction_withShapes_congr (cfg : Config) (shapes shapes' : List (Option TopicShape))
    (k : Nat) (pts : List Point)
    (hother : ∀ j, j ≠ k → shapes.getD j none = shapes'.getD j none)
    (hk : ∀ z z', (cfg.withShapes shapes).zones[k]? = some z →
      (cfg.withShapes shapes').zones[k]? = some z' → z.triggered pts = z'.triggered pts) :
    zonesAction (cfg.withShapes shapes) pts = zonesAction (cfg.withShapes shapes') pts := by
  apply zonesAction_eq_of_map
  apply List.ext_getElem?
  intro j
  rw [List.getElem?_map, List.getElem?_map]
  by_cases hj : j = k
  · subst hj
    have h1 := withShapes_getElem? cfg shapes j
    have h2 := withShapes_getElem? cfg shapes' j
    cases h0 : cfg.zones[j]? with
    | none => rw [h1, h2, h0]; rfl
    | some z0 =>
      rw [h0] at h1 h2
      simp only [Option.map_some] at h1 h2
      rw [h1, h2]
      simp only [Option.map_some, Option.some.injEq, Prod.mk.injEq]
      exact ⟨by rw [(withShape_fields z0 _).1, (withShape_fields z0 _).1], hk _ _ h1 h2⟩
  · rw [withShapes_getElem?_congr cfg shapes shapes' j (hother j hj)]

/-! ## Invariants of reachable states -/

/-- What every stored shape satisfies: at least three vertices, all within
the coordinate limit of `Shape.ok`. -/
def ShapeOk (sh : TopicShape) : Prop :=
  3 ≤ sh.current.length ∧ sh.current.all (·.inRange (coordLimit - 3)) = true

/-- **Lifecycle invariants.** What holds in every state reachable from
`State.init cfg` (`inv_init`, `inv_step`; over runs: `run_inv` in
`Trace.lean`):

* `quiet`: while the node is unconfigured or inactive, no zero is on record
  and the previous report is `normal`. (Not claimed for `finalized`: a node
  shut down from the active phase keeps these two fields as they were; they
  are never read again, `step_of_finalized`.)
* `empty`: while it is unconfigured, no scan and no shape is stored;
* `len`, `topic`: the shapes have one entry per zone, and an entry only
  at a topic zone;
* `ok`: every stored shape satisfies `ShapeOk`. -/
structure Inv (cfg : Config) (st : State) : Prop where
  quiet : st.phase = .unconfigured ∨ st.phase = .inactive →
    st.prevZero = false ∧ st.prevReport = .normal
  empty : st.phase = .unconfigured → st.scan = none ∧ ∀ o ∈ st.shapes, o = none
  len : st.shapes.length = cfg.zones.length
  topic : ∀ (k : Nat) (sh : TopicShape), st.shapes[k]? = some (some sh) →
    ∃ z : Zone, cfg.zones[k]? = some z ∧ z.shape = ZoneShape.topic
  ok : ∀ (k : Nat) (sh : TopicShape), st.shapes[k]? = some (some sh) → ShapeOk sh

theorem inv_init (cfg : Config) : Inv cfg (State.init cfg) where
  quiet := fun _ => ⟨rfl, rfl⟩
  empty := fun _ => ⟨rfl, by simp [State.init]⟩
  len := by simp [State.init]
  topic := by
    intro k sh h
    simp only [State.init, List.getElem?_map] at h
    cases hz : cfg.zones[k]? <;> simp [hz] at h
  ok := by
    intro k sh h
    simp only [State.init, List.getElem?_map] at h
    cases hz : cfg.zones[k]? <;> simp [hz] at h

theorem shapeOk_of_accept (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) (sh : TopicShape) (h : acceptShape cfg st k verts foreign = some sh) :
    ShapeOk sh := by
  obtain ⟨-, -, vs, -, hlen, h1, rfl⟩ := (acceptShape_eq_some_iff cfg st k verts foreign sh).mp h
  refine ⟨by simpa using hlen, ?_⟩
  simpa [List.all_eq_true] using h1

theorem shapeOk_refresh (sh : TopicShape) (verts : Option (List Coords)) (h : ShapeOk sh) :
    ShapeOk (sh.refresh verts) := by
  by_cases hf : sh.foreign = true
  swap
  · rw [refresh_of_base sh verts (by simpa using hf)]; exact h
  cases verts with
  | none => rw [refresh_of_no_transform]; exact h
  | some vs =>
    cases hp : placeVerts vs with
    | none => rw [refresh_of_refused sh vs (Or.inl hp)]; exact h
    | some ps =>
      obtain ⟨h1, rfl⟩ := (placeVerts_eq_some_iff vs ps).mp hp
      by_cases hlen : vs.length = sh.current.length
      · have hc := refresh_of_foreign sh vs hf hlen h1
        refine ⟨?_, ?_⟩
        · rw [hc]; simpa [hlen] using h.1
        · rw [hc]; simpa [List.all_eq_true] using h1
      · rw [refresh_of_refused sh vs (Or.inr hlen)]; exact h

/-- **The invariants are kept by every event.** -/
theorem inv_step (cfg : Config) (st : State) (e : Event) (h : Inv cfg st) :
    Inv cfg (step cfg st e).1 := by
  cases e with
  | configure =>
    rw [step_configure]
    by_cases hp : st.phase = .unconfigured
    · rw [if_pos hp]
      exact ⟨fun _ => h.quiet (Or.inl hp), fun hc => by simp at hc, h.len, h.topic, h.ok⟩
    · rw [if_neg hp]; exact h
  | activate =>
    rw [step_activate]
    by_cases hp : st.phase = .inactive
    · rw [if_pos hp]
      exact ⟨fun hc => by simp at hc, fun hc => by simp at hc, h.len, h.topic, h.ok⟩
    · rw [if_neg hp]; exact h
  | deactivate =>
    rw [step_deactivate]
    by_cases hp : st.phase = .active
    · rw [if_pos hp]
      exact ⟨fun _ => ⟨rfl, rfl⟩, fun hc => by simp at hc, h.len, h.topic, h.ok⟩
    · rw [if_neg hp]; exact h
  | cleanup =>
    rw [step_cleanup]
    by_cases hp : st.phase = .inactive
    · rw [if_pos hp]
      refine ⟨fun _ => h.quiet (Or.inr hp), fun _ => ⟨rfl, by simp⟩, by simpa using h.len, ?_, ?_⟩
      · intro k sh hk
        simp only [List.getElem?_map] at hk
        cases hs : st.shapes[k]? <;> simp [hs] at hk
      · intro k sh hk
        simp only [List.getElem?_map] at hk
        cases hs : st.shapes[k]? <;> simp [hs] at hk
    · rw [if_neg hp]; exact h
  | shutdown =>
    rw [step_shutdown]
    exact ⟨fun hc => by simp at hc, fun hc => by simp at hc, h.len, h.topic, h.ok⟩
  | scan now s =>
    rw [step_scan]
    by_cases hp : st.phase = .inactive ∨ st.phase = .active
    · rw [if_pos hp]
      refine ⟨h.quiet, fun hc => ?_, h.len, h.topic, h.ok⟩
      simp only at hc
      rcases hp with hp | hp <;> rw [hp] at hc <;> cases hc
    · rw [if_neg hp]; exact h
  | shape k verts foreign =>
    cases ha : acceptShape cfg st k verts foreign with
    | none => rw [step_shape_of_refused _ _ _ _ _ ha]; exact h
    | some sh =>
      rw [step_shape_of_accepted _ _ _ _ _ _ ha]
      obtain ⟨hp, ⟨z, hz, hzt⟩, -⟩ := (acceptShape_eq_some_iff cfg st k verts foreign sh).mp ha
      refine ⟨h.quiet, fun hc => ?_, by simpa using h.len, ?_, ?_⟩
      · simp only at hc
        rcases hp with hp | hp <;> rw [hp] at hc <;> cases hc
      · intro j sh' hj
        simp only at hj
        by_cases hjk : k = j
        · subst hjk; exact ⟨z, hz, hzt⟩
        · rw [List.getElem?_set_ne hjk] at hj
          exact h.topic j sh' hj
      · intro j sh' hj
        simp only at hj
        by_cases hjk : k = j
        · subst hjk
          have hlt : k < st.shapes.length := by
            by_contra hc
            rw [List.getElem?_eq_none (by simpa using hc)] at hj
            cases hj
          rw [List.getElem?_set_self hlt] at hj
          simp only [Option.some.injEq] at hj
          subst hj
          exact shapeOk_of_accept cfg st k verts foreign sh ha
        · rw [List.getElem?_set_ne hjk] at hj
          exact h.ok j sh' hj
  | cmd now v scanPts zoneVerts =>
    by_cases hp : st.phase = .active
    · cases hf : v.finite
      · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; exact h
      · rw [step_cmd _ _ _ _ _ _ hp hf]
        have key : ∀ (k : Nat) (sh : TopicShape),
            (shapesAfter cfg st now scanPts zoneVerts)[k]? = some (some sh) →
            ∃ sh0, st.shapes[k]? = some (some sh0) ∧ (ShapeOk sh0 → ShapeOk sh) := by
          intro k sh hk
          unfold shapesAfter at hk
          split at hk
          · exact ⟨sh, hk, id⟩
          · rw [refreshShapes_getElem?] at hk
            cases hs : st.shapes[k]? with
            | none => simp [hs] at hk
            | some o =>
              cases o with
              | none =>
                rw [hs] at hk
                by_cases hr : cfg.refreshes k = true <;> simp [hr] at hk
              | some sh0 =>
                refine ⟨sh0, rfl, fun hok => ?_⟩
                rw [hs] at hk
                by_cases hr : cfg.refreshes k = true
                · simp only [Option.map_some, hr, ↓reduceIte, Option.some.injEq] at hk
                  rw [← hk]
                  exact shapeOk_refresh sh0 _ hok
                · simp only [Option.map_some, hr, Bool.false_eq_true, ↓reduceIte,
                    Option.some.injEq] at hk
                  rw [← hk]
                  exact hok
        refine ⟨fun hc => ?_, fun hc => ?_, ?_, ?_, ?_⟩
        · simp only at hc
          rcases hc with hc | hc <;> rw [hp] at hc <;> cases hc
        · simp only at hc
          rw [hp] at hc
          cases hc
        · simp only
          unfold shapesAfter
          split
          · exact h.len
          · rw [refreshShapes_length]; exact h.len
        · intro k sh hk
          obtain ⟨sh0, h0, -⟩ := key k sh hk
          exact h.topic k sh0 h0
        · intro k sh hk
          obtain ⟨sh0, h0, hok⟩ := key k sh hk
          exact hok (h.ok k sh0 h0)
    · rw [step_cmd_of_not_active _ _ _ _ _ _ hp]; exact h

/-! ## After activation (N6) -/

/-- **After activation.** If the node is inactive, with no zero on record and
the previous report `normal` (as in every reachable inactive state:
`Inv.quiet`), then after `activate` it is active, and a finite command that
comes next is answered with a velocity, also when that velocity is zero: a
stop is never withheld right after an activation. Its report is published
unless it is `normal`. -/
theorem after_activation (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords)))
    (hp : st.phase = .inactive)
    (hq : st.prevZero = false) (hr : st.prevReport = .normal) (hf : v.finite = true) :
    (step cfg st .activate).1.phase = .active ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanPts zoneVerts)).2.velocity =
        some (evaluate cfg st now scanPts zoneVerts) ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanPts zoneVerts)).2.report =
        if (evaluate cfg st now scanPts zoneVerts).report v = .normal then none
        else some ((evaluate cfg st now scanPts zoneVerts).report v) := by
  have hst : (step cfg st .activate).1 = { st with phase := .active } := by
    rw [step_activate, if_pos hp]
  have he : evaluate cfg { st with phase := .active } now scanPts zoneVerts =
      evaluate cfg st now scanPts zoneVerts := evaluate_congr _ _ _ _ _ _ rfl rfl
  rw [hst]
  refine ⟨rfl, ?_, ?_⟩
  · rw [velocity_of_not_prevZero _ _ _ _ _ _ rfl hf (by exact hq), he]
  · rw [report_cmd _ _ _ _ _ _ rfl hf, he]
    simp only [hr]

/-- The same for a reachable state: the invariant supplies the two
hypotheses on the record. -/
theorem after_activation_of_inv (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanPts : Option (List Coords)) (zoneVerts : List (Option (List Coords))) (hinv : Inv cfg st)
    (hp : st.phase = .inactive) (hf : v.finite = true) :
    (step cfg st .activate).1.phase = .active ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanPts zoneVerts)).2.velocity =
        some (evaluate cfg st now scanPts zoneVerts) ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanPts zoneVerts)).2.report =
        if (evaluate cfg st now scanPts zoneVerts).report v = .normal then none
        else some ((evaluate cfg st now scanPts zoneVerts).report v) :=
  after_activation cfg st now v scanPts zoneVerts hp (hinv.quiet (Or.inr hp)).1
    (hinv.quiet (Or.inr hp)).2 hf

/-! ## Saturated points and topic zones (D12) -/

/-- **Every zone in force lies within the coordinate limit.** In a state
that satisfies the invariants, with an accepted configuration, every zone
that is tested for a command (a fixed one, or a topic zone with its shape,
refreshed) lies within the coordinate limit. Hence a saturated point is in
none of them (`contains_eq_false_of_far'`), and such points do not change
the action (`zonesAction_filter_inRange'`). -/
theorem liveConfig_bounded (cfg : Config) (st : State) (zoneVerts : List (Option (List Coords)))
    (hinv : Inv cfg st) (hok : cfg.ok = true) : ZonesBounded (liveConfig cfg st zoneVerts) := by
  intro z hz s hs
  obtain ⟨k, hk⟩ := List.mem_iff_getElem?.mp hz
  unfold liveConfig at hk
  obtain ⟨z0, h0, hzz, -⟩ := withShapes_action cfg _ k z hk
  cases hs0 : z0.shape with
  | fixed s0 =>
    rw [withShape_of_fixed z0 _ s0 hs0] at hzz
    subst hzz
    exact zonesBounded_of_ok cfg hok z (List.mem_of_getElem? h0) s hs
  | topic =>
    cases hsh : (refreshShapes cfg st.shapes zoneVerts).getD k none with
    | none =>
      rw [hsh, withShape_none] at hzz
      subst hzz
      rw [hs0] at hs
      cases hs
    | some sh =>
      rw [hsh, withShape_of_topic z0 sh hs0] at hzz
      subst hzz
      simp only [ZoneShape.fixed.injEq] at hs
      subst hs
      -- `sh` is a refreshed stored shape
      have hget : (refreshShapes cfg st.shapes zoneVerts)[k]? = some (some sh) := by
        rw [List.getD_eq_getElem?_getD] at hsh
        cases hg : (refreshShapes cfg st.shapes zoneVerts)[k]? with
        | none => simp [hg] at hsh
        | some o => simp only [hg, Option.getD_some] at hsh; rw [hsh]
      rw [refreshShapes_getElem?] at hget
      cases hst : st.shapes[k]? with
      | none => simp [hst] at hget
      | some o =>
        cases o with
        | none =>
          rw [hst] at hget
          by_cases hr : cfg.refreshes k = true <;> simp [hr] at hget
        | some sh0 =>
          have hok0 := hinv.ok k sh0 hst
          rw [hst] at hget
          have hsok : ShapeOk sh := by
            by_cases hr : cfg.refreshes k = true
            · simp only [Option.map_some, hr, ↓reduceIte, Option.some.injEq] at hget
              rw [← hget]
              exact shapeOk_refresh sh0 _ hok0
            · simp only [Option.map_some, hr, Bool.false_eq_true, ↓reduceIte,
                Option.some.injEq] at hget
              rw [← hget]
              exact hok0
          exact shapeBounded_polygon_of_all _ hsok.2

end ROSlean.CollisionMonitor
