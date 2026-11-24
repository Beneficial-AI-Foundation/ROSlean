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
    (h : ∀ now v scanTf zoneTfs, e ≠ .cmd now v scanTf zoneTfs) : (step cfg st e).2 = .silent := by
  cases e with
  | cmd now v scanTf zoneTfs => exact absurd rfl (h now v scanTf zoneTfs)
  | shape k verts place =>
    simp only [step]
    split <;> rfl
  | _ => rfl

/-- **Processed (iff).** The output of an event is not `silent` iff the
event is a command, the node is active and the command is finite. The output
is then the `answer` of `output_cmd`, with the velocity of `velocity_cmd` and
the report of `report_cmd`. -/
theorem processed_iff (cfg : Config) (st : State) (e : Event) :
    (step cfg st e).2 ≠ .silent ↔
      ∃ now v scanTf zoneTfs, e = .cmd now v scanTf zoneTfs ∧ st.phase = .active ∧
        v.finite = true := by
  constructor
  · intro h
    cases e with
    | cmd now v scanTf zoneTfs =>
      rw [Ne, output_eq_silent_iff] at h
      simp only [not_or, not_not, Bool.not_eq_false] at h
      exact ⟨now, v, scanTf, zoneTfs, rfl, h.1, h.2⟩
    | shape k verts place =>
      exact absurd (output_of_not_cmd cfg st _ (fun _ _ _ _ => by simp)) h
    | _ => exact absurd rfl h
  · rintro ⟨now, v, scanTf, zoneTfs, rfl, hp, hf⟩
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
  | cmd now v scanTf zoneTfs => exact (step_cmd_scan cfg st now v scanTf zoneTfs).2
  | shape k verts place =>
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
  | cmd now v scanTf zoneTfs => exact step_cmd_of_not_active _ _ _ _ _ _ (by simp [h])
  | shape k verts place => simp [step, acceptShape, h]
  | shutdown =>
    cases st
    simp only at h
    subst h
    rfl
  | _ => simp [step, h]

/-! ## Shape messages (E7) -/

theorem step_shape (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) :
    step cfg st (.shape k verts place) =
      match acceptShape cfg st k verts place with
      | some sh => ({ st with shapes := st.shapes.set k (some sh) }, .silent)
      | none => (st, .silent) := rfl

/-- What `TopicShape.place` accepts. -/
theorem place_eq_some_iff (verts : List (Nat × Nat)) (foreign : Bool) (t : Xform)
    (sh : TopicShape) :
    TopicShape.place verts foreign t = some sh ↔
      (∀ v ∈ verts, (t.vertexMm v.1 v.2).inRange (coordLimit - 3) = true) ∧
        sh = ⟨verts, foreign, verts.map (fun v => t.vertexMm v.1 v.2)⟩ := by
  unfold TopicShape.place
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
* the message has at least three vertices (N33), all with finite
  coordinates (ND6);
* it is in the base frame, or it is in another frame and its transform was
  found and is usable (N33, ND7);
* every vertex, placed in the base frame and rounded to the millimetre, lies
  within the coordinate limit of `Shape.ok` (ND6);

and `sh` holds the vertices as received, whether the frame is foreign, and
the placed vertices. -/
theorem acceptShape_eq_some_iff (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (sh : TopicShape) :
    acceptShape cfg st k verts place = some sh ↔
      (st.phase = .inactive ∨ st.phase = .active) ∧
      (∃ z, cfg.zones[k]? = some z ∧ z.shape = .topic) ∧
      3 ≤ verts.length ∧
      (∀ v ∈ verts, ROSleanCore.CollisionMonitorFFI.F32.finite v.1 = true ∧
        ROSleanCore.CollisionMonitorFFI.F32.finite v.2 = true) ∧
      ∃ t foreign,
        ((place = .base ∧ t = Xform.identity ∧ foreign = false) ∨
          (place = .foreign (some t) ∧ t.usable = true ∧ foreign = true)) ∧
        (∀ v ∈ verts, (t.vertexMm v.1 v.2).inRange (coordLimit - 3) = true) ∧
        sh = ⟨verts, foreign, verts.map (fun v => t.vertexMm v.1 v.2)⟩ := by
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
      by_cases hv : 3 ≤ verts.length ∧ ∀ v ∈ verts,
          ROSleanCore.CollisionMonitorFFI.F32.finite v.1 = true ∧
          ROSleanCore.CollisionMonitorFFI.F32.finite v.2 = true
      swap
      · have : (decide (3 ≤ verts.length) && verts.all (fun v =>
            ROSleanCore.CollisionMonitorFFI.F32.finite v.1 &&
            ROSleanCore.CollisionMonitorFFI.F32.finite v.2)) = false := by
          simpa [List.all_eq_true] using hv
        rw [this]
        simp only [Bool.false_eq_true, ↓reduceIte, reduceCtorEq, false_iff]
        rintro ⟨h1, h2, -⟩
        exact hv ⟨h1, h2⟩
      · have : (decide (3 ≤ verts.length) && verts.all (fun v =>
            ROSleanCore.CollisionMonitorFFI.F32.finite v.1 &&
            ROSleanCore.CollisionMonitorFFI.F32.finite v.2)) = true := by
          simpa [List.all_eq_true] using hv
        rw [this, if_pos rfl]
        cases place with
        | base =>
          rw [place_eq_some_iff]
          constructor
          · rintro ⟨h1, h2⟩
            exact ⟨hv.1, hv.2, Xform.identity, false, Or.inl ⟨rfl, rfl, rfl⟩, h1, h2⟩
          · rintro ⟨-, -, t, f, (⟨-, rfl, rfl⟩ | ⟨h, -⟩), h1, h2⟩
            · exact ⟨h1, h2⟩
            · cases h
        | foreign tf =>
          cases tf with
          | none =>
            simp only [reduceCtorEq, false_iff]
            rintro ⟨-, -, t, f, (⟨h, -⟩ | ⟨h, -⟩), -⟩ <;> cases h
          | some t0 =>
            simp only
            by_cases hu : t0.usable = true
            · rw [if_pos hu, place_eq_some_iff]
              constructor
              · rintro ⟨h1, h2⟩
                exact ⟨hv.1, hv.2, t0, true, Or.inr ⟨rfl, hu, rfl⟩, h1, h2⟩
              · rintro ⟨-, -, t, f, (⟨h, -⟩ | ⟨h, -, rfl⟩), h1, h2⟩
                · cases h
                · cases h
                  exact ⟨h1, h2⟩
            · rw [if_neg hu]
              simp only [reduceCtorEq, false_iff]
              rintro ⟨-, -, t, f, (⟨h, -⟩ | ⟨h, hu', -⟩), -⟩
              · cases h
              · cases h
                exact hu hu'

/-- **Shape accepted: the state.** An accepted message replaces entry `k` of
the shapes by the new shape; nothing else changes: not the other entries,
not the scan, the record of zeros, the stop stamp, the previous report, the
phase. Nothing is published. -/
theorem step_shape_of_accepted (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (sh : TopicShape) (h : acceptShape cfg st k verts place = some sh) :
    step cfg st (.shape k verts place) =
      ({ st with shapes := st.shapes.set k (some sh) }, .silent) := by
  rw [step_shape, h]

/-- A message that is not accepted changes nothing. -/
theorem step_shape_of_refused (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (h : acceptShape cfg st k verts place = none) :
    step cfg st (.shape k verts place) = (st, .silent) := by
  rw [step_shape, h]

/-- **Shape accepted (only if).** A shape message changes the state only if
it is accepted. The converse does not hold: an accepted message that carries
the shape the zone already has leaves the state as it is
(`step_shape_same`). -/
theorem step_shape_changes (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (h : (step cfg st (.shape k verts place)).1 ≠ st) :
    ∃ sh, acceptShape cfg st k verts place = some sh := by
  cases ha : acceptShape cfg st k verts place with
  | some sh => exact ⟨sh, rfl⟩
  | none => rw [step_shape_of_refused _ _ _ _ _ ha] at h; exact absurd rfl h

/-- An accepted message that carries the shape the zone already has does not
change the state: "accepted" and "the state changes" are not the same. -/
theorem step_shape_same (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (sh : TopicShape) (h : acceptShape cfg st k verts place = some sh)
    (hs : st.shapes[k]? = some (some sh)) : (step cfg st (.shape k verts place)).1 = st := by
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
    (zoneTfs : List (Option Xform)) :
    (refreshShapes cfg shapes zoneTfs).length = shapes.length := by
  simp [refreshShapes]

/-- Entry `k` after the refresh: refreshed with entry `k` of the transforms
if zone `k` is an enabled topic zone, unchanged otherwise. -/
theorem refreshShapes_getElem? (cfg : Config) (shapes : List (Option TopicShape))
    (zoneTfs : List (Option Xform)) (k : Nat) :
    (refreshShapes cfg shapes zoneTfs)[k]? =
      shapes[k]?.map (fun o =>
        if cfg.refreshes k then o.map (·.refresh (zoneTfs.getD k none)) else o) := by
  simp only [refreshShapes, List.getElem?_map, List.getElem?_zipIdx]
  cases shapes[k]? <;> simp

/-- The refresh neither gives a zone a shape nor takes one away. -/
theorem refreshShapes_getD_eq_none_iff (cfg : Config) (shapes : List (Option TopicShape))
    (zoneTfs : List (Option Xform)) (k : Nat) :
    (refreshShapes cfg shapes zoneTfs).getD k none = none ↔ shapes.getD k none = none := by
  simp only [List.getD_eq_getElem?_getD, refreshShapes_getElem?]
  cases h : shapes[k]? with
  | none => simp
  | some o =>
    cases o with
    | none => simp
    | some sh => by_cases hr : cfg.refreshes k = true <;> simp [hr]

/-- The refresh keeps what was received: only the current vertices move. -/
theorem refresh_raw (sh : TopicShape) (tf : Option Xform) :
    (sh.refresh tf).raw = sh.raw ∧ (sh.refresh tf).foreign = sh.foreign := by
  unfold TopicShape.refresh
  split
  · split
    · split
      · dsimp only
        split <;> exact ⟨rfl, rfl⟩
      · exact ⟨rfl, rfl⟩
    · exact ⟨rfl, rfl⟩
  · exact ⟨rfl, rfl⟩

/-- A shape received in the base frame is never moved. -/
theorem refresh_of_base (sh : TopicShape) (tf : Option Xform) (h : sh.foreign = false) :
    sh.refresh tf = sh := by
  simp [TopicShape.refresh, h]

/-- N34: a shape received in another frame follows that frame: with a usable
transform that keeps every vertex within the coordinate limit, its current
vertices are the received ones under that transform. -/
theorem refresh_of_foreign (sh : TopicShape) (t : Xform) (h : sh.foreign = true)
    (hu : t.usable = true)
    (hr : ∀ v ∈ sh.raw, (t.vertexMm v.1 v.2).inRange (coordLimit - 3) = true) :
    (sh.refresh (some t)).current = sh.raw.map (fun v => t.vertexMm v.1 v.2) := by
  have : (sh.raw.map (fun v => t.vertexMm v.1 v.2)).all (·.inRange (coordLimit - 3)) = true := by
    simpa [List.all_eq_true] using hr
  simp [TopicShape.refresh, h, hu, this]

/-- N34: without a usable transform the vertices stay where they were. -/
theorem refresh_of_no_transform (sh : TopicShape) (tf : Option Xform) (h : NoTransform tf) :
    sh.refresh tf = sh := by
  unfold TopicShape.refresh
  cases tf with
  | none => simp
  | some t => simp [(noTransform_some_iff t).mp h]

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
the state holds no shape for it, no command, with whatever transforms, is
answered with a stop or a slowdown by zone `k`. This is upstream's behaviour
(N32) and it is **not** fail-safe: a stop zone without shape does not
stop. -/
theorem evaluate_of_no_shape (cfg : Config) (st : State) (now : Nat) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (k : Nat) (z0 : Zone) (h0 : cfg.zones[k]? = some z0)
    (ht : z0.shape = .topic) (hn : st.shapes.getD k none = none) :
    evaluate cfg st now scanTf zoneTfs ≠ .stopZone k ∧
      ∀ r, evaluate cfg st now scanTf zoneTfs ≠ .slowdown k r := by
  have hn' : (refreshShapes cfg st.shapes zoneTfs).getD k none = none :=
    (refreshShapes_getD_eq_none_iff cfg st.shapes zoneTfs k).mpr hn
  constructor
  · intro h
    obtain ⟨pts, hpts, z, hz, -, htr, -⟩ :=
      (evaluate_eq_stopZone_iff cfg st now scanTf zoneTfs k).mp h
    rw [not_triggered_of_no_shape cfg _ k z0 z h0 ht hn' hz pts] at htr
    exact Bool.false_ne_true htr
  · intro r h
    obtain ⟨pts, hpts, -, z, hz, -, htr, -⟩ :=
      (evaluate_eq_slowdown_iff cfg st now scanTf zoneTfs k r).mp h
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
  · obtain ⟨now, v, scanTf, zoneTfs, rfl, hp, hf⟩ := (processed_iff cfg st e).mp hproc
    obtain ⟨h1, h2⟩ := evaluate_of_no_shape cfg st now scanTf zoneTfs k z0 h0 ht hn
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

/-- What every accepted shape satisfies: at least three vertices, as many
current vertices as received ones, all within the coordinate limit of
`Shape.ok`. -/
def ShapeOk (sh : TopicShape) : Prop :=
  3 ≤ sh.raw.length ∧ sh.current.length = sh.raw.length ∧
    sh.current.all (·.inRange (coordLimit - 3)) = true

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

theorem shapeOk_of_place (verts : List (Nat × Nat)) (foreign : Bool) (t : Xform) (sh : TopicShape)
    (hlen : 3 ≤ verts.length) (h : TopicShape.place verts foreign t = some sh) : ShapeOk sh := by
  obtain ⟨h1, rfl⟩ := (place_eq_some_iff verts foreign t sh).mp h
  refine ⟨hlen, by simp, ?_⟩
  simpa [List.all_eq_true] using h1

theorem shapeOk_of_accept (cfg : Config) (st : State) (k : Nat) (verts : List (Nat × Nat))
    (place : Placement) (sh : TopicShape) (h : acceptShape cfg st k verts place = some sh) :
    ShapeOk sh := by
  obtain ⟨-, -, hlen, -, t, f, -, h1, rfl⟩ := (acceptShape_eq_some_iff cfg st k verts place sh).mp h
  refine ⟨hlen, by simp, ?_⟩
  simpa [List.all_eq_true] using h1

theorem shapeOk_refresh (sh : TopicShape) (tf : Option Xform) (h : ShapeOk sh) :
    ShapeOk (sh.refresh tf) := by
  unfold TopicShape.refresh
  split
  · split
    · split
      · dsimp only
        split
        · rename_i hall
          exact ⟨h.1, by simp, hall⟩
        · exact h
      · exact h
    · exact h
  · exact h

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
  | shape k verts place =>
    cases ha : acceptShape cfg st k verts place with
    | none => rw [step_shape_of_refused _ _ _ _ _ ha]; exact h
    | some sh =>
      rw [step_shape_of_accepted _ _ _ _ _ _ ha]
      obtain ⟨hp, ⟨z, hz, hzt⟩, -⟩ := (acceptShape_eq_some_iff cfg st k verts place sh).mp ha
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
          exact shapeOk_of_accept cfg st k verts place sh ha
        · rw [List.getElem?_set_ne hjk] at hj
          exact h.ok j sh' hj
  | cmd now v scanTf zoneTfs =>
    by_cases hp : st.phase = .active
    · cases hf : v.finite
      · rw [step_cmd_of_not_finite _ _ _ _ _ _ hf]; exact h
      · rw [step_cmd _ _ _ _ _ _ hp hf]
        have key : ∀ (k : Nat) (sh : TopicShape),
            (shapesAfter cfg st now scanTf zoneTfs)[k]? = some (some sh) →
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
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hp : st.phase = .inactive)
    (hq : st.prevZero = false) (hr : st.prevReport = .normal) (hf : v.finite = true) :
    (step cfg st .activate).1.phase = .active ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanTf zoneTfs)).2.velocity =
        some (evaluate cfg st now scanTf zoneTfs) ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanTf zoneTfs)).2.report =
        if (evaluate cfg st now scanTf zoneTfs).report v = .normal then none
        else some ((evaluate cfg st now scanTf zoneTfs).report v) := by
  have hst : (step cfg st .activate).1 = { st with phase := .active } := by
    rw [step_activate, if_pos hp]
  have he : evaluate cfg { st with phase := .active } now scanTf zoneTfs =
      evaluate cfg st now scanTf zoneTfs := evaluate_congr _ _ _ _ _ _ rfl rfl
  rw [hst]
  refine ⟨rfl, ?_, ?_⟩
  · rw [velocity_of_not_prevZero _ _ _ _ _ _ rfl hf (by exact hq), he]
  · rw [report_cmd _ _ _ _ _ _ rfl hf, he]
    simp only [hr]

/-- The same for a reachable state: the invariant supplies the two
hypotheses on the record. -/
theorem after_activation_of_inv (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (scanTf : Option Xform) (zoneTfs : List (Option Xform)) (hinv : Inv cfg st)
    (hp : st.phase = .inactive) (hf : v.finite = true) :
    (step cfg st .activate).1.phase = .active ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanTf zoneTfs)).2.velocity =
        some (evaluate cfg st now scanTf zoneTfs) ∧
      (step cfg (step cfg st .activate).1 (.cmd now v scanTf zoneTfs)).2.report =
        if (evaluate cfg st now scanTf zoneTfs).report v = .normal then none
        else some ((evaluate cfg st now scanTf zoneTfs).report v) :=
  after_activation cfg st now v scanTf zoneTfs hp (hinv.quiet (Or.inr hp)).1
    (hinv.quiet (Or.inr hp)).2 hf

/-! ## Saturated points and topic zones (D12) -/

/-- **Every zone in force lies within the coordinate limit.** In a state
that satisfies the invariants, with an accepted configuration, every zone
that is tested for a command (a fixed one, or a topic zone with its shape,
refreshed) lies within the coordinate limit. Hence a saturated point, and
`Point.far`, is in none of them (`contains_eq_false_of_far'`), and such
points do not change the action (`zonesAction_filter_inRange'`). -/
theorem liveConfig_bounded (cfg : Config) (st : State) (zoneTfs : List (Option Xform))
    (hinv : Inv cfg st) (hok : cfg.ok = true) : ZonesBounded (liveConfig cfg st zoneTfs) := by
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
    cases hsh : (refreshShapes cfg st.shapes zoneTfs).getD k none with
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
      have hget : (refreshShapes cfg st.shapes zoneTfs)[k]? = some (some sh) := by
        rw [List.getD_eq_getElem?_getD] at hsh
        cases hg : (refreshShapes cfg st.shapes zoneTfs)[k]? with
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
          exact shapeBounded_polygon_of_all _ hsok.2.2

end ROSlean.CollisionMonitor
