import Mathlib.Tactic
import ROSleanCore.CollisionMonitorFFI
import ROSlean.CollisionMonitor.Convert
import ROSlean.CollisionMonitor.Properties
import ROSlean.CollisionMonitor.Trace
import ROSlean.CollisionMonitor.Transform

/-!
# The C interface computes the collision monitor model

The ROS node does not call `step` directly: it calls the exported wrappers in
`ROSleanCore.CollisionMonitorFFI`. These theorems tie the wrappers to the
model, so that properties proved about `step` and `run` are properties of what
the node executes.

* **Configuration.** The configuration the node runs with is what its builder
  calls say: zones in the order they were added, each with the action,
  threshold and flag of its call, a circle with the converted radius, and a
  polygon with the vertices of its `add_vertex` calls in order
  (`builderAddCircle_zones`, `foldl_addVertex`, `zoneOfRaw_fields`,
  `cmNew_eq`).
* **Refusals.** A monitor is ok only if the source was set, no raw value was
  refused, no vertex was given to no polygon, and `Config.ok` holds
  (`cmOk_iff`, `cmOk_cfg_ok`). The builder is fail-closed over any sequence
  of its five calls (`BuilderCall`): if any call, anywhere in the sequence,
  carries a refused value (a timeout, a radius, `min_points`, a vertex), the
  monitor is not ok, whatever the other calls are (`not_ok_of_refused_call`);
  so is it if a vertex is added when the zone added last is not a polygon
  (`not_ok_of_stray_vertex`), and if `set_source` was never called
  (`not_ok_without_set_source`). What is refused is characterised in
  `ROSlean.CollisionMonitor.Convert`.
* **Fail closed.** A monitor that is not ok answers every command with
  `stopSource`, after any sequence of calls (`apply_cmd_of_not_ok`,
  `foldl_exec_not_ok`).
* **Scan decoding.** The byte array of a scan is decoded by a total function
  into `size / 20` beams (`decodeBeams_length`, `getElem_decodeBeams`), every
  decoded pattern fits its field (`beamAt_lt`), and decoding inverts the
  encoding the shim performs (`decodeBeams_encodeBeams`). The coordinates
  are no longer converted here: they stay bit patterns until a command, and
  become points under the command's transform (`Xform.pointMm`;
  `Transform.pointMm_identity` says that the identity gives `coordMm`).
* **Events.** Each call is `step` on the event it delivers
  (`cmOnScan_eq`, `cmOnCmd_eq`, `exec_eq_apply`). The event of a command
  depends on whether the scan stored last was reported usable
  (`Call.event`, `events`): the identity transform if so, none if not. So
  after any sequence of calls, with no condition on them, the state is the
  one `run` reaches on the events they deliver (`foldl_exec_run`), from the
  state after `configure` and `activate` (`startState`, `run_startState`).
* **Read-back.** The three numbers the node reads after a command determine
  the velocity part of the model's output exactly (`readBack_eq`). The
  report and the difference between `silent` and a withheld velocity are
  not exposed by this interface yet (task T06).
* **End to end.** `cm_readBack_after`: create a monitor from a builder that is
  ok, make any sequence of calls of the two exported event functions (scans
  usable or not, commands), then deliver a command; what the three exported
  queries return is the velocity of the output of `step` for that command,
  in the state `run` reaches on the delivered events, on the configuration
  the builder holds. The statement is on the exported functions as the node
  calls them.

What remains outside Lean is the C++ shim: reading the parameters and the
clock, the floating-point work listed in the SPEC as T1 to T3, and acting on
the returned decision.

This file describes the interface as it stands after task T05: the calls of
the node of SPEC.md, read in the extended model. The calls of
SPEC_NAV2.md, section 8, and their theorems are task T06.
-/

namespace ROSlean.CollisionMonitor.FFI

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor.Convert

/-! ## Configuration -/

/-- A new builder: no zones, source enabled, both timeouts 0, and not
accepted, because the source was not set (code 5). -/
theorem builderNew_eq (u : Unit) :
    (builderNew u).cfg =
      { zones := [], sourceEnabled := true, sourceTimeout := 0, stopPubTimeout := 0 } ∧
    (builderNew u).error = 5 := ⟨rfl, rfl⟩

/-- `set_source` sets the source's flag and timeout, and nothing else of the
configuration. The timeout is the conversion of `sourceTimeoutNs`. -/
theorem builderSetSource_cfg (b : Builder) (enabled : UInt8) (node : UInt64) (hasOwn : UInt8)
    (own : UInt64) (t : Nat) (h : sourceTimeoutNs node.toNat (hasOwn != 0) own.toNat = some t) :
    (builderSetSource b enabled node hasOwn own).cfg =
      { b.cfg with sourceEnabled := enabled != 0, sourceTimeout := t } := by
  simp [builderSetSource, Builder.cfg, h]

/-- `set_stop_pub_timeout` sets that field and nothing else. -/
theorem builderSetStopPubTimeout_cfg (b : Builder) (timeout : UInt64) (t : Nat)
    (h : timeoutNs timeout.toNat = some t) :
    (builderSetStopPubTimeout b timeout).cfg = { b.cfg with stopPubTimeout := t } := by
  simp [builderSetStopPubTimeout, Builder.cfg, h]

/-- Action code 1 is `slowdown` with the ratio pattern as given. -/
theorem actionOfRaw_one (r : UInt64) : actionOfRaw 1 r = .slowdown r.toNat := rfl

/-- Every other action code is `stop`. -/
theorem actionOfRaw_of_ne_one (a : UInt8) (r : UInt64) (h : a ≠ 1) : actionOfRaw a r = .stop := by
  simp [actionOfRaw, h]

/-- The fields of a zone built from raw values, when `min_points` is
accepted. -/
theorem zoneOfRaw_fields (s : ZoneShape) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8) (n : Nat)
    (h : minPointsOfRaw mp.toInt = some n) :
    zoneOfRaw s a r mp en =
      ({ shape := s, action := actionOfRaw a r, minPoints := n, enabled := en != 0 }, 0) := by
  simp [zoneOfRaw, h]

/-- A refused `min_points` is recorded with code 4. -/
theorem zoneOfRaw_refused (s : ZoneShape) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8)
    (h : minPointsOfRaw mp.toInt = none) : (zoneOfRaw s a r mp en).2 = 4 := by
  simp [zoneOfRaw, h]

/-- `add_circle` appends one zone and changes nothing else: zones are indexed
in the order they are added. With accepted values the zone is the circle of
the converted radius. -/
theorem builderAddCircle_zones (b : Builder) (radius : UInt64) (a : UInt8) (r : UInt64)
    (mp : Int64) (en : UInt8) (rmm : Nat) (h : radiusMm radius.toNat = some rmm) :
    builderAddCircle b radius a r mp en =
      { b with zones := b.zones ++ [zoneOfRaw (.fixed (.circle rmm)) a r mp en] } := by
  simp [builderAddCircle, h]

/-- A refused radius is recorded with code 1. -/
theorem builderAddCircle_refused (b : Builder) (radius : UInt64) (a : UInt8) (r : UInt64)
    (mp : Int64) (en : UInt8) (h : radiusMm radius.toNat = none) :
    builderAddCircle b radius a r mp en =
      { b with zones := b.zones ++ [((zoneOfRaw (.fixed (.circle 0)) a r mp en).1, 1)] } := by
  simp [builderAddCircle, h]

/-- `add_polygon` appends one polygon zone without vertices. -/
theorem builderAddPolygon_zones (b : Builder) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8) :
    builderAddPolygon b a r mp en =
      { b with zones := b.zones ++ [zoneOfRaw (.fixed (.polygon [])) a r mp en] } := rfl

/-- If the zone added last is a polygon, an accepted vertex is appended to
it; the zones before it are untouched. -/
theorem addVertexToLast_polygon (p : Point) (zs : List (Zone × UInt8)) (ze : Zone × UInt8)
    (vs : List Point) (h : ze.1.shape = .fixed (.polygon vs)) :
    addVertexToLast (some p) (zs ++ [ze]) =
      zs ++ [({ ze.1 with shape := .fixed (.polygon (vs ++ [p])) }, ze.2)] := by
  induction zs with
  | nil => simp [addVertexToLast, h]
  | cons a as ih =>
    cases as with
    | nil => simpa [addVertexToLast] using ih
    | cons a' as' => simpa [addVertexToLast] using ih

/-- A refused vertex marks the polygon and leaves its vertices as they are. -/
theorem addVertexToLast_refused (zs : List (Zone × UInt8)) (ze : Zone × UInt8) (vs : List Point)
    (h : ze.1.shape = .fixed (.polygon vs)) :
    addVertexToLast none (zs ++ [ze]) = zs ++ [(ze.1, if ze.2 == 0 then 2 else ze.2)] := by
  induction zs with
  | nil => simp [addVertexToLast, h]
  | cons a as ih =>
    cases as with
    | nil => simpa [addVertexToLast] using ih
    | cons a' as' => simpa [addVertexToLast] using ih

/-- If the zone added last is not a fixed polygon (a circle, or a topic
zone), `add_vertex` has no effect. -/
theorem addVertexToLast_circle (v : Option Point) (zs : List (Zone × UInt8)) (ze : Zone × UInt8)
    (h : ∀ vs, ze.1.shape ≠ .fixed (.polygon vs)) :
    addVertexToLast v (zs ++ [ze]) = zs ++ [ze] := by
  induction zs with
  | nil =>
    simp only [List.nil_append, addVertexToLast]
  | cons a as ih =>
    cases as with
    | nil => simpa [addVertexToLast] using ih
    | cons a' as' => simpa [addVertexToLast] using ih

/-- With no zone, `add_vertex` has no effect. -/
theorem addVertexToLast_nil (v : Option Point) : addVertexToLast v [] = [] := rfl

/-- A list of zones that ends with a polygon. -/
theorem lastIsPolygon_concat (zs : List (Zone × UInt8)) (ze : Zone × UInt8) (vs : List Point)
    (h : ze.1.shape = .fixed (.polygon vs)) : lastIsPolygon (zs ++ [ze]) = true := by
  simp [lastIsPolygon, h]

/-- **A polygon is the vertices of its calls, in order.** `add_polygon`
followed by `add_vertex` calls whose values are all accepted appends one
polygon zone whose vertices are those of the calls, and leaves everything
else as it was (in particular, none of the vertices is a stray one). -/
theorem foldl_addVertex (b : Builder) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8)
    (ps : List (UInt32 × UInt32)) (vs : List Point)
    (h : ps.map (fun xy => vertexOfRaw xy.1 xy.2) = vs.map some) :
    ps.foldl (fun b xy => builderAddVertex b xy.1 xy.2) (builderAddPolygon b a r mp en) =
      { b with zones := b.zones ++ [zoneOfRaw (.fixed (.polygon vs)) a r mp en] } := by
  suffices key : ∀ (ps : List (UInt32 × UInt32)) (vs done : List Point) (b' : Builder),
      ps.map (fun xy => vertexOfRaw xy.1 xy.2) = vs.map some →
      b' = { b with zones := b.zones ++ [zoneOfRaw (.fixed (.polygon done)) a r mp en] } →
      ps.foldl (fun b xy => builderAddVertex b xy.1 xy.2) b' =
        { b with zones := b.zones ++ [zoneOfRaw (.fixed (.polygon (done ++ vs))) a r mp en] } by
    simpa using key ps vs [] _ h (builderAddPolygon_zones b a r mp en)
  intro ps
  induction ps with
  | nil =>
    intro vs done b' h hb
    cases vs with
    | nil => simpa using hb
    | cons v vs => simp at h
  | cons xy ps ih =>
    intro vs done b' h hb
    cases vs with
    | nil => simp at h
    | cons v vs =>
      simp only [List.map_cons, List.cons.injEq] at h
      rw [List.foldl_cons]
      have := ih vs (done ++ [v]) (builderAddVertex b' xy.1 xy.2) h.2 (by
        rw [hb]
        simp only [builderAddVertex, h.1]
        rw [addVertexToLast_polygon v b.zones _ done rfl, lastIsPolygon_concat _ _ done rfl]
        simp [zoneOfRaw])
      simpa using this

/-! ## Refusals -/

/-- A monitor starts in the active phase (`startState`), with the builder's
configuration and output `silent`; it is ok iff the builder reports no
error. -/
theorem cmNew_eq (b : Builder) :
    cmNew b = { cfg := b.cfg, ok := b.error == 0, err := b.error,
                zoneErrs := b.zones.map entryError, st := startState b.cfg, out := .silent,
                placed := true } := rfl

/-- The state a monitor starts in is the initial state with the phase
`active`: nothing else is touched by `configure` and `activate`. -/
theorem startState_eq (cfg : Config) :
    startState cfg = { State.init cfg with phase := .active } := rfl

/-- A run from the state a monitor starts in is a run from start-up that
begins with `configure` and `activate`: the theorems of `Trace.lean` about
runs from `State.init` apply to the node with this prefix. -/
theorem run_startState (cfg : Config) (es : List Event) :
    run cfg (startState cfg) es =
      ((run cfg (State.init cfg) ([.configure, .activate] ++ es)).1,
        (run cfg (State.init cfg) ([.configure, .activate] ++ es)).2.drop 2) := by
  rw [run_append]
  rfl

theorem zoneError_eq_zero_iff (z : Zone) : zoneError z = 0 ↔ z.ok = true := by
  unfold zoneError Zone.ok
  cases hsh : z.shape with
  | topic => cases ha : z.action.ok <;> simp [ZoneShape.ok]
  | fixed s =>
    cases hs : (ZoneShape.fixed s).ok <;> cases ha : z.action.ok <;> cases s <;> simp

theorem entryError_eq_zero_iff (ze : Zone × UInt8) :
    entryError ze = 0 ↔ ze.2 = 0 ∧ ze.1.ok = true := by
  unfold entryError
  by_cases h : ze.2 = 0
  · simp [h, zoneError_eq_zero_iff]
  · simp [h]

/-- **What "ok" means.** The builder reports no error iff no timeout was
refused, the source was set, no vertex was given to no polygon, no zone
carries a refused value, and every zone passes `Zone.ok`. -/
theorem error_eq_zero_iff (b : Builder) :
    b.error = 0 ↔
      b.sourceErr = 0 ∧ b.stopPubBad = false ∧ b.sourceSet = true ∧ b.stray = false ∧
        ∀ ze ∈ b.zones, ze.2 = 0 ∧ ze.1.ok = true := by
  have key : (b.zones.all fun ze => entryError ze == 0) = true ↔
      ∀ ze ∈ b.zones, ze.2 = 0 ∧ ze.1.ok = true := by
    simp only [List.all_eq_true, beq_iff_eq, entryError_eq_zero_iff]
  unfold Builder.error
  by_cases h1 : b.sourceErr = 0
  · cases h2 : b.stopPubBad
    · cases h4 : b.sourceSet
      · simp [h1]
      · cases h5 : b.stray
        · by_cases h3 : (b.zones.all fun ze => entryError ze == 0) = true
          · simpa [h1, h3] using key.mp h3
          · have := mt key.mpr h3
            simpa [h1, h3] using this
        · simp [h1]
    · simp [h1]
  · simp [h1]

theorem cmOk_iff (b : Builder) :
    cmOk (cmNew b) = true ↔
      b.sourceErr = 0 ∧ b.stopPubBad = false ∧ b.sourceSet = true ∧ b.stray = false ∧
        ∀ ze ∈ b.zones, ze.2 = 0 ∧ ze.1.ok = true := by
  rw [← error_eq_zero_iff]; simp [cmOk, cmNew]

/-- A monitor that is ok has a configuration that satisfies `Config.ok`: the
hypothesis of the geometric facts of `Geometry.lean` (a saturated point is in
no zone). -/
theorem cmOk_cfg_ok (b : Builder) (h : cmOk (cmNew b) = true) : (cmNew b).cfg.ok = true := by
  obtain ⟨-, -, -, -, hz⟩ := (cmOk_iff b).mp h
  simp only [cmNew, Builder.cfg, Config.ok, List.all_map, List.all_eq_true, Function.comp]
  exact fun ze hze => (hz ze hze).2

/-- A zone that carries a refused value, or fails `Zone.ok`, makes the monitor
not ok. -/
theorem not_ok_of_entry (b : Builder) (ze : Zone × UInt8) (hm : ze ∈ b.zones)
    (h : ze.2 ≠ 0 ∨ ze.1.ok = false) : cmOk (cmNew b) = false := by
  by_contra hc
  obtain ⟨-, -, -, -, hz⟩ := (cmOk_iff b).mp (by simpa using hc)
  rcases h with h | h
  · exact h (hz ze hm).1
  · rw [(hz ze hm).2] at h; cases h

/-- A refused source timeout (the node's or the source's own) is recorded,
whatever else the builder holds. Without this, a negative `source_timeout`
would become 0, "check disabled". -/
theorem not_ok_of_source_timeout (b : Builder) (enabled : UInt8) (node : UInt64) (hasOwn : UInt8)
    (own : UInt64)
    (h : timeoutNs node.toNat = none ∨ (hasOwn ≠ 0 ∧ timeoutNs own.toNat = none)) :
    (builderSetSource b enabled node hasOwn own).sourceErr ≠ 0 := by
  unfold builderSetSource
  by_cases hb : b.sourceErr = 0
  · rcases h with h | ⟨h1, h2⟩
    · simp [hb, h]
    · cases hn : timeoutNs node.toNat <;> simp [hb, h1, h2]
  · simp [hb]

theorem not_ok_of_stop_pub_timeout (b : Builder) (timeout : UInt64)
    (h : timeoutNs timeout.toNat = none) : (builderSetStopPubTimeout b timeout).stopPubBad = true := by
  simp [builderSetStopPubTimeout, h]

/-- The zone that `add_circle` appends has a non-zero error code if its radius
or its `min_points` is refused. -/
theorem entryError_addCircle_ne_zero (b : Builder) (radius : UInt64) (a : UInt8) (r : UInt64)
    (mp : Int64) (en : UInt8)
    (h : radiusMm radius.toNat = none ∨ minPointsOfRaw mp.toInt = none) :
    ∃ ze, (builderAddCircle b radius a r mp en).zones = b.zones ++ [ze] ∧ ze.2 ≠ 0 := by
  unfold builderAddCircle
  cases hr : radiusMm radius.toNat with
  | none => exact ⟨((zoneOfRaw (.fixed (.circle 0)) a r mp en).1, 1), by simp, by simp⟩
  | some v =>
    have hm : minPointsOfRaw mp.toInt = none := by
      rcases h with h | h
      · rw [hr] at h; cases h
      · exact h
    exact ⟨zoneOfRaw (.fixed (.circle v)) a r mp en, by simp, by simp [zoneOfRaw, hm]⟩

/-- A refused vertex leaves a non-zero code on the polygon it was meant for,
and later vertices do not clear it. -/
theorem addVertexToLast_code (v : Option Point) (zs : List (Zone × UInt8)) (ze : Zone × UInt8)
    (h : ze.2 ≠ 0) :
    ∃ ze', addVertexToLast v (zs ++ [ze]) = zs ++ [ze'] ∧ ze'.2 ≠ 0 := by
  cases hs : ze.1.shape with
  | topic => exact ⟨ze, addVertexToLast_circle v zs ze (by simp [hs]), h⟩
  | fixed s =>
    cases s with
    | circle r => exact ⟨ze, addVertexToLast_circle v zs ze (by simp [hs]), h⟩
    | polygon vs =>
      cases v with
      | some p => exact ⟨_, addVertexToLast_polygon p zs ze vs hs, h⟩
      | none => exact ⟨_, addVertexToLast_refused zs ze vs hs, by simp [h]⟩

theorem addVertexToLast_none_code (zs : List (Zone × UInt8)) (ze : Zone × UInt8) (vs : List Point)
    (hs : ze.1.shape = .fixed (.polygon vs)) :
    ∃ ze', addVertexToLast none (zs ++ [ze]) = zs ++ [ze'] ∧ ze'.2 ≠ 0 := by
  refine ⟨_, addVertexToLast_refused zs ze vs hs, ?_⟩
  by_cases h : ze.2 = 0 <;> simp [h]

/-! ### The builder is fail-closed over any sequence of calls -/

/-- One call of the C interface that changes a builder, with its arguments. -/
inductive BuilderCall where
  | setSource (enabled : UInt8) (nodeTimeout : UInt64) (hasOwn : UInt8) (ownTimeout : UInt64)
  | setStopPubTimeout (timeout : UInt64)
  | addCircle (radius : UInt64) (action : UInt8) (ratio : UInt64) (minPoints : Int64)
      (enabled : UInt8)
  | addPolygon (action : UInt8) (ratio : UInt64) (minPoints : Int64) (enabled : UInt8)
  | addVertex (x y : UInt32)

/-- Performs the call. -/
def BuilderCall.exec (b : Builder) : BuilderCall → Builder
  | .setSource en node hasOwn own => builderSetSource b en node hasOwn own
  | .setStopPubTimeout t => builderSetStopPubTimeout b t
  | .addCircle radius a r mp en => builderAddCircle b radius a r mp en
  | .addPolygon a r mp en => builderAddPolygon b a r mp en
  | .addVertex x y => builderAddVertex b x y

/-- The call carries a value that the conversions refuse: a timeout, a
radius, `min_points`, or a coordinate of a vertex. -/
def BuilderCall.Refused : BuilderCall → Prop
  | .setSource _ node hasOwn own =>
    timeoutNs node.toNat = none ∨ (hasOwn ≠ 0 ∧ timeoutNs own.toNat = none)
  | .setStopPubTimeout t => timeoutNs t.toNat = none
  | .addCircle radius _ _ mp _ => radiusMm radius.toNat = none ∨ minPointsOfRaw mp.toInt = none
  | .addPolygon _ _ mp _ => minPointsOfRaw mp.toInt = none
  | .addVertex x y => vertexOfRaw x y = none

/-- The builder carries the mark of a refusal: a refused timeout, a vertex
given to no polygon, or a zone with a non-zero code. -/
def Marked (b : Builder) : Prop :=
  b.sourceErr ≠ 0 ∨ b.stopPubBad = true ∨ b.stray = true ∨ ∃ ze ∈ b.zones, ze.2 ≠ 0

/-- A marked builder gives a monitor that is not ok. -/
theorem not_ok_of_marked (b : Builder) (h : Marked b) : cmOk (cmNew b) = false := by
  by_contra hc
  obtain ⟨h1, h2, -, h4, hz⟩ := (cmOk_iff b).mp (by simpa using hc)
  rcases h with h | h | h | ⟨ze, hm, h⟩
  · exact h h1
  · rw [h2] at h; cases h
  · rw [h4] at h; cases h
  · exact h (hz ze hm).1

/-- `add_vertex` clears no code. -/
theorem addVertexToLast_keeps_code (v : Option Point) :
    ∀ zs : List (Zone × UInt8), (∃ ze ∈ zs, ze.2 ≠ 0) → ∃ ze ∈ addVertexToLast v zs, ze.2 ≠ 0
  | [], h => by simpa using h
  | [ze], h => by
    obtain ⟨ze', hm, hne⟩ := h
    obtain rfl : ze' = ze := by simpa using hm
    obtain ⟨ze'', h1, h2⟩ := addVertexToLast_code v [] ze' hne
    exact ⟨ze'', by simp only [List.nil_append] at h1; simp [h1], h2⟩
  | ze :: ze2 :: zes, h => by
    have hrec : addVertexToLast v (ze :: ze2 :: zes) = ze :: addVertexToLast v (ze2 :: zes) := by
      simp [addVertexToLast]
    rw [hrec]
    obtain ⟨z, hm, hne⟩ := h
    rcases List.mem_cons.mp hm with rfl | hm
    · exact ⟨_, List.mem_cons_self, hne⟩
    · obtain ⟨z', hm', hne'⟩ := addVertexToLast_keeps_code v (ze2 :: zes) ⟨z, hm, hne⟩
      exact ⟨z', List.mem_cons_of_mem _ hm', hne'⟩

/-- If the zone added last is a polygon, the list ends with it. -/
theorem lastIsPolygon_iff (zs : List (Zone × UInt8)) :
    lastIsPolygon zs = true ↔ ∃ ys ze vs, zs = ys ++ [ze] ∧ ze.1.shape = .fixed (.polygon vs) := by
  unfold lastIsPolygon
  constructor
  · intro h
    cases hl : zs.getLast? with
    | none => simp [hl] at h
    | some ze =>
      obtain ⟨ys, rfl⟩ := List.getLast?_eq_some_iff.mp hl
      cases hs : ze.1.shape with
      | topic => simp [hs] at h
      | fixed s =>
        cases s with
        | circle r => simp [hs] at h
        | polygon vs => exact ⟨ys, ze, vs, rfl, hs⟩
  · rintro ⟨ys, ze, vs, rfl, hs⟩
    simp [hs]

/-- No call removes the mark of a refusal. -/
theorem exec_marked_mono (b : Builder) (c : BuilderCall) (h : Marked b) : Marked (c.exec b) := by
  rcases h with h | h | h | ⟨ze, hm, h⟩
  · cases c <;> first
      | exact .inl h
      | exact .inl (by simp [BuilderCall.exec, builderSetSource, h])
  · cases c <;> first
      | exact .inr (.inl h)
      | exact .inr (.inl (by simp [BuilderCall.exec, builderSetStopPubTimeout, h]))
  · cases c <;> first
      | exact .inr (.inr (.inl h))
      | exact .inr (.inr (.inl (by simp [BuilderCall.exec, builderAddVertex, h])))
  · refine .inr (.inr (.inr ?_))
    cases c with
    | setSource => exact ⟨ze, hm, h⟩
    | setStopPubTimeout => exact ⟨ze, hm, h⟩
    | addCircle => exact ⟨ze, by simp [BuilderCall.exec, builderAddCircle, hm], h⟩
    | addPolygon => exact ⟨ze, by simp [BuilderCall.exec, builderAddPolygon, hm], h⟩
    | addVertex x y => exact addVertexToLast_keeps_code _ _ ⟨ze, hm, h⟩

/-- A call that carries a refused value marks the builder, whatever the
builder holds. For a vertex: on its polygon if the zone added last is one,
and as a stray vertex otherwise. -/
theorem exec_marked_of_refused (b : Builder) (c : BuilderCall) (h : c.Refused) :
    Marked (c.exec b) := by
  cases c with
  | setSource en node hasOwn own => exact .inl (not_ok_of_source_timeout b en node hasOwn own h)
  | setStopPubTimeout t => exact .inr (.inl (not_ok_of_stop_pub_timeout b t h))
  | addCircle radius a r mp en =>
    obtain ⟨ze, h1, h2⟩ := entryError_addCircle_ne_zero b radius a r mp en h
    exact .inr (.inr (.inr ⟨ze, by simp [BuilderCall.exec, h1], h2⟩))
  | addPolygon a r mp en =>
    exact .inr (.inr (.inr ⟨zoneOfRaw (.fixed (.polygon [])) a r mp en,
      by simp [BuilderCall.exec, builderAddPolygon], by rw [zoneOfRaw_refused _ _ _ _ _ h]; decide⟩))
  | addVertex x y =>
    have hv : vertexOfRaw x y = none := h
    by_cases hl : lastIsPolygon b.zones = true
    · obtain ⟨ys, ze, vs, hzs, hs⟩ := (lastIsPolygon_iff _).mp hl
      obtain ⟨ze', h1, h2⟩ := addVertexToLast_none_code ys ze vs hs
      exact .inr (.inr (.inr ⟨ze', by simp [BuilderCall.exec, builderAddVertex, hv, hzs, h1], h2⟩))
    · exact .inr (.inr (.inl (by simp [BuilderCall.exec, builderAddVertex, hl])))

/-- A vertex added when the zone added last is not a polygon, or when there
is no zone, marks the builder, whether its value is accepted or not. -/
theorem exec_marked_of_stray (b : Builder) (x y : UInt32) (h : lastIsPolygon b.zones = false) :
    Marked (BuilderCall.exec b (.addVertex x y)) :=
  .inr (.inr (.inl (by simp [BuilderCall.exec, builderAddVertex, h])))

theorem foldl_exec_marked_mono (b : Builder) (cs : List BuilderCall) (h : Marked b) :
    Marked (cs.foldl BuilderCall.exec b) := by
  induction cs generalizing b with
  | nil => exact h
  | cons c cs ih => exact ih _ (exec_marked_mono b c h)

/-- **No refused value is dropped, wherever it occurs.** Start from any
builder and make any sequence of builder calls. If any one of them carries a
refused value (a timeout that is negative, not finite or too large; a radius
that is not finite or too large; a `min_points` outside the 32-bit range; a
vertex with a coordinate that is not finite), the monitor made from the
result is not ok, whatever the calls before and after it are: no later call
clears the refusal, and a refused vertex counts also when the zone added last
is a circle or when there is no zone. -/
theorem not_ok_of_refused_call (b : Builder) (cs : List BuilderCall)
    (h : ∃ c ∈ cs, c.Refused) : cmOk (cmNew (cs.foldl BuilderCall.exec b)) = false := by
  apply not_ok_of_marked
  obtain ⟨c, hc, hr⟩ := h
  obtain ⟨pre, post, rfl⟩ := List.append_of_mem hc
  rw [List.foldl_append, List.foldl_cons]
  exact foldl_exec_marked_mono _ _ (exec_marked_of_refused _ c hr)

/-- **A vertex that belongs to no polygon is not dropped either.** If
somewhere in the sequence a vertex is added while the zone added last is not
a polygon (or there is none), the monitor is not ok. -/
theorem not_ok_of_stray_vertex (b : Builder) (pre post : List BuilderCall) (x y : UInt32)
    (h : lastIsPolygon (pre.foldl BuilderCall.exec b).zones = false) :
    cmOk (cmNew ((pre ++ .addVertex x y :: post).foldl BuilderCall.exec b)) = false := by
  apply not_ok_of_marked
  rw [List.foldl_append, List.foldl_cons]
  exact foldl_exec_marked_mono _ _ (exec_marked_of_stray _ x y h)

/-- Only `set_source` sets the source. -/
theorem foldl_exec_sourceSet (b : Builder) (cs : List BuilderCall)
    (h : ∀ c ∈ cs, ∀ en node hasOwn own, c ≠ .setSource en node hasOwn own) :
    (cs.foldl BuilderCall.exec b).sourceSet = b.sourceSet := by
  induction cs generalizing b with
  | nil => rfl
  | cons c cs ih =>
    rw [List.foldl_cons, ih _ (fun c hc => h c (List.mem_cons_of_mem _ hc))]
    have := h c List.mem_cons_self
    cases c with
    | setSource en node hasOwn own => exact absurd rfl (this en node hasOwn own)
    | setStopPubTimeout => rfl
    | addCircle => rfl
    | addPolygon => rfl
    | addVertex => rfl

/-- **Not ok until the source is set.** A monitor made from a new builder by
calls among which there is no `set_source` is not ok, whatever the calls are:
the default source timeout of 0 ("check off") is never in force by
omission. -/
theorem not_ok_without_set_source (u : Unit) (cs : List BuilderCall)
    (h : ∀ c ∈ cs, ∀ en node hasOwn own, c ≠ .setSource en node hasOwn own) :
    cmOk (cmNew (cs.foldl BuilderCall.exec (builderNew u))) = false := by
  by_contra hc
  obtain ⟨-, -, h3, -⟩ := (cmOk_iff _).mp (by simpa using hc)
  rw [foldl_exec_sourceSet _ _ h] at h3
  cases h3

/-- **A refused vertex is never dropped silently**, the case of one polygon:
if any of the `add_vertex` calls after `add_polygon` carries a refused value,
the polygon ends with a non-zero error code (which names it in the node's
message). That the monitor is then not ok, here and for a vertex anywhere
else in the call sequence, is `not_ok_of_refused_call`. -/
theorem foldl_addVertex_refused (b : Builder) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8)
    (ps : List (UInt32 × UInt32)) (h : ∃ xy ∈ ps, vertexOfRaw xy.1 xy.2 = none) :
    ∃ ze, (ps.foldl (fun b xy => builderAddVertex b xy.1 xy.2)
      (builderAddPolygon b a r mp en)).zones = b.zones ++ [ze] ∧ ze.2 ≠ 0 := by
  -- Invariant: the zones are `b.zones ++ [ze]` with `ze` a polygon, or with a non-zero code.
  suffices key : ∀ (ps : List (UInt32 × UInt32)) (b' : Builder) (ze : Zone × UInt8),
      b'.zones = b.zones ++ [ze] →
      (ze.2 ≠ 0 ∨ ((∃ vs, ze.1.shape = .fixed (.polygon vs)) ∧ ∃ xy ∈ ps, vertexOfRaw xy.1 xy.2 = none)) →
      ∃ ze', (ps.foldl (fun b xy => builderAddVertex b xy.1 xy.2) b').zones = b.zones ++ [ze'] ∧
        ze'.2 ≠ 0 by
    exact key ps _ _ rfl (.inr ⟨⟨[], rfl⟩, h⟩)
  intro ps
  induction ps with
  | nil =>
    intro b' ze hb hc
    rcases hc with hc | ⟨-, xy, hxy, -⟩
    · exact ⟨ze, hb, hc⟩
    · cases hxy
  | cons xy ps ih =>
    intro b' ze hb hc
    rw [List.foldl_cons]
    rcases hc with hc | ⟨⟨vs, hvs⟩, xy', hxy', hnone⟩
    · obtain ⟨ze', h1, h2⟩ := addVertexToLast_code (vertexOfRaw xy.1 xy.2) b.zones ze hc
      exact ih _ ze' (by simp only [builderAddVertex, hb, h1]) (.inl h2)
    · cases hv : vertexOfRaw xy.1 xy.2 with
      | none =>
        obtain ⟨ze', h1, h2⟩ := addVertexToLast_none_code b.zones ze vs hvs
        exact ih _ ze' (by simp only [builderAddVertex, hb, hv, h1]) (.inl h2)
      | some p =>
        have h1 := addVertexToLast_polygon p b.zones ze vs hvs
        refine ih _ ({ ze.1 with shape := .fixed (.polygon (vs ++ [p])) }, ze.2)
          (by simp only [builderAddVertex, hb, hv, h1]) (.inr ⟨⟨_, rfl⟩, ?_⟩)
        rcases List.mem_cons.mp hxy' with rfl | hin
        · rw [hv] at hnone; cases hnone
        · exact ⟨xy', hin, hnone⟩

/-- A vertex is refused iff one of its two numbers is not finite. -/
theorem vertexOfRaw_eq_none_iff (x y : UInt32) :
    vertexOfRaw x y = none ↔ F32.finite x.toNat = false ∨ F32.finite y.toNat = false := by
  unfold vertexOfRaw
  have hx := vertexMm_isSome_iff x.toNat
  have hy := vertexMm_isSome_iff y.toNat
  cases h1 : vertexMm x.toNat <;> cases h2 : vertexMm y.toNat <;>
    cases h3 : F32.finite x.toNat <;> cases h4 : F32.finite y.toNat <;> simp_all

/-! ## Scan decoding -/

theorem byteAt_lt (a : ByteArray) (i : Nat) : byteAt a i < 256 := by
  unfold byteAt
  split
  · exact UInt8.toNat_lt _
  · decide

theorem u32At_lt (a : ByteArray) (o : Nat) : u32At a o < 2 ^ 32 := by
  have h0 := byteAt_lt a o
  have h1 := byteAt_lt a (o + 1)
  have h2 := byteAt_lt a (o + 2)
  have h3 := byteAt_lt a (o + 3)
  unfold u32At
  omega

theorem u64At_lt (a : ByteArray) (o : Nat) : u64At a o < 2 ^ 64 := by
  have h0 := u32At_lt a o
  have h1 := u32At_lt a (o + 4)
  unfold u64At
  omega

/-- Reading the two halves of a coordinate separately gives `coordMm` of the
whole 64-bit pattern. -/
theorem coordMmOfParts_halves (lo hi : Nat) (hlo : lo < 2 ^ 32) (hhi : hi < 2 ^ 32) :
    coordMmOfParts (decide (0x80000000 ≤ hi)) (hi % 0x80000000 * 0x100000000 + lo) =
      coordMm (hi * 0x100000000 + lo) := by
  unfold coordMm F64.neg
  have h1 : (hi * 0x100000000 + lo) % 0x8000000000000000 = hi % 0x80000000 * 0x100000000 + lo := by
    omega
  have h2 : ((hi * 0x100000000 + lo) / 0x8000000000000000 % 2 == 1) = decide (0x80000000 ≤ hi) := by
    by_cases h : 0x80000000 ≤ hi
    · have : (hi * 0x100000000 + lo) / 0x8000000000000000 % 2 = 1 := by omega
      simp [this, h]
    · have : (hi * 0x100000000 + lo) / 0x8000000000000000 % 2 = 0 := by omega
      simp [this, h]
  rw [h1, h2]

/-- Every decoded pattern fits its field, whatever the bytes: the range is a
32-bit pattern and the coordinates are 64-bit patterns. -/
theorem beamAt_lt (a : ByteArray) (i : Nat) :
    (beamAt a i).range < 2 ^ 32 ∧ (beamAt a i).x < 2 ^ 64 ∧ (beamAt a i).y < 2 ^ 64 :=
  ⟨u32At_lt .., u64At_lt .., u64At_lt ..⟩

/-- A byte array of any length is a scan of `size / 20` readings; a trailing
partial record is ignored. -/
theorem decodeBeams_length (a : ByteArray) : (decodeBeams a).length = a.size / 20 := by
  simp [decodeBeams, recordSize]

/-- Reading `i` of the decoded scan is record `i` of the array. -/
theorem getElem_decodeBeams (a : ByteArray) (i : Nat) (h : i < (decodeBeams a).length) :
    (decodeBeams a)[i] = beamAt a i := by
  simp [decodeBeams]

/-- An empty array is a scan without readings. -/
theorem decodeBeams_empty : decodeBeams ByteArray.empty = [] := rfl

/-- One reading as the shim hands it over: the range pattern (binary32) and
the patterns of the two coordinates in metres (binary64). -/
structure RawBeam where
  range : Nat
  x : Nat
  y : Nat

/-- The beam of the model that a raw reading denotes: the same three
patterns. -/
def RawBeam.beam (r : RawBeam) : Beam := { range := r.range, x := r.x, y := r.y }

/-- The patterns fit their fields. -/
def RawBeam.Fits (r : RawBeam) : Prop := r.range < 2 ^ 32 ∧ r.x < 2 ^ 64 ∧ r.y < 2 ^ 64

/-- The `k` little-endian bytes of a number. -/
def leBytes : Nat → Nat → List UInt8
  | 0, _ => []
  | k + 1, n => (n % 256).toUInt8 :: leBytes k (n / 256)

/-- The 20 bytes the shim writes for one reading (SPEC, "Scan event"). -/
def rawBytes (r : RawBeam) : List UInt8 := leBytes 4 r.range ++ leBytes 8 r.x ++ leBytes 8 r.y

/-- The byte array the shim builds for a scan. -/
def encodeBeams (rs : List RawBeam) : ByteArray := ⟨(rs.flatMap rawBytes).toArray⟩

theorem rawBytes_length (r : RawBeam) : (rawBytes r).length = 20 := rfl

theorem encodeBeams_size (rs : List RawBeam) : (encodeBeams rs).size = 20 * rs.length := by
  induction rs with
  | nil => rfl
  | cons b bs ih =>
    simp only [encodeBeams, ByteArray.size, List.flatMap_cons, List.size_toArray,
      List.length_append, rawBytes_length, List.length_cons] at ih ⊢
    omega

/-- A byte of an encoded scan, as a list lookup. -/
theorem byteAt_encodeBeams (rs : List RawBeam) (i : Nat) :
    byteAt (encodeBeams rs) i = (((rs.flatMap rawBytes)[i]?).map UInt8.toNat).getD 0 := by
  unfold byteAt
  by_cases h : i < (encodeBeams rs).size
  · have h' : i < (rs.flatMap rawBytes).length := by
      simpa [encodeBeams, ByteArray.size] using h
    rw [dif_pos h, List.getElem?_eq_getElem h']
    rfl
  · have h' : ¬ i < (rs.flatMap rawBytes).length := by
      simpa [encodeBeams, ByteArray.size] using h
    rw [dif_neg h, List.getElem?_eq_none (by omega)]
    rfl

/-- Byte `20 * i + j` of the encoding is byte `j` of record `i`. -/
theorem getElem?_flatMap_rawBytes (rs : List RawBeam) (i j : Nat) (hj : j < 20) :
    (rs.flatMap rawBytes)[20 * i + j]? = rs[i]?.bind (fun b => (rawBytes b)[j]?) := by
  induction rs generalizing i with
  | nil => simp
  | cons b bs ih =>
    rw [List.flatMap_cons]
    cases i with
    | zero =>
      rw [List.getElem?_append_left (by rw [rawBytes_length]; omega)]
      simp
    | succ i =>
      rw [List.getElem?_append_right (by rw [rawBytes_length]; omega), rawBytes_length]
      have : 20 * (i + 1) + j - 20 = 20 * i + j := by omega
      rw [this, ih]
      simp

theorem toNat_toUInt8_mod (n : Nat) : (n % 256).toUInt8.toNat = n % 256 := by
  simp [Nat.toUInt8]

/-- **Decoding inverts the encoding.** If the shim writes the records of the
SPEC for readings whose patterns fit their fields, the model sees exactly
those readings, in order: the range and the two coordinate patterns as
given, unconverted. -/
theorem decodeBeams_encodeBeams (rs : List RawBeam) (h : ∀ r ∈ rs, r.Fits) :
    decodeBeams (encodeBeams rs) = rs.map RawBeam.beam := by
  apply List.ext_getElem
  · rw [decodeBeams_length, encodeBeams_size, List.length_map]; omega
  · intro i h1 h2
    rw [List.length_map] at h2
    rw [getElem_decodeBeams, List.getElem_map]
    obtain ⟨hr, hx, hy⟩ := h rs[i] (List.getElem_mem h2)
    have hb : rs[i]? = some rs[i] := List.getElem?_eq_getElem h2
    have key : ∀ j, j < 20 → byteAt (encodeBeams rs) (20 * i + j) =
        (((rawBytes rs[i])[j]?).map UInt8.toNat).getD 0 := by
      intro j hj
      rw [byteAt_encodeBeams, getElem?_flatMap_rawBytes rs i j hj, hb]
      rfl
    have k0 := key 0 (by omega)
    have k1 := key 1 (by omega)
    have k2 := key 2 (by omega)
    have k3 := key 3 (by omega)
    have k4 := key 4 (by omega)
    have k5 := key 5 (by omega)
    have k6 := key 6 (by omega)
    have k7 := key 7 (by omega)
    have k8 := key 8 (by omega)
    have k9 := key 9 (by omega)
    have k10 := key 10 (by omega)
    have k11 := key 11 (by omega)
    have k12 := key 12 (by omega)
    have k13 := key 13 (by omega)
    have k14 := key 14 (by omega)
    have k15 := key 15 (by omega)
    have k16 := key 16 (by omega)
    have k17 := key 17 (by omega)
    have k18 := key 18 (by omega)
    have k19 := key 19 (by omega)
    simp only [rawBytes, leBytes, List.cons_append, List.nil_append, List.getElem?_cons_zero,
      List.getElem?_cons_succ, Option.map_some, Option.getD_some, Nat.add_zero,
      toNat_toUInt8_mod] at k0 k1 k2 k3 k4 k5 k6 k7 k8 k9 k10 k11 k12 k13 k14 k15 k16 k17 k18 k19
    have e0 : u32At (encodeBeams rs) (recordSize * i) = rs[i].range := by
      simp only [u32At, recordSize, k0, k1, k2, k3]
      omega
    have ex : u64At (encodeBeams rs) (recordSize * i + 4) = rs[i].x := by
      simp only [u64At, u32At, recordSize, Nat.add_assoc, Nat.reduceAdd, k4, k5, k6, k7, k8, k9,
        k10, k11]
      omega
    have ey : u64At (encodeBeams rs) (recordSize * i + 12) = rs[i].y := by
      simp only [u64At, u32At, recordSize, Nat.add_assoc, Nat.reduceAdd, k12, k13, k14, k15, k16,
        k17, k18, k19]
      omega
    simp only [beamAt, e0, ex, ey, RawBeam.beam]

/-- The scan the model sees: stamp (negative taken as 0, D8), the two limits
as given, and the decoded readings. -/
theorem scanOfRaw_fields (stamp : Int64) (rmin rmax : UInt32) (a : ByteArray) :
    (scanOfRaw stamp rmin rmax a).stamp = stamp.toInt.toNat ∧
    (scanOfRaw stamp rmin rmax a).rangeMin = rmin.toNat ∧
    (scanOfRaw stamp rmin rmax a).rangeMax = rmax.toNat ∧
    (scanOfRaw stamp rmin rmax a).beams = decodeBeams a := ⟨rfl, rfl, rfl, rfl⟩

/-! ## Events -/

theorem apply_cfg (m : Monitor) (e : Event) :
    (m.apply e).cfg = m.cfg ∧ (m.apply e).ok = m.ok ∧ (m.apply e).placed = m.placed := by
  unfold Monitor.apply
  split
  · exact ⟨rfl, rfl, rfl⟩
  · cases e <;> exact ⟨rfl, rfl, rfl⟩

/-- An ok monitor does what `step` says. -/
theorem apply_of_ok (m : Monitor) (e : Event) (h : m.ok = true) :
    (m.apply e).st = (step m.cfg m.st e).1 ∧ (m.apply e).out = (step m.cfg m.st e).2 := by
  unfold Monitor.apply
  rw [if_pos h]
  exact ⟨rfl, rfl⟩

/-- **Fail closed.** A monitor that is not ok answers every command in the
active phase with `stopSource`: whatever the command (finite or not), the
scans it was given, the transforms, and the time. -/
theorem apply_cmd_of_not_ok (m : Monitor) (now : Nat) (v : Twist) (scanTf : Option Xform)
    (zoneTfs : List (Option Xform)) (h : m.ok = false) (hp : m.st.phase = .active) :
    (m.apply (.cmd now v scanTf zoneTfs)).out = .answer (some .stopSource) none := by
  unfold Monitor.apply
  simp [h, hp]

/-- A monitor that is not ok keeps its state under scans, shape messages and
commands. -/
theorem apply_st_of_not_ok (m : Monitor) (e : Event) (h : m.ok = false)
    (he : (∃ now s, e = .scan now s) ∨ (∃ k verts place, e = .shape k verts place) ∨
      ∃ now v scanTf zoneTfs, e = .cmd now v scanTf zoneTfs) :
    (m.apply e).st = m.st := by
  unfold Monitor.apply
  rcases he with ⟨now, s, rfl⟩ | ⟨k, verts, place, rfl⟩ | ⟨now, v, scanTf, zoneTfs, rfl⟩ <;>
    simp [h]

/-- The scan call is `Monitor.apply` on a scan event, which stores the scan,
and records whether the scan was reported usable. -/
theorem cmOnScan_eq (m : Monitor) (now : UInt64) (stamp : Int64) (usable : UInt8)
    (rmin rmax : UInt32) (a : ByteArray) :
    cmOnScan m now stamp usable rmin rmax a =
      { m.apply (.scan now.toNat (scanOfRaw stamp rmin rmax a)) with placed := usable != 0 } :=
  rfl

/-- The command call is `Monitor.apply` on a command event with the six
patterns in the order `linear.x`, `linear.y`, `linear.z`, `angular.x`,
`angular.y`, `angular.z`, the identity transform for the scan if the scan
stored last was reported usable and none if it was not, and no zone
transforms. -/
theorem cmOnCmd_eq (m : Monitor) (now lx ly lz ax ay az : UInt64) :
    cmOnCmd m now lx ly lz ax ay az =
      m.apply (.cmd now.toNat ⟨lx.toNat, ly.toNat, lz.toNat, ax.toNat, ay.toNat, az.toNat⟩
        (if m.placed then some Xform.identity else none) []) := rfl

/-- **An unusable scan stops the robot by the source rule.** After a scan
call that the shim reports as unusable (D11), commands carry no transform
for the scan, and with an enabled source and a non-zero source timeout the
action demanded at every time, in every state, is `stopSource` (N27), until
a scan is reported usable. This is what the removed event `scanUnusable`
did. -/
theorem cmOnScan_unusable_stop (m : Monitor) (now : UInt64) (stamp : Int64) (rmin rmax : UInt32)
    (a : ByteArray) (he : m.cfg.sourceEnabled = true) (ht : 0 < m.cfg.sourceTimeout) :
    (cmOnScan m now stamp 0 rmin rmax a).placed = false ∧
      ∀ (st : State) (t : Nat) (lx ly lz ax ay az : UInt64),
        ∃ v, cmdEvent false (UInt64.ofNat t) lx ly lz ax ay az =
            .cmd (UInt64.ofNat t).toNat v none [] ∧
          evaluate m.cfg st (UInt64.ofNat t).toNat none [] = .stopSource := by
  refine ⟨rfl, fun st t lx ly lz ax ay az => ⟨_, rfl, ?_⟩⟩
  rw [evaluate_eq_stopSource_iff]
  exact ⟨he, ht, Or.inr (Or.inr noTransform_none)⟩

/-- After any sequence of events an ok monitor has the same configuration and
the state `run` reaches. -/
theorem foldl_apply (m : Monitor) (es : List Event) (h : m.ok = true) :
    (es.foldl Monitor.apply m).cfg = m.cfg ∧ (es.foldl Monitor.apply m).ok = true ∧
    (es.foldl Monitor.apply m).st = (run m.cfg m.st es).1 := by
  induction es generalizing m with
  | nil => exact ⟨rfl, h, rfl⟩
  | cons e es ih =>
    obtain ⟨h1, h2, h3⟩ := ih (m.apply e) (by rw [(apply_cfg m e).2.1]; exact h)
    rw [List.foldl_cons, h1, h3, (apply_cfg m e).1, (apply_of_ok m e h).1, run_cons]
    exact ⟨rfl, h2, rfl⟩

/-- One call of the C interface that delivers an event, with its arguments:
the two exported functions the node calls after start-up. -/
inductive Call where
  | scan (now : UInt64) (stamp : Int64) (usable : UInt8) (rangeMin rangeMax : UInt32)
      (beams : ByteArray)
  | cmd (now linX linY linZ angX angY angZ : UInt64)

/-- Performs the call. -/
def Call.exec (m : Monitor) : Call → Monitor
  | .scan now stamp usable rmin rmax a => cmOnScan m now stamp usable rmin rmax a
  | .cmd now lx ly lz ax ay az => cmOnCmd m now lx ly lz ax ay az

/-- The event of the model that a call delivers, given whether the scan
stored last was reported usable. A scan call delivers a `scan`; a command
call a `cmd` with the identity transform or none (`cmdEvent`). -/
def Call.event (placed : Bool) : Call → Event
  | .scan now stamp _ rmin rmax a => .scan now.toNat (scanOfRaw stamp rmin rmax a)
  | .cmd now lx ly lz ax ay az => cmdEvent placed now lx ly lz ax ay az

/-- Whether the scan stored last was reported usable, after the call. -/
def Call.placed (placed : Bool) : Call → Bool
  | .scan _ _ usable _ _ _ => usable != 0
  | .cmd _ _ _ _ _ _ _ => placed

/-- The events a sequence of calls delivers. -/
def events (placed : Bool) : List Call → List Event
  | [] => []
  | c :: cs => c.event placed :: events (c.placed placed) cs

/-- Whether the scan stored last was reported usable, after the calls. -/
def placedAfter (placed : Bool) (cs : List Call) : Bool :=
  cs.foldl (fun p c => c.placed p) placed

/-- Every call is `Monitor.apply` on the event it delivers, and records
whether the scan stored last is usable. -/
theorem exec_eq_apply (m : Monitor) (c : Call) :
    c.exec m = { m.apply (c.event m.placed) with placed := c.placed m.placed } := by
  cases c with
  | scan now stamp usable rmin rmax a => rfl
  | cmd now lx ly lz ax ay az =>
    show m.apply (cmdEvent m.placed now lx ly lz ax ay az) =
      { m.apply (cmdEvent m.placed now lx ly lz ax ay az) with placed := m.placed }
    have h := (apply_cfg m (cmdEvent m.placed now lx ly lz ax ay az)).2.2
    generalize m.apply (cmdEvent m.placed now lx ly lz ax ay az) = m' at h ⊢
    cases m'
    simp only at h
    subst h
    rfl

/-- No call changes the configuration or whether the monitor is ok. -/
theorem exec_cfg (m : Monitor) (c : Call) :
    (c.exec m).cfg = m.cfg ∧ (c.exec m).ok = m.ok ∧ (c.exec m).placed = c.placed m.placed := by
  rw [exec_eq_apply]
  exact ⟨(apply_cfg ..).1, (apply_cfg ..).2.1, rfl⟩

theorem foldl_exec_cfg (m : Monitor) (cs : List Call) :
    (cs.foldl Call.exec m).cfg = m.cfg ∧ (cs.foldl Call.exec m).ok = m.ok ∧
      (cs.foldl Call.exec m).placed = placedAfter m.placed cs := by
  induction cs generalizing m with
  | nil => exact ⟨rfl, rfl, rfl⟩
  | cons c cs ih =>
    rw [List.foldl_cons, (ih _).1, (ih _).2.1, (ih _).2.2, (exec_cfg m c).1, (exec_cfg m c).2.1,
      (exec_cfg m c).2.2]
    exact ⟨rfl, rfl, rfl⟩

/-- **Any sequence of calls is the sequence of its events.** After any
sequence of calls, with no condition on them (scans may be usable or not),
an ok monitor has the same configuration and the state `run` reaches on the
events the calls deliver. -/
theorem foldl_exec_run (m : Monitor) (cs : List Call) (h : m.ok = true) :
    (cs.foldl Call.exec m).cfg = m.cfg ∧ (cs.foldl Call.exec m).ok = true ∧
    (cs.foldl Call.exec m).st = (run m.cfg m.st (events m.placed cs)).1 := by
  induction cs generalizing m with
  | nil => exact ⟨rfl, h, rfl⟩
  | cons c cs ih =>
    obtain ⟨h1, h2, h3⟩ := ih (c.exec m) (by rw [(exec_cfg m c).2.1]; exact h)
    rw [List.foldl_cons, h1, h3, (exec_cfg m c).1, (exec_cfg m c).2.2]
    refine ⟨rfl, h2, ?_⟩
    have hst : (c.exec m).st = (step m.cfg m.st (c.event m.placed)).1 := by
      rw [exec_eq_apply]
      exact (apply_of_ok m _ h).1
    rw [hst, events, run_cons]

/-- A monitor that is not ok keeps its state under every call. -/
theorem exec_st_of_not_ok (m : Monitor) (c : Call) (h : m.ok = false) : (c.exec m).st = m.st := by
  rw [exec_eq_apply]
  cases c with
  | scan now stamp usable rmin rmax a => exact apply_st_of_not_ok m _ h (Or.inl ⟨_, _, rfl⟩)
  | cmd now lx ly lz ax ay az =>
    exact apply_st_of_not_ok m _ h (Or.inr (Or.inr ⟨_, _, _, _, rfl⟩))

/-- **Fail closed, on sequences.** If the monitor was not ok when it was
created, and is in the active phase (as every monitor of `cmNew` is), then
after any calls whatever, usable or not, the answer to a command is
`stopSource`. -/
theorem foldl_exec_not_ok (m : Monitor) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (h : m.ok = false) (hp : m.st.phase = .active) :
    (cmOnCmd (cs.foldl Call.exec m) now lx ly lz ax ay az).out =
      .answer (some .stopSource) none := by
  have hst : (cs.foldl Call.exec m).st = m.st := by
    induction cs generalizing m with
    | nil => rfl
    | cons c cs ih =>
      rw [List.foldl_cons, ih (c.exec m) (by rw [(exec_cfg m c).2.1]; exact h)
        (by rw [exec_st_of_not_ok m c h]; exact hp), exec_st_of_not_ok m c h]
  rw [cmOnCmd_eq]
  exact apply_cmd_of_not_ok _ _ _ _ _ (by rw [(foldl_exec_cfg m cs).2.1]; exact h)
    (by rw [hst]; exact hp)

/-! ## Reading the output back

The node reads three numbers. They determine the velocity part of the
output, because the zone index and the ratio are never truncated. -/

/-- The velocity action denoted by the three numbers the node reads; `none`
for kind 0 and for every unknown kind. -/
def actionOfCodes (kind : UInt8) (zone ratio : Nat) : Option Action :=
  if kind = 1 then some .pass
  else if kind = 2 then some (.slowdown zone ratio)
  else if kind = 3 then some (.stopZone zone)
  else if kind = 4 then some .stopSource
  else none

/-- The codes determine the velocity of the output. -/
theorem actionOfCodes_codes (o : Output) :
    actionOfCodes (kindOf o) (zoneOf o) (ratioOf o) = o.velocity := by
  unfold kindOf zoneOf ratioOf
  rcases o.velocity with _ | a
  · rfl
  · cases a <;> rfl

/-- A published stop names a configured zone. -/
theorem step_publish_stopZone (cfg : Config) (st : State) (e : Event) (k : Nat)
    (h : (step cfg st e).2.velocity = some (.stopZone k)) : k < cfg.zones.length := by
  obtain ⟨now, v, scanTf, zoneTfs, -, -, -, ha⟩ := step_velocity cfg st e _ h
  obtain ⟨pts, z, z0, -, -, hz, -⟩ := evaluate_stopZone_only_if cfg st now scanTf zoneTfs k ha.symm
  exact (List.getElem?_eq_some_iff.mp hz).1

/-- A published slowdown names a configured slowdown zone and carries that
zone's ratio, which is below the pattern of 1.0. -/
theorem step_publish_slowdown (cfg : Config) (st : State) (e : Event) (k r : Nat)
    (h : (step cfg st e).2.velocity = some (.slowdown k r)) :
    r < F64.one ∧ ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨now, v, scanTf, zoneTfs, -, -, -, ha⟩ := step_velocity cfg st e _ h
  exact evaluate_slowdown cfg st now scanTf zoneTfs k r ha.symm

/-- **The node reads back the model's velocity.** For an ok monitor with at
most 2³² zones, the kind, zone index and ratio that the three queries return
after an event determine the velocity of the output of `step` for that event
exactly: nothing is truncated on the way to C. (A published zone index is
below the number of zones, and a published ratio is below the pattern of
1.0.) Kind 0 is "no velocity": the event was not a processed command, or
the velocity is withheld (R15). -/
theorem readBack_eq (m : Monitor) (e : Event) (hok : m.ok = true)
    (hz : m.cfg.zones.length ≤ 2 ^ 32) :
    actionOfCodes (cmOutKind (m.apply e)) (cmOutZone (m.apply e)).toNat
      (cmOutRatio (m.apply e)).toNat = (step m.cfg m.st e).2.velocity := by
  have hzone : zoneOf (step m.cfg m.st e).2 < 2 ^ 32 := by
    unfold zoneOf
    rcases ho : (step m.cfg m.st e).2.velocity with _ | a
    · simp
    · cases a with
      | pass => simp
      | stopSource => simp
      | stopZone k => have := step_publish_stopZone _ _ _ _ ho; simp only; omega
      | slowdown k r =>
        obtain ⟨-, z, hk, -⟩ := step_publish_slowdown _ _ _ _ _ ho
        have := (List.getElem?_eq_some_iff.mp hk).1
        simp only; omega
  have hratio : ratioOf (step m.cfg m.st e).2 < 2 ^ 64 := by
    unfold ratioOf
    rcases ho : (step m.cfg m.st e).2.velocity with _ | a
    · simp
    · cases a with
      | pass => simp
      | stopSource => simp
      | stopZone k => simp
      | slowdown k r =>
        obtain ⟨hlt, -⟩ := step_publish_slowdown _ _ _ _ _ ho
        have h1 : F64.one < 2 ^ 64 := by unfold F64.one; norm_num
        simp only; omega
  simp only [cmOutKind, cmOutZone, cmOutRatio, (apply_of_ok m e hok).2]
  have e1 : (zoneOf (step m.cfg m.st e).2).toUInt32.toNat = zoneOf (step m.cfg m.st e).2 := by
    simp only [Nat.toUInt32, UInt32.toNat_ofNat']; omega
  have e2 : (ratioOf (step m.cfg m.st e).2).toUInt64.toNat = ratioOf (step m.cfg m.st e).2 := by
    simp only [Nat.toUInt64, UInt64.toNat_ofNat']; omega
  rw [e1, e2]
  exact actionOfCodes_codes _

/-! ## End to end

Stated on the exported functions in the order the node calls them: `cmNew` on
the builder, then `cmOnScan` and `cmOnCmd` in any order and number (`Call`,
`Call.exec`), then `cmOnCmd` for the command in question and the three
queries. The events of the model are those the calls deliver (`events`), from
the state after `configure` and `activate` (`startState`); by
`run_startState` the theorems of `Properties.lean`, `Lifecycle.lean` and
`Trace.lean`, which hold for every sequence of events, apply with
`es := [.configure, .activate] ++ events true cs`. -/

/-- **End to end.** Create a monitor from a builder that is ok, make any
sequence of calls of the two event functions (commands, usable scans and
unusable scans, in any order), then deliver a velocity command at node-clock
time `now`. The recorded output is the output of `step` for that command's
event, in the state `run` reaches from the start state on the events the
calls deliver, on the configuration the builder holds. -/
theorem cm_out_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hok : cmOk (cmNew b) = true) :
    (cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az).out =
      (step b.cfg (run b.cfg (startState b.cfg) (events true cs)).1
        (cmdEvent (placedAfter true cs) now lx ly lz ax ay az)).2 := by
  obtain ⟨h1, h2, h3⟩ := foldl_exec_run (cmNew b) cs hok
  have h4 := (foldl_exec_cfg (cmNew b) cs).2.2
  unfold cmOnCmd
  rw [(apply_of_ok _ _ h2).2, h1, h3, h4]
  rfl

/-- **End to end, as the node sees it.** For at most 2³² zones, the three
numbers the exported queries return after the command determine the velocity
of the model's output for it. -/
theorem cm_readBack_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hok : cmOk (cmNew b) = true) (hz : b.zones.length ≤ 2 ^ 32) :
    let m := cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az
    actionOfCodes (cmOutKind m) (cmOutZone m).toNat (cmOutRatio m).toNat =
      (step b.cfg (run b.cfg (startState b.cfg) (events true cs)).1
        (cmdEvent (placedAfter true cs) now lx ly lz ax ay az)).2.velocity := by
  intro m
  obtain ⟨h1, h2, h3⟩ := foldl_exec_run (cmNew b) cs hok
  have h4 := (foldl_exec_cfg (cmNew b) cs).2.2
  have h := readBack_eq (cs.foldl Call.exec (cmNew b))
    (cmdEvent (cs.foldl Call.exec (cmNew b)).placed now lx ly lz ax ay az) h2
    (by rw [h1]; simpa [cmNew, Builder.cfg] using hz)
  have hm : m = (cs.foldl Call.exec (cmNew b)).apply
      (cmdEvent (cs.foldl Call.exec (cmNew b)).placed now lx ly lz ax ay az) := rfl
  rw [hm, h, h1, h3, h4]
  rfl

/-- **End to end, every output.** The same for every call of the sequence,
not only for a command at its end: after the first `n + 1` calls the three
queries return the velocity of the output of `step` for the event of call
`n`, in the state `run` reaches on the events of the first `n` calls. (The
node reads the queries after commands only; for a scan call they say "no
velocity".) -/
theorem cm_readBack_at (b : Builder) (cs : List Call) (n : Nat) (c : Call)
    (hok : cmOk (cmNew b) = true) (hz : b.zones.length ≤ 2 ^ 32) (hc : cs[n]? = some c) :
    let m := (cs.take (n + 1)).foldl Call.exec (cmNew b)
    actionOfCodes (cmOutKind m) (cmOutZone m).toNat (cmOutRatio m).toNat =
      (step b.cfg (run b.cfg (startState b.cfg) (events true (cs.take n))).1
        (c.event (placedAfter true (cs.take n)))).2.velocity := by
  intro m
  obtain ⟨h1, h2, h3⟩ := foldl_exec_run (cmNew b) (cs.take n) hok
  have h4 := (foldl_exec_cfg (cmNew b) (cs.take n)).2.2
  have hm : m = c.exec ((cs.take n).foldl Call.exec (cmNew b)) := by
    simp only [m]
    rw [List.take_add_one, hc, Option.toList_some, List.foldl_append, List.foldl_cons,
      List.foldl_nil]
  have h := readBack_eq ((cs.take n).foldl Call.exec (cmNew b))
    (c.event ((cs.take n).foldl Call.exec (cmNew b)).placed) h2
    (by rw [h1]; simpa [cmNew, Builder.cfg] using hz)
  rw [hm, exec_eq_apply]
  refine Eq.trans ?_ (h.trans ?_)
  · rfl
  · rw [h1, h3, h4]
    rfl

/-- **End to end, when the configuration was refused.** If the builder is not
ok, the answer to every command, after any calls, is `stopSource`: the node
publishes zero. -/
theorem cm_not_ok_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hok : cmOk (cmNew b) = false) :
    cmOutKind (cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az) = 4 := by
  unfold cmOutKind
  rw [foldl_exec_not_ok (cmNew b) cs now lx ly lz ax ay az hok rfl]
  rfl

end ROSlean.CollisionMonitor.FFI
