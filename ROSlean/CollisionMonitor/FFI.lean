import Mathlib.Tactic
import ROSleanCore.CollisionMonitorFFI
import ROSlean.CollisionMonitor.Convert
import ROSlean.CollisionMonitor.Properties
import ROSlean.CollisionMonitor.Trace
import ROSlean.CollisionMonitor.Margin

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
  of its six calls (`BuilderCall`): if any call, anywhere in the sequence,
  carries a refused value (a timeout, a radius, `min_points`, a vertex), the
  monitor is not ok, whatever the other calls are (`not_ok_of_refused_call`);
  so is it if a vertex is added when the zone added last is not a polygon
  (`not_ok_of_stray_vertex`), and if `set_source` was never called
  (`not_ok_without_set_source`). What is refused is characterised in
  `ROSlean.CollisionMonitor.Convert`.
* **Fail closed.** A monitor that is not ok answers every command with
  `stopSource` while it is in the active phase, and with nothing otherwise,
  after any sequence of calls: never a pass, never a slowdown
  (`apply_cmd_of_not_ok`, `apply_out_of_not_ok`, `foldl_exec_not_ok`).
* **Decoding.** Each of the four byte arrays of the interface (the ranges
  of a scan, the coordinate pairs of a command, the vertices of a shape
  message, the zone vertices of a command) is decoded by a total function
  (`decodeBeams_length`, `decodePairs_length`, `decodeZoneVerts_empty`),
  every decoded pattern fits its field (`beamAt_lt`, `pairAt_lt`), and
  decoding inverts the encoding the shim performs
  (`decodeBeams_encodeRanges`, `decodePairs_encodePairs`,
  `decodeZoneVerts_encodeZoneVerts`). Coordinates are in the base frame and
  are kept as bit patterns; the model converts them when a command is
  evaluated (`coordMm`) or a shape is taken (`vertexCoordMm`). The pairs of
  a command fit the stored scan iff there is one per reading (`fits_decode`),
  and the points are then those of the valid readings (`points_decode`).
* **Events.** Each call is `step` on the event it delivers
  (`cmOnLifecycle_eq`, `cmOnScan_eq`, `cmOnShape_eq`, `cmOnCmd_eq`,
  `exec_st`), and everything an event carries is an argument of its call:
  the interface keeps nothing between calls but the monitor. So after any
  sequence of calls, with no condition on them, the state is the one `run`
  reaches on the events they deliver (`events`, `foldl_exec_run`), from the
  initial state `State.init`.
* **The plan of a command.** `cmCmdPlan` tells the shim which transforms to
  look up. If it says none, the command call returns the same monitor
  whatever the two arrays hold; if it says "the zones' only", whatever the
  scan coordinates are (`cmOnCmd_of_plan_zero`, `cmOnCmd_of_plan_one`); and
  it says "the scan's" exactly when the command is processed, the source is
  enabled and a scan is stored that is not too old (`cmdPlan_eq_three_iff`).
* **Shape messages.** The query after a shape call says whether the model
  accepted the message (`cmShapeAccepted_iff`); only then does the state
  change (`cmOnShape_st_of_accepted`, `cmOnShape_st_of_refused`).
* **Read-back.** The five numbers the node reads after a command determine
  the model's output exactly: whether the command was processed, the
  velocity action or that it is withheld, and the report (`readBack_eq`,
  `outputOfCodes_codes`, `kindOf_eq_zero_iff`).
* **End to end.** `cm_readBack_after`: create a monitor from a builder that is
  ok, make any sequence of lifecycle, scan, shape and command calls, then
  deliver a command; what the five exported queries return is the output of
  `step` for that command, in the state `run` reaches on the delivered
  events from `State.init`, on the configuration the builder holds. The
  statement is on the exported functions as the node calls them
  (`cm_readBack_after_fast` in `Fast.lean` for the command call C makes).

What remains outside Lean is the C++ shim: reading the parameters and the
clock, looking transforms up and applying them (the points and vertices a
call carries are the shim's word), the floating-point work listed in
SPEC_NAV2.md, section 7, and acting on the returned decision.
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
theorem zoneOfRaw_fields (s : ZoneShape) (a : UInt8) (r : UInt64) (mp : Int64) (en : UInt8)
    (n : Nat)
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

/-- `add_topic_polygon` appends a zone without shape, with the action,
threshold and flag of the call. -/
theorem builderAddTopicPolygon_zones (b : Builder) (a : UInt8) (r : UInt64) (mp : Int64)
    (en : UInt8) :
    (builderAddTopicPolygon b a r mp en).cfg.zones =
      b.cfg.zones ++ [(zoneOfRaw .topic a r mp en).1] := by
  simp [builderAddTopicPolygon, Builder.cfg]

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

/-- A monitor starts unconfigured (`State.init`), with the builder's
configuration and output `silent`; it is ok iff the builder reports no
error. -/
theorem cmNew_eq (b : Builder) :
    cmNew b = { cfg := b.cfg, ok := b.error == 0, err := b.error,
                zoneErrs := b.zones.map entryError, st := State.init b.cfg, out := .silent,
                accepted := false } := rfl

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
  | addTopicPolygon (action : UInt8) (ratio : UInt64) (minPoints : Int64) (enabled : UInt8)
  | addVertex (x y : UInt32)

/-- Performs the call. -/
def BuilderCall.exec (b : Builder) : BuilderCall → Builder
  | .setSource en node hasOwn own => builderSetSource b en node hasOwn own
  | .setStopPubTimeout t => builderSetStopPubTimeout b t
  | .addCircle radius a r mp en => builderAddCircle b radius a r mp en
  | .addPolygon a r mp en => builderAddPolygon b a r mp en
  | .addTopicPolygon a r mp en => builderAddTopicPolygon b a r mp en
  | .addVertex x y => builderAddVertex b x y

/-- The call carries a value that the conversions refuse: a timeout, a
radius, `min_points`, or a coordinate of a vertex. -/
def BuilderCall.Refused : BuilderCall → Prop
  | .setSource _ node hasOwn own =>
    timeoutNs node.toNat = none ∨ (hasOwn ≠ 0 ∧ timeoutNs own.toNat = none)
  | .setStopPubTimeout t => timeoutNs t.toNat = none
  | .addCircle radius _ _ mp _ => radiusMm radius.toNat = none ∨ minPointsOfRaw mp.toInt = none
  | .addPolygon _ _ mp _ => minPointsOfRaw mp.toInt = none
  | .addTopicPolygon _ _ mp _ => minPointsOfRaw mp.toInt = none
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
    | addTopicPolygon => exact ⟨ze, by simp [BuilderCall.exec, builderAddTopicPolygon, hm], h⟩
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
  | addTopicPolygon a r mp en =>
    exact .inr (.inr (.inr ⟨zoneOfRaw .topic a r mp en,
      by simp [BuilderCall.exec, builderAddTopicPolygon],
      by rw [zoneOfRaw_refused _ _ _ _ _ h]; decide⟩))
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
    | addTopicPolygon => rfl
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

/-! ## Decoding -/

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
32-bit pattern. -/
theorem beamAt_lt (a : ByteArray) (i : Nat) : (beamAt a i).range < 2 ^ 32 := u32At_lt ..

/-- The coordinates are 64-bit patterns. -/
theorem pairAt_lt (a : ByteArray) (i : Nat) :
    (pairAt a i).1 < 2 ^ 64 ∧ (pairAt a i).2 < 2 ^ 64 := ⟨u64At_lt .., u64At_lt ..⟩

/-- A byte array of any length is a scan of `size / 4` readings; a trailing
partial record is ignored. -/
theorem decodeBeams_length (a : ByteArray) : (decodeBeams a).length = a.size / 4 := by
  simp [decodeBeams, rangeSize]

/-- A byte array of any length is `size / 16` coordinate pairs. -/
theorem decodePairs_length (a : ByteArray) : (decodePairs a).length = a.size / 16 := by
  simp [decodePairs, pairSize]

/-- Reading `i` of the decoded scan is record `i` of the array. -/
theorem getElem_decodeBeams (a : ByteArray) (i : Nat) (h : i < (decodeBeams a).length) :
    (decodeBeams a)[i] = beamAt a i := by
  simp [decodeBeams]

/-- Pair `i` of the decoded coordinates is that of record `i` of the array. -/
theorem getElem_decodePairs (a : ByteArray) (i : Nat) (h : i < (decodePairs a).length) :
    (decodePairs a)[i] = pairAt a i := by
  simp [decodePairs]

/-- **When the coordinates of a command fit the stored scan**: iff the
coordinate array holds as many pairs as the scan array held readings. With
any other number the model treats the lookup as failed (E8, step 3). -/
theorem fits_decode (stamp : Int64) (rmin rmax : UInt32) (ranges points : ByteArray) :
    (scanOfRaw stamp rmin rmax ranges).fits (decodePairs points) = true ↔
      points.size / 16 = ranges.size / 4 := by
  simp [Scan.fits, scanOfRaw, decodeBeams_length, decodePairs_length]

/-- **The points of a command.** With one coordinate pair per reading of
the stored scan, the scan's points are the converted pairs of the readings
whose range is valid (R4), in order: reading `i` with pair `i`. -/
theorem points_decode (stamp : Int64) (rmin rmax : UInt32) (ranges points : ByteArray)
    (h : points.size / 16 = ranges.size / 4) :
    (scanOfRaw stamp rmin rmax ranges).points (decodePairs points) =
      ((List.range (ranges.size / rangeSize)).filter
        (fun i => (scanOfRaw stamp rmin rmax ranges).validBeam (beamAt ranges i))).map
        (fun i => scanPointMm (pairAt points i)) := by
  have h' : points.size / pairSize = ranges.size / rangeSize := h
  simp only [Scan.points, scanOfRaw, decodeBeams, decodePairs, h', List.zip_map', List.filter_map,
    List.map_map]
  rfl

/-- An empty array is a scan without readings. -/
theorem decodeBeams_empty : decodeBeams ByteArray.empty = [] := rfl

/-! ### Decoding inverts the shim's encoding -/

/-- The `k` little-endian bytes of a number. -/
def leBytes : Nat → Nat → List UInt8
  | 0, _ => []
  | k + 1, n => (n % 256).toUInt8 :: leBytes k (n / 256)

theorem leBytes_length (k n : Nat) : (leBytes k n).length = k := by
  induction k generalizing n with
  | zero => rfl
  | succ k ih => simp [leBytes, ih]

theorem toNat_toUInt8_mod (n : Nat) : (n % 256).toUInt8.toNat = n % 256 := by
  simp [Nat.toUInt8]

/-- The bytes of a number denote it, up to the size of the field. -/
theorem leNat_leBytes (k n : Nat) : leNat (leBytes k n) = n % 256 ^ k := by
  induction k generalizing n with
  | zero => simp [leBytes, leNat, Nat.mod_one]
  | succ k ih =>
    simp only [leBytes, leNat, toNat_toUInt8_mod, ih]
    rw [pow_succ, Nat.mul_comm (256 ^ k) 256, Nat.mod_mul]

/-- The 4 bytes the shim writes for a reading: its range pattern. -/
def rangeBytes (r : Nat) : List UInt8 := leBytes 4 r

/-- The 16 bytes the shim writes for a coordinate pair. -/
def pairBytes (c : Coords) : List UInt8 := leBytes 8 c.1 ++ leBytes 8 c.2

/-- The byte array the shim builds for the ranges of a scan. -/
def encodeRanges (rs : List Nat) : ByteArray := ⟨(rs.flatMap rangeBytes).toArray⟩

/-- The byte array the shim builds for a list of coordinate pairs: the
points of a command, or the vertices of a shape message. -/
def encodePairs (cs : List Coords) : ByteArray := ⟨(cs.flatMap pairBytes).toArray⟩

theorem pairBytes_length (c : Coords) : (pairBytes c).length = 16 := by
  simp [pairBytes, leBytes_length]

theorem flatMap_length_const {α : Type} (f : α → List UInt8) (n : Nat)
    (hf : ∀ x, (f x).length = n) (xs : List α) : (xs.flatMap f).length = n * xs.length := by
  induction xs with
  | nil => simp
  | cons x xs ih =>
    simp only [List.flatMap_cons, List.length_append, hf, ih, List.length_cons]
    rw [Nat.mul_succ]; omega

theorem encodeRanges_size (rs : List Nat) : (encodeRanges rs).size = 4 * rs.length := by
  simpa [encodeRanges, ByteArray.size] using
    flatMap_length_const rangeBytes 4 (fun r => leBytes_length 4 r) rs

theorem encodePairs_size (cs : List Coords) : (encodePairs cs).size = 16 * cs.length := by
  simpa [encodePairs, ByteArray.size] using flatMap_length_const pairBytes 16 pairBytes_length cs

/-- A byte of an array built from a list, as a list lookup. -/
theorem byteAt_mk (l : List UInt8) (i : Nat) :
    byteAt ⟨l.toArray⟩ i = ((l[i]?).map UInt8.toNat).getD 0 := by
  unfold byteAt
  by_cases h : i < (ByteArray.mk l.toArray).size
  · have h' : i < l.length := by simpa [ByteArray.size] using h
    rw [dif_pos h, List.getElem?_eq_getElem h']
    rfl
  · have h' : ¬ i < l.length := by simpa [ByteArray.size] using h
    rw [dif_neg h, List.getElem?_eq_none (by omega)]
    rfl

/-- Byte `n * i + j` of records of `n` bytes each is byte `j` of record
`i`. -/
theorem getElem?_flatMap_const {α : Type} (f : α → List UInt8) (n : Nat)
    (hf : ∀ x, (f x).length = n) (xs : List α) (i j : Nat) (hj : j < n) :
    (xs.flatMap f)[n * i + j]? = xs[i]?.bind (fun b => (f b)[j]?) := by
  induction xs generalizing i with
  | nil => simp
  | cons b bs ih =>
    rw [List.flatMap_cons]
    cases i with
    | zero =>
      rw [List.getElem?_append_left (by rw [hf]; omega)]
      simp
    | succ i =>
      have hge : (f b).length ≤ n * (i + 1) + j := by rw [hf, Nat.mul_succ]; omega
      rw [List.getElem?_append_right hge, hf]
      have : n * (i + 1) + j - n = n * i + j := by rw [Nat.mul_succ]; omega
      rw [this, ih]
      simp

/-- Reading `i` of an encoded scan is range `i`. -/
theorem u32At_encodeRanges (rs : List Nat) (h : ∀ r ∈ rs, r < 2 ^ 32) (i : Nat)
    (h2 : i < rs.length) : u32At (encodeRanges rs) (rangeSize * i) = rs[i] := by
  have hr := h rs[i] (List.getElem_mem h2)
  have hb : rs[i]? = some rs[i] := List.getElem?_eq_getElem h2
  have key : ∀ j, j < 4 → byteAt (encodeRanges rs) (4 * i + j) =
      (((rangeBytes rs[i])[j]?).map UInt8.toNat).getD 0 := by
    intro j hj
    rw [encodeRanges, byteAt_mk,
      getElem?_flatMap_const rangeBytes 4 (fun r => leBytes_length 4 r) rs i j hj, hb]
    rfl
  have k0 := key 0 (by omega)
  have k1 := key 1 (by omega)
  have k2 := key 2 (by omega)
  have k3 := key 3 (by omega)
  simp only [rangeBytes, leBytes, List.getElem?_cons_zero, List.getElem?_cons_succ,
    Option.map_some, Option.getD_some, Nat.add_zero, toNat_toUInt8_mod] at k0 k1 k2 k3
  simp only [u32At, rangeSize, k0, k1, k2, k3]
  omega

/-- The two fields of pair `i` of an encoded list of pairs are those of pair
`i`. -/
theorem pairAt_encodePairs (cs : List Coords) (h : ∀ c ∈ cs, c.1 < 2 ^ 64 ∧ c.2 < 2 ^ 64)
    (i : Nat) (h2 : i < cs.length) : pairAt (encodePairs cs) i = cs[i] := by
  obtain ⟨hx, hy⟩ := h cs[i] (List.getElem_mem h2)
  have hb : cs[i]? = some cs[i] := List.getElem?_eq_getElem h2
  have key : ∀ j, j < 16 → byteAt (encodePairs cs) (16 * i + j) =
      (((pairBytes cs[i])[j]?).map UInt8.toNat).getD 0 := by
    intro j hj
    rw [encodePairs, byteAt_mk, getElem?_flatMap_const pairBytes 16 pairBytes_length cs i j hj, hb]
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
  simp only [pairBytes, leBytes, List.cons_append, List.nil_append, List.getElem?_cons_zero,
    List.getElem?_cons_succ, Option.map_some, Option.getD_some, Nat.add_zero,
    toNat_toUInt8_mod] at k0 k1 k2 k3 k4 k5 k6 k7 k8 k9 k10 k11 k12 k13 k14 k15
  have e1 : u64At (encodePairs cs) (pairSize * i) = cs[i].1 := by
    simp only [u64At, u32At, pairSize, Nat.add_assoc, Nat.reduceAdd, k0, k1, k2, k3, k4, k5, k6, k7]
    omega
  have e2 : u64At (encodePairs cs) (pairSize * i + 8) = cs[i].2 := by
    simp only [u64At, u32At, pairSize, Nat.add_assoc, Nat.reduceAdd, k8, k9, k10, k11, k12, k13,
      k14, k15]
    omega
  simp only [pairAt, e1, e2]

/-- **Decoding inverts the encoding (ranges).** If the shim writes four
bytes per reading for patterns that fit them, the model sees exactly those
readings, in order. -/
theorem decodeBeams_encodeRanges (rs : List Nat) (h : ∀ r ∈ rs, r < 2 ^ 32) :
    decodeBeams (encodeRanges rs) = rs.map (fun r => { range := r }) := by
  apply List.ext_getElem
  · rw [decodeBeams_length, encodeRanges_size, List.length_map]; omega
  · intro i h1 h2
    rw [List.length_map] at h2
    rw [getElem_decodeBeams, List.getElem_map]
    simp only [beamAt, u32At_encodeRanges rs h i h2]

/-- **Decoding inverts the encoding (coordinates and vertices).** The model
sees the two patterns of every pair as given, unconverted, in order. -/
theorem decodePairs_encodePairs (cs : List Coords) (h : ∀ c ∈ cs, c.1 < 2 ^ 64 ∧ c.2 < 2 ^ 64) :
    decodePairs (encodePairs cs) = cs := by
  apply List.ext_getElem
  · rw [decodePairs_length, encodePairs_size]; omega
  · intro i h1 h2
    rw [getElem_decodePairs, pairAt_encodePairs cs h i h2]

/-- The bytes the shim writes for one zone in the array of zone vertices: a
count and the vertices, or the count `0xFFFFFFFF` for "none". -/
def zoneEntryBytes : Option (List Coords) → List UInt8
  | none => leBytes 4 0xFFFFFFFF
  | some vs => leBytes 4 vs.length ++ vs.flatMap pairBytes

/-- The byte array the shim builds for the zone vertices of a command. -/
def encodeZoneVerts (zs : List (Option (List Coords))) : ByteArray :=
  ⟨(zs.flatMap zoneEntryBytes).toArray⟩

theorem takePairs_flatMap (vs : List Coords) (rest : List UInt8)
    (h : ∀ c ∈ vs, c.1 < 2 ^ 64 ∧ c.2 < 2 ^ 64) :
    takePairs vs.length (vs.flatMap pairBytes ++ rest) = vs := by
  induction vs with
  | nil => rfl
  | cons c vs ih =>
    obtain ⟨hx, hy⟩ := h c List.mem_cons_self
    have e1 : (List.flatMap pairBytes (c :: vs) ++ rest) =
        leBytes 8 c.1 ++ (leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest)) := by
      simp [pairBytes, List.append_assoc]
    have t1 : (leBytes 8 c.1 ++ (leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest))).take 8 =
        leBytes 8 c.1 := List.take_left' (leBytes_length 8 c.1)
    have d1 : (leBytes 8 c.1 ++ (leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest))).drop 8 =
        leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest) := List.drop_left' (leBytes_length 8 c.1)
    have t2 : (leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest)).take 8 = leBytes 8 c.2 :=
      List.take_left' (leBytes_length 8 c.2)
    have d2 : (leBytes 8 c.1 ++ (leBytes 8 c.2 ++ (vs.flatMap pairBytes ++ rest))).drop 16 =
        vs.flatMap pairBytes ++ rest := by
      rw [← List.append_assoc]
      exact List.drop_left' (by simp [leBytes_length])
    rw [List.length_cons, takePairs, e1, t1, d1, t2, d2, ih (fun c hc => h c (List.mem_cons_of_mem _ hc)),
      leNat_leBytes, leNat_leBytes]
    have m1 : c.1 % 256 ^ 8 = c.1 := Nat.mod_eq_of_lt (by omega)
    have m2 : c.2 % 256 ^ 8 = c.2 := Nat.mod_eq_of_lt (by omega)
    rw [m1, m2]

/-- An entry of the zone vertices that the encoding can carry: a list of
fewer than 2³² − 1 vertices whose patterns fit 64 bits, or none. -/
def ZoneEntryFits : Option (List Coords) → Prop
  | none => True
  | some vs => vs.length < 0xFFFFFFFF ∧ ∀ c ∈ vs, c.1 < 2 ^ 64 ∧ c.2 < 2 ^ 64

theorem parseZoneVerts_flatMap (zs : List (Option (List Coords))) (fuel : Nat)
    (hfuel : zs.length ≤ fuel) (h : ∀ z ∈ zs, ZoneEntryFits z) :
    parseZoneVerts fuel (zs.flatMap zoneEntryBytes) = zs := by
  induction zs generalizing fuel with
  | nil =>
    cases fuel with
    | zero => rfl
    | succ f => simp [parseZoneVerts]
  | cons z zs ih =>
    cases fuel with
    | zero => simp at hfuel
    | succ f =>
      have hf : zs.length ≤ f := by simpa using hfuel
      have ihz := ih f hf (fun z hz => h z (List.mem_cons_of_mem _ hz))
      have hz := h z List.mem_cons_self
      rw [List.flatMap_cons]
      cases z with
      | none =>
        have t : (zoneEntryBytes none ++ zs.flatMap zoneEntryBytes).take 4 = leBytes 4 0xFFFFFFFF :=
          List.take_left' (leBytes_length 4 _)
        have d : (zoneEntryBytes none ++ zs.flatMap zoneEntryBytes).drop 4 =
            zs.flatMap zoneEntryBytes := List.drop_left' (leBytes_length 4 _)
        have hl : ¬ (zoneEntryBytes none ++ zs.flatMap zoneEntryBytes).length < 4 := by
          simp [zoneEntryBytes, leBytes_length]
        have hn : leNat (leBytes 4 0xFFFFFFFF) = 0xFFFFFFFF := by rw [leNat_leBytes]; norm_num
        rw [parseZoneVerts, if_neg hl, t, hn, if_pos rfl, d, ihz]
      | some vs =>
        obtain ⟨hlen, hfit⟩ := hz
        have e : zoneEntryBytes (some vs) ++ zs.flatMap zoneEntryBytes =
            leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes) := by
          simp [zoneEntryBytes, List.append_assoc]
        have t : (leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes)).take 4 =
            leBytes 4 vs.length := List.take_left' (leBytes_length 4 _)
        have d : (leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes)).drop 4 =
            vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes := List.drop_left' (leBytes_length 4 _)
        have hvl : (vs.flatMap pairBytes).length = 16 * vs.length :=
          flatMap_length_const pairBytes 16 pairBytes_length vs
        have d2 : (leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes)).drop
            (4 + 16 * vs.length) = zs.flatMap zoneEntryBytes := by
          rw [← List.append_assoc]
          exact List.drop_left' (by simp [leBytes_length, hvl])
        have hn : leNat (leBytes 4 vs.length) = vs.length := by
          rw [leNat_leBytes]; exact Nat.mod_eq_of_lt (by omega)
        have hl : ¬ (leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes)).length
            < 4 := by simp [leBytes_length]
        have hl2 : ¬ (leBytes 4 vs.length ++ (vs.flatMap pairBytes ++ zs.flatMap zoneEntryBytes)).length
            < 4 + 16 * vs.length := by simp [leBytes_length, hvl]
        have hne : vs.length ≠ 0xFFFFFFFF := by omega
        rw [e, parseZoneVerts, if_neg hl, t, hn, if_neg hne, if_neg hl2, d, d2,
          takePairs_flatMap vs _ hfit, ihz]

theorem zoneEntryBytes_length_pos (z : Option (List Coords)) : 1 ≤ (zoneEntryBytes z).length := by
  cases z <;> simp [zoneEntryBytes, leBytes_length] <;> omega

/-- **Decoding inverts the encoding (zone vertices).** The model sees, for
each zone in order, the vertices the shim wrote for it, or none. -/
theorem decodeZoneVerts_encodeZoneVerts (zs : List (Option (List Coords)))
    (h : ∀ z ∈ zs, ZoneEntryFits z) : decodeZoneVerts (encodeZoneVerts zs) = zs := by
  have hlen : zs.length ≤ (zs.flatMap zoneEntryBytes).length := by
    induction zs with
    | nil => simp
    | cons z zs ih =>
      have := zoneEntryBytes_length_pos z
      have := ih (fun z hz => h z (List.mem_cons_of_mem _ hz))
      simp only [List.flatMap_cons, List.length_append, List.length_cons]
      omega
  unfold decodeZoneVerts encodeZoneVerts
  simp only [ByteArray.size, List.size_toArray, List.toList_toArray]
  exact parseZoneVerts_flatMap zs _ hlen h

/-- No array is refused: an empty one carries no zone vertices. -/
theorem decodeZoneVerts_empty : decodeZoneVerts ByteArray.empty = [] := rfl

/-- The scan the model sees: stamp (negative taken as 0, D8), the two limits
as given, and the decoded readings. -/
theorem scanOfRaw_fields (stamp : Int64) (rmin rmax : UInt32) (a : ByteArray) :
    (scanOfRaw stamp rmin rmax a).stamp = stamp.toInt.toNat ∧
    (scanOfRaw stamp rmin rmax a).rangeMin = rmin.toNat ∧
    (scanOfRaw stamp rmin rmax a).rangeMax = rmax.toNat ∧
    (scanOfRaw stamp rmin rmax a).beams = decodeBeams a := ⟨rfl, rfl, rfl, rfl⟩

/-! ## Events -/

/-- No event changes the configuration, whether the monitor is ok, or the
record of the last shape call. -/
theorem apply_cfg (m : Monitor) (e : Event) :
    (m.apply e).cfg = m.cfg ∧ (m.apply e).ok = m.ok ∧ (m.apply e).accepted = m.accepted := by
  refine ⟨?_, ?_, ?_⟩ <;>
  · unfold Monitor.apply
    split
    · rfl
    · cases e <;> rfl

/-- An ok monitor does what `step` says. -/
theorem apply_of_ok (m : Monitor) (e : Event) (h : m.ok = true) :
    (m.apply e).st = (step m.cfg m.st e).1 ∧ (m.apply e).out = (step m.cfg m.st e).2 := by
  unfold Monitor.apply
  rw [if_pos h]
  exact ⟨rfl, rfl⟩

/-- **Fail closed.** A monitor that is not ok answers every command in the
active phase with `stopSource`: whatever the command (finite or not), the
scans it was given, the coordinates, and the time. -/
theorem apply_cmd_of_not_ok (m : Monitor) (now : Nat) (v : Twist) (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) (h : m.ok = false) (hp : m.st.phase = .active) :
    (m.apply (.cmd now v scanPts zoneVerts)).out = .answer (some .stopSource) none := by
  unfold Monitor.apply
  simp [h, hp]

/-- A monitor that is not ok keeps its state under scans, shape messages and
commands. -/
theorem apply_st_of_not_ok (m : Monitor) (e : Event) (h : m.ok = false)
    (he : (∃ now s, e = .scan now s) ∨ (∃ k verts foreign, e = .shape k verts foreign) ∨
      ∃ now v scanPts zoneVerts, e = .cmd now v scanPts zoneVerts) :
    (m.apply e).st = m.st := by
  unfold Monitor.apply
  rcases he with ⟨now, s, rfl⟩ | ⟨k, verts, foreign, rfl⟩ | ⟨now, v, scanPts, zoneVerts, rfl⟩ <;>
    simp [h]

/-- A monitor that is not ok never has an output that passes a command or
slows it down: its output is `silent` or the stop for an invalid source. -/
theorem apply_out_of_not_ok (m : Monitor) (e : Event) (h : m.ok = false) :
    (m.apply e).out = .silent ∨ (m.apply e).out = .answer (some .stopSource) none := by
  unfold Monitor.apply
  rw [if_neg (by simp [h])]
  cases e with
  | cmd now v p z =>
    by_cases hp : m.st.phase = .active
    · exact Or.inr (by simp [hp])
    · exact Or.inl (by simp [hp])
  | _ => exact Or.inl rfl

/-- The transition codes of `roslean_cm_on_lifecycle`. -/
theorem lifecycleEvent_codes :
    lifecycleEvent 0 = some .configure ∧ lifecycleEvent 1 = some .activate ∧
      lifecycleEvent 2 = some .deactivate ∧ lifecycleEvent 3 = some .cleanup ∧
      lifecycleEvent 4 = some .shutdown := ⟨rfl, rfl, rfl, rfl, rfl⟩

/-- Every other code is no event. -/
theorem lifecycleEvent_eq_none (t : UInt8) (h : 4 < t) : lifecycleEvent t = none := by
  have h' : 4 < t.toNat := UInt8.lt_iff_toNat_lt.mp h
  unfold lifecycleEvent
  have e : ∀ k : UInt8, k.toNat ≤ 4 → (t == k) = false := by
    intro k hk
    rw [beq_eq_false_iff_ne]
    intro heq
    rw [heq] at h'
    omega
  simp [e 0 (by decide), e 1 (by decide), e 2 (by decide), e 3 (by decide), e 4 (by decide)]

/-- The lifecycle call is `Monitor.apply` on the transition's event. -/
theorem cmOnLifecycle_eq (m : Monitor) (t : UInt8) (e : Event) (h : lifecycleEvent t = some e) :
    cmOnLifecycle m t = m.apply e := by
  simp [cmOnLifecycle, h]

/-- An unknown transition code changes nothing. -/
theorem cmOnLifecycle_of_unknown (m : Monitor) (t : UInt8) (h : 4 < t) : cmOnLifecycle m t = m := by
  simp [cmOnLifecycle, lifecycleEvent_eq_none t h]

/-- The phase query returns the code of the model's phase, and the codes are
distinct. -/
theorem cmPhase_eq (m : Monitor) :
    (cmPhase m = 0 ↔ m.st.phase = .unconfigured) ∧ (cmPhase m = 1 ↔ m.st.phase = .inactive) ∧
      (cmPhase m = 2 ↔ m.st.phase = .active) ∧ (cmPhase m = 3 ↔ m.st.phase = .finalized) := by
  unfold cmPhase
  cases m.st.phase <;> simp [phaseCode]

/-- The scan call is `Monitor.apply` on a scan event, which stores the
scan. -/
theorem cmOnScan_eq (m : Monitor) (now : UInt64) (stamp : Int64) (rmin rmax : UInt32)
    (a : ByteArray) :
    cmOnScan m now stamp rmin rmax a = m.apply (.scan now.toNat (scanOfRaw stamp rmin rmax a)) :=
  rfl

/-- The shape call is `Monitor.apply` on a shape event; the vertices are
those of the array, or none if `hasVerts` is 0. -/
theorem cmOnShape_eq (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8) (verts : ByteArray) :
    cmOnShape m zone foreign hasVerts verts =
      { m.apply (.shape zone.toNat (if hasVerts != 0 then some (decodePairs verts) else none)
          (foreign != 0)) with
        accepted := m.ok &&
          (acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0)).isSome } :=
  rfl

/-- **What "accepted" means.** The query after a shape call returns true iff
the monitor is ok and the model accepts the message (`acceptShape`;
characterised by `acceptShape_eq_some_iff`). -/
theorem cmShapeAccepted_iff (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8)
    (verts : ByteArray) :
    cmShapeAccepted (cmOnShape m zone foreign hasVerts verts) = true ↔
      m.ok = true ∧ ∃ sh, acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts)
        (foreign != 0) = some sh := by
  simp [cmShapeAccepted, cmOnShape, Option.isSome_iff_exists]

/-- **What the query about the mark returns**: whether the model's state
holds the mark of an unusable message for that zone (`shapeUnusable`). -/
theorem cmShapeUnusable_eq (m : Monitor) (zone : UInt32) :
    cmShapeUnusable m zone = shapeUnusable m.st.shapes zone.toNat := rfl

/-- **The query after an accepted shape call** (the one the shim makes): it
is true iff what the call installed is the mark of an unusable message (when
that is: `acceptShape_unusable_iff`), provided the state has an entry for
the zone (as every reachable state has, `Inv.len`). -/
theorem cmShapeUnusable_after (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8)
    (verts : ByteArray) (h : cmShapeAccepted (cmOnShape m zone foreign hasVerts verts) = true)
    (hk : zone.toNat < m.st.shapes.length) :
    cmShapeUnusable (cmOnShape m zone foreign hasVerts verts) zone = true ↔
      ∃ sh, acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0) = some sh ∧
        sh.unusable = true := by
  obtain ⟨hok, sh, hsh⟩ := (cmShapeAccepted_iff m zone foreign hasVerts verts).mp h
  have hst : (cmOnShape m zone foreign hasVerts verts).st =
      (step m.cfg m.st (.shape zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0))).1 := by
    change (m.apply (shapeEvent zone foreign hasVerts verts)).st = _
    rw [(apply_of_ok m _ hok).1, shapeEvent]
  rw [cmShapeUnusable_eq, hst, step_shape_unusable _ _ _ _ _ sh hsh hk]
  constructor
  · intro hu; exact ⟨sh, hsh, hu⟩
  · rintro ⟨sh', hsh', hu⟩
    rw [hsh] at hsh'
    cases hsh'
    exact hu

/-- An accepted shape call installs the shape in its zone, and nothing else
changes. -/
theorem cmOnShape_st_of_accepted (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8)
    (verts : ByteArray) (h : cmShapeAccepted (cmOnShape m zone foreign hasVerts verts) = true) :
    ∃ sh, acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0) = some sh ∧
      (cmOnShape m zone foreign hasVerts verts).st =
        { m.st with shapes := m.st.shapes.set zone.toNat (some sh) } := by
  obtain ⟨hok, sh, hsh⟩ := (cmShapeAccepted_iff m zone foreign hasVerts verts).mp h
  refine ⟨sh, hsh, ?_⟩
  change (m.apply (shapeEvent zone foreign hasVerts verts)).st = _
  rw [(apply_of_ok m _ hok).1, shapeEvent, step_shape_of_accepted _ _ _ _ _ sh hsh]

/-- A shape call that is not accepted changes no state. -/
theorem cmOnShape_st_of_refused (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8)
    (verts : ByteArray) (h : cmShapeAccepted (cmOnShape m zone foreign hasVerts verts) = false) :
    (cmOnShape m zone foreign hasVerts verts).st = m.st := by
  change (m.apply (shapeEvent zone foreign hasVerts verts)).st = _
  cases hok : m.ok with
  | false => exact apply_st_of_not_ok m _ hok (Or.inr (Or.inl ⟨_, _, _, rfl⟩))
  | true =>
    have hn : acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0) = none := by
      by_contra hc
      obtain ⟨sh, hsh⟩ := Option.ne_none_iff_exists'.mp hc
      have := (cmShapeAccepted_iff m zone foreign hasVerts verts).mpr ⟨hok, sh, hsh⟩
      rw [h] at this
      cases this
    rw [(apply_of_ok m _ hok).1, shapeEvent, step_shape_of_refused _ _ _ _ _ hn]

/-- The command call is `Monitor.apply` on a command event with the six
patterns in the order `linear.x`, `linear.y`, `linear.z`, `angular.x`,
`angular.y`, `angular.z`, the coordinate pairs of the array `points` (none
if `hasPoints` is 0) and the decoded zone vertices. -/
theorem cmOnCmd_eq (m : Monitor) (now lx ly lz ax ay az : UInt64) (hasPoints : UInt8)
    (points zoneVerts : ByteArray) :
    cmOnCmd m now lx ly lz ax ay az hasPoints points zoneVerts =
      m.apply (.cmd now.toNat ⟨lx.toNat, ly.toNat, lz.toNat, ax.toNat, ay.toNat, az.toNat⟩
        (if hasPoints != 0 then some (decodePairs points) else none)
        (decodeZoneVerts zoneVerts)) := rfl

/-! ## The plan of a command -/

/-- The plan is one of the three codes. -/
theorem cmdPlan_cases (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    cmdPlan cfg st now v = 0 ∨ cmdPlan cfg st now v = 1 ∨ cmdPlan cfg st now v = 3 := by
  unfold cmdPlan
  repeat' split
  all_goals simp

/-- With plan 0 the command is not processed, or its source is invalid
whatever coordinates it carries. -/
theorem sourcePoints_of_plan_zero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : cmdPlan cfg st now v = 0) (hp : st.phase = .active) (hf : v.finite = true)
    (p : Option (List Coords)) : sourcePoints cfg st now p = none := by
  unfold cmdPlan at h
  unfold sourcePoints
  simp only [hp, ne_eq, not_true_eq_false, if_false, hf, Bool.not_true, Bool.false_eq_true] at h
  cases he : cfg.sourceEnabled with
  | false => simp [he] at h
  | true =>
    simp only [he, Bool.not_true, Bool.false_eq_true, if_false] at h ⊢
    cases hs : st.scan with
    | none =>
      simp only [hs] at h ⊢
      split at h
      · simp at h
      · rename_i ht; simp [ht]
    | some s =>
      simp only [hs] at h ⊢
      split at h
      · rename_i ht; simp [ht]
      · simp at h

/-- With plan 1 the source has no points, whatever coordinates the command
carries. -/
theorem sourcePoints_of_plan_one (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (h : cmdPlan cfg st now v = 1) (p : Option (List Coords)) :
    sourcePoints cfg st now p = some [] := by
  unfold cmdPlan at h
  unfold sourcePoints
  split at h
  · simp at h
  · split at h
    · simp at h
    · cases he : cfg.sourceEnabled with
      | false => simp
      | true =>
        simp only [he, Bool.not_true, Bool.false_eq_true, if_false] at h ⊢
        cases hs : st.scan with
        | none =>
          simp only [hs] at h ⊢
          split at h
          · rename_i ht; simp [ht]
          · simp at h
        | some s =>
          simp only [hs] at h
          split at h <;> simp at h

/-- **Plan 0: nothing the shim could look up matters.** The step of a
command whose plan is 0 is the same for all scan coordinates and all zone
vertices, so the shim makes no lookup, as upstream makes none. -/
theorem step_cmd_of_plan_zero (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (p p' : Option (List Coords)) (z z' : List (Option (List Coords)))
    (h : cmdPlan cfg st now v = 0) :
    step cfg st (.cmd now v p z) = step cfg st (.cmd now v p' z') := by
  by_cases hp : st.phase = .active
  · cases hf : v.finite with
    | false => simp [step, hf]
    | true =>
      have h1 := sourcePoints_of_plan_zero cfg st now v h hp hf
      rw [step_cmd _ _ _ _ _ _ hp hf, step_cmd _ _ _ _ _ _ hp hf]
      simp only [evaluate, shapesAfter, h1]
  · simp [step, hp]

/-- **Plan 1: the scan's coordinates do not matter.** The step of a command
whose plan is 1 is the same for all scan coordinates: the shim looks up the
transforms of the topic zones only. -/
theorem step_cmd_of_plan_one (cfg : Config) (st : State) (now : Nat) (v : Twist)
    (p p' : Option (List Coords)) (z : List (Option (List Coords)))
    (h : cmdPlan cfg st now v = 1) :
    step cfg st (.cmd now v p z) = step cfg st (.cmd now v p' z) := by
  have h1 := sourcePoints_of_plan_one cfg st now v h
  simp only [step, evaluate, shapesAfter, h1]

/-- **Plan 3: upstream looks the scan's transform up.** The plan is 3 iff
the command is processed, the source is enabled, a scan is stored and it is
not too old. -/
theorem cmdPlan_eq_three_iff (cfg : Config) (st : State) (now : Nat) (v : Twist) :
    cmdPlan cfg st now v = 3 ↔
      st.phase = .active ∧ v.finite = true ∧ cfg.sourceEnabled = true ∧
        ∃ s, st.scan = some s ∧ ¬ (0 < cfg.sourceTimeout ∧ s.stamp + cfg.sourceTimeout < now) := by
  unfold cmdPlan
  by_cases hp : st.phase = .active
  · cases hf : v.finite with
    | false => simp [hp]
    | true =>
      cases he : cfg.sourceEnabled with
      | false => simp [hp]
      | true =>
        cases hs : st.scan with
        | none => simp [hp]; split <;> simp
        | some s =>
          simp only [hp, ne_eq, not_true_eq_false, if_false, Bool.not_true, Bool.false_eq_true,
            Option.some.injEq, exists_eq_left', true_and]
          split <;> rename_i ht <;> simp_all
  · simp [hp]

/-- The plan query on a monitor that is ok is `cmdPlan`; it returns a number
and no monitor, so it cannot change one. -/
theorem cmCmdPlan_of_ok (m : Monitor) (now lx ly lz ax ay az : UInt64) (h : m.ok = true) :
    cmCmdPlan m now lx ly lz ax ay az =
      cmdPlan m.cfg m.st now.toNat (twistOfRaw lx ly lz ax ay az) := by
  simp [cmCmdPlan, h]

/-- **If the plan is 0 the command call depends on neither array.** -/
theorem cmOnCmd_of_plan_zero (m : Monitor) (now lx ly lz ax ay az : UInt64)
    (hasPoints hasPoints' : UInt8) (points points' zoneVerts zoneVerts' : ByteArray)
    (h : cmCmdPlan m now lx ly lz ax ay az = 0) :
    cmOnCmd m now lx ly lz ax ay az hasPoints points zoneVerts =
      cmOnCmd m now lx ly lz ax ay az hasPoints' points' zoneVerts' := by
  unfold cmOnCmd cmdEvent Monitor.apply
  cases hok : m.ok with
  | false => simp
  | true =>
    rw [cmCmdPlan_of_ok m _ _ _ _ _ _ _ hok] at h
    simp only [if_true]
    rw [step_cmd_of_plan_zero m.cfg m.st _ _ (pointsOfRaw hasPoints points)
      (pointsOfRaw hasPoints' points') (decodeZoneVerts zoneVerts) (decodeZoneVerts zoneVerts') h]

/-- **If the plan is 1 the command call does not depend on the scan
coordinates.** -/
theorem cmOnCmd_of_plan_one (m : Monitor) (now lx ly lz ax ay az : UInt64)
    (hasPoints hasPoints' : UInt8) (points points' zoneVerts : ByteArray)
    (h : cmCmdPlan m now lx ly lz ax ay az = 1) :
    cmOnCmd m now lx ly lz ax ay az hasPoints points zoneVerts =
      cmOnCmd m now lx ly lz ax ay az hasPoints' points' zoneVerts := by
  unfold cmOnCmd cmdEvent Monitor.apply
  cases hok : m.ok with
  | false => simp [cmCmdPlan, hok] at h
  | true =>
    rw [cmCmdPlan_of_ok m _ _ _ _ _ _ _ hok] at h
    simp only [if_true]
    rw [step_cmd_of_plan_one m.cfg m.st _ _ (pointsOfRaw hasPoints points)
      (pointsOfRaw hasPoints' points') (decodeZoneVerts zoneVerts) h]

/-! ## Sequences of calls -/

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
the four exported functions the node calls after it has built the monitor.
(The command without a clock reading, `cmOnCmdNoClock`, is not an event of
the model and has its own theorems in `NoClock.lean`.) -/
inductive Call where
  | lifecycle (transition : UInt8)
  | scan (now : UInt64) (stamp : Int64) (rangeMin rangeMax : UInt32) (ranges : ByteArray)
  | shape (zone : UInt32) (foreign hasVerts : UInt8) (verts : ByteArray)
  | cmd (now linX linY linZ angX angY angZ : UInt64) (hasPoints : UInt8)
      (points zoneVerts : ByteArray)

/-- Performs the call. -/
def Call.exec (m : Monitor) : Call → Monitor
  | .lifecycle t => cmOnLifecycle m t
  | .scan now stamp rmin rmax a => cmOnScan m now stamp rmin rmax a
  | .shape zone foreign hasVerts verts => cmOnShape m zone foreign hasVerts verts
  | .cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    cmOnCmd m now lx ly lz ax ay az hasPoints points zoneVerts

/-- The events of the model that a call delivers: one, or none for a
lifecycle call with an unknown code. Everything an event carries is an
argument of its call: the interface keeps nothing between calls but the
monitor. -/
def Call.events : Call → List Event
  | .lifecycle t => (lifecycleEvent t).toList
  | .scan now stamp rmin rmax a => [.scan now.toNat (scanOfRaw stamp rmin rmax a)]
  | .shape zone foreign hasVerts verts => [shapeEvent zone foreign hasVerts verts]
  | .cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    [cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts]

/-- The events a sequence of calls delivers. -/
def events (cs : List Call) : List Event := cs.flatMap Call.events

/-- No call changes the configuration or whether the monitor is ok. -/
theorem exec_cfg (m : Monitor) (c : Call) : (c.exec m).cfg = m.cfg ∧ (c.exec m).ok = m.ok := by
  cases c with
  | lifecycle t =>
    simp only [Call.exec, cmOnLifecycle]
    cases lifecycleEvent t with
    | none => exact ⟨rfl, rfl⟩
    | some e => exact ⟨(apply_cfg m e).1, (apply_cfg m e).2.1⟩
  | scan now stamp rmin rmax a => exact ⟨(apply_cfg m _).1, (apply_cfg m _).2.1⟩
  | shape zone foreign hasVerts verts =>
    exact ⟨(apply_cfg m (shapeEvent zone foreign hasVerts verts)).1,
      (apply_cfg m (shapeEvent zone foreign hasVerts verts)).2.1⟩
  | cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    exact ⟨(apply_cfg m _).1, (apply_cfg m _).2.1⟩

/-- Every call takes an ok monitor to the state `run` reaches on the events
it delivers. -/
theorem exec_st (m : Monitor) (c : Call) (h : m.ok = true) :
    (c.exec m).st = (run m.cfg m.st c.events).1 := by
  cases c with
  | lifecycle t =>
    simp only [Call.exec, cmOnLifecycle, Call.events]
    cases lifecycleEvent t with
    | none => rfl
    | some e =>
      simp only [Option.toList_some, run_cons, run_nil]
      exact (apply_of_ok m e h).1
  | scan now stamp rmin rmax a =>
    simp only [Call.events, run_cons, run_nil]
    exact (apply_of_ok m _ h).1
  | shape zone foreign hasVerts verts =>
    simp only [Call.events, run_cons, run_nil]
    exact (apply_of_ok m (shapeEvent zone foreign hasVerts verts) h).1
  | cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    simp only [Call.events, run_cons, run_nil]
    exact (apply_of_ok m _ h).1

theorem foldl_exec_cfg (m : Monitor) (cs : List Call) :
    (cs.foldl Call.exec m).cfg = m.cfg ∧ (cs.foldl Call.exec m).ok = m.ok := by
  induction cs generalizing m with
  | nil => exact ⟨rfl, rfl⟩
  | cons c cs ih =>
    rw [List.foldl_cons, (ih _).1, (ih _).2, (exec_cfg m c).1, (exec_cfg m c).2]
    exact ⟨rfl, rfl⟩

/-- **Any sequence of calls is the sequence of its events.** After any
sequence of lifecycle, scan, shape and command calls, with no condition on
them, an ok monitor has the same configuration and the state `run` reaches
on the events the calls deliver. -/
theorem foldl_exec_run (m : Monitor) (cs : List Call) (h : m.ok = true) :
    (cs.foldl Call.exec m).cfg = m.cfg ∧ (cs.foldl Call.exec m).ok = true ∧
    (cs.foldl Call.exec m).st = (run m.cfg m.st (events cs)).1 := by
  induction cs generalizing m with
  | nil => exact ⟨rfl, h, rfl⟩
  | cons c cs ih =>
    obtain ⟨h1, h2, h3⟩ := ih (c.exec m) (by rw [(exec_cfg m c).2]; exact h)
    rw [List.foldl_cons, h1, h3, (exec_cfg m c).1, exec_st m c h]
    refine ⟨rfl, h2, ?_⟩
    simp only [events, List.flatMap_cons, run_append]

/-- **Fail closed, on sequences.** If the monitor was not ok when it was
created, then after any calls whatever the answer to a command is the stop
for an invalid source if the node is active, and nothing if it is not: never
a pass and never a slowdown. -/
theorem foldl_exec_not_ok (m : Monitor) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray) (h : m.ok = false) :
    (cmOnCmd (cs.foldl Call.exec m) now lx ly lz ax ay az hasPoints points zoneVerts).out =
      if (cs.foldl Call.exec m).st.phase = .active then .answer (some .stopSource) none
      else .silent := by
  have hok : (cs.foldl Call.exec m).ok = false := by rw [(foldl_exec_cfg m cs).2]; exact h
  unfold cmOnCmd cmdEvent Monitor.apply
  simp [hok]

/-! ## Reading the output back

The node reads five numbers. They determine the output, because the zone
indices and the ratio are never truncated. -/

/-- The velocity action denoted by the numbers the node reads; `none` for
kinds 0 and 5 and for every unknown kind. -/
def actionOfCodes (kind : UInt8) (zone ratio : Nat) : Option Action :=
  if kind = 1 then some .pass
  else if kind = 2 then some (.slowdown zone ratio)
  else if kind = 3 then some (.stopZone zone)
  else if kind = 4 then some .stopSource
  else none

/-- The report denoted by the numbers the node reads; `none` for code 0 and
for every unknown code. -/
def reportOfCodes (report : UInt8) (zone : Nat) : Option Report :=
  if report = 1 then some .normal
  else if report = 2 then some (.stop zone)
  else if report = 3 then some (.slowdown zone)
  else if report = 4 then some .source
  else none

/-- The output denoted by the five numbers the node reads: kind 0 is
`silent`, every other kind a processed command. -/
def outputOfCodes (kind report : UInt8) (zone ratio reportZone : Nat) : Output :=
  if kind = 0 then .silent
  else .answer (actionOfCodes kind zone ratio) (reportOfCodes report reportZone)

/-- The codes determine the output. -/
theorem outputOfCodes_codes (o : Output) :
    outputOfCodes (kindOf o) (reportOf o) (zoneOf o) (ratioOf o) (reportZoneOf o) = o := by
  cases o with
  | silent => rfl
  | answer vel rep =>
    rcases vel with _ | a <;> rcases rep with _ | r
    · rfl
    · cases r <;> rfl
    · cases a <;> rfl
    · cases a <;> cases r <;> rfl

/-- Kind 0 is `silent` and nothing else: the shim publishes nothing, on any
topic, exactly when the model's output is `silent`. -/
theorem kindOf_eq_zero_iff (o : Output) : kindOf o = 0 ↔ o = .silent := by
  cases o with
  | silent => simp [kindOf]
  | answer vel rep =>
    rcases vel with _ | a
    · simp [kindOf]
    · cases a <;> simp [kindOf]

/-- Kind 5 is a processed command whose velocity is withheld. -/
theorem kindOf_eq_five_iff (o : Output) : kindOf o = 5 ↔ ∃ rep, o = .answer none rep := by
  cases o with
  | silent => simp [kindOf]
  | answer vel rep =>
    rcases vel with _ | a
    · simp [kindOf]
    · cases a <;> simp [kindOf]

/-- A published stop names a configured zone. -/
theorem step_publish_stopZone (cfg : Config) (st : State) (e : Event) (k : Nat)
    (h : (step cfg st e).2.velocity = some (.stopZone k)) : k < cfg.zones.length := by
  obtain ⟨now, v, scanPts, zoneVerts, -, -, -, ha⟩ := step_velocity cfg st e _ h
  obtain ⟨pts, z, z0, -, -, hz, -⟩ := evaluate_stopZone_only_if cfg st now scanPts zoneVerts k ha.symm
  exact (List.getElem?_eq_some_iff.mp hz).1

/-- A published slowdown names a configured slowdown zone and carries that
zone's ratio, which is below the pattern of 1.0. -/
theorem step_publish_slowdown (cfg : Config) (st : State) (e : Event) (k r : Nat)
    (h : (step cfg st e).2.velocity = some (.slowdown k r)) :
    r < F64.one ∧ ∃ z, cfg.zones[k]? = some z ∧ z.action = .slowdown r := by
  obtain ⟨now, v, scanPts, zoneVerts, -, -, -, ha⟩ := step_velocity cfg st e _ h
  exact evaluate_slowdown cfg st now scanPts zoneVerts k r ha.symm

/-- A report names a configured zone. -/
theorem step_report_zone (cfg : Config) (st : State) (e : Event) (k : Nat)
    (h : (step cfg st e).2.report = some (.stop k) ∨
      (step cfg st e).2.report = some (.slowdown k)) : k < cfg.zones.length := by
  cases e with
  | cmd now v scanPts zoneVerts =>
    have key : ∀ r, (step cfg st (.cmd now v scanPts zoneVerts)).2.report = some r →
        (r = .stop k ∨ r = .slowdown k) → k < cfg.zones.length := by
      intro r hr hk
      obtain ⟨-, -, he, -⟩ := (report_eq_some_iff cfg st now v scanPts zoneVerts r).mp hr
      cases ha : evaluate cfg st now scanPts zoneVerts with
      | pass => rw [ha] at he; simp [Action.report] at he; subst he; simp at hk
      | stopSource => rw [ha] at he; simp [Action.report] at he; subst he; simp at hk
      | stopZone k' =>
        rw [ha] at he
        simp only [Action.report] at he
        subst he
        have hkk : k' = k := by simpa using hk
        subst hkk
        obtain ⟨pts, z, z0, -, -, hz, -⟩ :=
          evaluate_stopZone_only_if cfg st now scanPts zoneVerts k' ha
        exact (List.getElem?_eq_some_iff.mp hz).1
      | slowdown k' r' =>
        rw [ha] at he
        simp only [Action.report] at he
        obtain ⟨-, z, hz, -⟩ := evaluate_slowdown cfg st now scanPts zoneVerts k' r' ha
        split at he
        · subst he; simp at hk
        · subst he
          have hkk : k' = k := by simpa using hk
          subst hkk
          exact (List.getElem?_eq_some_iff.mp hz).1
    rcases h with h | h
    · exact key _ h (Or.inl rfl)
    · exact key _ h (Or.inr rfl)
  | shape k' verts foreign =>
    simp only [step] at h
    split at h <;> simp [Output.report] at h
  | _ => simp [step, Output.report] at h

/-- **The node reads back the model's output.** For an ok monitor with at
most 2³² zones, the five numbers that the queries return after an event
determine the output of `step` for that event exactly: the velocity action,
the report, and whether the event was a processed command at all. Nothing is
truncated on the way to C. -/
theorem readBack_eq (m : Monitor) (e : Event) (hok : m.ok = true)
    (hz : m.cfg.zones.length ≤ 2 ^ 32) :
    outputOfCodes (cmOutKind (m.apply e)) (cmOutReport (m.apply e)) (cmOutZone (m.apply e)).toNat
      (cmOutRatio (m.apply e)).toNat (cmOutReportZone (m.apply e)).toNat =
        (step m.cfg m.st e).2 := by
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
  have hrz : reportZoneOf (step m.cfg m.st e).2 < 2 ^ 32 := by
    unfold reportZoneOf
    rcases ho : (step m.cfg m.st e).2.report with _ | r
    · simp
    · cases r with
      | normal => simp
      | source => simp
      | stop k => have := step_report_zone m.cfg m.st e k (Or.inl ho); simp only; omega
      | slowdown k => have := step_report_zone m.cfg m.st e k (Or.inr ho); simp only; omega
  simp only [cmOutKind, cmOutZone, cmOutRatio, cmOutReport, cmOutReportZone, (apply_of_ok m e hok).2]
  have e1 : (zoneOf (step m.cfg m.st e).2).toUInt32.toNat = zoneOf (step m.cfg m.st e).2 := by
    simp only [Nat.toUInt32, UInt32.toNat_ofNat']; omega
  have e2 : (ratioOf (step m.cfg m.st e).2).toUInt64.toNat = ratioOf (step m.cfg m.st e).2 := by
    simp only [Nat.toUInt64, UInt64.toNat_ofNat']; omega
  have e3 : (reportZoneOf (step m.cfg m.st e).2).toUInt32.toNat =
      reportZoneOf (step m.cfg m.st e).2 := by
    simp only [Nat.toUInt32, UInt32.toNat_ofNat']; omega
  rw [e1, e2, e3]
  exact outputOfCodes_codes _

/-! ## End to end

Stated on the exported functions in the order the node calls them: `cmNew` on
the builder, then lifecycle, scan, shape and command calls in any order and
number (`Call`, `Call.exec`), then the command call in question and the five
queries. The events of the model are those the calls deliver (`events`), from
the initial state `State.init`: the theorems of `Properties.lean`,
`Lifecycle.lean` and `Trace.lean`, which hold for every sequence of events
from that state, apply with `es := events cs`. -/

/-- **End to end.** Create a monitor from a builder that is ok, make any
sequence of calls (lifecycle transitions, scans, shape messages, commands,
in any order), then deliver a velocity command. The recorded output is the
output of `step` for that command's event, in the state `run` reaches from
the initial state on the events the calls deliver, on the configuration the
builder holds. -/
theorem cm_out_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray) (hok : cmOk (cmNew b) = true) :
    (cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az hasPoints points zoneVerts).out =
      (step b.cfg (run b.cfg (State.init b.cfg) (events cs)).1
        (cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts)).2 := by
  obtain ⟨h1, h2, h3⟩ := foldl_exec_run (cmNew b) cs hok
  unfold cmOnCmd
  rw [(apply_of_ok _ _ h2).2, h1, h3]
  rfl

/-- **End to end, as the node sees it.** For at most 2³² zones, the five
numbers the exported queries return after the command determine the model's
output for it: the velocity action, the report, and whether the command was
processed. -/
theorem cm_readBack_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray)
    (hok : cmOk (cmNew b) = true) (hz : b.zones.length ≤ 2 ^ 32) :
    let m := cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az hasPoints points zoneVerts
    outputOfCodes (cmOutKind m) (cmOutReport m) (cmOutZone m).toNat (cmOutRatio m).toNat
        (cmOutReportZone m).toNat =
      (step b.cfg (run b.cfg (State.init b.cfg) (events cs)).1
        (cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts)).2 := by
  intro m
  obtain ⟨h1, h2, h3⟩ := foldl_exec_run (cmNew b) cs hok
  have h := readBack_eq (cs.foldl Call.exec (cmNew b))
    (cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts) h2
    (by rw [h1]; simpa [cmNew, Builder.cfg] using hz)
  have hm : m = (cs.foldl Call.exec (cmNew b)).apply
      (cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts) := rfl
  rw [hm, h, h1, h3]
  rfl

/-- **End to end: the phase the node is in.** After any calls the phase of
an ok monitor is the phase `run` reaches on their events; the lifecycle
theorems (`run_silent_before_activate`, `run_silent_after_deactivate`,
`run_silent_after_shutdown`) then say when a command is answered at all. -/
theorem cm_phase_after (b : Builder) (cs : List Call) (hok : cmOk (cmNew b) = true) :
    cmPhase (cs.foldl Call.exec (cmNew b)) =
      phaseCode (run b.cfg (State.init b.cfg) (events cs)).1.phase := by
  obtain ⟨-, -, h3⟩ := foldl_exec_run (cmNew b) cs hok
  unfold cmPhase
  rw [h3]
  rfl

/-- **End to end, when the configuration was refused.** If the builder is not
ok, the answer to every command, after any calls, is kind 4 (`stopSource`:
the node publishes zero) if the monitor is in the active phase and kind 0
(nothing) otherwise. The node fails its `configure` transition for such a
monitor and never activates it; this is the backstop. -/
theorem cm_not_ok_after (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray) (hok : cmOk (cmNew b) = false) :
    cmOutKind (cmOnCmd (cs.foldl Call.exec (cmNew b)) now lx ly lz ax ay az hasPoints points
        zoneVerts) =
      if (cs.foldl Call.exec (cmNew b)).st.phase = .active then 4 else 0 := by
  unfold cmOutKind
  rw [foldl_exec_not_ok (cmNew b) cs now lx ly lz ax ay az hasPoints points zoneVerts hok]
  split <;> rfl

end ROSlean.CollisionMonitor.FFI
