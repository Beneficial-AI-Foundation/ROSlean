import ROSlean.CollisionMonitor.FFI
import ROSleanCore.CollisionMonitorFast

/-!
# Collision monitor: the fast command path computes the same function

`core/ROSleanCore/CollisionMonitorFast.lean` holds a second way to compute
what `step` computes for a command, and the C interface calls it
(`cmOnCmdFast`, exported as `roslean_cm_on_cmd`). This file proves, function
by function, that nothing changed:

* `pointsBy_toList`: the one-pass points are `Scan.points`;
* `sourcePointsBy_eq`: the same for `sourcePoints`;
* `nearSegment_box`, `nearBoundary_near`: a point within the band of D4
  around a polygon's boundary is within 2 mm of the polygon's bounding box,
  so the test that is skipped outside it would have said "no";
* `containsEdges_eq`, `countFast_eq`, `triggeredFast_eq`,
  `zonesActionFast_eq`: the zone tests;
* `stepCmdFast_eq`: **the fast command step is `step`** on the command
  event, for every configuration, state, time, velocity and coordinates;
* `coordMmAt_eq`, `pointAt_eq`: decoding a coordinate from its two 32-bit
  halves is `coordMm` of the 64-bit pattern;
* `cmOnCmdFast_eq`, `synced_after`, `execFast_eq`: the exported command call
  returns the monitor that `cmOnCmd` returns, after any sequence of calls.

Every theorem of the other files is stated on `step` and `cmOnCmd`; by the
last item they are statements about what C calls.
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## Scan points -/

theorem pointsGo_toList (s : Scan) (f : Nat → Point) (g : Nat → Coords)
    (hf : ∀ i, f i = scanPointMm (g i)) (bs : List Beam) (i : Nat) (acc : Array Point) :
    (s.pointsGo f bs i acc).toList = acc.toList ++
      ((bs.zip ((List.range' i bs.length).map g)).filter (fun bc => s.validBeam bc.1)).map
        (fun bc => scanPointMm bc.2) := by
  induction bs generalizing i acc with
  | nil => simp [Scan.pointsGo]
  | cons b bs ih =>
    rw [Scan.pointsGo, ih]
    simp only [List.length_cons, List.range'_succ, List.map_cons, List.zip_cons_cons,
      List.filter_cons]
    by_cases hv : s.validBeam b = true
    · simp [hv, hf]
    · simp [hv]

/-- **The one-pass points are the scan's points.** If `f i` is the converted
pair `g i` for every index, `Scan.pointsBy` gives `Scan.points` under the
coordinates `g 0, g 1, …`, one per reading. -/
theorem pointsBy_toList (s : Scan) (f : Nat → Point) (g : Nat → Coords)
    (hf : ∀ i, f i = scanPointMm (g i)) :
    (s.pointsBy f).toList = s.points ((List.range s.beams.length).map g) := by
  rw [Scan.pointsBy, pointsGo_toList s f g hf]
  simp [Scan.points, List.range_eq_range']

/-- **`sourcePointsBy` is `sourcePoints`** under the coordinates
`g 0, …, g (n - 1)`, or none. -/
theorem sourcePointsBy_eq (cfg : Config) (st : State) (now : Nat) (n : Option Nat)
    (f : Nat → Point) (g : Nat → Coords) (hf : ∀ i, f i = scanPointMm (g i)) :
    sourcePointsBy cfg st now n f =
      (sourcePoints cfg st now (n.map fun n => (List.range n).map g)).map List.toArray := by
  unfold sourcePointsBy sourcePoints
  split
  · rfl
  · cases st.scan with
    | none => simp only; split <;> rfl
    | some s =>
      simp only
      split
      · rfl
      · cases n with
        | none => simp only [Option.map_none]; split <;> rfl
        | some k =>
          simp only [Option.map_some, Scan.fits, List.length_map, List.length_range]
          by_cases hk : k = s.beams.length
          · subst hk
            simp only [beq_self_eq_true, if_true, Option.map_some, Option.some.injEq]
            rw [← pointsBy_toList s f g hf]
          · have : (k == s.beams.length) = false := by simpa using hk
            simp only [this, Bool.false_eq_true, if_false]
            split <;> rfl

/-! ## The bounding box -/

private theorem abs_le_two_of_sq {a b : ℤ} (h : a * a + b * b ≤ 5) : -2 ≤ a ∧ a ≤ 2 := by
  have hb := mul_self_nonneg b
  constructor <;> nlinarith

/-- The algebra of the box test: a point whose foot on the line through `a`
and `b` lies strictly between them, and whose distance from that line is at
most √5, is less than 3 beyond both end points along the x axis. With the
vectors `ab` and `ap`. -/
private theorem near_aux (abx aby apx apy : ℤ)
    (h0 : 0 < abx * apx + aby * apy)
    (h1 : abx * apx + aby * apy < abx * abx + aby * aby)
    (h2 : (abx * apy - aby * apx) * (abx * apy - aby * apx) ≤ 5 * (abx * abx + aby * aby))
    (ha : 3 ≤ apx) (hb : 3 ≤ apx - abx) : False := by
  obtain ⟨d, hd⟩ : ∃ d, d = abx * apx + aby * apy := ⟨_, rfl⟩
  obtain ⟨L, hL⟩ : ∃ L, L = abx * abx + aby * aby := ⟨_, rfl⟩
  obtain ⟨c, hc⟩ : ∃ c, c = abx * apy - aby * apx := ⟨_, rfl⟩
  have key : (L - d) * apx + d * (apx - abx) = -(aby * c) := by subst hd hL hc; ring
  have h4 : aby * aby ≤ L := by rw [hL]; nlinarith [mul_self_nonneg abx]
  rw [← hd] at h0 h1
  rw [← hL] at h1 h2
  rw [← hc] at h2
  have h3 : 3 * L ≤ -(aby * c) := by
    rw [← key]
    nlinarith [mul_le_mul_of_nonneg_left ha (by linarith : (0 : ℤ) ≤ L - d),
      mul_le_mul_of_nonneg_left hb (le_of_lt h0)]
  have hLpos : 0 < L := by linarith
  have h5 : (aby * c) * (aby * c) ≤ 5 * L * L := by
    calc (aby * c) * (aby * c) = (aby * aby) * (c * c) := by ring
      _ ≤ L * (c * c) := mul_le_mul_of_nonneg_right h4 (mul_self_nonneg c)
      _ ≤ L * (5 * L) := mul_le_mul_of_nonneg_left h2 (le_of_lt hLpos)
      _ = 5 * L * L := by ring
  have h6 : (3 * L) * (3 * L) ≤ (aby * c) * (aby * c) := by
    nlinarith [mul_le_mul h3 h3 (by linarith) (by linarith)]
  nlinarith [mul_pos hLpos hLpos]

/-- **A point near a segment is near the segment's box.** If `p` is within
the margin of D4 (√5 mm) of the segment from `a` to `b`, then on each axis it
is at most 2 mm outside the range the two end points span. -/
theorem nearSegment_box (p a b : Point) (h : nearSegment p a b = true) :
    min a.x b.x - 2 ≤ p.x ∧ p.x ≤ max a.x b.x + 2 ∧
      min a.y b.y - 2 ≤ p.y ∧ p.y ≤ max a.y b.y + 2 := by
  have m1 := min_le_left a.x b.x
  have m2 := min_le_right a.x b.x
  have m3 := le_max_left a.x b.x
  have m4 := le_max_right a.x b.x
  have m5 := min_le_left a.y b.y
  have m6 := min_le_right a.y b.y
  have m7 := le_max_left a.y b.y
  have m8 := le_max_right a.y b.y
  simp only [nearSegment, polygonMarginSq, Bool.or_eq_true, Bool.and_eq_true,
    decide_eq_true_eq] at h
  rcases h with (h | h) | ⟨⟨h0, h1⟩, h2⟩
  · have h := of_decide_eq_true h
    have hx := abs_le_two_of_sq h
    have hy := abs_le_two_of_sq (a := p.y - a.y) (b := p.x - a.x) (by linarith)
    refine ⟨?_, ?_, ?_, ?_⟩ <;> omega
  · have h := of_decide_eq_true h
    have hx := abs_le_two_of_sq h
    have hy := abs_le_two_of_sq (a := p.y - b.y) (b := p.x - b.x) (by linarith)
    refine ⟨?_, ?_, ?_, ?_⟩ <;> omega
  · have h2 := of_decide_eq_true h2
    refine ⟨?_, ?_, ?_, ?_⟩
    · by_contra hc
      exact near_aux (-(b.x - a.x)) (b.y - a.y) (-(p.x - a.x)) (p.y - a.y)
        (by linarith) (by linarith) (by linarith) (by omega) (by omega)
    · by_contra hc
      exact near_aux (b.x - a.x) (b.y - a.y) (p.x - a.x) (p.y - a.y)
        (by linarith) (by linarith) (by linarith) (by omega) (by omega)
    · by_contra hc
      exact near_aux (-(b.y - a.y)) (b.x - a.x) (-(p.y - a.y)) (p.x - a.x)
        (by linarith) (by linarith) (by linarith) (by omega) (by omega)
    · by_contra hc
      exact near_aux (b.y - a.y) (b.x - a.x) (p.y - a.y) (p.x - a.x)
        (by linarith) (by linarith) (by linarith) (by omega) (by omega)

private theorem foldl_box (vs : List Point) (b0 : Box) :
    let b := vs.foldl
      (fun b p => ⟨min b.minX p.x, max b.maxX p.x, min b.minY p.y, max b.maxY p.y⟩) b0
    (b.minX ≤ b0.minX ∧ b0.maxX ≤ b.maxX ∧ b.minY ≤ b0.minY ∧ b0.maxY ≤ b.maxY) ∧
      ∀ v ∈ vs, b.minX ≤ v.x ∧ v.x ≤ b.maxX ∧ b.minY ≤ v.y ∧ v.y ≤ b.maxY := by
  induction vs generalizing b0 with
  | nil => simp
  | cons w ws ih =>
    simp only [List.foldl_cons, List.mem_cons, forall_eq_or_imp]
    obtain ⟨⟨h1, h2, h3, h4⟩, h5⟩ := ih ⟨min b0.minX w.x, max b0.maxX w.x, min b0.minY w.y,
      max b0.maxY w.y⟩
    simp only at h1 h2 h3 h4
    have m1 := min_le_left b0.minX w.x
    have m2 := min_le_right b0.minX w.x
    have m3 := le_max_left b0.maxX w.x
    have m4 := le_max_right b0.maxX w.x
    have m5 := min_le_left b0.minY w.y
    have m6 := min_le_right b0.minY w.y
    have m7 := le_max_left b0.maxY w.y
    have m8 := le_max_right b0.maxY w.y
    exact ⟨⟨by omega, by omega, by omega, by omega⟩, ⟨by omega, by omega, by omega, by omega⟩, h5⟩

/-- Every vertex is in the box of the vertices. -/
theorem boxOf_spec (vs : List Point) (v : Point) (hv : v ∈ vs) :
    (boxOf vs).minX ≤ v.x ∧ v.x ≤ (boxOf vs).maxX ∧
      (boxOf vs).minY ≤ v.y ∧ v.y ≤ (boxOf vs).maxY := by
  cases vs with
  | nil => cases hv
  | cons w ws =>
    obtain ⟨⟨h1, h2, h3, h4⟩, h5⟩ := foldl_box ws ⟨w.x, w.x, w.y, w.y⟩
    simp only at h1 h2 h3 h4
    rcases List.mem_cons.mp hv with rfl | hv
    · exact ⟨h1, h2, h3, h4⟩
    · exact h5 v hv

/-- Both ends of an edge are vertices. -/
theorem mem_of_mem_edges (vs : List Point) (e : Point × Point) (he : e ∈ edges vs) :
    e.1 ∈ vs ∧ e.2 ∈ vs := by
  unfold edges at he
  cases hl : vs.getLast? with
  | none => simp [hl] at he
  | some l =>
    simp only [hl] at he
    obtain ⟨h1, h2⟩ := List.of_mem_zip (a := e.1) (b := e.2) he
    refine ⟨?_, h2⟩
    rcases List.mem_cons.mp h1 with h | h
    · rw [h]; exact List.mem_of_getLast? hl
    · exact h

/-- **The test that is skipped would have said "no".** A point within the
margin of the polygon's boundary is within 2 mm of the polygon's bounding
box. -/
theorem nearBoundary_near (vs : List Point) (p : Point) (h : nearBoundary vs p = true) :
    (boxOf vs).near p = true := by
  simp only [nearBoundary, List.any_eq_true] at h
  obtain ⟨e, he, hn⟩ := h
  obtain ⟨h1, h2⟩ := mem_of_mem_edges vs e he
  obtain ⟨a1, a2, a3, a4⟩ := boxOf_spec vs e.1 h1
  obtain ⟨b1, b2, b3, b4⟩ := boxOf_spec vs e.2 h2
  obtain ⟨n1, n2, n3, n4⟩ := nearSegment_box p e.1 e.2 hn
  have m1 := le_min a1 b1
  have m2 := max_le a2 b2
  have m3 := le_min a3 b3
  have m4 := max_le a4 b4
  simp only [Box.near, Bool.and_eq_true, decide_eq_true_eq]
  exact ⟨⟨⟨by omega, by omega⟩, by omega⟩, by omega⟩

/-! ## Zones -/

/-- The polygon test with the edges and the box computed beforehand is
`Shape.contains`. -/
theorem containsEdges_eq (vs : List Point) (p : Point) :
    containsEdges (edges vs) (boxOf vs) p = (Shape.polygon vs).contains p := by
  simp only [containsEdges, Shape.contains, insidePolygon]
  congr 1
  cases hn : nearBoundary vs p with
  | false =>
    have : (edges vs).any (fun e => nearSegment p e.1 e.2) = false := hn
    rw [this, Bool.and_false]
  | true =>
    have : (edges vs).any (fun e => nearSegment p e.1 e.2) = true := hn
    rw [this, nearBoundary_near vs p hn]
    rfl

/-- Counting in an array with the shape prepared is counting with
`Shape.contains`. -/
theorem countIn_eq (s : Shape) (pts : Array Point) :
    s.countIn pts = pts.toList.countP s.contains := by
  cases s with
  | circle r =>
    simp only [Shape.countIn, ← Array.countP_toList]
    rfl
  | polygon vs =>
    simp only [Shape.countIn, ← Array.countP_toList]
    congr 1
    funext p
    exact containsEdges_eq vs p

/-- **The fast count is the count.** -/
theorem countFast_eq (z : Zone) (pts : Array Point) : z.countFast pts = z.count pts.toList := by
  unfold Zone.countFast Zone.count
  cases hs : z.shape with
  | fixed s => simp only [countIn_eq]; rfl
  | topic => exact (List.countP_eq_zero.mpr (fun p _ => by simp [ZoneShape.contains])).symm

theorem triggeredFast_eq (z : Zone) (pts : Array Point) :
    z.triggeredFast pts = z.triggered pts.toList := by
  simp only [Zone.triggeredFast, Zone.triggered, countFast_eq]

theorem firstStopFast_eq (cfg : Config) (pts : Array Point) :
    firstStopFast cfg pts = firstStop cfg pts.toList := by
  simp only [firstStopFast, firstStop, triggeredFast_eq]

theorem slowdownStepFast_eq (pts : Array Point) :
    slowdownStepFast pts = slowdownStep pts.toList := by
  funext best zi
  unfold slowdownStepFast slowdownStep
  cases zi.1.action with
  | stop => rfl
  | slowdown r => simp only [triggeredFast_eq]

theorem bestSlowdownFast_eq (cfg : Config) (pts : Array Point) :
    bestSlowdownFast cfg pts = bestSlowdown cfg pts.toList := by
  simp only [bestSlowdownFast, bestSlowdown, slowdownStepFast_eq]

/-- **The fast zone decision is the zone decision.** -/
theorem zonesActionFast_eq (cfg : Config) (pts : Array Point) :
    zonesActionFast cfg pts = zonesAction cfg pts.toList := by
  unfold zonesActionFast zonesAction
  rw [firstStopFast_eq, bestSlowdownFast_eq]
  cases firstStop cfg pts.toList with
  | some k => rfl
  | none => rcases bestSlowdown cfg pts.toList with ⟨r, _ | k⟩ <;> rfl

/-! ## A command -/

/-- **The fast command step is `step`.** For every configuration, state,
time and velocity: if `f i` is the converted pair `g i` for every index,
`stepCmdFast` returns the state and the output that `step` returns for the
command event carrying the coordinates `g 0, …, g (n - 1)` (or none). -/
theorem stepCmdFast_eq (cfg : Config) (st : State) (now : Nat) (v : Twist) (n : Option Nat)
    (f : Nat → Point) (g : Nat → Coords) (zoneVerts : List (Option (List Coords)))
    (hf : ∀ i, f i = scanPointMm (g i)) :
    stepCmdFast cfg st now v n f zoneVerts =
      step cfg st (.cmd now v (n.map fun n => (List.range n).map g) zoneVerts) := by
  have hs := sourcePointsBy_eq cfg st now n f g hf
  cases h : sourcePoints cfg st now (n.map fun n => (List.range n).map g) with
  | none =>
    rw [h] at hs
    simp only [Option.map_none] at hs
    simp only [stepCmdFast, step, evaluate, shapesAfter, liveConfig, hs, h]
  | some pts =>
    rw [h] at hs
    simp only [Option.map_some] at hs
    simp only [stepCmdFast, step, evaluate, shapesAfter, liveConfig, hs, h, zonesActionFast_eq]

end ROSlean.CollisionMonitor

namespace ROSlean.CollisionMonitor.FFI

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI

/-! ## Decoding a coordinate from its halves -/

/-- The conversion from the two halves of a pattern is `coordMm` of the
pattern. -/
theorem partsMm_eq (hi lo : Nat) (hhi : hi < 2 ^ 32) (hlo : lo < 2 ^ 32) :
    partsMm hi lo = coordMm (hi * 0x100000000 + lo) := by
  unfold partsMm coordMm
  have h1 : (hi * 0x100000000 + lo) % 0x8000000000000000 = hi % 0x80000000 * 0x100000000 + lo := by
    omega
  have h2 : F64.neg (hi * 0x100000000 + lo) = decide (0x80000000 ≤ hi) := by
    unfold F64.neg
    by_cases h : 0x80000000 ≤ hi
    · have : (hi * 0x100000000 + lo) / 0x8000000000000000 % 2 = 1 := by omega
      simp [this, h]
    · have : (hi * 0x100000000 + lo) / 0x8000000000000000 % 2 = 0 := by omega
      simp [this, h]
  rw [h1, h2]

theorem coordMmAt_eq (a : ByteArray) (o : Nat) : coordMmAt a o = coordMm (u64At a o) := by
  unfold coordMmAt u64At
  exact partsMm_eq _ _ (u32At_lt a (o + 4)) (u32At_lt a o)

/-- **The point read from the bytes is the model's point** of the record's
coordinates. -/
theorem pointAt_eq (a : ByteArray) (i : Nat) : pointAt a i = scanPointMm (coordsAt a i) := by
  simp only [pointAt, scanPointMm, coordsAt, coordMmAt_eq]

/-! ## The exported command call -/

/-- The record array a monitor keeps holds the coordinates it remembers. -/
def Synced (m : Monitor) : Prop := m.coords = m.raw.map decodeCoords

/-- **On a synced monitor the fast command is `Monitor.apply`** on the
command event that carries the remembered coordinates. -/
theorem applyCmdFast_eq (m : Monitor) (now : Nat) (v : Twist) (h : Synced m) :
    m.applyCmdFast now v = m.apply (.cmd now v m.coords []) := by
  unfold Monitor.applyCmdFast Monitor.apply
  rw [h]
  cases m.raw with
  | none =>
    simp only [Option.map_none]
    rw [stepCmdFast_eq m.cfg m.st now v none (fun _ => ⟨0, 0⟩) (fun _ => (0, 0)) []
      (fun _ => by decide)]
    rfl
  | some a =>
    simp only [Option.map_some]
    rw [stepCmdFast_eq m.cfg m.st now v (some (a.size / recordSize)) (pointAt a) (coordsAt a) []
      (pointAt_eq a)]
    rfl

/-- **The exported command call is `cmOnCmd`** on a synced monitor. -/
theorem cmOnCmdFast_eq (m : Monitor) (now lx ly lz ax ay az : UInt64) (h : Synced m) :
    cmOnCmdFast m now lx ly lz ax ay az = cmOnCmd m now lx ly lz ax ay az :=
  applyCmdFast_eq m _ _ h

theorem synced_cmNew (b : Builder) : Synced (cmNew b) := rfl

theorem synced_cmOnScan (m : Monitor) (now : UInt64) (stamp : Int64) (usable : UInt8)
    (rmin rmax : UInt32) (a : ByteArray) : Synced (cmOnScan m now stamp usable rmin rmax a) := by
  unfold Synced cmOnScan
  simp only
  split <;> rfl

theorem synced_exec (m : Monitor) (c : Call) (h : Synced m) : Synced (c.exec m) := by
  cases c with
  | scan now stamp usable rmin rmax a => exact synced_cmOnScan m now stamp usable rmin rmax a
  | cmd now lx ly lz ax ay az =>
    show (m.apply _).coords = (m.apply _).raw.map decodeCoords
    rw [(apply_cfg m _).2.2, apply_raw]
    exact h

/-- **Every monitor the calls can produce is synced.** -/
theorem synced_after (b : Builder) (cs : List Call) : Synced (cs.foldl Call.exec (cmNew b)) := by
  suffices h : ∀ m, Synced m → Synced (cs.foldl Call.exec m) from h _ (synced_cmNew b)
  induction cs with
  | nil => exact fun m h => h
  | cons c cs ih => exact fun m h => ih _ (synced_exec m c h)

/-- A call as C makes it: the command call is the exported one. -/
def Call.execFast (m : Monitor) : Call → Monitor
  | .scan now stamp usable rmin rmax a => cmOnScan m now stamp usable rmin rmax a
  | .cmd now lx ly lz ax ay az => cmOnCmdFast m now lx ly lz ax ay az

/-- **What C calls is what the theorems are about.** After any sequence of
scan and command calls on a new monitor, the exported functions have
produced exactly the monitor that the definitions the theorems are stated on
(`Call.exec`: `cmOnScan`, `cmOnCmd`) produce: the same state, the same
output, every field. -/
theorem execFast_eq (b : Builder) (cs : List Call) :
    cs.foldl Call.execFast (cmNew b) = cs.foldl Call.exec (cmNew b) := by
  suffices h : ∀ m, Synced m → cs.foldl Call.execFast m = cs.foldl Call.exec m from
    h _ (synced_cmNew b)
  induction cs with
  | nil => exact fun m _ => rfl
  | cons c cs ih =>
    intro m h
    have hc : Call.execFast m c = Call.exec m c := by
      cases c with
      | scan now stamp usable rmin rmax a => rfl
      | cmd now lx ly lz ax ay az => exact cmOnCmdFast_eq m now lx ly lz ax ay az h
    rw [List.foldl_cons, List.foldl_cons, hc]
    exact ih _ (synced_exec m c h)

end ROSlean.CollisionMonitor.FFI
