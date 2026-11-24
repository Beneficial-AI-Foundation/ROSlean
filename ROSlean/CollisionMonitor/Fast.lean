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
* `word64At_toNat`, `coordMagMm64_toNat`, `coordMm64_eq`, `coordMmAt_eq`,
  `pointAt_eq`: reading a coordinate's eight bytes into a machine word and
  converting it there is `coordMm` of the 64-bit pattern;
* `cmOnCmdFast_eq`, `execFast_eq`: the exported command call returns the
  monitor that `cmOnCmd` returns, on every monitor and so after any sequence
  of calls.

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

/-! ## Decoding a coordinate on machine words -/

/-- The machine word read from eight bytes is the pattern `u64At` reads. -/
theorem word64At_toNat (a : ByteArray) (o : Nat) : (word64At a o).toNat = u64At a o := by
  unfold word64At
  split
  · rename_i h
    have hb : ∀ k (hk : k < 8), byteAt a (o + k) = (a[o + k]'(by omega)).toNat := by
      intro k hk
      simp [byteAt, show o + k < a.size by omega]
    have h0 := hb 0 (by omega)
    have h1 := hb 1 (by omega)
    have h2 := hb 2 (by omega)
    have h3 := hb 3 (by omega)
    have h4 := hb 4 (by omega)
    have h5 := hb 5 (by omega)
    have h6 := hb 6 (by omega)
    have h7 := hb 7 (by omega)
    simp only [Nat.add_zero] at h0
    have l0 := (a[o]'(by omega)).toNat_lt
    have l1 := (a[o + 1]'(by omega)).toNat_lt
    have l2 := (a[o + 2]'(by omega)).toNat_lt
    have l3 := (a[o + 3]'(by omega)).toNat_lt
    have l4 := (a[o + 4]'(by omega)).toNat_lt
    have l5 := (a[o + 5]'(by omega)).toNat_lt
    have l6 := (a[o + 6]'(by omega)).toNat_lt
    have l7 := (a[o + 7]'(by omega)).toNat_lt
    simp only [u64At, u32At, Nat.add_assoc, Nat.reduceAdd, h0, h1, h2, h3, h4, h5, h6, h7,
      UInt64.toNat_add, UInt64.toNat_mul, UInt8.toNat_toUInt64, UInt64.toNat_ofNat]
    omega
  · rw [UInt64.toNat_ofNat']
    exact Nat.mod_eq_of_lt (u64At_lt a o)

/-- The magnitude computed on a machine word is `coordMagMm` of the
pattern. -/
theorem coordMagMm64_toNat (mag : UInt64) : (coordMagMm64 mag).toNat = coordMagMm mag.toNat := by
  have hn := mag.toNat_lt
  have he : (mag / 0x10000000000000 % 0x800).toNat = mag.toNat / 0x10000000000000 % 0x800 := by
    simp [UInt64.toNat_div, UInt64.toNat_mod]
  have hf : (mag % 0x10000000000000).toNat = mag.toNat % 0x10000000000000 := by
    simp [UInt64.toNat_mod]
  unfold coordMagMm64 coordMagMm
  simp only [F64.expField, F64.mant, F64.frac, F64.shift, F64.bias, coordLimit, ← he, ← hf]
  have hfl : (mag % 0x10000000000000).toNat < 0x10000000000000 := by rw [hf]; omega
  have hel : (mag / 0x10000000000000 % 0x800).toNat < 0x800 := by rw [he]; omega
  generalize mag / 0x10000000000000 % 0x800 = e at hel ⊢
  generalize mag % 0x10000000000000 = f at hfl ⊢
  by_cases h1 : 1033 ≤ e.toNat
  · have h1' : (1033 : UInt64) ≤ e := UInt64.le_iff_toNat_le.mpr h1
    simp [h1, h1']
  · have h1' : ¬ (1033 : UInt64) ≤ e := fun h => h1 (UInt64.le_iff_toNat_le.mp h)
    simp only [h1, h1', if_false]
    by_cases h0 : e.toNat = 0
    · have h0' : e < 1 := UInt64.lt_iff_toNat_lt.mpr (by simp [h0])
      simp only [h0, h0', if_true]
      have hz : (f.toNat * 1000) >>> (1075 - 1) = 0 := by
        rw [Nat.shiftRight_eq_div_pow]
        apply Nat.div_eq_of_lt
        calc f.toNat * 1000 < 2 ^ 63 := by omega
          _ ≤ 2 ^ (1075 - 1) := Nat.pow_le_pow_right (by norm_num) (by norm_num)
      rw [hz]
      simp
    · have h0' : ¬ e < 1 := fun h => h0 (by have := UInt64.lt_iff_toNat_lt.mp h; simpa using this)
      simp only [h0, h0', if_false]
      have hk : (1075 - e).toNat = 1075 - e.toNat := by
        rw [UInt64.toNat_sub_of_le _ _ (UInt64.le_iff_toNat_le.mpr (by simp; omega))]
        rfl
      have hm : ((f + 4503599627370496) * 1000).toNat = (f.toNat + 4503599627370496) * 1000 := by
        simp only [UInt64.toNat_mul, UInt64.toNat_add, UInt64.toNat_ofNat]
        omega
      rw [← hk, ← hm]
      have hml : ((f + 4503599627370496) * 1000).toNat < 2 ^ 63 := by rw [hm]; omega
      generalize 1075 - e = sh at hk ⊢
      generalize (f + 4503599627370496) * 1000 = m at hm hml ⊢
      by_cases hs : 64 ≤ sh.toNat
      · have hs' : (64 : UInt64) ≤ sh := UInt64.le_iff_toNat_le.mpr hs
        have hz : m.toNat >>> sh.toNat = 0 := by
          rw [Nat.shiftRight_eq_div_pow]
          apply Nat.div_eq_of_lt
          calc m.toNat < 2 ^ 63 := hml
            _ ≤ 2 ^ sh.toNat := Nat.pow_le_pow_right (by norm_num) (by omega)
        rw [hz]
        simp [hs']
      · have hs' : ¬ (64 : UInt64) ≤ sh := fun h => hs (UInt64.le_iff_toNat_le.mp h)
        simp only [hs', if_false]
        have hq : (m >>> sh).toNat = m.toNat >>> sh.toNat := by
          rw [UInt64.toNat_shiftRight]
          congr 1
          exact Nat.mod_eq_of_lt (by omega)
        rw [← hq]
        generalize m >>> sh = q
        by_cases hq1 : 1000000 ≤ q.toNat
        · have hq1' : (1000000 : UInt64) ≤ q := UInt64.le_iff_toNat_le.mpr hq1
          simp only [hq1', if_true]
          rw [Nat.min_def]
          split <;> simp <;> omega
        · have hq1' : ¬ (1000000 : UInt64) ≤ q := fun h => hq1 (UInt64.le_iff_toNat_le.mp h)
          simp only [hq1', if_false]
          rw [Nat.min_def]
          split <;> omega

/-- **The conversion on a machine word is `coordMm`** of the pattern. -/
theorem coordMm64_eq (b : UInt64) : coordMm64 b = coordMm b.toNat := by
  have hn := b.toNat_lt
  have hmag : (b % 0x8000000000000000).toNat = b.toNat % 0x8000000000000000 := by
    simp [UInt64.toNat_mod]
  have hneg : decide ((0x8000000000000000 : UInt64) ≤ b) = F64.neg b.toNat := by
    unfold F64.neg
    by_cases h : 0x8000000000000000 ≤ b.toNat
    · have h' : (0x8000000000000000 : UInt64) ≤ b := UInt64.le_iff_toNat_le.mpr h
      have : b.toNat / 0x8000000000000000 % 2 = 1 := by omega
      simp [h', this]
    · have h' : ¬ (0x8000000000000000 : UInt64) ≤ b := fun h' => h (UInt64.le_iff_toNat_le.mp h')
      have : b.toNat / 0x8000000000000000 % 2 = 0 := by omega
      simp [h', this]
  have hnan : (b % 0x8000000000000000 / 0x10000000000000 % 0x800 == 0x7FF &&
      b % 0x8000000000000000 % 0x10000000000000 != 0) =
      F64.isNaN (b.toNat % 0x8000000000000000) := by
    have he : (b % 0x8000000000000000 / 0x10000000000000 % 0x800).toNat =
        b.toNat % 0x8000000000000000 / 0x10000000000000 % 0x800 := by
      simp [UInt64.toNat_div, UInt64.toNat_mod]
    have hf : (b % 0x8000000000000000 % 0x10000000000000).toNat =
        b.toNat % 0x8000000000000000 % 0x10000000000000 := by
      simp [UInt64.toNat_mod]
    unfold F64.isNaN F64.expField F64.frac
    rw [← he, ← hf]
    generalize b % 0x8000000000000000 / 0x10000000000000 % 0x800 = e
    generalize b % 0x8000000000000000 % 0x10000000000000 = f
    have h1 : (e == 0x7FF) = (e.toNat == 0x7FF) := by
      rw [Bool.eq_iff_iff]
      simp [← UInt64.toNat_inj]
    have h2 : (f != 0) = (f.toNat != 0) := by
      rw [Bool.eq_iff_iff]
      simp [← UInt64.toNat_inj]
    rw [h1, h2]
  unfold coordMm64 coordMm coordMmOfParts
  simp only [coordMagMm64_toNat, hmag, hnan]
  rw [← hneg]

theorem coordMmAt_eq (a : ByteArray) (o : Nat) : coordMmAt a o = coordMm (u64At a o) := by
  rw [coordMmAt, coordMm64_eq, word64At_toNat]

/-- **The point read from the bytes is the model's point** of the pair's
coordinates. -/
theorem pointAt_eq (a : ByteArray) (i : Nat) : pointAt a i = scanPointMm (pairAt a i) := by
  simp only [pointAt, scanPointMm, pairAt, coordMmAt_eq]

/-! ## The exported command call -/

/-- **The fast command is `Monitor.apply`** on the command event that
carries the coordinate pairs of the array (or none) and the zone vertices. -/
theorem applyCmdFast_eq (m : Monitor) (now : Nat) (v : Twist) (hasPoints : Bool)
    (points : ByteArray) (zoneVerts : List (Option (List Coords))) :
    m.applyCmdFast now v hasPoints points zoneVerts =
      m.apply (.cmd now v (if hasPoints then some (decodePairs points) else none) zoneVerts) := by
  unfold Monitor.applyCmdFast Monitor.apply
  rw [stepCmdFast_eq m.cfg m.st now v (if hasPoints then some (points.size / pairSize) else none)
    (pointAt points) (pairAt points) zoneVerts (pointAt_eq points)]
  cases hasPoints <;> rfl

/-- **The exported command call is `cmOnCmd`**, on every monitor and for all
arguments: the same state, the same output, every field. -/
theorem cmOnCmdFast_eq (m : Monitor) (now lx ly lz ax ay az : UInt64) (hasPoints : UInt8)
    (points zoneVerts : ByteArray) :
    cmOnCmdFast m now lx ly lz ax ay az hasPoints points zoneVerts =
      cmOnCmd m now lx ly lz ax ay az hasPoints points zoneVerts := by
  unfold cmOnCmdFast cmOnCmd cmdEvent pointsOfRaw
  rw [applyCmdFast_eq]

/-- A call as C makes it: the command call is the exported one. -/
def Call.execFast (m : Monitor) : Call → Monitor
  | .lifecycle t => cmOnLifecycle m t
  | .scan now stamp rmin rmax a => cmOnScan m now stamp rmin rmax a
  | .shape zone foreign hasVerts verts => cmOnShape m zone foreign hasVerts verts
  | .cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    cmOnCmdFast m now lx ly lz ax ay az hasPoints points zoneVerts

/-- **What C calls is what the theorems are about.** Each exported call
returns the monitor that the definition the theorems are stated on returns
(`Call.exec`), on every monitor; so does any sequence of calls. -/
theorem execFast_eq_exec : Call.execFast = Call.exec := by
  funext m c
  cases c with
  | lifecycle t => rfl
  | scan now stamp rmin rmax a => rfl
  | shape zone foreign hasVerts verts => rfl
  | cmd now lx ly lz ax ay az hasPoints points zoneVerts =>
    exact cmOnCmdFast_eq m now lx ly lz ax ay az hasPoints points zoneVerts

theorem execFast_eq (m : Monitor) (cs : List Call) :
    cs.foldl Call.execFast m = cs.foldl Call.exec m := by
  rw [execFast_eq_exec]

/-- **End to end on the exported call.** `cm_readBack_after` for the command
call C makes: after any sequence of exported calls, the five numbers read
after the exported command call determine the model's output for its
event. -/
theorem cm_readBack_after_fast (b : Builder) (cs : List Call) (now lx ly lz ax ay az : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray)
    (hok : cmOk (cmNew b) = true) (hz : b.zones.length ≤ 2 ^ 32) :
    let m := cmOnCmdFast (cs.foldl Call.execFast (cmNew b)) now lx ly lz ax ay az hasPoints points
      zoneVerts
    outputOfCodes (cmOutKind m) (cmOutReport m) (cmOutZone m).toNat (cmOutRatio m).toNat
        (cmOutReportZone m).toNat =
      (step b.cfg (run b.cfg (State.init b.cfg) (events cs)).1
        (cmdEvent now lx ly lz ax ay az hasPoints points zoneVerts)).2 := by
  rw [execFast_eq, cmOnCmdFast_eq]
  exact cm_readBack_after b cs now lx ly lz ax ay az hasPoints points zoneVerts hok hz

end ROSlean.CollisionMonitor.FFI
