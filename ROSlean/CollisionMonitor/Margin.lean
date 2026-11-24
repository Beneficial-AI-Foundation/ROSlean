import Mathlib.Tactic
import ROSleanCore.CollisionMonitor
import ROSlean.CollisionMonitor.Convert

/-!
# Collision monitor: the conversions of base-frame coordinates, and the margin

Since task T05b the sensor-to-base transform is applied by the C++ shim in
`double`, as upstream applies it (option A of
`docs/collision_monitor/SPEC_NAV2.md`, section 5), and the model receives
coordinates **in the base frame** as binary64 patterns. This file proves
what the model does with them, against real numbers, and how much room the
polygon margin (D4) leaves for a difference between the shim's coordinates
and upstream's.

* `sval64 b`: the number a finite binary64 pattern denotes, with its sign.
* **Scan points** (`coordMm_eq_truncReal`, `coordMm_sub_lt`,
  `scanPointMm_sub_lt`): each coordinate of `scanPointMm c` is the value the
  shim handed over, in millimetres, truncated toward zero and saturated at
  `±coordLimit`; unless saturated it is less than 1 mm from it.
* **Topic-zone vertices** (`vertexCoordMm_sub_le`, `vertexPointMm_sub_le`,
  `vertexCoordMm_finite_of_lt`): a vertex coordinate that is within the
  coordinate limit is finite and within half a millimetre of the value the
  shim handed over. A coordinate that is not finite, or is 1024 m or more,
  is never within the limit, so such a vertex is always refused.
* **The margin with slack** (`contains_of_close`): the quantitative half of
  the safe-direction argument for any bound `dv` on the displacement of a
  vertex and `dp` on that of a point, per axis, with `2·(dv + dp)² ≤ 5`.
  `contains_of_rounded` is the case `dv = 1/2`, `dp = 1` of SPEC.md.
* **The margin for option A** (`contains_of_shim`, `contains_of_shim_fixed`,
  `circle_safe_shim`): if upstream's vertices and upstream's point differ
  from the shim's by at most `δ` per axis, and upstream's point lies on the
  boundary of upstream's polygon, the model counts the point. For a topic
  zone `δ ≤ 1/25` mm suffices, for a fixed polygon `δ ≤ 2/25` mm, for a
  circle `δ ≤ 1/2` mm.

What is **not** proved here, and is not provable in Lean without a model of
binary64 arithmetic and of the two compilers: that the shim's coordinates and
upstream's do differ by no more than such a `δ`. The shim evaluates upstream's
formula (N25) in `double` without fused multiply-add; upstream's arm64 binary
fuses. Both are within a few rounding errors of the exact image, about
10⁻⁸ mm for coordinates below 1000 m (SPEC_NAV2, section 5, and trusted item
T7). The theorems here say that 40,000 nm would still be enough.

The exact transform in integer arithmetic that task T05 had in this place
(`Xform.pointMm`, `pointMm_x_eq` and the theorems that followed from it) is
gone with option B.
-/

namespace ROSlean.CollisionMonitor.Margin

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor ROSlean.CollisionMonitor.Convert

/-! ## The number a pattern denotes, with its sign -/

/-- The number a finite binary64 pattern denotes. -/
noncomputable def sval64 (b : Nat) : ℝ := if F64.neg b = true then -val64 b else val64 b

theorem abs_sval64 (b : Nat) : |sval64 b| = val64 b := by
  unfold sval64
  split
  · rw [abs_neg, abs_of_nonneg (val64_nonneg b)]
  · exact abs_of_nonneg (val64_nonneg b)

/-! ## Truncation -/

/-- A real number truncated toward zero to an integer and saturated at
`±coordLimit`. -/
noncomputable def truncReal (r : ℝ) : ℤ :=
  if r < 0 then -((min coordLimit ⌊|r|⌋₊ : ℕ) : ℤ) else ((min coordLimit ⌊|r|⌋₊ : ℕ) : ℤ)

theorem natAbs_truncReal (r : ℝ) : (truncReal r).natAbs = min coordLimit ⌊|r|⌋₊ := by
  unfold truncReal
  split
  · rw [Int.natAbs_neg, Int.natAbs_natCast]
  · rw [Int.natAbs_natCast]

/-- Truncation never moves a value away from zero. -/
theorem abs_truncReal_le (r : ℝ) : |((truncReal r : ℤ) : ℝ)| ≤ |r| := by
  rw [abs_cast_eq_natAbs, natAbs_truncReal]
  calc ((min coordLimit ⌊|r|⌋₊ : ℕ) : ℝ) ≤ (⌊|r|⌋₊ : ℝ) := by exact_mod_cast Nat.min_le_right _ _
    _ ≤ |r| := Nat.floor_le (abs_nonneg r)

/-- Unless it saturates, truncation loses less than 1. -/
theorem abs_truncReal_gt (r : ℝ) (h : (truncReal r).natAbs < coordLimit) :
    |r| < |((truncReal r : ℤ) : ℝ)| + 1 := by
  rw [abs_cast_eq_natAbs]
  rw [natAbs_truncReal] at h ⊢
  have hm : min coordLimit ⌊|r|⌋₊ = ⌊|r|⌋₊ := by omega
  rw [hm]
  exact Nat.lt_floor_add_one _

/-- Truncation keeps the sign. -/
theorem truncReal_sign (r : ℝ) :
    (0 ≤ r → 0 ≤ truncReal r) ∧ (r ≤ 0 → truncReal r ≤ 0) := by
  unfold truncReal
  constructor
  · intro h
    rw [if_neg (by linarith)]
    exact Int.natCast_nonneg _
  · intro h
    split
    · simp
    · have h0 : r = 0 := by linarith
      simp [h0]

/-- Unless it saturates, the truncated value is within 1 of the real one, on
the side of zero. -/
theorem truncReal_sub_lt (r : ℝ) (h : (truncReal r).natAbs < coordLimit) :
    |r - ((truncReal r : ℤ) : ℝ)| < 1 := by
  have h1 := abs_truncReal_le r
  have h2 := abs_truncReal_gt r h
  have hs := truncReal_sign r
  rcases le_total 0 r with hr | hr
  · have ht : (0 : ℝ) ≤ ((truncReal r : ℤ) : ℝ) := by exact_mod_cast hs.1 hr
    rw [abs_of_nonneg hr] at h1 h2
    rw [abs_of_nonneg ht] at h1 h2
    rw [abs_lt]
    constructor <;> linarith
  · have ht : ((truncReal r : ℤ) : ℝ) ≤ 0 := by exact_mod_cast hs.2 hr
    rw [abs_of_nonpos hr] at h1 h2
    rw [abs_of_nonpos ht] at h1 h2
    rw [abs_lt]
    constructor <;> linarith

/-! ## Scan points -/

/-- `coordMm` is the value in millimetres, truncated toward zero and
saturated. -/
theorem coordMm_eq_truncReal (b : Nat) (hf : F64.finite b = true) :
    coordMm b = truncReal (sval64 b * 1000) := by
  have hfin : F64.finite (b % 0x8000000000000000) = true := by rw [finite_mod]; exact hf
  have hnan : F64.isNaN (b % 0x8000000000000000) = false := by
    unfold F64.isNaN
    unfold F64.finite at hfin
    simp only [bne_iff_ne, ne_eq] at hfin
    simp [hfin]
  -- the magnitude
  have hmag : coordMagMm (b % 0x8000000000000000) = min coordLimit ⌊val64 b * 1000⌋₊ := by
    have hle := coordMagMm_le_val _ hfin
    rw [val64_mod] at hle
    by_cases hsat : coordMagMm (b % 0x8000000000000000) < coordLimit
    · have hgt := coordMagMm_gt_val _ hsat
      rw [val64_mod] at hgt
      have hfl : ⌊val64 b * 1000⌋₊ = coordMagMm (b % 0x8000000000000000) := by
        rw [Nat.floor_eq_iff (by have := val64_nonneg b; positivity)]
        exact ⟨hle, hgt⟩
      rw [hfl]
      omega
    · have heq : coordMagMm (b % 0x8000000000000000) = coordLimit :=
        le_antisymm (coordMagMm_le _) (by omega)
      rw [heq] at hle ⊢
      have : coordLimit ≤ ⌊val64 b * 1000⌋₊ := Nat.le_floor hle
      omega
  have hv := val64_nonneg b
  unfold coordMm coordMmOfParts truncReal sval64
  rw [hnan, hmag]
  cases hneg : F64.neg b
  · have h0 : ¬ (val64 b * 1000 < 0) := by nlinarith
    simp only [Bool.false_and, Bool.false_eq_true, ↓reduceIte, h0,
      abs_of_nonneg (show (0 : ℝ) ≤ val64 b * 1000 by nlinarith)]
  · simp only [Bool.not_false, Bool.and_self, ↓reduceIte, neg_mul, abs_neg,
      abs_of_nonneg (show (0 : ℝ) ≤ val64 b * 1000 by nlinarith), Left.neg_neg_iff]
    by_cases hz : 0 < val64 b * 1000
    · rw [if_pos hz]
    · have h0 : val64 b * 1000 = 0 := by nlinarith
      rw [if_neg hz, h0]
      simp

/-- **A scan coordinate moves by less than 1 mm, toward zero.** For a finite
coordinate that does not saturate, the model's millimetre value is within
1 mm of the value the shim handed over. -/
theorem coordMm_sub_lt (b : Nat) (hf : F64.finite b = true)
    (h : (coordMm b).natAbs < coordLimit) :
    |sval64 b * 1000 - ((coordMm b : ℤ) : ℝ)| < 1 := by
  rw [coordMm_eq_truncReal b hf] at h ⊢
  exact truncReal_sub_lt _ h

/-- A scan point is within 1 mm, on each axis, of the point the shim handed
over, unless it saturates. -/
theorem scanPointMm_sub_lt (c : Coords) (hx : F64.finite c.1 = true) (hy : F64.finite c.2 = true)
    (hsx : (scanPointMm c).x.natAbs < coordLimit) (hsy : (scanPointMm c).y.natAbs < coordLimit) :
    |sval64 c.1 * 1000 - ((scanPointMm c).x : ℝ)| < 1 ∧
      |sval64 c.2 * 1000 - ((scanPointMm c).y : ℝ)| < 1 :=
  ⟨coordMm_sub_lt c.1 hx hsx, coordMm_sub_lt c.2 hy hsy⟩

/-- A scan point never lies outside `±coordLimit`, whatever the patterns. -/
theorem scanPointMm_inRange (c : Coords) : (scanPointMm c).inRange coordLimit = true := by
  simp [Point.inRange, scanPointMm, coordMm_natAbs_le]

/-! ## Vertices of a topic zone -/

/-- Below 1024 m the magnitude in millimetres is the exact product rounded to
the nearest integer, a half up. -/
theorem vertexMagMm_eq (b : Nat) (h : F64.expField b < 1033) :
    vertexMagMm b =
      (2 * (F64.mant b * 2 ^ F64.shift b * 1000) + 2 ^ F64.bias) / (2 * 2 ^ F64.bias) := by
  unfold vertexMagMm
  rw [if_neg (by omega)]
  have hb : F64.bias = 1075 := rfl
  have hs : F64.shift b ≤ 1032 := by
    unfold F64.shift; split <;> omega
  obtain ⟨k, hk⟩ : ∃ k, F64.bias - F64.shift b - 1 = k := ⟨_, rfl⟩
  have h2 : (2 : ℕ) ^ F64.bias = 2 ^ F64.shift b * (2 * 2 ^ k) := by
    rw [← pow_succ', ← pow_add]; congr 1; omega
  have hK : 0 < 2 ^ k := by positivity
  rw [hk, Nat.shiftRight_eq_div_pow, h2,
    show 2 * (F64.mant b * 2 ^ F64.shift b * 1000) + 2 ^ F64.shift b * (2 * 2 ^ k) =
      (2 * 2 ^ F64.shift b) * (F64.mant b * 1000 + 2 ^ k) by ring,
    show 2 * (2 ^ F64.shift b * (2 * 2 ^ k)) = (2 * 2 ^ F64.shift b) * (2 ^ k * 2) by ring,
    Nat.mul_div_mul_left _ _ (by positivity), ← Nat.div_div_eq_div_mul, Nat.add_div_right _ hK]

theorem vertexCoordMm_natAbs (b : Nat) : (vertexCoordMm b).natAbs = vertexMagMm b := by
  unfold vertexCoordMm
  split <;> simp

/-- **A vertex coordinate that is not finite is never within the limit**
(ND6): if the millimetre value is below `coordLimit`, the pattern denotes a
finite number below 1024 m. -/
theorem vertexCoordMm_finite_of_lt (b : Nat) (h : (vertexCoordMm b).natAbs < coordLimit) :
    F64.expField b < 1033 ∧ F64.finite b = true := by
  rw [vertexCoordMm_natAbs] at h
  have he : F64.expField b < 1033 := by
    by_contra hc
    unfold vertexMagMm at h
    rw [if_pos (by omega)] at h
    exact Nat.lt_irrefl _ h
  refine ⟨he, ?_⟩
  unfold F64.finite
  simp only [bne_iff_ne, ne_eq]
  omega

/-- **A vertex moves by at most half a millimetre.** A vertex coordinate
within the coordinate limit is within 1/2 mm of the value the shim handed
over. -/
theorem vertexCoordMm_sub_le (b : Nat) (h : (vertexCoordMm b).natAbs < coordLimit) :
    |sval64 b * 1000 - ((vertexCoordMm b : ℤ) : ℝ)| ≤ 1 / 2 := by
  obtain ⟨he, -⟩ := vertexCoordMm_finite_of_lt b h
  have hr := round_bounds (F64.mant b * 2 ^ F64.shift b * 1000) (2 ^ F64.bias) (by positivity)
  rw [← vertexMagMm_eq b he, ← val64_mul_1000] at hr
  unfold sval64 vertexCoordMm
  cases F64.neg b
  · simp only [Bool.false_eq_true, ↓reduceIte, Int.cast_natCast]
    rw [abs_sub_comm]
    exact hr
  · simp only [↓reduceIte, Int.cast_neg, Int.cast_natCast]
    rw [show -val64 b * 1000 - -(vertexMagMm b : ℝ) = (vertexMagMm b : ℝ) - val64 b * 1000 by ring]
    exact hr

/-- A vertex that `placeVerts` accepts is within 1/2 mm, on each axis, of the
vertex the shim handed over, and both its coordinates are finite. -/
theorem vertexPointMm_sub_le (v : Coords)
    (h : (vertexPointMm v).inRange (coordLimit - 3) = true) :
    |sval64 v.1 * 1000 - ((vertexPointMm v).x : ℝ)| ≤ 1 / 2 ∧
      |sval64 v.2 * 1000 - ((vertexPointMm v).y : ℝ)| ≤ 1 / 2 := by
  have hc : coordLimit = 1000000 := rfl
  simp only [Point.inRange, Bool.and_eq_true, vertexPointMm] at h
  obtain ⟨h1, h2⟩ := h
  rw [hc] at h1 h2
  have h1 := of_decide_eq_true h1
  have h2 := of_decide_eq_true h2
  exact ⟨vertexCoordMm_sub_le v.1 (by omega), vertexCoordMm_sub_le v.2 (by omega)⟩

theorem vertexPointMm_finite (v : Coords)
    (h : (vertexPointMm v).inRange (coordLimit - 3) = true) :
    F64.finite v.1 = true ∧ F64.finite v.2 = true := by
  have hc : coordLimit = 1000000 := rfl
  simp only [Point.inRange, Bool.and_eq_true, vertexPointMm] at h
  obtain ⟨h1, h2⟩ := h
  rw [hc] at h1 h2
  have h1 := of_decide_eq_true h1
  have h2 := of_decide_eq_true h2
  exact ⟨(vertexCoordMm_finite_of_lt v.1 (by omega)).2,
    (vertexCoordMm_finite_of_lt v.2 (by omega)).2⟩

/-! ## The margin covers the rounding, with room to spare

`nearBoundary_of_rounded` (`Geometry.lean`) for vertices that are indexed by
what was received, and for any pair of bounds within the margin. -/

/-- **The margin covers any displacement within its budget.** Let `vs` be a
list of vertex descriptions, `ex v` a real position of each (millimetres) and
`rnd v` an integer point within `dv` of it on each axis. Let `q` be a point
of the boundary of the real polygon (on the edge `e`, at parameter `t`) and
`p'` an integer point within `dp` of `q` on each axis. If
`2·(dv + dp)² ≤ 5`, that is `dv + dp ≤ 1.5811…`, the model counts `p'` as
inside the polygon of the rounded vertices. -/
theorem contains_of_close {α : Type} (vs : List α) (ex : α → ℝ × ℝ) (rnd : α → Point)
    (dv dp : ℝ) (hm : 2 * (dv + dp) ^ 2 ≤ 5)
    (hr : ∀ v ∈ vs, |(ex v).1 - (rnd v).x| ≤ dv ∧ |(ex v).2 - (rnd v).y| ≤ dv)
    (e : α × α) (he : e ∈ cyclicPairs vs) (q : ℝ × ℝ) (t : ℝ) (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = (ex e.1).1 + t * ((ex e.2).1 - (ex e.1).1))
    (hqy : q.2 = (ex e.1).2 + t * ((ex e.2).2 - (ex e.1).2))
    (p' : Point) (hpx : |q.1 - p'.x| ≤ dp) (hpy : |q.2 - p'.y| ≤ dp) :
    (Shape.polygon (vs.map rnd)).contains p' = true := by
  apply contains_of_nearBoundary
  rw [nearBoundary_iff, edges_eq_cyclicPairs, cyclicPairs_map]
  have hmem := mem_of_mem_cyclicPairs he
  refine ⟨Prod.map rnd rnd e, List.mem_map_of_mem he, ?_⟩
  exact nearSegment_of_close p' (rnd e.1) (rnd e.2) (ex e.1) (ex e.2) q t dv dp ht0 ht1 hqx hqy
    (hr _ hmem.1).1 (hr _ hmem.1).2 (hr _ hmem.2).1 (hr _ hmem.2).2 hpx hpy hm

/-- **The margin covers the rounding (any indexing).** The case of SPEC.md:
vertices rounded to the nearest millimetre (1/2 per axis), points truncated
(1 per axis), `2·(1/2 + 1)² = 4.5 ≤ 5`. -/
theorem contains_of_rounded {α : Type} (vs : List α) (ex : α → ℝ × ℝ) (rnd : α → Point)
    (hr : ∀ v ∈ vs, |(ex v).1 - (rnd v).x| ≤ 1 / 2 ∧ |(ex v).2 - (rnd v).y| ≤ 1 / 2)
    (e : α × α) (he : e ∈ cyclicPairs vs) (q : ℝ × ℝ) (t : ℝ) (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = (ex e.1).1 + t * ((ex e.2).1 - (ex e.1).1))
    (hqy : q.2 = (ex e.1).2 + t * ((ex e.2).2 - (ex e.1).2))
    (p' : Point) (hpx : |q.1 - p'.x| ≤ 1) (hpy : |q.2 - p'.y| ≤ 1) :
    (Shape.polygon (vs.map rnd)).contains p' = true :=
  contains_of_close vs ex rnd (1 / 2) 1 (by norm_num) hr e he q t ht0 ht1 hqx hqy p' hpx hpy

/-! ## The margin for option A

The shim computes a point `s` in `double`; upstream computes a point `u` for
the same reading, with the same formula but possibly other contractions
(fused multiply-add). The model truncates `s`. The theorems below take the
distance between `u` and `s`, per axis, as a parameter `δ`, and say how large
it may be. -/

/-- **Option A, a zone whose shape arrived on a topic.** Let the shim hand
over the vertices `raw` of a zone (accepted: all within the coordinate limit)
and the coordinates `c` of a reading (finite, not saturated). Let `up v` be
upstream's position of each vertex and `q` upstream's position of the
reading, in millimetres, each within `δ` of the shim's on each axis. If `q`
lies on the boundary of upstream's polygon (on the edge `e`, at parameter
`t`) and `δ ≤ 1/25` mm, the model counts the reading's point as inside the
zone.

Budget: a vertex is rounded by at most 1/2 mm and differs from upstream's by
at most `δ`; a point is truncated by less than 1 mm and differs from
upstream's by at most `δ`; `2·(3/2 + 2δ)² ≤ 5` holds for `δ ≤ 0.0405…`. -/
theorem contains_of_shim (δ : ℝ) (hδ0 : 0 ≤ δ) (hδ : δ ≤ 1 / 25) (raw : List Coords)
    (hraw : ∀ v ∈ raw, (vertexPointMm v).inRange (coordLimit - 3) = true)
    (up : Coords → ℝ × ℝ)
    (hup : ∀ v ∈ raw, |(up v).1 - sval64 v.1 * 1000| ≤ δ ∧ |(up v).2 - sval64 v.2 * 1000| ≤ δ)
    (c : Coords) (hx : F64.finite c.1 = true) (hy : F64.finite c.2 = true)
    (hsx : (scanPointMm c).x.natAbs < coordLimit) (hsy : (scanPointMm c).y.natAbs < coordLimit)
    (e : Coords × Coords) (he : e ∈ cyclicPairs raw) (q : ℝ × ℝ) (t : ℝ)
    (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = (up e.1).1 + t * ((up e.2).1 - (up e.1).1))
    (hqy : q.2 = (up e.1).2 + t * ((up e.2).2 - (up e.1).2))
    (hcx : |q.1 - sval64 c.1 * 1000| ≤ δ) (hcy : |q.2 - sval64 c.2 * 1000| ≤ δ) :
    (Shape.polygon (raw.map vertexPointMm)).contains (scanPointMm c) = true := by
  obtain ⟨px, py⟩ := scanPointMm_sub_lt c hx hy hsx hsy
  refine contains_of_close raw up vertexPointMm (1 / 2 + δ) (1 + δ) (by nlinarith) ?_ e he q t
    ht0 ht1 hqx hqy _ ?_ ?_
  · intro v hv
    obtain ⟨h1, h2⟩ := vertexPointMm_sub_le v (hraw v hv)
    obtain ⟨u1, u2⟩ := hup v hv
    rw [abs_le] at h1 h2 u1 u2
    constructor <;> rw [abs_le] <;> constructor <;> linarith
  · rw [abs_lt] at px
    rw [abs_le] at hcx ⊢
    constructor <;> linarith
  · rw [abs_lt] at py
    rw [abs_le] at hcy ⊢
    constructor <;> linarith

/-- **Option A, a polygon fixed by the parameters.** As `contains_of_shim`,
for a polygon whose vertices `vs` (real numbers, millimetres) are the same
for upstream and for the model, which rounds them by at most 1/2 mm on each
axis. Only the point differs: `δ ≤ 2/25` mm suffices. -/
theorem contains_of_shim_fixed (δ : ℝ) (hδ0 : 0 ≤ δ) (hδ : δ ≤ 2 / 25) (vs : List (ℝ × ℝ))
    (rnd : ℝ × ℝ → Point)
    (hr : ∀ v ∈ vs, |v.1 - (rnd v).x| ≤ 1 / 2 ∧ |v.2 - (rnd v).y| ≤ 1 / 2)
    (c : Coords) (hx : F64.finite c.1 = true) (hy : F64.finite c.2 = true)
    (hsx : (scanPointMm c).x.natAbs < coordLimit) (hsy : (scanPointMm c).y.natAbs < coordLimit)
    (e : (ℝ × ℝ) × (ℝ × ℝ)) (he : e ∈ cyclicPairs vs) (q : ℝ × ℝ) (t : ℝ)
    (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = e.1.1 + t * (e.2.1 - e.1.1)) (hqy : q.2 = e.1.2 + t * (e.2.2 - e.1.2))
    (hcx : |q.1 - sval64 c.1 * 1000| ≤ δ) (hcy : |q.2 - sval64 c.2 * 1000| ≤ δ) :
    (Shape.polygon (vs.map rnd)).contains (scanPointMm c) = true := by
  obtain ⟨px, py⟩ := scanPointMm_sub_lt c hx hy hsx hsy
  refine contains_of_close vs id rnd (1 / 2) (1 + δ) (by nlinarith) hr e he q t
    ht0 ht1 hqx hqy _ ?_ ?_
  · rw [abs_lt] at px
    rw [abs_le] at hcx ⊢
    constructor <;> linarith
  · rw [abs_lt] at py
    rw [abs_le] at hcy ⊢
    constructor <;> linarith

/-- **Option A, a circle.** Take a circle zone whose `radius` parameter has
pattern `br` and was accepted as `r` millimetres, and a reading whose
coordinates the shim handed over as the finite patterns `c`. Let `(ux, uy)`
be upstream's position of the reading in millimetres, within `δ ≤ 1/2` mm of
the shim's on each axis. If upstream's point lies strictly inside the
configured circle, the model counts the reading's point as inside the zone:
the 1 mm added to every radius (D4) covers the difference. -/
theorem circle_safe_shim (δ : ℝ) (hδ : δ ≤ 1 / 2) (br r : Nat) (hr : radiusMm br = some r)
    (c : Coords) (hx : F64.finite c.1 = true) (hy : F64.finite c.2 = true) (ux uy : ℝ)
    (hux : |ux - sval64 c.1 * 1000| ≤ δ) (huy : |uy - sval64 c.2 * 1000| ≤ δ)
    (hin : ux ^ 2 + uy ^ 2 < (val64 br * 1000) ^ 2) :
    (Shape.circle r).contains (scanPointMm c) = true := by
  have h0 : 0 ≤ val64 br * 1000 := mul_nonneg (val64_nonneg br) (by norm_num)
  have hge := radiusMm_ge br r hr
  apply contains_circle_of_truncated r _ (val64 br * 1000 + 1) (sval64 c.1 * 1000)
    (sval64 c.2 * 1000)
  · linarith
  · exact hge
  · generalize val64 br * 1000 = R at h0 hin
    generalize sval64 c.1 * 1000 = sx at hux
    generalize sval64 c.2 * 1000 = sy at huy
    rw [abs_le] at hux huy
    have hxa : ux ≤ R ∧ -R ≤ ux := by constructor <;> nlinarith [sq_nonneg uy]
    have hya : uy ≤ R ∧ -R ≤ uy := by constructor <;> nlinarith [sq_nonneg ux]
    obtain ⟨dx, rfl⟩ : ∃ dx, sx = ux + dx := ⟨sx - ux, by ring⟩
    obtain ⟨dy, rfl⟩ : ∃ dy, sy = uy + dy := ⟨sy - uy, by ring⟩
    have hdx : -(1 / 2) ≤ dx ∧ dx ≤ 1 / 2 := by constructor <;> linarith
    have hdy : -(1 / 2) ≤ dy ∧ dy ≤ 1 / 2 := by constructor <;> linarith
    nlinarith [mul_nonneg (sub_nonneg.mpr hxa.1) (show (0 : ℝ) ≤ 1 / 2 + dx by linarith),
      mul_nonneg (show (0 : ℝ) ≤ R + ux by linarith) (show (0 : ℝ) ≤ 1 / 2 - dx by linarith),
      mul_nonneg (sub_nonneg.mpr hya.1) (show (0 : ℝ) ≤ 1 / 2 + dy by linarith),
      mul_nonneg (show (0 : ℝ) ≤ R + uy by linarith) (show (0 : ℝ) ≤ 1 / 2 - dy by linarith),
      mul_nonneg (show (0 : ℝ) ≤ 1 / 2 - dx by linarith) (show (0 : ℝ) ≤ 1 / 2 + dx by linarith),
      mul_nonneg (show (0 : ℝ) ≤ 1 / 2 - dy by linarith) (show (0 : ℝ) ≤ 1 / 2 + dy by linarith)]
  · have := coordMm_abs_le c.1 hx
    rw [abs_mul, abs_sval64, abs_of_nonneg (show (0 : ℝ) ≤ 1000 by norm_num)]
    exact this
  · have := coordMm_abs_le c.2 hy
    rw [abs_mul, abs_sval64, abs_of_nonneg (show (0 : ℝ) ≤ 1000 by norm_num)]
    exact this

end ROSlean.CollisionMonitor.Margin
