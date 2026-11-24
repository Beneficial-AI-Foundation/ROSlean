import Mathlib.Tactic
import ROSleanCore.CollisionMonitor
import ROSlean.CollisionMonitor.Convert

/-!
# Collision monitor: the transform is applied exactly

`ROSleanCore.CollisionMonitorXform` applies a looked-up transform to scan
points (`Xform.pointMm`) and to zone vertices (`Xform.vertexMm`) in integer
arithmetic. This file proves what those functions compute, against real
numbers (`docs/collision_monitor/SPEC_NAV2.md`, N25, section 4 "Transform",
section 5).

* `sval64 b`, `sval32 b`: the number a finite pattern denotes, with its sign.
* `imageX t x y`, `imageY t x y`: the x and y, in metres, of the image of the
  point `(x, y, 0)` under the transform `t`, by upstream's formula (N25) in
  exact real arithmetic: the first two rows of the rotation matrix of the
  quaternion, each entry divided by `|q|²`, then the translation. For a
  quaternion of unit length this is the rotation it denotes
  (`rotX_of_unit`, `rotY_of_unit`); for any other it is that of its
  normalisation (`rotX_smul`, `rotY_smul`).
* **Points** (`pointMm_x_eq`, `pointMm_y_eq`): for a usable transform and
  finite coordinates, each coordinate of `t.pointMm x y` is the exact image in
  millimetres, truncated toward zero and saturated at `±coordLimit`
  (`truncReal`). Hence it never lies further from the axis than the exact
  image, has its sign, and unless it is saturated is less than 1 mm short
  (`pointMm_x_abs_le`, `pointMm_x_abs_gt`, `pointMm_x_sub_lt`, and the same
  for `y`).
* **Vertices** (`vertexMm_x_sub_le`, `vertexMm_y_sub_le`): each coordinate of
  `t.vertexMm x y` is within half a millimetre of the exact image.
* **The identity** (`pointMm_identity`): under the identity transform a
  finite point is converted exactly as `coordMm` converts it, so a scan in
  the base frame is treated as before the extension.
* **The margin covers the rounding** (`contains_of_transformed`): the
  analogue of `nearBoundary_of_rounded` for a polygon whose vertices are the
  images of received vertices under one transform and a point that is the
  image of a scan reading under another.

What is **not** proved here, and is not provable in Lean without a model of
binary64 arithmetic: how far upstream's own floating-point evaluation of the
same formula is from the exact image (SPEC_NAV2, trusted item T7).
-/

namespace ROSlean.CollisionMonitor.Transform

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor ROSlean.CollisionMonitor.Convert

/-! ## Dyadic numbers -/

/-- The number a dyadic denotes. -/
noncomputable def dval (a : Dy) : ℝ := (a.m : ℝ) * (2 : ℝ) ^ a.e

theorem dval_mul (a b : Dy) : dval (a.mul b) = dval a * dval b := by
  simp only [dval, Dy.mul]
  push_cast
  rw [zpow_add₀ (by norm_num)]
  ring

theorem dval_neg (a : Dy) : dval a.neg = -dval a := by
  simp [dval, Dy.neg]

theorem dval_double (a : Dy) : dval a.double = 2 * dval a := by
  simp only [dval, Dy.double]
  rw [zpow_add₀ (by norm_num), zpow_one]
  ring

private theorem zpow_split (e e' : ℤ) (h : e ≤ e') :
    (2 : ℝ) ^ e' = 2 ^ ((e' - e).toNat) * 2 ^ e := by
  rw [← zpow_natCast, ← zpow_add₀ (by norm_num)]
  congr 1
  rw [Int.toNat_of_nonneg (by omega)]
  ring

theorem dval_add (a b : Dy) : dval (a.add b) = dval a + dval b := by
  unfold Dy.add
  split_ifs with h1 h2 h3
  · simp [dval, h1]
  · simp [dval, h2]
  · simp only [dval]
    push_cast
    rw [zpow_split a.e b.e h3]
    ring
  · simp only [dval]
    push_cast
    rw [zpow_split b.e a.e (by omega)]
    ring

theorem dval_sub (a b : Dy) : dval (a.sub b) = dval a - dval b := by
  rw [Dy.sub, dval_add, dval_neg]
  ring

/-! ## The number a pattern denotes, with its sign -/

/-- The number a finite binary64 pattern denotes. -/
noncomputable def sval64 (b : Nat) : ℝ := if F64.neg b = true then -val64 b else val64 b

/-- The number a finite binary32 pattern denotes. -/
noncomputable def sval32 (b : Nat) : ℝ :=
  if ROSleanCore.CollisionMonitorFFI.F32.neg b = true then -val32 b else val32 b

theorem abs_sval64 (b : Nat) : |sval64 b| = val64 b := by
  unfold sval64
  split
  · rw [abs_neg, abs_of_nonneg (val64_nonneg b)]
  · exact abs_of_nonneg (val64_nonneg b)

private theorem zpow_shift (s B : ℕ) : (2 : ℝ) ^ ((s : ℤ) - (B : ℤ)) = (2 : ℝ) ^ s / (2 : ℝ) ^ B := by
  rw [zpow_sub₀ (by norm_num), zpow_natCast, zpow_natCast]

theorem dval_ofF64 (b : Nat) : dval (Dy.ofF64 b) = sval64 b := by
  have key : (F64.mant b : ℝ) * (2 : ℝ) ^ ((F64.shift b : ℤ) - ((1075 : ℕ) : ℤ)) = val64 b := by
    unfold val64 F64.bias
    rw [zpow_shift]
    simp only [Nat.cast_mul, Nat.cast_pow, Nat.cast_ofNat]
    rw [mul_div_assoc]
  unfold dval Dy.ofF64 sval64
  split
  · simp only [Int.cast_neg, Int.cast_natCast, neg_mul]
    exact congrArg _ key
  · simp only [Int.cast_natCast]
    exact key

theorem dval_ofF32 (b : Nat) : dval (Dy.ofF32 b) = sval32 b := by
  have key : (F32.mant b : ℝ) * (2 : ℝ) ^ ((F32.shift b : ℤ) - ((150 : ℕ) : ℤ)) = val32 b := by
    unfold val32 F32.bias
    rw [zpow_shift]
    simp only [Nat.cast_mul, Nat.cast_pow, Nat.cast_ofNat]
    rw [mul_div_assoc]
  unfold dval Dy.ofF32 sval32
  split
  · simp only [Int.cast_neg, Int.cast_natCast, neg_mul]
    exact congrArg _ key
  · simp only [Int.cast_natCast]
    exact key

/-! ## The exact image (N25) -/

/-- The x of the image of `(x, y, 0)` under the quaternion `(qx, qy, qz, qw)`
and the translation `ox`, as upstream's formula computes it in exact
arithmetic: `m00·x + m01·y + ox` with `m00 = 1 − 2(qy² + qz²)/d`,
`m01 = 2(qx·qy − qw·qz)/d`, `d = qx² + qy² + qz² + qw²`. -/
noncomputable def rotX (qx qy qz qw ox x y : ℝ) : ℝ :=
  ((qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 - 2 * (qy ^ 2 + qz ^ 2)) * x +
    2 * (qx * qy - qw * qz) * y) / (qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2) + ox

/-- The y of the image: `m10·x + m11·y + oy` with `m10 = 2(qx·qy + qw·qz)/d`,
`m11 = 1 − 2(qx² + qz²)/d`. -/
noncomputable def rotY (qx qy qz qw oy x y : ℝ) : ℝ :=
  (2 * (qx * qy + qw * qz) * x +
    (qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 - 2 * (qx ^ 2 + qz ^ 2)) * y) /
      (qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2) + oy

/-- For a quaternion of unit length this is the first row of the rotation
matrix it denotes, applied to `(x, y, 0)`, plus the translation. -/
theorem rotX_of_unit (qx qy qz qw ox x y : ℝ) (h : qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 = 1) :
    rotX qx qy qz qw ox x y =
      (1 - 2 * (qy ^ 2 + qz ^ 2)) * x + 2 * (qx * qy - qw * qz) * y + ox := by
  unfold rotX
  rw [h, div_one]

/-- The second row. -/
theorem rotY_of_unit (qx qy qz qw oy x y : ℝ) (h : qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 = 1) :
    rotY qx qy qz qw oy x y =
      2 * (qx * qy + qw * qz) * x + (1 - 2 * (qx ^ 2 + qz ^ 2)) * y + oy := by
  unfold rotY
  rw [h, div_one]

/-- A quaternion that is not of unit length acts as any non-zero multiple of
it does, in particular as its normalisation (`c = 1 / |q|`). -/
theorem rotX_smul (c qx qy qz qw ox x y : ℝ) (hc : c ≠ 0)
    (hq : qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 ≠ 0) :
    rotX (c * qx) (c * qy) (c * qz) (c * qw) ox x y = rotX qx qy qz qw ox x y := by
  unfold rotX
  have h1 : (c * qx) ^ 2 + (c * qy) ^ 2 + (c * qz) ^ 2 + (c * qw) ^ 2 =
      c ^ 2 * (qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2) := by ring
  rw [h1]
  field_simp

theorem rotY_smul (c qx qy qz qw oy x y : ℝ) (hc : c ≠ 0)
    (hq : qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2 ≠ 0) :
    rotY (c * qx) (c * qy) (c * qz) (c * qw) oy x y = rotY qx qy qz qw oy x y := by
  unfold rotY
  have h1 : (c * qx) ^ 2 + (c * qy) ^ 2 + (c * qz) ^ 2 + (c * qw) ^ 2 =
      c ^ 2 * (qx ^ 2 + qy ^ 2 + qz ^ 2 + qw ^ 2) := by ring
  rw [h1]
  field_simp

/-- The squared length of the quaternion of a transform. -/
noncomputable def normSq (t : Xform) : ℝ :=
  sval64 t.qx ^ 2 + sval64 t.qy ^ 2 + sval64 t.qz ^ 2 + sval64 t.qw ^ 2

/-- The x, in metres, of the exact image of `(x, y, 0)` under `t`. -/
noncomputable def imageX (t : Xform) (x y : ℝ) : ℝ :=
  rotX (sval64 t.qx) (sval64 t.qy) (sval64 t.qz) (sval64 t.qw) (sval64 t.ox) x y

/-- The y, in metres, of the exact image of `(x, y, 0)` under `t`. -/
noncomputable def imageY (t : Xform) (x y : ℝ) : ℝ :=
  rotY (sval64 t.qx) (sval64 t.qy) (sval64 t.qz) (sval64 t.qw) (sval64 t.oy) x y

theorem dval_den (t : Xform) : dval t.den = normSq t := by
  simp only [Xform.den, dval_add, dval_mul, dval_ofF64, normSq]
  ring

/-- A usable transform has finite numbers and a quaternion that is not
zero. -/
theorem usable_iff (t : Xform) : t.usable = true ↔ t.finite = true ∧ 0 < normSq t := by
  unfold Xform.usable
  rw [Bool.and_eq_true, decide_eq_true_iff, ← dval_den]
  refine and_congr_right fun _ => ?_
  unfold dval
  have hp : (0 : ℝ) < (2 : ℝ) ^ t.den.e := by positivity
  constructor
  · intro h
    exact mul_pos (by exact_mod_cast h) hp
  · intro h
    have := (pos_iff_pos_of_mul_pos h).mpr hp
    exact_mod_cast this

theorem normSq_pos (t : Xform) (h : t.usable = true) : 0 < normSq t := ((usable_iff t).mp h).2

theorem dval_numX (t : Xform) (x y : Dy) :
    dval (t.mat.numX x y) = imageX t (dval x) (dval y) * normSq t ∨ normSq t = 0 := by
  by_cases h : normSq t = 0
  · exact Or.inr h
  · left
    simp only [Mat.numX, Xform.mat, dval_add, dval_sub, dval_mul, dval_double, dval_ofF64,
      dval_den, imageX, rotX]
    unfold normSq at h ⊢
    field_simp

theorem dval_numY (t : Xform) (x y : Dy) :
    dval (t.mat.numY x y) = imageY t (dval x) (dval y) * normSq t ∨ normSq t = 0 := by
  by_cases h : normSq t = 0
  · exact Or.inr h
  · left
    simp only [Mat.numY, Xform.mat, dval_add, dval_sub, dval_mul, dval_double, dval_ofF64,
      dval_den, imageY, rotY]
    unfold normSq at h ⊢
    field_simp

/-! ## The quotient in millimetres -/

/-- `Dy.scaled` is the quotient times 1000, with a positive denominator. -/
theorem scaled_spec (num den : Dy) (hd : 0 < den.m) :
    0 < (num.scaled den).2 ∧
      ((num.scaled den).1 : ℝ) / ((num.scaled den).2 : ℝ) = dval num / dval den * 1000 := by
  have hdm : ((den.m.toNat : ℕ) : ℝ) = (den.m : ℝ) := by
    have : ((den.m.toNat : ℕ) : ℤ) = den.m := Int.toNat_of_nonneg (by omega)
    exact_mod_cast this
  have hdpos : (0 : ℝ) < (den.m : ℝ) := by exact_mod_cast hd
  have hn : 0 < den.m.toNat := by omega
  unfold Dy.scaled
  split
  · rename_i h0
    refine ⟨hn, ?_⟩
    simp [dval, h0]
  split
  · rename_i h
    refine ⟨hn, ?_⟩
    simp only [dval]
    push_cast
    rw [hdm, zpow_split den.e num.e h]
    have h2 : (0 : ℝ) < (2 : ℝ) ^ den.e := by positivity
    field_simp
  · rename_i h
    refine ⟨Nat.mul_pos hn (by positivity), ?_⟩
    simp only [dval]
    push_cast
    rw [hdm, zpow_split num.e den.e (by omega)]
    have h2 : (0 : ℝ) < (2 : ℝ) ^ num.e := by positivity
    have h3 : (0 : ℝ) < (2 : ℝ) ^ (den.e - num.e).toNat := by positivity
    field_simp

/-- A real number truncated toward zero to an integer and saturated at
`±coordLimit`. -/
noncomputable def truncReal (r : ℝ) : ℤ :=
  if r < 0 then -((min coordLimit ⌊|r|⌋₊ : ℕ) : ℤ) else ((min coordLimit ⌊|r|⌋₊ : ℕ) : ℤ)

theorem truncMm_eq (q : ℤ × ℕ) (hq : 0 < q.2) :
    truncMm q = truncReal ((q.1 : ℝ) / (q.2 : ℝ)) := by
  have hq' : (0 : ℝ) < (q.2 : ℝ) := by exact_mod_cast hq
  have hneg : ((q.1 : ℝ) / (q.2 : ℝ) < 0) ↔ q.1 < 0 := by
    rw [div_neg_iff]
    constructor
    · rintro (⟨_, h⟩ | ⟨h, _⟩)
      · linarith
      · exact_mod_cast h
    · intro h
      exact Or.inr ⟨by exact_mod_cast h, hq'⟩
  have habs : |(q.1 : ℝ) / (q.2 : ℝ)| = ((q.1.natAbs : ℕ) : ℝ) / (q.2 : ℝ) := by
    rw [abs_div, abs_cast_eq_natAbs, abs_of_pos hq']
  have hfloor : ⌊|(q.1 : ℝ) / (q.2 : ℝ)|⌋₊ = q.1.natAbs / q.2 := by
    rw [habs, Nat.floor_eq_iff (by positivity)]
    exact floor_bounds _ _ hq
  unfold truncMm truncReal
  simp only [hneg, hfloor]

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

/-- Rounding to nearest moves a quotient by at most 1/2. -/
theorem roundMm_sub_le (q : ℤ × ℕ) (hq : 0 < q.2) :
    |((roundMm q : ℤ) : ℝ) - (q.1 : ℝ) / (q.2 : ℝ)| ≤ 1 / 2 := by
  obtain ⟨n, hn⟩ : ∃ n : ℕ, q.1.natAbs = n := ⟨_, rfl⟩
  have hb := round_bounds n q.2 hq
  unfold roundMm
  rw [hn]
  split
  · rename_i hneg
    have hq1 : (q.1 : ℝ) = -(n : ℝ) := by
      have : (q.1 : ℤ) = -(n : ℤ) := by omega
      exact_mod_cast this
    have e : (((-(((2 * n + q.2) / (2 * q.2) : ℕ) : ℤ) : ℤ)) : ℝ) - (q.1 : ℝ) / (q.2 : ℝ) =
        -((((2 * n + q.2) / (2 * q.2) : ℕ) : ℝ) - (n : ℝ) / (q.2 : ℝ)) := by
      rw [hq1]
      simp only [Int.cast_neg, Int.cast_natCast]
      ring
    rw [e, abs_neg]
    exact hb
  · rename_i hneg
    have hq1 : (q.1 : ℝ) = (n : ℝ) := by
      have : (q.1 : ℤ) = (n : ℤ) := by omega
      exact_mod_cast this
    rw [hq1]
    exact_mod_cast hb

/-! ## Scan points -/

theorem usable_den_pos (t : Xform) (h : t.usable = true) : 0 < t.mat.d.m := by
  unfold Xform.usable at h
  rw [Bool.and_eq_true, decide_eq_true_iff] at h
  exact h.2

/-- **A scan point is the exact image, truncated (x).** For a usable
transform and finite coordinates, the x of `t.pointMm x y` is the x of the
exact image of the scan-frame point, in millimetres, truncated toward zero
and saturated at `±coordLimit`. -/
theorem pointMm_x_eq (t : Xform) (x y : Nat) (ht : t.usable = true) (hx : F64.finite x = true)
    (hy : F64.finite y = true) :
    (t.pointMm x y).x = truncReal (imageX t (sval64 x) (sval64 y) * 1000) := by
  have hd := usable_den_pos t ht
  have hn := normSq_pos t ht
  obtain ⟨hpos, hval⟩ := scaled_spec (t.mat.numX (Dy.ofF64 x) (Dy.ofF64 y)) t.mat.d hd
  simp only [Xform.pointMm, ht, ↓reduceIte, Mat.pointMm, hx, hy, Bool.and_self]
  rw [truncMm_eq _ hpos, hval]
  have hdd : dval t.mat.d = normSq t := dval_den t
  rcases dval_numX t (Dy.ofF64 x) (Dy.ofF64 y) with h | h
  · rw [h, hdd, dval_ofF64, dval_ofF64, mul_div_assoc, div_self (ne_of_gt hn), mul_one]
  · exact absurd h (ne_of_gt hn)

/-- **A scan point is the exact image, truncated (y).** -/
theorem pointMm_y_eq (t : Xform) (x y : Nat) (ht : t.usable = true) (hx : F64.finite x = true)
    (hy : F64.finite y = true) :
    (t.pointMm x y).y = truncReal (imageY t (sval64 x) (sval64 y) * 1000) := by
  have hd := usable_den_pos t ht
  have hn := normSq_pos t ht
  obtain ⟨hpos, hval⟩ := scaled_spec (t.mat.numY (Dy.ofF64 x) (Dy.ofF64 y)) t.mat.d hd
  simp only [Xform.pointMm, ht, ↓reduceIte, Mat.pointMm, hx, hy, Bool.and_self]
  rw [truncMm_eq _ hpos, hval]
  have hdd : dval t.mat.d = normSq t := dval_den t
  rcases dval_numY t (Dy.ofF64 x) (Dy.ofF64 y) with h | h
  · rw [h, hdd, dval_ofF64, dval_ofF64, mul_div_assoc, div_self (ne_of_gt hn), mul_one]
  · exact absurd h (ne_of_gt hn)

/-- **A point never moves away from the origin (x).** -/
theorem pointMm_x_abs_le (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true) :
    |(((t.pointMm x y).x : ℤ) : ℝ)| ≤ |imageX t (sval64 x) (sval64 y) * 1000| := by
  rw [pointMm_x_eq t x y ht hx hy]
  exact abs_truncReal_le _

theorem pointMm_y_abs_le (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true) :
    |(((t.pointMm x y).y : ℤ) : ℝ)| ≤ |imageY t (sval64 x) (sval64 y) * 1000| := by
  rw [pointMm_y_eq t x y ht hx hy]
  exact abs_truncReal_le _

/-- **Unless it saturates, a point is less than 1 mm short (x).** -/
theorem pointMm_x_abs_gt (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true)
    (hlt : (t.pointMm x y).x.natAbs < coordLimit) :
    |imageX t (sval64 x) (sval64 y) * 1000| < |(((t.pointMm x y).x : ℤ) : ℝ)| + 1 := by
  rw [pointMm_x_eq t x y ht hx hy] at hlt ⊢
  exact abs_truncReal_gt _ hlt

theorem pointMm_y_abs_gt (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true)
    (hlt : (t.pointMm x y).y.natAbs < coordLimit) :
    |imageY t (sval64 x) (sval64 y) * 1000| < |(((t.pointMm x y).y : ℤ) : ℝ)| + 1 := by
  rw [pointMm_y_eq t x y ht hx hy] at hlt ⊢
  exact abs_truncReal_gt _ hlt

/-- Unless it saturates, a point is within 1 mm of the exact image on each
axis (x). -/
theorem pointMm_x_sub_lt (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true)
    (hlt : (t.pointMm x y).x.natAbs < coordLimit) :
    |imageX t (sval64 x) (sval64 y) * 1000 - (((t.pointMm x y).x : ℤ) : ℝ)| < 1 := by
  rw [pointMm_x_eq t x y ht hx hy] at hlt ⊢
  exact truncReal_sub_lt _ hlt

theorem pointMm_y_sub_lt (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true)
    (hlt : (t.pointMm x y).y.natAbs < coordLimit) :
    |imageY t (sval64 x) (sval64 y) * 1000 - (((t.pointMm x y).y : ℤ) : ℝ)| < 1 := by
  rw [pointMm_y_eq t x y ht hx hy] at hlt ⊢
  exact truncReal_sub_lt _ hlt

/-- A point's coordinates are within `±coordLimit`, whatever the transform
and the patterns. -/
theorem pointMm_inRange (t : Xform) (x y : Nat) : (t.pointMm x y).inRange coordLimit = true := by
  have htr : ∀ q : ℤ × ℕ, (truncMm q).natAbs ≤ coordLimit := by
    intro q
    unfold truncMm
    split
    · rw [Int.natAbs_neg, Int.natAbs_natCast]; exact Nat.min_le_left _ _
    · rw [Int.natAbs_natCast]; exact Nat.min_le_left _ _
  unfold Xform.pointMm Mat.pointMm
  split
  · split
    · simp only [Point.inRange, Bool.and_eq_true, decide_eq_true_eq]
      exact ⟨htr _, htr _⟩
    · simp [Point.far, Point.inRange]
  · simp [Point.far, Point.inRange]

/-- A point that is not computed (the transform is not usable, or a
coordinate is not finite) is `Point.far`. -/
theorem pointMm_of_not (t : Xform) (x y : Nat)
    (h : t.usable = false ∨ F64.finite x = false ∨ F64.finite y = false) :
    t.pointMm x y = Point.far := by
  unfold Xform.pointMm Mat.pointMm
  rcases h with h | h | h <;> simp [h]

/-! ## Zone vertices -/

/-- **A vertex is within half a millimetre of its exact image (x).** -/
theorem vertexMm_x_sub_le (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F32.finite x = true) (hy : F32.finite y = true) :
    |(((t.vertexMm x y).x : ℤ) : ℝ) - imageX t (sval32 x) (sval32 y) * 1000| ≤ 1 / 2 := by
  have hd := usable_den_pos t ht
  have hn := normSq_pos t ht
  obtain ⟨hpos, hval⟩ := scaled_spec (t.mat.numX (Dy.ofF32 x) (Dy.ofF32 y)) t.mat.d hd
  have hr := roundMm_sub_le _ hpos
  rw [hval] at hr
  have hdd : dval t.mat.d = normSq t := dval_den t
  simp only [Xform.vertexMm, ht, ↓reduceIte, Mat.vertexMm, hx, hy, Bool.and_self]
  rcases dval_numX t (Dy.ofF32 x) (Dy.ofF32 y) with h | h
  · rw [h, hdd, dval_ofF32, dval_ofF32, mul_div_assoc, div_self (ne_of_gt hn), mul_one] at hr
    exact hr
  · exact absurd h (ne_of_gt hn)

/-- **A vertex is within half a millimetre of its exact image (y).** -/
theorem vertexMm_y_sub_le (t : Xform) (x y : Nat) (ht : t.usable = true)
    (hx : F32.finite x = true) (hy : F32.finite y = true) :
    |(((t.vertexMm x y).y : ℤ) : ℝ) - imageY t (sval32 x) (sval32 y) * 1000| ≤ 1 / 2 := by
  have hd := usable_den_pos t ht
  have hn := normSq_pos t ht
  obtain ⟨hpos, hval⟩ := scaled_spec (t.mat.numY (Dy.ofF32 x) (Dy.ofF32 y)) t.mat.d hd
  have hr := roundMm_sub_le _ hpos
  rw [hval] at hr
  have hdd : dval t.mat.d = normSq t := dval_den t
  simp only [Xform.vertexMm, ht, ↓reduceIte, Mat.vertexMm, hx, hy, Bool.and_self]
  rcases dval_numY t (Dy.ofF32 x) (Dy.ofF32 y) with h | h
  · rw [h, hdd, dval_ofF32, dval_ofF32, mul_div_assoc, div_self (ne_of_gt hn), mul_one] at hr
    exact hr
  · exact absurd h (ne_of_gt hn)

/-! ## The identity -/

theorem sval64_zero : sval64 0 = 0 := by
  have h1 : F64.mant 0 = 0 := by decide
  unfold sval64 val64
  rw [h1, Nat.zero_mul, Nat.cast_zero, zero_div, neg_zero, ite_self]

theorem sval64_one : sval64 0x3FF0000000000000 = 1 := by
  have h1 : F64.mant 0x3FF0000000000000 = 2 ^ 52 := by decide
  have h2 : F64.shift 0x3FF0000000000000 = 1023 := by decide
  have h3 : F64.neg 0x3FF0000000000000 = false := by decide
  have hprod : F64.mant 0x3FF0000000000000 * 2 ^ F64.shift 0x3FF0000000000000 = 2 ^ F64.bias := by
    rw [h1, h2, ← pow_add]
    exact congrArg (fun k => 2 ^ k) (by decide : 52 + 1023 = 1075)
  unfold sval64 val64
  rw [h3, hprod]
  simp only [Bool.false_eq_true, ↓reduceIte]
  exact div_self (by positivity)

theorem identity_usable : Xform.identity.usable = true := by decide

/-- The identity transform leaves every point where it is. -/
theorem image_identity (x y : ℝ) :
    imageX Xform.identity x y = x ∧ imageY Xform.identity x y = y := by
  simp only [imageX, imageY, rotX, rotY, Xform.identity, sval64_zero, sval64_one]
  constructor <;> ring

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

/-- **Under the identity a scan point is converted as before.** For finite
coordinates, the point of `Xform.identity` is `coordMm` of each coordinate:
a scan in the base frame, without base shift correction, gives exactly the
points the node of SPEC.md gave. (For a coordinate that is not finite both
give a point outside every accepted zone, but not the same one:
`Point.far` here, `±coordLimit` on that axis there.) -/
theorem pointMm_identity (x y : Nat) (hx : F64.finite x = true) (hy : F64.finite y = true) :
    Xform.identity.pointMm x y = ⟨coordMm x, coordMm y⟩ := by
  have h1 := pointMm_x_eq Xform.identity x y identity_usable hx hy
  have h2 := pointMm_y_eq Xform.identity x y identity_usable hx hy
  rw [(image_identity _ _).1] at h1
  rw [(image_identity _ _).2] at h2
  rw [← coordMm_eq_truncReal x hx] at h1
  rw [← coordMm_eq_truncReal y hy] at h2
  cases hp : Xform.identity.pointMm x y with
  | mk px py =>
    rw [hp] at h1 h2
    simp only at h1 h2
    rw [h1, h2]

/-! ## The margin covers the rounding, with transforms

`nearBoundary_of_rounded` (`Geometry.lean`) for vertices and points that are
indexed by what was received, not by their real positions. -/

/-- **The margin covers the rounding (any indexing).** Let `vs` be a list of
vertex descriptions, `ex v` the real position of each (millimetres) and
`rnd v` an integer point within 1/2 of it on each axis. Let `q` be a point of
the boundary of the real polygon (on the edge `e`, at parameter `t`) and `p'`
an integer point within 1 of `q` on each axis. Then the model counts `p'` as
inside the polygon of the rounded vertices. -/
theorem contains_of_rounded {α : Type} (vs : List α) (ex : α → ℝ × ℝ) (rnd : α → Point)
    (hr : ∀ v ∈ vs, |(ex v).1 - (rnd v).x| ≤ 1 / 2 ∧ |(ex v).2 - (rnd v).y| ≤ 1 / 2)
    (e : α × α) (he : e ∈ cyclicPairs vs) (q : ℝ × ℝ) (t : ℝ) (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = (ex e.1).1 + t * ((ex e.2).1 - (ex e.1).1))
    (hqy : q.2 = (ex e.1).2 + t * ((ex e.2).2 - (ex e.1).2))
    (p' : Point) (hpx : |q.1 - p'.x| ≤ 1) (hpy : |q.2 - p'.y| ≤ 1) :
    (Shape.polygon (vs.map rnd)).contains p' = true := by
  apply contains_of_nearBoundary
  rw [nearBoundary_iff, edges_eq_cyclicPairs, cyclicPairs_map]
  have hmem := mem_of_mem_cyclicPairs he
  refine ⟨Prod.map rnd rnd e, List.mem_map_of_mem he, ?_⟩
  exact nearSegment_of_close p' (rnd e.1) (rnd e.2) (ex e.1) (ex e.2) q t (1 / 2) 1 ht0 ht1 hqx hqy
    (hr _ hmem.1).1 (hr _ hmem.1).2 (hr _ hmem.2).1 (hr _ hmem.2).2 hpx hpy (by norm_num)

/-- The exact image, in millimetres, of a received vertex (binary32
patterns, metres) under `t`. -/
noncomputable def vertexImage (t : Xform) (v : Nat × Nat) : ℝ × ℝ :=
  (imageX t (sval32 v.1) (sval32 v.2) * 1000, imageY t (sval32 v.1) (sval32 v.2) * 1000)

/-- The exact image, in millimetres, of a scan-frame point (binary64
patterns, metres) under `t`. -/
noncomputable def pointImage (t : Xform) (x y : Nat) : ℝ × ℝ :=
  (imageX t (sval64 x) (sval64 y) * 1000, imageY t (sval64 x) (sval64 y) * 1000)

/-- **The margin covers the rounding, for transformed vertices and a
transformed point.** Let a polygon be received as the vertices `raw` (finite
binary32 patterns) and placed by the usable transform `tz`; let a scan
reading have the finite scan-frame coordinates `x`, `y` and be placed by the
usable transform `ts`, without saturating. If a point `q` of the boundary of
the exact image of the polygon is within 1 mm of the model's point on each
axis (in particular: if the exact image of the reading lies on that
boundary), the model counts the reading's point as inside the zone whose
vertices are `raw` under `tz`.

This is the quantitative half of the safe-direction argument for a topic
zone and for a scan in another frame (SPEC_NAV2, section 5): 1/2 mm per axis
for a vertex, less than 1 mm per axis for a point, both measured from the
**exact** images. The other half is prose, as for fixed zones: that the
boundary passes over the point (THEOREMS.md), and that upstream's
floating-point images are within 10⁻⁶ mm of the exact ones (T7). -/
theorem contains_of_transformed (tz : Xform) (raw : List (Nat × Nat)) (htz : tz.usable = true)
    (hraw : ∀ v ∈ raw, F32.finite v.1 = true ∧ F32.finite v.2 = true)
    (e : (Nat × Nat) × (Nat × Nat)) (he : e ∈ cyclicPairs raw) (q : ℝ × ℝ) (t : ℝ)
    (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = (vertexImage tz e.1).1 + t * ((vertexImage tz e.2).1 - (vertexImage tz e.1).1))
    (hqy : q.2 = (vertexImage tz e.1).2 + t * ((vertexImage tz e.2).2 - (vertexImage tz e.1).2))
    (p' : Point) (hpx : |q.1 - p'.x| ≤ 1) (hpy : |q.2 - p'.y| ≤ 1) :
    (Shape.polygon (raw.map (fun v => tz.vertexMm v.1 v.2))).contains p' = true := by
  refine contains_of_rounded raw (vertexImage tz) (fun v => tz.vertexMm v.1 v.2) ?_ e he q t
    ht0 ht1 hqx hqy p' hpx hpy
  intro v hv
  obtain ⟨h1, h2⟩ := hraw v hv
  constructor
  · rw [abs_sub_comm]; exact vertexMm_x_sub_le tz v.1 v.2 htz h1 h2
  · rw [abs_sub_comm]; exact vertexMm_y_sub_le tz v.1 v.2 htz h1 h2

/-- **A reading whose exact image lies on the boundary of the exact image of
the zone is counted.** The instance of `contains_of_transformed` with `q` the
exact image of the reading. -/
theorem contains_of_image_on_boundary (tz ts : Xform) (raw : List (Nat × Nat)) (x y : Nat)
    (htz : tz.usable = true) (hts : ts.usable = true)
    (hraw : ∀ v ∈ raw, F32.finite v.1 = true ∧ F32.finite v.2 = true)
    (hx : F64.finite x = true) (hy : F64.finite y = true)
    (hsx : (ts.pointMm x y).x.natAbs < coordLimit) (hsy : (ts.pointMm x y).y.natAbs < coordLimit)
    (e : (Nat × Nat) × (Nat × Nat)) (he : e ∈ cyclicPairs raw) (t : ℝ) (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : (pointImage ts x y).1 =
      (vertexImage tz e.1).1 + t * ((vertexImage tz e.2).1 - (vertexImage tz e.1).1))
    (hqy : (pointImage ts x y).2 =
      (vertexImage tz e.1).2 + t * ((vertexImage tz e.2).2 - (vertexImage tz e.1).2)) :
    (Shape.polygon (raw.map (fun v => tz.vertexMm v.1 v.2))).contains (ts.pointMm x y) = true :=
  contains_of_transformed tz raw htz hraw e he (pointImage ts x y) t ht0 ht1 hqx hqy _
    (le_of_lt (pointMm_x_sub_lt ts x y hts hx hy hsx))
    (le_of_lt (pointMm_y_sub_lt ts x y hts hx hy hsy))

end ROSlean.CollisionMonitor.Transform
