import Mathlib.Tactic
import ROSleanCore.CollisionMonitorConvert
import ROSlean.CollisionMonitor.Geometry
import ROSlean.CollisionMonitor.Bits

/-!
# Collision monitor: the conversions at the boundary

`ROSleanCore.CollisionMonitorConvert` turns the bit patterns the shim hands
over into the model's integers. This file proves what those conversions
guarantee, against the number a pattern denotes:

* `val64 b` and `val32 b` are the magnitudes of a finite binary64 and
  binary32 pattern, as real numbers. They agree with `magVal` of
  `ROSlean.CollisionMonitor.Bits` (`mant_shift_eq_magVal`).
* **Scan coordinates** (`coordMm`): the result is at most the true magnitude
  in millimetres and less than 1 mm below it, has the sign of the value, and
  never exceeds `coordLimit` (`coordMm_abs_le`, `coordMm_abs_gt`,
  `coordMm_natAbs_le`). So a point never moves away from the origin.
* **Radius** (`radiusMm`): the result is at least the true magnitude in
  millimetres plus 1 and less than that plus 2; the sign of the parameter does
  not matter; NaN, infinities and anything that would exceed `coordLimit` are
  refused (`radiusMm_ge`, `radiusMm_lt`, `radiusMm_abs`, `radiusMm_le_limit`,
  `radiusMm_of_not_finite`).
* **Circles are safe** (`circle_safe`): a point strictly inside the configured
  circle, in exact real arithmetic on the values the shim passed, is inside
  the model's circle.
* **Vertices** (`vertexMm`): within half a millimetre of the true value;
  non-finite values are refused (`vertexMm_abs_sub_le`).
* **Timeouts** (`timeoutNs`): negative, non-finite and huge values are
  refused, so that none of them can become 0, which would mean "check
  disabled" (`timeoutNs_of_neg`, `timeoutNs_of_not_finite`,
  `timeoutNs_of_huge`). That the accepted values are the nanoseconds upstream
  computes is tested against `rclcpp` (`smoke_timeouts.cpp`), not proved.
* **`min_points`** (`minPointsOfRaw_eq_some_iff`).
-/

namespace ROSlean.CollisionMonitor.Convert

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI

/-! ## Exact rounding of a quotient

The conversions are quotients of natural numbers by a power of two. These are
the three ways of rounding them, stated for any positive divisor. -/

theorem floor_bounds (n d : ℕ) (hd : 0 < d) :
    ((n / d : ℕ) : ℝ) ≤ (n : ℝ) / d ∧ (n : ℝ) / d < ((n / d : ℕ) : ℝ) + 1 := by
  have hd' : (0 : ℝ) < d := by exact_mod_cast hd
  have h1 : n / d * d ≤ n := Nat.div_mul_le_self n d
  have h2 : n < n / d * d + d := Nat.lt_div_mul_add hd
  have h1' : ((n / d : ℕ) : ℝ) * d ≤ n := by exact_mod_cast h1
  have h2' : (n : ℝ) < ((n / d : ℕ) : ℝ) * d + d := by exact_mod_cast h2
  constructor
  · rw [le_div_iff₀ hd']; exact h1'
  · rw [div_lt_iff₀ hd']; linarith

theorem ceil_bounds (n d : ℕ) (hd : 0 < d) :
    (n : ℝ) / d ≤ (((n + (d - 1)) / d : ℕ) : ℝ) ∧ (((n + (d - 1)) / d : ℕ) : ℝ) < (n : ℝ) / d + 1 := by
  have hd' : (0 : ℝ) < d := by exact_mod_cast hd
  have h1 : (n + (d - 1)) / d * d ≤ n + (d - 1) := Nat.div_mul_le_self _ d
  have h2 : n + (d - 1) < (n + (d - 1)) / d * d + d := Nat.lt_div_mul_add hd
  have h1n : (n + (d - 1)) / d * d < n + d := by omega
  have h2n : n ≤ (n + (d - 1)) / d * d := by omega
  have h1' : (((n + (d - 1)) / d : ℕ) : ℝ) * d < n + d := by exact_mod_cast h1n
  have h2' : (n : ℝ) ≤ (((n + (d - 1)) / d : ℕ) : ℝ) * d := by exact_mod_cast h2n
  constructor
  · rw [div_le_iff₀ hd']; exact h2'
  · rw [← sub_lt_iff_lt_add, lt_div_iff₀ hd']; linarith

theorem round_bounds (n d : ℕ) (hd : 0 < d) :
    |(((2 * n + d) / (2 * d) : ℕ) : ℝ) - (n : ℝ) / d| ≤ 1 / 2 := by
  have hd' : (0 : ℝ) < d := by exact_mod_cast hd
  have h1 : (2 * n + d) / (2 * d) * (2 * d) ≤ 2 * n + d := Nat.div_mul_le_self _ _
  have h2 : 2 * n + d < (2 * n + d) / (2 * d) * (2 * d) + 2 * d := Nat.lt_div_mul_add (by omega)
  have h1' : (((2 * n + d) / (2 * d) : ℕ) : ℝ) * (2 * d) ≤ 2 * n + d := by exact_mod_cast h1
  have h2' : (2 * (n : ℝ) + d) < (((2 * n + d) / (2 * d) : ℕ) : ℝ) * (2 * d) + 2 * d := by
    exact_mod_cast h2
  have e : (n : ℝ) / d * d = n := div_mul_cancel₀ _ (ne_of_gt hd')
  rw [abs_le]
  constructor <;> nlinarith

/-! ## The number a pattern denotes -/

/-- The magnitude of a finite binary64 pattern. -/
noncomputable def val64 (b : Nat) : ℝ :=
  ((F64.mant b * 2 ^ F64.shift b : ℕ) : ℝ) / ((2 ^ F64.bias : ℕ) : ℝ)

/-- The magnitude of a finite binary32 pattern. -/
noncomputable def val32 (b : Nat) : ℝ :=
  ((F32.mant b * 2 ^ F32.shift b : ℕ) : ℝ) / ((2 ^ F32.bias : ℕ) : ℝ)

theorem val64_nonneg (b : Nat) : 0 ≤ val64 b := by unfold val64; positivity

theorem val32_nonneg (b : Nat) : 0 ≤ val32 b := by unfold val32; positivity

/-- The sign bit does not enter the magnitude. -/
theorem fields_mod (b : Nat) :
    F64.expField (b % 0x8000000000000000) = F64.expField b ∧
    F64.frac (b % 0x8000000000000000) = F64.frac b := by
  unfold F64.expField F64.frac
  omega

theorem mant_mod (b : Nat) : F64.mant (b % 0x8000000000000000) = F64.mant b := by
  unfold F64.mant; rw [(fields_mod b).1, (fields_mod b).2]

theorem shift_mod (b : Nat) : F64.shift (b % 0x8000000000000000) = F64.shift b := by
  unfold F64.shift; rw [(fields_mod b).1]

theorem finite_mod (b : Nat) : F64.finite (b % 0x8000000000000000) = F64.finite b := by
  unfold F64.finite; rw [(fields_mod b).1]

theorem val64_mod (b : Nat) : val64 (b % 0x8000000000000000) = val64 b := by
  unfold val64; rw [mant_mod, shift_mod]

/-- The reading of a pattern used here is the one of `Bits.lean`: for a
pattern without sign bit, `mant * 2 ^ shift` is twice `magVal`, the magnitude
in units of 2⁻¹⁰⁷⁴ (and `2 ^ bias` is 2¹⁰⁷⁵). -/
theorem mant_shift_eq_magVal (m : Nat) (h : m < 0x8000000000000000) :
    F64.mant m * 2 ^ F64.shift m = 2 * magVal 52 m := by
  unfold F64.mant F64.shift F64.expField F64.frac magVal
  have h1 : m / 0x10000000000000 % 0x800 = m / 0x10000000000000 := by omega
  have h2 : (2 : ℕ) ^ 52 = 0x10000000000000 := by norm_num
  rw [h1, h2]
  by_cases h0 : m / 0x10000000000000 = 0
  · rw [if_pos h0, if_pos h0, if_pos h0]; ring
  · obtain ⟨k, hk⟩ : ∃ k, m / 0x10000000000000 = k + 1 := ⟨m / 0x10000000000000 - 1, by omega⟩
    rw [if_neg h0, if_neg h0, if_neg h0, hk, Nat.add_sub_cancel, pow_succ]
    ring

/-! ## Scan coordinates -/

theorem coordMagMm_le (mag : Nat) : coordMagMm mag ≤ coordLimit := by
  unfold coordMagMm
  split
  · exact Nat.le_refl _
  · exact Nat.min_le_left _ _

theorem shift_le (b : Nat) : F64.shift b ≤ 2047 ∧ 1 ≤ F64.shift b := by
  unfold F64.shift F64.expField; split <;> omega

/-- Below 1024 m the magnitude in millimetres is the floor of the exact
product, limited to `coordLimit`. -/
theorem coordMagMm_eq (mag : Nat) (h : F64.expField mag < 1033) :
    coordMagMm mag =
      min coordLimit (F64.mant mag * 2 ^ F64.shift mag * 1000 / 2 ^ F64.bias) := by
  unfold coordMagMm
  rw [if_neg (by omega)]
  congr 1
  have hb : F64.bias = 1075 := rfl
  have hs : F64.shift mag ≤ F64.bias := by
    unfold F64.shift; split <;> omega
  rw [Nat.shiftRight_eq_div_pow]
  have h2 : (2 : ℕ) ^ F64.bias = 2 ^ F64.shift mag * 2 ^ (F64.bias - F64.shift mag) := by
    rw [← pow_add]; congr 1; omega
  rw [h2, show F64.mant mag * 2 ^ F64.shift mag * 1000 =
    2 ^ F64.shift mag * (F64.mant mag * 1000) by ring]
  rw [Nat.mul_div_mul_left _ _ (by positivity)]

theorem val64_mul_1000 (b : Nat) :
    val64 b * 1000 = ((F64.mant b * 2 ^ F64.shift b * 1000 : ℕ) : ℝ) / ((2 ^ F64.bias : ℕ) : ℝ) := by
  unfold val64; push_cast; ring

/-- A finite value's magnitude in millimetres is never rounded up. -/
theorem coordMagMm_le_val (mag : Nat) (_hf : F64.finite mag = true) :
    (coordMagMm mag : ℝ) ≤ val64 mag * 1000 := by
  have hpos : 0 < 2 ^ F64.bias := by positivity
  by_cases h : F64.expField mag < 1033
  · rw [coordMagMm_eq mag h, val64_mul_1000]
    refine le_trans ?_ (floor_bounds _ _ hpos).1
    exact_mod_cast Nat.min_le_right _ _
  · -- 1024 m or more: the result is `coordLimit`, which is below the value.
    have hc : coordMagMm mag = coordLimit := by unfold coordMagMm; rw [if_pos (by omega)]
    have he : F64.expField mag ≠ 0 := by omega
    have hm : 2 ^ 52 ≤ F64.mant mag := by
      unfold F64.mant; rw [if_neg he]; omega
    have hs : F64.shift mag = F64.expField mag := by
      unfold F64.shift; rw [if_neg he]
    have hb : F64.bias = 1075 := rfl
    have hsplit : (2 : ℕ) ^ F64.bias = 2 ^ (F64.bias - 42) * 2 ^ 42 := by
      rw [← pow_add, Nat.sub_add_cancel (by omega)]
    have hp : (2 : ℕ) ^ (F64.bias - 42) ≤ 2 ^ F64.shift mag :=
      Nat.pow_le_pow_right (by norm_num) (by omega)
    have key : coordLimit * 2 ^ F64.bias ≤ F64.mant mag * 2 ^ F64.shift mag * 1000 := by
      rw [hsplit]
      generalize (2 : ℕ) ^ (F64.bias - 42) = X at hp ⊢
      generalize (2 : ℕ) ^ F64.shift mag = S at hp ⊢
      have h42 : coordLimit * 2 ^ 42 ≤ 2 ^ 52 * 1000 := by decide
      calc coordLimit * (X * 2 ^ 42) = coordLimit * 2 ^ 42 * X := by ring
        _ ≤ 2 ^ 52 * 1000 * X := Nat.mul_le_mul_right _ h42
        _ ≤ F64.mant mag * 1000 * S := Nat.mul_le_mul (Nat.mul_le_mul_right _ hm) hp
        _ = F64.mant mag * S * 1000 := by ring
    rw [hc, val64_mul_1000, le_div_iff₀ (by exact_mod_cast hpos)]
    exact_mod_cast key

/-- Unless it saturates, the magnitude in millimetres is less than 1 mm below
the true one. -/
theorem coordMagMm_gt_val (mag : Nat) (hlt : coordMagMm mag < coordLimit) :
    val64 mag * 1000 < (coordMagMm mag : ℝ) + 1 := by
  have hpos : 0 < 2 ^ F64.bias := by positivity
  have h : F64.expField mag < 1033 := by
    by_contra hc
    unfold coordMagMm at hlt
    rw [if_pos (by omega)] at hlt
    exact Nat.lt_irrefl _ hlt
  have heq := coordMagMm_eq mag h
  have hfl : coordMagMm mag = F64.mant mag * 2 ^ F64.shift mag * 1000 / 2 ^ F64.bias := by
    rw [heq] at hlt ⊢
    exact Nat.min_eq_right (Nat.le_of_lt (lt_of_not_ge fun hge => by
      rw [Nat.min_eq_left hge] at hlt; exact Nat.lt_irrefl _ hlt))
  rw [val64_mul_1000, hfl]
  exact (floor_bounds _ _ hpos).2

theorem natAbs_coordMmOfParts (neg : Bool) (mag : Nat) :
    (coordMmOfParts neg mag).natAbs = coordMagMm mag := by
  unfold coordMmOfParts
  split <;> simp

/-- A coordinate handed to the model is within `±coordLimit`, whatever the
pattern. -/
theorem coordMm_natAbs_le (b : Nat) : (coordMm b).natAbs ≤ coordLimit := by
  unfold coordMm
  rw [natAbs_coordMmOfParts]
  exact coordMagMm_le _

theorem abs_cast_eq_natAbs (z : Int) : |(z : ℝ)| = (z.natAbs : ℝ) := by
  rw [Nat.cast_natAbs, Int.cast_abs]

/-- **A point never moves away from the origin.** For a finite coordinate,
the magnitude of the model's millimetre value is at most the true magnitude
in millimetres. -/
theorem coordMm_abs_le (b : Nat) (hf : F64.finite b = true) :
    |((coordMm b : ℤ) : ℝ)| ≤ val64 b * 1000 := by
  have h := coordMagMm_le_val (b % 0x8000000000000000) (by rw [finite_mod]; exact hf)
  rw [val64_mod] at h
  rw [abs_cast_eq_natAbs]
  unfold coordMm
  rw [natAbs_coordMmOfParts]
  exact h

/-- Unless it saturates, the model's value is less than 1 mm short of the
true one. -/
theorem coordMm_abs_gt (b : Nat) (hlt : (coordMm b).natAbs < coordLimit) :
    val64 b * 1000 < |((coordMm b : ℤ) : ℝ)| + 1 := by
  have hn : (coordMm b).natAbs = coordMagMm (b % 0x8000000000000000) := by
    unfold coordMm; rw [natAbs_coordMmOfParts]
  have h := coordMagMm_gt_val (b % 0x8000000000000000) (by rw [← hn]; exact hlt)
  rw [val64_mod] at h
  rw [abs_cast_eq_natAbs, hn]
  exact h

/-- The sign of a coordinate is kept: a value with the sign bit clear is not
negative, one with the sign bit set (and not NaN) is not positive. -/
theorem coordMm_sign (b : Nat) :
    (F64.neg b = false → 0 ≤ coordMm b) ∧
    (F64.neg b = true → F64.isNaN (b % 0x8000000000000000) = false → coordMm b ≤ 0) := by
  unfold coordMm coordMmOfParts
  constructor
  · intro h; rw [h]; simp
  · intro h hn; rw [h, hn]; simp

/-! ## Radius -/

theorem radiusMm_of_not_finite (b : Nat) (h : F64.finite b = false) : radiusMm b = none := by
  unfold radiusMm; rw [h]; rfl

/-- The sign of the `radius` parameter does not matter (R9). -/
theorem radiusMm_abs (b : Nat) : radiusMm (b % 0x8000000000000000) = radiusMm b := by
  unfold radiusMm; rw [finite_mod, mant_mod, shift_mod]

theorem radiusOfQuot_eq_some (n d r : Nat) (h : radiusOfQuot n d = some r) :
    r = (n + (d - 1)) / d + 1 ∧ r ≤ coordLimit := by
  unfold radiusOfQuot at h
  by_cases hle : coordLimit < (n + (d - 1)) / d + 1
  · simp [hle] at h
  · simp only [hle, if_false, Option.some.injEq] at h
    exact ⟨h.symm, by omega⟩

theorem radiusMm_eq_some (b r : Nat) (h : radiusMm b = some r) :
    r = (F64.mant b * 1000 * 2 ^ F64.shift b + (2 ^ F64.bias - 1)) / 2 ^ F64.bias + 1 ∧
      r ≤ coordLimit := by
  unfold radiusMm at h
  by_cases hf : (!F64.finite b) = true
  · rw [if_pos hf] at h; cases h
  · rw [if_neg hf] at h
    exact radiusOfQuot_eq_some _ _ r h

/-- An accepted radius is at most `coordLimit`, so `Shape.ok` holds. -/
theorem radiusMm_le_limit (b r : Nat) (h : radiusMm b = some r) : r ≤ coordLimit :=
  (radiusMm_eq_some b r h).2

theorem val64_mul_1000' (b : Nat) :
    val64 b * 1000 = ((F64.mant b * 1000 * 2 ^ F64.shift b : ℕ) : ℝ) / ((2 ^ F64.bias : ℕ) : ℝ) := by
  unfold val64; push_cast; ring

/-- **The model's radius covers the configured one**: it is at least the true
magnitude in millimetres, plus 1. -/
theorem radiusMm_ge (b r : Nat) (h : radiusMm b = some r) : val64 b * 1000 + 1 ≤ (r : ℝ) := by
  obtain ⟨hr, -⟩ := radiusMm_eq_some b r h
  have := (ceil_bounds (F64.mant b * 1000 * 2 ^ F64.shift b) (2 ^ F64.bias) (by positivity)).1
  rw [val64_mul_1000', hr, Nat.cast_add, Nat.cast_one]
  generalize (2 : ℕ) ^ F64.bias = D at this ⊢
  linarith

/-- The model's radius exceeds the configured one by less than 2 mm. -/
theorem radiusMm_lt (b r : Nat) (h : radiusMm b = some r) : (r : ℝ) < val64 b * 1000 + 2 := by
  obtain ⟨hr, -⟩ := radiusMm_eq_some b r h
  have := (ceil_bounds (F64.mant b * 1000 * 2 ^ F64.shift b) (2 ^ F64.bias) (by positivity)).2
  rw [val64_mul_1000', hr, Nat.cast_add, Nat.cast_one]
  generalize (2 : ℕ) ^ F64.bias = D at this ⊢
  linarith

/-- **Circles are safe.** Take a circle zone whose `radius` parameter has
pattern `br` and was accepted as `r` millimetres, and a point whose
coordinates the shim passed as the finite patterns `bx` and `by'`. If the
point lies strictly inside the configured circle, in exact arithmetic on the
values these patterns denote, the model counts its millimetre point as inside
the zone. Rounding never loses an obstacle. -/
theorem circle_safe (br bx by' r : Nat) (hr : radiusMm br = some r)
    (hx : F64.finite bx = true) (hy : F64.finite by' = true)
    (hin : val64 bx ^ 2 + val64 by' ^ 2 < val64 br ^ 2) :
    (Shape.circle r).contains ⟨coordMm bx, coordMm by'⟩ = true := by
  have h0 := val64_nonneg br
  have hx0 := val64_nonneg bx
  have hy0 := val64_nonneg by'
  apply contains_circle_of_truncated r _ (val64 br * 1000) (val64 bx * 1000) (val64 by' * 1000)
  · exact mul_nonneg h0 (by norm_num)
  · have := radiusMm_ge br r hr; linarith
  · nlinarith
  · rw [abs_of_nonneg (mul_nonneg hx0 (by norm_num))]
    exact coordMm_abs_le bx hx
  · rw [abs_of_nonneg (mul_nonneg hy0 (by norm_num))]
    exact coordMm_abs_le by' hy

/-! ## Vertices -/

theorem vertexMm_of_not_finite (b : Nat) (h : F32.finite b = false) : vertexMm b = none := by
  unfold vertexMm; rw [h]; rfl

theorem vertexMm_isSome_iff (b : Nat) : (vertexMm b).isSome = true ↔ F32.finite b = true := by
  unfold vertexMm
  cases F32.finite b <;> simp

/-- **A vertex moves by at most half a millimetre.** -/
theorem vertexMm_abs_sub_le (b : Nat) (v : Int) (h : vertexMm b = some v) :
    |(|(v : ℝ)| - val32 b * 1000)| ≤ 1 / 2 := by
  unfold vertexMm at h
  split at h
  · cases h
  · simp only at h
    have hv := Option.some.inj h
    have hd : (2 : ℕ) ^ (F32.bias + 1) = 2 * 2 ^ F32.bias := by rw [pow_succ]; ring
    rw [hd] at hv
    have habs : |(v : ℝ)| =
        (((2 * (F32.mant b * 1000 * 2 ^ F32.shift b) + 2 ^ F32.bias) / (2 * 2 ^ F32.bias) : ℕ) : ℝ) := by
      rw [← hv]
      split
      · rw [Int.cast_neg, abs_neg, Int.cast_natCast, abs_of_nonneg (Nat.cast_nonneg _)]
      · rw [Int.cast_natCast, abs_of_nonneg (Nat.cast_nonneg _)]
    have hval : val32 b * 1000 =
        ((F32.mant b * 1000 * 2 ^ F32.shift b : ℕ) : ℝ) / ((2 ^ F32.bias : ℕ) : ℝ) := by
      unfold val32; push_cast; ring
    rw [habs, hval]
    exact round_bounds _ _ (by positivity)

/-! ## Timeouts

A refused timeout must not turn into 0: for the source that would mean "check
disabled". -/

theorem timeoutNs_of_not_finite (b : Nat) (h : F64.finite b = false) : timeoutNs b = none := by
  unfold timeoutNs; rw [h]; rfl

/-- A negative timeout is refused; only `-0.0` passes, as 0. -/
theorem timeoutNs_of_neg (b : Nat) (h : F64.neg b = true) (hm : F64.mant b ≠ 0) :
    timeoutNs b = none := by
  unfold timeoutNs
  split
  · rfl
  · rw [if_pos (by simp [h, hm])]

/-- A timeout of 9.2·10⁹ s or more is refused. -/
theorem timeoutNs_of_huge (b : Nat) (h : 9200000000 ≤ val64 b) : timeoutNs b = none := by
  unfold timeoutNs
  split
  · rfl
  · split
    · rfl
    · rw [if_pos]
      unfold val64 at h
      have hD : 0 < 2 ^ F64.bias := by positivity
      generalize (2 : ℕ) ^ F64.bias = D at h hD ⊢
      rw [le_div_iff₀ (by exact_mod_cast hD)] at h
      exact_mod_cast h

/-- An accepted timeout is finite, not negative (up to `-0.0`), and below
9.2·10⁹ s. -/
theorem timeoutNs_isSome (b : Nat) (h : (timeoutNs b).isSome = true) :
    F64.finite b = true ∧ (F64.neg b = true → F64.mant b = 0) ∧ val64 b < 9200000000 := by
  refine ⟨?_, ?_, ?_⟩
  · by_contra hc
    rw [timeoutNs_of_not_finite b (by simpa using hc)] at h; cases h
  · intro hn
    by_contra hc
    rw [timeoutNs_of_neg b hn hc] at h; cases h
  · by_contra hc
    rw [timeoutNs_of_huge b (not_lt.mp hc)] at h; cases h

/-- The source's timeout is refused whenever the node's `source_timeout` is,
and whenever its own is, if it has one. -/
theorem sourceTimeoutNs_eq_none (node : Nat) (hasOwn : Bool) (own : Nat)
    (h : timeoutNs node = none ∨ (hasOwn = true ∧ timeoutNs own = none)) :
    sourceTimeoutNs node hasOwn own = none := by
  unfold sourceTimeoutNs
  rcases h with h | ⟨h1, h2⟩
  · rw [h]
  · cases timeoutNs node with
    | none => rfl
    | some n => simp [h1, h2]

/-- With its own parameter set, the source's timeout is that parameter's
conversion. -/
theorem sourceTimeoutNs_own (node own n : Nat) (h : timeoutNs node = some n) :
    sourceTimeoutNs node true own = timeoutNs own := by
  unfold sourceTimeoutNs; rw [h]; rfl

/-! ## `min_points` -/

/-- `min_points` is accepted iff it is in the range of a 32-bit `int`; a
negative value is then 0. -/
theorem minPointsOfRaw_eq_some_iff (v : Int) (n : Nat) :
    minPointsOfRaw v = some n ↔ -2 ^ 31 ≤ v ∧ v ≤ 2 ^ 31 - 1 ∧ n = v.toNat := by
  unfold minPointsOfRaw
  split
  · rename_i h
    simp only [Bool.or_eq_true, decide_eq_true_eq] at h
    constructor
    · intro h'; cases h'
    · rintro ⟨h1, h2, -⟩; omega
  · rename_i h
    simp only [Bool.or_eq_true, decide_eq_true_eq, not_or, not_lt] at h
    constructor
    · intro h'; exact ⟨by omega, by omega, (Option.some.inj h').symm⟩
    · rintro ⟨-, -, rfl⟩; rfl

end ROSlean.CollisionMonitor.Convert
