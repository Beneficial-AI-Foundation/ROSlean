import Mathlib.Tactic
import ROSleanCore.CollisionMonitor

/-!
# Collision monitor: the IEEE-754 bit-pattern helpers

The model never computes with floating-point numbers; it classifies and
compares bit patterns with `F32.le`, `F64.isFinite`, `F64.isZero` and the
order of `Nat` (`ROSleanCore.CollisionMonitor`). This file proves what those
definitions do, against a small definition of the number a pattern denotes:

* `magVal w m` is the magnitude denoted by the sign-less pattern `m` of a
  binary format with `w` fraction bits, as a natural number of units of the
  smallest subnormal (2⁻¹⁴⁹ for binary32, 2⁻¹⁰⁷⁴ for binary64). It is
  IEEE-754's formula: `fraction` for exponent field 0, and
  `(2^w + fraction) · 2^(exponent − 1)` otherwise.
* `magVal w` is strictly increasing (`magVal_strictMono`), which is the fact
  "magnitudes are ordered as their patterns" that the model relies on.

What stays trusted is listed in `docs/collision_monitor/THEOREMS.md`: that
`magVal` is the right reading of a pattern, and that the C++ compiler's
`float` and `double` are IEEE-754 binary32 and binary64.
-/

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## The magnitude a pattern denotes -/

/-- The magnitude denoted by a sign-less pattern `m` in a binary format with
`w` fraction bits, in units of the smallest subnormal: exponent field
`m / 2^w`, fraction `m % 2^w`. For the pattern of an infinity this is the
power of two just above every finite magnitude, so the order still holds. -/
def magVal (w m : Nat) : Nat :=
  if m / 2 ^ w = 0 then m % 2 ^ w else (2 ^ w + m % 2 ^ w) * 2 ^ (m / 2 ^ w - 1)

/-- Magnitudes are ordered as their patterns. -/
theorem magVal_strictMono (w : Nat) : StrictMono (magVal w) := by
  intro a b hab
  have hpos : 0 < 2 ^ w := by positivity
  have ha := Nat.div_add_mod a (2 ^ w)
  have hb := Nat.div_add_mod b (2 ^ w)
  have hfa := Nat.mod_lt a hpos
  have hfb := Nat.mod_lt b hpos
  have hle : a / 2 ^ w ≤ b / 2 ^ w := Nat.div_le_div_right hab.le
  unfold magVal
  generalize a / 2 ^ w = ea at *
  generalize a % 2 ^ w = fa at *
  generalize b / 2 ^ w = eb at *
  generalize b % 2 ^ w = fb at *
  generalize 2 ^ w = P at *
  rcases hle.eq_or_lt with heq | hlt
  · subst heq
    have hf : fa < fb := by omega
    split_ifs
    · exact hf
    · exact Nat.mul_lt_mul_of_pos_right (by omega) (by positivity)
  · have heb : eb ≠ 0 := by omega
    simp only [heb, ↓reduceIte]
    have h1 : P * 2 ^ (eb - 1) ≤ (P + fb) * 2 ^ (eb - 1) :=
      Nat.mul_le_mul_right _ (by omega)
    refine lt_of_lt_of_le ?_ h1
    split_ifs with hea
    · calc fa < P := hfa
        _ = P * 1 := (Nat.mul_one P).symm
        _ ≤ P * 2 ^ (eb - 1) := Nat.mul_le_mul_left _ Nat.one_le_two_pow
    · obtain ⟨n, rfl⟩ : ∃ n, ea = n + 1 := ⟨ea - 1, by omega⟩
      calc (P + fa) * 2 ^ (n + 1 - 1) < (P + P) * 2 ^ n := by
            simp only [Nat.add_sub_cancel]
            exact Nat.mul_lt_mul_of_pos_right (by omega) (by positivity)
        _ = P * 2 ^ (n + 1) := by ring
        _ ≤ P * 2 ^ (eb - 1) :=
            Nat.mul_le_mul_left _ (Nat.pow_le_pow_right (by norm_num) (by omega))

theorem magVal_lt_iff (w a b : Nat) : magVal w a < magVal w b ↔ a < b :=
  (magVal_strictMono w).lt_iff_lt

theorem magVal_le_iff (w a b : Nat) : magVal w a ≤ magVal w b ↔ a ≤ b :=
  (magVal_strictMono w).le_iff_le

theorem magVal_zero (w : Nat) : magVal w 0 = 0 := by simp [magVal]

/-- Only the all-zero pattern denotes zero. -/
theorem magVal_eq_zero_iff (w m : Nat) : magVal w m = 0 ↔ m = 0 := by
  constructor
  · intro h
    by_contra hm
    have := (magVal_lt_iff w 0 m).mpr (Nat.pos_of_ne_zero hm)
    rw [magVal_zero] at this
    omega
  · rintro rfl
    exact magVal_zero w

/-! ## binary32: `F32.le` -/

/-- The number a binary32 pattern denotes, in units of 2⁻¹⁴⁹: the magnitude
with the sign applied. Meaningful when the pattern is not NaN. -/
def f32Value (b : Nat) : Int :=
  if F32.neg b then -(magVal 23 (F32.mag b) : Int) else (magVal 23 (F32.mag b) : Int)

/-- A pattern is NaN iff, without its sign bit, it lies above the pattern of
+∞. -/
theorem isNaN_iff (b : Nat) : F32.isNaN b = true ↔ 0x7F800000 < b % 0x80000000 := by
  unfold F32.isNaN F32.mag
  exact decide_eq_true_iff

/-- R4: `<=` is false when the left side is NaN. -/
theorem le_of_isNaN_left (a b : Nat) (h : F32.isNaN a = true) : F32.le a b = false := by
  simp [F32.le, h]

/-- R4: `<=` is false when the right side is NaN. -/
theorem le_of_isNaN_right (a b : Nat) (h : F32.isNaN b = true) : F32.le a b = false := by
  simp [F32.le, h]

/-- If `a <= b` holds, neither side is NaN. -/
theorem not_isNaN_of_le (a b : Nat) (h : F32.le a b = true) :
    F32.isNaN a = false ∧ F32.isNaN b = false := by
  cases ha : F32.isNaN a <;> cases hb : F32.isNaN b <;> simp_all [F32.le]

/-- Two patterns with the sign bit clear, neither NaN, compare as their
magnitude patterns. -/
theorem le_iff_of_nonneg (a b : Nat) (ha : F32.isNaN a = false) (hb : F32.isNaN b = false)
    (hna : F32.neg a = false) (hnb : F32.neg b = false) :
    F32.le a b = true ↔ F32.mag a ≤ F32.mag b := by
  simp [F32.le, ha, hb, hna, hnb]

/-- For patterns up to that of +∞ (the non-negative numbers and +∞),
`F32.le` is the order of the patterns. -/
theorem le_iff_of_le_inf (a b : Nat) (ha : a ≤ 0x7F800000) (hb : b ≤ 0x7F800000) :
    F32.le a b = true ↔ a ≤ b := by
  have hma : F32.mag a = a := by unfold F32.mag; omega
  have hmb : F32.mag b = b := by unfold F32.mag; omega
  have hna : F32.neg a = false := by
    have : a / 0x80000000 = 0 := by omega
    simp [F32.neg, this]
  have hnb : F32.neg b = false := by
    have : b / 0x80000000 = 0 := by omega
    simp [F32.neg, this]
  have hia : F32.isNaN a = false := by
    simp only [F32.isNaN, hma, decide_eq_false_iff_not]; omega
  have hib : F32.isNaN b = false := by
    simp only [F32.isNaN, hmb, decide_eq_false_iff_not]; omega
  rw [le_iff_of_nonneg a b hia hib hna hnb, hma, hmb]

/-- **`F32.le` is `≤` on the denoted numbers.** For two patterns that are not
NaN, `F32.le a b` holds iff the number denoted by `a` is at most the number
denoted by `b`. This includes `-0 <= +0` and `+0 <= -0` (both denote 0) and
the infinities (which `f32Value` places beyond every finite number). -/
theorem le_iff_value (a b : Nat) (ha : F32.isNaN a = false) (hb : F32.isNaN b = false) :
    F32.le a b = true ↔ f32Value a ≤ f32Value b := by
  have hmono := magVal_le_iff 23 (F32.mag a) (F32.mag b)
  have hmono' := magVal_le_iff 23 (F32.mag b) (F32.mag a)
  have hza := magVal_eq_zero_iff 23 (F32.mag a)
  have hzb := magVal_eq_zero_iff 23 (F32.mag b)
  unfold F32.le f32Value
  cases hna : F32.neg a <;> cases hnb : F32.neg b <;>
    simp only [ha, hb, Bool.not_false, Bool.and_self, Bool.true_and, decide_eq_true_eq,
      Bool.and_eq_true, beq_iff_eq, Bool.false_eq_true, ↓reduceIte]
  · rw [← hmono]; omega
  · rw [← hza, ← hzb]; omega
  · exact ⟨fun _ => by omega, fun _ => trivial⟩
  · rw [← hmono']; omega

/-- `<=` is reflexive on patterns that are not NaN. -/
theorem le_refl_of_not_isNaN (a : Nat) (ha : F32.isNaN a = false) : F32.le a a = true :=
  (le_iff_value a a ha ha).mpr (le_refl _)

/-- R4: a NaN reading is not valid. -/
theorem validBeam_of_isNaN_range (s : Scan) (b : Beam) (h : F32.isNaN b.range = true) :
    s.validBeam b = false := by
  simp [Scan.validBeam, le_of_isNaN_left _ _ h]

/-- R4: if `range_min` is NaN, no reading is valid. -/
theorem validBeam_of_isNaN_rangeMin (s : Scan) (b : Beam) (h : F32.isNaN s.rangeMin = true) :
    s.validBeam b = false := by
  simp [Scan.validBeam, le_of_isNaN_left _ _ h]

/-- R4: if `range_max` is NaN, no reading is valid. -/
theorem validBeam_of_isNaN_rangeMax (s : Scan) (b : Beam) (h : F32.isNaN s.rangeMax = true) :
    s.validBeam b = false := by
  simp [Scan.validBeam, le_of_isNaN_right _ _ h]

/-- R4: a reading is valid iff none of the reading, `range_min` and
`range_max` is NaN and the number the reading denotes lies between those of
`range_min` and `range_max`, inclusive. -/
theorem validBeam_iff_value (s : Scan) (b : Beam) :
    s.validBeam b = true ↔
      F32.isNaN s.rangeMin = false ∧ F32.isNaN b.range = false ∧ F32.isNaN s.rangeMax = false ∧
        f32Value s.rangeMin ≤ f32Value b.range ∧ f32Value b.range ≤ f32Value s.rangeMax := by
  unfold Scan.validBeam
  rw [Bool.and_eq_true]
  constructor
  · rintro ⟨h1, h2⟩
    have n1 := not_isNaN_of_le _ _ h1
    have n2 := not_isNaN_of_le _ _ h2
    exact ⟨n1.1, n1.2, n2.2, (le_iff_value _ _ n1.1 n1.2).mp h1,
      (le_iff_value _ _ n2.1 n2.2).mp h2⟩
  · rintro ⟨n1, n2, n3, h1, h2⟩
    exact ⟨(le_iff_value _ _ n1 n2).mpr h1, (le_iff_value _ _ n2 n3).mpr h2⟩

/-! ## binary64: `F64.isFinite`, `F64.isZero` and the order of ratios -/

/-- A pattern is finite iff, without its sign bit, it lies below the pattern
of +∞. Bits above the 64th are ignored. -/
theorem isFinite_iff (b : Nat) :
    F64.isFinite b = true ↔ b % 0x8000000000000000 < 0x7FF0000000000000 := by
  simp only [F64.isFinite, bne_iff_ne, ne_eq]
  omega

/-- A pattern is zero iff all bits below the sign bit are clear. -/
theorem isZero_iff (b : Nat) : F64.isZero b = true ↔ b % 0x8000000000000000 = 0 := by
  simp [F64.isZero]

/-- A 64-bit pattern is zero iff it is that of `+0.0` or of `-0.0`. -/
theorem isZero_iff_of_lt (b : Nat) (h : b < 0x10000000000000000) :
    F64.isZero b = true ↔ b = 0 ∨ b = 0x8000000000000000 := by
  rw [isZero_iff]
  omega

/-- A pattern is zero iff the magnitude it denotes is 0. -/
theorem isZero_iff_magVal (b : Nat) :
    F64.isZero b = true ↔ magVal 52 (b % 0x8000000000000000) = 0 := by
  rw [isZero_iff, magVal_eq_zero_iff]

/-- A zero is finite. -/
theorem isFinite_of_isZero (b : Nat) (h : F64.isZero b = true) : F64.isFinite b = true := by
  rw [isZero_iff] at h
  rw [isFinite_iff]
  omega

/-- The pattern `F64.one` denotes 1.0: 2¹⁰⁷⁴ units of 2⁻¹⁰⁷⁴. -/
theorem magVal_one : magVal 52 F64.one = 2 ^ 1074 := by
  decide +kernel

/-- **A ratio pattern below `F64.one` denotes a number in `[0, 1)`**, and
conversely: `r < F64.one` iff the sign bit of `r` is clear (and no higher bit
is set) and the magnitude `r` denotes is below that of 1.0. -/
theorem lt_one_iff (r : Nat) :
    r < F64.one ↔ r < 0x8000000000000000 ∧ magVal 52 r < 2 ^ 1074 := by
  rw [← magVal_one, magVal_lt_iff]
  unfold F64.one
  omega

/-- A ratio pattern below `F64.one` is finite. -/
theorem isFinite_of_lt_one (r : Nat) (h : r < F64.one) : F64.isFinite r = true := by
  rw [isFinite_iff]
  unfold F64.one at h
  omega

/-- Among ratio patterns below `F64.one`, only the all-zero pattern is a
zero. -/
theorem isZero_iff_of_lt_one (r : Nat) (h : r < F64.one) : F64.isZero r = true ↔ r = 0 := by
  rw [isZero_iff]
  unfold F64.one at h
  omega

/-- R2: a command is finite iff each of its six components, without the sign
bit, is below the pattern of +∞. -/
theorem finite_iff (v : Twist) :
    v.finite = true ↔
      v.linX % 0x8000000000000000 < 0x7FF0000000000000 ∧
      v.linY % 0x8000000000000000 < 0x7FF0000000000000 ∧
      v.linZ % 0x8000000000000000 < 0x7FF0000000000000 ∧
      v.angX % 0x8000000000000000 < 0x7FF0000000000000 ∧
      v.angY % 0x8000000000000000 < 0x7FF0000000000000 ∧
      v.angZ % 0x8000000000000000 < 0x7FF0000000000000 := by
  simp only [Twist.finite, Bool.and_eq_true, isFinite_iff, and_assoc]

/-- R15: the published components of a command are zero iff `linear.x`,
`linear.y` and `angular.z` have all bits below the sign bit clear. -/
theorem planarZero_iff (v : Twist) :
    v.planarZero = true ↔
      v.linX % 0x8000000000000000 = 0 ∧ v.linY % 0x8000000000000000 = 0 ∧
      v.angZ % 0x8000000000000000 = 0 := by
  simp only [Twist.planarZero, Bool.and_eq_true, isZero_iff, and_assoc]

end ROSlean.CollisionMonitor
