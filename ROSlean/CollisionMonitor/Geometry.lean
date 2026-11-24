import ROSlean.CollisionMonitor.Basic

/-!
# Collision monitor: the zone tests

What `Shape.contains` (`ROSleanCore.CollisionMonitor`) means, and why the
integer tests err only towards "inside" (SPEC, "Safe direction", D3, D4).

* Circle: `contains_circle_iff` (R9) and `contains_circle_of_truncated` (a
  point strictly inside the configured circle is inside the integer one).
* Polygon: `crosses_iff_upstream` (the multiplied-out crossing test is
  upstream's test with the division, in exact arithmetic);
  `contains_polygon_iff` (inside by upstream's rule, or near the boundary);
  `nearSegment_iff` (the margin test is exactly "at most √5 mm from the
  segment"); `nearBoundary_of_rounded` (a point of the boundary of the
  polygon as configured is, after rounding of the vertices and truncation of
  the point, within the margin).
* Saturated points: `contains_eq_false_of_far` (with `Shape.ok`, a point
  with a coordinate at the limit is in no zone, D12) and
  `zonesAction_filter_inRange`.

The one step of the safe-direction argument that is *not* proved is
topological; it is stated before `nearBoundary_of_rounded`.
-/

set_option linter.style.longLine false -- the casts to real numbers make long lines

namespace ROSlean.CollisionMonitor

open ROSleanCore.CollisionMonitor

/-! ## Circle (R9) -/

/-- R9: a point is inside a circle zone iff its squared distance from the
origin is strictly below the squared radius. -/
theorem contains_circle_iff (r : Nat) (p : Point) :
    (Shape.circle r).contains p = true ↔ p.x ^ 2 + p.y ^ 2 < (r : Int) ^ 2 := by
  unfold Shape.contains
  rw [decide_eq_true_iff]
  push_cast
  simp only [sq]

/-- **Safe direction for circles.** Let `(x, y)` be a point strictly inside
the circle of radius `R ≥ 0` (real numbers, millimetres). If the model's
radius `r` is at least `R` and the model's point `p` is no further from each
axis than `(x, y)` (truncation toward zero), then the model counts `p` as
inside. -/
theorem contains_circle_of_truncated (r : Nat) (p : Point) (R x y : ℝ) (hR0 : 0 ≤ R)
    (hR : R ≤ r) (hin : x ^ 2 + y ^ 2 < R ^ 2) (hx : |(p.x : ℝ)| ≤ |x|) (hy : |(p.y : ℝ)| ≤ |y|) :
    (Shape.circle r).contains p = true := by
  rw [contains_circle_iff]
  have h1 : (p.x : ℝ) ^ 2 ≤ x ^ 2 := sq_le_sq.mpr hx
  have h2 : (p.y : ℝ) ^ 2 ≤ y ^ 2 := sq_le_sq.mpr hy
  have h3 : R ^ 2 ≤ (r : ℝ) ^ 2 := pow_le_pow_left₀ hR0 hR 2
  have : ((p.x : ℝ)) ^ 2 + (p.y : ℝ) ^ 2 < (r : ℝ) ^ 2 := by linarith
  exact_mod_cast this

/-! ## Polygon: the crossing rule (R10, D3) -/

/-- The cyclic list of pairs (previous element, element), starting with (last,
first). `edges` is this for points. -/
def cyclicPairs {α : Type} (vs : List α) : List (α × α) :=
  match vs.getLast? with
  | none => []
  | some l => (l :: vs).zip vs

theorem edges_eq_cyclicPairs (vs : List Point) : edges vs = cyclicPairs vs := by
  unfold edges cyclicPairs
  cases vs.getLast? <;> rfl

theorem cyclicPairs_map {α β : Type} (f : α → β) (vs : List α) :
    cyclicPairs (vs.map f) = (cyclicPairs vs).map (Prod.map f f) := by
  unfold cyclicPairs
  rw [List.getLast?_map]
  cases vs.getLast? with
  | none => rfl
  | some l => simp [← List.map_cons, List.zip_map]

/-- Both ends of an edge are vertices. -/
theorem mem_of_mem_cyclicPairs {α : Type} {vs : List α} {e : α × α} (h : e ∈ cyclicPairs vs) :
    e.1 ∈ vs ∧ e.2 ∈ vs := by
  unfold cyclicPairs at h
  cases hl : vs.getLast? with
  | none => simp [hl] at h
  | some l =>
    rw [hl] at h
    obtain ⟨a, b⟩ := e
    have := List.of_mem_zip h
    refine ⟨?_, this.2⟩
    rcases List.mem_cons.mp this.1 with rfl | h1
    · exact List.mem_of_getLast? hl
    · exact h1

/-- A polygon with `n` vertices has `n` edges. -/
theorem length_edges (vs : List Point) : (edges vs).length = vs.length := by
  unfold edges
  cases hl : vs.getLast? with
  | none => simp [List.getLast?_eq_none_iff.mp hl]
  | some l => simp

/-- R10, D3: the crossing test of the model is upstream's test, evaluated in
exact (rational) arithmetic: the edge straddles the ray's height in the
half-open sense, and the intersection `a.x + (p.y − a.y)·(b.x − a.x)/(b.y −
a.y)` lies strictly to the right of the point. -/
theorem crosses_iff_upstream (p a b : Point) :
    crosses p a b = true ↔
      ((p.y ≤ a.y) ↔ (b.y < p.y)) ∧
        (p.x : ℚ) < a.x + ((p.y : ℚ) - a.y) * ((b.x : ℚ) - a.x) / ((b.y : ℚ) - a.y) := by
  unfold crosses
  simp only [Bool.and_eq_true, beq_iff_eq, decide_eq_decide]
  refine and_congr_right fun hs => ?_
  by_cases hd : 0 < b.y - a.y
  · have hd' : (0 : ℚ) < (b.y : ℚ) - a.y := by exact_mod_cast hd
    simp only [hd, ↓reduceIte, decide_eq_true_eq]
    rw [← sub_lt_iff_lt_add', lt_div_iff₀ hd']
    exact_mod_cast Iff.rfl
  · have hne : b.y ≠ a.y := by
      intro h
      rw [h] at hs
      omega
    have hd' : (b.y : ℚ) - a.y < 0 := by
      have : b.y - a.y < 0 := by omega
      exact_mod_cast this
    simp only [hd, ↓reduceIte, decide_eq_true_eq]
    rw [← sub_lt_iff_lt_add', lt_div_iff_of_neg hd']
    exact_mod_cast Iff.rfl

/-- A horizontal edge is never crossed (R10). -/
theorem crosses_of_horizontal (p a b : Point) (h : a.y = b.y) : crosses p a b = false := by
  unfold crosses
  have : (decide (p.y ≤ a.y) == decide (b.y < p.y)) = false := by
    rw [h]
    by_cases h' : p.y ≤ b.y <;> simp [h']
    omega
  simp [this]

/-- R10, D4: a point counts as inside a polygon zone iff it is inside by
upstream's rule (an odd number of crossed edges) or within the margin of the
boundary. In particular every point upstream's rule puts inside the integer
polygon is counted. -/
theorem contains_polygon_iff (vs : List Point) (p : Point) :
    (Shape.polygon vs).contains p = true ↔
      ((edges vs).countP (fun e => crosses p e.1 e.2) % 2 = 1) ∨
      ∃ e ∈ edges vs, nearSegment p e.1 e.2 = true := by
  simp [Shape.contains, insidePolygon, nearBoundary]

theorem contains_of_insidePolygon (vs : List Point) (p : Point) (h : insidePolygon vs p = true) :
    (Shape.polygon vs).contains p = true := by
  simp [Shape.contains, h]

theorem contains_of_nearBoundary (vs : List Point) (p : Point) (h : nearBoundary vs p = true) :
    (Shape.polygon vs).contains p = true := by
  simp [Shape.contains, h]

/-! ## Polygon: the margin (D4) -/

/-- **The margin test is a distance test.** `nearSegment p a b` holds iff
some point of the closed segment from `a` to `b` is at squared distance at
most 5 mm² from `p`, i.e. iff `p` is at most √5 mm from the segment. -/
theorem nearSegment_iff (p a b : Point) :
    nearSegment p a b = true ↔
      ∃ t : ℝ, 0 ≤ t ∧ t ≤ 1 ∧
        ((p.x : ℝ) - (a.x + t * ((b.x : ℝ) - a.x))) ^ 2 +
          ((p.y : ℝ) - (a.y + t * ((b.y : ℝ) - a.y))) ^ 2 ≤ 5 := by
  obtain ⟨px, py⟩ := p
  obtain ⟨ax, ay⟩ := a
  obtain ⟨bx, by'⟩ := b
  simp only [nearSegment, Bool.or_eq_true, Bool.and_eq_true, decide_eq_true_eq]
  simp only [polygonMarginSq]
  constructor
  · rintro ((h | h) | ⟨⟨h1, h2⟩, h3⟩)
    · refine ⟨0, le_refl _, zero_le_one, ?_⟩
      have : (((px : ℝ) - ax) * ((px : ℝ) - ax) + ((py : ℝ) - ay) * ((py : ℝ) - ay)) ≤ 5 := by
        exact_mod_cast h
      nlinarith
    · refine ⟨1, zero_le_one, le_refl _, ?_⟩
      have : (((px : ℝ) - bx) * ((px : ℝ) - bx) + ((py : ℝ) - by') * ((py : ℝ) - by')) ≤ 5 := by
        exact_mod_cast h
      nlinarith
    · have h1' : (0 : ℝ) < ((bx : ℝ) - ax) * ((px : ℝ) - ax) + ((by' : ℝ) - ay) * ((py : ℝ) - ay) := by
        exact_mod_cast h1
      have h2' : ((bx : ℝ) - ax) * ((px : ℝ) - ax) + ((by' : ℝ) - ay) * ((py : ℝ) - ay) <
          ((bx : ℝ) - ax) * ((bx : ℝ) - ax) + ((by' : ℝ) - ay) * ((by' : ℝ) - ay) := by
        exact_mod_cast h2
      have h3' : (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) *
          (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) ≤
          5 * (((bx : ℝ) - ax) * ((bx : ℝ) - ax) + ((by' : ℝ) - ay) * ((by' : ℝ) - ay)) := by
        exact_mod_cast h3
      generalize ((bx : ℝ) - ax) = u at *
      generalize ((by' : ℝ) - ay) = v at *
      have hL : 0 < u * u + v * v := lt_trans h1' h2'
      obtain ⟨T, hT⟩ : ∃ T : ℝ, T * (u * u + v * v) = u * ((px : ℝ) - ax) + v * ((py : ℝ) - ay) :=
        ⟨_, div_mul_cancel₀ _ hL.ne'⟩
      have hT0 : 0 ≤ T := by
        by_contra hneg
        nlinarith [mul_pos_of_neg_of_neg (not_le.mp hneg) (neg_neg_of_pos hL)]
      have hT1 : T ≤ 1 := by
        by_contra hgt
        nlinarith [mul_lt_mul_of_pos_right (not_le.mp hgt) hL]
      refine ⟨T, hT0, hT1, ?_⟩
      have hid : (((px : ℝ) - (ax + T * u)) ^ 2 + ((py : ℝ) - (ay + T * v)) ^ 2) * (u * u + v * v) =
          (u * ((py : ℝ) - ay) - v * ((px : ℝ) - ax)) * (u * ((py : ℝ) - ay) - v * ((px : ℝ) - ax)) +
            ((u * ((px : ℝ) - ax) + v * ((py : ℝ) - ay)) - T * (u * u + v * v)) ^ 2 := by
        ring
      rw [hT, sub_self] at hid
      refine le_of_mul_le_mul_right ?_ hL
      rw [hid]
      linarith
  · rintro ⟨t, ht0, ht1, hD⟩
    -- the three cases of the model: foot before `a`, beyond `b`, or between
    set dot : ℤ := (bx - ax) * (px - ax) + (by' - ay) * (py - ay) with hdot
    set len : ℤ := (bx - ax) * (bx - ax) + (by' - ay) * (by' - ay) with hlen
    have hdotR : (dot : ℝ) = ((bx : ℝ) - ax) * ((px : ℝ) - ax) + ((by' : ℝ) - ay) * ((py : ℝ) - ay) := by
      rw [hdot]; push_cast; ring
    have hlenR : (len : ℝ) = ((bx : ℝ) - ax) * ((bx : ℝ) - ax) + ((by' : ℝ) - ay) * ((by' : ℝ) - ay) := by
      rw [hlen]; push_cast; ring
    have hlen0 : (0 : ℝ) ≤ len := by
      rw [hlenR]; nlinarith [mul_self_nonneg ((bx : ℝ) - ax), mul_self_nonneg ((by' : ℝ) - ay)]
    -- the squared distance at parameter `t`, expanded
    have hexp : ((px : ℝ) - (ax + t * ((bx : ℝ) - ax))) ^ 2 + ((py : ℝ) - (ay + t * ((by' : ℝ) - ay))) ^ 2 =
        (((px : ℝ) - ax) * ((px : ℝ) - ax) + ((py : ℝ) - ay) * ((py : ℝ) - ay)) - 2 * t * dot +
          t ^ 2 * len := by
      rw [hdotR, hlenR]; ring
    rw [hexp] at hD
    by_cases hc1 : dot ≤ 0
    · left; left
      have hd : (dot : ℝ) ≤ 0 := by exact_mod_cast hc1
      have : (((px : ℝ) - ax) * ((px : ℝ) - ax) + ((py : ℝ) - ay) * ((py : ℝ) - ay)) ≤ 5 := by
        nlinarith [mul_nonneg ht0 (neg_nonneg.mpr hd), mul_nonneg (sq_nonneg t) hlen0]
      exact_mod_cast this
    · by_cases hc2 : len ≤ dot
      · left; right
        have hd : (len : ℝ) ≤ dot := by exact_mod_cast hc2
        have h1t : 0 ≤ 1 - t := by linarith
        have : (((px : ℝ) - bx) * ((px : ℝ) - bx) + ((py : ℝ) - by') * ((py : ℝ) - by')) ≤ 5 := by
          have e : (((px : ℝ) - bx) * ((px : ℝ) - bx) + ((py : ℝ) - by') * ((py : ℝ) - by')) =
              (((px : ℝ) - ax) * ((px : ℝ) - ax) + ((py : ℝ) - ay) * ((py : ℝ) - ay)) - 2 * dot + len := by
            rw [hdotR, hlenR]; ring
          rw [e]
          nlinarith [mul_nonneg h1t (sub_nonneg.mpr hd), mul_nonneg (mul_nonneg h1t h1t) hlen0]
        exact_mod_cast this
      · right
        refine ⟨⟨by omega, by omega⟩, ?_⟩
        have hlenpos : (0 : ℝ) < len := by
          have : (0 : ℤ) < len := by omega
          exact_mod_cast this
        have hid : ((((px : ℝ) - ax) * ((px : ℝ) - ax) + ((py : ℝ) - ay) * ((py : ℝ) - ay)) - 2 * t * dot +
            t ^ 2 * len) * len =
            (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) *
              (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) +
            (dot - t * len) ^ 2 := by
          rw [hdotR, hlenR]; ring
        have : (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) *
            (((bx : ℝ) - ax) * ((py : ℝ) - ay) - ((by' : ℝ) - ay) * ((px : ℝ) - ax)) ≤ 5 * (len : ℝ) := by
          nlinarith [sq_nonneg ((dot : ℝ) - t * len), mul_le_mul_of_nonneg_right hD hlenpos.le]
        rw [hlenR] at this
        exact_mod_cast this

/-- `nearBoundary` is `nearSegment` for some edge. -/
theorem nearBoundary_iff (vs : List Point) (p : Point) :
    nearBoundary vs p = true ↔ ∃ e ∈ edges vs, nearSegment p e.1 e.2 = true := by
  simp [nearBoundary]

/-- A vertex of the polygon is within the margin of its boundary, so it counts
as inside (unlike upstream, where it depends on the vertex, R10). -/
theorem nearSegment_left (a b : Point) : nearSegment a a b = true := by
  rw [nearSegment_iff]
  exact ⟨0, le_refl _, zero_le_one, by norm_num⟩

/-- **The margin covers the rounding (one edge).** Let `q` be a point of the
segment from `a` to `b` (real coordinates, millimetres). Let `a'`, `b'` be
integer points within `dv` of `a`, `b` on each axis, and `p'` an integer
point within `dp` of `q` on each axis, with `2·(dv + dp)² ≤ 5`. Then `p'` is
within the margin of the segment from `a'` to `b'`. -/
theorem nearSegment_of_close (p' a' b' : Point) (a b q : ℝ × ℝ) (t dv dp : ℝ)
    (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = a.1 + t * (b.1 - a.1)) (hqy : q.2 = a.2 + t * (b.2 - a.2))
    (hax : |a.1 - a'.x| ≤ dv) (hay : |a.2 - a'.y| ≤ dv)
    (hbx : |b.1 - b'.x| ≤ dv) (hby : |b.2 - b'.y| ≤ dv)
    (hpx : |q.1 - p'.x| ≤ dp) (hpy : |q.2 - p'.y| ≤ dp)
    (hm : 2 * (dv + dp) ^ 2 ≤ 5) : nearSegment p' a' b' = true := by
  rw [nearSegment_iff]
  refine ⟨t, ht0, ht1, ?_⟩
  rw [abs_le] at hax hay hbx hby hpx hpy
  have h1t : 0 ≤ 1 - t := by linarith
  have hx : |(p'.x : ℝ) - (a'.x + t * ((b'.x : ℝ) - a'.x))| ≤ dv + dp := by
    rw [abs_le]
    constructor <;>
      nlinarith [mul_le_mul_of_nonneg_left hax.1 h1t, mul_le_mul_of_nonneg_left hax.2 h1t,
        mul_le_mul_of_nonneg_left hbx.1 ht0, mul_le_mul_of_nonneg_left hbx.2 ht0]
  have hy : |(p'.y : ℝ) - (a'.y + t * ((b'.y : ℝ) - a'.y))| ≤ dv + dp := by
    rw [abs_le]
    constructor <;>
      nlinarith [mul_le_mul_of_nonneg_left hay.1 h1t, mul_le_mul_of_nonneg_left hay.2 h1t,
        mul_le_mul_of_nonneg_left hby.1 ht0, mul_le_mul_of_nonneg_left hby.2 ht0]
  have hx2 := sq_le_sq.mpr (le_trans hx (le_abs_self _))
  have hy2 := sq_le_sq.mpr (le_trans hy (le_abs_self _))
  linarith

/-- **The margin covers the rounding (whole polygon).** Let `vs` be the
vertices of a polygon in real coordinates (millimetres) and `rnd` any rounding
of them to integer points that moves each vertex by at most 1/2 on each axis.
Let `q` be a point of the boundary of `vs` (on the edge `e`, at parameter
`t`), and `p'` an integer point within 1 of `q` on each axis. Then the model
counts `p'` as inside the rounded polygon.

This is the quantitative half of the safe-direction argument of the SPEC:
1/2 per axis is the rounding of vertices to the nearest millimetre, 1 per
axis is the truncation of scan points, and `2·(1/2 + 1)² = 4.5 ≤ 5`.

The other half is **not proved**: that a point which upstream's rule (R10, in
exact arithmetic) puts inside the polygon `vs`, and whose truncation `p'` the
same rule puts outside the rounded polygon, has passed over the boundary on
the way. Precisely: moving every vertex and the point linearly from their
real positions to their rounded ones, there is a moment at which the moving
point lies on an edge of the moving polygon. That configuration satisfies the
hypotheses here (its vertices are still within 1/2, and its point within 1,
of the rounded ones on each axis), so this theorem then gives `contains`.
The missing step is the invariance of the crossing parity under a motion
that keeps the point off the boundary. -/
theorem nearBoundary_of_rounded (vs : List (ℝ × ℝ)) (rnd : ℝ × ℝ → Point)
    (hr : ∀ v ∈ vs, |v.1 - (rnd v).x| ≤ 1 / 2 ∧ |v.2 - (rnd v).y| ≤ 1 / 2)
    (e : (ℝ × ℝ) × (ℝ × ℝ)) (he : e ∈ cyclicPairs vs) (q : ℝ × ℝ) (t : ℝ)
    (ht0 : 0 ≤ t) (ht1 : t ≤ 1)
    (hqx : q.1 = e.1.1 + t * (e.2.1 - e.1.1)) (hqy : q.2 = e.1.2 + t * (e.2.2 - e.1.2))
    (p' : Point) (hpx : |q.1 - p'.x| ≤ 1) (hpy : |q.2 - p'.y| ≤ 1) :
    (Shape.polygon (vs.map rnd)).contains p' = true := by
  apply contains_of_nearBoundary
  rw [nearBoundary_iff, edges_eq_cyclicPairs, cyclicPairs_map]
  have hmem := mem_of_mem_cyclicPairs he
  refine ⟨Prod.map rnd rnd e, List.mem_map_of_mem he, ?_⟩
  exact nearSegment_of_close p' (rnd e.1) (rnd e.2) e.1 e.2 q t (1 / 2) 1 ht0 ht1 hqx hqy
    (hr _ hmem.1).1 (hr _ hmem.1).2 (hr _ hmem.2).1 (hr _ hmem.2).2 hpx hpy (by norm_num)

/-! ## Points far from every zone (D12) -/

/-- Crossings of a ray that starts to the left of both ends of an edge: the
edge is crossed iff it straddles the ray's height. -/
private theorem crosses_of_left (p a b : Point) (M : ℤ) (ha : M ≤ a.x) (hb : M ≤ b.x)
    (hp : p.x < M) :
    crosses p a b = (decide (p.y ≤ a.y) != decide (p.y ≤ b.y)) := by
  unfold crosses
  by_cases h1 : p.y ≤ a.y <;> by_cases h2 : p.y ≤ b.y
  · have : ¬ b.y < p.y := by omega
    simp [h1, h2, this]
  · have h3 : b.y < p.y := by omega
    have hd : ¬ a.y < b.y := by omega
    have key : (p.y - a.y) * (b.x - a.x) < (p.x - a.x) * (b.y - a.y) := by
      nlinarith [mul_nonneg (show (0 : ℤ) ≤ a.y - p.y by omega) (show (0 : ℤ) ≤ b.x - M by omega),
        mul_nonneg (show (0 : ℤ) ≤ p.y - b.y by omega) (show (0 : ℤ) ≤ a.x - M by omega),
        mul_pos (show (0 : ℤ) < a.y - b.y by omega) (show (0 : ℤ) < M - p.x by omega)]
    simp [h1, h2, h3, hd, key]
  · have h3 : ¬ b.y < p.y := by omega
    have hd : a.y < b.y := by omega
    suffices key : (p.x - a.x) * (b.y - a.y) < (p.y - a.y) * (b.x - a.x) by
      simp [h1, h2, h3, hd, key]
    nlinarith [mul_nonneg (show (0 : ℤ) ≤ p.y - a.y by omega) (show (0 : ℤ) ≤ b.x - M by omega),
      mul_nonneg (show (0 : ℤ) ≤ b.y - p.y by omega) (show (0 : ℤ) ≤ a.x - M by omega),
      mul_pos (show (0 : ℤ) < b.y - a.y by omega) (show (0 : ℤ) < M - p.x by omega)]
  · have : b.y < p.y := by omega
    simp [h1, h2, this]

/-- A ray that starts to the right of both ends of an edge does not cross
it. -/
private theorem crosses_of_right (p a b : Point) (M : ℤ) (ha : a.x ≤ M) (hb : b.x ≤ M)
    (hp : M < p.x) : crosses p a b = false := by
  unfold crosses
  by_cases h1 : p.y ≤ a.y <;> by_cases h2 : p.y ≤ b.y
  · have : ¬ b.y < p.y := by omega
    simp [h1, this]
  · have h3 : b.y < p.y := by omega
    have hd : ¬ a.y < b.y := by omega
    suffices key : (p.x - a.x) * (b.y - a.y) ≤ (p.y - a.y) * (b.x - a.x) by
      simp [h1, h3, hd, key]
    nlinarith [mul_nonneg (show (0 : ℤ) ≤ a.y - p.y by omega) (show (0 : ℤ) ≤ M - b.x by omega),
      mul_nonneg (show (0 : ℤ) ≤ p.y - b.y by omega) (show (0 : ℤ) ≤ M - a.x by omega),
      mul_pos (show (0 : ℤ) < a.y - b.y by omega) (show (0 : ℤ) < p.x - M by omega)]
  · have h3 : ¬ b.y < p.y := by omega
    have hd : a.y < b.y := by omega
    suffices key : (p.y - a.y) * (b.x - a.x) ≤ (p.x - a.x) * (b.y - a.y) by
      simp [h1, h3, hd, key]
    nlinarith [mul_nonneg (show (0 : ℤ) ≤ p.y - a.y by omega) (show (0 : ℤ) ≤ M - b.x by omega),
      mul_nonneg (show (0 : ℤ) ≤ b.y - p.y by omega) (show (0 : ℤ) ≤ M - a.x by omega),
      mul_pos (show (0 : ℤ) < b.y - a.y by omega) (show (0 : ℤ) < p.x - M by omega)]
  · have : b.y < p.y := by omega
    simp [h1, this]

/-- An edge wholly below the ray, or wholly at or above it, is not crossed. -/
private theorem crosses_of_same_side (p a b : Point)
    (h : (a.y < p.y ∧ b.y < p.y) ∨ (p.y ≤ a.y ∧ p.y ≤ b.y)) : crosses p a b = false := by
  unfold crosses
  rcases h with ⟨h1, h2⟩ | ⟨h1, h2⟩
  · have : ¬ p.y ≤ a.y := by omega
    simp [this, h2]
  · have : ¬ b.y < p.y := by omega
    simp [this, h1]

/-- Along a closed path the number of changes of a two-valued label is
even. -/
private theorem countP_changes {α : Type} (f : α → Bool) (a : α) (vs : List α) :
    ((a :: vs).zip vs).countP (fun e => f e.1 != f e.2) % 2 =
      if f a = f (vs.getLast?.getD a) then 0 else 1 := by
  induction vs generalizing a with
  | nil => simp
  | cons v vs ih =>
    rw [List.zip_cons_cons, List.countP_cons, List.getLast?_cons, Option.getD_some, Nat.add_mod,
      ih v]
    dsimp only
    cases f a <;> cases f v <;> cases f (vs.getLast?.getD v) <;> rfl

private theorem countP_changes_cyclic {α : Type} (f : α → Bool) (vs : List α) :
    (cyclicPairs vs).countP (fun e => f e.1 != f e.2) % 2 = 0 := by
  unfold cyclicPairs
  cases hl : vs.getLast? with
  | none => simp
  | some l =>
    have := countP_changes f l vs
    simpa [hl] using this

/-- If `p` is at least 3 mm beyond both ends of a segment along the x axis,
the foot test fails: the squared distance to the line is above the margin. -/
private theorem foot_far (u v w z : ℤ) (h1 : 3 ≤ w) (h2 : 3 ≤ w - u)
    (hd0 : 0 < u * w + v * z) (hd1 : u * w + v * z < u * u + v * v) :
    5 * (u * u + v * v) < (u * z - v * w) * (u * z - v * w) := by
  have hA : -(v * (u * z - v * w)) = w * (u * u + v * v) - (u * w + v * z) * u := by ring
  have hB : 3 * (u * u + v * v) ≤ w * (u * u + v * v) - (u * w + v * z) * u := by
    nlinarith [mul_le_mul_of_nonneg_right h1 (show (0 : ℤ) ≤ (u * u + v * v) - (u * w + v * z) by omega),
      mul_le_mul_of_nonneg_left h2 hd0.le]
  have hlen : 0 < u * u + v * v := by omega
  have hC : 3 * (u * u + v * v) ≤ -(v * (u * z - v * w)) := by rw [hA]; exact hB
  have hD : (3 * (u * u + v * v)) * (3 * (u * u + v * v)) ≤
      (v * (u * z - v * w)) * (v * (u * z - v * w)) := by
    nlinarith [mul_self_le_mul_self (by omega : (0 : ℤ) ≤ 3 * (u * u + v * v)) hC]
  have hE : (v * (u * z - v * w)) * (v * (u * z - v * w)) ≤
      (u * u + v * v) * ((u * z - v * w) * (u * z - v * w)) := by
    nlinarith [mul_nonneg (mul_self_nonneg u) (mul_self_nonneg (u * z - v * w))]
  by_contra hcon
  rw [not_lt] at hcon
  nlinarith [mul_le_mul_of_nonneg_left hcon hlen.le, mul_pos hlen hlen]

/-- A point at least 3 mm beyond both ends of a segment along one axis is not
within the margin of it. -/
private theorem nearSegment_of_beyond (p a b : Point)
    (h : (a.x + 3 ≤ p.x ∧ b.x + 3 ≤ p.x) ∨ (p.x + 3 ≤ a.x ∧ p.x + 3 ≤ b.x) ∨
      (a.y + 3 ≤ p.y ∧ b.y + 3 ≤ p.y) ∨ (p.y + 3 ≤ a.y ∧ p.y + 3 ≤ b.y)) :
    nearSegment p a b = false := by
  rw [Bool.eq_false_iff]
  intro hn
  simp only [nearSegment, Bool.or_eq_true, Bool.and_eq_true, decide_eq_true_eq] at hn
  simp only [polygonMarginSq] at hn
  rcases hn with (hn | hn) | ⟨⟨hd0, hd1⟩, hc⟩
  · rcases h with h | h | h | h
    · nlinarith [mul_self_nonneg (p.y - a.y), mul_self_nonneg (p.x - a.x - 3)]
    · nlinarith [mul_self_nonneg (p.y - a.y), mul_self_nonneg (a.x - p.x - 3)]
    · nlinarith [mul_self_nonneg (p.x - a.x), mul_self_nonneg (p.y - a.y - 3)]
    · nlinarith [mul_self_nonneg (p.x - a.x), mul_self_nonneg (a.y - p.y - 3)]
  · rcases h with h | h | h | h
    · nlinarith [mul_self_nonneg (p.y - b.y), mul_self_nonneg (p.x - b.x - 3)]
    · nlinarith [mul_self_nonneg (p.y - b.y), mul_self_nonneg (b.x - p.x - 3)]
    · nlinarith [mul_self_nonneg (p.x - b.x), mul_self_nonneg (p.y - b.y - 3)]
    · nlinarith [mul_self_nonneg (p.x - b.x), mul_self_nonneg (b.y - p.y - 3)]
  · rcases h with h | h | h | h
    · have := foot_far (b.x - a.x) (b.y - a.y) (p.x - a.x) (p.y - a.y) (by omega) (by omega)
        hd0 hd1
      omega
    · have := foot_far (-(b.x - a.x)) (b.y - a.y) (-(p.x - a.x)) (p.y - a.y) (by omega)
        (by omega) (by linarith) (by linarith)
      nlinarith
    · have := foot_far (b.y - a.y) (b.x - a.x) (p.y - a.y) (p.x - a.x) (by omega) (by omega)
        (by linarith) (by linarith)
      nlinarith
    · have := foot_far (-(b.y - a.y)) (b.x - a.x) (-(p.y - a.y)) (p.x - a.x) (by omega)
        (by omega) (by linarith) (by linarith)
      nlinarith

/-- **A polygon zone does not reach beyond its vertices by 3 mm.** A point
that is at least 3 mm beyond every vertex along one axis (to the right, to
the left, above or below) is not counted as inside: no odd number of
crossings, and not within the margin. The crossing rule and the margin
together therefore never claim a point outside the bounding box grown by
3 mm. -/
theorem contains_polygon_eq_false_of_beyond (vs : List Point) (p : Point)
    (h : (∀ v ∈ vs, v.x + 3 ≤ p.x) ∨ (∀ v ∈ vs, p.x + 3 ≤ v.x) ∨
      (∀ v ∈ vs, v.y + 3 ≤ p.y) ∨ (∀ v ∈ vs, p.y + 3 ≤ v.y)) :
    (Shape.polygon vs).contains p = false := by
  have hv : ∀ e ∈ edges vs, e.1 ∈ vs ∧ e.2 ∈ vs := fun e he =>
    mem_of_mem_cyclicPairs (edges_eq_cyclicPairs vs ▸ he)
  have hnear : nearBoundary vs p = false := by
    rw [Bool.eq_false_iff, Ne, nearBoundary_iff]
    rintro ⟨e, he, hn⟩
    have hb := hv e he
    rw [nearSegment_of_beyond] at hn
    · exact Bool.false_ne_true hn
    · rcases h with h | h | h | h
      · exact Or.inl ⟨h _ hb.1, h _ hb.2⟩
      · exact Or.inr (Or.inl ⟨h _ hb.1, h _ hb.2⟩)
      · exact Or.inr (Or.inr (Or.inl ⟨h _ hb.1, h _ hb.2⟩))
      · exact Or.inr (Or.inr (Or.inr ⟨h _ hb.1, h _ hb.2⟩))
  have hinside : insidePolygon vs p = false := by
    unfold insidePolygon
    rcases h with h | h | h | h
    · -- to the right of the polygon: no edge is crossed
      have : (edges vs).countP (fun e => crosses p e.1 e.2) = 0 := by
        rw [List.countP_eq_zero]
        intro e he
        have h1 := h _ (hv e he).1
        have h2 := h _ (hv e he).2
        rw [crosses_of_right p e.1 e.2 (p.x - 3) (by omega) (by omega) (by omega)]
        simp
      simp [this]
    · -- to the left: every edge that straddles the ray is crossed; their number is even
      have hcongr : (edges vs).countP (fun e => crosses p e.1 e.2) =
          (edges vs).countP (fun e => decide (p.y ≤ e.1.y) != decide (p.y ≤ e.2.y)) := by
        apply List.countP_congr
        intro e he
        have h1 := h _ (hv e he).1
        have h2 := h _ (hv e he).2
        rw [crosses_of_left p e.1 e.2 (p.x + 3) (by omega) (by omega) (by omega)]
      rw [hcongr, edges_eq_cyclicPairs,
        countP_changes_cyclic (fun v : Point => decide (p.y ≤ v.y)) vs]
      rfl
    · -- above the polygon
      have : (edges vs).countP (fun e => crosses p e.1 e.2) = 0 := by
        rw [List.countP_eq_zero]
        intro e he
        have h1 := h _ (hv e he).1
        have h2 := h _ (hv e he).2
        rw [crosses_of_same_side p e.1 e.2 (Or.inl ⟨by omega, by omega⟩)]
        simp
      simp [this]
    · -- below the polygon
      have : (edges vs).countP (fun e => crosses p e.1 e.2) = 0 := by
        rw [List.countP_eq_zero]
        intro e he
        have h1 := h _ (hv e he).1
        have h2 := h _ (hv e he).2
        rw [crosses_of_same_side p e.1 e.2 (Or.inr ⟨by omega, by omega⟩)]
        simp
      simp [this]
  simp [Shape.contains, hnear, hinside]

/-- **The crossing rule on a box.** For an axis-parallel box given by its
four corners in order, every point strictly inside is inside by upstream's
rule (exactly one edge is crossed), hence counted. Together with
`contains_polygon_eq_false_of_beyond` this fixes the sign conventions of
`crosses`: a rule with a flipped comparison fails one of the two. -/
theorem insidePolygon_box (x0 x1 y0 y1 : ℤ) (p : Point) (hx0 : x0 < p.x) (hx1 : p.x < x1)
    (hy0 : y0 < p.y) (hy1 : p.y < y1) :
    insidePolygon [⟨x1, y1⟩, ⟨x1, y0⟩, ⟨x0, y0⟩, ⟨x0, y1⟩] p = true := by
  have h1 : 0 < (p.x - x1) * (y0 - y1) := mul_pos_of_neg_of_neg (by omega) (by omega)
  have h2 : 0 < (p.x - x0) * (y1 - y0) := mul_pos (by omega) (by omega)
  have e1 : crosses p ⟨x0, y1⟩ ⟨x1, y1⟩ = false := crosses_of_horizontal _ _ _ rfl
  have e3 : crosses p ⟨x1, y0⟩ ⟨x0, y0⟩ = false := crosses_of_horizontal _ _ _ rfl
  have e2 : crosses p ⟨x1, y1⟩ ⟨x1, y0⟩ = true := by
    have a : p.y ≤ y1 := by omega
    have b : y0 ≤ y1 := by omega
    simp [crosses, a, hy0, b, h1]
  have e4 : crosses p ⟨x0, y0⟩ ⟨x0, y1⟩ = false := by
    have a : ¬ p.y ≤ y0 := by omega
    have b : ¬ y1 < p.y := by omega
    have c : ¬ y1 ≤ y0 := by omega
    simp [crosses, a, b, c, h2.le]
  simp [insidePolygon, edges, e1, e2, e3, e4]

/-- What `Shape.ok` requires. -/
theorem shape_ok_iff (s : Shape) :
    s.ok = true ↔
      match s with
      | .circle r => r ≤ 1000000
      | .polygon vs => 4 ≤ vs.length ∧
          ∀ v ∈ vs, -999997 ≤ v.x ∧ v.x ≤ 999997 ∧ -999997 ≤ v.y ∧ v.y ≤ 999997 := by
  cases s with
  | circle r =>
    simp only [Shape.ok, decide_eq_true_eq]
    simp only [coordLimit]
  | polygon vs =>
    simp only [Shape.ok, Point.inRange, Bool.and_eq_true, decide_eq_true_eq, List.all_eq_true]
    simp only [coordLimit]
    refine and_congr_right fun _ => forall_congr' fun v => forall_congr' fun _ => ?_
    omega

/-- What `Config.ok` requires of each zone: a fixed circle's radius is at most
1,000,000 mm; a fixed polygon has at least four vertices, each within
999,997 mm of both axes; a slowdown ratio has its sign bit clear (D5, D12,
R10). A topic zone has no shape in the configuration. -/
theorem config_ok_iff (cfg : Config) :
    cfg.ok = true ↔ ∀ z ∈ cfg.zones,
      z.shape.ok = true ∧ ∀ r, z.action = .slowdown r → r < 0x8000000000000000 := by
  simp only [Config.ok, List.all_eq_true, Zone.ok, Bool.and_eq_true]
  refine forall_congr' fun z => forall_congr' fun _ => and_congr_right fun _ => ?_
  cases z.action <;> simp [ZoneAction.ok]

/-- The shape lies within the coordinate limit: a circle's radius is at most
`coordLimit`, a polygon's vertices are at least 3 mm inside it. This is
`Shape.ok` without the count of vertices; the shape of a topic zone, which
may be a triangle (N33), satisfies it too. -/
def ShapeBounded (s : Shape) : Prop :=
  match s with
  | .circle r => r ≤ 1000000
  | .polygon vs => ∀ v ∈ vs, -999997 ≤ v.x ∧ v.x ≤ 999997 ∧ -999997 ≤ v.y ∧ v.y ≤ 999997

theorem shapeBounded_of_ok (s : Shape) (hok : s.ok = true) : ShapeBounded s := by
  rw [shape_ok_iff] at hok
  cases s with
  | circle r => exact hok
  | polygon vs => exact hok.2

/-- Vertices that pass the check of `TopicShape.ofMessage` bound a polygon. -/
theorem shapeBounded_polygon_of_all (vs : List Point)
    (h : vs.all (·.inRange (coordLimit - 3)) = true) : ShapeBounded (.polygon vs) := by
  intro v hv
  have := List.all_eq_true.mp h v hv
  simp only [Point.inRange, coordLimit, Bool.and_eq_true, Nat.reduceSub] at this
  have h1 := of_decide_eq_true this.1
  have h2 := of_decide_eq_true this.2
  omega

/-- **A saturated point is in no zone (D12).** If a shape lies within the
coordinate limit (`ShapeBounded`), a point with a coordinate of magnitude
`coordLimit` (1000 m) or more is not inside it: neither by the circle test,
nor by the crossing rule, nor by the margin. The model gives such a point for
a reading that is NaN, infinite or 1000 m or more away. -/
theorem contains_eq_false_of_far' (s : Shape) (p : Point) (hok : ShapeBounded s)
    (hfar : coordLimit ≤ p.x.natAbs ∨ coordLimit ≤ p.y.natAbs) : s.contains p = false := by
  simp only [coordLimit] at hfar
  cases s with
  | circle r =>
    simp only [ShapeBounded] at hok
    simp only [Shape.contains, decide_eq_false_iff_not, not_lt]
    have hr : ((r * r : Nat) : ℤ) ≤ 1000000 * 1000000 := by
      have : r * r ≤ 1000000 * 1000000 := Nat.mul_le_mul hok hok
      exact_mod_cast this
    rcases hfar with h | h
    · have : 1000000 * 1000000 ≤ p.x * p.x := by
        rcases (show 1000000 ≤ p.x ∨ p.x ≤ -1000000 by omega) with h | h <;> nlinarith
      nlinarith [mul_self_nonneg p.y]
    · have : 1000000 * 1000000 ≤ p.y * p.y := by
        rcases (show 1000000 ≤ p.y ∨ p.y ≤ -1000000 by omega) with h | h <;> nlinarith
      nlinarith [mul_self_nonneg p.x]
  | polygon vs =>
    simp only [ShapeBounded] at hok
    apply contains_polygon_eq_false_of_beyond
    rcases (show 1000000 ≤ p.x ∨ p.x ≤ -1000000 ∨ 1000000 ≤ p.y ∨ p.y ≤ -1000000 by omega)
      with h | h | h | h
    · refine Or.inl fun v hv => ?_
      have := hok v hv
      omega
    · refine Or.inr (Or.inl fun v hv => ?_)
      have := hok v hv
      omega
    · refine Or.inr (Or.inr (Or.inl fun v hv => ?_))
      have := hok v hv
      omega
    · refine Or.inr (Or.inr (Or.inr fun v hv => ?_))
      have := hok v hv
      omega

/-- The same for a shape that passes `Shape.ok`. -/
theorem contains_eq_false_of_far (s : Shape) (p : Point) (hok : s.ok = true)
    (hfar : coordLimit ≤ p.x.natAbs ∨ coordLimit ≤ p.y.natAbs) : s.contains p = false :=
  contains_eq_false_of_far' s p (shapeBounded_of_ok s hok) hfar

/-- Read the other way: a point inside a shape within the coordinate limit
has both coordinates strictly within the limit. -/
theorem inRange_of_contains' (s : Shape) (p : Point) (hok : ShapeBounded s)
    (h : s.contains p = true) : p.inRange (coordLimit - 1) = true := by
  by_contra hn
  have hfar : coordLimit ≤ p.x.natAbs ∨ coordLimit ≤ p.y.natAbs := by
    simp only [Point.inRange, Bool.and_eq_true, decide_eq_true_eq] at hn
    simp only [coordLimit] at hn ⊢
    omega
  rw [contains_eq_false_of_far' s p hok hfar] at h
  exact Bool.false_ne_true h

theorem inRange_of_contains (s : Shape) (p : Point) (hok : s.ok = true)
    (h : s.contains p = true) : p.inRange (coordLimit - 1) = true :=
  inRange_of_contains' s p (shapeBounded_of_ok s hok) h

/-- Every zone that has a shape has one within the coordinate limit. -/
def ZonesBounded (cfg : Config) : Prop :=
  ∀ z ∈ cfg.zones, ∀ s, z.shape = .fixed s → ShapeBounded s

theorem zonesBounded_of_ok (cfg : Config) (hok : cfg.ok = true) : ZonesBounded cfg := by
  intro z hz s hs
  have := ((config_ok_iff cfg).mp hok z hz).1
  rw [hs] at this
  exact shapeBounded_of_ok s this

/-- **Saturated points do not change the action.** If every zone lies within
the coordinate limit, the zones answer as if the points with a coordinate of
magnitude `coordLimit` or more had been left out. -/
theorem zonesAction_filter_inRange' (cfg : Config) (hok : ZonesBounded cfg) (pts : List Point) :
    zonesAction cfg (pts.filter (·.inRange (coordLimit - 1))) = zonesAction cfg pts := by
  apply zonesAction_congr
  intro z hz
  have hc : (pts.filter (·.inRange (coordLimit - 1))).countP z.shape.contains =
      pts.countP z.shape.contains := by
    rw [List.countP_filter]
    apply List.countP_congr
    intro p _
    simp only [Bool.and_eq_true, and_iff_left_iff_imp]
    cases hs : z.shape with
    | topic => simp [ZoneShape.contains]
    | fixed s => exact inRange_of_contains' s p (hok z hz s hs)
  unfold Zone.triggered Zone.count
  rw [hc]

/-- With an accepted configuration, saturated points do not change the
action. (For the zones as they stand at a command, topic zones included:
`liveConfig_bounded` in `Lifecycle.lean`.) -/
theorem zonesAction_filter_inRange (cfg : Config) (hok : cfg.ok = true) (pts : List Point) :
    zonesAction cfg (pts.filter (·.inRange (coordLimit - 1))) = zonesAction cfg pts :=
  zonesAction_filter_inRange' cfg (zonesBounded_of_ok cfg hok) pts

end ROSlean.CollisionMonitor
