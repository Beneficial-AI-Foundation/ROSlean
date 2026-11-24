/-!
# Collision monitor: bit patterns, points and transforms

The lowest layer of the collision monitor model
(`docs/collision_monitor/SPEC_NAV2.md`, sections 3.1 and 5; rule numbers
`N…` and differences `ND…` refer to that file):

* how a binary64 or binary32 bit pattern is read as a number (sign, exponent
  field, significand);
* a point of the base frame in whole millimetres, and the coordinate limit;
* `Xform`, a transform as tf2 hands it over (a quaternion and the x and y of
  a translation, six binary64 patterns), and its application to a point of a
  scan (`Xform.pointMm`) and to a vertex of a zone (`Xform.vertexMm`).

The transform is applied **exactly**: every pattern is read as the rational
number it denotes, the image is computed in integer arithmetic on dyadic
numbers (`Dy`), and only the final conversion to millimetres rounds. There is
no `Float` here and no Mathlib. Theorems are in
`ROSlean/CollisionMonitor/Transform.lean`.
-/

namespace ROSleanCore.CollisionMonitorFFI

/-! ## Decoding bit patterns -/

namespace F64

/-- The sign bit. -/
def neg (b : Nat) : Bool := b / 0x8000000000000000 % 2 == 1

/-- The exponent field, 0 to 2047. -/
def expField (b : Nat) : Nat := b / 0x10000000000000 % 0x800

/-- The fraction field. -/
def frac (b : Nat) : Nat := b % 0x10000000000000

/-- Neither infinite nor NaN. -/
def finite (b : Nat) : Bool := expField b != 0x7FF

def isNaN (b : Nat) : Bool := expField b == 0x7FF && frac b != 0

/-- The significand as an integer: with the implicit leading bit for a normal
number, without it for a subnormal one. -/
def mant (b : Nat) : Nat := if expField b = 0 then frac b else frac b + 0x10000000000000

/-- For a finite pattern the magnitude is `mant b * 2 ^ shift b / 2 ^ 1075`. -/
def shift (b : Nat) : Nat := if expField b = 0 then 1 else expField b

/-- The denominator's exponent in the formula of `shift`. -/
def bias : Nat := 1075

end F64

namespace F32

def neg (b : Nat) : Bool := b / 0x80000000 % 2 == 1

def expField (b : Nat) : Nat := b / 0x800000 % 0x100

def frac (b : Nat) : Nat := b % 0x800000

def finite (b : Nat) : Bool := expField b != 0xFF

def mant (b : Nat) : Nat := if expField b = 0 then frac b else frac b + 0x800000

/-- For a finite pattern the magnitude is `mant b * 2 ^ shift b / 2 ^ 150`. -/
def shift (b : Nat) : Nat := if expField b = 0 then 1 else expField b

def bias : Nat := 150

end F32

end ROSleanCore.CollisionMonitorFFI

namespace ROSleanCore.CollisionMonitor

/-! ## Points -/

/-- A point in the base frame, in millimetres. -/
structure Point where
  x : Int
  y : Int
  deriving Repr, DecidableEq

/-- Scan coordinates are saturated to `±coordLimit` mm. `Config.ok` keeps
every zone strictly inside, so that a saturated point is in no zone. -/
def coordLimit : Nat := 1000000

/-- Both coordinates are within `±limit`. -/
def Point.inRange (p : Point) (limit : Nat) : Bool :=
  decide (p.x.natAbs ≤ limit) && decide (p.y.natAbs ≤ limit)

/-- The point given for a reading whose coordinates are not finite, or under
a transform that is not usable: it lies in no accepted zone. -/
def Point.far : Point := ⟨coordLimit, coordLimit⟩

/-! ## Dyadic numbers

`⟨m, e⟩` is the number `m · 2 ^ e`. Sums align the two exponents; a zero is
never aligned, so that a zero component of a quaternion (exponent −1074)
does not shift the other term by a thousand bits. -/

structure Dy where
  m : Int
  e : Int
  deriving Repr, DecidableEq

namespace Dy

/-- The number a finite binary64 pattern denotes. -/
def ofF64 (b : Nat) : Dy :=
  ⟨if CollisionMonitorFFI.F64.neg b then -(CollisionMonitorFFI.F64.mant b : Int)
    else (CollisionMonitorFFI.F64.mant b : Int),
   (CollisionMonitorFFI.F64.shift b : Int) - 1075⟩

/-- The number a finite binary32 pattern denotes. -/
def ofF32 (b : Nat) : Dy :=
  ⟨if CollisionMonitorFFI.F32.neg b then -(CollisionMonitorFFI.F32.mant b : Int)
    else (CollisionMonitorFFI.F32.mant b : Int),
   (CollisionMonitorFFI.F32.shift b : Int) - 150⟩

def mul (a b : Dy) : Dy := ⟨a.m * b.m, a.e + b.e⟩

def neg (a : Dy) : Dy := ⟨-a.m, a.e⟩

/-- Twice the number. -/
def double (a : Dy) : Dy := ⟨a.m, a.e + 1⟩

def add (a b : Dy) : Dy :=
  if a.m = 0 then b
  else if b.m = 0 then a
  else if a.e ≤ b.e then ⟨a.m + b.m * 2 ^ (b.e - a.e).toNat, a.e⟩
  else ⟨a.m * 2 ^ (a.e - b.e).toNat + b.m, b.e⟩

def sub (a b : Dy) : Dy := a.add b.neg

/-- `num / den · 1000` as a quotient `(N, D)` of an integer by a natural
number; `D` is positive if `den.m` is. A zero numerator is not aligned. -/
def scaled (num den : Dy) : Int × Nat :=
  if num.m = 0 then (0, den.m.toNat)
  else if den.e ≤ num.e then (num.m * 1000 * 2 ^ (num.e - den.e).toNat, den.m.toNat)
  else (num.m * 1000, den.m.toNat * 2 ^ (den.e - num.e).toNat)

end Dy

/-- The quotient truncated toward zero and saturated at `±coordLimit`: a scan
point never moves away from the origin. -/
def truncMm (q : Int × Nat) : Int :=
  let mag : Nat := min coordLimit (q.1.natAbs / q.2)
  if q.1 < 0 then -(mag : Int) else (mag : Int)

/-- The quotient rounded to the nearest integer, a half away from zero. -/
def roundMm (q : Int × Nat) : Int :=
  let mag : Nat := (2 * q.1.natAbs + q.2) / (2 * q.2)
  if q.1 < 0 then -(mag : Int) else (mag : Int)

/-! ## Transforms (N25, ND1) -/

/-- A looked-up transform: the quaternion `(qx, qy, qz, qw)` and the x and y
of the translation, in metres, as binary64 bit patterns. The z of the
translation is not needed: the image is projected on the base frame's x–y
plane (N25). -/
structure Xform where
  qx : Nat
  qy : Nat
  qz : Nat
  qw : Nat
  ox : Nat
  oy : Nat
  deriving Repr, DecidableEq

namespace Xform

/-- The identity: quaternion (0, 0, 0, 1), translation (0, 0). -/
def identity : Xform := ⟨0, 0, 0, 0x3FF0000000000000, 0, 0⟩

/-- All six numbers are finite. -/
def finite (t : Xform) : Bool :=
  CollisionMonitorFFI.F64.finite t.qx && CollisionMonitorFFI.F64.finite t.qy &&
  CollisionMonitorFFI.F64.finite t.qz && CollisionMonitorFFI.F64.finite t.qw &&
  CollisionMonitorFFI.F64.finite t.ox && CollisionMonitorFFI.F64.finite t.oy

/-- `d = qx² + qy² + qz² + qw²`, exactly. -/
def den (t : Xform) : Dy :=
  let qx := Dy.ofF64 t.qx
  let qy := Dy.ofF64 t.qy
  let qz := Dy.ofF64 t.qz
  let qw := Dy.ofF64 t.qw
  ((qx.mul qx).add (qy.mul qy)).add ((qz.mul qz).add (qw.mul qw))

/-- The transform can be applied: its numbers are finite and the quaternion
is not zero (ND7). tf2 never returns one that is not (N25). -/
def usable (t : Xform) : Bool := t.finite && decide (0 < t.den.m)

end Xform

/-- The first two rows of the rotation matrix of a quaternion, times
`d = |q|²`, and the translation times `d`, as exact numbers:
the image of `(x, y, 0)` is `((n00·x + n01·y + tx) / d, (n10·x + n11·y + ty) / d)`.
Computed once per transform. -/
structure Mat where
  n00 : Dy
  n01 : Dy
  n10 : Dy
  n11 : Dy
  tx : Dy
  ty : Dy
  d : Dy

/-- N25, with the division by `d` postponed: `n00 = d − 2(qy² + qz²)`,
`n01 = 2(qx·qy − qw·qz)`, `n10 = 2(qx·qy + qw·qz)`, `n11 = d − 2(qx² + qz²)`. -/
def Xform.mat (t : Xform) : Mat :=
  let qx := Dy.ofF64 t.qx
  let qy := Dy.ofF64 t.qy
  let qz := Dy.ofF64 t.qz
  let qw := Dy.ofF64 t.qw
  let d := t.den
  { n00 := d.sub ((qy.mul qy).add (qz.mul qz)).double
    n01 := ((qx.mul qy).sub (qw.mul qz)).double
    n10 := ((qx.mul qy).add (qw.mul qz)).double
    n11 := d.sub ((qx.mul qx).add (qz.mul qz)).double
    tx := (Dy.ofF64 t.ox).mul d
    ty := (Dy.ofF64 t.oy).mul d
    d := d }

namespace Mat

/-- The x of the image of `(x, y)`, times `d`. -/
def numX (m : Mat) (x y : Dy) : Dy := ((m.n00.mul x).add (m.n01.mul y)).add m.tx

/-- The y of the image of `(x, y)`, times `d`. -/
def numY (m : Mat) (x y : Dy) : Dy := ((m.n10.mul x).add (m.n11.mul y)).add m.ty

/-- The image of the scan-frame point with binary64 coordinates `x`, `y`
(metres), in millimetres, each coordinate truncated toward zero and
saturated; `Point.far` if a coordinate is not finite. -/
def pointMm (m : Mat) (x y : Nat) : Point :=
  if CollisionMonitorFFI.F64.finite x && CollisionMonitorFFI.F64.finite y then
    ⟨truncMm ((m.numX (Dy.ofF64 x) (Dy.ofF64 y)).scaled m.d),
     truncMm ((m.numY (Dy.ofF64 x) (Dy.ofF64 y)).scaled m.d)⟩
  else Point.far

/-- The image of the vertex with binary32 coordinates `x`, `y` (metres), in
millimetres, each coordinate rounded to the nearest integer; `Point.far` if a
coordinate is not finite. Not saturated: a vertex beyond the coordinate limit
is refused by whoever uses it. -/
def vertexMm (m : Mat) (x y : Nat) : Point :=
  if CollisionMonitorFFI.F32.finite x && CollisionMonitorFFI.F32.finite y then
    ⟨roundMm ((m.numX (Dy.ofF32 x) (Dy.ofF32 y)).scaled m.d),
     roundMm ((m.numY (Dy.ofF32 x) (Dy.ofF32 y)).scaled m.d)⟩
  else Point.far

end Mat

/-- **The point of a scan reading in the base frame (N25, ND1).** For a
usable transform and finite coordinates: the exact image of `(x, y, 0)` under
the rotation the quaternion denotes (that of its normalisation, if it is not
of unit length) followed by the translation, projected on the x–y plane, in
millimetres, truncated toward zero and saturated at `±coordLimit`. Otherwise
`Point.far`. -/
def Xform.pointMm (t : Xform) (x y : Nat) : Point :=
  if t.usable then t.mat.pointMm x y else Point.far

/-- **The vertex of a zone in the base frame.** As `Xform.pointMm`, for
binary32 coordinates and with rounding to the nearest millimetre, a half away
from zero. -/
def Xform.vertexMm (t : Xform) (x y : Nat) : Point :=
  if t.usable then t.mat.vertexMm x y else Point.far

end ROSleanCore.CollisionMonitor
