/-!
# Collision monitor: bit patterns and points

The lowest layer of the collision monitor model
(`docs/collision_monitor/SPEC_NAV2.md`, sections 3.1 and 5):

* how a binary64 or binary32 bit pattern is read as a number (sign, exponent
  field, significand);
* a point of the base frame in whole millimetres, and the coordinate limit;
* the two conversions the model itself applies to coordinates that arrive
  with an event, already **in the base frame** (the sensor-to-base transform
  is applied by the C++ shim in `double`, as upstream applies it: option A
  of SPEC_NAV2, section 5):
  `coordMm` for a point of a scan (truncated toward zero, saturated) and
  `vertexCoordMm` for a vertex of a zone that arrives on a topic (rounded to
  the nearest millimetre).

There is no `Float` here and no Mathlib. Theorems are in
`ROSlean/CollisionMonitor/Convert.lean`.
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

end ROSleanCore.CollisionMonitor

namespace ROSleanCore.CollisionMonitorFFI

open ROSleanCore.CollisionMonitor

/-! ## Scan coordinates -/

/-- The magnitude of a coordinate in whole millimetres, rounded down, given
the pattern without its sign bit: `⌊|v| · 1000⌋`, or `coordLimit` if that is
`coordLimit` or more, or if the value is infinite or NaN.

A value of 1024 m or more saturates without any arithmetic; below that the
product `mant * 1000` is shifted right by at least 43 bits, and both fit a
machine word. -/
def coordMagMm (mag : Nat) : Nat :=
  if 1033 ≤ F64.expField mag then coordLimit
  else min coordLimit ((F64.mant mag * 1000) >>> (F64.bias - F64.shift mag))

/-- A coordinate in metres, given as sign and pattern without the sign bit, in
whole millimetres: truncated toward zero, so that a point never moves away
from the origin, and saturated at `±coordLimit`. NaN, of either sign, is
`+coordLimit` (SPEC, "Far and non-finite points"). -/
def coordMmOfParts (neg : Bool) (mag : Nat) : Int :=
  if neg && !F64.isNaN mag then -(coordMagMm mag : Int) else (coordMagMm mag : Int)

/-- `coordMmOfParts` for a whole binary64 pattern. -/
def coordMm (b : Nat) : Int := coordMmOfParts (F64.neg b) (b % 0x8000000000000000)

/-! ## Vertices that arrive with an event -/

/-- The magnitude of a vertex coordinate in whole millimetres, rounded to the
nearest, a half up: `⌊|v| · 1000 + 1/2⌋`. For a value of 1024 m or more, an
infinity or a NaN it is `coordLimit`, which no shape may reach (`Shape.ok`
and `placeVerts` demand 3 mm less), so such a vertex is always refused.

Below 1024 m the product `mant * 1000` is shifted right by at least 42 bits,
one bit short of the quotient; adding 1 and halving rounds it. Everything
fits a machine word. -/
def vertexMagMm (b : Nat) : Nat :=
  if 1033 ≤ F64.expField b then coordLimit
  else (((F64.mant b * 1000) >>> (F64.bias - F64.shift b - 1)) + 1) / 2

/-- A vertex coordinate in metres (binary64 pattern, base frame, as the shim
computed it) in whole millimetres: rounded to the nearest, a half away from
zero. `±coordLimit` if the value is 1024 m or more or not finite. -/
def vertexCoordMm (b : Nat) : Int :=
  if F64.neg b then -(vertexMagMm b : Int) else (vertexMagMm b : Int)

end ROSleanCore.CollisionMonitorFFI

namespace ROSleanCore.CollisionMonitor

/-- The point of a scan reading from its base-frame coordinates in metres
(binary64 patterns): each coordinate truncated toward zero and saturated
(`coordMm`). -/
def scanPointMm (c : Nat × Nat) : Point :=
  ⟨CollisionMonitorFFI.coordMm c.1, CollisionMonitorFFI.coordMm c.2⟩

/-- The vertex of a topic zone from its base-frame coordinates in metres
(binary64 patterns): each coordinate rounded to the nearest millimetre
(`vertexCoordMm`). -/
def vertexPointMm (c : Nat × Nat) : Point :=
  ⟨CollisionMonitorFFI.vertexCoordMm c.1, CollisionMonitorFFI.vertexCoordMm c.2⟩

end ROSleanCore.CollisionMonitor
