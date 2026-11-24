/-! T04 prototype: the cost of applying a rigid transform, given as binary64
bit patterns, to scan points in exact arithmetic (option B of
`docs/collision_monitor/SPEC_NAV2.md`, "Where the transform is applied").

Not part of any build and not the definition T05 has to use: it exists to
measure. No Mathlib. Run with `run_transform_cost.sh`. -/

/-- A dyadic rational `m * 2^e`. -/
structure Dy where
  m : Int
  e : Int

/-- `2^k`, by a shift. -/
@[inline] def pow2 (k : Nat) : Int := ((1 <<< k : Nat) : Int)

namespace Dy
@[inline] def mul (a b : Dy) : Dy := ⟨a.m * b.m, a.e + b.e⟩
@[inline] def add (a b : Dy) : Dy :=
  if a.e ≤ b.e then ⟨a.m + b.m * pow2 (b.e - a.e).toNat, a.e⟩
  else ⟨a.m * pow2 (a.e - b.e).toNat + b.m, b.e⟩
@[inline] def neg (a : Dy) : Dy := ⟨-a.m, a.e⟩
@[inline] def sub (a b : Dy) : Dy := add a (neg b)
@[inline] def dbl (a : Dy) : Dy := ⟨a.m, a.e + 1⟩
end Dy

/-- The number a finite binary64 pattern denotes. -/
def dyOfBits (b : UInt64) : Dy :=
  let frac := (b &&& 0xFFFFFFFFFFFFF).toNat
  let ex := ((b >>> 52) &&& 0x7FF).toNat
  let mag : Nat := if ex == 0 then frac else frac + 0x10000000000000
  let e : Int := (if ex == 0 then 1 else (ex : Int)) - 1075
  let m : Int := if (b >>> 63) == 1 then -(mag : Int) else (mag : Int)
  ⟨m, e⟩

/-- The first two rows of the rotation matrix of a quaternion, times
`d = |q|²`, and the translation times `d`. With these the image of a point
is `(n00·x + n01·y + tx) / d`, `(n10·x + n11·y + ty) / d`: upstream's
`tf2::Matrix3x3::setRotation` and `tf2::Transform::operator*` for a point with
`z = 0`, without rounding. -/
structure Xf where
  n00 : Dy
  n01 : Dy
  n10 : Dy
  n11 : Dy
  tx : Dy
  ty : Dy
  d : Dy

def Xf.ofBits (qx qy qz qw ox oy : UInt64) : Xf :=
  let x := dyOfBits qx
  let y := dyOfBits qy
  let z := dyOfBits qz
  let w := dyOfBits qw
  let xx := x.mul x
  let yy := y.mul y
  let zz := z.mul z
  let ww := w.mul w
  let d := (xx.add yy).add (zz.add ww)
  { n00 := d.sub (yy.add zz).dbl
    n01 := ((x.mul y).sub (w.mul z)).dbl
    n10 := ((x.mul y).add (w.mul z)).dbl
    n11 := d.sub (xx.add zz).dbl
    tx := (dyOfBits ox).mul d
    ty := (dyOfBits oy).mul d
    d := d }

/-- `1000 · n / d` truncated toward zero: metres to whole millimetres. -/
@[inline] def mmOf (n d : Dy) : Int :=
  let k := n.e - d.e
  if 0 ≤ k then Int.tdiv (1000 * n.m * pow2 k.toNat) d.m
  else Int.tdiv (1000 * n.m) (d.m * pow2 (-k).toNat)

@[inline] def Xf.apply (t : Xf) (px py : UInt64) : Int × Int :=
  let x := dyOfBits px
  let y := dyOfBits py
  let nx := ((t.n00.mul x).add (t.n01.mul y)).add t.tx
  let ny := ((t.n10.mul x).add (t.n11.mul y)).add t.ty
  (mmOf nx t.d, mmOf ny t.d)

/-! ## A fixed-point variant

The matrix is rounded once per transform to integers at scale `2^-30`, the
translation and each coordinate to units of `2^-20` m (about a micrometre),
each exactly from the number the pattern denotes. Per point there remain two
integer multiply-adds and a division by a constant. Its distance from the
exact image is bounded in SPEC_NAV2.md. -/

/-- `n / d · 2^s` rounded to the nearest integer, a half away from zero. -/
def scaleRound (n d : Dy) (s : Nat) : Int :=
  let k := n.e - d.e + (s : Int)
  let num := if 0 ≤ k then n.m * pow2 k.toNat else n.m
  let den := if 0 ≤ k then d.m else d.m * pow2 (-k).toNat
  let q := Int.tdiv (2 * num.natAbs + den.natAbs) (2 * den.natAbs)
  if (num < 0) != (den < 0) then -q else q

/-- A coordinate in units of `2^-20` m, truncated toward zero. -/
@[inline] def unitsOfBits (b : UInt64) : Int :=
  let v := dyOfBits b
  let k := v.e + 20
  if 0 ≤ k then v.m * pow2 k.toNat else Int.tdiv v.m (pow2 (-k).toNat)

structure Fx where
  m00 : Int
  m01 : Int
  m10 : Int
  m11 : Int
  tx : Int
  ty : Int

def Fx.ofXf (t : Xf) : Fx :=
  { m00 := scaleRound t.n00 t.d 30, m01 := scaleRound t.n01 t.d 30
    m10 := scaleRound t.n10 t.d 30, m11 := scaleRound t.n11 t.d 30
    tx := scaleRound t.tx t.d 20, ty := scaleRound t.ty t.d 20 }

/-- Millimetres, truncated toward zero, of a point given in units. -/
@[inline] def Fx.apply (t : Fx) (px py : Int) : Int × Int :=
  let x := Int.tdiv (t.m00 * px + t.m01 * py) 1073741824 + t.tx
  let y := Int.tdiv (t.m10 * px + t.m11 * py) 1073741824 + t.ty
  (Int.tdiv (1000 * x) 1048576, Int.tdiv (1000 * y) 1048576)

def main (args : List String) : IO Unit := do
  let n := 1000
  let reps := (args.head? >>= String.toNat?).getD 2000
  -- Points as the shim hands them over today: float products widened to double.
  let mut pts : Array (UInt64 × UInt64) := #[]
  for i in [0:n] do
    let a : Float32 := (-3.14159 + 0.00628 * i.toFloat).toFloat32
    let r : Float32 := (0.3 + 0.0077 * (i % 400).toFloat).toFloat32
    pts := pts.push ((r * a.cos).toFloat.toBits, (r * a.sin).toFloat.toBits)
  let half : Float := 0.3 / 2
  let o : Float := 0.0
  let yaw := Xf.ofBits o.toBits o.toBits half.sin.toBits half.cos.toBits
    (0.2 : Float).toBits (0.1 : Float).toBits
  -- A quaternion with all four components non-zero and not of unit length.
  let tilted := Xf.ofBits (0.01 : Float).toBits (0.02 : Float).toBits half.sin.toBits
    half.cos.toBits (0.2 : Float).toBits (0.1 : Float).toBits
  for (name, tr) in [("yaw and translation", yaw), ("general quaternion", tilted)] do
    let t0 ← IO.monoNanosNow
    let mut acc : Int := 0
    for _ in [0:reps] do
      for p in pts do
        let (x, y) := tr.apply p.1 p.2
        acc := acc + x + y
    let t1 ← IO.monoNanosNow
    IO.println s!"{name}: {(t1 - t0) / (reps * n)} ns per point ({reps} x {n} points), checksum {acc}"
  -- Fixed point: coordinates converted once per scan, the matrix once per command.
  let upts := pts.map fun p => (unitsOfBits p.1, unitsOfBits p.2)
  let mut worst : Nat := 0
  for (name, tr) in [("yaw and translation", yaw), ("general quaternion", tilted)] do
    let fx := Fx.ofXf tr
    let t0 ← IO.monoNanosNow
    let mut acc : Int := 0
    for _ in [0:reps] do
      for p in upts do
        let (x, y) := fx.apply p.1 p.2
        acc := acc + x + y
    let t1 ← IO.monoNanosNow
    IO.println s!"fixed point, {name}: {(t1 - t0) / (reps * n)} ns per point, checksum {acc}"
    for i in [0:n] do
      let (x, y) := tr.apply pts[i]!.1 pts[i]!.2
      let (fxx, fxy) := fx.apply upts[i]!.1 upts[i]!.2
      worst := max worst (max (x - fxx).natAbs (y - fxy).natAbs)
  IO.println s!"largest difference between the exact and the fixed-point result: {worst} mm"
  let (x, y) := yaw.apply (1.0 : Float).toBits o.toBits
  IO.println s!"(1, 0) m under yaw 0.3 rad and translation (0.2, 0.1) m: ({x}, {y}) mm; by hand (1155, 395)"
