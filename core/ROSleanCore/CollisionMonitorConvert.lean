import ROSleanCore.CollisionMonitor

/-!
# Conversions at the boundary of the collision monitor

The ROS shim hands over parameter values and scan coordinates as IEEE-754 bit
patterns, exactly as it received them. Everything that turns such a value into
one of the model's integers is here, in exact integer arithmetic: there is no
`Float` in this file.

| Value | Function | Rule |
| --- | --- | --- |
| scan coordinate, `double` metres, base frame | `coordMm` | millimetres truncated toward zero, saturated at ±`coordLimit`; NaN is `+coordLimit` |
| vertex coordinate of a topic zone, `double` metres, base frame | `vertexCoordMm` | millimetres rounded to nearest; ±`coordLimit` (always refused) if not finite or 1024 m or more |
| circle radius, `double` metres | `radiusMm` | magnitude in millimetres rounded up, plus 1; refused if not finite or 1000 m or more |
| polygon vertex coordinate, `float` metres | `vertexMm` | millimetres rounded to nearest; refused if not finite |
| timeout, `double` seconds | `timeoutNs` | nanoseconds as `rclcpp::Duration::from_seconds` computes them; refused if negative, not finite, or 9.2·10⁹ s or more |
| `min_points`, 64-bit integer | `minPointsOfRaw` | negative is 0; refused outside the 32-bit `int` range |

"Refused" is `none`: the builder then marks the configuration as not accepted
(`ROSleanCore.CollisionMonitorFFI`).

`coordMm` and `vertexCoordMm` are applied by the model itself, to the
coordinates a command or a shape message carries (`scanPointMm`,
`vertexPointMm`), so they are defined below the model, in
`ROSleanCore.CollisionMonitorPoint`, with the reading of a bit pattern. The
others are applied when the configuration is built and are defined here.

A finite binary64 value with pattern `b` has magnitude
`F64.mant b * 2 ^ F64.shift b / 2 ^ 1075`, and a finite binary32 value has
magnitude `F32.mant b * 2 ^ F32.shift b / 2 ^ 150`. The first three conversions
are exact functions of that rational number. `timeoutNs` instead reproduces a
floating-point computation of upstream (one rounded multiplication), so that
the node's timeouts are upstream's to the nanosecond.

Theorems about these functions are in `ROSlean/CollisionMonitor/FFI.lean`.
-/

namespace ROSleanCore.CollisionMonitorFFI

open ROSleanCore.CollisionMonitor

/-! The reading of a bit pattern (`F64.mant`, `F64.shift`, `F32.mant`, …),
`coordMm` and `vertexCoordMm` are in `ROSleanCore.CollisionMonitorPoint`. -/

/-! ## Shapes -/

/-- `⌈n / d⌉ + 1`, or `none` if that exceeds `coordLimit`. -/
def radiusOfQuot (n d : Nat) : Option Nat :=
  let r := (n + (d - 1)) / d + 1
  if coordLimit < r then none else some r

/-- A circle's radius in millimetres from the `radius` parameter (binary64,
metres): the magnitude, because upstream squares the radius (R9); rounded up
to whole millimetres; plus 1 (D4). `none` if the value is infinite or NaN, or
if the result exceeds `coordLimit` (D12). -/
def radiusMm (b : Nat) : Option Nat :=
  if !F64.finite b then none
  else radiusOfQuot (F64.mant b * 1000 * 2 ^ F64.shift b) (2 ^ F64.bias)

/-- A polygon vertex coordinate in millimetres from a number of the `points`
parameter (binary32, metres, as upstream parses it): rounded to the nearest
millimetre, a half away from zero. `none` if the value is infinite or NaN. -/
def vertexMm (b : Nat) : Option Int :=
  if !F32.finite b then none
  else
    let n := F32.mant b * 1000 * 2 ^ F32.shift b
    let mm : Nat := (2 * n + 2 ^ F32.bias) / 2 ^ (F32.bias + 1)
    some (if F32.neg b then -(mm : Int) else (mm : Int))

/-- `min_points` from the 64-bit parameter value. Upstream reads it into a
32-bit `int`, so a value outside that range would mean something else there:
refused. A negative value is 0: the zone triggers without any point (R11). -/
def minPointsOfRaw (v : Int) : Option Nat :=
  if v < -0x80000000 || 0x7FFFFFFF < v then none else some v.toNat

/-! ## Timeouts

Upstream converts a timeout with `rclcpp::Duration::from_seconds`, which is
`(int64_t)(seconds * 1e9)`: one IEEE multiplication, rounded to nearest, then
truncation. `nearest` is that rounding, on exact rationals. -/

/-- The binary64 value nearest to the positive rational `n / d`, ties to
even, as `(q, k)` meaning `q * 2 ^ k` with `2 ^ 52 ≤ q ≤ 2 ^ 53`. Requires
`0 < n` and `0 < d`; the result is the IEEE one if it is a normal number. -/
def nearest (n d : Nat) : Nat × Int :=
  let k0 : Int := (n.log2 : Int) - (d.log2 : Int) - 53
  let scaled := fun (k : Int) =>
    if 0 ≤ k then (n, d * 2 ^ k.toNat) else (n * 2 ^ (-k).toNat, d)
  let k := if (scaled k0).1 / (scaled k0).2 < 2 ^ 53 then k0 else k0 + 1
  let num := (scaled k).1
  let den := (scaled k).2
  let q := num / den
  let r := num % den
  (if den < 2 * r || (2 * r == den && q % 2 == 1) then q + 1 else q, k)

/-- `⌊q * 2 ^ k⌋`. -/
def truncDyadic (x : Nat × Int) : Nat :=
  if 0 ≤ x.2 then x.1 * 2 ^ x.2.toNat else x.1 / 2 ^ (-x.2).toNat

/-- `(int64_t)(x * 1e9)` for the non-negative double `x = n / d`. -/
def mulE9Trunc (n d : Nat) : Nat :=
  if n = 0 then 0 else truncDyadic (nearest (n * 1000000000) d)

/-- A timeout in nanoseconds from a parameter in seconds (binary64), as
`rclcpp::Duration::from_seconds` computes it. `none` (D9) if the value is
negative, infinite or NaN, or 9.2·10⁹ s or more; `-0.0` is 0. -/
def timeoutNs (b : Nat) : Option Nat :=
  if !F64.finite b then none
  else if F64.neg b && F64.mant b != 0 then none
  else
    let n := F64.mant b * 2 ^ F64.shift b
    if 9200000000 * 2 ^ F64.bias ≤ n then none
    else some (mulE9Trunc n (2 ^ F64.bias))

/-- What upstream makes of a timeout of `ns` nanoseconds when it uses it as
the default of another timeout parameter: `Duration::seconds()`, which is
`(double)ns / 1e9` with two roundings, and `from_seconds` again. The result
is `ns` or `ns - 1` (for example 2,093,999,999 becomes 2,093,999,998). -/
def nsViaSeconds (ns : Nat) : Nat :=
  if ns = 0 then 0
  else
    -- (double)ns
    let a := nearest ns 1
    -- … / 1e9
    let s := if 0 ≤ a.2 then nearest (a.1 * 2 ^ a.2.toNat) 1000000000
      else nearest a.1 (1000000000 * 2 ^ (-a.2).toNat)
    -- (int64_t)(… * 1e9)
    if 0 ≤ s.2 then mulE9Trunc (s.1 * 2 ^ s.2.toNat) 1 else mulE9Trunc s.1 (2 ^ (-s.2).toNat)

/-- The scan source's timeout (SPEC, R17): its own parameter if that is
set, otherwise the node's `source_timeout` passed through `nsViaSeconds`, as
upstream's default is. The node's value must be acceptable in both cases,
since upstream converts it in both. -/
def sourceTimeoutNs (nodeBits : Nat) (hasOwn : Bool) (ownBits : Nat) : Option Nat :=
  match timeoutNs nodeBits with
  | none => none
  | some nodeNs => if hasOwn then timeoutNs ownBits else some (nsViaSeconds nodeNs)

end ROSleanCore.CollisionMonitorFFI
