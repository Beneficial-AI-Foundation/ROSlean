import ROSleanCore.CollisionMonitor

/-!
# Review checks for the collision monitor model (CM1/CM2)

Numerical comparison of `core/ROSleanCore/CollisionMonitor.lean` with the
hardware floating-point operations and with a transcription of the upstream
C++ zone tests into Lean `Float` (binary64) and `Float32`.

Run from `core/`:

    lake env lean --run ../review/collision_monitor/Checks.lean

Nothing here is a proof. Each section prints what it compared and how many
disagreements it found; a section that is expected to find none says so.
-/

open ROSleanCore.CollisionMonitor

namespace Review

/-! ## Random numbers -/

abbrev R := StateM UInt64

def nextU64 : R UInt64 := do
  let x ← get
  let x := x ^^^ (x <<< 13)
  let x := x ^^^ (x >>> 7)
  let x := x ^^^ (x <<< 17)
  set x
  return x * 0x2545F4914F6CDD1D

/-- Uniform in `[0, 1)`. -/
def unit : R Float := do
  let x ← nextU64
  return (x >>> 11).toFloat / 9007199254740992.0

def uniform (lo hi : Float) : R Float := do
  let u ← unit
  return lo + u * (hi - lo)

def below (n : Nat) : R Nat := do
  let x ← nextU64
  return (x >>> 16).toNat % n

def pick [Inhabited α] (xs : Array α) : R α := do
  let i ← below xs.size
  return xs[i]!

/-! ## IEEE helpers against the hardware -/

/-- Sign × exponent class × fraction class for a format with `e` exponent bits
and `f` fraction bits: zeros, subnormals, the neighbourhood of 1, the largest
finite values, infinities and NaNs of both signs. -/
def grid (e f : Nat) : List Nat := Id.run do
  let emax := 2 ^ e - 1
  let bias := 2 ^ (e - 1) - 1
  let exps := [0, 1, 2, bias - 2, bias - 1, bias, bias + 1, emax - 1, emax]
  let fmax := 2 ^ f - 1
  let fracs := [0, 1, 2, 2 ^ (f - 1), fmax - 1, fmax]
  let mut out := []
  for s in [0, 1] do
    for x in exps do
      for m in fracs do
        out := (s * 2 ^ (e + f) + x * 2 ^ f + m) :: out
  return out

def hwLe32 (a b : Nat) : Bool :=
  decide (Float32.ofBits a.toUInt32 ≤ Float32.ofBits b.toUInt32)

def hwFinite64 (a : Nat) : Bool := (Float.ofBits a.toUInt64).isFinite

def hwZero64 (a : Nat) : Bool := Float.ofBits a.toUInt64 == 0.0

/-- The value is in `[0, 1)` and is not `-0.0` (hardware test; `1 / -0 = -inf`). -/
def hwUnitNonNeg (a : Nat) : Bool :=
  let x := Float.ofBits a.toUInt64
  x ≥ 0.0 && x < 1.0 && (1.0 / x > 0.0)

def rand32 : R Nat := do
  let k ← below 4
  if k == 0 then pick (grid 8 23).toArray
  else
    let x ← nextU64
    return (x >>> 32).toNat

def rand64 : R Nat := do
  let k ← below 4
  if k == 0 then pick (grid 11 52).toArray
  else
    let x ← nextU64
    return x.toNat

def checkIeee : R (List String) := do
  let g32 := grid 8 23
  let g64 := grid 11 52
  let mut badLe := 0
  let mut firstLe : Option (Nat × Nat) := none
  let mut nLe := 0
  -- every pair of the grid, also with bits above the width set
  for a in g32 do
    for b in g32 do
      nLe := nLe + 2
      if F32.le a b != hwLe32 a b then
        badLe := badLe + 1
        if firstLe.isNone then firstLe := some (a, b)
      if F32.le (a + 5 * 2 ^ 32) (b + 3 * 2 ^ 40) != hwLe32 a b then
        badLe := badLe + 1
        if firstLe.isNone then firstLe := some (a + 5 * 2 ^ 32, b + 3 * 2 ^ 40)
  for _ in [0:300000] do
    let a ← rand32
    let b ← rand32
    nLe := nLe + 1
    if F32.le a b != hwLe32 a b then
      badLe := badLe + 1
      if firstLe.isNone then firstLe := some (a, b)
  let mut badFin := 0
  let mut badZero := 0
  let mut badOne := 0
  let mut badOrd := 0
  let mut n64 := 0
  let test64 := fun (a : Nat) => (F64.isFinite a != hwFinite64 a, F64.isZero a != hwZero64 a,
    decide (a < F64.one) != hwUnitNonNeg a)
  for a in g64 do
    n64 := n64 + 2
    for a' in [a, a + 7 * 2 ^ 64] do
      let (x, y, _) := test64 a'
      if x then badFin := badFin + 1
      if y then badZero := badZero + 1
    if (test64 a).2.2 then badOne := badOne + 1
    for b in g64 do
      -- below `F64.one` the order of patterns is the order of values
      if a < F64.one && b < F64.one then
        if decide (a < b) != decide (Float.ofBits a.toUInt64 < Float.ofBits b.toUInt64) then
          badOrd := badOrd + 1
  for _ in [0:300000] do
    let a ← rand64
    let b ← rand64
    n64 := n64 + 1
    let (x, y, z) := test64 a
    if x then badFin := badFin + 1
    if y then badZero := badZero + 1
    if z then badOne := badOne + 1
    -- force both below one half of the time
    let a := if a % 2 == 0 then a % F64.one else a
    let b := if b % 2 == 0 then b % F64.one else b
    if a < F64.one && b < F64.one then
      if decide (a < b) != decide (Float.ofBits a.toUInt64 < Float.ofBits b.toUInt64) then
        badOrd := badOrd + 1
  return [
    s!"F32.le vs hardware <=: {nLe} pairs, {badLe} disagreements (expected 0){match firstLe with | some p => s!", first {p}" | none => ""}",
    s!"F64.isFinite vs hardware: {n64} patterns, {badFin} disagreements (expected 0)",
    s!"F64.isZero vs hardware == 0.0: {n64} patterns, {badZero} disagreements (expected 0)",
    s!"pattern < F64.one  iff  value in [0,1) and not -0.0: {badOne} disagreements (expected 0)",
    s!"order of patterns below F64.one vs order of values: {badOrd} disagreements (expected 0)"]

/-! ## Zone geometry: upstream's floating-point tests against the model

Upstream sees points and vertices as `double` metres. The model sees what the
shim of SPEC "Shim boundary" hands over. The claim under test ("Safe
direction"): a point that upstream counts as inside is counted by the
model. -/

structure P2 where
  x : Float
  y : Float
  deriving Inhabited

/-- `Polygon::isPointInside`, `src/polygon.cpp`, transcribed. -/
def upInside (poly : Array P2) (p : P2) : Bool := Id.run do
  let n := poly.size
  let mut res := false
  let mut i := n - 1
  for j in [0:n] do
    let a := poly[i]!
    let b := poly[j]!
    if decide (p.y ≤ a.y) == decide (p.y > b.y) then
      let xi := a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)
      if xi > p.x then res := !res
    i := j
  return res

/-- The shim's `to_mm` for a scan point coordinate. -/
def toMm (v : Float) : Int :=
  let t := v * 1000.0
  if t.isNaN || t ≥ 1e6 then 1000000
  else if t ≤ -1e6 then -1000000
  else t.toInt64.toInt

/-- The shim's `llround(x * 1000.0)` for a vertex coordinate. -/
def vertMm (v : Float) : Int := (Float.round (v * 1000.0)).toInt64.toInt

/-- The shim's `(int64_t)ceil(radius * 1000.0) + 1`, then Lean's `.toInt.toNat`. -/
def radMm (r : Float) : Nat := ((Float.ceil (r * 1000.0)).toInt64.toInt + 1).toNat

def f32 (x : Float) : Float := x.toFloat32.toFloat

def segDist (p a b : P2) : Float :=
  let abx := b.x - a.x
  let aby := b.y - a.y
  let len := abx * abx + aby * aby
  let t := if len == 0.0 then 0.0 else ((p.x - a.x) * abx + (p.y - a.y) * aby) / len
  let t := if t < 0.0 then 0.0 else if t > 1.0 then 1.0 else t
  let dx := p.x - (a.x + t * abx)
  let dy := p.y - (a.y + t * aby)
  Float.sqrt (dx * dx + dy * dy)

def boundaryDist (poly : Array P2) (p : P2) : Float := Id.run do
  let n := poly.size
  let mut best := 1e300
  let mut i := n - 1
  for j in [0:n] do
    let d := segDist p poly[i]! poly[j]!
    if d < best then best := d
    i := j
  return best

/-- A polygon with `float` vertices, of one of several kinds. -/
def genPolygon : R (Array P2) := do
  let kind ← below 6
  let s ← pick #[0.003, 0.03, 0.4, 0.4, 5.0, 50.0]
  let cx ← pick #[0.0, 0.0, 0.3, -2.0, 900.0]
  let cy ← pick #[0.0, 0.0, -0.3, 2.0, -900.0]
  let n := 4 + (← below 5)
  let mut vs : Array P2 := #[]
  if kind == 0 || kind == 4 then
    -- arbitrary, usually self-intersecting
    for _ in [0:n] do
      vs := vs.push ⟨f32 (cx + (← uniform (-s) s)), f32 (cy + (← uniform (-s) s))⟩
  else if kind == 1 || kind == 5 then
    -- star-shaped around the centre
    for i in [0:n] do
      let ang := (i.toFloat + (← uniform 0.0 0.9)) * 6.283185307179586 / n.toFloat
      let rad := s * (← uniform 0.3 1.0)
      vs := vs.push ⟨f32 (cx + rad * Float.cos ang), f32 (cy + rad * Float.sin ang)⟩
  else if kind == 2 then
    -- axis-parallel box, as in upstream's example file
    let x0 := f32 (cx - (← uniform 0.0 s))
    let x1 := f32 (cx + (← uniform 0.0 s))
    let y0 := f32 (cy - (← uniform 0.0 s))
    let y1 := f32 (cy + (← uniform 0.0 s))
    vs := #[⟨x1, y1⟩, ⟨x1, y0⟩, ⟨x0, y0⟩, ⟨x0, y1⟩]
  else
    -- sliver: almost collinear
    let sq ← pick #[1e-2, 1e-3, 1e-4]
    for _ in [0:n] do
      vs := vs.push ⟨f32 (cx + (← uniform (-s) s)), f32 (cy + sq * (← uniform (-s) s))⟩
  if kind == 4 || kind == 5 then
    -- closing vertex repeated, as many hand-written configurations do
    vs := vs.push vs[0]!
  return vs

/-- A point near the boundary of the polygon. -/
def genPoint (poly : Array P2) : R P2 := do
  let n := poly.size
  let j ← below n
  let a := poly[(j + n - 1) % n]!
  let b := poly[j]!
  let t ← uniform (-0.05) 1.05
  let mag ← pick #[0.0, 1e-12, 1e-7, 1e-4, 1e-3, 3e-3]
  let ang ← uniform 0.0 6.283185307179586
  let r ← uniform 0.0 mag
  let mut x := a.x + t * (b.x - a.x) + r * Float.cos ang
  let mut y := a.y + t * (b.y - a.y) + r * Float.sin ang
  -- sometimes put a coordinate next to a whole millimetre, where truncation jumps
  let snap ← below 4
  let eps ← pick #[0.0, 1e-13, -1e-13, 1e-8, -1e-8]
  if snap == 1 || snap == 3 then x := Float.round (x * 1000.0) / 1000.0 + eps
  if snap == 2 || snap == 3 then y := Float.round (y * 1000.0) / 1000.0 + eps
  return ⟨x, y⟩

def checkPolygons (polys ptsPer : Nat) : R (List String) := do
  let mut missed := 0
  let mut firstMiss : String := ""
  let mut inUp := 0
  let mut inModel := 0
  let mut extraMax := 0.0
  let mut needMax := 0.0
  let mut savedByMargin := 0
  let mut total := 0
  for _ in [0:polys] do
    let poly ← genPolygon
    let shape := Shape.polygon (poly.toList.map fun v => ⟨vertMm v.x, vertMm v.y⟩)
    for _ in [0:ptsPer] do
      let p ← genPoint poly
      total := total + 1
      let u := upInside poly p
      let m := shape.contains ⟨toMm p.x, toMm p.y⟩
      if u then inUp := inUp + 1
      if m then inModel := inModel + 1
      if u && !insidePolygon (poly.toList.map fun v => ⟨vertMm v.x, vertMm v.y⟩) ⟨toMm p.x, toMm p.y⟩ then
        -- the margin is what saves this point: how much of it was needed?
        let ip : P2 := ⟨Float.ofInt (toMm p.x), Float.ofInt (toMm p.y)⟩
        let d := boundaryDist (poly.map fun v => ⟨Float.ofInt (vertMm v.x), Float.ofInt (vertMm v.y)⟩) ip
        savedByMargin := savedByMargin + 1
        if d > needMax then needMax := d
      if u && !m then
        missed := missed + 1
        if firstMiss == "" then
          firstMiss := s!"polygon {poly.toList.map fun v => (v.x, v.y)} point ({p.x}, {p.y})"
      if m && !u then
        let d := boundaryDist poly p
        if d > extraMax then extraMax := d
  return [
    s!"polygons: {polys} polygons, {total} points near their boundaries; upstream inside {inUp}, model inside {inModel}",
    s!"  inside for upstream but not for the model: {missed} (expected 0) {firstMiss}",
    s!"  inside for upstream, outside the rounded polygon by R10 alone (so only the margin counts them): {savedByMargin}; largest distance of such a point from the rounded boundary: {needMax} mm (SPEC: below 2.13; margin 2.236)",
    s!"  largest distance outside the configured polygon that the model still counts: {extraMax * 1000.0} mm (SPEC: up to 4.4)"]

def checkCircles (n : Nat) : R (List String) := do
  let mut missed := 0
  let mut extraMax := 0.0
  let mut first := ""
  for _ in [0:n] do
    let r ← (do
      let k ← below 3
      if k == 0 then pick #[0.3, 0.25, 1.0, 0.001, 999.0, 0.0005]
      else if k == 1 then uniform 0.0005 2.0
      else return f32 (← uniform 0.01 50.0))
    let ang ← uniform 0.0 6.283185307179586
    let d ← uniform (-0.004) 0.004
    let x := (r + d) * Float.cos ang
    let y := (r + d) * Float.sin ang
    let u := decide (x * x + y * y < r * r)
    let m := (Shape.circle (radMm r)).contains ⟨toMm x, toMm y⟩
    if u && !m then
      missed := missed + 1
      if first == "" then first := s!"radius {r} point ({x}, {y})"
    if m && !u then
      let e := Float.sqrt (x * x + y * y) - r
      if e > extraMax then extraMax := e
  return [
    s!"circles: {n} points within 4 mm of the radius; inside for upstream but not for the model: {missed} (expected 0) {first}",
    s!"  largest distance outside the configured radius that the model still counts: {extraMax * 1000.0} mm (SPEC: up to 3.5)"]

/-! ## Slowdown selection: upstream's loop against the model with the shim's
multiplication -/

structure V3 where
  x : Float
  y : Float
  tw : Float

def V3.scale (v : V3) (r : Float) : V3 := ⟨v.x * r, v.y * r, v.tw * r⟩
def V3.norm (v : V3) : Float := v.x * v.x + v.y * v.y + v.tw * v.tw
def V3.bits (v : V3) : List UInt64 := [v.x.toBits, v.y.toBits, v.tw.toBits]
def V3.isZero (v : V3) : Bool := v.x == 0.0 && v.y == 0.0 && v.tw == 0.0

/-- `processStopSlowdownLimit` for slowdown zones that all trigger: the
requested velocity and the zone that set it. -/
def upSlowdown (v : V3) (rs : Array Float) : V3 × Option Nat := Id.run do
  let mut req := v
  let mut who := none
  for i in [0:rs.size] do
    let safe := v.scale rs[i]!
    if safe.norm < req.norm then
      req := safe
      who := some i
  return (req, who)

def modelSlowdown (v : V3) (rs : Array Float) : V3 × Action :=
  let cfg : Config :=
    { zones := rs.toList.map fun r => ⟨.circle 1, .slowdown r.toBits.toNat, 0, true⟩
      sourceEnabled := true, sourceTimeout := 0, stopPubTimeout := 0 }
  match zonesAction cfg [] with
  | .slowdown k r => (v.scale (Float.ofBits r.toUInt64), .slowdown k r)
  | a => (v, a)

def genComp (extreme : Bool) : R Float := do
  let k ← below (if extreme then 8 else 5)
  let sgn ← pick #[1.0, -1.0]
  if k == 0 then return 0.0 * sgn
  else if k < 5 then return sgn * (← uniform 0.001 3.0)
  else if k == 5 then return sgn * 1e-170
  else if k == 6 then return sgn * 1e160
  else return sgn * 5e-324

def genRatio (extreme : Bool) : R Float := do
  let k ← below 12
  if k == 0 then return 0.0
  else if k == 1 then return 0.5
  else if k == 2 then return Float.ofBits 0x3FDFFFFFFFFFFFFF -- just below 0.5
  else if k == 3 then return Float.ofBits 0x3FEFFFFFFFFFFFFF -- just below 1
  else if k == 4 then return 1.0
  else if k == 5 then return 2.0
  else if k == 6 then return 1.0 / 0.0
  else if k == 7 then return 0.0 / 0.0
  else if k == 8 && extreme then return 1e-320
  else uniform 0.0 1.0

def checkSlowdown (n : Nat) : R (List String) := do
  let mut diffOrdinary := 0
  let mut diffOrdinaryBig := 0
  let mut diffExtreme := 0
  let mut zeroCmdAction := 0
  let mut amplified := 0
  let mut zeroDisagree := 0
  let mut zeroDisagreeOrdinary := 0
  let mut firstBig := ""
  for it in [0:n] do
    let extreme := it % 4 == 0
    let v : V3 := ⟨← genComp extreme, ← genComp extreme, ← genComp extreme⟩
    let m := 1 + (← below 4)
    let mut rs : Array Float := #[]
    for _ in [0:m] do rs := rs.push (← genRatio extreme)
    let (u, who) := upSlowdown v rs
    let (w, act) := modelSlowdown v rs
    -- T1 / "No amplification" on the published bits
    for (a, b) in [(v.x, w.x), (v.y, w.y), (v.tw, w.tw)] do
      if !(Float.abs b ≤ Float.abs a) || ((a.toBits >>> 63) != (b.toBits >>> 63)) then
        amplified := amplified + 1
    if v.isZero then
      -- same bits, but upstream reports no action and the model a slowdown
      if who.isNone && act != .pass then zeroCmdAction := zeroCmdAction + 1
    -- R15's zero test: upstream on the product, the model on command and ratio
    let modelZero := act.yieldsZero ⟨v.x.toBits.toNat, v.y.toBits.toNat, 0, 0, 0, v.tw.toBits.toNat⟩
    if modelZero != u.isZero then
      if extreme then zeroDisagree := zeroDisagree + 1
      else zeroDisagreeOrdinary := zeroDisagreeOrdinary + 1
    if u.bits != w.bits then
      if extreme then diffExtreme := diffExtreme + 1
      else
        diffOrdinary := diffOrdinary + 1
        -- more than a rounding apart?
        let far := [(u.x, w.x), (u.y, w.y), (u.tw, w.tw)].any fun (a, b) =>
          Float.abs (a - b) > 1e-12 * (Float.abs a + Float.abs b)
        if far then
          diffOrdinaryBig := diffOrdinaryBig + 1
          if firstBig == "" then
            firstBig := s!"cmd ({v.x}, {v.y}, {v.tw}) ratios {rs.toList} upstream ({u.x}, {u.y}, {u.tw}) model ({w.x}, {w.y}, {w.tw})"
  return [
    s!"slowdown: {n} cases, non-negative ratios, every zone triggered",
    s!"  published bits differ, ordinary commands (|c| in [1e-3, 3] or 0): {diffOrdinary}, of which more than a rounding apart: {diffOrdinaryBig} (expected 0) {firstBig}",
    s!"  published bits differ, commands with a component of 1e-170, 1e160 or 5e-324, or a ratio of 1e-320: {diffExtreme} (D6 expects some)",
    s!"  zero command, upstream takes no action, model returns a slowdown: {zeroCmdAction} (same bits; not in D6)",
    s!"  upstream's zero test and Action.yieldsZero disagree: ordinary {zeroDisagreeOrdinary} (expected 0), extreme {zeroDisagree} (D6 expects some)",
    s!"  model output larger than the command or of other sign: {amplified} (expected 0)"]

/-! ## Timeouts: the per-source default goes through seconds twice upstream -/

/-- `Duration::from_seconds`: multiply by 1e9, truncate. -/
def fromSeconds (s : Float) : Int := (s * 1e9).toInt64.toInt

/-- `Duration::seconds()`. -/
def toSeconds (ns : Int) : Float := Float.ofInt ns / 1e9

def checkTimeoutRoundTrip : List String := Id.run do
  let mut found : List (Float × Int × Int) := []
  let mut count := 0
  -- node `source_timeout` values with up to three decimals, 0.001 .. 30.000
  for k in [1:30001] do
    let s := k.toFloat / 1000.0
    let once := fromSeconds s
    let twice := fromSeconds (toSeconds once)
    if once != twice then
      count := count + 1
      if found.length < 5 then found := found ++ [(s, once, twice)]
  return [
    s!"source_timeout k/1000 s for k = 1..30000: converted once and converted twice differ for {count} values; first {found}"]

end Review

open Review in
def main : IO Unit := do
  let run := fun {α} (r : R α) (seed : UInt64) => (r.run seed).1
  for l in run checkIeee 0x9E3779B97F4A7C15 do IO.println l
  for l in run (checkPolygons 20000 60) 0xD1B54A32D192ED03 do IO.println l
  for l in run (checkCircles 300000) 0x94D049BB133111EB do IO.println l
  for l in run (checkSlowdown 300000) 0xBF58476D1CE4E5B9 do IO.println l
  for l in checkTimeoutRoundTrip do IO.println l
  -- the shim's radius conversion for a negative radius (upstream uses radius²)
  IO.println s!"radius -0.3 m: (int64_t)ceil(r * 1000) + 1 = {(Float.ceil ((-0.3 : Float) * 1000.0)).toInt64.toInt + 1}, as Nat {radMm (-0.3)}; radius 0.3 m gives {radMm 0.3}"
