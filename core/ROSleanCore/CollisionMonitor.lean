/-!
# Collision monitor model

Executable model of the core of Nav2's `nav2_collision_monitor` for one laser
scan source, one velocity input, circle and fixed polygon zones and the `stop`
and `slowdown` actions, as specified in `docs/collision_monitor/SPEC.md` (rule
numbers `R…`, differences `D…` and assumptions `A…` refer to that file).

The core decides; it does not compute velocities. For each velocity command it
answers "publish nothing", "publish the command unchanged", "publish zero" or
"publish the command scaled by this ratio" (SPEC, "Where the velocity is
computed").

Everything is an integer:

* times are node-clock readings or message stamps in nanoseconds;
* scan points and zone shapes are in millimetres in the robot's base frame;
* floating-point values that are only classified or compared (scan ranges and
  range limits, velocity components, slowdown ratios) are passed as their
  IEEE-754 bit patterns, so that the comparison is exact.

All of these are plain `Nat` or `Int`. This file must not import Mathlib.
Theorems about it live in `ROSlean.CollisionMonitor`.
-/

namespace ROSleanCore.CollisionMonitor

/-! ## IEEE-754 bit patterns

Only classification and ordering, never arithmetic. A bit pattern is a `Nat`;
bits above the format's width are ignored. -/

/-! Operations on binary32 (`float`) bit patterns. -/
namespace F32

/-- The pattern without its sign bit. For a value that is not NaN, magnitudes
are ordered as their patterns. -/
def mag (b : Nat) : Nat := b % 0x80000000

/-- The sign bit is set. -/
def neg (b : Nat) : Bool := (b / 0x80000000) % 2 == 1

/-- NaN: exponent all ones and a non-zero fraction. -/
def isNaN (b : Nat) : Bool := 0x7F800000 < mag b

/-- IEEE `a <= b`: false if either is NaN; `-0` and `+0` are equal. -/
def le (a b : Nat) : Bool :=
  !isNaN a && !isNaN b &&
    (match neg a, neg b with
     | false, false => mag a ≤ mag b
     | true, true => mag b ≤ mag a
     | true, false => true
     | false, true => mag a == 0 && mag b == 0)

end F32

/-! Operations on binary64 (`double`) bit patterns. -/
namespace F64

/-- Neither infinite nor NaN: the exponent field is not all ones. -/
def isFinite (b : Nat) : Bool := (b / 0x10000000000000) % 0x800 != 0x7FF

/-- `+0.0` or `-0.0`. -/
def isZero (b : Nat) : Bool := b % 0x8000000000000000 == 0

/-- The pattern of `1.0`. -/
def one : Nat := 0x3FF0000000000000

end F64

/-! ## Configuration -/

/-- A point in the base frame, in millimetres. -/
structure Point where
  x : Int
  y : Int
  deriving Repr, DecidableEq

/-- Coordinates handed to the model are saturated to `±coordLimit` mm by the
shim (SPEC, "Shim boundary"). `Config.ok` keeps every zone strictly inside, so
that a saturated point is in no zone. -/
def coordLimit : Nat := 1000000

/-- Squared width, in mm², of the band around a polygon's boundary that counts
as inside (D4). It covers the rounding of points and vertices to whole
millimetres: √5 > √2 + √2 / 2. -/
def polygonMarginSq : Int := 5

/-- The shape of a zone (R9, R10). -/
inductive Shape where
  /-- A disc around the base frame's origin; the radius is in millimetres. -/
  | circle (radius : Nat)
  /-- A polygon given by its vertices in order; the last vertex is joined to
  the first. -/
  | polygon (vertices : List Point)
  deriving Repr, DecidableEq

/-- What a zone demands when it triggers (R12). -/
inductive ZoneAction where
  | stop
  /-- Scale the command by `ratio`, the binary64 bit pattern of the
  `slowdown_ratio` parameter. For values in `[0, 1]` the patterns are ordered
  as the values, and any pattern above `F64.one` (a ratio above 1, a negative
  ratio, infinity, NaN) can never be selected (R13). -/
  | slowdown (ratio : Nat)
  deriving Repr, DecidableEq

/-- One zone ("polygon" in Nav2's parameters). -/
structure Zone where
  shape : Shape
  action : ZoneAction
  /-- The zone triggers when at least this many points are inside (R11). With
  `0` it always triggers. -/
  minPoints : Nat
  enabled : Bool
  deriving Repr, DecidableEq

/-- Node configuration. The order of `zones` is the order of the `polygons`
parameter; it decides which zone is named and which of two equal slowdown
ratios wins (R12, R13). -/
structure Config where
  zones : List Zone
  /-- The scan source's `enabled` parameter (R6). -/
  sourceEnabled : Bool
  /-- `source_timeout` in nanoseconds; `0` disables the staleness check and
  the stop on missing data (R7, R8). -/
  sourceTimeout : Nat
  /-- `stop_pub_timeout` in nanoseconds (R15). -/
  stopPubTimeout : Nat
  deriving Repr, DecidableEq

/-! ## Events and state -/

/-- One reading of a laser scan, after the shim's conversion: the range as
received (binary32 bit pattern) and the point it denotes in the base frame. -/
structure Beam where
  range : Nat
  point : Point
  deriving Repr, DecidableEq

/-- A laser scan as the model sees it. -/
structure Scan where
  /-- `header.stamp` in nanoseconds. -/
  stamp : Nat
  /-- `range_min` and `range_max` as binary32 bit patterns. -/
  rangeMin : Nat
  rangeMax : Nat
  beams : List Beam
  deriving Repr, DecidableEq

/-- A velocity command: the six components as binary64 bit patterns. -/
structure Twist where
  linX : Nat
  linY : Nat
  linZ : Nat
  angX : Nat
  angY : Nat
  angZ : Nat
  deriving Repr, DecidableEq

/-- R2: no component is infinite or NaN. -/
def Twist.finite (v : Twist) : Bool :=
  F64.isFinite v.linX && F64.isFinite v.linY && F64.isFinite v.linZ &&
  F64.isFinite v.angX && F64.isFinite v.angY && F64.isFinite v.angZ

/-- The three components the monitor uses (`linear.x`, `linear.y`,
`angular.z`) are all zero (R15). -/
def Twist.planarZero (v : Twist) : Bool :=
  F64.isZero v.linX && F64.isZero v.linY && F64.isZero v.angZ

/-- What the node reacts to (R1). Each event carries the node-clock reading at
delivery. -/
inductive Event where
  /-- A scan arrived. Upstream does not read the clock here; `now` is carried
  so that statements about event sequences can speak about arrival order. -/
  | scan (now : Nat) (s : Scan)
  /-- A velocity command arrived. -/
  | cmd (now : Nat) (v : Twist)
  deriving Repr, DecidableEq

/-- Dynamic state. -/
structure State where
  /-- The scan received last (R5), not the one with the newest stamp. -/
  scan : Option Scan
  /-- The velocity demanded for the previous command was zero (R15). -/
  prevZero : Bool
  /-- Node-clock time at which the demanded velocity last became zero. -/
  stopStamp : Nat
  deriving Repr, DecidableEq

/-- Initial state: no scan, previous velocity non-zero, stop stamp 0 (R15). -/
def State.init : State := { scan := none, prevZero := false, stopStamp := 0 }

/-! ## Scan points -/

/-- R4: a reading is used iff `range_min <= range <= range_max` as `float`
comparisons. A NaN range, or a NaN limit, makes it unused. -/
def Scan.validBeam (s : Scan) (b : Beam) : Bool :=
  F32.le s.rangeMin b.range && F32.le b.range s.rangeMax

/-- The points of the valid readings, in scan order. -/
def Scan.points (s : Scan) : List Point :=
  (s.beams.filter s.validBeam).map (·.point)

/-! ## Zones -/

/-- The edges of a polygon as (previous vertex, vertex), starting with the edge
from the last vertex to the first, as upstream's loop does. -/
def edges (vs : List Point) : List (Point × Point) :=
  match vs.getLast? with
  | none => []
  | some l => (l :: vs).zip vs

/-- R10: the ray from `p` towards +x crosses the edge from `a` to `b`.
Upstream computes `x_inter = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y)`
and tests `x_inter > p.x`; here the division is multiplied out, which is exact
(D3). The first conjunct implies `a.y ≠ b.y`. -/
def crosses (p a b : Point) : Bool :=
  (decide (p.y ≤ a.y) == decide (b.y < p.y)) &&
    (let d := b.y - a.y
     let l := (p.y - a.y) * (b.x - a.x)
     let r := (p.x - a.x) * d
     if 0 < d then decide (r < l) else decide (l < r))

/-- R10: `p` is inside by upstream's rule, an odd number of crossings. -/
def insidePolygon (vs : List Point) (p : Point) : Bool :=
  (edges vs).countP (fun e => crosses p e.1 e.2) % 2 == 1

/-- `p` is within `√polygonMarginSq` mm of the segment from `a` to `b`: of an
end point, or of the line where its foot lies strictly between them. -/
def nearSegment (p a b : Point) : Bool :=
  let abx := b.x - a.x
  let aby := b.y - a.y
  let apx := p.x - a.x
  let apy := p.y - a.y
  let bpx := p.x - b.x
  let bpy := p.y - b.y
  let dot := abx * apx + aby * apy
  let len := abx * abx + aby * aby
  let cross := abx * apy - aby * apx
  decide (apx * apx + apy * apy ≤ polygonMarginSq) ||
  decide (bpx * bpx + bpy * bpy ≤ polygonMarginSq) ||
  (decide (0 < dot) && decide (dot < len) &&
    decide (cross * cross ≤ polygonMarginSq * len))

/-- D4: `p` is within the margin of the polygon's boundary. -/
def nearBoundary (vs : List Point) (p : Point) : Bool :=
  (edges vs).any (fun e => nearSegment p e.1 e.2)

/-- A point counts as inside the shape. Circle (R9): strictly inside the
radius. Polygon (R10, D4): inside by upstream's rule, or within the margin of
the boundary. -/
def Shape.contains : Shape → Point → Bool
  | .circle r, p => decide (p.x * p.x + p.y * p.y < ((r * r : Nat) : Int))
  | .polygon vs, p => insidePolygon vs p || nearBoundary vs p

/-- The number of the given points inside the zone. -/
def Zone.count (z : Zone) (pts : List Point) : Nat := pts.countP z.shape.contains

/-- R11: an enabled zone triggers when at least `minPoints` points are
inside. -/
def Zone.triggered (z : Zone) (pts : List Point) : Bool :=
  z.enabled && decide (z.minPoints ≤ z.count pts)

def Zone.isStop (z : Zone) : Bool :=
  match z.action with
  | .stop => true
  | .slowdown _ => false

/-! ## Decision -/

/-- What the monitor demands for one command (upstream's `robot_action`). -/
inductive Action where
  /-- No zone restricts the command. -/
  | pass
  /-- Scale by `ratio`, the slowdown ratio (binary64 bit pattern) of zone
  number `zone`. -/
  | slowdown (zone : Nat) (ratio : Nat)
  /-- Zero velocity, demanded by stop zone number `zone`. -/
  | stopZone (zone : Nat)
  /-- Zero velocity, because the scan source has no usable data (R8). -/
  | stopSource
  deriving Repr, DecidableEq

/-- The action demands zero velocity whatever the command. -/
def Action.isStop : Action → Bool
  | .stopZone _ | .stopSource => true
  | .pass | .slowdown _ _ => false

/-- R5 to R8: the points the zones are tested against at time `now`, or `none`
if the source is invalid and the robot must stop.

* A disabled source contributes no points and is never invalid (R6).
* With a timeout of 0, missing data is "no points" (R8).
* Otherwise the scan must exist and its stamp must not be more than the
  timeout in the past: `now - stamp > timeout` is invalid (R7). A stamp in the
  future is valid. -/
def sourcePoints (cfg : Config) (st : State) (now : Nat) : Option (List Point) :=
  if !cfg.sourceEnabled then some []
  else
    match st.scan with
    | none => if cfg.sourceTimeout = 0 then some [] else none
    | some s =>
      if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout < now then none
      else some s.points

/-- R12: the first triggered stop zone in configuration order, if any. -/
def firstStop (cfg : Config) (pts : List Point) : Option Nat :=
  (cfg.zones.zipIdx.find? (fun zi => zi.1.isStop && zi.1.triggered pts)).map (·.2)

/-- One iteration of the slowdown selection: `best` is the smallest ratio so
far and the zone that has it. A triggered slowdown zone replaces it only with
a strictly smaller ratio, so of two equal ratios the earlier zone is kept. -/
def slowdownStep (pts : List Point) (best : Nat × Option Nat) (zi : Zone × Nat) :
    Nat × Option Nat :=
  match zi.1.action with
  | .stop => best
  | .slowdown r => if zi.1.triggered pts && decide (r < best.1) then (r, some zi.2) else best

/-- R13: the triggered slowdown zone with the smallest ratio below 1, as
(ratio, zone). The scan starts from ratio 1 (`F64.one`) and no zone, so a
ratio of 1 or more is never selected. -/
def bestSlowdown (cfg : Config) (pts : List Point) : Nat × Option Nat :=
  cfg.zones.zipIdx.foldl (slowdownStep pts) (F64.one, none)

/-- R12, R13: the action the zones demand for the given points. A triggered
stop zone wins over every slowdown zone. -/
def zonesAction (cfg : Config) (pts : List Point) : Action :=
  match firstStop cfg pts with
  | some k => .stopZone k
  | none =>
    match bestSlowdown cfg pts with
    | (r, some k) => .slowdown k r
    | (_, none) => .pass

/-- The action demanded for a command arriving at `now` (upstream's `process`
up to the point where it publishes). It does not depend on the command. -/
def evaluate (cfg : Config) (st : State) (now : Nat) : Action :=
  match sourcePoints cfg st now with
  | none => .stopSource
  | some pts => zonesAction cfg pts

/-- The velocity the action yields for command `v` is zero in the three
components the monitor publishes (R15, D6). -/
def Action.yieldsZero (a : Action) (v : Twist) : Bool :=
  match a with
  | .stopZone _ | .stopSource => true
  | .pass => v.planarZero
  | .slowdown _ r => v.planarZero || F64.isZero r

/-- What the node does in answer to one event. -/
inductive Output where
  /-- Publish nothing. -/
  | silent
  /-- Publish the command with the action applied to it. -/
  | publish (a : Action)
  deriving Repr, DecidableEq

/-- Process one event.

* A scan replaces the stored one; nothing is published (R5).
* A command with a non-finite component is ignored: nothing is published and
  the state is unchanged (R2).
* Otherwise the action is evaluated and published, except that a zero velocity
  is no longer published once zero has been demanded for more than
  `stopPubTimeout` without interruption (R15). -/
def step (cfg : Config) (st : State) : Event → State × Output
  | .scan _ s => ({ st with scan := some s }, .silent)
  | .cmd now v =>
    if !v.finite then (st, .silent)
    else
      let a := evaluate cfg st now
      let zero := a.yieldsZero v
      let st' := { st with
        prevZero := zero
        stopStamp := if zero && !st.prevZero then now else st.stopStamp }
      if zero && st.prevZero && decide (st.stopStamp + cfg.stopPubTimeout < now) then
        (st', .silent)
      else (st', .publish a)

/-- Runs a sequence of events from a state, returning the final state and the
output for each event. -/
def run (cfg : Config) (st : State) : List Event → State × List Output
  | [] => (st, [])
  | e :: es =>
    let (st', out) := step cfg st e
    let (st'', outs) := run cfg st' es
    (st'', out :: outs)

/-! ## Accepted configurations

`step` is total and its safety properties do not depend on these checks. They
are what makes the shim's conversions sound (saturated points lie outside every
zone) and what the node rejects at start-up (SPEC, D-table). -/

def Point.inRange (p : Point) (limit : Nat) : Bool :=
  decide (p.x.natAbs ≤ limit) && decide (p.y.natAbs ≤ limit)

/-- A circle's radius is at most `coordLimit`. A polygon has at least four
vertices (R10: upstream's own rule) and lies at least 3 mm inside the
coordinate limit, more than the margin. -/
def Shape.ok : Shape → Bool
  | .circle r => decide (r ≤ coordLimit)
  | .polygon vs => decide (4 ≤ vs.length) && vs.all (·.inRange (coordLimit - 3))

/-- A slowdown ratio has its sign bit clear: not negative and not `-0.0`
(D5). -/
def ZoneAction.ok : ZoneAction → Bool
  | .stop => true
  | .slowdown r => decide (r < 0x8000000000000000)

def Zone.ok (z : Zone) : Bool := z.shape.ok && z.action.ok

/-- The configuration is one the node accepts. -/
def Config.ok (cfg : Config) : Bool := cfg.zones.all Zone.ok

end ROSleanCore.CollisionMonitor
