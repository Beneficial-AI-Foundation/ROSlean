import ROSleanCore.CollisionMonitorPoint

/-!
# Collision monitor model

Executable model of the core of Nav2's `nav2_collision_monitor` for one laser
scan source, one velocity input, circle and polygon zones and the `stop` and
`slowdown` actions, as specified in `docs/collision_monitor/SPEC.md` (rule
numbers `R…`, differences `D…` and assumptions `A…` refer to that file), with
the Nav2 extension of `docs/collision_monitor/SPEC_NAV2.md` (rules `N…`,
differences `ND…`, events `E1`–`E8`): the lifecycle, scan points and zone
vertices that arrive with each command already in the base frame, zones whose
shape arrives on a topic, and the report for the state topic.

The transform from a sensor's frame to the base frame is **not** in this
model. The C++ shim applies it in `double`, as upstream does (option A of
SPEC_NAV2, section 5), and hands over coordinates in the base frame. What the
model decides is everything after that: which readings count, the conversion
to millimetres, the zones and the action.

The core decides; it does not compute velocities. For each velocity command it
answers "publish nothing", "publish the command unchanged", "publish zero" or
"publish the command scaled by this ratio" (SPEC, "Where the velocity is
computed").

Everything is an integer:

* times are node-clock readings or message stamps in nanoseconds;
* zone shapes are in millimetres in the robot's base frame; of a scan the
  ranges are kept as received, and each command carries the base-frame
  coordinates of the stored scan's readings as bit patterns, which become
  points in millimetres when the command is evaluated (`scanPointMm`, in
  `ROSleanCore.CollisionMonitorPoint`);
* floating-point values that are only classified or compared (scan ranges and
  range limits, velocity components, slowdown ratios) are passed as their
  IEEE-754 bit patterns, so that the comparison is exact.

All of these are plain `Nat` or `Int`. This file must not import Mathlib.
Theorems about it live in `ROSlean.CollisionMonitor`.
-/

namespace ROSleanCore.CollisionMonitor

/-! ## IEEE-754 bit patterns

Only classification and ordering, never arithmetic. A bit pattern is a `Nat`.
The functions below ignore bits above the format's width. Slowdown ratios are
compared as natural numbers instead (`slowdownStep`), so a ratio pattern of
2⁶⁴ or more is never selected, not read modulo 2⁶⁴. -/

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

/-- Where a zone's shape comes from (N29). -/
inductive ZoneShape where
  /-- Given by the parameters. -/
  | fixed (s : Shape)
  /-- A polygon received on a topic. The configuration holds no shape; the
  state does once a message has been accepted (`State.shapes`). -/
  | topic
  deriving Repr, DecidableEq

/-- One zone ("polygon" in Nav2's parameters). -/
structure Zone where
  shape : ZoneShape
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

/-- One reading of a laser scan: the range as received (binary32 bit
pattern). Where its point lies is not stored: the coordinates in the base
frame depend on the transform at the time of each command (N22, N24) and
arrive with the command. -/
structure Beam where
  range : Nat
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

/-- Coordinates as the shim hands them over: `x` and `y` in metres **in the
base frame**, as binary64 bit patterns. -/
abbrev Coords := Nat × Nat

/-- The shape of a topic zone, as last accepted (N33, N34). The message
itself is kept by the shim, which recomputes the vertices for each command
if the message's frame is not the base frame. -/
structure TopicShape where
  /-- The message's frame is not the base frame. -/
  foreign : Bool
  /-- The vertices in the base frame, in millimetres (upstream's `poly_`). -/
  current : List Point
  deriving Repr, DecidableEq

/-- The lifecycle state of the node (N2–N8). -/
inductive Phase where
  | unconfigured
  | inactive
  | active
  | finalized
  deriving Repr, DecidableEq

/-- What a message on the state topic says (N13): no action, a stop or a
slowdown by the zone with that index, or a stop for an invalid source. -/
inductive Report where
  | normal
  | stop (zone : Nat)
  | slowdown (zone : Nat)
  | source
  deriving Repr, DecidableEq

/-- What the node reacts to. Scans and commands carry the node-clock reading
at delivery. -/
inductive Event where
  /-- E1: the lifecycle transition `configure`. -/
  | configure
  /-- E2. -/
  | activate
  /-- E3. -/
  | deactivate
  /-- E4. -/
  | cleanup
  /-- E5. -/
  | shutdown
  /-- E6: a scan arrived. Upstream does not read the clock here; `now` is
  carried so that statements about event sequences can speak about arrival
  order. -/
  | scan (now : Nat) (s : Scan)
  /-- E7: a polygon message arrived for zone `k`. `verts` are its vertices in
  the base frame, as the shim computed them (N33, step 3), or `none` if the
  transform from the message's frame could not be looked up; `foreign` says
  that the message's frame is not the base frame. -/
  | shape (k : Nat) (verts : Option (List Coords)) (foreign : Bool)
  /-- E8: a velocity command arrived. `scanPts` are the base-frame
  coordinates of the readings of the stored scan, one pair per reading in
  scan order, as the shim computed them under the transform it looked up for
  this command (N22–N25); `none` if the lookup failed or was not made.
  `zoneVerts` has one entry per zone: the vertices of that zone's stored
  message in the base frame under the transform looked up for this command
  (N34), or `none`. A missing transform is not an event: upstream looks the
  transform up at every command (N22). -/
  | cmd (now : Nat) (v : Twist) (scanPts : Option (List Coords))
      (zoneVerts : List (Option (List Coords)))
  deriving Repr, DecidableEq

/-- Dynamic state. -/
structure State where
  phase : Phase
  /-- The scan received last (R5), not the one with the newest stamp. -/
  scan : Option Scan
  /-- One entry per zone: the accepted shape of a topic zone; `none` for a
  fixed zone and for a topic zone without shape. -/
  shapes : List (Option TopicShape)
  /-- The velocity demanded for the previous command was zero (R15). -/
  prevZero : Bool
  /-- Node-clock time at which the demanded velocity last became zero. -/
  stopStamp : Nat
  /-- What the previous processed command reported (N13). -/
  prevReport : Report
  deriving Repr, DecidableEq

/-- Initial state: unconfigured, no scan, no shape for any zone, previous
velocity non-zero, stop stamp 0 (R15), previous report `normal`. -/
def State.init (cfg : Config) : State :=
  { phase := .unconfigured, scan := none, shapes := cfg.zones.map (fun _ => none),
    prevZero := false, stopStamp := 0, prevReport := .normal }

/-! ## Scan points -/

/-- R4: a reading is used iff `range_min <= range <= range_max` as `float`
comparisons. A NaN range, or a NaN limit, makes it unused. -/
def Scan.validBeam (s : Scan) (b : Beam) : Bool :=
  F32.le s.rangeMin b.range && F32.le b.range s.rangeMax

/-- The coordinates a command carries can be those of this scan: there is
exactly one pair per reading. Anything else is treated like a failed lookup
(`sourcePoints`); a shim that works as specified never produces it. -/
def Scan.fits (s : Scan) (c : List Coords) : Bool := c.length == s.beams.length

/-- The points of the scan's valid readings in the base frame, in scan order,
given the base-frame coordinates `c` of all its readings: reading `i` is
paired with pair `i` of `c`; the pairs of the readings that are not valid
(R4) are dropped; the others are converted to millimetres (`scanPointMm`).
Which readings count is decided here, from the stored ranges, not by the
shim. -/
def Scan.points (s : Scan) (c : List Coords) : List Point :=
  ((s.beams.zip c).filter (fun bc => s.validBeam bc.1)).map (fun bc => scanPointMm bc.2)

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

/-- A point counts as inside the zone's shape; never, if it has none. -/
def ZoneShape.contains : ZoneShape → Point → Bool
  | .fixed s, p => s.contains p
  | .topic, _ => false

/-- The zone has a shape. -/
def ZoneShape.isSet : ZoneShape → Bool
  | .fixed _ => true
  | .topic => false

def ZoneShape.isTopic : ZoneShape → Bool
  | .fixed _ => false
  | .topic => true

/-- The number of the given points inside the zone. -/
def Zone.count (z : Zone) (pts : List Point) : Nat := pts.countP z.shape.contains

/-- R11, N32: an enabled zone that has a shape triggers when at least
`minPoints` points are inside. A zone without shape never triggers, whatever
`minPoints` is. -/
def Zone.triggered (z : Zone) (pts : List Point) : Bool :=
  z.enabled && z.shape.isSet && decide (z.minPoints ≤ z.count pts)

/-! ## Topic zones (N32–N34) -/

/-- The zone with the shape it has in the state: a topic zone with an
accepted shape is the polygon of that shape's current vertices, with the
margin of D4 like every polygon; any other zone is unchanged. -/
def Zone.withShape (z : Zone) (sh : Option TopicShape) : Zone :=
  match z.shape, sh with
  | .topic, some t => { z with shape := .fixed (.polygon t.current) }
  | _, _ => z

/-- The configuration with every topic zone given the shape it has in
`shapes` (entry `k` for zone `k`; a missing entry is "no shape"). Zone
indices are unchanged. -/
def Config.withShapes (cfg : Config) (shapes : List (Option TopicShape)) : Config :=
  { cfg with zones := cfg.zones.zipIdx.map (fun zi => zi.1.withShape (shapes.getD zi.2 none)) }

/-- The base-frame vertices `verts` in millimetres (`vertexPointMm`), if
every one of them is within the coordinate limit of `Shape.ok`. A coordinate
that is not finite, or is 1024 m or more, is never within it (ND6). -/
def placeVerts (verts : List Coords) : Option (List Point) :=
  let vs := verts.map vertexPointMm
  if vs.all (·.inRange (coordLimit - 3)) then some vs else none

/-- N33, ND6: the shape a polygon message denotes, or `none` if the message
is ignored: the transform from its frame could not be looked up (`verts` is
`none`), it has fewer than three vertices, or a vertex is not finite or
beyond the coordinate limit. -/
def TopicShape.ofMessage (verts : Option (List Coords)) (foreign : Bool) : Option TopicShape :=
  match verts with
  | some vs =>
    if 3 ≤ vs.length then (placeVerts vs).map (fun ps => ⟨foreign, ps⟩) else none
  | none => none

/-- N34: a shape given in another frame follows that frame: when a command
carries the vertices of the stored message under a new transform, and they
are as many as before and all within the coordinate limit, they replace the
current ones; otherwise the current ones stay. A shape given in the base
frame never changes. -/
def TopicShape.refresh (sh : TopicShape) (verts : Option (List Coords)) : TopicShape :=
  if sh.foreign then
    match verts with
    | some vs =>
      match placeVerts vs with
      | some ps => if ps.length = sh.current.length then { sh with current := ps } else sh
      | none => sh
    | none => sh
  else sh

/-- Zone `k` exists, is enabled and is a topic zone. -/
def Config.refreshes (cfg : Config) (k : Nat) : Bool :=
  match cfg.zones[k]? with
  | some z => z.enabled && z.shape.isTopic
  | none => false

/-- N34, ND3: the shapes after the vertices a command carries have been
taken over: the shape of every enabled topic zone is refreshed with its entry
of `zoneVerts` (a missing entry is "no transform"). -/
def refreshShapes (cfg : Config) (shapes : List (Option TopicShape))
    (zoneVerts : List (Option (List Coords))) : List (Option TopicShape) :=
  shapes.zipIdx.map (fun si =>
    if cfg.refreshes si.2 then si.1.map (·.refresh (zoneVerts.getD si.2 none)) else si.1)

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

/-- R5 to R8, N22, N27: the points the zones are tested against at time
`now`, or `none` if the source is invalid and the robot must stop.

* A disabled source contributes no points and is never invalid (R6).
* With a timeout of 0, missing data is "no points" (R8).
* Otherwise the scan must exist and its stamp must not be more than the
  timeout in the past: `now - stamp > timeout` is invalid (R7). A stamp in the
  future is valid.
* Then the scan needs its points in the base frame: if the command carries
  coordinates that fit the scan (`Scan.fits`), the points are the scan's
  under them (`Scan.points`); without them the source is invalid, or has no
  points if the timeout is 0 (N27). -/
def sourcePoints (cfg : Config) (st : State) (now : Nat) (scanPts : Option (List Coords)) :
    Option (List Point) :=
  if !cfg.sourceEnabled then some []
  else
    match st.scan with
    | none => if cfg.sourceTimeout = 0 then some [] else none
    | some s =>
      if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout < now then none
      else
        match scanPts with
        | some c =>
          if s.fits c then some (s.points c)
          else if cfg.sourceTimeout = 0 then some [] else none
        | none => if cfg.sourceTimeout = 0 then some [] else none

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

/-- The zones as they are tested for a command that carries the zone
vertices `zoneVerts`: the configuration with the shapes of the state,
refreshed (N34). -/
def liveConfig (cfg : Config) (st : State) (zoneVerts : List (Option (List Coords))) : Config :=
  cfg.withShapes (refreshShapes cfg st.shapes zoneVerts)

/-- The action demanded for a command arriving at `now` with the given
coordinates (upstream's `process` up to the point where it publishes). It
does not depend on the command's velocity. -/
def evaluate (cfg : Config) (st : State) (now : Nat) (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) : Action :=
  match sourcePoints cfg st now scanPts with
  | none => .stopSource
  | some pts => zonesAction (liveConfig cfg st zoneVerts) pts

/-- E8 step 4: the shapes after a command: refreshed, unless the source is
invalid (upstream's zone loop is then not entered). -/
def shapesAfter (cfg : Config) (st : State) (now : Nat) (scanPts : Option (List Coords))
    (zoneVerts : List (Option (List Coords))) : List (Option TopicShape) :=
  match sourcePoints cfg st now scanPts with
  | none => st.shapes
  | some _ => refreshShapes cfg st.shapes zoneVerts

/-- The velocity the action yields for command `v` is zero in the three
components the monitor publishes (R15, D6). -/
def Action.yieldsZero (a : Action) (v : Twist) : Bool :=
  match a with
  | .stopZone _ | .stopSource => true
  | .pass => v.planarZero
  | .slowdown _ r => v.planarZero || F64.isZero r

/-- N13, N14: what the action reports for command `v`. A slowdown of a
command that is zero in the published components is no action. -/
def Action.report (a : Action) (v : Twist) : Report :=
  match a with
  | .stopSource => .source
  | .stopZone k => .stop k
  | .slowdown k _ => if v.planarZero then .normal else .slowdown k
  | .pass => .normal

/-- What the node does in answer to one event. -/
inductive Output where
  /-- The event is not a processed command: nothing is published, on any
  topic. -/
  | silent
  /-- A command was processed. `velocity` is the action to apply to the
  command and publish, `none` if the velocity is withheld (R15); `report` is
  the state message to publish, `none` if there is none (N13). The polygons
  of visualised zones are published in both cases (N16), which is what
  distinguishes `answer none none` from `silent`. -/
  | answer (velocity : Option Action) (report : Option Report)
  deriving Repr, DecidableEq

/-- The action published as a velocity, if any. -/
def Output.velocity : Output → Option Action
  | .silent => none
  | .answer vel _ => vel

/-- The state message published, if any. -/
def Output.report : Output → Option Report
  | .silent => none
  | .answer _ rep => rep

/-- The event was a command and was processed. -/
def Output.processed : Output → Bool
  | .silent => false
  | .answer _ _ => true

/-- E7: the shape that a polygon message for zone `k` installs, or `none` if
the message is ignored: the node is unconfigured or finalized, there is no
zone `k`, zone `k` is not a topic zone, or `TopicShape.ofMessage` refuses
the message. -/
def acceptShape (cfg : Config) (st : State) (k : Nat) (verts : Option (List Coords))
    (foreign : Bool) : Option TopicShape :=
  if st.phase = .inactive ∨ st.phase = .active then
    match cfg.zones[k]? with
    | some z =>
      match z.shape with
      | .topic => TopicShape.ofMessage verts foreign
      | .fixed _ => none
    | none => none
  else none

/-- Process one event (SPEC_NAV2, 3.2).

* E1–E5: a lifecycle transition changes the phase if it is allowed in the
  current one and is ignored otherwise. Deactivation resets the record of
  published zeros and the previous report (N6); cleanup discards the stored
  scan and every shape (N7); after shutdown nothing changes any more (ND4).
* E6: a scan replaces the stored one while the node is inactive or active
  (N4, R5).
* E7: an accepted shape message replaces the shape of its zone (N33).
* E8: a command is processed only in the active phase (N4) and only if it is
  finite (R2). Then the action is evaluated; the velocity is published,
  except that a zero velocity is no longer published once zero has been
  demanded for more than `stopPubTimeout` without interruption (R15); and the
  report is published if it differs from the previous one (N13).

Only a processed command has an output other than `silent`. -/
def step (cfg : Config) (st : State) : Event → State × Output
  | .configure =>
    (if st.phase = .unconfigured then { st with phase := .inactive } else st, .silent)
  | .activate =>
    (if st.phase = .inactive then { st with phase := .active } else st, .silent)
  | .deactivate =>
    (if st.phase = .active then
        { st with phase := .inactive, prevZero := false, prevReport := .normal }
      else st, .silent)
  | .cleanup =>
    (if st.phase = .inactive then
        { st with phase := .unconfigured, scan := none, shapes := st.shapes.map (fun _ => none) }
      else st, .silent)
  | .shutdown => ({ st with phase := .finalized }, .silent)
  | .scan _ s =>
    (if st.phase = .inactive ∨ st.phase = .active then { st with scan := some s } else st,
      .silent)
  | .shape k verts foreign =>
    match acceptShape cfg st k verts foreign with
    | some sh => ({ st with shapes := st.shapes.set k (some sh) }, .silent)
    | none => (st, .silent)
  | .cmd now v scanPts zoneVerts =>
    if st.phase ≠ .active then (st, .silent)
    else if !v.finite then (st, .silent)
    else
      let a := evaluate cfg st now scanPts zoneVerts
      let zero := a.yieldsZero v
      let r := a.report v
      ({ st with
          shapes := shapesAfter cfg st now scanPts zoneVerts
          prevZero := zero
          stopStamp := if zero && !st.prevZero then now else st.stopStamp
          prevReport := r },
        .answer
          (if zero && st.prevZero && decide (st.stopStamp + cfg.stopPubTimeout < now) then none
            else some a)
          (if r = st.prevReport then none else some r))

/-- E9 (SPEC, D15): a velocity command arrives while the node has **no valid
clock reading** (the last message on `/clock` carried a negative time).
Upstream's process ends on such a clock message; this node goes on, and
this function says what it answers until the clock is valid again.

The command carries no time, no scan coordinates and no zone vertices:
without a clock neither the age of the scan (R7) nor the period of R15 can
be judged. It is processed under the same two conditions as any command
(active phase, N4; finite, R2) and is then always answered with the stop for
an invalid source, whatever the configuration, the stored scan and the
zones. The zero is never withheld: R15 measures a time, and there is none.
For the same reason the record of published zeros is reset (`prevZero :=
false`), so that the period of R15 starts anew with the first command that
has a clock reading again. The phase, the stored scan, the shapes and
`stopStamp` are unchanged.

This is a transition of its own and not a constructor of `Event`, so that
`step` and every theorem about it stand as they are; the theorems about it
are in `ROSlean/CollisionMonitor/NoClock.lean`. -/
def stepNoClock (st : State) (v : Twist) : State × Output :=
  if st.phase ≠ .active then (st, .silent)
  else if !v.finite then (st, .silent)
  else
    ({ st with prevZero := false, prevReport := .source },
      .answer (some .stopSource) (if Report.source = st.prevReport then none else some .source))

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
are what makes the conversions sound (saturated points lie outside every
zone) and what the node rejects when it is configured (SPEC, D-table). -/

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

/-- A fixed shape passes `Shape.ok`. A topic zone has no shape to check: the
shapes it receives are checked one by one (`TopicShape.ofMessage`). -/
def ZoneShape.ok : ZoneShape → Bool
  | .fixed s => s.ok
  | .topic => true

def Zone.ok (z : Zone) : Bool := z.shape.ok && z.action.ok

/-- The configuration is one the node accepts. -/
def Config.ok (cfg : Config) : Bool := cfg.zones.all Zone.ok

end ROSleanCore.CollisionMonitor
