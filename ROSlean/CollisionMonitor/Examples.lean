import ROSleanCore.CollisionMonitor

/-!
# Worked examples for the collision monitor model

Each example is checked by the kernel (`by decide`). Rule numbers refer to
`docs/collision_monitor/SPEC.md` (`R…`, `D…`) and
`docs/collision_monitor/SPEC_NAV2.md` (`N…`, `ND…`).

Times are nanoseconds, written with `ms`. Points are millimetres. Floating
point values appear as IEEE-754 bit patterns; the ones used here are named
below.

The model receives, with every command, the base-frame coordinates of the
readings of the stored scan (the C++ shim applies the sensor-to-base
transform; SPEC_NAV2, section 5, option A). The examples therefore write a
scan as a `Sweep`: ranges together with the coordinates the shim would hand
over for them, and deliver events through `deliver`, which plays the shim:
a scan event stores the ranges, and every later `cmd` carries the
coordinates of the sweep given last, unless the example says otherwise
(`cmdNone`: the transform lookup failed; `cmdPts`: other coordinates, as
after the robot or the sensor frame has moved; `cmdZ`: vertices for a zone).

The first part is about the decision for a command in an active node: the
examples there start the node (`configure`, `activate`) and look at the
velocity part of the outputs only (`velocities`: `none` is "no velocity
published", `some a` is "the command with `a` applied"). The second part
("The Nav2 extension") has a scenario for each new event and looks at whole
outputs: `silent`, or `answer velocity report`.
-/

namespace ROSlean.CollisionMonitor.Examples

open ROSleanCore.CollisionMonitor

/-- Milliseconds to nanoseconds. -/
def ms (n : Nat) : Nat := n * 1000000

/-! Bit patterns of `float` range values. -/
def r0_05 : Nat := 0x3D4CCCCD
def r0_1 : Nat := 0x3DCCCCCD
def r0_2 : Nat := 0x3E4CCCCD
def r0_25 : Nat := 0x3E800000
def r0_5 : Nat := 0x3F000000
def r2 : Nat := 0x40000000
def r10 : Nat := 0x41200000
def r30 : Nat := 0x41F00000
def rInf : Nat := 0x7F800000
def rNaN : Nat := 0x7FC00000
def rNeg1 : Nat := 0xBF800000

/-! Bit patterns of `double` values: velocity components and slowdown ratios. -/
def d0 : Nat := 0
def dNeg0 : Nat := 0x8000000000000000
def d0_2 : Nat := 0x3FC999999999999A
def d0_3 : Nat := 0x3FD3333333333333
def d0_25 : Nat := 0x3FD0000000000000
def d0_5 : Nat := 0x3FE0000000000000
def d0_7 : Nat := 0x3FE6666666666666
def d0_75 : Nat := 0x3FE8000000000000
def d1 : Nat := F64.one
def d2 : Nat := 0x4000000000000000
def d10 : Nat := 0x4024000000000000
def dNeg0_5 : Nat := 0xBFE0000000000000
def dNeg0_85 : Nat := 0xBFEB333333333333
def dNeg1 : Nat := 0xBFF0000000000000
def dInf : Nat := 0x7FF0000000000000
def dNaN : Nat := 0x7FF8000000000000

/-- A command with the given `linear.x` and `angular.z`. -/
def twist (lin ang : Nat) : Twist := ⟨lin, 0, 0, 0, 0, ang⟩

/-- Forward at 0.5 m/s, turning at 0.2 rad/s. -/
def fwd : Twist := twist d0_5 d0_2

/-- All components zero. -/
def halt : Twist := twist d0 d0

/-- One reading as the examples write it: the range (binary32 pattern) and
the coordinates of its point in the base frame, in metres (binary64
patterns), as the shim computes them. -/
structure Rd where
  range : Nat
  x : Nat
  y : Nat

/-- A scan together with the base-frame coordinates of its readings. -/
structure Sweep where
  stamp : Nat
  rangeMin : Nat
  rangeMax : Nat
  rds : List Rd

/-- What the model stores of a sweep: stamp, limits and ranges. -/
def Sweep.scan (s : Sweep) : Scan :=
  { stamp := s.stamp, rangeMin := s.rangeMin, rangeMax := s.rangeMax,
    beams := s.rds.map (fun r => ⟨r.range⟩) }

/-- What a command carries for it: one coordinate pair per reading. -/
def Sweep.coords (s : Sweep) : List Coords := s.rds.map (fun r => (r.x, r.y))

/-- The points of the sweep's valid readings, in millimetres. -/
def Sweep.points (s : Sweep) : List Point := s.scan.points s.coords

/-- A sweep with range limits 0.1 m to 10 m. -/
def scanAt (stamp : Nat) (rds : List Rd) : Sweep :=
  { stamp, rangeMin := r0_1, rangeMax := r10, rds }

/-- A reading straight ahead at 0.2 m, 0.25 m, 0.5 m and 2 m, and one at
0.25 m to the left: the range, and the coordinates in the base frame in
metres. -/
def at0_2 : Rd := ⟨r0_2, d0_2, d0⟩
def at0_25 : Rd := ⟨r0_25, d0_25, d0⟩
def at0_5 : Rd := ⟨r0_5, d0_5, d0⟩
def at2 : Rd := ⟨r2, d2, d0⟩
def left0_25 : Rd := ⟨r0_25, d0, d0_25⟩

-- The millimetres of these readings. 0.2 m is not a binary number; its
-- double is a little above 0.2 and truncates to 200 mm. (The option only
-- lets the elaborator unfold the list functions far enough; the kernel
-- checks the result as for every other example.)
set_option maxRecDepth 4000 in
example : (scanAt 0 [at0_2, at0_25, at0_5, at2, left0_25]).points
    = [⟨200, 0⟩, ⟨250, 0⟩, ⟨500, 0⟩, ⟨2000, 0⟩, ⟨0, 250⟩] := by decide

/-- Zone 0: stop inside 0.3 m. Zone 1: slow down to half inside 0.6 m. Each
needs two points. The scan must be at most 2 s old; zero velocity is published
for 1 s. -/
def cfg : Config :=
  { zones := [⟨.fixed (.circle 300), .stop, 2, true⟩,
      ⟨.fixed (.circle 600), .slowdown d0_5, 2, true⟩]
    sourceEnabled := true
    sourceTimeout := ms 2000
    stopPubTimeout := ms 1000 }

/-- What the examples deliver. The lifecycle transitions and `shape` are the
model's events; `scan` gives a sweep; the four kinds of command differ in
what they carry (see the head of this file). -/
inductive Ev where
  | configure
  | activate
  | deactivate
  | cleanup
  | shutdown
  | scan (now : Nat) (s : Sweep)
  | shape (k : Nat) (verts : Option (List Coords)) (foreign : Bool)
  /-- A command with the coordinates of the sweep given last, unchanged. -/
  | cmd (now : Nat) (v : Twist)
  /-- A command without scan coordinates: the transform lookup failed. -/
  | cmdNone (now : Nat) (v : Twist)
  /-- A command with the given scan coordinates. -/
  | cmdPts (now : Nat) (v : Twist) (pts : List Coords)
  /-- As `cmd`, with the vertices `z` for zone 0 and none for zone 1. -/
  | cmdZ (now : Nat) (v : Twist) (z : Option (List Coords))

/-- The events of the model for a list of example events, as a shim that
remembers the coordinates of the sweep given last (`last`) delivers them. -/
def deliver : Option (List Coords) → List Ev → List Event
  | _, [] => []
  | last, .configure :: es => .configure :: deliver last es
  | last, .activate :: es => .activate :: deliver last es
  | last, .deactivate :: es => .deactivate :: deliver last es
  | last, .cleanup :: es => .cleanup :: deliver last es
  | last, .shutdown :: es => .shutdown :: deliver last es
  | _, .scan now s :: es => .scan now s.scan :: deliver (some s.coords) es
  | last, .shape k verts foreign :: es => .shape k verts foreign :: deliver last es
  | last, .cmd now v :: es => .cmd now v last [] :: deliver last es
  | last, .cmdNone now v :: es => .cmd now v none [] :: deliver last es
  | last, .cmdPts now v pts :: es => .cmd now v (some pts) [] :: deliver last es
  | last, .cmdZ now v z :: es => .cmd now v last [z, none] :: deliver last es

/-- The state after a run from start-up, where the node is unconfigured. -/
def final (c : Config) (es : List Ev) : State := (run c (State.init c) (deliver none es)).1

/-- The outputs of a run from start-up. -/
def outputs (c : Config) (es : List Ev) : List Output :=
  (run c (State.init c) (deliver none es)).2

/-- The two transitions that bring the node up. -/
def up : List Ev := [.configure, .activate]

/-- The state of a node that has just been brought up. -/
def active (c : Config) : State := final c up

/-- The velocity part of the outputs for the events `es`, delivered to a node
that has just been brought up. -/
def velocities (c : Config) (es : List Ev) : List (Option Action) :=
  ((run c (active c) (deliver none es)).2).map Output.velocity

/-- A command with the coordinates of the sweep given last. -/
def cmd (now : Nat) (v : Twist) : Ev := .cmd now v

/-! ## The three outcomes -/

-- Nothing near: the command passes (R14).
example : velocities cfg [.scan (ms 1000) (scanAt (ms 990) [at2, at2, at2]), cmd (ms 1010) fwd]
    = [none, some .pass] := by decide

-- Two points inside 0.3 m: stop, and the stop zone is named (R12).
example : velocities cfg [.scan (ms 1000) (scanAt (ms 990) [at0_25, left0_25, at2]),
      cmd (ms 1010) fwd]
    = [none, some (.stopZone 0)] := by decide

-- One point inside 0.3 m is below `min_points`; with a second inside 0.6 m the
-- slowdown zone triggers and hands back its ratio (R11, R13).
example : velocities cfg [.scan (ms 1000) (scanAt (ms 990) [at0_25, at0_5, at2]),
      cmd (ms 1010) fwd]
    = [none, some (.slowdown 1 d0_5)] := by decide

-- The circle test is strict: a point at exactly the radius is outside (R9).
example : (Shape.circle 300).contains ⟨300, 0⟩ = false := by decide
example : (Shape.circle 300).contains ⟨299, 0⟩ = true := by decide
example : (Shape.circle 300).contains ⟨-180, 239⟩ = true := by decide
example : (Shape.circle 300).contains ⟨-180, 240⟩ = false := by decide

/-! ## The scan source -/

-- No scan received yet: stop (R8).
example : velocities cfg [cmd (ms 1000) fwd] = [some .stopSource] := by decide

-- The scan's stamp is exactly 2 s in the past: still valid. One nanosecond
-- more: stop (R7). The age is measured from the header stamp, not from the
-- arrival time, which here is recent in both cases.
example : velocities cfg [.scan (ms 2900) (scanAt (ms 1000) [at2]), cmd (ms 3000) fwd,
      cmd (ms 3000 + 1) fwd]
    = [none, some .pass, some .stopSource] := by decide

-- A scan stamped in the future is valid (R7).
example : velocities cfg [.scan (ms 1000) (scanAt (ms 900000) [at2]), cmd (ms 1010) fwd]
    = [none, some .pass] := by decide

-- The scan received last is used, also when its stamp is older (R5): here it
-- is too old, and the robot stops although a fresher scan was received before.
example : velocities cfg [.scan (ms 5000) (scanAt (ms 4990) [at2]),
      .scan (ms 5005) (scanAt (ms 1000) [at2]), cmd (ms 5010) fwd]
    = [none, none, some .stopSource] := by decide

/-- As `cfg` with the timeout disabled. -/
def cfgNoTimeout : Config := { cfg with sourceTimeout := 0 }

-- With `source_timeout: 0` a missing scan is "no obstacles" (R8), and an old
-- scan is used however old it is (R7).
example : velocities cfgNoTimeout [cmd (ms 1000) fwd,
      .scan (ms 1001) (scanAt (ms 990) [at0_25, left0_25]), cmd (ms 900000) fwd]
    = [some .pass, none, some (.stopZone 0)] := by decide

/-- As `cfg` with the source disabled. -/
def cfgSourceOff : Config := { cfg with sourceEnabled := false }

-- A disabled source is neither required nor used (R6).
example : velocities cfgSourceOff [cmd (ms 1000) fwd,
      .scan (ms 1001) (scanAt (ms 1001) [at0_25, left0_25]), cmd (ms 1010) fwd]
    = [some .pass, none, some .pass] := by decide

/-! ## Invalid readings (R4) -/

-- Readings below `range_min`, above `range_max`, negative, infinite or NaN are
-- skipped one by one; their points do not count, whatever coordinates the
-- command carries for them.
set_option maxRecDepth 4000 in
example : (scanAt 0 [⟨r0_05, d0_5, d0⟩, ⟨r30, d0, d0⟩, ⟨rNeg1, d0, d0⟩, ⟨rInf, d0, d0⟩,
      ⟨rNaN, d0, d0⟩, at0_25]).points = [⟨250, 0⟩] := by decide

-- The limits themselves are valid.
example : (scanAt 0 [⟨r0_1, d0_2, d0⟩, ⟨r10, d10, d0⟩]).points
    = [⟨200, 0⟩, ⟨10000, 0⟩] := by decide

-- A scan in which no reading is valid is a scan without obstacles: the command
-- passes. This is upstream's rule, and it is fail-open (SPEC, "Open
-- decisions").
example : velocities cfg [.scan (ms 1000) (scanAt (ms 990) [⟨rNaN, d0, d0⟩, ⟨rNaN, d0, d0⟩]),
      cmd (ms 1010) fwd]
    = [none, some .pass] := by decide

-- So is an empty scan, and one whose limits are NaN.
example : velocities cfg [.scan (ms 1000) (scanAt (ms 990) []), cmd (ms 1010) fwd]
    = [none, some .pass] := by decide
example : velocities cfg [.scan (ms 1000) ⟨ms 990, rNaN, r10, [at0_25, left0_25]⟩,
      cmd (ms 1010) fwd]
    = [none, some .pass] := by decide

-- With `range_max` infinite an infinite reading is valid. Its coordinate is
-- not finite and is saturated at the coordinate limit, as is a NaN and a
-- finite coordinate of 1000 m or more; such a point lies in no accepted zone.
example : (Sweep.mk 0 r0_1 rInf [⟨rInf, dInf, d0⟩]).points = [⟨1000000, 0⟩] := by decide
example : scanPointMm (dNaN, d0) = ⟨1000000, 0⟩ := by decide
example : scanPointMm (0x40F86A0000000000, 0xC0F86A0000000000) = ⟨1000000, -1000000⟩ := by
  decide
example : (Shape.circle coordLimit).contains ⟨1000000, 0⟩ = false := by decide

-- **The number of points is the number of valid readings** (`points_length`):
-- the shim hands over a pair for every reading; which of them count is
-- decided from the stored ranges.
example : (scanAt 0 [at0_25, ⟨rNaN, d0_25, d0⟩, ⟨r30, d0_25, d0⟩, left0_25]).points.length = 2 := by
  decide

/-! ## Commands -/

-- A command with an infinite or NaN component is ignored: nothing is
-- published and the state does not change (R2). This includes the three
-- components the monitor does not otherwise use.
example : step cfg (active cfg) (.cmd (ms 1000) (twist dNaN d0) none [])
    = (active cfg, .silent) := by decide
example : step cfg (active cfg) (.cmd (ms 1000) ⟨d0_5, 0, dInf, 0, 0, 0⟩ none [])
    = (active cfg, .silent) := by decide

/-! ## Zero velocity is published for `stop_pub_timeout`, then not at all (R15) -/

/-- Two points inside the stop zone, stamped far in the future so that the
scan stays valid throughout. -/
def blocked : Ev := .scan 0 (scanAt (ms 900000) [at0_25, left0_25])

/-- Nothing near, likewise. -/
def clear : Ev := .scan 0 (scanAt (ms 900000) [at2])

-- Stop is published from the first stopped command (at 1000 ms) up to and
-- including 1 s later, and no longer after that. When the zone clears, the
-- command passes again, and the next stop starts a new second.
example : velocities cfg [blocked, cmd (ms 1000) fwd, cmd (ms 1500) fwd, cmd (ms 2000) fwd,
      cmd (ms 2001) fwd, cmd (ms 9000) fwd, clear, cmd (ms 9100) fwd, blocked,
      cmd (ms 9200) fwd, cmd (ms 10200) fwd, cmd (ms 10201) fwd]
    = [none, some (.stopZone 0), some (.stopZone 0), some (.stopZone 0),
       none, none, none, some .pass, none,
       some (.stopZone 0), some (.stopZone 0), none] := by decide

-- The same holds for a command that is itself zero: it passes for 1 s and is
-- then dropped. `-0.0` counts as zero.
example : velocities cfg [clear, cmd (ms 1000) halt, cmd (ms 2000) (twist dNeg0 d0),
      cmd (ms 2001) halt, cmd (ms 2002) fwd, cmd (ms 2003) halt]
    = [none, some .pass, some .pass, none, some .pass, some .pass] := by
  decide

-- A zero command after a stop continues the stop's period: the scan is cleared before
-- the zero command, so at 1900 ms no stop is demanded any more (the output is `pass`),
-- yet the zero at 2001 ms is dropped, 1 s after the stop at 1000 ms.
example : velocities cfg [blocked, cmd (ms 1000) fwd, clear, cmd (ms 1900) halt,
      cmd (ms 2001) halt]
    = [none, some (.stopZone 0), none, some .pass, none] := by decide

-- A stop that is never published (review finding 5). The robot idles: zero
-- commands for longer than 1 s, so the node no longer publishes a velocity.
-- Then an obstacle enters the stop zone and the planner commands 0.5 m/s. A
-- stop is demanded and no velocity is published, because the idle zeros and
-- the stop are one period of zero. The last velocity the base received is the
-- zero at 1000 ms (`run_withheld_last_published_zero`). Upstream does the
-- same. (The state message does say "stop": see "The report" below.)
example : velocities cfg [clear, cmd (ms 1000) halt, cmd (ms 2500) halt, blocked,
      cmd (ms 2600) fwd, cmd (ms 2700) fwd]
    = [none, some .pass, none, none, none, none] := by decide

-- A command that turns on the spot is not zero.
example : (twist d0 d0_2).planarZero = false := by decide

/-! ## Slowdown selection (R13) -/

/-- Three slowdown zones of the same shape with ratios 0.7, 0.3 and 0.3. -/
def cfgSlow : Config :=
  { cfg with zones := [⟨.fixed (.circle 600), .slowdown d0_7, 1, true⟩,
      ⟨.fixed (.circle 600), .slowdown d0_3, 1, true⟩, ⟨.fixed (.circle 600), .slowdown d0_3, 1, true⟩] }

-- The smallest ratio wins; of two equal ratios, the earlier zone.
example : zonesAction cfgSlow [⟨500, 0⟩] = .slowdown 1 d0_3 := by decide

/-- Slowdown zones whose ratios are 1, 2, -0.5 and NaN. -/
def cfgInert : Config :=
  { cfg with zones := [⟨.fixed (.circle 600), .slowdown d1, 1, true⟩, ⟨.fixed (.circle 600), .slowdown d2, 1, true⟩,
      ⟨.fixed (.circle 600), .slowdown dNeg0_5, 1, true⟩, ⟨.fixed (.circle 600), .slowdown dNaN, 1, true⟩] }

-- None of them is ever selected: the command passes. Upstream agrees for 1, 2
-- and NaN and reverses the robot for -0.5 (D5), which is why the node rejects
-- that configuration.
example : zonesAction cfgInert [⟨500, 0⟩] = .pass := by decide
example : cfgInert.ok = false := by decide

/-- A slowdown zone in front of a stop zone of the same shape. -/
def cfgOrder : Config :=
  { cfg with zones := [⟨.fixed (.circle 600), .slowdown d0_5, 1, true⟩, ⟨.fixed (.circle 600), .stop, 1, true⟩] }

-- A triggered stop zone wins wherever it is in the list (R12).
example : zonesAction cfgOrder [⟨500, 0⟩] = .stopZone 1 := by decide

-- A slowdown ratio of 0 yields zero velocity, so it is subject to R15.
example : (Action.slowdown 0 d0).yieldsZero fwd = true := by decide

/-! ## Zone parameters (R11) -/

/-- The stop zone disabled; a second stop zone with `min_points: 0`, also
disabled. -/
def cfgOff : Config :=
  { cfg with zones := [⟨.fixed (.circle 300), .stop, 2, false⟩, ⟨.fixed (.circle 300), .stop, 0, false⟩] }

-- A disabled zone never triggers.
example : zonesAction cfgOff [⟨100, 0⟩, ⟨0, 100⟩] = .pass := by decide

-- An enabled zone with `min_points: 0` triggers without any point.
example : zonesAction { cfg with zones := [⟨.fixed (.circle 300), .stop, 0, true⟩] } [] = .stopZone 0 := by
  decide

/-! ## Polygons (R10, D4) -/

/-- Upstream's example stop zone: 0.3 m ahead, 0.3 m to each side. -/
def box : Shape := .polygon [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩]

example : box.contains ⟨150, 0⟩ = true := by decide
example : box.contains ⟨150, -299⟩ = true := by decide
example : box.contains ⟨400, 0⟩ = false := by decide
example : box.contains ⟨-100, 0⟩ = false := by decide

-- Upstream's rule alone treats the sides differently: points on the left side
-- (y = 300) and on the rear side (x = 0) count as inside, points on the front
-- side (x = 300) and on the right side (y = -300) do not.
example : insidePolygon [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩] ⟨150, 300⟩ = true := by
  decide
example : insidePolygon [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩] ⟨300, 0⟩ = false := by
  decide
example : insidePolygon [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩] ⟨150, -300⟩ = false := by
  decide
example : insidePolygon [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩] ⟨0, 0⟩ = true := by
  decide

-- The model adds a band of √5 mm around the boundary (D4): 2 mm outside a side
-- is inside, 3 mm is not; diagonally off a corner, (2, 1) mm is inside and
-- (2, 2) mm is not.
example : box.contains ⟨300, 0⟩ = true := by decide
example : box.contains ⟨302, 0⟩ = true := by decide
example : box.contains ⟨303, 0⟩ = false := by decide
example : box.contains ⟨-2, -100⟩ = true := by decide
example : box.contains ⟨-3, -100⟩ = false := by decide
example : box.contains ⟨302, 301⟩ = true := by decide
example : box.contains ⟨302, 302⟩ = false := by decide

/-- An L-shaped zone: the square from 0 to 400 mm without its upper right
quarter. -/
def ell : Shape :=
  .polygon [⟨0, 0⟩, ⟨400, 0⟩, ⟨400, 200⟩, ⟨200, 200⟩, ⟨200, 400⟩, ⟨0, 400⟩]

example : ell.contains ⟨300, 100⟩ = true := by decide
example : ell.contains ⟨100, 300⟩ = true := by decide
example : ell.contains ⟨300, 300⟩ = false := by decide

/-- The box as a stop zone needing four points, as in upstream's example. -/
def cfgBox : Config := { cfg with zones := [⟨.fixed box, .stop, 4, true⟩] }

-- Three points inside pass; the fourth stops.
example : zonesAction cfgBox [⟨100, 0⟩, ⟨100, 50⟩, ⟨100, -50⟩, ⟨900, 0⟩] = .pass := by decide
example : zonesAction cfgBox [⟨100, 0⟩, ⟨100, 50⟩, ⟨100, -50⟩, ⟨200, 0⟩] = .stopZone 0 := by
  decide

/-! ## Accepted configurations -/

example : cfg.ok = true := by decide
example : cfgBox.ok = true := by decide

-- A triangle is rejected, as upstream rejects it (R10).
example : (Shape.polygon [⟨0, 0⟩, ⟨300, 0⟩, ⟨0, 300⟩]).ok = false := by decide

-- A zone must stay inside the coordinate limit, so that a saturated point is
-- in no zone.
example : (Shape.circle (coordLimit + 1)).ok = false := by decide
example : (Shape.polygon [⟨0, 0⟩, ⟨999998, 0⟩, ⟨999998, 5⟩, ⟨0, 5⟩]).ok = false := by decide
example : (Shape.polygon [⟨0, 0⟩, ⟨999997, 0⟩, ⟨999997, 5⟩, ⟨0, 5⟩]).contains ⟨1000000, 2⟩
    = false := by decide

-- A slowdown ratio of `-0.0` is rejected along with the negative ones (D5).
example : (ZoneAction.slowdown dNeg0).ok = false := by decide

/-! # The Nav2 extension

From here on the outputs are whole: `silent`, or `answer velocity report`
for a processed command. -/

/-- A processed command whose velocity is published, without a state
message. -/
def pub (a : Action) : Output := .answer (some a) none

/-- A processed command whose velocity is published, with a state message. -/
def pubR (a : Action) (r : Report) : Output := .answer (some a) (some r)

/-! ## The lifecycle (E1–E6, N2–N8) -/

-- Nothing is answered before the node is active: a command while
-- unconfigured, a command while inactive. A scan sent while unconfigured
-- reaches nobody (N2); a scan received while inactive is the one used after
-- activation (N4): here it shows two points in the stop zone. Activation
-- itself publishes nothing (N5). The first answer reports the state.
example : outputs cfg [cmd (ms 1) fwd, .scan (ms 2) (scanAt (ms 2) [at2]), .configure,
      cmd (ms 3) fwd, blocked, .activate, cmd (ms 1000) fwd]
    = [.silent, .silent, .silent, .silent, .silent, .silent,
       pubR (.stopZone 0) (.stop 0)] := by decide

-- Deactivation in the middle of a stop (N6). The stop is published from
-- 1000 ms to 2000 ms and withheld at 2001 ms (`answer none none`: the command
-- is processed, nothing is published but the zone polygons). After
-- `deactivate` nothing is answered. After `activate`, with the same scan and
-- one millisecond later, the zero **and the state** are published again, and
-- the zero for another full second.
example : outputs cfg (up ++ [blocked, cmd (ms 1000) fwd, cmd (ms 2000) fwd, cmd (ms 2001) fwd,
      .deactivate, cmd (ms 2002) fwd, .activate, cmd (ms 2003) fwd, cmd (ms 3003) fwd,
      cmd (ms 3004) fwd])
    = [.silent, .silent, .silent, pubR (.stopZone 0) (.stop 0), pub (.stopZone 0),
       .answer none none, .silent, .silent, .silent, pubR (.stopZone 0) (.stop 0),
       pub (.stopZone 0), .answer none none] := by decide

-- Cleanup discards the stored scan (N7): after cleanup, configure and
-- activate, the first command is stopped for an invalid source.
example : outputs cfg (up ++ [clear, cmd (ms 1000) fwd, .deactivate, .cleanup, .configure,
      .activate, cmd (ms 1100) fwd])
    = [.silent, .silent, .silent, pub .pass, .silent, .silent, .silent, .silent,
       pubR .stopSource .source] := by decide

-- After `shutdown` nothing is ever published again (ND4; upstream, shut down
-- from the active state, keeps forwarding commands). No transition leads out
-- of `finalized`.
example : outputs cfg (up ++ [clear, cmd (ms 1000) fwd, .shutdown, cmd (ms 1100) fwd,
      .configure, .activate, cmd (ms 1200) fwd])
    = [.silent, .silent, .silent, pub .pass, .silent, .silent, .silent, .silent, .silent] := by
  decide
example : (final cfg (up ++ [.shutdown, .configure, .activate])).phase = .finalized := by
  decide

-- **Counterexample to "if the phase is not active, no zero is on record"**
-- (SPEC_NAV2, section 4, "Lifecycle invariants", as first written): a node
-- shut down while a stop is in force is finalized with `prevZero` set and
-- the previous report `source`. The invariant holds for the unconfigured and
-- the inactive phase (`Inv.quiet`); here the fields are never read again.
example : final cfg (up ++ [cmd (ms 1000) fwd, .shutdown])
    = { State.init cfg with phase := .finalized, prevZero := true, stopStamp := ms 1000,
                            prevReport := .source } := by decide

-- Transitions the lifecycle does not allow change nothing: `activate` while
-- unconfigured, `cleanup` while active.
example : final cfg [.activate] = State.init cfg := by decide
example : final cfg (up ++ [.cleanup]) = active cfg := by decide

/-! ## Points that arrive with the command (E8, N22–N27) -/

-- The points belong to the command, not to the scan (N22): the same stored
-- scan stops the robot with the coordinates it was given with, and lets it
-- pass when a later command carries the coordinates the shim computes after
-- the sensor frame has moved 0.5 m ahead of the base (0.25 m ahead becomes
-- 0.75 m ahead; 0.25 m to the left becomes 0.5 m ahead and 0.25 m left).
example : outputs cfg (up ++ [blocked, cmd (ms 1000) fwd,
      .cmdPts (ms 1010) fwd [(d0_75, d0), (d0_5, d0_25)]])
    = [.silent, .silent, .silent, pubR (.stopZone 0) (.stop 0), pubR .pass .normal] := by decide

-- A missing transform is not an event: it is a command without points. With
-- a non-zero `source_timeout` it stops the robot for an invalid source (N27),
-- although the stored scan is fresh and shows nothing near; the next command
-- that carries points passes.
example : outputs cfg (up ++ [clear, .cmdNone (ms 1000) fwd, cmd (ms 1010) fwd])
    = [.silent, .silent, .silent, pubR .stopSource .source, pubR .pass .normal] := by decide

-- Coordinates that are not one pair per reading of the stored scan are not
-- used: they count as a missing transform (`wrong_count`). Here the stored
-- scan has two readings and the command carries one pair, then three.
example : outputs cfg (up ++ [blocked, .cmdPts (ms 1000) fwd [(d2, d0)],
      .cmdPts (ms 1010) fwd [(d2, d0), (d2, d0), (d2, d0)], cmd (ms 1020) fwd])
    = [.silent, .silent, .silent, pubR .stopSource .source, .answer (some .stopSource) none,
       pubR (.stopZone 0) (.stop 0)] := by decide

-- **The counterexample to "No transform" without its hypothesis.** With
-- `source_timeout: 0` a missing transform means "no obstacle" (N27): the scan
-- that stops the robot when its points are known lets it pass when the
-- transform is missing. This is upstream's behaviour, kept, and it is
-- fail-open.
example : outputs cfgNoTimeout (up ++ [blocked, cmd (ms 1000) fwd, .cmdNone (ms 1010) fwd])
    = [.silent, .silent, .silent, pubR (.stopZone 0) (.stop 0), pubR .pass .normal] := by decide

/-! ## Zones fed by a topic (E7, N32–N34) -/

/-! Bit patterns of `double` vertex coordinates in the base frame, as the
shim hands them over: a polygon message has `float` coordinates, and 0.3 as
a `float`, widened to `double`, is 0.300000011920929. -/
def w0_3 : Nat := 0x3FD3333340000000
def wNeg0_3 : Nat := 0xBFD3333340000000
def d5 : Nat := 0x4014000000000000
/-- 0.3 as a `float`, minus 1, in `double`: −0.699999988079071. -/
def wNeg0_7 : Nat := 0xBFE6666660000000

/-- The box of 0.3 m ahead and 0.3 m to each side, as the vertices of a
polygon message in the base frame. -/
def boxMsg : List Coords := [(w0_3, w0_3), (w0_3, wNeg0_3), (d0, wNeg0_3), (d0, w0_3)]

/-- A box from 0.3 m to 5 m ahead. -/
def farBoxMsg : List Coords := [(d5, w0_3), (d5, wNeg0_3), (w0_3, wNeg0_3), (w0_3, w0_3)]

/-- The box after the base has driven 1 m forward since the message: 1 m
further back. -/
def boxBehind : List Coords :=
  [(wNeg0_7, w0_3), (wNeg0_7, wNeg0_3), (dNeg1, wNeg0_3), (dNeg1, w0_3)]

-- A vertex is rounded to the nearest millimetre, a half away from zero
-- (0.0005 m is 1 mm, 0.0015 m is 2 mm; their doubles lie just above).
example : vertexPointMm (w0_3, wNeg0_7) = ⟨300, -700⟩ := by decide
example : vertexPointMm (0x3F40624DD2F1A9FC, 0x3F589374BC6A7EFA) = ⟨1, 2⟩ := by decide

-- A vertex coordinate that is not finite, or is 1024 m or more, becomes the
-- coordinate limit, which no shape may reach: the message is refused (ND6).
example : vertexPointMm (dNaN, dInf) = ⟨1000000, 1000000⟩ := by decide
example : vertexPointMm (0x4090000000000000, 0xC090000000000000) = ⟨1000000, -1000000⟩ := by
  decide
example : placeVerts [(d0, d0), (dNaN, d0), (d0, w0_3)] = none := by decide
-- 999.998 m is 999998 mm, 1 mm beyond what a shape may have.
example : placeVerts [(d0, d0), (0x408F3FFBE76C8B44, d0), (d0, w0_3)] = none := by decide

/-- Zone 0 is a stop zone fed by a topic, needing one point. Zone 1 is a
fixed slowdown circle that needs nine points and never triggers here. -/
def cfgTopic : Config :=
  { cfg with zones := [⟨.topic, .stop, 1, true⟩, ⟨.fixed (.circle 600), .slowdown d0_5, 9, true⟩] }

/-- One point 0.2 m ahead. -/
def near : Ev := .scan 0 (scanAt (ms 900000) [at0_2])

/-- One point 0.85 m behind. -/
def behind : Ev := .scan 0 (scanAt (ms 900000) [⟨r2, dNeg0_85, d0⟩])

-- **A stop zone without shape does not stop** (N32, upstream's behaviour,
-- not fail-safe): with a point 0.2 m ahead the command passes. After the
-- first accepted message the zone stops the robot. A replaced shape is in
-- force from the next command on.
example : outputs cfgTopic (up ++ [near, cmd (ms 1000) fwd, .shape 0 (some boxMsg) false,
      cmd (ms 1010) fwd, .shape 0 (some farBoxMsg) false, cmd (ms 1020) fwd])
    = [.silent, .silent, .silent, pub .pass, .silent, pubR (.stopZone 0) (.stop 0), .silent,
       pubR .pass .normal] := by decide

-- The accepted shape: whether its frame is foreign, and the vertices in
-- millimetres.
example : (final cfgTopic (up ++ [.shape 0 (some boxMsg) false])).shapes
    = [some ⟨false, [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩]⟩, none] := by decide

-- **Counterexample to "a shape message changes the state iff it is
-- accepted"** (SPEC_NAV2, section 4, "Shape accepted", as first written): the
-- same message a second time is accepted and changes nothing. Only "only if"
-- holds (`step_shape_changes`, `step_shape_same`).
example : (acceptShape cfgTopic (final cfgTopic (up ++ [.shape 0 (some boxMsg) false])) 0
      (some boxMsg) false).isSome = true := by decide
example : final cfgTopic (up ++ [.shape 0 (some boxMsg) false, .shape 0 (some boxMsg) false])
    = final cfgTopic (up ++ [.shape 0 (some boxMsg) false]) := by decide

-- Refused messages leave the shape in force (N33, ND6): two points; a NaN
-- coordinate (upstream accepts it and the zone goes blind); a frame whose
-- transform is missing, so that the shim has no vertices to hand over. The
-- robot is still stopped by the box.
example : outputs cfgTopic (up ++ [near, .shape 0 (some boxMsg) false,
      .shape 0 (some [(w0_3, w0_3), (d0, d0)]) false,
      .shape 0 (some [(d5, w0_3), (d5, dNaN), (w0_3, wNeg0_3)]) false,
      .shape 0 none true, cmd (ms 1000) fwd])
    = [.silent, .silent, .silent, .silent, .silent, .silent, .silent,
       pubR (.stopZone 0) (.stop 0)] := by decide

-- A triangle is enough for a topic zone (N33), where a fixed polygon needs
-- four vertices (R10).
example : (TopicShape.ofMessage (some [(w0_3, d0), (d0, w0_3), (d0, wNeg0_3)]) false).isSome
    = true := by decide

-- A message is ignored while the node is unconfigured, for a zone that is not
-- a topic zone, and for a zone that does not exist. It is taken while the
-- node is inactive (N4).
example : (final cfgTopic [.shape 0 (some boxMsg) false, .configure,
      .shape 1 (some boxMsg) false, .shape 7 (some boxMsg) false]).shapes = [none, none] := by
  decide
example : ((final cfgTopic [.configure, .shape 0 (some boxMsg) false]).shapes.map
      Option.isSome) = [true, false] := by decide

-- A shape received while inactive is used after activation (N35); cleanup
-- discards it (N7), and the zone is again without shape: the same point
-- passes.
example : outputs cfgTopic [.configure, .shape 0 (some boxMsg) false, near, .activate,
      cmd (ms 1000) fwd, .deactivate, .cleanup, .configure, .activate, near, cmd (ms 1010) fwd]
    = [.silent, .silent, .silent, .silent, pubR (.stopZone 0) (.stop 0), .silent, .silent,
       .silent, .silent, .silent, pub .pass] := by decide

-- **A shape in another frame follows that frame** (N34). The box is received
-- in a frame that coincides with the base: a point 0.2 m ahead stops the
-- robot. Then the base has moved 1 m and no new message has come: the shim
-- hands over the box where it now is, 1 m behind, and the same point passes.
-- This is the footprint that the robot drives out of (N34). A point 0.85 m
-- behind is now inside the zone; and when the zone's transform is missing,
-- the vertices stay where they were last, so it still stops. With the frames
-- coinciding again it passes.
example : outputs cfgTopic (up ++ [near, .shape 0 (some boxMsg) true,
      .cmdZ (ms 1000) fwd (some boxMsg), .cmdZ (ms 1010) fwd (some boxBehind), behind,
      .cmdZ (ms 1020) fwd none, .cmdZ (ms 1030) fwd (some boxMsg)])
    = [.silent, .silent, .silent, .silent, pubR (.stopZone 0) (.stop 0), pubR .pass .normal,
       .silent, pubR (.stopZone 0) (.stop 0), pubR .pass .normal] := by decide

-- The vertices of that zone after the command that carried `boxBehind`.
example : (final cfgTopic (up ++ [near, .shape 0 (some boxMsg) true,
      .cmdZ (ms 1010) fwd (some boxBehind)])).shapes
    = [some ⟨true, [⟨-700, 300⟩, ⟨-700, -300⟩, ⟨-1000, -300⟩, ⟨-1000, 300⟩]⟩, none] := by
  decide

-- A shape received in the base frame is not moved by anything a command
-- carries.
example : (final cfgTopic (up ++ [near, .shape 0 (some boxMsg) false,
      .cmdZ (ms 1010) fwd (some boxBehind)])).shapes
    = [some ⟨false, [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩]⟩, none] := by decide

-- Vertices that are refused leave the shape where it was: a different number
-- of vertices than the shape has (here a triangle for a box), or a vertex
-- that is not finite.
example : (final cfgTopic (up ++ [near, .shape 0 (some boxMsg) true,
      .cmdZ (ms 1010) fwd (some [(w0_3, d0), (d0, w0_3), (d0, wNeg0_3)]),
      .cmdZ (ms 1020) fwd (some [(dNaN, w0_3), (wNeg0_7, wNeg0_3), (dNeg1, wNeg0_3),
        (dNeg1, w0_3)])])).shapes
    = [some ⟨true, [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩]⟩, none] := by decide

/-! ## The report (N13, N14) -/

/-- Two points inside the slowdown zone and outside the stop zone. -/
def slow : Ev := .scan 0 (scanAt (ms 900000) [at0_5, at0_5])

-- No message for the first command if it passes; one for each change, none
-- while the report stays the same; a zero command in a triggered slowdown
-- zone reports `normal` (N14); a report is made **also when the velocity is
-- withheld** (at 2031 ms no change, at 2040 ms `normal` with no velocity).
example : outputs cfg (up ++ [clear, cmd (ms 1000) fwd, slow, cmd (ms 1010) fwd,
      cmd (ms 1020) fwd, cmd (ms 1030) halt, blocked, cmd (ms 1040) fwd, cmd (ms 2031) fwd,
      clear, cmd (ms 2040) halt, cmd (ms 2050) fwd])
    = [.silent, .silent, .silent, pub .pass, .silent,
       pubR (.slowdown 1 d0_5) (.slowdown 1), pub (.slowdown 1 d0_5),
       pubR (.slowdown 1 d0_5) .normal, .silent, pubR (.stopZone 0) (.stop 0),
       .answer none none, .silent, .answer none (some .normal), pub .pass] := by decide

-- The stop that is never published as a velocity (above) **is** reported:
-- after idle zeros, an obstacle in the stop zone gives `answer none (some
-- (stop 0))`: no velocity message, one state message.
example : outputs cfg (up ++ [clear, cmd (ms 1000) halt, cmd (ms 2001) halt, blocked,
      cmd (ms 2002) fwd, cmd (ms 2003) fwd])
    = [.silent, .silent, .silent, pub .pass, .answer none none, .silent,
       .answer none (some (.stop 0)), .answer none none] := by decide

end ROSlean.CollisionMonitor.Examples
