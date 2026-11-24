import ROSleanCore.CollisionMonitor

/-!
# Worked examples for the collision monitor model

Each example is checked by the kernel (`by decide`). Rule numbers refer to
`docs/collision_monitor/SPEC.md`.

Times are nanoseconds, written with `ms`. Points are millimetres. Floating
point values appear as IEEE-754 bit patterns; the ones used here are named
below.
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
def d0_5 : Nat := 0x3FE0000000000000
def d0_7 : Nat := 0x3FE6666666666666
def d1 : Nat := F64.one
def d2 : Nat := 0x4000000000000000
def dNeg0_5 : Nat := 0xBFE0000000000000
def dInf : Nat := 0x7FF0000000000000
def dNaN : Nat := 0x7FF8000000000000

/-- A command with the given `linear.x` and `angular.z`. -/
def twist (lin ang : Nat) : Twist := ⟨lin, 0, 0, 0, 0, ang⟩

/-- Forward at 0.5 m/s, turning at 0.2 rad/s. -/
def fwd : Twist := twist d0_5 d0_2

/-- All components zero. -/
def halt : Twist := twist d0 d0

/-- A scan with range limits 0.1 m to 10 m. -/
def scanAt (stamp : Nat) (beams : List Beam) : Scan :=
  { stamp, rangeMin := r0_1, rangeMax := r10, beams }

/-- A reading straight ahead at 0.2 m, 0.25 m, 0.5 m and 2 m, and one at
0.25 m to the left. -/
def at0_2 : Beam := ⟨r0_2, ⟨200, 0⟩⟩
def at0_25 : Beam := ⟨r0_25, ⟨250, 0⟩⟩
def at0_5 : Beam := ⟨r0_5, ⟨500, 0⟩⟩
def at2 : Beam := ⟨r2, ⟨2000, 0⟩⟩
def left0_25 : Beam := ⟨r0_25, ⟨0, 250⟩⟩

/-- Zone 0: stop inside 0.3 m. Zone 1: slow down to half inside 0.6 m. Each
needs two points. The scan must be at most 2 s old; zero velocity is published
for 1 s. -/
def cfg : Config :=
  { zones := [⟨.circle 300, .stop, 2, true⟩, ⟨.circle 600, .slowdown d0_5, 2, true⟩]
    sourceEnabled := true
    sourceTimeout := ms 2000
    stopPubTimeout := ms 1000 }

def outputs (c : Config) (es : List Event) : List Output := (run c State.init es).2

/-! ## The three outcomes -/

-- Nothing near: the command passes (R14).
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [at2, at2, at2]), .cmd (ms 1010) fwd]
    = [.silent, .publish .pass] := by decide

-- Two points inside 0.3 m: stop, and the stop zone is named (R12).
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [at0_25, left0_25, at2]),
      .cmd (ms 1010) fwd]
    = [.silent, .publish (.stopZone 0)] := by decide

-- One point inside 0.3 m is below `min_points`; with a second inside 0.6 m the
-- slowdown zone triggers and hands back its ratio (R11, R13).
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [at0_25, at0_5, at2]),
      .cmd (ms 1010) fwd]
    = [.silent, .publish (.slowdown 1 d0_5)] := by decide

-- The circle test is strict: a point at exactly the radius is outside (R9).
example : (Shape.circle 300).contains ⟨300, 0⟩ = false := by decide
example : (Shape.circle 300).contains ⟨299, 0⟩ = true := by decide
example : (Shape.circle 300).contains ⟨-180, 239⟩ = true := by decide
example : (Shape.circle 300).contains ⟨-180, 240⟩ = false := by decide

/-! ## The scan source -/

-- No scan received yet: stop (R8).
example : outputs cfg [.cmd (ms 1000) fwd] = [.publish .stopSource] := by decide

-- The scan's stamp is exactly 2 s in the past: still valid. One nanosecond
-- more: stop (R7). The age is measured from the header stamp, not from the
-- arrival time, which here is recent in both cases.
example : outputs cfg [.scan (ms 2900) (scanAt (ms 1000) [at2]), .cmd (ms 3000) fwd,
      .cmd (ms 3000 + 1) fwd]
    = [.silent, .publish .pass, .publish .stopSource] := by decide

-- A scan stamped in the future is valid (R7).
example : outputs cfg [.scan (ms 1000) (scanAt (ms 900000) [at2]), .cmd (ms 1010) fwd]
    = [.silent, .publish .pass] := by decide

-- The scan received last is used, also when its stamp is older (R5): here it
-- is too old, and the robot stops although a fresher scan was received before.
example : outputs cfg [.scan (ms 5000) (scanAt (ms 4990) [at2]),
      .scan (ms 5005) (scanAt (ms 1000) [at2]), .cmd (ms 5010) fwd]
    = [.silent, .silent, .publish .stopSource] := by decide

-- An unusable scan (D11: not in the base frame, and no sensor pose) discards
-- the stored scan: the robot stops as if no scan had ever arrived, although
-- the scan before it was fresh and showed nothing near. The next usable scan
-- ends that.
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [at2]), .cmd (ms 1010) fwd,
      .scanUnusable (ms 1020), .cmd (ms 1030) fwd, .cmd (ms 1040) fwd,
      .scan (ms 1050) (scanAt (ms 1045) [at2]), .cmd (ms 1060) fwd]
    = [.silent, .publish .pass, .silent, .publish .stopSource, .publish .stopSource,
       .silent, .publish .pass] := by decide

-- An unusable scan also discards a scan that would have stopped the robot by
-- a zone; the stop is then by the source rule.
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [at0_25, left0_25]),
      .cmd (ms 1010) fwd, .scanUnusable (ms 1020), .cmd (ms 1030) fwd]
    = [.silent, .publish (.stopZone 0), .silent, .publish .stopSource] := by decide

/-- As `cfg` with the timeout disabled. -/
def cfgNoTimeout : Config := { cfg with sourceTimeout := 0 }

-- With `source_timeout: 0` a missing scan is "no obstacles" (R8), and an old
-- scan is used however old it is (R7).
example : outputs cfgNoTimeout [.cmd (ms 1000) fwd,
      .scan (ms 1001) (scanAt (ms 990) [at0_25, left0_25]), .cmd (ms 900000) fwd]
    = [.publish .pass, .silent, .publish (.stopZone 0)] := by decide

-- With `source_timeout: 0` an unusable scan leaves "no obstacles" (R8): the
-- zone that the discarded scan triggered no longer stops the robot.
example : outputs cfgNoTimeout [.scan (ms 1000) (scanAt (ms 990) [at0_25, left0_25]),
      .cmd (ms 1010) fwd, .scanUnusable (ms 1020), .cmd (ms 1030) fwd]
    = [.silent, .publish (.stopZone 0), .silent, .publish .pass] := by decide

/-- As `cfg` with the source disabled. -/
def cfgSourceOff : Config := { cfg with sourceEnabled := false }

-- A disabled source is neither required nor used (R6).
example : outputs cfgSourceOff [.cmd (ms 1000) fwd,
      .scan (ms 1001) (scanAt (ms 1001) [at0_25, left0_25]), .cmd (ms 1010) fwd]
    = [.publish .pass, .silent, .publish .pass] := by decide

/-! ## Invalid readings (R4) -/

-- Readings below `range_min`, above `range_max`, negative, infinite or NaN are
-- skipped one by one; their points do not count.
example : (scanAt 0 [⟨r0_05, ⟨50, 0⟩⟩, ⟨r30, ⟨0, 0⟩⟩, ⟨rNeg1, ⟨0, 0⟩⟩, ⟨rInf, ⟨0, 0⟩⟩,
      ⟨rNaN, ⟨0, 0⟩⟩, at0_25]).points = [⟨250, 0⟩] := by decide

-- The limits themselves are valid.
example : (scanAt 0 [⟨r0_1, ⟨100, 0⟩⟩, ⟨r10, ⟨10000, 0⟩⟩]).points
    = [⟨100, 0⟩, ⟨10000, 0⟩] := by decide

-- A scan in which no reading is valid is a scan without obstacles: the command
-- passes. This is upstream's rule, and it is fail-open (SPEC, "Open
-- decisions").
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) [⟨rNaN, ⟨0, 0⟩⟩, ⟨rNaN, ⟨0, 0⟩⟩]),
      .cmd (ms 1010) fwd]
    = [.silent, .publish .pass] := by decide

-- So is an empty scan, and one whose limits are NaN.
example : outputs cfg [.scan (ms 1000) (scanAt (ms 990) []), .cmd (ms 1010) fwd]
    = [.silent, .publish .pass] := by decide
example : outputs cfg [.scan (ms 1000) ⟨ms 990, rNaN, r10, [at0_25, left0_25]⟩,
      .cmd (ms 1010) fwd]
    = [.silent, .publish .pass] := by decide

-- With `range_max` infinite an infinite reading is valid; its point is
-- saturated by the shim and lies in no accepted zone.
example : (Scan.mk 0 r0_1 rInf [⟨rInf, ⟨1000000, 0⟩⟩]).points = [⟨1000000, 0⟩] := by decide
example : (Shape.circle coordLimit).contains ⟨1000000, 0⟩ = false := by decide

/-! ## Commands -/

-- A command with an infinite or NaN component is ignored: nothing is
-- published and the state does not change (R2). This includes the three
-- components the monitor does not otherwise use.
example : step cfg State.init (.cmd (ms 1000) (twist dNaN d0)) = (State.init, .silent) := by
  decide
example : step cfg State.init (.cmd (ms 1000) ⟨d0_5, 0, dInf, 0, 0, 0⟩)
    = (State.init, .silent) := by decide

/-! ## Zero velocity is published for `stop_pub_timeout`, then not at all (R15) -/

/-- Two points inside the stop zone, stamped far in the future so that the
scan stays valid throughout. -/
def blocked : Event := .scan 0 (scanAt (ms 900000) [at0_25, left0_25])

/-- Nothing near, likewise. -/
def clear : Event := .scan 0 (scanAt (ms 900000) [at2])

-- Stop is published from the first stopped command (at 1000 ms) up to and
-- including 1 s later, and no longer after that. When the zone clears, the
-- command passes again, and the next stop starts a new second.
example : outputs cfg [blocked, .cmd (ms 1000) fwd, .cmd (ms 1500) fwd, .cmd (ms 2000) fwd,
      .cmd (ms 2001) fwd, .cmd (ms 9000) fwd, clear, .cmd (ms 9100) fwd, blocked,
      .cmd (ms 9200) fwd, .cmd (ms 10200) fwd, .cmd (ms 10201) fwd]
    = [.silent, .publish (.stopZone 0), .publish (.stopZone 0), .publish (.stopZone 0),
       .silent, .silent, .silent, .publish .pass, .silent,
       .publish (.stopZone 0), .publish (.stopZone 0), .silent] := by decide

-- The same holds for a command that is itself zero: it passes for 1 s and is
-- then dropped. `-0.0` counts as zero.
example : outputs cfg [clear, .cmd (ms 1000) halt, .cmd (ms 2000) (twist dNeg0 d0),
      .cmd (ms 2001) halt, .cmd (ms 2002) fwd, .cmd (ms 2003) halt]
    = [.silent, .publish .pass, .publish .pass, .silent, .publish .pass, .publish .pass] := by
  decide

-- A zero command after a stop continues the stop's period: the scan is cleared before
-- the zero command, so at 1900 ms no stop is demanded any more (the output is `pass`),
-- yet the zero at 2001 ms is dropped, 1 s after the stop at 1000 ms.
example : outputs cfg [blocked, .cmd (ms 1000) fwd, clear, .cmd (ms 1900) halt,
      .cmd (ms 2001) halt]
    = [.silent, .publish (.stopZone 0), .silent, .publish .pass, .silent] := by decide

-- A stop that is never published (review finding 5). The robot idles: zero
-- commands for longer than 1 s, so the node has gone silent. Then an obstacle
-- enters the stop zone and the planner commands 0.5 m/s. A stop is demanded
-- and nothing is published, because the idle zeros and the stop are one period
-- of zero. The last message the base received is the zero at 1000 ms
-- (`run_silent_last_published_zero`). Upstream does the same.
example : outputs cfg [clear, .cmd (ms 1000) halt, .cmd (ms 2500) halt, blocked,
      .cmd (ms 2600) fwd, .cmd (ms 2700) fwd]
    = [.silent, .publish .pass, .silent, .silent, .silent, .silent] := by decide

-- A command that turns on the spot is not zero.
example : (twist d0 d0_2).planarZero = false := by decide

/-! ## Slowdown selection (R13) -/

/-- Three slowdown zones of the same shape with ratios 0.7, 0.3 and 0.3. -/
def cfgSlow : Config :=
  { cfg with zones := [⟨.circle 600, .slowdown d0_7, 1, true⟩,
      ⟨.circle 600, .slowdown d0_3, 1, true⟩, ⟨.circle 600, .slowdown d0_3, 1, true⟩] }

-- The smallest ratio wins; of two equal ratios, the earlier zone.
example : zonesAction cfgSlow [⟨500, 0⟩] = .slowdown 1 d0_3 := by decide

/-- Slowdown zones whose ratios are 1, 2, -0.5 and NaN. -/
def cfgInert : Config :=
  { cfg with zones := [⟨.circle 600, .slowdown d1, 1, true⟩, ⟨.circle 600, .slowdown d2, 1, true⟩,
      ⟨.circle 600, .slowdown dNeg0_5, 1, true⟩, ⟨.circle 600, .slowdown dNaN, 1, true⟩] }

-- None of them is ever selected: the command passes. Upstream agrees for 1, 2
-- and NaN and reverses the robot for -0.5 (D5), which is why the node rejects
-- that configuration.
example : zonesAction cfgInert [⟨500, 0⟩] = .pass := by decide
example : cfgInert.ok = false := by decide

/-- A slowdown zone in front of a stop zone of the same shape. -/
def cfgOrder : Config :=
  { cfg with zones := [⟨.circle 600, .slowdown d0_5, 1, true⟩, ⟨.circle 600, .stop, 1, true⟩] }

-- A triggered stop zone wins wherever it is in the list (R12).
example : zonesAction cfgOrder [⟨500, 0⟩] = .stopZone 1 := by decide

-- A slowdown ratio of 0 yields zero velocity, so it is subject to R15.
example : (Action.slowdown 0 d0).yieldsZero fwd = true := by decide

/-! ## Zone parameters (R11) -/

/-- The stop zone disabled; a second stop zone with `min_points: 0`, also
disabled. -/
def cfgOff : Config :=
  { cfg with zones := [⟨.circle 300, .stop, 2, false⟩, ⟨.circle 300, .stop, 0, false⟩] }

-- A disabled zone never triggers.
example : zonesAction cfgOff [⟨100, 0⟩, ⟨0, 100⟩] = .pass := by decide

-- An enabled zone with `min_points: 0` triggers without any point.
example : zonesAction { cfg with zones := [⟨.circle 300, .stop, 0, true⟩] } [] = .stopZone 0 := by
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
def cfgBox : Config := { cfg with zones := [⟨box, .stop, 4, true⟩] }

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

end ROSlean.CollisionMonitor.Examples
