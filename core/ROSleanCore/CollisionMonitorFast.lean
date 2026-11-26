import ROSleanCore.CollisionMonitor

/-!
# Collision monitor: the command path, written to run fast

`ROSleanCore.CollisionMonitor.step` is written to be read and reasoned
about. Run as it stands, a command costs far more than it has to
(`docs/collision_monitor/COMPARISON.md`, defect 2): the coordinates are
lists of pairs of `Nat` patterns, of which every negative one is a big
number; the points pass through three lists (`zip`, `filter`, `map`), twice
(`evaluate` and `shapesAfter` each ask for them); and the polygon test
builds the list of edges anew, twice, for every point.

This file holds a second way to compute **the same function**. Nothing here
is a definition that a theorem of `ROSlean/CollisionMonitor/` is stated on;
each function comes with a theorem, in `ROSlean/CollisionMonitor/Fast.lean`,
that it equals the definition it replaces:

| Here | Equals | Theorem |
| --- | --- | --- |
| `Scan.pointsBy` | `Scan.points` | `pointsBy_toList` |
| `sourcePointsBy` | `sourcePoints` | `sourcePointsBy_eq` |
| `Shape.countIn`, `Zone.countFast` | `Zone.count` | `countFast_eq` |
| `Zone.triggeredFast` | `Zone.triggered` | `triggeredFast_eq` |
| `zonesActionFast` | `zonesAction` | `zonesActionFast_eq` |
| `stepCmdFast` | `step` on a command | `stepCmdFast_eq` |

The C interface calls `stepCmdFast`. What makes it fast:

* the caller hands over a function from a reading's index to its point,
  which decodes the coordinate from its bytes without building the 64-bit
  pattern (`ROSleanCore.CollisionMonitorFFI.pointAt`);
* the points are produced once, in one pass, into an array, and only if the
  command is processed and the scan is used;
* for a polygon the edges are computed once per zone and command, and the
  test of the band around the boundary (D4) is made only for a point within
  2 mm of the polygon's bounding box: no other point can be within √5 mm of
  an edge (`nearEdges_eq_false_of_not_near`).

No Mathlib here.
-/

namespace ROSleanCore.CollisionMonitor

/-! ## Scan points -/

/-- The loop of `Scan.pointsBy`: the readings left, the index of the first of
them, the points so far. -/
def Scan.pointsGo (s : Scan) (f : Nat → Point) : List Beam → Nat → Array Point → Array Point
  | [], _, acc => acc
  | b :: bs, i, acc => Scan.pointsGo s f bs (i + 1) (if s.validBeam b then acc.push (f i) else acc)

/-- The points of the scan's valid readings, in scan order, where `f i` is
the point of reading `i`. One pass; the function is not called for a reading
that is not valid (R4). -/
def Scan.pointsBy (s : Scan) (f : Nat → Point) : Array Point :=
  s.pointsGo f s.beams 0 (Array.mkEmpty s.beams.length)

/-- `sourcePoints` for coordinates given as their number `n` (`none`: there
are none) and the function `f` from a reading's index to its point. -/
def sourcePointsBy (cfg : Config) (st : State) (now : Nat) (n : Option Nat) (f : Nat → Point) :
    Option (Array Point) :=
  if !cfg.sourceEnabled then some #[]
  else
    match st.scan with
    | none => if cfg.sourceTimeout = 0 then some #[] else none
    | some s =>
      if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout < now then none
      else
        match n with
        | some n =>
          if n == s.beams.length then some (s.pointsBy f)
          else if cfg.sourceTimeout = 0 then some #[] else none
        | none => if cfg.sourceTimeout = 0 then some #[] else none

/-! ## Zones -/

/-- An axis-parallel box, in millimetres. -/
structure Box where
  minX : Int
  maxX : Int
  minY : Int
  maxY : Int

/-- The smallest box that holds the vertices (anything for no vertices). -/
def boxOf : List Point → Box
  | [] => ⟨0, 0, 0, 0⟩
  | v :: vs =>
    vs.foldl (fun b p => ⟨min b.minX p.x, max b.maxX p.x, min b.minY p.y, max b.maxY p.y⟩)
      ⟨v.x, v.x, v.y, v.y⟩

/-- The point is in the box or within 2 mm of it, on each axis. A point
within `√polygonMarginSq` mm of a segment between two points of the box has
this property, since 3² > 5. -/
def Box.near (b : Box) (p : Point) : Bool :=
  decide (b.minX - 2 ≤ p.x) && decide (p.x ≤ b.maxX + 2) &&
    decide (b.minY - 2 ≤ p.y) && decide (p.y ≤ b.maxY + 2)

/-- `Shape.contains` for a polygon whose edges `es` and bounding box `bb`
have been computed. -/
def containsEdges (es : List (Point × Point)) (bb : Box) (p : Point) : Bool :=
  (es.countP (fun e => crosses p e.1 e.2) % 2 == 1) ||
    (bb.near p && es.any (fun e => nearSegment p e.1 e.2))

/-- The number of the given points inside the shape. What depends on the
shape alone is computed once, not once per point. -/
def Shape.countIn : Shape → Array Point → Nat
  | .circle r, pts =>
    let rr : Int := ((r * r : Nat) : Int)
    pts.countP (fun p => decide (p.x * p.x + p.y * p.y < rr))
  | .polygon vs, pts =>
    let es := edges vs
    let bb := boxOf vs
    pts.countP (containsEdges es bb)

/-- `Zone.count` on an array of points. -/
def Zone.countFast (z : Zone) (pts : Array Point) : Nat :=
  match z.shape with
  | .fixed s => s.countIn pts
  | .topic => 0

/-- `Zone.triggered` on an array of points. -/
def Zone.triggeredFast (z : Zone) (pts : Array Point) : Bool :=
  z.enabled && z.shape.isSet && decide (z.minPoints ≤ z.countFast pts)

/-- `firstStop` on an array of points. -/
def firstStopFast (cfg : Config) (pts : Array Point) : Option Nat :=
  (cfg.zones.zipIdx.find? (fun zi => zi.1.isStop && zi.1.triggeredFast pts)).map (·.2)

/-- `slowdownStep` on an array of points. -/
def slowdownStepFast (pts : Array Point) (best : Nat × Option Nat) (zi : Zone × Nat) :
    Nat × Option Nat :=
  match zi.1.action with
  | .stop => best
  | .slowdown r => if zi.1.triggeredFast pts && decide (r < best.1) then (r, some zi.2) else best

/-- `bestSlowdown` on an array of points. -/
def bestSlowdownFast (cfg : Config) (pts : Array Point) : Nat × Option Nat :=
  cfg.zones.zipIdx.foldl (slowdownStepFast pts) (F64.one, none)

/-- `zonesAction` on an array of points. -/
def zonesActionFast (cfg : Config) (pts : Array Point) : Action :=
  match firstStopFast cfg pts with
  | some k => .stopZone k
  | none =>
    match bestSlowdownFast cfg pts with
    | (r, some k) => .slowdown k r
    | (_, none) => .pass

/-! ## A command -/

/-- `step` for a command (E8) whose scan coordinates are given as their
number `n` (`none`: there are none) and a function `f` from a reading's
index to its point in millimetres. If `f i = scanPointMm (g i)` for all `i`,
this is `step cfg st (.cmd now v (n.map fun n => (List.range n).map g)
zoneVerts)` (`stepCmdFast_eq`). -/
def stepCmdFast (cfg : Config) (st : State) (now : Nat) (v : Twist) (n : Option Nat)
    (f : Nat → Point) (zoneVerts : List (Option (List Coords))) : State × Output :=
  if st.phase ≠ .active then (st, .silent)
  else if !v.finite then (st, .silent)
  else
    let src := sourcePointsBy cfg st now n f
    let sh := refreshShapes cfg st.shapes zoneVerts
    let a :=
      match src with
      | none => Action.stopSource
      | some pts => zonesActionFast (cfg.withShapes sh) pts
    let zero := a.yieldsZero v
    let r := a.report v
    ({ st with
        shapes := (match src with
          | none => st.shapes
          | some _ => sh)
        prevZero := zero
        stopStamp := if zero && !st.prevZero then now else st.stopStamp
        prevReport := r },
      .answer
        (if zero && st.prevZero && decide (st.stopStamp + cfg.stopPubTimeout < now) then none
          else some a)
        (if r = st.prevReport then none else some r))

end ROSleanCore.CollisionMonitor
