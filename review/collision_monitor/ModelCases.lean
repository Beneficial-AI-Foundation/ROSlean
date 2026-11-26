import ROSleanCore.CollisionMonitor

/-!
# Review cases for the collision monitor model (CM1/CM2)

Each `example` is a fact about `core/ROSleanCore/CollisionMonitor.lean` that a
finding in `docs/REVIEWS.md` ("CM1/CM2") rests on, checked by the kernel.
None of them is a defect of the model against the SPEC's rules; they show
behaviour that has no example, or that a property or a sentence of the SPEC
does not cover.

Check from `core/`:

    lake env lean ../review/collision_monitor/ModelCases.lean
-/

namespace Review.ModelCases

open ROSleanCore.CollisionMonitor

def ms (n : Nat) : Nat := n * 1000000

def r0_1 : Nat := 0x3DCCCCCD
def r0_25 : Nat := 0x3E800000
def r2 : Nat := 0x40000000
def r10 : Nat := 0x41200000
def rNaN : Nat := 0x7FC00000
def d0_5 : Nat := 0x3FE0000000000000
def d0_2 : Nat := 0x3FC999999999999A

def fwd : Twist := ⟨d0_5, 0, 0, 0, 0, d0_2⟩
def halt : Twist := ⟨0, 0, 0, 0, 0, 0⟩

def scanAt (stamp : Nat) (beams : List Beam) : Scan :=
  { stamp, rangeMin := r0_1, rangeMax := r10, beams }

def near1 : Beam := ⟨r0_25, ⟨250, 0⟩⟩
def near2 : Beam := ⟨r0_25, ⟨0, 250⟩⟩
def far : Beam := ⟨r2, ⟨2000, 0⟩⟩

/-- Stop inside 0.3 m, half speed inside 0.6 m, two points each; scan at most
2 s old; zero published for 1 s. The configuration of `Examples.lean`. -/
def cfg : Config :=
  { zones := [⟨.circle 300, .stop, 2, true⟩, ⟨.circle 600, .slowdown d0_5, 2, true⟩]
    sourceEnabled := true
    sourceTimeout := ms 2000
    stopPubTimeout := ms 1000 }

def outputs (c : Config) (es : List Event) : List Output := (run c State.init es).2

/-! ## Finding 5: a stop that is never published

The robot is idle: zero commands for longer than `stop_pub_timeout`, so the
node has gone silent. Then an obstacle enters the stop zone and the planner
starts to command 0.5 m/s. The stop is demanded and nothing is published, not
even once: the zero period of the idle commands is continued. Upstream does
the same (`upstream_probe.py`, case `idle_then_stop`). The "Stop" property
allows it ("or `.silent`") without saying when. -/
example : outputs cfg [.scan 0 (scanAt (ms 900000) [far]), .cmd (ms 1000) halt,
      .cmd (ms 2500) halt, .scan (ms 2550) (scanAt (ms 900000) [near1, near2]),
      .cmd (ms 2600) fwd, .cmd (ms 2700) fwd]
    = [.silent, .publish .pass, .silent, .silent, .silent, .silent] := by decide

/-! ## Finding 9: a negative stamp is not "too old under any non-zero timeout"

D8 turns a negative stamp into 0. While the node clock is below the timeout
(simulated time shortly after start), stamp 0 is fresh and the scan is used. -/
example : outputs cfg [.scan (ms 100) (scanAt 0 [far]), .cmd (ms 1500) fwd]
    = [.silent, .publish .pass] := by decide

/-! ## Finding 10: a zero command in a slowdown zone

Upstream takes no action for a zero command (`0 < 0` is false) and reports
none; the model returns the slowdown. The published bits are the same. -/
example : outputs cfg [.scan 0 (scanAt (ms 900000) [⟨r0_25, ⟨500, 0⟩⟩, ⟨r0_25, ⟨0, 500⟩⟩]),
      .cmd (ms 1000) halt]
    = [.silent, .publish (.slowdown 1 d0_5)] := by decide

/-! ## Finding 1: what the shim's radius conversion yields for a negative radius

`(int64_t)ceil(-0.3 * 1000.0) + 1 = -299`, which `.toInt.toNat` makes 0.
The configuration is accepted and the zone contains nothing. Upstream uses
`radius²` and stops (case `negative_radius`). -/
example : (Shape.circle (Int.toNat (-299))).ok = true := by decide
example : (Shape.circle (Int.toNat (-299))).contains ⟨0, 0⟩ = false := by decide
example : (Config.ok { cfg with zones := [⟨.circle 0, .stop, 1, true⟩] }) = true := by decide
example : zonesAction { cfg with zones := [⟨.circle 0, .stop, 1, true⟩] } [⟨0, 0⟩, ⟨200, 0⟩]
    = .pass := by decide

/-! Finding 7: the same route for a negative `source_timeout` if the shim's refusal (D9) is
missing or wrong: `.toInt.toNat` gives 0, which means "check disabled". -/
example : outputs { cfg with sourceTimeout := Int.toNat (-2000000000) } [.cmd (ms 1000) fwd]
    = [.publish .pass] := by decide

/-! ## Finding 12: "Bad data (a)" read literally

The point of an invalid beam does occur in `Scan.points` when a valid beam
has the same point. -/
example : (scanAt 0 [⟨rNaN, ⟨250, 0⟩⟩, ⟨r0_25, ⟨250, 0⟩⟩]).points = [⟨250, 0⟩] := by decide

/-! ## Finding 15: configurations `Config.ok` accepts

No zone at all; a "polygon" whose four vertices coincide (a disc of √5 mm);
four collinear vertices (a strip 4.5 mm wide). -/
example : Config.ok ⟨[], false, 0, 0⟩ = true := by decide
example : (Shape.polygon [⟨300, 0⟩, ⟨300, 0⟩, ⟨300, 0⟩, ⟨300, 0⟩]).ok = true := by decide
example : (Shape.polygon [⟨0, 0⟩, ⟨100, 0⟩, ⟨200, 0⟩, ⟨300, 0⟩]).ok = true := by decide
example : (Shape.polygon [⟨0, 0⟩, ⟨100, 0⟩, ⟨200, 0⟩, ⟨300, 0⟩]).contains ⟨150, 2⟩ = true := by
  decide
example : (Shape.polygon [⟨0, 0⟩, ⟨100, 0⟩, ⟨200, 0⟩, ⟨300, 0⟩]).contains ⟨150, 3⟩ = false := by
  decide

/-! ## Finding 17: bits above the width of a ratio are not ignored

The file's header says bits above the format's width are ignored. That holds
for `F32.le`, `F64.isFinite` and `F64.isZero`; a ratio is compared as a `Nat`.
Harmless: such a ratio is never selected and `Config.ok` refuses it. -/
example : zonesAction { cfg with zones := [⟨.circle 600, .slowdown (2 ^ 64 + d0_5), 1, true⟩] }
    [⟨500, 0⟩] = .pass := by decide

/-! ## Behaviours without an example in `Examples.lean` -/

-- A polygon whose closing vertex is repeated, as hand-written configurations
-- often have it, behaves as the polygon without the repetition.
def box : List Point := [⟨300, 300⟩, ⟨300, -300⟩, ⟨0, -300⟩, ⟨0, 300⟩]
example : [(⟨150, 0⟩ : Point), ⟨150, 300⟩, ⟨300, 0⟩, ⟨303, 0⟩, ⟨-3, 0⟩, ⟨0, 0⟩, ⟨400, 400⟩].map
      (Shape.polygon (box ++ [⟨300, 300⟩])).contains
    = [(⟨150, 0⟩ : Point), ⟨150, 300⟩, ⟨300, 0⟩, ⟨303, 0⟩, ⟨-3, 0⟩, ⟨0, 0⟩, ⟨400, 400⟩].map
      (Shape.polygon box).contains := by decide

-- The clock jumps backwards (A2): a scan that was too old is valid again.
example : outputs cfg [.scan (ms 9000) (scanAt (ms 5000) [far]), .cmd (ms 9000) fwd,
      .cmd (ms 6000) fwd]
    = [.silent, .publish .stopSource, .publish .pass] := by decide

-- A polygon stop zone through `step`, with a slowdown zone listed before it.
example : outputs { cfg with zones := [⟨.circle 600, .slowdown d0_5, 1, true⟩,
        ⟨.polygon box, .stop, 1, true⟩] }
      [.scan 0 (scanAt (ms 900000) [⟨r0_25, ⟨150, 100⟩⟩]), .cmd (ms 1000) fwd]
    = [.silent, .publish (.stopZone 1)] := by decide

-- A slowdown ratio of 0 through `step`: zero is published for 1 s, then nothing.
example : outputs { cfg with zones := [⟨.circle 600, .slowdown 0, 1, true⟩] }
      [.scan 0 (scanAt (ms 900000) [⟨r0_25, ⟨250, 0⟩⟩]), .cmd (ms 1000) fwd, .cmd (ms 2000) fwd,
       .cmd (ms 2001) fwd]
    = [.silent, .publish (.slowdown 0 0), .publish (.slowdown 0 0), .silent] := by decide

end Review.ModelCases
