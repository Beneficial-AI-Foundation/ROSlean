import ROSlean.CollisionMonitor.NoClock
import ROSlean.CollisionMonitor.Fast
import ROSlean.CollisionMonitor.Trace

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor ROSlean.CollisionMonitor.FFI

/-! ### (1) erasing no-clock commands keeps phase, scan and shapes -/

def Sim (a b : State) : Prop := a.phase = b.phase ∧ a.scan = b.scan ∧ a.shapes = b.shapes

theorem sim_step (cfg : Config) (a b : State) (e : Event) (h : Sim a b) :
    Sim (step cfg a e).1 (step cfg b e).1 := by
  rcases a with ⟨p, s, sh, z, t, r⟩
  rcases b with ⟨p', s', sh', z', t', r'⟩
  simp only [Sim] at h
  obtain ⟨rfl, rfl, rfl⟩ := h
  cases e <;> simp only [step, acceptShape] <;> (repeat' split) <;>
    simp_all [Sim, shapesAfter, sourcePoints]

inductive MEv where
  | ev (e : Event)
  | nc (v : Twist)

def mstep (cfg : Config) (st : State) : MEv → State
  | .ev e => (step cfg st e).1
  | .nc v => (stepNoClock st v).1

def erase : List MEv → List Event
  | [] => []
  | .ev e :: l => e :: erase l
  | .nc _ :: l => erase l

theorem sim_mrun (cfg : Config) (l : List MEv) (a b : State) (h : Sim a b) :
    Sim (l.foldl (mstep cfg) a) (run cfg b (erase l)).1 := by
  induction l generalizing a b with
  | nil => exact h
  | cons m l ih =>
    cases m with
    | ev e =>
      simp only [List.foldl_cons, erase, run_cons, mstep]
      exact ih _ _ (sim_step cfg a b e h)
    | nc v =>
      simp only [List.foldl_cons, erase, mstep]
      refine ih _ _ ?_
      obtain ⟨h1, h2, h3, -⟩ := noClock_keeps a v
      exact ⟨h1.trans h.1, h2.trans h.2.1, h3.trans h.2.2⟩

#print axioms sim_mrun

/-! ### (2) the stop stamp is dead while prevZero is false (cleanup vs a new monitor) -/

def R (a b : State) : Prop :=
  a.phase = b.phase ∧ a.scan = b.scan ∧ a.shapes = b.shapes ∧ a.prevZero = b.prevZero ∧
    a.prevReport = b.prevReport ∧ (a.prevZero = true → a.stopStamp = b.stopStamp)

theorem R_step (cfg : Config) (a b : State) (e : Event) (h : R a b) :
    R (step cfg a e).1 (step cfg b e).1 ∧ (step cfg a e).2 = (step cfg b e).2 := by
  rcases a with ⟨p, s, sh, z, t, r⟩
  rcases b with ⟨p', s', sh', z', t', r'⟩
  simp only [R] at h
  obtain ⟨rfl, rfl, rfl, rfl, rfl, ht⟩ := h
  cases e with
  | cmd now v pts zv =>
    simp only [step]
    split
    · exact ⟨⟨rfl, rfl, rfl, rfl, rfl, ht⟩, rfl⟩
    · split
      · exact ⟨⟨rfl, rfl, rfl, rfl, rfl, ht⟩, rfl⟩
      · have he : evaluate cfg ⟨p, s, sh, z, t, r⟩ now pts zv = evaluate cfg ⟨p, s, sh, z, t', r⟩ now pts zv :=
          evaluate_congr cfg _ _ now pts zv rfl rfl
        have hs : shapesAfter cfg ⟨p, s, sh, z, t, r⟩ now pts zv = shapesAfter cfg ⟨p, s, sh, z, t', r⟩ now pts zv := by
          simp [shapesAfter, sourcePoints]
        rw [he, hs]
        cases z with
        | false =>
          simp [R]
          intro hz
          simp [hz]
        | true =>
          have := ht rfl
          subst this
          simp [R]
  | shape k verts foreign =>
    simp only [step, acceptShape]
    repeat' split
    all_goals simp_all [R]
  | _ => simp only [step] <;> (try split) <;> simp_all [R]

#print axioms R_step

/-- After a cleanup that takes effect in a state with the lifecycle invariant, the state is
`State.init` except for the stop stamp. -/
theorem cleanup_init (cfg : Config) (st : State) (h : Inv cfg st) (hp : st.phase = .inactive) :
    (step cfg st .cleanup).1 = { State.init cfg with stopStamp := st.stopStamp } := by
  have hq := h.quiet (Or.inr hp)
  have hl := h.len
  rcases st with ⟨p, s, sh, z, t, r⟩
  simp only at hp hq hl
  obtain ⟨rfl, rfl⟩ := hq
  subst hp
  simp only [step, State.init, if_true]
  congr 1
  apply List.ext_getElem
  · simp [hl]
  · intro i h1 h2; simp

/-! ### (3) more evals -/

def le (n : Nat) (bytes : Nat) : List UInt8 := (List.range bytes).map fun i => UInt8.ofNat (n / 256 ^ i)
def ba (l : List UInt8) : ByteArray := ⟨l.toArray⟩
def pairs (l : List (Nat × Nat)) : ByteArray := ba (l.flatMap fun p => le p.1 8 ++ le p.2 8)
def ranges (l : List Nat) : ByteArray := ba (l.flatMap fun r => le r 4)
def half : UInt64 := 0x3FE0000000000000
def one : Nat := 0x3FF0000000000000
def mone : Nat := 0xBFF0000000000000
def two : Nat := 0x4000000000000000

-- geometry extremes
#eval (Shape.circle 1000000).contains ⟨1000000, 0⟩
#eval (Shape.circle 1000000).contains ⟨-1000000, 0⟩
#eval (Shape.circle 1000000).contains ⟨0, 1000000⟩
#eval (Shape.circle 1000000).contains ⟨999999, 0⟩
def big : Shape := .polygon [⟨999997, 999997⟩, ⟨-999997, 999997⟩, ⟨-999997, -999997⟩, ⟨999997, -999997⟩]
#eval big.ok
#eval [big.contains ⟨1000000, 0⟩, big.contains ⟨-1000000, 5⟩, big.contains ⟨1000000, 1000000⟩, big.contains ⟨0, -1000000⟩,
       big.contains ⟨999999, 0⟩, big.contains ⟨999997, 999997⟩]

-- a topic stop zone, source timeout 2 s
def bldT : Builder :=
  builderAddTopicPolygon
    (builderSetStopPubTimeout (builderSetSource (builderNew ()) 1 0x4000000000000000 0 0) 0x3FF0000000000000)
    0 0 1 1
def t0 : Monitor := cmNew bldT
#eval cmOk t0
def tri : ByteArray := pairs [(one, one), (mone, one), (0, mone)]
def sqr : ByteArray := pairs [(one, one), (mone, one), (mone, mone), (one, mone)]
def acc (m : Monitor) := (cmShapeAccepted m, repr m.st.shapes, repr m.st.phase)
#eval acc (cmOnShape t0 0 0 1 tri)                                         -- unconfigured
#eval acc (cmOnShape (cmOnLifecycle t0 0) 0 0 1 tri)                       -- inactive
#eval acc (cmOnShape (cmOnLifecycle (cmOnLifecycle t0 0) 1) 0 1 1 tri)     -- active, foreign
#eval acc (cmOnShape (cmOnLifecycle (cmOnLifecycle t0 0) 4) 0 0 1 tri)     -- finalized
#eval acc (cmOnShape (cmOnLifecycle t0 0) 0 0 0 tri)                       -- no verts
#eval acc (cmOnShape (cmOnLifecycle t0 0) 0 0 1 (pairs [(one, one), (mone, one)]))  -- two
#eval acc (cmOnShape (cmOnLifecycle t0 0) 1 0 1 tri)                       -- no such zone
#eval acc (cmOnShape (cmOnLifecycle t0 0) 0 0 1 (pairs [(one, one), (mone, one), (0x7FF8000000000000, mone)]))  -- NaN
#eval acc (cmOnShape (cmOnLifecycle t0 0) 0 0 1 (ba (tri.data.toList ++ [1,2,3,4,5])))  -- trailing bytes
def tA : Monitor := cmOnShape (cmOnLifecycle (cmOnLifecycle t0 0) 1) 0 1 1 tri
def tS : Monitor := cmOnScan tA 5 5 0x3DCCCCCD 0x41200000 (ranges [0x3F000000])
def show' (m : Monitor) := (repr m.st.shapes, repr m.out, cmOutKind m, cmOutReport m)
def zv (n : Nat) (p : ByteArray) : ByteArray := ba (le n 4 ++ p.data.toList)
-- min_points 1, point (0,0) inside the triangle
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)]) (ba []))
-- refresh with 4 vertices for a 3-vertex shape: not taken
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)]) (zv 4 sqr))
-- refresh with 3 vertices far away (shifted by +2 in x: (3,1),( 1,1),(2,-1)): the point (0,0) is outside now
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)])
   (zv 3 (pairs [(0x4008000000000000, one), (one, one), (two, mone)])))
-- refresh with a NaN vertex: not taken, old shape still stops
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)])
   (zv 3 (pairs [(0x7FF8000000000000, one), (one, one), (two, mone)])))
-- count 0xFFFFFFFF = none; count says 3 but only 2 present: entry dropped
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)]) (zv 0xFFFFFFFF (ba [])))
#eval show' (cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)]) (zv 3 (pairs [(one, one), (one, one)])))
-- cleanup drops the shape
#eval acc (cmOnLifecycle (cmOnLifecycle tS 2) 3)
-- fast = reference on the malformed inputs
def same (m : Monitor) (hp : UInt8) (p z : ByteArray) : Bool :=
  let a := cmOnCmdFast m 6 half 0 0 0 0 0 hp p z
  let b := cmOnCmd m 6 half 0 0 0 0 0 hp p z
  decide (a.st = b.st) && decide (a.out = b.out)
#eval [same tS 1 (ba [1,2,3]) (ba [1]), same tS 1 (ba []) (ba [0xFF,0xFF,0xFF]), same tS 7 (pairs [(0,0)]) (zv 3 sqr),
       same tS 1 (ba ((pairs [(0,0)]).data.toList ++ [9])) (zv 0 (ba [])), same tS 0 (ba [1]) (zv 0xFFFFFFFE (ba [1,2]))]
-- not-ok monitor
def bad : Monitor := cmNew (builderNew ())
#eval (cmOk bad, cmError bad)
def badA := cmOnLifecycle (cmOnLifecycle bad 0) 1
#eval (cmOutKind (cmOnCmdNoClock badA 0x7FF8000000000000 0 0 0 0 0), cmOutKind (cmOnCmdFast badA 6 0x7FF8000000000000 0 0 0 0 0 1 (ba []) (ba [])),
       cmOutKind (cmOnCmdNoClock bad half 0 0 0 0 0), cmCmdPlan badA 6 half 0 0 0 0 0)
-- plan
#eval (cmCmdPlan tS 6 half 0 0 0 0 0, cmCmdPlan tA 6 half 0 0 0 0 0, cmCmdPlan tS 3000000000 half 0 0 0 0 0,
       cmCmdPlan tS 6 0x7FF0000000000000 0 0 0 0 0, cmCmdPlan (cmOnLifecycle tS 2) 6 half 0 0 0 0 0)
-- no-clock then R15: zeros withheld, then no clock, then clock back
def w1 := cmOnCmdFast tS 6 half 0 0 0 0 0 1 (pairs [(0, 0)]) (ba [])              -- stop published, stamp 6
def w2 := cmOnCmdFast w1 1500000000 half 0 0 0 0 0 1 (pairs [(0, 0)]) (ba [])     -- withheld
def w3 := cmOnCmdNoClock w2 half 0 0 0 0 0                                          -- stopSource, published
def w4 := cmOnCmdFast w3 1600000000 half 0 0 0 0 0 1 (pairs [(0, 0)]) (ba [])     -- published again
#eval [cmOutKind w1, cmOutKind w2, cmOutKind w3, cmOutKind w4]
#eval [cmOutReport w1, cmOutReport w2, cmOutReport w3, cmOutReport w4]
#eval repr w3.st
