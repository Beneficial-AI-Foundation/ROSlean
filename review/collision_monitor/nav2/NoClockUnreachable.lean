import ROSlean.CollisionMonitor.NoClock
import ROSlean.CollisionMonitor.Fast
import ROSlean.CollisionMonitor.Trace

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor ROSlean.CollisionMonitor.FFI

def le (n : Nat) (bytes : Nat) : List UInt8 := (List.range bytes).map fun i => UInt8.ofNat (n / 256 ^ i)
def ba (l : List UInt8) : ByteArray := ⟨l.toArray⟩
def pairs (l : List (Nat × Nat)) : ByteArray := ba (l.flatMap fun p => le p.1 8 ++ le p.2 8)
def ranges (l : List Nat) : ByteArray := ba (l.flatMap fun r => le r 4)

def half : UInt64 := 0x3FE0000000000000
def bld : Builder :=
  builderAddCircle
    (builderSetStopPubTimeout (builderSetSource (builderNew ()) 1 0x4000000000000000 0 0) 0x3FF0000000000000)
    0x3FF0000000000000 0 0 1 1
def m0 : Monitor := cmNew bld
#eval cmOk m0
#eval repr m0.cfg

def scanCall (m : Monitor) (rs : List Nat) : Monitor := cmOnScan m 5 5 0x3DCCCCCD 0x41200000 (ranges rs)
def show' (m : Monitor) := (repr m.st, repr m.out, cmOutKind m, cmOutReport m)

-- Q1: scan in each phase
#eval show' (scanCall m0 [0x3F000000])                                   -- unconfigured
#eval show' (scanCall (cmOnLifecycle m0 0) [0x3F000000])                 -- inactive
#eval show' (scanCall (cmOnLifecycle (cmOnLifecycle m0 0) 1) [0x3F000000]) -- active
#eval show' (scanCall (cmOnLifecycle (cmOnLifecycle m0 0) 4) [0x3F000000]) -- finalized
def mi := scanCall (cmOnLifecycle m0 0) [0x3F000000]
def mA := cmOnLifecycle mi 1
-- scan after shutdown from active with stored scan: model keeps OLD scan
#eval show' (scanCall (cmOnLifecycle mA 4) [0x3F000000, 0x3F000000])
-- cleanup then configure: what stays
#eval show' (cmOnLifecycle (cmOnLifecycle mA 2) 3)
-- command: point (0.5,0) inside circle -> stop
def cmdF (m : Monitor) (now : UInt64) (hp : UInt8) (p z : ByteArray) := cmOnCmdFast m now half 0 0 0 0 0 hp p z
def cmdR (m : Monitor) (now : UInt64) (hp : UInt8) (p z : ByteArray) := cmOnCmd m now half 0 0 0 0 0 hp p z
#eval show' (cmdF mA 6 1 (pairs [(0x3FE0000000000000, 0)]) (ba []))
#eval show' (cmdF mA 6 1 (pairs [(0x4014000000000000, 0)]) (ba []))  -- (5,0): pass
#eval show' (cmdF mA 6 0 (pairs [(0x4014000000000000, 0)]) (ba []))  -- no points
#eval show' (cmdF mA 6 1 (pairs []) (ba []))  -- wrong count
#eval show' (cmdF mA 6 1 (ba ((pairs [(0x4014000000000000, 0)]).data.toList ++ [1,2,3])) (ba []))  -- trailing bytes
#eval show' (cmdF mA 6 1 (ba [1,2,3]) (ba [0xFF,0xFF,0xFF,0xFE, 1]))  -- short, huge zone count
#eval show' (cmdF mA 6 1 (pairs [(0x4014000000000000, 0)]) (ba [0xFE,0xFF,0xFF,0xFF, 1]))  -- huge zone count
#eval (decodeZoneVerts (ba [0xFE,0xFF,0xFF,0xFF, 1]), decodeZoneVerts (ba [0,0,0,0, 0xFF,0xFF,0xFF,0xFF, 1,0,0]))
-- saturation / NaN / inf / -0 / subnormal
#eval [coordMm 0x7FF8000000000000, coordMm 0xFFF8000000000000, coordMm 0x7FF0000000000000, coordMm 0xFFF0000000000000,
       coordMm 0x8000000000000000, coordMm 1, coordMm 0x8000000000000001, coordMm 0x7FEFFFFFFFFFFFFF, coordMm 0xFFEFFFFFFFFFFFFF,
       coordMm 0x408F400000000000, coordMm 0x408F3FFFFFFFFFFF, coordMm 0xC08F3FFFFFFFFFFF]
#eval [vertexCoordMm 0x7FF8000000000000, vertexCoordMm 0xFFF8000000000000, vertexCoordMm 0xFFF0000000000000,
       vertexCoordMm 0x8000000000000000, vertexCoordMm 1, vertexCoordMm 0x408F3FF9DB22D0E5, vertexCoordMm 0x408F400000000000, vertexCoordMm 0x4090000000000000]
#eval show' (cmdF mA 6 1 (pairs [(0x7FF8000000000000, 0)]) (ba []))  -- NaN x on valid reading
-- no clock in each phase
def nc (m : Monitor) (lx : UInt64) := cmOnCmdNoClock m lx 0 0 0 0 0
#eval show' (nc m0 half)
#eval show' (nc (cmOnLifecycle m0 0) half)
#eval show' (nc mA half)
#eval show' (nc mA 0x7FF8000000000000)
#eval show' (nc (cmOnLifecycle mA 4) half)
#eval show' (nc (cmOnLifecycle mA 2) half)
-- after no-clock: next clocked command
#eval show' (cmdF (nc mA half) 6 1 (pairs [(0x4014000000000000, 0)]) (ba []))

/-- reachable-state invariant that stepNoClock breaks -/
def J (st : State) : Prop := st.prevReport = .source → st.prevZero = true

theorem J_step (cfg : Config) (st : State) (e : Event) (h : J st) : J (step cfg st e).1 := by
  unfold J at *
  cases e with
  | cmd now v p z =>
    simp only [step]
    split
    · exact h
    · split
      · exact h
      · simp only
        cases evaluate cfg st now p z <;> simp [Action.report, Action.yieldsZero]
        split <;> simp
  | shape k verts foreign =>
    simp only [step]
    split <;> exact h
  | deactivate => simp only [step]; split <;> simp_all
  | _ => simp only [step] <;> (try split) <;> exact h

theorem J_run (cfg : Config) (st : State) (es : List Event) (h : J st) : J (run cfg st es).1 := by
  induction es generalizing st with
  | nil => exact h
  | cons e es ih => rw [run_cons]; exact ih _ (J_step cfg st e h)

/-- The state after a processed no-clock command is reached by no run from start-up, for any configuration. -/
theorem noClock_unreachable (st : State) (v : Twist) (hp : st.phase = .active) (hf : v.finite = true)
    (cfg : Config) (es : List Event) : (stepNoClock st v).1 ≠ (run cfg (State.init cfg) es).1 := by
  intro heq
  have hJ := J_run cfg (State.init cfg) es (by simp [J, State.init])
  rw [← heq, noClock_state st v hp hf] at hJ
  simp [J] at hJ

#print axioms noClock_unreachable
#print axioms cm_readBack_after_fast
#print axioms cm_readBack_after
#print axioms cm_out_after
#print axioms cm_phase_after
#print axioms cm_not_ok_after
#print axioms cmOnCmdFast_eq
#print axioms execFast_eq_exec
#print axioms stepCmdFast_eq
#print axioms noClock_velocity
#print axioms inv_noClock
#print axioms velocity_after_noClock
#print axioms cmOnCmdNoClock_kind
#print axioms cmOnCmdNoClock_kind_cases
#print axioms foldl_exec_run
#print axioms reachable_inv
#print axioms run_stop
#print axioms run_withheld_last_published_zero
#print axioms liveConfig_bounded
#print axioms cmShapeAccepted_iff
#print axioms cmOnCmd_of_plan_zero
#print axioms coordMm64_eq
