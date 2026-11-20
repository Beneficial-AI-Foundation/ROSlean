import ROSleanCore.Mux

/-!
# Worked examples for the twist_mux model

Checked at compile time with `#guard`. Times are in arbitrary units here (the
model does not care); read them as milliseconds.
-/

namespace ROSlean.Mux.Examples

open ROSleanCore.Mux

/-- Input 0: planner, priority 10. Input 1: teleop, priority 100. Both expire
after 500. Lock 0: e-stop, priority 255, never expires. -/
def cfg : Config :=
  { inputs := [⟨10, 500⟩, ⟨100, 500⟩], locks := [⟨255, 0⟩], inputs_distinct := by decide }

def decisions (es : List Event) : List Bool := (run cfg (State.init cfg) es).2

-- The planner drives when nothing else is live (teleop has never spoken, so
-- by time 1000 it has expired: R5).
#guard decisions [.input 0 1000, .input 0 1100] = [true, true]

-- Teleop takes over; the planner is suppressed while teleop is live (R8, R9).
#guard decisions [.input 0 1000, .input 1 1050, .input 0 1100, .input 1 1150]
  = [true, true, false, true]

-- Teleop falls silent: the planner is still suppressed exactly at the timeout
-- and resumes strictly after it (R4).
#guard decisions [.input 1 1000, .input 0 1500, .input 0 1501] = [true, false, true]

-- The e-stop lock outranks both inputs; releasing it restores them (R6, R8).
#guard decisions [.input 1 1000, .lock 0 true 1010, .input 1 1020, .input 0 1030,
    .lock 0 false 1040, .input 1 1050]
  = [true, false, false, false, false, true]

-- With a clock near 0 (simulation time just started) both inputs count as
-- live, so the planner is suppressed by a teleop that has never spoken (R3,
-- R4). On a wall clock this cannot happen: every stamp-0 handle has expired.
#guard decisions [.input 0 100, .input 0 501] = [false, true]

/-- A lock at the same priority as teleop blocks the planner but not teleop
(R8). It has a timeout, so a silent publisher locks (R6). -/
def cfgEq : Config :=
  { inputs := [⟨10, 500⟩, ⟨100, 500⟩], locks := [⟨100, 200⟩], inputs_distinct := by decide }

/-- One input and a lock above it with a timeout of 200. -/
def cfgLock : Config :=
  { inputs := [⟨100, 500⟩], locks := [⟨200, 200⟩], inputs_distinct := by decide }

def decisionsEq (es : List Event) : List Bool := (run cfgEq (State.init cfgEq) es).2

-- Lock explicitly released: planner drives. Lock publisher then goes silent
-- past its 200 timeout: planner is blocked, teleop is not.
#guard decisionsEq [.lock 0 false 1000, .input 0 1100, .input 0 1201, .input 1 1202]
  = [false, true, false, true]

-- A never-heard lock with a timeout, as seen by teleop on a clock near 0 (so
-- that nothing else interferes). `cfgLock` has only teleop and a lock above it.
-- Not locked while the clock is within the lock's timeout (200), locked
-- after, and released by the first `false`.
#guard (run cfgLock (State.init cfgLock)
    [.input 0 200, .input 0 201, .lock 0 false 300, .input 0 301, .input 0 501]).2
  = [true, false, false, true, false]

-- The clock jumps backwards: nothing expires, stamps are overwritten (A2).
#guard decisions [.input 1 1000, .input 0 900, .input 0 1400, .input 0 1501]
  = [true, false, false, true]

/-- A lock strictly between the two inputs, plus a lower lock. -/
def cfgMid : Config :=
  { inputs := [⟨10, 500⟩, ⟨100, 500⟩], locks := [⟨5, 0⟩, ⟨50, 0⟩], inputs_distinct := by decide }

-- The low lock blocks nothing. The middle lock blocks the planner only; with
-- teleop silent and the planner masked, nothing is forwarded (R7, R8).
#guard (run cfgMid (State.init cfgMid)
    [.lock 0 true 1000, .input 0 1001, .lock 1 true 1002, .input 0 1003, .input 1 1004,
     .lock 1 false 1005, .input 0 1600]).2
  = [false, true, false, false, true, false, true]

/-- A never-expiring input (timeout 0) suppresses lower ones for ever without
having sent anything (R5), and a priority-0 input is never forwarded (R9). -/
def cfgOdd : Config :=
  { inputs := [⟨0, 500⟩, ⟨10, 500⟩, ⟨100, 0⟩], locks := [], inputs_distinct := by decide }

#guard (run cfgOdd (State.init cfgOdd) [.input 1 100000, .input 0 100001, .input 2 100002]).2
  = [false, false, true]

-- Out-of-range indices are never forwarded.
#guard decisions [.input 7 1000, .lock 7 true 1001, .input 0 1002] = [false, false, true]

-- A state with missing entries loses no updates: the lock is recorded.
#guard (run cfg ⟨[], [], []⟩ [.lock 0 true 10, .input 0 1000]).2 = [false, false]
#guard (run cfg ⟨[], [], []⟩ [.input 0 1000, .input 0 1001]).2 = [true, true]

-- Duplicate input priorities are rejected (D1); a lock may share a priority.
-- Priorities are clamped to 255 first, which can create duplicates (R1).
#guard (Config.validate [⟨10, 0⟩, ⟨10, 5⟩] []).toOption.isNone
#guard (Config.validate [⟨300, 0⟩, ⟨400, 0⟩] []).toOption.isNone
#guard ((Config.validate [⟨300, 0⟩] [⟨999, 0⟩]).toOption.map (·.locks)) = some [⟨255, 0⟩]
#guard (Config.validate [⟨10, 0⟩, ⟨20, 5⟩] [⟨10, 0⟩, ⟨10, 0⟩]).toOption.isSome

end ROSlean.Mux.Examples
