import ROSlean.CollisionMonitor.FFI
import ROSlean.CollisionMonitor.Convert

/-!
# CM3/CM4 review: checks on the interface theorems

Run from the repository root, after `lake build ROSlean`:

    lake env lean review/collision_monitor/InterfaceChecks.lean

1. The axioms behind the interface theorems.
2. The interface functions evaluated on call sequences the trace theorems did
   not cover at `cb25673`: those that contain an unusable scan.
3. Builder call orders the C++ wrapper does not produce.

Adapted after the fixes (branch `claude/cm-node-fixes`): an unusable scan is
now the event `scanUnusable` and is covered by `cm_readBack_after`;
`dropScan_stop` became `cmOnScan_unusable_stop`; and the two evaluations of
section 3 now show the fixed behaviour (the comments give both).
-/

open ROSleanCore.CollisionMonitor ROSleanCore.CollisionMonitorFFI
open ROSlean.CollisionMonitor ROSlean.CollisionMonitor.FFI ROSlean.CollisionMonitor.Convert

#print axioms cm_readBack_after
#print axioms cm_out_after
#print axioms cm_not_ok_after
#print axioms foldl_exec
#print axioms decodeBeams_encodeBeams
#print axioms cmOnScan_unusable_stop
#print axioms run_scanUnusable_stop
#print axioms not_ok_of_refused_call
#print axioms not_ok_without_set_source
#print axioms circle_safe
#print axioms coordMm_abs_le
#print axioms radiusMm_ge
#print axioms vertexMm_abs_sub_le
#print axioms timeoutNs_isSome
#print axioms cmOk_iff

/-! ## 2. Unusable scans

A stop circle of 0.3 m, `source_timeout` 2 s, `stop_pub_timeout` 1 s. -/

def b0 : Builder :=
  builderAddCircle
    (builderSetStopPubTimeout (builderSetSource (builderNew ()) 1 0x4000000000000000 0 0)
      0x3FF0000000000000)
    0x3FD3333333333333 0 0 1 1

def sec : UInt64 := 1000000000
def half : UInt64 := 0x3FE0000000000000  -- 0.5

/-- An empty usable scan stamped `t`. -/
def good (t : UInt64) : Call := .scan t t.toInt64 1 0 0x42C80000 ByteArray.empty
def bad (t : UInt64) : Call := .scan t t.toInt64 0 0 0x42C80000 ByteArray.empty
def go (t : UInt64) : Call := .cmd t half 0 0 0 0 0

def kinds (cs : List Call) : List UInt8 :=
  (cs.foldl (fun (acc : Monitor × List UInt8) c =>
    let m := c.exec acc.1
    (m, match c with
        | .cmd .. => acc.2 ++ [cmOutKind m]
        | _ => acc.2)) (cmNew b0, [])).2

-- The configuration is accepted.
#eval cmOk (cmNew b0)

-- pass; after an unusable scan: stopSource (4), published for 1 s, then silent (0).
#eval kinds [good sec, go sec, bad sec, go sec, go (2 * sec), go (2 * sec + 1)]

/-- The state after `[good, go, stop for a stale scan, good, go, bad]`: no
scan, previous velocity not zero, stop stamp not 0. `run` from `State.init`
could not reach a state with `scan = none` and `stopStamp ≠ 0` and
`prevZero = false` under this configuration while its events were scans and
commands only. With the event `scanUnusable` it does, and the trace theorems
apply to what follows. -/
def afterDrop : Monitor :=
  [good sec, go sec, go (4 * sec), good (4 * sec), go (4 * sec), bad (4 * sec)].foldl
    Call.exec (cmNew b0)

#eval (afterDrop.st.scan.isNone, afterDrop.st.prevZero, afterDrop.st.stopStamp)

/-! ## 3. Builder call orders -/

-- At `cb25673` a refused vertex that followed a circle left no mark and this was
-- `true` (also because the source was not set). Now `false`, with or without `set_source`.
#eval cmOk (cmNew (builderAddVertex (builderAddCircle (builderNew ()) 0x3FD3333333333333 0 0 1 1)
  0x7FC00000 0))
#eval cmOk (cmNew (builderAddVertex (builderAddCircle
  (builderSetSource (builderNew ()) 1 0x4000000000000000 0 0) 0x3FD3333333333333 0 0 1 1)
  0x7FC00000 0))

-- At `cb25673`: without `set_source` the source timeout was 0, and with no scan the
-- command passed (kind 1). Now the monitor is not ok and the answer is a stop (kind 4).
#eval cmOutKind (cmOnCmd (cmNew (builderAddCircle (builderNew ()) 0x3FD3333333333333 0 0 1 1))
  (5 * sec) half 0 0 0 0 0)
