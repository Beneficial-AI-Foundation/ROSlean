import ROSleanCore.Mux

/-!
# C interface to the twist_mux model

A flat API for the ROS shim: opaque objects and scalars only, so the C++ side
never inspects or builds Lean data. The matching C++ wrapper is
`ros/roslean_core/include/roslean_core/mux.hpp`; the two must be kept in sync
by hand, and `ros/roslean_core/test/test_mux_ffi.cpp` checks that they agree.

Every function here is a thin wrapper: all decisions are made by
`ROSleanCore.Mux.step`.

Ownership: Lean does not allow borrowed parameters on exported functions, so
every object argument is *consumed*. A caller that wants to keep using an
object after passing it to a query (`builderCheck`, `muxForward`) must
`lean_inc` it first.
-/

namespace ROSleanCore.FFI

open ROSleanCore.Mux

/-- A configuration under construction. -/
structure Builder where
  inputs : List Handle
  locks : List Handle

/-- A running mux: validated config, current state, and the decision for the
most recent event. -/
structure Mux where
  cfg : Config
  st : State
  /-- Whether the message of the most recent event is to be forwarded. -/
  forward : Bool

@[export roslean_mux_builder_new]
def builderNew (_ : Unit) : Builder := { inputs := [], locks := [] }

/-- Builds a handle from the raw parameter values: a negative priority becomes
0 (R1; the upper clamp is applied by `Config.validate`), and a timeout that is
zero or negative becomes 0, i.e. never expires (R2). -/
def handleOfRaw (priority timeoutNs : Int64) : Handle :=
  ⟨priority.toInt.toNat, timeoutNs.toInt.toNat⟩

/-- Appends an input; its index is the number of inputs added before it.
`priority` is the raw parameter value and `timeoutNs` the timeout truncated to
nanoseconds. -/
@[export roslean_mux_builder_add_input]
def builderAddInput (b : Builder) (priority timeoutNs : Int64) : Builder :=
  { b with inputs := b.inputs ++ [handleOfRaw priority timeoutNs] }

/-- Appends a lock; its index is the number of locks added before it. -/
@[export roslean_mux_builder_add_lock]
def builderAddLock (b : Builder) (priority timeoutNs : Int64) : Builder :=
  { b with locks := b.locks ++ [handleOfRaw priority timeoutNs] }

/-- Error codes of `builderCheck`. -/
def errorCode : Except ConfigError Config → UInt8
  | .ok _ => 0
  | .error .duplicateInputPriority => 1

/-- `0` if the configuration is valid, otherwise a non-zero error code:
`1` = two inputs share a priority. -/
@[export roslean_mux_builder_check]
def builderCheck (b : Builder) : UInt8 :=
  errorCode (Config.validate b.inputs b.locks)

/-- Creates a mux in its initial state. The caller must have checked the
builder with `builderCheck`; an invalid builder yields a mux with no inputs
and no locks, which never forwards anything. -/
@[export roslean_mux_new]
def muxNew (b : Builder) : Mux :=
  let cfg := match Config.validate b.inputs b.locks with
    | .ok cfg => cfg
    | .error _ => Config.empty
  { cfg, st := State.init cfg, forward := false }

/-- Applies one event and records its forwarding decision. -/
def Mux.apply (m : Mux) (e : Event) : Mux :=
  let (st, forward) := step m.cfg m.st e
  { m with st, forward }

/-- A message arrived on input `i` at node-clock time `tNs`. -/
@[export roslean_mux_on_input]
def muxOnInput (m : Mux) (i : UInt32) (tNs : UInt64) : Mux :=
  m.apply (.input i.toNat tNs.toNat)

/-- A message with the given value arrived on lock `j` at node-clock time `tNs`. -/
@[export roslean_mux_on_lock]
def muxOnLock (m : Mux) (j : UInt32) (value : Bool) (tNs : UInt64) : Mux :=
  m.apply (.lock j.toNat value tNs.toNat)

/-- The decision for the most recent event. -/
@[export roslean_mux_forward]
def muxForward (m : Mux) : Bool := m.forward

end ROSleanCore.FFI
