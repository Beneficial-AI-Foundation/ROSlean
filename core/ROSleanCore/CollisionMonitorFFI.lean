import ROSleanCore.CollisionMonitor
import ROSleanCore.CollisionMonitorConvert

/-!
# C interface to the collision monitor model

A flat API for the ROS shim, as laid down in
`docs/collision_monitor/SPEC.md`, "Shim boundary": opaque objects and scalars,
plus one `ByteArray` per laser scan. The matching C++ wrapper is
`ros/roslean_core/include/roslean_core/collision_monitor.hpp`; the two are kept
in sync by hand, the C prototypes in `ffi.h` are checked against the generated
C at build time, and `ros/roslean_core/test/test_cm_ffi.cpp` checks that the
wrapper and this file agree on the meaning of every argument.

The shim passes parameter values as it read them: `double` and `float` values
as bit patterns, integers as integers. Every conversion to the model's
millimetres and nanoseconds, and every refusal of a value, is made here
(`ROSleanCore.CollisionMonitorConvert`), not in C++. Every decision about a
command is made by `ROSleanCore.CollisionMonitor.step`.

A monitor whose configuration was refused is not advisory: it answers every
command with `stopSource` (`Monitor.apply`). The node is meant to exit at
start-up when `cmOk` is false; this is the backstop if it does not.

The builder is fail-closed: a configuration is accepted only if the scan
source was set (`builderSetSource`), and a refused value makes it not
accepted wherever in the sequence of calls it occurs. No later call clears a
refusal.

Ownership: every object argument is *consumed* (PLAN, lesson 8). A caller that
wants to keep using a monitor after a query must `lean_inc` it first.

Theorems about this file are in `ROSlean/CollisionMonitor/FFI.lean`.

**State of this file after task T05.** The model now has the lifecycle, the
transforms and the topic zones of `docs/collision_monitor/SPEC_NAV2.md`; this
interface does not yet (task T06 writes the calls of that document's section
8). Until then the exported functions keep their signatures and their
behaviour, as the node of SPEC.md uses them, by the following reading of the
old calls in the new model:

* `cmNew` delivers `configure` and `activate`: the monitor starts in the
  active phase, since the old node has no lifecycle;
* a scan call always stores the scan (event `scan`); `usable` is remembered
  in `Monitor.placed` and stands for the transform lookup: every command
  carries the identity transform if the scan stored last was usable, and no
  transform if it was not, which is the model's "the lookup failed" (the
  stop of R8 follows, as it did for the removed event `scanUnusable`);
* the beams are in the base frame, as the old shim hands them over, and the
  identity transform leaves them there (`Xform.identity.pointMm` is
  `coordMm` on finite coordinates);
* no zone is a topic zone, and no command carries zone transforms;
* `roslean_cm_out_kind` returns 0 both for `silent` and for a processed
  command whose velocity is withheld; the report is not exposed.
-/

namespace ROSleanCore.CollisionMonitorFFI

open ROSleanCore.CollisionMonitor

/-! ## Configuration -/

/-- A configuration under construction. Zones are kept in the order in which
they were added, each with the code of a raw value that was refused when it
was added (0: none; the codes are those of `zoneError`). -/
structure Builder where
  zones : List (Zone × UInt8)
  sourceEnabled : Bool
  sourceTimeout : Nat
  stopPubTimeout : Nat
  /-- 0, or which source timeout was refused: 1 the node's `source_timeout`,
  2 the source's own. Once non-zero it stays (the first refusal is kept). -/
  sourceErr : UInt8
  /-- `stop_pub_timeout` was refused, in this or an earlier call. -/
  stopPubBad : Bool
  /-- `builderSetSource` was called. Without it the source timeout would be
  0, "check off": such a builder is not accepted. -/
  sourceSet : Bool
  /-- `builderAddVertex` was called when the zone added last was not a
  polygon, or when there was no zone: the vertex belongs to nothing. -/
  stray : Bool

/-- The model configuration the builder holds. -/
def Builder.cfg (b : Builder) : Config :=
  { zones := b.zones.map (·.1), sourceEnabled := b.sourceEnabled,
    sourceTimeout := b.sourceTimeout, stopPubTimeout := b.stopPubTimeout }

/-- Why a zone is not accepted: 0 it is accepted (`Zone.ok`); 1 its `radius`
(a circle that fails `Shape.ok`); 2 its `points` (a polygon that fails
`Shape.ok`); 3 its `slowdown_ratio` (`ZoneAction.ok`). -/
def zoneError (z : Zone) : UInt8 :=
  if !z.shape.ok then
    match z.shape with
    | .fixed (.circle _) => 1
    | .fixed (.polygon _) => 2
    | .topic => 2
  else if !z.action.ok then 3 else 0

/-- The error of a zone of the builder: the refused raw value if there was one
(1 `radius`, 2 `points`, 4 `min_points`), otherwise `zoneError`. -/
def entryError (ze : Zone × UInt8) : UInt8 := if ze.2 != 0 then ze.2 else zoneError ze.1

/-- Why the configuration is not accepted: 0 it is accepted; 1 the node's
`source_timeout`; 2 the source's own `source_timeout`; 3 `stop_pub_timeout`;
5 the source was never set; 6 a vertex was added to no polygon; 4 a zone
(`entryError` says which and why). -/
def Builder.error (b : Builder) : UInt8 :=
  if b.sourceErr != 0 then b.sourceErr
  else if b.stopPubBad then 3
  else if !b.sourceSet then 5
  else if b.stray then 6
  else if b.zones.all (fun ze => entryError ze == 0) then 0 else 4

/-- A running monitor: configuration, whether it was accepted, current state,
and the output of the most recent event. -/
structure Monitor where
  cfg : Config
  /-- The configuration was accepted. If not, the monitor stops every command. -/
  ok : Bool
  /-- `Builder.error` and the `entryError` of every zone, for error messages. -/
  err : UInt8
  zoneErrs : List UInt8
  st : State
  out : Output
  /-- The scan stored last was reported as usable (see the head of this
  file): commands carry the identity transform for it; otherwise none. -/
  placed : Bool

/-- No zones, source enabled, both timeouts 0, nothing refused, and the
source not set: the monitor of a new builder is not accepted
(`Builder.error` is 5) until `builderSetSource` has been called. -/
@[export roslean_cm_builder_new]
def builderNew (_ : Unit) : Builder :=
  { zones := [], sourceEnabled := true, sourceTimeout := 0, stopPubTimeout := 0, sourceErr := 0,
    stopPubBad := false, sourceSet := false, stray := false }

/-- Sets the scan source's `enabled` flag (any non-zero value is `true`) and
its timeout. `nodeTimeout` is the node's `source_timeout` parameter and
`ownTimeout` the source's own, both in seconds as binary64 patterns;
`hasOwn` says whether the source's own parameter is set (`sourceTimeoutNs`).
A refusal recorded by an earlier call is kept. -/
@[export roslean_cm_builder_set_source]
def builderSetSource (b : Builder) (enabled : UInt8) (nodeTimeout : UInt64) (hasOwn : UInt8)
    (ownTimeout : UInt64) : Builder :=
  { b with
    sourceEnabled := enabled != 0
    sourceTimeout := (sourceTimeoutNs nodeTimeout.toNat (hasOwn != 0) ownTimeout.toNat).getD 0
    sourceSet := true
    sourceErr :=
      if b.sourceErr != 0 then b.sourceErr
      else if (timeoutNs nodeTimeout.toNat).isNone then 1
      else if hasOwn != 0 && (timeoutNs ownTimeout.toNat).isNone then 2 else 0 }

/-- Sets `stop_pub_timeout` from the parameter in seconds (binary64 pattern).
A refusal recorded by an earlier call is kept. -/
@[export roslean_cm_builder_set_stop_pub_timeout]
def builderSetStopPubTimeout (b : Builder) (timeout : UInt64) : Builder :=
  { b with
    stopPubTimeout := (timeoutNs timeout.toNat).getD 0
    stopPubBad := b.stopPubBad || (timeoutNs timeout.toNat).isNone }

/-- The zone action from its code: 1 is `slowdown` with the given ratio
pattern; 0 and every other value is `stop`, and the ratio is ignored. -/
def actionOfRaw (action : UInt8) (ratioBits : UInt64) : ZoneAction :=
  if action == 1 then .slowdown ratioBits.toNat else .stop

/-- A zone from a shape and the raw argument values, with the code of a
refused `min_points` (4) if it is outside the 32-bit range. -/
def zoneOfRaw (shape : ZoneShape) (action : UInt8) (ratioBits : UInt64) (minPoints : Int64)
    (enabled : UInt8) : Zone × UInt8 :=
  ({ shape, action := actionOfRaw action ratioBits,
     minPoints := (minPointsOfRaw minPoints.toInt).getD 0, enabled := enabled != 0 },
   if (minPointsOfRaw minPoints.toInt).isNone then 4 else 0)

/-- Appends a circle zone; its index is the number of zones added before it.
`radius` is the parameter in metres as a binary64 pattern (`radiusMm`); if it
is refused the zone is recorded with code 1. -/
@[export roslean_cm_builder_add_circle]
def builderAddCircle (b : Builder) (radius : UInt64) (action : UInt8) (ratioBits : UInt64)
    (minPoints : Int64) (enabled : UInt8) : Builder :=
  let ze := zoneOfRaw (.fixed (.circle ((radiusMm radius.toNat).getD 0))) action ratioBits
    minPoints enabled
  { b with zones := b.zones ++ [if (radiusMm radius.toNat).isNone then (ze.1, 1) else ze] }

/-- Appends a polygon zone that has no vertices yet. -/
@[export roslean_cm_builder_add_polygon]
def builderAddPolygon (b : Builder) (action : UInt8) (ratioBits : UInt64) (minPoints : Int64)
    (enabled : UInt8) : Builder :=
  { b with zones := b.zones ++ [zoneOfRaw (.fixed (.polygon [])) action ratioBits minPoints enabled] }

/-- Appends a vertex to the last zone of the list, if that zone is a polygon;
`none` is a vertex that was refused, and marks the polygon with code 2 unless
it already has a code. Nothing changes if the last zone is a circle or the
list is empty. -/
def addVertexToLast (v : Option Point) : List (Zone × UInt8) → List (Zone × UInt8)
  | [] => []
  | [ze] =>
    match ze.1.shape with
    | .fixed (.polygon vs) =>
      match v with
      | some p => [({ ze.1 with shape := .fixed (.polygon (vs ++ [p])) }, ze.2)]
      | none => [(ze.1, if ze.2 == 0 then 2 else ze.2)]
    | _ => [ze]
  | ze :: zes => ze :: addVertexToLast v zes

/-- The vertex of two numbers of the `points` parameter (binary32 patterns,
metres), or `none` if one of them is refused (`vertexMm`). -/
def vertexOfRaw (x y : UInt32) : Option Point :=
  match vertexMm x.toNat, vertexMm y.toNat with
  | some xm, some ym => some ⟨xm, ym⟩
  | _, _ => none

/-- The zone added last is a polygon. -/
def lastIsPolygon (zs : List (Zone × UInt8)) : Bool :=
  match zs.getLast? with
  | some ze =>
    match ze.1.shape with
    | .fixed (.polygon _) => true
    | _ => false
  | none => false

/-- Appends a vertex to the zone added last. If that zone is not a polygon, or
if there is no zone, the vertex (refused or not) belongs to nothing: the zones
are unchanged and the builder is marked (`stray`), so it is not accepted. -/
@[export roslean_cm_builder_add_vertex]
def builderAddVertex (b : Builder) (x y : UInt32) : Builder :=
  { b with
    zones := addVertexToLast (vertexOfRaw x y) b.zones
    stray := b.stray || !lastIsPolygon b.zones }

/-- The state a monitor starts in: the initial state after `configure` and
`activate`. The node of SPEC.md has no lifecycle and is active from its
start. -/
def startState (cfg : Config) : State := (run cfg (State.init cfg) [.configure, .activate]).1

/-- The monitor for the configuration built so far, in the active phase
(`startState`) and with output `silent`. It is accepted iff the source was
set, no raw value was refused, no vertex was added to no polygon, and
`Config.ok` holds. -/
@[export roslean_cm_new]
def cmNew (b : Builder) : Monitor :=
  { cfg := b.cfg, ok := b.error == 0, err := b.error, zoneErrs := b.zones.map entryError,
    st := startState b.cfg, out := .silent, placed := true }

/-- The configuration was accepted. The node must refuse to start if this is
`false`. -/
@[export roslean_cm_ok]
def cmOk (m : Monitor) : Bool := m.ok

/-- `Builder.error` of the builder the monitor was made from. -/
@[export roslean_cm_error]
def cmError (m : Monitor) : UInt8 := m.err

/-- `entryError` of zone number `k`, or 5 if there is no such zone. The shim
uses `cmError` and this only to name the offending parameter. -/
@[export roslean_cm_zone_error]
def cmZoneError (m : Monitor) (k : UInt32) : UInt8 := m.zoneErrs.getD k.toNat 5

/-! ## Scan decoding -/

/-- Byte `i` of the array as a number; 0 beyond the end. -/
def byteAt (a : ByteArray) (i : Nat) : Nat :=
  if h : i < a.size then (a[i]'h).toNat else 0

/-- The unsigned 32-bit little-endian number at offset `o`. -/
def u32At (a : ByteArray) (o : Nat) : Nat :=
  byteAt a o + 0x100 * byteAt a (o + 1) + 0x10000 * byteAt a (o + 2) +
    0x1000000 * byteAt a (o + 3)

/-- The binary64 pattern stored little-endian at offset `o`. -/
def u64At (a : ByteArray) (o : Nat) : Nat := u32At a (o + 4) * 0x100000000 + u32At a o

/-- Bytes per reading in the scan array: the range pattern (4 bytes), then
the `x` and `y` coordinates of its point in metres as binary64 patterns (8
bytes each), all little-endian. -/
def recordSize : Nat := 20

/-- Record `i` of the array: the reading's range pattern and the patterns of
its two coordinates, unconverted. -/
def beamAt (a : ByteArray) (i : Nat) : Beam :=
  { range := u32At a (recordSize * i)
    x := u64At a (recordSize * i + 4)
    y := u64At a (recordSize * i + 12) }

/-- The beams of a scan array: `size / 20` records in order; a remainder of
fewer than 20 bytes is ignored. Total for an array of any length. -/
def decodeBeams (a : ByteArray) : List Beam :=
  (List.range (a.size / recordSize)).map (beamAt a)

/-- The scan the model sees for the arguments of `cmOnScan`. A negative stamp
becomes 0 (D8). -/
def scanOfRaw (stampNs : Int64) (rangeMinBits rangeMaxBits : UInt32) (beams : ByteArray) : Scan :=
  { stamp := stampNs.toInt.toNat, rangeMin := rangeMinBits.toNat, rangeMax := rangeMaxBits.toNat,
    beams := decodeBeams beams }

/-! ## Events -/

/-- Applies one event and records its output. An accepted monitor does what
`step` says. A monitor that was not accepted keeps its state and answers
every command in the active phase with `stopSource`, whatever the command;
it follows the lifecycle like an accepted one. -/
def Monitor.apply (m : Monitor) (e : Event) : Monitor :=
  if m.ok then
    let (st, out) := step m.cfg m.st e
    { m with st, out }
  else
    match e with
    | .cmd _ _ _ _ =>
      { m with out := if m.st.phase = .active then .answer (some .stopSource) none else .silent }
    | .scan _ _ | .shape _ _ _ => { m with out := .silent }
    | .configure | .activate | .deactivate | .cleanup | .shutdown =>
      { m with st := (step m.cfg m.st e).1, out := .silent }

/-- A scan arrived at node-clock time `nowNs`: one `step` of the model on a
`scan` event, which stores the scan. `usable` is 0 if the scan's frame is not
the base frame and no sensor pose was configured (D11); it is recorded, and
decides what transform the following commands carry (`cmdEvent`): for an
unusable scan none, so that the source has no data until a usable scan
arrives (R8). -/
@[export roslean_cm_on_scan]
def cmOnScan (m : Monitor) (nowNs : UInt64) (stampNs : Int64) (usable : UInt8)
    (rangeMinBits rangeMaxBits : UInt32) (beams : ByteArray) : Monitor :=
  { m.apply (.scan nowNs.toNat (scanOfRaw stampNs rangeMinBits rangeMaxBits beams)) with
    placed := usable != 0 }

/-- The event of the model that a command call delivers: the scan's
transform is the identity if the scan stored last was usable, and missing if
it was not; there are no zone transforms. -/
def cmdEvent (placed : Bool) (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64) : Event :=
  .cmd nowNs.toNat ⟨linX.toNat, linY.toNat, linZ.toNat, angX.toNat, angY.toNat, angZ.toNat⟩
    (if placed then some Xform.identity else none) []

/-- A velocity command arrived at node-clock time `nowNs`; the six components
are binary64 bit patterns. -/
@[export roslean_cm_on_cmd]
def cmOnCmd (m : Monitor) (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64) : Monitor :=
  m.apply (cmdEvent m.placed nowNs linX linY linZ angX angY angZ)

/-! ## Output queries -/

/-- The code of an output's kind: 0 nothing to publish (`silent`, or a
processed command whose velocity is withheld), 1 `pass`, 2 `slowdown …`,
3 `stopZone …`, 4 `stopSource`. -/
def kindOf (o : Output) : UInt8 :=
  match o.velocity with
  | none => 0
  | some .pass => 1
  | some (.slowdown _ _) => 2
  | some (.stopZone _) => 3
  | some .stopSource => 4

/-- The zone index of an output of kind 2 or 3; 0 otherwise. -/
def zoneOf (o : Output) : Nat :=
  match o.velocity with
  | some (.slowdown k _) => k
  | some (.stopZone k) => k
  | _ => 0

/-- The ratio pattern of an output of kind 2; 0 otherwise. -/
def ratioOf (o : Output) : Nat :=
  match o.velocity with
  | some (.slowdown _ r) => r
  | _ => 0

/-- The kind of the most recent event's output. -/
@[export roslean_cm_out_kind]
def cmOutKind (m : Monitor) : UInt8 := kindOf m.out

/-- The zone index of the most recent event's output. -/
@[export roslean_cm_out_zone]
def cmOutZone (m : Monitor) : UInt32 := (zoneOf m.out).toUInt32

/-- The slowdown ratio of the most recent event's output. -/
@[export roslean_cm_out_ratio]
def cmOutRatio (m : Monitor) : UInt64 := (ratioOf m.out).toUInt64

/-! ## The conversions, for tests and log messages

The node never needs these to decide anything; they expose the conversions of
`ROSleanCore.CollisionMonitorConvert` so that they can be compared with
reference computations, and so that the node can log the values in use. -/

/-- `coordMm` of a binary64 pattern. -/
@[export roslean_cm_coord_mm]
def cmCoordMm (bits : UInt64) : Int64 := Int64.ofInt (coordMm bits.toNat)

/-- `radiusMm`, or -1 if the value is refused. -/
@[export roslean_cm_radius_mm]
def cmRadiusMm (bits : UInt64) : Int64 :=
  match radiusMm bits.toNat with
  | some r => Int64.ofInt r
  | none => -1

/-- `vertexMm` of a binary32 pattern, limited to ±2⁶², or the smallest
`Int64` if the value is refused. -/
@[export roslean_cm_vertex_mm]
def cmVertexMm (bits : UInt32) : Int64 :=
  match vertexMm bits.toNat with
  | some v => Int64.ofInt (max (-0x4000000000000000) (min 0x4000000000000000 v))
  | none => Int64.minValue

/-- `timeoutNs`, or -1 if the value is refused. -/
@[export roslean_cm_timeout_ns]
def cmTimeoutNs (bits : UInt64) : Int64 :=
  match timeoutNs bits.toNat with
  | some t => Int64.ofInt t
  | none => -1

/-- `sourceTimeoutNs`, or -1 if a value is refused. -/
@[export roslean_cm_source_timeout_ns]
def cmSourceTimeoutNs (nodeBits : UInt64) (hasOwn : UInt8) (ownBits : UInt64) : Int64 :=
  match sourceTimeoutNs nodeBits.toNat (hasOwn != 0) ownBits.toNat with
  | some t => Int64.ofInt t
  | none => -1

end ROSleanCore.CollisionMonitorFFI
