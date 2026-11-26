import ROSleanCore.CollisionMonitor
import ROSleanCore.CollisionMonitorConvert
import ROSleanCore.CollisionMonitorFast

/-!
# C interface to the collision monitor model

A flat API for the ROS shim, as laid down in
`docs/collision_monitor/SPEC.md`, "Shim boundary": opaque objects and scalars,
plus `ByteArray`s for readings, coordinates and vertices. The matching C++ wrapper is
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

**The calls after task T06** (`docs/collision_monitor/SPEC_NAV2.md`, section
8). A monitor starts unconfigured (`State.init`); the node delivers the
lifecycle transitions (`cmOnLifecycle`). A scan call hands over the ranges
only. A command call hands over, with the command, one pair of binary64
coordinates in the base frame per reading of the stored scan, as the shim
computed them for this command (or none), and for each topic zone its
vertices in the base frame (or none). A shape call hands over the vertices
of a polygon message in the base frame (or none).

* the command call that C sees is `cmOnCmdFast` (task T03): the same
  function as `cmOnCmd`, computed by `stepCmdFast`
  (`ROSleanCore.CollisionMonitorFast`), proved equal in
  `ROSlean/CollisionMonitor/Fast.lean`; a command without a clock reading
  is `cmOnCmdNoClock` (SPEC, D15);
* `cmCmdPlan` tells the shim which transforms upstream would look up for a
  command; it changes nothing;
* `roslean_cm_out_kind` returns 0 for `silent` and 5 for a processed command
  whose velocity is withheld; the report is read with `roslean_cm_out_report`
  and `roslean_cm_out_report_zone`.
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
  /-- The shape call made last was accepted (E7); `false` before any. -/
  accepted : Bool

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

/-- Appends a polygon zone whose shape arrives on a topic (N31): it has no
shape in the configuration. -/
@[export roslean_cm_builder_add_topic_polygon]
def builderAddTopicPolygon (b : Builder) (action : UInt8) (ratioBits : UInt64) (minPoints : Int64)
    (enabled : UInt8) : Builder :=
  { b with zones := b.zones ++ [zoneOfRaw .topic action ratioBits minPoints enabled] }

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

/-- The monitor for the configuration built so far, unconfigured
(`State.init`) and with output `silent`. It is accepted iff the source was
set, no raw value was refused, no vertex was added to no polygon, and
`Config.ok` holds. -/
@[export roslean_cm_new]
def cmNew (b : Builder) : Monitor :=
  { cfg := b.cfg, ok := b.error == 0, err := b.error, zoneErrs := b.zones.map entryError,
    st := State.init b.cfg, out := .silent, accepted := false }

/-- The configuration was accepted. The `configure` transition of the node
must fail if this is `false`. -/
@[export roslean_cm_ok]
def cmOk (m : Monitor) : Bool := m.ok

/-- `Builder.error` of the builder the monitor was made from. -/
@[export roslean_cm_error]
def cmError (m : Monitor) : UInt8 := m.err

/-- `entryError` of zone number `k`, or 5 if there is no such zone. The shim
uses `cmError` and this only to name the offending parameter. -/
@[export roslean_cm_zone_error]
def cmZoneError (m : Monitor) (k : UInt32) : UInt8 := m.zoneErrs.getD k.toNat 5

/-! ## Decoding -/

/-- Byte `i` of the array as a number; 0 beyond the end. -/
def byteAt (a : ByteArray) (i : Nat) : Nat :=
  if h : i < a.size then (a[i]'h).toNat else 0

/-- The unsigned 32-bit little-endian number at offset `o`. -/
def u32At (a : ByteArray) (o : Nat) : Nat :=
  byteAt a o + 0x100 * byteAt a (o + 1) + 0x10000 * byteAt a (o + 2) +
    0x1000000 * byteAt a (o + 3)

/-- The binary64 pattern stored little-endian at offset `o`. -/
def u64At (a : ByteArray) (o : Nat) : Nat := u32At a (o + 4) * 0x100000000 + u32At a o

/-- Bytes per reading in the array of a scan call: the range pattern,
little-endian. -/
def rangeSize : Nat := 4

/-- Bytes per coordinate pair in the arrays of a command call and of a shape
call: the `x` and `y` in metres, in the base frame, as binary64 patterns (8
bytes each), little-endian. -/
def pairSize : Nat := 16

/-- Reading `i` of a scan array: its range pattern. -/
def beamAt (a : ByteArray) (i : Nat) : Beam :=
  { range := u32At a (rangeSize * i) }

/-- Pair `i` of a coordinate array: the two patterns, unconverted. -/
def pairAt (a : ByteArray) (i : Nat) : Coords :=
  (u64At a (pairSize * i), u64At a (pairSize * i + 8))

/-- The beams of a scan array: `size / 4` readings in order; a remainder of
fewer than 4 bytes is ignored. Total for an array of any length. -/
def decodeBeams (a : ByteArray) : List Beam :=
  (List.range (a.size / rangeSize)).map (beamAt a)

/-- The pairs of a coordinate array: `size / 16` pairs in order; a remainder
of fewer than 16 bytes is ignored. -/
def decodePairs (a : ByteArray) : List Coords :=
  (List.range (a.size / pairSize)).map (pairAt a)

/-- The scan the model sees for the arguments of `cmOnScan`. A negative stamp
becomes 0 (D8). -/
def scanOfRaw (stampNs : Int64) (rangeMinBits rangeMaxBits : UInt32) (ranges : ByteArray) : Scan :=
  { stamp := stampNs.toInt.toNat, rangeMin := rangeMinBits.toNat, rangeMax := rangeMaxBits.toNat,
    beams := decodeBeams ranges }

/-- The little-endian number that a list of bytes denotes. -/
def leNat : List UInt8 → Nat
  | [] => 0
  | b :: bs => b.toNat + 256 * leNat bs

/-- `n` coordinate pairs from the front of a list of bytes, 16 bytes each. -/
def takePairs : Nat → List UInt8 → List Coords
  | 0, _ => []
  | n + 1, bs => (leNat (bs.take 8), leNat ((bs.drop 8).take 8)) :: takePairs n (bs.drop 16)

/-- The loop of `decodeZoneVerts`: at most `fuel` entries from the bytes. -/
def parseZoneVerts : Nat → List UInt8 → List (Option (List Coords))
  | 0, _ => []
  | fuel + 1, bs =>
    if bs.length < 4 then []
    else if leNat (bs.take 4) = 0xFFFFFFFF then none :: parseZoneVerts fuel (bs.drop 4)
    else if bs.length < 4 + 16 * leNat (bs.take 4) then []
    else
      some (takePairs (leNat (bs.take 4)) (bs.drop 4)) ::
        parseZoneVerts fuel (bs.drop (4 + 16 * leNat (bs.take 4)))

/-- The zone vertices of a command call: for each zone in order a 4-byte
little-endian count `n` and `16·n` bytes of vertices (pairs as in
`pairSize`), with `n = 0xFFFFFFFF` for "none". The list ends where the
array ends, or at an entry the array does not hold in full; a zone the list
does not reach has none (`refreshShapes`). Total for an array of any
length. -/
def decodeZoneVerts (a : ByteArray) : List (Option (List Coords)) :=
  parseZoneVerts a.size a.data.toList

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

/-- The lifecycle event of a transition code: 0 `configure`, 1 `activate`,
2 `deactivate`, 3 `cleanup`, 4 `shutdown` (E1–E5); none for any other
value. -/
def lifecycleEvent (t : UInt8) : Option Event :=
  if t == 0 then some .configure
  else if t == 1 then some .activate
  else if t == 2 then some .deactivate
  else if t == 3 then some .cleanup
  else if t == 4 then some .shutdown
  else none

/-- A lifecycle transition was performed: one `step` of the model on its
event. Any other code changes nothing. -/
@[export roslean_cm_on_lifecycle]
def cmOnLifecycle (m : Monitor) (transition : UInt8) : Monitor :=
  match lifecycleEvent transition with
  | some e => m.apply e
  | none => m

/-- The code of a phase: 0 unconfigured, 1 inactive, 2 active, 3
finalized. -/
def phaseCode : Phase → UInt8
  | .unconfigured => 0
  | .inactive => 1
  | .active => 2
  | .finalized => 3

/-- The phase of the model, for logs and tests. -/
@[export roslean_cm_phase]
def cmPhase (m : Monitor) : UInt8 := phaseCode m.st.phase

/-- A scan arrived at node-clock time `nowNs`: one `step` of the model on a
`scan` event. `ranges` holds the range patterns; where the readings lie in
the base frame is handed over with each command. -/
@[export roslean_cm_on_scan]
def cmOnScan (m : Monitor) (nowNs : UInt64) (stampNs : Int64)
    (rangeMinBits rangeMaxBits : UInt32) (ranges : ByteArray) : Monitor :=
  m.apply (.scan nowNs.toNat (scanOfRaw stampNs rangeMinBits rangeMaxBits ranges))

/-- The vertices a shape call hands over: none if `hasVerts` is 0 (the
transform from the message's frame could not be looked up), else the pairs
of the array. -/
def vertsOfRaw (hasVerts : UInt8) (verts : ByteArray) : Option (List Coords) :=
  if hasVerts != 0 then some (decodePairs verts) else none

/-- The event of the model that a shape call delivers. -/
def shapeEvent (zone : UInt32) (foreign hasVerts : UInt8) (verts : ByteArray) : Event :=
  .shape zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0)

/-- A polygon message for topic zone `zone` arrived (E7): one `step` of the
model on a `shape` event. `verts` holds the vertices in the base frame;
`foreign` is non-zero iff the message's frame is not the base frame. Whether
the message was accepted is recorded for `cmShapeAccepted`. -/
@[export roslean_cm_on_shape]
def cmOnShape (m : Monitor) (zone : UInt32) (foreign hasVerts : UInt8) (verts : ByteArray) :
    Monitor :=
  { m.apply (shapeEvent zone foreign hasVerts verts) with
    accepted := m.ok &&
      (acceptShape m.cfg m.st zone.toNat (vertsOfRaw hasVerts verts) (foreign != 0)).isSome }

/-- The shape call made last was accepted: the shim then keeps the message
for the lookups of later commands (N34) and for display (N16). -/
@[export roslean_cm_shape_accepted]
def cmShapeAccepted (m : Monitor) : Bool := m.accepted

/-- The state holds, for topic zone `zone`, the mark of a polygon message
with an unusable vertex (ND6): the zone demands its action until a usable
message arrives. After an accepted shape call the shim asks this to know
whether the message is one to keep for the lookups of later commands (a
shape) or not (the mark: there is nothing to look up or to display). A
query. -/
@[export roslean_cm_shape_unusable]
def cmShapeUnusable (m : Monitor) (zone : UInt32) : Bool :=
  shapeUnusable m.st.shapes zone.toNat

/-- The twist of the six patterns of a command call. -/
def twistOfRaw (linX linY linZ angX angY angZ : UInt64) : Twist :=
  ⟨linX.toNat, linY.toNat, linZ.toNat, angX.toNat, angY.toNat, angZ.toNat⟩

/-- The scan coordinates a command call hands over: none if `hasPoints` is
0, else the pairs of the array. -/
def pointsOfRaw (hasPoints : UInt8) (points : ByteArray) : Option (List Coords) :=
  if hasPoints != 0 then some (decodePairs points) else none

/-- The event of the model that a command call delivers. -/
def cmdEvent (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64) (hasPoints : UInt8)
    (points zoneVerts : ByteArray) : Event :=
  .cmd nowNs.toNat (twistOfRaw linX linY linZ angX angY angZ) (pointsOfRaw hasPoints points)
    (decodeZoneVerts zoneVerts)

/-- Which transforms upstream would look up for a command at `now` (N22,
N34): 0 none (the command is not processed, or the source is invalid before
any lookup: no scan, or a scan too old, with a non-zero timeout); 1 those of
the topic zones only (the source is disabled, or there is no scan and the
timeout is 0); 3 the scan's and the zones'. -/
def cmdPlan (cfg : Config) (st : State) (now : Nat) (v : Twist) : UInt8 :=
  if st.phase ≠ .active then 0
  else if !v.finite then 0
  else if !cfg.sourceEnabled then 1
  else
    match st.scan with
    | none => if cfg.sourceTimeout = 0 then 1 else 0
    | some s => if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout < now then 0 else 3

/-- `cmdPlan` for the command the shim is about to deliver; 0 for a monitor
that was not accepted (its answer depends on nothing). A query: the monitor
is not changed. -/
@[export roslean_cm_cmd_plan]
def cmCmdPlan (m : Monitor) (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64) : UInt8 :=
  if m.ok then cmdPlan m.cfg m.st nowNs.toNat (twistOfRaw linX linY linZ angX angY angZ) else 0

/-- A velocity command arrived at node-clock time `nowNs`; the six components
are binary64 bit patterns; `points` holds one coordinate pair per reading of
the stored scan (looked at only if `hasPoints` is not 0) and `zoneVerts` the
vertices of the topic zones (`decodeZoneVerts`). This is the definition the
theorems are stated on; the function exported to C is `cmOnCmdFast`, which
returns the same monitor (`ROSlean.CollisionMonitor.FFI.cmOnCmdFast_eq`). It
is exported under another name so that a test can run the two side by side
(`test_cm_ffi.cpp`); the node does not call it. -/
@[export roslean_cm_on_cmd_reference]
def cmOnCmd (m : Monitor) (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray) : Monitor :=
  m.apply (cmdEvent nowNs linX linY linZ angX angY angZ hasPoints points zoneVerts)

/-! ## The command call as exported -/

/-- The binary64 pattern stored little-endian at offset `o`, as a machine
word: `u64At a o` (`word64At_toNat`). Eight bytes are read after one bounds
check; an offset that leaves fewer than eight bytes takes the slow way, so
that the two agree for every offset. -/
def word64At (a : ByteArray) (o : Nat) : UInt64 :=
  if h : o + 8 ≤ a.size then
    (a[o]'(by omega)).toUInt64 + (a[o + 1]'(by omega)).toUInt64 * 0x100 +
      (a[o + 2]'(by omega)).toUInt64 * 0x10000 + (a[o + 3]'(by omega)).toUInt64 * 0x1000000 +
      (a[o + 4]'(by omega)).toUInt64 * 0x100000000 +
      (a[o + 5]'(by omega)).toUInt64 * 0x10000000000 +
      (a[o + 6]'(by omega)).toUInt64 * 0x1000000000000 +
      (a[o + 7]'(by omega)).toUInt64 * 0x100000000000000
  else UInt64.ofNat (u64At a o)

/-- `coordMagMm` on a machine word: the magnitude in whole millimetres of the
pattern `mag` (`coordMagMm64_toNat`). Every intermediate value fits 64 bits:
the significand is below 2⁵³ and 1000 below 2¹⁰. A shift by 64 bits or more,
which a machine shift does not do, is the case "less than a millimetre". -/
def coordMagMm64 (mag : UInt64) : UInt64 :=
  let e := mag / 0x10000000000000 % 0x800
  if 1033 ≤ e then 1000000
  else
    let frac := mag % 0x10000000000000
    let mant := if e < 1 then frac else frac + 0x10000000000000
    let sh := if e < 1 then 1074 else 1075 - e
    let q := if 64 ≤ sh then 0 else (mant * 1000) >>> sh
    if 1000000 ≤ q then 1000000 else q

/-- `coordMm` on a machine word (`coordMm64_eq`): sign and magnitude are
taken apart with machine operations, and the pattern, which is a big number
in Lean's runtime when the sign bit is set, never becomes a `Nat`. The
result is at most 1,000,000 in magnitude. -/
def coordMm64 (b : UInt64) : Int :=
  let mag := b % 0x8000000000000000
  let m : Int := ((coordMagMm64 mag).toNat : Int)
  let nan := mag / 0x10000000000000 % 0x800 == 0x7FF && mag % 0x10000000000000 != 0
  if 0x8000000000000000 ≤ b && !nan then -m else m

/-- `coordMm` of the binary64 pattern stored little-endian at offset `o`
(`coordMmAt_eq`). -/
def coordMmAt (a : ByteArray) (o : Nat) : Int := coordMm64 (word64At a o)

/-- The point of pair `i` of a coordinate array: `scanPointMm (pairAt a i)`
(`pointAt_eq`). -/
def pointAt (a : ByteArray) (i : Nat) : Point :=
  ⟨coordMmAt a (pairSize * i), coordMmAt a (pairSize * i + 8)⟩

/-- `Monitor.apply` on the command event that carries the coordinates of the
array `points` (none if `hasPoints` is false), computed by `stepCmdFast`:
the coordinates are never decoded into a list. -/
def Monitor.applyCmdFast (m : Monitor) (now : Nat) (v : Twist) (hasPoints : Bool)
    (points : ByteArray) (zoneVerts : List (Option (List Coords))) : Monitor :=
  if m.ok then
    let (st, out) :=
      stepCmdFast m.cfg m.st now v (if hasPoints then some (points.size / pairSize) else none)
        (pointAt points) zoneVerts
    { m with st, out }
  else
    { m with out := if m.st.phase = .active then .answer (some .stopSource) none else .silent }

/-- The command call as C sees it. On every monitor it returns what `cmOnCmd`
returns (`ROSlean.CollisionMonitor.FFI.cmOnCmdFast_eq`, `execFast_eq`). -/
@[export roslean_cm_on_cmd]
def cmOnCmdFast (m : Monitor) (nowNs : UInt64) (linX linY linZ angX angY angZ : UInt64)
    (hasPoints : UInt8) (points zoneVerts : ByteArray) : Monitor :=
  m.applyCmdFast nowNs.toNat (twistOfRaw linX linY linZ angX angY angZ) (hasPoints != 0) points
    (decodeZoneVerts zoneVerts)

/-- Applies a command that arrives without a valid clock reading
(`stepNoClock`) and records its output. A monitor that was not accepted
answers as it answers every command: `stopSource` in the active phase. -/
def Monitor.applyNoClock (m : Monitor) (v : Twist) : Monitor :=
  if m.ok then
    let (st, out) := stepNoClock m.st v
    { m with st, out }
  else
    { m with out := if m.st.phase = .active then .answer (some .stopSource) none else .silent }

/-- A velocity command arrived while the node has no valid clock reading
(SPEC, D15); the six components are binary64 bit patterns. There is no time
argument, and nothing is looked up. The answer is `stopSource` whenever the
command is processed. -/
@[export roslean_cm_on_cmd_no_clock]
def cmOnCmdNoClock (m : Monitor) (linX linY linZ angX angY angZ : UInt64) : Monitor :=
  m.applyNoClock ⟨linX.toNat, linY.toNat, linZ.toNat, angX.toNat, angY.toNat, angZ.toNat⟩

/-! ## Output queries -/

/-- The code of an output's kind: 0 `silent` (nothing is published, on any
topic); 1 `pass`, 2 `slowdown …`, 3 `stopZone …`, 4 `stopSource`; 5 a
processed command whose velocity is withheld (R15). -/
def kindOf : Output → UInt8
  | .silent => 0
  | .answer none _ => 5
  | .answer (some .pass) _ => 1
  | .answer (some (.slowdown _ _)) _ => 2
  | .answer (some (.stopZone _)) _ => 3
  | .answer (some .stopSource) _ => 4

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

/-- The code of an output's report: 0 none; 1 `normal`; 2 `stop`; 3
`slowdown`; 4 `source` (N13). -/
def reportOf (o : Output) : UInt8 :=
  match o.report with
  | none => 0
  | some .normal => 1
  | some (.stop _) => 2
  | some (.slowdown _) => 3
  | some .source => 4

/-- The zone index of a report of code 2 or 3; 0 otherwise. -/
def reportZoneOf (o : Output) : Nat :=
  match o.report with
  | some (.stop k) => k
  | some (.slowdown k) => k
  | _ => 0

/-- The report of the most recent event's output. -/
@[export roslean_cm_out_report]
def cmOutReport (m : Monitor) : UInt8 := reportOf m.out

/-- The zone index of the report of the most recent event's output. -/
@[export roslean_cm_out_report_zone]
def cmOutReportZone (m : Monitor) : UInt32 := (reportZoneOf m.out).toUInt32

/-! ## The conversions, for tests and log messages

The node never needs these to decide anything; they expose the conversions of
`ROSleanCore.CollisionMonitorConvert` so that they can be compared with
reference computations, and so that the node can log the values in use. -/

/-- `coordMm` of a binary64 pattern. -/
@[export roslean_cm_coord_mm]
def cmCoordMm (bits : UInt64) : Int64 := Int64.ofInt (coordMm bits.toNat)

/-- `vertexCoordMm` of a binary64 pattern: a topic zone's vertex coordinate
in millimetres (rounded to nearest; ±1,000,000 if not finite or 1024 m or
more). -/
@[export roslean_cm_vertex_coord_mm]
def cmVertexCoordMm (bits : UInt64) : Int64 := Int64.ofInt (vertexCoordMm bits.toNat)

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
