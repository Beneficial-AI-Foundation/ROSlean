# Collision monitor core: specification

Taken from the upstream C++ source of `nav2_collision_monitor`,
`ros-navigation/navigation2`, branch `jazzy` at commit `c704655` (package
version 1.3.14). The files cited below are byte-identical in tags 1.3.12 and
1.3.13, the versions `pixi.lock` resolves to. Paths are relative to
`nav2_collision_monitor/` unless they start with `nav2_util/` or `rclcpp/`.

Rule numbers `R…`, differences `D…` and assumptions `A…` are referenced from
the Lean model, `core/ROSleanCore/CollisionMonitor.lean`, and its examples,
`ROSlean/CollisionMonitor/Examples.lean`.

Scope of this specification: one laser scan source, one velocity command
input, circle zones and fixed polygon zones, the `stop` and `slowdown`
actions. Rules are stated for that subset; what upstream does beyond it is
listed under "Excluded features".

## Rules as implemented upstream

**Triggers**

- R1. The only thing that makes the node compute or publish is a message on
  the velocity input (`CollisionMonitor::cmdVelInCallback*` →
  `process`, `src/collision_monitor_node.cpp`). A scan message is only stored
  (`Scan::dataCallback`, `src/scan.cpp`). There is no timer. When commands
  cease, nothing is published, whatever the scan shows; when the scan goes
  stale, nothing is published until the next command arrives.
- R2. A command in which any of the six components is infinite or NaN is
  ignored: an error is logged, nothing is published and no state changes
  (`cmdVelInCallbackStamped`; `validateTwist`,
  `nav2_util/src/robot_utils.cpp`). The check covers `linear.z`, `angular.x`
  and `angular.y` although the node uses only `linear.x`, `linear.y` and
  `angular.z`.

**Scan data**

- R3. The points of a scan are computed when a command is processed
  (`Scan::getData`). With `float angle = angle_min`, for each index `i` of
  `ranges` in order: if reading `i` is valid (R4), the point is
  `(ranges[i] * cos(angle), ranges[i] * sin(angle))`, computed in single
  precision and then transformed to the base frame; in every case
  `angle += angle_increment` follows, also in single precision. The number of
  readings is `ranges.size()`. `angle_max`, `intensities`, `time_increment`
  and `scan_time` are not read, and the length of `ranges` is never compared
  with the angle fields.
- R4. Reading `i` is valid iff `ranges[i] >= range_min && ranges[i] <=
  range_max`, as `float` comparisons (`Scan::getData`). Therefore: a NaN
  reading is invalid; a reading of `+inf` is valid only if `range_max` is
  `+inf`; if `range_min` or `range_max` is NaN no reading is valid; a reading
  of `0` is valid if `range_min <= 0`. Invalid readings are skipped one by
  one. No property of the scan as a whole is checked: an empty scan, or one
  with no valid reading, is a scan with no obstacles.
- R5. The node keeps the scan received last (`data_ = msg`). It is not
  the one with the newest stamp: a scan that arrives late replaces a newer
  one.
- R6. A source has a parameter `enabled`. A disabled source is skipped: it
  contributes no points and cannot cause a stop (`process`, first loop).
- R7. A stored scan is *too old* at node-clock time `now` iff the source
  timeout is not zero and `now − header.stamp > source_timeout`, strictly, in
  integer nanoseconds (`Source::sourceValid`, `src/source.cpp`). The age is
  measured from the stamp in the message header, not from the arrival time.
  A scan stamped in the future is never too old. The timeout is converted
  from seconds by `rclcpp::Duration::from_seconds`, which multiplies by 10⁹
  and truncates (`rclcpp/src/rclcpp/duration.cpp`), so a positive timeout
  below 1 ns is zero.
- R8. If the source is enabled and its timeout is not zero, and either no
  scan has been received yet, or the stored scan is too old, or its transform
  to the base frame cannot be obtained, the action is *stop* with the name
  `invalid source`, and no zone is evaluated (`process`, first loop). If the
  timeout is zero, the same conditions only mean that the source contributes
  no points.

**Zones**

- R9. Circle (`Circle::getPointsInside`, `src/circle.cpp`): a point is inside
  iff `x² + y² < radius²`, strictly, around the base frame's origin. The
  radius is the parameter `radius`; its sign is lost by the squaring.
- R10. Polygon (`Polygon::isPointInside`, `src/polygon.cpp`): ray crossing.
  With the vertices in order and each edge running from the previous vertex
  `a` to the vertex `b` (the first edge from the last vertex to the first),
  an edge is crossed iff `(p.y <= a.y) == (p.y > b.y)` and
  `a.x + (p.y − a.y)·(b.x − a.x)/(b.y − a.y) > p.x`. The point is inside iff
  the number of crossed edges is odd. Consequences: horizontal edges are never
  crossed; points on the boundary are inside on some sides and outside on
  others (for the axis-parallel box of upstream's example, inside on the sides
  with the largest `y` and the smallest `x`, outside on the other two).
  The vertices come from the string parameter `points`, parsed as `float`
  (`nav2_util/src/array_parser.cpp`) and widened to `double`. A polygon with
  fewer than **four** vertices is refused (`Polygon::getPolygonFromString`:
  `vvf.size() <= 3`), although the log message says "at least three".
- R11. A zone has parameters `enabled` (default `true`) and `min_points`
  (default 4). A disabled zone is skipped. An enabled zone *triggers* iff the
  number of points inside is at least `min_points`
  (`processStopSlowdownLimit`: `>=`). With `min_points <= 0` a zone triggers
  without any point. The deprecated parameter `max_points`, if set, replaces
  `min_points` by `max_points + 1` (`Polygon::getCommonParameters`).

**Action**

- R12. Zones are visited in the order of the `polygons` parameter. A
  triggered `stop` zone sets the action to *stop* with that zone's name, and
  the loop ends before the next zone (`process`, second loop). So the first
  triggered stop zone wins over everything, including slowdown zones before
  it.
- R13. A triggered `slowdown` zone proposes the velocity `command ×
  slowdown_ratio`: the ratio is always applied to the incoming command, never
  to an already reduced velocity. The proposal replaces the current one iff
  its squared norm `x² + y² + tw²` is strictly smaller (`Velocity::operator<`,
  `include/nav2_collision_monitor/types.hpp`). Starting from the command
  itself, the result is: the triggered slowdown zone with the smallest
  `|slowdown_ratio|` wins, if that is below 1; of two equal ratios, the
  earlier zone; a ratio of 1 or more, infinite or NaN never takes effect.
  `slowdown_ratio` (default 0.5) is not range-checked: a negative ratio above
  −1 takes effect and **reverses** the command.
- R14. If no zone changes the action, the velocity is the command.
- R15. `publishVelocity`: let *zero* mean that the three components of the
  velocity to publish are all `== 0.0`. If the velocity is zero and the
  previous one (initially non-zero) was not, the current time is stored as
  the stop stamp (initially 0) and the velocity is published. If both are
  zero, it is published iff `now − stop_stamp > stop_pub_timeout` is false.
  A non-zero velocity is always published. The previous velocity is updated
  on every processed command, published or not. So zero is published for
  `stop_pub_timeout` (default 1 s) after it first appears, and then nothing
  is published until a non-zero velocity occurs. This applies in the same way
  to a stop demanded by a zone, to a stop for an invalid source, and to a
  command that is itself zero.
- R16. The published message is new: `linear.x`, `linear.y` and `angular.z`
  are the computed velocity, and `linear.z`, `angular.x` and `angular.y` are
  0. With `enable_stamped_cmd_vel` the header of the command is copied.

**Interface**

- R17. Parameters of the subset, with upstream's defaults:

  | Parameter | Type | Default |
  | --- | --- | --- |
  | `cmd_vel_in_topic` | string | `cmd_vel_smoothed` |
  | `cmd_vel_out_topic` | string | `cmd_vel` |
  | `enable_stamped_cmd_vel` | bool | `false` |
  | `source_timeout` | double, s | 2.0 |
  | `stop_pub_timeout` | double, s | 1.0 |
  | `observation_sources` | string array | none: required |
  | `<source>.type` | string | `scan` |
  | `<source>.topic` | string | `scan` |
  | `<source>.enabled` | bool | `true` |
  | `<source>.source_timeout` | double, s | the node's `source_timeout` |
  | `polygons` | string array | none: required |
  | `<zone>.type` | string | none: required (`circle`, `polygon`) |
  | `<zone>.radius` | double, m | circle: required |
  | `<zone>.points` | string `"[[x, y], …]"`, m | polygon: required |
  | `<zone>.action_type` | string | none: required (`stop`, `slowdown`) |
  | `<zone>.min_points` | integer | 4 |
  | `<zone>.slowdown_ratio` | double | 0.5 |
  | `<zone>.enabled` | bool | `true` |
  | `<zone>.sources_names` | string array | all of `observation_sources` |

  A parameter of the wrong YAML type makes configuration fail.
- R18. Topics: the command input is `Twist`, or `TwistStamped` with
  `enable_stamped_cmd_vel`, QoS keep-last 1; the output has the same type and
  QoS (`nav2_util/twist_subscriber.hpp`, `twist_publisher.hpp`). The scan
  subscription uses `SensorDataQoS`. The node's name is `collision_monitor`.
- R19. Upstream is a lifecycle node. While it is not active, commands are
  dropped without output. It also has a service `~/toggle` that switches all
  zones off (the invalid-source stop stays), and `enabled` of a zone or source
  can be changed at run time. Callbacks run one at a time
  (`rclcpp::spin`).

## Things the source does that the documentation does not say

Compared with the package README and the comments in the example parameter
file. `docs.nav2.org` could not be reached while this was written, so the
configuration guide was not compared.

- The node is not a watchdog on its own: without incoming commands it
  publishes nothing (R1), and after `stop_pub_timeout` of continuous zero it
  stops publishing zero (R15). `stop_pub_timeout` is not a timer.
- The README says a zone acts "if more than `N` points" are inside; the code
  uses at least `min_points` (R11).
- Scan age is taken from the header stamp. A sensor whose clock is ahead of
  the node's never times out; one whose clock is behind by more than the
  timeout always stops the robot (R7).
- A scan whose stamp has negative seconds makes upstream throw in the command
  callback (`rclcpp::Time` refuses it, `rclcpp/src/rclcpp/time.cpp`); nothing
  catches it, so the process ends (D8).
- There is no validation of a scan as a whole, and an all-NaN or empty scan
  reads as free space (R4).
- `source_timeout: 0` switches off not only the age check but also the stop
  before the first scan (R8).
- A negative `slowdown_ratio` reverses the robot (R13). Slowdown is skipped
  when the squared norms underflow to zero or overflow (command components
  below about 10⁻¹⁶² or above 10¹⁵⁴ in magnitude), because the comparison in
  R13 is then false.
- A polygon needs four vertices, not three (R10). Its boundary belongs to it
  on some sides only.
- The output drops `linear.z`, `angular.x` and `angular.y` (R16), yet a NaN
  in one of them discards the command (R2).
- `min_points: 0` makes a zone trigger always (R11).

## Units and representation in the Lean core

No `Float` occurs in the core. Three kinds of integers are used.

| Quantity | Representation | Conversion at the boundary |
| --- | --- | --- |
| Node clock, scan stamp, timeouts | `Nat`, nanoseconds | Upstream's own integers. Timeouts: seconds × 10⁹ truncated, as upstream (R7) |
| Scan points, polygon vertices | `Int`, millimetres, base frame | Rounded; see "Safe direction" |
| Circle radius | `Nat`, millimetres | Rounded up, plus 1 |
| Scan ranges, `range_min`, `range_max` | `Nat`: binary32 bit pattern | None: bits copied |
| Command components | `Nat`: binary64 bit pattern | None: bits copied |
| Slowdown ratio | `Nat`: binary64 bit pattern | None: bits copied |
| `min_points` | `Nat` | Negative values become 0 (same behaviour, R11) |

Values that are only compared or classified are passed as bit patterns
because that needs no rounding: `F32.le` is IEEE `<=` on patterns, `F64.isFinite`
and `F64.isZero` are the tests of R2 and R15, and for non-negative doubles the
order of the patterns is the order of the values, which is all R13 needs.
These definitions were compared with the hardware operations on random and
special patterns, with no disagreement (`review/collision_monitor/Checks.lean`).
`ROSlean/CollisionMonitor/Bits.lean` proves them against the IEEE-754 formula
for the number a pattern denotes (`F32.le` is `≤` on the denoted numbers and
false for NaN; a pattern below `F64.one` denotes a number in `[0, 1)`); what
stays trusted is listed in `THEOREMS.md`.

Bits above the format's width are ignored by `F32.le`, `F64.isFinite` and
`F64.isZero`. A slowdown ratio is compared as a natural number, so a pattern
of 2⁶⁴ or more is never selected rather than read modulo 2⁶⁴; `Config.ok`
refuses it and the C interface cannot produce it.

### Safe direction

"Safe" means: the integer model never misses an obstacle that the configured
zone contains, and never treats data as fresher than it is.

- **Ranges.** Not rounded. R4 is applied to the received bits.
- **Times.** Not rounded. The source timeout is truncated to nanoseconds as
  upstream; truncation makes it shorter, which is the strict side.
- **Scan points.** The shim computes each point in floating point exactly as
  R3 does, multiplies by 1000 and truncates each coordinate toward zero. A
  point therefore never moves away from the base frame's origin, and moves by
  less than 1 mm per axis.
- **Circle.** The radius in millimetres is rounded up and 1 is added (the 1
  covers the rounding of the multiplication by 1000). Together with the
  truncation of points: every point strictly inside the configured circle is
  inside the integer circle. In the other direction, a point up to 3.5 mm
  outside the configured radius can count as inside.
- **Polygon.** Truncation toward the origin is not a safe direction for a
  polygon: a zone that does not contain the origin loses points across its
  near edge. Vertices are instead rounded to the nearest millimetre (moved by
  at most 0.71 mm), points are truncated as above (moved by less than
  1.42 mm), and the model counts a point as inside if it is inside the integer
  polygon by R10 **or within √5 ≈ 2.24 mm of its boundary** (`nearBoundary`).
  Since 0.71 + 1.42 < 2.24, every point that R10 in exact arithmetic puts
  inside the configured polygon counts as inside in the model: if the rounded
  point is not inside the rounded polygon, the boundary passed over the point
  while both were being rounded, so the point is within 2.13 mm of the rounded
  boundary. In the other direction, a point up to 4.4 mm outside the
  configured polygon can count as inside. Of this argument the quantitative
  half is proved (`nearSegment_iff`: the margin test is exactly "at most √5 mm
  from the segment"; `nearBoundary_of_rounded`: a point of the configured
  boundary is, after rounding and truncation, within the margin). The
  topological half, "the boundary passed over the point", is prose; see
  `THEOREMS.md`.
- **Far and non-finite points.** A coordinate that is NaN, or whose magnitude
  is 1000 m or more, is passed as ±1,000,000 mm (NaN as +1,000,000).
  `Config.ok` requires every zone to stay inside that limit (circle radius at
  most 1,000,000 mm; polygon vertices at most 999,997 mm from each axis), so a
  saturated point is in no zone. Upstream agrees: a point with a NaN or
  infinite coordinate is inside no circle and no polygon.
- **Velocities.** Not converted at all; see the next section. Zero is the
  literal `0.0`.

## Where the velocity is computed

The core returns an **action**; the shim applies it to the floating-point
command. The core does not compute the output velocity in integers.

| Action | The shim publishes |
| --- | --- |
| `pass` | `linear.x`, `linear.y`, `angular.z` copied from the command, bit for bit |
| `stopZone k`, `stopSource` | 0.0 in all components |
| `slowdown k r` | each of the three components multiplied by the double whose bit pattern is `r` |

In every case `linear.z`, `angular.x`, `angular.y` are 0.0 as upstream (R16),
and the header is copied if messages are stamped.

Reasons:

1. Pass-through and stop are exact. No value goes through integers and back,
   so "the output equals the input" and "the output is zero" are statements
   about the published bits.
2. The core still sees the command, as six bit patterns, and itself decides
   what upstream decides from its value: whether it is finite (R2) and
   whether it is zero (R15). The shim performs no floating-point test.
3. The only floating-point operations left on the command path are the three
   multiplications of a slowdown. The ratio is handed back by the core as the
   same bit pattern it was configured with, so the shim holds no table of
   ratios and cannot pick the wrong one.
4. With velocities converted to integers the theorems would be about integer
   values on both sides of two rounding steps, the published value would
   differ from upstream's in the last digits, and pass-through would need a
   special case anyway.

What remains trusted, i.e. is not covered by a theorem about `step`:

- T1. *The slowdown multiplication.* For a finite double `x` and a double
  `r` with `0 ≤ r < 1`, the IEEE-754 product `x * r` rounded to nearest has
  the sign of `x` or is zero, its magnitude is at most `|x|` and within
  half a unit in the last place of `r·|x|`, and it does not grow when `r`
  is made smaller (the last is what "Slowdown bound" needs). The core guarantees the premise:
  a `slowdown` action always carries a pattern below `F64.one`.
- T2. *The scan conversion.* The computation of R3 in floating point (angle
  accumulation, `cos`, `sin`, the product, the fixed sensor pose of D11) and
  the saturating truncation to millimetres. The core sees the result only.
- T3. *The configuration conversion.* Radius and vertices to millimetres,
  timeouts to nanoseconds, the parsing of `points`.
- T4. *The bit-pattern definitions* `F32.le`, `F64.isFinite`, `F64.isZero`
  mean what IEEE-754 says. Proved against the formula for the number a
  pattern denotes (`Bits.lean`); trusted: that formula (`magVal`), and that
  the compiler's `float` and `double` are binary32 and binary64.
- T5. *The shim* delivers every message as one event with one clock reading,
  passes the fields named under "Shim boundary" unaltered, and publishes what
  the table above says and nothing else.

T1 could be removed later by multiplying in the core on bit patterns (a
correctly rounded binary64 product in integer arithmetic, about fifty lines)
and returning the output bits. It was left out because the proof of "no
amplification" for a hand-written multiplication is a project of its own.

## The model

`core/ROSleanCore/CollisionMonitor.lean`, namespace
`ROSleanCore.CollisionMonitor`.

- `Config`: `zones : List Zone`, `sourceEnabled`, `sourceTimeout`,
  `stopPubTimeout`. `Zone`: `shape` (`circle radius` or `polygon vertices`),
  `action` (`stop` or `slowdown ratio`), `minPoints`, `enabled`.
- `State`: `scan : Option Scan` (R5), `prevZero : Bool`, `stopStamp : Nat`
  (R15). `State.init` is no scan, `false`, 0.
- `Event`: `scan now s` or `cmd now v`. `Scan` is `stamp`, `rangeMin`,
  `rangeMax` and `beams`, each beam a range pattern with its point. `Twist` is
  six patterns.
- `Output`: `silent` or `publish a`, with `a : Action` one of `pass`,
  `slowdown zone ratio`, `stopZone zone`, `stopSource`.
- `step : Config → State → Event → State × Output`, and `run` over a list of
  events.

`step` on a command is built from named parts, so that properties can be
stated on each:

| Definition | Meaning | Rules |
| --- | --- | --- |
| `Scan.points s` | points of the valid readings | R4 |
| `sourcePoints cfg st now` | `some pts`: the points to test; `none`: invalid source | R5–R8 |
| `Shape.contains`, `Zone.count`, `Zone.triggered` | zone tests | R9–R11, D3, D4 |
| `firstStop`, `bestSlowdown`, `zonesAction cfg pts` | action for given points | R12–R14 |
| `evaluate cfg st now` | action for a command at `now`; independent of the command | R8, R12–R14 |
| `Action.yieldsZero a v` | the velocity to publish is zero | R15, D6 |
| `Config.ok` | the configuration is accepted | D5, D12, R10 |

## Properties

Proved about `step` and lifted to `run` in `ROSlean/CollisionMonitor/`; the
theorem for each is named in [THEOREMS.md](THEOREMS.md). `cfg`, `st`, `now`,
`v` are arbitrary unless restricted; no property needs `Config.ok`. "The
output of the command" is `(step cfg st (.cmd now v)).2`. A zone *is
triggered by `pts`* means `z.triggered pts = true`.

**Output.** The output of a command is `.silent` iff `v` is not finite, or
`(evaluate cfg st now).yieldsZero v` and `st.prevZero` and `st.stopStamp +
cfg.stopPubTimeout < now`; otherwise it is `.publish (evaluate cfg st now)`.
The properties below that end in "or `.silent`" are to be read with this:
silence occurs in these two cases and in no other.

**Only if.** `evaluate cfg st now` is `.stopSource` iff the source is
invalid; `.stopZone k` iff the source is valid and `k` is the first
triggered stop zone; `.slowdown k r` iff the source is valid, no stop zone is
triggered, and `k` is the first triggered slowdown zone with the smallest
ratio pattern `r`, which is below `F64.one`; `.pass` iff the source is valid
and neither kind is triggered.

**State.** After a finite command, `prevZero` is whether the demanded
velocity was zero, and `stopStamp` is `now` if it was zero and the previous
one was not, and unchanged otherwise. On runs from `State.init`: if a finite
command gets no answer, the message published last before it was a zero.
(Not: "a stop is published whenever it is demanded". After zero commands
for longer than `stop_pub_timeout`, an obstacle entering a stop zone together
with a non-zero command is answered with silence; the last published message
is the earlier zero. Example in `Examples.lean`.)

**Monotonicity.** Adding points never turns a stop into anything else,
and never turns a slowdown into `pass` or into a slowdown with a larger
ratio.

**Stop.** If `sourcePoints cfg st now = some pts` and some zone of
`cfg.zones` with action `stop` is triggered by `pts`, then
`evaluate cfg st now = .stopZone k`, where `k` is the index of the first such
zone; and the output of the command is `.publish (.stopZone k)` or
`.silent`, the latter exactly in the two cases of **Output**. In particular
it is never `.publish .pass` or `.publish (.slowdown …)`.

In words: if at least `min_points` valid points of the stored scan lie in an
enabled stop zone, the node publishes exactly zero or nothing. "Nothing"
occurs for a non-finite command (R2) and after `stop_pub_timeout` of
continuous zero (R15). The literal statement "the output velocity is exactly
zero" is therefore **false for upstream and for the model** when read as
"zero is published": counterexample in `Examples.lean`, the command at
2001 ms in the `stop_pub_timeout` example. See A3.

The hypothesis `sourcePoints … = some pts` unfolds to: the source is enabled,
a scan `s` is stored, it is not too old, and `pts = s.points`; or the source
is disabled or (timeout 0 and no scan) and `pts = []`, in which case only a
zone with `minPoints = 0` triggers.

**Stale data.** If `cfg.sourceEnabled`, `0 < cfg.sourceTimeout`, and either
`st.scan = none` or `st.scan = some s` with `s.stamp + cfg.sourceTimeout <
now`, then `evaluate cfg st now = .stopSource`, and the output of the command
is `.publish .stopSource` or `.silent`, the latter exactly in the two cases
of **Output**.

"Older than `source_timeout`" is by header stamp (R7), and "newest scan" is
the scan received last (R5). The property is **false without the two
hypotheses on the configuration**: with `source_timeout: 0` or a disabled
source, stale or missing data does not stop the robot (examples
`cfgNoTimeout`, `cfgSourceOff`). That is upstream's documented way of
switching the check off. On event sequences: after `run`, `st.scan` is the
scan of the last `scan` event, or `none` if there was none.

**Bad data.** The rule is R4 for readings (`Scan.validBeam`: a reading counts
iff `F32.le rangeMin range && F32.le range rangeMax`), R2 for commands
(`Twist.finite`), and D8 for a negative stamp. Statements: (a) readings that
are not valid never change a decision: the output of every command is the
same when they are removed from the stored scan, whatever points they carry
(the earlier wording, "does not occur in `Scan.points`", was the definition
restated, and false for a point shared with a valid reading);
(b) the output of a command that is not finite is `.silent` and the state is
unchanged; (c) `step` is total, and so are the conversions of the C interface
(a byte array of any length is a scan). Upstream itself does not satisfy
"never crashes": see D8.

**No amplification.** If the output of a command is `.publish a`, then `a`
is `.pass`, `.stopZone _`, `.stopSource`, or `.slowdown k r` with
`r < F64.one`; in the last case zone `k` of `cfg.zones` exists and has action
`.slowdown r`. With the table of the previous section and T1 this gives: on
each of the three axes the published magnitude is at most the commanded one
and the sign does not flip; the other three axes are 0. This holds for every
configuration, because a ratio pattern above `F64.one` (a ratio above 1,
negative, infinite or NaN) is never selected. It is **false for upstream**
with a negative ratio: `slowdown_ratio: -0.5` publishes `−0.5 ×` the command
(D5).

**Slowdown bound.** If `sourcePoints cfg st now = some pts`, and a zone of
`cfg.zones` with action `.slowdown rz` is triggered by `pts`, and
`rz < F64.one`, then `evaluate cfg st now` is `.stopZone _`, or
`.slowdown k r` with `r ≤ rz`. Moreover `r` is the smallest ratio pattern
among the triggered slowdown zones and `k` the first zone that has it. With
T1: the published magnitude on each axis is at most that of `command × rz`
computed in floating point, for *every* triggered slowdown zone. If
`rz = F64.one` the action may be `.pass`, which satisfies the bound with
equality.

**Pass-through.** If `v.finite`, `sourcePoints cfg st now = some pts` and no
zone of `cfg.zones` is triggered by `pts`, then `evaluate cfg st now =
.pass`, and the output of the command is `.publish .pass`, unless
`v.planarZero` and `st.prevZero` and `st.stopStamp + cfg.stopPubTimeout <
now`, in which case it is `.silent`. In particular a non-zero command is
always published unchanged. This rules out the always-stop node. The
exception is upstream's: a zero command is dropped like any other zero after
`stop_pub_timeout` (R15; example with `halt`).

**Totality.** `step` and `run` are total functions; Lean accepts them without
`partial` or `unsafe`.

Two more statements are needed to connect these to event sequences and are
cheap: `(step cfg st (.scan now s)).1.scan = some s` with the other fields
unchanged, and `(step cfg st (.cmd now v)).1.scan = st.scan`.

## Shim boundary

Lean's C interface passes scalars and Lean objects, never C structs. The
interface follows the pattern of `core/ROSleanCore/FFI.lean`: an opaque
builder, an opaque monitor object holding configuration, state and the last
output, and scalar queries. Function names are proposals; the arguments,
their types, their order and their meaning are the contract. Every Lean
object argument is consumed (PLAN, lesson 8).

Integer conversions on the Lean side: a signed argument that must be
non-negative is converted with `.toInt.toNat`, so a negative value becomes 0;
unsigned arguments are converted with `.toNat`.

### Start-up

The shim reads the parameters of R17, refuses what the tables under
"Excluded features" and "Deliberate differences" tell it to refuse, converts,
and calls:

| Call | Arguments | Meaning |
| --- | --- | --- |
| `roslean_cm_builder_new` | `()` | Empty builder: no zones, source enabled, both timeouts 0 |
| `roslean_cm_builder_set_source` | builder, `enabled : uint8`, `timeout_ns : int64` | `sourceEnabled`, `sourceTimeout` |
| `roslean_cm_builder_set_stop_pub_timeout` | builder, `timeout_ns : int64` | `stopPubTimeout` |
| `roslean_cm_builder_add_circle` | builder, `radius_mm : int64`, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a circle zone |
| `roslean_cm_builder_add_polygon` | builder, `action : uint8`, `ratio_bits : uint64`, `min_points : int64`, `enabled : uint8` | Appends a polygon zone with no vertices yet |
| `roslean_cm_builder_add_vertex` | builder, `x_mm : int64`, `y_mm : int64` | Appends a vertex to the zone added last; no effect if that is not a polygon |
| `roslean_cm_new` | builder | The monitor, in `State.init`, output `silent` |
| `roslean_cm_ok` | monitor | `uint8`: `Config.ok`. If 0 the node logs a fatal error and exits |

`action` is 0 for `stop` and 1 for `slowdown`; any other value is `stop`.
`ratio_bits` is ignored for `stop`. Zones are added in the order of the
`polygons` parameter; the zone index in an output is the position in that
order, from 0.

Conversions the shim performs (T3), all on finite doubles it has already
range-checked:

- `timeout_ns = (int64_t)(seconds * 1e9)`, the expression of
  `rclcpp::Duration::from_seconds`.
- `radius_mm = (int64_t)std::ceil(radius * 1000.0) + 1`.
- Vertices: the string is parsed with upstream's grammar into `float`s
  (`parseVVF`), each widened to `double`, and `x_mm = std::llround(x *
  1000.0)`, likewise `y_mm`.
- `ratio_bits`: the eight bytes of the `double` parameter (`memcpy`).
- `min_points`: the 64-bit parameter value.

### Scan event

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_scan` | monitor, `now_ns : uint64`, `stamp_ns : int64`, `range_min_bits : uint32`, `range_max_bits : uint32`, `beams : ByteArray` → monitor |

- `now_ns`: the node clock, read once in the callback.
- `stamp_ns = (int64_t)header.stamp.sec * 1000000000 + header.stamp.nanosec`.
  A negative value becomes 0 in Lean (D8).
- `range_min_bits`, `range_max_bits`: the four bytes of the `float` fields.
- `beams`: a Lean `ByteArray` of `12 × ranges.size()` bytes. Record `i`, at
  offset `12·i`, describes `ranges[i]`:

  | Bytes | Content |
  | --- | --- |
  | 0–3 | the bit pattern of `ranges[i]`, unsigned 32-bit, little-endian |
  | 4–7 | `x_mm`, signed 32-bit two's complement, little-endian |
  | 8–11 | `y_mm`, likewise |

  Lean decodes `size / 12` records and ignores a remainder. A record is
  produced for every reading, valid or not; validity is decided in Lean (R4).

The point of reading `i` (T2), in this order of operations:

```
float angle = msg.angle_min;
for (size_t i = 0; i < msg.ranges.size(); i++) {
  double x = msg.ranges[i] * std::cos(angle);   // float × float, then widened
  double y = msg.ranges[i] * std::sin(angle);
  if (pose is not (0, 0, 0)) {                  // D11
    double xb = px + cos_yaw * x - sin_yaw * y;
    double yb = py + sin_yaw * x + cos_yaw * y;
    x = xb; y = yb;
  }
  record i = { bits(msg.ranges[i]), to_mm(x), to_mm(y) };
  angle += msg.angle_increment;                 // float
}

int32_t to_mm(double v) {
  double t = v * 1000.0;
  if (std::isnan(t) || t >= 1e6) return 1000000;
  if (t <= -1e6) return -1000000;
  return (int32_t)std::trunc(t);
}
```

Nothing is returned that the shim acts on: the output of a scan event is
always `silent`.

### Command event

| Call | Arguments |
| --- | --- |
| `roslean_cm_on_cmd` | monitor, `now_ns : uint64`, `lin_x`, `lin_y`, `lin_z`, `ang_x`, `ang_y`, `ang_z : uint64` → monitor |
| `roslean_cm_out_kind` | monitor → `uint8` |
| `roslean_cm_out_zone` | monitor → `uint32` |
| `roslean_cm_out_ratio` | monitor → `uint64` |

- `now_ns`: the node clock, read once at the start of the callback (D7).
- The six `uint64` are the bytes of the six `double` components of the twist
  (`memcpy`). The shim does not test them.
- `out_kind`: 0 `silent`, 1 `publish pass`, 2 `publish (slowdown zone
  ratio)`, 3 `publish (stopZone zone)`, 4 `publish stopSource`.
  `out_zone` is the zone index for kinds 2 and 3 and 0 otherwise; `out_ratio`
  is the ratio pattern for kind 2 and 0 otherwise.
- The shim then publishes as the table in "Where the velocity is computed"
  says: nothing for kind 0; for kind 2 it copies `out_ratio` into a `double`
  and multiplies. For kinds 3 and 4 it may log the zone name or
  `invalid source`. It keeps no state of its own between events.

The C interface should come with theorems, as `ROSlean/Mux/FFI.lean` has,
that a sequence of calls yields `run` of the corresponding events and that
the decoding of `beams` is total.

## Excluded features

What the Lean node does when a configuration asks for a feature outside the
subset. "Refused" means: a fatal log message that names the parameter, and
exit code 1, before any subscription is created.

| Feature | Requested by | Lean node |
| --- | --- | --- |
| Several sources, or none | `observation_sources` with other than one name | Refused |
| Point cloud, range, polygon sources | `<source>.type` other than `scan` | Refused |
| Costmap source | not in Jazzy | — |
| `approach`, `limit` actions | `<zone>.action_type` | Refused |
| Zone that does nothing | `action_type: none` | Refused |
| Velocity polygons | `<zone>.type: velocity_polygon` | Refused |
| Dynamic polygon or radius | `points` / `radius` absent (upstream then needs `polygon_sub_topic`) | Refused. `polygon_sub_topic` next to a static shape is ignored, as upstream |
| Footprint subscription | only with `approach` | Refused with it |
| Zone bound to a subset of sources | `<zone>.sources_names` other than the one source | Refused |
| Deprecated `max_points` | `<zone>.max_points` set | Refused (D10) |
| Motion correction of scan points | `base_shift_correction` not explicitly `false` (upstream's default is `true`) | Refused (D11) |
| Transforms | `base_frame_id`, `odom_frame_id`, `transform_tolerance` | Accepted and ignored with a warning (D11) |
| State topic | `state_topic` not empty | Refused (D13) |
| Zone visualisation | `<zone>.visualize: true`, `polygon_pub_topic` | Accepted and ignored with a warning (D13) |
| Collision point markers | always present upstream | Not provided (D13) |
| Real-time priority | `use_realtime_priority: true` | Refused |
| Lifecycle, toggle service, run-time `enabled` | — | Not provided (D2) |

## Deliberate differences in ROSlean

| # | Upstream | ROSlean | Why |
| --- | --- | --- | --- |
| D1 | Features in the table above | Refused or ignored as stated | Scope. |
| D2 | Lifecycle node, inactive until activated; `~/toggle`; `enabled` changeable at run time (R19) | Ordinary node, active from start-up; no toggle; parameters read once | Scope. The events of the model are scans and commands only. A launch file that drives upstream through a lifecycle manager must not list this node. |
| D3 | Points in `double` metres; zone tests in floating point, the polygon test with a division | Points and shapes in integer millimetres; tests exact, the division multiplied out | No floating point in the core. A point within a few millimetres of a boundary can be classified differently; the direction is fixed by D4. |
| D4 | Zone is the configured shape | Circle radius rounded up plus 1 mm; polygon with a band of √5 mm around its boundary | Makes the rounding of D3 err only towards "inside" ("Safe direction"). Zones grow by up to 3.5 mm (circle) or 4.4 mm (polygon). Upstream's uneven treatment of polygon boundary points (R10) disappears: all boundary points are inside. |
| D5 | Any `slowdown_ratio` accepted; negative ones reverse the command or do nothing, `-0.0` gives zero | A ratio with the sign bit set is refused at start-up | The reversal is a hazard, and "no amplification" is false with it. The model would treat such a ratio as never selected; refusing avoids a silent difference. Ratios of 1 or more, infinite or NaN are accepted and never take effect, as upstream. |
| D6 | Slowdown selection and the zero test use the computed floating-point velocity | Selection by exact order of the ratios; zero iff stop, or the command is zero, or the selected ratio is 0 | The core does not compute velocities. Three differences. (a) When a product or squared norm underflows or overflows (command components below about 10⁻¹⁶² or above 10¹⁵⁴ in magnitude, or a ratio below about 10⁻¹⁶² with an ordinary command), or two ratios are within rounding of each other: upstream then passes the command unscaled or picks the other zone, and a product that underflows to zero is not treated as zero for R15. (b) For a command that is zero in the three components, in a triggered slowdown zone: upstream takes no action (`0 < 0` is false), the model returns `slowdown k r`. The published bits are the same (zero), but the action, and so `out_kind` and any log line, differ. That `evaluate` does not depend on the command is therefore a difference from upstream, not one of its rules. |
| D7 | The clock is read at the start of `process` and again in `publishVelocity` | One reading per event | A decision is a function of (state, event). Differs by the run time of the callback, only for the boundary of `stop_pub_timeout`. |
| D8 | A scan stamp with negative seconds ends the process with an uncaught exception when the next command arrives | The stamp is taken as 0: under a non-zero timeout the scan is too old once the node clock is past the timeout (`0 + source_timeout < now`) | The node must not crash on input. The scan is **used** with timeout 0, and also while the node clock is still at or below the timeout (simulated time shortly after start: with `source_timeout: 2.0` and the clock at 1.5 s the scan counts as fresh). The shim's sum does not detect every stamp upstream refuses (`sec = -1` with `nanosec ≥ 10⁹` is positive); see REVIEWS, CM1/CM2 finding 9. |
| D9 | Negative timeouts accepted (a negative `source_timeout` makes nearly every scan too old; a negative `stop_pub_timeout` publishes zero once); non-finite or huge ones are undefined behaviour in the conversion | Negative, non-finite, and timeouts of 9.2·10⁹ s or more are refused | Keeps times in `Nat`; avoids undefined behaviour. |
| D10 | `max_points` overrides `min_points`; `min_points` read into a 32-bit `int` | `max_points` refused; `min_points` 64-bit | One rule for the threshold in the core. Differs otherwise only beyond ±2³¹. |
| D11 | Scan points transformed to `base_frame_id` by TF, by default with correction for the robot's motion since the stamp; a failed lookup stops the robot (R8) | No TF. Points are taken to be in the base frame, or moved by a fixed sensor pose from the ROSlean-only parameter `<source>.roslean_sensor_pose: [x, y, yaw]` (metres, radians; default `[0, 0, 0]`). `header.frame_id` is not read | Scope. With `base_shift_correction: false`, a zero pose and scans whose frame is the base frame, upstream performs no lookup either and the two agree. |
| D12 | Coordinates unbounded | Coordinates saturate at ±1000 m; zones must lie inside | Bounded integers at the boundary; see "Safe direction". |
| D13 | Publishes zone shapes, collision point markers and, if `state_topic` is set, the current action | None of these | Scope. They carry no decision. |
| D14 | A configuration error fails the lifecycle transition; the process stays up, unconfigured | Fatal log, exit code 1 | As twist_mux D7. |

Everything else is reproduced, including R1 (no output without commands),
R4 (an invalid scan is free space), R7 (age by header stamp), R8 with
timeout 0, R11 with `min_points <= 0`, R15 and R16.

## Model assumptions

- A1. Events are delivered one at a time, each with one node-clock reading
  (R19, D7).
- A2. Nothing is assumed about the clock being monotonic, nor about stamps
  being ordered. If the clock jumps backwards, a stored scan is not too old
  and zero keeps being published, as upstream.
- A3. Stopping the robot when the node publishes nothing is the base driver's
  job. The node is silent when no command arrives (R1), when a command is not
  finite (R2), and once zero has been published for `stop_pub_timeout` (R15).
  A base that keeps executing its last command needs a watchdog (PLAN, lesson
  16).
- A4. The scan is in the base frame up to the configured sensor pose, and the
  sensor's clock agrees with the node's well enough for R7 to be meaningful
  (D11).

## Open decisions

For the user. Each is implemented as upstream does it until decided.

1. **Fail-closed on invalid scans.** Upstream skips invalid readings and
   never rejects a scan (R4). A fail-closed rule would add a predicate
   `Scan.malformed` and make `sourcePoints` return `none` (stop, named
   `invalid source`) for a malformed stored scan, regardless of
   `source_timeout`. What it would change depends on what "invalid" means:
   - *Structural only*: empty `ranges`; `range_min` or `range_max` NaN, or
     `range_min > range_max`; non-finite `angle_min` or `angle_increment`;
     `ranges.size()` inconsistent with `(angle_max − angle_min) /
     angle_increment`; negative stamp. No effect on a healthy sensor. The
     length check needs a tolerance, since drivers differ by one.
   - *Any NaN reading*: REP-117 defines NaN as an erroneous reading, but many
     drivers and simulators emit NaN or 0 for "no return"; the robot would
     stop in open space with those.
   - *Any reading outside the limits*: `+inf` is the standard "nothing in
     range" value, so nearly every real scan would stop the robot. Not
     usable.
   - A threshold, e.g. fewer than a configured fraction of valid readings.

   In each case: a new numbered difference; Pass-through gets the extra
   hypothesis "the stored scan is well formed"; Bad data becomes "a malformed
   scan stops the robot"; the comparison with upstream must leave such scans
   out. The shim would pass `angle_min`, `angle_max`, `angle_increment` as bit
   patterns and the length, so that the test is in Lean.
2. **No output without commands.** Upstream has no timer (R1). A timer that
   publishes zero when the scan is too old or a stop zone is occupied, also
   when no command arrives, would be a deliberate difference and a new event
   kind. Same question as the twist_mux stretch goal "silence leads to zero".
3. **Zero is published only for `stop_pub_timeout`** (R15). As upstream, the
   Stop property ends in "zero or nothing". Publishing zero for as long as a
   stop is demanded would make it "zero"; it is a difference.
4. **Age by header stamp** (R7). A sensor clock ahead of the node's defeats
   the timeout. Measuring from arrival time instead, or taking the older of
   the two, is a difference; the scan event already carries the arrival time.
5. **`source_timeout: 0` and `<source>.enabled: false`** switch the
   protection off. Accept as upstream (now), or refuse at start-up.
6. **Pass-through forwards three components.** The task statement asks for
   the received message unchanged; upstream publishes a new message with
   `linear.z`, `angular.x`, `angular.y` set to 0 (R16), and the spec follows
   upstream. Forwarding all six is a difference visible only to robots that
   use those components.
7. **`base_shift_correction` must be set to `false` explicitly**, because
   upstream's default cannot be honoured (D11). The alternative is to ignore
   it with a warning. Likewise the name and form of
   `<source>.roslean_sensor_pose`; the alternative is one lookup of the static
   transform at start-up.
8. **Zone growth of D4.** Up to 3.5 mm and 4.4 mm. The alternative without
   growth is a model that can miss a point within about 2 mm inside a
   boundary.
9. **`min_points <= 0`** makes a zone trigger always, as upstream. Refusing
   it at start-up is the alternative.
10. **`state_topic` and `visualize`** are not provided (D13); the example
    configurations of Nav2 set both.

## Note on testing against upstream

For a comparison on the same messages: `base_shift_correction: false`,
scans with `frame_id` equal to `base_frame_id`, upstream activated through
its lifecycle, `state_topic` empty. Expect differences only where D3, D4 and
D6 say: points within 5 mm of a zone boundary, counts exactly at `min_points`
when such a point decides, and commands with components below about 10⁻¹⁶² or above 10¹⁵⁴. A
scan stamp with negative seconds ends upstream (D8). Times agree exactly,
unlike twist_mux D2: upstream compares integer nanoseconds here; only D7
remains, at the boundary of `stop_pub_timeout`.
