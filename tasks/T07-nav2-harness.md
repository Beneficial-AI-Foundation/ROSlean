# T07 — Nav2 extension: upstream scenarios

Branch: `claude/cm-nav2-harness`. Needs: T02 and T04 merged. Can run beside
T05 and T06. Read [README.md](README.md) first.

**Note (T05b).** The user has decided that the Lean node applies the
sensor-to-base transform in C++ in `double`, as upstream does (option A of
SPEC_NAV2.md, section 5), compiled with `-ffp-contract=off` on every
platform. Nothing in the scenarios below changes. For the expectations it
means: the Lean node's points are no longer exact images but a second
floating-point evaluation of upstream's formula; where upstream's build
fuses multiply-adds (its arm64 binary does) the two can differ in the last
bits of a coordinate, an estimated 10⁻⁸ mm. Keep every point of a scenario
and of the random test at least 5 mm from every zone boundary, as the
harness already does, also under rotated and general transforms; do not
write a scenario that expects the two nodes to agree on a point that lies
on a boundary. A transform with a non-finite number or a zero quaternion
cannot be published through tf2 (N25); if a scenario finds a way, the Lean
node treats it as a failed lookup (ND7).

## Why

The harness (`ros/roslean_collision_monitor/test/`) pins down what upstream
does for fixed zones with the scan in the base frame. The extension needs the
same for lifecycle, transforms and footprint zones, written from upstream's
binary before the Lean node implements them.

## Do

Following the test plan in `docs/collision_monitor/SPEC_NAV2.md` (section
10: 25 rows, each with what upstream did when T04 probed it), add fixed
scenarios whose expectations are upstream's observed outputs, and extend the
random differential test. `review/collision_monitor/nav2/nav2_probe.py`
contains each sequence of messages as a probe that prints; its output on
macOS is in `review/collision_monitor/nav2/results/`. Re-observe on the
binary; do not copy expectations from the document. The harness first needs
what the head of section 10 lists: a `/tf` publisher with chosen stamps,
static transforms with arbitrary quaternions, a `PolygonStamped` publisher,
lifecycle transitions inside a scenario, recording of the polygon topics.
The scenarios are for:

1. Transforms: a scan frame offset and rotated from the base, also tilted
   (upstream projects, N25); a transform that changes after the scan
   (without correction upstream uses the newest one, N23); a missing
   transform; a transform that arrives while the node waits;
   `transform_tolerance` boundaries to the nanosecond. The tolerance is a
   wait in node-clock time inside the callback (N26): under simulated time
   a command whose lookup fails is answered only after the clock is
   advanced, so these steps use `late`.
2. Base shift correction: the odom frame moving between scan and command,
   straight and turning; correction switched off. With correction every
   command needs an odom sample at or after its own clock reading (N24):
   publish one before each command unless the scenario is about its
   absence.
3. Zones fed by `polygon_sub_topic`, with `stop` and `slowdown` (this is
   the route for the footprint; `footprint_topic` is read for `approach`
   only, N29): no message yet; a message before, between and after
   commands; a shape with two points and with three; a shape in another
   frame, in a frame that moves, and in a frame without transform;
   replacement of the shape; a NaN coordinate (ND6: the Lean node differs).
   The subscription is best effort (N31): wait for delivery, and do not rely
   on a latched message.
4. Lifecycle: commands and scans while unconfigured, inactive, active;
   deactivate and reactivate mid-stream; what is published on each
   transition.
5. `state_topic`: every message, including during the `stop_pub_timeout`
   silence.
6. Nav2's default `FootprintApproach` zone with `action_type` changed to
   `stop` and to `slowdown` and nothing else: T04 observed that upstream
   **fails to configure** (N29), so the expectation is the refusal. Then the
   same zone with `polygon_sub_topic` added (SPEC_NAV2.md, 8.6), fed with a
   footprint in the odom frame, including the base moving away from it
   without a new message (N34).

The scenarios take the node under test as an argument, as the existing ones
do. Register them against upstream and, with the differences ND… that
SPEC_NAV2.md names (ND4 shutdown, ND5 names and ND6 NaN shapes are the ones
a scenario can see), against the Lean node as expected failures until T06
lands (T08 turns them on). The bond and the lifecycle manager (row 9 of the
test plan) run on the wall clock and are better kept as a separate small
test. Add what you observe to `UPSTREAM_BEHAVIOUR.md`
as numbered findings; where the binary contradicts SPEC_NAV2.md, say so and
correct SPEC_NAV2.md.

## May change

`ros/roslean_collision_monitor/test/` (harness files, not `smoke_*`), the
marked harness block of the package's `CMakeLists.txt`, `pixi.toml` (tasks),
`UPSTREAM_BEHAVIOUR.md`, SPEC_NAV2.md (corrections).
