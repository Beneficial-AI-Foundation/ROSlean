# T07 — Nav2 extension: upstream scenarios

Branch: `claude/cm-nav2-harness`. Needs: T02 and T04 merged. Can run beside
T05 and T06. Read [README.md](README.md) first.

## Why

The harness (`ros/roslean_collision_monitor/test/`) pins down what upstream
does for fixed zones with the scan in the base frame. The extension needs the
same for lifecycle, transforms and footprint zones, written from upstream's
binary before the Lean node implements them.

## Do

Following the test plan in `docs/collision_monitor/SPEC_NAV2.md`, add fixed
scenarios whose expectations are upstream's observed outputs, and extend the
random differential test, for:

1. Transforms: a scan frame offset and rotated from the base; a transform
   that changes over time; a missing transform; a transform that arrives
   late; `transform_tolerance` boundaries to the nanosecond.
2. Base shift correction: the odom frame moving between scan and command,
   straight and turning; correction switched off.
3. Footprint zones with `stop` and `slowdown`: no message yet; a message
   before, between and after commands; a shape with too few points; a shape
   in another frame; replacement of the shape.
4. Lifecycle: commands and scans while unconfigured, inactive, active;
   deactivate and reactivate mid-stream; what is published on each
   transition.
5. `state_topic`: every message, including during the `stop_pub_timeout`
   silence.
6. Nav2's default `FootprintApproach` zone with `action_type` changed to
   `stop` and to `slowdown`.

The scenarios take the node under test as an argument, as the existing ones
do. Register them against upstream and, with the differences ND… that
SPEC_NAV2.md names, against the Lean node as expected failures until T06
lands (T08 turns them on). Add what you observe to `UPSTREAM_BEHAVIOUR.md`
as numbered findings; where the binary contradicts SPEC_NAV2.md, say so and
correct SPEC_NAV2.md.

## May change

`ros/roslean_collision_monitor/test/` (harness files, not `smoke_*`), the
marked harness block of the package's `CMakeLists.txt`, `pixi.toml` (tasks),
`UPSTREAM_BEHAVIOUR.md`, SPEC_NAV2.md (corrections).
