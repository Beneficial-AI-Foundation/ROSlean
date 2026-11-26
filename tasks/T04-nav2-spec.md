# T04 — Specification and design of the Nav2 extension

Branch: `claude/cm-nav2-spec`. Needs: nothing (it only adds a file).
Read [README.md](README.md) first.

## Why

Today the node is a gate for fixed `stop` and `slowdown` zones with the scan
in the base frame or at a hand-configured pose. The user has decided to extend
it so that Nav2 can launch and manage it in place of its own monitor, still
without the `approach` and `limit` actions. This task writes the
specification and the design. It writes no node code, no model code and no
proofs.

## Read

`docs/collision_monitor/SPEC.md` (all of it), `UPSTREAM_BEHAVIOUR.md`,
`THEOREMS.md`, both collision monitor sections of `docs/REVIEWS.md`,
`core/ROSleanCore/CollisionMonitor{,FFI,Convert}.lean`, the node source, and
the drop-in findings in `docs/TUTORIAL_COLLISION_MONITOR.md`. The upstream
source is `nav2_collision_monitor` on the `jazzy` branch of
ros-navigation/navigation2, at the version SPEC.md names; clone it outside
the repository tree. The upstream binary is in the pixi environment
(`review/collision_monitor/run_upstream_probe.sh` and `side_by_side.py` show
how to run it; it needs `bond_heartbeat_period: 0.0` without a lifecycle
manager).

## The three pieces

1. **Node plumbing.** A lifecycle interface that Nav2's lifecycle manager
   drives exactly as it drives upstream (transitions, what is created and
   destroyed in each, bond, what happens to commands and scans while
   inactive). `state_topic`: message type, when a message is published, its
   contents, also during the `stop_pub_timeout` silence. `visualize` and the
   polygon publishers. Which launch files and lifecycle lists of
   `nav2_bringup` name the node, and exactly what a user changes to run ours.
2. **Transforms.** How upstream gets each scan into the base frame: which TF
   lookups, at which times, with which tolerance and timeout; what happens
   when a lookup fails or is late (UPSTREAM_BEHAVIOUR.md describes a delayed
   stop); `base_shift_correction` through the odom frame: the exact
   computation, its precision, its order of operations, and how the 3D
   transform is reduced to the plane.
3. **The footprint as a zone.** `footprint_topic` and `polygon_sub_topic` for
   `stop` and `slowdown` zones: message types, QoS, frames and transforms of
   the received shape, the zone before the first message, malformed and
   too-small polygons, whether a new message replaces the shape atomically.
   Check on the binary whether Nav2's default `FootprintApproach` zone works
   with `action_type` changed to `stop` or `slowdown` and nothing else
   changed: that is the edit users will be told to make.

## Deliver: `docs/collision_monitor/SPEC_NAV2.md`

Number rules N1, N2, … and differences ND1, … . Probe scripts go under
`review/collision_monitor/nav2/`.

- **Upstream's rules**, each with file and function, each marked as read from
  source, observed on the binary, or both.
- **New model events** (for example: no transform for the stored scan, zone
  shape replaced, node activated or deactivated) and what `step` does for
  each, so that every failure case stops the robot and can be proved. Write
  each as a precise statement in the manner of SPEC.md's "Properties",
  including the if-and-only-if characterisations.
- **Where the transform is applied.** Option A: the shim applies it in
  floating point exactly as upstream does and hands Lean points as now
  (largest trusted part, closest agreement). Option B: the shim passes the
  transform's numbers as bit patterns and Lean applies it in exact arithmetic
  (trusted: only that tf2 returned the right numbers; costs time per point;
  not bit-for-bit). Estimate B's cost per scan (the current decode and
  convert costs 41 ns per reading on an M-series Mac), bound how far B's
  points can be from upstream's, say whether the polygon margin of √5 mm
  still guarantees "a point upstream counts is always counted", and
  recommend one. The same for base shift correction.
- **The trusted list after the change**, replacing T1–T5, and what the
  theorems will and will not say.
- **The shim boundary** for each new call, in the form of SPEC.md's section.
- **Edits to make to SPEC.md later** (the fate of `roslean_sensor_pose`, D2,
  D11, D13, the frame check): a list, not made now.
- **A test plan**: the scenarios the harness needs (moving transforms, a
  missing and a late transform, footprint messages before, during and after
  commands, lifecycle transitions mid-stream, state messages) and what
  upstream does in each, observed by you.
- **An implementation plan** for T05 (model and proofs), T06 (C interface and
  node) and T07 (harness), with what each needs from the others. Correct the
  task files T05 to T07 where your findings make them wrong.
- **"What still separates this from Nav2's default configuration"**: for
  `approach`, `limit`, velocity polygons, dynamic polygons, point cloud,
  range, polygon and costmap sources, several sources, and anything else
  upstream has that our node will still refuse or ignore: what it is, what
  `nav2_bringup`'s `nav2_params.yaml` and the package's example file use of
  it, and what closing it would take (work items, what would be proved, what
  would be trusted, rough size relative to the work done so far). Every
  statement checked against the source, with the reference. T11 builds the
  website page from this section.

## May change

`docs/collision_monitor/SPEC_NAV2.md`, `review/collision_monitor/nav2/`,
`tasks/T05-*.md` to `T07-*.md` (corrections). Nothing else.
