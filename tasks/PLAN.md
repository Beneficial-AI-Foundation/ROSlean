# Collision monitor: plan and task list

Status on 2026-11-23. Goal: a Lean replacement for the core of Nav2's
`nav2_collision_monitor` that Nav2 can launch and manage in place of its own
monitor, for `stop` and `slowdown` zones, with proved decision logic, tested
against the upstream binary, and a precise public statement of what is still
missing. Rules for sessions: [README.md](README.md).

All of this lives on the integration branch `collision-monitor`; `main` holds
none of it until the user merges the branch at the end.

## Where things stand

On `collision-monitor` (each merged after a rebuild on macOS arm64):

| Part | Where | State |
| --- | --- | --- |
| Specification | `docs/collision_monitor/SPEC.md` | R1–R19, D1–D14, read from source |
| Lean model | `core/ROSleanCore/CollisionMonitor.lean` | done |
| Proofs | `ROSlean/CollisionMonitor/` | every property of the plan's table, iff characterisations, trace theorems; no `sorry` |
| C interface and node | `core/ROSleanCore/CollisionMonitor{FFI,Convert}.lean`, `ros/roslean_core`, `ros/roslean_collision_monitor` | done; conversions in Lean |
| Upstream harness | `ros/roslean_collision_monitor/test/` | 55 scenarios and a random differential test, run on upstream only |
| Reviews | `docs/REVIEWS.md` | CM1/CM2 answered; CM3/CM4 has eight open findings |
| Gazebo demo | `ros/roslean_collision_monitor/demo/gazebo/` | headless check passes; never watched on a screen |

Not on `collision-monitor`:

| Branch | Contents | State |
| --- | --- | --- |
| `claude/cm-node-fixes` | Fixes for the eight CM3/CM4 findings, including `Event.scanUnusable` in the model | One commit on top of `collision-monitor`. The session was stopped before it reported. **Unverified.** |
| `claude/cm-comparison-wip` | Lean node through the harness, random comparison, fault injection, benchmarks, `COMPARISON.md` | Stopped before finishing. Uncommitted work was saved as one WIP commit on top of `462088e`. Placeholders (`{{…}}`) remain in the document. **Unverified.** It reports four defects. |

Nothing has been run on Linux x86-64 except the node's own tests under
emulation. The comparison with upstream has not been run on Linux at all.

## Tasks

Run a task when everything in "Needs" is merged into `collision-monitor`. Tasks in the
same row group can run at the same time.

| Task | What | Needs | Runs in cloud | Status |
| --- | --- | --- | --- | --- |
| [T00](T00-cloud-setup.md) | Cloud environment: set-up script, what runs there | — | yes | merged 586f733 |
| [T01](T01-finish-node-fixes.md) | Verify and finish `claude/cm-node-fixes` | — | yes | running |
| [T04](T04-nav2-spec.md) | Specification and design of the Nav2 extension (new file only) | — | yes | running |
| [T02](T02-finish-comparison.md) | Finish the comparison with upstream from the WIP branch; run it on Linux | T00, T01 | yes | open |
| [T03](T03-comparison-defects.md) | Fix the defects the comparison found (negative clock, slow polygon path) | T02 | yes | open |
| [T05](T05-nav2-model-proofs.md) | Extension: model events and proofs | T01, T04 | yes | open |
| [T07](T07-nav2-harness.md) | Extension: upstream scenarios for lifecycle, transforms, footprint | T02, T04 | yes | open |
| [T06](T06-nav2-node.md) | Extension: C interface and node (lifecycle, state topic, TF, footprint) | T03, T05 | yes | open |
| [T08](T08-nav2-review.md) | Independent review of the extension; Lean node through T07's scenarios | T06, T07 | yes | open |
| [T09](T09-nav2-fixes.md) | Fixes from T08 | T08 | yes | open |
| [T10](T10-nav2-demo.md) | Demo: Nav2 launches the Lean node in place of its monitor | T09 | no (needs a display) | open |
| [T11](T11-writeup-website.md) | Website page on what is missing and what it takes; README; trusted base | T09 (T10 if done) | yes | open |
| [T12](T12-fresh-machine-audit.md) | Someone with nothing but the README builds and runs it | T11 | yes | open |

```
T00 ─┐
T01 ─┼─ T02 ── T03 ─┐
     │      └─ T07 ─┼─────┐
T04 ─┴─ T05 ────────┴ T06 ┴ T08 ── T09 ─┬─ T11 ── T12
                                        └─ T10 (local)
```

First wave: T00, T01 and T04 together.

### File ownership

So that parallel tasks do not collide.

| Files | Tasks that may change them |
| --- | --- |
| `core/ROSleanCore/CollisionMonitor*.lean`, `ROSlean/CollisionMonitor/` | T01, then T03, T05, T06 (FFI files only), T09 |
| `ros/roslean_core`, `ros/roslean_collision_monitor/{src,include,config}`, `smoke_*` tests | T01, then T03, T06, T09 |
| `ros/roslean_collision_monitor/test/` (harness), `bench/` | T02, then T07, T08 |
| `docs/collision_monitor/SPEC.md` | T01, T03, T05, T06, T09 |
| `docs/collision_monitor/SPEC_NAV2.md` | T04; later tasks correct it |
| `docs/collision_monitor/COMPARISON.md` | T02, T08 |
| `site/`, `README.md`, `docs/PLAN.md` | T11 |
| `scripts/cloud_setup.sh`, `tasks/README.md` (environment section) | T00 |

## Decisions

Taken:

- ROS 2 Jazzy. Integers in the Lean core. Match upstream; every difference is
  numbered in the SPEC.
- Scope is extended by pieces 1 to 3 (lifecycle interface and state topic;
  transforms from TF with base shift correction; the footprint as a zone).
  `approach`, `limit`, velocity and dynamic polygons other than the footprint,
  and other sensor types stay out.
- No stretch work (braking model, velocity smoother) before the above is
  finished.
- With pieces 1 to 3: the sensor pose comes from TF, `state_topic` is
  published, the node is a lifecycle node.

Open, with the default that tasks follow until the user decides:

| Question | Default |
| --- | --- |
| Stop on structurally invalid scans (fail-closed)? | As upstream: readings are filtered one by one |
| Publish zero for as long as a stop is demanded, not only for `stop_pub_timeout`? This removes the never-published stop (THEOREMS.md, "What the robot receives"). | As upstream |
| A timer that publishes zero when commands cease? | As upstream: none |
| Scan age by arrival time instead of header stamp? | As upstream: header stamp |
| Publish zero for a non-finite command? | As upstream: ignored |
| Refuse zones that protect nothing (no zones, radius 0, `source_timeout: 0`)? | Accept and warn |
| Apply the transform in C++ as upstream (bit-for-bit, larger trusted part) or in Lean in exact arithmetic (smaller trusted part)? | T04 recommends; until decided, the option T04 recommends |
| Hysteresis against stop/slowdown flicker at a zone edge? | No |
| Keep the extra 1 mm on circle radii? | Keep |

## After each merge

Update the status column above (`open`, `running`, `branch pushed`,
`merged <commit>`), and move anything a report raised for the user into
"Decisions".
