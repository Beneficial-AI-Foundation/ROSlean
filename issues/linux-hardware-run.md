# Collision monitor: run the tests and benchmarks on Linux hardware and on x86-64

Does not block the write-up (`tasks/PLAN.md`, "Decisions", 2026-11-25).

## Why

Everything so far ran on one Apple M5 Pro: natively on macOS arm64, and in arm64 Linux containers on that Mac (a virtual machine). Not covered:

- x86-64, in any form. The node is compiled with `-ffp-contract=off` so that the transform gives the same bits everywhere; this has only been checked on arm64.
- Linux on its own hardware: its scheduler, its middleware timing, a real robot computer's speed.
- The final node's timing on Linux. The container figures in the documents are from an earlier version of the command path.

The documents say so (`README.md`, "Cost and platforms"; `site/gaps.html`, "Platforms never tested").

## Machines

At least one, better both:

1. an x86-64 machine with Ubuntu 24.04;
2. a robot computer or a board of that class (arm64 or x86-64), with Ubuntu 24.04.

Record for each: CPU model, cores, RAM, kernel, whether other load was running.

## Set-up

Two ways; do the first, and the second if time allows.

**pixi** (ROS from RoboStack, upstream 1.3.13):

```sh
sudo apt-get update && sudo apt-get install -y curl ca-certificates git
curl -fsSL https://pixi.sh/install.sh | sh
curl -sSf https://raw.githubusercontent.com/leanprover/elan/master/elan-init.sh | sh -s -- -y
git clone git@github.com:Beneficial-AI-Foundation/ROSlean.git && cd ROSlean
git checkout collision-monitor        # or main, once merged
pixi run build
```

The first build fetches the Mathlib cache and takes tens of minutes. `scripts/cloud_setup.sh` does the same on a machine without a graphics card.

**Docker** (ROS and Nav2 from apt, upstream 1.3.14): `docker build -f docker/Dockerfile -t roslean .` builds and runs the tests; `pixi run cm-linux-image` and `bench/cm_linux/run.sh OUT_DIR versions scenarios selftest random points faults qos bench soak ctest` run the comparison.

## What to run

| Step | Command | Time | Expected |
| --- | --- | --- | --- |
| 1. Tests | `pixi run test` | about 20 min on the Mac | 28 tests, 0 failures (29 on macOS) |
| 2. Scenarios beside upstream | part of step 1 (`roslean_scenario`, `roslean_nav2_scenario`) | | 0 failures; every difference a numbered one |
| 3. Random comparison | `pixi run cm-diff 0 300` | about 1.5 h in one process | 0 cases outside the bounds; compare the 300 log lines with `bench/results/cm_comparison/t03/random_?.log` |
| 4. Random cases of the Nav2 extension | `pixi run cm-nav2-random` | minutes | 0 cases disagree with the model |
| 5. Fault injection | `pixi run cm-faults` | minutes | 0 failures |
| 6. Cost | `pixi run bench-cm` | about 40 min, includes a 30-minute soak | the tables of COMPARISON.md, "Cost of a command", for this machine |
| 7. Demo check | `pixi run cm-check` and `pixi run cm-check --nav2` with a display; `pixi run cm-check-cloud` without (needs `xvfb`) | minutes | passes; `cm-check` without a display hangs silently (known, `docs/AUDIT.md`) |

Step 3 on x86-64 is the important one: it is the test that the two nodes decide alike when the floating-point transform runs on another architecture. A difference in the log lines against the arm64 logs is a finding even if no case is outside the bounds.

## What to report

- For steps 1 to 5 and 7: the summary lines, and the full log of anything that fails.
- For step 6: p50 and p99 per row for both nodes, the ratio, and the latency through the nodes; whether every command was answered at 10,000 readings.
- Whether `roslean_cmd_cost` meets the budget (50 / 150 / 1500 µs at 360 / 1080 / 10,000 readings). On a slow board it may not; that is a result, not a defect of the run.
- Anything the README's instructions did not cover.

Put logs under `bench/results/cm_comparison/<machine>/` and the figures into COMPARISON.md, `README.md` ("Cost and platforms") and `site/gaps.html` ("Platforms never tested", "Cost").

## Optional, on the robot computer

- A paced run over hours (commands at the robot's real rate) recording the longest command, as a first figure for the worst case. No such figure exists.
- The node in place of `nav2_collision_monitor` on a real robot's bring-up, with a real lidar. Nothing in this repository has run on hardware with a sensor.

## Known obstacles

- `pixi run build` on Linux needed two repairs (strip tool, pthread), fixed in `f9fcdeb` and `da8fd3d` and checked in an arm64 container only.
- The full suite was not repeated on Linux after those repairs; step 1 does that.
- Upstream's version differs by set-up: 1.3.12 (pixi, macOS), 1.3.13 (pixi, Linux), 1.3.14 (apt).
