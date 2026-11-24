# Task files for cloud sessions

Each `T??-*.md` file in this directory is a complete brief for one Claude Code
session. [PLAN.md](PLAN.md) lists the tasks, their order and their status.
This file holds the rules that apply to every task. A session reads this file,
then its task file, then the documents the task file names.

## The integration branch

All work of this effort lives on the branch `collision-monitor`. It plays the
part that `main` usually plays: sessions start from it, pull requests target
it, and finished task branches are merged into it. **Nothing is merged or
pushed to `main`.** Merging `collision-monitor` into `main` at the end is the
user's decision alone.

## Starting a task

In a terminal, in a checkout of this repository:

```sh
git fetch origin && git checkout collision-monitor && git pull --ff-only
claude --cloud "Check out the branch collision-monitor of this repository. Read tasks/README.md and tasks/T04-nav2-spec.md there and carry out the task. Follow both files exactly."
```

or paste the same sentence into a new session at <https://claude.ai/code> with
this repository and the branch `collision-monitor` selected. Only what is pushed to GitHub exists for the
session: no local file, no other session's memory.

## What a session knows

Nothing but the repository. Sessions do not talk to each other and nobody
answers questions while a session runs. So:

- Take your starting point from `origin/collision-monitor` unless the task names another
  branch.
- If the task depends on a task whose branch is not yet merged into `collision-monitor`
  (check [PLAN.md](PLAN.md) and `git log origin/collision-monitor`), stop and say so in
  your report. Do not rebuild the missing work yourself.
- When a decision is the user's and no default is given in the task or in
  [PLAN.md](PLAN.md) ("Decisions"), keep upstream's behaviour and list the
  question in your report.

## Environment

A cloud session is a fresh Linux machine. Run this first, from the checkout
(about 4 minutes; a second run takes a few seconds and changes nothing):

```sh
scripts/cloud_setup.sh
export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"   # in every new shell
```

It installs elan, the Lean toolchain, pixi, the Mathlib cache and the pixi
environment, and prints what it did. Everything below was measured by task
T00 ([reports/T00.md](reports/T00.md)) on one cloud session: x86-64, Ubuntu
24.04, 4 cores, 15 GB of memory, user `root`.

| Command | What it does | Time in the cloud |
| --- | --- | --- |
| `(cd core && lake build)` | The executable Lean core. No Mathlib. | 2 s |
| `lake exe cache get && lake build` | All theorems. The setup script has already fetched the cache (1.5 min). | 30 s |
| `pixi run build` | `colcon build` of the ROS packages. | 1 min |
| `pixi run test` | All colcon tests (11 tests in 4 packages). | 20 min |
| `review/collision_monitor/run_upstream_probe.sh $PWD/.pixi/envs/default` | Runs upstream's `nav2_collision_monitor` binary. | 1 min |
| `pixi run cm-check-cloud` | Headless Gazebo check of the demo. | 1 to 3 min; see below |

### What the network allows

Outbound traffic goes through a proxy with a list of allowed hosts, which the
user sets in the cloud environment's settings. A blocked host answers
`CONNECT tunnel failed, response 403`. Do not try to get round it; report it.

Allowed when T00 finished: `github.com` and `raw.githubusercontent.com` (git
and release downloads), `conda.anaconda.org`, `pypi.org`,
`archive.ubuntu.com`, and, added by the user during T00,
`releases.lean-lang.org`, `pixi.sh`, `cache.mathlib.org` and
`fuel.gazebosim.org`.

If the environment is ever set up anew without those four:

- `scripts/cloud_setup.sh` takes the Lean toolchain and pixi from github.com
  instead, and probes `cache.mathlib.org` and skips the cache if it is
  blocked. `lake build` then compiles Mathlib from source: 35 minutes on all
  four cores, once per session.
- **Do not run `lake exe cache get` by hand in that case.** It does not
  fail: it tries 9000 files one after the other, each failing after some
  seconds, and would run for hours.
- The Gazebo world cannot download its furniture; add `--no-furniture`
  (below).

### Gazebo

There is no display and no graphics card; `Xvfb` and Mesa's software
renderer are installed on the machine (`/usr/bin/xvfb-run`). In the cloud use

```sh
pixi run cm-check-cloud                                     # 1 to 3 minutes
pixi run cm-check-cloud --monitor nav2_collision_monitor    # upstream's node
```

It is `cm-check` under a virtual display with software rendering
(`LIBGL_ALWAYS_SOFTWARE=1`). Both commands pass here; the first run downloads
the furniture (95 MB, into `~/.gz/fuel`).

- `pixi run cm-check` without the virtual display: `gz sim` segfaults in
  `Ogre2RenderEngine`, and the check gives up only after six minutes.
- `--no-furniture` (launch argument `furniture:=false`) loads a copy of the
  world without the fifteen models from `fuel.gazebosim.org`, for a machine
  that cannot reach that host. The walls, the floor and the robot remain,
  which is all the check's scenario uses. It passes here too. A stopgap.
- If Gazebo exits early (world not loaded, no display), the check does not
  notice and waits for minutes. Look at `--log FILE`.
- Windows (`pixi run cm-gazebo`, RViz) were not tried.

### Other

- **Docker:** the client and `dockerd` are installed but no daemon runs.
  Started by hand (`dockerd &`) it comes up. Building or running a container
  was not tried; processes in a container do not reach the proxy without
  extra settings (`--network host`, the CA bundle).
- **Disk:** the Lean toolchain 3.5 GB, `.lake` 8 GB with the Mathlib cache,
  `.pixi` 5.9 GB; 45 GB were free afterwards.
- **Tools missing on the machine:** `zstd`.

If something cannot be installed or run where you are (no network to a host,
no Docker, no display), say so in the report. Never report a result you did
not obtain.

## Rules for the work

- **Lean:** no `sorry`, no `native_decide`, no new axioms. The core package
  (`core/`) never imports Mathlib.
- **Decisions in Lean.** The C++ shim moves data. Anything safety-relevant
  that must stay in C++ is listed as trusted in the SPEC, sits in one small
  file, and has its own tests.
- **Upstream is read from source and checked on the binary.** A statement
  about `nav2_collision_monitor` says which of the two it rests on.
- **A defect is not hidden.** If a property is false, report the
  counterexample; do not weaken a statement silently or bend the model.
- **Stay inside the task's files.** Each task lists what it may change.
  Other sessions change the rest at the same time.
- **README.md** is written by task T11 only.

## Finishing a task

1. Merge `origin/collision-monitor` into your branch once more and re-run the builds.
2. Write `tasks/reports/T??.md` on your branch (format below).
3. Commit in the style of `git log`, and push to the branch the task names.
4. Do not merge into `collision-monitor` yourself and never touch `main`.
   Open a pull request with base branch `collision-monitor` (not `main`) whose
   description is the report.

### Report format

```markdown
# T?? — title
Branch: claude/…            Base: <commit of origin/collision-monitor you started from>
## Done
## Run, with results          (command, platform, counts)
## Not run, and why
## Found wrong or changed     (in the SPEC, model, node, tests; with reproduction)
## Questions for the user
## Friction                   (what cost time; what in Lean or the tooling was awkward)
```

## Merging (for whoever orchestrates)

For each finished branch: fetch it, read the report, rebuild
(`lake build` in `core/` and in the root, `pixi run build`, the tests the
report names), merge into `collision-monitor`, update the status column in
[PLAN.md](PLAN.md), push. Where two branches touched the package's
`CMakeLists.txt` or `package.xml`, the merge is the union of both.
