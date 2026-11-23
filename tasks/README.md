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
(2 minutes without the pixi environment, 4 minutes with it; a second run
takes half a minute and changes nothing):

```sh
scripts/cloud_setup.sh
export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"   # in every new shell
```

It installs elan, the Lean toolchain, pixi and the pixi environment, and
prints what it did. Everything below was measured by task T00
([reports/T00.md](reports/T00.md)) on one cloud session: x86-64, Ubuntu
24.04, 4 cores, 15 GB of memory, 45 GB of free disk afterwards, user `root`.

| Command | What it does | Time in the cloud |
| --- | --- | --- |
| `(cd core && lake build)` | The executable Lean core. No Mathlib. | 2 s |
| `lake build` | All theorems. **Compiles Mathlib from source**, see below. | 35 min the first time, 6 s afterwards |
| `pixi run build` | `colcon build` of the ROS packages. | 1 min |
| `pixi run test` | All colcon tests (11 tests in 4 packages). | 20 min |
| `review/collision_monitor/run_upstream_probe.sh $PWD/.pixi/envs/default` | Runs upstream's `nav2_collision_monitor` binary. | 1 min |
| `pixi run cm-check` | Headless Gazebo check of the demo. | Fails as it is; see below. |

Start `lake build` early and in the background if the task needs the
theorems: it uses all four cores for 35 minutes, and timing-sensitive tests
(`pixi run test`, the probes) should not run beside it.

### What the network allows

Outbound traffic goes through a proxy with a list of allowed hosts. A blocked
host answers `CONNECT tunnel failed, response 403`. Do not try to get round
it; use what is allowed or report it.

| Host | Used for | From the cloud |
| --- | --- | --- |
| `github.com`, `raw.githubusercontent.com` (git, release downloads) | elan, Lean releases, pixi releases, Lean dependencies | allowed |
| `conda.anaconda.org` | the pixi environment (RoboStack, conda-forge) | allowed |
| `pypi.org`, `archive.ubuntu.com` | | allowed |
| `releases.lean-lang.org` | where `elan` fetches toolchains | **blocked**; the script takes the same release from github.com |
| `pixi.sh` | pixi's installer | **blocked**; the script takes the release from github.com |
| `cache.mathlib.org` | `lake exe cache get` | **blocked**; no Mathlib cache |
| `fuel.gazebosim.org` | the furniture of `apartment.sdf` | **blocked**; the world does not load |

- **Never run `lake exe cache get` here.** It does not fail: it tries about
  9000 files one after the other, each failing after some seconds, and would
  run for hours. `scripts/cloud_setup.sh` probes the host and skips the step.
  `pixi run prove` contains the same command; use `lake build` instead.
- Without the cache `lake build` compiles the part of Mathlib that
  `Mathlib.Tactic` needs (about 6600 jobs, `.lake` grows to 4.5 GB). It is
  done once per session; nothing of it survives the session.

### Gazebo

There is no display and no graphics card; `Xvfb` and Mesa's software
renderer are installed on the machine (`/usr/bin/xvfb-run`).

- `pixi run cm-check` as it is: Gazebo cannot download the furniture, exits,
  and the check does not notice: it was still waiting when it was killed
  after five minutes.
- With the furniture taken out of the world but no display: `gz sim`
  segfaults in `Ogre2RenderEngine`; the check gives up after six minutes.
- With the furniture taken out **and** a virtual display and software
  rendering, the check passes in one minute, with this repository's monitor
  and with upstream's:

  ```sh
  LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x1024x24" pixi run cm-check
  LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a -s "-screen 0 1280x1024x24" pixi run cm-check --monitor nav2_collision_monitor
  ```

  "Taken out" means: the fifteen `<include>` blocks with a
  `fuel.gazebosim.org` URI deleted from
  `ros/roslean_twist_mux/demo/gazebo/apartment.sdf`. T00 did that in the
  working tree for the run only and did not commit it. The walls, the floor
  and the robot remain, which is all the check's scenario uses. A task that
  needs the check in the cloud has to do the same (and not commit it), or
  be given a world file that needs no download.
- Windows (`pixi run cm-gazebo`, RViz) were not tried.

### Other

- **Docker:** the client and `dockerd` are installed but no daemon runs.
  Started by hand (`dockerd &`) it comes up. Building or running a container
  was not tried; processes in a container do not reach the proxy without
  extra settings (`--network host`, the CA bundle).
- **Disk:** the Lean toolchain 3.5 GB, `.lake` 4.5 GB, `.pixi` 5.9 GB.
- **Tools missing on the machine:** `zstd` (so the `.tar.zst` Lean release
  cannot be unpacked; the script uses the `.zip`).

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
