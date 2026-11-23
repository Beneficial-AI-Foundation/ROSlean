# Task files for cloud sessions

Each `T??-*.md` file in this directory is a complete brief for one Claude Code
session. [PLAN.md](PLAN.md) lists the tasks, their order and their status.
This file holds the rules that apply to every task. A session reads this file,
then its task file, then the documents the task file names.

## Starting a task

In a terminal, in a checkout of this repository:

```sh
claude --cloud "Read tasks/README.md and tasks/T04-nav2-spec.md in this repository and carry out the task. Follow both files exactly."
```

or paste the same sentence into a new session at <https://claude.ai/code> with
this repository selected. Only what is pushed to GitHub exists for the
session: no local file, no other session's memory.

## What a session knows

Nothing but the repository. Sessions do not talk to each other and nobody
answers questions while a session runs. So:

- Take your starting point from `origin/main` unless the task names another
  branch.
- If the task depends on a task whose branch is not yet merged into `main`
  (check [PLAN.md](PLAN.md) and `git log origin/main`), stop and say so in
  your report. Do not rebuild the missing work yourself.
- When a decision is the user's and no default is given in the task or in
  [PLAN.md](PLAN.md) ("Decisions"), keep upstream's behaviour and list the
  question in your report.

## Environment

A cloud session is a fresh Linux machine. Run `scripts/cloud_setup.sh` first
if it exists (task T00 creates it). Otherwise:

```sh
curl -sSf https://raw.githubusercontent.com/leanprover/elan/master/elan-init.sh | sh -s -- -y
curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.elan/bin:$HOME/.pixi/bin:$PATH"
```

| Command | What it does |
| --- | --- |
| `(cd core && lake build)` | The executable Lean core. No Mathlib. |
| `lake exe cache get && lake build` | All theorems. Downloads the Mathlib cache first. |
| `pixi run build` | `colcon build` of the ROS packages (first use installs ROS 2 Jazzy, about 1 GB). |
| `pixi run test` | All colcon tests, about 20 minutes. |
| `pixi run cm-check` | Headless Gazebo check of the demo. Needs rendering; may not work in the cloud. |

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

1. Merge `origin/main` into your branch once more and re-run the builds.
2. Write `tasks/reports/T??.md` on your branch (format below).
3. Commit in the style of `git log`, and push to the branch the task names.
4. Do not merge into `main`. Open a pull request against `main` whose
   description is the report.

### Report format

```markdown
# T?? — title
Branch: claude/…            Base: <commit of origin/main you started from>
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
report names), merge into `main`, update the status column in
[PLAN.md](PLAN.md), push. Where two branches touched the package's
`CMakeLists.txt` or `package.xml`, the merge is the union of both.
