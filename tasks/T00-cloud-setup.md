# T00 — Cloud environment

Branch: `claude/t00-cloud-setup`. Needs: nothing. Read [README.md](README.md) first.

## Why

Every later task runs on a fresh Linux machine. Nobody has yet established
what works there. Until now the project was built on macOS arm64 and in
Docker; Linux through pixi was run once (`docs/PLAN.md`, "Not done").

## Do

1. Write `scripts/cloud_setup.sh`: installs elan and pixi if missing, puts
   them on `PATH`, runs `lake exe cache get`, and installs the pixi
   environment. It must be safe to run twice and must print what it did.
2. On the machine you are on, run and time: `(cd core && lake build)`,
   `lake exe cache get && lake build`, `pixi run build`, `pixi run test`.
   Record architecture, OS, cores, memory, and whether Docker and a display
   are available.
3. Try the Gazebo check: `pixi run cm-check`. If it fails for lack of
   rendering, try software rendering (`LIBGL_ALWAYS_SOFTWARE=1`, and what
   `docs/TUTORIAL_GAZEBO.md` says about Linux). Record exactly what works.
4. Run the upstream node once by hand
   (`review/collision_monitor/run_upstream_probe.sh` shows how) to confirm
   that `nav2_collision_monitor` from RoboStack runs here.
5. Update the "Environment" section of `tasks/README.md` with what you
   found: the commands that work, how long each takes, and what cannot run
   in the cloud. Later sessions rely on this section.

## May change

`scripts/cloud_setup.sh`, `tasks/README.md` (environment section only),
`tasks/reports/T00.md`. If a build or test fails on Linux, do not fix the
code: report the failure with its output. T01 and T02 will take it up.
