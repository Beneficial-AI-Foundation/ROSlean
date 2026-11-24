# Fresh-machine audit (task T12)

Two questions: can someone with nothing but `README.md` build and run this
repository, and is every number and claim in `README.md` and on the pages
in `site/` true of the repository? Made on 2026-11-24 on the branch
`collision-monitor` at commit `6dbc5a0`, by a session that wrote none of
the code or the text.

"Observed" means seen in a run made for this audit. "Read" means compared
with a file of the repository and not run. Where a claim was checked
against a document and not against a run, the document is named.

## Summary

- **macOS arm64: the README is enough**, once `git`, `curl`, elan and pixi
  are there. `pixi run build`, `pixi run prove`, `pixi run test-quick` and
  `pixi run test` all ended with exit code 0; the mux and the collision
  monitor start as the README says. The demos were started; nobody saw a
  window (the screen was locked).
- **The Docker image builds as the README says**, with its tests: 28
  tests, 0 failures, beside apt's upstream 1.3.14.
- **Linux arm64 through pixi: `pixi run build` fails as the tree is**, in
  two places, on a plain `ubuntu:24.04`: no `strip` is found, and
  `test_cm_ffi` does not link without `-pthread`. With two workarounds
  that change no file of the repository, everything that needs no display
  passed. Findings 1 and 2. **After the audit** (added by the orchestrator,
  not part of the audit's own runs): Both were repaired after the audit (commits `f9fcdeb`, `da8fd3d`: `strip` is found from the compiler's name, `test_cm_ffi` links `Threads`). On the repaired tree, in the same kind of container, `pixi run build` succeeds as written, the library is stripped, and five tests were run and pass (`test_mux_ffi`, `test_cm_ffi`, `smoke_conversions`, `smoke_transforms`, `smoke_node`); the full suite was not run again there.
- **The theorems are as stated.** All 73 theorem names in the README and on
  the pages exist, their statements say what the text says (three small
  omissions of a hypothesis, finding 9), and every one of the 1,084
  theorems of the project depends on `propext`, `Classical.choice` and
  `Quot.sound` only. No `sorry`, `native_decide`, `axiom`, `unsafe`,
  `implemented_by` or `@[extern]` in any Lean file.
- **The refusals are as stated, with one exception.** 27 configurations
  were tried, 23 on the built node and 4 on upstream's for comparison. For Nav2's unchanged `nav2_params.yaml` the
  node prints `2 parameters refused`, where the README and `gaps.html`
  quoted `1 parameter refused`. Corrected.
- **The whole test suite passed on both**, in one run each: macOS
  `29 tests, 0 errors, 0 failures` in 39 minutes; the Linux container
  `28 tests, 0 errors, 0 failures` in 39 minutes, beside upstream 1.3.13,
  a version the pages do not name (finding 11).
- **The numbers** agree with the documents they come from, with the
  exceptions listed under "Discrepancies corrected". The headline cost
  table rests on a task report: its raw data is not in `bench/results/`.
  Re-measured here, it holds within 7 % (finding 5).
- **The pages now say what the Nav2 demo found** (task T10): with a stop
  zone in place of Nav2's `approach` zone, Nav2's tour ended with the robot
  standing for good in three runs of six, with both nodes.

## What "a fresh machine" was

There was no fresh machine. Two approximations, both on one Apple M5 Pro
(18 cores, macOS 26.7.2):

- **(A) macOS arm64.** A fresh `git clone` of `collision-monitor` over
  https into a new directory outside the repository, and every command run
  as `env -i HOME=<new empty directory> PATH=/usr/bin:/bin:/usr/sbin:/sbin
  TERM=… <command>`, with the two installers' directories added to `PATH`
  after installing. No installed elan, pixi, cache or shell configuration
  was visible. What this does not remove: the Xcode command line tools
  (`/Library/Developer/CommandLineTools`, which provide `git` and the
  macOS SDK), anything else installed system-wide, and the machine's fast
  network. The clone itself was made with the session's own environment
  (the session's tooling does not let `git` run through the `env -i`
  wrapper); it needed no credentials.
- **(B) Linux arm64.** A `ubuntu:24.04` container in Docker Desktop 29.9.3
  on the same Mac (a Linux virtual machine with 18 cores and 7.7 GB of
  memory), as root, starting from the bare image. Not Linux on its own
  hardware, not x86-64, no display, no graphics card.

Times are wall-clock on this machine and this network (the 4.5 GB
environment arrived in under two minutes); they are lower bounds for
anyone else.

## (A) macOS arm64: the README, step by step

| Step | Result | Time | Disk afterwards |
| --- | --- | --- | --- |
| Get the repository | The README gave no command. `git clone --branch collision-monitor https://github.com/Beneficial-AI-Foundation/ROSlean.git` worked without credentials | 3 s | 80 MB |
| `pixi run build`, as the first command | `env: pixi: No such file or directory`. The README says "Requires elan and pixi, both on `PATH`" and links to both, with no command | | |
| Install elan (`curl https://elan.lean-lang.org/elan-init.sh -sSf \| sh -s -- -y`, from the linked page) | ok | 1 s | |
| Install pixi (`curl -fsSL https://pixi.sh/install.sh \| sh`, from the linked page) | ok. Both installers change the shell's start-up files only: a new terminal is needed before `pixi` and `lake` are found | 2 s | 66 MB in the home directory |
| `pixi run build` | `Summary: 4 packages finished [1min 37s]`, exit 0. Installs the environment (4.5 GB in `.pixi/`, 4.7 GB more in pixi's cache under the home directory) and the Lean toolchain 4.37.0-rc1 (3.3 GB in `~/.elan`). pixi warns that it skipped three post-link scripts (gdk-pixbuf, graphviz, gtk3); no effect seen | 3 min 5 s | repository 4.6 GB, home 8.0 GB |
| `pixi run prove` | `Build completed successfully (6659 jobs)`, exit 0. Mathlib's cache was downloaded (7.8 GB in `.lake/`). 3,946 warning lines, all of them the same notice about Mathlib and the module system | 1 min 25 s | repository 12 GB, home 8.5 GB |
| `pixi run test-quick` | `Summary: 17 tests, 0 errors, 0 failures, 0 skipped`, exit 0 | 7 min 6 s | |
| `pixi run test` | `Summary: 29 tests, 0 errors, 0 failures, 0 skipped`, exit 0 (below) | 39 min 11 s | |
| Run the mux (`pixi run bash -c 'source install/setup.bash && ros2 run roslean_twist_mux twist_mux …'`) | The node `/twist_mux` runs, subscribed to `joy_vel`, `key_vel`, `nav_vel`, `e_stop`, `pause_navigation`, publishing `cmd_vel_out` (from `ros2 node info`) | | |
| Run the collision monitor with the launch file | `lifecycle_manager_collision_monitor`: "Managed nodes are active"; `ros2 lifecycle get /collision_monitor`: `active [3]` | | |
| "or by hand": the three `ros2` commands as printed | `env: ros2: No such file or directory`: unlike the two commands before it, this block is not wrapped in `pixi run bash -c 'source install/setup.bash && …'`. Inside the environment the three commands work (configure and activate: "Transitioning successful") | | |
| `pixi run demo`, then `pixi run demo-estop` and `pixi run demo-release` from a second process | The four processes started; the narrator printed, in order, `planner drives: it is the only live input and no lock is active / theorem forward_of_highest_live`, `e-stop lock ENGAGED`, `nothing is forwarded: the e-stop lock (255) is above every input / theorem not_forward_of_top_lock`, `e-stop lock released`, `planner drives …`: the lines the README shows. The turtlesim window was **not seen**. `demo-teleop` was not run (it needs a keyboard) | 40 s | |
| `pixi run gazebo`, `gazebo-nav2`, `gazebo-teleop` | Not run: they need a person at the screen (see "Not checked") | | |
| `pixi run cm-check` | `PASSED in 95 s (27 s simulated)`: first zero with the box 0.265 m away, 2 mm moved after it, at rest 0.263 m, no flicker. The first run downloads the apartment's furniture | 96 s | |
| `pixi run cm-check --monitor nav2_collision_monitor` | `PASSED in 32 s (27 s simulated)`: first zero at 0.265 m, 4 mm, at rest 0.263 m | 34 s | |
| `pixi run cm-check --nav2` (the tutorial's, not in the README's list) | `PASSED in 62 s (57 s simulated)` on its first start in this fresh environment: the node `collision_monitor` active, `Managed nodes are active`, the executable is the fresh clone's `roslean_collision_monitor/collision_monitor`; first zero at 0.275 m, 0 mm, at rest 0.275 m, no flicker | 63 s | |
| `pixi run cm-gazebo-scenario` (with a window), stopped by the audit after 90 s | From the log, in order: box dropped 1.2 m ahead; `monitor: SLOWDOWN, the command is published times 0.40`; `monitor: STOP, zero is published instead of the command`; `the robot is at rest, 0.26 m from the box`; `monitor: publishes NOTHING although commands arrive`; `box removed`; `the command passes unchanged`; `the robot drives on`. The window was **not seen** | 90 s | |
| `docker build -f docker/Dockerfile -t … .` from the fresh clone, as the README gives it (tests on), with `--no-cache` and under the tag `t12-audit-roslean-twist-mux`, because the README's tag exists on this machine | Built, exit 0. In the image's build: `Summary: 28 tests, 0 errors, 0 failures, 0 skipped` beside apt's `nav2_collision_monitor` 1.3.14 and `twist_mux` 4.5.0; `roslean_scenario` 318 checks in 73 runs; `roslean_nav2_scenario` `0 failure(s) in 272 checks, 40 scenario runs, 6 not run` (four because `nav2_bringup`, whose parameter file they use, is not in the image; the two the profile never runs); the install check passed (57 exported symbols, `libleanshared.so` 194 MB). The image is 1.65 GB. The README said the tests had not been run in this image since the Nav2 extension: corrected | 39 min 17 s | |
| `docker run --rm <image>` (without `--network host`) | The mux starts with the example configuration (five handler lines as on macOS); stopped after 25 s. The third command, with `-v $PWD/my.yaml`, was not run | | |

Altogether a fresh macOS machine needs about 21 GB (12 GB in the
repository, 8.5 GB in the home directory) and, on this network, about five
minutes before the first test can run.

## (B) Linux arm64 container: the README, step by step

| Step | Result | Time |
| --- | --- | --- |
| The bare image | No `git`, no `curl`, no certificates: neither the repository nor the two installers can be fetched. `apt-get update && apt-get install -y curl ca-certificates git` | 6 s |
| Clone, install elan and pixi | ok. pixi: "Could not detect shell type. Please permanently add '/root/.pixi/bin' to your $PATH" | 6 s |
| `pixi run build` | **Failed**, exit 127, in `roslean_core`: `CMAKE_STRIP-NOTFOUND --strip-unneeded -o …/libleanshared.so …`: `/bin/sh: 1: CMAKE_STRIP-NOTFOUND: not found`. Finding 1 | 41 s (environment: 5.9 GB) |
| `pixi run prove` | `Build completed successfully (6659 jobs)`, exit 0. Does not depend on the build | 1 min 15 s |
| Workaround 1: `apt-get install -y binutils`, then `pixi run build` again | **Failed** in the same way: the missing tool is cached in `build/roslean_core/CMakeCache.txt`. After `rm -rf build install log`: **failed**, exit 1, linking `test_cm_ffi`: `undefined reference to symbol 'pthread_create@@GLIBC_2.17'` … `libpthread.so.0: error adding symbols: DSO missing from command line`. Finding 2 | |
| Workaround 2: `rm -rf build install log`, then `pixi run bash -c 'export LDFLAGS="$LDFLAGS -pthread"; colcon build --symlink-install --base-paths ros --cmake-args -G Ninja'` | `Summary: 4 packages finished [24.2s]` | 24 s |
| `pixi run test-quick` | `Summary: 16 tests, 0 errors, 0 failures, 0 skipped`, exit 0. One test fewer than on macOS: `t2_point_as_upstream` is built on macOS only | 6 min 30 s |
| Run the mux; run the collision monitor with the launch file | Both as on macOS (`/twist_mux` in the graph; "Managed nodes are active", `active [3]`) | |
| `pixi run demo` | **Stops at the display**: `qt.qpa.xcb: could not connect to display`; `turtlesim_node` dies with exit code -6, the planner and the narrator go on | |
| `pixi run cm-check` | **Stops at the display, without saying so**: after `[obstacle] waiting for the robot to drive straight with free space ahead` nothing more was printed; ended by the audit after 150 s. The README calls this check "without windows"; it still needs a display to render the lidar (the tutorial says so in its table of symptoms). `pixi.toml` has `cm-check-cloud` for a machine without a display (needs `xvfb-run`); the README does not mention it, and it was not tried | 150 s |
| `pixi run test` | `Summary: 28 tests, 0 errors, 0 failures, 0 skipped`, exit 0 (below) | 38 min 57 s |

Disk in the container at the end: 20 GB under `/root` (5.9 GB `.pixi`,
7.9 GB `.lake`, 3.4 GB `~/.elan`, 1.7 GB caches). The stripped
`libleanshared.so` is 194 MB, as the README says ("about 190 MB").

## What the README assumed without saying

Corrected in the README by this audit unless marked otherwise.

- `git` and `curl`; on macOS the Xcode command line tools. Whether the
  build works on a Mac without them was **not tested** (they could not be
  hidden).
- How to get the repository, and that elan and pixi each need their own
  installer and a new shell.
- Disk: the README said "about 6 GB" for the ROS environment. Measured:
  4.5 GB on macOS and 5.9 GB on Linux, as much again in pixi's cache,
  3.3 GB of Lean toolchain, 7.8 GB of Mathlib: about 21 GB.
- A display for every demo, and on Linux also for `pixi run cm-check`.
- That the "by hand" commands are meant inside the environment.
- On Linux: `binutils` and the `-pthread` flag (findings 1 and 2; **not**
  something a README should have to say).
- Not corrected, because it is not an error: nothing says how long
  anything takes on an ordinary connection.

## The full test suite

`pixi run test` was run once on each fresh clone, as the README gives it.
No test failed in either run, so nothing was retried.

| | (A) macOS arm64 | (B) Linux arm64 container, after the two workarounds |
| --- | --- | --- |
| Summary line | `Summary: 29 tests, 0 errors, 0 failures, 0 skipped` | `Summary: 28 tests, 0 errors, 0 failures, 0 skipped` |
| Exit code, wall-clock time | 0, 39 min 11 s | 0, 38 min 57 s |
| `roslean_core` | 2 of 2 passed, 28 s | 2 of 2, 22 s |
| `roslean_twist_mux` | 2 of 2, 472 s: 69 decisions in the scenarios, 4,692 in 20 random cases, 0 divergences | 2 of 2, 479 s: the same counts, 0 divergences |
| `roslean_collision_monitor` | 25 of 25, 1,851 s | 24 of 24, 1,835 s (`t2_point_as_upstream` is macOS only) |
| Upstream beside which it ran (from the environment's `conda-meta`) | `nav2_collision_monitor` 1.3.12, `twist_mux` 4.5.0 | `nav2_collision_monitor` **1.3.13**, `twist_mux` 4.5.0 |
| `roslean_scenario` | `0 failure(s) in 318 checks, 73 scenario runs` | the same |
| `roslean_nav2_scenario` | `0 failure(s) in 289 checks, 44 scenario runs, 2 not run: ['footprint-approach-unchanged', 'shift-interpolation']` | the same |
| `roslean_fault_injection` | `0 failure(s) in 109 checks, 19 cases` | the same |
| `upstream_scenario`, `upstream_nav2_scenario` | 277 checks in 55 runs; 299 checks in 46 runs; 0 failures | the same |
| `roslean_nav2_random`, `upstream_nav2_random` | 0 of 3 and 0 of 10 cases disagree with the model | the same |

What ran beside these on the same machine: during the macOS run, the
README's two node commands (once, on ROS domain 0) and the container's
`test-quick` and full run, the latter for half an hour; during
`test-quick` on macOS, 26 short starts of a node on ROS domain 231
and the container's builds. No failure came of it. The whole suite takes
39 minutes, not "about 20 minutes … longer now" (README corrected).

`pixi.lock` pins a different upstream for each platform:
`nav2-collision-monitor` 1.3.12 and Python 3.12 for `osx-arm64`, 1.3.13 and
Python 3.14 for `linux-64` and `linux-aarch64`. The pages name 1.3.12
(macOS, RoboStack) and 1.3.14 (Linux, apt) as the binaries the node was
compared with; a reader who follows the README on Linux compares with a
third, 1.3.13. In this run every check passed beside it (finding 11).

## Findings

Matters of substance, left as they are. Numbered for reference.

1. **Linux through pixi: no `strip`, the build of `roslean_core` fails.**
   `ros/roslean_core/CMakeLists.txt`, line 132, runs `${CMAKE_STRIP}`. In
   the pixi environment on Linux arm64 CMake finds neither `strip` nor `ar`
   (`CMAKE_STRIP:FILEPATH=CMAKE_STRIP-NOTFOUND`,
   `CMAKE_AR:FILEPATH=CMAKE_AR-NOTFOUND` in the cache): the environment
   has `aarch64-conda-linux-gnu-strip` and no plain `strip`, and CMake
   finds the compiler as `cc` (a link to `aarch64-conda-linux-gnu-gcc`);
   `STRIP` and `AR` are not set in the environment. It works wherever the
   system has `binutils`, as the Docker image's base has; that the cloud
   session (T00) had them is inferred, not checked. Reproduce:
   `docker run -it ubuntu:24.04`, install `curl ca-certificates git`, elan
   and pixi, clone, `pixi run build`. A fix would be to look for the
   prefixed tool as well and to fail at configure time with a message when
   none is found. After installing `binutils` the build tree must be
   removed, because the missing tool stays in the cache.
2. **Linux through pixi: `test_cm_ffi` does not link.**
   `ros/roslean_core/test/test_cm_ffi.cpp` uses `std::thread` (the test of
   `lean_initialize_thread`, added after review CM5), and
   `ros/roslean_core/CMakeLists.txt` links it without `Threads::Threads`.
   The pixi environment's compiler links against its own glibc 2.28
   sysroot, where `pthread_create` is in `libpthread`; on Ubuntu's glibc
   and on macOS it is in the C library, so the apt-based Docker build and
   macOS do not notice. So **the tree as it is does not build on Linux
   through pixi**, which is the route the README's quick start describes.
   (Seen on arm64. The sysroot is conda's on x86-64 too, so the same is
   expected there; not run.)
3. **`pixi run cm-check` hangs without a display and does not say why**
   (container, above). `check_demo.py` waits for the robot to drive; the
   lidar never publishes. T00 reported the related case that it does not
   notice when Gazebo exits.
4. **All collision monitor links on the three pages point to
   `blob/main/…` or `tree/main/…`, and none of their targets is on `main`
   yet** (37 links to 26 files and directories; all exist on `collision-monitor`). The
   pages are deployed from `main`, so the links become valid with the same
   merge. The links with line numbers (`Properties.lean#L707` and so on)
   were all checked against this branch and are right today; any later
   edit of those files moves them. `https://beneficial-ai-foundation.github.io/ROSlean/gaps.html`
   returns 404 today for the same reason.
5. **The headline cost figures have no raw data in the repository.**
   "48.9 µs against 7.2 µs", the table of six pairs, "6 to 9 times" and the
   latencies through the nodes (5.8, 6.7, 7.5 ms against 6.3, 6.6, 6.4 ms)
   appear in `tasks/reports/T06.md` and `SPEC_NAV2.md` and nowhere else:
   not in COMPARISON.md, and `bench/results/cm_core.json` and
   `cm_node.json` are from the earlier node (commit `86ae565`, task T02).
   The pages agree with T06's report. The in-process figures were
   re-measured by the audit with T06's command (`pixi run bench-build`,
   then `bench_cm --readings N --zones Z --scan clear --iters 20000
   --warmup 2000`, one run each, in the fresh clone, while a Docker build
   was running on the same machine). Median per command, Lean / upstream,
   µs: 11.0 / 1.8 and 17.3 / 2.9 at 360 readings (pages: 10.3 / 1.7 and
   16.5 / 2.8); 31.8 / 4.2 and 49.9 / 7.4 at 1080 (30.0 / 4.1 and
   48.9 / 7.2); 290 / 32 and 461 / 61 at 10,000 (281 / 31 and 438 / 60).
   Ratios 6.0 to 9.0: "6 to 9 times" holds, and every figure is within 7 %
   of the page's. The scan callback: 3.8, 11.2 and 97 µs ("about 10 ns per
   reading"). The latencies through the nodes were not re-measured. What
   remains of the finding: the published table's own data is not kept.
6. **"4 to 9 times upstream's processor time"** in the table "Does it fit
   your robot" of `gaps.html`, against "6 to 9 times" everywhere else. The
   six pairs of the table give 5.9 to 9.1. The 4 may be the Linux figure
   (4.2 to 6.4, an earlier version); the sentence says "on the Mac". Left,
   because which was meant is the author's to say.
7. **`pixi.toml` says of `test-quick`: "a minute or two instead of
   twenty"**; the README says "about 7 minutes", and it took 7 min 6 s
   (macOS) and 6 min 30 s (Linux). The README is right.
   `ros/roslean_core/CMakeLists.txt` says the macOS library is "about
   4 MB"; it is 2.5 MB (README corrected, the comment is code).
8. **The twist_mux comparison is with version 4.5.0, the specification is
   read from 4.5.1.** `details.html` and `docs/SPEC.md` name the source tag
   4.5.1; the environment's binary and BENCHMARK.md's sources are 4.5.0.
   `details.html` does not say which version the 69 decisions and 4,692
   random decisions were compared with ("`ros-jazzy-twist-mux` from apt").
9. **Three statements leave out a hypothesis of their theorem.** None is
   false as a description of the node; each is narrower in Lean than in
   the sentence. (Corrected in README.md and `details.html` after
   the audit, by the orchestrator; the description below is of the audited
   commit.)
   - README, "Stale data, no transform" (`stale`, `no_transform`,
     `run_stale`, `run_no_transform`): "With a non-zero `source_timeout`".
     The theorems also require the source to be enabled
     (`he : cfg.sourceEnabled = true`). `details.html` has it in the row
     for `evaluate_eq_stopSource_iff` and not in the rows for `stale` and
     `no_transform`.
   - `details.html`, `noClock_velocity`: "A finite command that arrives
     while the node has no valid clock reading is answered with zero". The
     theorem requires the node to be active (`hp : st.phase = .active`).
     The README covers this with its sentence '"Command" means a velocity
     command that arrives while the node is active'; `details.html` has no
     such sentence.
   - README and `details.html`, `run_stop`: stated like `stop`. `run_stop`
     is about a stop zone with a fixed shape (`hfix : z.shape = .fixed
     sh`), an enabled source, a scan that is not too old and coordinates
     that fit it. THEOREMS.md has these.
10. **The Lean files under `review/` and `bench/experiments/` are not
    built by `pixi run prove`** (`lake build ROSlean`; eight files, among
    them `review/collision_monitor/Checks.lean`, which `gaps.html` cites
    for the 300,000 patterns per helper). Their results are as recorded in
    REVIEWS.md; not re-run here.
11. **Three upstream versions, of which the pages name two.** `pixi.lock`
    gives Linux `nav2_collision_monitor` 1.3.13 (see "The full test
    suite"). SPEC.md says the cited files are byte-identical in 1.3.12 and
    1.3.14; nothing in the repository says so of 1.3.13, though the suite
    passing beside it is evidence that it behaves alike where tested.
12. **`pixi run cm-check --nav2` came up on its first start here**, in a
    freshly installed environment, where T10's first start failed. One
    start; it says nothing about how often Nav2 fails to come up. The
    pages give T10's count (one failed start, none in 27 further ones).
13. **The tests and the demos leave files in `/tmp`.** After one
    `test-quick` and one `test` on macOS: 164 entries
    `/tmp/roslean_cm_*` and `/tmp/roslean_random_diff_*` (logs and
    parameter files of the scenario runs). After the launches and
    `cm-check --nav2`: 40 `launch_params_*`, a copy of Nav2's launch
    file and of the parameter file. Nothing removes them.

## Discrepancies corrected in this branch

| Where | Was | Is | Evidence |
| --- | --- | --- | --- |
| README "Under Nav2"; `gaps.html` "approach" | `1 parameter refused: parameter 'FootprintApproach.action_type': …` as the node's answer to Nav2's file | `2 parameters refused: …; parameter 'FootprintApproach.polygon_sub_topic': is not initialized, and the zone has no 'points': …` | Observed: the node on the `collision_monitor` section of the environment's `nav2_bringup/params/nav2_params.yaml`, unchanged. The one-parameter message is what an `approach` zone that has `points` gets |
| README "Cost and platforms"; `details.html` and `gaps.html`, the notes under the cost tables | "one Apple M5" | "one Apple M5 Pro" | `sysctl -n machdep.cpu.brand_string`; COMPARISON.md and the reports T02, T03 say M5 Pro; `gaps.html` said both |
| README "Linux and deployment" | `libroslean_core` on macOS: 4 MB | 2.5 MB | Observed: 2,470,280 bytes in `install/roslean_core/lib/` |
| README "Quick start" | "about 6 GB"; no word on `git`, `curl`, how to clone, the toolchain, Mathlib's size, a new shell | the measured sizes; the prerequisites; a `git clone` line | (A) and (B) above |
| README, "or by hand" | three `ros2` commands with no environment | "inside the environment (`pixi run bash`, then `source install/setup.bash`)" | Observed: `ros2: No such file or directory` |
| README "Linux and deployment" | "its tests have not been run in this image since the Nav2 extension" | the audit's build with tests and its summary line | the Docker row of (A) |
| README "Quick start", "What has been run where" | "`pixi run test` … about 20 minutes before the collision monitor's Nav2 extension, longer now"; no entry for Linux through pixi on the present tree | "about 40 minutes"; an entry for the fresh-clone run on macOS and one for Linux arm64 through pixi, with the two failures and the workarounds | (A), (B), and the two full runs |
| `details.html`, "Further reading" | a link "API documentation" to `docs/` | removed | No `site/docs/`; the Pages workflow publishes `site/` with no build step; the deployed URL returns 404 |
| `gaps.html`, "Platforms never tested", row "Linux arm64, container" | on the final node only the Nav2 scenarios, 20 random cases and the bond test | adds the audit's two full runs on Linux arm64 (the Docker image beside 1.3.14, pixi beside 1.3.13) | the Docker row of (A); "The full test suite" |
| `gaps.html`, "Platforms never tested", last row; `docs/PLAN.md`, "Not done", first item | "`pixi run test` as a whole was not run by any task" | the two runs of the audit and their summary lines; the Linux build failure as what is not done | "The full test suite" |
| README "Demo", "Under Nav2"; `index.html` "The demo" and the note under "Collision monitor"; `details.html` "The demo"; `gaps.html` "Under Nav2" and "Does it fit your robot"; `docs/PLAN.md` row T10 and "Not done" | pointers to the tutorial "for the run with a simulated robot", written before T10 reported; "in one run with the node as a lifecycle node" | T10's results, below | `tasks/reports/T10.md`; TUTORIAL_COLLISION_MONITOR.md, sections 5 and 7. Read, not re-run, except as noted under (A) |

What the pages now say of T10, each from T10's report and the tutorial:

- The check behind the mux with the node as it is now: three runs, three
  passed (at rest 0.254 to 0.260 m, 0 to 2 mm after the first zero).
  Before: "one run".
- The Nav2 variant's check: 10 runs of 10 passed with the Lean node, 4 of
  4 with upstream's node on the same parameters (`details.html` has the
  table of ranges).
- The first start of Nav2 in a freshly installed environment failed
  (lifecycle manager stopped at `Configuring behavior_server`); it did not
  recur in 27 further starts.
- **With the stop zone, Nav2's tour ended with the robot standing for good
  in three runs of six**: two of three with the Lean node (1, 1, 8 places
  reached), one of three with upstream's node on the same zones (8, 2, 5).
  Stock Nav2 with `approach` reached 11 places in its one run and did not
  stand. A stop zone is not a substitute for `approach`. Stated in the
  README ("Under Nav2", and a pointer in "Demo"), on the overview page
  (twice), in `details.html` (with the table of the seven runs), in
  `gaps.html` ("Under Nav2", and a new row in "Does it fit your robot") and
  in `docs/PLAN.md`.
- Nobody has watched either variant in a window.

## Numbers and claims that were checked and hold

Against the repository's files (read), unless "observed":

- Line counts: `Mux.lean` 152 ("150"); the mux shim 205 ("200"); the model
  1,232 lines in four files ("1,200"), 1,907 with the C interface
  ("1,900"); the proofs 8,939 ("8,900"); the node 1,393 ("1,400"), 1,959
  with its three headers ("2,000"); `transforms.hpp` 259 ("260"), with
  `conversions.hpp` 521 ("500"); `Geometry.lean` 715 ("700");
  `Convert.lean` and `Bits.lean` 724 ("700").
- Rule numbers: R1–R16 in `docs/SPEC.md` ("sixteen"); the ranges R1–R19,
  N1–N36, D1–D15, ND1–ND13, T1–T10, A1–A4 as the README gives them;
  15 + 13 = 28 differences ("Twenty-eight").
- twist_mux: 69 of 69 decisions and 0 divergences in 4,692 (REVIEWS.md);
  every figure of the two performance tables (BENCHMARK.md).
- Collision monitor comparison, against COMPARISON.md, "After the fixes of
  the review CM5": 318 checks in 73 scenario runs; 289 checks in 44 runs,
  2 not run; the stated differences D1 1, D4 20, D5 3, D6 1, D8 1, D9 2,
  D10 2 (30) and ND4 2, ND5 1, ND6 4, ND11 3 (10); 300 cases with 26,951
  commands, 586 differing, farthest 2.056 and 3.044 mm; 300 cases with
  16,797 commands; 26,951 + 16,797 = 43,748, the overview's figure; 109
  checks in 19 fault cases; 34.0 million points on 340 zones, 3.409 and
  4.167 mm; Linux: 289 checks, 20 cases with 1,182 commands. Of these the audit's own run of `pixi run test` printed: `0 failure(s) in
  318 checks, 73 scenario runs`; `0 failure(s) in 289 checks, 44 scenario
  runs, 2 not run: ['footprint-approach-unchanged',
  'shift-interpolation']`; `0 failure(s) in 109 checks, 19 cases`; for
  twist_mux 69 decisions in the scenarios and 4,692 in 20 random cases, 0
  divergences (against the environment's 4.5.0 binary). The random runs of
  300 cases, the point comparison and the Linux counts are not part of
  `pixi run test` and were not repeated.
- 11 tests on x86-64 (`tasks/reports/T00.md`); the 32-minute soak and its
  48 kB and 84 kB (COMPARISON.md); 300,000 patterns (REVIEWS.md); 530,000
  values and 200,000 transforms (THEOREMS.md, SPEC.md); the budget of 50,
  150, 1500 µs.
- Versions: upstream source at `c704655`, 1.3.14 (SPEC.md); the binary in
  the environment is `nav2-collision-monitor` 1.3.12 and Gazebo Sim 8.10
  (observed, `conda-meta`); `rmw_fastrtps_cpp`.
- The demo's geometry: stop zone to 0.30 m, slowdown zone to 0.70 m with
  ratio 0.4 (`demo/gazebo/monitor.yaml`), a 0.4 m box 1.2 m ahead
  (`obstacle.py`); the turtlesim demo's priorities 10, 100, 255 and the
  half second (`demo.yaml`); the narrator's lines (`narrator.py`).
- Every path of the README's layout table, every relative link of the
  README, every local file and anchor of the three pages, every external
  link tried (GitHub, index.ros.org, pixi.sh, the foundation's page, the
  benchmark branch and pull request 1): present, apart from finding 4 and
  the removed `docs/` link.
- Observed in the audit's own runs: `17 tests, 0 failures` for
  `test-quick`; "about 7 minutes"; `libleanshared.so` about 190 MB on
  Linux; every refusal message of `gaps.html` but one (next section); no
  `~/toggle` service; no collision point marker topic; a refused
  configuration leaves the process up and the node unconfigured.

## Theorems

**Names.** Every identifier in code font in the four files that is a
theorem of the Lean sources was collected by script: 72, and
`evaluate_isStop_iff`, which the overview page prints in full. All 73
exist exactly once. No name on the pages that looks like a theorem is
missing from the sources. All 46 links with a line number point at the
line where the theorem named begins.

**Statements.** The statement of each of the 73 was read beside every
sentence that cites it. The three statements printed on the overview page
(`forward_iff`, `evaluate_isStop_iff`, `no_amplification`) are the sources'
statements, compared by eye, token for token. The rest say what
the text says, with the three omissions of finding 9.

**Axioms.** A Lean file that imports `ROSlean` and calls `collectAxioms`
(what `#print axioms` prints) on each named theorem and then on every
theorem of every `ROSlean.*` and `ROSleanCore.*` module, run with
`lake env lean` in the fresh clone after `pixi run prove`:

- the 73 named theorems: 67 depend on `propext`, `Classical.choice`,
  `Quot.sound`; 4 on `propext`, `Classical.choice` (`muxForward_after`,
  `step_cmd_of_not_finite`, `triggered_unusable_of_triggered`,
  `velocity_eq_some_iff`); 1 on `propext`,
  `Quot.sound` (`refresh_of_refused`); 1 on `propext` alone
  (`noClock_velocity`). None depends on anything else.
- all 1,084 theorems of the project: the union of their axioms is
  `propext`, `Classical.choice`, `Quot.sound`. No `sorryAx`, no
  `Lean.ofReduceBool` (which `native_decide` would leave), no axiom of the
  project's own.

## Refused configurations

Each case: the example configuration or Nav2's `collision_monitor` section
with one change, the stand-alone executable started with it,
`ros2 lifecycle set /collision_monitor configure`, then the state and the
log. macOS, the fresh clone's build. In every refused case `configure`
answered "Transitioning failed", the node was `unconfigured [1]` afterwards
and the process was still running, as `gaps.html` says.

| Configuration | The node's message (after `N parameter(s) refused: `) | As quoted? |
| --- | --- | --- |
| Nav2's `nav2_params.yaml`, unchanged | `parameter 'FootprintApproach.action_type': only 'stop' and 'slowdown' are supported; got 'approach'. (To use Nav2's footprint zone, give it 'points', or 'action_type: stop' together with 'polygon_sub_topic'; 'footprint_topic' is read for 'approach' only.); parameter 'FootprintApproach.polygon_sub_topic': is not initialized, and the zone has no 'points': static points and sub topic both not defined. 'footprint_topic' is read for 'action_type: approach' only, which this node does not provide; give the zone 'points' (recommended), or set 'polygon_sub_topic' to the topic the footprint is published on` | **No**: two parameters, the pages quoted one. Corrected |
| The same on upstream's node | configures | as stated |
| `approach` on a zone of the example, which has `points` | the first of the two parts, as `1 parameter refused` | yes |
| Nav2's file with only `action_type: stop` | `parameter 'FootprintApproach.polygon_sub_topic': is not initialized, and the zone has no 'points': …` | yes (the README says it does not configure) |
| The same on upstream's node | `Error while getting common polygon parameters: parameter 'FootprintApproach.polygon_sub_topic' is not initialized`; `static points and sub topic both not defined`; "Transitioning failed" | yes |
| Nav2's file with the documented edit (`FootprintStop` with `points`) | configures, on both nodes | yes |
| Nav2's file with `action_type: stop` and `polygon_sub_topic` | configures, on both nodes; the Lean node warns that the zone takes no action until its first message | yes |
| `action_type: limit` | `parameter 'PolygonSlow.action_type': only 'stop' and 'slowdown' are supported; got 'limit'. (…)` | yes |
| `action_type: none` | the same with `got 'none'` | yes |
| `type: velocity_polygon` | `parameter 'PolygonStop.type': only 'circle' and 'polygon' are supported; got 'velocity_polygon'` | yes |
| a circle with `polygon_sub_topic` and no `radius` | `parameter 'CircleSlow.radius': required: a circle whose radius comes from a topic is not supported (ND12)` | yes |
| source `type: pointcloud`, `range`, `polygon` | `parameter 'scan.type': only 'scan' sources are supported; got 'pointcloud'` (`'range'`, `'polygon'`) | yes |
| two sources | `parameter 'observation_sources': exactly one source is supported; 2 given` | yes |
| `sources_names: ["other"]` | `parameter 'PolygonStop.sources_names': must be the one source 'scan' or unset` | yes |
| `sources_names: ["scan"]` | configures | yes ("or unset") |
| `max_points` | `parameter 'PolygonStop.max_points': deprecated and not supported; use min_points (D10)` | yes |
| `use_realtime_priority: true` | `parameter 'use_realtime_priority': real-time priority is not supported` | yes |
| four of these at once | `4 parameters refused: parameter 'use_realtime_priority': …; parameter 'scan.type': …; parameter 'PolygonStop.max_points': …; parameter 'PolygonSlow.action_type': …` | yes, one line of the stated form |
| a negative `slowdown_ratio` | `parameter 'PolygonSlow.slowdown_ratio': must not be negative (D5)` | (not quoted; D5 as stated) |
| a negative `source_timeout` | `parameter 'source_timeout': a timeout must be finite, not negative and below 9.2e9 s (D9)` | (not quoted; D9 as stated) |
| upstream's own example file (`nav2_collision_monitor/params/collision_monitor_params.yaml`), unchanged | configures: the zones and the source it defines without listing (`PolygonLimit`, `FootprintApproach`, `VelocityPolygonStop`, `pointcloud`) are not looked at | yes ("defines … but does not list it") |
| the example, unchanged: configure, activate | both "Transitioning successful"; no service with "toggle" in its name; topics: `/bond`, `/cmd_vel`, `/cmd_vel_smoothed`, `/collision_monitor_state`, `/polygon_slowdown`, `/polygon_stop`, `/scan`, `/tf`, `/tf_static` and the node's own: no marker topic | yes |
| `ros2 param set /collision_monitor PolygonStop.enabled false` on the active node | `Setting parameter failed: parameters of this node are read once, when it is configured, from the values given at start-up, and cannot be changed while it runs (docs/collision_monitor/SPEC.md, ND8); to change 'PolygonStop.enabled', restart the node with the new value`; the value stayed `True` | yes |

Not tried, because they need more than a parameter file: what the node
does after configuration (the gaps about a topic zone without shape, an
unusable vertex, a refused refresh, the component on a negative `/clock`,
`stop_pub_timeout`). Those rest on the theorems named and on the test
suite (below).

## `sorry`, `native_decide`, `axiom`, `unsafe`, `implemented_by`, `@[extern]`

`grep -rnE 'sorry|native_decide|axiom|unsafe|implemented_by|extern'` over
all 35 `.lean` files of the repository (root, `core/`, `ROSlean/`,
`review/`, `bench/experiments/`; not Mathlib under `.lake/`):

| Word | Hits | What they are |
| --- | --- | --- |
| `sorry` | 0 | |
| `native_decide` | 0 | |
| `axiom` | 42, all in `review/` | 41 `#print axioms NAME` commands in four review files (`InterfaceChecks.lean` 15, `nav2/ModelProbes.lean` 2, `nav2/NoClockUnreachable.lean` 22, `nav2/NoClockMixedRuns.lean` 2) and one comment line. No `axiom` declaration |
| `unsafe` | 0 | |
| `implemented_by` | 0 | |
| `extern` | 0 | |

Looked for in addition: `@[export …]`, 40 declarations in
`core/ROSleanCore/FFI.lean` (7), `CollisionMonitorFFI.lean` (32) and
`Noop.lean` (1): the C interface, Lean functions given a C name, which is
what the README describes; `opaque` (two hits, both the word in a
comment); `partial` (one hit, the word in a comment). So no declaration of
the project is replaced at run time by other code, and the interface
theorems are about the functions that are exported.

## Not checked, and why

- **Any window.** The Mac's screen was locked for the whole audit
  (`CGSSessionScreenIsLocked` true). The turtlesim demo, the Gazebo
  scenario with its window and the three headless checks ran, and what is
  reported of them is their text output. Nobody saw the turtle, the robot,
  Gazebo's window or RViz. No screenshot was taken.
- **Anything that needs a keyboard:** `pixi run demo-teleop`,
  `pixi run gazebo-teleop`.
- **`pixi run gazebo` and `pixi run gazebo-nav2`** (the twist_mux Gazebo
  demo): not started. Their tutorial is not in the audit's scope and their
  result is something to look at.
- **Nav2 driving the tour** (`pixi run cm-gazebo-bringup`,
  `nav2_observe.py tour`): not repeated. The tour figures on the pages are
  T10's, seven runs of 300 simulated seconds.
- **macOS without the Xcode command line tools**, and any Mac other than
  this one.
- **x86-64, Linux on its own hardware, a machine with a slow network.**
- **The demos on Linux**: no display in the container; `cm-check-cloud`
  (`xvfb-run`) was not tried.
- **`docker run … -v $PWD/my.yaml:/cfg.yaml …`**, the README's third Docker
  command: the README names no such file to start from.
- **`pixi run bench`, `pixi run bench-cm`, `pixi run cm-diff`,
  `pixi run cm-faults`, `bench/cm_linux/run.sh`**: about five hours
  together by their own descriptions. Of the benchmark only the in-process
  figures were re-measured (finding 5). The random runs of 300 cases, the
  34.0 million points, the soak, the Linux container's counts and the
  latencies through the nodes were compared with COMPARISON.md and the task
  reports, not run.
- **The x86-64 figures of twist_mux** (BENCHMARK.md): compared with the
  document only.
- **Whether each theorem is the right theorem.** The audit checked that the
  statements say what the sentences beside them say, and what they rest
  on. It did not review the model against upstream's source, the proofs'
  definitions (`IsFirstStop`, `SourceInvalid`, `ZeroPublishedLast` and the
  like were read only as far as the sentences needed), or the C++ shim.
  That is what REVIEWS.md records.
- **THEOREMS.md, SPEC.md, COMPARISON.md, the tutorials** as documents:
  used as sources, not audited. Where the README or a page disagrees with
  one of them it is listed above; whether they are right is outside this
  task.

## Left on the machine

Removed at the end: the fresh clone and the empty home directory under
`$TMPDIR/t12-fresh/`, the container `t12-fresh-b`, the image built from the
README's Dockerfile (under the tag `t12-audit-roslean-twist-mux`, so that
the existing `roslean-twist-mux` image was not replaced), the `ros2`
daemons the fresh clone had started, the files of finding 13. Left: the
`ros:jazzy-ros-base` base image in Docker's store (`ubuntu:24.04`, which the
audit had pulled, was removed), Docker's build cache, and whatever Fast DDS
keeps in `/tmp/boost_interprocess`, which other sessions use too. The logs
of the runs are in the session's scratch directory, not in the repository.
