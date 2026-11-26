# Binary packages: install the nodes without Lean and without a build

A proposal. Statements marked *(not verified)* come from memory of the tools' documentation or from an inconclusive search, not from a run or from this repository.

## Today

To use `roslean_twist_mux` or `roslean_collision_monitor` a user installs elan, clones the repository and builds with colcon (`README.md`, "Quick start"; about 21 to 24 GB with pixi and every step), or builds the Docker image themselves (`docker/Dockerfile`; 39 minutes with the tests in the arm64 audit). Nothing is published: no container image, no archive, no `.deb`, no conda package. The repository has one tag, `v4.37.0-rc1`, which is the Lean version (set by `.github/workflows/create-release.yml`), and no release of the nodes.

## Goal

One command installs the nodes on a machine with ROS 2 Jazzy. No Lean, no compiler, no checkout.

## What a build needs and what a run needs

| | Build | Run |
| --- | --- | --- |
| Lean toolchain (`lean-toolchain`: 4.37.0-rc1; 3.3 GB unpacked on macOS) | yes: `lake build` in `core/` writes nine C files (about 1.1 MB); the C compiler then needs `include/lean/lean.h` and the runtime libraries from the same toolchain | no |
| Mathlib, the top-level Lean package | no; proofs only (`docker/Dockerfile` copies `core/` alone) | no |
| Lean's runtime | linked | in the install tree. macOS: static, inside `libroslean_core` (the package is 2.4 MB). Linux: a stripped copy of `libleanshared.so` beside it, 194 MB |
| ROS 2 Jazzy; for the monitor, Nav2 (`nav2_util`, `nav2_msgs`, tf2, lifecycle) | yes | yes |

The install tree refers neither to the toolchain nor to the checkout; `docker/check_install.sh` tests that on Linux. It can be copied to a machine of the same OS and architecture if built without `--symlink-install`. Whether it can be unpacked under another path was not tested; one path in the `nav2_collision_monitor` overlay (upstream's detector library) is absolute.

Why 194 MB: Lean's static runtime is compiled for thread-local storage in executables, so on Linux it cannot go into a shared library (a link error on x86-64; `docs/PLAN.md`, lesson 14), and `libleanshared.so` holds all of Lean. Not tried: the static runtime linked into the node executables (the monitor's component is a shared library, so it would still need another way); a smaller shared runtime built from Lean's sources.

Licences. The `package.xml` files say Apache-2.0, but **the repository has no `LICENSE` file**. Lean is Apache-2.0 (`LICENSE` in the toolchain). The toolchain's `LICENSES` lists what its binaries bundle: LLVM (Apache-2.0 with LLVM exceptions; `libleanshared.so` carries a libc++), glibc (LGPL), GMP (LGPL v3), CaDiCaL (MIT). `roslean_core` installs both files into `share/roslean_core/lean_sdk`. Open: GMP is linked statically into `libroslean_core` on macOS, which has LGPL conditions; libuv is linked too and `LICENSES` does not mention it, nor mimalloc.

Platforms (`site/gaps.html`, "Platforms"): macOS arm64, everything. Linux arm64 in containers on the same Mac: the whole suite, 28 tests, with apt and with pixi. Linux x86-64: not run for the present collision monitor; an earlier version once (11 tests), and `twist_mux`, where the x86-64 build exposed the linking fault above. CI builds the Lean packages only (`lean_action_ci.yml`); no workflow builds or tests a ROS package.

## Prerequisites common to all options

1. **A `LICENSE` file**, and a check of the open licence points above.
2. **x86-64 in CI**: build and `colcon test` on `ubuntu-24.04` and `ubuntu-24.04-arm` runners (the arm64 runners are free for public repositories, per GitHub's documentation). Overlaps with `linux-hardware-run.md`, step 1; the 300-case comparison there stays a separate run.
3. **Versions**: tags for the nodes that do not collide with the Lean-version tags (for example `roslean-0.2.0`), the same number in every `package.xml`, and in each artefact's metadata the commit, the Lean version, and the ROS and Nav2 versions it was built and tested beside (1.3.12 to 1.3.14 so far, by set-up).
4. **A package for the demos.** `roslean_collision_monitor` and `roslean_twist_mux` name Gazebo, TurtleBot3 and `nav2_bringup` as run dependencies because the demos live in them. A `.deb` or conda package would pull in the simulator. Not needed for (a) and (b).
5. **A CI job that builds the artefacts** on a tag, per architecture, and tests the installed artefact (below).
6. **`roslean_nav2_bringup`** (launch files and parameters that start Nav2 with the Lean monitor; being written) belongs in every artefact: it is what makes the install usable in one command.

### Testing an installed artefact

- `docker/check_install.sh`: usable as it is for (a) and (b); it assumes colcon's isolated layout and needs adapting for a merged prefix (`.deb`, conda).
- Collision monitor: the Python tests are installed (`share/roslean_collision_monitor/test`) and take the node as `<package>/<executable>`: `scenario.py`, `scenario_nav2.py`, `fault_injection.py`, `random_diff.py`. They need upstream's `nav2_collision_monitor` beside the node, and four Nav2 scenarios need `nav2_bringup`.
- `twist_mux`: `scenario.py` and `random_diff.py` take the node as an argument but are not installed; run them from a checkout or install them.
- The C++ tests (`test_mux_ffi`, `test_cm_ffi`, `smoke_*`, `cmd_cost`) exist in the build tree only. They run in the job that builds the artefact, not on the artefact. `test_cm_ffi` throws and catches across the Lean runtime and must pass on every architecture.
- A start through `roslean_nav2_bringup` on a machine that never had Lean.

## Options

| | The user types | We build and maintain | Blocks or unknown | Effort (estimate) |
| --- | --- | --- | --- | --- |
| (a) Container image on GHCR | `docker run --rm --network host ghcr.io/beneficial-ai-foundation/roslean:jazzy` | A workflow that builds `docker/Dockerfile` for amd64 and arm64 on native runners and pushes a multi-arch manifest. The overlay and the bring-up are not in the image (`DROP_IN_COLLISION_MONITOR.md` on the `claude/cm-dropin` branch lists the Dockerfile changes) | amd64 never built for the monitor. Image is 1.65 GB. Rebuild when the `ros:jazzy-ros-base` image or Nav2 moves | small |
| (b) Install tree as an archive on GitHub Releases | `curl -L …/roslean-0.2.0-jazzy-noble-amd64.tar.zst \| sudo tar --zstd -x -C /opt`, `source /opt/roslean/setup.bash`; ROS dependencies by a listed `apt install` line | The same job as (a): archive `/opt/roslean` after `check_install.sh`. One archive per architecture; a macOS one only against a pinned pixi environment | Dependencies are not installed for the user. Built against apt's ROS of that day: whether it keeps loading after a Nav2 patch release is unknown. Unpacking at another path not tested | small |
| (c) `.deb` in our own apt repository | `sudo apt install ros-jazzy-roslean-collision-monitor` after adding a source and key | `bloom-generate rosdebian` and `fakeroot debian/rules binary` per package in a Noble container with elan; a signed repository (GitHub Pages, or a hosting service) *(routes not verified)* | The generated rules must not strip or rewrite the bundled `libleanshared.so`; `dh_shlibdeps` on a bundled library; a 190 MB package; rebuild at every Jazzy sync that changes Nav2. Prerequisite 4. The overlays cannot install into `/opt/ros/jazzy` (file collisions with `ros-jazzy-twist-mux`, `ros-jazzy-nav2-collision-monitor`) | medium |
| (d) conda packages in our own channel (prefix.dev) | `pixi add ros-jazzy-roslean-collision-monitor` with our channel added before `robostack-jazzy` | Recipes for `rattler-build`, built for `linux-64`, `linux-aarch64`, `osx-arm64` against `robostack-jazzy`; upload from CI (prefix.dev documents trusted publishing from GitHub Actions) | Lean at build time: no Lean package was found on conda-forge *(a search, not exhaustive)*, so the recipe downloads the toolchain. That is our choice in our own channel and likely not acceptable in RoboStack's or conda-forge's *(not verified)*. Rebuild when RoboStack rebuilds Jazzy. `linux-64` never built. Prerequisite 4 | medium |
| (e) ROS build farm (bloom release into `rosdistro`) | `sudo apt install ros-jazzy-roslean-collision-monitor`, no extra source | A release repository, a `rosdistro` entry, a release per change | The farm builds from declared dependencies, and Lean is not one. Whether a build may download a compiler there is *not verified*: the documentation found does not say, and reports from users disagree. In practice it waits for (f) or for Lean as a system package. Ask on the ROS infrastructure forum before any work | large |
| (f) Generated C and Lean's runtime in the repository; plain colcon build without Lean | `colcon build` in a workspace, no elan | The nine generated C files committed and checked in CI against a fresh `lake build`; `lean.h` and its companions; the runtime (`libleanrt`), Lean's standard library (`Init`, which the generated code calls) and GMP, libuv from source or from the system | The toolchain ships headers and binaries, not the runtime's C++ sources: they come from the Lean repository, with its build system. Whether the generated C is the same on every platform is *not verified*. Every Lean update repeats the work. Would also remove the 190 MB | large |

Rules for all of them:

- The overlay packages are named `twist_mux` and `nav2_collision_monitor`. They are never published under those names in a channel or repository that others share, and never into `/opt/ros/jazzy`. In (a) and (b) they can sit in the workspace under `/opt/roslean`, off by default. In (c) and (d) they are packages with names of our own and a prefix of their own, or left out.
- An artefact is published only from a commit at which CI checked the proofs (`lake build ROSlean`) and ran the tests on that architecture.
- The monitor is built against one Nav2 patch release. The artefact names it; a newer one is a new build and a new test run.

## Recommended order

1. **Prerequisites 1, 2, 3 and 5.** Nothing below is honest without an x86-64 test run, and the licence file should exist before binaries that contain Lean's runtime are handed out.
2. **(a) and (b) from one job.** The Dockerfile already produces a tested tree with no Lean in it; publishing the image and the archive is a workflow, not new packaging. Unlocks: trying the nodes in minutes; a robot with Ubuntu 24.04 and apt's Jazzy gets the nodes without a build. Add the overlay and `roslean_nav2_bringup` to the image here.
3. **Prerequisite 4, then (d).** The repository already builds with pixi (`pixi.toml` names three platforms; macOS arm64 and Linux arm64 were built), and this is the only route that serves macOS. Unlocks: `pixi add` in a user's project; a shorter quick start.
4. **(c)**, if users on apt ask for more than the archive: dependencies resolved by apt, upgrades, removal.
5. **(f) as an experiment, then (e).** (f) is the only way to the build farm and to distributions' own packages, and the only one that removes `libleanshared.so`. It is also the most work and the most likely to fail.

## Done when

- Tagging a release produces, without manual steps: a multi-arch image on GHCR and one archive per Linux architecture on the release page, each built and tested in CI on amd64 and arm64.
- On a machine with ROS 2 Jazzy and no Lean, the documented one command followed by the bring-up launch starts Nav2 with the Lean monitor; the installed scenario tests pass there.
- `README.md` and the two drop-in documents give that command first and the source build second.
- For (c) to (f): each has its own issue with a decision, or is closed with the reason.

## Out of scope

- ROS distributions other than Jazzy; Windows.
- Making the runtime smaller, except as part of (f).
- Publishing the overlays under upstream's names anywhere.
- Signing, SBOMs and reproducible builds beyond recording the commit and versions.
- Hardware and timing runs (`linux-hardware-run.md`).
