# Playground

`index.html` is a page of the project site: a small simulated robot in the
browser, with the decisions of `twist_mux` and of the collision monitor made
by the Lean code of `core/`, compiled to WebAssembly.

Pages publishes `site/` as it is, with no build step, so everything the page
loads is committed here.

| File | What it is |
| --- | --- |
| `roslean_core.wasm`, `roslean_core.js` | The Lean core and Lean's runtime, built by `tools/build.sh`. Generated; do not edit |
| `roslean_core.manifest` | What the two files were built from, as hashes |
| `shim.js` | The counterpart of the C++ wrappers and of the parts of the nodes around the core: configure, hand over events, carry out the answers. Computes the scan's points and the slowdown multiplication in floating point. Decides nothing |
| `sim.js` | The toy world: robot, lidar, boxes, planner, joystick, base watchdog, and the two wirings. No DOM |
| `playground.js`, `playground.css`, `index.html` | The page |
| `tools/` | Build, staleness check, equivalence check, tests |

## What is the Lean code and what is not

Lean, through the C interface of `ros/roslean_core/include/roslean_core/ffi.h`
(the same exported functions the ROS nodes call):

- the mux: `roslean_mux_builder_*`, `roslean_mux_new`, `roslean_mux_on_input`,
  `roslean_mux_on_lock`, `roslean_mux_forward`;
- the monitor: `roslean_cm_builder_*`, `roslean_cm_new`, `roslean_cm_ok`,
  `roslean_cm_on_lifecycle`, `roslean_cm_on_scan`, `roslean_cm_cmd_plan`,
  `roslean_cm_on_cmd` and the five `roslean_cm_out_*` queries.

Both are configured with the values of the Gazebo demo
(`ros/roslean_demos/gazebo/mux/gazebo.yaml`, `gazebo/monitor/monitor.yaml`),
written out again in `sim.js` (`PARAMS`); the page does not read the YAML
files.

Not Lean, and not verified: `shim.js` (which plays the part of the C++ shim:
the points of a scan in the base frame, the three multiplications of a
slowdown, which answer means "publish nothing"), the whole of `sim.js`, and
the page. `cos` and `sin` are JavaScript's, not the C library's.

The theorems of the project are about the Lean functions. Nothing is proved
about this page, about the WebAssembly build, or about Emscripten.

## Rebuilding

Needed: elan (the toolchain of `core/lean-toolchain`), an activated
[Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html),
git, and a scratch directory outside the repository. The script clones
Lean's sources at the toolchain's commit and libuv at the tag Lean's build
names into the scratch directory unless `LEAN4_SRC` and `LIBUV_SRC` point to
checkouts.

```sh
source <emsdk>/emsdk_env.sh
WORK=/path/to/scratch site/playground/tools/build.sh      # about half a minute after the clones
site/playground/tools/check_fresh.sh --lake               # the artefacts belong to this tree
WORK=/path/to/scratch site/playground/tools/equiv/run.sh  # native and wasm give the same answers
node site/playground/tools/test/run_node.mjs              # the scripted scenario on the page's logic
```

Rebuild whenever `core/`, `ros/roslean_core/src/runtime.c`, `ffi.h` or the
build scripts change. `tools/check_fresh.sh` fails if the committed files
are stale: without arguments it compares the hashes of the Lean sources, of
the glue and of the two artefacts with the manifest and needs no toolchain;
with `--lake` it also builds `core/` and compares the generated C. It is not
wired into CI.

Built here with Lean 4.37.0-rc1, Emscripten 7.0.1 and libuv v1.53.0 on macOS
arm64. Two builds on that machine gave byte-identical files. Not tried: any
other host, any other Emscripten version.

### How the build is put together

Lean compiles to C. `lake build` writes the C of the core to
`core/.lake/build/ir/`; the ROS package compiles those files and links them
with the runtime and standard library that the toolchain ships for the host.
For the browser, runtime and standard library have to be compiled as well:

- **No ready-made WebAssembly build.** The release of Lean 4.37.0-rc1 has
  assets for Linux, macOS and Windows only, and the toolchain directory has
  no WebAssembly libraries.
- **`Init` (Lean's standard library).** The toolchain ships its sources.
  `build.sh` compiles all 678 modules to C with the toolchain's own `lean -c`
  and the options of Lean's build (17 s). The result is byte for byte the C
  in `stage0/stdlib/Init` of Lean's repository at that commit; the script
  counts the identical files and records the number in the manifest. All of
  `Init` is compiled because the core's initialiser calls `initialize_Init`;
  the linker drops what is unreachable.
- **The runtime** (`src/runtime/*.cpp` of Lean's repository) is compiled with
  `-DLEAN_EMSCRIPTEN`, without `LEAN_MULTI_THREAD` (a page on GitHub Pages
  cannot use shared memory), without `LEAN_USE_GMP` (Lean's own big-number
  code is used) and without mimalloc (the C library's `malloc`).
- **Three accommodations, none of them a change to Lean's sources.** The
  build without threads does not compile as it stands at this version:
  `runtime/thread.h` uses `std::adopt_lock_t` without its header
  (`-include mutex`); `runtime/io.cpp` includes `<linux/fs.h>` and makes a
  Linux system call for one file operation (`tools/wasm_shim/linux/fs.h`
  makes that call fail; nothing here calls it); and the linked code reaches
  one function of libuv, `uv_strerror`, so libuv's `src/uv-common.c` is
  compiled, with empty stand-ins for three clean-up hooks
  (`tools/wasm_shim/uv_stubs.c`).
- **Link.** Exported: every function named in `ffi.h`, `malloc`, `free`.
  8 MB of stack (Lean recurses over lists), growable memory, WebAssembly
  exceptions (the legacy form, Emscripten's default).

Size: `roslean_core.wasm` 1,443,271 bytes (250,232 with `gzip -9`, 181,457
with brotli 11); `roslean_core.js` 59,973 bytes (16,780 gzip). Whether Pages
compresses `.wasm` was not checked.

## The equivalence check

`tools/equiv/run.sh` generates seeded random calls of the C interface
(`gen_cases.mjs`), makes them on the native build (`native_driver.c`, linked
against the toolchain's own runtime with GMP and mimalloc) and on the
WebAssembly build under node (`run_wasm.mjs`), and compares the printed
answers byte for byte.

- Mux: random handles (priorities also negative and above 255; timeouts
  zero, negative, positive), input and lock events with indices also out of
  range, clock readings that also go backwards and lie around 2³¹, 2³², 2⁵³,
  2⁶³ and 2⁶⁴.
- Monitor: one scan source, one to three zones (polygons as in the demo,
  also circles and topic zones), `min_points`, ratio, both timeouts, with
  ordinary values in half of the configurations and values the core must
  refuse in the rest; then lifecycle transitions, scans, shape messages and
  commands. Points are the page shim's or arbitrary doubles, including NaN,
  infinities and values within a fraction of a millimetre of a zone's edge.
  For small scans the reference definition (`roslean_cm_on_cmd_reference`)
  is run as well.

Result on macOS arm64, seeds 1 to 4, 1,500 mux and 1,500 monitor
configurations per seed: 617,465 calls, 521,014 answers, no difference.
Among the answers: 268,819 mux decisions; 6,000 configurations; 185,429
answers to scans and commands (50,639 pass, 10,868 slowdown, 16,623 stop by
a zone, 30,900 stop for an invalid source, 9,471 withheld, 66,928 silent).
`run.sh` prints these counts.

What differs between the two builds, and was therefore worth checking:

- **Small integers.** Lean keeps a small `Nat` or `Int` inside the pointer:
  63 bits natively, 31 bits on 32-bit WebAssembly. Clock readings in
  nanoseconds and products of millimetres are above 2³¹, so in the browser
  they are heap-allocated big numbers and go through Lean's built-in
  big-number code, where the native build computes in a machine word (and
  uses GMP above 2⁶³). Same arithmetic, different code.
- **`size_t` and pointers** are 32 bits; the 64-bit arguments of the
  interface cross as JavaScript `BigInt`.
- **Allocator and threads.** `malloc` instead of mimalloc; one thread.

The check compares the two builds of the Lean code. It does not check the
JavaScript shim against the C++ shim; the two compute a scan's points with
different `cos` and `sin`.

## Tests of the page

- `tools/test/run_node.mjs`: the scripted scenario (`scenario.mjs`: free
  patrol; a box ahead gives slowdown, stop, silent; trapped or not under each
  wiring; box removed; lock engaged and released; joystick priority) on
  `sim.js`, `shim.js` and the `.wasm` under node. 27 checks.
- `tools/test/chrome.mjs <base URL> <output dir>`: the same scenario inside
  the page in headless Chrome, console errors, each state under both
  wirings with screenshots, arrow keys with and without focus, 360 px
  width, requests and bytes. Serve `site/` first
  (`python3 -m http.server` in `site/`); the page does not load from
  `file://`.

Tried in headless Chrome 158 on macOS only. Firefox and Safari were not
tried. The page needs WebAssembly with exception handling and `BigInt`.

## Third-party code in the committed files

See `THIRD_PARTY.md`.
