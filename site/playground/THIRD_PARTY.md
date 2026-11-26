# Third-party code in `roslean_core.wasm` and `roslean_core.js`

The two files are compiled output. They contain, besides this project's own
code (Apache-2.0, see `LICENSE` at the repository root):

| Component | What of it is in the files | Licence |
| --- | --- | --- |
| [Lean 4](https://github.com/leanprover/lean4), version 4.37.0-rc1 | The runtime (`src/runtime`) and the standard library `Init`, compiled from source | Apache-2.0. Copyright Microsoft Corporation, Lean FRO and the other holders named in the source files |
| [libuv](https://github.com/libuv/libuv), v1.53.0 | One source file, `src/uv-common.c` (of which the linked code reaches `uv_strerror`) | MIT. Copyright Joyent, Inc. and other Node contributors; copyright libuv project contributors |
| [Emscripten](https://github.com/emscripten-core/emscripten), 7.0.1 | The JavaScript loader (`roslean_core.js`) and the parts of its system libraries the linker kept | MIT or University of Illinois/NCSA, at choice |
| musl libc (through Emscripten) | C library functions | MIT |
| LLVM libc++, libc++abi, compiler-rt, libunwind (through Emscripten) | C++ runtime support | Apache-2.0 with LLVM exceptions |
| dlmalloc (through Emscripten) | `malloc` | Public domain (CC0) |

The full licence texts are in the respective repositories: `LICENSE` and
`LICENSES` of Lean, `LICENSE` of libuv, `LICENSE` of Emscripten and the
licence files under its `system/lib/`.

Not in the files, although the native builds of the ROS package link them:
GMP (LGPL; here Lean's own big-number code is used instead), mimalloc, and
the code of CaDiCaL and OpenSSL. Which parts of the system libraries the
linker kept was not listed function by function; the table names every
library the link draws from.

`tools/wasm_shim/` is this project's own.
