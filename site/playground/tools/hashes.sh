# Sourced by build.sh and check_fresh.sh: the hashes recorded in
# ../roslean_core.manifest. Expects ROOT (the repository root).

sha() {
  if command -v sha256sum >/dev/null 2>&1; then sha256sum "$@"; else shasum -a 256 "$@"; fi
}

# One hash over a list of files (paths relative to ROOT, read from stdin):
# the hash of the sorted list of "hash  path" lines.
hash_list() {
  (cd "$ROOT" && LC_ALL=C sort | while IFS= read -r f; do sha "$f"; done) | sha | cut -d' ' -f1
}

# The Lean sources of the core and what fixes how they are compiled.
hash_lean_sources() {
  (cd "$ROOT" && ls core/lean-toolchain core/lakefile.toml core/lake-manifest.json \
      core/ROSleanCore.lean core/ROSleanCore/*.lean) | hash_list
}

# What is compiled and linked besides Lean's output and Lean's runtime, and
# the scripts that do it.
hash_glue() {
  (cd "$ROOT" && ls ros/roslean_core/src/runtime.c ros/roslean_core/include/roslean_core/ffi.h \
      site/playground/tools/build.sh site/playground/tools/cc1.sh \
      site/playground/tools/regen1.sh site/playground/tools/hashes.sh \
      site/playground/tools/wasm_shim/uv_stubs.c \
      site/playground/tools/wasm_shim/linux/fs.h) | hash_list
}

# The C that Lean generated for the core (after `lake build`).
hash_generated_c() {
  (cd "$ROOT" && find core/.lake/build/ir -name '*.c') | hash_list
}

manifest_get() {
  sed -n "s/^$1=//p" "$ROOT/site/playground/roslean_core.manifest"
}
