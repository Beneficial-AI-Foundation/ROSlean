#!/bin/sh
# One module of Lean's standard library to C, for xargs:
# regen1.sh <toolchain prefix> <output dir> <module path without .lean>
# The two options are the ones Lean's own build passes for its library
# (src/CMakeLists.txt, LEAN_EXTRA_OPTS).
mkdir -p "$2/$(dirname "$3")"
"$1/bin/lean" --root="$1/src/lean" -Dinterpreter.prefer_native=false -Dpp.rawOnError=true \
  -c "$2/$3.c" "$1/src/lean/$3.lean" >/dev/null 2>"$2/$3.err" || { echo "FAILED: $3 (see $2/$3.err)"; exit 255; }
