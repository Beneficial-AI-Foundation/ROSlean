#!/usr/bin/env bash
# Compiles TransformCost.lean to a native program with the toolchain of core/
# and runs it. Build products go to a temporary directory.
#
#   review/collision_monitor/nav2/run_transform_cost.sh [repetitions]
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$(mktemp -d "${TMPDIR:-/tmp}/cm_transform_cost.XXXXXX")"
cp "$here/TransformCost.lean" "$here/../../../core/lean-toolchain" "$out/"
cd "$out"
lean -c transform_cost.c TransformCost.lean
leanc -O3 "$out/transform_cost.c" -o "$out/transform_cost"
"$out/transform_cost" "${1:-2000}"
