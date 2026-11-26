#!/bin/sh
# One compilation, for xargs: cc1.sh <source> <object>. The compiler and its
# flags come from the environment (CC1, FLAGS), set by build.sh.
mkdir -p "$(dirname "$2")"
# shellcheck disable=SC2086
$CC1 $FLAGS -c "$1" -o "$2" 2>"$2.log" || { echo "FAILED: $1 (see $2.log)"; exit 255; }
