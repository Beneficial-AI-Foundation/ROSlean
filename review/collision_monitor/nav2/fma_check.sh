#!/usr/bin/env bash
# Does upstream's compiled Scan::getData fuse the multiplications and additions
# of the point transform (tf2::Transform * tf2::Vector3)? macOS only (otool).
# Prints the floating-point multiply and convert instructions of the function.
#
#   review/collision_monitor/nav2/fma_check.sh <env prefix>
set -eo pipefail
lib="$1/lib/libcollision_monitor_core.dylib"
if ! command -v otool >/dev/null; then
  echo "otool not found: this check is for macOS; on Linux use objdump -d on libcollision_monitor_core.so" >&2
  exit 2
fi
echo "# $(basename "$lib"), nav2_collision_monitor::Scan::getData:"
otool -tv "$lib" |
  awk '/^__ZN22nav2_collision_monitor4Scan7getData/{p=1} p&&/^__Z/&&!/Scan7getData/{p=0} p' |
  grep -E 'fmadd|fmsub|fnmadd|fnmsub|fmul|fcvt|vfmadd|mulsd|mulss' |
  awk '{print "  " $2, $3, $4, $5, $6}'
