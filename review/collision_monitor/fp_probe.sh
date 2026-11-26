#!/usr/bin/env bash
# CM3/CM4 review: compiles fp_probe.cpp with and without fused multiply-add,
# for arm64 and (on a Mac with Rosetta) for x86-64, and compares what the
# shim's floating-point code produces.
#
#   review/collision_monitor/fp_probe.sh <output directory>
set -eo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
out="$1"
mkdir -p "$out"
inc="$here/../../ros/roslean_collision_monitor/include"
cxx="${CXX:-/usr/bin/clang++}"
build() {  # name, flags...
  local name="$1"; shift
  if "$cxx" -std=c++17 -I"$inc" "$@" "$here/fp_probe.cpp" -o "$out/$name" 2>"$out/$name.err" \
      && "$out/$name" "$out/$name.bin"; then
    echo "built and ran: $name ($*)"
  else
    echo "could not build or run: $name ($*)"
  fi
}
build arm64_O0_off   -arch arm64 -O0 -ffp-contract=off
build arm64_O2_off   -arch arm64 -O2 -ffp-contract=off
build arm64_O2_on    -arch arm64 -O2 -ffp-contract=on
build arm64_O2_fast  -arch arm64 -O2 -ffp-contract=fast
build arm64_O2_default -arch arm64 -O2
build x86_O2_off     -arch x86_64 -O2 -ffp-contract=off
build x86_O2_fma     -arch x86_64 -O2 -mfma -ffp-contract=fast
build arm64_sanitized -arch arm64 -O1 -g -fsanitize=undefined,address -fno-sanitize-recover=undefined -ffp-contract=off
python3 - "$out" <<'EOF'
import math, os, struct, sys
out = sys.argv[1]
ref = open(os.path.join(out, "arm64_O0_off.bin"), "rb").read()
n1 = 200 * 500 * 20
def coords(b):
    return [struct.unpack_from("<dd", b, o + 4) for o in range(0, len(b), 20)]
def mm(v):
    return math.trunc(v * 1000)  # reporting only: approximate, not Lean's exact rule
for name in sorted(os.listdir(out)):
    if not name.endswith(".bin") or name in ("arm64_O0_off.bin", "arm64_sanitized.bin"):
        continue
    b = open(os.path.join(out, name), "rb").read()
    res = []
    for label, lo, hi in (("no pose", 0, n1), ("with pose", n1, 2 * n1)):
        a, c = coords(ref[lo:hi]), coords(b[lo:hi])
        diff = sum(x != y for p, q in zip(a, c) for x, y in zip(p, q))
        dmm = sum(mm(x) != mm(y) for p, q in zip(a, c) for x, y in zip(p, q))
        worst = max((abs(x - y) for p, q in zip(a, c) for x, y in zip(p, q)), default=0.0)
        res.append(f"{label}: {diff} of {2 * len(a)} coordinates differ "
                   f"(largest {worst:.3g} m; {dmm} differ in whole millimetres)")
    prod = sum(ref[2 * n1 + 8 * k: 2 * n1 + 8 * k + 8] != b[2 * n1 + 8 * k: 2 * n1 + 8 * k + 8]
               for k in range(100000))
    print(f"{name[:-4]} against arm64_O0_off: " + "; ".join(res) + f"; {prod} of 100000 products differ")
EOF
