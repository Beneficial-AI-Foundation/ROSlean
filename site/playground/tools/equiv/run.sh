#!/bin/sh
# The equivalence check: the same calls on the native build of the Lean core
# and on site/playground/roslean_core.wasm, outputs compared byte for byte.
#
#   WORK=<scratch dir> run.sh [seeds, default "1 2 3 4"] 
#
# Needs: lake and lean (elan), node 20 or later, a C compiler (CC, default
# cc). Environment: MUX (mux configurations per seed, default
# 1500), CM (monitor configurations per seed, default 1500).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../../../.." && pwd)
: "${WORK:?set WORK to a scratch directory outside the repository}"
SEEDS=${*:-1 2 3 4}
MUX=${MUX:-1500}
CM=${CM:-1500}
mkdir -p "$WORK/equiv"
W=$(cd "$WORK/equiv" && pwd)

"$HERE/../check_fresh.sh" --lake

# The native side: the C that Lean generated, runtime.c and the driver,
# linked statically against the toolchain's own Init and runtime (with GMP
# and mimalloc), the libraries the ROS package links on macOS
# (ros/roslean_core/CMakeLists.txt). The Linux line is untested.
IR="$ROOT/core/.lake/build/ir"
SYSROOT=$(cd "$ROOT/core" && lean --print-prefix)
if [ "$(uname)" = Darwin ]; then LIBS="-lc++ -Wl,-dead_strip"; else LIBS="-lstdc++ -lm -lpthread -ldl -Wl,--gc-sections"; fi
# shellcheck disable=SC2046,SC2086
${CC:-cc} -O3 -DNDEBUG -ffp-contract=off -fdata-sections -ffunction-sections -w \
  -I"$SYSROOT/include" -I"$ROOT/ros/roslean_core/include" \
  $(find "$IR" -name '*.c') "$ROOT/ros/roslean_core/src/runtime.c" "$HERE/native_driver.c" \
  -L"$SYSROOT/lib/lean" -L"$SYSROOT/lib" -lInit -lleanrt -lcadical -lgmp -luv $LIBS \
  -o "$W/native_driver"

fail=0
for seed in $SEEDS; do
  node "$HERE/gen_cases.mjs" "$seed" "$MUX" "$CM" > "$W/cases.$seed.txt"
  "$W/native_driver" < "$W/cases.$seed.txt" > "$W/native.$seed.txt"
  node "$HERE/run_wasm.mjs" < "$W/cases.$seed.txt" > "$W/wasm.$seed.txt"
  calls=$(wc -l < "$W/cases.$seed.txt" | tr -d ' ')
  answers=$(wc -l < "$W/native.$seed.txt" | tr -d ' ')
  if cmp -s "$W/native.$seed.txt" "$W/wasm.$seed.txt"; then
    echo "seed $seed: $calls calls, $answers answers, identical"
    awk '{ k = $1; if (k == "out") k = "out kind " $2; n[k]++ } END { for (k in n) printf "    %-12s %d\n", k, n[k] }' \
      "$W/native.$seed.txt" | LC_ALL=C sort
  else
    echo "seed $seed: $calls calls: OUTPUTS DIFFER (first lines of the diff below)"
    diff "$W/native.$seed.txt" "$W/wasm.$seed.txt" | head -20
    fail=1
  fi
done
exit $fail
