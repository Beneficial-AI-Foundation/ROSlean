#!/bin/bash
# Fails if the install tree given as $1 still depends on the Lean toolchain or
# on the source tree. Run with the ROS environment sourced.
set -u
prefix=$1
nodes="$prefix/roslean_twist_mux/lib/roslean_twist_mux/twist_mux
$prefix/roslean_collision_monitor/lib/roslean_collision_monitor/collision_monitor"
status=0

refs=$(grep -rIl -e '\.elan' -e '/src/' "$prefix")
if [ -n "$refs" ]; then
  echo "FAIL: files that mention the toolchain or the source tree:"; echo "$refs"; status=1
fi
refs=$(grep -rl -e '\.elan' "$prefix" --include='*.so' --include=twist_mux --include=noop_node \
  --include=collision_monitor)
if [ -n "$refs" ]; then
  echo "FAIL: binaries that mention the toolchain:"; echo "$refs"; status=1
fi

for node in $nodes; do
  echo "$node"
  deps=$(ldd "$node")
  echo "$deps" | grep -e roslean -e lean -e 'c++' -e 'not found'
  if ! echo "$deps" | grep -q "libroslean_core.so => $prefix/"; then
    echo "FAIL: the node does not load libroslean_core from the install tree"; status=1
  fi
  if ! echo "$deps" | grep -q "libleanshared.so => $prefix/"; then
    echo "FAIL: the node does not load libleanshared from the install tree"; status=1
  fi
  if echo "$deps" | grep -q -e elan -e 'not found'; then
    echo "FAIL: unresolved or toolchain libraries"; status=1
  fi
done
echo "libroslean_core.so needs:"; ldd "$prefix/roslean_core/lib/libroslean_core.so"
echo "exported symbols: $(nm -D --defined-only "$prefix/roslean_core/lib/libroslean_core.so" | wc -l)"
ls -la "$prefix/roslean_core/lib/"
exit $status
