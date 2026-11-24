#!/bin/bash
# Runs the collision-monitor comparison in the Linux image of this directory
# and writes the logs to a directory on the host.
#
#   docker build -f bench/cm_linux/Dockerfile -t roslean-cm-linux .
#   bench/cm_linux/run.sh OUT_DIR STEP [STEP ...]
#
# Steps:
#   versions   kernel, compiler, package versions
#   scenarios  the fixed scenarios: the Lean node with its profile, upstream,
#              the probes on upstream, the reference node (15 minutes)
#   selftest   the two self-tests of the random comparison
#   random     600 random cases, the Lean node beside upstream, in 6
#              processes at once (seeds 0-599), then 120 with the scan frame
#              turned (seeds 1000-1119) in 2 processes
#   points     band_bounds 100000 7
#   faults     fault injection, both nodes, the very large scans included
#   qos        probe_qos.py on both nodes
#   bench      per-call latency in process (25 minutes) and latency through
#              the nodes
#   node       latency through the nodes only: twice for 20 s per scan size
#              (the second run's file goes to OUT_DIR/second) and once for
#              120 s (OUT_DIR/long)
#   soak       both nodes under load for 32 minutes, memory sampled
#   ctest      the registered tests of roslean_collision_monitor (colcon test)
#   nav2       the tests of the Nav2 extension on upstream's node (tasks/T07):
#              scenario_nav2.py, random_nav2.py seeds 0-19, bond_manager.py.
#              The test directory of this checkout is mounted over the
#              image's copy, so the image need not be rebuilt for a changed
#              test. The image has neither nav2_bringup nor
#              nav2_lifecycle_manager: the five scenarios that edit
#              nav2_params.yaml and the manager part report "not run".
#
# Every step is one container (`docker run --rm`), so nothing is left over.
# The containers have their own network: the ROS_DOMAIN_IDs below separate
# the processes of one step from each other, not from the host.
set -euo pipefail

OUT=$(mkdir -p "$1" && cd "$1" && pwd)
shift
IMAGE=${IMAGE:-roslean-cm-linux}
T=ros/roslean_collision_monitor/test
NODE=roslean_collision_monitor/collision_monitor
UP=nav2_collision_monitor/collision_monitor

in_container() {  # NAME COMMAND: runs COMMAND in a fresh container, log to OUT/NAME.log
  local name=$1
  shift
  echo "== $name"
  docker run --rm -e PYTHONUNBUFFERED=1 -v "$OUT":/out "$IMAGE" bash -c "$*" \
    > "$OUT/$name.log" 2>&1 && echo "   exit 0" || echo "   exit $? (see $OUT/$name.log)"
}

for step in "$@"; do
  case $step in
    versions)
      in_container versions "uname -srm; . /etc/os-release; echo \$PRETTY_NAME; nproc; \
        grep MemTotal /proc/meminfo; gcc --version | head -1; \
        dpkg-query -W ros-jazzy-nav2-collision-monitor ros-jazzy-rclcpp ros-jazzy-rmw-fastrtps-cpp \
          ros-jazzy-fastrtps; (cd core && lean --version); \
        echo RMW=\${RMW_IMPLEMENTATION:-default}; \
        grep -E 'CMAKE_BUILD_TYPE' build/roslean_collision_monitor/CMakeCache.txt; \
        nm -D -u install/roslean_collision_monitor/lib/roslean_collision_monitor/collision_monitor \
          | grep -E 'sincosf|sinf|cosf'; \
        nm -D -u /opt/ros/jazzy/lib/libcollision_monitor_core.so | grep -E 'sincosf|sinf|cosf'"
      ;;
    scenarios)
      in_container scenario_roslean \
        "time ROS_DOMAIN_ID=31 python3 $T/scenario.py $NODE --profile profile_roslean" &
      in_container scenario_upstream "time ROS_DOMAIN_ID=32 python3 $T/scenario.py $UP" &
      in_container scenario_upstream_probes \
        "time ROS_DOMAIN_ID=33 python3 $T/scenario.py $UP --profile profile_upstream --own" &
      in_container scenario_reference \
        "time ROS_DOMAIN_ID=34 python3 $T/scenario.py $T/reference_node.py --no-tf --skip negative-stamp" &
      wait
      ;;
    selftest)
      in_container selftest_swap \
        "ROS_DOMAIN_ID=35 python3 $T/random_diff.py $NODE $UP 0 3 --roslean --self-test swap" &
      in_container selftest_shrink \
        "ROS_DOMAIN_ID=36 python3 $T/random_diff.py $NODE $UP 0 3 --roslean --self-test shrink" &
      wait
      ;;
    random)
      in_container random "
        for k in 0 1 2 3 4 5; do
          ROS_DOMAIN_ID=\$((40 + k)) python3 $T/random_diff.py $NODE $UP \$((k * 100)) \$((k * 100 + 100)) \
            --roslean --stats /out/random_\$k.json > /out/random_\$k.log 2>&1 &
        done
        for k in 0 1; do
          ROS_DOMAIN_ID=\$((50 + k)) python3 $T/random_diff.py $NODE $UP \$((1000 + k * 60)) \$((1060 + k * 60)) \
            --roslean --yaw --stats /out/yaw_\$k.json > /out/yaw_\$k.log 2>&1 &
        done
        wait
        tail -n 4 /out/random_?.log /out/yaw_?.log
        echo; echo 'seeds 0-599:'; python3 $T/diff_stats.py /out/random_?.json
        echo; echo 'seeds 1000-1119, --yaw:'; python3 $T/diff_stats.py /out/yaw_?.json"
      ;;
    points)
      in_container band_bounds "time build/roslean_collision_monitor/band_bounds 100000 7"
      ;;
    faults)
      in_container fault_injection \
        "time ROS_DOMAIN_ID=37 python3 $T/fault_injection.py $NODE $UP --big"
      ;;
    qos)
      in_container probe_qos "ROS_DOMAIN_ID=38 python3 $T/probe_qos.py $NODE --no-state-topic; \
        ROS_DOMAIN_ID=38 python3 $T/probe_qos.py $UP"
      ;;
    bench)
      in_container bench_core \
        "python3 bench/roslean_bench/scripts/run_cm_bench.py core --out /out"
      in_container bench_node \
        "ROS_DOMAIN_ID=39 python3 bench/roslean_bench/scripts/run_cm_bench.py node --out /out"
      ;;
    node)
      in_container bench_node \
        "ROS_DOMAIN_ID=39 python3 bench/roslean_bench/scripts/run_cm_bench.py node --out /out"
      in_container bench_node_2 \
        "ROS_DOMAIN_ID=39 python3 bench/roslean_bench/scripts/run_cm_bench.py node --out /out/second"
      in_container bench_node_long \
        "ROS_DOMAIN_ID=39 python3 bench/roslean_bench/scripts/run_cm_bench.py node --seconds 120 --out /out/long"
      ;;
    soak)
      in_container bench_soak \
        "ROS_DOMAIN_ID=39 python3 bench/roslean_bench/scripts/run_cm_bench.py soak --minutes 32 --out /out; \
         install/roslean_bench/lib/roslean_bench/bench_cm --which lean --scan slow \
           --soak 20000 --iters 200000 --warmup 0 | tee /out/cm_soak_core.json"
      ;;
    ctest)
      in_container ctest "time colcon test --base-paths ros --packages-select roslean_collision_monitor \
        --event-handlers console_direct+; colcon test-result --verbose"
      ;;
    nav2)
      HERE=$(cd "$(dirname "$0")/../.." && pwd)
      for run in "scenario_nav2:61:scenario_nav2.py" "random_nav2:62:random_nav2.py 0 20" \
                 "bond_manager:63:bond_manager.py"; do
        IFS=: read -r name domain script <<< "$run"
        echo "== $name"
        docker run --rm -e PYTHONUNBUFFERED=1 -v "$HERE/$T":/src/$T:ro "$IMAGE" \
          bash -c "time ROS_DOMAIN_ID=$domain python3 $T/$script" > "$OUT/$name.log" 2>&1 &
      done
      wait
      tail -n 2 "$OUT/scenario_nav2.log" "$OUT/random_nav2.log" "$OUT/bond_manager.log"
      ;;
    *)
      echo "unknown step: $step" >&2
      exit 2
      ;;
  esac
done
