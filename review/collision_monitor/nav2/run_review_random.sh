#!/usr/bin/env bash
# Review CM5 (task T08): the two random comparisons of the ROSlean node
# beside upstream, split over several processes, each on its own
# ROS_DOMAIN_ID and with a time limit of its own.
#
#   review/collision_monitor/nav2/run_review_random.sh diff|nav2 OUT_DIR [PROCESSES] [SEEDS_EACH] [FIRST_DOMAIN] [LIMIT_S]
#
#   diff   random_diff.py NODE UPSTREAM a b --roslean --stats OUT/diff_k.json
#   nav2   random_nav2.py NODE UPSTREAM a b
#
# Run inside the pixi environment, from the repository root:
#   pixi run bash review/collision_monitor/nav2/run_review_random.sh diff /tmp/out 6 50 180
# Each process writes OUT_DIR/<kind>_<k>.log; the script waits for all of
# them and prints the last lines. macOS has no `timeout`, so the limit is a
# watchdog that kills the process group members by PID.
# (no `set -u`: colcon's setup.bash reads unset variables)
KIND=${1:?diff or nav2}
OUT=${2:?output directory}
PROCS=${3:-6}
EACH=${4:-50}
DOMAIN0=${5:-180}
LIMIT=${6:-3000}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
T=$ROOT/ros/roslean_collision_monitor/test
NODE=roslean_collision_monitor/collision_monitor
UP=nav2_collision_monitor/collision_monitor
mkdir -p "$OUT"
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
cd "$T" || exit 2
pids=()
for ((k = 0; k < PROCS; k++)); do
  a=$((k * EACH)); b=$((a + EACH))
  if [ "$KIND" = diff ]; then
    ROS_DOMAIN_ID=$((DOMAIN0 + k)) python3 random_diff.py $NODE $UP $a $b --roslean \
      --stats "$OUT/diff_$k.json" > "$OUT/diff_$k.log" 2>&1 &
  else
    ROS_DOMAIN_ID=$((DOMAIN0 + k)) python3 random_nav2.py $NODE $UP $a $b \
      > "$OUT/nav2_$k.log" 2>&1 &
  fi
  pids+=($!)
done
( sleep "$LIMIT"; for p in "${pids[@]}"; do kill "$p" 2>/dev/null; done ) &
watchdog=$!
status=0
for p in "${pids[@]}"; do wait "$p" || status=1; done
kill "$watchdog" 2>/dev/null
tail -n 3 "$OUT"/${KIND}_*.log
exit $status
