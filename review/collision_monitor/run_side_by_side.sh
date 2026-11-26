#!/usr/bin/env bash
# Runs a review script that needs ROS (side_by_side.py, param_probe.py)
# against the binaries of a pixi (RoboStack) environment, without `pixi run`.
#
#   review/collision_monitor/run_side_by_side.sh <env prefix> <script> <lean node> [args ...]
#
# <env prefix> is e.g. <checkout>/.pixi/envs/default; <lean node> is
# <checkout>/install/roslean_collision_monitor/lib/roslean_collision_monitor/collision_monitor.
set -eo pipefail
prefix="$1"; script="$2"; shift 2
here="$(cd "$(dirname "$0")" && pwd)"
export CONDA_PREFIX="$prefix"
export PATH="$prefix/bin:$PATH"
# shellcheck disable=SC1091
source "$prefix/setup.bash"
export AMENT_PREFIX_PATH="$prefix"
export CM_PREFIX="$prefix"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-92}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="${TMPDIR:-/tmp}/cm_review_ros_log"
exec python3 "$here/$script" "$@"
