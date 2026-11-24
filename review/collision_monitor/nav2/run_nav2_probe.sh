#!/usr/bin/env bash
# Runs nav2_probe.py against the nav2_collision_monitor binary of a pixi
# (RoboStack) environment, without `pixi run`, so that nothing is written to
# the environment's checkout.
#
#   review/collision_monitor/nav2/run_nav2_probe.sh <env prefix> [case ...]
#
# <env prefix> is e.g. <checkout>/.pixi/envs/default. ROS_DOMAIN_ID defaults
# to 110; set it to run beside something else.
set -eo pipefail
prefix="$1"; shift
here="$(cd "$(dirname "$0")" && pwd)"
export CONDA_PREFIX="$prefix"
export PATH="$prefix/bin:$PATH"
# shellcheck disable=SC1091
source "$prefix/setup.bash"
export AMENT_PREFIX_PATH="$prefix"
export CM_PREFIX="$prefix"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-110}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR="${TMPDIR:-/tmp}/cm_nav2_probe_ros_log"
exec python3 "$here/nav2_probe.py" "$@"
