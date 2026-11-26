#!/usr/bin/env bash
# Records one take of the collision monitor demo into DIR. Run inside the pixi
# environment, with a display that can render (Gazebo renders the lidar and
# the camera; on macOS the screen must be unlocked):
#   pixi run bash video/cm/record.sh DIR [WIDTH HEIGHT RTF]
# WIDTH = 0 records the messages only, without the camera.
# The simulator gets a partition of its own, and ROS the domain ROS_DOMAIN_ID
# (default 84), so that a demo running on the same machine is not disturbed.
set -e
here=$(cd "$(dirname "$0")" && pwd)
mkdir -p "${1:?output directory}"
out=$(cd "$1" && pwd)
width=${2:-3840} height=${3:-2160} rtf=${4:-0.1}
source "$here/../../install/setup.bash"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-84} GZ_PARTITION=${GZ_PARTITION:-roslean_cm_video_$$}
python3 "$here/make_world.py" "$here/../../ros/roslean_demos/gazebo/mux/apartment.sdf" \
  "$out/world.sdf" "$width" "$height" "$rtf"
camera=true; [ "$width" = 0 ] && camera=false
ros2 launch "$here/record_launch.py" world:="$out/world.sdf" out:="$out" camera:=$camera
pkill -f "gz sim.*$out/world.sdf" || true
