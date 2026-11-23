#!/usr/bin/env bash
# Records one take into DIR. Run inside the pixi environment:
#   pixi run bash video/record.sh DIR [WIDTH HEIGHT RTF]
# Needs an X display with OpenGL 3.3; without one it starts Xvfb and uses
# Mesa's software renderer.
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(realpath -m "${1:?output directory}")
mkdir -p "$out"
source "$here/../install/setup.bash"
python3 "$here/make_world.py" "$here/../ros/roslean_twist_mux/demo/gazebo/apartment.sdf" \
  "$out/world.sdf" "${2:-3840}" "${3:-2160}" "${4:-0.05}"
if [ -z "$DISPLAY" ]; then
  Xvfb :77 -screen 0 1280x720x24 +extension GLX >/dev/null 2>&1 &
  trap "kill $!" EXIT
  export DISPLAY=:77 LIBGL_ALWAYS_SOFTWARE=1 __GLX_VENDOR_LIBRARY_NAME=mesa
  # A conda environment has libGL but no Mesa driver; borrow the system's.
  if [ -n "$CONDA_PREFIX" ] && [ -e /usr/lib/x86_64-linux-gnu/libGLX_mesa.so.0 ]; then
    mkdir -p "$out/gl" && ln -sf /usr/lib/x86_64-linux-gnu/libGLX_mesa.so.0 "$out/gl/"
    export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$out/gl"
  fi
fi
ros2 launch "$here/record_launch.py" world:="$out/world.sdf" out:="$out"
pkill -f "gz sim" || true
