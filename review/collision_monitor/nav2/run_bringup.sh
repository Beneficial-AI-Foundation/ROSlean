#!/usr/bin/env bash
# Review CM5 (task T08), item 5: Nav2's own navigation_launch.py with the
# monitor's package name changed, nav2_bringup's nav2_params.yaml with the
# single documented edit (SPEC_NAV2.md, 8.6: the footprint as a fixed stop
# polygon), headless, on the wall clock.
#
#   review/collision_monitor/nav2/run_bringup.sh OUT_DIR [node|component|upstream] [DOMAIN]
#
#   node       the Node action's package -> roslean_collision_monitor (use_composition:=False)
#   component  the ComposableNode's package and plugin -> the Lean component (use_composition:=True)
#   upstream   nothing changed in the launch file (the same parameters), for comparison
#
# Run inside the pixi environment from the repository root. Prints what
# bringup_check.py saw; the launch log is OUT_DIR/launch_<mode>.log.
MODE=${2:-node}
OUT=${1:?output directory}
DOMAIN=${3:-195}
ROOT=$(cd "$(dirname "$0")/../../.." && pwd)
mkdir -p "$OUT"
# shellcheck disable=SC1091
source "$ROOT/install/setup.bash"
SHARE=$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup
LAUNCH=$OUT/navigation_launch_$MODE.py
PARAMS=$OUT/nav2_params_edit.yaml

python3 - "$SHARE" "$LAUNCH" "$PARAMS" "$MODE" <<'EOF'
import sys
share, launch, params, mode = sys.argv[1:]
text = open(share + '/launch/navigation_launch.py').read()
if mode == 'node':
    old = "package='nav2_collision_monitor',\n                executable='collision_monitor',"
    assert text.count(old) == 1
    text = text.replace(old, old.replace('nav2_collision_monitor', 'roslean_collision_monitor'))
elif mode == 'component':
    old = ("package='nav2_collision_monitor',\n"
           "                        plugin='nav2_collision_monitor::CollisionMonitor',")
    assert text.count(old) == 1
    text = text.replace(old, "package='roslean_collision_monitor',\n"
                        "                        plugin='roslean_collision_monitor::CollisionMonitorNode',")
open(launch, 'w').write(text)
p = open(share + '/params/nav2_params.yaml').read()
old = p[p.index('    polygons: ["FootprintApproach"]'):p.index('    observation_sources: ["scan"]')]
new = '''    polygons: ["FootprintStop"]
    FootprintStop:
      type: "polygon"
      points: "[[0.22, 0.22], [0.22, -0.22], [-0.22, -0.22], [-0.22, 0.22]]"
      action_type: "stop"
      min_points: 6
'''
assert p.count(old) == 1
open(params, 'w').write(p.replace(old, new))
EOF

export ROS_DOMAIN_ID=$DOMAIN
COMPOSE=False
[ "$MODE" = component ] && COMPOSE=True
if [ "$COMPOSE" = True ]; then
  # navigation_launch.py loads into an existing container named nav2_container.
  ros2 run rclcpp_components component_container_isolated --ros-args -r __node:=nav2_container \
    > "$OUT/container_$MODE.log" 2>&1 &
  container=$!
fi
ros2 launch "$LAUNCH" use_sim_time:=False autostart:=True use_composition:=$COMPOSE \
  params_file:="$PARAMS" > "$OUT/launch_$MODE.log" 2>&1 &
launch=$!
python3 "$ROOT/review/collision_monitor/nav2/bringup_check.py" 120 2>&1 | tee "$OUT/check_$MODE.log"
kill -INT $launch 2>/dev/null
[ -n "${container:-}" ] && kill -INT "$container" 2>/dev/null
sleep 8
pkill -KILL -P $launch 2>/dev/null
kill -KILL $launch ${container:-} 2>/dev/null
grep -n "collision_monitor" "$OUT/launch_$MODE.log" "$OUT/container_$MODE.log" 2>/dev/null | head -40
grep -n "Managed nodes are active\|CRITICAL\|bond" "$OUT/launch_$MODE.log" "$OUT/container_$MODE.log" 2>/dev/null | head
