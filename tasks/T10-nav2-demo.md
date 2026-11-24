# T10 — Demo: Nav2 launches the Lean node

Branch: `claude/cm-nav2-demo`. Needs: T09 merged. **Needs a machine with a
display** (Gazebo renders the lidar); run it locally unless T00 found that the
cloud can render. Read [README.md](README.md) first.

## Do

1. Extend the Gazebo demo (`ros/roslean_collision_monitor/demo/gazebo/`,
   `docs/TUTORIAL_COLLISION_MONITOR.md`) with a variant in which Nav2's own
   bring-up starts `roslean_collision_monitor` in place of
   `nav2_collision_monitor`: the lifecycle manager activates it, the lidar
   pose comes from TF, the zone is Nav2's footprint with `action_type: stop`
   (and a slowdown zone), and no second monitor runs. Show the exact diff
   against Nav2's stock launch and parameter files in the tutorial.
2. The obstacle scenario and `check_demo.py` for this variant: free driving,
   slowdown, stop before the box, resume. Run it at least ten times and
   report the numbers; run upstream in the same variant for comparison.
3. Remove `roslean_sensor_pose` from the demo if T06 made it unnecessary.
4. Watch `pixi run cm-gazebo-scenario` and the Nav2 variant on a screen, and
   add a screenshot of each to the tutorial.
5. The tutorial's section on what the demo does and does not show, updated:
   in particular what the robot does differently from stock Nav2 because the
   zone stops instead of approaching.

## May change

`ros/roslean_collision_monitor/demo/`, `docs/TUTORIAL_COLLISION_MONITOR.md`,
`docs/images/`, `pixi.toml` (tasks), the marked demo block of the package's
`CMakeLists.txt`.
