# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""Nav2's navigation stack with the Lean-backed collision monitor.

This is nav2_bringup's navigation_launch.py, read from the installed file
when the launch starts, with the collision monitor taken from
roslean_collision_monitor (roslean_nav2_bringup/derive.py makes the edit and
stops the launch if Nav2's file is not as expected). The launch arguments
are Nav2's: namespace, use_sim_time, autostart, params_file,
use_composition, container_name, use_respawn, log_level. Three things
differ:

  params_file      defaults to this package's params/nav2_params.yaml: Nav2's
                   default parameters with zones this monitor accepts
  zones            slowdown (default) or stop_slowdown: which of this
                   package's two parameter files is the default
  check_params     true (default): a parameter file the monitor would refuse
                   is reported before anything is started

It is not stock Nav2's behaviour: this monitor has no `approach` action
(docs/DROP_IN_COLLISION_MONITOR.md).
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from roslean_nav2_bringup import derive


def generate_launch_description():
    path, text = derive.stock('navigation_launch.py')
    nav2 = derive.launch_description(path, derive.with_lean_monitor(text, path))
    return LaunchDescription(
        derive.arguments()
        + [OpaqueFunction(function=derive.resolve_params)]
        + derive.without_arguments(nav2, ['params_file', 'use_composition'], path))
