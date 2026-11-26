# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""Nav2's bring-up (localisation or SLAM, and navigation) with the
Lean-backed collision monitor.

This is nav2_bringup's bringup_launch.py, read from the installed file when
the launch starts, including this package's navigation_launch.py in the
place of Nav2's (roslean_nav2_bringup/derive.py makes the edit and stops the
launch if Nav2's file is not as expected). Localisation and SLAM are Nav2's
launch files, unchanged. The launch arguments are Nav2's: namespace,
use_namespace, slam, map, use_sim_time, params_file, autostart,
use_composition, use_respawn, log_level, use_localization. What differs:

  params_file, zones, check_params
                   as in this package's navigation_launch.py
  use_composition  defaults to False (Nav2: True): the stand-alone executable
                   is the form of the monitor that has every property claimed
                   for it (docs/collision_monitor/SPEC.md, D15)

It is not stock Nav2's behaviour: this monitor has no `approach` action
(docs/DROP_IN_COLLISION_MONITOR.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from roslean_nav2_bringup import derive


def generate_launch_description():
    navigation = os.path.join(get_package_share_directory(derive.PACKAGE),
                              'launch', 'navigation_launch.py')
    # Fail here, before anything is started, if navigation_launch.py cannot be derived.
    nav_path, nav_text = derive.stock('navigation_launch.py')
    derive.with_lean_monitor(nav_text, nav_path)

    path, text = derive.stock('bringup_launch.py')
    nav2 = derive.launch_description(path, derive.with_navigation_launch(text, navigation, path))
    return LaunchDescription(
        derive.arguments()
        + [OpaqueFunction(function=derive.resolve_params)]
        + derive.without_arguments(nav2, ['params_file', 'use_composition'], path))
