#!/usr/bin/env python3
# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""Forwards to collision_detector_node.launch.py of upstream nav2_collision_monitor.

The ROSlean overlay package named nav2_collision_monitor provides the
collision monitor only; the collision detector is upstream's. This file has
the name of upstream's launch file for the detector. It finds upstream's file
in the other prefixes of AMENT_PREFIX_PATH and includes it, so the arguments
(namespace, use_sim_time, params_file, use_composition, container_name) are
upstream's. The default of params_file is upstream's
params/collision_detector_params.yaml, which the overlay does not copy.

Upstream's file starts `collision_detector` from the package
nav2_collision_monitor, which is the overlay: the executable of that name
there forwards to upstream's, and the component
`nav2_collision_monitor::CollisionDetector` is listed in the overlay's entry
of the ament index with the path of upstream's library, if upstream was
installed when the overlay was built.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAME = 'collision_detector_node.launch.py'


def upstream_share():
    here = os.path.realpath(__file__)
    for prefix in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep):
        if not prefix:
            continue
        share = os.path.join(prefix, 'share', 'nav2_collision_monitor')
        candidate = os.path.join(share, 'launch', NAME)
        if os.path.isfile(candidate) and os.path.realpath(candidate) != here:
            return share
    raise RuntimeError(
        f'{NAME}: the collision detector is not provided by the ROSlean overlay of '
        'nav2_collision_monitor, and no upstream nav2_collision_monitor was found under it '
        'on AMENT_PREFIX_PATH. Install upstream nav2_collision_monitor and source it before '
        'this workspace.')


def generate_launch_description():
    share = upstream_share()
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'params', 'collision_detector_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all launched nodes'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', NAME))),
    ])
