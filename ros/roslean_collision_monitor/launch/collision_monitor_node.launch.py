#!/usr/bin/env python3

# Copyright (c) 2022 Samsung R&D Institute Russia
# Copyright 2026 Beneficial AI Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The launch file of nav2_collision_monitor, with the Lean-backed node.

Derived from collision_monitor_node.launch.py of nav2_collision_monitor
1.3.12. The launch arguments, their defaults, the node names, the lifecycle
manager and the remappings are the same. Three things differ:

  - the monitor is `roslean_collision_monitor/collision_monitor`;
  - with use_composition:=True (the default, as upstream) the component is
    `roslean_collision_monitor::CollisionMonitorNode`. As upstream's file,
    this one does not start a container: one named `container_name` must be
    running. The stand-alone executable (use_composition:=False) is the form
    that has every property claimed for the node (docs/collision_monitor/
    SPEC.md, D15 and ND10);
  - the default params_file is this package's
    params/collision_monitor_params.yaml: upstream's example without the
    zones and the source this node does not provide.

A parameter file with an `approach` or `limit` zone, a source other than one
laser scan, or a velocity polygon is refused when the node is configured
(docs/DROP_IN_COLLISION_MONITOR.md).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.substitutions import NotEqualsSubstitution
from launch_ros.actions import LoadComposableNodes, SetParameter
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Environment
    package_dir = get_package_share_directory('roslean_collision_monitor')

    # Constant parameters
    lifecycle_nodes = ['collision_monitor']
    autostart = True
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Launch arguments
    # 1. Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    # 2. Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            package_dir, 'params', 'collision_monitor_params.yaml'
        ),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Use composed bringup if True',
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition',
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare node launching commands
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('namespace'), '')
                ),
                namespace=namespace,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_collision_monitor',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[{'autostart': autostart}, {'node_names': lifecycle_nodes}],
                remappings=remappings,
            ),
            Node(
                package='roslean_collision_monitor',
                executable='collision_monitor',
                output='screen',
                emulate_tty=True,  # https://github.com/ros2/launch/issues/188
                parameters=[configured_params],
                remappings=remappings,
            ),
        ],
    )

    load_composable_nodes = GroupAction(
        condition=IfCondition(use_composition),
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            PushROSNamespace(
                condition=IfCondition(
                    NotEqualsSubstitution(LaunchConfiguration('namespace'), '')
                ),
                namespace=namespace,
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_collision_monitor',
                        parameters=[
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes},
                        ],
                        remappings=remappings,
                    ),
                    ComposableNode(
                        package='roslean_collision_monitor',
                        plugin='roslean_collision_monitor::CollisionMonitorNode',
                        name='collision_monitor',
                        parameters=[configured_params],
                        remappings=remappings,
                    ),
                ],
            ),
        ],
    )

    ld = LaunchDescription()

    # Launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)

    # Node launching commands
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
