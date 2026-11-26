#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
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

"""The launch file of upstream twist_mux, with the Lean-backed node as the mux.

Derived from twist_mux_launch.py of twist_mux 4.5.0 (author: Brighten Lee).
The launch arguments, their defaults, the remappings and the parameters are
the same; the default config files are this package's copies of upstream's.

The mux is `roslean_twist_mux/twist_mux`. This package has no `twist_marker`
and no `joystick_relay.py` (docs/SPEC.md, D3). They are started from the
package `twist_mux` when one is installed, exactly as upstream's launch file
starts them, and left out with a message when it is not.
"""

import os

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def helper_available(executable):
    try:
        prefix = get_package_prefix('twist_mux')
    except PackageNotFoundError:
        return False
    return os.path.isfile(os.path.join(prefix, 'lib', 'twist_mux', executable))


def generate_launch_description():
    config = os.path.join(get_package_share_directory('roslean_twist_mux'), 'config')
    default_config_locks = os.path.join(config, 'twist_mux_locks.yaml')
    default_config_topics = os.path.join(config, 'twist_mux_topics.yaml')
    default_config_joystick = os.path.join(config, 'joystick.yaml')

    actions = [
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'cmd_vel_out',
            default_value='twist_mux/cmd_vel',
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation time'),
        Node(
            package='roslean_twist_mux',
            executable='twist_mux',
            output='screen',
            remappings={('/cmd_vel_out', LaunchConfiguration('cmd_vel_out'))},
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_locks'),
                LaunchConfiguration('config_topics')]
        ),
    ]

    if helper_available('twist_marker'):
        actions.append(Node(
            package='twist_mux',
            executable='twist_marker',
            output='screen',
            remappings={('/twist', LaunchConfiguration('cmd_vel_out'))},
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'scale': 1.0,
                'vertical_position': 2.0}]))
    else:
        actions.append(LogInfo(
            msg='twist_marker is not started: package twist_mux is not installed'))

    if helper_available('joystick_relay.py'):
        actions.append(Node(
            package='twist_mux',
            executable='joystick_relay.py',
            output='screen',
            remappings={('joy_vel_in', 'input_joy/cmd_vel'),
                        ('joy_vel_out', 'joy_vel')},
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                LaunchConfiguration('config_joy')]))
    else:
        actions.append(LogInfo(
            msg='joystick_relay.py is not started: package twist_mux is not installed'))

    return LaunchDescription(actions)
