"""The Lean-backed collision monitor with a Nav2 lifecycle manager.

The counterpart of nav2_collision_monitor's collision_monitor_node.launch.py
(without composition): the node under the name `collision_monitor`, with the
remappings Nav2 uses for TF, and a lifecycle manager that configures and
activates it and watches its bond.

    ros2 launch roslean_collision_monitor collision_monitor_launch.py \
        params_file:=/path/to/params.yaml use_sim_time:=true

To use the node inside nav2_bringup instead, see
docs/collision_monitor/SPEC_NAV2.md, section 8.6: in a copy of
navigation_launch.py replace package 'nav2_collision_monitor' by
'roslean_collision_monitor' (the executable is `collision_monitor` in both).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('roslean_collision_monitor')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'example.yaml')),
        Node(
            package='roslean_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            namespace=namespace,
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}, {'autostart': True},
                        {'node_names': ['collision_monitor']}]),
    ])
