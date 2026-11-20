"""Demo: turtlesim driven through the Lean-backed twist_mux.

    planner (nav_vel, priority 10) --+
    teleop  (key_vel, priority 100) -+--> twist_mux --> /turtle1/cmd_vel
    e_stop  (lock, priority 255) ----+

Pass `mux_package:=twist_mux` to run upstream's node instead.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    demo = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo')
    return LaunchDescription([
        DeclareLaunchArgument('mux_package', default_value='roslean_twist_mux'),
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(
            package=LaunchConfiguration('mux_package'), executable='twist_mux',
            parameters=[os.path.join(demo, 'demo.yaml')],
            remappings=[('cmd_vel_out', '/turtle1/cmd_vel')]),
        ExecuteProcess(cmd=['python3', os.path.join(demo, 'planner.py')]),
        ExecuteProcess(
            cmd=['python3', os.path.join(demo, 'narrator.py'),
                 '--ros-args', '-r', 'cmd_vel_out:=/turtle1/cmd_vel'],
            output='screen'),
    ])
