"""Demo: a TurtleBot3 in a furnished apartment, driven through the Lean-backed
twist_mux.

    patrol  (nav_vel, priority 10) --+
    teleop  (key_vel, priority 100) -+--> twist_mux --> base watchdog --> Gazebo
    e_stop  (lock, priority 255) ----+       cmd_vel_mux             cmd_vel

The base watchdog stands in for the robot's own firmware: it stops the robot
when the mux goes silent (the mux itself never sends a stop command).

Arguments:
  mux_package:=twist_mux   run upstream's node instead
  gui:=false               no Gazebo window (for tests)

The first launch downloads the furniture from Gazebo Fuel (about 100 MB).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Where the robot starts.
START = ['-x', '0', '-y', '0', '-z', '0.01']
# Where the Gazebo camera looks from.
CAMERA = ('pose: {position: {x: -5.4, y: -4.0, z: 3.9}, '
          'orientation: {x: -0.146, y: 0.283, z: 0.435, w: 0.842}}')


def generate_launch_description():
    here = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    demo = os.path.dirname(here)
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world = os.path.join(here, 'apartment.sdf')
    sim_time = {'use_sim_time': True}

    return LaunchDescription([
        DeclareLaunchArgument('mux_package', default_value='roslean_twist_mux'),
        DeclareLaunchArgument('gui', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),

        # Simulator: server always, window unless gui:=false.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': f'-r -s -v1 {world}', 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': '-g -v1'}.items(),
            condition=IfCondition(LaunchConfiguration('gui'))),
        TimerAction(period=15.0, actions=[ExecuteProcess(
            cmd=['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                 '--reptype', 'gz.msgs.Boolean', '--timeout', '5000', '--req', CAMERA],
            condition=IfCondition(LaunchConfiguration('gui')))]),

        # Robot and the Gazebo <-> ROS bridge.
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'burger', '-file',
                        os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf')] + START),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(here, 'bridge.yaml')}]),

        # The mux. Its output goes to the base watchdog, which alone publishes
        # the topic that is bridged to the robot.
        Node(package=LaunchConfiguration('mux_package'), executable='twist_mux',
             parameters=[os.path.join(here, 'gazebo.yaml'), sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(executable='python3', arguments=[os.path.join(here, 'base_watchdog.py')],
             parameters=[sim_time]),

        Node(executable='python3', arguments=[os.path.join(here, 'patrol.py')],
             parameters=[sim_time]),
        ExecuteProcess(
            cmd=['python3', os.path.join(demo, 'narrator.py'),
                 '--ros-args', '-r', 'cmd_vel_out:=cmd_vel_mux', '-p', 'stamped:=true'],
            output='screen'),
    ])
