"""Records the footage for the demo video without opening a window.

The same nodes as the Gazebo demo (demo/gazebo/gazebo_launch.py): the
Lean-backed twist_mux with gazebo.yaml, the patrol planner and the base
watchdog. Added: a world with a camera (make_world.py), the scripted operator
and e-stop (scenario.py) and the recorder (recorder.py).

Arguments:
  world:=PATH   world written by make_world.py
  out:=DIR      where the take is written
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    demo = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    sim_time = {'use_sim_time': True}
    recorder = Node(executable='python3', arguments=[os.path.join(HERE, 'recorder.py')],
                    parameters=[sim_time, {'out': LaunchConfiguration('out')}], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('out'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v1', LaunchConfiguration('world')]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'burger', '-file',
                        os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', '0', '-y', '0', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(demo, 'bridge.yaml')}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
             arguments=['/video/image@sensor_msgs/msg/Image[gz.msgs.Image']),

        # Exactly as in the demo.
        Node(package='roslean_twist_mux', executable='twist_mux',
             parameters=[os.path.join(demo, 'gazebo.yaml'), sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(executable='python3', arguments=[os.path.join(demo, 'base_watchdog.py')],
             parameters=[sim_time]),
        Node(executable='python3', arguments=[os.path.join(demo, 'patrol.py')],
             parameters=[sim_time]),

        Node(executable='python3', arguments=[os.path.join(HERE, 'scenario.py')],
             parameters=[sim_time], output='screen'),
        recorder,
        RegisterEventHandler(OnProcessExit(target_action=recorder,
                                           on_exit=[EmitEvent(event=Shutdown())])),
    ])
