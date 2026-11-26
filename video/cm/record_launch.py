"""Records a take for the collision monitor video without opening a window.

The same nodes as the demo (ros/roslean_collision_monitor/demo/gazebo/
cm_gazebo_launch.py with scenario:=true): the Lean-backed twist_mux with
gazebo.yaml, the Lean-backed collision monitor with monitor.yaml behind it,
the lifecycle manager and robot_state_publisher the monitor needs, the base
watchdog, the patrol and the monitor's narrator.

Differences, for filming:
- the world is the filming world (make_world.py), which may have a camera;
- the monitor gets one more parameter, `state_topic`, so that it publishes
  what it decides (upstream's CollisionMonitorState) and the video can show
  the zone the monitor itself names;
- the box scenario is run by scenario.py, which calls the demo's obstacle.py
  and reports its events; recorder.py writes the take.

Arguments:
  world:=PATH      world written by make_world.py
  out:=DIR         where the take is written
  camera:=false    the world has no camera: record messages only
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))
GATE = 'cm_gate'                       # the monitor's node name, as in the demo
STATE_TOPIC = 'collision_monitor_state'


def generate_launch_description():
    cm = os.path.join(get_package_share_directory('roslean_collision_monitor'), 'demo', 'gazebo')
    mux = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    with open(os.path.join(get_package_share_directory('turtlebot3_description'),
                           'urdf', 'turtlebot3_burger.urdf')) as f:
        urdf = f.read().replace('${namespace}', '')
    sim_time = {'use_sim_time': True}
    out = LaunchConfiguration('out')
    scenario = ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'scenario.py'), os.path.join(cm, 'obstacle.py')],
        output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('world'),
        DeclareLaunchArgument('out'),
        DeclareLaunchArgument('camera', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', '-v1', LaunchConfiguration('world')]),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'burger', '-file',
                        os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf'),
                        '-x', '0', '-y', '0', '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(mux, 'bridge.yaml')}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
             arguments=['/video/image@sensor_msgs/msg/Image[gz.msgs.Image'],
             condition=IfCondition(LaunchConfiguration('camera'))),

        # As in the demo.
        Node(package='roslean_twist_mux', executable='twist_mux',
             parameters=[os.path.join(mux, 'gazebo.yaml'), sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
        Node(package='roslean_collision_monitor', executable='collision_monitor', name=GATE,
             parameters=[os.path.join(cm, 'monitor.yaml'), sim_time, {'state_topic': STATE_TOPIC}],
             output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_' + GATE,
             parameters=[sim_time, {'autostart': True, 'node_names': [GATE]}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': urdf}]),
        Node(executable='python3', arguments=[os.path.join(mux, 'base_watchdog.py')],
             parameters=[sim_time], remappings=[('cmd_vel_mux', 'cmd_vel_monitored')]),
        Node(executable='python3', arguments=[os.path.join(mux, 'patrol.py')],
             parameters=[sim_time]),
        ExecuteProcess(
            cmd=['python3', os.path.join(cm, 'monitor_narrator.py'), '--ros-args',
                 '-p', 'use_sim_time:=true', '-p', f'state_topic:={STATE_TOPIC}'],
            output='screen'),

        # For the video.
        Node(executable='python3', arguments=[os.path.join(HERE, 'recorder.py')],
             name='video_recorder', parameters=[sim_time, {'out': out}], output='screen'),
        Node(executable='python3', arguments=[os.path.join(HERE, 'recorder.py')],
             name='video_camera_recorder', parameters=[sim_time, {'out': out, 'camera': True}],
             output='screen', condition=IfCondition(LaunchConfiguration('camera'))),
        scenario,
        RegisterEventHandler(OnProcessExit(
            target_action=scenario,
            on_exit=[TimerAction(period=4.0, actions=[EmitEvent(event=Shutdown())])])),
    ])
