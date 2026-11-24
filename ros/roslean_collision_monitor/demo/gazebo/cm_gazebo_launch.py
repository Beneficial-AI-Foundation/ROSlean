"""Demo: a TurtleBot3 in the furnished apartment, with the Lean-backed twist_mux
and the Lean-backed collision monitor in series.

    planner (nav_vel, 10) ---+
    teleop  (key_vel, 100) --+--> twist_mux --> collision monitor --> base watchdog --> Gazebo
    e_stop  (lock, 255) -----+    cmd_vel_mux   cmd_vel_monitored     cmd_vel
                                                     ^
                                  lidar (scan) ------+

The mux picks a command; the monitor passes it, scales it down or replaces it
by zero, depending on what the lidar sees in its two zones (monitor.yaml).

The world, the bridge, the mux configuration, the patrol, the base watchdog
and the Nav2 set-up are those of the twist_mux demo and are used from where
that package installs them; only the monitor's configuration, the obstacle
scripts and the monitor's narrator are in this directory.

Arguments:
  monitor_package:=nav2_collision_monitor
                           run upstream's node instead of the Lean one. Both
                           are lifecycle nodes, so a lifecycle manager is
                           started; the transform from the lidar to the base
                           that they look up comes from robot_state_publisher.
  planner:=nav2            Nav2 instead of the patrol script
  tour:=false              with Nav2: do not send goals automatically
  rviz:=false              with Nav2: no RViz window
  mux_package:=twist_mux   run upstream's mux instead of the Lean one
  gui:=false               no windows (for the automated check)
  scenario:=true           drop a box in front of the robot and take it away
                           again, by itself (obstacle.py run)
  furniture:=false         leave the furniture out of the world: walls, floor
                           and robot only. For machines that cannot reach
                           Gazebo Fuel. A stopgap: see world_file() below.

The first launch downloads the furniture from Gazebo Fuel (about 100 MB).
"""

import os
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# Where the robot starts.
START = ['-x', '0', '-y', '0', '-z', '0.01']
# Where the Gazebo camera looks from.
CAMERA = ('pose: {position: {x: -5.4, y: -4.0, z: 3.9}, '
          'orientation: {x: -0.146, y: 0.283, z: 0.435, w: 0.842}}')
# The monitor's node name (see monitor.yaml for why it is not `collision_monitor`).
GATE = 'cm_gate'


def world_file(apartment):
    """The world to load: the apartment, or with furniture:=false a copy of it
    in a temporary file without the models that come from Gazebo Fuel.

    TODO: a stopgap for machines where fuel.gazebosim.org is blocked
    (tasks/reports/T00.md); better would be a world that needs no download.
    The argument is read from
    the command line because the world's path is needed before the launch
    arguments are resolved.
    """
    if not any(a in ('furniture:=false', 'furniture:=False') for a in sys.argv):
        return apartment
    with open(apartment) as f:
        text = f.read()
    bare, removed = re.subn(
        r'[ \t]*<include>(?:(?!</include>).)*?fuel\.gazebosim\.org.*?</include>\n?',
        '', text, flags=re.S)
    out = tempfile.NamedTemporaryFile(
        mode='w', prefix='apartment_bare_', suffix='.sdf', delete=False)
    out.write(bare)
    out.close()
    print(f'[cm_gazebo_launch] furniture:=false: {removed} models left out, world in {out.name}')
    return out.name


def generate_launch_description():
    here = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                        'demo', 'gazebo')
    # The pieces shared with the twist_mux demo.
    mux_demo = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo')
    mux_gz = os.path.join(mux_demo, 'gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world = world_file(os.path.join(mux_gz, 'apartment.sdf'))
    sim_time = {'use_sim_time': True}
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    with open(os.path.join(get_package_share_directory('turtlebot3_description'),
                           'urdf', 'turtlebot3_burger.urdf')) as f:
        urdf = f.read().replace('${namespace}', '')  # a placeholder in the file
    use_nav2 = LaunchConfigurationEquals('planner', 'nav2')
    use_patrol = LaunchConfigurationEquals('planner', 'patrol')

    return LaunchDescription([
        DeclareLaunchArgument('planner', default_value='patrol', choices=['patrol', 'nav2']),
        DeclareLaunchArgument('mux_package', default_value='roslean_twist_mux'),
        DeclareLaunchArgument('monitor_package', default_value='roslean_collision_monitor',
                              choices=['roslean_collision_monitor', 'nav2_collision_monitor']),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('tour', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scenario', default_value='false'),
        DeclareLaunchArgument('furniture', default_value='true', choices=['true', 'false']),
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
             parameters=[{'config_file': os.path.join(mux_gz, 'bridge.yaml')}]),

        # The mux, configured as in its own demo.
        Node(package=LaunchConfiguration('mux_package'), executable='twist_mux',
             parameters=[os.path.join(mux_gz, 'gazebo.yaml'), sim_time],
             remappings=[('cmd_vel_out', 'cmd_vel_mux')]),

        # The collision monitor, between the mux and the base. Both packages
        # call their executable `collision_monitor`; the topics are parameters
        # (monitor.yaml).
        Node(package=LaunchConfiguration('monitor_package'), executable='collision_monitor',
             name=GATE, parameters=[os.path.join(here, 'monitor.yaml'), sim_time],
             output='screen'),
        # Either node is a lifecycle node (the Lean one since task T06) and
        # does nothing until it is configured and activated.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_' + GATE,
             parameters=[sim_time, {'autostart': True, 'node_names': [GATE]}]),
        # The robot's frames, from the URDF. Either monitor looks up
        # base_scan -> base_footprint here; Nav2 needs them as well.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': urdf}]),

        # The base watchdog of the mux demo, fed by the monitor instead of by
        # the mux. It alone publishes the topic that is bridged to the robot.
        Node(executable='python3', arguments=[os.path.join(mux_gz, 'base_watchdog.py')],
             parameters=[sim_time], remappings=[('cmd_vel_mux', 'cmd_vel_monitored')]),

        # Planner, variant 1: the patrol script.
        Node(executable='python3', arguments=[os.path.join(mux_gz, 'patrol.py')],
             parameters=[sim_time], condition=use_patrol),

        # Planner, variant 2: Nav2, publishing on nav_vel (see nav2_params.yaml
        # in the mux demo).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(mux_gz, 'apartment_map.yaml'),
                'params_file': os.path.join(mux_gz, 'nav2_params.yaml'),
                'use_sim_time': 'true', 'use_composition': 'False', 'autostart': 'true',
            }.items(),
            condition=use_nav2),
        Node(executable='python3', arguments=[os.path.join(mux_gz, 'nav_tour.py')],
             parameters=[sim_time], output='screen',
             condition=IfCondition(PythonExpression([
                 "'", LaunchConfiguration('planner'), "' == 'nav2' and '",
                 LaunchConfiguration('tour'), "' == 'true'"]))),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'rviz_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('planner'), "' == 'nav2' and '",
                LaunchConfiguration('gui'), "' == 'true' and '",
                LaunchConfiguration('rviz'), "' == 'true'"])))]),

        # Two narrators: who drives (the mux), and what the monitor does to
        # the command.
        ExecuteProcess(
            cmd=['python3', os.path.join(mux_demo, 'narrator.py'),
                 '--ros-args', '-r', 'cmd_vel_out:=cmd_vel_mux', '-p', 'stamped:=true'],
            output='screen'),
        ExecuteProcess(
            cmd=['python3', os.path.join(here, 'monitor_narrator.py'),
                 '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen'),

        # The obstacle, by itself.
        ExecuteProcess(
            cmd=['python3', os.path.join(here, 'obstacle.py'), 'run'],
            output='screen', condition=IfCondition(LaunchConfiguration('scenario'))),
    ])
