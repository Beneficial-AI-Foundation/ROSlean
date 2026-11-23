"""Demo: a TurtleBot3 in a furnished apartment, driven through the Lean-backed
twist_mux.

    planner (nav_vel, priority 10) --+
    teleop  (key_vel, priority 100) -+--> twist_mux --> base watchdog --> Gazebo
    e_stop  (lock, priority 255) ----+       cmd_vel_mux             cmd_vel

The planner is either a scripted patrol (default) or Nav2 (`planner:=nav2`),
which localises with the lidar on a map of the apartment and drives to goals
sent by a tour script, from RViz ("Nav2 Goal") or to the `navigate_to_pose` action.

The base watchdog stands in for the robot's own firmware: it stops the robot
when the mux goes silent (the mux itself never sends a stop command).

Arguments:
  planner:=nav2            Nav2 instead of the patrol script
  tour:=false              with Nav2: do not send goals automatically
  rviz:=false              with Nav2: no RViz window (RViz shows the map, the lidar
                           and the plan, and sets goals with "Nav2 Goal")
  mux_package:=twist_mux   run upstream's node instead
  gui:=false               no windows (for tests)

The first launch downloads the furniture from Gazebo Fuel (about 100 MB).
"""

import os

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


def generate_launch_description():
    here = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    demo = os.path.dirname(here)
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    world = os.path.join(here, 'apartment.sdf')
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
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('tour', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
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

        # Planner, variant 1: the patrol script.
        Node(executable='python3', arguments=[os.path.join(here, 'patrol.py')],
             parameters=[sim_time], condition=use_patrol),

        # Planner, variant 2: Nav2. It needs the robot's frames (from the URDF)
        # and publishes its velocity commands on nav_vel (see nav2_params.yaml).
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': urdf}], condition=use_nav2),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'bringup_launch.py')),
            launch_arguments={
                'map': os.path.join(here, 'apartment_map.yaml'),
                'params_file': os.path.join(here, 'nav2_params.yaml'),
                'use_sim_time': 'true', 'use_composition': 'False', 'autostart': 'true',
            }.items(),
            condition=use_nav2),
        Node(executable='python3', arguments=[os.path.join(here, 'nav_tour.py')],
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

        ExecuteProcess(
            cmd=['python3', os.path.join(demo, 'narrator.py'),
                 '--ros-args', '-r', 'cmd_vel_out:=cmd_vel_mux', '-p', 'stamped:=true'],
            output='screen'),
    ])
