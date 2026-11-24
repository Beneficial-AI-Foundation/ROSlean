"""Demo: Nav2's own bring-up starts the Lean-backed collision monitor in the
place of nav2_collision_monitor.

    controller_server --cmd_vel_nav--> velocity_smoother --cmd_vel_smoothed-->
        collision_monitor (roslean_collision_monitor) --cmd_vel--> Gazebo
                 ^
    lidar (scan) + TF (base_scan -> base_footprint, odom -> base_footprint)

There is no twist_mux, no base watchdog and no second monitor: the chain is
Nav2's. Two things differ from a stock Nav2 start, and nothing else:

  1. the launch file: nav2_bringup's navigation_launch.py with the package
     of one Node action changed (edited_navigation_launch() below does the
     edit on the installed file and refuses to go on if the file is not as
     expected);
  2. the parameters: the `collision_monitor` section is nav2_monitor.yaml
     (the footprint as a fixed stop polygon, larger than the footprint, and
     a slowdown zone) in the place of Nav2's `approach` zone.

Nav2's localisation (map_server, amcl) is started from nav2_bringup's
localization_launch.py, unchanged; this file does what bringup_launch.py does
with use_composition:=False, except that it names the edited copy.

Arguments:
  monitor_package:=nav2_collision_monitor
                           upstream's node, from the unedited launch file,
                           with the same parameters: for comparison
  zones:=approach          with upstream's node only: Nav2's own zone
                           (`FootprintApproach`), nothing edited at all
  driver:=tour             (default) Nav2 drives: nav_tour.py sends goals
  driver:=patrol           the patrol script publishes on `cmd_vel_nav`, the
                           topic of Nav2's controller, and Nav2 gets no goal.
                           The patrol does not avoid obstacles, so a box in
                           its path shows the monitor at work (scenario:=true)
  driver:=none             nobody drives; send goals from RViz
  scenario:=true           drop a box in front of the robot and take it away
                           again, by itself (obstacle.py run)
  gui:=false               no windows (for the automated check)
  rviz:=false              no RViz window
  furniture:=false         as in cm_gazebo_launch.py
"""

import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cm_gazebo_launch import CAMERA, START, world_file  # noqa: E402

# The one edit of navigation_launch.py: the package of the Node action that
# starts the monitor when Nav2 runs without composition.
STOCK_NODE = ("                package='nav2_collision_monitor',\n"
              "                executable='collision_monitor',\n")
LEAN_NODE = ("                package='roslean_collision_monitor',\n"
             "                executable='collision_monitor',\n")


def edited_navigation_launch(stock):
    """A copy of nav2_bringup's navigation_launch.py that starts the Lean node."""
    with open(stock) as f:
        text = f.read()
    if text.count(STOCK_NODE) != 1:
        raise RuntimeError(f'{stock} is not the navigation_launch.py this demo was written for: '
                           'the Node action of nav2_collision_monitor was not found once')
    out = tempfile.NamedTemporaryFile(mode='w', prefix='navigation_launch_roslean_',
                                      suffix='.py', delete=False)
    out.write(text.replace(STOCK_NODE, LEAN_NODE))
    out.close()
    return out.name


def edited_params(base, section):
    """`base` (the twist_mux demo's nav2_params.yaml) with its
    `collision_monitor` section replaced by the file `section`."""
    with open(base) as f:
        lines = f.read().split('\n')
    while lines[0].startswith('#'):   # the header says where that file's output goes
        del lines[0]
    lines.insert(0, f'# Written by cm_nav2_launch.py: {base}\n'
                    f'# with the collision_monitor section of {section}')
    start = lines.index('collision_monitor:')
    end = next(i for i in range(start + 1, len(lines))
               if lines[i] and not lines[i][0].isspace() and not lines[i].startswith('#'))
    with open(section) as f:
        new = [line for line in f.read().split('\n') if not line.startswith('#')]
    out = tempfile.NamedTemporaryFile(mode='w', prefix='nav2_params_roslean_',
                                      suffix='.yaml', delete=False)
    out.write('\n'.join(lines[:start] + new + lines[end:]))
    out.close()
    return out.name


def nav2(context):
    """Nav2's localisation and navigation, as bringup_launch.py starts them."""
    here = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                        'demo', 'gazebo')
    mux_gz = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    package = LaunchConfiguration('monitor_package').perform(context)
    zones = LaunchConfiguration('zones').perform(context)

    navigation = os.path.join(nav2_launch, 'navigation_launch.py')
    if package == 'roslean_collision_monitor':
        navigation = edited_navigation_launch(navigation)
    if zones == 'approach':
        if package != 'nav2_collision_monitor':
            raise RuntimeError("zones:=approach needs monitor_package:=nav2_collision_monitor: "
                               "the Lean node does not provide the 'approach' action")
        # Nav2's own zone. Only the output topic is put back to Nav2's
        # (`cmd_vel`; the twist_mux demo sends it to the mux instead).
        with open(os.path.join(mux_gz, 'nav2_params.yaml')) as f:
            text = f.read()
        assert text.count('cmd_vel_out_topic: "nav_vel"') == 1
        out = tempfile.NamedTemporaryFile(mode='w', prefix='nav2_params_approach_',
                                          suffix='.yaml', delete=False)
        out.write(text.replace('cmd_vel_out_topic: "nav_vel"', 'cmd_vel_out_topic: "cmd_vel"'))
        out.close()
        params = out.name
    else:
        params = edited_params(os.path.join(mux_gz, 'nav2_params.yaml'),
                               os.path.join(here, 'nav2_monitor.yaml'))
    print(f'[cm_nav2_launch] monitor: {package}; navigation launch file: {navigation}; '
          f'parameters: {params}')
    common = {'params_file': params, 'use_sim_time': 'true', 'autostart': 'true',
              'use_composition': 'False'}
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'localization_launch.py')),
            launch_arguments={'map': os.path.join(mux_gz, 'apartment_map.yaml'),
                              **common}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation), launch_arguments=common.items()),
    ]


def generate_launch_description():
    here = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                        'demo', 'gazebo')
    mux_gz = os.path.join(get_package_share_directory('roslean_twist_mux'), 'demo', 'gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    nav2_launch = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    world = world_file(os.path.join(mux_gz, 'apartment.sdf'))
    sim_time = {'use_sim_time': True}
    with open(os.path.join(get_package_share_directory('turtlebot3_description'),
                           'urdf', 'turtlebot3_burger.urdf')) as f:
        urdf = f.read().replace('${namespace}', '')  # a placeholder in the file

    return LaunchDescription([
        DeclareLaunchArgument('monitor_package', default_value='roslean_collision_monitor',
                              choices=['roslean_collision_monitor', 'nav2_collision_monitor']),
        DeclareLaunchArgument('zones', default_value='stop', choices=['stop', 'approach']),
        DeclareLaunchArgument('driver', default_value='tour',
                              choices=['tour', 'patrol', 'none']),
        DeclareLaunchArgument('gui', default_value='true'),
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

        # Robot, the Gazebo <-> ROS bridge (it carries `cmd_vel`, the
        # monitor's output, to the robot) and the robot's frames.
        Node(package='ros_gz_sim', executable='create',
             arguments=['-name', 'burger', '-file',
                        os.path.join(tb3, 'models', 'turtlebot3_burger', 'model.sdf')] + START),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             parameters=[{'config_file': os.path.join(mux_gz, 'bridge.yaml')}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[sim_time, {'robot_description': urdf}]),

        # Nav2. Its lifecycle manager configures and activates the monitor.
        OpaqueFunction(function=nav2),
        TimerAction(period=5.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch, 'rviz_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('gui'), "' == 'true' and '",
                LaunchConfiguration('rviz'), "' == 'true'"])))]),

        # Who drives.
        Node(executable='python3', arguments=[os.path.join(mux_gz, 'nav_tour.py')],
             parameters=[sim_time], output='screen',
             condition=LaunchConfigurationEquals('driver', 'tour')),
        Node(executable='python3', arguments=[os.path.join(mux_gz, 'patrol.py')],
             parameters=[sim_time], remappings=[('nav_vel', 'cmd_vel_nav')],
             condition=LaunchConfigurationEquals('driver', 'patrol')),

        # The narrator of the monitor, on Nav2's topics.
        ExecuteProcess(
            cmd=['python3', os.path.join(here, 'monitor_narrator.py'), '--ros-args',
                 '-p', 'use_sim_time:=true', '-p', 'watchdog:=false',
                 '-p', 'state_topic:=collision_monitor_state', '-p', 'source_timeout:=1.0',
                 '-r', 'cmd_vel_mux:=cmd_vel_smoothed', '-r', 'cmd_vel_monitored:=cmd_vel'],
            output='screen'),

        # The obstacle, by itself.
        ExecuteProcess(
            cmd=['python3', os.path.join(here, 'obstacle.py'), 'run'],
            output='screen', condition=IfCondition(LaunchConfiguration('scenario'))),
    ])
