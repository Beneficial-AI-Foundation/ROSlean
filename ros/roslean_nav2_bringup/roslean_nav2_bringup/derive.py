# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""What the launch files of roslean_nav2_bringup share.

The launch files are not copies of nav2_bringup's. Each reads the installed
file of the same name when it is run, makes the edits listed below on its
text, and runs the result. So they follow a Nav2 update by themselves, as
long as the places that are edited are still there. If one is not found
exactly once, or if the edited text still names `nav2_collision_monitor`,
the launch stops with a message: it never goes on to start upstream's
monitor by default.

    navigation_launch.py   the Node and the ComposableNode of the collision
                           monitor: package, and for the component the plugin
    bringup_launch.py      the path of the navigation_launch.py it includes

Besides, the arguments `params_file` and `use_composition` are declared with
this package's defaults, and the parameter file is looked at before anything
is started (check_params.py of roslean_collision_monitor).
"""

import importlib.util
import os
import re
import types

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration

PACKAGE = 'roslean_nav2_bringup'
MONITOR = 'roslean_collision_monitor'
COMPONENT = 'roslean_collision_monitor::CollisionMonitorNode'

# zones:=<key> -> file in params/. The first is the default.
ZONES = {
    'slowdown': 'nav2_params.yaml',
    'stop_slowdown': 'nav2_params_stop_slowdown.yaml',
}

# (what, pattern in Nav2's navigation_launch.py, replacement). Quotes and
# white space may differ from Nav2 1.3.12; anything else may not.
_Q = r"""(['"])"""
MONITOR_EDITS = [
    ('the Node action of the collision monitor',
     re.compile(rf"package={_Q}nav2_collision_monitor\1,(\s*)executable={_Q}collision_monitor\3,"),
     rf"package='{MONITOR}',\2executable='collision_monitor',"),
    ('the ComposableNode of the collision monitor',
     re.compile(rf"package={_Q}nav2_collision_monitor\1,(\s*)"
                rf"plugin={_Q}nav2_collision_monitor::CollisionMonitor\3,"),
     rf"package='{MONITOR}',\2plugin='{COMPONENT}',"),
]
NAVIGATION_INCLUDE = re.compile(
    rf"os\.path\.join\(\s*launch_dir,\s*{_Q}navigation_launch\.py\1\s*\)")


class NotAsExpected(RuntimeError):
    """Nav2's launch file is not the one the edits were written for."""


def stock(name):
    """Path and text of nav2_bringup's installed launch file `name`."""
    path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', name)
    with open(path) as f:
        return path, f.read()


def _refuse(path, why):
    raise NotAsExpected(
        f'{PACKAGE}: cannot derive a launch file from {path}: {why}. This package was written '
        "for nav2_bringup 1.3 (Jazzy); the installed file differs where it is edited. Nothing "
        'was started. See roslean_nav2_bringup/derive.py for the edits.')


def with_lean_monitor(text, path='navigation_launch.py'):
    """The text of Nav2's navigation_launch.py with the collision monitor
    taken from roslean_collision_monitor, stand-alone and as a component."""
    for what, pattern, replacement in MONITOR_EDITS:
        found = len(pattern.findall(text))
        if found != 1:
            _refuse(path, f'{what} was found {found} times, not once')
        text = pattern.sub(replacement, text)
    if 'nav2_collision_monitor' in text:
        _refuse(path, "after the edits the file still names 'nav2_collision_monitor'")
    if text.count(f"package='{MONITOR}'") != 2 or text.count(COMPONENT) != 1:
        _refuse(path, f"the file already named '{MONITOR}'")
    return text


def with_navigation_launch(text, navigation, path='bringup_launch.py'):
    """The text of Nav2's bringup_launch.py, including the file `navigation`
    in the place of nav2_bringup's navigation_launch.py."""
    found = len(NAVIGATION_INCLUDE.findall(text))
    if found != 1:
        _refuse(path, f'the include of navigation_launch.py was found {found} times, not once')
    text = NAVIGATION_INCLUDE.sub(lambda _: repr(navigation), text)
    if text.count('navigation_launch.py') != 1:
        _refuse(path, 'navigation_launch.py is named in more than one place')
    if 'collision_monitor' in text:
        _refuse(path, 'the file starts a collision monitor itself')
    return text


def launch_description(path, text):
    """Runs the edited text of the launch file `path` and returns what its
    generate_launch_description() returns."""
    module = types.ModuleType('roslean_nav2_bringup_derived')
    module.__file__ = path
    exec(compile(text, f'{path} (edited by {PACKAGE})', 'exec'), module.__dict__)
    return module.generate_launch_description()


def without_arguments(description, names, path):
    """The entities of `description` without the declarations of the launch
    arguments `names`, each of which must be there once."""
    entities = list(description.entities)
    for name in names:
        declared = [e for e in entities
                    if isinstance(e, DeclareLaunchArgument) and e.name == name]
        if len(declared) != 1:
            _refuse(path, f"the launch argument '{name}' is declared {len(declared)} times, "
                          'not once')
        entities.remove(declared[0])
    return entities


def arguments():
    """The launch arguments this package declares itself."""
    params = os.path.join(get_package_share_directory(PACKAGE), 'params')
    default = next(iter(ZONES))
    return [
        DeclareLaunchArgument(
            'zones', default_value=default, choices=list(ZONES),
            description='Which parameter file of this package is used when params_file is '
                        'not given: ' + ', '.join(
                            f'{key}: {os.path.join(params, name)}' for key, name in ZONES.items())
                        + '. Without effect when params_file is given.'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Full path to the ROS2 parameters file to use for all launched nodes. '
                        'Empty (the default): the file of this package chosen by `zones`. '
                        "Nav2's own nav2_params.yaml is refused: its collision monitor zone "
                        'is an approach zone.'),
        DeclareLaunchArgument(
            'use_composition', default_value='False',
            description='Use composed bringup if True. False by default, here also in '
                        'bringup_launch.py: the stand-alone executable is the form of the '
                        'Lean-backed monitor that has every property claimed for it.'),
        DeclareLaunchArgument(
            'check_params', default_value='true', choices=['true', 'false'],
            description='Look at the collision monitor section of the parameter file before '
                        'anything is started, and stop with a message if the node would '
                        'refuse it.'),
    ]


def _checker():
    path = os.path.join(get_package_prefix(MONITOR), 'lib', MONITOR, 'check_params.py')
    spec = importlib.util.spec_from_file_location('roslean_check_params', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def refusals(path):
    """What check_params.py finds in the file: (the lines the node would
    refuse it for, a note or None). Raises OSError or yaml.YAMLError if the
    file cannot be read."""
    import yaml
    checker = _checker()
    with open(path) as f:
        content = yaml.safe_load(f)
    bad, seen = [], 0
    for where, section in checker.sections(content):
        if 'frequency' in section and 'cmd_vel_in_topic' not in section and 'detector' in where:
            continue  # a collision detector
        seen += 1
        bad += [f'{where}: {line}' for line in checker.unsupported(section)]
    return bad, None if seen else 'no collision monitor section was found in it'


def resolve_params(context):
    """For OpaqueFunction: settles `params_file` and looks at the file."""
    given = LaunchConfiguration('params_file').perform(context)
    if given and context.launch_configurations.get('roslean_params_settled') == given:
        return []   # bringup_launch.py did it before it included navigation_launch.py
    zones = LaunchConfiguration('zones').perform(context)
    if given:
        path, origin = given, 'given as params_file'
    else:
        path = os.path.join(get_package_share_directory(PACKAGE), 'params', ZONES[zones])
        origin = f'zones:={zones}'
    actions = [SetLaunchConfiguration('params_file', path),
               SetLaunchConfiguration('roslean_params_settled', path)]
    if LaunchConfiguration('check_params').perform(context) != 'true':
        return actions + [LogInfo(msg=f'[{PACKAGE}] parameters: {path} ({origin}); not checked')]
    if not os.path.isfile(path):
        raise RuntimeError(f'{PACKAGE}: the parameter file {path} does not exist. '
                           'Nothing was started.')
    bad, note = refusals(path)
    if bad:
        raise RuntimeError(
            f'{PACKAGE}: the collision monitor of this bring-up ({MONITOR}) would refuse the '
            f'parameter file {path}:\n' + ''.join(f'    {line}\n' for line in bad)
            + 'It provides the actions stop and slowdown, zones of type polygon and circle, '
              'and one laser scan. Nothing was started. What to do: take the collision_monitor '
              f'section of {os.path.join(get_package_share_directory(PACKAGE), "params")}'
              f'/{ZONES[next(iter(ZONES))]} into your file and fit the zones to your robot, or '
              "start stock Nav2 (ros2 launch nav2_bringup ...) to keep upstream's monitor. "
              'check_params:=false skips this check; the node then refuses the file itself '
              "and Nav2's lifecycle manager aborts the bring-up.")
    checked = 'collision monitor section: nothing unsupported found' if note is None \
        else f'not checked: {note}'
    return actions + [LogInfo(msg=f'[{PACKAGE}] collision monitor: {MONITOR}; '
                                  f'parameters: {path} ({origin}); {checked}')]
