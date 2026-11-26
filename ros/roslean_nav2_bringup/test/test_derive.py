#!/usr/bin/env python3
# Copyright 2026 Beneficial AI Foundation
# Licensed under the Apache License, Version 2.0.
"""The edits of nav2_bringup's launch files, on the installed files, and the
check of the parameter files. No node is started."""

import os
import sys
import unittest

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from roslean_nav2_bringup import derive

HERE = os.path.dirname(os.path.abspath(__file__))
PARAMS = os.path.join(HERE, '..', 'params')


class Navigation(unittest.TestCase):

    def setUp(self):
        self.path, self.text = derive.stock('navigation_launch.py')

    def test_installed_file_is_edited(self):
        edited = derive.with_lean_monitor(self.text, self.path)
        self.assertNotIn('nav2_collision_monitor', edited)
        self.assertEqual(edited.count("package='roslean_collision_monitor'"), 2)
        self.assertIn("plugin='roslean_collision_monitor::CollisionMonitorNode'", edited)
        # Nothing else changed: putting upstream's names back gives Nav2's file
        # (up to the quotes and white space the patterns allow, which 1.3 does not use).
        back = edited.replace("package='roslean_collision_monitor'",
                              "package='nav2_collision_monitor'").replace(
            derive.COMPONENT, 'nav2_collision_monitor::CollisionMonitor')
        self.assertEqual(back, self.text)

    def test_edited_file_runs_and_declares_nav2s_arguments(self):
        description = derive.launch_description(
            self.path, derive.with_lean_monitor(self.text, self.path))
        names = {e.name for e in description.entities if isinstance(e, DeclareLaunchArgument)}
        self.assertLessEqual({'namespace', 'use_sim_time', 'autostart', 'params_file',
                              'use_composition', 'container_name', 'use_respawn',
                              'log_level'}, names)
        rest = derive.without_arguments(description, ['params_file', 'use_composition'], 'x')
        self.assertEqual(len(rest), len(description.entities) - 2)

    def test_refused_when_not_as_expected(self):
        node = "package='nav2_collision_monitor',\n                executable="
        self.assertEqual(self.text.count(node), 1)
        for other in (
                self.text.replace(node, "package='nav2_collision_monitor2',\n   executable="),
                self.text.replace('nav2_collision_monitor::CollisionMonitor', 'x::Y'),
                self.text + "\n# Node(package='nav2_collision_monitor', executable='other')\n",
                self.text + self.text,
                derive.with_lean_monitor(self.text),
                ''):
            with self.assertRaises(derive.NotAsExpected):
                derive.with_lean_monitor(other)

    def test_other_quotes_and_white_space_are_accepted(self):
        other = self.text.replace("package='nav2_collision_monitor',\n",
                                  'package="nav2_collision_monitor",\n\n  ')
        self.assertNotEqual(other, self.text)
        self.assertNotIn('nav2_collision_monitor', derive.with_lean_monitor(other))


class Bringup(unittest.TestCase):

    def setUp(self):
        self.path, self.text = derive.stock('bringup_launch.py')

    def test_installed_file_is_edited(self):
        edited = derive.with_navigation_launch(self.text, '/somewhere/navigation_launch.py')
        self.assertIn("'/somewhere/navigation_launch.py'", edited)
        self.assertNotIn("launch_dir, 'navigation_launch.py'", edited)
        description = derive.launch_description(self.path, edited)
        derive.without_arguments(description, ['params_file', 'use_composition'], 'x')

    def test_refused_when_not_as_expected(self):
        for other in (self.text.replace("'navigation_launch.py'", "'nav_launch.py'"),
                      self.text + "\n# navigation_launch.py\n",
                      self.text + "\n# collision_monitor\n",
                      ''):
            with self.assertRaises(derive.NotAsExpected):
                derive.with_navigation_launch(other, '/somewhere/navigation_launch.py')


class Params(unittest.TestCase):

    def test_this_packages_files_are_accepted(self):
        for name in derive.ZONES.values():
            self.assertEqual(derive.refusals(os.path.join(PARAMS, name)), ([], None), name)

    def test_zones_of_each_file(self):
        import yaml
        zones = {}
        for key, name in derive.ZONES.items():
            with open(os.path.join(PARAMS, name)) as f:
                p = yaml.safe_load(f)['collision_monitor']['ros__parameters']
            zones[key] = [p[z]['action_type'] for z in p['polygons']]
        self.assertEqual(zones, {'slowdown': ['slowdown'], 'stop_slowdown': ['stop', 'slowdown']})

    def test_nav2s_file_is_refused(self):
        bad, note = derive.refusals(os.path.join(
            get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'))
        self.assertIsNone(note)
        self.assertTrue(any('approach' in line for line in bad), bad)


if __name__ == '__main__':
    sys.exit(not unittest.main(exit=False, verbosity=2).result.wasSuccessful())
