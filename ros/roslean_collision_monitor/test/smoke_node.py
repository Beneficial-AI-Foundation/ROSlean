#!/usr/bin/env python3
"""Smoke test of the collision monitor node over real ROS topics.

    smoke_node.py NODE

NODE is the path of the `collision_monitor` executable. The node runs with
`use_sim_time` and this script publishes `/clock`, so the node-clock time of
every event is chosen exactly. The script sends scans and velocity commands
and checks what comes out on `cmd_vel`: the command unchanged (pass-through),
zero (stop), the command scaled (slowdown), or nothing. Expectations follow
docs/collision_monitor/SPEC.md. It then checks that the node refuses to start
with configurations outside the supported subset, and names the parameter
(all refused parameters in one message), and that it warns about accepted
configurations that protect nothing.

This is a smoke test of the shim, not a comparison with upstream.

The node runs with its logger at debug level and the script reads its log.
An expectation of "nothing" is met only if nothing was published AND the
node logged that it left that command unanswered: a lost message or a node
that stopped listening fails the step. Published values are compared bit for
bit, so `-0.0` is not `0.0`.

The only wall-clock dependence is that messages must be delivered within
SETTLE seconds, and that a clock update or scan reaches the node before the
next message does.
"""

import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters, SetParametersAtomically
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
SETTLE = 0.2  # s of wall time allowed for delivery
NS = 1_000_000_000

# What the node logs (at debug level) for a command it does not answer.
NOT_ANSWERED = 'Command not answered'

# The command sent in every 'cmd' step: all six components non-zero, so that
# a copied linear.z, angular.x or angular.y would be noticed (R16).
CMD = (0.5, 0.25, -1.5, 0.5, -0.125, 0.2)  # linear x y z, angular x y z
ZERO = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
NAN = (0.5, 0.25, math.nan, 0.5, -0.125, 0.2)

PASS = ('pass', None)
STOP = ('zero', None)
NOTHING = ('nothing', None)


def slow(ratio):
    return ('scaled', ratio)


# A scan is five readings straight ahead (angles -0.02 .. 0.02 rad), given as
# their ranges in metres. Zones (smoke_params.yaml): slowdown circle 0.6 m at
# 0.5, slowdown circle 1.0 m at 0.75, stop box x in [0, 0.3] m; 3 points each.
FAR = [5.0] * 5
STEPS = [
    # (kind, argument[, expected, label])
    ('clock', 100 * NS),
    ('cmd', CMD, STOP, 'no scan yet: zero (invalid source, R8)'),
    ('scan', FAR),
    ('cmd', CMD, PASS, 'nothing in any zone: the command passes unchanged'),
    ('scan', [0.8] * 5),
    ('cmd', CMD, slow(0.75), 'five points in the 1.0 m circle: scaled by 0.75'),
    ('scan', [0.5] * 5),
    ('cmd', CMD, slow(0.5), 'in both circles: the smaller ratio, 0.5 (R13)'),
    ('scan', [0.5, 0.5, 5.0, 5.0, 5.0]),
    ('cmd', CMD, PASS, 'two points are below min_points 3: pass (R11)'),
    ('scan', [0.5, 0.5, math.nan, math.inf, 0.01]),
    ('cmd', CMD, PASS, 'NaN, inf and below-range_min readings do not count (R4)'),
    ('scan', [0.2] * 5),
    ('cmd', CMD, STOP, 'points in the stop box: zero, although slowdown zones come first (R12)'),
    ('clock', 101 * NS),
    ('cmd', CMD, STOP, 'zero still published exactly at stop_pub_timeout (R15)'),
    ('clock', 101 * NS + 1),
    ('cmd', CMD, NOTHING, 'nothing 1 ns after stop_pub_timeout'),
    ('scan', FAR),
    ('cmd', NAN, NOTHING, 'a command with NaN in linear.z is ignored (R2)'),
    ('cmd', ZERO, NOTHING, 'a zero command continues the zero phase: nothing'),
    ('cmd', CMD, PASS, 'the first non-zero velocity is published again'),
    ('cmd', ZERO, PASS, 'a zero command starts a new zero phase: published'),
    ('scan', []),
    ('cmd', CMD, PASS, 'an empty scan is free space (R4)'),
    # The stored scan is stamped 101 s + 1 ns; source_timeout is 2 s.
    ('clock', 103 * NS + 1),
    ('cmd', CMD, PASS, 'scan not too old exactly at source_timeout (R7)'),
    ('clock', 103 * NS + 2),
    ('cmd', CMD, STOP, 'scan too old 1 ns later: zero (R8)'),
    ('scan', [math.inf] * 5),
    ('cmd', CMD, PASS, 'a fresh scan of inf readings: pass'),
    # D11: a scan in another frame cannot be placed without a sensor pose.
    ('scan', FAR, 'laser_link'),
    ('cmd', CMD, STOP, 'a scan in another frame is not used: zero, as without data (R8)'),
    ('scan', FAR),
    ('cmd', CMD, PASS, 'the next scan in the base frame is used again'),
]

# Five readings around the direction pi of the scan's frame.
BEHIND = (math.pi - 0.02, 0.01)

# With `scan.roslean_sensor_pose: [0.0, 0.0, pi]` (a sensor mounted backwards)
# a scan in the sensor's frame is used, and what the sensor sees ahead is
# behind the robot: outside the stop box, inside the 0.6 m circle.
POSE_PARAMS = ['scan.roslean_sensor_pose:=[0.0, 0.0, 3.141592653589793]']
POSE_STEPS = [
    ('clock', 100 * NS),
    ('scan', [0.2] * 5, 'laser_link'),
    ('cmd', CMD, slow(0.5), 'sensor pose set: 0.2 m "ahead" of a backwards sensor is behind the robot'),
    ('scan', [0.2] * 5, 'laser_link', *BEHIND),
    ('cmd', CMD, STOP, 'and 0.2 m behind the backwards sensor is ahead of the robot: stop'),
    ('scan', FAR, 'laser_link'),
    ('cmd', CMD, PASS, 'sensor pose set: a scan in the sensor frame is used'),
    ('log', "'scan.roslean_sensor_pose' is set: scans are used whatever their frame_id", 1, 1,
     'start-up warning, once, that the pose switches the frame check off'),
    ('log', '[x, y, yaw] = [0, 0, 3.14159]', 1, 1, 'which gives the pose in use'),
    ('log', "Scan is in frame 'laser_link', not in the base frame 'base_link'. It is used", 1, 3,
     'throttled warning when a scan arrives in another frame while the pose is set'),
]

# Review CM3/CM4, finding 1 and 4f: a pose of zero is "the sensor is the base
# frame". The scan is used as if it were in the base frame, whatever the
# sensor's real mounting, and the node says so.
ZERO_POSE_PARAMS = ['scan.roslean_sensor_pose:=[0.0, 0.0, 0.0]']
ZERO_POSE_STEPS = [
    ('clock', 100 * NS),
    ('scan', [0.2] * 5, 'laser_link'),
    ('cmd', CMD, STOP, 'zero pose: a scan in laser_link is taken as if in the base frame: stop'),
    ('scan', [0.2] * 5, 'laser_link', *BEHIND),
    ('cmd', CMD, slow(0.5), 'zero pose: what the sensor sees behind itself counts as behind the robot'),
    ('scan', [0.2] * 5),
    ('cmd', CMD, STOP, 'zero pose: a scan in the base frame, the same'),
    ('log', '[x, y, yaw] = [0, 0, 0]', 1, 1, 'start-up warning with the zero pose'),
    ('log', "Scan is in frame 'laser_link', not in the base frame 'base_link'. It is used", 1, 3,
     'warning about the scan in another frame'),
]

# The same scans without the parameter: none of those in another frame is used.
NO_POSE_STEPS = [
    ('clock', 100 * NS),
    ('scan', FAR, 'laser_link'),
    ('cmd', CMD, STOP, 'no pose: a scan in laser_link with nothing near is not used: zero (R8)'),
    ('log', "'scan.roslean_sensor_pose' is set", 0, 0, 'no pose: no start-up warning about a pose'),
    ('log', "Scan is in frame 'laser_link', not in the base frame 'base_link', and", 1, 3,
     'no pose: error that the scan is not used'),
]

# Review CM3/CM4, finding 1, the reviewer's case, on the shipped example
# configuration (config/example.yaml: base frame base_footprint, stop box
# x in [0, 0.3] m with 4 points, slowdown zones behind it, timeouts 5 s and
# 2 s). The scan is in `laser_link`, and the sensor is in truth mounted
# backwards. With the example's former `roslean_sensor_pose: [0.0, 0.0, 0.0]`
# an obstacle 0.25 m ahead of the robot (behind the sensor) was passed with
# the 0.3 slowdown instead of a stop. Now the scan is not used, and the robot
# stops whatever the scan shows.
EXAMPLE_STEPS = [
    ('clock', 100 * NS),
    ('scan', [0.25] * 5, 'laser_link', *BEHIND),
    ('cmd', CMD, STOP, 'example.yaml: obstacle ahead of the robot, seen by a backwards sensor in '
     'laser_link: zero'),
    ('scan', [0.25] * 5, 'laser_link'),
    ('cmd', CMD, STOP, 'example.yaml: obstacle behind the robot, same sensor: zero as well (the scan is not used)'),
    ('scan', FAR, 'laser_link'),
    ('cmd', CMD, STOP, 'example.yaml: nothing near, scan in laser_link: zero'),
    ('scan', FAR, 'base_footprint'),
    ('cmd', CMD, PASS, 'example.yaml: a scan in base_footprint with nothing near: pass'),
    ('scan', [0.25] * 5, 'base_footprint'),
    ('cmd', CMD, STOP, 'example.yaml: five points in the stop box: zero'),
    ('scan', [0.25] * 5, 'base_footprint', *BEHIND),
    ('cmd', CMD, slow(0.3), 'example.yaml: five points behind the robot: the 0.3 slowdown'),
    ('log', "'scan.roslean_sensor_pose' is set", 0, 0, 'example.yaml sets no sensor pose'),
    ('log', "Scan is in frame 'laser_link', not in the base frame 'base_footprint', and", 1, 3,
     'example.yaml: error that the scan in laser_link is not used'),
    ('log', 'protects nothing', 0, 0, 'example.yaml: no warning about a configuration that protects nothing'),
]

# Review CM3/CM4, finding 4f: commands with negative zeros. The comparison is
# bit for bit. A passed command keeps its signs; a slowed-down one has the
# signs of the IEEE products; a stop is +0.0 in all three components.
NEG = (-0.0, 0.25, 1.0, 1.0, 1.0, -0.0)
NEGZERO_STEPS = [
    ('clock', 100 * NS),
    ('scan', FAR),
    ('cmd', NEG, PASS, '-0.0 in linear.x and angular.z passes as -0.0'),
    ('cmd', (-0.0, -0.0, 1.0, 1.0, 1.0, -0.0), PASS, 'a command of negative zeros is published as it is'),
    ('scan', [0.5] * 5),
    ('cmd', NEG, slow(0.5), '-0.0 scaled by 0.5 is -0.0'),
    ('scan', [0.2] * 5),
    ('cmd', (-0.5, -0.0, 1.0, 1.0, 1.0, -0.0), STOP, 'a stop is +0.0 in every component'),
]

# Review CM3/CM4, finding 4f: a scan of realistic size. 720 readings over the
# full circle, reading 360 straight ahead, reading 0 straight behind.
FULL = (-math.pi, 2.0 * math.pi / 720.0)


def full_scan(near=(), value=0.2):
    ranges = [5.0] * 720
    for k in near:
        ranges[k] = value
    return ranges


FULL_STEPS = [
    ('clock', 100 * NS),
    ('scan', full_scan(), 'base_link', *FULL),
    ('cmd', CMD, PASS, '720 readings, nothing near: pass'),
    ('scan', full_scan(range(358, 363)), 'base_link', *FULL),
    ('cmd', CMD, STOP, '720 readings, five of them 0.2 m ahead: stop'),
    ('scan', full_scan(range(359, 361)), 'base_link', *FULL),
    ('cmd', CMD, PASS, '720 readings, two ahead: below min_points 3, pass'),
    ('scan', full_scan(range(0, 5)), 'base_link', *FULL),
    ('cmd', CMD, slow(0.5), '720 readings, five of them 0.2 m behind: outside the stop box, slowdown 0.5'),
    ('scan', full_scan(range(178, 183), 0.8), 'base_link', *FULL),
    ('cmd', CMD, slow(0.75), '720 readings, five at 0.8 m to the right: slowdown 0.75'),
    ('scan', full_scan(range(0, 720), math.inf), 'base_link', *FULL),
    ('cmd', CMD, PASS, '720 infinite readings: pass'),
]

# Review CM3/CM4, finding 2: parameters are read once (D2). A change through
# the parameter service is rejected, with a reason that says so, and a name
# that was not given at start-up cannot be set. The node starts with the stop
# zone disabled; were the change accepted and ignored (as before the fix),
# the operator would believe the zone to be on while commands still pass.
DYNAMIC_PARAMS = ['Stop.enabled:=false']
READ_ONCE = 'read once'
# Each change of the action is logged once at INFO (WARN for an invalid
# source), in upstream's words, naming the zone. The steps are the first ones
# of STEPS plus repetitions, which must not be logged again.
ACTION_LOG_STEPS = [
    ('clock', 100 * NS),
    ('cmd', CMD, STOP, 'no scan yet: zero'),
    ('cmd', CMD, STOP, 'again'),
    ('log', 'Robot to stop due to invalid source', 1, 1, 'logged once: stop due to invalid source'),
    ('scan', FAR),
    ('cmd', CMD, PASS, 'pass'),
    ('cmd', CMD, PASS, 'again'),
    ('log', 'Robot to continue normal operation', 1, 1, 'logged once: normal operation'),
    ('scan', [0.8] * 5),
    ('cmd', CMD, slow(0.75), 'slowdown 0.75'),
    ('log', 'Robot to slowdown for 75.000000 percents due to SlowWide polygon', 1, 1,
     'logged once: slowdown, with ratio and zone name'),
    ('scan', [0.5] * 5),
    ('cmd', CMD, slow(0.5), 'slowdown 0.5 by another zone'),
    ('log', 'Robot to slowdown for 50.000000 percents due to Slow polygon', 1, 1,
     'logged: the change from one slowdown zone to another'),
    ('scan', [0.2] * 5),
    ('cmd', CMD, STOP, 'stop zone'),
    ('cmd', CMD, STOP, 'again'),
    ('log', 'Robot to stop due to Stop polygon', 1, 1, 'logged once: stop, with the zone name'),
    ('scan', FAR),
    ('cmd', CMD, PASS, 'pass again'),
    ('log', 'Robot to continue normal operation', 2, 2, 'logged a second time after the change back'),
]

DYNAMIC_STEPS = [
    ('clock', 100 * NS),
    ('scan', [0.2] * 5),
    ('cmd', CMD, slow(0.5), 'stop zone disabled at start-up: points in it only slow the robot down'),
    ('set', 'Stop.enabled', True, READ_ONCE, 'setting Stop.enabled to true is rejected: "read once"'),
    ('get', 'Stop.enabled', False, 'and the parameter still reads false'),
    ('cmd', CMD, slow(0.5), 'the node behaves as it reports: still the slowdown'),
    ('set', 'Stop.enabled', False, READ_ONCE, 'setting a parameter to the value it has is rejected too'),
    ('set', 'source_timeout', 0.0, READ_ONCE, 'setting source_timeout to 0 is rejected'),
    ('set', 'Slow.slowdown_ratio', 1.0, READ_ONCE, 'setting a slowdown ratio is rejected'),
    ('set', 'use_sim_time', False, READ_ONCE, 'setting use_sim_time is rejected'),
    ('set_atomically', 'Stop.enabled', True, READ_ONCE, 'set_parameters_atomically is rejected'),
    ('set', 'Stopp.enabled', True, 'declared', 'a misspelt name cannot be set: it was not declared'),
    ('set', 'Slow.enabled', False, 'declared',
     'nor can a parameter that exists upstream but was not given at start-up'),
    ('cmd', CMD, slow(0.5), 'after all of them: unchanged'),
    ('clock', 103 * NS),
    ('cmd', CMD, STOP, 'and source_timeout is still 2 s: the scan is too old at 103 s'),
]

# Parameters of a minimal configuration the node accepts.
BASE = {
    'base_shift_correction': 'false',
    'observation_sources': '[scan]',
    'polygons': '[Z]',
    'Z.type': 'circle',
    'Z.radius': '0.5',
    'Z.action_type': 'stop',
}
POLYGON = {'Z.type': 'polygon', 'Z.radius': None}

# Configurations the node must refuse: (label, changes to BASE, text that the
# error message must contain). A value of None removes the parameter.
BAD_CONFIGS = [
    ('base_shift_correction not set (D11)', {'base_shift_correction': None}, 'base_shift_correction'),
    ('base_shift_correction true', {'base_shift_correction': 'true'}, 'base_shift_correction'),
    ('two sources', {'observation_sources': '[scan, pointcloud]'}, 'observation_sources'),
    ('no observation_sources', {'observation_sources': None}, 'observation_sources'),
    ('point cloud source', {'scan.type': 'pointcloud'}, 'scan.type'),
    ('no polygons', {'polygons': None}, 'polygons'),
    ('action approach', {'Z.action_type': 'approach'}, 'Z.action_type'),
    ('action limit', {'Z.action_type': 'limit'}, 'Z.action_type'),
    ('action none', {'Z.action_type': 'none'}, 'Z.action_type'),
    ('no action_type', {'Z.action_type': None}, 'Z.action_type'),
    ('velocity polygon', {'Z.type': 'velocity_polygon'}, 'Z.type'),
    ('no type', {'Z.type': None}, 'Z.type'),
    ('circle without radius', {'Z.radius': None}, 'Z.radius'),
    ('radius of 2000 m (D12)', {'Z.radius': '2000.0'}, 'Z.radius'),
    ('infinite radius', {'Z.radius': '.inf'}, 'Z.radius'),
    ('polygon without points', POLYGON, 'Z.points'),
    ('polygon with three vertices (R10)',
     {**POLYGON, 'Z.points': '"[[1, 1], [1, -1], [-1, -1]]"'}, 'Z.points'),
    ('polygon with a malformed point',
     {**POLYGON, 'Z.points': '"[[1, 1], [1, -1], [-1, -1], [-1]]"'}, 'Z.points'),
    ('polygon vertex at 1000 m (D12)',
     {**POLYGON, 'Z.points': '"[[1000, 1], [1, -1], [-1, -1], [-1, 1]]"'}, 'Z.points'),
    ('negative slowdown ratio (D5)',
     {'Z.action_type': 'slowdown', 'Z.slowdown_ratio': '-0.5'}, 'Z.slowdown_ratio'),
    ('max_points (D10)', {'Z.max_points': '3'}, 'Z.max_points'),
    ('min_points beyond 32 bits (D10)', {'Z.min_points': '4294967297'}, 'Z.min_points'),
    ('NaN radius', {'Z.radius': '.nan'}, 'Z.radius'),
    ('polygon with an infinite coordinate',
     {**POLYGON, 'Z.points': '"[[1, 1], [1, -1], [-1, -1], [-1, 1e39]]"'}, 'Z.points'),
    ('huge source_timeout (D9)', {'source_timeout': '1.0e10'}, 'source_timeout'),
    ('zone bound to another source', {'Z.sources_names': '[other]'}, 'Z.sources_names'),
    ('state topic (D13)', {'state_topic': 'collision_monitor_state'}, 'state_topic'),
    ('real-time priority', {'use_realtime_priority': 'true'}, 'use_realtime_priority'),
    ('negative source_timeout (D9)', {'source_timeout': '-1.0'}, 'source_timeout'),
    ('negative timeout of the source', {'scan.source_timeout': '-1.0'}, 'scan.source_timeout'),
    ('NaN stop_pub_timeout (D9)', {'stop_pub_timeout': '.nan'}, 'stop_pub_timeout'),
    ('integer source_timeout (R17)', {'source_timeout': '2'}, 'source_timeout'),
    ('string min_points (R17)', {'Z.min_points': 'four'}, 'Z.min_points'),
    ('sensor pose with two numbers', {'scan.roslean_sensor_pose': '[0.0, 0.0]'},
     'scan.roslean_sensor_pose'),
]

# Review CM3/CM4, finding 7a: the message for a refused radius gives the true
# bound.
RADIUS_CASES = [
    ('999.9995', False), ('999.999', False), ('-999.999', False), ('999.9989', True),
    ('-999.9989', True),
]

# Review CM3/CM4, finding 7b: every refusal in one message. A parameter file
# in Nav2's layout with seven things wrong, some refused by the shim and some
# by the core. (The core gives one reason per zone: for B that is min_points,
# not the three vertices; for A the radius, not the ratio.)
MANY_WRONG = """
collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    state_topic: "collision_monitor_state"
    source_timeout: -1.0
    stop_pub_timeout: .nan
    polygons: ["A", "B", "C", "D"]
    A:
      type: "circle"
      radius: 2000.0
      action_type: "slowdown"
      slowdown_ratio: -0.5
    B:
      type: "polygon"
      points: "[[1, 1], [1, -1], [-1, -1]]"
      action_type: "stop"
      min_points: 4294967297
    C:
      type: "circle"
      radius: 0.3
      action_type: "approach"
      max_points: 3
    D:
      type: "polygon"
      points: "[[1, 1], [1, -1], [-1, -1], [-1]]"
      action_type: "stop"
    observation_sources: ["scan"]
    scan:
      type: "scan"
      source_timeout: -2.0
      roslean_sensor_pose: [0.0, 0.0]
"""
MANY_WRONG_NAMES = [
    'state_topic', 'base_shift_correction', 'source_timeout', 'scan.source_timeout',
    'stop_pub_timeout', 'A.radius', 'B.min_points', 'C.action_type', 'C.max_points', 'D.points',
    'scan.roslean_sensor_pose',
]

# Configurations the node must accept: (label, changes to BASE, texts that a
# warning must contain; an empty list means that no warning about a
# configuration that protects nothing may appear).
PROTECTS_NOTHING = 'protects nothing'
GOOD_CONFIGS = [
    ('the minimal configuration', {}, []),
    ('ignored parameters, zero timeouts, odd but legal values',
     {'base_frame_id': 'base_link', 'transform_tolerance': '0.5', 'Z.visualize': 'true',
      'Z.polygon_pub_topic': 'zone', 'source_timeout': '0.0', 'stop_pub_timeout': '0.0',
      'Z.min_points': '-1', 'Z.radius': '-0.5', 'Z.action_type': 'slowdown',
      'Z.slowdown_ratio': '7.0', 'Z.sources_names': '[scan]', 'state_topic': '""',
      'scan.roslean_sensor_pose': '[0.1, 0.0, 3.14]', 'scan.enabled': 'false'},
     ["'scan.enabled' is false", "'min_points' is -1", "'slowdown_ratio' is 7.0"]),
    # Review CM3/CM4, finding 7c: accepted, with a warning for each.
    ('source_timeout 0.0', {'source_timeout': '0.0'}, ['the source timeout is 0']),
    ("the source's own timeout 0.0", {'scan.source_timeout': '0.0'}, ['the source timeout is 0']),
    ('a non-zero own timeout over a zero node timeout',
     {'source_timeout': '0.0', 'scan.source_timeout': '1.0'}, []),
    ('a disabled source', {'scan.enabled': 'false'}, ["'scan.enabled' is false"]),
    ('min_points 0', {'Z.min_points': '0'}, ["zone 'Z': 'min_points' is 0"]),
    ('min_points 1', {'Z.min_points': '1'}, []),
    ('slowdown ratio 1.5', {'Z.action_type': 'slowdown', 'Z.slowdown_ratio': '1.5'},
     ["zone 'Z': 'slowdown_ratio' is 1.5"]),
    ('slowdown ratio 1.0', {'Z.action_type': 'slowdown', 'Z.slowdown_ratio': '1.0'},
     ["zone 'Z': 'slowdown_ratio' is 1.0"]),
    ('slowdown ratio NaN', {'Z.action_type': 'slowdown', 'Z.slowdown_ratio': '.nan'},
     ["zone 'Z': 'slowdown_ratio' is nan"]),
    ('slowdown ratio 0.99', {'Z.action_type': 'slowdown', 'Z.slowdown_ratio': '0.99'}, []),
    ('a ratio of 1.5 on a stop zone, where it is not used', {'Z.slowdown_ratio': '1.5'}, []),
    ('radius 0.0', {'Z.radius': '0.0'}, ["zone 'Z': 'radius' is 0"]),
    ('four collinear vertices', {**POLYGON, 'Z.points': '"[[0, 0], [1, 0], [2, 0], [3, 0]]"'},
     ["zone 'Z': the polygon of 'points' has no area"]),
    ('four vertices of a segment walked twice',
     {**POLYGON, 'Z.points': '"[[0, 0], [1, 1], [0, 0], [1, 1]]"'},
     ["zone 'Z': the polygon of 'points' has no area"]),
    ('a proper square', {**POLYGON, 'Z.points': '"[[1, 1], [1, -1], [-1, -1], [-1, 1]]"'}, []),
]
# Not tested: `polygons: []`. A parameter file or `-p` cannot express an empty
# list (rcl: "No parameter value set"), so the node's warning for a
# configuration without zones cannot be reached from here.


class Logs:
    """Collects the lines a process writes to stderr, in a thread."""

    def __init__(self, proc):
        self.lines = []
        self.thread = threading.Thread(target=self._read, args=(proc.stderr,), daemon=True)
        self.thread.start()

    def _read(self, stream):
        for line in stream:
            self.lines.append(line.rstrip('\n'))

    def count(self, needle):
        return sum(needle in line for line in list(self.lines))

    def wait_for(self, needle, at_least, spin, timeout=5.0):
        end = time.monotonic() + timeout
        while self.count(needle) < at_least and time.monotonic() < end:
            spin(0.02)
        return self.count(needle)


class Driver(Node):

    def __init__(self, stamped, logs):
        super().__init__('smoke_driver')
        self.logs = logs
        self.stamped = stamped
        self.msg_type = TwistStamped if stamped else Twist
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.cmd_pub = self.create_publisher(self.msg_type, 'cmd_vel_smoothed', 10)
        self.received = []
        self.create_subscription(self.msg_type, 'cmd_vel', self.received.append, 10)
        self.now_ns = 0
        self.count = 0
        self.results = []  # (label, ok, what happened)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_connected(self, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (all(p.get_subscription_count() > 0
                    for p in (self.clock_pub, self.scan_pub, self.cmd_pub))
                    and self.count_publishers('cmd_vel') > 0):
                self.spin_for(0.5)
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('node under test did not come up')

    def stamp(self):
        sec, nanosec = divmod(self.now_ns, NS)
        return sec, nanosec

    def send_scan(self, ranges, frame='base_link', angle_min=-0.02, increment=0.01):
        msg = LaserScan()
        msg.header.frame_id = frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.stamp()
        msg.angle_min = angle_min
        msg.angle_increment = increment
        msg.angle_max = angle_min + increment * max(len(ranges) - 1, 0)
        msg.range_min = 0.05
        msg.range_max = 10.0
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def send_cmd(self, values):
        msg = self.msg_type()
        twist = msg.twist if self.stamped else msg
        (twist.linear.x, twist.linear.y, twist.linear.z,
         twist.angular.x, twist.angular.y, twist.angular.z) = values
        self.count += 1
        if self.stamped:
            msg.header.frame_id = f'frame_{self.count}'
            msg.header.stamp.sec, msg.header.stamp.nanosec = 7, self.count
        self.cmd_pub.publish(msg)
        return msg

    def judge(self, sent, values, expected, unanswered_before):
        """Compares what came out for one command with the expectation."""
        kind, ratio = expected
        if kind == 'nothing':
            # Silence must be the node's answer to this command, not a lost
            # message: the node logs each command it leaves unanswered.
            unanswered = self.logs.wait_for(
                NOT_ANSWERED, unanswered_before + 1, self.spin_for) - unanswered_before
            got = self.received[:]
            self.received.clear()
            return (not got and unanswered == 1), (
                f'{len(got)} message(s); the node logged {unanswered} unanswered command(s)')
        got = self.received[:]
        self.received.clear()
        if len(got) != 1:
            return False, f'{len(got)} message(s)'
        out = got[0].twist if self.stamped else got[0]
        planar = (out.linear.x, out.linear.y, out.angular.z)
        rest = (out.linear.z, out.angular.x, out.angular.y)
        if kind == 'pass':
            want = (values[0], values[1], values[5])
        elif kind == 'zero':
            want = (0.0, 0.0, 0.0)
        else:
            # The same IEEE double products as the node computes.
            want = (values[0] * ratio, values[1] * ratio, values[5] * ratio)
        # Bit for bit: -0.0 is not 0.0.
        ok = bits(planar) == bits(want) and bits(rest) == bits((0.0, 0.0, 0.0))
        if self.stamped:
            ok = ok and got[0].header == sent.header
        return ok, f'published {planar}, other components {rest}'

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, f'/collision_monitor/{name}')
        try:
            if not client.wait_for_service(timeout_sec=10.0):
                return None
            future = client.call_async(request)
            end = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            return future.result() if future.done() else None
        finally:
            self.destroy_client(client)

    def set_param(self, name, value, atomically):
        """Asks the node to change a parameter; returns (successful, reason)."""
        pv = ParameterValue()
        if isinstance(value, bool):
            pv.type, pv.bool_value = ParameterType.PARAMETER_BOOL, value
        else:
            pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
        params = [Parameter(name=name, value=pv)]
        if atomically:
            res = self.call(SetParametersAtomically, 'set_parameters_atomically',
                            SetParametersAtomically.Request(parameters=params))
            return (None, 'no answer') if res is None else (res.result.successful, res.result.reason)
        res = self.call(SetParameters, 'set_parameters', SetParameters.Request(parameters=params))
        if res is None or len(res.results) != 1:
            return None, 'no answer'
        return res.results[0].successful, res.results[0].reason

    def get_bool(self, name):
        res = self.call(GetParameters, 'get_parameters', GetParameters.Request(names=[name]))
        if res is None or len(res.values) != 1 or res.values[0].type != ParameterType.PARAMETER_BOOL:
            return None
        return res.values[0].bool_value

    def run(self, steps):
        for step in steps:
            if step[0] == 'clock':
                self.now_ns = step[1]
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = self.stamp()
                self.clock_pub.publish(msg)
                self.spin_for(SETTLE)
            elif step[0] == 'scan':
                self.send_scan(*step[1:])
                self.spin_for(SETTLE)
            elif step[0] == 'log':
                _, needle, at_least, at_most, label = step
                n = self.logs.wait_for(needle, at_least, self.spin_for, timeout=2.0)
                self.results.append((label, at_least <= n <= at_most, f'{n} log line(s) with {needle!r}'))
            elif step[0] in ('set', 'set_atomically'):
                _, name, value, needle, label = step
                successful, reason = self.set_param(name, value, step[0] == 'set_atomically')
                self.results.append(
                    (label, successful is False and needle in reason,
                     f'successful={successful}, reason={reason!r}'))
            elif step[0] == 'get':
                _, name, want, label = step
                got = self.get_bool(name)
                self.results.append((label, got is want, f'{name} reads {got}'))
            else:
                _, values, expected, label = step
                self.received.clear()
                unanswered_before = self.logs.count(NOT_ANSWERED)
                sent = self.send_cmd(values)
                self.spin_for(SETTLE)
                ok, what = self.judge(sent, values, expected, unanswered_before)
                self.results.append((label, ok, what))
        self.spin_for(SETTLE)
        if self.received:
            self.results.append(('no late messages', False, f'{len(self.received)} message(s)'))


def bits(values):
    return struct.pack(f'<{len(values)}d', *values)


def stop(proc):
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_scenario(exe, title, steps, stamped=False, params=(), params_file='smoke_params.yaml'):
    args = [exe, '--ros-args', '--params-file', os.path.join(HERE, params_file),
            '-p', 'use_sim_time:=true', '-p', f'enable_stamped_cmd_vel:={str(stamped).lower()}',
            '--log-level', 'collision_monitor:=debug']
    for param in params:
        args += ['-p', param]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    d = Driver(stamped, Logs(proc))
    try:
        d.wait_connected()
        d.run(steps)
    finally:
        stop(proc)
        d.destroy_node()
    bad = 0
    print(f'--- scenario: {title}')
    for label, ok, what in d.results:
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  {label}" + ('' if ok else f'  [{what}]'))
    return bad


def refusal(args):
    """Starts the node with a configuration it must refuse; returns the exit
    code and the fatal lines of its log."""
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        _, err = proc.communicate(timeout=20)
        code = proc.returncode
    except subprocess.TimeoutExpired:
        proc.kill()
        _, err = proc.communicate()
        code = None
    return code, [line for line in err.splitlines() if '[FATAL]' in line]


def param_args(changes):
    params = {**BASE, **changes}
    args = ['--ros-args']
    for name, value in params.items():
        if value is not None:
            args += ['-p', f'{name}:={value}']
    return args


def run_bad_configs(exe, tmp):
    bad = 0
    print('--- configurations the node must refuse')
    for label, changes, needle in BAD_CONFIGS:
        code, fatal = refusal([exe] + param_args(changes))
        ok = code == 1 and len(fatal) == 1 and f"'{needle}'" in fatal[0] and \
            '1 parameter refused' in fatal[0]
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  refuses: {label} (exit code {code})"
              + ('' if ok else f'  [{fatal}]'))

    # Finding 7a: the radius message gives the true bound.
    for value, accepted in RADIUS_CASES:
        if accepted:
            continue
        code, fatal = refusal([exe] + param_args({'Z.radius': value}))
        ok = code == 1 and len(fatal) == 1 and 'at most 999.999 m in magnitude' in fatal[0] \
            and 'below 1000 m' not in fatal[0]
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  refuses radius {value} and gives the bound 999.999 m"
              + ('' if ok else f'  [{fatal}]'))

    # Finding 7b: all refusals in one message, from a parameter file.
    path = os.path.join(tmp, 'many_wrong.yaml')
    with open(path, 'w') as f:
        f.write(MANY_WRONG)
    code, fatal = refusal([exe, '--ros-args', '--params-file', path])
    missing = [n for n in MANY_WRONG_NAMES if len(fatal) != 1 or f"parameter '{n}'" not in fatal[0]]
    ok = code == 1 and len(fatal) == 1 and not missing and \
        f'{len(MANY_WRONG_NAMES)} parameters refused' in fatal[0]
    bad += not ok
    print(f"{'ok  ' if ok else 'FAIL'}  a parameter file with {len(MANY_WRONG_NAMES)} refused "
          'parameters: one fatal message names them all'
          + ('' if ok else f'  [exit code {code}; not named: {missing}; {fatal}]'))
    return bad


def starts(args):
    """Starts the node with a configuration it must accept, lets it run for
    three seconds and stops it. Returns (ok, what went wrong, its log)."""
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        _, err = proc.communicate(timeout=3)
        return False, f'exited with code {proc.returncode}: {err.strip()}', err
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        try:
            _, err = proc.communicate(timeout=10)
            return proc.returncode == 0, f'exit code {proc.returncode} on SIGINT', err
        except subprocess.TimeoutExpired:
            proc.kill()
            _, err = proc.communicate()
            return False, 'did not stop on SIGINT', err


def run_good_configs(exe):
    bad = 0
    print('--- configurations the node must accept, and what it must warn about')
    cases = [(label, [exe] + param_args(changes), warnings)
             for label, changes, warnings in GOOD_CONFIGS]
    for value, accepted in RADIUS_CASES:
        if accepted:
            cases.append((f'radius {value}', [exe] + param_args({'Z.radius': value}), []))
    example = os.path.join(HERE, '..', 'config', 'example.yaml')
    cases.append(('config/example.yaml', [exe, '--ros-args', '--params-file', example], []))
    for label, args, warnings in cases:
        ok, what, err = starts(args)
        warned = [line for line in err.splitlines() if '[WARN]' in line and PROTECTS_NOTHING in line]
        if ok and warnings:
            missing = [w for w in warnings if not any(w in line for line in warned)]
            ok = not missing and len(warned) == len(warnings)
            what = f'expected warnings with {warnings}; got {warned}'
        elif ok:
            ok, what = not warned, f'unexpected warning(s): {warned}'
        bad += not ok
        wants = f', warns {len(warnings)} time(s)' if warnings else ', no "protects nothing" warning'
        print(f"{'ok  ' if ok else 'FAIL'}  starts and stops cleanly: {label}{wants}"
              + ('' if ok else f'  [{what}]'))
    return bad


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    exe = sys.argv[1]
    rclpy.init()
    bad = 0
    try:
        bad += run_scenario(exe, 'Twist', STEPS)
        bad += run_scenario(exe, 'TwistStamped', STEPS, stamped=True)
        bad += run_scenario(exe, 'sensor pose, backwards', POSE_STEPS, params=POSE_PARAMS)
        bad += run_scenario(exe, 'sensor pose of zero', ZERO_POSE_STEPS, params=ZERO_POSE_PARAMS)
        bad += run_scenario(exe, 'no sensor pose', NO_POSE_STEPS)
        bad += run_scenario(exe, 'config/example.yaml, scan in another frame', EXAMPLE_STEPS,
                            params_file=os.path.join('..', 'config', 'example.yaml'))
        bad += run_scenario(exe, 'negative zeros', NEGZERO_STEPS)
        bad += run_scenario(exe, 'negative zeros, TwistStamped', NEGZERO_STEPS, stamped=True)
        bad += run_scenario(exe, 'a scan of 720 readings', FULL_STEPS)
        bad += run_scenario(exe, 'parameter changes at run time', DYNAMIC_STEPS, params=DYNAMIC_PARAMS)
        bad += run_scenario(exe, 'log of action changes', ACTION_LOG_STEPS)
    finally:
        rclpy.shutdown()
    with tempfile.TemporaryDirectory() as tmp:
        bad += run_bad_configs(exe, tmp)
        bad += run_good_configs(exe)
    print(f'{bad} failure(s)')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
