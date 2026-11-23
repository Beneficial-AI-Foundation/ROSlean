#!/usr/bin/env python3
"""Smoke test of the collision monitor node over real ROS topics.

    smoke_node.py NODE

NODE is the path of the `collision_monitor` executable. The node runs with
`use_sim_time` and this script publishes `/clock`, so the node-clock time of
every event is chosen exactly. The script sends scans and velocity commands
and checks what comes out on `cmd_vel`: the command unchanged (pass-through),
zero (stop), the command scaled (slowdown), or nothing. Expectations follow
docs/collision_monitor/SPEC.md. It then checks that the node refuses to start
with configurations outside the supported subset, and names the parameter.

This is a smoke test of the shim, not a comparison with upstream.

The only wall-clock dependence is that messages must be delivered within
SETTLE seconds, and that a clock update or scan reaches the node before the
next message does.
"""

import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan

HERE = os.path.dirname(os.path.abspath(__file__))
SETTLE = 0.2  # s of wall time allowed for delivery
NS = 1_000_000_000

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

# With `scan.roslean_sensor_pose: [0.0, 0.0, pi]` (a sensor mounted backwards)
# a scan in the sensor's frame is used, and what the sensor sees ahead is
# behind the robot: outside the stop box, inside the 0.6 m circle.
POSE_PARAMS = ['scan.roslean_sensor_pose:=[0.0, 0.0, 3.141592653589793]']
POSE_STEPS = [
    ('clock', 100 * NS),
    ('scan', [0.2] * 5, 'laser_link'),
    ('cmd', CMD, slow(0.5), 'sensor pose set: 0.2 m "ahead" of a backwards sensor is behind the robot'),
    ('scan', FAR, 'laser_link'),
    ('cmd', CMD, PASS, 'sensor pose set: a scan in the sensor frame is used'),
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

# Configurations the node must accept: (label, changes to BASE).
GOOD_CONFIGS = [
    ('the minimal configuration', {}),
    ('ignored parameters, zero timeouts, odd but legal values',
     {'base_frame_id': 'base_link', 'transform_tolerance': '0.5', 'Z.visualize': 'true',
      'Z.polygon_pub_topic': 'zone', 'source_timeout': '0.0', 'stop_pub_timeout': '0.0',
      'Z.min_points': '-1', 'Z.radius': '-0.5', 'Z.action_type': 'slowdown',
      'Z.slowdown_ratio': '7.0', 'Z.sources_names': '[scan]', 'state_topic': '""',
      'scan.roslean_sensor_pose': '[0.1, 0.0, 3.14]', 'scan.enabled': 'false'}),
]


class Driver(Node):

    def __init__(self, stamped):
        super().__init__('smoke_driver')
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

    def send_scan(self, ranges, frame='base_link'):
        msg = LaserScan()
        msg.header.frame_id = frame
        msg.header.stamp.sec, msg.header.stamp.nanosec = self.stamp()
        msg.angle_min = -0.02
        msg.angle_increment = 0.01
        msg.angle_max = 0.02
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

    def judge(self, sent, values, expected):
        """Compares what came out for one command with the expectation."""
        kind, ratio = expected
        got = self.received[:]
        self.received.clear()
        if kind == 'nothing':
            return (not got), f'{len(got)} message(s)'
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
        ok = planar == want and rest == (0.0, 0.0, 0.0)
        if self.stamped:
            ok = ok and got[0].header == sent.header
        return ok, f'published {planar}, other components {rest}'

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
            else:
                _, values, expected, label = step
                self.received.clear()
                sent = self.send_cmd(values)
                self.spin_for(SETTLE)
                ok, what = self.judge(sent, values, expected)
                self.results.append((label, ok, what))
        self.spin_for(SETTLE)
        if self.received:
            self.results.append(('no late messages', False, f'{len(self.received)} message(s)'))


def stop(proc):
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def run_scenario(exe, title, steps, stamped=False, params=()):
    args = [exe, '--ros-args', '--params-file', os.path.join(HERE, 'smoke_params.yaml'),
            '-p', 'use_sim_time:=true', '-p', f'enable_stamped_cmd_vel:={str(stamped).lower()}']
    for param in params:
        args += ['-p', param]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    d = Driver(stamped)
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


def param_args(changes):
    params = {**BASE, **changes}
    args = ['--ros-args']
    for name, value in params.items():
        if value is not None:
            args += ['-p', f'{name}:={value}']
    return args


def run_bad_configs(exe):
    bad = 0
    print('--- configurations the node must refuse')
    for label, changes, needle in BAD_CONFIGS:
        proc = subprocess.Popen([exe] + param_args(changes), stdout=subprocess.DEVNULL,
                                stderr=subprocess.PIPE, text=True)
        try:
            _, err = proc.communicate(timeout=20)
            code = proc.returncode
        except subprocess.TimeoutExpired:
            proc.kill()
            _, err = proc.communicate()
            code = None
        fatal = [line for line in err.splitlines() if '[FATAL]' in line]
        ok = code == 1 and any(f"'{needle}'" in line for line in fatal)
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  refuses: {label} (exit code {code})"
              + ('' if ok else f'  [{fatal}]'))
    return bad


def run_good_configs(exe):
    bad = 0
    print('--- configurations the node must accept')
    cases = [(label, param_args(changes)) for label, changes in GOOD_CONFIGS]
    example = os.path.join(HERE, '..', 'config', 'example.yaml')
    cases.append(('config/example.yaml', ['--ros-args', '--params-file', example]))
    for label, args in cases:
        proc = subprocess.Popen([exe] + args, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                                text=True)
        try:
            _, err = proc.communicate(timeout=3)
            ok, what = False, f'exited with code {proc.returncode}: {err.strip()}'
        except subprocess.TimeoutExpired:
            ok, what = True, ''
            proc.send_signal(signal.SIGINT)
            try:
                proc.communicate(timeout=10)
                ok, what = proc.returncode == 0, f'exit code {proc.returncode} on SIGINT'
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()
                ok, what = False, 'did not stop on SIGINT'
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  starts and stops cleanly: {label}"
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
        bad += run_scenario(exe, 'sensor pose', POSE_STEPS, params=POSE_PARAMS)
    finally:
        rclpy.shutdown()
    bad += run_bad_configs(exe)
    bad += run_good_configs(exe)
    print(f'{bad} failure(s)')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
