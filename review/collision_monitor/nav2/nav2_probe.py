#!/usr/bin/env python3
"""T04: probes of the upstream nav2_collision_monitor binary for the Nav2 extension.

What upstream does for the three pieces the extension adds: the lifecycle
interface with the state topic and the polygon publishers, transforms with
base shift correction, and zones whose shape arrives on a topic. The results
are quoted in docs/collision_monitor/SPEC_NAV2.md; a run on macOS arm64 is in
results/macos-arm64.txt beside this file.

Each case starts a fresh `collision_monitor` process with its own parameter
file. Except for the two bond cases the node runs under simulated time and
this script publishes /clock, so every node-clock reading is chosen exactly.
One event is sent at a time and the script waits SETTLE seconds of wall time
for the answer; that wait is the only wall-clock assumption.

    run_nav2_probe.sh <env prefix> [case ...]      (no case: all of them)
    run_nav2_probe.sh <env prefix> --list

Nothing here knows what the node should do: the script prints, it does not
assert.
"""
import math
import os
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
import yaml
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

NS = 1_000_000_000
SETTLE = 0.3
PREFIX = os.environ.get('CM_PREFIX', '')
NODE = 'collision_monitor'
T0 = 100 * NS
SQ = math.sqrt(0.5)

HEAD = """
collision_monitor:
  ros__parameters:
    use_sim_time: {sim}
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_in"
    cmd_vel_out_topic: "cmd_vel_out"
    state_topic: "cm_state"
    transform_tolerance: {tolerance}
    source_timeout: {source_timeout}
    base_shift_correction: {shift}
    stop_pub_timeout: {stop_pub_timeout}
    bond_heartbeat_period: {bond}
    polygons: [{names}]
{zones}
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
"""

STOP_CIRCLE = """    StopCircle:
      type: "circle"
      radius: 0.5
      action_type: "stop"
      min_points: 1
      visualize: {visualize}
"""
SLOW_CIRCLE = """    SlowCircle:
      type: "circle"
      radius: 1.0
      action_type: "slowdown"
      slowdown_ratio: 0.5
      min_points: 1
"""
FRONT_BOX = """    FrontBox:
      type: "polygon"
      points: "[[0.5, 0.3], [0.5, -0.3], [0.0, -0.3], [0.0, 0.3]]"
      action_type: "stop"
      min_points: 1
      visualize: {visualize}
      polygon_pub_topic: "front_box"
"""
RIGHT_BOX = """    RightBox:
      type: "polygon"
      points: "[[0.3, -0.1], [0.3, -0.5], [-0.3, -0.5], [-0.3, -0.1]]"
      action_type: "stop"
      min_points: 1
"""
TOPIC_ZONE = """    Dyn:
      type: "polygon"
      action_type: "{action}"
      polygon_sub_topic: "zone_shape"
      min_points: 1
      slowdown_ratio: 0.5
      visualize: {visualize}
{extra}"""


def params(names, zones, shift='False', tolerance=0.5, source_timeout=5.0,
           stop_pub_timeout=1.0, bond=0.0, sim='True'):
    return HEAD.format(names=', '.join('"%s"' % n for n in names), zones=zones, shift=shift,
                       tolerance=tolerance, source_timeout=source_timeout,
                       stop_pub_timeout=stop_pub_timeout, bond=bond, sim=sim)


def tf(parent, child, x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0), stamp=0):
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = divmod(stamp, NS)
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in q)
    return t


class Probe(Node):
    def __init__(self, cmd_topic='cmd_vel_in', out_topic='cmd_vel_out', state_topic='cm_state'):
        super().__init__('cm_nav2_probe')
        self.out, self.states, self.polys, self.markers = [], [], [], []
        self.proc = self.log_path = None
        self.now_ns = 0
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.shape_pubs = {}
        self.create_subscription(Twist, out_topic, self.out.append, 10)
        self.create_subscription(CollisionMonitorState, state_topic, self.states.append, 10)
        self.mark = (0, 0, 0)

    # -- plumbing -------------------------------------------------------------

    def spin(self, seconds=SETTLE):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_until(self, cond, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not cond():
            rclpy.spin_once(self, timeout_sec=0.01)
        return cond()

    def watch_polygon(self, topic):
        self.create_subscription(
            PolygonStamped, topic, lambda m, t=topic: self.polys.append((t, m)), 10)

    def watch_markers(self):
        self.create_subscription(
            MarkerArray, f'/{NODE}/collision_points_marker', self.markers.append, 10)

    def start(self, text, exe='nav2_collision_monitor/collision_monitor', extra=()):
        d = tempfile.mkdtemp(prefix='cm_nav2_probe_')
        path = os.path.join(d, 'params.yaml')
        with open(path, 'w') as f:
            f.write(text)
        self.log_path = os.path.join(d, 'node.log')
        pkg, name = exe.split('/')
        args = [os.path.join(PREFIX, 'lib', pkg, name), '--ros-args', '--params-file', path]
        for p in extra:
            args += ['-p', p]
        self.log_file = open(self.log_path, 'w')
        self.proc = subprocess.Popen(args, stdout=self.log_file, stderr=self.log_file)
        self.change = self.create_client(ChangeState, f'/{NODE}/change_state')
        self.get = self.create_client(GetState, f'/{NODE}/get_state')
        if not self.change.wait_for_service(timeout_sec=20.0):
            raise RuntimeError('no change_state service')
        self.spin(0.5)

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.proc:
            self.log_file.close()

    def node_log(self, *needles, limit=12):
        """Lines of the node's log that contain one of the needles (deduplicated)."""
        self.log_file.flush()
        seen = []
        with open(self.log_path) as f:
            for line in f:
                line = line.rstrip()
                if any(n in line for n in needles):
                    # drop the time stamp so that repeated lines collapse
                    key = line.split(']', 2)[-1] if line.startswith('[') else line
                    if key not in [k for k, _ in seen]:
                        seen.append((key, 1))
                    else:
                        seen = [(k, c + 1 if k == key else c) for k, c in seen]
        for key, count in seen[:limit]:
            print(f'      log x{count}:{key}')

    def transition(self, tid, label):
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.change.call_async(req)
        self.spin_until(fut.done, 30.0)
        ok = fut.result().success if fut.done() and fut.result() is not None else None
        self.spin(0.3)
        print(f'  {label}: success={ok}, state now {self.state()}')
        return ok

    def configure(self):
        return self.transition(Transition.TRANSITION_CONFIGURE, 'configure')

    def activate(self):
        return self.transition(Transition.TRANSITION_ACTIVATE, 'activate')

    def deactivate(self):
        return self.transition(Transition.TRANSITION_DEACTIVATE, 'deactivate')

    def cleanup(self):
        return self.transition(Transition.TRANSITION_CLEANUP, 'cleanup')

    def state(self):
        fut = self.get.call_async(GetState.Request())
        self.spin_until(fut.done, 10.0)
        r = fut.result() if fut.done() else None
        return f'{r.current_state.id} ({r.current_state.label})' if r else None

    def up(self):
        self.configure()
        self.activate()
        self.spin_until(lambda: self.cmd_pub.get_subscription_count() == 1
                        and self.scan_pub.get_subscription_count() == 1, 10.0)
        self.spin(0.3)

    def interfaces(self, label):
        self.spin(0.5)
        pubs = sorted(t for t, _ in self.get_publisher_names_and_types_by_node(NODE, '/'))
        subs = sorted(t for t, _ in self.get_subscriber_names_and_types_by_node(NODE, '/'))
        srvs = sorted(s for s, _ in self.get_service_names_and_types_by_node(NODE, '/')
                      if 'parameter' not in s)
        print(f'  {label}:')
        print(f'      publishers:    {pubs}')
        print(f'      subscriptions: {subs}')
        print(f'      services:      {srvs}')

    # -- inputs ---------------------------------------------------------------

    def clock(self, t_ns):
        self.now_ns = t_ns
        m = Clock()
        m.clock.sec, m.clock.nanosec = divmod(t_ns, NS)
        self.clock_pub.publish(m)
        self.spin()

    def scan(self, ranges, angle_min=0.0, inc=0.01, frame='base_footprint', stamp=None):
        m = LaserScan()
        stamp = self.now_ns if stamp is None else stamp
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, NS)
        m.header.frame_id = frame
        m.angle_min, m.angle_increment = float(angle_min), float(inc)
        m.range_min, m.range_max = 0.01, 100.0
        m.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(m)
        self.spin()

    def static(self, *transforms):
        self.tf_static_pub.publish(TFMessage(transforms=list(transforms)))
        self.spin()

    def dynamic(self, *transforms):
        self.tf_pub.publish(TFMessage(transforms=list(transforms)))
        self.spin()

    def shape(self, pts, frame='base_footprint', topic='zone_shape', stamp=None, latched=False):
        if topic not in self.shape_pubs:
            qos = QoSProfile(depth=10)
            if latched:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.shape_pubs[topic] = self.create_publisher(PolygonStamped, topic, qos)
            self.spin(0.5)
        m = PolygonStamped()
        stamp = self.now_ns if stamp is None else stamp
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, NS)
        m.header.frame_id = frame
        m.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in pts]
        self.shape_pubs[topic].publish(m)
        self.spin()

    # -- outputs --------------------------------------------------------------

    def _since(self):
        o, s, p = self.mark
        out = [(m.linear.x, m.linear.y, m.angular.z) for m in self.out[o:]]
        states = [(m.polygon_name, m.action_type) for m in self.states[s:]]
        polys = self.polys[p:]
        self.mark = (len(self.out), len(self.states), len(self.polys))
        text = f'velocity {out or "none"}, state {states or "none"}'
        if polys:
            text += ', polygons ' + str([
                (t, m.header.frame_id, m.header.stamp.sec * NS + m.header.stamp.nanosec,
                 len(m.polygon.points)) for t, m in polys])
        return text

    def cmd(self, label, x=0.5, y=0.0, w=0.0, z=0.0):
        self._since()
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z, m.linear.z = float(x), float(y), float(w), float(z)
        self.cmd_pub.publish(m)
        self.spin()
        print(f'  cmd ({x}, {y}, {w}) at {self.now_ns / NS:.9f} s [{label}]: {self._since()}')

    def since(self, label):
        self.spin()
        print(f'  ... {label}: {self._since()}')


# -- cases ---------------------------------------------------------------------

def case_lifecycle(p):
    """Interfaces in each lifecycle state; commands and scans in each state."""
    p.start(params(['StopCircle', 'FrontBox'],
                   STOP_CIRCLE.format(visualize='True') + FRONT_BOX.format(visualize='True')))
    p.watch_polygon('front_box')
    p.watch_polygon('StopCircle')
    p.interfaces('unconfigured')
    p.clock(T0)
    p.scan([0.2])
    p.cmd('unconfigured, obstacle at 0.2 m')
    p.configure()
    p.interfaces('inactive')
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1
                 and p.scan_pub.get_subscription_count() == 1, 10.0)
    p.cmd('inactive, no scan sent since configure')
    p.scan([0.2])
    p.cmd('inactive, obstacle at 0.2 m stored')
    p.cmd('inactive, a NaN command', x=float('nan'))
    p._since()
    p.activate()
    p.since('published by the transition to active')
    p.interfaces('active')
    p.cmd('active, the scan was received while inactive')
    p.cmd('active, again')
    p.clock(T0 + NS)
    p.cmd('1 s after the first zero: still published')
    p.clock(T0 + NS + 1)
    p.cmd('1 s + 1 ns after the first zero: withheld')
    p.deactivate()
    p.since('published by the transition to inactive')
    p.cmd('inactive again')
    p.activate()
    p.since('published by the second activation')
    p.cmd('active again, same clock, same scan: zero and state are published anew')
    p.clock(T0 + 2 * NS)
    p.cmd('1 s - 1 ns after re-activation: published (the stop stamp was renewed)')
    p.clock(T0 + 2 * NS + 2)
    p.cmd('1 s + 1 ns after the zero that followed re-activation: withheld')
    p.deactivate()
    p.cleanup()
    p.interfaces('unconfigured after cleanup')
    p.configure()
    p.activate()
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1, 10.0)
    p.spin(0.5)
    p.cmd('after cleanup, configure, activate: the stored scan is gone')
    p.scan([2.0])
    p.cmd('a clear scan')
    p.transition(Transition.TRANSITION_ACTIVE_SHUTDOWN, 'shutdown from active')
    p.interfaces('finalized')
    p.cmd('finalized (shut down from active)')
    p.node_log('Configuring', 'Activating', 'Deactivating', 'Cleaning', 'Shutting', 'bond')


def case_state(p):
    """Every state message, also while the velocity is withheld."""
    p.start(params(['StopCircle', 'SlowCircle'],
                   STOP_CIRCLE.format(visualize='False') + SLOW_CIRCLE, stop_pub_timeout=1.0))
    p.up()
    p.clock(T0)
    p.scan([2.0])
    p.cmd('clear: first command, passes')
    p.scan([0.7])
    p.cmd('point in the slowdown circle')
    p.cmd('same')
    p.scan([0.2])
    p.cmd('point in both: stop')
    p.clock(T0 + 2 * NS)
    p.cmd('2 s later, still stop: velocity withheld')
    p.scan([0.7], stamp=T0 + 2 * NS)
    p.cmd('slowdown, non-zero command: velocity returns')
    p.scan([0.2])
    p.cmd('stop again', x=0.5)
    p.clock(T0 + 4 * NS)
    p.cmd('withheld')
    p.scan([0.7])
    p.cmd('zero command in the slowdown zone while withheld', x=0.0)
    p.scan([2.0])
    p.cmd('zero command, clear, while withheld', x=0.0)
    p.clock(T0 + 20 * NS)
    p.cmd('scan now 16 s old: invalid source while withheld', x=0.0)
    p.cmd('same, non-zero command', x=0.5)
    p.cmd('a NaN command', x=float('nan'))
    info = p.get_publishers_info_by_topic('/cm_state')
    for i in info:
        print(f'  state topic: type {i.topic_type}, reliability {i.qos_profile.reliability.name}, '
              f'durability {i.qos_profile.durability.name}, depth {i.qos_profile.depth}')


def case_visualize(p):
    """The polygon publishers and the collision point markers."""
    p.start(params(['StopCircle', 'FrontBox'],
                   STOP_CIRCLE.format(visualize='True') + FRONT_BOX.format(visualize='True'),
                   stop_pub_timeout=1.0))
    p.watch_polygon('front_box')
    p.watch_polygon('StopCircle')
    p.clock(T0)
    p.configure()
    p.since('after configure')
    p.activate()
    p.since('after activate')
    for t, m in p.polys:
        pts = [(q.x, q.y, q.z) for q in m.polygon.points]
        print(f'      {t}: frame {m.header.frame_id!r}, stamp '
              f'{m.header.stamp.sec * NS + m.header.stamp.nanosec}, {len(pts)} points, '
              f'first four {pts[:4]}')
    for topic in ('/front_box', '/StopCircle'):
        for i in p.get_publishers_info_by_topic(topic):
            print(f'  {topic}: type {i.topic_type}, reliability {i.qos_profile.reliability.name}, '
                  f'durability {i.qos_profile.durability.name}')
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1, 10.0)
    p.clock(T0 + NS)
    p.scan([0.2, 0.2, 2.0])
    p.cmd('stop')
    p.cmd('a NaN command', x=float('nan'))
    p.clock(T0 + 5 * NS)
    p.cmd('withheld velocity')
    p.watch_markers()
    p.spin(1.0)
    p.cmd('with a subscriber on collision_points_marker')
    for m in p.markers[-1:]:
        for k in m.markers:
            print(f'      marker ns {k.ns!r}, frame {k.header.frame_id!r}, type {k.type}, '
                  f'points {[(round(q.x, 6), round(q.y, 6)) for q in k.points]}')
    p.deactivate()
    p.since('after deactivate')


def case_tf_latest(p):
    """Without base shift correction the newest transform is used, whatever its stamp."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'),
                   stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    p.dynamic(tf('base_footprint', 'laser', x=1.0, stamp=T0))
    p.scan([0.2], frame='laser')
    p.cmd('sensor 1 m ahead of the base at the scan stamp: point at 1.2 m')
    p.dynamic(tf('base_footprint', 'laser', x=0.0, stamp=T0 + NS // 2))
    p.cmd('a transform stamped 0.5 s after the scan and after the clock says the sensor is '
          'at the base: point at 0.2 m')
    p.clock(T0 + 4 * NS)
    p.cmd('the newest transform is 3.5 s old, the scan 4 s (timeout 5 s)')
    p.dynamic(tf('base_footprint', 'laser', x=1.0, stamp=T0 + NS // 4))
    p.cmd('a transform with a stamp between the two (sensor 1 m ahead) arrives late')
    p.node_log('Failed to get', 'Ignoring')


def case_tf_tilt(p):
    """Roll, pitch and height of the sensor: the 3D transform is projected on the plane."""
    p.start(params(['RightBox'], RIGHT_BOX, stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    half_pi = struct.unpack('f', struct.pack('f', math.pi / 2))[0]
    p.static(tf('base_footprint', 'laser_up'),
             tf('base_footprint', 'laser_high', z=5.0),
             tf('base_footprint', 'laser_flipped', q=(1.0, 0.0, 0.0, 0.0)),
             tf('base_footprint', 'laser_pitched', q=(0.0, SQ, 0.0, SQ)),
             tf('base_footprint', 'laser_double', q=(0.0, 0.0, 0.0, 2.0)),
             tf('base_footprint', 'laser_yaw90', q=(0.0, 0.0, SQ, SQ)))
    p.scan([0.3], angle_min=half_pi, frame='laser_up')
    p.cmd('upright sensor, ray to the left at 0.3 m; the zone is on the right')
    p.scan([0.3], angle_min=half_pi, frame='laser_flipped')
    p.cmd('sensor upside down (roll of half a turn): the ray lands on the right')
    p.scan([0.3], angle_min=-half_pi, frame='laser_high')
    p.cmd('sensor 5 m above the base, ray to the right: height is ignored')
    p.scan([0.3], angle_min=-half_pi, frame='laser_double')
    p.cmd('quaternion (0, 0, 0, 2), not normalised, ray to the right')
    p.scan([0.3], angle_min=0.0, frame='laser_yaw90')
    p.cmd('sensor turned a quarter turn to the left, ray ahead: lands on the left')
    p.scan([0.3], angle_min=math.pi, frame='laser_yaw90')
    p.cmd('same sensor, ray backwards: lands on the right')
    p.scan([3.0, 3.0], angle_min=-half_pi, inc=0.0, frame='laser_pitched')
    p.cmd('sensor pitched a quarter turn, rays to the right at 3 m (y unchanged by the pitch)')
    p.scan([0.3], angle_min=-half_pi, frame='laser_pitched')
    p.cmd('same sensor, ray to the right at 0.3 m')
    p.node_log('Failed to get', 'TF_', 'Ignoring transform')


def case_tf_pitch_circle(p):
    """A pitched sensor: a far point ahead is projected onto the base's origin."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False')))
    p.up()
    p.clock(T0)
    p.static(tf('base_footprint', 'laser_up'),
             tf('base_footprint', 'laser_pitched', q=(0.0, SQ, 0.0, SQ)))
    p.scan([3.0], frame='laser_up')
    p.cmd('upright sensor, point 3 m ahead; stop circle of 0.5 m')
    p.scan([3.0], frame='laser_pitched')
    p.cmd('sensor pitched a quarter turn (looking down or up), point 3 m along its x axis')


def case_tf_missing(p):
    """No transform, and a transform that arrives while the node waits for it."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), tolerance=0.5,
                   stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    p.scan([0.2], frame='laser')
    p.cmd('scan in a frame without transform; tolerance 0.5 s; clock stands still')
    p.spin(1.5)
    p.since('1.5 s of wall time later, clock unchanged')
    p.clock(T0 + NS // 2 - 1)
    p.since('clock advanced by tolerance - 1 ns')
    p.clock(T0 + NS // 2)
    p.since('clock advanced by exactly the tolerance')
    p.cmd('next command at the new clock')
    p.since('before the clock moves')
    p.static(tf('base_footprint', 'laser'))
    p.since('the transform arrives while the node waits; the clock has not moved')
    p.cmd('with the transform present')
    p.scan([0.2], frame='')
    p.cmd('a scan with an empty frame_id')
    p.clock(T0 + NS)
    p.since('after the tolerance')
    p.node_log('Failed to get', 'invalid source', 'Invalid')


def case_tf_missing_timeout0(p):
    """No transform with source_timeout 0 and with transform_tolerance 0."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'),
                   tolerance=0.0, source_timeout=0.0))
    p.up()
    p.clock(T0)
    p.scan([0.2], frame='laser')
    p.cmd('no transform, tolerance 0, source_timeout 0')
    p.static(tf('base_footprint', 'laser'))
    p.cmd('transform present')


def case_shift_straight(p):
    """Base shift correction: the odom frame moves between scan and command."""
    for shift in ('True', 'False'):
        print(f'  -- base_shift_correction: {shift}')
        if shift == 'False':
            p.stop()
            p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), shift=shift,
                           stop_pub_timeout=100.0))
            p.mark = (len(p.out), len(p.states), len(p.polys))
            t0 = T0 + 50 * NS
        else:
            p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), shift=shift,
                           stop_pub_timeout=100.0))
            t0 = T0
        p.up()
        p.clock(t0)
        p.cmd('no scan yet')
        p.dynamic(tf('odom', 'base_footprint', x=0.0, stamp=t0))
        p.scan([1.2], stamp=t0)
        p.cmd('scan stamped now, point 1.2 m ahead; odom sample at this instant only')
        p.dynamic(tf('odom', 'base_footprint', x=1.0, stamp=t0 + NS))
        p.clock(t0 + NS // 2)
        p.cmd('0.5 s later; odom samples at 0 s (x = 0) and 1 s (x = 1): the base is at 0.5 m')
        p.clock(t0 + NS)
        p.cmd('1 s later: the base has moved 1 m, the point is 0.2 m ahead')
        p.clock(t0 + NS + NS // 4)
        p.cmd('0.25 s after the newest odom sample; tolerance 0.5 s')
        p.clock(t0 + NS + NS // 4 + NS // 2)
        p.since('clock advanced by the tolerance')
        p.cmd('next command, still no newer odom sample')
        p.since('before anything else happens')
        p.dynamic(tf('odom', 'base_footprint', x=1.0, stamp=t0 + 2 * NS))
        p.since('an odom sample newer than the command arrives; clock unchanged')
        p.cmd('odom samples on both sides of the clock')
        p.node_log('Failed to get', limit=4)


def case_shift_turn(p):
    """Base shift correction when the base turns by half a turn."""
    p.start(params(['FrontBox'], FRONT_BOX.format(visualize='False'), shift='True',
                   stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    p.dynamic(tf('odom', 'base_footprint', stamp=T0))
    p.scan([0.3], stamp=T0)
    p.cmd('point 0.3 m ahead, in the front box')
    p.dynamic(tf('odom', 'base_footprint', q=(0.0, 0.0, 1.0, 0.0), stamp=T0 + NS))
    p.clock(T0 + NS)
    p.cmd('the base has turned half a turn since the scan: the point is behind it')
    p.scan([0.3], angle_min=math.pi, stamp=T0)
    p.cmd('a scan stamped before the turn with the point behind: now ahead')
    p.scan([0.3], angle_min=math.pi, stamp=T0 + NS)
    p.cmd('the same scan stamped after the turn')


def case_shift_sensor_frame(p):
    """Base shift correction with the scan in a sensor frame, and its failure cases."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), shift='True',
                   stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    p.static(tf('base_footprint', 'laser', x=0.5))
    p.dynamic(tf('odom', 'base_footprint', x=0.0, stamp=T0))
    p.dynamic(tf('odom', 'base_footprint', x=1.0, stamp=T0 + NS))
    p.scan([0.7], frame='laser', stamp=T0)
    p.clock(T0 + NS)
    p.cmd('sensor 0.5 m ahead of the base, point 0.7 m ahead of it at the scan stamp, '
          'base moved 1 m: 0.2 m ahead')
    p.scan([0.7], frame='laser', stamp=T0 - NS)
    p.cmd('a scan stamped 1 s before the oldest odom sample')
    p.clock(T0 + NS + NS // 2)
    p.since('after the tolerance')
    p.dynamic(tf('odom', 'base_footprint', x=1.0, stamp=T0 + 3 * NS))
    p.scan([0.7], frame='laser', stamp=T0 + 2 * NS)
    p.clock(T0 + 2 * NS)
    p.cmd('scan and command at 2 s, base at rest at 1 m: point 1.2 m ahead')
    p.node_log('Failed to get', limit=4)


def case_shift_no_odom(p):
    """Base shift correction with the scan already in the base frame and no odom frame."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), shift='True',
                   stop_pub_timeout=100.0))
    p.up()
    p.clock(T0)
    p.scan([2.0])
    p.cmd('scan in the base frame, nothing near, no transform published at all')
    p.clock(T0 + NS // 2)
    p.since('after the tolerance')
    p.dynamic(tf('odom', 'base_footprint', stamp=T0), tf('odom', 'base_footprint', stamp=T0 + NS))
    p.cmd('odom samples around the clock')
    p.node_log('Failed to get', limit=3)


def nav2_section(action, extra=None):
    """The collision_monitor section of nav2_bringup's nav2_params.yaml, edited."""
    path = os.path.join(PREFIX, 'share', 'nav2_bringup', 'params', 'nav2_params.yaml')
    with open(path) as f:
        section = yaml.safe_load(f)['collision_monitor']
    prm = section['ros__parameters']
    prm['FootprintApproach']['action_type'] = action
    prm['FootprintApproach'].update(extra or {})
    prm['use_sim_time'] = True
    prm['bond_heartbeat_period'] = 0.0
    return yaml.safe_dump({'collision_monitor': section})


def case_footprint_edit(p):
    """Nav2's default FootprintApproach zone with only action_type changed."""
    first = True
    for action in ('approach', 'stop', 'slowdown'):
        if not first:
            p.stop()
        first = False
        print(f'  -- nav2_params.yaml, FootprintApproach.action_type: {action}, nothing else')
        p.start(nav2_section(action))
        ok = p.configure()
        if ok:
            p.interfaces('inactive')
        p.node_log('rror', 'Making footprint', 'Subscribing', 'dynamic subscription')
    p.stop()
    print('  -- the package example file collision_monitor_params.yaml, polygons: '
          '["FootprintApproach"], action_type: stop')
    path = os.path.join(PREFIX, 'share', 'nav2_collision_monitor', 'params',
                        'collision_monitor_params.yaml')
    with open(path) as f:
        section = yaml.safe_load(f)
    prm = section['collision_monitor']['ros__parameters']
    prm['polygons'] = ['FootprintApproach']
    prm['FootprintApproach']['action_type'] = 'stop'
    prm['bond_heartbeat_period'] = 0.0
    p.start(yaml.safe_dump(section))
    p.configure()
    p.node_log('rror', 'Making footprint', 'Subscribing', 'dynamic subscription')


def case_footprint_sub_topic(p):
    """The same zone as `stop` with polygon_sub_topic pointed at the published footprint."""
    topic = '/local_costmap/published_footprint'
    p2 = Probe(cmd_topic='cmd_vel_smoothed', out_topic='cmd_vel',
               state_topic='collision_monitor_state')
    try:
        p2.start(nav2_section('stop', {'polygon_sub_topic': topic}))
        p2.configure()
        p2.interfaces('inactive')
        for i in p2.get_subscriptions_info_by_topic(topic):
            print(f'  subscription on {topic}: type {i.topic_type}, reliability '
                  f'{i.qos_profile.reliability.name}, durability {i.qos_profile.durability.name}')
        p2.activate()
        p2.spin_until(lambda: p2.cmd_pub.get_subscription_count() == 1, 10.0)
        t = T0
        p2.clock(t)
        p2.dynamic(tf('odom', 'base_footprint', x=5.0, y=3.0, stamp=t))
        p2.dynamic(tf('odom', 'base_footprint', x=5.0, y=3.0, stamp=t + NS))
        ring = [0.1] * 8
        p2.scan(ring, stamp=t)
        p2.cmd('no footprint message yet; 8 points 0.1 m ahead (min_points 6)')
        sq = [(5.25, 3.25), (5.25, 2.75), (4.75, 2.75), (4.75, 3.25)]
        p2.shape(sq, frame='odom', topic=topic)
        p2.cmd('footprint received: a 0.5 m square around the base, given in odom')
        p2.dynamic(tf('odom', 'base_footprint', x=6.0, y=3.0, stamp=t + 2 * NS))
        p2.dynamic(tf('odom', 'base_footprint', x=6.0, y=3.0, stamp=t + 3 * NS))
        p2.clock(t + 2 * NS)
        p2.scan(ring, stamp=t + 2 * NS)
        p2.cmd('the base has moved 1 m; no new footprint message; 8 points 0.1 m ahead')
        p2.scan([1.1] * 8, angle_min=math.pi, inc=0.0, stamp=t + 2 * NS)
        p2.cmd('8 points 1.1 m behind the base, where the footprint was')
        sq2 = [(x + 1.0, y) for x, y in sq]
        p2.shape(sq2, frame='odom', topic=topic)
        p2.scan(ring, stamp=t + 2 * NS)
        p2.cmd('a footprint message at the new position; 8 points 0.1 m ahead')
        p2.node_log('rror', 'not set', 'shape update', limit=6)
    finally:
        p2.stop()
        p2.destroy_node()


def case_polygon_topic(p):
    """A stop zone whose shape arrives on polygon_sub_topic."""
    p.start(params(['Dyn'], TOPIC_ZONE.format(action='stop', visualize='True', extra=''),
                   tolerance=0.5, stop_pub_timeout=100.0))
    p.watch_polygon('Dyn')
    p.clock(T0)
    p.configure()
    for i in p.get_subscriptions_info_by_topic('/zone_shape'):
        print(f'  subscription on /zone_shape: type {i.topic_type}, reliability '
              f'{i.qos_profile.reliability.name}, durability {i.qos_profile.durability.name}, '
              f'depth {i.qos_profile.depth}')
    box = [(0.5, 0.3), (0.5, -0.3), (0.0, -0.3), (0.0, 0.3)]
    p.activate()
    p.since('published by the activation, no shape yet')
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1, 10.0)
    p.scan([0.2])
    p.cmd('no shape received yet, point 0.2 m ahead')
    p.shape(box)
    p.cmd('front box received')
    p.shape([(0.5, 0.3), (0.5, -0.3)])
    p.cmd('a message with two points: refused, the box stays')
    p.shape([])
    p.cmd('a message with no points: refused, the box stays')
    p.shape([(-0.1, 0.3), (-0.1, -0.3), (-0.5, 0.0)])
    p.cmd('a triangle behind the base: accepted, the point ahead is outside')
    p.scan([0.2], angle_min=math.pi)
    p.cmd('point 0.2 m behind, inside the triangle')
    p.shape(box, frame='nowhere')
    p.since('a box in a frame without transform (the callback waits for the tolerance)')
    p.clock(T0 + NS)
    p.cmd('after the tolerance: the triangle is still the shape')
    p.static(tf('base_footprint', 'shifted', x=-0.5))
    p.shape(box, frame='shifted')
    p.cmd('the front box given in a frame 0.5 m behind the base: it lies behind, '
          'the point behind is inside')
    p.scan([0.2])
    p.cmd('point 0.2 m ahead: outside the shifted box')
    p.shape(box, frame='')
    p.since('a box with an empty frame_id')
    p.clock(T0 + 2 * NS)
    p.cmd('after the tolerance: shape unchanged')
    p.shape([(float('nan'), 0.3), (0.5, -0.3), (0.0, -0.3), (0.0, 0.3)])
    p.cmd('a box with a NaN coordinate, point 0.2 m ahead')
    p.shape([(0.5, 0.3), (0.5, -0.3), (0.0, -0.3), (0.0, 0.3), (0.5, 0.3)])
    p.cmd('a closed box (first point repeated)')
    p.shape([(0.5, 0.0), (0.5, 0.0), (0.5, 0.0)])
    p.cmd('three equal points')
    for t, m in p.polys[-1:]:
        print(f'      last published polygon on {t}: frame {m.header.frame_id!r}, '
              f'{[(q.x, q.y) for q in m.polygon.points]}')
    p.node_log('rror', 'not set', limit=8)


def case_polygon_topic_moving(p):
    """A shape given in a moving frame follows the newest transform at each command."""
    p.start(params(['Dyn'], TOPIC_ZONE.format(action='slowdown', visualize='False', extra=''),
                   tolerance=0.5))
    p.up()
    p.clock(T0)
    box = [(0.5, 0.3), (0.5, -0.3), (0.0, -0.3), (0.0, 0.3)]
    p.dynamic(tf('base_footprint', 'carrier', x=0.0, stamp=T0))
    p.shape(box, frame='carrier')
    p.scan([0.2])
    p.cmd('box in frame carrier, which coincides with the base: point inside, slowdown')
    p.dynamic(tf('base_footprint', 'carrier', x=2.0, stamp=T0 + NS))
    p.cmd('carrier has moved 2 m ahead, no new shape message')
    p.scan([2.2])
    p.cmd('point 2.2 m ahead')
    p.node_log('rror', limit=4)


def case_polygon_topic_inactive(p):
    """Shape messages while inactive and across cleanup; polygon_subscribe_transient_local."""
    extra = '      polygon_subscribe_transient_local: True\n'
    p.start(params(['Dyn'], TOPIC_ZONE.format(action='stop', visualize='False', extra=extra),
                   stop_pub_timeout=100.0))
    box = [(0.5, 0.3), (0.5, -0.3), (0.0, -0.3), (0.0, 0.3)]
    tri = [(-0.1, 0.3), (-0.1, -0.3), (-0.5, 0.0)]
    p.clock(T0)
    p.shape(box, latched=True)
    print('  a latched front box (reliable, transient local) was published before configure')
    p.configure()
    for i in p.get_subscriptions_info_by_topic('/zone_shape'):
        print(f'  subscription on /zone_shape: reliability {i.qos_profile.reliability.name}, '
              f'durability {i.qos_profile.durability.name}')
    p.spin(0.5)
    p.activate()
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1, 10.0)
    p.scan([0.2])
    p.cmd('point 0.2 m ahead: stop if the latched box was delivered')
    p.deactivate()
    p.shape(tri, latched=True)
    print('  a triangle behind the base was published while inactive')
    p.activate()
    p.cmd('active again, point 0.2 m ahead')
    p.scan([0.2], angle_min=math.pi)
    p.cmd('point 0.2 m behind: stop if the triangle was taken while inactive')
    p.deactivate()
    p.cleanup()
    p.configure()
    p.activate()
    p.spin_until(lambda: p.cmd_pub.get_subscription_count() == 1, 10.0)
    p.spin(0.5)
    p.scan([0.2], angle_min=math.pi)
    p.cmd('after cleanup, configure, activate, no new message: point 0.2 m behind')
    p.node_log('shape update', 'not set', limit=4)


def case_footprint_topic_ignored(p):
    """footprint_topic on a stop zone that has static points."""
    zone = """    Foot:
      type: "polygon"
      action_type: "stop"
      points: "[[0.5, 0.3], [0.5, -0.3], [0.0, -0.3], [0.0, 0.3]]"
      footprint_topic: "/local_costmap/published_footprint"
      polygon_sub_topic: "zone_shape"
      min_points: 1
"""
    p.start(params(['Foot'], zone))
    p.configure()
    p.interfaces('inactive: static points with footprint_topic and polygon_sub_topic set')


def case_bond(p):
    """The bond topic with heartbeats on, wall-clock time."""
    from bond.msg import Status
    msgs = []
    p.create_subscription(Status, '/bond', msgs.append, 100)
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), bond=0.1, sim='False'))
    p.configure()
    p.spin(1.0)
    print(f'  bond messages while inactive: {len(msgs)}')
    p.activate()
    p.spin(2.0)
    print(f'  bond messages in 2 s after activation: {len(msgs)}')
    for m in msgs[:1]:
        print(f'      id {m.id!r}, active {m.active}, heartbeat_timeout {m.heartbeat_timeout}, '
              f'heartbeat_period {m.heartbeat_period}')
    for i in p.get_publishers_info_by_topic('/bond'):
        print(f'  /bond publisher of {i.node_name}: reliability {i.qos_profile.reliability.name}, '
              f'durability {i.qos_profile.durability.name}')
    n = len(msgs)
    p.deactivate()
    p.spin(1.0)
    tail = msgs[n:]
    print(f'  bond messages after deactivation: {len(tail)}, active flags '
          f'{sorted(set(m.active for m in tail))}')
    p.node_log('bond')


def case_manager(p):
    """Nav2's lifecycle manager drives the node; then the node is killed."""
    p.start(params(['StopCircle'], STOP_CIRCLE.format(visualize='False'), bond=0.1, sim='False'))
    d = tempfile.mkdtemp(prefix='cm_nav2_probe_mgr_')
    log = open(os.path.join(d, 'manager.log'), 'w')
    mgr = subprocess.Popen(
        [os.path.join(PREFIX, 'lib', 'nav2_lifecycle_manager', 'lifecycle_manager'),
         '--ros-args', '-r', '__node:=lifecycle_manager_probe',
         '-p', 'autostart:=true', '-p', f'node_names:=[{NODE}]', '-p', 'bond_timeout:=4.0',
         '-p', 'attempt_respawn_reconnection:=false'],
        stdout=log, stderr=log)
    try:
        p.spin(8.0)
        print(f'  8 s after the lifecycle manager started: node state {p.state()}')
        p.spin(1.0)
        p.proc.send_signal(signal.SIGKILL)
        p.proc.wait()
        print('  node killed (SIGKILL)')
        p.spin(7.0)
    finally:
        mgr.send_signal(signal.SIGINT)
        try:
            mgr.wait(timeout=10)
        except subprocess.TimeoutExpired:
            mgr.kill()
        log.close()
    with open(log.name) as f:
        for line in f:
            if 'lifecycle_manager_probe' in line:
                print('      manager:' + line.rstrip().split(']', 2)[-1])
    p.node_log('bond', 'Configuring', 'Activating')


CASES = {
    'lifecycle': case_lifecycle,
    'state': case_state,
    'visualize': case_visualize,
    'bond': case_bond,
    'manager': case_manager,
    'tf-latest': case_tf_latest,
    'tf-tilt': case_tf_tilt,
    'tf-pitch-circle': case_tf_pitch_circle,
    'tf-missing': case_tf_missing,
    'tf-missing-timeout0': case_tf_missing_timeout0,
    'shift-straight': case_shift_straight,
    'shift-turn': case_shift_turn,
    'shift-sensor-frame': case_shift_sensor_frame,
    'shift-no-odom': case_shift_no_odom,
    'footprint-edit': case_footprint_edit,
    'footprint-sub-topic': case_footprint_sub_topic,
    'footprint-topic-ignored': case_footprint_topic_ignored,
    'polygon-topic': case_polygon_topic,
    'polygon-topic-moving': case_polygon_topic_moving,
    'polygon-topic-inactive': case_polygon_topic_inactive,
}


def main():
    names = sys.argv[1:]
    if names == ['--list']:
        print('\n'.join(CASES))
        return 0
    if not PREFIX:
        print('CM_PREFIX is not set; use run_nav2_probe.sh', file=sys.stderr)
        return 2
    print(f'# nav2_probe.py: {rclpy.get_rmw_implementation_identifier()}, '
          f'ROS_DISTRO {os.environ.get("ROS_DISTRO")}, {os.uname().sysname} {os.uname().machine}')
    for name in names or list(CASES):
        print(f'== {name}: {CASES[name].__doc__.strip()}')
        rclpy.init()
        p = Probe()
        try:
            CASES[name](p)
        except Exception as e:  # a probe prints; it does not stop at the first surprise
            print(f'  PROBE ERROR: {type(e).__name__}: {e}')
        finally:
            p.stop()
            p.destroy_node()
            rclpy.shutdown()
        print()
        sys.stdout.flush()
        time.sleep(1.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
