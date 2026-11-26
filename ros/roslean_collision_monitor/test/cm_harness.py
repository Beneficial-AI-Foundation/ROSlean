"""Shared machinery for the collision-monitor tests (scenario.py, random_diff.py).

The node under test runs with `use_sim_time` and the test publishes `/clock`,
so the node-clock time of every event is chosen exactly. The test also
publishes the laser scans, the velocity commands and, if asked, the static
transforms, and records everything the node publishes on its velocity output
and on its state topic.

Nothing here knows what the node should do. The only things assumed about the
node under test are its interface:

  * parameters as `nav2_collision_monitor` (a parameter file plus `-p`
    overrides), including `cmd_vel_in_topic`, `cmd_vel_out_topic`,
    `state_topic` and `enable_stamped_cmd_vel`;
  * a `sensor_msgs/LaserScan` subscription and a `/clock` subscription;
  * optionally the lifecycle services (`<node>/change_state`). If the node
    has them the test configures and activates it; if it does not, the node
    is taken to be active from start-up.

The only wall-clock dependence is that a message must be delivered and
handled within the settle time, and that a clock update or scan reaches the
node before the next command does.
"""

import math
import os
import signal
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, TransformStamped, Twist, TwistStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
NS = 1_000_000_000
UPSTREAM = 'nav2_collision_monitor/collision_monitor'

# Action types of nav2_msgs/CollisionMonitorState.
DO_NOTHING, STOP, SLOWDOWN, APPROACH, LIMIT = 0, 1, 2, 3, 4


def s(seconds):
    """Seconds (a binary fraction) to integer nanoseconds, exactly."""
    ns = seconds * NS
    assert ns == int(ns), seconds
    return int(ns)


def f32(x):
    """The float32 nearest to x, as a Python float (what a LaserScan carries)."""
    return struct.unpack('f', struct.pack('f', x))[0]


def f32_next(x, steps=1):
    """The float32 `steps` representable values above (below if negative) f32(x)."""
    bits = struct.unpack('i', struct.pack('f', x))[0]
    assert x > 0
    return struct.unpack('f', struct.pack('i', bits + steps))[0]


def bits(x):
    """Bit pattern of a double: distinguishes -0.0 from 0.0 and compares NaNs."""
    return struct.pack('d', float(x))


def resolve(node):
    """An executable path, or `<package>/<executable>` looked up in the ament index."""
    if os.path.sep in node and os.path.isfile(node):
        return node
    package, executable = node.split('/')
    from ament_index_python.packages import get_package_prefix
    prefix = get_package_prefix(package)
    refuse_overlay(package, prefix)
    return os.path.join(prefix, 'lib', package, executable)


def refuse_overlay(package, prefix):
    """Ends the test if `package` is the ROSlean overlay of that name.

    overlay/nav2_collision_monitor of this repository is the Lean-backed node
    under upstream's package name and installs the file checked here. A test
    that asks for upstream's node by its package name would otherwise compare
    the Lean-backed node with itself and pass.
    """
    if os.path.isfile(os.path.join(prefix, 'share', package, 'ROSLEAN_OVERLAY')):
        sys.exit(f'{package} at {prefix} is the ROSlean overlay, not upstream: run the tests '
                 'in an environment in which the overlay is not sourced')


def scan_msg(spec, now_ns, default_frame):
    """A LaserScan from a dict. Missing keys get plain defaults.

    ranges            list of floats (required)
    angle_min         default 0.0
    angle_increment   default 0.0
    angle_max         default angle_min + angle_increment * (len(ranges) - 1)
    range_min/max     default 0.05 / 10.0
    stamp             absolute header stamp in ns; default: the current clock
    age               alternative to stamp: stamp = current clock - age (ns)
    frame             default: the scenario's scan frame
    """
    m = LaserScan()
    stamp = spec.get('stamp', now_ns - spec.get('age', 0))
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, NS)
    m.header.frame_id = spec.get('frame', default_frame)
    n = len(spec['ranges'])
    m.angle_min = float(spec.get('angle_min', 0.0))
    m.angle_increment = float(spec.get('angle_increment', 0.0))
    m.angle_max = float(spec.get('angle_max', m.angle_min + m.angle_increment * max(n - 1, 0)))
    m.range_min = float(spec.get('range_min', 0.05))
    m.range_max = float(spec.get('range_max', 10.0))
    m.ranges = [float(r) for r in spec['ranges']]
    return m


def fan(n, r, centre=0.0, step=0.01):
    """n rays of range r, `step` rad apart, centred on the angle `centre`."""
    return {'ranges': [r] * n, 'angle_min': centre - step * (n - 1) / 2, 'angle_increment': step}


def ray(r, angle=0.0, **extra):
    """A scan of one ray. At angle 0 the point is exactly (float32(r), 0)."""
    return dict({'ranges': [r], 'angle_min': angle}, **extra)


def static_tf(parent, child, x=0.0, y=0.0, yaw_half_turns=0):
    """A static transform. Only yaw 0 or half a turn: both have exact quaternions."""
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.transform.translation.x, t.transform.translation.y = float(x), float(y)
    if yaw_half_turns % 2:
        t.transform.rotation.z, t.transform.rotation.w = 1.0, 0.0
    else:
        t.transform.rotation.w = 1.0
    return t


def xf(parent, child, x=0.0, y=0.0, z=0.0, q=(0.0, 0.0, 0.0, 1.0), stamp=0):
    """A transform with any translation and quaternion (x, y, z, w) and a chosen stamp (ns)."""
    t = TransformStamped()
    t.header.frame_id, t.child_frame_id = parent, child
    t.header.stamp.sec, t.header.stamp.nanosec = divmod(stamp, NS)
    t.transform.translation.x = float(x)
    t.transform.translation.y = float(y)
    t.transform.translation.z = float(z)
    (t.transform.rotation.x, t.transform.rotation.y,
     t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in q)
    return t


def polygon_msg(points, frame, stamp_ns):
    """A PolygonStamped from (x, y) pairs; the coordinates are float32 on the wire."""
    m = PolygonStamped()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp_ns, NS)
    m.header.frame_id = frame
    m.polygon.points = [Point32(x=float(x), y=float(y)) for x, y in points]
    return m


class Side:
    """One node under test: its process and what it has published."""

    def __init__(self, exe, name, out_topic, state_topic):
        self.exe, self.name = exe, name
        self.out_topic, self.state_topic = out_topic, state_topic
        self.proc = None
        self.out = []      # every velocity message received, in order
        self.states = []   # every state message received, in order
        self.polys = []    # every (topic, PolygonStamped) received on a watched topic
        self.lifecycle = None

    def start(self, params_file, params, log=None):
        args = [self.exe, '--ros-args', '--params-file', params_file,
                '-r', f'__node:={self.name}',
                '-p', 'use_sim_time:=true',
                '-p', f'cmd_vel_out_topic:={self.out_topic}',
                # Nav2 lifecycle nodes: no bond with a lifecycle manager.
                '-p', 'bond_heartbeat_period:=0.0']
        # A node without a state topic (state_topic None) is not given the
        # parameter at all: the ROSlean node refuses it (SPEC, D13).
        if self.state_topic:
            args += ['-p', f'state_topic:={self.state_topic}']
        for p in params:
            args += ['-p', p]
        self.log = open(log, 'w') if log else None
        out = self.log or subprocess.DEVNULL
        self.proc = subprocess.Popen(args, stdout=out, stderr=out)

    def stop(self):
        if self.proc is None:
            return
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        if self.log:
            self.log.close()


class Driver(Node):
    """Publishes the inputs of one or two nodes under test and records their outputs."""

    def __init__(self, name, sides, stamped, cmd_topic='cmd_vel_in', scan_topic='scan',
                 scan_frame='base_footprint'):
        super().__init__(name)
        self.sides = sides
        self.stamped = stamped
        self.scan_frame = scan_frame
        self.cmd_topic, self.scan_topic = cmd_topic, scan_topic
        self.T = TwistStamped if stamped else Twist
        self.now_ns = 0
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.cmd_pub = self.create_publisher(self.T, cmd_topic, 10)
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, 10)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.create_publisher(TFMessage, '/tf_static', latched)
        self.tf_dyn_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.shape_pubs = {}
        for side in sides:
            self.create_subscription(
                self.T, side.out_topic, lambda m, sd=side: sd.out.append(m), 10)
            if side.state_topic:
                self.create_subscription(
                    CollisionMonitorState, side.state_topic,
                    lambda m, sd=side: sd.states.append(m), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def spin_until(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.005)
        return cond()

    # -- bringing the nodes up ------------------------------------------------

    def _has_service(self, name):
        return any(n == name for n, _ in self.get_service_names_and_types())

    def detect_lifecycle(self, side, mode, timeout=20.0):
        """True if the node offers lifecycle services (mode 'auto'), else as told."""
        if mode != 'auto':
            return mode == 'yes'
        service = f'/{side.name}/change_state'
        end = time.monotonic() + timeout
        plain_since = None
        while time.monotonic() < end:
            if self._has_service(service):
                return True
            if side.proc.poll() is not None:
                return False
            # A plain node subscribes to its command topic straight away. A
            # lifecycle node does so only once configured, which is our job.
            if any(i.node_name == side.name
                   for i in self.get_subscriptions_info_by_topic(self.cmd_topic)):
                plain_since = plain_since or time.monotonic()
                if time.monotonic() - plain_since > 1.0:
                    return False
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError(f'{side.name}: neither lifecycle services nor a subscription appeared')

    def change_state(self, side, transition, timeout=20.0):
        client = self.create_client(ChangeState, f'/{side.name}/change_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'{side.name}: no change_state service')
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            if not self.spin_until(future.done, timeout):
                raise RuntimeError(f'{side.name}: transition {transition} timed out')
            return future.result().success
        finally:
            self.destroy_client(client)

    def bring_up(self, side, mode='auto'):
        """Returns 'active', or 'refused' if the node rejects its configuration.

        A lifecycle node refuses by failing the configure transition; a plain
        node refuses by exiting.
        """
        side.lifecycle = self.detect_lifecycle(side, mode)
        if side.lifecycle:
            if not self.change_state(side, Transition.TRANSITION_CONFIGURE):
                return 'refused'
            if not self.change_state(side, Transition.TRANSITION_ACTIVATE):
                return 'refused'
            return 'active'
        self.spin_for(0.5)
        return 'refused' if side.proc.poll() is not None else 'active'

    def publish_tf(self, transforms):
        if transforms:
            self.tf_pub.publish(TFMessage(transforms=transforms))

    # -- the Nav2 extension: /tf with stamps, zone shapes, polygon topics,
    # -- lifecycle transitions in the middle of a run -------------------------

    def publish_dynamic(self, transforms, settle):
        """Transforms on /tf, each with the stamp it carries (see `xf`)."""
        self.tf_dyn_pub.publish(TFMessage(transforms=list(transforms)))
        self.spin_for(settle)

    def publish_static(self, transforms, settle):
        """Static transforms sent while the node runs (one message)."""
        self.tf_pub.publish(TFMessage(transforms=list(transforms)))
        self.spin_for(settle)

    def shape_publisher(self, topic, latched=False):
        """The PolygonStamped publisher for a zone's `polygon_sub_topic`: reliable,
        and volatile unless `latched` (upstream's subscription is best effort)."""
        if topic not in self.shape_pubs:
            qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST)
            if latched:
                qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.shape_pubs[topic] = self.create_publisher(PolygonStamped, topic, qos)
        return self.shape_pubs[topic]

    def send_shape(self, topic, points, frame, settle, stamp=None):
        self.shape_publisher(topic).publish(
            polygon_msg(points, frame, self.now_ns if stamp is None else stamp))
        self.spin_for(settle)

    def watch_polygon(self, side, topic):
        """Record every PolygonStamped the node publishes on `topic` (volatile, so
        that only messages published while we listen are seen)."""
        self.create_subscription(
            PolygonStamped, topic, lambda m, t=topic: side.polys.append((t, m)), 10)

    def get_state(self, side, timeout=10.0):
        """The label of the node's lifecycle state, or None."""
        client = self.create_client(GetState, f'/{side.name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(GetState.Request())
            if not self.spin_until(future.done, timeout):
                return None
            return future.result().current_state.label
        finally:
            self.destroy_client(client)

    def endpoints_of(self, side, topic, publishers):
        """The endpoints of the node under test on a topic."""
        get = (self.get_publishers_info_by_topic if publishers
               else self.get_subscriptions_info_by_topic)
        return [i for i in get(topic) if i.node_name == side.name]

    def wait_connected(self, tf_listeners=0, timeout=20.0):
        """Wait until every node has all its subscriptions and its publisher.

        tf_listeners: how many of the nodes must have subscribed to the
        static transforms (the lifecycle nodes, if transforms are published).
        """
        n = len(self.sides)

        def ready():
            ok = (self.clock_pub.get_subscription_count() >= n
                  and self.cmd_pub.get_subscription_count() == n
                  and self.scan_pub.get_subscription_count() == n
                  and self.tf_pub.get_subscription_count() >= tf_listeners)
            for side in self.sides:
                ok = ok and self.count_publishers(side.out_topic) == 1
            return ok

        # The conditions must hold before and after half a second of spinning,
        # not just once: under load one publisher was counted and none was
        # listed half a second later (the graph can still show the endpoints
        # of the previous run's node for a moment, or the node has died).
        end = time.monotonic() + timeout
        while True:
            if not self.spin_until(ready, max(end - time.monotonic(), 0.0)):
                raise RuntimeError(
                    'node under test did not come up (exit codes %s)'
                    % [sd.proc.poll() for sd in self.sides])
            self.spin_for(0.5)
            if ready():
                return
            if time.monotonic() >= end:
                raise RuntimeError('the graph did not settle: endpoints appeared and went away '
                                   '(exit codes %s)' % [sd.proc.poll() for sd in self.sides])

    def endpoint_types(self, topic, publishers, timeout=5.0):
        """The message types of the publishers (or the subscriptions) of a topic.

        Waits until there is at least one, and raises RuntimeError if none
        appears: the list of endpoints can lag behind the counts that
        wait_connected looks at.
        """
        get = (self.get_publishers_info_by_topic if publishers
               else self.get_subscriptions_info_by_topic)
        found = []

        def some():
            found[:] = [i.topic_type for i in get(topic)]
            return bool(found)

        if not self.spin_until(some, timeout):
            raise RuntimeError('no %s of %s in the graph'
                               % ('publisher' if publishers else 'subscription', topic))
        return sorted(set(found))

    # -- events ---------------------------------------------------------------

    def set_clock(self, t_ns, settle):
        self.now_ns = t_ns
        m = Clock()
        m.clock.sec, m.clock.nanosec = divmod(t_ns, NS)
        self.clock_pub.publish(m)
        self.spin_for(settle)

    def send_scan(self, spec, settle):
        self.scan_pub.publish(scan_msg(spec, self.now_ns, self.scan_frame))
        self.spin_for(settle)

    def make_cmd(self, vel, ident):
        """vel: (vx, vy, wz) or all six (vx, vy, vz, wx, wy, wz)."""
        m = self.T()
        t = m.twist if self.stamped else m
        if len(vel) == 3:
            t.linear.x, t.linear.y, t.angular.z = (float(v) for v in vel)
        else:
            (t.linear.x, t.linear.y, t.linear.z,
             t.angular.x, t.angular.y, t.angular.z) = (float(v) for v in vel)
        if self.stamped:
            m.header.frame_id = f'cmd_{ident}'
            m.header.stamp.sec, m.header.stamp.nanosec = 7, ident
        return m

    def send_cmd(self, msg, settle, wait_for=None, max_wait=2.0):
        """Publish a command; return per side the (velocity, state) messages it caused.

        Waits `settle`, or up to `max_wait` until `wait_for(side)` holds for
        every side (used when an answer is expected, so that a slow machine
        does not turn an answer into a "nothing published").
        """
        marks = [(len(sd.out), len(sd.states)) for sd in self.sides]
        self.cmd_pub.publish(msg)
        self.spin_for(settle)
        if wait_for is not None:
            self.spin_until(lambda: all(wait_for(i, sd.out[m[0]:], sd.states[m[1]:])
                                        for i, (sd, m) in enumerate(zip(self.sides, marks))),
                            max_wait)
        return [(sd.out[m[0]:], sd.states[m[1]:]) for sd, m in zip(self.sides, marks)]

    def twist_of(self, msg):
        return msg.twist if self.stamped else msg


def twist6(t):
    return (t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z)


def same_floats(a, b):
    return len(a) == len(b) and all(bits(x) == bits(y) for x, y in zip(a, b))


def fmt(v):
    return '(' + ', '.join(repr(x) for x in v) + ')'


def finite(v):
    return all(math.isfinite(x) for x in v)
