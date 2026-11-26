#!/usr/bin/env python3
"""CM3/CM4 review: the Lean node and upstream nav2_collision_monitor side by side.

Each case starts both nodes with the same parameter file, under simulated time
(this script publishes /clock, so every node-clock reading is chosen exactly),
feeds both the same scans and commands on the same topics, and prints what
each published for every command. Lines where the two differ are marked DIFF.

    python3 side_by_side.py <lean collision_monitor executable> [case ...]

Environment: CM_PREFIX is the prefix that holds upstream's binary
(run_side_by_side.sh sets it).
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
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist, TwistStamped, PolygonStamped, Point32, TransformStamped
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters

NS = 1_000_000_000
SETTLE = 0.25

HEAD = """
/**:
  ros__parameters:
    use_sim_time: {sim}
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_in"
    cmd_vel_out_topic: "cmd_out"
    transform_tolerance: 0.1
    source_timeout: {source_timeout}
    base_shift_correction: False
    stop_pub_timeout: {stop_pub_timeout}
    bond_heartbeat_period: 0.0
    enable_stamped_cmd_vel: {stamped}
    polygons: [{names}]
{zones}
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
{source}
"""


def zone(name, **kw):
    lines = [f"    {name}:"]
    for k, v in kw.items():
        lines.append(f"      {k}: {v}")
    return name, "\n".join(lines)


def circle(name="Zone", radius=0.5, action="stop", min_points=1, **kw):
    return zone(name, type='"circle"', radius=radius, action_type=f'"{action}"',
                min_points=min_points, **kw)


def box(name="Zone", h=0.5, action="stop", min_points=1, **kw):
    pts = f'"[[{h}, {h}], [{h}, -{h}], [-{h}, -{h}], [-{h}, {h}]]"'
    return zone(name, type='"polygon"', points=pts, action_type=f'"{action}"',
                min_points=min_points, **kw)


def bits(v):
    return struct.unpack("<Q", struct.pack("<d", v))[0]


class Driver(Node):
    def __init__(self, stamped=False):
        super().__init__("cm_side_by_side")
        self.stamped = stamped
        t = TwistStamped if stamped else Twist
        self.t = t
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.scan_pub = self.create_publisher(LaserScan, "scan", 10)
        self.cmd_pub = self.create_publisher(t, "cmd_in", 10)
        self.poly_pub = self.create_publisher(PolygonStamped, "zone_poly", 10)
        latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_pub = self.create_publisher(TFMessage, "/tf_static", latched)
        self.out = {"lean": [], "up": []}
        self.create_subscription(t, "cmd_out_lean", self.out["lean"].append, 10)
        self.create_subscription(t, "cmd_out_up", self.out["up"].append, 10)
        self.change = self.create_client(ChangeState, "cm_up/change_state")
        self.now_ns = 0
        self.count = 0

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def transition(self, tid):
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.change.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return fut.result().success if fut.result() is not None else None

    def set_param(self, node, name, value):
        cli = self.create_client(SetParameters, f"{node}/set_parameters")
        if not cli.wait_for_service(timeout_sec=5.0):
            return "no service"
        req = SetParameters.Request()
        req.parameters = [Parameter(name, value=value).to_parameter_msg()]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        r = fut.result()
        return "no answer" if r is None else (
            "accepted" if r.results[0].successful else f"rejected ({r.results[0].reason})")

    def clock(self, ns):
        self.now_ns = ns
        m = Clock()
        m.clock.sec, m.clock.nanosec = divmod(ns, NS)
        self.clock_pub.publish(m)
        self.spin(SETTLE)

    def scan(self, ranges, angle_min=0.0, inc=0.0, frame="base_footprint", stamp=None,
             range_min=0.0, range_max=100.0):
        m = LaserScan()
        s = self.now_ns if stamp is None else stamp
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(s, NS)
        m.header.frame_id = frame
        m.angle_min = float(angle_min)
        m.angle_increment = float(inc)
        m.range_min = float(range_min)
        m.range_max = float(range_max)
        m.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(m)
        self.spin(SETTLE)

    def cmd(self, x=0.5, y=0.0, w=0.0, lz=0.0, advance=0):
        """Publishes one command and returns what each node published for it.
        With `advance`, the clock is moved on by that many ns after the
        command was delivered: upstream waits for a missing transform in node
        time (transform_tolerance), so with a frozen clock it never answers."""
        for o in self.out.values():
            o.clear()
        m = self.t()
        tw = m.twist if self.stamped else m
        tw.linear.x, tw.linear.y, tw.angular.z, tw.linear.z = float(x), float(y), float(w), float(lz)
        if self.stamped:
            self.count += 1
            m.header.frame_id = f"f{self.count}"
            m.header.stamp.sec = self.count
        self.cmd_pub.publish(m)
        self.spin(SETTLE)
        if advance:
            self.clock(self.now_ns + advance)
        res = {}
        for who, msgs in self.out.items():
            r = []
            for o in msgs:
                tw = o.twist if self.stamped else o
                vals = (tw.linear.x, tw.linear.y, tw.angular.z)
                extra = (tw.linear.z, tw.angular.x, tw.angular.y)
                item = tuple(bits(v) for v in vals + extra)
                if self.stamped:
                    item += (o.header.frame_id, o.header.stamp.sec)
                r.append((vals, item))
            res[who] = r
        return res


def fmt(r):
    if not r:
        return "nothing"
    return "; ".join("(" + ", ".join(repr(v) for v in vals) + ")" for vals, _ in r)


class Case:
    """Both nodes, started with the same parameter file."""

    def __init__(self, lean_exe, zones, source_timeout=2.0, stop_pub_timeout=1.0, stamped=False,
                 source="", lean_extra=(), sim=True):
        self.d = Driver(stamped)
        f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        f.write(HEAD.format(source_timeout=source_timeout, stop_pub_timeout=stop_pub_timeout,
                            stamped=str(stamped).lower(), sim=str(sim).lower(),
                            names=", ".join(f'"{n}"' for n, _ in zones),
                            zones="\n".join(z for _, z in zones), source=source))
        f.close()
        self.file = f.name
        up_exe = os.path.join(os.environ["CM_PREFIX"], "lib", "nav2_collision_monitor",
                              "collision_monitor")
        self.logs = {w: open(f.name + f".{w}.log", "w") for w in ("lean", "up")}
        lean_args = [lean_exe, "--ros-args", "--params-file", f.name, "-r",
                     "cmd_out:=cmd_out_lean"]
        for p in lean_extra:
            lean_args += ["-p", p]
        self.procs = {
            "lean": subprocess.Popen(lean_args, stdout=self.logs["lean"], stderr=subprocess.STDOUT),
            "up": subprocess.Popen(
                [up_exe, "--ros-args", "--params-file", f.name, "-r", "__node:=cm_up",
                 "-r", "cmd_out:=cmd_out_up"], stdout=self.logs["up"], stderr=subprocess.STDOUT),
        }
        d = self.d
        self.up_ok = False
        if d.change.wait_for_service(timeout_sec=20.0):
            conf = d.transition(Transition.TRANSITION_CONFIGURE)
            self.up_ok = bool(conf and d.transition(Transition.TRANSITION_ACTIVATE))
        end = time.monotonic() + 10.0
        while time.monotonic() < end and (
                d.cmd_pub.get_subscription_count() < 2 or d.scan_pub.get_subscription_count() < 2
                or (sim and d.clock_pub.get_subscription_count() < 2)
                or d.count_publishers("cmd_out_lean") == 0 or d.count_publishers("cmd_out_up") == 0):
            d.spin(0.1)
        d.spin(0.5)
        self.alive = {w: p.poll() is None for w, p in self.procs.items()}
        self.rows = 0
        self.diffs = 0

    def cmd(self, label, expect_diff=None, **kw):
        res = self.d.cmd(**kw)
        same = [i for _, i in res["lean"]] == [i for _, i in res["up"]]
        self.rows += 1
        mark = "same" if same else ("DIFF " + (expect_diff or "(not expected)"))
        if not same:
            self.diffs += 1
        print(f"    {label}: lean {fmt(res['lean'])} | upstream {fmt(res['up'])}  [{mark}]")
        return res

    def close(self):
        for w, p in self.procs.items():
            if p.poll() is not None:
                print(f"    note: {w} exited with code {p.returncode} during the case")
            p.send_signal(signal.SIGINT)
        for p in self.procs.values():
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
        self.d.destroy_node()
        time.sleep(0.3)

    def log(self, who, pattern):
        self.logs[who].flush()
        with open(self.file + f".{who}.log") as f:
            return [l.strip() for l in f if pattern in l][-3:]


T0 = 100 * NS


def header(c, title):
    print(f"  {title} (lean alive={c.alive['lean']}, upstream active={c.up_ok})")


def case_circle(exe):
    """Points on and next to a circle; a negative radius."""
    c = Case(exe, [circle(radius=0.5)])
    header(c, "circle, radius 0.5 m, stop, min_points 1")
    d = c.d
    d.clock(T0)
    c.cmd("no scan yet")
    for r, exp in [(0.4, None), (0.4999, None), (0.5, "D4"), (0.5005, "D4"), (0.5015, "D4"),
                   (0.5025, None), (0.6, None)]:
        d.scan([r])
        c.cmd(f"one point at range {r} m, angle 0", expect_diff=exp)
    c.close()
    c = Case(exe, [circle(radius=-0.3)])
    header(c, "circle, radius -0.3 m")
    d = c.d
    d.clock(T0)
    for r in (0.2, 0.31):
        d.scan([r])
        c.cmd(f"one point at {r} m")
    c.close()


def case_polygon(exe):
    """A point on each side of a square, and min_points exactly reached."""
    c = Case(exe, [box(h=0.5)])
    header(c, "square of half-width 0.5 m, stop, min_points 1")
    d = c.d
    d.clock(T0)
    hp = math.pi / 2
    for name, ang, r, exp in [
            ("front side (0.5, 0)", 0.0, 0.5, "D4"), ("left side (0, 0.5)", hp, 0.5, "D4"),
            ("rear side (-0.5, 0)", math.pi, 0.5, "D4"), ("right side (0, -0.5)", -hp, 0.5, "D4"),
            ("front, 3 mm outside", 0.0, 0.503, None), ("front, 6 mm outside", 0.0, 0.506, None),
            ("rear, 6 mm outside", math.pi, 0.506, None), ("interior", 0.0, 0.2, None),
            ("corner (0.5, 0.5)", math.pi / 4, 0.5 * math.sqrt(2.0), "D4")]:
        d.scan([r], angle_min=ang)
        c.cmd(name, expect_diff=exp)
    c.close()
    c = Case(exe, [box(h=0.5, min_points=3)])
    header(c, "square, min_points 3")
    d = c.d
    d.clock(T0)
    d.scan([0.2, 0.2, 5.0])
    c.cmd("two points inside")
    d.scan([0.2, 0.2, 0.2])
    c.cmd("three points inside (angle_increment 0: the same point three times)")
    d.scan([0.2, 0.2, 0.2, 0.2], inc=math.nan)
    c.cmd("four readings, angle_increment NaN: one point, then NaN points")
    d.scan([0.2, 0.2, 0.2], angle_min=math.nan)
    c.cmd("angle_min NaN")
    d.scan([0.2, 0.2, 0.2], range_min=0.3)
    c.cmd("three readings below range_min")
    d.scan([math.inf, math.inf, math.inf], range_max=math.inf)
    c.cmd("three infinite readings with range_max inf")
    d.scan([0.2, 0.2, 0.2], range_min=math.nan)
    c.cmd("range_min NaN")
    d.scan([])
    c.cmd("empty scan")
    c.close()


def case_timeout(exe):
    """The boundary of source_timeout (R7, R17) and of stop_pub_timeout (R15)."""
    c = Case(exe, [circle(radius=0.5)], source_timeout=2.094)
    header(c, "source_timeout 2.094 s (source's own not set), stop_pub_timeout 1 s")
    d = c.d
    d.clock(T0)
    d.scan([5.0])
    c.cmd("fresh scan, nothing near")
    for dt in (2_093_999_997, 2_093_999_998, 2_093_999_999, 2_094_000_000):
        d.clock(T0 + dt)
        c.cmd(f"scan stamp + {dt} ns")
    t1 = T0 + 2_093_999_999  # the first stop
    for dt in (NS - 1, NS, NS + 1):
        d.clock(t1 + dt)
        c.cmd(f"first stop + {dt} ns")
    d.scan([5.0], stamp=d.now_ns + 50 * NS)
    c.cmd("scan stamped 50 s in the future")
    c.cmd("zero command", x=0.0)
    c.cmd("minus-zero command", x=-0.0)
    d.clock(d.now_ns + NS + 1)
    c.cmd("zero command 1 s + 1 ns later", x=0.0)
    c.cmd("NaN in linear.z", lz=math.nan)
    c.cmd("command 0.5")
    c.close()
    c = Case(exe, [circle(radius=0.5)], source_timeout=0.0)
    header(c, "source_timeout 0.0")
    d = c.d
    d.clock(T0)
    c.cmd("no scan")
    d.scan([0.2], stamp=NS)
    c.cmd("scan stamped 99 s ago with a point inside")
    d.scan([5.0], frame="laser_link")
    c.cmd("then a scan in another frame, no transform; clock moved on 1 s after the command",
          advance=NS)
    c.close()


def case_slowdown(exe):
    """Slowdown products, a zero command in a slowdown zone, ratio 0, ratio above 1."""
    c = Case(exe, [circle(radius=1.0, action="slowdown", slowdown_ratio=0.3)])
    header(c, "slowdown circle 1.0 m, ratio 0.3")
    d = c.d
    d.clock(T0)
    d.scan([0.5])
    c.cmd("command (0.5, 0.1, -0.7)", x=0.5, y=0.1, w=-0.7)
    c.cmd("command (1e-5, 3.3, 0.123456789)", x=1e-5, y=3.3, w=0.123456789)
    c.cmd("zero command in the slowdown zone", x=0.0)
    d.clock(T0 + NS)
    c.cmd("zero command at + 1 s", x=0.0)
    d.clock(T0 + NS + 1)
    c.cmd("zero command at + 1 s + 1 ns", x=0.0)
    c.cmd("command 0.5")
    c.close()
    c = Case(exe, [circle(radius=1.0, action="slowdown", slowdown_ratio=0.0)])
    header(c, "slowdown circle, ratio 0.0")
    d = c.d
    d.clock(T0)
    d.scan([0.5])
    c.cmd("command 0.5")
    c.cmd("command -0.5", x=-0.5)
    d.clock(T0 + NS + 1)
    c.cmd("command 0.5 at + 1 s + 1 ns")
    c.close()
    c = Case(exe, [circle("A", radius=1.0, action="slowdown", slowdown_ratio=1.5),
                   circle("B", radius=1.0, action="slowdown", slowdown_ratio=0.9),
                   circle("C", radius=0.3, action="stop", min_points=0, enabled="false")])
    header(c, "ratios 1.5 and 0.9; a disabled stop zone with min_points 0")
    d = c.d
    d.clock(T0)
    d.scan([0.5])
    c.cmd("command 0.5")
    c.close()
    c = Case(exe, [circle(radius=0.3, min_points=0)])
    header(c, "stop circle with min_points 0")
    d = c.d
    d.clock(T0)
    d.scan([5.0])
    c.cmd("nothing near")
    c.close()


def case_frame(exe):
    """A scan in a frame other than base_frame_id, without and with a transform."""
    c = Case(exe, [circle(radius=0.5)])
    header(c, "scan in frame laser_link")
    d = c.d
    d.clock(T0)
    d.scan([5.0])
    c.cmd("scan in the base frame")
    d.scan([5.0], frame="laser_link")
    c.cmd("scan in laser_link, no transform published; clock moved on 1 s after the command",
          advance=NS)
    d.scan([5.0], frame="laser_link")
    tf = TransformStamped()
    tf.header.frame_id = "base_footprint"
    tf.child_frame_id = "laser_link"
    tf.transform.translation.x = 0.1
    tf.transform.rotation.w = 1.0
    d.tf_pub.publish(TFMessage(transforms=[tf]))
    d.spin(1.0)
    d.scan([5.0], frame="laser_link")
    c.cmd("the same after a static transform is published", expect_diff="D11")
    d.scan([0.35], frame="laser_link")
    c.cmd("point 0.35 m ahead of the sensor (0.45 m in the base frame)", expect_diff="D11")
    c.close()
    c = Case(exe, [circle(radius=0.5)], lean_extra=["scan.roslean_sensor_pose:=[0.1, 0.0, 0.0]"])
    header(c, "the same with scan.roslean_sensor_pose [0.1, 0, 0] for the Lean node")
    d = c.d
    d.clock(T0)
    d.tf_pub.publish(TFMessage(transforms=[tf]))
    d.spin(1.0)
    for r, exp in [(0.35, None), (0.39, None), (0.4, "D4"), (0.405, None), (5.0, None)]:
        d.scan([r], frame="laser_link")
        c.cmd(f"point {r} m ahead of the sensor", expect_diff=exp)
    c.close()


def case_example_pose(exe):
    """config/example.yaml sets `roslean_sensor_pose: [0.0, 0.0, 0.0]`. With that
    line the Lean node uses a scan whatever its frame. Here the sensor is
    mounted backwards (yaw pi), which the transform tells upstream."""
    front = zone("Zone", type='"polygon"', action_type='"stop"', min_points=1,
                 points='"[[0.5, 0.3], [0.5, -0.3], [0.0, -0.3], [0.0, 0.3]]"')
    c = Case(exe, [front], lean_extra=["scan.roslean_sensor_pose:=[0.0, 0.0, 0.0]"])
    header(c, "stop box ahead of the robot; Lean node with the pose line of example.yaml")
    d = c.d
    d.clock(T0)
    tf = TransformStamped()
    tf.header.frame_id = "base_footprint"
    tf.child_frame_id = "laser_link"
    tf.transform.rotation.z = 1.0
    tf.transform.rotation.w = 0.0
    d.tf_pub.publish(TFMessage(transforms=[tf]))
    d.spin(1.0)
    d.scan([0.3], angle_min=math.pi, frame="laser_link")
    c.cmd("obstacle 0.3 m ahead of the robot (behind the backwards sensor)",
          expect_diff="lost stop: the example's pose line")
    d.scan([0.3], angle_min=0.0, frame="laser_link")
    c.cmd("obstacle 0.3 m behind the robot", expect_diff="false stop: the example's pose line")
    c.close()


def case_source_off(exe):
    """A disabled source."""
    c = Case(exe, [circle(radius=0.5)], source="      enabled: False")
    header(c, "scan.enabled: False")
    d = c.d
    d.clock(T0)
    c.cmd("no scan")
    d.scan([0.2])
    c.cmd("point inside the stop circle")
    c.close()


def rss_kb(proc):
    out = subprocess.run(["ps", "-o", "rss=", "-p", str(proc.pid)], capture_output=True, text=True)
    return out.stdout.strip() or "?"


def case_huge(exe):
    """Large scans over ROS. Whether a scan arrived is seen from its effect:
    its last reading is inside the stop circle. (The subscription is best
    effort; a scan that the middleware drops reaches neither node.)"""
    c = Case(exe, [circle(radius=0.5)])
    header(c, "stop circle 0.5 m; large scans")
    d = c.d
    d.clock(T0)
    for n in (20_000, 100_000, 1_000_000):
        d.scan([5.0])
        c.cmd("small scan, nothing near")
        big = [5.0] * n
        big[-1] = 0.2
        d.scan(big, inc=1e-6)
        d.spin(3.0)
        c.cmd(f"{n} readings, the last one at 0.2 m")
        print(f"    resident memory: lean {rss_kb(c.procs['lean'])} kB, "
              f"upstream {rss_kb(c.procs['up'])} kB")
    c.close()


def case_wall(exe):
    """Wall-clock time (use_sim_time false): no boundary is probed."""
    c = Case(exe, [circle(radius=0.5)], source_timeout=1.0, sim=False)
    header(c, "wall clock, source_timeout 1 s")
    d = c.d

    def wall_scan(ranges):
        d.now_ns = d.get_clock().now().nanoseconds
        d.scan(ranges)

    c.cmd("no scan yet")
    wall_scan([5.0])
    c.cmd("fresh scan, nothing near")
    wall_scan([0.2])
    c.cmd("fresh scan, point inside")
    wall_scan([5.0])
    c.cmd("fresh scan, nothing near")
    d.spin(1.5)
    c.cmd("1.5 s later, no new scan")
    c.close()


def case_dynamic(exe):
    """Things that change upstream at run time: polygon_sub_topic next to static
    points, and the `enabled` parameter of a zone."""
    c = Case(exe, [box(h=0.5, polygon_sub_topic='"zone_poly"')])
    header(c, "static square of 0.5 m with polygon_sub_topic set")
    d = c.d
    d.clock(T0)
    d.scan([1.0])
    c.cmd("point at 1.0 m")
    m = PolygonStamped()
    m.header.frame_id = "base_footprint"
    for x, y in [(2.0, 2.0), (2.0, -2.0), (-2.0, -2.0), (-2.0, 2.0)]:
        m.polygon.points.append(Point32(x=x, y=y))
    print(f"    subscribers on the polygon topic: {d.poly_pub.get_subscription_count()}")
    d.poly_pub.publish(m)
    d.spin(0.5)
    c.cmd("the same after a 2 m square is published on the topic")
    c.close()
    c = Case(exe, [circle(radius=0.5, enabled="false")])
    header(c, "stop circle with enabled: false, then `Zone.enabled` set to true at run time")
    d = c.d
    d.clock(T0)
    d.scan([0.2])
    c.cmd("point inside, zone disabled")
    print(f"    set Zone.enabled true: lean {d.set_param('collision_monitor', 'Zone.enabled', True)}"
          f" | upstream {d.set_param('cm_up', 'Zone.enabled', True)}")
    d.spin(0.3)
    c.cmd("point inside after the parameter change",
          expect_diff="D2, but the change is reported as accepted")
    c.close()
    c = Case(exe, [circle(radius=0.5)])
    header(c, "stop circle, then `Zone.enabled` set to false and `scan.enabled` to false at run time")
    d = c.d
    d.clock(T0)
    d.scan([0.2])
    c.cmd("point inside")
    print(f"    set Zone.enabled false: lean {d.set_param('collision_monitor', 'Zone.enabled', False)}"
          f" | upstream {d.set_param('cm_up', 'Zone.enabled', False)}")
    d.spin(0.3)
    c.cmd("point inside after the parameter change",
          expect_diff="D2, but the change is reported as accepted")
    c.close()


def case_stamped(exe):
    """TwistStamped: the header is copied."""
    c = Case(exe, [circle(radius=0.5)], stamped=True)
    header(c, "enable_stamped_cmd_vel")
    d = c.d
    d.clock(T0)
    c.cmd("no scan")
    d.scan([5.0])
    c.cmd("pass")
    d.scan([0.2])
    c.cmd("stop")
    c.close()


def case_qos(exe):
    """Topic names, types and QoS of both nodes."""
    c = Case(exe, [circle(radius=0.5)])
    header(c, "endpoints")
    d = c.d
    for topic in ("/cmd_in", "/scan", "/cmd_out_lean", "/cmd_out_up", "/clock", "/tf", "/tf_static"):
        infos = [("sub", i) for i in d.get_subscriptions_info_by_topic(topic)]
        infos += [("pub", i) for i in d.get_publishers_info_by_topic(topic)]
        for kind, i in infos:
            if i.node_name in ("collision_monitor", "cm_up"):
                q = i.qos_profile
                print(f"    {topic} {kind} {i.node_name}: {i.topic_type}, "
                      f"{q.reliability.name}, {q.durability.name}, {q.history.name} "
                      f"depth {q.depth}")
    names = sorted(n for n, _ in d.get_topic_names_and_types())
    print(f"    all topics: {names}")
    c.close()


CASES = {f[5:]: v for f, v in list(globals().items()) if f.startswith("case_")}


def main():
    exe = sys.argv[1]
    names = sys.argv[2:] or list(CASES)
    rclpy.init()
    for n in names:
        print(f"case {n}")
        try:
            CASES[n](exe)
        except Exception as e:  # one case must not hide the others
            print(f"  harness error {e!r}")
        sys.stdout.flush()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
