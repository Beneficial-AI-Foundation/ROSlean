#!/usr/bin/env python3
"""Probes of the upstream nav2_collision_monitor binary for the CM1/CM2 review.

Each case starts a fresh `collision_monitor` process with its own parameter
file, drives it through its lifecycle, publishes scans and commands, and
prints what came out. Wall-clock time; the cases keep well away from timeout
boundaries.

Run with a ROS 2 Jazzy environment that contains nav2_collision_monitor
(see run_upstream_probe.sh):

    python3 upstream_probe.py [case ...]
"""
import math
import os
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from nav2_msgs.msg import CollisionMonitorState

BASE = """
collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_in"
    cmd_vel_out_topic: "cmd_out"
    state_topic: "cm_state"
    transform_tolerance: 0.1
    source_timeout: {source_timeout}
    base_shift_correction: False
    stop_pub_timeout: {stop_pub_timeout}
    bond_heartbeat_period: 0.0
    polygons: ["Zone"]
    Zone:
{zone}
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
"""

CIRCLE_STOP = """      type: "circle"
      radius: {radius}
      action_type: "stop"
      min_points: {min_points}
"""
CIRCLE_SLOW = """      type: "circle"
      radius: 0.3
      action_type: "slowdown"
      slowdown_ratio: {ratio}
      min_points: 1
"""
POLY_STOP = """      type: "polygon"
      points: "{points}"
      action_type: "stop"
      min_points: 1
"""
BOX = "[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]"


class Probe(Node):
    def __init__(self):
        super().__init__("cm_review_probe")
        self.out = []
        self.states = []
        self.cmd_pub = self.create_publisher(Twist, "cmd_in", QoSProfile(depth=1))
        self.scan_pub = self.create_publisher(LaserScan, "scan", qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_out", lambda m: self.out.append(m), 10)
        self.create_subscription(
            CollisionMonitorState, "cm_state", lambda m: self.states.append(m), 10)
        self.change = self.create_client(ChangeState, "collision_monitor/change_state")

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def transition(self, tid):
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.change.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return fut.result().success if fut.result() is not None else None

    def scan(self, ranges, frame="base_footprint", stamp_offset=0.0, sec=None,
             range_min=0.0, range_max=100.0, angle_min=0.0, inc=0.01):
        m = LaserScan()
        if sec is None:
            t = self.get_clock().now().nanoseconds + int(stamp_offset * 1e9)
            m.header.stamp.sec = t // 1000000000
            m.header.stamp.nanosec = t % 1000000000
        else:
            m.header.stamp.sec = sec
        m.header.frame_id = frame
        m.angle_min = angle_min
        m.angle_increment = inc
        m.range_min = range_min
        m.range_max = range_max
        m.ranges = [float(r) for r in ranges]
        # best effort: send a few times, the node keeps the last one
        for _ in range(3):
            self.scan_pub.publish(m)
            self.spin(0.1)

    def cmd(self, x, y=0.0, w=0.0, wait=0.4):
        """Publish one command; return the messages published in answer."""
        self.out.clear()
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = float(x), float(y), float(w)
        self.cmd_pub.publish(m)
        self.spin(wait)
        return [(o.linear.x, o.linear.y, o.angular.z) for o in self.out]


def start(probe, zone, source_timeout=2.0, stop_pub_timeout=1.0):
    """Start upstream; returns (process, configured, activated)."""
    f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    f.write(BASE.format(zone=zone, source_timeout=source_timeout,
                        stop_pub_timeout=stop_pub_timeout))
    f.close()
    exe = os.path.join(os.environ["CM_PREFIX"], "lib", "nav2_collision_monitor",
                       "collision_monitor")
    log = open(f.name + ".log", "w")
    proc = subprocess.Popen([exe, "--ros-args", "--params-file", f.name],
                            stdout=log, stderr=subprocess.STDOUT)
    if not probe.change.wait_for_service(timeout_sec=20.0):
        return proc, None, None, f.name + ".log"
    conf = probe.transition(Transition.TRANSITION_CONFIGURE)
    act = probe.transition(Transition.TRANSITION_ACTIVATE) if conf else None
    # let publishers and subscribers match
    end = time.time() + 10.0
    while conf and act and time.time() < end and (
            probe.cmd_pub.get_subscription_count() == 0
            or probe.scan_pub.get_subscription_count() == 0
            or probe.count_publishers("cmd_out") == 0):
        probe.spin(0.1)
    probe.spin(0.5)
    probe.states.clear()
    return proc, conf, act, f.name + ".log"


def stop(proc):
    proc.terminate()
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
    time.sleep(0.5)


def tail(log, pattern):
    with open(log) as f:
        return [l.strip() for l in f if pattern in l][-2:]


def case_sanity(p):
    """Harness check: a point inside a stop circle stops, outside passes."""
    proc, c, a, _ = start(p, CIRCLE_STOP.format(radius=0.3, min_points=1))
    p.scan([0.2])
    near = p.cmd(0.5)
    p.scan([2.0])
    far = p.cmd(0.5)
    stop(proc)
    print(f"sanity: configured={c} activated={a}; point at 0.2 m -> {near}; at 2.0 m -> {far}")


def case_negative_radius(p):
    """R9: the sign of `radius` is lost by the squaring."""
    proc, c, a, _ = start(p, CIRCLE_STOP.format(radius=-0.3, min_points=1))
    p.scan([0.2])
    near = p.cmd(0.5)
    p.scan([2.0])
    far = p.cmd(0.5)
    stop(proc)
    print(f"negative_radius (-0.3): configured={c}; point at 0.2 m -> {near}; at 2.0 m -> {far}")


def case_idle_then_stop(p):
    """R15: zero commands for longer than stop_pub_timeout, then an obstacle
    and a non-zero command: is the stop published?"""
    proc, c, a, _ = start(p, CIRCLE_STOP.format(radius=0.3, min_points=1),
                          stop_pub_timeout=0.5)
    p.scan([2.0])
    idle = [len(p.cmd(0.0, wait=0.3)) for _ in range(6)]
    p.scan([0.2])
    first = p.cmd(0.5)
    second = p.cmd(0.5)
    states = [(s.action_type, s.polygon_name) for s in p.states]
    p.scan([2.0])
    clear = p.cmd(0.5)
    stop(proc)
    print(f"idle_then_stop: messages per zero command {idle}; then obstacle in stop zone, "
          f"command 0.5 -> {first}, again -> {second}; state messages {states}; "
          f"after clearing -> {clear}")


def case_zero_cmd_slowdown(p):
    """R13/D6: a zero command in a triggered slowdown zone."""
    proc, c, a, _ = start(p, CIRCLE_SLOW.format(ratio=0.5))
    p.scan([0.2])
    z = p.cmd(0.0)
    sz = [(s.action_type, s.polygon_name) for s in p.states]
    p.states.clear()
    m = p.cmd(0.5)
    sm = [(s.action_type, s.polygon_name) for s in p.states]
    stop(proc)
    print(f"zero_cmd_slowdown: zero command -> {z}, state messages {sz}; "
          f"command 0.5 -> {m}, state messages {sm}")


def case_negative_ratio(p):
    """R13: a negative slowdown_ratio reverses the command."""
    proc, c, a, _ = start(p, CIRCLE_SLOW.format(ratio=-0.5))
    p.scan([0.2])
    m = p.cmd(0.5)
    stop(proc)
    print(f"negative_ratio (-0.5): command 0.5 -> {m}")


def case_box_boundary(p):
    """R10: boundary points of upstream's example box."""
    proc, c, a, _ = start(p, POLY_STOP.format(points=BOX))
    res = {}
    # (0.3f, 0): on the front side. Range 0 at any angle: (0, 0), on the rear side.
    for name, r in [("front side (0.3, 0)", 0.3), ("rear side (0, 0)", 0.0),
                    ("interior (0.15, 0)", 0.15), ("outside (0.5, 0)", 0.5)]:
        p.scan([r])
        res[name] = p.cmd(0.5)
    stop(proc)
    print(f"box_boundary: configured={c}; {res}")


def case_triangle(p):
    """R10: three vertices are refused."""
    proc, c, a, log = start(p, POLY_STOP.format(points="[[0.3, 0.3], [0.3, -0.3], [0.0, 0.0]]"))
    stop(proc)
    print(f"triangle: configured={c}; log: {tail(log, 'olygon')}")


def case_negative_stamp(p):
    """D8: a scan stamp with negative seconds."""
    proc, c, a, log = start(p, CIRCLE_STOP.format(radius=0.3, min_points=1))
    p.scan([2.0])
    before = p.cmd(0.5)
    p.scan([2.0], sec=-1)
    after = p.cmd(0.5)
    p.spin(1.0)
    rc = proc.poll()
    stop(proc)
    print(f"negative_stamp: before -> {before}; after a scan stamped sec=-1 -> {after}; "
          f"process exit code {rc}; log: {tail(log, 'what()')}")


def case_other_frame(p):
    """D11: a scan in a frame other than base_frame_id, with no transform published."""
    proc, c, a, log = start(p, CIRCLE_STOP.format(radius=0.3, min_points=1))
    p.scan([2.0], frame="laser_link")
    m = p.cmd(0.5, wait=1.5)
    m2 = p.cmd(0.5, wait=1.5)
    states = [(s.action_type, s.polygon_name) for s in p.states]
    rc = proc.poll()
    stop(proc)
    print(f"other_frame: scan with frame_id laser_link, nothing within 2 m, command 0.5 -> {m}, "
          f"again -> {m2}; state messages {states}; exit code {rc}; "
          f"log: {tail(log, 'ransform')}")


def case_min_points_wrap(p):
    """D10: min_points is read into a 32-bit int."""
    proc, c, a, _ = start(p, CIRCLE_STOP.format(radius=0.3, min_points=4294967297))
    p.scan([0.2])
    m = p.cmd(0.5)
    stop(proc)
    print(f"min_points_wrap (2^32 + 1): configured={c}; one point inside -> {m}")


CASES = {f[5:]: v for f, v in list(globals().items()) if f.startswith("case_")}


def main():
    names = sys.argv[1:] or list(CASES)
    rclpy.init()
    probe = Probe()
    for n in names:
        try:
            CASES[n](probe)
        except Exception as e:  # keep going: one case must not hide the others
            print(f"{n}: harness error {e!r}")
        sys.stdout.flush()
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
