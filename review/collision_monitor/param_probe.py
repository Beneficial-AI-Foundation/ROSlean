#!/usr/bin/env python3
"""CM3/CM4 review: which parameter files each node accepts.

For every configuration: start the Lean node with the file and report whether
it runs or exits, with its fatal and warning lines; start upstream with the
same file and report whether `configure` succeeds.

    python3 param_probe.py <lean collision_monitor executable> [name ...]

Environment: CM_PREFIX is the prefix that holds upstream's binary and Nav2's
parameter files (run_side_by_side.sh sets it).
"""
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition

PREFIX = os.environ["CM_PREFIX"]
NAV2_CM = os.path.join(PREFIX, "share/nav2_collision_monitor/params/collision_monitor_params.yaml")
NAV2_BRINGUP = os.path.join(PREFIX, "share/nav2_bringup/params/nav2_params.yaml")

SUBSET = """
collision_monitor:
  ros__parameters:
    base_frame_id: "base_footprint"
    odom_frame_id: "odom"
    cmd_vel_in_topic: "cmd_vel_smoothed"
    cmd_vel_out_topic: "cmd_vel"
    state_topic: ""
    transform_tolerance: 0.5
    source_timeout: 5.0
    base_shift_correction: False
    stop_pub_timeout: 2.0
    polygons: ["PolygonStop", "Circle"]
    PolygonStop:
      type: "polygon"
      points: "[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]"
      action_type: "stop"
      min_points: 4
      visualize: False
      enabled: True
    Circle:
      type: "circle"
      radius: 0.6
      action_type: "slowdown"
      slowdown_ratio: 0.3
      min_points: 4
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "scan"
      enabled: True
"""


def read(path):
    with open(path) as f:
        return f.read()


def sub(text, *pairs):
    for old, new in pairs:
        if not re.search(old, text, flags=re.M):
            raise RuntimeError(f"pattern {old!r} not found")
        text = re.sub(old, new, text, count=1, flags=re.M)
    return text


def configs():
    nav2 = read(NAV2_CM)
    yield "Nav2 collision_monitor_params.yaml, unchanged", nav2
    yield ("Nav2 collision_monitor_params.yaml with state_topic emptied and "
           "base_shift_correction False"), sub(
        nav2, (r'state_topic: "collision_monitor_state"', 'state_topic: ""'),
        (r"base_shift_correction: True", "base_shift_correction: False"))
    yield ("the same, state_topic line removed"), sub(
        nav2, (r'^ *state_topic: "collision_monitor_state"\n', ''),
        (r"base_shift_correction: True", "base_shift_correction: False"))
    yield ("the same, all five zones listed in `polygons`"), sub(
        nav2, (r'^ *state_topic: "collision_monitor_state"\n', ''),
        (r"base_shift_correction: True", "base_shift_correction: False"),
        (r'polygons: \["PolygonStop"\]', 'polygons: ["PolygonStop", "PolygonSlow", '
         '"PolygonLimit", "FootprintApproach", "VelocityPolygonStop"]'))
    yield ("the same, scan and pointcloud both listed"), sub(
        nav2, (r'^ *state_topic: "collision_monitor_state"\n', ''),
        (r"base_shift_correction: True", "base_shift_correction: False"),
        (r'observation_sources: \["scan"\]', 'observation_sources: ["scan", "pointcloud"]'))
    yield "nav2_bringup nav2_params.yaml, unchanged", read(NAV2_BRINGUP)
    yield "subset file", SUBSET
    s = SUBSET
    yield "subset: `points` as a list of numbers (older syntax)", sub(
        s, (r'points: ".*"', "points: [0.3, 0.3, 0.3, -0.3, 0.0, -0.3, 0.0, 0.3]"))
    yield "subset: radius: 1 (integer)", sub(s, (r"radius: 0.6", "radius: 1"))
    yield "subset: min_points: 4.0", sub(s, (r"min_points: 4$", "min_points: 4.0"))
    yield 'subset: enabled: "true" (string)', sub(s, (r"enabled: True$", 'enabled: "true"'))
    yield "subset: the same zone listed twice", sub(
        s, (r'polygons: \["PolygonStop", "Circle"\]', 'polygons: ["Circle", "Circle"]'))
    yield "subset: a listed zone that is not defined", sub(
        s, (r'polygons: \["PolygonStop", "Circle"\]', 'polygons: ["PolygonStop", "Missing"]'))
    yield 'subset: points: ""', sub(s, (r'points: ".*"', 'points: ""'))
    yield "subset: source_timeout: 0.0", sub(s, (r"source_timeout: 5.0", "source_timeout: 0.0"))
    yield "subset: scan.enabled: False", sub(s, (r"enabled: True\n$", "enabled: False\n"))
    yield "subset: min_points: 0", sub(s, (r"min_points: 4$", "min_points: 0"))
    yield "subset: slowdown_ratio: 1.5", sub(s, (r"slowdown_ratio: 0.3", "slowdown_ratio: 1.5"))
    yield "subset: slowdown_ratio: -0.0", sub(s, (r"slowdown_ratio: 0.3", "slowdown_ratio: -0.0"))
    yield "subset: radius: 0.0", sub(s, (r"radius: 0.6", "radius: 0.0"))
    yield "subset: radius: 999.9995", sub(s, (r"radius: 0.6", "radius: 999.9995"))
    yield "subset: stop_pub_timeout: -1.0", sub(s, (r"stop_pub_timeout: 2.0", "stop_pub_timeout: -1.0"))
    yield "subset: source_timeout: -1.0", sub(s, (r"source_timeout: 5.0", "source_timeout: -1.0"))
    yield 'subset: action_type: "Stop"', sub(s, (r'action_type: "stop"', 'action_type: "Stop"'))
    yield "subset: no polygons at all (polygons: [\"\"])", sub(
        s, (r'polygons: \["PolygonStop", "Circle"\]', 'polygons: [""]'))
    yield "subset: polygon_sub_topic next to a radius", sub(
        s, (r"radius: 0.6", 'radius: 0.6\n      polygon_sub_topic: "r"'))
    yield "subset: footprint_topic next to static points, action stop", sub(
        s, (r'action_type: "stop"', 'action_type: "stop"\n      footprint_topic: "fp"'))
    yield "subset: four collinear vertices", sub(
        s, (r'points: ".*"', 'points: "[[0.0, 0.0], [0.1, 0.0], [0.2, 0.0], [0.3, 0.0]]"'))
    yield "subset: enable_stamped_cmd_vel: True and use_sim_time: True", sub(
        s, (r'state_topic: ""', 'state_topic: ""\n    enable_stamped_cmd_vel: True\n'
            '    use_sim_time: True'))
    yield "subset: a misspelled optional parameter (min_point: 1)", sub(
        s, (r"min_points: 4$", "min_point: 1"))
    yield "subset: sources_names: [\"scan\"]", sub(
        s, (r'action_type: "stop"', 'action_type: "stop"\n      sources_names: ["scan"]'))
    yield "subset under the key /**", sub(s, (r"^collision_monitor:", "/**:"))


class Probe(Node):
    def __init__(self):
        super().__init__("cm_param_probe")
        self.change = self.create_client(ChangeState, "collision_monitor/change_state")

    def transition(self, tid):
        req = ChangeState.Request()
        req.transition.id = tid
        fut = self.change.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        return fut.result().success if fut.result() is not None else None


def lines(text, *levels):
    out = []
    for l in text.splitlines():
        for lv in levels:
            if f"[{lv}]" in l:
                out.append(lv + ": " + l.split("]: ", 1)[-1].strip())
    return out


def run_lean(exe, path):
    proc = subprocess.Popen([exe, "--ros-args", "--params-file", path], stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    try:
        out, _ = proc.communicate(timeout=3)
        state = f"exits with code {proc.returncode}"
    except subprocess.TimeoutExpired:
        proc.send_signal(signal.SIGINT)
        out, _ = proc.communicate(timeout=10)
        state = "runs"
    return state, lines(out, "FATAL", "ERROR", "WARN")


def run_upstream(probe, path):
    exe = os.path.join(PREFIX, "lib", "nav2_collision_monitor", "collision_monitor")
    with tempfile.TemporaryFile("w+") as log:
        proc = subprocess.Popen(
            [exe, "--ros-args", "--params-file", path, "-p", "bond_heartbeat_period:=0.0"],
            stdout=log, stderr=subprocess.STDOUT)
        state = "no lifecycle service"
        if probe.change.wait_for_service(timeout_sec=15.0):
            conf = probe.transition(Transition.TRANSITION_CONFIGURE)
            act = probe.transition(Transition.TRANSITION_ACTIVATE) if conf else None
            state = f"configure={conf} activate={act}"
        if proc.poll() is not None:
            state += f", exited with code {proc.returncode}"
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.seek(0)
        text = log.read()
    time.sleep(0.5)
    return state, lines(text, "FATAL", "ERROR")[:3]


def main():
    exe = sys.argv[1]
    want = sys.argv[2:]
    rclpy.init()
    probe = Probe()
    for name, text in configs():
        if want and not any(w in name for w in want):
            continue
        f = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
        f.write(text)
        f.close()
        print(name)
        state, msgs = run_lean(exe, f.name)
        print(f"  lean: {state}")
        for m in msgs:
            print(f"    {m}")
        state, msgs = run_upstream(probe, f.name)
        print(f"  upstream: {state}")
        for m in msgs:
            print(f"    {m}")
        sys.stdout.flush()
    probe.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
