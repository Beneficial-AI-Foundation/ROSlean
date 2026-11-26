#!/usr/bin/env python3
"""Runs the demo's box scenario for the video and tells the recorder what happened.

The scenario is the demo's own: `scenario()` of
ros/roslean_collision_monitor/demo/gazebo/obstacle.py (what `obstacle.py run`
and `pixi run cm-gazebo-scenario` execute). This node calls it and publishes
each of its events ('dropped' with the box's pose, 'rest', 'removed',
'moving', 'missed') on /video/event as JSON with the simulated time. After
the scenario it lets the robot drive on for TAIL seconds and publishes 'end'.

Two differences from `obstacle.py run`, for filming:
- The scenario starts AFTER seconds (simulated) into the run instead of at
  once. The robot is then past the start of the first leg of its patrol, so
  the box is dropped on the second leg, and the video has the first leg to
  show the robot driving freely.
- obstacle.py's waits give up after three times their length in real time,
  which a simulation slowed down for the camera exceeds. Here they are
  bounded in simulated time only.

Usage: scenario.py OBSTACLE_PY
"""

import importlib.util
import json
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

AFTER = 12.0     # s (simulated) before the scenario starts
TAIL = 16.0      # s (simulated) recorded after the robot drives on

spec = importlib.util.spec_from_file_location('obstacle', sys.argv[1])
obstacle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(obstacle)


class Scenario(obstacle.Observer):

    def __init__(self):
        super().__init__('video_scenario')
        self.events = self.create_publisher(String, 'video/event', 20)

    def spin_until(self, condition, timeout):
        """As Observer.spin_until, with the timeout in simulated time only."""
        start = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
            t = self.now()
            if t > 0.0 and (start is None or t < start):
                start = t
            if start is not None and t - start > timeout:
                return False
        return False

    def on_event(self, name, **details):
        if name == 'dropped':
            details['size'] = obstacle.SIZE
        self.events.publish(String(data=json.dumps(dict(details, name=name, t=self.now()))))
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    node = Scenario()
    try:
        node.spin_until(lambda: node.now() > AFTER, 1e9)
        ok = obstacle.scenario(node, on_event=node.on_event)
        t = node.now()
        node.spin_until(lambda: False, TAIL if ok else 1.0)
        node.on_event('end', ok=ok, tail=node.now() - t)
        node.spin_until(lambda: False, 1.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if obstacle.box_exists():
            obstacle.remove_box()


if __name__ == '__main__':
    main()
