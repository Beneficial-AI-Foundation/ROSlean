#!/usr/bin/env python3
"""Stand-in for the robot base's own command watchdog.

twist_mux forwards messages and never invents one: when every input is silent
or blocked it publishes nothing (docs/SPEC.md, R13 and assumption A3). A real
base stops when commands stop arriving; Gazebo's differential drive does not,
it keeps executing the last command. This node supplies that behaviour: it
passes the mux output on to the robot and, once the mux has been silent for
TIMEOUT seconds, sends zero velocity until commands resume.

It is part of the simulated robot, not of the mux, and is not verified.
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

TIMEOUT = 0.3  # s of simulated time


class BaseWatchdog(Node):

    def __init__(self):
        super().__init__('base_watchdog')
        self.pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.create_subscription(TwistStamped, 'cmd_vel_mux', self.on_cmd, 10)
        self.last = None
        self.create_timer(0.1, self.on_timer)

    def on_cmd(self, msg):
        self.last = self.get_clock().now()
        self.pub.publish(msg)

    def on_timer(self):
        now = self.get_clock().now()
        if self.last is None or (now - self.last).nanoseconds > TIMEOUT * 1e9:
            stop = TwistStamped()
            stop.header.stamp = now.to_msg()
            self.pub.publish(stop)


def main():
    rclpy.init()
    try:
        rclpy.spin(BaseWatchdog())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
