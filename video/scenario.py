#!/usr/bin/env python3
"""Plays the operator and the e-stop for the demo video, on simulated time.

The planner (the demo's patrol.py) runs by itself. This node waits until the
mux has forwarded its first message, calls that moment plus LEAD the start
of the video (t = 0), and then:

  TELEOP  publishes on key_vel at 10 Hz what a held teleop key would send;
  ESTOP   engages the e_stop lock, and releases it again.

The start time is published once on /video/t0 (seconds of simulated time) so
that the recorder can relate frames and messages to the storyboard.
"""

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float64

LEAD = 1.0
# (from, to, linear m/s, angular rad/s), in seconds after t = 0.
TELEOP = [
    (21.0, 22.5, 0.10, -0.90),   # swing away from the patrol route
    (22.5, 27.5, 0.20, 0.00),    # straight across the room
    (27.5, 30.0, 0.16, 0.70),    # and a curve
]
ESTOP = (39.0, 47.0)
END = 62.0


class Scenario(Node):

    def __init__(self):
        super().__init__('video_scenario')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.key = self.create_publisher(TwistStamped, 'key_vel', 10)
        self.lock = self.create_publisher(Bool, 'e_stop', 10)
        self.t0_pub = self.create_publisher(Float64, 'video/t0', latched)
        self.create_subscription(TwistStamped, 'cmd_vel_mux', self.on_out, 10)
        self.t0 = None
        self.locked = None
        self.create_timer(0.1, self.on_timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_out(self, _):
        if self.t0 is None:
            self.t0 = self.now() + LEAD
            self.t0_pub.publish(Float64(data=self.t0))
            self.get_logger().info(f't0 = {self.t0:.3f}')

    def on_timer(self):
        if self.t0 is None:
            return
        t = self.now() - self.t0
        for start, end, lin, ang in TELEOP:
            if start <= t < end:
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = 'base_link'
                cmd.twist.linear.x, cmd.twist.angular.z = lin, ang
                self.key.publish(cmd)
        locked = ESTOP[0] <= t < ESTOP[1]
        if locked != self.locked and (locked or self.locked is not None):
            self.lock.publish(Bool(data=locked))
        self.locked = locked
        if t > END:
            raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Scenario())
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass


if __name__ == '__main__':
    main()
