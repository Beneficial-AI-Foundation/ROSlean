#!/usr/bin/env python3
"""Stand-in for a planner: drives a fixed loop through the apartment.

Publishes a velocity command on `nav_vel` at 10 Hz, steering towards the next
waypoint. It keeps publishing whether or not the mux forwards it; when it is
overridden and the robot ends up elsewhere, it simply heads for its current
waypoint again.

The robot's position is taken from the simulator's ground truth, standing in
for a localisation system. (Wheel odometry alone drifts by tens of degrees
within one lap here, because the wheels slip in turns.)

It does not plan around obstacles. The only sensing is a guard: if the lidar
sees something closer than 0.3 m ahead, it turns on the spot instead of
driving forward.
"""

import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

# A rectangle in the open middle of the room (world coordinates, metres).
WAYPOINTS = [(0.4, 0.2), (0.4, -2.5), (-2.0, -2.5), (-2.0, 0.2)]
REACHED = 0.15      # m
SPEED = 0.18        # m/s (the burger's limit is 0.22)
TURN_GAIN = 1.5
MAX_TURN = 0.8      # rad/s
GUARD = 0.30        # m


class Patrol(Node):

    def __init__(self):
        super().__init__('demo_patrol')
        self.pub = self.create_publisher(TwistStamped, 'nav_vel', 10)
        self.create_subscription(TFMessage, 'ground_truth', self.on_truth, 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pose = None
        self.blocked = False
        self.target = 0
        self.create_timer(0.1, self.on_timer)

    def on_truth(self, msg):
        # The bridge drops the model names. The robot is the only model in
        # the world that is not static, and a model's own pose comes before
        # those of its links, so the first entry is the robot.
        if msg.transforms:
            p, q = msg.transforms[0].transform.translation, msg.transforms[0].transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (p.x, p.y, yaw)

    def on_scan(self, msg):
        # Rays within 25 degrees of straight ahead (index 0 is ahead).
        n = len(msg.ranges)
        k = max(1, int(math.radians(25) / msg.angle_increment)) if msg.angle_increment else 1
        ahead = list(msg.ranges[:k]) + list(msg.ranges[n - k:])
        near = [r for r in ahead if msg.range_min < r < GUARD]
        self.blocked = bool(near)

    def on_timer(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        if self.pose is not None:
            x, y, yaw = self.pose
            tx, ty = WAYPOINTS[self.target]
            if math.hypot(tx - x, ty - y) < REACHED:
                self.target = (self.target + 1) % len(WAYPOINTS)
                tx, ty = WAYPOINTS[self.target]
            error = math.atan2(ty - y, tx - x) - yaw
            error = math.atan2(math.sin(error), math.cos(error))
            cmd.twist.angular.z = max(-MAX_TURN, min(MAX_TURN, TURN_GAIN * error))
            # Drive forward only when roughly facing the waypoint and clear.
            if abs(error) < 0.5 and not self.blocked:
                cmd.twist.linear.x = SPEED
            elif self.blocked and abs(error) < 0.5:
                cmd.twist.angular.z = 0.6
        self.pub.publish(cmd)


def main():
    rclpy.init()
    try:
        rclpy.spin(Patrol())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
