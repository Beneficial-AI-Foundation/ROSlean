#!/usr/bin/env python3
"""Stand-in for a planner: drives in a circle at 10 Hz on `nav_vel`."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('demo_planner')
    pub = node.create_publisher(Twist, 'nav_vel', 10)
    cmd = Twist()
    cmd.linear.x = 1.0
    cmd.angular.z = 0.6
    node.create_timer(0.1, lambda: pub.publish(cmd))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
