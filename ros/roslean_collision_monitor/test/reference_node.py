#!/usr/bin/env python3
"""The model of cm_model.py as a plain ROS node: a stand-in for a node under test.

    reference_node.py --ros-args --params-file zones.yaml -p polygons:=[...] ...

It takes upstream's parameters, is not a lifecycle node and uses no
transforms: a scan in another frame than `base_frame_id` is unusable. It
exists to check the test harness itself on a node that is not upstream's
(`scenario.py test/reference_node.py --no-tf`), and to show that the model
reproduces the fixed scenarios. It is not meant to be deployed.
"""

import json
import sys

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from cm_model import Model, Zone, scan_points

NS = 1_000_000_000


class ReferenceNode(Node):

    def __init__(self):
        super().__init__('collision_monitor', allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        def param(name, default):
            p = self.get_parameter_or(name, None)
            return default if p is None or p.value is None else p.value

        zones = []
        for name in param('polygons', []):
            kind = param(f'{name}.type', None)
            if kind == 'circle':
                geometry = param(f'{name}.radius', None)
            else:
                # "[[x, y], [x, y], ...]": at least four vertices, as upstream.
                geometry = json.loads(param(f'{name}.points', '[]'))
                if len(geometry) < 4:
                    raise ValueError(f'{name}: a polygon needs at least 4 points')
            zones.append(Zone(name, kind, geometry, param(f'{name}.action_type', None),
                              param(f'{name}.slowdown_ratio', 0.5), param(f'{name}.min_points', 4)))
        self.model = Model(zones, param('source_timeout', 2.0), param('stop_pub_timeout', 1.0))
        self.base_frame = param('base_frame_id', 'base_footprint')
        self.stamped = param('enable_stamped_cmd_vel', False)
        T = TwistStamped if self.stamped else Twist
        self.out_pub = self.create_publisher(T, param('cmd_vel_out_topic', 'cmd_vel'), 1)
        state_topic = param('state_topic', '')
        self.state_pub = (self.create_publisher(CollisionMonitorState, state_topic, 1)
                          if state_topic else None)
        self.create_subscription(
            LaserScan, param('scan.topic', 'scan'), self.on_scan, qos_profile_sensor_data)
        self.create_subscription(T, param('cmd_vel_in_topic', 'cmd_vel_smoothed'), self.on_cmd, 1)

    def on_scan(self, msg):
        stamp = msg.header.stamp.sec * NS + msg.header.stamp.nanosec
        points = None
        if msg.header.frame_id == self.base_frame:
            points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                 msg.range_min, msg.range_max)
        self.model.scan_in(stamp, points)

    def on_cmd(self, msg):
        t = msg.twist if self.stamped else msg
        self.model.clock(self.get_clock().now().nanoseconds)
        out, state = self.model.cmd((t.linear.x, t.linear.y, t.linear.z,
                                     t.angular.x, t.angular.y, t.angular.z))
        if state is not None and self.state_pub is not None:
            self.state_pub.publish(
                CollisionMonitorState(polygon_name=state[0], action_type=state[1]))
        if out is not None:
            m = TwistStamped(header=msg.header) if self.stamped else Twist()
            o = m.twist if self.stamped else m
            o.linear.x, o.linear.y, o.angular.z = out[0], out[1], out[5]
            self.out_pub.publish(m)


def main():
    rclpy.init()
    try:
        node = ReferenceNode()
    except Exception as e:  # a configuration the node refuses
        print(f'refusing to start: {e}', file=sys.stderr)
        sys.exit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
