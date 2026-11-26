#!/usr/bin/env python3
"""Records one take of the collision monitor demo: what went into the monitor,
what came out of it, what it reported, what the lidar saw, where the robot
was, and (if the world has a camera) the camera's frames.

Output directory (parameter `out`):
  trace.json          see below
  frames/NNNNNN.jpg   one file per camera frame
  frames.json         {"frames": [t...], "size": [w, h]}: the time of each frame

The launch file starts this node twice: once for the messages, once with the
parameter `camera` set for the frames only, so that writing large images does
not delay the arrival times noted for the messages.

trace.json, all times in seconds of simulated time:
  "pose":   [[t, x, y, yaw]...]            the robot's true pose (ground_truth)
  "in":     [[t, sec, nsec, vx, vy, wz]...]  the monitor's input, cmd_vel_mux:
                                           arrival time, header stamp, velocity
  "out":    [[t, sec, nsec, vx, vy, wz]...]  the monitor's output, cmd_vel_monitored
  "base":   [[t, vx, wz]...]               what the base watchdog sent to the robot
  "state":  [[t, action_type, polygon_name]...]  the monitor's state topic
  "scan":   {"angle_min", "angle_increment", "range_min", "range_max",
             "t": [stamp...], "ranges": [[mm...]...]}   0 = no return
  "events": [{"name", "t", ...}...]        from scenario.py; 'dropped' has the
                                           box's pose (x, y, yaw)

Nothing is interpreted here; build_data.py pairs outputs with inputs.
"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def row(t, msg):
    return [round(t, 4), msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z]


class Recorder(Node):

    def __init__(self):
        super().__init__('video_recorder')
        self.out = self.declare_parameter('out', 'take').value
        self.camera = self.declare_parameter('camera', False).value
        self.create_subscription(String, 'video/event', self.on_event, 20)
        if self.camera:
            os.makedirs(os.path.join(self.out, 'frames'), exist_ok=True)
            self.trace = {'frames': [], 'events': []}
            self.create_subscription(Image, 'video/image', self.on_image, 30)
            return
        os.makedirs(self.out, exist_ok=True)
        self.trace = {'pose': [], 'in': [], 'out': [], 'base': [], 'state': [], 'events': [],
                      'scan': {'t': [], 'ranges': []}}
        T = self.trace
        self.create_subscription(TFMessage, 'ground_truth', self.on_truth, 100)
        self.create_subscription(TwistStamped, 'cmd_vel_mux',
                                 lambda m: T['in'].append(row(self.now(), m)), 100)
        self.create_subscription(TwistStamped, 'cmd_vel_monitored',
                                 lambda m: T['out'].append(row(self.now(), m)), 100)
        self.create_subscription(
            TwistStamped, 'cmd_vel',
            lambda m: T['base'].append([round(self.now(), 4), m.twist.linear.x, m.twist.angular.z]),
            100)
        self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state',
            lambda m: T['state'].append([round(self.now(), 4), m.action_type, m.polygon_name]), 100)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        # The robot is the only moving model; its pose comes first (see patrol.py).
        if not msg.transforms:
            return
        tf = msg.transforms[0]
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = stamp(tf) or self.now()
        if self.trace['pose'] and t <= self.trace['pose'][-1][0]:
            return
        self.trace['pose'].append([round(t, 4), round(p.x, 4), round(p.y, 4), round(yaw, 4)])

    def on_scan(self, msg):
        s = self.trace['scan']
        s.update(angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                 range_min=msg.range_min, range_max=msg.range_max, frame=msg.header.frame_id)
        s['t'].append(round(stamp(msg), 4))
        s['ranges'].append([int(round(r * 1000)) if msg.range_min <= r <= msg.range_max else 0
                            for r in msg.ranges])

    def on_event(self, msg):
        event = json.loads(msg.data)
        self.trace['events'].append(event)
        self.get_logger().info(f'event {msg.data}')
        if event['name'] == 'end':
            self.finish()

    def on_image(self, msg):
        from PIL import Image as PILImage
        n = len(self.trace['frames'])
        image = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
        image.save(os.path.join(self.out, 'frames', f'{n:06d}.jpg'), quality=95, subsampling=0)
        self.trace['frames'].append(round(stamp(msg), 4))
        self.trace['size'] = [msg.width, msg.height]
        if n % 60 == 0:
            self.get_logger().info(f'frame {n} at {stamp(msg):.2f} s')

    def finish(self):
        name = 'frames.json' if self.camera else 'trace.json'
        with open(os.path.join(self.out, name), 'w') as f:
            json.dump(self.trace, f)
        self.get_logger().info(name + ' written')
        raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Recorder())
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass


if __name__ == '__main__':
    main()
