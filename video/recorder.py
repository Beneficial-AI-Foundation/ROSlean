#!/usr/bin/env python3
"""Records one take: camera frames as JPEG files and a trace of what the mux did.

Output directory (parameter `out`):
  frames/NNNNNN.jpg   one file per camera frame from t = -PRE on
  trace.json          {"t0", "frames": [t...], "pose": [[t, x, y, yaw]...],
                       "in": {"planner": [t...], "teleop": [t...]},
                       "out": [[t, "planner" | "teleop" | "?"]...],
                       "lock": [[t, bool]...], "camera": {...}}

All times are simulated seconds after t0 (see scenario.py). A message on the
mux output is attributed to the input that sent a message with the same
header stamp and the same velocity; the mux forwards messages unchanged.
"""

import json
import math
import os

import rclpy
from geometry_msgs.msg import TwistStamped
from PIL import Image as PILImage
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float64
from tf2_msgs.msg import TFMessage

PRE = 1.0
END = 61.0
INPUTS = {'planner': 'nav_vel', 'teleop': 'key_vel'}


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def key(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec,
            msg.twist.linear.x, msg.twist.angular.z)


class Recorder(Node):

    def __init__(self):
        super().__init__('video_recorder')
        self.out = self.declare_parameter('out', 'take').value
        os.makedirs(os.path.join(self.out, 'frames'), exist_ok=True)
        self.t0 = None
        self.trace = {'frames': [], 'pose': [], 'in': {n: [] for n in INPUTS}, 'out': [], 'lock': []}
        self.sent = {}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, 'video/t0', self.on_t0, latched)
        self.create_subscription(Image, 'video/image', self.on_image, 30)
        self.create_subscription(TFMessage, 'ground_truth', self.on_truth, 10)
        self.create_subscription(Bool, 'e_stop', self.on_lock, 10)
        self.create_subscription(TwistStamped, 'cmd_vel_mux', self.on_out, 50)
        for name, topic in INPUTS.items():
            self.create_subscription(TwistStamped, topic,
                                     lambda msg, name=name: self.on_in(name, msg), 50)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_t0(self, msg):
        self.t0 = msg.data

    def on_in(self, name, msg):
        self.sent[key(msg)] = name
        if self.t0 is not None:
            self.trace['in'][name].append(round(stamp(msg) - self.t0, 4))

    def on_out(self, msg):
        # Attributed in finish(): the output can reach this node before the input does.
        if self.t0 is not None:
            self.trace['out'].append([round(self.now() - self.t0, 4), key(msg)])

    def on_lock(self, msg):
        if self.t0 is not None:
            self.trace['lock'].append([round(self.now() - self.t0, 4), msg.data])

    def on_truth(self, msg):
        # The robot is the only moving model; its pose comes first (see patrol.py).
        if self.t0 is None or not msg.transforms:
            return
        tf = msg.transforms[0]
        p, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = self.now() - self.t0
        self.trace['pose'].append([round(t, 4), round(p.x, 4), round(p.y, 4), round(yaw, 4)])

    def on_image(self, msg):
        if self.t0 is None:
            return
        t = stamp(msg) - self.t0
        if t < -PRE:
            return
        if t > END:
            self.finish()
        n = len(self.trace['frames'])
        image = PILImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data))
        image.save(os.path.join(self.out, 'frames', f'{n:06d}.jpg'), quality=95, subsampling=0)
        self.trace['frames'].append(round(t, 4))
        self.trace['size'] = [msg.width, msg.height]
        if n % 60 == 0:
            self.get_logger().info(f'frame {n} at t = {t:.2f}')

    def finish(self):
        self.trace['t0'] = self.t0
        self.trace['out'] = [[t, self.sent.get(tuple(k), '?')] for t, k in self.trace['out']]
        with open(os.path.join(self.out, 'trace.json'), 'w') as f:
            json.dump(self.trace, f)
        self.get_logger().info('take written')
        raise SystemExit


def main():
    rclpy.init()
    try:
        rclpy.spin(Recorder())
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass


if __name__ == '__main__':
    main()
