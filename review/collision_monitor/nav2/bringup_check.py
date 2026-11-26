#!/usr/bin/env python3
"""Review CM5 (task T08), item 5: the world around a headless Nav2 bring-up.

Publishes what a robot would (static transforms map -> odom -> base_footprint
-> base_link -> base_scan, a laser scan at 10 Hz) so that Nav2's servers can
activate, then drives the chain

    cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel

with a constant command, first with a clear scan and then with twenty
readings inside the footprint zone, and prints what arrived on `cmd_vel` and
on `collision_monitor_state`. Started by run_bringup.sh.
"""
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


class World(Node):
    def __init__(self):
        super().__init__('t08_world')
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf_static = self.create_publisher(TFMessage, '/tf_static', latched)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.out, self.states = [], []
        self.create_subscription(Twist, 'cmd_vel', self.out.append, 10)
        self.create_subscription(CollisionMonitorState, 'collision_monitor_state',
                                 self.states.append, 10)
        self.range = 3.0
        self.sending = False
        tfs = []
        for parent, child in (('map', 'odom'), ('odom', 'base_footprint'),
                              ('base_footprint', 'base_link'), ('base_link', 'base_scan')):
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.rotation.w = 1.0
            tfs.append(t)
        self.tf_static.publish(TFMessage(transforms=tfs))
        self.create_timer(0.1, self.tick)
        self.create_timer(0.05, self.tick_cmd)

    def tick(self):
        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_scan'
        m.angle_min, m.angle_increment = -0.5, 0.05
        m.range_min, m.range_max = 0.05, 10.0
        m.ranges = [self.range] * 20
        m.angle_max = m.angle_min + m.angle_increment * 19
        self.scan_pub.publish(m)

    def tick_cmd(self):
        if self.sending:
            m = Twist()
            m.linear.x = 0.2
            self.cmd_pub.publish(m)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def state_of(self, node):
        client = self.create_client(GetState, f'/{node}/get_state')
        if not client.wait_for_service(timeout_sec=2.0):
            return 'no service'
        future = client.call_async(GetState.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)
        return future.result().current_state.label if future.done() else 'no answer'


def main():
    wait_s = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    rclpy.init()
    w = World()
    servers = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
               'velocity_smoother', 'collision_monitor', 'bt_navigator', 'waypoint_follower']
    end = time.monotonic() + wait_s
    states = {}
    while time.monotonic() < end:
        w.spin_for(3.0)
        states = {n: w.state_of(n) for n in servers}
        print('  '.join(f'{n}={s}' for n, s in states.items()), flush=True)
        if all(s == 'active' for s in states.values()):
            break
    print('collision_monitor:', states.get('collision_monitor'))
    info = w.get_publishers_info_by_topic('cmd_vel')
    print('publishers of cmd_vel:', sorted(i.node_name for i in info))
    info = w.get_subscriptions_info_by_topic('cmd_vel_smoothed')
    print('subscribers of cmd_vel_smoothed:', sorted(i.node_name for i in info))

    def phase(label, seconds):
        o, s = len(w.out), len(w.states)
        w.spin_for(seconds)
        outs = w.out[o:]
        xs = sorted({round(m.linear.x, 3) for m in outs[len(outs) // 2:]})
        print(f'{label}: {len(outs)} messages on cmd_vel; linear.x in the second half: {xs}; '
              f'states: {[(m.polygon_name, m.action_type) for m in w.states[s:]]}', flush=True)

    w.sending = True
    phase('clear scan (3 m), command 0.2 m/s', 4.0)
    w.range = 0.15
    phase('twenty readings at 0.15 m, inside the footprint zone', 4.0)
    w.range = 3.0
    phase('clear again', 4.0)
    w.sending = False
    rclpy.shutdown()


if __name__ == '__main__':
    main()
