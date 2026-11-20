#!/usr/bin/env python3
"""Drives a twist_mux node through a fixed scenario over real ROS topics.

Starts the node given as `<package> <executable>`, publishes commands and lock
values, and records which commands come out on `cmd_vel_out`. Each command
carries a unique id in `twist.linear.x`, so forwarding is observed per
message. Prints one line per command and exits non-zero if any decision
differs from the expectation derived from docs/SPEC.md.

    scenario.py roslean_twist_mux twist_mux     # this repo's node
    scenario.py twist_mux twist_mux             # upstream, for comparison

All waits are long compared with the 0.4 s / 0.5 s timeouts in scenario.yaml,
so the outcome does not depend on scheduling jitter.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scenario.yaml')
SETTLE = 0.15  # s to wait for a forwarded message before deciding it was dropped


class Driver(Node):

    def __init__(self):
        super().__init__('scenario_driver')
        self.inputs = {
            name: self.create_publisher(TwistStamped, topic, 10)
            for name, topic in [('nav', 'nav_vel'), ('key', 'key_vel'), ('joy', 'joy_vel')]}
        self.locks = {
            name: self.create_publisher(Bool, topic, 10)
            for name, topic in [('watchdog', 'watchdog'), ('pause', 'pause_navigation'),
                                ('e_stop', 'e_stop')]}
        self.received = set()
        self.create_subscription(TwistStamped, 'cmd_vel_out', self.on_out, 10)
        self.next_id = 1
        self.results = []  # (label, expected, forwarded)

    def on_out(self, msg):
        self.received.add(int(msg.twist.linear.x))

    def spin_for(self, seconds, keep_watchdog=False):
        end = time.monotonic() + seconds
        last = 0.0
        while time.monotonic() < end:
            if keep_watchdog and time.monotonic() - last > 0.1:
                self.lock('watchdog', False, settle=0.0)
                last = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_connected(self, timeout=20.0):
        end = time.monotonic() + timeout
        pubs = list(self.inputs.values()) + list(self.locks.values())
        while time.monotonic() < end:
            if (all(p.get_subscription_count() > 0 for p in pubs)
                    and self.count_publishers('cmd_vel_out') > 0):
                self.spin_for(0.5)
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('node under test did not come up')

    def lock(self, name, value, settle=SETTLE):
        self.locks[name].publish(Bool(data=value))
        self.spin_for(settle)

    def cmd(self, name, expected, label):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(self.next_id)
        self.inputs[name].publish(msg)
        self.spin_for(SETTLE)
        forwarded = self.next_id in self.received
        self.results.append((label, expected, forwarded))
        self.next_id += 1


def scenario(d):
    # Wall clock: every stamp-0 handle has expired, so the watchdog lock
    # (timeout 0.4, never heard) is locked at priority 50 (R5, R6).
    d.cmd('nav', False, 'nav blocked: watchdog lock never heard')
    d.cmd('key', True, 'key (90) passes the watchdog lock (50)')
    d.spin_for(0.7)  # let key expire
    d.lock('watchdog', False)
    d.cmd('nav', True, 'nav drives once the watchdog reports false')
    d.cmd('key', True, 'key takes over')
    d.cmd('nav', False, 'nav suppressed while key is live')
    d.cmd('joy', True, 'joy takes over')
    d.cmd('key', False, 'key suppressed while joy is live')
    d.lock('pause', True)
    d.cmd('joy', True, 'pause lock (100) does not block joy (100)')
    d.cmd('key', False, 'key still suppressed')
    d.lock('e_stop', True)
    d.cmd('joy', False, 'e_stop lock (255) blocks joy')
    d.cmd('nav', False, 'e_stop lock blocks nav')
    d.lock('e_stop', False)
    d.cmd('joy', True, 'joy restored after e_stop released')
    d.spin_for(0.7, keep_watchdog=True)  # joy and key expire
    d.cmd('nav', False, 'nav blocked by pause lock (100) with joy and key expired')
    d.lock('pause', False)
    d.lock('watchdog', False)
    d.cmd('nav', True, 'nav drives: locks released, joy and key expired')
    d.spin_for(0.7)  # watchdog publisher silent past its timeout
    d.cmd('nav', False, 'nav blocked: watchdog publisher went silent')
    d.lock('watchdog', True)
    d.lock('watchdog', False)
    d.cmd('nav', True, 'nav drives again after watchdog reports false')


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    package, executable = sys.argv[1], sys.argv[2]
    proc = subprocess.Popen(
        ['ros2', 'run', package, executable, '--ros-args', '--params-file', CONFIG],
        start_new_session=True)  # own process group: `ros2 run` spawns the node as a child
    rclpy.init()
    d = Driver()
    try:
        d.wait_connected()
        scenario(d)
    finally:
        os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
        rclpy.shutdown()

    bad = 0
    for label, expected, forwarded in d.results:
        ok = expected == forwarded
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  {'forwarded' if forwarded else 'dropped  '}  {label}")
    print(f'DECISIONS {"".join("1" if f else "0" for _, _, f in d.results)}')
    print(f'{bad} mismatch(es) for {package}/{executable}')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
