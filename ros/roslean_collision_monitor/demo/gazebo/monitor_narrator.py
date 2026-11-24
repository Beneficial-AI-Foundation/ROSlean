#!/usr/bin/env python3
"""Prints what the collision monitor does to the command, and the theorem
that explains each change.

Watches the monitor's input (the mux output, `cmd_vel_mux`), its output
(`cmd_vel_monitored`) and the lidar. An output message is paired with the
input message that has the same header stamp (the monitor copies the header)
and classified by comparing the two velocities. (Nav2's velocity smoother
sends several messages with one stamp; the newest of them is taken unless an
older one equals the output.)

    equal                      -> the command passes
    a fraction of the command  -> slowdown
    zero, command not zero     -> stop

The narrator sees topics only. It cannot see which zone the node named, so
for a stop it tells the two possible causes apart by whether scans are
arriving. The theorem names refer to ROSlean/CollisionMonitor/Properties.lean
and Trace.lean; docs/collision_monitor/THEOREMS.md states each one.

Parameters, for the variant in which Nav2 starts the monitor
(cm_nav2_launch.py remaps the two velocity topics to Nav2's):
  watchdog        (true) whether a base watchdog follows the monitor
  state_topic     ('') the monitor's state topic; each message on it is printed
  source_timeout  (1.0) `source_timeout` of the monitor's configuration, s
"""

from collections import OrderedDict

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

SILENCE = 0.4         # s without the one or the other before saying so
SCAN_TIMEOUT = 1.0    # s, `source_timeout` in monitor.yaml (parameter source_timeout)
ACTIONS = {0: 'do nothing', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}

EXPLANATION = {
    'pass':
        'monitor: the command passes unchanged; no zone is occupied\n'
        '    theorems run_pass_through, velocity_eq_pass_iff',
    'slowdown':
        'monitor: SLOWDOWN, the command is published times {ratio:.2f}: the slowdown zone is\n'
        '    occupied and the stop zone is not\n'
        '    theorems velocity_eq_slowdown_iff, run_slowdown_bound, run_no_amplification',
    'stop':
        'monitor: STOP, zero is published instead of the command: scans are arriving, so\n'
        '    the stop zone is occupied\n'
        '    theorems evaluate_isStop_iff, run_stop, stop_not_pass_or_slowdown',
    'stop-source':
        'monitor: STOP, zero is published instead of the command: no scan has arrived for\n'
        '    {timeout} s, so the monitor has no usable data\n'
        '    theorems evaluate_isStop_iff, run_stale',
    'silent':
        'monitor: publishes NOTHING although commands arrive: it has published zero for\n'
        '    stop_pub_timeout and a stop is still demanded. The last message the base got\n'
        '    is a zero; {holder} (SPEC R15, A3)\n'
        '    theorems velocity_eq_none_iff, run_withheld_last_published_zero',
    'zero':
        'monitor: the command itself is zero and is passed on (for stop_pub_timeout, then\n'
        '    nothing: SPEC R15)\n'
        '    theorems pass_through, zero_period',
    'idle':
        'monitor: no command arrives, so nothing is published, whatever the lidar sees\n'
        '    (the monitor acts only on commands: SPEC R1)\n'
        '    theorem output_of_not_cmd',
}


def planar(twist):
    return (twist.linear.x, twist.linear.y, twist.angular.z)


class MonitorNarrator(Node):

    def __init__(self):
        super().__init__('demo_monitor_narrator')
        self.inputs = OrderedDict()   # header stamp -> velocities of recent commands
        self.holder = ('holding the robot is now the base watchdog\'s job'
                       if self.declare_parameter('watchdog', True).value else
                       'no watchdog follows the monitor here: the robot stands because\n'
                       '    the simulated base keeps executing that zero')
        self.scan_timeout = self.declare_parameter('source_timeout', SCAN_TIMEOUT).value
        state_topic = self.declare_parameter('state_topic', '').value
        if state_topic:
            self.create_subscription(CollisionMonitorState, state_topic, self.on_state, 50)
        self.state = None
        self.ratio = None
        self.last_in = self.last_out = self.last_scan = None
        self.last_in_zero = True
        # The monitor's own topics keep one message; the narrator keeps more,
        # so as not to miss any.
        self.create_subscription(TwistStamped, 'cmd_vel_mux', self.on_in, 50)
        self.create_subscription(TwistStamped, 'cmd_vel_monitored', self.on_out, 50)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)

    def report(self, state, ratio=None):
        changed = state != self.state
        # A different ratio is a different slowdown.
        if state == 'slowdown' and self.ratio is not None and abs(ratio - self.ratio) > 0.005:
            changed = True
        if changed:
            self.state, self.ratio = state, ratio
            print('\n>>> ' + EXPLANATION[state].format(
                ratio=ratio or 0.0, holder=self.holder, timeout=self.scan_timeout), flush=True)

    def on_state(self, msg):
        action = ACTIONS.get(msg.action_type, str(msg.action_type))
        zone = f", zone '{msg.polygon_name}'" if msg.polygon_name else ''
        print(f'\n    [the monitor\'s state topic] action: {action}{zone}', flush=True)

    def within(self, stamp, seconds):
        return stamp is not None and (self.get_clock().now() - stamp).nanoseconds < seconds * 1e9

    def on_scan(self, _msg):
        self.last_scan = self.get_clock().now()

    def on_in(self, msg):
        self.last_in = self.get_clock().now()
        self.last_in_zero = not any(planar(msg.twist))
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.inputs.setdefault(key, []).append(planar(msg.twist))
        while len(self.inputs) > 100:
            self.inputs.popitem(last=False)

    def on_out(self, msg):
        self.last_out = self.get_clock().now()
        cmds = self.inputs.get((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if cmds is None:
            return  # the command was sent before the narrator started
        out = planar(msg.twist)
        cmd = out if out in cmds else cmds[-1]
        if not any(cmd):
            self.report('zero')
        elif out == cmd:
            self.report('pass')
        elif not any(out):
            self.report('stop' if self.within(self.last_scan, self.scan_timeout) else 'stop-source')
        else:
            # The largest component of the command gives the ratio.
            i = max(range(3), key=lambda k: abs(cmd[k]))
            self.report('slowdown', out[i] / cmd[i])

    def on_timer(self):
        if self.last_in is None:
            return
        if not self.within(self.last_in, SILENCE):
            self.report('idle')
        elif (self.last_out is not None and not self.within(self.last_out, SILENCE)
              and not self.last_in_zero):
            self.report('silent')


def main():
    rclpy.init()
    try:
        rclpy.spin(MonitorNarrator())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
