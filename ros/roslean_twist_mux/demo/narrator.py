#!/usr/bin/env python3
"""Prints who is driving, and the theorem that explains each change.

Watches the inputs, the lock and the mux output. A message on the output is
attributed to the input whose latest message it equals. The theorem names
refer to ROSlean/Mux/Properties.lean and ROSlean/Mux/Trace.lean.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool

INPUTS = {'planner': 'nav_vel', 'teleop': 'key_vel'}
SILENCE = 0.4  # s without output before reporting that nothing is forwarded
GRACE = 0.3    # s after a lock change during which in-flight outputs are ignored

EXPLANATION = {
    # (driver, e-stop locked) -> text
    ('planner', False):
        'planner drives: it is the only live input and no lock is active\n'
        '    theorem forward_of_highest_live',
    ('teleop', False):
        'teleop drives; planner messages are dropped while teleop is live\n'
        '    theorems forward_of_highest_live, run_not_forward_of_higher_live',
    (None, True):
        'nothing is forwarded: the e-stop lock (255) is above every input\n'
        '    theorem not_forward_of_top_lock',
    (None, False):
        'nothing is forwarded: no input is publishing (the mux sends no stop command;\n'
        '    stopping is left to the base, see docs/SPEC.md R13)',
    ('waiting', False):
        'nothing is forwarded: teleop went silent but counts as live until its 0.5 s\n'
        '    timeout has passed, so the planner is still suppressed\n'
        '    theorem inputLive_iff_le',
}


class Narrator(Node):

    def __init__(self):
        super().__init__('demo_narrator')
        self.latest = {}
        self.heard = {}
        self.locked = False
        self.driver = 'unknown'
        self.last_out = self.get_clock().now()
        self.lock_changed = self.get_clock().now()
        for name, topic in INPUTS.items():
            self.create_subscription(Twist, topic, lambda msg, name=name: self.on_in(name, msg), 10)
        self.create_subscription(Bool, 'e_stop', self.on_lock, 10)
        self.create_subscription(Twist, 'cmd_vel_out', self.on_out, 10)
        self.create_timer(0.1, self.on_timer)

    def report(self, driver):
        if driver != self.driver:
            self.driver = driver
            text = EXPLANATION.get(
                (driver, self.locked),
                f'UNEXPECTED: {driver} is forwarded while the e-stop lock is engaged')
            print(f'\n>>> {text}', flush=True)

    def on_in(self, name, msg):
        self.latest[name] = msg
        self.heard[name] = self.get_clock().now()

    def recently(self, name, seconds):
        t = self.heard.get(name)
        return t is not None and (self.get_clock().now() - t).nanoseconds < seconds * 1e9

    def on_lock(self, msg):
        if msg.data != self.locked:
            self.locked = msg.data
            print(f"\n>>> e-stop lock {'ENGAGED' if msg.data else 'released'}", flush=True)
            self.driver = 'unknown'
            self.last_out = self.lock_changed = self.get_clock().now()

    def on_out(self, msg):
        self.last_out = self.get_clock().now()
        # The narrator may hear a lock message before the mux does, and a
        # message forwarded just before the lock may still be in flight.
        if (self.last_out - self.lock_changed).nanoseconds < GRACE * 1e9:
            return
        # Prefer the highest-priority match if both inputs sent equal messages.
        for name in ('teleop', 'planner'):
            if self.latest.get(name) == msg:
                self.report(name)
                return

    def on_timer(self):
        if (self.get_clock().now() - self.last_out).nanoseconds < SILENCE * 1e9:
            return
        if self.locked:
            self.report(None)
        elif self.recently('teleop', 1.0) and self.recently('planner', 0.3):
            self.report('waiting')
        elif not self.recently('planner', 0.3) and not self.recently('teleop', 0.3):
            self.report(None)


def main():
    rclpy.init()
    try:
        rclpy.spin(Narrator())
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
