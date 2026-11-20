#!/usr/bin/env python3
"""Drives a twist_mux node through fixed scenarios over real ROS topics.

The node runs with `use_sim_time` and this script publishes `/clock`, so the
node-clock time of every event is chosen exactly. Each command carries a
unique id, and the script records which commands come out on `cmd_vel_out`,
checks them against the expectation derived from docs/SPEC.md, and checks
that a forwarded message is identical to the one sent.

    scenario.py NODE            # check NODE against the expectations
    scenario.py NODE OTHER      # also require identical decisions from OTHER

NODE is a path to an executable, or `<package>/<executable>` (resolved with
`ros2 pkg prefix`), e.g. `roslean_twist_mux/twist_mux` or, for upstream,
`twist_mux/twist_mux`.

The only wall-clock dependence left is that messages must be delivered within
SETTLE seconds, and that a clock update reaches the node before the next
message does.
"""

import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

HERE = os.path.dirname(os.path.abspath(__file__))
SETTLE = 0.2  # s of wall time allowed for delivery
NS = 1_000_000_000


def s(seconds):
    """Seconds (a binary fraction) to integer nanoseconds."""
    ns = seconds * NS
    assert ns == int(ns)
    return int(ns)


# A scenario is a list of steps:
#   ('clock', t_ns)                    set the node clock
#   ('lock', name, value)              publish a lock value
#   ('cmd', name, expected, label)     publish a command; expect forwarded or not

MAIN = [
    # Clock far from 0: every never-heard handle with a timeout has expired,
    # as on a wall clock. So the watchdog lock is locked (R5, R6).
    ('clock', s(100)),
    ('cmd', 'nav', False, 'nav blocked: watchdog lock (50) never heard'),
    ('cmd', 'key', True, 'key (90) passes the watchdog lock (50)'),
    ('clock', s(101)),
    ('lock', 'watchdog', False),
    ('cmd', 'nav', True, 'nav drives: watchdog fresh, key expired'),
    ('cmd', 'key', True, 'key takes over'),
    ('cmd', 'nav', False, 'nav suppressed by live key (watchdog fresh)'),
    ('cmd', 'joy', True, 'joy takes over'),
    ('cmd', 'key', False, 'key suppressed by live joy'),
    ('lock', 'pause', True),
    ('cmd', 'joy', True, 'pause lock (100) does not block joy (100)'),
    ('cmd', 'key', False, 'key blocked'),
    ('lock', 'e_stop', True),
    ('cmd', 'joy', False, 'e_stop lock (255) blocks joy'),
    ('cmd', 'nav', False, 'e_stop lock blocks nav'),
    ('lock', 'e_stop', False),
    ('cmd', 'joy', True, 'joy restored after e_stop released'),
    ('lock', 'pause', False),
    # joy last heard at 101, timeout 0.5; watchdog last heard at 101, timeout 0.25.
    ('clock', s(101.25)),
    ('lock', 'watchdog', False),
    ('clock', s(101.5)),
    ('cmd', 'nav', False, 'joy still live exactly at its timeout'),
    ('lock', 'watchdog', False),
    ('clock', s(101.5) + 1),
    ('cmd', 'nav', True, 'joy and key expired 1 ns after their timeout'),
    ('clock', s(101.75)),
    ('cmd', 'nav', True, 'watchdog still fresh exactly at its timeout'),
    ('clock', s(101.75) + 1),
    ('cmd', 'nav', False, 'watchdog expired 1 ns after its timeout'),
    ('lock', 'watchdog', True),
    ('cmd', 'nav', False, 'watchdog reports true'),
    ('lock', 'watchdog', False),
    ('cmd', 'nav', True, 'watchdog reports false again'),
    ('cmd', 'bogus', False, 'message on an unconfigured topic'),
    # The clock jumps backwards (simulation reset): nothing expires (A2).
    ('clock', s(50)),
    ('cmd', 'nav', False, 'after a backwards jump joy (heard at 101) counts as live'),
    ('cmd', 'joy', True, 'joy drives after the jump'),
    ('clock', s(50.5) + 1),
    ('cmd', 'nav', False, 'watchdog is fresh but key (heard at 101) counts as live'),
    ('cmd', 'key', True, 'key drives: joy expired'),
]

START = [
    # Clock near 0: never-heard handles have NOT expired yet (R3, R4).
    ('clock', s(0.125)),
    ('cmd', 'nav', False, 'never-heard key and joy count as live near clock 0'),
    ('cmd', 'joy', True, 'joy drives; watchdog lock not yet expired'),
    ('clock', s(0.375)),
    ('cmd', 'key', False, 'key suppressed by live joy'),
    ('clock', s(0.75)),
    ('cmd', 'key', True, 'joy expired; key (90) passes the expired watchdog (50)'),
    ('cmd', 'nav', False, 'nav blocked by the expired watchdog'),
]

ODD = [
    ('clock', s(100)),
    ('cmd', 'low', False, 'never-heard inputs without a timeout suppress low (R5)'),
    ('cmd', 'zero', False, 'priority-0 input is never forwarded (R9)'),
    ('cmd', 'mid', False, 'mid suppressed by never-heard sticky (timeout 0)'),
    ('cmd', 'sticky', True, 'sticky (300, clamped to 255) drives'),
    ('lock', 'top', True),
    ('cmd', 'sticky', True, 'a 255 lock does not block a 255 input (R10a)'),
    ('cmd', 'mid', False, 'mid blocked'),
    ('clock', s(1000)),
    ('cmd', 'sticky', True, 'still driving 900 s later'),
    ('cmd', 'mid', False, 'sticky never expires'),
]

# name -> (config file, extra parameters, stamped?, steps)
SCENARIOS = {
    'main': ('scenario.yaml', [], True, MAIN),
    'main-unstamped': ('scenario.yaml', ['use_stamped:=false'], False, MAIN),
    'start-up': ('scenario.yaml', [], True, START),
    'odd': ('scenario_odd.yaml', [], True, ODD),
}

INPUT_TOPICS = {
    'nav': 'nav_vel', 'key': 'key_vel', 'joy': 'joy_vel', 'bogus': 'bogus_vel',
    'zero': 'zero_vel', 'low': 'low_vel', 'mid': 'mid_vel', 'sticky': 'sticky_vel'}
LOCK_TOPICS = {
    'watchdog': 'watchdog', 'pause': 'pause_navigation', 'e_stop': 'e_stop', 'top': 'top_lock'}

# A configuration that the node must refuse to start with: (description, params).
BAD_CONFIGS = [
    ('duplicate input priorities (D1)',
     ['topics.a.topic:=a', 'topics.a.timeout:=0.5', 'topics.a.priority:=10',
      'topics.b.topic:=b', 'topics.b.timeout:=0.5', 'topics.b.priority:=10']),
    ('priorities that collide after clamping (D1)',
     ['topics.a.topic:=a', 'topics.a.timeout:=0.5', 'topics.a.priority:=300',
      'topics.b.topic:=b', 'topics.b.timeout:=0.5', 'topics.b.priority:=255']),
    ('integer timeout (R14)',
     ['topics.a.topic:=a', 'topics.a.timeout:=1', 'topics.a.priority:=10']),
    ('missing priority (R14)',
     ['topics.a.topic:=a', 'topics.a.timeout:=0.5']),
    ('non-finite timeout (D5)',
     ['topics.a.topic:=a', 'topics.a.timeout:=.nan', 'topics.a.priority:=10']),
]


def resolve(node):
    if os.path.sep in node and os.path.isfile(node):
        return node
    package, executable = node.split('/')
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', package], text=True).strip()
    return os.path.join(prefix, 'lib', package, executable)


class Driver(Node):

    def __init__(self, stamped):
        super().__init__('scenario_driver')
        self.msg_type = TwistStamped if stamped else Twist
        self.stamped = stamped
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.inputs = {
            n: self.create_publisher(self.msg_type, t, 10) for n, t in INPUT_TOPICS.items()}
        self.locks = {n: self.create_publisher(Bool, t, 10) for n, t in LOCK_TOPICS.items()}
        self.sent = {}
        self.received = {}  # id -> list of messages
        self.create_subscription(self.msg_type, 'cmd_vel_out', self.on_out, 10)
        self.next_id = 1
        self.results = []  # (label, expected, forwarded)
        self.problems = []

    def twist(self, msg):
        return msg.twist if self.stamped else msg

    def on_out(self, msg):
        self.received.setdefault(int(self.twist(msg).linear.x), []).append(msg)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_connected(self, used_inputs, used_locks, timeout=20.0):
        pubs = [self.inputs[n] for n in used_inputs if n != 'bogus']
        pubs += [self.locks[n] for n in used_locks] + [self.clock_pub]
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if (all(p.get_subscription_count() > 0 for p in pubs)
                    and self.count_publishers('cmd_vel_out') > 0):
                self.spin_for(0.5)
                return
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('node under test did not come up')

    def run(self, steps):
        for step in steps:
            if step[0] == 'clock':
                msg = Clock()
                msg.clock.sec, msg.clock.nanosec = divmod(step[1], NS)
                self.clock_pub.publish(msg)
                self.spin_for(SETTLE)
            elif step[0] == 'lock':
                self.locks[step[1]].publish(Bool(data=step[2]))
                self.spin_for(SETTLE)
            else:
                _, name, expected, label = step
                msg = self.msg_type()
                twist = self.twist(msg)
                # Fill every field, so that an altered message is noticed.
                twist.linear.x = float(self.next_id)
                twist.linear.y, twist.linear.z = 0.25 * self.next_id, -1.5
                twist.angular.x, twist.angular.y, twist.angular.z = 0.5, -0.125, 2.0 * self.next_id
                if self.stamped:
                    msg.header.frame_id = f'frame_{self.next_id}'
                    msg.header.stamp.sec, msg.header.stamp.nanosec = 7, self.next_id
                self.sent[self.next_id] = msg
                self.inputs[name].publish(msg)
                self.spin_for(SETTLE)
                self.results.append((label, expected, self.next_id in self.received))
                self.next_id += 1
        self.spin_for(SETTLE)
        # Whatever came out must be exactly what was decided, once, unchanged.
        forwarded = {i + 1 for i, r in enumerate(self.results) if r[2]}
        if set(self.received) != forwarded:
            self.problems.append(f'late or unknown messages: {set(self.received) ^ forwarded}')
        for ident, msgs in self.received.items():
            if len(msgs) != 1:
                self.problems.append(f'message {ident} came out {len(msgs)} times')
            if ident in self.sent and msgs[0] != self.sent[ident]:
                self.problems.append(f'message {ident} was altered')


def run_scenario(exe, name):
    """Returns (decisions string, number of failures)."""
    config, extra, stamped, steps = SCENARIOS[name]
    args = [exe, '--ros-args', '--params-file', os.path.join(HERE, config),
            '-p', 'use_sim_time:=true']
    for p in extra:
        args += ['-p', p]
    proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    d = Driver(stamped)
    try:
        d.wait_connected({st[1] for st in steps if st[0] == 'cmd'},
                         {st[1] for st in steps if st[0] == 'lock'})
        d.run(steps)
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        d.destroy_node()

    bad = 0
    print(f'--- {name} ({os.path.basename(os.path.dirname(exe))})')
    for label, expected, forwarded in d.results:
        ok = expected == forwarded
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  {'forwarded' if forwarded else 'dropped  '}  {label}")
    for problem in d.problems:
        bad += 1
        print(f'FAIL  {problem}')
    return ''.join('1' if f else '0' for _, _, f in d.results), bad


def run_bad_configs(exe):
    bad = 0
    print('--- start-up errors')
    for label, params in BAD_CONFIGS:
        args = [exe, '--ros-args']
        for p in params:
            args += ['-p', p]
        proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            code = proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            code = None
        ok = code == 1
        bad += not ok
        print(f"{'ok  ' if ok else 'FAIL'}  refuses to start: {label} (exit code {code})")
    return bad


def main():
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    exe = resolve(sys.argv[1])
    other = resolve(sys.argv[2]) if len(sys.argv) == 3 else None
    rclpy.init()
    bad = 0
    try:
        for name in SCENARIOS:
            decisions, failures = run_scenario(exe, name)
            bad += failures
            print(f'DECISIONS {name} {decisions}')
            if other:
                other_decisions, other_failures = run_scenario(other, name)
                same = decisions == other_decisions
                bad += (not same) + other_failures
                print(f'DECISIONS {name} {other_decisions}')
                print(f"{'ok  ' if same else 'FAIL'}  {name}: both nodes "
                      f"{'agree' if same else 'DISAGREE'} on all {len(decisions)} decisions")
    finally:
        rclpy.shutdown()
    if 'roslean' in exe:
        bad += run_bad_configs(exe)
    print(f'{bad} failure(s)')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
