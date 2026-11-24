#!/usr/bin/env python3
"""Automated check of the collision monitor demo.

Starts the demo without windows, runs the obstacle scenario (obstacle.py),
records the monitor's input and output and the robot's true pose, and
asserts:

  1. before the box, the monitor passed the command unchanged and the robot
     drove at the planner's speed;
  2. with the box in the slowdown zone, the monitor published the command
     times the configured ratio, and the robot's true speed dropped;
  3. the monitor then published zero for a non-zero command, and the robot
     came to rest without touching the box and stayed at rest while the box
     was there;
  4. after the box was removed the command passed again and the robot drove
     on;
  5. at no time was a published velocity larger than the command.

It prints the measured speeds, the stopping distance and the gap to the box.
"Speed", "rest" and "gap" are from the simulator's ground truth; the box is
static, so its pose is the one it was created with.

    check_demo.py [--monitor nav2_collision_monitor] [--mux twist_mux] [--log FILE]
                  [--no-furniture] [--nav2]

With --nav2 it checks the variant in which Nav2's own bring-up starts the
monitor (cm_nav2_launch.py with driver:=patrol): the same scenario on Nav2's
topics and with the zones of nav2_monitor.yaml, and in addition that Nav2's
lifecycle manager activated the monitor, that exactly one monitor runs and
from which package, and what the monitor's state topic said.

Not a colcon test: Gazebo renders the lidar, so the check needs a machine
with a display (docs/TUTORIAL_GAZEBO.md), and the world needs the furniture
from Gazebo Fuel (--no-furniture leaves it out, for machines that cannot
reach Gazebo Fuel; the scenario uses the walls, the floor and the robot
only). It takes about a minute. Unless ROS_DOMAIN_ID and
GZ_PARTITION are set, it uses a domain and a partition of its own, so that a
demo running on the same machine is not disturbed.
"""

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '94')
os.environ.setdefault('GZ_PARTITION', f'roslean_cm_check_{os.getpid()}')

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obstacle  # noqa: E402

PLANNER_SPEED = 0.18   # m/s, SPEED in patrol.py
RATIO = 0.4            # slowdown_ratio in monitor.yaml and in nav2_monitor.yaml

# The two variants: the launch file, the monitor's input and output topics,
# and the front edge of the stop zone in metres ahead of the robot's origin.
VARIANTS = {
    'mux': dict(launch='cm_gazebo_launch.py', cmd_in='cmd_vel_mux', cmd_out='cmd_vel_monitored',
                stop_edge=0.30, node='cm_gate'),                # monitor.yaml
    'nav2': dict(launch='cm_nav2_launch.py', cmd_in='cmd_vel_smoothed', cmd_out='cmd_vel',
                 stop_edge=0.305, node='collision_monitor'),    # nav2_monitor.yaml
}
STATE_ACTIONS = {0: 'do nothing', 1: 'stop', 2: 'slowdown', 3: 'approach', 4: 'limit'}


def planar(twist):
    return (twist.linear.x, twist.linear.y, twist.angular.z)


class Recorder(obstacle.Observer):

    def __init__(self, variant=VARIANTS['mux']):
        super().__init__('demo_check')
        self.track = []      # (t, x, y, yaw), every pose
        self.commands = {}   # header stamp -> velocities, monitor input
        self.states = []     # (t, action, zone): the monitor's state topic
        self.bringup = {}    # what was seen of the running system (--nav2)
        self.outputs = []    # (t, command, published), monitor output paired with its input
        self.raw_outputs = []
        self.unpaired = 0
        self.events = {}     # name -> time
        self.box = None
        self.rest_gap = None
        self.still = None
        self.create_subscription(TwistStamped, variant['cmd_in'], self.on_in, 100)
        self.create_subscription(TwistStamped, variant['cmd_out'], self.on_out, 100)
        self.create_subscription(
            CollisionMonitorState, 'collision_monitor_state',
            lambda m: self.states.append((self.now(), m.action_type, m.polygon_name)), 100)

    def on_pose(self, t):
        self.track.append(self.poses[-1])

    def on_in(self, msg):
        # Nav2's velocity smoother sends several messages with the stamp of
        # the command it is smoothing, so a stamp can have several velocities.
        self.commands.setdefault(
            (msg.header.stamp.sec, msg.header.stamp.nanosec), []).append(planar(msg.twist))

    def on_out(self, msg):
        self.raw_outputs.append(
            (self.now(), (msg.header.stamp.sec, msg.header.stamp.nanosec), planar(msg.twist)))

    def pair(self):
        """Pairs each published message with the command of the same stamp.
        Done at the end, since a command may reach this node after its answer.
        If several commands carry that stamp, the one the published velocity
        equals is taken, else one it is RATIO times, else the newest."""
        for t, stamp, published in self.raw_outputs:
            commands = self.commands.get(stamp)
            if commands is None:
                self.unpaired += 1
                continue
            command = commands[-1]
            for candidate in commands:
                if candidate == published:
                    command = candidate
                    break
            else:
                for candidate in commands:
                    if scaled(candidate, published, RATIO):
                        command = candidate
                        break
            self.outputs.append((t, command, published))

    def lifecycle_state(self, node_name):
        client = self.create_client(GetState, f'/{node_name}/get_state')
        if not client.wait_for_service(timeout_sec=3.0):
            return 'no lifecycle service'
        future = client.call_async(GetState.Request())
        end = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result().current_state.label if future.done() else 'no answer'

    def look_at_bringup(self, demo_pid, node_name):
        """What runs, seen from outside while the demo is still up."""
        names = sorted(name for name, _ in self.get_node_names_and_namespaces())
        listing = subprocess.run(['ps', '-axo', 'pgid=,command='],
                                 capture_output=True, text=True).stdout
        executables = []
        for line in listing.splitlines():
            fields = line.split()
            if (len(fields) >= 2 and fields[0] == str(demo_pid)
                    and fields[1].endswith('/collision_monitor')):
                executables.append(fields[1])
        self.bringup = dict(
            state=self.lifecycle_state(node_name),
            monitors=[n for n in names if 'collision_monitor' in n or n == 'cm_gate'],
            lifecycle_managers=[n for n in names if n.startswith('lifecycle_manager')],
            nodes=names,
            executables=executables,
            publishers=sorted({i.node_name for i in
                               self.get_publishers_info_by_topic('cmd_vel')}),
            subscribers=sorted({i.node_name for i in
                                self.get_subscriptions_info_by_topic('cmd_vel_smoothed')}))

    def on_event(self, name, **details):
        self.events[name] = self.now()
        if name == 'missed':
            # The box was taken away again; forget this attempt.
            self.events.pop('dropped', None)
        self.box = details.get('box', self.box)
        self.rest_gap = details.get('gap', self.rest_gap)
        self.still = details.get('still', self.still)

    # ---- measurements on the recorded track ----

    def between(self, t0, t1):
        return [p for p in self.track if t0 <= p[0] <= t1]

    def mean_speed(self, t0, t1):
        return path_length(self.between(t0, t1)) / (t1 - t0) if t1 > t0 else 0.0

    def max_speed(self, t0, t1, window=0.5):
        best, t = 0.0, t0 + window
        while t <= t1:
            best = max(best, self.mean_speed(t - window, t))
            t += 0.1
        return best


def path_length(poses):
    return sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(poses, poses[1:]))


def is_zero(v):
    return not any(v)


def scaled(command, published, ratio):
    return all(abs(p - ratio * c) <= 1e-12 for c, p in zip(command, published))


def analyse_bringup(rec, package, log_text, check):
    """--nav2: who started the monitor, and is it the only one."""
    b = rec.bringup
    print('\n0. Nav2\'s bring-up')
    if not b:
        check(False, 'the running system was not looked at (the scenario ended early)')
        return
    check(b['state'] == 'active', f"the node collision_monitor is in the lifecycle state "
          f"'{b['state']}'")
    activated = [line for line in log_text.splitlines()
                 if 'lifecycle_manager_navigation' in line and 'Managed nodes are active' in line]
    check(bool(activated), "Nav2's lifecycle_manager_navigation reported 'Managed nodes are "
          "active'")
    check(b['monitors'] == ['collision_monitor'],
          f"one collision monitor node runs: {b['monitors']}")
    check(len(b['executables']) == 1 and f'/{package}/' in b['executables'][0],
          f"its executable: {b['executables']}")
    print(f"        publishers of cmd_vel: {b['publishers']}")
    print(f"        subscribers of cmd_vel_smoothed: {b['subscribers']}")
    print(f"        lifecycle managers: {b['lifecycle_managers']}")
    print(f"        {len(b['nodes'])} nodes: {', '.join(b['nodes'])}")
    said = [(STATE_ACTIONS.get(a, str(a)), zone) for _, a, zone in rec.states]
    check(('slowdown', 'FootprintSlow') in said and ('stop', 'FootprintStop') in said,
          'the state topic named the slowdown zone and the stop zone')
    print(f"        state topic, in order: {said}")


def analyse(rec, variant=VARIANTS['mux'], package=None, log_text=''):
    """Prints the measurements; returns the list of failed assertions."""
    failed = []

    def check(ok, text):
        print(f"  {'ok  ' if ok else 'FAIL'}  {text}")
        if not ok:
            failed.append(text)

    if variant is VARIANTS['nav2']:
        analyse_bringup(rec, package, log_text, check)

    ev = rec.events
    for name in ('dropped', 'rest', 'removed', 'moving'):
        if name not in ev:
            check(False, f"the scenario did not get as far as '{name}'")
            return failed
    t_drop, t_rest, t_removed, t_moving = ev['dropped'], ev['rest'], ev['removed'], ev['moving']
    t_end = rec.track[-1][0]

    def outputs(t0, t1):
        return [(t, c, p) for t, c, p in rec.outputs if t0 <= t <= t1 and not is_zero(c)]

    print('\n1. Free driving (the two seconds before the box)')
    before = outputs(t_drop - 2.0, t_drop)
    free_speed = rec.mean_speed(t_drop - 2.0, t_drop)
    check(len(before) >= 10 and all(c == p for _, c, p in before),
          f'the monitor passed all {len(before)} commands unchanged')
    check(free_speed > 0.85 * PLANNER_SPEED,
          f'true speed {free_speed:.3f} m/s (planner commands {PLANNER_SPEED} m/s)')

    print('\n2. Slowdown zone')
    approach = outputs(t_drop, t_rest)
    slow = [(t, c, p) for t, c, p in approach if scaled(c, p, RATIO) and c != p]
    stops = [(t, c, p) for t, c, p in approach if is_zero(p)]
    check(len(slow) >= 5, f'{len(slow)} commands were published times {RATIO}')
    check(bool(stops), f'{len(stops)} non-zero commands were answered with zero')
    if not slow or not stops:
        return failed
    t_slow, t_stop = slow[0][0], stops[0][0]
    check(t_slow < t_stop, 'the slowdown came before the stop')
    # Leave the robot half a second to decelerate before measuring.
    slow_speed = rec.mean_speed(min(t_slow + 0.5, t_stop), t_stop)
    check(0.01 < slow_speed < 0.6 * free_speed,
          f'true speed in the slowdown zone {slow_speed:.3f} m/s '
          f'({slow_speed / free_speed:.0%} of the free speed; {t_stop - t_slow:.1f} s)')
    pose_slow = rec.between(t_slow, t_end)[0]
    print(f'        slowdown began with the box {obstacle.gap(pose_slow[1:], rec.box):.3f} m '
          'from the robot')

    print('\n3. Stop zone')
    after_stop = rec.between(t_stop, t_removed)
    pose_stop, pose_rest = after_stop[0], after_stop[-1]
    gap_stop = obstacle.gap(pose_stop[1:], rec.box)
    gap_rest = obstacle.gap(pose_rest[1:], rec.box)
    gap_min = min(obstacle.gap(p[1:], rec.box) for p in rec.between(t_drop, t_removed)[::5])
    stopping = path_length(after_stop)
    held = outputs(t_stop, t_removed)
    check(all(is_zero(p) or scaled(c, p, RATIO) for _, c, p in held),
          'from the first zero until the box was removed, no command passed unchanged')
    relapses = sum(1 for a, b in zip(held, held[1:]) if is_zero(a[2]) and not is_zero(b[2]))
    print(f'        the decision went back from stop to slowdown {relapses} time(s) '
          '(lidar noise at the edge of the zone)')
    crept = path_length(rec.between(t_rest, t_removed))
    check(crept < 0.03 and rec.still,
          f'the robot was at rest when the box was removed, {t_removed - t_rest:.1f} s after '
          f'it had come to rest, and moved {crept * 1000:.0f} mm in between')
    check(gap_min > 0.02, f'the robot never touched the box: smallest gap {gap_min:.3f} m')
    print(f'        first zero published with the box {gap_stop:.3f} m from the robot')
    print(f'        stopping distance {stopping:.3f} m, from the first zero to rest')
    print(f'        at rest {gap_rest:.3f} m from the box (the front edge of the stop zone is '
          f"{variant['stop_edge'] - obstacle.BODY[1]:.3f} m ahead of the robot)")
    zeros = [t for t, _, p in rec.outputs if t_stop <= t <= t_removed and is_zero(p)]
    print(f'        the monitor published {len(zeros)} zeros over {zeros[-1] - zeros[0]:.2f} s '
          f'and was then silent for {t_removed - zeros[-1]:.1f} s (stop_pub_timeout)')

    print('\n4. After the box was removed')
    later = outputs(t_removed, t_end)
    passed = [(t, c, p) for t, c, p in later if c == p]
    moved = path_length(rec.between(t_removed, t_end))
    check(len(passed) >= 10, f'{len(passed)} commands passed unchanged')
    if passed:
        print(f'        first command passed {passed[0][0] - t_removed:.2f} s after the removal')
    check(moved > 0.3, f'the robot drove on: {moved:.2f} m in {t_end - t_removed:.1f} s')
    check(rec.max_speed(t_moving, t_end) > 0.85 * PLANNER_SPEED,
          f'and reached {rec.max_speed(t_moving, t_end):.3f} m/s again')

    print('\n5. Whole run')
    everything = [(c, p) for _, c, p in rec.outputs]
    check(all(abs(p[i]) <= abs(c[i]) and p[i] * c[i] >= 0 for c, p in everything for i in range(3)),
          f'none of the {len(everything)} published velocities exceeded its command '
          'or had the other sign')
    kinds = sum(1 for c, p in everything if not (c == p or is_zero(p) or scaled(c, p, RATIO)))
    check(kinds == 0, 'every published velocity was the command, zero, or the command times '
          f'{RATIO}')
    return failed


def group_members(pgid):
    """The processes of a process group: everything the demo started."""
    listing = subprocess.run(['ps', '-axo', 'pid=,pgid='], capture_output=True, text=True).stdout
    rows = (line.split() for line in listing.splitlines())
    return [int(row[0]) for row in rows if len(row) == 2 and int(row[1]) == pgid]


def stop_demo(demo):
    """Ctrl-C for the launch, then, for whatever is left of its process group
    after 20 s, SIGKILL. Processes are signalled one by one: on macOS a signal
    to the whole group is refused as soon as one member is already a zombie."""
    try:
        demo.send_signal(signal.SIGINT)
    except ProcessLookupError:
        pass
    deadline = time.monotonic() + 20
    while time.monotonic() < deadline:
        demo.poll()
        if not group_members(demo.pid):
            return
        time.sleep(0.5)
    for pid in group_members(demo.pid):
        try:
            os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    demo.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--monitor', default='roslean_collision_monitor',
                        help='package of the monitor (nav2_collision_monitor for upstream)')
    parser.add_argument('--mux', default='roslean_twist_mux')
    parser.add_argument('--log', help='keep the output of the demo in this file')
    parser.add_argument('--no-furniture', action='store_true',
                        help='load the world without the models from Gazebo Fuel')
    parser.add_argument('--nav2', action='store_true',
                        help="the variant in which Nav2's bring-up starts the monitor")
    args = parser.parse_args()
    variant = VARIANTS['nav2' if args.nav2 else 'mux']

    launch_file = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                               'demo', 'gazebo', variant['launch'])
    log = open(args.log, 'w+') if args.log else tempfile.TemporaryFile(mode='w+')
    if args.nav2:
        print("variant: Nav2's bring-up starts the monitor; the patrol script drives "
              'on cmd_vel_nav')
    print(f'starting the demo without windows (monitor: {args.monitor}, mux: '
          f"{'none' if args.nav2 else args.mux}, "
          f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}, "
          f"GZ_PARTITION={os.environ['GZ_PARTITION']})")
    wall = time.monotonic()
    demo = subprocess.Popen(
        ['ros2', 'launch', launch_file, 'gui:=false', f'monitor_package:={args.monitor}']
        + (['driver:=patrol'] if args.nav2 else [f'mux_package:={args.mux}'])
        + (['furniture:=false'] if args.no_furniture else []),
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    rec = Recorder(variant)

    def on_event(name, **details):
        rec.on_event(name, **details)
        if name == 'rest' and args.nav2:
            rec.look_at_bringup(demo.pid, variant['node'])

    try:
        completed = obstacle.scenario(rec, on_event=on_event)
        if completed:
            # Watch the robot drive on for a few seconds.
            t = rec.now()
            rec.spin_until(lambda: rec.now() - t >= 4.0, 10.0)
    finally:
        stop_demo(demo)
        rec.destroy_node()
        rclpy.try_shutdown()

    rec.pair()
    log.flush()
    log.seek(0)
    failed = analyse(rec, variant, args.monitor, log.read())
    if rec.unpaired > 5:
        failed.append(f'{rec.unpaired} published messages carried a stamp of no recorded command')
        print(f'  FAIL  {failed[-1]}')
    print(f'\n{"PASSED" if not failed else "FAILED"} in {time.monotonic() - wall:.0f} s '
          f'({rec.track[-1][0] if rec.track else 0:.0f} s simulated)')
    if failed and not args.log:
        log.seek(0)
        print('\n--- last lines of the demo output ---\n' + ''.join(log.readlines()[-40:]))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
