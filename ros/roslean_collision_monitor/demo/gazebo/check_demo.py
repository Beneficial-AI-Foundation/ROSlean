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

Not a colcon test: Gazebo renders the lidar, so the check needs a machine
with a display (docs/TUTORIAL_GAZEBO.md), and the world needs the furniture
from Gazebo Fuel. It takes about a minute. Unless ROS_DOMAIN_ID and
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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import obstacle  # noqa: E402

PLANNER_SPEED = 0.18   # m/s, SPEED in patrol.py
RATIO = 0.4            # slowdown_ratio in monitor.yaml
STOP_EDGE = 0.30       # m ahead of the robot's origin: front edge of the stop zone, monitor.yaml


def planar(twist):
    return (twist.linear.x, twist.linear.y, twist.angular.z)


class Recorder(obstacle.Observer):

    def __init__(self):
        super().__init__('demo_check')
        self.track = []      # (t, x, y, yaw), every pose
        self.commands = {}   # header stamp -> velocity, monitor input
        self.outputs = []    # (t, command, published), monitor output paired with its input
        self.raw_outputs = []
        self.unpaired = 0
        self.events = {}     # name -> time
        self.box = None
        self.rest_gap = None
        self.still = None
        self.create_subscription(TwistStamped, 'cmd_vel_mux', self.on_in, 100)
        self.create_subscription(TwistStamped, 'cmd_vel_monitored', self.on_out, 100)

    def on_pose(self, t):
        self.track.append(self.poses[-1])

    def on_in(self, msg):
        self.commands[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = planar(msg.twist)

    def on_out(self, msg):
        self.raw_outputs.append(
            (self.now(), (msg.header.stamp.sec, msg.header.stamp.nanosec), planar(msg.twist)))

    def pair(self):
        """Pairs each published message with the command of the same stamp.
        Done at the end, since a command may reach this node after its answer."""
        for t, stamp, published in self.raw_outputs:
            command = self.commands.get(stamp)
            if command is None:
                self.unpaired += 1
            else:
                self.outputs.append((t, command, published))

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


def analyse(rec):
    """Prints the measurements; returns the list of failed assertions."""
    failed = []

    def check(ok, text):
        print(f"  {'ok  ' if ok else 'FAIL'}  {text}")
        if not ok:
            failed.append(text)

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
          f'{STOP_EDGE - obstacle.BODY[1]:.3f} m ahead of the robot)')
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
    args = parser.parse_args()

    launch_file = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                               'demo', 'gazebo', 'cm_gazebo_launch.py')
    log = open(args.log, 'w') if args.log else tempfile.TemporaryFile(mode='w+')
    print(f'starting the demo without windows (monitor: {args.monitor}, mux: {args.mux}, '
          f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}, "
          f"GZ_PARTITION={os.environ['GZ_PARTITION']})")
    wall = time.monotonic()
    demo = subprocess.Popen(
        ['ros2', 'launch', launch_file, 'gui:=false', f'monitor_package:={args.monitor}',
         f'mux_package:={args.mux}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    rec = Recorder()
    try:
        completed = obstacle.scenario(rec, on_event=rec.on_event)
        if completed:
            # Watch the robot drive on for a few seconds.
            t = rec.now()
            rec.spin_until(lambda: rec.now() - t >= 4.0, 10.0)
    finally:
        stop_demo(demo)
        rec.destroy_node()
        rclpy.try_shutdown()

    rec.pair()
    failed = analyse(rec)
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
