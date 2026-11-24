#!/usr/bin/env python3
"""Measurements on the Nav2 variant of the demo (cm_nav2_launch.py) that
assert nothing: what the robot did, for the comparison of `stop` zones with
Nav2's `approach` zone in the tutorial (docs/TUTORIAL_COLLISION_MONITOR.md,
steps 7.5 and 7.6). Starts the demo without windows, like check_demo.py.

    nav2_observe.py box  [--monitor PKG] [--zones stop|approach] [--log FILE]
        The patrol drives at a box (obstacle.py's scenario). Prints, for each
        attempt, every half second: the gap between robot and box, the
        command, what the monitor published, and the true speed.

    nav2_observe.py tour [--monitor PKG] [--zones stop|approach] [--seconds N] [--log FILE]
        Nav2 drives the tour of nav_tour.py for N simulated seconds. Prints
        how far the robot got, what the tour reported, what the monitor's
        state topic said, and for how long the monitor held the robot.
"""

import argparse
import math
import os
import re
import subprocess
import sys
import tempfile
import time

os.environ.setdefault('ROS_DOMAIN_ID', '95')
os.environ.setdefault('GZ_PARTITION', f'roslean_cm_observe_{os.getpid()}')

import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import check_demo  # noqa: E402
import obstacle  # noqa: E402


class Recorder(check_demo.Recorder):

    def __init__(self):
        super().__init__(check_demo.VARIANTS['nav2'])
        self.inputs = []      # (t, velocity): every command to the monitor
        self.attempts = []    # [t_drop, box, t_end, outcome]
        self.last_scan = None

    def on_scan(self, msg):
        super().on_scan(msg)
        self.last_scan = msg

    def readings_in(self, x0, x1, y0, y1):
        """How many valid readings of the last scan lie in a rectangle of the
        base frame, computed here from the scan and the lidar's place."""
        count, angle = 0, self.last_scan.angle_min
        for r in self.last_scan.ranges:
            if self.last_scan.range_min <= r <= self.last_scan.range_max:
                x, y = obstacle.LIDAR_X + r * math.cos(angle), r * math.sin(angle)
                count += x0 < x < x1 and y0 < y < y1
            angle += self.last_scan.angle_increment
        return count

    def on_in(self, msg):
        super().on_in(msg)
        self.inputs.append((self.now(), check_demo.planar(msg.twist)))

    def on_event(self, name, **details):
        super().on_event(name, **details)
        if name == 'dropped':
            self.attempts.append([self.now(), details['box'], None, 'unfinished'])
        elif name in ('missed', 'rest') and self.attempts:
            self.attempts[-1][2:] = [self.now(), 'the robot turned away; box removed'
                                     if name == 'missed' else 'the robot came to rest']


def latest(series, t):
    """The last entry of a time-ordered list of (t, value) at or before t."""
    value = None
    for stamp, v in series:
        if stamp > t:
            break
        value = v
    return value


def box_profile(rec):
    outputs = [(t, p) for t, _, p in rec.raw_outputs]
    for number, (t_drop, box, t_end, outcome) in enumerate(rec.attempts, 1):
        t_end = t_end or rec.track[-1][0]
        print(f'\nattempt {number}: {outcome} after {t_end - t_drop:.1f} s')
        print('    t (s)  gap (m)   command (m/s, rad/s)   published (m/s, rad/s)   true speed (m/s)')
        t, smallest = t_drop, None
        while t <= t_end + 1e-9:
            poses = rec.between(t - 0.5, t)
            if poses:
                gap = obstacle.gap(poses[-1][1:], box)
                smallest = gap if smallest is None else min(smallest, gap)
                cmd = latest(rec.inputs, t) or (0.0, 0.0, 0.0)
                out = latest(outputs, t) or (0.0, 0.0, 0.0)
                last_out = max((s for s, _ in outputs if s <= t), default=None)
                shown = ('   (nothing for 0.3 s)' if last_out is None or t - last_out > 0.3
                         else f'{out[0]:8.3f} {out[2]:7.3f}      ')
                print(f'   {t - t_drop:5.1f}   {gap:6.3f}    {cmd[0]:8.3f} {cmd[2]:7.3f}        '
                      f'{shown}      {rec.mean_speed(t - 0.5, t):6.3f}')
            t += 0.5
        if smallest is not None:
            print(f'    smallest gap seen: {smallest:.3f} m')


def tour_summary(rec, log_text, seconds):
    t0, t1 = rec.track[0][0], rec.track[-1][0]
    print(f'\nobserved {t1 - t0:.0f} s of simulated time')
    print(f'distance driven (ground truth): {check_demo.path_length(rec.track):.2f} m')
    for line in log_text.splitlines():
        m = re.search(r'\[demo_nav_tour\]: (.*)', line)
        if m:
            print(f'    tour: {m.group(1)}')
    for pattern in ('Failed to make progress', 'Goal succeeded', 'Goal failed', 'Aborting',
                    'Running spin', 'Running backup', 'spin failed', 'backup failed',
                    'Collision Ahead'):
        count = sum(1 for line in log_text.splitlines() if pattern in line)
        if count:
            print(f"    Nav2's log has '{pattern}' {count} time(s)")
    said = {}
    for _, action, zone in rec.states:
        key = (check_demo.STATE_ACTIONS.get(action, str(action)), zone)
        said[key] = said.get(key, 0) + 1
    print(f'state topic: {said}')
    # Half-second bins: what the monitor did with non-zero commands.
    kinds = {'passed': 0, 'scaled down': 0, 'zero or nothing': 0, 'no command': 0}
    held, longest, t = 0.0, 0.0, t0
    outputs = [(s, p) for s, _, p in rec.raw_outputs]
    while t < t1:
        cmds = [v for s, v in rec.inputs if t <= s < t + 0.5 and any(v)]
        outs = [v for s, v in outputs if t <= s < t + 0.5 and any(v)]
        if not cmds:
            kind = 'no command'
        elif not outs:
            kind = 'zero or nothing'
        elif max(abs(v[0]) + abs(v[2]) for v in outs) < 0.99 * max(
                abs(v[0]) + abs(v[2]) for v in cmds):
            kind = 'scaled down'
        else:
            kind = 'passed'
        kinds[kind] += 1
        held = held + 0.5 if kind == 'zero or nothing' else 0.0
        longest = max(longest, held)
        t += 0.5
    print('half seconds in which Nav2 sent non-zero commands and the monitor '
          + ', '.join(f'{k}: {v}' for k, v in kinds.items()))
    print(f'longest time the monitor answered non-zero commands with zero or nothing: '
          f'{longest:.1f} s' + (' (still so at the end)' if held == longest and held > 0 else ''))
    moved = check_demo.path_length(rec.between(t1 - 30.0, t1))
    print(f'in the last 30 s the robot moved {moved:.2f} m')
    x, y, yaw = rec.track[-1][1:]
    print(f'at the end the robot was at ({x:.2f}, {y:.2f}), heading {math.degrees(yaw):.0f} deg')
    if rec.last_scan is not None:
        print('readings of the last scan inside the rectangle of FootprintStop: '
              f'{rec.readings_in(-0.155, 0.305, -0.155, 0.155)}, of FootprintSlow: '
              f'{rec.readings_in(-0.205, 0.705, -0.205, 0.205)} (min_points is 6)')


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('what', choices=['box', 'tour'])
    parser.add_argument('--monitor', default='roslean_collision_monitor')
    parser.add_argument('--zones', default='stop', choices=['stop', 'approach'])
    parser.add_argument('--seconds', type=float, default=300.0)
    parser.add_argument('--log')
    args = parser.parse_args()

    launch_file = os.path.join(get_package_share_directory('roslean_collision_monitor'),
                               'demo', 'gazebo', 'cm_nav2_launch.py')
    log = open(args.log, 'w+') if args.log else tempfile.TemporaryFile(mode='w+')
    print(f'{args.what}: monitor {args.monitor}, zones {args.zones}, '
          f"ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']}")
    demo = subprocess.Popen(
        ['ros2', 'launch', launch_file, 'gui:=false', f'monitor_package:={args.monitor}',
         f'zones:={args.zones}', 'driver:=' + ('patrol' if args.what == 'box' else 'tour')],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    rec = Recorder()
    try:
        if args.what == 'box':
            obstacle.scenario(rec, on_event=rec.on_event)
        else:
            # Count from the moment the robot first moves (Nav2 is up then).
            rec.spin_until(lambda: rec.travelled > 0.05, 120.0)
            start = rec.now()
            wall = time.monotonic()
            while rec.now() - start < args.seconds and time.monotonic() - wall < 4 * args.seconds:
                rclpy.spin_once(rec, timeout_sec=0.05)
    finally:
        check_demo.stop_demo(demo)
        rec.destroy_node()
        rclpy.try_shutdown()
    log.flush()
    log.seek(0)
    if not rec.track:
        print('no ground truth was received: the demo did not come up')
        return 1
    if args.what == 'box':
        box_profile(rec)
    else:
        tour_summary(rec, log.read(), args.seconds)
    return 0


if __name__ == '__main__':
    sys.exit(main())
