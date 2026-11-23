#!/usr/bin/env python3
"""The obstacle of the collision monitor demo: a box in the robot's path.

    obstacle.py drop     put the box AHEAD metres in front of the robot, now
    obstacle.py remove   take it away
    obstacle.py run      the whole scenario, without a person: wait until the
                         robot drives straight with free space ahead, drop the
                         box, wait until the robot has come to rest in front
                         of it, wait HOLD seconds, take the box away, and
                         watch the robot drive on

The box is created and removed through the simulator's own services
(`gz service`), so the collision monitor learns of it only through the lidar.
It is a static model: it cannot be pushed, and it does not appear on the
`ground_truth` topic, which the patrol script reads on the assumption that
the robot is the only moving model.

The robot's position is the simulator's ground truth. A real robot has no
such thing; it is used here to place the box and to measure, not to drive.
"""

import argparse
import math
import subprocess
import sys
import time
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

WORLD = 'apartment'
BOX = 'cm_box'
SIZE = 0.40          # m, edge of the box
AHEAD = 1.20         # m from the robot's origin to the near face of the box
HOLD = 6.0           # s (simulated) the robot is left standing before the box goes
CLEAR = 2.0          # m of free corridor required ahead before the box is dropped
CORRIDOR = 0.40      # m, half width of that corridor
REST = 0.005         # m/s: below this the robot counts as at rest
ATTEMPTS = 3

# The lidar's place on the robot (see monitor.yaml), for the corridor test.
LIDAR_X = -0.032

# The outline of the burger's collision shapes in its own frame, in metres:
# the body box (0.14 m, centred at x = -0.032) and the wheels (y = +-0.08,
# 0.018 m wide). From turtlebot3_gazebo/models/turtlebot3_burger/model.sdf.
BODY = (-0.102, 0.038, -0.089, 0.089)   # x_min, x_max, y_min, y_max


def gz_service(service, reqtype, request, node=None, timeout_ms=5000):
    """Calls a Gazebo service; returns True if it answered `data: true`.

    The call takes up to a second. With a `node`, the node keeps spinning
    meanwhile, so that it does not fall behind on the robot's pose.
    """
    process = subprocess.Popen(
        ['gz', 'service', '-s', service, '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
         '--timeout', str(timeout_ms), '--req', request],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    while node is not None and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.02)
    return 'data: true' in process.communicate()[0]


def box_pose_ahead(pose, ahead=AHEAD):
    """Pose (x, y, yaw) of a box whose near face is `ahead` metres in front of the robot."""
    x, y, yaw = pose
    d = ahead + SIZE / 2
    return (x + d * math.cos(yaw), y + d * math.sin(yaw), yaw)


def drop_box(box, node=None):
    x, y, yaw = box
    sdf = (f"<sdf version='1.9'><model name='{BOX}'><static>true</static>"
           f"<pose>{x:.4f} {y:.4f} {SIZE / 2} 0 0 {yaw:.4f}</pose><link name='link'>"
           f"<collision name='c'><geometry><box><size>{SIZE} {SIZE} {SIZE}</size></box>"
           "</geometry></collision><visual name='v'><geometry><box>"
           f"<size>{SIZE} {SIZE} {SIZE}</size></box></geometry><material>"
           "<ambient>0.9 0.35 0.1 1</ambient><diffuse>0.9 0.35 0.1 1</diffuse></material>"
           "</visual></link></model></sdf>")
    return gz_service(f'/world/{WORLD}/create', 'gz.msgs.EntityFactory',
                      f'sdf: "{sdf}" allow_renaming: false', node)


def box_exists():
    listing = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True).stdout
    return BOX in listing.split()


def remove_box(node=None):
    return gz_service(f'/world/{WORLD}/remove', 'gz.msgs.Entity',
                      f'name: "{BOX}" type: MODEL', node)


def _to_frame(frame, x, y):
    fx, fy, fyaw = frame
    dx, dy = x - fx, y - fy
    c, s = math.cos(fyaw), math.sin(fyaw)
    return (c * dx + s * dy, -s * dx + c * dy)


def _from_frame(frame, x, y):
    fx, fy, fyaw = frame
    c, s = math.cos(fyaw), math.sin(fyaw)
    return (fx + c * x - s * y, fy + s * x + c * y)


def _outline(rect, step=0.004):
    x0, x1, y0, y1 = rect
    nx, ny = int((x1 - x0) / step) + 1, int((y1 - y0) / step) + 1
    pts = []
    for i in range(nx + 1):
        x = x0 + (x1 - x0) * i / nx
        pts += [(x, y0), (x, y1)]
    for j in range(ny + 1):
        y = y0 + (y1 - y0) * j / ny
        pts += [(x0, y), (x1, y)]
    return pts


def _dist_to_rect(rect, x, y):
    """Distance from a point to a rectangle; 0 inside."""
    x0, x1, y0, y1 = rect
    return math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))


_BOX_RECT = (-SIZE / 2, SIZE / 2, -SIZE / 2, SIZE / 2)
_ROBOT_OUTLINE = _outline(BODY)
_BOX_OUTLINE = _outline(_BOX_RECT)


def gap(robot, box):
    """Distance in metres between the robot's outline and the box; 0 if they touch.

    Both are rectangles on the floor. The outline of each is sampled every
    4 mm and measured against the other, so the result is good to a few
    millimetres.
    """
    a = min(_dist_to_rect(_BOX_RECT, *_to_frame(box, *_from_frame(robot, px, py)))
            for px, py in _ROBOT_OUTLINE)
    b = min(_dist_to_rect(BODY, *_to_frame(robot, *_from_frame(box, px, py)))
            for px, py in _BOX_OUTLINE)
    return min(a, b)


class Observer(Node):
    """Follows the robot's true pose and the lidar."""

    def __init__(self, name='demo_obstacle'):
        super().__init__(name, parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.pose = None          # (x, y, yaw)
        self.poses = deque()      # (t, x, y, yaw) of the last two seconds
        self.travelled = 0.0
        self.last_turn = None     # time at which the robot last turned
        self.corridor_free = False
        self.create_subscription(TFMessage, 'ground_truth', self.on_truth, 50)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def now(self):
        """Simulated time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def on_truth(self, msg):
        # As in patrol.py: the first entry is the robot.
        if not msg.transforms:
            return
        p, q = msg.transforms[0].transform.translation, msg.transforms[0].transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = self.now()
        if self.poses and t <= self.poses[-1][0]:
            return
        if self.pose is not None:
            self.travelled += math.hypot(p.x - self.pose[0], p.y - self.pose[1])
        self.pose = (p.x, p.y, yaw)
        self.poses.append((t, p.x, p.y, yaw))
        while self.poses[0][0] < t - 2.0:
            self.poses.popleft()
        if abs(self.yaw_rate()) > 0.2:
            self.last_turn = t
        self.on_pose(t)

    def on_pose(self, t):
        """For subclasses."""

    def _back(self, seconds):
        t = self.poses[-1][0]
        for sample in self.poses:
            if sample[0] >= t - seconds:
                return sample
        return self.poses[0]

    def speed(self, window=0.5):
        """Ground speed in m/s over the last `window` seconds."""
        if len(self.poses) < 2:
            return 0.0
        t0, x0, y0, _ = self._back(window)
        t1, x1, y1, _ = self.poses[-1]
        return math.hypot(x1 - x0, y1 - y0) / (t1 - t0) if t1 > t0 else 0.0

    def yaw_rate(self, window=0.5):
        if len(self.poses) < 2:
            return 0.0
        t0, _, _, a0 = self._back(window)
        t1, _, _, a1 = self.poses[-1]
        d = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return d / (t1 - t0) if t1 > t0 else 0.0

    def on_scan(self, msg):
        # Is anything in the corridor ahead of the robot?
        free = True
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min <= r <= msg.range_max:
                x, y = LIDAR_X + r * math.cos(angle), r * math.sin(angle)
                if 0.0 < x < CLEAR + SIZE and abs(y) < CORRIDOR:
                    free = False
                    break
            angle += msg.angle_increment
        self.corridor_free = free

    def spin_until(self, condition, timeout):
        """Spins until `condition()` holds; False after `timeout` simulated
        seconds, or 3 x `timeout` of real time if the simulation is slow or gone."""
        start, wall = None, time.monotonic()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
            t = self.now()  # 0 until the simulator's clock has been heard
            if t > 0.0 and (start is None or t < start):
                start = t
            if ((start is not None and t - start > timeout)
                    or time.monotonic() - wall > 3 * timeout + 20):
                return False
        return False

    def good_moment(self):
        """Driving straight on a leg that has just begun, with free space ahead."""
        if self.pose is None or self.last_turn is None:
            return False
        since_turn = self.now() - self.last_turn
        return (self.travelled > 0.6 and 1.0 < since_turn < 4.0 and self.speed() > 0.12
                and self.corridor_free)


def say(text):
    print(f'\n[obstacle] {text}', flush=True)


def scenario(node, hold=HOLD, on_event=lambda name, **details: None):
    """Runs the scenario once; returns True if the robot stopped and drove on.

    `on_event` is told what happens ('dropped', 'rest', 'removed', 'moving',
    'missed'), for the automated check.
    """
    say('waiting for the robot to drive straight with free space ahead')
    for attempt in range(ATTEMPTS):
        if not node.spin_until(node.good_moment, 120.0):
            say('the robot never drove straight with free space ahead; giving up')
            return False
        box = box_pose_ahead(node.pose)
        if not drop_box(box, node):
            say('the simulator did not accept the box; is the demo running?')
            return False
        on_event('dropped', box=box)
        say(f'box dropped {AHEAD:.1f} m ahead of the robot, at ({box[0]:.2f}, {box[1]:.2f})')

        # The robot should now approach, slow down and stop. If it turns away
        # instead (a planner that avoids the box, or a waypoint reached before
        # it), take the box away and try again.
        outcome = []

        def decided():
            if (node.speed(1.0) < REST and abs(node.yaw_rate(1.0)) < 0.02
                    and gap(node.pose, box) < 0.6):
                outcome.append('rest')
            elif (_to_frame(node.pose, box[0], box[1])[0] < 0.0
                  or abs(node.yaw_rate()) > 0.2):
                outcome.append('away')
            return bool(outcome)

        node.spin_until(decided, 40.0)
        if outcome == ['rest']:
            break
        remove_box(node)
        on_event('missed')
        say('the robot did not come to rest in front of the box; box removed'
            + (', trying again' if attempt + 1 < ATTEMPTS else ''))
    else:
        return False

    distance = gap(node.pose, box)
    on_event('rest', gap=distance)
    say(f'the robot is at rest, {distance:.2f} m from the box; the box stays for {hold:.0f} s')
    t = node.now()
    node.spin_until(lambda: node.now() - t >= hold, hold + 5.0)
    still = node.speed(1.0) < REST
    remove_box(node)
    on_event('removed', still=still)
    say('box removed')
    if not node.spin_until(lambda: node.speed() > 0.12, 15.0):
        say('the robot did not drive on')
        return False
    on_event('moving')
    say('the robot drives on')
    return still


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('command', choices=['drop', 'remove', 'run'])
    parser.add_argument('--ahead', type=float, default=AHEAD,
                        help='drop: distance from the robot to the near face of the box (m)')
    parser.add_argument('--hold', type=float, default=HOLD,
                        help='run: seconds the box stays after the robot has stopped')
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    if args.command == 'remove':
        print('box removed' if remove_box() else 'no box to remove (or the demo is not running)')
        return 0

    rclpy.init()
    node = Observer()
    try:
        if args.command == 'drop':
            if not node.spin_until(lambda: node.pose is not None, 10.0):
                print('no ground truth from the simulator; is the demo running?')
                return 1
            if box_exists():
                remove_box(node)  # one box at a time
            box = box_pose_ahead(node.pose, args.ahead)
            ok = drop_box(box, node)
            print(f'box dropped at ({box[0]:.2f}, {box[1]:.2f}), {args.ahead:.2f} m ahead'
                  if ok else 'the simulator did not accept the box')
            return 0 if ok else 1
        return 0 if scenario(node, args.hold) else 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 1
    finally:
        node.destroy_node()


if __name__ == '__main__':
    sys.exit(main())
