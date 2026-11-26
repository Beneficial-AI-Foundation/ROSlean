#!/usr/bin/env python3
"""Keeps Nav2 busy: sends it to one place in the apartment after another.

When a goal fails, which is what happens when teleop or the e-stop holds the
robot for more than a few seconds and Nav2 gives up, the same goal is sent
again after a short pause. A goal set by hand in RViz takes over until it
ends; the tour then continues.
"""

import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# (name, x, y) in map coordinates.
STOPS = [('desk', -2.3, -2.3), ('kitchen', 0.2, 2.4), ('sofa', -2.6, 0.6),
         ('bedroom door', 2.0, -0.3), ('bookshelf', -3.4, -0.6)]


def main():
    rclpy.init()
    node = Node('demo_nav_tour')
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    log = node.get_logger()
    index = 0
    try:
        while not client.wait_for_server(timeout_sec=2.0):
            log.info('waiting for Nav2...', throttle_duration_sec=10.0)
        time.sleep(5.0)  # let localisation settle
        while rclpy.ok():
            name, x, y = STOPS[index]
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
            goal.pose.pose.orientation.w = 1.0
            log.info(f'next stop: {name} ({x}, {y})')
            sent = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, sent)
            handle = sent.result()
            if handle is None or not handle.accepted:
                time.sleep(3.0)
                continue
            result = handle.get_result_async()
            rclpy.spin_until_future_complete(node, result)
            if result.result().status == GoalStatus.STATUS_SUCCEEDED:
                log.info(f'reached {name}')
                index = (index + 1) % len(STOPS)
            else:
                log.info(f'did not reach {name}; trying again')
            time.sleep(3.0)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
