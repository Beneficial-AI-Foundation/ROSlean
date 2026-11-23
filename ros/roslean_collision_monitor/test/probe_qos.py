#!/usr/bin/env python3
"""Prints things about a collision-monitor node that have no fixed expected value.

    probe_qos.py [NODE]

1. The quality of service of the node's subscriptions and publishers.
2. How many of N commands published back to back are answered. Upstream's
   command subscription keeps one message, so most of a burst is dropped;
   the numbers vary from run to run. (This is why scenario.py and
   random_diff.py send one command at a time and wait for the answer.)

Not registered as a test: nothing here passes or fails.
"""

import os
import sys

import rclpy

from cm_harness import HERE, UPSTREAM, Driver, Side, fan, resolve, s


def main():
    exe = resolve(sys.argv[1] if len(sys.argv) > 1 else UPSTREAM)
    rclpy.init()
    side = Side(exe, 'collision_monitor', 'cmd_vel_out', 'cm_state')
    side.start(os.path.join(HERE, 'zones.yaml'), ['polygons:=[StopCircle]'])
    d = Driver('probe', [side], False)
    try:
        print('node is', d.bring_up(side))
        d.wait_connected(False)
        for topic in ['scan', 'cmd_vel_in', '/clock']:
            for i in d.get_subscriptions_info_by_topic(topic):
                q = i.qos_profile
                print(f'subscription {topic}: {i.topic_type}, {q.reliability.name}, '
                      f'{q.durability.name}')
        for topic in ['cmd_vel_out', 'cm_state']:
            for i in d.get_publishers_info_by_topic(topic):
                q = i.qos_profile
                print(f'publisher {topic}: {i.topic_type}, {q.reliability.name}, '
                      f'{q.durability.name}')
        d.set_clock(s(100), 0.2)
        d.send_scan(fan(5, 5.0), 0.2)
        for burst in (2, 5, 20, 100):
            answered = []
            for _ in range(5):
                n0 = len(side.out)
                for k in range(burst):
                    d.cmd_pub.publish(d.make_cmd((0.001 * (k + 1), 0.0, 0.0), k))
                d.spin_for(0.5)
                got = [m.linear.x for m in side.out[n0:]]
                assert got == sorted(got)
                answered.append((len(got), bool(got) and got[-1] == 0.001 * burst))
            print(f'burst of {burst:3}: answered {[a for a, _ in answered]}, '
                  f'last command answered {[int(b) for _, b in answered]}')
    finally:
        side.stop()
        d.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
