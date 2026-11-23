#!/usr/bin/env python3
"""Regenerates the Nav2 map of the apartment (apartment_map.pgm / .yaml).

Run it while the Gazebo demo is running with the patrol planner:

    make_map.py OUTPUT_DIR [SECONDS]

It listens to the lidar and to the simulator's ground-truth pose while the
robot patrols, traces every lidar ray into a grid, and writes an occupancy
map. Two things are added from the known layout, because the lidar (17 cm
above the floor, 3.5 m range) cannot see them from the patrol route: the
walls, and the footprints of furniture that the robot would see only the
legs of but cannot drive under. Only needed again if apartment.sdf changes.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

RES = 0.05                    # m per cell
ORIGIN = (-5.0, -4.0)         # world coordinates of the lower-left corner
SIZE = (200, 160)             # cells: 10 m x 8 m
LIDAR_X = -0.032              # lidar position in the robot frame

# (centre x, centre y, size x, size y), as in apartment.sdf.
WALLS = [(0, 3.56, 9.24, 0.12), (0, -3.56, 9.24, 0.12), (-4.56, 0, 0.12, 7.0),
         (4.56, 0, 0.12, 7.0), (1.3, -2.4, 0.12, 2.2), (3.6, -1.0, 1.8, 0.12)]
LOW_FURNITURE = [(-2.6, 3.0, 2.1, 0.95),    # sofa
                 (-4.0, 1.5, 0.9, 0.9),     # armchair
                 (3.2, -2.4, 2.3, 1.8),     # bed
                 (2.2, 1.4, 2.5, 1.7)]      # dining table and chairs


def main():
    out = sys.argv[1]
    seconds = float(sys.argv[2]) if len(sys.argv) > 2 else 120.0
    rclpy.init()
    node = Node('make_map', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=True)])
    hits = [[0] * SIZE[0] for _ in range(SIZE[1])]
    passes = [[0] * SIZE[0] for _ in range(SIZE[1])]
    state = {'pose': None, 'prev_yaw': None, 'steady': False, 'used': 0}

    def on_truth(msg):
        if not msg.transforms:
            return
        p, q = msg.transforms[0].transform.translation, msg.transforms[0].transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        # Use scans only while the robot is not turning: the pose and the scan
        # are not synchronised, and a small heading error smears the walls.
        state['steady'] = state['prev_yaw'] is not None and abs(yaw - state['prev_yaw']) < 2e-4
        state['prev_yaw'] = yaw
        state['pose'] = (p.x, p.y, yaw)

    def cell(x, y):
        return int((x - ORIGIN[0]) / RES), int((y - ORIGIN[1]) / RES)

    def on_scan(msg):
        if state['pose'] is None or not state['steady']:
            return
        x, y, yaw = state['pose']
        sx, sy = x + LIDAR_X * math.cos(yaw), y + LIDAR_X * math.sin(yaw)
        state['used'] += 1
        for i, r in enumerate(msg.ranges):
            hit = msg.range_min < r < msg.range_max
            length = r if hit else (msg.range_max if math.isinf(r) else 0.0)
            if length <= 0.0:
                continue
            a = yaw + msg.angle_min + i * msg.angle_increment
            steps = int(length / (RES / 2))
            for k in range(steps):
                cx, cy = cell(sx + k * RES / 2 * math.cos(a), sy + k * RES / 2 * math.sin(a))
                if 0 <= cx < SIZE[0] and 0 <= cy < SIZE[1]:
                    passes[cy][cx] += 1
            if hit:
                cx, cy = cell(sx + r * math.cos(a), sy + r * math.sin(a))
                if 0 <= cx < SIZE[0] and 0 <= cy < SIZE[1]:
                    hits[cy][cx] += 1

    node.create_subscription(TFMessage, 'ground_truth', on_truth, 50)
    node.create_subscription(LaserScan, 'scan', on_scan, qos_profile_sensor_data)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    for (bx, by, sx, sy) in WALLS + LOW_FURNITURE:
        (x0, y0), (x1, y1) = cell(bx - sx / 2, by - sy / 2), cell(bx + sx / 2, by + sy / 2)
        for cy in range(max(y0, 0), min(y1, SIZE[1] - 1) + 1):
            for cx in range(max(x0, 0), min(x1, SIZE[0] - 1) + 1):
                hits[cy][cx] = 999

    rows = []
    counts = {'occupied': 0, 'free': 0, 'unknown': 0}
    for cy in reversed(range(SIZE[1])):          # image row 0 is the top (max y)
        row = bytearray()
        for cx in range(SIZE[0]):
            if hits[cy][cx] >= 3:
                row.append(0); counts['occupied'] += 1
            elif passes[cy][cx] >= 6:
                row.append(254); counts['free'] += 1
            else:
                row.append(205); counts['unknown'] += 1
        rows.append(bytes(row))
    with open(os.path.join(out, 'apartment_map.pgm'), 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % SIZE + b''.join(rows))
    with open(os.path.join(out, 'apartment_map.yaml'), 'w') as f:
        f.write('image: apartment_map.pgm\nmode: trinary\nresolution: %s\n'
                'origin: [%s, %s, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'
                % (RES, ORIGIN[0], ORIGIN[1]))
    print(f"used {state['used']} scans: {counts}")


if __name__ == '__main__':
    main()
