#!/usr/bin/env python3
"""Review CM5 (task T08): probes of the Lean node beside upstream's binary.

    review_probe.py CASE [--logs DIR]

Both nodes run on the same /clock, scans, commands and transforms (the
harness of ros/roslean_collision_monitor/test). The script prints what each
node published at every step; it judges nothing. Run it inside the pixi
environment with ROS_DOMAIN_ID set (the review used 190 to 199):

    ROS_DOMAIN_ID=192 pixi run bash -c 'source install/setup.bash && \
        python3 review/collision_monitor/nav2/review_probe.py wait-clock-reset'

Cases:
  wait-clock-reset   the simulated clock jumps back by more than 3 s while a
                     command waits for a transform (N26). tf2's waiting loop
                     gives up then ("bag loop"); the node's own loop does not.
  wait-reentrancy    a second command and a scan arrive while a command waits.
  wait-sigint        SIGINT while a command waits: how the processes end.
  stale-clock        a command that was queued while another waited is evaluated
                     with the clock reading from before the wait: is a scan
                     older than source_timeout still used?
  shift-wait-reset   the same clock reset in the ordinary situation with base
                     shift correction: a command that waits a few
                     milliseconds for the next odometry transform.
"""
import argparse
import os
import signal
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.join(HERE, '..', '..', '..', 'ros', 'roslean_collision_monitor', 'test')
sys.path.insert(0, os.path.abspath(TEST))

import rclpy  # noqa: E402
from cm_harness import Driver, Side, ray, resolve, s, twist6, xf  # noqa: E402

LEAN = 'roslean_collision_monitor/collision_monitor'
UP = 'nav2_collision_monitor/collision_monitor'
PARAMS = os.path.join(os.path.abspath(TEST), 'zones_nav2.yaml')
V = (0.5, 0.0, 0.25)


class Probe:
    def __init__(self, params, logs):
        self.sides = [Side(resolve(LEAN), 'cm_lean', 'out_lean', 'state_lean'),
                      Side(resolve(UP), 'cm_up', 'out_up', 'state_up')]
        self.d = Driver('review_probe', self.sides, False)
        for sd in self.sides:
            log = os.path.join(logs, sd.name + '.log') if logs else None
            sd.start(PARAMS, ['polygons:=[StopCircle]'] + params, log)
        for sd in self.sides:
            state = self.d.bring_up(sd)
            print(f'{sd.name}: {state}')
        self.d.spin_for(1.0)
        self.marks = [(0, 0) for _ in self.sides]

    def report(self, what):
        parts = []
        for i, sd in enumerate(self.sides):
            o, st = self.marks[i]
            outs = [tuple(round(x, 4) for x in twist6(m)[:1] + twist6(m)[5:]) for m in sd.out[o:]]
            states = [(m.polygon_name, m.action_type) for m in sd.states[st:]]
            alive = 'running' if sd.proc.poll() is None else f'EXITED {sd.proc.returncode}'
            parts.append(f'{sd.name}: vel {outs} state {states} [{alive}]')
            self.marks[i] = (len(sd.out), len(sd.states))
        print(f'{what}\n    ' + '\n    '.join(parts), flush=True)

    def clock(self, t, settle=0.3):
        self.d.set_clock(t, settle)

    def cmd(self, v=V, settle=0.3):
        self.d.send_cmd(self.d.make_cmd(v, 0), settle)

    def responsive(self):
        for sd in self.sides:
            t0 = time.monotonic()
            st = self.d.get_state(sd, timeout=3.0)
            print(f'    get_state {sd.name}: {st} after {time.monotonic() - t0:.2f} s', flush=True)

    def stop(self):
        for sd in self.sides:
            sd.stop()
        self.d.destroy_node()


def wait_clock_reset(logs):
    p = Probe([], logs)
    T = s(100)
    p.clock(T)
    p.d.send_scan(ray(0.25, frame='laser'), 0.3)   # no transform for 'laser'
    p.cmd()
    p.report('t=100 s, scan in a frame without transform, command: both wait (tolerance 0.5 s)')
    p.clock(s(1))
    p.report('the clock jumps back to 1 s during the wait')
    p.responsive()
    for k in range(2, 8):
        p.clock(s(k))
        p.cmd()
        p.report(f'clock {k} s, another command')
    p.responsive()
    p.clock(T + s(1))
    p.report('the clock is back at 101 s (past the tolerance of the first wait)')
    p.cmd()
    p.report('a command at 101 s')
    p.responsive()
    p.stop()


def wait_reentrancy(logs):
    p = Probe([], logs)
    T = s(100)
    p.clock(T)
    p.d.send_scan(ray(0.25, frame='laser'), 0.3)
    p.cmd((0.5, 0.0, 0.0))
    p.report('command A (0.5) waits for the transform of frame laser')
    p.cmd((0.3, 0.0, 0.0))
    p.cmd((0.2, 0.0, 0.0))
    p.d.send_scan(ray(2.0), 0.3)  # in the base frame, outside the circle
    p.report('commands B (0.3), C (0.2) and a scan in the base frame arrive during the wait')
    p.clock(T + s(0.5))
    p.report('the tolerance has passed')
    p.clock(T + s(0.5))
    p.report('same clock again (nothing new expected)')
    p.cmd((0.1, 0.0, 0.0))
    p.report('command D (0.1) with the base-frame scan stored')
    p.stop()


def wait_sigint(logs):
    p = Probe([], logs)
    p.clock(s(100))
    p.d.send_scan(ray(0.25, frame='laser'), 0.3)
    p.cmd()
    p.report('a command waits')
    for sd in p.sides:
        sd.proc.send_signal(signal.SIGINT)
    for sd in p.sides:
        try:
            code = sd.proc.wait(timeout=8)
        except Exception:
            code = 'still running after 8 s'
        print(f'    {sd.name}: SIGINT during the wait -> {code}')
    p.clock(s(101))
    for sd in p.sides:
        try:
            code = sd.proc.wait(timeout=8)
        except Exception:
            code = 'still running after 8 s'
            sd.proc.kill()
        print(f'    {sd.name}: after the clock passed the tolerance -> {code}')
    p.d.destroy_node()


def shift_wait_reset(logs):
    p = Probe(['base_shift_correction:=true'], logs)
    T = s(100)
    p.clock(T)
    p.d.publish_dynamic([xf('odom', 'base_footprint', stamp=T - s(1)),
                         xf('odom', 'base_footprint', stamp=T)], 0.3)
    p.d.send_scan(ray(0.25), 0.3)
    p.cmd()
    p.report('t=100 s, odometry up to 100 s, scan in the stop circle: both stop')
    p.clock(T + s(0.05))
    p.cmd()
    p.report('t=100.05 s, odometry not yet there: both wait for it')
    p.clock(s(0.01))
    p.report('the simulation is reset: clock 0.01 s')
    for k in range(1, 6):
        t = s(k)
        p.clock(t)
        p.d.publish_dynamic([xf('odom', 'base_footprint', stamp=t - s(0.5)),
                             xf('odom', 'base_footprint', stamp=t)], 0.3)
        p.d.send_scan(ray(0.25), 0.3)
        p.cmd()
        p.report(f'clock {k} s, fresh odometry and scan, a command')
    p.responsive()
    p.stop()


def stale_clock(logs):
    p = Probe(['source_timeout:=0.3'], logs)
    T = s(100)
    p.clock(T)
    p.d.send_scan(ray(0.25, frame='laser'), 0.3)
    p.cmd((0.5, 0.0, 0.0))
    p.report('command A (0.5) waits for the transform of frame laser (tolerance 0.5 s)')
    p.d.send_scan(ray(2.0), 0.3)  # base frame, stamp 100 s, outside the circle
    p.cmd((0.3, 0.0, 0.0))
    p.report('during the wait: a scan in the base frame stamped 100 s, and command B (0.3)')
    p.clock(T + s(0.5), 0.6)
    p.report('clock 100.5 s: A is answered; B is evaluated at 100.5 s, the scan is 0.5 s old, '
             'source_timeout is 0.3 s, so B must be stopped')
    p.cmd((0.1, 0.0, 0.0))
    p.report('command C (0.1) at 100.5 s: the scan is too old')
    p.stop()


CASES = {'stale-clock': stale_clock, 'wait-clock-reset': wait_clock_reset, 'wait-reentrancy': wait_reentrancy,
         'wait-sigint': wait_sigint, 'shift-wait-reset': shift_wait_reset}

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('case', choices=sorted(CASES))
    ap.add_argument('--logs', default='')
    a = ap.parse_args()
    if a.logs:
        os.makedirs(a.logs, exist_ok=True)
    rclpy.init()
    try:
        CASES[a.case](a.logs)
    finally:
        rclpy.shutdown()
