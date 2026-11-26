#!/usr/bin/env python3
"""The bond and Nav2's lifecycle manager, on the wall clock.

    bond_manager.py [NODE] [--skip-manager]

NODE is a path to an executable or `<package>/<executable>`; the default is
upstream's `nav2_collision_monitor/collision_monitor`. The expectations are
what that binary was observed to do (Nav2 1.3.12; SPEC_NAV2.md N9, N10, row
9 of its test plan). Unlike scenario_nav2.py this test does not use
simulated time: the bond's heartbeats and the manager's timeouts run on the
wall clock, so the checks are bounds, not exact counts.

Part 1, the bond (`bond_heartbeat_period: 0.1`): no message on /bond while
inactive; heartbeats after activation, with the node's name as id; one
`active: false` message on deactivation.

Part 2, `nav2_lifecycle_manager` with `autostart` and `node_names: [NODE's
name]`: it configures and activates the node and connects the bond; when the
node is killed it reports the node down after `bond_timeout` (4 s).
"""

import argparse
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from bond.msg import Status
from lifecycle_msgs.msg import Transition

from cm_harness import HERE, UPSTREAM, Driver, Side, resolve

NAME = 'collision_monitor'
LINES = []


def note(ok, text):
    LINES.append(ok)
    print(f"{'ok  ' if ok else 'FAIL'}  {text}")
    sys.stdout.flush()


def start(exe, log):
    """The node with the bond on and on the wall clock (Side.start switches both off)."""
    args = [exe, '--ros-args', '--params-file', os.path.join(HERE, 'zones_nav2.yaml'),
            '-r', f'__node:={NAME}', '-p', 'polygons:=[StopCircle]',
            '-p', 'cmd_vel_out_topic:=cmd_vel_out', '-p', 'state_topic:=cm_state',
            '-p', 'bond_heartbeat_period:=0.1']
    return subprocess.Popen(args, stdout=log, stderr=log)


def stop(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()


def part_bond(exe, tmp):
    print('--- bond')
    side = Side(exe, NAME, 'cmd_vel_out', 'cm_state')
    d = Driver('bond_driver', [side], False)
    msgs = []
    d.create_subscription(Status, '/bond', msgs.append, 100)
    log = open(os.path.join(tmp, 'node_bond.log'), 'w')
    side.proc = start(exe, log)
    try:
        if not d.detect_lifecycle(side, 'auto'):
            note(False, 'the node offers no lifecycle services (or has exited)')
            return False
        note(d.change_state(side, Transition.TRANSITION_CONFIGURE), 'configure')
        d.spin_for(1.0)
        note(not msgs, f'{len(msgs)} message(s) on /bond while inactive (expected none)')
        note(d.change_state(side, Transition.TRANSITION_ACTIVATE), 'activate')
        d.spin_for(2.0)
        mine = [m for m in msgs if m.id == NAME]
        note(len(mine) >= 10 and len(mine) == len(msgs),
             f'{len(mine)} heartbeat(s) with id {NAME!r} in 2 s after activation, {len(msgs)} '
             'message(s) in all (expected at least 10 at a period of 0.1 s, all with that id)')
        note(bool(mine) and all(m.active and m.heartbeat_timeout == 4.0
                                and abs(m.heartbeat_period - 0.1) < 1e-6 for m in mine),
             'every heartbeat: active, heartbeat_timeout 4.0, heartbeat_period 0.1')
        info = d.endpoints_of(side, '/bond', True)
        qos = sorted({(i.qos_profile.reliability.name, i.qos_profile.durability.name)
                      for i in info})
        note(qos == [('RELIABLE', 'VOLATILE')], f'the /bond publisher of the node: {qos}')
        n = len(msgs)
        note(d.change_state(side, Transition.TRANSITION_DEACTIVATE), 'deactivate')
        d.spin_for(1.0)
        tail = msgs[n:]
        note(any(not m.active for m in tail),
             f'{sum(not m.active for m in tail)} message(s) with active: false on deactivation '
             '(expected at least one)')
        n = len(msgs)
        d.spin_for(1.0)
        note(len(msgs) == n, f'{len(msgs) - n} message(s) in the second after that (expected none)')
        return True
    finally:
        stop(side.proc)
        log.close()
        d.destroy_node()


def part_manager(exe, tmp):
    print('--- lifecycle manager')
    try:
        manager = resolve('nav2_lifecycle_manager/lifecycle_manager')
    except Exception:  # the package is not installed
        print('not run: nav2_lifecycle_manager is not installed')
        return
    side = Side(exe, NAME, 'cmd_vel_out', 'cm_state')
    d = Driver('manager_driver', [side], False)
    log = open(os.path.join(tmp, 'node_manager.log'), 'w')
    mlog_path = os.path.join(tmp, 'manager.log')
    mlog = open(mlog_path, 'w')
    side.proc = start(exe, log)
    mgr = subprocess.Popen(
        [manager, '--ros-args', '-r', '__node:=lifecycle_manager_cm_test',
         '-p', 'autostart:=true', '-p', f'node_names:=[{NAME}]', '-p', 'bond_timeout:=4.0',
         '-p', 'attempt_respawn_reconnection:=false'], stdout=mlog, stderr=mlog)

    def logged(text):
        mlog.flush()
        with open(mlog_path) as f:
            return text in f.read()

    try:
        # The node's lifecycle services are left alone while the manager
        # uses them: polling get_state made the manager's bring-up fail (T04).
        ok = d.spin_until(lambda: logged('Managed nodes are active'), 30.0)
        note(ok, 'the manager logs "Managed nodes are active"')
        for text in (f'Configuring {NAME}', f'Activating {NAME}',
                     f'Server {NAME} connected with bond'):
            note(logged(text), f'the manager logs "{text}"')
        d.spin_for(1.0)
        state = d.get_state(side)
        note(state == 'active', f'the node is {state}')
        side.proc.send_signal(signal.SIGKILL)
        side.proc.wait()
        t0 = time.monotonic()
        down = f'CRITICAL FAILURE: SERVER {NAME} IS DOWN'
        ok = d.spin_until(lambda: logged(down), 15.0)
        note(ok, f'{time.monotonic() - t0:.1f} s after the node was killed the manager logs '
             f'"{down}" (bond_timeout 4 s)')
    finally:
        stop(mgr)
        stop(side.proc)
        log.close()
        mlog.close()
        d.destroy_node()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('node', nargs='?', default=UPSTREAM)
    ap.add_argument('--skip-manager', action='store_true')
    opts = ap.parse_args()
    exe = resolve(opts.node)
    print(f'node under test: {exe}')
    tmp = tempfile.mkdtemp(prefix='roslean_cm_bond_')
    rclpy.init()
    try:
        # A node without lifecycle services cannot be managed either.
        if part_bond(exe, tmp) and not opts.skip_manager:
            part_manager(exe, tmp)
    finally:
        rclpy.shutdown()
    bad = sum(not ok for ok in LINES)
    print(f'{bad} failure(s) in {len(LINES)} checks; logs in {tmp}')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
