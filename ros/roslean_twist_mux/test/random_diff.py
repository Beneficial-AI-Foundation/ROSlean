#!/usr/bin/env python3
"""Randomised differential test of two twist_mux nodes.

For each seed, generates a random configuration and a random sequence of
events (clock updates including backward jumps, lock values, commands), runs
both nodes side by side on the very same messages and the same `/clock`, and
requires both to make the decisions predicted by an independent integer-ns
Python model of docs/SPEC.md.

    random_diff.py NODE OTHER [SEED_FROM SEED_TO]      (default seeds 0..19)

NODE and OTHER are executable paths or `<package>/<executable>`.

Coverage per case: 1-6 inputs and 0-4 locks; priorities including negative,
0, 255 and above; equal input priorities (ties go to the first handle in the
nodes' order, by parameter name); lock priorities equal to input priorities; zero, negative,
sub-nanosecond and decimal timeouts; handle names whose lexicographic order
differs from priority order; clock bases of 0, 100 s and Unix-scale; events
before any clock message; 25 % of cases unstamped.

Event times lie on a 50 ms grid and timeouts are odd multiples of 25 ms, so no
event falls within 25 ms of a timeout boundary. Exactly at a boundary the two
implementations can legitimately differ (docs/SPEC.md, D2); boundaries are
probed separately in scenario.py with exactly representable values.
"""

import os
import random
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

CFG_DIR = tempfile.mkdtemp(prefix='roslean_random_diff_')
NS = 1_000_000_000


def clamp(p):
    return max(0, min(255, p))


def tns(timeout):
    return int(timeout * 1e9)  # truncation, as duration_cast


class Model:
    def __init__(self, cfg, float_mode=False):
        self.inp = [(clamp(p), tns(t)) for (_, _, t, p) in cfg['inputs']]
        # Order in which the nodes scan the inputs: that of list_parameters,
        # which sorts full parameter names ("topics.<name>.<field>").
        self.order = sorted(range(len(self.inp)), key=lambda k: cfg['inputs'][k][0] + '.')
        self.lck = [(clamp(p), tns(t)) for (_, _, t, p) in cfg['locks']]
        self.istamp = [0] * len(self.inp)
        self.lstamp = [0] * len(self.lck)
        self.lval = [False] * len(self.lck)
        self.now = 0
        self.float_mode = float_mode

    def expired(self, timeout, stamp):
        if timeout <= 0:
            return False
        if self.float_mode:  # upstream arithmetic (non-negative times only)
            return (float(self.now) / 1e9 - float(stamp) / 1e9) > float(timeout) / 1e9
        return self.now - stamp > timeout

    def step(self, ev):
        if ev[0] == 'clock':
            self.now = max(ev[1], 0) if not self.float_mode else ev[1]
            return None
        if ev[0] == 'lock':
            self.lstamp[ev[1]] = self.now
            self.lval[ev[1]] = ev[2]
            return None
        i = ev[1]
        self.istamp[i] = self.now
        lp = 0
        for j, (p, t) in enumerate(self.lck):
            if self.expired(t, self.lstamp[j]) or self.lval[j]:
                lp = max(lp, p)
        best, sel = 0, None
        for k in self.order:
            p, t = self.inp[k]
            if not self.expired(t, self.istamp[k]) and p >= lp and p > best:
                best, sel = p, k
        return sel == i


def write_yaml(cfg, path):
    lines = ['/**:', '  ros__parameters:']
    for group in ('inputs', 'locks'):
        items = cfg[group]
        if not items:
            continue
        lines.append('    %s:' % ('topics' if group == 'inputs' else 'locks'))
        for (name, topic, timeout, prio) in items:
            lines += ['      %s:' % name, '        topic: %s' % topic,
                      '        timeout: %s' % repr(float(timeout)), '        priority: %d' % prio]
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


class Driver(Node):
    def __init__(self, tag, cfg):
        super().__init__('drv_' + tag)
        self.stamped = cfg.get('stamped', True)
        T = TwistStamped if self.stamped else Twist
        self.T = T
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        topics = sorted({t for (_, t, _, _) in cfg['inputs']})
        self.in_pubs = {t: self.create_publisher(T, t, 10) for t in topics}
        ltopics = sorted({t for (_, t, _, _) in cfg['locks']})
        self.lock_pubs = {t: self.create_publisher(Bool, t, 10) for t in ltopics}
        self.got = {'a': {}, 'b': {}}
        self.out = {}
        for side in 'ab':
            self.out[side] = self.create_subscription(
                T, f'{tag}_out_{side}', lambda m, s=side: self.on_out(s, m), 10)
        self.tag = tag

    def tw(self, m):
        return m.twist if self.stamped else m

    def on_out(self, side, m):
        self.got[side].setdefault(int(self.tw(m).linear.x), []).append(m)

    def spin_for(self, sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_ready(self, nclock=2, timeout=25.0, per_topic=None):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ok = self.clock_pub.get_subscription_count() == nclock
            for t, p in list(self.in_pubs.items()) + list(self.lock_pubs.items()):
                want = per_topic.get(t, 2) if per_topic else 2
                ok = ok and p.get_subscription_count() == want
            for side in 'ab':
                ok = ok and self.count_publishers(f'{self.tag}_out_{side}') == 1
            if ok:
                self.spin_for(0.4)
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return False


def run_case(tag, cfg, events, exes, settle=0.04, extra_params=(), expect_wait=1.0,
             per_topic=None, log_dir=None):
    """Returns dict with model/a/b decision strings and problems."""
    path = os.path.join(CFG_DIR, f'{tag}.yaml')
    write_yaml(cfg, path)
    stamped = cfg.get('stamped', True)
    procs = []
    for side, exe in zip('ab', exes):
        args = [exe, '--ros-args', '--params-file', path, '-p', 'use_sim_time:=true',
                '-p', f'use_stamped:={"true" if stamped else "false"}',
                '-r', f'__node:=mux_{side}', '-r', f'cmd_vel_out:={tag}_out_{side}']
        for p in extra_params:
            args += ['-p', p]
        if log_dir:
            err = open(os.path.join(log_dir, f'{tag}_{side}.log'), 'w')
        else:
            err = subprocess.DEVNULL
        procs.append(subprocess.Popen(args, stdout=err, stderr=err))
    d = Driver(tag, cfg)
    model = Model(cfg)
    fmodel = Model(cfg, float_mode=True)
    res = {'model': '', 'fmodel': '', 'a': '', 'b': '', 'problems': []}
    try:
        if not d.wait_ready(per_topic=per_topic):
            res['problems'].append('not ready (exit codes %s)' % [p.poll() for p in procs])
            return res
        ident = 0
        dec = {'a': [], 'b': []}
        mdec, fdec = [], []
        for ev in events:
            exp = model.step(ev)
            fexp = fmodel.step(ev)
            if ev[0] == 'clock':
                m = Clock()
                m.clock.sec, m.clock.nanosec = divmod(ev[1], NS)
                d.clock_pub.publish(m)
                d.spin_for(settle)
            elif ev[0] == 'lock':
                d.lock_pubs[cfg['locks'][ev[1]][1]].publish(Bool(data=ev[2]))
                d.spin_for(settle)
            else:
                ident += 1
                m = d.T()
                t = d.tw(m)
                t.linear.x = float(ident)
                t.angular.z = 0.5 * ident
                if stamped:
                    m.header.frame_id = f'f{ident}'
                    m.header.stamp.sec = 3
                d.in_pubs[cfg['inputs'][ev[1]][1]].publish(m)
                # wait: settle, or (if the model expects a forward) until both arrive
                end = time.monotonic() + (expect_wait if exp else settle)
                min_end = time.monotonic() + settle
                while time.monotonic() < end:
                    rclpy.spin_once(d, timeout_sec=0.005)
                    if (exp and ident in d.got['a'] and ident in d.got['b']
                            and time.monotonic() >= min_end):
                        break
                for side in 'ab':
                    dec[side].append(len(d.got[side].get(ident, [])))
                mdec.append(exp)
                fdec.append(fexp)
        d.spin_for(0.3)
        for side in 'ab':
            final = [len(d.got[side].get(i + 1, [])) for i in range(ident)]
            if final != dec[side]:
                res['problems'].append(f'late messages on {side}: {dec[side]} -> {final}')
            res[side] = ''.join(str(min(c, 9)) for c in dec[side])
        res['model'] = ''.join('1' if x else '0' for x in mdec)
        res['fmodel'] = ''.join('1' if x else '0' for x in fdec)
        for side, p in zip('ab', procs):
            if p.poll() is not None:
                res['problems'].append(f'node {side} died with {p.returncode}')
    finally:
        for p in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait()
        d.destroy_node()
    return res


def resolve(node):
    if os.path.sep in node and os.path.isfile(node):
        return node
    package, executable = node.split('/')
    prefix = subprocess.check_output(['ros2', 'pkg', 'prefix', package], text=True).strip()
    return os.path.join(prefix, 'lib', package, executable)


PRIOS = [-5, 0, 1, 2, 10, 49, 50, 51, 99, 100, 101, 254, 255, 300]
TIMEOUTS = [0.0, -1.0, 1e-10, 0.125, 0.125, 0.275, 0.275, 0.725, 1.325]
NAMES = ['zz', 'a', 'B', 'm_1', 'aa', 'Z', 'k9', 'navigation', 'b', 'x0', '_u', 'joy']
STEP = 50_000_000  # 0.05 s grid; timeouts are odd multiples of 0.025 s
BASES = [0, 0, 100, 100, 1_800_000_000, 2_147_000_000]


def gen(seed):
    r = random.Random(seed)
    tag = f'c{seed}'
    n_in = r.randint(1, 6)
    prios = [r.choice(PRIOS) for _ in range(n_in)]
    if n_in > 1 and r.random() < 0.5:  # force a tie in half of the cases
        a, b = r.sample(range(n_in), 2)
        prios[a] = prios[b]
    names = r.sample(NAMES, len(NAMES))
    inputs = [(names[i], f'{tag}_in_{i}', r.choice(TIMEOUTS), prios[i]) for i in range(n_in)]
    n_lk = r.randint(0, 4)
    lnames = r.sample(NAMES, len(NAMES))
    locks = [(lnames[j], f'{tag}_lock_{j}', r.choice(TIMEOUTS), r.choice(PRIOS))
             for j in range(n_lk)]
    # the yaml lists handles in generation order; the nodes list them
    # lexicographically -- indices below are generation order, mapped by topic.
    cfg = {'inputs': inputs, 'locks': locks, 'stamped': r.random() < 0.75}
    events = []
    base = r.choice(BASES) * NS
    now = base
    if base:
        if r.random() < 0.3:  # some events at clock 0, before any /clock message
            for _ in range(r.randint(1, 4)):
                events.append(('cmd', r.randrange(n_in)))
        events.append(('clock', now))
    for _ in range(r.randint(350, 550)):
        x = r.random()
        if x < 0.30:
            k = r.choice([1, 1, 2, 2, 3, 5, 6, 8, 14, 15, 27, 40])
            now += k * STEP
            events.append(('clock', now))
        elif x < 0.34 and now > 0:
            now = max(0, now - r.choice([1, 3, 10, 30, 2000]) * STEP)
            if r.random() < 0.15:
                now = 0
            events.append(('clock', now))
        elif x < 0.55 and n_lk:
            events.append(('lock', r.randrange(n_lk), r.random() < 0.4))
        else:
            events.append(('cmd', r.randrange(n_in)))
    return tag, cfg, events


def main():
    if len(sys.argv) not in (3, 5):
        sys.exit(__doc__)
    exes = [resolve(sys.argv[1]), resolve(sys.argv[2])]
    lo, hi = (int(sys.argv[3]), int(sys.argv[4])) if len(sys.argv) == 5 else (0, 20)
    rclpy.init()
    bad = 0
    for seed in range(lo, hi):
        tag, cfg, events = gen(seed)
        t0 = time.monotonic()
        res = run_case(tag, cfg, events, exes)
        ok = res['a'] == res['b'] == res['model'] and not res['problems'] and res['model']
        status = 'ok  '
        if not ok:  # retry slowly to separate harness timing from a real divergence
            res2 = run_case(tag + 'r', cfg, events, exes, settle=0.3)
            ok2 = res2['a'] == res2['b'] == res2['model'] and not res2['problems']
            status = 'ok   (on slow retry)' if ok2 else 'FAIL'
            bad += not ok2
            print(f'  first  {res}\n  retry  {res2}\n  cfg {cfg}\n  events {events}')
        print(f'{status}  seed={seed} inputs={len(cfg["inputs"])} locks={len(cfg["locks"])} '
              f'decisions={len(res["model"])} forwarded={res["model"].count("1")} '
              f'{time.monotonic() - t0:.1f}s', flush=True)
    rclpy.shutdown()
    print(f'{bad} divergence(s) in {hi - lo} random cases')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
