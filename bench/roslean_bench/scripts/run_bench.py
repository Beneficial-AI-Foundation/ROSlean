#!/usr/bin/env python3
"""Runs the run-time comparison of the Lean-backed twist_mux with upstream.

  run_bench.py core  [--out DIR] [--reps N] [--quick]
  run_bench.py node  [--out DIR] [--reps N] [--quick]
  run_bench.py all   [--out DIR]

`core` runs bench_core (one process per measurement): the decision alone,
in process. `node` starts each node binary in turn, drives it with load_probe
over real topics and records latency, loss, CPU time, memory and start-up
time. Results go to DIR/core.json and DIR/node.json; a summary is printed.

Source the workspace first (pixi run bench does).
"""

import argparse
import json
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix

BENCH = Path(get_package_prefix('roslean_bench')) / 'lib' / 'roslean_bench'
MUX = {
    'lean': Path(get_package_prefix('roslean_twist_mux')) / 'lib' / 'roslean_twist_mux' / 'twist_mux',
    'upstream': Path(get_package_prefix('twist_mux')) / 'lib' / 'twist_mux' / 'twist_mux',
}


# ---------------------------------------------------------------- core

def core_events(impl_set, inputs, locks, budget_s):
    """Number of events that keeps the slowest implementation near budget_s."""
    if 'lean' in impl_set:
        est_ns = 350 + 2.2 * inputs * inputs + 400 * locks + 0.2 * locks * locks
    else:
        est_ns = 300 + 60 * (inputs + locks)
    return int(max(300, min(2_000_000, budget_s * 1e9 / est_ns)))


def core_matrix(quick):
    """(label, impls, options). Every lean run is checked against the port."""
    budget = 0.3 if quick else 1.5
    m = []

    def add(label, impls, clock='synthetic', what='decide', inputs=3, locks=1, **kw):
        opts = dict(clock=clock, what=what, inputs=inputs, locks=locks, **kw)
        opts.setdefault('events', core_events(impls, inputs, locks, budget))
        m.append((label, impls, opts))

    syn = ('lean', 'port')
    ros = ('lean', 'upstream')
    # 1. Number of inputs, every input live, each speaking in turn.
    for n in (1, 2, 3, 4, 8, 16, 32, 64, 128, 256, 512, 1024):
        add(f'inputs={n}', syn, inputs=n, timeouts='live', speaker='rr')
    # 2. Number of locks.
    for k in (0, 1, 4, 16, 64, 256):
        add(f'locks={k}', syn, inputs=3, locks=k, timeouts='live', speaker='rr', **{'lock-rate': 0.1})
    # 3. Which input speaks (the state update walks to its index).
    for sp in ('first', 'last', 'random'):
        add(f'n=64 speaker={sp}', syn, inputs=64, timeouts='live', speaker=sp)
    # 4. Priority layout.
    for pr in ('asc', 'desc', 'equal', 'random'):
        add(f'n=64 prio={pr}', syn, inputs=64, timeouts='live', prio=pr, speaker='rr')
    # 5. Timeout regime; `huge` makes stamp + timeout exceed 2^63.
    for to in ('none', 'live', 'expired', 'mixed', 'huge'):
        add(f'n=8 timeouts={to}', syn, inputs=8, locks=2, timeouts=to, speaker='random',
            **{'lock-rate': 0.1})
    # 6. Clock readings at and above 2^63 ns: Lean's Nat leaves the machine word.
    add('n=8 t0=2^63', syn, inputs=8, locks=2, timeouts='mixed', speaker='random',
        t0=9223372036854775808, **{'lock-rate': 0.1})
    # 7. Churn: locks flapping, inputs expiring, random priorities with ties.
    add('churn n=8 m=4', syn, inputs=8, locks=4, timeouts='mixed', prio='random',
        speaker='random', events=200_000 if quick else 2_000_000,
        **{'lock-rate': 0.2, 'lock-true': 0.5})
    add('churn n=200 m=50', syn, inputs=200, locks=50, timeouts='mixed', prio='random',
        speaker='random', events=2_000 if quick else 20_000,
        **{'lock-rate': 0.3, 'lock-true': 0.5})
    # 8. Soak: memory and tail over a long run.
    add('soak n=4 m=2', syn, inputs=4, locks=2, timeouts='mixed', prio='random',
        speaker='random', events=1_000_000 if quick else 20_000_000,
        **{'lock-rate': 0.05, 'mean-dt': 20_000_000})
    # 9. Far outside any real configuration.
    add('n=2048 m=64', syn, inputs=2048, locks=64, timeouts='mixed', prio='random',
        speaker='random', events=100 if quick else 400, **{'lock-rate': 0.1})
    # 10. As deployed: node clock, upstream's own code.
    for n in (1, 3, 8, 32, 128, 512):
        for what in ('decide', 'callback'):
            add(f'ros {what} n={n}', ros, clock='ros', what=what, inputs=n, timeouts='live',
                speaker='rr')
    add('ros decide n=8 no timeouts', ros, clock='ros', inputs=8, timeouts='none', speaker='rr')
    add('ros callback n=3 m=1 locks 10%', ros, clock='ros', what='callback', inputs=3, locks=1,
        timeouts='live', speaker='rr', **{'lock-rate': 0.1, 'lock-true': 0.0})
    return m


def run_core(args):
    rows = []
    for label, impls, opts in core_matrix(args.quick):
        for impl in impls:
            runs = []
            for rep in range(args.reps):
                cmd = [str(BENCH / 'bench_core'), '--impl', impl, '--label', label]
                for k, v in opts.items():
                    cmd += [f'--{k}', str(v)]
                p = subprocess.run(cmd, capture_output=True, text=True)
                line = p.stdout.strip().splitlines()[-1] if p.stdout.strip() else ''
                if not line.startswith('{'):
                    sys.exit(f'{label} {impl}: no result\n{p.stderr[-2000:]}')
                r = json.loads(line)
                if p.returncode != 0 or r['mismatches']:
                    sys.exit(f'{label} {impl}: {r["mismatches"]} decisions differ from the port')
                runs.append(r)
            row = dict(runs[0])
            for k in ('mean_ns', 'p50_ns', 'p90_ns', 'p99_ns', 'p999_ns'):
                row[k] = statistics.median(r[k] for r in runs)
            row['max_ns'] = max(r['max_ns'] for r in runs)
            row['reps'] = len(runs)
            rows.append(row)
            print(f'{label:34s} {impl:8s} mean {row["mean_ns"]:>12.1f} ns  p50 {row["p50_ns"]:>10.0f}'
                  f'  p99 {row["p99_ns"]:>10.0f}  p99.9 {row["p999_ns"]:>10.0f}'
                  f'  max {row["max_ns"]:>11d}  rss {row["rss_end_kb"]} kB', flush=True)
    (args.out / 'core.json').write_text(json.dumps(rows, indent=1))


# ---------------------------------------------------------------- node

NODE_SCENARIOS = [
    # name, inputs, locks, active, lowest, rate Hz, pattern, seconds
    ('typical: 3 inputs at 50 Hz', dict(inputs=3, locks=1, rate=50, pattern='stagger')),
    ('fast: 4 inputs at 1 kHz', dict(inputs=4, locks=1, rate=1000, pattern='stagger')),
    ('flood: 1 of 3 inputs at 5 kHz', dict(inputs=3, locks=1, active=1, lowest=1, rate=5000,
                                            pattern='stagger')),
    ('16 inputs at 100 Hz, bursts', dict(inputs=16, locks=2, rate=100, pattern='burst')),
    ('64 inputs at 50 Hz, bursts', dict(inputs=64, locks=2, rate=50, pattern='burst')),
    ('256 inputs at 10 Hz, bursts', dict(inputs=256, locks=2, rate=10, pattern='burst')),
    ('256 inputs at 50 Hz, bursts', dict(inputs=256, locks=2, rate=50, pattern='burst')),
    ('512 inputs at 2 Hz, bursts', dict(inputs=512, locks=2, rate=2, pattern='burst')),
]


def write_params(path, inputs, locks):
    lines = ['/**:', '  ros__parameters:', '    use_stamped: true', '    topics:']
    for i in range(inputs):
        prio = 1 + i * 254 // max(inputs - 1, 1)
        lines += [f'      in_{i:05d}:', f'        topic: in_{i:05d}', '        timeout: 0.5',
                  f'        priority: {prio}']
    if locks:
        lines.append('    locks:')
    for j in range(locks):
        lines += [f'      lock_{j:05d}:', f'        topic: lock_{j:05d}', '        timeout: 2.0',
                  '        priority: 255']
    Path(path).write_text('\n'.join(lines) + '\n')


def proc_cpu_s(pid):
    f = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    return (int(f[11]) + int(f[12])) / os.sysconf('SC_CLK_TCK'), int(f[17])  # utime+stime, threads


def proc_mem_kb(pid):
    out = {}
    for line in Path(f'/proc/{pid}/smaps_rollup').read_text().splitlines():
        k, _, v = line.partition(':')
        if k in ('Rss', 'Pss', 'Private_Dirty', 'Private_Clean'):
            out[k] = int(v.split()[0])
    return out


def run_node_once(impl, sc, duration, env):
    with tempfile.TemporaryDirectory() as tmp:
        params = os.path.join(tmp, 'params.yaml')
        write_params(params, sc['inputs'], sc.get('locks', 0))
        cmd = [str(BENCH / 'load_probe'), '--duration', str(duration)]
        for k, v in sc.items():
            cmd += [f'--{k}', str(v)]
        probe = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, env=env)
        assert probe.stdout.readline().strip() == 'PROBE_UP'
        spawn_ns = time.monotonic_ns()
        mux = subprocess.Popen(
            [str(MUX[impl]), '--ros-args', '--params-file', params, '--log-level', 'warn'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env)
        try:
            # The probe prints its result when the load is over; sample the mux
            # once it is ready and again just before that.
            time.sleep(0.2)
            cpu0, wall0 = None, None
            out = None
            while out is None:
                if mux.poll() is not None:
                    raise RuntimeError(f'{impl} mux exited with {mux.returncode}')
                if cpu0 is None:
                    cpu0, wall0 = proc_cpu_s(mux.pid)[0], time.monotonic()
                try:
                    out, _ = probe.communicate(timeout=0.25)
                except subprocess.TimeoutExpired:
                    cpu1, threads = proc_cpu_s(mux.pid)
                    wall1 = time.monotonic()
                    mem = proc_mem_kb(mux.pid)
            if probe.returncode != 0:
                raise RuntimeError('probe failed')
            r = json.loads(out.strip().splitlines()[-1])
            r.update(impl=impl, startup_ms=(r['ready_mono_ns'] - spawn_ns) * 1e-6,
                     cpu_s=cpu1 - cpu0, wall_s=wall1 - wall0, threads=threads,
                     rss_kb=mem.get('Rss'), pss_kb=mem.get('Pss'),
                     private_kb=mem.get('Private_Dirty', 0) + mem.get('Private_Clean', 0))
            r['cpu_pct'] = 100.0 * r['cpu_s'] / r['wall_s']
            return r
        finally:
            mux.send_signal(signal.SIGINT)
            try:
                mux.wait(timeout=10)
            except subprocess.TimeoutExpired:
                mux.kill()
            if probe.poll() is None:
                probe.kill()


def run_node(args):
    env = dict(os.environ, ROS_DOMAIN_ID=os.environ.get('ROS_DOMAIN_ID', '89'))
    duration = 3 if args.quick else 10
    rows = []
    # Start-up alone: many short runs of the typical configuration.
    for impl in ('upstream', 'lean'):
        st = [run_node_once(impl, dict(inputs=3, locks=1, rate=50, warmup=0.2), 0.3, env)['startup_ms']
              for _ in range(3 if args.quick else 10)]
        rows.append(dict(scenario='startup', impl=impl, startup_ms_all=st,
                         startup_ms=statistics.median(st)))
        print(f'startup {impl:8s} median {statistics.median(st):.0f} ms  min {min(st):.0f}'
              f'  max {max(st):.0f}', flush=True)
    for name, sc in NODE_SCENARIOS:
        for rep in range(args.reps):
            for impl in (('upstream', 'lean') if rep % 2 == 0 else ('lean', 'upstream')):
                r = run_node_once(impl, sc, duration, env)
                r.update(scenario=name, rep=rep, **{f'cfg_{k}': v for k, v in sc.items()})
                rows.append(r)
                print(f'{name:32s} {impl:8s} got {r["received_winner"]:>6d}/{r["sent_winner"]:<6d}'
                      f' lat p50 {r["lat_p50_ns"] / 1e3:>9.0f} us  p99 {r["lat_p99_ns"] / 1e3:>9.0f}'
                      f'  max {r["lat_max_ns"] / 1e3:>9.0f}  cpu {r["cpu_pct"]:5.1f}%'
                      f'  pss {r["pss_kb"] / 1024:5.1f} MB  start {r["startup_ms"]:.0f} ms',
                      flush=True)
    (args.out / 'node.json').write_text(json.dumps(rows, indent=1))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('what', choices=('core', 'node', 'all'))
    ap.add_argument('--out', type=Path, default=Path('bench/results'))
    ap.add_argument('--reps', type=int, default=3)
    ap.add_argument('--quick', action='store_true', help='short runs, to check that everything works')
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    if args.what in ('core', 'all'):
        run_core(args)
    if args.what in ('node', 'all'):
        run_node(args)


if __name__ == '__main__':
    main()
