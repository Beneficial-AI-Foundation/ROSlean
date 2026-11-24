#!/usr/bin/env python3
"""Run-time comparison of the ROSlean collision monitor with nav2_collision_monitor.

  run_cm_bench.py core [--out DIR] [--quick]
  run_cm_bench.py node [--out DIR] [--seconds S]
  run_cm_bench.py soak [--out DIR] [--minutes M] [--scan-hz H] [--cmd-hz H] [--readings N]

`core` runs bench_cm: the scan callback and the command callback of each
implementation, in process, on the same messages, for scans of 360, 1080
and 10,000 readings. `node` starts both node binaries side by side on the
same topics and measures the latency of each over real topics with cm_load.
`soak` does the same for a long time and samples the resident memory and
CPU time of both processes every 10 s.

Results go to DIR/cm_core.json, DIR/cm_node.json, DIR/cm_soak.json; a
summary is printed. Source the workspace first (pixi run bench-cm does).
The results are discussed in docs/collision_monitor/COMPARISON.md.
"""

import argparse
import json
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from ament_index_python.packages import get_package_prefix

BENCH = Path(get_package_prefix('roslean_bench')) / 'lib' / 'roslean_bench'
NODES = {
    'lean': Path(get_package_prefix('roslean_collision_monitor'))
    / 'lib' / 'roslean_collision_monitor' / 'collision_monitor',
    'upstream': Path(get_package_prefix('nav2_collision_monitor'))
    / 'lib' / 'nav2_collision_monitor' / 'collision_monitor',
}

# The zones of Nav2's example configuration, as in bench_cm.cpp.
PARAMS = """/**:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: bench_cmd
    transform_tolerance: 0.0
    source_timeout: 2.0
    stop_pub_timeout: 1.0
    base_shift_correction: false
    enable_stamped_cmd_vel: true
    bond_heartbeat_period: 0.0
    polygons: ["Stop", "Slow"]
    Stop:
      type: "polygon"
      points: "[[0.3, 0.3], [0.3, -0.3], [0.0, -0.3], [0.0, 0.3]]"
      action_type: "stop"
      min_points: 4
    Slow:
      type: "polygon"
      points: "[[0.4, 0.4], [0.4, -0.4], [-0.4, -0.4], [-0.4, 0.4]]"
      action_type: "slowdown"
      slowdown_ratio: 0.3
      min_points: 4
    observation_sources: ["scan"]
    scan:
      type: "scan"
      topic: "bench_scan"
"""


def quantiles(d):
    return f"{d['p50']:>8} {d['p90']:>8} {d['p99']:>8} {d['p999']:>8} {d['max']:>9}"


def run_core(out, quick):
    results = []
    print(f"{'readings':>8} {'zones':>8} {'scan':>6} {'path':>14} "
          f"{'p50':>8} {'p90':>8} {'p99':>8} {'p99.9':>8} {'max':>9}   ns per call")
    for readings in (360, 1080, 10000):
        for zones in ('polygons', 'circles'):
            for scan in ('clear', 'slow', 'near'):
                iters = (2000 if quick else 20000) if readings < 5000 else (500 if quick else 5000)
                line = subprocess.run(
                    [BENCH / 'bench_cm', '--readings', str(readings), '--zones', zones,
                     '--scan', scan, '--iters', str(iters), '--warmup', str(iters // 10)],
                    check=True, capture_output=True, text=True).stdout.strip().splitlines()[-1]
                r = json.loads(line)
                results.append(r)
                for path in ('lean_scan_ns', 'lean_cmd_ns', 'upstream_scan_ns', 'upstream_cmd_ns'):
                    print(f'{readings:>8} {zones:>8} {scan:>6} {path[:-3]:>14} {quantiles(r[path])}')
    (out / 'cm_core.json').write_text(json.dumps(results, indent=1))


def cpu_seconds(text):
    """`ps -o time=` prints [[dd-]hh:]mm:ss[.cc]."""
    days, _, rest = text.strip().rpartition('-')
    parts = [float(x) for x in rest.split(':')]
    while len(parts) < 3:
        parts.insert(0, 0.0)
    return (int(days) if days else 0) * 86400 + parts[0] * 3600 + parts[1] * 60 + parts[2]


def sample(pid):
    out = subprocess.run(['ps', '-o', 'rss=,time=', '-p', str(pid)], capture_output=True, text=True)
    fields = out.stdout.split()
    if len(fields) < 2:
        return None
    return int(fields[0]), cpu_seconds(fields[1])


class Pair:
    """Both nodes on the same topics, upstream activated through its lifecycle."""

    def __init__(self, workdir):
        self.params = Path(workdir) / 'cm_bench.yaml'
        self.params.write_text(PARAMS)
        self.procs = {}

    def start(self):
        for name, exe in NODES.items():
            self.procs[name] = subprocess.Popen(
                [exe, '--ros-args', '--params-file', self.params, '-r', f'__node:=cm_{name}',
                 '-p', f'cmd_vel_out_topic:=out_{name}', '--log-level', 'warn'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        import rclpy
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState
        rclpy.init()
        node = rclpy.create_node('cm_bench_starter')
        client = node.create_client(ChangeState, '/cm_upstream/change_state')
        if not client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError('upstream did not offer its lifecycle service')
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            req = ChangeState.Request()
            req.transition.id = transition
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=60.0)
            if not (future.done() and future.result().success):
                raise RuntimeError(f'upstream refused transition {transition}')
        node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        for p in self.procs.values():
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        for p in self.procs.values():
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()


def load(readings, scan_hz, cmd_hz, seconds, window=0, stdout=subprocess.PIPE):
    return subprocess.Popen(
        [BENCH / 'cm_load', '--readings', str(readings), '--scan-hz', str(scan_hz),
         '--cmd-hz', str(cmd_hz), '--duration', str(seconds), '--window', str(window),
         '--scan-topic', 'bench_scan', '--cmd-topic', 'bench_cmd', '--outs', 'out_lean,out_upstream'],
        stdout=stdout, text=True)


def run_node(out, seconds):
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        pair = Pair(tmp)
        pair.start()
        try:
            for readings in (360, 1080, 10000):
                p = load(readings, 25, 50, seconds)
                lines = [json.loads(x) for x in p.communicate()[0].strip().splitlines()]
                r = dict(lines[0], readings=readings, totals=lines[-1])
                results.append(r)
                for name in ('out_lean', 'out_upstream'):
                    d = r[name]
                    print(f"{readings:>6} readings {name:>12}: {d['n']} answers, latency us "
                          f"p50 {d['p50_ns'] / 1e3:.0f} p90 {d['p90_ns'] / 1e3:.0f} "
                          f"p99 {d['p99_ns'] / 1e3:.0f} max {d['max_ns'] / 1e3:.0f}; "
                          f"answers that differ between the nodes: {r['mismatches']} of {r['compared']} "
                          f"(lean less restrictive {r['first_less_restrictive']}, "
                          f"upstream less restrictive {r['second_less_restrictive']})")
        finally:
            pair.stop()
    (out / 'cm_node.json').write_text(json.dumps(results, indent=1))


def run_soak(out, minutes, scan_hz, cmd_hz, readings):
    result = {'minutes': minutes, 'scan_hz': scan_hz, 'cmd_hz': cmd_hz, 'readings': readings,
              'samples': [], 'windows': []}
    with tempfile.TemporaryDirectory() as tmp:
        pair = Pair(tmp)
        pair.start()
        try:
            log = open(Path(tmp) / 'load.jsonl', 'w+')
            p = load(readings, scan_hz, cmd_hz, minutes * 60, window=60, stdout=log)
            t0 = time.monotonic()
            next_sample = 0.0
            while p.poll() is None:
                now = time.monotonic() - t0
                if now >= next_sample:
                    row = {'t': round(now, 1)}
                    for name, proc in pair.procs.items():
                        s = sample(proc.pid)
                        if s is None:
                            raise RuntimeError(f'{name} is gone after {now:.0f} s')
                        row[name + '_rss_kb'], row[name + '_cpu_s'] = s
                    result['samples'].append(row)
                    if int(now) % 60 < 10:
                        print(row, flush=True)
                    next_sample += 10.0
                time.sleep(0.2)
            log.seek(0)
            for line in log.read().strip().splitlines():
                result['windows'].append(json.loads(line))
            for name, proc in pair.procs.items():
                result[name + '_alive_at_end'] = proc.poll() is None
        finally:
            pair.stop()
    (out / 'cm_soak.json').write_text(json.dumps(result, indent=1))
    print(json.dumps(result['windows'][-1]))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('what', choices=['core', 'node', 'soak', 'all'])
    ap.add_argument('--out', default='bench/results')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--minutes', type=float, default=30)
    ap.add_argument('--scan-hz', type=float, default=200)
    ap.add_argument('--cmd-hz', type=float, default=400)
    ap.add_argument('--readings', type=int, default=1080)
    opts = ap.parse_args()
    out = Path(opts.out)
    out.mkdir(parents=True, exist_ok=True)
    if opts.what in ('core', 'all'):
        run_core(out, opts.quick)
    if opts.what in ('node', 'all'):
        run_node(out, opts.seconds)
    if opts.what in ('soak', 'all'):
        run_soak(out, opts.minutes, opts.scan_hz, opts.cmd_hz, opts.readings)


if __name__ == '__main__':
    sys.exit(main())
