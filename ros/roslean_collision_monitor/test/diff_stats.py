#!/usr/bin/env python3
"""Adds up the counts of several runs of `random_diff.py --roslean --stats FILE`.

    diff_stats.py FILE [FILE ...]

A long run can be split over several processes, each with its own range of
seeds and its own ROS_DOMAIN_ID (bench/cm_linux/run.sh does); this prints
the totals and the table of docs/collision_monitor/COMPARISON.md: how far
outside upstream's zone the point lay that decided each difference.
"""

import json
import sys

BINS = [0.0, 1e-6, 0.25e-3, 0.5e-3, 1e-3, 1.5e-3, 2e-3, 2.5e-3, 3e-3, 3.5e-3, 4e-3, 4.4e-3]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    total = {'cases': 0, 'commands': 0, 'differ': 0, 'published_differ': 0, 'ambiguous': 0,
             'scans': 0, 'reach': [], 'failed_seeds': [], 'retried_seeds': [],
             'band': {'circle': [0, 0], 'polygon': [0, 0]}}
    for path in sys.argv[1:]:
        with open(path) as f:
            part = json.load(f)
        for key in ('cases', 'commands', 'differ', 'published_differ', 'ambiguous', 'scans'):
            total[key] += part[key]
        for key in ('reach', 'failed_seeds', 'retried_seeds'):
            total[key] += part[key]
        for kind in total['band']:
            for k in (0, 1):
                total['band'][kind][k] += part['band'][kind][k]

    def pct(a, b):
        return f'{100 * a / b:.2f} %' if b else 'n/a'

    print(f"cases                               {total['cases']}")
    print(f"cases outside the bounds            {len(total['failed_seeds'])} "
          f"{sorted(total['failed_seeds'])}")
    print(f"cases passed only on the slow retry {len(total['retried_seeds'])} "
          f"{sorted(total['retried_seeds'])}")
    print(f"commands                            {total['commands']}")
    print(f"scans                               {total['scans']}")
    for kind in ('circle', 'polygon'):
        band, pts = total['band'][kind]
        print(f'point tests, {kind:8}               {pts}, in the growth band {band}')
    print(f"commands that depend on a band point {total['ambiguous']} "
          f"({pct(total['ambiguous'], total['commands'])})")
    print(f"commands with different demands     {total['differ']} "
          f"({pct(total['differ'], total['commands'])} of all, "
          f"{pct(total['differ'], total['ambiguous'])} of the dependent ones)")
    print(f"commands published differently      {total['published_differ']}")
    print()
    print('| Deciding point outside upstream\'s zone by | Circle | Polygon |')
    print('| --- | ---: | ---: |')
    count = {'circle': [0] * len(BINS), 'polygon': [0] * len(BINS)}
    far = {'circle': 0.0, 'polygon': 0.0}
    for d, kind in total['reach']:
        far[kind] = max(far[kind], d)
        k = max(i for i, lo in enumerate(BINS) if d >= lo) if d > 0 else 0
        # Bin 0 is "not outside at all" (within the 1 nm in which the test
        # does not claim to know upstream's verdict).
        if 0 < d < BINS[1]:
            k = 0
        count[kind][k] += 1

    def mm(x):
        return f'{x * 1000:g} mm'

    labels = ['below 1 µm'] + [f'{mm(BINS[i])} to {mm(BINS[i + 1])}'
                               for i in range(1, len(BINS) - 1)] + [f'beyond {mm(BINS[-1])}']
    for i, label in enumerate(labels):
        print(f"| {label} | {count['circle'][i]} | {count['polygon'][i]} |")
    print(f"| all | {sum(count['circle'])} | {sum(count['polygon'])} |")
    print(f"| farthest | {far['circle'] * 1000:.3f} mm | {far['polygon'] * 1000:.3f} mm |")


if __name__ == '__main__':
    main()
