#!/usr/bin/env python3
"""Review CM5 (task T08): would the tests fail on a plausible bug?

    mutations.py SCRATCH [NAME ...]

SCRATCH is a scratch copy of the repository's `core/`, `lean-toolchain`,
`ros/roslean_core` and `ros/roslean_collision_monitor`, already built once
with `colcon build --base-paths ros --cmake-args -G Ninja` run inside
SCRATCH (so that `build/` and `install/` are there). Nothing in the real
checkout is touched. Run inside the pixi environment.

For each mutation: one exact text replacement in one file, rebuild, run the
process-level tests (no ROS graph), restore. A mutation is "caught" by a test
if the test exits non-zero. The theorems are not rebuilt here: a mutation of
the Lean fast path would also break `lake build` of the proofs
(`stepCmdFast_eq`), which is the stronger guard; the question here is what
the *tests* notice.
"""
import os
import subprocess
import sys

TRANSFORMS = 'ros/roslean_collision_monitor/include/roslean_collision_monitor/transforms.hpp'
FAST = 'core/ROSleanCore/CollisionMonitorFast.lean'
FFI = 'core/ROSleanCore/CollisionMonitorFFI.lean'

# name: (file, old, new, what)
MUTATIONS = {
    # ---- the trusted transform (transforms.hpp) ----------------------------
    't-swap-frames': (TRANSFORMS,
                      't = buffer.lookupTransform(r.target, r.source, tf2::TimePointZero);',
                      't = buffer.lookupTransform(r.source, r.target, tf2::TimePointZero);',
                      'N23: target and source frame swapped'),
    't-swap-times': (TRANSFORMS,
                     'r.target_ns = now_ns;\n    r.source_ns = scan_stamp_ns;',
                     'r.target_ns = scan_stamp_ns;\n    r.source_ns = now_ns;',
                     'N24: the two times swapped'),
    't-shift-identity': (TRANSFORMS,
                         '  } else {\n    r.identity = scan_frame == base_frame;\n  }',
                         '  }\n  r.identity = scan_frame == base_frame;\n'
                         '  if (r.identity) {r.through_fixed = false;}',
                         'N24: no lookup when the scan is in the base frame'),
    't-angle-skips-invalid': (TRANSFORMS,
                              '    put_pair(out.data() + kPairSize * i, b[0], b[1]);\n'
                              '    angle += angle_increment;',
                              '    put_pair(out.data() + kPairSize * i, b[0], b[1]);\n'
                              '    if (std::isfinite(ranges[i])) {angle += angle_increment;}',
                              'R3: the angle does not advance over a reading that is not finite'),
    't-angle-double': (TRANSFORMS,
                       '  float angle = angle_min;\n  for (size_t i = 0; i < ranges.size(); ++i) {\n'
                       '    const auto s = scan_frame_point(ranges[i], angle);',
                       '  double angle = angle_min;\n  for (size_t i = 0; i < ranges.size(); ++i) {\n'
                       '    const auto s = scan_frame_point(ranges[i], static_cast<float>(angle));',
                       'R3: the angle accumulated in double precision'),
    't-cos-double': (TRANSFORMS,
                     'const float xf = range * std::cos(angle);',
                     'const float xf = static_cast<float>(range * std::cos(static_cast<double>(angle)));',
                     'R3: the cosine taken in double precision'),
    't-xy-swapped': (TRANSFORMS,
                     '  return {p_b.x(), p_b.y()};',
                     '  return {p_b.y(), p_b.x()};',
                     'N25: x and y of the image exchanged'),
    't-polygon-uses-z': (TRANSFORMS,
                         '  const tf2::Vector3 p_s(x, y, 0.0);',
                         '  const tf2::Vector3 p_s(x, y, 1.0);',
                         'N25: the point is taken at height 1, not 0'),
    't-zero-quaternion': (TRANSFORMS,
                          '  return finite && !zero;',
                          '  return finite;',
                          'ND7: a zero quaternion is used'),
    't-fixed-is-target': (TRANSFORMS,
                          '    r.fixed = odom_frame;',
                          '    r.fixed = base_frame;',
                          'N24: the fixed frame is the base frame'),
    't-latest-is-now': (TRANSFORMS,
                        't = buffer.lookupTransform(r.target, r.source, tf2::TimePointZero);',
                        't = buffer.lookupTransform(r.target, r.source, '
                        'tf2::TimePoint(std::chrono::nanoseconds(r.target_ns)));',
                        'N23: the lookup at a time (here 0) instead of the newest'),
    # ---- the fast command path (CollisionMonitorFast.lean, FFI) -------------
    'f-box-margin': (FAST,
                     'decide (b.minX - 2 ≤ p.x) && decide (p.x ≤ b.maxX + 2) &&\n'
                     '    decide (b.minY - 2 ≤ p.y) && decide (p.y ≤ b.maxY + 2)',
                     'decide (b.minX ≤ p.x) && decide (p.x ≤ b.maxX) &&\n'
                     '    decide (b.minY ≤ p.y) && decide (p.y ≤ b.maxY)',
                     'the band of D4 is tested only for points inside the bounding box'),
    'f-min-points': (FAST,
                     'z.enabled && z.shape.isSet && decide (z.minPoints ≤ z.countFast pts)',
                     'z.enabled && z.shape.isSet && decide (z.minPoints < z.countFast pts)',
                     'R11: a zone needs one point more than min_points'),
    'f-index-skips-invalid': (FAST,
                              '  | b :: bs, i, acc => Scan.pointsGo s f bs (i + 1) '
                              '(if s.validBeam b then acc.push (f i) else acc)',
                              '  | b :: bs, i, acc => if s.validBeam b then Scan.pointsGo s f bs (i + 1) '
                              '(acc.push (f i)) else Scan.pointsGo s f bs i acc',
                              'coordinate i is taken for the i-th *valid* reading'),
    'f-slowdown-tie': (FAST,
                       'if zi.1.triggeredFast pts && decide (r < best.1) then (r, some zi.2) else best',
                       'if zi.1.triggeredFast pts && decide (r ≤ best.1) then (r, some zi.2) else best',
                       'R13: of two slowdown zones with one ratio the later one is reported'),
    'f-stop-stamp': (FAST,
                     'stopStamp := if zero && !st.prevZero then now else st.stopStamp',
                     'stopStamp := if zero then now else st.stopStamp',
                     'R15: the stop stamp moves with every zero, so the zero never ceases'),
    'f-circle-le': (FAST,
                    'pts.countP (fun p => decide (p.x * p.x + p.y * p.y < rr))',
                    'pts.countP (fun p => decide (p.x * p.x + p.y * p.y ≤ rr))',
                    'a point exactly on the grown circle counts'),
    'f-timeout-le': (FAST,
                     'if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout < now then none',
                     'if 0 < cfg.sourceTimeout && s.stamp + cfg.sourceTimeout ≤ now then none',
                     'R7: a scan exactly as old as the timeout is too old'),
    'f-count-ge': (FAST,
                   '          if n == s.beams.length then some (s.pointsBy f)',
                   '          if n ≥ s.beams.length then some (s.pointsBy f)',
                   'more coordinates than readings are accepted'),
}

TESTS = [
    ('test_cm_ffi', 'build/roslean_core/test_cm_ffi'),
    ('smoke_transforms', 'build/roslean_collision_monitor/smoke_transforms'),
    ('smoke_conversions', 'build/roslean_collision_monitor/smoke_conversions'),
    ('band_bounds', 'build/roslean_collision_monitor/band_bounds 20000'),
    ('t2_point_as_upstream', 'build/roslean_collision_monitor/t2_point_as_upstream'),
]


def sh(cmd, cwd):
    return subprocess.run(cmd, shell=True, cwd=cwd, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True, timeout=900)


def main():
    scratch = os.path.abspath(sys.argv[1])
    names = sys.argv[2:] or list(MUTATIONS)
    rows = []
    for name in names:
        path, old, new, what = MUTATIONS[name]
        full = os.path.join(scratch, path)
        text = open(full, encoding='utf-8').read()
        if text.count(old) != 1:
            rows.append((name, what, 'NOT APPLIED (text found %d times)' % text.count(old)))
            continue
        open(full, 'w', encoding='utf-8').write(text.replace(old, new))
        try:
            b = sh('colcon build --base-paths ros --cmake-args -G Ninja', scratch)
            if b.returncode != 0:
                rows.append((name, what, 'does not build: ' + b.stdout[-300:].replace('\n', ' ')))
                continue
            wanted = TESTS if path != TRANSFORMS else [t for t in TESTS if t[0] != 'test_cm_ffi']
            caught = []
            for test, cmd in wanted:
                r = sh(cmd, scratch)
                if r.returncode != 0:
                    fails = [line for line in r.stdout.splitlines() if 'FAIL' in line]
                    caught.append('%s (%d failing checks)' % (test, len(fails)))
            rows.append((name, what, 'caught by ' + ', '.join(caught) if caught else 'MISSED'))
        finally:
            open(full, 'w', encoding='utf-8').write(text)
        print('%-24s %s' % (name, rows[-1][2]), flush=True)
    sh('colcon build --base-paths ros --cmake-args -G Ninja', scratch)
    print()
    for name, what, result in rows:
        print('| `%s` | %s | %s |' % (name, what, result))


if __name__ == '__main__':
    main()
