// Writes a case file for the equivalence check to stdout: calls of the C
// interface with random arguments, one per line (the format is described in
// native_driver.c). Deterministic in the seed.
//
//   node gen_cases.mjs <seed> <mux configurations> <monitor configurations>
//
// Mux: random handles (priorities also negative and above 255, timeouts
// zero, negative and positive), random input and lock events (indices also
// out of range), clock readings that move forwards and backwards and also
// lie above 2^31, 2^32, 2^53 and 2^63.
// Monitor: the shape of the demo's configuration (one scan source, polygon
// zones with stop and slowdown, min_points, ratio, the two timeouts) with
// random values, circles and topic zones besides, and values the core must
// refuse; then lifecycle transitions, scans and commands. Points are either
// what the page's shim computes from the scan or arbitrary doubles (NaN,
// infinities, huge values, values next to a zone's edge).

import { scanPoints } from '../../shim.js';

const [seed, nMux, nCm] = process.argv.slice(2).map(Number);

// xorshift32, good enough to spread test inputs.
let s = (seed * 2654435761 + 1) >>> 0 || 1;
function r32() { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s; }
function rnd() { return r32() / 4294967296; }
function int(a, b) { return a + Math.floor(rnd() * (b - a + 1)); }
function pick(xs) { return xs[int(0, xs.length - 1)]; }
function chance(p) { return rnd() < p; }
function r64() { return (BigInt(r32()) << 32n) | BigInt(r32()); }

const f64 = new Float64Array(1), u64 = new BigUint64Array(f64.buffer);
const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
const dbits = (v) => { f64[0] = v; return u64[0]; };
const fbits = (v) => { f32[0] = v; return u32[0]; };
const i64 = (v) => BigInt.asUintN(64, BigInt(v));
const hex = (bytes) => bytes.length === 0 ? '-' : Buffer.from(bytes.buffer, bytes.byteOffset, bytes.byteLength).toString('hex');

const lines = [];
const out = (...a) => { lines.push(a.join(' ')); if (lines.length > 2000) flush(); };
function flush() { process.stdout.write(lines.join('\n') + '\n'); lines.length = 0; }

// A double that is sometimes not an ordinary number.
function oddDouble() {
  return pick([NaN, Infinity, -Infinity, 0, -0, 1e300, -1e300, 5e-324, 999.9989, 999.999, 1000.5,
    -999.9989, 1e6, -1e6, 1e15, 4.9e-5, -4.9e-5]);
}

// ---------------------------------------------------------------- mux
function clockStart() {
  return pick([0n, 1n, 1000000000n, 2147483000n, 4294967000n, 1700000000000000000n,
    9007199254740000n, 9223372036854775000n, 18446744073709551000n, BigInt(int(0, 2000000000))]);
}
function muxTimeout() {
  if (chance(0.15)) return 0n;
  if (chance(0.08)) return i64(-int(1, 2000000000));
  if (chance(0.05)) return i64(r64() >> 1n);
  return BigInt(int(1, 2000)) * 1000000n;
}
function muxPriority() {
  if (chance(0.1)) return i64(-int(1, 500));
  if (chance(0.1)) return BigInt(int(256, 100000));
  if (chance(0.03)) return r64();
  return BigInt(int(0, 255));
}
for (let c = 0; c < nMux; c++) {
  const ni = int(0, 5), nl = int(0, 3);
  out('mux_builder');
  for (let i = 0; i < ni; i++) out('mux_add_input', muxPriority(), muxTimeout());
  for (let j = 0; j < nl; j++) out('mux_add_lock', muxPriority(), muxTimeout());
  out('mux_new');
  let t = clockStart();
  const ne = int(10, 80);
  for (let e = 0; e < ne; e++) {
    const step = chance(0.05) ? -BigInt(int(1, 3000)) * 1000000n :
      chance(0.1) ? BigInt(int(0, 5)) : BigInt(int(1, 900)) * 1000000n + BigInt(int(0, 999999));
    t = BigInt.asUintN(64, t + step);
    if (chance(0.03)) t = clockStart();
    if (chance(0.7)) out('mux_input', chance(0.05) ? int(ni, ni + 3) : int(0, Math.max(ni - 1, 0)), t);
    else out('mux_lock', chance(0.05) ? int(nl, nl + 3) : int(0, Math.max(nl - 1, 0)), int(0, 1), t);
  }
}

// ---------------------------------------------------------------- monitor
// About half of the monitor configurations are `clean`: every value ordinary,
// scans on time, points where the zones are. Those reach the zone decisions
// (pass, slowdown, stop, withheld) often; the others reach the refusals.
let clean = false;
const odd = (p) => !clean && chance(p);
function timeoutBits() {
  if (odd(0.1)) return dbits(0);
  if (odd(0.08)) return dbits(oddDouble());
  if (odd(0.05)) return dbits(-rnd());
  return dbits(clean ? pick([0.5, 1.0, 2.0, 0.5 + rnd()]) : pick([0.1, 0.2, 0.5, 1.0, 2.0, rnd() * 3]));
}
function ratioBits() {
  if (odd(0.08)) return dbits(oddDouble());
  if (odd(0.05)) return dbits(-rnd());
  if (odd(0.05)) return dbits(1 + rnd());
  return dbits(pick([0.4, 0.5, 0.0, 0.99, rnd()]));
}
function minPoints() {
  if (odd(0.05)) return i64(pick([0, -1, -5, 2147483647, 2147483648, -2147483648, -2147483649]));
  return BigInt(int(1, clean ? 4 : 8));
}
function vertexBits() {
  if (odd(0.03)) return fbits(oddDouble());
  return fbits((rnd() * 2 - 1) * pick([0.3, 0.8, 1.5, 3]));
}
function twist() {
  const v = [0, 0, 0, 0, 0, 0].map(() => dbits(0));
  if (chance(clean ? 0.05 : 0.15)) return v;                         // all zero
  v[0] = dbits(chance(0.1) ? 0 : (rnd() - 0.3) * 0.5);
  v[5] = dbits(chance(0.2) ? 0 : (rnd() - 0.5) * 2);
  if (chance(0.1)) v[1] = dbits((rnd() - 0.5) * 0.2);
  if (odd(0.03)) v[int(0, 5)] = dbits(oddDouble());
  if (odd(0.03)) v[int(0, 5)] = dbits(-0);
  return v;
}
function conv() {
  const d = chance(0.3) ? dbits(oddDouble()) : chance(0.5) ? r64() : dbits((rnd() - 0.5) * pick([1, 10, 2000, 1e7]));
  const which = pick(['coord', 'vcoord', 'radius', 'vertex', 'timeout', 'srctimeout', 'noop']);
  if (which === 'vertex') out('conv', which, chance(0.5) ? r32() : fbits((rnd() - 0.5) * 2000));
  else if (which === 'srctimeout') out('conv', which, d, int(0, 1), chance(0.5) ? timeoutBits() : r64());
  else out('conv', which, d);
}

for (let c = 0; c < nCm; c++) {
  // Configuration.
  clean = chance(0.5);
  const zones = [];
  const nz = odd(0.05) ? 0 : int(1, 3);
  out('cm_builder');
  const setSource = !odd(0.03);
  const first = chance(0.5);
  if (setSource && first) out('cm_source', odd(0.1) ? 0 : 1, timeoutBits(), chance(0.2) ? 1 : 0, timeoutBits());
  out('cm_stop_pub', timeoutBits());
  if (odd(0.01)) out('cm_vertex', vertexBits(), vertexBits());        // a vertex that belongs to nothing
  for (let k = 0; k < nz; k++) {
    const action = odd(0.03) ? int(2, 255) : int(0, 1), enabled = odd(0.1) ? 0 : 1;
    const kind = clean ? pick(['polygon', 'polygon', 'circle']) : pick(['polygon', 'polygon', 'polygon', 'circle', 'topic']);
    if (kind === 'circle') {
      const radius = odd(0.06) ? oddDouble() : rnd() * pick([0.3, 0.6, 1.2]);
      out('cm_circle', dbits(radius), action, ratioBits(), minPoints(), enabled);
      zones.push({ kind, radius });
    } else if (kind === 'topic') {
      out('cm_topic_polygon', action, ratioBits(), minPoints(), enabled);
      zones.push({ kind });
    } else {
      out('cm_polygon', action, ratioBits(), minPoints(), enabled);
      const verts = [];
      if (clean || chance(0.6)) {
        // A rectangle around the robot, as in the demo, with random extents.
        const x1 = Math.fround(0.1 + rnd() * 0.9), x0 = Math.fround(-rnd() * 0.3), y = Math.fround(0.05 + rnd() * 0.4);
        verts.push([x1, y], [x1, -y], [x0, -y], [x0, y]);
        for (const v of verts) out('cm_vertex', fbits(v[0]), fbits(v[1]));
      } else {
        const nv = chance(0.1) ? int(0, 3) : int(4, 7);
        for (let i = 0; i < nv; i++) { const x = vertexBits(), y = vertexBits(); out('cm_vertex', x, y); u32[0] = x; const fx = f32[0]; u32[0] = y; verts.push([fx, f32[0]]); }
      }
      zones.push({ kind, verts });
    }
  }
  if (setSource && !first) out('cm_source', odd(0.1) ? 0 : 1, timeoutBits(), chance(0.2) ? 1 : 0, timeoutBits());
  out('cm_new', nz);
  if (!odd(0.08)) { out('cm_lifecycle', 0); out('cm_lifecycle', 1); }
  else out('cm_lifecycle', int(0, 6));

  // Events.
  let t = pick([0n, 1000000000n, 4294967000n, 1700000000000000000n, BigInt(int(0, 2000000000))]);
  let scan = null, havePoints = false;
  const ne = int(15, 45);
  for (let e = 0; e < ne; e++) {
    t = BigInt.asUintN(64, t + (odd(0.04) ? -BigInt(int(1, 2000)) * 1000000n : BigInt(int(1, clean ? 120 : 400)) * 1000000n + BigInt(int(0, 999999))));
    const what = rnd();
    if (what < (clean ? 0.3 : 0.25) || (clean && e === 0)) {
      const n = pick([0, 1, 3, 8, 8, 24, 24, 60, 90, chance(0.3) ? 360 : 12]);
      const ranges = new Float32Array(n);
      const near = pick([0.2, 0.4, 0.8, 2.0]);
      for (let i = 0; i < n; i++) {
        ranges[i] = chance(0.25) ? Infinity : odd(0.03) ? NaN : chance(0.03) ? 0.05 : odd(0.03) ? -1 : odd(0.02) ? 50 : near * (0.5 + rnd());
      }
      const bytes = odd(0.03) && n > 0 ? new Uint8Array(ranges.buffer, 0, 4 * n - int(1, 3)) : new Uint8Array(ranges.buffer);
      const age = odd(0.1) ? BigInt(int(0, 3000)) * 1000000n : BigInt(int(0, 50)) * 1000000n;
      const stamp = odd(0.03) ? i64(-int(1, 1000000)) : odd(0.03) ? BigInt.asUintN(64, t + 500000000n) : BigInt.asUintN(64, t - age);
      out('cm_scan', t, stamp, fbits(clean ? 0.12 : pick([0.12, 0.0, 0.3])), fbits(clean ? 3.5 : pick([3.5, 12.0, 0.5, Infinity])), hex(bytes));
      scan = { ranges, angleMin: pick([0, -Math.PI]), inc: n ? 2 * Math.PI / n : 0 };
      havePoints = false;
    } else if (!clean && what < 0.29) {
      const k = int(0, nz + 1);
      const nv = int(0, 7), vs = new Float64Array(2 * nv);
      for (let i = 0; i < 2 * nv; i++) vs[i] = chance(0.04) ? oddDouble() : (rnd() - 0.5) * 2;
      const bytes = chance(0.05) && nv ? new Uint8Array(vs.buffer, 0, 16 * nv - int(1, 15)) : new Uint8Array(vs.buffer);
      out('cm_shape', k, int(0, 1), chance(0.9) ? 1 : 0, hex(bytes));
    } else if (!clean && what < 0.32) {
      out('cm_lifecycle', int(0, 5));
    } else if (!clean && what < 0.35) {
      out('cm_cmd_noclock', ...twist());
    } else if (what < 0.38) {
      conv();
    } else {
      const v = twist();
      if (chance(0.3)) out('cm_plan', t, ...v);
      // Points: the shim's, or arbitrary, or of the wrong number, or none.
      let has = 1, pts = '-';
      const n = scan ? scan.ranges.length : 0;
      const mode = rnd();
      if (havePoints && mode < 0.5) pts = '=';
      else if (mode < 0.8 && scan) {
        pts = hex(new Uint8Array(scanPoints(scan.angleMin, scan.inc, scan.ranges, pick([0, -0.032]), 0).buffer));
      } else if (mode < 0.95) {
        const m = odd(0.1) ? int(0, n + 2) : n, a = new Float64Array(2 * m);
        for (let i = 0; i < 2 * m; i++) {
          if (odd(0.03)) a[i] = oddDouble();
          else if (chance(clean ? 0.6 : 0.3) && zones.length) {
            // next to an edge or a vertex of a zone, within a few tenths of a millimetre
            const z = pick(zones);
            const base = z.kind === 'polygon' && z.verts.length ? pick(z.verts)[i % 2] : z.kind === 'circle' ? z.radius * (i % 2 ? Math.sin(i) : Math.cos(i)) : 0;
            a[i] = base + (rnd() - 0.5) * pick([0, 2e-4, 2e-3, 0.05]);
          } else a[i] = (rnd() - 0.5) * pick([0.5, 1.5, 4]);
        }
        pts = hex(new Uint8Array(a.buffer));
      } else if (clean) pts = hex(new Uint8Array(new Float64Array(2 * n).fill(2.5).buffer));
      else has = 0;
      if (pts !== '=' ) havePoints = pts !== '-';
      // Zone vertices: mostly none; sometimes records, well formed or not.
      let zv = '-';
      if (odd(0.12)) {
        const parts = [];
        for (let k = 0; k < nz + (chance(0.2) ? 1 : 0); k++) {
          const m = chance(0.4) ? 0xFFFFFFFF : int(0, 6);
          const head = new Uint8Array(4); new DataView(head.buffer).setUint32(0, m, true);
          parts.push(head);
          if (m !== 0xFFFFFFFF) { const a = new Float64Array(2 * m); for (let i = 0; i < 2 * m; i++) a[i] = chance(0.03) ? oddDouble() : (rnd() - 0.5) * 2; parts.push(new Uint8Array(a.buffer)); }
        }
        let all = Buffer.concat(parts.map((p) => Buffer.from(p.buffer, p.byteOffset, p.byteLength)));
        if (chance(0.1) && all.length) all = all.subarray(0, all.length - int(1, Math.min(5, all.length)));
        zv = all.length ? all.toString('hex') : '-';
      }
      // The reference definition is slow: for small scans only.
      let again = false;
      if (n <= 24 && chance(0.25)) { out('cm_cmd_ref', t, ...v, has, pts, zv); again = pts !== '-'; }
      out('cm_cmd', t, ...v, has, again ? '=' : pts, zv);
    }
  }
}
flush();
