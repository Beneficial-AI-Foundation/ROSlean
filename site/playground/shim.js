// The JavaScript shim: what the C++ of the two ROS nodes does around the Lean
// core, done here for the browser. It configures the core, hands over each
// message as one event, reads the core's answer and carries it out.
//
// No function in this file decides whether a message is forwarded, which
// zone is triggered or what is published. Those answers come from the
// functions of core/ROSleanCore (FFI.lean, CollisionMonitorFFI.lean), compiled
// to roslean_core.wasm, through the same C interface the ROS nodes call
// (ros/roslean_core/include/roslean_core/ffi.h).
//
// What is computed here, in floating point, as in the C++ shim
// (docs/collision_monitor/SPEC.md, "Shim boundary" and "Where the velocity
// is computed"): the points of a scan in the base frame, and the three
// multiplications of a slowdown.

import createRosleanCore from './roslean_core.js';

const f64 = new Float64Array(1), u64 = new BigUint64Array(f64.buffer);
const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);

/** The bit pattern of a double, as the C++ `bits_of`. */
export function bitsOfDouble(v) { f64[0] = v; return u64[0]; }
export function doubleOfBits(b) { u64[0] = b; return f64[0]; }
/** The bit pattern of the float nearest to `v`. */
export function bitsOfFloat(v) { f32[0] = v; return u32[0]; }

/** Seconds to whole nanoseconds, truncated, as rclcpp::Duration does for the mux's timeouts. */
export function secondsToNs(s) { return BigInt(Math.trunc(s * 1e9)); }

/** Loads roslean_core.wasm and initialises the Lean runtime and the core, once. */
export async function loadCore(options) {
  const M = await createRosleanCore(options);
  if (M._roslean_core_initialize() === 0) throw new Error('the Lean core failed to initialise');
  return M;
}

function byteArray(M, bytes) {
  // roslean_core_byte_array copies `size` bytes into a new Lean ByteArray.
  if (bytes.length === 0) return M._roslean_core_byte_array(0, 0);
  const p = M._malloc(bytes.length);
  M.HEAPU8.set(bytes, p);
  const a = M._roslean_core_byte_array(p, bytes.length);
  M._free(p);
  return a;
}

/** twist_mux. The counterpart of ros/roslean_core/include/roslean_core/mux.hpp. */
export class Mux {
  /**
   * @param inputs, locks arrays of {priority, timeout} (timeout in seconds),
   *   in the order of the parameter file; the index of a handle is its position.
   */
  constructor(M, inputs, locks) {
    this.M = M;
    let b = M._roslean_mux_builder_new(M._roslean_core_unit());
    for (const h of inputs) b = M._roslean_mux_builder_add_input(b, BigInt.asUintN(64, BigInt(h.priority)), BigInt.asUintN(64, secondsToNs(h.timeout)));
    for (const h of locks) b = M._roslean_mux_builder_add_lock(b, BigInt.asUintN(64, BigInt(h.priority)), BigInt.asUintN(64, secondsToNs(h.timeout)));
    this.obj = M._roslean_mux_new(b);
  }
  /** A message arrived on input `i` at node-clock time `tNs`. Returns whether to forward it. */
  onInput(i, tNs) {
    const M = this.M;
    this.obj = M._roslean_mux_on_input(this.obj, i, BigInt(tNs));
    M._roslean_core_inc(this.obj);            // the query consumes one reference
    return M._roslean_mux_forward(this.obj) !== 0;
  }
  /** A message with value `value` arrived on lock `j`. */
  onLock(j, value, tNs) {
    this.obj = this.M._roslean_mux_on_lock(this.obj, j, value ? 1 : 0, BigInt(tNs));
  }
  free() { if (this.obj) { this.M._roslean_core_dec(this.obj); this.obj = 0; } }
}

/** The codes of roslean_cm_out_kind and roslean_cm_out_report (ffi.h). */
export const Kind = { Silent: 0, Pass: 1, Slowdown: 2, StopZone: 3, StopSource: 4, Withheld: 5 };
export const Report = { None: 0, Normal: 1, Stop: 2, Slowdown: 3, Source: 4 };

/**
 * The points of a scan in the base frame, 16 bytes per reading: the order of
 * operations of `scan_points` in the C++ shim (SPEC, "Scan and shape
 * events"). The angle and the products are single precision (Math.fround);
 * cos and sin are JavaScript's, in double precision and then rounded, which
 * is not the C library's `cosf`. The transform is a translation by (tx, ty):
 * the only one this page has.
 */
export function scanPoints(angleMin, angleIncrement, ranges, tx, ty) {
  const out = new Float64Array(2 * ranges.length);
  let angle = Math.fround(angleMin);
  const inc = Math.fround(angleIncrement);
  for (let i = 0; i < ranges.length; i++) {
    const xf = Math.fround(ranges[i] * Math.fround(Math.cos(angle)));
    const yf = Math.fround(ranges[i] * Math.fround(Math.sin(angle)));
    out[2 * i] = xf + tx;
    out[2 * i + 1] = yf + ty;
    angle = Math.fround(angle + inc);
  }
  return out;
}

/**
 * The collision monitor. The counterpart of collision_monitor.hpp and of the
 * parts of collision_monitor_node.cpp that keep the last scan and carry out
 * the core's answer.
 */
export class Monitor {
  /**
   * @param cfg {sourceTimeout, stopPubTimeout, zones: [{name, action: 'stop'|'slowdown',
   *   slowdownRatio, minPoints, enabled, points: [[x, y], ...]}], sensor: [tx, ty]}
   *   with the values of the parameter file. Every number goes to the core as
   *   read; the core converts and refuses.
   */
  constructor(M, cfg) {
    this.M = M;
    this.cfg = cfg;
    let b = M._roslean_cm_builder_new(M._roslean_core_unit());
    b = M._roslean_cm_builder_set_source(b, 1, bitsOfDouble(cfg.sourceTimeout), 0, 0n);
    b = M._roslean_cm_builder_set_stop_pub_timeout(b, bitsOfDouble(cfg.stopPubTimeout));
    for (const z of cfg.zones) {
      b = M._roslean_cm_builder_add_polygon(
        b, z.action === 'slowdown' ? 1 : 0, bitsOfDouble(z.slowdownRatio === undefined ? 0.5 /* upstream's default */ : z.slowdownRatio),
        BigInt.asUintN(64, BigInt(z.minPoints)), z.enabled ? 1 : 0);
      for (const p of z.points) b = M._roslean_cm_builder_add_vertex(b, bitsOfFloat(p[0]), bitsOfFloat(p[1]));
    }
    this.obj = M._roslean_cm_new(b);
    M._roslean_core_inc(this.obj);
    if (M._roslean_cm_ok(this.obj) === 0) {
      M._roslean_core_inc(this.obj);
      throw new Error('the core refused the configuration, code ' + M._roslean_cm_error(this.obj));
    }
    // The lifecycle manager's part: configure, activate.
    this.obj = M._roslean_cm_on_lifecycle(this.obj, 0);
    this.obj = M._roslean_cm_on_lifecycle(this.obj, 1);
    this.scan = null;
  }

  /**
   * A scan arrived at node-clock time `nowNs`. The ranges go to the core;
   * the message is kept for the points of later commands, as the node does.
   * @param scan {stampNs, angleMin, angleIncrement, rangeMin, rangeMax, ranges: Float32Array}
   */
  onScan(nowNs, scan) {
    const M = this.M;
    this.scan = scan;
    const bytes = new Uint8Array(scan.ranges.buffer, scan.ranges.byteOffset, 4 * scan.ranges.length);
    this.obj = M._roslean_cm_on_scan(
      this.obj, BigInt(nowNs), BigInt.asUintN(64, BigInt(scan.stampNs)), bitsOfFloat(scan.rangeMin),
      bitsOfFloat(scan.rangeMax), byteArray(M, bytes));
  }

  /**
   * A velocity command arrived at node-clock time `nowNs`.
   * @param cmd [linear.x, linear.y, angular.z]
   * @return {kind, zone, ratio, report, reportZone, publish: [x, y, w] or null}
   *   `publish` is what the node would publish: null for nothing.
   */
  onCmd(nowNs, cmd) {
    const M = this.M, t = BigInt(nowNs);
    const lx = bitsOfDouble(cmd[0]), ly = bitsOfDouble(cmd[1]), az = bitsOfDouble(cmd[2]), z = 0n;
    // Which transforms the node would look up is the core's answer; 3 means
    // the scan's. This page has no zones fed by a topic.
    M._roslean_core_inc(this.obj);
    const plan = M._roslean_cm_cmd_plan(this.obj, t, lx, ly, z, z, z, az);
    let has = 0, points = new Uint8Array(0);
    if (plan === 3 && this.scan) {
      const s = this.scan, c = this.cfg.sensor || [0, 0];
      const pts = scanPoints(s.angleMin, s.angleIncrement, s.ranges, c[0], c[1]);
      points = new Uint8Array(pts.buffer);
      has = 1;
      this.points = pts;                       // for drawing only
    }
    this.obj = M._roslean_cm_on_cmd(
      this.obj, t, lx, ly, z, z, z, az, has, byteArray(M, points), byteArray(M, new Uint8Array(0)));
    const out = this.readOutput();
    out.publish = applyAnswer(out.kind, out.ratioBits, cmd);
    return out;
  }

  readOutput() {
    const M = this.M, o = {};
    M._roslean_core_inc(this.obj); o.kind = M._roslean_cm_out_kind(this.obj);
    M._roslean_core_inc(this.obj); o.zone = M._roslean_cm_out_zone(this.obj);
    M._roslean_core_inc(this.obj); o.ratioBits = M._roslean_cm_out_ratio(this.obj);
    M._roslean_core_inc(this.obj); o.report = M._roslean_cm_out_report(this.obj);
    M._roslean_core_inc(this.obj); o.reportZone = M._roslean_cm_out_report_zone(this.obj);
    o.ratio = o.kind === Kind.Slowdown ? doubleOfBits(o.ratioBits) : null;
    return o;
  }

  free() { if (this.obj) { this.M._roslean_core_dec(this.obj); this.obj = 0; } }
}

/**
 * What is published for the core's answer: `apply_answer` of the C++ shim.
 * Pass: the command. Slowdown: each component times the ratio, in double.
 * Stop: zero. Silent and withheld: nothing. An unknown kind: zero.
 */
export function applyAnswer(kind, ratioBits, cmd) {
  switch (kind) {
    case Kind.Silent: case Kind.Withheld: return null;
    case Kind.Pass: return [cmd[0], cmd[1], cmd[2]];
    case Kind.Slowdown: { const r = doubleOfBits(ratioBits); return [cmd[0] * r, cmd[1] * r, cmd[2] * r]; }
    default: return [0, 0, 0];
  }
}
