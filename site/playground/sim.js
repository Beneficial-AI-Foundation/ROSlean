// The toy world of the playground: a robot, a lidar, three boxes, a planner,
// a joystick, an emergency stop and a base with a watchdog, and the wiring
// that carries messages between them and the two nodes.
//
// Everything in this file is written for the page and none of it is the
// project's verified code. The two nodes are in shim.js and, behind it, in
// roslean_core.wasm. This file never decides what a node does: it delivers
// messages to the shim and does what comes back. No DOM here, so the same
// code runs under node for the tests.

import { Mux, Monitor, Kind, Report } from './shim.js';

/** The parameters of the project's Gazebo demo. */
export const PARAMS = {
  // ros/roslean_demos/gazebo/mux/gazebo.yaml
  mux: {
    inputs: [
      { name: 'planner', priority: 10, timeout: 0.5 },
      { name: 'joystick', priority: 100, timeout: 0.5 },
    ],
    locks: [{ name: 'e_stop', priority: 255, timeout: 0.0 }],
  },
  // ros/roslean_demos/gazebo/monitor/monitor.yaml
  monitor: {
    sourceTimeout: 1.0,
    stopPubTimeout: 1.0,
    zones: [
      { name: 'Stop', action: 'stop', minPoints: 4, enabled: true,
        points: [[0.30, 0.14], [0.30, -0.14], [-0.14, -0.14], [-0.14, 0.14]] },
      { name: 'Slow', action: 'slowdown', slowdownRatio: 0.4, minPoints: 4, enabled: true,
        points: [[0.70, 0.22], [0.70, -0.22], [-0.14, -0.22], [-0.14, 0.22]] },
    ],
    // The lidar sits 0.032 m behind the centre of the base (the TurtleBot3
    // burger's base_scan frame). A translation is the only transform here.
    sensor: [-0.032, 0],
  },
  watchdog: 0.3,              // s; ros/roslean_demos/gazebo/mux/base_watchdog.py
};

// The toy parts. None of these numbers is a parameter of a node.
const STEP_NS = 10_000_000;   // one step of the world: 10 ms
const SCAN_EVERY = 20;        // steps: 5 scans a second, as the demo's lidar
const CMD_EVERY = 10;         // steps: planner and joystick publish 10 times a second
const BEAMS = 360, RANGE_MIN = 0.12, RANGE_MAX = 3.5;
const PATROL_R = 1.45, PATROL_V = 0.18;
const ROBOT_R = 0.105;
export const WORLD = { xMin: -3.0, xMax: 3.0, yMin: -2.1, yMax: 2.1, patrolRadius: PATROL_R, robotRadius: ROBOT_R };
export const WIRINGS = ['monitor-last', 'joystick-bypass'];
const NS = 1e9;

export class World {
  /** @param M the loaded core (shim.js, loadCore) */
  constructor(M) {
    this.M = M;
    this.wiring = 'monitor-last';
    this.reset();
  }

  /** Everything back to the start, the two nodes included; the wiring is kept. */
  reset() {
    if (this.mux) this.mux.free();
    if (this.monitor) this.monitor.free();
    this.mux = new Mux(this.M, PARAMS.mux.inputs, PARAMS.mux.locks);
    this.monitor = new Monitor(this.M, PARAMS.monitor);
    // The node clock starts at 1 s: the mux's inputs start with a stamp of
    // 0, so in the first half second after 0 the joystick would count as
    // live and mask the planner.
    this.step = 0;
    this.tNs = NS;
    this.robot = { x: PATROL_R, y: 0, th: Math.PI / 2 };
    this.boxes = [{ x: -2.4, y: 1.5, s: 0.28 }, { x: -2.4, y: 0.95, s: 0.28 }, { x: -2.4, y: 0.4, s: 0.28 }];
    this.keys = { up: false, down: false, left: false, right: false };
    this.script = null;                      // a joystick command held until a time
    this.lock = false;
    this.base = { cmd: [0, 0, 0], stampNs: -Infinity, from: null };
    this.bumped = false;
    this.last = {
      planner: null, joystick: null,         // {tNs, forwarded} of the mux, per input
      monitor: null,                         // the core's last answer to a command
      state: Report.None, stateZone: 0,      // the last state message, as on the state topic
      scanPoints: null,
    };
    this.timeline = [];
  }

  setWiring(w) {
    if (!WIRINGS.includes(w)) throw new Error('unknown wiring ' + w);
    this.wiring = w;
    this.reset();
  }

  /** An emergency-stop message: the lock's value goes to the mux. */
  setLock(value) {
    this.lock = !!value;
    this.mux.onLock(0, this.lock, this.tNs);
  }

  /** The joystick holds `cmd` = [v, w] for `seconds`, as if keys were held. */
  holdJoystick(cmd, seconds) { this.script = { cmd, untilNs: this.tNs + seconds * NS }; }

  /** Puts the first box in the planner's way, about 0.9 m ahead of the robot. */
  placeBoxAhead() {
    const r = this.robot, b = this.boxes[0];
    const onCircle = Math.abs(Math.hypot(r.x, r.y) - PATROL_R) < 0.15;
    if (onCircle) {
      const a = Math.atan2(r.y, r.x) + 0.9 / PATROL_R;
      b.x = PATROL_R * Math.cos(a); b.y = PATROL_R * Math.sin(a);
    } else {
      b.x = r.x + 0.9 * Math.cos(r.th); b.y = r.y + 0.9 * Math.sin(r.th);
    }
    b.x = Math.max(WORLD.xMin + 0.2, Math.min(WORLD.xMax - 0.2, b.x));
    b.y = Math.max(WORLD.yMin + 0.2, Math.min(WORLD.yMax - 0.2, b.y));
  }

  // --- the toy lidar: ray casting against the boxes, in single precision ---
  scan() {
    const r = this.robot, s = PARAMS.monitor.sensor;
    const ox = r.x + s[0] * Math.cos(r.th) - s[1] * Math.sin(r.th);
    const oy = r.y + s[0] * Math.sin(r.th) + s[1] * Math.cos(r.th);
    const ranges = new Float32Array(BEAMS), inc = 2 * Math.PI / BEAMS;
    for (let i = 0; i < BEAMS; i++) {
      const a = r.th + i * inc, dx = Math.cos(a), dy = Math.sin(a);
      let best = Infinity;
      for (const b of this.boxes) {
        const h = b.s / 2;
        let t0 = -Infinity, t1 = Infinity, hit = true;
        for (const [o, d, c] of [[ox, dx, b.x], [oy, dy, b.y]]) {
          if (Math.abs(d) < 1e-12) { if (Math.abs(o - c) > h) hit = false; continue; }
          const a0 = (c - h - o) / d, a1 = (c + h - o) / d;
          t0 = Math.max(t0, Math.min(a0, a1)); t1 = Math.min(t1, Math.max(a0, a1));
        }
        if (hit && t1 >= t0 && t0 > 0 && t0 < best) best = t0;
      }
      ranges[i] = best;                      // no hit: +inf, as Gazebo's lidar reports it
    }
    return { stampNs: this.tNs, angleMin: 0, angleIncrement: inc, rangeMin: RANGE_MIN, rangeMax: RANGE_MAX, ranges };
  }

  // --- the toy planner: follow the circle ---
  plannerCmd() {
    const r = this.robot, ang = Math.atan2(r.y, r.x) + 0.35;
    const tx = PATROL_R * Math.cos(ang), ty = PATROL_R * Math.sin(ang);
    let err = Math.atan2(ty - r.y, tx - r.x) - r.th;
    err = Math.atan2(Math.sin(err), Math.cos(err));
    return [PATROL_V, 0, Math.max(-1.2, Math.min(1.2, 2.0 * err))];
  }

  joystickCmd() {
    if (this.script) {
      if (this.tNs < this.script.untilNs) return [this.script.cmd[0], 0, this.script.cmd[1]];
      this.script = null;
    }
    const k = this.keys;
    if (!(k.up || k.down || k.left || k.right)) return null;
    return [(k.up ? 0.2 : 0) - (k.down ? 0.15 : 0), 0, (k.left ? 1.0 : 0) - (k.right ? 1.0 : 0)];
  }

  // --- the wiring: who hears whom ---
  toBase(cmd, from) { this.base = { cmd, stampNs: this.tNs, from }; }

  toMonitor(cmd, from) {
    const out = this.monitor.onCmd(this.tNs, cmd);
    out.tNs = this.tNs; out.cmd = cmd; out.from = from;
    this.last.monitor = out;
    if (out.report !== Report.None) { this.last.state = out.report; this.last.stateZone = out.reportZone; }
    if (this.monitor.points) this.last.scanPoints = this.monitor.points;
    return out.publish;
  }

  toMux(index, name) {
    const forwarded = this.mux.onInput(index, this.tNs);
    this.last[name] = { tNs: this.tNs, forwarded };
    return forwarded;
  }

  /** One message from the planner (index 0) or the joystick (index 1). */
  deliver(index, cmd) {
    const name = PARAMS.mux.inputs[index].name;
    if (this.wiring === 'monitor-last') {
      // sources -> mux -> monitor -> base
      if (!this.toMux(index, name)) return;
      const pub = this.toMonitor(cmd, name);
      if (pub) this.toBase(pub, name);
    } else if (index === 0) {
      // planner -> monitor -> mux -> base
      const pub = this.toMonitor(cmd, name);
      if (pub && this.toMux(0, name)) this.toBase(pub, name);
    } else {
      // joystick -> mux -> base
      if (this.toMux(1, name)) this.toBase(cmd, name);
    }
  }

  /** Advances the world by one step of 10 ms. */
  tick() {
    this.step++;
    this.tNs += STEP_NS;
    if (this.step % SCAN_EVERY === 1) this.monitor.onScan(this.tNs, this.scan());
    if (this.step % CMD_EVERY === 2) this.deliver(0, this.plannerCmd());
    if (this.step % CMD_EVERY === 7) { const j = this.joystickCmd(); if (j) this.deliver(1, j); }

    // The base: its watchdog (as base_watchdog.py of the demo), then motion.
    if (this.tNs - this.base.stampNs > PARAMS.watchdog * NS) this.base = { cmd: [0, 0, 0], stampNs: this.base.stampNs, from: null };
    const r = this.robot, dt = STEP_NS / NS, [v, , w] = this.base.cmd;
    const nx = r.x + v * Math.cos(r.th) * dt, ny = r.y + v * Math.sin(r.th) * dt;
    r.th += w * dt;
    // A box does not give way and neither does the edge of the floor.
    const blocked = nx < WORLD.xMin + ROBOT_R || nx > WORLD.xMax - ROBOT_R || ny < WORLD.yMin + ROBOT_R || ny > WORLD.yMax - ROBOT_R ||
      this.boxes.some((b) => touches(nx, ny, b));
    if (!blocked) { r.x = nx; r.y = ny; }
    this.bumped = blocked && v !== 0;

    if (this.step % 10 === 0) {
      this.timeline.push(this.sample());
      if (this.timeline.length > 300) this.timeline.shift();
    }
  }

  /** Who the base last heard from, if its watchdog has not expired. */
  driver() { return this.base.from; }

  /** The monitor's last answer as one word, or 'none' if no command reached it lately. */
  monitorWord() {
    const m = this.last.monitor;
    if (!m || this.tNs - m.tNs > PARAMS.watchdog * NS) return 'none';
    switch (m.kind) {
      case Kind.Pass: return 'pass';
      case Kind.Slowdown: return 'slowdown';
      case Kind.StopZone: case Kind.StopSource: return 'stop';
      case Kind.Withheld: return 'silent';
      default: return 'none';
    }
  }

  /** The zone the last state message named, or -1. For drawing. */
  litZone() {
    if (this.monitorWord() === 'none') return -1;
    return this.last.state === Report.Stop || this.last.state === Report.Slowdown ? this.last.stateZone : -1;
  }

  sample() { return { driver: this.driver(), monitor: this.monitorWord(), lock: this.lock }; }
}

function touches(x, y, b) {
  const h = b.s / 2, dx = Math.max(Math.abs(x - b.x) - h, 0), dy = Math.max(Math.abs(y - b.y) - h, 0);
  return dx * dx + dy * dy < ROBOT_R * ROBOT_R;
}

/** Whether a box at (x, y) of side s would overlap the robot. For dragging. */
export function boxTouchesRobot(robot, x, y, s) { return touches(robot.x, robot.y, { x, y, s }); }
