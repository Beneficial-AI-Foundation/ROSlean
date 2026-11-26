// The page: drawing, the panel, buttons, keys and dragging. The world is
// sim.js; the two nodes are shim.js and roslean_core.wasm. Nothing here
// decides anything for a node.

import { loadCore, Kind, Report } from './shim.js';
import { World, PARAMS, WORLD, boxTouchesRobot } from './sim.js';

const $ = (id) => document.getElementById(id);
const stage = $('stage'), cv = $('floor'), g = cv.getContext('2d');
const strip = $('strip'), sg = strip.getContext('2d');
const LW = 900, LH = 640, S = 150, CX = LW / 2, CY = LH / 2;      // logical canvas: 150 px per metre
const sx = (x) => CX + x * S, sy = (y) => CY - y * S;

let colours = {};
function readColours() {
  const st = getComputedStyle(document.documentElement);
  for (const n of ['line', 'muted', 'ink', 'surface', 'bg', 'accent', 'proved', 'pass', 'slow', 'stop', 'silent', 'floor', 'robot', 'box', 'lidar']) {
    colours[n] = st.getPropertyValue('--' + n).trim();
  }
}
readColours();
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { readColours(); draw(); });

// The canvases keep their logical size; the backing store follows the screen.
function fit(canvas, ctx, lw, lh) {
  const dpr = Math.min(window.devicePixelRatio || 1, 3);
  const w = canvas.clientWidth || lw, h = canvas.clientHeight || w * lh / lw;
  const bw = Math.round(w * dpr), bh = Math.round(h * dpr);
  if (canvas.width !== bw || canvas.height !== bh) { canvas.width = bw; canvas.height = bh; }
  ctx.setTransform(bw / lw, 0, 0, bh / lh, 0, 0);
}

let world = null, paused = matchMedia('(prefers-reduced-motion: reduce)').matches;

const NS = 1e9;
const fmt = (c) => c ? `${c[0].toFixed(2)} m/s, ${c[2].toFixed(2)} rad/s` : 'nothing';
const STATE_TEXT = { pass: 'PASS', slowdown: 'SLOWDOWN', stop: 'STOP', silent: 'SILENT', none: 'no command' };

function describe() {
  const w = world, m = w.last.monitor, word = w.monitorWord();
  const recent = (x) => x && w.tNs - x.tNs <= 0.25 * NS;
  const input = (x) => !recent(x) ? 'quiet' : x.forwarded ? 'forwarded' : 'dropped';
  let mon = STATE_TEXT[word];
  if (word === 'slowdown') mon += ' ×' + m.ratio.toFixed(2);
  if (word === 'stop' && m.kind === Kind.StopSource) mon += ' (no valid scan)';
  const zoneName = (k) => (PARAMS.monitor.zones[k] || { name: '?' }).name;
  const state = { [Report.None]: '–', [Report.Normal]: 'normal', [Report.Stop]: zoneName(w.last.stateZone) + ': stop',
    [Report.Slowdown]: zoneName(w.last.stateZone) + ': slowdown', [Report.Source]: 'invalid source: stop' }[w.last.state];
  return {
    who: w.driver() || 'nobody', planner: input(w.last.planner), joystick: input(w.last.joystick),
    lock: w.lock ? 'ENGAGED' : 'released', word, mon,
    cmdIn: word === 'none' ? '–' : fmt(m.cmd), cmdOut: word === 'none' ? '–' : fmt(m.publish), state,
    base: fmt(w.base.cmd), dog: w.base.from ? 'fed' : 'holding the robot', bump: w.bumped ? 'touching' : 'none',
  };
}

function setText(id, text) { const e = $(id); if (e.textContent !== text) e.textContent = text; }

let liveText = '', liveAt = 0;
function panel() {
  const d = describe();
  setText('p-who', d.who === 'nobody' ? 'nobody' : d.who);
  setText('p-plan', d.planner); setText('p-joy', d.joystick); setText('p-lock', d.lock);
  setText('p-mon', d.mon); $('p-mon').className = 'state s-' + d.word;
  setText('p-in', d.cmdIn); setText('p-out', d.cmdOut); setText('p-state', d.state);
  setText('p-base', d.base); setText('p-dog', d.dog); setText('p-bump', d.bump);
  // One line for screen readers, changed at most every 1.5 s.
  const line = `${d.who === 'nobody' ? 'Nobody drives' : 'The ' + d.who + ' drives'}. Monitor: ${d.mon.toLowerCase()}.` +
    (world.lock ? ' Emergency stop engaged.' : '') + (world.bumped ? ' The robot touches a box.' : '');
  const now = performance.now();
  if (line !== liveText && now - liveAt > 1500) { liveText = line; liveAt = now; $('live').textContent = line; }
}

function draw() {
  if (!world) return;
  fit(cv, g, LW, LH);
  const w = world, r = w.robot, c = colours;
  g.clearRect(0, 0, LW, LH);
  // the patrol circle
  g.strokeStyle = c.line; g.lineWidth = 1.5; g.setLineDash([6, 8]);
  g.beginPath(); g.arc(CX, CY, WORLD.patrolRadius * S, 0, 2 * Math.PI); g.stroke(); g.setLineDash([]);
  // boxes
  g.fillStyle = c.box;
  for (const b of w.boxes) g.fillRect(sx(b.x - b.s / 2), sy(b.y + b.s / 2), b.s * S, b.s * S);
  // zones, in the base frame; the one the last state message named is filled
  const toWorld = (p) => [r.x + p[0] * Math.cos(r.th) - p[1] * Math.sin(r.th), r.y + p[0] * Math.sin(r.th) + p[1] * Math.cos(r.th)];
  const lit = w.litZone(), silent = w.monitorWord() === 'silent';
  PARAMS.monitor.zones.forEach((z, k) => {
    const col = z.action === 'stop' ? c.stop : c.slow;
    g.beginPath();
    z.points.forEach((p, i) => { const q = toWorld(p); if (i) g.lineTo(sx(q[0]), sy(q[1])); else g.moveTo(sx(q[0]), sy(q[1])); });
    g.closePath(); g.strokeStyle = col; g.lineWidth = 2; g.stroke();
    if (k === lit) { g.globalAlpha = silent ? 0.12 : 0.28; g.fillStyle = col; g.fill(); g.globalAlpha = 1; }
  });
  // the points handed to the core with the last command (base frame)
  const pts = w.last.scanPoints;
  if (pts) {
    g.fillStyle = c.lidar;
    for (let i = 0; i < pts.length; i += 2) {
      if (!Number.isFinite(pts[i]) || !Number.isFinite(pts[i + 1])) continue;
      const q = toWorld([pts[i], pts[i + 1]]);
      g.fillRect(sx(q[0]) - 1.5, sy(q[1]) - 1.5, 3, 3);
    }
  }
  // the robot
  g.fillStyle = c.robot; g.beginPath(); g.arc(sx(r.x), sy(r.y), WORLD.robotRadius * S, 0, 2 * Math.PI); g.fill();
  g.strokeStyle = c.floor; g.lineWidth = 3; g.beginPath(); g.moveTo(sx(r.x), sy(r.y));
  g.lineTo(sx(r.x + 0.1 * Math.cos(r.th)), sy(r.y + 0.1 * Math.sin(r.th))); g.stroke();
  if (w.lock) {
    g.strokeStyle = c.stop; g.lineWidth = 3; g.beginPath(); g.arc(sx(r.x), sy(r.y), WORLD.robotRadius * S + 5, 0, 2 * Math.PI); g.stroke();
  }
  panel();
  drawStrip();
}

function drawStrip() {
  fit(strip, sg, 900, 58);
  const c = colours, t = world.timeline, n = 300, cw = 900 / n;
  sg.clearRect(0, 0, 900, 58);
  const who = { planner: c.proved, joystick: c.accent };
  const mon = { pass: c.pass, slowdown: c.slow, stop: c.stop, silent: c.silent };
  for (let i = 0; i < t.length; i++) {
    const x = (n - t.length + i) * cw, s = t[i];
    if (who[s.driver]) { sg.fillStyle = who[s.driver]; sg.fillRect(x, 4, cw + 0.5, 22); }
    if (mon[s.monitor]) { sg.fillStyle = mon[s.monitor]; sg.fillRect(x, 32, cw + 0.5, 22); }
    if (s.lock) { sg.fillStyle = c.stop; sg.fillRect(x, 0, cw + 0.5, 3); }
  }
  sg.strokeStyle = c.line; sg.lineWidth = 1;
  for (let k = 1; k < 6; k++) { sg.beginPath(); sg.moveTo(k * 150, 0); sg.lineTo(k * 150, 58); sg.stroke(); }   // every 5 s
}

// --- the loop: world steps of 10 ms, at most 5 per frame ---
let prev = null, acc = 0;
function frame(ts) {
  if (prev !== null && !paused) {
    acc += Math.min(0.1, (ts - prev) / 1000);
    let n = 0;
    while (acc >= 0.01 && n < 5) { world.tick(); acc -= 0.01; n++; }
    if (n === 5) acc = 0;
  }
  prev = ts;
  draw();
  requestAnimationFrame(frame);
}

// --- controls ---
function setPaused(p) { paused = p; $('pause').setAttribute('aria-pressed', String(p)); $('pause').textContent = p ? 'Resume' : 'Pause'; }
function syncEstop() {
  $('estop').setAttribute('aria-pressed', String(world.lock));
  $('estop').textContent = world.lock ? 'Release the emergency stop' : 'Emergency stop';
}
function releaseKeys() { for (const k of Object.keys(world.keys)) world.keys[k] = false; }

function wire() {
  $('g-box').addEventListener('click', () => { world.placeBoxAhead(); setPaused(false); });
  $('g-joy').addEventListener('click', () => {
    // Two seconds of turning right while driving on, then the keys are the visitor's.
    world.holdJoystick([0.12, -1.0], 2);
    setPaused(false);
    stage.focus();
  });
  $('estop').addEventListener('click', () => { world.setLock(!world.lock); syncEstop(); setPaused(false); });
  $('pause').addEventListener('click', () => setPaused(!paused));
  $('reset').addEventListener('click', () => { world.reset(); syncEstop(); });
  for (const radio of document.querySelectorAll('input[name="wiring"]')) {
    radio.addEventListener('change', () => {
      if (!radio.checked) return;
      world.setWiring(radio.value);
      syncEstop();
      $('wire-a').toggleAttribute('hidden', radio.value !== 'monitor-last');
      $('wire-b').toggleAttribute('hidden', radio.value !== 'joystick-bypass');
    });
  }

  // Arrow keys drive only while the floor has focus, so the page still scrolls.
  const KEY = { ArrowUp: 'up', ArrowDown: 'down', ArrowLeft: 'left', ArrowRight: 'right' };
  stage.addEventListener('keydown', (e) => { if (KEY[e.key]) { world.keys[KEY[e.key]] = true; setPaused(false); e.preventDefault(); } });
  stage.addEventListener('keyup', (e) => { if (KEY[e.key]) world.keys[KEY[e.key]] = false; });
  stage.addEventListener('blur', releaseKeys);
  addEventListener('blur', releaseKeys);
  const hint = $('hint').textContent;
  stage.addEventListener('focus', () => { $('hint').textContent = 'arrow keys drive'; });
  stage.addEventListener('blur', () => { $('hint').textContent = hint; });

  // The pad: held while pressed, with a pointer or with Enter or Space.
  for (const b of document.querySelectorAll('[data-k]')) {
    const k = b.getAttribute('data-k');
    const on = (e) => { world.keys[k] = true; setPaused(false); if (e.pointerId !== undefined) b.setPointerCapture(e.pointerId); };
    const off = () => { world.keys[k] = false; };
    b.addEventListener('pointerdown', on);
    for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture', 'blur']) b.addEventListener(ev, off);
    b.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { on(e); e.preventDefault(); } });
    b.addEventListener('keyup', (e) => { if (e.key === 'Enter' || e.key === ' ') off(); });
    b.addEventListener('contextmenu', (e) => e.preventDefault());
  }

  // Dragging a box. A box cannot be put onto the robot.
  let drag = null;
  const at = (e) => { const r = cv.getBoundingClientRect(); return [((e.clientX - r.left) * LW / r.width - CX) / S, -((e.clientY - r.top) * LH / r.height - CY) / S]; };
  cv.addEventListener('pointerdown', (e) => {
    const p = at(e), slack = e.pointerType === 'touch' ? 0.2 : 0.08;
    drag = world.boxes.find((b) => Math.abs(p[0] - b.x) < b.s / 2 + slack && Math.abs(p[1] - b.y) < b.s / 2 + slack) || null;
    if (drag) { cv.setPointerCapture(e.pointerId); cv.style.cursor = 'grabbing'; }
  });
  cv.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const p = at(e), h = drag.s / 2;
    const x = Math.max(WORLD.xMin + h, Math.min(WORLD.xMax - h, p[0])), y = Math.max(WORLD.yMin + h, Math.min(WORLD.yMax - h, p[1]));
    if (!boxTouchesRobot(world.robot, x, y, drag.s + 0.04)) { drag.x = x; drag.y = y; }
  });
  for (const ev of ['pointerup', 'pointercancel']) cv.addEventListener(ev, () => { drag = null; cv.style.cursor = ''; });
  addEventListener('resize', draw);
}

const t0 = performance.now();
try {
  const M = await loadCore();
  world = new World(M);
  wire();
  setPaused(paused);
  $('loading').remove();
  window.__pg = { world, loadMs: performance.now() - t0, setPaused, describe, draw };   // for the tests
  requestAnimationFrame(frame);
} catch (err) {
  $('loading').textContent = 'The Lean core could not be loaded: ' + (err && err.message ? err.message : err) +
    '. This page needs a browser with WebAssembly, including WebAssembly exceptions.';
  window.__pg = { error: String(err) };
  throw err;
}
