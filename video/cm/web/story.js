// Storyboard, captions and the panel of the collision monitor video, shared
// by both pages. Everything is a function of the video time t (seconds), so
// that a frame can be drawn in any order. window.DATA is written by
// ../build_data.py; the times of the story are those of the recorded run
// (DATA.marks), only the wording is here.

const M = window.DATA.marks;      // drop, slow, stop, silent, removed, pass: video times
const CARD = { titleEnd: 4.6, endStart: window.DATA.endStart };
const LENGTH = window.DATA.length;
const COLORS = { pass: '#5aa9ff', slow: '#ffb42e', stop: '#ff5468', silent: '#aeb6c4',
                 lean: '#42dba0', off: '#566174', planner: '#5aa9ff' };

const BEATS = [
  { from: 5.2, to: 12.0,
    title: 'Two zones around the robot.',
    sub: '<span class="mono">collision_monitor</span> checks every velocity command against what the lidar sees ' +
         'in two zones: a <span class="slow">slowdown zone</span> and a <span class="stop">stop zone</span>.' },
  { from: 12.6, to: M.drop - 0.5, kicker: 'What you are watching', key: 'lean',
    title: 'This is its Lean implementation, running live.',
    sub: 'Every pass, slow-down or stop decision is made by a Lean 4 function. Its rules are machine-checked theorems.' },
  { from: M.drop, to: M.slow - 0.3,
    title: 'A box drops 1.2 m ahead.',
    sub: 'The monitor learns of it only through the lidar.' },
  { from: M.slow, to: M.stop - 0.3,
    title: 'The box is in the slowdown zone.',
    sub: 'The command is published times 0.40.',
    thm: 'run_slowdown_bound' },
  { from: M.stop, to: M.stop + 2.9,
    title: 'The box is in the stop zone.',
    sub: 'Zero is published instead of the command.',
    thm: 'run_stop' },
  { from: M.stop + 3.2, to: M.removed - 0.3,
    title: 'Then the monitor goes silent.',
    sub: 'After 1 s of zeros it publishes nothing, as Nav2’s monitor does. The last command the base got is a zero.',
    thm: 'run_withheld_last_published_zero' },
  { from: M.removed, to: M.removed + 5.6,
    title: 'The box is removed.',
    sub: 'No zone is occupied. The command passes unchanged.',
    thm: 'run_pass_through' },
  { from: M.removed + 6.0, to: CARD.endStart - 0.6, kicker: 'What is proved',
    title: 'Theorems about commands.',
    sub: 'They say which action the node takes for the points the lidar reports. They do not say that the robot cannot hit anything.' },
];

const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const smooth = (x) => { x = clamp(x); return x * x * (3 - 2 * x); };
const ease = (x) => { x = clamp(x); return 1 - Math.pow(1 - x, 3); };
// 0 outside [a, b], 1 inside, with ramps of length d.
const window01 = (t, a, b, d = 0.45) => smooth((t - a) / d) * smooth((b - t) / d);

const CHECK = (size, color = COLORS.lean) =>
  `<svg width="${size}" height="${size}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="11" fill="${color}"/>` +
  `<path d="M7 12.4l3.4 3.4L17.2 9" fill="none" stroke="#06231a" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/></svg>`;

// ---- What the monitor was doing at time t, from the recorded messages. ----

function lastAtOrBefore(times, t) {       // index of the last entry <= t, or -1
  let lo = 0, hi = times.length;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (times[mid] <= t) lo = mid + 1; else hi = mid; }
  return lo - 1;
}

// kind: what the newest output message was ('pass', 'slow', 'stop'; a zero
// command passed on counts as 'pass'); 'silent' if commands arrive and
// nothing has been published for 0.3 s; 'idle' if no command arrives.
// report: the newest message on the monitor's state topic.
function monitorState(t) {
  const D = window.DATA;
  const i = lastAtOrBefore(D.outT, t), j = lastAtOrBefore(D.inT, t), k = lastAtOrBefore(D.stateT, t);
  const commanded = j >= 0 && t - D.inT[j] <= 0.3;
  const published = i >= 0 && t - D.outT[i] <= 0.3;
  const n = lastAtOrBefore(D.scanT, t);
  return {
    kind: published ? (D.outKind[i] === 'zero' ? 'pass' : D.outKind[i]) : (commanded ? 'silent' : 'idle'),
    ratio: published ? D.outRatio[i] : null,
    cmd: commanded ? D.inV[j] : null,                 // [m/s, rad/s]
    pub: published ? D.outPub[i] : null,
    report: k >= 0 ? { action: D.stateAction[k], zone: D.stateZone[k] } : null,
    nearest: n >= 0 && t - D.scanT[n] <= 0.5 ? D.nearest[n] : null,
  };
}

const stateColor = (s) => COLORS[s.kind] || COLORS.off;
function stateLabel(s) {
  if (s.kind === 'slow') return 'SLOWDOWN ×' + s.ratio.toFixed(2);
  return { pass: 'PASS', stop: 'STOP', silent: 'SILENT', idle: 'NO COMMAND' }[s.kind];
}

// ---- DOM ----

function buildStory(options = {}) {
  const beats = BEATS.map((b) => Object.assign({}, b, (options.beats || {})[b.key] || {}));
  const stage = document.getElementById('stage');
  stage.insertAdjacentHTML('beforeend', `
    <div id="vignette"></div>
    <div id="shade"></div>
    <div id="brand"><i></i>ROSlean <span>· collision_monitor</span></div>
    <div id="caption"></div>
    <div id="hud"><svg viewBox="0 0 680 268" width="680" height="268"></svg></div>
    <div class="card" id="title">
      <div class="kicker"><i></i>ROSlean</div>
      <h1>A <em>verified</em><br>collision monitor</h1>
      <div class="lede">Slow down, or stop? The stop and slowdown core of Nav2’s collision monitor, as a ROS 2 node whose logic is written, and proved, in Lean 4.</div>
    </div>
    <div class="card" id="end">
      <h1>What was proved</h1>
      <ul>
        <li>${CHECK(34)}<b>Stop</b><span>A triggered stop zone gives a stop: zero is published, then nothing.</span></li>
        <li>${CHECK(34)}<b>Slowdown</b><span>Only slowdown zones triggered: a slowdown by the smallest of their ratios.</span></li>
        <li>${CHECK(34)}<b>No amplification</b><span>Every published action is pass, stop, or slowdown by a ratio below 1.</span></li>
      </ul>
      <div class="note">These are statements about the commands the node publishes, not about the robot not hitting things. Computing the scan’s points and multiplying by the ratio are C++ and not proved.</div>
      <div class="foot"><span class="mono">github.com/Beneficial-AI-Foundation/ROSlean</span>
        <span>${options.footer || ''}</span></div>
    </div>
    <div id="black"></div>`);
  const el = (id) => document.getElementById(id);
  const hud = el('hud').querySelector('svg');

  function caption(t) {
    const c = el('caption');
    const beat = beats.find((b) => t >= b.from - 0.01 && t <= b.to + 0.01);
    if (!beat) { c.style.opacity = 0; return; }
    if (c.dataset.key !== beat.title) {
      c.dataset.key = beat.title;
      c.innerHTML = (beat.kicker ? `<div class="kicker">${beat.kicker}</div>` : '') +
        `<h2>${beat.title}</h2><p>${beat.sub}</p>` +
        (beat.thm ? `<div class="thm">${CHECK(24)}<b>Theorem</b><span class="mono">${beat.thm}</span></div>` : '');
    }
    const a = window01(t, beat.from, beat.to, 0.4);
    c.style.opacity = a;
    c.style.transform = `translateY(${(1 - ease((t - beat.from) / 0.6)) * 18}px)`;
    const thm = c.querySelector('.thm');
    if (thm) thm.style.opacity = smooth((t - beat.from - 0.7) / 0.5);
  }

  // The strip at the bottom of the panel: the state at every twentieth of a
  // second of the run, as runs of one colour.
  const STRIP = { x: 28, y: 204, w: 624, h: 14, from: CARD.titleEnd, to: CARD.endStart };
  const runs = [];
  for (let t = STRIP.from; t < STRIP.to; t += 0.05) {
    const kind = monitorState(t).kind, last = runs[runs.length - 1];
    if (last && last.kind === kind) last.to = t + 0.05; else runs.push({ kind, from: t, to: t + 0.05 });
  }
  const sx = (t) => STRIP.x + STRIP.w * clamp((t - STRIP.from) / (STRIP.to - STRIP.from));

  const fmt = (v, unit) => (Math.abs(v) < 0.005 ? '0.00' : v.toFixed(2)) + ' ' + unit;
  const NODE = { x: 226, y: 62, w: 226, h: 72 };

  function panel(t, s) {
    const color = stateColor(s), midY = NODE.y + NODE.h / 2;
    let h = `<text class="head" x="28" y="38">WHAT THE MONITOR PUBLISHES</text>` +
            `<text class="head" x="652" y="38" text-anchor="end">NEAREST LIDAR POINT AHEAD ` +
            `<tspan class="small" style="fill:#f4f6fa" dx="6">${s.nearest == null ? '—' : s.nearest.toFixed(2) + ' m'}</tspan></text>`;
    // Command in.
    h += `<text class="label" x="28" y="76">COMMAND IN</text>` +
         (s.cmd ? `<text class="value" x="28" y="107">${fmt(s.cmd[0], 'm/s')}</text>` +
                  `<text class="small" x="28" y="129">${fmt(s.cmd[1], 'rad/s')}</text>`
                : `<text class="value" x="28" y="107" style="fill:${COLORS.off}">none</text>`);
    const live = (x0, x1, c) =>
      `<path d="M${x0} ${midY}H${x1}" stroke="${c}" stroke-width="3.5" opacity="0.35"/>` +
      `<path d="M${x0} ${midY}H${x1}" stroke="${c}" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="10 12" stroke-dashoffset="${-t * 70}"/>`;
    const dead = (x0, x1) => `<path d="M${x0} ${midY}H${x1}" stroke="#3a4353" stroke-width="2"/>`;
    h += s.cmd ? live(178, NODE.x, COLORS.pass) : dead(178, NODE.x);
    // The node.
    h += `<rect x="${NODE.x}" y="${NODE.y}" width="${NODE.w}" height="${NODE.h}" rx="12" fill="rgba(66,219,160,0.07)" stroke="${COLORS.lean}" stroke-width="2"/>` +
         `<text class="node" x="${NODE.x + NODE.w / 2}" y="${NODE.y + 31}" text-anchor="middle">collision_monitor</text>` +
         `<g transform="translate(${NODE.x + 28} ${NODE.y + 42})">${CHECK(16)}</g>` +
         `<text class="lean" x="${NODE.x + 50}" y="${NODE.y + 55}">Lean 4 · proved in part</text>`;
    // What it publishes.
    const x0 = NODE.x + NODE.w, x1 = 506;
    h += s.pub ? live(x0, x1, color)
      : dead(x0, x1) + (s.kind === 'silent'
        ? `<g transform="translate(${(x0 + x1) / 2} ${midY})" stroke="${color}" stroke-width="3" stroke-linecap="round"><path d="M-6 -6L6 6M6 -6L-6 6"/></g>` : '');
    h += `<text class="label" x="520" y="76">PUBLISHED</text>` +
         (s.pub ? `<text class="value" x="520" y="107" style="fill:${color}">${fmt(s.pub[0], 'm/s')}</text>` +
                  `<text class="small" x="520" y="129">${fmt(s.pub[1], 'rad/s')}</text>`
                : `<text class="value" x="520" y="107" style="fill:${color}">nothing</text>`);
    // The state, and what the monitor's own state topic last said.
    const what = { stop: 'zero published', silent: 'nothing published', pass: 'command unchanged',
                   slow: '', idle: '' }[s.kind];
    const action = ['do nothing', 'STOP', 'SLOWDOWN'];
    h += `<circle cx="36" cy="170" r="8" fill="${color}"/>` +
         `<text class="state" x="56" y="178" style="fill:${color}">${stateLabel(s)}<tspan dx="12">${what}</tspan></text>` +
         `<text class="small" x="652" y="177" text-anchor="end">state topic: ` +
         (s.report ? action[s.report.action] + (s.report.zone ? ` “${s.report.zone}”` : '') : '—') + `</text>`;
    // The strip, up to now.
    h += `<rect x="${STRIP.x}" y="${STRIP.y}" width="${STRIP.w}" height="${STRIP.h}" rx="4" fill="#1a2130"/>`;
    for (const r of runs) {
      if (r.from > t) break;
      const a = sx(r.from), b = sx(Math.min(r.to, t));
      h += `<rect x="${a}" y="${STRIP.y}" width="${Math.max(0.6, b - a)}" height="${STRIP.h}" fill="${COLORS[r.kind] || COLORS.off}"` +
           (r.kind === 'silent' ? ' opacity="0.55"' : '') + `/>`;
    }
    h += `<rect x="${sx(t) - 1.5}" y="${STRIP.y - 4}" width="3" height="${STRIP.h + 8}" rx="1.5" fill="#f4f6fa"/>`;
    let lx = STRIP.x;
    for (const [kind, name] of [['pass', 'PASS'], ['slow', 'SLOWDOWN'], ['stop', 'STOP'], ['silent', 'SILENT']]) {
      h += `<rect x="${lx}" y="238" width="10" height="10" rx="2" fill="${COLORS[kind]}"${kind === 'silent' ? ' opacity="0.55"' : ''}/>` +
           `<text class="legend" x="${lx + 16}" y="247.5">${name}</text>`;
      lx += 34 + name.length * 9.4;
    }
    h += `<text class="legend" x="652" y="247.5" text-anchor="end">THIS RUN, SO FAR</text>`;
    hud.innerHTML = h;
  }

  return function update(t) {
    const s = monitorState(t);
    const title = window01(t, -1, CARD.titleEnd, 0.6);
    const end = smooth((t - CARD.endStart) / 0.7);
    const body = smooth((t - CARD.titleEnd - 0.2) / 0.6) * (1 - smooth((t - CARD.endStart + 0.5) / 0.5));
    el('title').style.opacity = title;
    el('title').style.transform = `translateY(${(1 - ease(t / 0.9)) * 22}px)`;
    el('end').style.opacity = end;
    el('end').style.transform = `translateY(${(1 - ease((t - CARD.endStart) / 0.9)) * 22}px)`;
    el('shade').style.opacity = (options.shade || 0.74) * Math.max(title, end);
    el('hud').style.opacity = body;
    el('brand').style.opacity = body;
    el('vignette').style.opacity = body;
    caption(t);
    panel(t, s);
    // Fade from and to black.
    el('black').style.opacity = Math.max(1 - smooth(t / 0.5), smooth((t - (LENGTH - 0.6)) / 0.5));
    return { state: s, body, color: stateColor(s) };
  };
}
