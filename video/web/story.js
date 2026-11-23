// Storyboard, captions and the "who is driving" panel, shared by both videos.
// Everything is a function of the video time t (seconds), so that a frame can
// be drawn in any order. window.DATA is written by build_data.py.

const LENGTH = 59.0;
const CARD = { titleEnd: 4.6, endStart: 51.6 };
const COLORS = { planner: '#5aa9ff', teleop: '#ffb42e', estop: '#ff5468', lean: '#42dba0', off: '#566174' };

const BEATS = [
  { from: 5.2, to: 12.6,
    title: 'Many sources. One robot.',
    sub: '<span class="mono">twist_mux</span> forwards exactly one of them, chosen by priority.' },
  { from: 13.2, to: 20.4, kicker: 'What you are watching', key: 'lean',
    title: 'This is its Lean implementation, running live.',
    sub: 'Every forward-or-drop decision is made by a Lean 4 function. Its rules are machine-checked theorems.' },
  { from: 21.0, to: 30.1,
    title: 'The operator takes over.',
    sub: 'Higher priority wins. Planner commands are dropped.',
    thm: 'run_not_forward_of_higher_live' },
  { from: 30.5, to: 38.5,
    title: 'The operator lets go.',
    sub: 'Its 0.5 s timeout expires, and the planner drives again.',
    thm: 'forward_of_others_expired' },
  { from: 39.0, to: 46.7,
    title: 'Emergency stop.',
    sub: 'The lock outranks every input. Nothing is forwarded.',
    thm: 'not_forward_of_top_lock' },
  { from: 47.1, to: 51.0,
    title: 'Lock released.',
    sub: 'The planner resumes.' },
];

const clamp = (x, a = 0, b = 1) => Math.min(b, Math.max(a, x));
const smooth = (x) => { x = clamp(x); return x * x * (3 - 2 * x); };
const ease = (x) => { x = clamp(x); return 1 - Math.pow(1 - x, 3); };
// 0 outside [a, b], 1 inside, with ramps of length d.
const window01 = (t, a, b, d = 0.45) => smooth((t - a) / d) * smooth((b - t) / d);

const CHECK = (size, color = COLORS.lean) =>
  `<svg width="${size}" height="${size}" viewBox="0 0 24 24"><circle cx="12" cy="12" r="11" fill="${color}"/>` +
  `<path d="M7 12.4l3.4 3.4L17.2 9" fill="none" stroke="#06231a" stroke-width="2.6" stroke-linecap="round" stroke-linejoin="round"/></svg>`;

// ---- What the mux was doing at time t, from the recorded messages. ----

function lastAtOrBefore(times, t) {       // index of the last entry <= t, or -1
  let lo = 0, hi = times.length;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (times[mid] <= t) lo = mid + 1; else hi = mid; }
  return lo - 1;
}

function muxState(t) {
  const D = window.DATA;
  const recent = (times, within) => { const i = lastAtOrBefore(times, t); return i >= 0 && t - times[i] <= within; };
  const i = lastAtOrBefore(D.outT, t);
  const forwarded = i >= 0 && t - D.outT[i] <= 0.3 ? D.outSrc[i] : null;
  const k = lastAtOrBefore(D.lockT, t);
  return {
    forwarded,                                    // 'planner' | 'teleop' | null
    locked: k >= 0 ? D.lockV[k] : false,
    publishing: { planner: recent(D.inT.planner, 0.3), teleop: recent(D.inT.teleop, 0.3) },
  };
}

function driverColor(s) {
  if (s.forwarded) return COLORS[s.forwarded];
  return s.locked ? COLORS.estop : COLORS.off;
}

// ---- DOM ----

function buildStory(options = {}) {
  const beats = BEATS.map((b) => Object.assign({}, b, (options.beats || {})[b.key] || {}));
  const stage = document.getElementById('stage');
  stage.insertAdjacentHTML('beforeend', `
    <div id="vignette"></div>
    <div id="shade"></div>
    <div id="brand"><i></i>ROSlean <span>· twist_mux</span></div>
    <div id="caption"></div>
    <div id="hud"><svg viewBox="0 0 680 268" width="680" height="268"></svg></div>
    <div class="card" id="title">
      <div class="kicker"><i></i>ROSlean</div>
      <h1>A <em>verified</em><br>twist_mux</h1>
      <div class="lede">Who gets to drive the robot? Decided by a ROS 2 node whose logic is written, and proved, in Lean 4.</div>
    </div>
    <div class="card" id="end">
      <h1>What was proved</h1>
      <ul>
        <li>${CHECK(34)}<b>Priority</b><span>The highest-priority live input is the one forwarded.</span></li>
        <li>${CHECK(34)}<b>Timeouts</b><span>An input that has gone silent never blocks another.</span></li>
        <li>${CHECK(34)}<b>Locks</b><span>An engaged e-stop blocks every input below it.</span></li>
      </ul>
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
    if (thm) thm.style.opacity = smooth((t - beat.from - 0.9) / 0.5);
  }

  // The panel. Rows: e-stop lock, operator, planner; then the mux and the robot.
  const ROWS = [
    { id: 'estop', name: 'E-stop lock', prio: 255, y: 78 },
    { id: 'teleop', name: 'Operator', prio: 100, y: 142 },
    { id: 'planner', name: 'Planner', prio: 10, y: 206 },
  ];
  const MUX = { x: 366, y: 106, w: 176, h: 72 };

  function panel(t, s) {
    const midY = MUX.y + MUX.h / 2;
    let h = `<text class="head" x="28" y="38">WHO IS DRIVING</text>` +
            `<text class="head" x="282" y="38" text-anchor="end">PRIORITY</text>`;
    for (const r of ROWS) {
      let color, status, wire = 'idle';
      if (r.id === 'estop') {
        color = s.locked ? COLORS.estop : COLORS.off;
        status = s.locked ? 'ENGAGED' : 'RELEASED';
        wire = s.locked ? 'lock' : 'idle';
      } else if (s.forwarded === r.id) {
        color = COLORS[r.id]; status = 'FORWARDED'; wire = 'live';
      } else if (s.publishing[r.id]) {
        color = COLORS[r.id]; status = s.locked ? 'BLOCKED' : 'DROPPED'; wire = 'cut';
      } else {
        color = COLORS.off; status = 'SILENT';
      }
      const dim = wire === 'idle' ? 0.55 : 1;
      h += `<g opacity="${dim}"><circle cx="34" cy="${r.y - 7}" r="7" fill="${color}"/>` +
           `<text class="name" x="54" y="${r.y}">${r.name}</text>` +
           `<text class="status" x="54" y="${r.y + 21}" fill="${color}" style="fill:${color}">${status}</text>` +
           `<text class="prio" x="282" y="${r.y}" text-anchor="end">${r.prio}</text></g>`;
      // Wire from the row to the mux.
      const x0 = 298, y0 = r.y - 7, x1 = MUX.x, y1 = r.id === 'estop' ? MUX.y + 16 : (r.id === 'teleop' ? midY : MUX.y + MUX.h - 16);
      const path = `M${x0} ${y0} C${x0 + 36} ${y0} ${x1 - 36} ${y1} ${x1} ${y1}`;
      if (wire === 'live') {
        h += `<path d="${path}" fill="none" stroke="${color}" stroke-width="3.5" stroke-linecap="round" opacity="0.35"/>` +
             `<path d="${path}" fill="none" stroke="${color}" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="10 12" stroke-dashoffset="${-t * 70}"/>`;
      } else if (wire === 'cut') {
        h += `<path d="${path}" fill="none" stroke="${color}" stroke-width="2.5" stroke-linecap="round" opacity="0.45" stroke-dasharray="3 7"/>` +
             `<g transform="translate(${x1 - 14} ${y1})" stroke="${color}" stroke-width="3" stroke-linecap="round"><path d="M-6 -6L6 6M6 -6L-6 6"/></g>`;
      } else if (wire === 'lock') {
        h += `<path d="${path}" fill="none" stroke="${color}" stroke-width="3.5" stroke-linecap="round"/>`;
      } else {
        h += `<path d="${path}" fill="none" stroke="#3a4353" stroke-width="2" stroke-linecap="round"/>`;
      }
    }
    const out = s.forwarded ? COLORS[s.forwarded] : null;
    const border = s.locked ? COLORS.estop : COLORS.lean;
    h += `<rect x="${MUX.x}" y="${MUX.y}" width="${MUX.w}" height="${MUX.h}" rx="12" fill="rgba(66,219,160,0.07)" stroke="${border}" stroke-width="2"/>` +
         `<text class="mux" x="${MUX.x + MUX.w / 2}" y="${MUX.y + 31}" text-anchor="middle">twist_mux</text>` +
         `<g transform="translate(${MUX.x + 30} ${MUX.y + 42})">${CHECK(16)}</g>` +
         `<text class="lean" x="${MUX.x + 52}" y="${MUX.y + 55}">Lean 4 · proved</text>`;
    const ox0 = MUX.x + MUX.w, ox1 = 600;
    if (out) {
      h += `<path d="M${ox0} ${midY}H${ox1}" stroke="${out}" stroke-width="3.5" opacity="0.35"/>` +
           `<path d="M${ox0} ${midY}H${ox1}" stroke="${out}" stroke-width="3.5" stroke-linecap="round" stroke-dasharray="10 12" stroke-dashoffset="${-t * 70}"/>`;
    } else {
      h += `<path d="M${ox0} ${midY}H${ox1}" stroke="#3a4353" stroke-width="2"/>`;
    }
    // The robot: a disc with two wheels.
    const rc = out || '#566174';
    h += `<g transform="translate(624 ${midY})"><rect x="-19" y="-24" width="38" height="8" rx="3" fill="${rc}"/>` +
         `<rect x="-19" y="16" width="38" height="8" rx="3" fill="${rc}"/>` +
         `<circle r="19" fill="#0d121c" stroke="${rc}" stroke-width="3"/><path d="M-3 -7L8 0L-3 7Z" fill="${rc}"/></g>` +
         `<text class="out" x="624" y="${midY + 50}" text-anchor="middle">${out ? 'robot' : 'no command'}</text>`;
    hud.innerHTML = h;
  }

  return function update(t) {
    const s = muxState(t);
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
    return { state: s, body, color: driverColor(s) };
  };
}
