// Drives the page in headless Chrome over the DevTools protocol and checks
// it: no console errors, the scripted scenario on the page's own module
// instances, keyboard driving only with focus, no horizontal scroll at
// 360 px, page weight and load time. Writes screenshots.
//
//   node chrome.mjs <base URL of site/, e.g. http://127.0.0.1:8765/> <output dir>
//
// Needs node 22 or later (global WebSocket) and Chrome; CHROME names the
// binary (default: the macOS location). The site must be served over HTTP
// (python3 -m http.server in site/ will do): a module page does not load
// from file://.
import { spawn } from 'node:child_process';
import { mkdirSync, writeFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';

const [base, outDir] = process.argv.slice(2);
if (!base || !outDir) { console.error('usage: node chrome.mjs <base URL> <output dir>'); process.exit(2); }
mkdirSync(outDir, { recursive: true });
const CHROME = process.env.CHROME || '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';
const port = 9300 + Math.floor(Math.random() * 500);
const chrome = spawn(CHROME, ['--headless=new', '--disable-gpu', `--remote-debugging-port=${port}`,
  `--user-data-dir=${mkdtempSync(join(tmpdir(), 'pg-chrome-'))}`, '--no-first-run', 'about:blank'], { stdio: 'ignore' });
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

async function target() {
  for (let i = 0; i < 50; i++) {
    try {
      const list = await (await fetch(`http://127.0.0.1:${port}/json`)).json();
      const page = list.find((t) => t.type === 'page');
      if (page) return page.webSocketDebuggerUrl;
    } catch { /* not up yet */ }
    await sleep(200);
  }
  throw new Error('Chrome did not start');
}

const failures = [], notes = {};
const check = (name, ok, detail) => { console.log((ok ? 'ok    ' : 'FAIL  ') + name + (detail !== undefined ? '  ' + JSON.stringify(detail) : '')); if (!ok) failures.push(name); };

try {
  const ws = new WebSocket(await target());
  await new Promise((r) => { ws.onopen = r; });
  let id = 0; const pending = new Map(), events = [];
  ws.onmessage = (m) => {
    const msg = JSON.parse(m.data);
    if (msg.id && pending.has(msg.id)) { const p = pending.get(msg.id); pending.delete(msg.id); msg.error ? p.reject(new Error(msg.error.message)) : p.resolve(msg.result); }
    else events.push(msg);
  };
  const send = (method, params = {}) => new Promise((resolve, reject) => { pending.set(++id, { resolve, reject }); ws.send(JSON.stringify({ id, method, params })); });
  const evaluate = async (expression) => {
    const r = await send('Runtime.evaluate', { expression, awaitPromise: true, returnByValue: true });
    if (r.exceptionDetails) throw new Error('page: ' + (r.exceptionDetails.exception?.description || r.exceptionDetails.text));
    return r.result.value;
  };
  const shot = async (name) => {
    if (/^[ab]/.test(name)) { await evaluate('document.querySelector(".guide").scrollIntoView()'); await sleep(100); }
    const r = await send('Page.captureScreenshot', { format: 'png', captureBeyondViewport: false });
    writeFileSync(join(outDir, name + '.png'), Buffer.from(r.data, 'base64'));
  };
  const viewport = (w, h, mobile = false) => send('Emulation.setDeviceMetricsOverride', { width: w, height: h, deviceScaleFactor: mobile ? 2 : 1, mobile });
  const key = async (type, k) => send('Input.dispatchKeyEvent', { type, key: k, code: k, windowsVirtualKeyCode: { ArrowUp: 38, ArrowDown: 40, ArrowLeft: 37, ArrowRight: 39 }[k] });
  // The site has no favicon; the browser's request for one is not the page's.
  const favicon = (u) => /\/favicon\.ico$/.test(u || '');
  const errors = () => events.filter((e) => (e.method === 'Runtime.exceptionThrown') || (e.method === 'Runtime.consoleAPICalled' && ['error', 'warning'].includes(e.params.type)) || (e.method === 'Log.entryAdded' && ['error', 'warning'].includes(e.params.entry.level) && !favicon(e.params.entry.url)))
    .map((e) => e.params.exceptionDetails?.exception?.description || e.params.entry?.text || JSON.stringify(e.params.args?.map((a) => a.value)));
  // Advances the world by whole seconds while the page's own loop is paused.
  const run = (sec) => evaluate(`(() => { const w = __pg.world; for (let i = 0; i < ${Math.round(sec * 100)}; i++) w.tick(); __pg.draw(); return __pg.describe(); })()`);

  await send('Page.enable'); await send('Runtime.enable'); await send('Log.enable'); await send('Network.enable');
  await send('Network.setCacheDisabled', { cacheDisabled: true });

  // --- load, weight, time -------------------------------------------------
  await viewport(1280, 900);
  const url = base.replace(/\/?$/, '/') + 'playground/';
  const t0 = Date.now();
  await send('Page.navigate', { url });
  for (let i = 0; i < 100 && !(await evaluate('!!(window.__pg && (window.__pg.world || window.__pg.error))')); i++) await sleep(50);
  const interactiveMs = Date.now() - t0;
  await sleep(800);                                   // fonts
  const loaded = await evaluate('({ ok: !!(window.__pg && window.__pg.world), err: window.__pg && window.__pg.error, loadMs: window.__pg && window.__pg.loadMs, nav: performance.getEntriesByType("navigation")[0].domContentLoadedEventEnd })');
  check('the page loads and the Lean core initialises', loaded.ok, loaded);
  const reqs = {};
  for (const e of events) {
    if (e.method === 'Network.responseReceived') reqs[e.params.requestId] = { url: e.params.response.url, status: e.params.response.status, mime: e.params.response.mimeType };
    if (e.method === 'Network.loadingFinished' && reqs[e.params.requestId]) reqs[e.params.requestId].wire = e.params.encodedDataLength;
  }
  const own = Object.values(reqs).filter((r) => r.url.startsWith(base) && !favicon(r.url));
  notes.requests = Object.values(reqs).map((r) => ({ url: r.url.replace(base, ''), status: r.status, bytesOnTheWire: r.wire }));
  notes.interactiveMs = interactiveMs; notes.coreLoadMs = Math.round(loaded.loadMs);
  check('every request of the site succeeded', own.every((r) => r.status === 200), own.filter((r) => r.status !== 200));
  check('no console errors or warnings after loading', errors().length === 0, errors());

  // --- the scripted scenario, on the page's own modules ---------------------
  await evaluate('__pg.setPaused(true)');
  const res = await evaluate(`import('./tools/test/scenario.mjs').then((m) => m.runScenario(__pg.world))`);
  check(`the scripted scenario in the page (${res.log.length} checks)`, res.ok, res.failures);

  // --- screenshots of each state, both wirings ----------------------------
  for (const [wiring, tag] of [['monitor-last', 'a'], ['joystick-bypass', 'b']]) {
    await evaluate(`(() => { const r = document.querySelector('input[name="wiring"][value="${wiring}"]'); r.click(); __pg.setPaused(true); })()`);
    let d = await run(6);
    check(`[${wiring}] pass`, d.word === 'pass' && d.who === 'planner', d); await shot(`${tag}1-pass`);
    await evaluate('document.getElementById("g-box").click(); __pg.setPaused(true)');
    d = await run(4);
    for (let i = 0; i < 40 && d.word !== 'slowdown'; i++) d = await run(0.2);
    check(`[${wiring}] slowdown`, d.word === 'slowdown', d); await shot(`${tag}2-slowdown`);
    for (let i = 0; i < 100 && d.word !== 'stop'; i++) d = await run(0.1);
    check(`[${wiring}] stop`, d.word === 'stop' && d.state === 'Stop: stop', d); await shot(`${tag}3-stop`);
    for (let i = 0; i < 40 && d.word !== 'silent'; i++) d = await run(0.1);
    d = await run(1);
    check(`[${wiring}] silent, the watchdog holds the robot`, d.word === 'silent' && d.dog === 'holding the robot' && d.who === 'nobody', d); await shot(`${tag}4-silent`);
    // Trapped or not: hold "turn left" for 2 s through the joystick.
    const th0 = await evaluate('__pg.world.robot.th');
    await evaluate('__pg.world.holdJoystick([0, 1.0], 2)');
    d = await run(1.5);
    const turned = Math.abs(await evaluate('__pg.world.robot.th') - th0);
    if (wiring === 'monitor-last') check(`[${wiring}] trapped: the joystick's turn is zeroed`, turned === 0 && d.joystick === 'forwarded', { turned, d });
    else check(`[${wiring}] freed: the joystick turns the robot`, turned > 1 && d.who === 'joystick', { turned, d });
    await shot(`${tag}5-joystick-at-the-box`);
    await run(1.5);
    await evaluate('document.getElementById("estop").click(); __pg.setPaused(true)');
    d = await run(1);
    check(`[${wiring}] emergency stop`, d.lock === 'ENGAGED' && d.who === 'nobody', d); await shot(`${tag}6-emergency-stop`);
    await evaluate('document.getElementById("reset").click()');
  }
  await evaluate(`document.querySelector('input[name="wiring"][value="monitor-last"]').click()`);

  // --- keyboard: arrows drive only when the floor has focus ------------------
  await evaluate('__pg.setPaused(false); document.activeElement.blur(); window.scrollTo(0, 0)');
  await key('rawKeyDown', 'ArrowDown'); await sleep(400); await key('keyUp', 'ArrowDown');
  let k = await evaluate('({ scrolled: window.scrollY, joy: __pg.world.last.joystick })');
  check('without focus on the floor, an arrow key scrolls the page and does not drive', k.scrolled > 0 && k.joy === null, k);
  await evaluate('document.getElementById("stage").focus({ preventScroll: true }); window.scrollTo(0, 0)');
  await key('rawKeyDown', 'ArrowLeft'); await sleep(700); 
  k = await evaluate('({ scrolled: window.scrollY, who: __pg.world.driver(), joy: __pg.world.last.joystick })');
  await key('keyUp', 'ArrowLeft');
  check('with focus on the floor, an arrow key drives and the page does not scroll', k.scrolled === 0 && k.who === 'joystick', k);
  await sleep(1600);
  const live = await evaluate('document.getElementById("live").textContent');
  check('the aria-live line says who drives and the monitor state', /drives\. Monitor: /.test(live), live);
  await evaluate('__pg.setPaused(true); document.getElementById("reset").click()');

  // --- 360 px, light and dark ------------------------------------------------
  await viewport(360, 740, true);
  await sleep(300); await run(3);
  let m = await evaluate('({ sw: document.documentElement.scrollWidth, iw: window.innerWidth, bw: document.body.scrollWidth })');
  check('360 px: no horizontal scroll', m.sw <= m.iw && m.bw <= m.iw, m);
  await shot('c1-360-top');
  await evaluate('document.getElementById("try").scrollIntoView()'); await sleep(200); await shot('c2-360-stage');
  await evaluate('document.querySelector(".flow.wiring").scrollIntoView()'); await sleep(200); await shot('c3-360-wiring');
  await evaluate('document.querySelector(\'input[name="wiring"][value="joystick-bypass"]\').click(); __pg.setPaused(true)'); await sleep(200); await shot('c3b-360-wiring-bypass');
  await evaluate('document.querySelector(\'input[name="wiring"][value="monitor-last"]\').click(); __pg.setPaused(true)');
  await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: 'dark' }] });
  await sleep(300); await evaluate('document.getElementById("g-box").click(); __pg.setPaused(true)'); await run(9);
  await evaluate('document.getElementById("try").scrollIntoView()'); await sleep(200); await shot('c4-360-dark');
  await viewport(1280, 900); await sleep(300); await evaluate('window.scrollTo(0, 0); __pg.draw()'); await shot('d1-dark-desktop');
  await evaluate('document.getElementById("wiring").scrollIntoView()'); await sleep(200); await shot('d2-dark-wiring');
  check('no console errors or warnings at the end', errors().length === 0, errors());

  writeFileSync(join(outDir, 'report.json'), JSON.stringify(notes, null, 2));
  console.log(JSON.stringify(notes, null, 2));
  ws.close();
} finally {
  chrome.kill();
}
console.log(failures.length ? `${failures.length} FAILED` : 'all checks passed');
process.exit(failures.length ? 1 : 0);
