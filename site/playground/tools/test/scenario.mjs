// The scripted scenario, run on the page's own logic (sim.js, shim.js and
// roslean_core.wasm): under node by run_node.mjs, in a browser by
// scenario.html. It is the scenario of the prototype's pg_test.html (free
// patrol; a box ahead: slowdown, stop, silent; box removed; lock engaged
// and released) and, after it, being trapped and freed under each wiring.

import { WORLD } from '../../sim.js';

export function runScenario(world) {
  const log = [], failures = [];
  const check = (name, ok, detail) => { log.push({ name, ok: !!ok, detail }); if (!ok) failures.push(name); };
  const R = WORLD.patrolRadius;

  // Runs for `sec` seconds and counts, per step, the monitor's state and the driver.
  function run(sec) {
    const seen = {}, who = {}, order = [];
    for (let i = 0; i < sec * 100; i++) {
      world.tick();
      const m = world.monitorWord(), d = world.driver() || 'nobody';
      seen[m] = (seen[m] || 0) + 1; who[d] = (who[d] || 0) + 1;
      if (order[order.length - 1] !== m) order.push(m);
    }
    return { seen, who, order };
  }
  const pos = () => ({ x: world.robot.x, y: world.robot.y, th: world.robot.th });
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
  const round = (v) => Math.round(v * 1000) / 1000;

  for (const wiring of ['monitor-last', 'joystick-bypass']) {
    world.setWiring(wiring);
    const tag = `[${wiring}] `;

    // 1. Free patrol.
    let p0 = pos(), r = run(20);
    let radius = Math.hypot(world.robot.x, world.robot.y);
    check(tag + 'free patrol: the monitor passes and the planner drives',
      r.seen.pass > 1900 && !r.seen.stop && !r.seen.slowdown && !r.seen.silent && r.who.planner > 1900, r);
    check(tag + 'free patrol: the robot stays on its circle and moves',
      Math.abs(radius - R) < 0.06 && dist(p0, pos()) > 0.5, { radius: round(radius), moved: round(dist(p0, pos())) });

    // 2. A box ahead.
    world.placeBoxAhead();
    r = run(15);
    const iSlow = r.order.indexOf('slowdown'), iStop = r.order.indexOf('stop'), iSilent = r.order.indexOf('silent');
    check(tag + 'box ahead: slowdown, then stop, then silent',
      iSlow > 0 && iStop > iSlow && iSilent > iStop && r.order[r.order.length - 1] === 'silent', r.order);
    check(tag + 'box ahead: the stop zone is the one reported', world.litZone() === 0, world.litZone());
    const p1 = pos(); run(3);
    const gap = dist(pos(), world.boxes[0]);
    check(tag + 'box ahead: the robot is at rest, short of the box',
      dist(p1, pos()) === 0 && gap > 0.105 + 0.14 && gap < 0.7 && !world.bumped, { moved: dist(p1, pos()), gapToBoxCentre: round(gap) });
    check(tag + 'box ahead: the zero was published for stop_pub_timeout and then withheld',
      r.seen.stop >= 90 && r.seen.stop <= 130, r.seen);

    // 3. Trapped: the joystick asks to turn on the spot.
    const th0 = world.robot.th;
    world.holdJoystick([0, 1.0], 2);
    r = run(2.5);
    const turned = Math.abs(world.robot.th - th0);
    if (wiring === 'monitor-last') {
      check(tag + 'trapped: the joystick is selected by the mux but the stop zone zeroes its turn', turned === 0, { turned, who: r.who, seen: r.seen });
    } else {
      check(tag + 'not trapped: the joystick turns the robot', turned > 1.5 && r.who.joystick > 150, { turned: round(turned), who: r.who });
      // ... and can drive into the box.
      world.robot.th = th0;
      world.holdJoystick([0.2, 0], 4);
      run(4.5);
      check(tag + 'the joystick can drive into the box', dist(pos(), world.boxes[0]) < 0.105 + 0.21, { gapToBoxCentre: round(dist(pos(), world.boxes[0])) });
      world.holdJoystick([-0.15, 0], 3);
      run(3.5);
    }

    // 4. The box is taken away.
    world.boxes[0].x = -2.4; world.boxes[0].y = 1.5;
    p0 = pos(); r = run(6);
    check(tag + 'box removed: pass again, the planner drives on',
      r.order[r.order.length - 1] === 'pass' && dist(p0, pos()) > 0.4 && world.driver() === 'planner', { order: r.order, moved: round(dist(p0, pos())) });

    // 5. The emergency stop.
    world.setLock(true);
    r = run(3); const p2 = pos(); run(1);
    check(tag + 'lock engaged: nobody drives and the robot is at rest',
      world.driver() === null && dist(p2, pos()) === 0 && (r.who.nobody || 0) > 250, r.who);
    world.holdJoystick([0.2, 0], 1); run(1.2);
    check(tag + 'lock engaged: the joystick is blocked too', dist(p2, pos()) === 0 && world.last.joystick.forwarded === false, world.last.joystick);
    world.setLock(false);
    run(2);
    check(tag + 'lock released: the planner drives', world.driver() === 'planner' && dist(p2, pos()) > 0.1, world.driver());

    // 6. The joystick outranks the planner while it speaks, and for half a second after.
    world.holdJoystick([0, -1.0], 1);
    r = run(1);
    check(tag + 'joystick held: it drives, the planner is dropped',
      r.who.joystick > 80 && world.last.planner.forwarded === false, { who: r.who, planner: world.last.planner });
    r = run(2);
    check(tag + 'joystick released: the planner is back', world.driver() === 'planner', r.who);
  }
  world.setWiring('monitor-last');
  return { ok: failures.length === 0, failures, log };
}
