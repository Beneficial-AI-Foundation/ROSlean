// Runs the scripted scenario on the page's logic under node. Exit code 0 iff
// every check holds.  node run_node.mjs [-v]
import { loadCore } from '../../shim.js';
import { World } from '../../sim.js';
import { runScenario } from './scenario.mjs';

const world = new World(await loadCore());
const t0 = performance.now();
const res = runScenario(world);
for (const l of res.log) {
  console.log((l.ok ? 'ok    ' : 'FAIL  ') + l.name + (l.ok && !process.argv.includes('-v') ? '' : '\n        ' + JSON.stringify(l.detail)));
}
console.log(`${res.log.length} checks, ${res.failures.length} failed, ${Math.round(performance.now() - t0)} ms`);
process.exit(res.ok ? 0 : 1);
