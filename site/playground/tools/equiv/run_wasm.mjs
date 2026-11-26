// The counterpart of native_driver.c for the WebAssembly build: reads a case
// file on stdin, makes the same calls on site/playground/roslean_core.wasm
// and prints the same lines.
import { createInterface } from 'node:readline';
import createRosleanCore from '../../roslean_core.js';

const M = await createRosleanCore();
if (M._roslean_core_initialize() === 0) throw new Error('initialisation failed');

function byteArray(bytes) {
  if (bytes.length === 0) return M._roslean_core_byte_array(0, 0);
  const p = M._malloc(bytes.length);
  M.HEAPU8.set(bytes, p);
  const a = M._roslean_core_byte_array(p, bytes.length);
  M._free(p);
  return a;
}
let keep = new Uint8Array(0);
function bytes(tok, isPoints) {
  if (tok === '=' && isPoints) return keep;
  const b = tok === '-' ? new Uint8Array(0) : new Uint8Array(Buffer.from(tok, 'hex'));
  if (isPoints) keep = b;
  return b;
}
const lines = [];
const print = (s) => { lines.push(s); if (lines.length > 5000) flush(); };
function flush() { if (lines.length) process.stdout.write(lines.join('\n') + '\n'); lines.length = 0; }
function printOut(m) {
  M._roslean_core_inc(m); const kind = M._roslean_cm_out_kind(m);
  M._roslean_core_inc(m); const zone = M._roslean_cm_out_zone(m) >>> 0;
  M._roslean_core_inc(m); const ratio = BigInt.asUintN(64, M._roslean_cm_out_ratio(m));
  M._roslean_core_inc(m); const report = M._roslean_cm_out_report(m);
  M._roslean_core_inc(m); const rzone = M._roslean_cm_out_report_zone(m) >>> 0;
  print(`out ${kind} ${zone} ${ratio} ${report} ${rzone}`);
}
const B = BigInt, u32 = (s) => Number(B(s) & 0xFFFFFFFFn), u8 = (s) => Number(B(s) & 0xFFn);

let mb = 0, mux = 0, cb = 0, cm = 0;
for await (const line of createInterface({ input: process.stdin, crlfDelay: Infinity })) {
  const a = line.split(' ');
  switch (a[0]) {
    case '': case '#': break;
    case 'mux_builder': if (mb) M._roslean_core_dec(mb); mb = M._roslean_mux_builder_new(M._roslean_core_unit()); break;
    case 'mux_add_input': mb = M._roslean_mux_builder_add_input(mb, B(a[1]), B(a[2])); break;
    case 'mux_add_lock': mb = M._roslean_mux_builder_add_lock(mb, B(a[1]), B(a[2])); break;
    case 'mux_new': if (mux) M._roslean_core_dec(mux); mux = M._roslean_mux_new(mb); mb = 0; break;
    case 'mux_input':
      mux = M._roslean_mux_on_input(mux, u32(a[1]), B(a[2]));
      M._roslean_core_inc(mux); print('f ' + M._roslean_mux_forward(mux)); break;
    case 'mux_lock':
      mux = M._roslean_mux_on_lock(mux, u32(a[1]), u8(a[2]), B(a[3]));
      M._roslean_core_inc(mux); print('f ' + M._roslean_mux_forward(mux)); break;
    case 'cm_builder': if (cb) M._roslean_core_dec(cb); cb = M._roslean_cm_builder_new(M._roslean_core_unit()); break;
    case 'cm_source': cb = M._roslean_cm_builder_set_source(cb, u8(a[1]), B(a[2]), u8(a[3]), B(a[4])); break;
    case 'cm_stop_pub': cb = M._roslean_cm_builder_set_stop_pub_timeout(cb, B(a[1])); break;
    case 'cm_circle': cb = M._roslean_cm_builder_add_circle(cb, B(a[1]), u8(a[2]), B(a[3]), B(a[4]), u8(a[5])); break;
    case 'cm_polygon': cb = M._roslean_cm_builder_add_polygon(cb, u8(a[1]), B(a[2]), B(a[3]), u8(a[4])); break;
    case 'cm_topic_polygon': cb = M._roslean_cm_builder_add_topic_polygon(cb, u8(a[1]), B(a[2]), B(a[3]), u8(a[4])); break;
    case 'cm_vertex': cb = M._roslean_cm_builder_add_vertex(cb, u32(a[1]), u32(a[2])); break;
    case 'cm_new': {
      if (cm) M._roslean_core_dec(cm);
      cm = M._roslean_cm_new(cb); cb = 0;
      M._roslean_core_inc(cm); const ok = M._roslean_cm_ok(cm);
      M._roslean_core_inc(cm); let s = `new ${ok} ${M._roslean_cm_error(cm)}`;
      for (let k = 0; k <= Number(a[1]); k++) { M._roslean_core_inc(cm); s += ' ' + M._roslean_cm_zone_error(cm, k); }
      print(s); break;
    }
    case 'cm_lifecycle':
      cm = M._roslean_cm_on_lifecycle(cm, u8(a[1]));
      M._roslean_core_inc(cm); print('phase ' + M._roslean_cm_phase(cm)); break;
    case 'cm_scan':
      cm = M._roslean_cm_on_scan(cm, B(a[1]), B(a[2]), u32(a[3]), u32(a[4]), byteArray(bytes(a[5], false)));
      printOut(cm); break;
    case 'cm_shape': {
      cm = M._roslean_cm_on_shape(cm, u32(a[1]), u8(a[2]), u8(a[3]), byteArray(bytes(a[4], false)));
      M._roslean_core_inc(cm); const acc = M._roslean_cm_shape_accepted(cm);
      M._roslean_core_inc(cm); print(`shape ${acc} ${M._roslean_cm_shape_unusable(cm, u32(a[1]))}`); break;
    }
    case 'cm_plan':
      M._roslean_core_inc(cm);
      print('plan ' + M._roslean_cm_cmd_plan(cm, B(a[1]), B(a[2]), B(a[3]), B(a[4]), B(a[5]), B(a[6]), B(a[7]))); break;
    case 'cm_cmd': case 'cm_cmd_ref': {
      const pts = byteArray(bytes(a[9], true)), vs = byteArray(bytes(a[10], false));
      const args = [B(a[1]), B(a[2]), B(a[3]), B(a[4]), B(a[5]), B(a[6]), B(a[7]), u8(a[8]), pts, vs];
      if (a[0] === 'cm_cmd') { cm = M._roslean_cm_on_cmd(cm, ...args); printOut(cm); }
      else { M._roslean_core_inc(cm); const r = M._roslean_cm_on_cmd_reference(cm, ...args); printOut(r); M._roslean_core_dec(r); }
      break;
    }
    case 'cm_cmd_noclock':
      cm = M._roslean_cm_on_cmd_no_clock(cm, B(a[1]), B(a[2]), B(a[3]), B(a[4]), B(a[5]), B(a[6])); printOut(cm); break;
    case 'conv': {
      const x = B(a[2]); let r;
      switch (a[1]) {
        case 'coord': r = M._roslean_cm_coord_mm(x); break;
        case 'vcoord': r = M._roslean_cm_vertex_coord_mm(x); break;
        case 'radius': r = M._roslean_cm_radius_mm(x); break;
        case 'vertex': r = M._roslean_cm_vertex_mm(u32(a[2])); break;
        case 'timeout': r = M._roslean_cm_timeout_ns(x); break;
        case 'srctimeout': r = M._roslean_cm_source_timeout_ns(x, u8(a[3]), B(a[4])); break;
        default: r = M._roslean_noop(x);
      }
      print('conv ' + BigInt.asUintN(64, r)); break;
    }
    default: throw new Error('unknown call ' + a[0]);
  }
}
flush();
