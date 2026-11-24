/* C prototypes of the functions exported by the Lean library ROSleanCore.
 *
 * These mirror the `@[export]` declarations in core/ROSleanCore. They are
 * checked at build time: test/check_signatures.c includes this header and then
 * the C code Lean generated, so a mismatch is a compile error.
 *
 * Use the functions of this header only; do not call the Lean runtime
 * directly from a program that links libroslean_core.
 *
 * Ownership: every `lean_object *` argument is consumed and every
 * `lean_object *` result is owned by the caller.
 */

#ifndef ROSLEAN_CORE__FFI_H_
#define ROSLEAN_CORE__FFI_H_

#include <stddef.h>
#include <stdint.h>

/* Users of the library see Lean objects as opaque pointers and do not need
 * the Lean headers. The library's own sources include <lean/lean.h> first. */
#ifndef LEAN_VERSION_MAJOR
typedef struct roslean_opaque_lean_object lean_object;
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Hand-written helpers (src/runtime.c). */

/* Initialises the Lean runtime and the ROSleanCore library; call once, before
 * anything else. Returns non-zero on success. */
int roslean_core_initialize(void);
/* Lean's `()`: the argument of functions that take `Unit`. */
lean_object * roslean_core_unit(void);
/* Take or release one reference to an object. */
void roslean_core_inc(lean_object * o);
void roslean_core_dec(lean_object * o);
/* A new Lean `ByteArray` holding a copy of the `size` bytes at `data` (which
 * may be null if `size` is 0). */
lean_object * roslean_core_byte_array(const uint8_t * data, size_t size);

/* Exported by Lean. */

lean_object * initialize_ROSleanCore_ROSleanCore(uint8_t builtin);

uint64_t roslean_noop(uint64_t seq);

lean_object * roslean_mux_builder_new(lean_object * unit);
/* `priority` and `timeout_ns` are Lean `Int64`, which Lean passes as the
 * two's-complement bit pattern in a uint64_t. The compile-time check cannot
 * tell the two apart, so cast from int64_t at the call site. */
lean_object * roslean_mux_builder_add_input(lean_object * b, uint64_t priority, uint64_t timeout_ns);
lean_object * roslean_mux_builder_add_lock(lean_object * b, uint64_t priority, uint64_t timeout_ns);
lean_object * roslean_mux_new(lean_object * b);
lean_object * roslean_mux_on_input(lean_object * m, uint32_t i, uint64_t t_ns);
lean_object * roslean_mux_on_lock(lean_object * m, uint32_t j, uint8_t value, uint64_t t_ns);
uint8_t roslean_mux_forward(lean_object * m);

/* Collision monitor (core/ROSleanCore/CollisionMonitorFFI.lean; the meaning of
 * every argument is laid down in docs/collision_monitor/SPEC.md, "Shim
 * boundary"). Parameter values are passed as read: `double` values as their
 * bit patterns (`*_bits`, uint64_t), the numbers of a polygon's `points` as
 * `float` bit patterns (uint32_t). Lean converts and refuses. `min_points`
 * and `stamp_ns` are Lean `Int64`: cast from int64_t at the call site, as
 * above. Flags (`enabled`, `has_own`, `usable`) are true iff non-zero;
 * `action` is 1 for slowdown and stop for every other value. */
lean_object * roslean_cm_builder_new(lean_object * unit);
lean_object * roslean_cm_builder_set_source(
  lean_object * b, uint8_t enabled, uint64_t node_timeout_bits, uint8_t has_own,
  uint64_t own_timeout_bits);
lean_object * roslean_cm_builder_set_stop_pub_timeout(lean_object * b, uint64_t timeout_bits);
lean_object * roslean_cm_builder_add_circle(
  lean_object * b, uint64_t radius_bits, uint8_t action, uint64_t ratio_bits, uint64_t min_points,
  uint8_t enabled);
lean_object * roslean_cm_builder_add_polygon(
  lean_object * b, uint8_t action, uint64_t ratio_bits, uint64_t min_points, uint8_t enabled);
lean_object * roslean_cm_builder_add_vertex(lean_object * b, uint32_t x_bits, uint32_t y_bits);
lean_object * roslean_cm_new(lean_object * b);
uint8_t roslean_cm_ok(lean_object * m);
/* 0 accepted, 1 `source_timeout`, 2 the source's own `source_timeout`,
 * 3 `stop_pub_timeout`, 4 a zone, 5 `set_source` was never called, 6 a vertex
 * was added when the zone added last was not a polygon. A new builder is not
 * accepted (5), and no call clears a refusal. */
uint8_t roslean_cm_error(lean_object * m);
/* 0 zone `k` is accepted, 1 `radius`, 2 `points`, 3 `slowdown_ratio`,
 * 4 `min_points`, 5 no such zone. */
uint8_t roslean_cm_zone_error(lean_object * m, uint32_t k);
/* `beams` is a Lean ByteArray (roslean_core_byte_array) of 20-byte records. */
lean_object * roslean_cm_on_scan(
  lean_object * m, uint64_t now_ns, uint64_t stamp_ns, uint8_t usable, uint32_t range_min_bits,
  uint32_t range_max_bits, lean_object * beams);
lean_object * roslean_cm_on_cmd(
  lean_object * m, uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z);
/* The definition the theorems are stated on; `roslean_cm_on_cmd` is proved to
 * return the same monitor (ROSlean/CollisionMonitor/Fast.lean). Slow. For
 * tests only. */
lean_object * roslean_cm_on_cmd_reference(
  lean_object * m, uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z);
/* A command that arrives while the node has no valid clock reading (SPEC,
 * D15): there is no time argument. Answered with kind 4 when processed. */
lean_object * roslean_cm_on_cmd_no_clock(
  lean_object * m, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z);
uint8_t roslean_cm_out_kind(lean_object * m);
uint32_t roslean_cm_out_zone(lean_object * m);
uint64_t roslean_cm_out_ratio(lean_object * m);
/* The conversions on their own, for tests and log messages. Results are Lean
 * `Int64`: cast to int64_t. -1 (INT64_MIN for a vertex) means refused. */
uint64_t roslean_cm_coord_mm(uint64_t bits);
uint64_t roslean_cm_radius_mm(uint64_t bits);
uint64_t roslean_cm_vertex_mm(uint32_t bits);
uint64_t roslean_cm_timeout_ns(uint64_t bits);
uint64_t roslean_cm_source_timeout_ns(uint64_t node_bits, uint8_t has_own, uint64_t own_bits);

#ifdef __cplusplus
}
#endif

#endif  /* ROSLEAN_CORE__FFI_H_ */
