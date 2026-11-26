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
/* Lean's `lean_initialize_thread` and `lean_finalize_thread`. The Lean
 * runtime requires the first, before any Lean code runs, in every thread
 * that Lean did not create itself, other than the one that called
 * roslean_core_initialize; and the second before such a thread ends.
 * roslean::initialize() (runtime.hpp) does both. */
void roslean_core_initialize_thread(void);
void roslean_core_finalize_thread(void);
/* The number of calls of roslean_core_initialize_thread so far (for tests). */
int roslean_core_thread_inits(void);
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
 * boundary", and SPEC_NAV2.md, section 8). Parameter values are passed as read: `double` values as their
 * bit patterns (`*_bits`, uint64_t), the numbers of a polygon's `points` as
 * `float` bit patterns (uint32_t). Lean converts and refuses. `min_points`
 * and `stamp_ns` are Lean `Int64`: cast from int64_t at the call site, as
 * above. Flags (`enabled`, `has_own`, `foreign`, `has_verts`, `has_points`) are
 * true iff non-zero;
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
/* A polygon zone whose shape arrives on a topic: no vertices. */
lean_object * roslean_cm_builder_add_topic_polygon(
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
/* A lifecycle transition was performed: 0 configure, 1 activate,
 * 2 deactivate, 3 cleanup, 4 shutdown. Any other value changes nothing. A
 * new monitor is unconfigured. */
lean_object * roslean_cm_on_lifecycle(lean_object * m, uint8_t transition);
/* 0 unconfigured, 1 inactive, 2 active, 3 finalized. */
uint8_t roslean_cm_phase(lean_object * m);
/* `ranges` is a Lean ByteArray (roslean_core_byte_array): 4 bytes per
 * reading, the binary32 pattern of the range, little-endian. */
lean_object * roslean_cm_on_scan(
  lean_object * m, uint64_t now_ns, uint64_t stamp_ns, uint32_t range_min_bits,
  uint32_t range_max_bits, lean_object * ranges);
/* A polygon message for topic zone `zone`. `verts`: 16 bytes per vertex, the
 * binary64 patterns of x and y in the base frame, little-endian; looked at
 * only if `has_verts` is not 0 (0: the transform could not be looked up). */
lean_object * roslean_cm_on_shape(
  lean_object * m, uint32_t zone, uint8_t foreign, uint8_t has_verts, lean_object * verts);
/* The shape call made last was accepted: it installed a shape, or the mark
 * of a message with an unusable vertex. */
uint8_t roslean_cm_shape_accepted(lean_object * m);
/* The state holds, for zone `zone`, the mark of a polygon message with an
 * unusable vertex (ND6). A query. */
uint8_t roslean_cm_shape_unusable(lean_object * m, uint32_t zone);
/* Which transforms to look up for this command: 0 none, 1 those of the topic
 * zones, 3 the scan's and the zones'. A query. */
uint8_t roslean_cm_cmd_plan(
  lean_object * m, uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z);
/* `points`: 16 bytes per reading of the stored scan, the binary64 patterns
 * of x and y in the base frame, little-endian; looked at only if
 * `has_points` is not 0. `zone_verts`: for each zone in order a 4-byte
 * little-endian count n and 16 n bytes of vertices, n = 0xFFFFFFFF for
 * "none". */
lean_object * roslean_cm_on_cmd(
  lean_object * m, uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z, uint8_t has_points, lean_object * points,
  lean_object * zone_verts);
/* The definition the theorems are stated on; `roslean_cm_on_cmd` is proved to
 * return the same monitor (ROSlean/CollisionMonitor/Fast.lean). Slow. For
 * tests only. */
lean_object * roslean_cm_on_cmd_reference(
  lean_object * m, uint64_t now_ns, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z, uint8_t has_points, lean_object * points,
  lean_object * zone_verts);
/* A command that arrives while the node has no valid clock reading (SPEC,
 * D15): there is no time argument. Answered with kind 4 when processed. */
lean_object * roslean_cm_on_cmd_no_clock(
  lean_object * m, uint64_t lin_x, uint64_t lin_y, uint64_t lin_z,
  uint64_t ang_x, uint64_t ang_y, uint64_t ang_z);
/* 0 silent (publish nothing at all), 1 pass, 2 slowdown, 3 stop by a zone,
 * 4 stop for an invalid source, 5 processed but the velocity is withheld. */
uint8_t roslean_cm_out_kind(lean_object * m);
uint32_t roslean_cm_out_zone(lean_object * m);
uint64_t roslean_cm_out_ratio(lean_object * m);
/* 0 no state message, 1 normal, 2 stop, 3 slowdown, 4 invalid source; the
 * zone of 2 and 3 in roslean_cm_out_report_zone. */
uint8_t roslean_cm_out_report(lean_object * m);
uint32_t roslean_cm_out_report_zone(lean_object * m);
/* The conversions on their own, for tests and log messages. Results are Lean
 * `Int64`: cast to int64_t. -1 (INT64_MIN for a vertex) means refused. */
uint64_t roslean_cm_coord_mm(uint64_t bits);
uint64_t roslean_cm_vertex_coord_mm(uint64_t bits);
uint64_t roslean_cm_radius_mm(uint64_t bits);
uint64_t roslean_cm_vertex_mm(uint32_t bits);
uint64_t roslean_cm_timeout_ns(uint64_t bits);
uint64_t roslean_cm_source_timeout_ns(uint64_t node_bits, uint8_t has_own, uint64_t own_bits);

#ifdef __cplusplus
}
#endif

#endif  /* ROSLEAN_CORE__FFI_H_ */
