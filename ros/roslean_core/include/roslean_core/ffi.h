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

#ifdef __cplusplus
}
#endif

#endif  /* ROSLEAN_CORE__FFI_H_ */
