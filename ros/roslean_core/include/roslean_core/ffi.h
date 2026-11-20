/* C prototypes of the functions exported by the Lean library ROSleanCore.
 *
 * These mirror the `@[export]` declarations in ROSleanCore/*.lean. They are
 * checked at build time: test/check_signatures.c includes this header and then
 * the C code Lean generated, so a mismatch is a compile error.
 *
 * Ownership: every `lean_object *` argument is consumed and every
 * `lean_object *` result is owned by the caller.
 */

#ifndef ROSLEAN_CORE__FFI_H_
#define ROSLEAN_CORE__FFI_H_

#include <stdint.h>

#include <lean/lean.h>

#ifdef __cplusplus
extern "C" {
#endif

void lean_initialize_runtime_module();
lean_object * initialize_ROSlean_ROSleanCore(uint8_t builtin);

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
