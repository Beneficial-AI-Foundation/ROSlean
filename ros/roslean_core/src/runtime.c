/* The only code outside Lean's generated C that touches the Lean runtime.
 * It is compiled into libroslean_core, so that programs using the library
 * need neither the Lean headers nor a direct link to the Lean runtime. */

#include <string.h>

#include <lean/lean.h>

#include "roslean_core/ffi.h"

/* Part of the Lean runtime, but not declared in lean.h. */
void lean_initialize_runtime_module(void);
void lean_initialize_thread(void);
void lean_finalize_thread(void);

int roslean_core_initialize(void)
{
  lean_initialize_runtime_module();
  lean_object * res = initialize_ROSleanCore_ROSleanCore(1 /* builtin */);
  const int ok = lean_io_result_is_ok(res);
  if (!ok) {
    lean_io_result_show_error(res);
  }
  lean_dec(res);
  if (ok) {
    lean_io_mark_end_initialization();
  }
  return ok;
}

/* How often roslean_core_initialize_thread has run: for the test that the
 * wrappers call it in every thread that needs it. */
static int thread_inits = 0;

void roslean_core_initialize_thread(void)
{
  __atomic_add_fetch(&thread_inits, 1, __ATOMIC_SEQ_CST);
  lean_initialize_thread();
}

int roslean_core_thread_inits(void) {return __atomic_load_n(&thread_inits, __ATOMIC_SEQ_CST);}

void roslean_core_finalize_thread(void) {lean_finalize_thread();}

lean_object * roslean_core_unit(void) {return lean_box(0);}

void roslean_core_inc(lean_object * o) {lean_inc(o);}

void roslean_core_dec(lean_object * o) {lean_dec(o);}

lean_object * roslean_core_byte_array(const uint8_t * data, size_t size)
{
  lean_object * a = lean_alloc_sarray(1, size, size);
  if (size != 0) {
    memcpy(lean_sarray_cptr(a), data, size);
  }
  return a;
}
