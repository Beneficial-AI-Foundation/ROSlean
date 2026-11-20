/* The only code outside Lean's generated C that touches the Lean runtime.
 * It is compiled into libroslean_core, so that programs using the library
 * need neither the Lean headers nor a direct link to the Lean runtime. */

#include <lean/lean.h>

#include "roslean_core/ffi.h"

/* Part of the Lean runtime, but not declared in lean.h. */
void lean_initialize_runtime_module(void);

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

lean_object * roslean_core_unit(void) {return lean_box(0);}

void roslean_core_inc(lean_object * o) {lean_inc(o);}

void roslean_core_dec(lean_object * o) {lean_dec(o);}
