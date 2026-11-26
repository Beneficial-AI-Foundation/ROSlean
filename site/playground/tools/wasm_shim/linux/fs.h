/* Stand-in for <linux/fs.h>, which the Emscripten sysroot does not have.
 *
 * Lean's src/runtime/io.cpp (4.37.0-rc1) includes it on every platform that
 * is neither Windows nor macOS, for one function: the rename that must not
 * replace an existing file, made with syscall(SYS_renameat2, ...). Here that
 * call fails with ENOSYS. Nothing in ROSleanCore touches files, so the
 * function is never called by the page; it only has to compile.
 *
 * Seen by io.cpp alone (tools/build.sh adds this directory to its include
 * path only).
 */
#pragma once
#include <errno.h>
#ifndef RENAME_NOREPLACE
#define RENAME_NOREPLACE (1 << 0)
#endif
#ifndef SYS_renameat2
#define SYS_renameat2 0
#endif
static inline long roslean_no_syscall(long n, ...) { (void)n; errno = ENOSYS; return -1; }
#define syscall roslean_no_syscall
