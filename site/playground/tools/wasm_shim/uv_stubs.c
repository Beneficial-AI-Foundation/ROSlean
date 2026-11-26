/* The only part of libuv the linked code reaches is uv_strerror (Lean's
 * runtime uses it to word I/O errors), which is in libuv's src/uv-common.c.
 * That file also holds libuv's shutdown hook, which calls the clean-up
 * functions of three parts of libuv that are not linked here. They have
 * nothing to clean up. */
void uv__threadpool_cleanup(void) {}
void uv__signal_cleanup(void) {}
void uv__process_title_cleanup(void) {}
