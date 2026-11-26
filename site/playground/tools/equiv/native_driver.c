/* Reads a case file (gen_cases.mjs) on stdin, makes the calls it lists on the
 * native build of the Lean core through the C interface of
 * ros/roslean_core/include/roslean_core/ffi.h, and prints one line per
 * answer. run_wasm.mjs does the same on the WebAssembly build; run.sh
 * compares the two outputs byte for byte.
 *
 * One line is one call: a name and its arguments. Integers are unsigned
 * decimal (64-bit patterns), byte arrays are hex ("-" for empty, "=" for
 * "the bytes of the previous points argument again").
 */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "roslean_core/ffi.h"

static char * tok(char ** p)
{
  char * s = *p;
  while (*s == ' ') {s++;}
  if (*s == 0 || *s == '\n') {return NULL;}
  char * e = s;
  while (*e && *e != ' ' && *e != '\n') {e++;}
  if (*e) {*e++ = 0;}
  *p = e;
  return s;
}

static uint64_t u64(char ** p)
{
  char * t = tok(p);
  if (!t) {fprintf(stderr, "missing argument\n"); exit(2);}
  return strtoull(t, NULL, 10);
}

static uint8_t * keep = NULL;   /* the previous points argument */
static size_t keep_n = 0;

/* A hex token as bytes; the result is valid until the next call with the
 * same `slot`. */
static const uint8_t * bytes(char ** p, size_t * n, int is_points)
{
  static uint8_t * buf[2] = {NULL, NULL};
  char * t = tok(p);
  if (!t) {fprintf(stderr, "missing bytes\n"); exit(2);}
  if (t[0] == '=' && is_points) {*n = keep_n; return keep;}
  size_t len = (t[0] == '-') ? 0 : strlen(t) / 2;
  free(buf[is_points]);
  buf[is_points] = malloc(len + 1);
  for (size_t i = 0; i < len; i++) {
    unsigned v;
    sscanf(t + 2 * i, "%2x", &v);
    buf[is_points][i] = (uint8_t)v;
  }
  if (is_points) {
    free(keep);
    keep = malloc(len + 1);
    memcpy(keep, buf[1], len);
    keep_n = len;
  }
  *n = len;
  return buf[is_points];
}

static void print_out(lean_object * m)
{
  roslean_core_inc(m);
  unsigned kind = roslean_cm_out_kind(m);
  roslean_core_inc(m);
  uint32_t zone = roslean_cm_out_zone(m);
  roslean_core_inc(m);
  uint64_t ratio = roslean_cm_out_ratio(m);
  roslean_core_inc(m);
  unsigned report = roslean_cm_out_report(m);
  roslean_core_inc(m);
  uint32_t rzone = roslean_cm_out_report_zone(m);
  printf("out %u %" PRIu32 " %" PRIu64 " %u %" PRIu32 "\n", kind, zone, ratio, report, rzone);
}

int main(void)
{
  if (!roslean_core_initialize()) {return 1;}
  lean_object * mb = NULL, * mux = NULL, * cb = NULL, * cm = NULL;
  char * line = NULL;
  size_t cap = 0;
  while (getline(&line, &cap, stdin) > 0) {
    char * p = line;
    char * c = tok(&p);
    if (!c || c[0] == '#') {continue;}
    if (!strcmp(c, "mux_builder")) {
      if (mb) {roslean_core_dec(mb);}
      mb = roslean_mux_builder_new(roslean_core_unit());
    } else if (!strcmp(c, "mux_add_input")) {
      uint64_t a = u64(&p), b = u64(&p);
      mb = roslean_mux_builder_add_input(mb, a, b);
    } else if (!strcmp(c, "mux_add_lock")) {
      uint64_t a = u64(&p), b = u64(&p);
      mb = roslean_mux_builder_add_lock(mb, a, b);
    } else if (!strcmp(c, "mux_new")) {
      if (mux) {roslean_core_dec(mux);}
      mux = roslean_mux_new(mb);
      mb = NULL;
    } else if (!strcmp(c, "mux_input")) {
      uint64_t i = u64(&p), t = u64(&p);
      mux = roslean_mux_on_input(mux, (uint32_t)i, t);
      roslean_core_inc(mux);
      printf("f %u\n", (unsigned)roslean_mux_forward(mux));
    } else if (!strcmp(c, "mux_lock")) {
      uint64_t j = u64(&p), v = u64(&p), t = u64(&p);
      mux = roslean_mux_on_lock(mux, (uint32_t)j, (uint8_t)v, t);
      roslean_core_inc(mux);
      printf("f %u\n", (unsigned)roslean_mux_forward(mux));
    } else if (!strcmp(c, "cm_builder")) {
      if (cb) {roslean_core_dec(cb);}
      cb = roslean_cm_builder_new(roslean_core_unit());
    } else if (!strcmp(c, "cm_source")) {
      uint64_t a = u64(&p), b = u64(&p), d = u64(&p), e = u64(&p);
      cb = roslean_cm_builder_set_source(cb, (uint8_t)a, b, (uint8_t)d, e);
    } else if (!strcmp(c, "cm_stop_pub")) {
      cb = roslean_cm_builder_set_stop_pub_timeout(cb, u64(&p));
    } else if (!strcmp(c, "cm_circle")) {
      uint64_t r = u64(&p), a = u64(&p), ratio = u64(&p), mp = u64(&p), en = u64(&p);
      cb = roslean_cm_builder_add_circle(cb, r, (uint8_t)a, ratio, mp, (uint8_t)en);
    } else if (!strcmp(c, "cm_polygon") || !strcmp(c, "cm_topic_polygon")) {
      uint64_t a = u64(&p), ratio = u64(&p), mp = u64(&p), en = u64(&p);
      cb = c[3] == 'p' ?
        roslean_cm_builder_add_polygon(cb, (uint8_t)a, ratio, mp, (uint8_t)en) :
        roslean_cm_builder_add_topic_polygon(cb, (uint8_t)a, ratio, mp, (uint8_t)en);
    } else if (!strcmp(c, "cm_vertex")) {
      uint64_t x = u64(&p), y = u64(&p);
      cb = roslean_cm_builder_add_vertex(cb, (uint32_t)x, (uint32_t)y);
    } else if (!strcmp(c, "cm_new")) {
      uint64_t nz = u64(&p);
      if (cm) {roslean_core_dec(cm);}
      cm = roslean_cm_new(cb);
      cb = NULL;
      roslean_core_inc(cm);
      unsigned ok = roslean_cm_ok(cm);
      roslean_core_inc(cm);
      unsigned err = roslean_cm_error(cm);
      printf("new %u %u", ok, err);
      for (uint64_t k = 0; k <= nz; k++) {
        roslean_core_inc(cm);
        printf(" %u", (unsigned)roslean_cm_zone_error(cm, (uint32_t)k));
      }
      printf("\n");
    } else if (!strcmp(c, "cm_lifecycle")) {
      cm = roslean_cm_on_lifecycle(cm, (uint8_t)u64(&p));
      roslean_core_inc(cm);
      printf("phase %u\n", (unsigned)roslean_cm_phase(cm));
    } else if (!strcmp(c, "cm_scan")) {
      uint64_t now = u64(&p), stamp = u64(&p), rmin = u64(&p), rmax = u64(&p);
      size_t n;
      const uint8_t * d = bytes(&p, &n, 0);
      cm = roslean_cm_on_scan(
        cm, now, stamp, (uint32_t)rmin, (uint32_t)rmax, roslean_core_byte_array(d, n));
      print_out(cm);
    } else if (!strcmp(c, "cm_shape")) {
      uint64_t zone = u64(&p), foreign = u64(&p), has = u64(&p);
      size_t n;
      const uint8_t * d = bytes(&p, &n, 0);
      cm = roslean_cm_on_shape(
        cm, (uint32_t)zone, (uint8_t)foreign, (uint8_t)has, roslean_core_byte_array(d, n));
      roslean_core_inc(cm);
      unsigned acc = roslean_cm_shape_accepted(cm);
      roslean_core_inc(cm);
      unsigned unu = roslean_cm_shape_unusable(cm, (uint32_t)zone);
      printf("shape %u %u\n", acc, unu);
    } else if (!strcmp(c, "cm_plan")) {
      uint64_t now = u64(&p), v[6];
      for (int i = 0; i < 6; i++) {v[i] = u64(&p);}
      roslean_core_inc(cm);
      printf("plan %u\n", (unsigned)roslean_cm_cmd_plan(cm, now, v[0], v[1], v[2], v[3], v[4], v[5]));
    } else if (!strcmp(c, "cm_cmd") || !strcmp(c, "cm_cmd_ref")) {
      uint64_t now = u64(&p), v[6];
      for (int i = 0; i < 6; i++) {v[i] = u64(&p);}
      uint64_t has = u64(&p);
      size_t np, nv;
      const uint8_t * pts = bytes(&p, &np, 1);
      const uint8_t * vs = bytes(&p, &nv, 0);
      if (c[6] == 0) {
        cm = roslean_cm_on_cmd(
          cm, now, v[0], v[1], v[2], v[3], v[4], v[5], (uint8_t)has,
          roslean_core_byte_array(pts, np), roslean_core_byte_array(vs, nv));
        print_out(cm);
      } else {
        /* The definition the theorems are stated on, on a second reference
         * to the same monitor: its answer is printed and its result dropped,
         * so the state continues from the monitor as it was. */
        roslean_core_inc(cm);
        lean_object * r = roslean_cm_on_cmd_reference(
          cm, now, v[0], v[1], v[2], v[3], v[4], v[5], (uint8_t)has,
          roslean_core_byte_array(pts, np), roslean_core_byte_array(vs, nv));
        print_out(r);
        roslean_core_dec(r);
      }
    } else if (!strcmp(c, "cm_cmd_noclock")) {
      uint64_t v[6];
      for (int i = 0; i < 6; i++) {v[i] = u64(&p);}
      cm = roslean_cm_on_cmd_no_clock(cm, v[0], v[1], v[2], v[3], v[4], v[5]);
      print_out(cm);
    } else if (!strcmp(c, "conv")) {
      char * which = tok(&p);
      uint64_t a = u64(&p), r = 0;
      if (!strcmp(which, "coord")) {
        r = roslean_cm_coord_mm(a);
      } else if (!strcmp(which, "vcoord")) {
        r = roslean_cm_vertex_coord_mm(a);
      } else if (!strcmp(which, "radius")) {
        r = roslean_cm_radius_mm(a);
      } else if (!strcmp(which, "vertex")) {
        r = roslean_cm_vertex_mm((uint32_t)a);
      } else if (!strcmp(which, "timeout")) {
        r = roslean_cm_timeout_ns(a);
      } else if (!strcmp(which, "srctimeout")) {
        uint64_t has = u64(&p), own = u64(&p);
        r = roslean_cm_source_timeout_ns(a, (uint8_t)has, own);
      } else if (!strcmp(which, "noop")) {
        r = roslean_noop(a);
      }
      printf("conv %" PRIu64 "\n", r);
    } else {
      fprintf(stderr, "unknown call '%s'\n", c);
      return 2;
    }
  }
  return 0;
}
