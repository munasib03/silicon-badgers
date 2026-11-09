/* Exhaustive check of fp_exp.sv: mirrors the RTL arithmetic with GMP and
 * compares every finite input against MPFR's correctly rounded exp().
 * usage: check_fp_exp f16 | f32 <proc_index> <num_procs> */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <gmp.h>
#include <mpfr.h>
#include "exp_tables.h"

typedef unsigned __int128 u128;

/* Port of fp_round.sv. Returns bits; *ovf set on overflow. */
static uint32_t fp_round(int EXP, int FRAC, int sign, uint64_t mag, int exponent, int sticky_in, int *ovf) {
  int BIAS = (1 << (EXP - 1)) - 1, UNIT = 1 - BIAS - FRAC, EXP_MAX = (1 << EXP) - 1;
  *ovf = 0;
  uint32_t d = (uint32_t)sign << (EXP + FRAC);
  int msb = -1;
  for (int i = 0; i < 64; i++) if ((mag >> i) & 1) msb = i;
  if (msb < 0) return d;
  int lsb = (msb + exponent - FRAC > UNIT) ? msb + exponent - FRAC : UNIT;
  int rs = lsb - exponent;
  uint64_t kept = 0; int guard = 0, sticky = sticky_in;
  uint64_t keepmask = (FRAC + 2 >= 64) ? ~0ULL : ((1ULL << (FRAC + 2)) - 1);
  if (rs <= 0) kept = (mag << (-rs)) & keepmask;
  else if (rs > 65) sticky = 1;
  else {
    u128 ext = (u128)mag;
    kept = (uint64_t)(ext >> rs) & keepmask;
    guard = (int)((ext >> (rs - 1)) & 1);
    for (int i = 0; i < 64; i++) if (i < rs - 1) sticky |= (int)((mag >> i) & 1);
  }
  int up = guard && (sticky || (kept & 1));
  uint64_t r = (kept + up) & keepmask;
  if ((r >> (FRAC + 1)) & 1) { r >>= 1; lsb++; }
  int ef = ((r >> FRAC) & 1) ? lsb - UNIT + 1 : 0;
  if (ef >= EXP_MAX) { *ovf = 1; return 0; }
  return d | ((uint32_t)ef << FRAC) | (uint32_t)(r & ((1ULL << FRAC) - 1));
}

static mpz_t INV, LN2, C[6], T1[256], T2[256];
static mpz_t X, kp, kk, R, t, r, j, rr3, h, poly, pt, p, m;

static void init(void) {
  mpz_init_set_str(INV, INV_LN2_HEX, 16);
  mpz_init_set_str(LN2, LN2_HEX, 16);
  for (int i = 0; i < 6; i++) mpz_init_set_str(C[i], C_HEX[i], 16);
  for (int i = 0; i < 256; i++) { mpz_init_set_str(T1[i], T1_HEX[i], 16); mpz_init_set_str(T2[i], T2_HEX[i], 16); }
  mpz_inits(X, kp, kk, R, t, r, j, rr3, h, poly, pt, p, m, NULL);
}

/* Model of fp_exp.sv for F32 input bits a. */
static void model(uint32_t a, uint16_t *d16, int *e16, uint32_t *d32, int *e32) {
  int sign = a >> 31, ef = (a >> 23) & 0xff;
  uint32_t sig = ((ef != 0) << 23) | (a & 0x7fffff);
  int input_error = (ef == 255), tiny = ef < 97, huge = ef >= 134;
  uint64_t mag = 0; int exponent = 0, sticky = 0, o16, o32;
  if (tiny) mag = 1;
  else if (!huge) {
    mpz_set_ui(X, sig); mpz_mul_2exp(X, X, ef - 97); if (sign) mpz_neg(X, X);
    mpz_mul(kp, X, INV);
    mpz_set_ui(t, 1); mpz_mul_2exp(t, t, 84); mpz_add(kp, kp, t);
    mpz_fdiv_q_2exp(kk, kp, 85);
    long k = mpz_get_si(kk);
    mpz_mul_2exp(R, X, 75); mpz_mul(t, kk, LN2); mpz_sub(R, R, t);
    mpz_fdiv_q_2exp(r, R, 32);
    mpz_fdiv_q_2exp(j, r, 88); mpz_fdiv_r_2exp(j, j, 8); unsigned j1 = mpz_get_ui(j);
    mpz_fdiv_q_2exp(j, r, 80); mpz_fdiv_r_2exp(j, j, 8); unsigned j2 = mpz_get_ui(j);
    mpz_fdiv_r_2exp(rr3, r, 80);
    mpz_set(h, C[5]);
    for (int n = 4; n >= 0; n--) { mpz_mul(t, h, rr3); mpz_fdiv_q_2exp(t, t, 96); mpz_add(h, C[n], t); }
    mpz_set(poly, h);
    /* T1 index: j1 is two's complement; table stored for j = -128..127 */
    int js = (j1 >= 128) ? (int)j1 - 256 : (int)j1;
    mpz_mul(pt, T1[js + 128], T2[j2]); mpz_fdiv_q_2exp(pt, pt, 96);
    mpz_mul(p, pt, poly); mpz_fdiv_q_2exp(p, p, 96);
    if (mpz_sizeinbase(p, 2) > 97) { fprintf(stderr, "p too wide a=%08x\n", a); exit(2); }
    mpz_fdiv_q_2exp(m, p, 33);
    mag = mpz_get_ui(m);
    exponent = (int)k - 63; sticky = 1;
  }
  *d16 = (uint16_t)fp_round(5, 10, 0, mag, exponent, sticky, &o16);
  *d32 = fp_round(8, 23, 0, mag, exponent, sticky, &o32);
  *e16 = input_error || (huge && !sign) || o16;
  *e32 = input_error || (huge && !sign) || o32;
}

static float bits_f(uint32_t u) { float f; memcpy(&f, &u, 4); return f; }
static uint32_t f_bits(float f) { uint32_t u; memcpy(&u, &f, 4); return u; }

/* Correctly rounded reference. Returns 1 on overflow (result rounds to infinity). */
static int ref_exp(double x, int prec, int emin, int emax, double *out) {
  mpfr_t y; mpfr_init2(y, prec);
  mpfr_set_emin(emin); mpfr_set_emax(emax);
  mpfr_set_d(y, x, MPFR_RNDN);              /* exact: prec >= input precision */
  int tern = mpfr_exp(y, y, MPFR_RNDN);
  tern = mpfr_subnormalize(y, tern, MPFR_RNDN);
  int inf = mpfr_inf_p(y);
  *out = mpfr_get_d(y, MPFR_RNDN);
  mpfr_set_emin(mpfr_get_emin_min()); mpfr_set_emax(mpfr_get_emax_max());
  mpfr_clear(y);
  return inf;
}

static uint16_t f16_bits_from_double(double v) {
  /* v is an exactly representable non-negative F16 value */
  if (v == 0) return 0;
  int e; double fr = frexp(v, &e);           /* v = fr * 2^e, fr in [0.5,1) */
  int E = e - 1;                              /* v = 1.f * 2^E */
  if (E < -14) return (uint16_t)ldexp(v, 24); /* subnormal: v / 2^-24 */
  uint32_t frac = (uint32_t)((ldexp(fr, 11)) - 1024);
  return (uint16_t)(((E + 15) << 10) | frac);
}

int main(int argc, char **argv) {
  init();
  long bad = 0, checked = 0;
  if (!strcmp(argv[1], "f16")) {
    for (uint32_t h16 = 0; h16 < 65536; h16++) {
      int ef = (h16 >> 10) & 31;
      if (ef == 31) continue;
      /* exact F16 -> F32 */
      int s = h16 >> 15; uint32_t fr = h16 & 1023;
      double v = (ef == 0) ? ldexp(fr, -24) : ldexp(1024 + fr, ef - 25);
      if (s) v = -v;
      uint32_t a = f_bits((float)v);
      uint16_t d16; uint32_t d32; int e16, e32;
      model(a, &d16, &e16, &d32, &e32);
      double ref; int ovf = ref_exp(v, 11, -23, 16, &ref);
      checked++;
      if (ovf) { if (!e16) { bad++; if (bad < 20) printf("F16 x=%04x expected overflow got %04x\n", h16, d16); } continue; }
      uint16_t want = f16_bits_from_double(ref);
      if (e16 || d16 != want) { bad++; if (bad < 20) printf("F16 x=%04x want %04x got %04x err=%d\n", h16, want, d16, e16); }
    }
  } else {
    int idx = atoi(argv[2]), np = atoi(argv[3]);
    uint64_t lo = ((uint64_t)1 << 32) * idx / np, hi = ((uint64_t)1 << 32) * (idx + 1) / np;
    for (uint64_t u = lo; u < hi; u++) {
      uint32_t a = (uint32_t)u;
      if (((a >> 23) & 0xff) == 255) continue;
      uint16_t d16; uint32_t d32; int e16, e32;
      model(a, &d16, &e16, &d32, &e32);
      double ref; int ovf = ref_exp((double)bits_f(a), 24, -148, 128, &ref);
      checked++;
      if (ovf) { if (!e32) { bad++; if (bad < 20) printf("F32 x=%08x expected overflow got %08x\n", a, d32); } continue; }
      uint32_t want = f_bits((float)ref);
      if (e32 || d32 != want) { bad++; if (bad < 20) printf("F32 x=%08x want %08x got %08x err=%d\n", a, want, d32, e32); }
    }
  }
  printf("checked %ld mismatches %ld\n", checked, bad);
  return bad != 0;
}
