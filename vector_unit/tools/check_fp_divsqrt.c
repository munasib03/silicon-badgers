/* Cycle-accurate C mirrors of div_iter.sv (shared divider: FP DIV/RCP, integer) and
 * the sequential fp_rsqrt.sv, checked against MPFR and C integer division.
 * usage: check_fp_divsqrt int N | rsqrt16|rsqrt32|rcp16|rcp32 [idx np] | div16|div32 N */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <mpfr.h>
typedef unsigned __int128 u128;

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
  uint64_t keepmask = (1ULL << (FRAC + 2)) - 1;
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

/* div_iter.sv: restoring shift-subtract, cycle by cycle */
static void div_iter(uint64_t dividend, uint64_t divisor, int iterations, uint64_t *q, uint64_t *r) {
  u128 acc = 0; uint64_t quo = iterations == 64 ? dividend : dividend << (64 - iterations);
  for (int c = 0; c < iterations; c++) {
    u128 sh = ((acc << 1) | (quo >> 63)) & (((u128)1 << 65) - 1);
    if (sh >= (u128)divisor) { acc = sh - divisor; quo = (quo << 1) | 1; }
    else { acc = sh; quo = quo << 1; }
  }
  *q = quo; *r = (uint64_t)acc;
}

static uint32_t widen16(uint32_t h) {
  uint32_t s = h >> 15, e = (h >> 10) & 31, f = h & 1023;
  if (e == 31) return (s << 31) | (0xFFu << 23) | (f << 13);
  if (e) return (s << 31) | ((e + 112) << 23) | (f << 13);
  if (!f) return s << 31;
  int msb = 0; for (int i = 0; i < 10; i++) if ((f >> i) & 1) msb = i;
  uint32_t sh = (f << (10 - msb)) & 0x7FF;
  return (s << 31) | ((uint32_t)(msb + 103) << 23) | ((sh & 1023) << 13);
}

/* fp_rsqrt.sv: bit-by-bit root search on the F32-widened input */
static uint32_t rsqrt(int EXP, int FRAC, uint32_t a_in, int *err) {
  uint32_t a = EXP == 5 ? widen16(a_in) : a_in;
  const int N = 39;
  uint32_t ea = (a >> 23) & 255;
  uint64_t sig = ((uint64_t)(ea != 0) << 23) | (a & 0x7FFFFF);
  int e = ((ea == 0) ? 1 : (int)ea) - 150;
  int msb = 0; for (int i = 0; i < 24; i++) if ((sig >> i) & 1) msb = i;
  uint64_t m = (sig << (23 - msb)) & ((1ULL << 25) - 1);
  e -= 23 - msb;
  if (e & 1) { m <<= 1; e -= 1; }
  u128 target = (u128)1 << 78;
  uint64_t root = 0; int exact = 0;
  for (int idx = 27; idx >= 0; idx--) {
    uint64_t trial = root | (1ULL << idx);
    u128 prod = (u128)trial * trial * m;      /* < 2^81 fits */
    if (prod <= target) { root = trial; exact = prod == target; }
  }
  int ovf;
  uint32_t d = fp_round(EXP, FRAC, 0, root, -N - (e >> 1), !exact, &ovf);
  *err = (ea == 255) || (sig == 0) || (a >> 31) || ovf;
  return d;
}

/* vec_special.sv FP DIV/RCP: F32-widened operands, shared 64-bit divider, 50 iterations */
static uint32_t fdiv(int EXP, int FRAC, uint32_t a_in, uint32_t b_in, int *err) {
  uint32_t a = EXP == 5 ? widen16(a_in) : a_in, b = EXP == 5 ? widen16(b_in) : b_in;
  uint32_t ea = (a >> 23) & 255, eb = (b >> 23) & 255;
  uint64_t sa = ((uint64_t)(ea != 0) << 23) | (a & 0x7FFFFF), sb = ((uint64_t)(eb != 0) << 23) | (b & 0x7FFFFF);
  int xa = ((ea == 0) ? 1 : (int)ea) - 127, xb = ((eb == 0) ? 1 : (int)eb) - 127;
  int ma = 0, mb = 0;
  for (int i = 0; i < 24; i++) { if ((sa >> i) & 1) ma = i; if ((sb >> i) & 1) mb = i; }
  uint64_t na = (sa << (23 - ma)) & 0xFFFFFF, nb = (sb << (23 - mb)) & 0xFFFFFF;
  xa -= 23 - ma; xb -= 23 - mb;
  uint64_t q, r; div_iter(na << 26, nb, 50, &q, &r);
  int ovf;
  int sign = ((a ^ b) >> 31) & 1;
  uint32_t d = fp_round(EXP, FRAC, sign, sa == 0 ? 0 : q, xa - xb - 26, (sa != 0) && (r != 0), &ovf);
  *err = (ea == 255) || (eb == 255) || (sb == 0) || ovf;
  return d;
}

/* reference: decode bits to double (exact), compute in MPFR, round to format */
static double decode(int EXP, int FRAC, uint32_t a) {
  int BIAS = (1 << (EXP - 1)) - 1;
  uint32_t e = (a >> FRAC) & ((1u << EXP) - 1), f = a & ((1u << FRAC) - 1);
  double v = e ? ldexp((double)((1u << FRAC) | f), (int)e - BIAS - FRAC) : ldexp((double)f, 1 - BIAS - FRAC);
  return ((a >> (EXP + FRAC)) & 1) ? -v : v;
}
static uint32_t encode(int EXP, int FRAC, double v) {   /* v exactly representable */
  int BIAS = (1 << (EXP - 1)) - 1;
  uint32_t s = signbit(v) ? 1 : 0; v = fabs(v);
  if (v == 0) return s << (EXP + FRAC);
  int e; frexp(v, &e); int E = e - 1;
  if (E < 1 - BIAS) return (s << (EXP + FRAC)) | (uint32_t)ldexp(v, BIAS - 1 + FRAC);
  uint32_t f = (uint32_t)(ldexp(v, FRAC - E) - ldexp(1, FRAC));
  return (s << (EXP + FRAC)) | ((uint32_t)(E + BIAS) << FRAC) | f;
}
/* op: 0 rsqrt, 1 div. returns 1 if result is inf */
static int ref(int EXP, int FRAC, int op, double x, double y, double *out) {
  int BIAS = (1 << (EXP - 1)) - 1;
  mpfr_set_emin(1 - BIAS - FRAC + 1); mpfr_set_emax(BIAS + 1);
  mpfr_t a, b, r; mpfr_inits2(FRAC + 1, r, NULL); mpfr_inits2(64, a, b, NULL);
  mpfr_set_d(a, x, MPFR_RNDN); mpfr_set_d(b, y, MPFR_RNDN);
  int t = op == 0 ? mpfr_rec_sqrt(r, a, MPFR_RNDN) : mpfr_div(r, a, b, MPFR_RNDN);
  t = mpfr_subnormalize(r, t, MPFR_RNDN);
  int inf = mpfr_inf_p(r);
  *out = mpfr_get_d(r, MPFR_RNDN);
  mpfr_clears(a, b, r, NULL);
  mpfr_set_emin(mpfr_get_emin_min()); mpfr_set_emax(mpfr_get_emax_max());
  return inf;
}

static uint64_t rng = 88172645463325252ULL;
static uint64_t xr(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

static long bad = 0, checked = 0;
static void cmp(const char *name, int EXP, int FRAC, uint32_t a, uint32_t b, uint32_t got, int gerr, int wanterr, int inf, double refv) {
  checked++;
  uint32_t want = (wanterr || inf) ? 0 : encode(EXP, FRAC, refv);
  int we = wanterr || inf;
  if (we != gerr || (!we && want != got)) {
    bad++; if (bad < 15) printf("%s a=%x b=%x want %x(err %d) got %x(err %d)\n", name, a, b, want, we, got, gerr);
  }
}

int main(int argc, char **argv) {
  const char *m = argv[1];
  if (!strcmp(m, "int")) {   /* div_iter vs C / and %, 32- and 64-bit */
    long n = atol(argv[2]);
    for (long i = 0; i < n; i++) {
      int w = (i & 1) ? 64 : 32;
      uint64_t mask = w == 64 ? ~0ULL : 0xFFFFFFFFULL;
      uint64_t a = xr() >> (xr() % 64), b = xr() >> (xr() % 64);
      if (i % 7 == 0) b = xr() % 5; if (i % 11 == 0) a = mask - (xr() % 3);
      a &= mask; b &= mask; if (!b) b = 1;
      uint64_t q, r; div_iter(a, b, w, &q, &r); checked++;
      if (q != a / b || r != a % b) { bad++; if (bad < 10) printf("int w=%d a=%llx b=%llx q=%llx r=%llx\n", w, a, b, q, r); }
    }
    printf("int checked %ld mismatches %ld\n", checked, bad); return bad != 0;
  }
  int f32 = strstr(m, "32") != NULL;
  int EXP = f32 ? 8 : 5, FRAC = f32 ? 23 : 10, W = EXP + FRAC + 1;
  uint32_t EM = (1u << EXP) - 1;
  uint64_t total = 1ULL << W;
  uint64_t lo = 0, hi = total;
  if (argc > 3) { int idx = atoi(argv[2]), np = atoi(argv[3]); lo = total * idx / np; hi = total * (idx + 1) / np; }
  if (!strncmp(m, "rsqrt", 5) || !strncmp(m, "rcp", 3)) {
    int is_rcp = m[1] == 'c';
    uint32_t one = f32 ? 0x3F800000 : 0x3C00;
    for (uint64_t u = lo; u < hi; u++) {
      uint32_t a = (uint32_t)u; int err;
      uint32_t e = (a >> FRAC) & EM;
      int nonfinite = e == EM;
      double x = nonfinite ? 0 : decode(EXP, FRAC, a), r = 0; int inf = 0;
      if (is_rcp) {
        uint32_t got = fdiv(EXP, FRAC, one, a, &err);
        int werr = nonfinite || x == 0;
        if (!werr) inf = ref(EXP, FRAC, 1, 1.0, x, &r);
        cmp("rcp", EXP, FRAC, a, 0, got, err, werr, inf, r);
      } else {
        uint32_t got = rsqrt(EXP, FRAC, a, &err);
        int werr = nonfinite || x <= 0;
        if (!werr) inf = ref(EXP, FRAC, 0, x, 0, &r);
        cmp("rsqrt", EXP, FRAC, a, 0, got, err, werr, inf, r);
      }
    }
  } else { /* div: random pairs plus edge values */
    uint32_t edges[] = {0, 1, 2, 3, EM << FRAC, (EM << FRAC) - 1, 1u << FRAC, (1u << FRAC) - 1, (1u << FRAC) + 1,
                        ((uint32_t)(EM / 2) << FRAC), ((uint32_t)(EM / 2) << FRAC) + 1, ((uint32_t)(EM / 2) << FRAC) - 1};
    int ne = sizeof edges / sizeof edges[0];
    long n = atol(argc > 2 ? argv[2] : "20000000");
    for (long i = 0; i < n; i++) {
      uint32_t a, b;
      if (i < ne * ne * 4) { a = edges[(i / 4) % ne] | ((i & 1) << (W - 1)); b = edges[(i / 4 / ne) % ne] | ((i & 2) << (W - 2)); }
      else { a = (uint32_t)xr() & ((1u << W) - 1); b = (uint32_t)xr() & ((1u << W) - 1);
             if (i % 3 == 0) b = (b & ~(EM << FRAC)) | ((uint32_t)(xr() % 3) << FRAC); /* subnormal-ish divisors */ }
      int err; uint32_t got = fdiv(EXP, FRAC, a, b, &err);
      uint32_t ea = (a >> FRAC) & EM, eb = (b >> FRAC) & EM;
      int werr = ea == EM || eb == EM;
      double x = werr ? 0 : decode(EXP, FRAC, a), y = werr ? 0 : decode(EXP, FRAC, b), r = 0; int inf = 0;
      if (!werr && y == 0) werr = 1;
      if (!werr) inf = ref(EXP, FRAC, 1, x, y, &r);
      cmp("div", EXP, FRAC, a, b, got, err, werr, inf, r);
    }
  }
  printf("%s checked %ld mismatches %ld\n", m, checked, bad);
  return bad != 0;
}
