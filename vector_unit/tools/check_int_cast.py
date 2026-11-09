"""Python mirrors of vec_cast / vec_product (integer) / vec_special (integer) RTL,
compared with exact-arithmetic oracles (Fraction, gmpy2 MPFR)."""
import random, sys
from fractions import Fraction
import gmpy2
from gmpy2 import mpfr

M64 = (1 << 64) - 1
random.seed(1)
fails = {}
def fail(name, *info):
    fails[name] = fails.get(name, 0) + 1
    if fails[name] <= 5: print('FAIL', name, *info)

def sx(v, w): v &= (1 << w) - 1; return v - (1 << w) if v >> (w - 1) else v

# ---------------- fp_round mirror ----------------
def fp_round(EXP, FRAC, sign, mag, exponent, sticky_in):
    BIAS = (1 << (EXP - 1)) - 1; UNIT = 1 - BIAS - FRAC; EXP_MAX = (1 << EXP) - 1
    d = sign << (EXP + FRAC)
    if mag == 0: return d, 0
    msb = mag.bit_length() - 1
    lsb = max(msb + exponent - FRAC, UNIT); rs = lsb - exponent
    keep = (1 << (FRAC + 2)) - 1
    kept = guard = 0; sticky = sticky_in
    if rs <= 0: kept = (mag << -rs) & keep
    elif rs > 65: sticky = 1
    else:
        kept = (mag >> rs) & keep; guard = (mag >> (rs - 1)) & 1
        sticky |= int(mag & ((1 << (rs - 1)) - 1) != 0)
    r = (kept + (guard and (sticky or kept & 1))) & keep
    if r >> (FRAC + 1): r >>= 1; lsb += 1
    ef = lsb - UNIT + 1 if (r >> FRAC) & 1 else 0
    if ef >= EXP_MAX: return 0, 1
    return d | (ef << FRAC) | (r & ((1 << FRAC) - 1)), 0

# ---------------- oracles ----------------
FMT = {0: (5, 10), 1: (8, 23)}
def decode_f(bits, fmt):
    EXP, FRAC = FMT[fmt]; BIAS = (1 << (EXP - 1)) - 1
    s = bits >> (EXP + FRAC) & 1; e = bits >> FRAC & ((1 << EXP) - 1); f = bits & ((1 << FRAC) - 1)
    v = Fraction((1 << FRAC) | f, 1) * Fraction(2) ** (e - BIAS - FRAC) if e else Fraction(f) * Fraction(2) ** (1 - BIAS - FRAC)
    return -v if s else v, s

def oracle_float(val, neg_zero_sign, fmt):
    """Round exact Fraction to fmt with RNE and subnormals; returns (bits, overflow)."""
    EXP, FRAC = FMT[fmt]; BIAS = (1 << (EXP - 1)) - 1
    ctx = gmpy2.context(precision=FRAC + 1, emin=1 - BIAS - FRAC + 1, emax=BIAS + 1, subnormalize=True, round=gmpy2.RoundToNearest)
    with gmpy2.local_context(ctx):
        r = mpfr(gmpy2.mpq(val.numerator, val.denominator))
    if gmpy2.is_infinite(r): return 0, 1
    if r == 0:
        s = neg_zero_sign if val == 0 else int(val < 0)
        return s << (EXP + FRAC), 0
    fr = Fraction(int(gmpy2.mpz(r * mpfr(2) ** 200)), 1 << 200) if abs(r) < 1 else Fraction(gmpy2.mpq(r))
    fr = Fraction(gmpy2.mpq(r))
    s = int(fr < 0); fr = abs(fr)
    # encode
    e = fr.numerator.bit_length() - fr.denominator.bit_length()
    if Fraction(2) ** e > fr: e -= 1
    if e < 1 - BIAS:
        f = fr / Fraction(2) ** (1 - BIAS - FRAC); assert f.denominator == 1
        return (s << (EXP + FRAC)) | int(f), 0
    sig = fr / Fraction(2) ** (e - FRAC); assert sig.denominator == 1
    return (s << (EXP + FRAC)) | ((e + BIAS) << FRAC) | (int(sig) - (1 << FRAC)), 0

# ---------------- vec_cast mirror ----------------
LEGAL = [0, 1, 6, 7, 8, 9, 14, 15, 22, 30, 38, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 62, 63]
WIDTH = {2: 8, 3: 8, 4: 16, 5: 16, 6: 32, 7: 64}
SIGNED = {2: 1, 3: 0, 4: 1, 5: 0, 6: 1, 7: 1}
def vec_cast(a, b, sig, strict):
    ft, tt = sig >> 3 & 7, sig & 7
    ff, tf = ft <= 1, tt <= 1
    sign = mag = sexp = serr = 0
    if ft == 0:
        sign = a >> 15 & 1; e = a >> 10 & 31; mag = (int(e != 0) << 10) | (a & 1023); sexp = (1 if e == 0 else e) - 25; serr = e == 31
    elif ft == 1:
        sign = a >> 31 & 1; e = a >> 23 & 255; mag = (int(e != 0) << 23) | (a & 0x7fffff); sexp = (1 if e == 0 else e) - 150; serr = e == 255
    else:
        w = WIDTH[ft]; v = a & ((1 << w) - 1)
        if SIGNED[ft] and v >> (w - 1): sign = 1; mag = ((-v) & ((1 << w) - 1)) if w < 64 else (-v) & M64
        else: mag = v
    scale = sx(b, 32)
    scerr = scale < -255 or scale > 255 or (ff and tf and scale != 0)
    te = sexp + scale
    int_mag = 0; huge = 0
    if mag:
        if te >= 64: huge = 1
        elif te >= 0: int_mag = mag << te
        elif te >= -65:
            rs = -te; im = mag >> rs; g = mag >> (rs - 1) & 1; st = int(mag & ((1 << (rs - 1)) - 1) != 0)
            int_mag = im + (g and (st or im & 1))
    lims = {2: (127, 128), 3: (255, 0), 4: (32767, 32768), 5: (65535, 0), 6: (2**31 - 1, 2**31), 7: (2**63 - 1, 2**63)}
    lp, ln = lims.get(tt, (2**63 - 1, 2**63))
    clamp = huge or (int_mag > ln if sign else int_mag > lp)
    iv = ((-ln if sign else lp) if clamp else (-int_mag if sign else int_mag)) & M64
    err = (sig not in LEGAL) or serr or scerr
    res = sat = 0
    if not err:
        if tf:
            d, ovf = fp_round(*FMT[tt], sign, mag, te, 0); err = ovf; res = d
        else:
            sat = int(clamp); err = clamp and strict
            res = iv & ((1 << WIDTH[tt]) - 1)
    return (0 if err else res), int(bool(err)), sat

def oracle_cast(a, b, sig, strict):
    ft, tt = sig >> 3 & 7, sig & 7
    e = sx(b, 32)
    if e < -255 or e > 255 or (ft <= 1 and tt <= 1 and e != 0): return 0, 1, 0
    if ft <= 1:
        EXP, FRAC = FMT[ft]
        if (a >> FRAC & ((1 << EXP) - 1)) == (1 << EXP) - 1: return 0, 1, 0
        val, s = decode_f(a, ft)
    else:
        w = WIDTH[ft]; v = a & ((1 << w) - 1); val = Fraction(sx(v, w) if SIGNED[ft] else v); s = 0
    val = val * Fraction(2) ** e
    if tt <= 1:
        bits, ovf = oracle_float(val, s, tt)
        return (0, 1, 0) if ovf else (bits, 0, 0)
    r = round(val)   # ties to even
    w = WIDTH[tt]; lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if SIGNED[tt] else (0, (1 << w) - 1)
    sat = int(r < lo or r > hi); r = min(max(r, lo), hi)
    if sat and strict: return 0, 1, 1
    return r & ((1 << w) - 1), 0, sat

def rand_bits(w):
    k = random.random()
    if k < 0.2: return random.choice([0, 1, (1 << w) - 1, 1 << (w - 1), (1 << (w - 1)) - 1, (1 << (w - 1)) + 1])
    return random.getrandbits(w)

def rand_operand(t):
    w = {0: 16, 1: 32}.get(t, WIDTH.get(t, 64))
    v = rand_bits(w)
    return v | (random.getrandbits(64 - w) << w if random.random() < 0.1 and w < 64 else 0)   # junk high bits

def rand_exp(ff_tf):
    if ff_tf: return 0 if random.random() < 0.95 else random.choice([1, 0xFFFFFFFF])
    r = random.random()
    if r < 0.3: e = random.randint(-40, 40)
    elif r < 0.6: e = random.randint(-255, 255)
    elif r < 0.7: e = random.choice([-256, 256, -255, 255, 2**31 - 1, -2**31])
    else: e = random.randint(-70, 70)
    return e & 0xFFFFFFFF

N = int(sys.argv[1]) if len(sys.argv) > 1 else 200000
for i in range(N):
    sig = random.choice(LEGAL) if random.random() < 0.97 else random.randrange(64)
    a = rand_operand(sig >> 3 & 7); b = rand_exp((sig >> 3 & 7) <= 1 and (sig & 7) <= 1)
    strict = random.random() < 0.3
    got = vec_cast(a, b, sig, strict)
    if sig not in LEGAL:
        if not got[1]: fail('cast-illegal', sig)
        continue
    want = oracle_cast(a, b, sig, strict)
    if got != want: fail('cast', hex(a), sx(b, 32), sig, strict, 'got', got, 'want', want)
# spec witness: CAST.I64.F16 16785409, e=-24 -> 0x3c01
w = vec_cast(16785409, (-24) & 0xFFFFFFFF, 56, 0)
if w[0] != 0x3c01: fail('cast-witness', w)

# ---------------- vec_product integer mirror ----------------
def product(a, b, c, sig, strict):
    if sig == 3:
        r = sx(a, 16) * sx(b, 16) + sx(c, 32); e = not (-2**31 <= r < 2**31); return (0 if e else r & 0xFFFFFFFF), int(e), 0
    if sig == 4:
        r = sx(a, 32) * sx(b, 32) + sx(c, 64); e = not (-2**63 <= r < 2**63); return (0 if e else r & M64), int(e), 0
    if sig in (5, 6):
        p = sx(a, 32) * (sx(b, 16) if sig == 5 else b & 0xFFFF)
        cnt = c & 0xFFFFFFFF
        if cnt > 63: return 0, 1, 0
        if cnt == 0: r = p
        else:
            P = p & M64; sh = sx(P, 64) >> cnt; half = P >> (cnt - 1) & 1; st = (P & ((1 << (cnt - 1)) - 1)) != 0
            r = sh + (half and (st or sh & 1))
        ov = not (-2**31 <= r < 2**31)
        e = ov and strict
        if e: return 0, 1, 1
        r = min(max(r, -2**31), 2**31 - 1)
        return r & 0xFFFFFFFF, 0, int(ov)
    if sig == 7:
        r = sx(a, 64) * sx(b, 64); e = not (-2**63 <= r < 2**63); return (0 if e else r & M64), int(e), 0
    if sig == 8:
        r = a * b; e = r > M64; return (0 if e else r), int(e), 0

def oracle_product(a, b, c, sig, strict):
    if sig in (3, 4):
        w1, w2 = (16, 32) if sig == 3 else (32, 64)
        r = sx(a, w1) * sx(b, w1) + sx(c, w2)
        return (0, 1, 0) if not -2**(w2-1) <= r < 2**(w2-1) else (r % 2**w2, 0, 0)
    if sig in (5, 6):
        p = sx(a, 32) * (sx(b, 16) if sig == 5 else b & 0xFFFF)
        cnt = c & 0xFFFFFFFF
        if cnt > 63: return 0, 1, 0
        r = round(Fraction(p, 2 ** cnt)); sat = int(not -2**31 <= r < 2**31)
        if sat and strict: return 0, 1, 1
        return min(max(r, -2**31), 2**31 - 1) % 2**32, 0, sat
    if sig == 7:
        r = sx(a, 64) * sx(b, 64); return (0, 1, 0) if not -2**63 <= r < 2**63 else (r % 2**64, 0, 0)
    r = a * b; return (0, 1, 0) if r >= 2**64 else (r, 0, 0)

for i in range(N):
    sig = random.randint(3, 8)
    a = rand_bits(64); b = rand_bits(64)
    c = random.choice([random.randint(0, 70), rand_bits(32), 0xFFFFFFFF]) if sig in (5, 6) else rand_bits(64)
    if sig in (7, 8) and random.random() < 0.5:   # products near the 64-bit boundary
        a = rand_bits(random.randint(1, 64)); b = rand_bits(random.randint(1, 64))
    strict = random.random() < 0.3
    got, want = product(a, b, c, sig, strict), oracle_product(a, b, c, sig, strict)
    if got != want: fail('product', sig, hex(a), hex(b), hex(c), got, want)
# spec witness: product 2147483648 clamps to 2147483647 (MULSHIFT, C=0): 65536 * 32768
w = product((-65536) & 0xFFFFFFFF, 0x8000, 0, 5, 0)
if w != (0x7FFFFFFF, 0, 1): fail('mulshift-witness', w)

# ---------------- vec_special integer mirror ----------------
def special_int(a, b, sig):
    signed = not (sig & 1); wide = bool(sig >> 1 & 1)
    w = 64 if wide else 32; mask = (1 << w) - 1
    sa = signed and (a >> (w - 1) & 1); sb = signed and (b >> (w - 1) & 1)
    ma = (-a if sa else a) & mask; mb = (-b if sb else b) & mask
    lp = (1 << (w - 1)) - 1 if signed else mask; ln = 1 << (w - 1)
    qn = sa ^ sb
    mq = ma // mb if mb else 0; mr = ma % mb if mb else 0
    qov = mq > ln if qn else mq > lp
    q = (-mq if qn else mq) & mask; r = (-mr if sa else mr) & mask
    up = 2 * mr > mb or (2 * mr == mb and mq & 1)
    mrne = mq + up; rov = mrne > ln if qn else mrne > lp
    rq = (-mrne if qn else mrne) & mask
    if sig <= 7:
        e = mb == 0 or qov; return (0, 0, 1) if e else (q, r, 0)
    e = mb == 0 or rov; return (0, 0, 1) if e else (rq, 0, 0)

def oracle_special_int(a, b, sig):
    signed = not (sig & 1); w = 64 if sig >> 1 & 1 else 32
    x = sx(a, w) if signed else a & ((1 << w) - 1); y = sx(b, w) if signed else b & ((1 << w) - 1)
    lo, hi = (-(1 << (w - 1)), (1 << (w - 1)) - 1) if signed else (0, (1 << w) - 1)
    if y == 0: return 0, 0, 1
    if sig <= 7:
        q = abs(x) // abs(y) * (1 if (x < 0) == (y < 0) else -1); r = x - q * y
        if not lo <= q <= hi: return 0, 0, 1
        return q % 2**w, r % 2**w, 0
    q = round(Fraction(x, y))
    if not lo <= q <= hi: return 0, 0, 1
    return q % 2**w, 0, 0

for i in range(N):
    sig = random.randint(4, 11)
    w = 64 if sig >> 1 & 1 else 32
    a = rand_bits(w); b = rand_bits(w) if random.random() < 0.7 else random.choice([0, 1, 2, 3, (1 << w) - 1, (1 << w) - 2])
    got, want = special_int(a, b, sig), oracle_special_int(a, b, sig)
    if got != want: fail('divmod/div_rne', sig, hex(a), hex(b), got, want)
# spec examples
if special_int((-7) & 0xFFFFFFFF, 3, 4)[:2] != ((-2) & 0xFFFFFFFF, (-1) & 0xFFFFFFFF): fail('divmod-ex')
if special_int(7, 2, 9)[0] != 4 or special_int(5, 2, 9)[0] != 2: fail('div_rne-ex')

def isqrt64(v):
    op, res, one = v, 0, 1 << 62
    for _ in range(32):
        if op >= res + one: op -= res + one; res = (res >> 1) + one
        else: res >>= 1
        one >>= 2
    return res
import math
for v in [0, 1, 2, 3, 4, 10, M64, M64 - 1, 2**32 - 1, 2**62, (2**32 - 1)**2, (2**32 - 1)**2 - 1] + [random.getrandbits(random.randint(1, 64)) for _ in range(N)]:
    if isqrt64(v) != math.isqrt(v): fail('isqrt', v)

print('fails:', fails if fails else 'none')
