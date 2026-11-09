"""REDUCE.SUM: RTL heap tree over NUM_LANES (with 'present' pass-through) versus the
spec's logical tree over next_pow2(VL) positions. F32 adds are emulated exactly."""
import random, struct, gmpy2
from gmpy2 import mpfr
random.seed(2)
ctx = gmpy2.context(precision=24, emin=-148, emax=128, subnormalize=True)
def f32_add(x, y):   # x, y are F32 bit patterns
    fx, fy = [struct.unpack('<f', struct.pack('<I', v))[0] for v in (x, y)]
    with gmpy2.context(ctx):
        r = mpfr(fx) + mpfr(fy)
    if r == 0:   # IEEE RNE: exact zero sum is +0 unless both are -0
        return 0x80000000 if (x == 0x80000000 and y == 0x80000000) else 0
    return struct.unpack('<I', struct.pack('<f', float(r)))[0]

def next_pow2(v):
    t = 1
    while t < v: t <<= 1
    return t

def rtl(vals, enable, vl, N):
    levels = (N - 1).bit_length() if N > 1 else 0; T = 1 << levels; ts = next_pow2(vl)
    pres = [0] * (2 * T); s = [0] * (2 * T)
    for n in range(T):
        pres[T + n] = int(n < ts); s[T + n] = vals[n] if n < N and enable[n] else 0
    for n in range(T - 1, 0, -1):
        l, r = 2 * n, 2 * n + 1
        pres[n] = pres[l] | pres[r]
        s[n] = f32_add(s[l], s[r]) if pres[l] and pres[r] else (s[l] if pres[l] else s[r])
    return 0 if not any(enable) else s[1]

def spec(vals, enable, vl):
    if not any(enable): return 0
    P = next_pow2(vl)
    level = [vals[i] if i < vl and enable[i] else 0 for i in range(P)]
    while len(level) > 1:
        level = [f32_add(level[i], level[i + 1]) for i in range(0, len(level), 2)]
    return level[0]

pool = [0, 0x80000000, 0x3f800000, 0xbf800000, 0x00000001, 0x80000001, 0x4b800000, 0x33800000, 0x7f7fffff, 0x7f000000]
bad = 0
for it in range(20000):
    N = random.randint(1, 16); vl = random.randint(0, N)
    vals = [random.choice(pool) if random.random() < 0.6 else random.getrandbits(32) & 0xbf7fffff for _ in range(N)]
    enable = [int(i < vl and random.random() < 0.8) for i in range(N)]
    try:
        a, b = rtl(vals, enable, vl, N), spec(vals, enable, vl)
    except OverflowError:
        continue
    if a != b:
        bad += 1
        if bad < 5: print('mismatch', N, vl, [hex(v) for v in vals], enable, hex(a), hex(b))
print('reduce tree mismatches:', bad)
