"""Check the zero-column skip rule used by V2+: if every R(u,y) of column u lies in [-2^k_u, 2^k_u - 1]
(k = 10 9 9 10 11 11 12 12) then all eight coefficients Q(u,v) quantise to zero."""
import random, sys
sys.path.insert(0, '.')
from tables import COSINE, QUANT_SHIFT
from jpeglenc_model import round_s

K = [10, 9, 9, 10, 11, 11, 12, 12]

def q_of_column(Rcol, u):
    out = []
    for v in range(8):
        A = sum(Rcol[y] * COSINE[v * 8 + y] for y in range(8))
        out.append(round_s(round_s(A, 18), QUANT_SHIFT[v * 8 + u]))
    return out

bad = 0
rng = random.Random(1)
# 1. worst case: R = +-2^k (or 2^k-1 / -2^k) aligned with the signs of each C(v,.) row
for u in range(8):
    k = K[u]
    for v in range(8):
        for lo, hi in ((-(1 << k), (1 << k) - 1),):
            Rcol = [(hi if COSINE[v * 8 + y] > 0 else lo) for y in range(8)]
            Rcol2 = [(lo if COSINE[v * 8 + y] > 0 else hi) for y in range(8)]
            for R in (Rcol, Rcol2):
                if any(q_of_column(R, u)): bad += 1; print("FAIL worst", u, v, R)
# 2. random columns inside the range
for it in range(200000):
    u = rng.randrange(8); k = K[u]
    mode = rng.randrange(3)
    if mode == 0: R = [rng.randint(-(1 << k), (1 << k) - 1) for _ in range(8)]
    elif mode == 1: R = [rng.choice((-(1 << k), (1 << k) - 1)) for _ in range(8)]
    else: R = [rng.randint(-(1 << k), (1 << k) - 1) // (1 << rng.randrange(4)) for _ in range(8)]
    if any(q_of_column(R, u)): bad += 1; print("FAIL rand", u, R)
print("skip-bound check:", "OK" if bad == 0 else "%d failures" % bad)
# 3. how tight is it? largest |D| that can appear for small columns (must be < 2^(S-1))
for u in range(8):
    k = K[u]; worst = 0
    for v in range(8):
        R = [((1 << k) - 1 if COSINE[v * 8 + y] > 0 else -(1 << k)) for y in range(8)]
        A = sum(R[y] * COSINE[v * 8 + y] for y in range(8))
        worst = max(worst, abs(round_s(A, 18)) / float(1 << (QUANT_SHIFT[v * 8 + u] - 1)))
    print("column %d: worst |D| / zero-threshold = %.2f" % (u, worst))
