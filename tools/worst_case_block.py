"""Hill-climb for the pixel block that produces the longest entropy-coded bit string.
The DC delta is taken as the worst case (alternating very different blocks), so the number is an
estimate of the worst bytes/block; 256 * bytes/block bounds a whole image."""
import random, sys
sys.path.insert(0, '.')
from jpeglenc_model import block_coeffs, BitWriter, encode_block
from tables import ZIGZAG_ADDR

def block_img(pix):
    img = [128] * 16384
    for y in range(8):
        for x in range(8):
            img[y * 128 + x] = pix[y * 8 + x]
    return img

def bits_of(pix, prev_dc):
    img = block_img(pix)
    D, Q = block_coeffs(img, 0, 0)
    bw = BitWriter()
    dc, Z = encode_block(bw, Q, prev_dc)
    nb = len(bw.out) * 8 + bw.nbits
    return nb, dc

def search(seed, iters=30000):
    rng = random.Random(seed)
    # two alternating block types A and B to force big DC steps
    A = [rng.choice((0, 255)) for _ in range(64)]
    B = [rng.choice((0, 255)) for _ in range(64)]
    def score(A, B):
        nbA, dcA = bits_of(A, 0)
        nbB, dcB = bits_of(B, dcA)
        nbA2, _ = bits_of(A, dcB)
        return nbB + nbA2, dcA, dcB
    best, _, _ = score(A, B)
    for it in range(iters):
        T = rng.choice((A, B)); i = rng.randrange(64); old = T[i]
        T[i] = rng.choice((0, 255, rng.randrange(256)))
        s, _, _ = score(A, B)
        if s >= best: best = s
        else: T[i] = old
    return best / 2.0, A, B

if __name__ == '__main__':
    res = []
    for sd in range(int(sys.argv[1]) if len(sys.argv) > 1 else 6):
        b, A, B = search(sd)
        res.append(b)
        print("seed %d: %.1f bits/block = %.1f bytes/block -> image %.0f bytes" % (sd, b, b / 8, b / 8 * 256), flush=True)
    print("max %.1f bytes/block -> %.0f bytes/image" % (max(res) / 8, max(res) / 8 * 256))
