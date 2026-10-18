"""Bit-exact reference model of JPEGLENC (Lab05) built from the handout."""
from tables import COSINE, QUANT_SHIFT, ZIGZAG_ADDR, DC_LUT, AC_LUT


def round_s(a, n):
    mag = (abs(a) + (1 << (n - 1))) >> n
    return -mag if a < 0 else mag


def size_of(a):
    return abs(a).bit_length()


class BitWriter:
    def __init__(self):
        self.out = bytearray()
        self.acc = 0
        self.nbits = 0

    def put(self, value, length):
        for i in range(length - 1, -1, -1):
            self.acc = (self.acc << 1) | ((value >> i) & 1)
            self.nbits += 1
            if self.nbits == 8:
                self.out.append(self.acc)
                self.acc = 0
                self.nbits = 0

    def finish(self):
        if self.nbits:
            pad = 8 - self.nbits
            self.put((1 << pad) - 1, pad)
        return bytes(self.out)


def put_amplitude(bw, a, size):
    if size:
        bw.put(a if a >= 0 else (1 << size) - 1 + a, size)


def block_coeffs(img, bx, by):
    """Return (D, Q) as 64-entry lists indexed v*8+u."""
    blk = [[img[(by * 8 + y) * 128 + bx * 8 + x] - 128 for x in range(8)] for y in range(8)]
    # R[y][u] = sum_x P'(x,y) C(u,x)
    R = [[sum(blk[y][x] * COSINE[u * 8 + x] for x in range(8)) for u in range(8)] for y in range(8)]
    D = [0] * 64
    Q = [0] * 64
    for v in range(8):
        for u in range(8):
            A = sum(R[y][u] * COSINE[v * 8 + y] for y in range(8))
            d = round_s(A, 18)
            D[v * 8 + u] = d
            Q[v * 8 + u] = round_s(d, QUANT_SHIFT[v * 8 + u])
    return D, Q


def encode_block(bw, Q, prev_dc):
    Z = [Q[ZIGZAG_ADDR[i]] for i in range(64)]
    delta = Z[0] - prev_dc
    s = size_of(delta)
    ln, code = DC_LUT[s]
    bw.put(code, ln)
    put_amplitude(bw, delta, s)
    run = 0
    for k in range(1, 64):
        if Z[k] == 0:
            run += 1
            continue
        while run >= 16:
            ln, code = AC_LUT[0xF0]
            bw.put(code, ln)
            run -= 16
        s = size_of(Z[k])
        ln, code = AC_LUT[(run << 4) | s]
        bw.put(code, ln)
        put_amplitude(bw, Z[k], s)
        run = 0
    if run > 0:
        ln, code = AC_LUT[0x00]
        bw.put(code, ln)
    return Z[0], Z


def encode_image(img):
    assert len(img) == 16384
    bw = BitWriter()
    prev = 0
    for b in range(256):
        _, Q = block_coeffs(img, b & 15, b >> 4)
        prev, _ = encode_block(bw, Q, prev)
    return bw.finish()
