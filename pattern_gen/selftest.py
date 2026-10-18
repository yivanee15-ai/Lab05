"""Check the model against the official numbers embedded in PATTERN_v2.v / the handout."""
from jpeglenc_model import *
from tables import EXAMPLE_ROW

assert (round_s(25, 3), round_s(-25, 3), round_s(28, 3), round_s(-28, 3), round_s(-1024, 4)) == (3, -3, 4, -4, -64)
assert round_s(round_s(917504, 18), 3) == 1 and round_s(917504, 21) == 0

img = [128] * 16384
for y in range(8):
    for x in range(8):
        img[y * 128 + x] = 88
        img[y * 128 + 8 + x] = (EXAMPLE_ROW[y] >> ((7 - x) * 8)) & 255
        img[y * 128 + 16 + x] = (EXAMPLE_ROW[8 + y] >> ((7 - x) * 8)) & 255

# handout: constant 200 block -> Q(0,0)=36, bits 1110 100100 1010
c = [200] * 16384
D, Q = block_coeffs(c, 0, 0)
assert D[0] == 576 and Q[0] == 36 and not any(Q[1:])

bw = BitWriter(); prev = 0
for b in range(3):
    D, Q = block_coeffs(img, b, 0)
    if b == 0:
        assert Q[0] == -20 and not any(Q[1:])
    prev, Z = encode_block(bw, Q, prev)
    if b == 0:
        assert bytes(bw.out) == b"\xcb" and bw.nbits == 4 and bw.acc == 0xA
    if b == 1:
        assert (Z[0], Z[3], Z[22]) == (-15, 3, -2) and bw.nbits == 5 and bw.acc == 0x1A
exp = bytes.fromhex("CBA97E7FF3F2D5F59D9990C691903F74D750B36D0990DB771236A80303693CFA03")
assert bytes(bw.out) == exp and bw.nbits == 3 and bw.acc == 2, (bytes(bw.out).hex(), bw.nbits, bw.acc)
assert bw.finish()[-1] == 0x5F and len(bw.out) == 34
print("selftest OK: handout rounding, constant block, 3-block trace (33 bytes + 0x5F pad) all match")
