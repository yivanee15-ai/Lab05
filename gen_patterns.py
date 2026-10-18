"""Generate Lab05 test vectors.

Outputs (default ./out):
  input.txt      decimal format used by PATTERN_v1.v   (N, then N*16384 pixels)
  golden.txt     decimal format used by PATTERN_v1.v   (N, then count + bytes per image)
  input_hex.txt  hex format used by PATTERN_v2.v       (LAB05_HEX_V1 N, then 2-digit hex pixels)
  golden_hex.txt informational: golden bytes in hex, one image per line

Usage: python3 gen_patterns.py [--random N] [--seed S] [--out DIR]
"""
import argparse
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from jpeglenc_model import encode_image  # noqa: E402
from tables import EXAMPLE_ROW, NEG_TIE_ROW, COSINE  # noqa: E402

CONST_LEVELS = [0, 1, 7, 8, 63, 64, 88, 96, 127, 128, 129, 136, 200, 247, 254, 255]


def clamp(v):
    return 0 if v < 0 else 255 if v > 255 else int(v)


def make(fn):
    return [clamp(fn(i & 127, i >> 7)) for i in range(16384)]


def blk(x, y):
    return (y >> 3) * 16 + (x >> 3)


def directed():
    imgs = []
    names = []

    def add(name, img):
        names.append(name)
        imgs.append(img)

    for v in CONST_LEVELS:
        add(f"const_{v}", [v] * 16384)
    add("block_id_levels", make(lambda x, y: blk(x, y)))
    add("block_checker", make(lambda x, y: 255 if blk(x, y) & 1 else 0))
    add("pixel_checker", make(lambda x, y: 255 if (x ^ y) & 1 else 0))
    add("vertical_stripes", make(lambda x, y: 255 if x & 1 else 0))
    add("horizontal_stripes", make(lambda x, y: 255 if y & 1 else 0))
    add("half_split_x", make(lambda x, y: 0 if x < 64 else 255))
    add("half_split_y", make(lambda x, y: 255 if y < 64 else 0))
    add("ramp_x", make(lambda x, y: x * 2))
    add("ramp_y", make(lambda x, y: y * 2))
    add("ramp_diag", make(lambda x, y: x + y))
    add("local_ramp_x", make(lambda x, y: (x & 7) * 32))
    add("local_ramp_y", make(lambda x, y: (y & 7) * 32))
    add("impulse_first", make(lambda x, y: 255 if (x, y) == (0, 0) else 128))
    add("impulse_last", make(lambda x, y: 0 if (x, y) == (127, 127) else 128))
    add("diag_line", make(lambda x, y: 255 if (x & 7) == (y & 7) else 0))
    add("corner_dot", make(lambda x, y: 255 if (x & 7) == 7 and (y & 7) == 7 else 0))
    add("quadrant_xor", make(lambda x, y: 255 if ((x & 7) < 4) ^ ((y & 7) < 4) else 0))
    add("col0_bright", make(lambda x, y: 255 if (x & 7) == 0 else 128))
    add("row7_dark", make(lambda x, y: 0 if (y & 7) == 7 else 128))
    add("circle", make(lambda x, y: 220 if (x - 64) ** 2 + (y - 64) ** 2 < 40 ** 2 else 40))

    # 64 DCT basis functions, both signs, two amplitudes
    def basis(sign, amp):
        def f(x, y):
            b = blk(x, y)
            u, v = b & 7, (b >> 3) & 7
            val = COSINE[u * 8 + (x & 7)] * COSINE[v * 8 + (y & 7)] * amp / 65536
            return 128 + sign * val
        return f
    add("dct_basis_pos_64", make(basis(+1, 64)))
    add("dct_basis_neg_96", make(basis(-1, 96)))
    add("dct_basis_small_4", make(basis(+1, 4)))

    # official three-block trace image (blocks 0..2) and negative-half tie fixtures
    def trace(x, y):
        b = blk(x, y)
        lx, ly = x & 7, y & 7
        if b == 0:
            return 88
        if b == 1:
            return (EXAMPLE_ROW[ly] >> ((7 - lx) * 8)) & 255
        if b == 2:
            return (EXAMPLE_ROW[8 + ly] >> ((7 - lx) * 8)) & 255
        return 128
    add("official_trace", make(trace))

    tie_blocks = [0, 1, 2, 15, 16, 17, 127, 128, 129, 239, 240, 241, 253, 254, 255]

    def tie(x, y):
        b = blk(x, y)
        if b in tie_blocks:
            i = tie_blocks.index(b)
            return (NEG_TIE_ROW[i * 8 + (y & 7)] >> ((7 - (x & 7)) * 8)) & 255
        return 128
    add("negative_half_ties", make(tie))
    return names, imgs


def smooth():
    return [
        ("smooth_x", make(lambda x, y: 255 - x * 2)),
        ("smooth_xy", make(lambda x, y: x + 127 - y)),
        ("smooth_radial", make(lambda x, y: ((x - 64) ** 2 + (y - 64) ** 2) >> 5)),
        ("smooth_wave", make(lambda x, y: 128 + 100 * math.sin(x / 9.0) * math.cos(y / 13.0))),
    ]


def randoms(n, rng):
    out = []
    for k in range(n):
        kind = k % 4
        if kind == 0:
            img = [rng.randrange(256) for _ in range(16384)]
            name = "rand_full"
        elif kind == 1:
            img = [128 + rng.randrange(-2, 3) for _ in range(16384)]
            name = "rand_low_noise"
        elif kind == 2:
            img = [255 if rng.random() < 0.5 else 0 for _ in range(16384)]
            name = "rand_binary"
        else:
            base = [rng.randrange(256) for _ in range(256)]
            img = [clamp(base[blk(i & 127, i >> 7)] + rng.randrange(-6, 7)) for i in range(16384)]
            name = "rand_block_tone"
        out.append((f"{name}_{k}", img))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--random", type=int, default=24)
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--out", default="out")
    a = ap.parse_args()
    rng = random.Random(a.seed)

    names, imgs = directed()
    for n, im in smooth() + randoms(a.random, rng):
        names.append(n)
        imgs.append(im)

    os.makedirs(a.out, exist_ok=True)
    goldens = [encode_image(im) for im in imgs]
    with open(os.path.join(a.out, "input.txt"), "w") as f:
        f.write(f"{len(imgs)}\n")
        for im in imgs:
            f.write("\n".join(" ".join(str(p) for p in im[r * 128:(r + 1) * 128]) for r in range(128)) + "\n")
    with open(os.path.join(a.out, "golden.txt"), "w") as f:
        f.write(f"{len(imgs)}\n")
        for g in goldens:
            f.write(f"{len(g)}\n" + " ".join(str(b) for b in g) + "\n")
    with open(os.path.join(a.out, "input_hex.txt"), "w") as f:
        f.write(f"LAB05_HEX_V1 {len(imgs)}\n")
        for im in imgs:
            f.write("\n".join(" ".join(f"{p:02x}" for p in im[r * 128:(r + 1) * 128]) for r in range(128)) + "\n")
    with open(os.path.join(a.out, "golden_hex.txt"), "w") as f:
        for n, g in zip(names, goldens):
            f.write(f"{n} {len(g)} {g.hex()}\n")
    print(f"{len(imgs)} images -> {a.out}/")
    for i, (n, g) in enumerate(zip(names, goldens)):
        print(f"  {i:3d} {n:22s} {len(g):6d} bytes")
    print(f"total bytes {sum(len(g) for g in goldens)}, max {max(len(g) for g in goldens)}")


if __name__ == "__main__":
    main()
