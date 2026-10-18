#!/bin/bash
# Run one design version against the generated vectors (PATTERN_v1, golden from files) in parallel chunks.
# Usage: sim/run_suite.sh <version-dir e.g. versions/v2> [input.txt golden.txt] [chunks]
# Prints per-run status and the total latency over all images.
cd "$(dirname "$0")/.."
dir=${1%/}; tag=$(basename $dir)
inp=${2:-out/input.txt}; gold=${3:-out/golden.txt}; chunks=${4:-4}
mkdir -p sim/build
iverilog -g2012 -DRTL -o sim/build/$tag.vvp 00_TESTBED/TESTBED_iverilog.v 00_TESTBED/PATTERN_v1.v $dir/mem/*.v $dir/JPEGLENC.v || exit 1
n=$(head -1 $inp | tr -d ' \r\n')
per=$(( (n + chunks - 1) / chunks ))
rm -f sim/build/${tag}_log_*.txt
for ((c=0; c<chunks; c++)); do
  s=$((c*per)); cnt=$per; [ $((s+cnt)) -gt $n ] && cnt=$((n-s)); [ $cnt -le 0 ] && continue
  vvp sim/build/$tag.vvp +INPUT=$inp +GOLDEN=$gold +PATTERNS=$cnt +START=$s > sim/build/${tag}_log_$c.txt 2>&1 &
done
wait
python3 - $tag <<'PY'
import re, glob, sys
tag = sys.argv[1]
rows = []; ok = 0; bad = []
for f in sorted(glob.glob('sim/build/%s_log_*.txt' % tag)):
    t = open(f).read()
    if 'ALL_PATTERNS_PASS' in t: ok += 1
    else: bad.append(f)
    for m in re.finditer(r'PATTERN_PASS frame=(\d+) bytes=(\d+) latency=(\d+)', t):
        rows.append(tuple(map(int, m.groups())))
rows.sort()
tl = sum(r[2] for r in rows); tb = sum(r[1] for r in rows)
print("%s: images=%d chunks_ok=%d  total_bytes=%d total_latency=%d (overhead %.1f%%)" % (tag, len(rows), ok, tb, tl, 100.0 * (tl - tb) / max(tl, 1)))
if bad:
    print("FAILED chunks:", bad)
    for f in bad: print(open(f).read()[-600:])
PY
