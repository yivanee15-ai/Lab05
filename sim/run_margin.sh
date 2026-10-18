#!/bin/bash
# Hazard/margin check of the 2-stripe-buffer designs on a vector set (4 parallel chunks).
# Usage: sim/run_margin.sh <version-dir> [input golden]
cd "$(dirname "$0")/.."
dir=${1%/}; tag=$(basename $dir)
inp=${2:-out/input.txt}; gold=${3:-out/golden.txt}
tb=sim/build/${tag}_TB_margin.v
if grep -q "img_end_r" $dir/JPEGLENC.v; then sed 's/u_JPEGLENC.img_end)/u_JPEGLENC.img_end_r)/' sim/dbg/TB_margin.v > $tb; else cp sim/dbg/TB_margin.v $tb; fi
iverilog -g2012 -DRTL -o sim/build/${tag}_m.vvp $tb 00_TESTBED/PATTERN_v1.v $dir/mem/*.v $dir/JPEGLENC.v || exit 1
n=$(head -1 $inp | tr -d ' \r\n'); per=$(( (n + 3) / 4 ))
for c in 0 1 2 3; do
  s=$((c*per)); cnt=$per; [ $((s+cnt)) -gt $n ] && cnt=$((n-s)); [ $cnt -le 0 ] && continue
  vvp sim/build/${tag}_m.vvp +INPUT=$inp +GOLDEN=$gold +PATTERNS=$cnt +START=$s > sim/build/${tag}_m_$c.txt 2>&1 &
done
wait
cat sim/build/${tag}_m_*.txt | grep MARGIN | awk '{split($3,a,"="); split($4,b,"="); if (a[2]<m||m=="") m=a[2]; v+=b[2]; n++} END{print "images",n,"min_slack",m,"violations",v}'
