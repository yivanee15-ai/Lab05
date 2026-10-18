#!/bin/bash
# Run a version against PATTERN_v2's built-in 100-image suite (slow, single process).
# Usage: sim/run_v2pattern.sh <version-dir> [plusargs]
cd "$(dirname "$0")/.."
dir=${1%/}; tag=$(basename $dir); shift
mkdir -p sim/build
iverilog -g2012 -DRTL -o sim/build/${tag}_p2.vvp 00_TESTBED/TESTBED_iverilog.v 00_TESTBED/PATTERN_v2.v $dir/mem/*.v $dir/JPEGLENC.v || exit 1
vvp sim/build/${tag}_p2.vvp +BUILTIN=1 +COVERAGE=1 "$@" > sim/build/${tag}_p2.log 2>&1
tail -12 sim/build/${tag}_p2.log
