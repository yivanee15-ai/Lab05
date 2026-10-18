#!/bin/bash
# Usage: sim/run_iverilog_v1.sh <num_images> <start> [extra plusargs]
# Compile first with sim/compile_v1.sh; runs on disjoint ranges can go in parallel.
# Needs the vectors from `python3 gen_patterns.py` in ./out
cd "$(dirname "$0")/.."
[ -f sim/build/v1.vvp ] || sim/compile_v1.sh || exit 1
vvp sim/build/v1.vvp +INPUT=out/input.txt +GOLDEN=out/golden.txt +PATTERNS=${1:-3} +START=${2:-0} "${@:3}"
