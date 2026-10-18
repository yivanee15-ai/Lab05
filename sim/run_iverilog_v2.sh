#!/bin/bash
# PATTERN_v2 run (golden computed inside the pattern). Input: hex file from gen_patterns.py
cd "$(dirname "$0")/.."
mkdir -p sim/build
iverilog -g2012 -DRTL -o sim/build/v2.vvp 00_TESTBED/TESTBED.v 00_TESTBED/PATTERN_v2.v \
    04_MEM/SBUF_1024X8.v 04_MEM/OBUF_4096X32.v 01_RTL/JPEGLENC.v || exit 1
vvp sim/build/v2.vvp +INPUT_FILE=out/input_hex.txt "$@"
