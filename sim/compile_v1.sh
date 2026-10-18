#!/bin/bash
cd "$(dirname "$0")/.."
mkdir -p sim/build
iverilog -g2012 -DRTL -o sim/build/v1.vvp 00_TESTBED/TESTBED_iverilog.v 00_TESTBED/PATTERN_v1.v \
    04_MEM/SBUF_1024X8.v 04_MEM/OBUF_4096X32.v 01_RTL/JPEGLENC.v
