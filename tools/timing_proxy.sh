#!/bin/bash
# Crude static timing estimate: yosys + ABC delay-driven mapping to a small library whose delays follow the
# slow-corner cell delays seen in the DC reports. Flop clk->Q (0.54 ns) and setup (0.1 ns) are added by hand.
# Usage: tools/timing_proxy.sh <rtl.v> <target_ps> <mem.v>...
rtl=$1; tgt=$2; shift 2
sed "s/{D}/$tgt/" /home/user/Lab05/tools/abc_timing.script > /tmp/abc_timing.script
cat > /tmp/tp.ys <<YS
read_verilog -lib $@
read_verilog -sv $rtl
hierarchy -top JPEGLENC
synth -top JPEGLENC -flatten
abc -liberty /home/user/Lab05/tools/crude.lib -D $tgt -script /tmp/abc_timing.script
opt_clean
tee -o /tmp/tp_stat.txt stat -liberty /home/user/Lab05/tools/crude.lib
YS
yosys -l /tmp/tp.log /tmp/tp.ys > /dev/null 2>&1
grep -E "ABC: WireLoad|ABC: Start-point|ABC: End-point|Path" /tmp/tp.log | head -12
