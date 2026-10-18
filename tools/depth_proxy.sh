#!/bin/bash
# Longest combinational path (in gate levels after abc mapping) of a design, with yosys `ltp`.
# Usage: tools/depth_proxy.sh <rtl.v> <mem.v>...      (memories are black boxes)
rtl=$1; shift
cat > /tmp/depth.ys <<YS
read_verilog -lib $@
read_verilog -sv $rtl
hierarchy -top JPEGLENC
synth -top JPEGLENC -flatten
abc -g AND,NAND,OR,NOR,XOR,XNOR,ANDNOT,ORNOT,MUX
opt_clean
tee -o /tmp/depth.txt ltp -noff
YS
yosys -q /tmp/depth.ys > /dev/null 2>&1
grep -E "Longest topological path" /tmp/depth.txt
