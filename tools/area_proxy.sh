#!/bin/bash
# Relative logic-area proxy with yosys (memories are black boxes).
# Usage: tools/area_proxy.sh <rtl.v> <mem_model.v>... 
# Prints flop count, generic gate count and an area estimate calibrated against the V1 DC report.
rtl=$1; shift
mems="$@"
cat > /tmp/area_proxy.ys <<YS
read_verilog -lib $mems
read_verilog -sv $rtl
hierarchy -top JPEGLENC
synth -top JPEGLENC -flatten -run :fine
synth -top JPEGLENC -flatten -run fine:
abc -g AND,NAND,OR,NOR,XOR,XNOR,ANDNOT,ORNOT,MUX
opt_clean
tee -o /tmp/area_proxy.stat stat
YS
yosys -q -l /tmp/area_proxy.log /tmp/area_proxy.ys > /dev/null 2>&1
python3 - <<'PY'
import re
s=open('/tmp/area_proxy.stat').read()
cells={}
for m in re.finditer(r'^\s+(\S+)\s+(\d+)\s*$', s, re.M):
    cells[m.group(1)]=int(m.group(2))
ff=sum(v for k,v in cells.items() if 'DFF' in k.upper() or k=='DFF')
# generic gate equivalents (NAND2 = 1): inverter .6, and/or 1.3, xor 2.4, mux 2.2
w={'$_NOT_':0.6,'$_AND_':1.3,'$_NAND_':1.0,'$_OR_':1.3,'$_NOR_':1.0,'$_XOR_':2.4,'$_XNOR_':2.4,'$_ANDNOT_':1.3,'$_ORNOT_':1.3,'$_MUX_':2.2}
ge=sum(cells.get(k,0)*v for k,v in w.items())
# calibration on V1 (DC: comb 237735 um2, seq 158559 um2 for 2955 flops)
K_COMB = 237735.0 / 30167.0     # um2 per gate-eq, fitted on V1 (see tools/README)
K_FF   = 158559.0 / 2955.0
print("flops=%d gate_eq=%.0f  est_logic_area=%.0f um2 (comb %.0f + seq %.0f)" % (ff, ge, ge*K_COMB + ff*K_FF, ge*K_COMB, ff*K_FF))
open('/tmp/area_proxy.res','w').write("%d %.0f\n"%(ff,ge))
PY
