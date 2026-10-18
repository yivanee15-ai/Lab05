#!/usr/bin/env python3
"""Area breakdown by input cone with yosys: for each named group of signals, count the generic gates in the
combinational fan-in cone (stopping at flop outputs/inputs), plus the flops whose D input is in that group.
Usage: tools/area_cones.py <rtl.v> <mem.v>... ; edit GROUPS below for the design being measured."""
import subprocess, sys, re, os
rtl = sys.argv[1]; mems = sys.argv[2:]

GROUPS = {
 'row_engine+flags': ['rw0','rw1','rw2','rw3','rw4','rw5','rw6','rw7','big_row'],
 'R_readmux':        ['rc0','rc1','rc2','rc3','rc4','rc5','rc6','rc7'],
 'preadd':           ['pa0','pa1','pa2','pa3'],
 'R_read+MAC':       ['mac_sum'],
 'quant_stage1':     ['a_rnd','d_val'],
 'quantiser':        ['q_val','q_rnd'],
 'entropy_fields':   ['fld_bits','fld_len','fsz','famp','fcode'],
 'es_ctrl':          ['es_stall','run_in','need_zrl'],
 'packer':           ['tmp','pk_word','fl_word','ob_wdat','tcnt','rsh'],
 'scheduler':        ['nx','zz_nx','gt','mask'],
 'out_stage':        ['cur_byte','cur_fin','rd_issue','has_more'],
}
def with_keep(path):
    import re
    txt = open(path).read()
    names = set(n for g in GROUPS.values() for n in g)
    out = []
    for line in txt.split('\n'):
        m = re.match(r'^(\s*)(wire)\b(.*)$', line)
        if m:
            decl = m.group(3)
            # declared name = identifier right before '=' or ';' or ','
            nm = re.search(r'([A-Za-z_][A-Za-z_0-9]*)\s*(=|;|,)', decl)
            if nm and nm.group(1) in names:
                line = m.group(1) + '(* keep *) wire' + decl
        out.append(line)
    open('/tmp/area_keep.v', 'w').write('\n'.join(out))
    return '/tmp/area_keep.v'
rtl = with_keep(rtl)
WEIGHT = {'$_NOT_':0.6,'$_AND_':1.3,'$_NAND_':1.0,'$_OR_':1.3,'$_NOR_':1.0,'$_XOR_':2.4,'$_XNOR_':2.4,'$_ANDNOT_':1.3,'$_ORNOT_':1.3,'$_MUX_':2.2}
K_COMB = 237735.0 / 30167.0
base = "read_verilog -lib %s\nread_verilog -sv %s\nhierarchy -top JPEGLENC\nsynth -top JPEGLENC -flatten\nabc -g AND,NAND,OR,NOR,XOR,XNOR,ANDNOT,ORNOT,MUX\nopt_clean\n" % (' '.join(mems), rtl)
res = {}
for name, sigs in GROUPS.items():
    sel = ' '.join('w:%s' % s for s in sigs)
    script = base + "select -set start %s\nselect -set cone @start %%ci*:-$_DFF_P_[Q]:-$_DFFE_PP_[Q]:-$_DFF_PN0_[Q]:-$_DFFE_PN0P_[Q]:-$_DFFE_PN_[Q]:-$_DFFE_PN1P_[Q]:-$_DFF_PN1_[Q]:-$_SDFFCE_PN0N_[Q]:-$_DFFE_PN0N_[Q]\nselect -count @cone\ntee -o /tmp/cone_%s.txt stat @cone\n" % (sel, name.replace('+','_'))
    open('/tmp/cone.ys','w').write(script)
    subprocess.run(['yosys','-q','/tmp/cone.ys'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try: s = open('/tmp/cone_%s.txt' % name.replace('+','_')).read()
    except: s = ''
    cells = {m.group(1): int(m.group(2)) for m in re.finditer(r'^\s+(\S+)\s+(\d+)\s*$', s, re.M)}
    ge = sum(cells.get(k,0)*v for k,v in WEIGHT.items())
    res[name] = ge
    print("%-20s gate_eq=%7.0f  ~%7.0f um2" % (name, ge, ge*K_COMB))
