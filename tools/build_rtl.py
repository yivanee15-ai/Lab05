#!/usr/bin/env python3
"""Generate a JPEGLENC version (ROM tables + memory wrappers + behavioural memory models).
Usage: tools/build_rtl.py <version>        (reads tools/<version>.v.in, writes versions/<version>/)"""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from tables import COSINE, QUANT_SHIFT, ZIGZAG_ADDR, DC_LUT, AC_LUT

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

# per-version memory configuration: (module name, words, address bits, data bits)
CONFIG = {
    'v2': dict(sbuf=('SBUF_256X32', 256, 8, 32), obuf=('OBUF_4096X32', 4096, 12, 32)),
    'v3': dict(sbuf=('SBUF_256X32', 256, 8, 32), obuf=('OBUF_3072X32', 3072, 12, 32)),
    'v4': dict(sbuf=('SBUF_256X32', 256, 8, 32), obuf=('OBUF_3072X32', 3072, 12, 32)),
}

def sgn(v):
    return ('-' if v < 0 else '') + "9'sd%d" % abs(v)

def rom_cm4():
    return '\n'.join("            5'd%d: cm4 = %s;" % (v * 4 + j, sgn(COSINE[v * 8 + j])) for v in range(8) for j in range(4))

def rom_zzs():
    return '\n'.join("            6'd%d: zzs = 9'd%d;" % (i, ZIGZAG_ADDR[i] * 8 + QUANT_SHIFT[ZIGZAG_ADDR[i]]) for i in range(64))

def rom_dc():
    return '\n'.join("            4'd%d: dcrom = 21'h%05X;" % (i, (DC_LUT[i][0] << 16) | DC_LUT[i][1]) for i in range(12))

def rom_ac():
    return '\n'.join("            8'h%02X: acrom = 21'h%05X;" % (k, (v[0] << 16) | v[1]) for k, v in sorted(AC_LUT.items()))

def mask_lines():
    return '\n'.join("assign mask[%d] = bsel[%d];" % (i, ZIGZAG_ADDR[i] & 7) for i in range(1, 64))

def mask_lines_n(n):
    return '\n'.join("assign mask%d[%d] = bigf%d[%d];" % (n, i, n, ZIGZAG_ADDR[i] & 7) for i in range(1, 64))

def inst(mod, abits, dbits):
    p = []
    for i in range(abits): p.append(".A%d(addr[%d])" % (i, i))
    for i in range(dbits): p.append(".DO%d(dout[%d])" % (i, i))
    for i in range(dbits): p.append(".DI%d(di[%d])" % (i, i))
    p += [".CK(clk)", ".WEB(web)", ".OE(1'b1)", ".CS(cs)"]
    lines, cur = [], "        "
    for x in p:
        if len(cur) + len(x) > 100:
            lines.append(cur.rstrip()); cur = "        "
        cur += x + ", "
    lines.append(cur.rstrip().rstrip(','))
    return "    %s u_mem (\n%s\n    );" % (mod, '\n'.join(lines))

def model(name, words, abits, dbits):
    ports = ['A%d' % i for i in range(abits)] + ['DO%d' % i for i in range(dbits)] + ['DI%d' % i for i in range(dbits)] + ['CK', 'WEB', 'OE', 'CS']
    s = "`timescale 1ns/10ps\n// Behavioural stand-in for the memaker single-port SRAM %s (%dx%d).\n// Replace by the memaker output (same module name and ports) for synthesis.\n" % (name, words, dbits)
    s += "module %s (%s);\n" % (name, ', '.join(ports))
    s += "    input " + ', '.join('A%d' % i for i in range(abits)) + ";\n"
    s += "    input " + ', '.join('DI%d' % i for i in range(dbits)) + ";\n"
    s += "    output " + ', '.join('DO%d' % i for i in range(dbits)) + ";\n"
    s += "    input CK, WEB, OE, CS;\n"
    s += "    wire [%d:0] addr = {%s};\n" % (abits - 1, ', '.join('A%d' % i for i in range(abits - 1, -1, -1)))
    s += "    wire [%d:0] din  = {%s};\n" % (dbits - 1, ', '.join('DI%d' % i for i in range(dbits - 1, -1, -1)))
    s += "    reg  [%d:0] mem [0:%d];\n    reg  [%d:0] dq;\n" % (dbits - 1, words - 1, dbits - 1)
    s += "    always @(posedge CK) begin\n        if (CS) begin\n            if (!WEB) begin mem[addr] <= din; dq <= din; end\n            else dq <= mem[addr];\n        end\n    end\n"
    s += "    wire [%d:0] dout = OE ? dq : %d'bz;\n" % (dbits - 1, dbits)
    for i in range(dbits): s += "    assign DO%d = dout[%d];\n" % (i, i)
    s += "endmodule\n"
    return s

def main():
    ver = sys.argv[1]
    cfg = CONFIG[ver]
    t = open(os.path.join(ROOT, 'tools', ver + '.v.in')).read()
    sb, ob = cfg['sbuf'], cfg['obuf']
    rep = {
        '@@CM4@@': rom_cm4(), '@@ZZS@@': rom_zzs(), '@@DCROM@@': rom_dc(), '@@ACROM@@': rom_ac(),
        '@@MASK@@': mask_lines(), '@@MASK0@@': mask_lines_n(0), '@@MASK1@@': mask_lines_n(1),
        '@@SBUF_INST@@': inst(sb[0], sb[2], sb[3]), '@@OBUF_INST@@': inst(ob[0], ob[2], ob[3]),
        '@@OB_AMSB@@': str(ob[2] - 1),
    }
    for k, v in rep.items():
        t = t.replace(k, v)
    outdir = os.path.join(ROOT, 'versions', ver)
    os.makedirs(outdir, exist_ok=True)
    open(os.path.join(outdir, 'JPEGLENC.v'), 'w').write(t)
    memdir = os.path.join(outdir, 'mem')
    os.makedirs(memdir, exist_ok=True)
    for name, words, ab, db in (sb, ob):
        open(os.path.join(memdir, name + '.v'), 'w').write(model(name, words, ab, db))
    open(os.path.join(outdir, 'filelist.f'), 'w').write('TESTBED.v\n' + ''.join('../04_MEM/%s.v\n' % m[0] for m in (sb, ob)))
    print("built versions/%s" % ver)

main()
