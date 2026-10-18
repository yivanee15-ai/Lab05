`timescale 1ns/10ps
module TESTBED;
wire clk, rst_n, in_valid, out_valid; wire [7:0] in_data, out_data;
JPEGLENC u_JPEGLENC(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
PATTERN u_PATTERN(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
integer t0, cyc, nrow, ncol; reg seen_first, seen_last_in;
initial begin t0 = -1; cyc = 0; nrow = 0; ncol = 0; seen_first = 0; end
always @(posedge clk) begin
    cyc <= cyc + 1;
    if (u_JPEGLENC.in_done && t0 < 0) t0 = cyc;
    if (u_JPEGLENC.start_ok && t0 >= 0) $display("[%0d] stage1 block start bx=%0d", cyc - t0, u_JPEGLENC.nbx);
    if (u_JPEGLENC.start_ok && t0 < 0) nrow = nrow + 1;
    if (u_JPEGLENC.c_run && u_JPEGLENC.slot_end && u_JPEGLENC.adv && t0 >= 0) $display("[%0d] stage2 block end", cyc - t0);
    if (u_JPEGLENC.f_last) $display("[%0d] last field", cyc - t0);
    if (u_JPEGLENC.fl_req) $display("[%0d] flush", cyc - t0);
    if (out_valid && !seen_first) begin seen_first = 1; $display("[%0d] first out_valid", cyc - t0); end
    if (u_JPEGLENC.img_end) $display("[%0d] img_end", cyc - t0);
end
endmodule
