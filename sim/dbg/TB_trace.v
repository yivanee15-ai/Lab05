`timescale 1ns/10ps
// Debug trace for the tail latency of the first image (hierarchical references into the V1 design).
module TESTBED;
wire clk, rst_n, in_valid, out_valid; wire [7:0] in_data, out_data;
JPEGLENC u_JPEGLENC(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
PATTERN u_PATTERN(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
integer t0; integer cyc; integer nblk_row, nblk_col; reg seen_first;
initial begin t0 = -1; cyc = 0; nblk_row = 0; nblk_col = 0; seen_first = 0; end
always @(posedge clk) begin
    cyc <= cyc + 1;
    if (u_JPEGLENC.in_done && t0 < 0) t0 = cyc;
    if (u_JPEGLENC.start_ok && t0 >= 0 && 0) ;
    if (u_JPEGLENC.start_ok) begin nblk_row = nblk_row + 1; if (nblk_row > 240 || nblk_row < 6) $display("[%0d] row-stage block start #%0d bx=%0d", cyc - t0, nblk_row, u_JPEGLENC.nbx); end
    if (u_JPEGLENC.c_fin && u_JPEGLENC.adv) begin nblk_col = nblk_col + 1; if (nblk_col > 250 || nblk_col < 3) $display("[%0d] col-stage block done #%0d", cyc - t0, nblk_col); end
    if (u_JPEGLENC.f_last) $display("[%0d] last field", cyc - t0);
    if (u_JPEGLENC.fl_req) $display("[%0d] flush", cyc - t0);
    if (out_valid && !seen_first) begin seen_first = 1; $display("[%0d] first out_valid", cyc - t0); end
    if (u_JPEGLENC.img_end) $display("[%0d] img_end", cyc - t0);
end
endmodule
