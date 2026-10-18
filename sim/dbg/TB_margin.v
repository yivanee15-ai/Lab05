`timescale 1ns/10ps
// Precise hazard monitor for the stripe buffers: a word may be overwritten only after stage 1 has read it.
// Reports, per image, the minimum number of cycles between the last read of a word and its next write.
module TESTBED;
wire clk, rst_n, in_valid, out_valid; wire [7:0] in_data, out_data;
JPEGLENC u_JPEGLENC(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
PATTERN u_PATTERN(.clk(clk),.rst_n(rst_n),.in_valid(in_valid),.in_data(in_data),.out_valid(out_valid),.out_data(out_data));
integer cyc, viol, min_slack, img, w;
integer rd_t [0:511];           // {buffer, word} -> cycle of the last read
reg     fresh [0:511];          // word holds a stripe that has not been read yet
initial begin
    cyc = 0; viol = 0; min_slack = 1000000; img = 0;
    for (w = 0; w < 512; w = w + 1) begin rd_t[w] = -1; fresh[w] = 0; end
end
wire [8:0] wr_a = {u_JPEGLENC.wbuf, u_JPEGLENC.pix_cnt[9:2]};
wire [8:0] rd_a = {u_JPEGLENC.rbuf, u_JPEGLENC.rd_addr};
always @(posedge clk) begin
    cyc <= cyc + 1;
    if (u_JPEGLENC.wr_word) begin
        if (fresh[wr_a]) viol = viol + 1;
        else if (rd_t[wr_a] >= 0 && (cyc - rd_t[wr_a]) < min_slack) min_slack = cyc - rd_t[wr_a];
        fresh[wr_a] <= 1;
    end
    if (u_JPEGLENC.r_act) begin fresh[rd_a] <= 0; rd_t[rd_a] <= cyc; end
    if (u_JPEGLENC.img_end) begin
        $display("MARGIN image=%0d min_slack=%0d violations=%0d", img, min_slack, viol);
        img = img + 1; viol = 0; min_slack = 1000000;
        for (w = 0; w < 512; w = w + 1) begin fresh[w] <= 0; rd_t[w] <= -1; end
    end
end
endmodule
