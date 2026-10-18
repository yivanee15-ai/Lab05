`timescale 1ns/10ps

module TESTBED;

wire        clk;
wire        rst_n;
wire        in_valid;
wire [7:0]  in_data;
wire        out_valid;
wire [7:0]  out_data;

JPEGLENC u_JPEGLENC(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .in_data(in_data),
    .out_valid(out_valid),
    .out_data(out_data)
);

PATTERN u_PATTERN(
    .clk(clk),
    .rst_n(rst_n),
    .in_valid(in_valid),
    .in_data(in_data),
    .out_valid(out_valid),
    .out_data(out_data)
);

endmodule
