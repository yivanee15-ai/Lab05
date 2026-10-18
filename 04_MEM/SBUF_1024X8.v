`timescale 1ns/10ps
// Behavioural stand-in with the same ports as the Artisan RA1SH model:
// Q, CLK, CEN (active low), WEN (active low = write), A, D, OEN (active low).
// Replace by the generated memory (same module name) for synthesis.
module SBUF_1024X8 (Q, CLK, CEN, WEN, A, D, OEN);
    parameter BITS = 8;
    parameter word_depth = 1024;
    parameter addr_width = 10;
    output [BITS-1:0] Q;
    input CLK, CEN, WEN, OEN;
    input [addr_width-1:0] A;
    input [BITS-1:0] D;
    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] dq;
    always @(posedge CLK) begin
        if (!CEN) begin
            if (!WEN) begin mem[A] <= D; dq <= D; end
            else dq <= mem[A];
        end
    end
    assign Q = OEN ? {BITS{1'bz}} : dq;
endmodule
