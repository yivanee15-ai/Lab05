`timescale 1ns/10ps
// Behavioural stand-in for the memaker single-port SRAM SBUF_1024X8 (1024x8).
// Replace by the memaker output (same module name and ports) for synthesis.
module SBUF_1024X8 (A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, DO0, DO1, DO2, DO3, DO4, DO5, DO6, DO7, DI0, DI1, DI2, DI3, DI4, DI5, DI6, DI7, CK, WEB, OE, CS);
    input A0, A1, A2, A3, A4, A5, A6, A7, A8, A9;
    input DI0, DI1, DI2, DI3, DI4, DI5, DI6, DI7;
    output DO0, DO1, DO2, DO3, DO4, DO5, DO6, DO7;
    input CK, WEB, OE, CS;
    wire [9:0] addr = {A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};
    wire [7:0] din  = {DI7, DI6, DI5, DI4, DI3, DI2, DI1, DI0};
    reg  [7:0] mem [0:1023];
    reg  [7:0] dq;
    always @(posedge CK) begin
        if (CS) begin
            if (!WEB) begin mem[addr] <= din; dq <= din; end
            else dq <= mem[addr];
        end
    end
    wire [7:0] dout = OE ? dq : 8'bz;
    assign DO0 = dout[0];
    assign DO1 = dout[1];
    assign DO2 = dout[2];
    assign DO3 = dout[3];
    assign DO4 = dout[4];
    assign DO5 = dout[5];
    assign DO6 = dout[6];
    assign DO7 = dout[7];
endmodule
