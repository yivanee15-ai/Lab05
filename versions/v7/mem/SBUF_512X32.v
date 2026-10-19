`timescale 1ns/10ps
// Behavioural stand-in for the memaker single-port SRAM SBUF_512X32 (512x32).
// Replace by the memaker output (same module name and ports) for synthesis.
module SBUF_512X32 (A0, A1, A2, A3, A4, A5, A6, A7, A8, DO0, DO1, DO2, DO3, DO4, DO5, DO6, DO7, DO8, DO9, DO10, DO11, DO12, DO13, DO14, DO15, DO16, DO17, DO18, DO19, DO20, DO21, DO22, DO23, DO24, DO25, DO26, DO27, DO28, DO29, DO30, DO31, DI0, DI1, DI2, DI3, DI4, DI5, DI6, DI7, DI8, DI9, DI10, DI11, DI12, DI13, DI14, DI15, DI16, DI17, DI18, DI19, DI20, DI21, DI22, DI23, DI24, DI25, DI26, DI27, DI28, DI29, DI30, DI31, CK, WEB, OE, CS);
    input A0, A1, A2, A3, A4, A5, A6, A7, A8;
    input DI0, DI1, DI2, DI3, DI4, DI5, DI6, DI7, DI8, DI9, DI10, DI11, DI12, DI13, DI14, DI15, DI16, DI17, DI18, DI19, DI20, DI21, DI22, DI23, DI24, DI25, DI26, DI27, DI28, DI29, DI30, DI31;
    output DO0, DO1, DO2, DO3, DO4, DO5, DO6, DO7, DO8, DO9, DO10, DO11, DO12, DO13, DO14, DO15, DO16, DO17, DO18, DO19, DO20, DO21, DO22, DO23, DO24, DO25, DO26, DO27, DO28, DO29, DO30, DO31;
    input CK, WEB, OE, CS;
    wire [8:0] addr = {A8, A7, A6, A5, A4, A3, A2, A1, A0};
    wire [31:0] din  = {DI31, DI30, DI29, DI28, DI27, DI26, DI25, DI24, DI23, DI22, DI21, DI20, DI19, DI18, DI17, DI16, DI15, DI14, DI13, DI12, DI11, DI10, DI9, DI8, DI7, DI6, DI5, DI4, DI3, DI2, DI1, DI0};
    reg  [31:0] mem [0:511];
    reg  [31:0] dq;
    always @(posedge CK) begin
        if (CS) begin
            if (!WEB) begin mem[addr] <= din; dq <= din; end
            else dq <= mem[addr];
        end
    end
    wire [31:0] dout = OE ? dq : 32'bz;
    assign DO0 = dout[0];
    assign DO1 = dout[1];
    assign DO2 = dout[2];
    assign DO3 = dout[3];
    assign DO4 = dout[4];
    assign DO5 = dout[5];
    assign DO6 = dout[6];
    assign DO7 = dout[7];
    assign DO8 = dout[8];
    assign DO9 = dout[9];
    assign DO10 = dout[10];
    assign DO11 = dout[11];
    assign DO12 = dout[12];
    assign DO13 = dout[13];
    assign DO14 = dout[14];
    assign DO15 = dout[15];
    assign DO16 = dout[16];
    assign DO17 = dout[17];
    assign DO18 = dout[18];
    assign DO19 = dout[19];
    assign DO20 = dout[20];
    assign DO21 = dout[21];
    assign DO22 = dout[22];
    assign DO23 = dout[23];
    assign DO24 = dout[24];
    assign DO25 = dout[25];
    assign DO26 = dout[26];
    assign DO27 = dout[27];
    assign DO28 = dout[28];
    assign DO29 = dout[29];
    assign DO30 = dout[30];
    assign DO31 = dout[31];
endmodule
