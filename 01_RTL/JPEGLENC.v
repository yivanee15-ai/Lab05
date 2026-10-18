//############################################################################
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//   (C) Copyright Laboratory System Integration and Silicon Implementation
//   All Right Reserved
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   ICLAB 2026 Fall
//   Lab05 Exercise		: JPEG-Like image compression ENCorder (JPEGLENC)
//   Author     		: Yu-Yang Tang
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//   File Name   : JPEGLENC.v
//   Module Name : JPEGLENC
//   Release version : V1.0 (Release Date: 2026-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################
//
// Architecture (v1, correctness first)
//
//   in_data --> [3 x SBUF 1024x8 stripe buffers, used in rotation]
//                    | 1 pixel / cycle, block raster order
//                    v
//   row DCT  (8 constant multipliers, one pixel per cycle)  --> R[2][8][8] regs
//                    | 64 cycles per block
//                    v
//   column DCT + round(18) + quantise, coefficient i in zigzag order, 1 / cycle
//                    v
//   DC differential / AC run-length -> Huffman field (code + amplitude)
//                    v
//   bit packer (32-bit words) --> OBUF 4096x32 --> byte serialiser --> out_data
//
//   The input is continuous and never stalls, so a stripe (8 image rows,
//   1024 pixels) is stored in one SBUF.  While the next stripe is written
//   into another SBUF, the finished one is read block by block.  The third
//   SBUF gives slack if the entropy stage has to stall for ZRL symbols.
//
module JPEGLENC(
        input clk,
        input rst_n,
        input in_valid,
        input [7:0] in_data,

        output reg out_valid,
        output reg [7:0] out_data
        );

//============================================================================
// ROM functions
//============================================================================
// Integer cosine matrix C(u,x), scaled by 2^8
function signed [8:0] cmat;
    input [2:0] u;
    input [2:0] x;
    begin
        case ({u, x})
            6'd0: cmat = 9'sd181;
            6'd1: cmat = 9'sd181;
            6'd2: cmat = 9'sd181;
            6'd3: cmat = 9'sd181;
            6'd4: cmat = 9'sd181;
            6'd5: cmat = 9'sd181;
            6'd6: cmat = 9'sd181;
            6'd7: cmat = 9'sd181;
            6'd8: cmat = 9'sd251;
            6'd9: cmat = 9'sd213;
            6'd10: cmat = 9'sd142;
            6'd11: cmat = 9'sd50;
            6'd12: cmat = -9'sd50;
            6'd13: cmat = -9'sd142;
            6'd14: cmat = -9'sd213;
            6'd15: cmat = -9'sd251;
            6'd16: cmat = 9'sd237;
            6'd17: cmat = 9'sd98;
            6'd18: cmat = -9'sd98;
            6'd19: cmat = -9'sd237;
            6'd20: cmat = -9'sd237;
            6'd21: cmat = -9'sd98;
            6'd22: cmat = 9'sd98;
            6'd23: cmat = 9'sd237;
            6'd24: cmat = 9'sd213;
            6'd25: cmat = -9'sd50;
            6'd26: cmat = -9'sd251;
            6'd27: cmat = -9'sd142;
            6'd28: cmat = 9'sd142;
            6'd29: cmat = 9'sd251;
            6'd30: cmat = 9'sd50;
            6'd31: cmat = -9'sd213;
            6'd32: cmat = 9'sd181;
            6'd33: cmat = -9'sd181;
            6'd34: cmat = -9'sd181;
            6'd35: cmat = 9'sd181;
            6'd36: cmat = 9'sd181;
            6'd37: cmat = -9'sd181;
            6'd38: cmat = -9'sd181;
            6'd39: cmat = 9'sd181;
            6'd40: cmat = 9'sd142;
            6'd41: cmat = -9'sd251;
            6'd42: cmat = 9'sd50;
            6'd43: cmat = 9'sd213;
            6'd44: cmat = -9'sd213;
            6'd45: cmat = -9'sd50;
            6'd46: cmat = 9'sd251;
            6'd47: cmat = -9'sd142;
            6'd48: cmat = 9'sd98;
            6'd49: cmat = -9'sd237;
            6'd50: cmat = 9'sd237;
            6'd51: cmat = -9'sd98;
            6'd52: cmat = -9'sd98;
            6'd53: cmat = 9'sd237;
            6'd54: cmat = -9'sd237;
            6'd55: cmat = 9'sd98;
            6'd56: cmat = 9'sd50;
            6'd57: cmat = -9'sd142;
            6'd58: cmat = 9'sd213;
            6'd59: cmat = -9'sd251;
            6'd60: cmat = 9'sd251;
            6'd61: cmat = -9'sd213;
            6'd62: cmat = 9'sd142;
            6'd63: cmat = -9'sd50;
            default: cmat = 9'sd0;
        endcase
    end
endfunction

// Zigzag position i -> {v*8+u (6 bit), quantisation shift S(u,v) (3 bit)}
function [8:0] zzs;
    input [5:0] i;
    begin
        case (i)
            6'd0: zzs = 9'd4;
            6'd1: zzs = 9'd11;
            6'd2: zzs = 9'd68;
            6'd3: zzs = 9'd132;
            6'd4: zzs = 9'd76;
            6'd5: zzs = 9'd19;
            6'd6: zzs = 9'd28;
            6'd7: zzs = 9'd84;
            6'd8: zzs = 9'd140;
            6'd9: zzs = 9'd196;
            6'd10: zzs = 9'd260;
            6'd11: zzs = 9'd204;
            6'd12: zzs = 9'd148;
            6'd13: zzs = 9'd92;
            6'd14: zzs = 9'd37;
            6'd15: zzs = 9'd45;
            6'd16: zzs = 9'd101;
            6'd17: zzs = 9'd157;
            6'd18: zzs = 9'd212;
            6'd19: zzs = 9'd268;
            6'd20: zzs = 9'd325;
            6'd21: zzs = 9'd390;
            6'd22: zzs = 9'd333;
            6'd23: zzs = 9'd277;
            6'd24: zzs = 9'd221;
            6'd25: zzs = 9'd165;
            6'd26: zzs = 9'd110;
            6'd27: zzs = 9'd54;
            6'd28: zzs = 9'd62;
            6'd29: zzs = 9'd118;
            6'd30: zzs = 9'd174;
            6'd31: zzs = 9'd230;
            6'd32: zzs = 9'd286;
            6'd33: zzs = 9'd342;
            6'd34: zzs = 9'd398;
            6'd35: zzs = 9'd454;
            6'd36: zzs = 9'd463;
            6'd37: zzs = 9'd406;
            6'd38: zzs = 9'd350;
            6'd39: zzs = 9'd294;
            6'd40: zzs = 9'd238;
            6'd41: zzs = 9'd182;
            6'd42: zzs = 9'd126;
            6'd43: zzs = 9'd190;
            6'd44: zzs = 9'd246;
            6'd45: zzs = 9'd303;
            6'd46: zzs = 9'd358;
            6'd47: zzs = 9'd414;
            6'd48: zzs = 9'd471;
            6'd49: zzs = 9'd479;
            6'd50: zzs = 9'd423;
            6'd51: zzs = 9'd367;
            6'd52: zzs = 9'd311;
            6'd53: zzs = 9'd254;
            6'd54: zzs = 9'd318;
            6'd55: zzs = 9'd375;
            6'd56: zzs = 9'd431;
            6'd57: zzs = 9'd487;
            6'd58: zzs = 9'd495;
            6'd59: zzs = 9'd439;
            6'd60: zzs = 9'd383;
            6'd61: zzs = 9'd447;
            6'd62: zzs = 9'd503;
            6'd63: zzs = 9'd511;
            default: zzs = 9'd0;
        endcase
    end
endfunction

// DC Huffman table: SIZE -> {length (5 bit), code (16 bit)}
function [20:0] dcrom;
    input [3:0] sz;
    begin
        case (sz)
            4'd0: dcrom = 21'h20000;
            4'd1: dcrom = 21'h30002;
            4'd2: dcrom = 21'h30003;
            4'd3: dcrom = 21'h30004;
            4'd4: dcrom = 21'h30005;
            4'd5: dcrom = 21'h30006;
            4'd6: dcrom = 21'h4000E;
            4'd7: dcrom = 21'h5001E;
            4'd8: dcrom = 21'h6003E;
            4'd9: dcrom = 21'h7007E;
            4'd10: dcrom = 21'h800FE;
            4'd11: dcrom = 21'h901FE;
            default: dcrom = 21'd0;
        endcase
    end
endfunction

// AC Huffman table: (RUN<<4)|SIZE -> {length (5 bit), code (16 bit)}
function [20:0] acrom;
    input [7:0] sym;
    begin
        case (sym)
            8'h00: acrom = 21'h4000A;
            8'h01: acrom = 21'h20000;
            8'h02: acrom = 21'h20001;
            8'h03: acrom = 21'h30004;
            8'h04: acrom = 21'h4000B;
            8'h05: acrom = 21'h5001A;
            8'h06: acrom = 21'h70078;
            8'h07: acrom = 21'h800F8;
            8'h08: acrom = 21'hA03F6;
            8'h09: acrom = 21'h10FF82;
            8'h0A: acrom = 21'h10FF83;
            8'h11: acrom = 21'h4000C;
            8'h12: acrom = 21'h5001B;
            8'h13: acrom = 21'h70079;
            8'h14: acrom = 21'h901F6;
            8'h15: acrom = 21'hB07F6;
            8'h16: acrom = 21'h10FF84;
            8'h17: acrom = 21'h10FF85;
            8'h18: acrom = 21'h10FF86;
            8'h19: acrom = 21'h10FF87;
            8'h1A: acrom = 21'h10FF88;
            8'h21: acrom = 21'h5001C;
            8'h22: acrom = 21'h800F9;
            8'h23: acrom = 21'hA03F7;
            8'h24: acrom = 21'hC0FF4;
            8'h25: acrom = 21'h10FF89;
            8'h26: acrom = 21'h10FF8A;
            8'h27: acrom = 21'h10FF8B;
            8'h28: acrom = 21'h10FF8C;
            8'h29: acrom = 21'h10FF8D;
            8'h2A: acrom = 21'h10FF8E;
            8'h31: acrom = 21'h6003A;
            8'h32: acrom = 21'h901F7;
            8'h33: acrom = 21'hC0FF5;
            8'h34: acrom = 21'h10FF8F;
            8'h35: acrom = 21'h10FF90;
            8'h36: acrom = 21'h10FF91;
            8'h37: acrom = 21'h10FF92;
            8'h38: acrom = 21'h10FF93;
            8'h39: acrom = 21'h10FF94;
            8'h3A: acrom = 21'h10FF95;
            8'h41: acrom = 21'h6003B;
            8'h42: acrom = 21'hA03F8;
            8'h43: acrom = 21'h10FF96;
            8'h44: acrom = 21'h10FF97;
            8'h45: acrom = 21'h10FF98;
            8'h46: acrom = 21'h10FF99;
            8'h47: acrom = 21'h10FF9A;
            8'h48: acrom = 21'h10FF9B;
            8'h49: acrom = 21'h10FF9C;
            8'h4A: acrom = 21'h10FF9D;
            8'h51: acrom = 21'h7007A;
            8'h52: acrom = 21'hB07F7;
            8'h53: acrom = 21'h10FF9E;
            8'h54: acrom = 21'h10FF9F;
            8'h55: acrom = 21'h10FFA0;
            8'h56: acrom = 21'h10FFA1;
            8'h57: acrom = 21'h10FFA2;
            8'h58: acrom = 21'h10FFA3;
            8'h59: acrom = 21'h10FFA4;
            8'h5A: acrom = 21'h10FFA5;
            8'h61: acrom = 21'h7007B;
            8'h62: acrom = 21'hC0FF6;
            8'h63: acrom = 21'h10FFA6;
            8'h64: acrom = 21'h10FFA7;
            8'h65: acrom = 21'h10FFA8;
            8'h66: acrom = 21'h10FFA9;
            8'h67: acrom = 21'h10FFAA;
            8'h68: acrom = 21'h10FFAB;
            8'h69: acrom = 21'h10FFAC;
            8'h6A: acrom = 21'h10FFAD;
            8'h71: acrom = 21'h800FA;
            8'h72: acrom = 21'hC0FF7;
            8'h73: acrom = 21'h10FFAE;
            8'h74: acrom = 21'h10FFAF;
            8'h75: acrom = 21'h10FFB0;
            8'h76: acrom = 21'h10FFB1;
            8'h77: acrom = 21'h10FFB2;
            8'h78: acrom = 21'h10FFB3;
            8'h79: acrom = 21'h10FFB4;
            8'h7A: acrom = 21'h10FFB5;
            8'h81: acrom = 21'h901F8;
            8'h82: acrom = 21'hF7FC0;
            8'h83: acrom = 21'h10FFB6;
            8'h84: acrom = 21'h10FFB7;
            8'h85: acrom = 21'h10FFB8;
            8'h86: acrom = 21'h10FFB9;
            8'h87: acrom = 21'h10FFBA;
            8'h88: acrom = 21'h10FFBB;
            8'h89: acrom = 21'h10FFBC;
            8'h8A: acrom = 21'h10FFBD;
            8'h91: acrom = 21'h901F9;
            8'h92: acrom = 21'h10FFBE;
            8'h93: acrom = 21'h10FFBF;
            8'h94: acrom = 21'h10FFC0;
            8'h95: acrom = 21'h10FFC1;
            8'h96: acrom = 21'h10FFC2;
            8'h97: acrom = 21'h10FFC3;
            8'h98: acrom = 21'h10FFC4;
            8'h99: acrom = 21'h10FFC5;
            8'h9A: acrom = 21'h10FFC6;
            8'hA1: acrom = 21'h901FA;
            8'hA2: acrom = 21'h10FFC7;
            8'hA3: acrom = 21'h10FFC8;
            8'hA4: acrom = 21'h10FFC9;
            8'hA5: acrom = 21'h10FFCA;
            8'hA6: acrom = 21'h10FFCB;
            8'hA7: acrom = 21'h10FFCC;
            8'hA8: acrom = 21'h10FFCD;
            8'hA9: acrom = 21'h10FFCE;
            8'hAA: acrom = 21'h10FFCF;
            8'hB1: acrom = 21'hA03F9;
            8'hB2: acrom = 21'h10FFD0;
            8'hB3: acrom = 21'h10FFD1;
            8'hB4: acrom = 21'h10FFD2;
            8'hB5: acrom = 21'h10FFD3;
            8'hB6: acrom = 21'h10FFD4;
            8'hB7: acrom = 21'h10FFD5;
            8'hB8: acrom = 21'h10FFD6;
            8'hB9: acrom = 21'h10FFD7;
            8'hBA: acrom = 21'h10FFD8;
            8'hC1: acrom = 21'hA03FA;
            8'hC2: acrom = 21'h10FFD9;
            8'hC3: acrom = 21'h10FFDA;
            8'hC4: acrom = 21'h10FFDB;
            8'hC5: acrom = 21'h10FFDC;
            8'hC6: acrom = 21'h10FFDD;
            8'hC7: acrom = 21'h10FFDE;
            8'hC8: acrom = 21'h10FFDF;
            8'hC9: acrom = 21'h10FFE0;
            8'hCA: acrom = 21'h10FFE1;
            8'hD1: acrom = 21'hB07F8;
            8'hD2: acrom = 21'h10FFE2;
            8'hD3: acrom = 21'h10FFE3;
            8'hD4: acrom = 21'h10FFE4;
            8'hD5: acrom = 21'h10FFE5;
            8'hD6: acrom = 21'h10FFE6;
            8'hD7: acrom = 21'h10FFE7;
            8'hD8: acrom = 21'h10FFE8;
            8'hD9: acrom = 21'h10FFE9;
            8'hDA: acrom = 21'h10FFEA;
            8'hE1: acrom = 21'h10FFEB;
            8'hE2: acrom = 21'h10FFEC;
            8'hE3: acrom = 21'h10FFED;
            8'hE4: acrom = 21'h10FFEE;
            8'hE5: acrom = 21'h10FFEF;
            8'hE6: acrom = 21'h10FFF0;
            8'hE7: acrom = 21'h10FFF1;
            8'hE8: acrom = 21'h10FFF2;
            8'hE9: acrom = 21'h10FFF3;
            8'hEA: acrom = 21'h10FFF4;
            8'hF0: acrom = 21'hB07F9;
            8'hF1: acrom = 21'h10FFF5;
            8'hF2: acrom = 21'h10FFF6;
            8'hF3: acrom = 21'h10FFF7;
            8'hF4: acrom = 21'h10FFF8;
            8'hF5: acrom = 21'h10FFF9;
            8'hF6: acrom = 21'h10FFFA;
            8'hF7: acrom = 21'h10FFFB;
            8'hF8: acrom = 21'h10FFFC;
            8'hF9: acrom = 21'h10FFFD;
            8'hFA: acrom = 21'h10FFFE;
            default: acrom = 21'd0;
        endcase
    end
endfunction

// number of significant bits of a 12-bit magnitude
function [3:0] bitlen;
    input [11:0] m;
    begin
        if (m[11])      bitlen = 4'd12;
        else if (m[10]) bitlen = 4'd11;
        else if (m[9])  bitlen = 4'd10;
        else if (m[8])  bitlen = 4'd9;
        else if (m[7])  bitlen = 4'd8;
        else if (m[6])  bitlen = 4'd7;
        else if (m[5])  bitlen = 4'd6;
        else if (m[4])  bitlen = 4'd5;
        else if (m[3])  bitlen = 4'd4;
        else if (m[2])  bitlen = 4'd3;
        else if (m[1])  bitlen = 4'd2;
        else if (m[0])  bitlen = 4'd1;
        else            bitlen = 4'd0;
    end
endfunction

//============================================================================
// Input side: write pixels into the stripe buffers
//============================================================================
reg [13:0] pix_cnt;          // pixel index k = 128y + x of the running image
reg [1:0]  wbuf;             // stripe buffer being written (0..2)
reg        in_done;          // last pixel of the image has been accepted
reg [1:0]  rdy_cnt;          // finished stripes not yet started by the reader
wire       img_end;          // asserted with the last output byte of an image
wire       rd_stripe_start;

wire wr_stripe_end = in_valid && (pix_cnt[9:0] == 10'h3FF);
wire wr_last_pix   = in_valid && (pix_cnt == 14'd16383);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pix_cnt <= 14'd0;
        wbuf    <= 2'd0;
        in_done <= 1'b0;
    end
    else begin
        if (in_valid) pix_cnt <= pix_cnt + 14'd1;
        if (wr_stripe_end) wbuf <= (wbuf == 2'd2) ? 2'd0 : (wbuf + 2'd1);
        if (wr_last_pix) in_done <= 1'b1;
        else if (img_end) in_done <= 1'b0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) rdy_cnt <= 2'd0;
    else begin
        case ({wr_stripe_end, rd_stripe_start})
            2'b10:   rdy_cnt <= rdy_cnt + 2'd1;
            2'b01:   rdy_cnt <= rdy_cnt - 2'd1;
            default: rdy_cnt <= rdy_cnt;
        endcase
    end
end

//============================================================================
// Stripe buffers (3 x 1024 x 8, single port)
//============================================================================
reg  [1:0] rbuf;             // stripe buffer being read
reg        r_act;            // reader is issuing the pixels of a block
reg  [3:0] bx;               // block column inside the stripe
reg  [5:0] pcnt;             // pixel inside the block, {y[2:0], x[2:0]}

wire [2:0] sb_wr = {wbuf == 2'd2, wbuf == 2'd1, wbuf == 2'd0} & {3{in_valid}};
wire [2:0] sb_rd = {rbuf == 2'd2, rbuf == 2'd1, rbuf == 2'd0} & {3{r_act}};
wire [9:0] rd_addr = {pcnt[5:3], bx, pcnt[2:0]};
wire [7:0] sb_do0, sb_do1, sb_do2;

JPEGLENC_sbuf u_sb0 (.clk(clk), .cs(sb_wr[0] | sb_rd[0]), .web(~sb_wr[0]),
                     .addr(sb_wr[0] ? pix_cnt[9:0] : rd_addr), .di(in_data), .dout(sb_do0));
JPEGLENC_sbuf u_sb1 (.clk(clk), .cs(sb_wr[1] | sb_rd[1]), .web(~sb_wr[1]),
                     .addr(sb_wr[1] ? pix_cnt[9:0] : rd_addr), .di(in_data), .dout(sb_do1));
JPEGLENC_sbuf u_sb2 (.clk(clk), .cs(sb_wr[2] | sb_rd[2]), .web(~sb_wr[2]),
                     .addr(sb_wr[2] ? pix_cnt[9:0] : rd_addr), .di(in_data), .dout(sb_do2));

//============================================================================
// Reader / row-DCT stage
//============================================================================
reg [1:0] r_alloc;           // R buffer owned by the row stage or column stage
reg [1:0] r_full;            // R buffer holds a complete block
reg       rw_sel;            // next R buffer to allocate
reg       ib;                // R buffer of the block being issued

wire [3:0] nbx      = r_act ? (bx + 4'd1) : bx;
wire       slot_free = (!r_act) || (pcnt == 6'd63);
wire       start_ok  = slot_free && !r_alloc[rw_sel] && ((nbx != 4'd0) || (rdy_cnt != 2'd0));
assign     rd_stripe_start = start_ok && (nbx == 4'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_act <= 1'b0;
        bx    <= 4'd0;
        pcnt  <= 6'd0;
        rbuf  <= 2'd0;
        rw_sel <= 1'b0;
        ib    <= 1'b0;
    end
    else begin
        if (r_act) pcnt <= pcnt + 6'd1;
        if (r_act && (pcnt == 6'd63) && (bx == 4'd15))
            rbuf <= (rbuf == 2'd2) ? 2'd0 : (rbuf + 2'd1);
        if (start_ok) begin
            r_act  <= 1'b1;
            bx     <= nbx;
            pcnt   <= 6'd0;
            ib     <= rw_sel;
            rw_sel <= ~rw_sel;
        end
        else if (r_act && (pcnt == 6'd63)) begin
            r_act <= 1'b0;
            bx    <= nbx;
        end
    end
end

// one cycle later the SRAM data is available
reg       d_v;
reg [5:0] d_pcnt;
reg [1:0] d_rbuf;
reg       d_rb;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) d_v <= 1'b0;
    else        d_v <= r_act;
end
always @(posedge clk) begin
    d_pcnt <= pcnt;
    d_rbuf <= rbuf;
    d_rb   <= ib;
end

wire [7:0] sb_do = (d_rbuf == 2'd0) ? sb_do0 : ((d_rbuf == 2'd1) ? sb_do1 : sb_do2);
wire signed [7:0] pxs = {~sb_do[7], sb_do[6:0]};      // pixel - 128
wire [2:0] d_x = d_pcnt[2:0];
wire [2:0] d_y = d_pcnt[5:3];

// row DCT: R(u,y) = sum_x P'(x,y) C(u,x), one pixel per cycle for all 8 u
wire [8*19-1:0] row_sum;
genvar gu;
generate
    for (gu = 0; gu < 8; gu = gu + 1) begin : ROWMAC
        localparam [2:0] U = gu;
        reg  signed [18:0] acc;
        wire signed [8:0]  cf   = cmat(U, d_x);
        wire signed [16:0] prod = pxs * cf;
        wire signed [18:0] sum  = ((d_x == 3'd0) ? 19'sd0 : acc) + prod;
        always @(posedge clk) begin
            if (d_v) acc <= sum;
        end
        assign row_sum[gu*19 +: 19] = sum;
    end
endgenerate

reg signed [18:0] Rm [0:127];       // {R buffer, y, u}
integer k;
always @(posedge clk) begin
    if (d_v && (d_x == 3'd7)) begin
        for (k = 0; k < 8; k = k + 1)
            Rm[{d_rb, d_y, k[2:0]}] <= row_sum[k*19 +: 19];
    end
end

wire blk_done = d_v && (d_pcnt == 6'd63);      // last row of a block written

//============================================================================
// Column-DCT / quantiser stage (coefficients leave in zigzag order)
//============================================================================
reg        c_run;
reg  [5:0] ci;                  // zigzag index being processed
reg  [5:0] zz_a;                // v*8+u of ci
reg  [2:0] zz_s;                // quantisation shift of ci
reg        cbuf;                // R buffer being read by the column stage
wire       es_stall;
wire       adv = !es_stall;

wire [5:0] ci_inc = ci + 6'd1;
wire [8:0] zz_nx  = zzs(ci_inc);
wire       c_fin  = c_run && (ci == 6'd63);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        c_run <= 1'b0;
        ci    <= 6'd0;
        {zz_a, zz_s} <= zzs(6'd0);
        cbuf  <= 1'b0;
    end
    else if (adv) begin
        if (c_run) begin
            ci <= ci_inc;
            {zz_a, zz_s} <= zz_nx;
            if (c_fin) begin
                cbuf  <= ~cbuf;
                c_run <= r_full[~cbuf];
            end
        end
        else if (r_full[cbuf]) c_run <= 1'b1;
    end
end

// R buffer flags
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_alloc <= 2'b00;
        r_full  <= 2'b00;
    end
    else begin
        if (start_ok) r_alloc[rw_sel] <= 1'b1;
        if (blk_done) r_full[d_rb]    <= 1'b1;
        if (c_fin && adv) begin
            r_alloc[cbuf] <= 1'b0;
            r_full[cbuf]  <= 1'b0;
        end
    end
end

// stage 1: A(u,v) = sum_y R(u,y) C(v,y)
wire [2:0] cu = zz_a[2:0];
wire [2:0] cv = zz_a[5:3];
wire signed [18:0] rc0 = Rm[{cbuf, 3'd0, cu}];
wire signed [18:0] rc1 = Rm[{cbuf, 3'd1, cu}];
wire signed [18:0] rc2 = Rm[{cbuf, 3'd2, cu}];
wire signed [18:0] rc3 = Rm[{cbuf, 3'd3, cu}];
wire signed [18:0] rc4 = Rm[{cbuf, 3'd4, cu}];
wire signed [18:0] rc5 = Rm[{cbuf, 3'd5, cu}];
wire signed [18:0] rc6 = Rm[{cbuf, 3'd6, cu}];
wire signed [18:0] rc7 = Rm[{cbuf, 3'd7, cu}];
wire signed [8:0]  kc0 = cmat(cv, 3'd0);
wire signed [8:0]  kc1 = cmat(cv, 3'd1);
wire signed [8:0]  kc2 = cmat(cv, 3'd2);
wire signed [8:0]  kc3 = cmat(cv, 3'd3);
wire signed [8:0]  kc4 = cmat(cv, 3'd4);
wire signed [8:0]  kc5 = cmat(cv, 3'd5);
wire signed [8:0]  kc6 = cmat(cv, 3'd6);
wire signed [8:0]  kc7 = cmat(cv, 3'd7);
wire signed [27:0] mc0 = rc0 * kc0;
wire signed [27:0] mc1 = rc1 * kc1;
wire signed [27:0] mc2 = rc2 * kc2;
wire signed [27:0] mc3 = rc3 * kc3;
wire signed [27:0] mc4 = rc4 * kc4;
wire signed [27:0] mc5 = rc5 * kc5;
wire signed [27:0] mc6 = rc6 * kc6;
wire signed [27:0] mc7 = rc7 * kc7;
wire signed [28:0] col_sum = mc0 + mc1 + mc2 + mc3 + mc4 + mc5 + mc6 + mc7;

reg               s1_v;
reg  [5:0]        s1_idx;
reg  [2:0]        s1_s;
reg signed [28:0] s1_a;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) s1_v <= 1'b0;
    else if (adv) s1_v <= c_run;
end
always @(posedge clk) begin
    if (adv) begin
        s1_idx <= ci;
        s1_s   <= zz_s;
        s1_a   <= col_sum;
    end
end

// stage 2: D = round_s(A, 18), Q = round_s(D, S)
wire [28:0] a_mag  = s1_a[28] ? (~s1_a + 29'd1) : s1_a;
wire [28:0] a_rnd  = (a_mag + 29'd131072) >> 18;
wire signed [11:0] d_val = s1_a[28] ? -$signed({1'b0, a_rnd[10:0]}) : $signed({1'b0, a_rnd[10:0]});
wire [11:0] d_mag  = d_val[11] ? (~d_val + 12'd1) : d_val;
wire [11:0] q_rnd  = (d_mag + (12'd1 << (s1_s - 3'd1))) >> s1_s;
wire signed [11:0] q_val = d_val[11] ? -$signed({1'b0, q_rnd[10:0]}) : $signed({1'b0, q_rnd[10:0]});

reg               q_v;
reg  [5:0]        q_idx;
reg signed [11:0] q_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) q_v <= 1'b0;
    else if (adv) q_v <= s1_v;
end
always @(posedge clk) begin
    if (adv) begin
        q_idx <= s1_idx;
        q_r   <= q_val;
    end
end

//============================================================================
// Entropy stage: DC differential / AC run-length / Huffman field
//============================================================================
reg signed [11:0] prev_dc;
reg  [3:0] run15;               // zero run modulo 16
reg  [1:0] zrl_cnt;             // complete groups of 16 zeros waiting
reg  [7:0] es_blk;              // blocks finished in this image

reg        f_v;
reg  [4:0] f_len;
reg [25:0] f_bits;
reg        f_last;

wire q_nz   = (q_r != 12'sd0);
wire is_dc  = (q_idx == 6'd0);
wire is_end = (q_idx == 6'd63);
assign es_stall = q_v && !is_dc && q_nz && (zrl_cnt != 2'd0);

wire signed [12:0] fval = is_dc ? ({q_r[11], q_r} - {prev_dc[11], prev_dc}) : {q_r[11], q_r};
wire [11:0] fmag = fval[12] ? (~fval[11:0] + 12'd1) : fval[11:0];
wire [3:0]  fsz  = bitlen(fmag);
wire [11:0] famp = (fval[12] ? (fval[11:0] - 12'd1) : fval[11:0]) & ((13'd1 << fsz) - 13'd1);
wire [20:0] fcode = is_dc ? dcrom(fsz) : acrom({run15, fsz});
wire [4:0]  fcode_len = fcode[20:16];
wire [27:0] fld_bits  = ({12'd0, fcode[15:0]} << fsz) | {16'd0, famp};
wire [4:0]  fld_len   = fcode_len + {1'b0, fsz};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prev_dc <= 12'sd0;
        run15   <= 4'd0;
        zrl_cnt <= 2'd0;
        es_blk  <= 8'd0;
        f_v     <= 1'b0;
        f_len   <= 5'd0;
        f_bits  <= 26'd0;
        f_last  <= 1'b0;
    end
    else if (img_end) begin
        prev_dc <= 12'sd0;
        run15   <= 4'd0;
        zrl_cnt <= 2'd0;
        es_blk  <= 8'd0;
        f_v     <= 1'b0;
        f_last  <= 1'b0;
    end
    else begin
        f_v    <= 1'b0;
        f_last <= 1'b0;
        if (q_v) begin
            if (is_dc) begin
                prev_dc <= q_r;
                run15   <= 4'd0;
                zrl_cnt <= 2'd0;
                f_v     <= 1'b1;
                f_len   <= fld_len;
                f_bits  <= fld_bits[25:0];
            end
            else if (q_nz) begin
                f_v <= 1'b1;
                if (zrl_cnt != 2'd0) begin
                    f_len   <= 5'd11;
                    f_bits  <= 26'h7F9;
                    zrl_cnt <= zrl_cnt - 2'd1;
                end
                else begin
                    f_len   <= fld_len;
                    f_bits  <= fld_bits[25:0];
                    run15   <= 4'd0;
                end
            end
            else begin
                if (is_end) begin
                    f_v    <= 1'b1;
                    f_len  <= 5'd4;
                    f_bits <= 26'hA;
                end
                else begin
                    run15 <= run15 + 4'd1;
                    if (run15 == 4'd15) zrl_cnt <= zrl_cnt + 2'd1;
                end
            end
            if (is_end && !es_stall) begin
                run15   <= 4'd0;
                zrl_cnt <= 2'd0;
                es_blk  <= es_blk + 8'd1;
                f_last  <= (es_blk == 8'd255);
            end
        end
    end
end

//============================================================================
// Bit packer -> OBUF (4096 x 32)
//============================================================================
reg [63:0] pk_acc;
reg  [4:0] pk_cnt;
reg        fl_req;
reg [12:0] wr_words;            // words written to OBUF (including the final one)
reg        fin;                 // final word written
reg  [2:0] last_bytes;          // bytes used in the final word (1..4)

wire [5:0]  tcnt   = {1'b0, pk_cnt} + {1'b0, f_len};
wire [63:0] tmp    = (pk_acc << f_len) | {38'd0, f_bits};
wire        pk_wr  = f_v && (tcnt >= 6'd32);
wire [5:0]  rsh    = tcnt - 6'd32;
wire [63:0] tmp_sh = tmp >> rsh;
wire [31:0] pk_word = tmp_sh[31:0];

wire [2:0]  pad    = 3'd0 - pk_cnt[2:0];
wire [5:0]  cntp   = {1'b0, pk_cnt} + {3'd0, pad};
wire [63:0] vt     = (pk_acc << pad) | ((64'd1 << pad) - 64'd1);
wire [63:0] fl_sh  = vt << (6'd32 - cntp);
wire [31:0] fl_word = fl_sh[31:0];
wire        fl_wr  = fl_req && (cntp != 6'd0);

wire        ob_wr   = pk_wr | fl_wr;
wire [31:0] ob_wdat = pk_wr ? pk_word : fl_word;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pk_acc     <= 64'd0;
        pk_cnt     <= 5'd0;
        fl_req     <= 1'b0;
        wr_words   <= 13'd0;
        fin        <= 1'b0;
        last_bytes <= 3'd4;
    end
    else if (img_end) begin
        pk_cnt     <= 5'd0;
        fl_req     <= 1'b0;
        wr_words   <= 13'd0;
        fin        <= 1'b0;
        last_bytes <= 3'd4;
    end
    else begin
        fl_req <= f_v && f_last;
        if (f_v) begin
            pk_acc <= tmp;
            pk_cnt <= pk_wr ? rsh[4:0] : tcnt[4:0];
        end
        if (ob_wr) wr_words <= wr_words + 13'd1;
        if (fl_req) begin
            pk_cnt     <= 5'd0;
            fin        <= 1'b1;
            last_bytes <= (cntp == 6'd0) ? 3'd4 : cntp[5:3];
        end
    end
end

//============================================================================
// Output stage: OBUF read-back and byte serialiser
//============================================================================
reg [12:0] rd_idx;              // next word to read
reg [12:0] snd_idx;             // index of the word in cur
reg        inflight;
reg [31:0] nxt;
reg        nxt_v;
reg [31:0] cur;
reg        cur_v;
reg  [1:0] bidx;
wire [31:0] ob_do;

wire has_more = (rd_idx < wr_words);
wire rd_issue = in_done && has_more && !nxt_v && !inflight && !ob_wr;

JPEGLENC_obuf u_ob (.clk(clk), .cs(ob_wr | rd_issue), .web(~ob_wr),
                    .addr(ob_wr ? wr_words[11:0] : rd_idx[11:0]),
                    .di(ob_wdat), .dout(ob_do));

wire cur_last = fin && (snd_idx == (wr_words - 13'd1));
wire cur_fin  = cur_v && ((bidx == 2'd3) || (cur_last && ({1'b0, bidx} == (last_bytes - 3'd1))));
assign img_end = cur_fin && cur_last;

reg [7:0] cur_byte;
always @(*) begin
    case (bidx)
        2'd0:    cur_byte = cur[31:24];
        2'd1:    cur_byte = cur[23:16];
        2'd2:    cur_byte = cur[15:8];
        default: cur_byte = cur[7:0];
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_idx    <= 13'd0;
        snd_idx   <= 13'd0;
        inflight  <= 1'b0;
        nxt_v     <= 1'b0;
        cur_v     <= 1'b0;
        bidx      <= 2'd0;
        out_valid <= 1'b0;
        out_data  <= 8'd0;
    end
    else begin
        inflight  <= rd_issue;
        out_valid <= cur_v;
        out_data  <= cur_v ? cur_byte : 8'd0;
        if (rd_issue) rd_idx <= rd_idx + 13'd1;
        if (inflight) begin
            nxt   <= ob_do;
            nxt_v <= 1'b1;
        end
        if (img_end) begin
            rd_idx   <= 13'd0;
            snd_idx  <= 13'd0;
            nxt_v    <= 1'b0;
            cur_v    <= 1'b0;
            bidx     <= 2'd0;
        end
        else if (cur_v) begin
            if (cur_fin) begin
                snd_idx <= snd_idx + 13'd1;
                bidx    <= 2'd0;
                if (nxt_v) begin
                    cur   <= nxt;
                    nxt_v <= 1'b0;
                end
                else cur_v <= 1'b0;
            end
            else bidx <= bidx + 2'd1;
        end
        else if (nxt_v) begin
            cur   <= nxt;
            cur_v <= 1'b1;
            nxt_v <= 1'b0;
            bidx  <= 2'd0;
        end
    end
end

endmodule

//============================================================================
// Memory wrappers: map the bus ports onto the bit-blasted memaker ports.
// The memories themselves live in ../04_MEM (generate them with memaker and
// keep these instance names).
//============================================================================
module JPEGLENC_sbuf(
        input         clk,
        input         cs,
        input         web,
        input  [9:0]  addr,
        input  [7:0]  di,
        output [7:0]  dout
        );
    SBUF_1024X8 u_mem (
        .A0(addr[0]), .A1(addr[1]), .A2(addr[2]), .A3(addr[3]), .A4(addr[4]), .A5(addr[5]),
        .A6(addr[6]), .A7(addr[7]), .A8(addr[8]), .A9(addr[9]), .DO0(dout[0]), .DO1(dout[1]),
        .DO2(dout[2]), .DO3(dout[3]), .DO4(dout[4]), .DO5(dout[5]), .DO6(dout[6]), .DO7(dout[7]),
        .DI0(di[0]), .DI1(di[1]), .DI2(di[2]), .DI3(di[3]), .DI4(di[4]), .DI5(di[5]), .DI6(di[6]),
        .DI7(di[7]), .CK(clk), .WEB(web), .OE(1'b1), .CS(cs)
    );
endmodule

module JPEGLENC_obuf(
        input         clk,
        input         cs,
        input         web,
        input  [11:0] addr,
        input  [31:0] di,
        output [31:0] dout
        );
    OBUF_4096X32 u_mem (
        .A0(addr[0]), .A1(addr[1]), .A2(addr[2]), .A3(addr[3]), .A4(addr[4]), .A5(addr[5]),
        .A6(addr[6]), .A7(addr[7]), .A8(addr[8]), .A9(addr[9]), .A10(addr[10]), .A11(addr[11]),
        .DO0(dout[0]), .DO1(dout[1]), .DO2(dout[2]), .DO3(dout[3]), .DO4(dout[4]), .DO5(dout[5]),
        .DO6(dout[6]), .DO7(dout[7]), .DO8(dout[8]), .DO9(dout[9]), .DO10(dout[10]), .DO11(dout[11]),
        .DO12(dout[12]), .DO13(dout[13]), .DO14(dout[14]), .DO15(dout[15]), .DO16(dout[16]),
        .DO17(dout[17]), .DO18(dout[18]), .DO19(dout[19]), .DO20(dout[20]), .DO21(dout[21]),
        .DO22(dout[22]), .DO23(dout[23]), .DO24(dout[24]), .DO25(dout[25]), .DO26(dout[26]),
        .DO27(dout[27]), .DO28(dout[28]), .DO29(dout[29]), .DO30(dout[30]), .DO31(dout[31]),
        .DI0(di[0]), .DI1(di[1]), .DI2(di[2]), .DI3(di[3]), .DI4(di[4]), .DI5(di[5]), .DI6(di[6]),
        .DI7(di[7]), .DI8(di[8]), .DI9(di[9]), .DI10(di[10]), .DI11(di[11]), .DI12(di[12]),
        .DI13(di[13]), .DI14(di[14]), .DI15(di[15]), .DI16(di[16]), .DI17(di[17]), .DI18(di[18]),
        .DI19(di[19]), .DI20(di[20]), .DI21(di[21]), .DI22(di[22]), .DI23(di[23]), .DI24(di[24]),
        .DI25(di[25]), .DI26(di[26]), .DI27(di[27]), .DI28(di[28]), .DI29(di[29]), .DI30(di[30]),
        .DI31(di[31]), .CK(clk), .WEB(web), .OE(1'b1), .CS(cs)
    );
endmodule
