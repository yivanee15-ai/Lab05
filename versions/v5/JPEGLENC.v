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
//   Release version : V4.0 (Release Date: 2026-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################
//
// V4: V3 re-pipelined for a short clock period.
//
//   inputs / SRAM outputs are registered, every arithmetic block is cut so that
//   no stage holds more than one adder-tree + one carry-propagate adder:
//
//   SC  scheduler  : remaining-position mask, lowest set bit -> zigzag index -> (v,u,S)
//   C   R read     : R(u,j) mux, even/odd pre-add, constants C(v,j)
//   D   multiply   : two 2-product sums (carry-propagated)
//   E   sum+round1 : D = round_s(A,18) with one adder + exact-half correction
//   F   round2     : Q = round_s(D,S)
//   ES0 / ES1      : run/size/zero-run decision, then Huffman field generation
//   PK0 / PK1      : field alignment (barrel shifter, no loop), word merge, registered write
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
// C(v, j) for j = 0..3 (the other four entries follow from the symmetry)
function signed [8:0] cm4;
    input [2:0] v;
    input [1:0] j;
    begin
        case ({v, j})
            5'd0: cm4 = 9'sd181;
            5'd1: cm4 = 9'sd181;
            5'd2: cm4 = 9'sd181;
            5'd3: cm4 = 9'sd181;
            5'd4: cm4 = 9'sd251;
            5'd5: cm4 = 9'sd213;
            5'd6: cm4 = 9'sd142;
            5'd7: cm4 = 9'sd50;
            5'd8: cm4 = 9'sd237;
            5'd9: cm4 = 9'sd98;
            5'd10: cm4 = -9'sd98;
            5'd11: cm4 = -9'sd237;
            5'd12: cm4 = 9'sd213;
            5'd13: cm4 = -9'sd50;
            5'd14: cm4 = -9'sd251;
            5'd15: cm4 = -9'sd142;
            5'd16: cm4 = 9'sd181;
            5'd17: cm4 = -9'sd181;
            5'd18: cm4 = -9'sd181;
            5'd19: cm4 = 9'sd181;
            5'd20: cm4 = 9'sd142;
            5'd21: cm4 = -9'sd251;
            5'd22: cm4 = 9'sd50;
            5'd23: cm4 = 9'sd213;
            5'd24: cm4 = 9'sd98;
            5'd25: cm4 = -9'sd237;
            5'd26: cm4 = 9'sd237;
            5'd27: cm4 = -9'sd98;
            5'd28: cm4 = 9'sd50;
            5'd29: cm4 = -9'sd142;
            5'd30: cm4 = 9'sd213;
            5'd31: cm4 = -9'sd251;
            default: cm4 = 9'sd0;
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
// Input registers
//============================================================================
reg       iv_r;
reg [7:0] id_r;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) iv_r <= 1'b0;
    else        iv_r <= in_valid;
end
always @(posedge clk) id_r <= in_data;

//============================================================================
// Input side: 4 pixels -> one 32-bit word of the stripe buffer
//============================================================================
reg [13:0] pix_cnt;          // pixel index k = 128y + x of the running image
reg        wbuf;             // stripe buffer being written (0..1)
reg [23:0] sh;               // the previous three pixels (shift register)
reg        in_done;          // last pixel of the image has been accepted
reg [1:0]  rdy_cnt;          // finished stripes not yet started by stage 1
reg        img_end_r;        // one cycle after the last output byte
wire       rd_stripe_start;

wire wr_word       = iv_r && (pix_cnt[1:0] == 2'b11);
wire wr_stripe_end = iv_r && (pix_cnt[9:0] == 10'h3FF);
wire wr_last_pix   = iv_r && (pix_cnt == 14'd16383);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pix_cnt <= 14'd0;
        wbuf    <= 1'b0;
        in_done <= 1'b0;
    end
    else begin
        if (iv_r) pix_cnt <= pix_cnt + 14'd1;
        if (wr_stripe_end) wbuf <= ~wbuf;
        if (wr_last_pix) in_done <= 1'b1;
        else if (img_end_r) in_done <= 1'b0;
    end
end

always @(posedge clk) begin
    if (iv_r) sh <= {sh[15:0], id_r};
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
// Stripe buffers (one 512 x 32 single-port macro, two halves selected by the MSB):
// word = {buf, y[2:0], bx[3:0], half}. A write has priority over a read; the read slot waits.
//============================================================================
reg        rbuf;             // stripe buffer being read
reg        r_act;            // stage 1 is issuing the reads of a block
reg  [3:0] bx;               // block column inside the stripe
reg  [3:0] rr;               // read slot inside the block: {y[2:0], half}

wire       rd_ok   = r_act & ~wr_word;
wire [8:0] rd_addr = {rbuf, rr[3:1], bx, rr[0]};
wire [8:0] wr_addr = {wbuf, pix_cnt[9:2]};
wire [31:0] sb_di = {sh, id_r};
wire [31:0] sb_do;

JPEGLENC_sbuf u_sb (.clk(clk), .cs(wr_word | rd_ok), .web(~wr_word),
                    .addr(wr_word ? wr_addr : rd_addr), .di(sb_di), .dout(sb_do));

//============================================================================
// Stage 1: read issue
//============================================================================
reg [1:0] r_alloc;           // R buffer owned by stage 1 or stage 2
reg [1:0] r_full;            // R buffer holds a complete block not yet taken by the scheduler
reg       rw_sel;            // next R buffer to allocate
reg       ib;                // R buffer of the block being issued

wire [3:0] nbx       = r_act ? (bx + 4'd1) : bx;
wire       slot_free = (!r_act) || ((rr == 4'd15) && !wr_word);
wire       start_ok  = slot_free && !r_alloc[rw_sel] && ((nbx != 4'd0) || (rdy_cnt != 2'd0));
assign     rd_stripe_start = start_ok && (nbx == 4'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_act  <= 1'b0;
        bx     <= 4'd0;
        rr     <= 4'd0;
        rbuf   <= 1'b0;
        rw_sel <= 1'b0;
        ib     <= 1'b0;
    end
    else begin
        if (rd_ok) rr <= rr + 4'd1;
        if (rd_ok && (rr == 4'd15) && (bx == 4'd15))
            rbuf <= ~rbuf;
        if (start_ok) begin
            r_act  <= 1'b1;
            bx     <= nbx;
            rr     <= 4'd0;
            ib     <= rw_sel;
            rw_sel <= ~rw_sel;
        end
        else if (rd_ok && (rr == 4'd15)) begin
            r_act <= 1'b0;
            bx    <= nbx;
        end
    end
end

// t1: the SRAM word is on the output pins
reg       d1_v;
reg [3:0] d1_rr;
reg       d1_rbuf;
reg       d1_rb;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) d1_v <= 1'b0;
    else        d1_v <= rd_ok;
end
always @(posedge clk) begin
    d1_rr   <= rr;
    d1_rbuf <= rbuf;
    d1_rb   <= ib;
end

// t2: registered copy of the word
reg [31:0] do_r;
reg        d2_v;
reg [3:0]  d2_rr;
reg        d2_rb;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) d2_v <= 1'b0;
    else        d2_v <= d1_v;
end
always @(posedge clk) begin
    do_r  <= sb_do;
    d2_rr <= d1_rr;
    d2_rb <= d1_rb;
end

reg [31:0] hold;             // first half of the block row
always @(posedge clk) begin
    if (d2_v && !d2_rr[0]) hold <= do_r;
end

// butterfly of the second half with the held first half (pixel - 128 = flip MSB)
wire        row_v = d2_v && d2_rr[0];
wire signed [7:0] x0 = {~hold[31],  hold[30:24]};
wire signed [7:0] x1 = {~hold[23],  hold[22:16]};
wire signed [7:0] x2 = {~hold[15],  hold[14:8]};
wire signed [7:0] x3 = {~hold[7],   hold[6:0]};
wire signed [7:0] x4 = {~do_r[31],  do_r[30:24]};
wire signed [7:0] x5 = {~do_r[23],  do_r[22:16]};
wire signed [7:0] x6 = {~do_r[15],  do_r[14:8]};
wire signed [7:0] x7 = {~do_r[7],   do_r[6:0]};

reg               b_v;
reg  [2:0]        b_y;
reg               b_rb;
reg signed [8:0]  bs0, bs1, bs2, bs3, bd0, bd1, bd2, bd3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) b_v <= 1'b0;
    else        b_v <= row_v;
end
always @(posedge clk) begin
    b_y  <= d2_rr[3:1];
    b_rb <= d2_rb;
    bs0 <= x0 + x7;  bd0 <= x0 - x7;
    bs1 <= x1 + x6;  bd1 <= x1 - x6;
    bs2 <= x2 + x5;  bd2 <= x2 - x5;
    bs3 <= x3 + x4;  bd3 <= x3 - x4;
end

// row transform R(u,y) = sum_x P'(x,y) C(u,x), exact integers
wire signed [10:0] se0 = bs0 + bs1 + bs2 + bs3;
wire signed [10:0] se4 = bs0 - bs1 - bs2 + bs3;
wire signed [9:0]  sa  = bs0 - bs3;
wire signed [9:0]  sb  = bs1 - bs2;
wire signed [18:0] rw0 = se0 * 181;
wire signed [18:0] rw4 = se4 * 181;
wire signed [18:0] rw2 = sa * 237 + sb * 98;
wire signed [18:0] rw6 = sa * 98  - sb * 237;
wire signed [18:0] rw1 = bd0 * 251 + bd1 * 213 + bd2 * 142 + bd3 * 50;
wire signed [18:0] rw3 = bd0 * 213 - bd1 * 50  - bd2 * 251 - bd3 * 142;
wire signed [18:0] rw5 = bd0 * 142 - bd1 * 251 + bd2 * 50  + bd3 * 213;
wire signed [18:0] rw7 = bd0 * 50  - bd1 * 142 + bd2 * 213 - bd3 * 251;

reg signed [18:0] Rm [0:127];       // {R buffer, y, u}
always @(posedge clk) begin
    if (b_v) begin
        Rm[{b_rb, b_y, 3'd0}] <= rw0;
        Rm[{b_rb, b_y, 3'd1}] <= rw1;
        Rm[{b_rb, b_y, 3'd2}] <= rw2;
        Rm[{b_rb, b_y, 3'd3}] <= rw3;
        Rm[{b_rb, b_y, 3'd4}] <= rw4;
        Rm[{b_rb, b_y, 3'd5}] <= rw5;
        Rm[{b_rb, b_y, 3'd6}] <= rw6;
        Rm[{b_rb, b_y, 3'd7}] <= rw7;
    end
end

// big[u] = some R(u,y) lies outside [-2^k, 2^k - 1]; k = 10 9 9 10 11 11 12 12
// (otherwise every coefficient of column u quantises to zero).
// Only the top bits are registered; they are reduced one cycle after the row was written.
reg        t_v;
reg  [2:0] t_y;
reg        t_rb;
reg  [8:0] tp0;  reg [9:0] tp1;  reg [9:0] tp2;  reg [8:0] tp3;
reg  [7:0] tp4;  reg [7:0] tp5;  reg [6:0] tp6;  reg [6:0] tp7;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) t_v <= 1'b0;
    else        t_v <= b_v;
end
always @(posedge clk) begin
    t_y  <= b_y;
    t_rb <= b_rb;
    tp0 <= rw0[18:10];  tp1 <= rw1[18:9];   tp2 <= rw2[18:9];   tp3 <= rw3[18:10];
    tp4 <= rw4[18:11];  tp5 <= rw5[18:11];  tp6 <= rw6[18:12];  tp7 <= rw7[18:12];
end
wire [7:0] big_row;
assign big_row[0] = ~(&tp0 | ~|tp0);
assign big_row[1] = ~(&tp1 | ~|tp1);
assign big_row[2] = ~(&tp2 | ~|tp2);
assign big_row[3] = ~(&tp3 | ~|tp3);
assign big_row[4] = ~(&tp4 | ~|tp4);
assign big_row[5] = ~(&tp5 | ~|tp5);
assign big_row[6] = ~(&tp6 | ~|tp6);
assign big_row[7] = ~(&tp7 | ~|tp7);

reg [7:0] big_acc;
reg [7:0] bigf0, bigf1;
wire      blk_done = t_v && (t_y == 3'd7);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        big_acc <= 8'd0;
        bigf0   <= 8'd0;
        bigf1   <= 8'd0;
    end
    else if (t_v) begin
        big_acc <= (t_y == 3'd0) ? big_row : (big_acc | big_row);
        if (t_y == 3'd7) begin
            if (t_rb) bigf1 <= big_acc | big_row;
            else      bigf0 <= big_acc | big_row;
        end
    end
end

//============================================================================
// Stage SC: scheduler -- positions that may be non-zero, in zigzag order
//============================================================================
reg         sc_run;
reg  [63:0] mask_rem;           // positions of the current block still to be issued
reg         cbuf;               // R buffer of the block being scheduled
wire        es_stall;
wire        adv = !es_stall;

wire [63:0] mask0, mask1;       // possibly-non-zero positions of R buffer 0 / 1
assign mask0[0] = 1'b1;
assign mask1[0] = 1'b1;
assign mask0[1] = bigf0[1];
assign mask0[2] = bigf0[0];
assign mask0[3] = bigf0[0];
assign mask0[4] = bigf0[1];
assign mask0[5] = bigf0[2];
assign mask0[6] = bigf0[3];
assign mask0[7] = bigf0[2];
assign mask0[8] = bigf0[1];
assign mask0[9] = bigf0[0];
assign mask0[10] = bigf0[0];
assign mask0[11] = bigf0[1];
assign mask0[12] = bigf0[2];
assign mask0[13] = bigf0[3];
assign mask0[14] = bigf0[4];
assign mask0[15] = bigf0[5];
assign mask0[16] = bigf0[4];
assign mask0[17] = bigf0[3];
assign mask0[18] = bigf0[2];
assign mask0[19] = bigf0[1];
assign mask0[20] = bigf0[0];
assign mask0[21] = bigf0[0];
assign mask0[22] = bigf0[1];
assign mask0[23] = bigf0[2];
assign mask0[24] = bigf0[3];
assign mask0[25] = bigf0[4];
assign mask0[26] = bigf0[5];
assign mask0[27] = bigf0[6];
assign mask0[28] = bigf0[7];
assign mask0[29] = bigf0[6];
assign mask0[30] = bigf0[5];
assign mask0[31] = bigf0[4];
assign mask0[32] = bigf0[3];
assign mask0[33] = bigf0[2];
assign mask0[34] = bigf0[1];
assign mask0[35] = bigf0[0];
assign mask0[36] = bigf0[1];
assign mask0[37] = bigf0[2];
assign mask0[38] = bigf0[3];
assign mask0[39] = bigf0[4];
assign mask0[40] = bigf0[5];
assign mask0[41] = bigf0[6];
assign mask0[42] = bigf0[7];
assign mask0[43] = bigf0[7];
assign mask0[44] = bigf0[6];
assign mask0[45] = bigf0[5];
assign mask0[46] = bigf0[4];
assign mask0[47] = bigf0[3];
assign mask0[48] = bigf0[2];
assign mask0[49] = bigf0[3];
assign mask0[50] = bigf0[4];
assign mask0[51] = bigf0[5];
assign mask0[52] = bigf0[6];
assign mask0[53] = bigf0[7];
assign mask0[54] = bigf0[7];
assign mask0[55] = bigf0[6];
assign mask0[56] = bigf0[5];
assign mask0[57] = bigf0[4];
assign mask0[58] = bigf0[5];
assign mask0[59] = bigf0[6];
assign mask0[60] = bigf0[7];
assign mask0[61] = bigf0[7];
assign mask0[62] = bigf0[6];
assign mask0[63] = bigf0[7];
assign mask1[1] = bigf1[1];
assign mask1[2] = bigf1[0];
assign mask1[3] = bigf1[0];
assign mask1[4] = bigf1[1];
assign mask1[5] = bigf1[2];
assign mask1[6] = bigf1[3];
assign mask1[7] = bigf1[2];
assign mask1[8] = bigf1[1];
assign mask1[9] = bigf1[0];
assign mask1[10] = bigf1[0];
assign mask1[11] = bigf1[1];
assign mask1[12] = bigf1[2];
assign mask1[13] = bigf1[3];
assign mask1[14] = bigf1[4];
assign mask1[15] = bigf1[5];
assign mask1[16] = bigf1[4];
assign mask1[17] = bigf1[3];
assign mask1[18] = bigf1[2];
assign mask1[19] = bigf1[1];
assign mask1[20] = bigf1[0];
assign mask1[21] = bigf1[0];
assign mask1[22] = bigf1[1];
assign mask1[23] = bigf1[2];
assign mask1[24] = bigf1[3];
assign mask1[25] = bigf1[4];
assign mask1[26] = bigf1[5];
assign mask1[27] = bigf1[6];
assign mask1[28] = bigf1[7];
assign mask1[29] = bigf1[6];
assign mask1[30] = bigf1[5];
assign mask1[31] = bigf1[4];
assign mask1[32] = bigf1[3];
assign mask1[33] = bigf1[2];
assign mask1[34] = bigf1[1];
assign mask1[35] = bigf1[0];
assign mask1[36] = bigf1[1];
assign mask1[37] = bigf1[2];
assign mask1[38] = bigf1[3];
assign mask1[39] = bigf1[4];
assign mask1[40] = bigf1[5];
assign mask1[41] = bigf1[6];
assign mask1[42] = bigf1[7];
assign mask1[43] = bigf1[7];
assign mask1[44] = bigf1[6];
assign mask1[45] = bigf1[5];
assign mask1[46] = bigf1[4];
assign mask1[47] = bigf1[3];
assign mask1[48] = bigf1[2];
assign mask1[49] = bigf1[3];
assign mask1[50] = bigf1[4];
assign mask1[51] = bigf1[5];
assign mask1[52] = bigf1[6];
assign mask1[53] = bigf1[7];
assign mask1[54] = bigf1[7];
assign mask1[55] = bigf1[6];
assign mask1[56] = bigf1[5];
assign mask1[57] = bigf1[4];
assign mask1[58] = bigf1[5];
assign mask1[59] = bigf1[6];
assign mask1[60] = bigf1[7];
assign mask1[61] = bigf1[7];
assign mask1[62] = bigf1[6];
assign mask1[63] = bigf1[7];

// parallel-prefix OR: pre[i] = |mask_rem[i:0]
wire [63:0] pre1 = mask_rem | (mask_rem << 1);
wire [63:0] pre2 = pre1 | (pre1 << 2);
wire [63:0] pre3 = pre2 | (pre2 << 4);
wire [63:0] pre4 = pre3 | (pre3 << 8);
wire [63:0] pre5 = pre4 | (pre4 << 16);
wire [63:0] pre6 = pre5 | (pre5 << 32);
wire [63:0] below  = pre6 << 1;                  // below[i] = some lower position is still set
wire [63:0] oh     = mask_rem & ~below;          // lowest remaining position (one-hot)
wire [63:0] rem_nx = mask_rem & below;
wire        last_nx = ~|rem_nx;
reg  [5:0]  idx_nx;
integer     ii;
always @(*) begin
    idx_nx = 6'd0;
    for (ii = 0; ii < 64; ii = ii + 1)
        if (oh[ii]) idx_nx = idx_nx | ii[5:0];
end

wire sc_take_run  = sc_run && last_nx && r_full[~cbuf];   // switch straight to the next block
wire sc_take_idle = !sc_run && r_full[cbuf];              // start a block
wire sc_take      = adv && (sc_take_run || sc_take_idle);
wire sc_take_buf  = sc_run ? ~cbuf : cbuf;

reg        z_v;
reg        z_last;
reg        z_buf;
reg  [5:0] z_idx;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sc_run   <= 1'b0;
        cbuf     <= 1'b0;
        z_v      <= 1'b0;
        mask_rem <= 64'd0;
    end
    else if (adv) begin
        z_v <= sc_run;
        if (sc_run) begin
            if (last_nx) begin
                cbuf   <= ~cbuf;
                sc_run <= r_full[~cbuf];
                if (r_full[~cbuf]) mask_rem <= cbuf ? mask0 : mask1;
            end
            else mask_rem <= rem_nx;
        end
        else if (r_full[cbuf]) begin
            sc_run   <= 1'b1;
            mask_rem <= cbuf ? mask1 : mask0;
        end
    end
end
always @(posedge clk) begin
    if (adv && sc_run) begin
        z_last <= last_nx;
        z_buf  <= cbuf;
        z_idx  <= idx_nx;
    end
end

// R buffer flags: full = block ready for the scheduler, alloc = buffer still in use
wire c_last_fire = adv && z_v && z_last;     // last slot of a block leaves the R read stage
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        r_alloc <= 2'b00;
        r_full  <= 2'b00;
    end
    else begin
        if (start_ok)     r_alloc[rw_sel] <= 1'b1;
        if (blk_done)     r_full[t_rb]    <= 1'b1;
        if (sc_take)      r_full[sc_take_buf] <= 1'b0;
        if (c_last_fire)  r_alloc[z_buf]  <= 1'b0;
    end
end

//============================================================================
// Stage C: R(u,j) read and even/odd pre-add, constants C(v,j)
//============================================================================
wire [8:0] zz_c = zzs(z_idx);          // {v[2:0], u[2:0], S[2:0]}
wire [2:0] cu = zz_c[5:3];
wire [2:0] cv = zz_c[8:6];
wire signed [18:0] rc0 = Rm[{z_buf, 3'd0, cu}];
wire signed [18:0] rc1 = Rm[{z_buf, 3'd1, cu}];
wire signed [18:0] rc2 = Rm[{z_buf, 3'd2, cu}];
wire signed [18:0] rc3 = Rm[{z_buf, 3'd3, cu}];
wire signed [18:0] rc4 = Rm[{z_buf, 3'd4, cu}];
wire signed [18:0] rc5 = Rm[{z_buf, 3'd5, cu}];
wire signed [18:0] rc6 = Rm[{z_buf, 3'd6, cu}];
wire signed [18:0] rc7 = Rm[{z_buf, 3'd7, cu}];

reg               c_v, c_last;
reg  [5:0]        c_idx;
reg  [2:0]        c_s;
reg signed [19:0] c_pa0, c_pa1, c_pa2, c_pa3;
reg signed [8:0]  c_k0, c_k1, c_k2, c_k3;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) c_v <= 1'b0;
    else if (adv) c_v <= z_v;
end
always @(posedge clk) begin
    if (adv) begin
        c_last <= z_last;
        c_idx  <= z_idx;
        c_s    <= zz_c[2:0];
        c_pa0  <= cv[0] ? (rc0 - rc7) : (rc0 + rc7);
        c_pa1  <= cv[0] ? (rc1 - rc6) : (rc1 + rc6);
        c_pa2  <= cv[0] ? (rc2 - rc5) : (rc2 + rc5);
        c_pa3  <= cv[0] ? (rc3 - rc4) : (rc3 + rc4);
        c_k0   <= cm4(cv, 2'd0);
        c_k1   <= cm4(cv, 2'd1);
        c_k2   <= cm4(cv, 2'd2);
        c_k3   <= cm4(cv, 2'd3);
    end
end

//============================================================================
// Stage D: two sums of two products
//============================================================================
reg               d_v, d_last;
reg  [5:0]        d_idx;
reg  [2:0]        d_s;
reg signed [28:0] d_m01, d_m23;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) d_v <= 1'b0;
    else if (adv) d_v <= c_v;
end
always @(posedge clk) begin
    if (adv) begin
        d_last <= c_last;
        d_idx  <= c_idx;
        d_s    <= c_s;
        d_m01  <= c_pa0 * c_k0 + c_pa1 * c_k1;
        d_m23  <= c_pa2 * c_k2 + c_pa3 * c_k3;
    end
end

//============================================================================
// Stage E: A = m01 + m23, D = round_s(A, 18) with one adder:
//   tp = A + 2^17, D = (tp >>> 18) - (exact negative half)
//============================================================================
wire signed [29:0] tp_c = {d_m01[28], d_m01} + {d_m23[28], d_m23} + 30'sd131072;

reg               e_v, e_last;
reg  [5:0]        e_idx;
reg  [2:0]        e_s;
reg signed [29:0] e_tp;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) e_v <= 1'b0;
    else if (adv) e_v <= d_v;
end
always @(posedge clk) begin
    if (adv) begin
        e_last <= d_last;
        e_idx  <= d_idx;
        e_s    <= d_s;
        e_tp   <= tp_c;
    end
end

//============================================================================
// Stage F: D = (tp >>> 18) - (exact negative half); Q = round_s(D, S) = (D + 2^(S-1) - 1 + (D >= 0)) >>> S
//============================================================================
wire               corr = (e_tp[17:0] == 18'd0) && (e_tp[29] || (e_tp[29:18] == 12'd0));
wire signed [11:0] e_d  = e_tp[29:18] - {11'd0, corr};
reg  [5:0]         hm1;
always @(*) begin
    case (e_s)
        3'd3:    hm1 = 6'd3;
        3'd4:    hm1 = 6'd7;
        3'd5:    hm1 = 6'd15;
        3'd6:    hm1 = 6'd31;
        default: hm1 = 6'd63;
    endcase
end
wire signed [12:0] t2  = {e_d[11], e_d} + {7'd0, hm1} + {12'd0, ~e_d[11]};
wire signed [12:0] q13 = t2 >>> e_s;

reg               q_v, q_last;
reg  [5:0]        q_idx;
reg signed [11:0] q_r;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) q_v <= 1'b0;
    else if (adv) q_v <= e_v;
end
always @(posedge clk) begin
    if (adv) begin
        q_last <= e_last;
        q_idx  <= e_idx;
        q_r    <= q13[11:0];
    end
end

//============================================================================
// ES0: run / size / ZRL decision (one event per cycle, stalls the stages above for ZRLs)
//============================================================================
reg signed [11:0] prev_dc;
reg  [5:0] last_nz;             // zigzag index of the last non-zero coefficient
reg        zact;                // ZRL emission in progress
reg  [5:0] run_r;               // remaining run while ZRLs are emitted
reg  [7:0] es_blk;              // blocks finished in this image

wire q_nz   = (q_r != 12'sd0);
wire is_dc  = (q_idx == 6'd0);
wire [6:0] run_new = {1'b0, q_idx} - {1'b0, last_nz} - 7'd1;
wire [6:0] run_in  = zact ? {1'b0, run_r} : run_new;
wire need_zrl = |run_in[6:4];
wire zrl_cyc  = !is_dc && q_nz && need_zrl;
assign es_stall = q_v && zrl_cyc;

wire signed [12:0] fval = is_dc ? ({q_r[11], q_r} - {prev_dc[11], prev_dc}) : {q_r[11], q_r};
wire [11:0] fmag = fval[12] ? (~fval[11:0] + 12'd1) : fval[11:0];
wire [3:0]  fsz  = bitlen(fmag);
wire [11:0] fam  = fval[12] ? (fval[11:0] - 12'd1) : fval[11:0];
wire sym_v = is_dc || q_nz;
wire eob_v = q_last && !(!is_dc && q_nz && (q_idx == 6'd63));

reg        e1_v, e1_zrl, e1_dc, e1_sym, e1_eob, e1_last;
reg  [3:0] e1_sz, e1_run;
reg [11:0] e1_am;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        prev_dc <= 12'sd0;
        last_nz <= 6'd0;
        zact    <= 1'b0;
        run_r   <= 6'd0;
        es_blk  <= 8'd0;
        e1_v    <= 1'b0;
        e1_last <= 1'b0;
    end
    else if (img_end_r) begin
        prev_dc <= 12'sd0;
        last_nz <= 6'd0;
        zact    <= 1'b0;
        es_blk  <= 8'd0;
        e1_v    <= 1'b0;
        e1_last <= 1'b0;
    end
    else begin
        e1_v    <= 1'b0;
        e1_last <= 1'b0;
        if (q_v) begin
            e1_v    <= zrl_cyc || sym_v || eob_v;
            e1_zrl  <= zrl_cyc;
            e1_dc   <= is_dc;
            e1_sym  <= sym_v && !zrl_cyc;
            e1_eob  <= eob_v && !zrl_cyc;
            if (zrl_cyc) begin
                run_r <= run_in[5:0] - 6'd16;
                zact  <= 1'b1;
            end
            else begin
                zact <= 1'b0;
                if (is_dc)     begin prev_dc <= q_r; last_nz <= 6'd0; end
                else if (q_nz) last_nz <= q_idx;
                if (q_last) begin
                    es_blk  <= es_blk + 8'd1;
                    e1_last <= (es_blk == 8'd255);
                end
            end
        end
    end
end
always @(posedge clk) begin
    e1_sz  <= fsz;
    e1_run <= run_in[3:0];
    e1_am  <= fam;
end

//============================================================================
// ES1: Huffman code + amplitude (+ merged EOB) -> field
//============================================================================
reg        f_v;
reg  [4:0] f_len;
reg [29:0] f_bits;
reg        f_last;

wire [20:0] fcode = e1_dc ? dcrom(e1_sz) : acrom({e1_run, e1_sz});
wire [11:0] famp  = e1_am & ((13'd1 << e1_sz) - 13'd1);
wire [27:0] fld_bits = ({12'd0, fcode[15:0]} << e1_sz) | {16'd0, famp};
wire [4:0]  fld_len  = fcode[20:16] + {1'b0, e1_sz};
wire [29:0] fb_sym = e1_sym ? fld_bits[25:0] : 30'd0;
wire [4:0]  fl_sym = e1_sym ? fld_len : 5'd0;
wire [29:0] fb_all = e1_eob ? ((fb_sym << 4) | 30'hA) : fb_sym;
wire [4:0]  fl_all = e1_eob ? (fl_sym + 5'd4) : fl_sym;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        f_v    <= 1'b0;
        f_last <= 1'b0;
    end
    else begin
        f_v    <= e1_v;
        f_last <= e1_last;
    end
end
always @(posedge clk) begin
    f_len  <= e1_zrl ? 5'd11 : fl_all;
    f_bits <= e1_zrl ? 30'h7F9 : fb_all;
end

//============================================================================
// PK0: align the field inside a 64-bit window (current word | next word)
//============================================================================
reg  [4:0]  pk_cnt;             // bits already used in the current output word
reg         fl1;                // flush request (one cycle after the last field was merged)
wire [5:0]  tsum   = {1'b0, pk_cnt} + {1'b0, f_len};
wire [5:0]  shl    = 6'd0 - tsum;
wire [63:0] placed_c = {34'd0, f_bits} << shl;

reg         pl_v, pl_spill, pl_last;
reg  [63:0] placed;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pl_v     <= 1'b0;
        pl_last  <= 1'b0;
        pk_cnt   <= 5'd0;
    end
    else begin
        pl_v    <= f_v;
        pl_last <= f_last;
        if (f_v) pk_cnt <= tsum[4:0];
        if (fl1) pk_cnt <= 5'd0;
    end
end
always @(posedge clk) begin
    placed   <= placed_c;
    pl_spill <= (tsum >= 6'd32);
end

//============================================================================
// PK1: merge into the current word, registered write to OBUF
//============================================================================
reg [31:0] cw;
reg        wv;                  // OBUF write in this cycle
reg        wfin;                // ... the final word / end of image
reg  [2:0] wlb;
reg [31:0] wdat;
reg [12:0] wr_words;            // words written to OBUF
reg        fin;                 // final word written
reg  [2:0] last_bytes;          // bytes used in the final word (1..4)

wire [31:0] cw_new = cw | placed[63:32];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wv   <= 1'b0;
        wfin <= 1'b0;
        fl1  <= 1'b0;
        cw   <= 32'd0;
    end
    else begin
        wv   <= 1'b0;
        wfin <= 1'b0;
        fl1  <= pl_last;
        if (pl_v) begin
            if (pl_spill) begin
                wv   <= 1'b1;
                cw   <= placed[31:0];
            end
            else cw <= cw_new;
        end
        if (fl1) begin
            wv   <= (pk_cnt != 5'd0);
            wfin <= 1'b1;
            cw   <= 32'd0;
        end
    end
end
always @(posedge clk) begin
    if (pl_v && pl_spill) wdat <= cw_new;
    else if (fl1)         wdat <= cw | (32'hFFFFFFFF >> pk_cnt);
    wlb <= (pk_cnt == 5'd0) ? 3'd4 : ((pk_cnt + 5'd7) >> 3);
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_words   <= 13'd0;
        fin        <= 1'b0;
        last_bytes <= 3'd4;
    end
    else if (img_end_r) begin
        wr_words   <= 13'd0;
        fin        <= 1'b0;
        last_bytes <= 3'd4;
    end
    else begin
        if (wv) wr_words <= wr_words + 13'd1;
        if (wfin) begin
            fin        <= 1'b1;
            last_bytes <= wlb;
        end
    end
end

//============================================================================
// Output stage: OBUF read-back (prefetching) and byte serialiser
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
wire rd_issue = has_more && !nxt_v && !inflight && !wv;
wire snd      = cur_v && in_done;

JPEGLENC_obuf u_ob (.clk(clk), .cs(wv | rd_issue), .web(~wv),
                    .addr(wv ? wr_words[11:0] : rd_idx[11:0]),
                    .di(wdat), .dout(ob_do));

wire cur_last = fin && (snd_idx == (wr_words - 13'd1));
wire cur_fin  = snd && ((bidx == 2'd3) || (cur_last && ({1'b0, bidx} == (last_bytes - 3'd1))));
wire fin_last = cur_fin && cur_last;

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
    if (!rst_n) img_end_r <= 1'b0;
    else        img_end_r <= fin_last;
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
        out_valid <= snd;
        out_data  <= snd ? cur_byte : 8'd0;
        if (rd_issue) rd_idx <= rd_idx + 13'd1;
        if (inflight) begin
            nxt   <= ob_do;
            nxt_v <= 1'b1;
        end
        if (img_end_r) begin
            rd_idx   <= 13'd0;
            snd_idx  <= 13'd0;
        end
        if (fin_last) begin
            nxt_v    <= 1'b0;
            cur_v    <= 1'b0;
            bidx     <= 2'd0;
        end
        else if (snd) begin
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
        else if (nxt_v && !cur_v) begin
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
//============================================================================
module JPEGLENC_sbuf(
        input         clk,
        input         cs,
        input         web,
        input  [8:0]  addr,
        input  [31:0] di,
        output [31:0] dout
        );
    SBUF_512X32 u_mem (
        .A0(addr[0]), .A1(addr[1]), .A2(addr[2]), .A3(addr[3]), .A4(addr[4]), .A5(addr[5]),
        .A6(addr[6]), .A7(addr[7]), .A8(addr[8]), .DO0(dout[0]), .DO1(dout[1]), .DO2(dout[2]),
        .DO3(dout[3]), .DO4(dout[4]), .DO5(dout[5]), .DO6(dout[6]), .DO7(dout[7]), .DO8(dout[8]),
        .DO9(dout[9]), .DO10(dout[10]), .DO11(dout[11]), .DO12(dout[12]), .DO13(dout[13]),
        .DO14(dout[14]), .DO15(dout[15]), .DO16(dout[16]), .DO17(dout[17]), .DO18(dout[18]),
        .DO19(dout[19]), .DO20(dout[20]), .DO21(dout[21]), .DO22(dout[22]), .DO23(dout[23]),
        .DO24(dout[24]), .DO25(dout[25]), .DO26(dout[26]), .DO27(dout[27]), .DO28(dout[28]),
        .DO29(dout[29]), .DO30(dout[30]), .DO31(dout[31]), .DI0(di[0]), .DI1(di[1]), .DI2(di[2]),
        .DI3(di[3]), .DI4(di[4]), .DI5(di[5]), .DI6(di[6]), .DI7(di[7]), .DI8(di[8]), .DI9(di[9]),
        .DI10(di[10]), .DI11(di[11]), .DI12(di[12]), .DI13(di[13]), .DI14(di[14]), .DI15(di[15]),
        .DI16(di[16]), .DI17(di[17]), .DI18(di[18]), .DI19(di[19]), .DI20(di[20]), .DI21(di[21]),
        .DI22(di[22]), .DI23(di[23]), .DI24(di[24]), .DI25(di[25]), .DI26(di[26]), .DI27(di[27]),
        .DI28(di[28]), .DI29(di[29]), .DI30(di[30]), .DI31(di[31]), .CK(clk), .WEB(web), .OE(1'b1),
        .CS(cs)
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
    OBUF_3072X32 u_mem (
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
