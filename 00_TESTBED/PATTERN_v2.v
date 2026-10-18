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
//   File Name   : PATTERN.v
//   Module Name : PATTERN
//   Release version : V1.0 (Release Date: 2026-09)
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//############################################################################
`timescale 1ns/10ps

`ifdef RTL
    `define CYCLE_TIME 4.7
`endif
`ifdef GATE
    `define CYCLE_TIME 4.7
`endif
`ifdef POST
    `define CYCLE_TIME 4.7
`endif

module PATTERN(
    // output signals
    clk,
    rst_n,
    in_valid,
    in_data,

    // input signals
    out_valid,
    out_data
);

// ===============================================================
// I/O declaration
// ===============================================================
output reg          clk;
output reg          rst_n;
output reg          in_valid;
output reg  [7:0]   in_data;

input               out_valid;
input       [7:0]   out_data;

// ===============================================================
// Parameters and variables
// ===============================================================

// Added verification logic. All original TA template lines are preserved.
// This is a simulation-only PATTERN; tasks/functions and file I/O are not RTL.
// Usage: define RTL, GATE or POST as in the TA TESTBED/filelist.
// Default: 100 images, balanced by measured JPEG coefficient workload.
// Preserve original images 1..81 byte-for-byte for the same seed and simulator:
// 49 directed cases and 32 full-range random images. Append 19 balancing cases.
// High-load: at least 4032 of 16128 quantized AC coefficients are nonzero (25%).
// Low-load: fewer than 4032 nonzero quantized AC coefficients.
// This is an explicit workload proxy, not a TA-specified Shannon entropy rule.
// With default seed 2026, the original mix is 39 high-load / 42 low-load;
// append 11 fresh random images and 8 smooth images to obtain 50 / 50.
// Fixed checkerboards/impulses are classified by their actual AC workload.
// For another seed, appended types adapt to the measured counts of the first 81.
// +PATNUM overrides/file inputs report actual counts; 1:1 is guaranteed only for
// the complete built-in 100-image suite, not a truncated/custom transaction set.
// Image 49 (pattern_idx=48) explicitly targets negative quantization halves.
// Optional: +PATNUM=100 +SEED=2026 +LOG_EVERY=1
// Default: read ../00_TESTBED/input.txt from 01_RTL or 03_GATE; compute golden internally.
// Optional: +INPUT_FILE=path selects another input; +BUILTIN=1 uses the old suite.
// Large-file format: LAB05_HEX_V1 <decimal image count>, then two-digit hex pixels.
// The header avoids a complete counting pass. Headerless files remain supported.
// Optional file mode: +INPUT_FILE=path
// File mode accepts whitespace-separated TWO-digit hex pixels, no header,
// in raster order, exactly 16384 pixels per image. Count is inferred from EOF.
// It does not assume the format of an unprovided TA input.txt.
// All fixed tables below are copied from the four supplied TA table files.
// No external LUT/answer file or DesignWare library is required.
// Checks needing reports or SRAM hierarchy (macro area, WC corner, MEM_error,
// latches, synthesis runtime and SDF timing) remain tool/model-side checks.
`ifndef CYCLE_TIME
    `define CYCLE_TIME 20
`endif
parameter CYCLE = `CYCLE_TIME;
// User-selected stricter local timeout. The TA limit remains 50000 cycles.
// Set MAX_LATENCY back to 50000 when checking the full official allowance.
parameter MAX_LATENCY = 35000;
parameter DIRECTED_PATTERNS = 49;
parameter RANDOM_PATTERNS = 32;
parameter BALANCED_PATTERNS = 100;
parameter AC_DENSE_THRESHOLD = 4032;
parameter RESET_LOW_NS = 100.0;
parameter RESET_RECOVERY_CYCLES = 2;
parameter GAP_MIN = 3;
parameter GAP_MAX = 6;
parameter MAX_BYTES = 65536;

integer pattern_num, pattern_idx, beat_idx, output_idx;
integer latency_cycles, last_byte_cycle, log_every;
reg [63:0] total_latency, total_bytes;
integer data_seed, gap_seed, initial_seed, gap_cycles;
reg [1:0] gap_draw;
integer gap_hist [0:6];
integer init_idx, file_handle, scan_status, pixel_from_file;
integer builtin_mode, count_digit, count_char;
reg [63:0] file_pixel_count;
reg [8*512-1:0] input_path;
reg [8*64-1:0] input_token;
reg file_mode, clock_on, monitor_on, timing_on;
reg response_pending, diagnosed;
real last_posedge_time, input_end_time;

reg [7:0] image_data [0:16383];
reg [7:0] golden_byte [0:MAX_BYTES-1];
integer byte_block [0:MAX_BYTES-1];
integer golden_count, partial_bits, encoded_bits, model_block;
reg [7:0] partial_byte;
integer model_previous_dc;
integer cosine [0:63], quant_shift [0:63], zigzag_address [0:63];
integer dc_length [0:11], ac_length [0:255];
reg [15:0] dc_code [0:11], ac_code [0:255];
reg signed [63:0] row_dct [0:63];
integer dct_coeff [0:63], quant_coeff [0:63], zigzag_coeff [0:63];
reg [63:0] example_row [0:15];
reg [63:0] negative_tie_row [0:119];
integer tie_block [0:14], tie_address [0:14], tie_dct [0:14];

// Golden-derived coverage is observational; it never relaxes a check.
integer dc_hist [0:11], ac_hist [0:10], run_hist [0:15];
integer pad_hist [0:7];
integer negative_half_hist [0:7];
integer count_eob, count_zrl, count_last_ac, count_positive, count_negative;
integer count_double_round, count_raw_ff, count_unaligned_blocks;
integer coverage_on;
integer image_nonzero_ac, high_class_count, low_class_count, extra_smooth_count;
integer ac_symbol_hist [0:255], dc_signed_hist [0:17], ac_signed_hist [0:15];
integer block_offset_hist [0:7], trailing_hist [0:63], zrl_burst_hist [0:3];
integer first_boundary_hist [0:5], quant_boundary_hist [0:29];
integer coefficient_sign_hist [0:127];


// ===============================================================
// Clock
// ===============================================================

// Clock is held low during reset AND the recovery interval.
// After recovery, the first posedge occurs half a period later.
always begin
    wait (clock_on === 1'b1);
    #(CYCLE / 2.0);
    clk = ~clk;
end

always @(posedge clk) begin
    last_posedge_time = $realtime;
    timing_on = 1'b1;
end

// Allow propagation during the posedge-to-negedge half cycle in GLS.
// A change during the opposite half cycle is not a posedge-launched output.
// This does not replace the SRAM/SDF simulator's setup/hold checks.
always @(out_valid or out_data) begin
    #0;
    if ((monitor_on === 1'b1) && (rst_n === 1'b1)) begin
        if ((out_valid !== 1'b0) && (out_valid !== 1'b1)) begin
            stop_test("UNKNOWN_VALID: out_valid contains X/Z");
        end
        if ((clock_on === 1'b1) && (timing_on === 1'b1) &&
            (($realtime - last_posedge_time) >= CYCLE / 2.0)) begin
            stop_test("OUTPUT_EDGE: output changed in the negedge-to-posedge half cycle");
        end
    end
end

// Overlap must never occur, including between sampled clock edges.
always @(out_valid or in_valid) begin
    #0;
    if ((monitor_on === 1'b1) && (rst_n === 1'b1) &&
        (out_valid === 1'b1) && (in_valid === 1'b1)) begin
        stop_test("IO_OVERLAP: out_valid overlaps in_valid");
    end
end

// Multiple rises are legal inside one image: Lab05 permits output bubbles.
always @(posedge out_valid) begin
    #0;
    if ((monitor_on === 1'b1) && (rst_n === 1'b1) &&
        (response_pending !== 1'b1)) begin
        stop_test("UNEXPECTED_OUTPUT: no completed input transaction is pending");
    end
end

always @(negedge clk) begin
    #0;
    if (monitor_on === 1'b1) begin
        check_environment;
    end
end


// ===============================================================
// initial block
// ===============================================================

initial begin
    clk = 1'b0;
    clock_on = 1'b0;
    timing_on = 1'b0;
    rst_n = 1'b1;
    in_valid = 1'b0;
    in_data = 8'bx;
    monitor_on = 1'b0;
    response_pending = 1'b0;
    diagnosed = 1'b0;
    file_mode = 1'b1;
    file_handle = 0;
    input_path = "../00_TESTBED/input.txt";
    pattern_idx = 0;
    beat_idx = 0;
    output_idx = 0;
    golden_count = 0;
    total_latency = 64'd0;
    total_bytes = 64'd0;
    last_posedge_time = 0.0;
    input_end_time = 0.0;
    pattern_num = BALANCED_PATTERNS;
    image_nonzero_ac = 0;
    high_class_count = 0;
    low_class_count = 0;
    extra_smooth_count = 0;
    initial_seed = 2026;
    log_every = 1;
    scan_status = $value$plusargs("SEED=%d", initial_seed);
    scan_status = $value$plusargs("LOG_EVERY=%d", log_every);
    data_seed = initial_seed;
    gap_seed = initial_seed ^ 32'h13579bdf;
    for (init_idx = 0; init_idx <= 6; init_idx = init_idx + 1) gap_hist[init_idx] = 0;
    initialize_tables;
    clear_coverage;
    golden_self_check;
    negative_tie_self_check;

    if ((CYCLE <= 0.0) || (CYCLE > 20.0) ||
        ((CYCLE * 10.0 - $rtoi(CYCLE * 10.0 + 0.5)) > 1.0e-8) ||
        (($rtoi(CYCLE * 10.0 + 0.5) - CYCLE * 10.0) > 1.0e-8)) begin
        stop_test("CONFIG: clock period must be <=20 ns with 0.1 ns precision");
    end
    if ((MAX_LATENCY < 1) || (MAX_LATENCY > 50000) || (RESET_LOW_NS != 100.0) ||
        (RESET_RECOVERY_CYCLES < 2) || (GAP_MIN != 3) || (GAP_MAX != 6) ||
        (DIRECTED_PATTERNS != 49) || (RANDOM_PATTERNS != 32) ||
        (BALANCED_PATTERNS != 100) || (AC_DENSE_THRESHOLD != 4032) || (MAX_BYTES < 65536)) begin
        stop_test("CONFIG: do not relax Lab05 limits/reset/gap or reduce golden buffer");
    end
    if (log_every < 1) begin
        stop_test("CONFIG: LOG_EVERY must be positive");
    end

    builtin_mode = 0;
    scan_status = $value$plusargs("BUILTIN=%d", builtin_mode);
    if (builtin_mode == 1) begin
        file_mode = 1'b0;
        scan_status = $value$plusargs("PATNUM=%d", pattern_num);
    end
    else begin
        file_mode = 1'b1;
        scan_status = $value$plusargs("INPUT_FILE=%s", input_path);
        file_handle = $fopen(input_path, "r");
        if (file_handle == 0) stop_test("INPUT_FILE: cannot open ../00_TESTBED/input.txt or selected path");
        prepare_input_file;
    end
    if ((pattern_num < 1) || (pattern_num > 1000000)) begin
        stop_test("CONFIG: require 1..1000000 images");
    end
    $display("LAB05: images=%0d cycle=%0.1f ns reset_low=100 ns seed=%0d file_mode=%0d",
             pattern_num, CYCLE, initial_seed, file_mode);
    $display("Local response wait limit = %0d cycles (TA official limit: 50000 cycles)", MAX_LATENCY);

    $display("Image groups: HIGH if nonzero quantized AC >=4032/16128 (25 percent), otherwise LOW");
    reset_task;
    // No exact initial gap was imposed by the TA. Give four startup negedges.
    repeat (4) @(negedge clk);
    #0;
    for (pattern_idx = 0; pattern_idx < pattern_num; pattern_idx = pattern_idx + 1) begin
        if (file_mode === 1'b1) begin
            read_image_task;
        end
        else begin
            generate_image_task;
        end
        compute_golden_task;
        if (image_nonzero_ac >= AC_DENSE_THRESHOLD) high_class_count = high_class_count + 1;
        else low_class_count = low_class_count + 1;
        send_input_task;
        check_response_task;
        total_latency = total_latency + last_byte_cycle;
        total_bytes = total_bytes + golden_count;
        if ((((pattern_idx + 1) / log_every) * log_every == pattern_idx + 1) ||
            (pattern_idx + 1 == pattern_num)) begin
            $display("PASS image=%0d/%0d bytes=%0d latency=%0d cycles total_latency=%0d cycles",
                     pattern_idx + 1, pattern_num, golden_count, last_byte_cycle, total_latency);
        end
        if (pattern_idx + 1 < pattern_num) begin
            // check_response_task returns at N1, the first negedge after
            // out_valid fell following the FINAL byte, not an internal bubble.
            // Uniform two-bit draw gives N3, N4, N5 or N6, without modulo bias.
            gap_draw = $random(gap_seed);
            gap_cycles = GAP_MIN + gap_draw;
            gap_hist[gap_cycles] = gap_hist[gap_cycles] + 1;
            repeat (gap_cycles - 1) begin
                @(negedge clk);
                #0;
                check_environment;
            end
        end
    end
    if (file_mode === 1'b1) begin
        scan_status = $fscanf(file_handle, "%s", input_token);
        if (scan_status == 1) stop_test("INPUT_FILE: unexpected extra pixel after final image");
        $fclose(file_handle);
    end
    repeat (6) begin
        @(negedge clk);
        #0;
        check_environment;
    end
    if ((file_mode === 1'b0) && (pattern_num == BALANCED_PATTERNS) &&
        ((high_class_count != 50) || (low_class_count != 50))) begin
        $display("Actual image groups: HIGH=%0d LOW=%0d", high_class_count, low_class_count);
        stop_test("STIMULUS_BALANCE: complete built-in suite did not achieve 50 high / 50 low images");
    end
    display_pass_summary;
    $finish;
end

// A finite watchdog covers a stalled response, driver or clock.
initial begin
    wait (clock_on === 1'b1);
    #(CYCLE * (pattern_num * (16384.0 + MAX_LATENCY + 12.0) + 200.0));
    stop_test("WATCHDOG: simulation did not terminate in bounded time");
end



// ===============================================================
// Tasks
// ===============================================================

task stop_test;
    input [8*192-1:0] message;
    begin
        if (diagnosed !== 1'b1) begin
            diagnosed = 1'b1;
            $display("\nLAB05 CHECK FAILED: %0s", message);
            $display("image=%0d input_beat=%0d output_byte=%0d/%0d time=%0t",
                     pattern_idx + 1, beat_idx, output_idx, golden_count, $time);
            if (file_mode === 1'b1) $display("input_file=%0s", input_path);
            $fatal(1, "PATTERN terminated immediately");
        end
    end
endtask

task reset_task;
    begin
        force clk = 1'b0;
        #(CYCLE);
        rst_n = 1'b0;
        // TA clarification: exactly 100 ns low, independent of clock period.
        // Clock remains stopped: a synchronous-only reset cannot pass this.
        #(RESET_LOW_NS);
        if ((out_valid !== 1'b0) || (out_data !== 8'd0)) begin
            stop_test("RESET: outputs must be zero after the 100 ns asynchronous reset hold");
        end
        rst_n = 1'b1;
        monitor_on = 1'b1;
        #(RESET_RECOVERY_CYCLES * CYCLE);
        if ((out_valid !== 1'b0) || (out_data !== 8'd0)) begin
            stop_test("RESET_RECOVERY: outputs must stay zero while clock is stopped");
        end
        release clk;
        clock_on = 1'b1;
    end
endtask

task check_environment;
    begin
        if ((out_valid !== 1'b0) && (out_valid !== 1'b1)) begin
            stop_test("UNKNOWN_VALID: out_valid contains X/Z");
        end
        if ((out_valid === 1'b0) && (out_data !== 8'd0)) begin
            stop_test("IDLE_ZERO: out_data must be exactly zero when out_valid=0");
        end
        if ((in_valid === 1'b1) && (out_valid !== 1'b0)) begin
            stop_test("IO_OVERLAP: output asserted during input");
        end
        if ((out_valid === 1'b1) && (response_pending !== 1'b1)) begin
            stop_test("UNEXPECTED_OUTPUT: output asserted outside a pending response");
        end
        if ((out_valid === 1'b1) && ((^out_data) === 1'bx)) begin
            stop_test("UNKNOWN_DATA: encoded byte contains X/Z");
        end
    end
endtask

task send_input_task;
    begin
        // Caller is at a negedge. All 16384 pixels are valid consecutively.
        check_environment;
        response_pending = 1'b0;
        in_valid = 1'b1;
        for (beat_idx = 0; beat_idx < 16384; beat_idx = beat_idx + 1) begin
            in_data = image_data[beat_idx];
            @(negedge clk);
            #0;
            check_environment;
        end
        input_end_time = $realtime;
        response_pending = 1'b1;
        in_valid = 1'b0;
        in_data = 8'bx;
    end
endtask

task check_response_task;
    begin
        output_idx = 0;
        // TA E1 is this very negedge where in_valid becomes low: cycle ONE.
        // E5/E6/E8 outputs therefore produce a latency of E8 = 8 cycles.
        latency_cycles = 1;
        while (output_idx < golden_count) begin
            check_environment;
            if (latency_cycles > MAX_LATENCY) begin
                $display("Received %0d of %0d bytes; response wait limit=%0d cycles",
                         output_idx, golden_count, MAX_LATENCY);
                stop_test("LATENCY_OR_SHORT_OUTPUT: final byte exceeded the configured response wait limit");
            end
            if (out_valid === 1'b1) begin
                if (out_data !== golden_byte[output_idx]) begin
                    $display("Byte mismatch: index=%0d expected=%02h observed=%02h completed_in_block=%0d",
                             output_idx, golden_byte[output_idx], out_data, byte_block[output_idx]);
                    stop_test("BITSTREAM: output differs from the exact integer golden stream");
                end
                output_idx = output_idx + 1;
                last_byte_cycle = latency_cycles;
            end
            // An internal low out_valid is a legal bubble, not end of image.
            if (output_idx < golden_count) begin
                if (latency_cycles == MAX_LATENCY) begin
                    $display("Received %0d of %0d bytes; response wait limit=%0d cycles",
                             output_idx, golden_count, MAX_LATENCY);
                    stop_test("LATENCY_OR_SHORT_OUTPUT: wait limit reached before the final byte");
                end
                @(negedge clk);
                #0;
                latency_cycles = latency_cycles + 1;
            end
        end
        @(negedge clk);
        #0;
        check_environment;
        if (out_valid !== 1'b0) begin
            stop_test("EXTRA_OUTPUT: more bytes than the encoded image requires");
        end
        // Subsequent output pulses are rejected by the continuous monitors.
        response_pending = 1'b0;
    end
endtask

// Use signed 64-bit arithmetic throughout DCT accumulation and rounding.
function signed [63:0] round_signed;
    input signed [63:0] value;
    input integer shift;
    reg signed [63:0] magnitude, rounded;
    begin
        if (value < 0) magnitude = -value;
        else magnitude = value;
        rounded = (magnitude + (64'sd1 << (shift - 1))) >>> shift;
        if (value < 0) round_signed = -rounded;
        else round_signed = rounded;
    end
endfunction

function integer magnitude_size;
    input integer value;
    integer magnitude;
    begin
        if (value < 0) magnitude = -value;
        else magnitude = value;
        magnitude_size = 0;
        while (magnitude != 0) begin
            magnitude_size = magnitude_size + 1;
            magnitude = magnitude >> 1;
        end
    end
endfunction

task reset_model;
    begin
        golden_count = 0;
        partial_byte = 8'd0;
        partial_bits = 0;
        encoded_bits = 0;
        model_previous_dc = 0;
    end
endtask

task append_field;
    input [31:0] value;
    input integer length;
    integer bit_idx;
    begin
        if ((length < 0) || (length > 16)) stop_test("GOLDEN_MODEL: invalid field length");
        for (bit_idx = length - 1; bit_idx >= 0; bit_idx = bit_idx - 1) begin
            partial_byte = (partial_byte << 1) | value[bit_idx];
            partial_bits = partial_bits + 1;
            encoded_bits = encoded_bits + 1;
            if (partial_bits == 8) begin
                if (golden_count >= MAX_BYTES) stop_test("GOLDEN_MODEL: byte buffer overflow");
                golden_byte[golden_count] = partial_byte;
                byte_block[golden_count] = model_block;
                if ((coverage_on != 0) && (partial_byte == 8'hff)) count_raw_ff = count_raw_ff + 1;
                golden_count = golden_count + 1;
                partial_byte = 8'd0;
                partial_bits = 0;
            end
        end
    end
endtask

task append_amplitude;
    input integer value;
    input integer size;
    integer amplitude;
    begin
        if (size > 0) begin
            if (value < 0) amplitude = (1 << size) - 1 + value;
            else amplitude = value;
            append_field(amplitude, size);
            if (coverage_on != 0) begin
                if (value < 0) count_negative = count_negative + 1;
                else count_positive = count_positive + 1;
            end
        end
    end
endtask

task emit_ac_symbol;
    input integer symbol;
    begin
        if ((symbol < 0) || (symbol > 255) || (ac_length[symbol] == 0)) begin
            stop_test("GOLDEN_MODEL: AC symbol is absent from the fixed LUT");
        end
        if (coverage_on != 0) ac_symbol_hist[symbol] = ac_symbol_hist[symbol] + 1;
        append_field(ac_code[symbol], ac_length[symbol]);
    end
endtask

task encode_block;
    integer idx, delta_dc, size, run, symbol, signed_bin, burst;
    begin
        if (coverage_on != 0) block_offset_hist[partial_bits] = block_offset_hist[partial_bits] + 1;
        delta_dc = zigzag_coeff[0] - model_previous_dc;
        model_previous_dc = zigzag_coeff[0];
        size = magnitude_size(delta_dc);
        if (size > 11) stop_test("GOLDEN_MODEL: DC SIZE exceeds the fixed LUT");
        append_field(dc_code[size], dc_length[size]);
        append_amplitude(delta_dc, size);
        if (coverage_on != 0) begin
            dc_hist[size] = dc_hist[size] + 1;
            signed_bin = size + ((delta_dc < 0) ? 9 : 0);
            if (size <= 8) dc_signed_hist[signed_bin] = dc_signed_hist[signed_bin] + 1;
        end
        run = 0;
        for (idx = 1; idx < 64; idx = idx + 1) begin
            if (zigzag_coeff[idx] == 0) begin
                run = run + 1;
            end
            else begin
                if (coverage_on != 0) begin
                    burst = run >> 4;
                    zrl_burst_hist[burst] = zrl_burst_hist[burst] + 1;
                end
                // Emit ZRL only when a later nonzero coefficient is reached.
                while (run >= 16) begin
                    emit_ac_symbol(8'hf0);
                    run = run - 16;
                    if (coverage_on != 0) count_zrl = count_zrl + 1;
                end
                size = magnitude_size(zigzag_coeff[idx]);
                if (size > 10) stop_test("GOLDEN_MODEL: AC SIZE exceeds the fixed LUT");
                symbol = (run << 4) | size;
                emit_ac_symbol(symbol);
                append_amplitude(zigzag_coeff[idx], size);
                if (coverage_on != 0) begin
                    ac_hist[size] = ac_hist[size] + 1;
                    signed_bin = size + ((zigzag_coeff[idx] < 0) ? 8 : 0);
                    if (size <= 7) ac_signed_hist[signed_bin] = ac_signed_hist[signed_bin] + 1;
                    run_hist[run] = run_hist[run] + 1;
                end
                run = 0;
            end
        end
        if (coverage_on != 0) trailing_hist[run] = trailing_hist[run] + 1;
        if (run > 0) begin
            emit_ac_symbol(8'h00);
            if (coverage_on != 0) count_eob = count_eob + 1;
        end
        else begin
            if (coverage_on != 0) count_last_ac = count_last_ac + 1;
        end
        if ((coverage_on != 0) && (partial_bits != 0)) begin
            count_unaligned_blocks = count_unaligned_blocks + 1;
        end
    end
endtask

task transform_block;
    input integer block_id;
    integer bx, by, x, y, u, v, idx, remainder, boundary, sign_bin;
    reg signed [63:0] sample, accumulator, merged;
    begin
        bx = block_id & 15;
        by = block_id >> 4;
        for (y = 0; y < 8; y = y + 1) begin
            for (u = 0; u < 8; u = u + 1) begin
                accumulator = 64'sd0;
                for (x = 0; x < 8; x = x + 1) begin
                    sample = image_data[(by*8+y)*128 + bx*8+x];
                    sample = sample - 64'sd128;
                    accumulator = accumulator + sample * cosine[u*8+x];
                end
                row_dct[y*8+u] = accumulator; // No intermediate truncation/rounding.
            end
        end
        for (v = 0; v < 8; v = v + 1) begin
            for (u = 0; u < 8; u = u + 1) begin
                accumulator = 64'sd0;
                for (y = 0; y < 8; y = y + 1) begin
                    accumulator = accumulator + row_dct[y*8+u] * cosine[v*8+y];
                end
                idx = v*8+u; // Table convention: row v, column u.
                dct_coeff[idx] = round_signed(accumulator, 18);
                quant_coeff[idx] = round_signed(dct_coeff[idx], quant_shift[idx]);
                if ((coverage_on != 0) && (idx != 0) && (quant_coeff[idx] != 0)) begin
                    image_nonzero_ac = image_nonzero_ac + 1;
                end
                if ((coverage_on != 0) && (dct_coeff[idx] < 0) &&
                    (((-dct_coeff[idx]) & ((1 << quant_shift[idx]) - 1)) ==
                     (1 << (quant_shift[idx] - 1)))) begin
                    negative_half_hist[quant_shift[idx]] = negative_half_hist[quant_shift[idx]] + 1;
                end
                if (coverage_on != 0) begin
                    if (quant_coeff[idx] != 0) begin
                        sign_bin = idx*2 + ((quant_coeff[idx] < 0) ? 1 : 0);
                        coefficient_sign_hist[sign_bin] = coefficient_sign_hist[sign_bin] + 1;
                    end
                    if (accumulator < 0) remainder = (-accumulator) & 262143;
                    else remainder = accumulator & 262143;
                    if ((remainder >= 131071) && (remainder <= 131073)) begin
                        boundary = ((accumulator < 0) ? 3 : 0) + remainder - 131071;
                        first_boundary_hist[boundary] = first_boundary_hist[boundary] + 1;
                    end
                    remainder = ((dct_coeff[idx] < 0) ? -dct_coeff[idx] : dct_coeff[idx]) & ((1 << quant_shift[idx]) - 1);
                    boundary = (1 << (quant_shift[idx] - 1));
                    if ((remainder >= boundary-1) && (remainder <= boundary+1)) begin
                        sign_bin = (quant_shift[idx]-3)*6 + ((dct_coeff[idx] < 0) ? 3 : 0) + remainder-boundary+1;
                        quant_boundary_hist[sign_bin] = quant_boundary_hist[sign_bin] + 1;
                    end
                end
                merged = round_signed(accumulator, 18 + quant_shift[idx]);
                if ((coverage_on != 0) && (merged != quant_coeff[idx])) begin
                    count_double_round = count_double_round + 1;
                end
            end
        end
        for (idx = 0; idx < 64; idx = idx + 1) begin
            zigzag_coeff[idx] = quant_coeff[zigzag_address[idx]];
        end
    end
endtask

task finish_model;
    integer padding;
    begin
        if (coverage_on != 0) pad_hist[partial_bits] = pad_hist[partial_bits] + 1;
        if (partial_bits > 0) begin
            padding = 8 - partial_bits;
            append_field((1 << padding) - 1, padding);
        end
    end
endtask

task compute_golden_task;
    begin
        image_nonzero_ac = 0;
        reset_model;
        for (model_block = 0; model_block < 256; model_block = model_block + 1) begin
            transform_block(model_block);
            encode_block;
        end
        model_block = 255;
        finish_model;
        if (golden_count < 1) stop_test("GOLDEN_MODEL: an image cannot have an empty stream");
    end
endtask

function integer constant_pixel;
    input integer index;
    begin
        case (index & 15)
            0: constant_pixel = 0;   1: constant_pixel = 1;
            2: constant_pixel = 7;   3: constant_pixel = 8;
            4: constant_pixel = 63;  5: constant_pixel = 64;
            6: constant_pixel = 88;  7: constant_pixel = 96;
            8: constant_pixel = 127; 9: constant_pixel = 128;
           10: constant_pixel = 129;11: constant_pixel = 136;
           12: constant_pixel = 200;13: constant_pixel = 247;
           14: constant_pixel = 254;15: constant_pixel = 255;
            default: constant_pixel = 128;
        endcase
    end
endfunction

function integer negative_tie_case;
    input integer block_id;
    begin
        // Spread fixtures across block-row boundaries and the final block.
        case (block_id)
              0: negative_tie_case = 0;
              1: negative_tie_case = 1;
              2: negative_tie_case = 2;
             15: negative_tie_case = 3;
             16: negative_tie_case = 4;
             17: negative_tie_case = 5;
            127: negative_tie_case = 6;
            128: negative_tie_case = 7;
            129: negative_tie_case = 8;
            239: negative_tie_case = 9;
            240: negative_tie_case = 10;
            241: negative_tie_case = 11;
            253: negative_tie_case = 12;
            254: negative_tie_case = 13;
            255: negative_tie_case = 14;
            default: negative_tie_case = -1;
        endcase
    end
endfunction

// Keep the original generator intact so all previous vectors and RNG draws
// remain unchanged. New random draws occur only AFTER original image 81.
task generate_image_task;
    integer idx, x, y, dx, dy, value, random_value, smooth_kind;
    integer block_id, previous_block;
    integer block_tone [0:255];
    begin
        if (pattern_idx < DIRECTED_PATTERNS + RANDOM_PATTERNS) begin
            generate_original_image_task;
        end
        else if ((pattern_idx < BALANCED_PATTERNS) && (high_class_count >= 50)) begin
            // The high group is complete; fill the remaining low-load quota.
            // Eight deterministic smooth families cover both spatial axes,
            // curved gradients, block colors and low-frequency cosine content.
            smooth_kind = extra_smooth_count & 7;
            if (smooth_kind == 5) begin
                for (block_id = 0; block_id < 256; block_id = block_id + 1) begin
                    random_value = $random(data_seed);
                    block_tone[block_id] = random_value & 255;
                end
            end
            for (idx = 0; idx < 16384; idx = idx + 1) begin
                x = idx & 127; y = idx >> 7;
                dx = x - 64; dy = y - 64;
                if (dx < 0) dx = -dx;
                if (dy < 0) dy = -dy;
                case (smooth_kind)
                    0: value = 255 - x*2;
                    1: value = 255 - y*2;
                    2: value = x + 127 - y;
                    3: value = dx + dy;
                    4: value = ((x-64)*(x-64) + (y-64)*(y-64)) >> 5;
                    5: value = block_tone[(y >> 3)*16 + (x >> 3)];
                    6: begin
                        if (x < 64) value = 64 + x*2;
                        else value = 64 + (127-x)*2;
                    end
                    default: value = 128 + cosine[8+(x & 7)]/8 + cosine[8+(y & 7)]/16;
                endcase
                if (value < 0) value = 0;
                else if (value > 255) value = 255;
                image_data[idx] = value;
            end
            extra_smooth_count = extra_smooth_count + 1;
        end
        else begin
            // Independent full-range random pixels fill the high-load quota.
            for (idx = 0; idx < 16384; idx = idx + 1) begin
                random_value = $random(data_seed);
                image_data[idx] = random_value & 255;
            end
        end
    end
endtask

task generate_original_image_task;
    integer idx, x, y, lx, ly, block_id, value, random_value, basis, u, v, amplitude, tie_idx;
    begin
        for (idx = 0; idx < 16384; idx = idx + 1) begin
            x = idx & 127; y = idx >> 7;
            lx = x & 7; ly = y & 7;
            block_id = (y >> 3)*16 + (x >> 3);
            value = 128;
            if (pattern_idx < 16) begin
                value = constant_pixel(pattern_idx);
            end
            else begin
                case (pattern_idx)
                    16: value = block_id; // All 256 constant levels, block-raster order.
                    17: if (block_id & 1) value = 255; else value = 0;
                    18: value = constant_pixel((block_id*7 + (block_id >> 4)) & 15);
                    19: if ((x ^ y) & 1) value = 255; else value = 0;
                    20: if (x & 1) value = 255; else value = 0;
                    21: if (y & 1) value = 255; else value = 0;
                    22: if (x < 64) value = 0; else value = 255;
                    23: if (y < 64) value = 255; else value = 0;
                    24: value = x*2;
                    25: value = y*2;
                    26: value = x+y;
                    27: value = lx*32;
                    28: value = ly*32;
                    29: if (idx == 0) value = 255; else value = 128;
                    30: if (idx == 16383) value = 0; else value = 128;
                    31: begin
                        if ((lx == (block_id & 7)) && (ly == ((block_id >> 3) & 7))) begin
                            if (block_id & 1) value = 255; else value = 0;
                        end
                        else value = 128;
                    end
                    32: begin // The supplied trace's first three blocks.
                        if (block_id == 0) value = 88;
                        else if (block_id == 1) value = example_row[ly][(7-lx)*8 +: 8];
                        else if (block_id == 2) value = example_row[8+ly][(7-lx)*8 +: 8];
                        else value = 128;
                    end
                    33: begin // All 64 cosine bases, with both signs/amplitudes.
                        u = block_id & 7; v = (block_id >> 3) & 7;
                        basis = cosine[u*8+lx] * cosine[v*8+ly];
                        amplitude = 64 + 32*(block_id >> 6);
                        value = 128 + basis*amplitude/65536;
                        if (block_id & 64) value = 256-value;
                    end
                    34: begin random_value = $random(data_seed); value = 127 + (random_value & 3); end
                    35: begin random_value = $random(data_seed); value = 120 + (random_value & 15); end
                    36: begin random_value = $random(data_seed); if (random_value & 1) value = 255; else value = 0; end
                    37: begin random_value = $random(data_seed); value = (random_value & 15)*17; end
                    38: value = (y >> 3)*17;
                    39: value = (x >> 3)*17;
                    40: if (lx == ly) value = 255; else value = 0;
                    41: if ((lx == 7) && (ly == 7)) value = 255; else value = 0;
                    42: if ((lx < 4) ^ (ly < 4)) value = 255; else value = 0;
                    43: begin
                        case (block_id & 3)
                            0: value = constant_pixel(block_id >> 2);
                            1: if ((lx ^ ly) & 1) value = 255; else value = 0;
                            2: value = lx*32 + ly*4;
                            default: begin random_value = $random(data_seed); value = random_value & 255; end
                        endcase
                    end
                    44: if (lx == 0) value = 255; else value = 128;
                    45: if (ly == 7) value = 0; else value = 128;
                    46: begin // Low-amplitude bases near quantization thresholds.
                        u = block_id & 7; v = (block_id >> 3) & 7;
                        basis = cosine[u*8+lx] * cosine[v*8+ly];
                        amplitude = 1 << ((block_id >> 6) + 1);
                        value = 128 + basis*amplitude/65536;
                    end
                    47: begin // Transposed/asymmetric official example across all blocks.
                        if (block_id & 1) value = example_row[lx][(7-ly)*8 +: 8];
                        else value = example_row[8+ly][(7-lx)*8 +: 8];
                    end
                    48: begin
                        // Three exact negative halfway cases for EACH shift 3..7.
                        // They produce correct Q=-1/-2/-3; positive-bias shifting
                        // would incorrectly produce Q=0/-1/-2 instead.
                        tie_idx = negative_tie_case(block_id);
                        if (tie_idx >= 0) value = negative_tie_row[tie_idx*8+ly][(7-lx)*8 +: 8];
                        else value = 128;
                    end
                    default: begin random_value = $random(data_seed); value = random_value & 255; end
                endcase
            end
            if (value < 0) value = 0;
            else if (value > 255) value = 255;
            image_data[idx] = value;
        end
    end
endtask

task decode_pixel_token;
    input [8*64-1:0] token;
    output integer value;
    integer character_idx, digit;
    reg [7:0] character;
    begin
        if ((token[8*64-1:16] != 0) || (token[15:8] == 0)) begin
            stop_test("INPUT_FILE: each pixel must contain exactly two hex digits, without a header");
        end
        value = 0;
        for (character_idx = 1; character_idx >= 0; character_idx = character_idx - 1) begin
            character = token[character_idx*8 +: 8];
            if ((character >= "0") && (character <= "9")) digit = character - "0";
            else if ((character >= "A") && (character <= "F")) digit = character - "A" + 10;
            else if ((character >= "a") && (character <= "f")) digit = character - "a" + 10;
            else begin
                digit = 0;
                stop_test("INPUT_FILE: illegal pixel digit; X/Z/signs/prefixes are not accepted");
            end
            value = (value << 4) | digit;
        end
    end
endtask

// Header files stream once; legacy headerless files use the validated counting pass.
task prepare_input_file;
    begin
        input_token = 0;
        scan_status = $fscanf(file_handle, "%s", input_token);
        if (scan_status != 1) stop_test("INPUT_FILE: empty file");
        if (input_token == "LAB05_HEX_V1") begin
            input_token = 0;
            scan_status = $fscanf(file_handle, "%s", input_token);
            if ((scan_status != 1) || (input_token == 0)) stop_test("INPUT_FILE: missing image count");
            if (input_token[8*64-1:56] != 0) stop_test("INPUT_FILE: image count is too long");
            pattern_num = 0;
            for (count_char = 6; count_char >= 0; count_char = count_char - 1) begin
                count_digit = (input_token >> (count_char*8)) & 255;
                if (count_digit != 0) begin
                    if ((count_digit < "0") || (count_digit > "9")) begin
                        stop_test("INPUT_FILE: image count must be unsigned decimal digits");
                    end
                    pattern_num = pattern_num*10 + count_digit - "0";
                end
            end
            if ((pattern_num < 1) || (pattern_num > 1000000)) stop_test("INPUT_FILE: image count must be 1..1000000");
        end
        else begin
            $fclose(file_handle);
            file_handle = $fopen(input_path, "r");
            if (file_handle == 0) stop_test("INPUT_FILE: cannot reopen legacy file");
            count_file_patterns;
            $fclose(file_handle);
            file_handle = $fopen(input_path, "r");
            if (file_handle == 0) stop_test("INPUT_FILE: cannot reopen legacy pixels");
        end
    end
endtask

task count_file_patterns;
    begin
        file_pixel_count = 64'd0;
        input_token = 0;
        scan_status = $fscanf(file_handle, "%s", input_token);
        while (scan_status == 1) begin
            decode_pixel_token(input_token, pixel_from_file);
            file_pixel_count = file_pixel_count + 1;
            if (file_pixel_count > 64'd16384000000) stop_test("INPUT_FILE: more than one million images");
            input_token = 0;
            scan_status = $fscanf(file_handle, "%s", input_token);
        end
        if (!$feof(file_handle)) stop_test("INPUT_FILE: unreadable token");
        pattern_num = file_pixel_count / 16384;
        if ((pattern_num < 1) || (pattern_num*64'd16384 != file_pixel_count)) begin
            stop_test("INPUT_FILE: require a nonempty whole number of 128x128 images");
        end
    end
endtask

task read_image_task;
    integer idx;
    begin
        for (idx = 0; idx < 16384; idx = idx + 1) begin
            input_token = 0;
            scan_status = $fscanf(file_handle, "%s", input_token);
            if (scan_status != 1) stop_test("INPUT_FILE: EOF inside an image");
            decode_pixel_token(input_token, pixel_from_file);
            image_data[idx] = pixel_from_file;
        end
    end
endtask

task clear_coverage;
    integer idx;
    begin
        for (idx = 0; idx < 12; idx = idx + 1) dc_hist[idx] = 0;
        for (idx = 0; idx < 11; idx = idx + 1) ac_hist[idx] = 0;
        for (idx = 0; idx < 16; idx = idx + 1) run_hist[idx] = 0;
        for (idx = 0; idx < 8; idx = idx + 1) pad_hist[idx] = 0;
        for (idx = 0; idx < 8; idx = idx + 1) negative_half_hist[idx] = 0;
        count_eob = 0; count_zrl = 0; count_last_ac = 0;
        count_positive = 0; count_negative = 0;
        count_double_round = 0; count_raw_ff = 0; count_unaligned_blocks = 0;
        for (idx = 0; idx < 256; idx = idx + 1) ac_symbol_hist[idx] = 0;
        for (idx = 0; idx < 18; idx = idx + 1) dc_signed_hist[idx] = 0;
        for (idx = 0; idx < 16; idx = idx + 1) ac_signed_hist[idx] = 0;
        for (idx = 0; idx < 8; idx = idx + 1) block_offset_hist[idx] = 0;
        for (idx = 0; idx < 64; idx = idx + 1) trailing_hist[idx] = 0;
        for (idx = 0; idx < 4; idx = idx + 1) zrl_burst_hist[idx] = 0;
        for (idx = 0; idx < 6; idx = idx + 1) first_boundary_hist[idx] = 0;
        for (idx = 0; idx < 30; idx = idx + 1) quant_boundary_hist[idx] = 0;
        for (idx = 0; idx < 128; idx = idx + 1) coefficient_sign_hist[idx] = 0;
        coverage_on = 1;
    end
endtask

task initialize_tables;
    integer idx;
    begin
        // Every entry below is the exact supplied TA value, not a computed JPEG variant.
        for (idx = 0; idx < 256; idx = idx + 1) begin
            ac_length[idx] = 0; ac_code[idx] = 16'd0;
        end
        cosine[0] = 181; cosine[1] = 181; cosine[2] = 181; cosine[3] = 181; cosine[4] = 181; cosine[5] = 181; cosine[6] = 181; cosine[7] = 181;
        cosine[8] = 251; cosine[9] = 213; cosine[10] = 142; cosine[11] = 50; cosine[12] = -50; cosine[13] = -142; cosine[14] = -213; cosine[15] = -251;
        cosine[16] = 237; cosine[17] = 98; cosine[18] = -98; cosine[19] = -237; cosine[20] = -237; cosine[21] = -98; cosine[22] = 98; cosine[23] = 237;
        cosine[24] = 213; cosine[25] = -50; cosine[26] = -251; cosine[27] = -142; cosine[28] = 142; cosine[29] = 251; cosine[30] = 50; cosine[31] = -213;
        cosine[32] = 181; cosine[33] = -181; cosine[34] = -181; cosine[35] = 181; cosine[36] = 181; cosine[37] = -181; cosine[38] = -181; cosine[39] = 181;
        cosine[40] = 142; cosine[41] = -251; cosine[42] = 50; cosine[43] = 213; cosine[44] = -213; cosine[45] = -50; cosine[46] = 251; cosine[47] = -142;
        cosine[48] = 98; cosine[49] = -237; cosine[50] = 237; cosine[51] = -98; cosine[52] = -98; cosine[53] = 237; cosine[54] = -237; cosine[55] = 98;
        cosine[56] = 50; cosine[57] = -142; cosine[58] = 213; cosine[59] = -251; cosine[60] = 251; cosine[61] = -213; cosine[62] = 142; cosine[63] = -50;
        quant_shift[0] = 4;
        quant_shift[1] = 3;
        quant_shift[2] = 3;
        quant_shift[3] = 4;
        quant_shift[4] = 5;
        quant_shift[5] = 5;
        quant_shift[6] = 6;
        quant_shift[7] = 6;
        quant_shift[8] = 4;
        quant_shift[9] = 4;
        quant_shift[10] = 4;
        quant_shift[11] = 4;
        quant_shift[12] = 5;
        quant_shift[13] = 6;
        quant_shift[14] = 6;
        quant_shift[15] = 6;
        quant_shift[16] = 4;
        quant_shift[17] = 4;
        quant_shift[18] = 4;
        quant_shift[19] = 5;
        quant_shift[20] = 5;
        quant_shift[21] = 6;
        quant_shift[22] = 6;
        quant_shift[23] = 6;
        quant_shift[24] = 4;
        quant_shift[25] = 4;
        quant_shift[26] = 4;
        quant_shift[27] = 5;
        quant_shift[28] = 6;
        quant_shift[29] = 6;
        quant_shift[30] = 6;
        quant_shift[31] = 6;
        quant_shift[32] = 4;
        quant_shift[33] = 4;
        quant_shift[34] = 5;
        quant_shift[35] = 6;
        quant_shift[36] = 6;
        quant_shift[37] = 7;
        quant_shift[38] = 7;
        quant_shift[39] = 6;
        quant_shift[40] = 5;
        quant_shift[41] = 5;
        quant_shift[42] = 6;
        quant_shift[43] = 6;
        quant_shift[44] = 6;
        quant_shift[45] = 7;
        quant_shift[46] = 7;
        quant_shift[47] = 7;
        quant_shift[48] = 6;
        quant_shift[49] = 6;
        quant_shift[50] = 6;
        quant_shift[51] = 6;
        quant_shift[52] = 7;
        quant_shift[53] = 7;
        quant_shift[54] = 7;
        quant_shift[55] = 7;
        quant_shift[56] = 6;
        quant_shift[57] = 7;
        quant_shift[58] = 7;
        quant_shift[59] = 7;
        quant_shift[60] = 7;
        quant_shift[61] = 7;
        quant_shift[62] = 7;
        quant_shift[63] = 7;
        zigzag_address[0] = 0;
        zigzag_address[1] = 1;
        zigzag_address[2] = 8;
        zigzag_address[3] = 16;
        zigzag_address[4] = 9;
        zigzag_address[5] = 2;
        zigzag_address[6] = 3;
        zigzag_address[7] = 10;
        zigzag_address[8] = 17;
        zigzag_address[9] = 24;
        zigzag_address[10] = 32;
        zigzag_address[11] = 25;
        zigzag_address[12] = 18;
        zigzag_address[13] = 11;
        zigzag_address[14] = 4;
        zigzag_address[15] = 5;
        zigzag_address[16] = 12;
        zigzag_address[17] = 19;
        zigzag_address[18] = 26;
        zigzag_address[19] = 33;
        zigzag_address[20] = 40;
        zigzag_address[21] = 48;
        zigzag_address[22] = 41;
        zigzag_address[23] = 34;
        zigzag_address[24] = 27;
        zigzag_address[25] = 20;
        zigzag_address[26] = 13;
        zigzag_address[27] = 6;
        zigzag_address[28] = 7;
        zigzag_address[29] = 14;
        zigzag_address[30] = 21;
        zigzag_address[31] = 28;
        zigzag_address[32] = 35;
        zigzag_address[33] = 42;
        zigzag_address[34] = 49;
        zigzag_address[35] = 56;
        zigzag_address[36] = 57;
        zigzag_address[37] = 50;
        zigzag_address[38] = 43;
        zigzag_address[39] = 36;
        zigzag_address[40] = 29;
        zigzag_address[41] = 22;
        zigzag_address[42] = 15;
        zigzag_address[43] = 23;
        zigzag_address[44] = 30;
        zigzag_address[45] = 37;
        zigzag_address[46] = 44;
        zigzag_address[47] = 51;
        zigzag_address[48] = 58;
        zigzag_address[49] = 59;
        zigzag_address[50] = 52;
        zigzag_address[51] = 45;
        zigzag_address[52] = 38;
        zigzag_address[53] = 31;
        zigzag_address[54] = 39;
        zigzag_address[55] = 46;
        zigzag_address[56] = 53;
        zigzag_address[57] = 60;
        zigzag_address[58] = 61;
        zigzag_address[59] = 54;
        zigzag_address[60] = 47;
        zigzag_address[61] = 55;
        zigzag_address[62] = 62;
        zigzag_address[63] = 63;
        dc_length[0] = 2; dc_code[0] = 16'h0;
        dc_length[1] = 3; dc_code[1] = 16'h2;
        dc_length[2] = 3; dc_code[2] = 16'h3;
        dc_length[3] = 3; dc_code[3] = 16'h4;
        dc_length[4] = 3; dc_code[4] = 16'h5;
        dc_length[5] = 3; dc_code[5] = 16'h6;
        dc_length[6] = 4; dc_code[6] = 16'hE;
        dc_length[7] = 5; dc_code[7] = 16'h1E;
        dc_length[8] = 6; dc_code[8] = 16'h3E;
        dc_length[9] = 7; dc_code[9] = 16'h7E;
        dc_length[10] = 8; dc_code[10] = 16'hFE;
        dc_length[11] = 9; dc_code[11] = 16'h1FE;
        ac_length[8'h00] = 4; ac_code[8'h00] = 16'hA;
        ac_length[8'h01] = 2; ac_code[8'h01] = 16'h0;
        ac_length[8'h02] = 2; ac_code[8'h02] = 16'h1;
        ac_length[8'h03] = 3; ac_code[8'h03] = 16'h4;
        ac_length[8'h04] = 4; ac_code[8'h04] = 16'hB;
        ac_length[8'h05] = 5; ac_code[8'h05] = 16'h1A;
        ac_length[8'h06] = 7; ac_code[8'h06] = 16'h78;
        ac_length[8'h07] = 8; ac_code[8'h07] = 16'hF8;
        ac_length[8'h08] = 10; ac_code[8'h08] = 16'h3F6;
        ac_length[8'h09] = 16; ac_code[8'h09] = 16'hFF82;
        ac_length[8'h0A] = 16; ac_code[8'h0A] = 16'hFF83;
        ac_length[8'h11] = 4; ac_code[8'h11] = 16'hC;
        ac_length[8'h12] = 5; ac_code[8'h12] = 16'h1B;
        ac_length[8'h13] = 7; ac_code[8'h13] = 16'h79;
        ac_length[8'h14] = 9; ac_code[8'h14] = 16'h1F6;
        ac_length[8'h15] = 11; ac_code[8'h15] = 16'h7F6;
        ac_length[8'h16] = 16; ac_code[8'h16] = 16'hFF84;
        ac_length[8'h17] = 16; ac_code[8'h17] = 16'hFF85;
        ac_length[8'h18] = 16; ac_code[8'h18] = 16'hFF86;
        ac_length[8'h19] = 16; ac_code[8'h19] = 16'hFF87;
        ac_length[8'h1A] = 16; ac_code[8'h1A] = 16'hFF88;
        ac_length[8'h21] = 5; ac_code[8'h21] = 16'h1C;
        ac_length[8'h22] = 8; ac_code[8'h22] = 16'hF9;
        ac_length[8'h23] = 10; ac_code[8'h23] = 16'h3F7;
        ac_length[8'h24] = 12; ac_code[8'h24] = 16'hFF4;
        ac_length[8'h25] = 16; ac_code[8'h25] = 16'hFF89;
        ac_length[8'h26] = 16; ac_code[8'h26] = 16'hFF8A;
        ac_length[8'h27] = 16; ac_code[8'h27] = 16'hFF8B;
        ac_length[8'h28] = 16; ac_code[8'h28] = 16'hFF8C;
        ac_length[8'h29] = 16; ac_code[8'h29] = 16'hFF8D;
        ac_length[8'h2A] = 16; ac_code[8'h2A] = 16'hFF8E;
        ac_length[8'h31] = 6; ac_code[8'h31] = 16'h3A;
        ac_length[8'h32] = 9; ac_code[8'h32] = 16'h1F7;
        ac_length[8'h33] = 12; ac_code[8'h33] = 16'hFF5;
        ac_length[8'h34] = 16; ac_code[8'h34] = 16'hFF8F;
        ac_length[8'h35] = 16; ac_code[8'h35] = 16'hFF90;
        ac_length[8'h36] = 16; ac_code[8'h36] = 16'hFF91;
        ac_length[8'h37] = 16; ac_code[8'h37] = 16'hFF92;
        ac_length[8'h38] = 16; ac_code[8'h38] = 16'hFF93;
        ac_length[8'h39] = 16; ac_code[8'h39] = 16'hFF94;
        ac_length[8'h3A] = 16; ac_code[8'h3A] = 16'hFF95;
        ac_length[8'h41] = 6; ac_code[8'h41] = 16'h3B;
        ac_length[8'h42] = 10; ac_code[8'h42] = 16'h3F8;
        ac_length[8'h43] = 16; ac_code[8'h43] = 16'hFF96;
        ac_length[8'h44] = 16; ac_code[8'h44] = 16'hFF97;
        ac_length[8'h45] = 16; ac_code[8'h45] = 16'hFF98;
        ac_length[8'h46] = 16; ac_code[8'h46] = 16'hFF99;
        ac_length[8'h47] = 16; ac_code[8'h47] = 16'hFF9A;
        ac_length[8'h48] = 16; ac_code[8'h48] = 16'hFF9B;
        ac_length[8'h49] = 16; ac_code[8'h49] = 16'hFF9C;
        ac_length[8'h4A] = 16; ac_code[8'h4A] = 16'hFF9D;
        ac_length[8'h51] = 7; ac_code[8'h51] = 16'h7A;
        ac_length[8'h52] = 11; ac_code[8'h52] = 16'h7F7;
        ac_length[8'h53] = 16; ac_code[8'h53] = 16'hFF9E;
        ac_length[8'h54] = 16; ac_code[8'h54] = 16'hFF9F;
        ac_length[8'h55] = 16; ac_code[8'h55] = 16'hFFA0;
        ac_length[8'h56] = 16; ac_code[8'h56] = 16'hFFA1;
        ac_length[8'h57] = 16; ac_code[8'h57] = 16'hFFA2;
        ac_length[8'h58] = 16; ac_code[8'h58] = 16'hFFA3;
        ac_length[8'h59] = 16; ac_code[8'h59] = 16'hFFA4;
        ac_length[8'h5A] = 16; ac_code[8'h5A] = 16'hFFA5;
        ac_length[8'h61] = 7; ac_code[8'h61] = 16'h7B;
        ac_length[8'h62] = 12; ac_code[8'h62] = 16'hFF6;
        ac_length[8'h63] = 16; ac_code[8'h63] = 16'hFFA6;
        ac_length[8'h64] = 16; ac_code[8'h64] = 16'hFFA7;
        ac_length[8'h65] = 16; ac_code[8'h65] = 16'hFFA8;
        ac_length[8'h66] = 16; ac_code[8'h66] = 16'hFFA9;
        ac_length[8'h67] = 16; ac_code[8'h67] = 16'hFFAA;
        ac_length[8'h68] = 16; ac_code[8'h68] = 16'hFFAB;
        ac_length[8'h69] = 16; ac_code[8'h69] = 16'hFFAC;
        ac_length[8'h6A] = 16; ac_code[8'h6A] = 16'hFFAD;
        ac_length[8'h71] = 8; ac_code[8'h71] = 16'hFA;
        ac_length[8'h72] = 12; ac_code[8'h72] = 16'hFF7;
        ac_length[8'h73] = 16; ac_code[8'h73] = 16'hFFAE;
        ac_length[8'h74] = 16; ac_code[8'h74] = 16'hFFAF;
        ac_length[8'h75] = 16; ac_code[8'h75] = 16'hFFB0;
        ac_length[8'h76] = 16; ac_code[8'h76] = 16'hFFB1;
        ac_length[8'h77] = 16; ac_code[8'h77] = 16'hFFB2;
        ac_length[8'h78] = 16; ac_code[8'h78] = 16'hFFB3;
        ac_length[8'h79] = 16; ac_code[8'h79] = 16'hFFB4;
        ac_length[8'h7A] = 16; ac_code[8'h7A] = 16'hFFB5;
        ac_length[8'h81] = 9; ac_code[8'h81] = 16'h1F8;
        ac_length[8'h82] = 15; ac_code[8'h82] = 16'h7FC0;
        ac_length[8'h83] = 16; ac_code[8'h83] = 16'hFFB6;
        ac_length[8'h84] = 16; ac_code[8'h84] = 16'hFFB7;
        ac_length[8'h85] = 16; ac_code[8'h85] = 16'hFFB8;
        ac_length[8'h86] = 16; ac_code[8'h86] = 16'hFFB9;
        ac_length[8'h87] = 16; ac_code[8'h87] = 16'hFFBA;
        ac_length[8'h88] = 16; ac_code[8'h88] = 16'hFFBB;
        ac_length[8'h89] = 16; ac_code[8'h89] = 16'hFFBC;
        ac_length[8'h8A] = 16; ac_code[8'h8A] = 16'hFFBD;
        ac_length[8'h91] = 9; ac_code[8'h91] = 16'h1F9;
        ac_length[8'h92] = 16; ac_code[8'h92] = 16'hFFBE;
        ac_length[8'h93] = 16; ac_code[8'h93] = 16'hFFBF;
        ac_length[8'h94] = 16; ac_code[8'h94] = 16'hFFC0;
        ac_length[8'h95] = 16; ac_code[8'h95] = 16'hFFC1;
        ac_length[8'h96] = 16; ac_code[8'h96] = 16'hFFC2;
        ac_length[8'h97] = 16; ac_code[8'h97] = 16'hFFC3;
        ac_length[8'h98] = 16; ac_code[8'h98] = 16'hFFC4;
        ac_length[8'h99] = 16; ac_code[8'h99] = 16'hFFC5;
        ac_length[8'h9A] = 16; ac_code[8'h9A] = 16'hFFC6;
        ac_length[8'hA1] = 9; ac_code[8'hA1] = 16'h1FA;
        ac_length[8'hA2] = 16; ac_code[8'hA2] = 16'hFFC7;
        ac_length[8'hA3] = 16; ac_code[8'hA3] = 16'hFFC8;
        ac_length[8'hA4] = 16; ac_code[8'hA4] = 16'hFFC9;
        ac_length[8'hA5] = 16; ac_code[8'hA5] = 16'hFFCA;
        ac_length[8'hA6] = 16; ac_code[8'hA6] = 16'hFFCB;
        ac_length[8'hA7] = 16; ac_code[8'hA7] = 16'hFFCC;
        ac_length[8'hA8] = 16; ac_code[8'hA8] = 16'hFFCD;
        ac_length[8'hA9] = 16; ac_code[8'hA9] = 16'hFFCE;
        ac_length[8'hAA] = 16; ac_code[8'hAA] = 16'hFFCF;
        ac_length[8'hB1] = 10; ac_code[8'hB1] = 16'h3F9;
        ac_length[8'hB2] = 16; ac_code[8'hB2] = 16'hFFD0;
        ac_length[8'hB3] = 16; ac_code[8'hB3] = 16'hFFD1;
        ac_length[8'hB4] = 16; ac_code[8'hB4] = 16'hFFD2;
        ac_length[8'hB5] = 16; ac_code[8'hB5] = 16'hFFD3;
        ac_length[8'hB6] = 16; ac_code[8'hB6] = 16'hFFD4;
        ac_length[8'hB7] = 16; ac_code[8'hB7] = 16'hFFD5;
        ac_length[8'hB8] = 16; ac_code[8'hB8] = 16'hFFD6;
        ac_length[8'hB9] = 16; ac_code[8'hB9] = 16'hFFD7;
        ac_length[8'hBA] = 16; ac_code[8'hBA] = 16'hFFD8;
        ac_length[8'hC1] = 10; ac_code[8'hC1] = 16'h3FA;
        ac_length[8'hC2] = 16; ac_code[8'hC2] = 16'hFFD9;
        ac_length[8'hC3] = 16; ac_code[8'hC3] = 16'hFFDA;
        ac_length[8'hC4] = 16; ac_code[8'hC4] = 16'hFFDB;
        ac_length[8'hC5] = 16; ac_code[8'hC5] = 16'hFFDC;
        ac_length[8'hC6] = 16; ac_code[8'hC6] = 16'hFFDD;
        ac_length[8'hC7] = 16; ac_code[8'hC7] = 16'hFFDE;
        ac_length[8'hC8] = 16; ac_code[8'hC8] = 16'hFFDF;
        ac_length[8'hC9] = 16; ac_code[8'hC9] = 16'hFFE0;
        ac_length[8'hCA] = 16; ac_code[8'hCA] = 16'hFFE1;
        ac_length[8'hD1] = 11; ac_code[8'hD1] = 16'h7F8;
        ac_length[8'hD2] = 16; ac_code[8'hD2] = 16'hFFE2;
        ac_length[8'hD3] = 16; ac_code[8'hD3] = 16'hFFE3;
        ac_length[8'hD4] = 16; ac_code[8'hD4] = 16'hFFE4;
        ac_length[8'hD5] = 16; ac_code[8'hD5] = 16'hFFE5;
        ac_length[8'hD6] = 16; ac_code[8'hD6] = 16'hFFE6;
        ac_length[8'hD7] = 16; ac_code[8'hD7] = 16'hFFE7;
        ac_length[8'hD8] = 16; ac_code[8'hD8] = 16'hFFE8;
        ac_length[8'hD9] = 16; ac_code[8'hD9] = 16'hFFE9;
        ac_length[8'hDA] = 16; ac_code[8'hDA] = 16'hFFEA;
        ac_length[8'hE1] = 16; ac_code[8'hE1] = 16'hFFEB;
        ac_length[8'hE2] = 16; ac_code[8'hE2] = 16'hFFEC;
        ac_length[8'hE3] = 16; ac_code[8'hE3] = 16'hFFED;
        ac_length[8'hE4] = 16; ac_code[8'hE4] = 16'hFFEE;
        ac_length[8'hE5] = 16; ac_code[8'hE5] = 16'hFFEF;
        ac_length[8'hE6] = 16; ac_code[8'hE6] = 16'hFFF0;
        ac_length[8'hE7] = 16; ac_code[8'hE7] = 16'hFFF1;
        ac_length[8'hE8] = 16; ac_code[8'hE8] = 16'hFFF2;
        ac_length[8'hE9] = 16; ac_code[8'hE9] = 16'hFFF3;
        ac_length[8'hEA] = 16; ac_code[8'hEA] = 16'hFFF4;
        ac_length[8'hF0] = 11; ac_code[8'hF0] = 16'h7F9;
        ac_length[8'hF1] = 16; ac_code[8'hF1] = 16'hFFF5;
        ac_length[8'hF2] = 16; ac_code[8'hF2] = 16'hFFF6;
        ac_length[8'hF3] = 16; ac_code[8'hF3] = 16'hFFF7;
        ac_length[8'hF4] = 16; ac_code[8'hF4] = 16'hFFF8;
        ac_length[8'hF5] = 16; ac_code[8'hF5] = 16'hFFF9;
        ac_length[8'hF6] = 16; ac_code[8'hF6] = 16'hFFFA;
        ac_length[8'hF7] = 16; ac_code[8'hF7] = 16'hFFFB;
        ac_length[8'hF8] = 16; ac_code[8'hF8] = 16'hFFFC;
        ac_length[8'hF9] = 16; ac_code[8'hF9] = 16'hFFFD;
        ac_length[8'hFA] = 16; ac_code[8'hFA] = 16'hFFFE;
        example_row[0] = 64'h616265686C6F7173;
        example_row[1] = 64'h75726E68625D5856;
        example_row[2] = 64'h5C5C5D5E5F606162;
        example_row[3] = 64'h4D4F53585D626567;
        example_row[4] = 64'h6765625D58534F4D;
        example_row[5] = 64'h6261605F5E5D5C5C;
        example_row[6] = 64'h56585D62686E7275;
        example_row[7] = 64'h73716F6C68656261;
        example_row[8] = 64'h46F4D32A17E6DEBF;
        example_row[9] = 64'h7C6168C147EA4A25;
        example_row[10] = 64'hDFA9C1FD979992D8;
        example_row[11] = 64'h5B13F37572650AAE;
        example_row[12] = 64'h529B19126AB15584;
        example_row[13] = 64'h96BB8FF33707707E;
        example_row[14] = 64'hB466272871218ADE;
        example_row[15] = 64'h2E383ACACCB56A4D;
        // Block 0: S(1,0)=3, D=-4, correct Q=-1, biased Q=0.
        tie_block[0] = 0; tie_address[0] = 1; tie_dct[0] = -4;
        negative_tie_row[0] = 64'h837f808080808181;
        negative_tie_row[1] = 64'h7f7f808080808181;
        negative_tie_row[2] = 64'h7f7f808080808181;
        negative_tie_row[3] = 64'h7f7f808080808181;
        negative_tie_row[4] = 64'h7f7f808080808181;
        negative_tie_row[5] = 64'h7f7f808080808181;
        negative_tie_row[6] = 64'h7f7f808080808181;
        negative_tie_row[7] = 64'h7f7f808080808181;
        // Block 1: S(2,0)=3, D=-12, correct Q=-2, biased Q=-1.
        tie_block[1] = 1; tie_address[1] = 2; tie_dct[1] = -12;
        negative_tie_row[8] = 64'h7f7f818282817f7e;
        negative_tie_row[9] = 64'h7e7f818282817f7e;
        negative_tie_row[10] = 64'h7e7f818282817f7e;
        negative_tie_row[11] = 64'h7e7f818282817f7e;
        negative_tie_row[12] = 64'h7e7f818282817f7e;
        negative_tie_row[13] = 64'h7e7f818282817f7e;
        negative_tie_row[14] = 64'h7e7f818282817f7e;
        negative_tie_row[15] = 64'h7e7f818282817f7e;
        // Block 2: S(1,0)=3, D=-20, correct Q=-3, biased Q=-2.
        tie_block[2] = 2; tie_address[2] = 1; tie_dct[2] = -20;
        negative_tie_row[16] = 64'h7a7d7e7f81828383;
        negative_tie_row[17] = 64'h7d7d7e7f81828383;
        negative_tie_row[18] = 64'h7d7d7e7f81828383;
        negative_tie_row[19] = 64'h7d7d7e7f81828383;
        negative_tie_row[20] = 64'h7d7d7e7f81828383;
        negative_tie_row[21] = 64'h7d7d7e7f81828383;
        negative_tie_row[22] = 64'h7d7d7e7f81828383;
        negative_tie_row[23] = 64'h7d7d7e7f81828383;
        // Block 15: S(0,0)=4, D=-8, correct Q=-1, biased Q=0.
        tie_block[3] = 15; tie_address[3] = 0; tie_dct[3] = -8;
        negative_tie_row[24] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[25] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[26] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[27] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[28] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[29] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[30] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[31] = 64'h7f7f7f7f7f7f7f7f;
        // Block 16: S(3,0)=4, D=-24, correct Q=-2, biased Q=-1.
        tie_block[4] = 16; tie_address[4] = 3; tie_dct[4] = -24;
        negative_tie_row[32] = 64'h7c8184827e7c7f84;
        negative_tie_row[33] = 64'h7c8184827e7c7f84;
        negative_tie_row[34] = 64'h7c8184827e7c7f84;
        negative_tie_row[35] = 64'h7c8184827e7c7f84;
        negative_tie_row[36] = 64'h7c8184827e7c7f84;
        negative_tie_row[37] = 64'h7c8184827e7c7f84;
        negative_tie_row[38] = 64'h7c8184827e7c7f84;
        negative_tie_row[39] = 64'h7c8184827e7c7f84;
        // Block 17: S(0,1)=4, D=-40, correct Q=-3, biased Q=-2.
        tie_block[5] = 17; tie_address[5] = 8; tie_dct[5] = -40;
        negative_tie_row[40] = 64'h7979797979797979;
        negative_tie_row[41] = 64'h7a7a7a7a7a7a7a7a;
        negative_tie_row[42] = 64'h7c7c7c7c7c7c7c7c;
        negative_tie_row[43] = 64'h7f7f7f7f7f7f7f7f;
        negative_tie_row[44] = 64'h8181818181818181;
        negative_tie_row[45] = 64'h8484848484848484;
        negative_tie_row[46] = 64'h8686868686868686;
        negative_tie_row[47] = 64'h8787878787878787;
        // Block 127: S(4,0)=5, D=-16, correct Q=-1, biased Q=0.
        tie_block[6] = 127; tie_address[6] = 4; tie_dct[6] = -16;
        negative_tie_row[48] = 64'h7e82827e7e82827e;
        negative_tie_row[49] = 64'h7e82827e7e82827e;
        negative_tie_row[50] = 64'h7e82827e7e82827e;
        negative_tie_row[51] = 64'h7e82827e7e82827e;
        negative_tie_row[52] = 64'h7e82827e7e82827e;
        negative_tie_row[53] = 64'h7e82827e7e82827e;
        negative_tie_row[54] = 64'h7e82827e7e82827e;
        negative_tie_row[55] = 64'h7e82827e7e82827e;
        // Block 128: S(5,0)=5, D=-48, correct Q=-2, biased Q=-1.
        tie_block[7] = 128; tie_address[7] = 5; tie_dct[7] = -48;
        negative_tie_row[56] = 64'h7b887e7987827885;
        negative_tie_row[57] = 64'h7b887e7987827885;
        negative_tie_row[58] = 64'h7b887e7987827885;
        negative_tie_row[59] = 64'h7b887e7987827885;
        negative_tie_row[60] = 64'h7b887e7987827885;
        negative_tie_row[61] = 64'h7b887e7987827885;
        negative_tie_row[62] = 64'h7b887e7987827885;
        negative_tie_row[63] = 64'h7b887e7987827885;
        // Block 129: S(4,2)=5, D=-80, correct Q=-3, biased Q=-2.
        tie_block[8] = 129; tie_address[8] = 20; tie_dct[8] = -80;
        negative_tie_row[64] = 64'h6f8d8d73738d8d73;
        negative_tie_row[65] = 64'h7b85857b7b85857b;
        negative_tie_row[66] = 64'h857b7b85857b7b85;
        negative_tie_row[67] = 64'h8d73738d8d73738d;
        negative_tie_row[68] = 64'h8d73738d8d73738d;
        negative_tie_row[69] = 64'h857b7b85857b7b85;
        negative_tie_row[70] = 64'h7b85857b7b85857b;
        negative_tie_row[71] = 64'h738d8d73738d8d73;
        // Block 239: S(6,0)=6, D=-32, correct Q=-1, biased Q=0.
        tie_block[9] = 239; tie_address[9] = 6; tie_dct[9] = -32;
        negative_tie_row[72] = 64'h7e8c7b82827b857e;
        negative_tie_row[73] = 64'h7e857b82827b857e;
        negative_tie_row[74] = 64'h7e857b82827b857e;
        negative_tie_row[75] = 64'h7e857b82827b857e;
        negative_tie_row[76] = 64'h7e857b82827b857e;
        negative_tie_row[77] = 64'h7e857b82827b857e;
        negative_tie_row[78] = 64'h7e857b82827b857e;
        negative_tie_row[79] = 64'h7e857b82827b857e;
        // Block 240: S(7,0)=6, D=-96, correct Q=-2, biased Q=-1.
        tie_block[10] = 240; tie_address[10] = 7; tie_dct[10] = -96;
        negative_tie_row[80] = 64'h7d8972916f8e7783;
        negative_tie_row[81] = 64'h7d8972916f8e7783;
        negative_tie_row[82] = 64'h7d8972916f8e7783;
        negative_tie_row[83] = 64'h7d8972916f8e7783;
        negative_tie_row[84] = 64'h7d8972916f8e7783;
        negative_tie_row[85] = 64'h7d8972916f8e7783;
        negative_tie_row[86] = 64'h7d8972916f8e7783;
        negative_tie_row[87] = 64'h7d8972916f8e7783;
        // Block 241: S(6,3)=6, D=-160, correct Q=-3, biased Q=-2.
        tie_block[11] = 241; tie_address[11] = 30; tie_dct[11] = -160;
        negative_tie_row[88] = 64'h739f618d8d619f73;
        negative_tie_row[89] = 64'h8379877d7d877983;
        negative_tie_row[90] = 64'h8f5ca47171a45c8f;
        negative_tie_row[91] = 64'h886c947878946c88;
        negative_tie_row[92] = 64'h78946c88886c9478;
        negative_tie_row[93] = 64'h71a45c8f8f5ca471;
        negative_tie_row[94] = 64'h7d8779838379877d;
        negative_tie_row[95] = 64'h8d619f73739f618d;
        // Block 253: S(5,4)=7, D=-64, correct Q=-1, biased Q=0.
        tie_block[12] = 253; tie_address[12] = 37; tie_dct[12] = -64;
        negative_tie_row[96] = 64'h7a937e7789827586;
        negative_tie_row[97] = 64'h86758289777e8b7a;
        negative_tie_row[98] = 64'h86758289777e8b7a;
        negative_tie_row[99] = 64'h7a8b7e7789827586;
        negative_tie_row[100] = 64'h7a8b7e7789827586;
        negative_tie_row[101] = 64'h86758289777e8b7a;
        negative_tie_row[102] = 64'h86758289777e8b7a;
        negative_tie_row[103] = 64'h7a8b7e7789827586;
        // Block 254: S(4,6)=7, D=-192, correct Q=-2, biased Q=-1.
        tie_block[13] = 254; tie_address[13] = 52; tie_dct[13] = -192;
        negative_tie_row[104] = 64'h738d8d73738d8d73;
        negative_tie_row[105] = 64'ha661619f9f61619f;
        negative_tie_row[106] = 64'h619f9f61619f9f61;
        negative_tie_row[107] = 64'h8d73738d8d73738d;
        negative_tie_row[108] = 64'h8d73738d8d73738d;
        negative_tie_row[109] = 64'h619f9f61619f9f61;
        negative_tie_row[110] = 64'h9f61619f9f61619f;
        negative_tie_row[111] = 64'h738d8d73738d8d73;
        // Block 255: S(7,7)=7, D=-320, correct Q=-3, biased Q=-2.
        tie_block[14] = 255; tie_address[14] = 63; tie_dct[14] = -320;
        negative_tie_row[112] = 64'h7d89738f718d7783;
        negative_tie_row[113] = 64'h8967a554ac5b9977;
        negative_tie_row[114] = 64'h73a549c13fb75b8d;
        negative_tie_row[115] = 64'h8f54c133cd3fac71;
        negative_tie_row[116] = 64'h71ac3fcd33c1548f;
        negative_tie_row[117] = 64'h8d5bb73fc149a573;
        negative_tie_row[118] = 64'h77995bac54a56789;
        negative_tie_row[119] = 64'h83778d718f73897d;
    end
endtask

task negative_tie_self_check;
    integer idx, x, y, block_id, bx, by, address, shift, proper, biased;
    begin
        coverage_on = 0;
        for (idx = 0; idx < 16384; idx = idx + 1) image_data[idx] = 128;
        for (idx = 0; idx < 15; idx = idx + 1) begin
            block_id = tie_block[idx]; bx = block_id & 15; by = block_id >> 4;
            if (negative_tie_case(block_id) != idx) begin
                stop_test("NEGATIVE_HALF_SELF_CHECK: fixture block map differs");
            end
            for (y = 0; y < 8; y = y + 1) begin
                for (x = 0; x < 8; x = x + 1) begin
                    image_data[(by*8+y)*128+bx*8+x] = negative_tie_row[idx*8+y][(7-x)*8 +: 8];
                end
            end
            transform_block(block_id);
            address = tie_address[idx]; shift = quant_shift[address];
            proper = round_signed(tie_dct[idx], shift);
            biased = (tie_dct[idx] + (1 << (shift-1))) >>> shift;
            if ((dct_coeff[address] != tie_dct[idx]) ||
                (quant_coeff[address] != proper) || (proper == biased) ||
                (((-dct_coeff[address]) & ((1 << shift)-1)) != (1 << (shift-1)))) begin
                stop_test("NEGATIVE_HALF_SELF_CHECK: pixel fixture does not hit the exact negative half");
            end
        end
        reset_model;
        clear_coverage;
        $display("NEGATIVE_HALF_SELF_CHECK: 15 exact negative halves matched, three per shift 3..7");
    end
endtask

task golden_self_check;
    integer idx, x, y, expected;
    reg [8*33-1:0] expected_bytes;
    reg [64*16-1:0] expected_dct, expected_quant;
    begin
        coverage_on = 0;
        if ((round_signed(25,3) != 3) || (round_signed(-25,3) != -3) ||
            (round_signed(28,3) != 4) || (round_signed(-28,3) != -4) ||
            (round_signed(-1024,4) != -64) ||
            (round_signed(round_signed(917504,18),3) != 1) ||
            (round_signed(917504,21) != 0)) begin
            stop_test("GOLDEN_SELF_CHECK: symmetric/double rounding differs from the handout");
        end
        for (y = 0; y < 8; y = y + 1) begin
            for (x = 0; x < 8; x = x + 1) begin
                image_data[y*128+x] = 88;
                image_data[y*128+8+x] = example_row[y][(7-x)*8 +: 8];
                image_data[y*128+16+x] = example_row[8+y][(7-x)*8 +: 8];
            end
        end
        reset_model;
        model_block = 0;
        transform_block(0);
        for (idx = 0; idx < 64; idx = idx + 1) begin
            if (idx == 0) expected = -20; else expected = 0;
            if (zigzag_coeff[idx] != expected) stop_test("GOLDEN_SELF_CHECK: trace block 0 coefficients differ");
        end
        encode_block;
        if ((golden_count != 1) || (golden_byte[0] != 8'hcb) ||
            (partial_bits != 4) || (partial_byte != 8'ha)) begin
            stop_test("GOLDEN_SELF_CHECK: trace block 0 stream differs");
        end
        model_block = 1;
        transform_block(1);
        for (idx = 0; idx < 64; idx = idx + 1) begin
            case (idx)
                0: expected = -15;
                3: expected = 3;
                22: expected = -2;
                default: expected = 0;
            endcase
            if (zigzag_coeff[idx] != expected) stop_test("GOLDEN_SELF_CHECK: trace block 1 coefficients differ");
        end
        encode_block;
        if ((encoded_bits != 53) || (golden_count != 6) ||
            (partial_bits != 5) || (partial_byte != 8'h1a)) begin
            stop_test("GOLDEN_SELF_CHECK: trace block 1 cross-block packing differs");
        end
        model_block = 2;
        transform_block(2);
        expected_dct = 1024'hfffaffef0012ffd30004fff80030ffae00710018000cffe9ff62000d0063ffce0023ffbfffee003effa90016ffccffe1ffce00130098ffdfff83ffdbffb1fff0ffc0ff9affce0017ff90ffa00041001f0030fff1008bff830017fff2002a0075009a0034ffe3ff9dfff80030ffbfffe5003bffbb00810059ffe9ffb2ffc80005;
        expected_quant = 1024'h0000fffe0002fffd000000000001ffff000700020001fffffffb00000002ffff0002fffcffff0002fffd0000ffff0000fffd0001000afffffffeffffffff0000fffcfffafffe0000fffeffff00010000000200000002fffe0000000000000001000200010000fffe00000000ffff00000001ffff000100010000ffff00000000;
        for (idx = 0; idx < 64; idx = idx + 1) begin
            if ((dct_coeff[idx] != $signed(expected_dct[(63-idx)*16 +: 16])) ||
                (quant_coeff[idx] != $signed(expected_quant[(63-idx)*16 +: 16]))) begin
                stop_test("GOLDEN_SELF_CHECK: trace block 2 D/Q matrix differs");
            end
        end
        encode_block;
        expected_bytes = 264'hCBA97E7FF3F2D5F59D9990C691903F74D750B36D0990DB771236A80303693CFA03;
        if ((encoded_bits != 267) || (golden_count != 33) ||
            (partial_bits != 3) || (partial_byte != 8'h2)) begin
            stop_test("GOLDEN_SELF_CHECK: trace block 2 bit count differs");
        end
        for (idx = 0; idx < 33; idx = idx + 1) begin
            if (golden_byte[idx] != expected_bytes[(32-idx)*8 +: 8]) begin
                stop_test("GOLDEN_SELF_CHECK: official three-block trace bytes differ");
            end
        end
        finish_model;
        if ((golden_count != 34) || (golden_byte[33] != 8'h5f)) begin
            stop_test("GOLDEN_SELF_CHECK: official final one-bit padding differs");
        end
        reset_model;
        clear_coverage;
        $display("GOLDEN_SELF_CHECK: official D/Q matrices, three-block bytes and padding matched");
    end
endtask



//======================================
//           DISPLAY TASK
//======================================

task display_pass_summary;
    integer idx, show_coverage;
    begin
        // Short summary by default; +COVERAGE=1 also prints the coverage counters.
        // "execution cycles =" and "clock period =" are the lines the TA's 08_check reads.
        show_coverage = 0;
        if ($value$plusargs("COVERAGE=%d", show_coverage)) begin end
        $display("");
        $display("============================================================");
        display_center($sformatf("Congratulations! All %0d Lab05 images passed.", pattern_num));
        display_center($sformatf("execution cycles = %0d", total_latency));
        display_center($sformatf("clock period = %0.1f ns", CYCLE));
        display_center($sformatf("Image mix: HIGH=%0d LOW=%0d", high_class_count, low_class_count));
        $display("============================================================");
        if (show_coverage != 0) begin
            $display("Total encoded bytes = %0d", total_bytes);
            $display("Gap distribution N3=%0d N4=%0d N5=%0d N6=%0d",
                     gap_hist[3], gap_hist[4], gap_hist[5], gap_hist[6]);
            $display("Coverage: EOB=%0d ZRL=%0d last_AC_nonzero=%0d positive=%0d negative=%0d",
                     count_eob, count_zrl, count_last_ac, count_positive, count_negative);
            $display("Coverage: double_round_diff=%0d raw_FF=%0d unaligned_block_ends=%0d",
                     count_double_round, count_raw_ff, count_unaligned_blocks);
            $display("Negative quantization half coverage: shift3=%0d shift4=%0d shift5=%0d shift6=%0d shift7=%0d",
                     negative_half_hist[3], negative_half_hist[4], negative_half_hist[5],
                     negative_half_hist[6], negative_half_hist[7]);
            $write("Final residual bits before padding (0..7):");
            for (idx = 0; idx < 8; idx = idx + 1) $write(" %0d:%0d", idx, pad_hist[idx]);
            $display("");
            $write("DC SIZE coverage (0..11):");
            for (idx = 0; idx < 12; idx = idx + 1) $write(" %0d:%0d", idx, dc_hist[idx]);
            $display("");
            $write("AC SIZE coverage (1..10):");
            for (idx = 1; idx < 11; idx = idx + 1) $write(" %0d:%0d", idx, ac_hist[idx]);
            $display("");
            $write("AC residual RUN coverage (0..15):");
            for (idx = 0; idx < 16; idx = idx + 1) $write(" %0d:%0d", idx, run_hist[idx]);
            $display("");
            $write("AC SYMBOL bins (hex symbol:count):");
            for (idx = 0; idx < 256; idx = idx + 1) if (ac_symbol_hist[idx] != 0) $write(" %02h:%0d", idx[7:0], ac_symbol_hist[idx]);
            $display("");
            $write("DC signed SIZE bins (positive0..8,negative0..8):");
            for (idx = 0; idx < 18; idx = idx + 1) $write(" %0d:%0d", idx, dc_signed_hist[idx]);
            $display("");
            $write("AC signed SIZE bins (positive0..7,negative0..7):");
            for (idx = 0; idx < 16; idx = idx + 1) $write(" %0d:%0d", idx, ac_signed_hist[idx]);
            $display("");
            $write("Block start bit offsets 0..7:");
            for (idx = 0; idx < 8; idx = idx + 1) $write(" %0d:%0d", idx, block_offset_hist[idx]);
            $display("");
            $write("Trailing AC zeros 0..63:");
            for (idx = 0; idx < 64; idx = idx + 1) $write(" %0d:%0d", idx, trailing_hist[idx]);
            $display("");
            $write("ZRL burst per nonzero 0..3:");
            for (idx = 0; idx < 4; idx = idx + 1) $write(" %0d:%0d", idx, zrl_burst_hist[idx]);
            $display("");
            $write("First-round boundary bins (+ below/tie/above, - below/tie/above):");
            for (idx = 0; idx < 6; idx = idx + 1) $write(" %0d:%0d", idx, first_boundary_hist[idx]);
            $display("");
            $write("Quant boundary bins (shift3..7, each +below/tie/above,-below/tie/above):");
            for (idx = 0; idx < 30; idx = idx + 1) $write(" %0d:%0d", idx, quant_boundary_hist[idx]);
            $display("");
            $write("Coefficient nonzero signs (v*8+u, positive then negative):");
            for (idx = 0; idx < 128; idx = idx + 1) $write(" %0d:%0d", idx, coefficient_sign_hist[idx]);
            $display("");
            $display("============================================================");
        end
        $display("");
    end
endtask

// Print one line centred in the 60-character width of the summary frame.
task display_center;
    input string text;
    string line;
    integer pad;
    begin
        line = "";
        for (pad = (60 - text.len()) / 2; pad > 0; pad = pad - 1) line = {line, " "};
        $display("%s%s", line, text);
    end
endtask



endmodule
