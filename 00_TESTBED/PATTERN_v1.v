`timescale 1ns/10ps
// File-driven black-box checker for ICLAB 2026F Lab05.
// input.txt : N, then N raster images of exactly 16384 unsigned byte values.
// golden.txt: N, then for each image a byte count followed by those bytes.
// Decimal ASCII tokens only; whitespace separates tokens. No comments or hex.
// No JPEG reference model is embedded here: every expected byte comes from file.
// Use the course TESTBED interface. Do not synthesize this testbench.
// Outputs are sampled exactly at negedge. Gate settling after posedge is allowed.
// Changes during the low half-cycle, idle valid pulses and settled-reset pulses
// are rejected. Initial reset has a two-period propagation grace interval.
`ifndef CYCLE_TIME
    `define CYCLE_TIME 9
`endif

module PATTERN(clk, rst_n, in_valid, in_data, out_valid, out_data);
output reg clk;
output reg rst_n;
output reg in_valid;
output reg [7:0] in_data;
input out_valid;
input [7:0] out_data;

// Declare every module object before tasks, functions, and procedural blocks.
reg [7:0] pixels [0:16383];
reg [7:0] golden [0:65535];
integer golden_count;
real cycle_time;
reg config_done, clock_running, monitor_on;
integer seed, pattern_count, start_pattern;
integer frame, ordinal, pixel, phase, latency, received, last_latency;
integer first_latency, time_latency;
integer gap, max_latency, config_used, patterns_given;
reg [63:0] total_bytes, total_latency;
real input_fall_time, total_latency_ns;
localparam integer LATENCY_LIMIT = 50000;
localparam integer FINAL_GUARD_CYCLES = 50000;
integer input_fd, golden_fd, input_frames, golden_frames;
integer file_frame, file_item, parsed_value;
reg [8*1024-1:0] input_filename, golden_filename;
reg [63:0] input_tokens, golden_tokens, input_line, golden_line;

function integer ascii_space;
    input integer character;
    begin
        ascii_space = (character == 32 ||
                       (character >= 9 && character <= 13));
    end
endfunction

task file_format_stop;
    input integer which_file;
    input [8*64-1:0] field_name;
    input [8*96-1:0] reason;
    input integer character;
    begin
        if (which_file == 0)
            $fatal(1, "FILE_FORMAT_FAIL file=%0s field=%0s reason=%0s frame=%0d item=%0d token=%0d line=%0d char=%0d",
                   input_filename, field_name, reason, file_frame, file_item,
                   input_tokens, input_line, character);
        else
            $fatal(1, "FILE_FORMAT_FAIL file=%0s field=%0s reason=%0s frame=%0d item=%0d token=%0d line=%0d char=%0d",
                   golden_filename, field_name, reason, file_frame, file_item,
                   golden_tokens, golden_line, character);
    end
endtask

task read_character;
    input integer descriptor;
    input integer which_file;
    output integer character;
    begin
        character = $fgetc(descriptor);
        if (character == 10) begin
            if (which_file == 0) input_line = input_line + 1;
            else golden_line = golden_line + 1;
        end
    end
endtask

// Unlike $fscanf("%d"), this lexer rejects suffixes such as 12xyz, signs,
// hex prefixes, X/Z digits, commas, and integer overflow. At most 10 digits
// are accepted per token; leading zeroes are otherwise allowed.
task read_decimal;
    input integer descriptor;
    input integer which_file;
    input integer maximum;
    input [8*64-1:0] field_name;
    output integer value;
    integer character, digit, digits;
    begin
        if (which_file == 0) input_tokens = input_tokens + 1;
        else golden_tokens = golden_tokens + 1;
        read_character(descriptor, which_file, character);
        while (ascii_space(character))
            read_character(descriptor, which_file, character);
        if (character == -1)
            file_format_stop(which_file, field_name, "unexpected_eof", character);
        value = 0;
        digits = 0;
        while (character != -1 && !ascii_space(character)) begin
            if (character < 48 || character > 57)
                file_format_stop(which_file, field_name, "nondecimal_character", character);
            digits = digits + 1;
            if (digits > 10)
                file_format_stop(which_file, field_name, "token_too_long", character);
            digit = character - 48;
            // Check before multiply/add, so no intermediate integer can wrap.
            if (value > maximum / 10 ||
                (value == maximum / 10 && digit > maximum % 10))
                file_format_stop(which_file, field_name, "value_out_of_range", character);
            value = value * 10 + digit;
            read_character(descriptor, which_file, character);
        end
    end
endtask

task require_eof;
    input integer descriptor;
    input integer which_file;
    integer character;
    begin
        read_character(descriptor, which_file, character);
        while (ascii_space(character))
            read_character(descriptor, which_file, character);
        if (character != -1)
            file_format_stop(which_file, "end_of_file", "trailing_token_or_garbage", character);
    end
endtask

task load_frame;
    begin
        for (file_item = 0; file_item < 16384; file_item = file_item + 1) begin
            read_decimal(input_fd, 0, 255, "pixel", parsed_value);
            pixels[file_item] = parsed_value[7:0];
        end
        file_item = -1;
        read_decimal(golden_fd, 1, 65536, "golden_length", golden_count);
        if (golden_count == 0)
            file_format_stop(1, "golden_length", "zero_golden_length", -1);
        for (file_item = 0; file_item < golden_count; file_item = file_item + 1) begin
            read_decimal(golden_fd, 1, 255, "golden_byte", parsed_value);
            golden[file_item] = parsed_value[7:0];
        end
    end
endtask

task sample_outputs;
    begin
        if (out_valid !== 1'b0 && out_valid !== 1'b1)
            $fatal(1, "PATTERN_FAIL unknown_out_valid frame=%0d phase=%0d time=%0t", frame, phase, $time);
        if (out_valid === 1'b0 && out_data !== 8'h00)
            $fatal(1, "PATTERN_FAIL invalid_data_nonzero_or_X frame=%0d phase=%0d got=%h time=%0t", frame, phase, out_data, $time);
        if (rst_n === 1'b0 && (out_valid !== 1'b0 || out_data !== 8'h00))
            $fatal(1, "PATTERN_FAIL reset_outputs time=%0t", $time);
        if (phase == 2 && received < golden_count) begin
            // Independent time-based check of the inclusive E1=1 counter.
            time_latency = $rtoi(($realtime - input_fall_time) / cycle_time + 0.5) + 1;
            if (latency != time_latency)
                $fatal(1, "PATTERN_FAIL latency_counter_mismatch frame=%0d count=%0d timestamp_count=%0d", frame, latency, time_latency);
            if (latency > LATENCY_LIMIT)
                $fatal(1, "PATTERN_FAIL timeout_or_missing_byte frame=%0d received=%0d expected=%0d latency=%0d", frame, received, golden_count, latency);
        end
        if (out_valid === 1'b1) begin
            if (phase != 2)
                $fatal(1, "PATTERN_FAIL unexpected_output frame=%0d phase=%0d got=%h time=%0t", frame, phase, out_data, $time);
            if (received >= golden_count)
                $fatal(1, "PATTERN_FAIL extra_byte frame=%0d byte=%0d got=%h time=%0t", frame, received, out_data, $time);
            if (out_data !== golden[received]) begin
                $display("PATTERN_CONTEXT input=%0s golden=%0s seed=%0d", input_filename, golden_filename, seed);
                $fatal(1, "PATTERN_FAIL byte_mismatch frame=%0d byte=%0d got=%h expected=%h latency=%0d", frame, received, out_data, golden[received], latency);
            end
            if (received == 0) first_latency = latency;
            received = received + 1;
            if (received == golden_count) last_latency = latency;
        end
    end
endtask

task tick;
    begin
        @(negedge clk);
        if (phase == 2 && received < golden_count) latency = latency + 1;
        sample_outputs;
    end
endtask

// One driver owns all clock transitions. Reset is tested with clock stopped.
initial begin
    clk = 0;
    wait (config_done === 1'b1);
    forever begin
        #(cycle_time / 2.0);
        if (clock_running) clk = ~clk;
        else clk = 1'b0;
    end
end

// The specification forbids overlap at any time, not only sampling edges.
always @(in_valid or out_valid or rst_n) begin
    if (monitor_on === 1'b1 && rst_n === 1'b1 &&
        in_valid === 1'b1 && out_valid === 1'b1)
        $fatal(1, "PATTERN_FAIL overlap frame=%0d time=%0t", frame, $time);
end

// A posedge-driven output may settle during the high half-cycle in GATE.
// Never test data==0 on every event in that interval: valid and data can have
// different propagation delays. The ordinary negedge sample checks both.
// Once clk is low, any output change is late or occurs on the wrong edge.
always @(out_valid or out_data) begin
    if (monitor_on === 1'b1) begin
        if (rst_n === 1'b0) begin
            if (out_valid !== 1'b0 || out_data !== 8'h00)
                $fatal(1, "PATTERN_FAIL settled_reset_output_change time=%0t valid=%b data=%h", $time, out_valid, out_data);
        end
        else if (rst_n === 1'b1) begin
            if (clock_running === 1'b0 &&
                (out_valid !== 1'b0 || out_data !== 8'h00))
                $fatal(1, "PATTERN_FAIL stopped_clock_output time=%0t valid=%b data=%h", $time, out_valid, out_data);
            if (phase != 2 && out_valid !== 1'b0)
                $fatal(1, "PATTERN_FAIL idle_or_input_valid_pulse frame=%0d phase=%0d time=%0t valid=%b", frame, phase, $time, out_valid);
            if (clock_running === 1'b1 && clk === 1'b0)
                $fatal(1, "PATTERN_FAIL output_change_in_low_phase frame=%0d phase=%0d time=%0t valid=%b data=%h", frame, phase, $time, out_valid, out_data);
        end
    end
end

// Also inspect the boundary at which the continuous monitor is armed and
// reset is released; an already-bad constant value must not need a new event.
always @(monitor_on or rst_n) begin
    if (monitor_on === 1'b1 &&
        (rst_n === 1'b0 || clock_running === 1'b0) &&
        (out_valid !== 1'b0 || out_data !== 8'h00))
        $fatal(1, "PATTERN_FAIL reset_or_release_boundary time=%0t valid=%b data=%h", $time, out_valid, out_data);
end

initial begin
    config_done = 0; clock_running = 0; monitor_on = 0;
    rst_n = 1; in_valid = 0; in_data = 8'hxx;
    phase = 0; frame = -1; latency = 0; received = 0; last_latency = 0;
    first_latency = 0; time_latency = 0; input_fall_time = 0.0; total_latency_ns = 0.0;
    total_bytes = 0; total_latency = 0; max_latency = 0;
    cycle_time = `CYCLE_TIME; seed = 20261014;
    start_pattern = 0; pattern_count = 0;
    input_filename = "input.txt"; golden_filename = "golden.txt";
    input_tokens = 0; golden_tokens = 0; input_line = 1; golden_line = 1;
    file_frame = -1; file_item = -1;
    config_used = $value$plusargs("CYCLE=%f", cycle_time);
    config_used = $value$plusargs("SEED=%d", seed);
    config_used = $value$plusargs("START=%d", start_pattern);
    patterns_given = $value$plusargs("PATTERNS=%d", pattern_count);
    config_used = $value$plusargs("INPUT=%s", input_filename);
    config_used = $value$plusargs("GOLDEN=%s", golden_filename);
    if (!(cycle_time > 0.0 && cycle_time <= 20.0))
        $fatal(1, "PATTERN_FAIL CYCLE_must_be_positive_and_at_most_20ns");
    if ((cycle_time*10.0 - $rtoi(cycle_time*10.0+0.5) > 0.000001) ||
        (cycle_time*10.0 - $rtoi(cycle_time*10.0+0.5) < -0.000001))
        $fatal(1, "PATTERN_FAIL CYCLE_precision_must_be_0p1ns");
    input_fd = $fopen(input_filename, "r");
    if (input_fd == 0)
        $fatal(1, "FILE_FORMAT_FAIL cannot_open_input file=%0s", input_filename);
    golden_fd = $fopen(golden_filename, "r");
    if (golden_fd == 0)
        $fatal(1, "FILE_FORMAT_FAIL cannot_open_golden file=%0s", golden_filename);
    read_decimal(input_fd, 0, 1000000, "frame_count", input_frames);
    read_decimal(golden_fd, 1, 1000000, "frame_count", golden_frames);
    if (input_frames == 0)
        file_format_stop(0, "frame_count", "zero_frame_count", -1);
    if (golden_frames == 0)
        file_format_stop(1, "frame_count", "zero_frame_count", -1);
    if (input_frames != golden_frames)
        file_format_stop(1, "frame_count", "frame_count_mismatch", -1);
    if (!patterns_given) pattern_count = input_frames - start_pattern;
    if (start_pattern < 0 || start_pattern >= input_frames ||
        pattern_count < 1 || pattern_count > input_frames - start_pattern)
        $fatal(1, "PATTERN_FAIL invalid_pattern_range file_frames=%0d start=%0d count=%0d", input_frames, start_pattern, pattern_count);
    config_done = 1;
    $display("PATTERN_CONFIG file_frames=%0d frames=%0d start=%0d seed=%0d cycle_ns=%0.2f input=%0s golden=%0s", input_frames, pattern_count, start_pattern, seed, cycle_time, input_filename, golden_filename);

    // No clock edge occurs during this asynchronous reset check.
    #(cycle_time / 3.0) rst_n = 0;
    #(cycle_time * 2.0);
    if (out_valid !== 1'b0 || out_data !== 8'h00)
        $fatal(1, "PATTERN_FAIL asynchronous_reset_without_clock got_valid=%b got_data=%h", out_valid, out_data);
    monitor_on = 1;
    #(cycle_time / 4.0) rst_n = 1;
    #(cycle_time) clock_running = 1;
    repeat (3) tick;

    // Read every frame even when only a selected interval is simulated.
    // Parsing consumes host CPU only; it introduces no simulation-time gaps.
    for (file_frame = 0; file_frame < input_frames; file_frame = file_frame + 1) begin
        load_frame;
        if (file_frame >= start_pattern && file_frame < start_pattern + pattern_count) begin
            frame = file_frame;
            ordinal = frame - start_pattern;
            received = 0; latency = 0; last_latency = 0; first_latency = 0;
            phase = 1;
            in_valid = 1;
            for (pixel = 0; pixel < 16384; pixel = pixel + 1) begin
                in_data = pixels[pixel];
                // Each pixel remains stable over exactly one active clock edge.
                @(negedge clk);
                if (pixel != 16383) sample_outputs;
            end
            // This negative edge is E1, counted as latency cycle 1 by the handout.
            in_valid = 0; in_data = 8'hxx;
            input_fall_time = $realtime;
            phase = 2; latency = 1;
            sample_outputs;
            while (received < golden_count) tick;
            // After the exact final byte, every additional valid byte is rejected.
            tick;
            phase = 3;
            total_bytes = total_bytes + golden_count;
            total_latency = total_latency + last_latency;
            if (last_latency > max_latency) max_latency = last_latency;
            $display("PATTERN_PASS frame=%0d bytes=%0d latency=%0d", frame, golden_count, last_latency);
            $display("PATTERN_TIMING frame=%0d first_byte_cycle=%0d last_byte_cycle=%0d output_cycles=%0d non_output_cycles=%0d execution_ns=%0.2f",
                     frame, first_latency, last_latency, received, last_latency - received, last_latency * cycle_time);
            // Deterministically cover all four legal image gaps without reset.
            // SEED rotates the order; it does not change the input/golden files.
            gap = 3 + ((ordinal + (seed & 3)) % 4);
            repeat (gap - 1) tick;
        end
    end
    file_item = -1;
    require_eof(input_fd, 0);
    require_eof(golden_fd, 1);
    $fclose(input_fd);
    $fclose(golden_fd);
    $display("FILE_VALIDATION_PASS frames=%0d input_tokens=%0d golden_tokens=%0d", input_frames, input_tokens, golden_tokens);
    // Observe longer after the final transaction to detect delayed extra bytes.
    repeat (FINAL_GUARD_CYCLES) tick;
    $display("ALL_PATTERNS_PASS frames=%0d bytes=%0d max_latency=%0d total_latency=%0d", pattern_count, total_bytes, max_latency, total_latency);
    total_latency_ns = total_latency * cycle_time;
    $display("LATENCY_SUMMARY total_latency_cycles=%0d cycle_ns=%0.2f total_latency_ns=%0.2f tail_guard_cycles=%0d",
             total_latency, cycle_time, total_latency_ns, FINAL_GUARD_CYCLES);
    $finish;
end

// Independent bounded watchdog also catches a stuck testbench/clock.
initial begin
    wait (config_done === 1'b1);
    #(cycle_time * (70000.0 * pattern_count + FINAL_GUARD_CYCLES + 1000.0));
    $fatal(1, "PATTERN_FAIL global_watchdog frame=%0d phase=%0d", frame, phase);
end

endmodule
