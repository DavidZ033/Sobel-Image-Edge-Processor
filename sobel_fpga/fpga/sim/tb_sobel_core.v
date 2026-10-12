`timescale 1ns / 1ps
`default_nettype none

// Testbench for sobel_core.v on its own. It feeds whole frames straight into
// the core, by default one pixel on every single clock with no pause between
// frames, which is the hardest case and what a camera would do. run_sim.py
// supplies the pixels and checks the result against sobel_webcam.py.
//
// Plus-arguments:
//   +pixels=FILE   one 2-digit hex pixel per line, frame after frame (required)
//   +out=FILE      where to write the result bytes (required)
//   +width=N +height=N +frames=N
//   +view=N +threshold=N
//   +stall=N       0: a pixel on every clock. N: random pauses, on average
//                  one pixel every N clocks.
//   +vcd=FILE      dump waveforms for GTKWave
module tb_sobel_core;
    parameter integer MAX_WIDTH  = 2048;
    parameter integer MAX_PIXELS = 1_000_000;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg         in_valid  = 1'b0;
    reg  [7:0]  in_pixel  = 8'd0;
    reg  [15:0] in_col    = 16'd0;
    reg  [15:0] in_row    = 16'd0;
    reg  [2:0]  view      = 3'd0;
    reg  [7:0]  threshold = 8'd0;
    wire        out_valid;
    wire [7:0]  out_pixel;

    sobel_core #(.MAX_WIDTH(MAX_WIDTH)) dut (
        .clk       (clk),
        .in_valid  (in_valid),
        .in_pixel  (in_pixel),
        .in_col    (in_col),
        .in_row    (in_row),
        .view      (view),
        .threshold (threshold),
        .out_valid (out_valid),
        .out_pixel (out_pixel)
    );

    reg [7:0] pixels [0:MAX_PIXELS-1];
    reg [1023:0] pixel_file, out_file, vcd_file;
    integer width, height, frames, stall, seed, out_fd, i, value;

    // Results, and the delay from a pixel going in to its result coming out.
    integer sent_at [0:15];                 // clock numbers of the last 16 pixels in
    integer clock_no = 0, n_in = 0, n_out = 0, latency = -1;
    always @(posedge clk) begin
        clock_no = clock_no + 1;
        if (in_valid) begin
            sent_at[n_in % 16] = clock_no;
            n_in = n_in + 1;
        end
        if (out_valid) begin
            $fdisplay(out_fd, "%02x", out_pixel);
            if (latency < 0) latency = clock_no - sent_at[n_out % 16];
            if (clock_no - sent_at[n_out % 16] != latency) latency = 999;   // must never vary
            n_out = n_out + 1;
        end
    end

    initial begin
        if (!$value$plusargs("pixels=%s", pixel_file) ||
            !$value$plusargs("out=%s", out_file)) begin
            $display("usage: vvp tb.vvp +pixels=FILE +out=FILE +width=N +height=N ...");
            $finish;
        end
        if (!$value$plusargs("width=%d", width))    width  = 64;
        if (!$value$plusargs("height=%d", height))  height = 48;
        if (!$value$plusargs("frames=%d", frames))  frames = 1;
        if (!$value$plusargs("stall=%d", stall))    stall  = 0;
        if ($value$plusargs("view=%d", value))      view      = value[2:0];
        if ($value$plusargs("threshold=%d", value)) threshold = value[7:0];
        if ($value$plusargs("vcd=%s", vcd_file)) begin
            $dumpfile(vcd_file);
            $dumpvars(0, tb_sobel_core);
        end
        seed = 1;
        $readmemh(pixel_file, pixels);
        out_fd = $fopen(out_file, "w");

        repeat (4) @(posedge clk);
        for (i = 0; i < width * height * frames; i = i + 1) begin
            while (stall > 1 && ({$random(seed)} % stall) != 0) begin
                in_valid <= 1'b0;
                @(posedge clk);
            end
            in_valid <= 1'b1;
            in_pixel <= pixels[i];
            in_col   <= i % width;
            in_row   <= (i / width) % height;
            @(posedge clk);
        end
        in_valid <= 1'b0;
        repeat (20) @(posedge clk);

        $fclose(out_fd);
        $display("RESULT bytes_back=%0d latency=%0d", n_out, latency);
        $finish;
    end
endmodule

`default_nettype wire
