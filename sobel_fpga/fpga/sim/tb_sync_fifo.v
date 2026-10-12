`timescale 1ns / 1ps
`default_nettype none

// Testbench for sync_fifo.v on its own. Inside the full design the FIFO
// never gets anywhere near full, so this bench uses a tiny one and random
// traffic: bursts that fill it, stretches that drain it, and everything in
// between. A plain array remembers what went in. Everything that comes out
// has to match it, in the same order, with nothing lost or repeated.
//
// Plus-arguments:
//   +cycles=N   how many clocks of traffic (default 200000)
//   +vcd=FILE   dump waveforms for GTKWave
module tb_sync_fifo;
    parameter integer DEPTH = 8;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg        rst       = 1'b1;
    reg  [7:0] in_data   = 8'd0;
    reg        in_valid  = 1'b0;
    reg        out_ready = 1'b0;
    wire       full;
    wire [7:0] out_data;
    wire       out_valid;

    sync_fifo #(.WIDTH(8), .DEPTH(DEPTH)) dut (
        .clk       (clk),
        .rst       (rst),
        .in_data   (in_data),
        .in_valid  (in_valid),
        .full      (full),
        .out_data  (out_data),
        .out_valid (out_valid),
        .out_ready (out_ready)
    );

    // The reference: what has gone in and not come out yet.
    reg [7:0] expected [0:1048575];
    integer head = 0, tail = 0;                 // next to come out, next free slot
    integer pushes = 0, pops = 0, refused = 0, peak = 0, errors = 0;

    always @(posedge clk) begin
        if (!rst) begin
            if (in_valid && !full) begin        // accepted
                expected[tail % 1048576] = in_data;
                tail   = tail + 1;
                pushes = pushes + 1;
            end
            if (in_valid && full)
                refused = refused + 1;          // ignored, as documented
            if (out_valid && out_ready) begin   // taken
                if (head == tail || out_data !== expected[head % 1048576])
                    errors = errors + 1;        // came out of nowhere, or wrong byte
                else
                    head = head + 1;
                pops = pops + 1;
            end
            if (tail - head > peak)  peak = tail - head;
            if (tail - head > DEPTH) errors = errors + 1;   // holds more than it has room for
        end
    end

    reg [1023:0] vcd_file;
    integer cycles, seed, i, push_percent, pop_percent;

    initial begin
        if (!$value$plusargs("cycles=%d", cycles)) cycles = 200000;
        if ($value$plusargs("vcd=%s", vcd_file)) begin
            $dumpfile(vcd_file);
            $dumpvars(0, tb_sync_fifo);
        end
        seed = 1;
        push_percent = 50;
        pop_percent  = 50;

        repeat (3) @(negedge clk);
        rst = 1'b0;
        for (i = 0; i < cycles; i = i + 1) begin
            @(negedge clk);
            if (i % 1000 == 0) begin            // new traffic pattern every 1000 clocks
                case ({$random(seed)} % 6)
                    0: begin push_percent = 100; pop_percent = 100; end  // both every clock
                    1: begin push_percent = 100; pop_percent = 10;  end  // fills up
                    2: begin push_percent = 10;  pop_percent = 100; end  // runs empty
                    3: begin push_percent = 50;  pop_percent = 50;  end
                    4: begin push_percent = 90;  pop_percent = 80;  end
                    5: begin push_percent = 3;   pop_percent = 3;   end  // sparse, like the UART
                endcase
            end
            in_valid  = ({$random(seed)} % 100) < push_percent;
            in_data   = $random(seed);
            out_ready = ({$random(seed)} % 100) < pop_percent;
        end

        // Stop pushing and let it drain: nothing may be left behind.
        in_valid  = 1'b0;
        out_ready = 1'b1;
        repeat (2 * DEPTH + 8) @(negedge clk);
        if (head != tail || out_valid)
            errors = errors + 1;

        $display("RESULT pushes=%0d pops=%0d refused=%0d peak=%0d errors=%0d",
                 pushes, pops, refused, peak, errors);
        $finish;
    end
endmodule

`default_nettype wire
