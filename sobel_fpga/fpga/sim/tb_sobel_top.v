`timescale 1ns / 1fs
`default_nettype none

// Testbench for the whole design, pin for pin. It plays the laptop: it sends
// bytes into uart_rxd one bit at a time, decodes whatever comes back on
// uart_txd, and writes the received bytes to a file. run_sim.py prepares the
// bytes to send and checks the reply against sobel_webcam.py.
//
// The script file has one 4-digit hex word per line:
//   00dd   send byte dd
//   1nnn   keep the line idle for nnn * 64 bit times
//   2000   press and release the reset button
//   ffff   end of script
//
// Plus-arguments:
//   +script=FILE   words to play (required)
//   +out=FILE      where to write the bytes the FPGA sends back (required)
//   +bit_ns=83.4   the laptop's bit time in ns; default is exactly 1e9 / BAUD.
//                  Setting it a little off models two crystals that disagree.
//   +gap=N         extra idle bit times after every byte sent (default 0)
//   +vcd=FILE      dump waveforms for GTKWave
module tb_sobel_top;
    parameter integer BAUD         = 12_000_000;  // what the FPGA is built for
    parameter integer MAX_WIDTH    = 2048;
    parameter integer TIMEOUT_MS   = 1;           // short, so timeout tests run fast
    parameter integer SCRIPT_WORDS = 200_000;

    // ---------------------------------------------------------------------
    // The board.
    // ---------------------------------------------------------------------
    reg  clk12     = 1'b0;
    reg  btn_reset = 1'b0;
    reg  uart_rxd  = 1'b1;
    wire uart_txd;
    wire [3:0] led;
    wire rgb_r, rgb_g, rgb_b;

    always #41.6665 clk12 = ~clk12;               // 12 MHz

    sobel_top #(
        .BAUD       (BAUD),
        .MAX_WIDTH  (MAX_WIDTH),
        .TIMEOUT_MS (TIMEOUT_MS)
    ) dut (
        .clk12     (clk12),
        .btn_reset (btn_reset),
        .uart_rxd  (uart_rxd),
        .uart_txd  (uart_txd),
        .led       (led),
        .rgb_r     (rgb_r),
        .rgb_g     (rgb_g),
        .rgb_b     (rgb_b)
    );

    // ---------------------------------------------------------------------
    // Settings from the command line.
    // ---------------------------------------------------------------------
    reg [1023:0] script_file, out_file, vcd_file;
    real    bit_ns;
    integer gap;
    integer out_fd;

    reg [15:0] script [0:SCRIPT_WORDS-1];

    // ---------------------------------------------------------------------
    // Laptop to FPGA.
    // ---------------------------------------------------------------------
    task send_byte(input [7:0] value);
        integer k;
        begin
            uart_rxd = 1'b0;                      // start bit
            #(bit_ns);
            for (k = 0; k < 8; k = k + 1) begin   // data, LSB first
                uart_rxd = value[k];
                #(bit_ns);
            end
            uart_rxd = 1'b1;                      // stop bit, then any extra gap
            #(bit_ns * (1 + gap));
        end
    endtask

    // ---------------------------------------------------------------------
    // FPGA to laptop.
    // ---------------------------------------------------------------------
    integer bytes_back      = 0;
    integer bad_stop_bits   = 0;
    real    last_start_ns   = -1.0;
    real    shortest_gap_ns = 1.0e12;             // start bit to next start bit
    reg [7:0] received;
    integer   n;

    always begin
        @(negedge uart_txd);                      // start bit
        if (last_start_ns >= 0.0 && $realtime - last_start_ns < shortest_gap_ns)
            shortest_gap_ns = $realtime - last_start_ns;
        last_start_ns = $realtime;
        #(bit_ns * 1.5);                          // middle of data bit 0
        for (n = 0; n < 8; n = n + 1) begin
            received[n] = uart_txd;
            #(bit_ns);
        end
        if (uart_txd !== 1'b1)                    // now in the middle of the stop bit
            bad_stop_bits = bad_stop_bits + 1;
        $fdisplay(out_fd, "%02x", received);
        bytes_back = bytes_back + 1;
    end

    // ---------------------------------------------------------------------
    // Things worth knowing about the inside of the design.
    // ---------------------------------------------------------------------
    integer frames_done  = 0;
    integer frame_errors = 0;
    integer rx_errors    = 0;
    integer overflows    = 0;
    integer fifo_peak    = 0;
    reg [11:0] fifo_level;                        // same width as the FIFO's pointers

    always @(posedge dut.clk) begin
        if (dut.frame_done)  frames_done  = frames_done  + 1;
        if (dut.frame_error) frame_errors = frame_errors + 1;
        if (dut.rx_error)    rx_errors    = rx_errors    + 1;
        if (dut.overflow)    overflows    = overflows    + 1;
        fifo_level = dut.u_fifo.wr_ptr - dut.u_fifo.rd_ptr;
        if (fifo_level > fifo_peak) fifo_peak = fifo_level;
    end

    // ---------------------------------------------------------------------
    // Play the script.
    // ---------------------------------------------------------------------
    integer pc;
    integer seen;
    reg [15:0] word;

    initial begin
        if (!$value$plusargs("script=%s", script_file) ||
            !$value$plusargs("out=%s", out_file)) begin
            $display("usage: vvp tb.vvp +script=FILE +out=FILE [+bit_ns=NS] [+gap=N] [+vcd=FILE]");
            $finish;
        end
        if (!$value$plusargs("bit_ns=%f", bit_ns)) bit_ns = 1.0e9 / BAUD;
        if (!$value$plusargs("gap=%d", gap))       gap = 0;
        if ($value$plusargs("vcd=%s", vcd_file)) begin
            $dumpfile(vcd_file);
            $dumpvars(0, tb_sobel_top);
        end

        $readmemh(script_file, script);
        out_fd = $fopen(out_file, "w");

        wait (dut.rst == 1'b0);                   // clock locked, reset released
        #(bit_ns * 20);

        pc = 0;
        while (script[pc] !== 16'hffff) begin
            word = script[pc];
            case (word[15:12])
                4'h0: send_byte(word[7:0]);
                4'h1: #(bit_ns * 64 * word[11:0]);
                4'h2: begin
                    btn_reset = 1'b1;
                    #(bit_ns * 40);
                    btn_reset = 1'b0;
                    #(bit_ns * 40);
                    wait (dut.rst == 1'b0);
                end
                default: begin
                    $display("bad script word %h at line %0d", word, pc + 1);
                    $finish;
                end
            endcase
            pc = pc + 1;
        end

        // Wait for the FPGA to finish talking: no new byte for a while.
        seen = -1;
        while (seen != bytes_back) begin
            seen = bytes_back;
            #(bit_ns * 400);
        end

        $fclose(out_fd);
        $display("RESULT bytes_back=%0d frames_done=%0d frame_errors=%0d rx_errors=%0d overflows=%0d fifo_peak=%0d bad_stop_bits=%0d shortest_gap_bits=%0.4f",
                 bytes_back, frames_done, frame_errors, rx_errors, overflows,
                 fifo_peak, bad_stop_bits, shortest_gap_ns / bit_ns);
        $finish;
    end
endmodule

`default_nettype wire
