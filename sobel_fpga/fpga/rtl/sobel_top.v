`timescale 1ns / 1ps
`default_nettype none

// Top level for the Digilent Cmod S7 (Spartan-7 XC7S25).
//
//   laptop --USB--> FT2232 chip --uart_rxd--> uart_rx --> frame_rx --> sobel_core
//                                                                          |
//   laptop <--USB-- FT2232 chip <--uart_txd-- uart_tx <-- sync_fifo <------+
//
// The laptop sends one frame at a time over the board's USB serial port
// (protocol in frame_rx.v) and gets one result byte back per pixel.
//
// LEDs
//   led[0]  heartbeat: blinks while the design is loaded and running
//   led[1]  bytes are arriving from the laptop
//   led[2]  whole frames are being completed
//   led[3]  something went wrong in the last second or so: garbled byte
//           (usually a baud rate mismatch), bad header, timeout, or overflow
// BTN0 resets the design.
module sobel_top #(
    parameter integer BAUD       = 3_000_000,   // must match --baud on the laptop
    parameter integer MAX_WIDTH  = 2048,        // widest frame accepted, in pixels
    parameter integer TIMEOUT_MS = 200          // give up on a stalled frame after this long
) (
    input  wire       clk12,       // 12 MHz oscillator
    input  wire       btn_reset,   // BTN0, high while pressed
    input  wire       uart_rxd,    // from the laptop
    output wire       uart_txd,    // to the laptop
    output wire [3:0] led,
    output wire       rgb_r,       // RGB LED, low = on. Kept off.
    output wire       rgb_g,
    output wire       rgb_b
);
    // The FT2232 makes its baud rates as 12 MHz / (1, 1.5, 2, 2.125, 2.25, ...).
    // A 96 MHz clock is 8 x 12 MHz, so every one of those rates is a whole
    // number of our clocks per bit: 8 at 12 Mbaud, 32 at 3 Mbaud.
    localparam integer CLK_HZ       = 96_000_000;
    localparam integer CLKS_PER_BIT = (CLK_HZ + BAUD / 2) / BAUD;
    localparam integer TIMEOUT_CLKS = (CLK_HZ / 1000) * TIMEOUT_MS;

    // ---------------------------------------------------------------------
    // Clock: 12 MHz in, 96 MHz out.
    // ---------------------------------------------------------------------
    // The MMCM multiplies 12 MHz by 64 to run its internal oscillator at
    // 768 MHz (it must stay between 600 and 1200 MHz), then divides by 8.
    wire clk_fb, clk_mmcm, clk, locked;

    MMCME2_BASE #(
        .BANDWIDTH        ("OPTIMIZED"),
        .CLKIN1_PERIOD    (83.333),
        .DIVCLK_DIVIDE    (1),
        .CLKFBOUT_MULT_F  (64.0),
        .CLKOUT0_DIVIDE_F (8.0)
    ) u_mmcm (
        .CLKIN1   (clk12),
        .CLKFBIN  (clk_fb),
        .CLKFBOUT (clk_fb),
        .CLKOUT0  (clk_mmcm),
        .LOCKED   (locked),
        .PWRDWN   (1'b0),
        .RST      (1'b0)
    );

    BUFG u_bufg (.I(clk_mmcm), .O(clk));

    // ---------------------------------------------------------------------
    // Reset: held until the clock is stable, and while BTN0 is pressed.
    // ---------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) reg [1:0] btn_sync = 2'b00;
    always @(posedge clk)
        btn_sync <= {btn_sync[0], btn_reset};

    reg [7:0] rst_pipe = 8'hFF;
    always @(posedge clk or negedge locked) begin
        if (!locked) rst_pipe <= 8'hFF;
        else         rst_pipe <= {rst_pipe[6:0], btn_sync[1]};
    end
    wire rst = rst_pipe[7];

    // ---------------------------------------------------------------------
    // The pipeline.
    // ---------------------------------------------------------------------
    wire [7:0]  rx_data;
    wire        rx_valid, rx_error;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_rx (
        .clk   (clk),
        .rst   (rst),
        .rx    (uart_rxd),
        .data  (rx_data),
        .valid (rx_valid),
        .error (rx_error)
    );

    wire        pix_valid;
    wire [7:0]  pix_gray;
    wire [15:0] pix_col, pix_row;
    wire [2:0]  view;
    wire [7:0]  threshold;
    wire        frame_done, frame_error;

    frame_rx #(
        .MAX_WIDTH    (MAX_WIDTH),
        .TIMEOUT_CLKS (TIMEOUT_CLKS)
    ) u_frame_rx (
        .clk         (clk),
        .rst         (rst),
        .rx_data     (rx_data),
        .rx_valid    (rx_valid),
        .pix_valid   (pix_valid),
        .pix_gray    (pix_gray),
        .pix_col     (pix_col),
        .pix_row     (pix_row),
        .view        (view),
        .threshold   (threshold),
        .frame_done  (frame_done),
        .frame_error (frame_error)
    );

    wire       result_valid;
    wire [7:0] result;

    sobel_core #(.MAX_WIDTH(MAX_WIDTH)) u_sobel (
        .clk       (clk),
        .in_valid  (pix_valid),
        .in_pixel  (pix_gray),
        .in_col    (pix_col),
        .in_row    (pix_row),
        .view      (view),
        .threshold (threshold),
        .out_valid (result_valid),
        .out_pixel (result)
    );

    wire       fifo_full;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;

    sync_fifo #(.WIDTH(8), .DEPTH(2048)) u_fifo (
        .clk       (clk),
        .rst       (rst),
        .in_data   (result),
        .in_valid  (result_valid),
        .full      (fifo_full),
        .out_data  (tx_data),
        .out_valid (tx_valid),
        .out_ready (tx_ready)
    );

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_uart_tx (
        .clk   (clk),
        .rst   (rst),
        .data  (tx_data),
        .valid (tx_valid),
        .ready (tx_ready),
        .tx    (uart_txd)
    );

    // ---------------------------------------------------------------------
    // LEDs. Events last one clock (about 10 ns), so each one restarts a countdown
    // and the LED stays lit until the countdown runs out.
    // ---------------------------------------------------------------------
    wire overflow = result_valid && fifo_full;   // a result had nowhere to go

    reg [26:0] heartbeat = 27'd0;                // bit 26 flips every 0.7 s
    reg [22:0] rx_glow    = 23'd0;               // 2^23 clocks = 87 ms
    reg [22:0] frame_glow = 23'd0;
    reg [26:0] error_glow = 27'd0;               // 2^27 clocks = 1.4 s

    always @(posedge clk) begin
        heartbeat <= heartbeat + 1'b1;

        if (rx_valid || rx_error)   rx_glow <= {23{1'b1}};
        else if (rx_glow != 0)      rx_glow <= rx_glow - 1'b1;

        if (frame_done)             frame_glow <= {23{1'b1}};
        else if (frame_glow != 0)   frame_glow <= frame_glow - 1'b1;

        if (rst)                                      error_glow <= 27'd0;
        else if (rx_error || frame_error || overflow) error_glow <= {27{1'b1}};
        else if (error_glow != 0)                     error_glow <= error_glow - 1'b1;
    end

    assign led[0] = heartbeat[26] && !rst;
    assign led[1] = (rx_glow != 0);
    assign led[2] = (frame_glow != 0);
    assign led[3] = (error_glow != 0);

    assign rgb_r = 1'b1;
    assign rgb_g = 1'b1;
    assign rgb_b = 1'b1;
endmodule

`default_nettype wire
