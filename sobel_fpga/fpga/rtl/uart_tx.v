`timescale 1ns / 1ps
`default_nettype none

// UART transmitter: 8 data bits, no parity, 1 stop bit (8N1), LSB first.
//
// Handshake: when valid and ready are both high on a clock edge, the byte on
// data is taken and sent. ready is high while idle and also during the very
// last clock of a stop bit, so a waiting byte starts with no gap at all.
// Back-to-back bytes therefore take exactly 10 * CLKS_PER_BIT clocks each,
// which is, on paper, the same rate the bytes arrive at from the laptop
// (sync_fifo.v deals with "on paper").
module uart_tx #(
    parameter integer CLKS_PER_BIT = 32   // clock frequency / baud rate
) (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] data,
    input  wire       valid,
    output wire       ready,
    output reg        tx = 1'b1   // serial output, idles high
);
    localparam integer  CW       = $clog2(CLKS_PER_BIT);
    localparam [CW-1:0] FULL_BIT = CLKS_PER_BIT - 1;

    reg          busy      = 1'b0;
    reg [CW-1:0] clk_cnt   = {CW{1'b0}};  // clocks left in the current bit
    reg [3:0]    bits_left = 4'd0;        // bits still to send after the current one
    reg [8:0]    shift     = 9'h1FF;      // {stop bit, data byte}

    wire bit_done = (clk_cnt == 0);
    assign ready  = !busy || (bit_done && bits_left == 4'd0);

    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0;
            tx   <= 1'b1;
        end else if (ready) begin
            if (valid) begin
                busy      <= 1'b1;
                tx        <= 1'b0;             // start bit
                shift     <= {1'b1, data};
                bits_left <= 4'd9;             // 8 data bits and the stop bit
                clk_cnt   <= FULL_BIT;
            end else begin
                busy <= 1'b0;
                tx   <= 1'b1;
            end
        end else if (bit_done) begin
            tx        <= shift[0];
            shift     <= {1'b1, shift[8:1]};
            bits_left <= bits_left - 4'd1;
            clk_cnt   <= FULL_BIT;
        end else begin
            clk_cnt <= clk_cnt - 1'b1;
        end
    end
endmodule

`default_nettype wire
