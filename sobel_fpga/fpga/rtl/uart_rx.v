`timescale 1ns / 1ps
`default_nettype none

// UART receiver: 8 data bits, no parity, 1 stop bit (8N1), LSB first.
//
// The line idles high. A byte looks like this, each bit lasting CLKS_PER_BIT
// clocks:
//
//   idle start  d0   d1   d2   d3   d4   d5   d6   d7  stop  idle
//   ____       ____ ____ ____ ____ ____ ____ ____ ____ __________
//       \_____/____X____X____X____X____X____X____X____/
//
// The receiver waits for the falling edge of the start bit, counts half a bit
// to land in the middle of it, then samples once per bit. Sampling mid-bit
// leaves the most margin for the two clocks (ours and the laptop's USB chip)
// being slightly different.
module uart_rx #(
    parameter integer CLKS_PER_BIT = 32   // clock frequency / baud rate, 8 or more
) (
    input  wire       clk,
    input  wire       rst,
    input  wire       rx,      // serial input, not synchronous to clk
    output reg  [7:0] data  = 8'd0,   // the received byte
    output reg        valid = 1'b0,   // high for one clock when data is new
    output reg        error = 1'b0    // high for one clock when the stop bit was missing
);
    localparam integer  CW       = $clog2(CLKS_PER_BIT);
    localparam [CW-1:0] HALF_BIT = CLKS_PER_BIT / 2 - 1;
    localparam [CW-1:0] FULL_BIT = CLKS_PER_BIT - 1;

    // rx comes from another clock domain, so it passes through two flip-flops
    // before any logic looks at it. A third remembers the previous value so a
    // falling edge can be spotted.
    (* ASYNC_REG = "TRUE" *) reg [1:0] rx_sync = 2'b11;
    reg rx_prev = 1'b1;
    wire rx_s = rx_sync[1];
    always @(posedge clk) begin
        rx_sync <= {rx_sync[0], rx};
        rx_prev <= rx_s;
    end
    wire start_edge = rx_prev && !rx_s;

    reg          busy    = 1'b0;
    reg [CW-1:0] clk_cnt = {CW{1'b0}};  // clocks left until the next sample
    reg [3:0]    bit_idx = 4'd0;        // 0 start bit, 1 to 8 data bits, 9 stop bit
    reg [7:0]    shift   = 8'd0;

    always @(posedge clk) begin
        valid <= 1'b0;
        error <= 1'b0;

        if (rst) begin
            busy <= 1'b0;
        end else if (!busy) begin
            if (start_edge) begin
                busy    <= 1'b1;
                clk_cnt <= HALF_BIT;
                bit_idx <= 4'd0;
            end
        end else if (clk_cnt != 0) begin
            clk_cnt <= clk_cnt - 1'b1;
        end else begin
            // Middle of a bit: sample it.
            clk_cnt <= FULL_BIT;
            bit_idx <= bit_idx + 4'd1;
            if (bit_idx == 4'd0) begin
                if (rx_s) busy <= 1'b0;          // line went back high: a glitch, not a start bit
            end else if (bit_idx <= 4'd8) begin
                shift <= {rx_s, shift[7:1]};     // LSB arrives first
            end else begin
                busy <= 1'b0;                    // done; ready for the next start edge right away
                if (rx_s) begin
                    data  <= shift;
                    valid <= 1'b1;
                end else begin
                    error <= 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
