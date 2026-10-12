`timescale 1ns / 1ps
`default_nettype none

// First-in first-out buffer, one clock, built on a block RAM.
//
// It sits between the Sobel core and the UART transmitter. Results come out
// of the core at the rate bytes arrive from the laptop, and the transmitter
// sends them at the rate set by this board's clock. Those two rates are equal
// on paper but differ by up to a hundred parts per million or so in practice
// (two different crystals), so over a frame a small backlog can build up.
// This absorbs it: at 100 parts per million, a 640x480 frame leaves about 30
// bytes waiting here. The backlog drains before the next frame, because the
// laptop waits for the whole reply before it sends more.
//
// The oldest entry is shown on out_data whenever out_valid is high ("first
// word fall through"). It is removed on a clock edge where out_ready is also
// high.
module sync_fifo #(
    parameter integer WIDTH = 8,
    parameter integer DEPTH = 2048        // must be a power of two
) (
    input  wire             clk,
    input  wire             rst,
    // write side
    input  wire [WIDTH-1:0] in_data,
    input  wire             in_valid,     // store in_data (ignored while full)
    output wire             full,
    // read side
    output reg  [WIDTH-1:0] out_data  = {WIDTH{1'b0}},
    output reg              out_valid = 1'b0,
    input  wire             out_ready
);
    localparam integer AW = $clog2(DEPTH);

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // Pointers carry one extra bit so "full" and "empty" can be told apart:
    // equal pointers mean empty, pointers exactly DEPTH apart mean full.
    reg [AW:0] wr_ptr = {(AW+1){1'b0}};
    reg [AW:0] rd_ptr = {(AW+1){1'b0}};

    wire push = in_valid && !full;
    wire pop  = out_valid && out_ready;

    wire [AW:0] rd_ptr_next = rd_ptr + {{AW{1'b0}}, pop};

    assign full = (wr_ptr[AW] != rd_ptr[AW]) && (wr_ptr[AW-1:0] == rd_ptr[AW-1:0]);

    // The RAM. Reading takes one clock, so the read address is always the
    // entry that will be at the front on the next clock.
    always @(posedge clk) begin
        if (push)
            mem[wr_ptr[AW-1:0]] <= in_data;
        out_data <= mem[rd_ptr_next[AW-1:0]];
    end

    always @(posedge clk) begin
        if (rst) begin
            wr_ptr    <= {(AW+1){1'b0}};
            rd_ptr    <= {(AW+1){1'b0}};
            out_valid <= 1'b0;
        end else begin
            if (push)
                wr_ptr <= wr_ptr + 1'b1;
            rd_ptr <= rd_ptr_next;
            // The entry being read this clock is real only if it was written on
            // an earlier clock, which is exactly what wr_ptr (not yet counting
            // this clock's push) says.
            out_valid <= (wr_ptr != rd_ptr_next);
        end
    end
endmodule

`default_nettype wire
