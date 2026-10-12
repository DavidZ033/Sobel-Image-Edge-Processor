`timescale 1ns / 1ps
`default_nettype none

// Stage 1 of the pipeline: color to gray.
//
// Same integer formula as rgb_to_gray() in sobel_webcam.py:
//
//     gray = (77*R + 150*G + 29*B) >> 8
//
// The weights add up to 256, so the sum fits in 16 bits (255 * 256 = 65280)
// and ">> 8" is just "keep the top byte". It also means a pixel with
// R = G = B = p comes out as exactly p, which frame_rx uses to pass pixels
// that are already gray through this same path.
//
// Two clocks from in_valid to out_valid:
//   clock 1  the three multiplications
//   clock 2  the sum
module rgb_to_gray (
    input  wire       clk,
    input  wire       in_valid,
    input  wire [7:0] r,
    input  wire [7:0] g,
    input  wire [7:0] b,
    output reg        out_valid = 1'b0,
    output reg  [7:0] gray      = 8'd0
);
    reg [15:0] weighted_r = 16'd0;
    reg [15:0] weighted_g = 16'd0;
    reg [15:0] weighted_b = 16'd0;
    reg        valid_1    = 1'b0;

    always @(posedge clk) begin
        weighted_r <= 16'd77  * {8'd0, r};
        weighted_g <= 16'd150 * {8'd0, g};
        weighted_b <= 16'd29  * {8'd0, b};
        valid_1    <= in_valid;
    end

    wire [15:0] luma = weighted_r + weighted_g + weighted_b;

    always @(posedge clk) begin
        gray      <= luma[15:8];
        out_valid <= valid_1;
    end
endmodule

`default_nettype wire
