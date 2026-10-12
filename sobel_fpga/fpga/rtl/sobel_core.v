`timescale 1ns / 1ps
`default_nettype none

// Stages 2 to 5 of the pipeline: 3x3 window, gradients, magnitude, threshold.
//
// This is sobel_streaming(), magnitude() and threshold() from sobel_webcam.py
// as hardware. Pixels arrive one at a time in raster order (left to right, top
// to bottom) and there is no access to the rest of the frame, so:
//
//   line buffer   a RAM with one entry per column, holding the two pixels
//                 directly above the one arriving (rows r-1 and r-2)
//   window        nine registers; a new column shifts in on every pixel
//
//       p00 p01 p02        Gx: -1  0 +1        Gy: -1 -2 -1
//       p10 p11 p12            -2  0 +2             0  0  0
//       p20 p21 p22            -1  0 +1            +1 +2 +1
//
// When pixel (row r, column c) arrives it becomes p22, so the window is
// centered on (r-1, c-1). The window is complete once r >= 2 and c >= 2.
//
// One result byte comes out for every pixel that goes in, 7 clocks later:
//
//   view 0  gray        the pixel that just arrived, unchanged
//   view 1  |Gx|        clamped to 255      \
//   view 2  |Gy|        clamped to 255       |  for the window centered on
//   view 3  magnitude   |Gx| + |Gy|, clamped |  (r-1, c-1); 0 until the
//   view 4  edges       255 if magnitude >   |  window is complete
//                       threshold, else 0   /
//   (views 5 to 7 are not used and behave like view 4)
//
// So for views 1 to 4 the output picture is the input picture's result
// shifted down and right by one pixel. The laptop script shifts it back.
//
// The pipeline does one small step per clock. A pixel may arrive on every
// clock (in_valid held high), so the same core can later be fed straight from
// a camera instead of the UART.
module sobel_core #(
    parameter integer MAX_WIDTH = 2048    // widest frame the line buffer can hold
) (
    input  wire        clk,
    // pixel stream in
    input  wire        in_valid,
    input  wire [7:0]  in_pixel,    // gray
    input  wire [15:0] in_col,      // position of this pixel in the frame
    input  wire [15:0] in_row,
    // settings, changed only between frames
    input  wire [2:0]  view,
    input  wire [7:0]  threshold,
    // result stream out
    output reg         out_valid = 1'b0,
    output reg  [7:0]  out_pixel = 8'd0
);
    localparam integer AW = $clog2(MAX_WIDTH);

    function [9:0] abs_value(input signed [10:0] v);       // |v| for v in -1020..1020
        abs_value = v[10] ? (~v[9:0] + 10'd1) : v[9:0];
    endfunction

    function [7:0] clamp_255(input [10:0] v);
        clamp_255 = (v > 11'd255) ? 8'd255 : v[7:0];
    endfunction

    // valid[n] is high while stage n holds a real pixel. window_ok[n] says
    // whether that pixel's 3x3 window is complete.
    reg [6:1] valid     = 6'd0;
    reg [6:1] window_ok = 6'd0;
    always @(posedge clk) begin
        valid     <= {valid[5:1], in_valid};
        window_ok <= {window_ok[5:1], (in_row >= 16'd2) && (in_col >= 16'd2)};
    end

    // ---------------------------------------------------------------------
    // Clock 1: look up the two pixels above the incoming one.
    // ---------------------------------------------------------------------
    reg [15:0] line_ram [0:MAX_WIDTH-1];   // per column: {pixel 2 rows up, pixel 1 row up}
    reg [15:0] above   = 16'd0;
    reg [7:0]  pixel_1 = 8'd0;
    reg [AW-1:0] col_1 = {AW{1'b0}};

    always @(posedge clk) begin
        if (in_valid)
            above <= line_ram[in_col[AW-1:0]];
        pixel_1 <= in_pixel;
        col_1   <= in_col[AW-1:0];
    end

    // ---------------------------------------------------------------------
    // Clock 2: push the pixel down the line buffer and shift the window.
    // ---------------------------------------------------------------------
    // The entry for this column becomes {old 1-row-up, new pixel}: on the
    // next row those are the pixels 2 rows up and 1 row up.
    always @(posedge clk) begin
        if (valid[1])
            line_ram[col_1] <= {above[7:0], pixel_1};
    end

    reg [7:0] p00 = 8'd0, p01 = 8'd0, p02 = 8'd0;
    reg [7:0] p10 = 8'd0, p11 = 8'd0, p12 = 8'd0;
    reg [7:0] p20 = 8'd0, p21 = 8'd0, p22 = 8'd0;

    always @(posedge clk) begin
        if (valid[1]) begin
            p00 <= p01;  p01 <= p02;  p02 <= above[15:8];   // row r-2
            p10 <= p11;  p11 <= p12;  p12 <= above[7:0];    // row r-1
            p20 <= p21;  p21 <= p22;  p22 <= pixel_1;       // row r
        end
    end

    // ---------------------------------------------------------------------
    // Clock 3: the four weighted sums. Each is at most 4 * 255 = 1020.
    // ---------------------------------------------------------------------
    reg [9:0] left = 10'd0, right = 10'd0, top = 10'd0, bottom = 10'd0;

    always @(posedge clk) begin
        left   <= {2'b00, p00} + {1'b0, p10, 1'b0} + {2'b00, p20};   // p00 + 2*p10 + p20
        right  <= {2'b00, p02} + {1'b0, p12, 1'b0} + {2'b00, p22};   // p02 + 2*p12 + p22
        top    <= {2'b00, p00} + {1'b0, p01, 1'b0} + {2'b00, p02};   // p00 + 2*p01 + p02
        bottom <= {2'b00, p20} + {1'b0, p21, 1'b0} + {2'b00, p22};   // p20 + 2*p21 + p22
    end

    // ---------------------------------------------------------------------
    // Clock 4: the gradients, 11-bit signed (-1020..1020).
    // ---------------------------------------------------------------------
    reg signed [10:0] gx = 11'sd0, gy = 11'sd0;

    always @(posedge clk) begin
        gx <= $signed({1'b0, right})  - $signed({1'b0, left});
        gy <= $signed({1'b0, bottom}) - $signed({1'b0, top});
    end

    // ---------------------------------------------------------------------
    // Clock 5: absolute values.
    // ---------------------------------------------------------------------
    reg [9:0] abs_gx = 10'd0, abs_gy = 10'd0;

    always @(posedge clk) begin
        abs_gx <= abs_value(gx);
        abs_gy <= abs_value(gy);
    end

    // ---------------------------------------------------------------------
    // Clock 6: magnitude |Gx| + |Gy|, and everything clamped to 8 bits.
    // ---------------------------------------------------------------------
    reg [7:0] gx_8 = 8'd0, gy_8 = 8'd0, mag_8 = 8'd0;

    always @(posedge clk) begin
        gx_8  <= clamp_255({1'b0, abs_gx});
        gy_8  <= clamp_255({1'b0, abs_gy});
        mag_8 <= clamp_255({1'b0, abs_gx} + {1'b0, abs_gy});
    end

    // The gray view needs the pixel itself, delayed to line up with the rest
    // (it is in p22 after clock 2, and is used at clock 7).
    reg [31:0] gray_delay = 32'd0;
    always @(posedge clk)
        gray_delay <= {gray_delay[23:0], p22};

    // ---------------------------------------------------------------------
    // Clock 7: threshold, and pick what to send back.
    // ---------------------------------------------------------------------
    always @(posedge clk) begin
        out_valid <= valid[6];
        case (view)
            3'd0:    out_pixel <= gray_delay[31:24];
            3'd1:    out_pixel <= window_ok[6] ? gx_8  : 8'd0;
            3'd2:    out_pixel <= window_ok[6] ? gy_8  : 8'd0;
            3'd3:    out_pixel <= window_ok[6] ? mag_8 : 8'd0;
            default: out_pixel <= (window_ok[6] && mag_8 > threshold) ? 8'd255 : 8'd0;
        endcase
    end

    // Start the line buffer at zero so simulations begin from a clean state.
    // The values never matter: the first two rows of a frame are masked by
    // window_ok, and by then the buffer holds this frame's pixels.
    integer i;
    initial begin
        for (i = 0; i < MAX_WIDTH; i = i + 1)
            line_ram[i] = 16'd0;
    end
endmodule

`default_nettype wire
