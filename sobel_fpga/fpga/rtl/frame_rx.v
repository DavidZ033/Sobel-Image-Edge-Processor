`timescale 1ns / 1ps
`default_nettype none

// Turns the byte stream from the laptop into a stream of gray pixels.
//
// Every frame the laptop sends looks like this:
//
//   byte  0      0xA5       \  start marker
//   byte  1      0x5A       /
//   byte  2      control    bits 2:0 = view (0 gray, 1 |Gx|, 2 |Gy|, 3 magnitude, 4 edges)
//                           bits 6:3 = not used, send 0
//                           bit 7    = pixel format: 0 gray (1 byte per pixel)
//                                                    1 color (3 bytes per pixel: B, G, R)
//   byte  3      threshold  0 to 255, used by view 4
//   bytes 4, 5   width      low byte first, 1 to MAX_WIDTH
//   bytes 6, 7   height     low byte first, 1 or more
//   byte  8      check      bytes 2 to 7 XORed together
//   then         width * height pixels, left to right, top to bottom
//
// While waiting for a start marker, every other byte is ignored.
//
// Two things keep the two sides from getting stuck out of step:
//   * a header with a wrong check byte or an impossible size is thrown away;
//   * if the bytes stop arriving in the middle of a frame for TIMEOUT_CLKS
//     clocks, the frame is abandoned and the hunt for a start marker resumes.
//
// Bytes from the UART are about 80 clocks apart even at the fastest baud
// rate (10 bits of 8 clocks each), so the 2 clocks the gray conversion takes
// are always over long before the next byte shows up.
module frame_rx #(
    parameter integer MAX_WIDTH    = 2048,
    parameter integer TIMEOUT_CLKS = 19_200_000   // 200 ms at 96 MHz
) (
    input  wire        clk,
    input  wire        rst,
    // bytes from uart_rx
    input  wire [7:0]  rx_data,
    input  wire        rx_valid,
    // gray pixels to sobel_core
    output wire        pix_valid,
    output wire [7:0]  pix_gray,
    output reg  [15:0] pix_col     = 16'd0,
    output reg  [15:0] pix_row     = 16'd0,
    // settings for the current frame
    output reg  [2:0]  view        = 3'd0,
    output reg  [7:0]  threshold   = 8'd0,
    // status, each high for one clock
    output reg         frame_done  = 1'b0,   // the last pixel of a frame went out
    output reg         frame_error = 1'b0    // bad header or timeout
);
    localparam [7:0] MAGIC_0 = 8'hA5;
    localparam [7:0] MAGIC_1 = 8'h5A;

    localparam [3:0] S_MAGIC_0   = 4'd0,
                     S_MAGIC_1   = 4'd1,
                     S_CONTROL   = 4'd2,
                     S_THRESHOLD = 4'd3,
                     S_WIDTH_LO  = 4'd4,
                     S_WIDTH_HI  = 4'd5,
                     S_HEIGHT_LO = 4'd6,
                     S_HEIGHT_HI = 4'd7,
                     S_CHECK     = 4'd8,
                     S_PIXELS    = 4'd9;

    reg [3:0]  state    = S_MAGIC_0;
    reg        is_color = 1'b0;
    reg [15:0] width    = 16'd0;
    reg [15:0] height   = 16'd0;
    reg [7:0]  check    = 8'd0;       // running XOR of bytes 2 to 7

    wire size_ok = (width != 16'd0) && (width <= MAX_WIDTH) && (height != 16'd0);

    // ---------------------------------------------------------------------
    // Timeout: clocks since the last byte, counted only inside a frame.
    // ---------------------------------------------------------------------
    localparam integer TW = $clog2(TIMEOUT_CLKS + 1);
    reg [TW-1:0] quiet_clks = {TW{1'b0}};
    wire timed_out = (quiet_clks == TIMEOUT_CLKS);

    always @(posedge clk) begin
        if (rst || rx_valid || state == S_MAGIC_0)
            quiet_clks <= {TW{1'b0}};
        else
            quiet_clks <= quiet_clks + 1'b1;
    end

    // ---------------------------------------------------------------------
    // Pixel assembly: 1 byte per gray pixel, or 3 bytes (B, G, R) per color
    // pixel. A gray byte is handed to rgb_to_gray as R = G = B, which comes
    // out unchanged.
    // ---------------------------------------------------------------------
    reg [1:0] byte_idx = 2'd0;
    reg [7:0] blue     = 8'd0;
    reg [7:0] green    = 8'd0;

    wire pixel_byte    = (state == S_PIXELS) && rx_valid;
    wire pixel_complete = pixel_byte && (!is_color || byte_idx == 2'd2);

    rgb_to_gray u_gray (
        .clk       (clk),
        .in_valid  (pixel_complete),
        .r         (rx_data),
        .g         (is_color ? green : rx_data),
        .b         (is_color ? blue  : rx_data),
        .out_valid (pix_valid),
        .gray      (pix_gray)
    );

    wire last_col   = (pix_col == width  - 16'd1);
    wire last_row   = (pix_row == height - 16'd1);
    wire last_pixel = pix_valid && last_col && last_row;

    // ---------------------------------------------------------------------
    // The state machine.
    // ---------------------------------------------------------------------
    always @(posedge clk) begin
        frame_done  <= 1'b0;
        frame_error <= 1'b0;

        if (rst) begin
            state <= S_MAGIC_0;
        end else if (timed_out) begin
            state       <= S_MAGIC_0;
            frame_error <= 1'b1;
        end else begin
            if (rx_valid) begin
                case (state)
                    S_MAGIC_0:
                        if (rx_data == MAGIC_0) state <= S_MAGIC_1;
                    S_MAGIC_1:
                        if      (rx_data == MAGIC_1) state <= S_CONTROL;
                        else if (rx_data != MAGIC_0) state <= S_MAGIC_0;
                    S_CONTROL: begin
                        view     <= rx_data[2:0];
                        is_color <= rx_data[7];
                        check    <= rx_data;
                        state    <= S_THRESHOLD;
                    end
                    S_THRESHOLD: begin
                        threshold <= rx_data;
                        check     <= check ^ rx_data;
                        state     <= S_WIDTH_LO;
                    end
                    S_WIDTH_LO: begin
                        width[7:0] <= rx_data;
                        check      <= check ^ rx_data;
                        state      <= S_WIDTH_HI;
                    end
                    S_WIDTH_HI: begin
                        width[15:8] <= rx_data;
                        check       <= check ^ rx_data;
                        state       <= S_HEIGHT_LO;
                    end
                    S_HEIGHT_LO: begin
                        height[7:0] <= rx_data;
                        check       <= check ^ rx_data;
                        state       <= S_HEIGHT_HI;
                    end
                    S_HEIGHT_HI: begin
                        height[15:8] <= rx_data;
                        check        <= check ^ rx_data;
                        state        <= S_CHECK;
                    end
                    S_CHECK: begin
                        if (rx_data == check && size_ok) begin
                            state    <= S_PIXELS;
                            byte_idx <= 2'd0;
                            pix_col  <= 16'd0;
                            pix_row  <= 16'd0;
                        end else begin
                            state       <= S_MAGIC_0;
                            frame_error <= 1'b1;
                        end
                    end
                    S_PIXELS: begin
                        if (byte_idx == 2'd0) blue  <= rx_data;
                        if (byte_idx == 2'd1) green <= rx_data;
                        byte_idx <= pixel_complete ? 2'd0 : byte_idx + 2'd1;
                    end
                    default:
                        state <= S_MAGIC_0;
                endcase
            end

            // Keep track of where the pixel leaving for sobel_core sits in
            // the frame, and finish the frame after its last pixel.
            if (pix_valid) begin
                if (last_col) begin
                    pix_col <= 16'd0;
                    pix_row <= pix_row + 16'd1;
                end else begin
                    pix_col <= pix_col + 16'd1;
                end
                if (last_pixel) begin
                    state      <= S_MAGIC_0;
                    frame_done <= 1'b1;
                end
            end
        end
    end
endmodule

`default_nettype wire
