`timescale 1ns / 1fs

// Simulation-only stand-ins for the two Xilinx parts sobel_top.v uses, so the
// whole design can be simulated with the free Icarus Verilog simulator.
// Vivado never sees this file; it has the real parts built in.

// Clock multiplier: just produces the output clock the real one would
// (period = input period * DIVCLK_DIVIDE * CLKOUT0_DIVIDE_F / CLKFBOUT_MULT_F)
// and reports "locked" a little while after power-up.
module MMCME2_BASE #(
    parameter         BANDWIDTH        = "OPTIMIZED",
    parameter real    CLKIN1_PERIOD    = 0.0,
    parameter integer DIVCLK_DIVIDE    = 1,
    parameter real    CLKFBOUT_MULT_F  = 5.0,
    parameter real    CLKOUT0_DIVIDE_F = 1.0
) (
    input  wire CLKIN1,
    input  wire CLKFBIN,
    output wire CLKFBOUT,
    output reg  CLKOUT0,
    output reg  LOCKED,
    input  wire PWRDWN,
    input  wire RST
);
    localparam real HALF_PERIOD =
        CLKIN1_PERIOD * DIVCLK_DIVIDE * CLKOUT0_DIVIDE_F / CLKFBOUT_MULT_F / 2.0;

    assign CLKFBOUT = CLKIN1;

    initial begin
        CLKOUT0 = 1'b0;
        LOCKED  = 1'b0;
        #(20.0 * CLKIN1_PERIOD) LOCKED = 1'b1;
    end

    always #(HALF_PERIOD) CLKOUT0 = ~CLKOUT0;
endmodule

// Clock buffer: a plain wire as far as simulation is concerned.
module BUFG (
    input  wire I,
    output wire O
);
    assign O = I;
endmodule
