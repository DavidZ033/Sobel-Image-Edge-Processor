## Pin and timing constraints for sobel_top on the Digilent Cmod S7-25 (rev. B).
## Pin numbers are from Digilent's master XDC for this board.

## 12 MHz oscillator
set_property -dict { PACKAGE_PIN M9  IOSTANDARD LVCMOS33 } [get_ports clk12]
create_clock -name clk12 -period 83.333 [get_ports clk12]

## BTN0: reset (high while pressed)
set_property -dict { PACKAGE_PIN D2  IOSTANDARD LVCMOS33 } [get_ports btn_reset]

## The four green LEDs (high = on)
set_property -dict { PACKAGE_PIN E2  IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN K1  IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN J1  IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN E1  IOSTANDARD LVCMOS33 } [get_ports {led[3]}]

## RGB LED (low = on). The design holds all three high so it stays dark.
set_property -dict { PACKAGE_PIN F2  IOSTANDARD LVCMOS33 } [get_ports rgb_r]
set_property -dict { PACKAGE_PIN D3  IOSTANDARD LVCMOS33 } [get_ports rgb_g]
set_property -dict { PACKAGE_PIN F1  IOSTANDARD LVCMOS33 } [get_ports rgb_b]

## USB serial port (the FT2232 chip's second channel), named from the FPGA's side:
## uart_txd goes to the laptop, uart_rxd comes from it.
set_property -dict { PACKAGE_PIN L12 IOSTANDARD LVCMOS33 } [get_ports uart_txd]
set_property -dict { PACKAGE_PIN K15 IOSTANDARD LVCMOS33 } [get_ports uart_rxd]

## These pins have no timing relationship with the clock: the inputs go
## through synchronizer flip-flops and the outputs are slow, so tell the
## timing analysis not to worry about them.
set_false_path -from [get_ports {uart_rxd btn_reset}]
set_false_path -to   [get_ports {uart_txd led[*] rgb_r rgb_g rgb_b}]

## Configuration settings for this board
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
