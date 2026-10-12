# Loads sobel_top.bit onto the Cmod S7 over its USB cable.
#
# Run it the same ways as build.tcl:
#   * Vivado window:      Tools > Run Tcl Script...   and pick this file
#   * Vivado Tcl Shell:   cd C:/path/to/fpga   then   source program.tcl
#   * Command Prompt (after settings64.bat):
#                         vivado -mode batch -source program.tcl
# or do the same thing by hand: Open Hardware Manager > Open Target >
# Auto Connect > Program Device, and pick sobel_top.bit.
#
# The design stays loaded until the board loses power, so load it again
# after plugging the board back in.

set here [file dirname [file normalize [info script]]]
set bitstream [file join $here sobel_top.bit]
if {![file exists $bitstream]} {
    error "sobel_top.bit is not here yet. Run build.tcl first."
}

if {[catch {open_hw_manager}]} {
    open_hw                      ;# name of the command before Vivado 2019.2
}
connect_hw_server
if {[catch {open_hw_target} problem]} {
    catch {disconnect_hw_server}
    error "No Cmod S7 found. Is it plugged in, and are the cable drivers installed? ($problem)"
}

set device [lindex [get_hw_devices -quiet xc7s25*] 0]
if {$device eq ""} {
    catch {close_hw_target}
    catch {disconnect_hw_server}
    error "A programming cable answered, but there is no Cmod S7 (xc7s25) on it."
}
current_hw_device $device
set_property PROGRAM.FILE $bitstream $device
program_hw_devices $device

close_hw_target
disconnect_hw_server
puts "Loaded. LED0 on the board should be blinking."
