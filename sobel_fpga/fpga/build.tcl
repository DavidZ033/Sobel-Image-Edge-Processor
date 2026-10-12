# Builds the Sobel design for the Digilent Cmod S7-25 with Vivado.
#
# Three ways to run it on Windows:
#   1. In the Vivado window, with no project open:
#          Tools > Run Tcl Script...   and pick this file
#   2. Start menu > "Vivado <version> Tcl Shell", then at the Vivado% prompt:
#          cd C:/path/to/fpga              (forward slashes)
#          source build.tcl
#   3. In a Command Prompt, after running settings64.bat from the Vivado
#      install folder so that the vivado command is known:
#          vivado -mode batch -source build.tcl
#          vivado -mode batch -source build.tcl -tclargs 12000000
#
# It creates a normal Vivado project in the "vivado" folder next to this file
# (open vivado/sobel_fpga.xpr later to look around), runs synthesis and
# implementation, and copies the finished bitstream to sobel_top.bit here.
# A build takes a few minutes.
#
# Keep this folder somewhere with a short, plain path such as C:/fpga/sobel.
# Vivado has trouble with spaces, special characters and very long paths.

# Serial speed in bits per second. It must match --baud on the laptop.
# 3000000 is the safe starting point. 12000000 is the fastest the board's USB
# chip can do and gives up to 4 times the frame rate. 6000000 and 8000000 are
# the steps in between, in case 12000000 turns out to lose bytes.
set BAUD 3000000

if {[info exists ::argv] && [string is integer -strict [lindex $::argv 0]]} {
    set BAUD [lindex $::argv 0]
}

set here [file dirname [file normalize [info script]]]

if {[regexp {[^A-Za-z0-9_~:/.-]} $here] || [string length $here] > 100} {
    puts "WARNING: this folder's path is long, or has spaces or special characters in it:"
    puts "    $here"
    puts "Vivado can fail in such folders. If the build fails, move the project to a"
    puts "short, plain path such as C:/fpga/sobel and run this script again."
}

# Re-running the script replaces the previous project. Any other open project
# is left alone.
if {[llength [get_projects -quiet]] > 0} {
    if {[get_property NAME [current_project]] eq "sobel_fpga"} {
        close_project
    } else {
        error "Another project is open. Close it (File > Close Project) and run this script again."
    }
}

create_project -force sobel_fpga [file join $here vivado] -part xc7s25csga225-1
add_files [glob -directory [file join $here rtl] *.v]
add_files -fileset constrs_1 [list [file join $here constraints cmod_s7.xdc]]
set_property top sobel_top [current_fileset]
set_property generic "BAUD=$BAUD" [current_fileset]
update_compile_order -fileset sources_1

puts "Building for $BAUD baud. This takes a few minutes..."

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "Synthesis did not finish: [get_property STATUS [get_runs synth_1]]. See the Messages tab, or vivado/sobel_fpga.runs/synth_1/runme.log."
}

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "Implementation did not finish: [get_property STATUS [get_runs impl_1]]. See the Messages tab, or vivado/sobel_fpga.runs/impl_1/runme.log."
}

set bitstream [file join $here sobel_top.bit]
file copy -force [file join $here vivado sobel_fpga.runs impl_1 sobel_top.bit] $bitstream

# Worst negative slack: how much time the slowest path has to spare at 96 MHz.
# Zero or more means every path is fast enough.
set slack [get_property STATS.WNS [get_runs impl_1]]
puts ""
puts "Built for $BAUD baud: $bitstream"
if {$slack eq ""} {
    puts "Timing: open the project and check the timing summary."
} elseif {$slack < 0} {
    puts "WARNING: timing was not met (slack $slack ns). The design may misbehave."
} else {
    puts "Timing met with $slack ns to spare."
}
puts "Next: load it onto the board with Open Hardware Manager > Open Target >"
puts "Auto Connect > Program Device, or run program.tcl."
