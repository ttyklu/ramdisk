# quick_mod.tcl - out-of-context synth + place & route of one module, for timing iteration.
#   vivado -mode batch -source quick_mod.tcl -tclargs <top> <period_ns> <file.sv> ...
set top    [lindex $argv 0]
set period [lindex $argv 1]
set files  [lrange $argv 2 end]
set part   xc7z045ffg900-2
set_part $part
set_param general.maxThreads 8
foreach f $files { read_verilog -sv $f }
synth_design -top $top -part $part -mode out_of_context -flatten_hierarchy rebuilt
create_clock -name clk -period $period [get_ports clk]
opt_design
place_design
route_design
set p [get_timing_paths -max_paths 1 -setup]
puts [format "QUICK %s period=%s WNS=%.3f levels=%s from=%s to=%s" $top $period \
      [get_property SLACK $p] [get_property LOGIC_LEVELS $p] \
      [get_property NAME [get_property STARTPOINT_PIN $p]] [get_property NAME [get_property ENDPOINT_PIN $p]]]
foreach q [get_timing_paths -max_paths 12 -nworst 1 -unique_pins -setup] {
  puts [format "QPATH %.3f %s -> %s" [get_property SLACK $q] \
        [get_property NAME [get_property STARTPOINT_PIN $q]] [get_property NAME [get_property ENDPOINT_PIN $q]]]
}
