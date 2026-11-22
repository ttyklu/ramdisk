# ooc_build.tcl - out-of-context synthesis + place & route of nvme_top on the ZC706 part.
#   cd fpga && vivado -mode batch -source ooc_build.tcl [-tclargs <REC_LOG2> [synth|route] [period_ns]]
# Reports go to fpga/reports_rec<REC_LOG2>/. No PCIe / MIG IP and no pins: this measures the
# design's own resources and its timing at user_clk (default 8 ns = 125 MHz).
set rec   [expr {[llength $argv] > 0 ? [lindex $argv 0] : 17}]
set stage  [expr {[llength $argv] > 1 ? [lindex $argv 1] : "route"}]
set period [expr {[llength $argv] > 2 ? [lindex $argv 2] : 8.000}]
set part  xc7z045ffg900-2
set here  [file dirname [file normalize [info script]]]
set rtl   [file join $here .. rtl]
set out   [file join $here reports_rec$rec]
file mkdir $out

set_part $part
set_param general.maxThreads 8
foreach f {nvme_pkg nvme_identify ram_1r1w dw_ram2 dw_fifo rx128_to_64 tx64_to_128 tlp_rx tlp_tx nvme_regs dma_rd mwr_eng
           ddr_rd ddr_wr fletcher4 deflate_enc inflate xform nvme_ctrl nvme_top} {
  read_verilog -sv [file join $rtl $f.sv]
}

synth_design -top nvme_top -part $part -mode out_of_context -generic REC_LOG2=$rec \
             -flatten_hierarchy rebuilt
create_clock -name user_clk -period $period [get_ports clk]
report_utilization -file $out/util_synth.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/util_hier_synth.rpt
report_timing_summary -max_paths 20 -file $out/timing_synth.rpt
write_checkpoint -force $out/synth.dcp
if {$stage eq "synth"} { exit }

opt_design
place_design
phys_opt_design
route_design
report_utilization -file $out/util_route.rpt
report_utilization -hierarchical -hierarchical_depth 3 -file $out/util_hier_route.rpt
report_timing_summary -max_paths 50 -file $out/timing_route.rpt
report_timing -max_paths 30 -sort_by group -nworst 1 -unique_pins -file $out/worst_paths.rpt
write_checkpoint -force $out/route.dcp
