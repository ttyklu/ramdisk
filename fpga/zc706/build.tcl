# build.tcl - full ZC706 implementation of the NVMe RAM disk: synthesis, place & route, bitstream.
#   cd fpga/zc706 && vivado -mode batch -source gen_ip.tcl    (once, or after IP changes)
#   cd fpga/zc706 && vivado -mode batch -source build.tcl [-tclargs <REC_LOG2>]
# Output: fpga/zc706/out/zc706_nvme.bit and reports in fpga/zc706/out/.
set rec  [expr {[llength $argv] > 0 ? [lindex $argv 0] : 17}]
set here [file dirname [file normalize [info script]]]
set rtl  [file join $here .. .. rtl]
set out  [file join $here out]
file mkdir $out

create_project -in_memory -part xc7z045ffg900-2
set_property board_part xilinx.com:zc706:part0:1.5 [current_project]
set_param general.maxThreads 8

foreach ip {pcie ddr axi_cc axi_dw axi_rs} { read_ip [file join $here ip $ip $ip.xci] }
# out-of-context IP synthesis (checkpoints are cached next to the .xci)
foreach ip [get_ips] {
  generate_target {synthesis instantiation_template} $ip
  synth_ip $ip
}
foreach f {nvme_pkg nvme_identify ram_1r1w dw_ram2 dw_fifo rx128_to_64 tx64_to_128 tlp_rx tlp_tx nvme_regs
           dma_rd mwr_eng ddr_rd ddr_wr fletcher4 deflate_enc inflate xform nvme_ctrl nvme_top} {
  read_verilog -sv [file join $rtl $f.sv]
}
read_verilog -sv [file join $here zc706_top.sv]
read_xdc [file join $here zc706.xdc]

synth_design -top zc706_top -part xc7z045ffg900-2 -generic REC_LOG2=$rec -flatten_hierarchy rebuilt
write_checkpoint -force $out/synth.dcp
report_utilization -file $out/util_synth.rpt

opt_design
place_design
phys_opt_design
route_design
# post-route physical optimization if any setup path is still negative
if {[get_property SLACK [get_timing_paths -max_paths 1 -setup]] < 0} {
  phys_opt_design -directive AggressiveExplore
}
write_checkpoint -force $out/route.dcp
report_utilization -file $out/util_route.rpt
report_utilization -hierarchical -hierarchical_depth 2 -file $out/util_hier_route.rpt
report_timing_summary -max_paths 30 -file $out/timing_route.rpt
report_clock_utilization -file $out/clock_util.rpt
report_drc -file $out/drc.rpt
report_io -file $out/io.rpt

set wns [get_property SLACK [get_timing_paths -max_paths 1 -setup]]
set whs [get_property SLACK [get_timing_paths -max_paths 1 -hold]]
puts [format "RESULT WNS=%.3f WHS=%.3f" $wns $whs]
write_bitstream -force $out/zc706_nvme.bit
puts "RESULT bitstream $out/zc706_nvme.bit"
