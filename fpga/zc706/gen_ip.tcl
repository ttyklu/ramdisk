# gen_ip.tcl - generate the ZC706 board IP for the NVMe RAM disk into fpga/zc706/ip/.
#   cd fpga/zc706 && vivado -mode batch -source gen_ip.tcl [-tclargs <mig.prj>]
#  * pcie   : 7 Series Integrated Block for PCIe, Gen2 x4, 128-bit AXI-Stream @ 125 MHz,
#             BAR0 64-bit 16 KB, class 010802 (NVMe), MSI off, MSI-X 5 vectors (table BAR0+0x2000,
#             PBA BAR0+0x3000), ZC706 reference-board GT placement
#  * ddr    : MIG 7 series, PL DDR3 SODIMM from the ZC706 board preset (AXI4 512-bit, ui_clk 200 MHz)
#  * axi_cc : AXI clock converter, 64-bit, user_clk (125 MHz) -> ui_clk (200 MHz)
#  * axi_dw : AXI data width converter, 64 -> 512 bits at ui_clk
#  * axi_rs : AXI register slice, 512-bit, all channels registered (timing margin into the MIG)
set here [file dirname [file normalize [info script]]]
set board_dir /home/guangming/Xilinx/2026.1/Vivado/data/xhub/boards/XilinxBoardStore/boards/Xilinx/zc706/1.5
set prj [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] : "$board_dir/mig.prj"}]
set ipdir $here/ip
file mkdir $ipdir

create_project -in_memory -part xc7z045ffg900-2
set_property board_part xilinx.com:zc706:part0:1.5 [current_project]
set_property target_language Verilog [current_project]

# ---------------- PCIe ----------------
create_ip -name pcie_7x -vendor xilinx.com -library ip -module_name pcie -dir $ipdir
set_property CONFIG.mode_selection {Advanced} [get_ips pcie]      ;# exposes shared_logic_in_core
set_property -dict [list \
  CONFIG.Xlnx_Ref_Board {ZC706} \
  CONFIG.Maximum_Link_Width {X4} \
  CONFIG.Link_Speed {5.0_GT/s} \
  CONFIG.Trgt_Link_Speed {4'h2} \
  CONFIG.Interface_Width {128_bit} \
  CONFIG.User_Clk_Freq {125} \
  CONFIG.Device_ID {7024} \
  CONFIG.Bar0_Enabled {true} CONFIG.Bar0_Type {Memory} CONFIG.Bar0_64bit {true} \
  CONFIG.Bar0_Prefetchable {false} CONFIG.Bar0_Scale {Kilobytes} CONFIG.Bar0_Size {16} \
  CONFIG.Use_Class_Code_Lookup_Assistant {false} \
  CONFIG.Class_Code_Base {01} CONFIG.Class_Code_Sub {08} CONFIG.Class_Code_Interface {02} \
  CONFIG.MSI_Enabled {false} \
  CONFIG.MSIx_Enabled {true} CONFIG.MSIx_Table_Size {5} \
  CONFIG.MSIx_Table_BIR {BAR_1:0} CONFIG.MSIx_Table_Offset {2000} \
  CONFIG.MSIx_PBA_BIR {BAR_1:0} CONFIG.MSIx_PBA_Offset {3000} \
  CONFIG.Max_Payload_Size {256_bytes} \
] [get_ips pcie]
# clocking (pipe clock MMCM) and GT_COMMON inside the core, not in an external support wrapper
set_property CONFIG.en_ext_clk {false} [get_ips pcie]
set_property CONFIG.shared_logic_in_core {true} [get_ips pcie]

# ---------------- DDR3 (MIG) ----------------
create_ip -name mig_7series -vendor xilinx.com -library ip -module_name ddr -dir $ipdir
file copy -force $prj $ipdir/ddr/mig_zc706.prj
set_property CONFIG.XML_INPUT_FILE [file normalize $ipdir/ddr/mig_zc706.prj] [get_ips ddr]

# ---------------- AXI clock / width conversion ----------------
create_ip -name axi_clock_converter -vendor xilinx.com -library ip -module_name axi_cc -dir $ipdir
set_property -dict [list CONFIG.PROTOCOL {AXI4} CONFIG.DATA_WIDTH {64} CONFIG.ADDR_WIDTH {32} \
  CONFIG.ID_WIDTH {0} CONFIG.ACLK_ASYNC {1}] [get_ips axi_cc]

create_ip -name axi_dwidth_converter -vendor xilinx.com -library ip -module_name axi_dw -dir $ipdir
set_property -dict [list CONFIG.PROTOCOL {AXI4} CONFIG.SI_DATA_WIDTH {64} CONFIG.MI_DATA_WIDTH {512} \
  CONFIG.ADDR_WIDTH {32} CONFIG.SI_ID_WIDTH {0}] [get_ips axi_dw]

create_ip -name axi_register_slice -vendor xilinx.com -library ip -module_name axi_rs -dir $ipdir
set_property -dict [list CONFIG.PROTOCOL {AXI4} CONFIG.DATA_WIDTH {512} CONFIG.ADDR_WIDTH {32} \
  CONFIG.ID_WIDTH {0} CONFIG.REG_AW {1} CONFIG.REG_AR {1} CONFIG.REG_W {1} CONFIG.REG_R {1} CONFIG.REG_B {1}] \
  [get_ips axi_rs]

foreach ip {pcie ddr axi_cc axi_dw axi_rs} {
  generate_target {instantiation_template synthesis} [get_ips $ip]
  puts "GEN $ip done"
}
puts "PCIE_CFG msix_table_size=[get_property CONFIG.MSIx_Table_Size [get_ips pcie]] bar0=[get_property CONFIG.BAR0_Size_Vector [get_ips pcie]] link=[get_property CONFIG.Maximum_Link_Width [get_ips pcie]]/[get_property CONFIG.Link_Speed [get_ips pcie]] if=[get_property CONFIG.Interface_Width [get_ips pcie]]@[get_property CONFIG.User_Clk_Freq [get_ips pcie]]"
