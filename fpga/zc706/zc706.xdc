# zc706.xdc - board constraints for zc706_top (pins not already set by the PCIe / MIG IP).
# The PCIe IP places the GT lanes (GTXE2_CHANNEL_X0Y12..15, reference board ZC706) and the MIG
# places the DDR3 pins and the 200 MHz sys_clk (H9 / G9).

# ---- PCIe ----
create_clock -name pcie_refclk -period 10.000 [get_ports pcie_refclk_p]
set_property LOC IBUFDS_GTE2_X0Y6 [get_cells refclk_ibuf]

set_property PACKAGE_PIN AK23 [get_ports pcie_perst_n]
set_property IOSTANDARD LVCMOS25 [get_ports pcie_perst_n]   ;# bank 11 is 2.5 V (shared with led[0])
set_property PULLUP true [get_ports pcie_perst_n]
set_false_path -from [get_ports pcie_perst_n]

# ---- CPU reset push button (MIG sys_rst, active high) ----
set_property PACKAGE_PIN A8 [get_ports cpu_reset]
set_property IOSTANDARD LVCMOS15 [get_ports cpu_reset]
set_false_path -from [get_ports cpu_reset]

# ---- user LEDs ----
set_property PACKAGE_PIN Y21 [get_ports {led[0]}]
set_property IOSTANDARD LVCMOS25 [get_ports {led[0]}]
set_property PACKAGE_PIN G2 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS15 [get_ports {led[1]}]
set_property PACKAGE_PIN W21 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS25 [get_ports {led[2]}]
set_property PACKAGE_PIN A17 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS15 [get_ports {led[3]}]
set_false_path -to [get_ports {led[*]}]

# ---- clock domains: PCIe user_clk and MIG ui_clk only meet in the AXI clock converter ----
set_clock_groups -asynchronous \
  -group [get_clocks -of_objects [get_pins u_pcie/user_clk_out]] \
  -group [get_clocks -of_objects [get_pins u_ddr/ui_clk]]

# ---- bitstream ----
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property CONFIG_VOLTAGE 2.5 [current_design]
set_property CFGBVS VCCO [current_design]
