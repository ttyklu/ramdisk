// zc706_top.sv - ZC706 board top level for the NVMe RAM disk.
//
//   PCIe edge (Gen2 x4) --- pcie (7 Series Integrated Block, 128-bit AXI-Stream, 125 MHz)
//                              |  user_clk_out / user_reset_out
//                           nvme_top (REC_LOG2: 128 KB records by default)
//                              |  AXI4 64-bit @ 125 MHz
//                           axi_cc (clock converter, async) -> ui_clk 200 MHz
//                           axi_dw (64 -> 512 bits)
//                           ddr    (MIG 7 series, PL DDR3 SODIMM, 1 GB)
//
// LEDs: 0 = PCIe link up, 1 = DDR3 calibrated, 2 = user_clk heartbeat, 3 = DMA error (sticky)
module zc706_top #(
  parameter int REC_LOG2 = 17
)(
  // PCIe edge connector
  output logic [3:0]  pci_exp_txp,
  output logic [3:0]  pci_exp_txn,
  input  logic [3:0]  pci_exp_rxp,
  input  logic [3:0]  pci_exp_rxn,
  input  logic        pcie_refclk_p,      // 100 MHz from the slot
  input  logic        pcie_refclk_n,
  input  logic        pcie_perst_n,       // PERST#

  // PL DDR3 SODIMM
  inout  wire  [63:0] ddr3_dq,
  inout  wire  [7:0]  ddr3_dqs_n,
  inout  wire  [7:0]  ddr3_dqs_p,
  output logic [13:0] ddr3_addr,
  output logic [2:0]  ddr3_ba,
  output logic        ddr3_ras_n,
  output logic        ddr3_cas_n,
  output logic        ddr3_we_n,
  output logic        ddr3_reset_n,
  output logic [0:0]  ddr3_ck_p,
  output logic [0:0]  ddr3_ck_n,
  output logic [0:0]  ddr3_cke,
  output logic [0:0]  ddr3_cs_n,
  output logic [7:0]  ddr3_dm,
  output logic [0:0]  ddr3_odt,
  input  logic        sys_clk_p,          // 200 MHz system clock (MIG reference)
  input  logic        sys_clk_n,
  input  logic        cpu_reset,          // push button, active high

  output logic [3:0]  led
);

  // ---------------- PCIe ----------------
  logic pcie_refclk;
  IBUFDS_GTE2 refclk_ibuf (.O(pcie_refclk), .ODIV2(), .I(pcie_refclk_p), .IB(pcie_refclk_n), .CEB(1'b0));

  logic         user_clk, user_reset, user_lnk_up;
  logic [127:0] rx_tdata, tx_tdata;
  logic [15:0]  rx_tkeep, tx_tkeep;
  logic         rx_tlast, rx_tvalid, rx_tready, tx_tlast, tx_tvalid, tx_tready;
  logic [21:0]  rx_tuser;
  logic [3:0]   tx_tuser;
  logic [7:0]   cfg_bus_number;
  logic [4:0]   cfg_device_number;
  logic [2:0]   cfg_function_number;
  logic         cfg_msixenable, cfg_msixfm, cfg_to_turnoff;

  pcie u_pcie (
    .pci_exp_txp, .pci_exp_txn, .pci_exp_rxp, .pci_exp_rxn,
    .int_pclk_out_slave(), .int_pipe_rxusrclk_out(), .int_rxoutclk_out(), .int_dclk_out(),
    .int_mmcm_lock_out(), .int_userclk1_out(), .int_userclk2_out(), .int_oobclk_out(),
    .int_qplllock_out(), .int_qplloutclk_out(), .int_qplloutrefclk_out(), .int_pclk_sel_slave('0),
    .user_clk_out(user_clk), .user_reset_out(user_reset), .user_lnk_up(user_lnk_up), .user_app_rdy(),
    .tx_buf_av(), .tx_cfg_req(), .tx_err_drop(),
    .s_axis_tx_tready(tx_tready), .s_axis_tx_tdata(tx_tdata), .s_axis_tx_tkeep(tx_tkeep),
    .s_axis_tx_tlast(tx_tlast), .s_axis_tx_tvalid(tx_tvalid), .s_axis_tx_tuser(tx_tuser),
    .tx_cfg_gnt(1'b1),
    .m_axis_rx_tdata(rx_tdata), .m_axis_rx_tkeep(rx_tkeep), .m_axis_rx_tlast(rx_tlast),
    .m_axis_rx_tvalid(rx_tvalid), .m_axis_rx_tready(rx_tready), .m_axis_rx_tuser(rx_tuser),
    .rx_np_ok(1'b1), .rx_np_req(1'b1),
    .fc_cpld(), .fc_cplh(), .fc_npd(), .fc_nph(), .fc_pd(), .fc_ph(), .fc_sel('0),
    .cfg_mgmt_do(), .cfg_mgmt_rd_wr_done(), .cfg_status(), .cfg_command(), .cfg_dstatus(),
    .cfg_dcommand(), .cfg_lstatus(), .cfg_lcommand(), .cfg_dcommand2(), .cfg_pcie_link_state(),
    .cfg_pmcsr_pme_en(), .cfg_pmcsr_powerstate(), .cfg_pmcsr_pme_status(), .cfg_received_func_lvl_rst(),
    .cfg_mgmt_di('0), .cfg_mgmt_byte_en('0), .cfg_mgmt_dwaddr('0), .cfg_mgmt_wr_en(1'b0),
    .cfg_mgmt_rd_en(1'b0), .cfg_mgmt_wr_readonly(1'b0),
    .cfg_err_ecrc(1'b0), .cfg_err_ur(1'b0), .cfg_err_cpl_timeout(1'b0), .cfg_err_cpl_unexpect(1'b0),
    .cfg_err_cpl_abort(1'b0), .cfg_err_posted(1'b0), .cfg_err_cor(1'b0), .cfg_err_atomic_egress_blocked(1'b0),
    .cfg_err_internal_cor(1'b0), .cfg_err_malformed(1'b0), .cfg_err_mc_blocked(1'b0), .cfg_err_poisoned(1'b0),
    .cfg_err_norecovery(1'b0), .cfg_err_tlp_cpl_header('0), .cfg_err_cpl_rdy(), .cfg_err_locked(1'b0),
    .cfg_err_acs(1'b0), .cfg_err_internal_uncor(1'b0),
    .cfg_trn_pending(1'b0), .cfg_pm_halt_aspm_l0s(1'b0), .cfg_pm_halt_aspm_l1(1'b0),
    .cfg_pm_force_state_en(1'b0), .cfg_pm_force_state('0), .cfg_dsn(64'h0000_0001_10EE_7024),
    .cfg_interrupt(1'b0), .cfg_interrupt_rdy(), .cfg_interrupt_assert(1'b0), .cfg_interrupt_di('0),
    .cfg_interrupt_do(), .cfg_interrupt_mmenable(), .cfg_interrupt_msienable(),
    .cfg_interrupt_msixenable(cfg_msixenable), .cfg_interrupt_msixfm(cfg_msixfm),
    .cfg_interrupt_stat(1'b0), .cfg_pciecap_interrupt_msgnum('0),
    .cfg_to_turnoff(cfg_to_turnoff), .cfg_turnoff_ok(cfg_to_turnoff),     // a RAM disk can always turn off
    .cfg_bus_number(cfg_bus_number), .cfg_device_number(cfg_device_number),
    .cfg_function_number(cfg_function_number),
    .cfg_pm_wake(1'b0), .cfg_pm_send_pme_to(1'b0),
    .cfg_ds_bus_number('0), .cfg_ds_device_number('0), .cfg_ds_function_number('0),
    .cfg_mgmt_wr_rw1c_as_rw(1'b0), .cfg_msg_received(), .cfg_msg_data(),
    .cfg_bridge_serr_en(), .cfg_slot_control_electromech_il_ctl_pulse(),
    .cfg_root_control_syserr_corr_err_en(), .cfg_root_control_syserr_non_fatal_err_en(),
    .cfg_root_control_syserr_fatal_err_en(), .cfg_root_control_pme_int_en(),
    .cfg_aer_rooterr_corr_err_reporting_en(), .cfg_aer_rooterr_non_fatal_err_reporting_en(),
    .cfg_aer_rooterr_fatal_err_reporting_en(), .cfg_aer_rooterr_corr_err_received(),
    .cfg_aer_rooterr_non_fatal_err_received(), .cfg_aer_rooterr_fatal_err_received(),
    .cfg_msg_received_err_cor(), .cfg_msg_received_err_non_fatal(), .cfg_msg_received_err_fatal(),
    .cfg_msg_received_pm_as_nak(), .cfg_msg_received_pm_pme(), .cfg_msg_received_pme_to_ack(),
    .cfg_msg_received_assert_int_a(), .cfg_msg_received_assert_int_b(), .cfg_msg_received_assert_int_c(),
    .cfg_msg_received_assert_int_d(), .cfg_msg_received_deassert_int_a(), .cfg_msg_received_deassert_int_b(),
    .cfg_msg_received_deassert_int_c(), .cfg_msg_received_deassert_int_d(),
    .cfg_msg_received_setslotpowerlimit(),
    .pl_directed_link_change('0), .pl_directed_link_width('0), .pl_directed_link_speed(1'b0),
    .pl_directed_link_auton(1'b0), .pl_upstream_prefer_deemph(1'b1),
    .pl_sel_lnk_rate(), .pl_sel_lnk_width(), .pl_ltssm_state(), .pl_lane_reversal_mode(),
    .pl_phy_lnk_up(), .pl_tx_pm_state(), .pl_rx_pm_state(), .pl_link_upcfg_cap(), .pl_link_gen2_cap(),
    .pl_link_partner_gen2_supported(), .pl_initial_link_width(), .pl_directed_change_done(),
    .pl_received_hot_rst(), .pl_transmit_hot_rst(1'b0), .pl_downstream_deemph_source(1'b0),
    .cfg_err_aer_headerlog('0), .cfg_aer_interrupt_msgnum('0), .cfg_err_aer_headerlog_set(),
    .cfg_aer_ecrc_check_en(), .cfg_aer_ecrc_gen_en(), .cfg_vc_tcvc_map(),
    .sys_clk(pcie_refclk), .sys_rst_n(pcie_perst_n),
    .pcie_drp_clk(1'b1), .pcie_drp_en(1'b0), .pcie_drp_we(1'b0), .pcie_drp_addr('0), .pcie_drp_di('0),
    .pcie_drp_do(), .pcie_drp_rdy()
  );

  // ---------------- NVMe controller ----------------
  logic [31:0] n_awaddr, n_araddr;
  logic [7:0]  n_awlen, n_arlen;
  logic [2:0]  n_awsize, n_arsize;
  logic [1:0]  n_awburst, n_arburst, n_bresp, n_rresp;
  logic        n_awvalid, n_awready, n_wlast, n_wvalid, n_wready, n_bvalid, n_bready;
  logic        n_arvalid, n_arready, n_rlast, n_rvalid, n_rready;
  logic [63:0] n_wdata, n_rdata;
  logic [7:0]  n_wstrb;
  logic        dma_err;

  nvme_top #(.REC_LOG2(REC_LOG2)) u_nvme (
    .clk(user_clk), .rst(user_reset),
    .rx_tdata, .rx_tvalid, .rx_tready, .rx_tuser,
    .tx_tdata, .tx_tkeep, .tx_tlast, .tx_tvalid, .tx_tready, .tx_tuser,
    .cfg_bus_number, .cfg_device_number, .cfg_function_number,
    .cfg_interrupt_msixenable(cfg_msixenable), .cfg_interrupt_msixfm(cfg_msixfm),
    .m_axi_awaddr(n_awaddr), .m_axi_awlen(n_awlen), .m_axi_awsize(n_awsize), .m_axi_awburst(n_awburst),
    .m_axi_awvalid(n_awvalid), .m_axi_awready(n_awready),
    .m_axi_wdata(n_wdata), .m_axi_wstrb(n_wstrb), .m_axi_wlast(n_wlast), .m_axi_wvalid(n_wvalid),
    .m_axi_wready(n_wready),
    .m_axi_bresp(n_bresp), .m_axi_bvalid(n_bvalid), .m_axi_bready(n_bready),
    .m_axi_araddr(n_araddr), .m_axi_arlen(n_arlen), .m_axi_arsize(n_arsize), .m_axi_arburst(n_arburst),
    .m_axi_arvalid(n_arvalid), .m_axi_arready(n_arready),
    .m_axi_rdata(n_rdata), .m_axi_rresp(n_rresp), .m_axi_rlast(n_rlast), .m_axi_rvalid(n_rvalid),
    .m_axi_rready(n_rready),
    .dma_err
  );

  // ---------------- DDR3 (MIG) and the AXI path ----------------
  logic ui_clk, ui_rst, calib_done, mmcm_locked;
  logic ui_aresetn;
  always_ff @(posedge ui_clk) ui_aresetn <= !ui_rst && calib_done;   // DDR side held until calibrated

  // clock converter output (64-bit @ ui_clk) -> width converter input
  logic [31:0] c_awaddr, c_araddr;
  logic [7:0]  c_awlen, c_arlen;
  logic [2:0]  c_awsize, c_arsize, c_awprot, c_arprot;
  logic [1:0]  c_awburst, c_arburst, c_bresp, c_rresp;
  logic [0:0]  c_awlock, c_arlock;
  logic [3:0]  c_awcache, c_arcache, c_awregion, c_arregion, c_awqos, c_arqos;
  logic        c_awvalid, c_awready, c_wlast, c_wvalid, c_wready, c_bvalid, c_bready;
  logic        c_arvalid, c_arready, c_rlast, c_rvalid, c_rready;
  logic [63:0] c_wdata, c_rdata;
  logic [7:0]  c_wstrb;

  axi_cc u_axi_cc (
    .s_axi_aclk(user_clk), .s_axi_aresetn(!user_reset),
    .s_axi_awaddr(n_awaddr), .s_axi_awlen(n_awlen), .s_axi_awsize(n_awsize), .s_axi_awburst(n_awburst),
    .s_axi_awlock(1'b0), .s_axi_awcache(4'b0011), .s_axi_awprot(3'b000), .s_axi_awregion(4'd0),
    .s_axi_awqos(4'd0), .s_axi_awvalid(n_awvalid), .s_axi_awready(n_awready),
    .s_axi_wdata(n_wdata), .s_axi_wstrb(n_wstrb), .s_axi_wlast(n_wlast), .s_axi_wvalid(n_wvalid),
    .s_axi_wready(n_wready), .s_axi_bresp(n_bresp), .s_axi_bvalid(n_bvalid), .s_axi_bready(n_bready),
    .s_axi_araddr(n_araddr), .s_axi_arlen(n_arlen), .s_axi_arsize(n_arsize), .s_axi_arburst(n_arburst),
    .s_axi_arlock(1'b0), .s_axi_arcache(4'b0011), .s_axi_arprot(3'b000), .s_axi_arregion(4'd0),
    .s_axi_arqos(4'd0), .s_axi_arvalid(n_arvalid), .s_axi_arready(n_arready),
    .s_axi_rdata(n_rdata), .s_axi_rresp(n_rresp), .s_axi_rlast(n_rlast), .s_axi_rvalid(n_rvalid),
    .s_axi_rready(n_rready),
    .m_axi_aclk(ui_clk), .m_axi_aresetn(ui_aresetn),
    .m_axi_awaddr(c_awaddr), .m_axi_awlen(c_awlen), .m_axi_awsize(c_awsize), .m_axi_awburst(c_awburst),
    .m_axi_awlock(c_awlock), .m_axi_awcache(c_awcache), .m_axi_awprot(c_awprot), .m_axi_awregion(c_awregion),
    .m_axi_awqos(c_awqos), .m_axi_awvalid(c_awvalid), .m_axi_awready(c_awready),
    .m_axi_wdata(c_wdata), .m_axi_wstrb(c_wstrb), .m_axi_wlast(c_wlast), .m_axi_wvalid(c_wvalid),
    .m_axi_wready(c_wready), .m_axi_bresp(c_bresp), .m_axi_bvalid(c_bvalid), .m_axi_bready(c_bready),
    .m_axi_araddr(c_araddr), .m_axi_arlen(c_arlen), .m_axi_arsize(c_arsize), .m_axi_arburst(c_arburst),
    .m_axi_arlock(c_arlock), .m_axi_arcache(c_arcache), .m_axi_arprot(c_arprot), .m_axi_arregion(c_arregion),
    .m_axi_arqos(c_arqos), .m_axi_arvalid(c_arvalid), .m_axi_arready(c_arready),
    .m_axi_rdata(c_rdata), .m_axi_rresp(c_rresp), .m_axi_rlast(c_rlast), .m_axi_rvalid(c_rvalid),
    .m_axi_rready(c_rready)
  );

  // width converter output (512-bit) -> MIG
  logic [31:0]  d_awaddr, d_araddr;
  logic [7:0]   d_awlen, d_arlen;
  logic [2:0]   d_awsize, d_arsize, d_awprot, d_arprot;
  logic [1:0]   d_awburst, d_arburst, d_bresp, d_rresp;
  logic [0:0]   d_awlock, d_arlock;
  logic [3:0]   d_awcache, d_arcache, d_awregion, d_arregion, d_awqos, d_arqos;
  logic         d_awvalid, d_awready, d_wlast, d_wvalid, d_wready, d_bvalid, d_bready;
  logic         d_arvalid, d_arready, d_rlast, d_rvalid, d_rready;
  logic [511:0] d_wdata, d_rdata;
  logic [63:0]  d_wstrb;

  axi_dw u_axi_dw (
    .s_axi_aclk(ui_clk), .s_axi_aresetn(ui_aresetn),
    .s_axi_awaddr(c_awaddr), .s_axi_awlen(c_awlen), .s_axi_awsize(c_awsize), .s_axi_awburst(c_awburst),
    .s_axi_awlock(c_awlock), .s_axi_awcache(c_awcache), .s_axi_awprot(c_awprot), .s_axi_awregion(c_awregion),
    .s_axi_awqos(c_awqos), .s_axi_awvalid(c_awvalid), .s_axi_awready(c_awready),
    .s_axi_wdata(c_wdata), .s_axi_wstrb(c_wstrb), .s_axi_wlast(c_wlast), .s_axi_wvalid(c_wvalid),
    .s_axi_wready(c_wready), .s_axi_bresp(c_bresp), .s_axi_bvalid(c_bvalid), .s_axi_bready(c_bready),
    .s_axi_araddr(c_araddr), .s_axi_arlen(c_arlen), .s_axi_arsize(c_arsize), .s_axi_arburst(c_arburst),
    .s_axi_arlock(c_arlock), .s_axi_arcache(c_arcache), .s_axi_arprot(c_arprot), .s_axi_arregion(c_arregion),
    .s_axi_arqos(c_arqos), .s_axi_arvalid(c_arvalid), .s_axi_arready(c_arready),
    .s_axi_rdata(c_rdata), .s_axi_rresp(c_rresp), .s_axi_rlast(c_rlast), .s_axi_rvalid(c_rvalid),
    .s_axi_rready(c_rready),
    .m_axi_awaddr(d_awaddr), .m_axi_awlen(d_awlen), .m_axi_awsize(d_awsize), .m_axi_awburst(d_awburst),
    .m_axi_awlock(d_awlock), .m_axi_awcache(d_awcache), .m_axi_awprot(d_awprot), .m_axi_awregion(d_awregion),
    .m_axi_awqos(d_awqos), .m_axi_awvalid(d_awvalid), .m_axi_awready(d_awready),
    .m_axi_wdata(d_wdata), .m_axi_wstrb(d_wstrb), .m_axi_wlast(d_wlast), .m_axi_wvalid(d_wvalid),
    .m_axi_wready(d_wready), .m_axi_bresp(d_bresp), .m_axi_bvalid(d_bvalid), .m_axi_bready(d_bready),
    .m_axi_araddr(d_araddr), .m_axi_arlen(d_arlen), .m_axi_arsize(d_arsize), .m_axi_arburst(d_arburst),
    .m_axi_arlock(d_arlock), .m_axi_arcache(d_arcache), .m_axi_arprot(d_arprot), .m_axi_arregion(d_arregion),
    .m_axi_arqos(d_arqos), .m_axi_arvalid(d_arvalid), .m_axi_arready(d_arready),
    .m_axi_rdata(d_rdata), .m_axi_rresp(d_rresp), .m_axi_rlast(d_rlast), .m_axi_rvalid(d_rvalid),
    .m_axi_rready(d_rready)
  );

  ddr u_ddr (
    .ddr3_dq, .ddr3_dqs_n, .ddr3_dqs_p, .ddr3_addr, .ddr3_ba, .ddr3_ras_n, .ddr3_cas_n, .ddr3_we_n,
    .ddr3_reset_n, .ddr3_ck_p, .ddr3_ck_n, .ddr3_cke, .ddr3_cs_n, .ddr3_dm, .ddr3_odt,
    .sys_clk_p, .sys_clk_n,
    .ui_clk(ui_clk), .ui_clk_sync_rst(ui_rst), .ui_addn_clk_0(), .ui_addn_clk_1(), .ui_addn_clk_2(),
    .ui_addn_clk_3(), .ui_addn_clk_4(), .mmcm_locked(mmcm_locked), .aresetn(ui_aresetn),
    .app_sr_req(1'b0), .app_ref_req(1'b0), .app_zq_req(1'b0),
    .app_sr_active(), .app_ref_ack(), .app_zq_ack(),
    .s_axi_awid(2'b00), .s_axi_awaddr(d_awaddr), .s_axi_awlen(d_awlen), .s_axi_awsize(d_awsize),
    .s_axi_awburst(d_awburst), .s_axi_awlock(d_awlock), .s_axi_awcache(d_awcache), .s_axi_awprot(d_awprot),
    .s_axi_awqos(d_awqos), .s_axi_awvalid(d_awvalid), .s_axi_awready(d_awready),
    .s_axi_wdata(d_wdata), .s_axi_wstrb(d_wstrb), .s_axi_wlast(d_wlast), .s_axi_wvalid(d_wvalid),
    .s_axi_wready(d_wready), .s_axi_bready(d_bready), .s_axi_bid(), .s_axi_bresp(d_bresp),
    .s_axi_bvalid(d_bvalid),
    .s_axi_arid(2'b00), .s_axi_araddr(d_araddr), .s_axi_arlen(d_arlen), .s_axi_arsize(d_arsize),
    .s_axi_arburst(d_arburst), .s_axi_arlock(d_arlock), .s_axi_arcache(d_arcache), .s_axi_arprot(d_arprot),
    .s_axi_arqos(d_arqos), .s_axi_arvalid(d_arvalid), .s_axi_arready(d_arready),
    .s_axi_rready(d_rready), .s_axi_rid(), .s_axi_rdata(d_rdata), .s_axi_rresp(d_rresp),
    .s_axi_rlast(d_rlast), .s_axi_rvalid(d_rvalid),
    .init_calib_complete(calib_done), .device_temp(),
    .sys_rst(cpu_reset)
  );

  // ---------------- LEDs ----------------
  logic [26:0] beat;
  logic        err_seen;
  always_ff @(posedge user_clk) begin
    beat <= beat + 1'b1;
    if (user_reset) err_seen <= 1'b0;
    else if (dma_err) err_seen <= 1'b1;
  end
  assign led = {err_seen, beat[26], calib_done, user_lnk_up};

endmodule
