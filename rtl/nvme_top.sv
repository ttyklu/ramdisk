// nvme_top.sv - NVMe RAM disk controller top level
// Connects to:
//   * Xilinx 7 Series Integrated Block for PCI Express (pcie_7x), 128-bit AXI-Stream, Gen2 x4,
//     user_clk = 125 MHz. RX TLPs may straddle (is_sof / is_eof in rx_tuser). Configure the core with: BAR0 = 64-bit memory, 16 KB;
//     class code 0x010802; MSI-X table size NV, table BAR0 offset 0x2000, PBA BAR0 offset 0x3000.
//   * an AXI4 slave holding the RAM disk data (MIG DDR3 via an AXI interconnect / clock converter).
// Everything runs on user_clk.
// DDR map: namespace (NSZE x 4 KB) from DDR_BASE, then the compression staging ring (RING_MB).
module nvme_top #(
  parameter int          NQ            = 4,
  parameter int          NV            = NQ + 1,
  parameter int          RING_MB       = 64,                // staging ring
  parameter logic [63:0] NSZE          = 64'((1024 - RING_MB) * 256),   // 1 GB minus the ring, 4 KB blocks
  parameter logic [31:0] DDR_BASE      = 32'h0000_0000,
  parameter int          REC_LOG2      = 17,                // largest record: 17 = 128 KB ... 20 = 1 MB
  parameter bit          PAYLOAD_BSWAP = 1'b0
)(
  input  logic        clk,          // pcie_7x user_clk_out
  input  logic        rst,          // pcie_7x user_reset_out (active high)

  // pcie_7x RX (m_axis_rx_*)
  input  logic [127:0] rx_tdata,
  input  logic        rx_tvalid,
  output logic        rx_tready,
  input  logic [21:0] rx_tuser,     // [8:2] bar_hit, [14:10] is_sof, [21:17] is_eof

  // pcie_7x TX (s_axis_tx_*)
  output logic [127:0] tx_tdata,
  output logic [15:0] tx_tkeep,
  output logic        tx_tlast,
  output logic        tx_tvalid,
  input  logic        tx_tready,
  output logic [3:0]  tx_tuser,

  // pcie_7x configuration
  input  logic [7:0]  cfg_bus_number,
  input  logic [4:0]  cfg_device_number,
  input  logic [2:0]  cfg_function_number,
  input  logic        cfg_interrupt_msixenable,
  input  logic        cfg_interrupt_msixfm,

  // AXI4 master to DDR (64-bit)
  output logic [31:0] m_axi_awaddr,
  output logic [7:0]  m_axi_awlen,
  output logic [2:0]  m_axi_awsize,
  output logic [1:0]  m_axi_awburst,
  output logic        m_axi_awvalid,
  input  logic        m_axi_awready,
  output logic [63:0] m_axi_wdata,
  output logic [7:0]  m_axi_wstrb,
  output logic        m_axi_wlast,
  output logic        m_axi_wvalid,
  input  logic        m_axi_wready,
  input  logic [1:0]  m_axi_bresp,
  input  logic        m_axi_bvalid,
  output logic        m_axi_bready,
  output logic [31:0] m_axi_araddr,
  output logic [7:0]  m_axi_arlen,
  output logic [2:0]  m_axi_arsize,
  output logic [1:0]  m_axi_arburst,
  output logic        m_axi_arvalid,
  input  logic        m_axi_arready,
  input  logic [63:0] m_axi_rdata,
  input  logic [1:0]  m_axi_rresp,
  input  logic        m_axi_rlast,
  input  logic        m_axi_rvalid,
  output logic        m_axi_rready,

  // status
  output logic        dma_err
);

  localparam int RAW = 10;   // dma_rd ring: 1024 DWs
  localparam int FAW = 10;   // TX FIFO: 1024 DWs
  localparam int          NSLOT     = (RING_MB << 20) >> (REC_LOG2 + 2);   // slot = 4 x record
  localparam logic [31:0] RING_BASE = DDR_BASE + 32'(NSZE << 12);
  localparam int          MAX_BLKS  = (2 << REC_LOG2) / 4096;

  wire [15:0] completer_id = {cfg_bus_number, cfg_device_number, cfg_function_number};

  // ---------------- 128-bit core interface <-> 64-bit TLP beats ----------------
  logic [63:0] rx64_tdata, tx64_tdata;
  logic [7:0]  rx64_tkeep, tx64_tkeep;
  logic        rx64_tlast, rx64_tvalid, rx64_tready, tx64_tlast, tx64_tvalid, tx64_tready;
  logic [21:0] rx64_tuser;
  logic [3:0]  tx64_tuser;

  rx128_to_64 u_rx128 (
    .clk, .rst,
    .s_tdata(rx_tdata), .s_tvalid(rx_tvalid), .s_tready(rx_tready), .s_tuser(rx_tuser),
    .m_tdata(rx64_tdata), .m_tkeep(rx64_tkeep), .m_tlast(rx64_tlast), .m_tvalid(rx64_tvalid),
    .m_tready(rx64_tready), .m_tuser(rx64_tuser)
  );
  tx64_to_128 u_tx128 (
    .clk, .rst,
    .s_tdata(tx64_tdata), .s_tkeep(tx64_tkeep), .s_tlast(tx64_tlast), .s_tvalid(tx64_tvalid),
    .s_tready(tx64_tready), .s_tuser(tx64_tuser),
    .m_tdata(tx_tdata), .m_tkeep(tx_tkeep), .m_tlast(tx_tlast), .m_tvalid(tx_tvalid),
    .m_tready(tx_tready), .m_tuser(tx_tuser)
  );

  // ---------------- tlp_rx ----------------
  logic        creq_valid, creq_ready, creq_wr;
  logic [11:0] creq_dwaddr;
  logic [10:0] creq_len;
  logic [3:0]  creq_fbe, creq_lbe;
  logic [31:0] creq_d0, creq_d1;
  logic [15:0] creq_rid;
  logic [7:0]  creq_tag;
  logic [2:0]  creq_tc;
  logic [1:0]  creq_attr;
  logic        cpl_hdr_valid, cpl_hasdata, cpl_data_valid;
  logic [7:0]  cpl_tag;
  logic [6:0]  cpl_la;
  logic [2:0]  cpl_status;
  logic [10:0] cpl_len;
  logic [1:0]  cpl_data_cnt;
  logic [31:0] cpl_dw0, cpl_dw1;

  tlp_rx #(.PAYLOAD_BSWAP(PAYLOAD_BSWAP)) u_rx (
    .clk, .rst,
    .rx_tdata(rx64_tdata), .rx_tkeep(rx64_tkeep), .rx_tlast(rx64_tlast), .rx_tvalid(rx64_tvalid),
    .rx_tready(rx64_tready), .rx_tuser(rx64_tuser),
    .creq_valid, .creq_ready, .creq_wr, .creq_dwaddr, .creq_len, .creq_fbe, .creq_lbe,
    .creq_d0, .creq_d1, .creq_rid, .creq_tag, .creq_tc, .creq_attr,
    .cpl_hdr_valid, .cpl_tag, .cpl_la, .cpl_status, .cpl_len, .cpl_hasdata,
    .cpl_data_valid, .cpl_data_cnt, .cpl_dw0, .cpl_dw1
  );

  // ---------------- tlp_tx ----------------
  logic        c_valid, c_ready;
  logic [15:0] c_rid;
  logic [7:0]  c_tag;
  logic [2:0]  c_tc, c_status;
  logic [1:0]  c_attr, c_len;
  logic [6:0]  c_la;
  logic [11:0] c_bc;
  logic [31:0] c_d0, c_d1;
  logic        r_valid, r_ready;
  logic [63:0] r_addr;
  logic [10:0] r_len;
  logic [7:0]  r_tag;
  logic        w_valid, w_ready, w_done;
  logic [63:0] w_addr;
  logic [10:0] w_len;
  logic [1:0]  w_pop;
  logic [31:0] txf_head0, txf_head1;

  tlp_tx #(.PAYLOAD_BSWAP(PAYLOAD_BSWAP)) u_tx (
    .clk, .rst, .completer_id,
    .tx_tdata(tx64_tdata), .tx_tkeep(tx64_tkeep), .tx_tlast(tx64_tlast), .tx_tvalid(tx64_tvalid),
    .tx_tready(tx64_tready), .tx_tuser(tx64_tuser),
    .c_valid, .c_ready, .c_rid, .c_tag, .c_tc, .c_attr, .c_status, .c_la, .c_bc, .c_len, .c_d0, .c_d1,
    .r_valid, .r_ready, .r_addr, .r_len, .r_tag,
    .w_valid, .w_ready, .w_addr, .w_len, .w_done, .w_pop, .w_dw0(txf_head0), .w_dw1(txf_head1)
  );

  // ---------------- registers ----------------
  logic [31:0] cc, aqa;
  logic [63:0] asq, acq;
  logic        csts_rdy;
  logic [(NQ+1)*16-1:0] sq_tail_db, cq_head_db;
  logic        db_clr, db_clr_sq;
  logic [3:0]  db_clr_q;
  logic [NV*64-1:0] msix_addr;
  logic [NV*32-1:0] msix_data;
  logic [NV-1:0]    msix_mask, msix_pba;

  nvme_regs #(.NQ(NQ), .NV(NV)) u_regs (
    .clk, .rst,
    .creq_valid, .creq_ready, .creq_wr, .creq_dwaddr, .creq_len, .creq_fbe, .creq_lbe,
    .creq_d0, .creq_d1, .creq_rid, .creq_tag, .creq_tc, .creq_attr,
    .c_valid, .c_ready, .c_rid, .c_tag, .c_tc, .c_attr, .c_status, .c_la, .c_bc, .c_len, .c_d0, .c_d1,
    .cc, .aqa, .asq, .acq, .csts_rdy, .sq_tail_db, .cq_head_db,
    .db_clr, .db_clr_sq, .db_clr_q,
    .msix_addr, .msix_data, .msix_mask, .msix_pba
  );

  // ---------------- dma_rd ----------------
  logic         drd_job_valid, drd_job_ready, drd_idle;
  logic [63:0]  drd_job_addr;
  logic [11:0]  drd_job_len;
  logic [RAW:0] drd_avail;
  logic [31:0]  drd_head0, drd_head1;
  logic [1:0]   drd_pop, ctrl_drd_pop, ddrw_pop;
  logic         data_to_ddr;

  assign drd_pop = data_to_ddr ? ddrw_pop : ctrl_drd_pop;

  dma_rd #(.NT(16), .RAW(RAW)) u_drd (
    .clk, .rst,
    .job_valid(drd_job_valid), .job_ready(drd_job_ready), .job_addr(drd_job_addr), .job_len(drd_job_len),
    .r_valid, .r_ready, .r_addr, .r_len, .r_tag,
    .cpl_hdr_valid, .cpl_tag, .cpl_la, .cpl_status, .cpl_len, .cpl_hasdata,
    .cpl_data_valid, .cpl_data_cnt, .cpl_dw0, .cpl_dw1,
    .avail(drd_avail), .head0(drd_head0), .head1(drd_head1), .pop(drd_pop),
    .idle(drd_idle), .err(dma_err)
  );

  // ---------------- TX FIFO ----------------
  logic [1:0]   txf_push, ctrl_push, ddrr_push;
  logic [31:0]  txf_dw0, txf_dw1, ctrl_dw0, ctrl_dw1, ddrr_dw0, ddrr_dw1;
  logic [FAW:0] txf_count, txf_free;
  logic         ddrr_busy;
  logic         txf_flush;

  // xform (owns ddr_rd while busy) > ddr_rd (plain reads) > controller
  logic         xf_busy;
  logic [1:0]   xf_tx_push;
  logic [31:0]  xf_tx_dw0, xf_tx_dw1;
  assign txf_push = xf_busy ? xf_tx_push : ddrr_busy ? ddrr_push : ctrl_push;
  assign txf_dw0  = xf_busy ? xf_tx_dw0  : ddrr_busy ? ddrr_dw0  : ctrl_dw0;
  assign txf_dw1  = xf_busy ? xf_tx_dw1  : ddrr_busy ? ddrr_dw1  : ctrl_dw1;

  dw_fifo #(.AW(FAW)) u_txf (
    .clk, .rst, .flush(txf_flush),
    .push_cnt(txf_push), .push_dw0(txf_dw0), .push_dw1(txf_dw1),
    .pop_cnt(w_pop), .head0(txf_head0), .head1(txf_head1),
    .count(txf_count), .free(txf_free)
  );

  // ---------------- mwr_eng ----------------
  logic        mwr_job_valid, mwr_job_ready, mwr_idle;
  logic [63:0] mwr_job_addr;
  logic [11:0] mwr_job_len;

  mwr_eng #(.FAW(FAW)) u_mwr (
    .clk, .rst,
    .job_valid(mwr_job_valid), .job_ready(mwr_job_ready), .job_addr(mwr_job_addr), .job_len(mwr_job_len),
    .fifo_count(txf_count),
    .w_valid, .w_ready, .w_addr, .w_len, .w_done,
    .idle(mwr_idle)
  );

  // ---------------- DDR engines ----------------
  logic        ddrr_job_valid, ddrr_job_ready, ddrw_job_valid, ddrw_job_ready, ddrw_busy;
  logic [31:0] ddr_job_addr;
  logic [23:0] ddr_job_len;
  // xform side
  logic         xf_rdj_valid, xf_wrj_valid;
  logic [31:0]  xf_rdj_addr, xf_wrj_addr;
  logic [23:0]  xf_rdj_len, xf_wrj_len;
  logic [FAW:0] xf_rd_free, xf_wr_avail;
  logic [31:0]  xf_wr_dw0, xf_wr_dw1;
  logic [1:0]   ddrw_pop_all;

  ddr_rd #(.FAW(FAW)) u_ddrr (
    .clk, .rst,
    .job_valid(xf_busy ? xf_rdj_valid : ddrr_job_valid), .job_ready(ddrr_job_ready),
    .job_addr(xf_busy ? xf_rdj_addr : ddr_job_addr), .job_len(xf_busy ? xf_rdj_len : ddr_job_len),
    .fifo_free(xf_busy ? xf_rd_free : txf_free),
    .araddr(m_axi_araddr), .arlen(m_axi_arlen), .arsize(m_axi_arsize), .arburst(m_axi_arburst),
    .arvalid(m_axi_arvalid), .arready(m_axi_arready),
    .rdata(m_axi_rdata), .rlast(m_axi_rlast), .rvalid(m_axi_rvalid), .rready(m_axi_rready),
    .push_cnt(ddrr_push), .push_dw0(ddrr_dw0), .push_dw1(ddrr_dw1),
    .busy(ddrr_busy)
  );

  ddr_wr u_ddrw (
    .clk, .rst,
    .job_valid(xf_busy ? xf_wrj_valid : ddrw_job_valid), .job_ready(ddrw_job_ready),
    .job_addr(xf_busy ? xf_wrj_addr : ddr_job_addr), .job_len(xf_busy ? xf_wrj_len : ddr_job_len),
    .src_avail(xf_busy ? xf_wr_avail : drd_avail), .src_dw0(xf_busy ? xf_wr_dw0 : drd_head0),
    .src_dw1(xf_busy ? xf_wr_dw1 : drd_head1), .src_pop(ddrw_pop_all),
    .awaddr(m_axi_awaddr), .awlen(m_axi_awlen), .awsize(m_axi_awsize), .awburst(m_axi_awburst),
    .awvalid(m_axi_awvalid), .awready(m_axi_awready),
    .wdata(m_axi_wdata), .wstrb(m_axi_wstrb), .wlast(m_axi_wlast), .wvalid(m_axi_wvalid), .wready(m_axi_wready),
    .bresp(m_axi_bresp), .bvalid(m_axi_bvalid), .bready(m_axi_bready),
    .busy(ddrw_busy)
  );

  assign ddrw_pop = xf_busy ? 2'd0 : ddrw_pop_all;

  // ---------------- compression transform engine ----------------
  logic         xf_start, xf_dst_tx, xf_done, xf_over;
  logic [1:0]   xf_func;
  logic [31:0]  xf_rd_addr, xf_wr_addr, xf_adler;
  logic [23:0]  xf_rd_len, xf_in_len, xf_ck_len, xf_out_lim, xf_out_len, xf_wr_bytes, xf_consumed;
  logic [4:0]   xf_skip;
  logic [3:0]   xf_ierr;
  logic [63:0]  xf_fia, xf_fib, xf_fic, xf_fid, xf_foa, xf_fob, xf_foc, xf_fod;

  xform #(.FAW(FAW)) u_xf (
    .clk, .rst,
    .start(xf_start), .func(xf_func), .rd_addr(xf_rd_addr), .rd_len(xf_rd_len), .skip_dw(xf_skip),
    .in_len(xf_in_len), .ck_len(xf_ck_len), .dst_tx(xf_dst_tx), .wr_addr(xf_wr_addr), .out_lim(xf_out_lim),
    .busy(xf_busy), .done(xf_done), .out_len(xf_out_len), .wr_bytes(xf_wr_bytes), .over(xf_over),
    .fia(xf_fia), .fib(xf_fib), .fic(xf_fic), .fid(xf_fid),
    .foa(xf_foa), .fob(xf_fob), .foc(xf_foc), .fod(xf_fod),
    .adler(xf_adler), .ierr(xf_ierr), .consumed(xf_consumed),
    .rdj_valid(xf_rdj_valid), .rdj_ready(ddrr_job_ready), .rdj_addr(xf_rdj_addr), .rdj_len(xf_rdj_len),
    .rd_busy(ddrr_busy), .rd_push(xf_busy ? ddrr_push : 2'd0), .rd_dw0(ddrr_dw0), .rd_dw1(ddrr_dw1),
    .rd_free(xf_rd_free),
    .wrj_valid(xf_wrj_valid), .wrj_ready(ddrw_job_ready), .wrj_addr(xf_wrj_addr), .wrj_len(xf_wrj_len),
    .wr_busy(ddrw_busy), .wr_avail(xf_wr_avail), .wr_dw0(xf_wr_dw0), .wr_dw1(xf_wr_dw1),
    .wr_pop(xf_busy ? ddrw_pop_all : 2'd0),
    .tx_push(xf_tx_push), .tx_dw0(xf_tx_dw0), .tx_dw1(xf_tx_dw1), .tx_free(txf_free)
  );

  // ---------------- command engine ----------------
  nvme_ctrl #(.NQ(NQ), .NV(NV), .NSZE(NSZE), .DDR_BASE(DDR_BASE), .RAW(RAW), .FAW(FAW),
              .REC_LOG2(REC_LOG2), .MAX_BLKS(MAX_BLKS), .NSLOT(NSLOT), .RING_BASE(RING_BASE)) u_ctrl (
    .clk, .rst,
    .cc, .aqa, .asq, .acq, .csts_rdy, .sq_tail_db, .cq_head_db,
    .db_clr, .db_clr_sq, .db_clr_q,
    .msix_addr, .msix_data, .msix_mask,
    .msix_en(cfg_interrupt_msixenable), .msix_fm(cfg_interrupt_msixfm), .msix_pba,
    .drd_job_valid, .drd_job_ready, .drd_job_addr, .drd_job_len,
    .drd_avail, .drd_head0, .drd_head1, .drd_pop(ctrl_drd_pop), .drd_idle,
    .mwr_job_valid, .mwr_job_ready, .mwr_job_addr, .mwr_job_len, .mwr_idle,
    .txf_push(ctrl_push), .txf_dw0(ctrl_dw0), .txf_dw1(ctrl_dw1), .txf_free,
    .ddrr_job_valid, .ddrr_job_ready, .ddrr_busy,
    .ddrw_job_valid, .ddrw_job_ready, .ddrw_busy,
    .ddr_job_addr, .ddr_job_len, .data_to_ddr, .txf_flush,
    .xf_start, .xf_func, .xf_rd_addr, .xf_rd_len, .xf_skip, .xf_in_len, .xf_ck_len, .xf_dst_tx,
    .xf_wr_addr, .xf_out_lim, .xf_busy, .xf_done, .xf_out_len, .xf_wr_bytes, .xf_over,
    .xf_fi({xf_fid, xf_fic, xf_fib, xf_fia}), .xf_fo({xf_fod, xf_foc, xf_fob, xf_foa}),
    .xf_adler, .xf_ierr, .xf_consumed
  );

endmodule
