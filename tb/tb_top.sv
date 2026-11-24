// tb_top.sv - self-checking testbench for nvme_top
//  * Root-complex model on the pcie_7x AXI-Stream interface:
//      - host memory (sparse), MMIO reads/writes to BAR0
//      - services device MRd with CplDs split at 64 B RCB boundaries, interleaved across
//        tags in random order (per-tag order preserved), random gaps and TX backpressure
//      - captures device MWr into host memory, MSI-X writes counted per vector
//  * AXI4 DDR model with random stalls
//  * NVMe host driver: Linux-like bring-up, Identify, queues, random Read/Write with
//    PRP1 offsets, PRP2 and (chained) PRP lists, data checked against a disk model
`timescale 1ns/1ps
module tb_top #(
  parameter int REC_LOG2 = 17          // largest compression record: 17 = 128 KB, 20 = 1 MB
);
  import nvme_pkg::*;

  // ---------------- configuration ----------------
  localparam bit          BSWAP    = 1'b1;   // match pcie_7x AXIS payload byte order (as on the board)
  localparam int          NQ       = 4;
  localparam int          NV       = NQ + 1;
  localparam logic [63:0] NSZE     = 64'd245760;   // 1 GB minus the 64 MB staging ring
  localparam bit [31:0]   BAR0     = 32'hF000_0000;
  localparam bit [63:0]   MSI_ADDR = 64'h0000_0000_FEE0_0000;
  localparam int          N_IO     = 80;
  localparam int          CPL_TO   = 20000;  // DMA read completion timeout in the DUT (cycles)

  // ---------------- clock / reset ----------------
  logic clk = 1'b0;
  always #4 clk = ~clk;               // 125 MHz (PCIe user_clk, 128-bit interface)
  logic rst;

  // ---------------- DUT ----------------
  logic [127:0] rx_tdata; logic rx_tvalid, rx_tready; logic [21:0] rx_tuser;
  logic [127:0] tx_tdata; logic [15:0] tx_tkeep; logic tx_tlast, tx_tvalid, tx_tready; logic [3:0] tx_tuser;
  logic [31:0] awaddr, araddr; logic [7:0] awlen, arlen; logic [2:0] awsize, arsize; logic [1:0] awburst, arburst;
  logic awvalid, awready, wlast, wvalid, wready, bvalid, bready, arvalid, arready, rlast, rvalid, rready;
  logic [63:0] wdata, rdata; logic [7:0] wstrb; logic [1:0] bresp, rresp;
  logic dma_err;

  nvme_top #(.NQ(NQ), .NV(NV), .NSZE(NSZE), .REC_LOG2(REC_LOG2), .PAYLOAD_BSWAP(BSWAP), .CPL_TO(CPL_TO)) dut (
    .clk, .rst,
    .rx_tdata, .rx_tvalid, .rx_tready, .rx_tuser,
    .tx_tdata, .tx_tkeep, .tx_tlast, .tx_tvalid, .tx_tready, .tx_tuser,
    .cfg_bus_number(8'h01), .cfg_device_number(5'd1), .cfg_function_number(3'd0),
    .cfg_interrupt_msixenable(1'b1), .cfg_interrupt_msixfm(1'b0),
    .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize), .m_axi_awburst(awburst),
    .m_axi_awvalid(awvalid), .m_axi_awready(awready),
    .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
    .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
    .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize), .m_axi_arburst(arburst),
    .m_axi_arvalid(arvalid), .m_axi_arready(arready),
    .m_axi_rdata(rdata), .m_axi_rresp(rresp), .m_axi_rlast(rlast), .m_axi_rvalid(rvalid), .m_axi_rready(rready),
    .dma_err
  );

  axi_mem_model u_ddr (
    .clk, .rst,
    .awaddr, .awlen, .awsize, .awburst, .awvalid, .awready,
    .wdata, .wstrb, .wlast, .wvalid, .wready, .bresp, .bvalid, .bready,
    .araddr, .arlen, .arsize, .arburst, .arvalid, .arready,
    .rdata, .rresp, .rlast, .rvalid, .rready
  );

  // ---------------- error bookkeeping ----------------
  int errors = 0;
  function automatic void fail(input string msg);
    errors++;
    $display("[%0t] ERROR: %s", $time, msg);
    if (errors > 20) begin
      $display("TEST FAILED (too many errors)");
      $finish;
    end
  endfunction

  // ---------------- host memory (sparse, DW granular; unwritten reads as 0) ----------------
  bit [31:0] hmem [bit [63:0]];    // key = byte address >> 2

  function automatic bit [31:0] hr32a(input bit [63:0] a);      // a is DW aligned
    bit [63:0] k;
    k = a >> 2;
    return hmem.exists(k) ? hmem[k] : 32'd0;
  endfunction
  function automatic bit [7:0] hr8(input bit [63:0] a);
    bit [31:0] w;
    w = hr32a({a[63:2], 2'b00});
    return w[a[1:0]*8 +: 8];
  endfunction
  function automatic void hw8(input bit [63:0] a, input bit [7:0] v);
    bit [31:0] w;
    w = hr32a({a[63:2], 2'b00});
    w[a[1:0]*8 +: 8] = v;
    hmem[a >> 2] = w;
  endfunction
  function automatic bit [31:0] hr32(input bit [63:0] a);
    if (a[1:0] == 2'b00) return hr32a(a);
    return {hr8(a + 3), hr8(a + 2), hr8(a + 1), hr8(a)};
  endfunction
  function automatic void hw32(input bit [63:0] a, input bit [31:0] v);
    if (a[1:0] == 2'b00) hmem[a >> 2] = v;
    else for (int i = 0; i < 4; i++) hw8(a + i, v[i*8 +: 8]);
  endfunction
  function automatic bit [63:0] hr64(input bit [63:0] a);
    return {hr32(a + 4), hr32(a)};
  endfunction
  function automatic void hw64(input bit [63:0] a, input bit [63:0] v);
    hw32(a, v[31:0]);
    hw32(a + 4, v[63:32]);
  endfunction

  // payload DW <-> AXI-Stream representation
  function automatic bit [31:0] ax(input bit [31:0] d);
    return BSWAP ? bswap32(d) : d;
  endfunction

  // ---------------- RX driver (host -> device, 7-series 128-bit format) ----------------
  // TLPs are queued as DWs; each beat carries up to 4 DWs. A TLP may start at DW0 or DW2;
  // after a TLP ends in DW0/DW1 the next one sometimes starts at DW2 of the same beat
  // (straddle). rx_tuser: [8:2] bar_hit, [14:10] is_sof, [21:17] is_eof.
  bit [31:0] rxq_dw[$];
  bit        rxq_first[$], rxq_last[$];
  bit [6:0]  rxq_bar[$];
  int        rxq_tlps = 0;              // complete TLPs queued (for back-pressure in the TB)
  bit        rx_open  = 0;              // a TLP continues into the next beat
  int        n_straddle = 0, n_sof2 = 0;  // beats where a TLP ends and the next starts; starts at DW2

  function automatic void push_tlp(input bit [31:0] dws[$], input bit [21:0] u);
    foreach (dws[i]) begin
      rxq_dw.push_back(dws[i]);
      rxq_first.push_back(i == 0);
      rxq_last.push_back(i == dws.size() - 1);
      rxq_bar.push_back(u[8:2]);
    end
    rxq_tlps++;
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      rx_tvalid <= 1'b0;
      rx_open   =  0;
    end else if (!rx_tvalid || rx_tready) begin
      if (rxq_dw.size() > 0 && $urandom_range(0, 7) != 0) begin
        bit [127:0] d;
        bit [21:0]  u;
        int         k;
        d = '0; u = '0; k = 0;
        // a new TLP may start at DW2 (rarely without a straddle)
        if (!rx_open && $urandom_range(0, 15) == 0) k = 2;
        while (k < 4 && rxq_dw.size() > 0) begin
          bit f, l;
          f = rxq_first[0]; l = rxq_last[0];
          if (f && rx_open) break;                         // never happens: open TLP first
          if (f && k == 1) k = 2;                          // a TLP starts at DW0 or DW2 only
          if (f && k == 3) break;
          if (f && u[14]) break;                           // one start per beat
          if (f) begin
            u[14] = 1'b1; u[13:10] = 4'(k * 4); u[8:2] = rxq_bar[0];
            rx_open = 1;
          end
          d[32*k +: 32] = rxq_dw.pop_front();
          void'(rxq_first.pop_front()); void'(rxq_last.pop_front()); void'(rxq_bar.pop_front());
          if (l) begin
            u[21] = 1'b1; u[20:17] = 4'(k * 4 + 3);
            rx_open = 0;
            rxq_tlps--;
            k++;
            if (k <= 2 && $urandom_range(0, 1) == 0) break;  // sometimes no straddle
            continue;
          end
          k++;
        end
        if (u[14] && u[13:10] == 4'd8) n_sof2++;
        if (u[14] && u[21] && u[13:10] > u[20:17]) n_straddle++;
        rx_tdata  <= d;
        rx_tuser  <= u;
        rx_tvalid <= 1'b1;
      end else begin
        rx_tvalid <= 1'b0;
      end
    end
  end

  // ---------------- MMIO ----------------
  localparam bit [21:0] BAR_HIT0 = 22'(1 << 2);
  bit [31:0] mmio_rdata [256];
  bit        mmio_done  [256];
  bit [7:0]  mmio_tag = 0;

  task automatic mmio_write32(input bit [31:0] off, input bit [31:0] data);
    bit [31:0] t[$];
    t.delete();
    t.push_back(mk_h0(FMT_3DW_D, TLP_T_MEM, 3'd0, 2'd0, 10'd1));
    t.push_back({16'h0000, 8'h00, 4'h0, 4'hF});
    t.push_back(BAR0 + off);
    t.push_back(ax(data));
    push_tlp(t, BAR_HIT0);
    repeat (2) @(posedge clk);
  endtask

  task automatic mmio_write64(input bit [31:0] off, input bit [63:0] data);
    bit [31:0] t[$];
    t.delete();
    t.push_back(mk_h0(FMT_3DW_D, TLP_T_MEM, 3'd0, 2'd0, 10'd2));
    t.push_back({16'h0000, 8'h00, 4'hF, 4'hF});
    t.push_back(BAR0 + off);
    t.push_back(ax(data[31:0]));
    t.push_back(ax(data[63:32]));
    push_tlp(t, BAR_HIT0);
    repeat (2) @(posedge clk);
  endtask

  task automatic mmio_read32(input bit [31:0] off, output bit [31:0] data);
    bit [31:0] t[$];
    bit [7:0]  tag;
    int        to;
    t.delete();
    tag = mmio_tag++;
    mmio_done[tag] = 1'b0;
    t.push_back(mk_h0(FMT_3DW_ND, TLP_T_MEM, 3'd0, 2'd0, 10'd1));
    t.push_back({16'h0000, tag, 4'h0, 4'hF});
    t.push_back(BAR0 + off);
    push_tlp(t, BAR_HIT0);
    to = 0;
    while (!mmio_done[tag]) begin
      @(posedge clk);
      to++;
      if (to > 10000) begin fail($sformatf("MMIO read timeout off=%h", off)); break; end
    end
    data = mmio_rdata[tag];
  endtask

  // ---------------- TX monitor (device -> host) ----------------
  bit [31:0] txb[$];          // DWs of the TLP being received
  int        msi_cnt [NV];
  int        n_mwr = 0, n_mrd = 0;

  // pending device reads: one entry per MRd, served chunk by chunk
  bit [7:0]  pend_tag[$];
  bit [63:0] pend_addr[$];
  int        pend_rem[$];      // bytes
  bit [15:0] pend_rid[$];
  bit        tag_busy [256];
  bit        drop_mrd = 1'b0;  // host stops answering device reads (lost completions)

  function automatic void handle_tx_tlp();
    bit [31:0] dw[$];
    bit [31:0] h0, h1;
    bit [2:0]  fmt;
    bit [4:0]  typ;
    int        len, hdr;
    bit [63:0] addr;
    dw.delete();
    foreach (txb[i]) dw.push_back(txb[i]);
    h0  = dw[0];
    h1  = dw[1];
    fmt = h0[31:29];
    typ = h0[28:24];
    len = (h0[9:0] == 0) ? 1024 : int'(h0[9:0]);
    hdr = fmt[0] ? 4 : 3;
    if (typ == TLP_T_MEM) begin
      addr = fmt[0] ? {dw[2], dw[3][31:2], 2'b00} : {32'd0, dw[2][31:2], 2'b00};
      if (fmt[0] && dw[2] == 0) fail("4DW header used for an address below 4 GB");
      if (fmt[1]) begin
        // MWr
        n_mwr++;
        if (dw.size() < hdr + len) begin fail("MWr shorter than its length"); return; end
        if ((addr & 64'hFFF) + len * 4 > 4096) fail($sformatf("MWr crosses 4KB: addr=%h len=%0d", addr, len));
        if (len > 32) fail($sformatf("MWr payload %0d DW > 128 B", len));
        if (addr >= MSI_ADDR && addr < MSI_ADDR + 64'h1000) begin
          bit [31:0] d;
          d = ax(dw[hdr]);
          if (d < NV) msi_cnt[d]++;
          else fail($sformatf("MSI-X data %h out of range", d));
        end else begin
          for (int i = 0; i < len; i++) begin
            bit [31:0] d;
            bit [3:0]  be;
            d  = ax(dw[hdr + i]);
            be = (i == 0) ? h1[3:0] : (i == len - 1) ? h1[7:4] : 4'hF;
            if (be == 4'hF) hw32(addr + i*4, d);
            else for (int b = 0; b < 4; b++) if (be[b]) hw8(addr + i*4 + b, d[b*8 +: 8]);
          end
        end
      end else begin
        // MRd
        bit [7:0] tag;
        tag = h1[15:8];
        n_mrd++;
        if (drop_mrd) return;
        if (len > 32) fail($sformatf("MRd length %0d > 32 DW", len));
        if ((addr & 64'h7F) + len * 4 > 128) fail($sformatf("MRd crosses 128 B: addr=%h len=%0d", addr, len));
        if (tag_busy[tag]) fail($sformatf("MRd tag %0d reused while outstanding", tag));
        tag_busy[tag] = 1'b1;
        pend_tag.push_back(tag);
        pend_addr.push_back(addr);
        pend_rem.push_back(len * 4);
        pend_rid.push_back(h1[31:16]);
      end
    end else if (typ == TLP_T_CPL) begin
      bit [7:0] tag;
      tag = dw[2][15:8];
      if (h1[15:13] != 3'b000) fail($sformatf("register read completion status %b", h1[15:13]));
      mmio_rdata[tag] = (fmt[1]) ? ax(dw[3]) : 32'hFFFF_FFFF;
      mmio_done[tag]  = 1'b1;
    end else begin
      fail($sformatf("unexpected TLP fmt=%b type=%b", fmt, typ));
    end
  endfunction

  always @(posedge clk) begin
    if (rst) begin
      tx_tready <= 1'b0;
    end else begin
      if (tx_tvalid && tx_tready) begin
        for (int k = 0; k < 4; k++) if (tx_tkeep[4 * k]) txb.push_back(tx_tdata[32*k +: 32]);
        if (tx_tlast) begin
          handle_tx_tlp();
          txb.delete();
        end
      end
      tx_tready <= ($urandom_range(0, 9) != 0);
    end
  end

  // completion scheduler: random pending read, next chunk (RCB = 64 B), CplD to device
  always @(posedge clk) begin
    if (!rst && pend_tag.size() > 0 && rxq_tlps < 16 && $urandom_range(0, 2) == 0) begin
      int        i, rem, bnd, nb;
      bit [63:0] a;
      bit [31:0] t[$];
      t.delete();
      i   = $urandom_range(0, pend_tag.size() - 1);
      a   = pend_addr[i];
      rem = pend_rem[i];
      bnd = 64 - int'(a % 64);
      nb  = (rem < bnd || $urandom_range(0, 3) == 0) ? rem : bnd;  // sometimes whole
      t.push_back(mk_h0(FMT_3DW_D, TLP_T_CPL, 3'd0, 2'd0, 10'(nb / 4)));
      t.push_back({16'h0000, 3'b000, 1'b0, 12'(rem)});
      t.push_back({pend_rid[i], pend_tag[i], 1'b0, a[6:0]});
      for (int k = 0; k < nb / 4; k++) t.push_back(ax(hr32(a + k * 4)));
      push_tlp(t, 22'd0);
      if (nb == rem) begin
        tag_busy[pend_tag[i]] = 1'b0;
        pend_tag.delete(i); pend_addr.delete(i); pend_rem.delete(i); pend_rid.delete(i);
      end else begin
        pend_addr[i] = a + nb;
        pend_rem[i]  = rem - nb;
      end
    end
  end

  // ---------------- NVMe host driver ----------------
  bit [63:0] sq_base[NQ+1], cq_base[NQ+1];
  int        sq_size[NQ+1], cq_size[NQ+1], sq_tail[NQ+1], cq_head[NQ+1];
  bit        cq_ph[NQ+1];
  int        cq_vec[NQ+1];
  bit [15:0] next_cid = 0;

  typedef bit [31:0] sqe_t [16];

  function automatic sqe_t new_cmd(input bit [7:0] opc, input bit [31:0] nsid);
    sqe_t c;
    foreach (c[i]) c[i] = 32'd0;
    c[0] = {16'd0, 8'd0, opc};
    c[1] = nsid;
    return c;
  endfunction

  task automatic submit(input int q, input sqe_t c, output bit [15:0] cid);
    bit [63:0] a;
    cid  = next_cid++;
    c[0][31:16] = cid;
    a = sq_base[q] + 64'(sq_tail[q] * 64);
    for (int i = 0; i < 16; i++) hw32(a + i * 4, c[i]);
    sq_tail[q] = (sq_tail[q] + 1) % sq_size[q];
    mmio_write32(32'h1000 + 32'(8 * q), 32'(sq_tail[q]));
  endtask

  task automatic wait_cqe(input int q, input bit [15:0] cid, output bit [31:0] dw0, output bit [14:0] st);
    bit [63:0] a;
    bit [31:0] d2, d3;
    int        to;
    to = 0;
    a  = cq_base[q] + 64'(cq_head[q] * 16);
    d3 = hr32(a + 12);
    while (d3[16] != cq_ph[q]) begin
      repeat (8) @(posedge clk);
      to += 8;
      if (to > 40_000_000) begin fail($sformatf("CQE timeout q=%0d cid=%0d", q, cid)); st = '1; return; end
      d3 = hr32(a + 12);
    end
    dw0 = hr32(a);
    d2  = hr32(a + 8);
    st  = d3[31:17];
    if (d3[15:0] != cid) fail($sformatf("CQE cid %0d, expected %0d", d3[15:0], cid));
    if (d2[31:16] != 16'(q)) fail($sformatf("CQE sqid %0d, expected %0d", d2[31:16], q));
    cq_head[q] = (cq_head[q] + 1) % cq_size[q];
    if (cq_head[q] == 0) cq_ph[q] = ~cq_ph[q];
    mmio_write32(32'h1004 + 32'(8 * q), 32'(cq_head[q]));
  endtask

  // submit on SQ q whose CQ is cq_of_sq[q]
  int cq_of_sq[NQ+1];
  task automatic run_cmd(input int q, input sqe_t c, output bit [31:0] dw0, output bit [14:0] st);
    bit [15:0] cid;
    int        v, msi_before;
    v      = cq_vec[cq_of_sq[q]];
    msi_before = msi_cnt[v];
    submit(q, c, cid);
    wait_cqe(cq_of_sq[q], cid, dw0, st);
    repeat (100) @(posedge clk);
    if (msi_cnt[v] == msi_before) fail($sformatf("no MSI-X on vector %0d for q=%0d", v, q));
  endtask

  // ---------------- PRP construction ----------------
  bit [63:0] pages[$];
  bit [63:0] list_pool = 64'h0000_0000_4000_0000;

  // Lay out a transfer of nbytes starting at page offset off1; returns PRP1/PRP2
  task automatic build_prp(input bit [63:0] region, input int nbytes, input int off1,
                           output bit [63:0] prp1, output bit [63:0] prp2);
    int n;
    n = (off1 + nbytes + 4095) / 4096;
    pages.delete();
    for (int i = 0; i < n; i++) pages.push_back(region + 64'(((i * 37 + 5) % 1024) * 4096));
    prp1 = pages[0] + off1;
    prp2 = 64'd0;
    if (n == 2) prp2 = pages[1];
    else if (n > 2) begin
      bit [63:0] la;
      int        left;
      int        k;
      left = n - 1;
      k    = 1;
      la   = list_pool + 64'(4096 - 8 * $urandom_range(1, 40));   // often forces chaining
      list_pool += 64'h1_0000;
      prp2 = la;
      while (left > 0) begin
        if ((la % 4096) == 4088 && left > 1) begin
          bit [63:0] nxt;
          nxt = list_pool;
          list_pool += 64'h1_0000;
          hw64(la, nxt);
          la = nxt;
        end else begin
          hw64(la, pages[k]);
          k++; left--;
          la += 8;
        end
      end
    end
  endtask

  function automatic bit [63:0] xaddr(input int off1, input int j);
    int jj;
    jj = off1 + j;
    return pages[jj / 4096] + 64'(jj % 4096);
  endfunction

  // ---------------- disk model ----------------
  bit [31:0] disk [bit [63:0]];    // key = LBA * 1024 + DW index

  task automatic do_write(input int q, input bit [63:0] slba, input int nblk, input int off1);
    bit [63:0] p1, p2;
    sqe_t      c;
    bit [31:0] dw0;
    bit [14:0] st;
    int        nbytes;
    nbytes = nblk * 4096;
    build_prp(64'h0000_0000_2000_0000 + 64'($urandom_range(0, 15)) * 64'h10_0000, nbytes, off1, p1, p2);
    for (int j = 0; j < nbytes / 4; j++) begin
      bit [31:0] d;
      d = $urandom;
      hw32(xaddr(off1, j * 4), d);
      disk[slba * 1024 + j] = d;
    end
    c = new_cmd(8'h01, 32'd1);
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = slba[31:0]; c[11] = slba[63:32]; c[12] = 32'(nblk - 1);
    run_cmd(q, c, dw0, st);
    if (st != 0) fail($sformatf("WRITE q=%0d lba=%0d n=%0d status=%h", q, slba, nblk, st));
  endtask

  task automatic do_read(input int q, input bit [63:0] slba, input int nblk, input int off1);
    bit [63:0] p1, p2;
    sqe_t      c;
    bit [31:0] dw0;
    bit [14:0] st;
    int        nbytes, bad;
    nbytes = nblk * 4096;
    bad    = 0;
    build_prp(64'h0000_0001_8000_0000 + 64'($urandom_range(0, 15)) * 64'h10_0000, nbytes, off1, p1, p2);
    for (int j = 0; j < nbytes / 4; j++) hw32(xaddr(off1, j * 4), 32'hA5A5_A5A5);
    c = new_cmd(8'h02, 32'd1);
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = slba[31:0]; c[11] = slba[63:32]; c[12] = 32'(nblk - 1);
    run_cmd(q, c, dw0, st);
    if (st != 0) fail($sformatf("READ q=%0d lba=%0d n=%0d status=%h", q, slba, nblk, st));
    for (int j = 0; j < nbytes / 4; j++) begin
      bit [31:0] exp, got;
      bit [63:0] k;
      k   = slba * 1024 + 64'(j);
      exp = disk.exists(k) ? disk[k] : 32'd0;
      got = hr32(xaddr(off1, j * 4));
      if (got != exp) begin
        if (bad < 5) fail($sformatf("READ data lba=%0d dw=%0d got=%h exp=%h (off1=%0d n=%0d)",
                                    slba + j / 1024, j % 1024, got, exp, off1, nblk));
        bad++;
      end
    end
  endtask

  // ---------------- controller reset / bring-up ----------------
  // CC.EN = 0, then RDY must clear within max_cycles (well under CAP.TO)
  task automatic disable_ctrl(input string what, input int max_cycles);
    bit [31:0] r;
    int        t0;
    t0 = 0;
    mmio_write32(32'h14, 32'h0046_0000);
    r = 1;
    while (r[0] && t0 < max_cycles) begin
      repeat (50) @(posedge clk);
      t0 += 50;
      mmio_read32(32'h1C, r);
    end
    if (r[0]) fail($sformatf("%s: CSTS.RDY did not clear in %0d cycles", what, max_cycles));
    else      $display("[%0t] %s: CSTS.RDY cleared after ~%0d cycles", $time, what, t0);
  endtask

  // enable with the admin queue at the same place, then create I/O queue pair 1
  task automatic bring_up();
    bit [31:0] r, dw0;
    bit [14:0] st;
    sqe_t      c;
    for (int q = 0; q <= NQ; q++) begin sq_tail[q] = 0; cq_head[q] = 0; cq_ph[q] = 1; end
    for (int i = 0; i < 64; i++) begin
      hw32(cq_base[0] + 64'(i * 16 + 12), 32'd0);   // stale CQEs carry phase 1
      hw32(cq_base[1] + 64'(i * 16 + 12), 32'd0);
    end
    mmio_write32(32'h24, {4'd0, 12'(cq_size[0] - 1), 4'd0, 12'(sq_size[0] - 1)});
    mmio_write64(32'h28, sq_base[0]);
    mmio_write64(32'h30, cq_base[0]);
    mmio_write32(32'h14, 32'h0046_0001);
    r = 0;
    for (int i = 0; i < 100 && !r[0]; i++) mmio_read32(32'h1C, r);
    if (!r[0]) fail("bring-up: CSTS.RDY did not set");
    c = new_cmd(8'h05, 32'd0);
    c[6] = cq_base[1][31:0]; c[7] = cq_base[1][63:32];
    c[10] = {16'(cq_size[1] - 1), 16'd1}; c[11] = {16'd1, 14'd0, 1'b1, 1'b1};
    run_cmd(0, c, dw0, st);
    if (st != 0) fail($sformatf("bring-up: Create CQ status %h", st));
    c = new_cmd(8'h01, 32'd0);
    c[6] = sq_base[1][31:0]; c[7] = sq_base[1][63:32];
    c[10] = {16'(sq_size[1] - 1), 16'd1}; c[11] = {16'd1, 15'd0, 1'b1};
    run_cmd(0, c, dw0, st);
    if (st != 0) fail($sformatf("bring-up: Create SQ status %h", st));
  endtask

  // ---------------- main sequence ----------------
  initial begin
    bit [31:0] r, dw0;
    bit [14:0] st;
    sqe_t      c;
    bit [63:0] idbuf;
    bit [15:0] cid;
    idbuf = 64'h0000_0000_3000_0000;

    rst = 1'b1;
    repeat (20) @(posedge clk);
    rst = 1'b0;
    repeat (20) @(posedge clk);

    // MSI-X table: vector v -> MSI_ADDR, data v, unmasked
    for (int v = 0; v < NV; v++) begin
      mmio_write32(32'h2000 + 32'(16 * v) + 0, MSI_ADDR[31:0]);
      mmio_write32(32'h2000 + 32'(16 * v) + 4, MSI_ADDR[63:32]);
      mmio_write32(32'h2000 + 32'(16 * v) + 8, 32'(v));
      mmio_write32(32'h2000 + 32'(16 * v) + 12, 32'd0);
    end

    mmio_read32(32'h0, r);  $display("CAP.lo = %h", r);
    if (r[15:0] != 16'd4095 || !r[16]) fail("CAP.MQES/CQR");
    mmio_read32(32'h4, r);  $display("CAP.hi = %h", r);
    if (r[5] != 1'b1) fail("CAP.CSS NVM bit");
    mmio_read32(32'h68, r); $display("CRTO = %h", r);
    if (r != {16'd20, 16'd20}) fail("CRTO");
    mmio_read32(32'h8, r);  $display("VS     = %h", r);
    if (r != 32'h0002_0400) fail("VS");

    // ---- admin queue + enable ----
    sq_base[0] = 64'h0000_0000_0010_0000; sq_size[0] = 32;
    cq_base[0] = 64'h0000_0000_0011_0000; cq_size[0] = 32;
    sq_tail[0] = 0; cq_head[0] = 0; cq_ph[0] = 1; cq_vec[0] = 0; cq_of_sq[0] = 0;
    mmio_write32(32'h24, {4'd0, 12'(cq_size[0] - 1), 4'd0, 12'(sq_size[0] - 1)});
    mmio_write64(32'h28, sq_base[0]);
    mmio_write64(32'h30, cq_base[0]);
    mmio_write32(32'h14, 32'h0046_0001);   // IOCQES=4, IOSQES=6, MPS=0, CSS=NVM, EN=1
    r = 0;
    for (int i = 0; i < 100 && !r[0]; i++) mmio_read32(32'h1C, r);
    if (!r[0]) fail("CSTS.RDY did not set");
    $display("[%0t] controller ready", $time);

    // ---- Identify controller (PRP1 offset 0x800, second half lands in the PRP2 page) ----
    c = new_cmd(8'h06, 32'd0);
    c[6] = idbuf[31:0] + 32'h800; c[7] = idbuf[63:32];
    c[8] = idbuf[31:0] + 32'h5000; c[9] = idbuf[63:32];
    c[10] = 32'h1;
    run_cmd(0, c, dw0, st);
    if (st != 0) fail("Identify controller status");
    if ({hr8(idbuf + 64'h801), hr8(idbuf + 64'h800)} != 16'h10EE) fail("Identify VID");
    if (hr8(idbuf + 64'h800 + 77) != 8'(REC_LOG2 - 11)) fail("Identify MDTS");
    if (hr32(idbuf + 64'h800 + 80) != 32'h0002_0400) fail("Identify VER");
    if (hr8(idbuf + 64'h800 + 512) != 8'h66)      fail("Identify SQES");
    if (hr8(idbuf + 64'h800 + 513) != 8'h44)      fail("Identify CQES");
    if (hr32(idbuf + 64'h800 + 516) != 32'd1)     fail("Identify NN");
    if ({hr8(idbuf + 64'h5001), hr8(idbuf + 64'h5000)} != 16'd2500) fail("Identify PSD0 (PRP2 page)");
    begin
      string mn;
      mn = "";
      for (int i = 24; i < 44; i++) mn = $sformatf("%s%c", mn, hr8(idbuf + 64'h800 + i));
      $display("Identify MN = '%s'", mn);
    end

    // ---- Identify namespace ----
    c = new_cmd(8'h06, 32'd1);
    c[6] = idbuf[31:0] + 32'h2000; c[7] = 0;
    c[10] = 32'h0;
    run_cmd(0, c, dw0, st);
    if (st != 0) fail("Identify NS status");
    if (hr64(idbuf + 64'h2000) != NSZE) fail($sformatf("Identify NSZE %0d", hr64(idbuf + 64'h2000)));
    if (hr32(idbuf + 64'h2000 + 128) != 32'h000C_0000) fail("Identify LBAF0");

    // ---- Identify CNS 08h (I/O command set independent NS): NSTAT.NRDY ----
    c = new_cmd(8'h06, 32'd1);
    c[6] = idbuf[31:0] + 32'h7000; c[7] = 0; c[10] = 32'h8;
    run_cmd(0, c, dw0, st);
    if (st != 0) fail("Identify CNS 08h status");
    if (hr8(idbuf + 64'h7000 + 14) != 8'h01) fail("Identify CNS 08h NSTAT.NRDY");

    // ---- Identify CNS 06h (I/O command set specific): zeros, success ----
    c = new_cmd(8'h06, 32'd0);
    c[6] = idbuf[31:0] + 32'h8000; c[10] = 32'h6;
    run_cmd(0, c, dw0, st);
    if (st != 0) fail("Identify CNS 06h status");

    // ---- Set Features: number of queues ----
    c = new_cmd(8'h09, 32'd0);
    c[10] = 32'h7; c[11] = {16'(NQ - 1), 16'(NQ - 1)};
    run_cmd(0, c, dw0, st);
    if (st != 0 || dw0 != {16'(NQ - 1), 16'(NQ - 1)}) fail($sformatf("Set Features NQ dw0=%h st=%h", dw0, st));

    // ---- Get Log Page: SMART, 512 bytes ----
    c = new_cmd(8'h02, 32'hFFFF_FFFF);
    c[6] = idbuf[31:0] + 32'h9000; c[10] = {16'(128 - 1), 8'd0, 8'h02};
    run_cmd(0, c, dw0, st);
    if (st != 0) fail("Get Log Page status");
    if ({hr8(idbuf + 64'h9002), hr8(idbuf + 64'h9001)} != 16'd300) fail("SMART temperature");

    // ---- Async Event Requests (stay outstanding) ----
    for (int i = 0; i < 2; i++) begin
      c = new_cmd(8'h0C, 32'd0);
      submit(0, c, cid);
    end

    // ---- create I/O queues 1 and 2 ----
    sq_base[1] = 64'h0000_0001_0000_0000; sq_size[1] = 16; cq_base[1] = 64'h0000_0000_0012_0000; cq_size[1] = 8;
    sq_base[2] = 64'h0000_0000_0013_0000; sq_size[2] = 64; cq_base[2] = 64'h0000_0001_0001_0000; cq_size[2] = 64;
    for (int q = 1; q <= 2; q++) begin
      sq_tail[q] = 0; cq_head[q] = 0; cq_ph[q] = 1; cq_vec[q] = q; cq_of_sq[q] = q;
      c = new_cmd(8'h05, 32'd0);   // create CQ
      c[6] = cq_base[q][31:0]; c[7] = cq_base[q][63:32];
      c[10] = {16'(cq_size[q] - 1), 16'(q)};
      c[11] = {16'(q), 14'd0, 1'b1, 1'b1};      // IV=q, IEN, PC
      run_cmd(0, c, dw0, st);
      if (st != 0) fail($sformatf("Create CQ %0d status %h", q, st));
      c = new_cmd(8'h01, 32'd0);   // create SQ
      c[6] = sq_base[q][31:0]; c[7] = sq_base[q][63:32];
      c[10] = {16'(sq_size[q] - 1), 16'(q)};
      c[11] = {16'(q), 15'd0, 1'b1};            // CQID=q, PC
      run_cmd(0, c, dw0, st);
      if (st != 0) fail($sformatf("Create SQ %0d status %h", q, st));
    end

    // ---- error cases ----
    c = new_cmd(8'h05, 32'd0);  c[6] = 32'h0014_0000; c[10] = {16'd7, 16'd1}; c[11] = 32'h3;
    run_cmd(0, c, dw0, st);
    if (st[10:0] != ST_INVALID_QID) fail($sformatf("duplicate CQ create status %h", st));

    c = new_cmd(8'h02, 32'd1);  c[6] = 32'h2000_0000; c[10] = NSZE[31:0]; c[11] = NSZE[63:32];
    run_cmd(1, c, dw0, st);
    if (st[10:0] != ST_LBA_RANGE || !st[14]) fail($sformatf("LBA range status %h", st));

    c = new_cmd(8'h7F, 32'd1);
    run_cmd(2, c, dw0, st);
    if (st[10:0] != ST_INVALID_OPCODE) fail($sformatf("invalid opcode status %h", st));

    c = new_cmd(8'h01, 32'd1);  c[6] = 32'h2000_0000; c[12] = 32'((2 << REC_LOG2) / 4096);   // MDTS + 1 blocks
    run_cmd(1, c, dw0, st);
    if (st[10:0] != ST_INVALID_FIELD) fail($sformatf("MDTS status %h", st));

    c = new_cmd(8'h00, 32'd1);   // flush
    run_cmd(2, c, dw0, st);
    if (st != 0) fail("flush status");

    // ---- directed I/O: every PRP layout ----
    do_write(1, 64'd0,   1,  0);       // PRP1 only
    do_read (1, 64'd0,   1,  0);
    do_write(2, 64'd10,  2,  0);       // PRP1 + PRP2
    do_read (2, 64'd10,  2,  512);     // offset -> spills into 2 pages + PRP2... 3 pages => list
    do_write(1, 64'd100, 32, 0);       // max transfer, list
    do_read (1, 64'd100, 32, 4092);    // max transfer, odd-DW offset, 33 pages, chained list
    do_write(2, 64'd200, 7,  12);
    do_read (2, 64'd200, 7,  0);

    // ---- random I/O ----
    for (int it = 0; it < N_IO; it++) begin
      int        q, nblk, off1;
      bit [63:0] slba;
      q    = $urandom_range(1, 2);
      nblk = ($urandom_range(0, 3) == 0) ? 32 : $urandom_range(1, 12);
      slba = 64'($urandom_range(0, 400));
      off1 = ($urandom_range(0, 2) == 0) ? 0 : 4 * $urandom_range(0, 1023);
      if ($urandom_range(0, 1)) do_write(q, slba, nblk, off1);
      else                      do_read (q, slba, nblk, off1);
    end

    // ---- last LBA ----
    do_write(1, NSZE - 2, 2, 0);
    do_read (2, NSZE - 2, 2, 8);

    // ---- compression path ----
    run_comp_tests();

    // ---- delete queues, disable ----
    for (int q = 2; q >= 1; q--) begin
      c = new_cmd(8'h00, 32'd0); c[10] = 32'(q);
      run_cmd(0, c, dw0, st);
      if (st != 0) fail($sformatf("Delete SQ %0d status %h", q, st));
      c = new_cmd(8'h04, 32'd0); c[10] = 32'(q);
      run_cmd(0, c, dw0, st);
      if (st != 0) fail($sformatf("Delete CQ %0d status %h", q, st));
    end
    mmio_write32(32'h14, 32'h0046_0000);
    r = 1;
    for (int i = 0; i < 100 && r[0]; i++) mmio_read32(32'h1C, r);
    if (r[0]) fail("CSTS.RDY did not clear");
    if (dma_err) fail("dma_err set");

    // ---- CC.EN 1 -> 0 with I/O queues live and a command in flight ----
    // (what Linux sees after the UEFI NVMe driver left the controller enabled)
    bring_up();
    do_write(1, 64'd300, 4, 0);
    begin
      bit [63:0] p1, p2;
      build_prp(64'h0000_0000_2000_0000, 32 * 4096, 0, p1, p2);
      c = new_cmd(8'h01, 32'd1);
      c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
      c[10] = 32'd320; c[12] = 32'd31;
      submit(1, c, cid);
    end
    repeat (300) @(posedge clk);
    disable_ctrl("disable with a write in flight", 50000);
    mmio_read32(32'hF00, r);
    if (r[5:0] != 6'd0 || !r[8] || !r[9]) fail($sformatf("debug reg after disable %h", r));
    bring_up();
    do_read(1, 64'd300, 4, 0);
    if (dma_err) fail("dma_err set after in-flight disable");

    // ---- CC.EN 1 -> 0 while the host has stopped answering reads ----
    disable_ctrl("disable after reset test", 50000);
    bring_up();
    drop_mrd = 1'b1;
    c = new_cmd(8'h02, 32'd1); c[6] = 32'h2000_0000; c[12] = 32'd0;
    submit(1, c, cid);                      // the SQE fetch is never answered
    repeat (2000) @(posedge clk);
    mmio_read32(32'hF00, r);
    if (r[8]) fail($sformatf("dma_rd idle with a dropped read outstanding (dbg %h)", r));
    disable_ctrl("disable with lost completions", 4 * CPL_TO);
    mmio_read32(32'hF00, r);
    if (!r[15] || !r[14]) fail($sformatf("completion timeout not flagged (dbg %h)", r));
    drop_mrd = 1'b0;
    bring_up();
    do_write(1, 64'd400, 3, 8);
    do_read (1, 64'd400, 3, 0);
    disable_ctrl("final disable", 50000);
    $display("MRd TLPs: %0d  MWr TLPs: %0d  MSI-X per vector: %0d %0d %0d",
             n_mrd, n_mwr, msi_cnt[0], msi_cnt[1], msi_cnt[2]);
    $display("DDR bursts: %0d AW, %0d W, %0d B", u_ddr.n_aw, u_ddr.n_w, u_ddr.n_b);
    $display("RX beats: %0d straddled, %0d TLP starts at DW2", n_straddle, n_sof2);
    if (errors == 0) $display("TEST PASSED");
    else             $display("TEST FAILED (%0d errors)", errors);
    $finish;
  end

  `include "tb_comp.svh"

  // optional waveform dump: run with +vcd (Verilator build needs --trace)
  initial begin
    if ($test$plusargs("vcd")) begin
      $dumpfile("waves.vcd");
      $dumpvars(0, tb_top);
    end
  end

  // global watchdog
  initial begin
    #(2s);
    $display("TEST FAILED (watchdog)");
    $finish;
  end

endmodule
