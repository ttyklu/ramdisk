// axi_mem_model.sv - AXI4 slave memory model (64-bit data, INCR bursts) with random stalls.
// Unwritten locations read as zero.
`timescale 1ns/1ps
module axi_mem_model #(
  parameter int STALL_PCT = 25          // % of cycles a ready/valid is withheld
)(
  input  logic        clk,
  input  logic        rst,
  input  logic [31:0] awaddr,
  input  logic [7:0]  awlen,
  input  logic [2:0]  awsize,
  input  logic [1:0]  awburst,
  input  logic        awvalid,
  output logic        awready,
  input  logic [63:0] wdata,
  input  logic [7:0]  wstrb,
  input  logic        wlast,
  input  logic        wvalid,
  output logic        wready,
  output logic [1:0]  bresp,
  output logic        bvalid,
  input  logic        bready,
  input  logic [31:0] araddr,
  input  logic [7:0]  arlen,
  input  logic [2:0]  arsize,
  input  logic [1:0]  arburst,
  input  logic        arvalid,
  output logic        arready,
  output logic [63:0] rdata,
  output logic [1:0]  rresp,
  output logic        rlast,
  output logic        rvalid,
  input  logic        rready
);

  bit [63:0] mem [bit [31:0]];          // key = byte address >> 3

  function automatic bit [63:0] rd64(input bit [31:0] a);
    return mem.exists(a >> 3) ? mem[a >> 3] : 64'd0;
  endfunction

  function automatic bit go();
    return ($urandom_range(0, 99) >= STALL_PCT);
  endfunction

  // ---------------- write channel ----------------
  bit [31:0] aw_addr_q[$];
  bit [7:0]  aw_len_q[$];
  int        wbeat;
  int        b_pending;
  int        n_aw, n_w, n_b;

  assign bresp = 2'b00;

  always @(posedge clk) begin
    if (rst) begin
      awready   <= 1'b0;
      wready    <= 1'b0;
      bvalid    <= 1'b0;
      wbeat     = 0;
      b_pending = 0;
    end else begin
      if (awvalid && awready) begin
        if (awburst != 2'b01 || awsize != 3'd3) $error("axi_mem_model: unsupported AW burst/size");
        aw_addr_q.push_back(awaddr);
        aw_len_q.push_back(awlen);
        n_aw++;
      end
      if (wvalid && wready) begin
        bit [31:0] a;
        bit [63:0] old_v, new_v;
        if (aw_addr_q.size() == 0) $fatal(1, "axi_mem_model: W beat without AW");
        a     = aw_addr_q[0] + 32'(wbeat * 8);
        old_v = rd64(a);
        for (int i = 0; i < 8; i++) new_v[i*8 +: 8] = wstrb[i] ? wdata[i*8 +: 8] : old_v[i*8 +: 8];
        mem[a >> 3] = new_v;
        if (wlast != (wbeat == int'(aw_len_q[0]))) $error("axi_mem_model: WLAST mismatch");
        if (wbeat == int'(aw_len_q[0])) begin
          void'(aw_addr_q.pop_front());
          void'(aw_len_q.pop_front());
          wbeat = 0;
          b_pending++;
          n_w++;
        end else begin
          wbeat++;
        end
      end
      if (bvalid && bready) begin
        b_pending--;
        n_b++;
      end
      awready <= go();
      wready  <= go();
      if (bvalid && !bready) bvalid <= 1'b1;              // hold until accepted
      else                   bvalid <= (b_pending > 0) && go();
    end
  end

  // ---------------- read channel ----------------
  bit [31:0] ar_addr_q[$];
  bit [7:0]  ar_len_q[$];
  int        rbeat;

  assign rresp = 2'b00;

  always @(posedge clk) begin
    if (rst) begin
      arready <= 1'b0;
      rvalid  <= 1'b0;
      rlast   <= 1'b0;
      rbeat   = 0;
    end else begin
      if (arvalid && arready) begin
        if (arburst != 2'b01 || arsize != 3'd3) $error("axi_mem_model: unsupported AR burst/size");
        ar_addr_q.push_back(araddr);
        ar_len_q.push_back(arlen);
      end
      if (rvalid && rready) begin
        if (rlast) begin
          void'(ar_addr_q.pop_front());
          void'(ar_len_q.pop_front());
          rbeat = 0;
        end else begin
          rbeat++;
        end
      end
      arready <= go();
      if ((!rvalid || rready)) begin
        if (ar_addr_q.size() > 0 && go()) begin
          rvalid <= 1'b1;
          rdata  <= rd64(ar_addr_q[0] + 32'(rbeat * 8));
          rlast  <= (rbeat == int'(ar_len_q[0]));
        end else begin
          rvalid <= 1'b0;
          rlast  <= 1'b0;
        end
      end
    end
  end

endmodule
