// rx128_to_64.sv - 7-series PCIe 128-bit RX AXI-Stream (m_axis_rx_*, 125 MHz) to the 64-bit
// TLP beat format used by tlp_rx (beat0 = {H1,H0}, then two DWs per beat, tkeep 0x0F / 0xFF,
// tlast on the last beat).
//  * TLP boundaries come from m_axis_rx_tuser: is_sof = [14:10] (bit 14 present, [13:10] byte
//    offset 0 or 8), is_eof = [21:17] (bit 21 present, [20:17] offset of the last byte).
//    A beat may carry the end of one TLP and the start of the next (straddle at DW2).
//  * bar_hit (tuser[8:2]) is taken from the beat where a TLP starts.
//  * DWs go through a 16-entry DW queue; up to 4 DWs in, up to 2 out per cycle.
module rx128_to_64 (
  input  logic         clk,
  input  logic         rst,

  input  logic [127:0] s_tdata,
  input  logic         s_tvalid,
  output logic         s_tready,
  input  logic [21:0]  s_tuser,

  output logic [63:0]  m_tdata,
  output logic [7:0]   m_tkeep,
  output logic         m_tlast,
  output logic         m_tvalid,
  input  logic         m_tready,
  output logic [21:0]  m_tuser
);
  localparam int QD = 16;

  logic [31:0] q_d   [QD];
  logic        q_sof [QD];
  logic        q_eof [QD];
  logic [6:0]  q_bar [QD];
  logic [4:0]  wp, rp;                    // one extra bit
  wire  [4:0]  cnt = wp - rp;

  logic        in_pkt;                    // a TLP is open across beats
  logic [6:0]  cur_bar;

  wire       sof_p   = s_tuser[14];
  wire [1:0] sof_dw  = s_tuser[13:12];    // byte offset / 4 (0 or 2)
  wire       eof_p   = s_tuser[21];
  wire [1:0] eof_dw  = s_tuser[20:19];    // DW of the last byte

  // which DWs of the beat carry TLP data, and their markers
  logic [3:0] v, f, l;
  always_comb begin
    v = 4'b0; f = 4'b0; l = 4'b0;
    if (in_pkt) begin
      // continuing TLP from DW0
      if (eof_p) begin
        for (int k = 0; k < 4; k++) if (k <= int'(eof_dw)) v[k] = 1'b1;
        l[eof_dw] = 1'b1;
        if (sof_p && sof_dw > eof_dw) begin            // straddle: new TLP starts after it
          for (int k = 0; k < 4; k++) if (k >= int'(sof_dw)) v[k] = 1'b1;
          f[sof_dw] = 1'b1;
        end
      end else begin
        v = 4'b1111;
      end
    end else if (sof_p) begin
      f[sof_dw] = 1'b1;
      for (int k = 0; k < 4; k++)
        if (k >= int'(sof_dw) && (!eof_p || eof_dw < sof_dw || k <= int'(eof_dw))) v[k] = 1'b1;
      if (eof_p && eof_dw >= sof_dw) l[eof_dw] = 1'b1;
    end
  end
  wire [2:0] nin = 3'(v[0]) + 3'(v[1]) + 3'(v[2]) + 3'(v[3]);

  assign s_tready = (cnt <= 5'(QD - 4));
  wire   in_fire  = s_tvalid && s_tready;

  // output: two DWs of one TLP, or its last single DW
  wire [3:0] r0 = rp[3:0];
  wire [3:0] r1 = rp[3:0] + 4'd1;
  wire       h_ok  = (cnt >= 5'd1);
  wire       two   = (cnt >= 5'd2) && !q_eof[r0];
  assign m_tvalid = h_ok && (q_eof[r0] || two);
  assign m_tdata  = {two ? q_d[r1] : 32'd0, q_d[r0]};
  assign m_tkeep  = two ? 8'hFF : 8'h0F;
  assign m_tlast  = two ? q_eof[r1] : q_eof[r0];
  assign m_tuser  = {13'd0, q_bar[r0], 2'b00};
  wire   out_fire = m_tvalid && m_tready;

  always_ff @(posedge clk) begin
    if (rst) begin
      wp     <= '0;
      rp     <= '0;
      in_pkt <= 1'b0;
    end else begin
      if (in_fire) begin
        logic [4:0] w;
        logic [6:0] bar;
        w   = wp;
        bar = sof_p ? s_tuser[8:2] : cur_bar;
        for (int k = 0; k < 4; k++) begin
          if (v[k]) begin
            q_d  [w[3:0]] <= s_tdata[32*k +: 32];
            q_sof[w[3:0]] <= f[k];
            q_eof[w[3:0]] <= l[k];
            q_bar[w[3:0]] <= (f[k] || !in_pkt) ? s_tuser[8:2] : cur_bar;
            w = w + 5'd1;
          end
        end
        wp <= w;
        if (sof_p) cur_bar <= s_tuser[8:2];
        // packet state after this beat
        if (sof_p && (!eof_p || eof_dw < sof_dw)) in_pkt <= 1'b1;
        else if (eof_p)                           in_pkt <= 1'b0;
      end
      if (out_fire) rp <= rp + (two ? 5'd2 : 5'd1);
    end
  end
endmodule
