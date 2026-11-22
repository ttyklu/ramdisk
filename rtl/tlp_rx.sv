// tlp_rx.sv - parses TLPs from the 7-series PCIe core (64-bit AXI-Stream, m_axis_rx_*).
//  * MRd / MWr hitting BAR0  -> one completer request (creq_*) to nvme_regs
//  * Cpl / CplD              -> header + data events (cpl_*) to dma_rd
//  * everything else         -> dropped
// Beat layout (7-series 64-bit):  beat0 = {H1,H0}; beat1 = {D0,H2} (3DW) or {H3,H2} (4DW)
module tlp_rx import nvme_pkg::*; #(
  parameter bit PAYLOAD_BSWAP = 1'b0
)(
  input  logic        clk,
  input  logic        rst,

  input  logic [63:0] rx_tdata,
  input  logic [7:0]  rx_tkeep,
  input  logic        rx_tlast,
  input  logic        rx_tvalid,
  output logic        rx_tready,
  input  logic [21:0] rx_tuser,     // [8:2] bar_hit

  // completer request (register access)
  output logic        creq_valid,
  input  logic        creq_ready,
  output logic        creq_wr,
  output logic [11:0] creq_dwaddr,  // BAR offset [13:2]
  output logic [10:0] creq_len,     // DWs
  output logic [3:0]  creq_fbe,
  output logic [3:0]  creq_lbe,
  output logic [31:0] creq_d0,
  output logic [31:0] creq_d1,
  output logic [15:0] creq_rid,
  output logic [7:0]  creq_tag,
  output logic [2:0]  creq_tc,
  output logic [1:0]  creq_attr,

  // completions for our DMA reads
  output logic        cpl_hdr_valid,
  output logic [7:0]  cpl_tag,
  output logic [6:0]  cpl_la,
  output logic [2:0]  cpl_status,
  output logic [10:0] cpl_len,
  output logic        cpl_hasdata,
  output logic        cpl_data_valid,
  output logic [1:0]  cpl_data_cnt,
  output logic [31:0] cpl_dw0,
  output logic [31:0] cpl_dw1
);

  function automatic logic [31:0] pl(input logic [31:0] d);
    return PAYLOAD_BSWAP ? bswap32(d) : d;
  endfunction

  typedef enum logic [1:0] {K_DROP, K_CREQ, K_CPL} kind_t;

  logic        in_pkt;      // 0: next beat is beat0
  logic [3:0]  bidx;        // beat index within packet (saturates)
  kind_t       kind;
  logic [31:0] h0, h1;
  logic        is4, hasd;

  wire fire = rx_tvalid && rx_tready;

  // A captured register request blocks the RX stream until nvme_regs takes it.
  assign rx_tready = !creq_valid;

  // beat0 decode (combinational, only meaningful when !in_pkt)
  wire [2:0] b0_fmt  = rx_tdata[31:29];
  wire [4:0] b0_type = rx_tdata[28:24];
  wire       b0_bar0 = rx_tuser[2];
  kind_t     b0_kind;
  always_comb begin
    b0_kind = K_DROP;
    if (b0_type == TLP_T_MEM && b0_fmt[2] == 1'b0 && b0_bar0) b0_kind = K_CREQ;
    else if (b0_type == TLP_T_CPL && b0_fmt[2] == 1'b0 && b0_fmt[0] == 1'b0) b0_kind = K_CPL;
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      in_pkt         <= 1'b0;
      bidx           <= '0;
      kind           <= K_DROP;
      creq_valid     <= 1'b0;
      cpl_hdr_valid  <= 1'b0;
      cpl_data_valid <= 1'b0;
      cpl_data_cnt   <= '0;
    end else begin
      cpl_hdr_valid  <= 1'b0;
      cpl_data_valid <= 1'b0;

      if (creq_valid && creq_ready) creq_valid <= 1'b0;

      if (fire) begin
        if (!in_pkt) begin
          // ---------------- beat 0 ----------------
          h0     <= rx_tdata[31:0];
          h1     <= rx_tdata[63:32];
          is4    <= b0_fmt[0];
          hasd   <= b0_fmt[1];
          kind   <= b0_kind;
          bidx   <= 4'd1;
          in_pkt <= !rx_tlast;
          if (b0_kind == K_CREQ) begin
            creq_wr   <= b0_fmt[1];
            creq_len  <= (rx_tdata[9:0] == 10'd0) ? 11'd1024 : {1'b0, rx_tdata[9:0]};
            creq_tc   <= rx_tdata[22:20];
            creq_attr <= rx_tdata[13:12];
            creq_rid  <= rx_tdata[63:48];
            creq_tag  <= rx_tdata[47:40];
            creq_lbe  <= rx_tdata[39:36];
            creq_fbe  <= rx_tdata[35:32];
            creq_d0   <= '0;
            creq_d1   <= '0;
          end
        end else begin
          // ---------------- beat 1.. ----------------
          if (bidx != 4'hF) bidx <= bidx + 4'd1;
          if (rx_tlast) in_pkt <= 1'b0;

          case (kind)
            K_CREQ: begin
              if (bidx == 4'd1) begin
                if (is4) begin
                  creq_dwaddr <= rx_tdata[45:34];          // H3[13:2]
                end else begin
                  creq_dwaddr <= rx_tdata[13:2];           // H2[13:2]
                  creq_d0     <= pl(rx_tdata[63:32]);      // D0
                end
              end else if (bidx == 4'd2) begin
                if (is4) begin
                  creq_d0 <= pl(rx_tdata[31:0]);
                  creq_d1 <= pl(rx_tdata[63:32]);
                end else begin
                  creq_d1 <= pl(rx_tdata[31:0]);
                end
              end
              if (rx_tlast) creq_valid <= 1'b1;
            end

            K_CPL: begin
              if (bidx == 4'd1) begin
                cpl_hdr_valid <= 1'b1;
                cpl_tag       <= rx_tdata[15:8];
                cpl_la        <= rx_tdata[6:0];
                cpl_status    <= h1[15:13];
                cpl_len       <= (h0[9:0] == 10'd0) ? 11'd1024 : {1'b0, h0[9:0]};
                cpl_hasdata   <= hasd;
                if (hasd && rx_tkeep[4]) begin
                  cpl_data_valid <= 1'b1;
                  cpl_data_cnt   <= 2'd1;
                  cpl_dw0        <= pl(rx_tdata[63:32]);
                end
              end else begin
                if (hasd) begin
                  cpl_data_valid <= 1'b1;
                  cpl_data_cnt   <= rx_tkeep[4] ? 2'd2 : 2'd1;
                  cpl_dw0        <= pl(rx_tdata[31:0]);
                  cpl_dw1        <= pl(rx_tdata[63:32]);
                end
              end
            end

            default: ;
          endcase
        end
      end
    end
  end

endmodule
