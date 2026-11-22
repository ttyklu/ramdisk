// tlp_tx.sv - builds TLPs for the 7-series PCIe core (64-bit AXI-Stream, s_axis_tx_*).
// Three request ports, each a descriptor with valid/ready; a whole TLP is sent per accept.
//   C : completion for a register read (Cpl / CplD, 0..2 DW payload in the descriptor)
//   R : memory read request (MRd, no payload) for DMA reads of host memory
//   W : memory write (MWr, 1..32 DW); payload is popped from the TX DW FIFO (w_pop/w_dw*)
//       The W requester must make sure the FIFO already holds w_len DWs.
// Addresses below 4 GB use 3DW headers, others 4DW (as required by the PCIe spec).
module tlp_tx import nvme_pkg::*; #(
  parameter bit PAYLOAD_BSWAP = 1'b0
)(
  input  logic        clk,
  input  logic        rst,
  input  logic [15:0] completer_id,

  output logic [63:0] tx_tdata,
  output logic [7:0]  tx_tkeep,
  output logic        tx_tlast,
  output logic        tx_tvalid,
  input  logic        tx_tready,
  output logic [3:0]  tx_tuser,

  // C port
  input  logic        c_valid,
  output logic        c_ready,
  input  logic [15:0] c_rid,
  input  logic [7:0]  c_tag,
  input  logic [2:0]  c_tc,
  input  logic [1:0]  c_attr,
  input  logic [2:0]  c_status,
  input  logic [6:0]  c_la,
  input  logic [11:0] c_bc,
  input  logic [1:0]  c_len,       // payload DWs (0 = Cpl without data)
  input  logic [31:0] c_d0,
  input  logic [31:0] c_d1,

  // R port
  input  logic        r_valid,
  output logic        r_ready,
  input  logic [63:0] r_addr,
  input  logic [10:0] r_len,
  input  logic [7:0]  r_tag,

  // W port
  input  logic        w_valid,
  output logic        w_ready,
  input  logic [63:0] w_addr,
  input  logic [10:0] w_len,
  output logic        w_done,      // pulse when the last beat of a W TLP is accepted
  output logic [1:0]  w_pop,
  input  logic [31:0] w_dw0,
  input  logic [31:0] w_dw1
);

  function automatic logic [31:0] pl(input logic [31:0] d);
    return PAYLOAD_BSWAP ? bswap32(d) : d;
  endfunction

  typedef enum logic [1:0] {SEL_C, SEL_R, SEL_W} sel_t;

  logic        busy;
  sel_t        sel;
  logic        last_was_w;
  logic [1:0]  phase;            // 0: beat0, 1: beat1, 2: payload beats
  logic        is4;
  logic [10:0] rem;              // payload DWs still to send
  logic [31:0] h0, h1, h2, h3;
  logic [31:0] cd0, cd1;
  logic        ci;               // next C payload DW index

  // ---------------- arbitration (combinational ready) ----------------
  logic pick_c, pick_w, pick_r;
  always_comb begin
    pick_c = 1'b0; pick_w = 1'b0; pick_r = 1'b0;
    if (!busy) begin
      if (c_valid)                              pick_c = 1'b1;
      else if (w_valid && (!r_valid || !last_was_w)) pick_w = 1'b1;
      else if (r_valid)                         pick_r = 1'b1;
      else if (w_valid)                         pick_w = 1'b1;
    end
  end
  assign c_ready = pick_c;
  assign r_ready = pick_r;
  assign w_ready = pick_w;

  wire r_is4 = (r_addr[63:32] != 32'd0);
  wire w_is4 = (w_addr[63:32] != 32'd0);

  // ---------------- beat construction ----------------
  logic [31:0] p0, p1;
  always_comb begin
    if (sel == SEL_C) begin
      p0 = (ci == 1'b0) ? cd0 : cd1;
      p1 = cd1;
    end else begin
      p0 = w_dw0;
      p1 = w_dw1;
    end
  end

  logic [1:0] npay;   // payload DWs carried by the current beat
  always_comb begin
    tx_tdata = 64'd0;
    tx_tkeep = 8'hFF;
    tx_tlast = 1'b0;
    npay     = 2'd0;
    case (phase)
      2'd0: begin
        tx_tdata = {h1, h0};
      end
      2'd1: begin
        if (is4) begin
          tx_tdata = {h3, h2};
          tx_tlast = (rem == 11'd0);
        end else if (rem != 11'd0) begin
          tx_tdata = {pl(p0), h2};
          npay     = 2'd1;
          tx_tlast = (rem == 11'd1);
        end else begin
          tx_tdata = {32'd0, h2};
          tx_tkeep = 8'h0F;
          tx_tlast = 1'b1;
        end
      end
      default: begin
        if (rem >= 11'd2) begin
          tx_tdata = {pl(p1), pl(p0)};
          npay     = 2'd2;
          tx_tlast = (rem == 11'd2);
        end else begin
          tx_tdata = {32'd0, pl(p0)};
          tx_tkeep = 8'h0F;
          npay     = 2'd1;
          tx_tlast = 1'b1;
        end
      end
    endcase
  end

  assign tx_tvalid = busy;
  assign tx_tuser  = 4'd0;

  wire beat_fire = tx_tvalid && tx_tready;
  assign w_pop  = (beat_fire && sel == SEL_W) ? npay : 2'd0;

  always_ff @(posedge clk) begin
    if (rst) begin
      busy       <= 1'b0;
      sel        <= SEL_C;
      phase      <= 2'd0;
      rem        <= '0;
      last_was_w <= 1'b0;
      w_done     <= 1'b0;
      ci         <= 1'b0;
    end else begin
      w_done <= 1'b0;

      if (!busy) begin
        phase <= 2'd0;
        ci    <= 1'b0;
        if (pick_c) begin
          busy <= 1'b1;
          sel  <= SEL_C;
          is4  <= 1'b0;
          rem  <= {9'd0, c_len};
          cd0  <= c_d0;
          cd1  <= c_d1;
          h0   <= mk_h0((c_len != 2'd0) ? FMT_3DW_D : FMT_3DW_ND, TLP_T_CPL, c_tc, c_attr, {8'd0, c_len});
          h1   <= {completer_id, c_status, 1'b0, c_bc};
          h2   <= {c_rid, c_tag, 1'b0, c_la};
          h3   <= 32'd0;
        end else if (pick_w) begin
          busy       <= 1'b1;
          sel        <= SEL_W;
          last_was_w <= 1'b1;
          is4        <= w_is4;
          rem        <= w_len;
          h0   <= mk_h0(w_is4 ? FMT_4DW_D : FMT_3DW_D, TLP_T_MEM, 3'd0, 2'd0, w_len[9:0]);
          h1   <= {completer_id, 8'd0, (w_len == 11'd1) ? 4'h0 : 4'hF, 4'hF};
          h2   <= w_is4 ? w_addr[63:32] : {w_addr[31:2], 2'b00};
          h3   <= {w_addr[31:2], 2'b00};
        end else if (pick_r) begin
          busy       <= 1'b1;
          sel        <= SEL_R;
          last_was_w <= 1'b0;
          is4        <= r_is4;
          rem        <= '0;
          h0   <= mk_h0(r_is4 ? FMT_4DW_ND : FMT_3DW_ND, TLP_T_MEM, 3'd0, 2'd0, r_len[9:0]);
          h1   <= {completer_id, r_tag, (r_len == 11'd1) ? 4'h0 : 4'hF, 4'hF};
          h2   <= r_is4 ? r_addr[63:32] : {r_addr[31:2], 2'b00};
          h3   <= {r_addr[31:2], 2'b00};
        end
      end else if (beat_fire) begin
        rem <= rem - {9'd0, npay};
        if (sel == SEL_C && npay != 2'd0) ci <= 1'b1;
        if (phase != 2'd2) phase <= phase + 2'd1;
        if (tx_tlast) begin
          busy <= 1'b0;
          if (sel == SEL_W) w_done <= 1'b1;
        end
      end
    end
  end

endmodule
