// nvme_regs.sv - BAR0 (16 KB) of the NVMe controller
//   0x0000  controller registers (CAP, VS, INTMS/INTMC, CC, CSTS, NSSR, AQA, ASQ, ACQ)
//   0x1000  doorbells (DSTRD = 0): SQyTDBL at 0x1000 + 8y, CQyHDBL at 0x1004 + 8y
//   0x2000  MSI-X table (16 B per vector)
//   0x3000  MSI-X PBA
// Handles one completer request at a time; register reads produce a completion on the C port.
module nvme_regs import nvme_pkg::*; #(
  parameter int NQ = 4,                  // I/O queue pairs (queue IDs 1..NQ)
  parameter int NV = 5,                  // MSI-X vectors
  parameter logic [15:0] MQES = 16'd4095 // max queue entries, 0's based
)(
  input  logic        clk,
  input  logic        rst,

  // completer request from tlp_rx
  input  logic        creq_valid,
  output logic        creq_ready,
  input  logic        creq_wr,
  input  logic [11:0] creq_dwaddr,
  input  logic [10:0] creq_len,
  input  logic [3:0]  creq_fbe,
  input  logic [3:0]  creq_lbe,
  input  logic [31:0] creq_d0,
  input  logic [31:0] creq_d1,
  input  logic [15:0] creq_rid,
  input  logic [7:0]  creq_tag,
  input  logic [2:0]  creq_tc,
  input  logic [1:0]  creq_attr,

  // completion to tlp_tx C port
  output logic        c_valid,
  input  logic        c_ready,
  output logic [15:0] c_rid,
  output logic [7:0]  c_tag,
  output logic [2:0]  c_tc,
  output logic [1:0]  c_attr,
  output logic [2:0]  c_status,
  output logic [6:0]  c_la,
  output logic [11:0] c_bc,
  output logic [1:0]  c_len,
  output logic [31:0] c_d0,
  output logic [31:0] c_d1,

  // to / from the controller core
  output logic [31:0] cc,
  output logic [31:0] aqa,
  output logic [63:0] asq,
  output logic [63:0] acq,
  input  logic        csts_rdy,
  output logic [(NQ+1)*16-1:0] sq_tail_db,
  output logic [(NQ+1)*16-1:0] cq_head_db,
  input  logic        db_clr,            // clear one queue's doorbell
  input  logic        db_clr_sq,         // 1: SQ tail, 0: CQ head
  input  logic [3:0]  db_clr_q,
  output logic [NV*64-1:0] msix_addr,
  output logic [NV*32-1:0] msix_data,
  output logic [NV-1:0]    msix_mask,
  input  logic [NV-1:0]    msix_pba
);

  localparam int Q = NQ + 1;

  // ---------------- registers ----------------
  logic [31:0] intms;
  logic [15:0] sq_db [Q];
  logic [15:0] cq_db [Q];
  logic [63:0] mx_addr [NV];
  logic [31:0] mx_data [NV];
  logic [NV-1:0] mx_mask;

  // CAP: MQES, CQR=1, AMS=0, TO=20 (10 s); DSTRD=0, NSSRS=0, CSS bit0 (NVM),
  //      MPSMIN=MPSMAX=0 (4 KB), CRMS.CRWMS=1 (required for NVMe 2.x controllers)
  localparam logic [31:0] CAP_LO = {8'd20, 5'd0, 2'b00, 1'b1, MQES};
  localparam logic [31:0] CAP_HI = {3'd0, 1'b0, 1'b1, 3'd0, 4'd0, 4'd0, 2'b00, 1'b0, 8'h01, 1'b0, 4'd0};
  localparam logic [31:0] VS     = 32'h0002_0400;   // NVMe 2.4
  // CRTO (0x68): CRWMT = CRIMT = 20 x 500 ms, same as CAP.TO (required since CAP.CRMS.CRWMS = 1)
  localparam logic [31:0] CRTO   = {16'd20, 16'd20};

  wire [31:0] csts = {28'd0, (cc[15:14] != 2'b00) ? 2'b10 : 2'b00, 1'b0, csts_rdy};

  // ---------------- register read ----------------
  function automatic logic [31:0] rd_dw(input logic [11:0] a);
    logic [31:0] r;
    r = 32'd0;
    if (a < 12'h400) begin
      case (a)
        12'h000: r = CAP_LO;
        12'h001: r = CAP_HI;
        12'h002: r = VS;
        12'h003: r = intms;
        12'h004: r = intms;
        12'h005: r = cc;
        12'h007: r = csts;
        12'h009: r = aqa;
        12'h00A: r = asq[31:0];
        12'h00B: r = asq[63:32];
        12'h00C: r = acq[31:0];
        12'h00D: r = acq[63:32];
        12'h01A: r = CRTO;
        default: r = 32'd0;
      endcase
    end else if (a >= 12'h800 && a < 12'h800 + 4*NV) begin
      int v;
      v = (a - 12'h800) >> 2;
      case (a[1:0])
        2'd0: r = mx_addr[v][31:0];
        2'd1: r = mx_addr[v][63:32];
        2'd2: r = mx_data[v];
        2'd3: r = {31'd0, mx_mask[v]};
      endcase
    end else if (a == 12'hC00) begin
      r = 32'(msix_pba);
    end
    return r;
  endfunction

  // completion byte count / lower address for <= 2 DW reads
  function automatic logic [1:0] first_set(input logic [3:0] be);
    if (be[0]) return 2'd0;
    if (be[1]) return 2'd1;
    if (be[2]) return 2'd2;
    if (be[3]) return 2'd3;
    return 2'd0;
  endfunction
  function automatic logic [1:0] last_set(input logic [3:0] be);
    if (be[3]) return 2'd3;
    if (be[2]) return 2'd2;
    if (be[1]) return 2'd1;
    return 2'd0;
  endfunction

  // ---------------- request handling ----------------
  // state: 0 = take request, 1 = completion pending
  logic cpl_pend;
  assign creq_ready = creq_valid && !cpl_pend;
  wire  req_fire = creq_valid && creq_ready;

  always_ff @(posedge clk) begin
    if (rst) begin
      cc       <= '0;
      aqa      <= '0;
      asq      <= '0;
      acq      <= '0;
      intms    <= '0;
      mx_mask  <= '1;              // vectors masked at reset (PCIe spec)
      cpl_pend <= 1'b0;
      c_valid  <= 1'b0;
      for (int q = 0; q < Q; q++) begin sq_db[q] <= '0; cq_db[q] <= '0; end
      for (int v = 0; v < NV; v++) begin mx_addr[v] <= '0; mx_data[v] <= '0; end
    end else begin
      // completion handshake
      if (c_valid && c_ready) begin
        c_valid  <= 1'b0;
        cpl_pend <= 1'b0;
      end

      // doorbells are cleared while the controller is disabled
      if (!cc[0]) begin
        for (int q = 0; q < Q; q++) begin sq_db[q] <= '0; cq_db[q] <= '0; end
      end
      if (db_clr && db_clr_q < Q) begin
        if (db_clr_sq) sq_db[db_clr_q] <= '0;
        else           cq_db[db_clr_q] <= '0;
      end

      if (req_fire) begin
        if (creq_wr) begin
          // up to two DWs
          for (int k = 0; k < 2; k++) begin
            logic [11:0] a;
            logic [31:0] d;
            logic [3:0]  be;
            logic        en;
            a  = creq_dwaddr + 12'(k);
            d  = (k == 0) ? creq_d0 : creq_d1;
            be = (k == 0) ? creq_fbe : creq_lbe;
            en = (k == 0) || (creq_len >= 11'd2);
            if (en && be != 4'd0) begin
              if (a < 12'h400) begin
                case (a)
                  12'h003: intms <= intms | d;            // INTMS (set)
                  12'h004: intms <= intms & ~d;           // INTMC (clear)
                  12'h005: cc    <= be_merge(cc, d, be);
                  12'h009: aqa   <= be_merge(aqa, d, be);
                  12'h00A: asq[31:0]  <= be_merge(asq[31:0], d, be);
                  12'h00B: asq[63:32] <= be_merge(asq[63:32], d, be);
                  12'h00C: acq[31:0]  <= be_merge(acq[31:0], d, be);
                  12'h00D: acq[63:32] <= be_merge(acq[63:32], d, be);
                  default: ;
                endcase
              end else if (a < 12'h400 + 2*Q) begin
                if (cc[0]) begin
                  if (a[0] == 1'b0) sq_db[(a - 12'h400) >> 1] <= d[15:0];
                  else              cq_db[(a - 12'h400) >> 1] <= d[15:0];
                end
              end else if (a >= 12'h800 && a < 12'h800 + 4*NV) begin
                case (a[1:0])
                  2'd0: mx_addr[(a - 12'h800) >> 2][31:0]  <= be_merge(mx_addr[(a - 12'h800) >> 2][31:0], d, be);
                  2'd1: mx_addr[(a - 12'h800) >> 2][63:32] <= be_merge(mx_addr[(a - 12'h800) >> 2][63:32], d, be);
                  2'd2: mx_data[(a - 12'h800) >> 2]        <= be_merge(mx_data[(a - 12'h800) >> 2], d, be);
                  2'd3: if (be[0]) mx_mask[(a - 12'h800) >> 2] <= d[0];
                endcase
              end
            end
          end
        end else begin
          // read: build completion
          cpl_pend <= 1'b1;
          c_valid  <= 1'b1;
          c_rid    <= creq_rid;
          c_tag    <= creq_tag;
          c_tc     <= creq_tc;
          c_attr   <= creq_attr;
          if (creq_len > 11'd2) begin
            c_status <= 3'b001;              // UR: only 1-2 DW register reads supported
            c_len    <= 2'd0;
            c_bc     <= 12'd4;
            c_la     <= {creq_dwaddr[4:0], 2'b00};
            c_d0     <= '0;
            c_d1     <= '0;
          end else begin
            c_status <= 3'b000;
            c_len    <= creq_len[1:0];
            c_la     <= {creq_dwaddr[4:0], first_set(creq_fbe)};
            c_d0     <= rd_dw(creq_dwaddr);
            c_d1     <= rd_dw(creq_dwaddr + 12'd1);
            if (creq_len == 11'd1)
              c_bc <= (creq_fbe == 4'd0) ? 12'd1 : 12'(last_set(creq_fbe) - first_set(creq_fbe) + 3'd1);
            else
              c_bc <= 12'd8 - 12'(first_set(creq_fbe)) - 12'(3 - last_set(creq_lbe));
          end
        end
      end
    end
  end

  // ---------------- outputs ----------------
  always_comb begin
    for (int q = 0; q < Q; q++) begin
      sq_tail_db[q*16 +: 16] = sq_db[q];
      cq_head_db[q*16 +: 16] = cq_db[q];
    end
    for (int v = 0; v < NV; v++) begin
      msix_addr[v*64 +: 64] = mx_addr[v];
      msix_data[v*32 +: 32] = mx_data[v];
    end
  end
  assign msix_mask = mx_mask;

endmodule
