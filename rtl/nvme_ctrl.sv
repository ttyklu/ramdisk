// nvme_ctrl.sv - NVMe command engine (one command at a time).
//  * Controller enable / disable / shutdown, admin queue setup from AQA/ASQ/ACQ
//  * Round-robin arbitration over the admin SQ and up to NQ I/O SQs
//  * Admin commands handled in hardware: Identify (CNS 0/1/2, others return zeros),
//    Get Log Page, Set/Get Features, Create/Delete I/O SQ/CQ, Abort, Async Event Request
//  * NVM commands: Read, Write, Flush; 4 KB LBAs, MDTS = 128 KB
//  * PRP1/PRP2 and PRP lists (including chained list pages)
//  * Completion queue entries with phase tag; MSI-X interrupts with mask / pending bits
module nvme_ctrl import nvme_pkg::*; import nvme_identify::*; #(
  parameter int          NQ        = 4,
  parameter int          NV        = 5,
  parameter logic [63:0] NSZE      = 64'd262144,   // namespace size in 4 KB blocks
  parameter logic [31:0] DDR_BASE  = 32'h0000_0000,
  parameter int          RAW       = 10,
  parameter int          FAW       = 10,
  parameter int          MAX_BLKS  = 32            // MDTS in 4 KB blocks
)(
  input  logic                   clk,
  input  logic                   rst,

  // registers
  input  logic [31:0]            cc,
  input  logic [31:0]            aqa,
  input  logic [63:0]            asq,
  input  logic [63:0]            acq,
  output logic                   csts_rdy,
  input  logic [(NQ+1)*16-1:0]   sq_tail_db,
  input  logic [(NQ+1)*16-1:0]   cq_head_db,
  output logic                   db_clr,
  output logic                   db_clr_sq,
  output logic [3:0]             db_clr_q,
  input  logic [NV*64-1:0]       msix_addr,
  input  logic [NV*32-1:0]       msix_data,
  input  logic [NV-1:0]          msix_mask,
  input  logic                   msix_en,
  input  logic                   msix_fm,
  output logic [NV-1:0]          msix_pba,

  // dma_rd (host -> device)
  output logic                   drd_job_valid,
  input  logic                   drd_job_ready,
  output logic [63:0]            drd_job_addr,
  output logic [11:0]            drd_job_len,
  input  logic [RAW:0]           drd_avail,
  input  logic [31:0]            drd_head0,
  input  logic [31:0]            drd_head1,
  output logic [1:0]             drd_pop,
  input  logic                   drd_idle,

  // mwr_eng (device -> host)
  output logic                   mwr_job_valid,
  input  logic                   mwr_job_ready,
  output logic [63:0]            mwr_job_addr,
  output logic [11:0]            mwr_job_len,
  input  logic                   mwr_idle,

  // TX DW FIFO push (used when ddr_rd is not pushing)
  output logic [1:0]             txf_push,
  output logic [31:0]            txf_dw0,
  output logic [31:0]            txf_dw1,
  input  logic [FAW:0]           txf_free,

  // DDR engines
  output logic                   ddrr_job_valid,
  input  logic                   ddrr_job_ready,
  input  logic                   ddrr_busy,
  output logic                   ddrw_job_valid,
  input  logic                   ddrw_job_ready,
  input  logic                   ddrw_busy,
  output logic [31:0]            ddr_job_addr,
  output logic [17:0]            ddr_job_len,
  output logic                   data_to_ddr,     // ddr_wr owns the dma_rd pop port
  output logic                   txf_flush        // empty the TX FIFO (controller disabled)
);

  localparam int Q  = NQ + 1;
  localparam int NP = MAX_BLKS + 1;               // max PRP entries (incl. PRP1)

  typedef enum logic [5:0] {
    S_OFF, S_DISABLE, S_IDLE,
    S_FETCH, S_FETCH_RD, S_DECODE,
    S_PRP, S_PRPL_CALC, S_PRPL_FETCH, S_PRPL_RD,
    S_DATA_START, S_DATA_CHUNK, S_DATA_WAIT,
    S_CQE_ROOM, S_CQE_PUSH0, S_CQE_PUSH1, S_CQE_JOB, S_CQE_WAIT,
    S_INT, S_INT_PUSH, S_INT_JOB, S_INT_WAIT
  } st_t;
  st_t st;

  typedef enum logic [1:0] {SRC_DDR, SRC_ID, SRC_LOG} src_t;

  // ---------------- queue state ----------------
  logic [63:0] sq_base [Q];
  logic [15:0] sq_size [Q];      // 0's based
  logic [15:0] sq_head [Q];
  logic [2:0]  sq_cqid [Q];
  logic [Q-1:0] sq_valid;
  logic [63:0] cq_base [Q];
  logic [15:0] cq_size [Q];
  logic [15:0] cq_tail [Q];
  logic [2:0]  cq_iv   [Q];
  logic [Q-1:0] cq_phase, cq_ien, cq_valid;

  logic [2:0]  rr_last;
  logic [3:0]  aer_cnt;
  logic [NV-1:0] pba;
  assign msix_pba = pba;

  // ---------------- current command ----------------
  logic [31:0] cmd [16];
  logic [3:0]  ck;               // fetch DW index
  logic [2:0]  cur_q;
  logic [31:0] res_dw0;
  logic [10:0] status;           // {SCT, SC}

  wire [7:0]  opc   = cmd[0][7:0];
  wire [15:0] cid   = cmd[0][31:16];
  wire [31:0] nsid  = cmd[1];
  wire [63:0] prp1  = {cmd[7], cmd[6]};
  wire [63:0] prp2  = {cmd[9], cmd[8]};
  wire [31:0] cdw10 = cmd[10];
  wire [31:0] cdw11 = cmd[11];
  wire [31:0] cdw12 = cmd[12];

  // ---------------- transfer state ----------------
  logic        to_host;
  src_t        src;
  logic [7:0]  id_cns;
  logic [7:0]  log_id;
  logic [17:0] xfer;             // bytes
  logic [17:0] remaining;
  logic [31:0] ddr_addr;
  logic [63:0] prp [NP];
  logic [5:0]  prp_wi;           // next PRP slot to fill
  logic [5:0]  pi;               // current PRP slot in data phase
  logic [63:0] cur_addr;
  logic [63:0] list_addr;
  logic [5:0]  need;             // list entries still to read
  logic [9:0]  ent_cnt, ent_i;
  logic        chain;

  // generator (Identify / log data into TX FIFO)
  logic        gen_active;
  logic [10:0] gen_idx;          // DW index
  logic [10:0] gen_total;

  // interrupt
  logic [2:0]  int_v;

  // ---------------- helpers ----------------
  function automatic logic [15:0] qwrap(input logic [15:0] v, input logic [15:0] sz);
    return (v == sz) ? 16'd0 : v + 16'd1;
  endfunction

  function automatic logic [31:0] gen_dw(input src_t s, input logic [7:0] cns,
                                         input logic [7:0] lid, input logic [10:0] i);
    logic [31:0] r;
    r = 32'd0;
    if (s == SRC_ID) begin
      case (cns)
        8'h00: r = id_ns_dw(i[9:0], NSZE);
        8'h01: r = id_ctrl_dw(i[9:0]);
        8'h02: r = (i == 11'd0) ? 32'd1 : 32'd0;     // active NS list: NSID 1
        default: r = 32'd0;
      endcase
    end else if (s == SRC_LOG) begin
      if (lid == 8'h02) begin                         // SMART / health
        if (i == 11'd0) r = 32'h6401_2C00;            // temp 300 K, spare 100%
        else if (i == 11'd1) r = 32'h0000_000A;       // spare threshold 10%
      end
    end
    return r;
  endfunction

  // ---------------- arbitration ----------------
  logic       have_q;
  logic [2:0] next_q;
  always_comb begin
    have_q = 1'b0;
    next_q = 3'd0;
    for (int k = 1; k <= Q; k++) begin
      int q;
      q = (int'(rr_last) + k) % Q;
      if (!have_q && sq_valid[q] && (sq_head[q] != sq_tail_db[q*16 +: 16])) begin
        have_q = 1'b1;
        next_q = 3'(q);
      end
    end
  end

  // pending interrupt to service
  logic       have_pend;
  logic [2:0] pend_v;
  always_comb begin
    have_pend = 1'b0;
    pend_v    = 3'd0;
    for (int v = 0; v < NV; v++) begin
      if (!have_pend && pba[v] && !msix_mask[v]) begin
        have_pend = 1'b1;
        pend_v    = 3'(v);
      end
    end
    if (!msix_en || msix_fm) have_pend = 1'b0;
  end

  // ---------------- data-phase chunk ----------------
  wire [12:0] page_room = 13'd4096 - {1'b0, cur_addr[11:0]};
  wire [12:0] chunk     = ({1'b0, remaining} < {5'd0, page_room}) ? remaining[12:0] : page_room;

  // ---------------- PRP setup values ----------------
  wire [12:0] first_len = 13'd4096 - {1'b0, prp1[11:0]};
  wire [17:0] rem2      = xfer - {5'd0, first_len};
  wire [9:0]  list_room = 10'((13'd4096 - {1'b0, list_addr[11:0]}) >> 3);

  // ---------------- CQE ----------------
  wire [2:0]  cqid      = sq_cqid[cur_q];
  wire [15:0] cq_next   = qwrap(cq_tail[cqid], cq_size[cqid]);
  wire [31:0] cqe_dw2   = {13'd0, cur_q, sq_head[cur_q]};
  wire        st_err    = (status != ST_SUCCESS);
  wire [31:0] cqe_dw3   = {st_err, 1'b0, 2'b00, status, cq_phase[cqid], cid};

  // ---------------- outputs (defaults) ----------------
  always_comb begin
    drd_job_valid  = 1'b0;
    drd_job_addr   = 64'd0;
    drd_job_len    = 12'd0;
    drd_pop        = 2'd0;
    mwr_job_valid  = 1'b0;
    mwr_job_addr   = 64'd0;
    mwr_job_len    = 12'd0;
    txf_push       = 2'd0;
    txf_dw0        = 32'd0;
    txf_dw1        = 32'd0;
    ddrr_job_valid = 1'b0;
    ddrw_job_valid = 1'b0;
    ddr_job_addr   = ddr_addr;
    ddr_job_len    = xfer;

    case (st)
      S_FETCH: begin
        drd_job_valid = 1'b1;
        drd_job_addr  = sq_base[cur_q] + {42'd0, sq_head[cur_q], 6'd0};
        drd_job_len   = 12'd16;
      end
      S_FETCH_RD: begin
        if (drd_avail >= 2) drd_pop = 2'd2;
      end
      S_PRPL_FETCH: begin
        drd_job_valid = 1'b1;
        drd_job_addr  = list_addr;
        drd_job_len   = {1'b0, ent_cnt, 1'b0};        // 2 DWs per entry
      end
      S_PRPL_RD: begin
        if (drd_avail >= 2) drd_pop = 2'd2;
      end
      S_DISABLE: begin
        // drain anything left in the DMA ring so it is not read as a command later
        if (drd_avail >= 2)      drd_pop = 2'd2;
        else if (drd_avail == 1) drd_pop = 2'd1;
      end
      S_DATA_START: begin
        if (to_host && src == SRC_DDR) ddrr_job_valid = 1'b1;
        if (!to_host)                  ddrw_job_valid = 1'b1;
      end
      S_DATA_CHUNK: begin
        if (to_host) begin
          mwr_job_valid = 1'b1;
          mwr_job_addr  = cur_addr;
          mwr_job_len   = 12'(chunk >> 2);
        end else begin
          drd_job_valid = 1'b1;
          drd_job_addr  = cur_addr;
          drd_job_len   = 12'(chunk >> 2);
        end
      end
      S_CQE_PUSH0: begin
        if (txf_free >= 2) begin txf_push = 2'd2; txf_dw0 = res_dw0; txf_dw1 = 32'd0; end
      end
      S_CQE_PUSH1: begin
        if (txf_free >= 2) begin txf_push = 2'd2; txf_dw0 = cqe_dw2; txf_dw1 = cqe_dw3; end
      end
      S_CQE_JOB: begin
        mwr_job_valid = 1'b1;
        mwr_job_addr  = cq_base[cqid] + {44'd0, cq_tail[cqid], 4'd0};
        mwr_job_len   = 12'd4;
      end
      S_INT_PUSH: begin
        if (txf_free >= 1) begin txf_push = 2'd1; txf_dw0 = msix_data[int_v*32 +: 32]; end
      end
      S_INT_JOB: begin
        mwr_job_valid = 1'b1;
        mwr_job_addr  = msix_addr[int_v*64 +: 64];
        mwr_job_len   = 12'd1;
      end
      default: ;
    endcase

    // Identify / log generator runs alongside S_DATA_CHUNK / S_DATA_WAIT
    if (gen_active && txf_free >= 2) begin
      if (gen_total - gen_idx >= 11'd2) begin
        txf_push = 2'd2;
        txf_dw0  = gen_dw(src, id_cns, log_id, gen_idx);
        txf_dw1  = gen_dw(src, id_cns, log_id, gen_idx + 11'd1);
      end else begin
        txf_push = 2'd1;
        txf_dw0  = gen_dw(src, id_cns, log_id, gen_idx);
      end
    end
  end

  assign data_to_ddr = !to_host && (st == S_DATA_START || st == S_DATA_CHUNK || st == S_DATA_WAIT);
  assign txf_flush   = (st == S_OFF);

  // ---------------- main FSM ----------------
  always_ff @(posedge clk) begin
    if (rst) begin
      st         <= S_OFF;
      csts_rdy   <= 1'b0;
      sq_valid   <= '0;
      cq_valid   <= '0;
      cq_phase   <= '0;
      cq_ien     <= '0;
      rr_last    <= '0;
      aer_cnt    <= '0;
      pba        <= '0;
      gen_active <= 1'b0;
      db_clr     <= 1'b0;
      to_host    <= 1'b0;
    end else begin
      db_clr <= 1'b0;

      // generator progress
      if (gen_active && txf_free >= 2) begin
        if (gen_total - gen_idx >= 11'd2) gen_idx <= gen_idx + 11'd2;
        else                              gen_idx <= gen_idx + 11'd1;
        if (gen_total - gen_idx <= 11'd2) gen_active <= 1'b0;
      end

      case (st)
        // ---------------------------------------------------------------
        S_OFF: begin
          csts_rdy <= 1'b0;
          if (cc[0]) begin
            sq_base[0]  <= {asq[63:12], 12'd0};
            sq_size[0]  <= {4'd0, aqa[11:0]};
            sq_head[0]  <= '0;
            sq_cqid[0]  <= 3'd0;
            cq_base[0]  <= {acq[63:12], 12'd0};
            cq_size[0]  <= {4'd0, aqa[27:16]};
            cq_tail[0]  <= '0;
            cq_iv[0]    <= 3'd0;
            sq_valid    <= Q'(1);
            cq_valid    <= Q'(1);
            cq_phase    <= Q'(1);
            cq_ien      <= Q'(1);
            pba         <= '0;
            aer_cnt     <= '0;
            rr_last     <= '0;
            csts_rdy    <= 1'b1;
            st          <= S_IDLE;
          end
        end

        S_DISABLE: begin
          if (drd_idle && mwr_idle && !ddrr_busy && !ddrw_busy && !gen_active && drd_avail == '0) begin
            sq_valid <= '0;
            cq_valid <= '0;
            csts_rdy <= 1'b0;
            st       <= S_OFF;
          end
        end

        // ---------------------------------------------------------------
        S_IDLE: begin
          if (!cc[0]) begin
            st <= S_DISABLE;
          end else if (have_pend) begin
            int_v       <= pend_v;
            pba[pend_v] <= 1'b0;
            st          <= S_INT_PUSH;
          end else if (have_q) begin
            cur_q   <= next_q;
            rr_last <= next_q;
            st      <= S_FETCH;
          end
        end

        S_FETCH: begin
          if (drd_job_ready) begin
            sq_head[cur_q] <= qwrap(sq_head[cur_q], sq_size[cur_q]);
            ck             <= 4'd0;
            st             <= S_FETCH_RD;
          end
        end

        S_FETCH_RD: begin
          if (drd_avail >= 2) begin
            cmd[ck]          <= drd_head0;
            cmd[ck + 4'd1]   <= drd_head1;
            ck               <= ck + 4'd2;
            if (ck == 4'd14) st <= S_DECODE;
          end
        end

        // ---------------------------------------------------------------
        S_DECODE: begin
          res_dw0 <= 32'd0;
          status  <= ST_SUCCESS;
          st      <= S_CQE_ROOM;          // default: no data phase

          if (cur_q == 3'd0) begin
            // ======================= admin =======================
            case (opc)
              8'h06: begin  // Identify
                id_cns  <= cdw10[7:0];
                src     <= SRC_ID;
                to_host <= 1'b1;
                xfer    <= 18'd4096;
                st      <= S_PRP;
              end
              8'h02: begin  // Get Log Page
                logic [31:0] numd;
                numd = {cdw11[15:0], cdw10[31:16]} + 32'd1;
                log_id  <= cdw10[7:0];
                src     <= SRC_LOG;
                to_host <= 1'b1;
                xfer    <= (numd > 32'd1024) ? 18'd4096 : 18'({numd[10:0], 2'b00});
                st      <= S_PRP;
              end
              8'h09, 8'h0A: begin  // Set / Get Features
                if (cdw10[7:0] == 8'h07) res_dw0 <= {16'(NQ - 1), 16'(NQ - 1)};
              end
              8'h05: begin  // Create I/O CQ
                logic [15:0] qid, qs, iv;
                qid = cdw10[15:0]; qs = cdw10[31:16]; iv = cdw11[31:16];
                if (qid == 16'd0 || qid > 16'(NQ) || cq_valid[qid[2:0]])
                  status <= ST_INVALID_QID;
                else if (qs == 16'd0 || qs > 16'd4095)
                  status <= ST_INVALID_QSIZE;
                else if (!cdw11[0])
                  status <= ST_INVALID_FIELD;          // must be physically contiguous
                else if (cdw11[1] && iv >= 16'(NV))
                  status <= ST_INVALID_IV;
                else begin
                  cq_base [qid[2:0]] <= {prp1[63:2], 2'b00};
                  cq_size [qid[2:0]] <= qs;
                  cq_tail [qid[2:0]] <= '0;
                  cq_iv   [qid[2:0]] <= iv[2:0];
                  cq_ien  [qid[2:0]] <= cdw11[1];
                  cq_phase[qid[2:0]] <= 1'b1;
                  cq_valid[qid[2:0]] <= 1'b1;
                  db_clr    <= 1'b1; db_clr_sq <= 1'b0; db_clr_q <= {1'b0, qid[2:0]};
                end
              end
              8'h01: begin  // Create I/O SQ
                logic [15:0] qid, qs, cq;
                qid = cdw10[15:0]; qs = cdw10[31:16]; cq = cdw11[31:16];
                if (qid == 16'd0 || qid > 16'(NQ) || sq_valid[qid[2:0]])
                  status <= ST_INVALID_QID;
                else if (qs == 16'd0 || qs > 16'd4095)
                  status <= ST_INVALID_QSIZE;
                else if (!cdw11[0])
                  status <= ST_INVALID_FIELD;
                else if (cq == 16'd0 || cq > 16'(NQ) || !cq_valid[cq[2:0]])
                  status <= ST_CQ_INVALID;
                else begin
                  sq_base [qid[2:0]] <= {prp1[63:2], 2'b00};
                  sq_size [qid[2:0]] <= qs;
                  sq_head [qid[2:0]] <= '0;
                  sq_cqid [qid[2:0]] <= cq[2:0];
                  sq_valid[qid[2:0]] <= 1'b1;
                  db_clr    <= 1'b1; db_clr_sq <= 1'b1; db_clr_q <= {1'b0, qid[2:0]};
                end
              end
              8'h00: begin  // Delete I/O SQ
                logic [15:0] qid;
                qid = cdw10[15:0];
                if (qid == 16'd0 || qid > 16'(NQ) || !sq_valid[qid[2:0]])
                  status <= ST_INVALID_QID;
                else
                  sq_valid[qid[2:0]] <= 1'b0;
              end
              8'h04: begin  // Delete I/O CQ
                logic [15:0] qid;
                logic        used;
                qid  = cdw10[15:0];
                used = 1'b0;
                for (int q = 1; q < Q; q++)
                  if (sq_valid[q] && sq_cqid[q] == qid[2:0]) used = 1'b1;
                if (qid == 16'd0 || qid > 16'(NQ) || !cq_valid[qid[2:0]])
                  status <= ST_INVALID_QID;
                else if (used)
                  status <= ST_INVALID_QDEL;
                else
                  cq_valid[qid[2:0]] <= 1'b0;
              end
              8'h08: begin  // Abort: command not aborted
                res_dw0 <= 32'd1;
              end
              8'h0C: begin  // Asynchronous Event Request: held, never completed
                if (aer_cnt >= 4'd4) status <= ST_AER_LIMIT;
                else begin
                  aer_cnt <= aer_cnt + 4'd1;
                  st      <= S_IDLE;
                end
              end
              default: status <= ST_INVALID_OPCODE;
            endcase
          end else begin
            // ======================= NVM I/O =======================
            case (opc)
              8'h00: begin  // Flush
                if (nsid != 32'd1 && nsid != 32'hFFFF_FFFF) status <= ST_INVALID_NS;
              end
              8'h01, 8'h02: begin  // Write / Read
                logic [63:0] slba;
                logic [16:0] nblk;
                slba = {cdw11, cdw10};
                nblk = {1'b0, cdw12[15:0]} + 17'd1;
                if (nsid != 32'd1)
                  status <= ST_INVALID_NS;
                else if (nblk > 17'(MAX_BLKS))
                  status <= ST_INVALID_FIELD;
                else if (slba >= NSZE || (NSZE - slba) < {47'd0, nblk})
                  status <= ST_LBA_RANGE;
                else if (prp1[1:0] != 2'b00)
                  status <= ST_PRP_OFFSET;
                else begin
                  to_host  <= (opc == 8'h02);
                  src      <= SRC_DDR;
                  xfer     <= {nblk[5:0], 12'd0};
                  ddr_addr <= DDR_BASE + {slba[19:0], 12'd0};
                  st       <= S_PRP;
                end
              end
              default: status <= ST_INVALID_OPCODE;
            endcase
          end
        end

        // ---------------------------------------------------------------
        S_PRP: begin
          prp[0] <= {prp1[63:2], 2'b00};
          if ({1'b0, xfer} <= {6'd0, first_len}) begin
            st <= S_DATA_START;
          end else if (rem2 <= 18'd4096) begin
            prp[1] <= {prp2[63:12], 12'd0};
            st     <= S_DATA_START;
          end else begin
            need      <= 6'((rem2 + 18'd4095) >> 12);
            list_addr <= {prp2[63:3], 3'b000};
            prp_wi    <= 6'd1;
            st        <= S_PRPL_CALC;
          end
        end

        S_PRPL_CALC: begin
          // entries to read from the current list page; if more are needed than fit,
          // the last entry of the page is a pointer to the next list page
          if ({4'd0, need} > list_room) begin
            ent_cnt <= list_room;
            chain   <= 1'b1;
          end else begin
            ent_cnt <= {4'd0, need};
            chain   <= 1'b0;
          end
          st <= S_PRPL_FETCH;
        end

        S_PRPL_FETCH: begin
          if (drd_job_ready) begin
            ent_i <= '0;
            st    <= S_PRPL_RD;
          end
        end

        S_PRPL_RD: begin
          if (drd_avail >= 2) begin
            if (chain && ent_i == ent_cnt - 10'd1) begin
              list_addr <= {drd_head1, drd_head0[31:3], 3'b000};
            end else begin
              prp[prp_wi] <= {drd_head1, drd_head0[31:12], 12'd0};
              prp_wi      <= prp_wi + 6'd1;
              need        <= need - 6'd1;
            end
            ent_i <= ent_i + 10'd1;
            if (ent_i == ent_cnt - 10'd1)
              st <= chain ? S_PRPL_CALC : S_DATA_START;
          end
        end

        // ---------------------------------------------------------------
        S_DATA_START: begin
          remaining <= xfer;
          cur_addr  <= prp[0];
          pi        <= 6'd0;
          if (to_host && src == SRC_DDR) begin
            if (ddrr_job_ready) st <= S_DATA_CHUNK;
          end else if (!to_host) begin
            if (ddrw_job_ready) st <= S_DATA_CHUNK;
          end else begin
            gen_active <= 1'b1;
            gen_idx    <= '0;
            gen_total  <= 11'(xfer >> 2);
            st         <= S_DATA_CHUNK;
          end
        end

        S_DATA_CHUNK: begin
          if ((to_host && mwr_job_ready) || (!to_host && drd_job_ready)) begin
            remaining <= remaining - {5'd0, chunk};
            pi        <= pi + 6'd1;
            cur_addr  <= prp[pi + 6'd1];
            if (remaining == {5'd0, chunk}) st <= S_DATA_WAIT;
          end
        end

        S_DATA_WAIT: begin
          if (to_host) begin
            if (mwr_idle && !ddrr_busy && !gen_active) st <= S_CQE_ROOM;
          end else begin
            if (drd_idle && !ddrw_busy && drd_avail == '0) st <= S_CQE_ROOM;
          end
        end

        // ---------------------------------------------------------------
        S_CQE_ROOM: begin
          if (!cc[0]) st <= S_DISABLE;                    // controller reset: drop the CQE
          else if (cq_next != cq_head_db[cqid*16 +: 16]) st <= S_CQE_PUSH0;
        end
        S_CQE_PUSH0: if (txf_free >= 2) st <= S_CQE_PUSH1;
        S_CQE_PUSH1: if (txf_free >= 2) st <= S_CQE_JOB;
        S_CQE_JOB: begin
          if (mwr_job_ready) begin
            cq_tail[cqid] <= cq_next;
            if (cq_next == 16'd0) cq_phase[cqid] <= ~cq_phase[cqid];
            st <= S_CQE_WAIT;
          end
        end
        S_CQE_WAIT: if (mwr_idle) st <= S_INT;

        S_INT: begin
          if (!cq_ien[cqid]) begin
            st <= S_IDLE;
          end else if (msix_en && !msix_fm && !msix_mask[cq_iv[cqid]]) begin
            int_v <= cq_iv[cqid];
            st    <= S_INT_PUSH;
          end else begin
            pba[cq_iv[cqid]] <= 1'b1;
            st <= S_IDLE;
          end
        end
        S_INT_PUSH: if (txf_free >= 1) st <= S_INT_JOB;
        S_INT_JOB:  if (mwr_job_ready) st <= S_INT_WAIT;
        S_INT_WAIT: if (mwr_idle) st <= S_IDLE;

        default: st <= S_OFF;
      endcase
    end
  end

endmodule
