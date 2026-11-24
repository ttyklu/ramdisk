// nvme_ctrl.sv - NVMe command engine (one command at a time).
//  * Controller enable / disable / shutdown, admin queue setup from AQA/ASQ/ACQ
//  * Round-robin arbitration over the admin SQ and up to NQ I/O SQs
//  * Admin commands handled in hardware: Identify (CNS 0/1/2/8, others return zeros),
//    Get Log Page, Set/Get Features, Create/Delete I/O SQ/CQ, Abort, Async Event Request
//  * NVM commands: Read, Write, Flush; 4 KB LBAs, MDTS = 2 x largest record
//  * Compression (docs/compression_spec.md): Set/Get Features C0h (RAW by default),
//    Compress-to-Staging 81h, Commit 84h, Decompress-Read 86h, FFE-Send 89h, FFE-Receive 8Ah,
//    staging slots in DDR after the namespace, work done by the xform engine
//  * PRP1/PRP2 and PRP lists (including chained list pages)
//  * Completion queue entries with phase tag; MSI-X interrupts with mask / pending bits
module nvme_ctrl import nvme_pkg::*; import nvme_identify::*; #(
  parameter int          NQ        = 4,
  parameter int          NV        = 5,
  parameter logic [63:0] NSZE      = 64'd262144,   // namespace size in 4 KB blocks
  parameter logic [31:0] DDR_BASE  = 32'h0000_0000,
  parameter int          RAW       = 10,
  parameter int          FAW       = 10,
  parameter int          REC_LOG2  = 17,           // largest record: 17 = 128 KB ... 20 = 1 MB
  parameter int          MAX_BLKS  = (2 << REC_LOG2) / 4096,   // MDTS in 4 KB blocks
  parameter int          NSLOT     = 128,          // staging slots (power of 2)
  parameter logic [31:0] RING_BASE = 32'h3C00_0000 // first staging slot
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
  output logic [23:0]            ddr_job_len,
  output logic                   data_to_ddr,     // ddr_wr owns the dma_rd pop port
  output logic                   txf_flush,       // empty the TX FIFO (controller disabled)

  // xform engine
  output logic                   xf_start,
  output logic [1:0]             xf_func,
  output logic [31:0]            xf_rd_addr,
  output logic [23:0]            xf_rd_len,
  output logic [4:0]             xf_skip,
  output logic [23:0]            xf_in_len,
  output logic [23:0]            xf_ck_len,
  output logic                   xf_dst_tx,
  output logic [31:0]            xf_wr_addr,
  output logic [23:0]            xf_out_lim,
  input  logic                   xf_busy,
  input  logic                   xf_done,
  input  logic [23:0]            xf_out_len,
  input  logic [23:0]            xf_wr_bytes,
  input  logic                   xf_over,
  input  logic [255:0]           xf_fi,          // {d, c, b, a}
  input  logic [255:0]           xf_fo,
  input  logic [31:0]            xf_adler,
  input  logic [3:0]             xf_ierr,
  input  logic [23:0]            xf_consumed,

  output logic [5:0]             dbg_st          // FSM state, for the debug register
);

  localparam int Q  = NQ + 1;
  localparam int NP = MAX_BLKS + 1;               // max PRP entries (incl. PRP1)

  typedef enum logic [5:0] {
    S_OFF, S_DISABLE, S_IDLE,
    S_FETCH, S_FETCH_RD, S_DECODE,
    S_PRP, S_PRPL_CALC, S_PRPL_FETCH, S_PRPL_RD,
    S_DATA_START, S_DATA_CHUNK, S_DATA_WAIT,
    S_CQE_ROOM, S_CQE_PUSH0, S_CQE_PUSH1, S_CQE_JOB, S_CQE_WAIT,
    S_INT, S_INT_PUSH, S_INT_JOB, S_INT_WAIT,
    S_POST, S_XF_WAIT, S_C2S_FIN, S_DEC_FIN, S_SLOT_WR, S_RES_GEN, S_RES_JOB, S_RES_WAIT,
    S_SLOT_RD, S_SLOT_RD2, S_COMMIT_CHK, S_RECV_CHK, S_FREE_CQE,
    S_MD_FETCH, S_MD_RD, S_DREAD_GO, S_DESC_FETCH, S_DESC_RD, S_DESC_PARSE, S_FFE_CHK, S_FFE_DEST
  } st_t;
  st_t st;

  typedef enum logic [2:0] {SRC_DDR, SRC_ID, SRC_LOG, SRC_CRB, SRC_XF} src_t;

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
  logic [23:0] xfer;             // bytes
  logic [23:0] remaining;
  logic [23:0] pxfer;            // buffer length the PRPs describe, if larger than xfer (0 = xfer)
  logic [31:0] ddr_addr;
  logic [63:0] prp [NP];
  logic [9:0]  prp_wi;           // next PRP slot to fill
  logic [9:0]  pi;               // current PRP slot in data phase
  logic [63:0] cur_addr;
  logic [63:0] list_addr;
  logic [9:0]  need;             // list entries still to read
  logic [9:0]  ent_cnt, ent_i;
  logic        chain;

  // generator (Identify / log data into TX FIFO)
  logic        gen_active;
  logic [10:0] gen_idx;          // DW index
  logic [10:0] gen_total;

  // interrupt
  logic [2:0]  int_v;

  // ---------------- compression ----------------
  localparam int          SLOT_LOG2 = REC_LOG2 + 2;               // slot = 4 x record
  localparam int          SW        = (NSLOT > 1) ? $clog2(NSLOT) : 1;
  localparam logic [3:0]  MAXRS     = 4'(REC_LOG2 - 17);
  localparam logic [7:0]  MDTS_V    = 8'($clog2(MAX_BLKS));
  localparam logic [23:0] MAX_REC   = 24'(1) << REC_LOG2;

  typedef enum logic [2:0] {K_NONE, K_C2S, K_COMMIT, K_DREAD, K_FSEND, K_FRECV} kind_t;

  logic             cen;                     // compression commands enabled (RAW when 0)
  logic [3:0]       recsz;                   // record size limit = 128 KB << recsz
  wire  [23:0]      rec_lim = 24'h02_0000 << recsz;
  logic [NSLOT-1:0] slot_busy, slot_ffe;
  logic [15:0]      slot_lid [NSLOT];
  logic [511:0]     slot_mem [NSLOT];        // CRB per slot (block RAM)
  logic [511:0]     srec, srec_raw;      // block RAM output, then a flop stage
  logic [SW-1:0]    cur_slot;
  logic [31:0]      crb [16];
  kind_t            kind;
  logic [23:0]      k_len;                   // record / payload bytes
  logic [7:0]       k_thr;                   // threshold percent (0 = ZFS 12.5% rule)
  logic [63:0]      k_slba;
  logic [23:0]      k_psz, k_csize, k_ulen;
  logic             k_nochk, k_discard;
  logic [31:0]      dsc [128];               // FFE descriptors / MPTR descriptor
  logic [7:0]       dsc_n, dsc_i;
  logic [7:0]       dwi;
  logic [7:0]       f_fn;                    // 82h / 83h
  logic             f_has_fd, f_has_fpd, f_dlba, f_bad;
  logic [63:0]      f_dslba;
  logic [15:0]      f_dnlb;
  logic [7:0]       f_srci;
  logic [31:0]      f_fpd1, f_fpd2, f_fpd3;
  logic [23:0]      roff;                    // retained input offset in the raw area
  logic             xf_pend;
  st_t              xf_next;
  logic             f_lbasrc, f_cmt;
  logic [15:0]      k_nlb;

  wire [63:0] mptr = {cmd[5], cmd[4]};
  wire [31:0] cdw13 = cmd[13];
  wire [31:0] cdw14 = cmd[14];
  wire [31:0] cdw15 = cmd[15];

  function automatic logic [31:0] slot_raw(input logic [SW-1:0] sl);
    return RING_BASE + (32'(sl) << SLOT_LOG2);
  endfunction
  function automatic logic [31:0] slot_out(input logic [SW-1:0] sl);
    return slot_raw(sl) + (32'd1 << (SLOT_LOG2 - 1));
  endfunction
  function automatic logic [23:0] up4k(input logic [23:0] v);
    return (v + 24'hFFF) & 24'hFF_F000;
  endfunction
  function automatic logic [23:0] up128(input logic [23:0] v);
    return (v + 24'h7F) & 24'hFF_FF80;
  endfunction

  // first free slot, and the busy FFE slot holding this command's List ID. Registered: slot
  // state and CDW2 settle many cycles before the command is decoded, so one cycle of delay
  // is always safe.
  logic          have_free, have_lid;
  logic [SW-1:0] free_idx, lid_idx;
  always_ff @(posedge clk) begin
    logic          hf, hl;
    logic [SW-1:0] fi, li_;
    hf = 1'b0; fi = '0;
    hl = 1'b0; li_ = '0;
    for (int k = NSLOT - 1; k >= 0; k--) begin
      if (!slot_busy[k]) begin hf = 1'b1; fi = SW'(k); end
      if (slot_busy[k] && slot_ffe[k] && slot_lid[k] == cmd[2][15:0]) begin hl = 1'b1; li_ = SW'(k); end
    end
    have_free <= hf; free_idx <= fi;
    have_lid  <= hl; lid_idx  <= li_;
  end

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
        8'h01: begin
          r = id_ctrl_dw(i[9:0]);
          if (i == 11'd19) r[15:8] = MDTS_V;          // byte 77: MDTS from the build
        end
        8'h02: r = (i == 11'd0) ? 32'd1 : 32'd0;     // active NS list: NSID 1
        // I/O command set independent Identify NS (Linux uses it when VS >= 2.0):
        // byte 14 NSTAT.NRDY = 1, everything else zero (not shared, not read-only)
        8'h08: r = (i == 11'd3) ? 32'h0001_0000 : 32'd0;
        default: r = 32'd0;
      endcase
    end else if (s == SRC_CRB) begin
      if (i < 11'd14) r = crb[i[3:0]];               // DW 14-15 are internal
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
  wire [12:0] chunk     = (remaining < 24'(page_room)) ? remaining[12:0] : page_room;

  // ---------------- PRP setup values ----------------
  wire [12:0] first_len = 13'd4096 - {1'b0, prp1[11:0]};
  wire [23:0] plen      = (pxfer > xfer) ? pxfer : xfer;  // PRP layout follows the buffer length
  wire [23:0] rem2      = plen - 24'(first_len);
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
      S_MD_FETCH: begin
        drd_job_valid = 1'b1;
        drd_job_addr  = {mptr[63:2], 2'b00};
        drd_job_len   = 12'd16;
      end
      S_DESC_FETCH: begin
        drd_job_valid = 1'b1;
        drd_job_addr  = {prp1[63:2], 2'b00};
        drd_job_len   = 12'({dsc_n, 4'd0});
      end
      S_MD_RD, S_DESC_RD: begin
        if (drd_avail >= 2) drd_pop = 2'd2;
      end
      S_RES_JOB: begin
        mwr_job_valid = 1'b1;
        mwr_job_addr  = {mptr[63:2], 2'b00};
        mwr_job_len   = 12'd16;
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

  // ---------------- xform job setup ----------------
  task automatic set_xf(input logic [1:0] fn, input logic [31:0] ra, input logic [23:0] rl,
                        input logic [4:0] sk, input logic [23:0] il, input logic [23:0] cl,
                        input logic tx, input logic [31:0] wa, input logic [23:0] lim);
    xf_func    <= fn;
    xf_rd_addr <= ra;
    xf_rd_len  <= rl;
    xf_skip    <= sk;
    xf_in_len  <= il;
    xf_ck_len  <= cl;
    xf_dst_tx  <= tx;
    xf_wr_addr <= wa;
    xf_out_lim <= lim;
  endtask

  task automatic start_xf(input logic [1:0] fn, input logic [31:0] ra, input logic [23:0] rl,
                          input logic [4:0] sk, input logic [23:0] il, input logic [23:0] cl,
                          input logic tx, input logic [31:0] wa, input logic [23:0] lim, input st_t nxt);
    set_xf(fn, ra, rl, sk, il, cl, tx, wa, lim);
    xf_start <= 1'b1;
    xf_next  <= nxt;
    st       <= S_XF_WAIT;
  endtask

  // copy the stored bytes of the current slot (fields of a CRB): il bytes to wa / the TX FIFO
  task automatic copy_slot(input logic [511:0] rec, input logic [23:0] il, input logic tx,
                           input logic [31:0] wa, input logic [23:0] lim, input st_t nxt, input logic go);
    logic        inc, dec;
    logic [23:0] ro, n;
    logic [4:0]  sk;
    inc = rec[16];
    dec = rec[18];
    ro  = rec[14*32 +: 24];
    if (inc && !dec) begin
      // incompressible: raw data at roff in the raw area, only lsize bytes are real
      n  = (il > rec[32 +: 24]) ? rec[32 +: 24] : il;
      sk = 5'(ro[6:2]);
      if (go) start_xf(2'd0, slot_raw(cur_slot) + {8'd0, ro[23:7], 7'd0}, up128(24'({sk, 2'b00}) + n), sk, n, 24'd0, tx, wa, lim, nxt);
      else    set_xf  (2'd0, slot_raw(cur_slot) + {8'd0, ro[23:7], 7'd0}, up128(24'({sk, 2'b00}) + n), sk, n, 24'd0, tx, wa, lim);
    end else begin
      if (go) start_xf(2'd0, slot_out(cur_slot), up128(il), 5'd0, il, 24'd0, tx, wa, lim, nxt);
      else    set_xf  (2'd0, slot_out(cur_slot), up128(il), 5'd0, il, 24'd0, tx, wa, lim);
    end
  endtask

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
      slot_busy  <= '0;
      slot_ffe   <= '0;
      cen        <= 1'b0;
      recsz      <= '0;
      xf_start   <= 1'b0;
      xf_pend    <= 1'b0;
      kind       <= K_NONE;
    end else begin
      db_clr   <= 1'b0;
      xf_start <= 1'b0;

      // FFE-Receive: retained data follows the CRB once the generator is done
      if (xf_pend && !gen_active && (st == S_DATA_CHUNK || st == S_DATA_WAIT)) begin
        xf_start <= 1'b1;
        xf_pend  <= 1'b0;
      end

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
          if (drd_idle && mwr_idle && !ddrr_busy && !ddrw_busy && !gen_active && drd_avail == '0 && !xf_busy) begin
            sq_valid  <= '0;
            cq_valid  <= '0;
            slot_busy <= '0;                       // a controller reset frees every staging slot
            cen       <= 1'b0;                     // and returns to RAW
            recsz     <= '0;
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
          kind    <= K_NONE;
          pxfer   <= '0;
          st      <= S_CQE_ROOM;          // default: no data phase

          if (cur_q == 3'd0) begin
            // ======================= admin =======================
            case (opc)
              8'h06: begin  // Identify
                id_cns  <= cdw10[7:0];
                src     <= SRC_ID;
                to_host <= 1'b1;
                xfer    <= 24'd4096;
                st      <= S_PRP;
              end
              8'h02: begin  // Get Log Page
                logic [31:0] numd;
                numd = {cdw11[15:0], cdw10[31:16]} + 32'd1;
                log_id  <= cdw10[7:0];
                src     <= SRC_LOG;
                to_host <= 1'b1;
                xfer    <= (numd > 32'd1024) ? 24'd4096 : 24'({numd[10:0], 2'b00});
                st      <= S_PRP;
              end
              8'h09, 8'h0A: begin  // Set / Get Features
                if (cdw10[7:0] == 8'h07) res_dw0 <= {16'(NQ - 1), 16'(NQ - 1)};
                if (cdw10[7:0] == 8'hC0) begin             // compression configuration
                  if (opc == 8'h09) begin
                    if (cdw11[7:4] > MAXRS) status <= ST_INVALID_FIELD;
                    else begin
                      cen   <= cdw11[0];
                      recsz <= cdw11[7:4];
                    end
                  end else begin
                    res_dw0 <= {16'(NSLOT), MAXRS, 4'd0, recsz, 3'd0, cen};
                  end
                end
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
                  xfer     <= 24'({nblk[11:0], 12'd0});
                  ddr_addr <= DDR_BASE + {slba[19:0], 12'd0};
                  st       <= S_PRP;
                end
              end
              8'h81: begin  // Compress-to-Staging (phase 1)
                if (!cen) status <= ST_CMD_SEQ;
                else if (nsid != 32'd1) status <= ST_INVALID_NS;
                else if (cdw10 == 32'd0 || cdw10[8:0] != 9'd0 || cdw10 > 32'(rec_lim) ||
                         prp1[1:0] != 2'b00 || mptr[1:0] != 2'b00)
                  status <= ST_INVALID_FIELD;
                else if (!have_free) status <= ST_STAGING_FULL;
                else begin
                  cur_slot            <= free_idx;
                  slot_busy[free_idx] <= 1'b1;
                  slot_ffe[free_idx]  <= 1'b0;
                  kind     <= K_C2S;
                  k_len    <= cdw10[23:0];
                  k_thr    <= cdw11[7:0];
                  roff     <= '0;
                  to_host  <= 1'b0;
                  src      <= SRC_DDR;
                  xfer     <= cdw10[23:0];
                  ddr_addr <= slot_raw(free_idx);
                  st       <= S_PRP;
                end
              end
              8'h84: begin  // Commit (phase 2)
                if (!cen) status <= ST_CMD_SEQ;
                else if (nsid != 32'd1) status <= ST_INVALID_NS;
                else if (cdw13[15:0] >= 16'(NSLOT) || !slot_busy[cdw13[SW-1:0]]) status <= ST_BAD_HANDLE;
                else begin
                  cur_slot  <= cdw13[SW-1:0];
                  kind      <= K_COMMIT;
                  k_slba    <= {cdw11, cdw10};
                  k_nlb     <= cdw12[15:0];
                  k_discard <= cdw13[31];
                  st        <= S_SLOT_RD;
                end
              end
              8'h86: begin  // Decompress-Read
                logic [31:0] ext;
                ext = ({16'd0, cdw12[15:0]} + 32'd1) << 12;
                if (nsid != 32'd1) status <= ST_INVALID_NS;
                else if ({cdw11, cdw10} >= NSZE || (NSZE - {cdw11, cdw10}) < (64'(cdw12[15:0]) + 64'd1))
                  status <= ST_LBA_RANGE;
                else if (ext > 32'(MAX_REC) * 2 || cdw13 == 32'd0 || cdw13 > ext ||
                         cdw14 == 32'd0 || cdw14[1:0] != 2'b00 || cdw14 > 32'(MAX_REC) ||
                         prp1[1:0] != 2'b00 || (!cdw15[0] && mptr[1:0] != 2'b00))
                  status <= ST_INVALID_FIELD;
                else begin
                  kind    <= K_DREAD;
                  k_slba  <= {cdw11, cdw10};
                  k_psz   <= ext[23:0];
                  k_csize <= cdw13[23:0];
                  k_ulen  <= cdw14[23:0];
                  k_nochk <= cdw15[0];
                  st      <= cdw15[0] ? S_DREAD_GO : S_MD_FETCH;
                end
              end
              8'h89: begin  // FFE-Send
                logic [8:0] n;
                n = 9'(cdw11[15:0]) + 9'(cdw11[31:16]);
                if (!cen) status <= ST_CMD_SEQ;
                else if (nsid != 32'd1) status <= ST_INVALID_NS;
                else if (cdw11[15:8] != 8'd0 || cdw11[31:24] != 8'd0 || n == 9'd0 || n > 9'd8 ||
                         cdw10 < 32'({n, 6'd0}) || cdw10[6:0] != 7'd0 || cdw10 > 32'(MAX_BLKS) * 32'd4096 ||
                         prp1[1:0] != 2'b00 || (13'(prp1[11:0]) + 13'({n, 6'd0})) > 13'd4096)
                  status <= ST_INVALID_FIELD;
                else if (have_lid) status <= ST_INVALID_FIELD;            // List ID already in use
                else if (!have_free) status <= ST_STAGING_FULL;
                else begin
                  cur_slot            <= free_idx;
                  slot_busy[free_idx] <= 1'b1;
                  slot_ffe[free_idx]  <= 1'b1;
                  slot_lid[free_idx]  <= cmd[2][15:0];
                  kind  <= K_FSEND;
                  dsc_n <= n[7:0];
                  st    <= S_DESC_FETCH;
                end
              end
              8'h8A: begin  // FFE-Receive
                if (nsid != 32'd1) status <= ST_INVALID_NS;
                else if (!have_lid) status <= ST_BAD_HANDLE;
                else if (cdw10 == 32'd0 || cdw10[1:0] != 2'b00 || cdw10 > 32'(MAX_BLKS) * 32'd4096 ||
                         prp1[1:0] != 2'b00)
                  status <= ST_INVALID_FIELD;
                else begin
                  cur_slot <= lid_idx;
                  kind     <= K_FRECV;
                  st       <= S_SLOT_RD;
                end
              end
              default: status <= ST_INVALID_OPCODE;
            endcase
          end
        end

        // ---------------------------------------------------------------
        S_PRP: begin
          prp[0] <= {prp1[63:2], 2'b00};
          if (plen <= 24'(first_len)) begin
            st <= S_DATA_START;
          end else if (rem2 <= 24'd4096) begin
            prp[1] <= {prp2[63:12], 12'd0};
            st     <= S_DATA_START;
          end else begin
            need      <= 10'((rem2 + 24'd4095) >> 12);
            list_addr <= {prp2[63:3], 3'b000};
            prp_wi    <= 10'd1;
            st        <= S_PRPL_CALC;
          end
        end

        S_PRPL_CALC: begin
          // entries to read from the current list page; if more are needed than fit,
          // the last entry of the page is a pointer to the next list page
          if (need > list_room) begin
            ent_cnt <= list_room;
            chain   <= 1'b1;
          end else begin
            ent_cnt <= need;
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
              prp_wi      <= prp_wi + 10'd1;
              need        <= need - 10'd1;
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
          pi        <= 10'd0;
          if (to_host && src == SRC_DDR) begin
            if (ddrr_job_ready) st <= S_DATA_CHUNK;
          end else if (!to_host) begin
            if (ddrw_job_ready) st <= S_DATA_CHUNK;
          end else if (src == SRC_XF) begin
            xf_start <= 1'b1;                       // Decompress-Read: inflate straight to the TX FIFO
            st       <= S_DATA_CHUNK;
          end else if (src == SRC_CRB) begin
            gen_active <= 1'b1;                     // FFE-Receive: CRB first, then retained data
            gen_idx    <= '0;
            gen_total  <= (xfer >= 24'd64) ? 11'd16 : 11'(xfer >> 2);
            xf_pend    <= (xfer > 24'd64);
            st         <= S_DATA_CHUNK;
          end else begin
            gen_active <= 1'b1;
            gen_idx    <= '0;
            gen_total  <= 11'(xfer >> 2);
            st         <= S_DATA_CHUNK;
          end
        end

        S_DATA_CHUNK: begin
          if ((to_host && mwr_job_ready) || (!to_host && drd_job_ready)) begin
            remaining <= remaining - 24'(chunk);
            pi        <= pi + 10'd1;
            cur_addr  <= prp[pi + 10'd1];
            if (remaining == 24'(chunk)) st <= S_DATA_WAIT;
          end
        end

        S_DATA_WAIT: begin
          if (to_host) begin
            if (mwr_idle && !ddrr_busy && !gen_active && !xf_busy && !xf_pend && !xf_start) st <= S_POST;
          end else begin
            if (drd_idle && !ddrw_busy && drd_avail == '0) st <= S_POST;
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

        // ===============================================================
        // compression path
        // ===============================================================
        S_POST: begin
          case (kind)
            K_C2S:
              start_xf(2'd1, slot_raw(cur_slot), up128(k_len), 5'd0, k_len, k_len, 1'b0,
                       slot_out(cur_slot), MAX_REC << 1, S_C2S_FIN);
            K_FSEND: begin
              logic [4:0] sk;
              sk = 5'(roff[6:2]);
              if (f_fn == 8'h82)
                start_xf(2'd1, slot_raw(cur_slot) + {8'd0, roff[23:7], 7'd0}, up128(24'({sk, 2'b00}) + k_len),
                         sk, k_len, k_len, 1'b0, slot_out(cur_slot), MAX_REC << 1, S_C2S_FIN);
              else if (f_lbasrc)
                start_xf(2'd2, DDR_BASE + {k_slba[19:0], 12'd0}, k_psz, 5'd0, k_psz, k_psz, 1'b0,
                         slot_out(cur_slot), MAX_REC, S_DEC_FIN);
              else
                start_xf(2'd2, slot_raw(cur_slot) + {8'd0, roff[23:7], 7'd0}, up128(24'({sk, 2'b00}) + k_len),
                         sk, k_len, (k_len + 24'd3) & ~24'd3, 1'b0, slot_out(cur_slot), MAX_REC, S_DEC_FIN);
            end
            K_DREAD: begin
              if (!k_nochk && xf_fi != {dsc[11], dsc[10], dsc[9], dsc[8], dsc[7], dsc[6], dsc[5], dsc[4]})
                status <= ST_CKSUM;
              else if (xf_ierr != 4'd0)
                status <= ST_DECOMP;
              else if (xf_out_len != k_ulen || xf_over)
                status <= ST_LEN;
              st <= S_CQE_ROOM;
            end
            K_FRECV: st <= S_FREE_CQE;
            default: st <= S_CQE_ROOM;
          endcase
        end

        S_XF_WAIT: if (xf_done) st <= xf_next;

        // compressed: decide incompressible, build the CRB
        S_C2S_FIN: begin
          logic        inc;
          logic [23:0] cs, ps;
          logic [255:0] fl;
          cs = xf_out_len;
          if (k_thr == 8'd0) inc = xf_over || (cs > k_len - (k_len >> 3));
          else               inc = xf_over || (48'(cs) * 48'd100 > 48'(k_len) * 48'(k_thr));
          ps = inc ? up4k(k_len) : xf_wr_bytes;
          fl = inc ? xf_fi : xf_fo;
          crb[0]  <= {13'd0, 1'b0, 1'b1, inc, 16'(cur_slot)};   // flags: INCOMP, VALID, DECOMP
          crb[1]  <= 32'(k_len);
          crb[2]  <= inc ? 32'(k_len) : 32'(cs);
          crb[3]  <= 32'(ps);
          for (int k = 0; k < 8; k++) crb[4 + k] <= fl[32*k +: 32];
          crb[12] <= xf_adler;
          crb[13] <= {16'd0, slot_ffe[cur_slot] ? slot_lid[cur_slot] : 16'd0};
          crb[14] <= 32'(roff);
          crb[15] <= {31'd0, slot_ffe[cur_slot]};            // internal: FFE, COMMITTED, DECOMP
          res_dw0 <= {15'd0, inc, 16'(cur_slot)};
          f_cmt   <= 1'b0;
          xf_next <= (kind == K_C2S) ? S_RES_GEN : S_FFE_DEST;
          st      <= S_SLOT_WR;
        end

        // decompressed (FFE 83h): build the CRB
        S_DEC_FIN: begin
          logic [10:0] stt;
          stt = (xf_ierr != 4'd0) ? ST_DECOMP : xf_over ? ST_LEN : ST_SUCCESS;
          if (stt != ST_SUCCESS) begin
            status              <= stt;
            slot_busy[cur_slot] <= 1'b0;
            st                  <= S_CQE_ROOM;
          end else begin
            crb[0]  <= {13'd0, 1'b1, 1'b1, 1'b0, 16'(cur_slot)};
            crb[1]  <= 32'(xf_out_len);
            crb[2]  <= 32'(xf_consumed);
            crb[3]  <= 32'(xf_wr_bytes);
            for (int k = 0; k < 8; k++) crb[4 + k] <= xf_fi[32*k +: 32];
            crb[12] <= 32'd0;
            crb[13] <= {16'd0, slot_lid[cur_slot]};
            crb[14] <= 32'd0;
            crb[15] <= {29'd0, 1'b1, 1'b0, 1'b1};
            res_dw0 <= {16'd0, 16'(cur_slot)};
            f_cmt   <= 1'b0;
            xf_next <= S_FFE_DEST;
            st      <= S_SLOT_WR;
          end
        end

        S_SLOT_WR: begin
          logic [511:0] w;
          for (int k = 0; k < 16; k++) w[32*k +: 32] = crb[k];
          slot_mem[cur_slot] <= w;
          st <= xf_next;
        end

        // Compress-to-Staging result record to MPTR
        S_RES_GEN: begin
          src        <= SRC_CRB;
          gen_active <= 1'b1;
          gen_idx    <= '0;
          gen_total  <= 11'd16;
          st         <= S_RES_JOB;
        end
        S_RES_JOB:  if (mwr_job_ready) st <= S_RES_WAIT;
        S_RES_WAIT: if (mwr_idle && !gen_active) st <= S_CQE_ROOM;

        // FFE-Send: optional LBA destination, then done
        S_FFE_DEST: begin
          logic [511:0] w;
          for (int k = 0; k < 16; k++) w[32*k +: 32] = crb[k];
          if (f_dlba) begin
            f_dlba <= 1'b0;
            if (f_dslba >= NSZE || (NSZE - f_dslba) < (64'(f_dnlb) + 64'd1)) begin
              status <= ST_LBA_RANGE;
              slot_busy[cur_slot] <= 1'b0;
              st <= S_CQE_ROOM;
            end else if (crb[3] > ((32'(f_dnlb) + 32'd1) << 12)) begin
              status <= ST_LEN;
              slot_busy[cur_slot] <= 1'b0;
              st <= S_CQE_ROOM;
            end else begin
              f_cmt <= 1'b1;
              copy_slot(w, crb[3][23:0], 1'b0, DDR_BASE + {f_dslba[19:0], 12'd0}, crb[3][23:0], S_FFE_DEST, 1'b1);
            end
          end else if (f_cmt) begin
            f_cmt      <= 1'b0;
            crb[15][1] <= 1'b1;                               // committed: Receive returns the CRB only
            xf_next    <= S_CQE_ROOM;
            st         <= S_SLOT_WR;
          end else begin
            st <= S_CQE_ROOM;
          end
        end

        S_SLOT_RD: begin
          srec_raw <= slot_mem[cur_slot];
          st       <= S_SLOT_RD2;
        end
        S_SLOT_RD2: begin
          srec <= srec_raw;
          st   <= (kind == K_COMMIT) ? S_COMMIT_CHK : S_RECV_CHK;
        end

        S_COMMIT_CHK: begin
          if (k_discard) st <= S_FREE_CQE;
          else if (((32'(k_nlb) + 32'd1) << 12) != srec[96 +: 32]) begin
            status <= ST_INVALID_FIELD;
            st     <= S_CQE_ROOM;
          end else if (k_slba >= NSZE || (NSZE - k_slba) < (64'(k_nlb) + 64'd1)) begin
            status <= ST_LBA_RANGE;
            st     <= S_CQE_ROOM;
          end else begin
            copy_slot(srec, srec[96 +: 24], 1'b0, DDR_BASE + {k_slba[19:0], 12'd0}, srec[96 +: 24], S_FREE_CQE, 1'b1);
          end
        end

        S_FREE_CQE: begin
          slot_busy[cur_slot] <= 1'b0;
          st <= S_CQE_ROOM;
        end

        S_RECV_CHK: begin
          logic [23:0] ol, tot, dl;
          for (int k = 0; k < 16; k++) crb[k] <= srec[32*k +: 32];
          if (srec[15*32 + 1])  ol = 24'd0;                                // committed
          else if (srec[18])    ol = (srec[32 +: 24] + 24'd3) & ~24'd3;     // decompressed bytes
          else                  ol = srec[96 +: 24];                        // stored bytes
          tot = (cdw10[23:0] < 24'd64 + ol) ? cdw10[23:0] : 24'd64 + ol;
          dl  = (tot > 24'd64) ? tot - 24'd64 : 24'd0;
          copy_slot(srec, dl, 1'b1, 32'd0, dl, S_IDLE, 1'b0);
          res_dw0 <= 32'(tot);
          xfer    <= tot;
          pxfer   <= cdw10[23:0];                                        // PRPs describe the whole buffer
          to_host <= 1'b1;
          src     <= SRC_CRB;
          st      <= S_PRP;
        end

        // Decompress-Read: MPTR descriptor (expected Fletcher-4), then the data phase
        S_MD_FETCH: if (drd_job_ready) begin
          dwi <= '0;
          st  <= S_MD_RD;
        end
        S_MD_RD: if (drd_avail >= 2) begin
          dsc[dwi[6:0]]         <= drd_head0;
          dsc[dwi[6:0] + 7'd1]  <= drd_head1;
          dwi <= dwi + 8'd2;
          if (dwi == 8'd14) st <= S_DREAD_GO;
        end
        S_DREAD_GO: begin
          set_xf(2'd2, DDR_BASE + {k_slba[19:0], 12'd0}, k_psz, 5'd0, k_csize, k_psz, 1'b1, 32'd0, k_ulen);
          xfer    <= k_ulen;
          to_host <= 1'b1;
          src     <= SRC_XF;
          st      <= S_PRP;
        end

        // FFE-Send: descriptors from the first page of PRP1
        S_DESC_FETCH: if (drd_job_ready) begin
          dwi <= '0;
          st  <= S_DESC_RD;
        end
        S_DESC_RD: if (drd_avail >= 2) begin
          dsc[dwi[6:0]]        <= drd_head0;
          dsc[dwi[6:0] + 7'd1] <= drd_head1;
          dwi <= dwi + 8'd2;
          if (dwi + 8'd2 == {dsc_n[3:0], 4'd0}) begin
            dsc_i     <= '0;
            f_has_fd  <= 1'b0;
            f_has_fpd <= 1'b0;
            f_dlba    <= 1'b0;
            f_bad     <= 1'b0;
            st        <= S_DESC_PARSE;
          end
        end
        S_DESC_PARSE: begin
          logic [6:0] b;
          logic [7:0] t;
          b = {dsc_i[2:0], 4'd0};
          t = dsc[b][7:0];
          case (t)
            8'h01: begin
              f_dlba  <= dsc[b][8];
              f_dslba <= {dsc[b + 7'd4], dsc[b + 7'd3]};
              f_dnlb  <= dsc[b + 7'd5][15:0];
            end
            8'h03, 8'h04: ;
            8'h82, 8'h83: begin
              if (f_has_fd || dsc[b + 7'd1][31:16] != 16'd1) f_bad <= 1'b1;
              f_has_fd <= 1'b1;
              f_fn     <= t;
              f_srci   <= dsc[b + 7'd4][7:0];
            end
            8'hFE: begin
              f_has_fpd <= 1'b1;
              f_fpd1    <= dsc[b + 7'd1];
              f_fpd2    <= dsc[b + 7'd2];
              f_fpd3    <= dsc[b + 7'd3];
            end
            default: f_bad <= 1'b1;
          endcase
          dsc_i <= dsc_i + 8'd1;
          if (dsc_i + 8'd1 == dsc_n) st <= S_FFE_CHK;
        end
        S_FFE_CHK: begin
          logic [6:0]  sb;
          logic [7:0]  stp;
          logic [31:0] sd1, sd2, sd3, sd4;
          logic        ok;
          sb  = {f_srci[2:0], 4'd0};
          stp = dsc[sb][7:0];
          sd1 = dsc[sb + 7'd1];
          sd2 = dsc[sb + 7'd2];
          sd3 = dsc[sb + 7'd3];
          sd4 = dsc[sb + 7'd4];
          ok  = !f_bad && f_has_fd && (f_srci < dsc_n);
          f_lbasrc <= 1'b0;
          if (f_fn == 8'h82) begin
            ok = ok && stp == 8'h04 && sd1 != 32'd0 && sd1[1:0] == 2'b00 && sd1 <= 32'(rec_lim) &&
                 sd2[1:0] == 2'b00 && (sd2 + sd1) <= cdw10;
            if (f_has_fpd)
              ok = ok && f_fpd1[15:8] == 8'h01 && f_fpd1[31:24] <= 8'd100 && f_fpd1[1:0] == 2'b00 &&
                   f_fpd2 == 32'd0 && (f_fpd3 == 32'd0 || f_fpd3 == 32'd4096);
            k_len <= sd1[23:0];
            k_thr <= f_has_fpd ? f_fpd1[31:24] : 8'd0;
            roff  <= sd2[23:0];
          end else begin
            if (stp == 8'h04) begin
              ok = ok && sd1 != 32'd0 && sd2[1:0] == 2'b00 && (sd2 + sd1) <= cdw10;
              k_len <= sd1[23:0];
              roff  <= sd2[23:0];
            end else if (stp == 8'h03) begin
              ok = ok && (sd1 == 32'd0 || sd1 == 32'd1) && {sd3, sd2} < NSZE &&
                   (NSZE - {sd3, sd2}) >= (64'(sd4[15:0]) + 64'd1) &&
                   ((32'(sd4[15:0]) + 32'd1) << 12) <= 32'(MAX_REC) * 2;
              f_lbasrc <= 1'b1;
              k_slba   <= {sd3, sd2};
              k_psz    <= 24'((32'(sd4[15:0]) + 32'd1) << 12);
            end else begin
              ok = 1'b0;
            end
            if (f_has_fpd) ok = ok && f_fpd1[7:0] == 8'h01 && f_fpd2 == 32'd0 && f_fpd3 == 32'd0;
          end
          if (!ok) begin
            status              <= ST_INVALID_FIELD;
            slot_busy[cur_slot] <= 1'b0;
            st                  <= S_CQE_ROOM;
          end else begin
            xfer     <= cdw10[23:0];
            to_host  <= 1'b0;
            src      <= SRC_DDR;
            ddr_addr <= slot_raw(cur_slot);
            st       <= S_PRP;
          end
        end

        default: st <= S_OFF;
      endcase

      // CC.EN cleared while a command is between fetch and its data phase: drop it.
      // No data engine, generator or xform has been started yet, and S_DISABLE drains
      // whatever the SQ / PRP list fetch still returns. (Once the data phase has started
      // the command runs to S_CQE_ROOM, which also goes to S_DISABLE; dma_rd's completion
      // timeout bounds that even if the host stops answering reads.)
      if (!cc[0] && (st == S_FETCH || st == S_FETCH_RD || st == S_DECODE || st == S_PRP ||
                     st == S_PRPL_CALC || st == S_PRPL_FETCH || st == S_PRPL_RD))
        st <= S_DISABLE;
    end
  end

  assign dbg_st = 6'(st);

endmodule
