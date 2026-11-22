// xform.sv - DDR stream transform: DDR -> {COPY | DEFLATE | INFLATE} -> DDR or TX FIFO.
//  * Reads rd_len bytes from rd_addr through ddr_rd into an input DW FIFO.
//  * The first skip_dw DWs are dropped; of the rest, in_len bytes go to the function and
//    anything after that is drained. Fletcher-4 (fi*) covers the first ck_len bytes after the
//    skip, zero padded to a 4 KB multiple.
//  * Output to DDR (dst_tx = 0): written at wr_addr one 4 KB page per ddr_wr job, zero padded
//    to a page multiple, at most out_lim bytes kept (more sets 'over'); Fletcher-4 (fo*) of
//    everything written. Output to the TX FIFO (dst_tx = 1): exactly out_lim bytes, truncated
//    or zero filled.
//  * COPY moves whole DWs (in_len a DW multiple), 2 per cycle. DEFLATE / INFLATE are byte
//    engines: deflate_enc and inflate.
module xform #(
  parameter int FAW = 10
)(
  input  logic          clk,
  input  logic          rst,

  input  logic          start,
  input  logic [1:0]    func,          // 0 COPY, 1 DEFLATE, 2 INFLATE
  input  logic [31:0]   rd_addr,       // 128-byte aligned
  input  logic [23:0]   rd_len,        // bytes, multiple of 128
  input  logic [4:0]    skip_dw,
  input  logic [23:0]   in_len,
  input  logic [23:0]   ck_len,        // DW multiple
  input  logic          dst_tx,
  input  logic [31:0]   wr_addr,       // 4 KB aligned
  input  logic [23:0]   out_lim,       // DW multiple when dst_tx

  output logic          busy,
  output logic          done,
  output logic [23:0]   out_len,       // function output bytes (before limit / padding)
  output logic [23:0]   wr_bytes,      // bytes written to DDR (padded)
  output logic          over,
  output logic [63:0]   fia, fib, fic, fid,
  output logic [63:0]   foa, fob, foc, fod,
  output logic [31:0]   adler,         // DEFLATE: Adler-32 of the input
  output logic [3:0]    ierr,          // INFLATE: error code (0 = ok)
  output logic [23:0]   consumed,      // INFLATE: stream bytes

  // ddr_rd job and data sink
  output logic          rdj_valid,
  input  logic          rdj_ready,
  output logic [31:0]   rdj_addr,
  output logic [23:0]   rdj_len,
  input  logic          rd_busy,
  input  logic [1:0]    rd_push,
  input  logic [31:0]   rd_dw0,
  input  logic [31:0]   rd_dw1,
  output logic [FAW:0]  rd_free,

  // ddr_wr job and data source
  output logic          wrj_valid,
  input  logic          wrj_ready,
  output logic [31:0]   wrj_addr,
  output logic [23:0]   wrj_len,
  input  logic          wr_busy,
  output logic [FAW:0]  wr_avail,
  output logic [31:0]   wr_dw0,
  output logic [31:0]   wr_dw1,
  input  logic [1:0]    wr_pop,

  // TX FIFO
  output logic [1:0]    tx_push,
  output logic [31:0]   tx_dw0,
  output logic [31:0]   tx_dw1,
  input  logic [FAW:0]  tx_free
);
  localparam logic [1:0] F_COPY = 2'd0, F_DEFL = 2'd1, F_INFL = 2'd2;

  typedef enum logic [2:0] {X_IDLE, X_RD, X_RUN, X_FLUSH, X_PAD, X_WAIT} xs_t;
  xs_t xs;

  // job parameters
  logic [1:0]  fn;
  logic        totx;
  logic [21:0] rd_dws;        // DWs in the read stream
  logic [4:0]  skip;
  logic [23:0] inl, ckl, lim;
  logic [31:0] wbase;

  // ---------------- input FIFO ----------------
  logic [1:0]   xin_pop;
  logic [31:0]  xin_h0, xin_h1;
  logic [FAW:0] xin_cnt, xin_free;
  dw_fifo #(.AW(FAW)) u_xin (
    .clk, .rst, .flush(start),
    .push_cnt(rd_push), .push_dw0(rd_dw0), .push_dw1(rd_dw1),
    .pop_cnt(xin_pop), .head0(xin_h0), .head1(xin_h1),
    .count(xin_cnt), .free(xin_free)
  );
  assign rd_free = xin_free;

  // ---------------- output FIFO (DDR destination) ----------------
  logic [1:0]   xo_push;
  logic [31:0]  xo_d0, xo_d1;
  logic [FAW:0] xo_cnt, xo_free;
  dw_fifo #(.AW(FAW)) u_xout (
    .clk, .rst, .flush(start),
    .push_cnt(xo_push), .push_dw0(xo_d0), .push_dw1(xo_d1),
    .pop_cnt(wr_pop), .head0(wr_dw0), .head1(wr_dw1),
    .count(xo_cnt), .free(xo_free)
  );
  assign wr_avail = xo_cnt;

  // ---------------- engines ----------------
  logic        d_start, d_in_valid, d_in_ready, d_out_valid, d_out_ready, d_busy, d_done;
  logic [7:0]  d_out_byte;
  logic [23:0] d_csize;
  logic        i_start, i_in_valid, i_in_ready, i_in_eof, i_out_valid, i_out_ready, i_busy, i_done;
  logic [7:0]  i_out_byte;
  logic [23:0] i_produced;
  logic [7:0]  ub_byte;

  deflate_enc u_defl (
    .clk, .rst, .start(d_start), .total(21'(in_len)),
    .in_valid(d_in_valid), .in_ready(d_in_ready), .in_byte(ub_byte),
    .out_valid(d_out_valid), .out_ready(d_out_ready), .out_byte(d_out_byte),
    .busy(d_busy), .done(d_done), .csize(d_csize), .adler(adler)
  );
  inflate u_infl (
    .clk, .rst, .start(i_start),
    .in_valid(i_in_valid), .in_ready(i_in_ready), .in_byte(ub_byte), .in_eof(i_in_eof),
    .out_valid(i_out_valid), .out_ready(i_out_ready), .out_byte(i_out_byte),
    .busy(i_busy), .done(i_done), .err(ierr), .consumed(consumed), .produced(i_produced)
  );

  // ---------------- read-stream bookkeeping ----------------
  logic [21:0] popped;        // DWs popped from the read stream (including skipped)
  logic        rd_issued;
  logic        func_done;     // function finished (engine done seen / COPY input complete)
  logic [23:0] fed;           // bytes given to the byte engine
  logic [31:0] ub;            // unpacker DW
  logic [2:0]  ubn;           // bytes left in ub
  logic [1:0]  ubi;           // next byte index in ub

  wire         all_popped = (popped == rd_dws);
  // DW class by index k in the read stream
  function automatic logic is_fwd(input logic [21:0] k);       // carries function input
    if (k < 22'(skip)) return 1'b0;
    return ({(k - 22'(skip)), 2'b00} < inl);
  endfunction
  function automatic logic is_ck(input logic [21:0] k);
    if (k < 22'(skip)) return 1'b0;
    return ({(k - 22'(skip)), 2'b00} < ckl);
  endfunction

  // ---------------- output bookkeeping ----------------
  logic [23:0] ocnt;          // function output bytes seen
  logic [31:0] pk;            // packer
  logic [1:0]  pkn;
  logic [21:0] pushed;        // DWs pushed to the sink (DDR or TX)
  logic [21:0] ck_fed;        // DWs given to Fletcher-in
  logic [11:0] pages;         // 4 KB write jobs issued

  wire [FAW:0] sink_free = totx ? tx_free : xo_free;
  wire [21:0]  lim_dw    = 22'(lim >> 2);
  wire [21:0]  tgt_dw    = totx ? lim_dw : {pushed[21:10] + 12'(pushed[9:0] != 10'd0), 10'd0};

  // engine output byte (DEFLATE or INFLATE)
  wire        e_valid = (fn == F_DEFL) ? d_out_valid : (fn == F_INFL) ? i_out_valid : 1'b0;
  wire [7:0]  e_byte  = (fn == F_DEFL) ? d_out_byte  : i_out_byte;
  wire        keep    = (ocnt < lim);
  wire        pk_full = (pkn == 2'd3);
  // accept a byte when it will not need a push, or the sink has room for the push
  wire        e_ready = (xs == X_RUN) && (!keep || !pk_full || sink_free >= 2);
  wire        e_take  = e_valid && e_ready;
  assign d_out_ready = (fn == F_DEFL) && e_ready;
  assign i_out_ready = (fn == F_INFL) && e_ready;

  // ---------------- per-cycle data movement (combinational) ----------------
  logic [1:0]  pop_n;
  logic [1:0]  push_n;
  logic [31:0] push0, push1;
  logic [1:0]  ck_n;
  logic [31:0] ck0, ck1;
  logic        ub_load;
  always_comb begin
    pop_n = 2'd0; push_n = 2'd0; push0 = 32'd0; push1 = 32'd0;
    ck_n = 2'd0; ck0 = 32'd0; ck1 = 32'd0; ub_load = 1'b0;
    if (xs == X_RUN && xin_cnt != '0 && !all_popped) begin
      logic f0, f1;
      f0 = is_fwd(popped) && !func_done;
      f1 = is_fwd(popped + 22'd1) && !func_done;
      if (fn == F_COPY) begin
        if (!f0) begin
          pop_n = 2'd1;
          if (xin_cnt >= 2 && !f1 && popped + 22'd1 != rd_dws) pop_n = 2'd2;
        end else if (f1 && xin_cnt >= 2 && popped + 22'd1 != rd_dws && sink_free >= 2) begin
          pop_n = 2'd2; push_n = 2'd2; push0 = xin_h0; push1 = xin_h1;
        end else if (sink_free >= 2) begin
          pop_n = 2'd1; push_n = 2'd1; push0 = xin_h0;
        end
      end else begin
        if (!f0) pop_n = 2'd1;                                   // skip / drain
        else if (ubn == 3'd0 || (ubn == 3'd1 && (fn == F_DEFL ? d_in_ready : i_in_ready))) begin
          pop_n = 2'd1; ub_load = 1'b1;
        end
      end
      if (pop_n >= 2'd1 && is_ck(popped))         begin ck_n = 2'd1; ck0 = xin_h0; end
      if (pop_n == 2'd2 && is_ck(popped + 22'd1)) begin ck_n = ck_n + 2'd1; if (ck_n == 2'd1 && !is_ck(popped)) ck0 = xin_h1; else ck1 = xin_h1; end
    end
    // Fletcher-in zero padding to a 4 KB multiple once the stream is done
    if (xs == X_PAD && ck_fed[9:0] != 10'd0) begin
      ck_n = (ck_fed[9:0] == 10'h3FF) ? 2'd1 : 2'd2;
    end
    // byte-engine output: push a full packer DW
    if (xs == X_RUN && fn != F_COPY && e_take && keep && pk_full) begin
      push_n = 2'd1; push0 = {e_byte, pk[23:0]};
    end
    // flush a partial packer DW
    if (xs == X_FLUSH && pkn != 2'd0 && sink_free >= 2) begin
      push_n = 2'd1;
      case (pkn)
        2'd1:    push0 = {24'd0, pk[7:0]};
        2'd2:    push0 = {16'd0, pk[15:0]};
        default: push0 = {8'd0, pk[23:0]};
      endcase
    end
    // output zero padding
    if (xs == X_PAD && pushed < tgt_dw && sink_free >= 2) begin
      push_n = (tgt_dw - pushed >= 22'd2) ? 2'd2 : 2'd1;
    end
    // TX destination is exact: never push past out_lim
    if (totx && pushed + 22'(push_n) > lim_dw) push_n = (pushed < lim_dw) ? 2'd1 : 2'd0;
  end
  assign xin_pop = pop_n;

  // unpacker -> byte engines
  assign ub_byte     = ub[8*ubi +: 8];
  assign d_in_valid  = (fn == F_DEFL) && (ubn != 3'd0);
  assign i_in_valid  = (fn == F_INFL) && (ubn != 3'd0) && !func_done;
  assign i_in_eof    = (fed == inl) && (ubn == 3'd0);
  wire   u_take      = (fn == F_DEFL) ? (d_in_valid && d_in_ready) : (i_in_valid && i_in_ready);

  // sinks
  assign xo_push = totx ? 2'd0 : push_n;
  assign xo_d0   = push0;
  assign xo_d1   = push1;
  assign tx_push = totx ? push_n : 2'd0;
  assign tx_dw0  = push0;
  assign tx_dw1  = push1;

  fletcher4 u_fi (.clk, .clr(start), .cnt(ck_n), .w0(ck0), .w1(ck1), .a(fia), .b(fib), .c(fic), .d(fid));
  fletcher4 u_fo (.clk, .clr(start), .cnt(totx ? 2'd0 : push_n), .w0(push0), .w1(push1),
                  .a(foa), .b(fob), .c(foc), .d(fod));

  // ddr jobs
  assign rdj_valid = (xs == X_RD);
  assign rdj_addr  = rd_addr;
  assign rdj_len   = rd_len;
  assign wrj_valid = !totx && (xs != X_IDLE) && ({pages, 10'd0} < pushed) ;
  assign wrj_addr  = wbase + {8'd0, pages, 12'd0};
  assign wrj_len   = 24'd4096;
  wire   wrj_fire  = wrj_valid && wrj_ready;

  assign d_start = start && (func == F_DEFL);
  assign i_start = start && (func == F_INFL);
  assign out_len = (fn == F_COPY) ? inl : ocnt;
  assign wr_bytes = {pages, 12'd0};

  // ---------------- sequential ----------------
  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (rst) begin
      xs   <= X_IDLE;
      busy <= 1'b0;
    end else begin
      if (start) begin
        fn        <= func;
        totx      <= dst_tx;
        rd_dws    <= 22'(rd_len >> 2);
        skip      <= skip_dw;
        inl       <= in_len;
        ckl       <= ck_len;
        lim       <= out_lim;
        wbase     <= wr_addr;
        popped    <= '0;
        fed       <= '0;
        ub        <= '0;
        ubn       <= '0;
        ubi       <= '0;
        ocnt      <= '0;
        pk        <= '0;
        pkn       <= '0;
        pushed    <= '0;
        ck_fed    <= '0;
        pages     <= '0;
        over      <= 1'b0;
        func_done <= 1'b0;
        busy      <= 1'b1;
        xs        <= (rd_len == 24'd0) ? X_RUN : X_RD;
      end else begin
        popped <= popped + 22'(pop_n);
        pushed <= pushed + 22'(push_n);
        ck_fed <= ck_fed + 22'(ck_n);
        if (wrj_fire) pages <= pages + 12'd1;

        // unpacker (a reload in the same cycle as the last byte's take wins)
        if (u_take) begin
          fed <= fed + 24'd1;
          ubi <= ubi + 2'd1;
          ubn <= ubn - 3'd1;
        end
        if (ub_load) begin
          logic [23:0] rem;
          rem = inl - fed - 24'(ubn);              // function bytes not yet loaded
          ub  <= xin_h0;
          ubi <= 2'd0;
          ubn <= (rem >= 24'd4) ? 3'd4 : 3'(rem);
        end

        // packer
        if (e_take) begin
          ocnt <= ocnt + 24'd1;
          if (!keep) over <= 1'b1;
          else begin
            case (pkn)
              2'd0: pk[7:0]   <= e_byte;
              2'd1: pk[15:8]  <= e_byte;
              2'd2: pk[23:16] <= e_byte;
              default: ;
            endcase
            pkn <= pkn + 2'd1;                     // wraps to 0 after the 4th byte (pushed)
          end
        end
        if (xs == X_FLUSH && push_n != 2'd0) pkn <= 2'd0;

        // function completion
        if (fn == F_DEFL && d_done) func_done <= 1'b1;
        if (fn == F_INFL && i_done) func_done <= 1'b1;
        if (fn == F_COPY && xs == X_RUN && all_popped) func_done <= 1'b1;

        case (xs)
          X_RD:   if (rdj_ready) xs <= X_RUN;
          X_RUN:  if (all_popped && !rd_busy && func_done) xs <= X_FLUSH;
          X_FLUSH: if (pkn == 2'd0 || push_n != 2'd0) xs <= X_PAD;
          X_PAD: begin
            if (pushed + 22'(push_n) >= tgt_dw && (ck_fed[9:0] == 10'd0 || ck_fed[9:0] + 10'(ck_n) == 10'd0)) xs <= X_WAIT;
          end
          X_WAIT: begin
            if (totx || ({pages, 10'd0} >= pushed && !wr_busy && xo_cnt == '0 && !wrj_valid)) begin
              busy <= 1'b0;
              done <= 1'b1;
              xs   <= X_IDLE;
            end
          end
          default: ;
        endcase
      end
    end
  end

endmodule
