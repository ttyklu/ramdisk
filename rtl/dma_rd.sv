// dma_rd.sv - reads host memory over PCIe and delivers the data in order as a DW stream.
//  * Accepts jobs (DW-aligned host address, length in DWs, one page or less).
//  * Splits each job into MRd requests that never cross a 128-byte boundary (so a request
//    is at most 32 DWs and its completions are addressed by Lower Address[6:2]).
//  * Up to NT requests outstanding; tags are allocated in issue order.
//  * Completions may arrive out of order across tags; data is written straight into a
//    ring buffer at the position reserved for that request, and requests are committed
//    in issue order once all their data has arrived.
//  * The consumer pops 0..2 DWs per cycle while 'avail' > 0.
module dma_rd #(
  parameter int NT  = 16,      // outstanding read tags (power of 2)
  parameter int RAW = 10       // log2(ring size in DWs)
)(
  input  logic          clk,
  input  logic          rst,

  input  logic          job_valid,
  output logic          job_ready,
  input  logic [63:0]   job_addr,
  input  logic [11:0]   job_len,       // DWs, 1..1024

  output logic          r_valid,
  input  logic          r_ready,
  output logic [63:0]   r_addr,
  output logic [10:0]   r_len,
  output logic [7:0]    r_tag,

  input  logic          cpl_hdr_valid,
  input  logic [7:0]    cpl_tag,
  input  logic [6:0]    cpl_la,
  input  logic [2:0]    cpl_status,
  input  logic [10:0]   cpl_len,
  input  logic          cpl_hasdata,
  input  logic          cpl_data_valid,
  input  logic [1:0]    cpl_data_cnt,
  input  logic [31:0]   cpl_dw0,
  input  logic [31:0]   cpl_dw1,

  output logic [RAW:0]  avail,
  output logic [31:0]   head0,
  output logic [31:0]   head1,
  input  logic [1:0]    pop,
  output logic          idle,
  output logic          err             // sticky: a completion came back with an error status
);

  localparam int TW = $clog2(NT);

  logic [RAW:0]   alloc_ptr, commit_ptr, rd_ptr;
  logic [TW:0]    issue_cnt, commit_cnt;
  logic [RAW-1:0] req_base [NT];
  logic [4:0]     req_dw0  [NT];
  logic [5:0]     req_len  [NT];
  logic [5:0]     req_rem  [NT];

  logic           active;
  logic [63:0]    cur_addr;
  logic [11:0]    cur_rem;

  wire [TW:0]  outstanding = issue_cnt - commit_cnt;
  wire [RAW:0] used        = alloc_ptr - rd_ptr;
  wire [RAW:0] ring_free   = (RAW+1)'(1 << RAW) - used;

  wire [5:0]  to_bnd = 6'd32 - {1'b0, cur_addr[6:2]};
  wire [5:0]  nlen   = (cur_rem < {6'd0, to_bnd}) ? cur_rem[5:0] : to_bnd;

  assign job_ready = !active;
  assign r_valid   = active && (outstanding < (TW+1)'(NT)) && (ring_free >= {{(RAW-5){1'b0}}, nlen});
  assign r_addr    = cur_addr;
  assign r_len     = {5'd0, nlen};
  assign r_tag     = {{(8-TW){1'b0}}, issue_cnt[TW-1:0]};

  wire issue = r_valid && r_ready;

  // ---------------- completion write position ----------------
  logic [RAW-1:0] cidx;          // next ring index for the current completion's data
  logic [TW-1:0]  ctag;
  wire  [TW-1:0]  htag   = cpl_tag[TW-1:0];
  wire  [4:0]     hoff   = cpl_la[6:2] - req_dw0[htag];
  wire  [RAW-1:0] hidx   = req_base[htag] + {{(RAW-5){1'b0}}, hoff};
  wire  [RAW-1:0] widx   = cpl_hdr_valid ? hidx : cidx;
  wire  [TW-1:0]  wtag   = cpl_hdr_valid ? htag : ctag;
  wire  [1:0]     wcnt   = cpl_data_valid ? cpl_data_cnt : 2'd0;

  dw_ram2 #(.AW(RAW)) u_ring (
    .clk  (clk),
    .wcnt (wcnt),
    .widx (widx),
    .wd0  (cpl_dw0),
    .wd1  (cpl_dw1),
    .ridx (rd_ptr[RAW-1:0]),
    .rd0  (head0),
    .rd1  (head1)
  );

  wire [TW-1:0] ctagc     = commit_cnt[TW-1:0];
  wire          do_commit = (outstanding != '0) && (req_rem[ctagc] == 6'd0);

  always_ff @(posedge clk) begin
    if (rst) begin
      active     <= 1'b0;
      alloc_ptr  <= '0;
      commit_ptr <= '0;
      rd_ptr     <= '0;
      issue_cnt  <= '0;
      commit_cnt <= '0;
      cidx       <= '0;
      ctag       <= '0;
      err        <= 1'b0;
      for (int t = 0; t < NT; t++) req_rem[t] <= '0;
    end else begin
      // job accept
      if (job_valid && job_ready && job_len != 12'd0) begin
        active   <= 1'b1;
        cur_addr <= {job_addr[63:2], 2'b00};
        cur_rem  <= job_len;
      end

      // request issue
      if (issue) begin
        req_base[issue_cnt[TW-1:0]] <= alloc_ptr[RAW-1:0];
        req_dw0 [issue_cnt[TW-1:0]] <= cur_addr[6:2];
        req_len [issue_cnt[TW-1:0]] <= nlen;
        req_rem [issue_cnt[TW-1:0]] <= nlen;
        alloc_ptr <= alloc_ptr + {{(RAW-5){1'b0}}, nlen};
        issue_cnt <= issue_cnt + 1'b1;
        cur_addr  <= cur_addr + {56'd0, nlen, 2'b00};
        cur_rem   <= cur_rem - {6'd0, nlen};
        if (cur_rem == {6'd0, nlen}) active <= 1'b0;
      end

      // completion data
      if (cpl_hdr_valid) ctag <= htag;
      if (cpl_data_valid) begin
        cidx <= widx + {{(RAW-2){1'b0}}, cpl_data_cnt};
        req_rem[wtag] <= req_rem[wtag] - {4'd0, cpl_data_cnt};
      end else if (cpl_hdr_valid) begin
        cidx <= hidx;
        if (cpl_status != 3'b000) begin
          // unsuccessful completion: no data will come; release the request
          err           <= 1'b1;
          req_rem[htag] <= 6'd0;
        end
      end

      // in-order commit
      if (do_commit) begin
        commit_ptr <= commit_ptr + {{(RAW-5){1'b0}}, req_len[ctagc]};
        commit_cnt <= commit_cnt + 1'b1;
      end

      rd_ptr <= rd_ptr + {{(RAW-1){1'b0}}, pop};
    end
  end

  assign avail = commit_ptr - rd_ptr;
  assign idle  = !active && (outstanding == '0);

endmodule
