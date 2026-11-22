// deflate_enc.sv - zlib (RFC 1950) compressor, one record per start pulse.
//  Stage 1: LZ77 match finder. 64 KB history ring (32 KB window + 32 KB look-ahead),
//           4096-entry hash head table over 3-byte prefixes, greedy, one candidate,
//           matches 3..258 bytes, distance 1..32768.
//  Stage 2: Huffman coding with the fixed code (one BTYPE=01 block, BFINAL=1), packed
//           LSB first, then the Adler-32 trailer.
//  Byte stream in, byte stream out (valid/ready). All large memories are synchronous-read.
module deflate_enc #(
  parameter int LW = 21                    // record length width (2^20 = 1 MB fits)
)(
  input  logic          clk,
  input  logic          rst,

  input  logic          start,             // pulse while idle
  input  logic [LW-1:0] total,             // record length in bytes, >= 1

  input  logic          in_valid,
  output logic          in_ready,
  input  logic [7:0]    in_byte,

  output logic          out_valid,
  input  logic          out_ready,
  output logic [7:0]    out_byte,

  output logic          busy,
  output logic          done,              // one-cycle pulse after the last output byte
  output logic [23:0]   csize,             // output bytes so far (final at done)
  output logic [31:0]   adler              // Adler-32 of the input (final at done)
);

  // ---------------- length / distance code tables (RFC 1951 3.2.5) ----------------
  function automatic logic [8:0] len_base(input int i);
    case (i)
      0: return 3;   1: return 4;   2: return 5;   3: return 6;   4: return 7;   5: return 8;
      6: return 9;   7: return 10;  8: return 11;  9: return 13;  10: return 15; 11: return 17;
      12: return 19; 13: return 23; 14: return 27; 15: return 31; 16: return 35; 17: return 43;
      18: return 51; 19: return 59; 20: return 67; 21: return 83; 22: return 99; 23: return 115;
      24: return 131; 25: return 163; 26: return 195; 27: return 227; default: return 258;
    endcase
  endfunction
  function automatic int len_extra(input int i);
    if (i < 8 || i == 28) return 0;
    return (i - 4) / 4;
  endfunction
  function automatic logic [15:0] dist_base(input int i);
    case (i)
      0: return 1;     1: return 2;     2: return 3;     3: return 4;     4: return 5;
      5: return 7;     6: return 9;     7: return 13;    8: return 17;    9: return 25;
      10: return 33;   11: return 49;   12: return 65;   13: return 97;   14: return 129;
      15: return 193;  16: return 257;  17: return 385;  18: return 513;  19: return 769;
      20: return 1025; 21: return 1537; 22: return 2049; 23: return 3073; 24: return 4097;
      25: return 6145; 26: return 8193; 27: return 12289; 28: return 16385; default: return 24577;
    endcase
  endfunction
  function automatic int dist_extra(input int i);
    if (i < 4) return 0;
    return (i - 2) / 2;
  endfunction
  function automatic logic [8:0] rev9(input logic [8:0] v, input int n);
    logic [8:0] r;
    r = '0;
    for (int k = 0; k < 9; k++) if (k < n) r[k] = v[n - 1 - k];
    return r;
  endfunction

  // bits for a literal / end-of-block (sym 0..256)
  function automatic logic [37:0] lit_bits(input logic [8:0] sym);   // {n[5:0], bits[31:0]}
    logic [8:0] c; int n;
    if (sym < 144)      begin c = 9'h030 + sym;         n = 8; end
    else if (sym < 256) begin c = 9'h190 + (sym - 144); n = 9; end
    else                begin c = 9'h000 + (sym - 256); n = 7; end
    return {6'(n), 23'd0, rev9(c, n)};
  endfunction

  // bits for a match: length 3..258, distance 1..32768
  function automatic logic [37:0] match_bits(input logic [8:0] mlen, input logic [15:0] dst);
    int li, di, ln, le, de, n;
    logic [8:0]  lc;
    logic [31:0] b;
    li = 28;
    for (int i = 27; i >= 0; i--) if (mlen < len_base(i + 1)) li = i;
    if (mlen == 258) li = 28;
    di = 29;
    for (int i = 28; i >= 0; i--) if (dst < dist_base(i + 1)) di = i;
    if (li + 257 < 280) begin lc = 9'(li + 257 - 256);       ln = 7; end
    else                begin lc = 9'h0C0 + 9'(li + 257 - 280); ln = 8; end
    le = len_extra(li);
    de = dist_extra(di);
    b  = 32'(rev9(lc, ln));
    b  = b | (32'(mlen - len_base(li)) << ln);
    b  = b | (32'(rev9(9'(di), 5)) << (ln + le));
    b  = b | (32'(dst - dist_base(di)) << (ln + le + 5));
    n  = ln + le + 5 + de;
    return {6'(n), b};
  endfunction

  // ---------------- bit emitter ----------------
  logic [63:0] bbuf;
  logic [6:0]  bcnt;
  logic        put;
  logic [31:0] put_bits;
  logic [5:0]  put_n;
  wire         put_ok = (bcnt <= 7'd32);
  assign out_valid = (bcnt >= 7'd8);
  assign out_byte  = bbuf[7:0];
  wire         pop = out_valid && out_ready;

  always_ff @(posedge clk) begin
    if (rst || start) begin
      bbuf  <= '0;
      bcnt  <= '0;
      csize <= '0;
    end else begin
      logic [63:0] nb;
      logic [6:0]  nc;
      nb = pop ? (bbuf >> 8) : bbuf;
      nc = pop ? (bcnt - 7'd8) : bcnt;
      if (put) begin
        nb = nb | (64'(put_bits) << nc);
        nc = nc + 7'(put_n);
      end
      bbuf <= nb;
      bcnt <= nc;
      if (pop) csize <= csize + 24'd1;
    end
  end

  // ---------------- history ring (two copies, one read port each) and hash heads ----------------
  logic [LW-1:0] wp;                 // bytes received
  logic [LW-1:0] p;                  // current position
  logic [LW-1:0] tot;
  logic [15:0]   a_ra, b_ra;
  logic [7:0]    a_q, b_q;
  logic          hwe;
  logic [11:0]   h_wa, h_ra;
  logic [LW-1:0] h_wd, h_q;

  typedef enum logic [3:0] {
    S_IDLE, S_CLR, S_HDR, S_WAIT, S_RD2, S_RD3, S_HD, S_CMP, S_EMIT, S_EOB, S_FLUSH, S_TRL, S_DRAIN
  } st_t;
  st_t st;

  assign in_ready = busy && (st != S_IDLE) && (wp < tot) && ((wp - p) < LW'(32768));
  wire   take     = in_valid && in_ready;

  ram_1r1w #(.DW(8), .AW(16)) u_hist_a (.clk, .we(take), .waddr(wp[15:0]), .wdata(in_byte), .raddr(a_ra), .rdata(a_q));
  ram_1r1w #(.DW(8), .AW(16)) u_hist_b (.clk, .we(take), .waddr(wp[15:0]), .wdata(in_byte), .raddr(b_ra), .rdata(b_q));
  ram_1r1w #(.DW(LW), .AW(12)) u_head  (.clk, .we(hwe), .waddr(h_wa), .wdata(h_wd), .raddr(h_ra), .rdata(h_q));

  // ---------------- Adler-32 of the input ----------------
  logic [15:0] s1, s2;
  assign adler = {s2, s1};
  always_ff @(posedge clk) begin
    if (rst || start) begin
      s1 <= 16'd1;
      s2 <= 16'd0;
    end else if (take) begin
      logic [16:0] t1, t2;
      t1 = {1'b0, s1} + 17'(in_byte);
      if (t1 >= 17'd65521) t1 = t1 - 17'd65521;
      t2 = {1'b0, s2} + t1;
      if (t2 >= 17'd65521) t2 = t2 - 17'd65521;
      s1 <= t1[15:0];
      s2 <= t2[15:0];
    end
  end

  // ---------------- match finder ----------------
  logic [7:0]    b0, b1;
  logic [LW-1:0] cpos;               // candidate position
  logic [8:0]    mlen, maxlen;
  logic [15:0]   mdist;
  logic [37:0]   pend;               // pending symbol {n, bits}
  logic [11:0]   clr_i;
  logic [11:0]   hreg;

  wire [11:0] hash = {b0, 4'd0} ^ {2'd0, b1, 2'd0} ^ {4'd0, a_q};

  // read addresses are combinational: the RAM output is valid in the next cycle
  always_comb begin
    a_ra = p[15:0];
    b_ra = p[15:0] + 16'd1;
    h_ra = hash;
    case (st)
      S_RD2: a_ra = p[15:0] + 16'd2;
      S_HD:  begin a_ra = 16'(h_q - 1'b1); b_ra = p[15:0]; end
      S_CMP: begin a_ra = 16'(cpos) + 16'(mlen) + 16'd1; b_ra = p[15:0] + 16'(mlen) + 16'd1; end
      default: ;
    endcase
  end

  wire [LW-1:0] avail   = wp - p;
  wire [LW-1:0] remain  = tot - p;
  wire          look_ok = (wp == tot) || (avail >= LW'(262));

  always_comb begin
    put      = 1'b0;
    put_bits = '0;
    put_n    = '0;
    case (st)
      S_HDR:   begin put = put_ok; put_bits = 32'h0003_0178; put_n = 6'd19; end   // 78 01, BFINAL=1, BTYPE=01
      S_EMIT:  begin put = put_ok; put_bits = pend[31:0]; put_n = pend[37:32]; end
      S_EOB:   begin put = put_ok; put_bits = 32'd0; put_n = 6'd7; end              // symbol 256
      S_FLUSH: begin put = put_ok && (bcnt < 7'd8); put_bits = 32'd0; put_n = 6'((8 - bcnt[2:0]) & 7); end
      S_TRL:   begin put = put_ok; put_bits = {adler[7:0], adler[15:8], adler[23:16], adler[31:24]}; put_n = 6'd32; end
      default: ;
    endcase
  end

  always_ff @(posedge clk) begin
    done <= 1'b0;
    hwe  <= 1'b0;
    if (rst) begin
      st   <= S_IDLE;
      busy <= 1'b0;
      wp   <= '0;
      p    <= '0;
      tot  <= '0;
    end else begin
      if (take) wp <= wp + 1'b1;
      case (st)
        S_IDLE: if (start) begin
          busy  <= 1'b1;
          tot   <= total;
          wp    <= '0;
          p     <= '0;
          clr_i <= '0;
          st    <= S_CLR;
        end
        S_CLR: begin
          hwe   <= 1'b1;
          h_wa  <= clr_i;
          h_wd  <= '0;
          clr_i <= clr_i + 12'd1;
          if (clr_i == 12'hFFF) st <= S_HDR;
        end
        S_HDR: if (put_ok) st <= S_WAIT;
        S_WAIT: begin
          if (p == tot) st <= S_EOB;
          else if (look_ok && (wp > p)) begin
            if (remain < LW'(3)) st <= S_RD3;     // a_q = hist[p] next cycle
            else                 st <= S_RD2;     // a_q, b_q = hist[p], hist[p+1]
          end
        end
        S_RD2: begin
          b0 <= a_q;
          b1 <= b_q;
          st <= S_RD3;                            // a_q = hist[p+2] next cycle
        end
        S_RD3: begin
          if (remain < LW'(3)) begin
            pend <= lit_bits({1'b0, a_q});
            p    <= p + 1'b1;
            st   <= S_EMIT;
          end else begin
            hreg <= hash;                         // h_q = head[hash] next cycle
            st   <= S_HD;
          end
        end
        S_HD: begin
          // h_q = head[hash] (position + 1, 0 = empty); replace it with p + 1
          hwe  <= 1'b1;
          h_wa <= hreg;
          h_wd <= p + 1'b1;
          if (h_q != '0 && (p - (h_q - 1'b1)) <= LW'(32768)) begin
            cpos   <= h_q - 1'b1;
            mdist  <= 16'(p - (h_q - 1'b1));
            mlen   <= '0;
            maxlen <= (remain > LW'(258)) ? 9'd258 : 9'(remain);
            st     <= S_CMP;
          end else begin
            pend <= lit_bits({1'b0, b0});
            p    <= p + 1'b1;
            st   <= S_EMIT;
          end
        end
        S_CMP: begin
          // a_q = hist[cpos + mlen], b_q = hist[p + mlen]
          if (a_q == b_q && mlen + 9'd1 < maxlen) begin
            mlen <= mlen + 9'd1;
          end else begin
            logic [8:0] fl;
            fl = (a_q == b_q) ? (mlen + 9'd1) : mlen;
            if (fl >= 9'd3) begin
              pend <= match_bits(fl, mdist);
              p    <= p + LW'(fl);
            end else begin
              pend <= lit_bits({1'b0, b0});
              p    <= p + 1'b1;
            end
            st <= S_EMIT;
          end
        end
        S_EMIT:  if (put_ok) st <= S_WAIT;
        S_EOB:   if (put_ok) st <= S_FLUSH;
        S_FLUSH: if (put_ok && bcnt < 7'd8) st <= S_TRL;
        S_TRL:   if (put_ok) st <= S_DRAIN;
        S_DRAIN: if (bcnt == 7'd0) begin
          busy <= 1'b0;
          done <= 1'b1;
          st   <= S_IDLE;
        end
        default: st <= S_IDLE;
      endcase
    end
  end

endmodule
