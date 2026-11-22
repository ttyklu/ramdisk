// inflate.sv - zlib (RFC 1950 / 1951) decompressor: stored, fixed and dynamic blocks.
//  * Byte stream in (in_eof: the supplier has no more bytes), byte stream out.
//  * Huffman decoding is canonical and bit-serial (one code bit per cycle), as in zlib's
//    puff.c: per-length counts plus a symbol table sorted by code.
//  * 32 KB history ring for back references (synchronous-read RAM with a write bypass).
//  * Checks: zlib header, block type, stored LEN/NLEN, over-subscribed code sets, invalid
//    codes and symbols, distances, truncated input and the Adler-32 trailer.
module inflate (
  input  logic        clk,
  input  logic        rst,

  input  logic        start,          // pulse while idle
  input  logic        in_valid,
  output logic        in_ready,
  input  logic [7:0]  in_byte,
  input  logic        in_eof,         // level: all input bytes have been offered

  output logic        out_valid,
  input  logic        out_ready,
  output logic [7:0]  out_byte,

  output logic        busy,
  output logic        done,           // one-cycle pulse when finished (ok or error)
  output logic [3:0]  err,            // 0 = ok (valid at done)
  output logic [23:0] consumed,       // input bytes that belonged to the stream (at done)
  output logic [23:0] produced        // output bytes
);
  localparam logic [3:0] E_OK = 4'd0, E_HDR = 4'd1, E_BTYPE = 4'd2, E_STORED = 4'd3,
                         E_CODES = 4'd4, E_SYM = 4'd5, E_DIST = 4'd6, E_TRUNC = 4'd7,
                         E_ADLER = 4'd8;

  typedef enum logic [4:0] {
    S_IDLE, S_ZHDR, S_BHDR, S_ST_ALIGN, S_ST_LEN, S_ST_COPY,
    S_DYN_HDR, S_CL_RD,
    S_BCNT, S_BOFF, S_BSYM,
    S_HUFF, S_LENS_SYM, S_LENS_REP, S_REPW, S_LENS_DONE, S_FIXFILL,
    S_LIT_SYM, S_LEXTRA, S_DIST_SYM, S_DEXTRA, S_COPY_RD, S_COPY,
    S_TR_ALIGN, S_TRAILER, S_FIN
  } st_t;
  st_t st, ret;

  // ---------------- registers ----------------
  logic [31:0] bitbuf;
  logic [5:0]  bitcnt;
  logic [23:0] fed;
  logic        last;
  logic [15:0] slen;
  logic [8:0]  hlit;
  logic [5:0]  hdist;
  logic [4:0]  hclen;
  logic [8:0]  li, ntot, dbi;              // lens index, lens count, dist lens base
  logic [1:0]  bpass;                      // table being built: 0 = code-length, 1 = lit/len, 2 = dist
  logic [8:0]  bi, bn;
  logic [15:0] code, first, index;
  logic [4:0]  dlen;
  logic [1:0]  dtab;
  logic [8:0]  sym;
  logic [8:0]  mlen;
  logic [15:0] mdist;
  logic [4:0]  dsel;
  logic [14:0] op;
  logic [23:0] nout;
  logic [7:0]  obyte;
  logic        ovalid;
  logic [15:0] s1, s2;

  logic [3:0]  lens [320];                 // lit/len lengths at 0.., dist lengths at dbi..
  logic [56:0] cl;                         // code-length code lengths, 3 bits each
  logic [8:0]  rep_n;
  logic [3:0]  rep_v;
  logic [8:0]  ccnt [16];
  logic [8:0]  lcnt [16];
  logic [8:0]  dcnt [16];
  logic [8:0]  offs [16];
  logic [4:0]  csym [19];
  logic [8:0]  lsym [288];
  logic [4:0]  dsym [32];

  assign out_valid = ovalid;
  assign out_byte  = obyte;
  assign produced  = nout;

  // ---------------- tables of RFC 1951 ----------------
  function automatic logic [8:0] lbase(input logic [4:0] i);
    case (i)
      0: return 3;   1: return 4;   2: return 5;   3: return 6;   4: return 7;   5: return 8;
      6: return 9;   7: return 10;  8: return 11;  9: return 13;  10: return 15; 11: return 17;
      12: return 19; 13: return 23; 14: return 27; 15: return 31; 16: return 35; 17: return 43;
      18: return 51; 19: return 59; 20: return 67; 21: return 83; 22: return 99; 23: return 115;
      24: return 131; 25: return 163; 26: return 195; 27: return 227; default: return 258;
    endcase
  endfunction
  function automatic logic [4:0] lext(input logic [4:0] i);
    if (i < 8 || i == 28) return 5'd0;
    return 5'((i - 4) >> 2);
  endfunction
  function automatic logic [15:0] dbase(input logic [4:0] i);
    case (i)
      0: return 1;     1: return 2;     2: return 3;     3: return 4;     4: return 5;
      5: return 7;     6: return 9;     7: return 13;    8: return 17;    9: return 25;
      10: return 33;   11: return 49;   12: return 65;   13: return 97;   14: return 129;
      15: return 193;  16: return 257;  17: return 385;  18: return 513;  19: return 769;
      20: return 1025; 21: return 1537; 22: return 2049; 23: return 3073; 24: return 4097;
      25: return 6145; 26: return 8193; 27: return 12289; 28: return 16385; default: return 24577;
    endcase
  endfunction
  function automatic logic [4:0] dext(input logic [4:0] i);
    if (i < 4) return 5'd0;
    return 5'((i - 2) >> 1);
  endfunction
  function automatic logic [4:0] clorder(input logic [4:0] i);
    case (i)
      0: return 16; 1: return 17; 2: return 18; 3: return 0;  4: return 8;  5: return 7;
      6: return 9;  7: return 6;  8: return 10; 9: return 5;  10: return 11; 11: return 4;
      12: return 12; 13: return 3; 14: return 13; 15: return 2; 16: return 14; 17: return 1;
      default: return 15;
    endcase
  endfunction

  // ---------------- output slot / history ring ----------------
  wire emit  = ovalid && out_ready;
  wire oslot = !ovalid || out_ready;        // a byte can be loaded this cycle

  logic        hwe;
  logic [14:0] hwa, hra;
  logic [7:0]  hwd, hq_raw, byp_d;
  logic        byp;
  ram_1r1w #(.DW(8), .AW(15)) u_hist (.clk, .we(hwe), .waddr(hwa), .wdata(hwd), .raddr(hra), .rdata(hq_raw));
  always_ff @(posedge clk) begin
    byp   <= hwe && (hwa == hra);
    byp_d <= hwd;
  end
  wire [7:0] hq = byp ? byp_d : hq_raw;

  // ---------------- per-state bit requests (shared by the bit reader and the FSM) ----------------
  logic [5:0] need;                         // bits the state needs
  logic       want;                         // state reads bits this cycle (if available)
  logic       bal;                          // drop to the next byte boundary
  logic       extra_ok;                     // other conditions for the state to proceed
  always_comb begin
    need = 6'd0; want = 1'b0; bal = 1'b0; extra_ok = 1'b1;
    case (st)
      S_ZHDR:     begin want = 1'b1; need = 6'd16; end
      S_BHDR:     begin want = 1'b1; need = 6'd3; end
      S_ST_ALIGN: bal = 1'b1;
      S_ST_LEN:   begin want = 1'b1; need = 6'd32; end
      S_ST_COPY:  begin want = (slen != 16'd0); need = 6'd8; extra_ok = oslot; end
      S_DYN_HDR:  begin want = 1'b1; need = 6'd14; end
      S_CL_RD:    begin want = 1'b1; need = 6'd3; end
      S_HUFF:     begin want = 1'b1; need = 6'd1; end
      S_LENS_REP: begin want = 1'b1; need = (sym == 9'd16) ? 6'd2 : (sym == 9'd17) ? 6'd3 : 6'd7; end
      S_LEXTRA:   begin want = 1'b1; need = 6'(lext(5'(sym - 9'd257))); end
      S_DEXTRA:   begin want = 1'b1; need = 6'(dext(dsel)); end
      S_TR_ALIGN: bal = 1'b1;
      S_TRAILER:  begin want = 1'b1; need = 6'd32; extra_ok = !ovalid; end
      default: ;
    endcase
  end
  wire tk      = want && (bitcnt >= need) && extra_ok;
  wire starved = want && (bitcnt < need) && in_eof && !in_valid;

  // ---------------- bit reader ----------------
  assign in_ready = busy && (bitcnt <= 6'd24) && (st != S_FIN);
  wire   fill     = in_valid && in_ready;
  always_ff @(posedge clk) begin
    if (rst || start) begin
      bitbuf <= '0;
      bitcnt <= '0;
      fed    <= '0;
    end else begin
      logic [31:0] b;
      logic [5:0]  c;
      b = bitbuf;
      c = bitcnt;
      if (tk)  begin b = (need == 6'd32) ? 32'd0 : (b >> need); c = c - need; end
      if (bal) begin b = b >> c[2:0]; c = c - {3'd0, c[2:0]}; end
      if (fill) begin b = b | (32'(in_byte) << c); c = c + 6'd8; end
      bitbuf <= b;
      bitcnt <= c;
      if (fill) fed <= fed + 24'd1;
    end
  end

  // ---------------- Adler-32 of the output ----------------
  always_ff @(posedge clk) begin
    if (rst || start) begin
      s1 <= 16'd1; s2 <= 16'd0;
    end else if (emit) begin
      logic [16:0] t1, t2;
      t1 = {1'b0, s1} + 17'(obyte);
      if (t1 >= 17'd65521) t1 = t1 - 17'd65521;
      t2 = {1'b0, s2} + t1;
      if (t2 >= 17'd65521) t2 = t2 - 17'd65521;
      s1 <= t1[15:0];
      s2 <= t2[15:0];
    end
  end

  // ---------------- table builder helpers ----------------
  function automatic logic [3:0] getlen(input logic [1:0] pass, input logic [8:0] i);
    if (pass == 2'd0) return {1'b0, cl[i*3 +: 3]};
    if (pass == 2'd1) return lens[i];
    return lens[dbi + i];
  endfunction

  logic [8:0] cur_cnt;
  always_comb begin
    case (dtab)
      2'd0:    cur_cnt = ccnt[dlen[3:0]];
      2'd1:    cur_cnt = lcnt[dlen[3:0]];
      default: cur_cnt = dcnt[dlen[3:0]];
    endcase
  end

  // history read address / write port (combinational: RAM data valid next cycle)
  logic copy_go;
  assign copy_go = (st == S_COPY) && oslot && (mlen != 9'd0);
  always_comb begin
    hra = op - mdist[14:0];
    if (copy_go) hra = op + 15'd1 - mdist[14:0];
    hwe = 1'b0;
    hwa = op;
    hwd = 8'd0;
    if (copy_go) begin
      hwe = 1'b1; hwd = hq;
    end else if (st == S_LIT_SYM && sym < 9'd256 && oslot) begin
      hwe = 1'b1; hwd = sym[7:0];
    end else if (st == S_ST_COPY && tk) begin
      hwe = 1'b1; hwd = bitbuf[7:0];
    end
  end

  // lens[] has a single write port
  logic       lwe;
  logic [8:0] lwa;
  logic [3:0] lwd;
  always_comb begin
    lwe = 1'b0; lwa = li; lwd = 4'd0;
    case (st)
      S_FIXFILL:  begin lwe = 1'b1; lwd = (li < 9'd144) ? 4'd8 : (li < 9'd256) ? 4'd9 : (li < 9'd280) ? 4'd7 : (li < 9'd288) ? 4'd8 : 4'd5; end
      S_LENS_SYM: begin lwe = (sym < 9'd16); lwd = sym[3:0]; end
      S_REPW:     begin lwe = (rep_n != 9'd0); lwd = rep_v; end
      default: ;
    endcase
  end
  always_ff @(posedge clk) if (lwe) lens[lwa] <= lwd;

  // ---------------- FSM ----------------
  task automatic start_decode(input logic [1:0] t, input st_t r);
    dtab  <= t;
    code  <= '0;
    first <= '0;
    index <= '0;
    dlen  <= 5'd1;
    ret   <= r;
    st    <= S_HUFF;
  endtask

  // after a build: code-length code -> decode lengths; lit/len -> build dist; dist -> decode data
  task automatic start_build(input logic [1:0] pass, input logic [8:0] n);
    bpass <= pass;
    bn    <= n;
    bi    <= '0;
    for (int k = 0; k < 16; k++) begin
      if (pass == 2'd0) ccnt[k] <= '0;
      if (pass == 2'd1) lcnt[k] <= '0;
      if (pass == 2'd2) dcnt[k] <= '0;
    end
    st <= S_BCNT;
  endtask

  task automatic fail(input logic [3:0] e);
    err <= e;
    st  <= S_FIN;
  endtask

  always_ff @(posedge clk) begin
    done <= 1'b0;
    if (emit) ovalid <= 1'b0;
    if (rst) begin
      st     <= S_IDLE;
      busy   <= 1'b0;
      err    <= E_OK;
      ovalid <= 1'b0;
    end else if (starved && st != S_FIN) begin
      fail(E_TRUNC);
    end else begin
      case (st)
        S_IDLE: if (start) begin
          busy   <= 1'b1;
          err    <= E_OK;
          op     <= '0;
          nout   <= '0;
          ovalid <= 1'b0;
          st     <= S_ZHDR;
        end

        // zlib header: CMF, FLG
        S_ZHDR: if (tk) begin
          if (bitbuf[3:0] != 4'd8 || bitbuf[7:4] > 4'd7 || bitbuf[13] ||
              ((32'(bitbuf[7:0]) * 256 + 32'(bitbuf[15:8])) % 31) != 0)
            fail(E_HDR);
          else
            st <= S_BHDR;
        end

        S_BHDR: if (tk) begin
          last <= bitbuf[0];
          case (bitbuf[2:1])
            2'd0: st <= S_ST_ALIGN;
            2'd1: begin
              // fixed code: lit 0-143:8, 144-255:9, 256-279:7, 280-287:8; dist 0-29:5
              dbi   <= 9'd288;
              hdist <= 6'd30;
              li    <= '0;
              st    <= S_FIXFILL;
            end
            2'd2: st <= S_DYN_HDR;
            default: fail(E_BTYPE);
          endcase
        end

        S_FIXFILL: begin
          li <= li + 9'd1;
          if (li == 9'd319) start_build(2'd1, 9'd288);
        end

        // ---- stored block ----
        S_ST_ALIGN: st <= S_ST_LEN;
        S_ST_LEN: if (tk) begin
          if (bitbuf[15:0] != ~bitbuf[31:16]) fail(E_STORED);
          else begin
            slen <= bitbuf[15:0];
            st   <= S_ST_COPY;
          end
        end
        S_ST_COPY: begin
          if (slen == 16'd0) st <= last ? S_TR_ALIGN : S_BHDR;
          else if (tk) begin
            ovalid <= 1'b1;
            obyte  <= bitbuf[7:0];
            op     <= op + 15'd1;
            nout   <= nout + 24'd1;
            slen   <= slen - 16'd1;
          end
        end

        // ---- dynamic block header ----
        S_DYN_HDR: if (tk) begin
          hlit  <= 9'(bitbuf[4:0]) + 9'd257;
          hdist <= 6'(bitbuf[9:5]) + 6'd1;
          hclen <= 5'(bitbuf[13:10]) + 5'd4;
          cl <= '0;
          li <= '0;
          if (bitbuf[4:0] > 5'd29 || bitbuf[9:5] > 5'd29) fail(E_CODES);
          else st <= S_CL_RD;
        end
        S_CL_RD: if (tk) begin
          logic [5:0] ci;
          ci = 6'(clorder(li[4:0])) * 6'd3;
          cl[ci +: 3] <= bitbuf[2:0];
          li <= li + 9'd1;
          if (li + 9'd1 == 9'(hclen)) start_build(2'd0, 9'd19);
        end

        // ---- generic canonical table build ----
        S_BCNT: begin
          if (bi == bn) begin
            for (int k = 0; k < 16; k++) offs[k] <= '0;
            st <= S_BOFF;
          end else begin
            logic [3:0] l;
            l = getlen(bpass, bi);
            if (bpass == 2'd0) ccnt[l] <= ccnt[l] + 9'd1;
            if (bpass == 2'd1) lcnt[l] <= lcnt[l] + 9'd1;
            if (bpass == 2'd2) dcnt[l] <= dcnt[l] + 9'd1;
            bi <= bi + 9'd1;
          end
        end
        S_BOFF: begin
          int left;
          logic [8:0] o;
          logic [8:0] c;
          left = 1;
          o    = '0;
          for (int k = 1; k < 16; k++) begin
            c = (bpass == 2'd0) ? ccnt[k] : (bpass == 2'd1) ? lcnt[k] : dcnt[k];
            offs[k] <= o;
            o    = o + c;
            left = left * 2 - int'(c);
            if (left < 0) left = -100000;
          end
          bi <= '0;
          if (left < 0) fail(E_CODES);
          else st <= S_BSYM;
        end
        S_BSYM: begin
          if (bi == bn) begin
            if (bpass == 2'd0) begin
              li <= '0;                                   // code-length code built: read lengths
              start_decode(2'd0, S_LENS_SYM);
            end else if (bpass == 2'd1) begin
              start_build(2'd2, 9'(hdist));               // lit/len built: build distances
            end else begin
              start_decode(2'd1, S_LIT_SYM);              // both built: decode block data
            end
          end else begin
            logic [3:0] l;
            l = getlen(bpass, bi);
            if (l != 4'd0) begin
              if (bpass == 2'd0) csym[offs[l]] <= bi[4:0];
              if (bpass == 2'd1) lsym[offs[l]] <= bi;
              if (bpass == 2'd2) dsym[offs[l]] <= bi[4:0];
              offs[l] <= offs[l] + 9'd1;
            end
            bi <= bi + 9'd1;
          end
        end

        // ---- one Huffman symbol, one bit per cycle ----
        S_HUFF: if (tk) begin
          logic [15:0] c;
          c = code | 16'(bitbuf[0]);
          if (c < first + 16'(cur_cnt)) begin
            logic [15:0] ix;
            ix = index + (c - first);
            case (dtab)
              2'd0:    sym <= 9'(csym[ix[4:0]]);
              2'd1:    sym <= lsym[ix[8:0]];
              default: sym <= 9'(dsym[ix[4:0]]);
            endcase
            st <= ret;
          end else if (dlen == 5'd15) begin
            fail(E_CODES);
          end else begin
            index <= index + 16'(cur_cnt);
            first <= (first + 16'(cur_cnt)) << 1;
            code  <= c << 1;
            dlen  <= dlen + 5'd1;
          end
        end

        // ---- dynamic code lengths ----
        S_LENS_SYM: begin
          if (sym < 9'd16) begin
            li <= li + 9'd1;
            if (li + 9'd1 == ntot) st <= S_LENS_DONE;
            else start_decode(2'd0, S_LENS_SYM);
          end else if (sym == 9'd16 && li == 9'd0) begin
            fail(E_CODES);
          end else begin
            st <= S_LENS_REP;
          end
        end
        S_LENS_REP: if (tk) begin
          logic [8:0] n;
          logic [3:0] v;
          if (sym == 9'd16)      begin n = 9'd3  + 9'(bitbuf[1:0]); v = lens[li - 9'd1]; end
          else if (sym == 9'd17) begin n = 9'd3  + 9'(bitbuf[2:0]); v = 4'd0; end
          else                   begin n = 9'd11 + 9'(bitbuf[6:0]); v = 4'd0; end
          if (li + n > ntot) fail(E_CODES);
          else begin
            rep_n <= n;
            rep_v <= v;
            st    <= S_REPW;
          end
        end
        S_REPW: begin
          if (rep_n == 9'd0) begin
            if (li == ntot) st <= S_LENS_DONE;
            else start_decode(2'd0, S_LENS_SYM);
          end else begin
            li    <= li + 9'd1;
            rep_n <= rep_n - 9'd1;
          end
        end
        S_LENS_DONE: begin
          if (lens[256] == 4'd0) fail(E_CODES);       // no end-of-block code
          else begin
            dbi <= hlit;
            start_build(2'd1, hlit);
          end
        end

        // ---- block data ----
        S_LIT_SYM: begin
          if (sym < 9'd256) begin
            if (oslot) begin
              ovalid <= 1'b1;
              obyte  <= sym[7:0];
              op     <= op + 15'd1;
              nout   <= nout + 24'd1;
              start_decode(2'd1, S_LIT_SYM);
            end
          end else if (sym == 9'd256) begin
            st <= last ? S_TR_ALIGN : S_BHDR;
          end else if (sym > 9'd285) begin
            fail(E_SYM);
          end else begin
            st <= S_LEXTRA;
          end
        end
        S_LEXTRA: if (tk) begin
          logic [4:0] i;
          i = 5'(sym - 9'd257);
          mlen <= lbase(i) + 9'((bitbuf & ((32'd1 << lext(i)) - 32'd1)));
          start_decode(2'd2, S_DIST_SYM);
        end
        S_DIST_SYM: begin
          if (sym > 9'd29) fail(E_SYM);
          else begin
            dsel <= sym[4:0];
            st   <= S_DEXTRA;
          end
        end
        S_DEXTRA: if (tk) begin
          logic [15:0] d;
          d = dbase(dsel) + 16'((bitbuf & ((32'd1 << dext(dsel)) - 32'd1)));
          mdist <= d;
          if (24'(d) > nout || d > 16'd32768) fail(E_DIST);
          else st <= S_COPY_RD;
        end
        S_COPY_RD: st <= S_COPY;                // hq = hist[op - dist] next cycle
        S_COPY: begin
          if (mlen == 9'd0) start_decode(2'd1, S_LIT_SYM);
          else if (copy_go) begin
            ovalid <= 1'b1;
            obyte  <= hq;
            op     <= op + 15'd1;
            nout   <= nout + 24'd1;
            mlen   <= mlen - 9'd1;
          end
        end

        // ---- trailer ----
        S_TR_ALIGN: st <= S_TRAILER;
        S_TRAILER: if (tk) begin
          if ({bitbuf[7:0], bitbuf[15:8], bitbuf[23:16], bitbuf[31:24]} != {s2, s1}) err <= E_ADLER;
          st <= S_FIN;
        end

        S_FIN: if (!ovalid) begin
          consumed <= fed - 24'(bitcnt[5:3]);
          busy     <= 1'b0;
          done     <= 1'b1;
          st       <= S_IDLE;
        end
        default: st <= S_IDLE;
      endcase
    end
  end

  assign ntot = hlit + 9'(hdist);

endmodule
