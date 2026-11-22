// tb_comp.svh - compression-path tests, included inside tb_top (uses its host model and helpers).
// Commands and layouts: docs/compression_spec.md.

  localparam bit [7:0] OPC_C2S = 8'h81, OPC_COMMIT = 8'h84, OPC_DREAD = 8'h86,
                       OPC_FSEND = 8'h89, OPC_FRECV = 8'h8A;
  localparam int       REC_MAX   = 1 << REC_LOG2;
  localparam int       N_SLOT    = (64 << 20) >> (REC_LOG2 + 2);
  localparam bit [63:0] RES_BUF  = 64'h0000_0000_5000_0000;     // result records (MPTR)
  localparam bit [63:0] MD_BUF   = 64'h0000_0000_5000_1000;     // Decompress-Read descriptor
  localparam bit [63:0] SRC_RGN  = 64'h0000_0000_2000_0000;     // record buffers
  localparam bit [63:0] DST_RGN  = 64'h0000_0001_8000_0000;     // output buffers
  localparam bit [63:0] PAR_RGN  = 64'h0000_0000_6000_0000;     // FFE parameter data
  localparam bit [63:0] COMP_LBA = 64'd200000;                  // LBAs used by these tests

  byte unsigned rec [];          // current record
  logic [7:0]   zv_mem [1 << 22];  // zlib vectors file
  int           n_comp_checks = 0;

  function automatic int up4k(input int v);  return (v + 4095) & ~4095; endfunction
  function automatic int up128(input int v); return (v + 127) & ~127;   endfunction

  // ---- data ----
  function automatic void gen_rec(input int kind, input int n);
    string w [10] = '{"zfs", "record", "block", "nvme", "fpga", "the", "of", "data", "\n", "compress"};
    int i;
    rec = new[n];
    case (kind)
      0: for (int j = 0; j < n; j++) rec[j] = 8'h00;
      1: for (int j = 0; j < n; j++) rec[j] = 8'($urandom);
      2: begin
        i = 0;
        while (i < n) begin
          string s;
          s = w[$urandom_range(0, 9)];
          for (int k = 0; k < s.len() && i < n; k++) begin rec[i] = s[k]; i++; end
          if (i < n) begin rec[i] = " "; i++; end
        end
      end
      default: begin
        i = 0;
        while (i < n) begin
          byte unsigned b;
          int r;
          b = 8'($urandom);
          r = $urandom_range(1, 300);
          for (int k = 0; k < r && i < n; k++) begin rec[i] = b; i++; end
        end
      end
    endcase
  endfunction

  function automatic bit [31:0] adler32(input byte unsigned b []);
    int unsigned s1, s2;
    s1 = 1; s2 = 0;
    foreach (b[j]) begin
      s1 = (s1 + b[j]) % 65521;
      s2 = (s2 + s1) % 65521;
    end
    return {s2[15:0], s1[15:0]};
  endfunction

  // Fletcher-4 of n bytes read by a byte getter (DDR backdoor), {d, c, b, a}
  function automatic bit [7:0] ddr8(input bit [31:0] a);
    bit [63:0] w;
    w = u_ddr.rd64({a[31:3], 3'b000});
    return w[8*a[2:0] +: 8];
  endfunction
  function automatic bit [255:0] fl4_ddr(input bit [31:0] base, input int n);
    bit [63:0] fa, fb, fc, fd;
    fa = 0; fb = 0; fc = 0; fd = 0;
    for (int j = 0; j < n; j += 4) begin
      bit [31:0] w;
      w = {ddr8(base + 32'(j + 3)), ddr8(base + 32'(j + 2)), ddr8(base + 32'(j + 1)), ddr8(base + 32'(j))};
      fa += 64'(w); fb += fa; fc += fb; fd += fc;
    end
    return {fd, fc, fb, fa};
  endfunction
  function automatic bit [255:0] fl4_bytes(input byte unsigned b [], input int n);   // zero padded to n
    bit [63:0] fa, fb, fc, fd;
    fa = 0; fb = 0; fc = 0; fd = 0;
    for (int j = 0; j < n; j += 4) begin
      bit [31:0] w;
      w = 0;
      for (int k = 0; k < 4; k++) if (j + k < b.size()) w[8*k +: 8] = b[j + k];
      fa += 64'(w); fb += fa; fc += fb; fd += fc;
    end
    return {fd, fc, fb, fa};
  endfunction

  function automatic void cfail(input string m);
    fail({"COMP: ", m});
  endfunction

  // ---- host buffers ----
  task automatic put_buf(input bit [63:0] region, input byte unsigned b [], input int nbytes,
                         output bit [63:0] p1, output bit [63:0] p2);
    build_prp(region, nbytes, 0, p1, p2);
    for (int j = 0; j < nbytes; j += 4) begin
      bit [31:0] w;
      w = 0;
      for (int k = 0; k < 4; k++) if (j + k < b.size()) w[8*k +: 8] = b[j + k];
      hw32(xaddr(0, j), w);
    end
  endtask

  // ---- commands ----
  task automatic set_comp(input bit en, input int rs, output bit [14:0] st);
    sqe_t c; bit [31:0] dw0;
    c = new_cmd(8'h09, 32'd0); c[10] = 32'hC0; c[11] = {24'd0, 4'(rs), 3'd0, en};
    run_cmd(0, c, dw0, st);
  endtask

  task automatic c2s(input int q, input int n, input int thr, output bit [31:0] dw0, output bit [14:0] st);
    sqe_t c; bit [63:0] p1, p2;
    put_buf(SRC_RGN, rec, n, p1, p2);
    for (int k = 0; k < 16; k++) hw32(RES_BUF + 64'(4 * k), 32'hDEAD_BEEF);
    c = new_cmd(OPC_C2S, 32'd1);
    c[4] = RES_BUF[31:0]; c[5] = RES_BUF[63:32];
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = 32'(n); c[11] = 32'(thr);
    run_cmd(q, c, dw0, st);
  endtask

  task automatic commit(input int q, input bit [15:0] h, input bit [63:0] slba, input int nlb,
                        input bit discard, output bit [14:0] st);
    sqe_t c; bit [31:0] dw0;
    c = new_cmd(OPC_COMMIT, 32'd1);
    c[10] = slba[31:0]; c[11] = slba[63:32]; c[12] = 32'(nlb - 1); c[13] = {discard, 15'd0, h};
    run_cmd(q, c, dw0, st);
  endtask

  task automatic dread(input int q, input bit [63:0] slba, input int nlb, input int csize, input int ulen,
                       input bit nochk, input bit [255:0] expfl, output bit [14:0] st);
    sqe_t c; bit [31:0] dw0; bit [63:0] p1, p2;
    build_prp(DST_RGN, ulen, 0, p1, p2);
    for (int j = 0; j < ulen; j += 4) hw32(xaddr(0, j), 32'hA5A5_A5A5);
    for (int k = 0; k < 16; k++) hw32(MD_BUF + 64'(4 * k), (k >= 4 && k < 12) ? expfl[32*(k-4) +: 32] : 32'd0);
    c = new_cmd(OPC_DREAD, 32'd1);
    c[4] = MD_BUF[31:0]; c[5] = MD_BUF[63:32];
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = slba[31:0]; c[11] = slba[63:32]; c[12] = 32'(nlb - 1);
    c[13] = 32'(csize); c[14] = 32'(ulen); c[15] = {31'd0, nochk};
    run_cmd(q, c, dw0, st);
  endtask

  // compare the last dread / recv output (pages[] of the last build_prp) with b[]
  function automatic int out_mismatch(input int off, input byte unsigned b [], input int n);
    int bad;
    bad = 0;
    for (int j = 0; j < n; j++)
      if (hr8(xaddr(0, off + j)) != b[j]) begin
        if (bad < 3) $display("  out[%0d] = %02x, expected %02x", j, hr8(xaddr(0, off + j)), b[j]);
        bad++;
      end
    return bad;
  endfunction

  // front-door write of raw bytes (zero padded to whole blocks)
  task automatic write_raw(input int q, input bit [63:0] slba, input byte unsigned b [], output int nlb);
    sqe_t c; bit [31:0] dw0; bit [14:0] st; bit [63:0] p1, p2;
    nlb = up4k(b.size()) / 4096;
    put_buf(SRC_RGN, b, nlb * 4096, p1, p2);
    c = new_cmd(8'h01, 32'd1);
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = slba[31:0]; c[11] = slba[63:32]; c[12] = 32'(nlb - 1);
    run_cmd(q, c, dw0, st);
    if (st != 0) cfail($sformatf("raw write status %h", st));
  endtask

  // FFE-Send: [dest, source, function, FPD] descriptors, inline data at ioff
  task automatic ffe_send(input int q, input bit [15:0] lid, input bit [7:0] fn, input bit lba_src,
                          input int ioff, input bit dlba, input bit [63:0] dslba, input int dnlb,
                          input bit [63:0] sslba, input int snlb, input int thr, input byte unsigned pay [],
                          input bit [7:0] bad_type, output bit [31:0] dw0, output bit [14:0] st);
    byte unsigned pb [];
    int plen;
    sqe_t c; bit [63:0] p1, p2;
    plen = lba_src ? 256 : up128(ioff + pay.size());
    pb = new[plen];
    foreach (pb[j]) pb[j] = 0;
    // destination (01h)
    pb[0] = 8'h01; pb[1] = {7'd0, dlba};
    for (int k = 0; k < 8; k++) pb[12 + k] = dslba[8*k +: 8];
    pb[20] = 8'(dnlb - 1); pb[21] = 8'((dnlb - 1) >> 8);
    // source
    if (lba_src) begin
      pb[64] = 8'h03; pb[68] = 8'd1;
      for (int k = 0; k < 8; k++) pb[72 + k] = sslba[8*k +: 8];
      pb[80] = 8'(snlb - 1); pb[81] = 8'((snlb - 1) >> 8);
    end else begin
      pb[64] = 8'h04;
      for (int k = 0; k < 4; k++) pb[68 + k] = 8'(pay.size() >> (8 * k));
      for (int k = 0; k < 4; k++) pb[72 + k] = 8'(ioff >> (8 * k));
    end
    // function descriptor
    pb[128] = (bad_type != 0) ? bad_type : fn; pb[134] = 8'd1; pb[144] = 8'd1;
    // function parameter descriptor
    pb[192] = 8'hFE;
    if (fn == 8'h82) begin pb[197] = 8'h01; pb[198] = 8'd6; pb[199] = 8'(thr); end
    else             pb[196] = 8'h01;
    if (!lba_src) foreach (pay[j]) pb[ioff + j] = pay[j];
    put_buf(PAR_RGN, pb, plen, p1, p2);
    c = new_cmd(OPC_FSEND, 32'd1);
    c[2] = {16'd0, lid};
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = 32'(plen); c[11] = {16'd2, 16'd2}; c[12] = lba_src ? 32'd0 : 32'(pay.size());
    run_cmd(q, c, dw0, st);
  endtask

  task automatic ffe_recv(input int q, input bit [15:0] lid, input int len, output bit [31:0] dw0,
                          output bit [14:0] st);
    sqe_t c; bit [63:0] p1, p2;
    build_prp(DST_RGN, len, 0, p1, p2);
    for (int j = 0; j < len; j += 4) hw32(xaddr(0, j), 32'h5A5A_5A5A);
    c = new_cmd(OPC_FRECV, 32'd1);
    c[2] = {16'd0, lid};
    c[6] = p1[31:0]; c[7] = p1[63:32]; c[8] = p2[31:0]; c[9] = p2[63:32];
    c[10] = 32'(len);
    run_cmd(q, c, dw0, st);
  endtask

  // ---- one record through the two-phase write and back ----
  // returns the stored extent's facts for later tests
  task automatic comp_round_trip(input int kind, input int n, input int thr, input bit [63:0] slba,
                                 output bit inc, output int csize, output int psize, output bit [255:0] fl);
    bit [31:0] dw0, r [16];
    bit [14:0] st;
    bit [15:0] h;
    gen_rec(kind, n);
    c2s($urandom_range(1, 2), n, thr, dw0, st);
    n_comp_checks++;
    if (st != 0) begin cfail($sformatf("C2S kind=%0d n=%0d status %h", kind, n, st)); return; end
    for (int k = 0; k < 16; k++) r[k] = hr32(RES_BUF + 64'(4 * k));
    h     = r[0][15:0];
    inc   = r[0][16];
    csize = int'(r[2]);
    psize = int'(r[3]);
    fl    = {r[11], r[10], r[9], r[8], r[7], r[6], r[5], r[4]};
    if (dw0[15:0] != h || dw0[16] != inc) cfail("C2S CQE DW0 does not match the result record");
    if (!r[0][17] || r[0][18]) cfail("result flags");
    if (r[1] != 32'(n)) cfail($sformatf("LSIZE %0d", r[1]));
    if (psize != up4k(inc ? n : csize)) cfail($sformatf("PSIZE %0d for CSIZE %0d", psize, csize));
    if (r[12] != adler32(rec)) cfail($sformatf("Adler-32 %h, expected %h", r[12], adler32(rec)));
    if (inc && csize != n) cfail("INCOMP with CSIZE != LSIZE");
    if (thr == 0 && !inc && csize > n - n / 8) cfail("kept compressed above the 12.5% rule");
    if (thr == 0 && inc && kind == 0) cfail("zeros reported incompressible");
    commit($urandom_range(1, 2), h, slba, psize / 4096, 1'b0, st);
    if (st != 0) begin cfail($sformatf("Commit status %h", st)); return; end
    // stored extent (backdoor): checksum, and raw data or a zlib header
    if (fl4_ddr(32'(slba << 12), psize) != fl) cfail($sformatf("Fletcher-4 of the stored extent (kind=%0d n=%0d inc=%0d)", kind, n, inc));
    if (inc) begin
      for (int j = 0; j < psize; j++)
        if (ddr8(32'(slba << 12) + 32'(j)) != ((j < n) ? rec[j] : 8'h00)) begin
          cfail($sformatf("stored raw byte %0d", j)); break;
        end
    end else begin
      if (ddr8(32'(slba << 12)) != 8'h78) cfail("stored extent has no zlib header");
      for (int j = csize; j < psize; j++)
        if (ddr8(32'(slba << 12) + 32'(j)) != 8'h00) begin cfail("padding not zero"); break; end
      dread($urandom_range(1, 2), slba, psize / 4096, csize, n, 1'b0, fl, st);
      if (st != 0) cfail($sformatf("Decompress-Read status %h (kind=%0d n=%0d)", st, kind, n));
      else if (out_mismatch(0, rec, n) != 0) cfail($sformatf("Decompress-Read data (kind=%0d n=%0d)", kind, n));
    end
    $display("[%0t] comp kind=%0d n=%0d -> csize=%0d psize=%0d %s", $time, kind, n, csize, psize, inc ? "INCOMP" : "");
  endtask

  // =====================================================================
  task automatic run_comp_tests();
    bit [31:0]  dw0, r [16];
    bit [14:0]  st;
    sqe_t       c;
    bit         inc;
    int         csize, psize, nlb;
    bit [255:0] fl;
    bit [63:0]  lba;
    bit [15:0]  hs [$];
    string      zf;

    // ---- RAW by default ----
    c = new_cmd(8'h0A, 32'd0); c[10] = 32'hC0;
    run_cmd(0, c, dw0, st);
    if (st != 0 || dw0[0] != 1'b0 || dw0[15:12] != 4'(REC_LOG2 - 17) || dw0[31:16] != 16'(N_SLOT))
      cfail($sformatf("Get Features C0h dw0=%h st=%h", dw0, st));
    gen_rec(2, 4096);
    c2s(1, 4096, 0, dw0, st);
    if (st[10:0] != ST_CMD_SEQ) cfail($sformatf("C2S while RAW: status %h", st));
    ffe_send(1, 16'h77, 8'h82, 1'b0, 256, 1'b0, 0, 1, 0, 1, 0, rec, 8'h00, dw0, st);
    if (st[10:0] != ST_CMD_SEQ) cfail($sformatf("FFE-Send while RAW: status %h", st));
    set_comp(1'b1, REC_LOG2 - 17 + 1, st);
    if (st[10:0] != ST_INVALID_FIELD) cfail("record size above the build limit accepted");
    set_comp(1'b1, REC_LOG2 - 17, st);
    if (st != 0) cfail("enable compression");

    // ---- two-phase write round trips ----
    lba = COMP_LBA;
    for (int kind = 0; kind < 4; kind++) begin
      for (int si = 0; si < 4; si++) begin
        int n;
        n = (si == 0) ? 512 : (si == 1) ? 4096 : (si == 2) ? 3 * 4096 + 1536 : REC_MAX;
        comp_round_trip(kind, n, 0, lba, inc, csize, psize, fl);
        lba += 64'(REC_MAX / 4096 + 1);
      end
    end
    // threshold: 50% makes text incompressible, 100% keeps it compressed
    comp_round_trip(2, 65536, 30, lba, inc, csize, psize, fl);
    if (!inc) cfail("30% threshold should mark text incompressible");
    lba += 64'(REC_MAX / 4096 + 1);
    comp_round_trip(2, 65536, 100, lba, inc, csize, psize, fl);
    if (inc) cfail("100% threshold should keep text compressed");

    // ---- Decompress-Read errors (on the last extent: text, compressed) ----
    dread(1, lba, psize / 4096, csize, 65536, 1'b0, ~fl, st);
    if (st[10:0] != ST_CKSUM) cfail($sformatf("bad checksum: status %h", st));
    dread(1, lba, psize / 4096, csize, 65536, 1'b1, ~fl, st);
    if (st != 0) cfail($sformatf("NOCHK: status %h", st));
    dread(2, lba, psize / 4096, csize, 65536 - 4096, 1'b0, fl, st);
    if (st[10:0] != ST_LEN) cfail($sformatf("short ULEN: status %h", st));
    dread(2, lba, psize / 4096, csize, 65536 + 4096, 1'b0, fl, st);
    if (st[10:0] != ST_LEN) cfail($sformatf("long ULEN: status %h", st));
    u_ddr.mem[32'((lba << 12) + 64) >> 3] = 64'hFFFF_FFFF_FFFF_FFFF;   // corrupt the stream
    dread(2, lba, psize / 4096, csize, 65536, 1'b1, fl, st);
    if (st[10:0] != ST_DECOMP) cfail($sformatf("corrupt stream: status %h", st));
    dread(2, lba, psize / 4096, csize, 65536, 1'b0, fl, st);
    if (st[10:0] != ST_CKSUM) cfail($sformatf("corrupt stream with check: status %h", st));
    lba += 64'(REC_MAX / 4096 + 1);
    n_comp_checks += 6;

    // ---- zlib streams written by software (Python zlib: stored, fixed, dynamic) ----
    if ($value$plusargs("zvec=%s", zf)) begin
      int p, nv;
      byte unsigned raw [];
      $readmemh(zf, zv_mem);
      p = 0; nv = 0;
      forever begin
        int ul, cl;
        byte unsigned z [];
        ul = {zv_mem[p + 3], zv_mem[p + 2], zv_mem[p + 1], zv_mem[p]};
        cl = {zv_mem[p + 7], zv_mem[p + 6], zv_mem[p + 5], zv_mem[p + 4]};
        p += 8;
        if (ul == 0) break;
        rec = new[ul]; z = new[cl];
        foreach (rec[j]) rec[j] = zv_mem[p + j];
        p += ul;
        foreach (z[j]) z[j] = zv_mem[p + j];
        p += cl;
        if (ul > REC_MAX) continue;
        write_raw(1, lba, z, nlb);
        dread(2, lba, nlb, cl, ul, 1'b0, fl4_bytes(z, nlb * 4096), st);
        n_comp_checks++;
        if (st != 0) cfail($sformatf("zvec %0d (ulen %0d clen %0d): status %h", nv, ul, cl, st));
        else if (out_mismatch(0, rec, ul) != 0) cfail($sformatf("zvec %0d data", nv));
        // the same stream through FFE 83h with an inline source
        if (ul <= 65536) begin
          ffe_send(1, 16'h300 + 16'(nv), 8'h83, 1'b0, 256 + 4 * (nv % 3), 1'b0, 0, 1, 0, 1, 0, z, 8'h00, dw0, st);
          if (st != 0) cfail($sformatf("FFE 83h inline zvec %0d: status %h", nv, st));
          ffe_recv(2, 16'h300 + 16'(nv), 64 + ((ul + 3) & ~3), dw0, st);
          if (st != 0 || dw0 != 32'(64 + ((ul + 3) & ~3))) cfail($sformatf("FFE-Receive zvec %0d: %h %h", nv, st, dw0));
          else if (out_mismatch(64, rec, ul) != 0) cfail($sformatf("FFE 83h zvec %0d data", nv));
          n_comp_checks++;
        end
        lba += 64'(REC_MAX / 4096 + 1);
        nv++;
      end
      $display("[%0t] %0d software zlib streams decoded", $time, nv);
    end else begin
      $display("NOTE: +zvec not given, software zlib streams skipped");
    end

    // ---- staging slots: fill, full, discard ----
    gen_rec(2, 512);
    hs.delete();
    for (int k = 0; k < N_SLOT; k++) begin
      c2s(1 + k % 2, 512, 0, dw0, st);
      if (st != 0) begin cfail($sformatf("C2S %0d status %h", k, st)); break; end
      hs.push_back(dw0[15:0]);
    end
    c2s(1, 512, 0, dw0, st);
    if (st[10:0] != ST_STAGING_FULL) cfail($sformatf("staging full: status %h", st));
    commit(1, hs[0], lba, 2, 1'b0, st);                     // NLB does not match PSIZE
    if (st[10:0] != ST_INVALID_FIELD) cfail($sformatf("commit NLB mismatch: status %h", st));
    foreach (hs[k]) begin
      commit(2, hs[k], 0, 1, 1'b1, st);
      if (st != 0) cfail($sformatf("discard %0d: status %h", k, st));
    end
    commit(1, hs[0], lba, 1, 1'b0, st);
    if (st[10:0] != ST_BAD_HANDLE) cfail($sformatf("commit freed handle: status %h", st));
    commit(1, 16'hFFFF, lba, 1, 1'b0, st);
    if (st[10:0] != ST_BAD_HANDLE) cfail($sformatf("commit bad handle: status %h", st));
    n_comp_checks += 5;

    // ---- FFE-Send / FFE-Receive ----
    begin
      bit [63:0]  l82, l83;
      int         cs, ps, n;
      bit [255:0] f;
      byte unsigned pay [];
      n = (REC_MAX < 65536) ? REC_MAX : 65536;
      // reference: the same record through Compress-to-Staging + Commit
      l82 = lba; lba += 64'(REC_MAX / 4096 + 1);
      comp_round_trip(3, n, 0, l82, inc, cs, ps, f);
      pay = rec;
      // 82h, inline, no destination: Receive returns the CRB and the stored bytes
      ffe_send(1, 16'h0101, 8'h82, 1'b0, 256, 1'b0, 0, 1, 0, 1, 0, pay, 8'h00, dw0, st);
      if (st != 0) cfail($sformatf("FFE 82h status %h", st));
      ffe_recv(2, 16'h0101, 64 + ps + 4096, dw0, st);
      for (int k = 0; k < 16; k++) r[k] = hr32(xaddr(0, 4 * k));
      if (st != 0 || dw0 != 32'(64 + ps)) cfail($sformatf("FFE-Receive 82h: st %h bytes %0d", st, dw0));
      if (r[1] != 32'(n) || r[2] != 32'(cs) || r[3] != 32'(ps) || r[13][15:0] != 16'h0101 ||
          {r[11], r[10], r[9], r[8], r[7], r[6], r[5], r[4]} != f)
        cfail("FFE 82h CRB differs from the Compress-to-Staging result");
      for (int j = 0; j < ps; j++)
        if (hr8(xaddr(0, 64 + j)) != ddr8(32'(l82 << 12) + 32'(j))) begin
          cfail($sformatf("FFE 82h stored bytes: [%0d] = %02x, expected %02x (of %0d)", j, hr8(xaddr(0, 64 + j)), ddr8(32'(l82 << 12) + 32'(j)), ps));
          break;
        end
      // 82h, inline at an unaligned (DW) offset, destination LBA: Receive returns the CRB only
      l83 = lba; lba += 64'(REC_MAX / 4096 + 1);
      ffe_send(2, 16'h0102, 8'h82, 1'b0, 256 + 12, 1'b1, l83, ps / 4096, 0, 1, 0, pay, 8'h00, dw0, st);
      if (st != 0) cfail($sformatf("FFE 82h + dest status %h", st));
      for (int j = 0; j < ps; j++)
        if (ddr8(32'(l83 << 12) + 32'(j)) != ddr8(32'(l82 << 12) + 32'(j))) begin cfail("FFE 82h destination bytes"); break; end
      ffe_recv(1, 16'h0102, 4096, dw0, st);
      if (st != 0 || dw0 != 32'd64) cfail($sformatf("FFE-Receive after LBA destination: st %h bytes %0d", st, dw0));
      // 83h, LBA source: decompress the extent, Receive the record
      ffe_send(1, 16'h0103, 8'h83, 1'b1, 0, 1'b0, 0, 1, l83, ps / 4096, 0, pay, 8'h00, dw0, st);
      if (st != 0) cfail($sformatf("FFE 83h status %h", st));
      ffe_recv(2, 16'h0103, 64 + n, dw0, st);
      for (int k = 0; k < 16; k++) r[k] = hr32(xaddr(0, 4 * k));
      if (st != 0 || dw0 != 32'(64 + n)) cfail($sformatf("FFE-Receive 83h: st %h bytes %0d", st, dw0));
      if (!r[0][18] || r[1] != 32'(n) || r[2] != 32'(cs)) cfail($sformatf("FFE 83h CRB flags %h lsize %0d csize %0d", r[0], r[1], r[2]));
      if (out_mismatch(64, pay, n) != 0) cfail("FFE 83h data");
      // errors
      ffe_recv(1, 16'h0999, 4096, dw0, st);
      if (st[10:0] != ST_BAD_HANDLE) cfail($sformatf("Receive unknown List ID: status %h", st));
      ffe_send(1, 16'h0104, 8'h82, 1'b0, 256, 1'b0, 0, 1, 0, 1, 0, pay, 8'h07, dw0, st);
      if (st[10:0] != ST_INVALID_FIELD) cfail($sformatf("FFE bad descriptor type: status %h", st));
      ffe_send(1, 16'h0105, 8'h82, 1'b0, 256, 1'b0, 0, 1, 0, 1, 0, pay, 8'h00, dw0, st);
      ffe_send(2, 16'h0105, 8'h82, 1'b0, 256, 1'b0, 0, 1, 0, 1, 0, pay, 8'h00, dw0, st);
      if (st[10:0] != ST_INVALID_FIELD) cfail($sformatf("FFE duplicate List ID: status %h", st));
      ffe_recv(1, 16'h0105, 64, dw0, st);                    // CRB only, frees the slot
      if (st != 0 || dw0 != 32'd64) cfail("FFE-Receive CRB only");
      n_comp_checks += 10;
    end

    // ---- back to RAW ----
    set_comp(1'b0, 0, st);
    gen_rec(2, 4096);
    c2s(2, 4096, 0, dw0, st);
    if (st[10:0] != ST_CMD_SEQ) cfail($sformatf("C2S after disabling: status %h", st));
    $display("[%0t] compression tests: %0d checks", $time, n_comp_checks);
  endtask
