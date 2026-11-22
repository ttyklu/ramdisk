// tb_codec.sv - unit testbench for deflate_enc and inflate, driven by scripts/codec_test.py
//   +mode=deflate +in=<hex file> +n=<bytes> +out=<hex file>
//   +mode=inflate +in=<hex file> +n=<bytes> +out=<hex file>
//   +mode=round   +in=<hex file> +n=<bytes>          (deflate -> inflate, compared in the TB)
//   +stall=<0..99>  percent of cycles the source / sink stalls (default 20)
`timescale 1ns/1ps
module tb_codec;
  logic clk = 1'b0;
  always #2 clk = ~clk;
  logic rst;

  logic [7:0] src [1 << 21];
  int         n, stall, fo;
  string      mode, fin, fout;

  // deflate
  logic        d_start, d_in_valid, d_in_ready, d_out_valid, d_out_ready, d_busy, d_done;
  logic [7:0]  d_in_byte, d_out_byte;
  logic [23:0] d_csize;
  logic [31:0] d_adler;
  deflate_enc u_def (
    .clk, .rst, .start(d_start), .total(21'(n)),
    .in_valid(d_in_valid), .in_ready(d_in_ready), .in_byte(d_in_byte),
    .out_valid(d_out_valid), .out_ready(d_out_ready), .out_byte(d_out_byte),
    .busy(d_busy), .done(d_done), .csize(d_csize), .adler(d_adler)
  );

  // inflate
  logic        i_start, i_in_valid, i_in_ready, i_in_eof, i_out_valid, i_out_ready, i_busy, i_done;
  logic [7:0]  i_in_byte, i_out_byte;
  logic [3:0]  i_err;
  logic [23:0] i_consumed, i_produced;
  inflate u_inf (
    .clk, .rst, .start(i_start),
    .in_valid(i_in_valid), .in_ready(i_in_ready), .in_byte(i_in_byte), .in_eof(i_in_eof),
    .out_valid(i_out_valid), .out_ready(i_out_ready), .out_byte(i_out_byte),
    .busy(i_busy), .done(i_done), .err(i_err), .consumed(i_consumed), .produced(i_produced)
  );

  int  sp;            // source pointer
  int  nout;
  bit  go_src;
  logic [7:0] rbuf [$];   // deflate output kept for round mode

  function automatic bit pass_cycle();
    return ($urandom_range(0, 99) >= stall);
  endfunction

  initial begin
    if (!$value$plusargs("mode=%s", mode)) mode = "deflate";
    if (!$value$plusargs("in=%s", fin)) $fatal(1, "no +in");
    if (!$value$plusargs("n=%d", n)) $fatal(1, "no +n");
    if (!$value$plusargs("stall=%d", stall)) stall = 20;
    if (!$value$plusargs("out=%s", fout)) fout = "";
    $readmemh(fin, src, 0, n - 1);
    fo = 0;
    if (fout != "") fo = $fopen(fout, "w");
    rst = 1'b1;
    d_start = 1'b0; i_start = 1'b0;
    repeat (5) @(posedge clk);
    rst = 1'b0;
    @(posedge clk);
    if (mode == "inflate") i_start = 1'b1; else d_start = 1'b1;
    if (mode == "round") i_start = 1'b1;
    @(posedge clk);
    d_start = 1'b0; i_start = 1'b0;
  end

  // ---- source: file -> deflate (deflate/round) or file -> inflate (inflate) ----
  bit src_to_inf;
  assign src_to_inf = (mode == "inflate");
  logic       s_valid;
  logic [7:0] s_byte;
  logic       s_ready;
  always_comb begin
    s_ready = src_to_inf ? i_in_ready : d_in_ready;
  end
  always @(posedge clk) begin
    if (rst) begin
      s_valid <= 1'b0;
      sp      <= 0;
    end else begin
      if (s_valid && s_ready) s_valid <= 1'b0;
      if ((!s_valid || s_ready) && sp < n && pass_cycle()) begin
        s_valid <= 1'b1;
        s_byte  <= src[sp];
        sp      <= sp + 1;
      end
    end
  end
  assign d_in_valid = !src_to_inf && s_valid;
  assign d_in_byte  = s_byte;

  // ---- inflate input: from file (inflate) or from deflate output (round) ----
  logic r_ready;
  assign i_in_valid  = src_to_inf ? s_valid : (mode == "round" ? d_out_valid : 1'b0);
  assign i_in_byte   = src_to_inf ? s_byte  : d_out_byte;
  assign i_in_eof    = src_to_inf ? (sp >= n && !s_valid) : (mode == "round" ? !d_busy : 1'b1);
  assign d_out_ready = (mode == "round") ? i_in_ready : r_ready;

  // ---- sinks ----
  always @(posedge clk) r_ready <= pass_cycle();
  assign i_out_ready = r_ready;

  int rp;
  int errors = 0;
  always @(posedge clk) begin
    if (rst) begin
      nout = 0; rp = 0;
    end else begin
      if (mode == "deflate" && d_out_valid && d_out_ready) begin
        if (fo) $fdisplay(fo, "%02x", d_out_byte);
        nout++;
      end
      if (mode != "deflate" && i_out_valid && i_out_ready) begin
        if (fo) $fdisplay(fo, "%02x", i_out_byte);
        if (mode == "round") begin
          if (rp >= n || i_out_byte != src[rp]) begin
            if (errors < 5) $display("ROUND mismatch at %0d", rp);
            errors++;
          end
          rp++;
        end
        nout++;
      end
    end
  end

  always @(posedge clk) begin
    if (mode == "deflate" && d_done) begin
      $display("RESULT deflate csize=%0d adler=%08x nout=%0d", d_csize, d_adler, nout);
      if (fo) $fclose(fo);
      $finish;
    end
    if (mode != "deflate" && i_done) begin
      if (mode == "round" && rp != n) errors++;
      $display("RESULT inflate err=%0d consumed=%0d produced=%0d nout=%0d csize=%0d adler=%08x errors=%0d",
               i_err, i_consumed, i_produced, nout, d_csize, d_adler, errors);
      if (fo) $fclose(fo);
      $finish;
    end
  end

  initial begin
    #(400ms);
    $display("RESULT timeout");
    $finish;
  end
endmodule
