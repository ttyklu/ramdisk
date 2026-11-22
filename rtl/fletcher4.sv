// fletcher4.sv - ZFS Fletcher-4 over 32-bit little-endian words, 0..2 words per cycle.
//   a += w; b += a; c += b; d += c   (all 64-bit, wrapping)
// The input is registered first (one cycle of latency); sums are valid one cycle after the
// last word is presented. Two words at once use the closed form:
//   a' = a + w0 + w1
//   b' = b + 2a + 2w0 + w1
//   c' = c + 2b + 3a + 3w0 + w1
//   d' = d + 2c + 3b + 4a + 4w0 + w1
module fletcher4 (
  input  logic        clk,
  input  logic        clr,
  input  logic [1:0]  cnt,          // words this cycle (w0 first)
  input  logic [31:0] w0,
  input  logic [31:0] w1,
  output logic [63:0] a, b, c, d
);
  logic [1:0]  r_cnt;
  logic [63:0] x0, x1;              // registered words (x1 = 0 when only one word)

  always_ff @(posedge clk) begin
    if (clr) begin
      r_cnt <= '0;
      a <= '0; b <= '0; c <= '0; d <= '0;
    end else begin
      r_cnt <= cnt;
      x0    <= 64'(w0);
      x1    <= (cnt == 2'd2) ? 64'(w1) : 64'd0;
      if (r_cnt != 2'd0) begin
        // one word: one step of the recurrence; two words: the closed form above
        if (r_cnt == 2'd1) begin
          a <= a + x0;
          b <= b + a + x0;
          c <= c + b + a + x0;
          d <= d + c + b + a + x0;
        end else begin
          a <= a + x0 + x1;
          b <= b + (a << 1) + (x0 << 1) + x1;
          c <= c + (b << 1) + (a << 1) + a + (x0 << 1) + x0 + x1;
          d <= d + (c << 1) + (b << 1) + b + (a << 2) + (x0 << 2) + x1;
        end
      end
    end
  end
endmodule
