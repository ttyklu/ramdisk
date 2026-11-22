// fletcher4.sv - ZFS Fletcher-4 over 32-bit little-endian words, 0..2 words per cycle.
//   a += w; b += a; c += b; d += c   (all 64-bit, wrapping)
module fletcher4 (
  input  logic        clk,
  input  logic        clr,
  input  logic [1:0]  cnt,          // words this cycle (w0 first)
  input  logic [31:0] w0,
  input  logic [31:0] w1,
  output logic [63:0] a, b, c, d
);
  always_ff @(posedge clk) begin
    if (clr) begin
      a <= '0; b <= '0; c <= '0; d <= '0;
    end else if (cnt != 2'd0) begin
      logic [63:0] ta, tb, tc, td;
      ta = a + 64'(w0); tb = b + ta; tc = c + tb; td = d + tc;
      if (cnt == 2'd2) begin
        ta = ta + 64'(w1); tb = tb + ta; tc = tc + tb; td = td + tc;
      end
      a <= ta; b <= tb; c <= tc; d <= td;
    end
  end
endmodule
