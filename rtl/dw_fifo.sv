// dw_fifo.sv - FIFO of 32-bit DWs. Push 0..2 and pop 0..2 DWs per cycle.
// The caller must not push more than 'free' or pop more than 'count'.
module dw_fifo #(
  parameter int AW = 10
)(
  input  logic        clk,
  input  logic        rst,
  input  logic        flush,
  input  logic [1:0]  push_cnt,
  input  logic [31:0] push_dw0,
  input  logic [31:0] push_dw1,
  input  logic [1:0]  pop_cnt,
  output logic [31:0] head0,
  output logic [31:0] head1,
  output logic [AW:0] count,
  output logic [AW:0] free
);
  logic [AW:0] wp, rp;

  dw_ram2 #(.AW(AW)) u_ram (
    .clk  (clk),
    .wcnt (push_cnt),
    .widx (wp[AW-1:0]),
    .wd0  (push_dw0),
    .wd1  (push_dw1),
    .ridx (rp[AW-1:0]),
    .rd0  (head0),
    .rd1  (head1)
  );

  // count / free are registered but exact: computed from the next pointer values
  wire [AW:0] wp_n = wp + push_cnt;
  wire [AW:0] rp_n = rp + pop_cnt;
  always_ff @(posedge clk) begin
    if (rst || flush) begin
      wp    <= '0;
      rp    <= '0;
      count <= '0;
      free  <= (AW+1)'(1 << AW);
    end else begin
      wp    <= wp_n;
      rp    <= rp_n;
      count <= wp_n - rp_n;
      free  <= (AW+1)'(1 << AW) - (wp_n - rp_n);
    end
  end

endmodule
