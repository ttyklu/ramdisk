// dw_ram2.sv - DW-addressed RAM that writes and reads two consecutive DWs per cycle
// at any DW index. Built from two banks (even / odd DW index), each with a single
// write port, so Vivado can map it to LUTRAM. Reads are asynchronous.
module dw_ram2 #(
  parameter int AW = 10            // log2(depth in DWs)
)(
  input  logic          clk,
  input  logic [1:0]    wcnt,      // 0, 1 or 2 DWs to write
  input  logic [AW-1:0] widx,      // DW index of first DW
  input  logic [31:0]   wd0,       // DW written at widx
  input  logic [31:0]   wd1,       // DW written at widx+1 (when wcnt == 2)
  input  logic [AW-1:0] ridx,
  output logic [31:0]   rd0,       // DW at ridx
  output logic [31:0]   rd1        // DW at ridx+1
);
  localparam int HD = 1 << (AW - 1);

  logic [31:0] bank_e [HD];
  logic [31:0] bank_o [HD];

  wire [AW-1:0] wi0 = widx;
  wire [AW-1:0] wi1 = widx + 1'b1;

  // even bank: gets dw0 if widx is even, else dw1 (at widx+1)
  wire          we_e   = wi0[0] ? (wcnt == 2'd2) : (wcnt != 2'd0);
  wire [AW-2:0] wa_e   = wi0[0] ? wi1[AW-1:1] : wi0[AW-1:1];
  wire [31:0]   wd_e   = wi0[0] ? wd1 : wd0;
  // odd bank: gets dw0 if widx is odd, else dw1
  wire          we_o   = wi0[0] ? (wcnt != 2'd0) : (wcnt == 2'd2);
  wire [AW-2:0] wa_o   = wi0[0] ? wi0[AW-1:1] : wi1[AW-1:1];
  wire [31:0]   wd_o   = wi0[0] ? wd0 : wd1;

  always_ff @(posedge clk) begin
    if (we_e) bank_e[wa_e] <= wd_e;
  end

  always_ff @(posedge clk) begin
    if (we_o) bank_o[wa_o] <= wd_o;
  end

  wire [AW-1:0] ri0 = ridx;
  wire [AW-1:0] ri1 = ridx + 1'b1;

  assign rd0 = ri0[0] ? bank_o[ri0[AW-1:1]] : bank_e[ri0[AW-1:1]];
  assign rd1 = ri1[0] ? bank_o[ri1[AW-1:1]] : bank_e[ri1[AW-1:1]];

endmodule
