// ram_1r1w.sv - simple dual-port RAM: one write port, one synchronous read port
// (registered output, read-first on an address collision). Infers block RAM.
module ram_1r1w #(
  parameter int DW = 8,
  parameter int AW = 10
)(
  input  logic          clk,
  input  logic          we,
  input  logic [AW-1:0] waddr,
  input  logic [DW-1:0] wdata,
  input  logic [AW-1:0] raddr,
  output logic [DW-1:0] rdata
);
  logic [DW-1:0] mem [1 << AW];
  always_ff @(posedge clk) begin
    if (we) mem[waddr] <= wdata;
    rdata <= mem[raddr];
  end
endmodule
