// ddr_rd.sv - reads a DDR range over AXI4 (64-bit, 16-beat = 128 B bursts) and pushes the
// data into the TX DW FIFO (2 DWs per beat). An AR is only issued when the FIFO has room
// for that burst plus everything already in flight, so RREADY can stay high.
module ddr_rd #(
  parameter int FAW = 10
)(
  input  logic         clk,
  input  logic         rst,

  input  logic         job_valid,
  output logic         job_ready,
  input  logic [31:0]  job_addr,      // 128-byte aligned
  input  logic [23:0]  job_len,       // bytes, multiple of 128

  input  logic [FAW:0] fifo_free,

  output logic [31:0]  araddr,
  output logic [7:0]   arlen,
  output logic [2:0]   arsize,
  output logic [1:0]   arburst,
  output logic         arvalid,
  input  logic         arready,
  input  logic [63:0]  rdata,
  input  logic         rlast,
  input  logic         rvalid,
  output logic         rready,

  output logic [1:0]   push_cnt,
  output logic [31:0]  push_dw0,
  output logic [31:0]  push_dw1,

  output logic         busy
);

  logic [16:0] bursts_left;
  logic [31:0] addr;
  logic [FAW:0] inflight;      // beats requested but not yet received

  wire [FAW+1:0] need = ({1'b0, inflight} + (FAW+2)'(16)) << 1;   // DWs to reserve

  assign arvalid = (bursts_left != 17'd0) && ({1'b0, fifo_free} >= need);
  assign araddr  = addr;
  assign arlen   = 8'd15;
  assign arsize  = 3'd3;
  assign arburst = 2'b01;
  assign rready  = 1'b1;

  assign push_cnt = rvalid ? 2'd2 : 2'd0;
  assign push_dw0 = rdata[31:0];
  assign push_dw1 = rdata[63:32];

  assign busy      = (bursts_left != 17'd0) || (inflight != '0);
  assign job_ready = !busy;

  wire ar_fire = arvalid && arready;

  always_ff @(posedge clk) begin
    if (rst) begin
      bursts_left <= '0;
      inflight    <= '0;
      addr        <= '0;
    end else begin
      if (job_valid && job_ready) begin
        addr        <= job_addr;
        bursts_left <= job_len[23:7];
      end else if (ar_fire) begin
        addr        <= addr + 32'd128;
        bursts_left <= bursts_left - 17'd1;
      end
      inflight <= inflight + (ar_fire ? (FAW+1)'(16) : '0) - (rvalid ? (FAW+1)'(1) : '0);
    end
  end

endmodule
