// ddr_wr.sv - writes a DDR range over AXI4 (64-bit, 16-beat = 128 B bursts) from the
// dma_rd output stream (2 DWs per beat). W data for a burst is only sent after its AW.
// Done (busy low) once every burst has its B response.
module ddr_wr (
  input  logic         clk,
  input  logic         rst,

  input  logic         job_valid,
  output logic         job_ready,
  input  logic [31:0]  job_addr,      // 128-byte aligned
  input  logic [17:0]  job_len,       // bytes, multiple of 128

  input  logic [10:0]  src_avail,     // DWs available from dma_rd
  input  logic [31:0]  src_dw0,
  input  logic [31:0]  src_dw1,
  output logic [1:0]   src_pop,

  output logic [31:0]  awaddr,
  output logic [7:0]   awlen,
  output logic [2:0]   awsize,
  output logic [1:0]   awburst,
  output logic         awvalid,
  input  logic         awready,
  output logic [63:0]  wdata,
  output logic [7:0]   wstrb,
  output logic         wlast,
  output logic         wvalid,
  input  logic         wready,
  input  logic [1:0]   bresp,
  input  logic         bvalid,
  output logic         bready,

  output logic         busy
);

  logic        active;
  logic [10:0] nb;          // total bursts
  logic [10:0] aw_cnt;      // bursts whose AW was sent
  logic [10:0] w_cnt;       // bursts whose W data was fully sent
  logic [10:0] b_cnt;       // B responses received
  logic [3:0]  beat;
  logic [31:0] addr;

  assign awvalid = active && (aw_cnt != nb) && ((aw_cnt - b_cnt) < 11'd8);
  assign awaddr  = addr;
  assign awlen   = 8'd15;
  assign awsize  = 3'd3;
  assign awburst = 2'b01;

  assign wvalid  = active && (w_cnt != aw_cnt) && (src_avail >= 11'd2);
  assign wdata   = {src_dw1, src_dw0};
  assign wstrb   = 8'hFF;
  assign wlast   = (beat == 4'd15);
  assign bready  = 1'b1;

  wire w_fire = wvalid && wready;
  assign src_pop = w_fire ? 2'd2 : 2'd0;

  assign busy      = active;
  assign job_ready = !active;

  always_ff @(posedge clk) begin
    if (rst) begin
      active <= 1'b0;
      nb     <= '0;
      aw_cnt <= '0;
      w_cnt  <= '0;
      b_cnt  <= '0;
      beat   <= '0;
      addr   <= '0;
    end else begin
      if (job_valid && job_ready && job_len[17:7] != 11'd0) begin
        active <= 1'b1;
        nb     <= job_len[17:7];
        aw_cnt <= '0;
        w_cnt  <= '0;
        b_cnt  <= '0;
        beat   <= '0;
        addr   <= job_addr;
      end else if (active) begin
        if (awvalid && awready) begin
          aw_cnt <= aw_cnt + 11'd1;
          addr   <= addr + 32'd128;
        end
        if (w_fire) begin
          beat <= beat + 4'd1;
          if (wlast) w_cnt <= w_cnt + 11'd1;
        end
        if (bvalid) begin
          b_cnt <= b_cnt + 11'd1;
          if (b_cnt + 11'd1 == nb) active <= 1'b0;
        end
      end
    end
  end

endmodule
