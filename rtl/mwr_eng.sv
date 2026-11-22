// mwr_eng.sv - turns host-write jobs (address, length in DWs) into MWr TLPs.
// Each TLP is at most 32 DWs and never crosses a 128-byte boundary, which keeps it within
// any Max_Payload_Size >= 128 B and never crosses a 4 KB page.
// A TLP is only requested when the TX FIFO already holds all of its payload, and the next
// TLP waits until the previous one has been fully sent.
module mwr_eng #(
  parameter int FAW = 10
)(
  input  logic         clk,
  input  logic         rst,

  input  logic         job_valid,
  output logic         job_ready,
  input  logic [63:0]  job_addr,
  input  logic [11:0]  job_len,       // DWs

  input  logic [FAW:0] fifo_count,

  output logic         w_valid,
  input  logic         w_ready,
  output logic [63:0]  w_addr,
  output logic [10:0]  w_len,
  input  logic         w_done,

  output logic         idle
);

  typedef enum logic [1:0] {M_IDLE, M_ISSUE, M_WAIT} st_t;
  st_t st;

  logic [63:0] addr;
  logic [11:0] rem;
  logic [5:0]  tlen;

  wire [5:0] to_bnd = 6'd32 - {1'b0, addr[6:2]};
  wire [5:0] nlen   = (rem < {6'd0, to_bnd}) ? rem[5:0] : to_bnd;

  assign job_ready = (st == M_IDLE);
  assign idle      = (st == M_IDLE);
  assign w_valid   = (st == M_ISSUE) && (fifo_count >= {{(FAW-5){1'b0}}, nlen});
  assign w_addr    = addr;
  assign w_len     = {5'd0, nlen};

  always_ff @(posedge clk) begin
    if (rst) begin
      st <= M_IDLE;
    end else begin
      case (st)
        M_IDLE: begin
          if (job_valid && job_len != 12'd0) begin
            addr <= {job_addr[63:2], 2'b00};
            rem  <= job_len;
            st   <= M_ISSUE;
          end
        end
        M_ISSUE: begin
          if (w_valid && w_ready) begin
            tlen <= nlen;
            st   <= M_WAIT;
          end
        end
        M_WAIT: begin
          if (w_done) begin
            addr <= addr + {56'd0, tlen, 2'b00};
            rem  <= rem - {6'd0, tlen};
            st   <= (rem == {6'd0, tlen}) ? M_IDLE : M_ISSUE;
          end
        end
        default: st <= M_IDLE;
      endcase
    end
  end

endmodule
