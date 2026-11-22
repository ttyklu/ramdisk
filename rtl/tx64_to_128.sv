// tx64_to_128.sv - packs the 64-bit TX beats of tlp_tx into the 7-series PCIe 128-bit TX
// AXI-Stream (s_axis_tx_*, 125 MHz). Every TLP starts at DW0 of a 128-bit beat; tkeep is
// 0x000F / 0x00FF / 0x0FFF / 0xFFFF on the last beat.
module tx64_to_128 (
  input  logic         clk,
  input  logic         rst,

  input  logic [63:0]  s_tdata,
  input  logic [7:0]   s_tkeep,
  input  logic         s_tlast,
  input  logic         s_tvalid,
  output logic         s_tready,
  input  logic [3:0]   s_tuser,

  output logic [127:0] m_tdata,
  output logic [15:0]  m_tkeep,
  output logic         m_tlast,
  output logic         m_tvalid,
  input  logic         m_tready,
  output logic [3:0]   m_tuser
);
  logic        half;          // lower 64 bits of the next output beat are held
  logic [63:0] lo;
  logic [7:0]  lo_keep;
  logic [3:0]  lo_user;

  // output register is loaded when it is empty or being taken
  wire out_free = !m_tvalid || m_tready;
  // a beat that completes an output beat (second half, or a TLP end) needs the output free
  assign s_tready = out_free || (!half && !s_tlast);
  wire in_fire = s_tvalid && s_tready;

  always_ff @(posedge clk) begin
    if (rst) begin
      half     <= 1'b0;
      m_tvalid <= 1'b0;
    end else begin
      if (m_tvalid && m_tready) m_tvalid <= 1'b0;
      if (in_fire) begin
        if (!half) begin
          if (s_tlast) begin                    // a 1-beat TLP end: send the lower half alone
            m_tdata  <= {64'd0, s_tdata};
            m_tkeep  <= {8'd0, s_tkeep};
            m_tlast  <= 1'b1;
            m_tuser  <= s_tuser;
            m_tvalid <= 1'b1;
          end else begin
            lo      <= s_tdata;
            lo_keep <= s_tkeep;
            lo_user <= s_tuser;
            half    <= 1'b1;
          end
        end else begin
          m_tdata  <= {s_tdata, lo};
          m_tkeep  <= {s_tkeep, lo_keep};
          m_tlast  <= s_tlast;
          m_tuser  <= lo_user;
          m_tvalid <= 1'b1;
          half     <= 1'b0;
        end
      end
    end
  end
endmodule
