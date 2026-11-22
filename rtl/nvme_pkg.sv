// nvme_pkg.sv - shared constants and helpers for the ZC706 NVMe RAM disk
package nvme_pkg;

  // TLP type field values (bits [28:24] of header DW0)
  localparam logic [4:0] TLP_T_MEM = 5'b00000;   // MRd / MWr
  localparam logic [4:0] TLP_T_CPL = 5'b01010;   // Cpl / CplD

  // Fmt values (bits [31:29] of header DW0)
  localparam logic [2:0] FMT_3DW_ND = 3'b000;
  localparam logic [2:0] FMT_4DW_ND = 3'b001;
  localparam logic [2:0] FMT_3DW_D  = 3'b010;
  localparam logic [2:0] FMT_4DW_D  = 3'b011;

  // Build header DW0
  function automatic logic [31:0] mk_h0(input logic [2:0] fmt, input logic [4:0] typ,
                                        input logic [2:0] tc, input logic [1:0] attr,
                                        input logic [9:0] len);
    // [31:29] fmt [28:24] type [23] T9 [22:20] TC [19] T8 [18] Attr2 [17] LN
    // [16] TH [15] TD [14] EP [13:12] Attr [11:10] AT [9:0] Length
    return {fmt, typ, 1'b0, tc, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, attr, 2'b00, len};
  endfunction

  // Payload DW byte swap (used only if the PCIe core presents payload big-endian)
  function automatic logic [31:0] bswap32(input logic [31:0] d);
    return {d[7:0], d[15:8], d[23:16], d[31:24]};
  endfunction

  // Merge a 32-bit write with byte enables
  function automatic logic [31:0] be_merge(input logic [31:0] old_v, input logic [31:0] new_v,
                                           input logic [3:0] be);
    logic [31:0] r;
    for (int i = 0; i < 4; i++) r[i*8 +: 8] = be[i] ? new_v[i*8 +: 8] : old_v[i*8 +: 8];
    return r;
  endfunction

  // NVMe status codes {SCT[2:0], SC[7:0]}
  localparam logic [10:0] ST_SUCCESS          = {3'd0, 8'h00};
  localparam logic [10:0] ST_INVALID_OPCODE   = {3'd0, 8'h01};
  localparam logic [10:0] ST_INVALID_FIELD    = {3'd0, 8'h02};
  localparam logic [10:0] ST_DATA_XFER_ERR    = {3'd0, 8'h04};
  localparam logic [10:0] ST_INVALID_NS       = {3'd0, 8'h0B};
  localparam logic [10:0] ST_PRP_OFFSET       = {3'd0, 8'h13};
  localparam logic [10:0] ST_LBA_RANGE        = {3'd0, 8'h80};
  localparam logic [10:0] ST_CQ_INVALID       = {3'd1, 8'h00};
  localparam logic [10:0] ST_INVALID_QID      = {3'd1, 8'h01};
  localparam logic [10:0] ST_INVALID_QSIZE    = {3'd1, 8'h02};
  localparam logic [10:0] ST_AER_LIMIT        = {3'd1, 8'h05};
  localparam logic [10:0] ST_INVALID_IV       = {3'd1, 8'h08};
  localparam logic [10:0] ST_INVALID_QDEL     = {3'd1, 8'h0C};

endpackage
