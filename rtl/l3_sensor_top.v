//==============================================================================
// l3_sensor_top.v — 8-Row L3 Readout Funnel with Dual Packing Banks
//==============================================================================
// Part of: L3 Sensor (128 pixels = 8 rows × 16 pixels)
// Clock: 500 MHz
// - 8× l3_pixel_row (ROW_ID 0..7), each 16-pixel systolic row
// - 2× l3_packing_bank: Left bank (even rows 0,2,4,6), Right bank (odd rows 1,3,5,7)
// - Each bank produces a 128-bit FIFO write stream and a registered stall signal
//   used to generate row_stall for its 4 rows.
//==============================================================================

module l3_sensor_top (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        global_sync,

  // Flattened pixel inputs: row r gets spad_hit[r*16 +: 16], l1_in[r*512 +: 512]
  input  wire [127:0] spad_hit,   // [row*16 + pixel]
  input  wire [4095:0] l1_in,     // [row*512 + pixel*32 +: 32] per pixel

  // Left-bank FIFO interface (even rows 0,2,4,6)
  input  wire        fifo_left_ready,
  output wire        fifo_left_wr_en,
  output wire [127:0] fifo_left_wr_data,

  // Right-bank FIFO interface (odd rows 1,3,5,7)
  input  wire        fifo_right_ready,
  output wire        fifo_right_wr_en,
  output wire [127:0] fifo_right_wr_data
);

  // Row outputs (23-bit) and per-row stall
  wire [22:0] row_bus_0, row_bus_1, row_bus_2, row_bus_3, row_bus_4, row_bus_5, row_bus_6, row_bus_7;
  wire [7:0]  row_valid;
  wire [7:0]  row_stall;
  wire        stall_left;
  wire        stall_right;

  // Time Wall generator (L3 timer). For PPA, we keep it local to L3 and
  // do not propagate its global_sync into L2; only the Time Wall packets
  // are used for L3 stream activity.
  wire        l3timer_global_sync_unused;
  wire [31:0] time_wall_pkt;
  wire        time_wall_vld;

  l3_timer u_l3_timer (
    .clk          (clk),                     // Use 500 MHz clk for PPA; absolute period is not critical here
    .rst_n        (rst_n),
    .global_sync  (l3timer_global_sync_unused),
    .time_wall_pkt(time_wall_pkt),
    .time_wall_vld(time_wall_vld)
  );

  // Optional: per-bank debug outputs (unused here)
  wire [6:0]  seq_index_left;
  wire [6:0]  seq_index_right;
  wire        hb_flush_left;
  wire        hb_flush_right;

  // Row 0
  assign row_stall[0] = stall_left;
  l3_pixel_row #(.ROW_ID(3'd0)) u_row_0 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[0]),
    .spad_hit_0(spad_hit[0]),  .spad_hit_1(spad_hit[1]),  .spad_hit_2(spad_hit[2]),  .spad_hit_3(spad_hit[3]),
    .spad_hit_4(spad_hit[4]),  .spad_hit_5(spad_hit[5]),  .spad_hit_6(spad_hit[6]),  .spad_hit_7(spad_hit[7]),
    .spad_hit_8(spad_hit[8]),  .spad_hit_9(spad_hit[9]),  .spad_hit_10(spad_hit[10]), .spad_hit_11(spad_hit[11]),
    .spad_hit_12(spad_hit[12]), .spad_hit_13(spad_hit[13]), .spad_hit_14(spad_hit[14]), .spad_hit_15(spad_hit[15]),
    .l1_in_0(l1_in[0*32 +: 32]),   .l1_in_1(l1_in[1*32 +: 32]),   .l1_in_2(l1_in[2*32 +: 32]),   .l1_in_3(l1_in[3*32 +: 32]),
    .l1_in_4(l1_in[4*32 +: 32]),   .l1_in_5(l1_in[5*32 +: 32]),   .l1_in_6(l1_in[6*32 +: 32]),   .l1_in_7(l1_in[7*32 +: 32]),
    .l1_in_8(l1_in[8*32 +: 32]),   .l1_in_9(l1_in[9*32 +: 32]),   .l1_in_10(l1_in[10*32 +: 32]), .l1_in_11(l1_in[11*32 +: 32]),
    .l1_in_12(l1_in[12*32 +: 32]), .l1_in_13(l1_in[13*32 +: 32]), .l1_in_14(l1_in[14*32 +: 32]), .l1_in_15(l1_in[15*32 +: 32]),
    .row_bus_out(row_bus_0), .row_bus_out_valid(row_valid[0])
  );

  // Row 1
  assign row_stall[1] = stall_right;
  l3_pixel_row #(.ROW_ID(3'd1)) u_row_1 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[1]),
    .spad_hit_0(spad_hit[16]), .spad_hit_1(spad_hit[17]), .spad_hit_2(spad_hit[18]), .spad_hit_3(spad_hit[19]),
    .spad_hit_4(spad_hit[20]), .spad_hit_5(spad_hit[21]), .spad_hit_6(spad_hit[22]), .spad_hit_7(spad_hit[23]),
    .spad_hit_8(spad_hit[24]), .spad_hit_9(spad_hit[25]), .spad_hit_10(spad_hit[26]), .spad_hit_11(spad_hit[27]),
    .spad_hit_12(spad_hit[28]), .spad_hit_13(spad_hit[29]), .spad_hit_14(spad_hit[30]), .spad_hit_15(spad_hit[31]),
    .l1_in_0(l1_in[512+0*32 +: 32]), .l1_in_1(l1_in[512+1*32 +: 32]), .l1_in_2(l1_in[512+2*32 +: 32]), .l1_in_3(l1_in[512+3*32 +: 32]),
    .l1_in_4(l1_in[512+4*32 +: 32]), .l1_in_5(l1_in[512+5*32 +: 32]), .l1_in_6(l1_in[512+6*32 +: 32]), .l1_in_7(l1_in[512+7*32 +: 32]),
    .l1_in_8(l1_in[512+8*32 +: 32]), .l1_in_9(l1_in[512+9*32 +: 32]), .l1_in_10(l1_in[512+10*32 +: 32]), .l1_in_11(l1_in[512+11*32 +: 32]),
    .l1_in_12(l1_in[512+12*32 +: 32]), .l1_in_13(l1_in[512+13*32 +: 32]), .l1_in_14(l1_in[512+14*32 +: 32]), .l1_in_15(l1_in[512+15*32 +: 32]),
    .row_bus_out(row_bus_1), .row_bus_out_valid(row_valid[1])
  );

  // Row 2
  assign row_stall[2] = stall_left;
  l3_pixel_row #(.ROW_ID(3'd2)) u_row_2 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[2]),
    .spad_hit_0(spad_hit[32]), .spad_hit_1(spad_hit[33]), .spad_hit_2(spad_hit[34]), .spad_hit_3(spad_hit[35]),
    .spad_hit_4(spad_hit[36]), .spad_hit_5(spad_hit[37]), .spad_hit_6(spad_hit[38]), .spad_hit_7(spad_hit[39]),
    .spad_hit_8(spad_hit[40]), .spad_hit_9(spad_hit[41]), .spad_hit_10(spad_hit[42]), .spad_hit_11(spad_hit[43]),
    .spad_hit_12(spad_hit[44]), .spad_hit_13(spad_hit[45]), .spad_hit_14(spad_hit[46]), .spad_hit_15(spad_hit[47]),
    .l1_in_0(l1_in[1024+0*32 +: 32]), .l1_in_1(l1_in[1024+1*32 +: 32]), .l1_in_2(l1_in[1024+2*32 +: 32]), .l1_in_3(l1_in[1024+3*32 +: 32]),
    .l1_in_4(l1_in[1024+4*32 +: 32]), .l1_in_5(l1_in[1024+5*32 +: 32]), .l1_in_6(l1_in[1024+6*32 +: 32]), .l1_in_7(l1_in[1024+7*32 +: 32]),
    .l1_in_8(l1_in[1024+8*32 +: 32]), .l1_in_9(l1_in[1024+9*32 +: 32]), .l1_in_10(l1_in[1024+10*32 +: 32]), .l1_in_11(l1_in[1024+11*32 +: 32]),
    .l1_in_12(l1_in[1024+12*32 +: 32]), .l1_in_13(l1_in[1024+13*32 +: 32]), .l1_in_14(l1_in[1024+14*32 +: 32]), .l1_in_15(l1_in[1024+15*32 +: 32]),
    .row_bus_out(row_bus_2), .row_bus_out_valid(row_valid[2])
  );

  // Row 3
  assign row_stall[3] = stall_right;
  l3_pixel_row #(.ROW_ID(3'd3)) u_row_3 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[3]),
    .spad_hit_0(spad_hit[48]), .spad_hit_1(spad_hit[49]), .spad_hit_2(spad_hit[50]), .spad_hit_3(spad_hit[51]),
    .spad_hit_4(spad_hit[52]), .spad_hit_5(spad_hit[53]), .spad_hit_6(spad_hit[54]), .spad_hit_7(spad_hit[55]),
    .spad_hit_8(spad_hit[56]), .spad_hit_9(spad_hit[57]), .spad_hit_10(spad_hit[58]), .spad_hit_11(spad_hit[59]),
    .spad_hit_12(spad_hit[60]), .spad_hit_13(spad_hit[61]), .spad_hit_14(spad_hit[62]), .spad_hit_15(spad_hit[63]),
    .l1_in_0(l1_in[1536+0*32 +: 32]), .l1_in_1(l1_in[1536+1*32 +: 32]), .l1_in_2(l1_in[1536+2*32 +: 32]), .l1_in_3(l1_in[1536+3*32 +: 32]),
    .l1_in_4(l1_in[1536+4*32 +: 32]), .l1_in_5(l1_in[1536+5*32 +: 32]), .l1_in_6(l1_in[1536+6*32 +: 32]), .l1_in_7(l1_in[1536+7*32 +: 32]),
    .l1_in_8(l1_in[1536+8*32 +: 32]), .l1_in_9(l1_in[1536+9*32 +: 32]), .l1_in_10(l1_in[1536+10*32 +: 32]), .l1_in_11(l1_in[1536+11*32 +: 32]),
    .l1_in_12(l1_in[1536+12*32 +: 32]), .l1_in_13(l1_in[1536+13*32 +: 32]), .l1_in_14(l1_in[1536+14*32 +: 32]), .l1_in_15(l1_in[1536+15*32 +: 32]),
    .row_bus_out(row_bus_3), .row_bus_out_valid(row_valid[3])
  );

  // Row 4
  assign row_stall[4] = stall_left;
  l3_pixel_row #(.ROW_ID(3'd4)) u_row_4 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[4]),
    .spad_hit_0(spad_hit[64]), .spad_hit_1(spad_hit[65]), .spad_hit_2(spad_hit[66]), .spad_hit_3(spad_hit[67]),
    .spad_hit_4(spad_hit[68]), .spad_hit_5(spad_hit[69]), .spad_hit_6(spad_hit[70]), .spad_hit_7(spad_hit[71]),
    .spad_hit_8(spad_hit[72]), .spad_hit_9(spad_hit[73]), .spad_hit_10(spad_hit[74]), .spad_hit_11(spad_hit[75]),
    .spad_hit_12(spad_hit[76]), .spad_hit_13(spad_hit[77]), .spad_hit_14(spad_hit[78]), .spad_hit_15(spad_hit[79]),
    .l1_in_0(l1_in[2048+0*32 +: 32]), .l1_in_1(l1_in[2048+1*32 +: 32]), .l1_in_2(l1_in[2048+2*32 +: 32]), .l1_in_3(l1_in[2048+3*32 +: 32]),
    .l1_in_4(l1_in[2048+4*32 +: 32]), .l1_in_5(l1_in[2048+5*32 +: 32]), .l1_in_6(l1_in[2048+6*32 +: 32]), .l1_in_7(l1_in[2048+7*32 +: 32]),
    .l1_in_8(l1_in[2048+8*32 +: 32]), .l1_in_9(l1_in[2048+9*32 +: 32]), .l1_in_10(l1_in[2048+10*32 +: 32]), .l1_in_11(l1_in[2048+11*32 +: 32]),
    .l1_in_12(l1_in[2048+12*32 +: 32]), .l1_in_13(l1_in[2048+13*32 +: 32]), .l1_in_14(l1_in[2048+14*32 +: 32]), .l1_in_15(l1_in[2048+15*32 +: 32]),
    .row_bus_out(row_bus_4), .row_bus_out_valid(row_valid[4])
  );

  // Row 5
  assign row_stall[5] = stall_right;
  l3_pixel_row #(.ROW_ID(3'd5)) u_row_5 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[5]),
    .spad_hit_0(spad_hit[80]), .spad_hit_1(spad_hit[81]), .spad_hit_2(spad_hit[82]), .spad_hit_3(spad_hit[83]),
    .spad_hit_4(spad_hit[84]), .spad_hit_5(spad_hit[85]), .spad_hit_6(spad_hit[86]), .spad_hit_7(spad_hit[87]),
    .spad_hit_8(spad_hit[88]), .spad_hit_9(spad_hit[89]), .spad_hit_10(spad_hit[90]), .spad_hit_11(spad_hit[91]),
    .spad_hit_12(spad_hit[92]), .spad_hit_13(spad_hit[93]), .spad_hit_14(spad_hit[94]), .spad_hit_15(spad_hit[95]),
    .l1_in_0(l1_in[2560+0*32 +: 32]), .l1_in_1(l1_in[2560+1*32 +: 32]), .l1_in_2(l1_in[2560+2*32 +: 32]), .l1_in_3(l1_in[2560+3*32 +: 32]),
    .l1_in_4(l1_in[2560+4*32 +: 32]), .l1_in_5(l1_in[2560+5*32 +: 32]), .l1_in_6(l1_in[2560+6*32 +: 32]), .l1_in_7(l1_in[2560+7*32 +: 32]),
    .l1_in_8(l1_in[2560+8*32 +: 32]), .l1_in_9(l1_in[2560+9*32 +: 32]), .l1_in_10(l1_in[2560+10*32 +: 32]), .l1_in_11(l1_in[2560+11*32 +: 32]),
    .l1_in_12(l1_in[2560+12*32 +: 32]), .l1_in_13(l1_in[2560+13*32 +: 32]), .l1_in_14(l1_in[2560+14*32 +: 32]), .l1_in_15(l1_in[2560+15*32 +: 32]),
    .row_bus_out(row_bus_5), .row_bus_out_valid(row_valid[5])
  );

  // Row 6
  assign row_stall[6] = stall_left;
  l3_pixel_row #(.ROW_ID(3'd6)) u_row_6 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[6]),
    .spad_hit_0(spad_hit[96]), .spad_hit_1(spad_hit[97]), .spad_hit_2(spad_hit[98]), .spad_hit_3(spad_hit[99]),
    .spad_hit_4(spad_hit[100]), .spad_hit_5(spad_hit[101]), .spad_hit_6(spad_hit[102]), .spad_hit_7(spad_hit[103]),
    .spad_hit_8(spad_hit[104]), .spad_hit_9(spad_hit[105]), .spad_hit_10(spad_hit[106]), .spad_hit_11(spad_hit[107]),
    .spad_hit_12(spad_hit[108]), .spad_hit_13(spad_hit[109]), .spad_hit_14(spad_hit[110]), .spad_hit_15(spad_hit[111]),
    .l1_in_0(l1_in[3072+0*32 +: 32]), .l1_in_1(l1_in[3072+1*32 +: 32]), .l1_in_2(l1_in[3072+2*32 +: 32]), .l1_in_3(l1_in[3072+3*32 +: 32]),
    .l1_in_4(l1_in[3072+4*32 +: 32]), .l1_in_5(l1_in[3072+5*32 +: 32]), .l1_in_6(l1_in[3072+6*32 +: 32]), .l1_in_7(l1_in[3072+7*32 +: 32]),
    .l1_in_8(l1_in[3072+8*32 +: 32]), .l1_in_9(l1_in[3072+9*32 +: 32]), .l1_in_10(l1_in[3072+10*32 +: 32]), .l1_in_11(l1_in[3072+11*32 +: 32]),
    .l1_in_12(l1_in[3072+12*32 +: 32]), .l1_in_13(l1_in[3072+13*32 +: 32]), .l1_in_14(l1_in[3072+14*32 +: 32]), .l1_in_15(l1_in[3072+15*32 +: 32]),
    .row_bus_out(row_bus_6), .row_bus_out_valid(row_valid[6])
  );

  // Row 7
  assign row_stall[7] = stall_right;
  l3_pixel_row #(.ROW_ID(3'd7)) u_row_7 (
    .clk(clk), .rst_n(rst_n), .global_sync(global_sync), .row_stall(row_stall[7]),
    .spad_hit_0(spad_hit[112]), .spad_hit_1(spad_hit[113]), .spad_hit_2(spad_hit[114]), .spad_hit_3(spad_hit[115]),
    .spad_hit_4(spad_hit[116]), .spad_hit_5(spad_hit[117]), .spad_hit_6(spad_hit[118]), .spad_hit_7(spad_hit[119]),
    .spad_hit_8(spad_hit[120]), .spad_hit_9(spad_hit[121]), .spad_hit_10(spad_hit[122]), .spad_hit_11(spad_hit[123]),
    .spad_hit_12(spad_hit[124]), .spad_hit_13(spad_hit[125]), .spad_hit_14(spad_hit[126]), .spad_hit_15(spad_hit[127]),
    .l1_in_0(l1_in[3584+0*32 +: 32]), .l1_in_1(l1_in[3584+1*32 +: 32]), .l1_in_2(l1_in[3584+2*32 +: 32]), .l1_in_3(l1_in[3584+3*32 +: 32]),
    .l1_in_4(l1_in[3584+4*32 +: 32]), .l1_in_5(l1_in[3584+5*32 +: 32]), .l1_in_6(l1_in[3584+6*32 +: 32]), .l1_in_7(l1_in[3584+7*32 +: 32]),
    .l1_in_8(l1_in[3584+8*32 +: 32]), .l1_in_9(l1_in[3584+9*32 +: 32]), .l1_in_10(l1_in[3584+10*32 +: 32]), .l1_in_11(l1_in[3584+11*32 +: 32]),
    .l1_in_12(l1_in[3584+12*32 +: 32]), .l1_in_13(l1_in[3584+13*32 +: 32]), .l1_in_14(l1_in[3584+14*32 +: 32]), .l1_in_15(l1_in[3584+15*32 +: 32]),
    .row_bus_out(row_bus_7), .row_bus_out_valid(row_valid[7])
  );

  //----------------------------------------------------------------------------
  // Dual packing banks:
  // - Left bank: even rows 0,2,4,6
  // - Right bank: odd rows 1,3,5,7
  //----------------------------------------------------------------------------
  wire        fifo_left_wr_en_raw;
  wire [127:0] fifo_left_wr_data_raw;
  wire        fifo_right_wr_en_raw;
  wire [127:0] fifo_right_wr_data_raw;

  l3_packing_bank u_packing_left (
    .clk           (clk),
    .rst_n         (rst_n),
    .valid_in      ({row_valid[6], row_valid[4], row_valid[2], row_valid[0]}),
    .data_in_0     (row_bus_0),
    .data_in_1     (row_bus_2),
    .data_in_2     (row_bus_4),
    .data_in_3     (row_bus_6),
    .fifo_ready    (fifo_left_ready),
    .fifo_wr_en    (fifo_left_wr_en_raw),
    .fifo_wr_data  (fifo_left_wr_data_raw),
    .stall         (stall_left),
    .seq_index     (seq_index_left),
    .heartbeat_flush(hb_flush_left)
  );

  l3_packing_bank u_packing_right (
    .clk           (clk),
    .rst_n         (rst_n),
    .valid_in      ({row_valid[7], row_valid[5], row_valid[3], row_valid[1]}),
    .data_in_0     (row_bus_1),
    .data_in_1     (row_bus_3),
    .data_in_2     (row_bus_5),
    .data_in_3     (row_bus_7),
    .fifo_ready    (fifo_right_ready),
    .fifo_wr_en    (fifo_right_wr_en_raw),
    .fifo_wr_data  (fifo_right_wr_data_raw),
    .stall         (stall_right),
    .seq_index     (seq_index_right),
    .heartbeat_flush(hb_flush_right)
  );

  //----------------------------------------------------------------------------
  // Time Wall injection at 128-bit bus level (per-bank)
  //----------------------------------------------------------------------------
  // For PPA evaluation, we give Time Wall packets highest priority at the
  // L3 128-bit bus output: when time_wall_vld is asserted, we emit a
  // dedicated 128-bit word carrying the Time Wall packet, and suppress
  // the bank's normal write for that cycle. Existing bank / row logic
  // is left untouched.
  //
  // Time Wall format: [31:30]=2'b11, [29:0]=absolute timestamp.
  // Here we place the Time Wall in the least-significant 32-bit lane.
  // Other lanes are zeroed for simplicity.

  assign fifo_left_wr_en  = time_wall_vld ? 1'b1              : fifo_left_wr_en_raw;
  assign fifo_left_wr_data= time_wall_vld ? {96'b0, time_wall_pkt} : fifo_left_wr_data_raw;

  assign fifo_right_wr_en  = time_wall_vld ? 1'b1               : fifo_right_wr_en_raw;
  assign fifo_right_wr_data= time_wall_vld ? {96'b0, time_wall_pkt}  : fifo_right_wr_data_raw;

endmodule
