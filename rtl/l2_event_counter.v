//==============================================================================
// l2_event_counter.v — 16-L1 Event Accumulator with Zone Mask (10 ns window)
//==============================================================================
// Part of: l2PixelNode (16×8 Photodetector Readout ASIC)
// Clock: 500 MHz. 10 ns window = 5 cycles (modulo-5 counter).
// 16 L1 inputs × 2-bit (0..3 thresholds) → weighted sum; 4 zones (2×2).
// Adder tree for single-cycle 16-input sum; auto-reset after report.
//==============================================================================

module l2_event_counter #(
  parameter VAL_THR1 = 8'd1,   // weight for L1 value 1
  parameter VAL_THR2 = 8'd2,   // weight for L1 value 2
  parameter VAL_THR3 = 8'd3    // weight for L1 value 3
) (
  input  wire        clk,
  input  wire        rst_n,

  // 16 L1 inputs: l1_in[2*k+1:2*k] = pixel k (0, 1, 2, or 3)
  input  wire [31:0] l1_in,

  // Reported every 10 ns (end of 5th cycle)
  output reg  [ 7:0] energy_out,
  output reg  [ 3:0] zone_mask_out,
  output reg  [ 2:0] status_out,   // [0] = overflow_flag; [2:1] reserved
  output reg         report_valid  // high for one cycle when outputs are valid
);

  //----------------------------------------------------------------------------
  // Per-pixel weight lookup (0→0, 1→VAL_THR1, 2→VAL_THR2, 3→VAL_THR3)
  // Each weighted value fits in 8 bits; 16 pixels → tree sum needs 12 bits
  //----------------------------------------------------------------------------
  wire [7:0] w [0:15];
  genvar gi;
  generate
    for (gi = 0; gi < 16; gi = gi + 1) begin : weight_mux
      assign w[gi] = (l1_in[gi*2 +: 2] == 2'd0) ? 8'd0 :
                     (l1_in[gi*2 +: 2] == 2'd1) ? VAL_THR1 :
                     (l1_in[gi*2 +: 2] == 2'd2) ? VAL_THR2 : VAL_THR3;
    end
  endgenerate

  //----------------------------------------------------------------------------
  // Adder tree (4 levels) — 16 inputs → one 12-bit cycle sum
  // Level 1: 8 adders (pairs)
  //----------------------------------------------------------------------------
  wire [8:0] s1 [0:7];
  generate
    for (gi = 0; gi < 8; gi = gi + 1) begin : level1
      assign s1[gi] = w[gi*2] + w[gi*2 + 1];
    end
  endgenerate

  // Level 2: 4 adders
  wire [9:0] s2 [0:3];
  generate
    for (gi = 0; gi < 4; gi = gi + 1) begin : level2
      assign s2[gi] = s1[gi*2] + s1[gi*2 + 1];
    end
  endgenerate

  // Level 3: 2 adders
  wire [10:0] s3 [0:1];
  assign s3[0] = s2[0] + s2[1];
  assign s3[1] = s2[2] + s2[3];

  // Level 4: final sum
  wire [11:0] cycle_sum;
  assign cycle_sum = s3[0] + s3[1];

  //----------------------------------------------------------------------------
  // Zone activity this cycle (2×2 grid; 4×4 pixels: Z0=0,1,4,5; Z1=2,3,6,7; Z2=8,9,12,13; Z3=10,11,14,15)
  // Pixel k = l1_in[2*k+1:2*k]
  //----------------------------------------------------------------------------
  wire [3:0] zone_has_activity;
  assign zone_has_activity[0] = (l1_in[ 1: 0] != 2'b0) | (l1_in[ 3: 2] != 2'b0) |
                                 (l1_in[ 9: 8] != 2'b0) | (l1_in[11:10] != 2'b0);
  assign zone_has_activity[1] = (l1_in[ 5: 4] != 2'b0) | (l1_in[ 7: 6] != 2'b0) |
                                 (l1_in[13:12] != 2'b0) | (l1_in[15:14] != 2'b0);
  assign zone_has_activity[2] = (l1_in[17:16] != 2'b0) | (l1_in[19:18] != 2'b0) |
                                 (l1_in[25:24] != 2'b0) | (l1_in[27:26] != 2'b0);
  assign zone_has_activity[3] = (l1_in[21:20] != 2'b0) | (l1_in[23:22] != 2'b0) |
                                 (l1_in[29:28] != 2'b0) | (l1_in[31:30] != 2'b0);

  //----------------------------------------------------------------------------
  // Modulo-5 counter and accumulators
  //----------------------------------------------------------------------------
  reg [2:0] cycle_cnt;      // 0..4
  reg [15:0] total_energy;  // running sum over 5 cycles (saturate detection)
  reg [3:0] zone_mask;      // OR of zone_has_activity over window

  wire end_of_window = (cycle_cnt == 3'd4);  // 5th cycle

  // Include 5th cycle in report: final = accumulated (cycles 0..3) + current (cycle 4)
  wire [15:0] final_energy = total_energy + cycle_sum;
  wire [3:0]  final_zone_mask = zone_mask | zone_has_activity;
  wire overflow_flag = (final_energy > 16'd255);
  wire [7:0] energy_sat = overflow_flag ? 8'hFF : final_energy[7:0];

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cycle_cnt     <= 3'd0;
      total_energy  <= 16'd0;
      zone_mask     <= 4'd0;
      energy_out    <= 8'd0;
      zone_mask_out <= 4'd0;
      status_out    <= 3'd0;
      report_valid  <= 1'b0;
    end else begin
      report_valid <= 1'b0;

      if (end_of_window) begin
        // Report (includes 5th cycle) and auto-reset for next window (no dead time)
        energy_out    <= energy_sat;
        zone_mask_out <= final_zone_mask;
        status_out    <= {2'b0, overflow_flag};
        report_valid  <= 1'b1;

        total_energy <= 16'd0;
        zone_mask    <= 4'd0;
        cycle_cnt    <= 3'd0;
      end else begin
        total_energy <= total_energy + cycle_sum;
        zone_mask    <= zone_mask | zone_has_activity;
        cycle_cnt    <= cycle_cnt + 1'b1;
      end
    end
  end

endmodule
