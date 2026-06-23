//==============================================================================
// l3_logic.v — 4-Way Compaction + 128-bit Shadow Accumulator for L3 FIFO
//==============================================================================
// Part of: L3 System Top (16×8 Photodetector Readout ASIC)
// Clock: 500 MHz. Compacts 4×23-bit column slots → 32-bit L3 packets → 128-bit FIFO writes.
// High-priority Time Wall packet insertion. Supports up to 5 packets (1 Time Wall + 4 slots).
// Architecture: Spliced Bus (Sliding Window) for robust remainder handling.
//==============================================================================

module l3_logic (
  input  wire        clk,
  input  wire        rst_n,

  // Column bus inputs: 4 slots × 23-bit packets
  input  wire [22:0] slot0_data,
  input  wire        slot0_valid,
  input  wire [22:0] slot1_data,
  input  wire        slot1_valid,
  input  wire [22:0] slot2_data,
  input  wire        slot2_valid,
  input  wire [22:0] slot3_data,
  input  wire        slot3_valid,

  // Time Wall packet from L3Timer (high priority)
  input  wire [31:0] time_wall_pkt,
  input  wire        time_wall_vld,

  // L3 FIFO write interface (128-bit = 4×32-bit packets)
  output wire        fifo_wr_en,
  output wire [127:0] fifo_wr_data,

  // Status
  output wire [1:0]  fill_level_out
);

  //----------------------------------------------------------------------------
  // Step 1: Format conversion - 23-bit column packets → 32-bit L3 packets
  // Format: [31:30]=2'b00 (Event), [29:23]=Rsrv, [22:20]=RowID, [19:16]=L2ID, [15:0]=L2Payload
  //----------------------------------------------------------------------------
  wire [31:0] pkt0_32 = slot0_valid ? {2'b00, 7'b0, slot0_data} : 32'b0;
  wire [31:0] pkt1_32 = slot1_valid ? {2'b00, 7'b0, slot1_data} : 32'b0;
  wire [31:0] pkt2_32 = slot2_valid ? {2'b00, 7'b0, slot2_data} : 32'b0;
  wire [31:0] pkt3_32 = slot3_valid ? {2'b00, 7'b0, slot3_data} : 32'b0;

  // Count valid slots
  wire [2:0] num_slots = slot0_valid + slot1_valid + slot2_valid + slot3_valid;

  //----------------------------------------------------------------------------
  // Step 2: Barrel Shifter - Compact valid packets to left (combinational)
  // Priority: Slot 0 → Slot 1 → Slot 2 → Slot 3
  //----------------------------------------------------------------------------
  wire [31:0] compacted [0:3];
  
  // Priority encoder style: pack valid packets left
  assign compacted[0] = slot0_valid ? pkt0_32 :
                         slot1_valid ? pkt1_32 :
                         slot2_valid ? pkt2_32 :
                         slot3_valid ? pkt3_32 : 32'b0;
  
  assign compacted[1] = (slot0_valid && slot1_valid) ? pkt1_32 :
                        (slot0_valid && !slot1_valid && slot2_valid) ? pkt2_32 :
                        (slot0_valid && !slot1_valid && !slot2_valid && slot3_valid) ? pkt3_32 :
                        (!slot0_valid && slot1_valid && slot2_valid) ? pkt2_32 :
                        (!slot0_valid && slot1_valid && !slot2_valid && slot3_valid) ? pkt3_32 :
                        (!slot0_valid && !slot1_valid && slot2_valid && slot3_valid) ? pkt3_32 : 32'b0;
  
  assign compacted[2] = (slot0_valid && slot1_valid && slot2_valid) ? pkt2_32 :
                        (slot0_valid && slot1_valid && !slot2_valid && slot3_valid) ? pkt3_32 :
                        (slot0_valid && !slot1_valid && slot2_valid && slot3_valid) ? pkt3_32 :
                        (!slot0_valid && slot1_valid && slot2_valid && slot3_valid) ? pkt3_32 : 32'b0;
  
  assign compacted[3] = (slot0_valid && slot1_valid && slot2_valid && slot3_valid) ? pkt3_32 : 32'b0;

  //----------------------------------------------------------------------------
  // Step 3: High-Priority Time Wall insertion (5-packet support)
  // If time_wall_vld, Time Wall packet becomes first packet, shift others right
  // final_packets[0:4] can hold up to 5 packets (Time Wall + 4 slots)
  //----------------------------------------------------------------------------
  wire [31:0] final_packets [0:4];
  wire [2:0]  num_final;
  
  // num_final = num_slots + time_wall_vld (can be 0-5, no capping)
  assign num_final = num_slots + (time_wall_vld ? 1 : 0);
  
  // Time Wall is highest priority (index 0), then shift compacted packets right
  assign final_packets[0] = time_wall_vld ? time_wall_pkt : compacted[0];
  assign final_packets[1] = time_wall_vld ? compacted[0] : compacted[1];
  assign final_packets[2] = time_wall_vld ? compacted[1] : compacted[2];
  assign final_packets[3] = time_wall_vld ? compacted[2] : compacted[3];
  assign final_packets[4] = time_wall_vld ? compacted[3] : 32'b0;  // 5th packet only when Time Wall + 4 slots

  //----------------------------------------------------------------------------
  // Step 4: Shadow Accumulator (128-bit = 4×32-bit packets)
  // fill_level: 0, 1, 2, 3, or 4 packets already stored (expanded to 3 bits)
  // Layout: [127:96]=pkt3, [95:64]=pkt2, [63:32]=pkt1, [31:0]=pkt0
  //----------------------------------------------------------------------------
  reg [127:0] accumulator;  // Shadow buffer
  reg [2:0]   fill_level;   // 0-4 packets in accumulator (expanded to 3 bits for safety)

  assign fill_level_out = fill_level[1:0];  // Output only needs 2 bits for compatibility

  //----------------------------------------------------------------------------
  // Step 5: Spliced Bus (Sliding Window) - Linear combination of accumulator + final_packets
  // combined_bus[0:7] holds up to 8 packets (max: fill_level=4 + num_final=5 = 9, but capped at 8)
  //----------------------------------------------------------------------------
  reg [31:0] combined_bus [0:7];
  wire [3:0] total_count_raw = fill_level + num_final;
  wire [3:0] total_count = (total_count_raw > 4'd8) ? 4'd8 : total_count_raw;

  // Structured splicing logic to prevent bubbles: accumulator first, then final_packets
  integer i;
  always @(*) begin
    // 1. Default clear all slots
    for (i = 0; i < 8; i = i + 1) begin
      combined_bus[i] = 32'b0;
    end
    
    // 2. Place existing shadow data (accumulator) at indices [0:fill_level-1]
    // fill_level = number of packets stored (0-4)
    // fill_level = 0: no accumulator data
    // fill_level = 1: accumulator[31:0] at index 0
    // fill_level = 2: accumulator[31:0] at index 0, accumulator[63:32] at index 1
    // fill_level = 3: accumulator[31:0] at index 0, accumulator[63:32] at index 1, accumulator[95:64] at index 2
    // fill_level = 4: accumulator[31:0] at index 0, accumulator[63:32] at index 1, accumulator[95:64] at index 2, accumulator[127:96] at index 3
    if (fill_level >= 1) combined_bus[0] = accumulator[31:0];
    if (fill_level >= 2) combined_bus[1] = accumulator[63:32];
    if (fill_level >= 3) combined_bus[2] = accumulator[95:64];
    if (fill_level >= 4) combined_bus[3] = accumulator[127:96];
    
    // 3. Place new packets starting EXACTLY at fill_level offset
    // combined_bus[fill_level + i] = final_packets[i] for i = 0 to num_final-1
    // This ensures no gaps: accumulator uses [0:fill_level-1], final_packets use [fill_level:fill_level+num_final-1]
    if (num_final >= 1) combined_bus[fill_level + 0] = final_packets[0];
    if (num_final >= 2) combined_bus[fill_level + 1] = final_packets[1];
    if (num_final >= 3) combined_bus[fill_level + 2] = final_packets[2];
    if (num_final >= 4) combined_bus[fill_level + 3] = final_packets[3];
    if (num_final >= 5) combined_bus[fill_level + 4] = final_packets[4];
  end

  //----------------------------------------------------------------------------
  // Step 6: Simplified Write Logic using Spliced Bus
  //----------------------------------------------------------------------------
  // Write when total_count >= 4
  assign fifo_wr_en = (total_count >= 4);
  
  // Always take the first 4 slots from combined_bus for FIFO write
  assign fifo_wr_data = {combined_bus[3], combined_bus[2], combined_bus[1], combined_bus[0]};

  // Remainder handling using actual total_count subtraction
  wire [2:0] remainder_count = (total_count >= 4) ? (total_count[2:0] - 3'd4) : 3'd0;
  reg [127:0] accumulator_next;
  reg [2:0]   fill_level_next;

  always @(*) begin
    // Safety clear: prevent stale data leakage
    accumulator_next = 128'b0;
    
    if (total_count >= 4) begin
      // Write occurred: remainder = total_count - 4
      // fill_level_next can be 0-4 (3 bits)
      fill_level_next = (remainder_count > 4) ? 3'd0 : remainder_count;
      
      // Packets starting from index 4 are the remainders (must be taken from combined_bus[4:7])
      // Can store up to 4 packets in accumulator (remainder_count max = 4 when total_count = 8)
      accumulator_next[31:0]   = (remainder_count >= 1) ? combined_bus[4] : 32'b0;
      accumulator_next[63:32]  = (remainder_count >= 2) ? combined_bus[5] : 32'b0;
      accumulator_next[95:64]  = (remainder_count >= 3) ? combined_bus[6] : 32'b0;
      accumulator_next[127:96] = (remainder_count >= 4) ? combined_bus[7] : 32'b0;
    end else begin
      // No write: store all packets in accumulator
      fill_level_next = (total_count > 4) ? 3'd4 : total_count[2:0];
      accumulator_next[31:0]   = (total_count >= 1) ? combined_bus[0] : 32'b0;
      accumulator_next[63:32]  = (total_count >= 2) ? combined_bus[1] : 32'b0;
      accumulator_next[95:64]  = (total_count >= 3) ? combined_bus[2] : 32'b0;
      accumulator_next[127:96] = (total_count >= 4) ? combined_bus[3] : 32'b0;
    end
  end

  //----------------------------------------------------------------------------
  // Sequential update @ 500 MHz
  //----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      accumulator <= 128'b0;
      fill_level  <= 3'b0;
    end else begin
      accumulator <= accumulator_next;
      fill_level  <= fill_level_next;
    end
  end

endmodule
