//==============================================================================
// l3_pixel_row.v — Systolic Daisy-Chain Row of 16 L2 Pixel Nodes
//==============================================================================
// Part of: 16×8 Photodetector Readout ASIC (DarkSide-20k)
// Clock: 500 MHz
// Architecture: 16-stage systolic pipeline (left to right)
// Output: 23-bit packets = {ROW_ID[2:0], 20-bit_packet}
//==============================================================================

module l3_pixel_row #(
  parameter ROW_ID = 3'd0  // Row identifier (0-7 for 8 rows)
) (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        global_sync,  // From L3 timer (100MHz domain)

  // Per-pixel inputs (16 pixels) - individual ports for compatibility
  input  wire        spad_hit_0, spad_hit_1, spad_hit_2, spad_hit_3,
                     spad_hit_4, spad_hit_5, spad_hit_6, spad_hit_7,
                     spad_hit_8, spad_hit_9, spad_hit_10, spad_hit_11,
                     spad_hit_12, spad_hit_13, spad_hit_14, spad_hit_15,
  input  wire [31:0] l1_in_0, l1_in_1, l1_in_2, l1_in_3,
                     l1_in_4, l1_in_5, l1_in_6, l1_in_7,
                     l1_in_8, l1_in_9, l1_in_10, l1_in_11,
                     l1_in_12, l1_in_13, l1_in_14, l1_in_15,

  input  wire        row_stall,   // Phase 1: backpressure; broadcast to all 16 pixels

  // Row output: 23-bit packets (ROW_ID prepended to 20-bit bus)
  output wire [22:0] row_bus_out,
  output wire        row_bus_out_valid
);

  //----------------------------------------------------------------------------
  // Internal arrays for cleaner indexing in generate block
  //----------------------------------------------------------------------------
  wire        spad_hit [0:15];
  wire [31:0] l1_in [0:15];
  wire [19:0] pixel_bus [0:15];
  wire        pixel_bus_valid [0:15];

  // Map individual input ports to arrays
  assign spad_hit[0] = spad_hit_0; assign spad_hit[1] = spad_hit_1;
  assign spad_hit[2] = spad_hit_2; assign spad_hit[3] = spad_hit_3;
  assign spad_hit[4] = spad_hit_4; assign spad_hit[5] = spad_hit_5;
  assign spad_hit[6] = spad_hit_6; assign spad_hit[7] = spad_hit_7;
  assign spad_hit[8] = spad_hit_8; assign spad_hit[9] = spad_hit_9;
  assign spad_hit[10] = spad_hit_10; assign spad_hit[11] = spad_hit_11;
  assign spad_hit[12] = spad_hit_12; assign spad_hit[13] = spad_hit_13;
  assign spad_hit[14] = spad_hit_14; assign spad_hit[15] = spad_hit_15;

  assign l1_in[0] = l1_in_0; assign l1_in[1] = l1_in_1;
  assign l1_in[2] = l1_in_2; assign l1_in[3] = l1_in_3;
  assign l1_in[4] = l1_in_4; assign l1_in[5] = l1_in_5;
  assign l1_in[6] = l1_in_6; assign l1_in[7] = l1_in_7;
  assign l1_in[8] = l1_in_8; assign l1_in[9] = l1_in_9;
  assign l1_in[10] = l1_in_10; assign l1_in[11] = l1_in_11;
  assign l1_in[12] = l1_in_12; assign l1_in[13] = l1_in_13;
  assign l1_in[14] = l1_in_14; assign l1_in[15] = l1_in_15;

  //----------------------------------------------------------------------------
  // Bus-in for each pixel (avoids pixel_bus[i-1] when i=0 to prevent SIOB warning)
  //----------------------------------------------------------------------------
  wire [19:0] pixel_bus_in [0:15];
  wire        pixel_bus_valid_in [0:15];
  assign pixel_bus_in[0] = 20'b0;
  assign pixel_bus_valid_in[0] = 1'b0;
  genvar j;
  generate
    for (j = 1; j < 16; j = j + 1) begin : gen_bus_in
      assign pixel_bus_in[j] = pixel_bus[j-1];
      assign pixel_bus_valid_in[j] = pixel_bus_valid[j-1];
    end
  endgenerate

  //----------------------------------------------------------------------------
  // Generate block: Instantiate 16 l2_pixel_node instances in daisy-chain
  // Compatible with refactored L2 (FIFO-first, 2-level request buffer).
  // - bus_in / bus_in_valid: Pixel 0 = 0; Pixel i = pixel_bus_in[i] / pixel_bus_valid_in[i]
  // - bus_out / bus_out_valid: pixel_bus[i] / pixel_bus_valid[i] (daisy-chain left to right)
  // - global_sync: distributed to all 16 nodes
  // Pixel 15 (tail) output becomes row_bus_out with ROW_ID prepended (23-bit L3 format)
  //----------------------------------------------------------------------------
  genvar i;
  generate
    for (i = 0; i < 16; i = i + 1) begin : gen_pixel
      l2_pixel_node #(.NODE_ID(i)) u_pixel (
        .clk            (clk),
        .rst_n          (rst_n),
        .global_sync    (global_sync),
        .spad_hit       (spad_hit[i]),
        .l1_in          (l1_in[i]),
        .bus_in         (pixel_bus_in[i]),
        .bus_in_valid   (pixel_bus_valid_in[i]),
        .row_stall      (row_stall),
        .bus_out        (pixel_bus[i]),
        .bus_out_valid  (pixel_bus_valid[i]),
        .skid_valid_out (),
        .toggle_out     ()
      );
    end
  endgenerate

  //----------------------------------------------------------------------------
  // Row Output: Prepend 3-bit ROW_ID to 20-bit packet from Pixel 15 (tail)
  // Format: {ROW_ID[2:0], pixel_bus[15][19:0]} = 23-bit packet
  // Valid is 1:1 pass-through (no mismatch): row_bus_out_valid = pixel_bus_valid[15]
  // so every packet from the tail is visible with correct ROW_ID at [22:20]
  //----------------------------------------------------------------------------
  assign row_bus_out       = {ROW_ID[2:0], pixel_bus[15]};
  assign row_bus_out_valid = pixel_bus_valid[15];

endmodule
