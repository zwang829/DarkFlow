//==============================================================================
// l2_fine_timer.v — 15-bit Fine Time Counter for L2 Pixel Node
//==============================================================================
// Part of: l2PixelNode (16×8 Photodetector Readout ASIC)
// Clock: 500 MHz (2 ns period). Increments every cycle.
// Reset: Synchronous zero on global_sync from L3.
// Output: Relative timestamp for L2 time packets (15-bit offset).
//==============================================================================

module l2_fine_timer (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        global_sync,   // from L3; resets counter to 0

  output wire [14:0] fine_time_val
);

  reg [14:0] count;

  assign fine_time_val = count;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      count <= 15'b0;
    else if (global_sync)
      count <= 15'b0;
    else
      count <= count + 1'b1;
  end

endmodule
