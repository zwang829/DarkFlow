//==============================================================================
// l3_timer.v — 30-bit Absolute Timer with Periodic Global Sync
//==============================================================================
// Part of: L3 System Top (16×8 Photodetector Readout ASIC)
// Clock: 100 MHz (10 ns period). Generates global_sync every SYNC_PERIOD cycles.
// Time Wall Packet: [31:30]=2'b11 (Type), [29:0]=absolute_time (30-bit).
//==============================================================================

module l3_timer #(
  parameter SYNC_PERIOD = 1000  // Default: 10 microseconds at 100MHz (1000 * 10ns = 10us)
) (
  input  wire        clk,              // 100 MHz clock
  input  wire        rst_n,

  output reg         global_sync,      // 1-cycle pulse (10ns wide) to L2 array
  output reg [31:0]  time_wall_pkt,    // Time Wall Packet: {2'b11, abs_time[29:0]}
  output reg         time_wall_vld     // Valid flag for Time Wall Packet
);

  //----------------------------------------------------------------------------
  // 30-bit absolute time counter (increments every 10ns cycle)
  //----------------------------------------------------------------------------
  reg [29:0] abs_time_reg;

  //----------------------------------------------------------------------------
  // Cycle counter for sync period (counts 0 to SYNC_PERIOD-1)
  //----------------------------------------------------------------------------
  reg [9:0] cycle_cnt;  // Enough bits for SYNC_PERIOD (1000 needs 10 bits)

  //----------------------------------------------------------------------------
  // Sync generation: assert global_sync when cycle_cnt reaches SYNC_PERIOD-1
  // Latch abs_time_reg at sync moment to avoid timing jitters
  //----------------------------------------------------------------------------
  wire sync_trigger = (cycle_cnt == (SYNC_PERIOD - 1));

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      abs_time_reg  <= 30'b0;
      cycle_cnt     <= 10'b0;
      global_sync   <= 1'b0;
      time_wall_pkt <= 32'b0;
      time_wall_vld <= 1'b0;
    end else begin
      // Increment absolute time every cycle
      abs_time_reg <= abs_time_reg + 1'b1;

      // Generate sync pulse and Time Wall Packet
      if (sync_trigger) begin
        // Assert global_sync for 1 cycle
        global_sync <= 1'b1;
        
        // Latch current abs_time_reg into Time Wall Packet (clean capture)
        time_wall_pkt <= {2'b11, abs_time_reg};
        time_wall_vld <= 1'b1;
        
        // Reset cycle counter
        cycle_cnt <= 13'b0;
      end else begin
        // Clear sync pulse after 1 cycle
        global_sync <= 1'b0;
        time_wall_vld <= 1'b0;
        
        // Increment cycle counter
        cycle_cnt <= cycle_cnt + 1'b1;
      end
    end
  end

endmodule
