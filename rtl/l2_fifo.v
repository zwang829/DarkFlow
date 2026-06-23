//==============================================================================
// l2_fifo.v — Synchronous L2 FIFO (16×16b) with Registered FWFT for 500 MHz
//==============================================================================
// Part of: l2PixelNode (16×8 Photodetector Readout ASIC)
// First-Word Fall-Through: rd_data shows head whenever !empty (no rd_en needed).
// Output is registered for 2 ns timing; rd_en = "consume" (ack current, advance).
// Overflow/underflow: no write when full, no read when empty.
//==============================================================================

module l2_fifo (
  input  wire        clk,
  input  wire        rst_n,

  input  wire        wr_en,
  input  wire [15:0] wr_data,

  input  wire        rd_en,         // consume: ack current head, load next
  output wire [15:0] rd_data,

  output wire        full,
  output wire        empty,         // 1 => rd_data invalid; 0 => rd_data valid (FWFT)
  output wire [ 4:0] count
);

  //----------------------------------------------------------------------------
  // Memory: register array (no RAM macro) for 2 ns closure
  //----------------------------------------------------------------------------
  reg [15:0] mem [0:15];

  //----------------------------------------------------------------------------
  // Circular pointers (4-bit) and occupancy count (5-bit: 0..16)
  //----------------------------------------------------------------------------
  reg [3:0] wr_ptr;
  reg [3:0] rd_ptr;
  reg [4:0] count_r;

  //----------------------------------------------------------------------------
  // Status: full/empty from count (combinational)
  // Use combinational empty so local_valid drops when count=0; registered empty
  // caused 1-cycle lag and FIFO_EXIT to repeat same packet (send_local stuck high)
  //----------------------------------------------------------------------------
  assign full  = (count_r == 5'd16);
  assign empty = (count_r == 5'd0);
  assign count = count_r;

  //----------------------------------------------------------------------------
  // FWFT output stage: holds current head; valid when !empty (look-ahead for l2_logic)
  //----------------------------------------------------------------------------
  reg [15:0] rd_data_reg;
  assign rd_data = rd_data_reg;

  //----------------------------------------------------------------------------
  // When to load output register with new head (keep path shallow)
  // - First word into empty FIFO: show wr_data
  // - Consume last and write same cycle: new head is wr_data
  // - Consume with count>1: new head is mem[rd_ptr+1]
  //----------------------------------------------------------------------------
  wire do_wr = wr_en & ~full;
  wire do_rd = rd_en & ~empty;

  wire load_head_from_wr = do_wr & (empty | (do_rd & (count_r == 5'd1)));
  wire load_head_from_mem = do_rd & (count_r > 5'd1);
  wire [3:0] next_rd_ptr = rd_ptr + 1'b1;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr      <= 4'd0;
      rd_ptr      <= 4'd0;
      count_r     <= 5'd0;
      rd_data_reg <= 16'd0;
    end else begin
      if (do_wr)
        mem[wr_ptr] <= wr_data;

      if (load_head_from_wr)
        rd_data_reg <= wr_data;
      else if (load_head_from_mem)
        rd_data_reg <= mem[next_rd_ptr];
      // else keep rd_data_reg for FWFT hold or going empty

      if (do_wr)   wr_ptr <= wr_ptr + 1'b1;
      if (do_rd)   rd_ptr <= rd_ptr + 1'b1;

      count_r <= count_r + (do_wr ? 1'b1 : 1'b0) - (do_rd ? 1'b1 : 1'b0);
    end
  end

endmodule
