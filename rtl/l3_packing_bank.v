//==============================================================================
// l3_packing_bank.v — Per-Bank L3 Word Assembler (4 lanes → 128-bit FIFO)
//==============================================================================
// Part of: L3 Sensor (Dual-Bank Parallel Readout)
// Clock: 500 MHz
// - Serves 4 rows (23-bit row packets: {ROW_ID[22:20], 20-bit L2 payload})
// - Stage 0: Registers all inputs from L2 (valid_in_r, data_in_r0..3)
// - 8-slot buffer: reg [31:0] buffer[0:7], fill_level 0..8
// - Greedy capture: up to 4 new packets per cycle when fill_level <= 4
// - Tetris shift: on 128-bit write, buffer[4..7] shift down to buffer[0..3]
// - Atomic heartbeat flush at 1000 ns: old buffer flushed with 2'b11 Null pads,
//   same-cycle arrivals held and used to seed next epoch
// - 7-bit sequence index advanced every 10 ns, reset after heartbeat flush
// - All I/Os (fifo_wr_en, fifo_wr_data, stall) are registered
//==============================================================================

module l3_packing_bank (
  input  wire        clk,
  input  wire        rst_n,

  // Four row inputs for this bank (23-bit internal format)
  input  wire [3:0]  valid_in,    // one bit per row in this bank
  input  wire [22:0] data_in_0,
  input  wire [22:0] data_in_1,
  input  wire [22:0] data_in_2,
  input  wire [22:0] data_in_3,

  // 128-bit FIFO interface (bank-local)
  input  wire        fifo_ready,      // 0 => downstream FIFO cannot accept a write
  output reg         fifo_wr_en,
  output reg  [127:0] fifo_wr_data,

  // Bank-level backpressure toward rows (to be mapped to row_stall[*])
  output reg         stall,

  // 7-bit sequence index (10 ns step) and heartbeat flush indicator (1000 ns)
  output reg  [6:0]  seq_index,
  output reg         heartbeat_flush
);

  //----------------------------------------------------------------------------
  // Helper: 23→32 bit format: Type [31:30], Reserved [29:23], RowID [22:20],
  // PixelID [19:16], Payload [15:0].
  // Input [15]=1 => Time => 2'b10; [15]=0 => Event => 2'b01.
  //----------------------------------------------------------------------------
  function [31:0] fmt_23_to_32;
    input [22:0] d;
    begin
      fmt_23_to_32 = {
        d[15] ? 2'b10 : 2'b01,  // [31:30] Type (Time/Event)
        7'b0,                   // [29:23] Reserved
        d[22:20],               // [22:20] Row ID from packet
        d[19:16],               // [19:16] Pixel ID
        d[15:0]                 // [15:0]  L2 payload
      };
    end
  endfunction

  // Null package: Type=2'b11, rest 0
  localparam [31:0] NULL_PKT = {2'b11, 30'b0};

  //----------------------------------------------------------------------------
  // Stage 0: Register all inputs from L2
  //----------------------------------------------------------------------------
  reg [3:0]  valid_in_r;
  reg [22:0] data_in_r0, data_in_r1, data_in_r2, data_in_r3;

  //----------------------------------------------------------------------------
  // 8-slot buffer (32b each) and fill level
  //----------------------------------------------------------------------------
  reg [31:0] buffer [0:7];
  reg [3:0]  fill_level;      // 0..8

  // Pending arrivals captured in heartbeat cycle (to seed next epoch)
  reg [3:0]  pending_valid;
  reg [22:0] pending_data [0:3];

  // Idle counter for optional timeout flush (kept simple)
  reg [5:0]  idle_count;

  //----------------------------------------------------------------------------
  // Sequence index and heartbeat generator (10 ns tick, 1000 ns heartbeat)
  //----------------------------------------------------------------------------
  reg [2:0] ten_ns_div;       // 0..4 → 5 cycles = 10 ns
  reg [6:0] hb_tick_count;    // 0..99 → 100×10ns = 1000ns
  reg       seq_reset_pending;

  wire ten_ns_tick = (ten_ns_div == 3'd4);
  wire hb_pulse    = ten_ns_tick && (hb_tick_count == 7'd99);

  integer i;

  // Temporary / helper variables for non-heartbeat cycles (must be declared at
  // module scope for DC compatibility; older Verilog flows do not allow
  // declarations inside unnamed procedural blocks).
  reg [3:0]  src_valid;
  reg [22:0] src_data [0:3];
  reg [31:0] buf_tmp  [0:7];
  integer    j;
  integer    eff_fill;
  integer    accepted;
  integer    fill_after;

  //----------------------------------------------------------------------------
  // Sequential logic: Stage 0 capture, heartbeat/seq_index, buffer + control
  //----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      // Stage 0
      valid_in_r      <= 4'b0;
      data_in_r0      <= 23'b0;
      data_in_r1      <= 23'b0;
      data_in_r2      <= 23'b0;
      data_in_r3      <= 23'b0;

      // Buffer
      for (i = 0; i < 8; i = i + 1)
        buffer[i]      <= 32'b0;
      fill_level      <= 4'd0;

      pending_valid   <= 4'b0;
      pending_data[0] <= 23'b0;
      pending_data[1] <= 23'b0;
      pending_data[2] <= 23'b0;
      pending_data[3] <= 23'b0;

      idle_count      <= 6'd0;

      fifo_wr_en      <= 1'b0;
      fifo_wr_data    <= 128'b0;
      stall           <= 1'b0;

      ten_ns_div      <= 3'd0;
      hb_tick_count   <= 7'd0;
      seq_index       <= 7'd0;
      heartbeat_flush <= 1'b0;
      seq_reset_pending <= 1'b0;
    end else begin
      // Stage 0: register raw inputs from L2
      valid_in_r <= valid_in;
      data_in_r0 <= data_in_0;
      data_in_r1 <= data_in_1;
      data_in_r2 <= data_in_2;
      data_in_r3 <= data_in_3;

      // Default outputs each cycle
      fifo_wr_en      <= 1'b0;
      fifo_wr_data    <= 128'b0;
      heartbeat_flush <= 1'b0;

      // 10 ns and heartbeat counters
      if (ten_ns_tick) begin
        ten_ns_div <= 3'd0;
        if (hb_pulse) begin
          hb_tick_count <= 7'd0;
        end else begin
          hb_tick_count <= hb_tick_count + 7'd1;
        end
      end else begin
        ten_ns_div <= ten_ns_div + 3'd1;
      end

      // Sequence index: reset one tick after heartbeat flush, otherwise increment on 10ns tick
      if (seq_reset_pending && ten_ns_tick) begin
        seq_index        <= 7'd0;
        seq_reset_pending <= 1'b0;
      end else if (ten_ns_tick) begin
        seq_index <= seq_index + 7'd1;
      end

      // -----------------------------------------------------------------------
      // HEARTBEAT FLUSH HAS HIGHEST PRIORITY
      // -----------------------------------------------------------------------
      if (hb_pulse) begin
        // Atomic flush of existing buffer (old epoch). New arrivals this cycle
        // go into pending_* and will seed the next epoch.
        if (fill_level != 0) begin
          fifo_wr_en   <= 1'b1;
          fifo_wr_data[31:0]   <= (fill_level >= 1) ? buffer[0] : NULL_PKT;
          fifo_wr_data[63:32]  <= (fill_level >= 2) ? buffer[1] : NULL_PKT;
          fifo_wr_data[95:64]  <= (fill_level >= 3) ? buffer[2] : NULL_PKT;
          fifo_wr_data[127:96] <= (fill_level >= 4) ? buffer[3] : NULL_PKT;
          heartbeat_flush      <= 1'b1;
        end

        // Clear buffer for next epoch
        for (i = 0; i < 8; i = i + 1)
          buffer[i] <= 32'b0;
        fill_level <= 4'd0;

        // Capture current-cycle arrivals into pending_* (fenced from this flush)
        pending_valid   <= valid_in_r;
        pending_data[0] <= data_in_r0;
        pending_data[1] <= data_in_r1;
        pending_data[2] <= data_in_r2;
        pending_data[3] <= data_in_r3;

        // Mark that seq_index should reset on the next 10ns tick
        seq_reset_pending <= 1'b1;

        // Idle count can reset on activity (flush acts as activity)
        idle_count <= 6'd0;

        // Stall only reflects fifo_ready; buffer is empty so no space issue
        stall <= ~fifo_ready;
      end else begin
        // ---------------------------------------------------------------------
        // NON-HEARTBEAT CYCLE: Greedy capture + optional write + Tetris shift
        // ---------------------------------------------------------------------

        // Initialize temporary buffer with current buffer contents
        for (j = 0; j < 8; j = j + 1)
          buf_tmp[j] = buffer[j];

        // Choose source for this cycle:
        //  - If we have pending data from a heartbeat cycle, consume that first.
        //  - Otherwise, use current registered inputs.
        if (pending_valid != 4'b0) begin
          src_valid   = pending_valid;
          src_data[0] = pending_data[0];
          src_data[1] = pending_data[1];
          src_data[2] = pending_data[2];
          src_data[3] = pending_data[3];
          pending_valid   <= 4'b0;
          pending_data[0] <= 23'b0;
          pending_data[1] <= 23'b0;
          pending_data[2] <= 23'b0;
          pending_data[3] <= 23'b0;
        end else begin
          src_valid   = valid_in_r;
          src_data[0] = data_in_r0;
          src_data[1] = data_in_r1;
          src_data[2] = data_in_r2;
          src_data[3] = data_in_r3;
        end

        // Greedy capture: accept up to 4 new packets when we have room (fill<=4)
        eff_fill = fill_level;
        accepted = 0;
        if (fill_level <= 4) begin
          // Lanes 0..3 in order; no arbitration, just sequentially assign
          if (src_valid[0] && eff_fill < 8) begin
            buf_tmp[eff_fill] = fmt_23_to_32(src_data[0]);
            eff_fill = eff_fill + 1;
            accepted = accepted + 1;
          end
          if (src_valid[1] && eff_fill < 8) begin
            buf_tmp[eff_fill] = fmt_23_to_32(src_data[1]);
            eff_fill = eff_fill + 1;
            accepted = accepted + 1;
          end
          if (src_valid[2] && eff_fill < 8) begin
            buf_tmp[eff_fill] = fmt_23_to_32(src_data[2]);
            eff_fill = eff_fill + 1;
            accepted = accepted + 1;
          end
          if (src_valid[3] && eff_fill < 8) begin
            buf_tmp[eff_fill] = fmt_23_to_32(src_data[3]);
            eff_fill = eff_fill + 1;
            accepted = accepted + 1;
          end
        end

        // Idle counter: reset on any accept, otherwise increment/saturate
        if (accepted > 0)
          idle_count <= 6'd0;
        else if (idle_count >= 6'd32)
          idle_count <= 6'd32;
        else
          idle_count <= idle_count + 6'd1;

        // Decide whether to perform a normal 4-word write (do_write)
        // Heartbeat has already been handled above, so priority here is do_write.
        if ((eff_fill >= 4) && fifo_ready) begin
          // Normal 4-word write
          fifo_wr_en            <= 1'b1;
          fifo_wr_data[31:0]    <= buf_tmp[0];
          fifo_wr_data[63:32]   <= buf_tmp[1];
          fifo_wr_data[95:64]   <= buf_tmp[2];
          fifo_wr_data[127:96]  <= buf_tmp[3];

          fill_after = eff_fill - 4;
          // Tetris shift: buffer[4..7] → buffer[0..3]
          for (j = 0; j < 8; j = j + 1) begin
            if (j < fill_after)
              buffer[j] <= buf_tmp[j+4];
            else
              buffer[j] <= 32'b0;
          end
          fill_level <= fill_after[3:0];

          // Stall for next cycle if space will be low, or FIFO not ready now
          stall <= ((fill_after > 4) || ~fifo_ready);
        end else begin
          // No normal write: just commit updated buf_tmp/eff_fill
          for (j = 0; j < 8; j = j + 1)
            buffer[j] <= buf_tmp[j];
          fill_level <= eff_fill[3:0];

          stall <= ((eff_fill > 4) || ~fifo_ready);
        end
      end
    end
  end

endmodule

