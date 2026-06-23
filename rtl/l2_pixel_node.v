//==============================================================================
// l2_pixel_node.v — Top-Level L2 Tile (Fine Timer, Event Counter, FIFO, Bus Logic)
//==============================================================================
// Part of: 16×8 Photodetector Readout ASIC (DarkSide-20k)
// Integrates: l2_fine_timer, l2_event_counter, l2_fifo (FWFT), l2_logic.
// Packet Builder FSM: Time+Event on first hit with energy; Event-only otherwise.
// All logic at 500 MHz; SPAD hit synchronized 2-stage.
//==============================================================================

module l2_pixel_node #(
  parameter VAL_THR1 = 8'd1,
  parameter VAL_THR2 = 8'd2,
  parameter VAL_THR3 = 8'd3,
  parameter NODE_ID  = 4'd0   // Unique ID for debug (0=Pixel 0, 15=Pixel 15)
) (
  input  wire        clk,
  input  wire        rst_n,
  input  wire        global_sync,

  input  wire        spad_hit,           // async; 2-stage synced internally
  input  wire [31:0] l1_in,              // 16 L1 pixels × 2-bit → event_counter

  input  wire [19:0] bus_in,
  input  wire        bus_in_valid,
  input  wire        row_stall,     // Phase 1: backpressure; when high, freeze chain and gate FIFO read

  output wire [19:0] bus_out,
  output wire        bus_out_valid,

  output wire        skid_valid_out,
  output wire        toggle_out
);

  //----------------------------------------------------------------------------
  // 2-stage synchronizer for async spad_hit (500 MHz domain)
  //----------------------------------------------------------------------------
  reg spad_sync1, spad_sync2;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      spad_sync1 <= 1'b0;
      spad_sync2 <= 1'b0;
    end else begin
      spad_sync1 <= spad_hit;
      spad_sync2 <= spad_sync1;
    end
  end
  wire spad_hit_synced = spad_sync2;

  //----------------------------------------------------------------------------
  // CDC: 2-stage synchronizer for global_sync (100MHz -> 500MHz domain)
  //----------------------------------------------------------------------------
  reg global_sync_r1, global_sync_r2;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      global_sync_r1 <= 1'b0;
      global_sync_r2 <= 1'b0;
    end else begin
      global_sync_r1 <= global_sync;
      global_sync_r2 <= global_sync_r1;
    end
  end

  //----------------------------------------------------------------------------
  // Edge detection: rising edge of sync_r2 = 1-cycle pulse
  // Also: hold expecting_first_hit reset when global_sync is high (level-sensitive)
  // so a 50ns pulse guarantees all pixels see the reset
  //----------------------------------------------------------------------------
  reg global_sync_r2_dly;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      global_sync_r2_dly <= 1'b0;
    else
      global_sync_r2_dly <= global_sync_r2;
  end
  wire global_sync_aligned  = global_sync_r2 & ~global_sync_r2_dly;  // Edge pulse
  wire global_sync_held     = global_sync_r2;  // Level: sync has propagated

  //----------------------------------------------------------------------------
  // Sub-module: Fine Timer (15-bit relative time)
  //----------------------------------------------------------------------------
  wire [14:0] fine_time;
  l2_fine_timer u_fine_timer (
    .clk          (clk),
    .rst_n        (rst_n),
    .global_sync  (global_sync_aligned),  // Use CDC-aligned sync pulse
    .fine_time_val(fine_time)
  );

  //----------------------------------------------------------------------------
  // Sub-module: Event Counter (10 ns report, zones, energy)
  //----------------------------------------------------------------------------
  wire [7:0] energy_out;
  wire [3:0] zone_mask_out;
  wire [2:0] status_out;
  wire       report_valid;
  l2_event_counter #(
    .VAL_THR1 (VAL_THR1),
    .VAL_THR2 (VAL_THR2),
    .VAL_THR3 (VAL_THR3)
  ) u_event_counter (
    .clk           (clk),
    .rst_n         (rst_n),
    .l1_in         (l1_in),
    .energy_out    (energy_out),
    .zone_mask_out (zone_mask_out),
    .status_out    (status_out),
    .report_valid  (report_valid)
  );

  //----------------------------------------------------------------------------
  // L2 FIFO (FWFT): wire declarations (needed before FSM uses fifo_full)
  //----------------------------------------------------------------------------
  wire        send_local;   // From l2_logic; gated by ~row_stall for actual FIFO read
  wire        fifo_rd_en;   // Gated: only advance FIFO when row not stalled (no data loss)
  wire [15:0] fifo_rd_data;
  wire        fifo_empty;
  wire        fifo_full;
  wire [4:0]  fifo_count;

  //----------------------------------------------------------------------------
  // FSM state and data (declared first so consume_one and request buffer can use them)
  //----------------------------------------------------------------------------
  localparam IDLE = 2'b00, WRITE_TIME = 2'b01, WRITE_EVENT = 2'b10;
  reg [1:0]  state_r;
  reg        expecting_first_hit;
  reg [3:0]  zone_latched;
  reg [2:0]  status_latched;
  reg [7:0]  energy_latched;
  reg [14:0] fine_time_latched;

  // Registered FIFO interface ("Outer Loose"): 2ns setup, avoids state-vs-data race
  reg        fifo_wr_en;
  reg [15:0] fifo_wr_data;

  wire energy_gt_zero = (energy_out != 8'd0);

  // Edge detection: one hit = one report (avoid overcount on wide pulses)
  reg report_valid_d1;
  always @(posedge clk or negedge rst_n)
    if (!rst_n) report_valid_d1 <= 1'b0;
    else        report_valid_d1 <= report_valid;
  wire report_edge = report_valid && !report_valid_d1;

  reg [1:0] pending_reports;  // 0, 1, or 2 pending reports (2-level buffer)
  reg [7:0] report_energy [0:1];  // energy for slot 0 (oldest) and slot 1
  reg [3:0] report_zone   [0:1];
  reg [2:0] report_status [0:1];

  wire consume_one = (state_r == WRITE_EVENT) & ~fifo_full;

  //----------------------------------------------------------------------------
  // state_next: combinational next state (for FIFO look-ahead, zero-lag)
  //----------------------------------------------------------------------------
  reg [1:0] state_next;
  always @(*) begin
    state_next = state_r;
    case (state_r)
      IDLE: begin
        if ((pending_reports > 2'd0) & ~fifo_full)
          state_next = expecting_first_hit ? WRITE_TIME : WRITE_EVENT;
      end
      WRITE_TIME: begin
        if (~fifo_full) state_next = WRITE_EVENT;
      end
      WRITE_EVENT: begin
        if (~fifo_full) begin
          if (pending_reports <= 2'd1)
            state_next = IDLE;
          else
            state_next = expecting_first_hit ? WRITE_TIME : WRITE_EVENT;
        end
      end
      default: state_next = IDLE;
    endcase
  end

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pending_reports <= 2'd0;
      report_energy[0] <= 8'd0; report_energy[1] <= 8'd0;
      report_zone[0]   <= 4'd0; report_zone[1]   <= 4'd0;
      report_status[0] <= 3'd0; report_status[1] <= 3'd0;
    end else begin
      // Latch new report on rising edge only (avoid overcount)
      if (report_edge & energy_gt_zero & (pending_reports < 2'd2)) begin
        $display("[Pixel %0d] CAPTURE [%0t] report_edge=1 pending_before=%0d slot=%0d energy=0x%02X",
                 NODE_ID, $time, pending_reports, pending_reports, energy_out);
        if (pending_reports == 2'd0) begin
          report_energy[0] <= energy_out;
          report_zone[0]   <= zone_mask_out;
          report_status[0] <= status_out;
        end else begin
          report_energy[1] <= energy_out;
          report_zone[1]   <= zone_mask_out;
          report_status[1] <= status_out;
        end
      end
      // Shift slot 1 -> 0 when consuming and we had 2 pending
      if (consume_one & (pending_reports == 2'd2)) begin
        report_energy[0] <= report_energy[1];
        report_zone[0]   <= report_zone[1];
        report_status[0] <= report_status[1];
      end
      // Single combined update: +1 on edge capture, -1 on consume (same cycle OK)
      pending_reports <= pending_reports
        + (report_edge & energy_gt_zero & (pending_reports < 2'd2) ? 1'b1 : 1'b0)
        - (consume_one & (pending_reports > 2'd0) ? 1'b1 : 1'b0);
    end
  end

  //----------------------------------------------------------------------------
  // Packet Builder FSM: Trigger only on pending_reports, not raw report_valid
  // All FIFO writes happen in WRITE_TIME or WRITE_EVENT only (no IDLE direct-write)
  //----------------------------------------------------------------------------
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state_r             <= IDLE;
      expecting_first_hit <= 1'b1;
      zone_latched        <= 4'd0;
      status_latched      <= 3'd0;
      energy_latched      <= 8'd0;
      fine_time_latched   <= 15'd0;
      fifo_wr_en          <= 1'b0;
      fifo_wr_data        <= 16'd0;
    end else begin
      // Look-ahead: drive FIFO regs from state_next so data is ready when state transitions
      if (global_sync_aligned) begin
        fifo_wr_en   <= 1'b0;
        fifo_wr_data <= 16'd0;
      end else begin
        case (state_next)
          WRITE_TIME: begin
            fifo_wr_en   <= ~fifo_full;
            fifo_wr_data <= {1'b1, (state_r == IDLE ? fine_time : fine_time_latched)};
          end
          WRITE_EVENT: begin
            fifo_wr_en   <= ~fifo_full;
            fifo_wr_data <= {1'b0, zone_latched, status_latched, energy_latched};
          end
          default: begin
            fifo_wr_en   <= 1'b0;
            fifo_wr_data <= 16'd0;
          end
        endcase
      end

      if (global_sync_aligned || global_sync_held)
        expecting_first_hit <= 1'b1;  // Reset on edge or while sync held (50ns pulse)

      case (state_r)
        IDLE: begin
          if ((pending_reports > 2'd0) & ~fifo_full) begin
            zone_latched   <= report_zone[0];
            status_latched <= report_status[0];
            energy_latched <= report_energy[0];
            if (expecting_first_hit) begin
              fine_time_latched <= fine_time;  // Latch time when entering WRITE_TIME path
              state_r <= WRITE_TIME;
            end else begin
              state_r <= WRITE_EVENT;
            end
          end
        end
        WRITE_TIME: begin
          if (~fifo_full) begin
            state_r      <= WRITE_EVENT;
            expecting_first_hit <= 1'b0;  // Clear when Time packet is being registered
            `ifndef SYNTHESIS
            $display("[Pixel %0d] FSM_WRITE [%0t] state=WRITE_TIME exp_first=%0d fifo_wr_en=1 wr_data=0x%04X Type=%0d (expect 1) %s",
                     NODE_ID, $time, expecting_first_hit, {1'b1, fine_time_latched}, {1'b1, fine_time_latched}[15],
                     ({1'b1, fine_time_latched}[15] == 1'b1) ? "OK" : "MISMATCH!");
            `endif
          end
        end
        WRITE_EVENT: begin
          if (~fifo_full) begin
            `ifndef SYNTHESIS
            $display("[Pixel %0d] FSM_WRITE [%0t] state=WRITE_EVENT exp_first=%0d fifo_wr_en=1 wr_data=0x%04X Type=%0d (expect 0) %s",
                     NODE_ID, $time, expecting_first_hit, {1'b0, zone_latched, status_latched, energy_latched},
                     {1'b0, zone_latched, status_latched, energy_latched}[15],
                     ({1'b0, zone_latched, status_latched, energy_latched}[15] == 1'b0) ? "OK" : "MISMATCH!");
            `endif
            if (pending_reports <= 2'd1) begin
              state_r <= IDLE;
            end else begin
              zone_latched   <= report_zone[1];
              status_latched <= report_status[1];
              energy_latched <= report_energy[1];
              if (expecting_first_hit)
                fine_time_latched <= fine_time;  // Latch when chaining to WRITE_TIME
              state_r        <= expecting_first_hit ? WRITE_TIME : WRITE_EVENT;
            end
          end
        end
        default: begin
          state_r <= IDLE;
        end
      endcase
    end
  end

  //----------------------------------------------------------------------------
  // Granular debug + FATAL TIMING safety check
  //----------------------------------------------------------------------------
  always @(posedge clk) begin
    if (rst_n) begin
      if (fifo_rd_en)
        $display("[Pixel %0d] FIFO_EXIT [%0t] send_local=1 rd_data=0x%04X Type=%0d -> bus",
                 NODE_ID, $time, fifo_rd_data, fifo_rd_data[15]);
      // Verification: pending decrements when consume_one (same condition as WRITE_EVENT fifo_wr_en)
      if (consume_one && (pending_reports > 2'd0))
        $display("[Pixel %0d] VERIFY [%0t] pending_decrement consume_one=1 fifo_wr_en=1(WRITE_EVENT) pending=%0d",
                 NODE_ID, $time, pending_reports);
    end
  end

  //----------------------------------------------------------------------------
  // L2 FIFO (FWFT): writer = Packet Builder; reader = l2_logic (send_local)
  //----------------------------------------------------------------------------
  l2_fifo u_fifo (
    .clk     (clk),
    .rst_n   (rst_n),
    .wr_en   (fifo_wr_en),
    .wr_data (fifo_wr_data),
    .rd_en   (fifo_rd_en),
    .rd_data (fifo_rd_data),
    .full    (fifo_full),
    .empty   (fifo_empty),
    .count   (fifo_count)
  );

  //----------------------------------------------------------------------------
  // L2 Bus Logic: Autonomous readout from FIFO
  // local_data = 20-bit (NODE_ID[3:0] + 16-bit packet); high 4 bits identify pixel
  //----------------------------------------------------------------------------
  wire [19:0] local_data  = {NODE_ID[3:0], fifo_rd_data};
  wire        local_valid = ~fifo_empty;

  wire [19:0] bus_out_pre;
  wire        bus_out_valid_pre;

  l2_logic u_logic (
    .clk            (clk),
    .rst_n          (rst_n),
    .global_sync    (global_sync_aligned),
    .bus_in         (bus_in),
    .bus_in_valid   (bus_in_valid),
    .local_data     (local_data),
    .local_valid    (local_valid),
    .bus_out        (bus_out_pre),
    .bus_out_valid  (bus_out_valid_pre),
    .send_local_out (send_local),
    .skid_valid_out (skid_valid_out),
    .toggle_out     (toggle_out)
  );

  // Gate FIFO read: only consume when row is not stalled (prevents overwrite / data loss)
  assign fifo_rd_en = send_local & ~row_stall;

  //----------------------------------------------------------------------------
  // Phase 1: Freeze output when row_stall — hold bus_out/bus_out_valid so chain does not advance
  //----------------------------------------------------------------------------
  reg [19:0] bus_out_r;
  reg        bus_out_valid_r;
  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bus_out_r      <= 20'b0;
      bus_out_valid_r <= 1'b0;
    end else if (!row_stall) begin
      bus_out_r      <= bus_out_pre;
      bus_out_valid_r <= bus_out_valid_pre;
    end
    // when row_stall: hold previous values
  end
  assign bus_out       = bus_out_r;
  assign bus_out_valid = bus_out_valid_r;

endmodule
