# DarkFlow SiPM Readout — RTL Archive

Verilog implementation of the **DarkFlow SiPM Architecture**(L2 & L3) (see [DarkFlow: Hierarchical Digital SiPM Architecture with Low-Loss Dataflow Readout for Dark Matter Detection](https://dl.acm.org/doi/10.1145/3787109.3815257)) for design details). The design targets a 16×8 photodetector readout array with a two-level (L2/L3) hierarchical data path.

## About the paper
The definitive Version of Record was published in GLSVLSI '26, June 24–26, 2026, Finger Lakes, NY.  
DOI: https://doi.org/10.1145/3787109.3815257

The PDF included in this repository is the **author's corrected version**. It contains a minor correction in Figure 6 where the x-axis data have been multiplied by 10^9 to align with the hits/s/pixel unit used in the text. The conference proceedings inadvertently omitted this scaling. All textual and mathematical analysis remain unchanged.

> **Please cite the official ACM publication, not this repository:**

## Hierarchy (brief)

```
l3_sensor_top
├── l3_timer
├── 8 × l3_pixel_row
│   └── 16 × l2_pixel_node
│       ├── l2_fine_timer
│       ├── l2_event_counter
│       ├── l2_fifo
│       └── l2_logic
└── 2 × l3_packing_bank   (left: even rows, right: odd rows)
```

## `rtl/` — Design Files

| File | Function |
|------|----------|
| `l2_pixel_node.v` | Top-level L2 tile; integrates timer, event counter, FIFO, and bus logic into one pixel node |
| `l2_fine_timer.v` | 15-bit fine-grained timestamp counter, reset by `global_sync` |
| `l2_event_counter.v` | Accumulates weighted L1 pixel events over a 10 ns window; outputs energy and zone mask |
| `l2_fifo.v` | 16-entry synchronous FIFO for L2 event packets (FWFT) |
| `l2_logic.v` | Systolic bus arbiter with skid buffer and fairness toggle |
| `l3_pixel_row.v` | 16-pixel systolic row; daisy-chains L2 nodes and prepends row ID |
| `l3_sensor_top.v` | Top-level L3 sensor (8 rows); instantiates rows, timer, and dual packing banks |
| `l3_packing_bank.v` | Assembles 4 row streams into 128-bit FIFO words with heartbeat flush and backpressure |
| `l3_timer.v` | Absolute 30-bit timer; generates periodic `global_sync` and Time Wall packets |
| `l3_timer.txt` | Text copy of `l3_timer.v` |


## Reference

For packet formats, timing, arbitration policies, and packing algorithms, refer to [DarkFlow: Hierarchical Digital SiPM Architecture with Low-Loss Dataflow Readout for Dark Matter Detection](https://dl.acm.org/doi/10.1145/3787109.3815257)
