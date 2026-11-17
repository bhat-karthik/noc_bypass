# results/: run logs, sweep data and the CSV schema

Tracker: F64. Decision D58 (one CSV schema for every design). This file says what is in `results/`, what every CSV column means, and how to turn a sweep into charts with `templates/sweep_charts.xlsx`.

## 1. Layout

| Path | What it holds | Written by |
|---|---|---|
| `R-MMDD-nn.txt` (or `.log`) | One run's log, named as its row in the tracker's run log (Section 10) | you, after each run |
| `precheck/B<n>_precheck.log`, `B<n>_mutation.log` | Claude's Verilator pre-check and mutation check of each batch | Claude |
| `precheck/B8_knees.csv`, `B8_knee_points.csv` | The knee of every configuration the sweeps use (D61), and every point measured to find them | Claude |
| `<sweep>/<sweep>.csv` | One header, then one row per point of the sweep, in the sweep's order (Section 2) | `sim/vivado/sweep.tcl` (xsim), `sim/vcs/sweep.sh` (VCS, Xcelium) |
| `<sweep>/logs/<run_id>.log` | The simulation log of each point | the same |
| `<sweep>/sweep.txt` | One line per point and attempt: verdict, checks, cycles, seconds | the same |
| `<sweep>/trace/<run_id>.csv` | A point's time series, when the sweep gives PERF `+TRACE=<n>` (E6; Section 3) | the same |
| `<sweep>/<sweep>.part<k>.csv`, `sweep.part<k>.txt` | Part k's rows and lines when a sweep runs in parts side by side (`-part k/n`); every part merges all rows into `<sweep>.csv` when it ends | the same |
| `templates/sweep_charts.xlsx` | The chart workbook (Section 5) | Claude |
| `E0/` | The first E0 trial rows of 10 Nov (B5), kept as they were | `sweep.tcl` |

A sweep can be stopped and started again at any time: a point whose `run_id` is already in the CSV is skipped. A point that fails writes no row; `sweep.txt` lists it, and the next start tries it again.

## 2. The CSV (D58)

Every PERF point prints two lines that start with `CSV,`: the header and its row. `sweep.tcl` collects them into the sweep's CSV, the header once. Every design writes the same 46 columns in the same order; a column a design does not have holds `-`. So the rows of designs A–D of one sweep sit in one file and one sheet.

Columns 1–12: the configuration.

| # | Column | Meaning | Unit or values | Designs |
|---|---|---|---|---|
| 1 | `run_id` | The point's name (the sweep's first field, passed as `+RUNID`) | text | all |
| 2 | `design` | Which design | A, B, C, D | all |
| 3 | `N` | Mesh side | 4, 8, 16 | all |
| 4 | `V` | VCs per input port | 1, 2, 4 | all |
| 5 | `pattern` | Traffic pattern (D33) | uniform, transpose, bitcomp, bitrot, shuffle, tornado, neighbor, hotspot, region, phase | all |
| 6 | `p_data` | Share of 5-flit packets; the rest are single-flit | 0 to 1 | all |
| 7 | `b_max` | Burst limit B_MAX | flits | B–D |
| 8 | `t_mode` | Subslot rule | TIGHT, SAFE | B–D |
| 9 | `policy` | C: `fixed<k>` (k_L fixed at k). D: `PROP` or `SQRT`, then `_nodamp` without damping and `_k<lo>-<hi>` when the clamp is not D18's | text | C, D |
| 10 | `epoch` | Epoch length | cycles | C, D |
| 11 | `rate` | Offered load | packets per node per cycle | all |
| 12 | `seed` | Random seed | integer | all |

Columns 13–27: the metrics of the measurement window (tracker Section 8). A packet is measured when it is created inside the window, and every measured packet must be delivered.

| # | Column | Meaning | Unit | Metric |
|---|---|---|---|---|
| 13 | `lat_avg` | Mean latency: tail ejection minus creation, source queueing included | cycles | M01 |
| 14 | `lat_p50` | Median latency | cycles | M02 |
| 15 | `lat_p99` | 99th percentile (the histogram's last bin holds 4,095 cycles and more) | cycles | M02 |
| 16 | `lat_max` | Largest latency (exact) | cycles | M02 |
| 17 | `thr_off` | Offered throughput: flits of the packets created in the window | flits per node per cycle | M04 |
| 18 | `thr_acc` | Accepted throughput: flits that reached an NI during the window | flits per node per cycle | M04 |
| 19 | `byp_pkt` | Share of measured packets delivered by bypass (0 in design A) | 0 to 1 | M07 |
| 20 | `byp_flit` | The same share by flits | 0 to 1 | M07 |
| 21 | `jain` | Jain's fairness index over the sources' delivered flits | 0 to 1 | M18 |
| 22 | `lat_flow_max` | Worst average latency of one (source, destination) flow with at least 10 measured packets | cycles | M18 |
| 23 | `stalls` | Deadlocks the watchdog saw (the run stops at the first) | count | M17 |
| 24 | `t_stall` | Cycle of the last delivery before the first deadlock (0 if none) | cycle | M17 |
| 25 | `pkts` | Measured packets delivered | count | |
| 26 | `sat` | 1 when accepted is below 95% of offered (the point is past saturation) | 0, 1 | M04 |
| 27 | `cycles` | Length of the point: warm-up, window and drain | cycles | |

Columns 28–38: the RTL's bypass and reservation counters, summed over the mesh and counted over the whole point (`perf_probe`, F49). `-` in design A.

| # | Column | Meaning |
|---|---|---|
| 28 | `turns` | Subslots in which a router could send a burst |
| 29 | `bursts` | ... in which it sent one |
| 30 | `src_cancels` | Turns skipped because the destination cancelled |
| 31 | `lost` | ... while packets for that partner waited |
| 32 | `byp_flits` | Flits sent by bypass |
| 33 | `byp_pkts` | Packets sent by bypass |
| 34 | `decisions` | Reservation decisions at the destinations |
| 35 | `accepted` | ... that reserved space |
| 36 | `dst_cancels` | ... that cancelled (sent a token) |
| 37 | `noshows` | Reservations released because no burst came |
| 38 | `two_waiting` | Reservations made while another one waited for its burst |

Columns 39–46: the two-level timetable (M12, M15, M16). `-` in designs A and B.

| # | Column | Meaning | Unit |
|---|---|---|---|
| 39 | `k_mean` | k_L of the running frame, averaged over every cycle of the point | slots |
| 40 | `k_min` | Smallest plan over the point's epochs | slots |
| 41 | `k_max` | Largest plan over the point's epochs | slots |
| 42 | `k_changes` | How often the plan changed from one epoch to the next | count |
| 43 | `share_L` | Share of the point's cycles in the local phase (tile 0) | 0 to 1 |
| 44 | `turn_max_L` | Longest time between two turns of one pair inside a region (-1: none measured) | cycles |
| 45 | `turn_max_G` | ... of one pair across regions (-1: none measured) | cycles |
| 46 | `kL_trace` | The plan of every epoch of the point, separated by `;` | slots |

Derived quantities used by the experiments:

- Rates (D61): a sweep's `rate` is a fraction of its configuration's knee, the offered load at which design B's accepted throughput falls below 95 % of the offered; the run id carries the fraction as `k<f>` (E1_8x8_D_uniform_k0.90_s1: 0.9 of 8×8 uniform's knee). `precheck/B8_knees.csv` lists the knees: per configuration the knee, the last rate below it and the first past it, the peak accepted throughput and the most accepted past the knee. Plot curves against `rate` (absolute), or against `rate` divided by the knee.
- Saturation throughput of a curve: the largest `thr_acc` among its points (flits/node/cycle); as an offered load, the `rate` of the last point with `sat` = 0 (packets/node/cycle). A sweep that stops below saturation gives only a lower bound.
- Past the knee on 8×8, designs B–D collapse instead of levelling off (`thr_acc` 0.305 to 0.056 under uniform traffic for design B): a cycle of waiting packets is broken only when the waiting pair's bypass turn comes (D06, BP08). Report their saturation throughput as the peak below the collapse, and the collapse as the largest `thr_acc` past it (`B8_knees.csv`, column `acc_past_max`). Design A and design B with XY routing level off at about 0.32.
- Gain of design X over design B at one rate: `lat_avg(B) / lat_avg(X)`, or the ratio of their saturation throughputs.
- Cancel rate (E10): `dst_cancels / decisions`.
- Turn use: `bursts / turns`.

Design A routes XY and designs B–D minimal adaptive (D06, as in [1]), so A against B measures the bypass and the routing together: under transpose traffic XY saturates first. B against C and D measures the scheduling alone (E1).

Design D warms up for five epochs before its window opens, so that k_L has settled (D60). The sweeps run designs C and D on 8×8 and 16×16 with D19's epoch: `make_sweeps.tcl` adds `-define TB_EPOCH=32768` (`262144` on 16×16). The testbenches' 8×8 default, 4,096 cycles, is for the tests and the pre-check.

## 3. TRACE files (E6)

With `+TRACE=<n>`, PERF prints one `TRACE,` line every n cycles of warm-up and window, and `sweep.tcl` writes them to `trace/<run_id>.csv`:

| Column | Meaning | Unit |
|---|---|---|
| `run_id` | The point | text |
| `cycle` | Cycles since the point began | cycles |
| `pkts` | Packets delivered in those n cycles (any packet, measured or not) | count |
| `lat_avg` | Their mean latency | cycles |
| `k_L` | Tile 0's k_L at that cycle | slots |
| `share_L` | Tile 0's local-phase share in those n cycles | 0 to 1 |

## 4. Do two simulators agree? (S06)

`sim/compare_csv.tcl` compares the rows of two sources column by column, matched by `run_id`: two CSV files, two logs, two folders, or any mix. Every column must match, because the same seed gives the same run on every simulator.

```
tclsh sim/compare_csv.tcl results/S06/S06.csv results/precheck/S06_verilator.csv
```

In the Vivado Tcl console: `set argv {results/S06/S06.csv results/precheck/S06_verilator.csv}; source sim/compare_csv.tcl`. The last line says `S06 PASS: every row is identical` or `S06 FAIL`.

## 5. Charts: templates/sweep_charts.xlsx

1. Open the sweep's CSV in Excel, select all (Ctrl+A) and copy.
2. Open `templates/sweep_charts.xlsx` and save it under a new name (for example `results/E1_4x4/E1_4x4_charts.xlsx`), so the template stays empty.
3. On the `data` sheet, click cell A1 and paste. The workbook reads up to 2,000 rows.
4. On the `curves` sheet, type the pattern and, if the sweep has several, N, V and p_data in the yellow cells. The table fills from the data: latency and accepted throughput for each design at each rate. The two charts beside it are the latency curve and the throughput curve (as in [1] Fig. 11).
5. The `summary` sheet lists, per design and pattern, the saturation throughput, the latency at the lowest rate, the bypass share and the gain over design B.
6. The `trace` sheet takes a `trace/<run_id>.csv` the same way (paste at A1) and draws latency and k_L over time (E6).

The formulas average the rows that match, so two seeds of one point give their mean. A blank cell means no point of the sweep matches.
