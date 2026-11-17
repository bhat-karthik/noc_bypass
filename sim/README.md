# Running the simulations (F12)

Project: Design and Implementation of a Demand-Adaptive Buffer Reservation Scheduler for Router Bypassing in Mesh NoCs (MVE433P, RVCE).
The tracker doc is the single source of truth for decisions (D01–D60), files (F01–F76), tests and runs. This file says how to run things.

## 1. Layout

| Path | What | Tracker |
|---|---|---|
| `rtl/pkg/noc_pkg.sv` | Sizes, port numbers, flit/lookahead/cancel/credit types, coordinate and routing helpers | F01 |
| `rtl/common/rr_arbiter.sv` | Round-robin arbiter with mask and hold | F02 |
| `rtl/common/sync_fifo.sv` | FIFO with count; one per input VC | F03 |
| `rtl/router/route_compute.sv` | XY, minimal adaptive, and XY/YX for bypass | F04 |
| `rtl/router/input_unit.sv` | Input port: VCs of one packet each, route computed once at BW, SA→ST register, credits upstream; the read port through which the bypass source takes whole packets (B4) | F13 |
| `rtl/router/switch_allocator.sv` | VC and switch allocation in one cycle: separable input-first round robin, as Garnet | F14 |
| `rtl/router/crossbar.sv` | 5×5 crossbar into the ST registers; per-input bypass select, used by the bypass since B4 | F15 |
| `rtl/router/output_unit.sv` | Output VC state, credits per VC, LT register | F16 |
| `rtl/router/router.sv` | The 5-port router: three stages and a 1-cycle link, 4 cycles per hop; designs B–D add the bypass source and relay (B4) and the cancel relay (B5) | F17 |
| `rtl/router/bypass_source.sv` | Bypass sending side: chooses whole packets for the partner from any input VC (up to B_MAX flits), launches the lookahead, streams the burst through crossbar input 4; skips a turn its cancel token names (B5); built for area since B5 (D47) | F41 |
| `rtl/router/bypass_relay.sv` | Bypass passing and receiving side: lookahead in, route, switch-allocation masks, crossbar preset, lookahead out one hop ahead, delivery to the NI, and the lookahead of a burst for this router to the NI (B5) | F42 |
| `rtl/router/cancel_relay.sv` | Cancel tokens (B5, D12): forwards each token one hop per cycle over the burst's links in the other direction, launches its NI's token, hands a token for this router to its bypass source | F46 |
| `tb/common/tb_pkg.sv`, `tb_macros.svh` | Check counters, result line, coverage bins, CSV, seeded random streams | F05 |
| `tb/unit/tb_rr_arbiter.sv`, `tb_sync_fifo.sv`, `tb_route_compute.sv` | Unit tests U01–U03 | F06–F08 |
| `tb/unit/tb_switch_allocator.sv` | U04: the switch allocator against an exact reference model, VCs 1–4 | F18 |
| `tb/unit/tb_router.sv` | R01–R04: four routers (VCs 4/4/2/1, XY and adaptive) with source and sink models and a scoreboard | F19 |
| `rtl/ni/ni.sv` | Network interface: injects one packet at a time on the lowest idle Local VC; one ejection FIFO (25 flits in design A, 48 in B–D, D48), credit back when the core takes a regular flit; bypass flits keep their bypass bit and return no credit (B4); in design C a local subslot is decided with the local setup window and subslot length (B6) | F20 |
| `rtl/ni/resv_ctrl.sv` | Reservation controller (B5, D12, D48): one subslot ahead of every burst that may come, reserves B_MAX slots of the FIFO's bypass space or sends a cancel token; shrinks, releases and follows each reservation; the local gaps for design C's local subslots (B6, `dec_loc`) | F45 |
| `rtl/top/tile.sv` | One router and its NI: the unit placed N×N times; design B adds the timetable (slot counter, slot table, partner logic: sending now, deciding for the next subslot); design C (B6) has two timetables, its region's local one and the global one, under `phase_sequencer`; design D (B7) adds `demand_counter` and `ctrl_node`, and its sequencer takes the forwarded plan | F21 |
| `rtl/top/mesh_top.sv` | The N×N mesh: tiles, flit, credit, lookahead and cancel links, edge tie-offs; design C: the plan input, k_L, to every tile (B6); design D (B7): the controller's tree of 2-bit sum links and plan links, and `sched_ctrl` beside tile (0, 0), with the inputs `ctrl_pol_i` and `ctrl_damp_i`; since B8 the controller's clamp as parameters `K_LO` and `K_HI` (E7, E8) | F22 |
| `tb/common/traffic_pkg.sv` | The ten traffic patterns, packet mix, payload signatures, zero-load latency | F23 |
| `tb/common/tg_node.sv`, `sink_node.sv` | Per-node traffic source (Bernoulli or periodic) and sink (stalls, reassembly, every flit checked) | F24, F25 |
| `tb/common/scoreboard.sv`, `stats.sv` | Packets in flight (each delivered exactly once); the run's metrics and CSV row | F26, F27 |
| `tb/common/run_ctrl.sv` | Warm-up, window, drain; deadlock detection (no delivery for 2,000 cycles) | F28 |
| `tb/mesh/mesh_bench.sv`, `tb_mesh.sv`, `tb_mesh_dl.sv`, `tb_mesh_b.sv` | The mesh bench and its three tops: XY (N01–N04, N06–N08, PERF), adaptive with 1 VC (N05; BP08's design-A half), and design B (BP01–BP10); since B6 the bench also builds design C, with `mesh_checker`, `sched_monitor` and, for TL05, a design-B mesh beside it; since B7 design D, with `ctrl_checker` as well | F29 |
| `tb/mesh/tests_func.svh`, `tests_perf.svh` | The test tasks N01–N08 and PERF, included inside `mesh_bench` | F30, F31 |
| `tb/mesh/tests_bypass.svh` | The bypass tests BP01, BP02, BP04, BP05, included inside `mesh_bench` | F44 |
| `tb/mesh/tests_resv.svh` | The reservation tests BP03 and BP06–BP10, included inside `mesh_bench` (B5) | F69 |
| `tb/mesh/tb_mesh_c.sv`, `tb_mesh_cb.sv` | The design-C tops (B6): TL01–TL04 and TL06 on 4×4 with 2×2 regions (8×8 and 16×16 by define), and TL05: a design-B mesh beside design C, compared every cycle | F70, F72 |
| `tb/mesh/tests_two.svh` | The two-level timetable tests TL01–TL06, included inside `mesh_bench` (B6) | F71 |
| `tb/checkers/router_checker.sv` | The bypass checker: every burst of every subslot against the timetable (`sched_ref`) and the D38 cycles, on every link and NI; since B5 also every cancel token on every link; in design C it follows the frames: in a local subslot, every region's local timetable (B6) | F43 |
| `tb/checkers/ni_checker.sv` | An independent model of every NI's bypass space (B5): makes each reservation decision itself and compares the RTL's counts every cycle | F47 |
| `tb/checkers/mesh_checker.sv` | Design C (B6): every tile's sequencer in lockstep and equal to the frame reference, the frame rules from the RTL state alone, region containment on the links, a progress watchdog, and collisions from the lookaheads alone | F52 |
| `tb/common/sched_monitor.sv` | Design C (B6): every tile's turns and decisions against `sched_ref` every subslot; every pair once per local and global round; the interval between turns of every pair | F53 |
| `tb/unit/tb_resv_ctrl.sv` | U10: the reservation controller against a reference model in six timetable shapes | F48 |
| `tb/common/perf_probe.sv` | The RTL's bypass and reservation counters as extra CSV columns of designs B–D (B8: one CSV schema for every design, D58) | F49 |
| `rtl/pkg/sched_pkg.sv` | The BuffeRS timetable: groups (Eq. 1), rows (Eq. 2), slot numbers (Table II), partners, slot timing (SAFE, TIGHT), the cycle timing of one bypass (D38) and the NI FIFO depth of designs B–D (D48); design C's frames, K_FIXED, k_L and the epoch (B6); design D's controller constants, timing and policy tables (B7) | F32 |
| `rtl/sched/slot_table.sv` | A router's lookup table S0…S(n−1), loaded at reset, and its compare: active, and the other group of the slot | F33 |
| `rtl/sched/slot_counter.sv` | Clock to slot: setup window, subslot, cycle, slot number; an enable for frames; the pulse that decides cancel tokens one subslot ahead | F34 |
| `rtl/sched/group_to_router.sv` | The transmit and receive partner of a subslot, the mode (XY or YX) and the hops; region-relative in a local timetable | F35 |
| `rtl/sched/phase_sequencer.sv` | Design C's frames (B6; D16, D19, D52): two slot counters, the local one over REGION_N and the global one over MESH_N, each enabled only in its phase; k_L local slots, then one global slot; the epoch counter and the plan register, a new k_L applied at a frame start; one pointer when the region is the mesh (D50) | F50 |
| `tb/common/sched_ref.sv` | Reference model of the timetable, written independently of `sched_pkg`; since B6 also `frame_ref`, both timetables' positions in closed form from the cycle and the plan history; since B7 the controller's formulas, in real arithmetic | F36 |
| `tb/unit/tb_slot_table.sv`, `tb_slot_counter.sv`, `tb_group_to_router.sv` | U05–U07: the timetable RTL against the reference for every group, cycle, router, slot and subslot | F37–F39 |
| `tb/unit/tb_timetable_check.sv` | U08–U09: disjoint paths in every subslot; no collision in any cycle; the TIGHT minimum | F40 |
| `tb/unit/tb_phase_sequencer.sv` | U11: nine phase sequencers against `sched_ref`'s frame reference every cycle, and `sched_pkg`'s frame numbers | F51 |
| `rtl/ctrl/demand_counter.sv` | Design D's demand in each tile (B7; D17, D54): the router's occupied input VCs, read from its links, local when the packet's destination is in the tile's region; summed every 16th cycle into two saturating 14-bit sums, latched at each epoch tick | F54 |
| `rtl/ctrl/ctrl_node.sv` | Design D's controller network in each tile (B7, D55): adds its latched sums to its neighbours' one bit per cycle, on a fixed schedule from the tick, toward tile (0, 0); passes the plan on, one hop per cycle | F55 |
| `rtl/ctrl/sched_ctrl.sv` | Design D's controller (B7; D18, D56), beside tile (0, 0): share bin q = ⌊16 C_L / (C_L + C_G)⌋, the PROP or SQRT table, clamp, damping; the next k_L 42 cycles after each tick on 8×8 | F56 |
| `tb/unit/tb_demand_counter.sv` | U12: seven demand counters against a reference model every cycle (random links, saturation, resets) | F57 |
| `tb/unit/tb_sched_ctrl.sv` | U13: five controllers against `sched_ref`'s formulas, 10,000 epochs each, corners included; the 8×8 tables against the tracker's values | F58 |
| `tb/unit/tb_ctrl_chain.sv` | U14: 4×4, 8×8 and 16×16 networks of controller nodes with the controller: every node's sums, the plan, its arrival | F59 |
| `tb/mesh/tb_mesh_d.sv`, `tests_ctrl.svh` | The design-D top and the controller tests CT01–CT05 (B7), included inside `mesh_bench` | F74, F73 |
| `tb/checkers/ctrl_checker.sv` | Design D (B7): its own model of every tile's demand from the links; every epoch the latched sums, the mesh-wide sums, the plan and its arrival | F75 |
| `tb/mesh/tests_stress.svh` | The stress tests S01, S02, S03 and S05 (B8), included inside `mesh_bench`; S04 and S06 are a list and a sweep that run them and PERF | F76 |
| `sim/files_rtl.f`, `sim/files_tb.f` | Compile order for every simulator | F09 |
| `sim/eda/design.sv`, `design_part2.sv`, `testbench.sv`, `tb_part2.sv` to `tb_part5.sv`, `make_bundle.tcl` | EDA Playground bundles (comment lines left out since B5; since B6 one support module spans two tabs) and their generator | F10 |
| `sim/vivado/run.tcl` | Vivado xsim batch runner | F11 |
| `sim/vivado/regress.tcl`, `xsim_lib.tcl`, `sim/lists/*.list` | Regression lists in xsim, every count checked against the list; the procedures `regress.tcl` and `sweep.tcl` share (B8) | F61 |
| `sim/vivado/sweep.tcl`, `sim/sweeps/make_sweeps.tcl`, `sim/sweeps/*.sweep` | The experiments' PERF points (E0–E10, S06) run unattended, in parts side by side if wanted, into one CSV per experiment (B8) | F60 |
| `sim/compare_csv.tcl` | S06: two simulators' CSV rows compared column by column (B8) | F61 |
| `sim/vcs/Makefile`, `regress.sh`, `sweep.sh` | VCS and Verdi: any test by its ID, the lists, the sweeps (B8) | F62 |
| `sim/xcelium/run.sh` | Xcelium or Incisive: any test by its ID (B8) | F63 |
| `results/` | Your run logs (`R-MMDD-nn.log`); `results/precheck/` Claude's pre-delivery runs; a folder per sweep (B8); `README.md`: the CSV's columns (D58) and the layout; `templates/sweep_charts.xlsx`: the charts | F64 |

Run every command from the repository root (the folder that holds `rtl/`, `tb/` and `sim/`). The file lists use paths relative to it.

## 2. Conventions

RTL (decision D02)
- SystemVerilog-2012, synthesizable subset; no interfaces, no classes, no UVM.
- Names: `clk_i`, `rst_ni` (asynchronous, active low), inputs `_i`, outputs `_o`, flops `_q`.
- Ports are packed (packed structs, packed arrays), so every simulator and synthesis tool reads them the same way. One exception (D36): `mesh_top`'s per-node arrays are unpacked arrays of packed flits. A packed array of 64 flits is one 8,576-bit vector, and simulators rebuild all of it whenever one node's flit changes.
- Every file starts with `` `timescale 1ns/1ps ``.
- Simulation-only checks sit inside `` `ifndef SYNTHESIS `` and `translate_off`. They call `$fatal`, so a broken protocol (for example a push into a full FIFO) stops the run at the cycle it happens.
- Simulation-only hooks for the testbench (B4) sit inside `` `ifndef SYNTHESIS `` only: the router's test hold `tb_hold` (a directed test keeps packets waiting in chosen inputs), the bypass source's turn counters, the router's coverage counter of D38's rule, and (B5) the counters of `resv_ctrl` and `cancel_relay`. Synthesis never sees them; no normal run sets the hold.
- Lint: Verilator `-Wall` gives no warnings. The one waiver is `UNUSEDPARAM` in `noc_pkg` and `sched_pkg`, because a package of constants always has some that a given block does not use.
- `slot_table` loads its table while `rst_ni` is low (a synchronous load from the router's group, a tie-off), so reset must last at least one clock edge. Like buffer storage, those entries have no asynchronous reset.

Testbenches
- Every check goes through `` `TB_CHECK `` or `` `TB_CHECK_EQ ``. These count the check, compare four-state (an X never passes), and build the message only when the check fails.
- Every test ends with exactly one result line:
  `RESULT U01 PASS checks=310493 errors=0 seed=1`
  A run with zero checks fails. Each coverage bin is also a check: a bin that is never hit fails the test.
- The first line of each testbench lists the tests it runs, for example `// @tests U01`. `run.tcl` and `make_bundle.tcl` use these lines to find tests.
- Randomness comes from `tb_pkg` streams (splitmix64), not `$urandom`. With the same seed, Riviera, xsim, VCS, Xcelium and Verilator get the same stimulus and print the same `checks=` count. A failure seen on one simulator therefore replays on another.
- Plusargs: `+SEED=<n>`, `+MAXERR=<n>` (errors printed in full; the default is 20), `+CSV=<file>`, `+VCD`, and `+TEST=<id>`. `tb_router` holds four tests (R01–R04) and runs them all; `+TEST=R02` runs only R02.
- A testbench may use classes. `tb_timetable_check` does: Verilator inlines every call of a module function, and its checker with some 30 calls became a 17 MB C++ file; as class methods each is compiled once.
- Ten rules (D30, D36, D41, D45, D53) keep every simulator in step. Verilator 5.020 broke or hid each of the first five in B1, the next two in B2 and the eighth in B3; xsim 2024.2 broke the ninth in B4, B5 and B6; the tenth came from B6:
  - Read an associative array only inside an `if` that has already checked `exists()`. Never read it on the right of `&&`: Verilator does not stop at a false left side, and any read of a missing key creates that key. The same holds for a random draw: never put `chance()`, `rnd()` or a `tb_pkg::rng_*` call on the right of `&&` or `||`; draw into a variable inside an `if`. Verilator 5.020 drew there even after a false left side, so until 14 Nov U12's stimulus on Verilator differed from xsim's and Riviera's (its count did not: U12 checks every cycle whatever the stimulus).
  - Never pop a queue into a variable that is not read afterwards. Verilator drops the whole statement, pop included. Use `void'(q.pop_front())` when the value is not needed.
  - Give an array of queues 2^k entries and index it with exactly k bits. For any other size, Verilator's guarded read does not compile.
  - Name the top module in every flow. With two top modules (say a testbench and the unused router), Verilator gives every instance of a module the derived parameters of its first instance.
  - Index arrays only with unsigned values. A size cast keeps the signedness of what it casts (IEEE 1800-2017 §6.24.1), so `3'(sp)` with `int sp = 4` is −4: Riviera and xsim drop that access, while Verilator uses the bits and indexes element 4. Use a type cast to an unsigned type (`pslot_t'(sp)`) or a helper that returns `logic [k-1:0]`. The pre-check greps for size casts inside `[ ]`.
  - No recursive functions. Verilator 5.020 stops with an internal error (`No clone for package function`) on a package function that calls itself.
  - Choose a message string with `if`, never with `?:` between two string literals of different lengths. The literals are bit vectors, so the shorter one is padded with zero bytes, and each simulator prints those differently (Verilator prints spaces). The logs of two simulators would then differ.
  - Never call a function in a loop condition: compute the bound into a variable before the loop. Verilator 5.020 moves the increments of the called function's own loops out of those loops, which then never end (U05 hung this way while B3 was built).
  - Never cast a system function's result inside an expression, as in `longint'($countones(v))`. xsim 2024.2 left garbage in the upper bits of BP05's turn statistics (R-1109-01, 06, 07), while Verilator and Riviera counted right. Count with a loop instead; the pre-check scans for such casts. In B5 xsim also misread a cast used as an index inside a larger expression (`d.t_tail >= tt[int'(d.src)] + ...` read element 0) and a cast of a negation passed as an argument (`int'(!q[m].mode)` gave 0) (R-1110-02, 05). So in testbenches, cast into a variable first (`si = int'(d.src);`) and use the variable. In B6 the same misreading hung U11 on xsim for 36 hours (R-1111-01): `sched_pkg::epoch_len`, which elaborates correctly as a constant function, is also called at run time by U11, and there xsim misread `longint'(n_slots(n)) * frame_len(...)`, so its `while (e < r)` loop never ended. The rule therefore holds in RTL packages too, and every loop in a function has a fixed bound (`for (int i = 0; i < 40; i++) if (e < r) e = e * 2;`). xsim also reads an empty string parameter (`parameter string TAG = ""`) as one blank, so U11's INFO and COVER lines printed a blank more than Verilator's (R-1113-02-01): use no empty string parameters.
  - Never change, in one process, a signal that another process samples on the same clock edge, unless the change is a nonblocking assignment; and never call `$finish` on an edge where checkers run. Simulators order such processes differently. In B6 TL05 counted one check more on Verilator than on xsim and Riviera: the bench released reset with a blocking assignment at a falling edge, where the `always @(negedge clk)` checkers test `rst_n`, and Verilator ran the bench first. TL04 counted 71 more on Riviera: `$finish` came at a falling edge, and Riviera ran that edge's checkers first. The TL05 comparison now starts from a flag set at the rising edge (`sh_on <= rst_n`), and the design-C bench finishes on a rising edge; Verilator, xsim and Riviera give the same counts again (R-1111-03, R-1113-TL04).
- One more rule since B6, for every flow: a `` `define `` made in one testbench file stays defined in every file compiled after it (run.tcl, xvlog and Verilator compile all the files in one go). So every mesh top gives `TB_MESH_N` the same default, 4, and the design-C tops give `TB_REGION_N` theirs only when it is not set. A top that defaulted to 8 would silently turn every later top into an 8×8 mesh, or the other way round.

## 3. EDA Playground (Aldec Riviera-PRO)

EDA Playground's limits ([FAQ](https://eda-playground.readthedocs.io/en/latest/faq.html), [Settings](https://eda-playground.readthedocs.io/en/latest/settings.html)): under 100,000 characters per file, 1,000,000 per playground, 60 seconds and 100 MB per run. Added SystemVerilog files are compiled only through `` `include ``. Its Riviera-PRO is the EDU edition, which runs slower than the full one. The bundle is seven files since B7 (570,824 characters in all since B8). Since B5 the bundles leave out comment lines (pragmas stay): with them, the mesh bench and its four test files were over one file's limit. Read the comments in the repository files.

| File | Pane | What it holds |
|---|---|---|
| `sim/eda/design.sv` | right (design.sv) | the RTL up to `bypass_source`; its last line `` `include ``s design_part2.sv |
| `sim/eda/design_part2.sv` | right, a new tab named exactly `design_part2.sv` | `bypass_relay`, `resv_ctrl` and `cancel_relay` (B4–B5), `phase_sequencer` (B6) |
| `sim/eda/testbench.sv` | left (testbench.sv) | test selection, packages (with `sched_ref`, `perf_probe`, `ni_checker`, `router_checker`, `sched_monitor` and `mesh_checker`), U01; its last lines `` `include `` the three parts |
| `sim/eda/tb_part2.sv` | left, a new tab named exactly `tb_part2.sv` | U02–U04, R01–R04 and the mesh support modules (`tg_node`, `sink_node`, `run_ctrl`) |
| `sim/eda/tb_part3.sv` | left, a new tab named exactly `tb_part3.sv` | the mesh bench with its first four test files (N, PERF, BP tests) |
| `sim/eda/tb_part4.sv` | left, a new tab named exactly `tb_part4.sv` | the rest of the mesh bench (`tests_resv.svh`, `tests_two.svh`, `tests_ctrl.svh` and `tests_stress.svh`: the reservation, two-level, controller and stress tests), `tb_mesh`, `tb_mesh_dl`, U05–U07 |
| `sim/eda/tb_part5.sv` | left, a new tab named exactly `tb_part5.sv` | U08, U09, `tb_mesh_b`, U10, U11, `tb_mesh_c`, `tb_mesh_cb`, U12–U14 and `tb_mesh_d` (B7) |

Since B6 the mesh bench with its test files (six since B8) is longer than one tab can hold, so `make_bundle.tcl` splits it at a line between two of its parts: the first piece ends `tb_part3.sv`, the rest opens `tb_part4.sv`, both inside the same `` `ifdef ``. Paste every tab whole and the module compiles as one.

1. Regenerate the bundles if you changed any source file: `tclsh sim/eda/make_bundle.tcl` (or `source` it in the Vivado Tcl console). It prints the files to paste. A later batch may need more parts (`tb_part6.sv`, `design_part3.sv`); add each the same way. The bundles in the zip are already up to date.
2. On edaplayground.com, log in and set Languages to SystemVerilog/Verilog and Tools & Simulators to Aldec Riviera Pro (the newest version listed). Do not use Icarus: it cannot write package variables, which the check counters need.
3. Paste `sim/eda/design.sv` into the right pane and `sim/eda/testbench.sv` into the left pane. In the right pane, click **+** next to the last tab, create a file named exactly `design_part2.sv` and paste `sim/eda/design_part2.sv` into it. In the left pane do the same for `tb_part2.sv` to `tb_part5.sv`. Paste whole files; replace all seven after every new batch. Save the playground once: the tabs stay for every later run.
4. Choose the test:
   - Compile Options: `+define+TEST_U02`. Leave it empty for U01. The bundle holds U01–U11, R01–R04, N01–N08, PERF, BP01–BP10 and TL01–TL06; `+define+TEST_R03` runs R03 alone. `+define+TEST_BP08` builds `tb_mesh_b` with one VC (BP08 needs it); BP08's design-A half is `+define+TEST_N05` with Run Options `+TEST=BP08`. A define has no effect in Run Options: by then the code is already compiled. Check the log's first lines: the `vlog` command must show your define, and Riviera prints `Unit top modules: tb_...` for the testbench it compiled.
   - Alternatively, edit the `` `define TEST_U01 `` line near the top of testbench.sv.
   - B6: `+define+TEST_U11` builds the phase-sequencer test; `+define+TEST_TL01` to `TEST_TL04` and `TEST_TL06` build the design-C mesh on 4×4 with 2×2 regions and run that test; `+define+TEST_TL05` builds `tb_mesh_cb`. `TEST_TL04` also shortens the epoch to 512 cycles, so that its four plan switches fit in 60 seconds (its count differs from Vivado's, Section 4).
   - B7: `+define+TEST_U12 -dbg`, `+define+TEST_U13 -dbg` and `+define+TEST_U14` build the controller's unit tests. Keep `-dbg` (Riviera's debug compile, which turns its optimizer down) for U12 and U13: without it Riviera-PRO 2025.04 EDU computed wrong values in their testbenches while the DUTs were right (R-1113-U12, R-1113-U13; Section 9). With it both pass with the counts of Section 4 and the same stimulus as xsim and Verilator (R-1114-U12dbg, R-1114-U13dbg). The design-D mesh tests CT01–CT05 do not fit in EDA Playground: CT01's short form, 3,629 cycles of the 4×4 design-D mesh, was stopped at 60 s (R-1113-CT01), and CT03–CT05 need at least 7 epochs. Run them in Vivado (Section 4). `+define+TEST_CT01` to `TEST_CT05` still build `tb_mesh_d` with epochs of 512 cycles, for a simulator without that limit.
   - B8: the stress tests and PERF run on any mesh top through Run Options: `+define+TEST_N01` (design A), `+define+TEST_BP01` (B) or `+define+TEST_TL01` (C, 2×2 regions) with Run Options such as `+TEST=S05 +QUICK`. Design D's tops in the bundle use 512-cycle epochs (`TEST_CTxx`), so run its stress tests in Vivado. S06's points take `+TEST=PERF` and their plusargs (Section 15).
   - Mesh tests, U06–U14, BP05, BP08, BP10 and TL01–TL06: add `+QUICK` to Run Options, the short forms in Section 4's tables, so each run fits in 60 s. The mesh tests run on 4×4; the B3 short forms leave out the longest 16×16 parts. The full forms belong in Vivado. BP01–BP04, BP06, BP07 and BP09 are short; `+QUICK` changes nothing in them.
   - Run Options: `+SEED=1` (or empty; the seed is 1 by default).
5. Click Run. The end of the log should show `RESULT ... PASS checks=...` with the count from Section 4. The design-C tops give one or two compile warnings `VCP2910 Conditional expression 4<=4 at ?: operator is constant` (their size defaults); they are expected.
6. Waveforms: tick *Open EPWave after run* and add `+VCD` to Run Options. Every test makes a large dump, so use this only to debug a failing seed.
7. Save the playground. Copy the log into `results/R-MMDD-nn.log`, or paste the playground link into the run log.

Riviera may report modules a test does not use (for example `rr_arbiter` or `mesh_top` during U02) as extra top-level units. Ignore that message. The testbench side compiles only the chosen test's testbench and the modules it uses, so it adds no extra tops.

## 4. Vivado simulator (xsim)

### Batch, no project (recommended)

Use ONE of these two ways. Do not type `vivado ...` at the Tcl prompt of a running Vivado: Vivado hands it to Windows and starts a second copy of itself.

**A. Inside Vivado** (GUI Tcl console, or the *Vivado 2024.2 Tcl Shell* from the Start menu):

```
cd D:/noc_bypass
set argv {-test all}
source -notrace sim/vivado/run.tcl
```

To run one test with another seed, set `argv` first, for example `set argv {-test U02 -seed 7}`. Then source the script again. Add `-wave` to record waveforms. `-test R02` runs `tb_router` with `+TEST=R02`; `-test tb_router` or `-test all` runs R01–R04 together.

Mesh tests (B2): `-test N03` runs one test on 4×4; `-test tb_mesh` runs N01–N04 and N06–N08 in one simulation; `-test N05` runs `tb_mesh_dl`. Bypass tests (B4–B5): `-test BP01` runs one test on the design-B 4×4 mesh; `-test tb_mesh_b` runs BP01–BP07, BP09 and BP10 in one simulation; `-test BP02 -define TB_MESH_N=8` runs BP02 on 8×8; `-test U10` runs the reservation controller's unit test. BP08 needs one VC: `-test BP08 -define TB_NUM_VC=1` (design B), and `-test tb_mesh_dl -plus {+TEST=BP08}` for its design-A half. Add `-define TB_MESH_N=8` for 8×8 (or `=2` for 2×2). Each set of defines compiles into its own folder, so 4×4 and 8×8 logs do not overwrite each other: `work/tb_mesh/` and `work/tb_mesh__TB_MESH_N_8/`. Long tests take plusargs too: `set argv {-test N08 -plus "+N08_CYCLES=200000"}`. Section 10 of this file says what each test does.

**B. From a Windows command prompt** (run Vivado's `settings64.bat` first, e.g. `D:\Xilinx\Vivado\2024.2\settings64.bat`):

```
cd /d D:\noc_bypass
vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/run.tcl -tclargs -test all
```

`-notrace` stops Vivado from echoing every line of the script. `-nojournal -nolog` stops it leaving `vivado.jou` and `vivado.log` in the folder.

The script compiles everything in both file lists, elaborates the testbench and runs it. It prints any warning lines, then a summary:

```
tb_rr_arbiter        U01          PASS  checks=310493 errors=0   (35 s; warnings: compile 0, elaborate 0)
tb_router            R01 R02 R03 R04 PASS  checks=357967 errors=0   (.. s; warnings: compile 0, elaborate 0)
tb_mesh              N01 N02 N03 N04 N06 N07 N08 PERF PASS  checks=46273022 errors=0, 1231461 cycles in ... s = ... cycles/s   (...)
```

For mesh tests the summary also gives the simulation speed: the cycles the testbench simulated (its `INFO   cycles simulated` line) over the seconds xsim took. That number is M24, the input to the compute budget in the tracker (Section 9). Read it from a long run (N03, N08 or the whole of `tb_mesh`): in a short one, start-up time dominates. Please note it in the run log.

Logs go to `sim/vivado/work/<testbench>/` (`compile.log`, `elaborate.log`, `simulate.log`). With `-wave`, open `work/<testbench>/<testbench>.wdb` through *File > Simulation Waveform > Open*. The `-gui` option opens the xsim GUI on the test instead of running it. Keep the repository on a path without spaces.

On Windows, `xvlog`, `xelab` and `xsim` are batch files, and a batch file splits its arguments at `=`, `,` and `;`. The script therefore passes arguments such as `--testplusarg SEED=1` through an options file (`simulate.args`, read with `--file`). If you call xsim by hand on Windows, quote such arguments: `xsim tb_snap --testplusarg "SEED=1" --runall`.

### Regression lists and sweeps (B8)

`regress.tcl` runs a list of tests and checks every count against the list; `sweep.tcl` runs an experiment's performance points and collects their CSV rows. Both build each testbench once per set of defines, in the same `work/` folders as run.tcl, and reuse a build until a source file changes.

Inside Vivado (from the repository):

```
cd D:/noc_bypass
set argv {-list b7}; source -notrace sim/vivado/regress.tcl
set argv {-sweep E1_4x4}; source -notrace sim/vivado/sweep.tcl
```

From a Windows command prompt, after `settings64.bat` (the way to run overnight):

```
vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/regress.tcl -tclargs -list stress
vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/sweep.tcl -tclargs -sweep E1_4x4
```

**Lists** (`sim/lists/<name>.list`): one entry per line, `<label> <test ID or testbench> <checks expected at seed 1, or -> [-define N=V]... [-plus +ARG]...`. `regress.tcl` prints one line per entry with its verdict, then a summary, and writes the same lines to `sim/vivado/work/regress_<lists>_seed<n>.txt`. Verdicts: PASS (the PASS line and the expected count), COUNT (a PASS line with another count: the simulators disagree, so report it), FAIL, NONE (no RESULT line: a crash, a timeout or a `$fatal`), BUILD (compile or elaboration failed). Options: `-list <name>` (repeat it for several lists), `-only <glob>` (entries whose label matches, for example `-only CT*`), `-seed <n>` (other seeds report counts only), `-rebuild`, `-stop` (stop at the first entry that does not pass).

| List | Entries | What |
|---|---|---|
| `b0`, `b1`, `b2`, `b3`, `b5`, `b6`, `b7` | 3–13 | Each batch's tests on 4×4, full forms (b5 holds B4 and B5) |
| `all` | 22 | Every batch's main 4×4 run: the counts of this section, about two hours on xsim |
| `quick` | 22 | Every batch's short forms (`+QUICK`): a smoke regression, about half an hour on xsim, most of it building |
| `exit8` | 14 | The 8×8 runs of the batch exit checks: hours on xsim, run overnight |
| `stress` | 14 | S01–S03 and S05 in their short forms on every design they apply to (Section 15) |
| `stress_long` | 14 | The same in full: S01 is 10,000,000 cycles per design |
| `s04` | 16 | S04: sixteen configurations of designs B–D, each running S01 for 30,000 cycles; the verdict is PASS (the counts depend on the configuration) |

Two Vivado processes may run side by side when their entries build different testbenches or defines, for example the four S01 entries of `stress_long`: `-list stress_long -only S01_A*` in one window, `-only S01_B*` in the next, and so on. The summary file is then the last one's.

**Sweeps** (`sim/sweeps/<name>.sweep`): one PERF point per line, `<run_id> <testbench> -seed <n> [-define N=V]... -plus +TEST=PERF [-plus +ARG]...`. `tclsh sim/sweeps/make_sweeps.tcl` writes them from the experiment plan (tracker Section 9); edit its settings and run it again to change a sweep. `sweep.tcl` writes `results/<name>/<name>.csv` (one header, one row per point; `results/README.md` describes every column), `logs/<run_id>.log`, `sweep.txt` (one line per point: verdict, checks, cycles, seconds) and, for a point with `+TRACE`, `trace/<run_id>.csv`. A point is done when its `run_id` is in the CSV, so a sweep stopped at any time (close the window, or Ctrl+C) continues where it stopped when started again; a point that fails writes no row, is listed in `sweep.txt` and is tried again on the next start. Options: `-sweep <name>`, `-out <dir>`, `-max <n>` (at most n points this time), `-only <glob>` (only the points whose run_id matches, for example `-only *_uniform_*` or `-only *_D_*`), `-dry` (list what would run), `-rebuild`, `-part <k>/<n>`, `-merge`.

xsim uses one core. To use more, start n processes on one sweep with `-part 1/n` … `-part n/n`, each in its own command prompt: part k runs the points whose place in the sweep, counted from 0, leaves k−1 when divided by n, builds in its own folders (`work/<testbench>…__part<k>`), writes `<name>.part<k>.csv` and `sweep.part<k>.txt`, and merges every part's rows into `<name>.csv` in the sweep's order when it ends (`-merge` does only that). With `-only`, give every part the same glob: the parts then split the matching points. Start one part first and look at its memory in Task Manager once it simulates; then start as many as the cores and memory allow.

| Sweep | Points | Mesh | Designs | What (tracker Section 9) |
|---|---|---|---|---|
| `E0_4x4` | 480 | 4×4 | A, B | Does our BuffeRS reproduce [1]? Absolute rates 0.02–0.52, V 1 and 4, p_data 0 and 1/3, B_MAX 5, 10, 15 |
| `E1_4x4`, `E1_8x8` | 480, 234 | 4×4, 8×8 | A–D | Does two-level scheduling beat BuffeRS? Six patterns, ten fractions of each pattern's knee (0.1–1.2; design D on 8×8 nine, up to 0.95); two seeds on 4×4 |
| `E2_4x4_8x8`, `E2_16x16` | 90, 45 | up to 16×16 | B–D | Does the gain grow with mesh size? 0.2–1.2 of the knee (16×16 on VCS: Section 5; its knees are estimates) |
| `E3` | 144 | 4×4, 8×8 | A, B, D | How do results depend on the number of VCs (1, 2, 4)? 0.3–1.1 of the knee at that VC count |
| `E4` | 57 | 8×8 | B, D | Does the packet mix change the picture? E1's fractions of each mix's knee |
| `E5` | 30 | 8×8 | B–D | Where do regions pay off? Region-local share 0–0.9 |
| `E6` | 12 | 8×8 | B–D | Does the controller follow changing traffic? Phase traffic with `+TRACE` |
| `E7` | 60 | 8×8 | C, D | What do regions and adaptivity each contribute? PROP, SQRT, no damping (no K_MIN 0: D62) |
| `E8` | 60 | 8×8 | D | How sensitive are the chosen parameters? Epoch, K_MAX, region size, B_MAX |
| `E9` | 12 | 8×8 | B–D | Do the guarantees hold in practice? Hotspot at 0.5 and 0.9 of its knee, 200,000-cycle windows, two seeds |
| `E10` | 32 | 4×4 | B, D | How robust is the reservation? Sink stalls 0–20 %, 1 and 4 VCs, 0.5 and 0.9 of each knee |
| `S06` | 16 | 4×4 | A–D | Do the simulators agree? (Section 15) |

**Rates (D61).** A sweep's rates are fractions of each configuration's knee: the offered load, in packets/node/cycle, at which design B's accepted throughput falls below 95 % of the offered (the CSV's `sat`). B8 measured it for every pattern, mesh, VC count, packet mix and sink stall chance the sweeps use (a grid of rates, then four points inside the bracket; seed 1; the rows are the same on every simulator, Section 15): `make_sweeps.tcl` holds the table, `results/precheck/B8_knees.csv` the knees and `B8_knee_points.csv` every point. The run ids carry the fraction as `k<f>`. Two examples of why one scale for every pattern was wrong: hotspot (one node takes 20 % of the packets) saturates at 0.0303 packets/node/cycle on 8×8, less than a quarter of uniform's 0.1332, while transpose with adaptive routing goes on past uniform's knee. E0 keeps [1]'s absolute rates and S06 its B8 shares of `sat_rate()`.

**Past the knee on 8×8.** Designs B–D route minimally adaptively on every VC, and only the bypass timetable breaks a cycle of waiting packets (D06, BP08), when the waiting pair's turn comes: in design B about every 800 cycles on 4×4 and every 10,500 on 8×8 (in C and D, across regions, 1,250–1,480 and 13,000–22,000). On 8×8 a mesh past its knee therefore jams: under uniform traffic design B accepts 0.305 flits/node/cycle at 0.133 packets/node/cycle and 0.056 at 0.137, design C collapses at 0.133 and design D at 0.129 (0.97 of B's knee), while design A, and design B with XY routing (`TB_XY`), stay at 0.32 past their knee. A point past the collapse drains slowly; design D, which warms up for five epochs, would not drain within `+DRAIN`, so design D's 8×8 fractions stop at 0.95. On 4×4 no design collapses.

Designs C and D on 8×8 and 16×16 run with D19's epoch (`-define TB_EPOCH=32768`, `262144` on 16×16; D60), and design D warms up for five epochs before its window opens, so that k_L has settled. An 8×8 design-D point is therefore about 176,000 cycles: at the 8×8 speeds of earlier xsim runs (90–140 cycles/s) that is 20–35 minutes, against a few minutes for designs A–C. The sweeps hold about 240 such points (E1_8x8, E2, E3, E4, E5, E7, E8, E9), so plan the 8×8 experiments as several nights with parts side by side, and start with the 4×4 sweeps. The tests' 8×8 default epoch, 4,096 cycles, is the pre-check's.

### GUI project (if you prefer clicking)

1. Create an RTL project with no sources. Add every file listed in `sim/files_rtl.f` as a design source, in that order (B2 adds `rtl/ni/ni.sv`, `rtl/top/tile.sv`, `rtl/top/mesh_top.sv`; B3 adds `rtl/pkg/sched_pkg.sv` right after `noc_pkg.sv`, and the three files in `rtl/sched/`; B4 adds `rtl/router/bypass_source.sv` and `rtl/router/bypass_relay.sv`; B5 adds `rtl/ni/resv_ctrl.sv` and `rtl/router/cancel_relay.sv`; B6 adds `rtl/sched/phase_sequencer.sv` after the other three in `rtl/sched/`).
2. Add every file listed in `sim/files_tb.f`, plus `tb/common/tb_macros.svh`, `tb/mesh/tests_func.svh`, `tb/mesh/tests_perf.svh`, `tb/mesh/tests_bypass.svh`, `tb/mesh/tests_resv.svh` and `tb/mesh/tests_two.svh`, as simulation sources. Keep the packages (`tb_pkg`, `traffic_pkg`, `scoreboard`, `stats`, `sched_ref`, `perf_probe`, `ni_checker`, `router_checker`, `sched_monitor`, `mesh_checker`) above the modules in the compile order.
3. Set the file type of every `.sv` file to SystemVerilog.
4. Settings > General > Verilog options (the `...` button) > Verilog Include Files Search Paths: add `tb/common`, `tb/mesh` and `tb/checkers`.
5. In the simulation sources, right-click the testbench (for example `tb_sync_fifo`, or `tb_mesh` for N01–N08) and choose *Set as Top*.
6. Settings > Simulation > Simulation tab: set `xsim.simulate.runtime` to `all`. Set `xsim.simulate.xsim.more_options` to `-testplusarg SEED=1` (add `-testplusarg TEST=N02` for one mesh test). For 8×8, set `xsim.compile.xvlog.more_options` to `-d TB_MESH_N=8`.
7. Run Simulation > Run Behavioral Simulation. The result line appears in the Tcl console.

The batch script above is simpler for the mesh tests: it handles the file order, the include folders and the defines for you.

### Expected results at seed 1

Every simulator must print these counts. A different count with the same seed means the simulators disagree; report it.

| Test | Testbench | How | `checks=` at seed 1 |
|---|---|---|---|
| U01 | `tb_rr_arbiter` | `-test U01`, `+define+TEST_U01` | 310,493 |
| U02 | `tb_sync_fifo` | `-test U02`, `+define+TEST_U02` | 196,864 |
| U03 | `tb_route_compute` | `-test U03`, `+define+TEST_U03` | 856,450 |
| U04 | `tb_switch_allocator` | `-test U04`, `+define+TEST_U04` | 1,121,244 |
| R01 | `tb_router` | `-test R01`, `+define+TEST_R01` | 28,058 |
| R02 | `tb_router` | `-test R02`, `+define+TEST_R02` | 282,300 |
| R03 | `tb_router` | `-test R03`, `+define+TEST_R03` | 35,513 |
| R04 | `tb_router` | `-test R04`, `+define+TEST_R04` | 12,108 |
| R01–R04 | `tb_router` | `-test all` or `-test tb_router` | 357,967 |

The R01–R04 total is 12 below the sum of the four single runs: each run ends with one check per router that every packet returned its VC-free credit, which is 4 checks per run.

Mesh tests (B2), seed 1, one test per run (`-test N01`, `+define+TEST_N01`). The 8×8 and 2×2 columns need `-define TB_MESH_N=8` (or `=2`). Times are Verilator 5.020 on Claude's machine; expect xsim to take longer, and please record its time.

| Test | 2×2 | 4×4 | 8×8 | Cycles 4×4 / 8×8 | Verilator time 4×4 / 8×8 |
|---|---|---|---|---|---|
| N01 | 374 | 4,526 | 79,262 | 95 / 563 | 0.0 s / 0.5 s |
| N02 | 13,224 | 13,227 | 14,808 | 37,569 / 10,602 | 4 s / 9 s |
| N03 | 8,321,759 | 6,436,890 | 3,438,402 | 176,672 / 59,643 | 24 s / 49 s |
| N04 | 513,732 | 396,571 | 208,864 | 6,897 / 1,983 | 1 s / 2 s |
| N05 (`tb_mesh_dl`) | — | 1,949 | 1,413 | 4,153 / 4,097 | under 1 s |
| N06 | 31,867 | 150,814 | 1,226,930 | 3,950 / 10,706 | 1 s / 9 s |
| N07 | 251,491 | 190,766 | 102,986 | 5,637 / 1,996 | 1 s / 2 s |
| N08 | 12,618,509 | 39,069,420 | 83,286,723 | 1,000,039 / 1,000,076 | 137 s / 832 s |
| N01–N08 in one run (`-test tb_mesh`, no N05) | — | 46,273,022 | 88,332,937 | 1,231,461 / 1,085,257 | about 165 s / 16 min |

Short forms for EDA Playground: 4×4, Run Options `+QUICK`, each under 5,000 cycles. They shorten N02 to 2 periods, N03 to 1 repetition of 100 cycles, N04 to 250 + 500 cycles, N06 to steps of 300 cycles, N07 to 300 cycles and N08 to 2,000 cycles; the checks are the same.

| Test (`+define+TEST_...`) | Run Options | `checks=` at seed 1 | Cycles |
|---|---|---|---|
| N01 | `+QUICK` | 4,526 | 95 |
| N02 | `+QUICK` | 1,083 | 3,060 |
| N03 | `+QUICK` | 124,518 | 4,629 |
| N04 | `+QUICK` | 97,606 | 1,829 |
| N05 | (none) | 1,949 | 4,153 |
| N06 | `+QUICK` | 96,512 | 2,052 |
| N07 | `+QUICK` | 37,431 | 1,200 |
| N08 | `+QUICK` | 76,873 | 2,045 |

The older short forms still work and give the counts listed before: `+N03_REPS=1` (1,291,501, about 35,000 cycles) and `+N08_CYCLES=50000` (1,957,628). They are probably too long for 60 s on Riviera EDU.

Timetable tests (B3), seed 1. U05, U07, U08 and U09 draw no random numbers, so the seed changes nothing; U06's enable pattern follows the seed. Section 11 says what each test does.

| Test | Testbench | How | `checks=` | with `+QUICK` | Verilator time |
|---|---|---|---|---|---|
| U05 | `tb_slot_table` | `-test U05`, `+define+TEST_U05` | 24,462 | 24,462 | under 1 s |
| U06 | `tb_slot_counter` | `-test U06`, `+define+TEST_U06` | 837,406 | 99,202 | 1 s |
| U07 | `tb_group_to_router` | `-test U07`, `+define+TEST_U07` | 2,269,472 | 423,040 | 1 s |
| U08 | `tb_timetable_check` | `-test U08`, `+define+TEST_U08` | 4,786,435 | 756,480 | 1 s |
| U09 | `tb_timetable_check` | `-test U09`, `+define+TEST_U09` | 129 | 87 | 31 s |
| U08–U09 in one run | `tb_timetable_check` | `-test tb_timetable_check` | 4,786,564 | 756,567 | 32 s |

U09 holds few checks, each a whole round or a whole set of slot boundaries with no collision. Expect xsim to take about nine times as long as Verilator, so about 5 minutes for U09. `+U09_ALL` checks more (every subslot length from TIGHT to SAFE on 16×16, and 2×2 and 8×8 regions): U09 then has 198 checks and takes about 90 s on Verilator.

Bypass and reservation tests (B4–B5), seed 1, on the design-B mesh `tb_mesh_b`, one test per run (`-test BP01`, `+define+TEST_BP01`). Since B5 every test also checks every NI's reservations every cycle (`ni_checker`), so the B4 counts grew. BP01–BP04, BP06, BP07 and BP09 draw no random numbers, so neither the seed nor `+QUICK` changes them. BP05, BP08 and BP10 follow the seed; `+QUICK` shortens BP05 (4,000 to 1,000 cycles), BP08 (3,000 to 1,000 cycles of load) and BP10 (10,000 to 1,500 measured cycles). The 2×2 and 8×8 columns need `-define TB_MESH_N=2` (or `=8`). Section 12 says what each test does.

| Test | 2×2 | 4×4 | 4×4 `+QUICK` | 8×8 | Cycles 4×4 | Verilator time 4×4 |
|---|---|---|---|---|---|---|
| BP01 | 1,160 | 34,722 | 34,722 | — | 707 | under 1 s |
| BP02 | 2,030 | 42,758 | 42,758 | 1,672,182 | 854 | under 1 s |
| BP03 | 4,842 | 103,930 | 103,930 | 4,601,512 | 1,623 | under 1 s |
| BP04 | 2,990 | 47,496 | 47,496 | — | 953 | under 1 s |
| BP05 | 710,914 | 599,829 | 152,566 | — | 5,068 | 2 s |
| BP06 | 4,673 | 88,864 | 88,864 | — | 1,781 | under 1 s |
| BP07 | 14,631 | 239,570 | 239,570 | — | 4,839 | 2 s |
| BP09 | 4,262 | 85,066 | 85,066 | — | 1,623 | under 1 s |
| BP10 | 387,403 | 1,326,332 | 199,316 | — | 12,048 | 4 s |
| BP01–BP10 in one run (`-test tb_mesh_b`; BP08 not included) | 1,137,611 | 2,573,286 | 1,026,210 | 36,516,484 | 29,475 | 9 s |

The one-run totals are not sums of the single runs: each test then starts at another point of the timetable, and the random traffic starts from another state. With `+QUICK` the one run gives 1,026,210 checks on 4×4 (16,113 cycles) and 139,139 on 2×2. On 8×8 BP03 sends one packet for each of the 4,032 ordered pairs in one round of 10,332 cycles.

BP08 and U10 (`+define+TEST_BP08` builds `tb_mesh_b` with one VC; BP08's design-A half is `+define+TEST_N05` with Run Options `+TEST=BP08`):

| Test | How (Vivado) | `checks=` | with `+QUICK` | Cycles | Verilator time |
|---|---|---|---|---|---|
| BP08, design B | `-test BP08 -define TB_NUM_VC=1` | 1,173,542 | 379,431 | 17,937 | 2 s |
| BP08, design A | `-test tb_mesh_dl -plus {+TEST=BP08}` | 9,551 | 9,551 | 2,616 | under 1 s |
| U10 | `-test U10` (`tb_resv_ctrl`) | 869,821 | 289,930 | 182,615 (the 16×16 tester) | 4 s |

Other design-B configurations, the one run on 4×4 (defines as in Section 12): 1 VC 1,789,461, 2 VCs 2,029,175, B_MAX 5 2,454,566, B_MAX 15 2,733,460, SAFE subslots 2,772,277, XY regular routing 2,574,588.

Design A after B5: every U, R and N count above is unchanged; `-test tb_mesh -plus {+QUICK}` (N01–N04 and N06–N08 short forms in one run) gives 444,704 checks in 14,811 cycles.

Two-level timetable tests (B6), seed 1, on the design-C mesh `tb_mesh_c`: 4×4 with 2×2 regions by default, the project's 8×8 with 4×4 regions with `-define TB_MESH_N=8`. One test per run (`-test TL01`, `+define+TEST_TL01`); `-test tb_mesh_c` runs TL01–TL04 and TL06 in one simulation; TL05 runs on `tb_mesh_cb` (`-test TL05`). Times are Verilator 5.020 on Claude's machine; xsim takes longer, so please record its time. Section 13 says what each test does.

| Test | 4×4 (2×2 regions) | 4×4 `+QUICK` | 8×8 | Cycles 4×4 / 8×8 | Verilator time 4×4 / 8×8 |
|---|---|---|---|---|---|
| TL01 | 121,888 | 68,078 | 1,967,630 | 1,407 / 7,254 | under 1 s / 12 s |
| TL02 | 209,153 | 209,153 | 5,207,604 | 3,047 / 22,042 | under 1 s / 37 s |
| TL03 | 173,700 | 82,787 | 2,094,764 | 2,106 / 8,029 | under 1 s / 13 s |
| TL04 | 713,502 | 713,502 | 10,367,854 | 8,554 / 41,701 | 3 s / 68 s |
| TL05 (`tb_mesh_cb`) | 500,649 | 146,657 | 7,545,942 | 5,731 / 31,739 | 8 s / 2 min |
| TL06 | 442,714 | 296,594 | 31,799,704 | 4,485 / 108,612 | 1 s / 3 min |
| TL01–TL04, TL06 in one run (`-test tb_mesh_c`) | 1,634,484 | 1,461,178 | 55,587,325 | 19,149 / 207,395 | 7 s / 6 min |

| Test | Testbench | How | `checks=` | with `+QUICK` | Verilator time |
|---|---|---|---|---|---|
| U11 | `tb_phase_sequencer` | `-test U11`, `+define+TEST_U11` | 21,000,067 | 2,625,053 | 3 s |

TL02 and TL04 have no short form: `+QUICK` changes nothing in them. In EDA Playground TL04 (`+define+TEST_TL04`) runs with epochs of 512 cycles instead of 8,192, so its count there is 246,084 (2,962 cycles, with `+QUICK`). The TL04 and TL05 counts changed on 13 Nov, when two same-edge races in the bench were removed (Section 2, rule 10): every other count stayed as it was, and the xsim and Riviera-PRO logs of 11 and 13 Nov already show the new TL04 and TL05 counts. TL05 on 8×8 runs design C with 4×4 regions and k_L = 0 from reset, so only global slots, beside design B.

The exit check's 16×16 functional run is `-test TL01 -define TB_MESH_N=16` (16 regions, k_L 14, frames of 2,193 cycles). Verilator cannot build a 16×16 mesh on Claude's 8 GB machine, so xsim's log set the count: 8,813,898 checks in 9,558 cycles (R-1111-07, 7 min in all with the elaboration).

TL06 runs one period of the combined schedule by default. The tracker's 1,000,000 cycles: `-test TL06 -plus {+TL06_CYCLES=1000000}`, on 4×4 99,890,408 checks (1,000,048 cycles, 8 min on Verilator); with `+TL06_CYCLES=100000` on 4×4, 9,984,090 checks. On 8×8 a million cycles takes Verilator about half an hour, so expect hours on xsim.

Other design-C configurations, the one run on 4×4 with 2×2 regions (defines as in Section 13): SAFE subslots 2,325,004, B_MAX 5 1,732,278, B_MAX 15 2,293,099, 2 VCs 1,412,808, XY regular routing 1,634,357. The regular tests on design C (`+TEST=N01` and so on, 4×4, `+QUICK`): N01 10,280, N03 394,896, N06 221,057, N07 108,771; PERF with `+MEASURE=4000` 598,965.

Designs A and B after B6: every count above is unchanged.

Controller tests (B7), seed 1, on the design-D mesh `tb_mesh_d`: 4×4 with 2×2 regions and epochs of 2,048 cycles by default; the project's 8×8 with 4×4 regions and epochs of 4,096 cycles with `-define TB_MESH_N=8` (`-define TB_EPOCH=32768` gives D19's epoch). One test per run (`-test CT02`, `+define+TEST_CT02`); `-test tb_mesh_d` runs CT01–CT05 in one simulation. The default policy is PROP with damping; `-plus {+POLICY=SQRT}` and `-plus {+DAMP=0}` change them. Section 14 says what each test does.

| Test | 4×4 (2×2 regions) | 4×4 `+QUICK` | `+define+TEST_CTxx`, `+QUICK` (512-cycle epochs; too long for EDA Playground) | Cycles 4×4 | Verilator time 4×4 |
|---|---|---|---|---|---|
| CT01 | 1,257,636 | 1,257,636 | 314,806 | 14,381 | 8 s |
| CT02 | 2,389,921 | 1,823,944 | 456,604 | 26,664 | 21 s |
| CT03 | 2,756,268 | 2,008,703 | 502,317 | 30,760 | 19 s |
| CT04 | 3,062,877 | 2,688,476 | 768,301 | 34,863 | 19 s |
| CT05 | 1,606,240 | 1,270,771 | 720,347 | 17,235 | 10 s |
| CT01–CT05 in one run (`-test tb_mesh_d`) | 11,075,837 | 9,043,644 | — | 123,839 | 2 min |

On 8×8 the one run (`-test tb_mesh_d -define TB_MESH_N=8`) gives 93,009,953 checks in 342,947 cycles (17 min on Verilator; expect about an hour on xsim, most of it CT05's two global rounds at K_MAX, 52,992 cycles each). Run the 8×8 tests one at a time if that is too long: CT01 7,611,945 (28,740 cycles, 84 s), CT02 14,377,453 (53,319 cycles, 3 min), CT03 16,635,905 (61,501 cycles, 3 min), CT04 20,335,604 (77,901 cycles, 4 min), CT05 35,060,190 (125,926 cycles, 5 min).

Other design-D configurations, the one run on 4×4 with 2×2 regions (defines as in Section 13): SAFE subslots 21,365,343, B_MAX 5 11,030,799, B_MAX 15 21,324,633, 2 VCs 9,309,150, XY regular routing 11,071,791. The policies: `+POLICY=SQRT` 11,236,379, `+DAMP=0` 10,485,093, both 10,660,908.

| Test | Testbench | How | `checks=` | with `+QUICK` | Verilator time |
|---|---|---|---|---|---|
| U12 | `tb_demand_counter` | `-test U12`, `+define+TEST_U12` | 1,320,015 | 210,015 | 2 s |
| U13 | `tb_sched_ctrl` | `-test U13`, `+define+TEST_U13` | 3,358,679 | 419,900 | 1 s |
| U14 | `tb_ctrl_chain` | `-test U14`, `+define+TEST_U14` | 3,044 | 420 | 1 s |

Designs A, B and C after B7: every count above is unchanged.

Stress tests (B8), seed 1, on 4×4 (the lists `stress` and `stress_long`; what each test does: Section 15):

| Test | A `tb_mesh` | B `tb_mesh_b` | C `tb_mesh_c` | D `tb_mesh_d` | Cycles (D) |
|---|---|---|---|---|---|
| S01 `+QUICK` | 7,808,915 | 17,741,895 | 20,035,432 | 19,920,852 | 200,050 |
| S01 | PASS | PASS | PASS | PASS | 10,000,000 and the drain |
| S02 `+QUICK` | — | 1,245,842 | 1,409,802 | 1,404,594 | 14,176 |
| S02 | — | 7,251,645 | 8,199,426 | 8,166,292 | 82,156 |
| S03 `+QUICK`, `-define TB_NUM_VC=1` | — | 1,151,346 | 1,335,771 | 1,368,403 | 18,234 |
| S03, `-define TB_NUM_VC=1` | — | 5,304,216 | 6,133,525 | 6,167,954 | 82,096 |
| S05 `+QUICK` | 145,256 | 207,433 | 222,333 | 222,337 | 1,266 |
| S05 | 263,175 | 374,574 | 400,824 | 400,830 | 2,264 |

S01 in full was not pre-checked (about two hours per design on Verilator), so `stress_long` expects only PASS for it; Verilator ran design D's S01 for 2,000,000 cycles instead. At the 4×4 speeds of earlier xsim runs (780–860 cycles/s) the `stress` list takes about half an hour, `stress_long` without S01 about a quarter of an hour, and each S01 in full 3–4 hours: run the four side by side overnight (Section 4). S04 and S06 report no counts: S04's list expects PASS, and S06 is a comparison (Section 15).

B8 changed no count of B0–B7: all 22 runs of the list `all` gave the counts above (Verilator).

## 5. VCS and Verdi

`sim/vcs/Makefile` builds and runs any test by its ID, as run.tcl does for xsim. Each testbench and set of defines builds into its own folder, `sim/vcs/work/<testbench>[__<defines>]`, which is reused until a source file changes. From the repository root:

```
make -f sim/vcs/Makefile TEST=U01
make -f sim/vcs/Makefile TEST=CT02 DEFINES="TB_MESH_N=8"
make -f sim/vcs/Makefile TEST=tb_mesh_d PLUS="+QUICK"
make -f sim/vcs/Makefile TEST=S01 BENCH=tb_mesh_b PLUS="+QUICK"
make -f sim/vcs/Makefile regress LIST=stress
make -f sim/vcs/Makefile sweep SWEEP=E2_16x16 PART=1/2
```

Each run prints its RESULT line and the cycles it simulated; the log is `sim_<TEST>.log` in the build folder. `TEST` is a test ID or a testbench; a test that runs on several testbenches (PERF, N01, S01 …) also needs `BENCH`. `SEED=<n>` changes the seed. `regress` runs a list as regress.tcl does (`sh sim/vcs/regress.sh <list> [seed]`, summary in `sim/vcs/work/regress_<list>_seed<n>.txt`); `sweep` runs a sweep as sweep.tcl does, into the same `results/<name>/` files (`bash sim/vcs/sweep.sh <name> [k/n] ['glob']`, or `ONLY=` with make), so the CSV, `compare_csv.tcl` and the chart workbook work alike. E2's 16×16 mesh belongs here: xsim simulated 16×16 at 35 cycles/s (R-1111-07), and a design-D point there warms up for five epochs of 262,144 cycles.

Verdi: add `WAVE=1` to build for it (`-kdb -debug_access+all` and `+define+FSDB`; the run writes `dump.fsdb` in the build folder, `work/…__wave`), then `make -f sim/vcs/Makefile TEST=<id> WAVE=1 verdi`. Older VCS releases also need the Verdi PLI for FSDB: `VCS_EXTRA="-P $VERDI_HOME/share/PLI/VCS/LINUX64/novas.tab $VERDI_HOME/share/PLI/VCS/LINUX64/pli.a"`.

By hand, the same build and run are:

```
vcs -full64 -sverilog -timescale=1ns/1ps -f sim/files_rtl.f -f sim/files_tb.f +define+TB_MESH_N=8 -top tb_mesh_d -o simv_d8 -l vcs_d8.log
./simv_d8 +SEED=1 +TEST=CT02 -l sim_ct02.log
```

## 6. Xcelium (xrun) or Incisive (irun, NCLaunch)

`sim/xcelium/run.sh` builds and runs any test by its ID; `IRUN=1` uses irun (Incisive) instead of xrun. Each testbench and set of defines builds into `sim/xcelium/work/<testbench>[__<defines>]`. From the repository root:

```
sh sim/xcelium/run.sh -test U02
sh sim/xcelium/run.sh -test CT02 -define TB_MESH_N=8
sh sim/xcelium/run.sh -test S01 -bench tb_mesh_b -plus +QUICK
sh sim/xcelium/run.sh -test U02 -gui                      (SimVision)
SIM=xrun sh sim/vcs/regress.sh stress
SIM=xrun bash sim/vcs/sweep.sh S06
```

Options: `-test`, `-bench` (when the test runs on several testbenches), `-seed`, `-define N=V` and `-plus +ARG` (repeat them), `-gui` (`-access +rwc -gui`). By hand: `xrun -64bit -sv -timescale 1ns/1ps -f sim/files_rtl.f -f sim/files_tb.f -top tb_sync_fifo +SEED=1 -l xrun_u02.log`; `irun` takes the same arguments. In NCLaunch, point a run at `sim/files_rtl.f` and `sim/files_tb.f` (in that order), the include folders `tb/common`, `tb/mesh` and `tb/checkers`, the top module and the plusargs `+SEED=1 +TEST=<id>`.

Claude has neither VCS nor Xcelium: these scripts were checked with stand-ins that run the Verilator builds (`results/precheck/B8_precheck.log`), so the first real run of each is worth a look at its log.

## 7. Open-source tools (optional)

Verilator 5 runs every test. Claude uses it for the pre-delivery checks in `results/precheck/`.

```
verilator --binary --timing -f sim/files_rtl.f -f sim/files_tb.f --top-module tb_rr_arbiter -o tb_u01
./obj_dir/tb_u01 +SEED=1
```

The mesh tests need the include folders and, for 8×8, the define:

```
verilator --binary --timing -Itb/common -Itb/mesh -f sim/files_rtl.f -f sim/files_tb.f --top-module tb_mesh +define+TB_MESH_N=8 -o tb_mesh8
./obj_dir/tb_mesh8 +SEED=1 +TEST=N02
```

Yosys cannot read SystemVerilog packages directly. Convert the RTL with sv2v first; this is the fallback synthesis flow of D20.

```
sv2v -DSYNTHESIS rtl/pkg/noc_pkg.sv rtl/common/*.sv rtl/router/*.sv rtl/ni/*.sv rtl/top/*.sv > rtl_flat.v
yosys -p "read_verilog rtl_flat.v; chparam -set N 5 rr_arbiter; synth -top rr_arbiter; stat"
yosys -p "read_verilog rtl_flat.v; chparam -set NUM_VC 4 -set ALGO 1 router; synth -top router -flatten; stat"
yosys -p "read_verilog rtl_flat.v; chparam -set NUM_VC 4 -set MESH_N 8 tile; synth -top tile -flatten; stat"
```

The router run takes about 30 s and reports about 38,000 generic cells and 15,450 flip-flops for 4 VCs. 13,000 of those flip-flops are the 100 flit slots of the VC buffers (5 ports × 4 VCs × 5 flits × 130 bits).

The tile (router and NI, 4 VCs, XY, 8×8) reports 44,506 cells and 18,925 flip-flops; the NI alone 6,938 cells and 3,475 flip-flops, 3,300 of them in its 25-flit ejection FIFO (25 × 132 bits). With 1 VC the tile drops to 21,975 cells and 8,839 flip-flops.

The timetable blocks (B3):

```
sv2v -DSYNTHESIS rtl/pkg/noc_pkg.sv rtl/pkg/sched_pkg.sv rtl/sched/*.sv > sched_flat.v
yosys -p "read_verilog sched_flat.v; chparam -set TT_N 8 slot_table; synth -top slot_table -flatten; stat"
yosys -p "read_verilog sched_flat.v; chparam -set TT_N 8 -set T_SUB 34 -set S_SETUP 15 slot_counter; synth -top slot_counter -flatten; stat"
yosys -p "read_verilog sched_flat.v; chparam -set MESH_N 8 -set TT_N 8 group_to_router; synth -top group_to_router -flatten; stat"
```

On 8×8: `slot_table` 413 cells and 63 flip-flops (the 8 entries); `slot_counter` 115 cells and 20 flip-flops; `group_to_router` 287 cells (234 for a 4×4 region). A design-B tile needs one table, one counter and two partner blocks (the current subslot for sending, the next for reserving), about 1,100 cells: 3% of the router.

Design C (B6) adds one block, with `rtl/sched/phase_sequencer.sv` in the same file:

```
yosys -p "read_verilog sched_flat.v; chparam -set MESH_N 8 -set REGION_N 4 -set T_SUB_G 34 -set T_SUB_L 18 -set K_INIT 4 -set EPOCH 32768 phase_sequencer; synth -top phase_sequencer -flatten; stat"
```

`phase_sequencer` is 406 cells and 75 flip-flops on 8×8 (425 and 78 on 16×16): two slot counters, the frame state, the waiting plan and the epoch counter. The design-C tile (router and NI, 4 VCs, adaptive, 8×8 with 4×4 regions) is 64,390 cells and 22,426 flip-flops: 615 cells (1.0%) and 70 flip-flops more than design B's 63,775 and 22,356, for the sequencer, a second slot table and two more partner blocks. `resv_ctrl` with the local gaps is 331 cells (323 without). Design B's tile was 63,761 cells in B5: its logic is the same (`dec_loc` is tied to 0), and ABC maps the slightly different netlist 0.02% larger.

## 8. Logging a run in the tracker (Section 10)

A test counts as *Pass* only when its run row exists. Each row records:

- **Run ID:** `R-MMDD-nn` (for example `R-1103-01`), numbered per day.
- **Simulator:** for example `EDA Playground, Riviera-PRO 2023.04` or `Vivado 2023.2 xsim`.
- **Design, Mesh/VCs:** `—` for unit tests.
- **Test:** for example `U01`.
- **Configuration:** the seed, plus any plusargs.
- **Result:** for example `PASS, 310,493 checks`. Copy the numbers from the result line.
- **Files:** `results/R-1103-01.log`.

Then move the file's Status dropdown forward and add a dated note.

## 9. When something goes wrong

- **A test fails.** Rerun with the same seed: it fails the same way on every simulator. Add `+MAXERR=1000` to see every error, or `-wave` / `+VCD` for waveforms. Add a row to the issues log with the seed.
- **`$fatal` from a DUT check** (for example `push while full`). The message names the instance and the simulation time. The bug is in whatever drives that instance.
- **`'xvlog' not found`.** Start from the Vivado Tcl Shell, or source `settings64.sh` / `settings64.bat` first.
- **xsim prints its option list and `Expected a switch but found 1`.** An argument with `=` reached xsim split in two (the Windows batch rule above). run.tcl avoids this since 4 Nov; check that you have the current `sim/vivado/run.tcl`.
- **EDA Playground: unknown macro `TB_CHECK`.** testbench.sv was pasted only in part. Paste the whole file.
- **Different `checks=` counts on two simulators with the same seed.** Report it. The counts should match exactly.
- **`tb_router`: `send: packet ... was not queued`.** The testbench itself lost a packet before the router saw it, which is how the 6 Nov failure on Riviera and xsim started (a signed index; see the D30 rules in Section 2).
- **Mesh test: `deadlock, no packet delivered since cycle N`.** XY routing cannot deadlock, so in `tb_mesh` this is a bug: a lost credit, a VC never freed, or a packet that never leaves a buffer. Rerun that test alone with the same seed and `-wave` / `+VCD`. (In N05, with adaptive routing, the deadlock is the expected result.)
- **Mesh test: `mesh not idle afterwards, busy tiles 0000...0100`.** A flit, a busy VC or a missing credit was left behind. The bit string has one bit per node, node 0 on the right.
- **Mesh test: `the NI gave flit ..., but flit ... arrived first`.** The NI's ejection FIFO lost, changed or reordered a flit.
- **Mesh test: `... took N cycles, below its zero-load latency`.** A packet was faster than the pipeline allows (D32): a stage or a register is missing somewhere.
- **EDA Playground: `testbench.sv code must be less than 100000 characters`.** Use the three-file bundle of Section 3 (regenerate it with `make_bundle.tcl` after any change); paste each file into its own tab.
- **EDA Playground stops a mesh run before the RESULT line** (`Execution interrupted or reached maximum runtime`, exit code 137). Its runs stop at 60 s. Add `+QUICK` to Run Options, or run that test in Vivado; the design-D tests CT01–CT05 do not fit even in short form.
- **Riviera-PRO (EDA Playground) fails U12 or U13 while xsim and Verilator pass.** Compile with `-dbg` (Section 3). Without it Riviera-PRO 2025.04 EDU gave some testbench function calls wrong results: in U12 the model called a head for node 0 local on the 4×4 counter (the DUT, right, called it cross-region; R-1113-U12, R-1114-U12diag), and in U13 `ref_next_kl` returned 1 for an epoch without demand whose plan must stay 2 (R-1113-U13). With `-dbg` both pass with the counts of Section 4, every INFO line equal to xsim's and Verilator's (R-1114-U12dbg, R-1114-U13dbg). If another test ever passes on xsim and Verilator but fails on Riviera, rerun it with `-dbg` before suspecting the RTL.
- **U09 stops with `the shortest collision-free subslot is ... cycles, sched_pkg::t_tight says ...`.** The timing contract (D38) or the schedule changed, so TIGHT changed. Update `t_tight` in `sched_pkg.sv` and every number the tracker derives from it (Section 5).
- **Verilator hangs in a testbench loop and prints nothing.** Look for a function called in a loop condition (rule 8 in Section 2).
- **Bypass test: `burst R.. -> R.. (slot s subslot k, t=..): flit j (link i) expected on link X of R.. at cycle c (t+..), not there`.** router_checker expected a bypass flit on that link in that cycle (D38) and none came: a stage too many or too few in the source or relay, a lookahead that went the wrong way, or a mask that let a regular flit take the crossbar slot. `unexpected bypass flit` / `unexpected lookahead` is the opposite case. Rerun with the same seed and `+VCD`; the burst's t is in the message.
- **Bypass test: `$fatal` in `bypass_relay` (`burst flit missing`, `bypass flit ... without a lookahead`, `two lookaheads for output ...`) or in `crossbar` (`inputs ... all want output ...`).** The relay's state and the flits disagree, or a regular flit and a bypass flit met in the crossbar: the masks are wrong. The message names the router instance.
- **Bypass test: `the burst R.. -> R.. at cycle .. has N flits, expected M`.** The source chose other packets than the test expected (B_MAX, the packets' VCs, or the rule of D42).
- **Bypass test: `NI R.. at cycle ..: reserved .., bypass flits in the FIFO ..; the model says .. and ..`.** `ni_checker`'s model of that NI's bypass space disagrees with `resv_ctrl` (B5): a flit counted twice or not at all, a reservation not shrunk at its lookahead or not released at a no-show. `cancel token for R.. expected on ... at cycle .. (reverse link h), not there` / `unexpected cancel token on ...` comes from router_checker: the NI decided differently from the model, or a token took the wrong path or cycle.
- **Bypass test: `$fatal` in `resv_ctrl` (`fill .. + reserved .. exceed the bypass space ..`, `lookahead of a burst from nowhere reserved`, `.. reserved flits have not come for .. cycles (timeout)`).** The reservation rule (D48) or its follow-up broke, or a source sent a burst its NI had cancelled; the message names the NI.
- **Design C: `the sequencers are at ...; the frame reference says ...`** (mesh_checker). The tiles' `phase_sequencer` and `sched_ref`'s `frame_ref` disagree: a frame of the wrong length, a counter that moved while paused, or a plan applied at the wrong frame. `tiles out of step with R0` means two tiles disagree, for example a plan that reached them in different cycles.
- **Design C: `region-crossing links carry bypass traffic` in a local slot.** A local burst, lookahead or cancel token left its region: the local partner logic used global coordinates or the wrong table.
- **Design C: `lookaheads announce flits on a link or an NI that another burst holds then`, or `... disagree`.** mesh_checker saw two bursts collide, or a flit nobody announced, from the lookaheads alone (it knows nothing of the schedule). Most likely at a phase change, where the last subslot's tail meets the other timetable's first burst; the message gives the cycle and the link.
- **TL04: `the tiles switched at cycle ..., expected the frame start at ...`.** The plan was sampled or applied in the wrong cycle (D52: sampled at the end of cycle 255 of every epoch, applied at the first frame that starts after that).
- **TL05: `at cycle .. design C and design B differ at R..: ...`.** With one region (4×4) or k_L = 0 (8×8), design C must equal design B on every NI output and link; the message names the first signal that differed. The comparison stops there.
- **Design D: `tiles latched other demand sums than the model`.** `ctrl_checker`'s own count of occupied VCs (from the router's links) and a tile's `demand_counter` disagree: a head not seen, a free credit missed, the region test, the 16-cycle sampling or the saturation (D54). U12 tests the counter alone.
- **Design D: `the controller's sums are .. and .., the tiles' latched sums add up to ..`.** The serial sum tree lost or moved a bit: a node's start cycle (ct_start), a carry, or a link (D55). U14 tests the network alone.
- **Design D: `the controller's plan is .., expected ..`.** The share bin, the policy table, the clamp or damping (D56). The message gives the sums, the policy, damping and the last plan, enough to rerun the case in U13.
- **Design D: `tiles do not have plan ..` or `the last tile had the new plan only in cycle ..`.** The plan forwarding: a tile that does not pass its plan on, or a chain too long for A_DELAY.
- **Design D: `$fatal` in `ctrl_node` (`the demand sums overflow`).** A carry left after the last bit: the sum width (dm_sum_w) is too small for the mesh.
- **regress.tcl: COUNT.** The test passed with another count than the list expects: the simulators disagree. Rerun that entry alone with run.tcl and report both counts.
- **sweep.tcl: a point shows NONE, FAIL or NOCSV in `sweep.txt`.** Its log is `results/<sweep>/logs/<run_id>.log`. It wrote no row, and the next start runs it again. A point past saturation is not a failure: its row has `sat` = 1.
- **sweep.tcl: `could not replace <sweep>.csv`.** The CSV is open in Excel. Close it, then `set argv {-sweep <name> -merge}; source -notrace sim/vivado/sweep.tcl`.
- **Two Vivado windows on one sweep or list fail with xsim's `Could not open file ... for writing`.** Both ran the same build folder (the `xsim.type` entry below). Give each window its own part (`-part k/n`), or lists that build different testbenches.
- **A design-C test on 8×8 runs as 4×4.** Every mesh top defaults to 4×4; the 8×8 needs `-define TB_MESH_N=8`. The first line of the log (`TEST ... mesh 8x8 ...`) says which mesh ran.
- **xsim: `ERROR: [XSIM 43-3356] Could not open file xsim.dir/<testbench>_snap/xsim.type for writing`.** An earlier simulation of that testbench is still running: stopping a run in the Tcl console does not always end the simulator, and its process (`xsimk.exe`) keeps the snapshot's files open. Close Vivado, or end `xsimk.exe` in Task Manager, then run again (R-1111-01).
- **U11 runs for hours in xsim and prints nothing after its banner.** The `sched_pkg.sv` from before 13 Nov: xsim misread a cast in `epoch_len` at run time and its loop never ended (Section 2, rule 9). With the current file U11 takes minutes.
- **xsim is slow on 8×8.** Expected: the run.tcl summary prints the cycles per second. Run the 8×8 tests one at a time, and N08 with `-plus "+N08_CYCLES=200000"` if 1,000,000 cycles is too long; note it in the run log.
- **`tb_router`: `not idle after N cycles` or `packets missing`.** Packets or credits are stuck: a deadlock, a lost credit, or a VC that is never freed. The message names the router: T1 = 4 VCs XY, T2 = 4 VCs adaptive, T3 = 2 VCs adaptive, T4 = 1 VC XY. Rerun that seed with `+TEST=` on the failing test and `+VCD` or `-wave`.

## 10. Mesh tests (B2): what each test does

`tb_mesh` (XY routing, `TB_NUM_VC` VCs, default 4) holds N01–N04, N06–N08 and PERF; `tb_mesh_dl` (adaptive routing, 1 VC) holds N05. Both are thin tops over `mesh_bench`, which holds the mesh, a traffic source and a sink per node, the run control and the tests. Every packet of every test is checked at its sink: right node, head signature, every flit's data and order, delivered exactly once, and a latency no smaller than the zero-load latency of its hop count. Every flit the sink takes is also compared with the flit that reached the NI, which checks the NI's ejection FIFO. Every load phase ends with an idle check: no flit in any buffer or register, every VC idle, every credit home, every NI FIFO empty.

| Test | What it does | What it prints |
|---|---|---|
| N01 | Every node sends one packet to every node, itself included, all at once | `N01: 256 packets delivered by cycle 83` (4×4) |
| N02 | 1-flit packets at 0.002 packets/node/cycle, one per node every 500 cycles at a random phase, for transpose, bit rotation and shuffle. Every source sends the same number of packets, so the expected average latency is exact: 4(mean hops + 1) + 1 | `N02 transpose: average latency 15.000 cycles, expected 15.000; 100.0% of 400 packets at exactly zero-load` |
| N03 | 10 patterns × 3 loads (0.25, 0.5 and 0.9 × saturation) × 5 repetitions, mixed packets, then drain | one coverage line and one INFO line per pattern |
| N04 | Uniform at 0.5 × and 1.2 × the saturation load | latency and throughput of each; the 1.2 × line ends `saturated` |
| N05 | Adaptive routing with 1 VC at 0.6 packets/node/cycle until it deadlocks; then 2,000 more cycles to show the mesh is frozen, and the cycle of waiting packets | `N05: 6 packets wait in a cycle, each for the input VC the next one holds: R9.N R10.W R11.W R7.S R6.E R5.E` (4×4, seed 1) |
| N06 | 10% random sink stalls; the centre sink stops for 1,000 cycles; then every sink stops while every source offers a 1-flit packet every cycle, for C + 100 cycles, where C is the number of VCs (320 on 4×4, 1,408 on 8×8); then drain. A packet in the mesh holds at least one VC, so at most C fit: a source never refused would have pushed in more than C by itself, so every source must be blocked | how many packets the mesh held (at most C) and how long the sources were blocked |
| N07 | 5-flit packets only, uniform, bit complement and hotspot; packets on different VCs interleave at the sinks (with one VC none may) | interleaved packets seen |
| N08 | 1,000,000 cycles of uniform traffic at half the saturation load, then drain and the idle check | progress every 100,000 cycles |
| PERF | One measurement point from plusargs; prints the CSV row of the tracker's Section 8 | two `CSV,` lines (header and row) |

Loads are fractions of the saturation throughput: the most the mesh accepts under uniform traffic with mixed packets, measured with PERF in B2.

| Accepted at saturation, flits/node/cycle (packets/node/cycle) | 2×2 | 4×4 | 8×8 |
|---|---|---|---|
| 4 VCs, XY | 0.775 (0.332) | 0.602 (0.257) | 0.319 (0.137) |
| 1 VC, XY | 0.238 (0.102) | 0.167 (0.072) | 0.087 (0.037) |

Plusargs besides `+SEED`, `+TEST`, `+MAXERR`, `+CSV`, `+VCD`:

- `+QUICK` the short forms for EDA Playground (Section 4); `+N03_REPS=<n>` repetitions in N03 (default 5); `+N08_CYCLES=<n>` length of N08 (default 1,000,000); `+N05_RATE=<r>` load of N05 (default 0.6).
- PERF: `+PATTERN=<name>` (uniform, transpose, bitcomp, bitrot, shuffle, tornado, neighbor, hotspot, region, phase), `+RATE=<packets/node/cycle>` (default half of `sat_rate()`) or, since B8, `+RATE_FRAC=<share of sat_rate()>`, `+PDATA=<share of 5-flit packets>` (default 0.333), `+WARMUP=<cycles>` (2,000; design D five epochs, D60), `+MEASURE=<cycles>` (10,000), `+DRAIN=<cycles>` (200,000), `+STALL=<sink stall chance>` (0), `+RUNID=<text>`; for the patterns `+PLOCAL=<share inside the region>` (0.5), `+TREGION=<region side>`, `+HOTFRAC=<share to the hotspot>` (0.2), `+PHASE_A=`, `+PHASE_B=` and `+PHASE_LEN=<cycles>`; `+TRACE=<n>` a `TRACE,` line every n cycles (E6); design D also `+POLICY=SQRT` and `+DAMP=0`. Every design prints the same CSV columns (D58, `results/README.md`).

Example, one point of a latency curve on 8×8 in Vivado:

```
set argv {-test PERF -define TB_MESH_N=8 -plus "+PATTERN=transpose +RATE=0.05 +RUNID=A_8x8_tr_0.05 +CSV=perf.csv"}
source -notrace sim/vivado/run.tcl
```

The CSV file lands in `sim/vivado/work/tb_mesh__TB_MESH_N_8/`. Each PERF run writes the header and its row; keep the header once when you merge files.

Defines: `TB_MESH_N` (mesh side, default 4) and `TB_NUM_VC` (VCs of `tb_mesh`, default 4; Table III uses 1 or 4). `tb_mesh_dl` always has 1 VC.

## 11. Timetable tests (B3): what each test does

The timetable RTL (`sched_pkg`, `slot_table`, `slot_counter`, `group_to_router`) is checked against `sched_ref`, a reference written independently of it: groups and rows are found by walking the diagonals, slots by searching the Table II list, and partners by looking up the list of a subslot's bursts. U05–U07 prove that the RTL gives exactly the reference's schedule; U08–U09 prove that the reference's schedule never collides. Together they prove that the RTL's schedule never collides (D40).

| Test | What it does | What it prints |
|---|---|---|
| U05 | One `slot_table` per group, n = 2, 4, 8, 16. Every 8-bit slot value: the slot's groups are active and name each other, every other group is idle, a value beyond the round matches nothing. Every entry equals the reference and, on 4×4, Table II of [1]. A change of `grp_i` after reset changes nothing; a new reset loads it | `U05 4x4 group 0 table S0..S3 = {0, 1, 3, 6}` |
| U06 | Nine slot counters (n = 2–16; TIGHT and SAFE; B_MAX 5, 10, 15; the shortest legal subslot, Hmax + 1), each for a full round plus a slot, with a random enable and pauses at slot boundaries as in frames. Every cycle: setup, subslot, cycle and slot equal the reference position after the enabled cycles so far; the pulses fire exactly when they should and never while disabled | coverage lines |
| U07 | Every router, slot and subslot of the 2×2, 4×4, 8×8 and 16×16 timetables, and of 2×2, 4×4 and 8×8 regions on 8×8 and 16×16. Partner, mode and hops equal the reference; when A sends to B, B receives from A in the same mode; slot 8 of 4×4 gives the pairs of Fig. 3 of [1] | the Fig. 3 pairs |
| U08 | Every subslot of every timetable: paths minimal and disjoint (same-group bursts by XY and by YX, G6), each router sends once and receives once at most, local paths stay in their region, every ordered pair meets once per round | one line per timetable |
| U09 | Cycle by cycle, with every burst at B_MAX and a cancel token for every subslot: flits, lookaheads and cancel tokens never share a link and two flits never reach an NI in the same cycle, over full rounds and every local/global slot boundary. For n = 2, 4, 8, 16 and B_MAX = 5, 10, 15 it finds the shortest collision-free subslot, which must equal `sched_pkg::t_tight` | `U09 4x4 B_MAX=10: TIGHT = 18 cycles (SAFE 24, Hmax 6); ...` and the collision found one cycle shorter |

Timing of one bypass inside its subslot (D38), the contract the bypass datapath (B4) and the reservation controller (B5) must keep. The subslot starts at cycle t; the path has L hops, the burst B flits, flit j = 0…B−1, link i = 1…L from the source:

| Event | Cycle |
|---|---|
| The source checks for a cancel token and picks the packets | t |
| Flit j crosses the source's crossbar | t + 1 + j |
| Flit j is on link i | t + 1 + 2i + j |
| The lookahead is on lookahead link i | t + 2i |
| Flit j is written into the destination's NI | t + 2L + 3 + j |
| A cancel token for subslot k, decided in the first cycle of subslot k−1 (of the setup window for k = 0), is on reverse link h | decision + h |

A cancel token must reach its source by t, so a subslot lasts at least Hmax + 1 cycles, and the setup window of Hmax + 1 cycles covers subslot 0 (D13).

The TIGHT subslot U09 finds, in cycles (SAFE in brackets):

| Timetable (Hmax) | B_MAX = 5 | B_MAX = 10 | B_MAX = 15 |
|---|---|---|---|
| 2×2 (2) | 5 (11) | 10 (16) | 15 (21) |
| 4×4 (6) | 13 (19) | 18 (24) | 23 (29) |
| 8×8 (14) | 29 (35) | 34 (40) | 39 (45) |
| 16×16 (30) | 61 (67) | 66 (72) | 71 (77) |

That is TIGHT = 2·Hmax + B_MAX − 4, six cycles below SAFE (D39). On 4×4 and larger the limit is a destination's NI write port. In slot 6 of 4×4, R4 receives a burst from R15 (5 hops) in subslot 2 and from R0 (1 hop) in subslot 3. The second burst reaches R4 2 × (5 − 1) = 8 cycles earlier within its subslot, so the subslots must start at least 8 + B_MAX cycles apart. On 2×2 the limit is the source, which sends one burst at a time (T ≥ B_MAX). With B_MAX = 10, TIGHT gives slots of 79, 287 and 1,087 cycles and rounds of 790, 10,332 and 147,832 cycles on 4×4, 8×8 and 16×16 (SAFE: 1,030, 12,060 and 160,888).

Plusargs besides `+SEED`, `+TEST`, `+MAXERR`, `+VCD`: `+QUICK` (EDA Playground: U06's 16×16 counters run 4 slots; U07 checks the first 16 slots of 16×16 and 10 of the 8×8 regions; U08 leaves out 16×16; U09 checks n = 2, 4, 8 and frames of 4×4 regions on 8×8 with B_MAX = 10), `+U09_ALL` (the full set above).

## 12. Bypass and reservation tests (B4–B5): what each test does

`tb_mesh_b` is the design-B mesh: one global BuffeRS timetable in every tile, the bypass datapath (B4), reservation and cancellation (B5), minimal adaptive regular routing (D06) and 4 VCs. Defines: `TB_MESH_N` (default 4), `TB_NUM_VC` (4), `TB_B_MAX` (10), `TB_SAFE` (SAFE subslots instead of TIGHT), `TB_XY` (XY regular routing). Without `+TEST` it runs BP01–BP07, BP09 and BP10; BP08 needs one VC, so it runs only as `+TEST=BP08` with `TB_NUM_VC=1`. `+TEST=N01`, `N03`, `N06` or `N07` runs that regular test on design B instead.

How a bypass works (D07, D38, D42, D43). In the first cycle t of a subslot in which the timetable gives a router a partner, the bypass source looks at the front of every input VC of all five input ports for whole packets addressed to the partner whose head has not left yet. It takes, scanning from a start that moves on by one VC each burst, every packet that still fits in B_MAX flits, and sends its lookahead (destination, XY or YX, burst length). Flit j then leaves its VC at t + 1 + j and crosses the crossbar on input 4 (Local), whichever port it came from: no passing burst ever uses input 4, so the source never needs a crossbar input a passing burst is using (the rule D38 left to B4). Each router on the path gets the lookahead one cycle before the first flit, routes it, blocks switch allocation for that input and output for the B cycles of the burst, and lets each flit go from the link straight into the crossbar: 2 cycles per hop instead of 4. At the destination the burst goes to the NI, which keeps the bypass bit and returns no credit for it. Bypass flits take no buffer slot, credit or VC anywhere.

How a reservation works (D12, D48). One subslot ahead of every burst the timetable allows to a router (in the first cycle of the subslot before, or of the setup window for subslot 0), its NI decides. If the bypass flits in its FIFO, plus the flits already reserved, plus B_MAX fit in its bypass space, it reserves B_MAX slots; otherwise it sends a cancel token to the burst's source. The token runs back over the burst's own links in the other mode (XY and YX use the same links, reversed), one hop per cycle, so it is on reverse link h at the decision cycle + h and reaches the source before its subslot, which the source then skips. A reservation shrinks to the burst's length when the lookahead arrives at t + 2L, is released there if no burst came, and passes one slot to the FIFO with each flit written. The NI's FIFO holds NUM_VC × VC_DEPTH flits for regular traffic (the credits) plus a bypass space of 2·B_MAX + tail + 1 (`sched_pkg::byp_space`): 48 flits in all with 4 VCs, B_MAX 10 and TIGHT. So regular flits never block a bypass, and a sink that keeps up never sees a cancel.

Every test runs with `router_checker` (F43) and `ni_checker` (F47). For every subslot the checker takes the bursts the timetable allows from `sched_ref` (written independently of the RTL schedule) and asks `ni_checker`, a model of every NI's bypass space built from the links and the sinks alone, which of them their destination cancels. For an accepted burst it takes the length from the lookahead on the source's first link at t + 2 and expects every lookahead on its link at t + 2i, every flit on link i at t + 1 + 2i + j and in the destination's NI at t + 2L + 3 + j, the same flit on every link, whole packets for the partner, at most B_MAX flits. For a cancelled one it expects the token on every reverse link at its cycle and nothing from the source. Nothing else may carry a bypass flit, a lookahead or a token anywhere. `ni_checker` compares its counts with `resv_ctrl`'s every cycle and bounds the FIFO: regular flits at most the credits, all flits at most its depth. The sinks check every packet as in B2 (a bypass packet may not beat 2H + 2L + 3 cycles, D45), and every test ends with the idle check.

| Test | What it does | What it prints |
|---|---|---|
| BP01 | One 5-flit packet R2 → R9 on 4×4, held at R2 by the test hold until slot 8, subslot 2 (Table I and Fig. 3 of [1]); the hold is off in that cycle, so only the source's own hold and masks protect it | the cycles: decision, lookahead and flit 0 on link 1, tail in R9's NI |
| BP02 | The four corners, both ways: R0 ↔ R(n²−1) and R(n−1) ↔ R(n²−n), the longest paths, with two-packet bursts (10 and 6 flits); every packet's tail in its NI at exactly t + 2L + 3 + j. Run it on 8×8 too (`-define TB_MESH_N=8`). With TIGHT subslots the last flits land after the subslot ends, as U09 proved safe (D39) | each burst's length and the cycle its last flit lands |
| BP03 | One 5-flit packet for every ordered pair (240 on 4×4, 4,032 on 8×8), each injected 30 cycles before its pair's subslot and held at its source's Local input: every packet leaves by bypass in its own pair's subslot of one round and lands at t + 2L + 3 + j; one burst per pair, no turn cancelled, and the round has exactly one turn per ordered pair | `BP03: all 240 packets left by bypass in their own subslot of one round ...` |
| BP04 | More eligible flits than B_MAX. a) four 5-flit packets for the partner wait at the source: exactly two leave (10 flits), the other two wait. b) packets for the partner wait in two inputs of the source (two came from its east neighbour): whole packets, at most B_MAX, at least one from the East input, and nothing left out would fit. c) the source takes a packet from a mesh input while a burst from that input's neighbour crosses the same input in the same cycles (D38's rule): both arrive at their D38 cycles. d) on 2×2, where a subslot is exactly B_MAX cycles, a source sends a 10-flit burst and chooses its next burst, for its next partner, in the cycle the first one's last flit leaves | the bursts and how many packets came from another router |
| BP05 | Uniform traffic at 0.9 × the saturation load of design A, mixed packets, with bypass and reservations: no link, crossbar or lookahead conflict, every packet intact, idle afterwards. It lasts at least one timetable round (8×8: 10,332 cycles), so every pair gets its turn under load. Bursts of several packets and packets taken from mesh inputs are coverage goals (random traffic makes them rare in short or small runs; BP02 and BP04 make both happen on purpose) | bursts, packets and the bypass share (M07) |
| BP06 | R(n²−1)'s sink stalls for a round. Every other node holds packets for it, as many as one burst takes (two with B_MAX 10), injected just before its own turn: the first bursts fill the bypass space, then its NI cancels. Every cancelled source gets its token on every link of its path at its cycle, skips its turn with its packets waiting, and they arrive after the stall | `BP06: 2 of 15 turns to R15 accepted while its sink was stalled, then 13 cancelled ...` |
| BP07 | a) One idle round: every NI decides once per ordered pair, accepts every turn and releases each reservation as a no-show; no reservation is held at any slot boundary. b) One sink stalls for longer than a round while bursts and regular packets keep coming: its FIFO fills to its limit without overflow and its NI cancels; after the stall a burst to it is accepted and lands on time, and every credit comes home | the decisions, the cancels and the largest FIFO occupancy |
| BP08 | One VC, minimal adaptive routing, uniform traffic at 0.3 packets/node/cycle for 3,000 cycles (`+QUICK`: 1,000), longer until a cycle of waiting packets forms. Design A (`tb_mesh_dl`) deadlocks. Design B must deliver at least once per round, every cycle of waiting packets it forms (sampled every 50 cycles) must be broken by bypass, and the mesh drains. With `+QUICK` a cycle forming is a coverage goal: some seeds form none in 4,000 cycles | the first cycle of waiting packets, how many samples saw one, the bursts that broke them |
| BP09 | As BP03 for the pairs inside each group: the same-group slots (a = b), whose subslot 0 gives nobody a turn | as BP03 |
| BP10 | Uniform traffic at 0.20 packets/node/cycle on 4×4 with 4 VCs (elsewhere the same 78% of design A's saturation), 10,000 cycles measured: no turn cancelled while every sink keeps up; the sources', checker's and sinks' counts of bypassed packets agree; the bypass share is printed and written to the CSV with `perf_probe`'s columns | `BP10: bypass share ...` and a CSV row |
| U10 (`tb_resv_ctrl`) | `resv_ctrl` alone against a reference model of D48's rule, cycle by cycle, in six timetable shapes (2×2 to 16×16, B_MAX 5 to 15, TIGHT and SAFE, and a space smaller than the rule needs). Decisions, bursts and lookaheads come where the timetable puts them; the core takes flits with no stall, 25% and 75% stalls, then a full stall. Every rule must occur, and no turn may be cancelled while the core keeps up | one INFO line per shape |

The directed tests keep packets waiting with the router's simulation-only test hold and inject them 30 cycles before their subslot. A packet left out of a burst may leave later from another router on its path whose partner it is ([1] lets packets at every inport become bypass packets), or by the regular path.

BP05 and BP10 print the bypass share: about 0.3% of packets on 4×4, far below the 49–73% of [1] (issue of 9 Nov). The first E0 runs (`results/E0/R-1110-E0-01.csv`) show the same at every load from 0.02 to 0.23 packets/node/cycle and for uniform, transpose, bit rotation and shuffle traffic. E0 settles it before the comparison runs.

## 13. Two-level timetable tests (B6): what each test does

Design C splits the mesh into REGION_N × REGION_N regions (4×4 by default). Every region runs the 4×4 timetable of [1] at the same time in its own coordinates (the local timetable), and the whole mesh runs the global timetable of design B. A frame is k_L local slots, then one global slot; each timetable resumes where it stopped, so the phase changes only at slot boundaries (D15, D16). Design C keeps k_L fixed at K_FIXED, the number of local slots that take as long as one global slot: 4 on 8×8 (frames of 4 × 79 + 287 = 603 cycles) and 14 on 16×16. A local turn of every pair inside a region then comes at least once every 1,651 cycles on 8×8, instead of once every 10,332, while a cross-region pair waits one global round, 21,708 cycles. Every tile has its own `phase_sequencer` (F50): two slot counters, the frame logic, an epoch counter and the plan register. The plan input is sampled at the end of cycle A_DELAY − 1 = 255 of every epoch and takes effect at the first frame that starts after that (D19, D52); in design C the mesh ties it to K_FIXED, and TL03–TL04 force other plans. With REGION_N = MESH_N the sequencer keeps one pointer and the tile is design B's (D50).

`tb_mesh_c` (F70) is the design-C mesh with 4 VCs and minimal adaptive routing. Defines: `TB_MESH_N` (default 4, as in every mesh bench; the project's 8×8 needs `-define TB_MESH_N=8`), `TB_REGION_N` (2 on 4×4, else 4), `TB_NUM_VC` (4), `TB_B_MAX` (10), `TB_SAFE`, `TB_XY`, `TB_EPOCH` (default 2^13 = 8,192 cycles on 8×8 and larger, so a plan set at any time takes effect within about one epoch; D51). The default 4×4 mesh with 2×2 regions is the smallest two-level mesh (local slots of 23 cycles, k_L 3, frames of 148 cycles); it runs every TL test in seconds and is the EDA Playground form. Without `+TEST` it runs TL01–TL04 and TL06; `+TEST=N01`, `N03`, `N06`, `N07` or `PERF` runs that test on design C. `tb_mesh_cb` (F72) runs TL05.

Every design-C run has four checkers on, every cycle:
- `router_checker` (F43) and `ni_checker` (F47) as in design B, following the frames: in a local subslot the bursts allowed are the local timetable's in every region (`sched_ref::ref_mesh_xfers`), with the local setup window and subslot length for the decisions.
- `sched_monitor` (F53): every tile's turn (`tx_go`, partner, mode) and decision (`dec`, source, mode, hops, subslot 0, phase) against the reference at every subslot start and decision cycle, with or without traffic; every local turn stays in its region; at each round's end every pair inside a region had exactly one local turn per local round, and every ordered pair exactly one global turn per global round; the interval between turns of every pair.
- `mesh_checker` (F52): every tile's sequencer state equal to tile 0's (lockstep) and to the frame reference (`sched_ref::frame_ref`, the position computed in closed form from the cycle and the plan history); the frame rules seen from the RTL state alone (the phase changes only after a slot's last cycle, the paused counter does not move, k_L changes only at a frame start, every frame lasts exactly k_L local slots and one global slot and moves the slot numbers on by k_L and 1); no bypass flit, lookahead or cancel token on a link that crosses a region boundary in a local subslot; and a collision monitor that knows nothing of the schedule: each lookahead announces its link's flits for the next B cycles and the NI lookahead its writes, and two announcements may not overlap, nor a bypass flit come unannounced.

| Test | What it does | What it prints |
|---|---|---|
| TL01 | a) In every region one 5-flit packet for the region's next local turn, and one for a cross-region turn of the next global slot, each held at its source until its turn: each leaves by bypass in its own subslot and lands at t + 2L + 3 + j. b) Region traffic (70% inside the region) at 40% of design A's saturation for 6 frames and 4 local rounds (16×16: 3 and 2): no bypass traffic crosses a region boundary in a local subslot, every local turn stays inside its region, and bursts do cross region boundaries in global slots | the local-phase cycles checked, local turns, region-crossing bypass flits in global slots, local bursts per region (a and b together: a gives every region one) |
| TL02 | From the start of a global round, one 5-flit packet for every ordered pair (240 on 4×4, 4,032 on 8×8), each injected 30 cycles before its pair's first turn of that round (local for most pairs inside a region, global for the rest) and held at its source: every packet leaves by bypass in exactly that subslot and lands at t + 2L + 3 + j; the monitor's round checks pass for at least one whole global round and one local round inside the test | how many pairs a local turn served first, and the rounds checked |
| TL03 | Plan k_L = 3 on 8×8 (K_FIXED − 1 elsewhere), applied at the first frame start after the next sample; two directed bursts of B_MAX flits over the longest path of the last subslot before each phase change (local → global, then global → local), whose last flits land while the other timetable runs; then 12 frames with traffic, each checked to the cycle | the frame length, the sample and switch cycles, the boundary bursts |
| TL04 | Plans 0, K_MAX, 1, 3 in turn, each set at a random point of a random epoch, under region traffic with sinks that stall 30% of cycles: every tile switches in the same cycle, at the first frame start after the first sample after the plan was set (at least A_DELAY cycles after its epoch tick); the first slot after each switch makes reservations with the new phase's lengths; after the drain no reservation outlives its slot (in a slot's last cycle only reservations whose lookahead is due in that cycle are open), and the NI model agreed every cycle | each switch: when the plan was set, sampled and applied, and the flits reserved |
| TL05 (`tb_mesh_cb`) | A design-C mesh with one 4×4 region (D50) and a design-B mesh get the same traffic: region traffic at half of design A's saturation with random sink stalls and every sink stopped for 500 cycles, then BP06's cancellation scenario. Every NI output and every link of the two meshes must be equal in every cycle. `-define TB_MESH_N=8`: the same with 4×4 regions and k_L = 0 from reset (global slots only) | the cycles in which the meshes agreed, bursts, cancelled turns |
| TL06 | Random region traffic at half of design A's saturation for one period of the combined schedule (k_L 4 on 8×8: 180 frames, 108,540 cycles; at least three global rounds; `+TL06_CYCLES=1000000` for the tracker's million cycles): the longest interval between turns of a pair inside a region is at most 10 local slots plus ⌈10 / k_L⌉ global slots (1,651 cycles on 8×8), every cross-region pair's turns are exactly one global round apart (21,708 cycles), every ordered pair had two turns or more, and no turn is cancelled while every sink keeps up | the measured intervals against the bounds, the bypass share |
| U11 (`tb_phase_sequencer`) | Nine sequencers at once (8×8 with 4×4 regions TIGHT and SAFE, 8×8 with 2×2 regions and B_MAX 5, 16×16 with 4×4 and 8×8 regions, 4×4 with 2×2 regions, one pointer on 4×4 and 8×8, and a directed case where a sample falls in a frame's last cycle), the plan input changed at random: the whole state equal to `frame_ref` every cycle, the outputs and pulses with it, the epoch tick; thirteen checks of `sched_pkg`'s frame numbers against the tracker's (K_FIXED 4, 14, 3, 11; frames of 603, 524, 1,472 and 2,193 cycles; epochs 2^15 and 2^18; K_MAX 15 and 47; and 2^15 on 8×8 with B_MAX 5, the case where the epoch rule's k_L = 4 decides the power of two) | one INFO line per sequencer |

The 16×16 functional run (the exit check of B6) is TL01 with `-define TB_MESH_N=16`: 16 regions, k_L 14, frames of 2,193 cycles. Verilator needs more than the 8 GB of Claude's machine to build a 16×16 mesh, so this run is new on xsim; its count is recorded from your log. TL02 and TL06 are not meant for 16×16 (a global round there is 298,248 cycles).

The bypass share stays low in design C too (about 0.1% of packets on 8×8 in TL06, against 49–73% in [1]): the open question of B5, for E0.

## 14. Controller tests (B7): what each test does

Design D is design C with k_L set by demand. Every tile's `demand_counter` (F54) counts its router's occupied input VCs every 16th cycle of an epoch, split by whether the packet's destination is in the tile's region (local, C_L) or not (cross-region, C_G), into two 14-bit sums that saturate; at each epoch tick it latches them (D17, D54). In the epoch that follows, every tile's `ctrl_node` (F55) sends its sums one bit per cycle, least significant first, on two wires: each row adds from east to west into column 0, column 0 from south to north into tile (0, 0). Tile (x, y) adds bit k in cycle 2N − 1 − x − y + k after the tick, so every bit meets its partners with no valid wire. `sched_ctrl` (F56) beside tile (0, 0) has both sums in cycle 2N + W (W = 14 + log2(N²) bits; cycle 36 on 8×8), divides for the share bin q = ⌊16 C_L / (C_L + C_G)⌋ in five cycles, looks up the policy table, clamps to [K_MIN, K_MAX] = [1, 15] on 8×8 ([1, 47] on 16×16) and damps: the plan is set at the end of cycle 2N + W + 6. It travels back through the tiles one hop per cycle; the last tile has it by cycle 4N + W + 6 (40 on 4×4, 58 on 8×8, 92 on 16×16), long before `phase_sequencer` samples it at cycle A_DELAY − 1 = 255 (D52). The demand of epoch e therefore sets the frames from the first frame start after cycle 255 of epoch e + 1.

The table (D18, D56) is built at elaboration for both policies: bin q stands for C_L / C_G = (2q + 1) / (31 − 2q), its middle; PROP gives k_L = round(α C_L / C_G), SQRT round(α √(C_L / C_G)), α = (pair turns of a global slot) / (pair turns of one local slot in every region) = 7/6 on 8×8. On 8×8 PROP gives 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 6, 11, 15 for q = 0..15 and SQRT 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 4, 6. No demand at all keeps k_L; C_G = 0 (q = 16) gives K_MAX. Damping takes the mean of the old and the new k_L, rounded toward the new one, so k_L reaches any target within four epochs on 8×8 (six on 16×16, where the range is 1–47) and never overshoots. Policy and damping are inputs of the mesh (`ctrl_pol_i`, `ctrl_damp_i`), so one build runs every variant: `+POLICY=SQRT`, `+DAMP=0`.

`tb_mesh_d` (F74) is the design-D mesh with 4 VCs and minimal adaptive routing; its defines are those of `tb_mesh_c`, with `TB_EPOCH` 4,096 by default on 8×8 and larger (D19's 2^15 by `-define TB_EPOCH=32768`) and 512 under the bundle's `TEST_CTxx` defines. Every design-C checker runs in every design-D run, the frame reference taking each epoch's plan from the tiles, and `ctrl_checker` (F75) checks the controller end to end every epoch: every tile's latched sums against its own count from the links, the controller's sums, the plan of the controller and of every tile against `sched_ref`'s formulas (real arithmetic, written from the tracker), and every tile's arrival time. Traffic changes at epoch ticks; plan[m] below is the plan sampled in epoch m, set from epoch m − 1's demand.

| Test | What it does | What it prints |
|---|---|---|
| CT01 | Uniform traffic at 40% of design A's saturation for 6 epochs: k_L falls from K_FIXED. Every switch of k_L comes at a frame start at least A_DELAY cycles after its tick and at most one frame of the old k_L after the sample; at least one plan changes | the plans, the switch cycles, the latest arrival of a new plan |
| CT02 | Uniform traffic, then region traffic with 75% of packets inside the source's region, then uniform again, 4 epochs each (+QUICK 3): k_L rises in the first epoch planned from region traffic, plan[m1 + 1] > plan[m1], and falls in the first epoch planned from uniform traffic again, plan[m2 + 1] < plan[m2] | the plans of the three phases |
| CT03 | Region traffic (half inside the region) for 10 epochs with damping (+QUICK 6), then 4 without: from the fifth epoch on k_L stays within ±1 of the middle of its range (largest − smallest ≤ 2) | the plans with and without damping |
| CT04 | a) Traffic only inside the regions for 6 epochs (+QUICK 5): every tile's cross-region sum is 0 and k_L reaches K_MAX; b) every sink stopped for 1,500 cycles while every source offers traffic at saturation: the largest sums the mesh makes, still exact; c) traffic only across regions: k_L reaches its floor, the table's entry for the last epoch's share bin, which must be at most max(K_MIN, 1) (K_MIN with D18's K_MIN of 1; with E7's K_MIN of 0, 1 on 4×4, whose bins here are 4–7). C_L = 0 and saturated sums cannot happen in a mesh at these epochs (a packet always ends in its destination's region; 20 VCs fill a sum only after 13,107 cycles); U12–U14 test them exactly | the plans, the largest sums, the saturation goal |
| CT05 | Traffic only inside the regions until k_L = K_MAX, then two global rounds at K_MAX (+QUICK 1.2): every cross-region pair's turns come exactly one global round apart, n_slots(N) frames of K_MAX: 4,240 cycles on 4×4 with 2×2 regions, 52,992 on 8×8 TIGHT (67,680 SAFE) | the turn intervals across and inside regions |
| U12 (`tb_demand_counter`) | Seven counters at once (8×8/4, 4×4/2, 16×16/4, 16×16/8 with 2 VCs, 8×8/4 with 1 VC, a saturation run, a run with resets), random links: heads for free VCs, body, tail and bypass flits that must not count, free credits, credits that free nothing, a head and its VC's free credit together; epochs of random length. Every cycle the occupied VCs, which of them are local, and the latched sums equal the model's | one INFO line per counter |
| U13 (`tb_sched_ctrl`) | Five controllers (8×8/4, 16×16/4, 4×4/2, 16×16/8, and 8×8/2 with k_L 19 from reset, above its clamp), 10,000 epochs each with random policy and damping (epoch 0 always damps, so the clamp after damping is tested): random sums, no demand, C_G = 0, C_L = 0, the largest sums, sums exactly on a bin edge. The sums as shifted in, the share bin and the plan against `sched_ref`; the plan never changes outside its update; the 8×8 tables entry by entry against the values above | one INFO line per controller |
| U14 (`tb_ctrl_chain`) | 4×4, 8×8 and 16×16 networks of `ctrl_node` with `sched_ctrl`, 300 epochs each (epochs 1–3 always the corners: every node at its largest sums, no cross-region demand, one node only): every node's sums as it sent them against its subtree's, the controller's sums, every node's plan at the sample against `sched_ref`, and the arrival of each new plan by cycle 4N + W + 6 | the latest arrival against the bound |

The 8×8 one run shows the controller at work: under uniform traffic k_L goes to 1 (most demand crosses regions, so global slots get most of the time), under region traffic it rises to 3–4, with traffic only inside regions to K_MAX = 15, and back to 1 with traffic only across regions.

## 15. Stress tests (B8): what each test does

The stress tests are tasks of the mesh bench, chosen by `+TEST` on any mesh top: in Vivado `set argv {-test tb_mesh_b -plus {+TEST=S01 +QUICK}}`, with VCS `make -f sim/vcs/Makefile TEST=S01 BENCH=tb_mesh_b PLUS=+QUICK`. The lists `stress`, `stress_long` and `s04` (Section 4) hold every run with its expected count. Each runs every checker of its design. Decision D59.

- **S01, long run.** 10,000,000 cycles (`+QUICK` 200,000; `+S01_CYCLES=<n>`) at half of `sat_rate()`, the pattern switched every 50,000 cycles (`+QUICK` 10,000; `+S01_SEG=<n>`) through uniform, transpose, bit complement, bit rotation, shuffle, tornado, neighbour, hotspot and region traffic. Then the mesh drains and must be idle: no packet left, every VC idle, every credit home. It finds what only shows after millions of cycles: a credit, reservation or bypass slot that leaks one at a time, a counter that wraps, a source that starves. Every design. The bench's safety net (a run still going after 5,000,000 cycles plus N08's length fails) adds S01's length when S01 runs: the first xsim runs stopped at 6,000,000 cycles (R-1114-07 to 10).
- **S02, sinks that stall.** Designs B–D. The sinks stall with chance 1 %, 5 %, 10 % and 20 % per cycle, 20,000 cycles each (`+QUICK` 3,000) at half of `sat_rate()`. No reservation may be lost and no ejection FIFO may overflow (`ni_checker` and the FIFO's own check), and every packet arrives. Each level prints a line `S02,design,N,V,stall,decisions,accepted,dst_cancels,cancel_rate,turns,src_cancels,lat_avg,pkts`: how often the destinations cancel as the sinks slow down, a first look at E10.
- **S03, one VC near saturation.** Designs B–D with one VC (`-define TB_NUM_VC=1`) and minimal adaptive routing, at 0.9 of `sat_rate()` with uniform, transpose and hotspot traffic, 20,000 cycles each (`+QUICK` 4,000). The deadlock watchdog must stay silent and the mesh must drain: with one VC adaptive routing can close a cycle of waiting packets, which design A cannot leave (N05) and the bypass breaks (BP08).
- **S04, random configurations.** Sixteen configurations of designs B–D (`sim/lists/s04.list`): N 4 and 8, V 1, 2 and 4, B_MAX 5, 10 and 15, SAFE and TIGHT subslots, each value at least twice per design. Each runs S01 for 30,000 cycles with a pattern switch every 10,000. The counts depend on the configuration, so the list expects only PASS.
- **S05, no unknown values.** Every design, every cycle of 2,000 cycles of traffic (`+QUICK` 1,000) and its drain: no X or Z on any link or NI output: every valid bit of a flit, credit, lookahead or cancel token, and every field of one whose valid bit is 1; design D's demand sums and plans; the NI's handshakes and free counts. A record's other fields are not checked while its valid bit is 0: like buffer storage, the data fields of output registers have no reset and hold X until first written, and a port at the mesh's edge never writes them (the first xsim run checked whole records and failed from cycle 6, R-1114-02 and 04). It needs a four-state simulator (xsim, Riviera-PRO, VCS, Xcelium); Verilator has no X, so there the checks run but cannot fail, and its counts only confirm that they run.
- **S06, the simulators agree.** The sixteen points of `sim/sweeps/S06.sweep` (designs A–D, uniform and transpose traffic, 0.3 and 0.7 of saturation, a 500-cycle warm-up and a 2,000-cycle window) on xsim, compared with Verilator's rows in `results/precheck/S06_verilator.csv`: `set argv {-sweep S06}; source -notrace sim/vivado/sweep.tcl`, then `tclsh sim/compare_csv.tcl results/S06/S06.csv results/precheck/S06_verilator.csv` (in Vivado: `set argv {results/S06/S06.csv results/precheck/S06_verilator.csv}; source sim/compare_csv.tcl`). Every column of every row must match, because the same seed gives the same run on every simulator; the last line says `S06 PASS: every row is identical`. A Riviera-PRO log of any of the points (Run Options `+TEST=PERF +PATTERN=uniform +RATE_FRAC=0.30 +WARMUP=500 +MEASURE=2000 +RUNID=S06_B_uniform_f0.30` on `+define+TEST_BP01`, for example) can be compared the same way: `compare_csv.tcl` reads the `CSV,` lines of a log.
