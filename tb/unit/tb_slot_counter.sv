// =============================================================================
// File      tb/unit/tb_slot_counter.sv                             Tracker: F38
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U06: the slot counter's sequence (setup window, subslots, cycles,
//           slot number), its pulses and its enable, against the reference
//           every cycle, for full rounds of n = 2, 4, 8 and 16
// DUT       rtl/sched/slot_counter.sv (F34)
// Reference tb/common/sched_ref.sv (F36): the position computed from the
//           number of enabled cycles, not counted
// @tests    U06
// -----------------------------------------------------------------------------
// Nine counters with different sides and subslot lengths (TIGHT and SAFE,
// bursts of 5, 10 and 15 flits, and the shortest legal subslot Hmax + 1).
// Each runs one full round plus one slot of enabled cycles (+QUICK: the
// 16x16 counters run 4 slots, for EDA Playground's 60 s). The enable is
// random (85% of cycles), and after the first slot and one slot in four
// after that it stays low for 1-300 cycles, as when the other timetable of
// a frame runs (D16). Every cycle:
//   * setup, subslot, cycle and slot equal the reference position after the
//     enabled cycles so far (a disabled cycle shows the held position);
//   * sub_go, slot_last and nxt_go fire exactly when they should, and never
//     in a disabled cycle; nxt_sub names the subslot being decided.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module sc_tester
  import sched_pkg::*;
#(
  parameter int N     = 4,
  parameter int T_SUB = 18,
  parameter int SID   = 100     // random stream of this tester
) (
  input  logic go,
  output logic done
);
  localparam int     S     = s_setup(N);
  localparam int     LEN   = slot_len(N, T_SUB);
  localparam int     NSL   = n_slots(N);
  localparam longint TOTAL = longint'(NSL) * longint'(LEN) + longint'(LEN);

  logic              clk   = 1'b0;
  logic              rst_n = 1'b0;
  logic              en    = 1'b0;
  logic              setup, sub_go, slot_last, nxt_go;
  sub_t              sub, nxt_sub;
  logic [TCYC_W-1:0] cyc;
  slot_t             slot;

  slot_counter #(.TT_N(N), .T_SUB(T_SUB), .S_SETUP(S)) u_dut (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .en_i        (en),
    .setup_o     (setup),
    .sub_o       (sub),
    .cyc_o       (cyc),
    .slot_o      (slot),
    .sub_go_o    (sub_go),
    .slot_last_o (slot_last),
    .nxt_go_o    (nxt_go),
    .nxt_sub_o   (nxt_sub)
  );

  always #5 clk = ~clk;

  longint          e;      // enabled cycles so far
  longint          total;  // enabled cycles to run
  bit              quick;  // +QUICK
  int              n_ends; // slot ends seen
  int              gap;    // disabled cycles still to come in a forced pause
  int              draw;
  longint unsigned c_wrap = 0, c_gap_end = 0, c_gap_mid = 0, c_nxt_setup = 0, c_nxt_sub = 0, c_last = 0;
  longint unsigned c_sub [N];

  task automatic check_cycle();
    sched_ref_pkg::pos_t p;
    bit                  x_go, x_last, x_nxt;
    p      = sched_ref_pkg::ref_pos(N, T_SUB, S, e);
    x_go   = en && !p.setup && p.cyc == 0;
    x_last = en && !p.setup && p.sub == N - 1 && p.cyc == T_SUB - 1;
    x_nxt  = en && p.cyc == 0 && (p.setup || p.sub < N - 1);
    `TB_CHECK(setup === p.setup && int'(sub) == p.sub && int'(cyc) == p.cyc && int'(slot) == p.slot,
      ("%0dx%0d T=%0d after %0d enabled cycles: setup %b subslot %0d cycle %0d slot %0d, expected %b %0d %0d %0d",
       N, N, T_SUB, e, setup, sub, cyc, slot, p.setup, p.sub, p.cyc, p.slot))
    `TB_CHECK(sub_go === x_go && slot_last === x_last && nxt_go === x_nxt,
      ("%0dx%0d T=%0d after %0d enabled cycles (en %b): sub_go %b slot_last %b nxt_go %b, expected %b %b %b",
       N, N, T_SUB, e, en, sub_go, slot_last, nxt_go, x_go, x_last, x_nxt))
    if (x_nxt) begin
      `TB_CHECK(int'(nxt_sub) == (p.setup ? 0 : p.sub + 1),
        ("%0dx%0d T=%0d after %0d enabled cycles: nxt_sub %0d, expected %0d", N, N, T_SUB, e, nxt_sub,
         p.setup ? 0 : p.sub + 1))
      if (p.setup) c_nxt_setup++;
      else         c_nxt_sub++;
    end
    if (en && !p.setup) c_sub[p.sub]++;
    if (x_last) c_last++;
    if (en && e >= longint'(NSL) * longint'(LEN) && p.slot == 0 && p.setup && p.cyc == 0) c_wrap++;
    if (!en && gap == 0 && p.cyc != 0) c_gap_mid++;
  endtask

  initial begin
    done = 1'b0;
    for (int k = 0; k < N; k++) c_sub[k] = 0;
    wait (go === 1'b1);
    tb_pkg::rng_init(SID);
    repeat (2) @(posedge clk);
    #1 rst_n = 1'b1;
    quick  = $test$plusargs("QUICK");
    total  = (quick && N == 16) ? 4 * longint'(LEN) : TOTAL;
    e      = 0;
    gap    = 0;
    n_ends = 0;
    while (e < total) begin
      @(negedge clk);
      if (gap > 0) begin
        en = 1'b0;
        gap--;
        if (gap == 0) c_gap_end++;
      end else begin
        en = tb_pkg::rng_chance(SID, 0.85);
      end
      #1;
      check_cycle();
      // a phase change after the first slot and after one slot in four (D16)
      // (draws are never on the right of || or &&, so every simulator draws the same numbers, D30)
      if (en && slot_last) begin
        n_ends++;
        draw = int'(tb_pkg::rng_below(SID, 4));
        if (n_ends == 1 || draw == 0) gap = 1 + int'(tb_pkg::rng_below(SID, 300));
      end
      if (en) e++;
    end
    for (int k = 0; k < N; k++)
      tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d subslot %0d", N, N, T_SUB, k), c_sub[k]);
    if (total == TOTAL) tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d slot number wraps to 0", N, N, T_SUB), c_wrap);
    tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d last cycle of a slot", N, N, T_SUB), c_last);
    tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d pause at a slot boundary", N, N, T_SUB), c_gap_end);
    tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d pause inside a setup window or subslot", N, N, T_SUB), c_gap_mid);
    tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d cancel decision in the setup window", N, N, T_SUB), c_nxt_setup);
    tb_pkg::cover_bin($sformatf("%0dx%0d T=%0d cancel decision in a subslot", N, N, T_SUB), c_nxt_sub);
    done = 1'b1;
  end

endmodule


module tb_slot_counter;
  import sched_pkg::*;

  logic       go = 1'b0;
  logic [8:0] done;

  sc_tester #(.N(2),  .T_SUB(t_tight(2, 10)),  .SID(100)) t_2   (.go(go), .done(done[0]));
  sc_tester #(.N(4),  .T_SUB(t_tight(4, 10)),  .SID(101)) t_4t  (.go(go), .done(done[1]));
  sc_tester #(.N(4),  .T_SUB(t_safe(4, 10)),   .SID(102)) t_4s  (.go(go), .done(done[2]));
  sc_tester #(.N(4),  .T_SUB(t_tight(4, 5)),   .SID(103)) t_4b5 (.go(go), .done(done[3]));
  sc_tester #(.N(4),  .T_SUB(hmax(4) + 1),     .SID(104)) t_4m  (.go(go), .done(done[4]));
  sc_tester #(.N(8),  .T_SUB(t_tight(8, 10)),  .SID(105)) t_8t  (.go(go), .done(done[5]));
  sc_tester #(.N(8),  .T_SUB(t_safe(8, 15)),   .SID(106)) t_8s  (.go(go), .done(done[6]));
  sc_tester #(.N(16), .T_SUB(t_tight(16, 10)), .SID(107)) t_16t (.go(go), .done(done[7]));
  sc_tester #(.N(16), .T_SUB(t_safe(16, 10)),  .SID(108)) t_16s (.go(go), .done(done[8]));

  `TB_DUMP(tb_slot_counter)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tb_pkg::start("U06", "slot_counter: setup, subslots, cycles, slot number and enable equal the reference every cycle");
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::info("U06 counters: 2x2 T=10; 4x4 T=18, 24, 13, 7; 8x8 T=34, 45; 16x16 T=66, 72 (one round plus one slot each)");
    tb_pkg::finish();
  end

endmodule
