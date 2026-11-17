// =============================================================================
// File      tb/unit/tb_slot_table.sv                               Tracker: F37
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U05: the lookup tables equal Table II, for every group of
//           n = 2, 4, 8 and 16
// DUT       rtl/sched/slot_table.sv (F33), slot_of of rtl/pkg/sched_pkg.sv (F32)
// Reference tb/common/sched_ref.sv (F36): the slot list searched
// @tests    U05
// -----------------------------------------------------------------------------
// One tester per n with one slot_table per group. After reset, every 8-bit
// slot value is applied to all of them:
//   * a slot of the round makes exactly its groups active, each naming the
//     other group (its own in a same-group slot); every other group is idle;
//   * a value beyond the round matches nothing;
// so the table of group g holds, for every t, the slot of {g, t}. That slot
// is also compared with the reference list and, on 4 x 4, with the numbers
// printed in Table II of [1] (group 0's column is {0, 1, 3, 6}).
// The table is filled at reset: changing grp_i later changes nothing, and a
// new reset loads the new groups.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module st_tester
  import sched_pkg::*;
#(
  parameter int N = 4
) (
  input  logic go,
  output logic done
);
  localparam int NS = n_slots(N);

  logic    clk   = 1'b0;
  logic    rst_n = 1'b0;
  slot_t   slot  = '0;
  tt_grp_t grp [N];
  logic    act [N];
  tt_grp_t tgt [N];

  always #5 clk = ~clk;

  for (genvar g = 0; g < N; g++) begin : g_dut
    slot_table #(.TT_N(N)) u_dut (
      .clk_i  (clk),
      .rst_ni (rst_n),
      .grp_i  (grp[g]),
      .slot_i (slot),
      .act_o  (act[g]),
      .tgt_o  (tgt[g])
    );
  end

  // Table II of [1], 4 x 4: entry [g][t] = slot of groups g and t
  localparam int TABLE2 [4][4] = '{'{0, 1, 3, 6}, '{1, 2, 4, 7}, '{3, 4, 5, 8}, '{6, 7, 8, 9}};

  int              found [N][N];   // [g][t]: slot where group g was active with target t
  longint unsigned c_act = 0, c_idle = 0, c_same = 0, c_beyond = 0;
  longint unsigned c_tgt [N];
  string           line;

  task automatic do_reset();
    rst_n = 1'b0;
    repeat (2) @(posedge clk);
    #1 rst_n = 1'b1;
    @(negedge clk);
  endtask

  // Apply every slot value; check every group's outputs
  task automatic sweep(int shift);
    int a, b, g_of;
    bit exp_act;
    int exp_tgt;
    for (int v = 0; v < (1 << SLOT_W); v++) begin
      slot = slot_t'(v);
      #1;
      if (v < NS) ref_slot_pair(v, a, b);
      for (int k = 0; k < N; k++) begin
        g_of    = (k + shift) % N;          // the group this table was loaded with
        exp_act = (v < NS) && (g_of == a || g_of == b);
        exp_tgt = (g_of == a) ? b : a;
        `TB_CHECK(act[k] === exp_act,
          ("%0dx%0d table of group %0d, slot %0d: active %b, expected %b", N, N, g_of, v, act[k], exp_act))
        if (exp_act) begin
          `TB_CHECK(int'(tgt[k]) == exp_tgt,
            ("%0dx%0d table of group %0d, slot %0d: target %0d, expected %0d", N, N, g_of, v, tgt[k], exp_tgt))
          if (shift == 0) begin
            found[g_of][int'(tgt[k])] = v;
            c_act++;
            c_tgt[int'(tgt[k])]++;
            if (a == b) c_same++;
          end
        end else if (shift == 0) begin
          if (v < NS) c_idle++;
          else        c_beyond++;
        end
      end
    end
  endtask

  function automatic void ref_slot_pair(int v, output int a, output int b);
    sched_ref_pkg::ref_slot(N, v, a, b);
  endfunction

  initial begin
    done = 1'b0;
    for (int t = 0; t < N; t++) c_tgt[t] = 0;
    for (int g = 0; g < N; g++) for (int t = 0; t < N; t++) found[g][t] = -1;
    for (int k = 0; k < N; k++) grp[k] = tt_grp_t'(k);
    wait (go === 1'b1);
    do_reset();
    sweep(0);

    // Every table entry: the slot of {g, t} by the reference list, by the
    // package formula, and on 4 x 4 by the paper's table
    for (int g = 0; g < N; g++) begin
      for (int t = 0; t < N; t++) begin
        `TB_CHECK(found[g][t] == sched_ref_pkg::ref_table(N, g, t) && found[g][t] == slot_of(g, t),
          ("%0dx%0d group %0d meets group %0d in slot %0d; reference %0d, slot_of %0d", N, N, g, t,
           found[g][t], sched_ref_pkg::ref_table(N, g, t), slot_of(g, t)))
        if (N == 4) begin
          `TB_CHECK(found[g][t] == TABLE2[g][t],
            ("4x4 group %0d meets group %0d in slot %0d; Table II of [1] says %0d", g, t, found[g][t], TABLE2[g][t]))
        end
      end
    end
    if (N == 4) begin
      line = "";
      for (int t = 0; t < N; t++) begin
        if (t != 0) line = {line, ", "};
        line = {line, $sformatf("%0d", found[0][t])};
      end
      tb_pkg::info({"U05 4x4 group 0 table S0..S3 = {", line, "} (Table II of [1]: {0, 1, 3, 6})"});
    end

    // Filled at reset: new groups without a reset change nothing ...
    for (int k = 0; k < N; k++) grp[k] = tt_grp_t'((k + 1) % N);
    @(negedge clk);
    sweep(0);
    // ... and a reset loads them
    do_reset();
    sweep(1);

    tb_pkg::cover_bin($sformatf("%0dx%0d router active in its slot", N, N), c_act);
    tb_pkg::cover_bin($sformatf("%0dx%0d router idle in another group's slot", N, N), c_idle);
    tb_pkg::cover_bin($sformatf("%0dx%0d same-group slot", N, N), c_same);
    tb_pkg::cover_bin($sformatf("%0dx%0d slot value beyond the round matches nothing", N, N), c_beyond);
    for (int t = 0; t < N; t++)
      tb_pkg::cover_bin($sformatf("%0dx%0d target group %0d", N, N, t), c_tgt[t]);
    done = 1'b1;
  end

endmodule


module tb_slot_table;
  logic       go = 1'b0;
  logic [3:0] done;

  st_tester #(.N(2))  t_2  (.go(go), .done(done[0]));
  st_tester #(.N(4))  t_4  (.go(go), .done(done[1]));
  st_tester #(.N(8))  t_8  (.go(go), .done(done[2]));
  st_tester #(.N(16)) t_16 (.go(go), .done(done[3]));

  `TB_DUMP(tb_slot_table)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tb_pkg::start("U05", "slot_table: lookup tables equal Table II for every group, n = 2, 4, 8, 16");
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
