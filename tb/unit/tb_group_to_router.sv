// =============================================================================
// File      tb/unit/tb_group_to_router.sv                          Tracker: F39
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U07: partners follow Eq. (2) and the partner rule; transmit and
//           receive partners agree; Fig. 3 of [1] is reproduced
// DUT       rtl/sched/group_to_router.sv (F35), the partner functions of
//           rtl/pkg/sched_pkg.sv (F32)
// Reference tb/common/sched_ref.sv (F36): the bursts of each subslot listed
//           from the rows of the two groups
// @tests    U07
// -----------------------------------------------------------------------------
// Exhaustive: every router of the mesh, every slot, every subslot, for the
// global timetables 2x2, 4x4, 8x8, 16x16 and the local ones (4x4 regions on
// 8x8 and 16x16, 2x2 regions on 8x8 and 16x16, 8x8 regions on 16x16, every
// region at once). +QUICK (EDA Playground) stops the 16x16 timetable after 16
// slots and the 8x8 regions of 16x16 after 10; slot 0 already holds the
// longest path. The schedule inputs are what slot_table gives (checked in
// U05): active when the router's group is in the slot, with the other group.
//   * tx and rx valid, partner, mode and hops equal the reference;
//   * the outputs read zero and invalid when the router is not active;
//   * from the DUT's own outputs: if A sends to B, then B receives from A
//     with the same mode and hop count, and every router sends and receives
//     at most once in a subslot;
//   * 4x4 slot 8 (groups 2 and 3) gives exactly the pairs of Fig. 3 of [1].
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module gr_tester
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int MESH = 4,     // mesh side
  parameter int TT   = 4      // timetable side (TT < MESH: local, regions of TT x TT)
) (
  input  logic go,
  output logic done
);
  localparam int NR = MESH * MESH;
  localparam int NS = n_slots(TT);

  coord_t  x = '0, y = '0;
  logic    act = 1'b0;
  tt_grp_t tgt = '0;
  sub_t    sub = '0;
  logic    tx_vld, rx_vld, tx_mode, rx_mode;
  coord_t  tx_x, tx_y, rx_x, rx_y;
  hop_t    tx_hops, rx_hops;

  group_to_router #(.MESH_N(MESH), .TT_N(TT)) u_dut (
    .x_i       (x),
    .y_i       (y),
    .act_i     (act),
    .tgt_i     (tgt),
    .sub_i     (sub),
    .tx_vld_o  (tx_vld),
    .tx_x_o    (tx_x),
    .tx_y_o    (tx_y),
    .tx_mode_o (tx_mode),
    .tx_hops_o (tx_hops),
    .rx_vld_o  (rx_vld),
    .rx_x_o    (rx_x),
    .rx_y_o    (rx_y),
    .rx_mode_o (rx_mode),
    .rx_hops_o (rx_hops)
  );

  // Fig. 3 of [1]: 4x4 slot 8, pairs (router of group 2, router of group 3) per subslot
  localparam int FIG3 [4][4][2] = '{
    '{'{2, 3},  '{7, 4},  '{8, 9},   '{13, 14}},
    '{'{2, 14}, '{7, 3},  '{8, 4},   '{13, 9}},
    '{'{2, 9},  '{7, 14}, '{8, 3},   '{13, 4}},
    '{'{2, 4},  '{7, 9},  '{8, 14},  '{13, 3}}};

  // What the DUT said for every router in the current subslot
  bit  d_tx_v [NR], d_rx_v [NR], d_tx_m [NR], d_rx_m [NR];
  int  d_tx [NR], d_rx [NR], d_tx_h [NR], d_rx_h [NR];
  // What the reference says (chip ids, -1 = none)
  int  e_tx [NR], e_rx [NR];
  bit  e_tx_m [NR], e_rx_m [NR];

  longint unsigned c_tx = 0, c_rx = 0, c_idle_same = 0, c_inactive = 0, c_xy = 0, c_yx = 0, c_fig3 = 0;
  int              max_hops = 0;

  function automatic int iabs(int v);
    return (v < 0) ? -v : v;
  endfunction

  // Reference for one subslot, over every region
  task automatic build_ref(int slot, int s);
    sched_ref_pkg::xfer_q_t q;
    int                     src, dst;
    for (int r = 0; r < NR; r++) begin
      e_tx[r] = -1;
      e_rx[r] = -1;
      e_tx_m[r] = 1'b0;
      e_rx_m[r] = 1'b0;
    end
    q = sched_ref_pkg::ref_xfers(TT, slot, s, 1'b0);
    for (int by = 0; by < MESH; by += TT) begin
      for (int bx = 0; bx < MESH; bx += TT) begin
        foreach (q[i]) begin
          src = (by + q[i].sy) * MESH + bx + q[i].sx;
          dst = (by + q[i].dy) * MESH + bx + q[i].dx;
          if (e_tx[src] != -1 || e_rx[dst] != -1)
            tb_pkg::fail($sformatf("reference: router %0d or %0d used twice in slot %0d subslot %0d", src, dst, slot, s));
          e_tx[src]   = dst;
          e_tx_m[src] = q[i].mode;
          e_rx[dst]   = src;
          e_rx_m[dst] = q[i].mode;
        end
      end
    end
  endtask

  // Apply one router's inputs and record and check its outputs
  task automatic probe(int r, int slot, int s);
    int  a, b, g, txi, rxi, exp_h;
    bit  in_slot;
    x = coord_t'(r % MESH);
    y = coord_t'(r / MESH);
    sched_ref_pkg::ref_slot(TT, slot, a, b);
    g       = sched_ref_pkg::ref_group(r % MESH % TT, r / MESH % TT, TT);
    in_slot = (g == a || g == b);
    act     = in_slot;
    tgt     = tt_grp_t'((g == a) ? b : a);
    sub     = sub_t'(s);
    #1;
    d_tx_v[r] = tx_vld;
    d_rx_v[r] = rx_vld;
    d_tx_m[r] = tx_mode;
    d_rx_m[r] = rx_mode;
    d_tx[r]   = int'(tx_y) * MESH + int'(tx_x);
    d_rx[r]   = int'(rx_y) * MESH + int'(rx_x);
    d_tx_h[r] = int'(tx_hops);
    d_rx_h[r] = int'(rx_hops);
    txi = e_tx[r];
    rxi = e_rx[r];
    `TB_CHECK(tx_vld === (txi != -1) && rx_vld === (rxi != -1),
      ("%0dx%0d/%0d router %0d slot %0d subslot %0d: tx_vld %b rx_vld %b, expected %b %b", MESH, MESH, TT, r, slot, s,
       tx_vld, rx_vld, txi != -1, rxi != -1))
    if (txi != -1) begin
      exp_h = iabs(txi % MESH - r % MESH) + iabs(txi / MESH - r / MESH);
      `TB_CHECK(d_tx[r] == txi && tx_mode === e_tx_m[r] && int'(tx_hops) == exp_h,
        ("%0dx%0d/%0d router %0d slot %0d subslot %0d: sends to %0d mode %b in %0d hops, expected %0d mode %b in %0d",
         MESH, MESH, TT, r, slot, s, d_tx[r], tx_mode, tx_hops, txi, e_tx_m[r], exp_h))
      c_tx++;
      if (tx_mode == MODE_XY) c_xy++;
      else                    c_yx++;
      if (exp_h > max_hops) max_hops = exp_h;
    end else begin
      `TB_CHECK(tx_x == '0 && tx_y == '0 && tx_mode == 1'b0 && tx_hops == '0,
        ("%0dx%0d/%0d router %0d slot %0d subslot %0d: tx fields not zero while tx_vld is 0", MESH, MESH, TT, r, slot, s))
    end
    if (rxi != -1) begin
      exp_h = iabs(rxi % MESH - r % MESH) + iabs(rxi / MESH - r / MESH);
      `TB_CHECK(d_rx[r] == rxi && rx_mode === e_rx_m[r] && int'(rx_hops) == exp_h,
        ("%0dx%0d/%0d router %0d slot %0d subslot %0d: receives from %0d mode %b in %0d hops, expected %0d mode %b in %0d",
         MESH, MESH, TT, r, slot, s, d_rx[r], rx_mode, rx_hops, rxi, e_rx_m[r], exp_h))
      c_rx++;
    end else begin
      `TB_CHECK(rx_x == '0 && rx_y == '0 && rx_mode == 1'b0 && rx_hops == '0,
        ("%0dx%0d/%0d router %0d slot %0d subslot %0d: rx fields not zero while rx_vld is 0", MESH, MESH, TT, r, slot, s))
    end
    if (in_slot && a == b && s == 0) c_idle_same++;
    // The same router, not active: nothing at all
    if (in_slot) begin
      act = 1'b0;
      #1;
      `TB_CHECK(!tx_vld && !rx_vld && tx_x == '0 && tx_y == '0 && rx_x == '0 && rx_y == '0 && tx_hops == '0 && rx_hops == '0,
        ("%0dx%0d/%0d router %0d slot %0d subslot %0d: outputs not idle with act_i = 0", MESH, MESH, TT, r, slot, s))
      c_inactive++;
    end
  endtask

  // The DUT's outputs agree with themselves across the subslot
  task automatic cross_check(int slot, int s);
    for (int r = 0; r < NR; r++) begin
      if (d_tx_v[r]) begin
        `TB_CHECK(d_tx[r] >= 0 && d_tx[r] < NR && d_tx[r] != r && d_rx_v[d_tx[r]] && d_rx[d_tx[r]] == r &&
                  d_rx_m[d_tx[r]] == d_tx_m[r] && d_rx_h[d_tx[r]] == d_tx_h[r],
          ("%0dx%0d/%0d slot %0d subslot %0d: %0d sends to %0d, which does not receive from it in the same mode and hops",
           MESH, MESH, TT, slot, s, r, d_tx[r]))
      end
      if (d_rx_v[r]) begin
        `TB_CHECK(d_rx[r] >= 0 && d_rx[r] < NR && d_tx_v[d_rx[r]] && d_tx[d_rx[r]] == r,
          ("%0dx%0d/%0d slot %0d subslot %0d: %0d receives from %0d, which does not send to it",
           MESH, MESH, TT, slot, s, r, d_rx[r]))
      end
    end
  endtask

  // Fig. 3 of [1]: in slot 8 of the 4x4 timetable, group 2 and group 3 pair up as printed
  task automatic check_fig3(int s);
    int lo, hi;
    for (int k = 0; k < 4; k++) begin
      lo = FIG3[s][k][0];
      hi = FIG3[s][k][1];
      `TB_CHECK(d_tx_v[lo] && d_tx[lo] == hi && d_tx_v[hi] && d_tx[hi] == lo && d_tx_m[lo] == MODE_XY && d_tx_m[hi] == MODE_YX,
        ("Fig. 3, slot 8 subslot %0d: R%0d and R%0d should swap bursts (XY up, YX down); R%0d sends to %0d, R%0d to %0d",
         s, lo, hi, lo, d_tx[lo], hi, d_tx[hi]))
      c_fig3++;
    end
  endtask

  int n_run;   // slots checked

  initial begin
    done  = 1'b0;
    n_run = NS;
    if ($test$plusargs("QUICK") && TT == 16) n_run = 16;
    if ($test$plusargs("QUICK") && TT == 8 && MESH == 16) n_run = 10;
    wait (go === 1'b1);
    for (int slot = 0; slot < n_run; slot++) begin
      for (int s = 0; s < TT; s++) begin
        build_ref(slot, s);
        for (int r = 0; r < NR; r++) probe(r, slot, s);
        cross_check(slot, s);
        if (MESH == 4 && TT == 4 && slot == 8) check_fig3(s);
      end
    end
    if (MESH == 4 && TT == 4) begin
      tb_pkg::info("U07 4x4 slot 8 reproduces Fig. 3 of [1]: subslot 0 R2-R3 R7-R4 R8-R9 R13-R14, subslot 1 R2-R14 R7-R3 R8-R4 R13-R9,");
      tb_pkg::info("    subslot 2 R2-R9 R7-R14 R8-R3 R13-R4, subslot 3 R2-R4 R7-R9 R8-R14 R13-R3");
      tb_pkg::cover_bin("4x4 Fig. 3 pairs checked", c_fig3);
    end
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d router sends", MESH, MESH, TT), c_tx);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d router receives", MESH, MESH, TT), c_rx);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d path by XY", MESH, MESH, TT), c_xy);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d path by YX", MESH, MESH, TT), c_yx);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d idle in subslot 0 of a same-group slot", MESH, MESH, TT), c_idle_same);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d router not active", MESH, MESH, TT), c_inactive);
    tb_pkg::cover_bin($sformatf("%0dx%0d/%0d longest path, Hmax = %0d hops", MESH, MESH, TT, hmax(TT)),
                      (max_hops == hmax(TT)) ? 1 : 0);
    done = 1'b1;
  end

endmodule


module tb_group_to_router;
  logic       go = 1'b0;
  logic [8:0] done;

  gr_tester #(.MESH(2),  .TT(2))  t_2    (.go(go), .done(done[0]));
  gr_tester #(.MESH(4),  .TT(4))  t_4    (.go(go), .done(done[1]));
  gr_tester #(.MESH(8),  .TT(8))  t_8    (.go(go), .done(done[2]));
  gr_tester #(.MESH(16), .TT(16)) t_16   (.go(go), .done(done[3]));
  gr_tester #(.MESH(8),  .TT(4))  t_8r4  (.go(go), .done(done[4]));
  gr_tester #(.MESH(16), .TT(4))  t_16r4 (.go(go), .done(done[5]));
  gr_tester #(.MESH(8),  .TT(2))  t_8r2  (.go(go), .done(done[6]));
  gr_tester #(.MESH(16), .TT(2))  t_16r2 (.go(go), .done(done[7]));
  gr_tester #(.MESH(16), .TT(8))  t_16r8 (.go(go), .done(done[8]));

  `TB_DUMP(tb_group_to_router)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tb_pkg::start("U07", "group_to_router: partners follow Eq. (2) and D37, tx and rx agree, Fig. 3 reproduced");
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
