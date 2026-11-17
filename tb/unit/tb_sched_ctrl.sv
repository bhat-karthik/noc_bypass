// =============================================================================
// File      tb/unit/tb_sched_ctrl.sv                               Tracker: F58
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U13: design D's controller against sched_ref's formulas: the sums
//           shifted in, the share bin, the PROP and SQRT tables, the clamp,
//           damping, the two corners (no demand, no cross-region demand) and
//           the timing of the plan
// DUT       rtl/ctrl/sched_ctrl.sv (F56)
// Reference tb/common/sched_ref.sv (F36): ref_bin, ref_kl, ref_next_kl, in
//           real arithmetic from the tracker's formulas
// @tests    U13
// -----------------------------------------------------------------------------
// Five controllers run at the same time: 8x8/4 (k_L 4 from reset), 16x16/4
// (14), 4x4/2 (3), 16x16/8 (k_fixed) and 8x8/2 (19, above K_MAX 15: the
// clamp after damping, in epoch 0, which always damps). Each gets an epoch tick, then its two
// sums bit-serially in the cycles the sum tree delivers them (bit k in cycle
// 2N + k after the tick), with a random policy and damping, every
// epoch; epochs are ct_sums(N) + 8..40 cycles long. The sums: random over
// the whole range or small, or a corner: both 0 (no demand: k_L stays),
// C_G = 0 (K_MAX), C_L = 0 (bin 0), both at the mesh's largest sum
// (n^2 (2^14 - 1)), C_L at the largest and C_G = 1, and sums that put
// 16 C_L / (C_L + C_G) exactly on a bin's edge.
// Checks: in cycle ct_sums(N) both sums as shifted in (one check); after the
// plan is set, the share bin (white box, when there is demand) and the plan
// against ref_next_kl (one check each); every cycle the plan output holds
// the model's plan (one check: it changes only when it is set). Before the
// run, the 8x8/4 tables of PROP and SQRT entry by entry against the values
// written out in the tracker (D56), from sched_pkg and from sched_ref (64
// checks).
// Epochs: 10,000 per controller; +QUICK 1,250.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module ctl_tester
  import sched_pkg::*;
#(
  parameter int N   = 8,
  parameter int RG  = 4,
  parameter int KI  = 4,
  parameter int NEP = 10000,
  parameter int SID = 300
) (
  input  logic go,
  output logic done
);
  localparam int SW = dm_sum_w(N);
  localparam int CS = ct_sums(N);
  localparam int C0 = 2 * N;
  localparam longint SMAX = longint'(N * N) * longint'((1 << DM_W) - 1);

  logic       clk = 1'b0, rst_n = 1'b0, tick = 1'b0, pol = 1'b0, damp = 1'b0;
  logic [1:0] sum = 2'b00;
  kl_t        plan;

  always #5 clk = ~clk;

  sched_ctrl #(.MESH_N(N), .REGION_N(RG), .K_INIT(KI)) u_dut (
    .clk_i (clk), .rst_ni (rst_n), .tick_i (tick), .sum_i (sum),
    .policy_i (pol), .damp_i (damp), .plan_o (plan)
  );

  string  name;
  longint n_hold, n_kmax, n_bin0, n_edge, n_big, n_damp, n_sqrt;

  function automatic longint rnd_sum(int kind);
    longint v, a, b;
    int     r1, r2;
    r1 = int'(tb_pkg::rng_below(SID, 1 << 30));
    r2 = int'(tb_pkg::rng_below(SID, 1 << 30));
    a  = longint'(r1);
    b  = longint'(r2);
    v  = (a << 30) | b;
    if (kind == 0) v = v % (SMAX + 1);          // the whole range
    else           v = v % 4096;                // small
    return v;
  endfunction

  initial begin
    longint cl, cg, tq, t, nep;
    int     ep_len, k_model, k_exp, q, qe, kind, mm, r, gq, kp, tq2, ipol;
    bit     quick, ok, ppol, pdamp;
    done = 1'b0;
    wait (go === 1'b1);
    tb_pkg::rng_init(SID);
    quick = $test$plusargs("QUICK");
    nep   = longint'(NEP);
    if (quick) nep = nep / 8;
    name  = $sformatf("%0dx%0d/%0d", N, N, RG);
    k_model = KI;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    for (longint e = 0; e < nep; e++) begin
      // this epoch's sums
      kind = int'(tb_pkg::rng_below(SID, 20));
      mm   = int'(tb_pkg::rng_below(SID, 64));
      mm   = mm + 1;
      if (e == 0 || kind == 0) begin
        cl = 0; cg = 0; n_hold++;                       // no demand
      end else if (kind == 1) begin
        cl = rnd_sum(1) + 1; cg = 0; n_kmax++;           // C_G = 0
      end else if (kind == 2) begin
        cl = 0; cg = rnd_sum(1) + 1; n_bin0++;           // C_L = 0
      end else if (kind == 3) begin
        cl = SMAX; cg = SMAX; n_big++;                   // the largest sums
      end else if (kind == 4) begin
        cl = SMAX; cg = 1; n_big++;
      end else if (kind == 5) begin
        // exactly on the edge of bin q: C_L / S = q / 16
        q  = int'(tb_pkg::rng_below(SID, 15));
        q  = q + 1;
        tq2 = q * mm;
        cl  = longint'(tq2);
        tq2 = (16 - q) * mm;
        cg  = longint'(tq2);
        n_edge++;
      end else if (kind < 12) begin
        cl = rnd_sum(1); cg = rnd_sum(1);
      end else begin
        cl = rnd_sum(0); cg = rnd_sum(0);
      end
      r     = int'(tb_pkg::rng_below(SID, 4));
      ppol  = (r % 2) == 1;
      pdamp = (r / 2) == 1;
      // epoch 0 always damps: from a K_INIT above K_MAX only that epoch can
      // show the clamp after damping (B7 mutation check, M16)
      if (e == 0) pdamp = 1'b1;
      if (ppol)  n_sqrt++;
      if (pdamp) n_damp++;
      ep_len = int'(tb_pkg::rng_below(SID, 33));
      ep_len = ep_len + CS + 8;
      // the epoch, cycle by cycle (inputs at the falling edge, for the next
      // rising edge); cycle 0 is the tick
      for (int c = 0; c < ep_len; c++) begin
        tick = (c == 0);
        pol  = ppol;
        damp = pdamp;
        if (c >= C0 && c < C0 + SW) begin
          t   = cl >> (c - C0);
          sum[0] = t[0];
          t   = cg >> (c - C0);
          sum[1] = t[0];
        end else begin
          sum = 2'b00;
        end
        @(negedge clk);
        // (at this falling edge the registers show cycle c + 1)
        // the plan output holds the model's plan except after it is set
        if (c == CS + 6) begin
          ipol  = int'(ppol);
          k_exp = sched_ref_pkg::ref_next_kl(ipol, pdamp, N, RG, cl, cg, k_model);
          qe    = sched_ref_pkg::ref_bin(cl, cg);
          gq    = int'(u_dut.q_q);
          kp    = int'(plan);
          `TB_CHECK(kp == k_exp, ("U13 %s epoch %0d: plan %0d, expected %0d (C_L %0d C_G %0d policy %0d damping %0d last %0d)",
                                  name, e, kp, k_exp, cl, cg, ppol, pdamp, k_model))
          if (qe >= 0) begin
            `TB_CHECK(gq == qe, ("U13 %s epoch %0d: share bin %0d, expected %0d (C_L %0d C_G %0d)", name, e, gq, qe, cl, cg))
          end
          k_model = k_exp;
        end else begin
          kp = int'(plan);
          `TB_CHECK(kp == k_model, ("U13 %s epoch %0d cycle %0d: plan %0d changed outside its update, expected %0d",
                                    name, e, c, kp, k_model))
        end
        if (c == CS) begin
          tq = longint'(u_dut.sl_q);
          t  = longint'(u_dut.sg_q);
          ok = (tq == cl) && (t == cg);
          `TB_CHECK(ok, ("U13 %s epoch %0d: sums shifted in %0d %0d, sent %0d %0d", name, e, tq, t, cl, cg))
        end
      end
    end
    tb_pkg::info($sformatf("U13 %s: %0d epochs: %0d without demand, %0d with C_G = 0, %0d with C_L = 0, %0d at the largest sums, %0d on a bin edge, %0d SQRT, %0d damped",
                           name, nep, n_hold, n_kmax, n_bin0, n_big, n_edge, n_sqrt, n_damp));
    tb_pkg::cover_bin($sformatf("U13 %s epochs without demand", name), n_hold);
    tb_pkg::cover_bin($sformatf("U13 %s epochs with C_G = 0", name), n_kmax);
    tb_pkg::cover_bin($sformatf("U13 %s epochs with C_L = 0", name), n_bin0);
    tb_pkg::cover_bin($sformatf("U13 %s epochs at the largest sums", name), n_big);
    tb_pkg::cover_bin($sformatf("U13 %s epochs on a bin edge", name), n_edge);
    done = 1'b1;
  end

endmodule


module tb_sched_ctrl;
  import sched_pkg::*;

  logic       go = 1'b0;
  logic [4:0] done;

  ctl_tester #(.N(8),  .RG(4), .KI(4),  .SID(300)) t_8a  (.go(go), .done(done[0]));
  ctl_tester #(.N(16), .RG(4), .KI(14), .SID(301)) t_16a (.go(go), .done(done[1]));
  ctl_tester #(.N(4),  .RG(2), .KI(3),  .SID(302)) t_4a  (.go(go), .done(done[2]));
  ctl_tester #(.N(16), .RG(8), .KI(k_fixed(16, 8, 10, TM_TIGHT)), .SID(303)) t_16b (.go(go), .done(done[3]));
  ctl_tester #(.N(8),  .RG(2), .KI(19), .SID(304)) t_8b  (.go(go), .done(done[4]));   // above K_MAX 15

  `TB_DUMP(tb_sched_ctrl)

  initial tb_pkg::watchdog(100ms);

  // The 8x8/4 policy tables entry by entry against the tracker's values
  // (D56; written out here, not computed), from sched_pkg (the RTL's table)
  // and from sched_ref (the reference)
  task automatic tables();
    int exp_p [16];
    int exp_s [16];
    int gp, gs, rp, rs;
    exp_p = '{1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 3, 4, 6, 11, 15};
    exp_s = '{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 4, 6};
    for (int q = 0; q < 16; q++) begin
      gp = kl_table(POL_PROP, 8, 4, K_MIN, 15, q);
      gs = kl_table(POL_SQRT, 8, 4, K_MIN, 15, q);
      rp = sched_ref_pkg::ref_kl(0, 8, 4, q);
      rs = sched_ref_pkg::ref_kl(1, 8, 4, q);
      `TB_CHECK(gp == exp_p[q], ("U13 sched_pkg PROP table 8x8/4 bin %0d: %0d, the tracker says %0d", q, gp, exp_p[q]))
      `TB_CHECK(gs == exp_s[q], ("U13 sched_pkg SQRT table 8x8/4 bin %0d: %0d, the tracker says %0d", q, gs, exp_s[q]))
      `TB_CHECK(rp == exp_p[q], ("U13 sched_ref PROP table 8x8/4 bin %0d: %0d, the tracker says %0d", q, rp, exp_p[q]))
      `TB_CHECK(rs == exp_s[q], ("U13 sched_ref SQRT table 8x8/4 bin %0d: %0d, the tracker says %0d", q, rs, exp_s[q]))
    end
  endtask

  initial begin
    tb_pkg::start("U13", "sched_ctrl: sums, share bin, PROP and SQRT tables, clamp and damping against the reference formulas");
    tables();
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::info("U13 controllers: 8x8/4, 16x16/4, 4x4/2, 16x16/8, 8x8/2 (k_L 19 from reset)");
    tb_pkg::finish();
  end

endmodule
