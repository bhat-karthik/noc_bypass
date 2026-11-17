// =============================================================================
// File      tb/unit/tb_ctrl_chain.sv                               Tracker: F59
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U14: design D's controller network: N x N controller nodes wired
//           as in mesh_top, with sched_ctrl at (0, 0): the bit-serial sums
//           equal the arithmetic sums, the plan equals the reference, and it
//           reaches every node before the plan sample (cycle A_DELAY - 1)
// DUT       rtl/ctrl/ctrl_node.sv (F55), rtl/ctrl/sched_ctrl.sv (F56)
// Reference tb/common/sched_ref.sv (F36): ref_next_kl, ref_plan_by
// @tests    U14
// -----------------------------------------------------------------------------
// Three networks run at the same time: 4x4 (regions 2), 8x8 and 16x16
// (regions 4), each with epochs of 300 cycles (a tick, the sums, the plan,
// the sample at cycle 255, then some cycles more). Every epoch each node gets
// new latched sums (as demand_counter latches them at the tick): random over
// 0..2^14 - 1, or small, or one node only, or every node at 2^14 - 1 (the
// largest sums, which must not overflow), or no cross-region demand
// anywhere, or none at all; policy and damping change at random. Checks,
// per epoch: in cycle ct_sums(N) the controller's sums equal the sums of
// all nodes' values (one check); in cycle A_DELAY - 1 every node's plan
// equals ref_next_kl (one check); every node had the new plan by cycle
// ref_plan_by(N) = 4N + W + 6 (one check, when the plan changed). The
// latest arrival is printed.
// Epochs 1-3 are always the corners (largest sums, no cross-region
// demand, one node only); epoch 0 has no demand.
// Epochs: 300 per network; +QUICK 40.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module cc_tester
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int N   = 8,
  parameter int RG  = 4,
  parameter int KI  = 4,
  parameter int NEP = 300,
  parameter int EPL = 300,
  parameter int SID = 350
) (
  input  logic go,
  output logic done
);
  localparam int NN = N * N;
  localparam int CS = ct_sums(N);
  localparam int DMX = (1 << DM_W) - 1;

  logic clk = 1'b0, rst_n = 1'b0, tick = 1'b0, pol = 1'b0, damp = 1'b0;
  logic [DM_W-1:0] cl [NN] = '{default: '0}, cg [NN] = '{default: '0};   // the nodes' latched sums
  logic [DM_W-1:0] ncl [NN], ncg [NN];    // the next epoch's, chosen beforehand
  logic [1:0]      s_o [NN], s_e [NN], s_s [NN];
  kl_t             p_i [NN], p_o [NN];
  kl_t             c_plan;

  always #5 clk = ~clk;

  for (genvar y = 0; y < N; y++) begin : g_y
    for (genvar x = 0; x < N; x++) begin : g_x
      localparam int ID = y * N + x;
      if (x < N - 1) begin : g_e
        assign s_e[ID] = s_o[ID + 1];
      end else begin : g_e0
        assign s_e[ID] = 2'b00;
      end
      if (x == 0 && y < N - 1) begin : g_s
        assign s_s[ID] = s_o[ID + N];
      end else begin : g_s0
        assign s_s[ID] = 2'b00;
      end
      if (x > 0) begin : g_pw
        assign p_i[ID] = p_o[ID - 1];
      end else if (y > 0) begin : g_pn
        assign p_i[ID] = p_o[ID - N];
      end else begin : g_pc
        assign p_i[ID] = c_plan;
      end
      ctrl_node #(.MESH_N(N), .K_INIT(KI)) u_cn (
        .clk_i (clk), .rst_ni (rst_n), .cur_x_i (coord_t'(x)), .cur_y_i (coord_t'(y)), .tick_i (tick),
        .cl_i (cl[ID]), .cg_i (cg[ID]), .sum_e_i (s_e[ID]), .sum_s_i (s_s[ID]), .sum_o (s_o[ID]),
        .plan_i (p_i[ID]), .plan_o (p_o[ID])
      );
    end
  end

  // the latched sums change at the end of the tick cycle, as
  // demand_counter's do (a register: the change reaches the nodes with the
  // clock edge, not in the middle of a cycle)
  for (genvar i = 0; i < NN; i++) begin : g_lat
    always @(posedge clk) begin
      if (tick) begin
        cl[i] <= ncl[i];
        cg[i] <= ncg[i];
      end
    end
  end

  sched_ctrl #(.MESH_N(N), .REGION_N(RG), .K_INIT(KI)) u_ctrl (
    .clk_i (clk), .rst_ni (rst_n), .tick_i (tick), .sum_i (s_o[0]),
    .policy_i (pol), .damp_i (damp), .plan_o (c_plan)
  );

  string  name;
  longint n_big, n_zero, n_cg0, n_one, n_chg;
  // per node: the sums its subtree should send (row y from x east, or in
  // column 0 every row from y south), and the bits it sent this epoch
  longint xl [NN], xg [NN], gl [NN], gg [NN];

  function automatic void subtree_sums();
    longint a, b, t;
    for (int y = 0; y < N; y++) begin
      for (int x = 0; x < N; x++) begin
        a = 0;
        b = 0;
        if (x > 0) begin
          for (int xx = x; xx < N; xx++) begin
            t = longint'(ncl[y * N + xx]);
            a = a + t;
            t = longint'(ncg[y * N + xx]);
            b = b + t;
          end
        end else begin
          for (int yy = y; yy < N; yy++) begin
            for (int xx = 0; xx < N; xx++) begin
              t = longint'(ncl[yy * N + xx]);
              a = a + t;
              t = longint'(ncg[yy * N + xx]);
              b = b + t;
            end
          end
        end
        xl[y * N + x] = a;
        xg[y * N + x] = b;
      end
    end
  endfunction

  initial begin
    longint sl, sg, rl, rg2, nep;
    int     kind, k_model, k_exp, arr, arr_max, last_bad, by, nb, fb, v, r, hot, kp, t0, kb, ipol;
    bit     quick, ppol, pdamp, changed;
    longint bl, bg;
    done = 1'b0;
    for (int i = 0; i < NN; i++) begin
      ncl[i] = '0;
      ncg[i] = '0;
    end
    wait (go === 1'b1);
    tb_pkg::rng_init(SID);
    quick   = $test$plusargs("QUICK");
    nep     = longint'(NEP);
    if (quick) nep = nep / 8 + 3;
    name    = $sformatf("%0dx%0d", N, N);
    by      = sched_ref_pkg::ref_plan_by(N);
    k_model = KI;
    arr_max = 0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    for (longint e = 0; e < nep; e++) begin
      kind = int'(tb_pkg::rng_below(SID, 16));
      // epochs 1-3 are the corners, so that every length and seed covers
      // them (epoch 0 has no demand): every node at its largest sums, no
      // cross-region demand anywhere, one node only (B7 pre-check, seeds 2-4)
      if (e == 1) kind = 1;
      if (e == 2) kind = 2;
      if (e == 3) kind = 3;
      hot  = int'(tb_pkg::rng_below(SID, NN));
      r    = int'(tb_pkg::rng_below(SID, 4));
      ppol  = (r % 2) == 1;
      pdamp = (r / 2) == 1;
      last_bad = 0;
      // this epoch's latched sums, taken by the nodes at the end of its tick
      // cycle
      begin
          sl = 0;
          sg = 0;
          for (int i = 0; i < NN; i++) begin
            if (e == 0 || kind == 0) begin
              ncl[i] = '0;  ncg[i] = '0;
            end else if (kind == 1) begin
              ncl[i] = DM_W'(DMX);  ncg[i] = DM_W'(DMX);
            end else if (kind == 2) begin
              v = int'(tb_pkg::rng_below(SID, DMX + 1));
              ncl[i] = DM_W'(v);  ncg[i] = '0;
            end else if (kind == 3) begin
              v = int'(tb_pkg::rng_below(SID, DMX + 1));
              if (i == hot) begin ncl[i] = DM_W'(v);  ncg[i] = DM_W'(DMX - v); end
              else          begin ncl[i] = '0;  ncg[i] = '0; end
            end else if (kind < 9) begin
              v = int'(tb_pkg::rng_below(SID, 64));
              ncl[i] = DM_W'(v);
              v = int'(tb_pkg::rng_below(SID, 64));
               ncg[i] = DM_W'(v);
            end else begin
              v = int'(tb_pkg::rng_below(SID, DMX + 1));
              ncl[i] = DM_W'(v);
              v = int'(tb_pkg::rng_below(SID, DMX + 1));
               ncg[i] = DM_W'(v);
            end
            rl  = longint'(ncl[i]);
            rg2 = longint'(ncg[i]);
            sl  = sl + rl;
            sg  = sg + rg2;
          end
          if (e == 0 || kind == 0) n_zero++;
          else if (kind == 1)      n_big++;
          else if (kind == 2)      n_cg0++;
          else if (kind == 3)      n_one++;
          ipol    = int'(ppol);
          k_exp   = sched_ref_pkg::ref_next_kl(ipol, pdamp, N, RG, sl, sg, k_model);
          changed = (k_exp != k_model);
          subtree_sums();
          for (int i = 0; i < NN; i++) begin
            gl[i] = 0;
            gg[i] = 0;
          end
      end
      for (int c = 0; c < EPL; c++) begin
        tick = (c == 0);
        pol  = ppol;
        damp = pdamp;
        @(negedge clk);
        // each node's output in cycle c + 1 is bit c - ct_start of its sums
        for (int y = 0; y < N; y++) begin
          for (int x = 0; x < N; x++) begin
            t0 = 2 * N - 1 - x - y;
            kb = c - t0;
            if (kb >= 0 && kb < dm_sum_w(N)) begin
              bl = longint'(s_o[y * N + x][0]);
              bg = longint'(s_o[y * N + x][1]);
              gl[y * N + x] = gl[y * N + x] | (bl << kb);
              gg[y * N + x] = gg[y * N + x] | (bg << kb);
            end
          end
        end
        // the controller's sums, whole; every node's sums as it sent them
        if (c == CS) begin
          nb = 0;
          fb = -1;
          for (int i = 0; i < NN; i++) begin
            if (gl[i] != xl[i] || gg[i] != xg[i]) begin
              nb++;
              if (fb < 0) fb = i;
            end
          end
          `TB_CHECK(nb == 0, ("U14 %s epoch %0d: %0d nodes sent other sums than their subtrees hold; first node %0d sent %0d %0d, expected %0d %0d",
                              name, e, nb, fb, (fb < 0) ? 0 : gl[fb], (fb < 0) ? 0 : gg[fb], (fb < 0) ? 0 : xl[fb], (fb < 0) ? 0 : xg[fb]))
          rl  = longint'(u_ctrl.sl_q);
          rg2 = longint'(u_ctrl.sg_q);
          `TB_CHECK(rl == sl && rg2 == sg, ("U14 %s epoch %0d: the controller's sums %0d %0d, the nodes' %0d %0d", name, e, rl, rg2, sl, sg))
        end
        // (at this falling edge the registers show cycle c + 1)
        // which nodes still lack the new plan in cycle c + 1
        if (c < sched_pkg::A_DELAY - 1) begin
          for (int i = 0; i < NN; i++) begin
            kp = int'(p_o[i]);
            if (kp != k_exp) last_bad = c + 1;
          end
        end
        // the sample, at the end of cycle A_DELAY - 1: every node's plan
        if (c == sched_pkg::A_DELAY - 2) begin
          nb = 0;
          fb = -1;
          for (int i = 0; i < NN; i++) begin
            kp = int'(p_o[i]);
            if (kp != k_exp) begin
              nb++;
              if (fb < 0) fb = i;
            end
          end
          kp = int'(c_plan);
          `TB_CHECK(nb == 0 && kp == k_exp, ("U14 %s epoch %0d: controller plan %0d, %0d nodes without plan %0d (first %0d); sums %0d %0d, policy %0d, damping %0d, last %0d",
                                             name, e, kp, nb, k_exp, fb, sl, sg, ppol, pdamp, k_model))
          if (changed) begin
            // the last node lacked it in cycle last_bad, so it had it from
            // cycle last_bad + 1
            arr = last_bad + 1;
            `TB_CHECK(arr <= by, ("U14 %s epoch %0d: the last node had the new plan in cycle %0d, expected by %0d", name, e, arr, by))
            if (arr > arr_max) arr_max = arr;
            n_chg++;
          end
          k_model = k_exp;
        end
      end
    end
    tb_pkg::info($sformatf("U14 %s: %0d epochs, %0d plan changes, the last node had each new plan by cycle %0d (bound %0d, sample at %0d); %0d epochs with every sum at 2^14 - 1",
                           name, nep, n_chg, arr_max, by, sched_pkg::A_DELAY - 1, n_big));
    tb_pkg::cover_bin($sformatf("U14 %s epochs at the largest sums", name), n_big);
    tb_pkg::cover_bin($sformatf("U14 %s epochs without demand", name), n_zero);
    tb_pkg::cover_bin($sformatf("U14 %s epochs without cross-region demand", name), n_cg0);
    tb_pkg::cover_bin($sformatf("U14 %s plan changes timed", name), n_chg);
    done = 1'b1;
  end

endmodule


module tb_ctrl_chain;
  import sched_pkg::*;

  logic       go = 1'b0;
  logic [2:0] done;

  cc_tester #(.N(4),  .RG(2), .KI(3),  .SID(350)) t_4  (.go(go), .done(done[0]));
  cc_tester #(.N(8),  .RG(4), .KI(4),  .SID(351)) t_8  (.go(go), .done(done[1]));
  cc_tester #(.N(16), .RG(4), .KI(14), .SID(352)) t_16 (.go(go), .done(done[2]));

  `TB_DUMP(tb_ctrl_chain)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tb_pkg::start("U14", "ctrl_node network and sched_ctrl: bit-serial sums, the plan, and its arrival at every node before the sample");
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::info("U14 networks: 4x4, 8x8, 16x16");
    tb_pkg::finish();
  end

endmodule
