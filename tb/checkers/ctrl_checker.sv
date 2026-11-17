// =============================================================================
// File      tb/checkers/ctrl_checker.sv                            Tracker: F75
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Design D's controller, checked end to end in every design-D mesh
//           run (D57): its own model of every tile's demand from the
//           router's links, the latched sums, the mesh-wide sums, the plan
//           (sched_ref's controller formulas) and the cycle by which every
//           tile has it.
// Decisions D17 demand, D54 counting, D55 network timing, D56 arithmetic,
//           D57 B7 verification
// Tests     CT01-CT05 (tb_mesh_d) and every other design-D run
// -----------------------------------------------------------------------------
// Inputs (mesh_bench copies them at every falling edge): per tile the five
// input links of its router and the credits it sends back on them, the
// sums its demand_counter latched, the plan its sequencer samples; the
// controller's sums and plan; the bench's policy and damping inputs.
// The model follows D54 from the links alone: a VC is occupied from the
// cycle after its head arrives until its free credit has gone; it is local
// if the head's destination is in the tile's region; every 16th cycle of an
// epoch (the tick's included) adds the occupied VCs to two sums of 14 bits
// that saturate; a tick moves them to the latched pair. Checks, per epoch:
//   latched   in the cycle after a tick, every tile's latched sums equal the
//             model's (one check per tile)
//   sums      in cycle ct_sums(N) after a tick, the controller's sums equal
//             the model's mesh-wide sums (one check)
//   plan      in cycle A_DELAY - 1 (the sample), the controller's plan and
//             every tile's equal sched_ref's next k_L for those sums, the
//             policy and damping the controller used, and the last plan
//             (one check for the controller, one for all tiles)
//   arrival   every tile has the new plan by cycle ref_plan_by(N) (one
//             check per epoch whose plan changed); the latest arrival seen
//             is reported
// A class in a package (D41); arrays sized for 16x16.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package cchk_pkg;
  import noc_pkg::*;
  import sched_ref_pkg::*;

  /* verilator lint_off UNUSEDSIGNAL */

  localparam int MAXN  = 256;               // tiles
  localparam int MAXVC = MAXN * 5 * 4;      // input VCs
  localparam int DMMAX = (1 << 14) - 1;     // a saturated demand sum (D17)

  class cchk;
    int     n, nn, rg, nvc, ep, ad, cs, by;
    longint lep, lcs, lcs6, lad, lad1;      // the same as longint (README rule 9)
    bit     started;
    longint c0;
    // inputs of this cycle
    flit_t   fl  [MAXN * 5];
    credit_t cr  [MAXN * 5];
    int      rcl [MAXN], rcg [MAXN];        // the RTL's latched sums
    int      rpl [MAXN];                    // each tile's plan
    longint  rsl, rsg;                      // the controller's sums
    int      rkc;                           // the controller's plan
    int      pol;
    bit      damp;
    int      kmin, kmax;                    // the clamp (mesh_top's K_LO, K_HI)
    // model
    bit      occ [MAXVC], loc [MAXVC];
    int      acl [MAXN], acg [MAXN], lcl [MAXN], lcg [MAXN];
    longint  scl, scg;                      // mesh-wide latched sums
    int      k_cur, k_exp;                  // the plan in force, the one this epoch sets
    int      arr_max;                       // latest arrival of a new plan seen
    longint  n_epochs, n_changed, n_sat;
    int      k_lo, k_hi;                    // smallest and largest plan seen
    int      first_bad [MAXN];              // per tile: last cycle of this epoch without the new plan
    // per epoch, from its plan sample on: the plan, the share bin and the
    // mesh-wide sums it came from, the largest latched sum of one tile
    int      kq  [$];
    int      qq  [$];
    longint  slq [$], sgq [$];
    int      mxq [$];
    int      mx_tile;                       // largest latched sum of one tile, this epoch

    function new(int n_c, int rg_c, int nvc_c, int ep_c, int ad_c, int k0, int kmin_c = 1, int kmax_c = -1);
      int nn2, l;
      n       = n_c;
      nn      = n * n;
      rg      = rg_c;
      nvc     = nvc_c;
      ep      = ep_c;
      ad      = ad_c;
      nn2     = nn;
      l       = 0;
      for (int i = 0; i < 9; i++) begin
        if ((1 << l) < nn2) l++;
      end
      cs      = 2 * n + 14 + l;               // the sums are whole (D55), counted here
      by      = ref_plan_by(n);
      lep     = longint'(ep);
      lcs     = longint'(cs);
      lcs6    = lcs + 6;
      lad     = longint'(ad);
      lad1    = lad - 1;
      kmin    = kmin_c;
      kmax    = kmax_c;
      k_cur   = k0;
      k_exp   = k0;
      k_lo    = k0;
      k_hi    = k0;
      arr_max = 0;
      started = 1'b0;
      c0      = 0;
      rsl     = 0;
      rsg     = 0;
      rkc     = k0;
      pol     = 0;
      damp    = 1'b1;
      for (int i = 0; i < MAXN * 5; i++) begin
        fl[i] = '0;
        cr[i] = '0;
      end
      for (int r = 0; r < MAXN; r++) begin
        rcl[r] = 0;
        rcg[r] = 0;
        rpl[r] = k0;
      end
      for (int i = 0; i < MAXVC; i++) begin
        occ[i] = 1'b0;
        loc[i] = 1'b0;
      end
      for (int r = 0; r < MAXN; r++) begin
        acl[r] = 0;
        acg[r] = 0;
        lcl[r] = 0;
        lcg[r] = 0;
      end
    endfunction

    // destination d in the region of tile r
    function bit same_reg(int d, int r);
      int dx, dy, rx, ry;
      dx = d % n;
      dy = d / n;
      rx = r % n;
      ry = r / n;
      return (dx / rg == rx / rg) && (dy / rg == ry / rg);
    endfunction

    // one falling edge: c is the bench's cycle number
    function void step(longint c);
      longint e, ph, t;
      int     i, nl, ng, sl, sg, bad, fb, dst, vi, ci;
      bit     hd;
      head_t  h;
      if (!started) return;
      e  = c - c0;
      ph = e % lep;
      // a tick: the epoch's sums move to the latched pair
      if (ph == 0) begin
        scl = 0;
        scg = 0;
        for (int r = 0; r < nn; r++) begin
          lcl[r] = acl[r];
          lcg[r] = acg[r];
          acl[r] = 0;
          acg[r] = 0;
          t      = longint'(lcl[r]);
          scl    = scl + t;
          t      = longint'(lcg[r]);
          scg    = scg + t;
          if (lcl[r] == DMMAX || lcg[r] == DMMAX) n_sat++;
          if (lcl[r] > mx_tile) mx_tile = lcl[r];
          if (lcg[r] > mx_tile) mx_tile = lcg[r];
          first_bad[r] = 0;
        end
        n_epochs++;
      end
      // every 16th cycle: occupied VCs into the sums
      if (ph % 16 == 0) begin
        for (int r = 0; r < nn; r++) begin
          nl = 0;
          ng = 0;
          for (int k = 0; k < 5 * nvc; k++) begin
            i = r * 5 * nvc + k;
            if (occ[i] &&  loc[i]) nl++;
            if (occ[i] && !loc[i]) ng++;
          end
          sl = acl[r] + nl;
          sg = acg[r] + ng;
          if (sl > DMMAX) sl = DMMAX;
          if (sg > DMMAX) sg = DMMAX;
          acl[r] = sl;
          acg[r] = sg;
        end
      end
      // the cycle after a tick: every tile's latched sums
      if (ph == 1) begin
        bad = 0;
        fb  = -1;
        for (int r = 0; r < nn; r++) begin
          if (rcl[r] != lcl[r] || rcg[r] != lcg[r]) begin
            bad++;
            if (fb < 0) fb = r;
          end
        end
        `TB_CHECK(bad == 0, ("cycle %0d: %0d tiles latched other demand sums than the model; R%0d: local %0d cross-region %0d, model %0d %0d",
                             c, bad, fb, (fb < 0) ? 0 : rcl[fb], (fb < 0) ? 0 : rcg[fb], (fb < 0) ? 0 : lcl[fb], (fb < 0) ? 0 : lcg[fb]))
      end
      // both sums whole at the controller
      if (ph == lcs) begin
        `TB_CHECK(rsl == scl && rsg == scg, ("cycle %0d: the controller's sums are %0d and %0d, the tiles' latched sums add up to %0d and %0d",
                                             c, rsl, rsg, scl, scg))
      end
      // the controller sets the plan at the end of cycle cs + 6 with the
      // policy and damping of that cycle
      if (ph == lcs6) begin
        k_exp = ref_next_kl(pol, damp, n, rg, scl, scg, k_cur, kmin, kmax);
      end
      // arrival: until the plan sample, which tiles lack the new plan
      if (ph > lcs6 && ph < lad) begin
        for (int r = 0; r < nn; r++) begin
          if (rpl[r] != k_exp) first_bad[r] = int'(ph);
        end
      end
      // the sample: the controller's and every tile's plan
      if (ph == lad1) begin
        `TB_CHECK(rkc == k_exp, ("cycle %0d: the controller's plan is %0d, expected %0d (sums %0d and %0d, policy %0d, damping %0d, last plan %0d)",
                                 c, rkc, k_exp, scl, scg, pol, damp, k_cur))
        bad = 0;
        fb  = -1;
        for (int r = 0; r < nn; r++) begin
          if (rpl[r] != k_exp) begin
            bad++;
            if (fb < 0) fb = r;
          end
        end
        `TB_CHECK(bad == 0, ("cycle %0d (the plan sample): %0d tiles do not have plan %0d; first R%0d with %0d",
                             c, bad, k_exp, fb, (fb < 0) ? 0 : rpl[fb]))
        if (k_exp != k_cur) begin
          i = 0;
          for (int r = 0; r < nn; r++) begin
            if (first_bad[r] + 1 > i) i = first_bad[r] + 1;
          end
          `TB_CHECK(i <= by, ("cycle %0d: the last tile had the new plan %0d only in cycle %0d of the epoch, expected by %0d",
                              c, k_exp, i, by))
          if (i > arr_max) arr_max = i;
          n_changed++;
        end
        k_cur = k_exp;
        if (k_cur < k_lo) k_lo = k_cur;
        if (k_cur > k_hi) k_hi = k_cur;
        kq.push_back(k_cur);
        qq.push_back(ref_bin(scl, scg));
        slq.push_back(scl);
        sgq.push_back(scg);
        mxq.push_back(mx_tile);
        mx_tile = 0;
      end
      // this cycle's links change the occupancy from the next cycle on
      for (int r = 0; r < nn; r++) begin
        for (int p = 0; p < 5; p++) begin
          i  = r * 5 + p;
          vi = int'(fl[i].vc);
          ci = int'(cr[i].vc);
          hd = fl[i].vld && !fl[i].byp && is_head(fl[i].ftype);
          if (hd) begin
            h   = head_t'(fl[i].data);
            dst = int'(h.dst);
            occ[i * nvc + vi] = 1'b1;
            loc[i * nvc + vi] = same_reg(dst, r);
          end
          if (cr[i].vld && cr[i].vc_free && !(hd && vi == ci)) begin
            occ[i * nvc + ci] = 1'b0;
          end
        end
      end
    endfunction

    // the model's sums of the current epoch so far (tests read them)
    function longint run_l();
      longint s, t;
      s = 0;
      for (int r = 0; r < nn; r++) begin
        t = longint'(acl[r]);
        s = s + t;
      end
      return s;
    endfunction

    function longint run_g();
      longint s, t;
      s = 0;
      for (int r = 0; r < nn; r++) begin
        t = longint'(acg[r]);
        s = s + t;
      end
      return s;
    endfunction
  endclass

  // the design-D bench's checker (null otherwise): tests read the model
  cchk the_cchk;

  /* verilator lint_on UNUSEDSIGNAL */

endpackage
