// =============================================================================
// File      tb/mesh/tests_ctrl.svh                                 Tracker: F73
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The B7 tests of design D's controller on a design-D mesh, as tasks
//           of mesh_bench (included after tests_two.svh, whose helpers they
//           use; top tb_mesh_d).
// Decisions D17 demand, D18 split policy, D19 epoch, D54-D56 controller,
//           D57 B7 verification
// Tests     CT01 CT02 CT03 CT04 CT05 (tb_mesh_d)
// -----------------------------------------------------------------------------
// In every design-D run, every cycle, every design-C checker runs (router,
// NI, schedule and mesh checkers, the frame reference taking each epoch's
// plan from the tiles), and the controller checker (F75) models every
// tile's demand from its router's links: each epoch it checks every tile's
// latched sums, the controller's mesh-wide sums, the plan of the controller
// and of every tile against sched_ref's formulas, and that every tile has a
// new plan by cycle ref_plan_by(N) (58 on 8x8), long before the sample at
// cycle A_DELAY - 1 = 255. The traffic changes at epoch ticks. Each test
// adds (plan[m] = the plan sampled in epoch m, from epoch m - 1's demand):
//   CT01  epoch timing: uniform traffic at 40% of design A's saturation for
//         6 epochs (k_L falls from K_FIXED). Every switch of k_L comes at a
//         frame start at least A_DELAY cycles after its tick and at most
//         one frame of the old k_L after the sample; at least one plan
//         changes.
//   CT02  the controller follows demand: uniform traffic, then region
//         traffic with p_local = 0.75, then uniform again, 4 epochs each
//         (+QUICK 3): k_L rises in the first epoch whose plan comes from
//         region traffic, plan[m1 + 1] > plan[m1], and falls back in the
//         first one after the return, plan[m2 + 1] < plan[m2].
//   CT03  the loop is stable: region traffic (p_local 0.5) for 10 epochs
//         with damping: from the fifth on, k_L stays within +-1 of the
//         middle of its range (largest - smallest <= 2); then 4 epochs
//         without damping, whose range is reported.
//   CT04  extremes: a) traffic only inside the regions: every tile's
//         cross-region sum is 0 and k_L reaches K_MAX; b) every sink
//         stopped for 1,500 cycles while every source offers traffic: the
//         largest sums the mesh can make, still exact (the controller
//         checker), no overflow (ctrl_node stops the run on one);
//         c) traffic only across regions: k_L reaches its floor, the
//         table's entry for the share bin this traffic makes, which must be
//         at most max(K_MIN, 1): K_MIN with D18's K_MIN of 1; 1 with E7's
//         K_MIN of 0 on 4x4, whose bins here are 4..7 (entry 1; 0 needs a
//         bin of 3 or less). C_L = 0 and saturated 14-bit sums cannot
//         happen in a mesh with these epochs (a packet always ends in its
//         destination's region; 20 VCs fill a sum only after
//         16,384 x 16 / 20 cycles): U12-U14 test those corners exactly.
//   CT05  the bound holds at K_MAX: traffic only inside the regions until
//         k_L = K_MAX, then two global rounds at K_MAX (+QUICK 1.2, which
//         measures only the pairs whose turn comes early): every cross-region
//         pair's turns come exactly one global round apart, n_slots(N)
//         frames of K_MAX: 52,992 cycles on 8x8 TIGHT (67,680 SAFE), 4,240
//         on 4x4 with 2x2 regions.
// Policy and damping: +POLICY=SQRT, +DAMP=0 (default PROP, damping on);
// CT03 switches damping itself. +QUICK shortens CT02-CT05 for EDA
// Playground (tb_mesh_d sets 512-cycle epochs under TEST_CT0x there).
// =============================================================================

  /* verilator lint_off UNUSEDSIGNAL */

  localparam int CT_KMAX = K_HI;

  // the epoch number of a cycle, and the first cycle of epoch m
  function automatic int ep_of(int unsigned c);
    int a, b;
    a = int'(c);
    b = int'(c_rst);
    return (a - b) / EPOCH;
  endfunction

  function automatic int unsigned ep_t0(int m);
    int unsigned o;
    o = m * EPOCH;
    return c_rst + o;
  endfunction

  // the plan sampled in epoch m (-1 before its sample)
  function automatic int plan_of(int m);
    if (cchk_pkg::the_cchk == null) return -1;
    if (m < 0 || m >= cchk_pkg::the_cchk.kq.size()) return -1;
    return cchk_pkg::the_cchk.kq[m];
  endfunction

  // the controller's configuration from the plusargs (+POLICY=SQRT, +DAMP=0)
  task automatic ct_config();
    string pol;
    int    dmp;
    if ($value$plusargs("POLICY=%s", pol)) begin
      if (pol == "SQRT" || pol == "sqrt") ctrl_pol = 1'b1;
    end
    if ($value$plusargs("DAMP=%d", dmp)) ctrl_damp = (dmp != 0);
    tb_pkg::info($sformatf("design D controller: policy %s, damping %0d, epochs of %0d cycles, k_L %0d..%0d from %0d",
                           pol_name(), ctrl_damp, EPOCH, K_LO, CT_KMAX, K_INIT));
  endtask

  // the policy's name (if, not ?: between literals of different lengths)
  function automatic string pol_name();
    string s;
    if (ctrl_pol) s = "SQRT";
    else          s = "PROP";
    return s;
  endfunction

  // until the first cycle of the next epoch (a rising edge)
  task automatic to_tick();
    int unsigned t;
    t = ep_t0(ep_of(cyc) + 1);
    while (cyc + 1 < t) @(posedge clk);
  endtask

  // traffic for n whole epochs from the next tick; the run stops at a
  // deadlock. Returns the first epoch of the phase
  task automatic ct_phase(string what, int n, output int m0);
    bit ok;
    to_tick();
    m0 = ep_of(cyc + 1);
    traffic_pkg::gen_on = 1'b1;
    u_rc.wait_cycles(n * EPOCH, ok);
    `TB_CHECK(ok, ("%s: deadlock, no packet delivered since cycle %0d", what, sb_pkg::last_delivery))
    stop_if(ok, what);
  endtask

  // generators off, drain, idle check
  task automatic ct_drain(string what);
    bit ok;
    traffic_pkg::gen_on = 1'b0;
    u_rc.drain(DRAIN_MAX, ok);
    `TB_CHECK(ok, ("%s: the mesh did not drain", what))
    stop_if(ok, what);
    check_idle(what);
  endtask

  // the plans of epochs a..b, for messages
  function automatic string plans(int a, int b);
    string s;
    s = "";
    for (int m = a; m <= b; m++) begin
      if (m > a) s = {s, " "};
      s = {s, $sformatf("%0d", plan_of(m))};
    end
    return s;
  endfunction

  // ---------------------------------------------------------------- CT01
  task automatic ct01();
    int              m0, k_prev, n_sw, d_min, d_max, d, lim, li;
    longint          lf;
    int unsigned     t_end, ph;
    tb_pkg::phase("CT01 epoch timing: uniform traffic for 6 epochs; every k_L switch at a frame start, at least A_DELAY after its tick");
    traffic_defaults();
    traffic_pkg::pattern = PAT_UNIFORM;
    traffic_pkg::rate    = 0.4 * sat_rate();
    to_tick();
    m0     = ep_of(cyc + 1);
    t_end  = ep_t0(m0 + 6);
    k_prev = cur_kl();
    n_sw   = 0;
    d_min  = EPOCH;
    d_max  = 0;
    traffic_pkg::gen_on = 1'b1;
    while (cyc + 1 < t_end) begin
      @(posedge clk);
      if (cur_kl() != k_prev) begin
        // the frame that just started carries the new k_L
        ph  = (cyc - c_rst) % EPOCH;
        d   = int'(ph);
        lf  = flen_c(k_prev);
        li  = int'(lf);
        lim = sched_pkg::A_DELAY - 1 + li;
        `TB_CHECK(d >= sched_pkg::A_DELAY && d <= lim && p_st[0].lc == 0,
                  ("CT01: k_L %0d -> %0d in cycle %0d of its epoch, expected a frame start in %0d..%0d", k_prev, cur_kl(), d,
                   sched_pkg::A_DELAY, lim))
        if (d < d_min) d_min = d;
        if (d > d_max) d_max = d;
        n_sw++;
        k_prev = cur_kl();
      end
    end
    ct_drain("CT01");
    tb_pkg::info($sformatf("CT01: plans of epochs %0d..%0d: %s; %0d switches, %0d..%0d cycles after the tick; every tile had each new plan by cycle %0d (bound %0d)",
                           m0, m0 + 5, plans(m0, m0 + 5), n_sw, d_min, d_max, cchk_pkg::the_cchk.arr_max,
                           sched_ref_pkg::ref_plan_by(MESH_N)));
    tb_pkg::cover_bin("CT01 k_L switches", longint'(n_sw));
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- CT02
  task automatic ct02();
    int     m0, m1, m2, m3, np, k1, k1n, k2, k2n;
    longint nreg;
    np = quick() ? 3 : 4;
    tb_pkg::phase($sformatf("CT02 the controller follows demand: uniform, region (p_local 0.75), uniform; %0d epochs each", np));
    traffic_defaults();
    traffic_pkg::rate    = 0.4 * sat_rate();
    traffic_pkg::pattern = PAT_UNIFORM;
    ct_phase("CT02 uniform", np, m0);
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.75;
    ct_phase("CT02 region", np, m1);
    traffic_pkg::pattern = PAT_UNIFORM;
    ct_phase("CT02 uniform again", np, m2);
    m3 = m2 + np;
    ct_drain("CT02");
    k1  = plan_of(m1);
    k1n = plan_of(m1 + 1);
    k2  = plan_of(m2);
    k2n = plan_of(m2 + 1);
    tb_pkg::info($sformatf("CT02 (%s, damping %0d): plans uniform %s | region %s | uniform %s", pol_name(), ctrl_damp,
                           plans(m0, m1 - 1), plans(m1, m2 - 1), plans(m2, m3 - 1)));
    `TB_CHECK(k1n > k1, ("CT02: k_L did not rise in the first epoch planned from region traffic: plan %0d, then %0d", k1, k1n))
    `TB_CHECK(k2n < k2, ("CT02: k_L did not fall in the first epoch planned from uniform traffic again: plan %0d, then %0d", k2, k2n))
    k1   = m2 - m1;
    nreg = longint'(k1);
    tb_pkg::cover_bin("CT02 epochs planned from region traffic", nreg);
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- CT03
  task automatic ct03();
    int  m0, m1, ns, lo, hi, lo2, hi2, k;
    bit  d0;
    ns = quick() ? 6 : 10;
    tb_pkg::phase($sformatf("CT03 the loop is stable: region traffic (p_local 0.5) for %0d epochs with damping, then 4 without", ns));
    d0 = ctrl_damp;
    traffic_defaults();
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.5;
    traffic_pkg::rate    = 0.4 * sat_rate();
    ctrl_damp = 1'b1;
    ct_phase("CT03 damped", ns, m0);
    ctrl_damp = 1'b0;
    ct_phase("CT03 undamped", 4, m1);
    ct_drain("CT03");
    ctrl_damp = d0;
    // settled: the plans of epochs m0 + 4 .. m1 - 1 (the controller reads
    // the damping input when it sets each epoch's plan)
    lo = 99;
    hi = -1;
    for (int m = m0 + 4; m < m1; m++) begin
      k = plan_of(m);
      if (k < lo) lo = k;
      if (k > hi) hi = k;
    end
    lo2 = 99;
    hi2 = -1;
    for (int m = m1; m < m1 + 4; m++) begin
      k = plan_of(m);
      if (k >= 0 && k < lo2) lo2 = k;
      if (k > hi2) hi2 = k;
    end
    tb_pkg::info($sformatf("CT03 (%s): plans with damping %s (settled %0d..%0d); without %s (%0d..%0d)", pol_name(),
                           plans(m0, m1 - 1), lo, hi, plans(m1, m1 + 3), lo2, hi2));
    `TB_CHECK(hi >= 0 && hi - lo <= 2, ("CT03: with damping k_L ranged %0d..%0d after settling, more than +-1", lo, hi))
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- CT04
  task automatic ct04();
    int          m0, m1, m2, ne, k_end, mx, q_end, k_flr, k_top;
    longint      cg_max;
    int unsigned t;
    bit          ok;
    ne = quick() ? 5 : 6;
    tb_pkg::phase($sformatf("CT04 extremes: traffic only inside regions (%0d epochs), every sink stopped, traffic only across regions (%0d epochs)", ne, ne));
    traffic_defaults();
    // a) only inside the regions: no cross-region demand, k_L to K_MAX
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 1.0;
    traffic_pkg::rate    = 0.4 * sat_rate();
    ct_phase("CT04 inside regions", ne, m0);
    ct_drain("CT04 inside regions");
    cg_max = 0;
    for (int m = m0 + 1; m < m0 + ne; m++) begin
      if (m < cchk_pkg::the_cchk.sgq.size() && cchk_pkg::the_cchk.sgq[m] > cg_max) cg_max = cchk_pkg::the_cchk.sgq[m];
    end
    k_end = plan_of(m0 + ne - 1);
    tb_pkg::info($sformatf("CT04 a) inside regions: plans %s; cross-region demand %0d", plans(m0, m0 + ne - 1), cg_max));
    `TB_CHECK(cg_max == 0, ("CT04: cross-region demand %0d with traffic only inside the regions", cg_max))
    `TB_CHECK(k_end == CT_KMAX, ("CT04: k_L %0d after %0d epochs of traffic inside the regions, expected K_MAX %0d", k_end, ne, CT_KMAX))
    // b) every sink stopped for 1,500 cycles (under run_ctrl's deadlock
    // limit) while every source offers traffic at saturation
    traffic_pkg::pattern = PAT_UNIFORM;
    traffic_pkg::rate    = sat_rate();
    to_tick();
    m1 = ep_of(cyc + 1);
    t  = cyc + 1;
    traffic_pkg::fs_node  = FS_ALL;
    traffic_pkg::fs_start = t;
    traffic_pkg::fs_end   = t + 1500;
    traffic_pkg::gen_on   = 1'b1;
    u_rc.wait_cycles(EPOCH, ok);
    `TB_CHECK(ok, ("CT04 sinks stopped: deadlock, no packet delivered since cycle %0d", sb_pkg::last_delivery))
    stop_if(ok, "CT04 sinks stopped");
    ct_drain("CT04 sinks stopped");
    traffic_pkg::fs_node = FS_NONE;
    // the epoch with the stop is latched at the next tick: wait for its plan
    to_tick();
    repeat (sched_pkg::A_DELAY + 1) @(posedge clk);
    mx = (m1 + 1 < cchk_pkg::the_cchk.mxq.size()) ? cchk_pkg::the_cchk.mxq[m1 + 1] : -1;
    tb_pkg::info($sformatf("CT04 b) sinks stopped: largest sum of one tile %0d (14 bits saturate at %0d), mesh-wide %0d + %0d",
                           mx, cchk_pkg::DMMAX, (m1 + 1 < cchk_pkg::the_cchk.slq.size()) ? cchk_pkg::the_cchk.slq[m1 + 1] : 0,
                           (m1 + 1 < cchk_pkg::the_cchk.sgq.size()) ? cchk_pkg::the_cchk.sgq[m1 + 1] : 0));
    tb_pkg::cover_bin("CT04 epoch with every sink stopped, sums checked", (mx > 0) ? 1 : 0);
    tb_pkg::cover_goal("CT04 a tile's 14-bit sum saturated", cchk_pkg::the_cchk.n_sat);
    // c) only across regions: k_L to its floor, the table's entry for the
    // last epoch's share bin, at most max(K_MIN, 1)
    traffic_defaults();
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.0;
    traffic_pkg::rate    = 0.4 * sat_rate();
    ct_phase("CT04 across regions", ne, m2);
    ct_drain("CT04 across regions");
    k_end = plan_of(m2 + ne - 1);
    q_end = (m2 + ne - 1 < cchk_pkg::the_cchk.qq.size()) ? cchk_pkg::the_cchk.qq[m2 + ne - 1] : -1;
    k_flr = sched_ref_pkg::ref_kl(cchk_pkg::the_cchk.pol, MESH_N, REGION_N, q_end, K_LO, K_HI);
    k_top = (K_LO > 1) ? K_LO : 1;
    tb_pkg::info($sformatf("CT04 c) across regions: plans %s; share bin of the last epoch %0d, the table's entry %0d",
                           plans(m2, m2 + ne - 1), q_end, k_flr));
    `TB_CHECK(q_end >= 0 && k_end == k_flr && k_flr <= k_top,
              ("CT04: k_L %0d after %0d epochs of traffic across regions (share bin %0d), expected the table's entry %0d, at most %0d (K_MIN %0d)",
               k_end, ne, q_end, k_flr, k_top, K_LO))
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- CT05
  task automatic ct05();
    int          m0, tries, lim, k, gx_max, gx_min, gx_n, gl_max, gl_min, gl_n;
    longint      gr, nb, lgx_max, lgx_min, lgx_n;
    int unsigned t_w0, t_w1, nw;
    bit          ok;
    gr = flen_c(CT_KMAX);
    nb = longint'(BNS);
    gr = gr * nb;
    tb_pkg::phase($sformatf("CT05 the bound at K_MAX: traffic inside regions until k_L = %0d, then two global rounds (%0d cycles each)",
                            CT_KMAX, gr));
    traffic_defaults();
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 1.0;
    traffic_pkg::rate    = 0.4 * sat_rate();
    // until the frames run K_MAX (at most 8 epochs)
    tries = 0;
    lim   = 8 * EPOCH;
    k     = cur_kl();
    traffic_pkg::gen_on = 1'b1;
    while (k != CT_KMAX && tries < lim) begin
      @(posedge clk);
      tries++;
      k = cur_kl();
    end
    `TB_CHECK(k == CT_KMAX, ("CT05: k_L is %0d after 8 epochs of traffic inside the regions, expected %0d", k, CT_KMAX))
    smon_pkg::the_smon.clear_gaps();
    t_w0 = cyc;
    if (quick()) nw = int'(gr + gr / 5);
    else         nw = int'(2 * gr + gr / 20);
    u_rc.wait_cycles(nw, ok);
    `TB_CHECK(ok, ("CT05: deadlock, no packet delivered since cycle %0d", sb_pkg::last_delivery))
    stop_if(ok, "CT05");
    t_w1 = cyc;
    smon_pkg::the_smon.gaps(1'b0, gx_max, gx_min, gx_n);
    smon_pkg::the_smon.gaps(1'b1, gl_max, gl_min, gl_n);
    ct_drain("CT05");
    m0 = ep_of(t_w0);
    tb_pkg::info($sformatf("CT05: plans of epochs %0d..%0d: %s; across regions turns every %0d..%0d cycles (%0d pairs), bound %0d; inside regions every %0d..%0d",
                           m0, ep_of(t_w1), plans(m0, ep_of(t_w1)), gx_min, gx_max, gx_n, gr, gl_min, gl_max));
    lgx_max = longint'(gx_max);
    lgx_min = longint'(gx_min);
    lgx_n   = longint'(gx_n);
    if (CNR > 1) begin
      `TB_CHECK(gx_n > 0 && lgx_max <= gr, ("CT05: a cross-region pair waited %0d cycles between turns, bound %0d", gx_max, gr))
      `TB_CHECK(lgx_max == gr && lgx_min == gr,
                ("CT05: cross-region turns %0d..%0d cycles apart, expected exactly one global round at K_MAX, %0d", gx_min, gx_max, gr))
      tb_pkg::cover_bin("CT05 cross-region pairs measured at K_MAX", lgx_n);
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- summary
  function automatic void ct_summary();
    if (cchk_pkg::the_cchk == null) return;
    tl_summary();
    tb_pkg::info($sformatf("design D run: %0d epochs checked, %0d plan changes, k_L %0d..%0d; every tile had each new plan by cycle %0d (bound %0d); %0d saturated tile sums",
                           cchk_pkg::the_cchk.n_epochs, cchk_pkg::the_cchk.n_changed, cchk_pkg::the_cchk.k_lo,
                           cchk_pkg::the_cchk.k_hi, cchk_pkg::the_cchk.arr_max, sched_ref_pkg::ref_plan_by(MESH_N),
                           cchk_pkg::the_cchk.n_sat));
  endfunction

  /* verilator lint_on UNUSEDSIGNAL */
