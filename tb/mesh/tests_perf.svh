// =============================================================================
// File      tb/mesh/tests_perf.svh                                 Tracker: F31
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   PERF: one performance point of design A, B, C or D, set by
//           plusargs, with its CSV row (Section 8). sweep.tcl (F60) runs it
//           once per point of an experiment.
// Decisions D33 traffic, D34 run control, D48 reservation, D58 the CSV
// Tests     PERF (runs only when +TEST=PERF, or TEST_PERF in the bundle)
// -----------------------------------------------------------------------------
// Plusargs (defaults in brackets)
//   +PATTERN=<name>   uniform transpose bitcomp bitrot shuffle tornado
//                     neighbor hotspot region phase                [uniform]
//   +RATE=<real>      packets/node/cycle                 [half of sat_rate()]
//   +RATE_FRAC=<real> the rate as a share of sat_rate() (B8; +RATE wins)
//   +PDATA=<real>     share of 5-flit packets                         [0.333]
//   +PLOCAL=<real>    region: share of packets inside the source's region [0.5]
//   +TREGION=<n>      region: region side [REGION_N in C, D; N/2 in A, B]
//   +HOTFRAC=<real>   hotspot: share of packets to the hotspot          [0.2]
//   +PHASE_A=<name> +PHASE_B=<name> +PHASE_LEN=<n>
//                     phase: the two patterns and their period
//                                               [uniform, transpose, 500]
//   +WARMUP=<n>       cycles before the window      [2000; design D 5 epochs]
//   +MEASURE=<n>      cycles in the window                            [10000]
//   +DRAIN=<n>        most cycles to drain                           [200000]
//   +STALL=<real>     chance per cycle that a sink stalls                 [0]
//   +TRACE=<n>        also a TRACE line every n cycles of warm-up and window
//                     (E6): cycle, packets delivered in those n cycles, their
//                     mean latency, tile 0's k_L, its local-phase share   [0]
//   +RUNID=<text>     first CSV column [<design>_N<n>_V<v>_<pattern>]
//   (design D also takes +POLICY=SQRT and +DAMP=0, read before the test)
// Output: INFO lines, then two CSV lines (header and row), also written to
// +CSV=<file>. The cycles column is the length of this point: warm-up,
// window and drain. Every packet is checked as in N01-N08. Designs B-D add
// the bypass and reservation columns of perf_probe (F49), counted over the
// whole point; designs C and D add k_L and phase columns (M15) and the
// turn intervals sched_monitor measured in the point (M12):
//   k_mean     k_L of the running frame, averaged over every cycle of the point
//   k_min, k_max, k_changes   over the plans of the point's epochs
//   share_L    share of the point's cycles in the local phase (tile 0)
//   turn_max_L, turn_max_G    longest time between two turns of one pair
//              inside a region and across regions (-1: no pair measured)
//   kL_trace   the plan of every epoch of the point, ';'-separated (M15, M16)
// =============================================================================

  bit pf_trace_on = 1'b0;                      // perf_trace runs while it is set

  task automatic perf();
    string       pat_s, run_id, s, pol_s, ep_s, pa_s, pb_s, tr_s;
    pattern_e    pat, pq;
    real         rt, rf, pdata, stall, plocal, hfrac, f, fa, fb;
    int unsigned warm, meas, dmax, c0, trn, plen;
    int          treg, m0, m1, k, kmn, kmx, kch, kprev, gl_max, gl_min, gl_n, gg_max, gg_min, gg_n;
    bit          ok;
    int          stalls0;
    string       dsg;
    longint unsigned q_cyc, q_kl, q_loc;
    perf_pkg::snap_t pa, pb;
    dsg = "B";
    if (DESIGN == DESIGN_A) dsg = "A";
    if (DESIGN == DESIGN_C) dsg = "C";
    if (DESIGN == DESIGN_D) dsg = "D";
    if (!$value$plusargs("PATTERN=%s", pat_s)) pat_s = "uniform";
    if (!pat_parse(pat_s, pat)) begin
      tb_pkg::fail($sformatf("PERF: unknown pattern '%s'", pat_s));
      return;
    end
    if (!$value$plusargs("RATE=%f", rt)) begin
      if ($value$plusargs("RATE_FRAC=%f", rf)) rt = rf * sat_rate();
      else                                     rt = 0.5 * sat_rate();
    end
    if (!$value$plusargs("PDATA=%f", pdata))   pdata = 1.0 / 3.0;
    if (!$value$plusargs("STALL=%f", stall))   stall = 0.0;
    // design D: warm up for five epochs, so that k_L has settled when the
    // window opens (damping reaches any target on 8x8 within four, D60)
    if (!$value$plusargs("WARMUP=%d", warm)) begin
      warm = 2000;
      if (DESIGN == DESIGN_D) warm = 5 * EPOCH;
    end
    if (!$value$plusargs("MEASURE=%d", meas))  meas  = 10000;
    if (!$value$plusargs("DRAIN=%d", dmax))    dmax  = DRAIN_MAX;
    if (!$value$plusargs("TRACE=%d", trn))     trn   = 0;
    if (!$value$plusargs("RUNID=%s", run_id))  run_id = $sformatf("%s_N%0d_V%0d_%s", dsg, MESH_N, NUM_VC, pat_s);
    traffic_defaults();
    if ($value$plusargs("PLOCAL=%f", plocal))  traffic_pkg::p_local  = plocal;
    if ($value$plusargs("TREGION=%d", treg))   traffic_pkg::region_n = treg;
    if ($value$plusargs("HOTFRAC=%f", hfrac))  traffic_pkg::hot_frac = hfrac;
    if ($value$plusargs("PHASE_LEN=%d", plen)) traffic_pkg::phase_len = plen;
    if ($value$plusargs("PHASE_A=%s", pa_s)) begin
      if (pat_parse(pa_s, pq)) traffic_pkg::phase_a = pq;
      else tb_pkg::fail($sformatf("PERF: unknown pattern '%s'", pa_s));
    end
    if ($value$plusargs("PHASE_B=%s", pb_s)) begin
      if (pat_parse(pb_s, pq)) traffic_pkg::phase_b = pq;
      else tb_pkg::fail($sformatf("PERF: unknown pattern '%s'", pb_s));
    end
    // the configuration columns policy and epoch (D58)
    pol_s = "-";
    ep_s  = "-";
    if (TWO) ep_s = $sformatf("%0d", EPOCH);
    if (DESIGN == DESIGN_C) pol_s = $sformatf("fixed%0d", K_INIT);
    if (DESIGN == DESIGN_D) begin
      if (ctrl_pol) pol_s = "SQRT";
      else          pol_s = "PROP";
      if (!ctrl_damp) pol_s = {pol_s, "_nodamp"};
      if (K_LO != sched_pkg::K_MIN || K_HI != sched_pkg::k_max(MESH_N)) pol_s = $sformatf("%s_k%0d-%0d", pol_s, K_LO, K_HI);
    end
    tb_pkg::phase($sformatf("PERF %s: %s at %0.4f packets/node/cycle, p_data %0.3f, stall %0.3f, %0d + %0d cycles",
                            run_id, pat_s, rt, pdata, stall, warm, meas));
    traffic_pkg::pattern      = pat;
    traffic_pkg::rate         = rt;
    traffic_pkg::p_data       = pdata;
    traffic_pkg::sink_stall_p = stall;
    stats_pkg::reset(NN);
    stalls0 = u_rc.stalls;
    c0      = cyc;
    pa      = perf_snap();
    q_cyc   = pp_cyc;
    q_kl    = pp_kl;
    q_loc   = pp_loc;
    if (TWO) smon_pkg::the_smon.clear_gaps();
    if (trn > 0) begin
      $display("TRACE,run_id,cycle,pkts,lat_avg,k_L,share_L");
      pf_trace_on = 1'b1;
      fork
        begin
          u_rc.run_load(warm, meas, dmax, ok);
          pf_trace_on = 1'b0;
        end
        perf_trace(run_id, trn, c0);
      join
    end else begin
      u_rc.run_load(warm, meas, dmax, ok);
    end
    pb      = perf_snap();
    `TB_CHECK(ok, ("PERF: %s", why_stuck(stalls0, dmax)))
    stop_if(ok, "PERF");
    `TB_CHECK(stats_pkg::m_left == 0, ("PERF: %0d measured packets never delivered", stats_pkg::m_left))
    report("PERF");
    tb_pkg::info($sformatf("PERF: zero-load latency of the measured packets %0.2f, jain %0.4f, worst flow %0.2f",
                           stats_pkg::zl_avg(), stats_pkg::jain(), stats_pkg::lat_flow_max()));
    // one header for every design (D58): the columns a design lacks are "-"
    tb_pkg::csv({stats_pkg::csv_header(), ",", perf_pkg::header(), ",", perf_pkg::header_two()});
    if (DESIGN == DESIGN_A) begin
      tb_pkg::csv({stats_pkg::csv_row(run_id, dsg, MESH_N, NUM_VC, pat_s, pdata, rt, tb_pkg::seed,
                                      u_rc.stalls, u_rc.t_stall, cyc - c0),
                   ",", perf_pkg::row_na(), ",", perf_pkg::row_two_na()});
    end else if (!TWO) begin
      tb_pkg::info($sformatf("PERF: bypass share %0.2f%% of packets; %0d bursts in %0d turns, %0d turns cancelled",
                             100.0 * stats_pkg::byp_pkt(), pb.used - pa.used, pb.turns - pa.turns, pb.cxl - pa.cxl));
      tb_pkg::csv({stats_pkg::csv_row(run_id, dsg, MESH_N, NUM_VC, pat_s, pdata, rt, tb_pkg::seed,
                                      u_rc.stalls, u_rc.t_stall, cyc - c0, $sformatf("%0d", B_MAX), tmode_name()),
                   ",", perf_pkg::row(pa, pb), ",", perf_pkg::row_two_na()});
    end else begin
      // k_L over the point: the plan of every epoch from the first to the last
      m0  = ep_of(c0);
      m1  = ep_of(cyc);
      kmn = 1000;
      kmx = -1;
      kch = 0;
      kprev = -1;
      tr_s = "";
      for (int m = m0; m <= m1; m++) begin
        k = K_INIT;
        if (DESIGN == DESIGN_D) k = plan_of(m);
        if (k < 0) continue;
        if (k < kmn) kmn = k;
        if (k > kmx) kmx = k;
        if (kprev >= 0 && k != kprev) kch++;
        kprev = k;
        s = $sformatf("%0d", k);
        if (tr_s == "") tr_s = s;
        else            tr_s = {tr_s, ";", s};
      end
      smon_pkg::the_smon.gaps(1'b1, gl_max, gl_min, gl_n);
      smon_pkg::the_smon.gaps(1'b0, gg_max, gg_min, gg_n);
      if (gl_n == 0) gl_max = -1;
      if (gg_n == 0) gg_max = -1;
      q_cyc = pp_cyc - q_cyc;
      q_kl  = pp_kl - q_kl;
      q_loc = pp_loc - q_loc;
      tb_pkg::info($sformatf("PERF: bypass share %0.2f%% of packets; %0d bursts in %0d turns, %0d turns cancelled; k_L %0d..%0d, %0d changes",
                             100.0 * stats_pkg::byp_pkt(), pb.used - pa.used, pb.turns - pa.turns, pb.cxl - pa.cxl, kmn, kmx, kch));
      tb_pkg::info($sformatf("PERF: turns of one pair inside a region every %0d..%0d cycles (%0d pairs), across regions every %0d..%0d (%0d pairs)",
                             gl_min, gl_max, gl_n, gg_min, gg_max, gg_n));
      // (each cast into a variable first: README rule 9)
      fb = real'(q_cyc);
      f  = 0.0;
      fa = real'(q_kl);
      if (q_cyc > 0) f = fa / fb;
      s  = $sformatf("%0.3f,%0d,%0d,%0d", f, kmn, kmx, kch);
      f  = 0.0;
      fa = real'(q_loc);
      if (q_cyc > 0) f = fa / fb;
      s = $sformatf("%s,%0.4f,%0d,%0d,%s", s, f, gl_max, gg_max, tr_s);
      tb_pkg::csv({stats_pkg::csv_row(run_id, dsg, MESH_N, NUM_VC, pat_s, pdata, rt, tb_pkg::seed,
                                      u_rc.stalls, u_rc.t_stall, cyc - c0, $sformatf("%0d", B_MAX), tmode_name(),
                                      pol_s, ep_s),
                   ",", perf_pkg::row(pa, pb), ",", s});
    end
    check_idle("PERF");
    traffic_defaults();
  endtask

  // PERF's time series (+TRACE=n): one line every n cycles until the load
  // ends. The packets and latency come from stats_pkg's trace counters
  // (every delivery, window or not); k_L and the phase share from tile 0.
  task automatic perf_trace(string run_id, int unsigned n, int unsigned c0);
    int unsigned     k;
    longint unsigned np, sl, q0c, q0l, dc, dl;
    real             la, sh, fa, fb;
    int              kl;
    k   = 0;
    q0c = pp_cyc;
    q0l = pp_loc;
    stats_pkg::trace_take(np, sl);
    while (pf_trace_on) begin
      @(posedge clk);
      k++;
      if (k == n) begin
        k  = 0;
        stats_pkg::trace_take(np, sl);
        la = 0.0;
        fa = real'(sl);
        fb = real'(np);
        if (np > 0) la = fa / fb;
        dc  = pp_cyc - q0c;
        dl  = pp_loc - q0l;
        q0c = pp_cyc;
        q0l = pp_loc;
        sh  = 0.0;
        fa  = real'(dl);
        fb  = real'(dc);
        if (dc > 0) sh = fa / fb;
        kl  = int'(p_st[0].kl);
        $display("TRACE,%s,%0d,%0d,%0.2f,%0d,%0.4f", run_id, cyc - c0, np, la, kl, sh);
      end
    end
  endtask
