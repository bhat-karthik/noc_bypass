// =============================================================================
// File      tb/mesh/tests_stress.svh                               Tracker: F76
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The B8 stress tests as tasks of mesh_bench, run by +TEST on every
//           mesh top: S01 long runs, S02 sink stalls with bypass, S03 one VC
//           near saturation, S05 no unknown values after reset. S04 (random
//           configurations) and S06 (simulators agree) are lists that run
//           these and PERF (sim/lists/s04.list, sim/sweeps/S06.sweep).
// Decisions D59 stress tests
// Tests     S01 S02 S03 S05 (tb_mesh, tb_mesh_b, tb_mesh_c, tb_mesh_d)
// -----------------------------------------------------------------------------
// S01  10,000,000 cycles (+S01_CYCLES=<n>; +QUICK 200,000) at half of
//      sat_rate(), the pattern switched every 50,000 cycles (+S01_SEG=<n>;
//      +QUICK 10,000)
//      through uniform, transpose, bit complement, bit rotation, shuffle,
//      tornado, neighbour, hotspot and region traffic; every checker on.
//      Then the mesh drains and must be idle: no packet left, every VC idle,
//      every credit home (no leak).
// S02  designs B-D: sinks that stall with chance 1%, 5%, 10% and 20% per
//      cycle, 20,000 cycles each (+QUICK 3,000) at half of sat_rate(): no
//      reservation is lost and no ejection FIFO overflows (ni_checker, the
//      FIFO's own check), every packet arrives. Each level prints its
//      decisions, cancels and cancel rate as a CSV line ("S02,...").
// S03  designs B-D with one VC (-define TB_NUM_VC=1): minimal adaptive
//      routing at 0.9 of sat_rate() with uniform, transpose and hotspot
//      traffic, 20,000 cycles each (+QUICK 4,000): the deadlock watchdog
//      stays silent and the mesh drains (bypass breaks every cycle of
//      waiting packets, as in BP08).
// S05  every design: every cycle of 2,000 cycles of traffic (+QUICK 1,000)
//      and its drain, no X or Z on any mesh link (flits, credits,
//      lookaheads, cancel tokens: every valid bit, and every field of a
//      valid one; design D's demand sums and plans) or NI output (the
//      handshakes and free counts; every field of a valid flit). Needs a
//      four-state simulator (xsim, Riviera-PRO, VCS): on a two-state one
//      such as Verilator it only proves the checks run.
// =============================================================================

  // ---------------------------------------------------------------- S01
  task automatic s01();
    int unsigned total, seg, done, len;
    int          k, np;
    bit          ok, ok2;
    real         rf;
    pattern_e    pl [9];
    pl[0] = PAT_UNIFORM;  pl[1] = PAT_TRANSPOSE; pl[2] = PAT_BITCOMP;  pl[3] = PAT_BITROT;  pl[4] = PAT_SHUFFLE;
    pl[5] = PAT_TORNADO;  pl[6] = PAT_NEIGHBOR;  pl[7] = PAT_HOTSPOT;  pl[8] = PAT_REGION;
    np    = 9;
    total = 10000000;
    seg   = 50000;
    if (quick()) begin
      total = 200000;
      seg   = 10000;
    end
    if ($value$plusargs("S01_CYCLES=%d", total)) begin end
    if ($value$plusargs("S01_SEG=%d", seg)) begin end
    if (!$value$plusargs("S01_RATE_FRAC=%f", rf)) rf = 0.5;
    tb_pkg::phase($sformatf("S01: %0d cycles at %0.2f of sat_rate(), pattern switched every %0d cycles", total, rf, seg));
    traffic_defaults();
    traffic_pkg::rate = rf * sat_rate();
    stats_pkg::reset(NN);
    stats_pkg::open_window(cyc + 1, cyc + total + 1);
    traffic_pkg::gen_on = 1'b1;
    ok   = 1'b1;
    done = 0;
    k    = 0;
    while (done < total && ok) begin
      traffic_pkg::pattern = pl[k % np];
      len = seg;
      if (total - done < len) len = total - done;
      u_rc.wait_cycles(len, ok);
      done = done + len;
      k++;
    end
    traffic_pkg::gen_on = 1'b0;
    ok2 = 1'b0;
    if (ok) u_rc.drain(DRAIN_MAX, ok2);
    `TB_CHECK(ok && ok2, ("S01: the mesh stopped delivering after %0d cycles (%0d segments): %s", done, k, why_stuck(0, DRAIN_MAX)))
    stop_if(ok && ok2, "S01");
    report("S01");
    tb_pkg::info($sformatf("S01: %0d segments of %0d cycles, %0d packets delivered, bypass share %0.2f%%",
                           k, seg, stats_pkg::delivered_pkts(), 100.0 * stats_pkg::byp_pkt()));
    tb_pkg::cover_bin("S01 segments", longint'(k));
    check_idle("S01");
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- S02
  task automatic s02();
    real             lv [4];
    int unsigned     len;
    bit              ok;
    perf_pkg::snap_t pa, pb;
    longint unsigned dec, rcx, acc, trn, cxl;
    real             fa, fb, cr;
    if (DESIGN == DESIGN_A) begin
      tb_pkg::fail("S02 runs on designs B-D (tb_mesh_b, tb_mesh_c, tb_mesh_d)");
      return;
    end
    lv[0] = 0.01; lv[1] = 0.05; lv[2] = 0.10; lv[3] = 0.20;
    len = 20000;
    if (quick()) len = 3000;
    tb_pkg::csv("S02,design,N,V,stall,decisions,accepted,dst_cancels,cancel_rate,turns,src_cancels,lat_avg,pkts");
    for (int i = 0; i < 4; i++) begin
      tb_pkg::phase($sformatf("S02: sinks stall with chance %0.2f per cycle, %0d cycles at half of sat_rate()", lv[i], len));
      traffic_defaults();
      traffic_pkg::rate         = 0.5 * sat_rate();
      traffic_pkg::sink_stall_p = lv[i];
      stats_pkg::reset(NN);
      pa = perf_snap();
      u_rc.run_load(500, len, DRAIN_MAX, ok);
      pb = perf_snap();
      `TB_CHECK(ok, ("S02: stall %0.2f: %s", lv[i], why_stuck(0, DRAIN_MAX)))
      stop_if(ok, "S02");
      dec = pb.dec - pa.dec;
      acc = pb.acc - pa.acc;
      rcx = pb.rcx - pa.rcx;
      trn = pb.turns - pa.turns;
      cxl = pb.cxl - pa.cxl;
      cr  = 0.0;
      fa  = real'(rcx);
      fb  = real'(dec);
      if (dec > 0) cr = fa / fb;
      tb_pkg::csv($sformatf("S02,%s,%0d,%0d,%0.2f,%0d,%0d,%0d,%0.4f,%0d,%0d,%0.3f,%0d", design_name(), MESH_N, NUM_VC, lv[i],
                            dec, acc, rcx, cr, trn, cxl, stats_pkg::lat_avg(), stats_pkg::delivered_pkts()));
      `TB_CHECK(stats_pkg::m_left == 0, ("S02: stall %0.2f: %0d measured packets never delivered", lv[i], stats_pkg::m_left))
      traffic_pkg::sink_stall_p = 0.0;
      check_idle($sformatf("S02 stall %0.2f", lv[i]));
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- S03
  task automatic s03();
    pattern_e    pl [3];
    int unsigned len;
    bit          ok;
    int          st0;
    if (DESIGN == DESIGN_A || NUM_VC != 1) begin
      tb_pkg::fail("S03 runs on designs B-D with one VC: -define TB_NUM_VC=1 on tb_mesh_b, tb_mesh_c or tb_mesh_d");
      return;
    end
    pl[0] = PAT_UNIFORM; pl[1] = PAT_TRANSPOSE; pl[2] = PAT_HOTSPOT;
    len = 20000;
    if (quick()) len = 4000;
    for (int i = 0; i < 3; i++) begin
      tb_pkg::phase($sformatf("S03: one VC, %s traffic at 0.9 of sat_rate() for %0d cycles", pat_name(pl[i]), len));
      traffic_defaults();
      traffic_pkg::pattern = pl[i];
      traffic_pkg::rate    = 0.9 * sat_rate();
      stats_pkg::reset(NN);
      st0 = u_rc.stalls;
      u_rc.run_load(500, len, DRAIN_MAX, ok);
      `TB_CHECK(ok && u_rc.stalls == st0, ("S03: %s: the watchdog fired or the mesh did not drain: %s", pat_name(pl[i]), why_stuck(st0, DRAIN_MAX)))
      stop_if(ok, "S03");
      report($sformatf("S03 %s", pat_name(pl[i])));
      check_idle($sformatf("S03 %s", pat_name(pl[i])));
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- S05
  // the X check itself: every falling edge while s05_on (set and cleared at
  // rising edges by s05, read here at falling edges: no race, rule 10).
  // A valid bit must always be known; a record's other fields only while its
  // valid bit says it carries something. Like buffer storage, the data
  // fields of the RTL's output registers have no reset: they hold X until
  // first written, and a link that never carries anything, such as a port at
  // the mesh's edge, keeps them so. The first xsim run (R-1114-02 and 04)
  // checked whole records and failed from cycle 6 on every design
  function automatic bit s05_x_flit(flit_t f);
    if ($isunknown(f.vld)) return 1'b1;
    if (f.vld == 1'b1) return $isunknown(f);
    return 1'b0;
  endfunction
  function automatic bit s05_x_cred(credit_t c);
    if ($isunknown(c.vld)) return 1'b1;
    if (c.vld == 1'b1) return $isunknown(c);
    return 1'b0;
  endfunction
  function automatic bit s05_x_la(la_t l);
    if ($isunknown(l.vld)) return 1'b1;
    if (l.vld == 1'b1) return $isunknown(l);
    return 1'b0;
  endfunction
  function automatic bit s05_x_cx(cx_t x);
    if ($isunknown(x.vld)) return 1'b1;
    if (x.vld == 1'b1) return $isunknown(x);
    return 1'b0;
  endfunction

  bit s05_on = 1'b0;
  always @(negedge clk) begin : s05_chk
    flit_t   f;
    credit_t c;
    la_t     l;
    cx_t     x;
    bit      bad_ej, bad_fc, bad_lx;
    if (s05_on) begin
      for (int r = 0; r < NN; r++) begin
        // the NI's ejection flit counts while ej_valid is 1; its monitor copy
        // while its own valid bit is
        bad_ej = 1'b0;
        if (ej_valid[r] == 1'b1) begin
          f = ej_flit[r];
          if ($isunknown(f)) bad_ej = 1'b1;
        end
        f = ej_mon[r];
        if (s05_x_flit(f)) bad_ej = 1'b1;
        bad_fc = 1'b0;
        bad_lx = 1'b0;
        for (int p = 0; p < 4; p++) begin
          f = dut.t_out_flit[r][p];
          c = dut.t_out_cred[r][p];
          l = dut.t_out_la[r][p];
          x = dut.t_out_cx[r][p];
          if (s05_x_flit(f) || s05_x_cred(c)) bad_fc = 1'b1;
          if (s05_x_la(l) || s05_x_cx(x)) bad_lx = 1'b1;
        end
        `TB_CHECK(!$isunknown(inj_ready[r]) && !$isunknown(ej_valid[r]) && !$isunknown(ej_free[r]),
                  ("S05: cycle %0d: an unknown handshake or free count at NI %0d", cyc, r))
        `TB_CHECK(!bad_ej, ("S05: cycle %0d: an unknown field in a valid flit at NI %0d's outputs", cyc, r))
        `TB_CHECK(!bad_fc, ("S05: cycle %0d: an unknown valid bit, or field of a valid flit or credit, on a link of tile %0d", cyc, r))
        `TB_CHECK(!bad_lx, ("S05: cycle %0d: an unknown valid bit, or field of a valid lookahead or cancel token, on a link of tile %0d", cyc, r))
        `TB_CHECK(!$isunknown(dut.t_sum_o[r]) && !$isunknown(dut.t_plan_o[r]),
                  ("S05: cycle %0d: an unknown demand sum or plan on tile %0d's controller links", cyc, r))
      end
    end
  end

  task automatic s05();
    int unsigned len;
    bit          ok;
    len = 2000;
    if (quick()) len = 1000;
    tb_pkg::phase($sformatf("S05: no X or Z on any link or NI output, %0d cycles of traffic", len));
    traffic_defaults();
    traffic_pkg::rate = 0.5 * sat_rate();
    stats_pkg::reset(NN);
    s05_on = 1'b1;
    u_rc.run_load(200, len, DRAIN_MAX, ok);
    `TB_CHECK(ok, ("S05: %s", why_stuck(0, DRAIN_MAX)))
    stop_if(ok, "S05");
    repeat (20) @(posedge clk);
    s05_on = 1'b0;
    check_idle("S05");
    traffic_defaults();
  endtask
