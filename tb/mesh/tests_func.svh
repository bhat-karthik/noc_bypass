// =============================================================================
// File      tb/mesh/tests_func.svh                                 Tracker: F30
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The functional mesh tests N01-N08 as tasks of mesh_bench, which
//           includes this file inside its module body.
// Decisions D32 latency accounting, D33 traffic, D34 run control, D35 benches
// Tests     N01-N08
// -----------------------------------------------------------------------------
// Every packet of every test is checked by the sinks (destination, signature,
// data, flit order, exactly once, latency not below zero-load) and every load
// phase ends with the idle check of mesh_bench. Each test adds its own checks:
//   N01  one packet per ordered pair, all at once; all delivered
//   N02  periodic 1-flit traffic at 0.002 packets/node/cycle: average latency
//        within 1 cycle of the zero-load latency; 15/13/13 on 4x4
//   N03  10 patterns x 3 loads x N03_REPS repetitions, mixed packets
//   N04  1.2 x the saturation load is flagged saturated and still drains;
//        half of it is not flagged
//   N05  (tb_mesh_dl) adaptive routing with 1 VC deadlocks: detected, frozen,
//        time to deadlock recorded
//   N06  10% random sink stalls, one sink stopped, then every sink stopped:
//        nothing lost, back-pressure reaches every source (also on design
//        B, +TEST=N06: the stopped sinks make their NIs cancel bursts)
//   N07  data-only traffic: flits of interleaved packets never mixed
//   N08  1,000,000 cycles (+N08_CYCLES) of uniform traffic at half the
//        saturation load, then drain: every VC idle and every credit home
// Loads are fractions of the uniform saturation load sat_rate() (mixed
// packets, measured in B2 with PERF; see sim/README.md).
// +QUICK shortens every test to a few thousand cycles on 4x4, for EDA
// Playground's 60-second limit (its Riviera-PRO is the slower EDU edition):
// N02 2 periods, N03 1 repetition of 100 cycles, N04 250 + 500 cycles,
// N06 steps of 300 cycles, N07 300 cycles, N08 2,000 cycles. Explicit
// +N03_REPS and +N08_CYCLES still win. The checks are the same.
// =============================================================================

  localparam int unsigned DRAIN_MAX  = 200000;  // cycles allowed to drain one phase
  localparam int unsigned N02_PERIOD = 500;     // N02: one packet per node per period
  localparam int unsigned N06_STEP   = 1000;    // N06: cycles of each step (300 with +QUICK)

  // +QUICK: the short forms for EDA Playground (header)
  function automatic bit quick();
    return $test$plusargs("QUICK") != 0;
  endfunction

  // Uniform traffic, mixed packets (2.33 flits a packet): the most the mesh
  // accepts, packets/node/cycle. Measured with PERF on design A, XY, at loads
  // past the knee of the latency curve (B2). Accepted flits/node/cycle:
  //            2x2     4x4     8x8
  //   4 VCs   0.775   0.602   0.319
  //   1 VC    0.238   0.167   0.087
  // With one VC a packet holds its VC until its tail leaves the next router,
  // and a blocked head stalls its whole input port.
  function automatic real sat_rate();
    real r;
    if (NUM_VC == 1) begin
      case (MESH_N)
        2:       r = 0.102;
        4:       r = 0.0716;
        8:       r = 0.0371;
        default: r = 0.30 / real'(MESH_N);      // not measured
      endcase
    end else begin
      case (MESH_N)
        2:       r = 0.332;
        4:       r = 0.257;
        8:       r = 0.137;
        default: r = 1.1 / real'(MESH_N);       // not measured
      endcase
      if (NUM_VC < 4) r = r * real'(NUM_VC) / 4.0;   // 2 or 3 VCs: not measured
    end
    return r;
  endfunction

  // Cycles of generation per load phase, so every phase creates about the
  // same number of packets whatever the mesh size
  function automatic int unsigned scaled(int unsigned base);
    int unsigned c;
    c = base * 16 / NN;
    return (c < 200) ? 200 : c;
  endfunction

  function automatic real rabs(real x);
    return (x < 0.0) ? -x : x;
  endfunction

  // The node in the middle of the mesh
  function automatic int unsigned centre();
    return (MESH_N / 2) * MESH_N + MESH_N / 2;
  endfunction

  // Stop at the first phase that deadlocked or did not drain: the packets
  // still in flight would make every later phase fail the same way
  task automatic stop_if(bit ok, string what);
    if (!ok) begin
      tb_pkg::fail($sformatf("%s: stopping the run, the network did not empty", what));
      tb_pkg::finish();
    end
  endtask

  // Why a phase did not empty: a deadlock (stalls went up) or a slow drain.
  // Strings are chosen with if, not ?: between literals of different lengths,
  // which pads the shorter one and prints differently on each simulator.
  function automatic string why_stuck(int stalls0, int unsigned dmax);
    string s;
    if (u_rc.stalls != stalls0) s = $sformatf("deadlock, no packet delivered since cycle %0d", sb_pkg::last_delivery);
    else                        s = $sformatf("not drained after %0d cycles", dmax);
    return s;
  endfunction

  // One load phase: generators on for warm + meas cycles (the window is the
  // last meas cycles), then drain; then the checks every phase gets
  task automatic load_phase(string what, int unsigned warm, int unsigned meas);
    bit ok;
    int stalls0;
    stalls0 = u_rc.stalls;
    stats_pkg::reset(NN);
    u_rc.run_load(warm, meas, DRAIN_MAX, ok);
    `TB_CHECK(ok, ("%s: %s", what, why_stuck(stalls0, DRAIN_MAX)))
    stop_if(ok, what);
    `TB_CHECK(stats_pkg::m_left == 0, ("%s: %0d measured packets never delivered", what, stats_pkg::m_left))
    check_idle(what);
  endtask

  function automatic void report(string what);
    string sat;
    sat = "";
    if (stats_pkg::saturated()) sat = ", saturated";
    tb_pkg::info($sformatf("%s: %0d packets measured, latency avg %0.2f p99 %0d max %0d, offered %0.4f accepted %0.4f flits/node/cycle%s",
                           what, stats_pkg::delivered_pkts(), stats_pkg::lat_avg(), stats_pkg::lat_pct(0.99),
                           stats_pkg::lat_max, stats_pkg::thr_off(), stats_pkg::thr_acc(), sat));
  endfunction

  // ---------------------------------------------------------------- N01
  // Every node sends one packet to every node, itself included, all at the
  // same moment and each in its own random order; mixed packet lengths
  task automatic n01();
    bit              ok;
    longint unsigned d0;
    tb_pkg::phase($sformatf("N01 all pairs: %0d x %0d nodes, one packet per ordered pair, all at once",
                            NN, NN));
    traffic_defaults();
    stats_pkg::reset(NN);
    d0 = sb_pkg::n_delivered;
    traffic_pkg::burst_id++;
    @(posedge clk);                     // the generators queue the burst at the falling edge
    `TB_CHECK(sb_pkg::outstanding() == longint'(NN * NN),
              ("N01: %0d packets queued, expected %0d", sb_pkg::outstanding(), NN * NN))
    u_rc.drain(DRAIN_MAX, ok);
    `TB_CHECK(ok, ("N01: the burst did not drain (deadlock or more than %0d cycles)", DRAIN_MAX))
    stop_if(ok, "N01");
    `TB_CHECK(sb_pkg::n_delivered - d0 == longint'(NN * NN),
              ("N01: %0d packets delivered, expected %0d", sb_pkg::n_delivered - d0, NN * NN))
    tb_pkg::info($sformatf("N01: %0d packets delivered by cycle %0d", sb_pkg::n_delivered - d0, cyc));
    check_idle("N01");
  endtask

  // ---------------------------------------------------------------- N02
  // Zero load: one 1-flit packet per node every 500 cycles (0.002
  // packets/node/cycle) at a random phase. Every source sends the same number
  // of packets, so their average zero-load latency is exactly
  // 4 (mean hops + 1) + 1 (D32): 15, 13 and 13 cycles on 4x4.
  task automatic n02();
    pattern_e    pats   [3];
    real         target [3];
    int unsigned k_per;
    bit          ok;
    real         exp_avg;
    pats   = '{PAT_TRANSPOSE, PAT_BITROT, PAT_SHUFFLE};
    target = '{15.0, 13.0, 13.0};
    k_per  = (400 + NN - 1) / NN;       // periods, about 400 packets per pattern
    if (k_per < 4) k_per = 4;
    if (quick()) k_per = 2;
    tb_pkg::phase($sformatf("N02 zero-load latency: 1 packet/node every %0d cycles, 1-flit, %0d periods",
                            N02_PERIOD, k_per));
    foreach (pats[k]) begin
      traffic_defaults();
      traffic_pkg::pattern = pats[k];
      traffic_pkg::p_data  = 0.0;
      traffic_pkg::period  = int'(N02_PERIOD);
      traffic_pkg::per_t0  = cyc + 1;   // the first falling edge from now
      exp_avg = real'(zero_load(0, 1)) + 4.0 * mean_hops(pats[k], MESH_N);
      if (MESH_N == 4) begin
        `TB_CHECK(rabs(exp_avg - target[k]) < 1.0e-9,
                  ("N02 %s: expected average %0.3f, the tracker says %0.1f", pat_name(pats[k]), exp_avg, target[k]))
      end
      stats_pkg::reset(NN);
      stats_pkg::open_window(cyc + 1, cyc + 1 + k_per * N02_PERIOD);
      traffic_pkg::gen_on = 1'b1;
      u_rc.wait_cycles(k_per * N02_PERIOD, ok);
      traffic_pkg::gen_on = 1'b0;
      if (ok) u_rc.drain(DRAIN_MAX, ok);
      `TB_CHECK(ok, ("N02 %s: did not drain", pat_name(pats[k])))
      stop_if(ok, "N02");
      `TB_CHECK(stats_pkg::m_pkts == longint'(k_per * NN) && stats_pkg::m_left == 0,
                ("N02 %s: %0d packets created and %0d delivered, expected %0d", pat_name(pats[k]),
                 stats_pkg::m_pkts, stats_pkg::delivered_pkts(), k_per * NN))
      // the packets sent have exactly the pattern's mean hop count
      `TB_CHECK(rabs(stats_pkg::zl_avg() - exp_avg) < 1.0e-9,
                ("N02 %s: zero-load average of the packets sent %0.4f, expected %0.4f",
                 pat_name(pats[k]), stats_pkg::zl_avg(), exp_avg))
      // contention adds less than a cycle on average, and most packets see none
      `TB_CHECK(stats_pkg::lat_avg() - exp_avg <= 1.0,
                ("N02 %s: average latency %0.3f, expected %0.3f within 1 cycle", pat_name(pats[k]),
                 stats_pkg::lat_avg(), exp_avg))
      `TB_CHECK(stats_pkg::zl_share() >= 0.9,
                ("N02 %s: only %0.1f%% of packets at exactly the zero-load latency", pat_name(pats[k]),
                 100.0 * stats_pkg::zl_share()))
      tb_pkg::info($sformatf("N02 %s: average latency %0.3f cycles, expected %0.3f; %0.1f%% of %0d packets at exactly zero-load",
                             pat_name(pats[k]), stats_pkg::lat_avg(), exp_avg, 100.0 * stats_pkg::zl_share(),
                             stats_pkg::delivered_pkts()));
      check_idle($sformatf("N02 %s", pat_name(pats[k])));
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- N03
  // Every pattern at three loads, N03_REPS times each; the random streams
  // continue from one repetition to the next, so each one draws new traffic
  task automatic n03();
    real             loads [3];
    real             s_r;
    int              reps;
    int unsigned     gen;
    longint unsigned p0, pk;
    longint unsigned lmax;
    if (!$value$plusargs("N03_REPS=%d", reps)) reps = quick() ? 1 : 5;
    s_r   = sat_rate();
    loads = '{0.25 * s_r, 0.5 * s_r, 0.9 * s_r};
    gen   = quick() ? 100 : scaled(1000);
    tb_pkg::phase($sformatf("N03 random traffic: %0d patterns x 3 loads (%0.3f %0.3f %0.3f packets/node/cycle) x %0d, %0d cycles each",
                            NUM_PATTERNS, loads[0], loads[1], loads[2], reps, gen));
    for (int k = 0; k < NUM_PATTERNS; k++) begin
      p0   = sb_pkg::n_delivered;
      lmax = 0;
      for (int l = 0; l < 3; l++) begin
        for (int r = 0; r < reps; r++) begin
          traffic_defaults();
          traffic_pkg::pattern   = pattern_e'(k);
          traffic_pkg::rate      = loads[l];
          traffic_pkg::phase_len = int'(gen / 4);   // both phases of PHASE in every run
          load_phase($sformatf("N03 %s load %0d run %0d", pat_name(pattern_e'(k)), l + 1, r + 1), 0, gen);
          if (stats_pkg::lat_max > lmax) lmax = stats_pkg::lat_max;
        end
      end
      pk = sb_pkg::n_delivered - p0;
      tb_pkg::cover_bin($sformatf("N03 %s packets delivered", pat_name(pattern_e'(k))), pk);
      tb_pkg::info($sformatf("N03 %s: %0d packets, worst latency %0d cycles", pat_name(pattern_e'(k)), pk, lmax));
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- N04
  // Above saturation the source queues grow without limit: the detector
  // must flag it, and every measured packet must still arrive. Half the
  // saturation load is the control: not flagged.
  task automatic n04();
    int unsigned meas, warm;
    meas = quick() ? 500 : scaled(2000);
    warm = meas / 2;
    tb_pkg::phase($sformatf("N04 saturation: uniform at 0.5 and 1.2 x %0.3f packets/node/cycle, %0d + %0d cycles",
                            sat_rate(), warm, meas));
    traffic_defaults();
    traffic_pkg::rate = 0.5 * sat_rate();
    load_phase("N04 0.5 x saturation", warm, meas);
    report("N04 0.5 x saturation");
    `TB_CHECK(!stats_pkg::saturated(), ("N04: 0.5 x saturation flagged saturated (accepted %0.4f of offered %0.4f)",
                                        stats_pkg::thr_acc(), stats_pkg::thr_off()))
    traffic_defaults();
    traffic_pkg::rate = 1.2 * sat_rate();
    load_phase("N04 1.2 x saturation", warm, meas);
    report("N04 1.2 x saturation");
    `TB_CHECK(stats_pkg::saturated(), ("N04: 1.2 x saturation not flagged (accepted %0.4f of offered %0.4f)",
                                       stats_pkg::thr_acc(), stats_pkg::thr_off()))
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- N05
  // Design A with minimal adaptive routing and one VC has cyclic channel
  // dependencies. Under load it must deadlock: the detector of run_ctrl
  // fires (no delivery for dl_limit cycles while packets are in flight), and
  // the mesh is really frozen: nothing delivered and no flit on any link for
  // a further dl_limit cycles. It is a routing deadlock, not a lost credit:
  // the waiting heads form a cycle, each waiting for the input VC that the
  // next one holds. The run ends there; the idle check is skipped.

  // With one VC: follow "waits for" from every waiting head. A walk longer
  // than the number of input ports has entered a cycle; walk the cycle once
  // more to name it. Returns the cycle length (0: none) and its path.
  // A head waiting for Local waits only for its sink, which never stalls in
  // N05, so it ends a walk.
  function automatic bit waits(int t, int p);
    if (t < 0 || t >= NN || p < 0 || p >= NPORTS) return 1'b0;
    return in_wait[t][p] && in_port[t][p] != P_L;
  endfunction

  function automatic int wait_cycle(output string path);
    int t, p, nt, np, ct, cp, steps, len;
    path = "";
    for (int t0 = 0; t0 < NN; t0++) begin
      for (int p0 = 0; p0 < NPORTS; p0++) begin
        t     = t0;
        p     = p0;
        steps = 0;
        while (steps <= NN * NPORTS && waits(t, p)) begin
          nt = int'(nbr_id(node_id_t'(t), in_port[t][p], MESH_N));
          np = int'(opposite(in_port[t][p]));
          t  = nt;
          p  = np;
          steps++;
        end
        if (steps > NN * NPORTS) begin
          ct  = t;
          cp  = p;
          len = 0;
          do begin
            path = {path, $sformatf(" R%0d.%s", t, tb_pkg::port_name(port_t'(p)))};
            nt   = int'(nbr_id(node_id_t'(t), in_port[t][p], MESH_N));
            np   = int'(opposite(in_port[t][p]));
            t    = nt;
            p    = np;
            len++;
          end while (!(t == ct && p == cp) && len <= NN * NPORTS);
          return len;
        end
      end
    end
    return 0;
  endfunction

  task automatic n05();
    bit              ok;
    int unsigned     t0, lim, moves, t_dl;
    longint unsigned d0;
    real             r5;
    int              clen;
    string           cpath;
    lim = 200000;
    if (!$value$plusargs("N05_RATE=%f", r5)) r5 = 0.6;
    tb_pkg::phase($sformatf("N05 deadlock: adaptive routing, %0d VC, uniform at %0.3f packets/node/cycle, up to %0d cycles",
                            NUM_VC, r5, lim));
    traffic_defaults();
    traffic_pkg::rate = r5;
    stats_pkg::reset(NN);
    t0 = cyc;
    traffic_pkg::gen_on = 1'b1;
    u_rc.wait_cycles(lim, ok);
    traffic_pkg::gen_on = 1'b0;
    `TB_CHECK(!ok && u_rc.stalls == 1, ("N05: no deadlock in %0d cycles", lim))
    if (ok) return;
    tb_pkg::info($sformatf("N05: deadlock detected at cycle %0d; last delivery at cycle %0d, %0d cycles after the load started",
                           cyc, u_rc.t_stall, u_rc.t_stall - t0));
    tb_pkg::info($sformatf("N05: %0d packets delivered before it; %0d created and not delivered, most still in source queues",
                           sb_pkg::n_delivered, sb_pkg::outstanding()));
    // frozen: no delivery and no flit moving for another dl_limit cycles
    d0    = sb_pkg::n_delivered;
    moves = 0;
    repeat (u_rc.dl_limit) begin
      @(posedge clk);
      if (|link_busy) moves++;
    end
    `TB_CHECK(sb_pkg::n_delivered == d0, ("N05: %0d packets delivered after the deadlock", sb_pkg::n_delivered - d0))
    `TB_CHECK(moves == 0, ("N05: flits still moved in %0d cycles after the deadlock", moves))
    `TB_CHECK(!(&tile_idle), ("N05: deadlock reported but every router is empty"))
    if (NUM_VC == 1) begin
      clen = wait_cycle(cpath);
      `TB_CHECK(clen >= 2, ("N05: the mesh is frozen but no cycle of waiting packets exists (a lost credit?)"))
      tb_pkg::info($sformatf("N05: %0d packets wait in a cycle, each for the input VC the next one holds:%s",
                             clen, cpath));
    end
    t_dl = u_rc.t_stall - t0;
    tb_pkg::cover_bin("N05 cycles to deadlock", longint'(t_dl));
  endtask

  // ---------------------------------------------------------------- N06
  // Uniform traffic at half the saturation load with 10% random sink stalls,
  // in three steps:
  //   1  N06_STEP cycles of random stalls only
  //   2  N06_STEP cycles with the centre node's sink stopped: its router's
  //      Local output VCs all fill, and the other sinks keep receiving
  //   3  every sink stops, and every source offers a 1-flit packet every
  //      cycle for C + 100 cycles, C = the VCs in the mesh. Every packet in
  //      the mesh holds at least one VC (a router input VC, or the Local
  //      output VC whose credits its NI has not returned), so the mesh holds
  //      at most C packets. A source never blocked in step 3 would have put
  //      C + 100 packets in by itself: impossible while flow control works.
  //      So every source must be blocked, and at most C packets are inside.
  // Then the sinks restart and every packet must arrive.
  task automatic n06();
    int unsigned     m, t3, dl0, step;
    bit              ok;
    longint unsigned b0 [NN];
    longint unsigned n_blk, d_other, in_flits, in_pkts, cap_f, cap_p;
    int              vcs;
    int              stalls0;
    m     = centre();
    step  = quick() ? 300 : N06_STEP;
    // VCs of every input port a link or an NI feeds, plus every NI's share
    // of its router's Local output VCs; a VC holds VC_DEPTH flits
    vcs   = (NN + 4 * MESH_N * (MESH_N - 1)) * NUM_VC + NN * NUM_VC;
    cap_p = longint'(vcs);
    cap_f = longint'(vcs * VC_DEPTH);
    // design B: a bypass packet in an NI FIFO holds no VC, only bypass space
    if (DESIGN != DESIGN_A) begin
      cap_p += longint'(NN * SPACE);
      cap_f += longint'(NN * SPACE);
    end
    t3    = int'(vcs) + 100;
    tb_pkg::phase($sformatf("N06 back-pressure: uniform at %0.3f, 10%% sink stalls; node %0d stops for %0d cycles, then every node for %0d",
                            0.5 * sat_rate(), m, step, t3));
    traffic_defaults();
    traffic_pkg::rate         = 0.5 * sat_rate();
    traffic_pkg::sink_stall_p = 0.1;
    stats_pkg::reset(NN);
    stalls0 = u_rc.stalls;
    stats_pkg::open_window(cyc + 1, cyc + 1 + 2 * step);
    traffic_pkg::gen_on = 1'b1;
    // step 1
    u_rc.wait_cycles(step, ok);
    stop_if(ok, "N06 step 1");
    // step 2: the centre sink stops
    traffic_pkg::fs_node  = int'(m);
    traffic_pkg::fs_start = cyc + 1;
    traffic_pkg::fs_end   = cyc + 1 + step;
    d_other = sb_pkg::n_delivered;
    u_rc.wait_cycles(step, ok);
    stop_if(ok, "N06 step 2");
    `TB_CHECK(loc_full[m], ("N06: node %0d stopped for %0d cycles but its router's Local output VCs are not all busy", m, step))
    // the others kept receiving (the deadlock detector would also stop the run)
    tb_pkg::cover_bin("N06 packets delivered while one sink was stopped", sb_pkg::n_delivered - d_other);
    // step 3: every sink stops; every source offers a 1-flit packet a cycle.
    // No delivery for t3 cycles is expected here, so the deadlock limit is
    // raised above t3 for this step.
    foreach (b0[i]) b0[i] = v_blocked[i];
    traffic_pkg::rate     = 1.0;
    traffic_pkg::p_data   = 0.0;
    traffic_pkg::fs_node  = FS_ALL;
    traffic_pkg::fs_start = cyc + 1;
    traffic_pkg::fs_end   = cyc + 1 + t3;
    dl0 = u_rc.dl_limit;
    if (u_rc.dl_limit < t3 + 500) u_rc.dl_limit = t3 + 500;
    u_rc.wait_cycles(t3, ok);
    u_rc.dl_limit = dl0;
    stop_if(ok, "N06 step 3");
    n_blk = 0;
    for (int i = 0; i < NN; i++) begin
      `TB_CHECK(v_blocked[i] > b0[i],
                ("N06: every sink stopped for %0d cycles, but node %0d's source was never blocked", t3, i))
      n_blk += v_blocked[i] - b0[i];
    end
    in_pkts  = sum_heads() - sum_pkts();
    in_flits = sum_sent() - sum_taken();
    `TB_CHECK(in_pkts <= cap_p, ("N06: %0d packets inside the mesh, more than its %0d VCs can hold", in_pkts, cap_p))
    `TB_CHECK(in_flits <= cap_f, ("N06: %0d flits inside the mesh, more than its %0d buffer slots", in_flits, cap_f))
    tb_pkg::info($sformatf("N06: every sink stopped: %0d packets (%0d flits) inside the mesh, at most %0d fit (one per VC); every source blocked, %0d cycles in all",
                           in_pkts, in_flits, cap_p, n_blk));
    tb_pkg::cover_bin("N06 source cycles blocked while every sink was stopped", n_blk);
    // restart the sinks and drain
    traffic_pkg::gen_on = 1'b0;
    u_rc.drain(DRAIN_MAX, ok);
    `TB_CHECK(ok, ("N06: did not drain after the sinks restarted"))
    stop_if(ok, "N06");
    `TB_CHECK(u_rc.stalls == stalls0 && stats_pkg::m_left == 0,
              ("N06: %0d measured packets never delivered", stats_pkg::m_left))
    tb_pkg::cover_bin("N06 sink stall cycles", sum_stall());
    report("N06 steps 1-2");
    check_idle("N06");
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- N07
  // Only 5-flit packets. Packets on different VCs interleave flit by flit in
  // the NI FIFO; the sink rebuilds each from its VC and checks every flit's
  // index and data, so a flit out of order or in the wrong packet fails.
  // With one VC packets cannot interleave, and the test checks that none did.
  task automatic n07();
    pattern_e        pats [3];
    longint unsigned i0;
    int unsigned     meas;
    pats = '{PAT_UNIFORM, PAT_BITCOMP, PAT_HOTSPOT};
    meas = quick() ? 300 : scaled(1500);
    tb_pkg::phase($sformatf("N07 data packets only: uniform, bitcomp, hotspot at %0.3f packets/node/cycle, %0d cycles each",
                            0.3 * sat_rate(), meas));
    i0 = sum_inter();
    foreach (pats[k]) begin
      traffic_defaults();
      traffic_pkg::pattern = pats[k];
      traffic_pkg::p_data  = 1.0;
      traffic_pkg::rate    = 0.3 * sat_rate();
      load_phase($sformatf("N07 %s", pat_name(pats[k])), 0, meas);
      report($sformatf("N07 %s", pat_name(pats[k])));
    end
    // With 2+ VCs the router's Local output interleaves packets, and that
    // must have happened; with one VC the next packet cannot start before
    // the last tail has left, so none may interleave
    if (NUM_VC > 1) begin
      tb_pkg::cover_bin("N07 packets interleaved at a sink", sum_inter() - i0);
    end else begin
      `TB_CHECK(sum_inter() == i0, ("N07: %0d packets interleaved at a sink with one VC", sum_inter() - i0))
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- N08
  // A long run at half the saturation load, then drain: afterwards no flit
  // anywhere, every VC idle and every credit home (check_idle)
  task automatic n08();
    int unsigned n8, done, step;
    bit          ok;
    int          stalls0;
    if (!$value$plusargs("N08_CYCLES=%d", n8)) n8 = quick() ? 2000 : 1000000;
    if (n8 < 2000) n8 = 2000;
    tb_pkg::phase($sformatf("N08 endurance: %0d cycles, uniform at %0.3f packets/node/cycle, then drain",
                            n8, 0.5 * sat_rate()));
    traffic_defaults();
    traffic_pkg::rate = 0.5 * sat_rate();
    stats_pkg::reset(NN);
    stalls0 = u_rc.stalls;
    stats_pkg::open_window(cyc + 1001, cyc + 1 + n8);
    traffic_pkg::gen_on = 1'b1;
    done = 0;
    ok   = 1'b1;
    while (done < n8 && ok) begin
      step = (n8 - done < 100000) ? n8 - done : 100000;
      u_rc.wait_cycles(step, ok);
      done += step;
      if (done % 100000 == 0 && done < n8)
        tb_pkg::info($sformatf("N08: %0d of %0d cycles, %0d packets delivered", done, n8, sb_pkg::n_delivered));
    end
    traffic_pkg::gen_on = 1'b0;
    if (ok) u_rc.drain(DRAIN_MAX, ok);
    `TB_CHECK(ok, ("N08: %s", why_stuck(stalls0, DRAIN_MAX)))
    stop_if(ok, "N08");
    `TB_CHECK(stats_pkg::m_left == 0, ("N08: %0d measured packets never delivered", stats_pkg::m_left))
    `TB_CHECK(!stats_pkg::saturated(), ("N08: half the saturation load flagged saturated"))
    report("N08");
    check_idle("N08");
    traffic_defaults();
  endtask
