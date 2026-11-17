// =============================================================================
// File      tb/mesh/tests_two.svh                                  Tracker: F71
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The B6 tests of the two-level timetable on a design-C mesh, as
//           tasks of mesh_bench (included in its module body after
//           tests_resv.svh, whose helpers they use; tops tb_mesh_c and
//           tb_mesh_cb).
// Decisions D15 regions, D16 frames, D19 plan switch, D50 one pointer,
//           D51 B6 verification
// Tests     TL01 TL02 TL03 TL04 TL06 (tb_mesh_c), TL05 (tb_mesh_cb)
// -----------------------------------------------------------------------------
// In every design-C run, every cycle: router_checker (F43) checks every
// burst and cancel token against the frames, ni_checker (F47) every
// reservation, sched_monitor (F53) every tile's turns and decisions and the
// pairs of every local and global round, mesh_checker (F52) lockstep, frame
// and boundary rules, region containment and collisions from the
// lookaheads; the sinks check every packet; each test ends with the idle
// check. Each test adds:
//   TL01  regions are isolated: a) in every region, one 5-flit packet for
//         the region's next local turn, and one packet for a cross-region
//         global turn, each held at its source until its turn: each leaves
//         by bypass in its own subslot and lands at t + 2L + 3 + j;
//         b) region traffic (70% inside the region) at 40% of design A's
//         saturation for 6 frames and 4 local rounds (16x16: 3 frames and
//         2 local rounds): no bypass flit, lookahead or token crosses a
//         region boundary in a local subslot (mesh_checker), every local
//         turn stays in its region (sched_monitor), while bursts do cross
//         region boundaries in global slots.
//   TL02  every pair is covered: from a global round's start, one 5-flit
//         packet for every ordered pair (4,032 on 8x8), each injected just
//         before its pair's first turn of that round (local or global) and
//         held at its source: every packet leaves by bypass in exactly that
//         subslot and lands at t + 2L + 3 + j; sched_monitor's round checks
//         (every pair inside a region once per local round, every pair once
//         per global round) pass for at least one whole global round and
//         one local round inside the test.
//   TL03  frames follow the timing rules: plan k_L = 3 on 8x8 (K_FIXED - 1
//         elsewhere), applied at the first frame start after the next
//         sample; then 12 frames with traffic: every frame lasts k_L local
//         slots and one global slot to the cycle, the local slot number
//         goes on from where it stopped and the global one by one
//         (mesh_checker); two directed bursts of B_MAX flits over the
//         longest path of the last subslot before a phase change (local ->
//         global, then global -> local): their last flits land while the
//         other timetable runs, intact and at their D38 cycles.
//   TL04  plan changes are clean: plans 0, K_MAX, 1, 3 in turn, each set at
//         a random point of a random epoch, under traffic with sinks that
//         stall (so reservations, cancels and bursts are in flight at the
//         switches): every tile switches in the same cycle, at the first
//         frame start after the first sample after the plan was set (at
//         least A_DELAY cycles after its epoch tick); afterwards all
//         packets arrive, no reservation is left and the NI model agreed
//         every cycle (no reservation lost).
//   TL05  (tb_mesh_cb) on 4x4 the design is exactly BuffeRS: a design-C mesh
//         with one 4x4 region (D50) and a design-B mesh get the same
//         traffic (region traffic at half of design A's saturation with
//         random sink stalls and every sink stopped for 500 cycles, then
//         BP06's cancels); every NI output and every link equal every
//         cycle. With -define TB_MESH_N=8 the same with k_L = 0 (global
//         slots only).
//   TL06  turn bounds hold: random traffic for one period of the combined
//         schedule (8x8, k_L 4: 180 frames, 108,540 cycles; +TL06_CYCLES
//         sets another length, e.g. 1,000,000): the longest interval
//         between turns of a pair inside a region is at most 10 local slots
//         plus ceil(10 / k_L) global slots (1,651 cycles on 8x8), and every
//         cross-region pair's turns are exactly one global round apart
//         (21,708 cycles); every ordered pair had at least two turns; no
//         turn cancelled while every sink keeps up.
// The TL tests run with the epoch of tb_mesh_c (2^13 cycles on 8x8 and
// larger, D51), so a plan set at any time takes effect within about one
// epoch. Packets are injected and holds and plans changed at a rising edge
// only; the bench applies holds and plans at the falling edge.
// +QUICK shortens TL01 b), TL03, TL05 (a third of the traffic, no BP06
// phase) and TL06 (2 global rounds) for EDA Playground, where the default
// mesh is 4x4 with 2x2 regions; there TL04 runs with epochs of 512 cycles
// (tb_mesh_c).
// =============================================================================

  // (finders return fields the tests do not all read)
  /* verilator lint_off UNUSEDSIGNAL */

  // ---------------------------------------------------------------- timetable (design C)
  localparam int CTL = sched_pkg::t_sub(REGION_N, B_MAX, T_MODE);   // local subslot, cycles
  localparam int CSU = sched_pkg::s_setup(REGION_N);                // local setup window
  localparam int CLS = sched_pkg::slot_len(REGION_N, CTL);          // local slot
  localparam int CLR = sched_pkg::n_slots(REGION_N);                // local slots per round
  localparam int CNR = (MESH_N / REGION_N) * (MESH_N / REGION_N);   // regions
  localparam int TL_LEAD = 30;                                      // inject this long before the turn
  localparam int ADM1    = sched_pkg::A_DELAY - 1;                   // the sample's cycle in an epoch
  localparam int KMX     = sched_pkg::k_max(MESH_N);                // D18's K_MAX

  typedef struct {
    int src;
    int dst;
    int t;          // the subslot's first cycle
    bit loc;
    int slot;
    int sub;
  } turn_t;

  typedef turn_t turn_q_t[$];

  function automatic longint flen_c(int k);
    return longint'(k) * longint'(CLS) + longint'(BSL);
  endfunction

  // Region of router r
  function automatic int reg_c(int r);
    return ((r / MESH_N) / REGION_N) * (MESH_N / REGION_N) + (r % MESH_N) / REGION_N;
  endfunction

  // The tiles' current k_L (all equal: mesh_checker checks lockstep)
  function automatic int cur_kl();
    return int'(p_st[0].kl);
  endfunction

  // Every turn of the cycles [a, b), from the frame reference, in time order
  // (only the plans sampled so far are known: call it with no plan change ahead)
  function automatic turn_q_t turns_in(int unsigned a, int unsigned b);
    turn_q_t                q;
    turn_t                  tn;
    sched_ref_pkg::fpos_t   fp;
    sched_ref_pkg::pos_t    pz;
    sched_ref_pkg::xfer_q_t x;
    q = {};
    for (int unsigned c = a; c < b; c++) begin
      fp = sched_ref_pkg::the_fr.at(longint'(c) - longint'(c_rst));
      if (fp.loc) pz = fp.lp;
      else        pz = fp.gp;
      if (pz.setup || pz.cyc != 0) continue;
      x = sched_ref_pkg::ref_mesh_xfers(MESH_N, REGION_N, fp.loc, pz.slot, pz.sub);
      foreach (x[m]) begin
        tn.src  = x[m].sy * MESH_N + x[m].sx;
        tn.dst  = x[m].dy * MESH_N + x[m].dx;
        tn.t    = int'(c);
        tn.loc  = fp.loc;
        tn.slot = pz.slot;
        tn.sub  = pz.sub;
        q.push_back(tn);
      end
    end
    return q;
  endfunction

  // First cycle at or after c that starts a round of the global timetable
  function automatic int unsigned glob_round_start(int unsigned c);
    sched_ref_pkg::fpos_t fp;
    int unsigned          u;
    u = c;
    fp = sched_ref_pkg::the_fr.at(longint'(u) - longint'(c_rst));
    while (fp.loc || !(fp.gp.setup && fp.gp.cyc == 0 && fp.gp.slot == 0)) begin
      u++;
      fp = sched_ref_pkg::the_fr.at(longint'(u) - longint'(c_rst));
    end
    return u;
  endfunction

  // First cycle at or after c in which a slot starts (of either timetable)
  function automatic int unsigned slot_start(int unsigned c);
    sched_ref_pkg::fpos_t fp;
    sched_ref_pkg::pos_t  pz;
    int unsigned          u;
    u = c;
    while (1) begin
      fp = sched_ref_pkg::the_fr.at(longint'(u) - longint'(c_rst));
      if (fp.loc) pz = fp.lp;
      else        pz = fp.gp;
      if (pz.setup && pz.cyc == 0) break;
      u++;
    end
    return u;
  endfunction

  // First cycle at or after c that starts a frame
  function automatic int unsigned frame_start(int unsigned c);
    return int'(sched_ref_pkg::the_fr.next_frame(longint'(c) - longint'(c_rst)) + longint'(c_rst));
  endfunction

  // The first sample of the plan input at or after cycle c (the end of
  // cycle A_DELAY - 1 of an epoch)
  function automatic int unsigned next_sample(int unsigned c);
    longint e, m;
    e = longint'(c) - longint'(c_rst);
    m = (e - longint'(ADM1) + longint'(EPOCH) - 1) / longint'(EPOCH);
    if (e <= longint'(ADM1)) m = 0;
    return int'(longint'(c_rst) + m * longint'(EPOCH) + longint'(ADM1));
  endfunction

  // Set the plan and wait until every tile runs it; returns the cycle the
  // first frame with it started (the bench applies plan_req at the next
  // falling edge)
  // Set the plan at a rising edge (it reaches the tiles at the falling edge
  // of the next cycle, t_set) and wait until every tile runs it; t_sw is
  // the first cycle of the first frame with it
  task automatic set_plan(string what, int k, output int unsigned t_set, output int unsigned t_sw);
    int unsigned tmax;
    @(posedge clk);
    t_set    = cyc + 1;                // cyc still names the cycle this edge ends
    plan_req = sched_pkg::kl_t'(k);
    tmax     = t_set + 2 * EPOCH + int'(flen_c(63)) + 10;
    do @(negedge clk); while (cur_kl() != k && cyc < tmax);
    t_sw = cyc;
    `TB_CHECK(cur_kl() == k, ("%s: plan %0d reached the tiles in cycle %0d; they still run k_L %0d at cycle %0d", what, k, t_set,
                              cur_kl(), cyc))
  endtask

  // Packets for a list of turns (one per entry): npk packets of 5 flits
  // each (1-flit packets when B_MAX < 5), injected TL_LEAD cycles before the
  // turn, the source's Local input held from then until the turn has gone
  // by; then every one of them must leave by bypass in its own subslot
  task automatic held_turns(string what, turn_q_t q, int npk, output int n_ok);
    int                 L, src, dst, n_exp, flen, n_bad;
    int unsigned        t_end, t_cur, t_rel [NN];
    longint unsigned    u0, c0x;
    traffic_pkg::dlog_t d;
    int                 tq [NN][NN];
    for (int s = 0; s < NN; s++) begin
      t_rel[s] = 0;
      for (int dd = 0; dd < NN; dd++) tq[s][dd] = 0;
    end
    flen = (B_MAX >= MAX_PKT_FLITS) ? MAX_PKT_FLITS : 1;
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    n_exp = 0;
    u0    = sum_of(v_used);
    c0x   = sum_of(v_cxl);
    t_end = 0;
    t_cur = 0;
    foreach (q[m]) begin
      // once per subslot: at_cycle of a cycle already gone waits one more edge
      if (q[m].t > t_cur) begin
        at_cycle(q[m].t - TL_LEAD);
        t_cur = q[m].t;
      end
      src = q[m].src;
      dst = q[m].dst;
      `TB_CHECK(tq[src][dst] == 0, ("%s: R%0d -> R%0d listed twice", what, src, dst))
      tq[src][dst]  = q[m].t;
      hold_req[src] = 5'b10000;
      if (q[m].t + 2 > t_rel[src]) t_rel[src] = q[m].t + 2;
      for (int i = 0; i < npk; i++) traffic_pkg::direct(src, dst, flen);
      n_exp += npk;
      if (q[m].t > t_end) t_end = q[m].t;
      // release the holds of sources whose last turn has gone by
      for (int s = 0; s < NN; s++) if (t_rel[s] != 0 && t_rel[s] + TL_LEAD < cyc) begin
        hold_req[s] = '0;
        t_rel[s]    = 0;
      end
    end
    at_cycle(t_end + 2 * sched_pkg::hmax(MESH_N) + B_MAX + 4);
    for (int s = 0; s < NN; s++) hold_req[s] = '0;
    wait_delivered(what);
    n_ok  = 0;
    n_bad = 0;
    foreach (traffic_pkg::dlog[i]) begin
      d   = traffic_pkg::dlog[i];
      src = int'(d.src);
      dst = int'(d.dst);
      L   = nhops(src, dst);
      if (d.byp && tq[src][dst] != 0 && d.t_tail <= tq[src][dst] + sched_pkg::tc_eject(L, B_MAX - 1) &&
          d.t_tail >= tq[src][dst] + sched_pkg::tc_eject(L, flen - 1)) begin
        n_ok++;
      end else if (n_bad < 10) begin
        n_bad++;
        tb_pkg::info($sformatf("%s: packet %0d.%0d R%0d -> R%0d: bypass %0b, tail at %0d, its turn at %0d", what,
                               d.src, d.seq, d.src, d.dst, d.byp, d.t_tail, tq[src][dst]));
      end
    end
    `TB_CHECK(traffic_pkg::dlog.size() == n_exp && n_ok == n_exp,
              ("%s: %0d packets delivered, %0d of them by bypass in their own turn's burst; expected %0d", what,
               traffic_pkg::dlog.size(), n_ok, n_exp))
    `TB_CHECK(sum_of(v_cxl) - c0x == 0, ("%s: %0d turns cancelled", what, sum_of(v_cxl) - c0x))
    tb_pkg::info($sformatf("%s: %0d packets, %0d bursts, every packet by bypass in its own turn", what, n_exp,
                           sum_of(v_used) - u0));
    traffic_pkg::log_on = 1'b0;
  endtask

  // ---------------------------------------------------------------- TL01
  task automatic tl01();
    turn_q_t         q, all;
    bit              got_reg [CNR];
    bit              got_x;
    int              n_ok, nfr, nlr;
    int unsigned     ncyc;
    longint unsigned lb0 [CNR];
    longint unsigned xf0, ct0, lt0;
    tb_pkg::phase($sformatf("TL01 regions are isolated: %0d regions of %0dx%0d, k_L %0d", CNR, REGION_N, REGION_N, cur_kl()));
    traffic_defaults();
    xf0 = mchk_pkg::the_mchk.n_xflits;
    // local bursts per region over a and b: a's held packets give every
    // region one, b's region traffic adds those it happens to make (the
    // bypass share is low, so b alone may leave a region without one)
    for (int r = 0; r < CNR; r++) lb0[r] = longint'(rchk_pkg::the_chk.lb_reg[r]);
    // a) one held packet per region for its next local turn, and one for a
    //    cross-region turn (a global slot)
    // (two frames hold a local turn in every region and a global slot,
    // whose bursts cross region boundaries)
    all   = turns_in(cyc + 2 * TL_LEAD, cyc + 2 * TL_LEAD + 2 * int'(flen_c(cur_kl())));
    q     = {};
    got_x = 1'b0;
    for (int r = 0; r < CNR; r++) got_reg[r] = 1'b0;
    foreach (all[m]) begin
      if (all[m].loc) begin
        if (!got_reg[reg_c(all[m].src)]) begin
          got_reg[reg_c(all[m].src)] = 1'b1;
          q.push_back(all[m]);
        end
      end else if (!got_x && reg_c(all[m].src) != reg_c(all[m].dst)) begin
        got_x = 1'b1;
        q.push_back(all[m]);
      end
    end
    `TB_CHECK(q.size() == CNR + 1, ("TL01: found %0d held turns, expected one local turn per region (%0d) and one cross-region turn",
                                    q.size(), CNR))
    held_turns("TL01 a", q, 1, n_ok);
    check_idle("TL01 a");
    // b) region traffic
    ct0 = mchk_pkg::the_mchk.n_contain;
    lt0 = smon_pkg::the_smon.n_lturns;
    nfr = (MESH_N >= 16) ? 3 : 6;
    nlr = (MESH_N >= 16) ? 2 : 4;
    ncyc = int'(longint'(nfr) * flen_c(cur_kl())) + nlr * CLR * CLS;
    if (quick()) ncyc = ncyc / 2;
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.7;
    traffic_pkg::rate    = 0.4 * sat_rate();
    tb_pkg::info($sformatf("TL01 b: region traffic, 70%% inside the region, %0.4f packets/node/cycle, %0d cycles", traffic_pkg::rate, ncyc));
    load_phase("TL01 b", 0, ncyc);
    report("TL01 b");
    `TB_CHECK(mchk_pkg::the_mchk.n_contain - ct0 > 0, ("TL01: no local-phase cycle was checked for containment"))
    tb_pkg::info($sformatf("TL01: %0d local-phase cycles checked on every region-crossing link, %0d local turns stayed in their regions, %0d bypass flits crossed a region boundary in global slots",
                           mchk_pkg::the_mchk.n_contain - ct0, smon_pkg::the_smon.n_lturns - lt0, mchk_pkg::the_mchk.n_xflits - xf0));
    tb_pkg::cover_bin("TL01 bursts inside regions in local slots (a)", longint'(n_ok));
    tb_pkg::cover_bin("TL01 bypass flits crossing regions in global slots (a, b)", mchk_pkg::the_mchk.n_xflits - xf0);
    for (int r = 0; r < CNR; r++) begin
      if (r < 4 || r == CNR - 1)
        tb_pkg::cover_bin($sformatf("TL01 local bursts in region %0d (a, b)", r), longint'(rchk_pkg::the_chk.lb_reg[r]) - lb0[r]);
    end
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- TL02
  task automatic tl02();
    turn_q_t         all, q;
    bit              seen [NN][NN];
    int              n_ok, nl, ng;
    int unsigned     r0;
    longint unsigned lr0, gr0;
    longint          gr;
    gr = longint'(BNS) * flen_c(cur_kl());
    r0 = glob_round_start(cyc + 2 * TL_LEAD + 10);
    tb_pkg::phase($sformatf("TL02 every pair is covered: %0d ordered pairs, the global round from cycle %0d (%0d cycles at k_L %0d)",
                            NPAIRS, r0, gr, cur_kl()));
    traffic_defaults();
    lr0 = smon_pkg::the_smon.n_lrounds;
    gr0 = smon_pkg::the_smon.n_grounds;
    all = turns_in(r0, r0 + int'(gr));
    q   = {};
    nl  = 0;
    ng  = 0;
    for (int s = 0; s < NN; s++) for (int d = 0; d < NN; d++) seen[s][d] = 1'b0;
    foreach (all[m]) begin
      if (!seen[all[m].src][all[m].dst]) begin
        seen[all[m].src][all[m].dst] = 1'b1;
        q.push_back(all[m]);
        if (all[m].loc) nl++;
        else            ng++;
      end
    end
    `TB_CHECK(q.size() == NPAIRS, ("TL02: the reference gives %0d ordered pairs a turn in one global round, expected %0d", q.size(), NPAIRS))
    tb_pkg::info($sformatf("TL02: %0d pairs have their first turn in a local slot, %0d in a global slot", nl, ng));
    held_turns("TL02", q, 1, n_ok);
    in_cycle(r0 + int'(gr) + 2);
    `TB_CHECK(smon_pkg::the_smon.n_grounds > gr0 && smon_pkg::the_smon.n_lrounds > lr0,
              ("TL02: the monitor completed %0d global and %0d local rounds during the test", smon_pkg::the_smon.n_grounds - gr0,
               smon_pkg::the_smon.n_lrounds - lr0))
    tb_pkg::info($sformatf("TL02: %0d global and %0d local rounds checked: every pair once per global round, every pair inside a region once per local round",
                           smon_pkg::the_smon.n_grounds - gr0, smon_pkg::the_smon.n_lrounds - lr0));
    check_idle("TL02");
    tb_pkg::cover_bin("TL02 pairs served first by a local turn", longint'(nl));
    tb_pkg::cover_bin("TL02 pairs served first by a global turn", longint'(ng));
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- TL03
  // The last subslot of the frame's last local slot, and of its global
  // slot: the transfer with the longest path in each
  task automatic tl03();
    int              k3, n_ok, nfr, npk, best, L, n_k3;
    int unsigned     t_set, t_sw, f0, tl_last, tg_last;
    string           wh;
    turn_q_t         a, q;
    turn_t           tb;
    longint unsigned xt0, fr0;
    sched_ref_pkg::fpos_t fp;
    k3 = (K_INIT > 1) ? K_INIT - 1 : 2;
    tb_pkg::phase($sformatf("TL03 frames follow the timing rules: k_L = %0d (frames of %0d cycles: %0d local slots of %0d, one global slot of %0d)",
                            k3, flen_c(k3), k3, CLS, BSL));
    traffic_defaults();
    set_plan("TL03", k3, t_set, t_sw);
    tb_pkg::info($sformatf("TL03: plan %0d reached the tiles in cycle %0d, was sampled at the end of cycle %0d, and runs from the frame starting at %0d",
                           k3, t_set, next_sample(t_set), t_sw));
    // directed bursts across both phase changes of the next frame
    f0      = frame_start(cyc + 2 * TL_LEAD + 10);
    tl_last = f0 + (k3 - 1) * CLS + CSU + (REGION_N - 1) * CTL;
    tg_last = f0 + k3 * CLS + BS + (MESH_N - 1) * BT;
    npk     = (B_MAX >= MAX_PKT_FLITS) ? B_MAX / MAX_PKT_FLITS : B_MAX;
    if (npk > NUM_VC) npk = NUM_VC;
    q = {};
    for (int w = 0; w < 2; w++) begin
      if (w == 0) begin
        a  = turns_in(tl_last, tl_last + 1);
        wh = "last local";
      end else begin
        a  = turns_in(tg_last, tg_last + 1);
        wh = "global";
      end
      best = -1;
      L    = -1;
      foreach (a[m]) begin
        if (nhops(a[m].src, a[m].dst) > L) begin
          L    = nhops(a[m].src, a[m].dst);
          best = m;
        end
      end
      `TB_CHECK(best >= 0, ("TL03: no transfer in the last subslot of the %s slot of the frame at cycle %0d", wh, f0))
      if (best >= 0) begin
        tb = a[best];
        q.push_back(tb);
        tb_pkg::info($sformatf("TL03: %0d packets R%0d -> R%0d (%0d hops) for the last subslot of the %s slot, cycle %0d",
                               npk, tb.src, tb.dst, L, wh, tb.t));
      end
    end
    xt0 = rchk_pkg::the_chk.n_xtail;
    held_turns("TL03 boundaries", q, npk, n_ok);
    check_idle("TL03 boundaries");
    // frames under traffic
    nfr = quick() ? 4 : 12;
    fr0 = mchk_pkg::the_mchk.n_frames;
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.5;
    traffic_pkg::rate    = 0.4 * sat_rate();
    load_phase("TL03 frames", 0, int'(longint'(nfr) * flen_c(k3)));
    report("TL03 frames");
    n_k3 = int'(mchk_pkg::the_mchk.n_frames - fr0);
    `TB_CHECK(cur_kl() == k3 && n_k3 >= nfr, ("TL03: %0d frames checked with k_L %0d (now %0d), expected at least %0d", n_k3, k3, cur_kl(), nfr))
    tb_pkg::info($sformatf("TL03: %0d frames of k_L %0d checked: lengths, setup windows, resumed slot numbers", n_k3, k3));
    tb_pkg::cover_bin("TL03 bursts whose last flits land in the other phase", rchk_pkg::the_chk.n_xtail - xt0);
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- TL04
  task automatic tl04();
    int              plans [4];
    int              k, n_sw, res_at, rmax, n_after;
    int unsigned     t_set, t_sw, s_exp, f_exp, wait_c, t_b, t_bs, u;
    longint unsigned cx0, ac0;
    int              res_any, n_late, lim, nq;
    plans   = '{0, KMX, 1, 3};
    res_any = 0;
    tb_pkg::phase($sformatf("TL04 plan changes are clean: k_L %0d -> %0d -> %0d -> %0d -> %0d, each at a random point of an epoch of %0d cycles",
                            cur_kl(), plans[0], plans[1], plans[2], plans[3], EPOCH));
    traffic_defaults();
    traffic_pkg::pattern      = PAT_REGION;
    traffic_pkg::p_local      = 0.5;
    traffic_pkg::rate         = 0.3 * sat_rate();
    traffic_pkg::sink_stall_p = 0.3;
    traffic_pkg::gen_on       = 1'b1;
    cx0     = sum_of(v_rcx);
    ac0     = sum_of(v_acc);
    n_sw    = 0;
    n_after = 0;
    for (int i = 0; i < 4; i++) begin
      k      = plans[i];
      wait_c = tb_pkg::rng_below(SID, EPOCH);
      repeat (wait_c) @(posedge clk);
      set_plan($sformatf("TL04 plan %0d", k), k, t_set, t_sw);
      s_exp  = next_sample(t_set);
      // where it should have started: the first frame start after the
      // first sample after the plan reached the tiles (a falling edge in t_set)
      f_exp = frame_start(s_exp + 1);
      `TB_CHECK(t_sw == f_exp && f_exp - (s_exp - (sched_pkg::A_DELAY - 1)) >= sched_pkg::A_DELAY,
                ("TL04: plan %0d set at cycle %0d, sampled at %0d: the tiles switched at cycle %0d, expected the frame start at %0d",
                 k, t_set, s_exp, t_sw, f_exp))
      // reservations still open from before the switch, then the first
      // slot after it: its decisions reserve with the new phase's lengths
      res_at = sum_res();
      if (res_at > 0) res_any++;
      rmax = 0;
      u    = slot_start(t_sw + 1);
      while (cyc < u) begin
        @(negedge clk);
        if (sum_res() > rmax) rmax = sum_res();
      end
      if (rmax > 0) n_after++;
      n_sw++;
      tb_pkg::info($sformatf("TL04: plan %0d reached the tiles in cycle %0d, sampled at the end of cycle %0d; every tile switched at the frame start %0d; %0d flits reserved then, up to %0d in the first slot after",
                             k, t_set, s_exp, t_sw, res_at, rmax));
    end
    traffic_pkg::gen_on       = 1'b0;
    traffic_pkg::sink_stall_p = 0.0;
    wait_delivered("TL04");
    check_idle("TL04");
    // the last cycle of a slot, with no traffic left: no reservation
    // outlives its slot. A reservation may still be open in this cycle only
    // if its lookahead is due in it (the last subslot's longest path: with
    // B_MAX 5, TIGHT, its window ends exactly with the slot); the RTL holds
    // B_MAX flits for each open reservation and nothing else
    u = slot_start(cyc + 2);
    in_cycle(u - 1);
    n_late = 0;
    lim    = int'(u) - 1;
    for (int r = 0; r < NN; r++) begin
      nq = nchk_pkg::the_nchk.wdue[r].size();
      for (int i = 0; i < nq; i++) if (nchk_pkg::the_nchk.wdue[r][i] > lim) n_late++;
    end
    `TB_CHECK(n_late == 0 && sum_res() == B_MAX * nchk_pkg::the_nchk.waiting(),
              ("TL04: at the end of the slot before cycle %0d, %0d reservations are due after the slot ends; %0d flits reserved for %0d open reservations",
               u, n_late, sum_res(), nchk_pkg::the_nchk.waiting()))
    tb_pkg::info($sformatf("TL04: %0d plan switches, %0d reservations accepted and %0d cancelled meanwhile, none lost",
                           n_sw, sum_of(v_acc) - ac0, sum_of(v_rcx) - cx0));
    tb_pkg::cover_bin("TL04 plan switches with every tile in step", longint'(n_sw));
    tb_pkg::cover_bin("TL04 switches followed by reservations of the new phase", longint'(n_after));
    tb_pkg::cover_goal("TL04 switches with reservations open from before", longint'(res_any));
    tb_pkg::cover_goal("TL04 turns cancelled by a full NI meanwhile", sum_of(v_rcx) - cx0);
    // back to the fixed plan for the tests after this one
    set_plan("TL04 back to K_FIXED", K_INIT, t_b, t_bs);
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- TL05
  task automatic tl05();
    int unsigned     ncyc;
    longint unsigned u0, s0, x0;
    ncyc = (MESH_N <= 4) ? 4000 : 12000;
    if (quick()) ncyc = ncyc / 3;
    if (REGION_N == MESH_N)
      tb_pkg::phase($sformatf("TL05 design C with one %0dx%0d region (D50) against design B, every output and link every cycle, %0d cycles of traffic",
                              REGION_N, REGION_N, ncyc));
    else
      tb_pkg::phase($sformatf("TL05 design C with k_L = %0d against design B, every output and link every cycle, %0d cycles of traffic",
                              K_INIT, ncyc));
    `TB_CHECK(REGION_N == MESH_N || K_INIT == 0, ("TL05: needs one region or k_L = 0 (REGION_N %0d, K_INIT %0d)", REGION_N, K_INIT))
    traffic_defaults();
    traffic_pkg::pattern      = PAT_REGION;
    traffic_pkg::p_local      = 0.5;
    traffic_pkg::rate         = 0.5 * sat_rate();
    traffic_pkg::sink_stall_p = 0.1;
    // and every sink stopped for 500 cycles in the middle (back-pressure)
    traffic_pkg::fs_node      = FS_ALL;
    traffic_pkg::fs_start     = cyc + ncyc / 2;
    traffic_pkg::fs_end       = cyc + ncyc / 2 + 500;
    u0 = sum_of(v_used);
    x0 = sum_of(v_cxl);
    s0 = sh_same;
    load_phase("TL05", 0, ncyc);
    report("TL05");
    traffic_defaults();
    // then BP06's cancels: a stalled sink, bursts that fill its bypass
    // space, cancel tokens to every later source (design B's timetable,
    // which design C runs here); not in the short form
    if (!quick()) bp06();
    `TB_CHECK(!sh_diff && sh_same - s0 > longint'(ncyc),
              ("TL05: the meshes agreed for %0d cycles of the test (differed: %0b)", sh_same - s0, sh_diff))
    tb_pkg::info($sformatf("TL05: design C and design B agreed on every output and link for %0d cycles; %0d bursts, %0d turns cancelled",
                           sh_same - s0, sum_of(v_used) - u0, sum_of(v_cxl) - x0));
    tb_pkg::cover_bin("TL05 bursts in both meshes", sum_of(v_used) - u0);
    if (quick()) tb_pkg::cover_goal("TL05 turns cancelled in both meshes", sum_of(v_cxl) - x0);
    else         tb_pkg::cover_bin("TL05 turns cancelled in both meshes", sum_of(v_cxl) - x0);
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- TL06
  // Frames until the local and global positions repeat: f k = 0 mod CLR and
  // f = 0 mod BNS
  function automatic int sched_period(int k);
    int f;
    f = BNS;
    while ((f * k) % CLR != 0) f += BNS;
    return f;
  endfunction

  task automatic tl06();
    int unsigned     ncyc, t_set, t_sw, per;
    int              k, gl_max, gl_min, gl_n, gx_max, gx_min, gx_n, bound_l, n_all;
    longint          gr;
    perf_pkg::snap_t a, b;
    real             share;
    if (cur_kl() != K_INIT) set_plan("TL06", K_INIT, t_set, t_sw);
    k   = cur_kl();
    gr  = longint'(BNS) * flen_c(k);
    per = sched_period(k) * int'(flen_c(k));
    if (!$value$plusargs("TL06_CYCLES=%d", ncyc)) begin
      ncyc = per;
      if (longint'(ncyc) < 3 * gr) ncyc = int'(3 * gr);
      if (quick()) ncyc = int'(2 * gr);
    end
    bound_l = CLR * CLS + ((CLR + k - 1) / k) * BSL;
    tb_pkg::phase($sformatf("TL06 turn bounds: %0d cycles of traffic (one schedule period is %0d frames = %0d cycles); bounds: inside a region %0d, across regions %0d",
                            ncyc, sched_period(k), per, bound_l, gr));
    traffic_defaults();
    traffic_pkg::pattern = PAT_REGION;
    traffic_pkg::p_local = 0.5;
    traffic_pkg::rate    = 0.5 * sat_rate();
    smon_pkg::the_smon.clear_gaps();
    a = perf_snap();
    load_phase("TL06", 0, ncyc);
    report("TL06");
    b = perf_snap();
    smon_pkg::the_smon.gaps(1'b1, gl_max, gl_min, gl_n);
    smon_pkg::the_smon.gaps(1'b0, gx_max, gx_min, gx_n);
    n_all = gl_n + gx_n;
    `TB_CHECK(gl_max > 0 && gl_max <= bound_l,
              ("TL06: longest interval between turns of a pair inside a region %0d cycles, bound %0d", gl_max, bound_l))
    if (CNR > 1) begin
      `TB_CHECK(longint'(gx_max) == gr && longint'(gx_min) == gr,
                ("TL06: intervals between turns of a cross-region pair from %0d to %0d cycles, expected exactly one global round, %0d",
                 gx_min, gx_max, gr))
    end
    `TB_CHECK(n_all == NPAIRS, ("TL06: %0d of the %0d ordered pairs had two turns or more", n_all, NPAIRS))
    `TB_CHECK(b.cxl - a.cxl == 0, ("TL06: %0d turns cancelled while every sink kept up", b.cxl - a.cxl))
    share = stats_pkg::byp_pkt();
    tb_pkg::info($sformatf("TL06: inside a region turns every %0d..%0d cycles (bound %0d, %0d pairs); across regions every %0d..%0d (one global round %0d, %0d pairs)",
                           gl_min, gl_max, bound_l, gl_n, gx_min, gx_max, gr, gx_n));
    tb_pkg::info($sformatf("TL06: bypass share %0.2f%% of packets; %0d bursts in %0d turns, none cancelled", 100.0 * share,
                           b.used - a.used, b.turns - a.turns));
    tb_pkg::cover_bin("TL06 pairs inside regions measured", longint'(gl_n));
    if (CNR > 1) tb_pkg::cover_bin("TL06 cross-region pairs measured", longint'(gx_n));
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- summary
  // The design-C checkers' totals for the whole run (information only)
  function automatic void tl_summary();
    if (smon_pkg::the_smon == null) return;
    tb_pkg::info($sformatf("design C run: %0d local and %0d global subslots checked tile by tile, %0d local and %0d global turns, %0d local and %0d global rounds checked pair by pair",
                           smon_pkg::the_smon.n_lsub, smon_pkg::the_smon.n_gsub, smon_pkg::the_smon.n_lturns,
                           smon_pkg::the_smon.n_gturns, smon_pkg::the_smon.n_lrounds, smon_pkg::the_smon.n_grounds));
    tb_pkg::info($sformatf("design C run: %0d frames checked, %0d k_L changes, %0d local-phase cycles checked for containment, %0d lookaheads followed on links and %0d at NIs",
                           mchk_pkg::the_mchk.n_frames, mchk_pkg::the_mchk.n_k_changes, mchk_pkg::the_mchk.n_contain,
                           mchk_pkg::the_mchk.n_ann, mchk_pkg::the_mchk.n_nian));
    tb_pkg::info($sformatf("design C run: %0d bursts (%0d in local slots), %0d whose last flits landed in the other phase",
                           rchk_pkg::the_chk.n_bursts, rchk_pkg::the_chk.n_lbursts, rchk_pkg::the_chk.n_xtail));
  endfunction

  /* verilator lint_on UNUSEDSIGNAL */
