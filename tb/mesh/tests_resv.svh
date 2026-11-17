// =============================================================================
// File      tb/mesh/tests_resv.svh                                 Tracker: F69
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The B5 tests on a design-B mesh, as tasks of mesh_bench (included
//           in its module body after tests_bypass.svh, whose helpers they
//           use; top tb_mesh_b): turns per round, reservation, cancellation,
//           release, deadlock recovery, same-group slots, bypass share.
// Decisions D12 reservation and cancel, D38 timing contract, D48 reservation
//           space, D49 B5 verification
// Tests     BP03 BP06 BP07 BP08 BP09 BP10
// -----------------------------------------------------------------------------
// router_checker (F43) checks every burst and every cancel token against the
// timetable and D38's cycles, and ni_checker (F47) decides every reservation
// itself and follows every NI's bypass space, cycle by cycle; the sinks check
// every packet; each test ends with the idle check. Each test adds:
//   BP03  one 5-flit packet for every ordered pair (240 on 4x4, 4,032 on
//         8x8), each source's packets queued in the order of its turns and
//         held at its Local input: every packet leaves by bypass in its own
//         pair's subslot of the next round and lands at t + 2L + 3 + j; one
//         burst per pair, no turn cancelled (every sink keeps up, D48).
//   BP06  R(n^2-1)'s sink stalled for a whole round while every other node
//         holds packets for it, as many as one burst takes (two 5-flit
//         packets with B_MAX 10), each injected just before its own turn: the
//         first bursts fill the sink's bypass space, then its NI cancels;
//         every source whose turn is cancelled gets its token on every link
//         of its path at d + h (checker), holds it at its subslot (its cancel
//         counter, with its packets waiting) and sends nothing; nothing
//         overflows; all packets arrive after the stall.
//   BP07  a) one idle round: every NI decides once per ordered pair, accepts
//         every turn and releases every reservation as a no-show; at every
//         slot boundary no NI holds a reservation (released at slot end).
//         b) a forced long stall at one sink with bursts and regular packets
//         coming: the FIFO fills to its limit without overflow, turns are
//         cancelled; after the stall everything drains, the reservations
//         behave as in a), a burst to that node is accepted and delivered,
//         and every credit comes home (idle check).
//   BP08  (one VC: +define+TB_NUM_VC=1 +TEST=BP08) uniform traffic at 0.3
//         packets/node/cycle for 3,000 cycles (+QUICK 1,000), longer until a
//         cycle of waiting packets forms, with adaptive routing: design A
//         (tb_mesh_dl, +TEST=BP08) deadlocks; in design B cycles of waiting
//         packets form (a goal, not a check, with +QUICK) and are broken by
//         bypass, a delivery happens at least once per round (deadlock limit
//         = one round), and the network drains.
//   BP09  same-group slots (a = b): a 5-flit packet for every same-group
//         pair, held: each leaves in its subslot of slot {g, g}, bursts
//         collision-free (checker); subslot 0 of a same-group slot gives
//         nobody a turn (turns per round = ordered pairs).
//   BP10  uniform at 0.20 packets/node/cycle on 4x4 with 4 VCs (elsewhere
//         the same 78% of design A's saturation): the bypass share is
//         recorded (CSV columns of perf_probe), no turn is cancelled while
//         every sink keeps up, and the sources', checker's and sinks' counts
//         of bypassed packets agree.
// Packets are injected and holds changed at a rising edge only, as in
// tests_bypass.svh. A round starts with slot 0's setup window.
// =============================================================================

  // (finders return modes and lengths the tests do not all read)
  /* verilator lint_off UNUSEDSIGNAL */

  localparam int NPAIRS = NN * (NN - 1);       // ordered pairs: turns per round

  // The turn count at the falling edge of cycle snap_at (a test sets it
  // ahead, then reads turn_snap)
  int unsigned     snap_at   = 0;
  longint unsigned turn_snap = 0;
  always @(negedge clk) begin
    if (cyc == snap_at) turn_snap = sum_of(v_turns);
  end

  // First cycle at or after at_least that starts a round (slot 0's setup)
  function automatic int unsigned round_start(int unsigned at_least);
    longint r;
    if (longint'(at_least) <= longint'(c_rst)) return c_rst;
    r = (longint'(at_least) - longint'(c_rst) + longint'(BRL) - 1) / longint'(BRL);
    return int'(longint'(c_rst) + r * longint'(BRL));
  endfunction

  function automatic int grp_id(int r);
    return sched_pkg::grp_of(r % MESH_N, r / MESH_N, MESH_N);
  endfunction

  // Returns at the falling edge inside cycle c, where every register holds
  // its value from the start of the cycle (if c has passed, at once)
  task automatic in_cycle(int unsigned c);
    while (cyc < c) @(negedge clk);
  endtask

  // Sum of the RTL's reserved flits over every NI
  function automatic int sum_res();
    int s;
    s = 0;
    for (int i = 0; i < NN; i++) s += p_res[i];
    return s;
  endfunction

  // ---------------------------------------------------------------- BP03 / BP09
  // One 5-flit packet for every ordered pair (same_grp: only the pairs of a
  // same-group slot), each injected BYP_LEAD cycles before its pair's
  // subslot in the round starting at r0 and held at its source's Local
  // input: one round earlier the pair had its turn, so the packet can leave
  // by bypass only in that subslot
  task automatic pairs_round(string what, bit same_grp);
    int              L, n_exp, n_ok, n_bad, src, dst;
    int unsigned     r0, tq, t_end, ts;
    int unsigned     tt [NN][NN];
    longint unsigned c0, u0, w0, b0, ck0;
    sched_ref_pkg::xfer_q_t q;
    traffic_pkg::dlog_t d;
    traffic_defaults();
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    r0    = round_start(cyc + 200);
    n_exp = 0;
    for (int s = 0; s < NN; s++) for (int dd = 0; dd < NN; dd++) tt[s][dd] = 0;
    tb_pkg::info($sformatf("%s: the round starts at cycle %0d and has %0d cycles", what, r0, BRL));
    c0  = sum_of(v_cxl);
    u0  = sum_of(v_used);
    w0  = sum_of(v_turns);
    ck0 = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_bursts;
    for (int s = 0; s < NN; s++) hold_req[s] = 5'b10000;
    snap_at = r0;
    // subslot by subslot, in time order
    for (int sl = 0; sl < BNS; sl++) begin
      for (int k = 0; k < MESH_N; k++) begin
        q  = sched_ref_pkg::ref_xfers(MESH_N, sl, k, 1'b0);
        ts = r0 + sl * BSL + BS + k * BT;
        if (q.size() == 0) continue;
        at_cycle(ts - BYP_LEAD);
        foreach (q[m]) begin
          src = q[m].sy * MESH_N + q[m].sx;
          dst = q[m].dy * MESH_N + q[m].dx;
          if (same_grp && grp_id(src) != grp_id(dst)) continue;
          if (same_grp) begin
            `TB_CHECK(sl == sched_pkg::slot_of(grp_id(src), grp_id(src)) && k != 0,
                      ("%s: R%0d -> R%0d (group %0d) in slot %0d subslot %0d, not a same-group subslot", what, src, dst, grp_id(src), sl, k))
          end
          `TB_CHECK(tt[src][dst] == 0, ("%s: R%0d -> R%0d has two turns in one round", what, src, dst))
          tt[src][dst] = ts;
          traffic_pkg::direct(src, dst, MAX_PKT_FLITS);
          n_exp++;
        end
      end
    end
    `TB_CHECK(n_exp == (same_grp ? NN * (MESH_N - 1) : NPAIRS),
              ("%s: the round has %0d turns for the test's pairs, expected %0d", what, n_exp, same_grp ? NN * (MESH_N - 1) : NPAIRS))
    tb_pkg::info($sformatf("%s: %0d packets injected, each %0d cycles before its pair's subslot", what, n_exp, BYP_LEAD));
    // the turns of the round (b0 taken at r0 by the turn counter below)
    t_end = r0 + BRL + 2 * sched_pkg::hmax(MESH_N) + B_MAX + 4;
    in_cycle(r0 + BRL);
    b0 = turn_snap;
    `TB_CHECK(sum_of(v_turns) - b0 == longint'(NPAIRS),
              ("%s: %0d turns in one round, the timetable has %0d ordered pairs", what, sum_of(v_turns) - b0, NPAIRS))
    at_cycle(t_end);
    for (int s = 0; s < NN; s++) hold_req[s] = '0;
    wait_delivered(what);
    n_ok  = 0;
    n_bad = 0;
    foreach (traffic_pkg::dlog[i]) begin
      d  = traffic_pkg::dlog[i];
      src = int'(d.src);
      dst = int'(d.dst);
      tq  = tt[src][dst];
      L   = nhops(src, dst);
      if (d.byp && tq != 0 && d.t_tail == tq + sched_pkg::tc_eject(L, MAX_PKT_FLITS - 1)) begin
        n_ok++;
      end else if (n_bad < 10) begin
        n_bad++;
        tb_pkg::info($sformatf("%s: packet %0d.%0d R%0d -> R%0d: bypass %0b, tail at %0d, its turn at %0d", what,
                               d.src, d.seq, d.src, d.dst, d.byp, d.t_tail, tq));
      end
    end
    `TB_CHECK(traffic_pkg::dlog.size() == n_exp && n_ok == n_exp,
              ("%s: %0d packets delivered, %0d of them by bypass in their own subslot at t + 2L + 3 + 4; expected %0d",
               what, traffic_pkg::dlog.size(), n_ok, n_exp))
    `TB_CHECK(sum_of(v_used) - u0 == longint'(n_exp) && sum_of(v_cxl) - c0 == 0,
              ("%s: %0d bursts and %0d cancelled turns, expected %0d and 0", what, sum_of(v_used) - u0, sum_of(v_cxl) - c0, n_exp))
    if (rchk_pkg::the_chk != null) begin
      `TB_CHECK(rchk_pkg::the_chk.n_bursts - ck0 == longint'(n_exp),
                ("%s: the checker saw %0d bursts, expected %0d", what, rchk_pkg::the_chk.n_bursts - ck0, n_exp))
    end
    tb_pkg::info($sformatf("%s: all %0d packets left by bypass in their own subslot of one round and landed at t + 2L + 3 + j; %0d turns offered, none cancelled",
                           what, n_ok, sum_of(v_turns) - w0));
    traffic_pkg::log_on = 1'b0;
    check_idle(what);
    traffic_defaults();
  endtask

  task automatic bp03();
    tb_pkg::phase($sformatf("BP03 every ordered pair once per round: %0d packets, one per pair", NPAIRS));
    pairs_round("BP03", 1'b0);
  endtask

  task automatic bp09();
    tb_pkg::phase($sformatf("BP09 same-group slots: %0d packets, one per pair inside a group", NN * (MESH_N - 1)));
    pairs_round("BP09", 1'b1);
  endtask

  // ---------------------------------------------------------------- BP06
  // BP06's counts at the falling edge of cycle snap6_at (the round's start;
  // the test injects packets on both sides of it)
  int unsigned     snap6_at = 0;
  longint unsigned s6_cxl [NN];
  longint unsigned s6_lost [NN];
  longint unsigned s6_rcx = 0, s6_tok = 0, s6_ok = 0;
  always @(negedge clk) begin
    if (snap6_at != 0 && cyc == snap6_at) begin
      for (int s = 0; s < NN; s++) begin
        s6_cxl[s]  = v_cxl[s];
        s6_lost[s] = v_lost[s];
      end
      s6_rcx = v_rcx[NN - 1];
      s6_tok = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_tok;
      s6_ok  = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_cxok;
    end
  end

  // R(NN-1)'s sink stalls from 100 cycles before a round to its end. Every
  // other node gets npk 5-flit packets for it, as many as one burst takes
  // (B_MAX / 5, at most one per VC), injected BYP_LEAD cycles before its
  // turn in that round and held at its Local input: one round earlier the
  // pair had its turn, so the packets can leave only in this turn. The
  // bursts fill the stalled sink's bypass space until its NI cancels.
  task automatic bp06();
    int              dst, sl, k, n_cx, n_ok, n_bad, npk, blen, L, nxt, si;
    bit              mode, ok, in_t, exp_cx;
    int unsigned     r0, t_end, t_inj, t_lo, t_hi;
    longint unsigned n_c;
    int unsigned     tt [NN];
    bit              done [NN];
    longint unsigned c1 [NN], l1 [NN], rc1, tk1, ok1;
    traffic_pkg::dlog_t d;
    dst  = NN - 1;
    npk  = B_MAX / MAX_PKT_FLITS;
    if (npk > NUM_VC) npk = NUM_VC;
    if (npk < 1)      npk = 1;
    blen = npk * MAX_PKT_FLITS;
    // with every earlier turn accepted, the last one must find no room
    exp_cx = (NN - 2) * blen + B_MAX > SPACE;
    tb_pkg::phase($sformatf("BP06 cancellation: R%0d's sink stalled for a round while every other node holds %0d packets for it", dst, npk));
    traffic_defaults();
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    r0 = round_start(cyc + 200);
    for (int s = 0; s < NN; s++) begin
      tt[s]   = 0;
      done[s] = (s == dst);
      if (s == dst) continue;
      ok    = find_sub(s, dst, sl, k, mode);
      `TB_CHECK(ok, ("BP06: the timetable never lets R%0d send to R%0d", s, dst))
      tt[s] = sub_start(sl, k, r0);
    end
    t_end = r0 + BRL + 2 * sched_pkg::hmax(MESH_N) + B_MAX + 4;
    at_cycle(r0 - 120);
    traffic_pkg::fs_node  = dst;
    traffic_pkg::fs_start = r0 - 100;
    traffic_pkg::fs_end   = t_end;
    for (int s = 0; s < NN; s++) if (s != dst) hold_req[s] = 5'b10000;
    snap6_at = r0;
    // inject in time order, each source BYP_LEAD cycles before its turn
    for (int i = 0; i < NN - 1; i++) begin
      nxt = -1;
      for (int s = 0; s < NN; s++) if (!done[s] && (nxt < 0 || tt[s] < tt[nxt])) nxt = s;
      done[nxt] = 1'b1;
      t_inj = tt[nxt] - BYP_LEAD;
      at_cycle(t_inj);
      for (int m = 0; m < npk; m++) traffic_pkg::direct(nxt, dst, MAX_PKT_FLITS);
    end
    // the counts over exactly one round (turns of the next round, still in
    // the stall, cancel too but are not this test's)
    in_cycle(r0 + BRL);
    for (int s = 0; s < NN; s++) begin
      c1[s] = v_cxl[s];
      l1[s] = v_lost[s];
    end
    rc1 = v_rcx[dst];
    tk1 = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_tok;
    ok1 = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_cxok;
    snap6_at = 0;
    at_cycle(t_end);
    for (int s = 0; s < NN; s++) hold_req[s] = '0;
    wait_delivered("BP06");
    // every cancelled source held its token at its turn, with its packets waiting
    n_cx  = 0;
    n_bad = 0;
    for (int s = 0; s < NN; s++) begin
      if (s == dst) continue;
      if (c1[s] != s6_cxl[s]) begin
        n_cx++;
        if (c1[s] - s6_cxl[s] != 1 || l1[s] - s6_lost[s] != 1) begin
          n_bad++;
          tb_pkg::info($sformatf("BP06: R%0d: %0d turns cancelled, %0d of them with a packet waiting; its turn to R%0d at cycle %0d",
                                 s, c1[s] - s6_cxl[s], l1[s] - s6_lost[s], dst, tt[s]));
        end
      end
    end
    if (exp_cx) begin
      `TB_CHECK(n_cx >= 1, ("BP06: R%0d's sink was stalled for a round and no turn was cancelled", dst))
    end else begin
      tb_pkg::info($sformatf("BP06: %0d bursts of %0d flits fit in R%0d's bypass space of %0d flits, so no cancel is certain here",
                             NN - 1, blen, dst, SPACE));
    end
    `TB_CHECK(n_bad == 0, ("BP06: %0d cancelled sources did not skip exactly one turn with their packets waiting", n_bad))
    `TB_CHECK(longint'(n_cx) == rc1 - s6_rcx,
              ("BP06: R%0d's NI cancelled %0d turns, %0d sources skipped one", dst, rc1 - s6_rcx, n_cx))
    if (rchk_pkg::the_chk != null) begin
      `TB_CHECK(ok1 - s6_ok == longint'(n_cx),
                ("BP06: the checker saw %0d cancelled turns with nothing sent, expected %0d", ok1 - s6_ok, n_cx))
    end
    // accepted turns: every packet by bypass within its turn's burst;
    // cancelled ones: any other way. The source index is cast into a
    // variable first: xsim 2024.2 read tt[int'(d.src)] inside an expression
    // as element 0 (R-1110-02, README rule 9).
    n_ok = 0;
    foreach (traffic_pkg::dlog[i]) begin
      d    = traffic_pkg::dlog[i];
      si   = int'(d.src);
      L    = nhops(si, dst);
      t_lo = tt[si] + sched_pkg::tc_eject(L, MAX_PKT_FLITS - 1);
      t_hi = tt[si] + sched_pkg::tc_eject(L, blen - 1);
      in_t = d.byp && d.t_tail >= t_lo && d.t_tail <= t_hi;
      n_c  = c1[si] - s6_cxl[si];
      if ((n_c != 0) != in_t) n_ok++;
      else tb_pkg::info($sformatf("BP06: packet %0d.%0d from R%0d: bypass %0b, tail at %0d, its turn at %0d, the source's turns cancelled %0d",
                                  d.src, d.seq, d.src, d.byp, d.t_tail, tt[si], n_c));
    end
    `TB_CHECK(traffic_pkg::dlog.size() == (NN - 1) * npk && n_ok == (NN - 1) * npk,
              ("BP06: %0d packets delivered, %0d as expected (accepted ones by bypass in the turn, cancelled ones later); expected %0d",
               traffic_pkg::dlog.size(), n_ok, (NN - 1) * npk))
    tb_pkg::info($sformatf("BP06: %0d of %0d turns to R%0d accepted while its sink was stalled, then %0d cancelled (its bypass space peaked at %0d of %0d flits); the checker matched %0d cancel tokens on their links; every cancelled source skipped its turn and its packets arrived after the stall",
                           NN - 1 - n_cx, NN - 1, dst, n_cx, v_msum[dst], SPACE, tk1 - s6_tok));
    if (exp_cx) tb_pkg::cover_bin("BP06 turns cancelled", longint'(n_cx));
    else        tb_pkg::cover_goal("BP06 turns cancelled", longint'(n_cx));
    traffic_pkg::log_on = 1'b0;
    check_idle("BP06");
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- BP07
  task automatic bp07();
    int              dst, src, sl, k, bad, blen, nmax;
    bit              mode, ok;
    int unsigned     r0, t, t_end;
    longint unsigned d0, a0, x0, n0, c0;
    tb_pkg::phase("BP07 release: an idle round, then a long sink stall with bursts and regular packets coming");
    traffic_defaults();
    // a) one idle round: every turn accepted, every reservation a no-show
    r0 = round_start(cyc + 10);
    in_cycle(r0);
    d0 = sum_of(v_dec);
    a0 = sum_of(v_acc);
    x0 = sum_of(v_rcx);
    n0 = sum_of(v_nosh);
    bad = 0;
    for (int s = 0; s < BNS; s++) begin
      in_cycle(r0 + s * BSL);              // a slot's first setup cycle, before its decisions
      if (sum_res() != 0) bad++;
    end
    in_cycle(r0 + BRL);
    if (sum_res() != 0) bad++;
    `TB_CHECK(bad == 0, ("BP07 a: reservations still held at %0d of %0d slot boundaries", bad, BNS + 1))
    `TB_CHECK(sum_of(v_dec) - d0 == longint'(NPAIRS) && sum_of(v_acc) - a0 == sum_of(v_dec) - d0 &&
              sum_of(v_rcx) - x0 == 0 && sum_of(v_nosh) - n0 == sum_of(v_acc) - a0,
              ("BP07 a: one idle round: %0d decisions (expected %0d), %0d accepted, %0d cancelled, %0d released as no-shows",
               sum_of(v_dec) - d0, NPAIRS, sum_of(v_acc) - a0, sum_of(v_rcx) - x0, sum_of(v_nosh) - n0))
    tb_pkg::info($sformatf("BP07 a: one idle round: %0d decisions, all accepted, all released as no-shows; no reservation held at any of the %0d slot boundaries",
                           sum_of(v_dec) - d0, BNS + 1));
    // b) a long stall at R(n^2-1): every other node bursts a packet to it in
    //    its turn, and its neighbours send regular packets to it
    dst = NN - 1;
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    r0 = round_start(cyc + 200);
    c0 = v_rcx[dst];
    t_end = r0 + BRL + 2 * sched_pkg::hmax(MESH_N) + B_MAX + 4;
    at_cycle(r0 - 120);
    traffic_pkg::fs_node  = dst;
    traffic_pkg::fs_start = r0 - 110;
    traffic_pkg::fs_end   = t_end + 200;
    for (int s = 0; s < NN; s++) begin
      if (s == dst) continue;
      hold_req[s] = 5'b10000;
      traffic_pkg::direct(s, dst, MAX_PKT_FLITS);
    end
    // regular packets from the two neighbours: the hold keeps them at their
    // Local inputs, so release it there and give them another node's port
    at_cycle(r0 - 100);
    hold_req[dst - 1]      = '0;
    hold_req[dst - MESH_N] = '0;
    for (int i = 0; i < 2 * NUM_VC; i++) begin
      traffic_pkg::direct(dst - 1, dst, MAX_PKT_FLITS);
      traffic_pkg::direct(dst - MESH_N, dst, MAX_PKT_FLITS);
    end
    at_cycle(t_end);
    for (int s = 0; s < NN; s++) hold_req[s] = '0;
    nmax = (nchk_pkg::the_nchk == null) ? 0 : nchk_pkg::the_nchk.max_occ;
    wait_delivered("BP07 b");
    `TB_CHECK(v_rcx[dst] - c0 >= 1, ("BP07 b: R%0d's sink was stalled for a round and its NI cancelled nothing", dst))
    tb_pkg::info($sformatf("BP07 b: R%0d stalled for %0d cycles: its NI cancelled %0d turns; its FIFO held up to %0d of %0d flits (%0d regular at most, bypass space %0d)",
                           dst, t_end + 200 - (r0 - 110), v_rcx[dst] - c0, v_fifo[dst], EJ_DEPTH, NUM_VC * VC_DEPTH, SPACE));
    `TB_CHECK(v_fifo[dst] > longint'(NUM_VC * VC_DEPTH),
              ("BP07 b: the stalled FIFO held at most %0d flits; the test should fill it past the %0d regular ones", v_fifo[dst], NUM_VC * VC_DEPTH))
    // after the stall: an idle round behaves as in a), and a burst to dst is accepted
    r0 = round_start(cyc + 200);
    bad = 0;
    for (int s = 0; s < BNS; s++) begin
      in_cycle(r0 + s * BSL);
      if (sum_res() != 0) bad++;
    end
    `TB_CHECK(bad == 0, ("BP07 b: after the stall, reservations still held at %0d slot boundaries", bad))
    traffic_pkg::dlog.delete();
    src = 0;
    ok  = find_sub(src, dst, sl, k, mode);
    `TB_CHECK(ok, ("BP07: the timetable never lets R%0d send to R%0d", src, dst))
    t   = sub_start(sl, k, cyc + BYP_LEAD + 2);
    at_cycle(t - BYP_LEAD);
    hold_req[src] = 5'b10000;
    traffic_pkg::direct(src, dst, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[src] = '0;
    wait_delivered("BP07 b after");
    check_burst("BP07 b after the stall", src, dst, t, MAX_PKT_FLITS, blen);
    tb_pkg::info($sformatf("BP07 b: after the stall R%0d accepted R%0d's turn at cycle %0d and the burst landed on time; the largest FIFO occupancy the model saw anywhere was %0d",
                           dst, src, t, nmax));
    traffic_pkg::log_on = 1'b0;
    check_idle("BP07");
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- BP08
  task automatic bp08();
    bit              ok;
    int unsigned     load, dl0, n_cyc, t_first, cl;
    real             r8;
    string           path;
    longint unsigned u0, p0;
    int              stalls0, clen;
    if (NUM_VC != 1) begin
      tb_pkg::fail($sformatf("BP08 needs one VC (+define+TB_NUM_VC=1); this bench has %0d", NUM_VC));
      return;
    end
    if (!$value$plusargs("BP08_RATE=%f", r8)) r8 = 0.3;
    load = quick() ? 1000 : 3000;
    if (DESIGN == DESIGN_A) begin
      bp08_a(r8, load);
      return;
    end
    tb_pkg::phase($sformatf("BP08 deadlock recovery: 1 VC, adaptive routing, uniform at %0.3f packets/node/cycle for %0d cycles or until a cycle of waiting packets forms (at most %0d); a delivery at least once per round (%0d cycles)",
                            r8, load, 4 * load, BRL));
    traffic_defaults();
    traffic_pkg::rate = r8;
    stats_pkg::reset(NN);
    stalls0 = u_rc.stalls;
    dl0 = u_rc.dl_limit;
    u_rc.dl_limit = BRL + 2 * sched_pkg::hmax(MESH_N) + B_MAX + 50;   // one round: every waiting packet's turn comes
    u0 = sum_of(v_used);
    p0 = sum_of(v_bpkts);
    n_cyc   = 0;
    t_first = 0;
    traffic_pkg::gen_on = 1'b1;
    sb_pkg::last_delivery = cyc;
    // at least load cycles, and on until a cycle of waiting packets has
    // formed (some seeds need longer than QUICK's 1,000 cycles)
    for (int unsigned c = 0; c < load || (n_cyc == 0 && c < 4 * load); c++) begin
      @(posedge clk);
      if (u_rc.deadlocked()) break;
      if (c % 50 == 0) begin
        clen = wait_cycle(path);
        if (clen >= 2) begin
          n_cyc++;
          if (t_first == 0) begin
            t_first = cyc;
            cl      = clen;
            tb_pkg::info($sformatf("BP08: at cycle %0d, %0d packets wait in a cycle, each for the input VC the next one holds:%s",
                                   cyc, clen, path));
          end
        end
      end
    end
    traffic_pkg::gen_on = 1'b0;
    `TB_CHECK(!u_rc.deadlocked(), ("BP08: no delivery for %0d cycles while generating", u_rc.dl_limit))
    u_rc.drain(DRAIN_MAX, ok);
    u_rc.dl_limit = dl0;
    `TB_CHECK(ok && u_rc.stalls == stalls0, ("BP08: %s", why_stuck(stalls0, DRAIN_MAX)))
    stop_if(ok, "BP08");
    // the full form must see a deadlock form and break; in the short form
    // (EDA Playground) some seeds form none in 4,000 cycles: a goal there
    if (!quick()) begin
      `TB_CHECK(n_cyc >= 1, ("BP08: no cycle of waiting packets formed; raise +BP08_RATE"))
    end
    tb_pkg::info($sformatf("BP08: cycles of waiting packets seen in %0d samples (first at cycle %0d, %0d packets); all broken: %0d bursts carried %0d packets, every packet delivered",
                           n_cyc, t_first, cl, sum_of(v_used) - u0, sum_of(v_bpkts) - p0));
    if (quick()) tb_pkg::cover_goal("BP08 samples with a cycle of waiting packets", longint'(n_cyc));
    else         tb_pkg::cover_bin("BP08 samples with a cycle of waiting packets", longint'(n_cyc));
    check_idle("BP08");
    traffic_defaults();
  endtask

  // BP08's other half (tb_mesh_dl, design A): the same traffic deadlocks
  task automatic bp08_a(real r8, int unsigned load);
    bit          ok;
    int          clen;
    string       path;
    int unsigned t0;
    tb_pkg::phase($sformatf("BP08 design A: 1 VC, adaptive routing, uniform at %0.3f packets/node/cycle for %0d cycles: expected to deadlock",
                            r8, load));
    traffic_defaults();
    traffic_pkg::rate = r8;
    stats_pkg::reset(NN);
    t0 = cyc;
    traffic_pkg::gen_on = 1'b1;
    u_rc.wait_cycles(load, ok);
    traffic_pkg::gen_on = 1'b0;
    if (ok) u_rc.drain(DRAIN_MAX, ok);
    `TB_CHECK(!ok && u_rc.stalls == 1, ("BP08 design A: no deadlock (it drained)"))
    if (ok) return;
    clen = wait_cycle(path);
    `TB_CHECK(clen >= 2, ("BP08 design A: frozen, but no cycle of waiting packets found"))
    tb_pkg::info($sformatf("BP08 design A: deadlock, the last delivery at cycle %0d (%0d cycles after the load started); %0d packets wait in a cycle:%s",
                           u_rc.t_stall, u_rc.t_stall - t0, clen, path));
    tb_pkg::cover_bin("BP08 design A cycles to deadlock", longint'(u_rc.t_stall) - longint'(t0));
  endtask

  // ---------------------------------------------------------------- BP10
  task automatic bp10();
    int unsigned     meas;
    real             share, sflit, r10;
    perf_pkg::snap_t a, b;
    longint unsigned bp0, ck0, cx0;
    meas = quick() ? 1500 : 10000;
    // 0.20 on 4x4 with 4 VCs (the tracker's point); elsewhere the same share
    // (78%) of design A's saturation, so that no mesh is saturated
    r10  = 0.20 * sat_rate() / 0.257;
    tb_pkg::phase($sformatf("BP10 bypass share: uniform at %0.3f packets/node/cycle (78%% of design A's saturation), %0d cycles measured",
                            r10, meas));
    traffic_defaults();
    traffic_pkg::rate = r10;
    a   = perf_snap();
    bp0 = sum_of(v_bpkts);
    ck0 = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_pkts;
    cx0 = (nchk_pkg::the_nchk == null) ? 0 : nchk_pkg::the_nchk.n_cxl;
    load_phase("BP10", meas / 5, meas);
    report("BP10");
    b     = perf_snap();
    share = stats_pkg::byp_pkt();
    sflit = stats_pkg::byp_flit();
    `TB_CHECK(b.cxl - a.cxl == 0 && b.rcx - a.rcx == 0,
              ("BP10: %0d turns cancelled (%0d by the NIs) while every sink kept up", b.cxl - a.cxl, b.rcx - a.rcx))
    if (nchk_pkg::the_nchk != null) begin
      `TB_CHECK(nchk_pkg::the_nchk.n_cxl - cx0 == 0, ("BP10: the NI model cancelled %0d turns", nchk_pkg::the_nchk.n_cxl - cx0))
    end
    if (rchk_pkg::the_chk != null) begin
      `TB_CHECK(rchk_pkg::the_chk.n_pkts - ck0 == sum_of(v_bpkts) - bp0,
                ("BP10: the checker counted %0d bypassed packets, the sources sent %0d", rchk_pkg::the_chk.n_pkts - ck0, sum_of(v_bpkts) - bp0))
    end
    `TB_CHECK(share > 0.0 && share < 1.0 && b.used > a.used,
              ("BP10: bypass share %0.4f of the measured packets, %0d bursts", share, b.used - a.used))
    tb_pkg::info($sformatf("BP10: bypass share %0.2f%% of packets, %0.2f%% of flits; %0d bursts in %0d turns (%0.2f%%), none cancelled; [1] reports 49-73%% (open question for E0)",
                           100.0 * share, 100.0 * sflit, b.used - a.used, b.turns - a.turns,
                           100.0 * real'(b.used - a.used) / real'((b.turns > a.turns) ? b.turns - a.turns : 1)));
    tb_pkg::csv({stats_pkg::csv_header(), ",", perf_pkg::header()});
    tb_pkg::csv({stats_pkg::csv_row($sformatf("BP10_N%0d_V%0d", MESH_N, NUM_VC), "B", MESH_N, NUM_VC, "uniform", 1.0 / 3.0, r10,
                                    tb_pkg::seed, u_rc.stalls, u_rc.t_stall, meas, $sformatf("%0d", B_MAX), tmode_name()),
                 ",", perf_pkg::row(a, b)});
    tb_pkg::cover_bin("BP10 packets by bypass", sum_of(v_bpkts) - bp0);
    traffic_defaults();
  endtask
  /* verilator lint_on UNUSEDSIGNAL */
