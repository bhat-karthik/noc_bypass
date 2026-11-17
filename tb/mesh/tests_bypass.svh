// =============================================================================
// File      tb/mesh/tests_bypass.svh                               Tracker: F44
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The bypass tests BP01, BP02, BP04 and BP05 on a design-B mesh,
//           as tasks of mesh_bench (included in its module body; top
//           tb_mesh_b). Reservations are forced to succeed (B4): every source
//           sends whenever it has packets for its partner.
// Decisions D11 burst size, D38 timing contract, D42 bypass source,
//           D45 B4 verification
// Tests     BP01 BP02 BP04 BP05
// -----------------------------------------------------------------------------
// router_checker (F43) checks every burst of every test against the
// timetable and D38's cycles; the sinks check every packet; each test ends
// with the idle check. Each test adds:
//   BP01  one 5-flit packet R2 -> R9 on 4x4, held at R2 until slot 8,
//         subslot 2 (Table I, Fig. 3 of [1]): the timetable puts it there, it
//         leaves as a 5-flit burst and lands at t + 2L + 3 + j (D38), and
//         the checker saw every one of its lookahead, link and NI events.
//         Other mesh sizes use the same coordinates, (2,0) -> (1,2) (2x2:
//         (1,0) -> (0,1)), in whatever subslot the timetable gives.
//   BP02  the corners, both ways (R0 <-> R(n^2-1), R(n-1) <-> R(n^2-n)):
//         the longest path, Hmax hops, with two-packet bursts (10 and 6
//         flits), each packet's tail in its NI at exactly t + 2L + 3 + j. On
//         4x4 and 8x8 (+define+TB_MESH_N=8) [the tracker's stimulus]. With
//         TIGHT subslots the last flits land after the subslot ends, by
//         design (D39; U09 proves it safe), so "within the subslot" is
//         checked as "at the D38 cycles". Then (n >= 3) the test looks in
//         the timetable for two bursts back to back on one link: a burst A
//         and, in the next subslot, a burst B whose lookahead reaches a
//         router in the cycle A's last flit crosses it on the same input
//         (T + 2 i_B = 2 i_A + B_A). If such a pair exists it runs both;
//         if not (none on 4x4 or 8x8 with TIGHT: A would need a longest
//         path whose last link is B's first or second), it says so: the
//         relay's back-to-back case cannot occur in this timetable.
//   BP04  more eligible flits than B_MAX:
//         a) four 5-flit packets for the partner at the source: exactly two
//            go (B_MAX = 10 flits), the other two wait and then go by the
//            regular path;
//         b) (n >= 3) packets for the partner at two inputs of the source:
//            two arrive from its east neighbour and wait in its East input
//            VCs, one waits at its Local input; the burst is whole packets,
//            at most B_MAX flits, takes at least one packet from the East
//            input (bypass from a router that is not the packet's source,
//            [1] Sec. III-D), and leaves out only packets that would not fit;
//         c) D38's rule: a source S takes a 5-flit packet from one of its
//            mesh inputs while a burst X -> Y from that input's neighbour X
//            crosses the same input in the same cycles (S is X's first hop).
//            S's flits use crossbar input 4, so both bursts arrive intact at
//            their D38 cycles. The test finds such an X, S, Y and S's
//            partner D such that X's packet for D takes the link X -> S: D
//            beyond S along x (an idle mesh routes x first, also adaptive:
//            ties go to x, D06), or in S's column beyond S along y;
//         d) (2x2, where a subslot is exactly B_MAX cycles) a source sends a
//            10-flit burst and chooses the next one, for the next partner, in
//            the cycle the first burst's last flit leaves.
//   BP05  uniform traffic at 0.9 x the saturation load of design A, mixed
//         packets, with bypass: no link, crossbar or lookahead conflict
//         (checker, RTL assertions), every packet intact, idle afterwards;
//         bursts must occur, over at least one timetable round. Bursts of
//         several packets and packets taken from mesh inputs are counted as
//         coverage goals, not checks: random traffic makes them rare in short
//         or small runs, and BP02 and BP04 make both happen on purpose.
// Directed tests hold packets with the router's test hold (hold_req) and
// change holds and packet requests at a rising edge only, so every
// simulator sees the same order. Packets are injected BYP_LEAD cycles before
// the subslot they are meant for: enough for 20 flits to reach their VCs, and
// no earlier occurrence of that subslot (a round apart) falls in between.
// A packet left out of a burst may later leave from another router on its
// path whose partner it is (any inport, [1] Sec. III-D), or regularly.
// The test hold is released at the edge that starts cycle t, the subslot's
// first cycle: from then on only the source's own hold and masks keep the
// switch allocator off the chosen packets, the Local input and the first-hop
// output.
// Burst sizes the tests expect follow NUM_VC (one packet waits per input VC)
// and B_MAX; where the rotating start of the source's scan decides which
// packets fit, the tests check the rule instead (whole packets, <= B_MAX,
// nothing left out fits).
// BP05 runs at least one timetable round: 4,000 cycles on 4x4 (+QUICK:
// 1,000), 10,332 on 8x8.
// =============================================================================

  // ---------------------------------------------------------------- timetable (design B)
  localparam int BT  = sched_pkg::t_sub(MESH_N, B_MAX, T_MODE);   // subslot, cycles
  localparam int BS  = sched_pkg::s_setup(MESH_N);                // setup window
  localparam int BSL = sched_pkg::slot_len(MESH_N, BT);
  localparam int BRL = sched_pkg::n_slots(MESH_N) * BSL;          // round
  localparam int BNS = sched_pkg::n_slots(MESH_N);
  localparam int BYP_WAIT = 200000;                               // cycles a directed test may wait
  localparam int BYP_LEAD = 30;                                   // inject this long before the subslot

  // First cycle at or after at_least in which subslot k of slot sl starts
  function automatic int unsigned sub_start(int sl, int k, int unsigned at_least);
    longint off, t0, r;
    off = longint'(sl) * longint'(BSL) + longint'(BS) + longint'(k) * longint'(BT);
    t0  = longint'(c_rst) + off;
    if (longint'(at_least) <= t0) return int'(t0);
    r = (longint'(at_least) - t0 + longint'(BRL) - 1) / longint'(BRL);
    return int'(t0 + r * longint'(BRL));
  endfunction

  // Slot and subslot in which src sends to dst, from the reference model
  function automatic bit find_sub(int src, int dst, output int sl, output int k, output bit mode);
    sched_ref_pkg::xfer_q_t q;
    sl   = -1;
    k    = -1;
    mode = 1'b0;
    for (int s = 0; s < BNS; s++) begin
      for (int kk = 0; kk < MESH_N; kk++) begin
        q = sched_ref_pkg::ref_xfers(MESH_N, s, kk, 1'b0);
        foreach (q[m]) begin
          if (q[m].sy * MESH_N + q[m].sx == src && q[m].dy * MESH_N + q[m].dx == dst) begin
            sl   = s;
            k    = kk;
            mode = q[m].mode;
          end
        end
      end
    end
    return sl >= 0;
  endfunction

  function automatic int nhops(int a, int b);
    return hops(coord_t'(a % MESH_N), coord_t'(a / MESH_N), coord_t'(b % MESH_N), coord_t'(b / MESH_N));
  endfunction

  // Returns at the rising edge that ends cycle c (cyc still reads c there)
  // (if cycle c has passed, at the next rising edge)
  task automatic at_cycle(int unsigned c);
    if (cyc > c) begin
      @(posedge clk);
      return;
    end
    while (cyc < c) @(negedge clk);
    @(posedge clk);
  endtask

  // Until every packet created is delivered (or BYP_WAIT cycles)
  task automatic wait_delivered(string what);
    int unsigned     t0;
    longint unsigned left;               // D41: no call in the loop condition
    t0   = cyc;
    left = sb_pkg::outstanding();
    while (left != 0 && cyc - t0 < BYP_WAIT) begin
      @(negedge clk);
      left = sb_pkg::outstanding();
    end
    `TB_CHECK(left == 0, ("%s: %0d packets not delivered after %0d cycles", what, left, BYP_WAIT))
    stop_if(left == 0, what);
  endtask

  // The delivery log entry of packet (src, seq); 0 when there is none
  function automatic bit find_dlog(int src, int unsigned seq, output traffic_pkg::dlog_t d);
    d = '0;
    foreach (traffic_pkg::dlog[i]) begin
      if (int'(traffic_pkg::dlog[i].src) == src && traffic_pkg::dlog[i].seq == seq) begin
        d = traffic_pkg::dlog[i];
        return 1'b1;
      end
    end
    return 1'b0;
  endfunction

  function automatic longint unsigned sum_of(longint unsigned a [NN]);
    longint unsigned s;
    s = 0;
    for (int i = 0; i < NN; i++) s += a[i];
    return s;
  endfunction

  // The packets of the burst src sent at t: count, and each one's source,
  // sequence number and length (from the checker's record of link 1)
  function automatic int get_burst_pkts(int src, int unsigned t, output int psrc [16], output int pseq [16],
                                        output int plen [16]);
    int np;
    np = 0;
    for (int i = 0; i < 16; i++) begin
      psrc[i] = -1;
      pseq[i] = -1;
      plen[i] = 0;
    end
    if (rchk_pkg::the_chk == null) return 0;
    np = rchk_pkg::the_chk.burst_pkts(src, int'(t), psrc, pseq, plen);
    return np;
  endfunction

  // One burst against what the test expects: its length (0: any), every
  // packet delivered by bypass with its tail in the NI at t + 2L + 3 + j
  // (a delivery log entry is read only in part)
  /* verilator lint_off UNUSEDSIGNAL */
  task automatic check_burst(string what, int src, int dst, int unsigned t, int exp_len, output int got_len);
    int                 np, L, j, psrc [16], pseq [16], plen [16];
    traffic_pkg::dlog_t d;
    bit                 ok;
    L       = nhops(src, dst);
    got_len = (rchk_pkg::the_chk == null) ? -1 : rchk_pkg::the_chk.burst_len(src, int'(t));
    if (exp_len > 0) begin
      `TB_CHECK(got_len == exp_len, ("%s: the burst R%0d->R%0d at cycle %0d has %0d flits, expected %0d",
                                     what, src, dst, t, got_len, exp_len))
    end else begin
      `TB_CHECK(got_len >= 1 && got_len <= B_MAX, ("%s: the burst R%0d->R%0d at cycle %0d has %0d flits",
                                                   what, src, dst, t, got_len))
    end
    np = get_burst_pkts(src, t, psrc, pseq, plen);
    j  = 0;
    for (int i = 0; i < np; i++) begin
      j  = j + plen[i];
      ok = find_dlog(psrc[i], pseq[i], d);
      `TB_CHECK(ok && d.byp && int'(d.dst) == dst && d.t_tail == t + sched_pkg::tc_eject(L, j - 1),
                ("%s: packet %0d.%0d of the burst R%0d->R%0d (flits %0d-%0d): delivered %0b, bypass %0b, at R%0d, tail at cycle %0d, expected t + 2L + 3 + %0d = %0d",
                 what, psrc[i], pseq[i], src, dst, j - plen[i], j - 1, ok, d.byp, d.dst, d.t_tail, j - 1,
                 t + sched_pkg::tc_eject(L, j - 1)))
    end
    `TB_CHECK(j == got_len, ("%s: the burst's packets add up to %0d flits, the burst has %0d", what, j, got_len))
  endtask
  /* verilator lint_on UNUSEDSIGNAL */

  // ---------------------------------------------------------------- BP01
  task automatic bp01();
    int          src, dst, sl, k, L, blen, nev0, nev;
    bit          mode, ok;
    int unsigned t;
    tb_pkg::phase("BP01 one bypass at the exact cycles: a 5-flit packet held at its source until its subslot");
    traffic_defaults();
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    if (MESH_N == 2) begin
      src = 1;                           // (1,0) -> (0,1)
      dst = 2;
    end else begin
      src = 2;                           // (2,0) -> (1,2): R2 -> R9 on 4x4
      dst = 2 * MESH_N + 1;
    end
    ok = find_sub(src, dst, sl, k, mode);
    L  = nhops(src, dst);
    `TB_CHECK(ok, ("BP01: the timetable never lets R%0d send to R%0d", src, dst))
    if (MESH_N == 4) begin
      `TB_CHECK(sl == 8 && k == 2 && mode == MODE_XY,
                ("BP01: R2 -> R9 in slot %0d, subslot %0d, mode %0d; Table I and Fig. 3 of [1] say slot 8, subslot 2, XY",
                 sl, k, mode))
    end
    t = sub_start(sl, k, cyc + BYP_LEAD + 2);
    tb_pkg::info($sformatf("BP01: R%0d -> R%0d, %0d hops, slot %0d subslot %0d by %s; the subslot starts at cycle %0d",
                           src, dst, L, sl, k, (mode == MODE_XY) ? "XY" : "YX", t));
    nev0 = (rchk_pkg::the_chk == null) ? 0 : int'(rchk_pkg::the_chk.n_ev);
    at_cycle(t - BYP_LEAD);
    hold_req[src] = 5'b10000;            // Local input: the packet waits in its VC
    traffic_pkg::direct(src, dst, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[src] = '0;                  // off in cycle t: the source's hold must win
    wait_delivered("BP01");
    check_burst("BP01", src, dst, t, MAX_PKT_FLITS, blen);
    // lookaheads on links 2..L, 5 flits on each of the L links, 5 NI writes
    nev = (rchk_pkg::the_chk == null) ? -1 : int'(rchk_pkg::the_chk.n_ev) - nev0;
    `TB_CHECK(nev == (L - 1) + MAX_PKT_FLITS * L + MAX_PKT_FLITS,
              ("BP01: the checker matched %0d lookahead, link and NI events, expected %0d", nev,
               (L - 1) + MAX_PKT_FLITS * L + MAX_PKT_FLITS))
    `TB_CHECK(traffic_pkg::dlog.size() == 1, ("BP01: %0d packets delivered, expected 1", traffic_pkg::dlog.size()))
    if (traffic_pkg::dlog.size() == 1) begin
      tb_pkg::info($sformatf("BP01: created at cycle %0d, burst of %0d flits decided at %0d, lookahead on link 1 at %0d, flit 0 on link 1 at %0d, tail in R%0d's NI at %0d: latency %0d cycles (regular zero-load %0d)",
                             traffic_pkg::dlog[0].t_create, blen, t, t + sched_pkg::tc_la(1), t + sched_pkg::tc_link(1, 0),
                             dst, traffic_pkg::dlog[0].t_tail, traffic_pkg::dlog[0].t_tail - traffic_pkg::dlog[0].t_create,
                             zero_load(L, MAX_PKT_FLITS)));
    end
    traffic_pkg::log_on = 1'b0;
    check_idle("BP01");
    traffic_defaults();
  endtask

  // ---------------------------------------------------------------- BP02
  // Action a of BP02: a < 4 injects pair a, a >= 4 releases pair a - 4
  function automatic int unsigned act_t(int unsigned t [4], int a);
    if (a < 4) return t[a] - BYP_LEAD;
    return t[a - 4] - 1;
  endfunction

  task automatic bp02();
    int          src [4], dst [4], len2 [4], sl [4], k [4], blen, ord [8], tmp, exp;
    bit          mode [4], ok;
    int unsigned t [4];
    tb_pkg::phase($sformatf("BP02 corner to corner, both ways: %0d hops, two-packet bursts", 2 * (MESH_N - 1)));
    traffic_defaults();
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    src  = '{0, NN - 1, MESH_N - 1, NN - MESH_N};
    dst  = '{NN - 1, 0, NN - MESH_N, MESH_N - 1};
    len2 = '{MAX_PKT_FLITS, 1, MAX_PKT_FLITS, 1};      // second packet: bursts of 10 and 6 flits
    for (int i = 0; i < 4; i++) begin
      ok = find_sub(src[i], dst[i], sl[i], k[i], mode[i]);
      `TB_CHECK(ok && nhops(src[i], dst[i]) == 2 * (MESH_N - 1),
                ("BP02: R%0d -> R%0d not in the timetable or not a longest path", src[i], dst[i]))
      t[i] = sub_start(sl[i], k[i], cyc + BYP_LEAD + 2);
      tb_pkg::info($sformatf("BP02: R%0d -> R%0d in slot %0d subslot %0d by %s, at cycle %0d: %0d + %0d flits",
                             src[i], dst[i], sl[i], k[i], (mode[i] == MODE_XY) ? "XY" : "YX", t[i],
                             MAX_PKT_FLITS, len2[i]));
    end
    // each pair: inject BYP_LEAD cycles before its subslot, release the hold
    // at the edge that starts it; the eight actions in time order
    for (int i = 0; i < 8; i++) ord[i] = i;
    for (int i = 0; i < 8; i++) begin
      for (int j = 0; j < 7 - i; j++) begin
        if (act_t(t, ord[j]) > act_t(t, ord[j + 1])) begin
          tmp        = ord[j];
          ord[j]     = ord[j + 1];
          ord[j + 1] = tmp;
        end
      end
    end
    for (int i = 0; i < 8; i++) begin
      at_cycle(act_t(t, ord[i]));
      if (ord[i] < 4) begin
        hold_req[src[ord[i]]] = 5'b10000;
        traffic_pkg::direct(src[ord[i]], dst[ord[i]], MAX_PKT_FLITS);
        traffic_pkg::direct(src[ord[i]], dst[ord[i]], len2[ord[i]]);
      end else begin
        hold_req[src[ord[i] - 4]] = '0;
      end
    end
    wait_delivered("BP02");
    for (int i = 0; i < 4; i++) begin
      // with one VC only the first packet waits in the router; a pair that
      // does not fit in B_MAX goes as the scan's start decides
      if (NUM_VC == 1)                          exp = MAX_PKT_FLITS;
      else if (MAX_PKT_FLITS + len2[i] <= B_MAX) exp = MAX_PKT_FLITS + len2[i];
      else                                      exp = 0;
      check_burst($sformatf("BP02 R%0d->R%0d", src[i], dst[i]), src[i], dst[i], t[i], exp, blen);
      tb_pkg::info($sformatf("BP02: R%0d -> R%0d: %0d flits left at cycle %0d, the last in R%0d's NI at cycle %0d (t + %0d; the subslot has %0d cycles)",
                             src[i], dst[i], blen, t[i], dst[i], t[i] + sched_pkg::tc_eject(2 * (MESH_N - 1), blen - 1),
                             sched_pkg::tc_eject(2 * (MESH_N - 1), blen - 1), BT));
    end
    `TB_CHECK(traffic_pkg::dlog.size() == 8, ("BP02: %0d packets delivered, expected 8", traffic_pkg::dlog.size()))
    check_idle("BP02");
    if (MESH_N >= 3) bp02b();
    traffic_pkg::log_on = 1'b0;
    traffic_defaults();
  endtask

  // A burst of b flits can be made of whole packets waiting in NUM_VC VCs
  // with one outcome only: n5 five-flit and n1 one-flit packets, all fitting
  function automatic bit burst_mix(int b, output int n5, output int n1);
    for (int a = 2; a >= 0; a--) begin
      if (b - MAX_PKT_FLITS * a >= 0 && a + (b - MAX_PKT_FLITS * a) <= NUM_VC && b <= B_MAX) begin
        n5 = a;
        n1 = b - MAX_PKT_FLITS * a;
        return 1'b1;
      end
    end
    n5 = 0;
    n1 = 0;
    return 1'b0;
  endfunction

  // Bursts A (subslot k) and B (subslot k + 1 of the same slot) on one link,
  // A's link i_A and B's link i_B, with T + 2 i_B = 2 i_A + B_A
  function automatic bit find_abut(output int sl, output int k, output int sa, output int da, output int sb,
                                   output int db, output int ba, output int ia, output int ib);
    sched_ref_pkg::xfer_q_t qa, qb;
    sched_ref_pkg::link_q_t pa, pb;
    int                     n5, n1;
    for (int s = 0; s < BNS; s++) begin
      for (int kk = 0; kk + 1 < MESH_N; kk++) begin
        qa = sched_ref_pkg::ref_xfers(MESH_N, s, kk, 1'b0);
        qb = sched_ref_pkg::ref_xfers(MESH_N, s, kk + 1, 1'b0);
        foreach (qa[m]) begin
          pa = sched_ref_pkg::ref_path(qa[m].sx, qa[m].sy, qa[m].dx, qa[m].dy, qa[m].mode);
          foreach (qb[m2]) begin
            pb = sched_ref_pkg::ref_path(qb[m2].sx, qb[m2].sy, qb[m2].dx, qb[m2].dy, qb[m2].mode);
            foreach (pa[h]) begin
              foreach (pb[h2]) begin
                if (pa[h].x == pb[h2].x && pa[h].y == pb[h2].y && pa[h].p == pb[h2].p &&
                    BT + 2 * (h2 + 1) - 2 * (h + 1) >= 1 &&
                    burst_mix(BT + 2 * (h2 + 1) - 2 * (h + 1), n5, n1) && n5 + n1 >= 1) begin
                  sl = s;
                  k  = kk;
                  sa = qa[m].sy * MESH_N + qa[m].sx;
                  da = qa[m].dy * MESH_N + qa[m].dx;
                  sb = qb[m2].sy * MESH_N + qb[m2].sx;
                  db = qb[m2].dy * MESH_N + qb[m2].dx;
                  ia = h + 1;
                  ib = h2 + 1;
                  ba = BT + 2 * ib - 2 * ia;
                  return 1'b1;
                end
              end
            end
          end
        end
      end
    end
    return 1'b0;
  endfunction

  task automatic bp02b();
    int              sl, k, sa, da, sb, db, ba, ia, ib, n5, n1, blen;
    bit              ok;
    int unsigned     t;
    longint unsigned a0;
    traffic_pkg::dlog.delete();
    ok = find_abut(sl, k, sa, da, sb, db, ba, ia, ib);
    if (!ok) begin
      tb_pkg::info($sformatf("BP02 back to back: no two bursts of consecutive subslots can meet back to back on a link in this timetable (%0dx%0d, T = %0d)",
                             MESH_N, MESH_N, BT));
      return;
    end
    ok = burst_mix(ba, n5, n1);
    t  = sub_start(sl, k, cyc + BYP_LEAD + 2);
    tb_pkg::info($sformatf("BP02 back to back: slot %0d subslot %0d at cycle %0d: R%0d -> R%0d (%0d flits) uses on its link %0d the link that R%0d -> R%0d uses as its link %0d in the next subslot",
                           sl, k, t, sa, da, ba, ia, sb, db, ib));
    a0 = sum_of(v_abut);
    at_cycle(t - BYP_LEAD);
    hold_req[sa] = 5'b10000;
    for (int i = 0; i < n5; i++) traffic_pkg::direct(sa, da, MAX_PKT_FLITS);
    for (int i = 0; i < n1; i++) traffic_pkg::direct(sa, da, 1);
    at_cycle(t + BT - BYP_LEAD);
    hold_req[sb] = 5'b10000;
    traffic_pkg::direct(sb, db, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[sa] = '0;
    at_cycle(t + BT - 1);
    hold_req[sb] = '0;
    wait_delivered("BP02 back to back");
    check_burst($sformatf("BP02 R%0d->R%0d", sa, da), sa, da, t, ba, blen);
    tb_pkg::info($sformatf("BP02 back to back: A's last flit crosses the shared link's far router at cycle %0d (t + %0d), B's lookahead reaches it then",
                           t + 2 * ia + blen, 2 * ia + blen));
    check_burst($sformatf("BP02 R%0d->R%0d", sb, db), sb, db, t + BT, MAX_PKT_FLITS, blen);
    tb_pkg::cover_bin("BP02 bursts back to back on one link", sum_of(v_abut) - a0);
    check_idle("BP02 back to back");
  endtask

  // ---------------------------------------------------------------- BP04
  // The burst of src at t against the packets that were waiting for dst:
  // whole packets of them, at most B_MAX flits, and nothing left out fits
  task automatic check_maximal(string what, int src, int unsigned t, int nw, int wsrc [8], int wseq [8], int wlen [8],
                               output int n_from);
    int  np, psrc [16], pseq [16], plen [16], sum, left_min;
    bit  in_burst;
    np       = get_burst_pkts(src, t, psrc, pseq, plen);
    sum      = 0;
    left_min = 1000;
    n_from   = 0;
    for (int i = 0; i < np; i++) sum += plen[i];
    for (int w = 0; w < nw; w++) begin
      in_burst = 1'b0;
      for (int i = 0; i < np; i++) begin
        if (psrc[i] == wsrc[w] && pseq[i] == wseq[w]) in_burst = 1'b1;
      end
      if (!in_burst && wlen[w] < left_min) left_min = wlen[w];
      if (in_burst && wsrc[w] != src) n_from++;
    end
    for (int i = 0; i < np; i++) begin
      in_burst = 1'b0;
      for (int w = 0; w < nw; w++) begin
        if (psrc[i] == wsrc[w] && pseq[i] == wseq[w]) in_burst = 1'b1;
      end
      `TB_CHECK(in_burst, ("%s: the burst carries packet %0d.%0d, which was not waiting", what, psrc[i], pseq[i]))
    end
    `TB_CHECK(np >= 1 && sum <= B_MAX && (left_min == 1000 || sum + left_min > B_MAX),
              ("%s: the burst takes %0d packets, %0d flits (B_MAX %0d); the shortest packet left waiting has %0d flits",
               what, np, sum, B_MAX, left_min))
    tb_pkg::info($sformatf("%s: %0d of %0d waiting packets sent, %0d flits; %0d of them from another router's packets",
                           what, np, nw, sum, n_from));
  endtask

  task automatic bp04();
    int          s, d, e, sl, k, blen, nw, n_from, exp;
    int          wsrc [8], wseq [8], wlen [8];
    bit          mode, ok;
    int unsigned t;
    tb_pkg::phase($sformatf("BP04 the burst limit: more eligible flits than B_MAX = %0d", B_MAX));
    traffic_defaults();
    traffic_pkg::dlog.delete();
    traffic_pkg::log_on = 1'b1;
    // a) four 5-flit packets waiting at the source's Local input
    s  = (MESH_N / 2) * MESH_N + MESH_N / 2;
    d  = 0;
    ok = find_sub(s, d, sl, k, mode);
    `TB_CHECK(ok, ("BP04: the timetable never lets R%0d send to R%0d", s, d))
    t  = sub_start(sl, k, cyc + BYP_LEAD + 2);
    tb_pkg::info($sformatf("BP04 a: R%0d -> R%0d at cycle %0d (slot %0d subslot %0d, %s): %0d five-flit packets waiting",
                           s, d, t, sl, k, (mode == MODE_XY) ? "XY" : "YX", NUM_VC));
    at_cycle(t - BYP_LEAD);
    hold_req[s] = 5'b10000;
    for (int i = 0; i < NUM_VC; i++) traffic_pkg::direct(s, d, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[s] = '0;
    wait_delivered("BP04 a");
    // all 5-flit packets: as many as wait and fit, whatever the scan's start
    exp = B_MAX / MAX_PKT_FLITS;
    if (exp > NUM_VC) exp = NUM_VC;
    check_burst("BP04 a", s, d, t, exp * MAX_PKT_FLITS, blen);
    nw = 0;
    foreach (traffic_pkg::dlog[i]) begin
      if (nw < 8) begin
        wsrc[nw] = int'(traffic_pkg::dlog[i].src);
        wseq[nw] = int'(traffic_pkg::dlog[i].seq);
        wlen[nw] = int'(traffic_pkg::dlog[i].len);
        nw++;
      end
    end
    check_maximal("BP04 a", s, t, nw, wsrc, wseq, wlen, n_from);
    // the packets left out waited, then went by the regular path or by a
    // later burst from a router on their path
    nw = 0;
    foreach (traffic_pkg::dlog[i]) if (!traffic_pkg::dlog[i].byp) nw++;
    `TB_CHECK(traffic_pkg::dlog.size() == NUM_VC, ("BP04 a: %0d packets delivered, expected %0d",
                                                   traffic_pkg::dlog.size(), NUM_VC))
    tb_pkg::info($sformatf("BP04 a: the %0d packets left out arrived later, %0d of them by the regular path",
                           NUM_VC - blen / MAX_PKT_FLITS, nw));
    check_idle("BP04 a");
    // b) packets for the partner at the source's East and Local inputs
    if (MESH_N >= 3) begin
      traffic_pkg::dlog.delete();
      s  = (MESH_N - 1) * MESH_N + 1;    // (1, n-1)
      d  = s - 1;                        // its west neighbour
      e  = s + 1;                        // its east neighbour: its packets for d pass s
      ok = find_sub(s, d, sl, k, mode);
      `TB_CHECK(ok, ("BP04: the timetable never lets R%0d send to R%0d", s, d))
      t  = sub_start(sl, k, cyc + BYP_LEAD + 2);
      tb_pkg::info($sformatf("BP04 b: R%0d -> R%0d at cycle %0d; R%0d's packets for R%0d wait in R%0d's East input",
                             s, d, t, e, d, s));
      at_cycle(t - BYP_LEAD);
      hold_req[s] = 5'b10010;            // East and Local inputs
      traffic_pkg::direct(e, d, MAX_PKT_FLITS);
      traffic_pkg::direct(e, d, 1);
      traffic_pkg::direct(s, d, MAX_PKT_FLITS);
      at_cycle(t - 1);
      hold_req[s] = '0;
      wait_delivered("BP04 b");
      check_burst("BP04 b", s, d, t, 0, blen);
      nw = 0;
      foreach (traffic_pkg::dlog[i]) begin
        if (nw < 8) begin
          wsrc[nw] = int'(traffic_pkg::dlog[i].src);
          wseq[nw] = int'(traffic_pkg::dlog[i].seq);
          wlen[nw] = int'(traffic_pkg::dlog[i].len);
          nw++;
        end
      end
      `TB_CHECK(nw == 3, ("BP04 b: %0d packets delivered, expected 3", nw))
      check_maximal("BP04 b", s, t, nw, wsrc, wseq, wlen, n_from);
      // with B_MAX >= 10 every start of the scan takes one of the East packets
      // (with B_MAX 5 the burst holds one packet, and the scan's start decides
      // which: a goal there, not a bin)
      if (B_MAX >= 2 * MAX_PKT_FLITS) begin
        `TB_CHECK(n_from >= 1, ("BP04 b: the burst took no packet from R%0d's East input", s))
        tb_pkg::cover_bin("BP04 packets bypassed from a mesh input of a router on their path", longint'(n_from));
      end else begin
        tb_pkg::cover_goal("BP04 packets bypassed from a mesh input of a router on their path", longint'(n_from));
      end
      check_idle("BP04 b");
      bp04c();
    end
    if (BT == B_MAX && NUM_VC >= 3) bp04d();
    traffic_pkg::log_on = 1'b0;
    traffic_defaults();
  endtask

  // A router that sends in subslots k and k + 1 of one slot
  function automatic bit find_two_turns(output int sl, output int k, output int sr, output int d1, output int d2);
    sched_ref_pkg::xfer_q_t qa, qb;
    for (int s = 0; s < BNS; s++) begin
      for (int kk = 0; kk + 1 < MESH_N; kk++) begin
        qa = sched_ref_pkg::ref_xfers(MESH_N, s, kk, 1'b0);
        qb = sched_ref_pkg::ref_xfers(MESH_N, s, kk + 1, 1'b0);
        foreach (qa[m]) begin
          foreach (qb[m2]) begin
            if (qa[m].sx == qb[m2].sx && qa[m].sy == qb[m2].sy) begin
              sl = s;
              k  = kk;
              sr = qa[m].sy * MESH_N + qa[m].sx;
              d1 = qa[m].dy * MESH_N + qa[m].dx;
              d2 = qb[m2].dy * MESH_N + qb[m2].dx;
              return 1'b1;
            end
          end
        end
      end
    end
    return 1'b0;
  endfunction

  task automatic bp04d();
    int              sl, k, sr, d1, d2, blen;
    bit              ok;
    int unsigned     t;
    longint unsigned z0;
    traffic_pkg::dlog.delete();
    ok = find_two_turns(sl, k, sr, d1, d2);
    `TB_CHECK(ok, ("BP04 d: no router sends in two consecutive subslots"))
    if (!ok) return;
    t  = sub_start(sl, k, cyc + BYP_LEAD + 2);
    tb_pkg::info($sformatf("BP04 d: R%0d sends %0d flits to R%0d at cycle %0d and chooses its burst to R%0d at cycle %0d, as the last flit leaves",
                           sr, B_MAX, d1, t, d2, t + BT));
    z0 = sum_of(v_b2b);
    at_cycle(t - BYP_LEAD);
    hold_req[sr] = 5'b10000;
    traffic_pkg::direct(sr, d1, MAX_PKT_FLITS);
    traffic_pkg::direct(sr, d1, MAX_PKT_FLITS);
    traffic_pkg::direct(sr, d2, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[sr] = '0;
    wait_delivered("BP04 d");
    check_burst($sformatf("BP04 d R%0d->R%0d", sr, d1), sr, d1, t, 2 * MAX_PKT_FLITS, blen);
    tb_pkg::info($sformatf("BP04 d: the first burst's last flit left R%0d at cycle %0d", sr, t + sched_pkg::tc_xbar(blen - 1)));
    check_burst($sformatf("BP04 d R%0d->R%0d", sr, d2), sr, d2, t + BT, MAX_PKT_FLITS, blen);
    tb_pkg::cover_bin("BP04 bursts chosen as the previous burst sent its last flit", sum_of(v_b2b) - z0);
    check_idle("BP04 d");
  endtask

  // A subslot in which X sends to Y through S (S right after X) while S
  // sends to D, D straight beyond S as seen from X; 0 if none
  function automatic bit find_rule_case(output int sl, output int k, output int x, output int y, output int sr,
                                        output int d, output int pin);
    sched_ref_pkg::xfer_q_t q;
    sched_ref_pkg::link_q_t pth;
    int                     ex, ey, sx, sy, dx, dy;
    for (int s = 0; s < BNS; s++) begin
      for (int kk = 0; kk < MESH_N; kk++) begin
        q = sched_ref_pkg::ref_xfers(MESH_N, s, kk, 1'b0);
        foreach (q[m]) begin
          pth = sched_ref_pkg::ref_path(q[m].sx, q[m].sy, q[m].dx, q[m].dy, q[m].mode);
          if (pth.size() >= 2) begin
            sched_ref_pkg::ref_link_end(pth[0], ex, ey);          // S
            foreach (q[n2]) begin
              if (q[n2].sx == ex && q[n2].sy == ey) begin
                sx = q[m].sx;
                sy = q[m].sy;
                dx = q[n2].dx;
                dy = q[n2].dy;
                // X's packet for D leaves X towards S
                if ((pth[0].p == int'(P_E) && dx >= ex) || (pth[0].p == int'(P_W) && dx <= ex) ||
                    (pth[0].p == int'(P_S) && dx == sx && dy > ey) || (pth[0].p == int'(P_N) && dx == sx && dy < ey)) begin
                  sl  = s;
                  k   = kk;
                  x   = sy * MESH_N + sx;
                  y   = q[m].dy * MESH_N + q[m].dx;
                  sr  = ey * MESH_N + ex;
                  d   = dy * MESH_N + dx;
                  pin = sched_ref_pkg::ref_opposite(pth[0].p);
                  return 1'b1;
                end
              end
            end
          end
        end
      end
    end
    return 1'b0;
  endfunction

  task automatic bp04c();
    int          sl, k, x, y, sr, d, pin, blen, exp;
    bit          ok;
    int unsigned t;
    longint unsigned u0;
    traffic_pkg::dlog.delete();
    ok = find_rule_case(sl, k, x, y, sr, d, pin);
    `TB_CHECK(ok, ("BP04 c: no subslot where a source's mesh input is crossed by a passing burst"))
    if (!ok) return;
    t  = sub_start(sl, k, cyc + BYP_LEAD + 22);
    tb_pkg::info($sformatf("BP04 c: slot %0d subslot %0d at cycle %0d: R%0d -> R%0d passes R%0d, entering on its %s input, while R%0d sends to R%0d a packet from R%0d waiting in that same input",
                           sl, k, t, x, y, sr, tb_pkg::port_name(port_t'(pin)), sr, d, x));
    u0 = sum_of(v_rule);
    // the packet for D first travels from X to S's input and waits there
    at_cycle(t - BYP_LEAD - 20);
    hold_req[sr] = NPORTS'(1) << pin;
    traffic_pkg::direct(x, d, MAX_PKT_FLITS);
    // then X's packets for Y wait at X's Local input
    at_cycle(t - BYP_LEAD);
    hold_req[x] = 5'b10000;
    traffic_pkg::direct(x, y, MAX_PKT_FLITS);
    traffic_pkg::direct(x, y, MAX_PKT_FLITS);
    at_cycle(t - 1);
    hold_req[sr] = '0;
    hold_req[x]  = '0;
    wait_delivered("BP04 c");
    exp = B_MAX / MAX_PKT_FLITS;
    if (exp > NUM_VC) exp = NUM_VC;
    if (exp > 2)      exp = 2;
    check_burst($sformatf("BP04 c R%0d->R%0d", sr, d), sr, d, t, MAX_PKT_FLITS, blen);
    check_burst($sformatf("BP04 c R%0d->R%0d", x, y), x, y, t, exp * MAX_PKT_FLITS, blen);
    tb_pkg::info($sformatf("BP04 c: R%0d's burst of %0d flits crossed R%0d at cycles %0d-%0d while R%0d's own flits left at %0d-%0d",
                           x, blen, sr, t + sched_pkg::tc_link(1, 0), t + sched_pkg::tc_link(1, blen - 1), sr,
                           t + sched_pkg::tc_xbar(0), t + sched_pkg::tc_xbar(MAX_PKT_FLITS - 1)));
    tb_pkg::cover_bin("BP04 cycles a source read a mesh input that a passing burst crossed (D38 rule)", sum_of(v_rule) - u0);
    check_idle("BP04 c");
  endtask

  // ---------------------------------------------------------------- BP05
  task automatic bp05();
    int unsigned     meas;
    longint unsigned b0, f0, p0, m0, r0, u0, w0, x0, a0, z0, ck0, ckb0;
    real             share;
    meas = quick() ? 1000 : scaled(4000);
    // at least one timetable round, so every ordered pair has its turn under
    // load (4x4: 790 cycles; 8x8: 10,332, where the scaled length is 1,000)
    if (meas < sched_pkg::round_len(MESH_N, sched_pkg::t_sub(MESH_N, B_MAX, T_MODE)))
      meas = sched_pkg::round_len(MESH_N, sched_pkg::t_sub(MESH_N, B_MAX, T_MODE));
    tb_pkg::phase($sformatf("BP05 bypass under heavy traffic: uniform at 0.9 x %0.3f packets/node/cycle, %0d cycles",
                            sat_rate(), meas));
    traffic_defaults();
    traffic_pkg::rate = 0.9 * sat_rate();
    b0   = sum_of(v_used);
    f0   = sum_of(v_bflits);
    p0   = sum_of(v_bpkts);
    m0   = sum_of(v_multi);
    r0   = sum_of(v_remote);
    u0   = sum_of(v_rule);
    w0   = sum_of(v_turns);
    x0   = sum_of(v_full);
    a0   = sum_of(v_abut);
    z0   = sum_of(v_b2b);
    ck0  = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_ev;
    ckb0 = (rchk_pkg::the_chk == null) ? 0 : rchk_pkg::the_chk.n_bursts;
    load_phase("BP05", meas / 4, meas);
    report("BP05");
    share = stats_pkg::byp_pkt();
    tb_pkg::info($sformatf("BP05: %0d bursts in %0d turns, %0d packets (%0d flits) by bypass; %0d bursts of several packets, %0d of exactly B_MAX flits; %0d packets taken from mesh inputs; bypass share %0.1f%% of packets, %0.1f%% of flits",
                           sum_of(v_used) - b0, sum_of(v_turns) - w0, sum_of(v_bpkts) - p0, sum_of(v_bflits) - f0,
                           sum_of(v_multi) - m0, sum_of(v_full) - x0, sum_of(v_remote) - r0, 100.0 * share,
                           100.0 * stats_pkg::byp_flit()));
    if (rchk_pkg::the_chk != null) begin
      `TB_CHECK(rchk_pkg::the_chk.n_bursts - ckb0 == sum_of(v_used) - b0,
                ("BP05: the checker saw %0d bursts, the sources sent %0d", rchk_pkg::the_chk.n_bursts - ckb0, sum_of(v_used) - b0))
      tb_pkg::info($sformatf("BP05: the checker matched %0d lookahead, link and NI events at their D38 cycles",
                             rchk_pkg::the_chk.n_ev - ck0));
    end
    tb_pkg::cover_bin("BP05 bursts", sum_of(v_used) - b0);
    // goals, not checks: random traffic makes them rare in short or small
    // runs; BP02 and BP04 make both happen on purpose
    tb_pkg::cover_goal("BP05 bursts of two or more packets", sum_of(v_multi) - m0);
    tb_pkg::cover_goal("BP05 packets bypassed from a mesh input", sum_of(v_remote) - r0);
    tb_pkg::info($sformatf("BP05: corner cases: a source read a mesh input while a passing burst crossed it in %0d cycles (D38 rule, allowed); %0d bursts arrived on a link the cycle the previous one's last flit crossed; %0d bursts were chosen while the source's previous burst sent its last flit",
                           sum_of(v_rule) - u0, sum_of(v_abut) - a0, sum_of(v_b2b) - z0));
    traffic_defaults();
  endtask
