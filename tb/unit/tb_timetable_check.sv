// =============================================================================
// File      tb/unit/tb_timetable_check.sv                          Tracker: F40
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Tests     U08: the paths of one subslot never share a directed link, for both
//                directions inside one group; every router sends and receives
//                at most once per subslot; every ordered pair meets once per
//                round; local paths stay inside their region
//           U09: no two flits, lookaheads or cancel tokens use the same link,
//                and no two flits the same NI write port, in any cycle, over
//                full rounds and every local/global slot transition, with
//                every burst at B_MAX; SAFE and TIGHT; prints the TIGHT minimum
// Model     tb/common/sched_ref.sv (F36) for the bursts and paths; the cycle
//           offsets of one bypass are the contract of rtl/pkg/sched_pkg.sv
//           (D38). U05-U07 prove the RTL schedule equals sched_ref exactly, so
//           a collision-free sched_ref means a collision-free RTL schedule.
// @tests    U08 U09
// -----------------------------------------------------------------------------
// Worst case everywhere: every router of the active groups sends a burst of
// B_MAX flits in every subslot, and every destination sends a cancel token
// for every subslot. A shorter burst or a missing token only frees resources,
// so this covers every traffic.
// Resources per router (indices of the occupancy table):
//   0-3  flit link out of port N, E, S, W     4     the NI write port (ejection)
//   5-8  lookahead link out of N, E, S, W     9-12  cancel link out of N, E, S, W
//   13   the source's bypass read (one bypass flit per cycle leaves a source)
// A cancel token must reach its source before its subslot starts (D38); a
// late one counts as a failure like a collision.
// U09 steps:
//   1. For n = 2, 4, 8, 16 and B_MAX = 5, 10, 15: try T = 1, 2, ... on a full
//      round of design B, including the turn into the next round, until the
//      first T with no collision and no late token: the TIGHT minimum. It must
//      equal sched_pkg::t_tight, the value the RTL uses. Every T from TIGHT to
//      SAFE is then checked too (on 16x16 only TIGHT and SAFE unless
//      +U09_ALL), so the collision-free lengths form one range.
//   2. Frames (designs C, D): with the package's TIGHT and with SAFE lengths,
//      a full local round (every region at once), a full global round, and
//      every local slot followed by every global slot and every global slot
//      by every local slot (only subslots that can reach across the boundary
//      are simulated). Regions of 4x4 on 8x8 and 16x16; with +U09_ALL also
//      2x2 regions on 8x8 and 16x16 and 8x8 regions on 16x16.
// +QUICK (EDA Playground's 60 s): U08 without the 16x16 timetable; U09 step 1
// for n = 2, 4, 8 only, step 2 for 4x4 regions on 8x8 with B_MAX = 10.
// The checker is a class so that Verilator compiles each method once.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package tt_check_pkg;
  import noc_pkg::*;
  import sched_pkg::*;
  import sched_ref_pkg::*;

  localparam int MAXR  = 256;         // routers of a 16x16 mesh
  localparam int RK    = 16;          // resources per router
  localparam int W     = 256;         // cycles remembered per resource
  localparam int K_FL  = 0;
  localparam int K_EJ  = 4;
  localparam int K_LA  = 5;
  localparam int K_CX  = 9;
  localparam int K_SRC = 13;

  class tt_check;
    int    stamp [];                  // [res * W + cycle mod W]: cycle + 1 of the last use (0: never)
    int    owner [];                  // the burst that used it
    int    now;                       // every run starts after every earlier mark
    int    mesh;                      // mesh of the run, for router ids
    int    n_coll, n_late;
    bit    stop_first;                // stop a run at its first failure
    string first_fail;
    bit    all_cfg;                   // +U09_ALL
    // U08
    int    seen [];                   // tag of the subslot that last used a link
    int    sent [], recv [];
    int    served [];                 // [src * MAXR + dst]
    int    tag;

    function new();
      stamp   = new[MAXR * RK * W];
      owner   = new[MAXR * RK * W];
      seen    = new[MAXR * 4];
      sent    = new[MAXR];
      recv    = new[MAXR];
      served  = new[MAXR * MAXR];
      now     = 0;
      mesh    = 4;
      tag     = 0;
      all_cfg = 1'b0;
      reset_counts();
    endfunction

    // -------------------------------------------------------------------------
    // Names for messages
    // -------------------------------------------------------------------------
    function string rname(int r);
      return $sformatf("R%0d (%0d,%0d)", r, r % mesh, r / mesh);
    endfunction

    function string who_name(int w);
      return $sformatf("R%0d->R%0d (slot %0d, subslot %0d)", (w >> 8) & 255, w & 255, w >> 20, (w >> 16) & 15);
    endfunction

    function string res_name(int res);
      int    r, k;
      string s;
      r = res / RK;
      k = res % RK;
      if (k < K_EJ)       s = $sformatf("flit link %s of %s", tb_pkg::port_name(3'(k)), rname(r));
      else if (k == K_EJ) s = $sformatf("NI write port of %s", rname(r));
      else if (k < K_CX)  s = $sformatf("lookahead link %s of %s", tb_pkg::port_name(3'(k - K_LA)), rname(r));
      else if (k < K_SRC) s = $sformatf("cancel link %s of %s", tb_pkg::port_name(3'(k - K_CX)), rname(r));
      else                s = $sformatf("bypass read of %s", rname(r));
      return s;
    endfunction

    function void reset_counts();
      n_coll     = 0;
      n_late     = 0;
      first_fail = "";
    endfunction

    function bit failed();
      return (n_coll != 0) || (n_late != 0);
    endfunction

    // -------------------------------------------------------------------------
    // Occupancy
    // -------------------------------------------------------------------------
    function void use_res(int res, int cyc, int who, int base);
      int unsigned i;
      i = 32'(res) * 32'(W) + 32'(cyc % W);       // res, cyc >= 0
      if (stamp[i] == cyc + 1) begin
        n_coll++;
        if (first_fail == "")
          first_fail = $sformatf("%s used twice at cycle +%0d, by %s and %s",
                                 res_name(res), cyc - base, who_name(owner[i]), who_name(who));
      end else begin
        stamp[i] = cyc + 1;
        owner[i] = who;
      end
    endfunction

    // Every resource the bursts of subslot k use. n: timetable side; the
    // timetable runs in every n x n region of the mesh at once. t: the
    // subslot's first cycle; d: the cycle its cancel tokens are decided.
    function void run_subslot(int n, int slot, int k, int t, int d, int b_max, int base);
      xfer_q_t q;
      link_q_t p;
      int      len, src, dst, who, ux, uy, r0;
      q = ref_xfers(n, slot, k, 1'b0);
      for (int by = 0; by < mesh; by += n) begin
        for (int bx = 0; bx < mesh; bx += n) begin
          foreach (q[m]) begin
            p   = ref_path(bx + q[m].sx, by + q[m].sy, bx + q[m].dx, by + q[m].dy, q[m].mode);
            len = p.size();
            src = (by + q[m].sy) * mesh + bx + q[m].sx;
            dst = (by + q[m].dy) * mesh + bx + q[m].dx;
            who = (slot << 20) | (k << 16) | (src << 8) | dst;
            // cancel token, from the destination back along the path: reverse
            // link h leaves the far end of forward link len-h+1 the other way
            for (int h = 1; h <= len; h++) begin
              ref_link_end(p[len - h], ux, uy);
              use_res((uy * mesh + ux) * RK + K_CX + ref_opposite(p[len - h].p), d + tc_cancel(h), who, base);
            end
            if (d + len + 1 > t) begin
              n_late++;
              if (first_fail == "")
                first_fail = $sformatf("the cancel token for %s reaches the source at cycle +%0d, after its subslot starts at +%0d",
                                       who_name(who), d + len + 1 - base, t - base);
            end
            for (int j = 0; j < b_max; j++) use_res(src * RK + K_SRC, t + tc_xbar(j), who, base);
            for (int i = 1; i <= len; i++) begin
              r0 = p[i - 1].y * mesh + p[i - 1].x;
              use_res(r0 * RK + K_LA + p[i - 1].p, t + tc_la(i), who, base);
              for (int j = 0; j < b_max; j++) use_res(r0 * RK + K_FL + p[i - 1].p, t + tc_link(i, j), who, base);
            end
            for (int j = 0; j < b_max; j++) use_res(dst * RK + K_EJ, t + tc_eject(len, j), who, base);
          end
        end
      end
    endfunction

    // A full round of one timetable, then the first slot of the next round
    function void run_round(int n, int tsub, int b_max);
      int base, ts, ns, sl, s_set, t, d;
      base  = now + W;
      ts    = base;
      ns    = n_slots(n);
      s_set = s_setup(n);
      for (int i = 0; i <= ns; i++) begin
        sl = i % ns;
        for (int k = 0; k < n; k++) begin
          t = ts + s_set + k * tsub;
          d = (k == 0) ? t - s_set : t - tsub;
          run_subslot(n, sl, k, t, d, b_max, base);
          if (stop_first && failed()) begin
            now = t + 4 * W;
            return;
          end
        end
        ts = ts + s_set + n * tsub;
      end
      now = ts + 4 * W;
    endfunction

    // Slot sx of timetable nx followed directly by slot sy of timetable ny.
    // Only subslots of X whose bursts may still move after X ends, and
    // subslots of Y whose first resource use comes before X's last one, can meet.
    function void run_pair(int nx, int sx, int tx, int ny, int sy, int ty, int b_max);
      int base, x_end, x_last, t, d, sxs, sys, safe_x;
      base   = now + W;
      sxs    = s_setup(nx);
      sys    = s_setup(ny);
      safe_x = t_safe(nx, b_max);
      x_end  = base + sxs + nx * tx;
      x_last = x_end - tx + safe_x;              // last use by X's last subslot
      for (int k = 0; k < nx; k++) begin
        t = base + sxs + k * tx;
        if (t + safe_x >= x_end) run_subslot(nx, sx, k, t, (k == 0) ? t - sxs : t - tx, b_max, base);
      end
      for (int k = 0; k < ny; k++) begin
        t = x_end + sys + k * ty;
        d = (k == 0) ? t - sys : t - ty;
        if (d + 1 <= x_last) run_subslot(ny, sy, k, t, d, b_max, base);
      end
      now = x_end + sys + ny * ty + 4 * W;
    endfunction

    // -------------------------------------------------------------------------
    // U08: spatial checks of every subslot. Timetable side n on an m x m mesh.
    // -------------------------------------------------------------------------
    function void u08_timetable(int n, int m);
      xfer_q_t         q;
      link_q_t         p;
      int              ns, nr, src, dst, ex, ey, bx0, by0, n_pairs, n_bad, n_regions, hops_m;
      int unsigned     lid;
      longint unsigned c_links, c_same;
      mesh    = m;
      ns      = n_slots(n);
      nr      = m * m;
      c_links = 0;
      c_same  = 0;
      for (int yx = 0; yx < 2; yx++) begin
        for (int a = 0; a < MAXR * MAXR; a++) served[a] = 0;
        for (int sl = 0; sl < ns; sl++) begin
          for (int s = 0; s < n; s++) begin
            tag++;
            for (int r = 0; r < nr; r++) begin
              sent[r] = 0;
              recv[r] = 0;
            end
            q = ref_xfers(n, sl, s, yx[0]);
            for (int by = 0; by < m; by += n) begin
              for (int bx = 0; bx < m; bx += n) begin
                foreach (q[i]) begin
                  src    = (by + q[i].sy) * m + bx + q[i].sx;
                  dst    = (by + q[i].dy) * m + bx + q[i].dx;
                  p      = ref_path(bx + q[i].sx, by + q[i].sy, bx + q[i].dx, by + q[i].dy, q[i].mode);
                  hops_m = ((q[i].dx > q[i].sx) ? q[i].dx - q[i].sx : q[i].sx - q[i].dx) +
                           ((q[i].dy > q[i].sy) ? q[i].dy - q[i].sy : q[i].sy - q[i].dy);
                  `TB_CHECK(p.size() == hops_m && src != dst,
                    ("U08 %0dx%0d/%0d slot %0d subslot %0d: path R%0d->R%0d is not minimal", m, m, n, sl, s, src, dst))
                  foreach (p[h]) begin
                    lid = 32'((p[h].y * m + p[h].x) * 4 + p[h].p);
                    `TB_CHECK(seen[lid] != tag,
                      ("U08 %0dx%0d/%0d slot %0d subslot %0d (same group by %s): link %s of R%0d is on two paths",
                       m, m, n, sl, s, yx[0] ? "YX" : "XY", tb_pkg::port_name(3'(p[h].p)), p[h].y * m + p[h].x))
                    seen[lid] = tag;
                    ref_link_end(p[h], ex, ey);
                    bx0 = (p[h].x / n) * n;
                    by0 = (p[h].y / n) * n;
                    `TB_CHECK(ex >= bx0 && ex < bx0 + n && ey >= by0 && ey < by0 + n && bx0 == bx && by0 == by,
                      ("U08 %0dx%0d/%0d slot %0d subslot %0d: path R%0d->R%0d leaves its region", m, m, n, sl, s, src, dst))
                    c_links++;
                  end
                  sent[src]++;
                  recv[dst]++;
                  if (yx == 0) served[src * MAXR + dst]++;
                  if (ref_group(q[i].sx, q[i].sy, n) == ref_group(q[i].dx, q[i].dy, n)) c_same++;
                end
              end
            end
            for (int r = 0; r < nr; r++) begin
              `TB_CHECK(sent[r] <= 1 && recv[r] <= 1,
                ("U08 %0dx%0d/%0d slot %0d subslot %0d: R%0d sends %0d and receives %0d bursts", m, m, n, sl, s, r,
                 sent[r], recv[r]))
            end
          end
        end
        if (yx == 0) begin
          // every ordered pair inside a region exactly once per round, and no other pair
          n_pairs = 0;
          n_bad   = 0;
          for (int a = 0; a < nr; a++) begin
            for (int b = 0; b < nr; b++) begin
              if (a != b && (a % m) / n == (b % m) / n && (a / m) / n == (b / m) / n) begin
                n_pairs++;
                if (served[a * MAXR + b] != 1) n_bad++;
              end else if (served[a * MAXR + b] != 0) begin
                n_bad++;
              end
            end
          end
          n_regions = (m / n) * (m / n);
          `TB_CHECK(n_bad == 0 && n_pairs == n_regions * n * n * (n * n - 1),
            ("U08 %0dx%0d/%0d: %0d ordered pairs not served exactly once per round", m, m, n, n_bad))
        end
      end
      tb_pkg::info($sformatf("U08 %0dx%0d mesh, %0dx%0d timetable in %0d region(s): %0d slots x %0d subslots; paths disjoint (same-group bursts by XY and by YX); every pair once per round",
                             m, m, n, n, (m / n) * (m / n), ns, n));
      tb_pkg::cover_bin($sformatf("U08 %0dx%0d/%0d links checked", m, m, n), c_links);
      tb_pkg::cover_bin($sformatf("U08 %0dx%0d/%0d same-group bursts", m, m, n), c_same);
    endfunction

    // -------------------------------------------------------------------------
    // U09 step 1: the TIGHT minimum of a global timetable (design B)
    // -------------------------------------------------------------------------
    function void u09_tight(int n, int b_max);
      int    tight, safe, pkg, last_fail_t, n_late_t, hm;
      string last_fail;
      bit    contig;
      mesh        = n;
      safe        = t_safe(n, b_max);
      pkg         = t_tight(n, b_max);
      hm          = hmax(n);
      tight       = -1;
      last_fail   = "";
      last_fail_t = -1;
      stop_first  = 1'b1;
      for (int t = 1; t <= safe && tight < 0; t++) begin
        reset_counts();
        run_round(n, t, b_max);
        if (!failed()) begin
          tight = t;
        end else begin
          last_fail   = first_fail;
          last_fail_t = t;
        end
      end
      `TB_CHECK(tight == pkg,
        ("U09 %0dx%0d B_MAX=%0d: the shortest collision-free subslot is %0d cycles, sched_pkg::t_tight says %0d",
         n, n, b_max, tight, pkg))
      tb_pkg::info($sformatf("U09 %0dx%0d B_MAX=%0d: TIGHT = %0d cycles (SAFE %0d, Hmax %0d); slot %0d, round %0d cycles",
                             n, n, b_max, tight, safe, hm, slot_len(n, tight), round_len(n, tight)));
      if (last_fail_t > 0)
        tb_pkg::info($sformatf("U09 %0dx%0d B_MAX=%0d: at T=%0d, %s", n, n, b_max, last_fail_t, last_fail));
      // A late cancel token is caught: at T = Hmax the longest path is one cycle short
      reset_counts();
      run_round(n, hm, b_max);
      n_late_t = n_late;
      tb_pkg::cover_bin($sformatf("U09 %0dx%0d B_MAX=%0d late cancel token caught at T=Hmax", n, n, b_max), longint'(n_late_t));
      tb_pkg::cover_bin($sformatf("U09 %0dx%0d B_MAX=%0d collision caught at T=TIGHT-1", n, n, b_max),
                        (last_fail_t == tight - 1 && last_fail_t > 0) ? 1 : 0);
      // Every longer subslot up to SAFE is collision-free as well
      stop_first = 1'b0;
      contig     = 1'b1;
      for (int t = tight + 1; t <= safe; t++) begin
        if (n <= 8 || all_cfg || t == safe) begin
          reset_counts();
          run_round(n, t, b_max);
          `TB_CHECK(!failed(),
            ("U09 %0dx%0d B_MAX=%0d T=%0d: %0d collisions, %0d late tokens; first: %s", n, n, b_max, t, n_coll, n_late,
             first_fail))
          if (failed()) contig = 1'b0;
        end
      end
      if (contig && tight > 0) begin
        if (n <= 8 || all_cfg)
          tb_pkg::info($sformatf("U09 %0dx%0d B_MAX=%0d: no collision for any T = %0d..%0d", n, n, b_max, tight, safe));
        else
          tb_pkg::info($sformatf("U09 %0dx%0d B_MAX=%0d: no collision at T = %0d and %0d", n, n, b_max, tight, safe));
      end
    endfunction

    // -------------------------------------------------------------------------
    // U09 step 2: frames, local timetable side rg on an n x n mesh
    // -------------------------------------------------------------------------
    function void u09_frames(int n, int rg, int b_max, int mode);
      int    tl, tg, nl, ng, n_pairs;
      string what;
      mesh       = n;
      tl         = t_sub(rg, b_max, mode);
      tg         = t_sub(n, b_max, mode);
      nl         = n_slots(rg);
      ng         = n_slots(n);
      stop_first = 1'b0;
      if (mode == TM_TIGHT) what = "TIGHT";
      else                  what = "SAFE";
      reset_counts();
      run_round(rg, tl, b_max);                // local round, every region at once
      run_round(n, tg, b_max);                 // global round
      `TB_CHECK(!failed(),
        ("U09 %0dx%0d frames, %0dx%0d regions, B_MAX=%0d, %s: the rounds have %0d collisions, %0d late tokens; first: %s",
         n, n, rg, rg, b_max, what, n_coll, n_late, first_fail))
      reset_counts();
      n_pairs = 0;
      for (int l = 0; l < nl; l++) begin
        for (int g = 0; g < ng; g++) begin
          run_pair(rg, l, tl, n, g, tg, b_max);  // local slot, then global slot
          run_pair(n, g, tg, rg, l, tl, b_max);  // global slot, then local slot
          n_pairs += 2;
        end
      end
      `TB_CHECK(!failed(),
        ("U09 %0dx%0d frames, %0dx%0d regions, B_MAX=%0d, %s: %0d collisions, %0d late tokens at slot boundaries; first: %s",
         n, n, rg, rg, b_max, what, n_coll, n_late, first_fail))
      tb_pkg::info($sformatf("U09 %0dx%0d frames, %0dx%0d regions, B_MAX=%0d, %s (local T=%0d, global T=%0d): rounds and all %0d local/global slot boundaries collision-free",
                             n, n, rg, rg, b_max, what, tl, tg, n_pairs));
      tb_pkg::cover_bin($sformatf("U09 %0dx%0d/%0d B_MAX=%0d %s slot boundaries checked", n, n, rg, b_max, what),
                        longint'(n_pairs));
    endfunction

  endclass

endpackage


module tb_timetable_check;
  import sched_pkg::*;
  import tt_check_pkg::*;

  `TB_DUMP(tb_timetable_check)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tt_check c;
    int      nlist [4];
    int      blist [3];
    int      rlist [5][2];
    int      nfr, nn, nb, b0;
    bit      quick;
    nlist = '{2, 4, 8, 16};
    blist = '{5, 10, 15};
    rlist = '{'{8, 4}, '{16, 4}, '{8, 2}, '{16, 2}, '{16, 8}};    // mesh, region side
    tb_pkg::start("U08-U09", "timetable: disjoint paths per subslot; no collision in any cycle; the TIGHT minimum");
    c         = new();
    c.all_cfg = $test$plusargs("U09_ALL");
    quick     = $test$plusargs("QUICK");
    nfr       = c.all_cfg ? 5 : (quick ? 1 : 2);
    nn        = quick ? 3 : 4;                 // n = 2, 4, 8 (and 16)
    b0        = quick ? 1 : 0;                 // quick frames: B_MAX = 10 only
    nb        = quick ? 2 : 3;

    if (tb_pkg::test_on("U08")) begin
      tb_pkg::phase("U08 spatial: every subslot of every timetable");
      for (int i = 0; i < nn; i++) c.u08_timetable(nlist[i], nlist[i]);
      for (int i = 0; i < 5; i++) c.u08_timetable(rlist[i][1], rlist[i][0]);
    end

    if (tb_pkg::test_on("U09")) begin
      tb_pkg::phase("U09 step 1: the TIGHT minimum of each timetable, full rounds at B_MAX");
      for (int i = 0; i < nn; i++)
        for (int j = 0; j < 3; j++)
          c.u09_tight(nlist[i], blist[j]);
      tb_pkg::phase("U09 step 2: frames, every local/global slot boundary");
      for (int i = 0; i < nfr; i++)
        for (int j = b0; j < nb; j++)
          for (int md = 0; md < 2; md++)
            c.u09_frames(rlist[i][0], rlist[i][1], blist[j], (md == 0) ? TM_TIGHT : TM_SAFE);
    end
    tb_pkg::finish();
  end

endmodule
