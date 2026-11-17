// =============================================================================
// File      tb/checkers/router_checker.sv                          Tracker: F43
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Checks every bypass in a design-B or design-C mesh against the
//           timetable and the timing contract, cycle by cycle, from the links
//           alone:
//           which bursts may exist (sched_ref, written independently of the
//           RTL schedule), which of them their destination cancels (the
//           ni_checker model, F47), where each flit, lookahead and cancel
//           token must be in every cycle (sched_pkg tc_*, D38), and that
//           nothing else carries a bypass flit, a lookahead or a token.
// Decisions D38 timing contract, D40 verification split, D41 checker class,
//           D45 B4 verification, D12 and D48 reservation, D16 frames
// Tests     BP01-BP10 (tb_mesh_b); TL01-TL06 (tb_mesh_c, tb_mesh_cb); kills
//           B3 mutant M21
// -----------------------------------------------------------------------------
// Decisions (D12, D38): in the first cycle d of every subslot k (and the
// first cycle of the setup window, for subslot 0) every destination of the
// next subslot decides. The model (nchk.decide) reserves or cancels; for a
// cancel, the token {source, the other mode} must be on reverse link h at
// d + h (ref_path from the destination back to the source, in the other
// mode), and the source must send nothing in that subslot.
// Every subslot (start t, from the cycles since reset, sched_ref's ref_pos)
// the bursts allowed are sched_ref's ref_xfers, over ref_path's links.
// For each one not cancelled, at t + 2 the lookahead on the source's first
// link says whether a burst left and its length B (none: nothing may
// follow, and the reservation is released at t + 2L). Then:
//   lookahead   on lookahead link i at t + tc_la(i), i = 1..L: same
//               destination, mode and length, vc 0
//   flit j      on link i at t + tc_link(i, j), bypass bit set, the same
//               flit on every link (recorded on link 1)
//   NI          flit j on the destination's Local output (the NI write) at
//               t + tc_eject(L, j)
//   burst       whole packets back to back (head ... tail by each head's
//               length), all addressed to the partner, 1 <= B <= B_MAX
// Any bypass flit, lookahead or cancel token on a link or Local output at a
// cycle where none is expected is an error, and so is an expected one that
// is missing (link exclusivity: a regular flit there would have collided).
// The model follows every accepted burst: its lookahead's length at t + 2L
// (nchk.due), each flit written into the NI (nchk.end_cycle). Regular
// flits are checked by the sinks and the scoreboard; credits and VC states
// by the RTL assertions (output_unit, input_unit, bypass_relay) and the
// idle check after every test.
// Sampling: link registers change at the rising edge; mesh_bench copies every
// link into the checker at the falling edge, in cycle cyc (as sink_node), and
// calls step(). Position 0 of every slot counter is the cycle in which reset
// rises (the bench records it at the next rising edge).
// Design C (set_frames): sched_ref's frame_ref gives, every cycle, which
// timetable runs and both positions; a local subslot allows the local
// timetable's bursts in every region at once (ref_mesh_xfers), with the
// local setup window and subslot length for its decisions. Everything else
// is the same check.
// The checker is a class in a package (D41), so the file holds no module: an
// EDA Playground bundle compiles it for every test without a second top. Its
// arrays are sized for a 16x16 mesh.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package rchk_pkg;
  import noc_pkg::*;
  import sched_pkg::*;
  import sched_ref_pkg::*;

  // The checker keeps whole flits, heads and lookaheads and reads only the
  // fields it checks
  /* verilator lint_off UNUSEDSIGNAL */

  localparam int W    = 256;          // cycles of expected events kept (ring)
  localparam int NB   = 4096;         // bursts kept (ring)
  localparam int MAXN = 256;          // routers
  localparam int MAXL = 32;           // hops on a path
  localparam int NRES = 13;           // per router: flit links N E S W, la links N E S W, NI, cancel links N E S W

  // An expected event: a flit (kind 0), a lookahead (kind 1), an NI write
  // (kind 2) of burst b, or a cancel token (kind 3) for source idx in mode
  // ext, at resource res in cycle cyc; idx = flit j or link i otherwise
  typedef struct {
    int cyc;
    int kind;
    int res;
    int b;
    int idx;
    int lnk;
    int ext;
  } ev_t;

  // A lookahead due at an NI: node, cycle, burst length (0: no-show)
  typedef struct {
    int cyc;
    int node;
    int len;
  } due_t;

  class rchk;
    int  n, nn, tsub, sset, bmax;
    int  c0;
    bit  started;
    // design C: the frames (null: one global timetable, design B), the
    // region side and the local subslot length and setup window
    sched_ref_pkg::frame_ref fr;
    int  rg, tl, sl;
    // observations of this cycle, per resource
    flit_t obs_fl [];
    la_t   obs_la [];
    cx_t   obs_cx [];
    bit    obs_v  [];
    // the NI model (null: every turn is accepted)
    nchk_pkg::nchk nck;
    // per source: the next subslot it was decided for, and the decision
    int    dec_t   [MAXN];
    bit    dec_acc [MAXN];
    due_t  dues [$];
    // bursts (ring of NB)
    int    b_src  [NB];
    int    b_dst  [NB];
    int    b_mode [NB];
    int    b_len  [NB];               // hops L
    int    b_t    [NB];
    int    b_n    [NB];               // flits B (-1 until the lookahead is seen)
    int    b_slot [NB];
    int    b_sub  [NB];
    bit    b_loc  [NB];               // a burst of the local timetable
    int    b_pr   [NB][MAXL];         // link i leaves router b_pr[i-1] through port b_pp[i-1]
    int    b_pp   [NB][MAXL];
    ftype_t           b_ft  [NB][16];
    logic [127:0]     b_dat [NB][16];
    bit    b_acc  [NB];               // its destination reserved space
    bit    b_cx   [NB];               // its destination cancelled it
    int    nxt;                       // next burst id
    int    pend [$];                  // bursts whose lookahead is due at t + 2
    ev_t   bucket [W][$];             // expected events by cycle
    // statistics
    longint unsigned n_sub, n_allowed, n_bursts, n_flits, n_pkts, n_multi, n_full, n_ev, n_long;
    longint unsigned n_dec, n_cxd, n_tok, n_cxok;
    longint unsigned n_lsub, n_lallowed, n_lbursts, n_xtail;    // design C: local subslots, ...
    int    lb_reg [MAXN];             // design C: local bursts per region
    int    max_b;

    function new(int mesh_n, int t_sub_c, int s_setup_c, int b_max);
      n     = mesh_n;
      nn    = mesh_n * mesh_n;
      tsub  = t_sub_c;
      sset  = s_setup_c;
      bmax  = b_max;
      obs_fl = new[MAXN * NRES];
      obs_la = new[MAXN * NRES];
      obs_cx = new[MAXN * NRES];
      obs_v  = new[MAXN * NRES];
      nck    = null;
      for (int r = 0; r < MAXN; r++) begin
        dec_t[r]   = -1;
        dec_acc[r] = 1'b0;
      end
      started = 1'b0;
      c0      = 0;
      nxt     = 0;
      n_sub = 0; n_allowed = 0; n_bursts = 0; n_flits = 0; n_pkts = 0; n_multi = 0; n_full = 0;
      n_ev = 0; n_long = 0; max_b = 0;
      n_dec = 0; n_cxd = 0; n_tok = 0; n_cxok = 0;
      n_lsub = 0; n_lallowed = 0; n_lbursts = 0; n_xtail = 0;
      for (int r = 0; r < MAXN; r++) lb_reg[r] = 0;
      fr = null;
      rg = mesh_n;
      tl = t_sub_c;
      sl = s_setup_c;
    endfunction

    // Design C: the frame model, the region side, the local subslot length
    // and setup window
    function void set_frames(sched_ref_pkg::frame_ref f, int region_n, int t_loc, int s_loc);
      fr = f;
      rg = region_n;
      tl = t_loc;
      sl = s_loc;
    endfunction

    function string res_name(int res);
      int    r, k;
      string s;
      r = res / NRES;
      k = res % NRES;
      if (k < 4)       s = $sformatf("link %s of R%0d", tb_pkg::port_name(3'(k)), r);
      else if (k < 8)  s = $sformatf("lookahead link %s of R%0d", tb_pkg::port_name(3'(k - 4)), r);
      else if (k == 8) s = $sformatf("Local output (NI write) of R%0d", r);
      else             s = $sformatf("cancel link %s of R%0d", tb_pkg::port_name(3'(k - 9)), r);
      return s;
    endfunction

    function string bname(int b);
      string w;
      if (b_loc[b]) w = "local slot";
      else          w = "slot";
      return $sformatf("burst R%0d->R%0d (%s %0d subslot %0d, t=%0d, %0d hops)",
                       b_src[b], b_dst[b], w, b_slot[b], b_sub[b], b_t[b], b_len[b]);
    endfunction

    function void expect_ev(int cyc, int kind, int res, int b, int idx, int lnk, int ext);
      ev_t e;
      e.cyc  = cyc;
      e.kind = kind;
      e.res  = res;
      e.b    = b;
      e.idx  = idx;
      e.lnk  = lnk;
      e.ext  = ext;
      bucket[cyc % W].push_back(e);           // cyc >= 0: an unsigned index (D30)
    endfunction

    // The NI model learns at cycle cyc whether node's reserved burst came
    function void add_due(int cyc, int node, int len);
      due_t du;
      du.cyc  = cyc;
      du.node = node;
      du.len  = len;
      dues.push_back(du);
    endfunction

    // Cycle d decides subslot k of slot sn (of the local timetable when
    // loc), which starts at t: every destination of that subslot reserves
    // or cancels (model); a cancel must put its token on each reverse link
    // h at d + h
    function void decide_all(int d, int t, bit loc, int sn, int k);
      xfer_q_t q;
      link_q_t p, rp;
      int      src, dst, L, bmode;
      bit      acc;
      q = ref_mesh_xfers(n, rg, loc, sn, k);
      foreach (q[m]) begin
        p   = ref_path(q[m].sx, q[m].sy, q[m].dx, q[m].dy, q[m].mode);
        src = q[m].sy * n + q[m].sx;
        dst = q[m].dy * n + q[m].dx;
        L   = p.size();
        acc = 1'b1;
        if (nck != null) acc = nck.decide(dst, t + tc_la(L));
        n_dec++;
        dec_t[src]   = t;
        dec_acc[src] = acc;
        if (!acc) begin
          n_cxd++;
          rp = ref_path(q[m].dx, q[m].dy, q[m].sx, q[m].sy, !q[m].mode);
          `TB_CHECK(rp.size() == L, ("checker: the way back from R%0d to R%0d has %0d hops, the burst %0d", dst, src, rp.size(), L))
          // the token's mode through a plain variable: xsim 2024.2 gave 0
          // for int'(!q[m].mode) passed as an argument (R-1110-02)
          bmode = q[m].mode ? 0 : 1;
          foreach (rp[h])
            expect_ev(d + tc_cancel(h + 1), 3, (rp[h].y * n + rp[h].x) * NRES + 9 + rp[h].p, -1, src, h + 1, bmode);
        end
      end
    endfunction

    // Subslot k of slot sn (local timetable when loc) starts at t: the
    // bursts the timetable allows
    function void subslot_start(int t, bit loc, int sn, int k);
      xfer_q_t q;
      link_q_t p;
      int      b;
      q = ref_mesh_xfers(n, rg, loc, sn, k);
      n_sub++;
      if (loc) n_lsub++;
      foreach (q[m]) begin
        p = ref_path(q[m].sx, q[m].sy, q[m].dx, q[m].dy, q[m].mode);
        b = nxt % NB;
        nxt++;
        b_src[b]  = q[m].sy * n + q[m].sx;
        b_dst[b]  = q[m].dy * n + q[m].dx;
        b_mode[b] = int'(q[m].mode);
        b_len[b]  = p.size();
        b_t[b]    = t;
        b_n[b]    = -1;
        b_slot[b] = sn;
        b_sub[b]  = k;
        b_loc[b]  = loc;
        `TB_CHECK(dec_t[b_src[b]] == t, ("checker: %s was never decided (reserve or cancel) one subslot ahead", bname(b)))
        b_acc[b]  = (dec_t[b_src[b]] == t) && dec_acc[b_src[b]];
        b_cx[b]   = (dec_t[b_src[b]] == t) && !dec_acc[b_src[b]];
        foreach (p[h]) begin
          b_pr[b][h] = p[h].y * n + p[h].x;
          b_pp[b][h] = p[h].p;
        end
        pend.push_back(b);
        n_allowed++;
        if (loc) n_lallowed++;
      end
    endfunction

    // t + 2: the lookahead on the first link decides whether a burst left
    function void resolve(int b, int c);
      int res, bl, L, te;
      la_t a;
      L   = b_len[b];
      res = b_pr[b][0] * NRES + 4 + b_pp[b][0];
      tb_pkg::n_checks++;
      if (b_cx[b]) begin                       // cancelled: the source must send nothing
        b_n[b] = 0;
        if (obs_v[res]) begin
          tb_pkg::fail($sformatf("%s: its destination cancelled it, but the source sent a burst of %0d flits",
                                 bname(b), obs_la[res].len));
          obs_v[res] = 1'b0;
        end else begin
          n_cxok++;
        end
        return;
      end
      if (!obs_v[res]) begin
        b_n[b] = 0;
        if (b_acc[b]) add_due(b_t[b] + tc_la(L), b_dst[b], 0);   // no-show: released at t + 2L
        return;
      end
      a         = obs_la[res];
      obs_v[res] = 1'b0;                       // consumed
      bl        = int'(a.len);
      `TB_CHECK(int'(a.dst) == b_dst[b] && int'(a.mode) == b_mode[b] && a.vc == '0 && bl >= 1 && bl <= bmax,
                ("%s: lookahead on its first link says R%0d, mode %0d, vc %0d, %0d flits (B_MAX %0d)",
                 bname(b), a.dst, a.mode, a.vc, bl, bmax))
      if (bl < 1 || bl > bmax) bl = (bl < 1) ? 1 : bmax;
      b_n[b] = bl;
      n_bursts++;
      n_flits += longint'(bl);
      if (bl == bmax) n_full++;
      if (bl > max_b) max_b = bl;
      te = b_loc[b] ? tl : tsub;
      if (2 * L + 3 + bl - 1 >= te) n_long++;     // last flit lands in a later subslot (TIGHT)
      if (b_loc[b]) begin
        n_lbursts++;
        lb_reg[((b_src[b] / n) / rg) * (n / rg) + (b_src[b] % n) / rg]++;
      end
      if (b_acc[b]) add_due(b_t[b] + tc_la(L), b_dst[b], bl);
      for (int i = 2; i <= L; i++)
        expect_ev(b_t[b] + tc_la(i), 1, b_pr[b][i - 1] * NRES + 4 + b_pp[b][i - 1], b, i, i, 0);
      for (int i = 1; i <= L; i++)
        for (int j = 0; j < bl; j++)
          expect_ev(b_t[b] + tc_link(i, j), 0, b_pr[b][i - 1] * NRES + b_pp[b][i - 1], b, j, i, 0);
      for (int j = 0; j < bl; j++)
        expect_ev(b_t[b] + tc_eject(L, j), 2, b_dst[b] * NRES + 8, b, j, L + 1, 0);
      if (c != b_t[b] + tc_la(1))
        tb_pkg::fail($sformatf("checker: %s resolved at cycle %0d", bname(b), c));
    endfunction

    // The packets of a burst, from its flits on link 1
    function void burst_integrity(int b);
      int    j, k, len, np;
      head_t h;
      j  = 0;
      np = 0;
      while (j < b_n[b]) begin
        h   = head_t'(b_dat[b][j]);
        len = int'(h.len);
        `TB_CHECK(is_head(b_ft[b][j]) && (len == 1 || len == MAX_PKT_FLITS) && int'(h.dst) == b_dst[b] && j + len <= b_n[b],
                  ("%s: flit %0d should start a whole packet for R%0d: type %b, length %0d, destination %0d, burst %0d flits",
                   bname(b), j, b_dst[b], b_ft[b][j], len, h.dst, b_n[b]))
        if (!(is_head(b_ft[b][j]) && (len == 1 || len == MAX_PKT_FLITS) && j + len <= b_n[b])) return;
        for (k = 1; k < len; k++)
          `TB_CHECK(!is_head(b_ft[b][j + k]) && is_tail(b_ft[b][j + k]) == (k == len - 1),
                    ("%s: flit %0d (packet flit %0d of %0d) has type %b", bname(b), j + k, k, len, b_ft[b][j + k]))
        `TB_CHECK(len == 1 ? b_ft[b][j] == FT_SINGLE : b_ft[b][j] == FT_HEAD,
                  ("%s: packet at flit %0d of length %0d starts with type %b", bname(b), j, len, b_ft[b][j]))
        j  = j + len;
        np++;
      end
      n_pkts += longint'(np);
      if (np > 1) n_multi++;
    endfunction

    // One cycle: obs_* hold this cycle's bypass flits, lookaheads and cancel
    // tokens; the NI model's RTL samples are in nck
    function void step(int c);
      pos_t  pz;
      sched_ref_pkg::fpos_t fp;
      bit    loc;
      int    e, b, nleft, r, otgt, omode, tn, tt, ts;
      ev_t   ev;
      string what;
      due_t  du;
      if (!started) return;
      // 0. the NI model at the start of the cycle, and this cycle's NI writes
      if (nck != null) begin
        nck.compare(c);
        for (int i = 0; i < nn; i++) nck.arr_now[i] = obs_v[i * NRES + 8];
      end
      e  = c - c0;
      if (fr == null) begin
        pz  = ref_pos(n, tsub, sset, longint'(e));
        loc = 1'b0;
      end else begin
        fp  = fr.at(longint'(e));
        loc = fp.loc;
        if (loc) pz = fp.lp;
        else     pz = fp.gp;
      end
      if (loc) begin
        tn = rg;
        tt = tl;
        ts = sl;
      end else begin
        tn = n;
        tt = tsub;
        ts = sset;
      end
      // 1. a subslot starts: the bursts it allows (decided one subslot ago)
      if (!pz.setup && pz.cyc == 0) subslot_start(c, loc, pz.slot, pz.sub);
      // 2. decisions for the next subslot (setup window: subslot 0), from
      //    the NI model's state at the start of the cycle
      if (pz.cyc == 0) begin
        if (pz.setup)              decide_all(c, c + ts, loc, pz.slot, 0);
        else if (pz.sub < tn - 1)  decide_all(c, c + tt, loc, pz.slot, pz.sub + 1);
      end
      // 3. lookaheads due on first links
      nleft = pend.size();
      for (int k = 0; k < nleft; k++) begin
        b = pend.pop_front();
        if (b_t[b] + tc_la(1) == c) resolve(b, c);
        else                        pend.push_back(b);
      end
      // 4. events due now
      nleft = bucket[c % W].size();
      for (int k = 0; k < nleft; k++) begin
        ev = bucket[c % W].pop_front();
        if (ev.cyc != c) begin
          bucket[c % W].push_back(ev);     // a later lap of the ring
          continue;
        end
        b = ev.b;
        tb_pkg::n_checks++;
        if (ev.kind == 3) begin                // a cancel token
          if (!obs_v[ev.res]) begin
            tb_pkg::fail($sformatf("cancel token for R%0d expected on %s at cycle %0d (reverse link %0d), not there",
                                   ev.idx, res_name(ev.res), c, ev.lnk));
            continue;
          end
          obs_v[ev.res] = 1'b0;
          n_tok++;
          otgt  = int'(obs_cx[ev.res].tgt);
          omode = int'(obs_cx[ev.res].mode);
          `TB_CHECK(otgt == ev.idx && omode == ev.ext,
                    ("cancel token on %s at cycle %0d is for R%0d mode %0d, expected R%0d mode %0d", res_name(ev.res), c,
                     otgt, omode, ev.idx, ev.ext))
          continue;
        end
        n_ev++;
        if (!obs_v[ev.res]) begin
          if (ev.kind == 1) what = $sformatf("lookahead (link %0d)", ev.idx);
          else              what = $sformatf("flit %0d (link %0d)", ev.idx, ev.lnk);
          tb_pkg::fail($sformatf("%s: %s expected on %s at cycle %0d (t+%0d), not there",
                                 bname(b), what, res_name(ev.res), c, c - b_t[b]));
          continue;
        end
        obs_v[ev.res] = 1'b0;
        if (ev.kind == 1) begin
          `TB_CHECK(int'(obs_la[ev.res].dst) == b_dst[b] && int'(obs_la[ev.res].mode) == b_mode[b] &&
                    int'(obs_la[ev.res].len) == b_n[b] && obs_la[ev.res].vc == '0,
                    ("%s: lookahead on %s says R%0d mode %0d %0d flits", bname(b), res_name(ev.res),
                     obs_la[ev.res].dst, obs_la[ev.res].mode, obs_la[ev.res].len))
        end else begin
          // design C: the burst's last flit lands while the other timetable runs (coverage)
          if (ev.kind == 2 && ev.idx == b_n[b] - 1 && fr != null) begin
            fp = fr.at(longint'(e));
            if (fp.loc != b_loc[b]) n_xtail++;
          end
          `TB_CHECK(obs_fl[ev.res].vc == '0, ("%s: flit %0d on %s has vc %0d", bname(b), ev.idx, res_name(ev.res), obs_fl[ev.res].vc))
          if (ev.kind == 0 && ev.lnk == 1) begin
            b_ft[b][ev.idx]  = obs_fl[ev.res].ftype;
            b_dat[b][ev.idx] = obs_fl[ev.res].data;
            if (ev.idx == b_n[b] - 1) burst_integrity(b);
          end else begin
            `TB_CHECK(obs_fl[ev.res].ftype == b_ft[b][ev.idx] && obs_fl[ev.res].data == b_dat[b][ev.idx],
                      ("%s: flit %0d changed on %s", bname(b), ev.idx, res_name(ev.res)))
          end
        end
      end
      // 5. anything left over was not expected
      for (int i = 0; i < nn * NRES; i++) begin
        if (obs_v[i]) begin
          r = i % NRES;
          tb_pkg::n_checks++;
          if (r >= 4 && r < 8)
            tb_pkg::fail($sformatf("unexpected lookahead on %s at cycle %0d: R%0d mode %0d %0d flits",
                                   res_name(i), c, obs_la[i].dst, obs_la[i].mode, obs_la[i].len));
          else if (r >= 9)
            tb_pkg::fail($sformatf("unexpected cancel token on %s at cycle %0d: for R%0d mode %0d",
                                   res_name(i), c, obs_cx[i].tgt, obs_cx[i].mode));
          else
            tb_pkg::fail($sformatf("unexpected bypass flit on %s at cycle %0d (type %b)", res_name(i), c, obs_fl[i].ftype));
          obs_v[i] = 1'b0;
        end
      end
      // 6. the NI model at the end of the cycle: lookaheads due, flits written
      if (nck != null) begin
        nleft = dues.size();
        for (int k = 0; k < nleft; k++) begin
          du = dues.pop_front();
          if (du.cyc == c)     nck.due(du.node, c, du.len);
          else if (du.cyc > c) dues.push_back(du);
          else                 tb_pkg::fail($sformatf("checker: lookahead of R%0d due at cycle %0d missed", du.node, du.cyc));
        end
        nck.end_cycle(c);
      end
    endfunction

    // Flits of the burst from src that started at t (-1: no such burst yet)
    function int burst_len(int src, int t);
      int r;
      r = -1;
      for (int k = 0; k < NB; k++) begin
        if (b_src[k] == src && b_t[k] == t && k < nxt) r = b_n[k];
      end
      return r;
    endfunction

    // The packets of the burst from src that started at t: how many, and
    // each one's source, sequence number and length, in burst order
    function int burst_pkts(int src, int t, output int psrc [16], output int pseq [16], output int plen [16]);
      int    j, np, len;
      head_t h;
      np = 0;
      for (int i = 0; i < 16; i++) begin
        psrc[i] = -1;
        pseq[i] = -1;
        plen[i] = 0;
      end
      for (int k = 0; k < NB; k++) begin
        if (k < nxt && b_src[k] == src && b_t[k] == t && b_n[k] > 0) begin
          j = 0;
          while (j < b_n[k] && np < 16) begin
            h   = head_t'(b_dat[k][j]);
            len = int'(h.len);
            if (len < 1) break;               // burst_integrity has reported it
            psrc[np] = int'(h.src);
            pseq[np] = int'(h.seq);
            plen[np] = len;
            np++;
            j = j + len;
          end
        end
      end
      return np;
    endfunction

    // No burst still on its way
    function bit idle();
      int s;
      s = 0;
      for (int k = 0; k < W; k++) s += bucket[k].size();
      return (s == 0);
    endfunction
  endclass

  // The bench's checker (null when no design-B or design-C mesh is simulated)
  rchk the_chk;
  /* verilator lint_on UNUSEDSIGNAL */

endpackage

