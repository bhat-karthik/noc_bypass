// =============================================================================
// File      tb/checkers/mesh_checker.sv                            Tracker: F52
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Mesh-wide checks of the two-level timetable, cycle by cycle:
//           every tile's sequencer in lockstep and equal to the frame
//           reference; the frame and boundary rules seen from the RTL state
//           alone; region containment of the local phase on the links; a
//           progress watchdog; and an independent collision monitor that
//           knows nothing of the schedule, only the lookaheads on the links.
// Decisions D15 regions, D16 frames, D19 plan switch, D40 verification
//           split, D41 checker class
// Tests     TL01, TL03, TL04 (tb_mesh_c) and every other design-C run
// -----------------------------------------------------------------------------
// Inputs, per tile and cycle (mesh_bench copies them at the falling edge):
// the sequencer's state (seq_st_t), and per output link whether it carries
// a bypass flit, a lookahead (and its length) or a cancel token; per tile
// the lookahead its router hands the NI (la_loc) and whether a bypass flit
// is written into the NI (ej_mon). Checks:
//   lockstep    every tile's state equals tile 0's (one check per cycle)
//   reference   tile 0's state equals frame_ref's position (one per cycle)
//   boundaries  from tile 0's state in this cycle and the last one (one per
//               cycle): the phase changes only after the last cycle of the
//               running timetable's slot, and the other timetable then
//               starts a setup window; the paused counter does not move;
//               k_L changes only when a frame starts (after a global slot);
//               a frame starts with no local slot done
//   frames      at every frame start (one check): the frame just ended
//               lasted k_L local slots plus one global slot, in cycles, the
//               global slot number advanced by one and the local one by k_L
//   containment in every cycle of the local phase outside the local setup
//               window, no bypass flit, lookahead or cancel token on a link
//               that crosses a region boundary (one per cycle). The global
//               slot's last flits land within the local setup window
//               (tail <= 6 cycles < 7), and the local timetable's paths
//               never leave their region.
//   watchdog    a frame starts at least every frame of k_L = 63 (fatal
//               otherwise: the timetable is stuck)
//   collisions  a lookahead of B flits on a link in cycle c announces that
//               link's flits for cycles c+1 .. c+B (D38); the NI's lookahead
//               announces its writes for c+3 .. c+2+B. Two announcements
//               that overlap on one link or one NI are a collision; a bypass
//               flit on a link or into an NI in a cycle nobody announced,
//               or none in an announced cycle, is an error (one check per
//               cycle for the links, one for the NIs).
// A class in a package (D41); arrays are sized for 16x16.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package mchk_pkg;
  import noc_pkg::*;
  import sched_pkg::*;
  import sched_ref_pkg::*;

  /* verilator lint_off UNUSEDSIGNAL */

  localparam int MAXN = 256;            // routers

  class mchk;
    int      n, nn, rg, tl, tg, c0;
    bit      started, two;
    frame_ref fr;
    longint  ll, lg;                    // slot lengths
    // inputs of this cycle
    seq_st_t st     [MAXN];
    bit      fl_v   [MAXN * 4];         // bypass flit on output link r * 4 + p
    bit      la_v   [MAXN * 4];         // lookahead on output link
    int      la_len [MAXN * 4];
    bit      cx_v   [MAXN * 4];         // cancel token on output link
    bit      ej_v   [MAXN];             // bypass flit written into the NI
    bit      lal_v  [MAXN];             // lookahead for the NI
    int      lal_len[MAXN];
    // state
    seq_st_t prev;
    bit      have_prev;
    // announced cycles, one bit per cycle from this one (bit 0) on, shifted
    // once per cycle: a link's flits, an NI's writes. An NI's next burst may
    // be announced while its last one is still being written (its lookahead
    // comes 3 cycles ahead), so a single window per NI is not enough
    longint unsigned lk_m [MAXN * 4];
    longint unsigned ni_m [MAXN];
    int      lk_beg [MAXN * 4];         // the latest announcement, for messages
    int      lk_end [MAXN * 4];
    int      ni_beg [MAXN];
    int      ni_end [MAXN];
    bit      xreg  [MAXN * 4];         // the link crosses a region boundary
    int      f_start, f_k, f_lslot, f_gslot, f_lends;   // the current frame
    // statistics
    longint unsigned n_frames, n_k_changes, n_lg, n_gl, n_xflits, n_ann, n_nian, n_contain;

    function new(int mesh_n, int region_n, int t_loc, int t_glb, frame_ref f);
      int x, y, s_l, s_g;
      n   = mesh_n;
      nn  = mesh_n * mesh_n;
      rg  = region_n;
      tl  = t_loc;
      tg  = t_glb;
      fr  = f;
      two = (region_n < mesh_n);
      s_l = 2 * (rg - 1) + 1;             // setup windows, Hmax + 1 (D13)
      s_g = 2 * (n - 1) + 1;
      ll  = longint'(s_l) + longint'(rg) * longint'(tl);
      lg  = longint'(s_g) + longint'(n) * longint'(tg);
      c0  = 0;
      started   = 1'b0;
      have_prev = 1'b0;
      for (int r = 0; r < nn; r++) begin
        x = r % n;
        y = r / n;
        xreg[r * 4 + int'(P_N)] = (y > 0) && (y % rg == 0);
        xreg[r * 4 + int'(P_E)] = (x < n - 1) && ((x + 1) % rg == 0);
        xreg[r * 4 + int'(P_S)] = (y < n - 1) && ((y + 1) % rg == 0);
        xreg[r * 4 + int'(P_W)] = (x > 0) && (x % rg == 0);
        for (int p = 0; p < 4; p++) begin
          lk_m  [r * 4 + p] = '0;
          lk_beg[r * 4 + p] = -1;
          lk_end[r * 4 + p] = -2;
        end
        ni_m  [r] = '0;
        ni_beg[r] = -1;
        ni_end[r] = -2;
        st[r]      = '0;
        ej_v[r]    = 1'b0;
        lal_v[r]   = 1'b0;
        lal_len[r] = 0;
        for (int p = 0; p < 4; p++) begin
          fl_v[r * 4 + p]   = 1'b0;
          la_v[r * 4 + p]   = 1'b0;
          la_len[r * 4 + p] = 0;
          cx_v[r * 4 + p]   = 1'b0;
        end
      end
      f_start = 0;
      f_k     = -1;
      f_lends = 0;
      n_frames = 0; n_k_changes = 0; n_lg = 0; n_gl = 0; n_xflits = 0; n_ann = 0; n_nian = 0; n_contain = 0;
    endfunction

    function string st_str(seq_st_t s);
      return $sformatf("phase %0d k_L %0d done %0d | local %0d/%0d/%0d slot %0d | global %0d/%0d/%0d slot %0d",
                       s.loc, s.kl, s.lc, s.l_setup, s.l_sub, s.l_cyc, s.l_slot, s.g_setup, s.g_sub, s.g_cyc, s.g_slot);
    endfunction

    // The state frame_ref predicts for cycle e, in the RTL's format
    function seq_st_t ref_st(longint e);
      fpos_t   p;
      seq_st_t s;
      p = fr.at(e);
      s = '0;
      s.loc = p.loc;
      s.kl  = kl_t'(p.k);
      s.lc  = kl_t'(p.lc);
      if (two) begin
        s.l_setup = p.lp.setup;
        s.l_sub   = sub_t'(p.lp.sub);
        s.l_cyc   = TCYC_W'(p.lp.cyc);
        s.l_slot  = slot_t'(p.lp.slot);
      end
      s.g_setup = p.gp.setup;
      s.g_sub   = sub_t'(p.gp.sub);
      s.g_cyc   = TCYC_W'(p.gp.cyc);
      s.g_slot  = slot_t'(p.gp.slot);
      return s;
    endfunction

    // The last cycle of a slot of the local (loc) or global counter in state s
    function bit slot_last(seq_st_t s, bit loc);
      if (loc) return !s.l_setup && int'(s.l_sub) == rg - 1 && int'(s.l_cyc) == tl - 1;
      return !s.g_setup && int'(s.g_sub) == n - 1 && int'(s.g_cyc) == tg - 1;
    endfunction

    function void step(int c);
      seq_st_t s0, x;
      int      nbad, fb, pb, rb, e, fl_bad, ni_bad, ann_bad, k, flc;
      bit      ok, fstart, in_win;
      longint unsigned nm;
      string   why, fs;
      if (!started) return;
      e  = c - c0;
      s0 = st[0];
      // lockstep
      nbad = 0;
      fb   = -1;
      for (int r = 1; r < nn; r++) begin
        if (st[r] != s0) begin
          if (nbad == 0) fb = r;
          nbad++;
        end
      end
      fs = "-";
      if (fb >= 0) fs = st_str(st[fb]);
      `TB_CHECK(nbad == 0, ("cycle %0d: %0d tiles out of step with R0; R%0d: %s; R0: %s", c, nbad, fb, fs, st_str(s0)))
      // reference
      x = ref_st(longint'(e));
      `TB_CHECK(s0 == x, ("cycle %0d: the sequencers are at %s; the frame reference says %s", c, st_str(s0), st_str(x)))
      // boundaries, from the RTL state alone
      fstart = (e == 0);
      if (have_prev) begin
        ok  = 1'b1;
        why = "";
        if (s0.loc != prev.loc) begin
          if (!slot_last(prev, prev.loc)) begin
            ok  = 1'b0;
            why = "the phase changed inside a slot";
          end else if (s0.loc ? !(s0.l_setup && s0.l_cyc == '0) : !(s0.g_setup && s0.g_cyc == '0)) begin
            ok  = 1'b0;
            why = "the other timetable did not resume at a setup window";
          end
          if (prev.loc) n_lg++;
          else          n_gl++;
        end
        if (two && prev.loc && s0.loc && (s0.g_setup != prev.g_setup || s0.g_sub != prev.g_sub ||
                                          s0.g_cyc != prev.g_cyc || s0.g_slot != prev.g_slot)) begin
          ok  = 1'b0;
          why = "the global counter moved in the local phase";
        end
        if (two && !prev.loc && !s0.loc && (s0.l_setup != prev.l_setup || s0.l_sub != prev.l_sub ||
                                            s0.l_cyc != prev.l_cyc || s0.l_slot != prev.l_slot)) begin
          ok  = 1'b0;
          why = "the local counter moved in the global phase";
        end
        fstart = !prev.loc && slot_last(prev, 1'b0);       // a global slot just ended
        if (s0.kl != prev.kl) begin
          n_k_changes++;
          if (!fstart) begin
            ok  = 1'b0;
            why = "k_L changed outside a frame start";
          end
        end
        if (two && fstart && (s0.lc != '0 || s0.loc != (s0.kl != '0))) begin
          ok  = 1'b0;
          why = "a frame started with local slots done, or in the wrong phase";
        end
        `TB_CHECK(ok, ("cycle %0d: %s; before: %s; now: %s", c, why, st_str(prev), st_str(s0)))
        if (prev.loc && slot_last(prev, 1'b1)) f_lends++;
      end
      // frames
      flc = e - f_start;                  // cycles since the frame started
      if (fstart) begin
        if (f_k >= 0) begin
          k = f_k;
          `TB_CHECK(longint'(flc) == longint'(k) * ll + lg && f_lends == (two ? k : 0) &&
                    int'(s0.g_slot) == (f_gslot + 1) % n_slots(n) &&
                    (!two || int'(s0.l_slot) == (f_lslot + k) % n_slots(rg)),
                    ("cycle %0d: the frame from cycle %0d lasted %0d cycles with %0d local slots (k_L %0d: %0d cycles); global slot %0d -> %0d, local slot %0d -> %0d",
                     c, f_start + c0, flc, f_lends, k, longint'(k) * ll + lg, f_gslot, s0.g_slot, f_lslot, s0.l_slot))
          n_frames++;
        end
        f_start = e;
        f_k     = int'(s0.kl);
        f_lslot = int'(s0.l_slot);
        f_gslot = int'(s0.g_slot);
        f_lends = 0;
      end
      flc = e - f_start;
      if (longint'(flc) > 63 * ll + lg)
        $fatal(1, "mesh_checker: no frame has started since cycle %0d (now %0d): the timetable is stuck", f_start + c0, c);
      // containment
      if (two && s0.loc && !s0.l_setup) begin
        nbad = 0;
        fb   = -1;
        for (int i = 0; i < nn * 4; i++) begin
          if (xreg[i] && (fl_v[i] || la_v[i] || cx_v[i])) begin
            if (nbad == 0) fb = i;
            nbad++;
          end
        end
        if (fb < 0) fb = 0;
        pb = fb % 4;
        rb = fb / 4;
        `TB_CHECK(nbad == 0, ("cycle %0d (local slot %0d): %0d region-crossing links carry bypass traffic; first: link %s of R%0d (flit %0d lookahead %0d token %0d)",
                              c, s0.l_slot, nbad, tb_pkg::port_name(3'(pb)), rb, fl_v[fb], la_v[fb], cx_v[fb]))
        n_contain++;
      end else if (two && !s0.loc) begin
        for (int i = 0; i < nn * 4; i++) if (xreg[i] && fl_v[i]) n_xflits++;
      end
      // collisions, from the lookaheads alone
      ann_bad = 0;
      for (int i = 0; i < nn * 4; i++) begin
        lk_m[i] = lk_m[i] >> 1;                       // bit 0: this cycle
        if (la_v[i]) begin
          nm = ((64'd1 << la_len[i]) - 64'd1) << 1;   // cycles c+1 .. c+B
          if ((lk_m[i] & nm) != '0) ann_bad++;        // overlaps a burst announced before
          lk_m[i]   = lk_m[i] | nm;
          lk_beg[i] = c + 1;
          lk_end[i] = c + la_len[i];
          n_ann++;
        end
      end
      for (int r = 0; r < nn; r++) begin
        ni_m[r] = ni_m[r] >> 1;
        if (lal_v[r]) begin
          nm = ((64'd1 << lal_len[r]) - 64'd1) << 3;  // cycles c+3 .. c+2+B
          if ((ni_m[r] & nm) != '0) ann_bad++;
          ni_m[r]   = ni_m[r] | nm;
          ni_beg[r] = c + 3;
          ni_end[r] = c + 2 + lal_len[r];
          n_nian++;
        end
      end
      `TB_CHECK(ann_bad == 0, ("cycle %0d: %0d lookaheads announce flits on a link or an NI that another burst holds then", c, ann_bad))
      fl_bad = 0;
      fb     = -1;
      for (int i = 0; i < nn * 4; i++) begin
        in_win = lk_m[i][0];
        if (fl_v[i] != in_win) begin
          if (fl_bad == 0) fb = i;
          fl_bad++;
        end
      end
      if (fb < 0) fb = 0;
      pb = fb % 4;
      rb = fb / 4;
      `TB_CHECK(fl_bad == 0, ("cycle %0d: %0d links where bypass flits and lookaheads disagree; first: link %s of R%0d, flit %0d, latest announcement cycles %0d..%0d",
                              c, fl_bad, tb_pkg::port_name(3'(pb)), rb, fl_v[fb], lk_beg[fb], lk_end[fb]))
      ni_bad = 0;
      fb     = -1;
      for (int r = 0; r < nn; r++) begin
        in_win = ni_m[r][0];
        if (ej_v[r] != in_win) begin
          if (ni_bad == 0) fb = r;
          ni_bad++;
        end
      end
      if (fb < 0) fb = 0;
      `TB_CHECK(ni_bad == 0, ("cycle %0d: %0d NIs where bypass writes and the NI lookaheads disagree; first R%0d, write %0d, latest announcement cycles %0d..%0d",
                              c, ni_bad, fb, ej_v[fb], ni_beg[fb], ni_end[fb]))
      prev      = s0;
      have_prev = 1'b1;
    endfunction
  endclass

  // The bench's checker (null when no design-C mesh is simulated)
  mchk the_mchk;
  /* verilator lint_on UNUSEDSIGNAL */

endpackage
