// =============================================================================
// File      tb/common/sched_monitor.sv                             Tracker: F53
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Checks the schedule the tiles compute, every cycle, against the
//           reference (sched_ref's frame_ref and ref_mesh_xfers), with or
//           without traffic: who may send to whom in every subslot, and who
//           decides for whom one subslot ahead; and keeps per ordered pair
//           of routers the turns of each local and global round and the
//           intervals between turns.
// Decisions D15 regions, D16 frames, D37 partner rule, D40 verification
//           split, D41 checker class
// Tests     TL01-TL06 (tb_mesh_c, tb_mesh_cb): every design-C run
// -----------------------------------------------------------------------------
// Inputs, per tile and cycle (mesh_bench copies them at the falling edge):
// the tile's tx_go, tx_dst, tx_mode (the bypass turn it gives its router)
// and dec, dec_s0, dec_loc, dec_src, dec_mode, dec_hops (the decision it
// gives its NI). The reference position of the cycle says which timetable
// runs, its slot, subslot and cycle. Then:
//   subslot start   every tile with a transfer in ref_mesh_xfers of that
//                   subslot has tx_go with that partner and mode; no other
//                   tile has tx_go (one check per tile)
//   other cycles    no tile has tx_go (one check)
//   decision cycle  (first cycle of a setup window, or of a subslot but the
//                   last) every destination of the next subslot has dec
//                   with its source, the mode, the hops, s0 and the phase;
//                   no other tile has dec (one check per tile); other
//                   cycles: no dec anywhere (one check)
//   local turn      the partner is in the source's region (TL01)
// Rounds: at the start of each round of the local timetable (slot 0's
// setup window, after the first), every ordered pair inside a region has had
// exactly one local turn in the round just ended and no other pair any (one
// check); likewise every ordered pair of the mesh exactly one global turn
// per global round (TL02).
// Intervals: per ordered pair the cycles between consecutive turns (local
// or global), the largest and smallest since clear_gaps() (TL06 reads them).
// A class in a package (D41); arrays are sized for 16x16.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package smon_pkg;
  import noc_pkg::*;
  import sched_ref_pkg::*;

  /* verilator lint_off UNUSEDSIGNAL */

  localparam int MAXN = 256;            // routers

  class smon;
    int  n, nn, rg, c0;
    bit  started;
    frame_ref fr;
    // inputs of this cycle
    bit  tx_go   [MAXN];
    int  tx_dst  [MAXN];
    bit  tx_mode [MAXN];
    bit  dec     [MAXN];
    bit  dec_s0  [MAXN];
    bit  dec_loc [MAXN];
    int  dec_src [MAXN];
    bit  dec_mode[MAXN];
    int  dec_hops[MAXN];
    // per ordered pair src * nn + dst
    int  last_t  [];
    int  max_gap [];
    int  min_gap [];
    int  cnt_l   [];
    int  cnt_g   [];
    bit  same_r  [];                    // both routers in one region
    // statistics
    longint unsigned n_lturns, n_gturns, n_lrounds, n_grounds, n_lsub, n_gsub, n_ldec, n_gdec;
    int  turns_reg [MAXN];              // local turns per region
    bit  seen_l, seen_g;                // a round has started (the next round start ends it)

    function new(int mesh_n, int region_n, frame_ref f);
      fpos_t fp0;
      n   = mesh_n;
      nn  = mesh_n * mesh_n;
      rg  = region_n;
      fr  = f;
      c0  = 0;
      started = 1'b0;
      last_t  = new[nn * nn];
      max_gap = new[nn * nn];
      min_gap = new[nn * nn];
      cnt_l   = new[nn * nn];
      cnt_g   = new[nn * nn];
      same_r  = new[nn * nn];
      for (int s = 0; s < nn; s++) begin
        for (int d = 0; d < nn; d++) begin
          last_t[s * nn + d] = -1;
          cnt_l[s * nn + d]  = 0;
          cnt_g[s * nn + d]  = 0;
          same_r[s * nn + d] = (reg_of(s) == reg_of(d)) && (s != d);
        end
      end
      clear_gaps();
      for (int r = 0; r < MAXN; r++) begin
        turns_reg[r] = 0;
        tx_go[r]     = 1'b0;
        tx_dst[r]    = 0;
        tx_mode[r]   = 1'b0;
        dec[r]       = 1'b0;
        dec_s0[r]    = 1'b0;
        dec_loc[r]   = 1'b0;
        dec_src[r]   = 0;
        dec_mode[r]  = 1'b0;
        dec_hops[r]  = 0;
      end
      n_lturns = 0; n_gturns = 0; n_lrounds = 0; n_grounds = 0;
      n_lsub = 0; n_gsub = 0; n_ldec = 0; n_gdec = 0;
      // the timetable that runs from reset starts its first round in cycle 0,
      // before the first cycle the bench samples (cycle 1)
      fp0    = f.at(0);
      seen_l = fp0.loc;
      seen_g = !fp0.loc;
    endfunction

    // Region of router r (row-major order of the regions)
    function int reg_of(int r);
      return ((r / n) / rg) * (n / rg) + (r % n) / rg;
    endfunction

    // Hops of the path from a to b (Manhattan distance; both orders)
    function int hops(int a, int b);
      int dx, dy;
      dx = (a % n) - (b % n);
      dy = (a / n) - (b / n);
      if (dx < 0) dx = -dx;
      if (dy < 0) dy = -dy;
      return dx + dy;
    endfunction

    // Start the interval statistics afresh (the next turn of each pair
    // opens its first interval)
    function void clear_gaps();
      for (int i = 0; i < nn * nn; i++) begin
        max_gap[i] = -1;
        min_gap[i] = -1;
        last_t[i]  = -1;
      end
    endfunction

    // A turn of src to dst at cycle c (local timetable when loc)
    function void turn(int src, int dst, bit loc, int c);
      int k, g, rs;
      k  = src * nn + dst;
      rs = reg_of(src);
      if (last_t[k] >= 0) begin
        g = c - last_t[k];
        if (max_gap[k] < 0 || g > max_gap[k]) max_gap[k] = g;
        if (min_gap[k] < 0 || g < min_gap[k]) min_gap[k] = g;
      end
      last_t[k] = c;
      if (loc) begin
        cnt_l[k]++;
        n_lturns++;
        turns_reg[rs]++;
        `TB_CHECK(rs == reg_of(dst),
                  ("local turn at cycle %0d: R%0d (region %0d) sends to R%0d in region %0d", c, src, rs, dst, reg_of(dst)))
      end else begin
        cnt_g[k]++;
        n_gturns++;
      end
    endfunction

    // End of a local (loc) or global round: every pair's count, then clear
    function void round_end(bit loc, int c);
      int  bad, fs, fd, fcnt, want, k;
      bad  = 0;
      fs   = -1;
      fd   = -1;
      fcnt = 0;
      for (int s = 0; s < nn; s++) begin
        for (int d = 0; d < nn; d++) begin
          if (s == d) continue;
          k = s * nn + d;
          if (loc) want = same_r[k] ? 1 : 0;
          else     want = 1;
          if ((loc ? cnt_l[k] : cnt_g[k]) != want) begin
            if (bad == 0) begin
              fs   = s;
              fd   = d;
              fcnt = loc ? cnt_l[k] : cnt_g[k];
            end
            bad++;
          end
          if (loc) cnt_l[k] = 0;
          else     cnt_g[k] = 0;
        end
      end
      if (loc) begin
        `TB_CHECK(bad == 0, ("local round ending at cycle %0d: %0d ordered pairs without exactly one local turn per pair inside a region (none outside); first R%0d -> R%0d had %0d",
                             c, bad, fs, fd, fcnt))
        n_lrounds++;
      end else begin
        `TB_CHECK(bad == 0, ("global round ending at cycle %0d: %0d ordered pairs without exactly one global turn; first R%0d -> R%0d had %0d",
                             c, bad, fs, fd, fcnt))
        n_grounds++;
      end
    endfunction

    // One cycle
    function void step(int c);
      fpos_t   fp;
      pos_t    pz;
      bit      loc;
      int      e, tn, sub2, src, dst, nbad;
      xfer_q_t q;
      bit      want  [MAXN];
      int      w_dst [MAXN];
      bit      w_md  [MAXN];
      string   ph;
      if (!started) return;
      e   = c - c0;
      fp  = fr.at(longint'(e));
      loc = fp.loc;
      if (loc) begin
        pz = fp.lp;
        tn = rg;
        ph = "local";
      end else begin
        pz = fp.gp;
        tn = n;
        ph = "global";
      end
      // rounds: slot 0's setup window starts a round of the running timetable
      if (pz.setup && pz.cyc == 0 && pz.slot == 0) begin
        if (loc) begin
          if (seen_l) round_end(1'b1, c);
          seen_l = 1'b1;
        end else begin
          if (seen_g) round_end(1'b0, c);
          seen_g = 1'b1;
        end
      end
      // turns
      if (!pz.setup && pz.cyc == 0) begin
        q = ref_mesh_xfers(n, rg, loc, pz.slot, pz.sub);
        for (int r = 0; r < nn; r++) want[r] = 1'b0;
        foreach (q[m]) begin
          src        = q[m].sy * n + q[m].sx;
          want[src]  = 1'b1;
          w_dst[src] = q[m].dy * n + q[m].dx;
          w_md[src]  = q[m].mode;
        end
        if (loc) n_lsub++;
        else     n_gsub++;
        for (int r = 0; r < nn; r++) begin
          if (want[r]) begin
            `TB_CHECK(tx_go[r] && tx_dst[r] == w_dst[r] && tx_mode[r] == w_md[r],
                      ("cycle %0d (%s slot %0d subslot %0d): R%0d should send to R%0d by %s; tile gives turn %0d to R%0d mode %0d",
                       c, ph, pz.slot, pz.sub, r, w_dst[r], w_md[r] ? "YX" : "XY",
                       tx_go[r], tx_dst[r], tx_mode[r]))
            if (tx_go[r]) turn(r, tx_dst[r], loc, c);
          end else begin
            `TB_CHECK(!tx_go[r], ("cycle %0d (%s slot %0d subslot %0d): R%0d has no partner, the tile gives it a turn to R%0d",
                                  c, ph, pz.slot, pz.sub, r, tx_dst[r]))
          end
        end
      end else begin
        nbad = 0;
        src  = -1;
        for (int r = 0; r < nn; r++) begin
          if (tx_go[r]) begin
            if (nbad == 0) src = r;
            nbad++;
          end
        end
        `TB_CHECK(nbad == 0, ("cycle %0d is not the start of a subslot, but %0d tiles give a turn (first R%0d)", c, nbad, src))
      end
      // decisions
      if (pz.cyc == 0 && (pz.setup || pz.sub < tn - 1)) begin
        sub2 = pz.setup ? 0 : pz.sub + 1;
        q    = ref_mesh_xfers(n, rg, loc, pz.slot, sub2);
        for (int r = 0; r < nn; r++) want[r] = 1'b0;
        foreach (q[m]) begin
          dst        = q[m].dy * n + q[m].dx;
          want[dst]  = 1'b1;
          w_dst[dst] = q[m].sy * n + q[m].sx;     // here: the source
          w_md[dst]  = q[m].mode;
        end
        if (loc) n_ldec++;
        else     n_gdec++;
        for (int r = 0; r < nn; r++) begin
          if (want[r]) begin
            `TB_CHECK(dec[r] && dec_src[r] == w_dst[r] && dec_mode[r] == w_md[r] && dec_hops[r] == hops(r, w_dst[r]) &&
                      dec_s0[r] == (sub2 == 0) && dec_loc[r] == loc,
                      ("cycle %0d (%s slot %0d, deciding subslot %0d): R%0d should decide for R%0d, mode %0d, %0d hops; tile says dec %0d from R%0d mode %0d %0d hops s0 %0d local %0d",
                       c, ph, pz.slot, sub2, r, w_dst[r], w_md[r], hops(r, w_dst[r]),
                       dec[r], dec_src[r], dec_mode[r], dec_hops[r], dec_s0[r], dec_loc[r]))
          end else begin
            `TB_CHECK(!dec[r], ("cycle %0d (%s slot %0d, deciding subslot %0d): R%0d has no sender, the tile decides for R%0d",
                                c, ph, pz.slot, sub2, r, dec_src[r]))
          end
        end
      end else begin
        nbad = 0;
        src  = -1;
        for (int r = 0; r < nn; r++) begin
          if (dec[r]) begin
            if (nbad == 0) src = r;
            nbad++;
          end
        end
        `TB_CHECK(nbad == 0, ("cycle %0d is no decision cycle, but %0d tiles decide (first R%0d)", c, nbad, src))
      end
    endfunction

    // Largest and smallest interval over the pairs inside a region (same
    // = 1) or across regions (same = 0); -1 when no interval was seen
    function void gaps(bit same, output int gmax, output int gmin, output int npairs);
      gmax   = -1;
      gmin   = -1;
      npairs = 0;
      for (int s = 0; s < nn; s++) begin
        for (int d = 0; d < nn; d++) begin
          if (s == d || same_r[s * nn + d] != same || max_gap[s * nn + d] < 0) continue;
          npairs++;
          if (max_gap[s * nn + d] > gmax) gmax = max_gap[s * nn + d];
          if (gmin < 0 || min_gap[s * nn + d] < gmin) gmin = min_gap[s * nn + d];
        end
      end
    endfunction
  endclass

  // The bench's monitor (null when no design-C mesh is simulated)
  smon the_smon;
  /* verilator lint_on UNUSEDSIGNAL */

endpackage
