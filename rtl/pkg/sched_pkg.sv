// =============================================================================
// File      rtl/pkg/sched_pkg.sv                                   Tracker: F32
// Project   Demand-adaptive buffer reservation scheduler for router bypassing
//           in mesh NoCs (MVE433P, RV College of Engineering)
// Purpose   The BuffeRS timetable [1, Sec. III]: router groups (Eq. 1), the
//           router of a group in a given row (Eq. 2), slot numbers (Table II),
//           the transmit and receive partner in every subslot, path mode and
//           length, the slot timing (setup window, SAFE and TIGHT subslots),
//           and the cycle offsets of one bypass inside its subslot, which the
//           bypass datapath (B4) and the reservation controller (B5) follow;
//           the ejection FIFO depth of designs B-D; the frames of designs C
//           and D (B6).
// Decisions D08 groups, table, partner; D09 path direction; D10 subslot
//           length; D13 setup window; D37 partner rule; D38 bypass timing;
//           D39 TIGHT subslot length; D48 reservation space; D15, D16, D18,
//           D19, D50 frames
// Tests     U05-U09 (tb_slot_table, tb_slot_counter, tb_group_to_router,
//           tb_timetable_check); U11 (tb_phase_sequencer)
// -----------------------------------------------------------------------------
// A timetable has a side n: n = MESH_N for the global timetable, and
// n = REGION_N for the local one, which every region runs at the same time in
// region-relative coordinates (designs C and D, D15). Every function takes n.
// Coordinates follow noc_pkg: x the column (0 west), y the row (0 north).
//   Group   g(x, y) = (x - y) mod n: n wrapped diagonals, each with one router
//           in every row and every column (Eq. 1; as printed, Eq. 1 puts the
//           main diagonal in group n, so we use the mod form Fig. 2 draws).
//   Row     the router of group w in row k is ((w + k) mod n, k) (Eq. 2 with
//           the sign of its wrap term corrected: rid = w + (n+1)k - n[k+w>=n]).
//   Slot    one per unordered group pair {a, b}, a <= b, numbered as in
//           Table II: slot = b(b+1)/2 + a. A round has n(n+1)/2 slots.
//   Subslot slot {a, b} has n subslots s = 0..n-1 (D37, Fig. 3, Fig. 2(j)):
//             a < b: row j of group a and row (j - s) mod n of group b swap a
//                    burst each way: a -> b by XY, b -> a by YX (D09), over
//                    the same links in opposite directions;
//             a = b: row j sends to row (j + s) mod n by XY and receives from
//                    row (j - s) mod n; nobody sends in subslot 0.
//           Every ordered pair of routers meets exactly once per round.
//   Timing  a slot is S = Hmax + 1 setup cycles, then n subslots of T cycles,
//           with Hmax = 2(n - 1) the longest path (D13, D10).
// =============================================================================
`timescale 1ns/1ps

// A package of constants is a library: most blocks use only some of them.
/* verilator lint_off UNUSEDPARAM */
package sched_pkg;
  import noc_pkg::*;

  // ---------------------------------------------------------------------------
  // Widths (fixed maxima for a 16 x 16 timetable and 15-flit bursts)
  // ---------------------------------------------------------------------------
  localparam int TT_GRP_W = 4;   // group, 0..15
  localparam int SLOT_W   = 8;   // slot, 0..135 (16 x 17 / 2 slots)
  localparam int SUB_W    = 4;   // subslot, 0..15
  localparam int TCYC_W   = 7;   // cycle inside a setup window or a subslot, 0..127
  localparam int HOP_W    = 5;   // hops on a path, 0..30

  typedef logic [TT_GRP_W-1:0] tt_grp_t;
  typedef logic [SLOT_W-1:0]   slot_t;
  typedef logic [SUB_W-1:0]    sub_t;
  typedef logic [HOP_W-1:0]    hop_t;

  // Subslot rule (parameter T_MODE, D10)
  localparam int TM_SAFE  = 0;
  localparam int TM_TIGHT = 1;

  // ---------------------------------------------------------------------------
  // Slot timing (D10, D13, D39)
  // ---------------------------------------------------------------------------
  // Longest path, in hops
  function automatic int hmax(int n);
    return 2 * (n - 1);
  endfunction

  // Setup window: a cancel token crosses the longest path, one hop per cycle,
  // before the first subslot starts (D13)
  function automatic int s_setup(int n);
    return hmax(n) + 1;
  endfunction

  // SAFE: a subslot drains its longest burst before the next one starts (D10)
  function automatic int t_safe(int n, int bmax);
    return 2 * hmax(n) + bmax + 2;
  endfunction

  // TIGHT: the shortest subslot U09 proves collision-free, for every n and
  // bmax it checks; the limit is a destination's NI write port (D39)
  function automatic int t_tight(int n, int bmax);
    return 2 * hmax(n) + bmax - 4;
  endfunction

  function automatic int t_sub(int n, int bmax, int mode);
    return (mode == TM_SAFE) ? t_safe(n, bmax) : t_tight(n, bmax);
  endfunction

  function automatic int n_slots(int n);
    return n * (n + 1) / 2;
  endfunction

  function automatic int slot_len(int n, int t);
    return s_setup(n) + n * t;
  endfunction

  function automatic int round_len(int n, int t);
    return n_slots(n) * slot_len(n, t);
  endfunction

  // ---------------------------------------------------------------------------
  // Groups, rows and slots (Eq. 1, Eq. 2, Table II)
  // ---------------------------------------------------------------------------
  // v mod n, in 0..n-1 for either sign of v. Every timetable side is a power
  // of two (2, 4, 8 or 16), so this is the low bits of v in two's complement,
  // and synthesis builds no divider.
  function automatic int wrap(int v, int n);
    return v & (n - 1);
  endfunction

  // 1 when n is a power of two (a timetable side must be)
  function automatic bit pow2(int n);
    return (n > 0) && ((n & (n - 1)) == 0);
  endfunction

  // n(n+1)/2 for n = 0..15: the first slot of group n's row of Table II.
  // A table, so the slot table's reset logic stays small.
  function automatic slot_t tri_num(tt_grp_t n);
    slot_t r;
    case (n)
      4'd0:    r = 8'd0;
      4'd1:    r = 8'd1;
      4'd2:    r = 8'd3;
      4'd3:    r = 8'd6;
      4'd4:    r = 8'd10;
      4'd5:    r = 8'd15;
      4'd6:    r = 8'd21;
      4'd7:    r = 8'd28;
      4'd8:    r = 8'd36;
      4'd9:    r = 8'd45;
      4'd10:   r = 8'd55;
      4'd11:   r = 8'd66;
      4'd12:   r = 8'd78;
      4'd13:   r = 8'd91;
      4'd14:   r = 8'd105;
      default: r = 8'd120;
    endcase
    return r;
  endfunction

  // Group of the router at (x, y) (Eq. 1)
  function automatic int grp_of(int x, int y, int n);
    return wrap(x - y, n);
  endfunction

  // Column of the router of group w in row k (Eq. 2); that router is (row_x, k)
  function automatic int row_x(int w, int k, int n);
    return wrap(w + k, n);
  endfunction

  // Slot of the group pair {a, b}, in either order (Table II)
  function automatic int slot_of(int a, int b);
    int lo, hi;
    lo = (a < b) ? a : b;
    hi = (a < b) ? b : a;
    return hi * (hi + 1) / 2 + lo;
  endfunction

  // ---------------------------------------------------------------------------
  // Partners (D37). The router of group g in row j, in subslot s of the slot
  // whose other group is o (o = g in a same-group slot). Coordinates are the
  // timetable's own: region-relative for a local timetable.
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic   vld;    // a burst goes this way in this subslot
    coord_t x;      // the partner
    coord_t y;
    logic   mode;   // MODE_XY or MODE_YX: the dimension order of the burst's path
  } partner_t;

  // Where this router sends
  function automatic partner_t tx_partner(int g, int j, int o, int s, int n);
    partner_t p;
    int       r;
    p = '0;
    r = 0;
    if (o == g) begin                 // same group: to row j + s, by XY; idle when s = 0
      if (s != 0) begin
        r      = wrap(j + s, n);
        p.vld  = 1'b1;
        p.mode = MODE_XY;
      end
    end else if (g < o) begin         // lower group: to row j - s of the higher one, by XY
      r      = wrap(j - s, n);
      p.vld  = 1'b1;
      p.mode = MODE_XY;
    end else begin                    // higher group: to row j + s of the lower one, by YX
      r      = wrap(j + s, n);
      p.vld  = 1'b1;
      p.mode = MODE_YX;
    end
    if (p.vld) begin
      p.x = coord_t'(row_x(o, r, n));
      p.y = coord_t'(r);
    end
    return p;
  endfunction

  // Who sends to this router; mode is the mode of that incoming path, so a
  // cancel token goes back to the sender in the other mode
  function automatic partner_t rx_partner(int g, int j, int o, int s, int n);
    partner_t p;
    int       r;
    p = '0;
    r = 0;
    if (o == g) begin                 // same group: from row j - s, by XY
      if (s != 0) begin
        r      = wrap(j - s, n);
        p.vld  = 1'b1;
        p.mode = MODE_XY;
      end
    end else if (g < o) begin         // lower group: row j - s of the higher one sends by YX
      r      = wrap(j - s, n);
      p.vld  = 1'b1;
      p.mode = MODE_YX;
    end else begin                    // higher group: row j + s of the lower one sends by XY
      r      = wrap(j + s, n);
      p.vld  = 1'b1;
      p.mode = MODE_XY;
    end
    if (p.vld) begin
      p.x = coord_t'(row_x(o, r, n));
      p.y = coord_t'(r);
    end
    return p;
  endfunction

  // ---------------------------------------------------------------------------
  // Timing of one bypass inside its subslot (D38). t is the subslot's first
  // cycle, L the hops of the path, B the flits of the burst, j = 0..B-1 the
  // flit, i = 1..L the link from the source (link i enters the i-th router
  // after the source). Two cycles per hop (ST, then LT), the lookahead one
  // cycle ahead of the head flit on its own wires (D07).
  //   t                 the source checks for a cancel token and picks packets
  //   t + 1 + j         flit j crosses the source's crossbar
  //   t + 1 + 2i + j    flit j is on link i
  //   t + 2i            the lookahead is on lookahead link i
  //   t + 2L + 3 + j    flit j is written into the destination's NI
  // The last flit of the longest burst reaches the NI at t + T_SAFE, the first
  // cycle of the next subslot, where nothing of that subslot uses a link yet.
  // A cancel token for subslot k is decided in the first cycle of subslot k-1
  // (of the setup window for k = 0), is on reverse link h (h = 1..L, from the
  // destination) h cycles later and is seen by the source one cycle after the
  // last hop. So T >= Hmax + 1, and S = Hmax + 1 covers subslot 0.
  // ---------------------------------------------------------------------------
  localparam int TC_DECIDE = 0;

  function automatic int tc_xbar(int j);
    return 1 + j;
  endfunction

  function automatic int tc_link(int i, int j);
    return 1 + 2 * i + j;
  endfunction

  function automatic int tc_la(int i);
    return 2 * i;
  endfunction

  function automatic int tc_eject(int l, int j);
    return 2 * l + 3 + j;
  endfunction

  // Cycle (after the decision) at which a cancel token is on reverse link h
  function automatic int tc_cancel(int h);
    return h;
  endfunction

  // ---------------------------------------------------------------------------
  // Ejection FIFO of designs B-D (D48). Regular flits never hold more than
  // NUM_VC * VC_DEPTH slots (credits); the rest, the bypass space, is
  // reserved one burst at a time by resv_ctrl. A decision for subslot k+1
  // (cycle t_k) counts the bypass flits in the FIFO, the flits still to come
  // from bursts that started before it, B_MAX for the burst of subslot k
  // (its length is not known yet) and B_MAX for the new one. With every sink
  // taking a flit each cycle the FIFO holds at most one flit, and the bursts
  // that started before t_k land at most ej_tail flits at or after t_k: the
  // NI writes one flit per cycle and the last one of them lands by
  // t_k - T + 2 Hmax + 2 + B_MAX. So byp_space = 2 B_MAX + ej_tail + 1 never
  // cancels a turn while every sink keeps up (TIGHT: tail 7, SAFE: 1), and
  // a stalled sink makes the destination cancel instead of overflowing.
  // ---------------------------------------------------------------------------
  function automatic int ej_tail(int n, int bmax, int mode);
    int e;
    e = 2 * hmax(n) + 3 + bmax - t_sub(n, bmax, mode);
    return (e > 0) ? e : 0;
  endfunction

  function automatic int byp_space(int n, int bmax, int mode);
    return 2 * bmax + ej_tail(n, bmax, mode) + 1;
  endfunction

  // Ejection FIFO depth of designs B-D: 48 flits for 4 VCs of 5, B_MAX 10,
  // TIGHT (design A keeps 25, D31). The same for designs C and D: a local
  // subslot leaves the same tail (2 Hmax + 3 + B_MAX - T is 7 for TIGHT and
  // 1 for SAFE whatever the side), and a decision at the first cycle of a
  // slot that follows a slot of the other timetable sees the same bound.
  function automatic int ej_depth_b(int nvc, int vcd, int n, int bmax, int mode);
    return nvc * vcd + byp_space(n, bmax, mode);
  endfunction

  // ---------------------------------------------------------------------------
  // Frames of designs C and D (D15, D16, D18, D19, D50). Every region of
  // REGION_N x REGION_N routers runs the local timetable (side rg) at the
  // same time; a frame is k_L local slots, then one slot of the global
  // timetable (side n); each timetable resumes where it stopped. The plan
  // k_L is sampled A_DELAY - 1 cycles after each epoch tick and holds from
  // the first frame that starts after that cycle (phase_sequencer, F50).
  // ---------------------------------------------------------------------------
  localparam int KL_W    = 6;       // k_L, 0..63 (K_MAX is 47 on 16x16)
  localparam int A_DELAY = 256;     // epoch tick -> plan sampled, cycles (D19)

  typedef logic [KL_W-1:0] kl_t;

  // Length of a frame of kl local slots
  function automatic int frame_len(int n, int rg, int bmax, int mode, int kl);
    return kl * slot_len(rg, t_sub(rg, bmax, mode)) + slot_len(n, t_sub(n, bmax, mode));
  endfunction

  // k_L of design C: as many local slots as take one global slot's time,
  // rounded (a 50/50 split): 4 on 8x8, 14 on 16x16 (TIGHT, B_MAX 10; 3 and
  // 11 with SAFE); 1 when the region is the mesh
  function automatic int k_fixed(int n, int rg, int bmax, int mode);
    int g, l;
    g = slot_len(n, t_sub(n, bmax, mode));
    l = slot_len(rg, t_sub(rg, bmax, mode));
    return (2 * g + l) / (2 * l);
  endfunction

  // Largest k_L (D18): 15 up to 8x8, 47 on 16x16
  function automatic int k_max(int n);
    return (n >= 16) ? 47 : 15;
  endfunction

  // Epoch (D19): the smallest power of two that holds one global round at
  // k_L = 4: 2^15 cycles on 8x8, 2^18 on 16x16
  // Plain assignments and a fixed number of steps: run time (U11) is not
  // elaboration, and xsim 2024.2 got the old form wrong there (a cast of a
  // call inside an expression, README rule 9), so its while loop never ended
  function automatic int epoch_len(int n, int rg, int bmax, int mode);
    longint r, e;
    int     ns, fl;
    ns = n_slots(n);
    fl = frame_len(n, rg, bmax, mode, 4);
    r  = longint'(ns);
    r  = r * fl;
    e  = 1;
    for (int i = 0; i < 40; i++) begin
      if (e < r) e = e * 2;
    end
    return int'(e);
  endfunction

  // ---------------------------------------------------------------------------
  // The controller of design D (B7; D17, D18, D54-D56). Every tile counts its
  // occupied input VCs every DM_SAMPLE cycles into two saturating DM_W-bit
  // sums, local (the packet's destination is in the tile's region) and
  // cross-region, latched at each epoch tick (demand_counter, F54). The
  // tiles add the latched sums bit-serially along a tree to tile (0, 0)
  // (ctrl_node, F55), where sched_ctrl (F56) turns them into the next k_L,
  // which the tiles forward back to each other before the plan sample.
  // ---------------------------------------------------------------------------
  localparam int DM_W      = 14;   // demand sums per tile, bits (D17)
  localparam int DM_SAMPLE = 16;   // sampling period, cycles (D17)
  localparam int K_MIN     = 1;    // smallest k_L the controller sets (D18)
  localparam int POL_PROP  = 0;    // policy: k_L = round(alpha C_L / C_G) (D18)
  localparam int POL_SQRT  = 1;    // policy: k_L = round(alpha sqrt(C_L / C_G)) (D18)
  localparam int CT_W      = 8;    // controller cycle count after a tick, saturating at 255

  // Width of the mesh-wide sums: n^2 tiles of DM_W bits
  function automatic int dm_sum_w(int n);
    return DM_W + $clog2(n * n);
  endfunction

  // Cycle after a tick in which tile (x, y) adds bit 0 of its sums (D55):
  // each row adds toward column 0, column 0 toward tile (0, 0), one hop per
  // cycle, so the leaf of row y starts one cycle after the leaf of row y + 1
  function automatic int ct_start(int x, int y, int n);
    return 2 * n - 1 - x - y;
  endfunction

  // Cycle after a tick in which sched_ctrl holds both whole sums (bit k of
  // tile (0, 0)'s output arrives in cycle 2n + k)
  function automatic int ct_sums(int n);
    return 2 * n + dm_sum_w(n);
  endfunction

  // Ordered router pairs that one slot of an n x n timetable serves:
  // n^2 (n^2 - 1) pairs in n (n + 1) / 2 slots, so 2n (n - 1): 4, 24, 112
  // and 480 for n = 2, 4, 8, 16
  function automatic int pair_turns(int n);
    return 2 * n * (n - 1);
  endfunction

  // The policy table (D18, D56): k_L for share bin q = 0..15, q =
  // floor(16 C_L / (C_L + C_G)). The bin's middle, C_L / C_G = u / v with
  // u = 2q + 1 and v = 31 - 2q, stands for the bin. alpha = pair_turns(n) /
  // ((n / rg)^2 pair_turns(rg)) weighs a global slot's pair turns against
  // a local slot's in every region (7/6 on 8x8 with 4x4 regions). Both
  // policies round half up; the result is clamped to [kmin, kmax]. Integer
  // arithmetic and a loop of fixed bound, so every tool builds the same
  // table at elaboration (README rules 8 and 9).
  function automatic int kl_table(int pol, int n, int rg, int kmin, int kmax, int q);
    longint a, b, u, v, k, lhs, rhs, nr, lo, hi, t;
    int     pg, pl, ri, qi;
    pg  = pair_turns(n);
    pl  = pair_turns(rg);
    ri  = n / rg;
    qi  = 2 * q + 1;
    a   = longint'(pg);            // alpha = a / b
    b   = longint'(pl);
    nr  = longint'(ri);
    b   = b * nr * nr;
    u   = longint'(qi);
    v   = 64'sd32 - u;
    lo  = longint'(kmin);
    hi  = longint'(kmax);
    k   = 64'sd0;
    if (pol == POL_SQRT) begin
      // the largest k with (k - 1/2)^2 <= alpha^2 u / v
      rhs = 64'sd4 * a * a * u;
      for (int i = 1; i < 64; i++) begin
        t   = longint'(i);
        lhs = 64'sd2 * t - 64'sd1;
        lhs = lhs * lhs * b * b * v;
        if (lhs <= rhs) k = t;
      end
    end else begin
      k = (64'sd2 * a * u + b * v) / (64'sd2 * b * v);
    end
    if (k < lo) k = lo;
    if (k > hi) k = hi;
    return int'(k);
  endfunction

  // The sequencer's whole state, for the testbench's lockstep check
  typedef struct packed {
    logic              loc;      // the local timetable runs this cycle
    kl_t               kl;       // k_L of this frame
    kl_t               lc;       // local slots finished in this frame (0 in its global slot)
    logic              l_setup;  // local counter: setup window, subslot, cycle, slot
    sub_t              l_sub;
    logic [TCYC_W-1:0] l_cyc;
    slot_t             l_slot;
    logic              g_setup;  // global counter
    sub_t              g_sub;
    logic [TCYC_W-1:0] g_cyc;
    slot_t             g_slot;
  } seq_st_t;

endpackage
/* verilator lint_on UNUSEDPARAM */
