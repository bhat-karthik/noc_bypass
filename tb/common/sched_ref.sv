// =============================================================================
// File      tb/common/sched_ref.sv                                 Tracker: F36
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Reference model of the BuffeRS timetable for U05-U09, written
//           independently of rtl/pkg/sched_pkg.sv: groups and rows are found
//           by walking the wrapped diagonals, not with the mod formulas; slots
//           come from the list "for b, for a <= b" by search, not from
//           b(b+1)/2 + a; the bursts of a subslot are listed from the rows of
//           the two groups and every partner question is answered by looking
//           in that list; the slot counter's position is computed from the
//           number of enabled cycles, not counted.
// Used by   tb_slot_table (U05), tb_slot_counter (U06), tb_group_to_router
//           (U07), tb_timetable_check (U08, U09); router_checker (F43); B6:
//           tb_phase_sequencer (U11), mesh_checker (F52), sched_monitor (F53)
// -----------------------------------------------------------------------------
// The rules it encodes are the specification: Table II order [1], the
// partner rule D37 (Fig. 3, Fig. 2(j)), the path direction D09, and the slot
// structure D10/D13 (setup window, then n subslots), and the frames of
// designs C and D (D16, D19, D50: frame_ref, in closed form from the cycle
// number and the plan history). Coordinates are those of the timetable:
// region-relative for a local timetable; testers add the region's corner
// (ref_mesh_xfers does it for a whole mesh).
// =============================================================================
`timescale 1ns/1ps

package sched_ref_pkg;
  import noc_pkg::*;

  // One burst of a subslot: from (sx, sy) to (dx, dy), by XY or YX
  typedef struct {
    int sx;
    int sy;
    int dx;
    int dy;
    bit mode;
  } xfer_t;

  typedef xfer_t xfer_q_t[$];

  // One directed link of a path: it leaves router (x, y) through port p
  typedef struct {
    int x;
    int y;
    int p;
  } link_t;

  typedef link_t link_q_t[$];

  // Position of a slot counter
  typedef struct {
    bit setup;
    int sub;
    int cyc;
    int slot;
  } pos_t;

  // ---------------------------------------------------------------------------
  // Groups and rows, by walking the wrapped diagonals
  // ---------------------------------------------------------------------------
  // Group of (x, y): the column where its diagonal, followed up and to the
  // left with wrap-around, meets row 0
  function automatic int ref_group(int x, int y, int n);
    int cx;
    cx = x;
    for (int k = y; k > 0; k--) cx = (cx == 0) ? n - 1 : cx - 1;
    return cx;
  endfunction

  // Column of the router of group w in row k: from (w, 0) one step down and to
  // the right per row, with wrap-around
  function automatic int ref_row_x(int w, int k, int n);
    int cx;
    cx = w;
    for (int r = 0; r < k; r++) cx = (cx == n - 1) ? 0 : cx + 1;
    return cx;
  endfunction

  // ---------------------------------------------------------------------------
  // The slot list of Table II: for b = 0..n-1, for a = 0..b
  // ---------------------------------------------------------------------------
  function automatic int ref_n_slots(int n);
    int c;
    c = 0;
    for (int b = 0; b < n; b++)
      for (int a = 0; a <= b; a++)
        c++;
    return c;
  endfunction

  // Groups a <= b of slot `slot` (both -1 if there is no such slot)
  function automatic void ref_slot(int n, int slot, output int a, output int b);
    int c;
    c = 0;
    a = -1;
    b = -1;
    for (int bb = 0; bb < n; bb++) begin
      for (int aa = 0; aa <= bb; aa++) begin
        if (c == slot) begin
          a = aa;
          b = bb;
        end
        c++;
      end
    end
  endfunction

  // Slot in which groups g and t meet, found by searching the list
  // (The bound is computed before the loop: Verilator 5.020 hoists the
  // increments of a function called in a loop condition out of that function's
  // own loops, which then never end; rule D41.)
  function automatic int ref_table(int n, int g, int t);
    int a, b, found, ns;
    found = -1;
    ns    = ref_n_slots(n);
    for (int s = 0; s < ns; s++) begin
      ref_slot(n, s, a, b);
      if ((a == g && b == t) || (a == t && b == g)) found = s;
    end
    return found;
  endfunction

  // ---------------------------------------------------------------------------
  // The bursts of subslot s of a slot (D37, D09). same_yx routes the bursts
  // of a same-group slot by YX instead of XY (U08 checks that both work).
  // ---------------------------------------------------------------------------
  function automatic xfer_q_t ref_xfers(int n, int slot, int s, bit same_yx); /*verilator no_inline_task*/
    xfer_q_t q;
    xfer_t   f;
    int      a, b, k;
    q = {};
    ref_slot(n, slot, a, b);
    for (int j = 0; j < n; j++) begin
      if (a != b) begin
        // row j of the lower group and row j - s of the higher group swap bursts
        k      = (j + n - s) % n;
        f.sx   = ref_row_x(a, j, n);
        f.sy   = j;
        f.dx   = ref_row_x(b, k, n);
        f.dy   = k;
        f.mode = MODE_XY;                       // towards the higher group: X first
        q.push_back(f);
        f.sx   = ref_row_x(b, k, n);
        f.sy   = k;
        f.dx   = ref_row_x(a, j, n);
        f.dy   = j;
        f.mode = MODE_YX;                       // towards the lower group: Y first
        q.push_back(f);
      end else if (s != 0) begin
        // inside one group: row j sends to row j + s
        k      = (j + s) % n;
        f.sx   = ref_row_x(a, j, n);
        f.sy   = j;
        f.dx   = ref_row_x(a, k, n);
        f.dy   = k;
        f.mode = same_yx ? MODE_YX : MODE_XY;
        q.push_back(f);
      end
    end
    return q;
  endfunction

  // ---------------------------------------------------------------------------
  // Position of a slot counter after e enabled cycles since reset
  // ---------------------------------------------------------------------------
  function automatic pos_t ref_pos(int n, int t_sub, int s_setup, longint e);
    longint len, w;
    pos_t   p;
    len    = longint'(s_setup) + longint'(n) * longint'(t_sub);
    w      = e % len;
    p.slot = int'((e / len) % longint'(ref_n_slots(n)));
    if (w < longint'(s_setup)) begin
      p.setup = 1'b1;
      p.sub   = 0;
      p.cyc   = int'(w);
    end else begin
      p.setup = 1'b0;
      p.sub   = int'((w - longint'(s_setup)) / longint'(t_sub));
      p.cyc   = int'((w - longint'(s_setup)) % longint'(t_sub));
    end
    return p;
  endfunction

  // ---------------------------------------------------------------------------
  // The links of a dimension-order path, walked hop by hop
  // ---------------------------------------------------------------------------
  function automatic link_q_t ref_path(int sx, int sy, int dx, int dy, bit mode); /*verilator no_inline_task*/
    link_q_t q;
    link_t   l;
    int      x, y;
    q = {};
    x = sx;
    y = sy;
    if (mode == MODE_XY) begin
      while (x != dx) begin
        l.x = x; l.y = y; l.p = (dx > x) ? int'(P_E) : int'(P_W);
        q.push_back(l);
        x = (dx > x) ? x + 1 : x - 1;
      end
    end
    while (y != dy) begin
      l.x = x; l.y = y; l.p = (dy > y) ? int'(P_S) : int'(P_N);
      q.push_back(l);
      y = (dy > y) ? y + 1 : y - 1;
    end
    while (x != dx) begin                       // only for YX: X after Y
      l.x = x; l.y = y; l.p = (dx > x) ? int'(P_E) : int'(P_W);
      q.push_back(l);
      x = (dx > x) ? x + 1 : x - 1;
    end
    return q;
  endfunction

  // The router at the far end of link l
  function automatic void ref_link_end(link_t l, output int x, output int y);
    x = l.x;
    y = l.y;
    case (l.p)
      int'(P_N): y = l.y - 1;
      int'(P_E): x = l.x + 1;
      int'(P_S): y = l.y + 1;
      default:   x = l.x - 1;
    endcase
  endfunction

  // Port on which a link arrives at its far end
  function automatic int ref_opposite(int p);
    int r;
    case (p)
      int'(P_N): r = int'(P_S);
      int'(P_E): r = int'(P_W);
      int'(P_S): r = int'(P_N);
      default:   r = int'(P_E);
    endcase
    return r;
  endfunction

  // ---------------------------------------------------------------------------
  // The bursts of a subslot on an n x n mesh. Global (loc = 0): the n x n
  // timetable. Local (loc = 1): the rg x rg timetable in every region at
  // once, each burst moved by its region's corner (D15).
  // ---------------------------------------------------------------------------
  function automatic xfer_q_t ref_mesh_xfers(int n, int rg, bit loc, int slot, int s); /*verilator no_inline_task*/
    xfer_q_t q, r;
    xfer_t   f;
    if (!loc) return ref_xfers(n, slot, s, 1'b0);
    q = ref_xfers(rg, slot, s, 1'b0);
    r = {};
    for (int by = 0; by < n; by += rg) begin
      for (int bx = 0; bx < n; bx += rg) begin
        foreach (q[m]) begin
          f      = q[m];
          f.sx   = bx + q[m].sx;
          f.sy   = by + q[m].sy;
          f.dx   = bx + q[m].dx;
          f.dy   = by + q[m].dy;
          r.push_back(f);
        end
      end
    end
    return r;
  endfunction

  // ---------------------------------------------------------------------------
  // Frames of designs C and D (D16, D19, D50), in closed form. A frame is k
  // local slots, then one global slot; each timetable's position comes from
  // its own count of running cycles (ref_pos). The plan history is a list of
  // segments: from cycle sc[i] (a frame start, cycles since reset), frames
  // of sk[i] local slots, with sel[i] local and seg[i] global cycles run
  // before it. A plan sampled at the end of cycle s starts a segment at the
  // first frame start after s; a sample that has not started yet when the
  // next comes is replaced. The RTL counts cycles in two counters and a
  // state machine; this model divides.
  // ---------------------------------------------------------------------------
  typedef struct {
    bit   loc;        // the local timetable runs
    int   k;          // k_L of this frame (0 with one pointer)
    int   lc;         // local slots finished in this frame (0 in its global slot)
    pos_t lp;         // local counter
    pos_t gp;         // global counter
    bit   fstart;     // the first cycle of a frame
  } fpos_t;

  class frame_ref;
    int     n, rg, tl, tg, sl, sg;    // sides, subslot lengths, setup windows
    bit     two;                      // two pointers (rg < n)
    longint ll, lg;                   // slot lengths
    longint sc  [$];
    int     sk  [$];
    longint sel [$];
    longint seg [$];
    int     cur;                      // segment of the last at() (cycles only grow)

    function new(int n_c, int rg_c, int tl_c, int tg_c, int k0);
      n   = n_c;
      rg  = rg_c;
      tl  = tl_c;
      tg  = tg_c;
      sl  = 2 * (rg - 1) + 1;         // Hmax + 1 (D13), counted here, not from sched_pkg
      sg  = 2 * (n - 1) + 1;
      two = (rg < n);
      ll  = longint'(sl) + longint'(rg) * longint'(tl);
      lg  = longint'(sg) + longint'(n) * longint'(tg);
      sc.push_back(0);
      sk.push_back(two ? k0 : 0);
      sel.push_back(0);
      seg.push_back(0);
      cur = 0;
    endfunction

    function longint flen(int k);
      return longint'(k) * ll + lg;
    endfunction

    // The plan k was registered at the end of cycle s. Returns 0 (one
    // pointer: no plan), 1 (a new segment, from frame_c) or 2 (it replaced
    // a sample that had not taken effect)
    // (plain assignments, no cast inside an expression: README rule 9)
    function int sample(longint s, int k);
      int     j;
      longint i, f, fl, kj, cj, el, eg;
      if (!two) return 0;
      j  = sc.size() - 1;
      cj = sc[j];
      if (cj > s) begin               // the newest segment has not started: replace its plan
        sk[j] = k;
        return 2;
      end
      kj = longint'(sk[j]);
      fl = flen(sk[j]);
      i  = (s - cj) / fl + 1;         // frames of segment j that start at or before s
      f  = cj + i * fl;
      el = sel[j];
      eg = seg[j];
      sc.push_back(f);
      sk.push_back(k);
      sel.push_back(el + i * kj * ll);
      seg.push_back(eg + i * lg);
      return 1;
    endfunction

    // Start of the newest segment (the frame the latest plan starts at)
    function longint last_start();
      return sc[sc.size() - 1];
    endfunction

    // The first cycle at or after e that starts a frame
    function longint next_frame(longint e);
      fpos_t  p;
      longint c;
      c = e;
      p = at(c);
      while (!p.fstart) begin
        c++;
        p = at(c);
      end
      return c;
    endfunction

    // Position in cycle e (cycles since reset rose)
    function fpos_t at(longint e);
      fpos_t  p;
      longint w, fl, f, r, el, eg, kl, s0, se, sg0;
      int     k, ns;
      ns = sc.size();
      if (cur >= ns) cur = 0;
      s0 = sc[cur];
      if (s0 > e) cur = 0;
      for (int j = 0; j < ns; j++) begin   // forward to the segment that holds e (no call in a loop condition)
        if (cur + 1 >= ns) break;
        s0 = sc[cur + 1];
        if (s0 > e) break;
        cur++;
      end
      k        = sk[cur];
      kl       = longint'(k);
      s0       = sc[cur];
      se       = sel[cur];
      sg0      = seg[cur];
      w        = e - s0;
      p.k      = k;
      p.lc     = 0;
      p.fstart = 1'b0;
      if (!two) begin
        p.loc    = 1'b0;
        el       = 0;
        eg       = e;
        p.fstart = (e % lg == 0);
      end else if (k == 0) begin
        p.loc    = 1'b0;
        el       = se;
        eg       = sg0 + w;
        p.fstart = (w % lg == 0);
      end else begin
        fl       = flen(k);
        f        = w / fl;
        r        = w % fl;
        p.fstart = (r == 0);
        if (r < kl * ll) begin
          p.loc = 1'b1;
          p.lc  = int'(r / ll);
          el    = se + f * kl * ll + r;
          eg    = sg0 + f * lg;
        end else begin
          p.loc = 1'b0;                 // the global slot (lc reads 0 there, as the RTL's)
          el    = se + (f + 1) * kl * ll;
          eg    = sg0 + f * lg + (r - kl * ll);
        end
      end
      p.lp = ref_pos(rg, tl, sl, el);
      p.gp = ref_pos(n, tg, sg, eg);
      return p;
    endfunction
  endclass

  // The design-C mesh bench's frame model (null otherwise): its checkers and
  // tests share it
  frame_ref the_fr;

  // ---------------------------------------------------------------------------
  // The controller of design D (D17, D18, D56), written from the tracker's
  // formulas, not from sched_pkg: real arithmetic where the RTL uses integer
  // tables. Pair turns per slot come from counting ordered pairs and slots;
  // alpha weighs one global slot against one local slot in every region.
  // (Plain variables everywhere: README rule 9.)
  // ---------------------------------------------------------------------------
  // Ordered pairs of routers one slot of an n x n timetable serves
  function automatic int ref_pair_turns(int n);
    int ns, np;
    ns = ref_n_slots(n);
    np = n * n * (n * n - 1);
    return np / ns;
  endfunction

  // alpha (D18): global pair turns per slot over the local pair turns of
  // one local slot in all (n / rg)^2 regions; 7/6 on 8x8 with 4x4 regions
  function automatic real ref_alpha(int n, int rg);
    int  pg, pl, nr, d;
    real a, b;
    pg = ref_pair_turns(n);
    pl = ref_pair_turns(rg);
    nr = n / rg;
    d  = pl * nr * nr;
    a  = real'(pg);
    b  = real'(d);
    return a / b;
  endfunction

  // D18's clamp: K_MIN 1; K_MAX 15 up to 8x8, 47 on 16x16
  function automatic int ref_kmax(int n);
    if (n >= 16) return 47;
    return 15;
  endfunction

  // Share bin q = floor(16 C_L / (C_L + C_G)), 0..16; -1 when both are 0
  function automatic int ref_bin(longint cl, longint cg);
    longint s, t;
    s = cl + cg;
    if (s == 0) return -1;
    t = 64'sd16 * cl;
    t = t / s;
    return int'(t);
  endfunction

  // The policy's k_L for bin q = 0..15 (D18): the bin's middle stands for it,
  // C_L / C_G = (2q + 1) / (31 - 2q); PROP alpha C_L / C_G, SQRT
  // alpha sqrt(C_L / C_G), rounded half up, clamped to [1, K_MAX]
  function automatic int ref_kl(int pol, int n, int rg, int q, int kmin = 1, int kmax = -1);
    real    al, r, u, v, x;
    int     kmx, k, pg, pl, nr, ui, vi;
    longint num, den, t;
    kmx = kmax;
    if (kmx < 0) kmx = ref_kmax(n);
    ui  = 2 * q + 1;
    vi  = 31 - 2 * q;
    if (pol == 1) begin
      al = ref_alpha(n, rg);
      u  = real'(ui);
      v  = real'(vi);
      r  = u / v;
      x  = $sqrt(r);
      x  = al * x;
    end else begin
      // a u / (b v) from exact integers: a quotient of exactly m + 1/2
      // stays exact for the rounding
      pg  = ref_pair_turns(n);
      pl  = ref_pair_turns(rg);
      nr  = n / rg;
      num = longint'(pg);
      t   = longint'(ui);
      num = num * t;
      den = longint'(pl);
      t   = longint'(nr);
      den = den * t * t;
      t   = longint'(vi);
      den = den * t;
      x   = real'(num);
      r   = real'(den);
      x   = x / r;
    end
    x = x + 0.5;
    k = $rtoi(x);
    if (k < kmin) k = kmin;
    if (k > kmx)  k = kmx;
    return k;
  endfunction

  // The next k_L (D56): no demand keeps k_old; C_G = 0 gives K_MAX; else the
  // policy table; damping takes the mean of old and new, rounded toward new
  // (kmin, kmax: the clamp; -1 is D18's K_MAX for the mesh)
  function automatic int ref_next_kl(int pol, bit damp, int n, int rg, longint cl, longint cg, int k_old,
                                     int kmin = 1, int kmax = -1);
    int q, kt, kn, kmx;
    kmx = kmax;
    if (kmx < 0) kmx = ref_kmax(n);
    q   = ref_bin(cl, cg);
    if (q < 0)        kt = k_old;
    else if (q >= 16) kt = kmx;
    else              kt = ref_kl(pol, n, rg, q, kmin, kmx);
    if (kt < kmin) kt = kmin;
    if (kt > kmx)  kt = kmx;
    if (!damp)           kn = kt;
    else if (kt > k_old) kn = (k_old + kt + 1) / 2;
    else                 kn = (k_old + kt) / 2;
    if (kn < kmin) kn = kmin;
    if (kn > kmx)  kn = kmx;
    return kn;
  endfunction

  // Cycle after an epoch tick by which the last tile of an n x n mesh has
  // the new plan (D55): the sums are whole in cycle 2n + W (W = 14 +
  // log2(n^2)), the plan is set 6 cycles later and needs 2n more registers
  // to reach tile (n-1, n-1)
  function automatic int ref_plan_by(int n);
    int nn, l, w;
    nn = n * n;
    l  = 0;
    for (int i = 0; i < 9; i++) begin
      if ((1 << l) < nn) l++;
    end
    w = 14 + l;
    return 4 * n + w + 6;
  endfunction

endpackage
