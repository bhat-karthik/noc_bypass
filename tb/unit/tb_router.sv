// =============================================================================
// File      tb/unit/tb_router.sv                                   Tracker: F19
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Tests     R01 every input reaches every output with 1- and 5-flit packets on
//               every VC, intact; at zero load 4 cycles per hop, the credit
//               back after 2 cycles, and the head on output VC 0
//           R02 back-pressure: sinks read slowly or stop at random, so credits
//               and VCs are withheld, then all drain; no overflow, no loss,
//               traffic resumes
//           R03 packets from several VCs interleave on one output without
//               mixing; no deadlock
//           R04 adaptive routing follows the free downstream VCs (the U03
//               rule); a head keeps its route while it waits (D26)
// DUT       rtl/router/router.sv (F17) with F13-F16
// @tests    R01 R02 R03 R04
// -----------------------------------------------------------------------------
// Four routers run side by side: V = 4 with XY, V = 4 adaptive, V = 2
// adaptive and V = 1 with XY, each at (1,1) of a 4x4 mesh. Around each
// router the testbench models its five neighbours:
//   source models  drive the input links like an upstream output unit: one
//                  packet per VC, a credit per flit, a new packet only after
//                  the VC-free credit, flits of several VCs interleaved;
//   sink models    take the output links like a downstream input unit: a
//                  5-flit buffer per VC that must never overflow, at most one
//                  flit read per cycle, a credit per flit read, VC-free with
//                  the tail; reads can be slowed or held;
//   scoreboard     every flit is compared with what was sent (port, order,
//                  type, all 128 data bits); every packet must arrive once.
// Timing: the models act at the falling clock edge (read the DUT outputs,
// drive its inputs); the test tasks act at the rising edge, so the two never
// race and every simulator sees the same order of events.
// R01-R03 use destinations with one productive direction, so the expected
// port does not depend on the routing algorithm; every packet must also leave
// on a minimal path. +TEST=R02 runs one test; without +TEST all four run.
// Queue arrays have 2^k entries and are indexed only through pq() and pv(),
// which return unsigned k-bit values. Two reasons: the C++ that Verilator
// 5.020 emits for a guarded read of an array of queues whose index might be
// out of range does not compile; and a size cast keeps the signedness of its
// operand (IEEE 1800-2017 6.24.1), so 3'(4) is the 3-bit signed value -4, an
// index outside every array that Riviera and xsim drop (Verilator does not).
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module rt_tester
  import noc_pkg::*;
#(
  parameter int V    = 4,       // VCs per port
  parameter int ALGO = RT_XY,   // regular routing of the router
  parameter int TID  = 1        // tester number: packet ids and random streams
) (
  input  logic clk,
  input  logic rst_n,
  input  logic go,
  output logic done
);
  localparam int MN       = 4;  // 4x4 mesh
  localparam int RX       = 1;  // router under test at (1,1)
  localparam int RY       = 1;
  localparam int DEPTH    = 5;  // flits per VC
  localparam int LAT_HOP  = 4;  // zero load: flit on the input link -> on the output link
  localparam int LAT_CRED = 2;  // zero load: flit on the input link -> its credit back
  localparam logic [2:0] EXP_ANY = 3'd7;   // expected port decided by the test

  // ---------------------------------------------------------------- DUT
  flit_t   [NPORTS-1:0] in_flit  = '0;   // driven by the source models
  credit_t [NPORTS-1:0] in_cred;          // DUT -> source models
  flit_t   [NPORTS-1:0] out_flit;         // DUT -> sink models
  credit_t [NPORTS-1:0] out_cred = '0;   // driven by the sink models

  la_t [3:0] unused_la;     // lookahead outputs, zero in design A
  cx_t [3:0] unused_cx;     // cancel token outputs, zero in design A
  la_t       unused_lal;    // lookahead for the NI, zero in design A

  router #(
    .MESH_N   (MN),
    .NUM_VC   (V),
    .VC_DEPTH (DEPTH),
    .ALGO     (ALGO),
    .DESIGN   (DESIGN_A)
  ) dut (
    .clk_i        (clk),
    .rst_ni       (rst_n),
    .cur_x_i      (coord_t'(RX)),
    .cur_y_i      (coord_t'(RY)),
    .in_flit_i    (in_flit),
    .in_credit_o  (in_cred),
    .out_flit_o   (out_flit),
    .out_credit_i (out_cred),
    // bypass ports (B4, B5): unused by design A
    .in_la_i      ('0),
    .out_la_o     (unused_la),
    .in_cx_i      ('0),
    .out_cx_o     (unused_cx),
    .cx_inj_i     ('0),
    .la_loc_o     (unused_lal),
    .tx_go_i      (1'b0),
    .tx_dst_i     ('0),
    .tx_mode_i    (1'b0)
  );

  // ---------------------------------------------------------------- packets
  typedef struct packed {
    logic [31:0] id;       // tester, source port, sequence number
    logic [7:0]  dst;      // destination router
    logic [2:0]  len;      // flits, 1..5
    logic [2:0]  src;      // input port it enters on
    logic [2:0]  exp;      // output port it must leave on; EXP_ANY = checked by the test
    logic        any_vc;   // 1: the lowest idle input VC; 0: VC vc
    logic [1:0]  vc;
  } pkt_t;

  function automatic logic [31:0] sig32(logic [31:0] x);
    logic [63:0] z;
    z = tb_pkg::mix64({32'h0, x});
    return z[31:0] ^ z[63:32];
  endfunction

  // The 128 data bits of flit idx: a head carries head_t, the rest a pattern
  // tied to the whole packet descriptor and the flit index
  function automatic logic [FLIT_DATA_W-1:0] payload(pkt_t p, int idx);
    head_t       h;
    logic [63:0] z;
    if (idx == 0) begin
      h       = '0;
      h.dst   = p.dst;
      h.src   = node_id_t'(p.src);
      h.len   = p.len;
      h.cls   = (p.len > 3'd1) ? CLS_DATA : CLS_CTRL;
      h.seq   = p.id;
      h.check = sig32(p.id);
      return h;
    end
    z = tb_pkg::mix64(64'(p) ^ {32'(idx), 32'h9e37_79b9});
    return {p.id, 32'(idx), z};
  endfunction

  function automatic ftype_t ftype_of(int idx, int len);
    if (len == 1)       return FT_SINGLE;
    if (idx == 0)       return FT_HEAD;
    if (idx == len - 1) return FT_TAIL;
    return FT_BODY;
  endfunction

  // A destination whose only productive direction from (1,1) is port q
  function automatic int dst_for(int q, int r);
    case (q)
      0:       return 1;                  // (1,0)
      1:       return (r % 2 != 0) ? 7 : 6;    // (3,1) or (2,1)
      2:       return (r % 2 != 0) ? 13 : 9;   // (1,3) or (1,2)
      3:       return 4;                  // (0,1)
      default: return 5;                  // (1,1): itself, the Local port
    endcase
  endfunction

  // 1 when output q of (RX,RY) lies on a minimal path to router dst
  function automatic bit productive(int q, int dst);
    int dx, dy;
    dx = dst % MN - RX;
    dy = dst / MN - RY;
    case (q)
      0:       return dy < 0;             // N
      1:       return dx > 0;             // E
      2:       return dy > 0;             // S
      3:       return dx < 0;             // W
      default: return dx == 0 && dy == 0;
    endcase
  endfunction

  // Unsigned indices into the queue arrays (see the note at the top): a type
  // cast to an unsigned type gives an unsigned value; a size cast would not
  typedef logic [4:0] slot_t;    // per-VC queues: 32 entries
  typedef logic [2:0] pslot_t;   // per-port queue: 8 entries

  // Slot of (port, VC) in the per-VC queue arrays
  function automatic slot_t pv(int p, int v);
    if (p < 0 || p >= NPORTS || v < 0 || v >= V) $fatal(1, "pv: port %0d VC %0d out of range", p, v);
    return slot_t'(p * 4 + v);
  endfunction

  // Slot of a port in txq
  function automatic pslot_t pq(int p);
    if (p < 0 || p >= NPORTS) $fatal(1, "pq: port %0d out of range", p);
    return pslot_t'(p);
  endfunction

  // ---------------------------------------------------------------- scoreboard
  pkt_t        sb_pkt   [int unsigned];       // every packet sent, by id
  int          sb_port  [int unsigned];       // port it arrived on, -1 not yet
  bit          sb_done  [int unsigned];       // tail arrived
  int unsigned sb_tsend [longint unsigned];   // cycle flit (id, idx) was on the input link
  int          sb_left = 0;                   // packets not yet complete at a sink
  int          seq [NPORTS];
  int unsigned cyc = 0;                       // cycle number
  bit          chk_lat = 1'b0;                // R01: zero-load timing checks

  always @(posedge clk) cyc <= cyc + 1;

  function automatic int unsigned send(int src, int dst, int len, logic [2:0] exp, bit any_vc, int vc);
    pkt_t p;
    int   n;
    `TB_CHECK(src >= 0 && src < NPORTS && dst >= 0 && dst < MN * MN && len >= 1 &&
              len <= MAX_PKT_FLITS && vc >= 0 && vc < V,
      ("T%0d send: bad packet src %0d dst %0d len %0d vc %0d", TID, src, dst, len, vc))
    p.id     = {8'(TID), 3'(src), 21'(seq[src])};
    seq[src] = seq[src] + 1;
    p.dst    = 8'(dst);
    p.len    = 3'(len);
    p.src    = 3'(src);
    p.exp    = exp;
    p.any_vc = any_vc;
    p.vc     = 2'(vc);
    sb_pkt[p.id]  = p;
    sb_port[p.id] = -1;
    sb_done[p.id] = 1'b0;
    sb_left++;
    n = txq[pq(src)].size();
    txq[pq(src)].push_back(p);
    // a dropped push would otherwise show up only as a timeout much later
    `TB_CHECK(txq[pq(src)].size() == n + 1, ("T%0d send: packet %h was not queued at port %0d", TID, p.id, src))
    return p.id;
  endfunction

  // ---------------------------------------------------------------- source models
  pkt_t        txq    [8][$];          // packets waiting to start, at pq(port)
  bit          s_busy [NPORTS][4];     // input VC held (until its VC-free credit)
  int          s_cred [NPORTS][4];     // credits of the input VC
  bit          s_has  [NPORTS][4];     // packet in progress with flits left
  pkt_t        s_pkt  [NPORTS][4];
  int          s_idx  [NPORTS][4];     // next flit index
  int          s_rr   [NPORTS];
  real         s_rate [NPORTS];        // chance of sending in a cycle
  int unsigned s_ctq  [32][$];         // send cycles of flits whose credit is pending, at pv()
  int          sid_src, sid_snk, sid_tst;

  longint unsigned c_novc = 0, c_vcfree = 0;

  function automatic void sources_cycle();
    flit_t f;
    int    v, k;
    bit    sent, blocked;
    pkt_t  p;
    for (int sp = 0; sp < NPORTS; sp++) begin
      // credits returned by the DUT input unit
      if (in_cred[sp].vld) begin
        v = int'(in_cred[sp].vc);
        `TB_CHECK(v < V, ("T%0d port %0d: credit for VC %0d", TID, sp, v))
        if (v < V) begin
          s_cred[sp][v]++;
          `TB_CHECK(s_cred[sp][v] <= DEPTH, ("T%0d port %0d VC %0d: %0d credits", TID, sp, v, s_cred[sp][v]))
          `TB_CHECK(s_ctq[pv(sp, v)].size() > 0, ("T%0d port %0d VC %0d: credit for no flit", TID, sp, v))
          if (s_ctq[pv(sp, v)].size() > 0) begin
            k = int'(cyc - s_ctq[pv(sp, v)][0]);
            void'(s_ctq[pv(sp, v)].pop_front());
            if (chk_lat) begin
              `TB_CHECK(k == LAT_CRED, ("T%0d port %0d VC %0d: credit after %0d cycles, expected %0d at zero load",
                                         TID, sp, v, k, LAT_CRED))
            end
          end
          if (in_cred[sp].vc_free) begin
            `TB_CHECK(s_busy[sp][v] && !s_has[sp][v] && s_cred[sp][v] == DEPTH,
              ("T%0d port %0d VC %0d: VC-free with busy=%0b flits left=%0b credits=%0d", TID, sp, v,
               s_busy[sp][v], s_has[sp][v], s_cred[sp][v]))
            s_busy[sp][v] = 1'b0;
            c_vcfree++;
          end
        end
      end
      // send at most one flit
      f    = '0;
      sent = 1'b0;
      // back-pressure on this source: its next packet finds no input VC free
      if (txq[pq(sp)].size() > 0) begin
        blocked = 1'b1;
        for (int vv = 0; vv < V; vv++) begin
          if (!s_busy[sp][vv] && (txq[pq(sp)][0].any_vc || int'(txq[pq(sp)][0].vc) == vv)) blocked = 1'b0;
        end
        if (blocked) c_novc++;
      end
      if (tb_pkg::rng_chance(sid_src, s_rate[sp])) begin
        for (int kk = 0; kk < V; kk++) begin
          v = (s_rr[sp] + kk) % V;
          if (!sent && s_has[sp][v] && s_cred[sp][v] > 0) begin
            sent = 1'b1;
          end else if (!sent && !s_busy[sp][v] && txq[pq(sp)].size() > 0 &&
                       (txq[pq(sp)][0].any_vc || int'(txq[pq(sp)][0].vc) == v)) begin
            p              = txq[pq(sp)][0];
            void'(txq[pq(sp)].pop_front());
            s_pkt[sp][v]   = p;
            s_idx[sp][v]   = 0;
            s_busy[sp][v]  = 1'b1;
            s_has[sp][v]   = 1'b1;
            sent           = 1'b1;
          end
          if (sent && f.vld == 1'b0) begin
            p       = s_pkt[sp][v];
            f.vld   = 1'b1;
            f.ftype = ftype_of(s_idx[sp][v], int'(p.len));
            f.vc    = vc_id_t'(v);
            f.data  = payload(p, s_idx[sp][v]);
            sb_tsend[{32'h0, p.id} * 8 + longint'(s_idx[sp][v])] = cyc;
            s_ctq[pv(sp, v)].push_back(cyc);
            s_cred[sp][v]--;
            s_idx[sp][v]++;
            if (s_idx[sp][v] == int'(p.len)) s_has[sp][v] = 1'b0;
            s_rr[sp] = (v + 1) % V;
          end
        end
      end
      in_flit[sp] = f;
    end
  endfunction

  // ---------------------------------------------------------------- sink models
  // The data of a flit is checked when it arrives; the buffer keeps its type
  // (a tail frees the VC when read) and its arrival cycle (not read the same cycle)
  typedef struct packed {
    ftype_t      ft;
    logic [31:0] t;
  } rxf_t;

  rxf_t        rxb     [32][$];        // buffer of output port q, VC v at pv(q, v)
  bit          k_alloc [NPORTS][4];    // VC holds a packet (head in, tail not yet read)
  int unsigned k_id    [NPORTS][4];
  int          k_cnt   [NPORTS][4];    // flits of that packet received
  int          k_rr    [NPORTS];
  real         k_rate  [NPORTS];       // chance of reading a flit in a cycle
  bit          k_hold  [NPORTS];       // read nothing
  int          k_last  [NPORTS];       // VC of the last flit on the output link

  longint unsigned c_full = 0, c_hold = 0, c_inter = 0, c_allvc = 0;
  longint unsigned c_outvc [4];

  function automatic void receive(int q, flit_t f);
    int          v, idx;
    int unsigned id;
    head_t       h, e;
    pkt_t        p;
    rxf_t        x;
    longint unsigned key;
    if (!f.vld) return;
    v = int'(f.vc);
    h = head_t'(f.data);
    `TB_CHECK(!f.byp, ("T%0d output %0d: bypass flit from a design-A router", TID, q))
    `TB_CHECK(v < V, ("T%0d output %0d: flit on VC %0d", TID, q, v))
    if (v >= V) return;
    `TB_CHECK(rxb[pv(q, v)].size() < DEPTH,
      ("T%0d output %0d VC %0d: overflow, the router sent without a credit", TID, q, v))
    if (rxb[pv(q, v)].size() == DEPTH - 1) c_full++;
    if (is_head(f.ftype)) begin
      `TB_CHECK(!k_alloc[q][v], ("T%0d output %0d VC %0d: new head while the VC still holds a packet", TID, q, v))
      id = h.seq;
      `TB_CHECK(sb_pkt.exists(id) != 0, ("T%0d output %0d: head of unknown packet %h", TID, q, id))
      if (sb_pkt.exists(id) == 0) return;
      `TB_CHECK(sb_port[id] == -1, ("T%0d output %0d: packet %h arrived twice", TID, q, id))
      `TB_CHECK(productive(q, int'(sb_pkt[id].dst)),
        ("T%0d packet %h to router %0d left on port %0d, off every minimal path", TID, id, sb_pkt[id].dst, q))
      k_alloc[q][v] = 1'b1;
      k_id[q][v]    = id;
      k_cnt[q][v]   = 0;
      sb_port[id]   = q;
      c_outvc[v]++;
      if (chk_lat) begin
        `TB_CHECK(v == 0, ("T%0d output %0d: at zero load a head takes VC 0, not %0d", TID, q, v))
      end
    end else begin
      `TB_CHECK(k_alloc[q][v] && k_cnt[q][v] > 0,
        ("T%0d output %0d VC %0d: body or tail without a head", TID, q, v))
      if (!(k_alloc[q][v] && k_cnt[q][v] > 0)) return;
    end
    // interleaving: a flit of another VC while the last VC's packet is
    // unfinished. Nested ifs: sb_pkt is read only with the id of a held VC
    // (in Verilator 5.020 any read of a missing key creates it, and && does
    // not stop the read)
    if (k_last[q] >= 0 && k_last[q] != v) begin
      if (k_alloc[q][k_last[q]]) begin
        if (k_cnt[q][k_last[q]] < int'(sb_pkt[k_id[q][k_last[q]]].len)) c_inter++;
      end
    end
    k_last[q] = v;
    id  = k_id[q][v];
    idx = k_cnt[q][v];
    p   = sb_pkt[id];
    `TB_CHECK(idx < int'(p.len), ("T%0d output %0d VC %0d: packet %h has more than %0d flits", TID, q, v, id, p.len))
    `TB_CHECK(f.ftype == ftype_of(idx, int'(p.len)),
      ("T%0d output %0d VC %0d: flit %0d of packet %h has type %b", TID, q, v, idx, id, f.ftype))
    if (idx == 0) begin
      e = head_t'(payload(p, 0));
      `TB_CHECK(h == e,
        ("T%0d output %0d VC %0d: head of packet %h is corrupt (got/expected dst %0d/%0d src %0d/%0d len %0d/%0d check %h/%h)",
         TID, q, v, id, h.dst, e.dst, h.src, e.src, h.len, e.len, h.check, e.check))
    end else begin
      `TB_CHECK(f.data == payload(p, idx),
        ("T%0d output %0d VC %0d: flit %0d of packet %h is corrupt", TID, q, v, idx, id))
    end
    if (p.exp != EXP_ANY) begin
      `TB_CHECK(q == int'(p.exp), ("T%0d packet %h left on port %0d, expected %0d", TID, id, q, p.exp))
    end
    key = {32'h0, id} * 8 + longint'(idx);
    `TB_CHECK(sb_tsend.exists(key) != 0, ("T%0d packet %h flit %0d arrived before it was sent", TID, id, idx))
    if (chk_lat && sb_tsend.exists(key) != 0) begin
      `TB_CHECK(cyc - sb_tsend[key] == LAT_HOP,
        ("T%0d packet %h flit %0d: %0d cycles from input %0d to output %0d, expected %0d at zero load",
         TID, id, idx, cyc - sb_tsend[key], p.src, q, LAT_HOP))
    end
    k_cnt[q][v]++;
    x.ft = f.ftype;
    x.t  = cyc;
    rxb[pv(q, v)].push_back(x);
    if (is_tail(f.ftype)) begin
      `TB_CHECK(k_cnt[q][v] == int'(p.len), ("T%0d packet %h ended after %0d of %0d flits", TID, id, k_cnt[q][v], p.len))
      sb_done[id] = 1'b1;
      sb_left--;
    end
  endfunction

  function automatic void sinks_cycle();
    credit_t c;
    int      v;
    bit      popped, taken;
    for (int q = 0; q < NPORTS; q++) begin
      receive(q, out_flit[q]);
      // the router's output q has no free VC: every sink VC holds a packet
      taken = 1'b1;
      for (int vv = 0; vv < V; vv++) begin
        if (!k_alloc[q][vv]) taken = 1'b0;
      end
      if (taken) c_allvc++;
      c      = '0;
      popped = 1'b0;
      if (k_hold[q]) c_hold++;
      else if (tb_pkg::rng_chance(sid_snk, k_rate[q])) begin
        for (int kk = 0; kk < V; kk++) begin
          v = (k_rr[q] + kk) % V;
          if (!popped && rxb[pv(q, v)].size() > 0 && rxb[pv(q, v)][0].t < cyc) begin
            c.vld     = 1'b1;
            c.vc      = vc_id_t'(v);
            c.vc_free = is_tail(rxb[pv(q, v)][0].ft);
            if (c.vc_free) k_alloc[q][v] = 1'b0;
            void'(rxb[pv(q, v)].pop_front());
            popped  = 1'b1;
            k_rr[q] = (v + 1) % V;
          end
        end
      end
      out_cred[q] = c;
    end
  endfunction

  bit models_on = 1'b0;

  // models_on rises after reset, so the models need not look at rst_n
  always @(negedge clk) begin
    if (models_on) begin
      sinks_cycle();
      sources_cycle();
    end
  end

  // ---------------------------------------------------------------- helpers
  function automatic string algo_name();
    if (ALGO == RT_XY) return "XY";
    return "adaptive";
  endfunction

  function automatic bit all_idle();
    for (int sp = 0; sp < NPORTS; sp++) begin
      if (txq[pq(sp)].size() != 0) return 1'b0;
      for (int v = 0; v < V; v++) begin
        if (s_busy[sp][v] || s_has[sp][v] || s_cred[sp][v] != DEPTH) return 1'b0;
        if (k_alloc[sp][v] || rxb[pv(sp, v)].size() != 0) return 1'b0;
      end
    end
    return 1'b1;
  endfunction

  function automatic void set_rates(real src, real snk);
    for (int sp = 0; sp < NPORTS; sp++) begin
      s_rate[sp] = src;
      k_rate[sp] = snk;
      k_hold[sp] = 1'b0;
    end
  endfunction

  // Wait until every packet arrived and every credit and VC is home
  task automatic wait_idle(string what, int timeout);
    int t;
    t = 0;
    while (!(sb_left == 0 && all_idle()) && t < timeout) begin
      @(posedge clk);
      t++;
    end
    `TB_CHECK(sb_left == 0 && all_idle(),
      ("T%0d %s: not idle after %0d cycles, %0d packets still in flight", TID, what, timeout, sb_left))
  endtask

  task automatic wait_arrived(string what, int timeout);
    int t;
    t = 0;
    while (sb_left != 0 && t < timeout) begin
      @(posedge clk);
      t++;
    end
    `TB_CHECK(sb_left == 0, ("T%0d %s: %0d packets missing after %0d cycles", TID, what, sb_left, timeout))
  endtask

  // ---------------------------------------------------------------- R01
  task automatic test_r01();
    longint unsigned n;
    n = 0;
    tb_pkg::phase($sformatf("T%0d R01: every input to every output, every VC, 1 and 5 flits", TID));
    set_rates(1.0, 1.0);
    chk_lat = 1'b1;
    for (int sp = 0; sp < NPORTS; sp++)
      for (int q = 0; q < NPORTS; q++)
        for (int v = 0; v < V; v++)
          for (int l = 0; l < 2; l++) begin
            void'(send(sp, dst_for(q, v), (l == 0) ? 1 : MAX_PKT_FLITS, 3'(q), 1'b0, v));
            wait_idle("R01", 200);
            n++;
          end
    chk_lat = 1'b0;
    tb_pkg::cover_bin($sformatf("T%0d R01 input-output-VC-length cases", TID), n);
  endtask

  // ---------------------------------------------------------------- R02
  task automatic test_r02();
    int    q, len;
    real   rates [4];
    longint unsigned novc0, allvc0, hold0, full0;
    tb_pkg::phase($sformatf("T%0d R02: back-pressure from random sink stalls", TID));
    rates[0] = 0.1; rates[1] = 0.3; rates[2] = 0.6; rates[3] = 1.0;
    novc0   = c_novc;
    allvc0  = c_allvc;
    hold0   = c_hold;
    full0   = c_full;
    for (int sp = 0; sp < NPORTS; sp++) begin
      s_rate[sp] = 0.6 + 0.4 * real'(tb_pkg::rng_below(sid_tst, 5)) / 4.0;
      for (int n = 0; n < 300; n++) begin
        q   = int'(tb_pkg::rng_below(sid_tst, NPORTS));
        len = tb_pkg::rng_chance(sid_tst, 0.6) ? MAX_PKT_FLITS : 1;
        if (tb_pkg::rng_chance(sid_tst, 0.1)) len = 2 + int'(tb_pkg::rng_below(sid_tst, 3));
        void'(send(sp, dst_for(q, n), len, 3'(q), 1'b1, 0));
      end
    end
    for (int w = 0; w < 60; w++) begin
      for (int k = 0; k < NPORTS; k++) begin
        k_rate[k] = rates[tb_pkg::rng_below(sid_tst, 4)];
        k_hold[k] = tb_pkg::rng_chance(sid_tst, 0.3);
      end
      repeat (100) @(posedge clk);
    end
    set_rates(1.0, 1.0);
    wait_idle("R02", 20000);
    // No bin for a flit waiting for a credit: a VC takes a new packet only
    // when every credit is home, and VC_DEPTH holds the longest packet, so
    // back-pressure shows as busy VCs. Each credit is still checked above.
    tb_pkg::cover_bin($sformatf("T%0d R02 a packet found every input VC busy (cycles)", TID), c_novc - novc0);
    tb_pkg::cover_bin($sformatf("T%0d R02 an output had every VC taken (cycles)", TID), c_allvc - allvc0);
    tb_pkg::cover_bin($sformatf("T%0d R02 a sink held its reads (cycles)", TID), c_hold - hold0);
    tb_pkg::cover_bin($sformatf("T%0d R02 a sink VC buffer filled to %0d flits", TID, DEPTH), c_full - full0);
  endtask

  // ---------------------------------------------------------------- R03
  task automatic test_r03();
    longint unsigned inter0;
    longint unsigned outvc0 [4];
    tb_pkg::phase($sformatf("T%0d R03: all inputs and VCs to one output, mixed lengths", TID));
    inter0 = c_inter;
    for (int v = 0; v < 4; v++) outvc0[v] = c_outvc[v];
    set_rates(1.0, 1.0);
    for (int q = 0; q < NPORTS; q++) begin
      for (int sp = 0; sp < NPORTS; sp++) begin
        for (int n = 0; n < 3 * V; n++) begin
          void'(send(sp, dst_for(q, n), (n % 3 == 0) ? 1 : ((n % 3 == 1) ? MAX_PKT_FLITS : 3), 3'(q), 1'b1, 0));
        end
      end
      wait_idle("R03", 5000);
    end
    if (V > 1) tb_pkg::cover_bin($sformatf("T%0d R03 flits of two packets interleaved on a link", TID), c_inter - inter0);
    for (int v = 0; v < V; v++)
      tb_pkg::cover_bin($sformatf("T%0d R03 output VC %0d used", TID, v), c_outvc[v] - outvc0[v]);
  endtask

  // ---------------------------------------------------------------- R04
  longint unsigned c_y_more = 0, c_tie = 0, c_x_more = 0, c_kept = 0;

  // Occupy kx output VCs behind port px and ky behind py, then send a probe
  // that may go either way; it must take the port the U03 rule names.
  task automatic r04_case(int px, int py, int probe_src, int probe_dst, int kx, int ky, bit keep_test);
    int unsigned id;
    int          exp;
    int          t;
    k_hold[px] = 1'b1;
    k_hold[py] = 1'b1;
    for (int n = 0; n < kx; n++) void'(send(int'(P_L), dst_for(px, n), 1, 3'(px), 1'b1, 0));
    for (int n = 0; n < ky; n++) void'(send(int'(P_L), dst_for(py, n), 1, 3'(py), 1'b1, 0));
    wait_arrived("R04 occupy", 200);
    repeat (2) @(posedge clk);
    // free VCs now: V-kx behind px, V-ky behind py
    if (ALGO == RT_ADAPTIVE && (V - ky) > (V - kx)) exp = py;
    else                                            exp = px;
    if (ALGO == RT_ADAPTIVE) begin
      if ((V - ky) > (V - kx))      c_y_more++;
      else if ((V - ky) == (V - kx)) c_tie++;
      else                          c_x_more++;
    end
    id = send(probe_src, probe_dst, 1, EXP_ANY, 1'b1, 0);
    t  = 0;
    while (sb_tsend.exists({32'h0, id} * 8) == 0 && t < 100) begin
      @(posedge clk);
      t++;
    end
    @(posedge clk);                              // the probe's head is now in a VC
    if (keep_test) begin
      // free the other port first: the probe must still wait for its own port
      k_hold[py] = 1'b0;
      repeat (40) @(posedge clk);
      `TB_CHECK(sb_port[id] == -1,
        ("T%0d R04: the probe left on port %0d before its own port %0d had a free VC", TID, sb_port[id], px))
      c_kept++;
    end
    k_hold[px] = 1'b0;
    k_hold[py] = 1'b0;
    wait_idle("R04", 500);
    `TB_CHECK(sb_port[id] == exp,
      ("T%0d R04 %s: free VCs %0d behind port %0d, %0d behind port %0d: probe left on %0d, expected %0d",
       TID, algo_name(), V - kx, px, V - ky, py, sb_port[id], exp))
  endtask

  task automatic test_r04();
    tb_pkg::phase($sformatf("T%0d R04: route choice against free downstream VCs (%s)", TID, algo_name()));
    set_rates(1.0, 1.0);
    for (int kx = 0; kx <= V; kx++) begin
      for (int ky = 0; ky <= V; ky++) begin
        r04_case(int'(P_E), int'(P_S), int'(P_N), 10, kx, ky, 1'b0);   // to (2,2): E or S
        r04_case(int'(P_W), int'(P_N), int'(P_S), 0,  kx, ky, 1'b0);   // to (0,0): W or N
      end
    end
    // all VCs busy on both sides: a tie, so X; then free Y first (D26)
    r04_case(int'(P_E), int'(P_S), int'(P_W), 10, V, V, 1'b1);
    r04_case(int'(P_W), int'(P_N), int'(P_E), 0,  V, V, 1'b1);
    if (ALGO == RT_ADAPTIVE) begin
      tb_pkg::cover_bin($sformatf("T%0d R04 Y had more free VCs", TID), c_y_more);
      tb_pkg::cover_bin($sformatf("T%0d R04 tie, X taken", TID), c_tie);
      tb_pkg::cover_bin($sformatf("T%0d R04 X had more free VCs", TID), c_x_more);
    end
    tb_pkg::cover_bin($sformatf("T%0d R04 route kept while waiting for a VC", TID), c_kept);
  endtask

  // ---------------------------------------------------------------- main
  initial begin
    done = 1'b0;
    for (int sp = 0; sp < NPORTS; sp++) begin
      seq[sp]    = 0;
      s_rr[sp]   = 0;
      k_rr[sp]   = 0;
      k_last[sp] = -1;
      for (int v = 0; v < 4; v++) begin
        s_busy[sp][v]  = 1'b0;
        s_has[sp][v]   = 1'b0;
        s_cred[sp][v]  = DEPTH;
        s_idx[sp][v]   = 0;
        k_alloc[sp][v] = 1'b0;
        k_id[sp][v]    = 0;
        k_cnt[sp][v]   = 0;
      end
    end
    for (int v = 0; v < 4; v++) c_outvc[v] = 0;
    set_rates(1.0, 1.0);
    wait (go === 1'b1);
    sid_src = 4000 + 4 * TID;
    sid_snk = sid_src + 1;
    sid_tst = sid_src + 2;
    tb_pkg::rng_init(sid_src);
    tb_pkg::rng_init(sid_snk);
    tb_pkg::rng_init(sid_tst);
    @(posedge clk);
    models_on = 1'b1;
    if (tb_pkg::test_on("R01")) test_r01();
    if (tb_pkg::test_on("R02")) test_r02();
    if (tb_pkg::test_on("R03")) test_r03();
    if (tb_pkg::test_on("R04")) test_r04();
    tb_pkg::info($sformatf("T%0d (V=%0d, %s): %0d packets checked, %0d VC-free credits", TID, V,
                           algo_name(), sb_pkt.num(), c_vcfree));
    `TB_CHECK(c_vcfree == longint'(sb_pkt.num()),
      ("T%0d: %0d packets but %0d VC-free credits", TID, sb_pkt.num(), c_vcfree))
    done = 1'b1;
  end

endmodule


module tb_router;
  import noc_pkg::*;

  logic       clk   = 1'b0;
  logic       rst_n = 1'b0;
  logic       go    = 1'b0;
  logic [3:0] done;

  always #5 clk = ~clk;

  rt_tester #(.V(4), .ALGO(RT_XY),       .TID(1)) t_v4_xy (.clk(clk), .rst_n(rst_n), .go(go), .done(done[0]));
  rt_tester #(.V(4), .ALGO(RT_ADAPTIVE), .TID(2)) t_v4_ad (.clk(clk), .rst_n(rst_n), .go(go), .done(done[1]));
  rt_tester #(.V(2), .ALGO(RT_ADAPTIVE), .TID(3)) t_v2_ad (.clk(clk), .rst_n(rst_n), .go(go), .done(done[2]));
  rt_tester #(.V(1), .ALGO(RT_XY),       .TID(4)) t_v1_xy (.clk(clk), .rst_n(rst_n), .go(go), .done(done[3]));

  `TB_DUMP(tb_router)

  initial tb_pkg::watchdog(20ms);

  initial begin
    tb_pkg::start("R01-R04", "router: ports, VCs, timing, back-pressure, interleaving, adaptive choice");
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    go    = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
