// =============================================================================
// File      tb/common/sink_node.sv                                 Tracker: F25
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Traffic sink of one node: takes flits from the NI (always ready,
//           random stalls, or a forced stall window), timestamps ejection,
//           reassembles packets per VC and checks every flit.
// Decisions D14 network interface, D32 latency accounting, D45 bypass packets
// Tests     N01-N08, BP01-BP10 and every mesh run
// -----------------------------------------------------------------------------
// Ejection time: ej_mon is the router's Local output link. A flit on it in
// cycle T reaches the NI in T; the sink records T and the flit, in arrival
// order. The NI FIFO keeps that order, so the k-th flit taken must be the
// k-th that arrived: type, VC and data equal (checks the NI's FIFO).
// Checks on every flit taken, per VC (a bypass flit, byp = 1, uses a context
// of its own, MAX_VC: a burst brings whole packets back to back):
//   every  equal to the flit that arrived, bypass bit included; valid set;
//          a bypass flit carries VC 0
//   head   the VC held no packet; addressed to this node; head signature,
//          length, class and reserved bits right; the scoreboard has the
//          packet in flight
//   other  a head came first; data equal to body_data(); a tail exactly at
//          the packet's last flit
// Each delivered packet goes to the statistics with its tail's ejection time
// and whether it came by bypass, after a check that its latency is not below
// the zero-load latency of its hop count (D32; for a bypass packet the bypass
// bound zero_load_byp, D45): only contention and waiting can add to it.
// With traffic_pkg::log_on it also goes to the delivery log.
// Forced stall: fs_node is this node or FS_ALL, and the cycle is in
// [fs_start, fs_end).
// Stalls: ready is decided on the falling edge for the next rising edge.
// n_btake counts the bypass flits taken, at the rising edge that takes them,
// for the NI model of ni_checker (read at the next falling edge).
// @support tb_mesh tb_mesh_dl tb_mesh_b tb_mesh_c tb_mesh_cb tb_mesh_d
// (the EDA bundle compiles this module only for the tests of those benches)
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module sink_node
  import noc_pkg::*;
  import traffic_pkg::*;
#(
  parameter int ID     = 0,      // node id
  parameter int NUM_VC = 4,
  parameter int SID    = 400     // random stream of this sink
) (
  input  logic        clk,
  input  logic        rst_n,
  input  int unsigned cyc,
  input  logic        ej_valid,
  output logic        ej_ready,
  input  flit_t       ej_flit,
  input  flit_t       ej_mon
);

  typedef struct packed {
    logic [31:0] t;                    // cycle the flit reached the NI
    flit_t       f;
  } arr_t;
  arr_t            arr_q [$];          // flits in the NI FIFO, in arrival order
  arr_t            na;                 // a flit arriving
  bit              got      = 1'b0;    // a flit was taken at the last rising edge
  flit_t           got_flit;
  // packet being reassembled, per VC, and one more context for bypass flits
  localparam int   NCX = MAX_VC + 1;
  bit              busy [NCX];
  int              src_ [NCX];
  int unsigned     seq_ [NCX];
  int              idx_ [NCX];
  int              len_ [NCX];
  int unsigned     tc_  [NCX];
  int              last_vc  = -1;
  bit              is_idle  = 1'b1;    // no packet open and nothing in the NI FIFO
  // counters for coverage
  longint unsigned n_flits = 0, n_pkts = 0, n_stall = 0, n_inter = 0, max_fifo = 0;
  longint unsigned n_zl    = 0;        // packets delivered in exactly their zero-load latency
  longint unsigned n_byp   = 0;        // packets delivered by bypass
  longint unsigned n_btake = 0;        // bypass flits taken from the NI (rising edge)

  initial begin
    ej_ready = 1'b1;
    got_flit = '0;
    for (int v = 0; v < NCX; v++) busy[v] = 1'b0;
  end

  always @(posedge clk) begin
    got      <= ej_valid & ej_ready;
    got_flit <= ej_flit;
    if (ej_valid && ej_ready && ej_flit.byp) n_btake <= n_btake + 1;
  end

  // A packet's tail is in: no packet can beat the pipeline (D32), so its
  // latency is at least the zero-load latency of its hop count
  // v: the context (a VC, or MAX_VC for a bypass packet)
  function automatic void deliver(int v, int unsigned t_tail);
    int     n, h, zl;
    longint lat;
    bit     byp;
    traffic_pkg::dlog_t d;
    byp = (v == MAX_VC);
    n   = traffic_pkg::mesh_n;
    h   = hops(coord_t'(src_[v] % n), coord_t'(src_[v] / n), coord_t'(ID % n), coord_t'(ID / n));
    if (byp) zl = zero_load_byp(h, len_[v]);
    else     zl = zero_load(h, len_[v]);
    lat = longint'(t_tail) - longint'(tc_[v]);
    `TB_CHECK(lat >= longint'(zl),
              ("node %0d: packet %0d.%0d (%0d flits, %0d hops, bypass %0b) took %0d cycles, below its least latency %0d",
               ID, src_[v], seq_[v], len_[v], h, byp, lat, zl))
    if (lat == longint'(zl)) n_zl++;
    if (byp) n_byp++;
    busy[v] = 1'b0;
    n_pkts++;
    stats_pkg::on_deliver(src_[v], ID, len_[v], tc_[v], t_tail, zl, byp);
    sb_pkg::delivered(cyc);
    if (traffic_pkg::log_on) begin
      d.src      = 8'(src_[v]);
      d.dst      = 8'(ID);
      d.seq      = seq_[v];
      d.len      = 3'(len_[v]);
      d.t_create = tc_[v];
      d.t_tail   = t_tail;
      d.byp      = byp;
      traffic_pkg::dlog.push_back(d);
    end
  endfunction

  function automatic void take_flit(flit_t f);
    int           v;
    int unsigned  t_arr;
    arr_t         a;
    head_t        h;
    sb_pkg::rec_t r;
    bit           known;
    `TB_CHECK(arr_q.size() > 0, ("node %0d: flit taken but none arrived", ID))
    if (arr_q.size() == 0) return;
    a     = arr_q[0];
    t_arr = a.t;
    void'(arr_q.pop_front());
    n_flits++;
    `TB_CHECK(f.vld && f.byp == a.f.byp && f.ftype == a.f.ftype && f.vc == a.f.vc && f.data == a.f.data,
              ("node %0d: the NI gave flit %b/VC %0d/byp %0b/%h, but flit %b/VC %0d/byp %0b/%h arrived first",
               ID, f.ftype, f.vc, f.byp, f.data, a.f.ftype, a.f.vc, a.f.byp, a.f.data))
    v = int'(f.vc);
    `TB_CHECK(v < NUM_VC && !(f.byp && v != 0), ("node %0d: flit on VC %0d (bypass %0b), %0d VCs", ID, v, f.byp, NUM_VC))
    if (v >= NUM_VC) return;
    if (f.byp) v = MAX_VC;               // the bypass context
    // flits of two packets interleaved: another VC while the last VC's packet is open
    if (last_vc >= 0 && last_vc != v) begin
      if (busy[last_vc]) n_inter++;
    end
    last_vc = v;
    if (is_head(f.ftype)) begin
      h = head_t'(f.data);
      `TB_CHECK(!busy[v], ("node %0d VC %0d: head of packet %0d.%0d while packet %0d.%0d is open",
                           ID, v, h.src, h.seq, src_[v], seq_[v]))
      `TB_CHECK(int'(h.dst) == ID, ("node %0d: packet %0d.%0d for node %0d delivered here",
                                    ID, h.src, h.seq, h.dst))
      `TB_CHECK(h.check == head_check(int'(h.src), h.seq, int'(h.dst), int'(h.len), h.t_create),
                ("node %0d VC %0d: head of packet %0d.%0d is corrupt", ID, v, h.src, h.seq))
      `TB_CHECK((h.len == 3'd1 || h.len == 3'(MAX_PKT_FLITS)) && ((f.ftype == FT_SINGLE) == (h.len == 3'd1)),
                ("node %0d VC %0d: packet %0d.%0d has length %0d and head type %b",
                 ID, v, h.src, h.seq, h.len, f.ftype))
      `TB_CHECK(h.rsvd_hi == '0 && h.rsvd_lo == '0 && h.cls == ((h.len == 3'd1) ? CLS_CTRL : CLS_DATA),
                ("node %0d VC %0d: packet %0d.%0d has class %b, reserved bits %h %h",
                 ID, v, h.src, h.seq, h.cls, h.rsvd_hi, h.rsvd_lo))
      known = sb_pkg::take(int'(h.src), h.seq, r);
      `TB_CHECK(known, ("node %0d: packet %0d.%0d is unknown or was delivered before", ID, h.src, h.seq))
      if (known) begin
        `TB_CHECK(int'(r.dst) == ID && r.len == h.len && r.t_create == h.t_create,
                  ("node %0d: packet %0d.%0d differs from the one created", ID, h.src, h.seq))
      end
      busy[v] = 1'b1;
      src_[v] = int'(h.src);
      seq_[v] = h.seq;
      idx_[v] = 1;
      len_[v] = int'(h.len);
      tc_[v]  = h.t_create;
      if (is_tail(f.ftype)) deliver(v, t_arr);
    end else begin
      `TB_CHECK(busy[v], ("node %0d VC %0d: body or tail flit without a head", ID, v))
      if (!busy[v]) return;
      `TB_CHECK(f.data == body_data(src_[v], seq_[v], idx_[v]),
                ("node %0d VC %0d: flit %0d of packet %0d.%0d is corrupt", ID, v, idx_[v], src_[v], seq_[v]))
      `TB_CHECK(is_tail(f.ftype) == (idx_[v] == len_[v] - 1),
                ("node %0d VC %0d: flit %0d of a %0d-flit packet has type %b", ID, v, idx_[v], len_[v], f.ftype))
      idx_[v]++;
      if (is_tail(f.ftype)) deliver(v, t_arr);
    end
  endfunction

  always @(negedge clk) begin
    if (rst_n) begin
      if (ej_mon.vld) begin
        na.t = cyc;
        na.f = ej_mon;
        arr_q.push_back(na);
        stats_pkg::on_arrive(cyc);
        if (longint'(arr_q.size()) > max_fifo) max_fifo = longint'(arr_q.size());
      end
      if (got) take_flit(got_flit);
      // ready for the next rising edge (stall settings in traffic_pkg);
      // nested ifs keep the draws identical on every simulator (D30)
      ej_ready = 1'b1;
      if ((fs_node == ID || fs_node == FS_ALL) && cyc >= fs_start && cyc < fs_end) begin
        ej_ready = 1'b0;
      end else if (sink_stall_p > 0.0) begin
        if (tb_pkg::rng_chance(SID, sink_stall_p)) ej_ready = 1'b0;
      end
      if (!ej_ready) n_stall++;
      is_idle = (arr_q.size() == 0);
      for (int v = 0; v < NCX; v++) begin
        if (busy[v]) is_idle = 1'b0;
      end
    end
  end

endmodule
