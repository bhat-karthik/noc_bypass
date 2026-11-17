// =============================================================================
// File      rtl/router/bypass_source.sv                            Tracker: F41
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The sending side of a bypass: at the first cycle of a subslot in
//           which the timetable lets this router send, unless a cancel token
//           came for that turn, find the whole packets waiting for the
//           partner, choose up to B_MAX flits of them, launch the lookahead
//           with the burst length, and stream the flits through crossbar
//           input 4 (Local), one per cycle.
// Decisions D07 bypass datapath, D11 burst size, D12 cancel tokens,
//           D38 timing contract, D42 bypass source, D47 area-lean choice
// Tests     BP01-BP10 (tb_mesh_b)
// -----------------------------------------------------------------------------
// Timing (D38), the subslot starting at cycle t with tx_go_i = 1:
//   t       a cancel token for this turn (cx_q): send nothing. Otherwise
//           choose the packets (combinational, from registered VC state);
//           the chosen VCs leave switch allocation (hold_o) and the
//           lookahead goes into the la ST register of the first-hop output
//           (relay), so it is on lookahead link 1 at t + 2
//   t+1+j   flit j leaves its VC (pop_o) and crosses the crossbar on input 4
//           towards the first-hop output: on link 1 at t + 3 + j
// Cancel token (D12, B5): the destination decides one subslot ahead and its
//   token reaches this router at cycle t - 1 at the latest (cancel_i, from
//   the cancel relay). cx_q keeps it until the next tx_go_i, the turn it
//   cancels, and clears there. A token in cycle t itself, or a second one
//   before the turn, cannot happen (D38) and stops the simulation.
// Eligible packets (D42): a whole packet at the front of any input VC of any
//   of the five input ports, its head not yet sent (input unit bp_rdy_o),
//   addressed to the partner. [1] lets packets at every inport become bypass
//   packets (Sec. III-D, Fig. 5), which is what breaks deadlock cycles.
// Choice (D42's rule, built as D47): each VC k = port * NUM_VC + vc gives an
//   eligibility record {ok, length}. The NP records are rotated once so that
//   the scan starts at ptr_q, scanned in fixed order with a 5-bit running
//   sum (a packet is taken if it still fits in B_MAX), and the choice is
//   rotated back. ptr_q moves by one VC every burst. A packet is left out
//   only if it would not fit after the packets before it, so nothing left
//   out fits in the space that remains (the choice is maximal, BP04). The
//   B4 form read every step through an NP-way index (20,472 cells); this
//   one has two rotators and one adder chain, with the same result in every
//   state.
// Read port: the flits enter the crossbar on input 4 whichever port they come
//   from. No passing burst ever enters on input 4 (the NI sends no bypass
//   flits), so a source never takes a crossbar input that a passing burst
//   uses: the rule D38 left to B4 holds by construction.
// Switch-allocation masks (1 = blocked):
//   input 4 and the first-hop output over [t, t + B - 1], so their crossbar
//   slots are free over [t + 1, t + B]; cycle t comes straight from the
//   choice, later cycles from the registered burst. The input port being read
//   in a cycle, so it sends one flit and one credit per cycle.
// Bypass flits carry byp = 1 and vc = 0; they use no credit or VC anywhere.
// The lookahead's vc field stays 0: a burst lands in the NI's shared FIFO,
// in the space the destination reserved for it (D48), not in a VC.
// Turn counters (simulation only, read by the testbench): turns offered,
// turns used, turns cancelled, cancelled turns that had packets to send,
// flits and packets sent, bursts of several packets, packets taken from a
// port other than Local, bursts of B_MAX flits, bursts chosen back to back
// with the previous one. They count with loops (D45: no cast of a system
// function's result).
// =============================================================================
`timescale 1ns/1ps

module bypass_source
  import noc_pkg::*;
#(
  parameter int MESH_N = 8,
  parameter int NUM_VC = 4,                         // 1..4
  parameter int B_MAX  = 10,                        // flits per burst, 5..15
  parameter int NP     = NPORTS * NUM_VC,           // derived, do not set
  parameter int KW     = $clog2(NPORTS * NUM_VC)    // derived, do not set
) (
  input  logic                           clk_i,
  input  logic                           rst_ni,
  input  coord_t                         cur_x_i,      // this router
  input  coord_t                         cur_y_i,
  // the timetable: this router may send in the subslot starting now
  input  logic                           tx_go_i,
  input  node_id_t                       tx_dst_i,     // the partner
  input  logic                           tx_mode_i,    // MODE_XY or MODE_YX
  // the cancel relay: a cancel token for this router arrived (its next turn)
  input  logic                           cancel_i,
  // read port of every input unit
  input  logic  [NPORTS-1:0][NUM_VC-1:0] rdy_i,        // whole packet waiting, nothing sent
  input  flit_t [NPORTS-1:0][NUM_VC-1:0] front_i,      // front flit of every VC
  output logic  [NPORTS-1:0][NUM_VC-1:0] hold_o,       // packets taken now
  output logic  [NPORTS-1:0][NUM_VC-1:0] pop_o,        // flit leaving now
  // crossbar input 4 (Local)
  output logic                           xb_en_o,
  output flit_t                          xb_flit_o,
  output port_t                          xb_port_o,
  // switch-allocation masks
  output logic  [NPORTS-1:0]             in_mask_o,
  output logic  [NPORTS-1:0]             out_mask_o,
  // lookahead for the relay's la ST register of output la_port_o
  output logic                           la_vld_o,
  output port_t                          la_port_o,
  output la_t                            la_o,
  output logic                           busy_o        // a burst is streaming
);

  localparam int RW = 1 + PKT_LEN_W;     // eligibility record: {ok, length}
  localparam int RB = NP * RW;           // all records
  localparam int SW = 5;                 // running sum: B_MAX + 5 <= 20 < 32

  // ---------------------------------------------------------------- state
  logic [NP-1:0]      sel_q;     // packets of the burst still to send
  logic [KW-1:0]      ptr_q;     // where the next choice starts its scan
  logic [KW-1:0]      start_q;   // where this burst's scan started: its order
  port_t              port_q;    // first-hop output of the burst
  logic [BURST_W-1:0] rem_q;     // flits of the burst still to send
  logic               cx_q;      // a cancel token for the next turn

  // ---------------------------------------------------------------- records
  // Flat view: VC (p, v) is entry k = p * NUM_VC + v
  flit_t           fr [NP];
  logic [NP*RW-1:0] rec;         // record k at bits [k*RW +: RW]

  always_comb begin
    /* verilator lint_off UNUSEDSIGNAL */
    head_t h;                    // only dst and len are read
    /* verilator lint_on UNUSEDSIGNAL */
    for (int p = 0; p < NPORTS; p++) begin
      for (int v = 0; v < NUM_VC; v++) begin
        h                                = head_t'(front_i[p][v].data);
        fr[p * NUM_VC + v]               = front_i[p][v];
        rec[(p * NUM_VC + v) * RW +: RW] = {rdy_i[p][v] && (h.dst == tx_dst_i), h.len};
      end
    end
  end

  // ---------------------------------------------------------------- choice (cycle t)
  // rotate: scan step m reads record (ptr_q + m) mod NP
  logic [2*RB-1:0] rec2;
  logic [RB-1:0]   rrec;

  assign rec2 = {rec, rec};
  assign rrec = RB'(rec2 >> (ptr_q * RW));

  // scan once in fixed order with a running sum
  logic [NP-1:0] tk;             // taken, in scan order
  logic [SW-1:0] sum;

  always_comb begin
    logic [SW-1:0] nx;
    sum = '0;
    for (int m = 0; m < NP; m++) begin
      nx    = sum + SW'(rrec[m * RW +: PKT_LEN_W]);
      tk[m] = rrec[m * RW + PKT_LEN_W] && (nx <= SW'(B_MAX));
      if (tk[m]) sum = nx;
    end
  end

  // rotate back: take[k] = tk[(k - ptr_q) mod NP]
  logic [2*NP-1:0]    tk2;
  logic [NP-1:0]      take;
  logic [BURST_W-1:0] blen;
  logic               go;
  port_t              p1;

  assign tk2  = {tk, tk};
  assign take = NP'(tk2 >> (NP - int'(ptr_q)));
  assign blen = BURST_W'(sum);
  assign go   = tx_go_i && !cx_q && (sum != '0);
  assign p1   = dor_port(cur_x_i, cur_y_i, id_x(tx_dst_i, MESH_N), id_y(tx_dst_i, MESH_N), tx_mode_i);

  // ---------------------------------------------------------------- streaming (t+1 ...)
  // cur: the first VC of the burst left, in the burst's order (from start_q)
  logic [2*NP-1:0] sel2;
  logic [NP-1:0]   srot;
  logic [KW-1:0]   m0, cur;
  logic            strm;         // a flit leaves this cycle
  logic            last_of_pkt;  // it is a tail

  assign sel2 = {sel_q, sel_q};
  assign srot = NP'(sel2 >> start_q);

  always_comb begin
    int cs;
    m0 = '0;
    for (int m = NP - 1; m >= 0; m--) begin
      if (srot[m]) m0 = KW'(m);                     // the lowest set bit wins
    end
    cs = int'(start_q) + int'(m0);
    if (cs >= NP) cs = cs - NP;
    cur         = KW'(cs);
    strm        = (sel_q != '0);
    last_of_pkt = is_tail(fr[cur].ftype);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sel_q   <= '0;
      ptr_q   <= '0;
      start_q <= '0;
      port_q  <= P_L;
      rem_q   <= '0;
      cx_q    <= 1'b0;
    end else begin
      if (go) begin
        sel_q   <= take;
        start_q <= ptr_q;
        ptr_q   <= (int'(ptr_q) == NP - 1) ? '0 : ptr_q + 1'b1;
        port_q  <= p1;
        rem_q   <= blen;
      end else if (strm) begin
        if (last_of_pkt) sel_q[cur] <= 1'b0;
        rem_q <= rem_q - 1'b1;
      end
      if (tx_go_i)       cx_q <= 1'b0;              // the turn the token was for
      else if (cancel_i) cx_q <= 1'b1;
    end
  end

  // ---------------------------------------------------------------- outputs
  always_comb begin
    hold_o     = '0;
    pop_o      = '0;
    in_mask_o  = '0;
    out_mask_o = '0;
    for (int p = 0; p < NPORTS; p++) begin
      for (int v = 0; v < NUM_VC; v++) begin
        hold_o[p][v] = go && take[p * NUM_VC + v];
        pop_o[p][v]  = strm && (int'(cur) == p * NUM_VC + v);
      end
    end
    // input 4 and the first-hop output: [t, t + B - 1]
    if (go || rem_q >= BURST_W'(2)) in_mask_o[P_L] = 1'b1;
    if (go)                         out_mask_o[p1] = 1'b1;
    if (rem_q >= BURST_W'(2))       out_mask_o[port_q] = 1'b1;
    // the port being read
    if (strm) in_mask_o[int'(cur) / NUM_VC] = 1'b1;
  end

  always_comb begin
    xb_en_o         = strm;
    xb_port_o       = port_q;
    xb_flit_o       = fr[cur];
    xb_flit_o.vld   = strm;
    xb_flit_o.vc    = '0;
    xb_flit_o.byp   = 1'b1;
    la_vld_o        = go;
    la_port_o       = p1;
    la_o            = '0;
    la_o.vld        = go;
    la_o.dst        = tx_dst_i;
    la_o.mode       = tx_mode_i;
    la_o.len        = blen;
  end

  assign busy_o = strm || cx_q;

`ifndef SYNTHESIS
  // Turn counters (the testbench reads them; M08). A hook, so under
  // `ifndef SYNTHESIS only, outside the synthesis-off pragmas: every
  // simulator keeps it. Counted with loops (D45).
  longint unsigned n_turns;          // subslots in which this router may send
  longint unsigned n_used;           // ... and sent a burst
  longint unsigned n_cxl;            // ... cancelled by a token from the destination
  longint unsigned n_lost;           // ... cancelled while packets for the partner waited
  longint unsigned n_flits;          // flits sent by bypass
  longint unsigned n_pkts;           // packets sent by bypass
  longint unsigned n_multi;          // bursts of two or more packets
  longint unsigned n_remote;         // packets taken from a port other than Local
  longint unsigned n_full;           // bursts of exactly B_MAX flits
  longint unsigned n_b2b;            // bursts chosen while the last one sent its last flit

  // packets of the choice: all of them, and those from the four mesh inputs
  // (entries 0..NP-NUM_VC-1)
  int unsigned     n_take_all, n_take_mesh;
  always_comb begin
    n_take_all  = 0;
    n_take_mesh = 0;
    for (int k = 0; k < NP; k++) begin
      if (take[k]) begin
        n_take_all = n_take_all + 1;
        if (k < NP - NUM_VC) n_take_mesh = n_take_mesh + 1;
      end
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      n_turns  <= 0;
      n_used   <= 0;
      n_cxl    <= 0;
      n_lost   <= 0;
      n_flits  <= 0;
      n_pkts   <= 0;
      n_multi  <= 0;
      n_remote <= 0;
      n_full   <= 0;
      n_b2b    <= 0;
    end else begin
      if (tx_go_i) n_turns <= n_turns + 1;
      if (tx_go_i && cx_q) begin
        n_cxl <= n_cxl + 1;
        if (sum != '0) n_lost <= n_lost + 1;
      end
      if (go && strm) n_b2b <= n_b2b + 1;
      if (go) begin
        n_used   <= n_used + 1;
        n_flits  <= n_flits + longint'(sum);
        n_pkts   <= n_pkts + longint'(n_take_all);
        n_remote <= n_remote + longint'(n_take_mesh);
        if (n_take_all > 1)         n_multi <= n_multi + 1;
        if (int'(sum) == B_MAX)     n_full  <= n_full + 1;
      end
    end
  end

  // synopsys translate_off
  initial begin
    if (NUM_VC < 1 || NUM_VC > MAX_VC)
      $fatal(1, "%m: NUM_VC = %0d, must be 1..%0d", NUM_VC, MAX_VC);
    if (B_MAX < MAX_PKT_FLITS || B_MAX > MAX_BURST)
      $fatal(1, "%m: B_MAX = %0d, must hold a %0d-flit packet and be at most %0d", B_MAX, MAX_PKT_FLITS, MAX_BURST);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      if ($isunknown({tx_go_i, cancel_i}))
        $fatal(1, "%m: X on tx_go_i or cancel_i");
      if (cancel_i && cx_q)
        $fatal(1, "%m: a second cancel token arrived before the turn the first one cancels");
      if (cancel_i && tx_go_i)
        $fatal(1, "%m: a cancel token arrived in the cycle of the turn itself, too late (D38)");
      if (go) begin
        if (tx_dst_i == xy_to_id(cur_x_i, cur_y_i, MESH_N))
          $fatal(1, "%m: the timetable makes this router send to itself");
        // a new burst may start only as the previous one sends its last flit
        if (strm && !(rem_q == BURST_W'(1) && $countones(sel_q) == 1 && last_of_pkt))
          $fatal(1, "%m: a burst starts while the previous one still has %0d flits to send", rem_q);
      end
      if (strm) begin
        if (!fr[cur].vld)
          $fatal(1, "%m: bypass reads VC %0d of port %0d, which is empty", int'(cur) % NUM_VC, int'(cur) / NUM_VC);
        if (rem_q == '0)
          $fatal(1, "%m: the burst sends more flits than its lookahead announced");
        if ($countones(sel_q) == 1 && last_of_pkt && rem_q != BURST_W'(1))
          $fatal(1, "%m: the burst ends with %0d announced flits unsent", int'(rem_q) - 1);
      end else if (rem_q != '0) begin
        $fatal(1, "%m: %0d announced flits never sent", rem_q);
      end
    end
  end
  // synopsys translate_on
`endif

endmodule
