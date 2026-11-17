// =============================================================================
// File      rtl/ni/resv_ctrl.sv                                    Tracker: F45
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The receiving side of D12: one subslot ahead of every burst that
//           may come to this router, reserve B_MAX slots of the NI's bypass
//           space or, if they are not free, send a cancel token to the
//           burst's source; then follow each reservation until its burst has
//           landed (or did not come) and release it.
// Decisions D12 reservation and cancel, D13 setup window, D38 timing
//           contract, D48 reservation space, D16 frames (local gaps)
// Tests     U10 (tb/unit/tb_resv_ctrl.sv); BP03, BP06-BP10 (tb_mesh_b);
//           TL01-TL06 (tb_mesh_c, local and global subslots)
// -----------------------------------------------------------------------------
// The NI's ejection FIFO holds regular flits, at most NUM_VC * VC_DEPTH of
// them (the router sends them only against credits), and bypass flits, which
// come without flow control at the cycles the timetable fixes. The rest of
// the FIFO, SPACE flits (sched_pkg byp_space), is the bypass space. Counts:
//   fill   bypass flits in the FIFO (+1 written, -1 taken by the core)
//   res    flits reserved and still to come
// Rule: fill + res never exceeds SPACE, so the FIFO never overflows.
// Decision (dec_i in cycle d, D38): the timetable says a burst from
//   dec_src_i over dec_hops_i = L hops may come in the next subslot, which
//   starts at t = d + GAP (GAP0 = the setup window for subslot 0, decided in
//   its first cycle; GAPK = one subslot otherwise, decided in the first
//   cycle of the subslot before). In designs C and D a subslot of the local
//   timetable (dec_loc_i) has its own lengths, GAP0_L and GAPK_L; a decision
//   always falls in the slot of the subslot it decides, so its phase is the
//   phase of that subslot.
//     fill + res + B_MAX <= SPACE   res += B_MAX (accepted)
//     otherwise                     cx_o = {source, the other mode} goes to
//                                   the cancel relay now (cancelled: the
//                                   source will not send in that subslot)
// Follow-up of an accepted burst: its lookahead reaches this router at
//   t + 2L (D38). res then drops by B_MAX - B, B the burst's length from the
//   lookahead; with no lookahead in that cycle the source had nothing to
//   send (a no-show) and res drops by B_MAX. Each bypass flit written into
//   the FIFO moves one flit from res to fill. A reservation therefore ends
//   when its burst's last flit lands, at t + 2L + 2 + B, or at t + 2L for a
//   no-show; none outlives its burst (D12's timeout is this fixed bound;
//   the check below stops a run whose reserved flits stop coming).
// At most two reservations wait for their lookahead at once: a burst's
// lookahead arrives before the decision two subslots later (2 Hmax < T);
// each waits in a countdown to its lookahead's cycle. A decision and a
// lookahead never fall in the same cycle (2L < T). Both hold across a frame's
// phase change too: the last subslot of a slot decides nothing, so the next
// decision comes T cycles after that subslot starts, whichever timetable
// runs next.
// res_o and fill_o are the counts, for the testbench's model.
// =============================================================================
`timescale 1ns/1ps

module resv_ctrl
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int B_MAX = 10,                 // flits per burst
  parameter int SPACE = 28,                 // bypass space of the FIFO, flits (D48)
  parameter int GAP0  = s_setup(8),         // decision -> start of subslot 0
  parameter int GAPK  = t_tight(8, 10),     // decision -> start of a later subslot
  parameter int GAP0_L = GAP0,              // the same for the local timetable (designs C, D)
  parameter int GAPK_L = GAPK,
  parameter int CW    = $clog2(SPACE + 1)   // derived, do not set
) (
  input  logic          clk_i,
  input  logic          rst_ni,
  // the timetable: a burst may come in the next subslot; decide now
  input  logic          dec_i,
  input  logic          dec_s0_i,           // the next subslot is subslot 0
  input  logic          dec_loc_i,          // ... of the local timetable (designs C, D)
  input  node_id_t      dec_src_i,          // its source
  input  logic          dec_mode_i,         // the mode of its path
  input  hop_t          dec_hops_i,         // its length L
  // the router: the lookahead of a burst for this NI, this cycle
  input  la_t           la_i,
  // the NI's FIFO: a bypass flit written / taken by the core this cycle
  input  logic          push_i,
  input  logic          pop_i,
  // cancel token for the router's cancel relay
  output cx_t           cx_o,
  // the counts
  output logic [CW-1:0] res_o,
  output logic [CW-1:0] fill_o
);

  localparam int TW = 8;                    // countdown: GAP + 2L - 1 <= 127 + 60

  logic [CW-1:0] res_q, fill_q;
  logic [1:0]    w_vld_q;                   // reservations waiting for their lookahead
  logic [TW-1:0] w_cnt_q [2];               // cycles until each one is due
  logic          ok, acc, cxl, any_due, slot;
  logic [1:0]    due;
  logic [TW-1:0] load;
  int            res_n, gap;

  always_comb begin
    ok      = int'(fill_q) + int'(res_q) + B_MAX <= SPACE;
    acc     = dec_i && ok;
    cxl     = dec_i && !ok;
    due[0]  = w_vld_q[0] && (w_cnt_q[0] == '0);
    due[1]  = w_vld_q[1] && (w_cnt_q[1] == '0);
    any_due = |due;
    res_n   = int'(res_q);
    if (acc) res_n = res_n + B_MAX;
    if (any_due) begin
      if (la_i.vld) res_n = res_n - (B_MAX - int'(la_i.len));
      else          res_n = res_n - B_MAX;
    end
    if (push_i) res_n = res_n - 1;
  end

  // the waiting slot a new reservation takes, and its countdown
  assign slot = w_vld_q[0];
  always_comb begin
    if (dec_loc_i) gap = dec_s0_i ? GAP0_L : GAPK_L;
    else           gap = dec_s0_i ? GAP0 : GAPK;
  end
  assign load = TW'(gap + 2 * int'(dec_hops_i) - 1);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      res_q      <= '0;
      fill_q     <= '0;
      w_vld_q    <= '0;
      w_cnt_q[0] <= '0;
      w_cnt_q[1] <= '0;
    end else begin
      res_q  <= CW'(res_n);
      fill_q <= CW'(int'(fill_q) + int'(push_i) - int'(pop_i));
      for (int i = 0; i < 2; i++) begin
        if (due[i])          w_vld_q[i] <= 1'b0;
        else if (w_vld_q[i]) w_cnt_q[i] <= w_cnt_q[i] - 1'b1;
      end
      if (acc) begin
        if (slot) begin
          w_vld_q[1] <= 1'b1;
          w_cnt_q[1] <= load;
        end else begin
          w_vld_q[0] <= 1'b1;
          w_cnt_q[0] <= load;
        end
      end
    end
  end

  always_comb begin
    cx_o      = '0;
    cx_o.vld  = cxl;
    cx_o.tgt  = dec_src_i;
    cx_o.mode = !dec_mode_i;
  end

  assign res_o  = res_q;
  assign fill_o = fill_q;

  // the lookahead's destination, mode and vc field are the router's business
  logic unused_la;
  assign unused_la = ^{la_i.dst, la_i.mode, la_i.vc};

`ifndef SYNTHESIS
  // Counters (the testbench reads them; a hook, so outside the synthesis-off
  // pragmas): decisions, accepted, cancelled, no-shows, lookaheads matched,
  // bypass flits written, accepted while another reservation waited, and the
  // largest fill, res and fill + res seen
  longint unsigned n_dec, n_acc, n_cxl, n_noshow, n_la, n_in, n_two;
  int unsigned     max_fill, max_res, max_sum;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      n_dec    <= 0;
      n_acc    <= 0;
      n_cxl    <= 0;
      n_noshow <= 0;
      n_la     <= 0;
      n_in     <= 0;
      n_two    <= 0;
      max_fill <= 0;
      max_res  <= 0;
      max_sum  <= 0;
    end else begin
      if (dec_i)                n_dec    <= n_dec + 1;
      if (acc)                  n_acc    <= n_acc + 1;
      if (cxl)                  n_cxl    <= n_cxl + 1;
      if (any_due && !la_i.vld) n_noshow <= n_noshow + 1;
      if (any_due && la_i.vld)  n_la     <= n_la + 1;
      if (push_i)               n_in     <= n_in + 1;
      if (acc && w_vld_q != '0) n_two    <= n_two + 1;
      if (int'(fill_q) > int'(max_fill))               max_fill <= int'(fill_q);
      if (int'(res_q) > int'(max_res))                 max_res  <= int'(res_q);
      if (int'(fill_q) + int'(res_q) > int'(max_sum))  max_sum  <= int'(fill_q) + int'(res_q);
    end
  end

  // synopsys translate_off
  int unsigned idle_q;                      // cycles reserved flits have not come
  initial begin
    if (SPACE < B_MAX)
      $fatal(1, "%m: SPACE = %0d cannot hold one burst of B_MAX = %0d flits", SPACE, B_MAX);
    if (GAP0 < 1 || GAPK < 1 || GAP0 + 2 * hmax(MAX_MESH_N) > (1 << TW) || GAPK + 2 * hmax(MAX_MESH_N) > (1 << TW))
      $fatal(1, "%m: GAP0 = %0d, GAPK = %0d do not fit the %0d-bit countdown", GAP0, GAPK, TW);
    if (GAP0_L < 1 || GAPK_L < 1 || GAP0_L + 2 * hmax(MAX_MESH_N) > (1 << TW) || GAPK_L + 2 * hmax(MAX_MESH_N) > (1 << TW))
      $fatal(1, "%m: GAP0_L = %0d, GAPK_L = %0d do not fit the %0d-bit countdown", GAP0_L, GAPK_L, TW);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      idle_q <= 0;
    end else begin
      if ($isunknown({dec_i, la_i.vld, push_i, pop_i}) || (dec_i && $isunknown({dec_s0_i, dec_loc_i})))
        $fatal(1, "%m: X on dec_i, dec_s0_i, dec_loc_i, la_i.vld, push_i or pop_i");
      if (dec_i && (w_vld_q == 2'b11))
        $fatal(1, "%m: a third reservation waits for its lookahead");
      if (dec_i && any_due)
        $fatal(1, "%m: a decision in the cycle a lookahead is due (2L >= T?)");
      if (dec_i && dec_hops_i == '0)
        $fatal(1, "%m: a burst of 0 hops (from this router to itself)");
      if (due == 2'b11)
        $fatal(1, "%m: two reservations due in one cycle");
      if (la_i.vld && !any_due)
        $fatal(1, "%m: lookahead of a burst from nowhere reserved (%0d flits): the source ignored a cancel token?", la_i.len);
      if (la_i.vld && (la_i.len == '0 || int'(la_i.len) > B_MAX))
        $fatal(1, "%m: lookahead announces %0d flits (B_MAX %0d)", la_i.len, B_MAX);
      if (res_n < 0)
        $fatal(1, "%m: more bypass flits arrived than were reserved");
      if (pop_i && fill_q == '0)
        $fatal(1, "%m: the core took a bypass flit, but none is in the FIFO");
      if (int'(fill_q) + int'(res_q) > SPACE)
        $fatal(1, "%m: fill %0d + reserved %0d exceed the bypass space %0d", fill_q, res_q, SPACE);
      // reserved flits arrive back to back from 3 cycles after their lookahead
      if (res_q == '0 || push_i || any_due || w_vld_q != '0) idle_q <= 0;
      else                                                  idle_q <= idle_q + 1;
      if (idle_q > 2)
        $fatal(1, "%m: %0d reserved flits have not come for %0d cycles (timeout)", res_q, idle_q);
    end
  end
  // synopsys translate_on
`endif

endmodule
