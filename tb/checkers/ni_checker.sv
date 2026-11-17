// =============================================================================
// File      tb/checkers/ni_checker.sv                              Tracker: F47
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   An independent model of every NI's bypass space (D12, D48): it
//           makes each reservation decision from its own counts, follows
//           each accepted burst, and checks the RTL against it every cycle:
//           the decision (router_checker sees the RTL's cancel tokens on
//           the links), the reserved and filled counts, and that the FIFO
//           never holds more regular flits than the credits allow or more
//           flits than it has.
// Decisions D12 reservation and cancel, D38 timing contract, D40
//           verification split, D41 checker class, D48 reservation space
// Tests     BP01-BP10 (tb_mesh_b): no bypass without a reservation, no
//           overflow, cancels in time, release rules
// -----------------------------------------------------------------------------
// Per node, from the links and the sink alone:
//   fill   bypass flits written into the FIFO (ej_mon with byp) before this
//          cycle, minus those the core took (sink_node n_btake)
//   res    B_MAX for each accepted burst until its lookahead's cycle
//          t + 2L, then the burst's length (0 for a no-show), minus each of
//          its flits as it is written
// Decision for subslot k at cycle d (router_checker calls decide()): accept
// if fill + res + B_MAX <= SPACE (sched_pkg::byp_space), else cancel. The
// RTL must agree: router_checker expects the cancel token on every reverse
// link at d + h exactly when this model cancels, and nothing from the
// source then.
// Every cycle, per node: the RTL's res and fill (white box, resv_ctrl) equal
// the model's; fill + res <= SPACE; the FIFO's occupancy (EJ_DEPTH minus
// ej_free) is at most EJ_DEPTH, at least fill, and its regular part at
// most NUM_VC * VC_DEPTH.
// The checker is a class in a package (D41); router_checker owns its calls.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

package nchk_pkg;

  // node numbers index arrays of MAXN entries: their upper bits are unread
  /* verilator lint_off UNUSEDSIGNAL */

  localparam int MAXN = 256;            // routers

  class nchk;
    int              nn, bmax, space, ejd, regv;
    // model, per node
    int              res   [MAXN];
    longint          arr   [MAXN];      // bypass flits written, cycles before this one
    longint          taken [MAXN];      // bypass flits the core took (sink, sampled this cycle)
    int              wdue  [MAXN][$];   // accepted bursts: cycle their lookahead is due
    bit              arr_now [MAXN];    // a bypass flit is written this cycle
    // the RTL, sampled this cycle (bench)
    int              p_res  [MAXN];
    int              p_fill [MAXN];
    int              p_occ  [MAXN];     // flits in the FIFO
    // statistics
    longint unsigned n_dec, n_acc, n_cxl, n_noshow, n_la, n_cmp;
    int              max_fill, max_sum, max_occ, max_reg;

    function new(int nodes, int b_max, int space_c, int ej_depth, int reg_c);
      nn    = nodes;
      bmax  = b_max;
      space = space_c;
      ejd   = ej_depth;
      regv  = reg_c;
      res     = '{default: 0};
      arr     = '{default: 0};
      taken   = '{default: 0};
      arr_now = '{default: 1'b0};
      p_res   = '{default: 0};
      p_fill  = '{default: 0};
      p_occ   = '{default: 0};
      n_dec = 0; n_acc = 0; n_cxl = 0; n_noshow = 0; n_la = 0; n_cmp = 0;
      max_fill = 0; max_sum = 0; max_occ = 0; max_reg = 0;
    endfunction

    function int fill(int r);
      return int'(arr[r] - taken[r]);
    endfunction

    // Start of cycle c: the model against the RTL and the FIFO bounds
    function void compare(int c);
      int f, reg_part;
      for (int r = 0; r < nn; r++) begin
        f        = fill(r);
        reg_part = p_occ[r] - f;
        n_cmp++;
        tb_pkg::n_checks++;
        if (p_res[r] != res[r] || p_fill[r] != f)
          tb_pkg::fail($sformatf("NI R%0d at cycle %0d: reserved %0d, bypass flits in the FIFO %0d; the model says %0d and %0d",
                                 r, c, p_res[r], p_fill[r], res[r], f));
        `TB_CHECK(f >= 0 && res[r] >= 0 && f + res[r] <= space,
                  ("NI R%0d at cycle %0d: bypass flits %0d + reserved %0d, bypass space %0d", r, c, f, res[r], space))
        `TB_CHECK(p_occ[r] <= ejd && reg_part >= 0 && reg_part <= regv,
                  ("NI R%0d at cycle %0d: %0d flits in the FIFO, %0d of them bypass: %0d regular (credits allow %0d), depth %0d",
                   r, c, p_occ[r], f, reg_part, regv, ejd))
        if (f > max_fill)          max_fill = f;
        if (f + res[r] > max_sum)  max_sum  = f + res[r];
        if (p_occ[r] > max_occ)    max_occ  = p_occ[r];
        if (reg_part > max_reg)    max_reg  = reg_part;
      end
    endfunction

    // A burst may come to node r in the subslot decided now; its lookahead
    // would arrive at cycle due_c. Returns 1 to accept (reserve), 0 to cancel.
    function bit decide(int r, int due_c);
      bit ok;
      n_dec++;
      ok = fill(r) + res[r] + bmax <= space;
      if (ok) begin
        res[r] += bmax;
        wdue[r].push_back(due_c);
        n_acc++;
      end else begin
        n_cxl++;
      end
      return ok;
    endfunction

    // End of cycle c: the lookahead of the oldest accepted burst of node r is
    // due now; len flits (0: none came, a no-show)
    function void due(int r, int c, int len);
      int d;
      `TB_CHECK(wdue[r].size() > 0, ("NI R%0d at cycle %0d: a lookahead is due with no burst reserved", r, c))
      if (wdue[r].size() == 0) return;
      d = wdue[r].pop_front();
      `TB_CHECK(d == c, ("NI R%0d: the oldest reservation's lookahead is due at cycle %0d, not %0d", r, d, c))
      if (len == 0) begin
        res[r] -= bmax;
        n_noshow++;
      end else begin
        res[r] -= bmax - len;
        n_la++;
      end
    endfunction

    // End of cycle c: the bypass flits written this cycle
    function void end_cycle(int c);
      for (int r = 0; r < nn; r++) begin
        if (arr_now[r]) begin
          arr[r]++;
          res[r]--;
          `TB_CHECK(res[r] >= 0, ("NI R%0d at cycle %0d: a bypass flit arrived that no reservation covered", r, c))
        end
        arr_now[r] = 1'b0;
      end
    endfunction

    // Reservations still waiting for their lookahead, over all nodes
    function int waiting();
      int s;
      s = 0;
      for (int r = 0; r < nn; r++) s += wdue[r].size();
      return s;
    endfunction
  endclass

  // The bench's model (null when no design-B mesh is simulated)
  nchk the_nchk;
  /* verilator lint_on UNUSEDSIGNAL */

endpackage
