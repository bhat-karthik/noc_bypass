// =============================================================================
// File      tb/unit/tb_switch_allocator.sv                         Tracker: F18
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U04: the switch allocator gives one grant per input and output,
//           an output VC only if free, respects masks, starves no one
// DUT       rtl/router/switch_allocator.sv (F14)
// @tests    U04
// -----------------------------------------------------------------------------
// Four allocators run side by side, V = 1, 2, 3 and 4 VCs per port. Each is
// compared every cycle with a reference model of the allocation rule (D28,
// Garnet's SwitchAllocator): grants, VCs and output VCs must match exactly.
// Separately from the model, every cycle also checks the properties the
// router relies on:
//   one grant at most per input and per output; a grant only for a VC that
//   may compete (flit waiting, VC or credit available, ports not masked);
//   a head takes the lowest idle VC of its output, other flits keep theirs.
// Phases
//   1 Random     10,000 cycles of random requests, ports, VC states, credits
//                and masks, in four densities.
//   2 Persistent every VC of every input requests all the time: each must
//                win within NPORTS x V cycles (the round-robin bound).
// Pass: no mismatch, no property violation, every coverage bin hit.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module sa_tester
  import noc_pkg::*;
#(
  parameter int V      = 4,
  parameter int CYCLES = 10000
) (
  input  logic clk,
  input  logic rst_n,
  input  logic go,
  output logic done
);
  // ---------------------------------------------------------------- DUT
  logic   [NPORTS-1:0][V-1:0] req       = '0;
  port_t  [NPORTS-1:0][V-1:0] port      = '0;
  logic   [NPORTS-1:0][V-1:0] active    = '0;
  vc_id_t [NPORTS-1:0][V-1:0] outvc     = '0;
  logic   [NPORTS-1:0]        has_free  = '0;
  vc_id_t [NPORTS-1:0]        alloc_vc  = '0;
  logic   [NPORTS-1:0][V-1:0] credit_ok = '0;
  logic   [NPORTS-1:0]        in_mask   = '0;
  logic   [NPORTS-1:0]        out_mask  = '0;
  logic   [NPORTS-1:0]        in_gnt, out_gnt, out_alloc;
  vc_id_t [NPORTS-1:0]        in_vc, in_outvc, out_vc;

  switch_allocator #(.NUM_VC(V)) dut (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .req_i       (req),
    .port_i      (port),
    .active_i    (active),
    .outvc_i     (outvc),
    .has_free_i  (has_free),
    .alloc_vc_i  (alloc_vc),
    .credit_ok_i (credit_ok),
    .in_mask_i   (in_mask),
    .out_mask_i  (out_mask),
    .in_gnt_o    (in_gnt),
    .in_vc_o     (in_vc),
    .in_outvc_o  (in_outvc),
    .out_gnt_o   (out_gnt),
    .out_alloc_o (out_alloc),
    .out_vc_o    (out_vc)
  );

  // ---------------------------------------------------------------- reference model
  int m_in_ptr  [NPORTS];   // per input: VC with the highest priority
  int m_out_ptr [NPORTS];   // per output: input with the highest priority
  int m_pick    [NPORTS];   // stage 1: VC chosen by each input, -1 none
  int m_win     [NPORTS];   // stage 2: input that wins each output, -1 none

  int sid;                  // random stream

  // The rule: may VC (i, v) compete this cycle?
  function automatic bit may_compete(int i, int v);
    int p;
    p = int'(port[i][v]);
    if (!req[i][v] || in_mask[i] || out_mask[p]) return 1'b0;
    if (active[i][v]) return credit_ok[p][int'(outvc[i][v])];
    return has_free[p];
  endfunction

  function automatic void model_eval();
    int v, i;
    for (int ii = 0; ii < NPORTS; ii++) begin
      m_pick[ii] = -1;
      for (int k = 0; k < V; k++) begin
        v = (m_in_ptr[ii] + k) % V;
        if (m_pick[ii] < 0 && may_compete(ii, v)) m_pick[ii] = v;
      end
    end
    for (int o = 0; o < NPORTS; o++) begin
      m_win[o] = -1;
      for (int k = 0; k < NPORTS; k++) begin
        i = (m_out_ptr[o] + k) % NPORTS;
        if (m_win[o] < 0 && m_pick[i] >= 0 && int'(port[i][m_pick[i]]) == o) m_win[o] = i;
      end
    end
  endfunction

  function automatic int model_outvc(int i);
    int v;
    v = m_pick[i];
    return active[i][v] ? int'(outvc[i][v]) : int'(alloc_vc[port[i][v]]);
  endfunction

  function automatic void model_update();
    for (int o = 0; o < NPORTS; o++) begin
      if (m_win[o] >= 0) begin
        m_out_ptr[o]       = (m_win[o] + 1) % NPORTS;
        m_in_ptr[m_win[o]] = (m_pick[m_win[o]] + 1) % V;   // only winners move
      end
    end
  endfunction

  // ---------------------------------------------------------------- coverage
  longint unsigned c_conflict = 0, c_alloc = 0, c_active = 0, c_cred_block = 0,
                   c_free_block = 0, c_in_mask = 0, c_out_mask = 0, c_five = 0,
                   c_loser = 0;
  int wait_cnt [NPORTS][V];
  int wait_max = 0;
  bit fair_phase = 1'b0;

  // ---------------------------------------------------------------- checks
  task automatic check_cycle();
    int  n_out, gi, p, v, n_gnt, n_pick [NPORTS];
    model_eval();
    // exact comparison with the model
    for (int i = 0; i < NPORTS; i++) begin
      bit exp_gnt;
      exp_gnt = (m_pick[i] >= 0) && (m_win[port[i][m_pick[i]]] == i);
      `TB_CHECK(in_gnt[i] === exp_gnt,
        ("V=%0d input %0d: in_gnt %b, model %b", V, i, in_gnt[i], exp_gnt))
      if (exp_gnt) begin
        `TB_CHECK(int'(in_vc[i]) == m_pick[i] && int'(in_outvc[i]) == model_outvc(i),
          ("V=%0d input %0d: VC %0d outVC %0d, model VC %0d outVC %0d", V, i, in_vc[i],
           in_outvc[i], m_pick[i], model_outvc(i)))
      end
    end
    for (int o = 0; o < NPORTS; o++) begin
      `TB_CHECK(out_gnt[o] === (m_win[o] >= 0),
        ("V=%0d output %0d: out_gnt %b, model winner %0d", V, o, out_gnt[o], m_win[o]))
      if (m_win[o] >= 0) begin
        gi = m_win[o];
        `TB_CHECK(out_alloc[o] === !active[gi][m_pick[gi]] && int'(out_vc[o]) == model_outvc(gi),
          ("V=%0d output %0d: alloc %b VC %0d, model alloc %b VC %0d", V, o, out_alloc[o],
           out_vc[o], !active[gi][m_pick[gi]], model_outvc(gi)))
      end else begin
        `TB_CHECK(out_alloc[o] === 1'b0, ("V=%0d output %0d: alloc without a grant", V, o))
      end
    end
    // properties, from the raw inputs and the DUT outputs only
    n_gnt = 0;
    for (int o = 0; o < NPORTS; o++) begin
      n_out = 0;
      for (int i = 0; i < NPORTS; i++) begin
        if (in_gnt[i] === 1'b1 && int'(port[i][in_vc[i]]) == o) n_out++;
      end
      `TB_CHECK(n_out == int'(out_gnt[o]),
        ("V=%0d output %0d: %0d inputs granted, out_gnt %b", V, o, n_out, out_gnt[o]))
    end
    for (int i = 0; i < NPORTS; i++) begin
      if (in_gnt[i] === 1'b1) begin
        n_gnt++;
        v = int'(in_vc[i]);
        p = int'(port[i][v]);
        `TB_CHECK(v < V && may_compete(i, v),
          ("V=%0d input %0d: VC %0d granted but it may not compete", V, i, v))
        `TB_CHECK(active[i][v] ? (in_outvc[i] == outvc[i][v]) : (in_outvc[i] == alloc_vc[p] && has_free[p]),
          ("V=%0d input %0d: output VC %0d is wrong", V, i, in_outvc[i]))
      end
    end
    // coverage
    for (int o = 0; o < NPORTS; o++) n_pick[o] = 0;
    for (int i = 0; i < NPORTS; i++) begin
      if (m_pick[i] >= 0) begin
        n_pick[port[i][m_pick[i]]]++;
        if (m_win[port[i][m_pick[i]]] != i) c_loser++;
      end
      for (int vv = 0; vv < V; vv++) begin
        if (req[i][vv]) begin
          if (in_mask[i]) c_in_mask++;
          else if (out_mask[port[i][vv]]) c_out_mask++;
          else if (active[i][vv] && !credit_ok[port[i][vv]][int'(outvc[i][vv])]) c_cred_block++;
          else if (!active[i][vv] && !has_free[port[i][vv]]) c_free_block++;
        end
      end
      if (in_gnt[i] === 1'b1) begin
        if (active[i][int'(in_vc[i])]) c_active++; else c_alloc++;
      end
    end
    for (int o = 0; o < NPORTS; o++) if (n_pick[o] > 1) c_conflict++;
    if (n_gnt == NPORTS) c_five++;
  endtask

  // the round-robin bound, checked while every VC requests all the time
  function automatic void track_waits();
    for (int i = 0; i < NPORTS; i++) begin
      for (int v = 0; v < V; v++) begin
        if (in_gnt[i] === 1'b1 && int'(in_vc[i]) == v) wait_cnt[i][v] = 0;
        else begin
          wait_cnt[i][v]++;
          if (wait_cnt[i][v] > wait_max) wait_max = wait_cnt[i][v];
          `TB_CHECK(wait_cnt[i][v] < NPORTS * V,
            ("V=%0d input %0d VC %0d waited %0d cycles (bound %0d)", V, i, v, wait_cnt[i][v], NPORTS * V))
        end
      end
    end
  endfunction

  // ---------------------------------------------------------------- stimulus
  function automatic void randomize_inputs(real p_req);
    for (int i = 0; i < NPORTS; i++) begin
      for (int v = 0; v < V; v++) begin
        req[i][v]       = tb_pkg::rng_chance(sid, p_req);
        port[i][v]      = port_t'(tb_pkg::rng_below(sid, NPORTS));
        active[i][v]    = tb_pkg::rng_chance(sid, 0.5);
        outvc[i][v]     = vc_id_t'(tb_pkg::rng_below(sid, V));
        credit_ok[i][v] = tb_pkg::rng_chance(sid, 0.75);
      end
      has_free[i] = tb_pkg::rng_chance(sid, 0.75);
      alloc_vc[i] = vc_id_t'(tb_pkg::rng_below(sid, V));
      in_mask[i]  = tb_pkg::rng_chance(sid, 0.08);
      out_mask[i] = tb_pkg::rng_chance(sid, 0.08);
    end
  endfunction

  // One cycle: inputs change after the falling edge, outputs are checked,
  // the DUT and the model step at the rising edge.
  task automatic step();
    #1;
    check_cycle();
    if (fair_phase) track_waits();
    @(posedge clk);
    model_update();
    @(negedge clk);
  endtask

  initial begin
    real p_req;
    done = 1'b0;
    for (int i = 0; i < NPORTS; i++) begin
      m_in_ptr[i]  = 0;
      m_out_ptr[i] = 0;
      for (int v = 0; v < V; v++) wait_cnt[i][v] = 0;
    end
    wait (go === 1'b1);
    sid = 3000 + V;
    tb_pkg::rng_init(sid);
    @(negedge clk);
    // phase 1: random
    for (int c = 0; c < CYCLES; c++) begin
      case ((c / 500) % 4)
        0:       p_req = 0.15;
        1:       p_req = 0.4;
        2:       p_req = 0.7;
        default: p_req = 0.95;
      endcase
      randomize_inputs(p_req);
      step();
    end
    // phase 2: persistent requests, nothing blocked
    for (int i = 0; i < NPORTS; i++) begin
      for (int v = 0; v < V; v++) begin
        req[i][v]       = 1'b1;
        port[i][v]      = port_t'(tb_pkg::rng_below(sid, NPORTS));
        active[i][v]    = tb_pkg::rng_chance(sid, 0.5);
        outvc[i][v]     = vc_id_t'(tb_pkg::rng_below(sid, V));
        credit_ok[i][v] = 1'b1;
      end
      has_free[i] = 1'b1;
      in_mask[i]  = 1'b0;
      out_mask[i] = 1'b0;
    end
    fair_phase = 1'b1;
    for (int c = 0; c < 40 * NPORTS * V; c++) step();
    fair_phase = 1'b0;
    req = '0;
    tb_pkg::cover_bin($sformatf("V=%0d two or more inputs want one output", V), c_conflict);
    tb_pkg::cover_bin($sformatf("V=%0d a stage-1 winner loses its output", V), c_loser);
    tb_pkg::cover_bin($sformatf("V=%0d head granted with a new output VC", V), c_alloc);
    tb_pkg::cover_bin($sformatf("V=%0d flit granted on its held output VC", V), c_active);
    tb_pkg::cover_bin($sformatf("V=%0d request blocked: no credit", V), c_cred_block);
    tb_pkg::cover_bin($sformatf("V=%0d request blocked: no idle output VC", V), c_free_block);
    tb_pkg::cover_bin($sformatf("V=%0d request blocked: input masked", V), c_in_mask);
    tb_pkg::cover_bin($sformatf("V=%0d request blocked: output masked", V), c_out_mask);
    tb_pkg::cover_bin($sformatf("V=%0d five grants in one cycle", V), c_five);
    tb_pkg::info($sformatf("V=%0d longest wait with persistent requests: %0d cycles (bound %0d)",
                           V, wait_max, NPORTS * V));
    done = 1'b1;
  end

endmodule


module tb_switch_allocator;
  logic       clk   = 1'b0;
  logic       rst_n = 1'b0;
  logic       go    = 1'b0;
  logic [3:0] done;

  always #5 clk = ~clk;

  sa_tester #(.V(1)) t_v1 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[0]));
  sa_tester #(.V(2)) t_v2 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[1]));
  sa_tester #(.V(3)) t_v3 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[2]));
  sa_tester #(.V(4)) t_v4 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[3]));

  `TB_DUMP(tb_switch_allocator)

  initial tb_pkg::watchdog(10ms);

  initial begin
    tb_pkg::start("U04", "switch_allocator: one grant per input and output, VCs, masks, no starvation");
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    go    = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
