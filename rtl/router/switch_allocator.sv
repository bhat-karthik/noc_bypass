// =============================================================================
// File      rtl/router/switch_allocator.sv                         Tracker: F14
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Stage 2 of the router, VA+SA in one cycle: chooses at most one
//           flit per input port and per output port, and gives a head its
//           output VC together with the switch, as Garnet does.
// Decisions D03 pipeline, D05 credits, D07 bypass masks, D28 allocation
// Tests     U04 (tb/unit/tb_switch_allocator.sv), R01-R04
// -----------------------------------------------------------------------------
// A VC (input i, VC v) may compete when it has a flit at its front and
//   - a head (not yet active) needs an idle VC at its output port,
//   - any other flit needs a credit on the output VC its packet holds,
//   - neither its input nor its output port is masked this cycle.
// Separable input-first allocation with round-robin arbiters (rr_arbiter):
//   1. per input port, one competing VC;
//   2. per output port, one of the inputs whose chosen VC wants it.
// An output arbiter moves its priority past every winner. An input arbiter
// moves its priority only when its VC also won the output, so a VC that
// loses the second stage keeps its turn (Garnet updates m_round_robin_invc
// the same way). A head that wins takes alloc_vc_i of its output: the lowest
// idle VC there. Grants are combinational; the input units register them.
// Masks (B4): in_mask_i blocks an input whose crossbar input carries a bypass
// flit; out_mask_i blocks an output a bypass burst occupies. Both are zero in
// design A.
// =============================================================================
`timescale 1ns/1ps

module switch_allocator
  import noc_pkg::*;
#(
  parameter int NUM_VC = 4,                              // 1..4
  parameter int VCI_W  = (NUM_VC > 1) ? $clog2(NUM_VC) : 1, // derived, do not set
  parameter int PI_W   = $clog2(NPORTS)                  // derived, do not set
) (
  input  logic                            clk_i,
  input  logic                            rst_ni,
  // per input port and VC, from the input units
  input  logic   [NPORTS-1:0][NUM_VC-1:0] req_i,       // a flit waits at the front
  input  port_t  [NPORTS-1:0][NUM_VC-1:0] port_i,      // its output port
  input  logic   [NPORTS-1:0][NUM_VC-1:0] active_i,    // its packet holds an output VC
  input  vc_id_t [NPORTS-1:0][NUM_VC-1:0] outvc_i,     // that output VC
  // per output port, from the output units
  input  logic   [NPORTS-1:0]             has_free_i,  // an output VC is idle
  input  vc_id_t [NPORTS-1:0]             alloc_vc_i,  // the lowest idle one
  input  logic   [NPORTS-1:0][NUM_VC-1:0] credit_ok_i, // per output VC: a credit is left
  // bypass masks, 1 = blocked this cycle (zero in design A)
  input  logic   [NPORTS-1:0]             in_mask_i,
  input  logic   [NPORTS-1:0]             out_mask_i,
  // grants per input port
  output logic   [NPORTS-1:0]             in_gnt_o,    // input i sends a flit
  output vc_id_t [NPORTS-1:0]             in_vc_o,     // from this VC
  output vc_id_t [NPORTS-1:0]             in_outvc_o,  // on this output VC
  // grants per output port
  output logic   [NPORTS-1:0]             out_gnt_o,   // output o gets a flit
  output logic   [NPORTS-1:0]             out_alloc_o, // a head that takes alloc_vc_i[o]
  output vc_id_t [NPORTS-1:0]             out_vc_o     // the output VC used
);

  // ---------------------------------------------------------------- eligibility
  logic [NPORTS-1:0][NUM_VC-1:0] elig;

  always_comb begin
    port_t p;      // output port of VC (i, v)
    logic  cok;    // its output VC has a credit
    for (int i = 0; i < NPORTS; i++) begin
      for (int v = 0; v < NUM_VC; v++) begin
        p   = port_i[i][v];
        cok = 1'b0;
        for (int k = 0; k < NUM_VC; k++) begin
          if (outvc_i[i][v] == vc_id_t'(k)) cok = credit_ok_i[p][k];
        end
        elig[i][v] = req_i[i][v] & ~in_mask_i[i] & ~out_mask_i[p] &
                     (active_i[i][v] ? cok : has_free_i[p]);
      end
    end
  end

  // ---------------------------------------------------------------- stage 1: per input
  logic   [NPORTS-1:0]             in_vld;     // the input has a competing VC
  logic   [NPORTS-1:0][VCI_W-1:0]  in_idx;     // which one
  port_t  [NPORTS-1:0]             in_port;    // the output port it wants
  logic   [NPORTS-1:0]             in_act;     // it already holds an output VC
  logic   [NPORTS-1:0][NUM_VC-1:0] unused_in_oh;

  for (genvar i = 0; i < NPORTS; i++) begin : g_in
    rr_arbiter #(.N(NUM_VC)) u_arb (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .req_i     (elig[i]),
      .mask_i    ('0),
      .hold_i    (1'b0),
      .update_i  (in_gnt_o[i]),     // move only when the VC won its output too
      .gnt_o     (unused_in_oh[i]),
      .gnt_idx_o (in_idx[i]),
      .gnt_vld_o (in_vld[i])
    );
    assign in_port[i] = port_i[i][in_idx[i]];
    assign in_act[i]  = active_i[i][in_idx[i]];
  end

  // ---------------------------------------------------------------- stage 2: per output
  logic [NPORTS-1:0][NPORTS-1:0] oreq;   // [o][i]: input i wants output o
  logic [NPORTS-1:0][NPORTS-1:0] ognt;   // [o][i]: one-hot winner of output o
  logic [NPORTS-1:0][PI_W-1:0]   owin;   // index of the winner of output o

  for (genvar o = 0; o < NPORTS; o++) begin : g_out
    for (genvar i = 0; i < NPORTS; i++) begin : g_req
      assign oreq[o][i] = in_vld[i] & (in_port[i] == port_t'(o));
    end
    rr_arbiter #(.N(NPORTS)) u_arb (
      .clk_i     (clk_i),
      .rst_ni    (rst_ni),
      .req_i     (oreq[o]),
      .mask_i    ('0),
      .hold_i    (1'b0),
      .update_i  (out_gnt_o[o]),
      .gnt_o     (ognt[o]),
      .gnt_idx_o (owin[o]),
      .gnt_vld_o (out_gnt_o[o])
    );
  end

  // ---------------------------------------------------------------- results
  always_comb begin
    for (int i = 0; i < NPORTS; i++) begin
      in_gnt_o[i] = 1'b0;
      for (int o = 0; o < NPORTS; o++) in_gnt_o[i] = in_gnt_o[i] | ognt[o][i];
      in_vc_o[i]    = vc_id_t'(in_idx[i]);
      in_outvc_o[i] = in_act[i] ? outvc_i[i][in_idx[i]] : alloc_vc_i[in_port[i]];
    end
  end

  always_comb begin
    for (int o = 0; o < NPORTS; o++) begin
      out_alloc_o[o] = out_gnt_o[o] & ~in_act[owin[o]];
      out_vc_o[o]    = in_outvc_o[owin[o]];
    end
  end

endmodule
