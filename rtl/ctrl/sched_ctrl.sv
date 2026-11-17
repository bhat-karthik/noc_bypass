// =============================================================================
// File      rtl/ctrl/sched_ctrl.sv                                 Tracker: F56
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Design D's controller (D18, D56), one per mesh beside tile
//           (0, 0): takes the mesh-wide demand sums C_L and C_G from the sum
//           tree once per epoch and sets the next k_L: share bin, policy
//           table, clamp, damping; the plan goes out through tile (0, 0).
// Decisions D18 split policy, D55 controller timing, D56 controller
//           arithmetic
// Tests     U13 (tb/unit/tb_sched_ctrl.sv), U14 (tb/unit/tb_ctrl_chain.sv);
//           CT01-CT05 (tb_mesh_d)
// -----------------------------------------------------------------------------
// Timing (cycles after the epoch tick; D55): bit k of both sums arrives on
// sum_i in cycle 2N + k (sum_i[0] local, sum_i[1] cross-region, least
// significant bit first) and is shifted in; in cycle ct_sums(N) = 2N + W
// (W = dm_sum_w(N)) both are whole. The share bin then takes five cycles of
// restoring division, and the new k_L is registered at the end of cycle
// ct_sums(N) + 6 (plan_o): cycle 42 after the tick on 8x8, 60 on 16x16.
// Through the tiles it reaches the last one 2N - 1 cycles later.
// Arithmetic (D18, D56), with S = C_L + C_G:
//   S = 0        no demand: k_L stays as it is
//   C_G = 0      q = 16: k_L = K_HI
//   otherwise    q = floor(16 C_L / S), 0..15: k_L = the policy table's
//                entry q (sched_pkg::kl_table, built at elaboration for
//                PROP and for SQRT; policy_i picks one)
//   clamp        to [K_LO, K_HI] (D18: 1 and 15 on 8x8, 1 and 47 on 16x16)
//   damping      damp_i = 1: the mean of the old and the new k_L, rounded
//                toward the new one, so k_L reaches any target within four
//                epochs and never overshoots it; the result is clamped
//                once more (only a K_INIT outside the clamp needs it)
// policy_i and damp_i are configuration inputs (tie-offs in a chip): one
// build runs every variant. From reset k_L = K_INIT (K_FIXED).
// =============================================================================
`timescale 1ns/1ps

module sched_ctrl
  import sched_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int REGION_N = 4,
  parameter int K_INIT   = 4,              // k_L from reset
  parameter int K_LO     = K_MIN,          // clamp (D18)
  parameter int K_HI     = k_max(MESH_N)
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        tick_i,              // first cycle of an epoch
  input  logic [1:0]  sum_i,               // from tile (0, 0)'s ctrl_node
  input  logic        policy_i,            // POL_PROP (0) or POL_SQRT (1)
  input  logic        damp_i,              // damping on
  output kl_t         plan_o               // the plan, into tile (0, 0)
);

  localparam int SW = dm_sum_w(MESH_N);    // bits of each sum
  localparam int CS = ct_sums(MESH_N);     // cycle in which both sums are whole
  localparam int C0 = 2 * MESH_N;          // cycle in which bit 0 arrives

  logic [CT_W-1:0] ct_q;
  logic [SW-1:0]   sl_q, sg_q;             // the sums, shifted in
  logic [SW+1:0]   r_q, d_q;               // division: remainder, divisor S
  logic [4:0]      q_q;                    // share bin, 0..16
  kl_t             k_q;

  // ---------------------------------------------------------------- policy tables
  kl_t [15:0] tab_p, tab_s;
  for (genvar i = 0; i < 16; i++) begin : g_tab
    localparam int KP = kl_table(POL_PROP, MESH_N, REGION_N, K_LO, K_HI, i);
    localparam int KS = kl_table(POL_SQRT, MESH_N, REGION_N, K_LO, K_HI, i);
    assign tab_p[i] = kl_t'(KP);
    assign tab_s[i] = kl_t'(KS);
  end

  // ---------------------------------------------------------------- next k_L
  logic          shift, load, step0, stepk, upd;
  logic [SW+1:0] r2;
  kl_t           kt, kc, kd, kn;
  logic [6:0]    ks;

  assign shift = (ct_q >= CT_W'(C0)) && (ct_q < CT_W'(CS)) && !tick_i;
  assign load  = (ct_q == CT_W'(CS)) && !tick_i;
  assign step0 = (ct_q == CT_W'(CS + 1)) && !tick_i;
  assign stepk = (ct_q >= CT_W'(CS + 2)) && (ct_q <= CT_W'(CS + 5)) && !tick_i;
  assign upd   = (ct_q == CT_W'(CS + 6)) && !tick_i;

  // the doubled remainder for the steps of weight 8, 4, 2, 1
  assign r2 = {r_q[SW:0], 1'b0};

  always_comb begin
    // policy table, or the two corners
    if (d_q == '0)
      kt = k_q;
    else if (q_q[4])
      kt = kl_t'(K_HI);
    else if (policy_i)
      kt = tab_s[q_q[3:0]];
    else
      kt = tab_p[q_q[3:0]];
    // clamp
    if (kt < kl_t'(K_LO))
      kc = kl_t'(K_LO);
    else if (kt > kl_t'(K_HI))
      kc = kl_t'(K_HI);
    else
      kc = kt;
    // damping: mean of old and new, rounded toward the new
    ks = 7'(k_q) + 7'(kc) + 7'(kc > k_q);
    kd = damp_i ? ks[KL_W:1] : kc;
    // clamped once more: matters only for a K_INIT outside the clamp
    if (kd < kl_t'(K_LO))
      kn = kl_t'(K_LO);
    else if (kd > kl_t'(K_HI))
      kn = kl_t'(K_HI);
    else
      kn = kd;
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ct_q <= '1;
      sl_q <= '0;
      sg_q <= '0;
      r_q  <= '0;
      d_q  <= '0;
      q_q  <= '0;
      k_q  <= kl_t'(K_INIT);
    end else begin
      ct_q <= tick_i ? CT_W'(1) : (ct_q == '1) ? ct_q : ct_q + 1'b1;
      if (shift) begin
        sl_q <= {sum_i[0], sl_q[SW-1:1]};
        sg_q <= {sum_i[1], sg_q[SW-1:1]};
      end
      if (load) begin
        r_q <= {2'b00, sl_q};
        d_q <= {2'b00, sl_q} + {2'b00, sg_q};
        q_q <= '0;
      end
      if (step0) begin                      // weight 16: C_L = S, so C_G = 0
        if (r_q >= d_q) begin
          r_q    <= r_q - d_q;
          q_q[4] <= 1'b1;
        end
      end
      if (stepk) begin                      // weights 8, 4, 2, 1, into q_q[3:0]
        if (r2 >= d_q) begin
          r_q      <= r2 - d_q;
          q_q[3:0] <= {q_q[2:0], 1'b1};
        end else begin
          r_q      <= r2;
          q_q[3:0] <= {q_q[2:0], 1'b0};
        end
      end
      if (upd) k_q <= kn;
    end
  end

  assign plan_o = k_q;

  logic unused_ks;
  assign unused_ks = ks[0];

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (K_LO < 0 || K_HI < K_LO || K_HI >= (1 << KL_W))
      $fatal(1, "%m: clamp [%0d, %0d] outside 0..%0d", K_LO, K_HI, (1 << KL_W) - 1);
    if (CS + 6 + 2 * MESH_N > A_DELAY - 1)
      $fatal(1, "%m: the plan would reach the last tile after the sample (cycle %0d)", A_DELAY - 1);
  end
  // synopsys translate_on
`endif

endmodule
