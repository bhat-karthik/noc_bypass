// =============================================================================
// File      rtl/top/mesh_top.sv                                    Tracker: F22
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The MESH_N x MESH_N mesh: one tile per node, flit, credit and
//           lookahead links between neighbours, and the edge tie-offs.
// Decisions D14 network interface, D20 generic tile, D31 NI structure,
//           D44 design-B tile and lookahead links, D12 cancel links, D15 D16
//           D19 two-level timetable
// Tests     N01-N08 (tb/mesh); BP01-BP10 (tb/mesh/tb_mesh_b.sv); TL01-TL06
//           (tb/mesh/tb_mesh_c.sv, tb_mesh_cb.sv); CT01-CT05
//           (tb/mesh/tb_mesh_d.sv)
// -----------------------------------------------------------------------------
// Node id = y*MESH_N + x (noc_pkg): x grows east, y grows south. The core
// ports are arrays indexed by node id.
// Per-node arrays are unpacked (D36). A packed array is one vector: on 8x8
// the flit links were a 34,304-bit vector that Verilator rebuilt whenever
// one tile's output changed, and an event-driven simulator (xsim, Riviera)
// wakes every reader of a vector when any part of it changes.
// Links: tile (x, y) East output -> tile (x+1, y) West input, and so on; each
// credit runs the other way on the same link. A port at the mesh edge sees
// no flits and no credits; routing never uses it, and a simulation check
// stops the run if a flit ever leaves the mesh.
// Lookahead links (B4) and cancel links (B5) run beside the flit links, in
// the same direction (a cancel token travels against its burst, so it uses
// the cancel links of the opposite direction); in design A every tile
// drives them with zero. Design D's controller links (B7, D55) form a tree:
// every tile sends its 2-bit serial sums west (column 0: north) and takes
// them from the east (column 0: also from the south); tile (0, 0)'s go to
// sched_ctrl, whose plan enters tile (0, 0) and travels back the same tree
// (west to east along each row, north to south down column 0).
// EJ_DEPTH: design A 25 (D31); designs B-D sched_pkg::ej_depth_b (D48), the
// parameter's default.
// Design C (B6): every tile runs the two-level timetable in REGION_N x
// REGION_N regions (default 4) with k_L = K_FIXED from reset (default
// sched_pkg::k_fixed: 4 on 8x8, 14 on 16x16) and epochs of EPOCH cycles
// (default sched_pkg::epoch_len: 2^15 on 8x8, 2^18 on 16x16). plan_i goes to
// every tile; it is sampled A_DELAY - 1 cycles after each epoch tick, so a
// design-C mesh ties it to K_FIXED (the testbench drives it: TL04 forces
// other plans). REGION_N = MESH_N gives design B's schedule (D50). Designs
// A and B ignore REGION_N, K_FIXED, EPOCH and plan_i.
// Design D (B7): the same tiles with demand counters and the controller
// network; sched_ctrl (D18, D56) sets k_L every epoch from the mesh-wide
// demand, starting from K_FIXED; plan_i is not used, ctrl_pol_i and
// ctrl_damp_i choose the policy and damping. Designs A-C ignore those two.
// =============================================================================
`timescale 1ns/1ps

module mesh_top
  import noc_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int NUM_VC   = 4,
  parameter int VC_DEPTH = 5,
  parameter int ALGO     = RT_XY,
  parameter int DESIGN   = DESIGN_A,
  parameter int B_MAX    = 10,                     // flits per bypass burst (designs B-D)
  parameter int T_MODE   = sched_pkg::TM_TIGHT,    // subslot rule (D10)
  parameter int REGION_N = (MESH_N >= 4) ? 4 : MESH_N,   // region side (designs C, D)
  parameter int K_FIXED  = sched_pkg::k_fixed(MESH_N, REGION_N, B_MAX, T_MODE),  // k_L from reset
  parameter int EPOCH    = sched_pkg::epoch_len(MESH_N, REGION_N, B_MAX, T_MODE), // epoch, cycles
  parameter int K_LO     = sched_pkg::K_MIN,       // design D: clamp of k_L (D18; E7, E8 vary it)
  parameter int K_HI     = sched_pkg::k_max(MESH_N),
  // NI ejection FIFO: 25 in design A (D31), ej_depth_b in designs B-D (D48)
  parameter int EJ_DEPTH = (DESIGN == DESIGN_A) ? 25 : sched_pkg::ej_depth_b(NUM_VC, VC_DEPTH, MESH_N, B_MAX, T_MODE),
  parameter int NN       = MESH_N * MESH_N,        // derived, do not set
  parameter int EFW      = $clog2(EJ_DEPTH + 1)    // derived, do not set
) (
  input  logic                clk_i,
  // (design D: sched_ctrl, small enough for Verilator to inline here, uses
  // rst_ni as an asynchronous reset, and the simulation-only edge checks
  // below read it at clock edges)
  /* verilator lint_off SYNCASYNCNET */
  input  logic                rst_ni,
  /* verilator lint_on SYNCASYNCNET */
  // k_L for the frames after the next plan sample (design C; design D sets
  // its own)
  input  sched_pkg::kl_t      plan_i,
  // design D's controller configuration (D56): POL_PROP (0) or POL_SQRT
  // (1), and damping on; tie-offs in a chip
  input  logic                ctrl_pol_i,
  input  logic                ctrl_damp_i,
  // core side of every NI, index = node id
  input  logic                inj_valid_i [NN],
  output logic                inj_ready_o [NN],
  input  flit_t               inj_flit_i  [NN],
  output logic                ej_valid_o  [NN],
  input  logic                ej_ready_i  [NN],
  output flit_t               ej_flit_o   [NN],
  output logic  [EFW-1:0]     ej_free_o   [NN],
  output flit_t               ej_mon_o    [NN]
);

  // per tile (unpacked) and port N, E, S, W (packed, as the tile's ports)
  flit_t   [3:0] t_in_flit [NN], t_out_flit [NN];
  credit_t [3:0] t_in_cred [NN], t_out_cred [NN];
  la_t     [3:0] t_in_la   [NN], t_out_la   [NN];
  cx_t     [3:0] t_in_cx   [NN], t_out_cx   [NN];
  // design D's controller network (B7, D55): sums toward (0, 0), plan away
  // (in designs A-C the tiles drive zeros that nothing reads; in design D
  // only tile (0, 0)'s tick is read and the last tile's plan_o is not)
  /* verilator lint_off UNUSEDSIGNAL */
  logic    [1:0] t_sum_e [NN], t_sum_s [NN], t_sum_o [NN];
  sched_pkg::kl_t t_plan_i [NN], t_plan_o [NN];
  logic          t_tick  [NN];
  /* verilator lint_on UNUSEDSIGNAL */
  sched_pkg::kl_t c_plan;              // sched_ctrl's plan, into tile (0, 0)

  for (genvar y = 0; y < MESH_N; y++) begin : g_row
    for (genvar x = 0; x < MESH_N; x++) begin : g_col
      localparam int ID = y * MESH_N + x;

      // North: from the South output of (x, y-1)
      if (y > 0) begin : g_n
        assign t_in_flit[ID][P_N]  = t_out_flit[ID - MESH_N][P_S];
        assign t_out_cred[ID][P_N] = t_in_cred[ID - MESH_N][P_S];
        assign t_in_la[ID][P_N]    = t_out_la[ID - MESH_N][P_S];
        assign t_in_cx[ID][P_N]    = t_out_cx[ID - MESH_N][P_S];
      end else begin : g_n_edge
        assign t_in_flit[ID][P_N]  = '0;
        assign t_out_cred[ID][P_N] = '0;
        assign t_in_la[ID][P_N]    = '0;
        assign t_in_cx[ID][P_N]    = '0;
      end
      // East: from the West output of (x+1, y)
      if (x < MESH_N - 1) begin : g_e
        assign t_in_flit[ID][P_E]  = t_out_flit[ID + 1][P_W];
        assign t_out_cred[ID][P_E] = t_in_cred[ID + 1][P_W];
        assign t_in_la[ID][P_E]    = t_out_la[ID + 1][P_W];
        assign t_in_cx[ID][P_E]    = t_out_cx[ID + 1][P_W];
      end else begin : g_e_edge
        assign t_in_flit[ID][P_E]  = '0;
        assign t_out_cred[ID][P_E] = '0;
        assign t_in_la[ID][P_E]    = '0;
        assign t_in_cx[ID][P_E]    = '0;
      end
      // South: from the North output of (x, y+1)
      if (y < MESH_N - 1) begin : g_s
        assign t_in_flit[ID][P_S]  = t_out_flit[ID + MESH_N][P_N];
        assign t_out_cred[ID][P_S] = t_in_cred[ID + MESH_N][P_N];
        assign t_in_la[ID][P_S]    = t_out_la[ID + MESH_N][P_N];
        assign t_in_cx[ID][P_S]    = t_out_cx[ID + MESH_N][P_N];
      end else begin : g_s_edge
        assign t_in_flit[ID][P_S]  = '0;
        assign t_out_cred[ID][P_S] = '0;
        assign t_in_la[ID][P_S]    = '0;
        assign t_in_cx[ID][P_S]    = '0;
      end
      // West: from the East output of (x-1, y)
      if (x > 0) begin : g_w
        assign t_in_flit[ID][P_W]  = t_out_flit[ID - 1][P_E];
        assign t_out_cred[ID][P_W] = t_in_cred[ID - 1][P_E];
        assign t_in_la[ID][P_W]    = t_out_la[ID - 1][P_E];
        assign t_in_cx[ID][P_W]    = t_out_cx[ID - 1][P_E];
      end else begin : g_w_edge
        assign t_in_flit[ID][P_W]  = '0;
        assign t_out_cred[ID][P_W] = '0;
        assign t_in_la[ID][P_W]    = '0;
        assign t_in_cx[ID][P_W]    = '0;
      end

      // design D: each row sums into column 0, column 0 into (0, 0); the
      // plan comes from the west (from the north in column 0; sched_ctrl at
      // (0, 0)). Designs A-C: plan_i to every tile, no sums.
      if (DESIGN == DESIGN_D) begin : g_cn
        if (x < MESH_N - 1) begin : g_se
          assign t_sum_e[ID] = t_sum_o[ID + 1];
        end else begin : g_se_edge
          assign t_sum_e[ID] = 2'b00;
        end
        if (x == 0 && y < MESH_N - 1) begin : g_ss
          assign t_sum_s[ID] = t_sum_o[ID + MESH_N];
        end else begin : g_ss_none
          assign t_sum_s[ID] = 2'b00;
        end
        if (x > 0) begin : g_pw
          assign t_plan_i[ID] = t_plan_o[ID - 1];
        end else if (y > 0) begin : g_pn
          assign t_plan_i[ID] = t_plan_o[ID - MESH_N];
        end else begin : g_pc
          assign t_plan_i[ID] = c_plan;
        end
      end else begin : g_no_cn
        assign t_sum_e[ID]  = 2'b00;
        assign t_sum_s[ID]  = 2'b00;
        assign t_plan_i[ID] = plan_i;
      end

      tile #(
        .MESH_N   (MESH_N),
        .NUM_VC   (NUM_VC),
        .VC_DEPTH (VC_DEPTH),
        .ALGO     (ALGO),
        .DESIGN   (DESIGN),
        .EJ_DEPTH (EJ_DEPTH),
        .B_MAX    (B_MAX),
        .T_MODE   (T_MODE),
        .REGION_N (REGION_N),
        .K_INIT   (K_FIXED),
        .EPOCH    (EPOCH)
      ) u_tile (
        .clk_i        (clk_i),
        .rst_ni       (rst_ni),
        .cur_x_i      (coord_t'(x)),
        .cur_y_i      (coord_t'(y)),
        .plan_i       (t_plan_i[ID]),
        .sum_e_i      (t_sum_e[ID]),
        .sum_s_i      (t_sum_s[ID]),
        .sum_o        (t_sum_o[ID]),
        .plan_o       (t_plan_o[ID]),
        .tick_o       (t_tick[ID]),
        .in_flit_i    (t_in_flit[ID]),
        .in_credit_o  (t_in_cred[ID]),
        .out_flit_o   (t_out_flit[ID]),
        .out_credit_i (t_out_cred[ID]),
        .in_la_i      (t_in_la[ID]),
        .out_la_o     (t_out_la[ID]),
        .in_cx_i      (t_in_cx[ID]),
        .out_cx_o     (t_out_cx[ID]),
        .inj_valid_i  (inj_valid_i[ID]),
        .inj_ready_o  (inj_ready_o[ID]),
        .inj_flit_i   (inj_flit_i[ID]),
        .ej_valid_o   (ej_valid_o[ID]),
        .ej_ready_i   (ej_ready_i[ID]),
        .ej_flit_o    (ej_flit_o[ID]),
        .ej_free_o    (ej_free_o[ID]),
        .ej_mon_o     (ej_mon_o[ID])
      );

`ifndef SYNTHESIS
      // synopsys translate_off
      always_ff @(posedge clk_i) begin
        if (rst_ni) begin
          if (y == 0 && t_out_flit[ID][P_N].vld)
            $fatal(1, "%m: router %0d sent a flit north, off the mesh", ID);
          if (x == MESH_N - 1 && t_out_flit[ID][P_E].vld)
            $fatal(1, "%m: router %0d sent a flit east, off the mesh", ID);
          if (y == MESH_N - 1 && t_out_flit[ID][P_S].vld)
            $fatal(1, "%m: router %0d sent a flit south, off the mesh", ID);
          if (x == 0 && t_out_flit[ID][P_W].vld)
            $fatal(1, "%m: router %0d sent a flit west, off the mesh", ID);
          if ((y == 0 && t_out_la[ID][P_N].vld) || (x == MESH_N - 1 && t_out_la[ID][P_E].vld) ||
              (y == MESH_N - 1 && t_out_la[ID][P_S].vld) || (x == 0 && t_out_la[ID][P_W].vld))
            $fatal(1, "%m: router %0d sent a lookahead off the mesh", ID);
          if ((y == 0 && t_out_cx[ID][P_N].vld) || (x == MESH_N - 1 && t_out_cx[ID][P_E].vld) ||
              (y == MESH_N - 1 && t_out_cx[ID][P_S].vld) || (x == 0 && t_out_cx[ID][P_W].vld))
            $fatal(1, "%m: router %0d sent a cancel token off the mesh", ID);
        end
      end
      // synopsys translate_on
`endif
    end
  end

  // ---------------------------------------------------------------- design D controller
  if (DESIGN == DESIGN_D) begin : g_ctrl
    sched_ctrl #(
      .MESH_N   (MESH_N),
      .REGION_N (REGION_N),
      .K_INIT   (K_FIXED),
      .K_LO     (K_LO),
      .K_HI     (K_HI)
    ) u_ctrl (
      .clk_i    (clk_i),
      .rst_ni   (rst_ni),
      .tick_i   (t_tick[0]),
      .sum_i    (t_sum_o[0]),
      .policy_i (ctrl_pol_i),
      .damp_i   (ctrl_damp_i),
      .plan_o   (c_plan)
    );
    logic unused_d;
    assign unused_d = ^plan_i;
  end else begin : g_no_ctrl
    assign c_plan = '0;
    logic unused_c;
    assign unused_c = ^{ctrl_pol_i, ctrl_damp_i, c_plan};
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (MESH_N < 2 || MESH_N > MAX_MESH_N)
      $fatal(1, "%m: MESH_N = %0d, must be 2..%0d", MESH_N, MAX_MESH_N);
  end
  // synopsys translate_on
`endif

endmodule
