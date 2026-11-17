// =============================================================================
// File      rtl/top/tile.sv                                        Tracker: F21
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   One mesh tile: the router and its network interface. The router's
//           Local port (P_L) connects to the NI; the four mesh ports and the
//           NI's core side are the tile's ports. Design B adds the BuffeRS
//           timetable [1, Fig. 10], which tells the router when it may send a
//           bypass burst and to whom, the lookahead links (B4), and the
//           reservation decisions and cancel links (B5). Design C has the
//           two-level timetable instead (B6): a local and a global one,
//           in frames.
// Decisions D14 network interface, D20 one generic tile per design
//           (coordinates are ports), D31 NI structure, D44 design-B tile,
//           D12 and D48 reservation, D15 D16 D19 D50 two-level timetable,
//           D54 D55 demand and controller network (design D)
// Tests     N01-N08 (tb/mesh); BP01-BP10 (tb/mesh/tb_mesh_b.sv); TL01-TL06
//           (tb/mesh/tb_mesh_c.sv, tb_mesh_cb.sv); CT01-CT05
//           (tb/mesh/tb_mesh_d.sv)
// -----------------------------------------------------------------------------
// Mesh port index = router port: 0 North, 1 East, 2 South, 3 West.
// Design B timetable (one global timetable, TT_N = MESH_N, enabled every
// cycle): slot_counter -> slot_table -> group_to_router. In the first cycle
// of a subslot in which the router has a partner to send to, tx_go asks the
// router to send; its bypass source skips the turn if a cancel token came
// for it. A second group_to_router looks one subslot ahead (slot_counter's
// nxt_go and nxt_sub): when a burst may come to this router in the next
// subslot, the NI's reservation controller decides now whether to reserve
// space for it or to cancel it (D12, D38). The subslot length is
// t_sub(MESH_N, B_MAX, T_MODE): TIGHT by default (D10, D39). Designs B-D
// size the NI FIFO with ej_depth_b (D48), the default of EJ_DEPTH.
// Design C timetable (D15, D16): phase_sequencer runs a local slot counter
// (side REGION_N) and a global one (side MESH_N) in frames of k_L local
// slots and one global slot. Each has its own slot table (the local one
// with the router's group in its region) and two group_to_router copies
// (this subslot, next subslot; the local ones in region coordinates); the
// running timetable's outputs drive tx_go and the NI's decision as in
// design B, and dec_loc tells the NI which subslot lengths apply. plan_i
// is the k_L sampled after each epoch tick (design C: K_FIXED, tied by the
// mesh; TL04 forces others). With REGION_N = MESH_N only the global
// timetable runs and the tile is design B's (D50). Design D (B7, D54, D55)
// adds demand_counter, which counts the router's occupied input VCs from
// its links (local or cross-region), and ctrl_node, which adds the latched
// counts bit-serially toward tile (0, 0) and forwards the plan sched_ctrl
// sets there: the sequencer samples the forwarded plan instead of plan_i.
// =============================================================================
`timescale 1ns/1ps

module tile
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int NUM_VC   = 4,
  parameter int VC_DEPTH = 5,
  parameter int ALGO     = RT_XY,      // design A routes XY (D06)
  parameter int DESIGN   = DESIGN_A,
  parameter int B_MAX    = 10,         // flits per bypass burst (designs B-D)
  parameter int T_MODE   = TM_TIGHT,   // subslot rule (D10)
  parameter int REGION_N = 4,          // region side (designs C, D, D15)
  parameter int K_INIT   = 4,          // k_L from reset (designs C, D)
  parameter int EPOCH    = 1 << 15,    // epoch, cycles (designs C, D, D19)
  // NI ejection FIFO: 25 in design A (D31), ej_depth_b in designs B-D (D48)
  parameter int EJ_DEPTH = (DESIGN == DESIGN_A) ? 25 : ej_depth_b(NUM_VC, VC_DEPTH, MESH_N, B_MAX, T_MODE),
  parameter int EFW      = $clog2(EJ_DEPTH + 1)   // derived, do not set
) (
  input  logic             clk_i,
  // The slot table loads while reset is held (D41); everything else resets
  // asynchronously, so design B uses rst_ni both ways
  /* verilator lint_off SYNCASYNCNET */
  input  logic             rst_ni,
  /* verilator lint_on SYNCASYNCNET */
  input  coord_t           cur_x_i,
  input  coord_t           cur_y_i,
  // k_L for the frames after the next plan sample (design C); design D: the
  // plan forwarded from the upstream tile (or from sched_ctrl at (0, 0))
  input  kl_t              plan_i,
  // design D's controller network (B7): the demand sums, bit-serial, from
  // the east and south neighbours and toward tile (0, 0); the plan to the
  // downstream tiles; the epoch tick (tile (0, 0)'s drives sched_ctrl).
  // Zero in designs A-C.
  input  logic [1:0]       sum_e_i,
  input  logic [1:0]       sum_s_i,
  output logic [1:0]       sum_o,
  output kl_t              plan_o,
  output logic             tick_o,
  // mesh links, index = port (N, E, S, W)
  input  flit_t   [3:0]    in_flit_i,
  output credit_t [3:0]    in_credit_o,
  output flit_t   [3:0]    out_flit_o,
  input  credit_t [3:0]    out_credit_i,
  // lookahead and cancel links (designs B-D; zero in design A)
  input  la_t     [3:0]    in_la_i,
  output la_t     [3:0]    out_la_o,
  input  cx_t     [3:0]    in_cx_i,
  output cx_t     [3:0]    out_cx_o,
  // core side of the NI
  input  logic             inj_valid_i,
  output logic             inj_ready_o,
  input  flit_t            inj_flit_i,
  output logic             ej_valid_o,
  input  logic             ej_ready_i,
  output flit_t            ej_flit_o,
  output logic [EFW-1:0]   ej_free_o,
  output flit_t            ej_mon_o
);

  flit_t   [NPORTS-1:0] r_in_flit, r_out_flit;
  credit_t [NPORTS-1:0] r_in_cred, r_out_cred;

  assign r_in_flit[3:0]  = in_flit_i;
  assign in_credit_o     = r_in_cred[3:0];
  assign out_flit_o      = r_out_flit[3:0];
  assign r_out_cred[3:0] = out_credit_i;

  // ---------------------------------------------------------------- timetable
  logic     tx_go, tx_mode;
  node_id_t tx_dst;
  // reservation decision for the next subslot (to the NI)
  logic     dec, dec_s0, dec_loc, dec_mode;
  node_id_t dec_src;
  hop_t     dec_hops;

  if (DESIGN == DESIGN_B) begin : g_tt
    logic              sub_go, act, tx_vld, nxt_go;
    logic              unused_setup, unused_last;
    sub_t              sub, nxt_sub;
    slot_t             slot;
    logic [TCYC_W-1:0] unused_cyc;
    tt_grp_t           tgt;
    coord_t            tx_x, tx_y, rx_x, rx_y, nrx_x, nrx_y, ntx_x, ntx_y;
    logic              rx_vld, rx_mode, nrx_vld, nrx_mode, ntx_vld, ntx_mode;
    hop_t              tx_hops, rx_hops, nrx_hops, ntx_hops;

    slot_counter #(
      .TT_N    (MESH_N),
      .T_SUB   (t_sub(MESH_N, B_MAX, T_MODE)),
      .S_SETUP (s_setup(MESH_N))
    ) u_cnt (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .en_i        (1'b1),
      .setup_o     (unused_setup),
      .sub_o       (sub),
      .cyc_o       (unused_cyc),
      .slot_o      (slot),
      .sub_go_o    (sub_go),
      .slot_last_o (unused_last),
      .nxt_go_o    (nxt_go),
      .nxt_sub_o   (nxt_sub)
    );

    slot_table #(.TT_N(MESH_N)) u_tab (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .grp_i  (tt_grp_t'(grp_of(int'(cur_x_i), int'(cur_y_i), MESH_N))),
      .slot_i (slot),
      .act_o  (act),
      .tgt_o  (tgt)
    );

    // this subslot: where to send
    group_to_router #(.MESH_N(MESH_N), .TT_N(MESH_N)) u_g2r (
      .x_i       (cur_x_i),
      .y_i       (cur_y_i),
      .act_i     (act),
      .tgt_i     (tgt),
      .sub_i     (sub),
      .tx_vld_o  (tx_vld),
      .tx_x_o    (tx_x),
      .tx_y_o    (tx_y),
      .tx_mode_o (tx_mode),
      .tx_hops_o (tx_hops),
      .rx_vld_o  (rx_vld),
      .rx_x_o    (rx_x),
      .rx_y_o    (rx_y),
      .rx_mode_o (rx_mode),
      .rx_hops_o (rx_hops)
    );

    // the next subslot (same slot; in the setup window, subslot 0 of this
    // slot): who may send here, decided one subslot ahead
    group_to_router #(.MESH_N(MESH_N), .TT_N(MESH_N)) u_g2r_nxt (
      .x_i       (cur_x_i),
      .y_i       (cur_y_i),
      .act_i     (act),
      .tgt_i     (tgt),
      .sub_i     (nxt_sub),
      .tx_vld_o  (ntx_vld),
      .tx_x_o    (ntx_x),
      .tx_y_o    (ntx_y),
      .tx_mode_o (ntx_mode),
      .tx_hops_o (ntx_hops),
      .rx_vld_o  (nrx_vld),
      .rx_x_o    (nrx_x),
      .rx_y_o    (nrx_y),
      .rx_mode_o (nrx_mode),
      .rx_hops_o (nrx_hops)
    );

    assign tx_go    = sub_go & tx_vld;
    assign tx_dst   = xy_to_id(tx_x, tx_y, MESH_N);
    assign dec      = nxt_go & nrx_vld;
    assign dec_s0   = (nxt_sub == '0);
    assign dec_loc  = 1'b0;
    assign dec_src  = xy_to_id(nrx_x, nrx_y, MESH_N);
    assign dec_mode = nrx_mode;
    assign dec_hops = nrx_hops;

    logic unused_tt;
    assign unused_tt = ^{unused_setup, unused_last, unused_cyc, tx_hops, rx_vld, rx_x, rx_y, rx_mode, rx_hops,
                         ntx_vld, ntx_x, ntx_y, ntx_mode, ntx_hops, plan_i, sum_e_i, sum_s_i};
    assign sum_o  = '0;
    assign plan_o = '0;
    assign tick_o = 1'b0;
  end else if (DESIGN == DESIGN_C || DESIGN == DESIGN_D) begin : g_tt2
    // ---------------------------------------------- two-level timetable (D15, D16)
    logic    loc, sub_go, nxt_go, act_l, act_g, tick;
    sub_t    sub_l, sub_g, nsub_l, nsub_g;
    slot_t   slot_l, slot_g;
    tt_grp_t tgt_l, tgt_g;
    kl_t     unused_kl;
    kl_t     plan_seq;                 // the plan the sequencer samples
    seq_st_t seq_st;                   // the sequencer's state (testbench lockstep check)
    // partner logic, index: 0 local this subslot, 1 local next subslot,
    // 2 global this subslot, 3 global next subslot
    logic   [3:0] tv, rv, tm, rm;
    coord_t [3:0] tx, ty, rx, ry;
    hop_t   [3:0] th, rh;

    phase_sequencer #(
      .MESH_N   (MESH_N),
      .REGION_N (REGION_N),
      .T_SUB_G  (t_sub(MESH_N, B_MAX, T_MODE)),
      .T_SUB_L  (t_sub(REGION_N, B_MAX, T_MODE)),
      .K_INIT   (K_INIT),
      .EPOCH    (EPOCH)
    ) u_seq (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .plan_i      (plan_seq),
      .loc_o       (loc),
      .slot_l_o    (slot_l),
      .sub_l_o     (sub_l),
      .nxt_sub_l_o (nsub_l),
      .slot_g_o    (slot_g),
      .sub_g_o     (sub_g),
      .nxt_sub_g_o (nsub_g),
      .sub_go_o    (sub_go),
      .nxt_go_o    (nxt_go),
      .kl_o        (unused_kl),
      .tick_o      (tick),
      .st_o        (seq_st)
    );

    if (DESIGN == DESIGN_D) begin : g_ctrl
      // ---------------------------------------------- demand and controller network (B7)
      logic [DM_W-1:0] cl, cg;

      demand_counter #(
        .MESH_N   (MESH_N),
        .REGION_N (REGION_N),
        .NUM_VC   (NUM_VC)
      ) u_dm (
        .clk_i     (clk_i),
        .rst_ni    (rst_ni),
        .cur_x_i   (cur_x_i),
        .cur_y_i   (cur_y_i),
        .tick_i    (tick),
        .in_flit_i (r_in_flit),
        .in_cred_i (r_in_cred),
        .cl_o      (cl),
        .cg_o      (cg)
      );

      ctrl_node #(
        .MESH_N (MESH_N),
        .K_INIT (K_INIT)
      ) u_cn (
        .clk_i   (clk_i),
        .rst_ni  (rst_ni),
        .cur_x_i (cur_x_i),
        .cur_y_i (cur_y_i),
        .tick_i  (tick),
        .cl_i    (cl),
        .cg_i    (cg),
        .sum_e_i (sum_e_i),
        .sum_s_i (sum_s_i),
        .sum_o   (sum_o),
        .plan_i  (plan_i),
        .plan_o  (plan_seq)
      );

      assign plan_o = plan_seq;
    end else begin : g_fixed
      // design C: the mesh's plan input straight into the sequencer
      assign plan_seq = plan_i;
      assign sum_o    = '0;
      assign plan_o   = '0;
      logic unused_sum;
      assign unused_sum = ^{sum_e_i, sum_s_i};
    end
    assign tick_o = tick;

    // the local table holds the router's group in its region's timetable
    slot_table #(.TT_N(REGION_N)) u_tab_l (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .grp_i  (tt_grp_t'(grp_of(int'(cur_x_i), int'(cur_y_i), REGION_N))),
      .slot_i (slot_l),
      .act_o  (act_l),
      .tgt_o  (tgt_l)
    );

    slot_table #(.TT_N(MESH_N)) u_tab_g (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .grp_i  (tt_grp_t'(grp_of(int'(cur_x_i), int'(cur_y_i), MESH_N))),
      .slot_i (slot_g),
      .act_o  (act_g),
      .tgt_o  (tgt_g)
    );

    group_to_router #(.MESH_N(MESH_N), .TT_N(REGION_N)) u_g2r_l (
      .x_i (cur_x_i), .y_i (cur_y_i), .act_i (act_l), .tgt_i (tgt_l), .sub_i (sub_l),
      .tx_vld_o (tv[0]), .tx_x_o (tx[0]), .tx_y_o (ty[0]), .tx_mode_o (tm[0]), .tx_hops_o (th[0]),
      .rx_vld_o (rv[0]), .rx_x_o (rx[0]), .rx_y_o (ry[0]), .rx_mode_o (rm[0]), .rx_hops_o (rh[0])
    );

    group_to_router #(.MESH_N(MESH_N), .TT_N(REGION_N)) u_g2r_ln (
      .x_i (cur_x_i), .y_i (cur_y_i), .act_i (act_l), .tgt_i (tgt_l), .sub_i (nsub_l),
      .tx_vld_o (tv[1]), .tx_x_o (tx[1]), .tx_y_o (ty[1]), .tx_mode_o (tm[1]), .tx_hops_o (th[1]),
      .rx_vld_o (rv[1]), .rx_x_o (rx[1]), .rx_y_o (ry[1]), .rx_mode_o (rm[1]), .rx_hops_o (rh[1])
    );

    group_to_router #(.MESH_N(MESH_N), .TT_N(MESH_N)) u_g2r_g (
      .x_i (cur_x_i), .y_i (cur_y_i), .act_i (act_g), .tgt_i (tgt_g), .sub_i (sub_g),
      .tx_vld_o (tv[2]), .tx_x_o (tx[2]), .tx_y_o (ty[2]), .tx_mode_o (tm[2]), .tx_hops_o (th[2]),
      .rx_vld_o (rv[2]), .rx_x_o (rx[2]), .rx_y_o (ry[2]), .rx_mode_o (rm[2]), .rx_hops_o (rh[2])
    );

    group_to_router #(.MESH_N(MESH_N), .TT_N(MESH_N)) u_g2r_gn (
      .x_i (cur_x_i), .y_i (cur_y_i), .act_i (act_g), .tgt_i (tgt_g), .sub_i (nsub_g),
      .tx_vld_o (tv[3]), .tx_x_o (tx[3]), .tx_y_o (ty[3]), .tx_mode_o (tm[3]), .tx_hops_o (th[3]),
      .rx_vld_o (rv[3]), .rx_x_o (rx[3]), .rx_y_o (ry[3]), .rx_mode_o (rm[3]), .rx_hops_o (rh[3])
    );

    // the running timetable's partners: local (0, 1) or global (2, 3)
    logic [1:0] i_now, i_nxt;
    assign i_now    = loc ? 2'd0 : 2'd2;
    assign i_nxt    = loc ? 2'd1 : 2'd3;
    assign tx_go    = sub_go & tv[i_now];
    assign tx_dst   = xy_to_id(tx[i_now], ty[i_now], MESH_N);
    assign tx_mode  = tm[i_now];
    assign dec      = nxt_go & rv[i_nxt];
    assign dec_s0   = loc ? (nsub_l == '0) : (nsub_g == '0);
    assign dec_loc  = loc;
    assign dec_src  = xy_to_id(rx[i_nxt], ry[i_nxt], MESH_N);
    assign dec_mode = rm[i_nxt];
    assign dec_hops = rh[i_nxt];

    logic unused_tt2;
    assign unused_tt2 = ^{unused_kl, seq_st, th, rv[0], rv[2], rx[0], rx[2], ry[0], ry[2],
                          rm[0], rm[2], rh[0], rh[2], tv[1], tv[3], tx[1], tx[3], ty[1], ty[3], tm[1], tm[3]};
  end else begin : g_no_tt
    assign tx_go    = 1'b0;
    assign tx_dst   = '0;
    assign tx_mode  = 1'b0;
    assign dec      = 1'b0;
    assign dec_s0   = 1'b0;
    assign dec_loc  = 1'b0;
    assign dec_src  = '0;
    assign dec_mode = 1'b0;
    assign dec_hops = '0;
    assign sum_o    = '0;
    assign plan_o   = '0;
    assign tick_o   = 1'b0;
    logic unused_plan;
    assign unused_plan = ^{plan_i, sum_e_i, sum_s_i};
  end

  cx_t cx_inj;          // the NI's cancel token, into the router's cancel relay
  la_t la_loc;          // the router's lookahead of a burst for the NI

  router #(
    .MESH_N   (MESH_N),
    .NUM_VC   (NUM_VC),
    .VC_DEPTH (VC_DEPTH),
    .ALGO     (ALGO),
    .DESIGN   (DESIGN),
    .B_MAX    (B_MAX)
  ) u_router (
    .clk_i        (clk_i),
    .rst_ni       (rst_ni),
    .cur_x_i      (cur_x_i),
    .cur_y_i      (cur_y_i),
    .in_flit_i    (r_in_flit),
    .in_credit_o  (r_in_cred),
    .out_flit_o   (r_out_flit),
    .out_credit_i (r_out_cred),
    .in_la_i      (in_la_i),
    .out_la_o     (out_la_o),
    .in_cx_i      (in_cx_i),
    .out_cx_o     (out_cx_o),
    .cx_inj_i     (cx_inj),
    .la_loc_o     (la_loc),
    .tx_go_i      (tx_go),
    .tx_dst_i     (tx_dst),
    .tx_mode_i    (tx_mode)
  );

  ni #(
    .NUM_VC   (NUM_VC),
    .VC_DEPTH (VC_DEPTH),
    .EJ_DEPTH (EJ_DEPTH),
    .BYPASS   ((DESIGN != DESIGN_A) ? 1 : 0),
    .B_MAX    (B_MAX),
    .GAP0     (s_setup(MESH_N)),
    .GAPK     (t_sub(MESH_N, B_MAX, T_MODE)),
    .GAP0_L   (s_setup(REGION_N)),
    .GAPK_L   (t_sub(REGION_N, B_MAX, T_MODE))
  ) u_ni (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .inj_valid_i (inj_valid_i),
    .inj_ready_o (inj_ready_o),
    .inj_flit_i  (inj_flit_i),
    .link_o      (r_in_flit[P_L]),
    .credit_i    (r_in_cred[P_L]),
    .ej_link_i   (r_out_flit[P_L]),
    .credit_o    (r_out_cred[P_L]),
    .ej_valid_o  (ej_valid_o),
    .ej_ready_i  (ej_ready_i),
    .ej_flit_o   (ej_flit_o),
    .ej_free_o   (ej_free_o),
    .ej_mon_o    (ej_mon_o),
    .dec_i       (dec),
    .dec_s0_i    (dec_s0),
    .dec_loc_i   (dec_loc),
    .dec_src_i   (dec_src),
    .dec_mode_i  (dec_mode),
    .dec_hops_i  (dec_hops),
    .la_loc_i    (la_loc),
    .cx_o        (cx_inj)
  );

endmodule
