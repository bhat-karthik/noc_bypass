// =============================================================================
// File      rtl/router/router.sv                                   Tracker: F17
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The 5-port router of every design: five input units, the switch
//           allocator, the crossbar and five output units. Three router
//           stages and a 1-cycle link: 4 cycles per hop (Table III of [1]).
//           Designs B-D add the bypass: the source (F41) and the relay (F42),
//           with lookahead links on the four mesh ports, and the cancel
//           relay (F46) with cancel links on the four mesh ports (B5).
// Decisions D03 pipeline, D05 VCs and credits, D06 routing, D26 route chosen
//           once, D27 registers, D28 allocation, D29 bypass hooks,
//           D42 bypass source, D43 bypass relay, D12 cancel tokens
// Tests     R01-R04 (tb/unit/tb_router.sv); BP01-BP10 (tb_mesh_b)
// -----------------------------------------------------------------------------
// A flit on in_flit_i[p] in cycle t
//   t    BW+RC  written into its VC; a head gets its output port
//   t+1  VA+SA  wins the switch (a head also gets an output VC)
//   t+2  ST     crosses the crossbar into the ST register of its output
//   t+3  LT     sits in the link register; out_flit_o shows it in t+4,
//               when the next router writes it (its BW)
// Credits: input unit p returns one credit per flit on in_credit_o[p] the
// cycle after the flit leaves (Garnet's 1-cycle credit link); output unit q
// takes credits from out_credit_i[q].
// The router's coordinates are inputs, so one synthesized tile serves every
// position (D20). Ports without a neighbour are tied to zero by the mesh.
// Bypass (designs B-D, B4): tx_go_i says this router may send a burst to
// tx_dst_i in the subslot that starts now (the tile's timetable). The source
// drives crossbar input 4 and the relay inputs 0-3; both add switch-
// allocation masks (D29 hooks). in_la_i / out_la_o are the lookahead links
// of ports N, E, S, W. Cancel tokens (B5): in_cx_i / out_cx_o are the cancel
// links of ports N, E, S, W; cx_inj_i is the token the router's own NI
// launches; a token for this router cancels its source's next turn.
// la_loc_o gives the NI the lookahead of a burst coming to it. Design A ties
// all of it to zero: same logic as B1-B3.
// Test hold (simulation only): the bench may set bits of tb_hold to block
// switch allocation for those inputs, so packets wait in their VCs and a
// directed bypass test can build a burst. No normal run sets it.
// =============================================================================
`timescale 1ns/1ps

module router
  import noc_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int NUM_VC   = 4,            // 1..4
  parameter int VC_DEPTH = 5,            // flits per VC
  parameter int ALGO     = RT_ADAPTIVE,  // regular routing: RT_XY or RT_ADAPTIVE
  parameter int DESIGN   = DESIGN_A,     // DESIGN_A: no bypass; B-D: bypass datapath
  parameter int B_MAX    = 10            // flits per bypass burst (designs B-D)
) (
  input  logic                    clk_i,
  input  logic                    rst_ni,
  input  coord_t                  cur_x_i,
  input  coord_t                  cur_y_i,
  // input links (index = port: N, E, S, W, L) and the credits sent back on them
  input  flit_t   [NPORTS-1:0]    in_flit_i,
  output credit_t [NPORTS-1:0]    in_credit_o,
  // output links and the credits coming back on them
  output flit_t   [NPORTS-1:0]    out_flit_o,
  input  credit_t [NPORTS-1:0]    out_credit_i,
  // lookahead links of the mesh ports N, E, S, W (designs B-D)
  input  la_t     [3:0]           in_la_i,
  output la_t     [3:0]           out_la_o,
  // cancel links of the mesh ports N, E, S, W, and the NI's own token (B-D)
  input  cx_t     [3:0]           in_cx_i,
  output cx_t     [3:0]           out_cx_o,
  input  cx_t                     cx_inj_i,
  // to the NI: the lookahead of a burst for this router (designs B-D)
  output la_t                     la_loc_o,
  // from the timetable: send a burst to tx_dst_i in the subslot starting now
  input  logic                    tx_go_i,
  input  node_id_t                tx_dst_i,
  input  logic                    tx_mode_i
);

  // input units -> switch allocator
  logic   [NPORTS-1:0][NUM_VC-1:0]   req, active;
  port_t  [NPORTS-1:0][NUM_VC-1:0]   vc_port;
  vc_id_t [NPORTS-1:0][NUM_VC-1:0]   outvc;
  // output units -> switch allocator and route computation
  logic   [NPORTS-1:0]               has_free;
  vc_id_t [NPORTS-1:0]               alloc_vc;
  logic   [NPORTS-1:0][NUM_VC-1:0]   credit_ok;
  logic   [NPORTS-1:0][VC_CNT_W-1:0] free_cnt;
  // switch allocator -> units
  logic   [NPORTS-1:0]               in_gnt, out_gnt, out_alloc;
  vc_id_t [NPORTS-1:0]               in_vc, in_outvc, out_vc;
  // datapath
  flit_t  [NPORTS-1:0]               sa_flit, st_flit;
  port_t  [NPORTS-1:0]               sa_port;
  // bypass hooks (D29) and the bypass read port of the input units
  logic   [NPORTS-1:0]               in_mask, out_mask, byp_en;
  flit_t  [NPORTS-1:0]               byp_flit;
  port_t  [NPORTS-1:0]               byp_port;
  logic   [NPORTS-1:0][NUM_VC-1:0]   bp_rdy, bp_hold, bp_pop;
  flit_t  [NPORTS-1:0][NUM_VC-1:0]   bp_front;
  logic                              bp_busy;     // a burst or lookahead is inside (testbench idle check)

`ifndef SYNTHESIS
  // (a hook the testbench writes: under `ifndef SYNTHESIS only, outside
  // the synthesis-off pragmas, so every simulator keeps it)
  logic   [NPORTS-1:0]               tb_hold;     // test hold, set only by a directed test
  initial tb_hold = '0;
`endif

  for (genvar p = 0; p < NPORTS; p++) begin : g_port
    input_unit #(
      .MESH_N   (MESH_N),
      .NUM_VC   (NUM_VC),
      .VC_DEPTH (VC_DEPTH),
      .ALGO     (ALGO)
    ) u_in (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .cur_x_i     (cur_x_i),
      .cur_y_i     (cur_y_i),
      .flit_i      (in_flit_i[p]),
      .credit_o    (in_credit_o[p]),
      .free_vc_i   (free_cnt),
      .req_o       (req[p]),
      .port_o      (vc_port[p]),
      .active_o    (active[p]),
      .outvc_o     (outvc[p]),
      .gnt_i       (in_gnt[p]),
      .gnt_vc_i    (in_vc[p]),
      .gnt_outvc_i (in_outvc[p]),
      .sa_flit_o   (sa_flit[p]),
      .sa_port_o   (sa_port[p]),
      .bp_rdy_o    (bp_rdy[p]),
      .bp_front_o  (bp_front[p]),
      .bp_hold_i   (bp_hold[p]),
      .bp_pop_i    (bp_pop[p])
    );

    output_unit #(
      .NUM_VC   (NUM_VC),
      .VC_DEPTH (VC_DEPTH)
    ) u_out (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .credit_i    (out_credit_i[p]),
      .gnt_i       (out_gnt[p]),
      .alloc_i     (out_alloc[p]),
      .gnt_vc_i    (out_vc[p]),
      .has_free_o  (has_free[p]),
      .alloc_vc_o  (alloc_vc[p]),
      .credit_ok_o (credit_ok[p]),
      .free_cnt_o  (free_cnt[p]),
      .st_flit_i   (st_flit[p]),
      .link_o      (out_flit_o[p])
    );
  end

  switch_allocator #(.NUM_VC(NUM_VC)) u_sa (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .req_i       (req),
    .port_i      (vc_port),
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

  crossbar u_xb (
    .clk_i      (clk_i),
    .rst_ni     (rst_ni),
    .sa_flit_i  (sa_flit),
    .sa_port_i  (sa_port),
    .byp_en_i   (byp_en),
    .byp_flit_i (byp_flit),
    .byp_port_i (byp_port),
    .st_flit_o  (st_flit)
  );

  // ---------------------------------------------------------------- bypass
  // Design A has no bypass: every hook is zero, as in B1-B3 (D29).
  if (DESIGN == DESIGN_A) begin : g_no_bypass
    assign in_mask  = '0;
    assign out_mask = '0;
    assign byp_en   = '0;
    assign byp_flit = '0;
    assign byp_port = '0;
    assign bp_hold  = '0;
    assign bp_pop   = '0;
    assign out_la_o = '0;
    assign out_cx_o = '0;
    assign la_loc_o = '0;
    assign bp_busy  = 1'b0;
    logic unused_bp;
    assign unused_bp = ^{bp_rdy, bp_front, in_la_i, tx_go_i, tx_dst_i, tx_mode_i, bp_busy, in_cx_i, cx_inj_i};
`ifndef SYNTHESIS
    logic unused_hold;
    assign unused_hold = ^tb_hold;
`endif
  end else begin : g_bypass
    logic  [NPORTS-1:0] src_in_mask, src_out_mask, rl_in_mask, rl_out_mask;
    logic               src_la_vld, src_busy, rl_busy, cx_hit, cx_busy;
    port_t              src_la_port;
    la_t                src_la;
    logic  [3:0]        rl_en;
    flit_t [3:0]        rl_flit;
    port_t [3:0]        rl_port;

    bypass_source #(
      .MESH_N (MESH_N),
      .NUM_VC (NUM_VC),
      .B_MAX  (B_MAX)
    ) u_src (
      .clk_i      (clk_i),
      .rst_ni     (rst_ni),
      .cur_x_i    (cur_x_i),
      .cur_y_i    (cur_y_i),
      .tx_go_i    (tx_go_i),
      .tx_dst_i   (tx_dst_i),
      .tx_mode_i  (tx_mode_i),
      .cancel_i   (cx_hit),
      .rdy_i      (bp_rdy),
      .front_i    (bp_front),
      .hold_o     (bp_hold),
      .pop_o      (bp_pop),
      .xb_en_o    (byp_en[P_L]),
      .xb_flit_o  (byp_flit[P_L]),
      .xb_port_o  (byp_port[P_L]),
      .in_mask_o  (src_in_mask),
      .out_mask_o (src_out_mask),
      .la_vld_o   (src_la_vld),
      .la_port_o  (src_la_port),
      .la_o       (src_la),
      .busy_o     (src_busy)
    );

    bypass_relay #(
      .MESH_N (MESH_N),
      .B_MAX  (B_MAX)
    ) u_relay (
      .clk_i         (clk_i),
      .rst_ni        (rst_ni),
      .cur_x_i       (cur_x_i),
      .cur_y_i       (cur_y_i),
      .la_i          (in_la_i),
      .flit_i        (in_flit_i[3:0]),
      .src_la_vld_i  (src_la_vld),
      .src_la_port_i (src_la_port),
      .src_la_i      (src_la),
      .xb_en_o       (rl_en),
      .xb_flit_o     (rl_flit),
      .xb_port_o     (rl_port),
      .in_mask_o     (rl_in_mask),
      .out_mask_o    (rl_out_mask),
      .la_o          (out_la_o),
      .la_loc_o      (la_loc_o),
      .busy_o        (rl_busy)
    );

    cancel_relay #(.MESH_N(MESH_N)) u_cxr (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .cur_x_i (cur_x_i),
      .cur_y_i (cur_y_i),
      .cx_i    (in_cx_i),
      .inj_i   (cx_inj_i),
      .cx_o    (out_cx_o),
      .hit_o   (cx_hit),
      .busy_o  (cx_busy)
    );

    assign byp_en[3:0]   = rl_en;
    assign byp_flit[3:0] = rl_flit;
    assign byp_port[3:0] = rl_port;
    assign out_mask      = src_out_mask | rl_out_mask;
    assign bp_busy       = src_busy | rl_busy | cx_busy;
    logic unused_busy;                 // read by the testbench only
    assign unused_busy   = bp_busy;
`ifndef SYNTHESIS
    assign in_mask       = src_in_mask | rl_in_mask | tb_hold;

    // Coverage of D38's rule (the testbench reads it): cycles in which the
    // source reads a packet from a mesh input whose crossbar input a passing
    // burst uses at the same time (allowed: the source's flits take crossbar
    // input 4)
    longint unsigned n_rule;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        n_rule <= 0;
      end else begin
        for (int p = 0; p < 4; p++) begin
          if ((|bp_pop[p]) && rl_en[p]) n_rule <= n_rule + 1;
        end
      end
    end
`else
    assign in_mask       = src_in_mask | rl_in_mask;
`endif
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (NUM_VC < 1 || NUM_VC > MAX_VC)
      $fatal(1, "%m: NUM_VC = %0d, must be 1..%0d", NUM_VC, MAX_VC);
    if (VC_DEPTH < MAX_PKT_FLITS)
      $fatal(1, "%m: VC_DEPTH = %0d cannot hold a %0d-flit packet", VC_DEPTH, MAX_PKT_FLITS);
    if (ALGO != RT_XY && ALGO != RT_ADAPTIVE)
      $fatal(1, "%m: regular routing must be RT_XY or RT_ADAPTIVE, not %0d", ALGO);
    if (DESIGN < DESIGN_A || DESIGN > DESIGN_D)
      $fatal(1, "%m: DESIGN = %0d does not exist", DESIGN);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else if (DESIGN == DESIGN_A) begin
      for (int p = 0; p < NPORTS; p++) begin
        if (in_flit_i[p].vld && in_flit_i[p].byp)
          $fatal(1, "%m: bypass flit on input %0d of a design-A router", p);
      end
      if (in_la_i != '0 || tx_go_i || in_cx_i != '0 || cx_inj_i.vld)
        $fatal(1, "%m: lookahead, send order or cancel token at a design-A router");
    end else begin
      if (in_flit_i[P_L].vld && in_flit_i[P_L].byp)
        $fatal(1, "%m: bypass flit from the NI");
    end
  end
  // synopsys translate_on
`endif

endmodule
