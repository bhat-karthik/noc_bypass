// =============================================================================
// File      tb/mesh/mesh_bench.sv                                  Tracker: F29
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The mesh test bench: clock and reset, the mesh, one traffic
//           generator and one sink per node, run control, a white-box idle
//           check, and the test tasks (tests_func.svh, tests_perf.svh,
//           tests_bypass.svh, tests_resv.svh, tests_two.svh). The tops
//           tb_mesh, tb_mesh_dl, tb_mesh_b, tb_mesh_c and tb_mesh_cb pick
//           its configuration.
// Decisions D24 testbench rules, D30 portability rules, D34 run control,
//           D35 mesh benches, D45 B4 verification, D49 B5 verification,
//           D51 B6 verification
// Tests     N01-N08, PERF (design A); BP01-BP10, PERF (design B); TL01-TL06,
//           PERF (design C)
// -----------------------------------------------------------------------------
// Mesh size: compile with +define+TB_MESH_N=8 (run.tcl: -define TB_MESH_N=8);
// the default is 4. Plusargs (besides +SEED, +TEST, +MAXERR, +CSV):
//   +N03_REPS=<n>     repetitions per pattern and load in N03 (default 5)
//   +N08_CYCLES=<n>   length of N08 (default 1,000,000)
//   PERF: +PATTERN=<name> +RATE=<packets/node/cycle> +PDATA=<share>
//         +WARMUP=<cycles> +MEASURE=<cycles> +STALL=<share> +RUNID=<text>
// Random streams: generator of node i 100+i, sink of node i 400+i, bench 50.
// Every test ends with the idle check: no flit anywhere, every VC idle and
// every credit home, in every router and NI (N08's criterion); in design B
// also no burst or lookahead inside a router and none expected by the
// bypass checker.
// Design B (DESIGN = DESIGN_B): router_checker (F43) checks every bypass
// and every cancel token against the timetable and D38's cycles, with
// ni_checker (F47) deciding each reservation and following every NI's
// bypass space; hold_req[node] sets the router's simulation-only test hold
// (switch allocation blocked for those inputs), applied at the falling edge
// after a test changes it at a rising edge. The NI FIFO is 25 flits in
// design A (D31) and sched_pkg::ej_depth_b in design B (48 flits for 4 VCs,
// B_MAX 10, TIGHT; D48).
// Design C (DESIGN = DESIGN_C, B6): the same two checkers follow the frames
// (sched_ref's frame_ref, shared), sched_monitor (F53) checks every tile's
// schedule outputs and counts turns per pair and round, and mesh_checker
// (F52) checks lockstep, frame and boundary rules, region containment and
// collisions from the lookaheads alone. The bench drives the mesh's plan
// input: a test sets plan_req, the bench applies it at the falling edge
// (as the hold) and tells frame_ref every plan the tiles sample (cycle
// A_DELAY - 1 of each epoch). SHADOW = 1 (tb_mesh_cb) adds a design-B mesh
// with the same inputs and compares every output and link every cycle.
// @support tb_mesh tb_mesh_dl tb_mesh_b tb_mesh_c tb_mesh_cb tb_mesh_d
// (the EDA bundle compiles this module only for the tests of those benches)
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module mesh_bench
  import noc_pkg::*;
  import traffic_pkg::*;
#(
  parameter int    MESH_N = 4,
  parameter int    NUM_VC = 4,
  parameter int    ALGO   = RT_XY,
  parameter int    DESIGN = DESIGN_A,
  parameter int    B_MAX  = 10,                    // design B: flits per burst
  parameter int    T_MODE = sched_pkg::TM_TIGHT,   // design B: subslot rule
  parameter string ID     = "N01-N08",    // test id when +TEST is not given
  parameter int    REGION_N = (MESH_N >= 4) ? 4 : MESH_N,                          // design C: region side
  parameter int    K_INIT   = sched_pkg::k_fixed(MESH_N, REGION_N, B_MAX, T_MODE),  // design C: k_L from reset
  parameter int    EPOCH    = sched_pkg::epoch_len(MESH_N, REGION_N, B_MAX, T_MODE), // design C: epoch, cycles
  parameter int    SHADOW   = 0,          // design C: compare with a design-B mesh (TL05)
  parameter int    K_LO     = sched_pkg::K_MIN,          // design D: clamp of k_L (E7, E8)
  parameter int    K_HI     = sched_pkg::k_max(MESH_N)
) ();

  localparam int NN       = MESH_N * MESH_N;
  localparam int VC_DEPTH = 5;
  localparam int EJ_DEPTH = (DESIGN == DESIGN_A) ? 25 : sched_pkg::ej_depth_b(NUM_VC, VC_DEPTH, MESH_N, B_MAX, T_MODE);
  localparam int SPACE    = EJ_DEPTH - NUM_VC * VC_DEPTH;   // design B: the bypass space (D48)
  localparam int EFW      = $clog2(EJ_DEPTH + 1);
  localparam int SID      = 50;            // the bench's random stream
  localparam bit TWO      = (DESIGN == DESIGN_C) || (DESIGN == DESIGN_D);   // two-level timetable

  // ---------------------------------------------------------------- clock, reset
  logic        clk   = 1'b0;
  // (the bench also reads rst_n at clock edges, to start its checkers; the
  // controller of design D, small enough to be inlined, uses it as an
  // asynchronous reset)
  /* verilator lint_off SYNCASYNCNET */
  logic        rst_n = 1'b0;
  /* verilator lint_on SYNCASYNCNET */
  int unsigned cyc   = 0;
  int unsigned c_rst = 0;      // the cycle in which reset rose: position 0 of every slot counter

  always #5 clk = ~clk;
  always @(posedge clk) cyc <= cyc + 1;

  // ---------------------------------------------------------------- mesh
  // per node, unpacked as the mesh ports (D36)
  logic            inj_valid [NN], inj_ready [NN], ej_valid [NN], ej_ready [NN];
  flit_t           inj_flit  [NN], ej_flit   [NN], ej_mon   [NN];
  logic  [EFW-1:0] ej_free   [NN];

  // design C: the plan the tiles sample (a test sets plan_req; the bench
  // applies it at the falling edge, g_chk); TL05: cycles in which the
  // design-B shadow mesh agreed, and whether it ever differed (g_shadow)
  // (plan_req is read only in design C's g_chk: unused in designs A and B)
  sched_pkg::kl_t  plan     = sched_pkg::kl_t'(K_INIT);
  /* verilator lint_off UNUSEDSIGNAL */
  sched_pkg::kl_t  plan_req = sched_pkg::kl_t'(K_INIT);
  /* verilator lint_on UNUSEDSIGNAL */
  longint unsigned sh_same  = 0;
  bit              sh_diff  = 1'b0;
  // design D: the controller's configuration (+POLICY=SQRT, +DAMP=0; a test
  // may change them at a rising edge), the plan the tiles sampled last
  logic            ctrl_pol  = 1'b0;
  logic            ctrl_damp = 1'b1;

  mesh_top #(
    .MESH_N   (MESH_N),
    .NUM_VC   (NUM_VC),
    .VC_DEPTH (VC_DEPTH),
    .ALGO     (ALGO),
    .DESIGN   (DESIGN),
    .EJ_DEPTH (EJ_DEPTH),
    .B_MAX    (B_MAX),
    .T_MODE   (T_MODE),
    .REGION_N (REGION_N),
    .K_FIXED  (K_INIT),
    .EPOCH    (EPOCH),
    .K_LO     (K_LO),
    .K_HI     (K_HI)
  ) dut (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .plan_i      (plan),
    .ctrl_pol_i  (ctrl_pol),
    .ctrl_damp_i (ctrl_damp),
    .inj_valid_i (inj_valid),
    .inj_ready_o (inj_ready),
    .inj_flit_i  (inj_flit),
    .ej_valid_o  (ej_valid),
    .ej_ready_i  (ej_ready),
    .ej_flit_o   (ej_flit),
    .ej_free_o   (ej_free),
    .ej_mon_o    (ej_mon)
  );

  // ---------------------------------------------------------------- nodes
  longint unsigned v_sent    [NN];   // per node: flits its NI took
  longint unsigned v_taken   [NN];   // per node: flits its sink took
  longint unsigned v_heads   [NN];   // per node: packets its NI took (heads)
  longint unsigned v_pkts    [NN];   // per node: packets its sink took whole (tails)
  longint unsigned v_blocked [NN];   // per node: cycles a flit waited for its NI
  longint unsigned v_stall   [NN];   // per node: cycles its sink stalled
  longint unsigned v_inter   [NN];   // per node: interleaved packets at the sink
  longint unsigned v_fifo    [NN];   // per node: most flits in its NI FIFO
  /* verilator lint_off UNUSEDSIGNAL */
  longint unsigned v_btake   [NN];   // per node: bypass flits its sink took (design B's NI model)
  /* verilator lint_on UNUSEDSIGNAL */
  int              v_queued  [NN];   // per node: packets in its source queue
  bit              v_sink_ok [NN];   // per node: sink has no open packet

  for (genvar i = 0; i < NN; i++) begin : g_node
    tg_node #(.ID(i), .MESH_N(MESH_N), .SID(100 + i)) u_tg (
      .clk       (clk),
      .rst_n     (rst_n),
      .cyc       (cyc),
      .inj_valid (inj_valid[i]),
      .inj_ready (inj_ready[i]),
      .inj_flit  (inj_flit[i])
    );
    sink_node #(.ID(i), .NUM_VC(NUM_VC), .SID(400 + i)) u_sink (
      .clk      (clk),
      .rst_n    (rst_n),
      .cyc      (cyc),
      .ej_valid (ej_valid[i]),
      .ej_ready (ej_ready[i]),
      .ej_flit  (ej_flit[i]),
      .ej_mon   (ej_mon[i])
    );
    assign v_sent[i]    = u_tg.n_sent;
    assign v_taken[i]   = u_sink.n_flits;
    assign v_heads[i]   = u_tg.n_heads;
    assign v_pkts[i]    = u_sink.n_pkts;
    assign v_blocked[i] = u_tg.n_blocked;
    assign v_queued[i]  = u_tg.queued;
    assign v_stall[i]   = u_sink.n_stall;
    assign v_inter[i]   = u_sink.n_inter;
    assign v_fifo[i]    = u_sink.max_fifo;
    assign v_btake[i]   = u_sink.n_btake;
    assign v_sink_ok[i] = u_sink.is_idle;
  end

  run_ctrl u_rc (.clk(clk), .cyc(cyc));

  // ---------------------------------------------------------------- probes
  // White box, per tile:
  //   tile_idle  no flit buffered or in a register, every VC idle with all
  //              its credits home, in the router and the NI (idle check)
  //   link_busy  a flit on one of the router's output links or on the NI's
  //              link this cycle (N05: frozen)
  //   loc_full   every VC of the router's Local output is held by a packet
  //              (N06: a stopped sink holds back its router)
  //   in_wait    per input port: VC 0 holds a head still waiting for its
  //              output VC; in_port: the output it waits for (N05)
  logic [NN-1:0]     tile_idle, link_busy, loc_full;
  logic [NPORTS-1:0] in_wait [NN];
  port_t [NPORTS-1:0] in_port [NN];
  // design B: the links the bypass checker watches (read only by g_chk, so
  // unread in design A), the test hold, the source's turn counters, the
  // reservation controller's counters and counts (zero in design A)
  /* verilator lint_off UNUSEDSIGNAL */
  flit_t [3:0]       o_flit  [NN];
  la_t   [3:0]       o_la    [NN];
  cx_t   [3:0]       o_cx    [NN];
  /* verilator lint_on UNUSEDSIGNAL */
  logic [NPORTS-1:0] hold_req [NN];
  longint unsigned   v_turns [NN], v_used [NN], v_bflits [NN], v_bpkts [NN];
  longint unsigned   v_multi [NN], v_remote [NN], v_full [NN], v_rule [NN], v_b2b [NN], v_abut [NN];
  longint unsigned   v_cxl [NN], v_lost [NN];                       // source: turns cancelled, with packets waiting
  longint unsigned   v_dec [NN], v_acc [NN], v_rcx [NN], v_nosh [NN], v_two [NN];   // NI: decisions ...
  int                p_res [NN], v_msum [NN];                      // NI: reserved, largest sum
  /* verilator lint_off UNUSEDSIGNAL */
  int                p_fill [NN];                                  // NI: bypass flits (design B's NI model)
  // design C: each tile's sequencer state, the turn and decision it gives
  // its router and NI, and its router's lookahead for the NI
  sched_pkg::seq_st_t p_st    [NN];
  logic               p_txgo  [NN], p_txmd [NN], p_dec [NN], p_ds0 [NN], p_dloc [NN], p_dmd [NN];
  node_id_t           p_txdst [NN], p_dsrc [NN];
  sched_pkg::hop_t    p_dhops [NN];
  la_t                p_lal   [NN];
  // design D: each tile's router input links and the credits it sends back
  // (the controller checker's own demand model), the sums its
  // demand_counter latched, the plan its sequencer samples; the
  // controller's sums and plan
  flit_t   [NPORTS-1:0]           p_rin  [NN];
  credit_t [NPORTS-1:0]           p_rcr  [NN];
  logic [sched_pkg::DM_W-1:0]     p_cl   [NN], p_cg [NN];
  sched_pkg::kl_t                 p_plan [NN];
  logic [sched_pkg::dm_sum_w(MESH_N)-1:0] p_sl, p_sg;
  sched_pkg::kl_t                 p_kc;
  /* verilator lint_on UNUSEDSIGNAL */

  for (genvar y = 0; y < MESH_N; y++) begin : g_iy
    for (genvar x = 0; x < MESH_N; x++) begin : g_ix
      localparam int T = y * MESH_N + x;
      logic [NPORTS-1:0] port_ok, out_vld, w;
      port_t [NPORTS-1:0] rp;
      logic [NUM_VC-1:0] ni_cr;
      for (genvar p = 0; p < NPORTS; p++) begin : g_ip
        logic [NUM_VC-1:0] cr, st;
        assign w[p]  = int'(dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_in.state_q[0]) == 1;   // VC_ROUTED
        assign rp[p] = dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_in.port_q[0];
        for (genvar v = 0; v < NUM_VC; v++) begin : g_iv
          assign cr[v] = int'(dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_out.cred_q[v]) == VC_DEPTH;
          assign st[v] = int'(dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_in.state_q[v]) == 0;
        end
        assign port_ok[p] = (dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_out.busy_q == '0)
                          && (&cr) && (&st)
                          && (&dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_in.empty)
                          && !dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_in.sa_vld_q
                          && !dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_out.link_vld_q;
        assign out_vld[p] = dut.g_row[y].g_col[x].u_tile.u_router.g_port[p].u_out.link_vld_q;
      end
      for (genvar v = 0; v < NUM_VC; v++) begin : g_nv
        assign ni_cr[v] = int'(dut.g_row[y].g_col[x].u_tile.u_ni.u_inj.cred_q[v]) == VC_DEPTH;
      end
      assign tile_idle[T] = (&port_ok)
                          && !dut.g_row[y].g_col[x].u_tile.u_router.bp_busy
                          && (dut.g_row[y].g_col[x].u_tile.u_router.u_xb.st_vld_q == '0)
                          && (dut.g_row[y].g_col[x].u_tile.u_ni.u_inj.busy_q == '0)
                          && (&ni_cr)
                          && !dut.g_row[y].g_col[x].u_tile.u_ni.u_inj.link_vld_q
                          && !dut.g_row[y].g_col[x].u_tile.u_ni.in_pkt_q
                          && dut.g_row[y].g_col[x].u_tile.u_ni.ej_empty;
      assign in_wait[T]   = w;
      assign in_port[T]   = rp;
      assign link_busy[T] = (|out_vld) || dut.g_row[y].g_col[x].u_tile.u_ni.u_inj.link_vld_q;
      assign loc_full[T]  = &dut.g_row[y].g_col[x].u_tile.u_router.g_port[P_L].u_out.busy_q;
      assign o_flit[T]    = dut.g_row[y].g_col[x].u_tile.out_flit_o;
      assign o_la[T]      = dut.g_row[y].g_col[x].u_tile.out_la_o;
      assign o_cx[T]      = dut.g_row[y].g_col[x].u_tile.out_cx_o;
      // the test hold reaches the router at the falling edge
      always @(negedge clk) dut.g_row[y].g_col[x].u_tile.u_router.tb_hold = hold_req[T];
      if (DESIGN != DESIGN_A) begin : g_bp
        assign v_turns[T]  = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_turns;
        assign v_used[T]   = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_used;
        assign v_bflits[T] = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_flits;
        assign v_bpkts[T]  = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_pkts;
        assign v_multi[T]  = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_multi;
        assign v_remote[T] = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_remote;
        assign v_full[T]   = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_full;
        assign v_rule[T]   = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.n_rule;
        assign v_b2b[T]    = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_b2b;
        assign v_abut[T]   = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_relay.n_abut;
        assign v_cxl[T]    = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_cxl;
        assign v_lost[T]   = dut.g_row[y].g_col[x].u_tile.u_router.g_bypass.u_src.n_lost;
        assign v_dec[T]    = dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.n_dec;
        assign v_acc[T]    = dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.n_acc;
        assign v_rcx[T]    = dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.n_cxl;
        assign v_nosh[T]   = dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.n_noshow;
        assign v_two[T]    = dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.n_two;
        assign p_res[T]    = int'(dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.res_q);
        assign p_fill[T]   = int'(dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.fill_q);
        assign v_msum[T]   = int'(dut.g_row[y].g_col[x].u_tile.u_ni.g_byp.u_resv.max_sum);
      end else begin : g_no_bp
        assign v_turns[T]  = 0;
        assign v_used[T]   = 0;
        assign v_bflits[T] = 0;
        assign v_bpkts[T]  = 0;
        assign v_multi[T]  = 0;
        assign v_remote[T] = 0;
        assign v_full[T]   = 0;
        assign v_rule[T]   = 0;
        assign v_b2b[T]    = 0;
        assign v_abut[T]   = 0;
        assign v_cxl[T]    = 0;
        assign v_lost[T]   = 0;
        assign v_dec[T]    = 0;
        assign v_acc[T]    = 0;
        assign v_rcx[T]    = 0;
        assign v_nosh[T]   = 0;
        assign v_two[T]    = 0;
        assign p_res[T]    = 0;
        assign p_fill[T]   = 0;
        assign v_msum[T]   = 0;
      end
      if (TWO) begin : g_tl
        assign p_st[T]    = dut.g_row[y].g_col[x].u_tile.g_tt2.u_seq.st_o;
        assign p_txgo[T]  = dut.g_row[y].g_col[x].u_tile.tx_go;
        assign p_txdst[T] = dut.g_row[y].g_col[x].u_tile.tx_dst;
        assign p_txmd[T]  = dut.g_row[y].g_col[x].u_tile.tx_mode;
        assign p_dec[T]   = dut.g_row[y].g_col[x].u_tile.dec;
        assign p_ds0[T]   = dut.g_row[y].g_col[x].u_tile.dec_s0;
        assign p_dloc[T]  = dut.g_row[y].g_col[x].u_tile.dec_loc;
        assign p_dsrc[T]  = dut.g_row[y].g_col[x].u_tile.dec_src;
        assign p_dmd[T]   = dut.g_row[y].g_col[x].u_tile.dec_mode;
        assign p_dhops[T] = dut.g_row[y].g_col[x].u_tile.dec_hops;
        assign p_lal[T]   = dut.g_row[y].g_col[x].u_tile.la_loc;
      end else begin : g_no_tl
        assign p_st[T]    = '0;
        assign p_txgo[T]  = 1'b0;
        assign p_txdst[T] = '0;
        assign p_txmd[T]  = 1'b0;
        assign p_dec[T]   = 1'b0;
        assign p_ds0[T]   = 1'b0;
        assign p_dloc[T]  = 1'b0;
        assign p_dsrc[T]  = '0;
        assign p_dmd[T]   = 1'b0;
        assign p_dhops[T] = '0;
        assign p_lal[T]   = '0;
      end
      if (DESIGN == DESIGN_D) begin : g_dm
        assign p_rin[T]  = dut.g_row[y].g_col[x].u_tile.r_in_flit;
        assign p_rcr[T]  = dut.g_row[y].g_col[x].u_tile.r_in_cred;
        assign p_cl[T]   = dut.g_row[y].g_col[x].u_tile.g_tt2.g_ctrl.cl;
        assign p_cg[T]   = dut.g_row[y].g_col[x].u_tile.g_tt2.g_ctrl.cg;
        assign p_plan[T] = dut.g_row[y].g_col[x].u_tile.g_tt2.plan_seq;
      end else begin : g_no_dm
        assign p_rin[T]  = '0;
        assign p_rcr[T]  = '0;
        assign p_cl[T]   = '0;
        assign p_cg[T]   = '0;
        assign p_plan[T] = '0;
      end
    end
  end

  if (DESIGN == DESIGN_D) begin : g_cp
    assign p_sl = dut.g_ctrl.u_ctrl.sl_q;
    assign p_sg = dut.g_ctrl.u_ctrl.sg_q;
    assign p_kc = dut.g_ctrl.u_ctrl.plan_o;
  end else begin : g_no_cp
    assign p_sl = '0;
    assign p_sg = '0;
    assign p_kc = '0;
  end

  // design B: every bypass and cancel token checked against the timetable
  // and D38's cycles (router_checker, F43), every reservation against the
  // NI model (ni_checker, F47): every falling edge the links, the NI counts
  // and the sinks' bypass takes go into the checkers. Design C: the same,
  // following the frames, plus sched_monitor (F53) and mesh_checker (F52).
  if (DESIGN == DESIGN_B || TWO) begin : g_chk
    rchk_pkg::rchk chk;
    nchk_pkg::nchk nck;
    sched_ref_pkg::frame_ref fr;
    smon_pkg::smon smo;
    mchk_pkg::mchk mck;

    initial begin
      chk = new(MESH_N, sched_pkg::t_sub(MESH_N, B_MAX, T_MODE), sched_pkg::s_setup(MESH_N), B_MAX);
      nck = new(NN, B_MAX, SPACE, EJ_DEPTH, NUM_VC * VC_DEPTH);
      chk.nck = nck;
      rchk_pkg::the_chk  = chk;
      nchk_pkg::the_nchk = nck;
      if (TWO) begin
        fr  = new(MESH_N, REGION_N, sched_pkg::t_sub(REGION_N, B_MAX, T_MODE), sched_pkg::t_sub(MESH_N, B_MAX, T_MODE), K_INIT);
        chk.set_frames(fr, REGION_N, sched_pkg::t_sub(REGION_N, B_MAX, T_MODE), sched_pkg::s_setup(REGION_N));
        smo = new(MESH_N, REGION_N, fr);
        mck = new(MESH_N, REGION_N, sched_pkg::t_sub(REGION_N, B_MAX, T_MODE), sched_pkg::t_sub(MESH_N, B_MAX, T_MODE), fr);
        smon_pkg::the_smon = smo;
        mchk_pkg::the_mchk = mck;
        sched_ref_pkg::the_fr = fr;
      end
    end

    // position 0 of the slot counters: the cycle in which reset rose
    always @(posedge clk) begin
      if (rst_n && !chk.started) begin
        chk.c0      = int'(cyc);
        chk.started = 1'b1;
        if (TWO) begin
          smo.c0      = int'(cyc);
          smo.started = 1'b1;
          mck.c0      = int'(cyc);
          mck.started = 1'b1;
        end
      end
    end

    // (the checkers start sampling in the cycle after reset rose; that first
    // cycle only decides subslot 0 of slot 0, a same-group slot, which has
    // no transfer)
    always @(negedge clk) begin
      int k, e;
      // design C: the plan a test asked for reaches the tiles now; frame_ref
      // learns each plan the tiles sample (end of cycle A_DELAY - 1 of an epoch)
      // design D: the plan every tile samples comes from the controller
      // (tile 0's; the controller checker compares every tile's)
      if (DESIGN == DESIGN_C) plan = plan_req;
      if (TWO && chk.started) begin
        e = int'(cyc) - chk.c0;
        if (e % EPOCH == sched_pkg::A_DELAY - 1) begin
          if (DESIGN == DESIGN_D) k = int'(p_plan[0]);
          else                    k = int'(plan);
          k = fr.sample(longint'(e), k);
        end
      end
      if (chk.started) begin
        for (int r = 0; r < NN; r++) begin
          for (int p = 0; p < 4; p++) begin
            k = r * rchk_pkg::NRES + p;
            chk.obs_fl[k] = o_flit[r][p];
            chk.obs_v[k]  = o_flit[r][p].vld && o_flit[r][p].byp;
            k = r * rchk_pkg::NRES + 4 + p;
            chk.obs_la[k] = o_la[r][p];
            chk.obs_v[k]  = o_la[r][p].vld;
          end
          k = r * rchk_pkg::NRES + 8;
          chk.obs_fl[k] = ej_mon[r];
          chk.obs_v[k]  = ej_mon[r].vld && ej_mon[r].byp;
          for (int p = 0; p < 4; p++) begin
            k = r * rchk_pkg::NRES + 9 + p;
            chk.obs_cx[k] = o_cx[r][p];
            chk.obs_v[k]  = o_cx[r][p].vld;
          end
          chk.nck.p_res[r]  = p_res[r];
          chk.nck.p_fill[r] = p_fill[r];
          chk.nck.p_occ[r]  = EJ_DEPTH - int'(ej_free[r]);
          chk.nck.taken[r]  = longint'(v_btake[r]);
        end
        if (TWO) begin
          for (int r = 0; r < NN; r++) begin
            smo.tx_go[r]    = p_txgo[r];
            smo.tx_dst[r]   = int'(p_txdst[r]);
            smo.tx_mode[r]  = p_txmd[r];
            smo.dec[r]      = p_dec[r];
            smo.dec_s0[r]   = p_ds0[r];
            smo.dec_loc[r]  = p_dloc[r];
            smo.dec_src[r]  = int'(p_dsrc[r]);
            smo.dec_mode[r] = p_dmd[r];
            smo.dec_hops[r] = int'(p_dhops[r]);
            mck.st[r]       = p_st[r];
            for (int p = 0; p < 4; p++) begin
              mck.fl_v[r * 4 + p]   = o_flit[r][p].vld && o_flit[r][p].byp;
              mck.la_v[r * 4 + p]   = o_la[r][p].vld;
              mck.la_len[r * 4 + p] = int'(o_la[r][p].len);
              mck.cx_v[r * 4 + p]   = o_cx[r][p].vld;
            end
            mck.ej_v[r]    = ej_mon[r].vld && ej_mon[r].byp;
            mck.lal_v[r]   = p_lal[r].vld;
            mck.lal_len[r] = int'(p_lal[r].len);
          end
        end
        chk.step(int'(cyc));
        if (TWO) begin
          smo.step(int'(cyc));
          mck.step(int'(cyc));
        end
      end
    end
  end

  // design D: the controller checked end to end (ctrl_checker, F75): every
  // falling edge the links and the RTL's sums and plans go into its model
  if (DESIGN == DESIGN_D) begin : g_cchk
    cchk_pkg::cchk cck;

    initial begin
      cck = new(MESH_N, REGION_N, NUM_VC, EPOCH, sched_pkg::A_DELAY, K_INIT, K_LO, K_HI);
      cchk_pkg::the_cchk = cck;
    end

    always @(posedge clk) begin
      if (rst_n && !cck.started) begin
        cck.c0      = longint'(cyc);
        cck.started = 1'b1;
      end
    end

    always @(negedge clk) begin
      if (cck.started) begin
        for (int r = 0; r < NN; r++) begin
          for (int p = 0; p < NPORTS; p++) begin
            cck.fl[r * NPORTS + p] = p_rin[r][p];
            cck.cr[r * NPORTS + p] = p_rcr[r][p];
          end
          cck.rcl[r] = int'(p_cl[r]);
          cck.rcg[r] = int'(p_cg[r]);
          cck.rpl[r] = int'(p_plan[r]);
        end
        cck.rsl  = longint'(p_sl);
        cck.rsg  = longint'(p_sg);
        cck.rkc  = int'(p_kc);
        cck.pol  = int'(ctrl_pol);
        cck.damp = ctrl_damp;
        cck.step(longint'(cyc));
      end
    end
  end

  // design C, TL05: a design-B mesh with the same inputs; every output and
  // every link equal every cycle (D50 on a 4x4 mesh, k_L = 0 elsewhere)
  if (DESIGN == DESIGN_C && SHADOW != 0) begin : g_shadow
    logic            s_inj_ready [NN], s_ej_valid [NN];
    flit_t           s_ej_flit   [NN], s_ej_mon   [NN];
    logic  [EFW-1:0] s_ej_free   [NN];
    flit_t [3:0]     s_flit [NN];
    la_t   [3:0]     s_la   [NN];
    cx_t   [3:0]     s_cx   [NN];

    mesh_top #(
      .MESH_N   (MESH_N),
      .NUM_VC   (NUM_VC),
      .VC_DEPTH (VC_DEPTH),
      .ALGO     (ALGO),
      .DESIGN   (DESIGN_B),
      .EJ_DEPTH (EJ_DEPTH),
      .B_MAX    (B_MAX),
      .T_MODE   (T_MODE)
    ) u_shadow (
      .clk_i       (clk),
      .rst_ni      (rst_n),
      .plan_i      (plan),
      .ctrl_pol_i  (1'b0),
      .ctrl_damp_i (1'b0),
      .inj_valid_i (inj_valid),
      .inj_ready_o (s_inj_ready),
      .inj_flit_i  (inj_flit),
      .ej_valid_o  (s_ej_valid),
      .ej_ready_i  (ej_ready),
      .ej_flit_o   (s_ej_flit),
      .ej_free_o   (s_ej_free),
      .ej_mon_o    (s_ej_mon)
    );

    for (genvar y = 0; y < MESH_N; y++) begin : g_sy
      for (genvar x = 0; x < MESH_N; x++) begin : g_sx
        assign s_flit[y * MESH_N + x] = u_shadow.g_row[y].g_col[x].u_tile.out_flit_o;
        assign s_la[y * MESH_N + x]   = u_shadow.g_row[y].g_col[x].u_tile.out_la_o;
        assign s_cx[y * MESH_N + x]   = u_shadow.g_row[y].g_col[x].u_tile.out_cx_o;
        always @(negedge clk) u_shadow.g_row[y].g_col[x].u_tile.u_router.tb_hold = hold_req[y * MESH_N + x];
      end
    end

    // compare from the falling edge after the first rising edge out of
    // reset: reset rises on a falling edge, where a test of rst_n would race
    // the main block (Verilator counted that edge, xsim and Riviera did not)
    logic sh_on = 1'b0;
    always @(posedge clk) sh_on <= rst_n;

    // one check per cycle while the two agree; the first difference is
    // reported and ends the comparison (everything after it differs too)
    always @(negedge clk) begin
      int    bad;
      string what;
      if (sh_on && !sh_diff) begin
        bad  = -1;
        what = "";
        for (int r = 0; r < NN; r++) begin
          if (bad < 0) begin
            if (s_inj_ready[r] != inj_ready[r] || s_ej_valid[r] != ej_valid[r] || s_ej_free[r] != ej_free[r]) begin
              bad  = r;
              what = $sformatf("NI handshake: inj_ready %0d/%0d ej_valid %0d/%0d ej_free %0d/%0d", inj_ready[r],
                               s_inj_ready[r], ej_valid[r], s_ej_valid[r], ej_free[r], s_ej_free[r]);
            end else if (s_ej_flit[r] != ej_flit[r] || s_ej_mon[r] != ej_mon[r]) begin
              bad  = r;
              what = "ejected flit or the NI's input link";
            end else if (s_flit[r] != o_flit[r]) begin
              bad  = r;
              what = "an output link's flit";
            end else if (s_la[r] != o_la[r]) begin
              bad  = r;
              what = "an output lookahead link";
            end else if (s_cx[r] != o_cx[r]) begin
              bad  = r;
              what = "an output cancel link";
            end
          end
        end
        `TB_CHECK(bad < 0, ("TL05: at cycle %0d design C and design B differ at R%0d: %s", cyc, bad, what))
        if (bad >= 0) sh_diff = 1'b1;
        else          sh_same++;
      end
    end
  end

  // designs C and D: cycles, k_L of the running frame and local-phase
  // cycles of tile 0, summed from reset, for PERF's k_mean and share_L
  // (M15, B8). Nonblocking updates, read by the tests at rising edges
  // (README rule 10); each cast into a variable first (rule 9).
  longint unsigned pp_cyc = 0, pp_kl = 0, pp_loc = 0;
  always @(posedge clk) begin : b_pp
    longint unsigned kk, ll;
    kk = longint'(p_st[0].kl);
    ll = longint'(p_st[0].loc);
    if (rst_n) begin
      pp_cyc <= pp_cyc + 1;
      pp_kl  <= pp_kl + kk;
      pp_loc <= pp_loc + ll;
    end
  end

  // ---------------------------------------------------------------- helpers
  function automatic longint unsigned sum_sent();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_sent[i];
    return s;
  endfunction

  function automatic longint unsigned sum_taken();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_taken[i];
    return s;
  endfunction

  function automatic longint unsigned sum_heads();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_heads[i];
    return s;
  endfunction

  function automatic longint unsigned sum_pkts();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_pkts[i];
    return s;
  endfunction

  function automatic longint unsigned sum_blocked();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_blocked[i];
    return s;
  endfunction

  function automatic longint unsigned sum_stall();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_stall[i];
    return s;
  endfunction

  function automatic longint unsigned sum_inter();
    longint unsigned s = 0;
    for (int i = 0; i < NN; i++) s += v_inter[i];
    return s;
  endfunction

  function automatic longint unsigned max_fifo();
    longint unsigned m = 0;
    for (int i = 0; i < NN; i++) if (v_fifo[i] > m) m = v_fifo[i];
    return m;
  endfunction

  // design B: the RTL's bypass and reservation counters, summed (perf_probe)
  function automatic perf_pkg::snap_t perf_snap();
    perf_pkg::snap_t z;
    z.turns  = sum_of(v_turns);
    z.used   = sum_of(v_used);
    z.cxl    = sum_of(v_cxl);
    z.lost   = sum_of(v_lost);
    z.bflits = sum_of(v_bflits);
    z.bpkts  = sum_of(v_bpkts);
    z.dec    = sum_of(v_dec);
    z.acc    = sum_of(v_acc);
    z.rcx    = sum_of(v_rcx);
    z.nosh   = sum_of(v_nosh);
    z.two    = sum_of(v_two);
    return z;
  endfunction

  // every source queue empty, every sink without an open packet, every NI
  // ejection FIFO empty (all EJ_DEPTH slots free)
  function automatic bit nodes_idle();
    for (int i = 0; i < NN; i++) begin
      if (v_queued[i] != 0 || !v_sink_ok[i] || int'(ej_free[i]) != EJ_DEPTH) return 1'b0;
    end
    return 1'b1;
  endfunction

  // A few cycles for the last credits to return, then the whole mesh idle
  task automatic check_idle(string when);
    repeat (12) @(posedge clk);
    `TB_CHECK(&tile_idle, ("%s: mesh not idle afterwards, busy tiles %b", when, ~tile_idle))
    `TB_CHECK(nodes_idle(), ("%s: a source queue, sink or NI ejection FIFO still holds a packet", when))
    `TB_CHECK(sb_pkg::outstanding() == 0,
              ("%s: %0d packets created but not delivered", when, sb_pkg::outstanding()))
    if (rchk_pkg::the_chk != null) begin
      `TB_CHECK(rchk_pkg::the_chk.idle(), ("%s: the bypass checker still expects flits", when))
    end
  endtask

  // Traffic settings back to their defaults before each test
  function automatic void traffic_defaults();
    traffic_pkg::mesh_n       = MESH_N;
    traffic_pkg::pattern      = PAT_UNIFORM;
    traffic_pkg::rate         = 0.0;
    traffic_pkg::p_data       = 1.0 / 3.0;
    traffic_pkg::hot_node     = -1;
    traffic_pkg::hot_frac     = 0.2;
    traffic_pkg::region_n     = TWO ? REGION_N : 0;
    traffic_pkg::p_local      = 0.5;
    traffic_pkg::phase_a      = PAT_UNIFORM;
    traffic_pkg::phase_b      = PAT_TRANSPOSE;
    traffic_pkg::phase_len    = 500;
    traffic_pkg::gen_on       = 1'b0;
    traffic_pkg::period       = 0;
    traffic_pkg::per_t0       = 0;
    traffic_pkg::sink_stall_p = 0.0;
    traffic_pkg::fs_node      = FS_NONE;
    traffic_pkg::fs_start     = 0;
    traffic_pkg::fs_end       = 0;
  endfunction

  function automatic string algo_name();
    if (ALGO == RT_XY) return "XY";
    return "adaptive";
  endfunction

  function automatic string tmode_name();
    string m;
    if (T_MODE == sched_pkg::TM_TIGHT) m = "TIGHT";
    else                               m = "SAFE";
    return m;
  endfunction

  function automatic string design_name();
    string s, m;
    m = tmode_name();
    if (DESIGN == DESIGN_A)      s = "design A";
    else if (DESIGN == DESIGN_B) s = $sformatf("design B, B_MAX %0d, %s subslots of %0d cycles", B_MAX, m,
                                               sched_pkg::t_sub(MESH_N, B_MAX, T_MODE));
    else if (DESIGN == DESIGN_D) s = $sformatf("design D, %0dx%0d regions, k_L %0d from reset and by demand every epoch of %0d, B_MAX %0d, %s subslots of %0d (local %0d) cycles",
                                               REGION_N, REGION_N, K_INIT, EPOCH, B_MAX, m,
                                               sched_pkg::t_sub(MESH_N, B_MAX, T_MODE), sched_pkg::t_sub(REGION_N, B_MAX, T_MODE));
    else if (REGION_N == MESH_N) s = $sformatf("design C with one %0dx%0d region (D50), B_MAX %0d, %s subslots of %0d cycles",
                                               REGION_N, REGION_N, B_MAX, m, sched_pkg::t_sub(MESH_N, B_MAX, T_MODE));
    else                         s = $sformatf("design C, %0dx%0d regions, k_L %0d from reset, epoch %0d, B_MAX %0d, %s subslots of %0d (local %0d) cycles",
                                               REGION_N, REGION_N, K_INIT, EPOCH, B_MAX, m,
                                               sched_pkg::t_sub(MESH_N, B_MAX, T_MODE), sched_pkg::t_sub(REGION_N, B_MAX, T_MODE));
    return s;
  endfunction

  initial begin
    for (int i = 0; i < NN; i++) hold_req[i] = '0;
  end

  `include "tests_func.svh"
  `include "tests_perf.svh"
  `include "tests_bypass.svh"
  `include "tests_resv.svh"
  `include "tests_two.svh"
  `include "tests_ctrl.svh"
  `include "tests_stress.svh"

  // ---------------------------------------------------------------- main
  initial begin
    tb_pkg::start(ID, $sformatf("mesh %0dx%0d, %0d VC, %s routing, %s", MESH_N, MESH_N,
                                NUM_VC, algo_name(), design_name()));
    tb_pkg::rng_init(SID);
    for (int i = 0; i < NN; i++) begin
      tb_pkg::rng_init(100 + i);
      tb_pkg::rng_init(400 + i);
    end
    traffic_defaults();
    repeat (5) @(posedge clk);
    @(negedge clk) rst_n = 1'b1;
    c_rst = cyc;
    @(posedge clk);
    // PERF runs only when +TEST (or the bundle's define) names it
    if (DESIGN == DESIGN_C) begin
      if (SHADOW != 0) begin
        if (tb_pkg::test_on("TL05")) tl05();
      end else begin
        if (tb_pkg::test_on("TL01")) tl01();
        if (tb_pkg::test_on("TL02")) tl02();
        if (tb_pkg::test_on("TL03")) tl03();
        if (tb_pkg::test_on("TL04")) tl04();
        if (tb_pkg::test_on("TL06")) tl06();
        // regular tests on design C, only when +TEST names one
        if (tb_pkg::sel_test == "N01") n01();
        if (tb_pkg::sel_test == "N03") n03();
        if (tb_pkg::sel_test == "N06") n06();
        if (tb_pkg::sel_test == "N07") n07();
        if (tb_pkg::sel_test == "PERF") perf();
        // the B8 stress tests, only when +TEST names one (tests_stress.svh)
        if (tb_pkg::sel_test == "S01") s01();
        if (tb_pkg::sel_test == "S02") s02();
        if (tb_pkg::sel_test == "S03") s03();
        if (tb_pkg::sel_test == "S05") s05();
      end
      tl_summary();
    end else if (DESIGN == DESIGN_D) begin
      ct_config();
      if (tb_pkg::test_on("CT01")) ct01();
      if (tb_pkg::test_on("CT02")) ct02();
      if (tb_pkg::test_on("CT03")) ct03();
      if (tb_pkg::test_on("CT04")) ct04();
      if (tb_pkg::test_on("CT05")) ct05();
      // regular tests on design D, only when +TEST names one
      if (tb_pkg::sel_test == "N01") n01();
      if (tb_pkg::sel_test == "N03") n03();
      if (tb_pkg::sel_test == "N06") n06();
      if (tb_pkg::sel_test == "N07") n07();
      if (tb_pkg::sel_test == "PERF") perf();
      // the B8 stress tests, only when +TEST names one (tests_stress.svh)
      if (tb_pkg::sel_test == "S01") s01();
      if (tb_pkg::sel_test == "S02") s02();
      if (tb_pkg::sel_test == "S03") s03();
      if (tb_pkg::sel_test == "S05") s05();
      ct_summary();
    end else if (DESIGN != DESIGN_A) begin
      if (tb_pkg::test_on("BP01")) bp01();
      if (tb_pkg::test_on("BP02")) bp02();
      if (tb_pkg::test_on("BP03")) bp03();
      if (tb_pkg::test_on("BP04")) bp04();
      if (tb_pkg::test_on("BP05")) bp05();
      if (tb_pkg::test_on("BP06")) bp06();
      if (tb_pkg::test_on("BP07")) bp07();
      if (tb_pkg::test_on("BP09")) bp09();
      if (tb_pkg::test_on("BP10")) bp10();
      // BP08 needs one VC (+define+TB_NUM_VC=1): run alone, by +TEST
      if (tb_pkg::sel_test == "BP08") bp08();
      // regular tests on design B, only when +TEST names one
      if (tb_pkg::sel_test == "N01") n01();
      if (tb_pkg::sel_test == "N03") n03();
      if (tb_pkg::sel_test == "N06") n06();
      if (tb_pkg::sel_test == "N07") n07();
      if (tb_pkg::sel_test == "PERF") perf();
      // the B8 stress tests, only when +TEST names one (tests_stress.svh)
      if (tb_pkg::sel_test == "S01") s01();
      if (tb_pkg::sel_test == "S02") s02();
      if (tb_pkg::sel_test == "S03") s03();
      if (tb_pkg::sel_test == "S05") s05();
    end else if (ALGO == RT_XY) begin
      if (tb_pkg::test_on("N01")) n01();
      if (tb_pkg::test_on("N02")) n02();
      if (tb_pkg::test_on("N03")) n03();
      if (tb_pkg::test_on("N04")) n04();
      if (tb_pkg::test_on("N06")) n06();
      if (tb_pkg::test_on("N07")) n07();
      if (tb_pkg::test_on("N08")) n08();
      if (tb_pkg::sel_test == "PERF") perf();
      // the B8 stress tests, only when +TEST names one (tests_stress.svh)
      if (tb_pkg::sel_test == "S01") s01();
      if (tb_pkg::sel_test == "S02") s02();
      if (tb_pkg::sel_test == "S03") s03();
      if (tb_pkg::sel_test == "S05") s05();
    end else begin
      if (tb_pkg::test_on("N05")) n05();
      // BP08's design-A half: the traffic that design B survives deadlocks here
      if (tb_pkg::sel_test == "BP08") bp08();
    end
    // designs C-D: a test that ends on a falling edge (TL04 alone) moves to
    // the next rising edge before the end, so the checkers' falling-edge
    // block runs in that cycle on every simulator (Riviera ran it before the
    // main block's $finish, Verilator after: 71 checks apart)
    if ((DESIGN == DESIGN_C || DESIGN == DESIGN_D) && clk === 1'b0) @(posedge clk);
    tb_pkg::info($sformatf("cycles simulated %0d", cyc));
    tb_pkg::finish();
  end

  // Safety net for a run that never ends: 5 million cycles plus N08's length,
  // plus S01's when S01 runs (10,000,000 cycles or +S01_CYCLES: its full form
  // outlasted the old limit of 6,000,000 cycles on xsim, R-1114-07 to 10). The
  // test name comes from the plusarg, not tb_pkg::sel_test, which another
  // initial block sets
  initial begin
    int unsigned n8, n1;
    string       tn;
    time         t_max;
    if (!$value$plusargs("N08_CYCLES=%d", n8)) n8 = 1000000;
    n1 = 0;
    if ($value$plusargs("TEST=%s", tn)) begin
      if (tn == "S01") begin
        n1 = 10000000;
        if ($value$plusargs("S01_CYCLES=%d", n1)) begin end
      end
    end
    t_max = (time'(n8) + time'(n1) + time'(5000000)) * 10;
    tb_pkg::watchdog(t_max);
  end

endmodule
