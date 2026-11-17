// =============================================================================
// File      tb/unit/tb_phase_sequencer.sv                          Tracker: F51
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U11: the two-level timetable's sequencer against the frame
//           reference every cycle: frames of k_L local slots and one global
//           slot, each counter resuming where it stopped, phase changes at
//           slot boundaries only, the epoch tick, the plan sampled A_DELAY - 1
//           cycles after each tick and applied at the next frame start, and
//           the one-pointer case (region = mesh, D50)
// DUT       rtl/sched/phase_sequencer.sv (F50)
// Reference tb/common/sched_ref.sv (F36): frame_ref, the position in closed
//           form from the cycle number and the plan history (each
//           timetable's position from its own count of running cycles)
// @tests    U11
// -----------------------------------------------------------------------------
// Nine sequencers with different meshes, regions, subslot rules, burst
// lengths, epochs and sample delays run at the same time:
//   8x8/4 TIGHT B10  k 4, epoch 4096, delay 256      (the design-C default sides)
//   8x8/4 SAFE  B10  k 3, epoch 1024, delay 7        (frames longer than an epoch)
//   8x8/2 TIGHT B5   k_fixed, epoch 2048, delay 1    (2x2 regions)
//   16x16/4 TIGHT B10 k 14, epoch 2048, delay 256    (K_MAX 47: samples replaced)
//   16x16/8 SAFE B15 k_fixed, epoch 8192, delay 100  (8x8 regions)
//   4x4/2 TIGHT B10  k 3, epoch 512, delay 100       (the small two-level mesh)
//   4x4/4            one pointer (D50), epoch 512
//   8x8/8            one pointer (D50), epoch 1024
//   8x8/4 TIGHT B10  k 0 from reset, epoch 1148 = 4 global slots, delay 287:
//                    the first sample falls in the last cycle of a frame and
//                    the plan held at 5 takes effect at once (directed)
// The plan input changes at random cycles (one in 400; a quarter of the new
// values 0, 15% D18's K_MAX, 5% 63 = the register's limit, the rest
// 1..K_MAX-1), so samples catch every kind of change (8x8/2 starts at
// k_fixed = 19, above 8x8's K_MAX). Every cycle, for each sequencer:
//   * the whole state (phase, k_L, local slots done, both counters' setup,
//     subslot, cycle and slot) equals frame_ref's position;
//   * loc, k_L, both counters' slot, subslot and next subslot outputs, and
//     the running counter's sub_go and nxt_go pulses follow that position;
//     tick at cycle 0 of every epoch.
// Before the run, thirteen checks of sched_pkg's frame functions against the
// tracker's numbers (K_FIXED 4, 14, 3, 11; frames of 603, 524, 1,472 and
// 2,193 cycles; epochs 2^15 and 2^18; K_MAX 15 and 47), and the epoch rule's
// k_L = 4 where it matters: on 8x8 with B_MAX 5 one global round at k_L = 4
// (17,388 cycles) needs 2^15, at k_L = 3 (15,264) 2^14 would do.
// Run length: 400,000 cycles (16x16: 600,000; 4x4: 200,000); +QUICK an eighth.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module ps_tester
  import sched_pkg::*;
#(
  parameter int N    = 8,
  parameter int RG   = 4,
  parameter int BM   = 10,
  parameter int MODE = TM_TIGHT,
  parameter int K0   = 4,
  parameter int EP   = 4096,
  parameter int AD   = 256,
  parameter int PFIX = -1,       // >= 0: the plan is held at this value for the first 4 epochs
  parameter int TOT  = 400000,   // cycles to run
  parameter int SID  = 100,      // random stream of this tester
  parameter bit XTAG = 1'b0      // name ends " k0 epoch <EP>" (no empty string parameter: xsim
                                 // 2024.2 prints "" as one blank, README rule 9)
) (
  input  logic go,
  output logic done
);
  localparam int TG = t_sub(N, BM, MODE);
  localparam int TL = t_sub(RG, BM, MODE);
  localparam int KM = k_max(N);
  localparam bit TWO = (RG < N);

  logic    clk   = 1'b0;
  logic    rst_n = 1'b0;
  kl_t     plan;
  logic    loc, sub_go, nxt_go, tick;
  slot_t   slot_l, slot_g;
  sub_t    sub_l, sub_g, nsub_l, nsub_g;
  kl_t     kl;
  seq_st_t st;

  phase_sequencer #(
    .MESH_N   (N),
    .REGION_N (RG),
    .T_SUB_G  (TG),
    .T_SUB_L  (TL),
    .K_INIT   (K0),
    .EPOCH    (EP),
    .A_DLY    (AD)
  ) u_dut (
    .clk_i       (clk),
    .rst_ni      (rst_n),
    .plan_i      (plan),
    .loc_o       (loc),
    .slot_l_o    (slot_l),
    .sub_l_o     (sub_l),
    .nxt_sub_l_o (nsub_l),
    .slot_g_o    (slot_g),
    .sub_g_o     (sub_g),
    .nxt_sub_g_o (nsub_g),
    .sub_go_o    (sub_go),
    .nxt_go_o    (nxt_go),
    .kl_o        (kl),
    .tick_o      (tick),
    .st_o        (st)
  );

  always #5 clk = ~clk;

  sched_ref_pkg::frame_ref fr;
  string                   name;
  longint unsigned c_apply = 0, c_k0 = 0, c_kmax = 0, c_lg = 0, c_gl = 0, c_now = 0, c_repl = 0;
  longint unsigned c_gwrap = 0, c_tick = 0, c_frames = 0, c_k63 = 0;
  int              k_prev;
  bit              loc_prev;

  // xsim 2024.2 misreads some casts inside expressions (R-1110-02): every
  // DUT field goes through a plain variable before it is compared
  task automatic check_cycle(longint e);
    sched_ref_pkg::fpos_t p;
    sched_ref_pkg::pos_t  rp;
    int  s_loc, s_kl, s_lc, s_lset, s_lsub, s_lcyc, s_lslot, s_gset, s_gsub, s_gcyc, s_gslot;
    int  o_loc, o_kl, o_sl, o_bl, o_nl, o_sg, o_bg, o_ng, o_go, o_ngo, o_tick;
    int  x_loc, x_lset, x_gset, x_go, x_ngo, x_nl, x_ng, x_tick, nrun;
    longint epx;
    p       = fr.at(e);
    epx     = longint'(EP);
    x_loc   = p.loc ? 1 : 0;
    x_lset  = p.lp.setup ? 1 : 0;
    x_gset  = p.gp.setup ? 1 : 0;
    s_loc   = int'(st.loc);
    s_kl    = int'(st.kl);
    s_lc    = int'(st.lc);
    s_lset  = int'(st.l_setup);
    s_lsub  = int'(st.l_sub);
    s_lcyc  = int'(st.l_cyc);
    s_lslot = int'(st.l_slot);
    s_gset  = int'(st.g_setup);
    s_gsub  = int'(st.g_sub);
    s_gcyc  = int'(st.g_cyc);
    s_gslot = int'(st.g_slot);
    o_loc   = int'(loc);
    o_kl    = int'(kl);
    o_sl    = int'(slot_l);
    o_bl    = int'(sub_l);
    o_nl    = int'(nsub_l);
    o_sg    = int'(slot_g);
    o_bg    = int'(sub_g);
    o_ng    = int'(nsub_g);
    o_go    = int'(sub_go);
    o_ngo   = int'(nxt_go);
    o_tick  = int'(tick);
    if (p.loc) begin
      rp   = p.lp;
      nrun = RG;
    end else begin
      rp   = p.gp;
      nrun = N;
    end
    x_go   = (!rp.setup && rp.cyc == 0) ? 1 : 0;
    x_ngo  = (rp.cyc == 0 && (rp.setup || rp.sub < nrun - 1)) ? 1 : 0;
    x_nl   = p.lp.setup ? 0 : ((p.lp.sub + 1) & 15);
    x_ng   = p.gp.setup ? 0 : ((p.gp.sub + 1) & 15);
    x_tick = (e % epx == 0) ? 1 : 0;
    // the whole state
    `TB_CHECK(s_loc == x_loc && s_kl == p.k && s_lc == p.lc,
      ("%s cycle %0d: phase %0d k_L %0d local slots done %0d; expected %0d %0d %0d", name, e, s_loc, s_kl, s_lc,
       p.loc, p.k, p.lc))
    if (TWO) begin
      `TB_CHECK(s_lset == x_lset && s_lsub == p.lp.sub && s_lcyc == p.lp.cyc && s_lslot == p.lp.slot,
        ("%s cycle %0d: local counter setup %0d subslot %0d cycle %0d slot %0d; expected %0d %0d %0d %0d", name, e,
         s_lset, s_lsub, s_lcyc, s_lslot, p.lp.setup, p.lp.sub, p.lp.cyc, p.lp.slot))
    end else begin
      `TB_CHECK(s_lset == 0 && s_lsub == 0 && s_lcyc == 0 && s_lslot == 0 && o_sl == 0 && o_bl == 0 && o_nl == 0,
        ("%s cycle %0d: one pointer, but the local counter reads setup %0d subslot %0d cycle %0d slot %0d", name, e,
         s_lset, s_lsub, s_lcyc, s_lslot))
    end
    `TB_CHECK(s_gset == x_gset && s_gsub == p.gp.sub && s_gcyc == p.gp.cyc && s_gslot == p.gp.slot,
      ("%s cycle %0d: global counter setup %0d subslot %0d cycle %0d slot %0d; expected %0d %0d %0d %0d", name, e,
       s_gset, s_gsub, s_gcyc, s_gslot, p.gp.setup, p.gp.sub, p.gp.cyc, p.gp.slot))
    // the outputs
    `TB_CHECK(o_loc == x_loc && o_kl == p.k && o_go == x_go && o_ngo == x_ngo && o_tick == x_tick,
      ("%s cycle %0d: loc %0d k_L %0d sub_go %0d nxt_go %0d tick %0d; expected %0d %0d %0d %0d %0d", name, e,
       o_loc, o_kl, o_go, o_ngo, o_tick, p.loc, p.k, x_go, x_ngo, x_tick))
    if (TWO) begin
      `TB_CHECK(o_sl == p.lp.slot && o_bl == p.lp.sub && o_nl == x_nl,
        ("%s cycle %0d: local slot %0d subslot %0d next %0d; expected %0d %0d %0d", name, e, o_sl, o_bl, o_nl,
         p.lp.slot, p.lp.sub, x_nl))
    end
    `TB_CHECK(o_sg == p.gp.slot && o_bg == p.gp.sub && o_ng == x_ng,
      ("%s cycle %0d: global slot %0d subslot %0d next %0d; expected %0d %0d %0d", name, e, o_sg, o_bg, o_ng,
       p.gp.slot, p.gp.sub, x_ng))
    // coverage
    if (p.fstart) begin
      c_frames++;
      if (e > 0 && p.k != k_prev) c_apply++;
      if (p.k == 0)  c_k0++;
      if (p.k == KM) c_kmax++;
      if (p.k == 63) c_k63++;
      k_prev = p.k;
    end
    if (e > 0 && loc_prev && !p.loc) c_lg++;
    if (e > 0 && !loc_prev && p.loc) c_gl++;
    if (!p.loc && e > 0 && p.gp.setup && p.gp.cyc == 0 && p.gp.slot == 0) c_gwrap++;
    if (x_tick == 1) c_tick++;
    loc_prev = p.loc;
  endtask

  initial begin
    longint e, total, tq, epl, adl;
    int     r, nv, kp;
    kl_t    kk;
    bit     quick;
    string  mname;
    done = 1'b0;
    wait (go === 1'b1);
    tb_pkg::rng_init(SID);
    quick = $test$plusargs("QUICK");
    tq    = longint'(TOT);
    epl   = longint'(EP);
    adl   = longint'(AD);
    total = quick ? tq / 8 : tq;
    if (MODE == TM_TIGHT) mname = "TIGHT";
    else                  mname = "SAFE";
    if (TWO && XTAG) name = $sformatf("%0dx%0d/%0d %s B%0d k0 epoch %0d", N, N, RG, mname, BM, EP);
    else if (TWO)    name = $sformatf("%0dx%0d/%0d %s B%0d", N, N, RG, mname, BM);
    else             name = $sformatf("%0dx%0d one pointer", N, N);
    fr       = new(N, RG, TL, TG, K0);
    k_prev   = TWO ? K0 : 0;
    loc_prev = 1'b0;
    if (PFIX >= 0) plan = kl_t'(PFIX);
    else           plan = kl_t'(K0);
    repeat (2) @(posedge clk);
    #1 rst_n = 1'b1;
    for (e = 0; e < total; e++) begin
      @(negedge clk);
      check_cycle(e);
      // a new plan value now and then (draws never on the right of && or ||, D30)
      r = int'(tb_pkg::rng_below(SID, 400));
      if (r == 0 && !(PFIX >= 0 && e < 4 * epl)) begin
        nv = int'(tb_pkg::rng_below(SID, 20));
        if (nv < 5)      kk = '0;
        else if (nv < 8) kk = kl_t'(KM);
        else if (nv < 9) kk = '1;                 // 63, the largest k_L the register holds
        else begin                                // 1 .. KM - 1 (no cast inside an expression: rule 9)
          nv = int'(tb_pkg::rng_below(SID, KM - 1));
          nv = nv + 1;
          kk = kl_t'(nv);
        end
        plan = kk;
      end
      // the sequencer registers plan_i at the end of cycle AD - 1 of each epoch
      if (e % epl == adl - 1) begin
        kp = int'(plan);                          // never a cast as an argument (R-1110-02)
        r  = fr.sample(e, kp);
        if (r == 2) c_repl++;
        if (r == 1 && fr.last_start() == e + 1) c_now++;
      end
    end
    tb_pkg::info($sformatf("U11 %s: %0d cycles, %0d frames, %0d plan changes applied, local/global slots %0d/%0d cycles",
                           name, total, c_frames, c_apply, slot_len(RG, TL), slot_len(N, TG)));
    tb_pkg::cover_bin($sformatf("U11 %s frame starts", name), c_frames);
    tb_pkg::cover_bin($sformatf("U11 %s epoch ticks", name), c_tick);
    if (TWO) begin
      tb_pkg::cover_bin($sformatf("U11 %s new k_L applied at a frame start", name), c_apply);
      // with +QUICK the random plans may not reach every value (goals)
      if (quick) begin
        tb_pkg::cover_goal($sformatf("U11 %s frames with k_L = 0 (global only)", name), c_k0);
        tb_pkg::cover_goal($sformatf("U11 %s frames with k_L = K_MAX", name), c_kmax);
      end else begin
        tb_pkg::cover_bin($sformatf("U11 %s frames with k_L = 0 (global only)", name), c_k0);
        tb_pkg::cover_bin($sformatf("U11 %s frames with k_L = K_MAX", name), c_kmax);
      end
      tb_pkg::cover_bin($sformatf("U11 %s local -> global at a slot end", name), c_lg);
      tb_pkg::cover_bin($sformatf("U11 %s global -> local at a frame start", name), c_gl);
      if (PFIX >= 0)
        tb_pkg::cover_bin($sformatf("U11 %s sample in a frame's last cycle, applied at once", name), c_now);
      else
        tb_pkg::cover_goal($sformatf("U11 %s sample in a frame's last cycle, applied at once", name), c_now);
      tb_pkg::cover_goal($sformatf("U11 %s sample replaced before taking effect", name), c_repl);
      tb_pkg::cover_goal($sformatf("U11 %s frames with k_L = 63 (register limit)", name), c_k63);
    end
    tb_pkg::cover_goal($sformatf("U11 %s global slot number wraps to 0", name), c_gwrap);
    done = 1'b1;
  end

endmodule


module tb_phase_sequencer;
  import sched_pkg::*;

  logic       go = 1'b0;
  logic [8:0] done;

  ps_tester #(.N(8),  .RG(4), .BM(10), .MODE(TM_TIGHT), .K0(4),  .EP(4096), .AD(256), .TOT(400000), .SID(100)) t_8a (.go(go), .done(done[0]));
  ps_tester #(.N(8),  .RG(4), .BM(10), .MODE(TM_SAFE),  .K0(3),  .EP(1024), .AD(7),   .TOT(400000), .SID(101)) t_8b (.go(go), .done(done[1]));
  ps_tester #(.N(8),  .RG(2), .BM(5),  .MODE(TM_TIGHT), .K0(k_fixed(8, 2, 5, TM_TIGHT)), .EP(2048), .AD(1), .TOT(400000), .SID(102)) t_8c (.go(go), .done(done[2]));
  ps_tester #(.N(16), .RG(4), .BM(10), .MODE(TM_TIGHT), .K0(14), .EP(2048), .AD(256), .TOT(600000), .SID(103)) t_16a (.go(go), .done(done[3]));
  ps_tester #(.N(16), .RG(8), .BM(15), .MODE(TM_SAFE),  .K0(k_fixed(16, 8, 15, TM_SAFE)), .EP(8192), .AD(100), .TOT(600000), .SID(104)) t_16b (.go(go), .done(done[4]));
  ps_tester #(.N(4),  .RG(2), .BM(10), .MODE(TM_TIGHT), .K0(3),  .EP(512),  .AD(100), .TOT(200000), .SID(105)) t_4a (.go(go), .done(done[5]));
  ps_tester #(.N(4),  .RG(4), .BM(10), .MODE(TM_TIGHT), .K0(1),  .EP(512),  .AD(100), .TOT(200000), .SID(106)) t_4o (.go(go), .done(done[6]));
  ps_tester #(.N(8),  .RG(8), .BM(10), .MODE(TM_TIGHT), .K0(1),  .EP(1024), .AD(256), .TOT(400000), .SID(107)) t_8o (.go(go), .done(done[7]));
  ps_tester #(.N(8),  .RG(4), .BM(10), .MODE(TM_TIGHT), .K0(0),  .EP(1148), .AD(287), .PFIX(5), .TOT(400000), .SID(108), .XTAG(1'b1)) t_8d (.go(go), .done(done[8]));

  `TB_DUMP(tb_phase_sequencer)

  initial tb_pkg::watchdog(100ms);

  // sched_pkg's frame functions against the tracker's Section 5 numbers
  // (written out here, not computed): the testbenches and the mesh derive
  // K_FIXED and EPOCH from these functions, so nothing else would notice a
  // wrong one
  task automatic pkg_numbers();
    int got [13];
    int exp [13];
    got[0]  = k_fixed(8, 4, 10, TM_TIGHT);
    got[1]  = k_fixed(16, 4, 10, TM_TIGHT);
    got[2]  = k_fixed(8, 4, 10, TM_SAFE);
    got[3]  = k_fixed(16, 4, 10, TM_SAFE);
    got[4]  = frame_len(8, 4, 10, TM_TIGHT, 4);
    got[5]  = frame_len(8, 4, 10, TM_TIGHT, 3);
    got[6]  = frame_len(8, 4, 10, TM_TIGHT, 15);
    got[7]  = frame_len(16, 4, 10, TM_TIGHT, 14);
    got[8]  = epoch_len(8, 4, 10, TM_TIGHT);
    got[9]  = epoch_len(16, 4, 10, TM_TIGHT);
    got[10] = k_max(8);
    got[11] = k_max(16);
    got[12] = epoch_len(8, 4, 5, TM_TIGHT);   // a round at k_L = 4: 36 x 483 = 17,388
    exp = '{4, 14, 3, 11, 603, 524, 1472, 2193, 32768, 262144, 15, 47, 32768};
    for (int i = 0; i < 13; i++)
      `TB_CHECK(got[i] == exp[i], ("U11 sched_pkg number %0d (K_FIXED 8x8, 16x16, SAFE 8x8, 16x16; frames k 4, 3, 15, 16x16 k 14; epochs; K_MAX; epoch 8x8 B_MAX 5): %0d, the tracker says %0d",
                                   i, got[i], exp[i]))
  endtask

  initial begin
    tb_pkg::start("U11", "phase_sequencer: frames, resumed counters, epoch and plan switch equal the frame reference every cycle");
    pkg_numbers();
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::info("U11 sequencers: 8x8/4 TIGHT and SAFE, 8x8/2 B5, 16x16/4, 16x16/8 SAFE B15, 4x4/2, one pointer on 4x4 and 8x8, a sample at a frame's end");
    tb_pkg::finish();
  end

endmodule
