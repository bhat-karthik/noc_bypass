// =============================================================================
// File      tb/unit/tb_demand_counter.sv                           Tracker: F57
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U12: design D's demand counter against a reference model every
//           cycle: which input VCs are occupied, local or cross-region, the
//           sums every 16th cycle of an epoch, saturation at 2^14 - 1, the
//           latch at each tick, and reset
// DUT       rtl/ctrl/demand_counter.sv (F54)
// @tests    U12
// -----------------------------------------------------------------------------
// Seven counters run at the same time, each with its own random links:
//   8x8/4 at (5, 2), 4 VCs     4x4/2 at (1, 3), 4 VCs    16x16/4 at (15, 15)
//   16x16/8 at (7, 8), 2 VCs   8x8/4 at (0, 0), 1 VC
//   8x8/4 at (3, 3) SAT: every VC held for epochs of 16,384 cycles, first
//                    all local, then all cross-region, then mixed, so both
//                    sums saturate
//   8x8/2 at (6, 1) RST: a reset in the middle of an epoch, twice
// Stimulus, per port and cycle (driven at the falling edge): a head for a
// VC the model holds free (its destination inside the tile's region about
// half the time), body, tail and bypass flits (which must not count), the
// credit that frees an occupied VC, credits that free nothing, now and then
// a head and a free credit for the same VC in one cycle (the head wins).
// Ticks come after epochs of random length (16 x 4..64 cycles, a quarter
// of them not a multiple of 16), always in the first cycle after reset as
// phase_sequencer gives them. The model follows D54 from the same links.
// Checks, every cycle and counter (one check): the occupied VCs, which of
// them are local, and the latched sums cl_o and cg_o equal the model's.
// Run length 200,000 cycles (SAT 120,000); +QUICK an eighth (SAT 60,000).
// One draw per statement and never on the right of && (README Section 2,
// rule 1): Verilator 5.020 drew there even after a false left side, so until
// 14 Nov its U12 stimulus differed from xsim's (the count did not).
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

// (heads and helper arguments carry more bits than a check reads)
/* verilator lint_off UNUSEDSIGNAL */
module dm_tester
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int N    = 8,
  parameter int RG   = 4,
  parameter int NVC  = 4,
  parameter int X    = 0,
  parameter int Y    = 0,
  parameter int MODE = 0,        // 0 random, 1 saturation, 2 resets
  parameter int TOT  = 200000,
  parameter int SID  = 200
) (
  input  logic go,
  output logic done
);
  localparam int NV = NPORTS * NVC;

  logic clk = 1'b0, rst_n = 1'b0, tick = 1'b0;
  flit_t   [NPORTS-1:0] fl;
  credit_t [NPORTS-1:0] cr;
  logic [DM_W-1:0] cl, cg;

  always #5 clk = ~clk;

  demand_counter #(.MESH_N(N), .REGION_N(RG), .NUM_VC(NVC)) u_dut (
    .clk_i (clk), .rst_ni (rst_n), .cur_x_i (coord_t'(X)), .cur_y_i (coord_t'(Y)),
    .tick_i (tick), .in_flit_i (fl), .in_cred_i (cr), .cl_o (cl), .cg_o (cg)
  );

  // ---------------------------------------------------------------- model
  bit  m_occ [NV], m_loc [NV];
  int  m_al, m_ag, m_cl, m_cg, m_ph;
  // the stimulus of the cycle just ended (the DUT took it at the rising edge)
  bit      p_tick;
  flit_t   [NPORTS-1:0] p_fl;
  credit_t [NPORTS-1:0] p_cr;
  string   name;
  longint  n_samp, n_tick, n_sat, n_both, n_heads, n_free, n_ign;

  function automatic bit in_reg(int d);
    int dx, dy;
    dx = d % N;
    dy = d / N;
    return (dx / RG == X / RG) && (dy / RG == Y / RG);
  endfunction

  function automatic void m_reset();
    for (int i = 0; i < NV; i++) begin
      m_occ[i] = 1'b0;
      m_loc[i] = 1'b0;
    end
    m_al = 0;
    m_ag = 0;
    m_cl = 0;
    m_cg = 0;
    m_ph = 0;
  endfunction

  // one cycle: sample (with the state at its start), latch at a tick, then
  // the links change the occupancy
  function automatic void m_step();
    int    nl, ng, i, vi, ci, d;
    bit    hd;
    head_t h;
    nl = 0;
    ng = 0;
    for (int k = 0; k < NV; k++) begin
      if (m_occ[k] &&  m_loc[k]) nl++;
      if (m_occ[k] && !m_loc[k]) ng++;
    end
    if (p_tick) begin
      m_cl = m_al;
      m_cg = m_ag;
      m_al = nl;
      m_ag = ng;
      m_ph = 1;
      n_tick++;
      n_samp++;
    end else begin
      if (m_ph == 0) begin
        m_al = m_al + nl;
        m_ag = m_ag + ng;
        if (m_al > 16383) m_al = 16383;
        if (m_ag > 16383) m_ag = 16383;
        n_samp++;
      end
      m_ph = (m_ph + 1) % 16;
    end
    if (m_al == 16383 || m_ag == 16383) n_sat++;
    for (int p = 0; p < NPORTS; p++) begin
      vi = int'(p_fl[p].vc);
      ci = int'(p_cr[p].vc);
      hd = p_fl[p].vld && !p_fl[p].byp && is_head(p_fl[p].ftype);
      if (hd) begin
        h = head_t'(p_fl[p].data);
        d = int'(h.dst);
        i = p * NVC + vi;
        m_occ[i] = 1'b1;
        m_loc[i] = in_reg(d);
      end
      if (p_cr[p].vld && p_cr[p].vc_free && !(hd && vi == ci)) begin
        i = p * NVC + ci;
        m_occ[i] = 1'b0;
      end
    end
  endfunction

  // ---------------------------------------------------------------- stimulus
  int unsigned c_ep, ep_len;          // cycles into the epoch, its length
  int          sat_phase;

  function automatic int rnd(int n);
    int r;
    r = int'(tb_pkg::rng_below(SID, n));
    return r;
  endfunction

  function automatic bit chance(real p);
    return tb_pkg::rng_chance(SID, p);
  endfunction

  // a destination inside (loc = 1) or outside the tile's region
  function automatic int pick_dst(bit loc);
    int d, t;
    d = 0;
    for (int k = 0; k < 64; k++) begin
      t = rnd(N * N);
      if (in_reg(t) == loc) begin
        d = t;
        break;
      end
    end
    return d;
  endfunction

  function automatic flit_t mk_flit(ftype_t t, int vc, bit byp, int dst);
    flit_t f;
    head_t h;
    int    sr, ck;
    sr      = rnd(N * N);
    ck      = rnd(1 << 30);
    h       = '0;
    h.dst   = node_id_t'(dst);
    h.src   = node_id_t'(sr);
    h.len   = 3'd5;
    h.check = ck;
    f       = '0;
    f.vld   = 1'b1;
    f.ftype = t;
    f.vc    = vc_id_t'(vc);
    f.byp   = byp;
    f.data  = h;
    return f;
  endfunction

  task automatic drive();
    int     v, d, k, t, cvc;
    bit     loc, fr, hp;
    ftype_t ft;
    fl   = '0;
    cr   = '0;
    tick = 1'b0;
    // the epoch
    if (c_ep + 1 >= ep_len) begin
      tick = 1'b1;
      c_ep = 0;
      if (MODE == 1) ep_len = 16384 + 16 * rnd(8);
      else begin
        ep_len = 16 * (4 + rnd(61));
        if (chance(0.25)) ep_len = ep_len + 1 + rnd(15);
      end
    end else begin
      c_ep++;
    end
    for (int p = 0; p < NPORTS; p++) begin
      if (MODE == 1) begin
        // saturation: keep every VC occupied (all local, then all cross-region,
        // then half and half), never free one; refill after a phase change
        for (int vv = 0; vv < NVC; vv++) begin
          k = p * NVC + vv;
          if (sat_phase == 0)      loc = 1'b1;
          else if (sat_phase == 1) loc = 1'b0;
          else                     loc = (vv % 2 == 0);
          if (!m_occ[k] || m_loc[k] != loc) begin
            // free a VC of the wrong kind first, then a new head later
            if (m_occ[k] && !cr[p].vld) begin
              cr[p].vld     = 1'b1;
              cr[p].vc      = vc_id_t'(vv);
              cr[p].vc_free = 1'b1;
            end else if (!m_occ[k] && !fl[p].vld) begin
              d     = pick_dst(loc);
              fl[p] = mk_flit(FT_HEAD, vv, 1'b0, d);
            end
          end
        end
        continue;
      end
      // a flit
      if (chance(0.45)) begin
        t = rnd(10);
        v = rnd(NVC);
        if (t < 5) begin
          // a head for a free VC, if one is free
          k = -1;
          for (int vv = 0; vv < NVC; vv++) begin
            if (!m_occ[p * NVC + ((v + vv) % NVC)] && k < 0) k = (v + vv) % NVC;
          end
          if (k >= 0) begin
            loc = chance(0.5);
            if (chance(0.3)) ft = FT_SINGLE;
            else             ft = FT_HEAD;
            d     = pick_dst(loc);
            fl[p] = mk_flit(ft, k, 1'b0, d);
            n_heads++;
          end
        end else if (t < 8) begin
          if (t == 5)      ft = FT_BODY;
          else if (t == 6) ft = FT_TAIL;
          else             ft = FT_BODY;
          d     = rnd(N * N);
          fl[p] = mk_flit(ft, v, 1'b0, d);
          n_ign++;
        end else begin
          // a bypass flit, even a head: never counted
          if (t == 8) ft = FT_HEAD;
          else        ft = FT_TAIL;
          d     = rnd(N * N);
          fl[p] = mk_flit(ft, v, 1'b1, d);
          n_ign++;
        end
      end
      // a credit
      if (chance(0.35)) begin
        cvc = rnd(NVC);
        cr[p].vld = 1'b1;
        cr[p].vc  = vc_id_t'(cvc);
        fr = 1'b0;
        if (m_occ[p * NVC + cvc]) fr = chance(0.4);
        if (fr) begin
          cr[p].vc_free = 1'b1;
          n_free++;
        end else if (chance(0.05)) begin
          cr[p].vc_free = 1'b1;          // frees a VC that is free already
        end
      end
      // now and then a head and its VC's free credit together: the head wins
      hp = 1'b0;
      if (fl[p].vld && !fl[p].byp && is_head(fl[p].ftype)) hp = chance(0.02);
      if (hp) begin
        cr[p].vld     = 1'b1;
        cr[p].vc      = fl[p].vc;
        cr[p].vc_free = 1'b1;
        n_both++;
      end
    end
  endtask

  // ---------------------------------------------------------------- run
  initial begin
    longint total, c, rst_at1, rst_at2;
    bit     quick, ok;
    logic [NV-1:0] g_occ, g_loc, e_occ, e_loc;
    int     x_cl, x_cg;
    done = 1'b0;
    fl   = '0;
    cr   = '0;
    wait (go === 1'b1);
    tb_pkg::rng_init(SID);
    quick = $test$plusargs("QUICK");
    total = longint'(TOT);
    if (quick) total = total / 8;
    if (MODE == 1 && quick) total = 60000;
    if (MODE == 0)      name = $sformatf("%0dx%0d/%0d at (%0d, %0d), %0d VCs", N, N, RG, X, Y, NVC);
    else if (MODE == 1) name = $sformatf("%0dx%0d/%0d at (%0d, %0d) SAT", N, N, RG, X, Y);
    else                name = $sformatf("%0dx%0d/%0d at (%0d, %0d) RST", N, N, RG, X, Y);
    rst_at1 = total / 3 + 7;
    rst_at2 = (2 * total) / 3 + 3;
    m_reset();
    p_tick    = 1'b0;
    p_fl      = '0;
    p_cr      = '0;
    sat_phase = 0;
    ep_len    = 1;                       // the first cycle after reset is a tick
    c_ep      = 0;
    repeat (3) @(negedge clk);
    rst_n = 1'b1;
    for (c = 0; c < total; c++) begin
      @(negedge clk);
      // the cycle that just ended
      m_step();
      // compare
      for (int k = 0; k < NV; k++) begin
        g_occ[k] = u_dut.occ_q[k / NVC][k % NVC];
        g_loc[k] = u_dut.loc_q[k / NVC][k % NVC] & g_occ[k];
        e_occ[k] = m_occ[k];
        e_loc[k] = m_loc[k] & m_occ[k];
      end
      x_cl = int'(cl);
      x_cg = int'(cg);
      ok   = (g_occ == e_occ) && (g_loc == e_loc) && (x_cl == m_cl) && (x_cg == m_cg);
      `TB_CHECK(ok, ("U12 %s cycle %0d: occupied %b local %b sums %0d %0d; model %b %b %0d %0d", name, c,
                     g_occ, g_loc, x_cl, x_cg, e_occ, e_loc, m_cl, m_cg))
      // resets in the middle of an epoch (asynchronous, at a falling edge)
      if (MODE == 2 && (c == rst_at1 || c == rst_at2)) begin
        rst_n = 1'b0;
        m_reset();
        fl = '0;
        cr = '0;
        tick = 1'b0;
        p_tick = 1'b0;
        p_fl = '0;
        p_cr = '0;
        repeat (2) @(negedge clk);
        `TB_CHECK(cl == '0 && cg == '0 && u_dut.occ_q == '0, ("U12 %s: state not cleared by reset", name))
        rst_n  = 1'b1;
        ep_len = 1;
        c_ep   = 0;
      end
      // saturation phases: all local, all cross-region, mixed
      if (MODE == 1) sat_phase = int'((c * 3) / total);
      drive();
      p_tick = tick;
      p_fl   = fl;
      p_cr   = cr;
    end
    tb_pkg::info($sformatf("U12 %s: %0d cycles, %0d ticks, %0d samples, %0d heads, %0d VCs freed, %0d flits that must not count, %0d head-and-free pairs",
                           name, total, n_tick, n_samp, n_heads, n_free, n_ign, n_both));
    tb_pkg::cover_bin($sformatf("U12 %s epochs latched", name), n_tick);
    if (MODE == 1) tb_pkg::cover_bin($sformatf("U12 %s cycles with a saturated sum", name), n_sat);
    if (MODE == 0) tb_pkg::cover_bin($sformatf("U12 %s heads and free credits together", name), n_both);
    done = 1'b1;
  end

endmodule
/* verilator lint_on UNUSEDSIGNAL */


module tb_demand_counter;
  import sched_pkg::*;

  logic       go = 1'b0;
  logic [6:0] done;

  dm_tester #(.N(8),  .RG(4), .NVC(4), .X(5),  .Y(2),  .MODE(0), .TOT(200000), .SID(200)) t_a (.go(go), .done(done[0]));
  dm_tester #(.N(4),  .RG(2), .NVC(4), .X(1),  .Y(3),  .MODE(0), .TOT(200000), .SID(201)) t_b (.go(go), .done(done[1]));
  dm_tester #(.N(16), .RG(4), .NVC(4), .X(15), .Y(15), .MODE(0), .TOT(200000), .SID(202)) t_c (.go(go), .done(done[2]));
  dm_tester #(.N(16), .RG(8), .NVC(2), .X(7),  .Y(8),  .MODE(0), .TOT(200000), .SID(203)) t_d (.go(go), .done(done[3]));
  dm_tester #(.N(8),  .RG(4), .NVC(1), .X(0),  .Y(0),  .MODE(0), .TOT(200000), .SID(204)) t_e (.go(go), .done(done[4]));
  dm_tester #(.N(8),  .RG(4), .NVC(4), .X(3),  .Y(3),  .MODE(1), .TOT(120000), .SID(205)) t_s (.go(go), .done(done[5]));
  dm_tester #(.N(8),  .RG(2), .NVC(4), .X(6),  .Y(1),  .MODE(2), .TOT(200000), .SID(206)) t_r (.go(go), .done(done[6]));

  `TB_DUMP(tb_demand_counter)

  initial tb_pkg::watchdog(100ms);

  initial begin
    tb_pkg::start("U12", "demand_counter: occupied input VCs, local and cross-region sums every 16 cycles, saturation, latch at each tick");
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::info("U12 counters: 8x8/4, 4x4/2, 16x16/4, 16x16/8 with 2 VCs, 8x8/4 with 1 VC, a saturation run, a reset run");
    tb_pkg::finish();
  end

endmodule
