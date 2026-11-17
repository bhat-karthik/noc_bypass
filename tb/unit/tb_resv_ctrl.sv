// =============================================================================
// File      tb/unit/tb_resv_ctrl.sv                                Tracker: F48
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U10: the reservation rules of resv_ctrl (D12, D48) against a
//           reference model, cycle by cycle: reserve one subslot ahead,
//           re-check before every subslot, cancel when short, shrink at the
//           lookahead, release no-shows, never exceed the bypass space
// DUT       rtl/ni/resv_ctrl.sv (F45)
// Reference this file: the decision rule and the two counts, from D48's text
// @tests    U10
// -----------------------------------------------------------------------------
// Six testers, each with its own timetable shape (mesh side, B_MAX, TIGHT or
// SAFE; one with the smallest legal space, B_MAX), run at the same time.
// Each drives the DUT as a tile and an NI would (D38):
//   decisions   at the first cycle of every setup window (subslot 0) and of
//               every subslot but the last, for the next subslot; one in ten
//               has no partner (as subslot 0 of a same-group slot); a random
//               source, mode and path length L = 1..Hmax
//   bursts      planned at the decision: B = 0 (a no-show) .. B_MAX flits,
//               lookahead at t + 2L, flits written at t + 2L + 3 + j, no two
//               bursts writing in one cycle (the NI port, U09); a cancelled
//               burst is never sent
//   the core    takes a bypass flit whenever one is there and it is ready:
//               always ready, 25% stalls, 75% stalls, a full stall of eight
//               slots (300 cycles at least), then always ready again; each
//               other phase lasts 40 slots (+QUICK: 12), 2,000 cycles (600)
//               at least
// Every cycle: the DUT's reserved and filled counts equal the model's;
// fill + res <= SPACE; at a decision the DUT cancels exactly when the model
// does, with the token {source, the other mode}; no token otherwise. While
// the core keeps up (from reset, and after the stall once the FIFO is down
// to one flit), no turn may be cancelled (D48's sizing; not checked for the
// tester with the smallest space). Every rule must occur: accept, cancel,
// no-show, shrink, two reservations waiting (impossible with the smallest
// space), accept and cancel of subslot 0, a cancel caused by a stalled
// core, fill + res reaching SPACE - B_MAX + 1 or more.
// +QUICK: 12 slots per phase instead of 40 (EDA Playground).
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module resv_tester
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int N      = 4,
  parameter int BM     = 10,
  parameter int TMODE  = TM_TIGHT,
  parameter int SMALL  = 0,          // 1: SPACE = BM, the smallest legal bypass space
  parameter int SID    = 700
) (
  input  logic go,
  output logic done
);
  localparam int S     = s_setup(N);
  localparam int T     = t_sub(N, BM, TMODE);
  localparam int HM    = hmax(N);
  localparam int SPACE = (SMALL != 0) ? BM : byp_space(N, BM, TMODE);
  localparam int LEN   = slot_len(N, T);
  localparam int RING  = 512;
  localparam int RB    = $clog2(RING);
  localparam int CW    = $clog2(SPACE + 1);

  logic          clk   = 1'b0;
  logic          rst_n = 1'b0;
  logic          dec = 1'b0, dec_s0 = 1'b0, dec_mode = 1'b0, push = 1'b0, pop = 1'b0;
  node_id_t      dec_src = '0;
  hop_t          dec_hops = '0;
  la_t           la = '0;
  cx_t           cx;
  logic [CW-1:0] res_o, fill_o;

  resv_ctrl #(.B_MAX(BM), .SPACE(SPACE), .GAP0(S), .GAPK(T)) u_dut (
    .clk_i      (clk),
    .rst_ni     (rst_n),
    .dec_i      (dec),
    .dec_s0_i   (dec_s0),
    .dec_loc_i  (1'b0),                  // design B's timetable (designs C, D: TL tests)
    .dec_src_i  (dec_src),
    .dec_mode_i (dec_mode),
    .dec_hops_i (dec_hops),
    .la_i       (la),
    .push_i     (push),
    .pop_i      (pop),
    .cx_o       (cx),
    .res_o      (res_o),
    .fill_o     (fill_o)
  );

  always #5 clk = ~clk;

  // ---------------------------------------------------------------- plan and model
  int          c;                      // cycles since reset rose
  int          la_at   [RING];         // lookahead length due at a cycle (-1: none)
  bit          push_at [RING];         // a burst flit written at a cycle
  int          due_at  [RING];         // the model's release: burst length (-1: nothing due)
  int          res_m, fill_m;          // the model
  // the decision driven in the current cycle (planned at the edge before)
  bit          p_dec, p_s0, p_mode;
  int          p_src, p_l, p_b, p_t;
  int unsigned slots_ph, phase, ph_end, ph_len, st_len;
  bit          rdy;
  bit          kept_up;               // the core is always ready and the FIFO is down to one flit
  // coverage
  longint unsigned n_dec, n_acc, n_cx, n_cx0, n_acc0, n_nosh, n_shr, n_two, n_cx_stall, n_ready_cx, n_near;
  int          max_sum;
  string       tmode_s;

  // position in the timetable of cycle cc: a decision for which subslot
  function automatic bit is_dec(int cc, output bit s0, output int t);
    int p, k;
    p  = cc % LEN;
    s0 = 1'b0;
    t  = 0;
    if (p == 0) begin
      s0 = 1'b1;
      t  = cc + S;
      return 1'b1;
    end
    if (p < S) return 1'b0;
    k = (p - S) / T;
    if ((p - S) % T == 0 && k < N - 1) begin
      t = cc + T;
      return 1'b1;
    end
    return 1'b0;
  endfunction

  // plan the decision of cycle cc (called at the edge before it)
  function automatic void plan(int cc);
    bit s0, clash;
    int t, l, b, l0;
    p_dec = 1'b0;
    if (!is_dec(cc, s0, t)) return;
    if (tb_pkg::rng_below(SID, 10) == 0) return;              // no partner in that subslot
    p_dec  = 1'b1;
    p_s0   = s0;
    p_t    = t;
    p_src  = int'(tb_pkg::rng_below(SID, 256));
    p_mode = tb_pkg::rng_chance(SID, 0.5);
    b      = int'(tb_pkg::rng_below(SID, BM + 2));            // 0 .. BM + 1
    if (b > BM) b = BM;                                        // full bursts more often
    if (tb_pkg::rng_chance(SID, 0.25)) b = 0;                  // a no-show
    l0     = 1 + int'(tb_pkg::rng_below(SID, HM));
    // the burst's flits must not meet another burst's at the NI port
    p_l = l0;
    p_b = b;
    if (b > 0) begin
      p_b = 0;
      for (int k = 0; k < HM && p_b == 0; k++) begin
        l = 1 + (l0 - 1 + k) % HM;
        clash = 1'b0;
        for (int j = 0; j < b; j++) if (push_at[(t + 2 * l + 3 + j) % RING]) clash = 1'b1;
        if (!clash) begin
          p_l = l;
          p_b = b;
        end
      end
    end
  endfunction

  initial begin
    for (int i = 0; i < RING; i++) begin
      la_at[i]   = -1;
      push_at[i] = 1'b0;
      due_at[i]  = -1;
    end
    res_m = 0; fill_m = 0; c = 0; phase = 0;
    n_dec = 0; n_acc = 0; n_cx = 0; n_cx0 = 0; n_acc0 = 0; n_nosh = 0; n_shr = 0; n_two = 0;
    n_cx_stall = 0; n_ready_cx = 0; n_near = 0; max_sum = 0;
    done = 1'b0;
    slots_ph = $test$plusargs("QUICK") ? 12 : 40;
    rdy = 1'b1;
    kept_up = 1'b1;
    wait (go);
    repeat (3) @(posedge clk);
    @(negedge clk) rst_n = 1'b1;           // cycle 0: the cycle in which reset rises
    ph_len = slots_ph * LEN;
    if (ph_len < 50 * slots_ph) ph_len = 50 * slots_ph;           // small timetables: phases of 600 / 2,000 cycles at least
    st_len = 8 * LEN;
    if (st_len < 300) st_len = 300;
    ph_end = ph_len;
    forever begin
      // ---- the falling edge inside cycle c
      logic [RB-1:0] slot_c;
      bit            acc, waiting;
      slot_c = RB'(c % RING);
      // 1. the registered counts: the state at the start of cycle c
      tb_pkg::n_checks++;
      if (int'(res_o) != res_m || int'(fill_o) != fill_m)
        tb_pkg::fail($sformatf("U10 N=%0d B_MAX=%0d SPACE=%0d cycle %0d: DUT reserved %0d filled %0d, model %0d and %0d",
                               N, BM, SPACE, c, res_o, fill_o, res_m, fill_m));
      `TB_CHECK(fill_m + res_m <= SPACE && fill_m >= 0 && res_m >= 0,
                ("U10 N=%0d cycle %0d: fill %0d + reserved %0d, space %0d", N, c, fill_m, res_m, SPACE))
      if (fill_m + res_m > max_sum) max_sum = fill_m + res_m;
      // 2. the inputs of cycle c (the DUT samples them at the edge that ends it)
      plan(c);
      case (phase)
        0, 4:    rdy = 1'b1;
        1:       rdy = !tb_pkg::rng_chance(SID + 1, 0.25);
        2:       rdy = !tb_pkg::rng_chance(SID + 1, 0.75);
        default: rdy = 1'b0;
      endcase
      dec      = p_dec;
      dec_s0   = p_s0;
      dec_src  = node_id_t'(p_src);
      dec_mode = p_mode;
      dec_hops = hop_t'(p_l);
      la       = '0;
      la.vld   = (la_at[slot_c] > 0);
      la.len   = BURST_W'((la_at[slot_c] > 0) ? la_at[slot_c] : 0);
      push     = push_at[slot_c];
      pop      = rdy && (fill_m > 0);
      #1;
      // 3. the decision against the model
      if (dec) begin
        acc     = fill_m + res_m + BM <= SPACE;
        waiting = 1'b0;
        for (int i = 1; i < S + T + 2 * HM + 2; i++) if (due_at[(c + i) % RING] >= 0) waiting = 1'b1;
        n_dec++;
        `TB_CHECK(cx.vld == !acc, ("U10 N=%0d cycle %0d: fill %0d reserved %0d: the DUT cancels %0b, the model accepts %0b", N, c,
                                   fill_m, res_m, cx.vld, acc))
        if (cx.vld) begin
          `TB_CHECK(int'(cx.tgt) == p_src && cx.mode == !p_mode,
                    ("U10 N=%0d cycle %0d: token for R%0d mode %0d, expected R%0d mode %0d", N, c, cx.tgt, cx.mode, p_src, !p_mode))
        end
        if (acc) begin
          n_acc++;
          if (p_s0) n_acc0++;
          if (waiting) n_two++;
          res_m += BM;
          due_at[(p_t + 2 * p_l) % RING] = p_b;
          if (p_b > 0) begin
            la_at[(p_t + 2 * p_l) % RING] = p_b;
            for (int j = 0; j < p_b; j++) push_at[(p_t + 2 * p_l + 3 + j) % RING] = 1'b1;
          end
        end else begin
          n_cx++;
          if (p_s0) n_cx0++;
          if (phase >= 1 && phase <= 3) n_cx_stall++;
          if (kept_up) n_ready_cx++;
        end
        if (fill_m + res_m + BM > SPACE - BM) n_near++;
      end else begin
        `TB_CHECK(!cx.vld, ("U10 N=%0d cycle %0d: a cancel token with no decision", N, c))
      end
      // 4. the cycle's other events, as the DUT applies them at its end
      if (due_at[slot_c] >= 0) begin
        if (due_at[slot_c] == 0) begin
          res_m -= BM;
          n_nosh++;
        end else begin
          res_m -= BM - due_at[slot_c];
          n_shr++;
        end
        due_at[slot_c] = -1;
      end
      if (push) begin
        fill_m++;
        res_m--;
      end
      if (pop) fill_m--;
      // D48's claim holds from reset while the core keeps up, and again after
      // a stall once the FIFO is down to one flit (it then stays there)
      if (phase == 0)                 kept_up = 1'b1;
      else if (phase != 4)            kept_up = 1'b0;
      else if (fill_m <= 1)           kept_up = 1'b1;
      push_at[slot_c] = 1'b0;
      la_at[slot_c]   = -1;
      // the core's phase
      if (c + 1 >= int'(ph_end)) begin
        phase++;
        if (phase == 3) ph_end = ph_end + st_len;             // the full stall: eight slots, 300 cycles at least
        else            ph_end = ph_end + ph_len;
        if (phase > 4) break;
      end
      @(negedge clk);
      c = c + 1;
    end
    // finished: the DUT back into reset while the other testers run
    dec   = 1'b0;
    la    = '0;
    push  = 1'b0;
    pop   = 1'b0;
    rst_n = 1'b0;
    // every rule exercised
    `TB_CHECK(n_acc > 0 && n_cx > 0 && n_nosh > 0 && n_shr > 0 && (n_two > 0 || SMALL != 0) && n_acc0 > 0 && n_cx0 > 0 &&
              n_cx_stall > 0 && max_sum >= SPACE - BM + 1,
              ("U10 N=%0d B_MAX=%0d SPACE=%0d: a rule never occurred: accepted %0d, cancelled %0d, no-shows %0d, shrunk %0d, two waiting %0d, subslot 0 accepted %0d cancelled %0d, cancelled while stalled %0d, largest fill + reserved %0d",
               N, BM, SPACE, n_acc, n_cx, n_nosh, n_shr, n_two, n_acc0, n_cx0, n_cx_stall, max_sum))
    if (SMALL == 0) begin
      `TB_CHECK(n_ready_cx == 0, ("U10 N=%0d B_MAX=%0d: %0d turns cancelled while the core was always ready (D48 sizing)", N, BM, n_ready_cx))
    end
    if (TMODE == TM_TIGHT) tmode_s = "TIGHT";
    else                   tmode_s = "SAFE";
    tb_pkg::info($sformatf("U10 N=%0d B_MAX=%0d %s SPACE=%0d: %0d cycles, %0d decisions: %0d accepted (%0d of subslot 0, %0d while another waited), %0d cancelled (%0d while the core stalled, %0d while it kept up); %0d no-shows, %0d shrunk at the lookahead; largest fill + reserved %0d",
                           N, BM, tmode_s, SPACE, c, n_dec, n_acc, n_acc0, n_two, n_cx, n_cx_stall, n_ready_cx, n_nosh, n_shr, max_sum));
    tb_pkg::cover_bin($sformatf("U10 N=%0d B_MAX=%0d SPACE=%0d cancels", N, BM, SPACE), n_cx);
    done = 1'b1;
  end
endmodule

module tb_resv_ctrl;
  import sched_pkg::*;
  logic go = 1'b0;
  logic [5:0] dn;
  resv_tester #(.N(4),  .BM(10), .TMODE(TM_TIGHT), .SMALL(0), .SID(701)) u_t1 (.go(go), .done(dn[0]));
  resv_tester #(.N(8),  .BM(15), .TMODE(TM_TIGHT), .SMALL(0), .SID(703)) u_t2 (.go(go), .done(dn[1]));
  resv_tester #(.N(2),  .BM(5),  .TMODE(TM_TIGHT), .SMALL(0), .SID(705)) u_t3 (.go(go), .done(dn[2]));
  resv_tester #(.N(4),  .BM(10), .TMODE(TM_SAFE),  .SMALL(0), .SID(707)) u_t4 (.go(go), .done(dn[3]));
  resv_tester #(.N(16), .BM(10), .TMODE(TM_TIGHT), .SMALL(0), .SID(709)) u_t5 (.go(go), .done(dn[4]));
  resv_tester #(.N(4),  .BM(10), .TMODE(TM_TIGHT), .SMALL(1), .SID(711)) u_t6 (.go(go), .done(dn[5]));

  initial begin
    tb_pkg::start("U10", "resv_ctrl: reserve, re-check, cancel, shrink, release (six timetable shapes)");
    for (int i = 701; i <= 712; i++) tb_pkg::rng_init(i);
    go = 1'b1;
    wait (&dn);
    tb_pkg::finish();
  end

  initial tb_pkg::watchdog(64'd200_000_000);

  `TB_DUMP(tb_resv_ctrl)
endmodule
