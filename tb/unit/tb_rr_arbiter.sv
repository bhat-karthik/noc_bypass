// =============================================================================
// File      tb/unit/tb_rr_arbiter.sv                               Tracker: F06
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U01: the arbiter grants one requester, respects the mask, rotates
//           priority and holds a grant
// DUT       rtl/common/rr_arbiter.sv (F02)
// @tests    U01
// -----------------------------------------------------------------------------
// Ten arbiters run side by side, N = 1..8, 12 and 20 (20 = 5 ports x 4 VCs).
// Each has its own reference model and random stream.
//   1 Exhaustive  N <= 8. The arbiter can only be in N+1 states: just after
//                 reset, or "pointer at p" for each p (the last winner is then
//                 always p-1). Every state is driven with every eligible
//                 pattern and hold on and off: all requests x all masks for
//                 N <= 5, every eligible pattern plus random masked-out
//                 requests for N = 6..8. This is a proof by exhaustion.
//   2 Rotation    Everyone requests: grants visit p, p+1, ... in order.
//   3 Hold        The winner keeps the grant while it requests and is not
//                 masked; then priority moves on.
//   4 Random      10,000 cycles (20,000 for N = 20) of random requests, masks,
//                 hold and update, compared with the model every cycle, with
//                 the round-robin bound: an eligible requester is passed over
//                 at most N-1 times.
// Pass: no mismatch and every coverage bin hit.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module arb_tester #(
  parameter int N          = 4,
  parameter bit EXHAUSTIVE = 1'b1,
  parameter int CYCLES     = 10000
) (
  input  logic clk,
  input  logic rst_n,
  input  logic go,
  output logic done
);
  localparam int IW = (N > 1) ? $clog2(N) : 1;
  localparam logic [N-1:0] ALL = {N{1'b1}};

  logic [N-1:0]  req    = '0;
  logic [N-1:0]  mask   = '0;
  logic          hold   = 1'b0;
  logic          update = 1'b0;
  logic [N-1:0]  gnt;
  logic [IW-1:0] gidx;
  logic          gvld;

  rr_arbiter #(.N(N)) dut (
    .clk_i    (clk),
    .rst_ni   (rst_n),
    .req_i    (req),
    .mask_i   (mask),
    .hold_i   (hold),
    .update_i (update),
    .gnt_o    (gnt),
    .gnt_idx_o(gidx),
    .gnt_vld_o(gvld)
  );

  // ---------------------------------------------------------------------------
  // Reference model: the specification written as plainly as possible
  // ---------------------------------------------------------------------------
  int m_ptr      = 0;      // highest-priority requester
  int m_last     = 0;      // last accepted winner
  bit m_last_vld = 1'b0;

  int sid;                 // random stream of this tester

  // Winner for these inputs and the model state; -1 when nobody is eligible
  function automatic int model_grant(logic [N-1:0] r, logic [N-1:0] m, logic h);
    logic [N-1:0] e;
    int g;
    e = r & ~m;
    g = -1;
    if (h && m_last_vld && e[m_last]) g = m_last;
    else begin
      for (int k = 0; k < N; k++) begin
        if (g < 0 && e[(m_ptr + k) % N]) g = (m_ptr + k) % N;
      end
    end
    return g;
  endfunction

  // ---------------------------------------------------------------------------
  // Coverage and fairness bookkeeping
  // ---------------------------------------------------------------------------
  longint unsigned c_hold = 0, c_mask = 0, c_wrap = 0, c_idle = 0, c_upd = 0;
  int wait_cnt [N];
  int wait_max = 0;
  int seen_idx;            // DUT grant index seen in the last check
  bit seen_vld;

  // Compare every output with the model for the inputs now applied
  task automatic check_outputs(string ph);
    int            g;
    logic [N-1:0]  exp_oh;
    logic [IW-1:0] exp_idx;
    logic [N-1:0]  e;
    g       = model_grant(req, mask, hold);
    exp_oh  = '0;
    exp_idx = '0;
    if (g >= 0) begin
      exp_oh[g] = 1'b1;
      exp_idx   = IW'(g);
    end
    `TB_CHECK(gnt === exp_oh && gidx === exp_idx && gvld === (g >= 0),
      ("N=%0d %s: req=%b mask=%b hold=%0b (ptr %0d, last %0d/%0b): got gnt=%b idx=%0d vld=%b, expected winner %0d",
       N, ph, req, mask, hold, m_ptr, m_last, m_last_vld, gnt, gidx, gvld, g))
    seen_idx = int'(gidx);
    seen_vld = gvld;
    // coverage
    e = req & ~mask;
    if (g < 0) c_idle++;
    else if (hold && m_last_vld && e[m_last]) c_hold++;
    else if (g < m_ptr) c_wrap++;
    if (model_grant(req, '0, hold) != g) c_mask++;
  endtask

  // Model state update at the clock edge, plus the round-robin bound
  task automatic model_clock();
    int           g;
    logic [N-1:0] e;
    bit           hh;
    e  = req & ~mask;
    g  = model_grant(req, mask, hold);
    hh = hold && m_last_vld && e[m_last];
    for (int i = 0; i < N; i++) begin
      if (!e[i] || hh) wait_cnt[i] = 0;
      else if (update && g >= 0) begin
        if (g == i) wait_cnt[i] = 0;
        else begin
          wait_cnt[i]++;
          if (wait_cnt[i] > wait_max) wait_max = wait_cnt[i];
          `TB_CHECK(wait_cnt[i] <= N - 1,
            ("N=%0d: requester %0d passed over %0d times in a row", N, i, wait_cnt[i]))
        end
      end
    end
    if (update && g >= 0) begin
      m_ptr      = (g + 1) % N;
      m_last     = g;
      m_last_vld = 1'b1;
      c_upd++;
    end
  endtask

  // One clock cycle: inputs change after the falling edge, outputs are checked,
  // DUT and model step at the rising edge. Returns 1 ns after the rising edge.
  task automatic step(logic [N-1:0] r, logic [N-1:0] m, logic h, logic u, string ph);
    @(negedge clk);
    req = r; mask = m; hold = h; update = u;
    #1;
    check_outputs(ph);
    @(posedge clk);
    model_clock();
    #1;
  endtask

  // Combinational probe: update stays 0, so the state cannot change
  task automatic probe(logic [N-1:0] r, logic [N-1:0] m, logic h);
    req = r; mask = m; hold = h; update = 1'b0;
    #1;
    check_outputs("exhaustive");
  endtask

  // Give requester p the highest priority: grant p-1 alone and accept it
  task automatic steer(int p);
    logic [N-1:0] oh;
    oh = '0;
    oh[(p + N - 1) % N] = 1'b1;
    step(oh, '0, 1'b0, 1'b1, "steer");
  endtask

  function automatic logic [N-1:0] rand_bits(real p);
    logic [N-1:0] v;
    for (int i = 0; i < N; i++) v[i] = tb_pkg::rng_chance(sid, p);
    return v;
  endfunction

  // ---------------------------------------------------------------------------
  // Phases
  // ---------------------------------------------------------------------------
  task automatic phase_exhaustive();
    logic [N-1:0] x, y;
    for (int s = -1; s < N; s++) begin          // s = -1: the state right after reset
      if (s >= 0) steer(s);
      for (int h = 0; h < 2; h++) begin
        if (N <= 5) begin
          for (int r = 0; r < (1 << N); r++)
            for (int m = 0; m < (1 << N); m++)
              probe(N'(r), N'(m), h[0]);
        end else begin
          for (int e = 0; e < (1 << N); e++)
            for (int k = 0; k < 4; k++) begin
              // x: requests that the mask removes; y: mask bits with no request
              if (k == 0) begin
                x = '0;
                y = '0;
              end else begin
                x = rand_bits(0.5) & ~N'(e);
                y = rand_bits(0.5) & ~N'(e) & ~x;
              end
              probe(N'(e) | x, x | y, h[0]);
            end
        end
      end
    end
  endtask

  task automatic phase_rotation();
    int p;
    p = int'(tb_pkg::rng_below(sid, N));
    steer(p);
    for (int k = 0; k < 3 * N; k++) begin
      step(ALL, '0, 1'b0, 1'b1, "rotation");
      `TB_CHECK(seen_vld && seen_idx == (p + k) % N,
        ("N=%0d rotation: grant %0d, expected %0d", N, seen_idx, (p + k) % N))
    end
  endtask

  task automatic phase_hold();
    int           j;
    logic [N-1:0] m;
    j = int'(tb_pkg::rng_below(sid, N));
    steer(j);
    step(ALL, '0, 1'b0, 1'b1, "hold");           // j wins and is accepted
    for (int k = 0; k < 4; k++) begin
      step(ALL, '0, 1'b1, 1'b1, "hold");         // hold: j keeps winning
      `TB_CHECK(seen_vld && seen_idx == j, ("N=%0d hold: grant %0d, expected %0d", N, seen_idx, j))
    end
    if (N > 1) begin
      m    = '0;
      m[j] = 1'b1;
      step(ALL, m, 1'b1, 1'b0, "hold, holder masked");   // a masked holder loses the hold
      `TB_CHECK(seen_vld && seen_idx == (j + 1) % N,
        ("N=%0d hold with holder masked: grant %0d, expected %0d", N, seen_idx, (j + 1) % N))
      step(ALL & ~m, '0, 1'b1, 1'b1, "hold, holder gone"); // the holder stops requesting
      `TB_CHECK(seen_vld && seen_idx == (j + 1) % N,
        ("N=%0d hold released: grant %0d, expected %0d", N, seen_idx, (j + 1) % N))
    end
  endtask

  task automatic phase_random();
    real          p_req;
    logic [N-1:0] r_v, m_v;
    logic         h_v, u_v;
    for (int c = 0; c < CYCLES; c++) begin
      case ((c / 500) % 3)                        // sparse, medium, dense requests
        0:       p_req = 0.15;
        1:       p_req = 0.5;
        default: p_req = 0.9;
      endcase
      // one draw per statement: simulators may evaluate call arguments in any order
      r_v = rand_bits(p_req);
      m_v = rand_bits(0.25);
      h_v = tb_pkg::rng_chance(sid, 0.3);
      u_v = tb_pkg::rng_chance(sid, 0.75);
      step(r_v, m_v, h_v, u_v, "random");
    end
  endtask

  initial begin
    done = 1'b0;
    for (int i = 0; i < N; i++) wait_cnt[i] = 0;
    wait (go === 1'b1);
    sid = N;
    tb_pkg::rng_init(sid);
    if (EXHAUSTIVE) phase_exhaustive();
    phase_rotation();
    phase_hold();
    phase_random();
    @(negedge clk);              // leave the DUT idle: other testers may still run
    req    = '0;
    hold   = 1'b0;
    update = 1'b0;
    tb_pkg::cover_bin($sformatf("N=%0d hold decides the grant", N), c_hold);
    tb_pkg::cover_bin($sformatf("N=%0d mask changes the winner", N), c_mask);
    if (N > 1) tb_pkg::cover_bin($sformatf("N=%0d grant wraps around past N-1", N), c_wrap);
    tb_pkg::cover_bin($sformatf("N=%0d nobody eligible", N), c_idle);
    tb_pkg::cover_bin($sformatf("N=%0d grants accepted", N), c_upd);
    tb_pkg::info($sformatf("N=%0d longest run of grants to others: %0d (bound %0d)", N, wait_max, N - 1));
    done = 1'b1;
  end

endmodule


module tb_rr_arbiter;
  logic       clk   = 1'b0;
  logic       rst_n = 1'b0;
  logic       go    = 1'b0;
  logic [9:0] done;

  always #5 clk = ~clk;

  arb_tester #(.N(1))                                  t_n1  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[0]));
  arb_tester #(.N(2))                                  t_n2  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[1]));
  arb_tester #(.N(3))                                  t_n3  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[2]));
  arb_tester #(.N(4))                                  t_n4  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[3]));
  arb_tester #(.N(5))                                  t_n5  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[4]));
  arb_tester #(.N(6))                                  t_n6  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[5]));
  arb_tester #(.N(7))                                  t_n7  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[6]));
  arb_tester #(.N(8))                                  t_n8  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[7]));
  arb_tester #(.N(12), .EXHAUSTIVE(1'b0))              t_n12 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[8]));
  arb_tester #(.N(20), .EXHAUSTIVE(1'b0), .CYCLES(20000)) t_n20 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[9]));

  `TB_DUMP(tb_rr_arbiter)

  initial tb_pkg::watchdog(10ms);

  initial begin
    tb_pkg::start("U01", "rr_arbiter: one grant, mask, rotation, hold");
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    go    = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
