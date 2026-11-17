// =============================================================================
// File      tb/unit/tb_sync_fifo.sv                                Tracker: F07
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U02: the FIFO keeps order and count under simultaneous push and pop
// DUT       rtl/common/sync_fifo.sv (F03)
// @tests    U02
// -----------------------------------------------------------------------------
// Eleven FIFOs run side by side against a queue model:
//   depths 1, 2, 3, 4, 5, 8 and 25 with 16-bit data, legal requests only
//     (DUT checks on, as in the router);
//   depth 5 with 134-bit flits: the exact input-VC buffer of the router;
//   depths 1, 5 and 25 with illegal requests too (DUT checks off): a push
//     while full or a pop while empty must change nothing.
// Phases per FIFO:
//   1 Fill and drain twice (pointers wrap), full and empty flags exact
//   2 Full throughput: push and pop together while full, count stays DEPTH
//   3 Streaming: push and pop together with one entry, count stays 1
//   4 Random: 10,000 cycles in four regimes (filling, draining, balanced,
//     busy) so that full and empty are both reached often
// Every cycle: empty, full and count equal the model, and while not empty
// rdata equals the oldest model entry. Pass: no mismatch, every bin hit.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module fifo_tester #(
  parameter int W      = 16,
  parameter int DEPTH  = 4,
  parameter bit LEGAL  = 1'b1,   // 1: legal requests only; 0: illegal ones too
  parameter int CYCLES = 10000,
  parameter int STREAM = 0       // random stream id, unique per tester
) (
  input  logic clk,
  input  logic rst_n,
  input  logic go,
  output logic done
);
  localparam int CW = $clog2(DEPTH + 1);

  logic          push  = 1'b0;
  logic          pop   = 1'b0;
  logic [W-1:0]  wdata = '0;
  logic [W-1:0]  rdata;
  logic          empty, full;
  logic [CW-1:0] count;

  sync_fifo #(.WIDTH(W), .DEPTH(DEPTH), .CHECK(LEGAL)) dut (
    .clk_i   (clk),
    .rst_ni  (rst_n),
    .push_i  (push),
    .wdata_i (wdata),
    .pop_i   (pop),
    .rdata_o (rdata),
    .empty_o (empty),
    .full_o  (full),
    .count_o (count)
  );

  logic [W-1:0] model [$];       // reference: a queue, oldest first
  longint unsigned c_full = 0, c_drained = 0, c_pp_full = 0, c_pp_mid = 0,
                   c_bad_push = 0, c_bad_pop = 0, n_push = 0;

  function automatic logic [W-1:0] rand_word();
    logic [W-1:0] v;
    tb_pkg::rng_t r;
    r = '0;
    for (int b = 0; b < W; b++) begin
      if (b % 64 == 0) r = tb_pkg::rng_next(STREAM);
      v[b] = r[b % 64];
    end
    return v;
  endfunction

  task automatic check_outputs(string ph);
    int sz;
    sz = model.size();
    `TB_CHECK(empty === (sz == 0) && full === (sz == DEPTH) && count === CW'(sz),
      ("D=%0d W=%0d %s: empty=%b full=%b count=%0d, model holds %0d", DEPTH, W, ph, empty, full, count, sz))
    if (sz > 0) begin
      `TB_CHECK(rdata === model[0],
        ("D=%0d W=%0d %s: rdata 'h%0h, oldest entry 'h%0h", DEPTH, W, ph, rdata, model[0]))
    end
  endtask

  // One cycle. Inputs change after the falling edge; outputs are checked;
  // DUT and model step at the rising edge (pop first: the FIFO is first-word
  // fall-through, so a push into a full FIFO uses the entry the pop frees).
  task automatic step(bit pu, bit po, string ph);
    int sz;
    bit do_pop, do_push;
    logic [W-1:0] d;
    d = rand_word();
    @(negedge clk);
    push = pu; pop = po; wdata = d;
    #1;
    check_outputs(ph);
    sz = model.size();
    if (sz == DEPTH) c_full++;
    if (pu && po && sz == DEPTH) c_pp_full++;
    if (pu && po && sz > 0 && sz < DEPTH) c_pp_mid++;
    if (pu && !po && sz == DEPTH) c_bad_push++;
    if (po && sz == 0) c_bad_pop++;
    @(posedge clk);
    do_pop  = po && sz > 0;
    do_push = pu && (sz < DEPTH || do_pop);
    if (do_pop) void'(model.pop_front());
    if (do_push) begin
      model.push_back(d);
      n_push++;
    end
    if (do_pop && model.size() == 0) c_drained++;
    #1;
  endtask

  // ---------------------------------------------------------------------------
  // Phases
  // ---------------------------------------------------------------------------
  task automatic phase_fill_drain();
    for (int rep = 0; rep < 2; rep++) begin
      for (int i = 0; i < DEPTH; i++) step(1'b1, 1'b0, "fill");
      if (!LEGAL) step(1'b1, 1'b0, "push while full");
      for (int i = 0; i < DEPTH; i++) step(1'b0, 1'b1, "drain");
      if (!LEGAL) step(1'b0, 1'b1, "pop while empty");
    end
  endtask

  task automatic phase_full_throughput();
    for (int i = 0; i < DEPTH; i++) step(1'b1, 1'b0, "fill");
    for (int i = 0; i < 2 * DEPTH + 3; i++) step(1'b1, 1'b1, "push and pop while full");
    for (int i = 0; i < DEPTH; i++) step(1'b0, 1'b1, "drain");
  endtask

  task automatic phase_streaming();
    step(1'b1, 1'b0, "stream start");
    for (int i = 0; i < 2 * DEPTH + 3; i++) step(1'b1, 1'b1, "stream");
    step(1'b0, 1'b1, "stream end");
  endtask

  task automatic phase_random();
    real p_push, p_pop;
    bit  pu, po;
    for (int c = 0; c < CYCLES; c++) begin
      case ((c / 250) % 4)
        0:       begin p_push = 0.8; p_pop = 0.2; end   // filling
        1:       begin p_push = 0.2; p_pop = 0.8; end   // draining
        2:       begin p_push = 0.5; p_pop = 0.5; end   // balanced
        default: begin p_push = 0.9; p_pop = 0.9; end   // busy
      endcase
      po = tb_pkg::rng_chance(STREAM, p_pop);
      pu = tb_pkg::rng_chance(STREAM, p_push);
      if (LEGAL) begin
        // what the router guarantees: pop only when not empty, push only when
        // not full unless a pop frees an entry in the same cycle
        po = po && model.size() > 0;
        pu = pu && (model.size() < DEPTH || po);
      end
      step(pu, po, "random");
    end
  endtask

  string tag;     // label for coverage lines

  initial begin
    done = 1'b0;
    if (LEGAL) tag = $sformatf("D=%0d W=%0d legal", DEPTH, W);
    else       tag = $sformatf("D=%0d W=%0d illegal", DEPTH, W);
    wait (go === 1'b1);
    tb_pkg::rng_init(STREAM);
    phase_fill_drain();
    phase_full_throughput();
    phase_streaming();
    phase_random();
    @(negedge clk);              // leave the DUT idle: other testers may still run
    push = 1'b0;
    pop  = 1'b0;
    tb_pkg::cover_bin({tag, ": cycles full"}, c_full);
    tb_pkg::cover_bin({tag, ": drained to empty"}, c_drained);
    tb_pkg::cover_bin({tag, ": push and pop while full"}, c_pp_full);
    if (DEPTH > 1) tb_pkg::cover_bin({tag, ": push and pop part full"}, c_pp_mid);
    tb_pkg::cover_bin({tag, ": pointer wraps"}, n_push / longint'(DEPTH));
    if (!LEGAL) begin
      tb_pkg::cover_bin({tag, ": push while full ignored"}, c_bad_push);
      tb_pkg::cover_bin({tag, ": pop while empty ignored"}, c_bad_pop);
    end
    done = 1'b1;
  end

endmodule


module tb_sync_fifo;
  import noc_pkg::*;

  logic        clk   = 1'b0;
  logic        rst_n = 1'b0;
  logic        go    = 1'b0;
  logic [10:0] done;

  always #5 clk = ~clk;

  // legal requests, DUT checks on
  fifo_tester #(.DEPTH(1),  .STREAM(1))  t_d1  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[0]));
  fifo_tester #(.DEPTH(2),  .STREAM(2))  t_d2  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[1]));
  fifo_tester #(.DEPTH(3),  .STREAM(3))  t_d3  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[2]));
  fifo_tester #(.DEPTH(4),  .STREAM(4))  t_d4  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[3]));
  fifo_tester #(.DEPTH(5),  .STREAM(5))  t_d5  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[4]));
  fifo_tester #(.DEPTH(8),  .STREAM(8))  t_d8  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[5]));
  fifo_tester #(.DEPTH(25), .STREAM(25)) t_d25 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[6]));
  // the router's input-VC buffer: one 5-flit packet of 134-bit flits
  fifo_tester #(.W(FLIT_W), .DEPTH(5), .STREAM(105)) t_vc (.clk(clk), .rst_n(rst_n), .go(go), .done(done[7]));
  // illegal requests too, DUT checks off
  fifo_tester #(.DEPTH(1),  .LEGAL(1'b0), .STREAM(201)) t_i1  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[8]));
  fifo_tester #(.DEPTH(5),  .LEGAL(1'b0), .STREAM(205)) t_i5  (.clk(clk), .rst_n(rst_n), .go(go), .done(done[9]));
  fifo_tester #(.DEPTH(25), .LEGAL(1'b0), .STREAM(225)) t_i25 (.clk(clk), .rst_n(rst_n), .go(go), .done(done[10]));

  `TB_DUMP(tb_sync_fifo)

  initial tb_pkg::watchdog(10ms);

  initial begin
    tb_pkg::start("U02", "sync_fifo: order and count under simultaneous push and pop");
    repeat (3) @(posedge clk);
    @(negedge clk);
    rst_n = 1'b1;
    go    = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
