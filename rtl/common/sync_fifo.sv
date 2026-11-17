// =============================================================================
// File      rtl/common/sync_fifo.sv                                Tracker: F03
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Single-clock FIFO with an occupancy count. One per input VC holds
//           one packet (D05); the NI uses deeper ones (ejection buffer, D14).
// Tests     U02 (tb/unit/tb_sync_fifo.sv)
// -----------------------------------------------------------------------------
// Behaviour
//   First-word fall-through: while empty_o is 0, rdata_o shows the oldest
//   entry, so the reader sees a head flit without spending a read cycle.
//   A push is written at the clock edge and is visible from the next cycle.
//   push_i and pop_i may come in the same cycle at any fill level; when the
//   FIFO is full, the pop frees the entry the push needs.
//   Illegal requests (push while full without a pop, pop while empty) change
//   nothing. With CHECK = 1 they stop the simulation, because in the router
//   they can only come from a credit or allocation bug. So does an X on
//   push_i or pop_i.
//   rdata_o is undefined while empty_o is 1 (X in four-state simulators):
//   gate every use of it with !empty_o.
//   Any DEPTH from 1 up; it need not be a power of two (a VC holds 5 flits).
//   The storage has no reset, like a register file; only the pointers and the
//   count do. This keeps buffer area comparable with [1] (Fig. 18).
// =============================================================================
`timescale 1ns/1ps

module sync_fifo #(
  parameter int WIDTH = 8,
  parameter int DEPTH = 4,
  parameter bit CHECK = 1'b1,                            // simulation checks on
  parameter int PW    = (DEPTH > 1) ? $clog2(DEPTH) : 1, // pointer width: derived
  parameter int CW    = $clog2(DEPTH + 1)                // count width: derived
) (
  input  logic             clk_i,
  input  logic             rst_ni,     // asynchronous, active low
  input  logic             push_i,
  input  logic [WIDTH-1:0] wdata_i,
  input  logic             pop_i,
  output logic [WIDTH-1:0] rdata_o,    // oldest entry; valid while empty_o is 0
  output logic             empty_o,
  output logic             full_o,
  output logic [CW-1:0]    count_o     // entries held, 0..DEPTH
);

  logic [WIDTH-1:0] mem_q [DEPTH];
  logic [PW-1:0]    rd_q, wr_q;
  logic [CW-1:0]    cnt_q;
  logic             do_push, do_pop;

  assign empty_o = (cnt_q == '0);
  assign full_o  = (cnt_q == CW'(DEPTH));
  assign count_o = cnt_q;
  assign rdata_o = mem_q[rd_q];

  assign do_pop  = pop_i & ~empty_o;
  assign do_push = push_i & (~full_o | do_pop);

  // Pointers wrap at DEPTH, which need not be a power of two
  function automatic logic [PW-1:0] next_ptr(logic [PW-1:0] p);
    return (p == PW'(DEPTH - 1)) ? '0 : p + PW'(1);
  endfunction

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      rd_q  <= '0;
      wr_q  <= '0;
      cnt_q <= '0;
    end else begin
      if (do_push) wr_q <= next_ptr(wr_q);
      if (do_pop)  rd_q <= next_ptr(rd_q);
      if (do_push && !do_pop)      cnt_q <= cnt_q + CW'(1);
      else if (do_pop && !do_push) cnt_q <= cnt_q - CW'(1);
    end
  end

  always_ff @(posedge clk_i) begin
    if (do_push) mem_q[wr_q] <= wdata_i;
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  // Simulation-only checks, evaluated at every clock edge out of reset
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else if (CHECK) begin
      if ($isunknown({push_i, pop_i}))
        $fatal(1, "%m: X on push_i or pop_i");
      if (push_i && full_o && !pop_i)
        $fatal(1, "%m: push while full (credit or allocation bug)");
      if (pop_i && empty_o)
        $fatal(1, "%m: pop while empty (allocation bug)");
    end
  end
  // synopsys translate_on
`endif

endmodule
