// =============================================================================
// File      rtl/common/rr_arbiter.sv                               Tracker: F02
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Round-robin arbiter with a request mask and grant hold. Used by the
//           switch allocator (input and output stages), VC choice and the NI.
// Tests     U01 (tb/unit/tb_rr_arbiter.sv)
// -----------------------------------------------------------------------------
// Behaviour
//   eligible  = req_i & ~mask_i   (mask_i: requester blocked this cycle,
//               e.g. no credit downstream)
//   hold      If hold_i is 1 and the last accepted winner is still eligible,
//             it wins again. Keeps a resource for a whole packet or burst.
//   otherwise The first eligible requester at or after the pointer wins,
//             searching upward and wrapping around.
//   update_i  The caller used the grant: the pointer moves to winner + 1.
//             Without update_i the state does not change, so a requester
//             that loses a later stage (the output arbiter of a separable
//             allocator) keeps its priority for the next cycle.
//   Grants are combinational from the inputs and the state.
//   Fairness  With hold_i at 0 and update_i on every grant, an eligible
//             requester waits for at most N-1 grants to others.
// Implementation
//   Two "lowest set bit" pickers: one over the eligible requests at or above
//   the pointer, one over all eligible requests. When the first finds nothing,
//   the second gives the wrap-around winner. x & -x keeps only the lowest set
//   bit of x (two's complement), so the logic is O(N) with no priority chain
//   built from a loop.
// =============================================================================
`timescale 1ns/1ps

module rr_arbiter #(
  parameter int N  = 4,                        // requesters, 1 or more
  parameter int IW = (N > 1) ? $clog2(N) : 1   // index width: derived, do not set
) (
  input  logic          clk_i,
  input  logic          rst_ni,      // asynchronous, active low
  input  logic [N-1:0]  req_i,
  input  logic [N-1:0]  mask_i,      // 1 = requester may not win this cycle
  input  logic          hold_i,      // 1 = keep the last accepted winner if eligible
  input  logic          update_i,    // 1 = grant accepted: move priority past the winner
  output logic [N-1:0]  gnt_o,       // one-hot; zero when nobody is eligible
  output logic [IW-1:0] gnt_idx_o,   // index of the winner; 0 when gnt_vld_o is 0
  output logic          gnt_vld_o    // a requester was granted
);

  logic [IW-1:0] ptr_q;        // requester with the highest priority
  logic [IW-1:0] last_q;       // last accepted winner
  logic          last_vld_q;   // last_q holds a real winner (0 after reset)

  logic [N-1:0] elig;          // eligible requests
  logic [N-1:0] elig_hi;       // eligible requests at or above the pointer
  logic [N-1:0] pick_hi;       // lowest of elig_hi
  logic [N-1:0] pick_all;      // lowest of elig: the wrap-around winner
  logic [N-1:0] last_oh;       // last_q as a one-hot vector
  logic         hold_hit;      // the hold rule decides this cycle

  assign elig     = req_i & ~mask_i;
  assign elig_hi  = elig & ({N{1'b1}} << ptr_q);
  assign pick_hi  = elig_hi & (-elig_hi);
  assign pick_all = elig & (-elig);

  always_comb begin
    last_oh         = '0;
    last_oh[last_q] = 1'b1;
  end

  assign hold_hit  = hold_i & last_vld_q & elig[last_q];
  assign gnt_o     = hold_hit ? last_oh : ((|elig_hi) ? pick_hi : pick_all);
  assign gnt_vld_o = |elig;

  // One-hot to index: OR together the indices of the set bits (one at most)
  always_comb begin
    gnt_idx_o = '0;
    for (int i = 0; i < N; i++) begin
      if (gnt_o[i]) gnt_idx_o = gnt_idx_o | IW'(i);
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ptr_q      <= '0;
      last_q     <= '0;
      last_vld_q <= 1'b0;
    end else if (update_i && gnt_vld_o) begin
      ptr_q      <= (gnt_idx_o == IW'(N - 1)) ? '0 : gnt_idx_o + IW'(1);
      last_q     <= gnt_idx_o;
      last_vld_q <= 1'b1;
    end
  end

endmodule
