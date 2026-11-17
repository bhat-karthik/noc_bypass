// =============================================================================
// File      rtl/sched/slot_table.sv                                Tracker: F33
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The lookup table S0..S(n-1) of a router and its compare [1, Fig. 10]:
//           S_t holds the slot in which the router's group meets group t, the
//           router's column of Table II. The current slot is compared with
//           every entry; a match makes the router active and names the other
//           group of the slot.
// Decisions D08 groups, table, partner (the table is filled at reset)
// Tests     U05 (tb/unit/tb_slot_table.sv)
// -----------------------------------------------------------------------------
// Example   4 x 4, group 0: S0..S3 = {0, 1, 3, 6}, the first column of
//           Table II [1, Sec. V]. In slot 3 the router is active with group 2.
// Reset     The table is loaded from grp_i while rst_ni is low, so reset must
//           last at least one clock edge (every testbench and the mesh hold it
//           for several). Like buffer storage, the entries have no
//           asynchronous reset (D23); grp_i is a tie-off computed from the
//           router's coordinates, and later changes of it are ignored.
// Local     The local timetable of designs C and D uses its own slot_table
//           with TT_N = REGION_N and the region-relative group.
// =============================================================================
`timescale 1ns/1ps

module slot_table
  import sched_pkg::*;
#(
  parameter int TT_N = 8      // timetable side: MESH_N (global) or REGION_N (local)
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  tt_grp_t grp_i,      // this router's group in the timetable
  input  slot_t   slot_i,     // current slot, from slot_counter
  output logic    act_o,      // the router's group is one of the slot's groups
  output tt_grp_t tgt_o       // the other group of the slot (own group in a same-group slot)
);

  slot_t tab_q [TT_N];        // tab_q[t] = slot of the group pair {grp_i, t}

  // Slot of {g, t} = hi(hi+1)/2 + lo (Table II), with tri_num() for hi(hi+1)/2;
  // for a fixed t only g varies, so each entry is a compare, an add and a mux
  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      for (int t = 0; t < TT_N; t++)
        tab_q[t] <= (tt_grp_t'(t) <= grp_i) ? tri_num(grp_i) + slot_t'(t) : tri_num(tt_grp_t'(t)) + slot_t'(grp_i);
    end
  end

  // The entries of one table are all different, so at most one matches
  always_comb begin
    act_o = 1'b0;
    tgt_o = '0;
    for (int t = 0; t < TT_N; t++) begin
      if (tab_q[t] == slot_i) begin
        act_o = 1'b1;
        tgt_o = tt_grp_t'(t);
      end
    end
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (TT_N < 2 || TT_N > 16 || !pow2(TT_N))
      $fatal(1, "%m: TT_N = %0d; the timetable side must be 2, 4, 8 or 16", TT_N);
  end
  always_ff @(posedge clk_i) begin
    if (!rst_ni) begin
      // the table is loaded from grp_i at this edge
      if ($isunknown(grp_i) || int'(grp_i) >= TT_N)
        $fatal(1, "%m: group %0d does not exist in a %0d x %0d timetable", grp_i, TT_N, TT_N);
    end else if ($isunknown({slot_i, act_o, tgt_o})) begin
      $fatal(1, "%m: X on slot_i, act_o or tgt_o");
    end
  end
  // synopsys translate_on
`endif

endmodule
