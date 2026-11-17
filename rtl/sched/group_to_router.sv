// =============================================================================
// File      rtl/sched/group_to_router.sv                           Tracker: F35
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   "Group to Router" of [1, Fig. 10]: from the other group of the
//           slot (slot_table) and the subslot (slot_counter), the router this
//           router sends to and the router it receives from in that subslot
//           (Eq. 2 and the partner rule D37), the dimension order of each path
//           (D09) and its length in hops. In a local timetable the partners
//           are found in region-relative coordinates and moved back to chip
//           coordinates by adding the region's corner.
// Decisions D08 partner, D09 path direction, D15 regions, D37 partner rule
// Tests     U07 (tb/unit/tb_group_to_router.sv)
// -----------------------------------------------------------------------------
// Example   4 x 4, slot 8 (groups 2 and 3), subslot 2: R2 (x 2, y 0; group 2,
//           row 0) sends to R9 (x 1, y 2; group 3, row 2) by XY and receives
//           from R9, which sends by YX over the same links [1, Fig. 3].
// Outputs   Pure combinational logic of tie-off coordinates and the two
//           schedule inputs. The partner fields read zero when the matching
//           valid is 0: when act_i is 0, and in subslot 0 of a same-group slot.
// Use       The current subslot drives the sending side (B4); nxt_sub_o of
//           slot_counter drives a second copy for the receiving side, which
//           reserves or cancels one subslot ahead (B5). rx_mode_o is the mode
//           of the incoming path; the cancel token goes back in the other mode.
// =============================================================================
`timescale 1ns/1ps

module group_to_router
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int MESH_N = 8,   // mesh side
  parameter int TT_N   = 8    // timetable side: MESH_N (global) or REGION_N (local)
) (
  input  coord_t  x_i,        // this router, chip coordinates (tie-offs)
  input  coord_t  y_i,
  input  logic    act_i,      // slot_table: the router's group is in the slot
  input  tt_grp_t tgt_i,      // slot_table: the other group of the slot
  input  sub_t    sub_i,      // the subslot
  output logic    tx_vld_o,   // this router sends a burst in the subslot ...
  output coord_t  tx_x_o,     // ... to this router
  output coord_t  tx_y_o,
  output logic    tx_mode_o,  // MODE_XY or MODE_YX
  output hop_t    tx_hops_o,
  output logic    rx_vld_o,   // a burst comes to this router in the subslot ...
  output coord_t  rx_x_o,     // ... from this router
  output coord_t  rx_y_o,
  output logic    rx_mode_o,  // mode of the incoming path
  output hop_t    rx_hops_o
);

  int       bx, by;           // region corner (0, 0 for the global timetable)
  int       g, j;             // this router's group and row in the timetable
  partner_t tp, rp;

  always_comb begin
    bx = int'(x_i) & ~(TT_N - 1);     // TT_N is a power of two
    by = int'(y_i) & ~(TT_N - 1);
    j  = int'(y_i) - by;
    g  = grp_of(int'(x_i) - bx, j, TT_N);
    tp = tx_partner(g, j, int'(tgt_i), int'(sub_i), TT_N);
    rp = rx_partner(g, j, int'(tgt_i), int'(sub_i), TT_N);

    tx_vld_o  = act_i && tp.vld;
    tx_x_o    = tx_vld_o ? coord_t'(bx + int'(tp.x)) : '0;
    tx_y_o    = tx_vld_o ? coord_t'(by + int'(tp.y)) : '0;
    tx_mode_o = tx_vld_o && tp.mode;
    tx_hops_o = tx_vld_o ? hop_t'(hops(x_i, y_i, tx_x_o, tx_y_o)) : '0;

    rx_vld_o  = act_i && rp.vld;
    rx_x_o    = rx_vld_o ? coord_t'(bx + int'(rp.x)) : '0;
    rx_y_o    = rx_vld_o ? coord_t'(by + int'(rp.y)) : '0;
    rx_mode_o = rx_vld_o && rp.mode;
    rx_hops_o = rx_vld_o ? hop_t'(hops(x_i, y_i, rx_x_o, rx_y_o)) : '0;
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (TT_N < 2 || TT_N > 16 || !pow2(TT_N) || MESH_N > MAX_MESH_N || MESH_N % TT_N != 0)
      $fatal(1, "%m: TT_N = %0d, MESH_N = %0d; the timetable side must be 2, 4, 8 or 16 and divide the mesh side",
             TT_N, MESH_N);
  end
  // synopsys translate_on
`endif

endmodule
