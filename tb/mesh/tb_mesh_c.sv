// =============================================================================
// File      tb/mesh/tb_mesh_c.sv                                   Tracker: F70
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of the design-C mesh tests: the two-level timetable (B6) in
//           REGION_N x REGION_N regions, frames of k_L = K_FIXED local slots
//           and one global slot, with the bypass datapath (B4), reservations
//           and cancel tokens (B5) and minimal adaptive regular routing.
// Decisions D15 D16 D19 two-level timetable, D50 one pointer, D51 B6
//           verification
// Tests     TL01 TL02 TL03 TL04 TL06
// -----------------------------------------------------------------------------
// Defines: TB_MESH_N (mesh side, default 4, as every mesh bench: the
// project's 8x8 needs -define TB_MESH_N=8), TB_REGION_N (default 2 on 4x4,
// 4 on larger meshes), TB_NUM_VC (default 4), TB_B_MAX (default 10), TB_SAFE
// (SAFE subslots), TB_XY (XY regular routing), TB_EPOCH (epoch in cycles;
// default 2^13 = 8192 on 8x8 and larger, D51, so that a plan changes within
// about one epoch; on 4x4 sched_pkg::epoch_len, 2048).
// The defaults are those of the other benches because run.tcl compiles
// every testbench file together and a define one file sets is seen by the
// files after it (tb_mesh_b.sv sets TB_MESH_N to 4).
//   Vivado: run.tcl -test TL02 -define TB_MESH_N=8 ;
//           run.tcl -test TL01 -define TB_MESH_N=16
//   EDA Playground: compile options +define+TEST_TL01 (4x4 with 2x2 regions,
//   so +QUICK fits Riviera's 60 seconds; +define+TEST_TL04 also sets epochs
//   of 512 cycles, as -define TB_EPOCH=512 does in Vivado).
// Without +TEST it runs TL01, TL02, TL03, TL04 and TL06 in order (TL05 is
// tb_mesh_cb). +TEST=N01, N03, N06, N07 or PERF runs that test on design C
// instead (every design-C checker on).
// @tests TL01 TL02 TL03 TL04 TL06
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

`ifndef TB_MESH_N
  `define TB_MESH_N 4
`endif
`ifndef TB_REGION_N
  `define TB_REGION_N ((`TB_MESH_N <= 4) ? 2 : 4)
`endif
`ifndef TB_NUM_VC
  `define TB_NUM_VC 4
`endif
`ifndef TB_B_MAX
  `define TB_B_MAX 10
`endif

module tb_mesh_c;

`ifdef TB_SAFE
  localparam int TM = sched_pkg::TM_SAFE;
`else
  localparam int TM = sched_pkg::TM_TIGHT;
`endif
`ifdef TB_XY
  localparam int AL = noc_pkg::RT_XY;
`else
  localparam int AL = noc_pkg::RT_ADAPTIVE;
`endif
  localparam int RG = `TB_REGION_N;
`ifdef TB_EPOCH
  localparam int EP = `TB_EPOCH;
`elsif TEST_TL04
  // the EDA Playground form of TL04 (the bundle's define): epochs of 512
  // cycles, so its four plan changes fit in Riviera's 60 seconds
  localparam int EP = 512;
`else
  localparam int EP = (`TB_MESH_N >= 8) ? 8192 : sched_pkg::epoch_len(`TB_MESH_N, RG, `TB_B_MAX, TM);
`endif

  mesh_bench #(
    .MESH_N   (`TB_MESH_N),
    .NUM_VC   (`TB_NUM_VC),
    .ALGO     (AL),
    .DESIGN   (noc_pkg::DESIGN_C),
    .B_MAX    (`TB_B_MAX),
    .T_MODE   (TM),
    .ID       ("TL01-TL06"),
    .REGION_N (RG),
    .K_INIT   (sched_pkg::k_fixed(`TB_MESH_N, RG, `TB_B_MAX, TM)),
    .EPOCH    (EP)
  ) u_bench ();

  `TB_DUMP(tb_mesh_c)

endmodule
