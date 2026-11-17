// =============================================================================
// File      tb/mesh/tb_mesh_b.sv                                   Tracker: F29
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of the design-B mesh tests: BuffeRS with one global
//           timetable, the bypass datapath (B4), reservations and cancel
//           tokens (B5), minimal adaptive regular routing (D06).
// Decisions D35 mesh benches, D44 design-B tile, D45 B4 verification,
//           D49 B5 verification
// Tests     BP01-BP10
// -----------------------------------------------------------------------------
// Defines: TB_MESH_N (mesh side, default 4), TB_NUM_VC (default 4),
// TB_B_MAX (default 10), TB_SAFE (SAFE subslots instead of TIGHT),
// TB_XY (XY regular routing instead of adaptive).
//   Vivado: run.tcl -test BP02 -define TB_MESH_N=8
//   EDA Playground: compile options +define+TEST_BP01
// Without +TEST it runs BP01-BP07, BP09 and BP10 in order. BP08 needs one
// VC: run.tcl -test BP08 -define TB_NUM_VC=1 (EDA Playground: the define
// TEST_BP08 sets one VC by itself). +TEST=N01, N03, N06, N07 or PERF runs
// that test on design B instead (the bypass checkers on).
// @tests BP01 BP02 BP03 BP04 BP05 BP06 BP07 BP08 BP09 BP10
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

`ifndef TB_MESH_N
  `define TB_MESH_N 4
`endif
`ifdef TEST_BP08
  `ifndef TB_NUM_VC
    `define TB_NUM_VC 1
  `endif
`endif
`ifndef TB_NUM_VC
  `define TB_NUM_VC 4
`endif
`ifndef TB_B_MAX
  `define TB_B_MAX 10
`endif

module tb_mesh_b;

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

  mesh_bench #(
    .MESH_N (`TB_MESH_N),
    .NUM_VC (`TB_NUM_VC),
    .ALGO   (AL),
    .DESIGN (noc_pkg::DESIGN_B),
    .B_MAX  (`TB_B_MAX),
    .T_MODE (TM),
    .ID     ("BP01-BP10")
  ) u_bench ();

  `TB_DUMP(tb_mesh_b)

endmodule
