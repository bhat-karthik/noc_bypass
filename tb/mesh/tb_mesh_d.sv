// =============================================================================
// File      tb/mesh/tb_mesh_d.sv                                   Tracker: F74
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of the design-D mesh tests: design C's two-level timetable
//           with demand counters in every tile and the controller (B7),
//           which sets k_L every epoch from the mesh-wide demand.
// Decisions D17 D18 D19 demand, policy, epoch; D54-D56 controller;
//           D57 B7 verification
// Tests     CT01 CT02 CT03 CT04 CT05
// -----------------------------------------------------------------------------
// Defines as tb_mesh_c: TB_MESH_N (default 4: the project's 8x8 needs
// -define TB_MESH_N=8), TB_REGION_N (2 on 4x4, else 4), TB_NUM_VC (4),
// TB_B_MAX (10), TB_SAFE, TB_XY, TB_EPOCH (epoch in cycles; default 4096
// on 8x8 and larger, a shorter epoch than D19's 2^15 so that the tests
// see many plans in minutes; on 4x4 sched_pkg::epoch_len, 2048; 512 under
// the EDA bundle's TEST_CT0x defines). Plusargs +POLICY=SQRT, +DAMP=0.
//   Vivado: run.tcl -test CT02 -define TB_MESH_N=8
//   EDA Playground: compile options +define+TEST_CT02, run options +QUICK
// Without +TEST it runs CT01-CT05 in order. +TEST=N01, N03, N06, N07 or
// PERF runs that test on design D instead (every checker on).
// @tests CT01 CT02 CT03 CT04 CT05
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

module tb_mesh_d;

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
`elsif TEST_CT01
  localparam int EP = 512;
`elsif TEST_CT02
  localparam int EP = 512;
`elsif TEST_CT03
  localparam int EP = 512;
`elsif TEST_CT04
  localparam int EP = 512;
`elsif TEST_CT05
  localparam int EP = 512;
`else
  localparam int EP = (`TB_MESH_N >= 8) ? 4096 : sched_pkg::epoch_len(`TB_MESH_N, RG, `TB_B_MAX, TM);
`endif

  mesh_bench #(
    .MESH_N   (`TB_MESH_N),
    .NUM_VC   (`TB_NUM_VC),
    .ALGO     (AL),
    .DESIGN   (noc_pkg::DESIGN_D),
    .B_MAX    (`TB_B_MAX),
    .T_MODE   (TM),
    .ID       ("CT01-CT05"),
`ifdef TB_K_MIN
    .K_LO     (`TB_K_MIN),
`endif
`ifdef TB_K_MAX
    .K_HI     (`TB_K_MAX),
`endif
    .REGION_N (RG),
    .K_INIT   (sched_pkg::k_fixed(`TB_MESH_N, RG, `TB_B_MAX, TM)),
    .EPOCH    (EP)
  ) u_bench ();

  `TB_DUMP(tb_mesh_d)

endmodule
