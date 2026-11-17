// =============================================================================
// File      tb/mesh/tb_mesh_cb.sv                                  Tracker: F72
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of TL05: a design-C mesh and a design-B mesh with the same
//           traffic, every NI output and every link compared every cycle.
//           On 4x4 the region is the whole mesh (one pointer, D50); on a
//           larger mesh design C runs with k_L = 0 from reset (global slots
//           only). Either way design C must be exactly BuffeRS.
// Decisions D50 one pointer, D51 B6 verification
// Tests     TL05
// -----------------------------------------------------------------------------
// Defines: TB_MESH_N (default 4), TB_NUM_VC (4), TB_B_MAX (10), TB_SAFE,
// TB_XY, as tb_mesh_c.
//   Vivado: run.tcl -test TL05 ; run.tcl -test TL05 -define TB_MESH_N=8
//   EDA Playground: compile options +define+TEST_TL05
// @tests TL05
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

`ifndef TB_MESH_N
  `define TB_MESH_N 4
`endif
`ifndef TB_NUM_VC
  `define TB_NUM_VC 4
`endif
`ifndef TB_B_MAX
  `define TB_B_MAX 10
`endif

module tb_mesh_cb;

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
  // one region on 4x4 (and 2x2); 4x4 regions with k_L = 0 on larger meshes
  localparam int RG = (`TB_MESH_N <= 4) ? `TB_MESH_N : 4;

  mesh_bench #(
    .MESH_N   (`TB_MESH_N),
    .NUM_VC   (`TB_NUM_VC),
    .ALGO     (AL),
    .DESIGN   (noc_pkg::DESIGN_C),
    .B_MAX    (`TB_B_MAX),
    .T_MODE   (TM),
    .ID       ("TL05"),
    .REGION_N (RG),
    .K_INIT   (0),
    .SHADOW   (1)
  ) u_bench ();

  `TB_DUMP(tb_mesh_cb)

endmodule
