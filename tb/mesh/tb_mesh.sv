// =============================================================================
// File      tb/mesh/tb_mesh.sv                                     Tracker: F29
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of the design-A mesh tests: XY routing, NUM_VC VCs.
// Decisions D35 mesh benches
// Tests     N01 N02 N03 N04 N06 N07 N08, and PERF on request
// -----------------------------------------------------------------------------
// Defines: TB_MESH_N (mesh side, default 4), TB_NUM_VC (default 4).
//   Vivado: run.tcl -test N03 -define TB_MESH_N=8
//   EDA Playground: compile options +define+TEST_N03 +define+TB_MESH_N=8
// Without +TEST it runs N01-N04 and N06-N08 in order (not PERF).
// @tests N01 N02 N03 N04 N06 N07 N08 PERF
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

`ifndef TB_MESH_N
  `define TB_MESH_N 4
`endif
`ifndef TB_NUM_VC
  `define TB_NUM_VC 4
`endif

module tb_mesh;

  mesh_bench #(
    .MESH_N (`TB_MESH_N),
    .NUM_VC (`TB_NUM_VC),
    .ALGO   (noc_pkg::RT_XY),
    .ID     ("N01-N08")
  ) u_bench ();

  `TB_DUMP(tb_mesh)

endmodule
