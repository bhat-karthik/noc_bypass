// =============================================================================
// File      tb/mesh/tb_mesh_dl.sv                                  Tracker: F29
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Top of the deadlock test: design A with minimal adaptive routing
//           and one VC, a configuration that is expected to deadlock (N05;
//           and BP08's design-A half, +TEST=BP08).
// Decisions D06 routing, D34 run control, D35 mesh benches, D49 B5 tests
// Tests     N05 (and BP08's design-A half)
// -----------------------------------------------------------------------------
// Define TB_MESH_N sets the mesh side (default 4). Without +TEST it runs N05;
// +TEST=BP08 runs BP08's traffic and expects the deadlock (BP08 is listed
// under tb_mesh_b, so a bundle define picks one top only):
//   Vivado: run.tcl -test tb_mesh_dl -plus {+TEST=BP08}
//   EDA Playground: +define+TEST_N05, run option +TEST=BP08
// @tests N05
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

`ifndef TB_MESH_N
  `define TB_MESH_N 4
`endif

module tb_mesh_dl;

  mesh_bench #(
    .MESH_N (`TB_MESH_N),
    .NUM_VC (1),
    .ALGO   (noc_pkg::RT_ADAPTIVE),
    .ID     ("N05")
  ) u_bench ();

  `TB_DUMP(tb_mesh_dl)

endmodule
