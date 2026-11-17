// =============================================================================
// File      rtl/router/route_compute.sv                            Tracker: F04
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Output port for a packet at the current router. ALGO selects:
//             RT_XY        X first, then Y (design A; deadlock-free on its own)
//             RT_ADAPTIVE  minimal adaptive (designs B, C, D; D06). When both
//                          X and Y bring the packet closer, take the one whose
//                          downstream input port has more free VCs; a tie
//                          goes to X.
//             RT_DOR       XY or YX, chosen per packet by mode_i (bypass
//                          paths, D09)
// Tests     U03 (tb/unit/tb_route_compute.sv), R04
// -----------------------------------------------------------------------------
// Purely combinational; the router uses it in the BW+RC stage (D03).
// The router's own coordinates are inputs, not parameters, so one synthesized
// tile serves every position in the mesh (D20).
// Every choice is minimal: the chosen port always shortens the distance by
// one hop, so a packet takes exactly the Manhattan distance (no livelock).
// RT_ADAPTIVE alone can deadlock (test N05); in designs B-D the bypass
// timetable drains any cyclic wait (test BP08), as argued in [1].
// =============================================================================
`timescale 1ns/1ps

module route_compute
  import noc_pkg::*;
#(
  parameter int MESH_N = 8,            // mesh side, 2..16
  parameter int ALGO   = RT_ADAPTIVE   // RT_XY, RT_ADAPTIVE or RT_DOR
) (
  input  coord_t                          cur_x_i,    // this router
  input  coord_t                          cur_y_i,
  input  node_id_t                        dst_i,      // destination router
  input  logic                            mode_i,     // RT_DOR only: MODE_XY or MODE_YX
  input  logic [NPORTS-1:0][VC_CNT_W-1:0] free_vc_i,  // RT_ADAPTIVE only: free VCs at
                                                      // the downstream input of each port
  output port_t                           port_o
);

  coord_t              dst_x, dst_y;
  logic                x_prod, y_prod;   // a move in x (in y) brings the packet closer
  port_t               x_port, y_port;   // the productive port in x (in y)
  logic [VC_CNT_W-1:0] x_free, y_free;   // free VCs behind x_port (y_port)

  assign dst_x  = id_x(dst_i, MESH_N);
  assign dst_y  = id_y(dst_i, MESH_N);
  assign x_prod = (dst_x != cur_x_i);
  assign y_prod = (dst_y != cur_y_i);
  assign x_port = (dst_x > cur_x_i) ? P_E : P_W;
  assign y_port = (dst_y > cur_y_i) ? P_S : P_N;
  assign x_free = (dst_x > cur_x_i) ? free_vc_i[P_E] : free_vc_i[P_W];
  assign y_free = (dst_y > cur_y_i) ? free_vc_i[P_S] : free_vc_i[P_N];

  always_comb begin
    if (!x_prod && !y_prod) begin
      port_o = P_L;                                         // arrived
    end else if (ALGO == RT_ADAPTIVE) begin
      if      (!y_prod) port_o = x_port;                    // only x is productive
      else if (!x_prod) port_o = y_port;                    // only y is productive
      else              port_o = (y_free > x_free) ? y_port : x_port;   // tie: X
    end else if (ALGO == RT_DOR && mode_i == MODE_YX) begin
      port_o = y_prod ? y_port : x_port;                    // Y first
    end else begin
      port_o = x_prod ? x_port : y_port;                    // X first: RT_XY, RT_DOR in XY mode
    end
  end

  // Each algorithm reads only some inputs; this keeps lint quiet for all ALGO
  logic unused_inputs;
  assign unused_inputs = ^{mode_i, free_vc_i, x_free, y_free};

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (MESH_N < 2 || MESH_N > MAX_MESH_N)
      $fatal(1, "%m: MESH_N = %0d, must be 2..%0d", MESH_N, MAX_MESH_N);
    if (ALGO != RT_XY && ALGO != RT_ADAPTIVE && ALGO != RT_DOR)
      $fatal(1, "%m: unknown ALGO %0d", ALGO);
  end
  // synopsys translate_on
`endif

endmodule
