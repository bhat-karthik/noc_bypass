// =============================================================================
// File      rtl/ctrl/ctrl_node.sv                                  Tracker: F55
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Design D's controller network in one tile (D55): adds the tile's
//           latched demand sums to the sums coming from its neighbours, one
//           bit per cycle, and passes them on toward tile (0, 0); forwards
//           the plan (k_L) the other way, one hop per cycle.
// Decisions D55 controller network and timing
// Tests     U14 (tb/unit/tb_ctrl_chain.sv); CT01-CT05 (tb_mesh_d)
// -----------------------------------------------------------------------------
// Sum tree  Row y adds from east to west into column 0; column 0 adds from
//           south to north into tile (0, 0), whose output goes to
//           sched_ctrl. Every link carries two bits per cycle, the local sum
//           (bit 0) and the cross-region sum (bit 1), least significant bit
//           first. Tile (x, y) adds bit k of its own sums, of its east input
//           and (column 0 only) of its south input in cycle
//           ct_start(x, y) + k = 2N - 1 - x - y + k after the epoch tick,
//           k = 0..dm_sum_w(N) - 1, and registers the result: the row y
//           leaf starts in cycle N - y, so every operand bit meets its
//           partners with no valid wire and no buffer. Three operand bits
//           and a carry of up to 2 per stream. Outside its window a node
//           sends zeros.
// Plan      plan_i comes from the west neighbour (from the north one in
//           column 0; from sched_ctrl at (0, 0)); plan_o is it, registered.
//           It drives the tile's phase_sequencer and the east neighbour
//           (and the south one in column 0). The last tile has a new plan
//           2N - 1 cycles after sched_ctrl, long before the plan sample
//           (cycle A_DELAY - 1, D52).
// Coordinates are ports (D20): one generic tile.
// =============================================================================
`timescale 1ns/1ps

module ctrl_node
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int MESH_N = 8,
  parameter int K_INIT = 4                 // plan from reset
) (
  input  logic             clk_i,
  input  logic             rst_ni,
  input  coord_t           cur_x_i,
  input  coord_t           cur_y_i,
  input  logic             tick_i,         // first cycle of an epoch
  input  logic [DM_W-1:0]  cl_i,           // this tile's latched sums (demand_counter)
  input  logic [DM_W-1:0]  cg_i,
  input  logic [1:0]       sum_e_i,        // from the east neighbour
  input  logic [1:0]       sum_s_i,        // from the south neighbour (column 0)
  output logic [1:0]       sum_o,          // toward tile (0, 0)
  input  kl_t              plan_i,         // from upstream
  output kl_t              plan_o          // registered: sequencer, downstream
);

  localparam int SW = dm_sum_w(MESH_N);    // bits of the mesh-wide sums

  logic [CT_W-1:0] ct_q;                   // cycle since the last tick (tick = 0), saturating
  logic [1:0]      cl_c, cg_c;             // carries
  logic [1:0]      sum_q;
  kl_t             plan_q;

  // ---------------------------------------------------------------- window
  logic [CT_W-1:0] t0, k;
  logic            act;

  assign t0  = CT_W'(2 * MESH_N - 1) - CT_W'(cur_x_i) - CT_W'(cur_y_i);
  assign k   = ct_q - t0;
  assign act = !tick_i && (ct_q >= t0) && (k < CT_W'(SW));

  // ---------------------------------------------------------------- serial adders
  logic       ol, og, se, ss_l, ss_g;
  logic [2:0] tl, tg;

  assign ol   = (k < CT_W'(DM_W)) ? cl_i[k[$clog2(DM_W)-1:0]] : 1'b0;
  assign og   = (k < CT_W'(DM_W)) ? cg_i[k[$clog2(DM_W)-1:0]] : 1'b0;
  assign se   = (cur_x_i == '0);           // column 0 also adds its south input
  assign ss_l = se & sum_s_i[0];
  assign ss_g = se & sum_s_i[1];
  assign tl   = 3'(ol) + 3'(sum_e_i[0]) + 3'(ss_l) + 3'(cl_c);
  assign tg   = 3'(og) + 3'(sum_e_i[1]) + 3'(ss_g) + 3'(cg_c);

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ct_q   <= '1;
      cl_c   <= '0;
      cg_c   <= '0;
      sum_q  <= '0;
      plan_q <= kl_t'(K_INIT);
    end else begin
      ct_q   <= tick_i ? CT_W'(1) : (ct_q == '1) ? ct_q : ct_q + 1'b1;
      plan_q <= plan_i;
      if (act) begin
        sum_q <= {tg[0], tl[0]};
        cl_c  <= tl[2:1];
        cg_c  <= tg[2:1];
      end else begin
        sum_q <= '0;
        cl_c  <= '0;
        cg_c  <= '0;
      end
    end
  end

  assign sum_o  = sum_q;
  assign plan_o = plan_q;

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (4 * MESH_N + SW + 8 > A_DELAY)
      $fatal(1, "%m: a %0dx%0d mesh cannot sum and forward the plan within A_DELAY = %0d", MESH_N, MESH_N, A_DELAY);
  end
  // a carry left over after the last bit would be a lost sum bit (act is
  // never true in reset)
  always_ff @(posedge clk_i) begin
    if (act && k == CT_W'(SW - 1) && (tl[2:1] != '0 || tg[2:1] != '0))
      $fatal(1, "%m: the demand sums overflow %0d bits", SW);
  end
  // synopsys translate_on
`endif

endmodule
