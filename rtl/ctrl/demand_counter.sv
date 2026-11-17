// =============================================================================
// File      rtl/ctrl/demand_counter.sv                             Tracker: F54
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Design D's demand measure in one tile (D17, D54): how many of
//           the router's input VCs hold a packet, split into packets whose
//           destination is in this tile's region (local) and the rest
//           (cross-region), summed every DM_SAMPLE cycles over an epoch and
//           latched at the next epoch tick.
// Decisions D17 demand, D54 counting from the links
// Tests     U12 (tb/unit/tb_demand_counter.sv); CT01-CT05 (tb_mesh_d)
// -----------------------------------------------------------------------------
// Occupancy is read from the router's links alone, so the router is not
// changed: input VC v of port p is occupied from the cycle after a head
// (FT_HEAD or FT_SINGLE, not a bypass flit) arrives on in_flit_i[p] for VC v
// until the router sends the credit that frees it (in_cred_i[p], vc_free).
// The head's destination decides local or cross-region once, on arrival.
// The five ports include the NI's: a packet that waits at its source counts.
// A bypass burst takes its packets out of the VCs with the same credits, so
// those VCs empty as for regular flits.
// Sums     Sampled in every cycle whose number since the epoch tick is a
//          multiple of DM_SAMPLE, the tick included: popcounts of occupied
//          local and occupied cross-region VCs (loops, README rule 9) are
//          added to two DM_W-bit sums that saturate at 2^DM_W - 1. At a tick
//          the sums of the epoch just ended move to cl_o and cg_o, and the
//          new sums start with the tick's own sample. Reset is a tick
//          (phase_sequencer's epoch counter starts at its cycle 0).
// =============================================================================
`timescale 1ns/1ps

module demand_counter
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int REGION_N = 4,
  parameter int NUM_VC   = 4
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  input  coord_t                       cur_x_i,
  input  coord_t                       cur_y_i,
  input  logic                         tick_i,      // first cycle of an epoch
  input  flit_t      [NPORTS-1:0]      in_flit_i,   // the router's input links
  input  credit_t    [NPORTS-1:0]      in_cred_i,   // the credits it sends back on them
  output logic       [DM_W-1:0]        cl_o,        // last epoch: local occupancy sum
  output logic       [DM_W-1:0]        cg_o         // last epoch: cross-region occupancy sum
);

  localparam int NV  = NPORTS * NUM_VC;                // VCs counted
  localparam int PCW = $clog2(NV + 1);                 // a popcount
  localparam int SW  = $clog2(DM_SAMPLE);              // sample phase
  localparam int RGW = $clog2(REGION_N);               // coordinate bits inside a region

  logic [NPORTS-1:0][NUM_VC-1:0] occ_q, loc_q;
  logic [SW-1:0]                 ph_q;
  logic [DM_W-1:0]               al_q, ag_q, cl_q, cg_q;

  // ---------------------------------------------------------------- occupancy
  // region of a node: its coordinates without the RGW low bits
  function automatic logic same_region(node_id_t d, coord_t x, coord_t y);
    coord_t dx, dy;
    dx = id_x(d, MESH_N);
    dy = id_y(d, MESH_N);
    return ((dx >> RGW) == (x >> RGW)) && ((dy >> RGW) == (y >> RGW));
  endfunction

  for (genvar p = 0; p < NPORTS; p++) begin : g_port
    head_t hd;
    logic  arr;
    assign hd  = head_t'(in_flit_i[p].data);
    assign arr = in_flit_i[p].vld & ~in_flit_i[p].byp & is_head(in_flit_i[p].ftype);

    for (genvar v = 0; v < NUM_VC; v++) begin : g_vc
      logic set, clr;
      assign set = arr & (in_flit_i[p].vc == vc_id_t'(v));
      assign clr = in_cred_i[p].vld & in_cred_i[p].vc_free & (in_cred_i[p].vc == vc_id_t'(v));

      always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
          occ_q[p][v] <= 1'b0;
          loc_q[p][v] <= 1'b0;
        end else if (set) begin               // a new packet (a free credit cannot come with it)
          occ_q[p][v] <= 1'b1;
          loc_q[p][v] <= same_region(hd.dst, cur_x_i, cur_y_i);
        end else if (clr) begin
          occ_q[p][v] <= 1'b0;
        end
      end
    end

    // a head's other fields are not needed here
    logic unused_hd;
    assign unused_hd = ^{hd.rsvd_hi, hd.check, hd.t_create, hd.seq, hd.len, hd.rsvd_lo, hd.cls, hd.src};
  end

  // ---------------------------------------------------------------- sums
  logic [PCW-1:0]  n_loc, n_glb;
  logic [DM_W:0]   nl_sum, ng_sum;                     // one bit more, for saturation
  logic            samp;

  always_comb begin
    n_loc = '0;
    n_glb = '0;
    for (int p = 0; p < NPORTS; p++) begin
      for (int v = 0; v < NUM_VC; v++) begin
        if (occ_q[p][v] &&  loc_q[p][v]) n_loc = n_loc + 1'b1;
        if (occ_q[p][v] && !loc_q[p][v]) n_glb = n_glb + 1'b1;
      end
    end
  end

  // the sample phase follows the epoch: 0 at every tick
  assign samp = tick_i | (ph_q == '0);

  always_comb begin
    logic [DM_W:0] bl, bg;
    // the base the sample adds to: the running sums, or zero at a tick
    bl = tick_i ? '0 : {1'b0, al_q};
    bg = tick_i ? '0 : {1'b0, ag_q};
    nl_sum = bl + (DM_W + 1)'(n_loc);
    ng_sum = bg + (DM_W + 1)'(n_glb);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      ph_q <= '0;
      al_q <= '0;
      ag_q <= '0;
      cl_q <= '0;
      cg_q <= '0;
    end else begin
      ph_q <= tick_i ? SW'(1) : ph_q + 1'b1;
      if (tick_i) begin
        cl_q <= al_q;
        cg_q <= ag_q;
      end
      if (samp) begin
        al_q <= nl_sum[DM_W] ? {DM_W{1'b1}} : nl_sum[DM_W-1:0];
        ag_q <= ng_sum[DM_W] ? {DM_W{1'b1}} : ng_sum[DM_W-1:0];
      end
    end
  end

  assign cl_o = cl_q;
  assign cg_o = cg_q;

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (REGION_N < 2 || REGION_N > MESH_N || (REGION_N & (REGION_N - 1)) != 0)
      $fatal(1, "%m: REGION_N = %0d, must be a power of two, 2..MESH_N", REGION_N);
    if ((MESH_N & (MESH_N - 1)) != 0)
      $fatal(1, "%m: MESH_N = %0d, must be a power of two", MESH_N);
  end
  // synopsys translate_on
`endif

endmodule
