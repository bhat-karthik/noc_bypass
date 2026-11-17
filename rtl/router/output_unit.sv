// =============================================================================
// File      rtl/router/output_unit.sv                              Tracker: F16
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   One router output port: the state of the NUM_VC input VCs of the
//           downstream router (idle or held, credits), the lowest idle VC for
//           a new packet, the free-VC count adaptive routing reads, and the
//           link register (stage 4, LT).
// Decisions D03 pipeline, D05 credits and VC release, D06 free-VC count,
//           D27 registers, D28 allocation
// Tests     R01-R04 (tb/unit/tb_router.sv), U04 uses its outputs as inputs
// -----------------------------------------------------------------------------
// Credits   One counter per output VC, VC_DEPTH at reset. A flit granted by
//           the switch allocator takes one (gnt_i); a credit from downstream
//           gives one back (credit_i). Both may happen in the same cycle.
// VCs       A head granted with alloc_i takes the lowest idle VC (alloc_vc_o),
//           as Garnet's select_free_vc does. The VC stays held until the
//           downstream input says the tail has left (credit_i.vc_free); by
//           then every credit of the packet has come back, which a check
//           below confirms.
// Link      The crossbar's ST register feeds link_q, so a flit spends one
//           cycle on the link (LT) and the downstream router writes it in
//           the next cycle (BW): 4 cycles per hop in total.
// =============================================================================
`timescale 1ns/1ps

module output_unit
  import noc_pkg::*;
#(
  parameter int NUM_VC   = 4,   // 1..4
  parameter int VC_DEPTH = 5    // flits per downstream VC
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  // credits from the downstream input port
  input  credit_t             credit_i,
  // switch allocation result for this output, this cycle
  input  logic                gnt_i,       // a flit is granted to this output
  input  logic                alloc_i,     // it is a head and takes alloc_vc_o
  input  vc_id_t              gnt_vc_i,    // output VC the flit travels on
  // state for the switch allocator and for route computation
  output logic                has_free_o,  // at least one output VC is idle
  output vc_id_t              alloc_vc_o,  // lowest idle VC; valid with has_free_o
  output logic [NUM_VC-1:0]   credit_ok_o, // per VC: at least one credit
  output logic [VC_CNT_W-1:0] free_cnt_o,  // number of idle VCs
  // ST register (crossbar) -> LT register -> link
  input  flit_t               st_flit_i,
  output flit_t               link_o
);

  localparam int CRW = $clog2(VC_DEPTH + 1);

  logic [NUM_VC-1:0] busy_q;               // VC held by a packet
  logic [CRW-1:0]    cred_q [NUM_VC];      // free buffer slots downstream
  logic [NUM_VC-1:0] inc, dec, take, rel;  // per VC this cycle

  for (genvar v = 0; v < NUM_VC; v++) begin : g_vc
    assign inc[v]  = credit_i.vld & (credit_i.vc == vc_id_t'(v));
    assign dec[v]  = gnt_i & (gnt_vc_i == vc_id_t'(v));
    assign take[v] = alloc_i & (gnt_vc_i == vc_id_t'(v));
    assign rel[v]  = inc[v] & credit_i.vc_free;
    assign credit_ok_o[v] = (cred_q[v] != '0);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      busy_q <= '0;
      for (int v = 0; v < NUM_VC; v++) cred_q[v] <= CRW'(VC_DEPTH);
    end else begin
      for (int v = 0; v < NUM_VC; v++) begin
        if (inc[v] && !dec[v])      cred_q[v] <= cred_q[v] + CRW'(1);
        else if (dec[v] && !inc[v]) cred_q[v] <= cred_q[v] - CRW'(1);
        if (take[v])     busy_q[v] <= 1'b1;
        else if (rel[v]) busy_q[v] <= 1'b0;
      end
    end
  end

  // Lowest idle VC (scan from the top so the lowest one is written last)
  always_comb begin
    alloc_vc_o = '0;
    has_free_o = 1'b0;
    for (int v = NUM_VC - 1; v >= 0; v--) begin
      if (!busy_q[v]) begin
        alloc_vc_o = vc_id_t'(v);
        has_free_o = 1'b1;
      end
    end
  end

  always_comb begin
    free_cnt_o = '0;
    for (int v = 0; v < NUM_VC; v++) free_cnt_o = free_cnt_o + VC_CNT_W'(!busy_q[v]);
  end

  // LT register: the valid bit is reset, the payload is not
  logic  link_vld_q;
  flit_t link_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) link_vld_q <= 1'b0;
    else         link_vld_q <= st_flit_i.vld;
  end

  always_ff @(posedge clk_i) begin
    if (st_flit_i.vld) link_q <= st_flit_i;
  end

  always_comb begin
    link_o     = link_q;
    link_o.vld = link_vld_q;
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      if ($isunknown({credit_i.vld, gnt_i, alloc_i}))
        $fatal(1, "%m: X on credit_i.vld, gnt_i or alloc_i");
      if (credit_i.vld && int'(credit_i.vc) >= NUM_VC)
        $fatal(1, "%m: credit for VC %0d, only %0d VCs", credit_i.vc, NUM_VC);
      if (gnt_i && int'(gnt_vc_i) >= NUM_VC)
        $fatal(1, "%m: grant on VC %0d, only %0d VCs", gnt_vc_i, NUM_VC);
      if (alloc_i && (!has_free_o || gnt_vc_i != alloc_vc_o))
        $fatal(1, "%m: head given VC %0d, the offered idle VC is %0d (any idle: %0b)",
               gnt_vc_i, alloc_vc_o, has_free_o);
      for (int v = 0; v < NUM_VC; v++) begin
        if (dec[v] && !inc[v] && cred_q[v] == '0)
          $fatal(1, "%m: flit sent on VC %0d without a credit", v);
        if (inc[v] && !dec[v] && int'(cred_q[v]) == VC_DEPTH)
          $fatal(1, "%m: credit for VC %0d beyond the buffer depth %0d", v, VC_DEPTH);
        if (dec[v] && !take[v] && !busy_q[v])
          $fatal(1, "%m: flit sent on VC %0d, which no packet holds", v);
        if (rel[v] && !busy_q[v])
          $fatal(1, "%m: VC %0d freed but it was idle", v);
        if (rel[v] && int'(cred_q[v]) + 1 - int'(dec[v]) != VC_DEPTH)
          $fatal(1, "%m: VC %0d freed with %0d of %0d credits home", v,
                 int'(cred_q[v]) + 1 - int'(dec[v]), VC_DEPTH);
      end
    end
  end
  // synopsys translate_on
`endif

endmodule
