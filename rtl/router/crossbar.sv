// =============================================================================
// File      rtl/router/crossbar.sv                                 Tracker: F15
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Stage 3 of the router, ST: moves each input's flit to the output
//           port it was granted and holds it in that output's ST register.
//           Each input can instead pass a bypass flit straight from its link
//           (B4), which is how a bypass hop takes only ST + LT.
// Decisions D03 pipeline, D07 bypass datapath, D27 registers
// Tests     R01-R04 (tb/unit/tb_router.sv)
// -----------------------------------------------------------------------------
// Source of input i: the SA->ST register of input unit i, or, when
// byp_en_i[i] is 1, the bypass flit arriving on link i in this cycle.
// The switch allocator (and, in B4, the bypass masks) give every output at
// most one source per cycle, so each output is a one-hot AND-OR of the
// inputs: no priority chain. A simulation check stops the run if two
// sources ever want the same output.
// The ST registers keep the valid bit under reset; the payload has no reset.
// =============================================================================
`timescale 1ns/1ps

module crossbar
  import noc_pkg::*;
(
  input  logic                clk_i,
  input  logic                rst_ni,
  // per input: the SA->ST registers
  input  flit_t [NPORTS-1:0]  sa_flit_i,
  input  port_t [NPORTS-1:0]  sa_port_i,
  // per input: a bypass flit from the link instead (B4; zero in design A)
  input  logic  [NPORTS-1:0]  byp_en_i,
  input  flit_t [NPORTS-1:0]  byp_flit_i,
  input  port_t [NPORTS-1:0]  byp_port_i,
  // per output: the ST registers
  output flit_t [NPORTS-1:0]  st_flit_o
);

  flit_t [NPORTS-1:0]             src;
  port_t [NPORTS-1:0]             src_port;
  logic  [NPORTS-1:0][NPORTS-1:0] hit;       // [o][i]: input i goes to output o
  logic  [NPORTS-1:0][FLIT_W-1:0] sel;       // flit chosen for output o

  always_comb begin
    for (int i = 0; i < NPORTS; i++) begin
      src[i]      = byp_en_i[i] ? byp_flit_i[i] : sa_flit_i[i];
      src_port[i] = byp_en_i[i] ? byp_port_i[i] : sa_port_i[i];
    end
    for (int o = 0; o < NPORTS; o++) begin
      sel[o] = '0;
      for (int i = 0; i < NPORTS; i++) begin
        hit[o][i] = src[i].vld & (src_port[i] == port_t'(o));
        sel[o]    = sel[o] | ({FLIT_W{hit[o][i]}} & FLIT_W'(src[i]));
      end
    end
  end

  logic  [NPORTS-1:0] st_vld_q;
  flit_t [NPORTS-1:0] st_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) st_vld_q <= '0;
    else for (int o = 0; o < NPORTS; o++) st_vld_q[o] <= |hit[o];
  end

  always_ff @(posedge clk_i) begin
    for (int o = 0; o < NPORTS; o++) begin
      if (|hit[o]) st_q[o] <= flit_t'(sel[o]);
    end
  end

  always_comb begin
    for (int o = 0; o < NPORTS; o++) begin
      st_flit_o[o]     = st_q[o];
      st_flit_o[o].vld = st_vld_q[o];
    end
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      for (int o = 0; o < NPORTS; o++) begin
        if ($countones(hit[o]) > 1)
          $fatal(1, "%m: inputs %b all want output %0d in the same cycle", hit[o], o);
      end
      for (int i = 0; i < NPORTS; i++) begin
        if (src[i].vld && int'(src_port[i]) >= NPORTS)
          $fatal(1, "%m: input %0d sends to port %0d, which does not exist", i, src_port[i]);
      end
    end
  end
  // synopsys translate_on
`endif

endmodule
