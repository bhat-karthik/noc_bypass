// =============================================================================
// File      rtl/router/cancel_relay.sv                             Tracker: F46
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Cancel tokens (D12): a destination that cannot reserve space for
//           the next subslot's burst sends one token back to that burst's
//           source, over the burst's own path in the opposite direction, one
//           hop per cycle. Each router forwards the tokens passing through it
//           by dimension order in the token's mode, launches its own NI's
//           token, and hands a token addressed to it to its bypass source.
// Decisions D12 reservation and cancel, D38 timing contract, D48 reservation
// Tests     BP06, BP07; every design-B test (router_checker expects each token
//           on every link of its path at its cycle)
// -----------------------------------------------------------------------------
// Timing (D38, sched_pkg tc_cancel): the destination decides in cycle d, the
// first cycle of the subslot before (for subslot 0, the first cycle of the
// setup window). Its token enters the cx register of its first reverse hop
// in cycle d, so it is on reverse link h at d + h (h = 1..L; link L enters
// the source). The source sees it on its input in cycle d + L and holds it
// from d + L + 1 <= t, the first cycle of the cancelled subslot, because the
// subslot and the setup window are at least Hmax + 1 cycles long.
// Path: the burst goes S -> D in mode m and the token D -> S in the other
// mode, over the same links: XY from S to D and YX from D to S are the same
// links reversed. The tokens of one decision therefore share no link when
// the bursts share none, which U09 checks for every timetable we build;
// tokens of different decisions never meet (one decision's tokens are gone
// after Hmax cycles, before the next decision).
// Registers: one per mesh output; the valid bit is reset, the payload is not
// (D27).
// =============================================================================
`timescale 1ns/1ps

module cancel_relay
  import noc_pkg::*;
#(
  parameter int MESH_N = 8
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  coord_t      cur_x_i,       // this router
  input  coord_t      cur_y_i,
  input  cx_t  [3:0]  cx_i,          // tokens on the four mesh inputs (N, E, S, W)
  input  cx_t         inj_i,         // this router's own token (its NI cancels a burst)
  output cx_t  [3:0]  cx_o,          // tokens on the four mesh outputs
  output logic        hit_o,         // a token for this router: its next turn is cancelled
  output logic        busy_o         // a token is in one of the registers
);

  node_id_t   me;
  port_t      in_port [4];           // output of a token arriving on input p
  port_t      inj_port;              // output of the router's own token
  logic [3:0] in_hit;                // the token on input p is for this router
  cx_t  [3:0] st_d;
  logic [3:0] vld_q;
  cx_t  [3:0] cx_q;

  assign me = xy_to_id(cur_x_i, cur_y_i, MESH_N);

  always_comb begin
    for (int p = 0; p < 4; p++) begin
      in_port[p] = dor_port(cur_x_i, cur_y_i, id_x(cx_i[p].tgt, MESH_N), id_y(cx_i[p].tgt, MESH_N), cx_i[p].mode);
      in_hit[p]  = cx_i[p].vld && (cx_i[p].tgt == me);
    end
    inj_port = dor_port(cur_x_i, cur_y_i, id_x(inj_i.tgt, MESH_N), id_y(inj_i.tgt, MESH_N), inj_i.mode);
    for (int q = 0; q < 4; q++) begin
      st_d[q] = '0;
      if (inj_i.vld && int'(inj_port) == q) st_d[q] = inj_i;
      for (int p = 0; p < 4; p++) begin
        if (cx_i[p].vld && !in_hit[p] && int'(in_port[p]) == q) st_d[q] = cx_i[p];
      end
    end
  end

  assign hit_o = |in_hit;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      vld_q <= '0;
    end else begin
      for (int q = 0; q < 4; q++) vld_q[q] <= st_d[q].vld;
    end
  end

  always_ff @(posedge clk_i) begin
    for (int q = 0; q < 4; q++) begin
      if (st_d[q].vld) cx_q[q] <= st_d[q];
    end
  end

  always_comb begin
    for (int q = 0; q < 4; q++) begin
      cx_o[q]     = cx_q[q];
      cx_o[q].vld = vld_q[q];
    end
  end

  assign busy_o = (vld_q != '0);

`ifndef SYNTHESIS
  // Coverage (the testbench reads it; a hook, so outside the synthesis-off
  // pragmas): tokens launched here, forwarded, and delivered to this router
  longint unsigned n_inj, n_fwd, n_hit;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      n_inj <= 0;
      n_fwd <= 0;
      n_hit <= 0;
    end else begin
      if (inj_i.vld) n_inj <= n_inj + 1;
      if (hit_o)     n_hit <= n_hit + 1;
      for (int p = 0; p < 4; p++) begin
        if (cx_i[p].vld && !in_hit[p]) n_fwd <= n_fwd + 1;
      end
    end
  end

  // synopsys translate_off
  always_ff @(posedge clk_i or negedge rst_ni) begin
    int n_out, n_in;
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      if ($isunknown({inj_i.vld, cx_i[0].vld, cx_i[1].vld, cx_i[2].vld, cx_i[3].vld}))
        $fatal(1, "%m: X on a cancel token valid bit");
      if (inj_i.vld && (inj_i.tgt == me || int'(inj_i.tgt) >= MESH_N * MESH_N))
        $fatal(1, "%m: this router's NI launches a cancel token for router %0d", inj_i.tgt);
      n_in = 0;
      for (int p = 0; p < 4; p++) begin
        if (cx_i[p].vld) begin
          if (int'(cx_i[p].tgt) >= MESH_N * MESH_N)
            $fatal(1, "%m: cancel token on input %0d for router %0d, outside the mesh", p, cx_i[p].tgt);
          if (!in_hit[p] && int'(in_port[p]) == p)
            $fatal(1, "%m: cancel token on input %0d would turn back out of the same port", p);
        end
        if (in_hit[p]) n_in++;
      end
      if (n_in > 1)
        $fatal(1, "%m: %0d cancel tokens for this router in one cycle", n_in);
      for (int q = 0; q < 4; q++) begin
        n_out = 0;
        if (inj_i.vld && int'(inj_port) == q) n_out++;
        for (int p = 0; p < 4; p++) begin
          if (cx_i[p].vld && !in_hit[p] && int'(in_port[p]) == q) n_out++;
        end
        if (n_out > 1)
          $fatal(1, "%m: %0d cancel tokens for output %0d in one cycle", n_out, q);
      end
    end
  end
  // synopsys translate_on
`endif

endmodule
