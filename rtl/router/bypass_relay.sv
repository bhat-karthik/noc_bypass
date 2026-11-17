// =============================================================================
// File      rtl/router/bypass_relay.sv                             Tracker: F42
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The passing and receiving side of a bypass: takes the lookahead
//           of a burst on one of the four mesh inputs, routes it by
//           dimension order, blocks switch allocation for the burst's input
//           and output, sets the crossbar so each flit goes straight from the
//           link into the ST register, and forwards the lookahead one hop
//           ahead. At the destination the burst leaves through the Local
//           output to the NI, and the lookahead goes to the NI's
//           reservation controller (la_loc_o, B5). Also launches the
//           source's own lookahead.
// Decisions D07 bypass datapath, D09 path direction, D38 timing contract,
//           D43 bypass relay, D48 reservation
// Tests     BP01-BP10 (tb_mesh_b)
// -----------------------------------------------------------------------------
// Timing at the i-th router of a path (D38), subslot starting at t:
//   c = t + 2i   the lookahead arrives on la_i[p]: the output q (dor_port with
//                the lookahead's mode; Local at the destination) and the
//                burst length are loaded, and switch allocation is blocked
//                for input p and output q over [c, c + B - 1] (cycle c
//                straight from the lookahead), so neither has a regular flit
//                in the crossbar over [c + 1, c + B]
//   c + 1 + j    flit j is on link p; it crosses the crossbar (input p ->
//                output q) into the ST register, is in the LT register next
//                cycle, and is on the next link at c + 3 + j
//   c + 2        the forwarded lookahead is on output q's lookahead link
//                (la ST register at the end of c, la LT register at the end
//                of c + 1), one cycle ahead of flit 0 again
// A new lookahead may arrive on an input in the cycle the previous burst's
// last flit crosses (bursts on one link back to back); earlier is an error.
// The source's lookahead (bypass_source) enters the la ST register of its
// first-hop output in its decision cycle, so it is on link 1 at t + 2.
// Bypass flits are never written into a VC and take no credit or VC; the
// input unit ignores them (flit.byp) and the output unit counts no credit.
// la_loc_o: the lookahead of a burst for this router, in the cycle it
// arrives (t + 2L); the NI's reservation controller reads the burst's length
// from it, or a no-show from its absence. At most one per cycle (the NI
// writes one flit per cycle, U09).
// =============================================================================
`timescale 1ns/1ps

module bypass_relay
  import noc_pkg::*;
#(
  parameter int MESH_N = 8,
  parameter int B_MAX  = 10                 // largest burst, flits
) (
  input  logic                clk_i,
  input  logic                rst_ni,
  input  coord_t              cur_x_i,      // this router
  input  coord_t              cur_y_i,
  // the four mesh inputs (N, E, S, W): lookaheads and flits
  input  la_t   [3:0]         la_i,
  input  flit_t [3:0]         flit_i,
  // the source's lookahead, for the ST register of output src_la_port_i
  input  logic                src_la_vld_i,
  input  port_t               src_la_port_i,
  input  la_t                 src_la_i,
  // crossbar inputs 0-3
  output logic  [3:0]         xb_en_o,
  output flit_t [3:0]         xb_flit_o,
  output port_t [3:0]         xb_port_o,
  // switch-allocation masks, 1 = blocked
  output logic  [NPORTS-1:0]  in_mask_o,
  output logic  [NPORTS-1:0]  out_mask_o,
  // lookaheads on the four mesh outputs (the la LT registers)
  output la_t   [3:0]         la_o,
  // the lookahead of a burst for this router's NI, this cycle
  output la_t                 la_loc_o,
  output logic                busy_o        // a burst or lookahead is inside
);

  // ---------------------------------------------------------------- per input
  port_t              la_port [4];      // output of an arriving lookahead
  logic [BURST_W-1:0] cnt_q   [4];      // flits of the burst still to come
  port_t              out_q   [4];      // their output

  always_comb begin
    for (int p = 0; p < 4; p++)
      la_port[p] = dor_port(cur_x_i, cur_y_i, id_x(la_i[p].dst, MESH_N), id_y(la_i[p].dst, MESH_N), la_i[p].mode);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      for (int p = 0; p < 4; p++) begin
        cnt_q[p] <= '0;
        out_q[p] <= P_L;
      end
    end else begin
      for (int p = 0; p < 4; p++) begin
        if (la_i[p].vld) begin
          cnt_q[p] <= la_i[p].len;
          out_q[p] <= la_port[p];
        end else if (cnt_q[p] != '0) begin
          cnt_q[p] <= cnt_q[p] - 1'b1;
        end
      end
    end
  end

  always_comb begin
    in_mask_o  = '0;
    out_mask_o = '0;
    for (int p = 0; p < 4; p++) begin
      xb_en_o[p]   = (cnt_q[p] != '0);
      xb_port_o[p] = out_q[p];
      xb_flit_o[p] = flit_i[p];
      if (la_i[p].vld) begin
        in_mask_o[p]           = 1'b1;
        out_mask_o[la_port[p]] = 1'b1;
      end
      if (cnt_q[p] >= BURST_W'(2)) begin
        in_mask_o[p]         = 1'b1;
        out_mask_o[out_q[p]] = 1'b1;
      end
    end
  end

  // ---------------------------------------------------------------- lookahead registers
  la_t  [3:0] st_d;
  logic [3:0] st_vld_q, lt_vld_q;
  la_t  [3:0] st_q, lt_q;

  always_comb begin
    for (int q = 0; q < 4; q++) begin
      st_d[q] = '0;
      if (src_la_vld_i && int'(src_la_port_i) == q) st_d[q] = src_la_i;
      for (int p = 0; p < 4; p++) begin
        if (la_i[p].vld && int'(la_port[p]) == q) st_d[q] = la_i[p];
      end
    end
  end

  // valid bits are reset, the payload is not (D27)
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      st_vld_q <= '0;
      lt_vld_q <= '0;
    end else begin
      for (int q = 0; q < 4; q++) begin
        st_vld_q[q] <= st_d[q].vld;
        lt_vld_q[q] <= st_vld_q[q];
      end
    end
  end

  always_ff @(posedge clk_i) begin
    for (int q = 0; q < 4; q++) begin
      if (st_d[q].vld)  st_q[q] <= st_d[q];
      if (st_vld_q[q])  lt_q[q] <= st_q[q];
    end
  end

  always_comb begin
    for (int q = 0; q < 4; q++) begin
      la_o[q]     = lt_q[q];
      la_o[q].vld = lt_vld_q[q];
    end
  end

  always_comb begin
    la_loc_o = '0;
    for (int p = 0; p < 4; p++) begin
      if (la_i[p].vld && la_port[p] == P_L) la_loc_o = la_i[p];
    end
  end

  always_comb begin
    busy_o = (st_vld_q != '0) || (lt_vld_q != '0);
    for (int p = 0; p < 4; p++) busy_o = busy_o || (cnt_q[p] != '0);
  end

`ifndef SYNTHESIS
  // Coverage (the testbench reads it; a hook, so outside the synthesis-off
  // pragmas):
  // lookaheads that arrived in the cycle the previous burst's last flit
  // crossed on the same input, i.e. bursts back to back on one link
  longint unsigned n_abut;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      n_abut <= 0;
    end else begin
      for (int p = 0; p < 4; p++) begin
        if (la_i[p].vld && cnt_q[p] == BURST_W'(1)) n_abut <= n_abut + 1;
      end
    end
  end

  // synopsys translate_off
  initial begin
    if (B_MAX < 1 || B_MAX > MAX_BURST)
      $fatal(1, "%m: B_MAX = %0d, must be 1..%0d", B_MAX, MAX_BURST);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      for (int q = 0; q < 4; q++) begin
        if (int'(src_la_vld_i && int'(src_la_port_i) == q) +
            int'(la_i[0].vld && int'(la_port[0]) == q) + int'(la_i[1].vld && int'(la_port[1]) == q) +
            int'(la_i[2].vld && int'(la_port[2]) == q) + int'(la_i[3].vld && int'(la_port[3]) == q) > 1)
          $fatal(1, "%m: two lookaheads for output %0d in one cycle", q);
      end
      if (int'(la_i[0].vld && la_port[0] == P_L) + int'(la_i[1].vld && la_port[1] == P_L) +
          int'(la_i[2].vld && la_port[2] == P_L) + int'(la_i[3].vld && la_port[3] == P_L) > 1)
        $fatal(1, "%m: two lookaheads for the Local output in one cycle");
      if (src_la_vld_i && int'(src_la_port_i) >= 4)
        $fatal(1, "%m: the source launches a lookahead on port %0d", src_la_port_i);
      for (int p = 0; p < 4; p++) begin
        if ($isunknown({la_i[p].vld, flit_i[p].vld}))
          $fatal(1, "%m: X on the lookahead or flit valid of input %0d", p);
        if (la_i[p].vld) begin
          if (la_i[p].len == '0 || int'(la_i[p].len) > B_MAX)
            $fatal(1, "%m: lookahead on input %0d announces %0d flits (B_MAX %0d)", p, la_i[p].len, B_MAX);
          if (int'(la_i[p].dst) >= MESH_N * MESH_N)
            $fatal(1, "%m: lookahead on input %0d for router %0d, outside the mesh", p, la_i[p].dst);
          if (la_port[p] == port_t'(p))
            $fatal(1, "%m: lookahead on input %0d would turn back out of the same port", p);
          if (cnt_q[p] >= BURST_W'(2))
            $fatal(1, "%m: lookahead on input %0d while %0d flits of the last burst are still to come", p, cnt_q[p]);
        end
        if (cnt_q[p] != '0 && !(flit_i[p].vld && flit_i[p].byp))
          $fatal(1, "%m: burst flit missing on input %0d (%0d still announced)", p, cnt_q[p]);
        if (cnt_q[p] == '0 && flit_i[p].vld && flit_i[p].byp)
          $fatal(1, "%m: bypass flit on input %0d without a lookahead", p);
      end
    end
  end
  // synopsys translate_on
`endif

endmodule
