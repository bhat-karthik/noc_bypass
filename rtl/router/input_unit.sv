// =============================================================================
// File      rtl/router/input_unit.sv                               Tracker: F13
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   One router input port: NUM_VC virtual channels of VC_DEPTH flits,
//           buffer write and route computation (stage 1, BW+RC), the per-VC
//           state the switch allocator reads, the SA->ST pipeline register,
//           the credit returned upstream, and the read port through which the
//           bypass source (F41) takes whole packets out of the VCs (B4).
// Decisions D03 pipeline, D04 and D05 one packet per VC and credits,
//           D06 routing, D26 route chosen once, D27 registers, D28 allocation,
//           D42 bypass source
// Tests     R01-R04 (tb/unit/tb_router.sv); BP00-BP05 (bypass, B4)
// -----------------------------------------------------------------------------
// Stage 1, BW+RC (cycle t): a flit on flit_i is written into VC flit_i.vc at
//   the end of the cycle. A head also gets its output port from
//   route_compute in the same cycle and keeps it while it waits for an
//   output VC (D26, as Garnet does).
// Stage 2, VA+SA (cycle t+1 or later): every VC with a flit at its front
//   asks the switch allocator. When it wins, the flit leaves the buffer at the
//   end of the cycle into the SA->ST register (sa_flit_o), stamped with its
//   output VC. The credit for the slot it frees goes upstream in the next
//   cycle (credit_o, the 1-cycle credit link); the credit of a tail also says
//   the VC is free for a new packet.
// VC states IDLE (no packet), ROUTED (head waiting for an output VC),
//   ACTIVE (output VC held; each flit goes when it wins the switch),
//   BYPASS (the bypass source took the packet; it leaves through bp_pop_i).
// Bypass read port (designs B-D; tied to zero in design A):
//   bp_rdy_o[v]    the VC holds a whole packet, head at the front, nothing
//                  sent yet: a packet the source may take (D42)
//   bp_front_o[v]  the VC's front flit (vld = not empty)
//   bp_hold_i[v]   the source takes the packet in this cycle: the VC leaves
//                  switch allocation now and is BYPASS from the next cycle
//   bp_pop_i[v]    the front flit leaves for the bypass in this cycle; the
//                  credit goes upstream in the next cycle, as for a regular
//                  flit. The router masks switch allocation for this input
//                  in a cycle with a bypass pop, so one flit and one credit
//                  leave per cycle at most.
// Bypass flits passing through (flit_i.byp) never enter the buffers.
// =============================================================================
`timescale 1ns/1ps

module input_unit
  import noc_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int NUM_VC   = 4,            // 1..4
  parameter int VC_DEPTH = 5,            // flits per VC: one 5-flit packet
  parameter int ALGO     = RT_ADAPTIVE   // RT_XY or RT_ADAPTIVE
) (
  input  logic                            clk_i,
  input  logic                            rst_ni,
  input  coord_t                          cur_x_i,     // this router
  input  coord_t                          cur_y_i,
  // link from the upstream router (or from the NI on the Local port)
  input  flit_t                           flit_i,
  output credit_t                         credit_o,
  // free VCs behind every output port, for adaptive routing
  input  logic [NPORTS-1:0][VC_CNT_W-1:0] free_vc_i,
  // to the switch allocator
  output logic   [NUM_VC-1:0]             req_o,       // a flit waits at the front of the VC
  output port_t  [NUM_VC-1:0]             port_o,      // output port of the VC's packet
  output logic   [NUM_VC-1:0]             active_o,    // its output VC is already held
  output vc_id_t [NUM_VC-1:0]             outvc_o,     // that output VC
  // from the switch allocator
  input  logic                            gnt_i,       // the front flit of gnt_vc_i goes now
  input  vc_id_t                          gnt_vc_i,
  input  vc_id_t                          gnt_outvc_i, // output VC it travels on
  // SA->ST pipeline register, to the crossbar
  output flit_t                           sa_flit_o,
  output port_t                           sa_port_o,
  // bypass read port, to and from the bypass source (zero in design A)
  output logic   [NUM_VC-1:0]             bp_rdy_o,    // a whole packet waits, nothing sent
  output flit_t  [NUM_VC-1:0]             bp_front_o,  // front flit of each VC
  input  logic   [NUM_VC-1:0]             bp_hold_i,   // the source takes these packets now
  input  logic   [NUM_VC-1:0]             bp_pop_i     // front flit leaves for the bypass
);

  localparam int FTW = $bits(ftype_t);
  localparam int FW  = FTW + FLIT_DATA_W;   // a stored flit: type and data

  typedef enum logic [1:0] {
    VC_IDLE   = 2'd0,
    VC_ROUTED = 2'd1,
    VC_ACTIVE = 2'd2,
    VC_BYPASS = 2'd3
  } vc_state_e;

  localparam int CW  = $clog2(VC_DEPTH + 1);

  vc_state_e         state_q [NUM_VC];
  port_t             port_q  [NUM_VC];
  vc_id_t            outvc_q [NUM_VC];
  logic [NUM_VC-1:0] push, pop, gpop, empty;
  logic [FW-1:0]     rdata   [NUM_VC];
  logic [CW-1:0]     count   [NUM_VC];

  // ---------------------------------------------------------------- BW + RC
  logic  wr;          // a flit for the buffers arrives
  head_t in_head;
  port_t rc_port;

  assign wr      = flit_i.vld & ~flit_i.byp;
  assign in_head = head_t'(flit_i.data);

  route_compute #(.MESH_N(MESH_N), .ALGO(ALGO)) u_rc (
    .cur_x_i   (cur_x_i),
    .cur_y_i   (cur_y_i),
    .dst_i     (in_head.dst),
    .mode_i    (MODE_XY),
    .free_vc_i (free_vc_i),
    .port_o    (rc_port)
  );

  // ---------------------------------------------------------------- VCs
  logic [NUM_VC-1:0] unused_full;
  head_t             front_head [NUM_VC];

  for (genvar v = 0; v < NUM_VC; v++) begin : g_vc
    assign push[v] = wr & (flit_i.vc == vc_id_t'(v));
    assign gpop[v] = gnt_i & (gnt_vc_i == vc_id_t'(v));     // regular: won the switch
    assign pop[v]  = gpop[v] | bp_pop_i[v];                 // or taken by the bypass

    sync_fifo #(.WIDTH(FW), .DEPTH(VC_DEPTH)) u_buf (
      .clk_i   (clk_i),
      .rst_ni  (rst_ni),
      .push_i  (push[v]),
      .wdata_i ({flit_i.ftype, flit_i.data}),
      .pop_i   (pop[v]),
      .rdata_o (rdata[v]),
      .empty_o (empty[v]),
      .full_o  (unused_full[v]),
      .count_o (count[v])
    );

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        state_q[v] <= VC_IDLE;
        port_q[v]  <= P_L;
        outvc_q[v] <= '0;
      end else begin
        if (push[v] && is_head(flit_i.ftype)) begin
          state_q[v] <= VC_ROUTED;
          port_q[v]  <= rc_port;
        end
        if (bp_hold_i[v]) begin
          state_q[v] <= VC_BYPASS;                      // the bypass source took the packet
        end
        if (gpop[v]) begin
          if (is_tail(rdata[v][FW-1 -: FTW])) begin
            state_q[v] <= VC_IDLE;                      // the packet has left
          end else if (is_head(rdata[v][FW-1 -: FTW])) begin
            state_q[v] <= VC_ACTIVE;                    // head won a VC and the switch
            outvc_q[v] <= gnt_outvc_i;
          end
        end
        if (bp_pop_i[v] && is_tail(rdata[v][FW-1 -: FTW])) begin
          state_q[v] <= VC_IDLE;                        // the bypassed packet has left
        end
      end
    end

    // A VC competes for the switch while it holds a regular packet; a packet
    // the bypass source takes in this cycle leaves allocation at once
    assign req_o[v]    = ~empty[v] & (state_q[v] == VC_ROUTED || state_q[v] == VC_ACTIVE) & ~bp_hold_i[v];
    assign port_o[v]   = port_q[v];
    assign active_o[v] = (state_q[v] == VC_ACTIVE);
    assign outvc_o[v]  = outvc_q[v];

    // Bypass read port: a whole packet (D04: one per VC) whose head is still
    // at the front; the head's length says how many flits make it whole
    assign front_head[v] = head_t'(rdata[v][FLIT_DATA_W-1:0]);
    assign bp_rdy_o[v]   = (state_q[v] == VC_ROUTED) && (int'(count[v]) == int'(front_head[v].len));
    always_comb begin
      bp_front_o[v]       = '0;
      bp_front_o[v].vld   = ~empty[v];
      bp_front_o[v].ftype = rdata[v][FW-1 -: FTW];
      bp_front_o[v].vc    = vc_id_t'(v);
      bp_front_o[v].data  = rdata[v][FLIT_DATA_W-1:0];
    end
  end

  // ---------------------------------------------------------------- SA -> ST
  logic [FW-1:0] gnt_data;      // front flit of the granted VC
  port_t         gnt_port;

  always_comb begin
    gnt_data = rdata[0];
    gnt_port = port_q[0];
    for (int v = 1; v < NUM_VC; v++) begin
      if (gnt_vc_i == vc_id_t'(v)) begin
        gnt_data = rdata[v];
        gnt_port = port_q[v];
      end
    end
  end

  // The VC and flit type of a bypass pop (at most one per cycle)
  logic   bp_any;
  vc_id_t bp_vc;
  ftype_t bp_ftype;

  always_comb begin
    bp_any   = |bp_pop_i;
    bp_vc    = '0;
    bp_ftype = rdata[0][FW-1 -: FTW];
    for (int v = 0; v < NUM_VC; v++) begin
      if (bp_pop_i[v]) begin
        bp_vc    = vc_id_t'(v);
        bp_ftype = rdata[v][FW-1 -: FTW];
      end
    end
  end

  logic                   sa_vld_q;
  ftype_t                 sa_ftype_q;
  vc_id_t                 sa_vc_q;
  logic [FLIT_DATA_W-1:0] sa_data_q;
  port_t                  sa_port_q;
  credit_t                credit_q;

  // Valid bits and credits are reset; the payload is not (D27). A flit that
  // leaves for the bypass returns its credit exactly as a regular one does.
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sa_vld_q <= 1'b0;
      credit_q <= '0;
    end else begin
      sa_vld_q         <= gnt_i;
      credit_q.vld     <= gnt_i | bp_any;
      credit_q.vc      <= gnt_i ? gnt_vc_i : bp_vc;
      credit_q.vc_free <= gnt_i ? is_tail(gnt_data[FW-1 -: FTW]) : (bp_any & is_tail(bp_ftype));
    end
  end

  always_ff @(posedge clk_i) begin
    if (gnt_i) begin
      sa_ftype_q <= gnt_data[FW-1 -: FTW];
      sa_vc_q    <= gnt_outvc_i;
      sa_data_q  <= gnt_data[FLIT_DATA_W-1:0];
      sa_port_q  <= gnt_port;
    end
  end

  always_comb begin
    sa_flit_o       = '0;
    sa_flit_o.vld   = sa_vld_q;
    sa_flit_o.ftype = sa_ftype_q;
    sa_flit_o.vc    = sa_vc_q;
    sa_flit_o.data  = sa_data_q;
  end

  assign sa_port_o = sa_port_q;
  assign credit_o  = credit_q;

  // Only the head's destination is read from an incoming flit, and only the
  // length from a VC's front head
  logic unused_bits;
  always_comb begin
    unused_bits = ^{unused_full, in_head.rsvd_hi, in_head.check, in_head.t_create,
                    in_head.seq, in_head.len, in_head.rsvd_lo, in_head.cls, in_head.src};
    for (int v = 0; v < NUM_VC; v++)
      unused_bits = unused_bits ^ (^{front_head[v].rsvd_hi, front_head[v].check, front_head[v].t_create,
                                     front_head[v].seq, front_head[v].rsvd_lo, front_head[v].cls,
                                     front_head[v].src, front_head[v].dst});
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      if ($isunknown({flit_i.vld, gnt_i}))
        $fatal(1, "%m: X on flit_i.vld or gnt_i");
      if (wr && int'(flit_i.vc) >= NUM_VC)
        $fatal(1, "%m: flit for VC %0d, only %0d VCs", flit_i.vc, NUM_VC);
      if (gnt_i && (int'(gnt_vc_i) >= NUM_VC || !(|(req_o & (NUM_VC'(1) << gnt_vc_i)))))
        $fatal(1, "%m: grant for VC %0d, which has nothing to send", gnt_vc_i);
      for (int v = 0; v < NUM_VC; v++) begin
        if (push[v] && is_head(flit_i.ftype) && state_q[v] != VC_IDLE)
          $fatal(1, "%m: head arrived on VC %0d, which still holds a packet", v);
        if (push[v] && !is_head(flit_i.ftype) && state_q[v] == VC_IDLE)
          $fatal(1, "%m: body or tail arrived on idle VC %0d", v);
        if (push[v] && state_q[v] == VC_BYPASS)
          $fatal(1, "%m: flit arrived on VC %0d, whose whole packet the bypass already took", v);
        if (bp_hold_i[v] && !bp_rdy_o[v])
          $fatal(1, "%m: bypass took VC %0d, which holds no whole waiting packet", v);
        if (bp_pop_i[v] && (state_q[v] != VC_BYPASS || empty[v]))
          $fatal(1, "%m: bypass pop from VC %0d in state %0d (empty %0b)", v, state_q[v], empty[v]);
      end
      if ($isunknown({bp_hold_i, bp_pop_i}))
        $fatal(1, "%m: X on bp_hold_i or bp_pop_i");
      if ($countones(bp_pop_i) > 1 || (bp_any && gnt_i))
        $fatal(1, "%m: two flits leave in one cycle (bypass pops %b, switch grant %0b)", bp_pop_i, gnt_i);
    end
  end
  // synopsys translate_on
`endif

endmodule
