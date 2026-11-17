// =============================================================================
// File      rtl/ni/ni.sv                                           Tracker: F20
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Network interface between a core (in this project the testbench
//           traffic source and sink) and the Local port of its router.
// Decisions D14 network interface, D31 NI structure, D32 latency accounting,
//           D43 bypass flits, D12 and D48 reservation
// Tests     N01-N08 (tb/mesh); BP01-BP10 (bypass, B4-B5); U10 (resv_ctrl)
// -----------------------------------------------------------------------------
// Injection
//   The core offers a packet flit by flit, head first, with inj_valid_i and
//   inj_ready_o. A head is taken when one of the router's Local input VCs is
//   idle and gets the lowest idle one, as a router output does (it is the
//   same output_unit, D28). The rest of the packet follows on that VC, one
//   flit per cycle while it has a credit. Packets go one at a time, in the
//   order the core offers them. The flit leaves through the output unit's
//   link register, so it is on the link the cycle after the core offered it.
// Ejection
//   Every flit from the router's Local output enters one FIFO of EJ_DEPTH
//   flits, whatever its VC (D14). The core takes flits in arrival order with
//   ej_valid_o and ej_ready_i. A flit's credit goes back to the router the
//   cycle after the core takes it, so a slow core pushes back into the
//   network; the credit of a tail also frees its VC. EJ_DEPTH is at least
//   NUM_VC * VC_DEPTH, so the router, which sends only against credits,
//   cannot overflow the FIFO with regular flits.
// ej_mon_o is the router's Local output link itself. The testbench timestamps
//   ejection there: a flit is ejected when it reaches the NI, as Garnet counts
//   it (D32).
// Bypass flits (BYPASS = 1, designs B-D): a bypass burst reaches the NI on
//   the same Local output link, as whole packets back to back. Its flits
//   enter the same FIFO with their bypass bit, so the core can tell them
//   from regular flits (ej_flit_o.byp); they took no credit and no VC in the
//   router, so the NI returns no credit when the core takes one (D43).
//   With BYPASS = 0 (design A) the FIFO has no bypass bit, as in B2.
// Reservation (BYPASS = 1, D12, D48; designs C and D add dec_loc_i and the
//   local timetable's lengths GAP0_L, GAPK_L): the FIFO's slots beyond NUM_VC *
//   VC_DEPTH are the bypass space. resv_ctrl decides, one subslot ahead of
//   every burst that may come (dec_*, from the tile's timetable), whether
//   B_MAX of them are free; if not, its cancel token (cx_o) goes back to the
//   burst's source through the router's cancel relay. It follows each
//   accepted burst through the router's lookahead for this NI (la_loc_i) and
//   the bypass flits written and taken, so regular and bypass flits together
//   never exceed EJ_DEPTH, whatever the core does. Designs B-D size the FIFO
//   with sched_pkg::ej_depth_b (48 flits by default), so a sink that keeps
//   up never sees a turn cancelled.
// =============================================================================
`timescale 1ns/1ps

module ni
  import noc_pkg::*;
  import sched_pkg::*;
#(
  parameter int NUM_VC   = 4,     // VCs of the router's Local ports
  parameter int VC_DEPTH = 5,     // flits per VC
  parameter int EJ_DEPTH = 25,    // ejection FIFO, flits (D31; designs B-D: ej_depth_b, D48)
  parameter int BYPASS   = 0,     // 1: bypass flits arrive (designs B-D)
  parameter int B_MAX    = 10,    // flits per burst (BYPASS = 1)
  parameter int GAP0     = 15,    // decision -> start of subslot 0: the setup window (BYPASS = 1)
  parameter int GAPK     = 34,    // decision -> start of a later subslot: one subslot (BYPASS = 1)
  parameter int GAP0_L   = 15,    // the same for the local timetable (designs C, D)
  parameter int GAPK_L   = 34,
  parameter int EFW      = $clog2(EJ_DEPTH + 1)   // derived, do not set
) (
  input  logic           clk_i,
  input  logic           rst_ni,
  // core: injection
  input  logic           inj_valid_i,
  output logic           inj_ready_o,
  input  flit_t          inj_flit_i,   // ftype and data are used; vld, vc, byp are not
  // router Local input port
  output flit_t          link_o,
  input  credit_t        credit_i,
  // router Local output port
  input  flit_t          ej_link_i,
  output credit_t        credit_o,
  // core: ejection
  output logic           ej_valid_o,
  input  logic           ej_ready_i,
  output flit_t          ej_flit_o,
  output logic [EFW-1:0] ej_free_o,    // free FIFO slots
  output flit_t          ej_mon_o,     // the router's Local output link
  // reservation (BYPASS = 1): a burst may come in the next subslot
  input  logic           dec_i,
  input  logic           dec_s0_i,     // ... it is subslot 0
  input  logic           dec_loc_i,    // ... of the local timetable (designs C, D)
  input  node_id_t       dec_src_i,    // ... from this source
  input  logic           dec_mode_i,   // ... by this mode
  input  hop_t           dec_hops_i,   // ... over this many hops
  input  la_t            la_loc_i,     // the router: lookahead of a burst for this NI
  output cx_t            cx_o          // to the router: cancel token
);

  // ---------------------------------------------------------------- injection
  logic                in_pkt_q;       // a packet is part way through
  vc_id_t              cur_vc_q;       // its VC
  logic                has_free;
  vc_id_t              alloc_vc;
  logic [NUM_VC-1:0]   credit_ok;
  logic [VC_CNT_W-1:0] unused_free_cnt;
  logic                head, cur_cok, acc;
  vc_id_t              use_vc;
  flit_t               inj_st;

  assign head = is_head(inj_flit_i.ftype);

  // Credit of the packet's VC; a loop keeps the index inside NUM_VC
  always_comb begin
    cur_cok = 1'b0;
    for (int v = 0; v < NUM_VC; v++) begin
      if (cur_vc_q == vc_id_t'(v)) cur_cok = credit_ok[v];
    end
  end

  // A head needs an idle VC; any other flit a credit on its packet's VC
  assign inj_ready_o = head ? (!in_pkt_q && has_free) : (in_pkt_q && cur_cok);
  assign acc         = inj_valid_i & inj_ready_o;
  assign use_vc      = head ? alloc_vc : cur_vc_q;

  always_comb begin
    inj_st       = '0;
    inj_st.vld   = acc;
    inj_st.ftype = inj_flit_i.ftype;
    inj_st.vc    = use_vc;
    inj_st.data  = inj_flit_i.data;
  end

  output_unit #(.NUM_VC(NUM_VC), .VC_DEPTH(VC_DEPTH)) u_inj (
    .clk_i       (clk_i),
    .rst_ni      (rst_ni),
    .credit_i    (credit_i),
    .gnt_i       (acc),
    .alloc_i     (acc & head),
    .gnt_vc_i    (use_vc),
    .has_free_o  (has_free),
    .alloc_vc_o  (alloc_vc),
    .credit_ok_o (credit_ok),
    .free_cnt_o  (unused_free_cnt),
    .st_flit_i   (inj_st),
    .link_o      (link_o)
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      in_pkt_q <= 1'b0;
      cur_vc_q <= '0;
    end else if (acc) begin
      if (head) cur_vc_q <= alloc_vc;
      in_pkt_q <= !is_tail(inj_flit_i.ftype);
    end
  end

  // ---------------------------------------------------------------- ejection
  localparam int FTW = $bits(ftype_t);
  localparam int BW  = (BYPASS != 0) ? 1 : 0;
  localparam int EW  = BW + FTW + VC_ID_W + FLIT_DATA_W;   // stored flit: [bypass,] type, VC, data

  logic           ej_empty, unused_full, ej_pop, ej_byp;
  logic [EFW-1:0] ej_cnt;
  logic [EW-1:0]  ej_rdata, ej_wdata;
  ftype_t         ej_ftype;
  vc_id_t         ej_vc;

  if (BYPASS != 0) begin : g_byp
    localparam int SPACE = EJ_DEPTH - NUM_VC * VC_DEPTH;     // the bypass space (D48)
    assign ej_wdata = {ej_link_i.byp, ej_link_i.ftype, ej_link_i.vc, ej_link_i.data};
    assign ej_byp   = ej_rdata[EW-1];

    logic [$clog2(SPACE + 1)-1:0] res, fill;
    resv_ctrl #(
      .B_MAX (B_MAX),
      .SPACE (SPACE),
      .GAP0   (GAP0),
      .GAPK   (GAPK),
      .GAP0_L (GAP0_L),
      .GAPK_L (GAPK_L)
    ) u_resv (
      .clk_i      (clk_i),
      .rst_ni     (rst_ni),
      .dec_i      (dec_i),
      .dec_s0_i   (dec_s0_i),
      .dec_loc_i  (dec_loc_i),
      .dec_src_i  (dec_src_i),
      .dec_mode_i (dec_mode_i),
      .dec_hops_i (dec_hops_i),
      .la_i       (la_loc_i),
      .push_i     (ej_link_i.vld & ej_link_i.byp),
      .pop_i      (ej_pop & ej_byp),
      .cx_o       (cx_o),
      .res_o      (res),
      .fill_o     (fill)
    );
    logic unused_cnt;                  // the testbench reads the counts in u_resv
    assign unused_cnt = ^{res, fill};
  end else begin : g_no_byp
    assign ej_wdata = {ej_link_i.ftype, ej_link_i.vc, ej_link_i.data};
    assign ej_byp   = 1'b0;
    assign cx_o     = '0;
    logic unused_resv;
    assign unused_resv = ^{dec_i, dec_s0_i, dec_loc_i, dec_src_i, dec_mode_i, dec_hops_i, la_loc_i};
  end

  sync_fifo #(.WIDTH(EW), .DEPTH(EJ_DEPTH)) u_ej (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),
    .push_i  (ej_link_i.vld),
    .wdata_i (ej_wdata),
    .pop_i   (ej_pop),
    .rdata_o (ej_rdata),
    .empty_o (ej_empty),
    .full_o  (unused_full),
    .count_o (ej_cnt)
  );

  assign ej_ftype   = ej_rdata[FTW + VC_ID_W + FLIT_DATA_W - 1 -: FTW];
  assign ej_vc      = ej_rdata[FLIT_DATA_W +: VC_ID_W];
  assign ej_valid_o = !ej_empty;
  assign ej_pop     = ej_valid_o & ej_ready_i;
  assign ej_free_o  = EFW'(EJ_DEPTH) - ej_cnt;
  assign ej_mon_o   = ej_link_i;

  always_comb begin
    ej_flit_o       = '0;
    ej_flit_o.vld   = ej_valid_o;
    ej_flit_o.ftype = ej_ftype;
    ej_flit_o.vc    = ej_vc;
    ej_flit_o.byp   = ej_byp;
    ej_flit_o.data  = ej_rdata[FLIT_DATA_W-1:0];
  end

  // Credit for a regular flit the core took; 1-cycle credit link, as a
  // router's. A bypass flit took no credit, so none goes back.
  credit_t credit_q;
  logic    cred;

  assign cred = ej_pop & ~ej_byp;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      credit_q <= '0;
    end else begin
      credit_q.vld     <= cred;
      credit_q.vc_free <= cred & is_tail(ej_ftype);
      if (cred) credit_q.vc <= ej_vc;
    end
  end

  assign credit_o = credit_q;

  // The core's vld, vc and byp bits are not used; nor is the link's byp bit
  // in design A
  logic unused_bits;
  assign unused_bits = ^{inj_flit_i.vld, inj_flit_i.vc, inj_flit_i.byp, ej_link_i.byp,
                         unused_full, unused_free_cnt};

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (EJ_DEPTH < NUM_VC * VC_DEPTH)
      $fatal(1, "%m: EJ_DEPTH = %0d is below NUM_VC * VC_DEPTH = %0d", EJ_DEPTH, NUM_VC * VC_DEPTH);
    if (BYPASS != 0 && EJ_DEPTH < NUM_VC * VC_DEPTH + B_MAX)
      $fatal(1, "%m: EJ_DEPTH = %0d leaves no space for a burst of %0d flits beside %0d regular ones",
             EJ_DEPTH, B_MAX, NUM_VC * VC_DEPTH);
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else begin
      if ($isunknown({inj_valid_i, ej_ready_i, ej_link_i.vld}))
        $fatal(1, "%m: X on inj_valid_i, ej_ready_i or the ejection link valid bit");
      if (inj_valid_i && head && in_pkt_q)
        $fatal(1, "%m: core offered a head before the tail of its previous packet");
      if (inj_valid_i && !head && !in_pkt_q)
        $fatal(1, "%m: core offered a body or tail flit without a head");
      if (ej_link_i.vld && unused_full && !ej_pop)
        $fatal(1, "%m: ejection FIFO overflow: a flit arrived with no free slot");
      if (BYPASS == 0 && ej_link_i.vld && ej_link_i.byp)
        $fatal(1, "%m: bypass flit at the NI of a design without bypass");
    end
  end
  // synopsys translate_on
`endif

endmodule
