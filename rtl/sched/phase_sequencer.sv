// =============================================================================
// File      rtl/sched/phase_sequencer.sv                           Tracker: F50
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The two-level timetable of designs C and D: a local slot counter
//           (side REGION_N; every region runs the same local timetable at
//           the same time, in its own coordinates) and a global one (side
//           MESH_N), frames of k_L local slots and one global slot, the plan
//           register and the epoch counter.
// Decisions D15 regions, D16 frames, D19 epoch and plan switch, D50 one
//           pointer when the region is the mesh
// Tests     U11 (tb/unit/tb_phase_sequencer.sv); TL01-TL06 (tb_mesh_c,
//           tb_mesh_cb)
// -----------------------------------------------------------------------------
// Frames    A frame is k_L local slots, then one global slot. Only the running
//           timetable's counter is enabled, so each resumes where it stopped
//           (D16): the local counter halts after the last cycle of the
//           frame's k_L-th local slot, and the global counter starts its
//           next slot (setup window first) in the next cycle; after that
//           global slot the next frame starts. The phase therefore changes
//           only at slot boundaries. k_L = 0 makes every frame one global
//           slot: design B's schedule. The first frame after reset has
//           k_L = K_INIT.
// Plan      The epoch counter runs from reset; its cycle 0 is an epoch tick
//           (tick_o; reset counts as the first). In the cycle A_DLY - 1 after
//           every tick plan_i is sampled (registered at the end of that
//           cycle); the sampled value becomes k_L from the first frame that
//           starts after that cycle, so a new plan never starts less than
//           A_DLY cycles after its tick (D19). A sample that has not taken
//           effect yet when the next one comes is replaced by it. Every
//           tile has its own sequencer; all leave reset together and see the
//           same plan_i, so all of them switch in the same cycle. Any k_L
//           that fits KL_W bits works; D18's limits (K_MIN, K_MAX) are the
//           controller's (B7).
// One pointer  REGION_N = MESH_N: the region is the whole mesh. Only the
//           global counter exists and it runs every cycle; loc_o and k_L read
//           0, plan_i is not used, and the tile behaves exactly as design
//           B's (D50, checked by TL05).
// Outputs   Both counters' slots, subslots and next subslots (each drives
//           its own slot table and partner logic in the tile); loc_o names
//           the timetable that runs this cycle, and sub_go_o and nxt_go_o
//           are its pulses (slot_counter gates the other one's to 0). st_o
//           packs the whole state for the testbench's lockstep check.
// =============================================================================
`timescale 1ns/1ps

module phase_sequencer
  import sched_pkg::*;
#(
  parameter int MESH_N   = 8,
  parameter int REGION_N = 4,
  parameter int T_SUB_G  = t_tight(8, 10),   // global subslot, cycles
  parameter int T_SUB_L  = t_tight(4, 10),   // local subslot, cycles
  parameter int K_INIT   = 4,                // k_L from reset
  parameter int EPOCH    = 1 << 15,          // epoch, cycles
  parameter int A_DLY    = A_DELAY,          // epoch tick -> plan sampled
  parameter int EW       = $clog2(EPOCH)     // derived, do not set
) (
  input  logic    clk_i,
  input  logic    rst_ni,
  input  kl_t     plan_i,        // k_L for the frames after the next sample
  output logic    loc_o,         // the local timetable runs this cycle
  output slot_t   slot_l_o,      // local counter
  output sub_t    sub_l_o,
  output sub_t    nxt_sub_l_o,
  output slot_t   slot_g_o,      // global counter
  output sub_t    sub_g_o,
  output sub_t    nxt_sub_g_o,
  output logic    sub_go_o,      // first cycle of a subslot (running timetable)
  output logic    nxt_go_o,      // cancel decision for its next subslot
  output kl_t     kl_o,          // k_L of the current frame
  output logic    tick_o,        // first cycle of an epoch
  output seq_st_t st_o           // whole state (testbench)
);

  // ---------------------------------------------------------------- epoch
  logic [EW-1:0] ep_q;
  logic          samp;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni)                         ep_q <= '0;
    else if (ep_q == EW'(EPOCH - 1))     ep_q <= '0;
    else                                 ep_q <= ep_q + 1'b1;
  end

  assign tick_o = (ep_q == '0);
  assign samp   = (ep_q == EW'(A_DLY - 1));

  if (REGION_N == MESH_N) begin : g_one
    // ---------------------------------------------------------------- one pointer (D50)
    logic              g_setup, g_go, g_last, g_ngo;
    sub_t              g_sub, g_nsub;
    logic [TCYC_W-1:0] g_cyc;
    slot_t             g_slot;

    slot_counter #(
      .TT_N    (MESH_N),
      .T_SUB   (T_SUB_G),
      .S_SETUP (s_setup(MESH_N))
    ) u_glb (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .en_i        (1'b1),
      .setup_o     (g_setup),
      .sub_o       (g_sub),
      .cyc_o       (g_cyc),
      .slot_o      (g_slot),
      .sub_go_o    (g_go),
      .slot_last_o (g_last),
      .nxt_go_o    (g_ngo),
      .nxt_sub_o   (g_nsub)
    );

    assign loc_o       = 1'b0;
    assign slot_l_o    = '0;
    assign sub_l_o     = '0;
    assign nxt_sub_l_o = '0;
    assign slot_g_o    = g_slot;
    assign sub_g_o     = g_sub;
    assign nxt_sub_g_o = g_nsub;
    assign sub_go_o    = g_go;
    assign nxt_go_o    = g_ngo;
    assign kl_o        = '0;

    always_comb begin
      st_o         = '0;
      st_o.g_setup = g_setup;
      st_o.g_sub   = g_sub;
      st_o.g_cyc   = g_cyc;
      st_o.g_slot  = g_slot;
    end

    logic unused_one;
    assign unused_one = ^{g_last, samp, plan_i};
  end else begin : g_two
    // ---------------------------------------------------------------- two pointers, frames
    logic              l_setup, l_go, l_last, l_ngo;
    logic              g_setup, g_go, g_last, g_ngo;
    sub_t              l_sub, l_nsub, g_sub, g_nsub;
    logic [TCYC_W-1:0] l_cyc, g_cyc;
    slot_t             l_slot, g_slot;
    logic              loc_q, pend_vld_q, apply;
    kl_t               k_q, lc_q, pend_q, k_nxt, k_new;

    slot_counter #(
      .TT_N    (REGION_N),
      .T_SUB   (T_SUB_L),
      .S_SETUP (s_setup(REGION_N))
    ) u_loc (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .en_i        (loc_q),
      .setup_o     (l_setup),
      .sub_o       (l_sub),
      .cyc_o       (l_cyc),
      .slot_o      (l_slot),
      .sub_go_o    (l_go),
      .slot_last_o (l_last),
      .nxt_go_o    (l_ngo),
      .nxt_sub_o   (l_nsub)
    );

    slot_counter #(
      .TT_N    (MESH_N),
      .T_SUB   (T_SUB_G),
      .S_SETUP (s_setup(MESH_N))
    ) u_glb (
      .clk_i       (clk_i),
      .rst_ni      (rst_ni),
      .en_i        (!loc_q),
      .setup_o     (g_setup),
      .sub_o       (g_sub),
      .cyc_o       (g_cyc),
      .slot_o      (g_slot),
      .sub_go_o    (g_go),
      .slot_last_o (g_last),
      .nxt_go_o    (g_ngo),
      .nxt_sub_o   (g_nsub)
    );

    // k_L of the next frame: the plan sampled in this cycle, else the one
    // waiting; it takes effect when the frame ends (the global slot's last
    // cycle)
    assign k_nxt = samp ? plan_i : pend_q;
    assign apply = g_last && (samp || pend_vld_q);
    assign k_new = apply ? k_nxt : k_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        loc_q      <= (K_INIT != 0);
        k_q        <= kl_t'(K_INIT);
        lc_q       <= '0;
        pend_q     <= '0;
        pend_vld_q <= 1'b0;
      end else begin
        if (samp) begin
          pend_q     <= plan_i;
          pend_vld_q <= 1'b1;
        end
        if (apply) pend_vld_q <= 1'b0;
        if (l_last) begin                     // a local slot ends ...
          if (lc_q == k_q - 1'b1) begin       // ... the frame's last: the global slot is next
            loc_q <= 1'b0;
            lc_q  <= '0;
          end else begin
            lc_q  <= lc_q + 1'b1;
          end
        end
        if (g_last) begin                     // the frame ends
          k_q   <= k_new;
          loc_q <= (k_new != '0);
          lc_q  <= '0;
        end
      end
    end

    assign loc_o       = loc_q;
    assign slot_l_o    = l_slot;
    assign sub_l_o     = l_sub;
    assign nxt_sub_l_o = l_nsub;
    assign slot_g_o    = g_slot;
    assign sub_g_o     = g_sub;
    assign nxt_sub_g_o = g_nsub;
    assign sub_go_o    = l_go | g_go;
    assign nxt_go_o    = l_ngo | g_ngo;
    assign kl_o        = k_q;

    always_comb begin
      st_o.loc     = loc_q;
      st_o.kl      = k_q;
      st_o.lc      = lc_q;
      st_o.l_setup = l_setup;
      st_o.l_sub   = l_sub;
      st_o.l_cyc   = l_cyc;
      st_o.l_slot  = l_slot;
      st_o.g_setup = g_setup;
      st_o.g_sub   = g_sub;
      st_o.g_cyc   = g_cyc;
      st_o.g_slot  = g_slot;
    end

`ifndef SYNTHESIS
    // synopsys translate_off
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        // nothing to check during reset
      end else begin
        if (samp && $isunknown(plan_i))
          $fatal(1, "%m: X on plan_i when it is sampled");
        if (loc_q && k_q == '0)
          $fatal(1, "%m: local phase with k_L = 0");
        if (int'(lc_q) >= int'(k_q) && k_q != '0)
          $fatal(1, "%m: %0d local slots finished in a frame of k_L = %0d", lc_q, k_q);
      end
    end
    // synopsys translate_on
`endif
  end

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (REGION_N < 2 || !pow2(REGION_N) || REGION_N > MESH_N || MESH_N % REGION_N != 0)
      $fatal(1, "%m: REGION_N = %0d, MESH_N = %0d; the region side must be a power of two that divides the mesh side",
             REGION_N, MESH_N);
    if (K_INIT < 0 || K_INIT >= (1 << KL_W))
      $fatal(1, "%m: K_INIT = %0d; k_L must be 0..%0d", K_INIT, (1 << KL_W) - 1);
    if (A_DLY < 1 || EPOCH <= A_DLY || EW < 1)
      $fatal(1, "%m: EPOCH = %0d, A_DLY = %0d; the plan must be sampled inside the epoch", EPOCH, A_DLY);
  end
  // synopsys translate_on
`endif

endmodule
