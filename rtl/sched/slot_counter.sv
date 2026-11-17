// =============================================================================
// File      rtl/sched/slot_counter.sv                              Tracker: F34
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   "CLK to Slot" of [1, Fig. 10]: turns clock cycles into the
//           position in the timetable. A slot is a setup window of S_SETUP
//           cycles, then TT_N subslots of T_SUB cycles; slots run 0 ..
//           n(n+1)/2 - 1 and wrap. Also the next-subslot descriptor that the
//           reservation controller (B5) uses to decide a cancel token one
//           subslot ahead.
// Decisions D10 subslot length, D13 setup window, D16 frames (enable),
//           D38 bypass timing
// Tests     U06 (tb/unit/tb_slot_counter.sv)
// -----------------------------------------------------------------------------
// Every router has its own counter. All of them leave reset in the same
// cycle, so they agree without any global wire [1, Sec. III-E].
// Enable    The counter moves only in cycles with en_i = 1, so a timetable
//           that pauses while the other phase runs resumes where it stopped
//           (D16). Its outputs describe the current cycle only when en_i = 1;
//           the pulses (sub_go_o, slot_last_o, nxt_go_o) are 0 otherwise.
//           Design B ties en_i to 1. The frame sequencer (B6) changes phase
//           only after slot_last_o.
// Pulses    sub_go_o     first cycle t of a subslot: sources check for a
//                        cancel token and pick packets (D38)
//           nxt_go_o     a cancel decision: setup cycle 0 decides subslot 0,
//                        the first cycle of subslot k decides subslot k+1;
//                        nxt_sub_o names the subslot decided
//           slot_last_o  last cycle of the slot
// =============================================================================
`timescale 1ns/1ps

module slot_counter
  import sched_pkg::*;
#(
  parameter int TT_N    = 8,                  // timetable side
  parameter int T_SUB   = t_tight(8, 10),     // subslot length, cycles
  parameter int S_SETUP = s_setup(8)          // setup window, cycles
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              en_i,          // this timetable runs in this cycle
  output logic              setup_o,       // in the slot's setup window
  output sub_t              sub_o,         // subslot, 0..n-1 (0 in the setup window)
  output logic [TCYC_W-1:0] cyc_o,         // cycle inside the setup window or the subslot
  output slot_t             slot_o,        // slot, 0..n(n+1)/2 - 1
  output logic              sub_go_o,      // first cycle of a subslot
  output logic              slot_last_o,   // last cycle of the slot
  output logic              nxt_go_o,      // cancel decision for subslot nxt_sub_o
  output sub_t              nxt_sub_o
);

  localparam int NSLOT = n_slots(TT_N);

  logic              setup_q;
  sub_t              sub_q;
  logic [TCYC_W-1:0] cyc_q;
  slot_t             slot_q;
  logic              last_cyc, last_sub;

  assign last_cyc = setup_q ? (cyc_q == TCYC_W'(S_SETUP - 1)) : (cyc_q == TCYC_W'(T_SUB - 1));
  assign last_sub = (sub_q == SUB_W'(TT_N - 1));

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      setup_q <= 1'b1;
      sub_q   <= '0;
      cyc_q   <= '0;
      slot_q  <= '0;
    end else if (en_i) begin
      if (!last_cyc) begin
        cyc_q <= cyc_q + 1'b1;
      end else begin
        cyc_q <= '0;
        if (setup_q) begin
          setup_q <= 1'b0;            // subslot 0 starts
        end else if (!last_sub) begin
          sub_q <= sub_q + 1'b1;
        end else begin                // the slot ends: the next one starts with its setup window
          setup_q <= 1'b1;
          sub_q   <= '0;
          slot_q  <= (slot_q == SLOT_W'(NSLOT - 1)) ? '0 : slot_q + 1'b1;
        end
      end
    end
  end

  assign setup_o     = setup_q;
  assign sub_o       = sub_q;
  assign cyc_o       = cyc_q;
  assign slot_o      = slot_q;
  assign sub_go_o    = en_i && !setup_q && (cyc_q == '0);
  assign slot_last_o = en_i && !setup_q && last_sub && last_cyc;
  assign nxt_go_o    = en_i && (cyc_q == '0) && (setup_q || !last_sub);
  assign nxt_sub_o   = setup_q ? '0 : sub_q + 1'b1;

`ifndef SYNTHESIS
  // synopsys translate_off
  initial begin
    if (TT_N < 2 || TT_N > 16 || !pow2(TT_N))
      $fatal(1, "%m: TT_N = %0d; the timetable side must be 2, 4, 8 or 16", TT_N);
    if (S_SETUP < hmax(TT_N) + 1 || S_SETUP >= (1 << TCYC_W))
      $fatal(1, "%m: S_SETUP = %0d; a cancel token needs %0d cycles to cross the longest path (D13)",
             S_SETUP, hmax(TT_N) + 1);
    if (T_SUB < hmax(TT_N) + 1 || T_SUB >= (1 << TCYC_W))
      $fatal(1, "%m: T_SUB = %0d; a cancel token decided one subslot ahead needs %0d cycles (D38)",
             T_SUB, hmax(TT_N) + 1);
  end
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      // nothing to check during reset
    end else if ($isunknown(en_i)) begin
      $fatal(1, "%m: X on en_i");
    end
  end
  // synopsys translate_on
`endif

endmodule
