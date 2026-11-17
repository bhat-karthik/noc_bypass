// =============================================================================
// File      tb/common/run_ctrl.sv                                  Tracker: F28
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Run control for mesh tests: warm-up, measurement window and
//           drain; deadlock detection; the stall metrics M17.
// Decisions D34 run control
// Tests     N04, N05, N08 and every mesh run
// -----------------------------------------------------------------------------
// run_load(warm, meas, drain_max): generators on for warm + meas cycles with
//   the window [warm, warm + meas) open, then off; then drain until every
//   packet created is delivered.
// Deadlock: packets are in flight and none has been delivered for DL_LIMIT
//   cycles. A network that still works delivers something somewhere every
//   few cycles, even far past saturation; a deadlocked one never does again.
//   The run stops, stalls goes to 1 and t_stall records the cycle of the last
//   delivery (time to deadlock, M17). A forced sink stall must stay shorter
//   than DL_LIMIT.
// The tasks return 0 when a deadlock or the drain limit stopped them.
// @support tb_mesh tb_mesh_dl tb_mesh_b tb_mesh_c tb_mesh_cb tb_mesh_d
// (the EDA bundle compiles this module only for the tests of those benches)
// =============================================================================
`timescale 1ns/1ps

module run_ctrl (
  input logic        clk,
  input int unsigned cyc
);

  int unsigned dl_limit = 2000;    // cycles without a delivery that mean deadlock
  int          stalls   = 0;       // deadlocks seen (M17)
  int unsigned t_stall  = 0;       // cycle of the last delivery before the first one

  function automatic bit deadlocked();
    return sb_pkg::outstanding() != 0 && (cyc - sb_pkg::last_delivery) > dl_limit;
  endfunction

  // n cycles, watching for deadlock
  task automatic wait_cycles(int unsigned n, output bit ok);
    ok = 1'b1;
    sb_pkg::last_delivery = cyc;
    repeat (n) begin
      @(posedge clk);
      if (deadlocked()) begin
        if (stalls == 0) t_stall = sb_pkg::last_delivery;
        stalls++;
        ok = 1'b0;
        return;
      end
    end
  endtask

  // until every packet created has been delivered, at most max cycles
  task automatic drain(int unsigned max, output bit ok);
    int unsigned t0;
    t0 = cyc;
    ok = 1'b1;
    sb_pkg::last_delivery = cyc;
    while (sb_pkg::outstanding() != 0) begin
      @(posedge clk);
      if (deadlocked()) begin
        if (stalls == 0) t_stall = sb_pkg::last_delivery;
        stalls++;
        ok = 1'b0;
        return;
      end
      if (cyc - t0 > max) begin
        ok = 1'b0;
        return;
      end
    end
  endtask

  task automatic run_load(int unsigned warm, int unsigned meas, int unsigned drain_max, output bit ok);
    stats_pkg::open_window(cyc + warm + 1, cyc + warm + meas + 1);
    traffic_pkg::gen_on = 1'b1;
    wait_cycles(warm + meas, ok);
    traffic_pkg::gen_on = 1'b0;
    if (ok) drain(drain_max, ok);
  endtask

endmodule
