// =============================================================================
// File      tb/common/tb_macros.svh                     Tracker: F05 (with tb_pkg)
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Check and waveform macros for every testbench. Include it at the
//           top of each testbench file; tb_pkg.sv must be compiled first.
// -----------------------------------------------------------------------------
//   `TB_CHECK(cond, ("format", args...))
//       Counts one check. The message is formatted only when the check fails,
//       so a million passing checks cost no string work. An X or Z condition
//       fails: only a clean 1 passes.
//   `TB_CHECK_EQ(got, expected, "what")
//       Four-state compare (!==): an X or Z in got never matches.
//   `TB_DUMP(top)
//       Run with +VCD to write dump.vcd (EDA Playground: tick "Open EPWave").
//       Compile with +define+FSDB to write dump.fsdb for Verdi instead.
// =============================================================================
`ifndef TB_MACROS_SVH
`define TB_MACROS_SVH

`define TB_CHECK(COND, MSG) \
  begin \
    tb_pkg::n_checks++; \
    if ((COND) !== 1'b1) tb_pkg::fail($sformatf MSG); \
  end

`define TB_CHECK_EQ(GOT, EXP, WHAT) \
  begin \
    tb_pkg::n_checks++; \
    if ((GOT) !== (EXP)) \
      tb_pkg::fail($sformatf("%s: got 'h%0h, expected 'h%0h", WHAT, GOT, EXP)); \
  end

`ifdef FSDB
`define TB_DUMP(TOP) \
  initial begin $fsdbDumpfile("dump.fsdb"); $fsdbDumpvars(0, TOP); end
`else
`define TB_DUMP(TOP) \
  initial begin if ($test$plusargs("VCD")) begin $dumpfile("dump.vcd"); $dumpvars(0, TOP); end end
`endif

`endif
