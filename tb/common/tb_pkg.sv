// =============================================================================
// File      tb/common/tb_pkg.sv                                    Tracker: F05
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Services shared by every testbench: check and error counters,
//           the PASS/FAIL result line, run banner and plusargs, coverage bins,
//           CSV output, a watchdog, and a seeded random generator that gives
//           the same numbers on every simulator.
// Used by   every testbench; the check macros live in tb_macros.svh
// -----------------------------------------------------------------------------
// Plusargs (read by start)
//   +SEED=<n>     seed of every random stream (default 1)
//   +MAXERR=<n>   errors printed in full (default 20); all of them are counted
//   +CSV=<file>   also write the CSV rows to <file>; they always go to the log
//   +VCD          dump every signal to dump.vcd (EDA Playground: EPWave)
//   +TEST=<id>    in a testbench that holds several tests (tb_router: R01-R04),
//                 run only that one; without it the testbench runs them all.
//                 The EDA Playground bundle sets the same choice with the
//                 compile define TEST_<id> (macro TB_TEST); +TEST wins.
// One result line per run, for the run log and for scripts:
//   RESULT <test> PASS checks=<n> errors=0 seed=<s>
//   RESULT <test> FAIL checks=<n> errors=<e> seed=<s>
// A run with zero checks fails: a test that checks nothing proves nothing.
// -----------------------------------------------------------------------------
// Why our own random generator
//   $urandom returns different numbers on Riviera, xsim, VCS and Verilator for
//   the same seed, and its sequence depends on which process draws first. Here
//   every stream (one per tester now, one per traffic source later) keeps its
//   own 64-bit state, derived from the seed and a stream id, and steps it with
//   splitmix64. The same seed then gives the same stimulus on every simulator
//   and in any process order, so a failure seen in Vivado replays in VCS/Verdi.
// =============================================================================
`timescale 1ns/1ps

package tb_pkg;

  // ---------------------------------------------------------------------------
  // Run state
  // ---------------------------------------------------------------------------
  longint unsigned n_checks  = 0;
  longint unsigned n_errors  = 0;
  longint unsigned max_print = 20;
  longint unsigned seed      = 1;
  string           test_id   = "";
  string           sel_test  = "";   // +TEST=<id>, empty = every test
  int              csv_fd    = 0;

  localparam string LINE = "==========================================================================";

  // ---------------------------------------------------------------------------
  // Messages
  // ---------------------------------------------------------------------------
  // Count an error; print the first max_print of them in full
  function automatic void fail(string msg);
    n_errors++;
    if (n_errors <= max_print)
      $display("ERROR  [%0t] %s: %s", $time, test_id, msg);
    if (n_errors == max_print)
      $display("ERROR  [%0t] %s: later errors are counted, not printed", $time, test_id);
  endfunction

  function automatic void info(string msg);
    $display("INFO   %s", msg);
  endfunction

  function automatic void phase(string name);
    $display("PHASE  %s", name);
  endfunction

  // s padded with spaces to at least w characters (for aligned columns)
  function automatic string pad(string s, int w);
    string r;
    r = s;
    while (r.len() < w) r = {r, " "};
    return r;
  endfunction

  // A coverage bin: one check that the event happened at least once
  function automatic void cover_bin(string what, longint unsigned hits);
    n_checks++;
    $display("COVER  %s %0d", pad(what, 52), hits);
    if (hits == 0) fail({"coverage hole: ", what});
  endfunction

  // A coverage goal of a random test (B4): printed like a bin, but not a
  // check: a run that misses it does not fail, since a directed test makes
  // the event happen on purpose
  function automatic void cover_goal(string what, longint unsigned hits);
    $display("COVER  %s %0d", pad(what, 52), hits);
    if (hits == 0) info({"coverage goal not reached in this run: ", what});
  endfunction

  // ---------------------------------------------------------------------------
  // Start and finish
  // ---------------------------------------------------------------------------
  // 1 when test `id` should run: no +TEST was given, or +TEST names it
  function automatic bit test_on(string id);
    return (sel_test == "") || (sel_test == id);
  endfunction

  // id names the test, or the group of tests the testbench holds ("R01-R04");
  // with +TEST=<id> the result line names that one test
  task automatic start(string id, string title);
    string path;
    bit    got_seed, got_maxerr, got_test;
    got_test = $value$plusargs("TEST=%s", sel_test);
`ifdef TB_TEST
    if (!got_test) begin
      sel_test = `TB_TEST;
      got_test = 1'b1;
    end
`endif
    test_id  = got_test ? sel_test : id;
    // Keep the results: xsim warns when $value$plusargs is called as a task
    got_seed   = $value$plusargs("SEED=%d", seed);
    got_maxerr = $value$plusargs("MAXERR=%d", max_print);
    if ($value$plusargs("CSV=%s", path)) begin
      csv_fd = $fopen(path, "w");
      if (csv_fd == 0) fail({"cannot open CSV file ", path});
    end
    $display("%s", LINE);
    $display("TEST   %s  %s", id, title);
    if (got_seed) $display("SEED   %0d (from +SEED)", seed);
    else          $display("SEED   %0d (default)", seed);
    if (got_maxerr) $display("MAXERR %0d errors printed in full", max_print);
    $display("%s", LINE);
  endtask

  task automatic finish();
    string verdict;
    verdict = (n_errors == 0 && n_checks != 0) ? "PASS" : "FAIL";
    $display("%s", LINE);
    $display("RESULT %s %s checks=%0d errors=%0d seed=%0d", test_id, verdict, n_checks, n_errors, seed);
    $display("%s", LINE);
    if (csv_fd != 0) $fclose(csv_fd);
    if (verdict == "FAIL") $fatal(1, "%s failed", test_id);
    $finish;
  endtask

  // Ends the run as a failure if it is still going after max_time
  task automatic watchdog(time max_time);
    #(max_time);
    fail($sformatf("watchdog: still running after %0t", max_time));
    finish();
  endtask

  // ---------------------------------------------------------------------------
  // Random streams (splitmix64)
  // A stream is a number 0..RNG_STREAMS-1 chosen by the testbench (a tester's
  // size, a router id, ...), never by the order processes start, so the
  // numbers do not depend on the simulator. rng_init(id) once, after start().
  // Rule: one draw per statement. The order in which a simulator evaluates
  // the arguments of a call, or the operands of an expression, is not defined,
  // so f(rng_next(s), rng_next(s)) can differ between simulators.
  // ---------------------------------------------------------------------------
  typedef logic [63:0] rng_t;

  localparam int   RNG_W       = 12;
  localparam int   RNG_STREAMS = 1 << RNG_W;
  localparam rng_t GOLDEN      = 64'h9E37_79B9_7F4A_7C15;

  rng_t rng_st   [RNG_STREAMS];
  bit   rng_used [RNG_STREAMS];

  function automatic rng_t mix64(rng_t z);
    z = (z ^ (z >> 30)) * 64'hBF58_476D_1CE4_E5B9;
    z = (z ^ (z >> 27)) * 64'h94D0_49BB_1331_11EB;
    return z ^ (z >> 31);
  endfunction

  function automatic void rng_init(int id);
    if (id < 0 || id >= RNG_STREAMS) begin
      fail($sformatf("random stream %0d out of range 0..%0d", id, RNG_STREAMS - 1));
      return;
    end
    if (rng_used[id]) fail($sformatf("random stream %0d initialised twice", id));
    rng_used[id] = 1'b1;
    rng_st[id]   = mix64(rng_t'(seed) ^ mix64(rng_t'(id) + 64'd1));
  endfunction

  // Next 64 random bits of stream id
  function automatic rng_t rng_next(int id);
    logic [RNG_W-1:0] k;
    if (id < 0 || id >= RNG_STREAMS) begin
      fail($sformatf("random stream %0d out of range", id));
      return '0;
    end
    k         = RNG_W'(id);
    rng_st[k] = rng_st[k] + GOLDEN;
    return mix64(rng_st[k]);
  endfunction

  // Uniform in 0..n-1 (n of 0 gives 0)
  function automatic int unsigned rng_below(int id, int unsigned n);
    rng_t r;
    r = rng_next(id);
    return (n == 0) ? 0 : 32'(r % rng_t'(n));
  endfunction

  // 1 with probability p, 0.0 <= p <= 1.0 (resolution 2^-32)
  function automatic bit rng_chance(int id, real p);
    rng_t        r;
    logic [31:0] u;
    r = rng_next(id);
    u = r[63:32] ^ r[31:0];
    return longint'(u) < longint'(p * 4294967296.0);
  endfunction

  // ---------------------------------------------------------------------------
  // CSV rows: always to the log with the prefix "CSV,", and to +CSV=<file>
  // ---------------------------------------------------------------------------
  function automatic void csv(string row);
    $display("CSV,%s", row);
    if (csv_fd != 0) $fdisplay(csv_fd, "%s", row);
  endfunction

  // ---------------------------------------------------------------------------
  // Names for messages
  // ---------------------------------------------------------------------------
  function automatic string port_name(logic [2:0] p);
    string s;
    case (p)
      noc_pkg::P_N: s = "N";
      noc_pkg::P_E: s = "E";
      noc_pkg::P_S: s = "S";
      noc_pkg::P_W: s = "W";
      noc_pkg::P_L: s = "L";
      default:      s = "?";
    endcase
    return s;
  endfunction

endpackage
