// =============================================================================
// File      tb/common/perf_probe.sv                                Tracker: F49
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   The RTL's own bypass and reservation counters, summed over the
//           mesh, as extra CSV columns of a design B-D performance point:
//           turns, bursts, cancels, reservations, bypassed flits and packets.
//           Designs C and D add k_L, the phase time and the turn intervals
//           (header_two, B8).
// Decisions D41 checker and helper classes in packages, D48 reservation
// Tests     PERF and BP10 on design B (E0-E10 rows)
// -----------------------------------------------------------------------------
// mesh_bench takes a snapshot (perf_snap) before and after the measurement
// and row() prints the differences. The counters are simulation-only hooks
// in bypass_source and resv_ctrl, the same ones the bypass tests read.
// Columns:
//   turns        subslots in which a router could send
//   bursts       ... and sent a burst
//   src_cancels  turns skipped because the destination cancelled
//   lost         ... while packets for that partner waited
//   byp_flits    flits sent by bypass (sources)
//   byp_pkts     packets sent by bypass (sources)
//   decisions    reservation decisions at the destinations
//   accepted     ... that reserved space
//   dst_cancels  ... that cancelled (sent a token)
//   noshows      reservations released because no burst came
//   two_waiting  reservations made while another one waited for its burst
// =============================================================================
`timescale 1ns/1ps

package perf_pkg;

  typedef struct {
    longint unsigned turns, used, cxl, lost, bflits, bpkts, dec, acc, rcx, nosh, two;
  } snap_t;

  function automatic string header();
    return "turns,bursts,src_cancels,lost,byp_flits,byp_pkts,decisions,accepted,dst_cancels,noshows,two_waiting";
  endfunction

  // designs C and D (B8): k_L, the phase time and the turn intervals of the
  // point, computed by PERF (tests_perf.svh says what each one is)
  function automatic string header_two();
    return "k_mean,k_min,k_max,k_changes,share_L,turn_max_L,turn_max_G,kL_trace";
  endfunction

  // the same columns as "-", for the designs that lack them, so that every
  // PERF row of every design has the same columns (D58)
  function automatic string row_na();
    return "-,-,-,-,-,-,-,-,-,-,-";
  endfunction
  function automatic string row_two_na();
    return "-,-,-,-,-,-,-,-";
  endfunction

  function automatic string row(snap_t a, snap_t b);
    return $sformatf("%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d",
                     b.turns - a.turns, b.used - a.used, b.cxl - a.cxl, b.lost - a.lost,
                     b.bflits - a.bflits, b.bpkts - a.bpkts, b.dec - a.dec, b.acc - a.acc,
                     b.rcx - a.rcx, b.nosh - a.nosh, b.two - a.two);
  endfunction

endpackage
