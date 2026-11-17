// =============================================================================
// File      tb/common/scoreboard.sv                                Tracker: F26
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Package sb_pkg: every packet created and not yet delivered, so
//           each one is delivered exactly once, intact, at the right node.
// Tests     N01-N08 and every mesh run
// -----------------------------------------------------------------------------
// A generator calls add() when it creates a packet. A sink calls take() when
// the packet's head reaches it: take() returns the record and removes it, so
// a second delivery of the same packet finds nothing and fails, and a packet
// never created is caught the same way. delivered() counts the packet once
// its tail is in. Records live only while a packet is in flight, so long
// runs keep only the packets in the network and the source queues.
// Key: {source, sequence number}. Associative arrays are read only after an
// exists() check in an enclosing if (D30).
// =============================================================================
`timescale 1ns/1ps

package sb_pkg;
  import noc_pkg::*;

  typedef struct packed {
    node_id_t             dst;
    logic [PKT_LEN_W-1:0] len;
    logic [31:0]          t_create;
  } rec_t;

  rec_t            live [longint unsigned];   // packets in flight
  longint unsigned n_created     = 0;
  longint unsigned n_delivered   = 0;
  // read by run_ctrl only, so unused in a testbench without a mesh (U01-R04)
  /* verilator lint_off UNUSEDSIGNAL */
  int unsigned     last_delivery = 0;         // cycle of the latest delivery
  /* verilator lint_on UNUSEDSIGNAL */

  function automatic longint unsigned key(int src, int unsigned seq);
    return (longint'(src) << 32) | longint'(seq);
  endfunction

  function automatic void add(int src, int unsigned seq, int dst, int len, int unsigned t_create);
    rec_t r;
    longint unsigned k;
    k = key(src, seq);
    if (dst < 0 || dst > 255 || !(len == 1 || len == MAX_PKT_FLITS)) begin
      tb_pkg::fail($sformatf("scoreboard: packet %0d.%0d has destination %0d and length %0d", src, seq, dst, len));
      return;
    end
    if (live.exists(k) != 0) begin
      tb_pkg::fail($sformatf("scoreboard: packet %0d.%0d created twice", src, seq));
      return;
    end
    r.dst      = node_id_t'(dst);
    r.len      = PKT_LEN_W'(len);
    r.t_create = t_create;
    live[k]    = r;
    n_created++;
  endfunction

  // 1 and the record when the packet is in flight; the record is removed
  function automatic bit take(int src, int unsigned seq, output rec_t r);
    longint unsigned k;
    k = key(src, seq);
    r = '0;
    if (live.exists(k) == 0) return 1'b0;
    r = live[k];
    live.delete(k);
    return 1'b1;
  endfunction

  function automatic void delivered(int unsigned cyc);
    n_delivered++;
    last_delivery = cyc;
  endfunction

  function automatic longint unsigned outstanding();
    return n_created - n_delivered;
  endfunction

endpackage
