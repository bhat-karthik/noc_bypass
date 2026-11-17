// =============================================================================
// File      tb/common/tg_node.sv                                   Tracker: F24
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Traffic source of one node: a Bernoulli packet generator, an
//           unbounded source queue (D14), packet ids and timestamps, and the
//           flit-by-flit handshake with the NI.
// Decisions D14 network interface, D32 latency accounting, D33 traffic
// Tests     N01-N08 and every mesh run
// -----------------------------------------------------------------------------
// Timing (the models act on the falling edge, the design on the rising edge):
//   negedge of cycle c: if traffic_pkg::gen_on, create a packet with
//     probability rate, or at this period's phase in periodic mode
//     (t_create = c), and queue it; then offer the oldest queued flit to the NI.
//   posedge ending cycle c: the NI takes the flit if it is ready; it is on
//     the link to the router in cycle c + 1.
//   The handshake is sampled at the rising edge, before the design updates,
//   so every simulator sees the same values.
// send() queues one directed packet through the same path; a test asks for
// one with traffic_pkg::direct() (B4), taken at the next falling edge.
// Every packet goes to the scoreboard (sb_pkg) and to the statistics
// (stats_pkg) when it is created.
// @support tb_mesh tb_mesh_dl tb_mesh_b tb_mesh_c tb_mesh_cb tb_mesh_d
// (the EDA bundle compiles this module only for the tests of those benches)
// =============================================================================
`timescale 1ns/1ps

module tg_node
  import noc_pkg::*;
  import traffic_pkg::*;
#(
  parameter int ID     = 0,      // node id
  parameter int MESH_N = 4,
  parameter int SID    = 100     // random stream of this generator
) (
  input  logic        clk,
  input  logic        rst_n,
  input  int unsigned cyc,
  output logic        inj_valid,
  input  logic        inj_ready,
  output flit_t       inj_flit
);

  pkt_t            q [$];              // source queue
  int              idx       = 0;      // next flit of q[0]
  int unsigned     seq       = 0;
  bit              took      = 1'b0;   // the NI took the flit offered last cycle
  int              burst_seen = 0;     // last all-pairs burst served
  int              d, l;               // destination and length of a new packet
  int unsigned     ph        = 0;      // periodic injection: phase in this period
  int              nd        = 0;      // directed packets waiting in traffic_pkg
  int              queued    = 0;      // packets in the source queue (for the bench)
  longint unsigned n_sent    = 0;      // flits the NI took
  longint unsigned n_heads   = 0;      // packets whose head the NI took
  longint unsigned n_blocked = 0;      // cycles an offered flit waited for the NI
  longint unsigned max_queue = 0;      // longest source queue seen, packets
  // all_pairs() order; a module variable, not a local: Verilator inlines the
  // function into the always block and would build and free a local queue
  // on every falling edge (10% of the run time on 8x8, B2)
  int              order [$];
  localparam logic [7:0] KQ = 8'(ID);   // this node's directed queue
  traffic_pkg::dir_t dd;

  initial begin
    inj_valid = 1'b0;
    inj_flit  = '0;
  end

  // Queue a packet created now
  function automatic void send(int dst, int len);
    pkt_t p;
    p.seq      = seq;
    p.t_create = cyc;
    p.dst      = node_id_t'(dst);
    p.len      = PKT_LEN_W'(len);
    seq++;
    q.push_back(p);
    if (longint'(q.size()) > max_queue) max_queue = longint'(q.size());
    sb_pkg::add(ID, p.seq, dst, len, cyc);
    stats_pkg::on_create(len, cyc);
  endfunction

  // One packet to every node, the source included, in a random order
  // (Fisher-Yates from this generator's stream, one draw per statement)
  function automatic void all_pairs();
    int k, j, t;
    order.delete();
    for (int i = 0; i < MESH_N * MESH_N; i++) order.push_back(i);
    for (k = MESH_N * MESH_N - 1; k > 0; k--) begin
      j        = int'(tb_pkg::rng_below(SID, k + 1));
      t        = order[k];
      order[k] = order[j];
      order[j] = t;
    end
    foreach (order[i]) begin
      l = pkt_len(SID);
      send(order[i], l);
    end
  endfunction

  always @(posedge clk) begin
    took <= inj_valid & inj_ready;
    if (inj_valid && !inj_ready) n_blocked <= n_blocked + 1;
  end

  always @(negedge clk) begin
    if (rst_n) begin
      // 1. the flit offered last cycle was taken: move to the next one
      if (took) begin
        if (q.size() == 0) begin
          tb_pkg::fail($sformatf("node %0d: the NI took a flit from an empty queue", ID));
        end else begin
          n_sent++;
          if (idx == 0) n_heads++;
          idx++;
          if (idx >= int'(q[0].len)) begin
            void'(q.pop_front());
            idx = 0;
          end
        end
      end
      // 2. create this cycle's packets: directed ones, an all-pairs burst, a
      //    periodic packet, or a Bernoulli draw (nested ifs: no draw inside a
      //    condition, D30)
      nd = traffic_pkg::dir_q[KQ].size();
      for (int k = 0; k < nd; k++) begin
        dd = traffic_pkg::dir_q[KQ].pop_front();
        send(int'(dd.dst), int'(dd.len));
      end
      if (burst_id != burst_seen) begin
        burst_seen = burst_id;
        all_pairs();
      end
      if (gen_on) begin
        if (period > 0) begin
          if (cyc >= per_t0) begin
            if ((cyc - per_t0) % period == 0) ph = tb_pkg::rng_below(SID, period);
            if ((cyc - per_t0) % period == ph) begin
              d = dest(pattern, ID, MESH_N, cyc, SID);
              l = pkt_len(SID);
              send(d, l);
            end
          end
        end else begin
          if (tb_pkg::rng_chance(SID, rate)) begin
            d = dest(pattern, ID, MESH_N, cyc, SID);
            l = pkt_len(SID);
            send(d, l);
          end
        end
      end
      queued = q.size();
      // 3. offer the oldest flit
      if (q.size() > 0) begin
        inj_valid = 1'b1;
        inj_flit  = make_flit(ID, q[0], idx);
      end else begin
        inj_valid = 1'b0;
        inj_flit  = '0;
      end
    end
  end

endmodule
