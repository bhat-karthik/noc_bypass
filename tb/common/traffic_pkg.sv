// =============================================================================
// File      tb/common/traffic_pkg.sv                               Tracker: F23
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Synthetic traffic: destination patterns, the packet mix, the
//           settings every traffic generator reads, and the payload each flit
//           carries, so a sink can check a packet without storing it.
// Decisions D33 traffic definitions; G5 packet mix and units
// Tests     N01-N08 and every experiment
// -----------------------------------------------------------------------------
// Patterns on an n x n mesh, node id = y*n + x, b = log2(n*n) id bits:
//   UNIFORM    any node, the source included (as Garnet's uniform_random)
//   TRANSPOSE  (x, y) -> (y, x)
//   BITCOMP    every id bit inverted
//   BITROT     id rotated right by one bit
//   SHUFFLE    id rotated left by one bit
//   TORNADO    (x + ceil(n/2) - 1, y + ceil(n/2) - 1), modulo n
//   NEIGHBOR   (x + 1, y + 1), modulo n (on 4x4 the same as TORNADO: ceil(4/2) - 1 = 1)
//   HOTSPOT    the hotspot node with probability hot_frac, else UNIFORM
//   REGION     with probability p_local a node of the source's own region
//              (region_n x region_n nodes), else a node outside it
//   PHASE      phase_a and phase_b alternating every phase_len cycles
// A packet a node sends to itself crosses its own router as a 0-hop packet (G5).
// Rates are packets per node per cycle (Bernoulli: one draw per node and
// cycle), or one packet per node per period (periodic, N02). A packet is a
// 5-flit data packet with probability p_data, else a 1-flit control packet.
// Payload: the head carries head_t (noc_pkg) with a signature over its
// fields; every other flit carries 128 bits derived from (source, sequence
// number, flit index). The sink recomputes both.
// Every random choice draws from the caller's stream, one draw per statement
// (D24), so all simulators make the same choices.
// Directed packets (B4): direct(src, dst, len) queues a packet at generator
// src, which takes it at its next falling edge. Delivery log: while log_on is
// set, every sink appends each packet it delivers to dlog (source, sequence,
// creation and tail ejection cycles, bypass or not), for directed checks.
// =============================================================================
`timescale 1ns/1ps

package traffic_pkg;
  import noc_pkg::*;

  typedef enum int {
    PAT_UNIFORM   = 0,
    PAT_TRANSPOSE = 1,
    PAT_BITCOMP   = 2,
    PAT_BITROT    = 3,
    PAT_SHUFFLE   = 4,
    PAT_TORNADO   = 5,
    PAT_NEIGHBOR  = 6,
    PAT_HOTSPOT   = 7,
    PAT_REGION    = 8,
    PAT_PHASE     = 9
  } pattern_e;

  localparam int NUM_PATTERNS = 10;

  // A packet waiting in a source queue
  typedef struct packed {
    logic [31:0]          seq;        // per-source sequence number
    logic [31:0]          t_create;   // cycle it was created
    node_id_t             dst;
    logic [PKT_LEN_W-1:0] len;        // 1 or 5 flits
  } pkt_t;

  // ---------------------------------------------------------------- settings
  // The bench sets these before each run; generators read them every cycle.
  // Every testbench compiles this package, and one without a mesh (U01-R04)
  // reads none of them: the same waiver as noc_pkg's constants.
  /* verilator lint_off UNUSEDSIGNAL */
  /* verilator lint_off UNUSEDPARAM */
  int       mesh_n    = 4;
  pattern_e pattern   = PAT_UNIFORM;
  real      rate      = 0.0;            // packets per node per cycle
  real      p_data    = 1.0 / 3.0;      // share of 5-flit packets (P_DATA)
  int       hot_node  = -1;             // -1: the centre node (n/2, n/2)
  real      hot_frac  = 0.2;
  int       region_n  = 0;              // 0: n/2 (4 on 8x8, as REGION_N)
  real      p_local   = 0.5;
  pattern_e phase_a   = PAT_UNIFORM;
  pattern_e phase_b   = PAT_TRANSPOSE;
  int       phase_len = 500;
  bit       gen_on    = 1'b0;           // generators create packets only while set
  // Directed traffic: every generator queues one packet to every node (all
  // pairs, N01), in its own random order, when burst_id changes
  int       burst_id  = 0;
  // Periodic injection (N02): with period > 0, every generator creates exactly
  // one packet in each period of `period` cycles from cycle per_t0, at a phase
  // drawn at the start of the period, instead of Bernoulli draws at `rate`.
  // Every source then sends the same number of packets, so the average hop
  // count of the packets sent equals the pattern's mean exactly.
  int          period    = 0;
  int unsigned per_t0    = 0;
  // Sinks: chance per cycle of not taking a flit, and one forced stall window
  // [fs_start, fs_end) at node fs_node (FS_NONE: none, FS_ALL: every node)
  localparam int FS_NONE = -1;
  localparam int FS_ALL  = -2;
  real      sink_stall_p = 0.0;
  int       fs_node      = FS_NONE;
  int unsigned fs_start  = 0;
  int unsigned fs_end    = 0;

  // Directed packets per source (an array of queues: 2^8 entries, 8-bit
  // indices, D30)
  typedef struct packed {
    logic [7:0]           dst;
    logic [PKT_LEN_W-1:0] len;
  } dir_t;
  dir_t     dir_q [256][$];

  // Delivery log
  typedef struct packed {
    logic [7:0]  src;
    logic [7:0]  dst;
    logic [31:0] seq;
    logic [2:0]  len;
    logic [31:0] t_create;
    logic [31:0] t_tail;
    logic        byp;
  } dlog_t;
  bit       log_on = 1'b0;
  dlog_t    dlog [$];

  // node ids fit in 8 bits, lengths in 3
  function automatic void direct(int src, int dst, int len);
    dir_t       d;
    logic [7:0] k;
    d.dst = 8'(dst);
    d.len = PKT_LEN_W'(len);
    k     = 8'(src);
    dir_q[k].push_back(d);
  endfunction
  /* verilator lint_on UNUSEDPARAM */
  /* verilator lint_on UNUSEDSIGNAL */

  // ---------------------------------------------------------------- names
  function automatic string pat_name(pattern_e p);
    case (p)
      PAT_UNIFORM:   return "uniform";
      PAT_TRANSPOSE: return "transpose";
      PAT_BITCOMP:   return "bitcomp";
      PAT_BITROT:    return "bitrot";
      PAT_SHUFFLE:   return "shuffle";
      PAT_TORNADO:   return "tornado";
      PAT_NEIGHBOR:  return "neighbor";
      PAT_HOTSPOT:   return "hotspot";
      PAT_REGION:    return "region";
      default:       return "phase";
    endcase
  endfunction

  // 1 and the pattern when s names one (as pat_name spells it)
  function automatic bit pat_parse(string s, output pattern_e p);
    p = PAT_UNIFORM;
    for (int i = 0; i < NUM_PATTERNS; i++) begin
      if (s == pat_name(pattern_e'(i))) begin
        p = pattern_e'(i);
        return 1'b1;
      end
    end
    return 1'b0;
  endfunction

  // ---------------------------------------------------------------- patterns
  function automatic int id_bits(int n);
    return $clog2(n * n);
  endfunction

  function automatic int region_side(int n);
    if (region_n > 0) return region_n;
    return (n >= 2) ? n / 2 : 1;
  endfunction

  function automatic int hotspot(int n);
    if (hot_node >= 0) return hot_node;
    return (n / 2) * n + n / 2;
  endfunction

  // Destination of a fixed pattern (a permutation); -1 for a random pattern
  function automatic int perm_dest(pattern_e p, int src, int n);
    int b, x, y, k, m;
    b = id_bits(n);
    m = (1 << b) - 1;
    x = src % n;
    y = src / n;
    k = (n + 1) / 2 - 1;                 // ceil(n/2) - 1
    case (p)
      PAT_TRANSPOSE: return x * n + y;
      PAT_BITCOMP:   return (~src) & m;
      PAT_BITROT:    return ((src >> 1) | ((src & 1) << (b - 1))) & m;
      PAT_SHUFFLE:   return ((src << 1) | (src >> (b - 1))) & m;
      PAT_TORNADO:   return ((y + k) % n) * n + (x + k) % n;
      PAT_NEIGHBOR:  return ((y + 1) % n) * n + (x + 1) % n;
      default:       return -1;
    endcase
  endfunction

  // Destination under a pattern other than PHASE, drawing from stream sid
  function automatic int dest_one(pattern_e p, int src, int n, int sid);
    int r, x0, y0, k, d;
    bit in_reg;
    case (p)
      PAT_UNIFORM: begin
        d = int'(tb_pkg::rng_below(sid, n * n));
        return d;
      end
      PAT_HOTSPOT: begin
        if (tb_pkg::rng_chance(sid, hot_frac)) return hotspot(n);
        d = int'(tb_pkg::rng_below(sid, n * n));
        return d;
      end
      PAT_REGION: begin
        r  = region_side(n);
        x0 = ((src % n) / r) * r;
        y0 = ((src / n) / r) * r;
        // nested ifs, not ||: a draw must not depend on how a simulator
        // evaluates a condition (D30)
        in_reg = 1'b1;
        if (r < n) in_reg = tb_pkg::rng_chance(sid, p_local);
        if (in_reg) begin
          k = int'(tb_pkg::rng_below(sid, r * r));
          return (y0 + k / r) * n + x0 + k % r;
        end
        // the k-th node outside the region, in id order
        k = int'(tb_pkg::rng_below(sid, n * n - r * r));
        for (int i = 0; i < n * n; i++) begin
          if ((i % n) >= x0 && (i % n) < x0 + r && (i / n) >= y0 && (i / n) < y0 + r) continue;
          if (k == 0) return i;
          k--;
        end
        return src;
      end
      default: return perm_dest(p, src, n);
    endcase
  endfunction

  // Destination for a packet created in cycle cyc. PHASE picks phase_a or
  // phase_b by the cycle; a phase that is itself PHASE counts as UNIFORM.
  // (No recursion: Verilator 5.020 cannot clone a recursive package function.)
  function automatic int dest(pattern_e p, int src, int n, int unsigned cyc, int sid);
    pattern_e q;
    q = p;
    if (p == PAT_PHASE) begin
      q = ((cyc / int'(phase_len)) % 2 == 0) ? phase_a : phase_b;
      if (q == PAT_PHASE) q = PAT_UNIFORM;
    end
    return dest_one(q, src, n, sid);
  endfunction

  // Packet length: 5 flits with probability p_data, else 1
  function automatic int pkt_len(int sid);
    if (tb_pkg::rng_chance(sid, p_data)) return MAX_PKT_FLITS;
    return 1;
  endfunction

  // ---------------------------------------------------------------- payload
  function automatic logic [31:0] head_check(int src, int unsigned seq, int dst, int len,
                                             int unsigned t_create);
    logic [63:0] z;
    z = tb_pkg::mix64({32'(src), 32'(seq)} ^ {32'(dst), 32'(t_create)} ^ {32'(len), 32'h5bd1_e995});
    return z[63:32] ^ z[31:0];
  endfunction

  function automatic logic [FLIT_DATA_W-1:0] body_data(int src, int unsigned seq, int idx);
    logic [63:0] a, b;
    a = tb_pkg::mix64({32'(src), 32'(seq)} ^ {32'(idx), 32'h0});
    b = tb_pkg::mix64(a ^ 64'hc2b2_ae3d_27d4_eb4f);
    return {a, b};
  endfunction

  // Flit idx of a packet from src (ftype and data; the NI sets vc)
  function automatic flit_t make_flit(int src, pkt_t p, int idx);
    flit_t f;
    head_t h;
    f       = '0;
    f.vld   = 1'b1;
    if (p.len == 3'd1)                    f.ftype = FT_SINGLE;
    else if (idx == 0)                    f.ftype = FT_HEAD;
    else if (idx == int'(p.len) - 1)      f.ftype = FT_TAIL;
    else                                  f.ftype = FT_BODY;
    if (idx == 0) begin
      h          = '0;
      h.dst      = p.dst;
      h.src      = node_id_t'(src);
      h.cls      = (p.len == 3'd1) ? CLS_CTRL : CLS_DATA;
      h.len      = p.len;
      h.seq      = p.seq;
      h.t_create = p.t_create;
      h.check    = head_check(src, p.seq, int'(p.dst), int'(p.len), p.t_create);
      f.data     = h;
    end else begin
      f.data = body_data(src, p.seq, idx);
    end
    return f;
  endfunction

  // ---------------------------------------------------------------- expectations
  // Zero-load latency of a len-flit packet over h hops (D32): one cycle in the
  // NI, 4 cycles per router (3 stages and the link), then len - 1 more flits
  function automatic int zero_load(int h, int len);
    return 4 * (h + 1) + 1 + (len - 1);
  endfunction

  // Least latency of a packet that ends its trip by bypass (D45): its len
  // flits are all in a VC len + 1 cycles after creation at the earliest; the
  // burst starts no earlier (D38: t), flit j is in the NI at t + 2h + 3 + j,
  // and the tail is flit len - 1 or later. Regular hops before the bypass
  // only add to it (4 cycles each instead of 2).
  function automatic int zero_load_byp(int h, int len);
    return 2 * h + 2 * len + 3;
  endfunction

  // Mean hop count of a fixed pattern over every source (UNIFORM: every pair)
  function automatic real mean_hops(pattern_e p, int n);
    longint s;
    int     d, cnt;
    s   = 0;
    cnt = 0;
    for (int a = 0; a < n * n; a++) begin
      if (p == PAT_UNIFORM) begin
        for (int b = 0; b < n * n; b++) begin
          s += longint'(hops(coord_t'(a % n), coord_t'(a / n), coord_t'(b % n), coord_t'(b / n)));
          cnt++;
        end
      end else begin
        d = perm_dest(p, a, n);
        s += longint'(hops(coord_t'(a % n), coord_t'(a / n), coord_t'(d % n), coord_t'(d / n)));
        cnt++;
      end
    end
    return real'(s) / real'(cnt);
  endfunction

endpackage
