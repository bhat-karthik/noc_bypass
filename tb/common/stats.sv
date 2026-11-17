// =============================================================================
// File      tb/common/stats.sv                                     Tracker: F27
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Purpose   Package stats_pkg: the metrics of one run (Section 8) and its CSV
//           row.
// Metrics   M01 lat_avg, M02 lat_p50 / lat_p99 / lat_max, M04 thr_off /
//           thr_acc, M07 byp_pkt / byp_flit (bypass share, B4), M17 stalls /
//           t_stall, M18 jain / lat_flow_max
// Tests     N02-N04, N06-N08, PERF
// -----------------------------------------------------------------------------
// Window: [w_start, w_end) in cycles. A packet is measured when it is created
// inside the window (Section 8); every measured packet must be delivered.
//   latency  tail ejection minus creation, source queueing included (G7);
//            ejection = the cycle the tail reaches its NI (D32)
//   p50, p99 from a 1-cycle histogram of 4,096 bins; the last bin holds
//            4,095 cycles and more, and lat_max is exact
//   thr_off  flits of the packets created in the window, per node per cycle
//   thr_acc  flits that reached an NI during the window, per node per cycle
//   jain     Jain's index over sources of the measured flits delivered
//   lat_flow_max  worst average latency of a (source, destination) flow with
//            at least FLOW_MIN measured packets
//   byp_pkt  measured packets delivered by bypass / measured packets
//            delivered; byp_flit the same by flits (M07). The CSV prints 0
//            when no packet came by bypass (design A), as in B2.
// CSV: the configuration columns of Section 8, then the metric columns, then
// pkts (measured packets), sat (accepted below 95% of offered) and cycles.
// =============================================================================
`timescale 1ns/1ps

package stats_pkg;

  localparam int HIST     = 4096;
  localparam int FLOW_MIN = 10;

  int unsigned     w_start = 0, w_end = 0;
  int              n_nodes = 1;
  longint unsigned off_flits, acc_flits;
  longint unsigned m_pkts, m_flits, m_left;    // measured: created, flits, not yet delivered
  longint unsigned lat_sum, lat_max;
  longint unsigned zl_sum, zl_hits;           // zero-load latency of the packets measured; how many met it
  longint unsigned byp_pkts, byp_flits;       // measured packets (flits) delivered by bypass
  longint unsigned hist [HIST];
  longint unsigned src_flits [256];
  longint unsigned flow_sum [int];
  longint unsigned flow_cnt [int];
  longint unsigned tr_pkts, tr_lat;           // every delivery since trace_take (PERF's +TRACE, B8)

  // ---------------------------------------------------------------- window
  function automatic void reset(int nodes);
    n_nodes   = nodes;
    w_start   = 0;
    w_end     = 0;
    off_flits = 0;
    acc_flits = 0;
    m_pkts    = 0;
    m_flits   = 0;
    m_left    = 0;
    lat_sum   = 0;
    lat_max   = 0;
    zl_sum    = 0;
    zl_hits   = 0;
    byp_pkts  = 0;
    byp_flits = 0;
    for (int i = 0; i < HIST; i++) hist[i] = 0;
    for (int i = 0; i < 256; i++) src_flits[i] = 0;
    flow_sum.delete();
    flow_cnt.delete();
    tr_pkts   = 0;
    tr_lat    = 0;
  endfunction

  // the deliveries and their summed latency since the last call (+TRACE)
  function automatic void trace_take(output longint unsigned n, output longint unsigned s);
    n       = tr_pkts;
    s       = tr_lat;
    tr_pkts = 0;
    tr_lat  = 0;
  endfunction

  function automatic void open_window(int unsigned s, int unsigned e);
    w_start = s;
    w_end   = e;
  endfunction

  function automatic bit in_window(int unsigned t);
    return (t >= w_start) && (t < w_end);
  endfunction

  // ---------------------------------------------------------------- events
  function automatic void on_create(int len, int unsigned t);
    if (in_window(t)) begin
      off_flits += longint'(len);
      m_pkts++;
      m_left++;
    end
  endfunction

  function automatic void on_arrive(int unsigned t);
    if (in_window(t)) acc_flits++;
  endfunction

  // zl: the packet's zero-load latency (D32), from the sink; byp: it came by bypass
  function automatic void on_deliver(int src, int dst, int len, int unsigned t_create, int unsigned t_eject,
                                     int zl, bit byp);
    longint unsigned lat;
    int unsigned     d;
    int              f;
    d        = t_eject - t_create;
    lat      = longint'(d);
    tr_pkts++;
    tr_lat  += lat;
    if (!in_window(t_create)) return;
    lat_sum += lat;
    zl_sum  += longint'(zl);
    if (lat == longint'(zl)) zl_hits++;
    if (lat > lat_max) lat_max = lat;
    if (lat >= longint'(HIST) - 1) hist[HIST-1]++;
    else                           hist[int'(lat)]++;
    m_flits += longint'(len);
    m_left--;
    if (byp) begin
      byp_pkts++;
      byp_flits += longint'(len);
    end
    src_flits[src] += longint'(len);
    f = src * 256 + dst;
    if (flow_cnt.exists(f) != 0) begin
      flow_cnt[f] = flow_cnt[f] + 1;
      flow_sum[f] = flow_sum[f] + lat;
    end else begin
      flow_cnt[f] = 1;
      flow_sum[f] = lat;
    end
  endfunction

  // ---------------------------------------------------------------- metrics
  function automatic longint unsigned delivered_pkts();
    return m_pkts - m_left;
  endfunction

  function automatic real lat_avg();
    if (delivered_pkts() == 0) return 0.0;
    return real'(lat_sum) / real'(delivered_pkts());
  endfunction

  // Average zero-load latency of the measured packets, and the share of
  // them delivered in exactly that time (N02)
  function automatic real zl_avg();
    if (delivered_pkts() == 0) return 0.0;
    return real'(zl_sum) / real'(delivered_pkts());
  endfunction

  function automatic real zl_share();
    if (delivered_pkts() == 0) return 0.0;
    return real'(zl_hits) / real'(delivered_pkts());
  endfunction

  // Smallest latency l with at least q of the measured packets at or below l
  function automatic int lat_pct(real q);
    longint unsigned need, cum;
    if (delivered_pkts() == 0) return 0;
    need = longint'(q * real'(delivered_pkts()) + 0.999999);
    if (need < 1) need = 1;
    cum = 0;
    for (int i = 0; i < HIST; i++) begin
      cum += hist[i];
      if (cum >= need) return i;
    end
    return HIST - 1;
  endfunction

  function automatic real window_len();
    return real'(w_end - w_start);
  endfunction

  function automatic real thr_off();
    if (w_end <= w_start) return 0.0;
    return real'(off_flits) / (real'(n_nodes) * window_len());
  endfunction

  function automatic real thr_acc();
    if (w_end <= w_start) return 0.0;
    return real'(acc_flits) / (real'(n_nodes) * window_len());
  endfunction

  function automatic real jain();
    real s, s2, x;
    s  = 0.0;
    s2 = 0.0;
    for (int i = 0; i < n_nodes; i++) begin
      x   = real'(src_flits[i]);
      s  += x;
      s2 += x * x;
    end
    if (s2 == 0.0) return 0.0;
    return (s * s) / (real'(n_nodes) * s2);
  endfunction

  function automatic real lat_flow_max();
    real worst, a;
    worst = 0.0;
    foreach (flow_cnt[f]) begin
      if (flow_cnt[f] >= longint'(FLOW_MIN)) begin
        if (flow_sum.exists(f) != 0) begin
          a = real'(flow_sum[f]) / real'(flow_cnt[f]);
          if (a > worst) worst = a;
        end
      end
    end
    return worst;
  endfunction

  function automatic bit saturated();
    return thr_acc() < 0.95 * thr_off();
  endfunction

  // Bypass share (M07)
  function automatic real byp_pkt();
    if (delivered_pkts() == 0) return 0.0;
    return real'(byp_pkts) / real'(delivered_pkts());
  endfunction

  function automatic real byp_flit();
    if (m_flits == 0) return 0.0;
    return real'(byp_flits) / real'(m_flits);
  endfunction

  function automatic string byp_cols();
    if (byp_pkts == 0) return "0,0";
    return $sformatf("%0.4f,%0.4f", byp_pkt(), byp_flit());
  endfunction

  // ---------------------------------------------------------------- CSV
  function automatic string csv_header();
    return {"run_id,design,N,V,pattern,p_data,b_max,t_mode,policy,epoch,rate,seed,",
            "lat_avg,lat_p50,lat_p99,lat_max,thr_off,thr_acc,byp_pkt,byp_flit,",
            "jain,lat_flow_max,stalls,t_stall,pkts,sat,cycles"};
  endfunction

  // b_max and t_mode: the burst limit and subslot rule of designs B-D ("-" in
  // design A); policy and epoch: design C's fixed k_L or design D's policy,
  // and the epoch of designs C and D ("-" otherwise, B8)
  function automatic string csv_row(string run_id, string dsg, int n, int v, string pat,
                                    real p_data, real rate, longint unsigned seed,
                                    int stalls, int unsigned t_stall, int unsigned cycles,
                                    string b_max_s = "-", string t_mode_s = "-",
                                    string policy_s = "-", string epoch_s = "-");
    return $sformatf("%s,%s,%0d,%0d,%s,%0.4f,%s,%s,%s,%s,%0.4f,%0d,%0.3f,%0d,%0d,%0d,%0.4f,%0.4f,%s,%0.4f,%0.3f,%0d,%0d,%0d,%0d,%0d",
                     run_id, dsg, n, v, pat, p_data, b_max_s, t_mode_s, policy_s, epoch_s, rate, seed,
                     lat_avg(), lat_pct(0.50), lat_pct(0.99), lat_max, thr_off(), thr_acc(), byp_cols(),
                     jain(), lat_flow_max(), stalls, t_stall, delivered_pkts(),
                     saturated(), cycles);
  endfunction

endpackage
