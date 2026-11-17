// =============================================================================
// File      tb/unit/tb_route_compute.sv                            Tracker: F08
// Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
// Test      U03: route compute returns minimal ports; adaptive picks more free
//           VCs, ties to X; XY and YX exact. Also checks the noc_pkg helpers.
// DUT       rtl/router/route_compute.sv (F04), helpers of rtl/pkg/noc_pkg.sv
// @tests    U03
// -----------------------------------------------------------------------------
// Meshes 2x2, 4x4, 8x8 and 16x16, each with three DUTs: RT_XY, RT_DOR and
// RT_ADAPTIVE. For every current router and every destination:
//   * XY and YX equal the reference, and RT_XY / RT_ADAPTIVE ignore mode_i;
//   * adaptive equals the reference with random free-VC counts; up to 8x8,
//     every combination 0..4 x 0..4 of free VCs on the two productive ports;
//   * every port chosen is minimal: one hop closer, or Local on arrival;
//   * noc_pkg::dor_port and hops agree with the reference.
// Up to 8x8, a packet is also walked hop by hop from every source to every
// destination with each algorithm: it must arrive in exactly the Manhattan
// distance, and XY (YX) must never turn from Y back to X (X back to Y), the
// rule that makes dimension order deadlock-free.
// The package checks cover id <-> (x, y), neighbours and opposite ports,
// flit-type helpers, and the bit positions of the head fields.
// =============================================================================
`timescale 1ns/1ps
`include "tb_macros.svh"

module rc_tester
  import noc_pkg::*;
#(
  parameter int N         = 4,
  parameter bit ADAPT_ALL = 1'b1,   // all free-VC combinations on the productive ports
  parameter bit WALKS     = 1'b1    // hop-by-hop walks for every pair
) (
  input  logic go,
  output logic done
);
  coord_t                          cx   = '0;
  coord_t                          cy   = '0;
  node_id_t                        dst  = '0;
  logic                            mode = MODE_XY;
  logic [NPORTS-1:0][VC_CNT_W-1:0] free = '0;
  port_t                           p_xy, p_dor, p_ad;

  route_compute #(.MESH_N(N), .ALGO(RT_XY)) u_xy (
    .cur_x_i(cx), .cur_y_i(cy), .dst_i(dst), .mode_i(mode), .free_vc_i(free), .port_o(p_xy));
  route_compute #(.MESH_N(N), .ALGO(RT_DOR)) u_dor (
    .cur_x_i(cx), .cur_y_i(cy), .dst_i(dst), .mode_i(mode), .free_vc_i(free), .port_o(p_dor));
  route_compute #(.MESH_N(N), .ALGO(RT_ADAPTIVE)) u_ad (
    .cur_x_i(cx), .cur_y_i(cy), .dst_i(dst), .mode_i(mode), .free_vc_i(free), .port_o(p_ad));

  int sid;   // random stream

  longint unsigned c_port [3][NPORTS];   // [XY, YX, adaptive][port] choices seen
  longint unsigned c_y_more = 0, c_tie = 0, c_x_more = 0, c_one_dim = 0;

  // ---------------------------------------------------------------------------
  // Reference: the rules written with signed distances
  // ---------------------------------------------------------------------------
  function automatic int iabs(int v);
    return (v < 0) ? -v : v;
  endfunction

  function automatic port_t ref_dor(int x, int y, int tx, int ty, bit yx);
    int    dx, dy;
    port_t r;
    dx = tx - x;
    dy = ty - y;
    if (dx == 0 && dy == 0)  r = P_L;
    else if (!yx && dx != 0) r = (dx > 0) ? P_E : P_W;
    else if (!yx)            r = (dy > 0) ? P_S : P_N;
    else if (dy != 0)        r = (dy > 0) ? P_S : P_N;
    else                     r = (dx > 0) ? P_E : P_W;
    return r;
  endfunction

  function automatic port_t ref_adaptive(int x, int y, int tx, int ty,
                                         logic [NPORTS-1:0][VC_CNT_W-1:0] f);
    int    dx, dy;
    port_t px, py, r;
    dx = tx - x;
    dy = ty - y;
    px = (dx > 0) ? P_E : P_W;
    py = (dy > 0) ? P_S : P_N;
    if (dx == 0 && dy == 0) r = P_L;
    else if (dx == 0)       r = py;
    else if (dy == 0)       r = px;
    else                    r = (f[py] > f[px]) ? py : px;   // strictly more free VCs, else X
    return r;
  endfunction

  // The property every routing decision must have, whatever the algorithm
  function automatic bit step_is_minimal(int x, int y, int tx, int ty, port_t p);
    int nx, ny;
    bit ok;
    nx = x;
    ny = y;
    ok = 1'b1;
    if (x == tx && y == ty) return (p == P_L);
    case (p)
      P_N:     ny = y - 1;
      P_E:     nx = x + 1;
      P_S:     ny = y + 1;
      P_W:     nx = x - 1;
      default: ok = 1'b0;       // Local before arrival, or not a port
    endcase
    return ok && (iabs(tx - nx) + iabs(ty - ny) == iabs(tx - x) + iabs(ty - y) - 1);
  endfunction

  function automatic void randomize_free();
    for (int p = 0; p < NPORTS; p++) free[p] = VC_CNT_W'(tb_pkg::rng_below(sid, MAX_VC + 1));
  endfunction

  // ---------------------------------------------------------------------------
  // Every (current, destination) pair
  // ---------------------------------------------------------------------------
  task automatic probe_pair(int x, int y, int t);
    int    tx, ty;
    port_t px, py, exp;
    tx   = t % N;
    ty   = t / N;
    cx   = coord_t'(x);
    cy   = coord_t'(y);
    dst  = node_id_t'(t);
    randomize_free();

    mode = MODE_XY;
    #1;
    `TB_CHECK(p_xy === ref_dor(x, y, tx, ty, 1'b0),
      ("%0dx%0d XY at (%0d,%0d) to (%0d,%0d): got %s", N, N, x, y, tx, ty, tb_pkg::port_name(p_xy)))
    `TB_CHECK(p_dor === ref_dor(x, y, tx, ty, 1'b0),
      ("%0dx%0d DOR/XY at (%0d,%0d) to (%0d,%0d): got %s", N, N, x, y, tx, ty, tb_pkg::port_name(p_dor)))
    `TB_CHECK(p_ad === ref_adaptive(x, y, tx, ty, free),
      ("%0dx%0d adaptive at (%0d,%0d) to (%0d,%0d), free N%0d E%0d S%0d W%0d: got %s", N, N, x, y, tx, ty,
       free[P_N], free[P_E], free[P_S], free[P_W], tb_pkg::port_name(p_ad)))
    `TB_CHECK(step_is_minimal(x, y, tx, ty, p_xy) && step_is_minimal(x, y, tx, ty, p_ad),
      ("%0dx%0d at (%0d,%0d) to (%0d,%0d): a choice is not minimal (XY %s, adaptive %s)", N, N, x, y, tx, ty,
       tb_pkg::port_name(p_xy), tb_pkg::port_name(p_ad)))
    `TB_CHECK(dor_port(cx, cy, coord_t'(tx), coord_t'(ty), MODE_XY) === ref_dor(x, y, tx, ty, 1'b0),
      ("noc_pkg::dor_port XY wrong at (%0d,%0d) to (%0d,%0d)", x, y, tx, ty))
    `TB_CHECK(hops(cx, cy, coord_t'(tx), coord_t'(ty)) == iabs(tx - x) + iabs(ty - y),
      ("noc_pkg::hops wrong for (%0d,%0d) to (%0d,%0d)", x, y, tx, ty))
    c_port[0][p_xy]++;
    c_port[2][p_ad]++;
    if ((x == tx) != (y == ty)) c_one_dim++;

    mode = MODE_YX;
    #1;
    `TB_CHECK(p_dor === ref_dor(x, y, tx, ty, 1'b1),
      ("%0dx%0d DOR/YX at (%0d,%0d) to (%0d,%0d): got %s", N, N, x, y, tx, ty, tb_pkg::port_name(p_dor)))
    `TB_CHECK(step_is_minimal(x, y, tx, ty, p_dor),
      ("%0dx%0d YX at (%0d,%0d) to (%0d,%0d): %s is not minimal", N, N, x, y, tx, ty, tb_pkg::port_name(p_dor)))
    `TB_CHECK(p_xy === ref_dor(x, y, tx, ty, 1'b0) && p_ad === ref_adaptive(x, y, tx, ty, free),
      ("%0dx%0d at (%0d,%0d) to (%0d,%0d): RT_XY or RT_ADAPTIVE changed with mode_i", N, N, x, y, tx, ty))
    `TB_CHECK(dor_port(cx, cy, coord_t'(tx), coord_t'(ty), MODE_YX) === ref_dor(x, y, tx, ty, 1'b1),
      ("noc_pkg::dor_port YX wrong at (%0d,%0d) to (%0d,%0d)", x, y, tx, ty))
    c_port[1][p_dor]++;

    // Both directions productive: the free-VC rule decides
    if (x != tx && y != ty) begin
      px = (tx > x) ? P_E : P_W;
      py = (ty > y) ? P_S : P_N;
      for (int fx = 0; fx <= MAX_VC; fx++) begin
        for (int fy = 0; fy <= MAX_VC; fy++) begin
          if (ADAPT_ALL || (fx == int'(free[px]) && fy == int'(free[py]))) begin
            randomize_free();
            free[px] = VC_CNT_W'(fx);
            free[py] = VC_CNT_W'(fy);
            #1;
            exp = (fy > fx) ? py : px;
            `TB_CHECK(p_ad === exp,
              ("%0dx%0d adaptive at (%0d,%0d) to (%0d,%0d), free %s=%0d %s=%0d: got %s, expected %s", N, N,
               x, y, tx, ty, tb_pkg::port_name(px), fx, tb_pkg::port_name(py), fy,
               tb_pkg::port_name(p_ad), tb_pkg::port_name(exp)))
            if (fy > fx) c_y_more++;
            else if (fy == fx) c_tie++;
            else c_x_more++;
          end
        end
      end
    end
  endtask

  // ---------------------------------------------------------------------------
  // Walk a packet hop by hop. algo: 0 XY (RT_XY), 1 YX (RT_DOR), 2 adaptive
  // ---------------------------------------------------------------------------
  task automatic walk(int s, int t, int algo);
    int    x, y, tx, ty, manh, n_hops;
    bit    moved_x, moved_y, arrived;
    port_t p;
    x       = s % N;
    y       = s / N;
    tx      = t % N;
    ty      = t / N;
    manh    = iabs(tx - x) + iabs(ty - y);
    n_hops  = 0;
    moved_x = 1'b0;
    moved_y = 1'b0;
    arrived = 1'b0;
    mode    = (algo == 1) ? MODE_YX : MODE_XY;
    dst     = node_id_t'(t);
    for (int h = 0; h <= manh && !arrived; h++) begin
      cx = coord_t'(x);
      cy = coord_t'(y);
      if (algo == 2) randomize_free();
      #1;
      p = (algo == 0) ? p_xy : ((algo == 1) ? p_dor : p_ad);
      if (p == P_L) arrived = 1'b1;
      else begin
        if (algo == 0 && moved_y && (p == P_E || p == P_W))
          tb_pkg::fail($sformatf("%0dx%0d XY walk %0d->%0d turns from Y back to X", N, N, s, t));
        if (algo == 1 && moved_x && (p == P_N || p == P_S))
          tb_pkg::fail($sformatf("%0dx%0d YX walk %0d->%0d turns from X back to Y", N, N, s, t));
        case (p)
          P_N:     begin y = y - 1; moved_y = 1'b1; end
          P_S:     begin y = y + 1; moved_y = 1'b1; end
          P_E:     begin x = x + 1; moved_x = 1'b1; end
          P_W:     begin x = x - 1; moved_x = 1'b1; end
          default: tb_pkg::fail($sformatf("%0dx%0d walk %0d->%0d: port %0d", N, N, s, t, p));
        endcase
        n_hops++;
      end
    end
    `TB_CHECK(arrived && x == tx && y == ty && n_hops == manh,
      ("%0dx%0d walk %0d->%0d (algo %0d): ended at (%0d,%0d) after %0d hops, arrived=%0b, distance %0d",
       N, N, s, t, algo, x, y, n_hops, arrived, manh))
  endtask

  // ---------------------------------------------------------------------------
  // noc_pkg coordinate and neighbour helpers on this mesh
  // ---------------------------------------------------------------------------
  task automatic check_pkg_helpers();
    int x, y, nx, ny;
    bit exp_has;
    for (int id = 0; id < N * N; id++) begin
      x = id % N;
      y = id / N;
      `TB_CHECK(id_x(node_id_t'(id), N) == coord_t'(x) && id_y(node_id_t'(id), N) == coord_t'(y) &&
                xy_to_id(coord_t'(x), coord_t'(y), N) == node_id_t'(id),
        ("%0dx%0d: id %0d <-> (x, y) wrong", N, N, id))
      for (int p = 0; p < NPORTS; p++) begin
        nx = x;
        ny = y;
        case (p)
          0:       ny = y - 1;
          1:       nx = x + 1;
          2:       ny = y + 1;
          3:       nx = x - 1;
          default: ;
        endcase
        exp_has = (nx >= 0 && nx < N && ny >= 0 && ny < N);
        `TB_CHECK(has_nbr(coord_t'(x), coord_t'(y), port_t'(p), N) === exp_has,
          ("%0dx%0d: has_nbr(%0d,%0d,%s) wrong", N, N, x, y, tb_pkg::port_name(port_t'(p))))
        if (exp_has && p != int'(P_L)) begin
          `TB_CHECK(nbr_id(node_id_t'(id), port_t'(p), N) == node_id_t'(ny * N + nx),
            ("%0dx%0d: nbr_id(%0d,%s) wrong", N, N, id, tb_pkg::port_name(port_t'(p))))
          `TB_CHECK(nbr_id(nbr_id(node_id_t'(id), port_t'(p), N), opposite(port_t'(p)), N) == node_id_t'(id),
            ("%0dx%0d: going %s and back from %0d does not return", N, N, tb_pkg::port_name(port_t'(p)), id))
        end
      end
    end
  endtask

  initial begin
    done = 1'b0;
    for (int a = 0; a < 3; a++) for (int p = 0; p < NPORTS; p++) c_port[a][p] = 0;
    wait (go === 1'b1);
    sid = 1000 + N;
    tb_pkg::rng_init(sid);
    check_pkg_helpers();
    for (int y = 0; y < N; y++)
      for (int x = 0; x < N; x++)
        for (int t = 0; t < N * N; t++)
          probe_pair(x, y, t);
    if (WALKS) begin
      for (int s = 0; s < N * N; s++)
        for (int t = 0; t < N * N; t++)
          for (int a = 0; a < 3; a++)
            walk(s, t, a);
    end
    for (int p = 0; p < NPORTS; p++) begin
      tb_pkg::cover_bin($sformatf("%0dx%0d XY chooses %s", N, N, tb_pkg::port_name(port_t'(p))), c_port[0][p]);
      tb_pkg::cover_bin($sformatf("%0dx%0d YX chooses %s", N, N, tb_pkg::port_name(port_t'(p))), c_port[1][p]);
      tb_pkg::cover_bin($sformatf("%0dx%0d adaptive chooses %s", N, N, tb_pkg::port_name(port_t'(p))), c_port[2][p]);
    end
    if (N > 1) begin
      tb_pkg::cover_bin($sformatf("%0dx%0d adaptive: Y has more free VCs", N, N), c_y_more);
      tb_pkg::cover_bin($sformatf("%0dx%0d adaptive: tie goes to X", N, N), c_tie);
      tb_pkg::cover_bin($sformatf("%0dx%0d adaptive: X has more free VCs", N, N), c_x_more);
      tb_pkg::cover_bin($sformatf("%0dx%0d only one productive direction", N, N), c_one_dim);
    end
    done = 1'b1;
  end

endmodule


module tb_route_compute;
  import noc_pkg::*;

  logic       go = 1'b0;
  logic [3:0] done;

  rc_tester #(.N(2))                                     t_2  (.go(go), .done(done[0]));
  rc_tester #(.N(4))                                     t_4  (.go(go), .done(done[1]));
  rc_tester #(.N(8))                                     t_8  (.go(go), .done(done[2]));
  rc_tester #(.N(16), .ADAPT_ALL(1'b0), .WALKS(1'b0))    t_16 (.go(go), .done(done[3]));

  `TB_DUMP(tb_route_compute)

  initial tb_pkg::watchdog(100ms);

  // Constants and layouts the rest of the design relies on
  task automatic check_package();
    head_t  h;
    flit_t  f;
    logic [FLIT_DATA_W-1:0] d;
    `TB_CHECK($bits(head_t) == FLIT_DATA_W, ("head_t is %0d bits, not %0d", $bits(head_t), FLIT_DATA_W))
    `TB_CHECK(FLIT_W == 134 && LA_W == 16 && CX_W == 10 && CREDIT_W == 4,
      ("link widths: flit %0d, lookahead %0d, cancel %0d, credit %0d", FLIT_W, LA_W, CX_W, CREDIT_W))
    `TB_CHECK(is_head(FT_HEAD) && !is_tail(FT_HEAD) && !is_head(FT_BODY) && !is_tail(FT_BODY) &&
              !is_head(FT_TAIL) && is_tail(FT_TAIL) && is_head(FT_SINGLE) && is_tail(FT_SINGLE),
      ("flit type helpers wrong"))
    for (int p = 0; p < NPORTS; p++) begin
      `TB_CHECK(opposite(opposite(port_t'(p))) == port_t'(p) &&
                (p == int'(P_L) || opposite(port_t'(p)) != port_t'(p)),
        ("opposite(%0d) wrong", p))
    end
    // Head field positions: the NI and the testbench rely on them
    h          = '0;
    h.dst      = 8'hA5;
    h.src      = 8'h3C;
    h.cls      = CLS_DATA;
    h.len      = 3'd5;
    h.seq      = 32'h1234_5678;
    h.t_create = 32'h9ABC_DEF0;
    h.check    = 32'h0F0F_0F0F;
    d          = h;
    `TB_CHECK(d[7:0] == 8'hA5 && d[15:8] == 8'h3C && d[16] == 1'b1 && d[23:21] == 3'd5 &&
              d[55:24] == 32'h1234_5678 && d[87:56] == 32'h9ABC_DEF0 && d[119:88] == 32'h0F0F_0F0F &&
              d[127:120] == 8'h00 && d[20:17] == 4'h0,
      ("head_t field positions differ from the documented layout: %h", d))
    f       = '0;
    f.vld   = 1'b1;
    f.ftype = FT_SINGLE;
    f.data  = d;
    `TB_CHECK(f[FLIT_W-1] == 1'b1 && f[FLIT_DATA_W-1:0] == d, ("flit_t layout differs: vld on top, data at the bottom"))
  endtask

  initial begin
    tb_pkg::start("U03", "route_compute: minimal ports; adaptive by free VCs, ties to X; XY and YX exact");
    check_package();
    go = 1'b1;
    wait (&done === 1'b1);
    tb_pkg::finish();
  end

endmodule
