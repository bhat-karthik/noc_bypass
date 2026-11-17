// =============================================================================
// File      rtl/pkg/noc_pkg.sv                                     Tracker: F01
// Project   Demand-adaptive buffer reservation scheduler for router bypassing
//           in mesh NoCs (MVE433P, RV College of Engineering)
// Purpose   Sizes, port names, the flit, lookahead, cancel and credit types,
//           and the coordinate and dimension-order helpers every block shares.
// Decisions D03 router, D04 packets, D05 VCs and credits, D06 routing,
//           D07 lookahead, D09 bypass direction, D12 cancel tokens
// Tests     used by every test; the helpers are checked in U03
// -----------------------------------------------------------------------------
// Conventions for the whole code base
//   Mesh    N x N routers. x is the column: 0 at the west edge, growing east.
//           y is the row: 0 at the north edge, growing south. id = y*N + x, so
//           R0 is the north-west corner and ids run row by row, as in the
//           figures and Table I of BuffeRS [1].
//   Ports   0 North (y-1), 1 East (x+1), 2 South (y+1), 3 West (x-1),
//           4 Local (the network interface).
//   Widths  Fixed maxima: 16x16 mesh, 4 VCs, 5-flit packets, 15-flit bursts.
//           Smaller configurations keep the unused top bits at zero and
//           synthesis removes them.
// =============================================================================
`timescale 1ns/1ps

// A package of constants is a library: most blocks use only some of them.
/* verilator lint_off UNUSEDPARAM */
package noc_pkg;

  // ---------------------------------------------------------------------------
  // Sizes
  // ---------------------------------------------------------------------------
  localparam int MAX_MESH_N    = 16;   // largest mesh side we build
  localparam int NODE_ID_W     = 8;    // router id, 0..255
  localparam int COORD_W       = 4;    // x or y, 0..15
  localparam int MAX_VC        = 4;    // VCs per input port (Table III: 1 or 4)
  localparam int VC_ID_W       = 2;    // VC number, 0..3
  localparam int VC_CNT_W      = 3;    // a number of VCs, 0..4
  localparam int FLIT_DATA_W   = 128;  // link and flit width (Table III)
  localparam int MAX_PKT_FLITS = 5;    // data packets 5 flits, control 1 (D04)
  localparam int PKT_LEN_W     = 3;    // packet length, 1..5
  localparam int MAX_BURST     = 15;   // largest bypass burst in flits (G2)
  localparam int BURST_W       = 4;    // burst length, 0..15
  localparam int NPORTS        = 5;    // N, E, S, W, Local
  localparam int PORT_W        = 3;    // port number, 0..4

  typedef logic [NODE_ID_W-1:0] node_id_t;
  typedef logic [COORD_W-1:0]   coord_t;
  typedef logic [VC_ID_W-1:0]   vc_id_t;
  typedef logic [PORT_W-1:0]    port_t;

  // ---------------------------------------------------------------------------
  // Port numbers
  // ---------------------------------------------------------------------------
  localparam port_t P_N = 3'd0;
  localparam port_t P_E = 3'd1;
  localparam port_t P_S = 3'd2;
  localparam port_t P_W = 3'd3;
  localparam port_t P_L = 3'd4;

  // ---------------------------------------------------------------------------
  // The four designs compared in the project (parameter DESIGN of the mesh)
  // ---------------------------------------------------------------------------
  localparam int DESIGN_A = 0;   // regular router, no bypass (baseline)
  localparam int DESIGN_B = 1;   // BuffeRS [1] as published: one global timetable
  localparam int DESIGN_C = 2;   // two-level timetable with a fixed k_L (K_FIXED)
  localparam int DESIGN_D = 3;   // two-level timetable, k_L set by demand every epoch

  // ---------------------------------------------------------------------------
  // Routing algorithms (parameter ALGO of route_compute)
  // ---------------------------------------------------------------------------
  localparam int RT_XY       = 0;  // dimension order, X first (design A)
  localparam int RT_ADAPTIVE = 1;  // minimal adaptive by free downstream VCs (D06)
  localparam int RT_DOR      = 2;  // XY or YX per packet, chosen by a mode bit (bypass, D09)

  localparam logic MODE_XY = 1'b0;
  localparam logic MODE_YX = 1'b1;

  // ---------------------------------------------------------------------------
  // Flit types. Bit 0 marks a head and bit 1 a tail, so a 1-flit packet
  // (SINGLE) is both a head and a tail.
  // ---------------------------------------------------------------------------
  typedef logic [1:0] ftype_t;
  localparam ftype_t FT_BODY   = 2'b00;
  localparam ftype_t FT_HEAD   = 2'b01;
  localparam ftype_t FT_TAIL   = 2'b10;
  localparam ftype_t FT_SINGLE = 2'b11;

  // Message class carried in the head (D04)
  localparam logic CLS_CTRL = 1'b0;    // 1-flit control packet
  localparam logic CLS_DATA = 1'b1;    // 5-flit data packet

  // ---------------------------------------------------------------------------
  // Head flit payload, 128 bits. Routers read only dst; the NI also reads len.
  // The other fields sit in payload bits a real packet carries anyway; the
  // testbench uses them to check every packet and to measure latency (G7).
  // ---------------------------------------------------------------------------
  typedef struct packed {
    logic [7:0]           rsvd_hi;   // [127:120] zero
    logic [31:0]          check;     // [119:88]  packet signature set by the source
    logic [31:0]          t_create;  // [87:56]   cycle the packet was created
    logic [31:0]          seq;       // [55:24]   sequence number per source
    logic [PKT_LEN_W-1:0] len;       // [23:21]   flits in the packet, 1..5
    logic [3:0]           rsvd_lo;   // [20:17]   zero
    logic                 cls;       // [16]      CLS_CTRL or CLS_DATA
    node_id_t             src;       // [15:8]    source router
    node_id_t             dst;       // [7:0]     destination router
  } head_t;

  // ---------------------------------------------------------------------------
  // Link types. A link carries at most one of each per cycle.
  // ---------------------------------------------------------------------------
  // A flit on a link: 134 bits
  typedef struct packed {
    logic                   vld;     // a flit is present this cycle
    ftype_t                 ftype;   // FT_HEAD, FT_BODY, FT_TAIL or FT_SINGLE
    vc_id_t                 vc;      // VC it occupies at the receiving input port
    logic                   byp;     // the flit travels on a bypass path
    logic [FLIT_DATA_W-1:0] data;    // head_t for a head, payload otherwise
  } flit_t;

  // Lookahead: travels one hop ahead of a bypass burst so the next router sets
  // its crossbar and blocks switch allocation for that output (D07)
  typedef struct packed {
    logic               vld;
    node_id_t           dst;         // destination router of the burst
    logic               mode;        // MODE_XY or MODE_YX
    vc_id_t             vc;          // entry reserved at the destination
    logic [BURST_W-1:0] len;         // flits in the burst
  } la_t;

  // Cancel token: tells one source not to send in its subslot (D12)
  typedef struct packed {
    logic     vld;
    node_id_t tgt;                   // source router the token is for
    logic     mode;                  // direction of the path the token follows
  } cx_t;

  // Credit sent upstream when a flit leaves an input VC (D05)
  typedef struct packed {
    logic   vld;
    vc_id_t vc;
    logic   vc_free;                 // the tail left, so the VC is free again
  } credit_t;

  localparam int FLIT_W   = $bits(flit_t);     // 134
  localparam int LA_W     = $bits(la_t);       // 16
  localparam int CX_W     = $bits(cx_t);       // 10
  localparam int CREDIT_W = $bits(credit_t);   // 4

  // ---------------------------------------------------------------------------
  // Flit helpers
  // ---------------------------------------------------------------------------
  function automatic logic is_head(ftype_t t);
    return (t == FT_HEAD) || (t == FT_SINGLE);
  endfunction

  function automatic logic is_tail(ftype_t t);
    return (t == FT_TAIL) || (t == FT_SINGLE);
  endfunction

  // ---------------------------------------------------------------------------
  // Coordinates. n is the mesh side. With n = 4, 8 or 16 the remainder and the
  // division below are bit slices after synthesis.
  // ---------------------------------------------------------------------------
  function automatic coord_t id_x(node_id_t id, int n);
    return coord_t'(int'(id) % n);
  endfunction

  function automatic coord_t id_y(node_id_t id, int n);
    return coord_t'(int'(id) / n);
  endfunction

  function automatic node_id_t xy_to_id(coord_t x, coord_t y, int n);
    return node_id_t'(int'(y) * n + int'(x));
  endfunction

  // Hops on a minimal path between (ax, ay) and (bx, by)
  function automatic int hops(coord_t ax, coord_t ay, coord_t bx, coord_t by);
    int dx, dy;
    dx = int'(ax) - int'(bx);
    dy = int'(ay) - int'(by);
    return ((dx < 0) ? -dx : dx) + ((dy < 0) ? -dy : dy);
  endfunction

  // 1 when router (x, y) of an n x n mesh has a neighbour through port p.
  // Local always exists.
  function automatic logic has_nbr(coord_t x, coord_t y, port_t p, int n);
    logic r;
    case (p)
      P_N:     r = (y != '0);
      P_E:     r = (int'(x) < n - 1);
      P_S:     r = (int'(y) < n - 1);
      P_W:     r = (x != '0);
      default: r = 1'b1;
    endcase
    return r;
  endfunction

  // Id of the neighbour through port p. Valid only when has_nbr is 1;
  // for Local it returns the router itself.
  function automatic node_id_t nbr_id(node_id_t id, port_t p, int n);
    int x, y;
    x = int'(id) % n;
    y = int'(id) / n;
    case (p)
      P_N:     y = y - 1;
      P_E:     x = x + 1;
      P_S:     y = y + 1;
      P_W:     x = x - 1;
      default: ;
    endcase
    return node_id_t'(y * n + x);
  endfunction

  // Port at the other end of a link: a flit that leaves through E arrives on W
  function automatic port_t opposite(port_t p);
    port_t r;
    case (p)
      P_N:     r = P_S;
      P_E:     r = P_W;
      P_S:     r = P_N;
      P_W:     r = P_E;
      default: r = P_L;
    endcase
    return r;
  endfunction

  // Next output on a dimension-order path from (cx, cy) to (tx, ty).
  // MODE_XY moves along x first, MODE_YX along y first; P_L at the target.
  // Bypass paths use it with the mode of the slot (D09); design A uses XY.
  function automatic port_t dor_port(coord_t cx, coord_t cy, coord_t tx, coord_t ty, logic mode);
    port_t r;
    if (cx == tx && cy == ty) r = P_L;
    else if (mode == MODE_XY) r = (cx != tx) ? ((tx > cx) ? P_E : P_W) : ((ty > cy) ? P_S : P_N);
    else                      r = (cy != ty) ? ((ty > cy) ? P_S : P_N) : ((tx > cx) ? P_E : P_W);
    return r;
  endfunction

endpackage
/* verilator lint_on UNUSEDPARAM */
