// =============================================================================
// File      sim/files_rtl.f                                   Tracker: F09
// RTL compile order. Paths are relative to the repository root: run every
// simulator from there (sim/vivado/run.tcl does this for you).
// Packages first; one file per line; add new RTL files at the end of their
// batch section.
// =============================================================================
// ---- B0 foundation (packages first: sched_pkg is B3's, used by later tiles)
rtl/pkg/noc_pkg.sv
rtl/pkg/sched_pkg.sv
rtl/common/rr_arbiter.sv
rtl/common/sync_fifo.sv
rtl/router/route_compute.sv
// ---- B1 router core
rtl/router/input_unit.sv
rtl/router/output_unit.sv
rtl/router/switch_allocator.sv
rtl/router/crossbar.sv
rtl/router/router.sv
// ---- B2 network: NI, tile, mesh
rtl/ni/ni.sv
rtl/top/tile.sv
rtl/top/mesh_top.sv
// ---- B3 timetable: slot table, slot counter, partner logic
rtl/sched/slot_table.sv
rtl/sched/slot_counter.sv
rtl/sched/group_to_router.sv
// ---- B4 bypass datapath: source and relay (router.sv instantiates them)
rtl/router/bypass_source.sv
rtl/router/bypass_relay.sv
// ---- B5 reservation and cancel tokens (ni.sv and router.sv instantiate them)
rtl/ni/resv_ctrl.sv
rtl/router/cancel_relay.sv
// ---- B6 two-level timetable (tile.sv instantiates it in design C)
rtl/sched/phase_sequencer.sv
// ---- B7 design D controller: demand counter and controller node (tile.sv
//      instantiates them in design D), the controller (mesh_top.sv)
rtl/ctrl/demand_counter.sv
rtl/ctrl/ctrl_node.sv
rtl/ctrl/sched_ctrl.sv
