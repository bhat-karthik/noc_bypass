// =============================================================================
// File      sim/files_tb.f                                    Tracker: F09
// Testbench compile order, after sim/files_rtl.f. Paths are relative to the
// repository root. Each testbench file names the tests it runs in a line
// "// @tests <IDs>"; run.tcl and make_bundle.tcl find tests that way.
// =============================================================================
+incdir+tb/common
+incdir+tb/mesh
+incdir+tb/checkers
// ---- packages (every testbench may use them)
tb/common/tb_pkg.sv
tb/common/traffic_pkg.sv
tb/common/scoreboard.sv
tb/common/stats.sv
tb/common/sched_ref.sv
tb/common/perf_probe.sv
tb/checkers/ni_checker.sv
tb/checkers/router_checker.sv
tb/common/sched_monitor.sv
tb/checkers/mesh_checker.sv
tb/checkers/ctrl_checker.sv
// ---- B0 unit tests
tb/unit/tb_rr_arbiter.sv
tb/unit/tb_sync_fifo.sv
tb/unit/tb_route_compute.sv
// ---- B1 router tests
tb/unit/tb_switch_allocator.sv
tb/unit/tb_router.sv
// ---- B2 mesh tests: support modules, then the two tops
tb/common/tg_node.sv
tb/common/sink_node.sv
tb/common/run_ctrl.sv
tb/mesh/mesh_bench.sv
tb/mesh/tb_mesh.sv
tb/mesh/tb_mesh_dl.sv
// ---- B3 timetable unit tests
tb/unit/tb_slot_table.sv
tb/unit/tb_slot_counter.sv
tb/unit/tb_group_to_router.sv
tb/unit/tb_timetable_check.sv
// ---- B4 bypass tests: the design-B mesh top (router_checker.sv, with the
//      checker package, is in the package section above)
tb/mesh/tb_mesh_b.sv
// ---- B5 reservation: the unit test (ni_checker.sv and perf_probe.sv, with
//      their packages, are in the package section above)
tb/unit/tb_resv_ctrl.sv
// ---- B6 two-level timetable: the unit test and the design-C mesh tops
//      (sched_monitor.sv and mesh_checker.sv, with their packages, are in the
//      package section above; mesh_bench includes tests_two.svh)
tb/unit/tb_phase_sequencer.sv
tb/mesh/tb_mesh_c.sv
tb/mesh/tb_mesh_cb.sv
// ---- B7 design D controller: the unit tests and the design-D mesh top
//      (ctrl_checker.sv, with its package, is in the package section above;
//      mesh_bench includes tests_ctrl.svh)
tb/unit/tb_demand_counter.sv
tb/unit/tb_sched_ctrl.sv
tb/unit/tb_ctrl_chain.sv
tb/mesh/tb_mesh_d.sv
