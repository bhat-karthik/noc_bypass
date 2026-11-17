# =============================================================================
# File      sim/vivado/regress.tcl                                 Tracker: F61
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Runs a regression list in the Vivado simulator (xsim) and checks
#           every test's RESULT line against the count the list expects at
#           seed 1. Each testbench is built once per set of defines and the
#           build is reused while no source changes, so a list of twenty
#           tests compiles only as often as it has different builds.
# -----------------------------------------------------------------------------
# Vivado Tcl console (from the repository):
#     cd D:/noc_bypass
#     set argv {-list b7}; source -notrace sim/vivado/regress.tcl
# Windows command prompt, after Vivado's settings64.bat:
#     vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/regress.tcl -tclargs -list b7
# Options
#   -list <name>    sim/lists/<name>.list; repeat for several lists
#   -only <label>   run only the entries whose label matches (glob, e.g. CT*)
#   -seed <n>       random seed (default 1). The expected counts hold for
#                   seed 1; with another seed the counts are reported only
#   -rebuild        build every testbench again even if up to date
#   -stop           stop at the first entry that does not pass
# List format, one entry per line ('#' starts a comment):
#   <label> <test ID or testbench> <expected checks or -> [-define N=V]... [-plus +ARG]...
# e.g.  CT05_8x8  CT05  35060190  -define TB_MESH_N=8
#       TL06_q    TL06  296594    -plus +QUICK
# Verdicts: PASS (PASS line and the expected count), COUNT (PASS line, other
# count: the simulators disagree, report it), FAIL (FAIL line), NONE (no
# RESULT line: crash, timeout or a $fatal), BUILD (compile or elaboration
# failed).
# Output: one line per entry, a summary, and the same lines in
# sim/vivado/work/regress_<lists>_seed<n>.txt; logs in work/<testbench>/.
# =============================================================================

# Vivado's source takes -notrace (no echo of every command); plain Tcl's does not
set rg_lib [file join [file dirname [file normalize [info script]]] xsim_lib.tcl]
if {[llength [info commands get_property]]} { source -notrace $rg_lib } else { source $rg_lib }

set rg_lists {}
set rg_only  *
set rg_seed  1
set rg_force 0
set rg_stop  0
if {![info exists argv]} { set argv {} }
for {set i 0} {$i < [llength $argv]} {incr i} {
  set a [lindex $argv $i]
  switch -- $a {
    -list    { incr i; lappend rg_lists [lindex $argv $i] }
    -only    { incr i; set rg_only [lindex $argv $i] }
    -seed    { incr i; set rg_seed [lindex $argv $i] }
    -rebuild { set rg_force 1 }
    -stop    { set rg_stop 1 }
    default  { error "regress.tcl: unknown option '$a' (options: -list -only -seed -rebuild -stop)" }
  }
}
if {[llength $rg_lists] == 0} { error "regress.tcl: name a list, e.g. set argv {-list b7} (lists in sim/lists)" }

# ---------------------------------------------------------------- read the lists
set rg_entries {}
foreach name $rg_lists {
  set path [file join $xl_root sim lists "$name.list"]
  if {![file exists $path]} { error "regress.tcl: no list $path" }
  set ln 0
  foreach line [split [xl_slurp $path] "\n"] {
    incr ln
    regsub {#.*$} $line {} line
    set line [string trim $line]
    if {$line eq ""} continue
    set w [regexp -all -inline {\S+} $line]
    if {[llength $w] < 3} { error "regress.tcl: $name.list line $ln: expected <label> <test> <count>" }
    lassign $w label test exp
    set defs {}
    set plus {}
    for {set k 3} {$k < [llength $w]} {incr k} {
      switch -- [lindex $w $k] {
        -define { incr k; lappend defs [lindex $w $k] }
        -plus   { incr k; lappend plus [string trimleft [lindex $w $k] "+"] }
        default { error "regress.tcl: $name.list line $ln: unknown option '[lindex $w $k]'" }
      }
    }
    if {![string match $rg_only $label]} continue
    set top [xl_top_of $test]
    if {$top eq ""} { error "regress.tcl: $name.list line $ln: no testbench runs '$test'" }
    lappend rg_entries [list $label $test $top $exp $defs $plus]
  }
}
puts "regress.tcl: [llength $rg_entries] entries from [join $rg_lists {, }], seed $rg_seed"

# ---------------------------------------------------------------- run
set rg_lines {}
set rg_n [dict create PASS 0 COUNT 0 FAIL 0 NONE 0 BUILD 0]
set rg_t0 [clock seconds]
set rg_built {}
foreach e $rg_entries {
  lassign $e label test top exp defs plus
  set key "$top|$defs"
  if {[dict exists $rg_built $key]} {
    lassign [dict get $rg_built $key] ok work w msg
  } else {
    puts "regress.tcl: $label: building $top [join $defs { }]"
    lassign [xl_build $top $defs $rg_force] ok work w msg
    dict set rg_built $key [list $ok $work $w $msg]
    puts "regress.tcl: $label: $top: $msg ($w warnings)"
  }
  if {!$ok} {
    set v BUILD
    set line [format "%-14s %-6s %-22s BUILD   %s" $label $test $top $msg]
  } else {
    set p [list "SEED=$rg_seed"]
    if {[regexp {^[A-Z][A-Z0-9]*$} $test]} { lappend p "TEST=$test" }
    set p [concat $p $plus]
    puts "regress.tcl: $label: simulating"
    set r [xl_sim $work $top $p "sim_$label.log"]
    set v [dict get $r verdict]
    set c [dict get $r checks]
    if {$v eq "PASS" && $rg_seed == 1 && $exp ne "-" && $c ne $exp} { set v COUNT }
    set expt [expr {$exp eq "-" || $rg_seed != 1 ? "" : " (expected [xl_commas $exp])"}]
    set cyc [dict get $r cycles]
    set ctxt [expr {$cyc eq "-" ? "" : ", [xl_commas $cyc] cycles"}]
    set line [format "%-14s %-6s %-22s %-6s  checks=%s errors=%s%s%s, %.0f s" $label $test $top $v \
                [xl_commas $c] [dict get $r errors] $expt $ctxt [dict get $r secs]]
    if {$v eq "NONE"} { append line "\n               [xl_head [dict get $r log] 3]" }
  }
  dict incr rg_n $v
  lappend rg_lines $line
  puts $line
  if {$rg_stop && $v ne "PASS"} { puts "regress.tcl: -stop: stopping at $label"; break }
}

set rg_sum [format "%d entries: %d PASS, %d COUNT, %d FAIL, %d NONE, %d BUILD in %d min" [llength $rg_lines] \
              [dict get $rg_n PASS] [dict get $rg_n COUNT] [dict get $rg_n FAIL] [dict get $rg_n NONE] [dict get $rg_n BUILD] \
              [expr {([clock seconds] - $rg_t0 + 30) / 60}]]
set rg_out [file join $xl_here work "regress_[join $rg_lists _]_seed$rg_seed.txt"]
xl_spit $rg_out "regress.tcl [join $rg_lists { }], seed $rg_seed, [clock format [clock seconds] -format {%Y-%m-%d %H:%M}]\n[join $rg_lines \n]\n$rg_sum\n"
puts "==================== regress.tcl summary: [join $rg_lists { }], seed $rg_seed ===================="
foreach l $rg_lines { puts $l }
puts $rg_sum
puts "Summary file: $rg_out"
