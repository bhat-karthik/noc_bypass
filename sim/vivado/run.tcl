# =============================================================================
# File      sim/vivado/run.tcl                                     Tracker: F11
# Project   Demand-adaptive buffer reservation scheduler (MVE433P, RVCE)
# Purpose   Compile, elaborate and run tests in the Vivado simulator (xsim)
#           without a Vivado project. Works on Windows and Linux.
# -----------------------------------------------------------------------------
# Pick ONE of the two ways; do not type the "vivado ..." line inside Vivado.
# 1. Vivado GUI or Vivado Tcl Shell, at its Tcl prompt:
#      cd D:/noc_bypass
#      set argv {-test all}
#      source -notrace sim/vivado/run.tcl
# 2. Windows command prompt, after running Vivado's settings64.bat:
#      cd /d D:\noc_bypass
#      vivado -mode batch -notrace -nojournal -nolog -source sim/vivado/run.tcl -tclargs -test all
# Options
#   -test <id>      a test ID (U01, U02, ..., N01, PERF), a testbench module
#                   name, or all: every testbench in sim/files_tb.f. Default
#                   U01. IDs come from the "// @tests" line of each testbench.
#   -seed <n>       random seed (default 1)
#   -plus "<args>"  more plusargs, e.g. -plus "+MAXERR=100 +CSV=u01.csv"
#   -define <N=V>   a compile define, e.g. -define TB_MESH_N=8 (8x8 mesh);
#                   repeat it for several. Each set of defines builds in its
#                   own work directory, e.g. work/tb_mesh__TB_MESH_N_8.
#   -wave           also record every signal to work/<top>/<top>.wdb; open it
#                   in Vivado with File > Simulation Waveform > Open
#   -gui            open the xsim GUI on the test instead of running it
# Output
#   sim/vivado/work/<top>/compile.log, elaborate.log, simulate.log
#   One summary line per test: PASS or FAIL, checks, run time, and the
#   number of compile and elaboration warnings (lint clean means 0 and 0).
#   Mesh tests also report the simulation speed: the cycles the testbench
#   simulated (its "INFO   cycles simulated" line) over the time xsim took.
# Windows note
#   xvlog, xelab and xsim are .bat files on Windows, and a batch file splits
#   its arguments at '=', ',' and ';'. "--testplusarg SEED=1" would reach xsim
#   as "SEED" and "1". Arguments like that go through an options file
#   (--file), which the tool reads itself (issue of 4 Nov).
# =============================================================================

set nb_here [file dirname [file normalize [info script]]]
set nb_root [file normalize [file join $nb_here .. ..]]

# ---------------------------------------------------------------- options
set nb_opt(test) U01
set nb_opt(seed) 1
set nb_opt(plus) ""
set nb_opt(wave) 0
set nb_opt(gui)  0
set nb_opt(defs) {}
if {![info exists argv]} { set argv {} }
for {set i 0} {$i < [llength $argv]} {incr i} {
  set a [lindex $argv $i]
  switch -- $a {
    -test   { incr i; set nb_opt(test) [lindex $argv $i] }
    -seed   { incr i; set nb_opt(seed) [lindex $argv $i] }
    -plus   { incr i; set nb_opt(plus) [lindex $argv $i] }
    -define { incr i; lappend nb_opt(defs) [lindex $argv $i] }
    -wave   { set nb_opt(wave) 1 }
    -gui    { set nb_opt(gui) 1 }
    default { error "run.tcl: unknown option '$a' (options: -test -seed -plus -define -wave -gui)" }
  }
}

# ---------------------------------------------------------------- helpers
proc nb_slurp {path} {
  set fh [open $path r]
  set txt [read $fh]
  close $fh
  return $txt
}

# A file list: returns {sources include-dirs defines}; paths from the repo root
proc nb_read_flist {path root} {
  set srcs {}
  set incs {}
  set defs {}
  foreach line [split [nb_slurp $path] "\n"] {
    regsub {//.*$} $line {} line
    set line [string trim $line]
    if {$line eq ""} continue
    if {[string match {+incdir+*} $line]} {
      lappend incs [file join $root [string range $line 8 end]]
    } elseif {[string match {+define+*} $line]} {
      lappend defs [string range $line 8 end]
    } else {
      lappend srcs [file join $root $line]
    }
  }
  return [list $srcs $incs $defs]
}

proc nb_tests_of {path} {
  if {[regexp {//\s*@tests\s+([^\n]*)} [nb_slurp $path] -> ids]} { return [string trim $ids] }
  return {}
}

proc nb_tool {name} {
  set exe [auto_execok $name]
  if {$exe eq ""} {
    error "run.tcl: '$name' not found. Run from Vivado or the Vivado Tcl Shell, so that Vivado's bin directory is on the PATH."
  }
  return $exe
}

# Run a command with stdout and stderr in a log file; 1 when it exits with 0
proc nb_exec {log cmd} {
  if {[catch {exec {*}$cmd >& $log}]} { return 0 }
  return 1
}

# Run a Vivado tool: exe, then the fixed arguments, then the options. When an
# option holds a character a Windows batch file splits on (= , ;), the options
# go on one line of <log>.args and the tool reads them itself with --file, so
# they arrive intact on every operating system.
proc nb_run_tool {log exe fixed opts} {
  set risky 0
  foreach a $opts {
    if {[regexp {[=,;]} $a]} { set risky 1 }
  }
  if {!$risky} { return [nb_exec $log [concat $exe $fixed $opts]] }
  set optfile "[file rootname $log].args"
  set fh [open $optfile w]
  fconfigure $fh -translation lf
  puts $fh [join $opts " "]
  close $fh
  return [nb_exec $log [concat $exe $fixed [list --file $optfile]]]
}

# The lines of a log that match a pattern, at most n of them
proc nb_grep {log pattern n} {
  if {![file exists $log]} { return {} }
  set out {}
  foreach line [split [nb_slurp $log] "\n"] {
    if {[regexp $pattern $line] && [llength $out] < $n} { lappend out $line }
  }
  return $out
}

proc nb_head {log n} {
  if {![file exists $log]} { return "" }
  set lines [split [nb_slurp $log] "\n"]
  return [join [lrange $lines 0 [expr {$n - 1}]] "\n"]
}

proc nb_count {log pattern} {
  if {![file exists $log]} { return 0 }
  set n 0
  foreach line [split [nb_slurp $log] "\n"] {
    if {[regexp $pattern $line]} { incr n }
  }
  return $n
}

proc nb_tail {log n} {
  if {![file exists $log]} { return "" }
  set lines [split [string trimright [nb_slurp $log] "\n"] "\n"]
  return [join [lrange $lines end-[expr {$n - 1}] end] "\n"]
}

# ---------------------------------------------------------------- tests to run
lassign [nb_read_flist [file join $nb_root sim files_rtl.f] $nb_root] rtl_srcs rtl_incs rtl_defs
lassign [nb_read_flist [file join $nb_root sim files_tb.f]  $nb_root] tb_srcs  tb_incs  tb_defs

set nb_tops {}
foreach f $tb_srcs {
  set ids [nb_tests_of $f]
  if {[llength $ids] == 0} continue
  set mod [file rootname [file tail $f]]
  if {$nb_opt(test) eq "all" || $nb_opt(test) eq $mod || [lsearch -exact $ids $nb_opt(test)] >= 0} {
    lappend nb_tops [list $mod $ids]
  }
}
if {[llength $nb_tops] == 0} {
  error "run.tcl: no testbench in sim/files_tb.f runs '$nb_opt(test)'"
}

set xvlog [nb_tool xvlog]
set xelab [nb_tool xelab]
set xsim  [nb_tool xsim]

# xvlog: options and files on the command line, as in B0/B1. Defines go in
# a separate list: a value such as TB_MESH_N=8 holds '=', which the Windows
# batch wrapper would split, so nb_run_tool passes them in an options file
# (--file), the same way xsim's plusargs travel.
set vargs [list --sv]
foreach d [concat $rtl_incs $tb_incs] { lappend vargs --include $d }
set vargs [concat $vargs $rtl_srcs $tb_srcs]
set vdefs {}
foreach d [concat $rtl_defs $tb_defs $nb_opt(defs)] { lappend vdefs --define $d }

# work directory suffix for the -define set: TB_MESH_N=8 -> __TB_MESH_N_8
set nb_wsuf ""
foreach d $nb_opt(defs) { append nb_wsuf "__[regsub -all {[^A-Za-z0-9]+} $d {_}]" }

set plus [list "SEED=$nb_opt(seed)"]
foreach p $nb_opt(plus) { lappend plus [string trimleft $p "+"] }
# a test ID (capitals and digits, e.g. N03 or PERF), not "all" or a module name
if {[regexp {^[A-Z][A-Z0-9]*$} $nb_opt(test)]} { lappend plus "TEST=$nb_opt(test)" }

set debug off
if {$nb_opt(wave) || $nb_opt(gui)} { set debug typical }

# ---------------------------------------------------------------- run
set nb_old [pwd]
set nb_summary {}
foreach t $nb_tops {
  lassign $t top ids
  set work [file join $nb_here work "$top$nb_wsuf"]
  file mkdir $work
  cd $work
  set t0 [clock seconds]
  puts "run.tcl: $top (tests $ids), seed $nb_opt(seed): compiling"
  set ok [nb_run_tool compile.log $xvlog $vargs $vdefs]
  if {$ok} {
    puts "run.tcl: $top: elaborating"
    set ok [nb_run_tool elaborate.log $xelab {} [list --debug $debug --timescale 1ns/1ps --snapshot ${top}_snap work.$top]]
  }
  set wc [nb_count compile.log {^WARNING}]
  set we [nb_count elaborate.log {^WARNING}]
  foreach w [concat [nb_grep compile.log {^WARNING} 10] [nb_grep elaborate.log {^WARNING} 10]] {
    puts "run.tcl: $top: $w"
  }
  if {!$ok} {
    set result "BUILD FAILED: see [file join $work compile.log] and elaborate.log"
    puts [nb_tail compile.log 15]
    puts [nb_tail elaborate.log 15]
  } else {
    set sargs {}
    foreach p $plus { lappend sargs --testplusarg $p }
    if {$nb_opt(gui)} {
      set fh [open gui.args w]
      fconfigure $fh -translation lf
      puts $fh [join $sargs " "]
      close $fh
      exec {*}$xsim ${top}_snap --file gui.args --gui &
      set result "xsim GUI opened"
    } else {
      if {$nb_opt(wave)} {
        set fh [open wave.tcl w]
        puts $fh "log_wave -recursive *"
        puts $fh "run all"
        puts $fh "quit"
        close $fh
        lappend sargs --tclbatch wave.tcl --wdb ${top}.wdb
      } else {
        lappend sargs --runall
      }
      puts "run.tcl: $top: simulating"
      set ts0 [clock milliseconds]
      nb_run_tool simulate.log $xsim [list ${top}_snap] $sargs
      set ts [expr {([clock milliseconds] - $ts0) / 1000.0}]
      set txt ""
      if {[file exists simulate.log]} { set txt [nb_slurp simulate.log] }
      set speed ""
      if {[regexp {INFO\s+cycles simulated\s+(\d+)} $txt -> ncyc] && $ts > 0} {
        set speed [format ", %d cycles in %.1f s = %.0f cycles/s" $ncyc $ts [expr {$ncyc / $ts}]]
      }
      if {[regexp {RESULT\s+(\S+)\s+(PASS|FAIL)\s+checks=(\d+)\s+errors=(\d+)} $txt -> tid verdict nchk nerr]} {
        puts [nb_tail simulate.log 6]
        set result "$verdict  checks=$nchk errors=$nerr$speed"
      } else {
        # The cause is near the top of the log (xsim prints its error first)
        puts [nb_head simulate.log 8]
        set result "NO RESULT LINE (crash or timeout): see [file join $work simulate.log]"
      }
    }
  }
  set dt [expr {[clock seconds] - $t0}]
  lappend nb_summary [format "%-20s %-12s %s   (%d s; warnings: compile %d, elaborate %d)" $top $ids $result $dt $wc $we]
  cd $nb_old
}

set nb_dtxt ""
if {[llength $nb_opt(defs)]} { set nb_dtxt ", defines [join $nb_opt(defs) { }]" }
puts "==================== run.tcl summary, seed $nb_opt(seed)$nb_dtxt ===================="
foreach s $nb_summary { puts $s }
puts "Logs: [file join $nb_here work]/<testbench>/"
